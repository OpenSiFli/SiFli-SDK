/**
****************************************************************************************
*
* @file lld_adv.c
*
* @brief LLD Advertising source code
*
* Copyright (C) RivieraWaves 2009-2017
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDADV
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"         // stack configuration

#if (BLE_BROADCASTER)

#include <string.h>

#include "co_math.h"
#include "co_endian.h"
#include "ble_util.h"            // BLE utility functions

#include "ke_mem.h"
#include "ke_task.h"             // kernel task management
#include "rwip.h"

#include "lld.h"                 // link driver API
#include "lld_int.h"             // link layer driver internal

#include "llm.h"                 // link layer manager definitions

#include "em_map.h"
#include "dbg.h"

#include "sch_arb.h"             // Scheduling Arbiter
#include "sch_prog.h"            // Scheduling Programmer
#include "sch_slice.h"           // Scheduling Slicer

#include "reg_blecore.h"         // BLE core registers
#include "reg_em_ble_cs.h"       // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors
#include "reg_em_ble_ral.h"      // BLE EM Resolving list
#include "reg_em_et.h"           // EM Exchange Table
#if (EAVESDROPPING_SUPPORT)
    #include "reg_ipcore.h"          // IP core registers
    #include "lm.h"
#endif // EAVESDROPPING_SUPPORT

#include "btdm_patch.h"

#undef __STATIC
#define __STATIC
/*
 * DEFINES
 *****************************************************************************************
 */

/// MAX_RAND value used for advDelay (range of 0 ms to 10 ms so modulo 17 operation, value in slots)
#define LLD_ADV_MAX_RAND            17

/// MAX_RAND value used for advDelay when event is cancelled (range of 0 ms to 10 ms so modulo 17 operation, value in slots)
#define LLD_ADV_MAX_RAND_CANCEL     17

/// Maximum duration of high duty cycle advertising in slots (2048 slots, 1.28 s)
#define MAX_HDC_ADV_DUR             2048

/*
 * ENUMERATION DEFINITION
 *****************************************************************************************
 */

/// Advertising event states
enum ADV_EVT_STATE
{
    ADV_EVT_WAIT,
    ADV_EVT_ACTIVE,
    ADV_EVT_END,
};

/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

/// Table indicating the duration of the CONNECT_REQ packet (in us) depending on the PHY used
const uint16_t con_req_dur_tab[] =
{
    [CO_RATE_1MBPS  ]  = 352,   // (1+4+(2+34)+3)*8 = 44*8 = 352
    [CO_RATE_2MBPS  ]  = 180,   // (2+4+(2+34)+3)*4 = 45*4 = 180
    [CO_RATE_125KBPS]  = 2896,  // 80+256+16+24+(2+34)*8*8+24*8+3*8 = 2896
    [CO_RATE_500KBPS]  = 1006,  // 80+256+16+24+(2+34)*8*2+24*2+3*2 = 1006
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LLD ADV data structure
struct lld_adv_data_tag
{
    /// Advertising data buffer
    uint16_t adv_data_buf;

    /// Scan response data buffer
    uint16_t scan_rsp_data_buf;

    /// Advertising data length in bytes
    uint16_t adv_data_len;

    /// Scan response data length in bytes
    uint16_t scan_rsp_data_len;
};

/// LLD ADV environment structure
struct lld_adv_env_tag
{
    /// Advertising Scheduling Arbiter data
    struct sch_arb_elt_tag evt;

    /// Advertising data, used to update the advertising or scan response data in a safe manner
    struct lld_adv_data_tag data;

    /// BD Address of the peer device (only for directed advertising)
    struct bd_addr peer_addr;

    /// Advertising Scheduling Arbiter data for the aux event
    struct sch_arb_elt_tag aux_evt;

    /// Advertising timeout timestamp in half-slots (312.5 us) (indicates when the timeout occurs)
    uint32_t timeout_ts;

    /// Event timestamp of the last time the event priority was updated in half-slots (312.5 us)
    uint32_t last_prio_upd_ts;

    /// Aux event timestamp of the last time the aux event priority was updated in half-slots (312.5 us)
    uint32_t last_prio_upd_aux_ts;

    /// Advertising interval in slots (625 us) (only for low duty cycle advertising)
    uint32_t intv;

    /// Periodic advertising interval in units of 1.25 ms
    uint32_t per_adv_intv;

    /// Total advertising duration in us
    uint32_t total_adv_dur_us;

    /// Duration of the AUX_ADV_IND PDU in us
    uint32_t aux_adv_ind_dur_us;

    /**
    * Advertising event properties
    * Bit number Parameter Description
    * 0          Connectable advertising
    * 1          Scannable advertising
    * 2          Directed advertising
    * 3          High Duty Cycle Directed Connectable advertising (<= 3.75 ms Advertising Interval)
    * 4          Use legacy advertising PDUs
    * 5          Omit advertiser's address from all PDUs ("anonymous advertising")
    * 6          Include TxPower in the extended header of the advertising PDU
    * All other bits Reserved for future use
    */
    uint16_t properties;

    /// AdvDataInfo (ADI) field
    uint16_t adi;

    /// Maximum number of extended advertising events
    uint16_t max_ext_adv_evt;

    /// AUX Offset value in AuxPtr field on primary advertising channels (units of 30 or 300 us)
    uint16_t prim_aux_offset;

    /// AUX Offset value in AuxPtr field on secondary advertising channels (units of 30 or 300 us)
    uint16_t sec_aux_offset;

    /// Current advertising data buffer
    uint16_t curr_adv_data_buf;

    /// Current scan response data buffer
    uint16_t curr_scan_rsp_data_buf;

    /// Current advertising data length (in octets)
    uint16_t curr_adv_data_len;

    /// Current scan response data length (in octets)
    uint16_t curr_scan_rsp_data_len;

    /// Extended advertising event count
    uint8_t ext_adv_evt_cnt;

    /// Activity identifier
    uint8_t act_id;

    /// Periodic advertising activity identifier
    uint8_t per_act_id;

    /// Activity identifier for the connection (only for connectable advertising)
    uint8_t con_act_id;

    /// state of the advertising mode (@see enum ADV_EVT_STATE)
    uint8_t state;

    /// Scan request notification enable
    uint8_t scan_req_notif_en;

    /**
     * Local address type
     * 0x00 Public Device Address (default)
     * 0x01 Random Device Address
     * 0x02 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use public address.
     * 0x03 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use random address from LE_Set_Random_Address.
     */
    uint8_t own_addr_type;

    /**
     * Peer address type (needed for directed advertising and by privacy for undirected advertising)
     * 0x00 Public Device Address (default) or Public Identity Address
     * 0x01 Random Device Address or Random (static) Identity Address
     * 0x02 - 0xFF Reserved for future use
     */
    uint8_t peer_addr_type;

    /// Max advertising Tx power (in dBm)
    int8_t adv_tx_pwr;

    /// LLD primary advertising rate (@see enum lld_rate)
    uint8_t lld_prim_adv_rate;

    /// Secondary advertising max skip
    uint8_t sec_adv_max_skip;

    /// LLD secondary advertising rate (@see enum lld_rate)
    uint8_t lld_sec_adv_rate;

    /// Current auxiliary channel index
    uint8_t aux_ch_idx;

    /// Total number of Tx descriptors used
    uint8_t total_txdesc_count;

    /// Number of primary advertising channels used
    uint8_t prim_adv_ch_nb;

#if (EAVESDROPPING_SUPPORT)
    /// Primary advertising channel map
    uint8_t prim_adv_ch_map;

    /// Current channel map
    uint8_t curr_ch_map;

    /// Current Tx descriptor base index
    uint8_t curr_txdesc_base_index;

    /// Number of primary advertising packets that have been transmitted
    uint8_t prim_adv_pkt_tx;
#endif // EAVESDROPPING_SUPPORT

    /// Indicate whether sync info has to be updated
    bool sync_update;

    /// Indicate whether it is allowed to skip events
    bool skip_en;

#if (BT_53)
    /// Specify reasons for refreshing RPAs
    uint8_t chg_reasons;
#endif // (BT_53)

#if (EAVESDROPPING_SUPPORT)
#if (BLE_BROADCASTER || BLE_PERIPHERAL)
    /// Custom advertising parameters structure
    struct lld_cust_adv_params cust_adv_params;
    /// Timestamp of the last custom advertising event in half-slots
    uint32_t last_cust_evt_ts;

    /// Manual advertising parameters structure
    struct lld_manual_adv_params manual_adv_params;
#endif //(BLE_BROADCASTER || BLE_PERIPHERAL)
#endif //(EAVESDROPPING_SUPPORT)
};

/// Scan request structure
/*@TRACE*/
struct scan_req
{
    /// Scanner address
    struct bd_addr      scana;

    /// Advertiser address
    struct bd_addr      adva;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD ADV environment variable
__STATIC struct lld_adv_env_tag *lld_adv_env[BLE_ACTIVITY_MAX];

#if (EAVESDROPPING_SUPPORT)
    /**
    * Activity ID of the advertising set that uses manual trigger
    * Note: it is assumed that only 1 advertising set can use manual trigger
    * Note: it is assumed that ADVTRIG IRQ happens only when manual trigger advertising is enabled, no protection against spurious IRQ
    */
    uint8_t lld_adv_trig_id = 0;
#endif // EAVESDROPPING_SUPPORT

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void lld_adv_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);

#if (EAVESDROPPING_SUPPORT)
/**
 ****************************************************************************************
 * @brief Custom advertising scheduling
 ****************************************************************************************
 */
__STATIC bool lld_adv_custom_next_channel(uint8_t act_id)
{
    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];
    bool found = (adv_par->prim_adv_ch_nb == 1);
    bool new_evt = (found) ? true : false;
    uint8_t att = 0;

    ASSERT_ERR(adv_par->properties & ADV_LEGACY);

    while (!found && (att < 3))
    {
        adv_par->curr_ch_map = adv_par->curr_ch_map << 1;
        if (adv_par->curr_ch_map > 4)
        {
            new_evt = true;
            adv_par->curr_ch_map = 1;
        }

        if (adv_par->curr_ch_map & adv_par->prim_adv_ch_map)
        {
            found = true;
        }

        att++;
    }

    if (found)
    {
        uint8_t  cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

        // Set the new channel map
        em_ble_chmap2_advchmap_setf(cs_idx, adv_par->curr_ch_map);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    return (new_evt);
}

/**
 ****************************************************************************************
 * @brief Custom extended advertising scheduling
 ****************************************************************************************
 */
__STATIC bool lld_adv_ext_custom_next_channel(uint8_t act_id)
{
    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];
    bool found = false;
    bool new_evt = false;
    uint8_t att = 0;

    ASSERT_ERR(!(adv_par->properties & ADV_LEGACY));
    ASSERT_ERR(!adv_par->cust_adv_params.group_ext_adv_ind);

    while (!found && (att < 3))
    {
        adv_par->curr_ch_map = adv_par->curr_ch_map << 1;
        if (adv_par->curr_ch_map > 4)
        {
            new_evt = true;
            adv_par->curr_ch_map = 1;
        }

        if (adv_par->curr_ch_map & adv_par->prim_adv_ch_map)
        {
            found = true;
        }

        att++;
    }

    if (found)
    {
        uint8_t  cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

        // Set the new channel map
        em_ble_chmap2_advchmap_setf(cs_idx, adv_par->curr_ch_map);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    return (new_evt);
}

/**
 ****************************************************************************************
 * @brief Set timestamp and offset of the next advertising event
 ****************************************************************************************
 */
__STATIC void lld_adv_cust_set_evt_timings(uint8_t act_id)
{
    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];
    struct sch_arb_elt_tag *evt = &(lld_adv_env[act_id]->evt);

    uint32_t clk_offset = 0;
    uint16_t bit_offset = 0;
    int16_t diff = 0;
    uint32_t timestamp_piconet = evt->time.hs;

    // Check if the BD address corresponds to an active connection
    uint8_t link_id = lm_find_link_id(adv_par->cust_adv_params.bd_addr);

    if (link_id < MAX_NB_ACTIVE_ACL)
    {
        // Get the timings of the link
        ld_acl_timing_info_get(link_id, &clk_offset, &bit_offset);
        timestamp_piconet = CLK_ADD_2(timestamp_piconet, clk_offset);
    }

    diff = 2 * adv_par->cust_adv_params.time_offset - CO_MOD(timestamp_piconet, (2 * adv_par->cust_adv_params.periodicity));

    // Apply the half-microsecond alignment
    diff += (2 * adv_par->cust_adv_params.us_offset) / HALF_SLOT_SIZE;
    int16_t delay = bit_offset + ((2 * adv_par->cust_adv_params.us_offset) % HALF_SLOT_SIZE);

    // Adjust delay and timestamp if needed
    if (delay < 0)
    {
        uint8_t num_hs = CO_DIVIDE_CEIL((-delay), HALF_SLOT_SIZE);
        delay += num_hs * HALF_SLOT_SIZE;
        diff -= num_hs;
    }
    else if (delay >= HALF_SLOT_SIZE)
    {
        uint8_t num_hs = delay / HALF_SLOT_SIZE;
        delay -= num_hs * HALF_SLOT_SIZE;
        diff += num_hs;
    }

    while (diff < 0)
    {
        diff += 2 * adv_par->cust_adv_params.periodicity;
    }

    evt->time.hs = CLK_ADD_2(evt->time.hs, diff);
    evt->time.hus = (uint16_t)delay;
}

/**
 ****************************************************************************************
 * @brief Custom advertising scheduling
 ****************************************************************************************
 */
__STATIC void lld_adv_custom_sched(uint8_t act_id, bool new_evt)
{
    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];
    struct sch_arb_elt_tag *evt = &(lld_adv_env[act_id]->evt);

    bool found = false;
    uint8_t att = 0;
    uint16_t adv_delay_hs = 2 * adv_par->cust_adv_params.periodicity;

    ASSERT_ERR(adv_par->properties & ADV_LEGACY);

    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

    // Move timestamp
    evt->time.hs = (!new_evt) ? CLK_ADD_2(evt->time.hs, adv_delay_hs) : CLK_ADD_2(adv_par->last_cust_evt_ts, 2 * adv_par->intv);
    lld_adv_cust_set_evt_timings(act_id);

    if (new_evt && adv_par->cust_adv_params.rand_en)
    {
        // Add a random delay of 1 periodicity with probability 1/16
        if (co_rand_byte() % 16 == 0)
        {
            evt->time.hs = CLK_ADD_2(evt->time.hs, adv_delay_hs);
        }
    }

    while ((!found) && (att < 16))
    {
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            found = true;
        }
        else
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

            // Move timestamp
            evt->time.hs = CLK_ADD_2(evt->time.hs, adv_delay_hs);

            att++;
        }
    }

    if (found)
    {
        adv_par->state = ADV_EVT_WAIT;

        if (new_evt)
        {
            adv_par->last_cust_evt_ts = evt->time.hs;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

__STATIC void lld_adv_aux_ch_idx_set(uint8_t act_id);

/**
 ****************************************************************************************
 * @brief Custom extended advertising scheduling
 ****************************************************************************************
 */
__STATIC void lld_adv_ext_custom_sched(uint8_t act_id, bool new_evt)
{
    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];
    struct sch_arb_elt_tag *evt = &(lld_adv_env[act_id]->evt);

    bool found = false;
    uint8_t att = 0;
    uint16_t adv_delay_hs = 2 * adv_par->cust_adv_params.periodicity;

    ASSERT_ERR(!(adv_par->properties & ADV_LEGACY));

    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

    // Move timestamp
    evt->time.hs = (!new_evt) ? CLK_ADD_2(evt->time.hs, adv_delay_hs) : CLK_ADD_2(adv_par->last_cust_evt_ts, 2 * adv_par->intv);
    lld_adv_cust_set_evt_timings(act_id);

    if (new_evt && adv_par->cust_adv_params.rand_en)
    {
        // Add a random delay of 1 periodicity with probability 1/16
        if (co_rand_byte() % 16 == 0)
        {
            evt->time.hs = CLK_ADD_2(evt->time.hs, adv_delay_hs);
        }
    }

    while ((!found) && (att < 16))
    {
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            found = true;
        }
        else
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

            // Reset the number of primary advertising packets sent
            adv_par->prim_adv_pkt_tx = 0;
            // Select the first Tx descriptor in the chain
            adv_par->curr_txdesc_base_index = 0;
            if (!adv_par->cust_adv_params.group_ext_adv_ind)
            {
                // Select the first primary advertising channel
                adv_par->curr_ch_map = 4;
                lld_adv_ext_custom_next_channel(act_id);
            }

            // Move timestamp
            evt->time.hs = CLK_ADD_2(evt->time.hs, adv_delay_hs);

            att++;
        }
    }

    if (found)
    {
        if ((adv_par->curr_txdesc_base_index == 0)
                && (adv_par->cust_adv_params.group_ext_adv_ind || (adv_par->prim_adv_pkt_tx == 0)))
        {
            // Set auxiliary channel index in packets and CS
            lld_adv_aux_ch_idx_set(act_id);
        }

        adv_par->state = ADV_EVT_WAIT;

        if (new_evt)
        {
            adv_par->last_cust_evt_ts = evt->time.hs;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

#endif // EAVESDROPPING_SUPPORT

/**
 ****************************************************************************************
 * @brief Cleanup adv environment variable and send indication if needed
 ****************************************************************************************
 */
__STATIC void lld_adv_end(uint8_t act_id, bool indicate, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_END_FUNC_BIT, act_id, indicate, status);

    if (lld_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_adv_env[act_id]->evt);
        struct sch_arb_elt_tag *aux_evt = &(lld_adv_env[act_id]->aux_evt);
        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

        // Remove event
        sch_arb_remove(evt, false);

        // Remove aux event if needed
        if (adv_par->skip_en)
        {
            sch_arb_remove(aux_evt, false);
        }

#if (EAVESDROPPING_SUPPORT)
        if (adv_par->manual_adv_params.en)
        {
            // Disable the advertising trigger
            ble_advtrigcntl_advtrigen_setf(0);
            ble_intack0_advtrigintack_clearf(1);

            // Clear the ADV_PENDING status bit
            ip_deepslcntl_clear_adv_pending_setf(1);

            // Re-enable deep sleep
            rwip_prevent_sleep_clear(RW_BLE_ACTIVE_MODE);
        }

        if (!(adv_par->properties & ADV_LEGACY) && adv_par->cust_adv_params.en)
        {
            // Re-enable automatic auxptr management
            ble_advtim_tx_auxptr_thr_setf(LLD_TX_AUX_PTR_THR);
        }
        else
#endif // EAVESDROPPING_SUPPORT

            if (adv_par->properties & ADV_DIRECT_HI)
            {
                // Unregister high duty cycle advertising from scheduling parameters
                sch_slice_bg_remove(BLE_HDC_ADV);
            }

        if (indicate)
        {
            struct lld_adv_end_ind *ind = KE_MSG_ALLOC(LLD_ADV_END_IND, TASK_LLM, TASK_NONE, lld_adv_end_ind);

            // Report advertising end to LLM
            ind->act_id = act_id;
            ind->status = status;
            ind->connected = false;
            ind->nb_ext_adv_evts = ((status != CO_ERROR_NO_ERROR) && (adv_par->max_ext_adv_evt != 0)) ? adv_par->ext_adv_evt_cnt : 0;
            ke_msg_send(ind);
        }

        // Remove permission/status of CS as now unused
        DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(EM_BLE_CS_ACT_ID_TO_INDEX(act_id))), REG_EM_BLE_CS_SIZE, false, false, false);

        // Free event memory
        ke_free(lld_adv_env[act_id]);
        lld_adv_env[act_id] = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Sets the SyncInfo field in AUX_ADV_IND
 ****************************************************************************************
 */
__STATIC void lld_adv_sync_info_set(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_SYNC_INFO_SET_FUNC_BIT, act_id);

    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];
    struct sch_arb_elt_tag *evt = (adv_par->skip_en) ? &adv_par->aux_evt : &adv_par->evt;

    ASSERT_ERR((!(adv_par->properties & (ADV_LEGACY | ADV_CON | ADV_SCAN))) && (adv_par->per_adv_intv != 0) && (!adv_par->sync_update));

    struct le_ch_map ch_map;
    uint32_t sync_ind_ts;
    int32_t diff_hs, diff_us;
    uint16_t pa_evt_cnt, sync_pkt_offset;
    uint8_t offset_units, byte0, byte1, byte16, byte17;
    uint8_t status;

    // Get the latest values of the AUX_SYNC_IND PDU timestamp and paEventCounter from the periodic advertiser
    status = lld_per_adv_sync_info_get(adv_par->per_act_id, &sync_ind_ts, &pa_evt_cnt, &ch_map);

    if (status == CO_ERROR_NO_ERROR)
    {
        // Make sure that the AUX_SYNC_IND comes after the end of the AUX_ADV_IND
        diff_hs  = CLK_DIFF(evt->time.hs, sync_ind_ts);
        diff_us = ((diff_hs * HALF_SLOT_SIZE) >> 1) - adv_par->prim_aux_offset * 30;
        if (diff_us <= (int32_t)(adv_par->aux_adv_ind_dur_us + BLE_AFS_DUR))
        {
            uint32_t add_us = adv_par->aux_adv_ind_dur_us + BLE_AFS_DUR - diff_us;
            uint16_t add_per_adv_intv = CO_DIVIDE_CEIL(2 * add_us, adv_par->per_adv_intv * 4 * HALF_SLOT_SIZE);

            sync_ind_ts = CLK_ADD_2(sync_ind_ts, add_per_adv_intv * adv_par->per_adv_intv * 4);
            pa_evt_cnt += add_per_adv_intv;
            diff_hs = CLK_DIFF(evt->time.hs, sync_ind_ts);
            diff_us = ((diff_hs * HALF_SLOT_SIZE) >> 1) - adv_par->prim_aux_offset * 30;
            ASSERT_ERR(diff_us >= (int32_t)(adv_par->aux_adv_ind_dur_us + BLE_AFS_DUR));
        }

        if (diff_us <= OFFSET_LIMIT)
        {
            sync_pkt_offset = (diff_us < OFFSET_THRESHOLD) ? (diff_us / 30) : (diff_us / 300);
            offset_units = (diff_us < OFFSET_THRESHOLD) ? 0 : 1;
        }
        else
        {
            // A value of 0 for the Sync Packet Offset indicates that the time to the next AUX_SYNC_IND packet is greater than can be represented
            sync_pkt_offset = 0;
            offset_units = 0;
        }

        byte0 = sync_pkt_offset & 0xFF;
        byte1 = ((sync_pkt_offset >> 8) & 0x1F) | (offset_units << 5);
        byte16 = pa_evt_cnt & 0xFF;
        byte17 = (pa_evt_cnt >> 8) & 0xFF;
        ch_map.map[4] = (ch_map.map[4] & 0x1F) | (rwip_sca_get() << 5);

        // Sets SyncInfo field in AUX_ADV_IND
        {
            uint8_t aux_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, 1);
            uint16_t ext_header_txbuf_offset = em_ble_txaedataptr_getf(aux_txdesc_idx);
            uint8_t sync_info_offset = 0;

            // Verify that the extended header offset is initialised
            ASSERT_ERR(ext_header_txbuf_offset != 0);

            if (!(adv_par->properties & ADV_ANONYMOUS))
            {
                sync_info_offset += BD_ADDR_LEN;
            }
            if (adv_par->properties & ADV_DIRECT)
            {
                sync_info_offset += BD_ADDR_LEN;
            }

            // The ADI field is mandatory in AUX_ADV_IND
            sync_info_offset += BLE_EXT_ADI_LEN;

            if (em_ble_txaeheader_txauxptr_getf(aux_txdesc_idx))
            {
                sync_info_offset += BLE_EXT_AUX_PTR_LEN;
            }

            // Copy 4 bytes
            em_wr((void *)&byte0, ext_header_txbuf_offset + sync_info_offset, 1);
            em_wr((void *)&byte1, ext_header_txbuf_offset + sync_info_offset + 1, 1);
            em_wr((void *)&byte16, ext_header_txbuf_offset + sync_info_offset + 16, 1);
            em_wr((void *)&byte17, ext_header_txbuf_offset + sync_info_offset + 17, 1);

            // Copy channel map and SCA
            em_wr((void *)&ch_map.map[0], ext_header_txbuf_offset + sync_info_offset + 4, LE_CH_MAP_LEN);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Sets the auxiliary channel index in packets and CS
 ****************************************************************************************
 */
__STATIC void lld_adv_aux_ch_idx_set(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_AUX_CH_IDX_SET_FUNC_BIT, act_id);

    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

    // Point to ADV_EXT_IND
    uint8_t txdesc_count = 0;
    uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_count);

    if (em_ble_txaeheader_txauxptr_getf(txdesc_idx))
    {
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(adv_par->act_id);

        // Select an auxiliary channel index
        adv_par->aux_ch_idx  = lld_ch_idx_get();

        // Update the channel index
        em_ble_txauxptr0_tx_ll_ch_setf(txdesc_idx, adv_par->aux_ch_idx);

        // Update CS
        em_ble_chmap2_ch_aux_setf(cs_idx, adv_par->aux_ch_idx);

        // Update all auxiliary packets
        while (1)
        {
            // Point to next packet
            txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, ++txdesc_count);

            // Release descriptor
            em_ble_txcntl_txdone_setf(txdesc_idx, 0);

            // If last packet, exit the loop
            if (txdesc_count == (adv_par->total_txdesc_count - 1))
                break;

            // Update the channel index
            em_ble_txauxptr0_tx_ll_ch_setf(txdesc_idx, adv_par->aux_ch_idx);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Prepares an extended advertising packet
 ****************************************************************************************
 */

__STATIC uint32_t lld_adv_ext_pkt_prepare(uint8_t act_id, uint8_t txdesc_idx,
        uint8_t type, uint8_t mode, bool adva, bool targeta, bool adi, bool aux_ptr, bool sync_info, bool tx_power,
        uint16_t *ext_header_txbuf_offset, uint16_t *data_txbuf_offset, uint16_t *data_len)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_EXT_PKT_PREPARE_FUNC_BIT, uint32_t, 13, act_id, txdesc_idx, type, mode, adva, targeta, adi,
                                            aux_ptr, sync_info, tx_power, ext_header_txbuf_offset, data_txbuf_offset, data_len);

    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

    uint32_t dur_us = 0;
    uint32_t aux_offset = 0;
    bool offset_units = 0;
    uint8_t ext_header_len, offset;
    uint8_t adv_data_len = 0;
    bool prim_pkt = (txdesc_idx == EM_BLE_TXDESC_INDEX(adv_par->act_id, 0));

    // Set advertising data descriptor next pointer to 0
    em_ble_txcntl_nextptr_setf(txdesc_idx, 0);

    ext_header_len = 0;
    offset = 0;

    // First calculate the extended header length
    if (adva)
    {
        ext_header_len += BD_ADDR_LEN;
    }
    if (targeta)
    {
        ext_header_len += BD_ADDR_LEN;
    }
    if (adi)
    {
        ext_header_len += BLE_EXT_ADI_LEN;
    }
    if (aux_ptr)
    {
        ext_header_len += BLE_EXT_AUX_PTR_LEN;
    }
    if (sync_info)
    {
        ext_header_len += BLE_EXT_SYNC_LEN;
    }
    if (tx_power)
    {
        ext_header_len += BLE_EXT_TX_PWR_LEN;
    }
    if (ext_header_len > 0)
    {
        ext_header_len += 1; // Extended header flags field (1 octet)
    }

    // The aux_ptr field needs to be set if the data does not fit in a single packet
    if (((ext_header_len + (*data_len)) > BLE_ADV_FRAG_SIZE_TX) && (aux_ptr == false))
    {
        aux_ptr = true;
        if (ext_header_len == 0)
        {
            ext_header_len += 1; // Extended header flags field (1 octet)
        }
        ext_header_len += BLE_EXT_AUX_PTR_LEN;
    }

    if (adva)
    {
        offset += BD_ADDR_LEN;
    }
    if (targeta)
    {
        // Copy target address
        em_wr((void *)&adv_par->peer_addr, *ext_header_txbuf_offset + offset, BD_ADDR_LEN);

        offset += BD_ADDR_LEN;
    }
    if (adi)
    {
        // Copy adv_data_info
        em_wr((void *)&adv_par->adi, *ext_header_txbuf_offset + offset, BLE_EXT_ADI_LEN);

        offset += BLE_EXT_ADI_LEN;
    }
    if (aux_ptr)
    {
        offset += BLE_EXT_AUX_PTR_LEN;
    }
    if (sync_info)
    {
        uint8_t sync_info_field[BLE_EXT_SYNC_LEN];
        uint8_t sca = rwip_sca_get();

        memset(&sync_info_field[0], 0x00, BLE_EXT_SYNC_LEN);
        co_write16(&sync_info_field[2], co_htobs(adv_par->per_adv_intv));

        lld_per_adv_init_info_get(adv_par->per_act_id, (struct access_addr *)&sync_info_field[9], (struct crc_init *)&sync_info_field[13], (struct le_ch_map *)&sync_info_field[4]);

        sync_info_field[8] = (sync_info_field[8] & 0x1F) | (sca << 5);

        // Copy sync_info
        em_wr((void *)&sync_info_field[0], *ext_header_txbuf_offset + offset, BLE_EXT_SYNC_LEN);

        offset += BLE_EXT_SYNC_LEN;
    }
    if (tx_power)
    {
        uint8_t txpwr_idx;
        int8_t rad_tx_pwr;
        uint8_t rf_mod;

        if (adv_par->adv_tx_pwr != ADV_TX_PWR_NO_PREF)
        {
            // Update the transceiver Tx power
            rf_mod = rwip_rf.txpwr_mdlt_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), 0);
            txpwr_idx = rwip_rf.txpwr_cs_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), TXPWR_CS_LOWER, rf_mod);
            em_ble_txrxcntl_set(EM_BLE_CS_ACT_ID_TO_INDEX(act_id), txpwr_idx);
        }
        else
        {
            // Using max transceiver Tx power
            txpwr_idx = rwip_rf.txpwr_max;
            rf_mod = rwip_rf.txpwr_max_mod;
        }

        // Determine the radiative Tx power
        rad_tx_pwr = lld_tx_power_dbm_get(txpwr_idx, rf_mod);

        // Copy Tx power
        em_wr((void *)&rad_tx_pwr, *ext_header_txbuf_offset + offset, BLE_EXT_TX_PWR_LEN);

        offset += BLE_EXT_TX_PWR_LEN;
    }
    if (*data_len > 0)
    {
        uint8_t max_adv_data_len = BLE_ADV_FRAG_SIZE_TX - ext_header_len;

        adv_data_len = ((*data_len) < max_adv_data_len) ? *data_len : max_adv_data_len;

        (*data_len) -= adv_data_len;
    }

    em_ble_txaeheader_pack(txdesc_idx, /*txrsvd*/ 0, /*txpow*/ tx_power,
                           /*txsync*/ sync_info, /*txauxptr*/ aux_ptr, /*txadi*/ adi,
                           /*txsupp*/ 0, /*txtgta*/ targeta, /*txadva*/ adva,
                           /*txaemode*/ mode, /*txaelength*/ ext_header_len);

    if (prim_pkt)
    {
        dur_us = adv_par->prim_adv_ch_nb * (ble_util_pkt_dur_in_us((ext_header_len + adv_data_len + 1), adv_par->lld_prim_adv_rate) + BLE_IFS_DUR);
    }
    else
    {
        dur_us = ble_util_pkt_dur_in_us((ext_header_len + adv_data_len + 1), adv_par->lld_sec_adv_rate);

        // Save the duration of the AUX_ADV_IND PDU
        if (txdesc_idx == EM_BLE_TXDESC_INDEX(adv_par->act_id, 1))
        {
            adv_par->aux_adv_ind_dur_us = dur_us;
        }
    }

    if (aux_ptr)
    {
        dur_us += BLE_AFS_DUR;

        if (dur_us <= OFFSET_LIMIT)
        {
            aux_offset = (dur_us < OFFSET_THRESHOLD) ? (dur_us / 30) : (dur_us / 300);
            offset_units = (dur_us < OFFSET_THRESHOLD) ? 0 : 1;

            aux_offset++;
            if (prim_pkt)
                adv_par->prim_aux_offset = aux_offset;
            else
                adv_par->sec_aux_offset = aux_offset;

            dur_us = (offset_units) ? aux_offset * 300 : aux_offset * 30;
        }
    }

    em_ble_txauxptr0_pack(txdesc_idx, /*txauxoffsetlsb*/ aux_offset, /*txauxoffsetunit*/ offset_units, /*txauxca*/ (rwip_max_drift_get() <= 50), /*txllch*/ adv_par->aux_ch_idx);
    em_ble_txauxptr1_pack(txdesc_idx, /*txauxphy*/ adv_par->lld_sec_adv_rate, /*txauxoffsetmsb*/(aux_offset >> 8));
    em_ble_txaedataptr_setf(txdesc_idx, *ext_header_txbuf_offset);

    (*ext_header_txbuf_offset) += offset;

    // Set advertising data Tx descriptor length to (extended header length + advertising data length + 1)
    em_ble_txphadv_pack(txdesc_idx, (ext_header_len + adv_data_len + 1) /*txadvlen*/,
                        targeta && (adv_par->peer_addr_type & ADDR_RAND) /*txrxadd*/,
                        adva && (adv_par->own_addr_type & ADDR_RAND) /*txtxadd*/,
                        0 /*txchsel2*/,
                        0 /*txadvrfu*/,
                        type /*txtype*/);

    // Set data pointer
    em_ble_txdataptr_setf(txdesc_idx, *data_txbuf_offset);

    (*data_txbuf_offset) += adv_data_len;

    // Release descriptor
    em_ble_txcntl_txdone_setf(txdesc_idx, 0);

    return (dur_us);
}

/**
 ****************************************************************************************
 * @brief Constructs the extended advertising packet chain
 ****************************************************************************************
 */
__STATIC void lld_adv_ext_chain_construct(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_EXT_CHAIN_CONSTRUCT_FUNC_BIT, act_id);

    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

    uint8_t adv_mode = adv_par->properties & (ADV_CON | ADV_SCAN);
    uint8_t type, mode;
    bool adva, targeta, adi, aux_ptr, sync_info, tx_power;

    uint8_t txdesc_index = 0;
    uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);
    uint16_t ext_header_txbuf_offset = EM_BLE_ADVEXTHDRTXBUF_OFF(adv_par->act_id);
    uint16_t data_txbuf_offset = 0;
    uint16_t rem_data_len = 0;

    txdesc_index++;
    uint8_t aux_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);
    uint8_t prev_txdesc_idx;

    // Select an auxiliary channel index
    adv_par->aux_ch_idx  = lld_ch_idx_get();

    adv_par->prim_aux_offset = 0;
    adv_par->sec_aux_offset = 0;
    adv_par->total_adv_dur_us = 0;

    if (!adv_mode) // Non-scannable, non-connectable
    {
        if ((adv_par->curr_adv_data_len == 0) && (adv_par->per_adv_intv == 0))
        {
            // Prepare a single ADV_EXT_IND packet without aux_ptr
            type = BLE_ADV_EXT_IND;
            mode = adv_mode;
            adva = !(adv_par->properties & ADV_ANONYMOUS);
            targeta = (adv_par->properties & ADV_DIRECT);
            adi = false;
            aux_ptr = false;
            sync_info = false;
            tx_power = (adv_par->properties & ADV_TX_PWR);
            adv_par->total_adv_dur_us += lld_adv_ext_pkt_prepare(act_id, txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

            // Cannot skip events if there is no AUX_ADV_IND
            adv_par->skip_en = false;
        }
        else // Data present
        {
            // Prepare the ADV_EXT_IND packet with the aux_ptr field pointing to the next packet
            type = BLE_ADV_EXT_IND;
            mode = adv_mode;
            adva = false;
            targeta = false;
            adi = true;
            aux_ptr = true;
            sync_info = false;
            tx_power = false;
            adv_par->total_adv_dur_us += lld_adv_ext_pkt_prepare(act_id, txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

            prev_txdesc_idx = aux_txdesc_idx;

            data_txbuf_offset = adv_par->curr_adv_data_buf;
            rem_data_len = adv_par->curr_adv_data_len;

            // Prepare the AUX_ADV_IND packet, aux_ptr will be automatically set if necessary
            type = BLE_AUX_ADV_IND;
            mode = adv_mode;
            adva = !(adv_par->properties & ADV_ANONYMOUS);
            targeta = (adv_par->properties & ADV_DIRECT);
            adi = true;
            aux_ptr = false;
            sync_info = (adv_par->per_adv_intv != 0);
            tx_power = (adv_par->properties & ADV_TX_PWR);
            adv_par->total_adv_dur_us += lld_adv_ext_pkt_prepare(act_id, aux_txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

            // Some of the info in the packet have to be duplicated in the CS
            em_ble_chmap2_ch_aux_setf(cs_idx, adv_par->aux_ch_idx);
            em_ble_thrcntl_ratecntl_aux_rate_setf(cs_idx, adv_par->lld_sec_adv_rate);
            em_ble_auxtxdescptr_setf(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, aux_txdesc_idx));


            // Construct a chain of AUX_CHAIN_IND packets if needed
            while (rem_data_len)
            {
                txdesc_index++;
                uint8_t chain_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);

                // Prepare an AUX_CHAIN_IND packet, aux_ptr will be automatically set if necessary
                type = BLE_AUX_CHAIN_IND;
                mode = 0;
                adva = false;
                targeta = false;
                adi = true;
                aux_ptr = false;
                sync_info = false;
                tx_power = false;
                adv_par->total_adv_dur_us += lld_adv_ext_pkt_prepare(act_id, chain_txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

                // Link previous Tx descriptor to this one
                em_ble_txcntl_nextptr_setf(prev_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, chain_txdesc_idx));
                prev_txdesc_idx = chain_txdesc_idx;
            }
        }
    }
    else if (adv_mode & ADV_CON) // Connectable
    {
        // Prepare the ADV_EXT_IND packet with the aux_ptr field pointing to the next packet
        type = BLE_ADV_EXT_IND;
        mode = adv_mode;
        adva = false;
        targeta = false;
        adi = true;
        aux_ptr = true;
        sync_info = false;
        tx_power = false;
        adv_par->total_adv_dur_us += lld_adv_ext_pkt_prepare(act_id, txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

        data_txbuf_offset = adv_par->curr_adv_data_buf;
        rem_data_len = adv_par->curr_adv_data_len;

        // Prepare the ADV_EXT_IND packet
        type = BLE_AUX_ADV_IND;
        mode = adv_mode;
        adva = true;
        targeta = (adv_par->properties & ADV_DIRECT);
        adi = true;
        aux_ptr = false;
        sync_info = false;
        tx_power = (adv_par->properties & ADV_TX_PWR);
        adv_par->total_adv_dur_us += lld_adv_ext_pkt_prepare(act_id, aux_txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

        // The data must fit into a single packet
        ASSERT_ERR(rem_data_len == 0);

        // Some of the info in the packet have to be duplicated in the CS
        em_ble_chmap2_ch_aux_setf(cs_idx, adv_par->aux_ch_idx);
        em_ble_thrcntl_ratecntl_aux_rate_setf(cs_idx, 1);
        em_ble_auxtxdescptr_setf(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, aux_txdesc_idx));

        // Extend the duration to include a potential AUX_CONNECT_REQ
        {
            uint8_t rx_phy = (adv_par->lld_sec_adv_rate == CO_RATE_500KBPS) ? CO_RATE_125KBPS : adv_par->lld_sec_adv_rate;
            adv_par->total_adv_dur_us += BLE_IFS_DUR + ble_util_pkt_dur_in_us(PDU_CON_REQ_LEN, rx_phy);
        }

        txdesc_index++;
        uint8_t connect_rsp_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);

        // Link previous Tx descriptor to this one
        em_ble_txcntl_nextptr_setf(aux_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, connect_rsp_txdesc_idx));

        // Prepare the AUX_CONNECT_RSP packet
        type = BLE_AUX_CONNECT_RSP;
        mode = 0;
        adva = true;
        targeta = true;
        adi = false;
        aux_ptr = false;
        sync_info = false;
        tx_power = false;
        lld_adv_ext_pkt_prepare(act_id, connect_rsp_txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);
    }
    else if (adv_mode & ADV_SCAN) // Scannable
    {
        uint8_t scan_rsp_txdesc_idx;

        type = BLE_ADV_EXT_IND;
        mode = adv_mode;
        adva = false;
        targeta = false;
        adi = true;
        aux_ptr = true;
        sync_info = false;
        tx_power = false;
        adv_par->total_adv_dur_us += lld_adv_ext_pkt_prepare(act_id, txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

        // Prepare the AUX_ADV_IND packet
        type = BLE_AUX_ADV_IND;
        mode = adv_mode;
        adva = true;
        targeta = (adv_par->properties & ADV_DIRECT);
        adi = true;
        aux_ptr = false;
        sync_info = false;
        tx_power = (adv_par->properties & ADV_TX_PWR);
        adv_par->total_adv_dur_us += lld_adv_ext_pkt_prepare(act_id, aux_txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

        // Some of the info in the packet have to be duplicated in the CS
        em_ble_chmap2_ch_aux_setf(cs_idx, adv_par->aux_ch_idx);
        em_ble_thrcntl_ratecntl_aux_rate_setf(cs_idx, 1);
        em_ble_auxtxdescptr_setf(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, aux_txdesc_idx));

        // Extend the duration to include a potential AUX_SCAN_REQ
        {
            uint8_t rx_phy = (adv_par->lld_sec_adv_rate == CO_RATE_500KBPS) ? CO_RATE_125KBPS : adv_par->lld_sec_adv_rate;
            adv_par->total_adv_dur_us += BLE_IFS_DUR + ble_util_pkt_dur_in_us(PDU_SCAN_REQ_LEN, rx_phy);
        }

        txdesc_index++;
        scan_rsp_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);
        data_txbuf_offset = adv_par->curr_scan_rsp_data_buf;
        rem_data_len = adv_par->curr_scan_rsp_data_len;

        prev_txdesc_idx = scan_rsp_txdesc_idx;

        // Link previous descriptor to this one
        em_ble_txcntl_nextptr_setf(aux_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, scan_rsp_txdesc_idx));

        // Prepare the AUX_SCAN_RSP packet, aux_ptr will be automatically set if necessary
        type = BLE_AUX_SCAN_RSP;
        mode = 0;
        adva = true;
        targeta = false;
        adi = true;
        aux_ptr = false;
        sync_info = false;
        tx_power = false;
        lld_adv_ext_pkt_prepare(act_id, scan_rsp_txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

        // Construct a chain of AUX_CHAIN_IND packets if needed
        while (rem_data_len)
        {
            txdesc_index++;
            uint8_t chain_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);

            // Prepare an AUX_CHAIN_IND packet, aux_ptr will be automatically set if necessary
            type = BLE_AUX_CHAIN_IND;
            mode = 0;
            adva = false;
            targeta = false;
            adi = false;
            aux_ptr = false;
            sync_info = false;
            tx_power = false;
            lld_adv_ext_pkt_prepare(act_id, chain_txdesc_idx, type, mode, adva, targeta, adi, aux_ptr, sync_info, tx_power, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len);

            // Link previous Tx descriptor to this one
            em_ble_txcntl_nextptr_setf(prev_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, chain_txdesc_idx));
            prev_txdesc_idx = chain_txdesc_idx;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    adv_par->total_txdesc_count = txdesc_index + 1;

#if (EAVESDROPPING_SUPPORT)
    if (adv_par->cust_adv_params.en)
    {
        // Set minimum duration according to size
        lld_adv_env[act_id]->evt.duration_min = co_max(2 * adv_par->aux_adv_ind_dur_us + BLE_RESERVATION_TIME_MARGIN_HUS, HALF_SLOT_SIZE);
    }
    else
#endif // EAVESDROPPING_SUPPORT
    {
        // Set minimum duration according to size
        lld_adv_env[act_id]->evt.duration_min = 2 * adv_par->total_adv_dur_us + BLE_RESERVATION_TIME_MARGIN_HUS;
    }
}

/**
 ****************************************************************************************
 * @brief Update the ADI DID
 ****************************************************************************************
 */
__STATIC void lld_adv_adi_did_update(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_ADI_DID_UPDATE_FUNC_BIT, act_id);

    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];
    // Generate a new ADI DID
    uint16_t adi_did = co_rand_hword() & BLE_ADI_DID_MASK;

    // DID shall not be the same as the previously used one, LL 4.4.2.11
    while (adi_did == GETF(adv_par->adi, BLE_ADI_DID))
    {
        adi_did = co_rand_hword() & BLE_ADI_DID_MASK;
    }

    // Update the ADI DID
    SETF(adv_par->adi, BLE_ADI_DID, adi_did);
}

#if (BT_53)
/**
 ****************************************************************************************
 * @brief Refresh the RPAs
 ****************************************************************************************
 */
__STATIC void lld_adv_rpa_refresh(uint8_t act_id)
{
    // Retrieve RAL pointer from CS
    uint8_t ral_idx = ((em_ble_peer_ralptr_getf(EM_BLE_CS_ACT_ID_TO_INDEX(act_id)) << 2) - REG_EM_BLE_RAL_ADDR_GET(0)) / REG_EM_BLE_RAL_SIZE;

    // check if resolving address list entry is valid
    if (em_ble_ral_info_entry_valid_getf(ral_idx))
    {
        // retrieve if local and peer irk are valid
        bool l_irk_valid = em_ble_ral_info_local_irk_valid_getf(ral_idx);
        bool p_irk_valid = em_ble_ral_info_peer_irk_valid_getf(ral_idx);

        // set the renew bit to 1 if corresponding IRK is valid
        em_ble_ral_info_local_rpa_renew_setf(ral_idx, l_irk_valid);
        em_ble_ral_info_peer_rpa_renew_setf(ral_idx, p_irk_valid);
    }
}
#endif // (BT_53)

/**
 ****************************************************************************************
 * @brief Set advertising data  // COMMENT [FBE]  Doxygen comment
 ****************************************************************************************
 */
__STATIC void lld_adv_adv_data_set(uint8_t act_id, uint8_t len, uint16_t data, bool release_old_buf)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_ADV_DATA_SET_FUNC_BIT, act_id, len, data, release_old_buf);

    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

    uint16_t old_buf_offset = 0;

#if (BT_53)
    if (release_old_buf && (adv_par->chg_reasons & RPA_ADV_DATA_CHG))
    {
        // Refresh RPAs
        lld_adv_rpa_refresh(act_id);
    }
#endif // (BT_53)

    if (adv_par->properties & ADV_LEGACY)
    {
        uint8_t adv_txdesc_idx = EM_BLE_TXDESC_INDEX(act_id, 0);
        uint16_t adv_data_txbuf_offset = data;

        if (release_old_buf)
        {
            old_buf_offset = em_ble_txdataptr_getf(adv_txdesc_idx);
        }

        // Set advertising data Tx descriptor fields
        em_ble_txphadv_txadvlen_setf(adv_txdesc_idx, (BD_ADDR_LEN + len));
        em_ble_txdataptr_setf(adv_txdesc_idx, adv_data_txbuf_offset);
    }
    else
    {
        uint8_t adv_txdesc_idx = EM_BLE_TXDESC_INDEX(act_id, 1);

        // Update the ADI DID
        lld_adv_adi_did_update(act_id);

        if (release_old_buf)
        {
            old_buf_offset = em_ble_txdataptr_getf(adv_txdesc_idx);
        }

        adv_par->curr_adv_data_len = len;
        adv_par->curr_adv_data_buf = data;
        lld_adv_ext_chain_construct(adv_par->act_id);
    }

    if ((release_old_buf) && (old_buf_offset != 0))
    {
        ble_util_buf_adv_tx_free(old_buf_offset);
    }
}

/**
 ****************************************************************************************
 * @brief Set scan response data // COMMENT [FBE]  Doxygen comment
 ****************************************************************************************
 */
__STATIC void lld_adv_scan_rsp_data_set(uint8_t act_id, uint8_t len, uint16_t data, bool release_old_buf)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_SCAN_RSP_DATA_SET_FUNC_BIT, act_id, len, data, release_old_buf);

    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

    uint16_t old_buf_offset = 0;

#if (BT_53)
    if (release_old_buf && (adv_par->chg_reasons & RPA_SCAN_RSP_DATA_CHG))
    {
        // Refresh RPAs
        lld_adv_rpa_refresh(act_id);
    }
#endif // (BT_53)

    if (adv_par->properties & ADV_LEGACY)
    {
        uint8_t adv_txdesc_idx = EM_BLE_TXDESC_INDEX(act_id, 0);
        uint8_t scan_rsp_txdesc_idx = EM_BLE_TXDESC_INDEX(act_id, 1);
        uint16_t scan_rsp_data_txbuf_offset = data;

        if (release_old_buf)
        {
            old_buf_offset = em_ble_txdataptr_getf(scan_rsp_txdesc_idx);
        }

        // Set scan response data Tx descriptor fields
        em_ble_txphadv_txtype_setf(scan_rsp_txdesc_idx, BLE_SCAN_RSP);
        em_ble_txphadv_txadvlen_setf(scan_rsp_txdesc_idx, (BD_ADDR_LEN + len));
        em_ble_txdataptr_setf(scan_rsp_txdesc_idx, scan_rsp_data_txbuf_offset);

        // Set scan response data descriptor next pointer
        em_ble_txcntl_nextptr_setf(scan_rsp_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, adv_txdesc_idx));
    }
    else
    {
        uint8_t scan_rsp_txdesc_idx = EM_BLE_TXDESC_INDEX(act_id, 2);

        // Update the ADI DID
        lld_adv_adi_did_update(act_id);

        if (release_old_buf)
        {
            old_buf_offset = em_ble_txdataptr_getf(scan_rsp_txdesc_idx);
        }

        adv_par->curr_scan_rsp_data_len = len;
        adv_par->curr_scan_rsp_data_buf = data;
        lld_adv_ext_chain_construct(adv_par->act_id);
    }

    if ((release_old_buf) && (old_buf_offset != 0))
    {
        ble_util_buf_adv_tx_free(old_buf_offset);
    }
}

/**
 ****************************************************************************************
 * @brief Prepares the SyncInfo field in AUX_ADV_IND
 ****************************************************************************************
 */
__STATIC void lld_adv_sync_info_prepare(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_SYNC_INFO_PREPARE_FUNC_BIT, act_id);

    // Point to parameters
    struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

    // Update the advertising DID when periodic advertising is enabled
    if (adv_par->per_adv_intv != 0)
    {
        // Update the ADI DID
        lld_adv_adi_did_update(act_id);
    }

    lld_adv_ext_chain_construct(adv_par->act_id);
}

/**
 ****************************************************************************************
 * @brief Check the reception during activity
 ****************************************************************************************
 */
__STATIC bool lld_adv_pkt_rx(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_PKT_RX_FUNC_BIT, bool, act_id);

    bool con_req_received = false;

    // Check if a packet has been received
    while (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(act_id)))
    {
        // Get current RX descriptor index
        uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;
        // Retrieve RX status and type
        uint16_t rxstat_pkt = em_ble_rxstatadv_get(rxdesc_idx);

        // Trace the current RX descriptor
        TRC_REQ_RX_DESC(LLD_ADV, act_id, REG_EM_BLE_RX_DESC_ADDR_GET(rxdesc_idx));

        // Check if packet reception is correct
        // Workaround HW issue of not setting EM_BLE_LEN_ERR_BIT for 0 length BLE_CONNECT_REQ/BLE_SCAN_REQ
        if (((rxstat_pkt & LLD_ADV_ERR_MASK) == 0) && (em_ble_rxphadv_rxadvlen_getf(rxdesc_idx) != 0))
        {
            uint8_t rxtype = em_ble_rxphadv_rxtype_getf(rxdesc_idx);

#if (BLE_PERIPHERAL)
            // COMMENT [FBE] to reduce duration of the process in interrupt context, we should only provide some privacy,
            // sync info and data pointer, and then let LLM to do the job.
            if (rxtype == BLE_CONNECT_IND)
            {
                struct pdu_con_req data;
                struct bd_addr init_id_addr, init_rpa, adv_rpa;

                em_rd((void *)&data, em_ble_rxdataptr_get(rxdesc_idx), PDU_CON_REQ_LEN);

                // Check received parameters
                if (!((data.lldata.timeout > CON_SUP_TO_MAX) || (data.lldata.timeout < CON_SUP_TO_MIN) ||
                        (data.lldata.interval < CON_INTERVAL_MIN) || (data.lldata.interval > CON_INTERVAL_MAX) ||
                        (data.lldata.latency > CON_LATENCY_MAX) ||
                        // The connSupervisionTimeout shall be larger than (1 + connSlaveLatency) * connInterval * 2. (See [Vol 6] Part B, Section 4.5.2).
                        // supervision timeout (mult of 10 ms); conn interval (mult of 1.25 ms)
                        // (hci_sup_to * 10) <= ((1+latency)* interval*1.25*2)
                        //to simplify computation and remove floating point we refactor everything by 4/10
                        // (hci_sup_to * 4) <= ((1+latency) * interval
                        (((uint32_t)data.lldata.timeout << 2) <= ((1 + (uint32_t)data.lldata.latency) * (uint32_t)data.lldata.interval)) ||
                        (ble_util_nb_good_channels(&data.lldata.ch_map) < DATA_CHANNEL_USED_NB_MIN)))
                {
                    uint8_t init_addr_type;
                    // Retrieve RAL pointer from RX descriptor
                    uint16_t ral_ptr = em_ble_rxralptr_getf(rxdesc_idx);

                    con_req_received = true;

                    if (ral_ptr != 0)
                    {
                        uint8_t ral_idx = (ral_ptr - REG_EM_BLE_RAL_ADDR_GET(0)) / REG_EM_BLE_RAL_SIZE;
                        // Get initiator's ID address from RAL
                        em_rd(&init_id_addr, ral_ptr + EM_BLE_RAL_PEER_ID_INDEX * 2, BD_ADDR_LEN);
                        // Get peer address type
                        init_addr_type = em_ble_ral_info_peer_id_type_getf(ral_idx);
                        // Check if initiator is using an RPA
                        if (em_ble_ral_info_peer_irk_valid_getf(ral_idx) && em_ble_ral_info_peer_rpa_valid_getf(ral_idx)
                                && !co_bdaddr_compare(&data.inita, &init_id_addr))
                        {
                            // Indicate initiator address is RPA
                            init_addr_type |= ADDR_RPA_MASK;

                            // Get initiator's RPA from received packet
                            init_rpa = data.inita;
                        }
                        else
                        {
                            // Assert if privacy error filtering is not set and peer IRK is valid (network privacy mode)
                            ASSERT_ERR(!em_ble_ral_info_peer_irk_valid_getf(ral_idx) || em_ble_ral_info_pef_getf(ral_idx));
                            memset(&init_rpa, 0, sizeof(struct bd_addr));
                        }
                    }
                    else
                    {
                        // gets the bd_addr type
                        init_addr_type = em_ble_rxphadv_rxtxadd_getf(rxdesc_idx);
                        // Get initiator's address
                        init_id_addr = data.inita;
                        memset(&init_rpa, 0, sizeof(struct bd_addr));
                    }

                    // Check if local RPA is used
                    if (em_ble_filtpol_ralcntl_local_rpa_sel_getf(EM_BLE_CS_ACT_ID_TO_INDEX(act_id)))
                    {
                        // Retrieve RAL pointer from CS
                        uint8_t ral_idx = ((em_ble_peer_ralptr_getf(EM_BLE_CS_ACT_ID_TO_INDEX(act_id)) << 2) - REG_EM_BLE_RAL_ADDR_GET(0)) / REG_EM_BLE_RAL_SIZE;
                        ASSERT_ERR(ral_idx < BLE_RAL_MAX);
                        ASSERT_ERR(em_ble_ral_info_local_irk_valid_getf(ral_idx));
                        ASSERT_ERR(em_ble_ral_info_local_rpa_valid_getf(ral_idx));

                        // Copy local resolvable address from RAL
                        em_rd(&adv_rpa, REG_EM_BLE_RAL_ADDR_GET(ral_idx) + EM_BLE_RAL_LOCAL_RPA_INDEX * 2, BD_ADDR_LEN);
                    }
                    else
                    {
                        memset(&adv_rpa, 0, sizeof(struct bd_addr));
                    }

                    // inform upper layer that connection request has been received and provide new connection information
                    {
                        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];
                        struct lld_adv_end_ind *ind = KE_MSG_ALLOC(LLD_ADV_END_IND, TASK_LLM, TASK_NONE, lld_adv_end_ind);

                        uint8_t rate = em_ble_rxchass_rate_getf(rxdesc_idx);

                        // Compute distance to the start of the transmit window
                        // transmitWindowDelay shall be 1.25 ms when a CONNECT_IND PDU is used, 2.5 ms when an AUX_CONNECT_REQ PDU is used on an LE Uncoded PHY, and 3.75 ms when an AUX_CONNECT_REQ PDU is used on the LE Coded PHY
                        uint16_t winoffset = data.lldata.winoffset + 1 + !(adv_par->properties & ADV_LEGACY) + (rate >= CO_RATE_125KBPS);

                        // Compute the start of the Central's transmit window
                        uint32_t first_anchor_hs = CLK_ADD_2((em_ble_rxclknsync1_clknrxsync1_getf(rxdesc_idx) << 16) | em_ble_rxclknsync0_clknrxsync0_getf(rxdesc_idx), 4 * winoffset);
                        uint16_t first_anchor_hus = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(rxdesc_idx) + 2 * (con_req_dur_tab[rate] - lld_exp_sync_pos_tab[rate]);

                        if (first_anchor_hus >= HALF_SLOT_SIZE)
                        {
                            uint8_t num_hs = first_anchor_hus / HALF_SLOT_SIZE;
                            first_anchor_hus -= num_hs * HALF_SLOT_SIZE;
                            first_anchor_hs = CLK_ADD_2(first_anchor_hs, num_hs);
                        }

                        TRC_REQ_ADV_RX_PDU(rxdesc_idx, sizeof(data), &data);

                        // Report advertising end to LLM
                        ind->act_id = act_id;
                        ind->rate = rate;
                        ind->status = CO_ERROR_NO_ERROR;
                        ind->connected = true;
                        ind->ch_sel_2 = (adv_par->properties & ADV_LEGACY) ? em_ble_rxphadv_rxchsel2_getf(rxdesc_idx) : true;
                        ind->nb_ext_adv_evts = (adv_par->max_ext_adv_evt != 0) ? adv_par->ext_adv_evt_cnt : 0;
                        ind->interval = data.lldata.interval;
                        ind->latency = data.lldata.latency;
                        ind->timeout = data.lldata.timeout;
                        ind->ch_map = data.lldata.ch_map;
                        ind->master_sca = (data.lldata.hop_sca >> 5) & 0x07;
                        ind->local_rpa = adv_rpa;
                        ind->peer_rpa = init_rpa;
                        ind->peer_id_addr = init_id_addr;
                        ind->peer_addr_type = init_addr_type;
                        ke_msg_send(ind);

                        // Start connection
                        {
                            struct lld_con_params params;

                            params.aa               = data.lldata.aa;
                            params.crcinit          = data.lldata.crcinit;
                            params.winsize          = data.lldata.winsize;
                            params.winoffset        = winoffset;
                            params.interval         = data.lldata.interval;
                            params.latency          = data.lldata.latency;
                            params.timeout          = data.lldata.timeout;
                            params.ch_map           = data.lldata.ch_map;
                            params.hop_inc          = data.lldata.hop_sca & 0x1F;
                            params.master_sca       = (data.lldata.hop_sca >> 5) & 0x07;
                            params.first_anchor_hs  = first_anchor_hs;
                            params.first_anchor_hus = first_anchor_hus;
                            params.role             = ROLE_SLAVE;
                            params.rate             = rate;
                            params.ch_sel_2         = (adv_par->properties & ADV_LEGACY) ? em_ble_rxphadv_rxchsel2_getf(rxdesc_idx) : true;

                            lld_con_start(adv_par->con_act_id, &params);
                        }
                    }
                }
            }
            else
#endif // (BLE_PERIPHERAL)
                if (lld_adv_env[act_id]->scan_req_notif_en && (rxtype == BLE_SCAN_REQ))
                {
                    struct scan_req data;
                    struct bd_addr scanner_id_addr;
                    uint8_t scanner_addr_type;
                    //uint16_t out_len = PDU_SCAN_REQ_LEN;
                    uint16_t ral_ptr;

                    //co_util_unpack((uint8_t*) data, (uint8_t*) (uint32_t) em_ble_rxdataptr_get(rxdesc_idx), &out_len, PDU_SCAN_REQ_LEN, "6B6B");
                    em_rd((void *)&data, em_ble_rxdataptr_get(rxdesc_idx), PDU_SCAN_REQ_LEN);

                    // retrieve RAL pointer
                    ral_ptr = em_ble_rxralptr_getf(rxdesc_idx);
                    // retrieve peer device identity
                    if (ral_ptr != 0)
                    {
                        uint8_t ral_idx = (ral_ptr - REG_EM_BLE_RAL_ADDR_GET(0)) / REG_EM_BLE_RAL_SIZE;
                        // Do a burst read in the exchange memory
                        em_rd(&scanner_id_addr, ral_ptr + EM_BLE_RAL_PEER_ID_INDEX * 2,  BD_ADDR_LEN);
                        // gets peer address type
                        scanner_addr_type = em_ble_ral_info_peer_id_type_getf(ral_idx) | (em_ble_ral_info_peer_irk_valid_getf(ral_idx) << 1);
                    }
                    else
                    {
                        // gets the bd_addr type
                        scanner_addr_type = em_ble_rxphadv_rxtxadd_getf(rxdesc_idx);
                        // Get scanner's address
                        scanner_id_addr = data.scana;
                    }

                    TRC_REQ_ADV_RX_PDU(rxdesc_idx, sizeof(data), (void *) &data);

                    // inform upper layer that scan request has been received
                    {
                        struct lld_scan_req_ind *ind = KE_MSG_ALLOC(LLD_SCAN_REQ_IND, TASK_LLM, TASK_NONE, lld_scan_req_ind);
                        ind->act_id = act_id;
                        ind->addr_type = scanner_addr_type;
                        ind->addr = scanner_id_addr;
                        ke_msg_send(ind);
                    }
                }
        }

        // Free RX descriptor
        lld_rxdesc_free();
    }

    return (con_req_received);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_adv_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_EVT_START_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LEADV, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_adv_env_tag *adv_par = (struct lld_adv_env_tag *) evt;
        uint16_t adj_offset_us = 0; // Skip adjustment offset in us
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(adv_par->act_id);
        uint8_t ae_nps = 0;

        if ((!(adv_par->properties & (ADV_LEGACY | ADV_CON | ADV_SCAN))) && (adv_par->per_adv_intv != 0) && (!adv_par->sync_update) && (!adv_par->skip_en))
        {
            // Set SyncInfo field in AUX_ADV_IND
            lld_adv_sync_info_set(adv_par->act_id);
        }

        if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
        {
            // MAXEVTIME is still expressed in slots (units of 625 us)
            em_ble_maxevtime_set(cs_idx, CLK_SUB(adv_par->timeout_ts, evt->time.hs) / 2);
        }

        // Update the AuxPtr field to point to the next AUX_ADV_IND PDU if skip is enabled
        if (adv_par->skip_en)
        {
            uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, 0);

            uint32_t diff_hs  = CLK_SUB(adv_par->aux_evt.time.hs, evt->time.hs);
            uint32_t diff_us = ((diff_hs * HALF_SLOT_SIZE) >> 1) + adv_par->prim_aux_offset * 30;
            uint16_t aux_offset = 0;
            bool offset_units = 0;

            if (diff_us <= OFFSET_LIMIT)
            {
                aux_offset = (diff_us < OFFSET_THRESHOLD) ? (diff_us / 30) : (diff_us / 300);
                offset_units = (diff_us < OFFSET_THRESHOLD) ? 0 : 1;
                adj_offset_us = (diff_us < OFFSET_THRESHOLD) ? CO_MOD(diff_us, 30) : CO_MOD(diff_us, 300);
            }

            em_ble_txauxptr0_pack(txdesc_idx, /*txauxoffsetlsb*/ aux_offset, /*txauxoffsetunit*/ offset_units, /*txauxca*/ (rwip_max_drift_get() <= 50), /*txllch*/ adv_par->aux_ch_idx);
            em_ble_txauxptr1_pack(txdesc_idx, /*txauxphy*/ adv_par->lld_sec_adv_rate, /*txauxoffsetmsb*/(aux_offset >> 8));
        }

#if (EAVESDROPPING_SUPPORT)
        if (!(adv_par->properties & ADV_LEGACY) && adv_par->cust_adv_params.en)
        {
            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(adv_par->act_id);
            uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, adv_par->curr_txdesc_base_index);
            uint16_t aux_offset = ((adv_par->curr_txdesc_base_index == 0) && !adv_par->cust_adv_params.group_ext_adv_ind) ? ((adv_par->prim_adv_ch_nb - adv_par->prim_adv_pkt_tx) * adv_par->cust_adv_params.periodicity * SLOT_SIZE) / 30 : (adv_par->cust_adv_params.periodicity * SLOT_SIZE) / 30;

            if (adv_par->curr_txdesc_base_index != 0)
            {
                em_ble_auxtxdescptr_setf(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, txdesc_idx));
                ae_nps = 1;
            }
            else // adv_par->curr_txdesc_base_index == 0
            {
                em_ble_acltxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, txdesc_idx));
            }

            // Set the aux_offset
            em_ble_txauxptr0_txauxoffset_lsb_setf(txdesc_idx, aux_offset);
            em_ble_txauxptr1_txauxoffset_msb_setf(txdesc_idx, (aux_offset >> 8));

            // Disable automatic auxptr management
            ble_advtim_tx_auxptr_thr_setf(0);
        }
#endif // EAVESDROPPING_SUPPORT
        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk        = &lld_adv_frm_cbk;
            prog_par.time.hs        = evt->time.hs;
#if (EAVESDROPPING_SUPPORT)
            if (adv_par->manual_adv_params.en || adv_par->cust_adv_params.en)
                prog_par.time.hus = evt->time.hus;
            else
#endif // EAVESDROPPING_SUPPORT
                prog_par.time.hus       = adj_offset_us << 1;
            prog_par.cs_idx         = cs_idx;
            prog_par.dummy          = cs_idx;
            prog_par.bandwidth      = evt->duration_min;
            prog_par.prio_1         = evt->current_prio;
            prog_par.prio_2         = 0;
            prog_par.prio_3         = rwip_priority[RWIP_PRIO_ADV_AUX_IDX].value;
            prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
            prog_par.add.ble.ae_nps = ae_nps;
            prog_par.add.ble.iso    = 0;
            prog_par.mode           = SCH_PROG_BLE;
            sch_prog_push(&prog_par);
        }

        // Move state
        adv_par->state = ADV_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEADV, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification (AUX event)
 ****************************************************************************************
 */
__STATIC void lld_adv_aux_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_AUX_EVT_START_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LEADV, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_adv_env_tag *adv_par = CONTAINER_OF(evt, struct lld_adv_env_tag, aux_evt);
        uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, 0);
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(adv_par->act_id);

        if ((!(adv_par->properties & (ADV_LEGACY | ADV_CON | ADV_SCAN))) && (adv_par->per_adv_intv != 0) && (!adv_par->sync_update))
        {
            // Set SyncInfo field in AUX_ADV_IND
            lld_adv_sync_info_set(adv_par->act_id);
        }

        if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
        {
            // MAXEVTIME is still expressed in slots (units of 625 us)
            em_ble_maxevtime_set(cs_idx, CLK_SUB(adv_par->timeout_ts, evt->time.hs) / 2);
        }

        em_ble_txauxptr0_pack(txdesc_idx, /*txauxoffsetlsb*/ adv_par->prim_aux_offset, /*txauxoffsetunit*/ 0, /*txauxca*/ (rwip_max_drift_get() <= 50), /*txllch*/ adv_par->aux_ch_idx);
        em_ble_txauxptr1_pack(txdesc_idx, /*txauxphy*/ adv_par->lld_sec_adv_rate, /*txauxoffsetmsb*/(adv_par->prim_aux_offset >> 8));

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk        = &lld_adv_frm_cbk;
            prog_par.time.hs        = evt->time.hs;
            prog_par.time.hus       = 0;
            prog_par.cs_idx         = cs_idx;
            prog_par.dummy          = cs_idx;
            prog_par.bandwidth      = evt->duration_min;
            prog_par.prio_1         = evt->current_prio;
            prog_par.prio_2         = 0;
            prog_par.prio_3         = evt->current_prio;
            prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
            prog_par.add.ble.ae_nps = 0;
            prog_par.add.ble.iso    = 0;
            prog_par.mode           = SCH_PROG_BLE;
            sch_prog_push(&prog_par);
        }

        // Move state
        adv_par->state = ADV_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEADV, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_adv_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_EVT_CANCELED_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LEADV, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_adv_env_tag *adv_par = (struct lld_adv_env_tag *) evt;

        ASSERT_ERR(adv_par->state == ADV_EVT_WAIT);

#if (EAVESDROPPING_SUPPORT)
        if (adv_par->cust_adv_params.en || adv_par->manual_adv_params.en)
        {
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));
        }
        else
#endif // EAVESDROPPING_SUPPORT
        {
            // Increment priority
            if (adv_par->properties & ADV_DIRECT_HI)
            {
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_HDC_IDX));
            }
            else if (CLK_SUB(evt->time.hs, adv_par->last_prio_upd_ts) >= 2 * adv_par->intv)
            {
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

                // Save the timestamp
                adv_par->last_prio_upd_ts = evt->time.hs;

                // Apply a random advDelay
                if (LLD_ADV_MAX_RAND_CANCEL > 0)
                {
                    uint16_t adv_delay_hs = 2 * CO_MOD(co_rand_word(), LLD_ADV_MAX_RAND_CANCEL);

                    if (adv_par->skip_en)
                    {
                        // Adjust the advDelay for the skip adjustment offset
                        if (adv_delay_hs == 0)
                            adv_delay_hs++;
                        else if (adv_delay_hs == 2 * (LLD_ADV_MAX_RAND_CANCEL - 1))
                            adv_delay_hs--;
                    }

                    evt->time.hs = CLK_ADD_2(evt->time.hs, adv_delay_hs);
                }
            }
        }

#if (EAVESDROPPING_SUPPORT)
        if (adv_par->manual_adv_params.en)
        {
            // Clear the ADV_PENDING status bit
            ip_deepslcntl_clear_adv_pending_setf(1);

            // Re-enable deep sleep
            rwip_prevent_sleep_clear(RW_BLE_ACTIVE_MODE);

            // Wait for the interrupt
            adv_par->state = ADV_EVT_WAIT;
        }
        else if (adv_par->cust_adv_params.en)
        {
            if (adv_par->properties & ADV_LEGACY)
            {
                lld_adv_custom_sched(adv_par->act_id, false);
            }
            else // Extended advertising
            {
                // Re-enable automatic auxptr management
                ble_advtim_tx_auxptr_thr_setf(LLD_TX_AUX_PTR_THR);

                // Reset the number of primary advertising packets sent
                adv_par->prim_adv_pkt_tx = 0;
                // Select the first Tx descriptor in the chain
                adv_par->curr_txdesc_base_index = 0;
                if (!adv_par->cust_adv_params.group_ext_adv_ind)
                {
                    // Select the first primary advertising channel
                    adv_par->curr_ch_map = 4;
                    lld_adv_ext_custom_next_channel(adv_par->act_id);
                }

                lld_adv_ext_custom_sched(adv_par->act_id, false);
            }
        }
        else
#endif // EAVESDROPPING_SUPPORT
            // Reschedule ASAP
            if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
            {
                if (!adv_par->skip_en)
                {
                    if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
                    {
                        lld_adv_end(adv_par->act_id, true, CO_ERROR_ADV_TO);
                    }
                    else
                    {
                        ASSERT_ERR_FORCE(0);
                    }
                }
            }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEADV, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification (AUX event)
 ****************************************************************************************
 */
__STATIC void lld_adv_aux_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_AUX_EVT_CANCELED_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LEADV, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_adv_env_tag *adv_par = CONTAINER_OF(evt, struct lld_adv_env_tag, aux_evt);

        if (CLK_SUB(evt->time.hs, adv_par->last_prio_upd_aux_ts) >= 2 * adv_par->intv)
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_AUX_IDX));

            // Save the timestamp
            adv_par->last_prio_upd_aux_ts = evt->time.hs;
        }

        // Reschedule ASAP
        if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
        {
            if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
            {
                // Restore the AuxPtr of the primary advertising channel PDUs
                uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, 0);
                em_ble_txauxptr0_pack(txdesc_idx, /*txauxoffsetlsb*/ adv_par->prim_aux_offset, /*txauxoffsetunit*/ 0, /*txauxca*/ (rwip_max_drift_get() <= 50), /*txllch*/ adv_par->aux_ch_idx);
                em_ble_txauxptr1_pack(txdesc_idx, /*txauxphy*/ adv_par->lld_sec_adv_rate, /*txauxoffsetmsb*/(adv_par->prim_aux_offset >> 8));

                adv_par->skip_en = false;
                adv_par->evt.duration_min = evt->duration_min;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEADV, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_adv_frm_isr(uint8_t act_id, uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_FRM_ISR_FUNC_BIT, act_id, timestamp, abort);

    DBG_SWDIAG(LEADV, FRM_ISR, 1);

    if (lld_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_adv_env[act_id]->evt);
        struct sch_arb_elt_tag *aux_evt = &(lld_adv_env[act_id]->aux_evt);
        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

        // Remove event
        if (adv_par->skip_en && (timestamp == aux_evt->time.hs))
            sch_arb_remove(aux_evt, true);
        else
            sch_arb_remove(evt, true);

        // Update extended advertising event count
        adv_par->ext_adv_evt_cnt++;

        // Check advertising end
        if ((adv_par->state == ADV_EVT_END)
                || ((adv_par->max_ext_adv_evt != 0) && (adv_par->ext_adv_evt_cnt >= adv_par->max_ext_adv_evt)))
        {
            uint8_t status;

            // Release any descriptors that might have been used
            while (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(act_id)))
            {
                // Free RX descriptor
                lld_rxdesc_free();
            }

            status = ((adv_par->max_ext_adv_evt != 0) && (adv_par->ext_adv_evt_cnt >= adv_par->max_ext_adv_evt)) ? CO_ERROR_LIMIT_REACHED : CO_ERROR_NO_ERROR;
            lld_adv_end(act_id, true, status);
        }
        else
        {
            bool con_req_received = lld_adv_pkt_rx(adv_par->act_id);

            // Check if CONNECT_REQ has been received
            if (con_req_received)
            {
                lld_adv_end(act_id, false, CO_ERROR_NO_ERROR);
            }
            else
            {
#if (EAVESDROPPING_SUPPORT)
                if (!adv_par->cust_adv_params.en || (adv_par->properties & ADV_LEGACY)
                        || (((adv_par->properties & ADV_CON) && (adv_par->curr_txdesc_base_index == 1))
                            || ((adv_par->properties & ADV_SCAN) && (adv_par->curr_txdesc_base_index == 1))
                            || (adv_par->curr_txdesc_base_index == (adv_par->total_txdesc_count - 1))))
#endif // EAVESDROPPING_SUPPORT
                {
                    // Check if the host has updated the advertising data
                    if (adv_par->data.adv_data_buf != 0)
                    {
                        // Set the advertising data if it is allowed by the advertising properties
                        if ((!(adv_par->properties & ADV_LEGACY))
                                || ((!(adv_par->properties & ADV_DIRECT)) && (adv_par->properties & ADV_LEGACY)))
                        {
                            lld_adv_adv_data_set(act_id, adv_par->data.adv_data_len, adv_par->data.adv_data_buf, true);
                            adv_par->data.adv_data_buf = 0;
                        }
                    }

                    // Check if the host has updated the scan response data
                    if (adv_par->data.scan_rsp_data_buf != 0)
                    {
                        // Set the scan response data if it is allowed by the advertising properties
                        if (((!(adv_par->properties & ADV_LEGACY)) && (adv_par->properties & ADV_SCAN))
                                || ((!(adv_par->properties & ADV_DIRECT)) && (adv_par->properties & ADV_LEGACY)))
                        {
                            lld_adv_scan_rsp_data_set(act_id, adv_par->data.scan_rsp_data_len, adv_par->data.scan_rsp_data_buf, true);
                            adv_par->data.scan_rsp_data_buf = 0;
                        }
                    }

                    // Check if sync info has to be added
                    if ((!(adv_par->properties & (ADV_LEGACY | ADV_CON | ADV_SCAN))) && (adv_par->sync_update))
                    {
                        lld_adv_sync_info_prepare(adv_par->act_id);
                        adv_par->sync_update = false;
                    }
                }

                if (adv_par->properties & ADV_DIRECT_HI)
                {
                    // High duty cycle advertising, try to reschedule before timeout
                    evt->current_prio = abort
                                        ? RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_HDC_IDX))
                                        : rwip_priority[RWIP_PRIO_ADV_HDC_IDX].value;

                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_HDC_IDX));
                    evt->asap_limit = adv_par->timeout_ts;

                    // Start event as soon as possible with a limit of event duration
                    evt->time.hs = timestamp;
                    // Try to schedule before the high duty cycle timeout timestamp
                    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    {
                        adv_par->state = ADV_EVT_WAIT;
                    }
                    else
                    {
                        lld_adv_end(act_id, true, CO_ERROR_ADV_TO);
                    }
                }
                else
                {
                    // The auxiliary channel index needs to be updated only after the AUX_ADV_IND PDU has been sent
                    if ((!(adv_par->properties & ADV_LEGACY))
                            && ((adv_par->skip_en && (aux_evt->time.hs == timestamp)) || (!adv_par->skip_en)))
                    {
#if (EAVESDROPPING_SUPPORT)
                        if (adv_par->cust_adv_params.en)
                        {
                            // Do nothing, the auxiliary channel index is set elsewhere
                        }
                        else
#endif // EAVESDROPPING_SUPPORT
                        {
                            // Set auxiliary channel index in packets and CS
                            lld_adv_aux_ch_idx_set(act_id);
                        }
                    }

                    // Low duty cycle advertising, reschedule
                    if (adv_par->skip_en && (aux_evt->time.hs == timestamp))
                    {
                        if (!abort)
                        {
                            aux_evt->current_prio = rwip_priority[RWIP_PRIO_ADV_AUX_IDX].value;
                        }
                        else if (CLK_SUB(timestamp, adv_par->last_prio_upd_aux_ts) >= 2 * adv_par->intv)
                        {
                            aux_evt->current_prio = RWIP_PRIO_ADD_2(aux_evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_AUX_IDX));
                        }

                        // Save the timestamp
                        adv_par->last_prio_upd_aux_ts = timestamp;
                    }
                    else
                    {
                        if (!abort)
                        {
                            evt->current_prio = rwip_priority[RWIP_PRIO_ADV_IDX].value;
                        }
#if (EAVESDROPPING_SUPPORT)
                        else if (adv_par->cust_adv_params.en || adv_par->manual_adv_params.en)
                        {
                            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));
                        }
#endif // EAVESDROPPING_SUPPORT
                        else if (CLK_SUB(timestamp, adv_par->last_prio_upd_ts) >= 2 * adv_par->intv)
                        {
                            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));
                        }

                        // Save the timestamp
                        adv_par->last_prio_upd_ts = timestamp;
                    }

                    if ((adv_par->skip_en) && (timestamp == evt->time.hs))
                    {
                        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));
                        evt->asap_limit = CLK_SUB(aux_evt->time.hs, 2 * adv_par->intv);
                    }
                    else if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
                    {
                        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));
                        evt->asap_limit = adv_par->timeout_ts;
                    }
                    else
                    {
                        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));
                    }

#if (EAVESDROPPING_SUPPORT)
                    if (adv_par->manual_adv_params.en)
                    {
                        // Clear the ADV_PENDING status bit
                        ip_deepslcntl_clear_adv_pending_setf(1);

                        // Re-enable deep sleep
                        rwip_prevent_sleep_clear(RW_BLE_ACTIVE_MODE);

                        // Wait for the interrupt
                        adv_par->state = ADV_EVT_WAIT;
                    }
                    else if (adv_par->cust_adv_params.en)
                    {
                        if (adv_par->properties & ADV_LEGACY)
                        {
                            bool new_evt = false;

                            if (!abort)
                            {
                                new_evt = lld_adv_custom_next_channel(act_id);
                            }

                            lld_adv_custom_sched(act_id, new_evt);
                        }
                        else // Extended advertising
                        {
                            bool new_evt = false;

                            // Re-enable automatic auxptr management
                            ble_advtim_tx_auxptr_thr_setf(LLD_TX_AUX_PTR_THR);

                            if (abort)
                            {
                                // Reset the number of primary advertising packets sent
                                adv_par->prim_adv_pkt_tx = 0;
                                // Select the first Tx descriptor in the chain
                                adv_par->curr_txdesc_base_index = 0;
                                if (!adv_par->cust_adv_params.group_ext_adv_ind)
                                {
                                    // Select the first primary advertising channel
                                    adv_par->curr_ch_map = 4;
                                    lld_adv_ext_custom_next_channel(act_id);
                                }
                            }
                            else // !abort
                            {
                                if (adv_par->curr_txdesc_base_index == 0)
                                {
                                    if (!adv_par->cust_adv_params.group_ext_adv_ind)
                                    {
                                        // Increment the number of primary advertising packets sent
                                        adv_par->prim_adv_pkt_tx++;
                                    }
                                    else
                                    {
                                        adv_par->prim_adv_pkt_tx = adv_par->prim_adv_ch_nb;
                                    }
                                    if (adv_par->prim_adv_pkt_tx < adv_par->prim_adv_ch_nb)
                                    {
                                        lld_adv_ext_custom_next_channel(act_id);
                                    }
                                    else // All primary advertising packets have been transmitted
                                    {
                                        // Move to the next Tx descriptor
                                        adv_par->curr_txdesc_base_index++;
                                    }
                                }
                                else
                                {
                                    // Move to the next Tx descriptor
                                    adv_par->curr_txdesc_base_index++;
                                }

                                // The AUX_CONNECT_RSP/AUX_SCAN_RSP PDU is automatically sent by the HW if needed, so restart after the AUX_ADV_IND has been transmitted
                                if (((adv_par->properties & ADV_CON) && (adv_par->curr_txdesc_base_index == 2))
                                        || ((adv_par->properties & ADV_SCAN) && (adv_par->curr_txdesc_base_index == 2)))
                                {
                                    adv_par->curr_txdesc_base_index = adv_par->total_txdesc_count;
                                }

                                if (adv_par->curr_txdesc_base_index == adv_par->total_txdesc_count)
                                {
                                    // Reset the number of primary advertising packets sent
                                    adv_par->prim_adv_pkt_tx = 0;
                                    // Select the first Tx descriptor in the chain
                                    adv_par->curr_txdesc_base_index = 0;
                                    if (!adv_par->cust_adv_params.group_ext_adv_ind)
                                    {
                                        // Select the first primary advertising channel
                                        adv_par->curr_ch_map = 4;
                                        lld_adv_ext_custom_next_channel(act_id);
                                    }
                                    new_evt = true;
                                }
                            }

                            // Make sure that the next Tx descriptor is ready
#if (PLF_DEBUG)
                            {
                                uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, adv_par->curr_txdesc_base_index);
                                ASSERT_INFO((em_ble_txcntl_txdone_getf(txdesc_idx) == 0), adv_par->curr_txdesc_base_index, txdesc_idx);
                            }
#endif //(PLF_DEBUG)

                            lld_adv_ext_custom_sched(act_id, new_evt);
                        }
                    }
                    else
#endif // EAVESDROPPING_SUPPORT
                    {
                        // Apply a random advDelay
                        {
                            uint16_t adv_delay_hs = 2 * CO_MOD(co_rand_word(), LLD_ADV_MAX_RAND);

                            if (adv_par->skip_en)
                            {
                                // Adjust the advDelay for the skip adjustment offset
                                if (adv_delay_hs == 0)
                                    adv_delay_hs++;
                                else if (adv_delay_hs == 2 * (LLD_ADV_MAX_RAND - 1))
                                    adv_delay_hs--;
                            }

                            evt->time.hs = CLK_ADD_3(timestamp, 2 * adv_par->intv, adv_delay_hs);
                        }

                        // Try to reschedule
                        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                        {
                            adv_par->state = ADV_EVT_WAIT;

                            // Insert the aux event if needed
                            if (adv_par->skip_en && (aux_evt->time.hs == timestamp))
                            {
                                // Choose the timestamp of the next aux event taking the max_skip parameter into account
                                uint32_t next_aux_evt_ts = CLK_ADD_2(evt->time.hs, 2 * adv_par->sec_adv_max_skip * adv_par->intv);

                                // Check if the timestamp needs to be adjusted in case there is a timeout
                                if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
                                {
                                    SCH_ARB_ASAP_STG_SET(aux_evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_AUX_IDX));
                                    aux_evt->asap_limit = adv_par->timeout_ts;
                                    if (CLK_LOWER_EQ(adv_par->timeout_ts, next_aux_evt_ts))
                                    {
                                        next_aux_evt_ts = adv_par->timeout_ts;
                                    }
                                }
                                else
                                {
                                    SCH_ARB_ASAP_STG_SET(aux_evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_AUX_IDX));
                                }

                                aux_evt->time.hs = next_aux_evt_ts;
                                if (sch_arb_insert(aux_evt) != SCH_ARB_ERROR_OK)
                                {
                                    adv_par->skip_en = false;
                                    evt->duration_min = aux_evt->duration_min;
                                }
                            }
                        }
                        else
                        {
                            if (adv_par->skip_en)
                            {
                                // Failed to schedule, so make sure that evt->time.hs is different from aux_evt->time.hs
                                evt->time.hs = LLD_CLOCK_UNDEF;

                                // As end of event at frm_isr, transition back to EVT_WAIT.
                                adv_par->state = ADV_EVT_WAIT;
                            }

                            if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
                            {
                                if (evt->asap_limit == adv_par->timeout_ts)
                                {
                                    lld_adv_end(act_id, true, CO_ERROR_ADV_TO);
                                }
                            }
                            else if (!adv_par->skip_en)
                            {
                                ASSERT_ERR_FORCE(0);
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEADV, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle skip interrupt
 ****************************************************************************************
 */
__STATIC void lld_adv_frm_skip_isr(uint8_t act_id, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_FRM_SKIP_ISR_FUNC_BIT, act_id, timestamp);

    DBG_SWDIAG(LEADV, EVT_CANCELED, 1);

    if (lld_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_adv_env[act_id]->evt);
        struct sch_arb_elt_tag *aux_evt = &(lld_adv_env[act_id]->aux_evt);
        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

        ASSERT_ERR((adv_par->state == ADV_EVT_ACTIVE) || (adv_par->state == ADV_EVT_END));

        // Check advertising end
        if (adv_par->state == ADV_EVT_END)
        {
            lld_adv_end(act_id, true, CO_ERROR_NO_ERROR);
        }
        else if (adv_par->skip_en && (timestamp == aux_evt->time.hs))
        {
            sch_arb_remove(aux_evt, true);

            if (CLK_SUB(timestamp, adv_par->last_prio_upd_aux_ts) >= 2 * adv_par->intv)
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_AUX_IDX));

                // Save the timestamp
                adv_par->last_prio_upd_aux_ts = timestamp;
            }

            // Reschedule ASAP
            if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
            {
                if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
                {
                    lld_adv_end(adv_par->act_id, true, CO_ERROR_ADV_TO);
                }
                else
                {
                    ASSERT_ERR_FORCE(0);
                }
            }
            else
            {
                adv_par->state = ADV_EVT_WAIT;
            }
        }
        else
        {
            sch_arb_remove(evt, true);

#if (EAVESDROPPING_SUPPORT)
            if (adv_par->cust_adv_params.en || adv_par->manual_adv_params.en)
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));
            }
            else
#endif // EAVESDROPPING_SUPPORT
            {
                // Increment priority
                if (adv_par->properties & ADV_DIRECT_HI)
                    evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_HDC_IDX));
                else if (CLK_SUB(timestamp, adv_par->last_prio_upd_ts) >= 2 * adv_par->intv)
                {
                    evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

                    // Save the timestamp
                    adv_par->last_prio_upd_ts = evt->time.hs;
                }
            }

#if (EAVESDROPPING_SUPPORT)
            if (adv_par->manual_adv_params.en)
            {
                // Clear the ADV_PENDING status bit
                ip_deepslcntl_clear_adv_pending_setf(1);

                // Re-enable deep sleep
                rwip_prevent_sleep_clear(RW_BLE_ACTIVE_MODE);

                // Wait for the interrupt
                adv_par->state = ADV_EVT_WAIT;
            }
            else if (adv_par->cust_adv_params.en)
            {
                if (adv_par->properties & ADV_LEGACY)
                {
                    lld_adv_custom_sched(adv_par->act_id, false);
                }
                else // Extended advertising
                {
                    // Re-enable automatic auxptr management
                    ble_advtim_tx_auxptr_thr_setf(LLD_TX_AUX_PTR_THR);

                    // Reset the number of primary advertising packets sent
                    adv_par->prim_adv_pkt_tx = 0;
                    // Select the first Tx descriptor in the chain
                    adv_par->curr_txdesc_base_index = 0;
                    if (!adv_par->cust_adv_params.group_ext_adv_ind)
                    {
                        // Select the first primary advertising channel
                        adv_par->curr_ch_map = 4;
                        lld_adv_ext_custom_next_channel(adv_par->act_id);
                    }

                    lld_adv_ext_custom_sched(adv_par->act_id, false);
                }
            }
            else
#endif // EAVESDROPPING_SUPPORT
                // Reschedule ASAP
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    if (!adv_par->skip_en)
                    {
                        if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
                        {
                            lld_adv_end(adv_par->act_id, true, CO_ERROR_ADV_TO);
                        }
                        else
                        {
                            ASSERT_ERR_FORCE(0);
                        }
                    }
                }
                else
                {
                    adv_par->state = ADV_EVT_WAIT;
                }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEADV, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_adv_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_FRM_CBK_FUNC_BIT, timestamp, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO: // Post-bandwidth contested as low priority (prio_2 = 0)
    {
        lld_adv_frm_isr(EM_BLE_CS_IDX_TO_ACT_ID(dummy), timestamp, false);
    }
    break;
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    {
        lld_adv_frm_isr(EM_BLE_CS_IDX_TO_ACT_ID(dummy), timestamp, true);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        // No need to act upon the Rx interrupt for advertising
    } break;
    case SCH_FRAME_IRQ_SKIP:
    {
        lld_adv_frm_skip_isr(EM_BLE_CS_IDX_TO_ACT_ID(dummy), timestamp);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lld_adv_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_INIT_FUNC_BIT, is_reset);

    if (is_reset)
    {
        for (int8_t act_id = (BLE_ACTIVITY_MAX - 1); act_id >= 0 ; act_id--)
        {
            // Check if advertising is ongoing
            if (lld_adv_env[act_id] != NULL)
            {
                // Free event memory
                ke_free(lld_adv_env[act_id]);
            }
        }
    }
    // No break

    // Initialize environment
    memset(&lld_adv_env, 0, sizeof(lld_adv_env));
}

uint8_t lld_adv_start(uint8_t act_id, struct lld_adv_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_START_FUNC_BIT, uint8_t, act_id, params);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    uint8_t rf_mod;
    uint8_t cs_val;

    DBG_SWDIAG(LEADV, START, 1);

    // Check if advertising is inactive
    if (lld_adv_env[act_id] == NULL)
    {
        // Allocate event
        lld_adv_env[act_id] = LLD_ALLOC_ENV(lld_adv_env_tag);

        if (lld_adv_env[act_id] != NULL)
        {
            // Point to parameters
            struct sch_arb_elt_tag *evt     = &(lld_adv_env[act_id]->evt);
            struct sch_arb_elt_tag *aux_evt = &(lld_adv_env[act_id]->aux_evt);
            struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

            uint32_t clock  = lld_read_clock();
            uint8_t  cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

            LLD_INIT_ENV(adv_par, lld_adv_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel           = &lld_adv_evt_canceled_cbk;
            evt->cb_start            = &lld_adv_evt_start_cbk;
            evt->current_prio        = rwip_priority[RWIP_PRIO_ADV_IDX].value;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

            // Initialize event parameters (advertising part)
            adv_par->act_id            = act_id;
            adv_par->con_act_id        = params->con_act_id;
            adv_par->intv              = params->prim_adv_intv;
            adv_par->properties        = params->adv_evt_properties;
            adv_par->scan_req_notif_en = params->scan_req_notif_en;
            adv_par->peer_addr         = params->peer_addr;
            adv_par->own_addr_type     = params->own_addr_type;
            adv_par->peer_addr_type    = params->peer_addr_type;
            adv_par->adv_tx_pwr        = params->adv_tx_pwr;
            adv_par->sec_adv_max_skip  = params->sec_adv_max_skip;
            adv_par->lld_prim_adv_rate = ((params->prim_adv_phy > 2) && (lld_env.le_coded_phy_500)) ? params->prim_adv_phy : params->prim_adv_phy - 1;
            adv_par->lld_sec_adv_rate  = ((params->sec_adv_phy > 2) && (lld_env.le_coded_phy_500)) ? params->sec_adv_phy : params->sec_adv_phy - 1;
            adv_par->prim_adv_ch_nb    = NB_ONE_BITS(params->prim_adv_ch_map);
#if (EAVESDROPPING_SUPPORT)
            adv_par->prim_adv_ch_map = params->prim_adv_ch_map;
            adv_par->curr_ch_map = 1;

            // Copy the manual adv parameters
            memcpy(&adv_par->manual_adv_params, &params->manual_adv_params, sizeof(struct lld_manual_adv_params));

            if (adv_par->manual_adv_params.en)
            {
                // Save the ID of the advertising set that uses manual trigger
                lld_adv_trig_id = act_id;
            }
            else
            {
                // Copy the custom adv parameters
                memcpy(&adv_par->cust_adv_params, &params->cust_adv_params, sizeof(struct lld_cust_adv_params));

                if (adv_par->cust_adv_params.en)
                {
                    ASSERT_ERR(!(adv_par->properties & ADV_DIRECT_HI));

                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

                    if (adv_par->properties & ADV_LEGACY)
                    {
                        adv_par->cust_adv_params.group_ext_adv_ind = false;
                    }
                }
            }
#endif // EAVESDROPPING_SUPPORT
            // Set the ADI field (SID and DID)
            {
                uint16_t adi_did = co_rand_hword() & BLE_ADI_DID_MASK;

                SETF(adv_par->adi, BLE_ADI_DID, adi_did);
                SETF(adv_par->adi, BLE_ADI_SID, params->adv_sid);
            }
            adv_par->max_ext_adv_evt = params->max_ext_adv_evt;
            adv_par->per_act_id      = 0xFF;

            adv_par->curr_adv_data_len      = params->init_adv_data_len;
            adv_par->curr_adv_data_buf      = params->init_adv_data_buf;
            adv_par->curr_scan_rsp_data_len = params->init_scan_rsp_data_len;
            adv_par->curr_scan_rsp_data_buf = params->init_scan_rsp_data_buf;
#if (BT_53)
            adv_par->chg_reasons = params->chg_reasons;
#endif // (BT_53)

            // Pre-initialization of conditionally set fields
            em_ble_chmap2_set(cs_idx, 0);
            em_ble_thrcntl_ratecntl_set(cs_idx, 0);
            em_ble_auxtxdescptr_set(cs_idx, 0);

            if (!(adv_par->properties & ADV_LEGACY))
            {
                adv_par->skip_en = (adv_par->sec_adv_max_skip > 0);

#if (EAVESDROPPING_SUPPORT)
                // Disable skipping events when performing manual/custom advertising
                if (adv_par->cust_adv_params.en || adv_par->manual_adv_params.en)
                {
                    adv_par->skip_en = false;
                }
#endif // EAVESDROPPING_SUPPORT

                lld_adv_ext_chain_construct(adv_par->act_id);

#if (EAVESDROPPING_SUPPORT)
                // Do not allow chains of scannable data
                if (!(adv_par->properties & ADV_LEGACY) && adv_par->cust_adv_params.en && (adv_par->properties & ADV_SCAN))
                {
                    ASSERT_ERR(adv_par->total_txdesc_count <= 3);
                }
#endif // EAVESDROPPING_SUPPORT

                if (adv_par->skip_en)
                {
                    // Initialize event parameters (common part)
                    aux_evt->cb_cancel           = &lld_adv_aux_evt_canceled_cbk;
                    aux_evt->cb_start            = &lld_adv_aux_evt_start_cbk;
                    aux_evt->current_prio        = rwip_priority[RWIP_PRIO_ADV_AUX_IDX].value;
                    aux_evt->duration_min        = evt->duration_min;
                    SCH_ARB_ASAP_STG_SET(aux_evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_AUX_IDX));
                }
            }
            else
            {
                uint8_t adv_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, 0);
                uint8_t scan_rsp_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, 1);
                uint8_t txtype = BLE_ADV_IND;
                uint16_t pkt_dur_us = PDU_1MBPS_LEN_US(BD_ADDR_LEN + ADV_DATA_LEN);

                // The duration of the event depends on the type of advertising as described in the HW FS
                // T_IFS is normally subtracted at the end because the last advertising packet does not need it
                // High duty cycle advertising is governed by the ADVTIM-ADVINT register
                switch (adv_par->properties)
                {
                case ADV_IND:
                {
                    txtype = BLE_ADV_IND;
                    // prim_adv_ch_nb*(pkt_dur + T_IFS + min_scan_dur + T_IFS) - T_IFS
                    evt->duration_min = adv_par->prim_adv_ch_nb * 2 * (pkt_dur_us + 2 * BLE_IFS_DUR + BLE_ADV_MIN_SCAN_WIN_SIZE_US) - 2 * BLE_IFS_DUR + BLE_RESERVATION_TIME_MARGIN_HUS;
                }
                break;
                case ADV_DIRECT_LO_IND:
                {
                    txtype = BLE_ADV_DIRECT_IND;
                    pkt_dur_us = PDU_1MBPS_LEN_US(2 * BD_ADDR_LEN);
                    // prim_adv_ch_nb*(pkt_dur + T_IFS + min_scan_dur + T_IFS) - T_IFS
                    evt->duration_min = adv_par->prim_adv_ch_nb * 2 * (pkt_dur_us + 2 * BLE_IFS_DUR + BLE_ADV_MIN_SCAN_WIN_SIZE_US) - 2 * BLE_IFS_DUR + BLE_RESERVATION_TIME_MARGIN_HUS;
                }
                break;
                case ADV_DIRECT_HI_IND:
                {
                    txtype = BLE_ADV_DIRECT_IND;
                    // prim_adv_ch_nb*adv_int
                    evt->duration_min = adv_par->prim_adv_ch_nb * 2 * BLE_ADV_INT_US + BLE_RESERVATION_TIME_MARGIN_HUS;
                }
                break;
                case ADV_SCAN_IND:
                {
                    txtype = BLE_ADV_SCAN_IND;
                    // prim_adv_ch_nb*(pkt_dur + T_IFS + min_scan_dur + T_IFS) - T_IFS
                    evt->duration_min = adv_par->prim_adv_ch_nb * 2 * (pkt_dur_us + 2 * BLE_IFS_DUR + BLE_ADV_MIN_SCAN_WIN_SIZE_US) - 2 * BLE_IFS_DUR + BLE_RESERVATION_TIME_MARGIN_HUS;
                }
                break;
                case ADV_NONCONN_IND:
                {
                    txtype = BLE_ADV_NONCONN_IND;
                    // prim_adv_ch_nb*(pkt_dur + T_IFS) - T_IFS
                    evt->duration_min = adv_par->prim_adv_ch_nb * 2 * (pkt_dur_us + BLE_IFS_DUR) - 2 * BLE_IFS_DUR + BLE_RESERVATION_TIME_MARGIN_HUS;
                }
                break;
                default:
                {
                    ASSERT_ERR_FORCE(0);
                }
                break;
                }

#if (EAVESDROPPING_SUPPORT)
                if (adv_par->cust_adv_params.en)
                {
                    evt->duration_min = 4 * HALF_SLOT_SIZE;
                }
#endif // EAVESDROPPING_SUPPORT

                em_ble_txphadv_pack(adv_txdesc_idx, BD_ADDR_LEN /*txadvlen*/,
                                    (txtype == BLE_ADV_DIRECT_IND) && (params->peer_addr_type & ADDR_RAND) /*txrxadd*/,
                                    params->own_addr_type & ADDR_RAND /*txtxadd*/,
                                    1 /*txchsel2*/,
                                    0 /*txadvrfu*/,
                                    txtype /*txtype*/);

                if ((adv_par->properties & ADV_DIRECT) || (params->init_adv_data_len > 0))
                {
                    if (adv_par->properties & ADV_DIRECT)
                    {
                        // A buffer is needed to store the peer's BD address
                        uint16_t txbuf_offset = EM_BLE_ADVEXTHDRTXBUF_OFF(adv_par->act_id);
                        // Copy address
                        em_wr((void *)&params->peer_addr.addr, txbuf_offset, BD_ADDR_LEN);
                        // Set peer address
                        lld_adv_adv_data_set(act_id, BD_ADDR_LEN, txbuf_offset, false);
                    }
                    else // Advertising data length > 0
                    {
                        // Set advertising data
                        lld_adv_adv_data_set(act_id, params->init_adv_data_len, params->init_adv_data_buf, false);
                    }
                }
                else
                {
                    // Set advertising data Tx descriptor fields
                    em_ble_txdataptr_setf(adv_txdesc_idx, 0);
                }

                em_ble_txphadv_pack(scan_rsp_txdesc_idx, 0 /*txadvlen*/,
                                    0 /*txrxadd*/,
                                    params->own_addr_type & ADDR_RAND /*txtxadd*/,
                                    0 /*txchsel2*/,
                                    0 /*txadvrfu*/,
                                    BLE_SCAN_RSP /*txtype*/);

                if (!(adv_par->properties & ADV_DIRECT))
                {
                    if (params->init_scan_rsp_data_len > 0)
                    {
                        // Set scan response data
                        lld_adv_scan_rsp_data_set(act_id, params->init_scan_rsp_data_len, params->init_scan_rsp_data_buf, false);
                    }
                    else
                    {
                        // Set scan response data Tx descriptor fields
                        em_ble_txphadv_txadvlen_setf(scan_rsp_txdesc_idx, (BD_ADDR_LEN + params->init_scan_rsp_data_len));
                        em_ble_txdataptr_setf(scan_rsp_txdesc_idx, 0);

                        // Set scan response data descriptor next pointer
                        em_ble_txcntl_nextptr_setf(scan_rsp_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, adv_txdesc_idx));
                    }
                }

                if ((adv_par->properties & ADV_DIRECT) || (!(adv_par->properties & ADV_SCAN)))
                {
                    // Set advertising data descriptor next pointer to 0
                    em_ble_txcntl_nextptr_setf(adv_txdesc_idx, 0);
                }
                else
                {
                    // Set advertising data descriptor next pointer
                    em_ble_txcntl_nextptr_setf(adv_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, scan_rsp_txdesc_idx));
                    // Release descriptor
                    em_ble_txcntl_txdone_setf(scan_rsp_txdesc_idx, 0);
                }

                // Release descriptor
                em_ble_txcntl_txdone_setf(adv_txdesc_idx, 0);
            }

            // Set control structure fields
            if (adv_par->properties & ADV_DIRECT_HI)
            {
                // Register high duty cycle advertising
                sch_slice_bg_add(BLE_HDC_ADV);
                // Update event duration
                evt->duration_min = sch_slice_params.scan_evt_dur;

                // High duty cycle advertiser
                em_ble_cntl_pack(cs_idx,
                                 RWIP_COEX_GET(ADV, TXBSY),
                                 RWIP_COEX_GET(ADV, RXBSY),
                                 RWIP_COEX_GET(ADV, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                                 0,
#endif // EAVESDROPPING_SUPPORT
                                 EM_BLE_CS_FMT_HDC_ADV);
            }
            else
            {
                if (adv_par->properties & ADV_LEGACY)
                {
                    // Low duty cycle advertiser
                    em_ble_cntl_pack(cs_idx,
                                     RWIP_COEX_GET(ADV, TXBSY),
                                     RWIP_COEX_GET(ADV, RXBSY),
                                     RWIP_COEX_GET(ADV, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                                     0,
#endif // EAVESDROPPING_SUPPORT
                                     EM_BLE_CS_FMT_LDC_ADV);
                }
                else
                {
                    // Extended advertiser
                    em_ble_cntl_pack(cs_idx,
                                     RWIP_COEX_GET(ADV, TXBSY),
                                     RWIP_COEX_GET(ADV, RXBSY),
                                     RWIP_COEX_GET(ADV, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                                     0,
#endif // EAVESDROPPING_SUPPORT
                                     EM_BLE_CS_FMT_EXT_ADV);
                }
            }

            // update advertiser filter policy
            {
                uint16_t ral_ptr = 0;
                bool local_rpa_sel = false;
                bool ral_mode = false;
                bool ral_en = false;

                // Check if targeted device is present in RAL
                if (params->ral_idx < BLE_RAL_MAX)
                {
                    // If local RPA is to be used, check if local IRK is valid
                    if (((params->own_addr_type & ADDR_RPA_MASK) != 0) && em_ble_ral_info_local_irk_valid_getf(params->ral_idx))
                    {
                        local_rpa_sel = true;
                        ral_en = true;
                        ral_mode = true;
                        ral_ptr = REG_EM_ADDR_GET(BLE_RAL, params->ral_idx);
                    }
                    // If resolution enabled and adv directed to a peer, check if peer IRK is valid
                    else if ((params->addr_resolution_en && (adv_par->properties & ADV_DIRECT)) && em_ble_ral_info_peer_irk_valid_getf(params->ral_idx))
                    {
                        ral_en = true;
                        ral_mode = true;
                        ral_ptr = REG_EM_ADDR_GET(BLE_RAL, params->ral_idx);
                    }
                    // If resolution enabled and adv undirected
                    else if (params->addr_resolution_en && !(adv_par->properties & ADV_DIRECT))
                    {
                        ral_en = true;
                    }
                }
                // If resolution enabled and adv undirected
                else if (params->addr_resolution_en && !(adv_par->properties & ADV_DIRECT))
                {
                    ral_en = true;
                }

                em_ble_filtpol_ralcntl_pack(cs_idx, /*filterpolicy*/params->adv_filter_policy, /*ralresolen*/ params->addr_resolution_en,
                                            /*peradvfilten*/false,
                                            /*localrpasel*/local_rpa_sel, /*ralmode*/ral_mode, /*ralen*/ral_en);

                // Set peer device configuration
                if (ral_mode)
                {
                    // set ral_ptr
                    em_ble_peer_ralptr_setf(cs_idx, ral_ptr);

                    if (((adv_par->properties & ADV_DIRECT_HI) || (adv_par->properties & ADV_DIRECT)) && em_ble_ral_info_peer_irk_valid_getf(params->ral_idx))
                    {
                        // Ask to the HW to regenerate a peer RPA
                        em_ble_ral_info_peer_rpa_renew_setf(params->ral_idx, 1);
                    }
                }
                else
                {
                    // set peer address
                    // Do a burst write in the exchange memory
                    em_wr(&(params->peer_addr), REG_EM_BLE_CS_ADDR_GET(cs_idx) + EM_BLE_ADV_BD_ADDR_INDEX * 2, sizeof(struct bd_addr));
                    em_ble_adv_bd_addr_type_setf(cs_idx, params->peer_addr_type);
                }
            }

#if (BT_53)
            // If the data changed when the set was disabled
            if (params->adv_rpa_upd)
            {
                // Refresh RPAs
                lld_adv_rpa_refresh(act_id);
            }
#endif // (BT_53)

#if (BLE_ADV_CH_IDX)
            // Select primary advertising channel ascending order mode only when performing high duty cycle advertising
            // In all other cases select primary advertising channel randomized order mode
            if (!(adv_par->properties & ADV_DIRECT_HI))
            {
                em_ble_hopcntl_pack(cs_idx, /*fhen*/true,/*hopmode*/LLD_HOP_MODE_RAND_PRIM_ADV_CH, /*hopint*/ 17, /* chidx */0);
            }
            else
#endif // (BLE_ADV_CH_IDX)
            {
                em_ble_hopcntl_pack(cs_idx, /*fhen*/true,/*hopmode*/LLD_HOP_MODE_CHAN_SEL_1, /*hopint*/ 0, /* chidx */39);
            }

            if (adv_par->adv_tx_pwr != ADV_TX_PWR_NO_PREF)
            {
                // Apply transceiver Tx power level equivalent to radiative Tx power parameter value
                rf_mod = rwip_rf.txpwr_mdlt_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), 0);
                cs_val = rwip_rf.txpwr_cs_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), TXPWR_CS_LOWER, rf_mod);
                em_ble_txrxcntl_set(cs_idx, cs_val);
            }
            else
            {
                // Maximum Tx power
                em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_max);
                rf_mod = rwip_rf.txpwr_max_mod;
                cs_val = rwip_rf.txpwr_max;
            }
            rwip_rf.txpwr_cs_set(TYPE_BLE, cs_idx, rf_mod, cs_val);

            // Set the Tx descriptor pointer in the CS
            {
                uint8_t adv_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, 0);
                em_ble_acltxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, adv_txdesc_idx));
            }

            // set Synchronization Word
            em_ble_syncwl_set(cs_idx, LE_ADV_CH_ACC_ADDR_L);
            em_ble_syncwh_set(cs_idx, LE_ADV_CH_ACC_ADDR_H);
            // set CRC Initialization value
            em_ble_crcinit0_set(cs_idx, 0x5555);
            em_ble_crcinit1_pack(cs_idx, /*rxmaxctebuf*/ 0, /*crcinit1*/ 0x55);
            // Set Rx Max buf and Rx Max Time @0x0 -> v4.0 behavior
            em_ble_rxmaxbuf_set(cs_idx, 0x0);
            em_ble_rxmaxtime_set(cs_idx, 0x0);
            // set max event time in slots (unit of 625 us)
            {
                // 8 slots should be enough even for the longest legacy advertising event
                uint16_t max_evt_time = (adv_par->properties & ADV_LEGACY) ? 8 : 0; //adv_par->total_txdesc_count*adv_par->sec_aux_offset*30/SLOT_SIZE;
                //ASSERT_ERR(max_evt_time < adv_par->intv);
                em_ble_maxevtime_set(cs_idx, max_evt_time);
            }


            // Set Rx/Tx threshold + rate
            em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/0, /*txthr*/0, /*auxrate*/ adv_par->lld_sec_adv_rate,
                                         /*rxrate*/ adv_par->lld_prim_adv_rate, /*txrate*/ adv_par->lld_prim_adv_rate);


            // Set link field
            em_ble_linkcntl_pack(cs_idx, /*hplpmode*/ 0, /*linklbl*/ cs_idx, /*sas*/ false, /*nullrxllidflt*/true, /*micmode*/ENC_MIC_PRESENT,
                                 /*cryptmode*/ENC_MODE_PKT_PLD_CNT, /*txcrypten*/ false, /*rxcrypten*/ false,
                                 /*privnpub*/ (params->own_addr_type & ADDR_MASK));

            // Set the Device identity (BD Address)
            em_ble_lebdaddr_set(cs_idx, 0, co_read16p(&params->own_addr.addr[0]));
            em_ble_lebdaddr_set(cs_idx, 1, co_read16p(&params->own_addr.addr[2]));
            em_ble_lebdaddr_set(cs_idx, 2, co_read16p(&params->own_addr.addr[4]));

            // Set the advertising channel map
            em_ble_chmap0_set(cs_idx, 0);
            em_ble_chmap1_set(cs_idx, 0);
#if (EAVESDROPPING_SUPPORT)
            if (adv_par->cust_adv_params.en && !adv_par->cust_adv_params.group_ext_adv_ind)
            {
                while (!(adv_par->curr_ch_map & adv_par->prim_adv_ch_map))
                {
                    adv_par->curr_ch_map = adv_par->curr_ch_map << 1;
                }

                em_ble_chmap2_advchmap_setf(cs_idx, adv_par->curr_ch_map);
            }
            else
#endif // EAVESDROPPING_SUPPORT
            {
                em_ble_chmap2_advchmap_setf(cs_idx, params->prim_adv_ch_map);
            }

            // Disable unused control
            em_ble_rxwincntl_set(cs_idx, 0);
            em_ble_txdfantpattcntl_set(cs_idx, 0);
            em_ble_minevtime_set(cs_idx, 0);
            em_ble_evtcnt_setf(cs_idx, 0);
            em_ble_txheadercntl_set(cs_idx, 0);

            TRC_REQ_ADV_TX_PDU(EM_BLE_TXDESC_INDEX(adv_par->act_id, 0), cs_idx);

            GLOBAL_INT_DISABLE();

#if (EAVESDROPPING_SUPPORT)
            if (adv_par->manual_adv_params.en)
            {
                // No timeout
                adv_par->timeout_ts = LLD_CLOCK_UNDEF;

                // Prevent going to deep sleep until after the trigger
                rwip_prevent_sleep_set(RW_BLE_ACTIVE_MODE);

                // Enable the advertising trigger
                ble_advtrigcntl_advtrigen_setf(1);

                // Wait for the interrupt
                adv_par->state = ADV_EVT_WAIT;

                status = CO_ERROR_NO_ERROR;
            }
            else if (adv_par->cust_adv_params.en)
            {
                evt->time.hs = lld_read_clock();

                // Temporarily disable randomization
                adv_par->cust_adv_params.rand_en = false;

                // Subtract the periodicity as it will be added back by the scheduling function
                evt->time.hs = CLK_SUB(evt->time.hs, 2 * adv_par->cust_adv_params.periodicity);

                // Schedule
                if (adv_par->properties & ADV_LEGACY)
                {
                    lld_adv_custom_sched(act_id, false);
                }
                else
                {
                    // Select the first Tx descriptor in the chain
                    adv_par->curr_txdesc_base_index = 0;
                    if (!adv_par->cust_adv_params.group_ext_adv_ind)
                    {
                        // Select the first primary advertising channel
                        adv_par->curr_ch_map = 4;
                        lld_adv_ext_custom_next_channel(act_id);
                    }

                    lld_adv_ext_custom_sched(act_id, false);
                }

                // Initialize the timestamp of the last custom advertising event
                adv_par->last_cust_evt_ts = evt->time.hs;

                // Restore randomization
                adv_par->cust_adv_params.rand_en = params->cust_adv_params.rand_en;

                status = CO_ERROR_NO_ERROR;
            }
            else
#endif // EAVESDROPPING_SUPPORT
            {
                // Schedule event ASAP
                if (adv_par->skip_en)
                    evt->duration_min = adv_par->prim_aux_offset * 30 + HALF_SLOT_SIZE; // Add some margin for the skip adjustment offset

                evt->time.hs = clock;
                evt->time.hus = 0;
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                {
                    lld_adv_duration_update(act_id, params->duration);
                    adv_par->state = ADV_EVT_WAIT;

                    // Initialize the timestamp of the last priority update
                    adv_par->last_prio_upd_ts = evt->time.hs;

                    // Insert the aux event if needed
                    if (adv_par->skip_en)
                    {
                        if (adv_par->timeout_ts != LLD_CLOCK_UNDEF)
                        {
                            SCH_ARB_ASAP_STG_SET(aux_evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_AUX_IDX));
                            aux_evt->asap_limit = adv_par->timeout_ts;
                        }

                        aux_evt->time.hs = CLK_ADD_2(evt->time.hs, 2 * adv_par->sec_adv_max_skip * adv_par->intv);
                        if (sch_arb_insert(aux_evt) != SCH_ARB_ERROR_OK)
                        {
                            adv_par->skip_en = false;
                            evt->duration_min = aux_evt->duration_min;
                        }
                        else
                        {
                            // Initialize the timestamp of the last priority update
                            adv_par->last_prio_upd_aux_ts = aux_evt->time.hs;
                        }
                    }
                }
                else
                {
                    ASSERT_ERR_FORCE(0);
                }

                status = CO_ERROR_NO_ERROR;
            }

            GLOBAL_INT_RESTORE();
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }

    DBG_SWDIAG(LEADV, START, 0);

    return (status);
}

uint8_t lld_adv_stop(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_STOP_FUNC_BIT, uint8_t, act_id);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(LEADV, STOP, 1);

    GLOBAL_INT_DISABLE();

    if (lld_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

        switch (adv_par->state)
        {
        case ADV_EVT_WAIT:
        {
            lld_adv_end(act_id, true, CO_ERROR_NO_ERROR);
        }
        break;

        case ADV_EVT_ACTIVE:
        {
            // Abort advertising, and minimize maxevtime in case pre-fetch
            em_ble_maxevtime_set(EM_BLE_CS_ACT_ID_TO_INDEX(adv_par->act_id), 1);
            ble_rwblecntl_advert_abort_setf(1);
            // Move state
            adv_par->state = ADV_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(LEADV, STOP, 0);

    return (status);
}

// Update advertising data in a safe manner
void lld_adv_adv_data_update(uint8_t act_id, uint8_t len, uint16_t buffer)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_ADV_DATA_UPDATE_FUNC_BIT, act_id, len, buffer);

    GLOBAL_INT_DISABLE();

    if (lld_adv_env[act_id] != NULL)
    {
        if (lld_adv_env[act_id]->state == ADV_EVT_WAIT)
        {
            lld_adv_adv_data_set(act_id, len, buffer, true);
        }
        else
        {
            lld_adv_env[act_id]->data.adv_data_len = len;
            lld_adv_env[act_id]->data.adv_data_buf = buffer;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}

// Update scan response data in a safe manner
void lld_adv_scan_rsp_data_update(uint8_t act_id, uint8_t len, uint16_t buffer)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_SCAN_RSP_DATA_UPDATE_FUNC_BIT, act_id, len, buffer);

    GLOBAL_INT_DISABLE();

    if (lld_adv_env[act_id] != NULL)
    {
        if (lld_adv_env[act_id]->state == ADV_EVT_WAIT)
        {
            lld_adv_scan_rsp_data_set(act_id, len, buffer, true);
        }
        else
        {
            lld_adv_env[act_id]->data.scan_rsp_data_len = len;
            lld_adv_env[act_id]->data.scan_rsp_data_buf = buffer;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}

void lld_adv_duration_update(uint8_t act_id, uint16_t duration)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_DURATION_UPDATE_FUNC_BIT, act_id, duration);

    GLOBAL_INT_DISABLE();

    if (lld_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_adv_env[act_id]->evt);
        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

        if (adv_par->properties & ADV_DIRECT_HI)
        {
            adv_par->timeout_ts = CLK_ADD_2(evt->time.hs, 2 * MAX_HDC_ADV_DUR);
        }
        else if (duration != 0)
        {
            adv_par->timeout_ts = CLK_ADD_2(evt->time.hs, 2 * 16 * duration);
        }
        else
        {
            adv_par->timeout_ts = LLD_CLOCK_UNDEF;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}

void lld_adv_rand_addr_update(uint8_t act_id, struct bd_addr addr)
{

    GLOBAL_INT_DISABLE();

    if (lld_adv_env[act_id] != NULL)
    {
        if (lld_adv_env[act_id]->own_addr_type & ADDR_RAND)
        {
            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

            // Set the Random Address
            em_ble_lebdaddr_set(cs_idx, 0, co_read16p(&addr.addr[0]));
            em_ble_lebdaddr_set(cs_idx, 1, co_read16p(&addr.addr[2]));
            em_ble_lebdaddr_set(cs_idx, 2, co_read16p(&addr.addr[4]));
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}

void lld_adv_restart(uint8_t act_id, uint16_t duration, uint8_t max_ext_adv_evt)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_RESTART_FUNC_BIT, act_id, duration, max_ext_adv_evt);

    GLOBAL_INT_DISABLE();

    if (lld_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_adv_env[act_id]->evt);
        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

        // Reset the event count
        adv_par->ext_adv_evt_cnt = 0;

        // Update the maximum number of extended advertising events
        adv_par->max_ext_adv_evt = max_ext_adv_evt;

#if (EAVESDROPPING_SUPPORT)
        if (adv_par->manual_adv_params.en)
        {
            // No timeout
            adv_par->timeout_ts = LLD_CLOCK_UNDEF;

            // Prevent going to deep sleep until after the trigger
            rwip_prevent_sleep_set(RW_BLE_ACTIVE_MODE);

            // Enable the advertising trigger
            ble_advtrigcntl_advtrigen_setf(1);

            // Wait for the interrupt
            adv_par->state = ADV_EVT_WAIT;
        }
        else
#endif // EAVESDROPPING_SUPPORT
            // Update the duration
            if (adv_par->properties & ADV_DIRECT_HI)
            {
                adv_par->timeout_ts = CLK_ADD_2(evt->time.hs, 2 * MAX_HDC_ADV_DUR);
            }
            else if (duration != 0)
            {
                adv_par->timeout_ts = CLK_ADD_2(evt->time.hs, 2 * 16 * duration);
            }
            else
            {
                adv_par->timeout_ts = LLD_CLOCK_UNDEF;
            }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}

void lld_adv_sync_info_update(uint8_t act_id, uint8_t per_adv_id, uint16_t per_adv_interval)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_ADV_PATCH_TYPE, LLD_ADV_SYNC_INFO_UPDATE_FUNC_BIT, act_id, per_adv_id, per_adv_interval);

    GLOBAL_INT_DISABLE();

    if (lld_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

        // Check if sync info can be added
        if (!(adv_par->properties & (ADV_LEGACY | ADV_CON | ADV_SCAN)))
        {
            adv_par->per_act_id = per_adv_id;
            adv_par->per_adv_intv = per_adv_interval;
            adv_par->sync_update = true;

            if (lld_adv_env[act_id]->state == ADV_EVT_WAIT)
            {
                lld_adv_sync_info_prepare(act_id);
                adv_par->sync_update = false;
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}

#if (BT_53)
void lld_adv_chg_reasons_update(uint8_t act_id, uint8_t chg_reasons)
{
    GLOBAL_INT_DISABLE();

    if (lld_adv_env[act_id] != NULL)
    {
        // Store change reasons
        lld_adv_env[act_id]->chg_reasons = chg_reasons;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}
#endif // (BT_53)

#if (EAVESDROPPING_SUPPORT)
void lld_adv_trig_isr(void)
{
    uint8_t act_id = lld_adv_trig_id;

    if (lld_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_adv_env_tag *adv_par = lld_adv_env[act_id];

        if (adv_par->manual_adv_params.en)
        {
            struct sch_arb_elt_tag *evt = &(adv_par->evt);

            uint32_t base_cnt = ble_advtrigstat0_advtrigclkncnt_getf();
            uint32_t fine_cnt = LLD_FINECNT_MAX - ble_advtrigstat1_advtrigfinecnt_getf();
            uint32_t rand_delay_hus = 2 * CO_MOD(co_rand_word(), (adv_par->manual_adv_params.max_rand_offset + 1));

            // Add the time offset
            uint32_t delay_hus = co_max(rwip_prog_delay * HALF_SLOT_SIZE, 2 * adv_par->manual_adv_params.time_offset);
            base_cnt += delay_hus / HALF_SLOT_SIZE;
            fine_cnt += CO_MOD(delay_hus, HALF_SLOT_SIZE);

            // Add the random offset
            base_cnt += rand_delay_hus / HALF_SLOT_SIZE;
            fine_cnt += CO_MOD(rand_delay_hus, HALF_SLOT_SIZE);

            base_cnt += fine_cnt / HALF_SLOT_SIZE;
            fine_cnt = CO_MOD(fine_cnt, HALF_SLOT_SIZE);

            ASSERT_ERR(base_cnt > (lld_read_clock() + rwip_prog_delay));

            // Schedule event
            evt->current_prio = rwip_priority[RWIP_PRIO_ADV_AUX_IDX].value;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_AUX_IDX));

            evt->time.hs = base_cnt;
            evt->time.hus = fine_cnt;
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                adv_par->state = ADV_EVT_WAIT;

                // Initialize the timestamp of the last priority update
                adv_par->last_prio_upd_ts = evt->time.hs;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
    }
}
#endif // EAVESDROPPING_SUPPORT

#endif // (BLE_BROADCASTER)
///@} LLDADV
