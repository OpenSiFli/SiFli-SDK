/**
****************************************************************************************
*
* @file lld_test_mode.c
*
* @brief LLD test mode source code
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDTESTMODE
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"         // stack configuration
#if (BLE_TEST_MODE_SUPPORT)

#include "ke_mem.h"
#include "ke_msg.h"              // kernel messages
#include "rwip.h"

#include "lld.h"                 // link driver API
#include "lld_int.h"             // link layer driver internal

#include "sch_arb.h"             // Scheduling Arbiter
#include "sch_prog.h"            // Scheduling Programmer

#include "reg_blecore.h"         // BLE core registers
#include "reg_em_ble_cs.h"       // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors

#include "reg_em_ble_rx_cte_desc.h" // BLE EM RX CTE descriptors
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Link ID used for test mode
#define TEST_LINK_ID             0

/// Duration of test mode event in slots (2 slots)
#define TEST_DUR                 2

/// Test mode maximum size
#define TEST_SIZE_MAX            255

/// Error mask for CTE packet reception
#define LLD_TEST_CTE_ERR_MASK    (EM_BLE_RXCTEERR_BIT|EM_BLE_CRC_ERR_BIT|EM_BLE_SYNC_ERR_BIT)

/// Test mode event states
enum TEST_EVT_STATE
{
    TEST_EVT_WAIT,
    TEST_EVT_ACTIVE,
    TEST_EVT_END,
};

/// Test mode event states
enum TEST_TYPE
{
    TEST_RX,
    TEST_TX,
};

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LLD test mode environment structure
struct lld_test_env_tag
{
    /// Pointer to inquiry event
    struct sch_arb_elt_tag evt;

    /// Buffer used for sending the test data
    uint16_t em_buf;

    /// Type (0: RX | 1: TX)
    uint8_t type;

    /// RF channel, N = (F - 2402) / 2
    uint8_t channel;

    /// Length of test data
    uint8_t data_len;

    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t payload;

    /// current state of the test mode
    uint8_t state;

    /**
     * CTE length
     * 0x00 No Constant Tone Extension
     * 0x02 - 0x14 Length of the Constant Tone Extension in 8 us units
     * All other values Reserved for future use
     */
    uint8_t cte_len;

    /**
     * CTE type
     * 0x00 AoA Constant Tone Extension
     * 0x01 AoD Constant Tone Extension with 1 us slots
     * 0x02 AoD Constant Tone Extension with 2 us slots
     * All other values Reserved for future use
     */
    uint8_t cte_type;

    /**
     * Slot durations
     * 0x01 Switching and sampling slots are 1 us each
     * 0x02 Switching and sampling slots are 2 us each
     * All other values Reserved for future use
     */
    uint8_t slot_dur;

    /**
     * Length of switching pattern
     * 0x02 - 0x4B The number of Antenna IDs in the pattern
     * All other values Reserved for future use
     */
    uint8_t switching_pattern_len;

    /// Antenna IDs
    //uint8_t antenna_id[BLE_MAX_SW_PAT_LEN];

#if (EAVESDROPPING_SUPPORT)
    struct lld_cust_tx_test_params cust_tx_test_params;
#endif //(EAVESDROPPING_SUPPORT)
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD test mode environment variable
__STATIC struct lld_test_env_tag *lld_test_env;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void lld_test_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup test environment variable
 ****************************************************************************************
 */
__STATIC void lld_test_cleanup(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_CLEANUP_FUNC_BIT);

    if (lld_test_env != NULL)
    {
        // Re-enable force AGC mechanism
        rwip_rf.force_agc_enable(true);
        // Clear the Regulatory Body and RF Testing Register
        ble_rftestcntl_set(0);
        // Enable the whitening
        ble_rwblecntl_whit_dsb_setf(0);

        // Remove permission/status of CS as now unused
        DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(EM_BLE_CS_ACT_ID_TO_INDEX(TEST_LINK_ID))), REG_EM_BLE_CS_SIZE, false, false, false);

        // Free event memory
        ke_free(lld_test_env);
        lld_test_env = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief retrieve BLE channel from frequency offset
 *
 * @param[in] freq Frequency offset (0 = 2400 MHz), step 2MHz
 *
 * @return corresponding BLE channel
 ****************************************************************************************
 */
__STATIC uint8_t lld_test_freq2chnl(uint8_t freq)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_FREQ2CHNL_FUNC_BIT, uint8_t, freq);

    uint8_t chnl = 0;

    switch (freq)
    {
    case (0):
    {
        freq = 37;
    }
    break;

    case (12):
    {
        freq = 38;
    }
    break;

    case (39):
    {
        // Nothing
    } break;

    default:
    {
        if (freq < 12)
        {
            freq -= 1;
        }
        else
        {
            freq -= 2;
        }
    }
    break;
    }
    chnl = freq;

    return (chnl);
}

/**
 ****************************************************************************************
 * @brief Fulfills the payload for the transmit test mode.
 *
 * This function fulfills the payload for the transmit test mode.
 *
 * @param[in] pattern_type         type of the pattern.
 * @param[in] payload_len          length of the payload.
 *
 ****************************************************************************************
 */
__INLINE void lld_gen_pattern(uint8_t pattern_type, uint8_t payload_len, uint8_t *payload)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_TEST_PATCH_TYPE, LLD_GEN_PATTERN_FUNC_BIT, pattern_type, payload_len, payload);

    uint8_t pattern = 0;
    // get the pattern
    switch (pattern_type)
    {
    case PAYL_11110000:
        pattern = 0xF0;
        break;
    case PAYL_10101010:
        pattern = 0xAA;
        break;
    case PAYL_ALL_1:
        pattern = 0xFF;
        break;
    case PAYL_ALL_0:
        pattern = 0x00;
        break;
    case PAYL_00001111:
        pattern = 0x0F;
        break;
    case PAYL_01010101:
        pattern = 0x55;
        break;
    default:
        ASSERT_ERR(pattern_type <= PAYL_01010101);
        break;
    }
    // fulfill the payload
    memset(payload, pattern, payload_len);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_test_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_EVT_START_CBK_FUNC_BIT, evt);

    //DBG_SWDIAG(TEST, TEST_EVT_START, 1);

    ASSERT_ERR(&(lld_test_env->evt) == evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_test_env_tag *test_par = (struct lld_test_env_tag *) evt;
        struct sch_prog_params prog_par;
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(TEST_LINK_ID);

        // Push the programming to SCH PROG
        prog_par.frm_cbk        = &lld_test_frm_cbk;
        prog_par.time.hs        = evt->time.hs;
        prog_par.time.hus       = 0;
        prog_par.cs_idx         = cs_idx;
        prog_par.dummy          = cs_idx;
        prog_par.bandwidth      = evt->duration_min;
        prog_par.prio_1         = evt->current_prio;
        prog_par.prio_2         = 0;
        prog_par.prio_3         = 0;
        prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
        prog_par.add.ble.ae_nps = 0;
        prog_par.add.ble.iso    = 0;
        prog_par.mode           = SCH_PROG_BLE;
        sch_prog_push(&prog_par);

        // Move state
        test_par->state = TEST_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR(0);
    }

    //DBG_SWDIAG(TEST, TEST_EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_test_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_EVT_CANCELED_CBK_FUNC_BIT, evt);

    //DBG_SWDIAG(TEST, TEST_EVT_CANCELED, 1);

    ASSERT_ERR(&(lld_test_env->evt) == evt);

    if (evt != NULL)
    {
        ASSERT_ERR(((struct lld_test_env_tag *) evt)->state == TEST_EVT_WAIT);

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

        // Reschedule ASAP
        if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
        {
            ASSERT_ERR(0);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    //DBG_SWDIAG(TEST, TEST_EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle Rx interrupt
 ****************************************************************************************
 */
__STATIC void lld_test_rx_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_RX_ISR_FUNC_BIT, timestamp);

    //DBG_SWDIAG(TEST, TEST_RX_ISR, 1);

    if (lld_test_env != NULL)
    {
        // Check if a packet has been received
        while (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(TEST_LINK_ID)))
        {
#if BLE_CONLESS_CTE_RX
            // Point to parameters
            struct lld_test_env_tag *test_par = lld_test_env;

            if ((test_par->type == TEST_RX) && (test_par->cte_len != NO_CTE))
            {
                // Get current RX descriptor index
                uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;
                // Retrieve RX CTE PTR
                uint16_t rxcteptr = em_ble_rxcteptr_get(rxdesc_idx);

                if (rxcteptr != 0)
                {
                    ASSERT_INFO(rxcteptr >= EM_BLE_RX_CTE_DESC_OFFSET, rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);
                    ASSERT_INFO(rxcteptr <  EM_BLE_RX_CTE_DESC_END,    rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);

                    // Get RX CTE descriptor index
                    uint8_t em_rx_cte_desc_idx = (rxcteptr - EM_BLE_RX_CTE_DESC_OFFSET) / REG_EM_BLE_RX_CTE_DESC_SIZE;
                    // Retrieve RX status
                    uint16_t rxstat = em_ble_rxstatadv_get(rxdesc_idx);

                    uint16_t rxphcte = em_ble_rxphcte_get(rxdesc_idx);
                    uint8_t nbrxiqsamp = GETF(rxphcte, EM_BLE_NBRXIQSAMP);

                    // The CP bit corresponds to RXCHSEL2 in the Rx descriptor
                    uint16_t cp = em_ble_rxphadv_rxchsel2_getf(rxdesc_idx);

                    // If CP bit and no error
                    if (cp && ((rxstat & LLD_TEST_CTE_ERR_MASK) == 0) && (nbrxiqsamp != 0))
                    {
                        // Report CTE reception
                        struct lld_conless_cte_rx_ind *msg = KE_MSG_ALLOC(LLD_CONLESS_CTE_RX_IND, TASK_LLM, TASK_NONE, lld_conless_cte_rx_ind);

                        msg->act_id = TEST_LINK_ID;
                        msg->em_rx_cte_desc_idx = em_rx_cte_desc_idx;
                        msg->channel_idx = em_ble_rxchass_used_ch_idx_getf(rxdesc_idx);
                        msg->rssi = 10 * rwip_rf.rssi_convert(em_ble_rxchass_rssi_getf(rxdesc_idx)) - lld_env.rx_path_comp; // Convert to 0.1 dBm
                        msg->rssi_antenna_id = 0;
                        msg->cte_type = GETF(rxphcte, EM_BLE_RXCTETYPE);
                        msg->slot_dur = test_par->slot_dur;
                        msg->sample_cnt = nbrxiqsamp;
                        msg->pa_evt_cnt = 0;

                        ke_msg_send(msg);
                    }
                    else
                    {
                        // Release the Rx CTE descriptor
                        em_ble_rxctecntl_rxdone_setf(em_rx_cte_desc_idx, 0);
                    }
                }
            }
#endif // BLE_CONLESS_CTE_RX

            // Free RX descriptor
            lld_rxdesc_free();
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    //DBG_SWDIAG(TEST, TEST_RX_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_test_frm_isr(uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_FRM_ISR_FUNC_BIT, timestamp, abort);

    //DBG_SWDIAG(TEST, TEST_FRM_ISR, 1);

    if (lld_test_env != NULL)
    {
        // Point to parameters
        struct lld_test_env_tag *test_par = lld_test_env;
        struct sch_arb_elt_tag *evt = &(lld_test_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

#if (EAVESDROPPING_SUPPORT)
        if ((test_par->type == TEST_TX) && (test_par->cust_tx_test_params.enable) && (test_par->cust_tx_test_params.tx_packets > 0) && !abort)
        {
            // The number of remaining Tx packets must be 0 at this point
            ASSERT_ERR(ble_rftesttxstat_txpktcnt_getf() == 0);

            test_par->state = TEST_EVT_END;
        }
#endif //(EAVESDROPPING_SUPPORT)

        // Check test mode end
        if (test_par->state == TEST_EVT_END)
        {
            // Report test mode end to LLM
            struct lld_test_end_ind *ind = KE_MSG_ALLOC(LLD_TEST_END_IND, TASK_LLM, TASK_NONE, lld_test_end_ind);
            ind->status = CO_ERROR_NO_ERROR;
            ind->nb_pkt_recv = (test_par->type == TEST_RX) ? em_ble_rxccmpktcnt0_get(EM_BLE_CS_ACT_ID_TO_INDEX(TEST_LINK_ID)) : 0;
            ke_msg_send(ind);

            if (test_par->type == TEST_TX)
            {
                // Release TX buffer
                ble_util_buf_acl_tx_free(test_par->em_buf);
            }

            // Free event memory
            lld_test_cleanup();
        }
        else
        {

            // update event priority
            evt->current_prio = abort
                                ? RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX))
                                : rwip_priority[RWIP_PRIO_ADV_IDX].value;

            // Reschedule ASAP
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

            // Try to reschedule
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                test_par->state = TEST_EVT_WAIT;
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    //DBG_SWDIAG(TEST, TEST_FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_test_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_FRM_CBK_FUNC_BIT, timestamp, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    {
        lld_test_frm_isr(timestamp, false);
    }
    break;
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        lld_test_frm_isr(timestamp, true);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        lld_test_rx_isr(timestamp);
    }
    break;
    default:
    {
        ASSERT_INFO(0, dummy, irq_type);
    }
    break;
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

uint8_t lld_test_start(struct lld_test_params *params)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_START_FUNC_BIT, uint8_t, params);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    uint8_t rf_mod;
    uint8_t txpwr;

    // Check if test mode is inactive
    if (lld_test_env == NULL)
    {
        // Allocate environment
        lld_test_env = LLD_ALLOC_ENV(lld_test_env_tag);

        if (lld_test_env != NULL)
        {
            // Point to parameters
            struct lld_test_env_tag *test_par = lld_test_env;
            struct sch_arb_elt_tag *evt = &(lld_test_env->evt);

            uint32_t clock = lld_read_clock();
            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(TEST_LINK_ID);

            LLD_INIT_ENV(test_par, lld_test_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &lld_test_evt_canceled_cbk;
            evt->cb_start         = &lld_test_evt_start_cbk;
            evt->current_prio     = rwip_priority[RWIP_PRIO_ADV_IDX].value;
            evt->duration_min     = 2 * TEST_DUR * HALF_SLOT_SIZE;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_ADV_IDX));

            // Initialize event parameters (test mode part)
            test_par->type = params->type;
            test_par->channel = params->channel;
            test_par->cte_len = params->cte_len;
            test_par->cte_type = params->cte_type;
            test_par->slot_dur = params->slot_dur;
            test_par->switching_pattern_len = params->switching_pattern_len;

            // Pre-initialization of conditionally set fields
            em_ble_crcinit1_set(cs_idx, 0);
            em_ble_rxdfantpattcntl_set(cs_idx, 0);

            // Check the type of the test (Rx/Tx)
            if (params->type == TEST_RX)
            {
                //disable the whitening
                ble_rwblecntl_whit_dsb_setf(1);

                // Set the RF test control register
                // Rx packet count enabled, and reported in CS-RXCCMPKTCNT and RFTESTRXSTAT-RXPKTCNT on RF abort command
                ble_rftestcntl_rxpktcnten_setf(1);
                // Clear counter dedicated for the test mode
                em_ble_rxccmpktcnt0_set(cs_idx, 0);
                em_ble_rxccmpktcnt1_set(cs_idx, 0);
                em_ble_rxccmpktcnt2_set(cs_idx, 0);

                // Update the control structure according to the parameters
                em_ble_cntl_set(cs_idx, EM_BLE_CS_FMT_RX_TEST);

                // set Wide-open mode, Size of the Rx window in slots
                em_ble_rxwincntl_pack(cs_idx, 1, ((75 + 1) >> 1));

                // extend length to 251
                em_ble_rxmaxbuf_set(cs_idx, TEST_SIZE_MAX);
                em_ble_rxmaxtime_set(cs_idx, 0);

                // Check if CTE is enabled
                if (params->cte_len != NO_CTE)
                {
                    // If multiple antennae are used
                    if (params->cte_type == CTE_TYPE_AOA)
                    {
                        // Set the Antenna ID pointer
                        em_ble_rxdfantswptr_set(cs_idx, EM_BLE_RX_ANTENNA_ID_OFFSET >> 2);

                        // Write antenna IDs to EM
                        em_wr(&params->antenna_id[0], EM_BLE_RX_ANTENNA_ID_OFFSET, params->switching_pattern_len);

                        // Set the length of the switching pattern
                        em_ble_rxdfantpattcntl_rx_ant_patt_length_setf(cs_idx, params->switching_pattern_len);
                    }
                    else
                    {
                        // Disable antenna switching
                        em_ble_rxdfantpattcntl_rx_ant_patt_length_setf(cs_idx, 0);
                    }

                    // max I&Q samples per CTE
                    em_ble_crcinit1_rxmaxctebuf_setf(cs_idx, LLD_MAX_CTE_IQ_SAMPLES);

                    // max sampled CTE per event - 0x1F - no limit
                    em_ble_rxdfantpattcntl_max_samp_cte_setf(cs_idx, 0x1F);

                    // Set RXDFCNTL
                    {
                        uint8_t slot_dur = (params->cte_type == CTE_TYPE_AOA) ? params->slot_dur : params->cte_type;
                        uint8_t df_type = (params->cte_type == CTE_TYPE_AOA) ? 1 : 2;
                        em_ble_rxdfcntl_pack(cs_idx, /*DFRSPEN*/ 0, /*DFSWCNTL*/ slot_dur, /*DFSAMPCNTL*/ slot_dur, /*DFTYPE*/ df_type, /*DFFILTEREN*/ 0, /*DFEN*/ 1);
                    }
                }
                else
                {
                    // Disable antenna switching
                    em_ble_rxdfantpattcntl_rx_ant_patt_length_setf(cs_idx, 0);
                    // Disable CTE reception
                    em_ble_rxdfcntl_set(cs_idx, 0);
                }

                // Disable unused control
                em_ble_txrxcntl_set(cs_idx, 0);
                em_ble_txheadercntl_set(cs_idx, 0);

                // Disable force AGC mechanism
                rwip_rf.force_agc_enable(false);
            }
            else if (params->type == TEST_TX)
            {
                test_par->data_len = params->data_len;
                test_par->payload = params->payload;

#if (EAVESDROPPING_SUPPORT)
                test_par->cust_tx_test_params.enable = params->cust_tx_test_params.enable;
                test_par->cust_tx_test_params.tx_packets = params->cust_tx_test_params.tx_packets;
                test_par->cust_tx_test_params.tx_power = params->cust_tx_test_params.tx_power;
#endif //(EAVESDROPPING_SUPPORT)

                // System ram buffer to be filled
                uint8_t data[TEST_SIZE_MAX];
                // Exchange memory buffer to be filled
                uint16_t buf_ptr = ble_util_buf_acl_tx_alloc();
                uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(TEST_LINK_ID, 0);

                if (buf_ptr != 0)
                {
                    // Save test buffer pointer
                    test_par->em_buf = buf_ptr;

                    em_ble_txdataptr_setf(txdesc_idx, test_par->em_buf);

                    //disable the whitening
                    ble_rwblecntl_whit_dsb_setf(1);

                    // Set the RF test control register
                    // Tx packet count enabled, and reported in CS-TXCCMPKTCNT and RFTESTRXSTAT-TXPKTCNT on RF abort command
                    ble_rftestcntl_txpktcnten_setf(1);
                    // Clear counter dedicated for the test mode
                    em_ble_txccmpktcnt0_set(cs_idx, 0);
                    em_ble_txccmpktcnt1_set(cs_idx, 0);
                    em_ble_txccmpktcnt2_set(cs_idx, 0);

                    //check the type of test
                    switch (params->payload)
                    {
                    case PAYL_PSEUDO_RAND_9:
                    case PAYL_PSEUDO_RAND_15:
                        // sets the type of the PRBS
                        ble_rftestcntl_prbstype_setf(params->payload & 0x1);
                        // sets the source to PRBS generator
                        ble_rftestcntl_txpldsrc_setf(1);
                        break;
                    case PAYL_11110000:
                    case PAYL_10101010:
                    case PAYL_ALL_1:
                    case PAYL_ALL_0:
                    case PAYL_00001111:
                    case PAYL_01010101:
                        lld_gen_pattern(params->payload, params->data_len, data);
                        em_wr((void *)&data, test_par->em_buf, params->data_len);
                        // sets the source to CS
                        ble_rftestcntl_txpldsrc_setf(0);
                        break;
                    default:
                        ASSERT_ERR(params->payload <= PAYL_01010101);
                        break;
                    }

                    // set the pattern type and the length in the header
                    em_ble_txphadv_pack(txdesc_idx,                  // index for the descriptor
                                        params->data_len,            // Length
                                        0,                           // txrxadd - peer address type
                                        0,                           // txtxadd - updated by HW
                                        (params->cte_len != NO_CTE), // txchsel2 - Indicates CTE presence
                                        0,                           // txadvrfu - Not set for the moment
                                        params->payload);            // type of PDU

                    // set the CTE type and time
                    {
                        uint8_t cte_type = (params->cte_len != NO_CTE) ? params->cte_type : CTE_TYPE_AOA;

                        em_ble_txphcte_pack(txdesc_idx,                // index for the descriptor
                                            cte_type,                  // Tx CTE type
                                            0,                         // Tx CTE RFU
                                            params->cte_len);          // Tx CTE time
                    }

                    // Free descriptor
                    em_ble_txcntl_txdone_setf(txdesc_idx, 0);
                }
                else
                {
                    // clean-up allocated memory
                    lld_test_cleanup();

                    return (CO_ERROR_MEMORY_CAPA_EXCEED);
                }

                // Check if CTE is enabled and multiple antennae are used
                if ((params->cte_len != NO_CTE) && (params->cte_type != CTE_TYPE_AOA))
                {
                    // Set the Antenna ID pointer
                    em_ble_txdfantswptr_set(cs_idx, EM_BLE_TX_ANTENNA_ID_OFFSET >> 2);

                    // Write antenna IDs to EM
                    em_wr(&params->antenna_id[0], EM_BLE_TX_ANTENNA_ID_OFFSET, params->switching_pattern_len);

                    // Set the length of the switching pattern
                    em_ble_txdfantpattcntl_tx_ant_patt_length_setf(cs_idx, params->switching_pattern_len);
                }
                else
                {
                    // Disable antenna switching
                    em_ble_txdfantpattcntl_tx_ant_patt_length_setf(cs_idx, 0);
                }

                // Set the Tx power level
                switch (params->tx_pwr_lvl)
                {
                case MIN_TX_PWR_LVL:
                    // Minimum Tx power
                    rf_mod = rwip_rf.txpwr_min_mod;
                    txpwr = rwip_rf.txpwr_min;
                    //em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_min);
                    break;
                case MAX_TX_PWR_LVL:
                    // Maximum Tx power
                    rf_mod = rwip_rf.txpwr_max_mod;
                    txpwr = rwip_rf.txpwr_max;
                    //em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_max);
                    break;
                default:
                    // Apply selected Tx power level
                    rf_mod = rwip_rf.txpwr_mdlt_get(params->tx_pwr_lvl, 0);
                    txpwr = rwip_rf.txpwr_cs_get(params->tx_pwr_lvl, TXPWR_CS_NEAREST, rf_mod);
                    //em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_cs_get(params->tx_pwr_lvl, TXPWR_CS_NEAREST));
                    break;
                }


                // Set the Tx descriptor pointer in the CS
                em_ble_acltxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, txdesc_idx));

                // Update the control structure according to the parameters
                em_ble_cntl_set(cs_idx, EM_BLE_CS_FMT_TX_TEST);

#if (EAVESDROPPING_SUPPORT)
                if (test_par->cust_tx_test_params.enable)
                {
                    // Apply selected Tx power level
                    em_ble_txrxcntl_txpwr_setf(cs_idx, rwip_rf.txpwr_cs_get(test_par->cust_tx_test_params.tx_power - lld_env.tx_path_comp / 10, TXPWR_CS_NEAREST));

                    // Configure the number of packets to be transmitted
                    em_ble_txccmpktcnt0_set(cs_idx, test_par->cust_tx_test_params.tx_packets);
                }
#endif //(EAVESDROPPING_SUPPORT)
            }
            else
            {
                ASSERT_ERR(0);
            }

            // Initialize the test mode control structure
            em_ble_syncwl_set(cs_idx, 0x4129);
            em_ble_syncwh_set(cs_idx, 0x7176);
            // set CRC Initialization value
            em_ble_crcinit0_set(cs_idx, 0x5555);
            em_ble_crcinit1_crcinit1_setf(cs_idx, 0x55);
            // Get the channel index matching with the provided frequency
            em_ble_hopcntl_pack(cs_idx, /*fhen*/false,/*hopmode*/LLD_HOP_MODE_CHAN_SEL_1, /*hopint*/ 0,
                                /* chidx */lld_test_freq2chnl(params->channel));

            // Set link label
            em_ble_linkcntl_pack(cs_idx, /*hplpmode*/ 0, /* linklbl */ cs_idx, /*sas*/ 0, /*nullrxllidflt*/0, /*micmode*/0,
                                 /*cryptmode*/0, /*txcrypten*/ 0, /*rxcrypten*/ 0, /*privnpub*/ 0);

            // Disable unused control
            em_ble_chmap0_set(cs_idx, 0);
            em_ble_chmap1_set(cs_idx, 0);
            em_ble_chmap2_set(cs_idx, 0);
            em_ble_minevtime_set(cs_idx, 0);
            em_ble_maxevtime_set(cs_idx, 0);
            em_ble_evtcnt_setf(cs_idx, 0);
            em_ble_txheadercntl_set(cs_idx, 0);

            // Set Rx/Tx threshold + rate
            {
                uint8_t tx_phy = (params->type == TEST_TX) ? (params->phy - 1) : 0;
                uint8_t rx_phy = (params->type == TEST_RX) ? (params->phy - 1) : 0;

                em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/1, /*txthr*/0, /*auxrate*/0,
                                             /*rxrate*/rx_phy, /*txrate*/tx_phy);
            }
            // Initialize packet counter
            em_ble_rxccmpktcnt0_set(cs_idx, 0);
            em_ble_rxccmpktcnt1_set(cs_idx, 0);
            em_ble_rxccmpktcnt2_set(cs_idx, 0);

            // Schedule event ASAP
            evt->time.hs = clock;

            rwip_rf.txpwr_cs_set(TYPE_BLE, cs_idx, rf_mod, txpwr);

            GLOBAL_INT_DISABLE();

            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                test_par->state = TEST_EVT_WAIT;
            }
            else
            {
                ASSERT_ERR(0);
            }

            GLOBAL_INT_RESTORE();

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR(0);
        }
    }

    return (status);
}

uint8_t lld_test_stop(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_STOP_FUNC_BIT, uint8_t);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_test_env != NULL)
    {
        // Point to parameters
        struct lld_test_env_tag *test_par = lld_test_env;
        struct sch_arb_elt_tag *evt = &(lld_test_env->evt);

        switch (test_par->state)
        {
        case TEST_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            if (test_par->type == TEST_TX)
            {
                // Release TX buffer
                ble_util_buf_acl_tx_free(test_par->em_buf);
            }

            // Report test mode end to LLM
            struct lld_test_end_ind *ind = KE_MSG_ALLOC(LLD_TEST_END_IND, TASK_LLM, TASK_NONE, lld_test_end_ind);
            ind->status = CO_ERROR_NO_ERROR;
            ind->nb_pkt_recv = (test_par->type == TEST_RX) ? em_ble_rxccmpktcnt0_get(EM_BLE_CS_ACT_ID_TO_INDEX(TEST_LINK_ID)) : 0;
            ke_msg_send(ind);

            // Free event memory
            lld_test_cleanup();
        }
        break;

        case TEST_EVT_ACTIVE:
        {
            // Abort the event
            ble_rwblecntl_set(ble_rwblecntl_get() | BLE_RFTEST_ABORT_BIT);

            // Move state
            test_par->state = TEST_EVT_END;
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

    return (status);
}


void lld_test_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_TEST_PATCH_TYPE, LLD_TEST_INIT_FUNC_BIT, is_reset);

    if (!is_reset)
    {
        lld_test_env = NULL;
    }
    else
    {
        // clean-up allocated memory
        lld_test_cleanup();
    }
}

#endif // (BLE_TEST_MODE_SUPPORT)
///@} LLDTESTMODE
