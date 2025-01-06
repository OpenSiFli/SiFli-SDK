/**
****************************************************************************************
*
* @file lld_scan.c
*
* @brief LLD Scan source code
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDSCAN
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"         // stack configuration
#if (BLE_OBSERVER)

#include <string.h>

#include "co_utils.h"
#include "co_math.h"

#include "ke_mem.h"
#include "ke_task.h"             // kernel task management
#include "rwip.h"

#include "lld.h"                 // link driver API
#include "lld_int.h"             // link layer driver internal

#if (EAVESDROPPING_SUPPORT)
    #include "lm.h"                  // BR/EDR clock reference for segmented scans
#endif // (EAVESDROPPING_SUPPORT)

#include "dbg.h"

#include "sch_arb.h"             // Scheduling Arbiter
#include "sch_prog.h"            // Scheduling Programmer
#include "sch_slice.h"           // Scheduling Slicer

#include "em_map.h"

#include "reg_blecore.h"         // BLE core registers
#include "reg_em_ble_cs.h"       // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors
#include "reg_em_ble_ral.h"      // BLE RAL structures
#include "reg_em_ble_wpal.h"     // BLE WL structures
#include "reg_em_et.h"           // EM Exchange Table
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Minimum scheduling delay threshold where SW tries to advance the schedule (in BT slots)
#define LLD_SCAN_EVT_DELAY_RESCHED_MIN        30

/// RX Threshold for scanning
#define LLD_SCAN_RX_THRESHOLD                 1

/// Maintain fixed interval on scan windows
#define LLD_SCAN_STRICT_INTERVAL              1

/// Scan Duration Units in usecs from standard N * 10 ms units
#define SCAN_DURATION_USECS(duration)        ((duration) * 10000)

/// Scan Period Units in usecs from standard N * 1.28 sec units
#define SCAN_PERIOD_USECS(period)         ((period) * 1280000)

/// Data status of extended advertising event - Incomplete, data discarded, report not generated
#define ADV_EVT_DATA_STATUS_DISCARDED           0xFF

/// Explicit flush
#define LLD_SCAN_RX_DATA_FLUSH      (HOST_ADV_DATA_LEN_MAX+1)

/// Bit field definition scanning PHYs
#define LLD_PHY_1M_POS            0
#define LLD_PHY_1M_BIT            0x01
#define LLD_PHY_CODED_POS         1
#define LLD_PHY_CODED_BIT         0x02

/*
 * Mapping of scan_id to phy_idx & evt_idx for lld_scan_evt_rx_params:
 *  SCAN_ID      PHY_IDX         EVT_IDX
 * -------------------------------------------------------
 *  0            PHY_1M          EVT_DFT
 *  1            PHY_CODED       EVT_DFT
 *  2            PHY_1M          EVT_AUX
 *  3            PHY_CODED       EVT_AUX
 *-------------------------------------------------------
 */
#define LLD_SCAN_ID_PHY_POS     0
#define LLD_SCAN_ID_PHY_BIT     0x01
#define LLD_SCAN_ID_EVT_POS     1
#define LLD_SCAN_ID_EVT_BIT     0x02

/// Number of SCAN events per PHY
#define NUM_SCAN_EVTS                   (2)

#if (EAVESDROPPING_SUPPORT)
    /// Macro to check if segmented scans are enabled for the specified primary PHY
    #define LLD_SCAN_SEG_EN(phy_idx) (lld_scan_seg_env && (lld_scan_seg_env->scan_phys & CO_BIT(phy_idx)))
#endif // (EAVESDROPPING_SUPPORT)

/*
 * ENUMERATION DEFINITION
 *****************************************************************************************
 */

/// Scan event states
enum SCAN_EVT_STATE
{
    SCAN_EVT_WAIT,
    SCAN_EVT_ACTIVE,
    SCAN_EVT_TERM,
    SCAN_EVT_END,
};

/// Scan AUX PDU states
enum SCAN_AUX_STATE
{
    RX_ADV_EXT_IND = 0,
    RX_AUX_ADV_IND,
    RX_AUX_SCAN_RSP,
    RX_AUX_CHAIN_IND,
    RX_AUX_SYNC_IND,
};

/// Scan AUX offload modes
enum SCAN_OFFLOAD_MODE
{
    NO_AUX_OFFLOAD = 0,
    SW_CNTL_AUX_OFFLOAD,
    HW_CNTL_AUX_OFFLOAD,
};

/// Scan event types
enum SCAN_EVT_TYPE
{
    SCAN_EVT_DFT = 0,
    SCAN_EVT_AUX,
};

/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

/// Table indicating the max duration of an AUX_ADV_IND->AUX_SCAN_REQ->AUX_SCAN_RSP event (in us) depending on the PHY used
const uint16_t lld_scan_max_aux_dur_tab[CO_RATE_MAX] =
{
    [CO_RATE_1MBPS  ]  = (PDU_1MBPS_LEN_US(0)   + BLE_IFS_DUR + PDU_1MBPS_LEN_US(PDU_SCAN_REQ_LEN)   + BLE_IFS_DUR + PDU_1MBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX)),
    [CO_RATE_2MBPS  ]  = (PDU_2MBPS_LEN_US(0)   + BLE_IFS_DUR + PDU_2MBPS_LEN_US(PDU_SCAN_REQ_LEN)   + BLE_IFS_DUR + PDU_2MBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX)),
    [CO_RATE_500KBPS]  = (PDU_500KBPS_LEN_US(0) + BLE_IFS_DUR + PDU_500KBPS_LEN_US(PDU_SCAN_REQ_LEN) + BLE_IFS_DUR + PDU_500KBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX)),
    [CO_RATE_125KBPS]  = (PDU_125KBPS_LEN_US(0) + BLE_IFS_DUR + PDU_125KBPS_LEN_US(PDU_SCAN_REQ_LEN) + BLE_IFS_DUR + PDU_125KBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX)),
};

/// Table for mapping legacy PDU to event type
const uint8_t lld_scan_map_legacy_pdu_to_evt_type[] =
{
    [BLE_ADV_IND] = ADV_IND,
    [BLE_ADV_DIRECT_IND] = ADV_DIRECT_IND,
    [BLE_ADV_NONCONN_IND] = ADV_NONCONN_IND,
    [BLE_SCAN_RSP] = SCAN_RSP_TO_ADV_IND,
    [BLE_ADV_SCAN_IND] = ADV_SCAN_IND,
};

#if (EAVESDROPPING_SUPPORT)
/// Segmented Scan states
enum SEG_SCAN_STATE
{
    SEG_SCAN_ENABLING,
    SEG_SCAN_ON,
};

/// Table indicating length of preamble and access address adjustment on scan windows
const uint16_t lld_scan_preamble_access_addr_tab[NUM_SCAN_EVTS] =
{
    [LLD_PHY_1M_POS] = BLE_PREAMBLE_ACCESS_ADDR_DUR_1MBPS,
    [LLD_PHY_CODED_POS] = BLE_PREAMBLE_ACCESS_ADDR_DUR_125KBPS,
};
#endif //EAVESDROPPING_SUPPORT


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LLD SCAN RX event parameters structure
struct lld_scan_evt_params
{
    /// Pointer to aux scan event
    struct sch_arb_elt_tag evt;

    /// ADI of current receive chain
    uint16_t adi_data;

    /// Aggregate length of reception (bytes)
    uint16_t rx_chain_cnt_bytes;

    /// Chain packet count
    uint8_t rx_chain_cnt_pkt;

    /// Indicates offload software/hardware control (@see SCAN_OFFLOAD_MODE)
    uint8_t aux_offload;

    /// Next AUX packet (@see enum SCAN_AUX_STATE)
    uint8_t aux_state;

    /// Control Structure index
    uint8_t cs_idx;

    /// Scan activity state (@see enum  SCAN_EVT_STATE)
    uint8_t state;

    /// Index of the scan parameters
    uint8_t index;

    /// Packet count in this event
    uint8_t pkt_cnt;

    /// Periodic advertising filters CS enabled
    bool peradv_filt_en;

    /// Active extended advertising report information
    /*
     * Active extended advertising report information container:
     * - Accumulates advertising report info between adv_ext_ind & aux_adv_ind.
     * - Retains advertising report information for subsequent aux_scan_rsp and aux_chain_ind.
     */
    struct lld_adv_rep_ind ext_adv_rep_info;
};

/// LLD SCAN event parameters structure
struct lld_scan_env_params
{
    /// Scanning events
    struct lld_scan_evt_params evt[NUM_SCAN_EVTS];

    /// Scheduling information for next AUX channel
    struct lld_calc_aux_rx_out aux_rx_out;

    /// Remaining slots in current scan window (in 625us BT slots)
    uint32_t win_rem_slots;

    /// Event timestamp of the last time the event priority was updated in half-slots (312.5 us)
    uint32_t last_prio_upd_ts;

#if LLD_SCAN_STRICT_INTERVAL
    /// Anchor timestamp  last window start (half-slots)
    uint32_t anchor_ts;
#endif // LLD_SCAN_STRICT_INTERVAL

    /// Scan interval in slots (625 us)
    uint16_t intv;

    /// Scan window in slots (625 us)
    uint16_t win;

    /// Scanning type (0 = PHY_1M, 1 = PHY_CODED)
    uint8_t type;

    /// Rate information of primary scan (@see enum lld_rate)
    uint8_t rate;
};

/// LLD SCAN environment structure
struct lld_scan_env_tag
{
    /// Scanning event parameters
    struct lld_scan_env_params *params[MAX_SCAN_PHYS];

    /// End of scan (half slots)
    uint32_t scan_end_ts;

    /// Bit field of active scanning PHYs (bit 0: 1M | bit 1: coded)
    uint8_t scan_phys;

    ///Activity Identifier
    uint8_t act_id;

    /// BD Address of the local device
    struct bd_addr own_addr;

    /// Local address type
    uint8_t own_addr_type;

    /// Scan filter policy
    uint8_t filter_policy;

    /// RAL resolution enable
    uint8_t ral_resol_en;

    /// Extended scanning
    bool ext_scan;
};

/// LLD SYNC environment structure
struct lld_scan_sync_env_tag
{
    ///  Activitiy Identifier
    uint8_t act_id;

    /// Filter Policy  (0x00 - Use parameters; 0x01 - Use periodic advertiser list)
    uint8_t filter_policy;

    /// Advertising SID indicated when starting sync without PAL
    uint8_t adv_sid;

    /// Advertising address type
    uint8_t adv_addr_type;

    /// Advertiser address
    struct bd_addr adv_addr;
};

#if (EAVESDROPPING_SUPPORT)
/// Segmented Scan parameters per PHY
struct lld_seg_params
{
    /// Time offset (in half slots)
    uint16_t        time_offset;
    /// Periodicity (in half slots)
    uint16_t        period;
    /// Duration of each segment (in half-us)
    uint16_t        duration;
    /// Segmenets per scan (in number of segments)
    uint16_t        segments;
    /// Half-us offset (in half-us)
    int16_t         hus_offset;

    /// Segmented scan state (@see enum SEG_SCAN_STATE)
    uint8_t         seg_scan_state;
    /// Segments counter
    uint16_t        seg_cnt;
    /// Timestamp of the first scan segment (in half slots)
    uint32_t        first_seg_ts;
};

/// LLD Segmented Scan environment structure
struct lld_scan_seg_env_tag
{
    /// BD address of a master, if an active slave connection is present, the scan timings are based on this piconet
    struct bd_addr bd_addr;

    /// Bit field of active scanning PHYs (bit 0: 1M | bit 1: coded)
    uint8_t scan_phys;

    /// Segmented Scan parameters per PHY
    struct lld_seg_params params[MAX_SCAN_PHYS];
};
#endif // (EAVESDROPPING_SUPPORT)

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD SCAN environment variable
__STATIC struct lld_scan_env_tag *lld_scan_env;

/// LLD SYNC environment variable
__STATIC struct lld_scan_sync_env_tag *lld_scan_sync_env;

#if (EAVESDROPPING_SUPPORT)
    /// LLD segmented scan environment variable
    __STATIC struct lld_scan_seg_env_tag *lld_scan_seg_env;
#endif // (EAVESDROPPING_SUPPORT)

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */
__STATIC bool lld_scan_sync_accept(uint8_t scan_id, uint8_t index_pkt, uint8_t addr_type, struct bd_addr peer_id_addr, uint8_t rx_adv_sid);
__STATIC bool lld_scan_sync_info_unpack(struct sync_info *p_syncinfo, uint16_t em_addr);
__STATIC void lld_scan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
__STATIC void lld_scan_trunc_ind(struct lld_scan_evt_params *scan_evt);

/**
 ****************************************************************************************
 * @brief End Scanning
 ****************************************************************************************
 */
__STATIC void lld_scan_end(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_END_FUNC_BIT);

    struct lld_scan_env_params *scan_par;
    struct lld_scan_evt_params *scan_evt;

    for (uint8_t scan_idx = 0; scan_idx < MAX_SCAN_PHYS; scan_idx++)
    {
        if (lld_scan_env->params[scan_idx] == NULL)
            continue;

        // Point to parameters
        scan_par = lld_scan_env->params[scan_idx];

        for (uint8_t evt_idx = 0; evt_idx < NUM_SCAN_EVTS; evt_idx++)
        {
            scan_evt = &scan_par->evt[evt_idx];

            if (SCAN_EVT_WAIT == scan_evt->state)
            {
                // Remove event
                sch_arb_remove(&(scan_evt->evt), false);
                // Unregister the scan window from scheduling parameters
                sch_slice_bg_remove(BLE_SCAN);

                // Terminate an eventual incomplete report chain
                if (RX_AUX_CHAIN_IND == scan_evt->aux_state)
                {
                    // Incomplete advertisement, generate a truncated report
                    lld_scan_trunc_ind(scan_evt);
                }

                // Event is ended
                scan_evt->state = SCAN_EVT_END;
            }
        }

        if ((scan_par->evt[SCAN_EVT_DFT].state == SCAN_EVT_END) && (scan_par->evt[SCAN_EVT_AUX].state == SCAN_EVT_END))
        {
            // Remove permission/status of CS as now unused
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(scan_par->evt[SCAN_EVT_DFT].cs_idx)), REG_EM_BLE_CS_SIZE, false, false, false);
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(scan_par->evt[SCAN_EVT_AUX].cs_idx)), REG_EM_BLE_CS_SIZE, false, false, false);

            // Free this scanning entity
            ke_free(lld_scan_env->params[scan_idx]);
            lld_scan_env->params[scan_idx] = NULL;
            lld_scan_env->scan_phys &= ~(1 << scan_idx);
        }
        else // an event is active
        {
            if (SCAN_EVT_ACTIVE == scan_par->evt[SCAN_EVT_DFT].state)
            {
                scan_evt = &scan_par->evt[SCAN_EVT_DFT];

                // Abort scan, and minimize maxevtime in case pre-fetch
                ble_rwblecntl_scan_abort_setf(1);
                // Minimize maxevtime in case pre-fetch
                em_ble_maxevtime_set(scan_evt->cs_idx, 1);
                // Indicate the entity needs to be terminated
                scan_evt->state = SCAN_EVT_TERM;
            }

            if (SCAN_EVT_ACTIVE == scan_par->evt[SCAN_EVT_AUX].state)
            {
                scan_evt = &scan_par->evt[SCAN_EVT_AUX];

                // Minimize maxevtime in case pre-fetch
                em_ble_maxevtime_set(scan_evt->cs_idx, 1);
                // Indicate the entity needs to be terminated
                scan_evt->state = SCAN_EVT_TERM;
            }
        }
    }

    // Check if no more scanning entity
    if (!lld_scan_env->scan_phys)
    {
        // Report scan end to LLM
        struct lld_scan_end_ind *ind = KE_MSG_ALLOC(LLD_SCAN_END_IND, TASK_LLM, TASK_NONE, lld_scan_end_ind);
        ind->act_id = lld_scan_env->act_id;
        ind->status = CO_ERROR_NO_ERROR;
        ke_msg_send(ind);

        // Free common event memory
        ke_free(lld_scan_env);
        lld_scan_env = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Evaluate AUX_PTR
 ****************************************************************************************
 */
__STATIC uint8_t lld_scan_aux_ptr_check(uint16_t rxauxptr, uint8_t rxdesc_idx, uint8_t scan_id)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_AUX_PTR_CHECK_FUNC_BIT, uint8_t, rxauxptr, rxdesc_idx, scan_id);
    uint32_t aux_data;
    uint8_t aux_phy;
    bool phy_support;

    uint8_t aux_offload = NO_AUX_OFFLOAD;

    em_rd((void *)&aux_data, rxauxptr, BLE_EXT_AUX_PTR_LEN);

    aux_phy = (aux_data & BLE_AUX_PHY_MASK) >> BLE_AUX_PHY_LSB;

    /* Check if PHY supported. AUX PHY will be same on all chained PDUs, so sufficient to just
       check on PHY support in the initial ADV_EXT_IND aux_data. */
    phy_support = ((BLE_PHY_1MBPS_SUPPORT) && (AUX_PHY_1MBPS == aux_phy))
                  || ((BLE_PHY_2MBPS_SUPPORT) && (AUX_PHY_2MBPS == aux_phy))
                  || ((BLE_PHY_CODED_SUPPORT) && (AUX_PHY_CODED == aux_phy));

    // Check if HW follow
    if (0 == em_ble_rxstatadv_followauxptr_getf(rxdesc_idx))
    {
        uint8_t evt_idx = GETB(scan_id, LLD_SCAN_ID_EVT);
        uint8_t phy_idx = GETB(scan_id, LLD_SCAN_ID_PHY);
        struct lld_scan_env_params *scan_par = lld_scan_env->params[phy_idx];
        struct lld_scan_evt_params *scan_evt = &scan_par->evt[evt_idx];

        /* Check if resource available */
        bool aux_av = ((evt_idx == SCAN_EVT_AUX) || (scan_par->evt[SCAN_EVT_AUX].state == SCAN_EVT_END));

#if (BLE_ADV_FRAG_NB_RX_MAX)
        if ((RX_AUX_CHAIN_IND == scan_evt->aux_state) && !(scan_evt->rx_chain_cnt_pkt < BLE_ADV_FRAG_NB_RX_MAX))
        {
            // No SW follow - The maximum length has been reached
        }
        else
#endif //(BLE_ADV_FRAG_NB_RX_MAX)
            if (phy_support && aux_av && lld_calc_aux_rx(&scan_par->aux_rx_out, rxdesc_idx, aux_data))
            {
                // Calculated AUX RX activity for Software reschedule
                aux_offload = SW_CNTL_AUX_OFFLOAD;
            }
    }
    else
    {
        if (phy_support)
        {
            // Indicate mode is HW follow
            aux_offload = HW_CNTL_AUX_OFFLOAD;
        }
    }

    return aux_offload;
}

/**
 ****************************************************************************************
 * @brief Process LE scan activity
 ****************************************************************************************
 */
__STATIC void lld_scan_process_pkt_rx(uint8_t scan_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_PROCESS_PKT_RX_FUNC_BIT, scan_id);

    uint8_t phy_idx = GETB(scan_id, LLD_SCAN_ID_PHY);
    uint8_t evt_idx = GETB(scan_id, LLD_SCAN_ID_EVT);

    struct lld_scan_env_params *scan_par = lld_scan_env->params[phy_idx];
    struct lld_scan_evt_params *scan_evt = &scan_par->evt[evt_idx];

    uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(lld_scan_env->act_id, 0); // txdesc for SCAN_REQ/AUX_SCAN_REQ

    // Process rx packet if done
    while (lld_rxdesc_check(scan_evt->cs_idx))
    {
        DBG_SWDIAG(LESCAN, PKT_RX, 1);

        uint16_t rxstat_pkt;
        uint16_t rxdataptr;
        uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;

        // Fetch Rx type
        uint8_t rxtype = em_ble_rxphadv_rxtype_getf(rxdesc_idx);

        // Fetch rx dataptr
        rxdataptr = em_ble_rxdataptr_get(rxdesc_idx);

        // Trace the current RX descriptor
        TRC_REQ_RX_DESC(LLD_SCAN, lld_scan_env->act_id, REG_EM_BLE_RX_DESC_ADDR_GET(lld_env.curr_rxdesc_index));

        // Check if chain should be flushed
        if (scan_evt->rx_chain_cnt_bytes > HOST_ADV_DATA_LEN_MAX)
        {
            // Free RX descriptor
            lld_rxdesc_free();
            continue;
        }

        // Retrieve RX status and type
        rxstat_pkt = em_ble_rxstatadv_get(rxdesc_idx);

        // Check if packet reception is correct
        if ((((rxstat_pkt & LLD_ADV_ERR_MASK) == 0) || ((rxstat_pkt & LLD_ADV_ERR_MASK) == EM_BLE_PRIV_ERR_BIT))
                // Workaround HW issue of not setting EM_BLE_LEN_ERR_BIT for 0 length ADV_DIRECT_IND
                && !((rxtype == BLE_ADV_DIRECT_IND) && (em_ble_rxphadv_rxadvlen_getf(rxdesc_idx) == 0)))
        {
            struct lld_adv_rep_ind *ind = &scan_evt->ext_adv_rep_info;
            uint8_t addr_type = ADDR_NONE;
            uint8_t tgta_offset = 0;
            uint8_t adva_offset = 0;
            uint8_t adv_data_offset = 0;
            uint8_t adv_data_len = 0;
            uint16_t rxaeheader = 0;

            // Flag that a packet has been received
            scan_evt->pkt_cnt++;

            ASSERT_ERR((rxtype < BLE_RESERVED_PDU_TYPE) && (rxtype != BLE_CONNECT_IND) && (rxtype != BLE_SCAN_REQ) && (rxtype != BLE_AUX_CONNECT_RSP));

            // Determine data length/offset for accessing RXDATAPTR based on Advertising Payload Format
            if (rxtype < BLE_ADV_EXT_IND)
            {
                struct bd_addr peer_id_addr;

                em_rd((void *)&peer_id_addr.addr[0], rxdataptr + adva_offset, BD_ADDR_LEN);

                if ((BLE_SCAN_RSP == rxtype) && (0 != memcmp(&peer_id_addr.addr[0], &ind->peer_id_addr.addr[0], BD_ADDR_LEN)))
                {
                    // Collision detected. Discard incomplete advertising report.
                    ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                }
                else
                {
                    // Clear history in adv report container of previously accumulated chain
                    uint8_t prev_evt_type = ind->evt_type;
                    memset(ind, 0, sizeof(scan_evt->ext_adv_rep_info));

                    // Determine the LE Advertising Report Event Type for legacy PDU
                    ind->evt_type = lld_scan_map_legacy_pdu_to_evt_type[rxtype];

                    if ((BLE_SCAN_RSP == rxtype) && (ADV_SCAN_IND == prev_evt_type))
                    {
                        ind->evt_type = SCAN_RSP_TO_ADV_SCAN_IND;
                    }

                    if (rxtype != BLE_ADV_DIRECT_IND)
                    {
                        adv_data_offset = BD_ADDR_LEN;
                        adv_data_len = em_ble_rxphadv_rxadvlen_getf(rxdesc_idx) - adv_data_offset;
                    }
                    else
                    {
                        ind->rx_rxadd = em_ble_rxphadv_rxrxadd_getf(rxdesc_idx);
                        tgta_offset = BD_ADDR_LEN;
                    }

                    // Set event parameters
                    ind->data_status = ADV_EVT_DATA_STATUS_COMPLETE;

                    ind->rate1 = CO_RATE_1MBPS;
                    ind->rate2 = CO_RATE_UNDEF; // n/a
                    ind->adi_present = false;
                    ind->tx_power = REP_ADV_DBM_UNKNOWN;

                    // Determine Peer address type (pre-RPA check)
                    ind->addr_type = addr_type = em_ble_rxphadv_rxtxadd_getf(rxdesc_idx);

                    if (addr_type != ADDR_NONE)
                    {
                        // Save received peer address
                        ind->peer_id_addr = peer_id_addr;
                    }
                }
            }
            else // Common Extended Advertising Payload Format
            {
                uint8_t rxaelength = 0;
                uint8_t rxaemode = 0;
                int i = 0;

                rxaelength = em_ble_rxaeheader_rxaelength_getf(rxdesc_idx);
                ASSERT_ERR(rxaelength < em_ble_rxphadv_rxadvlen_getf(rxdesc_idx));

                adva_offset =  0;
                adv_data_offset = (rxaelength) ? (rxaelength - BLE_EXT_ADV_HEADER_FLAGS_LEN) : (0);
                adv_data_len = em_ble_rxphadv_rxadvlen_getf(rxdesc_idx) - rxaelength - BLE_EXT_ADV_PRE_HEADER_LEN;

                scan_evt->aux_offload = NO_AUX_OFFLOAD;

                if (rxaelength > 0)
                    rxaeheader = em_ble_rxaeheader_get(rxdesc_idx);

                // Determine the LE Advertising Report Event Type for Ext Adv PDU
                switch (scan_evt->aux_state)
                {
                case RX_ADV_EXT_IND:
                {
                    // Clear history in adv report container of previously accumulated chain
                    memset(ind, 0, sizeof(scan_evt->ext_adv_rep_info));
                    scan_evt->rx_chain_cnt_bytes = 0;
                    scan_evt->rx_chain_cnt_pkt = 0;

                    // ADV_EXT_IND processed on the default event index
                    ASSERT_ERR(SCAN_EVT_DFT == evt_idx);

                    // Set initial event status
                    ind->data_status = ADV_EVT_DATA_STATUS_INCOMPLETE;

                    // Extract initial information about phy, aemode & aeheader

                    ind->rate1 = scan_par->rate;

                    rxaemode = em_ble_rxaeheader_rxaemode_getf(rxdesc_idx);

                    if (BLE_MODE_CONNECTABLE == rxaemode)
                    {
                        ind->evt_type |= CON_ADV_EVT_MSK;
                    }
                    else if (BLE_MODE_SCANNABLE == rxaemode)
                    {
                        ind->evt_type |= SCAN_ADV_EVT_MSK;
                    }

                    ind->adi_present = false;

                    if (rxaelength > 0)
                    {
                        if (rxaeheader & EM_BLE_RXADVA_BIT) // AdvA
                        {
                            // AdvA - reserved for future use in ADV_EXT_IND when the mode is connectable or scannable, and otherwise reserved for future
                            // use on the LE Coded PHY for NCNS with auxiliary packet (C1) - Ignore field in these cases.
                            if ((BLE_MODE_NON_CON_SCAN == rxaemode) && ((scan_par->rate == CO_RATE_1MBPS) || !(rxaeheader & EM_BLE_RXAUXPTR_BIT)))
                            {
                                // Determine Peer address type (pre-RPA check). Fetch peer address from dataptr later.
                                addr_type = em_ble_rxphadv_rxtxadd_getf(rxdesc_idx);

                                // Fetch received peer address
                                em_rd((void *)&ind->peer_id_addr.addr[0], rxdataptr + adva_offset, BD_ADDR_LEN);
                            }
                            else
                            {
                                rxaeheader &= ~EM_BLE_RXADVA_BIT; // clear bit - so effectively ignored elsewhere in driver.
                            }

                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXTGTA_BIT) // TargetA
                        {
                            // TargetA - reserved for future use in ADV_EXT_IND when the mode is connectable or scannable, and otherwise reserved for future
                            // use on the LE Coded PHY for NCNS with auxiliary packet (C1) - Ignore field in these cases.
                            if ((BLE_MODE_NON_CON_SCAN == rxaemode) && ((scan_par->rate == CO_RATE_1MBPS) || !(rxaeheader & EM_BLE_RXAUXPTR_BIT)))
                            {
                                // Identify event type as directed to this Target address.
                                ind->evt_type |= DIR_ADV_EVT_MSK;
                                ind->rx_rxadd = em_ble_rxphadv_rxrxadd_getf(rxdesc_idx);
                                tgta_offset = i;
                            }
                            else
                            {
                                rxaeheader &= ~EM_BLE_RXTGTA_BIT; // clear bit - so effectively ignored elsewhere in driver.
                            }

                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXCTE_BIT) // Supp Info
                        {
                            // CTE info - reserved for future use in ADV_EXT_IND - Ignore field when received
                            i += BLE_EXT_CTE_INFO_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXADI_BIT) // AdvDataInfo
                        {
                            // Check for ADI match
                            bool adi_match = true; //em_ble_rxstatadv_adi_match_getf(rxdesc_idx);

                            scan_evt->adi_data = em_rd16p(rxdataptr + i);
                            ind->adi = scan_evt->adi_data;
                            ind->adi_present = true;

                            // A scanner in create sync mode should disable ADI filtering so that it does not miss potential SyncInfo
                            if (!adi_match && (lld_scan_sync_env == NULL))
                            {
                                // Collision detected. Discard incomplete advertising report.
                                ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                                break;
                            }

                            i += BLE_EXT_ADI_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXAUXPTR_BIT) // AuxPtr
                        {
                            // Evaluate the aux pointer data, determine offload status
                            scan_evt->aux_offload = lld_scan_aux_ptr_check(rxdataptr + i, rxdesc_idx, scan_id);

                            if (NO_AUX_OFFLOAD == scan_evt->aux_offload)
                            {
                                // Phy not supported or not follow. Discard incomplete advertising report.
                                ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                                break;
                            }

                            scan_evt->aux_state = RX_AUX_ADV_IND;
                            i += BLE_EXT_AUX_PTR_LEN;
                        }
                        else
                        {
                            // No following Aux_Ptr - so advertising report complete with this PDU. Discard if no AdvA (mandatory).
                            ind->data_status = (ADDR_NONE == ind->addr_type) ? ADV_EVT_DATA_STATUS_DISCARDED : ADV_EVT_DATA_STATUS_COMPLETE;
                            ind->rate2 = CO_RATE_UNDEF; // n/a
                        }

                        if (rxaeheader & EM_BLE_RXSYNC_BIT) // SyncInfo
                        {
                            // SyncInfo Field - reserved for future use in ADV_EXT_IND - Ignore field when received
                            i += BLE_EXT_SYNC_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXPOW_BIT) // TxPower
                        {
                            // TxPower - reserved for future use on the LE Coded PHY, except for NCNS without auxiliary packet (C1) - Ignore field in these cases.
                            if ((scan_par->rate == CO_RATE_1MBPS) || ((BLE_MODE_NON_CON_SCAN == rxaemode) && !(rxaeheader & EM_BLE_RXAUXPTR_BIT)))
                            {
                                // Tx Power Level - save to advertising report.
                                ind->tx_power = em_rd8p(rxdataptr + i);
                            }

                            i += BLE_EXT_TX_PWR_LEN;
                        }
                        else
                        {
                            ind->tx_power = REP_ADV_DBM_UNKNOWN;
                        }

                        /* ACAD Field - reserved for future use in ADV_EXT_IND
                           Ignore field when received.*/
                    }
                    else
                    {
                        // No following Data - so advertising report will complete with this PDU
                        ind->data_status = ADV_EVT_DATA_STATUS_COMPLETE;
                    }

                    ind->addr_type = addr_type;

                    // In accordance with BT standard, there is no adv_data in ADV_EXT_IND
                    adv_data_len = 0;
                }
                break;

                case RX_AUX_ADV_IND:
                {
                    scan_evt->rx_chain_cnt_bytes = adv_data_len;

                    ind->rate2 = em_ble_rxchass_rate_getf(rxdesc_idx);

                    if (rxaelength > 0)
                    {
                        // Check if required to process syncInfo
                        bool create_sync = (lld_scan_sync_env != NULL);

                        if (rxaeheader & EM_BLE_RXADVA_BIT) // AdvA
                        {
                            // AdvA - This field is reserved for future use for non connectable non scannable if the corresponding field
                            // in the ADV_EXT_IND PDU was present (C4) - Ignore field in these cases.
                            if ((BLE_MODE_NON_CON_SCAN != rxaemode) || (ADDR_NONE == ind->addr_type))
                            {
                                // Determine Peer address type (pre-RPA check). Fetch peer address from dataptr later.
                                ind->addr_type = addr_type = em_ble_rxphadv_rxtxadd_getf(rxdesc_idx);

                                // Fetch received peer address
                                em_rd((void *)&ind->peer_id_addr.addr[0], rxdataptr + adva_offset, BD_ADDR_LEN);

                                // If WL enabled, must check IN_WHL status for AUX packets with AdvA
                                if (((SCAN_ALLOW_ADV_WLST == lld_scan_env->filter_policy) || ((SCAN_ALLOW_ADV_WLST_AND_INIT_RPA == lld_scan_env->filter_policy)
                                        && !(rxaeheader & EM_BLE_RXTGTA_BIT))) && (0 == em_ble_rxstatadv_in_whl_getf(rxdesc_idx)))
                                {
                                    ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                                }
                            }
                            else
                            {
                                rxaeheader &= ~EM_BLE_RXADVA_BIT; // clear bit - so effectively ignored elsewhere in driver.
                            }

                            i += BD_ADDR_LEN;
                        }
                        // Discard anonymous advertisements if are not reported
                        else if (((SCAN_ALLOW_ADV_WLST == lld_scan_env->filter_policy) || (SCAN_ALLOW_ADV_WLST_AND_INIT_RPA == lld_scan_env->filter_policy))
                                 && (ADDR_NONE == ind->addr_type) && ble_rwblecntl_anonymous_adv_filt_en_getf())
                        {
                            ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                        }

                        if (rxaeheader & EM_BLE_RXTGTA_BIT) // TargetA
                        {
                            // TargetA - reserved for future use if NCNS where TargetA already received in ADV_EXT_IND (C2) - Ignore field in this case
                            if (ind->evt_type != DIR_ADV_EVT_MSK)
                            {
                                // Identify event type as directed to this Target address.
                                ind->evt_type |= DIR_ADV_EVT_MSK;
                                ind->rx_rxadd = em_ble_rxphadv_rxrxadd_getf(rxdesc_idx);
                                tgta_offset = i;
                            }
                            else
                            {
                                rxaeheader &= ~EM_BLE_RXTGTA_BIT; // clear bit - so effectively ignored elsewhere in driver.
                            }

                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXCTE_BIT) // Supp Info
                        {
                            // CTE info - reserved for future use in AUX_ADV_IND - Ignore field when received
                            i += BLE_EXT_CTE_INFO_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXADI_BIT)// AdvDataInfo
                        {
                            // Shall have same value as the field in the PDU pointing to this PDU
                            uint16_t adi_data = em_rd16p(rxdataptr + i);

                            if (adi_data != scan_evt->adi_data)
                            {
                                // Collision detected. Discard incomplete advertising report & do not create sync.
                                ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;

                                create_sync = false;
                            }

                            i += BLE_EXT_ADI_LEN;
                        }
                        else
                        {
                            // ADI field is mandatory on AUX_ADV_IND, and on any AUX_CHAIN_IND following these (C3).
                            ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                            create_sync = false;
                            break;
                        }

                        if (ADV_EVT_DATA_STATUS_DISCARDED != ind->data_status)
                        {
                            if (rxaeheader & EM_BLE_RXAUXPTR_BIT) // AuxPtr
                            {
                                // Evaluate the aux pointer data, determine offload status
                                scan_evt->aux_offload = lld_scan_aux_ptr_check(rxdataptr + i, rxdesc_idx, scan_id);

                                if (NO_AUX_OFFLOAD == scan_evt->aux_offload)
                                {
                                    // No chained packet will follow, indicate as truncated.
                                    ind->data_status = ADV_EVT_DATA_STATUS_TRUNCATED;
                                }

                                scan_evt->aux_state = RX_AUX_CHAIN_IND;
                                i += BLE_EXT_AUX_PTR_LEN;
                            }
                            else if ((ind->evt_type & SCAN_ADV_EVT_MSK) && (SCAN_ACTIVE == scan_par->type))
                            {
                                // SCAN_RSP considered separate Data - so advertising report will indicate complete with this PDU
                                ind->data_status = ADV_EVT_DATA_STATUS_COMPLETE;

                                // Scannable packet received in active scan, so expect AUX_SCAN_RSP for data
                                scan_evt->aux_state = RX_AUX_SCAN_RSP;
                            }
                            else
                            {
                                // No following Data - so advertising report will complete with this PDU
                                ind->data_status = ADV_EVT_DATA_STATUS_COMPLETE;

                                // revert aux state to waiting for next ADV_EXT_IND
                                scan_evt->aux_state = RX_ADV_EXT_IND;
                            }
                        }
                        else // ADV_EVT_DATA_STATUS_DISCARDED
                        {
                            if (rxaeheader & EM_BLE_RXAUXPTR_BIT) // AuxPtr
                            {
                                i += BLE_EXT_AUX_PTR_LEN;
                            }
                        }

                        if (rxaeheader & EM_BLE_RXSYNC_BIT) // SyncInfo
                        {
                            uint8_t rx_adv_sid = GETF(ind->adi, BLE_ADI_SID);

                            // If attempting to create sync, validate AdvA, Type & sid
                            if (create_sync && lld_scan_sync_accept(scan_id, rxdesc_idx, ind->addr_type, ind->peer_id_addr, rx_adv_sid))
                            {
                                struct lld_sync_start_req *req_sync = KE_MSG_ALLOC(LLD_SYNC_START_REQ, TASK_LLM, TASK_NONE, lld_sync_start_req);

                                // unpack syncInfo field and indicate Periodic Advertising Rx SYNC should start
                                if (lld_scan_sync_info_unpack(&req_sync->syncinfo, rxdataptr + i))
                                {
                                    DBG_SWDIAG(LESCAN, SYNC_INFO, 1);

                                    int16_t fine_cnt;
                                    // Prepare relevant data and send for Periodic Advertising Rx SYNC start
                                    req_sync->act_id = lld_scan_sync_env->act_id;
                                    req_sync->rate = em_ble_rxchass_rate_getf(rxdesc_idx);
                                    req_sync->base_cnt = (em_ble_rxclknsync1_clknrxsync1_getf(rxdesc_idx) << 16)
                                                         | em_ble_rxclknsync0_clknrxsync0_getf(rxdesc_idx);
                                    fine_cnt = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(rxdesc_idx) - (2 * lld_exp_sync_pos_tab[req_sync->rate]);
                                    ASSERT_ERR((fine_cnt > (-2 * HALF_SLOT_SIZE)) && (fine_cnt < HALF_SLOT_SIZE));
                                    if (fine_cnt < 0)
                                    {
                                        uint8_t num_hs = CO_DIVIDE_CEIL((-fine_cnt), HALF_SLOT_SIZE);
                                        fine_cnt += num_hs * HALF_SLOT_SIZE;
                                        req_sync->base_cnt = CLK_SUB(req_sync->base_cnt, num_hs);
                                    }
                                    req_sync->fine_cnt = fine_cnt;

                                    req_sync->rxralptr = em_ble_rxralptr_getf(rxdesc_idx);

                                    req_sync->adv_sid = rx_adv_sid;

                                    req_sync->adv_addr_type = lld_scan_sync_env->adv_addr_type;
                                    memcpy(&req_sync->adv_addr.addr[0], &lld_scan_sync_env->adv_addr.addr[0], BD_ADDR_LEN);

                                    ke_msg_send(req_sync);

                                    // Free memory used for create sync and stop looking for syncinfo
                                    ke_free(lld_scan_sync_env);
                                    lld_scan_sync_env = NULL;

                                    DBG_SWDIAG(LESCAN, SYNC_INFO, 0);
                                }
                                else
                                {
                                    // Discard syncinfo due to rejected parameters
                                    ke_msg_free(ke_param2msg(req_sync));
                                }
                            }

                            // Extract interval for reporting
                            ind->interval = em_rd16p(rxdataptr + i + sizeof(uint16_t));

                            i += BLE_EXT_SYNC_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXPOW_BIT) // TxPower
                        {
                            // Tx Power Level - save to advertising report.
                            ind->tx_power = em_rd8p(rxdataptr + i);
                            i += BLE_EXT_TX_PWR_LEN;
                        }

                        /* ACAD Field present if i < rxaelength. However, ACAD in our standard IP
                           Ignore field when received.*/
                    }
                    else
                    {
                        // No AEHEADER => No ADI. ADI field is mandatory on AUX_ADV_IND, and on any AUX_CHAIN_IND following these (C3).
                        ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;

                        // revert aux state to waiting for next ADV_EXT_IND
                        scan_evt->aux_state = RX_ADV_EXT_IND;
                    }
                }
                break;

                case RX_AUX_SCAN_RSP:
                {
                    scan_evt->rx_chain_cnt_bytes = adv_data_len;

                    ind->data_status = ADV_EVT_DATA_STATUS_INCOMPLETE;

                    ind->evt_type |= SCAN_RSP_EVT_MSK;

                    if (rxaelength > 0)
                    {
                        if (rxaeheader & EM_BLE_RXADVA_BIT) // AdvA
                        {
                            // Determine Peer address type (pre-RPA check). Fetch peer address from dataptr later.
                            ind->addr_type = addr_type = em_ble_rxphadv_rxtxadd_getf(rxdesc_idx);

                            // If WL enabled, must check IN_WHL status for AUX packets with AdvA
                            if (((SCAN_ALLOW_ADV_WLST == lld_scan_env->filter_policy) || ((SCAN_ALLOW_ADV_WLST_AND_INIT_RPA == lld_scan_env->filter_policy)
                                    && !(rxaeheader & EM_BLE_RXTGTA_BIT))) && (0 == em_ble_rxstatadv_in_whl_getf(rxdesc_idx)))
                            {
                                ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                                break;
                            }

                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXTGTA_BIT) // TargetA
                        {
                            // TargetA - reserved for future use in AUX_SCAN_RSP - Ignore field when received
                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXCTE_BIT) // Supp Info
                        {
                            // CTE info - reserved for future use in AUX_SCAN_RSP - Ignore field when received
                            i += BLE_EXT_CTE_INFO_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXADI_BIT) // AdvDataInfo
                        {
                            // Shall have same value as the field in the PDU pointing to this PDU
                            uint16_t adi_data;

                            adi_data = em_rd16p(rxdataptr + i);
                            if (scan_evt->adi_data != adi_data)
                            {
                                // ADI mismatch. Discard the scan response.
                                ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                                break;
                            }

                            i += BLE_EXT_ADI_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXAUXPTR_BIT) // AuxPtr
                        {
                            // Evaluate the aux pointer data, determine offload status
                            scan_evt->aux_offload = lld_scan_aux_ptr_check(rxdataptr + i, rxdesc_idx, scan_id);

                            if (NO_AUX_OFFLOAD == scan_evt->aux_offload)
                            {
                                // No chained packet will follow, indicate as truncated.
                                ind->data_status = ADV_EVT_DATA_STATUS_TRUNCATED;
                            }

                            scan_evt->aux_state = RX_AUX_CHAIN_IND;
                            i += BLE_EXT_AUX_PTR_LEN;
                        }
                        else
                        {
                            // No following Data - so advertising report will complete with this PDU
                            ind->data_status = ADV_EVT_DATA_STATUS_COMPLETE;

                            // revert aux state to waiting for next ADV_EXT_IND
                            scan_evt->aux_state = RX_ADV_EXT_IND;
                        }

                        if (rxaeheader & EM_BLE_RXSYNC_BIT) // SyncInfo
                        {
                            // SyncInfo Field - reserved for future use in ADV_EXT_IND - Ignore field when received
                            i += BLE_EXT_SYNC_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXPOW_BIT) // TxPower
                        {
                            // Tx Power Level - save to advertising report.
                            ind->tx_power = em_rd8p(rxdataptr + i);
                            i += BLE_EXT_TX_PWR_LEN;
                        }

                        /* ACAD Field present if i < rxaelength. However, ACAD in our standard IP
                           Ignore field when received.*/
                    }
                    else
                    {
                        // No following Data - so advertising report will complete with this PDU
                        ind->data_status = ADV_EVT_DATA_STATUS_COMPLETE;

                        // revert aux state to waiting for next ADV_EXT_IND
                        scan_evt->aux_state = RX_ADV_EXT_IND;
                    }
                }
                break;

                case RX_AUX_CHAIN_IND:
                {
                    scan_evt->rx_chain_cnt_bytes += adv_data_len;
                    scan_evt->rx_chain_cnt_pkt++;

                    // Shall not exceed Host Advertising Data maximum - Truncate as required for Host.
                    if (scan_evt->rx_chain_cnt_bytes > HOST_ADV_DATA_LEN_MAX)
                    {
                        ind->data_status = ADV_EVT_DATA_STATUS_TRUNCATED;
                        adv_data_len -= (scan_evt->rx_chain_cnt_bytes - HOST_ADV_DATA_LEN_MAX);

                        scan_evt->aux_state = RX_ADV_EXT_IND;
                        break;
                    }

                    if (rxaelength > 0)
                    {
                        if (rxaeheader & EM_BLE_RXADVA_BIT) // AdvA
                        {
                            // AdvA - reserved for future use in AUX_CHAIN_IND - Ignore field when received.
                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXTGTA_BIT) // TargetA
                        {
                            // TargetA - reserved for future use in AUX_CHAIN_IND - Ignore field when received
                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXCTE_BIT) // Supp Info
                        {
                            // CTE info - reserved for future use in AUX_CHAIN_IND - Ignore field when received
                            i += BLE_EXT_CTE_INFO_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXADI_BIT)// AdvDataInfo
                        {
                            // Shall have same value as the field in the PDU pointing to this PDU
                            uint16_t adi_data;

                            adi_data = em_rd16p(rxdataptr + i);
                            if (scan_evt->adi_data != adi_data)
                            {
                                // Collision detected. Report advertisement as truncated & do not send new data.
                                ind->data_status = ADV_EVT_DATA_STATUS_TRUNCATED;
                                adv_data_len = 0;

                                scan_evt->aux_state = RX_ADV_EXT_IND;
                                break;
                            }

                            i += BLE_EXT_ADI_LEN;
                        }
                        else if (!(ind->evt_type & SCAN_RSP_EVT_MSK))
                        {
                            // ADI field is mandatory on AUX_ADV_IND, and on any AUX_CHAIN_IND following these (C3).
                            ind->data_status = ADV_EVT_DATA_STATUS_TRUNCATED;
                            adv_data_len = 0;
                            scan_evt->aux_state = RX_ADV_EXT_IND;
                            break;
                        }

                        if (rxaeheader & EM_BLE_RXAUXPTR_BIT) // AuxPtr
                        {
                            // Evaluate the aux pointer data, determine offload status
                            scan_evt->aux_offload = lld_scan_aux_ptr_check(rxdataptr + i, rxdesc_idx, scan_id);

                            if (NO_AUX_OFFLOAD == scan_evt->aux_offload)
                            {
                                // No chained packet will follow, indicate as truncated.
                                ind->data_status = ADV_EVT_DATA_STATUS_TRUNCATED;
                            }

                            scan_evt->aux_state = RX_AUX_CHAIN_IND;
                            i += BLE_EXT_AUX_PTR_LEN;
                        }
                        else
                        {
                            // No following Data - so advertising report will complete with this PDU
                            ind->data_status = ADV_EVT_DATA_STATUS_COMPLETE;

                            // revert aux state to waiting for next ADV_EXT_IND
                            scan_evt->aux_state = RX_ADV_EXT_IND;
                        }

                        if (rxaeheader & EM_BLE_RXSYNC_BIT) // SyncInfo
                        {
                            // SyncInfo Field - reserved for future use in ADV_EXT_IND - Ignore field when received
                            i += BLE_EXT_SYNC_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXPOW_BIT) // TxPower
                        {
                            // Tx Power Level - save to advertising report.
                            ind->tx_power = em_rd8p(rxdataptr + i);
                            i += BLE_EXT_TX_PWR_LEN;
                        }

                        /* ACAD Field - reserved for future use in AUX_CHAIN_IND
                           Ignore field when received.*/
                    }
                    else
                    {
                        // No AEHEADER => No ADI. ADI field is mandatory on AUX_ADV_IND, and on any AUX_CHAIN_IND following these (C3).
                        ind->data_status = (ind->evt_type & SCAN_RSP_EVT_MSK) ? ADV_EVT_DATA_STATUS_COMPLETE : ADV_EVT_DATA_STATUS_TRUNCATED;

                        // revert aux state to waiting for next ADV_EXT_IND
                        scan_evt->aux_state = RX_ADV_EXT_IND;
                    }
                }
                break;

                default:
                {
                    ASSERT_ERR_FORCE(0);
                }
                break;
                }
            }

            // If to be discarded for any reason, do not process further.
            if (ADV_EVT_DATA_STATUS_DISCARDED != ind->data_status)
            {
                do
                {
                    // Privacy Error on Directed Advertisements are conditionally discarded. Privacy Error on Undirected Advertisements always discarded.
                    if (((rxstat_pkt & EM_BLE_PRIV_ERR_BIT) != 0) && (!(ind->evt_type & DIR_ADV_EVT_MSK) || (lld_scan_env->filter_policy <= SCAN_ALLOW_ADV_WLST)))
                    {
                        ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                        break;
                    }

                    // Need to fetch the target address for directed advertising reports - which may be unresolved)
                    if ((ind->evt_type == ADV_DIRECT_IND) || (rxaeheader & EM_BLE_RXTGTA_BIT))
                    {
                        // Fetch target address
                        em_rd((void *)&ind->target_id_addr.addr[0], rxdataptr + tgta_offset, BD_ADDR_LEN);
                        ind->local_add_match = em_ble_rxstatadv_bdaddr_match_getf(rxdesc_idx);

                        // Discard if Directed Advertisements have BD Address mismatch using a Target Identity Address
                        if (!ind->local_add_match && ((ADDR_PUBLIC == ind->rx_rxadd) || ((ind->target_id_addr.addr[BD_ADDR_LEN - 1] & 0xC0) == RND_STATIC_ADDR)
                                                      || (lld_scan_env->filter_policy < SCAN_ALLOW_ADV_ALL_AND_INIT_RPA)))
                        {
                            ind->data_status = ADV_EVT_DATA_STATUS_DISCARDED;
                            break;
                        }
                    }

                    if (addr_type != ADDR_NONE)
                    {
                        // Fetch RAL related information for LLM
                        ind->peer_add_match = ((rxstat_pkt & EM_BLE_PEER_ADD_MATCH_BIT) != 0);

                        ind->rxralptr = em_ble_rxralptr_getf(rxdesc_idx);
                    }

                    // Indicate to host except if waiting on an AUX_ADV_IND
                    if (RX_AUX_ADV_IND != scan_evt->aux_state)
                    {
                        struct lld_adv_rep_ind *ind_msg = KE_MSG_ALLOC(LLD_ADV_REP_IND, TASK_LLM, TASK_NONE, lld_adv_rep_ind);

                        // Determine RSSI
                        ind->rssi = rwip_rf.rssi_convert(em_ble_rxchass_rssi_getf(rxdesc_idx)) - lld_env.rx_path_comp / 10;

                        // Identify chain ID (CHAIN 2 is set when reporting a packet received on primary scan entity while there is an ongoing SW offload entity)
                        ind->chain = ((evt_idx == SCAN_EVT_DFT) && (scan_par->evt[SCAN_EVT_AUX].state != SCAN_EVT_END)) ? 2 : 1;

                        // Copy retained info into new msg, and continue construction
                        memcpy(ind_msg, &scan_evt->ext_adv_rep_info, sizeof(struct lld_adv_rep_ind));

                        // Set data_len, with adv_data_len to copy
                        ind_msg->data_len = adv_data_len;

                        if (adv_data_len)
                        {
                            // Set dataptr to copy data from
                            ind_msg->em_buf = rxdataptr;
                            ind_msg->data_offset = adv_data_offset;

                            // Clear RX data pointer
                            em_ble_rxdataptr_setf(lld_env.curr_rxdesc_index, 0);
                        }

                        ind_msg->act_id = lld_scan_env->act_id;

                        ke_msg_send(ind_msg);

                        TRC_REQ_SCAN_RX_PDU(rxdesc_idx, ind_msg);
                    }

                }
                while (0);
            }

            // If PDU discarded, reset the aux state
            if (ADV_EVT_DATA_STATUS_DISCARDED == ind->data_status)
            {
                // Ensure a HW Follow chain is discarded also
                scan_evt->rx_chain_cnt_bytes = LLD_SCAN_RX_DATA_FLUSH;

                scan_evt->aux_state = RX_ADV_EXT_IND;
            }
        }

        // Free RX descriptor
        lld_rxdesc_free();

        DBG_SWDIAG(LESCAN, PKT_RX, 0);
    }

    if (em_ble_txcntl_txdone_getf(txdesc_idx))
    {
        // Release SCAN_REQ/AUX_SCAN_REQ descriptor
        em_ble_txcntl_txdone_setf(txdesc_idx, 0);
    }
}

/**
 ****************************************************************************************
 * @brief Generate truncated report
 ****************************************************************************************
 */
__STATIC void lld_scan_trunc_ind(struct lld_scan_evt_params *scan_evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_TRUNC_IND_FUNC_BIT, scan_evt);

    struct lld_adv_rep_ind *ind = KE_MSG_ALLOC(LLD_ADV_REP_IND, TASK_LLM, TASK_NONE, lld_adv_rep_ind);
    struct lld_scan_env_params *scan_par = lld_scan_env->params[scan_evt->index];

    // Copy retained info into new msg, and continue construction
    memcpy(ind, &scan_evt->ext_adv_rep_info, sizeof(struct lld_adv_rep_ind));

    // Identify chain ID (CHAIN 2 is set when reporting a packet received on primary scan entity while there is an ongoing SW offload entity)
    ind->chain = ((&scan_par->evt[SCAN_EVT_DFT] == scan_evt) && (scan_par->evt[SCAN_EVT_AUX].state != SCAN_EVT_END)) ? 2 : 1;

    ind->data_status = ADV_EVT_DATA_STATUS_TRUNCATED;
    ind->act_id = lld_scan_env->act_id;

    ke_msg_send(ind);
}

/**
 ****************************************************************************************
 * @brief Extract/unpack Syncinfo from EM.
 ****************************************************************************************
 */
__STATIC bool lld_scan_sync_info_unpack(struct sync_info *p_syncinfo, uint16_t em_addr)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_SYNC_INFO_UNPACK_FUNC_BIT, bool, p_syncinfo, em_addr);

    bool par_ok;

    // Extract sync_packet_offset (13 bits), offset_units (1 bit), rfu (2 bits)
    p_syncinfo->sync_offset = em_rd16p(em_addr);
    p_syncinfo->offset_units = GETB(p_syncinfo->sync_offset, BLE_SYNC_OFFSET_UNITS);
    p_syncinfo->sync_offset &= BLE_SYNC_OFFSET_MASK;
    em_addr += sizeof(uint16_t);

    // Extract interval (2 octets)
    p_syncinfo->interval = em_rd16p(em_addr);
    em_addr += sizeof(uint16_t);

    // Extract channel map (37 bits), SCA (3 bits)
    em_rd((void *)&p_syncinfo->ch_map, em_addr, LE_CH_MAP_LEN);
    p_syncinfo->sca = (p_syncinfo->ch_map.map[LE_CH_MAP_LEN - 1] & BLE_SYNC_SCA_MASK) >> BLE_SYNC_SCA_LSB;
    p_syncinfo->ch_map.map[LE_CH_MAP_LEN - 1] &= BLE_SYNC_CH_MAP_END_MASK;
    em_addr += LE_CH_MAP_LEN;

    // Extr act AA (4 octets)
    em_rd((void *)&p_syncinfo->aa, em_addr, ACCESS_ADDR_LEN);
    em_addr += sizeof(uint32_t);

    // Extract CRCinit (3 octets)
    em_rd((void *)&p_syncinfo->crcinit, em_addr, CRC_INIT_LEN);
    em_addr += CRC_INIT_LEN;

    // Extract event counter (2 octets)
    p_syncinfo->evt_counter = em_rd16p(em_addr);
    em_addr += sizeof(uint16_t);

    // Check paramaters of SyncInfo. A value of 0 for the Sync Packet Offset indicates that the time to the next AUX_SYNC_IND packet
    // is greater than can be represented. The interval shall not be less than the minimum (7.5ms)
    par_ok = ((p_syncinfo->sync_offset != PER_SYNC_OFFSET_UNSPECIFIED) && (p_syncinfo->interval >= PER_SYNC_INTERVAL_MIN));

    return par_ok;
}

/**
 ****************************************************************************************
 * @brief Check if valid device AdvA, & AdvType to create sync (SID checked separately)
 ****************************************************************************************
 */
__STATIC bool lld_scan_sync_accept(uint8_t scan_id, uint8_t index_pkt, uint8_t addr_type, struct bd_addr rx_adva, uint8_t rx_adv_sid)
{
    uint8_t sync_accept = false;

    if (PER_SYNC_FILT_USE_PAL == lld_scan_sync_env->filter_policy)
    {
        struct lld_scan_env_params *scan_par = lld_scan_env->params[GETB(scan_id, LLD_SCAN_ID_PHY)];
        struct lld_scan_evt_params *scan_evt = &scan_par->evt[GETB(scan_id, LLD_SCAN_ID_EVT)];

        if ((scan_evt->peradv_filt_en) && em_ble_rxstatadv_in_peradval_getf(index_pkt))
        {
            uint8_t pal_idx = (em_ble_rxwpalptr_get(index_pkt) - (EM_BLE_WPAL_OFFSET)) / (REG_EM_BLE_WPAL_SIZE);

            // Store to lld_scan_sync_env for device to use & validation with SID
            memcpy((void *)&lld_scan_sync_env->adv_addr.addr[0], &rx_adva.addr[0], BD_ADDR_LEN);
            lld_scan_sync_env->adv_addr_type = addr_type;

            // Confirm SID:
            sync_accept = ((lld_env.adv_sids[pal_idx] & (1 << rx_adv_sid)) != 0);
        }
    }
    else // PER_SYNC_FILT_IGNORE_PAL - Check explicit parameters
    {
        // If RAL resolution enabled and found in RAL
        if ((lld_scan_env->ral_resol_en) && (0 != em_ble_rxralptr_getf(index_pkt)))
        {
            struct bd_addr peer_id_addr;

            uint8_t pos = (em_ble_rxralptr_getf(index_pkt) - EM_BLE_RAL_OFFSET) / REG_EM_BLE_RAL_SIZE;
            // Do a burst read in the exchange memory
            em_rd(&peer_id_addr, REG_EM_BLE_RAL_ADDR_GET(pos) + EM_BLE_RAL_PEER_ID_INDEX * 2, sizeof(struct bd_addr));
            addr_type = em_ble_ral_info_peer_id_type_getf(pos) & ADDR_MASK;

            // Explicitly check Type, AdvA and SID with Create Sync Parameters.
            sync_accept = ((addr_type == lld_scan_sync_env->adv_addr_type) && !memcmp(&peer_id_addr.addr[0], &lld_scan_sync_env->adv_addr.addr[0], BD_ADDR_LEN)
                           && (lld_scan_sync_env->adv_sid == rx_adv_sid));

            if (sync_accept)
            {
                // Address resolution enabled, and found in the RAL
                memcpy((void *)&lld_scan_sync_env->adv_addr.addr[0], &rx_adva.addr[0], BD_ADDR_LEN);
                lld_scan_sync_env->adv_addr_type = ADDR_RAND;
            }
        }
        else
        {
            // Explicitly check Type, AdvA and SID with Create Sync Parameters.
            sync_accept = ((addr_type == lld_scan_sync_env->adv_addr_type) && !memcmp(&rx_adva.addr[0], &lld_scan_sync_env->adv_addr.addr[0], BD_ADDR_LEN)
                           && (lld_scan_sync_env->adv_sid == rx_adv_sid));
        }
    }

    return sync_accept;
}


/**
 ****************************************************************************************
 * @brief Schedule aux LE scan activity
 ****************************************************************************************
 */
__STATIC void lld_scan_aux_sched(uint8_t scan_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_AUX_SCHED_FUNC_BIT, scan_id);

    uint8_t phy_idx = GETB(scan_id, LLD_SCAN_ID_PHY);
    uint8_t evt_idx = GETB(scan_id, LLD_SCAN_ID_EVT);

    // Point to parameters (Parent event)
    struct lld_scan_env_tag *scan_env = lld_scan_env;
    struct lld_scan_env_params *scan_par = scan_env->params[phy_idx];
    struct lld_scan_evt_params *scan_evt = &scan_par->evt[evt_idx];

    // Point to parameters (Aux event)
    struct lld_scan_evt_params *aux_evt = &scan_par->evt[SCAN_EVT_AUX];
    struct sch_arb_elt_tag *evt = &(aux_evt->evt);

    // If offloading from default event, copy all context information over for auxiliary event
    if (SCAN_EVT_DFT == evt_idx)
    {
        // copy packet chain information
        aux_evt->rx_chain_cnt_bytes = scan_evt->rx_chain_cnt_bytes;
        aux_evt->rx_chain_cnt_pkt = scan_evt->rx_chain_cnt_pkt;
        aux_evt->aux_state = scan_evt->aux_state;
        aux_evt->adi_data = scan_evt->adi_data;

        // copy over retained advertising report information
        memcpy(&aux_evt->ext_adv_rep_info, &scan_evt->ext_adv_rep_info, sizeof(struct lld_adv_rep_ind));

        // copy all extadv HW status for the offload sequence
        em_ble_extadvstat_set(aux_evt->cs_idx, em_ble_extadvstat_get(scan_evt->cs_idx));

        // Set the aux state to Wait
        aux_evt->state = SCAN_EVT_WAIT;
    }

    evt->time.hs      = scan_par->aux_rx_out.time.hs;
    evt->time.hus     = scan_par->aux_rx_out.time.hus;
    evt->duration_min = ((lld_scan_max_aux_dur_tab[(scan_par->aux_rx_out.rate)] + scan_par->aux_rx_out.sync_win_size_us) << 1) + BLE_RESERVATION_TIME_MARGIN_HUS;

    evt->current_prio = co_max(rwip_priority[RWIP_PRIO_AUX_RX_IDX].value, scan_evt->evt.current_prio);

    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
    {
        uint8_t cs_idx = aux_evt->cs_idx;

        uint8_t max_evt_slots;
        uint8_t aux_rate = scan_par->aux_rx_out.rate;
        uint32_t aux_sync_win_size_us = scan_par->aux_rx_out.sync_win_size_us;

        // Set Rx/Tx threshold + rate
        em_ble_thrcntl_ratecntl_aux_rate_setf(cs_idx, aux_rate);

        // Set event time limit for offload: sync_win + aux_adv_ind(max) + t_ifs + aux_scan_req + t_ifs + aux_scan_rsp(max) (not used in HW follow)
        max_evt_slots = (lld_scan_max_aux_dur_tab[aux_rate] + aux_sync_win_size_us + (SLOT_SIZE - 1)) / SLOT_SIZE;

        em_ble_maxevtime_set(cs_idx, max_evt_slots);

        // Update the max AUX CHAIN Rx Byte
        em_ble_rxmaxauxchain_pack(cs_idx, /*maxrxchdesc*/ BLE_ADV_FRAG_NB_RX_MAX - aux_evt->rx_chain_cnt_pkt, /*maxrxchbyte*/ HOST_ADV_DATA_LEN_MAX - aux_evt->rx_chain_cnt_bytes);

        // Set Wide-open mode, Size of the Rx window in slots
        if (aux_sync_win_size_us > EM_BLE_CS_RXWINSZ_MAX)
        {
            // Size of the Rx half window in half-slots (wide-open mode)
            uint8_t sync_win_slots = (aux_sync_win_size_us + (SLOT_SIZE - 1)) / SLOT_SIZE;

            em_ble_rxwincntl_pack(cs_idx, 1, sync_win_slots);
        }
        else
        {
            // Size of the Rx half window in us (normal mode)
            em_ble_rxwincntl_pack(cs_idx, 0, (aux_sync_win_size_us + 1) >> 1);
        }

        // set Channel Index for Aux.
        em_ble_chmap2_ch_aux_setf(cs_idx, scan_par->aux_rx_out.ch_idx);

        if (RX_AUX_ADV_IND == aux_evt->aux_state)
        {
            // If AdvA already processed, clear Filter policy configurations
            if (aux_evt->ext_adv_rep_info.addr_type == ADDR_NONE)
            {
                // set Filter policy configuration
                em_ble_filtpol_ralcntl_pack(cs_idx, scan_env->filter_policy, /*ralresolen*/ scan_env->ral_resol_en,
                                            /*peradvfilten*/ aux_evt->peradv_filt_en,
                                            /*local_rpa_sel*/((scan_env->own_addr_type & ADDR_RPA_MASK) != 0), /*ralmode*/0,
                                            /*ral_en*/(scan_env->ral_resol_en || ((scan_env->own_addr_type & ADDR_RPA_MASK) != 0)));
            }
            else
            {
                // If AdvA already processed, clear Filter policy configurations
                em_ble_filtpol_ralcntl_pack(cs_idx, 0, 0, 0, 0, 0, 0);
            }

            em_ble_cntl_format_setf(cs_idx, (SCAN_ACTIVE == scan_par->type) ? EM_BLE_CS_FMT_EXT_ACTIVE_SCAN : EM_BLE_CS_FMT_EXT_PASSIVE_SCAN);
        }
        else
        {
            // clear Filter policy configurations for chain sequences
            em_ble_filtpol_ralcntl_pack(cs_idx, 0, 0, 0, 0, 0, 0);

            // Remove active scan for chain sequences
            em_ble_cntl_format_setf(cs_idx, EM_BLE_CS_FMT_EXT_PASSIVE_SCAN);
        }

        aux_evt->aux_offload = SW_CNTL_AUX_OFFLOAD;
        aux_evt->pkt_cnt = 0;
    }
    else
    {
        if (RX_AUX_CHAIN_IND == aux_evt->aux_state)
        {
            // Unable to schedule chained data, generate a truncated report
            lld_scan_trunc_ind(scan_evt);
        }

        // Event is ended
        aux_evt->state = SCAN_EVT_END;
    }
}

/**
 ****************************************************************************************
 * @brief Schedule next LE scan activity
 ****************************************************************************************
 */
__STATIC void lld_scan_sched(uint8_t phy_idx, uint32_t timestamp, bool resched)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_SCHED_FUNC_BIT, phy_idx, timestamp, resched);

    uint8_t evt_idx = SCAN_EVT_DFT;

    struct lld_scan_env_tag *scan_env = lld_scan_env;
    struct lld_scan_env_params *scan_par = scan_env->params[phy_idx];

    struct lld_scan_evt_params *scan_evt = &scan_par->evt[evt_idx];
    struct sch_arb_elt_tag *evt = &(scan_evt->evt);

    uint8_t cs_idx = scan_evt->cs_idx;

    uint32_t clock = lld_read_clock();
#if LLD_SCAN_STRICT_INTERVAL
    int32_t effective_slots = (CLK_SUB(clock, scan_par->anchor_ts) + 1) / 2;
#else // !LLD_SCAN_STRICT_INTERVAL
    int32_t effective_slots = (CLK_SUB(clock, timestamp) + 1) / 2;
#endif  // !LLD_SCAN_STRICT_INTERVAL

#if LLD_SCAN_STRICT_INTERVAL
    if (effective_slots < scan_par->win)
    {
        // Subtract the number of effective scanning slots since anchor
        scan_par->win_rem_slots = scan_par->win - effective_slots;
    }
#else // !LLD_SCAN_STRICT_INTERVAL
    if (effective_slots < scan_par->win_rem_slots)
    {
        // Subtract the number of effective scanning slots
        scan_par->win_rem_slots -= effective_slots;
    }
#endif  // !LLD_SCAN_STRICT_INTERVAL
    else
    {
        uint32_t win_rem_slots = scan_par->win;

        // Unregister the scan window from scheduling parameters
        sch_slice_bg_remove(BLE_SCAN);

#if LLD_SCAN_STRICT_INTERVAL
        // Add scan window period - maintain a consistent interval
        scan_par->anchor_ts = CLK_ADD_2(scan_par->anchor_ts, 2 * (scan_par->intv * (1 + ((effective_slots - scan_par->win) / scan_par->intv))));
        evt->time.hs = scan_par->anchor_ts;
        if (CLK_DIFF(clock, scan_par->anchor_ts) < 0)
        {
            win_rem_slots -= (CLK_SUB(clock, scan_par->anchor_ts) + 1) / 2;
            evt->time.hs = clock;
        }
#else // !LLD_SCAN_STRICT_INTERVAL
        // Add scan window period delay adjusted
        evt->time.hs = CO_ALIGN2_LO(CLK_ADD_2(clock, 2 * (scan_par->intv - scan_par->win)));
#endif // !LLD_SCAN_STRICT_INTERVAL

        // Reset delay to slot boundary
        evt->time.hus = 0;

        // Reload window length
        scan_par->win_rem_slots = win_rem_slots;
    }

#if LLD_SCAN_STRICT_INTERVAL
    // Allow short scan durations in heavily contested bandwidth, with win_rem_slots determined on HW arbitration
    evt->duration_min = 2 * HALF_SLOT_SIZE;
#else // !LLD_SCAN_STRICT_INTERVAL
    evt->duration_min = co_min((2 * scan_par->win_rem_slots * SLOT_SIZE), sch_slice_params.scan_evt_dur);
#endif // !LLD_SCAN_STRICT_INTERVAL

    // Check if scan has reached its deadline
    if ((scan_env->scan_end_ts <= RWIP_MAX_CLOCK_TIME) && (CLK_LOWER_EQ(scan_env->scan_end_ts, evt->time.hs)))
    {
        // The scan is over
        lld_scan_end();
    }
    else
    {
#if !LLD_SCAN_STRICT_INTERVAL
        // Reset priority on non-abort, or if abort expected from HW arbitration
        if (!resched)
#endif // !LLD_SCAN_STRICT_INTERVAL
        {
            // Reset the current priority
            evt->current_prio = rwip_priority[RWIP_PRIO_SCAN_IDX].value;
            // Save the timestamp
            scan_par->last_prio_upd_ts = evt->time.hs;
        }

        scan_evt->aux_offload = NO_AUX_OFFLOAD;

        em_ble_maxevtime_set(cs_idx, scan_par->win_rem_slots);

        // set Wide-open mode, Size of the Rx half-window in half-slots
        em_ble_rxwincntl_pack(cs_idx, 1, co_min(scan_par->win_rem_slots, EM_BLE_CS_RXWINSZ_MAX));

        // initialize all extadv status as start of new sequence
        em_ble_extadvstat_pack(cs_idx, 0, 0, 0, 0, 0);

        // Set the max AUX CHAIN Rx Byte and Rx Desc
        em_ble_rxmaxauxchain_pack(cs_idx, /*maxrxchdesc*/ BLE_ADV_FRAG_NB_RX_MAX, /*maxrxchbyte*/ HOST_ADV_DATA_LEN_MAX);

#if !(BLE_HOST_PRESENT) // BLE HOST workaround allows random address update on active scans
        // random address may have changed on scan parameters update
        if (scan_env->own_addr_type & ADDR_RPA_MASK)
#endif // !(BLE_HOST_PRESENT)
        {
            // Set the Device identity (BD Address)
            em_ble_lebdaddr_set(cs_idx, 0, co_read16p(&scan_env->own_addr.addr[0]));
            em_ble_lebdaddr_set(cs_idx, 1, co_read16p(&scan_env->own_addr.addr[2]));
            em_ble_lebdaddr_set(cs_idx, 2, co_read16p(&scan_env->own_addr.addr[4]));
        }

        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            scan_evt->pkt_cnt = 0;
            // Reset aggregate data length
            scan_evt->rx_chain_cnt_bytes = 0;
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }
    }
}

#if (EAVESDROPPING_SUPPORT)
/**
 ****************************************************************************************
 * @brief Set timestamp and offset of the next scan segment
 ****************************************************************************************
 */
__STATIC void lld_scan_set_seg_timings(uint8_t phy_idx, struct sch_arb_elt_tag *evt)
{
    // Point to parameters
    struct lld_seg_params *seg_params = &lld_scan_seg_env->params[phy_idx];

    uint32_t clk_offset = 0;
    uint16_t bit_offset = 0;
    int16_t diff = 0;
    uint32_t timestamp_piconet = evt->time.hs;

    // Check if the BD address corresponds to an active connection
    uint8_t link_id = lm_find_link_id(lld_scan_seg_env->bd_addr);

    DBG_SWDIAG(SEG_LESCAN, SET_TIME, 1);

    if (link_id < MAX_NB_ACTIVE_ACL && (SLAVE_ROLE == ld_acl_role_get(link_id)))
    {
        // Get the timings of the link
        ld_acl_timing_info_get(link_id, &clk_offset, &bit_offset);
        timestamp_piconet = CLK_ADD_2(timestamp_piconet, clk_offset);
    }

    diff = (seg_params->time_offset) - (timestamp_piconet % seg_params->period);

    // Apply the microsecond alignment
    diff += seg_params->hus_offset / HALF_SLOT_SIZE;
    int16_t delay = bit_offset + ((seg_params->hus_offset) % HALF_SLOT_SIZE);

    // Adjust delay and timestamp if needed
    if (delay < 0)
    {
        delay += HALF_SLOT_SIZE;
        diff--;
    }
    else if (delay >= HALF_SLOT_SIZE)
    {
        delay -= HALF_SLOT_SIZE;
        diff++;
    }

    // Adjust delay forward if negative
    if (diff < 0)
    {
        diff = seg_params->period - CO_MOD((-diff), seg_params->period);
    }

    evt->time.hs = CLK_ADD_2(evt->time.hs, diff);
    evt->time.hus = (uint16_t)delay;

    DBG_SWDIAG(SEG_LESCAN, SET_TIME, 0);
}

/**
 ****************************************************************************************
 * @brief Schedule next LE segmented scan activity
 ****************************************************************************************
 */
__STATIC void lld_scan_seg_sched(uint8_t phy_idx, uint32_t timestamp, bool resched)
{
    // Point to parameters
    struct lld_scan_env_tag *scan_env = lld_scan_env;
    struct lld_scan_env_params *scan_par = scan_env->params[phy_idx];
    struct lld_seg_params *seg_params = &lld_scan_seg_env->params[phy_idx];
    struct lld_scan_evt_params *scan_evt = &scan_par->evt[SCAN_EVT_DFT];
    struct sch_arb_elt_tag *evt = &(scan_evt->evt);

    uint8_t cs_idx = scan_evt->cs_idx;
    bool completed = false;

    DBG_SWDIAG(SEG_LESCAN, SCHED, 1);

    switch (seg_params->seg_scan_state)
    {
    case SEG_SCAN_ENABLING:
    {
        DBG_SWDIAG(SEG_LESCAN, ENABLING, 1);

        evt->time.hs = rwip_time_get().hs;

        // Set the first segment timings
        lld_scan_set_seg_timings(phy_idx, evt);

        seg_params->first_seg_ts = evt->time.hs;

        // Set the event as strictly periodic
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, 0);
        // Set the event duration
        evt->duration_min = seg_params->duration;

        scan_par->win_rem_slots = CO_DIVIDE_CEIL(seg_params->duration, 2 * HALF_SLOT_SIZE);
        // Set max event time
        em_ble_maxevtime_set(cs_idx, scan_par->win_rem_slots);

        // set Normal mode for precision control, Size of the Rx half-window in usecs.
        // for a precise receive duration, deduct Preamble and Access Address size otherwise included
        em_ble_rxwincntl_pack(cs_idx, 0, (seg_params->duration >> 2) - (lld_scan_preamble_access_addr_tab[scan_evt->index] >> 1));

        // Initialize segment counter
        seg_params->seg_cnt = 0;

        // Set as active
        seg_params->seg_scan_state = SEG_SCAN_ON;

        DBG_SWDIAG(SEG_LESCAN, ENABLING, 0);

    } // No break

    case SEG_SCAN_ON:
    {
        // Unregister the scan window from scheduling parameters
        sch_slice_bg_remove(BLE_SCAN);

        DBG_SWDIAG(SEG_LESCAN, RESCHED, (resched ? 1 : 0));

        // Reset priority on non-abort, or if abort expected from HW arbitration
        if (!resched)
        {
            // Reset the current priority
            evt->current_prio = rwip_priority[RWIP_PRIO_SCAN_IDX].value;
            // Save the timestamp
            scan_par->last_prio_upd_ts = evt->time.hs;
        }

        // initialize all extadv status as start of new sequence
        em_ble_extadvstat_pack(cs_idx, 0, 0, 0, 0, 0);
        scan_evt->aux_offload = NO_AUX_OFFLOAD;

        // random address may have changed on scan parameters update
        if (scan_env->own_addr_type & ADDR_RPA_MASK)
        {
            // Set the Device identity (BD Address)
            em_ble_lebdaddr_set(cs_idx, 0, co_read16p(&scan_env->own_addr.addr[0]));
            em_ble_lebdaddr_set(cs_idx, 1, co_read16p(&scan_env->own_addr.addr[2]));
            em_ble_lebdaddr_set(cs_idx, 2, co_read16p(&scan_env->own_addr.addr[4]));
        }

        // Schedule the next window
        do
        {
            if (seg_params->seg_cnt == 0)
            {
                // Go to next scan interval
                seg_params->first_seg_ts = CLK_ADD_2(seg_params->first_seg_ts, (scan_par->intv << 1));
                evt->time.hs = seg_params->first_seg_ts;
                // Align to next segment timings
                lld_scan_set_seg_timings(phy_idx, evt);
            }
            else
            {
                // Go to next segment
                evt->time.hs = CLK_ADD_2(evt->time.hs, seg_params->period);
            }

            // Increment segment counter
            CO_VAL_INC(seg_params->seg_cnt, seg_params->segments);

            DBG_SWDIAG(SEG_LESCAN, RESCHED, 0);

            // Check if scan has reached its deadline
            if ((scan_env->scan_end_ts <= RWIP_MAX_CLOCK_TIME) && (CLK_LOWER_EQ(scan_env->scan_end_ts, evt->time.hs)))
            {
                DBG_SWDIAG(SEG_LESCAN, STOP, 1);

                // The scan is over
                lld_scan_end();

                completed = true;

                DBG_SWDIAG(SEG_LESCAN, STOP, 0);
            }
            else
            {
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                {
                    // Set state
                    scan_evt->state = SCAN_EVT_WAIT;

                    // Reset packet count & aggregate data length
                    scan_evt->pkt_cnt = 0;
                    scan_evt->rx_chain_cnt_bytes = 0;

                    completed = true;
                }
                else
                {
                    DBG_SWDIAG(SEG_LESCAN, RESCHED, 1);

                    // Increment priority, and retry for next segment
                    evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_SCAN_IDX));
                    // Save the timestamp
                    scan_par->last_prio_upd_ts = evt->time.hs;
                }
            }
        }
        while (!completed);
    }
    break;

    default:
    {
        ASSERT_INFO_FORCE(0, seg_params->seg_scan_state, 0);
    }
    break;
    }

    DBG_SWDIAG(SEG_LESCAN, SCHED, 0);
}
#endif // (EAVESDROPPING_SUPPORT)

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_scan_frm_eof_isr(uint8_t scan_id, uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_FRM_EOF_ISR_FUNC_BIT, scan_id, timestamp, abort);

    DBG_SWDIAG(LESCAN, FRM_EOF_ISR, 1);

    uint8_t phy_idx = GETB(scan_id, LLD_SCAN_ID_PHY);
    uint8_t evt_idx = GETB(scan_id, LLD_SCAN_ID_EVT);

    if ((lld_scan_env != NULL) && (lld_scan_env->params[phy_idx] != NULL))
    {
        // Point to parameters
        struct lld_scan_env_params *scan_par = lld_scan_env->params[phy_idx];
        struct lld_scan_evt_params *scan_evt = &scan_par->evt[evt_idx];

        // Check if scan is under termination
        bool end = (scan_evt->state == SCAN_EVT_TERM);

        // Set the state back to Wait, as the frame is completed
        scan_evt->state = SCAN_EVT_WAIT;

        // If scan is under termination
        if (end)
        {
            // Flush potentially consumed descriptor(s)
            while (lld_rxdesc_check(scan_evt->cs_idx))
            {
                // Free RX descriptor
                lld_rxdesc_free();
            }

            // Close the driver
            lld_scan_end();
        }
        else
        {
            bool rx_err = false;

            // Remove event
            sch_arb_remove(&(scan_evt->evt), true);

            // Check for T_MAFS error prior to Process packet rx
            if (em_ble_txrxcntl_rxmafserr_getf(scan_evt->cs_idx))
            {
                em_ble_txrxcntl_rxmafserr_setf(scan_evt->cs_idx, 0);

                /*If the Controller does not listen for or does not receive the AUX_ADV_IND PDU, no report shall be  generated.*/
                if (RX_AUX_CHAIN_IND == scan_evt->aux_state)
                {
                    // Incomplete advertisement, generate a truncated report
                    lld_scan_trunc_ind(scan_evt);
                }

                // Flush potentially consumed descriptor(s)
                while (lld_rxdesc_check(scan_evt->cs_idx))
                {
                    // Free RX descriptor
                    lld_rxdesc_free();
                }

                rx_err = true;
            }
            else
            {
                // Process packet rx - handle the remaining descriptors (below threshold)
                lld_scan_process_pkt_rx(scan_id);

                // Check for processing status after Process packet rx
                if ((HW_CNTL_AUX_OFFLOAD == scan_evt->aux_offload) || ((0 == scan_evt->pkt_cnt) && (SW_CNTL_AUX_OFFLOAD == scan_evt->aux_offload)))
                {
                    /*If the Controller does not listen for or does not receive the AUX_ADV_IND PDU, no report shall be  generated.*/
                    if (RX_AUX_CHAIN_IND == scan_evt->aux_state)
                    {
                        // Incomplete advertisement, generate a truncated report
                        lld_scan_trunc_ind(scan_evt);
                    }

                    rx_err = true;
                }

                if (RX_AUX_SCAN_RSP == scan_evt->aux_state)
                {
                    // If have not received SCAN_RSP at end of event, then reset the aux state
                    rx_err = true;
                }

                // Schedule a SW Offload event if required
                if (!rx_err && (SW_CNTL_AUX_OFFLOAD == scan_evt->aux_offload))
                {
                    lld_scan_aux_sched(scan_id);
                }
            }

            if (SCAN_EVT_DFT == evt_idx)
            {
                scan_evt->aux_offload = NO_AUX_OFFLOAD;
                scan_evt->aux_state = RX_ADV_EXT_IND;

#if (EAVESDROPPING_SUPPORT)
                // If segmented scan enabled for this PHY
                if (LLD_SCAN_SEG_EN(phy_idx))
                {
                    // Schedule next segmented scan
                    lld_scan_seg_sched(phy_idx, timestamp, abort);
                }
                else
#endif // (EAVESDROPPING_SUPPORT)
                {
                    // Schedule next scan
                    lld_scan_sched(phy_idx, timestamp, abort);
                }
            }
            else if ((rx_err) || (NO_AUX_OFFLOAD == scan_evt->aux_offload)) // End the aux event If rx errors, or complete.
            {
                scan_evt->state = SCAN_EVT_END;
            }
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, scan_id, lld_scan_env);
    }

    DBG_SWDIAG(LESCAN, FRM_EOF_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle RX interrupt
 ****************************************************************************************
 */
__STATIC void lld_scan_frm_rx_isr(uint8_t scan_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_FRM_RX_ISR_FUNC_BIT, scan_id);

    uint8_t phy_idx = GETB(scan_id, LLD_SCAN_ID_PHY);

    if ((lld_scan_env != NULL) && (lld_scan_env->params[phy_idx] != NULL))
    {
        // Process packet rx - handle received descriptors (threshold reached)
        lld_scan_process_pkt_rx(scan_id);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle skip interrupt
 ****************************************************************************************
 */
__STATIC void lld_scan_frm_skip_isr(uint8_t scan_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_FRM_SKIP_ISR_FUNC_BIT, scan_id);

    DBG_SWDIAG(LESCAN, EVT_CANCELED, 1);

    uint8_t phy_idx = GETB(scan_id, LLD_SCAN_ID_PHY);

    if ((lld_scan_env != NULL) && (lld_scan_env->params[phy_idx] != NULL))
    {
        uint8_t evt_idx = GETB(scan_id, LLD_SCAN_ID_EVT);

        // Point to parameters
        struct lld_scan_env_params *scan_par = lld_scan_env->params[phy_idx];
        struct lld_scan_evt_params *scan_evt = &scan_par->evt[evt_idx];
        struct sch_arb_elt_tag *evt = &(scan_evt->evt);

        uint32_t clock = lld_read_clock();

        // Check if scan is under termination
        bool end = (scan_evt->state == SCAN_EVT_TERM);

        // Set the state back to Wait, as the frame is completed
        scan_evt->state = SCAN_EVT_WAIT;

        // If scan is under termination
        if (end)
        {
            // Close the driver
            lld_scan_end();
        }
        else
        {
            // Remove event
            sch_arb_remove(&(scan_evt->evt), true);

            if (RX_AUX_CHAIN_IND == scan_evt->aux_state)
            {
                // Unable to schedule chained data, generate a truncated report
                lld_scan_trunc_ind(scan_evt);
                scan_evt->aux_state = RX_ADV_EXT_IND;
            }

            if (CLK_SUB(clock, scan_par->last_prio_upd_ts) >= 2 * scan_par->intv)
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_SCAN_IDX));

                // Save the timestamp
                scan_par->last_prio_upd_ts = clock;
            }

            if (SCAN_EVT_DFT == evt_idx)
            {
#if (EAVESDROPPING_SUPPORT)
                // If segmented scan enabled for this PHY
                if (LLD_SCAN_SEG_EN(phy_idx))
                {
                    // Schedule next segmented scan
                    lld_scan_seg_sched(phy_idx, clock, true);
                }
                else
#endif // (EAVESDROPPING_SUPPORT)
                    // Reschedule ASAP
                    if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                    {
                        ASSERT_ERR_FORCE(0);
                    }
            }
            else // End the aux event
            {
                scan_evt->state = SCAN_EVT_END;
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESCAN, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_scan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_FRM_CBK_FUNC_BIT, timestamp, dummy, irq_type);

    ASSERT_INFO(dummy < 2 * MAX_SCAN_PHYS, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO: // Post-bandwidth contested as low priority (prio_2 = 0)
    {
        lld_scan_frm_eof_isr(dummy, timestamp, false);
    }
    break;
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    {
        lld_scan_frm_eof_isr(dummy, timestamp, true);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        lld_scan_frm_rx_isr(dummy);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        lld_scan_frm_skip_isr(dummy);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_scan_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_EVT_START_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LESCAN, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_scan_evt_params *scan_evt = (struct lld_scan_evt_params *) evt;

        // update if PAL filter enabled
        scan_evt->peradv_filt_en = ((lld_scan_sync_env) && (0 != lld_scan_sync_env->filter_policy));
        em_ble_filtpol_ralcntl_peradv_filt_en_setf(scan_evt->cs_idx, scan_evt->peradv_filt_en);

        do
        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk        = &lld_scan_frm_cbk;
            prog_par.time.hs        = evt->time.hs;
            prog_par.time.hus       = evt->time.hus;
            prog_par.cs_idx         = scan_evt->cs_idx;
            prog_par.dummy = 0;
            SETB(prog_par.dummy, LLD_SCAN_ID_PHY, scan_evt->index);
            SETB(prog_par.dummy, LLD_SCAN_ID_EVT, SCAN_EVT_DFT);
            prog_par.bandwidth      = evt->duration_min;
            prog_par.prio_1         = evt->current_prio;
            prog_par.prio_2         = 0;
            prog_par.prio_3         = rwip_priority[RWIP_PRIO_AUX_RX_IDX].value;
            prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
            prog_par.add.ble.ae_nps = 0;
            prog_par.add.ble.iso    = 0;
            prog_par.mode           = SCH_PROG_BLE;
            sch_prog_push(&prog_par);

#if (EAVESDROPPING_SUPPORT)
            DBG_SWDIAG(SEG_LESCAN, PHY_1MBPS, ((LLD_PHY_1M_POS == scan_evt->index) && LLD_SCAN_SEG_EN(scan_evt->index)));
            DBG_SWDIAG(SEG_LESCAN, PHY_CODED, ((LLD_PHY_CODED_POS == scan_evt->index) && LLD_SCAN_SEG_EN(scan_evt->index)));
#endif // (EAVESDROPPING_SUPPORT)

            DBG_SWDIAG(LESCAN, EVT_IDX, scan_evt->index);

            // Register the scan window as active to scheduling parameters
            sch_slice_bg_add(BLE_SCAN);

            // Move state
            scan_evt->state = SCAN_EVT_ACTIVE;

        }
        while (0);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

#if (EAVESDROPPING_SUPPORT)
    DBG_SWDIAG(SEG_LESCAN, PHY_1MBPS, 0);
    DBG_SWDIAG(SEG_LESCAN, PHY_CODED, 0);
#endif // (EAVESDROPPING_SUPPORT)

    DBG_SWDIAG(LESCAN, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_scan_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_EVT_CANCELED_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LESCAN, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_scan_evt_params *scan_evt = (struct lld_scan_evt_params *) evt;
        struct lld_scan_env_params *scan_par = lld_scan_env->params[scan_evt->index];

        if (scan_evt->state == SCAN_EVT_WAIT)
        {
            uint32_t clock = lld_read_clock();

            if (RX_AUX_CHAIN_IND == scan_evt->aux_state)
            {
                // Unable to schedule chained data, generate a truncated report
                lld_scan_trunc_ind(scan_evt);
                scan_evt->aux_state = RX_ADV_EXT_IND;
            }

            if (CLK_SUB(clock, scan_par->last_prio_upd_ts) >= 2 * scan_par->intv)
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_SCAN_IDX));

                // Save the timestamp
                scan_par->last_prio_upd_ts = clock;
            }

#if (EAVESDROPPING_SUPPORT)
            // If segmented scan enabled for this PHY
            if (LLD_SCAN_SEG_EN(scan_evt->index))
            {
                // Schedule next segmented scan
                lld_scan_seg_sched(scan_evt->index, clock, true);
            }
            else
#endif // (EAVESDROPPING_SUPPORT)
                // Reschedule ASAP
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    ASSERT_ERR_FORCE(0);
                }
        }
        else
        {
            ASSERT_INFO_FORCE(0, scan_evt->state, scan_evt->index);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESCAN, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event aux start notification
 ****************************************************************************************
 */
__STATIC void lld_scan_aux_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_AUX_EVT_START_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LESCAN, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_scan_evt_params *scan_evt = (struct lld_scan_evt_params *) evt;

        if (RX_AUX_ADV_IND == scan_evt->aux_state)
        {
            // update if PAL filter enabled
            scan_evt->peradv_filt_en = ((lld_scan_sync_env) && (0 != lld_scan_sync_env->filter_policy));
            em_ble_filtpol_ralcntl_peradv_filt_en_setf(scan_evt->cs_idx, scan_evt->peradv_filt_en);
        }

        do
        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk        = &lld_scan_frm_cbk;
            prog_par.time.hs        = evt->time.hs;
            prog_par.time.hus       = evt->time.hus;
            prog_par.cs_idx         = scan_evt->cs_idx;
            prog_par.dummy = 0;
            SETB(prog_par.dummy, LLD_SCAN_ID_PHY, scan_evt->index);
            SETB(prog_par.dummy, LLD_SCAN_ID_EVT, SCAN_EVT_AUX);
            prog_par.bandwidth      = evt->duration_min;
            prog_par.prio_1         = evt->current_prio;
            prog_par.prio_2         = 0;
            prog_par.prio_3         = rwip_priority[RWIP_PRIO_AUX_RX_IDX].value;
            prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
            prog_par.add.ble.ae_nps = 1;
            prog_par.add.ble.iso    = 0;
            prog_par.mode           = SCH_PROG_BLE;
            sch_prog_push(&prog_par);

            DBG_SWDIAG(LESCAN, EVT_IDX, scan_evt->index);

            // Move state
            scan_evt->state = SCAN_EVT_ACTIVE;

        }
        while (0);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESCAN, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_scan_aux_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_AUX_EVT_CANCELED_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LESCAN, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_scan_evt_params *scan_evt = (struct lld_scan_evt_params *) evt;
        struct lld_scan_env_params *scan_par = lld_scan_env->params[scan_evt->index];

        if (scan_evt->state == SCAN_EVT_WAIT)
        {
            uint32_t clock = lld_read_clock();

            // Unable to schedule chained data, generate a truncated report
            lld_scan_trunc_ind(scan_evt);

            scan_evt->state = SCAN_EVT_END;

            if (CLK_SUB(clock, scan_par->last_prio_upd_ts) >= 2 * scan_par->intv)
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_SCAN_IDX));

                // Save the timestamp
                scan_par->last_prio_upd_ts = clock;
            }
        }
        else
        {
            ASSERT_INFO_FORCE(0, scan_evt-> state, scan_evt->index);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESCAN, EVT_CANCELED, 0);
}



/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lld_scan_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_INIT_FUNC_BIT, is_reset);

    if (!is_reset)
    {
        lld_scan_env = NULL;
        lld_scan_sync_env = NULL;
#if (EAVESDROPPING_SUPPORT)
        lld_scan_seg_env = NULL;
#endif // (EAVESDROPPING_SUPPORT)
    }
    else
    {
        // Check if create sync is active
        if (lld_scan_sync_env != NULL)
        {
            ke_free(lld_scan_sync_env);
            lld_scan_sync_env = NULL;
        }

        if (lld_scan_env != NULL)
        {
            // Free activities for each scanning PHY
            for (int i = 0; i < MAX_SCAN_PHYS; i++)
            {
                if (lld_scan_env->params[i])
                {
                    ke_free(lld_scan_env->params[i]);
                }
            }

            // Free common event memory
            ke_free(lld_scan_env);
            lld_scan_env = NULL;
        }

#if (EAVESDROPPING_SUPPORT)
        // Check if segmented scan is configured
        if (lld_scan_seg_env != NULL)
        {
            ke_free(lld_scan_seg_env);
            lld_scan_seg_env = NULL;
        }
#endif // (EAVESDROPPING_SUPPORT)
    }
}

uint8_t lld_scan_start(uint8_t act_id, struct lld_scan_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_START_FUNC_BIT, uint8_t, act_id, params);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    struct lld_scan_env_tag *scan_env;
    struct lld_scan_env_params *scan_par;
    struct sch_arb_elt_tag *evt;

    DBG_SWDIAG(LESCAN, START, 1);

    // Check if scanning is inactive
    if (lld_scan_env == NULL)
    {
        // Allocate common environment
        scan_env = LLD_ALLOC_ENV(lld_scan_env_tag);

        if (scan_env != NULL)
        {
            uint32_t clock = lld_read_clock();

            uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(act_id, 0);

            bool config_txdesc = false;
            uint8_t cs_idx;
            uint8_t cs_fmt;

            LLD_INIT_ENV(scan_env, lld_scan_env_tag);

            if (params->scan_phys & PHY_1MBPS_BIT)
            {
                // Allocate 1M PHY environment
                scan_env->params[LLD_PHY_1M_POS] = LLD_ALLOC_ENV(lld_scan_env_params);

                if (scan_env->params[LLD_PHY_1M_POS] != NULL)
                {
                    // Point to parameters
                    scan_par = scan_env->params[LLD_PHY_1M_POS];
                    LLD_INIT_ENV(scan_par, lld_scan_env_params);

                    // Initialize event parameters (common part)
                    evt = &(scan_par->evt[SCAN_EVT_DFT].evt);

                    evt->cb_cancel        = &lld_scan_evt_canceled_cbk;
                    evt->cb_start         = &lld_scan_evt_start_cbk;
                    evt->current_prio     = rwip_priority[RWIP_PRIO_SCAN_IDX].value;
                    evt->duration_min     = co_min((2 * params->win_1m * SLOT_SIZE), sch_slice_params.scan_evt_dur);

                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_SCAN_IDX));

                    // Initialize aux event parameters (SW offload support)
                    evt = &(scan_par->evt[SCAN_EVT_AUX].evt);

                    evt->cb_cancel        = &lld_scan_aux_evt_canceled_cbk;
                    evt->cb_start         = &lld_scan_aux_evt_start_cbk;

                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_AUX_RX_IDX));

                    // Initialize event parameters (scanning part)
                    scan_par->intv = params->intv_1m;
                    scan_par->win = params->win_1m;
                    scan_par->type = params->type_1m;
                    scan_par->rate = CO_RATE_1MBPS; //00: 1Mbps - uncoded PHY
                    scan_par->win_rem_slots = scan_par->win;

                    // Base event parameters
                    scan_par->evt[SCAN_EVT_DFT].index = LLD_PHY_1M_POS;
                    scan_par->evt[SCAN_EVT_DFT].cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);
                    scan_par->evt[SCAN_EVT_DFT].state = SCAN_EVT_WAIT;
                    // Offload event parameters
                    scan_par->evt[SCAN_EVT_AUX].index = LLD_PHY_1M_POS;
                    scan_par->evt[SCAN_EVT_AUX].cs_idx = EM_BLE_CS_AUX_SCAN_IDX1;
                    scan_par->evt[SCAN_EVT_AUX].state = SCAN_EVT_END; // Idle

                    config_txdesc = (scan_par->type == SCAN_ACTIVE);

                    // Indicate scanning PHY 1M is present
                    SETB(scan_env->scan_phys, LLD_PHY_1M, 1);
                }
                else
                {
                    status = CO_ERROR_MEMORY_CAPA_EXCEED;
                    ASSERT_ERR_FORCE(0);
                }
            }

            if (params->scan_phys & PHY_CODED_BIT)
            {
                // Allocate coded PHY environment
                scan_env->params[LLD_PHY_CODED_POS] = LLD_ALLOC_ENV(lld_scan_env_params);

                if (scan_env->params[LLD_PHY_CODED_POS] != NULL)
                {
                    // Point to parameters
                    scan_par = scan_env->params[LLD_PHY_CODED_POS];
                    LLD_INIT_ENV(scan_par, lld_scan_env_params);

                    // Initialize event parameters (common part)
                    evt = &(scan_par->evt[SCAN_EVT_DFT].evt);

                    evt->cb_cancel        = &lld_scan_evt_canceled_cbk;
                    evt->cb_start         = &lld_scan_evt_start_cbk;
                    evt->current_prio     = rwip_priority[RWIP_PRIO_SCAN_IDX].value;
                    evt->duration_min     = co_min((2 * params->win_c * SLOT_SIZE), sch_slice_params.scan_evt_dur);

                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_SCAN_IDX));

                    // Initialize aux event parameters (SW offload support)
                    evt = &(scan_par->evt[SCAN_EVT_AUX].evt);

                    evt->cb_cancel        = &lld_scan_aux_evt_canceled_cbk;
                    evt->cb_start         = &lld_scan_aux_evt_start_cbk;

                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_AUX_RX_IDX));

                    // Initialize event parameters (scanning part)
                    scan_par->intv = params->intv_c;
                    scan_par->win = params->win_c;
                    scan_par->type = params->type_c;
                    scan_par->rate = CO_RATE_500KBPS; //1x: 125kbps or 500kbps - coded PHY
                    scan_par->win_rem_slots = scan_par->win;
                    // Base event parameters
                    scan_par->evt[SCAN_EVT_DFT].index = LLD_PHY_CODED_POS;
                    scan_par->evt[SCAN_EVT_DFT].cs_idx = EM_BLE_CS_EXT_SCAN_IDX2;
                    scan_par->evt[SCAN_EVT_DFT].state = SCAN_EVT_WAIT;
                    // Offload event parameters
                    scan_par->evt[SCAN_EVT_AUX].index = LLD_PHY_CODED_POS;
                    scan_par->evt[SCAN_EVT_AUX].cs_idx = EM_BLE_CS_AUX_SCAN_IDX2;
                    scan_par->evt[SCAN_EVT_AUX].state = SCAN_EVT_END; // Idle

                    config_txdesc |= (scan_par->type == SCAN_ACTIVE);

                    // Indicate scanning PHY 1M is present
                    SETB(scan_env->scan_phys, LLD_PHY_CODED, 1);
                }
                else
                {
                    status = CO_ERROR_MEMORY_CAPA_EXCEED;
                    ASSERT_ERR_FORCE(0);
                }
            }

            // Initialize common scanning parameters
            scan_env->own_addr = params->own_addr;
            scan_env->own_addr_type = params->own_addr_type;
            scan_env->ext_scan = params->ext_scan;
            scan_env->filter_policy = params->filter_policy;
            scan_env->act_id = act_id;
            scan_env->ral_resol_en = params->addr_resolution_en;

            // Set scan time limit
            if (params->duration)
            {
                scan_env->scan_end_ts = CLK_ADD_2(clock, ((2 * SCAN_DURATION_USECS(params->duration)) / HALF_SLOT_SIZE));
            }
            else
            {
                scan_env->scan_end_ts = LLD_CLOCK_UNDEF;
            }

            // Configure CS -

            for (uint8_t phy_idx = 0; phy_idx < MAX_SCAN_PHYS; phy_idx++)
            {
                if (scan_env->params[phy_idx] == NULL)
                    continue;

                // Point to parameters
                scan_par = scan_env->params[phy_idx];

                scan_par->evt[SCAN_EVT_DFT].aux_state = RX_ADV_EXT_IND;


                for (uint8_t evt_idx = 0; evt_idx < NUM_SCAN_EVTS; evt_idx++)
                {
                    // CS configuration:
                    cs_idx =  scan_par->evt[evt_idx].cs_idx;

                    // Set permission/status of CS as R/W but uninitialized
                    DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

                    // set Synchronization Word
                    em_ble_syncwl_set(cs_idx, LE_ADV_CH_ACC_ADDR_L);
                    em_ble_syncwh_set(cs_idx, LE_ADV_CH_ACC_ADDR_H);
                    // set CRC Initialization value
                    em_ble_crcinit0_set(cs_idx, 0x5555);
                    em_ble_crcinit1_pack(cs_idx, /*rxmaxctebuf*/ 0, /*crcinit1*/ 0x55);

                    // initialize all extadv status
                    em_ble_extadvstat_pack(cs_idx, 0, 0, 0, 0, 0);


                    // update Scan filter policy
                    /**
                     * Scan filter policy:
                     * 0x00 Accept all
                     *   - advertisement packets except directed advertising packets not addressed to this device (default)
                     * 0x01 Accept only
                     *   - advertisement packets from devices where the advertiser's address is in the White list.
                     *   - Directed advertising packets which are not addressed for this device shall be ignored.
                     * 0x02 Accept all
                     *   - undirected advertisement packets, and
                     *   - directed advertising packets where the initiator address is a resolvable private address, and
                     *   - directed advertising packets addressed to this device.
                     * 0x03 Accept all
                     *   - advertisement packets from devices where the advertiser's address is in the White list, and
                     *   - directed advertising packets where the initiator address is a resolvable private address, and
                     *   - directed advertising packets addressed to this device.
                     * 0x04 - 0xFF Reserved for future use.
                     */
                    scan_par->evt[SCAN_EVT_DFT].peradv_filt_en = ((lld_scan_sync_env) && (0 != lld_scan_sync_env->filter_policy));

                    em_ble_filtpol_ralcntl_pack(cs_idx, scan_env->filter_policy, /*ralresolen*/ params->addr_resolution_en,
                                                /*peradvfilten*/ scan_par->evt[SCAN_EVT_DFT].peradv_filt_en,
                                                /*local_rpa_sel*/((params->own_addr_type & ADDR_RPA_MASK) != 0), /*ralmode*/0,
                                                /*ral_en*/(scan_env->ral_resol_en || ((scan_env->own_addr_type & ADDR_RPA_MASK) != 0)));

                    // set Frequency Hopping, Channel Index
                    em_ble_hopcntl_pack(cs_idx, /*fhen*/ 1, /*hopsel*/ 0, /*hopint*/ 0, /*chidx*/ 39);

                    // Set Rx Max buf and Rx Max Time @0x0 -> v4.0 behavior
                    em_ble_rxmaxbuf_set(cs_idx, 0x0);
                    em_ble_rxmaxtime_set(cs_idx, 0x0);

                    // Set link field
                    em_ble_linkcntl_pack(cs_idx, /*hplpmode*/ 0, /*linklbl*/ cs_idx, /*sas*/ false, /*nullrxllidflt*/true,
                                         /*micmode*/ENC_MIC_PRESENT, /*cryptmode*/ENC_MODE_PKT_PLD_CNT,
                                         /*txcrypten*/ false, /*rxcrypten*/ false,
                                         /*privnpub*/ ((params->own_addr_type & ADDR_MASK) != ADDR_PUBLIC));

                    // Set the Device identity (BD Address)
                    em_ble_lebdaddr_set(cs_idx, 0, co_read16p(&params->own_addr.addr[0]));
                    em_ble_lebdaddr_set(cs_idx, 1, co_read16p(&params->own_addr.addr[2]));
                    em_ble_lebdaddr_set(cs_idx, 2, co_read16p(&params->own_addr.addr[4]));

                    // set the TX power level for SCAN_REQ
                    em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_max);

                    if (scan_par->type == SCAN_ACTIVE)
                    {
                        // Pointer to the Tx Descriptor
                        em_ble_acltxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, txdesc_idx));

                        // Prepare scan format
                        cs_fmt = ((scan_env->ext_scan) ? EM_BLE_CS_FMT_EXT_ACTIVE_SCAN : EM_BLE_CS_FMT_ACTIVE_SCAN);
                    }
                    else
                    {
                        // Configure as NULL the Tx Descriptor
                        em_ble_acltxdescptr_set(cs_idx, 0);

                        // Prepare scan format
                        cs_fmt = ((scan_env->ext_scan) ? EM_BLE_CS_FMT_EXT_PASSIVE_SCAN : EM_BLE_CS_FMT_PASSIVE_SCAN);
                    }

                    // Configure as NULL the Aux Tx Descriptor (aux_scan_req uses acltxdescptr to prevent from duplicate programming.)
                    em_ble_auxtxdescptr_set(cs_idx, 0);

                    // Set the max AUX CHAIN Rx Byte and Rx Desc
                    em_ble_rxmaxauxchain_pack(cs_idx, /*maxrxchdesc*/ BLE_ADV_FRAG_NB_RX_MAX, /*maxrxchbyte*/ HOST_ADV_DATA_LEN_MAX);

                    // Set the scan type for this PHY
                    em_ble_cntl_pack(cs_idx,
                                     RWIP_COEX_GET(SCAN, TXBSY),
                                     RWIP_COEX_GET(SCAN, RXBSY),
                                     RWIP_COEX_GET(SCAN, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                                     0,
#endif // EAVESDROPPING_SUPPORT
                                     cs_fmt);

                    // Set Rx/Tx threshold + rate
                    em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/LLD_SCAN_RX_THRESHOLD, /*txthr*/0, /*auxrate*/0,
                                                 /*rxrate*/scan_par->rate, /*txrate*/scan_par->rate);

                    if (SCAN_EVT_DFT == evt_idx)
                    {
                        em_ble_maxevtime_set(cs_idx, scan_par->win_rem_slots);

                        // set Wide-open mode, Size of the Rx half-window in half-slots
                        em_ble_rxwincntl_pack(cs_idx, 1, co_min(scan_par->win_rem_slots, EM_BLE_CS_RXWINSZ_MAX));
                    }

                    // Disable antenna switching
                    em_ble_rxdfantpattcntl_set(cs_idx, 0);
                    // Disable CTE reception
                    em_ble_rxdfcntl_set(cs_idx, 0);

                    // Disable unused control
                    em_ble_chmap0_set(cs_idx, 0);
                    em_ble_chmap1_set(cs_idx, 0);
                    em_ble_chmap2_set(cs_idx, 0);
                    em_ble_minevtime_set(cs_idx, 0);
                    em_ble_evtcnt_setf(cs_idx, 0);
                    em_ble_txheadercntl_set(cs_idx, 0);
                }
            }

            // Configure TXDESC -

            // configure the Tx Descriptor for SCAN_REQ/AUX_SCAN_REQ
            if (config_txdesc)
            {
                // set Tx for BLE_SCAN_REQ / BLE_AUX_SCAN_REQ
                em_ble_txphadv_txtype_setf(txdesc_idx, BLE_SCAN_REQ);

                //when sending SCAN_REQ the packet controller does not read anything from TX buffer.
                em_ble_txdataptr_setf(txdesc_idx, 0);

                // Set the length
                em_ble_txphadv_txadvlen_setf(txdesc_idx, (2 * BD_ADDR_LEN));

                // Set SCAN_REQ Tx descriptor fields
                em_ble_txcntl_nextptr_setf(txdesc_idx, 0);

                // Release descriptor
                em_ble_txcntl_txdone_setf(txdesc_idx, 0);
            }

            // Reset the Backoff Register
            ble_actscancntl_pack(/*backoff*/ 1, /*upperlimit*/ 1);

            GLOBAL_INT_DISABLE();

            // Assign configured parameters
            lld_scan_env = scan_env;

            // Schedule events ASAP
            for (uint8_t phy_idx = 0; phy_idx < MAX_SCAN_PHYS; phy_idx++)
            {
                if (scan_env->params[phy_idx] == NULL)
                    continue;

                scan_par = scan_env->params[phy_idx];

                // Save the timestamp
                scan_par->last_prio_upd_ts = clock;

#if (EAVESDROPPING_SUPPORT)
                // If segmented scan enabled for this PHY
                if (LLD_SCAN_SEG_EN(phy_idx))
                {
                    // Schedule first segmented scan
                    lld_scan_seg_env->params[phy_idx].seg_scan_state = SEG_SCAN_ENABLING;
                    lld_scan_seg_sched(phy_idx, clock, false);
                }
                else
#endif // (EAVESDROPPING_SUPPORT)
                {
                    evt = &(scan_par->evt[SCAN_EVT_DFT].evt);
                    evt->time.hs = clock;

                    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    {
                        scan_par->evt[SCAN_EVT_DFT].aux_offload = NO_AUX_OFFLOAD;
                        scan_par->evt[SCAN_EVT_DFT].pkt_cnt = 0;
                        scan_par->evt[SCAN_EVT_DFT].rx_chain_cnt_bytes = 0;

#if LLD_SCAN_STRICT_INTERVAL
                        scan_par->anchor_ts = clock;
#endif // LLD_SCAN_STRICT_INTERVAL

                        // push next phy to be program just after current one
                        clock = CLK_ADD_2(clock, (scan_par->win) << 1);
                    }
                    else
                    {
                        ASSERT_ERR_FORCE(0);
                    }
                }
            }

            GLOBAL_INT_RESTORE();

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }

    DBG_SWDIAG(LESCAN, START, 0);

    return (status);
}

uint8_t lld_scan_stop(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_STOP_FUNC_BIT, uint8_t);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    DBG_SWDIAG(LESCAN, STOP, 1);

    if (lld_scan_env != NULL)
    {
        // Stop scan
        lld_scan_end();

        status = CO_ERROR_NO_ERROR;
    }

    DBG_SWDIAG(LESCAN, STOP, 0);

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_scan_params_update(uint16_t duration, struct bd_addr *bd_addr)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_PARAMS_UPDATE_FUNC_BIT, uint8_t, duration, bd_addr);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_scan_env != NULL)
    {
        // Point to parameters
        struct lld_scan_env_tag *scan_env = lld_scan_env;

        // Set scan time limit
        if (duration)
        {
            scan_env->scan_end_ts = CLK_ADD_2(lld_read_clock(), ((2 * SCAN_DURATION_USECS(duration)) / HALF_SLOT_SIZE));
        }
        else
        {
            scan_env->scan_end_ts = LLD_CLOCK_UNDEF;
        }

        // Any change to the random address shall also take effect
        if (bd_addr)
        {
            scan_env->own_addr = *bd_addr;
        }

        // Restart scanning
        for (uint8_t phy_idx = 0; phy_idx < MAX_SCAN_PHYS; phy_idx++)
        {
            if (lld_scan_env->params[phy_idx] != NULL)
            {
                // Point to parameters
                struct lld_scan_env_params *scan_par = lld_scan_env->params[phy_idx];
                struct sch_arb_elt_tag *evt = &(scan_par->evt[SCAN_EVT_DFT].evt);

                // Check if event is waiting
                if (scan_par->evt[SCAN_EVT_DFT].state == SCAN_EVT_WAIT)
                {
                    uint32_t clock = lld_read_clock();

                    if (CLK_DIFF(clock, evt->time.hs) > (2 * LLD_SCAN_EVT_DELAY_RESCHED_MIN))
                    {
                        // Remove from schedule
                        sch_arb_remove(evt, false);

#if (EAVESDROPPING_SUPPORT)
                        // If segmented scan enabled for this PHY
                        if (LLD_SCAN_SEG_EN(phy_idx))
                        {
                            // Schedule next segmented scan
                            lld_scan_seg_sched(phy_idx, clock, false);
                        }
                        else
#endif // (EAVESDROPPING_SUPPORT)
                        {
                            // Reschedule ASAP
                            evt->time.hs = clock;
                            sch_arb_insert(evt);
                        }
                    }
                }
            }
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}

uint8_t lld_scan_create_sync(uint8_t act_id, uint8_t filter_policy, uint8_t adv_sid, uint8_t adv_addr_type, struct bd_addr *adv_addr)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_CREATE_SYNC_FUNC_BIT, uint8_t, 5, act_id, filter_policy, adv_sid, adv_addr_type, adv_addr);
    uint8_t status = CO_ERROR_NO_ERROR;
    // Check if create sync is active
    ASSERT_ERR(lld_scan_sync_env == NULL);

    GLOBAL_INT_DISABLE();

    lld_scan_sync_env = LLD_ALLOC_ENV(lld_scan_sync_env_tag);

    if (lld_scan_sync_env != NULL)
    {
        lld_scan_sync_env->act_id = act_id;

        lld_scan_sync_env->filter_policy = filter_policy;

        if (adv_addr != NULL)
        {
            lld_scan_sync_env->adv_sid = adv_sid;
            lld_scan_sync_env->adv_addr_type = adv_addr_type;
            memcpy(&lld_scan_sync_env->adv_addr.addr[0], &adv_addr->addr[0], BD_ADDR_LEN);
        }
    }
    else
    {
        status = CO_ERROR_MEMORY_CAPA_EXCEED;
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();

    return status;
}

uint8_t lld_scan_create_sync_cancel(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_CREATE_SYNC_CANCEL_FUNC_BIT, uint8_t, act_id);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    // Check if create sync is active
    if (lld_scan_sync_env != NULL)
    {
        // Free memory used for create sync and stop looking for syncinfo
        ke_free(lld_scan_sync_env);
        lld_scan_sync_env = NULL;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}

#if(BLE_HOST_PRESENT)
void lld_scan_rand_addr_update(const struct bd_addr *p_addr)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SCAN_PATCH_TYPE, LLD_SCAN_RAND_ADDR_UPDATE_FUNC_BIT, p_addr);

    GLOBAL_INT_DISABLE();

    if (lld_scan_env != NULL)
    {
        lld_scan_env->own_addr = *p_addr;
    }

    GLOBAL_INT_RESTORE();
}
#endif // (BLE_HOST_PRESENT)

#if (EAVESDROPPING_SUPPORT)
uint8_t lld_scan_seg_start(struct lld_scan_seg_params *params)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    DBG_SWDIAG(SEG_LESCAN, START, 1);

    GLOBAL_INT_DISABLE();

    // Check if segmented scanning is configured
    if (lld_scan_seg_env == NULL)
    {
        // Allocate environment
        lld_scan_seg_env = LLD_ALLOC_ENV(lld_scan_seg_env_tag);
        ASSERT_ERR(lld_scan_seg_env);
    }

    // If environment allocated
    if (lld_scan_seg_env != NULL)
    {
        memset(lld_scan_seg_env, 0, sizeof(struct lld_scan_seg_env_tag));

        lld_scan_seg_env->bd_addr = params->bd_addr;
        lld_scan_seg_env->scan_phys = 0;

        // Save 1M PHY params if specified
        if (params->scan_phys & PHY_1MBPS_BIT)
        {
            // Point to parameters
            struct lld_seg_params *seg_params = &lld_scan_seg_env->params[LLD_PHY_1M_POS];

            // Segmented scan configured/enabled for this PHY
            lld_scan_seg_env->scan_phys |= LLD_PHY_1M_BIT;

            // Save parameters, convert to half-slots and half-us for internal driver usage
            seg_params->time_offset = params->time_offset_1m << 1;
            seg_params->period = params->periodicity_1m << 1;
            seg_params->duration = params->duration_1m << 1;
            seg_params->segments = params->segments_1m;
            seg_params->hus_offset = params->us_offset_1m << 1;

            // Set state to enable on this PHY for the new configuration
            seg_params->seg_scan_state = SEG_SCAN_ENABLING;
        }

        // Save coded PHY params if specified
        if (params->scan_phys & PHY_CODED_BIT)
        {
            // Point to parameters
            struct lld_seg_params *seg_params = &lld_scan_seg_env->params[LLD_PHY_CODED_POS];

            // Segmented scan configured/enabled for this PHY
            lld_scan_seg_env->scan_phys |= LLD_PHY_CODED_BIT;

            // Save parameters, convert to half-slots and half-us for internal driver usage
            seg_params->time_offset = params->time_offset_c << 1;
            seg_params->period = params->periodicity_c << 1;
            seg_params->duration = params->duration_c << 1;
            seg_params->segments = params->segments_c;
            seg_params->hus_offset = params->us_offset_c << 1;

            // Set state to enable on this PHY for the new configuration
            seg_params->seg_scan_state = SEG_SCAN_ENABLING;
        }
    }
    else
    {
        // Cannot configure
        status = CO_ERROR_MEMORY_CAPA_EXCEED;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(SEG_LESCAN, START, 0);

    return status;
}

uint8_t lld_scan_seg_stop(void)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    DBG_SWDIAG(SEG_LESCAN, STOP, 1);

    GLOBAL_INT_DISABLE();

    // Check if segmented scanning is configured
    if (lld_scan_seg_env != NULL)
    {
        // Check if scanning enabled
        if (lld_scan_env != NULL)
        {
            if (lld_scan_seg_env->scan_phys & LLD_PHY_1M_BIT)
            {
                // Point to parameters
                struct lld_scan_env_params *scan_par = lld_scan_env->params[LLD_PHY_1M_POS];

                if (scan_par)
                {
                    struct lld_scan_evt_params *scan_evt = &scan_par->evt[SCAN_EVT_DFT];
                    struct sch_arb_elt_tag *evt = &(scan_evt->evt);

                    // Restore defaults for next scheduling
                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_SCAN_IDX));
                    evt->current_prio = rwip_priority[RWIP_PRIO_SCAN_IDX].value;
                    scan_par->win_rem_slots = scan_par->win;
                }
            }

            if (lld_scan_seg_env->scan_phys & LLD_PHY_CODED_BIT)
            {
                // Point to parameters
                struct lld_scan_env_params *scan_par = lld_scan_env->params[LLD_PHY_CODED_POS];

                if (scan_par)
                {
                    struct lld_scan_evt_params *scan_evt = &scan_par->evt[SCAN_EVT_DFT];
                    struct sch_arb_elt_tag *evt = &(scan_evt->evt);

                    // Restore defaults for next scheduling
                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_SCAN_IDX));
                    evt->current_prio = rwip_priority[RWIP_PRIO_SCAN_IDX].value;
                    scan_par->win_rem_slots = scan_par->win;
                }
            }
        }

        // Free memory used for segmented scanning control
        ke_free(lld_scan_seg_env);
        lld_scan_seg_env = NULL;
    }
    else
    {
        // Nothing configured
        status = CO_ERROR_COMMAND_DISALLOWED;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(SEG_LESCAN, STOP, 0);

    return status;
}
#endif // (EAVESDROPPING_SUPPORT)

#endif // (BLE_OBSERVER)

///@} LLDSCAN
