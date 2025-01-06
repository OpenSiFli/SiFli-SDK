/**
****************************************************************************************
*
* @file lld_bi.c
*
* @brief LLD Broadcast Isochronous source code
*
* Copyright (C) RivieraWaves 2009-2017
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDBI
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"               // stack configuration
#include "rwip.h"                      // IP definitions

#if (BLE_BIS)

#include <string.h>
#include "co_math.h"
#include "co_endian.h"                 // For CRC init computing
#include "ble_util.h"                  // BLE utility functions
#include "dbg.h"
#include "arch.h"                      // Platform definition
#include "ke_mem.h"
#include "ke_task.h"                   // Kernel task management
#include "rwip.h"
#include "lld.h"                       // link driver API
#include "lld_int.h"                   // link layer driver internal
#include "lld_int_iso.h"               // LLD Internal API for ISO
#include "reg_blecore.h"               // BLE core registers
#include "reg_em_ble_cs.h"             // BLE EM Control Structure
#include "reg_em_ble_rx_desc.h"        // BLE EM RX descriptors
#include "reg_em_ble_tx_iso_desc.h"    // BLE EM TX ISO descriptors
#include "reg_em_ble_rx_iso_desc.h"    // BLE EM RX ISO descriptors

#include "reg_em_ble_tx_iso_buf.h"     // For BIS PDU transmission
#include "reg_em_ble_rx_iso_buf.h"     // For BIS PDU reception

#include "data_path.h"                 // Data path environment
#include "ble_util_buf.h"              // To manage hopping sequences

#include "sch_alarm.h"                 // Scheduling Alarm
#include "sch_arb.h"                   // Scheduling Arbiter
#include "sch_prog.h"                  // Scheduling Programmer
#include "sch_slice.h"                 // Scheduling Slicer
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Maximum distance between end of a sub event and beginning of next sub event in a scheduling event (in half slots)
#define LLD_BI_MAX_SE_SCHED_DISTANCE           (6)

/// Max CSSN value
#define LLD_BI_CSSN_MAX                        (7)
/// Invalid CSSN value - used for initialization
#define LLD_BI_CSSN_INVALID                    (0xFF)

/// Minimum sub-event distance between long sync windows (for receiver only) (in half-us)
#define LLD_BI_SYNC_WIN_MIN_DISTANCE           (1200)

/*
 * MACROS
 *****************************************************************************************
 */

/// Default number of channels scanned during channel scan event
#define LLD_BI_SCAN_NB_CHAN      (4)
/// Default scan window size during channel scan event (in us)
#define LLD_BI_SCAN_WIN_SIZE    (14)

/// Get BIS structure
#define LLD_BIS_GET(act_id)     ((struct lld_bis_env *)lld_bis_env[act_id])
/// Get BIG structure
#define LLD_BIG_GET(grp_hdl)    ((struct lld_big_env *)lld_big_env[grp_hdl])

/*
 * ENUMERATION DEFINITION
 *****************************************************************************************
 */

/// Type of the instant-related procedure
enum lld_big_instant_proc_type
{
    /// No procedure
    INSTANT_PROC_NO_PROC,
    /// Channel map update procedure
    INSTANT_PROC_CH_MAP_UPD,
};

/// BIS information bit field
///
///   7   6   5   4   3      2        1     0
/// +---+---+---+---+---+---------+------+-----+
/// |        RFU        | STARTED | PROG | MIC |
/// +---+---+---+---+---+---------+------+-----+
///
enum lld_bis_info_fields
{
    /// Indicate that MIC Failure has been detected
    LLD_BIS_INFO_MIC_FAIL_POS    = 0,
    LLD_BIS_INFO_MIC_FAIL_BIT    = 0x01,

    /// Indicate if one subevent has been programmed for this BIS event
    LLD_BIS_INFO_PROG_POS        = 1,
    LLD_BIS_INFO_PROG_BIT        = 0x02,

    /// Indicate that BIS has been started
    LLD_BIS_INFO_STARTED_POS     = 2,
    LLD_BIS_INFO_STARTED_BIT     = 0x04,
};

/// BIG information bit field
///
///    7       6      5         4          3          2         1        0
/// +------+------+----------+-----------+---------+------------+------+---------+
/// | INIT | SYNC | RECEIVER | ALARM PROG| NEW_EVT | INTERLACED | PROG | ENCRYPT |
/// +------+------+----------+-----------+---------+------------+------+---------+
///
///      15         14          13         12       11          10      9       8
/// +---------+------------+----------+---------+-----------+--------+------+-------+
/// | PROG_EN | SYNC_ESTAB | CHMAP_UP | HOP_TOG | SCHED_REJ | CSE_UP | STOP | ALARM |
/// +---------+------------+----------+---------+-----------+--------+------+-------+

enum lld_big_info_fields
{
    /// Indicate if BIG is encrypted
    LLD_BIG_ENCRYPT_POS       = 0,
    LLD_BIG_ENCRYPT_BIT       = 0x0001,

    /// Indicate that a subevent is programmed
    LLD_BIG_PROG_POS          = 1,
    LLD_BIG_PROG_BIT          = 0x0002,

    /// Indicate that streams are interleaved
    LLD_BIG_INTERLEAVED_POS   = 2,
    LLD_BIG_INTERLEAVED_BIT   = 0x0004,

    /// Indicate a BIG event is ongoing
    LLD_BIG_EVT_ONGOING_POS   = 3,
    LLD_BIG_EVT_ONGOING_BIT   = 0x0008,

    /// Indicate that scheduling alarm is for programming new sub-events
    LLD_BIG_ALARM_PROG_POS    = 4,
    LLD_BIG_ALARM_PROG_BIT    = 0x0010,

    /// Indicate if the device is BIG receiver
    LLD_BIG_RECEIVER_POS         = 5,
    LLD_BIG_RECEIVER_BIT         = 0x0020,

    /// Indicate that synchronization has been caught during a scheduling event (broadcast receiver only)
    LLD_BIG_SYNC_POS          = 6,
    LLD_BIG_SYNC_BIT          = 0x0040,

    /// Indicate the sub-events are programmed using synchronization information caught during the event (broadcast receiver only)
    LLD_BIG_PROG_SYNC_POS     = 7,
    LLD_BIG_PROG_SYNC_BIT     = 0x0080,

    /// Indicate that scheduling alarm is currently programmed
    LLD_BIG_ALARM_POS         = 8,
    LLD_BIG_ALARM_BIT         = 0x0100,

    /// Indicate that BIG has to be stopped
    LLD_BIG_STOP_POS          = 9,
    LLD_BIG_STOP_BIT          = 0x0200,

    LLD_BIG_CSE_CS_UPDATE_POS = 10,
    LLD_BIG_CSE_CS_UPDATE_BIT = 0x0400,

    /// Indicate that scheduling has been rejected
    LLD_BIG_SCHED_REJECT_POS  = 11,
    LLD_BIG_SCHED_REJECT_BIT  = 0x0800,

    /// HOP Sequence pointer in use
    LLD_BIG_HOP_TOG_POS       = 12,
    LLD_BIG_HOP_TOG_BIT       = 0x1000,

    /// Channel map of control sub-event must be updated
    LLD_BIG_CH_MAP_UPD_POS    = 13,
    LLD_BIG_CH_MAP_UPD_BIT    = 0x2000,

    /// Indicates if BIG sync has been established (broadcast receiver only)
    LLD_BIG_SYNC_ESTAB_POS     = 14,
    LLD_BIG_SYNC_ESTAB_BIT     = 0x4000,

    /// Programming of BIG sub-events is enabled
    LLD_BIG_PROG_EN_POS        = 15,
    LLD_BIG_PROG_EN_BIT        = 0x8000,
};

/// Data information bit field
///
///    7    6    5    4    3    2     1     0
/// +----+----+----+----+----+----+------+------+
/// |             RFU             | DONE | PROG |
/// +----+----+----+----+----+----+------+------+
///
enum lld_bis_data_info_fields
{
    /// Indicate that address of buffer in the exchange memory has been retrieved
    /// Valid only for BIS Data
    LLD_BIS_DATA_PROG_POS           = 0,
    LLD_BIS_DATA_PROG_BIT           = 0x01,

    /// Indicate that data buffer has been released
    LLD_BIS_DATA_DONE_POS           = 1,
    LLD_BIS_DATA_DONE_BIT           = 0x02,
};

/// Sub event status
///
///    7    6    5    4     3      2      1      0
/// +----+----+----+----+------+------+-------+-----+
/// |        RFU        | SKIP | PROG | RX_OK | REJ |
/// +----+----+----+----+------+------+-------+-----+
///
enum lld_bi_se_status_fields
{
    /// Indicate that sub event has been rejected by scheduler
    LLD_BI_SE_STATUS_REJ_POS       = 0,
    LLD_BI_SE_STATUS_REJ_BIT       = 0x01,

    /// Indicate that data packet has been properly received during this event
    LLD_BI_SE_STATUS_RX_OK_POS     = 1,
    LLD_BI_SE_STATUS_RX_OK_BIT     = 0x02,

    /// Indicate that Sub Event properly programmed
    LLD_BI_SE_STATUS_PROG_POS      = 2,
    LLD_BI_SE_STATUS_PROG_BIT      = 0x04,

    /// Indicate that Sub Event has been skipped
    LLD_BI_SE_STATUS_SKIP_POS      = 3,
    LLD_BI_SE_STATUS_SKIP_BIT      = 0x08,

    /// Indicate that Sub Event has been skipped at programming
    LLD_BI_SE_STATUS_SKIP_PROG_POS = 4,
    LLD_BI_SE_STATUS_SKIP_PROG_BIT = 0x10,

    /// Indicate that data has been programmed for the Sub Event
    LLD_BI_SE_STATUS_DATA_PROG_POS = 5,
    LLD_BI_SE_STATUS_DATA_PROG_BIT = 0x20,
};

/// Fields of dummy value provided to scheduling arbiter
///
///   31 - 24     23 - 16      15 - 8     7 - 0
/// +---------+-------------+----------+---------+
/// |   TYPE  | SUB_EVT_IDX | ACT_ID   | GRP_HDL |
/// +---------+-------------+----------+---------+
///
enum lld_bi_dummy_fields
{
    /// Group Handle
    LLD_BI_DUMMY_GRP_HDL_LSB           = 0,
    LLD_BI_DUMMY_GRP_HDL_MASK          = 0x000000FF,

    /// BIS activity ID
    LLD_BI_DUMMY_ACT_ID_LSB            = 8,
    LLD_BI_DUMMY_ACT_ID_MASK           = 0x0000FF00,

    /// Sub event index
    LLD_BI_DUMMY_SUB_EVT_IDX_LSB       = 16,
    LLD_BI_DUMMY_SUB_EVT_IDX_MASK      = 0x00FF0000,

    /// Type (@see enum lld_bi_dummy_type)
    LLD_BI_DUMMY_TYPE_LSB              = 24,
    LLD_BI_DUMMY_TYPE_MASK             = 0xFF000000,
};

/// Values for type field in dummy value
enum lld_bi_dummy_type
{
    /// Transmission/reception of BIS data
    LLD_BI_DUMMY_TYPE_DATA = 0,
    /// Transmission/reception of BIS control
    LLD_BI_DUMMY_TYPE_CNTL,
    /// Channel scan (for BIS broadcaster)
    LLD_BI_DUMMY_TYPE_CHAN_SCAN,
};

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// Union for storing the update information for any procedure
union lld_big_instant_proc_new_par
{
    /// New channel map
    struct le_ch_map ch_map;
};

/// Parameters update information
struct lld_big_instant_proc_info
{
    /// Data associated with procedure
    union lld_big_instant_proc_new_par data;

    /// Instant when the new parameter apply (compared to connection event counter)
    uint16_t instant;

    /**
     * Type of update procedure:
     *    - 0: No update ongoing
     *    - 1: Channel map update
     */
    uint8_t type;
};

/// Structure containing data mapping information information
struct lld_bi_data_map
{
    /// Offset from @see lld_big_env data_pos value
    uint8_t offset;
    /// Indicate if transmission or reception attempts will be the last one
    bool    last;
};

/// Structure containing information about an BIS transmission or reception
struct lld_bi_data
{
    /// Exchange memory buffer index
    uint8_t  buf_idx;
    /// Data info (@see enum lld_bis_data_info_fields)
    uint8_t  data_info;
};

/// Subevent information
struct lld_bi_se
{
    /// Offsets between Group Anchor and sub-event  (in half microseconds)
    uint32_t            offset;
    /// Timestamp to be programmed, half slot part
    uint32_t            timestamp;
    /// Duration of packed event (in half microseconds); valid if nb_packed != 0
    uint32_t            duration_packed;

#if (BLE_OBSERVER)
    /// Expected synchronization timestamp, half slot part
    uint32_t            exp_sync_ts;
    /// Expected synchronization timestamp, half microseconds part
    uint16_t            exp_sync_bit_off;
#endif //(BLE_OBSERVER)

    /// Timestamp to be programmed, half microseconds part
    uint16_t            bit_offset;
    /// BIS activity identifier
    uint8_t             bis_act_id;
    /// BIS Sub-event index
    uint8_t             bis_sub_evt_idx;

    /// Descriptor index
    uint8_t             desc_idx;
    /// Indicate if sub event is currently programmed
    uint8_t             state;

    /// Number of subevents packed in same scheduled event
    /// Value != 0 for first sub_event of a packed sub-event
    uint8_t             nb_packed;
};

/// LLD BIG structure
struct lld_big_env
{
    /// Scheduling arbiter element for BIS data - !!!! Must remain the first element !!!!
    struct sch_arb_elt_tag           elt;
    /// Alarm used for 3 purposes: initializing CS at BIG start, handling a skip of an entire event, programming subevents
    struct sch_alarm_tag             alarm;
    /// Callback function used to provide the GPIO timestamp at local synchronization (NULL if local sync disabled)
    data_path_local_sync_cb          cb_local_sync;
    /// Callback function used to provide the event timing information to a data path (only for receiver, NULL if peer sync disabled)
    data_path_peer_sync_cb           cb_peer_sync;
    /// List of BISes belonging to this BIG (sorted by BIS offset values)
    struct co_list                   list_bis;
    /// Pointer to an array containing subevent information
    struct lld_bi_se                *p_se_infos;
    /// Update procedure information
    struct lld_big_instant_proc_info instant_proc;
#if (BLE_OBSERVER)
    /// Scheduled RX window size (in half microseconds)
    uint32_t                         rx_win_size;
    /// Supervision timeout (in half slots)
    uint32_t                         timeout;
#endif //(BLE_OBSERVER)
    /// Group anchor bluetooth timestamp in microseconds
    uint32_t                         anchor_bts;
    /// Group anchor timestamp - half slot part
    uint32_t                         anchor_ts;
    /// ISO Interval (in microseconds) - all streams have the same ISO interval
    uint32_t                         iso_interval_us;
    /// Group anchor timestamp - half microseconds part
    uint16_t                         anchor_bit_off;
    /// ISO Interval (in half slots) - all streams have the same ISO interval
    uint16_t                         iso_interval;
#if (BLE_OBSERVER)
    /// Offset with broadcast receiver (in half microseconds)
    int32_t                          receiver_offset;
    /// Drift accumulated between expected sync time and effective sync time (in half us)
    int32_t                          sync_drift_acc;
    /// Value of CLKN when the last sync has been detected (in half slots)
    uint32_t                         last_sync_ts;
    /// Sleep clock accuracy (in ppm)
    uint16_t                         sca;
    /// Default reception window size (0 if sync detected, 30hus or 300hus according to big offset unit)
    uint16_t                         def_rx_win_size;
    /// Event counter of the first BIG event the device attempted to receive (broadcast receiver only)
    uint16_t                         first_big_evt_cnt;
    /// Data path peer synchronization enable bit field (Bit 15:0: enable peer sync for BIS)
    uint16_t                         dp_peer_sync_en_bf;
#endif //(BLE_OBSERVER)
    /// Data path peer synchronization enable bit field (Bit 15:0: enable local sync for BIS)
    uint16_t                         dp_local_sync_en_bf;
    /// Event counter
    uint16_t                         big_evt_cnt;
    /// Payload counter (39 bits) - Increment by BN after each iso event
    uint16_t                         bis_pkt_cnt[3];
    /// Stream info (@see enum lld_big_info_fields)
    uint16_t                         big_info;
    /// Total number of sub events for this BIG
    uint16_t                         big_nse;
#if (BLE_BROADCASTER)
    /// Pointer to the control sub-event message to transmit
    uint16_t                         ctrl_se_em_ptr;
    /// Pointer to the next control sub-event message to transmit
    uint16_t                         next_ctrl_se_em_ptr;
#endif //(BLE_OBSERVER)
    /// Group Handle
    uint8_t                          grp_hdl;
#if (BLE_OBSERVER)
    /// Activity identifier of periodic sync driver
    uint8_t                          per_sync_id;
#endif //(BLE_OBSERVER)
    /// Priority index
    uint8_t                          prio_idx;
    /// Stop reason
    uint8_t                          stop_reason;
    /// Number of subevents per BIS (Range 0x01-0x1F)
    uint8_t                          bis_nse;
    /// Maximum PDU size (in bytes)
    uint8_t                          max_pdu;
    /// Number of new packet per ISO interval (Burst Number) (0: no new packet -  Range 0x01-0x1F)
    uint8_t                          bn;
    /// Number of monitored data packets for each BIS
    /// (provide number of elements in @see lld_bis_env p_data_array)
    uint8_t                          nb_data_pkt;
    /// Index in @see lld_bis_env p_data_array of first packet handled during BIS event
    uint8_t                          data_pos;
    /// Receive / Transmit PHY Rate (0: 1Mbps | 1: 2 Mbps | 2: 125 Kbps | 3: 500 Kbps)
    uint8_t                          rate;
    /// Current channel map used for hopping computation
    struct le_ch_map                 ch_map;

    // -- Scan information
#if (BLE_BROADCASTER)
    /// Channel Map for channel assessment
    struct le_ch_map                 scan_ch_map;
    /// Channel index for channel assess
    uint8_t                          scan_chidx;
#endif //(BLE_BROADCASTER)
    /// CSSN value
    uint8_t                          cssn;
    /// Remaining number of BIS Control reception/transmission attempts
    uint8_t                          nb_cntl_tx;
#if (BLE_BROADCASTER)
    /// Number of Control transmission attempts for the next BIG Control PDU
    uint8_t                          next_nb_cntl_tx;
#endif //(BLE_BROADCASTER)
    // -- Event ongoing information
    /// Sub event index, indicate index of next subevent processed by HW
    uint8_t                          in_proc_se_idx;
    /// Sub event index for programming, indicate index of next subevent to be programmed in ET
    uint8_t                          prog_se_idx;
    /// Index of first sub-event in list of packed sub-events.
    uint8_t                          ref_se_idx;
    /// Number of sub events currently programmed
    uint8_t                          nb_prog;

    /// Pointer to data offset mapping array
    struct lld_bi_data_map           data_map[__ARRAY_EMPTY];
};

/// LLD BIS structure
struct lld_bis_env
{
    /// List header
    struct co_list_hdr               hdr;

    // -- ISO stream info
    /// Hopping specific information
    struct lld_iso_hop_inf           hop_inf;
    /// Sub event interval (in half microseconds)
    uint32_t                         sub_interval;
    /// BIS offset in the BIG (in half microseconds) - Time from the BIG anchor to BIS anchor
    uint32_t                         bis_offset_in_big;
#if (BLE_BROADCASTER)
    /// mask used to ensure that data are initiated with ISOAL in correct order
    uint32_t                         data_init_mask;
#endif // (BLE_BROADCASTER)
#if (BLE_OBSERVER)
    /// Value of CLKN when the last correct CRC packet is received
    uint32_t                         last_crc_ok_ts;
#endif // (BLE_OBSERVER)
    /// Duration of a sub event (in half microseconds)
    uint16_t                         duration;
    /// Pointer to the exchange memory where hopping sequences are computed
    uint16_t                         hop_ptr[2];
    /// Group Handle
    uint8_t                          grp_hdl;
    /// Activity ID
    uint8_t                          act_id;
    /// BIS information (@see enum lld_bis_info_fields)
    uint8_t                          bis_info;

    // -- BIS Specific
    /// Index of next ISO descriptor to be used for programming
    uint8_t                          next_desc_idx;
    /// Array of used ISO descriptors
    uint8_t                          iso_descs_idx[BLE_NB_ISODESC_PER_BIS];

    /// Flush Counter - Incremented each time a sub event is programmed
    uint8_t                          flush_cnt;
    /// Indicate the environment is dedicated to the control sub-event
    bool                             ctrl_se;

    // -- BIS Data Specific
    /// Pointer to data structures used for this BIS - Index in this array provides the index
    /// of the buffer to be used
    struct lld_bi_data               data[__ARRAY_EMPTY];
};


#if !(BLE_BIS_IN_ROM)

/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD BIS environment variable
__STATIC struct lld_bis_env *lld_bis_env[BLE_ACTIVITY_MAX];
/// LLD BIG environment variable
__STATIC struct lld_big_env *lld_big_env[BLE_ISO_GROUP_MAX];

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void lld_bi_prog(struct lld_big_env *p_big, uint8_t current_prio);
__STATIC void lld_bi_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
__STATIC void lld_bi_evt_start_cbk(struct sch_arb_elt_tag *p_elt);
__STATIC void lld_bi_evt_stop_cbk(struct sch_arb_elt_tag *p_elt);
__STATIC void lld_bi_evt_canceled_cbk(struct sch_arb_elt_tag *p_elt);
__STATIC bool lld_bi_end_subevt(struct lld_bis_env *p_bis, struct lld_big_env *p_big, uint8_t se_status);
__STATIC void lld_bi_alarm_cbk(struct sch_alarm_tag *p_alarm);
__STATIC void lld_bi_sched(struct lld_big_env *p_big, uint32_t clock);
__STATIC void lld_bis_skip_subevent(struct lld_bi_se *p_se, bool prog);

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Free an BIS structure after a stop request. This function stops the
 * data path if started and frees the ISO descriptors allocated for the link.
 *
 * @param[in] p_bis        Pointer to the BIS structure
 * @param[in] stop_reason  Reason why the BIS has be stopped
 * @param[in] nb_data_pkt  Number of monitored data packets for each BIS
 * @param[in] reset        Used to know if a reset is on-going
 ****************************************************************************************
 */
__STATIC void lld_bis_free(struct lld_bis_env *p_bis, uint8_t stop_reason, uint8_t nb_data_pkt, bool reset)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_FREE_BIT, p_bis, stop_reason, nb_data_pkt, reset);

    if (!reset)
    {
        // Counter
        uint8_t cnt;

        if (!p_bis->ctrl_se)
        {
            // Stop ISOAL
            lld_isoal_stop(p_bis->act_id, stop_reason);

            // Free ISO Buffers
            for (cnt = 0; cnt < nb_data_pkt; cnt++)
            {
                ble_util_buf_iso_free(p_bis->data[cnt].buf_idx);
            }
        }

        // Free ISO descriptors
        for (cnt = 0; cnt < BLE_NB_ISODESC_PER_BIS; cnt++)
        {
            if (p_bis->iso_descs_idx[cnt] == BLE_UTIL_ISO_INDEX_INVALID)
            {
                break;
            }

            ble_util_isodesc_free(p_bis->iso_descs_idx[cnt]);
        }

        // Update BIS environment
        lld_bis_env[p_bis->act_id] = NULL;

        if (!p_bis->ctrl_se)
        {
            // Hopping sequence free
            ble_util_hop_seq_free(p_bis->hop_ptr[0]);
            ble_util_hop_seq_free(p_bis->hop_ptr[1]);

            // Cancel hopping computation
            lld_iso_hop_cancel(&(p_bis->hop_inf));
        }
    }

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id))), REG_EM_BLE_CS_SIZE, false, false, false);

    // Free the BIS structure
    ke_free(p_bis);
}

/**
 ****************************************************************************************
 * @brief Free Sub-event information for a given BIG structure.
 *
 * @param[in] p_big        Pointer to the BIG structure
 * @param[in] p_list       Pointer to a list that will contain the newly available scheduling
 *                         structures so that they can be reused
 ****************************************************************************************
 */
__STATIC void lld_big_free_se(struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_FREE_SE_BIT, p_big);

    if (p_big->p_se_infos)
    {
        // Free the array containing subevent information for scheduling
        ke_free(p_big->p_se_infos);
        p_big->p_se_infos = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Free an BIG structure and send a LLD_BIG_STOP_IND message to the LLI task.
 *
 * @param[in] p_big        Pointer to the BIG structure
 * @param[in] indicate     Indicate if LLD_BIG_STOP_IND message has to be sent to the LLI task.
 * @param[in] reset        Used to know if a reset is on-going
 ****************************************************************************************
 */
__STATIC void lld_big_free(struct lld_big_env *p_big, bool indicate, bool reset)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_FREE_BIT, p_big, indicate, reset);

    // Free all streams linked with this BIG
    while (co_list_pick(&p_big->list_bis))
    {
        struct lld_bis_env *p_bis = (struct lld_bis_env *)co_list_pop_front(&p_big->list_bis);

        // Free the BIS structure
        lld_bis_free(p_bis, p_big->stop_reason, p_big->nb_data_pkt, reset);
    }

#if (BLE_BROADCASTER)
    // free control sub-event message
    if (p_big->ctrl_se_em_ptr != 0)
    {
        ble_util_buf_llcp_tx_free(p_big->ctrl_se_em_ptr);
    }
#endif // (BLE_BROADCASTER)

    if (indicate)
    {
        // Send the lld_big_sync_estab_ind with status "failed to be established" only when broadcast receiver and sync not yet established
        // In all other cases send the lld_big_stop_ind
        if (!GETB(p_big->big_info, LLD_BIG_RECEIVER) || GETB(p_big->big_info, LLD_BIG_SYNC_ESTAB))
        {
            // Inform link layer controller that driver is stopped for the provided BIG
            struct lld_big_stop_ind *p_msg = KE_MSG_ALLOC(LLD_BIG_STOP_IND, TASK_LLI, TASK_NONE, lld_big_stop_ind);
            p_msg->grp_hdl = p_big->grp_hdl;
            p_msg->reason  = p_big->stop_reason;
            ke_msg_send(p_msg);
        }
        else
        {
            // Inform link layer controller that BIG sync has failed to be established
            struct lld_big_sync_estab_ind *p_msg = KE_MSG_ALLOC(LLD_BIG_SYNC_ESTAB_IND, TASK_LLI, TASK_NONE, lld_big_sync_estab_ind);
            p_msg->grp_hdl = p_big->grp_hdl;
            p_msg->status = CO_ERROR_CONN_FAILED_TO_BE_EST;
            ke_msg_send(p_msg);

        }
    }

    // Stop alarm
    sch_alarm_clear(&p_big->alarm);

    // Free Sub-event information
    lld_big_free_se(p_big);

    // Update BIG environment
    lld_big_env[p_big->grp_hdl] = NULL;

    // Free the BIG structure
    ke_free(p_big);
}

/**
 ****************************************************************************************
 * @brief Set scheduling information for all sub events within a BIG (interlaced case)
 *
 * @param[in] p_big      BIG structure for which the scheduling information are computed
 ****************************************************************************************
 */
__STATIC void lld_big_compute_sched_int(struct lld_big_env *p_big)
{
    // Sub event index
    uint8_t se_idx = 0;
    // Sub event counter
    uint8_t sub_evt_cnt = 0;
    // BIS information
    struct lld_bis_env *p_bis;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_COMPUTE_SCHED_INT_BIT, p_big);

    // Fill array of subevent information
    while (true)
    {
        // Number of subevents to be added for next sub event cnt value
        uint8_t nb_added_se = 0;

        // Get first BIS information
        p_bis = (struct lld_bis_env *)co_list_pick(&p_big->list_bis);

        while (p_bis && (!p_bis->ctrl_se))
        {
            // Check if BIS has been started
            if (GETB(p_bis->bis_info, LLD_BIS_INFO_STARTED))
            {
                if (sub_evt_cnt < p_bis->hop_inf.nse)
                {
                    // Get information for this subevent
                    struct lld_bi_se *p_se = &p_big->p_se_infos[se_idx];

                    p_se->bis_act_id      = p_bis->act_id;
                    p_se->offset          = p_bis->bis_offset_in_big + (sub_evt_cnt * p_bis->sub_interval);
                    p_se->bis_sub_evt_idx = sub_evt_cnt;
                    p_se->nb_packed       = 0;
                    p_se->state           = 0;

                    se_idx++;

                    // Check if a sub event will be added for next sub event counter value
                    if ((sub_evt_cnt + 1) < p_bis->hop_inf.nse)
                    {
                        nb_added_se++;
                    }
                }
            }

            // Get next BIS
            p_bis = (struct lld_bis_env *)(p_bis->hdr.next);
        }

        if (!nb_added_se)
        {
            break;
        }

        sub_evt_cnt++;
    }

    {
        // Get scheduling information for control subevent
        struct lld_bi_se *p_se = &p_big->p_se_infos[p_big->big_nse - 1];

        // Get last BIS
        p_bis = (struct lld_bis_env *)p_big->list_bis.last;

        // Sanity check
        ASSERT_ERR(p_bis->ctrl_se);

        p_se->bis_act_id      = p_bis->act_id;
        p_se->offset          = p_bis->bis_offset_in_big;
        p_se->bis_sub_evt_idx = 0;
        p_se->nb_packed       = 0;
        p_se->state           = 0;
    }
}

/**
 ****************************************************************************************
 * @brief Set scheduling information for all sub events within a BIG (sequential case)
 *
 * @param[in] p_big      BIG structure for which the scheduling information are computed
 ****************************************************************************************
 */
__STATIC void lld_big_compute_sched_seq(struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_COMPUTE_SCHED_SEQ_BIT, p_big);

    // Sub event index
    uint8_t se_idx = 0;
    // Get first BIS information
    struct lld_bis_env *p_bis = (struct lld_bis_env *)co_list_pick(&p_big->list_bis);

    // Fill array of subevent information
    while (p_bis)
    {
        // Check if BIS has been started
        if (GETB(p_bis->bis_info, LLD_BIS_INFO_STARTED))
        {
            // Counter
            uint8_t se_cnt;

            for (se_cnt = 0; se_cnt < p_bis->hop_inf.nse; se_cnt++)
            {
                // Get subevent information
                struct lld_bi_se *p_se = &p_big->p_se_infos[se_idx];

                p_se->bis_act_id      = p_bis->act_id;
                p_se->bis_sub_evt_idx = se_cnt;
                p_se->offset          = p_bis->bis_offset_in_big + (se_cnt * p_bis->sub_interval);
                p_se->state           = 0;
                p_se->nb_packed       = 0;

                se_idx++;
            }
        }
        // Get next BIS
        p_bis = (struct lld_bis_env *)(p_bis->hdr.next);
    }
}

/**
 ****************************************************************************************
 * @brief Compute scheduling information after introduction or deletion of a BIS for
 * a given BIG
 *
 * @param[in] p_big       Pointer to the BIG structure
 *
 * @return CO_ERROR_NO_ERROR if scheduling scheme has been properly established,
 *         CO_ERROR_MEMORY_CAPA_EXCEED if scheduling cannot be established due to lack of memory
 *         resources
 ****************************************************************************************
 */
__STATIC uint8_t lld_big_compute_sched(struct lld_big_env *p_big)
{
    // Returned status
    uint8_t status = CO_ERROR_NO_ERROR;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_COMPUTE_SCHED_BIT, uint8_t, p_big);

    // Allocate array that will contains all sub-events information
    p_big->p_se_infos = ke_malloc_system(sizeof(struct lld_bi_se) * p_big->big_nse, KE_MEM_ENV);

    if (p_big->p_se_infos)
    {
        // Sub event index
        uint8_t             se_idx     = 0;
        // Subevent information
        struct lld_bi_se   *p_se;
        struct lld_bi_se   *p_se_ref;
        // BIS structure for this subevent
        struct lld_bis_env *p_bis;
        // End of subevent offset
        uint32_t            end_se_offset;

        // Compute subevent positions
        if (GETB(p_big->big_info, LLD_BIG_INTERLEAVED))
        {
            lld_big_compute_sched_int(p_big);
        }
        else
        {
            lld_big_compute_sched_seq(p_big);
        }

        // Information of first sub-event of packed sub-events
        p_se     = &(p_big->p_se_infos[0]);
        p_se_ref = &(p_big->p_se_infos[0]);
        p_bis    = LLD_BIS_GET(p_se->bis_act_id);

        // End of subevent offset for reference sub event
        end_se_offset = p_se_ref->offset + p_bis->duration;

        // Now separate subevent into scheduling events based on distance between subevents
        for (se_idx = 0 ; se_idx < p_big->big_nse; se_idx++)
        {
            p_se  = &(p_big->p_se_infos[se_idx]);
            p_bis = LLD_BIS_GET(p_se->bis_act_id);

            // Check distance with end of previously handled subevent
            if ((p_se != p_se_ref) && ((p_se->offset - end_se_offset) > HS_TO_HUS(LLD_BI_MAX_SE_SCHED_DISTANCE)))
            {
                // Use new reference for following sub-event packed together
                p_se_ref = p_se;
            }

            // Compute end of subevent offset
            end_se_offset = p_se->offset + p_bis->duration;

            // Initialize first packed sub-event
            p_se_ref->duration_packed = end_se_offset - p_se_ref->offset;
            p_se_ref->nb_packed      += 1;
        }

        p_big->ref_se_idx = 0;
    }
    else
    {
        // Error, Scheduling computation not performed
        ASSERT_ERR(0);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Generate access address for an BIS
 *
 * To derive the Access Address for an BIS with an BIS number BIS(n) the Link Layer shall calculate
 * an diversifier as shown below.
 * D = ((35 x N) + 42) MOD 128 = D7D6D5D4D3D2D1D0.
 * and set diversifier word as
 * DW = D0D0D0D0 D0D0D1D6 D1b0D5D4 b0D3D2b0 00000000 00000000b
 *
 * AA = DW XOR SAA
 *
 * @param[in] bis_idx          BIS index (between 1 and n, with n = number of streams in the BIG)
 * @param[in] seed_acc_addr    Seed access address of the Group
 *
 * @return Access address to be used for the BIS
 ****************************************************************************************
 */
__STATIC uint32_t lld_bis_aa_gen(uint8_t bis_idx, uint32_t seed_acc_addr)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_AA_GEN_BIT, uint32_t, bis_idx, seed_acc_addr);

    // diversifier ((35 x N) + 42) MOD 128
    uint8_t d = ((35 * bis_idx) + 42) & 0x7F;
    // diversifier word
    uint32_t dw = 0x00000000;

    if ((d & CO_BIT(0)) != 0)
    {
        dw |= 0xFC000000;
    }
    if ((d & CO_BIT(1)) != 0)
    {
        dw |= 0x02800000;
    }
    if ((d & CO_BIT(2)) != 0)
    {
        dw |= 0x00020000;
    }
    if ((d & CO_BIT(3)) != 0)
    {
        dw |= 0x00040000;
    }
    if ((d & CO_BIT(4)) != 0)
    {
        dw |= 0x00100000;
    }
    if ((d & CO_BIT(5)) != 0)
    {
        dw |= 0x00200000;
    }
    if ((d & CO_BIT(6)) != 0)
    {
        dw |= 0x01000000;
    }

    // Return access address: AA = DW XOR SAA
    return (dw ^ seed_acc_addr);
}

/**
 ****************************************************************************************
 * @brief Insert a BIS structure in the list of BISes for a given BIG.
 * BIS elements are sorted by BIS offset values
 *
 * @param[in] p_bis     BIS structure
 * @param[in] p_big     BIG structure
 ****************************************************************************************
 */
__STATIC void lld_bis_insert(struct lld_bis_env *p_bis, struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_INSERT_BIT, p_bis, p_big);

    // Get first BIS element
    struct lld_bis_env *p_list_bis = (struct lld_bis_env *)co_list_pick(&p_big->list_bis);

    if (!p_list_bis)
    {
        // Insert the BIS at the beginning of the list
        co_list_push_front(&p_big->list_bis, &p_bis->hdr);
    }
    else
    {
        while (p_list_bis)
        {
            if (p_bis->bis_offset_in_big <= p_list_bis->bis_offset_in_big)
            {
                // Insert the BIS
                co_list_insert_before(&p_big->list_bis, &p_list_bis->hdr, &p_bis->hdr);
                break;
            }

            // Get next element
            p_list_bis = (struct lld_bis_env *)(p_list_bis->hdr.next);
        }

        if (!p_list_bis)
        {
            // Insert the BIS at the end of the list
            co_list_push_back(&p_big->list_bis, &p_bis->hdr);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Allocate an BIG structure
 *
 * @param[in] p_params       BIG Driver Parameters
 * @param[in] p_big_info            Pointer to BIG info (used to generate ACAD data on broadcaster side)
 * @param[in] nb_pt_evt             Number of pre-transmitted events
 * @param[out] p_big_start_time     Pointer to store BIG start time
 *
 * @return Pointer to the allocated BIG structure.
 ****************************************************************************************
 */
__STATIC struct lld_big_env *lld_big_alloc(struct lld_big_params *p_params, struct big_info *p_big_info, uint8_t nb_pt_evt,
        struct lld_big_start_time *p_big_start_time)
{
    // Allocate a new BIG structure
    struct lld_big_env *p_big = (struct lld_big_env *)ke_malloc_system(sizeof(struct lld_big_env) +
                                p_big_info->nse * sizeof(struct lld_bi_data_map), KE_MEM_ENV);

    // Check if structure has been allocated
    if (p_big)
    {
        do
        {
            // Pointer to scheduling arbiter element for BIS data
            struct sch_arb_elt_tag *p_elt = &p_big->elt;
            // Pointer to alarm
            struct sch_alarm_tag *p_alarm = &p_big->alarm;
            // Counter
            uint8_t cnt;
            // Position in data offset array
            uint8_t idx = 0;

            // Clear content of the structure
            memset(p_big, 0, sizeof(struct lld_big_env));

            // Fill data offset mapping array - Immediate repetition part
            for (cnt = 0; cnt < p_big_info->irc; cnt++)
            {
                // Counter
                uint8_t bn_cnt;

                for (bn_cnt = 0; bn_cnt < p_big_info->bn; bn_cnt++)
                {
                    p_big->data_map[idx].offset = bn_cnt;
                    p_big->data_map[idx].last = ((cnt + 1) == p_big_info->irc);

                    idx++;
                }
            }


            // Fill data offset mapping array - Pre-transmission part
            for (cnt = 1; cnt <= nb_pt_evt; cnt++)
            {
                // Counter
                uint8_t bn_cnt;
                // Offset
                uint8_t offset = (cnt * p_big_info->pto * p_big_info->bn);

                for (bn_cnt = 0; bn_cnt < p_big_info->bn; bn_cnt++)
                {
                    p_big->data_map[idx].offset = offset + bn_cnt;
                    p_big->data_map[idx].last   = false;

                    idx++;
                }
            }

            // Configure the BIG
            p_big->grp_hdl           = p_params->grp_hdl;
            p_big->rate              = p_params->rate;

            // One subevent for update
            p_big->big_nse           = (p_big_info->nse * p_big_info->num_bis) + 1;
            p_big->bis_nse           = p_big_info->nse;
            p_big->bn                = p_big_info->bn;
            p_big->max_pdu           = p_params->max_pdu;
            p_big->nb_data_pkt       = p_big_info->bn + (p_big_info->pto * (p_big_info->nse - (p_big_info->irc * p_big_info->bn)));
            p_big->iso_interval      = FRAME_TO_HS(p_big_info->iso_interval);
            p_big->iso_interval_us   = FRAME_TO_US(p_big_info->iso_interval);
            p_big->prio_idx          = (p_params->role == ROLE_BROADCASTER) ? RWIP_PRIO_M_BIS_IDX : RWIP_PRIO_S_BIS_IDX;

            SETB(p_big->big_info, LLD_BIG_ENCRYPT,    p_big_info->encrypted);
            SETB(p_big->big_info, LLD_BIG_RECEIVER, (p_params->role == ROLE_BROADCAST_RECEIVER));
            SETB(p_big->big_info, LLD_BIG_INTERLEAVED, (p_params->packing == ISO_PACKING_INTERLEAVED));
            SETB(p_big->big_info, LLD_BIG_HOP_TOG,    1);

            // Save channel map
            memcpy(&(p_big->ch_map), &(p_big_info->ch_map), LE_CH_MAP_LEN);

#if (BLE_BROADCASTER)
            if (p_params->role == ROLE_BROADCASTER)
            {
                // Set channel map for channel scan as all channels enabled
                memset(&(p_big->scan_ch_map.map[0]), 0xFF, LE_CH_MAP_LEN);
                p_big->scan_ch_map.map[LE_CH_MAP_LEN - 1] &= 0x1F;

                // Initialize with channel 36 as channel will be incremented by 1
                p_big->scan_chidx    = 36;
            }
            else
#endif //(BLE_BROADCASTER)
            {
#if (BLE_OBSERVER)
                p_big->cssn    = LLD_BI_CSSN_INVALID;
                // convert timeout from 10ms step to half slot step (multiply by 32).
                p_big->timeout = (((uint32_t)p_params->sync_timeout) << 5);
#endif //(BLE_OBSERVER)
            }

            p_alarm->cb_alarm    = &lld_bi_alarm_cbk;
            p_elt->cb_cancel     = &lld_bi_evt_canceled_cbk;
            p_elt->cb_start      = &lld_bi_evt_start_cbk;
            p_elt->cb_stop       = &lld_bi_evt_stop_cbk;
            p_elt->current_prio  = rwip_priority[p_big->prio_idx].value;
            p_elt->time.hus      = 0;
            p_elt->stop_latency  = 0;
            SCH_ARB_ASAP_STG_SET(p_elt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(p_big->prio_idx));

            // Update BIG environment
            lld_big_env[p_params->grp_hdl] = p_big;

#if (BLE_BROADCASTER)
            // Compute timestamp of BIG first subevent
            if (p_params->role == ROLE_BROADCASTER)
            {
                // Ensure that first event can be programmed
                uint32_t clock = CLK_ADD_2(lld_read_clock(), rwip_prog_delay * 2);

                p_big->anchor_ts      = CLK_ADD_2(clock, p_big->iso_interval - CO_MOD(clock, p_big->iso_interval) + p_params->act_offset);
                p_big->anchor_bit_off = 0;
            }
            else
#endif //(BLE_BROADCASTER)
            {
#if (BLE_OBSERVER)
                uint32_t big_evt_cnt;
                struct lld_sync_info_for_big per_sync_info;
                uint32_t big_offset;

                // Retrieve timing information provided by periodic sync activity
                lld_sync_info_for_big_get(p_params->per_sync_id, p_params->ref_event_cnt, &per_sync_info);

                p_big->sca             = co_sca2ppm[per_sync_info.ca];
                // keep information about default window size
                p_big->def_rx_win_size = US_TO_HUS(p_big_info->big_offset_unit ? 300 : 30);
                // compute big offset as offset present in packet + half of offset step in half microseconds
                big_offset             = p_big_info->big_offset * p_big->def_rx_win_size + (p_big->def_rx_win_size >> 1);
                // Compute new BIG parameters
                p_big->last_sync_ts    = per_sync_info.last_sync_ts;
                p_big->anchor_ts       = CLK_ADD_2(per_sync_info.ref_timestamp, (big_offset / HALF_SLOT_SIZE));
                p_big->anchor_bit_off  = per_sync_info.ref_bit_off + CO_MOD(big_offset, HALF_SLOT_SIZE);
                if (p_big->anchor_bit_off > HALF_SLOT_SIZE)
                {
                    p_big->anchor_bit_off -= HALF_SLOT_SIZE;
                    p_big->anchor_ts       = CLK_ADD_2(p_big->anchor_ts, 1);
                }
                p_big->sync_drift_acc = per_sync_info.sync_drift_acc;

                // Save payload counter
                p_big->bis_pkt_cnt[0] = co_read16p(&p_big_info->bis_pkt_cnt[0]);
                p_big->bis_pkt_cnt[1] = co_read16p(&p_big_info->bis_pkt_cnt[2]);
                p_big->bis_pkt_cnt[2] = co_read16p(&p_big_info->bis_pkt_cnt[4]) & 0x7F;

                // Compute BIG event counter (payload_counter / BN)
                // divide MSB of BIS packet counter by BN
                big_evt_cnt           = ((co_read16p(&p_big_info->bis_pkt_cnt[3]) & 0x7FFF) / p_big_info->bn) << 24;
                // keep rest of the previous euclidian function before doing division of LSB
                big_evt_cnt          += ((CO_MOD((co_read16p(&p_big_info->bis_pkt_cnt[3]) & 0x7FFF), p_big_info->bn) << 24)
                                         + co_read24p(&p_big_info->bis_pkt_cnt[0])) / p_big_info->bn;

                p_big->big_evt_cnt = big_evt_cnt & 0xFFFF;
                p_big->per_sync_id = p_params->per_sync_id;
#endif //(BLE_OBSERVER)
            }
            // convert anchor to bluetooth timestamp
            p_big->anchor_bts     = rwip_bt_time_to_bts(p_big->anchor_ts, p_big->anchor_bit_off);

            p_big_start_time->big_start_hs = p_big->anchor_ts;
            p_big_start_time->big_start_us = p_big->anchor_bit_off;

        }
        while (0);
    }

    return (p_big);
}

/**
 ****************************************************************************************
 * @brief Allocate an BIS structure.
 *
 * @param[in] p_big_params      BIG Parameters for first BIS created in the new Group
 * @param[in] p_bis_params      BIS parametrs
 * @param[in] p_big             BIG structure
 * @param[in] seed_acc_addr     Seed Access address used to generate BIS Access addresses
 * @param[in] sub_interval      Sub-event interval in us
 * @param[in] bis_spacing       BIS spacing in us
 *
 * @return Pointer to the allocated BIS structure.
 ****************************************************************************************
 */
__STATIC struct lld_bis_env *lld_bis_alloc(struct lld_big_params *p_big_params, struct lld_bis_params *p_bis_params,
        struct lld_big_env *p_big, uint32_t seed_acc_addr, uint32_t sub_interval,
        uint32_t bis_spacing)
{
    // Stream index used for control sub-event
    bool ctrl_se = (p_bis_params->idx == 0);

    // Number of data packets
    uint8_t nb_data_pkt = (ctrl_se) ? 1 : p_big->nb_data_pkt;

    // Allocate a new BIS structure
    struct lld_bis_env *p_bis = (struct lld_bis_env *)ke_malloc_system(sizeof(struct lld_bis_env) +
                                (nb_data_pkt * sizeof(struct lld_bi_data)), KE_MEM_ENV);

    if (p_bis != NULL)
    {
        // Counter
        uint8_t cnt;
        // Initialize content of allocated array
        memset(p_bis, 0, sizeof(struct lld_bis_env) + (nb_data_pkt * sizeof(struct lld_bi_data)));

        // Initialize event parameters (connection part)
        p_bis->act_id   = p_bis_params->act_id;
        p_bis->grp_hdl  = p_big_params->grp_hdl;
        p_bis->ctrl_se = ctrl_se;

#if (BLE_BROADCASTER)
        p_bis->data_init_mask = CO_BIT(nb_data_pkt - p_big->bn) - 1;
#endif // (BLE_BROADCASTER)

        // Compute the access address to use for BIS
        p_bis->hop_inf.acc_code = lld_bis_aa_gen(p_bis_params->idx, seed_acc_addr);
        memcpy(p_bis->hop_inf.ch_map.map, p_big->ch_map.map, LE_CH_MAP_LEN);

        if (ctrl_se)
        {
            p_bis->hop_inf.nse = 1;
            p_bis->duration    = US_TO_HUS(p_big_params->upd_air_dur);
            p_bis->bis_offset_in_big = US_TO_HUS(p_big_params->update_offset);

            // Only one ISO descriptor is needed
            p_bis->iso_descs_idx[0] = ble_util_isodesc_alloc();

            // Allocate TX ISO descriptors
            for (cnt = 1; cnt < BLE_NB_ISODESC_PER_BIS; cnt++)
            {
                p_bis->iso_descs_idx[cnt] = BLE_UTIL_ISO_INDEX_INVALID;
            }

#if (BLE_OBSERVER)
            if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
            {
                SETB(p_bis->data[0].data_info, LLD_BIS_DATA_DONE, 1);
            }
#endif //(BLE_OBSERVER)
        }
        else
        {
            p_bis->hop_inf.nse    = p_big->bis_nse;
            p_bis->duration       = US_TO_HUS(p_big_params->air_exch_dur);
            p_bis->sub_interval   = US_TO_HUS(sub_interval);
            p_bis->bis_offset_in_big = (p_bis_params->idx - 1) * US_TO_HUS(bis_spacing);
#if (BLE_OBSERVER)
            p_bis->last_crc_ok_ts = p_big->last_sync_ts;
#endif // (BLE_OBSERVER)

            GLOBAL_INT_DISABLE();
            // Allocate TX ISO descriptors
            for (cnt = 0; cnt < BLE_NB_ISODESC_PER_BIS; cnt++)
            {
                p_bis->iso_descs_idx[cnt] = ble_util_isodesc_alloc();
            }

            // Allocate ISO Buffers
            for (cnt = 0; cnt < nb_data_pkt; cnt++)
            {
                p_bis->data[cnt].buf_idx = ble_util_buf_iso_alloc();
            }
            GLOBAL_INT_RESTORE();

            // Hopping sequence allocation
            p_bis->hop_ptr[0] = ble_util_hop_seq_alloc();
            p_bis->hop_ptr[1] = ble_util_hop_seq_alloc();
        }


        // Consider the BIS as started
        SETB(p_bis->bis_info, LLD_BIS_INFO_STARTED, 1);

        // Update BIS environment
        lld_bis_env[p_bis_params->act_id] = p_bis;

        // Insert the BIS in the list of BISes for the provided group
        lld_bis_insert(p_bis, p_big);
    }

    return (p_bis);
}

/**
 ****************************************************************************************
 * Request to compute the Hopping scheme for a specific CIS event
 *
 * @param[in] p_cis      CIS information
 * @param[in] hop_em_ptr Exchange memory pointer that will contain hoping channels
 * @param[in] evt_cnt    Reference Event counter
 ****************************************************************************************
 */
__STATIC void lld_bi_compute_hop_scheme(struct lld_big_env *p_big, uint16_t evt_cnt)
{
    struct lld_bis_env *p_bis;

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_COMPUTE_HOP_SCHEME_2_FUNC_BIT, p_big, evt_cnt);

    // Check if the instant associated with update procedure is reached or over-stepped
    if ((p_big->instant_proc.type == INSTANT_PROC_CH_MAP_UPD)
            && BLE_UTIL_INSTANT_PASSED(p_big->instant_proc.instant, evt_cnt))
    {
        memcpy(p_big->ch_map.map, p_big->instant_proc.data.ch_map.map, LE_CH_MAP_LEN);

        // Update BIS channel map
        p_bis = (struct lld_bis_env *) co_list_pick(&(p_big->list_bis));
        while (p_bis != NULL)
        {
            if (!p_bis->ctrl_se)
            {
                memcpy(p_bis->hop_inf.ch_map.map, p_big->ch_map.map, LE_CH_MAP_LEN);
            }
            p_bis = (struct lld_bis_env *) p_bis->hdr.next;
        }

        // Clear update procedure
        p_big->instant_proc.type = INSTANT_PROC_NO_PROC;

        // mark that control sub-event channel map must be updated
        SETB(p_big->big_info, LLD_BIG_CH_MAP_UPD, 1);
    }

    // compute hopping sequence for all BIS
    p_bis = (struct lld_bis_env *) co_list_pick(&(p_big->list_bis));
    while (p_bis != NULL)
    {
        if (!p_bis->ctrl_se)
        {
            p_bis->hop_inf.em_ptr  = p_bis->hop_ptr[!GETB(p_big->big_info, LLD_BIG_HOP_TOG)];
            p_bis->hop_inf.evt_cnt = evt_cnt;

            lld_iso_hop_compute(&(p_bis->hop_inf));
        }
        p_bis = (struct lld_bis_env *) p_bis->hdr.next;
    }
}


/**
 ****************************************************************************************
 * @brief Handle load of ISO buffer
 ****************************************************************************************
 */
__STATIC void lld_bis_data_load(struct lld_big_env *p_big, struct lld_bis_env *p_bis, struct lld_bi_data *p_data,
                                uint8_t data_offset, uint16_t *p_pld_cnt)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_DATA_LOAD_BIT, 5, p_big, p_bis, p_data, data_offset, p_pld_cnt);

    if (p_pld_cnt != NULL)
    {
        // Retrieve payload counter
        memcpy(&p_pld_cnt[0], &p_big->bis_pkt_cnt[0], sizeof(p_big->bis_pkt_cnt));
        LLD_ISO_PYLD_CNT_ADD(p_pld_cnt, data_offset);
    }

    if (!GETB(p_data->data_info, LLD_BIS_DATA_PROG))
    {
        DBG_SWDIAG(BI_DATA, LOAD, 1);

#if (BLE_BROADCASTER)
        if (!GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            // At BIS initialization, the loading of data is skipped for the N first packets
            if ((p_bis->data_init_mask & CO_BIT(data_offset)) == 0)
            {
                // compute the BTS time for the buffer
                // TRUNC(data_offset / p_big->bn) is used to retrieve group interval offset
                uint32_t bts = p_big->anchor_bts + (p_big->iso_interval_us * (data_offset / p_big->bn));

                // Compute payload counter for this packet
                lld_isoal_tx_get(p_bis->act_id, p_data->buf_idx, bts, false);
            }
        }
        else
#endif //(BLE_BROADCASTER)
        {
#if (BLE_OBSERVER)
            // ensure that RX buffer is ready to receive
            em_ble_rxisobufsetup_pack(p_data->buf_idx, /*invl*/ LLD_ISO_INVL_SYNC_ERR, /*llid*/ 0, /*length*/ 0);
#endif //(BLE_OBSERVER)
        }

        // Keep in mind that buffer has been programmed
        SETB(p_data->data_info, LLD_BIS_DATA_PROG, 1);
#if (BLE_BROADCASTER)
        p_bis->data_init_mask &= ~CO_BIT(data_offset);
#endif // (BLE_BROADCASTER)

        DBG_SWDIAG(BI_DATA, LOAD, 0);
    }
#if (BLE_BROADCASTER)
    else
    {
        ASSERT_ERR((p_bis->data_init_mask & CO_BIT(data_offset)) == 0);
    }
#endif // (BLE_BROADCASTER)
}

/**
 ****************************************************************************************
 * @brief Handle release of ISO buffer when no more used
 ****************************************************************************************
 */
__STATIC void lld_bis_data_release(struct lld_big_env *p_big, struct lld_bis_env *p_bis, struct lld_bi_data *p_data,
                                   uint8_t data_offset)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_DATA_RELEASE_BIT, p_big, p_bis, p_data, data_offset);
#if (BLE_OBSERVER)
    if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
    {
        lld_isoal_rx_done(p_bis->act_id, p_data->buf_idx, p_big->anchor_bts, !GETB(p_data->data_info, LLD_BIS_DATA_DONE));
    }
#endif //(BLE_OBSERVER)

    // prepare for next buffer
    SETB(p_data->data_info, LLD_BIS_DATA_PROG, 0);
    SETB(p_data->data_info, LLD_BIS_DATA_DONE, 0);
}

/**
 ****************************************************************************************
 * @brief Program content of RX or TX ISO descriptor depending on our role.
 ****************************************************************************************
 */
__STATIC void lld_bis_data_prog(struct lld_bis_env *p_bis, struct lld_big_env *p_big, struct lld_bi_data *p_data,
                                struct lld_bi_se *p_se, uint8_t offset)
{
    // Get index of descriptor to be used
    uint8_t idx = p_bis->iso_descs_idx[p_bis->next_desc_idx];
    // Compute payload counter for this packet
    uint16_t pld_cnt[3];

    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_DATA_PROG_FUNC_BIT, 5, p_bis, p_big, p_data, p_se, offset);

    DBG_SWDIAG(BI_DATA, PROG, 1);

    if (!p_bis->ctrl_se)
    {
#if (BLE_BROADCASTER)
        if (!GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            // check if some data are not yet initialized before current data packet.
            // it's required that TX at ISOAL level are filled in the correct order.
            uint32_t data_init_mask = p_bis->data_init_mask & (CO_BIT(offset) - 1);
            while (data_init_mask != 0)
            {
                uint8_t data_offset = co_ctz(data_init_mask);
                lld_bis_data_load(p_big, p_bis, &(p_bis->data[data_offset]), data_offset, pld_cnt);
                data_init_mask = p_bis->data_init_mask & (CO_BIT(offset) - 1);
            }
        }
#endif // (BLE_BROADCASTER)

        // Load data for BIS
        lld_bis_data_load(p_big, p_bis, p_data, offset, pld_cnt);
    }
    else
    {
        // offset always equals to zero for control sub-event
        memcpy(&pld_cnt[0], &p_big->bis_pkt_cnt[0], 3 * sizeof(uint16_t));
    }

    do
    {
#if (BLE_OBSERVER)
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            if (!em_ble_rxisoptr_rxdone_getf(idx))
            {
                ASSERT_INFO(0, p_bis->act_id, idx);
                break;
            }

            // Set Descriptor info
            em_ble_rxisocnt0_rxpld_cnt0_setf(idx, pld_cnt[0]);
            em_ble_rxisocnt1_rxpld_cnt1_setf(idx, pld_cnt[1]);
            em_ble_rxisocnt2_pack(idx, p_bis->flush_cnt, pld_cnt[2] & EM_BLE_RXPLD_CNT2_MASK);

            // Set Buffer pointer
            em_ble_rxisobufptr_setf(idx, ((!p_bis->ctrl_se) ? ble_util_buf_iso_emptr_get(p_data->buf_idx) : 0) >> 2);
            // Mark descriptor as valid
            em_ble_rxisoptr_rxdone_setf(idx, 0);
        }
        else
#endif //(BLE_OBSERVER)
        {
#if (BLE_BROADCASTER)
            // LLID, CSTF and CSSN values
            uint8_t cstf = 0;
            uint8_t cssn = 0;

            // Check if descriptor can be used
            if (!em_ble_txisoptr_txdone_getf(idx))
            {
                ASSERT_INFO(0, p_bis->act_id, idx);
                break;
            }

            // ISO Event
            if (!p_bis->ctrl_se)
            {
                cstf = (p_big->nb_cntl_tx > 0);
            }

            cssn = p_big->cssn;

            // Set payload counter
            em_ble_txisocnt0_txpld_cnt0_setf(idx,  pld_cnt[0]);
            em_ble_txisocnt1_txpld_cnt1_setf(idx,  pld_cnt[1]);
            em_ble_txisocnt2_pack(idx, p_bis->flush_cnt, pld_cnt[2] & EM_BLE_TXPLD_CNT2_MASK);
            em_ble_txbisph_pack(idx, /*txbisrfu*/    0,
                                /*txcstf*/   cstf,
                                /*txcssn*/   cssn);

            // Set Buffer pointer
            em_ble_txisobufptr_setf(idx, ((!p_bis->ctrl_se) ? ble_util_buf_iso_emptr_get(p_data->buf_idx)
                                          : p_big->ctrl_se_em_ptr) >> 2);
            // Mark descriptor as valid
            em_ble_txisoptr_txdone_setf(idx, 0);
#endif //(BLE_BROADCASTER)
        }

        // Keep descriptor index
        p_se->desc_idx = idx;

        // Indicate data has been programmed
        SETB(p_se->state, LLD_BI_SE_STATUS_DATA_PROG, true);

        if (!p_bis->ctrl_se)
        {
            // Update index of next ISO descriptor structure to be used
            CO_VAL_INC(p_bis->next_desc_idx, BLE_NB_ISODESC_PER_BIS);
        }
    }
    while (0);

    DBG_SWDIAG(BI_DATA, PROG, 0);
}

/**
 ****************************************************************************************
 * @brief Schedule next anchor points for next sub event to be scheduled.
 ****************************************************************************************
 */
__STATIC void lld_bi_alarm_cbk(struct sch_alarm_tag *p_alarm)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_ALARM_CBK_2_FUNC_BIT, p_alarm);

    // Get Group for which alarm has been triggered
    struct lld_big_env *p_big = (struct lld_big_env *)((uint32_t)p_alarm - offsetof(struct lld_big_env, alarm));

    // Clear bit indicating that alarm is running
    SETB(p_big->big_info, LLD_BIG_ALARM, 0);

    if (GETB(p_big->big_info, LLD_BIG_ALARM_PROG))
    {
        // Get Reference sub-event scheduling structure
        struct lld_bi_se *p_ref_se = &(p_big->p_se_infos[p_big->ref_se_idx]);
        struct lld_bi_se *p_se = &p_big->p_se_infos[p_big->prog_se_idx];
        struct lld_bis_env *p_bis = LLD_BIS_GET(p_se->bis_act_id);

        // Program next sub-event
        lld_bi_prog(p_big, p_big->elt.current_prio);

        // If at least one sub-event remaining, program a timer for next sub-event
        if (p_big->prog_se_idx < (p_big->ref_se_idx + p_ref_se->nb_packed))
        {
            struct lld_bi_se *p_next_se = &p_big->p_se_infos[p_big->prog_se_idx];

            // Get pointer to alarm
            struct sch_alarm_tag *p_alarm = &p_big->alarm;
            ASSERT_ERR(GETB(p_big->big_info, LLD_BIG_ALARM) == 0);

            int32_t bit_off;
            uint32_t target_clock;

#if (BLE_OBSERVER)
            // Calculate the expected synchronization for the sub-event
            if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
            {
                int32_t exp_sync_bit_off = p_se->exp_sync_bit_off + (p_next_se->offset - p_se->offset);
                p_next_se->exp_sync_ts = CLK_ADD_2(p_se->exp_sync_ts, exp_sync_bit_off / HALF_SLOT_SIZE);
                exp_sync_bit_off = CO_MOD(exp_sync_bit_off, HALF_SLOT_SIZE);
                p_next_se->exp_sync_bit_off = exp_sync_bit_off;
            }

            // If sync found in one of the previous sub-events, adjust the programming timestamp
            if (GETB(p_big->big_info, LLD_BIG_RECEIVER) && GETB(p_big->big_info, LLD_BIG_SYNC))
            {
                // Timings are computed from the originally expected synchronization and the timings from last synchronization
                bit_off = p_next_se->exp_sync_bit_off + p_big->receiver_offset - US_TO_HUS(BLE_NORMAL_WIN_SIZE) / 2;
                target_clock = p_next_se->exp_sync_ts;

                while (bit_off < 0)
                {
                    bit_off += HALF_SLOT_SIZE;
                    target_clock = CLK_SUB(target_clock, 1);
                }

                target_clock = CLK_ADD_2(target_clock, bit_off / HALF_SLOT_SIZE);
                bit_off = CO_MOD(bit_off, HALF_SLOT_SIZE);

                // Indicate the sub-events are now programmed with synchronization adjustment
                SETB(p_big->big_info, LLD_BIG_PROG_SYNC, 1);
            }
            else
#endif //(BLE_OBSERVER)
            {
                // Timings are computed from the previous sub-event
                bit_off = p_se->bit_offset + (p_next_se->offset - p_se->offset);
                target_clock = CLK_ADD_2(p_se->timestamp, bit_off / HALF_SLOT_SIZE);
                bit_off      = CO_MOD(bit_off, HALF_SLOT_SIZE);
            }

            // Sub-event timestamp
            p_next_se->timestamp = target_clock;
            p_next_se->bit_offset = bit_off;


            // Program alarm in advance of targeted sub-event
            p_alarm->time.hs = CLK_SUB(p_next_se->timestamp, rwip_prog_delay);
            p_alarm->time.hus = p_next_se->bit_offset;

            // Indicate alarm timer is running
            SETB(p_big->big_info, LLD_BIG_ALARM, 1);

            // Set the alarm
            sch_alarm_set(p_alarm);
        }
        else
        {
            SETB(p_big->big_info, LLD_BIG_ALARM_PROG, 0);
        }

        // If no sub-event programmed and no other sub-event ongoing
        if (p_big->nb_prog == 0)
        {
            // The end of the skipped sub-event can be processed here
            lld_bi_end_subevt(p_bis, p_big, true);
        }
    }
#if (BLE_OBSERVER)
    else if (GETB(p_big->big_info, LLD_BIG_SCHED_REJECT))
    {
        // Counter
        uint8_t cnt;
        // Get Reference sub-event scheduling structure
        struct lld_bi_se *p_ref_se = &(p_big->p_se_infos[p_big->ref_se_idx]);
        // maximum SE index of the packed scheduled group
        uint8_t max_se_idx = p_big->ref_se_idx + p_ref_se->nb_packed;

        // Clear flag indicating that the scheduling is rejected
        SETB(p_big->big_info, LLD_BIG_SCHED_REJECT, false);

        // Consider all subevents in the scheduling structure as done
        for (cnt = p_big->in_proc_se_idx; cnt < max_se_idx ; cnt++)
        {
            // Get activity ID for this subevent
            uint8_t act_id = p_big->p_se_infos[cnt].bis_act_id;

            // Handle end of sub event
            lld_bi_end_subevt(LLD_BIS_GET(act_id), p_big, LLD_BI_SE_STATUS_REJ_BIT);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }
#endif //(BLE_OBSERVER)
}

/**
 ****************************************************************************************
 * @brief Schedule next anchor points for next sub event to be scheduled.
 *
 * @param[in] p_big         BIG structure
 * @param[in] clock         Current clock
 ****************************************************************************************
 */
__STATIC void lld_bi_sched(struct lld_big_env *p_big, uint32_t clock)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_SCHED_2_FUNC_BIT, p_big, clock);

    DBG_SWDIAG(BI, SCHED, 1);
    // Indicate if event has been inserted or not
    bool timeout = false;

    do
    {
        // Get event parameters
        struct sch_arb_elt_tag *p_evt      = &p_big->elt;
        // Get packed event structure structure
        struct lld_bi_se       *p_ref_se   = &(p_big->p_se_infos[p_big->ref_se_idx]);
        // Get pointer to scheduling information for the first subevent to be scheduled
        struct lld_bi_se       *p_first_se = &(p_big->p_se_infos[p_big->in_proc_se_idx]);

        // Expected next anchor point (half slot part)
        uint32_t target_clock;
        // Expected next anchor point (half microsecond part)
        int32_t bit_off;

        // Compute anchor point for first subevent based on BIG anchor point
        bit_off = p_big->anchor_bit_off + p_first_se->offset;
        target_clock = CLK_ADD_2(p_big->anchor_ts, bit_off / HALF_SLOT_SIZE);
        bit_off = CO_MOD(bit_off, HALF_SLOT_SIZE);

#if (BLE_OBSERVER)
        // Check if we are broadcast receiver, if yes we have to take care of
        // drift with broadcaster device and rx sync window.
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            // first element to get sub-event interval
            struct lld_bis_env *p_bis = (struct lld_bis_env *)co_list_pick(&p_big->list_bis);

            // Window widening limit, after which a disconnection occurs
            uint32_t win_lim = (p_bis->hop_inf.nse < 3) ? (uint32_t)(p_big->iso_interval * HALF_SLOT_SIZE / 2 - US_TO_HUS(BLE_IFS_DUR)) : p_bis->sub_interval;

            // Keep expected synchronization time
            p_first_se->exp_sync_ts = target_clock;
            p_first_se->exp_sync_bit_off = bit_off;

            // Compute RX timings
            p_big->rx_win_size = lld_rx_timing_compute(p_big->last_sync_ts, &target_clock, &bit_off, p_big->sca, p_big->rate, 0);

            // Disconnect if the window widening value has reached its limit
            if ((p_big->rx_win_size / 2) >= win_lim)
            {
                timeout = true;
                // Here we have a synchronization timeout
                break;
            }
        }
#endif //(BLE_OBSERVER)

        p_first_se->timestamp  = target_clock;
        p_first_se->bit_offset = bit_off;

        // Check that first subevent is not in the past
        if (CLK_GREATER_THAN(p_first_se->timestamp, clock))
        {
            // Prepare the EA event
            p_evt->time.hs      = p_first_se->timestamp;
            p_evt->time.hus     = p_first_se->bit_offset;
            p_evt->duration_min = p_ref_se->duration_packed - (p_first_se->offset - p_ref_se->offset);

#if (BLE_OBSERVER)
            p_evt->duration_min += p_big->rx_win_size;
#endif //(BLE_OBSERVER)

            // And try to insert it
            if (sch_arb_insert(p_evt) == SCH_ARB_ERROR_OK)
                break;
        }

        // Event cannot be scheduled, increment priority
        p_evt->current_prio = RWIP_PRIO_ADD_2(p_evt->current_prio, RWIP_PRIO_INC(p_big->prio_idx));

        // Mark that scheduling has been rejected
        SETB(p_big->big_info, LLD_BIG_SCHED_REJECT, 1);

        {
            // Get pointer to alarm
            struct sch_alarm_tag *p_alarm = &p_big->alarm;

            ASSERT_ERR(GETB(p_big->big_info, LLD_BIG_ALARM) == 0);

            // Program alarm to handle scheduling rejected
            p_alarm->time.hs = p_first_se->timestamp;
            p_alarm->time.hus = 0;

            // Indicate alarm timer is running
            SETB(p_big->big_info, LLD_BIG_ALARM, 1);

            // Set the alarm
            sch_alarm_set(p_alarm);
        }
    }
    while (0);

    if (timeout)
    {
        p_big->stop_reason = CO_ERROR_CON_TIMEOUT;

        // Free the BIG structure
        lld_big_free(p_big, true, false);
    }

    DBG_SWDIAG(BI, SCHED, 0);
}

/**
 ****************************************************************************************
 * @brief Link RX or TX ISO descriptors together for a given BIS. Mark them as done.
 *
 * @param[in] p_bis    Pointer to the BIS structure
 * @param[in] p_big    Pointer to the BIG structure
 ****************************************************************************************
 */
__STATIC void lld_bis_configure_descs(struct lld_bis_env *p_bis, struct lld_big_env *p_big)
{
    // Counter
    uint8_t desc_cnt;
    // Number of descriptors
    uint8_t nb_desc = (!p_bis->ctrl_se) ? BLE_NB_ISODESC_PER_BIS : 1;

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_CONFIGURE_DESCS_BIT, p_bis, p_big);

    // Link ISO descriptors together and mark them as not ready
    for (desc_cnt = 0; desc_cnt < nb_desc; desc_cnt++)
    {
        uint8_t isodesc_idx      = p_bis->iso_descs_idx[desc_cnt];
        uint8_t next_isodesc_idx = (desc_cnt < (nb_desc - 1))
                                   ? p_bis->iso_descs_idx[desc_cnt + 1]
                                   : p_bis->iso_descs_idx[0];

#if (BLE_OBSERVER && BLE_BROADCASTER)
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
#endif //(BLE_OBSERVER && BLE_BROADCASTER)
        {
#if (BLE_OBSERVER)
            // Set next pointer - Mark as done
            em_ble_rxisoptr_pack(isodesc_idx, 1, REG_EM_ADDR_GET(BLE_RX_ISO_DESC, next_isodesc_idx));
#endif //(BLE_OBSERVER)
        }
#if (BLE_OBSERVER && BLE_BROADCASTER)
        else
#endif //(BLE_OBSERVER && BLE_BROADCASTER)
        {
#if (BLE_BROADCASTER)
            // Set next pointer - Mark as done
            em_ble_txisoptr_pack(isodesc_idx, 1, 0, REG_EM_ADDR_GET(BLE_TX_ISO_DESC, next_isodesc_idx));
#endif //(BLE_BROADCASTER)
        }
    }
}

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Compute channel map for the next channel scanner event. HW will open a scan window
 * for all channels marked as one. Once channel map is computed, it is set in the CS.
 *
 * @param[in] p_bis     Pointer to the BIS structure, shall be index 0.
 * @param[in] p_big     Pointer to the BIG structure
 ****************************************************************************************
 */
__STATIC void lld_bi_scan_set_ch_map(struct lld_bis_env *p_bis, struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_SCAN_SET_CHMAP_2_FUNC_BIT, p_bis, p_big);

    // CS Index
    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id);
    // Channel map
    struct le_ch_map ch_map =  {{0}};
    // Remaining number of channels for map construction
    uint8_t nb_chan = LLD_BI_SCAN_NB_CHAN;
    // Channel index
    uint8_t chidx = p_big->scan_chidx;

    // Compute channel map
    while (nb_chan)
    {
        // Byte in channel map
        uint8_t byte_pos;
        // Position in channel map byte
        uint8_t pos;

        // Increase channel index
        chidx++;

        if (chidx == 37)
        {
            chidx = 0;
        }

        byte_pos = chidx >> 3;
        pos = CO_MOD(chidx, 8);

        if (p_big->scan_ch_map.map[byte_pos] & CO_BIT(pos))
        {
            ch_map.map[byte_pos] |= CO_BIT(pos);
            nb_chan--;
        }
    }

    // Keep channel index for next channel scan event
    p_big->scan_chidx = chidx;

    // Set channel map
    em_ble_chmap0_llchmap0_setf(cs_idx, co_read16p(&ch_map.map[0]));
    em_ble_chmap1_llchmap1_setf(cs_idx, co_read16p(&ch_map.map[2]));
    em_ble_chmap2_llchmap2_setf(cs_idx, ch_map.map[4]);
}

/**
 ****************************************************************************************
 * @brief Fill the control structure used by a given BIG (index = 0) for starting
 * a channel scanner subevent.
 *
 * @param[in] p_bis     Pointer to the BIS structure, shall be index 0.
 * @param[in] p_big     Pointer to the BIG structure
 ****************************************************************************************
 */
__STATIC void lld_bi_scan_fill_cs(struct lld_bis_env *p_bis, struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_SCAN_FILL_CS_2_FUNC_BIT, p_bis, p_big);

    // CS Index
    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id);

    // CS Format
    em_ble_cntl_format_setf(cs_idx, EM_BLE_CS_FMT_CHAN_SCAN);

    // Threshold and Rate Control
    em_ble_thrcntl_ratecntl_rxthr_setf(cs_idx, 1);
    em_ble_thrcntl_ratecntl_rxrate_setf(cs_idx, CO_RATE_1MBPS);

    // Set event duration
    em_ble_maxevtime_set(cs_idx, p_bis->duration);
    em_ble_minevtime_set(cs_idx, p_bis->duration);

    // Hopping control
    em_ble_hopcntl_pack(cs_idx, /*fhen*/     true,
                        /*hop_mode*/ LLD_HOP_MODE_CHAN_SEL_1,
                        /*hopint*/   1,
                        /*chidx*/    p_big->scan_chidx);

    // Set RX window size (half size)
    em_ble_rxwincntl_pack(cs_idx, 0, LLD_BI_SCAN_WIN_SIZE >> 1);

    // Set sync word
    em_ble_syncwl_set(cs_idx, 0x5555);
    em_ble_syncwh_set(cs_idx, 0x5555);
}

/**
 ****************************************************************************************
 * @brief Fill the control structure used by a given channel (channel index = 0) for starting
 * an control subevent.
 *
 * @param[in] p_bis     Pointer to the BIS structure, shall be channel with index 0.
 * @param[in] p_big     Pointer to the BIG structure
 ****************************************************************************************
 */
__STATIC void lld_bi_cntl_fill_cs(struct lld_bis_env *p_bis, struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_CNTL_FILL_CS_2_FUNC_BIT, p_bis, p_big);

    // CS Index
    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id);

    // CS Format
    em_ble_cntl_format_setf(cs_idx, EM_BLE_CS_FMT_MST_CONNECT);

    // Threshold and Rate Control
    em_ble_thrcntl_ratecntl_rxthr_setf(cs_idx, 10);

    // Set event duration
    em_ble_maxevtime_set(cs_idx, p_bis->duration);
    em_ble_minevtime_set(cs_idx, p_bis->duration);

    // Hopping control
    em_ble_hopcntl_pack(cs_idx, /*fhen*/     true,
                        /*hop_mode*/ LLD_HOP_MODE_CHAN_SEL_2,
                        /*hopint*/   1,
                        /*chidx*/    0);

    // Set sync word
    em_ble_syncwl_set(cs_idx, p_bis->hop_inf.acc_code >> 0);
    em_ble_syncwh_set(cs_idx, p_bis->hop_inf.acc_code >> 16);

    // Set channel map
    em_ble_chmap0_set(cs_idx, co_read16p(&p_big->ch_map.map[0]));
    em_ble_chmap1_set(cs_idx, co_read16p(&p_big->ch_map.map[2]));
    em_ble_chmap2_set(cs_idx, co_read16p(&p_big->ch_map.map[4]));
}

/**
 ****************************************************************************************
 * @brief Update BIG control subevent
 *
 * @param[in] p_big Pointer to the BIG structure
 ****************************************************************************************
 */
__STATIC void lld_big_cntl_update(struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_CNTL_UPDATE_BIT, p_big);

    // Initialize number of TX attempts
    p_big->nb_cntl_tx = p_big->next_nb_cntl_tx;

    SETB(p_big->big_info, LLD_BIG_CSE_CS_UPDATE, 1);

    // Increase CSSN value
    CO_VAL_INC(p_big->cssn, LLD_BI_CSSN_MAX);

    // Set address of buffer in the exchange memory
    p_big->ctrl_se_em_ptr = p_big->next_ctrl_se_em_ptr;

    p_big->next_nb_cntl_tx = 0;
    p_big->next_ctrl_se_em_ptr = 0;
}
#endif //(BLE_BROADCASTER)

/**
 ****************************************************************************************
 * @brief Fill the control structure used by a given BIS for BIS data transmission or
 * reception. Done once when BIS is started.
 *
 * @param[in] p_bis         Pointer to the BIS structure.
 * @param[in] p_big         Pointer to the BIG structure.
 * @param[in] p_bis_params  Pointer to the BIS parameters.
 * @param[in] crc_init      CRC Init value
 * @param[in] giv           Group Initialization vector
 ****************************************************************************************
 */
__STATIC void lld_bi_fill_cs(struct lld_bis_env *p_bis, struct lld_big_env *p_big,
                             struct lld_big_params *p_bis_params, uint32_t crc_init, uint8_t *giv)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_FILL_CS_2_FUNC_BIT, 5, p_bis, p_big, p_bis_params, crc_init, giv);
    // CS Index
    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id);
    // Indicate if BIG will use encryption
    bool encrypt = GETB(p_big->big_info, LLD_BIG_ENCRYPT);

    // Set permission/status of CS as R/W but uninitialized
    DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

    // BLE control
    em_ble_cntl_pack(cs_idx, RWIP_COEX_GET(CON, TXBSY),
                     RWIP_COEX_GET(CON, RXBSY),
                     RWIP_COEX_GET(CON, DNABORT),
                     ((p_bis_params->role == ROLE_BROADCASTER)
                      ? EM_BLE_CS_FMT_MST_CONNECT : EM_BLE_CS_FMT_SLV_CONNECT));

    // ISO link control
    em_ble_isolinkcntl_pack(cs_idx, /*streamlbl*/   p_bis->act_id,
                            /*grouplbl*/    p_big->grp_hdl,
                            /*isosyncmode*/ 0,
                            /*isosyncen*/   1,
                            /*isotype*/     LLD_ISO_MODE_BI);

    // Thresholds and Rates
    em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/   10,
                                 /*txthr*/   10,
                                 /*auxrate*/ p_big->rate,
                                 /*rxrate*/  p_big->rate,
                                 /*txrate*/  p_big->rate);

    // Link control
    em_ble_linkcntl_pack(cs_idx, /*hplpmode*/      0,
                         /*linklbl*/       cs_idx,
                         /*sas*/           false,
                         /*nullrxllidflt*/ true,
                         /*micmode*/       ENC_MIC_PRESENT,
                         /*cryptmode*/     ENC_MODE_PKT_PLD_CNT,
                         /*txcrypten*/     encrypt,
                         /*rxcrypten*/     encrypt,
                         /*privnpub*/      false);

    // Synchronization Word
    em_ble_syncwl_set(cs_idx, p_bis->hop_inf.acc_code >> 0);
    em_ble_syncwh_set(cs_idx, p_bis->hop_inf.acc_code >> 16);

    // CRC Initialization value for BIS Data PDUs
    /*
     * For every BIS Data PDU the CRC initialization value shall be calculated as follows:
     * CRC initialization value = ((BaseCRCInit<<8) && BIS(n),
     *      where BaseCRCInit is the BaseCRCInit field in the BIGInfo and the BIS(n)
     *      is the BIS number.
     * For every BIS Control PDU the CRC initialization value = (BaseCRCInit<<8) && 0x00.
     */
    em_ble_crcinit0_set(cs_idx, (crc_init & 0xFFFF));
    em_ble_crcinit1_pack(cs_idx, /*rxmaxctebuf*/ 0, /*crcinit1*/ (crc_init >> 16));

    // Hopping control
    em_ble_hopcntl_pack(cs_idx, /*fhen*/     true,
                        /*hop_mode*/ (!p_bis->ctrl_se) ? LLD_HOP_MODE_SEQ_PTR : LLD_HOP_MODE_CHAN_SEL_2,
                        /*hopint*/   1, // Not used in channel selection 2
                        /*chidx*/    0);

    // TX power control
    em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_max);

    // Set encryption parameters
    if (encrypt)
    {
        uint8_t reg_idx;
        uint8_t key_idx;

        // Configure Initialization vector in CS
        // BIS_IV[0:63]  =  (GIV[0:31] XOR BIS_AA[0:31]) || GIV[32:63]
        for (reg_idx = 0, key_idx = 0; reg_idx < EM_BLE_IV_COUNT; reg_idx++)
        {
            uint16_t iv_part = co_read16p(&(giv[key_idx]));

            // BIS_IV[0:31]  =  GIV[0:31] XOR BIS_AA[0:31]
            if (reg_idx < 2)
            {
                iv_part ^= (uint16_t)(p_bis->hop_inf.acc_code >> (16 * reg_idx));
            }

            em_ble_iv_setf(cs_idx, reg_idx, iv_part);
            key_idx += 2;
        }


        // Configure Session key in CS
        for (reg_idx = 0, key_idx = (KEY_LEN - 1); reg_idx < EM_BLE_SK_COUNT; reg_idx++)
        {
            // encrypted_data is LSB first and  SK is MSB first
            em_ble_sk_setf(cs_idx, reg_idx,
                           (p_bis_params->gsk.ltk[key_idx - 1] << 8) | p_bis_params->gsk.ltk[key_idx]);
            key_idx -= 2;
        }
    }

    // Set max & min event time
    em_ble_maxevtime_set(cs_idx, p_bis->duration);
    em_ble_minevtime_set(cs_idx, p_bis->duration);

#if (BLE_OBSERVER && BLE_BROADCASTER)
    if (p_bis_params->role == ROLE_BROADCAST_RECEIVER)
#endif //(BLE_OBSERVER && BLE_BROADCASTER)
    {
#if (BLE_OBSERVER)
        // Set Rx Max buf and Rx Max Time + ISO receive payload length authorized
        uint8_t pkt_len = ((p_bis->ctrl_se) ? LE_MIN_OCTETS : p_big->max_pdu) + (encrypt ? MIC_LEN : 0);

        em_ble_rxmaxbuf_pack(cs_idx, pkt_len, pkt_len);
        em_ble_rxmaxtime_set(cs_idx, 0);

        // Set the normal win size
        em_ble_rxwincntl_pack(cs_idx, 0, BLE_RATE_NORMAL_WIN_SIZE(p_big->rate) >> 1);
#endif //(BLE_OBSERVER)

        // Disable unused control
        em_ble_acltxdescptr_set(cs_idx, 0);
    }
#if (BLE_OBSERVER && BLE_BROADCASTER)
    else
#endif //(BLE_OBSERVER && BLE_BROADCASTER)
    {
#if (BLE_BROADCASTER)
        // Initialize the TX descriptor pointer
        em_ble_acltxdescptr_set(cs_idx, 0);
#endif //(BLE_BROADCASTER)

        // Disable unused control
        em_ble_rxmaxbuf_set(cs_idx, 0);
        em_ble_rxwincntl_set(cs_idx, 0);
    }

    // Set channel map
    em_ble_chmap0_set(cs_idx, co_read16p(&p_big->ch_map.map[0]));
    em_ble_chmap1_set(cs_idx, co_read16p(&p_big->ch_map.map[2]));
    em_ble_chmap2_set(cs_idx, co_read16p(&p_big->ch_map.map[4]));

    // Initialize flush event counter
    em_ble_isoevtcntl_pack(cs_idx, /*flushcnt*/ 0, /*subevtcnt*/ 0);

    // ISO TX/RX control
    em_ble_isotxrxcntl_set(cs_idx, 0);

#if (BLE_OBSERVER && BLE_BROADCASTER)
    if (p_bis_params->role == ROLE_BROADCAST_RECEIVER)
#endif //(BLE_OBSERVER && BLE_BROADCASTER)
    {
#if (BLE_OBSERVER)
        // Set address of first ISO descriptor in the control structure
        em_ble_isorxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_RX_ISO_DESC, p_bis->iso_descs_idx[0]));
#endif //(BLE_OBSERVER)
    }
#if (BLE_OBSERVER && BLE_BROADCASTER)
    else
#endif //(BLE_OBSERVER && BLE_BROADCASTER)
    {
#if (BLE_BROADCASTER)
        // Set address of first ISO descriptor in the control structure
        em_ble_isotxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_ISO_DESC, p_bis->iso_descs_idx[0]));
#endif //(BLE_BROADCASTER)
    }

    // Initialize packet counters
    for (uint8_t i = 0; i < 2; i++)
    {
        em_ble_counter0_set(cs_idx, i, 0);
        em_ble_counter1_set(cs_idx, i, 0);
        em_ble_counter2_set(cs_idx, i, 0);
        em_ble_counter3_set(cs_idx, i, 0);
        em_ble_counter4_set(cs_idx, i, 0);
        em_ble_counter5_set(cs_idx, i, 0);
        em_ble_counter6_set(cs_idx, i, 0);
    }

    // Disable unused control
    em_ble_evtcnt_setf(cs_idx, 0);
    em_ble_hopcs2cntl0_set(cs_idx, 0);
    em_ble_hopcs2cntl1_set(cs_idx, 0);
    em_ble_txheadercntl_set(cs_idx, 0);
}

/**
 ****************************************************************************************
 * @brief Fill the exchange table
 *
 * @param[in] p_big     Pointer to the BIG information
 * @param[in] current_prio  Current priority
 ****************************************************************************************
 */
__STATIC void lld_bi_prog(struct lld_big_env *p_big, uint8_t current_prio)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_PROG_2_FUNC_BIT, p_big, current_prio);
    // Retrieve sub-event information
    struct lld_bi_se   *p_se = &(p_big->p_se_infos[p_big->prog_se_idx]);
    // Get BIS structure
    struct lld_bis_env *p_bis = LLD_BIS_GET(p_se->bis_act_id);

    // Get index of used control structure
    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id);
    // Programming parameters pushed to SCH PROG
    struct sch_prog_params prog_par;
    // Information about data packet transmitted or received during the ended sub event
    struct lld_bi_data *p_data;
    // Offset
    uint8_t data_offset;
    // Sub-event type
    uint8_t type;

    DBG_SWDIAG(BI, PROG, 1);

    do
    {
        // Get current time
        rwip_time_t current_time = rwip_time_get();

        // Indicate data has not been programmed
        SETB(p_se->state, LLD_BI_SE_STATUS_DATA_PROG, false);

        if (!p_bis->ctrl_se)
        {
            // Get data offset
            data_offset = p_big->data_map[p_se->bis_sub_evt_idx].offset;

            // Get information about data packet transmitted or received during this sub event
            p_data = &p_bis->data[CO_MOD((p_big->data_pos + data_offset), p_big->nb_data_pkt)];

            // Subevent for BIS data
            type = LLD_BI_DUMMY_TYPE_DATA;
        }
        else
        {
            // Get information about data packet transmitted or received during the ended sub event
            p_data = &p_bis->data[0];

            // Offset is BN
            data_offset = p_big->bn;

            // Get type of control subevent
            type = ((p_big->nb_cntl_tx == 0) && !GETB(p_big->big_info, LLD_BIG_RECEIVER))
                   ? LLD_BI_DUMMY_TYPE_CHAN_SCAN  // Channel scanner event
                   : LLD_BI_DUMMY_TYPE_CNTL; // Transmission or reception of BIS control PDU
        }

        /*
         * Check if sub-event need and can be programmed
         *
         * The sub-event is not programmed in following conditions:
         *    - Programming is disabled (from arbiter)
         *    - The alarm is processed too late for programming the HW (at least 1 half-slot in advance)
         *    - For receiver only:
         *        - Its data has already been received
         *        - The reception window is larger than 2xsub-interval - margin, and a sub-event is already ongoing, to avoid potential race conditions
         */
        if ((!GETB(p_big->big_info, LLD_BIG_PROG_EN)
                && CLK_GREATER_THAN_HUS(current_time.hs, current_time.hus, CLK_SUB(p_se->timestamp, 1), p_se->bit_offset))
#if (BLE_OBSERVER)
                || (GETB(p_big->big_info, LLD_BIG_RECEIVER)
                    && (GETB(p_data->data_info, LLD_BIS_DATA_DONE)
                        || ((p_big->rx_win_size > (2 * p_bis->sub_interval - LLD_BI_SYNC_WIN_MIN_DISTANCE) && (p_big->nb_prog > 0)))))
#endif //(BLE_OBSERVER)
           )
        {
            SETB(p_se->state, LLD_BI_SE_STATUS_SKIP, true);
            // Increment flush counter
            p_bis->flush_cnt++;
            break;
        }

        prog_par.frm_cbk        = &lld_bi_frm_cbk;
        prog_par.time.hs        = p_se->timestamp;
        prog_par.time.hus       = p_se->bit_offset;
        prog_par.cs_idx         = cs_idx;
        prog_par.bandwidth      = p_bis->duration;
        prog_par.prio_1         = current_prio;
        prog_par.prio_2         = current_prio;
        prog_par.prio_3         = 0;
        prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
        prog_par.add.ble.ae_nps = 0;
        prog_par.add.ble.iso    = (type != LLD_BI_DUMMY_TYPE_CHAN_SCAN);
        prog_par.add.ble.rsvd   = (p_se->bis_sub_evt_idx == 0);
        prog_par.add.ble.sic    = GETB(p_big->big_info, LLD_BIG_RECEIVER) && !GETB(p_big->big_info, LLD_BIG_PROG_SYNC) && GETB(p_big->big_info, LLD_BIG_PROG);
        prog_par.mode           = SCH_PROG_BLE;

        // Update priority for Scanning sub-event
        if ((p_bis->ctrl_se) && !GETB(p_big->big_info, LLD_BIG_RECEIVER)
                && (p_big->nb_cntl_tx == 0))
        {
            prog_par.prio_1         = rwip_priority[RWIP_PRIO_BIS_SCAN_IDX].value;
            prog_par.prio_2         = 0;
        }

        // Fill dummy value
        prog_par.dummy = 0;
        SETF(prog_par.dummy, LLD_BI_DUMMY_ACT_ID,      p_bis->act_id);
        SETF(prog_par.dummy, LLD_BI_DUMMY_GRP_HDL,     p_big->grp_hdl);
        SETF(prog_par.dummy, LLD_BI_DUMMY_SUB_EVT_IDX, p_big->prog_se_idx);
        SETF(prog_par.dummy, LLD_BI_DUMMY_TYPE,        type);

        sch_prog_push(&prog_par);

        // Mark as programmed
        SETB(p_se->state, LLD_BI_SE_STATUS_PROG, true);
        // Increase number of programmed sub-event
        p_big->nb_prog++;

        if (p_bis->ctrl_se)
        {
#if (BLE_BROADCASTER)
            // Check if Control sub-event Control structure must be updated
            if (GETB(p_big->big_info, LLD_BIG_CSE_CS_UPDATE))
            {
                if ((p_big->nb_cntl_tx > 0))
                {
                    // Prepare CS for BIS control PDU transmission
                    lld_bi_cntl_fill_cs(p_bis, p_big);
                }
                else
                {
                    // Prepare CS for channel scan event
                    lld_bi_scan_fill_cs(p_bis, p_big);
                }

                SETB(p_big->big_info, LLD_BIG_CSE_CS_UPDATE, 0);
            }
#endif // (BLE_BROADCASTER)
        }

#if (BLE_BROADCASTER)
        if (type == LLD_BI_DUMMY_TYPE_CHAN_SCAN)
        {
            // Prepare channel map
            lld_bi_scan_set_ch_map(p_bis, p_big);
            break;
        }
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
        // Set sync window size in the control structure if we are broadcast receiver
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            // Get synchronization window size in microseconds
            uint32_t sync_win_size_us = GETB(p_big->big_info, LLD_BIG_PROG_SYNC) ? BLE_RATE_NORMAL_WIN_SIZE(p_big->rate) : HUS_TO_US(p_big->rx_win_size + 1);

            if (sync_win_size_us > EM_BLE_CS_RXWINSZ_MAX)
            {
                // Size of the Rx HALF window in half-slots (wide-open mode)
                em_ble_rxwincntl_pack(cs_idx, 1, (sync_win_size_us + (SLOT_SIZE - 1)) / SLOT_SIZE);
            }
            else
            {
                // Size of the Rx HALF window in us (normal mode)
                em_ble_rxwincntl_pack(cs_idx, 0, sync_win_size_us >> 1);
            }
        }
#endif //(BLE_OBSERVER)

        // If first sub-event programmed for the BIS event, prepare the control structure
        if (!GETB(p_bis->bis_info, LLD_BIS_INFO_PROG))
        {
            // Update the connection event counter and set it in the control structure
            em_ble_evtcnt_setf(cs_idx, p_big->big_evt_cnt);

            // Set sub event counter
            em_ble_isoevtcntl_subevtcnt_setf(cs_idx, p_se->bis_sub_evt_idx); // TODO Error here

            // If not control subevent
            if (!p_bis->ctrl_se)
            {
                // Point to the hopping sequence to use
                em_ble_hopptr_hop_seq_ptr_setf(EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id), p_bis->hop_ptr[GETB(p_big->big_info, LLD_BIG_HOP_TOG)] >> 2);
            }

            // Keep in mind that the CS has been filled for the BIS event
            SETB(p_bis->bis_info, LLD_BIS_INFO_PROG, 1);
        }
        else if (p_big->nb_prog == 1)
        {
            // Set sub-event counter
            em_ble_isoevtcntl_subevtcnt_setf(cs_idx, p_se->bis_sub_evt_idx);
        }

        // Program data transmission/reception for this sub-event
        lld_bis_data_prog(p_bis, p_big, p_data, p_se, data_offset);

        // Increment flush counter
        p_bis->flush_cnt++;
    }
    while (0);

    // Move on next sub event to be programmed in the scheduling event
    p_big->prog_se_idx++;

    DBG_SWDIAG(BI, PROG, 0);
}

/**
 ****************************************************************************************
 * @brief Handle end of BIS event when BIG event is over.
 *
 * @param[in] p_bis    Pointer to the BIS structure
 *
 * @return true if BIS has to be stopped, false otherwise
 ****************************************************************************************
 */
__STATIC bool lld_bis_end_evt(struct lld_bis_env *p_bis, struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_END_EVT_BIT, bool, p_bis, p_big);

    bool stop = false;
    // Counter
    uint8_t cnt;

    // Reset BIS information
    SETB(p_bis->bis_info, LLD_BIS_INFO_PROG, 0);

    if (!p_bis->ctrl_se)
    {
        // Clean BN first data structures
        for (cnt = 0; cnt < p_big->bn; cnt++)
        {
            // Get data structure
            struct lld_bi_data *p_data = &p_bis->data[CO_MOD((p_big->data_pos + cnt), p_big->nb_data_pkt)];
            p_data->data_info = 0;
        }

#if (BLE_OBSERVER)
        // Broadcast receiver, check if supervision timeout reached
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER) && GETB(p_big->big_info, LLD_BIG_SYNC_ESTAB))
        {
            uint32_t clock = lld_read_clock();

            // Check if sync timeout is reached
            if (CLK_SUB(clock, p_bis->last_crc_ok_ts) >= p_big->timeout)
            {
                p_big->stop_reason = CO_ERROR_CON_TIMEOUT;
                stop = true;
            }
        }
#endif // (BLE_OBSERVER)
    }
#if (BLE_BROADCASTER)
    // Broadcaster and Control sub-event in use
    else if (!GETB(p_big->big_info, LLD_BIG_RECEIVER) && (p_big->nb_cntl_tx > 0) && (!GETB(p_big->big_info, LLD_BIG_CSE_CS_UPDATE)))
    {
        // Decrement number of attempts
        p_big->nb_cntl_tx--;

        if (p_big->nb_cntl_tx == 0)
        {
            // Inform LLI task that PDU has been transmitted
            struct lld_big_tx_ind *p_msg = KE_MSG_ALLOC(LLD_BIG_TX_IND, TASK_LLI, TASK_NONE, lld_big_tx_ind);
            p_msg->grp_hdl = p_big->grp_hdl;
            // Send the allocated message
            ke_msg_send(p_msg);

            // Keep in mind that channel scan has to be restarted
            SETB(p_big->big_info, LLD_BIG_CSE_CS_UPDATE, 1);

            // Clean-up control buffer
            ble_util_buf_llcp_tx_free(p_big->ctrl_se_em_ptr);
        }
    }
#endif // (BLE_BROADCASTER)

    return stop;
}

#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Prepare next air time to reserve in Scheduling arbiter.
 *
 * @param[in] p_big Pointer to the BIG structure
 ****************************************************************************************
 */
__STATIC void lld_bi_prep_rsv_sched(struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_PREP_RSV_SCHED_2_FUNC_BIT, p_big);

    while (p_big->ref_se_idx < p_big->big_nse)
    {
        // Counter
        uint8_t cnt;
        // Sub-Event reference
        struct lld_bi_se *p_ref_se = &(p_big->p_se_infos[p_big->ref_se_idx]);

        // Check if at least one subevent has to be programmed
        for (cnt = p_big->ref_se_idx; cnt < (p_big->ref_se_idx + p_ref_se->nb_packed); cnt++)
        {
            // And structure for the sub event
            struct lld_bi_se   *p_se = &p_big->p_se_infos[cnt];
            // Get BIS structure
            struct lld_bis_env *p_bis = LLD_BIS_GET(p_se->bis_act_id);
            // Information about data packet transmitted or received during the ended sub event
            struct lld_bi_data *p_data;
            uint8_t data_offset = 0;
            bool data_release = false;

            if (!p_bis->ctrl_se)
            {
                // Get data offset structure
                struct lld_bi_data_map *p_data_map = &p_big->data_map[p_se->bis_sub_evt_idx];

                data_offset = p_data_map->offset;
                p_data = &p_bis->data[CO_MOD((p_big->data_pos + data_offset), p_big->nb_data_pkt)];

                data_release = p_data_map->last;
            }
            else
            {
                p_data = &p_bis->data[0];
            }

            // Check if data has already been received
            if (!GETB(p_data->data_info, LLD_BIS_DATA_DONE))
            {
                break;
            }

            // release data if needed
            if (data_release)
            {
                lld_bis_data_release(p_big, p_bis, p_data, data_offset);
            }
        }

        // Check if at least one sub event has to be programmed
        if (cnt < (p_big->ref_se_idx + p_ref_se->nb_packed))
        {
            // Skip subevents
            p_big->in_proc_se_idx = cnt;
            p_big->prog_se_idx    = cnt;
            break;
        }

        // Update Reference structure
        p_big->ref_se_idx = p_big->ref_se_idx + p_ref_se->nb_packed;
    }
}
#endif // (BLE_OBSERVER)

/**
 ****************************************************************************************
 * @brief Handle end of BIG event.
 *
 * @param[in] p_big Pointer to the BIG structure
 *
 * @return true if BIG event can be rescheduled, else false
 ****************************************************************************************
 */
__STATIC bool lld_big_end_evt(struct lld_big_env *p_big)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_END_EVT_BIT, bool, p_big);

    // Indicate if BIG has been stopped
    bool stop = GETB(p_big->big_info, LLD_BIG_STOP);
    DBG_SWDIAG(BI, END_STREAM, 1);

    do
    {
        // Get first BIS information
        struct lld_bis_env *p_bis = (struct lld_bis_env *)co_list_pick(&p_big->list_bis);
        // Indicate if BIG event has been programmed or fully rejected
        bool was_prog = GETB(p_big->big_info, LLD_BIG_EVT_ONGOING);

#if (BLE_OBSERVER)
        // If the BIG sync has not been established within 6 intervals
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER) && (!GETB(p_big->big_info, LLD_BIG_SYNC_ESTAB) && (BLE_UTIL_EVT_CNT_DIFF(p_big->big_evt_cnt, p_big->first_big_evt_cnt) >= 5)))
        {
            p_big->stop_reason = CO_ERROR_CONN_FAILED_TO_BE_EST;
            stop = true;
        }
#endif // (BLE_OBSERVER)

        // Check if at least one BIS has to be stopped
        if (stop)
        {
            // Free the BIG structure
            lld_big_free(p_big, true, false);
            break;
        }

        // Update BIG anchor point parameters
        p_big->anchor_ts  = CLK_ADD_2(p_big->anchor_ts, p_big->iso_interval);
        p_big->big_evt_cnt++;
        p_big->anchor_bts = rwip_bt_time_to_bts(p_big->anchor_ts, p_big->anchor_bit_off);

        // Increase payload counter
        LLD_ISO_PYLD_CNT_ADD(p_big->bis_pkt_cnt, p_big->bn);

        // Indicate no BIG event ongoing
        SETB(p_big->big_info, LLD_BIG_EVT_ONGOING, 0);

        while (p_bis && !stop)
        {
            // Perform operation for end of BIS event
            stop = lld_bis_end_evt(p_bis, p_big);

            // Get next BIS
            p_bis = (struct lld_bis_env *)(p_bis->hdr.next);
        }

        // Check if at least one BIS has to be stopped
        if (stop)
        {
            // Free the BIG structure
            lld_big_free(p_big, true, false);
            break;
        }

        // Restart computation of hopping for next event if not programmed
        if (!was_prog)
        {
            lld_bi_compute_hop_scheme(p_big, p_big->big_evt_cnt);
        }

        // Toggle the hopping scheme buffer
        TOGB(p_big->big_info, LLD_BIG_HOP_TOG);

        // Update channel map of control sub-event
        if (GETB(p_big->big_info, LLD_BIG_CH_MAP_UPD))
        {
            uint8_t cs_idx;

            SETB(p_big->big_info, LLD_BIG_CH_MAP_UPD, 0);
            p_bis = (struct lld_bis_env *)(p_big->list_bis.last);
            cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id);

            em_ble_chmap0_llchmap0_setf(cs_idx, co_read16p(&p_big->ch_map.map[0]));
            em_ble_chmap1_llchmap1_setf(cs_idx, co_read16p(&p_big->ch_map.map[2]));
            em_ble_chmap2_llchmap2_setf(cs_idx, p_big->ch_map.map[4]);
        }

        // Update data position
        p_big->data_pos = CO_MOD((p_big->data_pos + p_big->bn), p_big->nb_data_pkt);

        // Come back on first Sub-event structure
        p_big->ref_se_idx     = 0;
        p_big->prog_se_idx    = 0;
        p_big->in_proc_se_idx = 0;

#if (BLE_OBSERVER)
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            // Get next air time that can be reserved
            lld_bi_prep_rsv_sched(p_big);

            // Should never happen
            ASSERT_ERR(p_big->ref_se_idx != p_big->big_nse);
        }
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
        // Broadcaster and no Control sub-event
        if (!GETB(p_big->big_info, LLD_BIG_RECEIVER) && (p_big->nb_cntl_tx == 0) && (p_big->next_nb_cntl_tx > 0))
        {
            // Update the control subevent
            lld_big_cntl_update(p_big);
        }
#endif // (BLE_BROADCASTER)
    }
    while (0);

    DBG_SWDIAG(BI, END_STREAM, 0);

    return (!stop);
}

/**
 ****************************************************************************************
 * @brief Handle end of scheduled activity.
 *
 * @param[in] p_ref_se      Pointer to the Reference pack information
 * @param[in] p_big         Pointer to the BIG structure
 ****************************************************************************************
 */
__STATIC void lld_bi_end_sched_act(struct lld_bi_se *p_ref_se, struct lld_big_env *p_big)
{
    // Get scheduling element event
    struct sch_arb_elt_tag *p_elt = &p_big->elt;

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_END_SCHED_ACT_2_FUNC_BIT, p_ref_se, p_big);

    DBG_SWDIAG(BI, END_SCHED, 1);

    ASSERT_INFO(p_big->nb_prog == 0, p_big->prog_se_idx, p_big->nb_prog);

    // Remove scheduling element
    sch_arb_remove(p_elt, true);

    SETB(p_big->big_info, LLD_BIG_PROG, 0);

#if (BLE_OBSERVER)
    // If broadcast receiver, apply receiver offset on anchor time.
    if (GETB(p_big->big_info, LLD_BIG_RECEIVER) && GETB(p_big->big_info, LLD_BIG_SYNC))
    {
        uint32_t new_offset;
        uint32_t prev_offset = CO_MOD(p_big->anchor_ts, p_big->iso_interval);

        // Stream anchor timestamp - half microseconds part
        int32_t big_anchor_bit_off = p_big->anchor_bit_off + p_big->receiver_offset;

        while (big_anchor_bit_off < 0)
        {
            big_anchor_bit_off += HALF_SLOT_SIZE;
            p_big->anchor_ts = CLK_SUB(p_big->anchor_ts, 1);
        }

        // Update BIG anchor position
        p_big->anchor_ts      = CLK_ADD_2(p_big->anchor_ts, big_anchor_bit_off / HALF_SLOT_SIZE);
        p_big->anchor_bit_off = CO_MOD(big_anchor_bit_off, HALF_SLOT_SIZE);
        p_big->anchor_bts     = rwip_bt_time_to_bts(p_big->anchor_ts, p_big->anchor_bit_off);
        p_big->sync_drift_acc += p_big->receiver_offset;

        // Clear sync bit
        SETB(p_big->big_info, LLD_BIG_SYNC, 0);
        SETB(p_big->big_info, LLD_BIG_PROG_SYNC, 0);

        // Update associated periodic sync
        lld_sync_sync_time_update(p_big->per_sync_id, p_big->last_sync_ts, p_big->sync_drift_acc);

        // Check if data path synchronization is enabled
        if (p_big->cb_peer_sync)
        {
            p_big->cb_peer_sync(p_big->big_evt_cnt, p_big->anchor_bts);
        }

        // If offset has changed
        new_offset = CO_MOD(p_big->anchor_ts, p_big->iso_interval);
        if (new_offset != prev_offset)
        {
            // Indicate new offset to LLC
            struct lld_big_sync_offset_upd_ind *msg = KE_MSG_ALLOC(LLD_BIG_SYNC_OFFSET_UPD_IND, TASK_LLI, TASK_NONE, lld_big_sync_offset_upd_ind);
            msg->grp_hdl = p_big->grp_hdl;
            msg->act_offset = new_offset;
            ke_msg_send(msg);
        }
    }
#endif //(BLE_OBSERVER)

    // Move to the next pack of event
    p_big->ref_se_idx = p_big->ref_se_idx + p_ref_se->nb_packed;

    p_big->in_proc_se_idx = p_big->ref_se_idx;
    p_big->prog_se_idx    = p_big->ref_se_idx;

#if (BLE_OBSERVER)
    if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
    {
        // Get next air time that can be reserved
        lld_bi_prep_rsv_sched(p_big);
    }
#endif //(BLE_OBSERVER)

    do
    {
        // Check if BIG event is finished
        if (p_big->ref_se_idx == p_big->big_nse)
        {
            if (!lld_big_end_evt(p_big))
            {
                // Stream has been deleted
                break;
            }
        }

        // Schedule the next event block
        lld_bi_sched(p_big, lld_read_clock());

    }
    while (0);

    DBG_SWDIAG(BI, END_SCHED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle end of subevent.
 *
 * @param[in] p_bis      Pointer to the BIS structure
 * @param[in] p_big      Pointer to the BIG structure
 * @param[in] se_status  Provide sub event status (@see enum lld_bi_se_status_fields)
 *
 * @return True if all scheduled event are handled, False else
 ****************************************************************************************
 */
__STATIC bool lld_bi_end_subevt(struct lld_bis_env *p_bis, struct lld_big_env *p_big, uint8_t se_status)
{
    // Get Reference Sub-event structure
    struct lld_bi_se       *p_ref_se = &(p_big->p_se_infos[p_big->ref_se_idx]);
    // And In process structure for the sub event
    struct lld_bi_se       *p_cur_se = &(p_big->p_se_infos[p_big->in_proc_se_idx]);
    // Data offset structure
    struct lld_bi_data_map *p_data_map;
    // Information about data packet transmitted or received during the ended sub event
    struct lld_bi_data     *p_data;

    bool ret = false;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_END_SUBEVT_2_FUNC_BIT, bool, p_bis, p_big, se_status);

    // event has been skipped, handled it has end of sub-event
    if (GETB(p_cur_se->state, LLD_BI_SE_STATUS_SKIP))
    {
        // Dynamically update control structure for next programmed sub-event
        lld_bis_skip_subevent(p_cur_se, GETB(p_cur_se->state, LLD_BI_SE_STATUS_PROG));

        SETB(p_cur_se->state, LLD_BI_SE_STATUS_SKIP, false);
    }

    // Update programming information
    if (GETB(p_cur_se->state, LLD_BI_SE_STATUS_PROG))
    {
        p_big->nb_prog--;
        // Mark the sub event has not programmed anymore
        SETB(p_cur_se->state, LLD_BI_SE_STATUS_PROG, false);
    }

    // Check if subevent was intended to transmit or receive BIS data
    if (!p_bis->ctrl_se)
    {
        p_data_map = &p_big->data_map[p_cur_se->bis_sub_evt_idx];
        p_data = &p_bis->data[CO_MOD((p_big->data_pos + p_data_map->offset), p_big->nb_data_pkt)];

        // Check if broadcast reception succeeded
#if (BLE_OBSERVER)
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER) && !GETB(p_data->data_info, LLD_BIS_DATA_DONE) && GETB(se_status, LLD_BI_SE_STATUS_RX_OK))
        {
            // Keep in mind that the data buffer has been received properly
            SETB(p_data->data_info, LLD_BIS_DATA_DONE, 1);
        }
#endif //(BLE_OBSERVER)

        // Buffer is released only if last attempt of transmission/reception has been done.
        if (p_data_map->last)
        {
            lld_bis_data_release(p_big, p_bis, p_data, p_data_map->offset);
        }
    }

    // Move on next sub-event for which next end of sub event is expected
    p_big->in_proc_se_idx++;

    if (p_big->in_proc_se_idx == (p_big->ref_se_idx + p_ref_se->nb_packed))
    {
        // End current scheduling event
        lld_bi_end_sched_act(p_ref_se, p_big);
        ret = true;
    }

    return ret;
}

#if (BLE_OBSERVER)

/**
 ****************************************************************************************
 * @brief Check the reception during BIS activity
 *
 * @brief p_bis         Pointer to the BIS structure
 * @brief p_big         Pointer to the BIG structure
 * @brief timestamp     EOF interruption timestamp
 *
 * @return true if data packet has been properly received, else false
 ****************************************************************************************
 */
__STATIC uint8_t lld_bis_rx(struct lld_bis_env *p_bis, struct lld_big_env *p_big, uint32_t timestamp)
{
    // Reception status
    uint8_t rx_status = RWIP_RX_OTHER_ERROR;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_RX_BIT, uint8_t, p_bis, p_big, timestamp);

    // Check if a packet has been received (RX Done bit set to 1)
    if (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id)))
    {
        // Get index of RX descriptor
        uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;
        uint8_t rx_cssn;
        // Retrieve RX status
        uint16_t rxstat      = em_ble_rxstatce_get(rxdesc_idx);
        uint16_t acl_buf_ptr = em_ble_rxdataptr_getf(rxdesc_idx);

        // Trace the current RX descriptor
        TRC_REQ_RX_DESC(LLD_CON, p_bis->act_id, REG_EM_BLE_RX_DESC_ADDR_GET(rxdesc_idx));

        do
        {
            // Check synchronization status
            if (rxstat & EM_BLE_SYNC_ERR_BIT)
            {
                rx_status = RWIP_RX_SYNC_ERROR;
                break;
            }
            else
            {
                // Get information for this sub-event
                struct lld_bi_se *p_se = &(p_big->p_se_infos[p_big->in_proc_se_idx]);

                // *** Synchronize broadcast receiver after correct reception
                // Note: this algorithm supports large sync windows but not multi-attempts

                // Read clock value where sync has been found
                uint32_t base_cnt = (em_ble_rxclknsync1_clknrxsync1_getf(rxdesc_idx) << 16)
                                    | em_ble_rxclknsync0_clknrxsync0_getf(rxdesc_idx);

                // Read bit position where sync has been found
                uint16_t fine_cnt = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(rxdesc_idx);
                // Compute new bit offset (remove synchronization word duration)
                int32_t new_bit_off = fine_cnt - US_TO_HUS(lld_exp_sync_pos_tab[p_big->rate]);

                // Keep in mind that synchronization has been caught
                SETB(p_big->big_info, LLD_BIG_SYNC, 1);
                // reset default window reception size
                p_big->def_rx_win_size = 0;

                // If BIG sync has not yet been established
                if (!GETB(p_big->big_info, LLD_BIG_SYNC_ESTAB))
                {
                    // Inform link layer controller that BIG sync has been established
                    struct lld_big_sync_estab_ind *p_msg = KE_MSG_ALLOC(LLD_BIG_SYNC_ESTAB_IND, TASK_LLI, TASK_NONE, lld_big_sync_estab_ind);
                    p_msg->grp_hdl = p_big->grp_hdl;
                    p_msg->status = CO_ERROR_NO_ERROR;
                    p_msg->act_offset = CO_MOD(p_big->anchor_ts, p_big->iso_interval);
                    ke_msg_send(p_msg);

                    // Keep in mind that BIG sync has been established
                    SETB(p_big->big_info, LLD_BIG_SYNC_ESTAB, 1);
                }

                // If reception has drifted on the previous slot
                while (new_bit_off < 0)
                {
                    new_bit_off += HALF_SLOT_SIZE;
                    base_cnt = CLK_SUB(base_cnt, 1);
                }

                // Save the values corresponding to the last detected sync
                p_big->last_sync_ts = base_cnt;

                // Compute difference with expected synchronization
                p_big->receiver_offset = (CLK_DIFF(p_se->exp_sync_ts, base_cnt) * HALF_SLOT_SIZE)
                                         + (new_bit_off - p_se->exp_sync_bit_off);
            }

            // Check payload status
            if (rxstat & (EM_BLE_RXTIME_ERR_BIT | EM_BLE_LEN_ERR_BIT | EM_BLE_CRC_ERR_BIT | EM_BLE_LLID_ERR_BIT))
            {
                if (rxstat & EM_BLE_CRC_ERR_BIT)
                    rx_status = RWIP_RX_CRC_ERROR;
                break;
            }

            if (rxstat & EM_BLE_MIC_ERR_BIT)
            {
                // Keep in mind that a MIC error has been detected, BIS will be stopped
                // as soon as possible
                SETB(p_bis->bis_info, LLD_BIS_INFO_MIC_FAIL, 1);

                // Set stop reason
                p_big->stop_reason = CO_ERROR_TERMINATED_MIC_FAILURE;

                // Stop the BIG
                SETB(p_big->big_info, LLD_BIG_STOP, 1);
                break;
            }

            // save last time a valid CRC is received
            p_bis->last_crc_ok_ts = p_big->last_sync_ts;

            // Check that BIS control PDU has not already been received
            rx_cssn = em_ble_rxphbis_rxcssn_getf(rxdesc_idx);

            // BIS
            if (!p_bis->ctrl_se)
            {
                struct lld_bis_env *p_bis_cse = (struct lld_bis_env *)p_big->list_bis.last;
                if (em_ble_rxphbis_rxcstf_getf(rxdesc_idx) && (rx_cssn != p_big->cssn))
                {
                    // Mark data ready to be received
                    SETB(p_bis_cse->data[0].data_info, LLD_BIS_DATA_DONE, 0);
                }
                else
                {
                    // Mark data as received
                    SETB(p_bis_cse->data[0].data_info, LLD_BIS_DATA_DONE, 1);
                }
            }
            // control sub-Event
            else
            {
                if (rx_cssn != p_big->cssn)
                {
                    // Inform the LLI task that an BIS control PDU has been received
                    struct lld_big_rx_ind *p_msg = KE_MSG_ALLOC(LLD_BIG_RX_IND, TASK_LLI, TASK_NONE, lld_big_rx_ind);

                    p_msg->grp_hdl  = p_big->grp_hdl;
                    p_msg->length      = em_ble_rxphbis_rxlen_getf(rxdesc_idx);

                    // Check if MIC length should be removed
                    if (GETB(p_big->big_info, LLD_BIG_ENCRYPT) && (p_msg->length > 0))
                    {
                        p_msg->length -= MIC_LEN;
                    }

                    p_msg->em_buf      = acl_buf_ptr;
                    p_msg->evt_cnt     = p_big->big_evt_cnt;

                    // Send the allocated message
                    ke_msg_send(p_msg);

                    // Mark data as received
                    SETB(p_bis->data[0].data_info, LLD_BIS_DATA_DONE, 1);

                    // Clear ACL buffer pointer
                    em_ble_rxdataptr_setf(lld_env.curr_rxdesc_index, 0);

                    // Keep received value, next one will have to be different
                    p_big->cssn = rx_cssn;

                    // Ensure new RX buffer will be allocated
                    acl_buf_ptr = 0;
                }
            }

            rx_status = RWIP_RX_OK;
        }
        while (0);

        // Check if the RX descriptor needs an ACL buffer
        if (!acl_buf_ptr)
        {
            // Find new buffer, link to the descriptor
            em_ble_rxdataptr_setf(lld_env.curr_rxdesc_index, ble_util_buf_rx_alloc());
        }

        // Maintain channel assessment information
        {
            // Get channel information from the exchange memory
            uint16_t rxchass   = em_ble_rxchass_get(lld_env.curr_rxdesc_index);
            uint8_t  rxrssi    = (rxchass & EM_BLE_RSSI_MASK) >> EM_BLE_RSSI_LSB;
            uint8_t  usedchidx = (rxchass & EM_BLE_USED_CH_IDX_MASK) >> EM_BLE_USED_CH_IDX_LSB;

            rwip_channel_assess_ble(usedchidx, rx_status, rxrssi, timestamp, true);
        }


        // Free RX descriptor
        lld_rxdesc_free();
    }

    // Return reception status
    return (rx_status);
}
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Manage content of RX descriptor after end of channel scanner subevent
 ****************************************************************************************
 */
__STATIC void lld_bis_scan_rx(struct lld_bis_env *p_bis, struct lld_big_env *p_big, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_SCAN_RX_BIT, p_bis, p_big, timestamp);

    // Check if a descriptor has been used
    if (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(p_bis->act_id)))
    {
        // Get channel information from the exchange memory
        uint16_t rxchass   = em_ble_rxchass_get(lld_env.curr_rxdesc_index);
        uint8_t  rxrssi    = (rxchass & EM_BLE_RSSI_MASK) >> EM_BLE_RSSI_LSB;
        uint8_t  usedchidx = (rxchass & EM_BLE_USED_CH_IDX_MASK) >> EM_BLE_USED_CH_IDX_LSB;

        rwip_channel_assess_ble(usedchidx, RWIP_RX_OTHER_ERROR, rxrssi, timestamp, false);

        // Free RX descriptor
        lld_rxdesc_free();
    }
}
#endif //(BLE_BROADCASTER)

/**
 ****************************************************************************************
 * @brief Skip subevent
 ****************************************************************************************
 */
__STATIC void lld_bis_skip_subevent(struct lld_bi_se *p_se, bool prog)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_SKIP_SUBEVENT_BIT, p_se, prog);

    // Get BIS and BIG information for this subevent
    struct lld_bis_env *p_bis = LLD_BIS_GET(p_se->bis_act_id);
    struct lld_big_env *p_big = LLD_BIG_GET(p_bis->grp_hdl);
    // Get CS index for this subevent
    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_se->bis_act_id);
    // Retrieve sub event counter and flush counter
    uint8_t sub_evt_cnt = em_ble_isoevtcntl_subevtcnt_getf(cs_idx);
    uint8_t flush_cnt   = em_ble_isoevtcntl_flushcnt_getf(cs_idx);
    bool chan_scan = (!GETB(p_big->big_info, LLD_BIG_RECEIVER) && (p_big->nb_cntl_tx == 0) && p_bis->ctrl_se);

    // Increment them
    sub_evt_cnt++;
    if (!chan_scan)
        flush_cnt++;

    // Set the values in the CS
    em_ble_isoevtcntl_pack(cs_idx, flush_cnt, sub_evt_cnt);

    // Check if subevent has been programmed with data
    if (prog && GETB(p_se->state, LLD_BI_SE_STATUS_DATA_PROG))
    {
        // Mark used descriptor as done and move on next one
#if (BLE_OBSERVER)
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            em_ble_rxisoptr_rxdone_setf(p_se->desc_idx, 1);
            em_ble_isorxdescptr_set(cs_idx, em_ble_rxisoptr_nextptr_getf(p_se->desc_idx));
        }
#endif //(BLE_OBSERVER)
#if (BLE_BROADCASTER)
        if (!GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            em_ble_txisoptr_txdone_setf(p_se->desc_idx, 1);
            em_ble_isotxdescptr_set(cs_idx, em_ble_txisoptr_nextptr_getf(p_se->desc_idx));
        }
#endif //(BLE_BROADCASTER)
    }
    else if (!p_bis->ctrl_se)
    {
        // Get data offset
        uint8_t data_offset = p_big->data_map[p_se->bis_sub_evt_idx].offset;

        // Get information about data packet supposed to be transmitted or received during this sub event
        struct lld_bi_data *p_data = &p_bis->data[CO_MOD((p_big->data_pos + data_offset), p_big->nb_data_pkt)];

        // Load data for BIS
        lld_bis_data_load(p_big, p_bis, p_data, data_offset, NULL);
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_bi_evt_start_cbk(struct sch_arb_elt_tag *p_elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_EVT_START_CBK_2_FUNC_BIT, p_elt);

    DBG_SWDIAG(BI, START, 1);

    if (p_elt != NULL)
    {
        // Point to BIG parameters
        struct lld_big_env  *p_big    = (struct lld_big_env *)p_elt;
        // Get Sub-event structure reference for programming
        struct lld_bi_se    *p_ref_se = &(p_big->p_se_infos[p_big->ref_se_idx]);
        // Get pointer to information for the first sub-event to be scheduled
        struct lld_bi_se    *p_first_se = &(p_big->p_se_infos[p_big->in_proc_se_idx]);

        // Check if local synchronization is enabled and a new BIG event starts
        if ((p_big->cb_local_sync != NULL) && !GETB(p_big->big_info, LLD_BIG_EVT_ONGOING))
        {
            // Sample ISO counter and trigger GPIO0
            ble_isocntcntl_isosamp_setf(1);
            while (ble_isocntcntl_isosamp_getf());

            // Provide the associated timestamp via the callback
            p_big->cb_local_sync(ble_isocntsamp_get());
        }

        // Mark that programming is enabled
        SETB(p_big->big_info, LLD_BIG_PROG_EN, 1);

        // Program sub event
        lld_bi_prog(p_big, p_elt->current_prio);

        // Keep in mind that BIG event is in progress
        SETB(p_big->big_info, LLD_BIG_PROG, 1);

        // If at least one sub-event remaining, program a timer for next sub-event
        if (p_big->prog_se_idx < (p_big->ref_se_idx + p_ref_se->nb_packed))
        {
            struct lld_bi_se *p_next_se = &p_big->p_se_infos[p_big->prog_se_idx];

            // Get pointer to alarm
            struct sch_alarm_tag *p_alarm = &p_big->alarm;

            int32_t bit_off = p_first_se->bit_offset + (p_next_se->offset - p_first_se->offset);
            uint32_t target_clock = CLK_ADD_2(p_first_se->timestamp, bit_off / HALF_SLOT_SIZE);
            bit_off      = CO_MOD(bit_off, HALF_SLOT_SIZE);

            p_next_se->timestamp = target_clock;
            p_next_se->bit_offset = bit_off;

#if (BLE_OBSERVER)
            // Also set expected synchronization time for next sub events
            if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
            {
                int32_t exp_sync_bit_off = p_first_se->exp_sync_bit_off + (p_next_se->offset - p_first_se->offset);
                p_next_se->exp_sync_ts = CLK_ADD_2(p_first_se->exp_sync_ts, exp_sync_bit_off / HALF_SLOT_SIZE);
                exp_sync_bit_off = CO_MOD(exp_sync_bit_off, HALF_SLOT_SIZE);
                p_next_se->exp_sync_bit_off = exp_sync_bit_off;
            }
#endif //(BLE_OBSERVER)

            ASSERT_ERR(GETB(p_big->big_info, LLD_BIG_ALARM) == 0);

            // Program alarm in advance of targeted sub-event
            p_alarm->time.hs = CLK_SUB(p_next_se->timestamp, rwip_prog_delay);
            p_alarm->time.hus = p_next_se->bit_offset;

            // Indicate alarm timer is running
            SETB(p_big->big_info, LLD_BIG_ALARM, 1);
            SETB(p_big->big_info, LLD_BIG_ALARM_PROG, 1);

            // Set the alarm
            sch_alarm_set(p_alarm);
        }

        // New event of a BIG
        if (!GETB(p_big->big_info, LLD_BIG_EVT_ONGOING))
        {
            // Launch the computation of the hopping sequence for next event
            lld_bi_compute_hop_scheme(p_big, p_big->big_evt_cnt + 1);

            // Indicate BIG event ongoing
            SETB(p_big->big_info, LLD_BIG_EVT_ONGOING, 1);
        }
    }

    DBG_SWDIAG(BI, START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event stop notification
 ****************************************************************************************
 */
__STATIC void lld_bi_evt_stop_cbk(struct sch_arb_elt_tag *p_elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_EVT_STOP_CBK_2_FUNC_BIT, p_elt);

    DBG_SWDIAG(BI, START, 1);

    if (p_elt != NULL)
    {
        // Point to BIG parameters
        struct lld_big_env  *p_big    = (struct lld_big_env *)p_elt;

        // Mark that programming is disabled
        SETB(p_big->big_info, LLD_BIG_PROG_EN, 0);
    }

    DBG_SWDIAG(BI, START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_bi_evt_canceled_cbk(struct sch_arb_elt_tag *p_elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_EVT_CANCELED_CBK_2_FUNC_BIT, p_elt);

    DBG_SWDIAG(BI, CANCEL, 1);

    if (p_elt != NULL)
    {
        // Point to BIG parameters
        struct lld_big_env *p_big    = (struct lld_big_env *)p_elt;
        // Get Reference sub-event scheduling structure
        struct lld_bi_se   *p_ref_se = &(p_big->p_se_infos[p_big->ref_se_idx]);
        // Get pointer to alarm
        struct sch_alarm_tag *p_alarm = &p_big->alarm;

        // Increment priority
        p_elt->current_prio = RWIP_PRIO_ADD_2(p_elt->current_prio, RWIP_PRIO_INC(p_big->prio_idx));

        // Mark that scheduling has been rejected
        SETB(p_big->big_info, LLD_BIG_SCHED_REJECT, 1);

        ASSERT_ERR(GETB(p_big->big_info, LLD_BIG_ALARM) == 0);

        // Program alarm to handle scheduling rejected
        p_alarm->time.hs = p_ref_se->timestamp;
        p_alarm->time.hus = 0;

        // Indicate alarm timer is running
        SETB(p_big->big_info, LLD_BIG_ALARM, 1);

        // Set the alarm
        sch_alarm_set(p_alarm);
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(BI, CANCEL, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt (after end of each programmed sub event)
 ****************************************************************************************
 */
__STATIC void lld_bi_frm_isr(struct lld_bis_env *p_bis, struct lld_big_env *p_big, uint8_t sub_evt_idx,
                             uint32_t timestamp, uint8_t type, bool abort)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_FRM_ISR_2_FUNC_BIT, 6, p_bis, p_big, sub_evt_idx, timestamp, type, abort);
    // And scheduling structure
    struct sch_arb_elt_tag *p_elt = &p_big->elt;
    // Sub event status
    uint8_t se_status = 0;

    DBG_SWDIAG(BI, END, 1);

    ASSERT_INFO(p_big->in_proc_se_idx == sub_evt_idx, p_big->in_proc_se_idx, sub_evt_idx);

#if (BLE_OBSERVER)
    if (GETB(p_big->big_info, LLD_BIG_RECEIVER) && (type != LLD_BI_DUMMY_TYPE_CHAN_SCAN))
    {
        // Check reception status for this subevent
        if (lld_bis_rx(p_bis, p_big, timestamp) == RWIP_RX_OK)
        {
            SETB(se_status, LLD_BI_SE_STATUS_RX_OK, 1);
        }
    }
#endif //(BLE_OBSERVER)

    // Reset priority
    if (!abort)
    {
        p_elt->current_prio = rwip_priority[p_big->prio_idx].value;
    }

    // Handle end of sub event
    if (!lld_bi_end_subevt(p_bis, p_big, se_status))
    {
        uint8_t cnt;
        struct lld_bi_se *p_ref_se = &p_big->p_se_infos[p_big->ref_se_idx];
        // maximum SE index of the packed scheduled group
        uint8_t max_se_idx = p_big->ref_se_idx + p_ref_se->nb_packed;

        // handled skipped sub-events
        for (cnt = p_big->in_proc_se_idx; cnt < max_se_idx ; cnt++)
        {
            // Get sub event information
            struct lld_bi_se *p_cur_se = &p_big->p_se_infos[cnt];
            // next element programmed (and not skipped) - nothing more to do
            if (!GETB(p_cur_se->state, LLD_BI_SE_STATUS_SKIP))
            {
                break;
            }

            // Handle end of sub event --> rejected
            lld_bi_end_subevt(LLD_BIS_GET(p_cur_se->bis_act_id), p_big, LLD_BI_SE_STATUS_REJ_BIT);
        }
    }

    DBG_SWDIAG(BI, END, 0);
}

/**
 ****************************************************************************************
 * @brief Handle skip interrupt
 ****************************************************************************************
 */
__STATIC void lld_bi_skip_isr(struct lld_bis_env *p_bis, struct lld_big_env *p_big, uint8_t sub_evt_idx)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_SKIP_ISR_2_FUNC_BIT, p_bis, p_big, sub_evt_idx);

    // Get information about skipped sub_event
    struct lld_bi_se *p_se = &p_big->p_se_infos[sub_evt_idx];

    DBG_SWDIAG(BI, SKIP, 1);

    // Mark sub-event as skipped
    SETB(p_se->state, LLD_BI_SE_STATUS_SKIP, true);

    // If the skipped sub-event is first element of program queue, handle the end, otherwise it will be done later
    if (sub_evt_idx == p_big->in_proc_se_idx)
    {
        // End the sub-event
        lld_bi_end_subevt(p_bis, p_big, 0);
    }

    DBG_SWDIAG(BI, SKIP, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_bi_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_FRM_CBK_2_FUNC_BIT, timestamp, dummy, irq_type);

    // Extract type of event
    uint8_t             type        = (uint8_t)GETF(dummy, LLD_BI_DUMMY_TYPE);
    // Get BIS structure
    struct lld_bis_env *p_bis       = LLD_BIS_GET((uint8_t)GETF(dummy, LLD_BI_DUMMY_ACT_ID));
    // Get BIG structure
    struct lld_big_env *p_big       = LLD_BIG_GET((uint8_t)GETF(dummy, LLD_BI_DUMMY_GRP_HDL));
    // Extract subevent index
    uint8_t             sub_evt_idx = (uint8_t)GETF(dummy, LLD_BI_DUMMY_SUB_EVT_IDX);

    if (p_bis && p_big)
    {
        // Choose the appropriate handler based on the type of interruption
        switch (irq_type)
        {
        case (SCH_FRAME_IRQ_EOF):
        {
            lld_bi_frm_isr(p_bis, p_big, sub_evt_idx, timestamp, type, false);
        }
        break;
        case (SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO):
        case (SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO):
        {
            lld_bi_frm_isr(p_bis, p_big, sub_evt_idx, timestamp, type, true);
        }
        break;

#if (BLE_OBSERVER)
        case (SCH_FRAME_IRQ_RX_ISO):
        {
            // Don't do anything
        } break;
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
        case (SCH_FRAME_IRQ_TX_ISO):
        {
            // Don't do anything
        } break;
#endif //(BLE_BROADCASTER)

        case (SCH_FRAME_IRQ_SKIP):
        {
            lld_bi_skip_isr(p_bis, p_big, sub_evt_idx);
        }
        break;

#if (BLE_BROADCASTER)
        case (SCH_FRAME_IRQ_RX):
        {
            lld_bis_scan_rx(p_bis, p_big, timestamp);
        }
        break;
#endif //(BLE_BROADCASTER)

        default:
        {
            ASSERT_INFO(0, dummy, irq_type);
        }
        break;
        }
    }
    else
    {
        ASSERT_INFO(0, dummy, irq_type);
    }
}

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Initialize the periodic ADV ACAD data
 *
 * @param activity_id       BIG activity identifier
 * @param p_big_info_data   BIG Info data packed
 * @param encrypted         BIG is encrypted or not
 *
 ****************************************************************************************
 */
__STATIC void lld_big_init_info(uint8_t activity_id, const uint8_t *p_big_info_data, bool encrypted)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_INIT_INFO_BIT, activity_id, p_big_info_data, encrypted);

    uint16_t big_info_len = (encrypted ? BLE_EXT_ACAD_BIG_INFO_ENC_LEN : BLE_EXT_ACAD_BIG_INFO_LEN);
    // pick an extended header that will be used to replace the periodic adv header.
    // Put the ACAD data at end of buffer to ensure that there is enough space for periodic adv extended header info
    // + 2 for ad_type + length
    uint16_t em_ptr = EM_BLE_ADVEXTHDRTXBUF_OFF(activity_id) + EM_BLE_ADVEXTHDRTXBUF_SIZE - (BLE_EXT_ACAD_BIG_INFO_ENC_LEN + 2);
    // ensure that the extended header is able to provide at least maximum AE Header length
    ASSERT_INFO(EM_BLE_ADVEXTHDRTXBUF_SIZE >= BLE_EXT_MAX_HEADER_LEN, EM_BLE_ADVEXTHDRTXBUF_SIZE, activity_id);

    // Adv Data length
    // + 1 for ad_type
    em_wr8p(em_ptr + 0, big_info_len + 1);
    // Adv Data type
    em_wr8p(em_ptr + 1, BLE_EXT_ACAD_BIG_INFO_AD_TYPE);
    // copy Big Info in exchange memory
    em_wr(p_big_info_data, em_ptr + 2, BLE_EXT_ACAD_BIG_INFO_ENC_LEN);
}
#endif //(BLE_BROADCASTER)


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */



uint8_t lld_big_start(struct lld_big_params *p_big_params, struct big_info *p_big_info, const uint8_t *p_big_info_data,
                      struct lld_big_start_time *p_big_start_time)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_START_BIT, uint8_t, p_big_params, p_big_info, p_big_info_data, p_big_start_time);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    do
    {
        // Get BIG structure for provided group handle
        struct lld_big_env *p_big = LLD_BIG_GET(p_big_params->grp_hdl);
        // Counter
        uint8_t cnt;
        // Pointer to BIS
        struct lld_bis_env *p_bis;
        uint32_t clock;

        // ====================================================================
        // STREAM INITIALIZATION
        // ====================================================================

        // Check if a BIG with same group handle has already been started
        if (p_big)
        {
            break;
        }

        // Allocate a new BIG structure
        p_big = lld_big_alloc(p_big_params, p_big_info, p_big_params->nb_pt_evt, p_big_start_time);

        if (!p_big)
        {
            // Not enough memory
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            break;
        }

        for (cnt = 0; cnt < p_big_info->num_bis + 1; cnt++)
        {
            // ====================================================================
            // BIS INITIALIZATION
            // ====================================================================

            // Get BIS parameters
            struct lld_bis_params *p_bis_params = &p_big_params->bis_params[cnt];

            // Allocate BIS structure
            p_bis = lld_bis_alloc(p_big_params, p_bis_params, p_big,
                                  p_big_info->seed_access_addr, p_big_info->sub_interval,
                                  p_big_info->bis_spacing);

            if (!p_bis)
            {
                lld_big_free(p_big, false, false);

                status = CO_ERROR_MEMORY_CAPA_EXCEED;
                break;
            }

            // Initialize descriptors
            lld_bis_configure_descs(p_bis, p_big);

            if (!p_bis->ctrl_se)
            {
                // Compute synchronization reference offset for the specified mode (Vol 6 Part G Chapter 3.2)
                int32_t sync_ref_offset = p_big_params->update_offset + (p_big_info->framing == ISO_FRAMED_MODE) * (p_big_info->sdu_interval + p_big_params->iso_interval_us);

                // Start ISOAL for transmission / reception
                lld_isoal_start(p_bis->act_id,
                                (p_big_params->role == ROLE_BROADCASTER ? ISO_SEL_TX : ISO_SEL_RX),
                                BLE_ACTID_TO_BISHDL(p_bis->act_id),
                                p_big_info->sdu_interval,
                                p_big_params->trans_latency,
                                sync_ref_offset,
                                p_big_info->max_sdu,
                                p_big_params->iso_interval_us,
                                p_big_info->bn,
                                p_big_info->max_pdu,
                                p_big_info->framing,
                                p_big_info->encrypted,
                                false);
            }

            // ====================================================================
            // CONTROL STRUCTURE INITIALIZATION
            // ====================================================================

            // Fill control structure
            lld_bi_fill_cs(p_bis, p_big, p_big_params, ((p_big_info->base_crc_init << 8) | p_big_params->bis_params[cnt].idx), p_big_info->giv);

#if (BLE_BROADCASTER)
            if (p_bis->ctrl_se && !GETB(p_big->big_info, LLD_BIG_RECEIVER))
            {
                // Fill control structure for channel scan
                lld_bi_scan_fill_cs(p_bis, p_big);
            }
#endif //(BLE_BROADCASTER)
        }

#if (BLE_BROADCASTER)
        // ====================================================================
        // INITIALIZE BIS SYNC INFO DATA
        // ====================================================================
        if (p_big_params->role == ROLE_BROADCASTER)
        {
            lld_big_init_info(p_big_params->bis_params[0].act_id, p_big_info_data,
                              GETB(p_big->big_info, LLD_BIG_ENCRYPT));
        }
#endif // (BLE_BROADCASTER)

        // ====================================================================
        // SCHEDULING INITIALIZATION
        // ====================================================================
        status = lld_big_compute_sched(p_big);

        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // Point on first scheduling element
        p_big->ref_se_idx = 0;

        GLOBAL_INT_DISABLE();

        // ====================================================================
        // SCHEDULING
        // ====================================================================

        // read current clock value
        clock = lld_read_clock();
#if (BLE_OBSERVER)
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            // Allow 1 half-slot to complete the hopping calculation before starting the next one
            clock = CLK_ADD_2(clock, (rwip_prog_delay + 1));
        }
#endif // (BLE_OBSERVER)

#if (BLE_OBSERVER)
        p_big->first_big_evt_cnt = p_big->big_evt_cnt;
#endif //(BLE_OBSERVER)

        // Schedule the first scheduling event
        lld_bi_sched(p_big, clock);

        if (!GETB(p_big->big_info, LLD_BIG_SCHED_REJECT))
        {
            // First computation of hopping for first event
            lld_bi_compute_hop_scheme(p_big, p_big->big_evt_cnt);
        }

        // Toggle the hopping scheme buffer
        TOGB(p_big->big_info, LLD_BIG_HOP_TOG);

        GLOBAL_INT_RESTORE();

    }
    while (0);

    return (status);
}

uint8_t lld_big_stop(uint8_t grp_hdl, uint8_t reason)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_STOP_FUNC_BIT, uint8_t, grp_hdl, reason);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();
    // Get associated BIG structure
    struct lld_big_env *p_big = LLD_BIG_GET(grp_hdl);

    // Check if indicated BIG exists
    if (p_big != NULL)
    {
        // Set stop reason
        p_big->stop_reason = reason;

        // If BIG event is in progress do not stop the BIG now
        if (!GETB(p_big->big_info, LLD_BIG_PROG))
        {
            // Remove the event from the EA
            sch_arb_remove(&p_big->elt, false);

            // Free the BIG structure
            lld_big_free(p_big, true, false);
        }
        else
        {
            // Keep in mind that BIG has to be stopped
            SETB(p_big->big_info, LLD_BIG_STOP, 1);
        }

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_bis_local_sync_en(uint8_t act_id, data_path_local_sync_cb cb_local_sync)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_LOCAL_SYNC_EN_BIT, uint8_t, act_id, cb_local_sync);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get BIS structure
    struct lld_bis_env *p_bis = LLD_BIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated BIS exists
    if (p_bis != NULL)
    {
        // Get BIG structure
        struct lld_big_env *p_big = LLD_BIG_GET(p_bis->grp_hdl);

        // Store the callback
        p_big->cb_local_sync = cb_local_sync;

        // Indicate local sync enabled for the BIS
        p_big->dp_local_sync_en_bf |= (1 << act_id);

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_bis_local_sync_dis(uint8_t act_id, data_path_local_sync_cb cb_local_sync)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_LOCAL_SYNC_DIS_BIT, uint8_t, act_id, cb_local_sync);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get BIS structure
    struct lld_bis_env *p_bis = LLD_BIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated BIS exists
    if (p_bis != NULL)
    {
        // Get BIG structure
        struct lld_big_env *p_big = LLD_BIG_GET(p_bis->grp_hdl);

        // Indicate local sync disabled for the BIS
        p_big->dp_local_sync_en_bf &= ~(1 << act_id);

        // If local sync disabled in all BISes
        if (p_big->dp_local_sync_en_bf == 0)
        {
            // Clear the callback
            p_big->cb_local_sync = NULL;
        }

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#if (BLE_OBSERVER)
uint8_t lld_bis_peer_sync_en(uint8_t act_id, data_path_peer_sync_cb cb_peer_sync)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_PEER_SYNC_EN_BIT, uint8_t, act_id, cb_peer_sync);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get BIS structure
    struct lld_bis_env *p_bis = LLD_BIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated BIS exists
    if (p_bis != NULL)
    {
        // Get BIG structure
        struct lld_big_env *p_big = LLD_BIG_GET(p_bis->grp_hdl);

        // Store the callback
        p_big->cb_peer_sync = cb_peer_sync;

        // Indicate peer sync enabled for the BIS
        p_big->dp_peer_sync_en_bf |= (1 << act_id);

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_bis_peer_sync_dis(uint8_t act_id, data_path_peer_sync_cb cb_peer_sync)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_PEER_SYNC_DIS_BIT, uint8_t, act_id, cb_peer_sync);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get BIS structure
    struct lld_bis_env *p_bis = LLD_BIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated BIS exists
    if (p_bis != NULL)
    {
        // Get BIG structure
        struct lld_big_env *p_big = LLD_BIG_GET(p_bis->grp_hdl);

        // Indicate peer sync disabled for the BIS
        p_big->dp_peer_sync_en_bf &= ~(1 << act_id);

        // If peer sync disabled in all BISes
        if (p_big->dp_peer_sync_en_bf == 0)
        {
            // Clear the callback
            p_big->cb_peer_sync = NULL;
        }

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // (BLE_OBSERVER)

uint16_t lld_big_event_counter_get(uint8_t grp_hdl)
{
    uint16_t evt_cnt = 0;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_EVENT_COUNTER_GET_BIT, uint16_t, grp_hdl);

    GLOBAL_INT_DISABLE();
    struct lld_big_env *p_big = LLD_BIG_GET(grp_hdl);

    if (p_big != NULL)
    {
        // Return the next BIG event
        evt_cnt = p_big->big_evt_cnt + GETB(p_big->big_info, LLD_BIG_EVT_ONGOING);
    }
    GLOBAL_INT_RESTORE();

    return (evt_cnt);
}

uint8_t lld_big_ch_map_update(uint8_t grp_hdl, struct le_ch_map *p_map, uint16_t instant)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_CHMAP_UPDATE_BIT, uint8_t, grp_hdl, p_map, instant);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    GLOBAL_INT_DISABLE();

    // Get BIG structure
    struct lld_big_env *p_big = LLD_BIG_GET(grp_hdl);

    // Check if indicated BIG exists
    if (p_big != NULL)
    {
        // Verify that no update procedure is ongoing
        if (p_big->instant_proc.type == INSTANT_PROC_NO_PROC)
        {
            // Store new parameters
            p_big->instant_proc.type    = INSTANT_PROC_CH_MAP_UPD;
            p_big->instant_proc.instant = instant;
            memcpy(&p_big->instant_proc.data.ch_map.map[0], p_map, LE_CH_MAP_LEN);

            // No error triggered
            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            ASSERT_INFO(0, p_big->instant_proc.type, grp_hdl);
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#if (BLE_BROADCASTER)
uint8_t lld_big_cntl_tx(uint8_t grp_hdl, uint16_t em_ptr, uint8_t length, uint8_t nb_tx)
{
    // Status
    uint8_t status = CO_ERROR_NO_ERROR;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_CNTL_TX_BIT, uint8_t, grp_hdl, em_ptr, length, nb_tx);

    GLOBAL_INT_DISABLE();
    // Get BIG structure
    struct lld_big_env *p_big = LLD_BIG_GET(grp_hdl);

    do
    {
        uint16_t tx_iso_buf_setup = 0;

        if (!p_big)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Only broadcaster can send a Control PDU
        if (GETB(p_big->big_info, LLD_BIG_RECEIVER))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check that transmission of an BIS control PDU is not already in progress
        if (p_big->nb_cntl_tx != 0)
        {
            status = CO_ERROR_CONTROLLER_BUSY;
            break;
        }

        // Initialize number of TX attempts
        p_big->next_nb_cntl_tx = nb_tx;

        // Initialize TX ISO buffer that contains pdu length
        SETF(tx_iso_buf_setup, EM_BLE_TXISOLENGTH, length + (GETB(p_big->big_info, LLD_BIG_ENCRYPT) ? MIC_LEN : 0));
        SETF(tx_iso_buf_setup, EM_BLE_TXISOLLID, LLID_CNTL);

        // Copy buffer into exchange memory
        em_wr(&tx_iso_buf_setup, em_ptr, 2);

        // Set address of buffer in the exchange memory
        p_big->next_ctrl_se_em_ptr = em_ptr;

        if (!GETB(p_big->big_info, LLD_BIG_EVT_ONGOING))
        {
            // Update the control subevent
            lld_big_cntl_update(p_big);
        }
    }
    while (0);

    GLOBAL_INT_RESTORE();

    // Return status
    return (status);
}


#endif //(BLE_BROADCASTER)


uint8_t lld_bis_stats_get(uint8_t act_id, uint32_t *crc_error_packets, uint32_t *rx_unreceived_packets)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BIS_STATS_GET_2_FUNC_BIT, uint8_t, act_id, crc_error_packets, rx_unreceived_packets);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // retrieve
    struct lld_bis_env *p_bis = LLD_BIS_GET(act_id);
    // Get associated BIG structure
    struct lld_big_env *p_big = (p_bis != NULL) ? LLD_BIG_GET(p_bis->grp_hdl) : NULL;

    GLOBAL_INT_DISABLE();

    // Check that BIG is enabled
    if ((p_big != NULL) && !GETB(p_big->big_info, LLD_BIG_STOP) && GETB(p_big->big_info, LLD_BIG_RECEIVER))
    {
        // Get index of used control structure
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

        *crc_error_packets        = (em_ble_counter4_get(cs_idx, 1) << 16) | em_ble_counter4_get(cs_idx, 0);
        *rx_unreceived_packets    = (em_ble_counter5_get(cs_idx, 1) << 16) | em_ble_counter5_get(cs_idx, 0);

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_bis_pld_cnt_get(uint8_t act_id, uint32_t *pld_cnt)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIS_PLD_CNT_GET_BIT, uint8_t, act_id, pld_cnt);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get BIG/BIS structures
    struct lld_bis_env *p_bis = LLD_BIS_GET(act_id);
    struct lld_big_env *p_big = (p_bis != NULL) ? LLD_BIG_GET(p_bis->grp_hdl) : NULL;

    GLOBAL_INT_DISABLE();

    // Check that BIG is enabled
    if ((p_big != NULL) && !GETB(p_big->big_info, LLD_BIG_STOP))
    {
        // Get the first 4 bytes of the payload counter
        memcpy(pld_cnt, &p_big->bis_pkt_cnt[0], sizeof(uint32_t));
        *pld_cnt -= p_big->bn;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // (BLE_BIS_IN_ROM)


void lld_bi_init_patch(bool is_reset)
{
    //FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_INIT_2_FUNC_BIT, init_type);
#if !(BLE_BIS_IN_ROM)
    if (is_reset)
    {
        // Counter
        uint8_t cnt;

        // Flush all created BIG
        for (cnt = 0; cnt < BLE_ISO_GROUP_MAX; cnt++)
        {
            struct lld_big_env *p_big = LLD_BIG_GET(cnt);

            if (p_big)
            {
                // Clean the structure
                lld_big_free(p_big, false, true);
            }
        }
    }

    // by default set BIS and BIG pointers to null
    memset(&lld_big_env, 0, sizeof(lld_big_env));
    memset(&lld_bis_env, 0, sizeof(lld_bis_env));

#endif // BLE_BIS_IN_ROM
}

#if (BLE_BROADCASTER)

uint8_t lld_big_update_info_patch(uint8_t chan_0_act_id, uint32_t per_adv_timestamp, uint16_t per_ext_hdr_em_ptr, uint8_t max_acad_size)
{
    //FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_BI_1_PATCH_TYPE, LLD_BIG_UPDATE_INFO_BIT, uint8_t, chan_0_act_id, per_adv_timestamp, per_ext_hdr_em_ptr, max_acad_size);

    uint8_t acad_len = 0;
#if !(BLE_BIS_IN_ROM)

    // retrieve
    struct lld_bis_env *p_bis = LLD_BIS_GET(chan_0_act_id);
    // Get associated BIG structure
    struct lld_big_env *p_big = (p_bis != NULL) ? LLD_BIG_GET(p_bis->grp_hdl) : NULL;

    // Check that BIG is enabled
    if ((p_big != NULL) && !GETB(p_big->big_info, LLD_BIG_STOP))
    {
        acad_len = (GETB(p_big->big_info, LLD_BIG_ENCRYPT)
                    ? BLE_EXT_ACAD_BIG_INFO_ENC_LEN : BLE_EXT_ACAD_BIG_INFO_LEN) + 2 ;// + 2 for ad_type + length

        // If BIGInfo can fit
        if (acad_len <= max_acad_size)
        {
            uint32_t temp;
            uint32_t big_offset = 0;
            uint32_t big_unit;
            uint32_t evt_cnt_diff  = 0;
            uint16_t bis_pkt_cnt[3];

            // Compute if following anchor point and ensure that we have at least 1 slot between anchor_ts and next periodic adv packet
            if (p_big->anchor_ts < (per_adv_timestamp + 2))
            {
                evt_cnt_diff = (CLK_DIFF(p_big->anchor_ts, per_adv_timestamp + 2) + (p_big->iso_interval - 1)) / p_big->iso_interval;
            }

            // here BIG offset is in half us
            big_offset = CLK_DIFF(per_adv_timestamp, CLK_ADD_2(p_big->anchor_ts, evt_cnt_diff * p_big->iso_interval)) * HALF_SLOT_SIZE;
            // Get BIG offset in us
            big_offset = CO_DIVIDE_CEIL(big_offset, 2);

            // use 300 us big offset step
            if (big_offset >= BLE_BIG_OFFSET_30_US_MAX)
            {
                big_unit = 1;
                big_offset = big_offset / 300; // we must compute truncated value
            }
            else // Use 30us big offset step
            {
                big_unit = 0;
                big_offset = big_offset / 30; // we must compute truncated value
            }

            // pick an extended header that will be used to replace the periodic adv header.
            // Put the ACAD data at end of buffer to ensure that there is enough space for periodic adv extended header info
            // + 2 for ad_type + length
            uint16_t em_ptr = EM_BLE_ADVEXTHDRTXBUF_OFF(chan_0_act_id) + EM_BLE_ADVEXTHDRTXBUF_SIZE
                              - (BLE_EXT_ACAD_BIG_INFO_ENC_LEN);


            // read field that contains BIG offset information
            temp = em_rd32p(em_ptr + BIG_OFFSET_POS);

            // set Big Offset and Big offset unit fields
            SETF(temp, BIG_OFFSET,      big_offset);
            SETF(temp, BIG_OFFSET_UNIT, big_unit);

            // update value field in acad data
            em_wr32p(em_ptr + BIG_OFFSET_POS, temp);

            // ** Physical Channel Info **
            // Channel map 37 bits - PHY 3 bits
            em_wr(p_big->ch_map.map, em_ptr + BIG_CH_MAP_LSB_POS, LE_CH_MAP_LEN - 1);
            // Read-modify-write the last byte to include the PHY field
            em_rd(&temp, em_ptr + BIG_CH_MAP_MSB_POS, 1);
            SETF(temp, BIG_CH_MAP_MSB, p_big->ch_map.map[LE_CH_MAP_LEN - 1]);
            em_wr(&temp, em_ptr + BIG_CH_MAP_MSB_POS, 1);

            // copy packet counter
            memcpy(bis_pkt_cnt, p_big->bis_pkt_cnt, sizeof(bis_pkt_cnt));
            LLD_ISO_PYLD_CNT_ADD(bis_pkt_cnt, p_big->bn * evt_cnt_diff);
            // Payload Counter 39 bits - Framing 1 bit
            em_wr((uint8_t *)bis_pkt_cnt, em_ptr + BIG_BIS_PLD_COUNT_LSB_POS, BLE_PLD_CNT_SIZE - 1);
            // Read-modify-write the last byte to include the Framing field
            em_rd(&temp, em_ptr + BIG_BIS_PLD_COUNT_MSB_POS, 1);
            SETF(temp, BIG_BIS_PLD_COUNT_MSB, (uint8_t) bis_pkt_cnt[2]);
            em_wr(&temp, em_ptr + BIG_BIS_PLD_COUNT_MSB_POS, 1);

            // update exchange memory pointer according to periodic advertiser base extended header and ad_type + length
            em_ptr -= 2;

            // copy periodic ADV extended header for first transmission
            em_cpy(per_ext_hdr_em_ptr, em_ptr, acad_len);
        }
        else // BIGInfo cannot fit
        {
            acad_len = 0;
        }
    }
#endif //BLE_BIS_IN_ROM
    return acad_len;
}
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
void lld_bi_sync_time_update_patch(uint8_t act_id, uint32_t last_sync_ts, int32_t sync_drift_acc)
{
    // Find BIG corresponding to this ACL link
    struct lld_big_env *p_big;
    int cnt = 0;

    //FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_BI_2_PATCH_TYPE, LLD_BI_SYNC_TIME_UPDATE_2_FUNC_BIT, act_id, last_sync_ts, sync_drift_acc);
#if !(BLE_BIS_IN_ROM)
    // Parse all groups
    for (; cnt < BLE_ISO_GROUP_MAX; cnt++)
    {
        p_big = lld_big_env[cnt];

        // Check if activity ID matches and BIG event is not ongoing
        if (p_big && (p_big->per_sync_id == act_id) && !GETB(p_big->big_info, LLD_BIG_PROG))
        {
            // Get drift since last drift value
            int32_t drift_val = sync_drift_acc - p_big->sync_drift_acc;
            // BIG anchor timestamp - half microseconds part
            int32_t big_anchor_bit_off = p_big->anchor_bit_off + drift_val;

            // Update BIG anchor position
            while (big_anchor_bit_off < 0)
            {
                big_anchor_bit_off += HALF_SLOT_SIZE;
                p_big->anchor_ts = CLK_SUB(p_big->anchor_ts, 1);
            }

            p_big->anchor_ts = CLK_ADD_2(p_big->anchor_ts, big_anchor_bit_off / HALF_SLOT_SIZE);
            p_big->anchor_bit_off = CO_MOD(big_anchor_bit_off, HALF_SLOT_SIZE);
            p_big->last_sync_ts = last_sync_ts;
            p_big->sync_drift_acc = sync_drift_acc;

            // If event is scheduled, update timings, assuming the sync window reduces thanks to new synchronization infos
            if (!GETB(p_big->big_info, LLD_BIG_PROG) && !GETB(p_big->big_info, LLD_BIG_ALARM))
            {
                // Get event parameters
                struct sch_arb_elt_tag *p_evt = &p_big->elt;
                // Get packed event structure structure
                struct lld_bi_se       *p_ref_se   = &(p_big->p_se_infos[p_big->ref_se_idx]);
                // Get pointer to information for the first subevent to be scheduled
                struct lld_bi_se       *p_first_se = &(p_big->p_se_infos[p_big->in_proc_se_idx]);

                // Expected next anchor point (half microsecond part)
                int32_t bit_off = p_big->anchor_bit_off + p_first_se->offset;
                // Expected next anchor point (half slot part)
                uint32_t target_clock = CLK_ADD_2(p_big->anchor_ts, bit_off / HALF_SLOT_SIZE);

                // Compute anchor point for first sub-event based on BIG anchor point
                bit_off = CO_MOD(bit_off, HALF_SLOT_SIZE);

                // Keep expected synchronization time
                p_first_se->exp_sync_ts = target_clock;
                p_first_se->exp_sync_bit_off = bit_off;

                // Compute RX timings
                p_big->rx_win_size = lld_rx_timing_compute(p_big->last_sync_ts, &target_clock, &bit_off, p_big->sca, p_big->rate, 0);

                p_first_se->timestamp  = target_clock;
                p_first_se->bit_offset = bit_off;

                // Update the activity arbitration element
                p_evt->time.hs      = p_first_se->timestamp;
                p_evt->time.hus     = p_first_se->bit_offset;
                p_evt->duration_min = p_ref_se->duration_packed - (p_first_se->offset - p_ref_se->offset);
#if (BLE_OBSERVER)
                p_evt->duration_min += p_big->rx_win_size / 2;
#endif // (BLE_OBSERVER)
            }
        }
    }
#endif // BLE_BIS_IN_ROM
}
#endif //(BLE_OBSERVER)

#endif //(BLE_BIS)

///@} LLDBIS
