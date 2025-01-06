/**
****************************************************************************************
*
* @file lld_ci.c
*
* @brief LLD Connected Isochronous driver source code
*
* Copyright (C) RivieraWaves 2009-2018
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDCI
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"               // stack configuration

#if (BLE_CIS)

#include <string.h>
#include "co_math.h"
#include "ble_util.h"                  // BLE utility functions
#include "dbg.h"
#include "arch.h"                      // platform definition
#include "ke_mem.h"
#include "ke_task.h"                   // kernel task management
#include "rwip.h"
#include "lld.h"                       // link driver API
#include "lld_int.h"                   // link layer driver internal
#include "lld_int_iso.h"               // LLD Internal API for ISO
#include "reg_blecore.h"               // BLE core registers
#include "reg_em_ble_cs.h"             // BLE EM Control Structure
#include "reg_em_ble_rx_desc.h"        // BLE EM RX descriptors
#include "reg_em_ble_tx_iso_desc.h"    // BLE EM TX ISO descriptors
#include "reg_em_ble_rx_iso_desc.h"    // BLE EM RX ISO descriptors
#include "reg_em_ble_rx_iso_buf.h"     // BLE EM RX ISO buffer
#include "data_path.h"                 // Data path environment
#include "ble_util_buf.h"              // To manage ISO buffers

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
#define LLD_CI_MAX_SE_SCHED_DISTANCE           (6)

/// Distance between scheduling alarm and timestamp of the first scheduling event (in half slots)
#define LLD_CI_ALARM_INIT_DISTANCE             (6)

/// Minimum sub-event distance between long sync windows (for slave only) (in half-us)
#define LLD_CI_SYNC_WIN_MIN_DISTANCE           (1000)

/// If the CIS supervision timer reaches 6 * ISO_Interval before the CIS is established, the CIS shall be considered lost
#define LLD_CI_EST_SUP_TO                      (6)

/*
 * MACROS
 *****************************************************************************************
 */

/// Compute the flush instant for a given inserted data packets for a given CIS
///     FI = FC + (NSE x FT) - ((BN - n) x FLOOR(NSE / BN)) - 1, 1 <= n <= BN
/// with
///     FI = Flush instant
///     FC = Flush counter
///     BN = Burst number
///     FT = Flush Timeout
///     NSE = Number of subevents
#define LLD_CIS_GET_FLUSH_INSTANT(p_cis, p_data, cnt) \
    (p_cis->flush_cnt + (p_cis->hop_inf.nse * p_data->ft) - ((p_data->bn - cnt) * (p_cis->hop_inf.nse / p_data->bn)) - 1)

/// Get CIS structure
#define LLD_CIS_GET(act_id)      ((struct lld_cis_env *)lld_cis_env[act_id])
/// Get CIG structure
#define LLD_CIG_GET(grp_hdl)     ((struct lld_cig_env *)lld_cig_env[grp_hdl])

/*
 * ENUMERATION DEFINITION
 *****************************************************************************************
 */

/// Type of the instant-related procedure
enum lld_cis_instant_proc_type
{
    /// No procedure
    INSTANT_PROC_NO_PROC,
    /// Channel map update procedure
    INSTANT_PROC_CH_MAP_UPD,
};

/// CIS information bit field
///
///     9       8      7          6         5          4            3          2        1       0
/// +-------+-------+------+---------+---------+-------------+-----------+--------+--------+-------+
/// | ESTAB | TXACK | ENCRYPT | HOP_UPD | HOP_TOG |  UPDATE_CS  |  STARTED  |  SKIP  |  PROG  |  MIC  |
/// +-------+-------+------+---------+---------+-------------+-----------+--------+--------+-------+
///
enum lld_cis_info_fields
{
    /// Indicate that MIC Failure has been detected
    LLD_CIS_INFO_MIC_FAIL_POS    = 0,
    LLD_CIS_INFO_MIC_FAIL_BIT    = 0x0001,

    /// Indicate if one subevent has been programmed for this CIS event
    LLD_CIS_INFO_PROG_POS        = 1,
    LLD_CIS_INFO_PROG_BIT        = 0x0002,

    /// Indicate that sub events for this CIS can be skipped
    LLD_CIS_INFO_SKIP_POS        = 2,
    LLD_CIS_INFO_SKIP_BIT        = 0x0004,

    /// Indicate that CIS has been started
    LLD_CIS_INFO_STARTED_POS     = 3,
    LLD_CIS_INFO_STARTED_BIT     = 0x0008,

    /// Indicate that CS has to be updated by SW because subevents have been skipped and
    /// have not been seen by the HW
    LLD_CIS_INFO_UPDATE_CS_POS   = 4,
    LLD_CIS_INFO_UPDATE_CS_BIT   = 0x0010,

    /// HOP Sequence pointer in use
    LLD_CIS_INFO_HOP_TOG_POS     = 5,
    LLD_CIS_INFO_HOP_TOG_BIT     = 0x0020,

    /// HOP Sequence shall be updated
    LLD_CIS_INFO_HOP_UPD_POS     = 6,
    LLD_CIS_INFO_HOP_UPD_BIT     = 0x0040,

    /// Indicate if CIS is encrypted
    LLD_CIS_INFO_ENCRYPT_POS     = 7,
    LLD_CIS_INFO_ENCRYPT_BIT     = 0x0080,

    /// Indicate if Master acknowledgment of last isochronous packet received  has been transmitted
    LLD_CIS_INFO_TXACK_POS       = 8,
    LLD_CIS_INFO_TXACK_BIT       = 0x0100,

    /// Indicate if the CIS has been established
    LLD_CIS_INFO_ESTAB_POS       = 9,
    LLD_CIS_INFO_ESTAB_BIT       = 0x0200,
};

/// CIG information bit field
///
///     10           9           8          7         6       5      4       3       2          1         0
/// +---------+------------+-----------+---------+--------+------+-------+------+---------+------------+------+
/// | PROG_EN | ALARM_PROG | SCHED_REJ |  ALARM  |  INIT  | SYNC | SLAVE | WAIT | NEW_EVT | INTERLACED | PROG |
/// +---------+------------+-----------+---------+--------+------+-------+------+---------+------------+------+
///
enum lld_cig_info_fields
{
    /// Indicate that a subevent is programmed
    LLD_CIG_INFO_PROG_POS         = 0,
    LLD_CIG_INFO_PROG_BIT         = 0x0001,

    /// Indicate that streams of this CIG are interleaved
    LLD_CIG_INFO_INTERLEAVED_POS  = 1,
    LLD_CIG_INFO_INTERLEAVED_BIT  = 0x0002,

    /// Indicate that a CIG event is ongoing
    LLD_CIG_EVT_ONGOING_POS       = 2,
    LLD_CIG_EVT_ONGOING_BIT       = 0x0004,

    /// Indicate that CIG event is waiting for programming
    LLD_CIG_INFO_WAIT_POS         = 3,
    LLD_CIG_INFO_WAIT_BIT         = 0x0008,

    /// Indicate if device is slave of CIG
    LLD_CIG_INFO_SLAVE_POS        = 4,
    LLD_CIG_INFO_SLAVE_BIT        = 0x0010,

    /// Indicate that synchronization has been caught during a scheduling event
    LLD_CIG_INFO_SYNC_POS         = 5,
    LLD_CIG_INFO_SYNC_BIT         = 0x0020,

    /// Indicate that CIG is under initialization
    /// Only valid when slave in order to reduce effect of drift with master for scheduling of first CIS event
    LLD_CIG_INFO_INIT_POS         = 6,
    LLD_CIG_INFO_INIT_BIT         = 0x0040,

    /// Indicate that scheduling alarm is currently programmed
    LLD_CIG_INFO_ALARM_POS        = 7,
    LLD_CIG_INFO_ALARM_BIT        = 0x0080,

    /// Indicate that scheduling has been rejected
    LLD_CIG_SCHED_REJECT_POS      = 8,
    LLD_CIG_SCHED_REJECT_BIT      = 0x0100,

    /// Indicate that scheduling alarm is for programming new sub-events
    LLD_CIG_ALARM_PROG_POS        = 9,
    LLD_CIG_ALARM_PROG_BIT        = 0x0200,

    /// Programming of CIG sub-events is enabled
    LLD_CIG_PROG_EN_POS           = 10,
    LLD_CIG_PROG_EN_BIT           = 0x0400,

    /// Indicate the sub-events are programmed using synchronization information caught during the event (slave only)
    LLD_CIG_PROG_SYNC_POS         = 11,
    LLD_CIG_PROG_SYNC_BIT         = 0x0800,
};


/// Sub event status
///
///    7    6    5    4   3    2     1      0
/// +----+----+----+----+----+----+------+------+
/// |    |    |    |    |    |    | SKIP | PROG |
/// +----+----+----+----+----+----+------+------+
///
enum lld_ci_se_status_fields
{
    /// Indicate that Sub Event properly programmed
    LLD_CI_SE_STATUS_PROG_POS      = 0,
    LLD_CI_SE_STATUS_PROG_BIT      = 0x01,

    /// Indicate that Sub Event has been skipped
    LLD_CI_SE_STATUS_SKIP_POS      = 1,
    LLD_CI_SE_STATUS_SKIP_BIT      = 0x02,
};

/// Fields of dummy value provided to scheduling arbiter
///
///   31 - 24     23 - 16      15 - 8     7 - 0
/// +---------+-------------+----------+---------+
/// |   RFU  | SUB_EVT_IDX | ACT_ID   | GRP_HDL |
/// +---------+-------------+----------+---------+
///
enum lld_ci_dummy_fields
{
    /// Group Handle
    LLD_CI_DUMMY_GRP_HDL_LSB           = 0,
    LLD_CI_DUMMY_GRP_HDL_MASK          = 0x000000FF,

    /// Activity ID
    LLD_CI_DUMMY_ACT_ID_LSB            = 8,
    LLD_CI_DUMMY_ACT_ID_MASK           = 0x0000FF00,

    /// Sub event index
    LLD_CI_DUMMY_SUB_EVT_IDX_LSB       = 16,
    LLD_CI_DUMMY_SUB_EVT_IDX_MASK      = 0x00FF0000,
};

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// Union for storing the update information for any procedure
union lld_cis_instant_proc_new_par
{
    /// New channel map to used at instant
    struct le_ch_map ch_map;
};

/// Parameters update information
struct lld_cis_instant_proc_info
{
    /// Data associated with procedure
    union lld_cis_instant_proc_new_par data;

    /// Instant when the new parameter apply (compared to connection event counter)
    uint16_t instant;

    /**
     * Type of update procedure:
     *    - 0: No update ongoing
     *    - 1: Channel map update
     */
    uint8_t type;
};

/// Structure containing Data programation information about an CIS transmission or reception
struct lld_ci_data_prog_info
{
    /// Flush instant
    uint8_t            flush_instant;
    /// Index of used ISO descriptor
    uint8_t            desc_idx;
    /// Index of used buffer
    uint8_t            buf_idx;
    /// Programmed
    bool               prog;
};

/// Data management structure
struct lld_ci_data
{
    /// Payload counter (39 bits)
    uint16_t                         pld_cnt[3];
    /// Index of next ISO descriptor or buffer to be used - in iso_descs_idx and iso_bufs_idx arrays
    uint8_t                          next_iso_idx;
    /// Index of next ISO descriptor or buffer in use - in iso_descs_idx and iso_bufs_idx arrays
    uint8_t                          curr_iso_idx;
    /// Flush timeout (Range 0x01-0x1F)
    uint8_t                          ft;
    /// Number of new packets per ISO interval (Burst Number) (0: no new packet -  Range 0x01-0x1F)
    uint8_t                          bn;
    /// Maximum PDU size (Range 0x00-0xFB) (in bytes)
    uint8_t                          max_pdu;
    /// Array of used ISO descriptors
    uint8_t                          iso_descs_idx[BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS];
    /// Array of used ISO buffers
    uint8_t                          iso_bufs_idx[BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS];

    // Data queue management

    /// Index of data item in used
    uint8_t                          data_idx;
    /// Index of data item to prepare
    uint8_t                          data_prep_idx;
    /// Number of data item prepared
    uint8_t                          data_prep_nb;
    /// Number of data item programmed
    uint8_t                          data_prog_nb;
    /// Size of the data queue (BN * FT)
    uint8_t                          data_queue_size;
    /// Queue of data structures used for programming of descriptors and buffers (TX path)
    struct lld_ci_data_prog_info     data_queue[__ARRAY_EMPTY];
};


/// Subevent information
struct lld_ci_se
{
    ///Offsets between Group Anchor and sub-event (in half microseconds)
    uint32_t            offset;
    /// Timestamp to be programmed, half slot part
    uint32_t            timestamp;
    /// Duration of packed event (in half microseconds); valid if nb_packed != 0
    uint32_t            duration_packed;

#if (BLE_PERIPHERAL)
    /// Expected synchronization timestamp, half slot part
    uint32_t            exp_sync_ts;
    /// Expected synchronization timestamp, half microseconds part
    uint16_t            exp_sync_bit_off;
#endif //(BLE_PERIPHERAL)

    /// Timestamp to be programmed, half microseconds part
    uint16_t            bit_offset;
    /// Activity id of CIS this subevent belongs to
    uint8_t             cis_act_id;
    /// CIS Subevent index
    uint8_t             cis_sub_evt_idx;

    /// Indicate if sub event is currently programmed
    uint8_t             state;
    /// Number of subevents packed in same scheduled event
    /// Value != 0 for first sub_event of a packed sub-event
    uint8_t             nb_packed;
};

/// LLD CIG structure
struct lld_cig_env
{
    /// CIS event arbiter data - !!!! Must remain the first element !!!!
    struct sch_arb_elt_tag      evt;

    /// Alarm for 3 purposes: initializing CS at CIG start, handling a skip of an entire event, programming subevents
    struct sch_alarm_tag        alarm;

    /// Callback function used to provide the GPIO timestamp at local synchronization (NULL if local sync disabled)
    data_path_local_sync_cb     cb_local_sync;

    /// Callback function used to provide the event timing information to a data path (only for slave, NULL if peer sync disabled)
    data_path_peer_sync_cb      cb_peer_sync;

    /// Bit field providing activity ids for streams to be stopped upon next end of CIG event
    uint32_t                    stop_activity_ids;

    /// Data path local synchronization enable bit field (Bit 15:0: enable local sync for CIS TX, bit 31:16: enable local sync for CIS RX)
    uint32_t                     dp_local_sync_en_bf;

    /// Data path peer synchronization enable bit field (Bit 15:0: enable peer sync for CIS TX, bit 31:16: enable peer sync for CIS RX)
    uint32_t                     dp_peer_sync_en_bf;

    /// List of streams belonging to this CIG (sorted by stream offset values)
    struct co_list              list_cis;

    /// Pointer to an array containing subevent information
    struct lld_ci_se           *p_se_infos;

#if (BLE_PERIPHERAL)
    /// Scheduled RX window size (in half microseconds)
    uint32_t                    rx_win_size;
#endif //(BLE_PERIPHERAL)

    /// CIG anchor timestamp - half slot part
    uint32_t                    anchor_ts;
    /// CIG anchor timestamp - half microseconds part
    uint16_t                    anchor_bit_off;
    /// CIG event counter
    uint16_t                    cig_evt_cnt;

#if (BLE_PERIPHERAL)
    /// Offset with slave (in half microseconds)
    int32_t                     slave_offset;
    /// Drift accumulated between expected sync time and effective sync time (in half us)
    int32_t                     sync_drift_acc;
    /// Value of CLKN when the last sync has been detected (in half slots)
    uint32_t                    last_sync_ts;
    /// Master Sleep clock accuracy (in ppm)
    uint16_t                    master_sca;
#endif //(BLE_PERIPHERAL)
    /// ISO Interval (in half slots) - all streams have the same ISO interval
    uint16_t                    iso_interval;

    /// CIG info (@see enum lld_cig_info_fields)
    uint16_t                    cig_info;
    /// Total number of sub events for this CIG
    uint8_t                     cig_nse;
    /// Number of CIS for this CIG
    uint8_t                     nb_cis;
    /// Group Handle
    uint8_t                     grp_hdl;
    /// Priority index
    uint8_t                     prio_idx;

    /// Sub event index, indicate index of next sub-event processed by HW
    uint8_t                     in_proc_se_idx;
    /// Sub event index for programming, indicate index of next sub-event to be programmed in ET
    uint8_t                     prog_se_idx;
    /// Index of first sub-event in list of packed sub-events.
    uint8_t                     ref_se_idx;
    /// Number of sub events currently programmed
    uint8_t                     nb_prog;

#if (BLE_PERIPHERAL)
    /// Connection link id
    uint8_t                     con_link_id;
#endif //(BLE_PERIPHERAL)

    /// Current index that contains last valid Bluetooth timestamp value
    uint8_t                     bts_idx;
    /// Size of Bluetooth timestamp array
    uint8_t                     bts_size;
    /// Array of Group bluetooth timestamp
    uint32_t                    ref_anchor_bts_array[__ARRAY_EMPTY];
};

/// LLD CIS structure
struct lld_cis_env
{
    /// List header
    struct co_list_hdr               hdr;

    // -- Data information
    /// Transmission Data information
    struct lld_ci_data              *p_tx;
    /// Reception Data information
    struct lld_ci_data              *p_rx;

    // -- ISO stream info
    /// Sub event interval (in half microseconds)
    uint32_t                         sub_interval;
    /// CIS offset in the CIG (in half microseconds) - Time from CIG anchor to CIS anchor
    uint32_t                         cis_offset_in_cig;
    /// Duration of a sub event (in half microseconds)
    uint32_t                         sub_evt_dur;
    /// Pointer to the exchange memory where hopping sequences are computed
    uint16_t                         hop_ptr[2];
    /// CIS information (@see enum lld_cis_info_fields)
    uint16_t                         cis_info;
    /// Group Handle
    uint8_t                          grp_hdl;
    /// Activity identifier
    uint8_t                          act_id;
    /// Connection link id
    uint8_t                          con_link_id;
#if BLE_PWR_CTRL
    /// Transmit rate
    uint8_t                          tx_rate;
#endif // BLE_PWR_CTRL
    /// Last RSSI value received in dBm
    int8_t                           last_rssi;

    // CIS Specific
    /// Hopping specific information
    struct lld_iso_hop_inf           hop_inf;
    /// Connection update procedure information
    struct lld_cis_instant_proc_info instant_proc;
    /// Value of CLKN when the last correct CRC packet is received (in half-slots)
    uint32_t                         last_crc_ok_ts;
    /// CIG event counter for first insertion of CIS in CIG
    uint16_t                         start_cig_evt_cnt;
    /// CIG event counter when the CIS supervision timer is started
    uint16_t                         sup_to_start_cig_evt_cnt;
    /// Event counter
    uint16_t                         cis_evt_cnt;
    /// Amount by which the event counter should be incremented
    uint16_t                         evt_inc;
    /// Sub Event Counter
    uint8_t                          sub_evt_cnt;
    /// Flush Counter
    uint8_t                          flush_cnt;
    /// Stop reason
    uint8_t                          stop_reason;
    /// Number of CIS sub events currently programmed
    uint8_t                          nb_prog;

#if (BLE_PERIPHERAL) || (BLE_PWR_CTRL)
    /// Receive PHY Rate (0: 1Mbps | 1: 2 Mbps | 2: 125 Kbps | 3: 500 Kbps)
    uint8_t                          rx_rate;
#endif //(BLE_PERIPHERAL) || (BLE_PWR_CTRL)
};

#if (BLE_CIS_IN_ROM)
/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD CIS environment variable
__STATIC struct lld_cis_env *lld_cis_env[BLE_ACTIVITY_MAX];
/// LLD CIG environment variable
__STATIC struct lld_cig_env *lld_cig_env[BLE_ISO_GROUP_MAX];

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void lld_cis_prog(struct lld_cig_env *p_cig, uint8_t current_prio);
__STATIC void lld_ci_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
__STATIC void lld_ci_evt_start_cbk(struct sch_arb_elt_tag *p_evt);
__STATIC void lld_ci_evt_stop_cbk(struct sch_arb_elt_tag *p_evt);
__STATIC void lld_ci_evt_canceled_cbk(struct sch_arb_elt_tag *p_evt);
__STATIC bool lld_ci_end_subevt(struct lld_cis_env *p_cis, struct lld_cig_env *p_cig, bool rejected);
#if (BLE_PERIPHERAL)
    __STATIC void lld_ci_alarm_cbk(struct sch_alarm_tag *p_alarm);
#endif //(BLE_PERIPHERAL)
__STATIC bool lld_ci_sched(struct lld_cig_env *p_cig);

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Free an CIS structure. Send a LLD_CIS_STOP_IND to LLI task if required.
 *
 * @param[in] p_cis        Pointer to the CIS structure
 * @param[in] indicate     Indicate LLI task that driver has been stopped for the CIS
 * @param[in] reset        Used to know if a reset is on-going
 ****************************************************************************************
 */
__STATIC void lld_cis_free(struct lld_cis_env *p_cis, bool indicate, bool reset)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_FREE_FUNC_BIT, p_cis, indicate, reset);

    if (indicate)
    {
        if (p_cis->stop_reason == CO_ERROR_CONN_FAILED_TO_BE_EST)
        {
            // Inform link layer controller that the CIS failed to be established
            struct lld_cis_estab_ind *p_msg = KE_MSG_ALLOC(LLD_CIS_ESTAB_IND, TASK_LLI, TASK_NONE, lld_cis_estab_ind);
            p_msg->act_id = p_cis->act_id;
            p_msg->status = p_cis->stop_reason;
            ke_msg_send(p_msg);
        }
        else
        {
            // Inform link layer controller that driver is stopped for the provided CIS
            struct lld_cis_stop_ind *p_msg = KE_MSG_ALLOC(LLD_CIS_STOP_IND, TASK_LLI, TASK_NONE, lld_cis_stop_ind);
            p_msg->act_id = p_cis->act_id;
            p_msg->reason   = p_cis->stop_reason;
            ke_msg_send(p_msg);
        }
    }

    if (!reset)
    {
        // Counter
        uint8_t cnt;
        struct lld_ci_data *p_tx = p_cis->p_tx;
        struct lld_ci_data *p_rx = p_cis->p_rx;

        // stop ISOAL
        lld_isoal_stop(p_cis->act_id, p_cis->stop_reason);


        // Free TX ISO descriptors
        for (cnt = 0; cnt < BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS; cnt++)
        {
            ble_util_isodesc_free(p_tx->iso_descs_idx[cnt]);

            // Free TX ISO buffers
            if (p_tx->max_pdu > 0)
            {
                ble_util_buf_iso_free(p_tx->iso_bufs_idx[cnt]);
            }
        }

        // Free RX ISO descriptors
        for (cnt = 0; cnt < BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS; cnt++)
        {
            ble_util_isodesc_free(p_rx->iso_descs_idx[cnt]);

            // Free RX ISO buffers
            if (p_rx->max_pdu > 0)
            {
                ble_util_buf_iso_free(p_rx->iso_bufs_idx[cnt]);
            }
        }

        // Hopping sequence free
        ble_util_hop_seq_free(p_cis->hop_ptr[0]);
        ble_util_hop_seq_free(p_cis->hop_ptr[1]);

        // Cancel hopping computation
        lld_iso_hop_cancel(&(p_cis->hop_inf));

        // Update CIS environment
        lld_cis_env[p_cis->act_id] = NULL;
    }

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(EM_BLE_CS_ACT_ID_TO_INDEX(p_cis->act_id))), REG_EM_BLE_CS_SIZE, false, false, false);

    // Free the CIS structure
    ke_free(p_cis);
}

/**
 ****************************************************************************************
 * @brief Free scheduling information for a given CIG structure.
 *
 * @param[in] p_cig        Pointer to the CIG structure
 ****************************************************************************************
 */
__STATIC void lld_cig_free_sched(struct lld_cig_env *p_cig)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIG_FREE_SCHED_FUNC_BIT, p_cig);

    if (p_cig->p_se_infos)
    {
        // Free the array containing subevent information for scheduling
        ke_free(p_cig->p_se_infos);
        p_cig->p_se_infos = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Free an CIG structure
 *
 * @param[in] p_cig      Pointer to the CIG structure
 ****************************************************************************************
 */
__STATIC void lld_cig_free(struct lld_cig_env *p_cig)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIG_FREE_FUNC_BIT, p_cig);

    // Stop alarm
    sch_alarm_clear(&p_cig->alarm);

    // Free scheduling information
    lld_cig_free_sched(p_cig);

    // Update CIG environment
    lld_cig_env[p_cig->grp_hdl] = NULL;

    // Free the CIG structure
    ke_free(p_cig);
}

/**
 ****************************************************************************************
 * @brief Insert a CIS structure in the list of CISes for a given CIG.
 * CIS elements are sorted by CIS offset values in the CIG
 *
 * @param[in] p_cis     CIS structure
 * @param[in] p_cig     CIG structure
 ****************************************************************************************
 */
__STATIC void lld_cis_insert(struct lld_cis_env *p_cis, struct lld_cig_env *p_cig)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_INSERT_FUNC_BIT, p_cis, p_cig);

    // Get first CIS element
    struct lld_cis_env *p_list_cis = (struct lld_cis_env *)co_list_pick(&p_cig->list_cis);

    if (!p_list_cis)
    {
        // Insert the CIS at the beginning of the list
        co_list_push_front(&p_cig->list_cis, &p_cis->hdr);
    }
    else
    {
        while (p_list_cis)
        {
            if (p_cis->cis_offset_in_cig <= p_list_cis->cis_offset_in_cig)
            {
                // Insert the CIS
                co_list_insert_before(&p_cig->list_cis, &p_list_cis->hdr, &p_cis->hdr);
                break;
            }

            // Get next element
            p_list_cis = (struct lld_cis_env *)(p_list_cis->hdr.next);
        }

        if (!p_list_cis)
        {
            // Insert the CIS at the end of the list
            co_list_push_back(&p_cig->list_cis, &p_cis->hdr);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Set scheduling information for all sub events within a CIG (interlaced case)
 *
 * @param[in] p_cig      CIG structure for which the scheduling information are computed
 ****************************************************************************************
 */
__STATIC void lld_cig_compute_sched_int(struct lld_cig_env *p_cig)
{
    // Sub event index
    uint8_t se_idx = 0;
    // Sub event counter
    uint8_t sub_evt_cnt = 0;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIG_COMPUTE_SCHED_INT_FUNC_BIT, p_cig);

    DBG_SWDIAG(CI_DATA, COMP_SCHED_INT, 1);

    // Fill array of subevent information
    while (true)
    {
        // Number of subevents to be added for next sub event cnt value
        uint8_t nb_added_se = 0;
        // Get first CIS information
        struct lld_cis_env *p_cis = (struct lld_cis_env *)co_list_pick(&p_cig->list_cis);

        while (p_cis)
        {
            // Check if CIS has been started
            if (GETB(p_cis->cis_info, LLD_CIS_INFO_STARTED))
            {
                if (sub_evt_cnt < p_cis->hop_inf.nse)
                {
                    // Get information for this subevent
                    struct lld_ci_se *p_se = &p_cig->p_se_infos[se_idx];

                    p_se->cis_act_id      = p_cis->act_id;
                    p_se->offset          = p_cis->cis_offset_in_cig + (sub_evt_cnt * p_cis->sub_interval);
                    p_se->cis_sub_evt_idx = sub_evt_cnt;
                    p_se->nb_packed       = 0;
                    p_se->state           = 0;

                    se_idx++;

                    // Check if a sub event will be added for next sub event counter value
                    if ((sub_evt_cnt + 1) < p_cis->hop_inf.nse)
                    {
                        nb_added_se++;
                    }
                }
            }

            // Get next CIS
            p_cis = (struct lld_cis_env *)(p_cis->hdr.next);
        }

        if (!nb_added_se)
        {
            break;
        }

        sub_evt_cnt++;
    }

    DBG_SWDIAG(CI_DATA, COMP_SCHED_INT, 0);
}

/**
 ****************************************************************************************
 * @brief Set scheduling information for all sub events within a CIG (sequential case)
 *
 * @param[in] p_cig      CIG structure for which the scheduling information are computed
 ****************************************************************************************
 */
__STATIC void lld_cig_compute_sched_seq(struct lld_cig_env *p_cig)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIG_COMPUTE_SCHED_SEQ_FUNC_BIT, p_cig);

    // Sub event index
    uint8_t se_idx = 0;
    // Get first CIS information
    struct lld_cis_env *p_cis = (struct lld_cis_env *)co_list_pick(&p_cig->list_cis);

    DBG_SWDIAG(CI_DATA, COMP_SCHED_SEQ, 1);

    // Fill array of subevent information
    while (p_cis)
    {
        // Check if CIS has been started
        if (GETB(p_cis->cis_info, LLD_CIS_INFO_STARTED))
        {
            // Counter
            uint8_t se_cnt;
            for (se_cnt = 0; se_cnt < p_cis->hop_inf.nse; se_cnt++)
            {
                // Get subevent information
                struct lld_ci_se *p_se = &p_cig->p_se_infos[se_idx];

                p_se->cis_act_id      = p_cis->act_id;
                p_se->cis_sub_evt_idx = se_cnt;
                p_se->offset          = p_cis->cis_offset_in_cig + (se_cnt * p_cis->sub_interval);
                p_se->state           = 0;
                p_se->nb_packed       = 0;

                se_idx++;
            }
        }
        // Get next CIS
        p_cis = (struct lld_cis_env *)(p_cis->hdr.next);
    }

    DBG_SWDIAG(CI_DATA, COMP_SCHED_SEQ, 0);
}

/**
 ****************************************************************************************
 * @brief Compute scheduling information after introduction or deletion of a CIS for
 * a given CIG
 *
 * @param[in] p_cig       Pointer to the CIG structure
 *
 * @return CO_ERROR_NO_ERROR if scheduling scheme has been properly established,
 *         CO_ERROR_MEMORY_CAPA_EXCEED if scheduling cannot be established due to lack of memory
 *         resources
 ****************************************************************************************
 */
__STATIC uint8_t lld_cig_compute_sched(struct lld_cig_env *p_cig)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIG_COMPUTE_SCHED_FUNC_BIT, uint8_t, p_cig);

    // Returned status
    uint8_t status = CO_ERROR_NO_ERROR;

    DBG_SWDIAG(CI_DATA, COMP_SCHED, 1);

    // Free content of scheduling list
    lld_cig_free_sched(p_cig);

    // Allocate array that will contains all subevent information for scheduling
    p_cig->p_se_infos = ke_malloc_system(sizeof(struct lld_ci_se) * p_cig->cig_nse, KE_MEM_ENV);

    if (p_cig->p_se_infos)
    {
        // Sub event index
        uint8_t             se_idx = 0;
        // Subevent information
        struct lld_ci_se   *p_se;
        struct lld_ci_se   *p_se_ref;
        // CIS structure for this subevent
        struct lld_cis_env *p_cis;
        // End of subevent offset
        uint32_t            end_se_offset;

        // Compute subevent positions
        if (GETB(p_cig->cig_info, LLD_CIG_INFO_INTERLEAVED))
        {
            lld_cig_compute_sched_int(p_cig);
        }
        else
        {
            lld_cig_compute_sched_seq(p_cig);
        }

        // Information of first sub-event of packed sub-events
        p_se     = &(p_cig->p_se_infos[0]);
        p_se_ref = &(p_cig->p_se_infos[0]);
        p_cis = LLD_CIS_GET(p_se->cis_act_id);

        // End of subevent offset for reference sub event
        end_se_offset = p_se_ref->offset + p_cis->sub_evt_dur;

        // Now separate subevent into scheduling events based on distance between subevents
        for (se_idx = 0 ; se_idx < p_cig->cig_nse; se_idx++)
        {
            p_se  = &(p_cig->p_se_infos[se_idx]);
            p_cis = LLD_CIS_GET(p_se->cis_act_id);

            // Check distance with end of previously handled subevent
            if ((p_se != p_se_ref) && ((p_se->offset - end_se_offset) > HS_TO_HUS(LLD_CI_MAX_SE_SCHED_DISTANCE)))
            {
                // Use new reference for following sub-event packed together
                p_se_ref = p_se;
            }

            // Compute end of subevent offset
            end_se_offset = p_se->offset + p_cis->sub_evt_dur;

            // Initialize first packed sub-event
            p_se_ref->duration_packed = end_se_offset - p_se_ref->offset;
            p_se_ref->nb_packed      += 1;
        }
    }
    else
    {
        // Error, Scheduling computation not performed
        status = CO_ERROR_MEMORY_CAPA_EXCEED;
        ASSERT_ERR_FORCE(0);
    }


    DBG_SWDIAG(CI_DATA, COMP_SCHED, 0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Remove a given CIS.
 *
 * @param[in] p_cig      Pointer to the CIG structure the CIS belongs to
 * @param[in] p_cis      Pointer to the CIS structure
 *
 * @return true if CIG structure has been cl
 ****************************************************************************************
 */
__STATIC void lld_cis_remove(struct lld_cig_env *p_cig, struct lld_cis_env *p_cis)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_REMOVE_FUNC_BIT, p_cig, p_cis);

    // Remove CIS from the list of streans
    co_list_extract(&p_cig->list_cis, &p_cis->hdr);

    // Decrease number of streams for this CIG
    p_cig->nb_cis--;

    if (GETB(p_cis->cis_info, LLD_CIS_INFO_STARTED))
    {
        // Update number of subevents
        p_cig->cig_nse -= p_cis->hop_inf.nse;
    }

    // Free the CIS structure
    lld_cis_free(p_cis, true, false);
}

/**
 ****************************************************************************************
 * @brief Allocate an CIG structure
 *
 * @param[in] p_cis_params      Parameters for first CIS created in the new CIG
 * @param[in] p_con_cis_info    CIS parameters linked with BLE connection
 *
 * @return Pointer to the allocated CIG structure.
 ****************************************************************************************
 */
__STATIC struct lld_cig_env *lld_cig_alloc(struct lld_cis_params *p_cis_params,
        struct lld_con_info_for_cis *p_con_cis_info)
{
    uint8_t bts_size = co_max(p_cis_params->tx_ft, p_cis_params->rx_ft);

    // Allocate a new CIG structure
    struct lld_cig_env *p_cig = (struct lld_cig_env *)ke_malloc_system(sizeof(struct lld_cig_env)
                                + sizeof(uint32_t) * bts_size, KE_MEM_ENV);

    // Check if structure has been allocated
    if (p_cig)
    {
        // Pointer to arbitration parameters
        struct sch_arb_elt_tag *p_evt = &p_cig->evt;
#if (BLE_PERIPHERAL)
        // Pointer to alarm
        struct sch_alarm_tag *p_alarm = &p_cig->alarm;
#endif //(BLE_PERIPHERAL)

        // Clear content of the structure
        memset(p_cig, 0, sizeof(struct lld_cig_env));

        // Configure the CIG
        p_cig->grp_hdl           = p_cis_params->grp_hdl;
        p_cig->bts_size          = bts_size;
#if (BLE_PERIPHERAL)
        p_cig->con_link_id       = p_cis_params->con_link_id;
        p_cig->master_sca        = p_con_cis_info->master_sca;
        p_cig->sync_drift_acc    = p_con_cis_info->sync_drift_acc;
        p_cig->last_sync_ts      = p_con_cis_info->last_sync_ts;
#endif //(BLE_PERIPHERAL)
        // Interval kept in half slots
        p_cig->iso_interval      = FRAME_TO_HS(p_cis_params->iso_interval);
        p_cig->prio_idx          = (p_cis_params->role == MASTER_ROLE) ? RWIP_PRIO_M_CIS_IDX : RWIP_PRIO_S_CIS_IDX;
        p_cig->cig_evt_cnt       = (lld_read_clock() / p_cig->iso_interval);

        SETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE, p_cis_params->role);
        SETB(p_cig->cig_info, LLD_CIG_INFO_INIT, 1);

#if (BLE_PERIPHERAL)
        p_alarm->cb_alarm    = &lld_ci_alarm_cbk;
#endif //(BLE_PERIPHERAL)
        p_evt->cb_cancel     = &lld_ci_evt_canceled_cbk;
        p_evt->cb_start      = &lld_ci_evt_start_cbk;
        p_evt->cb_stop       = &lld_ci_evt_stop_cbk;
        p_evt->current_prio  = rwip_priority[p_cig->prio_idx].value;
        p_evt->time.hus      = 0;
        p_evt->stop_latency  = 0;
        SCH_ARB_ASAP_STG_SET(p_evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(p_cig->prio_idx));

        // Update CIG environment
        lld_cig_env[p_cis_params->grp_hdl] = p_cig;
    }

    return (p_cig);
}

/**
 ****************************************************************************************
 * @brief Allocate an CIS structure.
 *
 * @param[in]  p_cis_params      Parameters for first CIS created in the new CIG
 * @param[in]  p_con_cis_info    CIS parameters linked with BLE connection
 * @param[in]  p_cig             CIG structure
 * @param[in]  act_id            CIS activity id
 * @param[out] p_anchor_ts       Pointer to CIS anchor timestamp
 * @param[out] p_anchor_bit_off  Pointer to CIS anchor bit offset
 *
 * @return Pointer to the allocated CIS structure.
 ****************************************************************************************
 */
__STATIC struct lld_cis_env *lld_cis_alloc(struct lld_cis_params *p_cis_params,
        struct lld_con_info_for_cis *p_con_cis_info,
        struct lld_cig_env *p_cig, uint8_t act_id,
        uint32_t *p_anchor_ts, uint32_t *p_anchor_bit_off)
{
    uint32_t tx_env_size = CO_ALIGN4_HI(sizeof(struct lld_ci_data)
                                        + sizeof(struct lld_ci_data_prog_info) * p_cis_params->tx_ft * p_cis_params->tx_bn);
    uint32_t rx_env_size = CO_ALIGN4_HI(sizeof(struct lld_ci_data)
                                        + sizeof(struct lld_ci_data_prog_info) * p_cis_params->rx_ft * p_cis_params->rx_bn);

    // Allocate a new CIS structure + TX and RX queue management
    uint8_t *p_env       = (uint8_t *)ke_malloc_system(CO_ALIGN4_HI(sizeof(struct lld_cis_env)) + tx_env_size + rx_env_size,
                           KE_MEM_ENV);

    // Map allocated pointers
    struct lld_cis_env *p_cis = (struct lld_cis_env *) p_env;
    struct lld_ci_data *p_tx  = (struct lld_ci_data *)(p_env + CO_ALIGN4_HI(sizeof(struct lld_cis_env)));
    struct lld_ci_data *p_rx  = (struct lld_ci_data *)(p_env + CO_ALIGN4_HI(sizeof(struct lld_cis_env)) + tx_env_size);

    // Check if structure has been allocated
    if (p_cis)
    {
        // CIS anchor timestamp - half slot part
        uint32_t cis_anchor_ts;
        // CIS anchor timestamp - half microseconds part
        uint16_t cis_anchor_bit_off;
        // CIG anchor timestamp - half slot part
        uint32_t cig_anchor_ts;
        // CIG anchor timestamp - half microseconds part
        uint16_t cig_anchor_bit_off;
        // Counter
        uint8_t  cnt;
        uint32_t clock;
        int32_t  clock_diff;

        // clean-up environment
        memset(p_cis, 0, sizeof(struct lld_cis_env));
        memset(p_tx,  0, sizeof(struct lld_ci_data));
        memset(p_rx,  0, sizeof(struct lld_ci_data));

        p_cis->p_tx = p_tx;
        p_cis->p_rx = p_rx;

        // Initialize event parameters (connection part)
        p_cis->act_id            = act_id;
        p_cis->grp_hdl           = p_cis_params->grp_hdl;

        // CIS Link parameters
        p_cis->hop_inf.acc_code  = p_cis_params->access_addr;
        p_cis->hop_inf.nse       = p_cis_params->nse;
        p_cis->hop_inf.busy      = false;
        p_tx->ft                 = p_cis_params->tx_ft;
        p_tx->bn                 = p_cis_params->tx_bn;
        p_tx->max_pdu            = p_cis_params->tx_max_pdu;
        p_tx->data_queue_size    = p_cis_params->tx_ft * p_tx->bn;
        p_rx->ft                 = p_cis_params->rx_ft;
        p_rx->bn                 = p_cis_params->rx_bn;
        p_rx->max_pdu            = p_cis_params->rx_max_pdu;
        p_rx->data_queue_size    = p_cis_params->rx_ft * p_rx->bn;
        p_cis->sub_interval      = US_TO_HUS(p_cis_params->sub_interval);
        p_cis->sub_evt_dur       = US_TO_HUS(p_cis_params->sub_evt_dur);
#if (BLE_PERIPHERAL) || (BLE_PWR_CTRL)
        p_cis->rx_rate           = p_cis_params->rx_rate;
#endif //(BLE_PERIPHERAL) || (BLE_PWR_CTRL)
        p_cis->cis_offset_in_cig  = US_TO_HUS(p_cis_params->cis_spacing);
        p_cis->con_link_id       = p_cis_params->con_link_id;
#if BLE_PWR_CTRL
        p_cis->tx_rate           = p_cis_params->tx_rate;
#endif // BLE_PWR_CTRL
        // Save channel map
        memcpy(p_cis->hop_inf.ch_map.map, p_con_cis_info->ch_map.map, LE_CH_MAP_LEN);

        // Hopping sequence allocation
        p_cis->hop_ptr[0]        = ble_util_hop_seq_alloc();
        p_cis->hop_ptr[1]        = ble_util_hop_seq_alloc();
        SETB(p_cis->cis_info, LLD_CIS_INFO_HOP_TOG, 0);
        SETB(p_cis->cis_info, LLD_CIS_INFO_HOP_UPD, 1);
        SETB(p_cis->cis_info, LLD_CIS_INFO_ENCRYPT, p_con_cis_info->encrypted);

        // clear programmation counter
        p_cis->nb_prog           = 0;

        // Allocate TX ISO descriptors
        for (cnt = 0; cnt < BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS; cnt++)
        {
            p_tx->iso_descs_idx[cnt] = ble_util_isodesc_alloc();

            // Allocate TX ISO buffers
            if (p_tx->max_pdu > 0)
            {
                p_tx->iso_bufs_idx[cnt] = ble_util_buf_iso_alloc();
            }
        }

        // Allocate RX ISO descriptors
        for (cnt = 0; cnt < BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS; cnt++)
        {
            p_rx->iso_descs_idx[cnt] = ble_util_isodesc_alloc();

            // Allocate RX ISO buffers
            if (p_rx->max_pdu > 0)
            {
                p_rx->iso_bufs_idx[cnt] = ble_util_buf_iso_alloc();
            }
        }

        // Update CIS environment
        lld_cis_env[act_id] = p_cis;

        // Insert the CIS in the list of CISes for the provided CIG
        lld_cis_insert(p_cis, p_cig);

        // Increase number of CIS in the CIG
        p_cig->nb_cis++;

        // From second CIS, determine packing arrangement
        if (p_cig->nb_cis == 2)
        {
            SETB(p_cig->cig_info, LLD_CIG_INFO_INTERLEAVED, (p_cis_params->packing == ISO_PACKING_INTERLEAVED));
        }

        // Compute first anchor timestamp for the CIS
        cis_anchor_ts      = p_con_cis_info->ref_timestamp;
        cis_anchor_bit_off = p_con_cis_info->ref_bit_off;

        // Update anchor position with provided connection offset (connection offset already includes the CIS offset)
        cis_anchor_ts       = CLK_ADD_2(cis_anchor_ts, US_TO_HUS(p_cis_params->con_offset) / HALF_SLOT_SIZE);
        cis_anchor_bit_off += CO_MOD(US_TO_HUS(p_cis_params->con_offset), HALF_SLOT_SIZE);

        // Check if next slot has been crossed over
        if (cis_anchor_bit_off >= HALF_SLOT_SIZE)
        {
            cis_anchor_ts       = CLK_ADD_2(cis_anchor_ts, 1);
            cis_anchor_bit_off -= HALF_SLOT_SIZE;
        }

        // Compute the CIG anchor
        cig_anchor_ts      = CLK_SUB(cis_anchor_ts, US_TO_HUS(p_cis_params->cis_spacing) / HALF_SLOT_SIZE);
        cig_anchor_bit_off = cis_anchor_bit_off + HALF_SLOT_SIZE - CO_MOD(US_TO_HUS(p_cis_params->cis_spacing), HALF_SLOT_SIZE);

        // Check if next slot has been crossed over
        if (cig_anchor_bit_off >= HALF_SLOT_SIZE)
        {
            cig_anchor_bit_off -= HALF_SLOT_SIZE;
        }
        else
        {
            cig_anchor_ts = CLK_SUB(cig_anchor_ts, 1);
        }

        // read current clock
        clock = CLK_ADD_2(lld_read_clock(), 2 * rwip_prog_delay);
        clock_diff = CLK_DIFF(cis_anchor_ts, clock);

        // Start point might be in the past, check how many CIS events have been missed - Use a margin
        if (clock_diff > 0)
        {
            uint16_t evt_diff = CO_DIVIDE_CEIL(clock_diff, p_cig->iso_interval);

            cis_anchor_ts    = CLK_ADD_2(cis_anchor_ts, p_cig->iso_interval * evt_diff);
            cig_anchor_ts    = CLK_ADD_2(cig_anchor_ts, p_cig->iso_interval * evt_diff);
            p_cis->cis_evt_cnt += evt_diff;

            // Update payload counters
            LLD_ISO_PYLD_CNT_ADD(p_tx->pld_cnt, p_tx->bn * p_cis->cis_evt_cnt);
            LLD_ISO_PYLD_CNT_ADD(p_rx->pld_cnt, p_rx->bn * p_cis->cis_evt_cnt);
        }
        // retrieve the start event counter
        p_cis->start_cig_evt_cnt = p_cig->cig_evt_cnt;
        *p_anchor_ts        = cis_anchor_ts;
        *p_anchor_bit_off   = cis_anchor_bit_off;

        // If first CIS scheduled for the CIG, keep anchor position for the CIG
        if (p_cig->nb_cis == 1)
        {
            p_cig->anchor_ts      = cig_anchor_ts;
            p_cig->anchor_bit_off = cig_anchor_bit_off;
        }
        else
        {
            int16_t evt_diff;

            clock_diff = CLK_DIFF(p_cig->anchor_ts, cig_anchor_ts);
            evt_diff   = CO_DIVIDE_ROUND(co_abs(clock_diff), p_cig->iso_interval);

            // update start CIG event counter
            if (clock_diff > 0)
            {
                p_cis->start_cig_evt_cnt = p_cig->cig_evt_cnt + evt_diff;
            }
            else
            {
                ASSERT_ERR_FORCE(0); // Not yet supported

                // change CIG anchor
                p_cig->cig_evt_cnt = p_cig->cig_evt_cnt - evt_diff;
                p_cis->start_cig_evt_cnt = p_cig->cig_evt_cnt;

                p_cig->anchor_ts      = cig_anchor_ts;
                p_cig->anchor_bit_off = cig_anchor_bit_off;
            }
        }
    }

    return (p_cis);
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
__STATIC void lld_ci_compute_hop_scheme(struct lld_cis_env *p_cis, uint16_t hop_em_ptr, uint16_t evt_cnt)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CI_COMPUTE_HOP_SCHEME_FUNC_BIT, p_cis, hop_em_ptr, evt_cnt);

    struct lld_iso_hop_inf *p_hop_inf = &(p_cis->hop_inf);

    // Check if the instant associated with update procedure is reached or over-stepped
    if ((p_cis->instant_proc.type == INSTANT_PROC_CH_MAP_UPD)
            && BLE_UTIL_INSTANT_PASSED(p_cis->instant_proc.instant, evt_cnt))
    {
        memcpy(p_hop_inf->ch_map.map, p_cis->instant_proc.data.ch_map.map, LE_CH_MAP_LEN);

        // Clear update procedure
        p_cis->instant_proc.type = INSTANT_PROC_NO_PROC;
    }

    p_hop_inf->em_ptr  = hop_em_ptr;
    p_hop_inf->evt_cnt = evt_cnt;

    // Request computation of hopping for next event
    lld_iso_hop_compute(p_hop_inf);
}

/**
 ****************************************************************************************
 * @brief Prepare data transmission for the next CIS event. It is considered here that
 * the function is called during EOF interrupt handling and that flush counter value
 * has been incremented for the next sub-event.
 *
 * @param[in] p_cig Pointer to the CIG structure of current CIS
 * @param[in] p_cis Pointer to the CIS structure for which data has to be prepared.
 ****************************************************************************************
 */
__STATIC void lld_cis_data_prepare(struct lld_cig_env *p_cig, struct lld_cis_env *p_cis)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_DATA_PREPARE_FUNC_BIT, p_cig, p_cis);

    // Counter
    uint8_t cnt;
    struct lld_ci_data *p_tx = p_cis->p_tx;
    struct lld_ci_data *p_rx = p_cis->p_rx;

    // Check if CIS data has to be transmitted
    if (p_tx->max_pdu)
    {
        // Push BN data descriptors
        for (cnt = 1; cnt <= p_tx->bn; cnt++)
        {
            // Take a free data descriptor
            struct lld_ci_data_prog_info *p_data = &(p_tx->data_queue[p_tx->data_prep_idx]);

            DBG_SWDIAG(CI_DATA, PREPARE_TX, 1);

            // Sanity check
            ASSERT_INFO(p_tx->data_prep_nb < p_tx->data_queue_size, p_tx->data_prep_nb, p_tx->data_prep_idx);

            // Fill the data descriptor
            p_data->flush_instant = LLD_CIS_GET_FLUSH_INSTANT(p_cis, p_tx, cnt);
            p_data->prog = false;

            // Increment prepare data index
            CO_VAL_INC(p_tx->data_prep_idx, p_tx->data_queue_size);
            // increment number of data prepared
            p_tx->data_prep_nb++;

            DBG_SWDIAG(CI_DATA, PREPARE_TX, 0);
        }

        // update payload counter
        LLD_ISO_PYLD_CNT_ADD(p_tx->pld_cnt, p_tx->bn);
    }

    // Check if CIS data has to be received'
    if (p_rx->max_pdu)
    {
        // Push BN data descriptors
        for (cnt = 1; cnt <= p_rx->bn; cnt++)
        {
            // Take a free data descriptor
            struct lld_ci_data_prog_info *p_data = &(p_rx->data_queue[p_rx->data_prep_idx]);

            DBG_SWDIAG(CI_DATA, PREPARE_RX, 1);

            // Sanity check
            ASSERT_INFO(p_rx->data_prep_nb < p_rx->data_queue_size, p_rx->data_prep_nb, p_rx->data_prep_idx);

            // Fill the data descriptor
            p_data->flush_instant = LLD_CIS_GET_FLUSH_INSTANT(p_cis, p_rx, cnt);
            p_data->prog = false;

            // Increment prepare data index
            CO_VAL_INC(p_rx->data_prep_idx, p_rx->data_queue_size);

            // increment number of data prepared
            p_rx->data_prep_nb++;

            DBG_SWDIAG(CI_DATA, PREPARE_RX, 0);
        }

        // update payload counter
        LLD_ISO_PYLD_CNT_ADD(p_rx->pld_cnt, p_rx->bn);
    }
}

/**
 ****************************************************************************************
 * @brief Program the TX ISO descriptors to be used during the next CIS events.
 * Called during handling of TX ISO and during handling of Start IRQs.
 *
 * @param[in] p_cig  Pointer to the CIG structure.
 * @param[in] p_cis  Pointer to the CIS structure for which data has to be prepared.
 ****************************************************************************************
 */
__STATIC void lld_cis_data_tx_prog(struct lld_cig_env *p_cig, struct lld_cis_env *p_cis)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_DATA_TX_PROG_FUNC_BIT, p_cig, p_cis);

    do
    {
        // Get index of used control structure
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_cis->act_id);
        struct lld_ci_data *p_tx = p_cis->p_tx;
        uint8_t tx_desc_idx = p_tx->iso_descs_idx[p_tx->next_iso_idx];

        // Check if CIS data has to be transmitted
        if (p_tx->max_pdu == 0)
            break;

        // Check if remaining data to be programmed
        if (p_tx->data_prog_nb >= p_tx->data_prep_nb)
            break;

        // Check if descriptor can be used
        if (!em_ble_txisoptr_txdone_getf(tx_desc_idx))
            break;

        // Prepare a new packet
        {
            uint8_t data_prog_idx = CO_MOD((p_tx->data_idx + p_tx->data_prog_nb), p_tx->data_queue_size);
            // Get next data structure to be programmed
            struct lld_ci_data_prog_info *p_data = &(p_tx->data_queue[data_prog_idx]);
            uint8_t tx_buf_idx  = p_tx->iso_bufs_idx[p_tx->next_iso_idx];
            uint16_t pld_cnt[3];
            uint8_t  pld_offset = p_tx->data_prep_nb - p_tx->data_prog_nb;
            uint8_t  bts_idx    = p_cig->bts_idx;

            DBG_SWDIAG(CI_DATA, PROG_TX, 1);

            // Compute payload counter for this packet
            memcpy(&pld_cnt[0], &p_tx->pld_cnt[0], 3 * sizeof(uint16_t));
            LLD_ISO_PYLD_CNT_SUB(pld_cnt, pld_offset);
            CO_VAL_SUB(bts_idx, (pld_offset - 1) / p_tx->bn, p_cig->bts_size);

            // Set payload counter and flush instant
            em_ble_txisocnt0_txpld_cnt0_setf(tx_desc_idx,  pld_cnt[0]);
            em_ble_txisocnt1_txpld_cnt1_setf(tx_desc_idx,  pld_cnt[1]);
            em_ble_txisocnt2_pack(tx_desc_idx, p_data->flush_instant, pld_cnt[2] & EM_BLE_TXPLD_CNT2_MASK);

            em_ble_txcisph_pack(tx_desc_idx, /*txcisrfu1*/ 0,
                                /*txcisnpi*/  0,
                                /*txcisrfu0*/ 0,
                                /*txcie*/     0,
                                /*txsn*/      pld_cnt[0] & 0x1,
                                /*txnesn*/    0);

            // Set Buffer pointer
            em_ble_txisobufptr_setf(tx_desc_idx, ble_util_buf_iso_emptr_get(tx_buf_idx) >> 2);
            // Mark descriptor as valid
            em_ble_txisoptr_txdone_setf(tx_desc_idx, 0);

            // Update data structure
            p_data->prog     = true;
            p_data->desc_idx = tx_desc_idx;
            p_data->buf_idx  = tx_buf_idx;

            // check if program index and data index are equals
            if (data_prog_idx == p_tx->data_idx)
            {
                em_ble_isotxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_ISO_DESC, tx_desc_idx));
            }

            // request ISOAL to fill TX buffer
            lld_isoal_tx_get(p_cis->act_id, tx_buf_idx, p_cig->ref_anchor_bts_array[bts_idx], false);

            // Update index of next TX ISO descriptor to be used
            CO_VAL_INC(p_tx->next_iso_idx, BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS);

            // Clear ISO buffer for next index if txdone
            tx_desc_idx = p_tx->iso_descs_idx[p_tx->next_iso_idx];
            if (em_ble_txisoptr_txdone_getf(tx_desc_idx))
            {
                em_ble_txisobufptr_setf(tx_desc_idx, 0);
            }
            // Increment program data index
            CO_VAL_INC(data_prog_idx, p_tx->data_queue_size);
            // Increment number of data item programmed
            p_tx->data_prog_nb++;

            DBG_SWDIAG(CI_DATA, PROG_TX, 0);
        }
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Program the RX ISO descriptors to be used during the next CIS events.
 * Called during handling of RX ISO and during handling of Start IRQs.
 *
 * @param[in] p_cis Pointer to the CIS structure for which data has to be prepared.
 ****************************************************************************************
 */
__STATIC void lld_cis_data_rx_prog(struct lld_cis_env *p_cis)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_DATA_RX_PROG_FUNC_BIT, p_cis);

    do
    {
        // Get index of used control structure
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_cis->act_id);
        struct lld_ci_data *p_rx = p_cis->p_rx;
        uint8_t rx_desc_idx = p_rx->iso_descs_idx[p_rx->next_iso_idx];

        // Check if CIS data have to be received
        if (p_rx->max_pdu == 0)
        {
            em_ble_isorxdescptr_set(cs_idx, 0);
            break;
        }

        // Check if remaining data to be programmed
        if (p_rx->data_prog_nb >= p_rx->data_prep_nb)
            break;

        // Check if a descriptor is available to be used
        if (p_rx->data_prog_nb >= BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS)
            break;

        // Confirm descriptor can be used
        ASSERT_ERR(em_ble_rxisoptr_rxdone_getf(rx_desc_idx));

        {
            uint8_t data_prog_idx = CO_MOD(p_rx->data_idx + p_rx->data_prog_nb, p_rx->data_queue_size);
            // Get next data structure to be programmed
            struct lld_ci_data_prog_info *p_data = &(p_rx->data_queue[data_prog_idx]);
            uint8_t rx_buf_idx  = p_rx->iso_bufs_idx[p_rx->next_iso_idx];
            uint16_t pld_cnt[3];

            DBG_SWDIAG(CI_DATA, PROG_RX, 1);

            // Compute payload counter for this packet
            memcpy(&pld_cnt[0], &p_rx->pld_cnt[0], 3 * sizeof(uint16_t));
            LLD_ISO_PYLD_CNT_SUB(pld_cnt, p_rx->data_prep_nb - p_rx->data_prog_nb);

            // Set Descriptor info
            em_ble_rxisocnt0_rxpld_cnt0_setf(rx_desc_idx,  pld_cnt[0]);
            em_ble_rxisocnt1_rxpld_cnt1_setf(rx_desc_idx,  pld_cnt[1]);
            em_ble_rxisocnt2_pack(rx_desc_idx, p_data->flush_instant, pld_cnt[2] & EM_BLE_RXPLD_CNT2_MASK);

            // Set Buffer pointer
            em_ble_rxisobufsetup_invl_setf(rx_buf_idx, LLD_ISO_INVL_SYNC_ERR);
            em_ble_rxisobufptr_setf(rx_desc_idx, ble_util_buf_iso_emptr_get(rx_buf_idx) >> 2);
            // Mark descriptor as valid
            em_ble_rxisoptr_rxdone_setf(rx_desc_idx, 0);

            // Update data structure
            p_data->prog     = true;
            p_data->desc_idx = rx_desc_idx;
            p_data->buf_idx  = rx_buf_idx;

            // Update index of next RX ISO descriptor to be used
            CO_VAL_INC(p_rx->next_iso_idx, BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS);

            // check if program index and data index are equals
            if (data_prog_idx == p_rx->data_idx)
            {
                em_ble_isorxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_RX_ISO_DESC, rx_desc_idx));
            }

            // Increment program data index
            CO_VAL_INC(data_prog_idx, p_rx->data_queue_size);
            // Increment number of data item programmed
            p_rx->data_prog_nb++;

            DBG_SWDIAG(CI_DATA, PROG_RX, 0);
        }
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Check if RX and TX descriptors have to be flushed when a subevent is ended whereas
 * it has not been executed by the HW.
 * Done before incrementing flush counter value
 *
 * @param[in] p_cig Pointer to the CIG structure for which data has to be prepared.
 * @param[in] p_cis Pointer to the CIS structure for which data has to be prepared.
 ****************************************************************************************
 */
__STATIC void lld_cis_data_skip_subevt(struct lld_cig_env *p_cig, struct lld_cis_env *p_cis)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_DATA_SKIP_SUBEVT_FUNC_BIT, p_cig, p_cis);

    struct lld_ci_data *p_tx = p_cis->p_tx;
    struct lld_ci_data *p_rx = p_cis->p_rx;

    if (p_tx->max_pdu)
    {
        // Check if there is remaining data prepared for transmission
        if (p_tx->data_prep_nb > 0)
        {
            // Take first programmed data structure
            struct lld_ci_data_prog_info *p_data = &(p_tx->data_queue[p_tx->data_idx]);

            // Check if flush instant has been reached
            if (p_data->flush_instant == p_cis->flush_cnt)
            {
                uint8_t  bts_idx    = p_cig->bts_idx;
                CO_VAL_SUB(bts_idx, (p_tx->data_prep_nb - 1) / p_tx->bn, p_cig->bts_size);

                // Increment current data index : Point on next data item
                CO_VAL_INC(p_tx->data_idx, p_tx->data_queue_size);
                // Update index of next TX ISO descriptor to be used
                CO_VAL_INC(p_tx->curr_iso_idx, BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS);
                // decrement number of element prepared
                p_tx->data_prep_nb--;

                // Point CS To next ISO descriptor
                {
                    // Get index of used control structure
                    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_cis->act_id);
                    // Compute index of descriptor to be used
                    uint8_t tx_desc_idx = p_tx->iso_descs_idx[p_tx->curr_iso_idx];
                    // Clear ISO Buffer if the descriptor is done
                    if (em_ble_txisoptr_txdone_getf(tx_desc_idx))
                    {
                        em_ble_txisobufptr_setf(tx_desc_idx, 0);
                    }

                    em_ble_isotxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_ISO_DESC, tx_desc_idx));
                }

                if (p_data->prog)
                {
                    // decrement number of element programmed
                    p_tx->data_prog_nb--;
                    p_data->prog = false;
                    // Set TX done bit
                    em_ble_txisoptr_txdone_setf(p_data->desc_idx, 1);
                    // Clear ISO buffer
                    em_ble_txisobufptr_setf(p_data->desc_idx, 0);

                    // Program next data structure
                    lld_cis_data_tx_prog(p_cig, p_cis);
                }
                else
                {
                    // Inform ISOAL that buffer transmission is skipped (and never programmed)
                    lld_isoal_tx_get(p_cis->act_id, BLE_UTIL_ISO_INDEX_INVALID, p_cig->ref_anchor_bts_array[bts_idx], true);
                }
            }
        }
        // else nothing more to be done
    }

    if (p_rx->max_pdu)
    {
        // Check if there is remaining data prepared for reception
        if (p_rx->data_prep_nb > 0)
        {
            // Take first programmed data structure
            struct lld_ci_data_prog_info *p_data = &(p_rx->data_queue[p_rx->data_idx]);

            // Check if flush instant has been reached
            if (p_data->flush_instant == p_cis->flush_cnt)
            {
                uint8_t  bts_idx    = p_cig->bts_idx;
                CO_VAL_SUB(bts_idx, (p_rx->data_prep_nb - 1) / p_rx->bn, p_cig->bts_size);

                // Increment current data index : Point on next data item
                CO_VAL_INC(p_rx->data_idx, p_rx->data_queue_size);
                // Update index of next RX ISO descriptor to be used
                CO_VAL_INC(p_rx->curr_iso_idx, BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS);
                // decrement number of element prepared
                p_rx->data_prep_nb--;

                // Point CS To next ISO descriptor
                {
                    // Get index of used control structure
                    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_cis->act_id);
                    // Compute index of descriptor to be used
                    uint8_t rx_desc_idx = p_rx->iso_descs_idx[p_rx->curr_iso_idx];

                    em_ble_isorxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_RX_ISO_DESC, rx_desc_idx));
                }

                // Inform ISOAL that reception is skipped
                if (p_data->prog)
                {
                    lld_isoal_rx_done(p_cis->act_id, p_data->buf_idx, p_cig->ref_anchor_bts_array[bts_idx], false);
                }
                else
                {
                    lld_isoal_rx_done(p_cis->act_id, BLE_UTIL_ISO_INDEX_INVALID, p_cig->ref_anchor_bts_array[bts_idx], true);
                }

                if (p_data->prog)
                {
                    // decrement number of element programmed
                    p_rx->data_prog_nb--;
                    p_data->prog = false;

                    // Set RX done bit
                    em_ble_rxisoptr_rxdone_setf(p_data->desc_idx, 1);

                    // Program next data structure
                    lld_cis_data_rx_prog(p_cis);
                }
            }
        }
        // else nothing more to be done
    }
}

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Schedule next anchor points for next sub event to be scheduled.
 ****************************************************************************************
 */
__STATIC void lld_ci_alarm_cbk(struct sch_alarm_tag *p_alarm)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CI_ALARM_CBK_FUNC_BIT, p_alarm);

    // Get CIG for which alarm has been triggered
    struct lld_cig_env *p_cig = (struct lld_cig_env *)((uint32_t)p_alarm - offsetof(struct lld_cig_env, alarm));

    // Clear bit indicating that alarm is running
    SETB(p_cig->cig_info, LLD_CIG_INFO_ALARM, 0);

    if (GETB(p_cig->cig_info, LLD_CIG_ALARM_PROG))
    {
        bool cig_stop = false;

        // Get Reference sub-event scheduling structure
        struct lld_ci_se *p_ref_se = &(p_cig->p_se_infos[p_cig->ref_se_idx]);
        struct lld_ci_se *p_se = &p_cig->p_se_infos[p_cig->prog_se_idx];
        struct lld_cis_env *p_cis = LLD_CIS_GET(p_se->cis_act_id);

        // Get current time
        rwip_time_t current_time = rwip_time_get();

        /*
         * Check if sub-event need and can be programmed
         *
         * The sub-event is not programmed in following conditions:
         *    - Programming is disabled (from arbiter)
         *    - CIS event is over (data transmitted/received or other reason)
         *    - The alarm is processed too late for programming the HW (at least 1 half-slot in advance)
         *    - For slave only:
         *        - The reception window is larger than 2xsub-interval - margin, and a sub-event is already ongoing, to avoid potential race conditions
         */
        if ((!GETB(p_cig->cig_info, LLD_CIG_PROG_EN) && !GETB(p_cis->cis_info, LLD_CIS_INFO_SKIP)
                && CLK_GREATER_THAN_HUS(current_time.hs, current_time.hus, CLK_SUB(p_se->timestamp, 1), p_se->bit_offset))
#if (BLE_PERIPHERAL)
                || (GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE)
                    && ((p_cig->rx_win_size > (2 * p_cis->sub_interval - LLD_CI_SYNC_WIN_MIN_DISTANCE) && (p_cig->nb_prog > 0))))
#endif //(BLE_PERIPHERAL)
           )
        {
            // If no sub-event programmed and no other sub-event ongoing
            if (p_cig->nb_prog == 0)
            {
                // The end of the skipped sub-event can be processed here
                cig_stop = lld_ci_end_subevt(p_cis, p_cig, true);
            }
            else
            {
                // Mark sub-event as skipped
                SETB(p_se->state, LLD_CI_SE_STATUS_SKIP, true);
            }
        }
        else
        {
            // Program next sub event
            lld_cis_prog(p_cig, p_cig->evt.current_prio);
        }

        if (!cig_stop)
        {
            // Increment sub-event program counter
            p_cig->prog_se_idx += 1;

            // If at least one sub-event remaining, program a timer for next sub-event
            if (p_cig->prog_se_idx < (p_cig->ref_se_idx + p_ref_se->nb_packed))
            {
                struct lld_ci_se *p_next_se = &p_cig->p_se_infos[p_cig->prog_se_idx];

                // Get pointer to alarm
                struct sch_alarm_tag *p_alarm = &p_cig->alarm;
                ASSERT_ERR(GETB(p_cig->cig_info, LLD_CIG_INFO_ALARM) == 0);

                int32_t bit_off;
                uint32_t target_clock;

#if (BLE_PERIPHERAL)
                // Calculate the expected synchronization for the sub-event
                if (GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE))
                {
                    int32_t exp_sync_bit_off = p_se->exp_sync_bit_off + (p_next_se->offset - p_se->offset);
                    p_next_se->exp_sync_ts = CLK_ADD_2(p_se->exp_sync_ts, exp_sync_bit_off / HALF_SLOT_SIZE);
                    exp_sync_bit_off = CO_MOD(exp_sync_bit_off, HALF_SLOT_SIZE);
                    p_next_se->exp_sync_bit_off = exp_sync_bit_off;
                }

                // If sync found in one of the previous sub-events, adjust the programming timestamp
                if (GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE) && GETB(p_cig->cig_info, LLD_CIG_INFO_SYNC))
                {
                    // Timings are computed from the originally expected synchronization and the timings from last synchronization
                    bit_off = p_next_se->exp_sync_bit_off + p_cig->slave_offset - US_TO_HUS(BLE_NORMAL_WIN_SIZE) / 2;
                    target_clock = p_next_se->exp_sync_ts;

                    if (bit_off < 0)
                    {
                        uint8_t num_hs = CO_DIVIDE_CEIL((-bit_off), HALF_SLOT_SIZE);
                        bit_off += num_hs * HALF_SLOT_SIZE;
                        target_clock = CLK_SUB(target_clock, num_hs);
                    }
                    else if (bit_off >= HALF_SLOT_SIZE)
                    {
                        uint8_t num_hs = bit_off / HALF_SLOT_SIZE;
                        bit_off -= num_hs * HALF_SLOT_SIZE;
                        target_clock = CLK_ADD_2(target_clock, num_hs);
                    }

                    // Indicate the sub-events are now programmed with synchronization adjustment
                    SETB(p_cig->cig_info, LLD_CIG_PROG_SYNC, 1);
                }
                else
#endif //(BLE_PERIPHERAL)
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

                // Keep in mind that alarm timer is running
                SETB(p_cig->cig_info, LLD_CIG_INFO_ALARM, 1);

                // Set the alarm
                sch_alarm_set(p_alarm);
            }
            else
            {
                SETB(p_cig->cig_info, LLD_CIG_ALARM_PROG, 0);
            }
        }
    }
    else if (GETB(p_cig->cig_info, LLD_CIG_SCHED_REJECT))
    {
        uint8_t cnt;
        // Get packed event structure structure
        struct lld_ci_se *p_ref_se = &(p_cig->p_se_infos[p_cig->ref_se_idx]);
        // maximum SE index of the packed scheduled group
        uint8_t max_se_idx = p_cig->ref_se_idx + p_ref_se->nb_packed;

        SETB(p_cig->cig_info, LLD_CIG_SCHED_REJECT, 0);

        // Consider all sub-events in the scheduling structure as done
        for (cnt = p_cig->in_proc_se_idx; cnt < max_se_idx; cnt++)
        {
            // Get activity id for sub-event
            uint8_t cis_act_id = p_cig->p_se_infos[cnt].cis_act_id;

            // Handle end of sub event
            lld_ci_end_subevt(LLD_CIS_GET(cis_act_id), p_cig, true);
        }
    }
    else if (GETB(p_cig->cig_info, LLD_CIG_INFO_INIT))
    {
        // Get first CIS
        struct lld_cis_env *p_cis = (struct lld_cis_env *)co_list_pick(&p_cig->list_cis);

        p_cig->cig_nse = 0;
        p_cig->bts_idx = 0;

        // compute next group Bluetooth timestamp
        p_cig->ref_anchor_bts_array[p_cig->bts_idx] = rwip_bt_time_to_bts(p_cig->anchor_ts, p_cig->anchor_bit_off);

        while (p_cis != NULL)
        {
            if (p_cis->start_cig_evt_cnt == p_cig->cig_evt_cnt)
            {
                // Update number of sub events for the CIG
                p_cig->cig_nse += p_cis->hop_inf.nse;
                // Mark CIS started
                SETB(p_cis->cis_info, LLD_CIS_INFO_STARTED, 1);

                // prepare data immediately
                lld_cis_data_prepare(p_cig, p_cis);
            }

            p_cis = (struct lld_cis_env *) p_cis->hdr.next;
        }

        // compute scheduling
        lld_cig_compute_sched(p_cig);

        // Point on first scheduling element
        p_cig->ref_se_idx = 0;

        SETB(p_cig->cig_info, LLD_CIG_INFO_INIT, 0);

        // Schedule the next scheduling event
        lld_ci_sched(p_cig);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}
#endif //(BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Schedule next anchor points for next sub event to be scheduled.
 ****************************************************************************************
 */
__STATIC bool lld_ci_sched(struct lld_cig_env *p_cig)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_SCHED_2_FUNC_BIT, bool, p_cig);

    // Get event parameters
    struct sch_arb_elt_tag *p_evt = &p_cig->evt;
    // Indicate if event has been inserted or not
    bool timeout = false;

    DBG_SWDIAG(CI, SCHED, 1);

    do
    {
        // Get pointer to the reference sub-event of the scheduling event
        struct lld_ci_se       *p_ref_se   = &(p_cig->p_se_infos[p_cig->ref_se_idx]);
        // Get pointer to the first sub-event to be scheduled
        struct lld_ci_se       *p_first_se = &(p_cig->p_se_infos[p_cig->in_proc_se_idx]);

        // Expected next anchor point (half slot part)
        uint32_t target_clock;
        // Expected next anchor point (half microsecond part)
        int32_t bit_off;

        // Compute anchor point for first sub-event based on CIG anchor point
        bit_off = p_cig->anchor_bit_off + p_first_se->offset;
        target_clock = CLK_ADD_2(p_cig->anchor_ts, bit_off / HALF_SLOT_SIZE);
        bit_off = CO_MOD(bit_off, HALF_SLOT_SIZE);

#if (BLE_PERIPHERAL)
        // Calculate RX sync window for slave role
        if (GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE))
        {
            // Retrieve CIS of first subevent to be scheduled
            struct lld_cis_env *p_cis = (struct lld_cis_env *) LLD_CIS_GET(p_first_se->cis_act_id);
            // Window widening limit, after which a disconnection occurs (Core Spec LL 4.2.4)
            uint32_t win_lim = (p_cis->hop_inf.nse < 3) ? (uint32_t)(p_cig->iso_interval * HALF_SLOT_SIZE / 2 - US_TO_HUS(BLE_IFS_DUR)) : p_cis->sub_interval;

            // Keep expected synchronization time
            p_first_se->exp_sync_ts = target_clock;
            p_first_se->exp_sync_bit_off = bit_off;

            // Compute RX timings
            p_cig->rx_win_size = lld_rx_timing_compute(p_cig->last_sync_ts, &target_clock, &bit_off, p_cig->master_sca, p_cis->rx_rate, 0);

            // If the window widening value has reached its limit, disconnect CIS due to synchronization timeout
            if ((p_cig->rx_win_size / 2) >= win_lim)
            {
                timeout = true;
                break;
            }
        }
#endif //(BLE_PERIPHERAL)

        p_first_se->timestamp  = target_clock;
        p_first_se->bit_offset = bit_off;

        // Check that first sub-event is not in the past
        if (CLK_GREATER_THAN(p_first_se->timestamp, lld_read_clock()))
        {
            // Prepare the activity arbitration element
            p_evt->time.hs      = p_first_se->timestamp;
            p_evt->time.hus     = p_first_se->bit_offset;
            p_evt->duration_min = p_ref_se->duration_packed - (p_first_se->offset - p_ref_se->offset);
#if (BLE_PERIPHERAL)
            p_evt->duration_min += p_cig->rx_win_size / 2;
#endif // (BLE_PERIPHERAL)

            // And try to insert it
            if (sch_arb_insert(p_evt) == SCH_ARB_ERROR_OK)
            {
                // Keep in mind that for this CIG a scheduling event is waiting for programming
                SETB(p_cig->cig_info, LLD_CIG_INFO_WAIT, 1);

                break;
            }
        }

        // Event cannot be scheduled, increment priority
        p_evt->current_prio = RWIP_PRIO_ADD_2(p_evt->current_prio, RWIP_PRIO_INC(p_cig->prio_idx));

        // Mark that scheduling has been rejected
        SETB(p_cig->cig_info, LLD_CIG_SCHED_REJECT, 1);

        {
            // Get pointer to alarm
            struct sch_alarm_tag *p_alarm = &p_cig->alarm;

            ASSERT_ERR(GETB(p_cig->cig_info, LLD_CIG_INFO_ALARM) == 0);

            // Program alarm to handle scheduling rejected
            p_alarm->time.hs = p_first_se->timestamp;
            p_alarm->time.hus = 0;

            // Keep in mind that alarm timer is running
            SETB(p_cig->cig_info, LLD_CIG_INFO_ALARM, 1);

            // Set the alarm
            sch_alarm_set(p_alarm);
        }

    }
    while (0);

    if (timeout)
    {
        while (!co_list_is_empty(&(p_cig->list_cis)))
        {
            struct lld_cis_env *p_cis = (struct lld_cis_env *) co_list_pick(&(p_cig->list_cis));
            p_cis->stop_reason = CO_ERROR_CON_TIMEOUT;
            lld_cis_remove(p_cig, p_cis);
        }

        // Free the CIG structure
        lld_cig_free(p_cig);
    }

    DBG_SWDIAG(CI, SCHED, 0);

    return !timeout;
}

/**
 ****************************************************************************************
 * @brief Fill the exchange table
 *
 * @param p_cig         Pointer to the CIG structure
 * @param current_prio  Current priority
 ****************************************************************************************
 */
__STATIC void lld_cis_prog(struct lld_cig_env *p_cig, uint8_t current_prio)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_PROG_FUNC_BIT, p_cig, current_prio);

    // Get information of sub event
    struct lld_ci_se *p_se = &p_cig->p_se_infos[p_cig->prog_se_idx];
    // Get CIS structure
    struct lld_cis_env *p_cis = LLD_CIS_GET(p_se->cis_act_id);
    // Get index of used control structure
    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_se->cis_act_id);
    // Used to select the event type (primary/reTx) and the audio buffer (0/1)
    bool primary = (p_se->cis_sub_evt_idx == 0) ? true : false;
    // Programming parameters pushed to SCH PROG
    struct sch_prog_params prog_par;

    DBG_SWDIAG(CI, PROG, 1);

    prog_par.frm_cbk        = &lld_ci_frm_cbk;
    prog_par.time.hs        = p_se->timestamp;
    prog_par.time.hus       = p_se->bit_offset;
    prog_par.cs_idx         = cs_idx;
    prog_par.dummy          = cs_idx;
    prog_par.bandwidth      = p_cis->sub_evt_dur;
    prog_par.prio_1         = current_prio;
    prog_par.prio_2         = current_prio;
    prog_par.prio_3         = 0;
    prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
    prog_par.add.ble.ae_nps = 0;
    prog_par.add.ble.iso    = 1;
    prog_par.add.ble.rsvd   = primary;
    prog_par.add.ble.sic    = GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE) && !GETB(p_cig->cig_info, LLD_CIG_PROG_SYNC) && GETB(p_cig->cig_info, LLD_CIG_INFO_PROG);
    prog_par.mode           = SCH_PROG_BLE;

    SETF(prog_par.dummy, LLD_CI_DUMMY_ACT_ID,      p_cis->act_id);
    SETF(prog_par.dummy, LLD_CI_DUMMY_GRP_HDL,     p_cig->grp_hdl);
    SETF(prog_par.dummy, LLD_CI_DUMMY_SUB_EVT_IDX, p_cig->prog_se_idx);

    sch_prog_push(&prog_par);

    SETB(p_se->state, LLD_CI_SE_STATUS_PROG, true);
    p_cig->nb_prog     += 1;
    p_cis->nb_prog     += 1;

#if (BLE_PERIPHERAL)
    // Set sync window size in the control structure
    if (GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE))
    {
        // Get synchronization window size in microseconds
        uint32_t sync_win_size_us = GETB(p_cig->cig_info, LLD_CIG_PROG_SYNC) ? BLE_RATE_NORMAL_WIN_SIZE(p_cis->rx_rate) : HUS_TO_US(p_cig->rx_win_size + 1);

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
#endif //(BLE_PERIPHERAL)

    if (GETB(p_cis->cis_info, LLD_CIS_INFO_UPDATE_CS))
    {
        struct lld_ci_data *p_rx = p_cis->p_rx;

        // Set sub event counter
        em_ble_isoevtcntl_subevtcnt_setf(cs_idx, p_cis->sub_evt_cnt);
        // Set flush counter
        em_ble_isoevtcntl_flushcnt_setf(cs_idx, p_cis->flush_cnt);

        // Update next expected sequence number
        if (p_rx->max_pdu)
        {
            // Compute payload counter for this packet
            uint16_t pld_cnt[3];
            memcpy(&pld_cnt[0], &p_rx->pld_cnt[0], 3 * sizeof(uint16_t));
            LLD_ISO_PYLD_CNT_SUB(pld_cnt, p_rx->data_prep_nb);

            em_ble_isotxrxcntl_isolastnesn_setf(cs_idx, pld_cnt[0] & 0x1);
        }

        // Keep in mind that the CS has been filled for the CIS event
        SETB(p_cis->cis_info, LLD_CIS_INFO_UPDATE_CS, 0);
    }

    // If first sub-event programmed for the stream, prepare the control structure
    if (!GETB(p_cis->cis_info, LLD_CIS_INFO_PROG))
    {
        // Update the connection event counter
        p_cis->cis_evt_cnt += p_cis->evt_inc;
        p_cis->evt_inc = 0;
        em_ble_evtcnt_setf(cs_idx, p_cis->cis_evt_cnt);

        // Point to the hopping sequence to use
        em_ble_hopptr_hop_seq_ptr_setf(EM_BLE_CS_ACT_ID_TO_INDEX(p_cis->act_id), p_cis->hop_ptr[GETB(p_cis->cis_info, LLD_CIS_INFO_HOP_TOG)] >> 2);

        // Indicate the stream as programmed
        SETB(p_cis->cis_info, LLD_CIS_INFO_PROG, 1);
    }

    // Program RX and TX data
    lld_cis_data_tx_prog(p_cig, p_cis);
    lld_cis_data_rx_prog(p_cis);

    DBG_SWDIAG(CI, PROG, 0);
}

/**
 ****************************************************************************************
 * @brief Handle end of CIG event.
 *
 * @param[in] p_cig      Pointer to the CIG structure
 *
 * @return true if CIG event can be rescheduled, else false
 ****************************************************************************************
 */
__STATIC bool lld_cig_end_evt(struct lld_cig_env *p_cig)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIG_END_EVT_FUNC_BIT, bool, p_cig);

    // Indicate if CIG has been stopped
    bool cig_continue = true;
    // Counter
    uint8_t cnt;
    // Scheduling scheme update is required
    bool upd_sched = false;
    // Indicate if CIG event has been programmed or fully rejected
    bool was_prog = GETB(p_cig->cig_info, LLD_CIG_EVT_ONGOING);

    // Update CIG anchor point parameters
    p_cig->anchor_ts = CLK_ADD_2(p_cig->anchor_ts, p_cig->iso_interval);
    p_cig->cig_evt_cnt++;

    // Indicate no CIG event ongoing
    SETB(p_cig->cig_info, LLD_CIG_EVT_ONGOING, 0);

    // Parse the CIG list for any CIS that has timed out
    {
        // Get first CIS information
        struct lld_cis_env *p_cis = (struct lld_cis_env *)co_list_pick(&p_cig->list_cis);

        while (p_cis)
        {
            // If the CIS has already been established
            if (GETB(p_cis->cis_info, LLD_CIS_INFO_ESTAB))
            {
                uint32_t sup_to = lld_con_sup_to_get(p_cis->con_link_id);

                // Check if the link supervision timeout has been exceeded
                if ((sup_to > 0) && (CLK_SUB(p_cig->anchor_ts, p_cis->last_crc_ok_ts) >= sup_to))
                {
                    p_cis->stop_reason = CO_ERROR_CON_TIMEOUT;
                    p_cig->stop_activity_ids |= CO_BIT(p_cis->act_id);
                }
            }
            else // The CIS has not been established yet
            {
                // If the CIS has not been established within 6 intervals it is considered lost
                if (BLE_UTIL_EVT_CNT_DIFF(p_cig->cig_evt_cnt, p_cis->sup_to_start_cig_evt_cnt) >= LLD_CI_EST_SUP_TO)
                {
                    p_cis->stop_reason = CO_ERROR_CONN_FAILED_TO_BE_EST;
                    p_cig->stop_activity_ids |= CO_BIT(p_cis->act_id);
                }
            }

            // Get next CIS
            p_cis = (struct lld_cis_env *)(p_cis->hdr.next);
        }
    }

    // Check if at least one CIS has to be stopped
    if (p_cig->stop_activity_ids)
    {
        for (cnt = 0; (cnt < BLE_ACTIVITY_MAX) && p_cig->stop_activity_ids; cnt++)
        {
            if (p_cig->stop_activity_ids & CO_BIT(cnt))
            {
                lld_cis_remove(p_cig, LLD_CIS_GET(cnt));

                // Clear bit
                p_cig->stop_activity_ids &= ~CO_BIT(cnt);
            }
        }

        // Update scheduling information, if possible, else will be done once CIG event is done
        if (p_cig->nb_cis != 0)
        {
            upd_sched = true;
        }
        else
        {
            // Free the CIG structure
            lld_cig_free(p_cig);

            cig_continue = false;
        }
    }

    if (cig_continue)
    {
        // Get first CIS information
        struct lld_cis_env *p_cis = (struct lld_cis_env *)co_list_pick(&p_cig->list_cis);

        // compute next group Bluetooth timestamp
        CO_VAL_INC(p_cig->bts_idx, p_cig->bts_size);
        p_cig->ref_anchor_bts_array[p_cig->bts_idx] = rwip_bt_time_to_bts(p_cig->anchor_ts, p_cig->anchor_bit_off);

        while (p_cis)
        {
            // Check if CIS has been started
            if (GETB(p_cis->cis_info, LLD_CIS_INFO_STARTED))
            {
                // -->> Perform operation for end of CIS event

                // Reset CIS information
                SETB(p_cis->cis_info, LLD_CIS_INFO_SKIP,  false);
                SETB(p_cis->cis_info, LLD_CIS_INFO_PROG,  false);
                SETB(p_cis->cis_info, LLD_CIS_INFO_TXACK, false);

                p_cis->evt_inc++;
                p_cis->sub_evt_cnt = 0;
                p_cis->nb_prog     = 0;

                // Prepare data transmissions and receptions for the next event
                lld_cis_data_prepare(p_cig, p_cis);

                // Toggle the hopping scheme buffer
                TOGB(p_cis->cis_info, LLD_CIS_INFO_HOP_TOG);

                if (!was_prog)
                {
                    // Restart computation of hopping for next event
                    lld_ci_compute_hop_scheme(p_cis, p_cis->hop_ptr[GETB(p_cis->cis_info, LLD_CIS_INFO_HOP_TOG)], p_cis->cis_evt_cnt + p_cis->evt_inc);
                }
            }
            else
            {
                // Check if CIS can be started
                if (p_cig->cig_evt_cnt == p_cis->start_cig_evt_cnt)
                {
                    // Start the CIS
                    SETB(p_cis->cis_info, LLD_CIS_INFO_STARTED, 1);
                    // prepare data
                    lld_cis_data_prepare(p_cig, p_cis);

                    // Update number of sub events for the CIG
                    p_cig->cig_nse += p_cis->hop_inf.nse;

                    // Update of scheduling is required
                    upd_sched = true;
                }
            }

            // Get next CIS
            p_cis = (struct lld_cis_env *)(p_cis->hdr.next);
        }

        if (upd_sched)
        {
            lld_cig_compute_sched(p_cig);
        }

        // Point on first scheduling element
        p_cig->ref_se_idx     = 0;
        p_cig->prog_se_idx    = 0;
        p_cig->in_proc_se_idx = 0;
    }

    return (cig_continue);
}

/**
 ****************************************************************************************
 * @brief Handle end of scheduling activity.
 *
 * @param[in] p_ref_se      Pointer to the Reference pack information
 * @param[in] p_cig         Pointer to the CIG structure
 *
 * @return true if CIG is stopped, false if CIG is still active
 ****************************************************************************************
 */
__STATIC bool lld_ci_end_sched_act(struct lld_ci_se *p_ref_se, struct lld_cig_env *p_cig)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CI_END_SCHED_ACT_FUNC_BIT, bool, p_ref_se, p_cig);

    // Get activity arbitration element
    struct sch_arb_elt_tag *p_evt = &p_cig->evt;
    bool resched = true;

    // Clear counters
    p_cig->in_proc_se_idx = 0;
    p_cig->prog_se_idx = 0;
    ASSERT_INFO(p_cig->nb_prog == 0, p_cig->nb_prog, 0);

    // Indicate no sub-event is programmed
    SETB(p_cig->cig_info, LLD_CIG_INFO_PROG, 0);

    // Remove activity arbitration element
    sch_arb_remove(p_evt, true);

#if (BLE_PERIPHERAL)
    // If slave, apply slave offset on anchor time.
    if (GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE) &&
            GETB(p_cig->cig_info, LLD_CIG_INFO_SYNC))
    {
        uint32_t ref_anchor_pt;

        // CIG anchor timestamp - half microseconds part
        int32_t cig_anchor_bit_off = p_cig->anchor_bit_off + p_cig->slave_offset;

        // Update CIG anchor position
        if (cig_anchor_bit_off < 0)
        {
            uint8_t num_hs = CO_DIVIDE_CEIL((-cig_anchor_bit_off), HALF_SLOT_SIZE);
            cig_anchor_bit_off += num_hs * HALF_SLOT_SIZE;
            p_cig->anchor_ts    = CLK_SUB(p_cig->anchor_ts, num_hs);
        }
        else if (cig_anchor_bit_off >= HALF_SLOT_SIZE)
        {
            uint8_t num_hs = cig_anchor_bit_off / HALF_SLOT_SIZE;
            cig_anchor_bit_off -= num_hs * HALF_SLOT_SIZE;
            p_cig->anchor_ts    = CLK_ADD_2(p_cig->anchor_ts, num_hs);
        }

        p_cig->anchor_bit_off  = cig_anchor_bit_off;
        p_cig->sync_drift_acc += p_cig->slave_offset;

        // Clear sync bit
        SETB(p_cig->cig_info, LLD_CIG_INFO_SYNC, 0);
        SETB(p_cig->cig_info, LLD_CIG_PROG_SYNC, 0);

        // Update ACL connection
        lld_con_sync_time_update(p_cig->con_link_id, p_cig->last_sync_ts, p_cig->sync_drift_acc);

        // Convert to a timestamp on Bluetooth Timestamp in us
        ref_anchor_pt = rwip_bt_time_to_bts(p_cig->anchor_ts, p_cig->anchor_bit_off);

        // compute next group Bluetooth timestamp
        p_cig->ref_anchor_bts_array[p_cig->bts_idx] = ref_anchor_pt;

        // Check if data path synchronization is enabled
        if (p_cig->cb_peer_sync)
        {
            p_cig->cb_peer_sync(p_cig->cig_evt_cnt, ref_anchor_pt);
        }
    }
#endif //(BLE_PERIPHERAL)

    // Move to the next scheduling structure
    p_cig->ref_se_idx = p_cig->ref_se_idx + p_ref_se->nb_packed;

    // Prepare next air time to reserve in Scheduling arbiter.
    while (p_cig->ref_se_idx < p_cig->cig_nse)
    {
        // Counter
        uint8_t cnt;
        // Update Sub-Event reference
        p_ref_se = &(p_cig->p_se_infos[p_cig->ref_se_idx]);

        // Check if at least one sub-event has to be programmed
        for (cnt = p_cig->ref_se_idx; cnt < (p_cig->ref_se_idx + p_ref_se->nb_packed); cnt++)
        {
            // Get activity id
            uint8_t cis_act_id = p_cig->p_se_infos[cnt].cis_act_id;
            // Get CIS structure
            struct lld_cis_env *p_cis = LLD_CIS_GET(cis_act_id);

            if (!GETB(p_cis->cis_info, LLD_CIS_INFO_SKIP))
            {
                // Can schedule this sub-event
                break;
            }
        }

        // Check if at least one sub event has to be programmed
        if (cnt < (p_cig->ref_se_idx + p_ref_se->nb_packed))
        {
            // Skip sub-events
            p_cig->in_proc_se_idx = cnt;
            p_cig->prog_se_idx    = cnt;
            break;
        }

        // Update Reference structure
        p_cig->ref_se_idx = p_cig->ref_se_idx + p_ref_se->nb_packed;
    }

    // Check if CIG event is done
    if (p_cig->ref_se_idx == p_cig->cig_nse)
    {
        // Handle end of CIG event
        resched = lld_cig_end_evt(p_cig);
    }

    if (resched)
    {
        // Schedule the next event block
        resched = lld_ci_sched(p_cig);
    }

    return !resched;
}

/**
 ****************************************************************************************
 * @brief Handle end of CI subevent.
 *
 * @param [in] p_cis      Pointer to the CIS structure
 * @param [in] p_cig      Pointer to the CIG structure
 * @param [in] rejected   Indicate if subevent has been rejected during insertion in arbitration
 *
 * @return true if CIG is stopped, false if CIG is still active
 ****************************************************************************************
 */
__STATIC bool lld_ci_end_subevt(struct lld_cis_env *p_cis, struct lld_cig_env *p_cig, bool rejected)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_END_SUBEVT_2_FUNC_BIT, bool, p_cis, p_cig, rejected);

    // Get Reference Sub-event structure
    struct lld_ci_se *p_ref_se = &(p_cig->p_se_infos[p_cig->ref_se_idx]);
    // And In process structure for the sub event
    struct lld_ci_se *p_cur_se = &(p_cig->p_se_infos[p_cig->in_proc_se_idx]);
    bool cig_stop = false;

    DBG_SWDIAG(CI, END, 1);

    // Update programation information
    if (GETB(p_cur_se->state, LLD_CI_SE_STATUS_PROG))
    {
        p_cig->nb_prog--;
        p_cis->nb_prog--;
        // Mark the sub event has not programmed anymore
        SETB(p_cur_se->state, LLD_CI_SE_STATUS_PROG, false);
    }

    // If event has not been started by HW, perform operation not done by HW
    if (rejected)
    {
        // Get CS index for this sub-event
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(p_cur_se->cis_act_id);
        // Retrieve sub-event counter and flush counter
        uint8_t sub_evt_cnt = em_ble_isoevtcntl_subevtcnt_getf(cs_idx);
        uint8_t flush_cnt   = em_ble_isoevtcntl_flushcnt_getf(cs_idx);
        // Increment them
        sub_evt_cnt++;
        flush_cnt++;
        // Set the values in the CS
        em_ble_isoevtcntl_pack(cs_idx, flush_cnt, sub_evt_cnt);

        lld_cis_data_skip_subevt(p_cig, p_cis);

        // CS will have to be updated by SW
        SETB(p_cis->cis_info, LLD_CIS_INFO_UPDATE_CS, 1);
        SETB(p_cur_se->state, LLD_CI_SE_STATUS_SKIP, 0);
    }

    // Increment flush counter
    p_cis->flush_cnt++;

    // Increment sub-event counter
    CO_VAL_INC(p_cis->sub_evt_cnt, p_cis->hop_inf.nse);

    if (!GETB(p_cis->cis_info, LLD_CIS_INFO_SKIP))
    {
        // Check if all sub-events for this CIS have been processed
        if (p_cis->sub_evt_cnt != 0)
        {
            struct lld_ci_data *p_tx = p_cis->p_tx;
            struct lld_ci_data *p_rx = p_cis->p_rx;

            // Check if next sub-events for this CIS need to be scheduled, if it has not already
            // been decide to skip them
            bool skip = (p_tx->data_prep_nb == 0) && (p_rx->data_prep_nb == 0);

            // For a master, force scheduling of a sub-event to send acknowledgment to slave
            if ((!GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE)) && (p_rx->max_pdu > 0)
                    && ((p_rx->data_prep_nb == 0)) && !GETB(p_cis->cis_info, LLD_CIS_INFO_TXACK))
            {
                // mark that TX ack ready to be programmed
                SETB(p_cis->cis_info, LLD_CIS_INFO_TXACK, true);

                // if CIS sub-event already programmed, following events can be skipped
                skip = skip && (p_cis->nb_prog > 0);
            }

            SETB(p_cis->cis_info, LLD_CIS_INFO_SKIP, skip);
        }
    }

    // Move on next subevent to be handled
    p_cig->in_proc_se_idx++;

    // Check if all subevents have been processed
    if (p_cig->in_proc_se_idx == (p_cig->ref_se_idx + p_ref_se->nb_packed))
    {
        // Scheduling event is done, prepare next one
        cig_stop = lld_ci_end_sched_act(p_ref_se, p_cig);
    }

    DBG_SWDIAG(CI, END, 0);

    return cig_stop;
}

/**
 ****************************************************************************************
 * @brief Check the reception during CIS activity
 *
 * @brief p_cig       Pointer to the CIG structure
 * @brief p_cis       Pointer to the CIS structure
 * @brief timestamp   EOF interruption timestamp
 ****************************************************************************************
 */
__STATIC void lld_cis_rx(struct lld_cig_env *p_cig, struct lld_cis_env *p_cis, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_RX_FUNC_BIT, p_cig, p_cis, timestamp);

    // Check if a packet has been received (RX Done bit set to 1)
    if (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(p_cis->act_id)))
    {
        // Get index of RX descriptor
        uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;
        // Retrieve RX status
        uint16_t rxstatcis = em_ble_rxstatcis_get(rxdesc_idx);
        uint16_t rxphcis   = em_ble_rxphcis_get(rxdesc_idx);
        uint16_t rxchass = em_ble_rxchass_get(rxdesc_idx);
        uint8_t  rx_status = RWIP_RX_OTHER_ERROR;
#if BLE_PWR_CTRL
        uint8_t rx_rate = (rxchass & EM_BLE_RATE_MASK) >> EM_BLE_RATE_LSB;
#endif // BLE_PWR_CTRL

        // Trace the current RX descriptor
        TRC_REQ_RX_DESC(LLD_CON, p_cis->act_id, REG_EM_BLE_RX_DESC_ADDR_GET(rxdesc_idx));

        do
        {
            uint32_t base_cnt;

            // Check synchronization status
            if (rxstatcis & EM_BLE_SYNC_ERR_BIT)
            {
                break;
            }

            // Read clock value where sync has been found
            base_cnt = (em_ble_rxclknsync1_clknrxsync1_getf(rxdesc_idx) << 16)
                       | em_ble_rxclknsync0_clknrxsync0_getf(rxdesc_idx);

#if BLE_PWR_CTRL
            // Save rx rate - required to distinguish S2/S8
            p_cis->rx_rate = rx_rate;
#endif // BLE_PWR_CTRL

#if (BLE_PERIPHERAL)
            // Check if device is slave
            if (GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE))
            {
                // Get information of this subevent
                struct lld_ci_se *p_se = &p_cig->p_se_infos[p_cig->in_proc_se_idx];

                // *** Synchronize slave after correct reception
                // Note: this algorithm supports large sync windows but not multi-attempts

                // Read bit position where sync has been found
                uint16_t fine_cnt = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(rxdesc_idx);
                // Compute new bit offset (remove synchronization word duration)
                int32_t new_bit_off = fine_cnt - US_TO_HUS(lld_exp_sync_pos_tab[p_cis->rx_rate]);

                // Keep in mind that synchronization has been caught
                SETB(p_cig->cig_info, LLD_CIG_INFO_SYNC, 1);

                // Adjust into a half slot range
                if (new_bit_off < 0)
                {
                    uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
                    new_bit_off += num_hs * HALF_SLOT_SIZE;
                    base_cnt = CLK_SUB(base_cnt, num_hs);
                }

                // Compute difference with expected synchronization
                p_cig->slave_offset = (CLK_DIFF(p_se->exp_sync_ts, base_cnt) * HALF_SLOT_SIZE) + (new_bit_off - p_se->exp_sync_bit_off);

                // Save the values corresponding to the last detected sync
                p_cig->last_sync_ts = base_cnt;
            }
#endif //(BLE_PERIPHERAL)

            // Check packet status (CRC error)
            if (rxstatcis & (EM_BLE_CRC_ERR_BIT))
            {
                rx_status = RWIP_RX_CRC_ERROR;
                break;
            }

            // Save the timestamp of the last correct CRC packet for this CIS
            p_cis->last_crc_ok_ts = base_cnt;

            // If the CIS has not been established yet
            if (!GETB(p_cis->cis_info, LLD_CIS_INFO_ESTAB))
            {
                SETB(p_cis->cis_info, LLD_CIS_INFO_ESTAB, true);

                // Inform link layer controller that the CIS has been established
                struct lld_cis_estab_ind *p_msg = KE_MSG_ALLOC(LLD_CIS_ESTAB_IND, TASK_LLI, TASK_NONE, lld_cis_estab_ind);
                p_msg->act_id = p_cis->act_id;
                p_msg->status = CO_ERROR_NO_ERROR;
                ke_msg_send(p_msg);
            }

            // Consider channel as good
            rx_status = RWIP_RX_OK;

            // Check packet status
            if (rxstatcis & (EM_BLE_RXTIME_ERR_BIT | EM_BLE_LEN_ERR_BIT | EM_BLE_LLID_ERR_BIT |  EM_BLE_SN_ERR_BIT))
                break;

            // Check if CIE bit is received from peer
            if (rxphcis & EM_BLE_RXCIE_BIT)
            {
                // Skip next subevents for this CIS
                SETB(p_cis->cis_info, LLD_CIS_INFO_SKIP, 1);
            }

            if (rxstatcis & EM_BLE_MIC_ERR_BIT)
            {
                // Keep in mind that a MIC error has been detected, CIS will be stopped
                // as soon as possible
                SETB(p_cis->cis_info, LLD_CIS_INFO_MIC_FAIL, true);
                // save stop reason
                p_cis->stop_reason = CO_ERROR_TERMINATED_MIC_FAILURE;
                // Keep also the activity id for the CIS to be stopped
                p_cig->stop_activity_ids |= CO_BIT(p_cis->act_id);

                break;
            }
        }
        while (0);

        // Process RSSI information
        {
            // Get channel information from the exchange memory
            uint16_t rxchass   = em_ble_rxchass_get(lld_env.curr_rxdesc_index);
            uint8_t  rxrssi    = GETF(rxchass, EM_BLE_RSSI);
            uint8_t  usedchidx = GETF(rxchass, EM_BLE_USED_CH_IDX);
            int8_t   rssi      = rwip_rf.rssi_convert(rxrssi) - lld_env.rx_path_comp / 10;

            p_cis->last_rssi = rssi;

            rwip_channel_assess_ble(usedchidx, rx_status, rxrssi, timestamp, true);

#if BLE_PWR_CTRL
            if (rx_status == RWIP_RX_OK)
            {
                // Update connection for power control
                lld_con_rssi_update(p_cis->con_link_id, p_cis->rx_rate, rssi);
            }
#endif //BLE_PWR_CTRL
        }


        // Free RX descriptor
        lld_rxdesc_free();
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_ci_evt_start_cbk(struct sch_arb_elt_tag *p_evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_EVT_START_CBK_2_FUNC_BIT, p_evt);

    DBG_SWDIAG(CI, START, 1);

    if (p_evt != NULL)
    {
        // Point to CIG parameters
        struct lld_cig_env  *p_cig    = (struct lld_cig_env *)p_evt;
        // Get reference sub-event structure to be programmed
        struct lld_ci_se    *p_ref_se = &(p_cig->p_se_infos[p_cig->ref_se_idx]);
        // Get pointer to information for the first sub-event to be scheduled
        struct lld_ci_se    *p_first_se = &(p_cig->p_se_infos[p_cig->in_proc_se_idx]);

        // maximum SE index of the packed scheduled group
        uint8_t max_se_idx = p_cig->ref_se_idx + p_ref_se->nb_packed;

        ASSERT_ERR(p_ref_se != NULL);

        // Check if local synchronization is enabled and a new CIG event starts
        if ((p_cig->cb_local_sync != NULL) && !GETB(p_cig->cig_info, LLD_CIG_EVT_ONGOING))
        {
            // Sample ISO counter and trigger GPIO0
            ble_isocntcntl_isosamp_setf(1);
            while (ble_isocntcntl_isosamp_getf());

            // Provide the associated timestamp via the callback
            p_cig->cb_local_sync(ble_isocntsamp_get());
        }

        // Clear WAIT bit
        SETB(p_cig->cig_info, LLD_CIG_INFO_WAIT, 0);

        // Mark that programming is enabled
        SETB(p_cig->cig_info, LLD_CIG_PROG_EN, 1);

        // Fill the exchange table
        lld_cis_prog(p_cig, p_evt->current_prio);

        // Increment sub-event program counter
        p_cig->prog_se_idx += 1;

        // Keep in mind that CIG event is in progress
        SETB(p_cig->cig_info, LLD_CIG_INFO_PROG, 1);

        // Program a timer for second sub-event
        if (p_cig->prog_se_idx < max_se_idx)
        {
            struct lld_ci_se *p_next_se = &p_cig->p_se_infos[p_cig->prog_se_idx];

            // Get pointer to alarm
            struct sch_alarm_tag *p_alarm = &p_cig->alarm;

            int32_t bit_off = p_first_se->bit_offset + (p_next_se->offset - p_first_se->offset);
            uint32_t target_clock = CLK_ADD_2(p_first_se->timestamp, bit_off / HALF_SLOT_SIZE);
            bit_off = CO_MOD(bit_off, HALF_SLOT_SIZE);

            p_next_se->timestamp = target_clock;
            p_next_se->bit_offset = bit_off;

#if (BLE_PERIPHERAL)
            // Also set expected synchronization time for next sub events
            if (GETB(p_cig->cig_info, LLD_CIG_INFO_SLAVE))
            {
                int32_t exp_sync_bit_off = p_first_se->exp_sync_bit_off + (p_next_se->offset - p_first_se->offset);
                p_next_se->exp_sync_ts = CLK_ADD_2(p_first_se->exp_sync_ts, exp_sync_bit_off / HALF_SLOT_SIZE);
                exp_sync_bit_off = CO_MOD(exp_sync_bit_off, HALF_SLOT_SIZE);
                p_next_se->exp_sync_bit_off = exp_sync_bit_off;
            }
#endif //(BLE_PERIPHERAL)

            ASSERT_ERR(GETB(p_cig->cig_info, LLD_CIG_INFO_ALARM) == 0);

            // Program alarm in advance of targeted sub-event
            p_alarm->time.hs = CLK_SUB(p_next_se->timestamp, rwip_prog_delay);
            p_alarm->time.hus = p_next_se->bit_offset;

            // Keep in mind that alarm timer is running
            SETB(p_cig->cig_info, LLD_CIG_INFO_ALARM, 1);
            SETB(p_cig->cig_info, LLD_CIG_ALARM_PROG, 1);

            // Set the alarm
            sch_alarm_set(p_alarm);
        }

        // New event of a CIG
        if (!GETB(p_cig->cig_info, LLD_CIG_EVT_ONGOING))
        {
            // Get first CIS information
            struct lld_cis_env *p_cis = (struct lld_cis_env *)co_list_pick(&p_cig->list_cis);

            // Fill array of sub-event information
            while (p_cis)
            {
                // Ensure that CIS is started
                if (GETB(p_cis->cis_info, LLD_CIS_INFO_STARTED))
                {
                    // Request computation of hopping for next event
                    lld_ci_compute_hop_scheme(p_cis, p_cis->hop_ptr[!GETB(p_cis->cis_info, LLD_CIS_INFO_HOP_TOG)], p_cis->cis_evt_cnt + p_cis->evt_inc + 1);
                }

#if BLE_PWR_CTRL
                {
                    uint8_t pwr_lvl;
                    // Fetch transmit power level
                    if (CO_ERROR_NO_ERROR == lld_con_tx_pwr_lvl_get(p_cis->con_link_id, p_cis->tx_rate, &pwr_lvl))
                    {
                        // Configure transmit power
                        em_ble_txrxcntl_txpwr_setf(EM_BLE_CS_ACT_ID_TO_INDEX(p_cis->act_id), pwr_lvl);
                    }
                }
#endif // BLE_PWR_CTRL

                // Get next CIS
                p_cis = (struct lld_cis_env *)(p_cis->hdr.next);
            }

            // Indicate CIG event is ongoing
            SETB(p_cig->cig_info, LLD_CIG_EVT_ONGOING, 1);
        }
    }

    DBG_SWDIAG(CI, START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event stop notification
 ****************************************************************************************
 */
__STATIC void lld_ci_evt_stop_cbk(struct sch_arb_elt_tag *p_elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_EVT_STOP_CBK_2_FUNC_BIT, p_elt);

    DBG_SWDIAG(CI, START, 1);

    if (p_elt != NULL)
    {
        // Point to CIG parameters
        struct lld_cig_env  *p_cig    = (struct lld_cig_env *)p_elt;

        // Mark that programming is disabled
        SETB(p_cig->cig_info, LLD_CIG_PROG_EN, 0);
    }

    DBG_SWDIAG(CI, START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_ci_evt_canceled_cbk(struct sch_arb_elt_tag *p_evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_EVT_CANCELED_CBK_2_FUNC_BIT, p_evt);

    DBG_SWDIAG(CI, CANCEL, 1);

    if (p_evt != NULL)
    {
        // Point to CIG parameters
        struct lld_cig_env  *p_cig = (struct lld_cig_env *)p_evt;
        // Get currently use scheduling structure
        struct lld_ci_se    *p_ref_se = &(p_cig->p_se_infos[p_cig->ref_se_idx]);
        // Get pointer to alarm
        struct sch_alarm_tag *p_alarm = &p_cig->alarm;

        // Increment priority
        p_evt->current_prio = RWIP_PRIO_ADD_2(p_evt->current_prio, RWIP_PRIO_INC(p_cig->prio_idx));

        // Mark that scheduling has been rejected
        SETB(p_cig->cig_info, LLD_CIG_SCHED_REJECT, 1);

        ASSERT_ERR(GETB(p_cig->cig_info, LLD_CIG_INFO_ALARM) == 0);

        // Program alarm to handle scheduling rejected
        p_alarm->time.hs = p_ref_se->timestamp;
        p_alarm->time.hus = 0;

        // Keep in mind that alarm timer is running
        SETB(p_cig->cig_info, LLD_CIG_INFO_ALARM, 1);

        // Set the alarm
        sch_alarm_set(p_alarm);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(CI, CANCEL, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt (after end of each programmed sub event)
 ****************************************************************************************
 */
__STATIC void lld_ci_frm_isr(struct lld_cis_env *p_cis, struct lld_cig_env *p_cig, uint8_t sub_evt_idx,
                             uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_FRM_ISR_2_FUNC_BIT, 5, p_cis, p_cig, sub_evt_idx, timestamp, abort);

    // And arbitration structure
    struct sch_arb_elt_tag *p_evt = &p_cig->evt;
    bool cig_stop = false;

    ASSERT_INFO(p_cig->in_proc_se_idx == sub_evt_idx, p_cig->in_proc_se_idx, sub_evt_idx);

    // Reset priority
    if (!abort)
    {
        p_evt->current_prio = rwip_priority[p_cig->prio_idx].value;
    }

    // Check reception status for this sub-event
    lld_cis_rx(p_cig, p_cis, timestamp);

    // Handle end of sub event
    cig_stop = lld_ci_end_subevt(p_cis, p_cig, false);

    // handled skipped sub-events
    while (!cig_stop && (p_cig->in_proc_se_idx < p_cig->prog_se_idx))
    {
        struct lld_ci_se *p_skip_se = &(p_cig->p_se_infos[p_cig->in_proc_se_idx]);

        // event has been skipped, handle it as end of sub-event
        if (GETB(p_skip_se->state, LLD_CI_SE_STATUS_SKIP))
        {
            p_cis = LLD_CIS_GET(p_skip_se->cis_act_id);
            cig_stop = lld_ci_end_subevt(p_cis, p_cig, true);
        }
        // update index to next pointer programmed
        else if (!GETB(p_skip_se->state, LLD_CI_SE_STATUS_PROG))
        {
            p_cig->in_proc_se_idx++;
        }
        else
        {
            break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Handle skip interrupt
 * NOTE: It is considered here that an CIS sub-event can only be skipped due to collision
 * with a non-ISO event (ADV, SCAN, ...) with an unknown duration.
 * Hence the end interrupt following the skip interrupt will not be for the skipped CIS
 * or the CIG the skipped CIS belongs to.
 ****************************************************************************************
 */
__STATIC void lld_ci_skip_isr(struct lld_cis_env *p_cis, struct lld_cig_env *p_cig, uint8_t sub_evt_idx,
                              uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_SKIP_ISR_2_FUNC_BIT, p_cis, p_cig, sub_evt_idx, timestamp);

    DBG_SWDIAG(CI, SKIP, 1);

    // Also get arbitration structure
    struct sch_arb_elt_tag *p_evt = &p_cig->evt;
    // Get information about skipped sub-event
    struct lld_ci_se *p_se = &p_cig->p_se_infos[sub_evt_idx];

    // Increment priority
    p_evt->current_prio = RWIP_PRIO_ADD_2(p_evt->current_prio, RWIP_PRIO_INC(p_cig->prio_idx));

    // Check if the skipped sub-event is the next one or the one after
    if (sub_evt_idx == p_cig->in_proc_se_idx)
    {
        // End the sub-event as if it had been handled by the hardware
        lld_ci_end_subevt(p_cis, p_cig, true);
    }
    else
    {
        // Mark sub-event as skipped
        SETB(p_se->state, LLD_CI_SE_STATUS_SKIP, true);
    }

    DBG_SWDIAG(CI, SKIP, 0);
}

/**
 ****************************************************************************************
 * @brief Handle TX ISO interrupt
 ****************************************************************************************
 */
__STATIC void lld_ci_tx_iso_isr(struct lld_cis_env *p_cis, struct lld_cig_env *p_cig, uint8_t sub_evt_idx,
                                uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_TX_ISO_ISR_2_FUNC_BIT, p_cis, p_cig, sub_evt_idx, timestamp);

    struct lld_ci_data *p_tx = p_cis->p_tx;
    DBG_SWDIAG(CI, TX_ISO, 1);

    if (p_tx->max_pdu > 0)
    {
        // Extract first element from list of TX data structures
        struct lld_ci_data_prog_info *p_data = &(p_tx->data_queue[p_tx->data_idx]);
        ASSERT_INFO(p_data->prog, p_tx->data_idx, p_tx->data_prog_nb);

        // Increment current data index : Point on next data item
        CO_VAL_INC(p_tx->data_idx, p_tx->data_queue_size);
        // Update index of next TX ISO descriptor to be used
        CO_VAL_INC(p_tx->curr_iso_idx, BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS);

        // decrement number of element prepared and programmed
        p_tx->data_prep_nb--;
        p_tx->data_prog_nb--;

        p_data->prog = false;
        // Program next data structure
        lld_cis_data_tx_prog(p_cig, p_cis);
    }

    DBG_SWDIAG(CI, TX_ISO, 0);
}

/**
 ****************************************************************************************
 * @brief Handle RX ISO interrupt
 ****************************************************************************************
 */
__STATIC void lld_ci_rx_iso_isr(struct lld_cis_env *p_cis, struct lld_cig_env *p_cig, uint8_t sub_evt_idx,
                                uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_RX_ISO_ISR_2_FUNC_BIT, p_cis, p_cig, sub_evt_idx, timestamp);

    struct lld_ci_data *p_rx = p_cis->p_rx;

    DBG_SWDIAG(CI, RX_ISO, 1);

    if (p_rx->max_pdu > 0)
    {
        uint8_t  bts_idx = p_cig->bts_idx;

        // Extract first element from list of RX data structures
        struct lld_ci_data_prog_info *p_data = &(p_rx->data_queue[p_rx->data_idx]);
        ASSERT_INFO(p_data->prog, p_rx->data_idx, p_rx->data_prog_nb);

        // Compute payload counter for this packet
        CO_VAL_SUB(bts_idx, (p_rx->data_prep_nb - 1) / p_rx->bn, p_cig->bts_size);

        // Inform the ISOAL module that buffer is done and data can be used
        lld_isoal_rx_done(p_cis->act_id, p_data->buf_idx, p_cig->ref_anchor_bts_array[bts_idx], false);

        // Increment current data index : Point on next data item
        CO_VAL_INC(p_rx->data_idx, p_rx->data_queue_size);
        // Update index of next RX ISO descriptor to be used
        CO_VAL_INC(p_rx->curr_iso_idx, BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS);
        // decrement number of element prepared and programmed
        p_rx->data_prep_nb--;
        p_rx->data_prog_nb--;

        p_data->prog = false;
        // Program next data structure
        lld_cis_data_rx_prog(p_cis);
    }

    DBG_SWDIAG(CI, RX_ISO, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_ci_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_FRM_CBK_2_FUNC_BIT, timestamp, dummy, irq_type);

    // Get CIS structure
    struct lld_cis_env *p_cis       = LLD_CIS_GET((uint8_t)GETF(dummy, LLD_CI_DUMMY_ACT_ID));
    // Get CIG structure
    struct lld_cig_env *p_cig       = LLD_CIG_GET((uint8_t)GETF(dummy, LLD_CI_DUMMY_GRP_HDL));
    // Extract subevent index
    uint8_t             sub_evt_idx = (uint8_t)GETF(dummy, LLD_CI_DUMMY_SUB_EVT_IDX);

    if (p_cis && p_cig)
    {
        // Choose the appropriate handler based on the type of interruption
        switch (irq_type)
        {
        case (SCH_FRAME_IRQ_EOF):
        {
            lld_ci_frm_isr(p_cis, p_cig, sub_evt_idx, timestamp, false);
        }
        break;
        case (SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO):
        case (SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO):
        {
            lld_ci_frm_isr(p_cis, p_cig, sub_evt_idx, timestamp, true);
        }
        break;

        case (SCH_FRAME_IRQ_RX_ISO):
        {
            lld_ci_rx_iso_isr(p_cis, p_cig, sub_evt_idx, timestamp);
        }
        break;

        case (SCH_FRAME_IRQ_TX_ISO):
        {
            lld_ci_tx_iso_isr(p_cis, p_cig, sub_evt_idx, timestamp);
        }
        break;

        case (SCH_FRAME_IRQ_SKIP):
        {
            lld_ci_skip_isr(p_cis, p_cig, sub_evt_idx, timestamp);
        }
        break;

        default:
        {
            ASSERT_INFO_FORCE(0, dummy, irq_type);
        }
        break;
        }
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */


uint8_t lld_cis_start(uint8_t act_id, struct lld_cis_params *p_cis_params,
                      uint32_t *p_cis_start_hs, uint16_t *p_cis_start_hus)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_START_FUNC_BIT, uint8_t, act_id, p_cis_params, p_cis_start_hs, p_cis_start_hus);

    // Returned status
    uint8_t status;
    uint32_t cis_anchor_ts = 0;
    uint32_t cis_anchor_bit_off = 0;

    GLOBAL_INT_DISABLE();

    do
    {
        // CS Index
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);
        // Get CIG structure for provided CIG handle
        struct lld_cig_env *p_cig = LLD_CIG_GET(p_cis_params->grp_hdl);
        // CIS structure
        struct lld_cis_env *p_cis;
        // Connection information
        struct lld_con_info_for_cis con_info;
        // Counter
        uint8_t cnt;
        uint8_t pkt_len;
        struct lld_ci_data *p_tx;
        struct lld_ci_data *p_rx;
        uint32_t iso_interval_us;

        // Get connection information
        status = lld_con_info_for_cis_get(p_cis_params->con_link_id,
                                          p_cis_params->con_evt_cnt, &con_info);

        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // ====================================================================
        // CIG INITIALIZATION (if needed)
        // ====================================================================

        // Check if a CIG with same CIG handle has already been started
        if (!p_cig)
        {
            // Allocate a new CIG structure
            p_cig = lld_cig_alloc(p_cis_params, &con_info);

            if (!p_cig)
            {
                // Not enough memory
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
                break;
            }
        }

        // ====================================================================
        // CIS INITIALIZATION
        // ====================================================================

        // Allocate a new CIS structure
        p_cis = lld_cis_alloc(p_cis_params, &con_info, p_cig, act_id, &cis_anchor_ts, &cis_anchor_bit_off);

        if (!p_cis)
        {
            // Not enough memory, remove CIG if no more used
            if (p_cig->nb_cis == 0)
            {
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
                lld_cig_free(p_cig);
            }

            break;
        }
        // compute iso interval in microseconds
        iso_interval_us = (((uint32_t)p_cig->iso_interval) * HALF_SLOT_SIZE) >> 1 ;

        // Start ISOAL for transmission
        if (p_cis_params->tx_max_sdu > 0)
        {
            status = lld_isoal_start(act_id,
                                     ISO_SEL_TX,
                                     BLE_ACTID_TO_CISHDL(act_id),
                                     p_cis_params->tx_sdu_interval,
                                     p_cis_params->tx_trans_latency,
                                     0,
                                     p_cis_params->tx_max_sdu,
                                     iso_interval_us,
                                     p_cis_params->tx_bn,
                                     p_cis_params->tx_max_pdu,
                                     p_cis_params->framing,
                                     con_info.encrypted,
                                     false);

            if (CO_ERROR_MEMORY_CAPA_EXCEED == status)
            {
                lld_cig_free(p_cig);
                break;
            }
        }

        // Start ISOAL for reception
        if (p_cis_params->rx_max_sdu > 0)
        {
            // Compute synchronization reference offset for the specified mode (Vol 6 Part G Chapter 3.2)
            int32_t sync_ref_offset = 0;

            // Unframed / master
            if ((p_cis_params->framing == ISO_UNFRAMED_MODE) && (p_cis_params->role == MASTER_ROLE))
            {
                // SDU_Synchronization_Reference = CIS reference anchor point + CIS_Sync_Delay - CIG_Sync_Delay - ISO_Interval + SDU interval
                sync_ref_offset = -p_cis_params->cis_spacing - iso_interval_us + p_cis_params->rx_sdu_interval;
            }
            // Unframed / slave
            else if ((p_cis_params->framing == ISO_UNFRAMED_MODE) && (p_cis_params->role == SLAVE_ROLE))
            {
                // SDU_Synchronization_Reference = CIS reference anchor point + CIS_Sync_Delay + (FT_M_To_S - 1) x ISO_Interval
                sync_ref_offset = p_cis_params->cig_sync_delay + ((p_cis_params->rx_ft - 1) * iso_interval_us);
            }
            // Framed / master
            else if ((p_cis_params->framing == ISO_FRAMED_MODE) && (p_cis_params->role == MASTER_ROLE))
            {
                // SDU_Synchronization_Reference = CIS Reference Anchor point + CIS_Sync_Delay - CIG_Sync_Delay - Time_Offset
                sync_ref_offset = -p_cis_params->cis_spacing;
            }
            // Framed / slave
            else if ((p_cis_params->framing == ISO_FRAMED_MODE) && (p_cis_params->role == SLAVE_ROLE))
            {
                // SDU_Synchronization_Reference = CIS Reference Anchor point + CIS_Sync_Delay + SDU_Interval_M_To_S + FT_M_To_S x ISO_Interval - Time_Offset
                sync_ref_offset = p_cis_params->rx_trans_latency - p_cis_params->cis_spacing;
            }

            status = lld_isoal_start(act_id,
                                     ISO_SEL_RX,
                                     BLE_ACTID_TO_CISHDL(act_id),
                                     p_cis_params->rx_sdu_interval,
                                     p_cis_params->rx_trans_latency,
                                     sync_ref_offset,
                                     p_cis_params->rx_max_sdu,
                                     iso_interval_us,
                                     p_cis_params->rx_bn,
                                     p_cis_params->rx_max_pdu,
                                     p_cis_params->framing,
                                     con_info.encrypted,
                                     false);

            if (CO_ERROR_MEMORY_CAPA_EXCEED == status)
            {
                lld_cig_free(p_cig);
                break;
            }
        }

        // ====================================================================
        // CONTROL STRUCTURE INITIALIZATION
        // ====================================================================

        // Set permission/status of CS as R/W but uninitialized
        DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

        // BLE control
        em_ble_cntl_pack(cs_idx, RWIP_COEX_GET(CON, TXBSY),
                         RWIP_COEX_GET(CON, RXBSY),
                         RWIP_COEX_GET(CON, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                         0,
#endif // EAVESDROPPING_SUPPORT
                         ((p_cis_params->role == MASTER_ROLE)
                          ? EM_BLE_CS_FMT_MST_CONNECT : EM_BLE_CS_FMT_SLV_CONNECT));

        // ISO link control
        em_ble_isolinkcntl_pack(cs_idx, /*streamlbl*/  act_id,
                                /*grouplbl*/   p_cis_params->grp_hdl,
                                /*isosyncmode*/ 0,
                                /*isosyncen*/   1,
                                /*isotype*/     LLD_ISO_MODE_CI);

        // Thresholds and Rates
        em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/   10,
                                     /*txthr*/   10,
                                     /*auxrate*/ p_cis_params->tx_rate,
                                     /*rxrate*/  p_cis_params->rx_rate,
                                     /*txrate*/  p_cis_params->tx_rate);

        // Link control
        em_ble_linkcntl_pack(cs_idx, /*hplpmode*/      0,
                             /*linklbl*/       cs_idx,
                             /*sas*/           false,
                             /*nullrxllidflt*/ true,
                             /*micmode*/       ENC_MIC_PRESENT,
                             /*cryptmode*/     ENC_MODE_PKT_PLD_CNT,
                             /*txcrypten*/     con_info.encrypted,
                             /*rxcrypten*/     con_info.encrypted,
                             /*privnpub*/      false);

        // ISO TX/RX
        em_ble_isotxdescptr_set(cs_idx, 0);

#if BLE_PWR_CTRL
        {
            uint8_t con_tx_rate;
            // Get connection tx rate
            if (CO_ERROR_NO_ERROR == lld_con_tx_rate_get(p_cis_params->con_link_id, &con_tx_rate))
            {
                // If the CIS tx rate differs, signal power change indication
                if (p_cis_params->tx_rate != con_tx_rate)
                {
                    struct lld_cis_pwr_change_ind *msg = KE_MSG_ALLOC(LLD_CIS_PWR_CHANGE_IND,
                                                         KE_BUILD_ID(TASK_LLC, p_cis_params->con_link_id),
                                                         TASK_NONE, lld_cis_pwr_change_ind);
                    msg->tx_rate = p_cis_params->tx_rate; // CIS transmit rate
                    msg->en = true; // enabled
                    ke_msg_send(msg);
                }
            }
        }
#endif // BLE_PWR_CTRL

        // Synchronization Word
        em_ble_syncwl_set(cs_idx, p_cis_params->access_addr >> 0);
        em_ble_syncwh_set(cs_idx, p_cis_params->access_addr >> 16);
        // CRC Initialization value
        em_ble_crcinit0_set(cs_idx, (con_info.crcinit.crc[1] << 8) | con_info.crcinit.crc[0]);
        em_ble_crcinit1_pack(cs_idx, /*rxmaxctebuf*/ 0, /*crcinit1*/ con_info.crcinit.crc[2]);

        // Hopping control
        em_ble_hopcntl_pack(cs_idx, /*fhen*/     true,
                            /*hop_mode*/ LLD_HOP_MODE_SEQ_PTR,
                            /*hopint*/   1, // Not used in channel selection 2
                            /*chidx*/    0);

        // TX power control
        em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_max);

        // Set encryption parameters
        if (con_info.encrypted)
        {
            uint8_t reg_idx;
            uint8_t key_idx;

            // Configure Initialization vector in CS
            // CIS_IV[0:63]  =  (CON_IV[0:31] XOR CIS_AA[0:31]) || CON_IV[32:63]
            for (reg_idx = 0, key_idx = 0; reg_idx < EM_BLE_IV_COUNT; reg_idx++)
            {
                uint16_t iv_part = co_read16p(&(con_info.iv.vect[key_idx]));

                // CIS_IV[0:31]  =  CON_IV[0:31] XOR CIS_AA[0:31]
                if (reg_idx < 2)
                {
                    iv_part ^= (uint16_t)(p_cis_params->access_addr >> (16 * reg_idx));
                }

                em_ble_iv_setf(cs_idx, reg_idx, iv_part);
                key_idx += 2;
            }

            // Configure Session key in CS
            for (reg_idx = 0, key_idx = (KEY_LEN - 1); reg_idx < EM_BLE_SK_COUNT; reg_idx++)
            {
                // encrypted_data is LSB first and  SK is MSB first
                em_ble_sk_setf(cs_idx, reg_idx,
                               (con_info.sk.ltk[key_idx - 1] << 8) | con_info.sk.ltk[key_idx]);
                key_idx -= 2;
            }
        }

        // Set max & min event time
        em_ble_maxevtime_set(cs_idx, p_cis->sub_evt_dur);
        em_ble_minevtime_set(cs_idx, p_cis->sub_evt_dur);

        // Set Rx Max buf and Rx Max Time + ISO receive payload length authorized
        pkt_len = p_cis_params->rx_max_pdu + ((con_info.encrypted) ? MIC_LEN : 0);
        em_ble_rxmaxbuf_pack(cs_idx, pkt_len, pkt_len);
        em_ble_rxmaxtime_set(cs_idx, 0);

        // Set the normal win size
        em_ble_rxwincntl_pack(cs_idx, 0, BLE_RATE_NORMAL_WIN_SIZE(p_cis->rx_rate) >> 1);

        // Set channel map
        em_ble_chmap0_set(cs_idx, co_read16p(&con_info.ch_map.map[0]));
        em_ble_chmap1_set(cs_idx, co_read16p(&con_info.ch_map.map[2]));
        em_ble_chmap2_set(cs_idx, co_read16p(&con_info.ch_map.map[4]));

        // Initialize flush event counter
        em_ble_isoevtcntl_pack(cs_idx, /*flushcnt*/ 0, /*subevtcnt*/ 0);

        // Initialize the TX descriptor pointer
        em_ble_acltxdescptr_set(cs_idx, 0);

        // ISO TX/RX control
        em_ble_isotxrxcntl_set(cs_idx, 0);

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

        p_tx = p_cis->p_tx;
        // Link TX ISO descriptors together and mark them as not ready
        for (cnt = 0; cnt < BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS; cnt++)
        {
            uint8_t isodesc_idx      = p_tx->iso_descs_idx[cnt];
            uint8_t next_isodesc_idx = (cnt < (BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS - 1))
                                       ? p_tx->iso_descs_idx[cnt + 1]
                                       : p_tx->iso_descs_idx[0];

            // Set next pointer - Mark as done
            em_ble_txisoptr_pack(isodesc_idx, 1, 0, REG_EM_ADDR_GET(BLE_TX_ISO_DESC, next_isodesc_idx));
            // Clear ISO buffer
            em_ble_txisobufptr_setf(isodesc_idx, 0);
        }

        p_rx = p_cis->p_rx;
        // Link RX ISO descriptors together and mark them as not ready
        for (cnt = 0; cnt < BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS; cnt++)
        {
            uint8_t isodesc_idx      = p_rx->iso_descs_idx[cnt];
            uint8_t next_isodesc_idx = (cnt < (BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS - 1))
                                       ? p_rx->iso_descs_idx[cnt + 1]
                                       : p_rx->iso_descs_idx[0];

            // Set next pointer - Mark as done
            em_ble_rxisoptr_pack(isodesc_idx, 1, REG_EM_ADDR_GET(BLE_RX_ISO_DESC, next_isodesc_idx));
        }

        // Disable unused control
        em_ble_hopcs2cntl0_set(cs_idx, 0);
        em_ble_hopcs2cntl1_set(cs_idx, 0);
        em_ble_txheadercntl_set(cs_idx, 0);

        // SW workaround for CIE bit issue, automatic management is disabled
        ble_rwblecntl_cie_dsb_setf(1);

        // ====================================================================
        // PREPARE FIRST EVENTS
        // ====================================================================

        // First computation of hopping for first event
        lld_ci_compute_hop_scheme(p_cis, p_cis->hop_ptr[GETB(p_cis->cis_info, LLD_CIS_INFO_HOP_TOG)], p_cis->cis_evt_cnt);

        // Request alarm to be started only if new CIS cannot be inserted automatically by a start event
        if (p_cig->nb_cis == 1)
        {
            // Get pointer to alarm
            struct sch_alarm_tag *p_alarm = &p_cig->alarm;

            // Program alarm for compensation of drift
            p_alarm->time.hs = CLK_SUB(cis_anchor_ts, LLD_CI_ALARM_INIT_DISTANCE);
            p_alarm->time.hus = 0;

            // Keep in mind that alarm timer is running
            SETB(p_cig->cig_info, LLD_CIG_INFO_ALARM, 1);

            // Set the alarm
            sch_alarm_set(p_alarm);
        }
        // else CIS will be started during end of CIG handling as it is not marked as STARTED

        // The CIS supervision timer is started here
        p_cis->sup_to_start_cig_evt_cnt = p_cis->start_cig_evt_cnt;

        *p_cis_start_hs  = cis_anchor_ts;
        *p_cis_start_hus = cis_anchor_bit_off;
    }
    while (0);

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_cis_stop(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_STOP_FUNC_BIT, uint8_t, act_id);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get CIS structures
    struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated CIS exists
    if (p_cis != NULL)
    {
        // Get associated CIG structure
        struct lld_cig_env *p_cig = LLD_CIG_GET(p_cis->grp_hdl);

#if BLE_PWR_CTRL
        {
            uint8_t con_tx_rate;
            // Get connection tx rate
            if (CO_ERROR_NO_ERROR == lld_con_tx_rate_get(p_cis->con_link_id, &con_tx_rate))
            {
                // If the CIS tx rate differs, signal power change indication
                if (p_cis->tx_rate != con_tx_rate)
                {
                    struct lld_cis_pwr_change_ind *msg = KE_MSG_ALLOC(LLD_CIS_PWR_CHANGE_IND, KE_BUILD_ID(TASK_LLC, p_cis->con_link_id), TASK_NONE, lld_cis_pwr_change_ind);
                    msg->tx_rate = p_cis->tx_rate; // CIS transmit rate
                    msg->en = false; // disabled
                    ke_msg_send(msg);
                }
            }
        }
#endif // BLE_PWR_CTRL

        // Set stop reason
        p_cis->stop_reason = CO_ERROR_CON_TERM_BY_LOCAL_HOST;

        // If no subevent programmed and one CIS only in the CIG, activity can be immediately stopped
        if (!GETB(p_cig->cig_info, LLD_CIG_INFO_PROG) && (p_cig->nb_cis == 1))
        {
            // Remove the event from the EA
            sch_arb_remove(&p_cig->evt, false);

            // CIS can be removed now
            lld_cis_remove(p_cig, p_cis);

            // Free the CIG structure
            lld_cig_free(p_cig);
        }
        // else wait for next event
        else
        {
            // Keep in mind that CIS has to be stopped when CIG event will be over
            p_cig->stop_activity_ids |= CO_BIT(act_id);

            // Skip next subevents for this CIS
            SETB(p_cis->cis_info, LLD_CIS_INFO_SKIP, 1);
        }

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_cis_local_sync_en(uint8_t act_id, uint8_t direction, data_path_local_sync_cb cb_local_sync)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_LOCAL_SYNC_EN_FUNC_BIT, uint8_t, act_id, direction, cb_local_sync);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get CIS structure
    struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated CIS exists
    if (p_cis != NULL)
    {
        // Get associated CIG structure
        struct lld_cig_env *p_cig = LLD_CIG_GET(p_cis->grp_hdl);

        // Store the callback
        p_cig->cb_local_sync = cb_local_sync;

        // Indicate local sync enabled for the direction and the CIS
        p_cig->dp_local_sync_en_bf |= (1 << (act_id + (16 * direction)));

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_cis_local_sync_dis(uint8_t act_id, uint8_t direction, data_path_local_sync_cb cb_local_sync)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_LOCAL_SYNC_DIS_FUNC_BIT, uint8_t, act_id, direction, cb_local_sync);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get CIS structure
    struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated CIS exists
    if (p_cis != NULL)
    {
        // Get associated CIG structure
        struct lld_cig_env *p_cig = LLD_CIG_GET(p_cis->grp_hdl);

        // Indicate local sync disabled for the direction and the CIS
        p_cig->dp_local_sync_en_bf &= ~(1 << (act_id + (16 * direction)));

        // If local sync disabled in both directions and all CISes
        if (p_cig->dp_local_sync_en_bf == 0)
        {
            // Clear the callback
            p_cig->cb_local_sync = NULL;
        }

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#if (BLE_PERIPHERAL)
uint8_t lld_cis_peer_sync_en(uint8_t act_id, uint8_t direction, data_path_peer_sync_cb cb_peer_sync)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_PEER_SYNC_EN_FUNC_BIT, uint8_t, act_id, direction, cb_peer_sync);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get CIS structure
    struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated CIS exists
    if (p_cis != NULL)
    {
        // Get associated CIG structure
        struct lld_cig_env *p_cig = LLD_CIG_GET(p_cis->grp_hdl);

        // Store the callback
        p_cig->cb_peer_sync = cb_peer_sync;

        // Indicate local sync enabled for the direction and the CIS
        p_cig->dp_peer_sync_en_bf |= (1 << (act_id + (16 * direction)));

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_cis_peer_sync_dis(uint8_t act_id, uint8_t direction, data_path_peer_sync_cb cb_peer_sync)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_PEER_SYNC_DIS_FUNC_BIT, uint8_t, act_id, direction, cb_peer_sync);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get CIS structure
    struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated CIS exists
    if (p_cis != NULL)
    {
        // Get associated CIG structure
        struct lld_cig_env *p_cig = LLD_CIG_GET(p_cis->grp_hdl);

        // Indicate peer sync disabled for the direction and the CIS
        p_cig->dp_peer_sync_en_bf &= ~(1 << (act_id + (16 * direction)));

        // If peer sync disabled in both directions and all CISes
        if (p_cig->dp_peer_sync_en_bf == 0)
        {
            // Clear the callback
            p_cig->cb_peer_sync = NULL;
        }

        // No error has been triggered
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // (BLE_PERIPHERAL)



void lld_cis_aa_gen(uint8_t act_id, uint32_t *p_acc_addr)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_AA_GEN_FUNC_BIT, act_id, p_acc_addr);

    ASSERT_INFO(p_acc_addr != NULL, act_id, 0);

    // For the moment only use the legacy access address generation
    lld_aa_gen((uint8_t *)p_acc_addr, act_id);
}

uint8_t lld_cis_stats_get(uint8_t act_id, uint32_t *tx_unacked_packets, uint32_t *tx_flushed_packets, uint32_t *tx_last_subevent_packets,
                          uint32_t *retransmitted_packets, uint32_t *crc_error_packets, uint32_t *rx_unreceived_packets,
                          uint32_t *duplicate_packets)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_STATS_GET_FUNC_BIT, uint8_t, 8, act_id, tx_unacked_packets, tx_flushed_packets, tx_last_subevent_packets,
                                            retransmitted_packets, crc_error_packets, rx_unreceived_packets, duplicate_packets);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get CIS structure
    struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);

    GLOBAL_INT_DISABLE();

    // Check if indicated CIS exists
    if (p_cis != NULL)
    {
        // Get index of used control structure
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

        *tx_unacked_packets       = (em_ble_counter0_get(cs_idx, 1) << 16) | em_ble_counter0_get(cs_idx, 0);
        *tx_flushed_packets       = (em_ble_counter1_get(cs_idx, 1) << 16) | em_ble_counter1_get(cs_idx, 0);
        *tx_last_subevent_packets = (em_ble_counter2_get(cs_idx, 1) << 16) | em_ble_counter2_get(cs_idx, 0);
        *retransmitted_packets    = (em_ble_counter3_get(cs_idx, 1) << 16) | em_ble_counter3_get(cs_idx, 0);
        *crc_error_packets        = (em_ble_counter4_get(cs_idx, 1) << 16) | em_ble_counter4_get(cs_idx, 0);
        *rx_unreceived_packets    = (em_ble_counter5_get(cs_idx, 1) << 16) | em_ble_counter5_get(cs_idx, 0);
        *duplicate_packets        = (em_ble_counter6_get(cs_idx, 1) << 16) | em_ble_counter6_get(cs_idx, 0);

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_cis_pld_cnt_get(uint8_t act_id, uint8_t direction, uint32_t *pld_cnt)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_PLD_CNT_GET_FUNC_BIT, uint8_t, act_id, direction, pld_cnt);

    // Returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Get CIG/CIS structure
    struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);
    struct lld_cig_env *p_cig = LLD_CIG_GET(p_cis->grp_hdl);

    GLOBAL_INT_DISABLE();

    // Check if indicated CIS exists
    if (p_cis != NULL)
    {
        if (direction == ISO_SEL_TX)
        {
            struct lld_ci_data *p_tx = p_cis->p_tx;

            // Get the first 4 bytes of the payload counter
            memcpy(pld_cnt, &p_tx->pld_cnt[0], sizeof(uint32_t));
            *pld_cnt -= p_tx->bn;

            // If CIG is waiting for next event
            if (GETB(p_cig->cig_info, LLD_CIG_INFO_WAIT))
            {
                *pld_cnt -= p_tx->bn;
            }
        }
        else
        {
            struct lld_ci_data *p_rx = p_cis->p_rx;

            // Get the first 4 bytes of the payload counter
            memcpy(pld_cnt, &p_rx->pld_cnt[0], sizeof(uint32_t));
            *pld_cnt -= p_rx->bn;

            // If CIG is waiting for next event
            if (GETB(p_cig->cig_info, LLD_CIG_INFO_WAIT))
            {
                *pld_cnt -= p_rx->bn;
            }
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

int8_t lld_cis_rssi_get(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CIS_RSSI_GET_FUNC_BIT, int8_t, act_id);

    int8_t rssi = 0;

    GLOBAL_INT_DISABLE();

    // Get CIS structure
    struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);

    if (p_cis != NULL)
    {
        // Get the last RSSI value
        rssi = p_cis->last_rssi;
    }
    else
    {
        ASSERT_INFO_FORCE(0, act_id, 0);
    }

    GLOBAL_INT_RESTORE();

    return (rssi);
}
#endif //(BLE_CIS_IN_ROM)

void lld_ci_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_INIT_2_FUNC_BIT, is_reset);
#if (BLE_CIS_IN_ROM)
    if (is_reset)
    {
        // Counter
        uint8_t cnt;

        // Flush all created CIG
        for (cnt = 0; cnt < BLE_ISO_GROUP_MAX; cnt++)
        {
            struct lld_cig_env *p_cig = lld_cig_env[cnt];

            if (p_cig)
            {
                // Clean the structure
                lld_cig_free(p_cig);
            }
        }

        // Flush all created CIS
        for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
        {
            struct lld_cis_env *p_cis = lld_cis_env[cnt];

            if (p_cis)
            {
                // Clean the structure
                lld_cis_free(p_cis, false, true);
            }
        }
    }


    // By default set CIS and CIG pointers to null
    memset(&lld_cig_env, 0, sizeof(lld_cig_env));
    memset(&lld_cis_env, 0, sizeof(lld_cis_env));
#endif // BLE_CIS_IN_ROM
}

void lld_ci_ch_map_update(uint8_t link_id, struct le_ch_map *p_ch_map, uint32_t ref_timestamp)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CI_1_PATCH_TYPE, LLD_CI_CH_MAP_UPDATE_FUNC_BIT, link_id, p_ch_map, ref_timestamp);
#if (BLE_CIS_IN_ROM)
    uint8_t act_id;
    for (act_id = 0 ; act_id < BLE_ACTIVITY_MAX ; act_id++)
    {
        // Get CIS structure
        struct lld_cis_env *p_cis = LLD_CIS_GET(act_id);

        // Check if indicated CIS exists and CIS managed by specific link
        if ((p_cis != NULL) && (p_cis->con_link_id == link_id))
        {
            // Verify that no update procedure is ongoing
            if (p_cis->instant_proc.type == INSTANT_PROC_NO_PROC)
            {
                struct lld_cig_env *p_cig = LLD_CIG_GET(p_cis->grp_hdl);
                int32_t clock_diff = CLK_DIFF(p_cig->anchor_ts, ref_timestamp);

                // Instant is in the future
                if (clock_diff > 0)
                {
                    uint16_t evt_diff = CO_DIVIDE_CEIL(clock_diff, p_cig->iso_interval);

                    // Store new parameters
                    p_cis->instant_proc.type    = INSTANT_PROC_CH_MAP_UPD;
                    p_cis->instant_proc.instant = p_cis->cis_evt_cnt + p_cis->evt_inc + evt_diff;
                    memcpy(&p_cis->instant_proc.data.ch_map.map[0], p_ch_map, LE_CH_MAP_LEN);
                }
                // Instant is in the past, update immediately channel map
                else
                {
                    memcpy(&p_cis->hop_inf.ch_map, p_ch_map, LE_CH_MAP_LEN);
                }
            }
            else
            {
                ASSERT_INFO_FORCE(0, p_cis->instant_proc.type, act_id);
            }
        }
    }
#endif // (BLE_CIS_IN_ROM)
}

#if (BLE_PERIPHERAL)
void lld_ci_sync_time_update(uint8_t link_id, uint32_t last_sync_ts, int32_t sync_drift_acc)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CI_2_PATCH_TYPE, LLD_CI_SYNC_TIME_UPDATE_2_FUNC_BIT, link_id, last_sync_ts, sync_drift_acc);
#if (BLE_CIS_IN_ROM)
    // Find CIG corresponding to this ACL link
    struct lld_cig_env *p_cig;
    int cnt = 0;

    // Parse all groups
    for (; cnt < BLE_ISO_GROUP_MAX; cnt++)
    {
        p_cig = lld_cig_env[cnt];

        // Check if activity ID matches and CIG event is not ongoing
        if (p_cig && (p_cig->con_link_id == link_id) && !GETB(p_cig->cig_info, LLD_CIG_INFO_PROG))
        {
            // Get drift since last drift value
            int32_t drift_val = sync_drift_acc - p_cig->sync_drift_acc;
            // CIG anchor timestamp - half microseconds part
            int32_t cig_anchor_bit_off = p_cig->anchor_bit_off + drift_val;

            // Update CIG anchor position
            if (cig_anchor_bit_off < 0)
            {
                uint8_t num_hs = CO_DIVIDE_CEIL((-cig_anchor_bit_off), HALF_SLOT_SIZE);
                cig_anchor_bit_off += num_hs * HALF_SLOT_SIZE;
                p_cig->anchor_ts = CLK_SUB(p_cig->anchor_ts, num_hs);
            }
            else if (cig_anchor_bit_off >= HALF_SLOT_SIZE)
            {
                uint8_t num_hs = cig_anchor_bit_off / HALF_SLOT_SIZE;
                cig_anchor_bit_off -= num_hs * HALF_SLOT_SIZE;
                p_cig->anchor_ts = CLK_ADD_2(p_cig->anchor_ts, num_hs);
            }

            p_cig->anchor_bit_off = cig_anchor_bit_off;
            p_cig->last_sync_ts = last_sync_ts;
            p_cig->sync_drift_acc = sync_drift_acc;

            // If event is scheduled, update timings, assuming the sync window reduces thanks to new synchronization infos
            if (GETB(p_cig->cig_info, LLD_CIG_INFO_WAIT))
            {
                // Get event parameters
                struct sch_arb_elt_tag *p_evt = &p_cig->evt;
                // Get packed event structure structure
                struct lld_ci_se       *p_ref_se   = &(p_cig->p_se_infos[p_cig->ref_se_idx]);
                // Get pointer to information for the first subevent to be scheduled
                struct lld_ci_se       *p_first_se = &(p_cig->p_se_infos[p_cig->in_proc_se_idx]);
                // Retrieve CIS of first subevent to be scheduled
                struct lld_cis_env *p_cis = (struct lld_cis_env *) LLD_CIS_GET(p_first_se->cis_act_id);

                // Expected next anchor point (half microsecond part)
                int32_t bit_off = p_cig->anchor_bit_off + p_first_se->offset;
                // Expected next anchor point (half slot part)
                uint32_t target_clock = CLK_ADD_2(p_cig->anchor_ts, bit_off / HALF_SLOT_SIZE);

                // Compute anchor point for first sub-event based on CIG anchor point
                bit_off = CO_MOD(bit_off, HALF_SLOT_SIZE);

                // Keep expected synchronization time
                p_first_se->exp_sync_ts = target_clock;
                p_first_se->exp_sync_bit_off = bit_off;

                // Compute RX timings
                p_cig->rx_win_size = lld_rx_timing_compute(p_cig->last_sync_ts, &target_clock, &bit_off, p_cig->master_sca, p_cis->rx_rate, 0);

                p_first_se->timestamp  = target_clock;
                p_first_se->bit_offset = bit_off;

                // Update the activity arbitration element
                p_evt->time.hs      = p_first_se->timestamp;
                p_evt->time.hus     = p_first_se->bit_offset;
                p_evt->duration_min = p_ref_se->duration_packed - (p_first_se->offset - p_ref_se->offset) + p_cig->rx_win_size / 2;
            }
        }
    }
#endif // BLE_CIS_IN_ROM
}
#endif //(BLE_PERIPHERAL)


#endif //(BLE_CIS)

///@} LLDCIS
