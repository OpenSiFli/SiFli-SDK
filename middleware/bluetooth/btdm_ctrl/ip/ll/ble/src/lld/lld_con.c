/**
****************************************************************************************
*
* @file lld_con.c
*
* @brief LLD Connection source code
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDCON
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <board.h>
#include "rwip_config.h"               // stack configuration
#include "rwip.h"                      // IP definitions
#if(BLE_CENTRAL || BLE_PERIPHERAL)

#include <string.h>

#include "co_math.h"
#include "ble_util.h"                  // BLE utility functions

#include "ke_mem.h"
#include "ke_task.h"                   // kernel task management
#include "rwip.h"

#include "lld.h"                       // link driver API
#include "lld_int.h"                   // link layer driver internal

#include "dbg.h"

#include "sch_arb.h"                   // Scheduling Arbiter
#include "sch_prog.h"                  // Scheduling Programmer
#include "sch_slice.h"                 // Scheduling Slicer

#include "reg_blecore.h"               // BLE core registers
#include "reg_em_ble_cs.h"             // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"        // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"        // BLE EM RX descriptors
#if BLE_CON_CTE_REQ
    #include "reg_em_ble_rx_cte_desc.h"    // BLE EM RX CTE descriptors
#endif // BLE_CON_CTE_REQ

#if (BLE_ISO_MODE_0)
    #include "arch.h"                      // platform definition
    #include "ble_util_buf.h"          // BLE ISO Descriptor management
    #include "reg_em_ble_tx_iso_desc.h"    // BLE EM TX ISO descriptors
    #include "reg_em_ble_rx_iso_desc.h"    // BLE EM RX ISO descriptors
    #include "reg_em_ble_rx_iso_buf.h"     // BLE EM RX ISO buffer
    #include "reg_em_ble_tx_iso_buf.h"     // BLE EM RX ISO buffer
#endif //(BLE_ISO_MODE_0)

#if (BLE_CIS || BLE_ISO_MODE_0)
    #include "lld_int_iso.h"               // CIS specific definitions
#endif // (BLE_CIS)
#include "btdm_patch.h"

#undef __STATIC
#define __STATIC

/*
 * DEFINES
 *****************************************************************************************
 */


/// RX Threshold for connection
#define LLD_CON_RX_THRESHOLD                    1
#define LLD_CON_TX_THRESHOLD                    1

/// LLD connection event duration min default (in us)
#define LLD_CON_EVT_DUR_MIN_DFT                 (2*SLOT_SIZE)

/// LLD connection wait state before counting connection events for power control
#define LLD_CON_PWR_CTRL_EVT_CNT_WAIT     0xFF

#if (BLE_ISO_MODE_0)
    /// Define number of ISO TX descriptors and buffers per AM0 channel
    #define BLE_NB_TX_ISO_DESC_BUF_PER_AM0_CHAN (1)
    /// Define number of ISO RX descriptors and buffers per AM0 channel
    #define BLE_NB_RX_ISO_DESC_BUF_PER_AM0_CHAN (1)
#endif //(BLE_ISO_MODE_0)

/*
 * ENUMERATION DEFINITION
 *****************************************************************************************
 */

/// Type of the instant-related procedure
enum INSTANT_PROC_TYPE
{
    INSTANT_PROC_NO_PROC,
    INSTANT_PROC_CON_PAR_UPD,
    INSTANT_PROC_CH_MAP_UPD,
    INSTANT_PROC_PHY_UPD,
};

/// Connection event states
enum CON_EVT_STATE
{
    CON_EVT_WAIT,
    CON_EVT_ACTIVE,
    CON_EVT_END,
};


/// Connection information bit field
///
///   15   14   13   12   11   10    9    8    7   6     5    4    3    2    1    0
/// +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
/// |    |    |S_TX|MICU|MICL|NESN|HSEL|SYNC|MIC |TX_E|RX_E|TX_F|TX_T|RX_T|ESTB|ROLE|
/// +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
///
enum lld_con_fields
{
    /// Connection role (Master: 0, Slave: 1)
    LLD_CON_ROLE_LSB             = 0,
    LLD_CON_ROLE_MASK            = 0x0001,

    /// Boolean indicating whether the connection is established (true) or not (false)
    LLD_CON_ESTABLISHED_POS      = 1,
    LLD_CON_ESTABLISHED_BIT      = 0x0002,

    /// Boolean indicating whether Rx traffic has been received
    LLD_CON_RX_TRAFFIC_POS       = 2,
    LLD_CON_RX_TRAFFIC_BIT       = 0x0004,

    /// Boolean indicating whether there is Tx traffic to be transmitted
    LLD_CON_TX_TRAFFIC_POS       = 3,
    LLD_CON_TX_TRAFFIC_BIT       = 0x0008,

    /// TX flow ON/OFF
    LLD_CON_TX_FLOW_POS          = 4,
    LLD_CON_TX_FLOW_BIT          = 0x0010,

    /// Boolean indicating whether Rx traffic is encrypted
    LLD_CON_RX_ENCRYPT_POS       = 5,
    LLD_CON_RX_ENCRYPT_BIT       = 0x0020,

    /// Boolean indicating whether Tx traffic is encrypted
    LLD_CON_TX_ENCRYPT_POS       = 6,
    LLD_CON_TX_ENCRYPT_BIT       = 0x0040,

    /// Boolean indicating that MIC Failure has been detected
    LLD_CON_MIC_FAILURE_POS      = 7,
    LLD_CON_MIC_FAILURE_BIT      = 0x0080,

    /// Boolean indicating that Sync detected on event
    LLD_CON_EVT_SYNC_POS         = 8,
    LLD_CON_EVT_SYNC_BIT         = 0x0100,

    /// Hopping Selection 1 (legacy hopping) enabled
    LLD_CON_HOP_SEL_1_POS         = 9,
    LLD_CON_HOP_SEL_1_BIT         = 0x0200,

    /// Boolean indicating whether a packet from the master has been received with the NESN set to one (used for slave latency)
    LLD_CON_RX_NESN_1_POS         = 10,
    LLD_CON_RX_NESN_1_BIT         = 0x0400,

#if (BLE_ISO_MODE_0)
    /// Indicate that MIC Less encryption shall be used next time encryption is started
    LLD_CON_MIC_LESS_POS          = 11,
    LLD_CON_MIC_LESS_BIT          = 0x0800,

    /// MIC LESS Encryption used
    LLD_CON_MIC_LESS_USED_POS     = 12,
    LLD_CON_MIC_LESS_USED_BIT     = 0x1000,
#endif //(BLE_ISO_MODE_0)

    /// Slave transmits a single packet per connection event (used for slave only)
    LLD_CON_SINGLE_TX_POS         = 13,
    LLD_CON_SINGLE_TX_BIT         = 0x2000,
};

#if (BLE_ISO_MODE_0)
/// Audio Toggling Information
enum audio_tog
{
    /// Event buffer toggling
    AUDIO_EVT_TOG_BIT = 0x01,
    AUDIO_EVT_TOG_POS = 0,
};
#endif //(BLE_ISO_MODE_0)

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// New parameters for connection parameter update procedure
struct lld_instant_proc_new_par_con_par_upd
{
    /// Interval in half-slots
    uint32_t interval;
    /// Timeout in half-slots
    uint32_t timeout;
    /// Window offset in half-slots
    uint16_t win_off;
    /// Slave latency
    uint16_t latency;
    /// Indicates that the parameters have been updated, the scheduling uses new parameters
    bool params_updated;
};

/// New parameters for channel map update procedure
struct lld_instant_proc_new_par_ch_map_upd
{
    /// New channel map
    struct le_ch_map ch_map;
};

/// New parameters for physical layer update procedure
struct lld_instant_proc_new_par_phy_upd
{
    /// Transmit rate to be set (@see enum lld_rate)
    uint8_t tx_rate;
    /// Receive rate to be set (@see enum lld_rate)
    uint8_t rx_rate;
};

/// Union for storing the update information for any procedure
union lld_instant_proc_new_par
{
    struct lld_instant_proc_new_par_con_par_upd  con_par;
    struct lld_instant_proc_new_par_ch_map_upd   ch_map_upd_par;
    struct lld_instant_proc_new_par_phy_upd      phy;
};

/// Parameters update information
struct lld_instant_proc_info
{
    /// Data associated with procedure
    union lld_instant_proc_new_par data;

    /// Instant when the new parameter apply (compared to connection event counter)
    uint16_t instant;

    /**
     * Type of update procedure:
     *    - 0: No update ongoing
     *    - 1: Connection parameter update
     *    - 2: Channel map update
     *    - 3: Physical layer update
     */
    uint8_t type;

    /// Instant proc done
    bool done;
};


#if (BLE_ISO_MODE_0)

/// Isochronous Mode 0 data management structure
struct lld_am0_data
{
    /// Index of next ISO descriptor or buffer in use - in descs_idx and bufs_idx arrays
    uint8_t                          curr_iso_idx;
    /// PDU size (Range 0x00-0xFB) (in bytes)
    uint8_t                          pdu_size;
    /// Number of Buffer
    uint8_t                          buf_nb;
    /// Array of used ISO descriptors
    uint8_t                          iso_descs_idx[BLE_NB_TX_ISO_DESC_BUF_PER_AM0_CHAN];
    /// Array of used ISO buffers
    uint8_t                          iso_bufs_idx[BLE_NB_RX_ISO_DESC_BUF_PER_AM0_CHAN];
};

/// Isochronous Mode 0 information structure
struct lld_con_am0_info
{
    // -- Data information
    /// Transmission Data information
    struct lld_am0_data              tx;
    /// Reception Data information
    struct lld_am0_data              rx;

    /// Isochronous channel
    uint8_t channel;

    /// Anchor timestamp - BTS (in us)
    uint32_t                    anchor_bts;

    /// Indicate TX isr triggered for transmitted payload
    bool tx_isr_done;
    /// Indicate RX isr triggered for payload reception
    bool rx_isr_done;

    /**
     * Event counter parity, used for selecting primary or reTx
     *    * 0: even values -> primary | odd values -> reTx
     *    * 1: even values -> reTx    | odd values -> primary
     */
    uint8_t evt_parity;

    /// Indicate the prestart of the audio slave link
    bool prestart;
    /// Indicate the audio link is enabled
    bool enable;
    /// Indicate the audio event is successful
    bool success;
    /// Indicate that buffer must be reloaded (first instant they will be used)
    bool reload_buffer;
};
#endif //(BLE_ISO_MODE_0)

#if (BT_53)
/// Subrate parameters
struct lld_subrate
{
    /// Subrate factor (unit of connection event)
    uint16_t factor;
    /// New subrate factor (unit of connection event)
    uint16_t new_factor;
    /// Last subrated event (unit of connection event)
    uint16_t last_subrated_evt;
    /// New last subrated event (unit of connection event)
    uint16_t new_last_subrated_evt;
    /// New peripheral latency (unit of connection event)
    uint16_t new_latency;
    /// Continuation number (unit of connection event)
    uint16_t cont_num;
    /// New continuation number (unit of connection event)
    uint16_t new_cont_num;
    /// Number of remaining continuation attempts (unit of connection event)
    uint16_t cont_att;
    /// Connection counter of the last scheduled event
    uint16_t last_sched_evt;
    /// New supervision timeout in half-slots
    uint32_t new_timeout;
    /// Subrate transition mode (true or false)
    bool transition_mode;
    /// New subrate transition mode (true or false)
    bool new_transition_mode;
    /// Indicates if an update is pending (true or false)
    bool update;

    /// Indicates if data has been transmitted or received during the connection event (true or false)
    bool data;
};
#endif // (BT_53)

/// LLD Connection environment structure
struct lld_con_env_tag
{
    /// Connection Scheduling Arbiter data
    struct sch_arb_elt_tag       evt;

    /// LLCP TX element
    struct ble_em_llcp_buf_elt  *llcp_tx;
    /// ACL TX queue
    struct co_list               queue_acl_tx;
    /// Current buffer element processing in queue
    struct ble_em_acl_buf_elt   *curr_buf_elt;

    /// Connection update procedure information
    struct lld_instant_proc_info instant_proc;

#if (BLE_ISO_MODE_0)
    /// Isochronous Mode 0 information structure
    struct lld_con_am0_info      am0;
#endif //(BLE_ISO_MODE_0)

#if (BT_53)
    /// Subrate parameters structure
    struct lld_subrate           subrate;
#endif //(BLE_ISO_MODE_0)

    /// Next event timestamp (corresponding to the middle of the sync window when slave)
    uint32_t                     next_ts;
    /// Scheduled RX window size (in half-us)
    uint32_t                     sync_win_size;

    /// Value of CLKN when the last sync was detected
    uint32_t                     last_sync_ts;

    /// Timestamp (in half-slots) of the last connection anchor point where sync was detected
    uint32_t                     anchor_ts;

#if (BLE_CIS)
    /// Drift accumulated between expected sync time and effective sync time (in half us)
    int32_t                      sync_drift_acc;
#endif // (BLE_CIS)

    /// Value of CLKN when the last correct CRC packet is received
    uint32_t                     last_crc_ok_ts;

    /// Default event duration, for a single packet exchange, considering maximum data payloads (in half-us)
    uint32_t                     duration_min;

    /// Interval in half-slots
    uint32_t                     interval;
    /// Timeout in half-slots
    uint32_t                     timeout;

    /// Next event bit offset (in half-us)
    int16_t                      next_bit_off;

    /// Value of bit offset when the last sync was detected
    int16_t                      last_sync_bit_off;
    /**
     * Additional reception window used by slave for establishment and update (transmitWindowSize) (in half-slots)
     * Note: this value is:
     *    - non-zero during connection establishment, up to synchronizing to master
     *    - non-zero while waiting for connection update instant (but not used until the instant)
     *    - non-zero in the first connection update attempts, up to synchronizing to master
     *    - zero otherwise
     */
    uint16_t                     winsize;
    /// Latency
    uint16_t                     latency;
    /// Event duration, used only for slave, when Host has indicated the preferred duration (N * 0.625 ms)
    uint16_t                     evt_duration;

    /// Master Sleep clock accuracy (in ppm, only for slave)
    uint16_t                     master_sca;

    /// Connection event counter
    uint16_t                     evt_cnt;

    /// Amount by which the connection event counter should be incremented
    uint16_t                     evt_inc;

    /// Value of the connection event counter when the last sync was detected
    uint16_t                     last_sync_evt_cnt;

    /// Last channel index set in the control structure
    uint16_t                     last_cs_ch_idx;

    /// Length of data remaining to transmit from the current buffer
    uint16_t                     rem_data_len;
    /// Link information bit field (@see enum lld_con_fields)
    uint16_t                     link_info;

    ///  Maximum effective transmit size (in bytes)
    uint16_t                     eff_tx_octets;
    ///  Maximum effective transmit time (in us)
    uint16_t                     eff_tx_time;

    ///  Maximum effective receive size (in bytes)
    uint16_t                     eff_rx_octets;
    ///  Maximum effective receive time (in us)
    uint16_t                     eff_rx_time;

#if BLE_PWR_CTRL
    /// Path loss minimum event count (in connection intervals)
    uint16_t                    path_loss_evt_min;
    /// Path loss event count (in connection intervals)
    uint16_t                    path_loss_evt_cnt;
#endif // BLE_PWR_CTRL

    /// Maximum number of bytes that can be transmitted in data payload
    uint8_t                      max_tx_len;

    /// Hopping increment (random value in the range of 5 to 16)
    uint8_t                      hop_inc;
    /// Link identifier
    uint8_t                      link_id;
    /// Connection event state
    uint8_t                      state;
    /// Index of TX descriptor currently used by HW [0:N-1]
    uint8_t                      txdesc_index_hw;
    /// Index of TX descriptor currently used by SW [0:N-1]
    uint8_t                      txdesc_index_sw;
    /// Number of prepared TX descriptors [0:N]
    uint8_t                      txdesc_cnt;

    /// Rate used for reception (@see enum lld_rate)
    uint8_t                      rx_rate;
    /// Rate used for transmission (@see enum lld_rate)
    uint8_t                      tx_rate;

    /// Last RSSI value received in dBm
    int8_t                       last_rssi;

#if BLE_PWR_CTRL
    /// Current transmit power level for each PHY (power table index)
    uint8_t                      tx_pwr_lvl[CO_RATE_MAX];
    /// Current transmit power level for each PHY (power table index)
    uint8_t                      tx_pwr_type[CO_RATE_MAX];

    /// Current remote transmit power for each PHY (dBm, or otherwise BLE_PWR_UNKNOWN)
    int8_t                       remote_tx_pwr[CO_RATE_MAX];
    /// Current remote transmit power flags for each PHY (see @enum pwr_ctrl_flags)
    uint8_t                      remote_tx_pwr_flags[CO_RATE_MAX];
    /// Averaged RSSI value received in dBm for each PHY
    int8_t                       av_rssi[CO_RATE_MAX];
    /// Power control event count (in connection intervals)
    uint8_t                      tx_pwr_evt_cnt;
    /// Path loss monitoring enable
    bool                         path_loss_monitoring;
    /// Path loss high threshold (dBm)
    uint8_t                      path_loss_hi_thr;
    /// Path loss high hysteresis  (dBm)
    uint8_t                      path_loss_hi_hyst;
    /// Path loss low threshold (dBm)
    uint8_t                      path_loss_lo_thr;
    /// Path loss low hysteresis (dBm)
    uint8_t                      path_loss_lo_hyst;
    /// Path loss zone - active zone (@see enum le_path_loss_zone)
    uint8_t                      path_loss_zone;
    /// Path loss zone - new zone (@see enum le_path_loss_zone)
    uint8_t                      new_path_loss_zone;
#endif // BLE_PWR_CTRL

    /// Boolean indicating whether slave latency has been applied
    bool                         latency_applied;

#if BLE_CON_CTE_RSP
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
#endif // BLE_CON_CTE_RSP

    /// Arbitration priority elevation
    uint8_t                     prio_elevation;

#if (RW_DEBUG)
    /// number of events to trace
    uint8_t trc_nb_events;
    /// Pointer to the data array
    uint16_t *p_trc_data;
    /// Pointer to the data array
    uint16_t trc_data[HCI_CON_MAX_TRACE_NB_EVENT];
#endif // (RW_DEBUG)
};

/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */

/// Table indicating the duration (in us) of 1 byte
const uint16_t byte_tx_time[] =
{
    [CO_RATE_1MBPS  ]  = 8,
    [CO_RATE_2MBPS  ]  = 4,
    [CO_RATE_125KBPS]  = 64,
    [CO_RATE_500KBPS]  = 16,
};

/// Table indicating the duration (in us) of transmitting the fixed part of a LE data packet without MIC (preamble, access code, header, CRC)
const uint16_t fixed_tx_time[] =
{
    // 1Mbps => Pr:8 + AA:32 + Hdr:16 + CRC:24
    [CO_RATE_1MBPS  ]  = 80,
    // 2Mbps => Pr:8 + AA:16 + Hdr:8 + CRC:12
    [CO_RATE_2MBPS  ]  = 44,
    // 125kbps => Pr:80 + AA:256 + CI:16 + TERM1:24 + Hdr:128 + CRC:192 + TERM2:24
    [CO_RATE_125KBPS]  = 720,
    // 500kbps => Pr:80 + AA:256 + CI:16 + TERM1:24 + Hdr:32 + CRC:48 + TERM2:6
    [CO_RATE_500KBPS]  = 462,
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD Connection environment variable
__STATIC struct lld_con_env_tag *lld_con_env[BLE_ACTIVITY_MAX];

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void lld_con_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
__STATIC void lld_con_tx_prog(uint8_t link_id);

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieves the current priority index
 *
 * @param[in]  link_id       Link identifier
 * @param[out] prio_idx      Priority index
 ****************************************************************************************
 */
__INLINE uint8_t lld_con_prio_idx_get(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_PRIO_IDX_GET_2_FUNC_BIT, uint8_t, link_id);
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    uint8_t prio_idx;

    ASSERT_ERR(con_par != NULL);

    // If the connection has not been established yet
    if (!GETB(con_par->link_info, LLD_CON_ESTABLISHED))
    {
        prio_idx = RWIP_PRIO_CONNECT_ESTAB_IDX;
    }
    // If a procedure with instant is pending, and the instant is reached
    else if ((con_par->instant_proc.type != INSTANT_PROC_NO_PROC)
             && (BLE_UTIL_INSTANT_PASSED(con_par->instant_proc.instant, (con_par->evt_cnt + con_par->evt_inc))))
    {
        prio_idx = RWIP_PRIO_CONNECT_INSTANT_IDX;
    }
    // If data was received or needs to be transmitted
    else if (GETB(con_par->link_info, LLD_CON_RX_TRAFFIC)
             || GETB(con_par->link_info, LLD_CON_TX_TRAFFIC))
    {
        prio_idx = RWIP_PRIO_CONNECT_ACT_IDX;
    }
    else
    {
        prio_idx = RWIP_PRIO_CONNECT_DFT_IDX;
    }

    return (prio_idx);
}
/**
 ****************************************************************************************
 * @brief Calculate and set the maximum latency that can be used on this link
 *
 * @param[in]  link_id       Link identifier
 ****************************************************************************************
 */
__STATIC void lld_con_max_lat_calc(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_MAX_LAT_CALC_1_FUNC_BIT, link_id);
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    if (con_par->latency > 0)
    {
        uint16_t local_drift = rwip_max_drift_get();
        uint16_t peer_drift = con_par->master_sca;
#if (BT_53)
        uint32_t eff_intv = con_par->interval * con_par->subrate.factor;
#else // (BT_53)
        uint32_t eff_intv = con_par->interval;
#endif // (BT_53)

        // Make sure that the latency does not result in disconnection due to window widening rules
        uint32_t latency = (800 * HALF_SLOT_SIZE - 3200 * (BLE_IFS_DUR + 2 * BLE_MAX_JITTER) / eff_intv) / (local_drift + peer_drift) - 1;

        ASSERT_ERR(con_par->timeout > eff_intv);
        // Make sure that the latency does not result in a timeout
        latency = co_min(latency, con_par->timeout / eff_intv - 1);

        // The latency must be lower than or equal to the one proposed
        con_par->latency = co_min(latency, con_par->latency);
    }

    ASSERT_ERR(con_par->latency <= CON_LATENCY_MAX);
}

#if (BLE_ISO_MODE_0)
/**
 ****************************************************************************************
 * @brief Handle rejection of audio event by Evert Arbitrer.
 *
 * @param[in] p_con_par    Connection parameters
 ****************************************************************************************
 */
__STATIC void lld_con_am0_reject(struct lld_con_env_tag *con_par)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_AM0_REJECT_1_FUNC_BIT, con_par);
    // AM0 structure
    struct lld_con_am0_info *am0_info = &(con_par->am0);

    DBG_SWDIAG(AM0, REJECT, 1);

    // Reference anchor point (in us, based on BT timestamp)
    uint32_t ref_anchor = am0_info->anchor_bts;

    if ((am0_info->rx.pdu_size != 0) && !am0_info->rx_isr_done)
    {
        // Data information
        struct lld_am0_data *p_rx  = &(con_par->am0.rx);

        // RX buffer index
        uint8_t buf_idx = p_rx->iso_bufs_idx[p_rx->curr_iso_idx];

        // Inform ISOAL that reception is skipped
        lld_isoal_rx_done(con_par->link_id, buf_idx, ref_anchor, true);
    }

    if ((am0_info->tx.pdu_size != 0) && !am0_info->tx_isr_done)
    {
        // Data information
        struct lld_am0_data *p_tx  = &(con_par->am0.tx);

        // Inform ISOAL that buffer transmission is skipped (and never programmed)
        lld_isoal_tx_get(con_par->link_id, BLE_UTIL_ISO_INDEX_INVALID, ref_anchor, true);

        // progress to nxt TX buffer
        CO_VAL_INC(p_tx->curr_iso_idx, p_tx->buf_nb);
    }

    // reload buffers
    am0_info->reload_buffer = true;

    DBG_SWDIAG(AM0, REJECT, 0);
}
#endif //(BLE_ISO_MODE_0)

/**
 ****************************************************************************************
 * @brief Set the max data length used for transmission
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  tx_rate       TX rate considered for the transmission (@see enum lld_rate)
 * @param[in]  con_intv      Connection interval considered for the transmission (in half-slots)
 ****************************************************************************************
 */
__STATIC void lld_con_tx_len_update(uint8_t link_id, uint8_t tx_rate, uint32_t con_intv)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_LEN_UPDATE_2_FUNC_BIT, link_id, tx_rate, con_intv);
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];
    uint32_t max_tx_time, max_payl_time;
    uint8_t max_tx_len;

    // Set the negotiated max TX length as initial value
    max_tx_len = con_par->eff_tx_octets;

    // Compute the maximum transmission time, considering the scheduling/programming latency, and empty packet from peer, in us
    max_tx_time = ((con_intv - rwip_prog_delay - 3) * HALF_SLOT_SIZE) / 2;
    max_tx_time -= (fixed_tx_time[con_par->rx_rate] + BLE_IFS_DUR);

    // Compute the maximum transmission time, considering the negotiated parameter
    max_tx_time = co_min(max_tx_time, con_par->eff_tx_time);

    // Compute maximum number of bytes in payload for respecting the max transmission time
    max_payl_time = max_tx_time - fixed_tx_time[tx_rate];
    if (GETB(con_par->link_info, LLD_CON_TX_ENCRYPT)
#if (BLE_ISO_MODE_0)
            && (!GETB(con_par->link_info, LLD_CON_MIC_LESS_USED))
#endif // (BLE_ISO_MODE_0)
       )
    {
        max_payl_time -= (MIC_LEN * byte_tx_time[tx_rate]);
    }

    // Update maximum TX length
    max_tx_len = co_min(max_tx_len, max_payl_time / byte_tx_time[tx_rate]);

    // Update maximum TX length
    con_par->max_tx_len = max_tx_len;
}

/**
 ****************************************************************************************
 * @brief Set the BLE event time min/max values
 *
 * @param[in]  link_id       Link identifier
 ****************************************************************************************
 */
__STATIC void lld_con_evt_time_update(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_EVT_TIME_UPDATE_1_FUNC_BIT, link_id);
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];
    struct sch_arb_elt_tag *evt = &(con_par->evt);
    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);
    uint32_t maxevtime_limit_slot = 0;
    uint16_t max_tx_time, max_rx_time;
    bool single_tx = GETB(con_par->link_info, LLD_CON_SINGLE_TX) || !GETB(con_par->link_info, LLD_CON_TX_FLOW);

    // Compute the time for transmitting the maximum packet length
    max_tx_time = ble_util_pkt_dur_in_us(con_par->max_tx_len + (GETB(con_par->link_info, LLD_CON_TX_ENCRYPT) * MIC_LEN), con_par->tx_rate);

    // Compute the maximum receive time (the most restrictive between max RX time and max RX octets)
    max_rx_time = ble_util_pkt_dur_in_us(con_par->eff_rx_octets + (GETB(con_par->link_info, LLD_CON_RX_ENCRYPT) * MIC_LEN), con_par->rx_rate);
    max_rx_time = co_min(max_rx_time, con_par->eff_rx_time);

    // If single TX, maximum event time is configured to 0
    if (!single_tx)
    {
        // Compute the maximum duration of an event, considering the scheduling/programming latency
        maxevtime_limit_slot = (con_par->interval - rwip_prog_delay - 2) / 2;

        if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
        {
            // Compute reception window size in us
            uint32_t rx_win_size_us = (con_par->interval * (rwip_max_drift_get() + con_par->master_sca)) / 1600 + BLE_MAX_JITTER;

            // Update max event time
            maxevtime_limit_slot -= rx_win_size_us / SLOT_SIZE;

            // If Host prefers smaller event duration
            if (con_par->evt_duration != 0)
            {
                maxevtime_limit_slot = co_min(maxevtime_limit_slot, con_par->evt_duration);
            }
        }
    }

    // Set CS fields, MAXEVTIME/MINEVTIME are still expressed in slots (units of 625 us), as are Maximum/Minimum_CE_Length params.
    em_ble_maxevtime_set(cs_idx, maxevtime_limit_slot);
    em_ble_minevtime_set(cs_idx, maxevtime_limit_slot);

    // Compute minimum event duration, covering a single exchange of maximum packet lengths
    con_par->duration_min = 2 * (max_tx_time + BLE_IFS_DUR +  max_rx_time) + BLE_RESERVATION_TIME_MARGIN_HUS;
    evt->duration_min = con_par->duration_min;

    // Update connection parameters in the slicer
    sch_slice_per_add(BLE_CON, con_par->link_id, con_par->interval, evt->duration_min, false);
}

/**
 ****************************************************************************************
 * @brief Cleanup connection environment variable due to link stop
 *
 * @param[in] link_id      Link identifier
 * @param[in] indicate     Indicate Link Controller that disconnection occurs
 * @param[in] reason       Used to know disconnection reason
 ****************************************************************************************
 */
__STATIC void lld_con_cleanup(uint8_t link_id, bool indicate, uint8_t reason)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_CLEANUP_1_FUNC_BIT, link_id, indicate, reason);

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        if (indicate)
        {
            // inform Link layer controller that driver is stopped
            struct lld_disc_ind *msg = KE_MSG_ALLOC(LLD_DISC_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_disc_ind);
            msg->reason = reason;
            ke_msg_send(msg);

            // Free any remaining LLCP/ACL buffers
            {
                struct ble_em_acl_buf_elt *acl_buf_elt = (struct ble_em_acl_buf_elt *)co_list_pop_front(&(con_par->queue_acl_tx));

                // Check descriptors
                while (con_par->txdesc_cnt > 0)
                {
                    uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(con_par->link_id, con_par->txdesc_index_hw);
                    uint8_t txllid = em_ble_txphce_txllid_getf(txdesc_idx);

                    // Check LLCP/ACL
                    switch (txllid)
                    {
                    case LLID_CNTL:
                    {
                        uint16_t data_ptr = em_ble_txdataptr_getf(txdesc_idx);

                        // Free buffer
                        ble_util_buf_llcp_tx_free(data_ptr);

                        ASSERT_ERR(con_par->llcp_tx == NULL);
                    }
                    break;
                    case LLID_CONTINUE:
                    case LLID_START:
                        // ACL data will be handled later
                        break;
                    default:
                    {
                        ASSERT_ERR_FORCE(0);
                    }
                    break;
                    }

                    // Update TX pointer and counter
                    con_par->txdesc_cnt--;
                    con_par->txdesc_index_hw = CO_MOD((con_par->txdesc_index_hw + 1), BLE_NB_TX_DESC_PER_CON);

                    // Check next descriptor
                    txdesc_idx = EM_BLE_TXDESC_INDEX(con_par->link_id, con_par->txdesc_index_hw);
                }

                if (con_par->llcp_tx != NULL)
                {
                    ble_util_buf_llcp_tx_free(con_par->llcp_tx->buf_ptr);
                }

                while (acl_buf_elt != NULL)
                {
                    ble_util_buf_acl_tx_free(acl_buf_elt->buf_ptr);

                    acl_buf_elt = (struct ble_em_acl_buf_elt *)co_list_pop_front(&(con_par->queue_acl_tx));
                }
            }

#if (BLE_ISO_MODE_0)
            // Stop audio if enabled
            lld_con_am0_stop(link_id);
#endif //(BLE_ISO_MODE_0)
        }

        // Unregister connection
        sch_slice_per_remove(BLE_CON, link_id);

        // Remove permission/status of CS as now unused
        DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(EM_BLE_CS_ACT_ID_TO_INDEX(link_id))), REG_EM_BLE_CS_SIZE, false, false, false);

        // Free event memory
        ke_free(lld_con_env[link_id]);
        lld_con_env[link_id] = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Update connection as per the ongoing procedure
 ****************************************************************************************
 */
__STATIC_NO_INLINE void lld_con_instant_proc_end(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_INSTANT_PROC_END_1_FUNC_BIT, link_id);

    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    // Check which procedure is ongoing
    switch (con_par->instant_proc.type)
    {
    case  INSTANT_PROC_CON_PAR_UPD:
    {
        // Report connection parameters update completion to LLC
        ke_msg_send_basic(LLD_CON_PARAM_UPD_CFM, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE);

        // Update max TX length
        lld_con_tx_len_update(link_id, con_par->tx_rate, con_par->interval);

        // Update min/max event time, if needed
        lld_con_evt_time_update(link_id);

        // Adjust the anchor point of the last sync
        con_par->anchor_ts = CLK_ADD_2(con_par->anchor_ts, con_par->instant_proc.data.con_par.win_off);
    }
    break;
    case  INSTANT_PROC_CH_MAP_UPD:
    {
        // Report channel map update completion to LLC
        ke_msg_send_basic(LLD_CH_MAP_UPD_CFM, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE);
    }
    break;
    case  INSTANT_PROC_PHY_UPD:
    {
#if BLE_PWR_CTRL
        // Report the PHY change and new Transmit power to LLC where the TX rate has changed
        if (con_par->tx_rate != con_par->instant_proc.data.phy.tx_rate)
        {
            struct lld_con_pwr_change_ind *msg = KE_MSG_ALLOC(LLD_CON_PWR_CHANGE_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_con_pwr_change_ind);
            msg->tx_rate = con_par->instant_proc.data.phy.tx_rate; // new transmit rate
            msg->pwr = (int8_t)lld_tx_power_dbm_get(con_par->tx_pwr_lvl[msg->tx_rate], con_par->tx_pwr_type[msg->tx_rate]); // new transmit power
            msg->d_rate = con_par->tx_rate; // disabled rate
            ke_msg_send(msg);
        }

        // Reset the RSSI averaging history for the relevant rate(s) if the RX rate has changed
        if (con_par->rx_rate != con_par->instant_proc.data.phy.rx_rate)
        {
            // If receiving on the coded rate, independent S2 and S8 averaging
            if (PHY_CODED_VALUE == co_rate_to_phy[con_par->instant_proc.data.phy.rx_rate])
            {
                con_par->av_rssi[CO_RATE_125KBPS] = LLD_RSSI_UNDEF;
                con_par->av_rssi[CO_RATE_500KBPS] = LLD_RSSI_UNDEF;
            }
            else
            {
                con_par->av_rssi[con_par->instant_proc.data.phy.rx_rate] = LLD_RSSI_UNDEF;
            }
        }
#endif // BLE_PWR_CTRL

        // Store the new rates
        con_par->rx_rate = con_par->instant_proc.data.phy.rx_rate;
        con_par->tx_rate = con_par->instant_proc.data.phy.tx_rate;

        // If TX rate is one of the coded PHY, the max transmission time should guarantee a minimum payload of 27 bytes
        if ((con_par->tx_rate == CO_RATE_125KBPS) || (con_par->tx_rate == CO_RATE_500KBPS))
        {
            con_par->eff_tx_time = co_max(con_par->eff_tx_time, LE_MIN_TIME_CODED);
        }

        // If RX rate is one of the coded PHY, the max reception time should guarantee a minimum payload of 27 bytes
        if ((con_par->rx_rate == CO_RATE_125KBPS) || (con_par->rx_rate == CO_RATE_500KBPS))
        {
            con_par->eff_rx_time = co_max(con_par->eff_rx_time, LE_MIN_TIME_CODED);
        }

        // Update max TX length
        lld_con_tx_len_update(link_id, con_par->tx_rate, con_par->interval);

        // Update min/max event time, if needed
        lld_con_evt_time_update(link_id);

        // Report physical layer update completion to LLC
        ke_msg_send_basic(LLD_PHY_UPD_CFM, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE);
    }
    break;
    default:
    {
        // Should not happen
        ASSERT_WARN(0, 0, 0);
    }
    break;
    }

    // Clear update procedure
    con_par->instant_proc.type = INSTANT_PROC_NO_PROC;
    con_par->instant_proc.done = false;
}

#if (BT_53)
/**
 ****************************************************************************************
 * @brief Calculates the distance to the next applicable event taking subrating into account
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  immediate     Indicates if peripheral latency can be applied (false) or not (true)
 * @param[out] diff          Number of connection events until the next applicable event
 ****************************************************************************************
 */
__STATIC uint16_t lld_con_subr_dist(uint8_t link_id, bool immediate)
{
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];
    uint16_t next_evt = con_par->evt_cnt + con_par->evt_inc;
    uint16_t diff = 0;

    // If targeting a subrated event
    if ((con_par->subrate.factor > 1) && (con_par->subrate.transition_mode || (con_par->subrate.cont_att == 0)))
    {
        uint16_t lim = 0xFFFF;

        // Move the subrated event close to the next conection event
        if (!BLE_UTIL_INSTANT_PASSED(next_evt, con_par->subrate.last_subrated_evt))
        {
            con_par->subrate.last_subrated_evt += CO_DIVIDE_CEIL((next_evt - con_par->subrate.last_subrated_evt), con_par->subrate.factor) * con_par->subrate.factor;
        }
        else if ((con_par->subrate.last_subrated_evt - next_evt) >= con_par->subrate.factor)
        {
            con_par->subrate.last_subrated_evt -= ((con_par->subrate.last_subrated_evt - next_evt) / con_par->subrate.factor) * con_par->subrate.factor;
        }
        diff = con_par->subrate.last_subrated_evt - next_evt;

        // If transition mode
        if (con_par->subrate.transition_mode && (diff > 0))
        {
            uint16_t new_diff;

            // Move the subrated event close to the next conection event
            if (!BLE_UTIL_INSTANT_PASSED(next_evt, con_par->subrate.new_last_subrated_evt))
            {
                con_par->subrate.new_last_subrated_evt += CO_DIVIDE_CEIL((next_evt - con_par->subrate.new_last_subrated_evt), con_par->subrate.new_factor) * con_par->subrate.new_factor;
            }
            else if ((con_par->subrate.new_last_subrated_evt - next_evt) >= con_par->subrate.new_factor)
            {
                con_par->subrate.new_last_subrated_evt -= ((con_par->subrate.new_last_subrated_evt - next_evt) / con_par->subrate.new_factor) * con_par->subrate.new_factor;
            }
            new_diff = con_par->subrate.new_last_subrated_evt - next_evt;

            diff = co_min(diff, new_diff);
        }

        // If a connection update procedure is pending
        if (con_par->instant_proc.type == INSTANT_PROC_CON_PAR_UPD)
        {
            // If instant is in the future
            if (!BLE_UTIL_INSTANT_PASSED(con_par->instant_proc.instant, next_evt))
            {
                // Ensure waking up at (instant - 1)
                lim = con_par->instant_proc.instant - 1 - next_evt;
            }
            // If (instant + 1) is in the future
            else if (!BLE_UTIL_INSTANT_PASSED((con_par->instant_proc.instant + 1), next_evt))
            {
                // Ensure waking up at the instant
                lim = con_par->instant_proc.instant - next_evt;
            }
        }

        // Check if latency can be applied
        if (GETB(con_par->link_info, LLD_CON_ESTABLISHED) && (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE))
        {
            // Slave latency can be applied if there was a sync in the last frame and there is no Tx/Rx traffic
            if (!immediate
                    && (con_par->latency > 0)
                    && GETB(con_par->link_info, LLD_CON_RX_NESN_1)
                    && (!GETB(con_par->link_info, LLD_CON_RX_TRAFFIC))
                    && (!GETB(con_par->link_info, LLD_CON_TX_TRAFFIC)))
            {
                uint16_t latency_diff = diff + con_par->latency * con_par->subrate.factor;

                if (latency_diff <= lim)
                {
                    diff = latency_diff;
                    con_par->latency_applied = true;
                }
            }
        }

        diff = co_min(diff, lim);
    }

    return (diff);
}

/**
 ****************************************************************************************
 * @brief Checks if the next scheduled connection event is a subrated event (true or false)
 *
 * @param[in]  link_id       Link identifier
 * @param[out] subr_evt      Indicates whether the next scheduled connection event is a subrated event (true) or not (false)
 ****************************************************************************************
 */
__STATIC bool lld_con_is_subr_evt(uint8_t link_id)
{
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];
    bool subr_evt = (con_par->subrate.factor == 1);

    if (con_par->subrate.factor > 1)
    {
        uint16_t subr_evt_phase = CO_MOD(con_par->subrate.last_subrated_evt, con_par->subrate.factor);
        uint16_t next_evt = con_par->evt_cnt + con_par->evt_inc;
        uint16_t next_evt_phase = CO_MOD(next_evt, con_par->subrate.factor);

        subr_evt = (next_evt_phase == subr_evt_phase);
    }

    return (subr_evt);
}

/**
 ****************************************************************************************
 * @brief Updates the number of continuation attempts when an event is skipped
 *
 * @param[in]  link_id       Link identifier
 ****************************************************************************************
 */
__STATIC void lld_con_evt_skip(uint8_t link_id)
{
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    if (con_par->subrate.factor > 1)
    {
        // If subrated event
        if (lld_con_is_subr_evt(con_par->link_id))
        {
            con_par->subrate.cont_att = 0;
        }
        // If continuation event
        else if (con_par->subrate.cont_att > 0)
        {
            con_par->subrate.cont_att--;
        }
    }
}
#endif // (BT_53)

/**
 ****************************************************************************************
 * @brief Schedule next anchor point
 ****************************************************************************************
 */
__STATIC void lld_con_sched(uint8_t link_id, uint32_t clock, bool immediate)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_SCHED_2_FUNC_BIT, link_id, clock, immediate);

    // Point to parameters
    struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);
    struct lld_con_env_tag *con_par = lld_con_env[link_id];
    uint32_t target_clock, timeout;
    bool found = false;
    int32_t new_bit_off = 0;
    uint32_t rx_win_size = 2 * BLE_RATE_NORMAL_WIN_SIZE(con_par->rx_rate);

    DBG_SWDIAG(LECON, SCHED, 1);

    // Find the next anchor point
    target_clock = con_par->next_ts;

    con_par->latency_applied = false;

#if (BT_53)
    // Check if a subrate update is pending
    if (con_par->subrate.update)
    {
        con_par->subrate.transition_mode = con_par->subrate.new_transition_mode;

        if (!con_par->subrate.transition_mode)
        {
            con_par->subrate.factor = con_par->subrate.new_factor;
            con_par->subrate.last_subrated_evt = con_par->subrate.new_last_subrated_evt;
            con_par->subrate.cont_num = con_par->subrate.new_cont_num;

            con_par->latency = con_par->subrate.new_latency;
            con_par->timeout = con_par->subrate.new_timeout;

            if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
            {
                // If the Peripheral receives an LL_SUBRATE_IND PDU with S15-14 = 0b11 in a connection event with connEventCounter15-14 = 0b00
                if (((con_par->subrate.last_subrated_evt >> 14) == 0x3)
                        && ((con_par->evt_cnt >> 14) == 0x0)
                        && (con_par->subrate.factor > 1))
                {
                    // Update the base event as if the value of evt_cnt had just wrapped
                    con_par->subrate.last_subrated_evt = con_par->subrate.factor - CO_MOD((0x10000 - con_par->subrate.last_subrated_evt), con_par->subrate.factor);
                }

                // Select an appropriate latency
                lld_con_max_lat_calc(link_id);
            }
        }

        // Reset continuation attempts
        con_par->subrate.cont_att = 0;

        con_par->subrate.update = false;
    }
#endif // (BT_53)

#if (BLE_ISO_MODE_0)
    if (con_par->am0.enable)
    {
        if (con_par->am0.success)
        {
            con_par->am0.reload_buffer = true;
            // Do not schedule retx event if isochronous payload correctly received and transmitted during primary event
            if (!GETB(con_par->evt_cnt - con_par->am0.evt_parity, AUDIO_EVT_TOG) && ((con_par->instant_proc.type == INSTANT_PROC_NO_PROC)
                    // Latency cannot be used if instant not in the future
                    || !BLE_UTIL_INSTANT_PASSED(con_par->instant_proc.instant, con_par->evt_cnt)))
            {
                // Adds 1 interval to current event
                target_clock = CLK_ADD_2(target_clock, con_par->interval);
                con_par->evt_inc++;
            }
        }
    }
    else
#endif //(BLE_ISO_MODE_0)
    {
#if (BT_53)
        // Target the next applicable event taking peripheral latency into account
        {
            uint16_t diff = lld_con_subr_dist(link_id, immediate);
            con_par->evt_inc += diff;
            target_clock = CLK_ADD_2(target_clock, diff * con_par->interval);
        }
#else // (BT_53)
        if (GETB(con_par->link_info, LLD_CON_ESTABLISHED) && (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE))
        {
            // Slave latency can be applied if there was a sync in the last frame and there is no Tx/Rx traffic
            if (!immediate
                    && (con_par->latency > 0)
                    && GETB(con_par->link_info, LLD_CON_RX_NESN_1)
                    && (!GETB(con_par->link_info, LLD_CON_RX_TRAFFIC))
                    && (!GETB(con_par->link_info, LLD_CON_TX_TRAFFIC)))
            {
                uint16_t latency = con_par->latency;

                // If an instant is pending
                if (con_par->instant_proc.type != INSTANT_PROC_NO_PROC)
                {
                    // Ensure waking-up before the instant (for connection update), or at the instant (for other procedures)
                    uint16_t limit = con_par->instant_proc.instant - (con_par->instant_proc.type == INSTANT_PROC_CON_PAR_UPD);

                    // If instant is in the future
                    if (!BLE_UTIL_INSTANT_PASSED(limit, con_par->evt_cnt))
                    {
                        latency = co_min(latency, (limit - con_par->evt_cnt - 1));
                    }
                    else
                    {
                        // Latency cannot be used
                        latency = 0;
                    }
                }

                if (latency > 0)
                {
                    // Apply latency
                    target_clock = CLK_ADD_2(target_clock, latency * con_par->interval);
                    con_par->evt_inc += latency;
                    con_par->latency_applied = true;
                }
            }
        }
#endif // (BT_53)
    }

    // Make sure the target clock is in the future
    while (CLK_LOWER_EQ(target_clock, clock))
    {
#if (BLE_ISO_MODE_0)
        if (con_par->am0.enable)
        {
            uint8_t tog = (con_par->evt_cnt + con_par->evt_inc - con_par->am0.evt_parity);

            // If event was supposed to be a retx event, call handler for audio isr
            if (GETB(tog, AUDIO_EVT_TOG))
            {
                // Notify rejected
                lld_con_am0_reject(con_par);
            }
            else
            {
                // Advance the anchor timestamp 2 x con_intv_us = 2 x (con_intv in hus)/2
                con_par->am0.anchor_bts += (con_par->interval * HALF_SLOT_SIZE);
            }
        }
#endif //(BLE_ISO_MODE_0)

#if (BT_53)
        lld_con_evt_skip(link_id);
#endif // (BT_53)

        target_clock = CLK_ADD_2(target_clock, con_par->interval);
        con_par->evt_inc++;

#if (BT_53)
        // Target the next applicable event
        {
            uint16_t diff = lld_con_subr_dist(link_id, true);
            con_par->evt_inc += diff;
            target_clock = CLK_ADD_2(target_clock, diff * con_par->interval);
        }
#endif // (BT_53)
    }

    // Check if waiting for a connection update instant
    if ((con_par->instant_proc.type == INSTANT_PROC_CON_PAR_UPD)
            && !con_par->instant_proc.data.con_par.params_updated
            && (BLE_UTIL_INSTANT_PASSED(con_par->instant_proc.instant, (uint16_t)(con_par->evt_cnt + con_par->evt_inc))))
    {
        // The master targets the beginning of the window, the slave targets the middle of the window
        target_clock = CLK_ADD_2(target_clock, con_par->instant_proc.data.con_par.win_off + (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE) * (con_par->winsize >> 1));

#if (BT_53)
        // If the connection interval changes
        if (con_par->interval != con_par->instant_proc.data.con_par.interval)
        {
            con_par->subrate.factor = 1;
            con_par->subrate.cont_num = 0;
            con_par->subrate.cont_att = 0;
        }
#endif // (BT_53)

        // Update connection parameters
        con_par->interval = con_par->instant_proc.data.con_par.interval;
        con_par->latency  = con_par->instant_proc.data.con_par.latency;
        con_par->timeout  = con_par->instant_proc.data.con_par.timeout;

        // Parameters have been updated
        con_par->instant_proc.data.con_par.params_updated = true;

        // Select an appropriate latency
        if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
        {
            lld_con_max_lat_calc(con_par->link_id);
        }

        // The timeout is reset at the instant
        con_par->last_crc_ok_ts = target_clock;
    }

    timeout = (GETB(con_par->link_info, LLD_CON_ESTABLISHED)) ? con_par->timeout : 6 * con_par->interval;
    // Try to schedule and adjust the Rx window size accordingly if needed
    while (CLK_SUB(target_clock, con_par->last_crc_ok_ts) < timeout)
    {
        // The scheduling timestamp is initialized to the targeted clock value of the sync
        uint32_t sched_ts = target_clock;

#if (BLE_PERIPHERAL)
        if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
        {
            // Initialize additional RX window size
            uint32_t add_win_size = 0;

            new_bit_off = con_par->last_sync_bit_off;

            // Add transmitWindowSize if used for synchronizing to the master at connection update instant
            if ((con_par->instant_proc.type != INSTANT_PROC_CON_PAR_UPD) || con_par->instant_proc.data.con_par.params_updated)
            {
                add_win_size += con_par->winsize * HALF_SLOT_SIZE;
            }

            // Compute RX timings
            {
                uint8_t rx_rate = con_par->rx_rate;

                // Check if waiting for a PHY update instant
                if ((con_par->instant_proc.type == INSTANT_PROC_PHY_UPD)
                        && (BLE_UTIL_INSTANT_PASSED(con_par->instant_proc.instant, (uint16_t)(con_par->evt_cnt + con_par->evt_inc))))
                {
                    rx_rate = con_par->instant_proc.data.phy.rx_rate;
                }

                rx_win_size = lld_rx_timing_compute(con_par->last_sync_ts, &sched_ts, &new_bit_off, con_par->master_sca, rx_rate, add_win_size);
            }

            // Check that the sync window does not include a transmitWindowSize
            if (add_win_size == 0)
            {
                // Check window size if not greater than half of a connection interval duration
                if (((rx_win_size / 2) >= (con_par->interval * HALF_SLOT_SIZE / 2 - 2 * BLE_IFS_DUR)))
                    break;
            }

            // Set new bit offset as event delay for scheduling
            evt->time.hus = new_bit_off;

            // Add sync win size to the minimal reservation duration
            evt->duration_min = con_par->duration_min + rx_win_size / 2;
        }
#endif // (BLE_PERIPHERAL)

        evt->time.hs = sched_ts;
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            found = true;
            break;
        }
        else
        {
#if (BLE_ISO_MODE_0)
            if (con_par->am0.enable)
            {
                uint8_t tog = (con_par->evt_cnt + con_par->evt_inc - con_par->am0.evt_parity);

                // If event was supposed to be a retx event, call handler for audio isr
                if (GETB(tog, AUDIO_EVT_TOG))
                {
                    // Notify rejected
                    lld_con_am0_reject(con_par);
                }
                else
                {
                    // Advance the anchor timestamp 2 x con_intv_us = 2 x (con_intv in hus)/2
                    con_par->am0.anchor_bts += (con_par->interval * HALF_SLOT_SIZE);
                }
            }
#endif //(BLE_ISO_MODE_0)

#if (BT_53)
            lld_con_evt_skip(link_id);
#endif // (BT_53)

            // Move timestamp to the next anchor point
            target_clock = CLK_ADD_2(target_clock, con_par->interval);
            con_par->evt_inc++;

            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(lld_con_prio_idx_get(link_id)));

#if (BT_53)
            // Target the next applicable event
            {
                uint16_t diff = lld_con_subr_dist(link_id, true);
                con_par->evt_inc += diff;
                target_clock = CLK_ADD_2(target_clock, diff * con_par->interval);
            }
#endif // (BT_53)

            // Check if waiting for a connection update instant
            if ((con_par->instant_proc.type == INSTANT_PROC_CON_PAR_UPD)
                    && !con_par->instant_proc.data.con_par.params_updated
                    && (BLE_UTIL_INSTANT_PASSED(con_par->instant_proc.instant, (uint16_t)(con_par->evt_cnt + con_par->evt_inc))))
            {
                // The master targets the beginning of the window
                target_clock = CLK_ADD_2(target_clock, con_par->instant_proc.data.con_par.win_off);

                // The slave targets the middle of the window
                if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
                {
                    target_clock = CLK_ADD_2(target_clock, con_par->winsize / 2);
                }

                // Update connection parameters
                con_par->interval = con_par->instant_proc.data.con_par.interval;
                con_par->latency = con_par->instant_proc.data.con_par.latency;
                con_par->timeout = con_par->instant_proc.data.con_par.timeout;

                // Select an appropriate latency
                if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
                {
                    lld_con_max_lat_calc(con_par->link_id);
                }

                // Parameters have been updated
                con_par->instant_proc.data.con_par.params_updated = true;

                // Update the timeout
                timeout = con_par->timeout;

                // The timeout is reset at the instant
                con_par->last_crc_ok_ts = target_clock;
            }
        }
    }

    if (found)
    {
        con_par->state = CON_EVT_WAIT;
        con_par->next_ts = target_clock;

        if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
        {
            // Store the values calculated for the bit offset and Rx sync window size
            con_par->next_bit_off = new_bit_off;
            con_par->sync_win_size = rx_win_size;
        }
    }
    else
    {
        lld_con_cleanup(link_id, true, GETB(con_par->link_info, LLD_CON_ESTABLISHED) ? CO_ERROR_CON_TIMEOUT : CO_ERROR_CONN_FAILED_TO_BE_EST);
    }

    DBG_SWDIAG(LECON, SCHED, 0);
}

/**
 ****************************************************************************************
 * @brief Check the reception during activity
 ****************************************************************************************
 */
__STATIC_NO_INLINE void lld_con_rx(uint8_t link_id, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_RX_2_FUNC_BIT, link_id, timestamp);

    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    // Indicate no Rx traffic by default
    SETB(con_par->link_info, LLD_CON_RX_TRAFFIC, true);

    // Check if a packet has been received
    while (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(link_id)))
    {
        uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;

        // Retrieve RX status
        uint16_t rxstat = em_ble_rxstatce_get(rxdesc_idx);

        uint16_t rxphce = em_ble_rxphce_get(rxdesc_idx);
        uint8_t rxllid = (rxphce & EM_BLE_RXLLID_MASK) >> EM_BLE_RXLLID_LSB;
        uint16_t rxchass = em_ble_rxchass_get(rxdesc_idx);
        uint8_t rxrssi = (rxchass & EM_BLE_RSSI_MASK) >> EM_BLE_RSSI_LSB;
        uint8_t usedchidx = (rxchass & EM_BLE_USED_CH_IDX_MASK) >> EM_BLE_USED_CH_IDX_LSB;
#if BLE_PWR_CTRL
        uint8_t rx_rate = (rxchass & EM_BLE_RATE_MASK) >> EM_BLE_RATE_LSB;
#endif // BLE_PWR_CTRL

        // Trace the current RX descriptor
        TRC_REQ_RX_DESC(LLD_CON, link_id, REG_EM_BLE_RX_DESC_ADDR_GET(rxdesc_idx));

        uint8_t rx_status = RWIP_RX_OTHER_ERROR;
#if BLE_CON_CTE_REQ
        bool cte_rx_ind_sent = false;
#endif // BLE_CON_CTE_REQ

        do
        {
            uint16_t rxlength;
            uint16_t rxdataptr;
            uint32_t base_cnt = 0;

            // Check synchronization status
            if (rxstat & EM_BLE_SYNC_ERR_BIT)
            {
                rx_status = RWIP_RX_NO_SYNC;
                break;
            }

#if (BT_53)
            DBG_SWDIAG(SUBRATE, RX_IND, 1);
            // Peripheral can transmit if tx traffic
            if ((GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE) && (con_par->txdesc_cnt > 0))
            {
                con_par->subrate.data = true;
                DBG_SWDIAG(SUBRATE, DATA, con_par->subrate.data);
            }
            DBG_SWDIAG(SUBRATE, RX_IND, 0);
#endif //(BT_53)

            // Save sync time of last received packet
            if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
            {
                // Read clock value where sync has been found
                base_cnt = (em_ble_rxclknsync1_clknrxsync1_getf(rxdesc_idx) << 16)
                           | (em_ble_rxclknsync0_clknrxsync0_getf(rxdesc_idx) <<  0);

                // Save the values corresponding to the last detected sync
                con_par->last_sync_ts = base_cnt;
            }

#if BLE_PWR_CTRL
            // Save rx rate - required to distinguish S2/S8
            con_par->rx_rate = rx_rate;
#endif // BLE_PWR_CTRL

            // Update last sync timestamp
            // The sync info is saved for the first packet only
            if (!GETB(con_par->link_info, LLD_CON_EVT_SYNC))
            {
                // Indicate that sync has been found
                SETB(con_par->link_info, LLD_CON_EVT_SYNC, true);

                // Save the connection event counter
                con_par->last_sync_evt_cnt = con_par->evt_cnt;

#if (BLE_CENTRAL)
                if (GETF(con_par->link_info, LLD_CON_ROLE) == MASTER_ROLE)
                {
                    // Save the timestamp of the last anchor point
                    con_par->last_sync_ts = con_par->next_ts;
                    con_par->anchor_ts = con_par->next_ts;
                }
                else
#endif // (BLE_CENTRAL)
                {
#if (BLE_PERIPHERAL)
                    uint32_t new_offset;
                    uint32_t prev_offset = CO_MOD(con_par->anchor_ts, con_par->interval);

                    // *** Synchronize slave after correct reception
                    // Note: this algorithm supports large sync windows but not multi-attempts

                    // Read bit position where sync has been found
                    uint16_t fine_cnt = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(rxdesc_idx);

                    // Compute new sync timestamp - note base time already loaded.
                    uint32_t new_sync_ts = base_cnt;

                    // Compute new bit offset
                    int16_t new_bit_off = fine_cnt - 2 * lld_exp_sync_pos_tab[con_par->rx_rate];
                    ASSERT_ERR((new_bit_off > (-2 * HALF_SLOT_SIZE)) && (new_bit_off < HALF_SLOT_SIZE));

                    // Adjust bit offset and sync timestamp if needed
                    if (new_bit_off < 0)
                    {
                        uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
                        new_bit_off += num_hs * HALF_SLOT_SIZE;
                        new_sync_ts = CLK_SUB(new_sync_ts, num_hs);
                    }

#if (BLE_CIS)
                    // Check that the sync window does not include a TransmitWindowSize
                    if (con_par->winsize == 0
                            || ((con_par->instant_proc.type == INSTANT_PROC_CON_PAR_UPD) && !con_par->instant_proc.data.con_par.params_updated))
                    {
                        // Compute the delta time between expected packet start and effective synchronization time
                        int32_t delta_time       = (CLK_DIFF(con_par->next_ts, new_sync_ts) * HALF_SLOT_SIZE)
                                                   + (new_bit_off - con_par->last_sync_bit_off);
                        con_par->sync_drift_acc += delta_time;

                        // Update CIG if present
                        lld_ci_sync_time_update(link_id, con_par->last_sync_ts, con_par->sync_drift_acc);
                    }
#endif // (BLE_CIS)

                    // Save the values corresponding to the last detected sync
                    con_par->last_sync_bit_off = new_bit_off;
                    // After a successful sync set next_ts to new_sync_ts
                    con_par->next_ts = new_sync_ts;

                    // Save the timestamp of the last anchor point
                    con_par->anchor_ts = new_sync_ts;

                    // Compute new connection offset
                    new_offset = CO_MOD(con_par->anchor_ts, con_par->interval);

                    // If connection not yet established or connection offset has changed
                    if (!GETB(con_par->link_info, LLD_CON_ESTABLISHED) || (new_offset != prev_offset))
                    {
                        // Indicate new offset to LLC
                        struct lld_con_offset_upd_ind *msg = KE_MSG_ALLOC(LLD_CON_OFFSET_UPD_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_con_offset_upd_ind);
                        msg->con_offset = new_offset;
                        ke_msg_send(msg);
                    }

                    // Check if not waiting for a connection update
                    if (!(con_par->instant_proc.type == INSTANT_PROC_CON_PAR_UPD))
                    {
                        // Clear TransmitWindowSize
                        con_par->winsize = 0;
                    }
#endif // (BLE_PERIPHERAL)
                }

                // If connection not yet established
                if (!GETB(con_par->link_info, LLD_CON_ESTABLISHED))
                {
                    // The connection is considered established at this point
                    SETB(con_par->link_info, LLD_CON_ESTABLISHED, true);
                }

                if (!GETB(con_par->link_info, LLD_CON_RX_NESN_1)
                        && GETB(con_par->link_info, LLD_CON_ESTABLISHED)
                        && GETB(rxphce, EM_BLE_RXNESN))
                {
                    // A packet has been received with the NESN set to one, slave latency can be applied now
                    SETB(con_par->link_info, LLD_CON_RX_NESN_1, true);
                }
            }

#if (BLE_ISO_MODE_0)
            if (con_par->am0.enable)
            {
                // If slave link and first LE audio primary event, initialize the toggling information
                if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
                {
                    // Check if we have received an audio packet (LLID = 0)
                    if (!rxllid)
                    {
                        uint32_t bts = rwip_bt_time_to_bts(con_par->last_sync_ts, con_par->last_sync_bit_off);

                        if (!GETB(rxphce, EM_BLE_RXSN) && con_par->am0.prestart)
                        {
                            // This event is the first primary audio event, use its counter value to derive the toggling info
                            con_par->am0.evt_parity = con_par->evt_cnt & AUDIO_EVT_TOG_BIT;
                            con_par->am0.prestart = false;
                        }

                        // Anchor timestamp
                        con_par->am0.anchor_bts = (!GETB(con_par->evt_cnt - con_par->am0.evt_parity, AUDIO_EVT_TOG)) ?
                                                  (bts) : (bts - ((con_par->interval * HALF_SLOT_SIZE) >> 1));
                    }
                }
            }
#endif //(BLE_ISO_MODE_0)

            // Check CRC
            if (rxstat & EM_BLE_CRC_ERR_BIT)
            {
                rx_status = RWIP_RX_CRC_ERROR;
                break;
            }

            // Reload supervision timeout
            con_par->last_crc_ok_ts = con_par->last_sync_ts;

            // Consider channel as good
            rx_status = RWIP_RX_OK;

            // Check packet status
            if (rxstat & (EM_BLE_RXTIME_ERR_BIT | EM_BLE_LEN_ERR_BIT | EM_BLE_LLID_ERR_BIT |  EM_BLE_SN_ERR_BIT))
                break;

            if (rxstat & EM_BLE_MIC_ERR_BIT)
            {
                // Disconnect due to a MIC error
                SETB(con_par->link_info, LLD_CON_MIC_FAILURE, true);
                con_par->state = CON_EVT_END;
                break;
            }

            // extract length
            rxlength = GETF(rxphce, EM_BLE_RXLEN);

            // Empty packet
            if (rxlength == 0)
            {
                // Indicate no Rx traffic if peer indicates no More Data
                SETB(con_par->link_info, LLD_CON_RX_TRAFFIC, GETB(rxphce, EM_BLE_RXMD));
                break;
            }

            // Audio is not processed here
            if (rxllid == LLID_RFU)
                break;

            // Check if MIC is present
            if ((rxlength > MIC_LEN) && GETB(con_par->link_info, LLD_CON_RX_ENCRYPT)
#if (BLE_ISO_MODE_0)
                    && (!GETB(con_par->link_info, LLD_CON_MIC_LESS_USED))
#endif // (BLE_ISO_MODE_0)
               )
            {
                rxlength -= MIC_LEN;
            }

#if (BT_53)
            if ((rx_status == RWIP_RX_OK) && (rxlength > 0))
            {
                con_par->subrate.data = true;
                DBG_SWDIAG(SUBRATE, DATA, con_par->subrate.data);
            }
#endif // (BT_53)

            rxdataptr = em_ble_rxdataptr_getf(rxdesc_idx);

            // Check LLCP reception
            if (rxllid == LLID_CNTL)
            {
                DBG_SWDIAG(LEDATA, LLCP_RX, 1);

                struct lld_llcp_rx_ind *msg = KE_MSG_ALLOC_DYN(LLD_LLCP_RX_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_llcp_rx_ind, rxlength);
                msg->length = rxlength;
                msg->em_buf = rxdataptr;
                msg->event_cnt = con_par->evt_cnt;
                ke_msg_send(msg);

                DBG_SWDIAG(LEDATA, LLCP_RX, 0);
            }

            // Check ACL reception
            else if ((rxllid == LLID_CONTINUE) || (rxllid == LLID_START))
            {
                DBG_SWDIAG(LEDATA, ACL_RX, 1);

                struct lld_acl_rx_ind *msg = KE_MSG_ALLOC(LLD_ACL_RX_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_acl_rx_ind);
                msg->em_buf   = rxdataptr;
                msg->data_len = rxlength;
                msg->llid     = rxllid;

                ke_msg_send(msg);

                TRC_REQ_ACL_RX_PDU(msg)

                DBG_SWDIAG(LEDATA, ACL_RX, 0);
            }

            else
            {
                ASSERT_INFO_FORCE(0, rxllid, link_id);
                break;
            }

#if BLE_CON_CTE_REQ
            uint16_t rxcteptr = em_ble_rxcteptr_getf(rxdesc_idx);
            if (rxcteptr != 0)
            {
                ASSERT_INFO(rxcteptr >= EM_BLE_RX_CTE_DESC_OFFSET, rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);
                ASSERT_INFO(rxcteptr <  EM_BLE_RX_CTE_DESC_END,    rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);

                // CP bit and no error
                if (GETB(rxphce, EM_BLE_RXCP) && (rxstat == 0) && GETF(rxphce, EM_BLE_RXLEN))
                {
                    // Report CTE reception
                    struct lld_con_cte_rx_ind *msg = KE_MSG_ALLOC(LLD_CON_CTE_RX_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_con_cte_rx_ind);
                    uint16_t rxphcte = em_ble_rxphcte_get(rxdesc_idx);
                    msg->cte_time = GETF(rxphcte, EM_BLE_RXCTETIME);
                    msg->cte_type = GETF(rxphcte, EM_BLE_RXCTETYPE);
                    msg->phy = co_rate_to_phy[con_par->rx_rate];
                    msg->channel_idx = usedchidx;
                    msg->rssi = 10 * rwip_rf.rssi_convert(rxrssi) - lld_env.rx_path_comp; // Convert to 0.1 dBm
                    msg->rssi_antenna_id = 0;
                    msg->con_evt_cnt = con_par->evt_cnt;
                    msg->nb_iq_samp = GETF(rxphcte, EM_BLE_NBRXIQSAMP);
                    msg->em_rx_cte_desc_idx = (rxcteptr - EM_BLE_RX_CTE_DESC_OFFSET) / REG_EM_BLE_RX_CTE_DESC_SIZE;
                    ke_msg_send(msg);

                    cte_rx_ind_sent = true;
                }
            }
            // Notify LLC if this is a CTE response without CTE
            else if ((rxllid == LLID_CNTL) && (em_rd8p(rxdataptr) == LL_CTE_RSP_OPCODE) && !GETB(rxphce, EM_BLE_RXCP))
            {
                // Indicate no CTE has been received (LL_CTE_RSP should normally have one)
                struct lld_con_cte_rx_ind *msg = KE_MSG_ALLOC(LLD_CON_CTE_RX_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_con_cte_rx_ind);
                msg->cte_time = 0;
                msg->nb_iq_samp = 0;
                ke_msg_send(msg);

                cte_rx_ind_sent = true;
            }
#endif // BLE_CON_CTE_REQ

            // Clear RX data pointer
            em_ble_rxdataptr_setf(rxdesc_idx, 0);
        }
        while (0);

        // Save RSSI value in dBm
        con_par->last_rssi = rwip_rf.rssi_convert(rxrssi) - lld_env.rx_path_comp / 10;

        // Provide reception information for channel assessment
        rwip_channel_assess_ble(usedchidx, rx_status, rxrssi, timestamp, true);

#if BLE_PWR_CTRL
        if (rx_status == RWIP_RX_OK)
        {
            lld_con_rssi_update(link_id, rx_rate, con_par->last_rssi);
        }
#endif // BLE_PWR_CTRL

        // Free RX CTE descriptor if no CTE Rx indication has been sent
#if BLE_CON_CTE_REQ
        if (!cte_rx_ind_sent)
        {
            uint16_t rxcteptr = em_ble_rxcteptr_getf(rxdesc_idx);
            if (rxcteptr != 0)
            {
                ASSERT_INFO(rxcteptr >= EM_BLE_RX_CTE_DESC_OFFSET, rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);
                ASSERT_INFO(rxcteptr <  EM_BLE_RX_CTE_DESC_END,    rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);

                // Clear RX done bit
                uint16_t em_rx_cte_desc_idx = (rxcteptr - EM_BLE_RX_CTE_DESC_OFFSET) / REG_EM_BLE_RX_CTE_DESC_SIZE;
                em_ble_rxctecntl_rxdone_setf(em_rx_cte_desc_idx, 0);
            }
        }
#endif // BLE_CON_CTE_REQ

        // Free RX descriptor
        lld_rxdesc_free();
    }
}

/**
 ****************************************************************************************
 * @brief Check the transmission during activity
 ****************************************************************************************
 */
__STATIC_NO_INLINE void lld_con_tx(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_2_FUNC_BIT, link_id);
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];
    uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(con_par->link_id, con_par->txdesc_index_hw);

#if (BT_53)
    DBG_SWDIAG(SUBRATE, TX_IND, 1);
    DBG_SWDIAG(SUBRATE, TX_IND, 0);
#endif // (BT_53)

    // Check if descriptor is consumed
    while ((con_par->txdesc_cnt > 0) && em_ble_txcntl_txdone_getf(txdesc_idx))
    {
        uint8_t txllid = em_ble_txphce_txllid_getf(txdesc_idx);
        uint16_t data_ptr = em_ble_txdataptr_getf(txdesc_idx);

        // Check if LLCP or ACL has been acknowledged
        switch (txllid)
        {
        case LLID_CNTL:
        {
            DBG_SWDIAG(LEDATA, LLCP_ACK, 1);

            // Free buffer
            ble_util_buf_llcp_tx_free(data_ptr);

            // Report LLCP TX confirmation
            ke_msg_send_basic(LLD_LLCP_TX_CFM, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE);

            DBG_SWDIAG(LEDATA, LLCP_ACK, 0);
        }
        break;
        case LLID_CONTINUE:
        case LLID_START:
        {
            DBG_SWDIAG(LEDATA, ACL_ACK, 1);

            uint16_t txlength = em_ble_txphce_txlen_getf(txdesc_idx);

            if ((txlength > MIC_LEN) && GETB(con_par->link_info, LLD_CON_TX_ENCRYPT)
#if (BLE_ISO_MODE_0)
                    && (!GETB(con_par->link_info, LLD_CON_MIC_LESS_USED))
#endif // (BLE_ISO_MODE_0)
               )
            {
                txlength -= MIC_LEN;
            }

            if (txlength > 0)
            {
                struct ble_em_acl_buf_elt *acl_buf_elt = (struct ble_em_acl_buf_elt *)co_list_pick(&(con_par->queue_acl_tx));

                ASSERT_ERR(acl_buf_elt != NULL);

                uint16_t data_len = GETF(acl_buf_elt->data_len_flags, BLE_EM_ACL_DATA_LEN);


                // Check if current ACL buffer has been completely sent
                if ((data_ptr + txlength) >= (acl_buf_elt->buf_ptr + data_len))
                {
                    // Remove the data element from the queue as the data is acknowledged
                    co_list_pop_front(&con_par->queue_acl_tx);

                    if (acl_buf_elt == con_par->curr_buf_elt)
                    {
                        // Force then next programming to pick first element of the list
                        con_par->curr_buf_elt = NULL;
                    }

                    // Free buffer
                    ble_util_buf_acl_tx_free(acl_buf_elt->buf_ptr);

                    // Report ACL TX confirmation
                    ke_msg_send_basic(LLD_ACL_TX_CFM, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE);

                    TRC_REQ_ACL_ACK_TX_PDU(txdesc_idx);
                }
            }

            DBG_SWDIAG(LEDATA, ACL_ACK, 0);
        }
        break;
        default:
        {
            ASSERT_ERR_FORCE(0);
        }
        break;
        }

        // MD bit is always 1, even though the packets exchanged are empty
        // Clear MD bit on txdone - HW workaround
        em_ble_txphce_txmd_setf(txdesc_idx, 0);

        // Update TX pointer and counter
        con_par->txdesc_cnt--;

#if (RW_WLAN_COEX)
        // Configure COEX for inactive CON if not more data
        if (0 == con_par->txdesc_cnt)
        {
            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(con_par->link_id);
            em_ble_cntl_pack(cs_idx,
                             RWIP_COEX_GET(CON, TXBSY),
                             RWIP_COEX_GET(CON, RXBSY),
                             RWIP_COEX_GET(CON, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                             0,
#endif // EAVESDROPPING_SUPPORT
                             em_ble_cntl_format_getf(cs_idx));
        }
#endif //(RW_WLAN_COEX)

        DBG_SWDIAG(TX, TX_CNT, con_par->txdesc_cnt);

        con_par->txdesc_index_hw = CO_MOD((con_par->txdesc_index_hw + 1), BLE_NB_TX_DESC_PER_CON);

        // Check next descriptor
        txdesc_idx = EM_BLE_TXDESC_INDEX(con_par->link_id, con_par->txdesc_index_hw);
    }

    // Indicate TX traffic if there is remaining packet to be sent
    SETB(con_par->link_info, LLD_CON_TX_TRAFFIC, (con_par->txdesc_cnt > 0));
}

/**
 ****************************************************************************************
 * @brief Program new packets for transmission
 ****************************************************************************************
 */
__STATIC void lld_con_tx_prog(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_PROG_2_FUNC_BIT, link_id);
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];
    struct ble_em_acl_buf_elt *curr_buf;
    bool is_data_to_tx;
    struct ble_em_llcp_buf_elt *llcp_buf_elt;

    if (!con_par->curr_buf_elt)
    {
        con_par->curr_buf_elt = (struct ble_em_acl_buf_elt *)co_list_pick(&(con_par->queue_acl_tx));

        if (con_par->curr_buf_elt)
        {
            con_par->rem_data_len = GETF(con_par->curr_buf_elt->data_len_flags, BLE_EM_ACL_DATA_LEN);
        }
    }

    curr_buf = con_par->curr_buf_elt;
    is_data_to_tx = (GETB(con_par->link_info, LLD_CON_TX_FLOW) && (curr_buf && ((con_par->rem_data_len != 0) || curr_buf->hdr.next)));
    llcp_buf_elt = con_par->llcp_tx;

    if (llcp_buf_elt || is_data_to_tx)
    {
        uint8_t prev_txdesc_idx;
        bool single_tx = GETB(con_par->link_info, LLD_CON_SINGLE_TX) || !GETB(con_par->link_info, LLD_CON_TX_FLOW);

        if (BLE_NB_TX_DESC_PER_CON == con_par->txdesc_cnt)
        {
            // Is more data to TX so set MD bit of last prepared tx descriptor
            prev_txdesc_idx = EM_BLE_TXDESC_INDEX(con_par->link_id, CO_MOD((con_par->txdesc_index_sw + (BLE_NB_TX_DESC_PER_CON - 1)), BLE_NB_TX_DESC_PER_CON));
            if (!single_tx && (0 == em_ble_txcntl_txdone_getf(prev_txdesc_idx)))
            {
                em_ble_txphce_txmd_setf(prev_txdesc_idx, 1);
            }
        }
        else
        {
            // Prepare new packets
            for (uint8_t i = con_par->txdesc_cnt; i < BLE_NB_TX_DESC_PER_CON; i++)
            {
                uint16_t txlength = 0;
                uint16_t data_ptr;
                uint8_t txdesc_idx;
                uint8_t tx_llid;
                bool txmd;
                bool txcp = false;

                // Check LLCP TX queue
                if (llcp_buf_elt)
                {
                    txlength = llcp_buf_elt->length;
                    data_ptr = llcp_buf_elt->buf_ptr;
                    tx_llid = LLID_CNTL;

#if BLE_CON_CTE_RSP
                    // Check if this is a CTE response
                    txcp = (em_rd8p(llcp_buf_elt->buf_ptr) == LL_CTE_RSP_OPCODE);
#endif // BLE_CON_CTE_RSP

                    // no more LLCP packet to send
                    con_par->llcp_tx = NULL;
                    llcp_buf_elt = NULL;
                }
                // Check if ACL flow is ON
                else if (is_data_to_tx)
                {
                    // check if next buffer should be loaded
                    if ((con_par->rem_data_len == 0) && curr_buf->hdr.next)
                    {
                        con_par->curr_buf_elt = (struct ble_em_acl_buf_elt *)curr_buf->hdr.next;
                        con_par->rem_data_len = GETF(con_par->curr_buf_elt->data_len_flags, BLE_EM_ACL_DATA_LEN);
                        curr_buf = con_par->curr_buf_elt;
                    }

                    // EM position of next data to transmit
                    data_ptr = curr_buf->buf_ptr + GETF(curr_buf->data_len_flags, BLE_EM_ACL_DATA_LEN) - con_par->rem_data_len;

                    // retrieve LLID
                    tx_llid = (GETF(curr_buf->data_len_flags, BLE_EM_ACL_PBF) != PBF_CONT_HL_FRAG) ? LLID_START : LLID_CONTINUE;

                    // Check maximum packet length
                    txlength = co_min(con_par->rem_data_len, con_par->max_tx_len);

                    // force following fragment to be continue fragment
                    SETF(curr_buf->data_len_flags, BLE_EM_ACL_PBF, PBF_CONT_HL_FRAG);
                    // Update remaining data length
                    con_par->rem_data_len -= txlength;
                }
                else // nothing more to TX
                {
                    break;
                }

                // update current tx descriptor and next one
                txdesc_idx = EM_BLE_TXDESC_INDEX(con_par->link_id, con_par->txdesc_index_sw);
                prev_txdesc_idx = EM_BLE_TXDESC_INDEX(con_par->link_id, CO_MOD((con_par->txdesc_index_sw + (BLE_NB_TX_DESC_PER_CON - 1)), BLE_NB_TX_DESC_PER_CON));

                is_data_to_tx = (GETB(con_par->link_info, LLD_CON_TX_FLOW) && (curr_buf && ((con_par->rem_data_len != 0) || curr_buf->hdr.next)));

                // Check if MIC should be added
                if (GETB(con_par->link_info, LLD_CON_TX_ENCRYPT)
#if (BLE_ISO_MODE_0)
                        && (!GETB(con_par->link_info, LLD_CON_MIC_LESS_USED))
#endif // (BLE_ISO_MODE_0)
                   )
                {
                    txlength += MIC_LEN;
                }

                // Fill descriptor
                /*
                 * MD is set to 1:
                 *   - If there is more data to send
                 *   - OR if device is slave AND default slave MD is configured to 1
                 */
                txmd = !single_tx && (is_data_to_tx || (lld_env.dft_slave_md && (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)));
                em_ble_txphce_pack(txdesc_idx, /*txlen*/ txlength, /*txaclrfu*/ 0, /*txcp*/ txcp, /*txmd*/ txmd, /*txsn*/ 0, /*txnesn*/ 0, /*txllid*/ tx_llid);
                // set Data Pointer
                em_ble_txdataptr_setf(txdesc_idx, data_ptr);

                // MD bit of current and previous descriptor
                if (!single_tx && (0 == em_ble_txcntl_txdone_getf(prev_txdesc_idx)))
                {
                    em_ble_txphce_txmd_setf(prev_txdesc_idx, 1);
                }

#if BLE_CON_CTE_RSP
                if (txcp)
                {
                    em_ble_txphcte_pack(/*elt_idx*/ txdesc_idx, /*txctetype*/ con_par->cte_type, /*txcterfu*/ 0, /*txctetime*/ con_par->cte_len);
                }
#endif // BLE_CON_CTE_RSP

                // Release descriptor
                em_ble_txcntl_txdone_setf(txdesc_idx, 0);

                // Update TX pointer and counter
                con_par->txdesc_index_sw = CO_MOD((con_par->txdesc_index_sw + 1), BLE_NB_TX_DESC_PER_CON);

#if (RW_WLAN_COEX)
                // Configure COEX for active CON_DATA if not prev data
                if (0 == con_par->txdesc_cnt)
                {
                    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(con_par->link_id);
                    em_ble_cntl_pack(cs_idx,
                                     RWIP_COEX_GET(CON_DATA, TXBSY),
                                     RWIP_COEX_GET(CON_DATA, RXBSY),
                                     RWIP_COEX_GET(CON_DATA, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                                     0,
#endif // EAVESDROPPING_SUPPORT
                                     em_ble_cntl_format_getf(cs_idx));
                }
#endif //(RW_WLAN_COEX)

                con_par->txdesc_cnt++;

                TRC_REQ_ACL_PROG_TX_PDU(txdesc_idx);

                DBG_SWDIAG(TX, TX_CNT, con_par->txdesc_cnt);
            }
        }
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_con_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_EVT_START_CBK_1_FUNC_BIT, evt);
    DBG_SWDIAG(LECON, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = (struct lld_con_env_tag *) evt;
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(con_par->link_id);

        // Update the connection event counter
        con_par->evt_cnt += con_par->evt_inc;

#if (RW_DEBUG)
        if (con_par->trc_nb_events)
        {
            con_par->trc_nb_events--;
            *(con_par->p_trc_data++) = con_par->evt_cnt;

            if (!con_par->trc_nb_events)
            {
                struct lld_con_evt_trace_ind *msg = KE_MSG_ALLOC(LLD_CON_EVT_TRACE_IND, KE_BUILD_ID(TASK_LLC, con_par->link_id), TASK_NONE, lld_con_evt_trace_ind);
                msg->nb_events = con_par->p_trc_data - &lld_con_env[con_par->link_id]->trc_data[0];
                memcpy(msg->data, &con_par->trc_data[0], msg->nb_events * sizeof(uint16_t));
                ke_msg_send(msg);
            }
        }
#endif // (RW_DEBUG)

        TRC_REQ_EVT_CNT(BLE_LINKID_TO_CONHDL(con_par->link_id), con_par->evt_cnt);

        em_ble_evtcnt_setf(cs_idx, con_par->evt_cnt);
        // mark that no sync found on current event
        SETB(con_par->link_info, LLD_CON_EVT_SYNC, false);

        if (GETB(con_par->link_info, LLD_CON_HOP_SEL_1))
        {
            // Select the next CS channel index
            int16_t next_ch  = CO_MOD((con_par->last_cs_ch_idx + (con_par->evt_inc * con_par->hop_inc)), BLE_DATA_CHANNEL_NB);

            if (next_ch < 0)
            {
                next_ch += BLE_DATA_CHANNEL_NB;
            }

            // Set it in the control structure
            em_ble_hopcntl_ch_idx_setf(cs_idx, next_ch);

            // Update last CS channel index used
            con_par->last_cs_ch_idx = next_ch;
        }

        // Reset evt_inc
        con_par->evt_inc = 0;

        if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
        {
            uint32_t sync_win_size_us = (con_par->sync_win_size + 1) >> 1;

            if (sync_win_size_us > EM_BLE_CS_RXWINSZ_MAX)
            {
                // Size of the Rx half window in half-slots (wide-open mode)
                em_ble_rxwincntl_pack(cs_idx, 1, (sync_win_size_us + (SLOT_SIZE - 1)) / SLOT_SIZE);
            }
            else
            {
                // Size of the Rx half window in us (normal mode)
                em_ble_rxwincntl_pack(cs_idx, 0, (sync_win_size_us + 1) >> 1);
            }
        }

        // Check if the instant associated with update procedure is reached or over-stepped
        if ((con_par->instant_proc.type != INSTANT_PROC_NO_PROC) && BLE_UTIL_INSTANT_PASSED(con_par->instant_proc.instant, con_par->evt_cnt))
        {
            // Check which procedure is ongoing
            switch (con_par->instant_proc.type)
            {
            case  INSTANT_PROC_CON_PAR_UPD:
            {
                ASSERT_ERR(con_par->instant_proc.data.con_par.params_updated);
            }
            break;
            case  INSTANT_PROC_CH_MAP_UPD:
            {
                ASSERT_ERR((con_par->instant_proc.data.ch_map_upd_par.ch_map.map[4] & 0xE0) == 0);
                em_ble_chmap0_llchmap0_setf(cs_idx, co_read16p(&con_par->instant_proc.data.ch_map_upd_par.ch_map.map[0]));
                em_ble_chmap1_llchmap1_setf(cs_idx, co_read16p(&con_par->instant_proc.data.ch_map_upd_par.ch_map.map[2]));
                em_ble_chmap2_llchmap2_setf(cs_idx, con_par->instant_proc.data.ch_map_upd_par.ch_map.map[4]);
            }
            break;
            case  INSTANT_PROC_PHY_UPD:
            {
                em_ble_thrcntl_ratecntl_rxrate_setf(cs_idx, con_par->instant_proc.data.phy.rx_rate);
                em_ble_thrcntl_ratecntl_txrate_setf(cs_idx, con_par->instant_proc.data.phy.tx_rate);
            }
            break;
            default:
            {
                // Should not happen
                ASSERT_WARN(0, 0, 0);
            }
            break;
            }

            // Postpone sending the message until the event has been programmed
            con_par->instant_proc.done = true;
        }

#if BLE_PWR_CTRL
        // Configure transmit power
        em_ble_txrxcntl_txpwr_setf(cs_idx, con_par->tx_pwr_lvl[em_ble_thrcntl_ratecntl_txrate_getf(cs_idx)]);
#endif // BLE_PWR_CTRL

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk        = &lld_con_frm_cbk;
            prog_par.time.hs        = evt->time.hs;
            prog_par.time.hus       = evt->time.hus;
            prog_par.cs_idx         = cs_idx;
            prog_par.dummy          = cs_idx;
            // Add sync win size to the minimal reservation duration
            prog_par.bandwidth      = con_par->duration_min + con_par->sync_win_size;
            prog_par.prio_1         = evt->current_prio;
            prog_par.prio_2         = 0;
            prog_par.prio_3         = 0;
            prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
            prog_par.add.ble.ae_nps = 0;
            prog_par.add.ble.iso    = 0;
            prog_par.add.ble.sic    = 0;
            prog_par.mode           = SCH_PROG_BLE;

#if (BLE_ISO_MODE_0)
            if (con_par->am0.enable)
            {
                DBG_SWDIAG(AM0, EVT_START, 1);

                // AM0 structure
                struct lld_con_am0_info *am0_info = &(con_par->am0);
                // Used to select the event type (primary/reTx) and the audio buffer (0/1)
                uint8_t primary = !GETB(con_par->evt_cnt - am0_info->evt_parity, AUDIO_EVT_TOG);

                // Set iso parameters for the frame
                prog_par.add.ble.iso    = 1;
                prog_par.add.ble.rsvd   = primary;

                if (primary)
                {
                    // Advance the anchor timestamp 2 x con_intv_us = 2 x (con_intv in hus)/2
                    con_par->am0.anchor_bts += (con_par->interval * HALF_SLOT_SIZE);
                }

                // Clear the status
                am0_info->success    = false;

                if (am0_info->reload_buffer)
                {
                    am0_info->reload_buffer  = false;

                    am0_info->tx_isr_done    = (am0_info->tx.pdu_size == 0);
                    am0_info->rx_isr_done    = (am0_info->rx.pdu_size == 0);

                    if (am0_info->tx.pdu_size > 0)
                    {
                        uint8_t tx_desc_idx = am0_info->tx.iso_descs_idx[am0_info->tx.curr_iso_idx];

                        // mark descriptor valid
                        em_ble_txisoptr_txdone_setf(tx_desc_idx, 0);

                        // set descriptor in memory
                        em_ble_isotxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_ISO_DESC, tx_desc_idx));
                    }
                    else
                    {
                        em_ble_isotxdescptr_set(cs_idx, 0);
                    }

                    if (am0_info->rx.pdu_size > 0)
                    {
                        uint8_t rx_desc_idx = am0_info->rx.iso_descs_idx[am0_info->rx.curr_iso_idx];
                        uint8_t rx_buf_idx = am0_info->rx.iso_bufs_idx[am0_info->rx.curr_iso_idx];

                        // Set Buffer pointer
                        em_ble_rxisobufsetup_invl_setf(rx_buf_idx, LLD_ISO_INVL_SYNC_ERR);
                        em_ble_rxisobufptr_setf(rx_desc_idx, ble_util_buf_iso_emptr_get(rx_buf_idx) >> 2);
                        // mark descriptor valid
                        em_ble_rxisoptr_rxdone_setf(rx_desc_idx, 0);

                        // set descriptor in memory
                        em_ble_isorxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_RX_ISO_DESC, rx_desc_idx));
                    }
                    else
                    {
                        em_ble_isorxdescptr_set(cs_idx, 0);
                    }

                    // clear flush event counter
                    em_ble_isoevtcntl_flushcnt_setf(cs_idx, primary ? 0 : 1);
                }

                DBG_SWDIAG(AM0, EVT_START, 0);
            }
#endif //(BLE_ISO_MODE_0)

            sch_prog_push(&prog_par);
        }

        // Move state
        if (con_par->state != CON_EVT_END)
        {
            con_par->state = CON_EVT_ACTIVE;
        }

#if (BT_53)
        // Initialize subrate data
        con_par->subrate.data = (GETF(con_par->link_info, LLD_CON_ROLE) == MASTER_ROLE) && GETB(con_par->link_info, LLD_CON_TX_TRAFFIC);

        DBG_SWDIAG(SUBRATE, DATA, con_par->subrate.data);
#endif // (BT_53)
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LECON, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_con_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_EVT_CANCELED_CBK_1_FUNC_BIT, evt);
    DBG_SWDIAG(LECON, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = (struct lld_con_env_tag *) evt;

        uint32_t clock = lld_read_clock();

        ASSERT_ERR(con_par->state == CON_EVT_WAIT);

#if (BT_53)
        lld_con_evt_skip(con_par->link_id);
#endif // (BT_53)

        // Advance timestamp
        con_par->next_ts = CLK_ADD_2(con_par->next_ts, con_par->interval);
        con_par->evt_inc++;

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(lld_con_prio_idx_get(con_par->link_id)));

#if (BLE_ISO_MODE_0)
        if (con_par->am0.enable)
        {
            DBG_SWDIAG(AM0, EVT_CANCELED, 1);

            uint8_t primary = !GETB(con_par->evt_cnt - con_par->am0.evt_parity, AUDIO_EVT_TOG);

            if (primary)
            {
                // Advance the anchor timestamp 2 x con_intv_us = 2 x (con_intv in hus)/2
                con_par->am0.anchor_bts += (con_par->interval * HALF_SLOT_SIZE);
            }
            else
            {
                // ISO and supposed to be a retx event - Notify rejected
                lld_con_am0_reject(con_par);
            }

            DBG_SWDIAG(AM0, EVT_CANCELED, 0);
        }
#endif // (BLE_ISO_MODE_0)

        // Reschedule
        lld_con_sched(con_par->link_id, clock, true);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LECON, EVT_CANCELED, 0);
}

#if BLE_PWR_CTRL
__STATIC void lld_con_path_loss_monitoring(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_PATH_LOSS_MONITORING_1_FUNC_BIT, link_id);
    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    uint8_t rx_rate = con_par->rx_rate;

    if ((BLE_PWR_UNKNOWN != con_par->remote_tx_pwr[rx_rate]) && ((int8_t)LLD_RSSI_UNDEF != con_par->av_rssi[rx_rate]))
    {
        uint8_t new_zone = con_par->new_path_loss_zone;
        uint8_t path_loss = 0;

        // Determine the path loss
        if (con_par->remote_tx_pwr[rx_rate] > con_par->av_rssi[rx_rate])
        {
            path_loss = (uint8_t)(con_par->remote_tx_pwr[rx_rate] - con_par->av_rssi[rx_rate]);
        }

        if (LLD_PATH_LOSS_UNDEF == con_par->path_loss_zone)
        {
            // Set path loss zone accordingly
            new_zone = ((path_loss > con_par->path_loss_hi_thr) ? BLE_PATH_LOSS_HIGH : ((path_loss > con_par->path_loss_lo_thr) ? BLE_PATH_LOSS_MID : BLE_PATH_LOSS_LOW));
        }
        // If not in middle zone, and path loss indicates beyond threshold into middle zone (with hysteresis), count events in middle zone
        else if (((BLE_PATH_LOSS_LOW == con_par->path_loss_zone) && (path_loss > (con_par->path_loss_lo_thr + con_par->path_loss_lo_hyst)))
                 || ((BLE_PATH_LOSS_HIGH == con_par->path_loss_zone) && (path_loss < (con_par->path_loss_hi_thr - con_par->path_loss_hi_hyst))))
        {
            con_par->path_loss_evt_cnt = ((new_zone == BLE_PATH_LOSS_MID) ? (con_par->path_loss_evt_cnt + 1) : (1));
            new_zone = BLE_PATH_LOSS_MID;
        }
        // If in middle zone, and path loss indicates beyond threshold into low zone (with hysteresis), count events in low zone
        else if ((BLE_PATH_LOSS_MID == con_par->path_loss_zone) && (path_loss < (con_par->path_loss_lo_thr - con_par->path_loss_lo_hyst)))
        {
            con_par->path_loss_evt_cnt = ((new_zone == BLE_PATH_LOSS_LOW) ? (con_par->path_loss_evt_cnt + 1) : (1));
            new_zone = BLE_PATH_LOSS_LOW;
        }
        // If in middle zone, and path loss indicates beyond threshold into high zone (with hysteresis), count events in high zone
        else if ((BLE_PATH_LOSS_MID == con_par->path_loss_zone) && (path_loss > (con_par->path_loss_hi_thr + con_par->path_loss_hi_hyst)))
        {
            con_par->path_loss_evt_cnt = ((new_zone == BLE_PATH_LOSS_HIGH) ? (con_par->path_loss_evt_cnt + 1) : (1));
            new_zone = BLE_PATH_LOSS_HIGH;
        }
        else
        {
            // In expected zone, so reset the path loss event count
            con_par->path_loss_evt_cnt = 0;
        }

        con_par->new_path_loss_zone = new_zone;

        /*
         * Check relative to Minimum time, in number of connection events, to be observed once the path crosses the threshold before an event is generated.
         */
        if ((con_par->path_loss_evt_cnt > con_par->path_loss_evt_min) || ((LLD_PATH_LOSS_UNDEF == con_par->path_loss_zone) && (LLD_PATH_LOSS_UNDEF != new_zone)))
        {
            // Indicate a path loss threshold is crossed
            struct lld_con_path_loss_change_ind *msg =  KE_MSG_ALLOC(LLD_CON_PATH_LOSS_CHANGE_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_con_path_loss_change_ind);
            msg->path_loss = path_loss;
            msg->zone = new_zone;
            ke_msg_send(msg);

            // Update driver status to the new path loss zone
            con_par->path_loss_zone = new_zone;
            con_par->path_loss_evt_cnt = 0;
        }
    }
}
#endif // BLE_PWR_CTRL

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_con_frm_isr(uint8_t link_id, uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_FRM_ISR_1_FUNC_BIT, link_id, timestamp, abort);
    DBG_SWDIAG(LECON, FRM_ISR, 1);

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Remove event
        sch_arb_remove(evt, true);

        // Check connection end
        if (con_par->state == CON_EVT_END)
        {
            lld_con_cleanup(link_id, true, GETB(con_par->link_info, LLD_CON_MIC_FAILURE)
                            ? CO_ERROR_TERMINATED_MIC_FAILURE : CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        else
        {
            uint32_t clock = lld_read_clock();
            uint8_t prio_idx;

#if (BT_53)
            if (con_par->subrate.factor > 1)
            {
                // If data
                if ((!abort) && con_par->subrate.data)
                {
                    con_par->subrate.cont_att = con_par->subrate.cont_num;
                }
                // If no data
                else
                {
                    con_par->subrate.cont_att = (lld_con_is_subr_evt(link_id) || (con_par->subrate.cont_att <= 1)) ? 0 : con_par->subrate.cont_att - 1;
                }
            }
#endif // (BT_53)

            // Advance timestamp
            con_par->next_ts = CLK_ADD_2(con_par->next_ts, con_par->interval);
            con_par->evt_inc++;

            prio_idx = lld_con_prio_idx_get(link_id);

            // update event priority
            evt->current_prio = abort
                                ? RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(prio_idx))
                                : RWIP_PRIO_ADD_2(rwip_priority[prio_idx].value, con_par->prio_elevation);

#if BLE_PWR_CTRL
            // Perform path loss monitoring if enabled
            if (con_par->path_loss_monitoring)
            {
                lld_con_path_loss_monitoring(link_id);
            }
            // Power control event decounter
            if (con_par->tx_pwr_evt_cnt && (LLD_CON_PWR_CTRL_EVT_CNT_WAIT != con_par->tx_pwr_evt_cnt))
            {
                con_par->tx_pwr_evt_cnt--;
            }
#endif // BLE_PWR_CTRL

            // Schedule the next connection event
            lld_con_sched(link_id, clock, !GETB(con_par->link_info, LLD_CON_EVT_SYNC) || (con_par->last_sync_ts != con_par->last_crc_ok_ts));
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LECON, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle RX interrupt
 ****************************************************************************************
 */
__STATIC_NO_INLINE void lld_con_rx_isr(uint8_t link_id, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_RX_ISR_2_FUNC_BIT, link_id, timestamp);
    DBG_FUNC_ENTER(lld_con_rx_isr);
    DBG_SWDIAG(LECON, RX_ISR, 1);

    if (lld_con_env[link_id] != NULL)
    {
        // Check reception
        lld_con_rx(link_id, timestamp);
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(LECON, RX_ISR, 0);
    DBG_FUNC_EXIT(lld_con_rx_isr);
}


/**
 ****************************************************************************************
 * @brief Handle TX interrupt
 ****************************************************************************************
 */
__STATIC_NO_INLINE void lld_con_tx_isr(uint8_t link_id, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_ISR_2_FUNC_BIT, link_id, timestamp);

    DBG_FUNC_ENTER(lld_con_tx_isr);
    DBG_SWDIAG(LECON, TX_ISR, 1);

    if (lld_con_env[link_id] != NULL)
    {
        // Check transmission
        lld_con_tx(link_id);

        // Push more data for transmission
        lld_con_tx_prog(link_id);
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(LECON, TX_ISR, 0);
    DBG_FUNC_EXIT(lld_con_tx_isr);
}

/**
 ****************************************************************************************
 * @brief Handle skip interrupt
 ****************************************************************************************
 */
__STATIC_NO_INLINE void lld_con_frm_skip_isr(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_FRM_SKIP_ISR_1_FUNC_BIT, act_id);

    DBG_SWDIAG(LECON, EVT_CANCELED, 1);

    if (lld_con_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_con_env[act_id]->evt);
        struct lld_con_env_tag *con_par = lld_con_env[act_id];

        uint32_t clock = lld_read_clock();

        ASSERT_ERR((con_par->state == CON_EVT_ACTIVE) || (con_par->state == CON_EVT_END));

        // Remove event
        sch_arb_remove(evt, true);

        // Check connection end
        if (con_par->state == CON_EVT_END)
        {
            lld_con_cleanup(act_id, true, CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        else
        {
#if (BT_53)
            lld_con_evt_skip(con_par->link_id);
#endif // (BT_53)

            // Advance timestamp
            con_par->next_ts = CLK_ADD_2(con_par->next_ts, con_par->interval);
            con_par->evt_inc++;

            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(lld_con_prio_idx_get(act_id)));

#if (BLE_ISO_MODE_0)
            // ISO and supposed to be a retx event
            if (con_par->am0.enable && GETB(con_par->evt_cnt - con_par->am0.evt_parity, AUDIO_EVT_TOG))
            {
                // Notify rejected
                lld_con_am0_reject(con_par);
            }
#endif // (BLE_ISO_MODE_0)

            // Reschedule
            lld_con_sched(con_par->link_id, clock, true);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LECON, EVT_CANCELED, 0);
}

#if (BLE_ISO_MODE_0)
/**
 ****************************************************************************************
 * @brief Handle TX ISO AM0 interrupt
 ****************************************************************************************
 */
__STATIC void lld_con_tx_am0_isr(uint8_t link_id, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_AM0_ISR_2_FUNC_BIT, link_id, timestamp);

    struct lld_con_env_tag *con_par  = lld_con_env[link_id];

    DBG_SWDIAG(AM0, TX_ISR, 1);

    if (con_par != NULL)
    {
        // AM0 structure
        struct lld_con_am0_info *am0_info = &(con_par->am0);
        // Data information
        struct lld_am0_data *p_tx  = &(con_par->am0.tx);

        if (p_tx->pdu_size > 0)
        {
            // Reference anchor point (in us, based on BT timestamp)
            uint32_t ref_anchor = am0_info->anchor_bts;

            // progress to nxt TX buffer
            CO_VAL_INC(p_tx->curr_iso_idx, am0_info->tx.buf_nb);

            // TX buffer index
            uint8_t buf_idx = p_tx->iso_bufs_idx[p_tx->curr_iso_idx];

            // request ISOAL to fill TX buffer
            lld_isoal_tx_get(link_id, buf_idx, ref_anchor, false);
        }

        // Mark transmission succeed
        am0_info->tx_isr_done = true;

        // Indicate the audio event was successful
        con_par->am0.success = am0_info->rx_isr_done;
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(AM0, TX_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle RX ISO AM0 interrupt
 ****************************************************************************************
 */
__STATIC void lld_con_rx_am0_isr(uint8_t link_id, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_RX_AM0_ISR_2_FUNC_BIT, link_id, timestamp);

    struct lld_con_env_tag *con_par  = lld_con_env[link_id];

    DBG_SWDIAG(AM0, RX_ISR, 1);

    if (con_par != NULL)
    {
        // AM0 structure
        struct lld_con_am0_info *am0_info = &(con_par->am0);
        // Data information
        struct lld_am0_data *p_rx  = &(con_par->am0.rx);

        if (am0_info->rx.pdu_size > 0)
        {
            // Reference anchor point (in us, based on BT timestamp)
            uint32_t ref_anchor = am0_info->anchor_bts;

            // RX buffer index
            uint8_t buf_idx = am0_info->rx.iso_bufs_idx[am0_info->rx.curr_iso_idx];

            // Inform the ISOAL module that buffer is done and data can be used
            lld_isoal_rx_done(link_id, buf_idx, ref_anchor, false);

            // progress to nxt RX buffer
            CO_VAL_INC(p_rx->curr_iso_idx, am0_info->rx.buf_nb);
        }

        // Mark reception succeed
        am0_info->rx_isr_done = true;

        // Indicate the audio event was successful
        con_par->am0.success = am0_info->tx_isr_done;
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(AM0, RX_ISR, 0);
}
#endif // (BLE_ISO_MODE_0)

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_con_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_FRM_CBK_1_FUNC_BIT, timestamp, dummy, irq_type);

    uint8_t link_id = EM_BLE_CS_IDX_TO_ACT_ID(dummy);
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    if (con_par && con_par->instant_proc.done)
    {
        // End procedure & notify LLC once the instant is reached
        lld_con_instant_proc_end(link_id);
    }

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        lld_con_frm_isr(link_id, timestamp, false);
    }
    break;
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    {
        lld_con_frm_isr(link_id, timestamp, true);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        lld_con_rx_isr(link_id, timestamp);
    }
    break;
    case SCH_FRAME_IRQ_TX:
    {
        lld_con_tx_isr(link_id, timestamp);
    }
    break;
#if (BLE_ISO_MODE_0)
    case SCH_FRAME_IRQ_RX_ISO:
    {
        lld_con_rx_am0_isr(link_id, timestamp);
    }
    break;
    case SCH_FRAME_IRQ_TX_ISO:
    {
        lld_con_tx_am0_isr(link_id, timestamp);
    }
    break;
#endif // (BLE_ISO_MODE_0)
    case SCH_FRAME_IRQ_SKIP:
    {
        lld_con_frm_skip_isr(link_id);
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

uint8_t lld_con_start(uint8_t link_id, struct lld_con_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_START_2_FUNC_BIT, uint8_t, link_id, params);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if there is no connection
    if (lld_con_env[link_id] == NULL)
    {
        // Allocate event
        lld_con_env[link_id] = LLD_ALLOC_ENV(lld_con_env_tag);

        if (lld_con_env[link_id] != NULL)
        {
            // Point to parameters
            struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);
            struct lld_con_env_tag *con_par = lld_con_env[link_id];

            uint32_t clock = lld_read_clock();
            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

            LLD_INIT_ENV(con_par, lld_con_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel           = &lld_con_evt_canceled_cbk;
            evt->cb_start            = &lld_con_evt_start_cbk;
            evt->current_prio        = rwip_priority[RWIP_PRIO_CONNECT_ESTAB_IDX].value;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, 0);

            // Initialize event parameters (connection part)
            con_par->link_id = link_id;
            SETF(con_par->link_info, LLD_CON_ROLE, params->role);
            SETB(con_par->link_info, LLD_CON_RX_NESN_1, false);
            con_par->winsize = 4 * params->winsize;
            con_par->interval = 4 * params->interval;
            con_par->latency = params->latency;
            con_par->timeout = 32 * params->timeout;
            con_par->hop_inc = params->hop_inc;
            con_par->master_sca = co_sca2ppm[params->master_sca];
            SETB(con_par->link_info, LLD_CON_TX_FLOW, true);

            con_par->eff_tx_octets = BLE_MIN_OCTETS;
            con_par->eff_tx_time   = ((params->rate == CO_RATE_125KBPS) || (params->rate == CO_RATE_500KBPS)) ? LE_MIN_TIME_CODED : LE_MIN_TIME;
            con_par->eff_rx_octets = BLE_MIN_OCTETS;
            con_par->eff_rx_time   = con_par->eff_tx_time;
            con_par->max_tx_len    = BLE_MIN_OCTETS;
            con_par->rx_rate = params->rate;
            con_par->tx_rate = params->rate;


#if BLE_PWR_CTRL
            // Initialize all power levels to max
            memset(&con_par->tx_pwr_lvl[0], rwip_rf.txpwr_max, CO_RATE_MAX);
            memset(&con_par->tx_pwr_type[0], rwip_rf.txpwr_max_mod, CO_RATE_MAX);
            // Set RSSI averaging history to undefined
            memset(&con_par->av_rssi[0], LLD_RSSI_UNDEF, CO_RATE_MAX);
            // Initialize remote power levels to unknown
            memset(&con_par->remote_tx_pwr[0], BLE_PWR_UNKNOWN, CO_RATE_MAX);
            memset(&con_par->remote_tx_pwr_flags[0], LLD_PWR_FLGS_UNKNOWN, CO_RATE_MAX);
#endif // BLE_PWR_CTRL

#if (RW_DEBUG)
            // Disable the tracing of the connection event information
            con_par->trc_nb_events = 0;
#endif // (RW_DEBUG)

#if (BT_53)
            // Initialize subrate parameters
            con_par->subrate.factor = 1;
#endif // (BT_53)

            // Select an appropriate latency
            if (params->role == SLAVE_ROLE)
            {
                lld_con_max_lat_calc(con_par->link_id);
            }

            // Set control structure fields
            em_ble_cntl_pack(cs_idx,
                             RWIP_COEX_GET(CON, TXBSY),
                             RWIP_COEX_GET(CON, RXBSY),
                             RWIP_COEX_GET(CON, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                             0,
#endif // EAVESDROPPING_SUPPORT
                             ((params->role == MASTER_ROLE) ? EM_BLE_CS_FMT_MST_CONNECT : EM_BLE_CS_FMT_SLV_CONNECT));

            // Set Rx/Tx threshold + rate
            em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/LLD_CON_RX_THRESHOLD, /*txthr*/LLD_CON_TX_THRESHOLD, /*auxrate*/0,
                                         /*rxrate*/params->rate, /*txrate*/params->rate);
            // Set link field
            em_ble_linkcntl_pack(cs_idx, /*hplpmode*/ 0, /*linklbl*/ cs_idx, /*sas*/ false, /*nullrxllidflt*/false,
                                 /*micmode*/ENC_MIC_PRESENT, /*cryptmode*/ENC_MODE_PKT_PLD_CNT, /*txcrypten*/ false,
                                 /*rxcrypten*/ false, /*privnpub*/ false);

            // set Synchronization Word
            em_ble_syncwl_set(cs_idx, (params->aa.addr[1] << 8) | params->aa.addr[0]);
            em_ble_syncwh_set(cs_idx, (params->aa.addr[3] << 8) | params->aa.addr[2]);
            // set CRC Initialization value
            em_ble_crcinit0_set(cs_idx, (params->crcinit.crc[1] << 8) | params->crcinit.crc[0]);
            em_ble_crcinit1_pack(cs_idx, /*rxmaxctebuf*/ 0, /*crcinit1*/ params->crcinit.crc[2]);

            // Initialize hopping
            em_ble_hopcntl_pack(cs_idx, /*fhen*/true, /*hop_mode*/((params->ch_sel_2) ? LLD_HOP_MODE_CHAN_SEL_2 : LLD_HOP_MODE_CHAN_SEL_1),
                                /*hopint*/ (con_par->hop_inc & 0x1F), /* chidx */0);

            // Initialize sub even and flush time
            em_ble_isoevtcntl_pack(cs_idx, /*flushcnt*/ 0, /*subevtcnt*/ 0);

            // keep if channel selection 1 is enabled
            SETB(con_par->link_info, LLD_CON_HOP_SEL_1, !params->ch_sel_2);

            em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_max);

            // Assess initial timing parameters
            lld_con_evt_time_update(link_id);

            // Set Rx Max buf and Rx Max Time
            em_ble_rxmaxbuf_set(cs_idx, LE_MAX_OCTETS);
            em_ble_rxmaxtime_set(cs_idx, 0);

            // Set the normal win size
            em_ble_rxwincntl_pack(cs_idx, 0, BLE_RATE_NORMAL_WIN_SIZE(con_par->rx_rate) >> 1);

            em_ble_chmap0_set(cs_idx, co_read16p(&params->ch_map.map[0]));
            em_ble_chmap1_set(cs_idx, co_read16p(&params->ch_map.map[2]));
            em_ble_chmap2_set(cs_idx, co_read16p(&params->ch_map.map[4]));
            //em_ble_chmap2_set(cs_idx, (uint16_t)(llm_util_check_map_validity(params->chm.map[0],LE_CHNL_MAP_LEN) << 8) | data.chm.map[4]);

            // Disable antenna switching
            em_ble_txdfantpattcntl_set(cs_idx, 0);
            em_ble_rxdfantpattcntl_set(cs_idx, 0);
            em_ble_rxdfantswptr_set(cs_idx, 0);
            // By default, CTE reception is enabled, IQ sampling is disabled
            em_ble_rxdfcntl_set(cs_idx, EM_BLE_DFEN_BIT);

            // Set the event counter offset fields to 0
            em_ble_evtcnt_offset0_set(cs_idx, 0x0000);
            em_ble_evtcnt_offset1_set(cs_idx, 0x0000);
            em_ble_evtcnt_offset2_set(cs_idx, 0x0000);

            // Disable unused control
            em_ble_txheadercntl_set(cs_idx, 0);

            // Initialize TX descriptors fields
            for (int i = 0 ; i < BLE_NB_TX_DESC_PER_CON ; i++)
            {
                uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(con_par->link_id, i);
                uint8_t txdesc_idx_next = EM_BLE_TXDESC_INDEX(con_par->link_id, CO_MOD((i + 1), BLE_NB_TX_DESC_PER_CON));

                em_ble_txcntl_pack(txdesc_idx, 1 /*txdone*/, REG_EM_ADDR_GET(BLE_TX_DESC, txdesc_idx_next) /*nextptr*/);
                em_ble_txphce_pack(txdesc_idx, /*txlen*/ 0, /*txaclrfu*/ 0, /*txcp*/ 0, /*txmd*/ 0, /*txsn*/ 0, /*txnesn*/ 0, /*txllid*/ 0);
                em_ble_txdataptr_setf(txdesc_idx, 0);
            }
            // Set the Tx descriptor pointer
            em_ble_acltxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, EM_BLE_TXDESC_INDEX(con_par->link_id, 0)));

            // Initialize timing parameters
            con_par->last_sync_ts = CLK_SUB(params->first_anchor_hs, 4 * params->winoffset);
            con_par->last_sync_bit_off = params->first_anchor_hus;
            con_par->next_bit_off = params->first_anchor_hus;
            con_par->anchor_ts = con_par->last_sync_ts;
            con_par->last_crc_ok_ts = con_par->last_sync_ts;
            con_par->next_ts = params->first_anchor_hs + con_par->winsize / 2;
            con_par->sync_win_size = (params->role == SLAVE_ROLE) ? con_par->winsize * HALF_SLOT_SIZE : 2 * BLE_RATE_NORMAL_WIN_SIZE(con_par->rx_rate);

            // Register connection
            sch_slice_per_add(BLE_CON, con_par->link_id, con_par->interval, evt->duration_min, false);

            // Schedule connection
            lld_con_sched(link_id, clock, true);

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }

    return (status);
}

uint8_t lld_con_stop(uint8_t link_id, bool immediate)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_STOP_2_FUNC_BIT, uint8_t, link_id, immediate);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        switch (con_par->state)
        {
        case CON_EVT_WAIT:
        {
            // check if next event has to be executed
            if (immediate)
            {
                // Remove event
                sch_arb_remove(evt, false);

                lld_con_cleanup(link_id, true, CO_ERROR_CON_TERM_BY_LOCAL_HOST);
                break;
            }
        }
        // no break;

        case CON_EVT_ACTIVE:
        {
            // Move state
            con_par->state = CON_EVT_END;
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
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_llcp_tx(uint8_t link_id, struct ble_em_llcp_buf_elt *buf_elt)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_LLCP_TX_1_FUNC_BIT, uint8_t, link_id, buf_elt);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(LEDATA, LLCP_TX, 1);
    DBG_SWDIAG(LETX, LLCP_TX, 1);

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // The queue_llcp_tx is removed as there should never be two LLCP in transmit. Assert if this is attempted.
        ASSERT_ERR(NULL == con_par->llcp_tx);

        // Push at the end of LLCP TX queue
        con_par->llcp_tx = buf_elt;

        // Indicate TX traffic
        SETB(con_par->link_info, LLD_CON_TX_TRAFFIC, true);

        // Push new data for transmission - should be called unconditionally, if possible the LLCP will be pushed in the ongoing event
        lld_con_tx_prog(link_id);

        // Check if waiting for the next event scheduled with slave latency
        if ((con_par->state == CON_EVT_WAIT)
                && (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
                && con_par->latency_applied)
        {
            struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);
            uint32_t clock = lld_read_clock();
            uint32_t distance = CLK_SUB(con_par->next_ts, clock);

            // Check if event is scheduled later than 1 connection interval
            if (distance > con_par->interval)
            {
                // Number of events to advance
                uint16_t evt_dec = distance / con_par->interval;

                ASSERT_ERR(con_par->evt_inc > evt_dec);

                // Move target timestamp
                con_par->next_ts = CLK_SUB(con_par->next_ts, evt_dec * con_par->interval);
                // Move event counter increment
                con_par->evt_inc -= evt_dec;

                // Extract event from schedule
                sch_arb_remove(evt, false);

                // Reschedule with event earlier
                lld_con_sched(link_id, clock, true);
            }
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(LEDATA, LLCP_TX, 0);
    DBG_SWDIAG(LETX, LLCP_TX, 0);

    return (status);
}

uint8_t lld_con_data_tx(uint8_t link_id, struct ble_em_acl_buf_elt *buf_elt)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_DATA_TX_1_FUNC_BIT, uint8_t, link_id, buf_elt);

    DBG_FUNC_ENTER(lld_con_data_tx);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(LEDATA, ACL_TX, 1);
    DBG_SWDIAG(LETX, ACL_TX, 1);

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Push at the end of ACL TX queue
        co_list_push_back(&con_par->queue_acl_tx, &buf_elt->hdr);

        // Indicate TX traffic
        SETB(con_par->link_info, LLD_CON_TX_TRAFFIC, true);

        // Push new data for transmission - should be called unconditionally, if possible the LLCP will be pushed in the ongoing event
        lld_con_tx_prog(link_id);

        // Check if waiting for the next event scheduled with slave latency
        if ((con_par->state == CON_EVT_WAIT)
                && (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
                && con_par->latency_applied)
        {
            struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);
            uint32_t clock = lld_read_clock();
            uint32_t distance = CLK_SUB(con_par->next_ts, clock);

            // Check if event is scheduled later than 1 connection interval
            if (distance > con_par->interval)
            {
                // Number of events to advance
                uint16_t evt_dec = distance / con_par->interval;

                ASSERT_ERR(con_par->evt_inc > evt_dec);

                // Move target timestamp
                con_par->next_ts = CLK_SUB(con_par->next_ts, evt_dec * con_par->interval);
                // Move event counter increment
                con_par->evt_inc -= evt_dec;

                // Extract event from schedule
                sch_arb_remove(evt, false);

                // Reschedule with event earlier
                lld_con_sched(link_id, clock, true);
            }
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(LEDATA, ACL_TX, 0);
    DBG_SWDIAG(LETX, ACL_TX, 0);
    DBG_FUNC_EXIT(lld_con_data_tx);

    return (status);
}

uint8_t lld_con_data_flow_set(uint8_t link_id, bool enable)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_DATA_FLOW_SET_1_FUNC_BIT, uint8_t, link_id, enable);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Set data flow
        SETB(con_par->link_info, LLD_CON_TX_FLOW, enable);

        // Update event time parameters
        lld_con_evt_time_update(link_id);

        // If flow is restarted, pending data may be pushed for transmission
        if (enable)
        {
            // Push new data for transmission - should be called unconditionally, if possible the LLCP will be pushed in the ongoing event
            lld_con_tx_prog(link_id);

            // Check if data to send and waiting for the next event scheduled with slave latency
            if ((con_par->txdesc_cnt > 0)
                    && (con_par->state == CON_EVT_WAIT)
                    && (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
                    && con_par->latency_applied)
            {
                struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);
                uint32_t clock = lld_read_clock();
                uint32_t distance = CLK_SUB(con_par->next_ts, clock);

                // Check if event is scheduled later than 1 connection interval
                if (distance > con_par->interval)
                {
                    // Number of events to advance
                    uint16_t evt_dec = distance / con_par->interval;

                    ASSERT_ERR(con_par->evt_inc > evt_dec);

                    // Move target timestamp
                    con_par->next_ts = CLK_SUB(con_par->next_ts, evt_dec * con_par->interval);
                    // Move event counter increment
                    con_par->evt_inc -= evt_dec;

                    // Extract event from schedule
                    sch_arb_remove(evt, false);

                    // Reschedule with event earlier
                    lld_con_sched(link_id, clock, false);
                }
            }
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_param_update(uint8_t link_id, uint8_t win_size, uint16_t win_off, uint16_t interval, uint16_t latency, uint16_t timeout, uint16_t instant)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_PARAM_UPDATE_1_FUNC_BIT, uint8_t, 7, link_id, win_size, win_off, interval, latency, timeout, instant);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Verify that no update procedure is ongoing
        if (con_par->instant_proc.type == INSTANT_PROC_NO_PROC)
        {
            // Store new parameters
            con_par->instant_proc.type = INSTANT_PROC_CON_PAR_UPD;
            con_par->instant_proc.instant = instant;
            con_par->instant_proc.data.con_par.win_off = 4 * win_off;
            con_par->instant_proc.data.con_par.interval = 4 * interval;
            con_par->instant_proc.data.con_par.latency = latency;
            con_par->instant_proc.data.con_par.timeout = 32 * timeout;
            con_par->winsize = 4 * win_size;

            // If new interval is smaller, and PHY is coded
            if ((con_par->interval > 4 * interval) && (con_par->tx_rate >= CO_RATE_125KBPS))
            {
                // Update max TX length
                lld_con_tx_len_update(link_id, con_par->tx_rate, 4 * interval);
            }

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            ASSERT_INFO_FORCE(0, con_par->instant_proc.type, 0);
        }

        // Indicate that the parameters have not been updated yet and the procedure is not complete
        con_par->instant_proc.data.con_par.params_updated = false;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#if (BT_53)
uint8_t lld_con_subrate(uint8_t link_id, uint16_t subrate, uint16_t base_evt, uint16_t latency, uint16_t cont_num, uint16_t timeout, bool t_mode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        con_par->subrate.new_factor = subrate;
        con_par->subrate.new_last_subrated_evt = base_evt;
        con_par->subrate.new_latency = latency;
        con_par->subrate.new_cont_num = cont_num;
        con_par->subrate.new_timeout = 32 * timeout;
        con_par->subrate.new_transition_mode = (GETF(con_par->link_info, LLD_CON_ROLE) == MASTER_ROLE) ? t_mode : false;
        con_par->subrate.update = true;

        status = CO_ERROR_NO_ERROR;

        // If transition mode wait for the end of the next scheduled event before applying the new parameters
        // This is because the LL_SUBRATE_IND PDU is expected to be transmitted during the next event
        // If not transition mode update the parameters immediately
        if ((con_par->state == CON_EVT_WAIT) && !con_par->subrate.new_transition_mode)
        {
            struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);
            uint32_t clock = lld_read_clock();
            uint32_t distance = CLK_SUB(con_par->next_ts, clock);

            // Number of events to advance
            uint16_t evt_dec = distance / con_par->interval;

            ASSERT_ERR(con_par->evt_inc > evt_dec);

            // Move target timestamp
            con_par->next_ts = CLK_SUB(con_par->next_ts, evt_dec * con_par->interval);
            // Move event counter increment
            con_par->evt_inc -= evt_dec;

            // Extract event from schedule
            sch_arb_remove(evt, false);

            // Reschedule
            lld_con_sched(link_id, clock, true);
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // (BT_53)

uint8_t lld_con_ch_map_update(uint8_t link_id, struct le_ch_map *map, uint16_t instant)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_CH_MAP_UPDATE_1_FUNC_BIT, uint8_t, link_id, map, instant);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Verify that no update procedure is ongoing
        if (con_par->instant_proc.type == INSTANT_PROC_NO_PROC)
        {
            // Store new parameters
            con_par->instant_proc.type = INSTANT_PROC_CH_MAP_UPD;
            con_par->instant_proc.instant = instant;
            memcpy(&con_par->instant_proc.data.ch_map_upd_par.ch_map.map[0], map, LE_CH_MAP_LEN);

#if (BLE_CIS)
            {
                uint32_t ref_timestamp = 0;
                int16_t  evt_cnt_diff;

                // Compute reference timestamp according to provided map update instant
                evt_cnt_diff   = BLE_UTIL_EVT_CNT_DIFF(instant, con_par->evt_cnt + con_par->evt_inc);
                ref_timestamp  = CLK_ADD_2(con_par->next_ts, evt_cnt_diff * con_par->interval);

                // inform related CIS that channel map will be updated
                lld_ci_ch_map_update(link_id, map, ref_timestamp);
            }
#endif // (BLE_CIS)

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            ASSERT_INFO_FORCE(0, con_par->instant_proc.type, 0);
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_data_len_update(uint8_t link_id, uint16_t eff_tx_time, uint16_t eff_tx_octets, uint16_t eff_rx_time, uint16_t eff_rx_octets)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_DATA_LEN_UPDATE_1_FUNC_BIT, uint8_t, 5, link_id, eff_tx_time, eff_tx_octets, eff_rx_time, eff_rx_octets);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Update effective maximum TX/RX bytes and time on connection
        con_par->eff_tx_octets = eff_tx_octets;
        con_par->eff_tx_time = eff_tx_time;
        con_par->eff_rx_octets = eff_rx_octets;
        con_par->eff_rx_time = eff_rx_time;

        // If TX rate is one of the coded PHY, the max transmission time should guarantee a minimum payload of 27 bytes
        if ((con_par->tx_rate == CO_RATE_125KBPS) || (con_par->tx_rate == CO_RATE_500KBPS))
        {
            con_par->eff_tx_time = co_max(con_par->eff_tx_time, LE_MIN_TIME_CODED);
        }

        // If RX rate is one of the coded PHY, the max reception time should guarantee a minimum payload of 27 bytes
        if ((con_par->rx_rate == CO_RATE_125KBPS) || (con_par->rx_rate == CO_RATE_500KBPS))
        {
            con_par->eff_rx_time = co_max(con_par->eff_rx_time, LE_MIN_TIME_CODED);
        }

        // Update max TX length
        lld_con_tx_len_update(link_id, con_par->tx_rate, con_par->interval);

        // Reassess min/max event time
        lld_con_evt_time_update(link_id);

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_phys_update(uint8_t link_id, uint8_t tx_rate, uint8_t rx_rate, uint16_t instant)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_PHYS_UPDATE_2_FUNC_BIT, uint8_t, link_id, tx_rate, rx_rate, instant);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Verify that no update procedure is ongoing
        if (con_par->instant_proc.type == INSTANT_PROC_NO_PROC)
        {
            // Store new parameters
            con_par->instant_proc.type = INSTANT_PROC_PHY_UPD;
            con_par->instant_proc.instant = instant;
            con_par->instant_proc.data.phy.tx_rate = tx_rate;
            con_par->instant_proc.data.phy.rx_rate = rx_rate;

            // If new TX rate is slower
            if (byte_tx_time[tx_rate] > byte_tx_time[con_par->tx_rate])
            {
                // Update max TX length
                lld_con_tx_len_update(link_id, tx_rate, con_par->interval);
            }

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            ASSERT_INFO_FORCE(0, con_par->instant_proc.type, 0);
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_tx_len_update_for_intv(uint8_t link_id, uint16_t interval)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_LEN_UPDATE_FOR_INTV_2_FUNC_BIT, uint8_t, link_id, interval);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // If new interval is smaller, and PHY is coded
        if ((con_par->interval > 4 * interval) && (con_par->tx_rate >= CO_RATE_125KBPS))
        {
            // Update max TX length as per future interval
            lld_con_tx_len_update(link_id, con_par->tx_rate, 4 * interval);
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_tx_len_update_for_rate(uint8_t link_id, uint8_t tx_rate)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_LEN_UPDATE_FOR_RATE_2_FUNC_BIT, uint8_t, link_id, tx_rate);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // If TX rate is the connection rate
        if (tx_rate == CO_RATE_UNDEF)
        {
            // Update max TX length as per current rate
            lld_con_tx_len_update(link_id, con_par->tx_rate, con_par->interval);
        }
        // If new TX rate is slower
        else if (byte_tx_time[tx_rate] > byte_tx_time[con_par->tx_rate])
        {
            // Update max TX length as per future rate
            lld_con_tx_len_update(link_id, tx_rate, con_par->interval);
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint16_t lld_con_event_counter_get(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_EVENT_COUNTER_GET_1_FUNC_BIT, uint16_t, link_id);

    uint16_t evt_cnt = 0;

    GLOBAL_INT_DISABLE();
    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        evt_cnt = con_par->evt_cnt;
    }
    GLOBAL_INT_RESTORE();

    return (evt_cnt);
}

void lld_con_tx_enc(uint8_t link_id, bool enable)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_ENC_2_FUNC_BIT, link_id, enable);

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

        em_ble_linkcntl_txcrypt_en_setf(cs_idx, enable);

        SETB(con_par->link_info, LLD_CON_TX_ENCRYPT, enable);

#if (BLE_ISO_MODE_0)
        SETB(con_par->link_info, LLD_CON_MIC_LESS_USED, GETB(con_par->link_info, LLD_CON_MIC_LESS) & enable);
#endif //(BLE_ISO_MODE_0)

        // Update event time parameters
        lld_con_evt_time_update(link_id);
    }

    GLOBAL_INT_RESTORE();
}

void lld_con_rx_enc(uint8_t link_id, bool enable)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_RX_ENC_2_FUNC_BIT, link_id, enable);

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

        em_ble_linkcntl_rxcrypt_en_setf(cs_idx, enable);

        SETB(con_par->link_info, LLD_CON_RX_ENCRYPT, enable);

#if (BLE_ISO_MODE_0)
        // Store the information for packet RX/TX management (ensure to remove/add MIC length MIC-less packets)
        SETB(con_par->link_info, LLD_CON_MIC_LESS_USED, GETB(con_par->link_info, LLD_CON_MIC_LESS) & enable);

        if (GETB(con_par->link_info, LLD_CON_MIC_LESS_USED))
        {
            DBG_SWDIAG(AM0, CRYPT, 1);

            // Prepare CS
            em_ble_linkcntl_crypt_mode_setf(cs_idx, ENC_MODE_EVT_CNT);
            em_ble_linkcntl_mic_mode_setf(cs_idx, ENC_MIC_LESS);

            if (con_par->am0.enable)
            {
                // Set maximum ISO receive payload length authorized (no MIC at all)
                em_ble_rxmaxbuf_isorxmaxbuf_setf(cs_idx, con_par->am0.rx.pdu_size);
            }

            DBG_SWDIAG(AM0, CRYPT, 0);
        }
#endif //(BLE_ISO_MODE_0)

        // Update event time parameters
        lld_con_evt_time_update(link_id);
    }

    GLOBAL_INT_RESTORE();
}

void lld_con_enc_key_load(uint8_t link_id, struct ltk *key, struct initialization_vector *iv)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_ENC_KEY_LOAD_1_FUNC_BIT, link_id, key, iv);

    if (lld_con_env[link_id] != NULL)
    {
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);
        uint8_t reg_idx;
        uint8_t key_idx;

        // Initialize the CCM counters
        em_ble_txccmpktcnt0_set(cs_idx, 0);
        em_ble_txccmpktcnt1_set(cs_idx, 0);
        em_ble_txccmpktcnt2_set(cs_idx, 0);
        em_ble_rxccmpktcnt0_set(cs_idx, 0);
        em_ble_rxccmpktcnt1_set(cs_idx, 0);
        em_ble_rxccmpktcnt2_set(cs_idx, 0);

        // Configure Initialization vector in CS
        for (reg_idx = 0, key_idx = 0; reg_idx < EM_BLE_IV_COUNT; reg_idx++)
        {
            em_ble_iv_setf(cs_idx, reg_idx, co_read16p(&(iv->vect[key_idx])));
            key_idx += 2;
        }

        // Configure Session key in CS
        for (reg_idx = 0, key_idx = (KEY_LEN - 1); reg_idx < EM_BLE_SK_COUNT; reg_idx++)
        {
            // encrypted_data is LSB first and  SK is MSB first
            em_ble_sk_setf(cs_idx, reg_idx, (key->ltk[key_idx - 1] << 8) | key->ltk[key_idx]);
            key_idx -= 2;
        }
    }
}

int8_t lld_con_current_tx_power_get(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_CURRENT_TX_POWER_GET_1_FUNC_BIT, int8_t, link_id);

    int8_t tx_pwr = 0;
    uint8_t rf_mod;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

        rf_mod = em_ble_cntl_rf_modtype_getf(cs_idx);
        // Get TX power from CS
        if (rf_mod == MOD_IQ)
        {
            tx_pwr = em_ble_filtpol_ralcntl_iqval_getf(cs_idx);
        }
        else
        {
            tx_pwr = em_ble_txrxcntl_txpwr_getf(cs_idx);
        }
        // Convert CS config to radiative dBm
        tx_pwr = lld_tx_power_dbm_get(tx_pwr, rf_mod);
    }

    GLOBAL_INT_RESTORE();

    return (tx_pwr);
}

int8_t lld_con_rssi_get(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_RSSI_GET_2_FUNC_BIT, int8_t, link_id);

    int8_t rssi = 0;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Get the last RSSI value
        rssi = con_par->last_rssi;
    }

    GLOBAL_INT_RESTORE();

    return (rssi);
}

#if BLE_PWR_CTRL

int8_t lld_con_tx_power_get(uint8_t link_id, uint8_t phy)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_POWER_GET_2_FUNC_BIT, int8_t, link_id, phy);

    int8_t tx_pwr = 0;

    if (lld_con_env[link_id] != NULL)
    {
        uint8_t rate = co_phypwr_to_rate[phy];

        ASSERT_ERR(rate < CO_RATE_MAX);

        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Convert to dbM
        tx_pwr = (int8_t)lld_tx_power_dbm_get(con_par->tx_pwr_lvl[rate], con_par->tx_pwr_type[rate]);
    }

    return (tx_pwr);
}

int8_t lld_con_tx_power_adj(uint8_t link_id, uint8_t phy, int8_t delta)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TX_POWER_ADJ_2_FUNC_BIT, int8_t, link_id, phy, delta);

    int8_t tx_pwr = 0;

    if (lld_con_env[link_id] != NULL)
    {
        uint8_t rate = co_phypwr_to_rate[phy];
        uint8_t type;

        ASSERT_ERR(rate < CO_RATE_MAX);

        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        if (PWR_CTRL_DELTA_MAX == delta)
        {
            // Set power level to max
            con_par->tx_pwr_lvl[rate] = rwip_rf.txpwr_max;
            type = rwip_rf.txpwr_max_mod;
        }
        else
        {
            type = con_par->tx_pwr_type[rate];
            // Determine nearest higher transceiver power level
            tx_pwr = (int8_t)rwip_rf.txpwr_dbm_get(con_par->tx_pwr_lvl[rate], MOD_GFSK, type) + delta;
            // Fetch value equal to or higher than requested (ensures shall not reduce power lower than a peer's specified APR value)
            con_par->tx_pwr_lvl[rate] = rwip_rf.txpwr_cs_get(tx_pwr, TXPWR_CS_HIGHER, type);
        }

        // Radiative transmit power corresponding to the new power level
        tx_pwr = (int8_t)lld_tx_power_dbm_get(con_par->tx_pwr_lvl[rate], type);
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    return tx_pwr;
}

uint8_t lld_con_apr_get(uint8_t link_id, uint8_t rx_rate)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_APR_GET_1_FUNC_BIT, uint8_t, link_id, rx_rate);

    uint8_t apr = PWR_CTRL_APR_UNKNOWN;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Check there is an RSSI value measured at this rate
        if ((int8_t)LLD_RSSI_UNDEF != con_par->av_rssi[rx_rate])
        {
            // Determine the APR from the averaged RSSI value
            int8_t rssi = con_par->av_rssi[rx_rate];
            apr = (rssi > rwip_rf.rssi_low_thr) ? (rssi - rwip_rf.rssi_low_thr) : 0;

            /* When a device reports a non-zero APR , it should wait at least two connection intervals to see if the peer
               device has made use of it to change its local transmit power before initiating a
               new Power Control Request procedure to request a remote transmit power level change. */
            if (apr)
            {
                con_par->tx_pwr_evt_cnt = PWR_CTRL_EVT_CNT_MIN;
            }
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();

    return (apr);
}

uint8_t lld_con_rx_rate_get(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_RX_RATE_GET_2_FUNC_BIT, uint8_t, link_id);

    int8_t rx_rate = 0;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Get the active rx rate value
        rx_rate = con_par->rx_rate;
    }

    GLOBAL_INT_RESTORE();

    return (rx_rate);
}

uint8_t lld_con_tx_rate_get(uint8_t link_id, uint8_t *tx_rate)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_3_PATCH_TYPE, LLD_CON_TX_RATE_GET_3_FUNC_BIT, uint8_t, link_id, tx_rate);

    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Get the active rx rate value
        *tx_rate = con_par->tx_rate;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_tx_pwr_lvl_get(uint8_t link_id, uint8_t tx_rate, uint8_t *pwr_lvl)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CON_3_PATCH_TYPE, LLD_CON_TX_PWR_LVL_GET_3_FUNC_BIT, uint8_t, link_id, tx_rate, pwr_lvl);

    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    *pwr_lvl = rwip_rf.txpwr_max;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Read the transmit power level for the specified rate
        *pwr_lvl = con_par->tx_pwr_lvl[tx_rate];

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

void lld_con_remote_tx_pwr_set(uint8_t link_id, uint8_t tx_rate, int8_t tx_pwr, uint8_t flags)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_REMOTE_TX_PWR_SET_2_FUNC_BIT, link_id, tx_rate, tx_pwr, flags);

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        if ((LLD_PWR_FLGS_UNKNOWN != flags) || (con_par->remote_tx_pwr[tx_rate] != tx_pwr))
        {
            // Update flags if known, or to unknown if tx_pwr changed
            con_par->remote_tx_pwr_flags[tx_rate] = flags;
        }

        if (con_par->remote_tx_pwr[tx_rate] != tx_pwr)
        {
            // Force restart on RSSI averaging
            con_par->av_rssi[tx_rate] = LLD_RSSI_UNDEF;
        }

        // Update the remote transmit power
        con_par->remote_tx_pwr[tx_rate] = tx_pwr;

        // Power control event count after receiving a power control response
        con_par->tx_pwr_evt_cnt = PWR_CTRL_EVT_CNT_MIN;
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();
}

int8_t lld_con_remote_tx_pwr_get(uint8_t link_id, uint8_t rx_rate, uint8_t *flags)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_REMOTE_TX_PWR_GET_2_FUNC_BIT, uint8_t, link_id, rx_rate, flags);

    int8_t tx_pwr = BLE_PWR_UNKNOWN;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Read the remote transmit power for the specified rate & associated flags
        tx_pwr = con_par->remote_tx_pwr[rx_rate];
        *flags = con_par->remote_tx_pwr_flags[rx_rate];
    }

    GLOBAL_INT_RESTORE();

    return tx_pwr;
}

void lld_con_path_loss_monitor_config(uint8_t link_id, uint8_t hi_thr, uint8_t hi_hyst,
                                      uint8_t lo_thr, uint8_t lo_hyst, uint16_t min_evt)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_PATH_LOSS_MONITOR_CONFIG_2_FUNC_BIT, 6, link_id, hi_thr, hi_hyst, lo_thr, lo_hyst, min_evt);

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Configure/Update path loss monitoring parameters
        con_par->path_loss_hi_thr = hi_thr;
        con_par->path_loss_hi_hyst = hi_hyst;
        con_par->path_loss_lo_thr = lo_thr;
        con_par->path_loss_lo_hyst = lo_hyst;
        con_par->path_loss_evt_min = min_evt;

        // Initialize path loss measurements
        con_par->path_loss_evt_cnt = 0;
        con_par->path_loss_zone = LLD_PATH_LOSS_UNDEF;
        con_par->new_path_loss_zone = LLD_PATH_LOSS_UNDEF;
    }

    GLOBAL_INT_RESTORE();
}

bool lld_con_path_loss_monitor_en(uint8_t link_id, bool en)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_PATH_LOSS_MONITOR_EN_2_FUNC_BIT, bool, link_id, en);

    bool activated = false;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Configure path loss monitoring enable
        con_par->path_loss_monitoring = en;

        // Path loss monitoring is activated when enabled and the remote transmit power is known
        activated = en && (BLE_PWR_UNKNOWN != con_par->remote_tx_pwr[con_par->rx_rate]);
    }

    GLOBAL_INT_RESTORE();

    return (activated);
}

void lld_con_rssi_update(uint8_t link_id, uint8_t rx_rate, int8_t rssi)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_RSSI_UPDATE_2_FUNC_BIT, link_id, rx_rate, rssi);

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        int16_t av_rssi;

        // RSSI average calculation
        if ((int8_t)LLD_RSSI_UNDEF != con_par->av_rssi[rx_rate])
        {
            av_rssi = con_par->av_rssi[rx_rate]; // convert to 16bit signed for up-scaling

            // Determine average based on a weighted history of previous samples
            av_rssi = ((av_rssi << RW_RSSI_AV_WEIGHT) - av_rssi + rssi + (1 << (RW_RSSI_AV_WEIGHT - 1))) >> RW_RSSI_AV_WEIGHT;
        }
        else
        {
            // No average accumulated yet, so take the current value
            av_rssi = (int16_t)rssi;
            // Ensure wait a minumum event count for RSSI average accumulation
            con_par->tx_pwr_evt_cnt = PWR_CTRL_EVT_CNT_MIN;
        }

        // Update the average for this rate
        con_par->av_rssi[rx_rate] = av_rssi;

        if (!con_par->tx_pwr_evt_cnt)
        {
            // Ensure radio driver has defined RSSI thresholds
            ASSERT_ERR(rwip_rf.rssi_low_thr && rwip_rf.rssi_high_thr);

            // Check if RSSI average is outside golden receiver range, and peer has not indicated a max/min which would prevent the adjustment
            if (((av_rssi < rwip_rf.rssi_low_thr) && !GETB(con_par->remote_tx_pwr_flags[rx_rate], BLE_PWR_CTRL_MAX))
                    || ((av_rssi > rwip_rf.rssi_high_thr) && !GETB(con_par->remote_tx_pwr_flags[rx_rate], BLE_PWR_CTRL_MIN)))
            {
                // Indicate a power control request
                struct lld_con_pwr_ctrl_ind *msg =  KE_MSG_ALLOC(LLD_CON_PWR_CTRL_IND, KE_BUILD_ID(TASK_LLC, link_id), TASK_NONE, lld_con_pwr_ctrl_ind);
                // Adjustment to APR target within the golden receive range.
                msg->delta = rwip_rf.rssi_low_thr - av_rssi + RW_RSSI_APR_TARGET;
                msg->rx_rate = rx_rate;
                ke_msg_send(msg);

                // Power control event count wait (the event decount restarts on processing of the response)
                con_par->tx_pwr_evt_cnt = LLD_CON_PWR_CTRL_EVT_CNT_WAIT;
            }
        }
    }
}

#endif // BLE_PWR_CTRL

uint16_t lld_con_offset_get(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_OFFSET_GET_1_FUNC_BIT, uint16_t, link_id);

    uint16_t con_offset = 0;

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Get the latest connection offset
        con_offset = CO_MOD(con_par->next_ts, con_par->interval);
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();

    return (con_offset);
}

#if (BLE_PERIPHERAL)
void lld_con_pref_slave_latency_set(uint8_t link_id, uint16_t latency)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_PREF_SLAVE_LATENCY_SET_2_FUNC_BIT, link_id, latency);

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Use preferred latency if the connection parameters allow it
        con_par->latency = latency;
        lld_con_max_lat_calc(link_id);
    }

    GLOBAL_INT_RESTORE();
}

void lld_con_pref_slave_evt_dur_set(uint8_t link_id, uint16_t duration, bool single_tx)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_PREF_SLAVE_EVT_DUR_SET_2_FUNC_BIT, link_id, duration, single_tx);

    GLOBAL_INT_DISABLE();

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Use preferred latency only if lower than the current latency
        con_par->evt_duration = duration;

        // Set single TX bit
        SETB(con_par->link_info, LLD_CON_SINGLE_TX, single_tx);

        // Update event time parameters
        lld_con_evt_time_update(link_id);
    }

    GLOBAL_INT_RESTORE();
}
#endif // (BLE_PERIPHERAL)

#if (BLE_ISO_MODE_0)

uint8_t lld_con_am0_use_mic_less(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_AM0_USE_MIC_LESS_1_FUNC_BIT, uint8_t, link_id);

    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    GLOBAL_INT_DISABLE();

    // Get connection parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    if (con_par != NULL)
    {
        // CS Index
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

        // Prepare connection for MIC-less encryption
        SETB(con_par->link_info, LLD_CON_MIC_LESS, 1);

        // Prepare connection for handling LEAS packets
        em_ble_linkcntl_nullrxllidflt_setf(cs_idx, 1);

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_am0_start(uint8_t link_id, uint8_t iso_channel, struct lld_con_am0_params *params)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_AM0_START_1_FUNC_BIT, uint8_t, link_id, iso_channel, params);

    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    GLOBAL_INT_DISABLE();
    DBG_SWDIAG(AM0, START, 1);

    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    if (con_par != NULL)
    {
        // CS Index
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);
        // AM0 structure
        struct lld_con_am0_info *p_am0 = &(con_par->am0);
        // ISO Interval = 2*con_interval in us = con_interval in hus
        uint32_t iso_interval_us = con_par->interval * HALF_SLOT_SIZE;
        // Counter
        uint8_t  cnt;

        // Data information
        struct lld_am0_data *p_tx  = &(con_par->am0.tx);
        struct lld_am0_data *p_rx  = &(con_par->am0.rx);

        // Set parameters
        p_am0->enable         = true;
        p_am0->prestart       = (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE);
        p_am0->reload_buffer  = true;
        p_am0->channel        = iso_channel;
        p_am0->evt_parity     = 0;

        p_am0->anchor_bts = rwip_bt_time_to_bts(con_par->last_sync_ts, con_par->last_sync_bit_off);

        p_tx->pdu_size = params->tx_size;
        p_rx->pdu_size = params->rx_size;

        p_tx->curr_iso_idx = 0;
        p_rx->curr_iso_idx = 0;

        p_rx->buf_nb      = BLE_NB_RX_ISO_DESC_BUF_PER_AM0_CHAN;
        p_tx->buf_nb      = BLE_NB_TX_ISO_DESC_BUF_PER_AM0_CHAN;

        // AM0 should be configured as MIC-less
        ASSERT_ERR(GETB(con_par->link_info, LLD_CON_MIC_LESS));

        // If encryption already enabled
        if (GETB(con_par->link_info, LLD_CON_TX_ENCRYPT) || GETB(con_par->link_info, LLD_CON_RX_ENCRYPT))
        {
            // AM0 configured MIC-less
            SETB(con_par->link_info, LLD_CON_MIC_LESS_USED, 1);
            ASSERT_ERR(em_ble_linkcntl_crypt_mode_getf(cs_idx) == ENC_MODE_EVT_CNT);

            em_ble_linkcntl_crypt_mode_setf(cs_idx, ENC_MODE_EVT_CNT);
            em_ble_linkcntl_mic_mode_setf(cs_idx, ENC_MIC_LESS);
        }

        // Initialize ISO link control
        em_ble_isolinkcntl_pack(cs_idx, /*channellbl*/iso_channel, /*streamlbl*/0, /*isosyncmode*/1, /*isosyncen*/1, /*isotype*/LLD_ISO_MODE_0);

        // The connEventCounterOffset must be set to zero (R33, 59.2.5)
        em_ble_evtcnt_offset0_setf(cs_idx, 0);
        em_ble_evtcnt_offset1_setf(cs_idx, 0);
        em_ble_evtcnt_offset2_setf(cs_idx, 0);

        // Initialize sub event and flush time
        em_ble_isoevtcntl_pack(cs_idx, /*flushcnt*/ 0, /*subevtcnt*/ 0);

        // Set maximum ISO receive payload length authorized
        em_ble_rxmaxbuf_isorxmaxbuf_setf(cs_idx, p_rx->pdu_size);

        // ISO TX/RX control
        em_ble_isotxrxcntl_set(cs_idx, 0);

        // Disable unused control
        em_ble_hopcs2cntl0_set(cs_idx, 0);
        em_ble_hopcs2cntl1_set(cs_idx, 0);
        em_ble_txheadercntl_set(cs_idx, 0);

        for (cnt = 0; cnt < p_tx->buf_nb; cnt++)
        {
            // Allocate TX ISO descriptors
            uint8_t tx_desc_idx = ble_util_isodesc_alloc();
            p_tx->iso_descs_idx[cnt] = tx_desc_idx;

            // Allocate TX ISO buffers
            p_tx->iso_bufs_idx[cnt] = (p_tx->pdu_size > 0) ? ble_util_buf_iso_alloc() : 0;

            // Initialize TX ISO buffer
            em_ble_txisobufsetup_pack(p_tx->iso_bufs_idx[cnt], /*mute*/ false, /*llid*/ LLID_UNFRAMED_END, /*length*/ 0);

            // Configure TX descriptors
            em_ble_txisoptr_pack(tx_desc_idx, /*txdone*/1, /*txsent*/0, /*nextptr*/0); // Mark descriptors as not ready
            em_ble_txisocnt0_txpld_cnt0_setf(tx_desc_idx, 0);
            em_ble_txisocnt1_txpld_cnt1_setf(tx_desc_idx, 0);
            em_ble_txisocnt2_txpld_cnt2_setf(tx_desc_idx, 0);
            em_ble_txisocnt2_txflushinstant_setf(tx_desc_idx, 1);  // Mark flush instant to the retransmission sub-event (which is retx slot)
            em_ble_txisophm0_pack(tx_desc_idx, /*rfu*/0, /*txmd*/0, /*txsn*/0, /*txnesn*/0);
            em_ble_txisobufptr_setf(tx_desc_idx, ((p_tx->pdu_size > 0) ? (ble_util_buf_iso_emptr_get(p_tx->iso_bufs_idx[cnt]) >> 2) : 0));
        }

        for (cnt = 0; cnt < p_rx->buf_nb; cnt++)
        {
            // Allocate RX ISO descriptors
            uint8_t rx_desc_idx = ble_util_isodesc_alloc();
            p_rx->iso_descs_idx[cnt] = rx_desc_idx;

            // Allocate RX ISO buffers
            p_rx->iso_bufs_idx[cnt] = (p_rx->pdu_size > 0) ? ble_util_buf_iso_alloc() : 0;

            // Configure RX descriptors
            em_ble_rxisoptr_pack(rx_desc_idx, /*rxdone*/1, /*nexptr*/0); // Mark descriptors as not ready
            em_ble_rxisocnt0_rxpld_cnt0_setf(rx_desc_idx, 0);
            em_ble_rxisocnt1_rxpld_cnt1_setf(rx_desc_idx, 0);
            em_ble_rxisocnt2_rxpld_cnt2_setf(rx_desc_idx, 0);
            em_ble_rxisocnt2_rxflushinstant_setf(rx_desc_idx, 1); // Mark flush instant to the retransmission sub-event (which is retx slot)
            em_ble_rxisobufptr_setf(rx_desc_idx, ((p_rx->pdu_size > 0) ? (ble_util_buf_iso_emptr_get(p_rx->iso_bufs_idx[cnt]) >> 2) : 0));
            em_ble_rxisobufsetup_invl_setf(p_rx->iso_bufs_idx[cnt], LLD_ISO_INVL_SYNC_ERR);
        }

        status = CO_ERROR_NO_ERROR;

        // Start ISOAL for transmission
        if (p_tx->pdu_size > 0)
        {
            status = lld_isoal_start(/*activity identifier*/link_id,
                     /*direction*/ISO_SEL_TX,
                     /*channel handle*/p_am0->channel,
                     /*sdu_interval*/iso_interval_us,
                     /*transport latency*/iso_interval_us,
                     /*sync offset*/ 0,
                     /*max_sdu*/p_tx->pdu_size,
                     /*iso_interval*/iso_interval_us,
                     /*BN*/1,
                     /*max_pdu*/p_tx->pdu_size,
                     /*framing*/ISO_UNFRAMED_MODE,
                     /*mic_present*/false,
                     /*am0*/true);
        }

        // Start ISOAL for reception
        if ((p_rx->pdu_size > 0) && (CO_ERROR_NO_ERROR == status))
        {
            status = lld_isoal_start(/*activity identifier*/link_id,
                     /*direction*/ISO_SEL_RX,
                     /*channel handle*/p_am0->channel,
                     /*sdu_interval*/iso_interval_us,
                     /*transport latency*/iso_interval_us,
                     /*sync offset*/ 0,
                     /*max_sdu*/p_rx->pdu_size,
                     /*iso_interval*/iso_interval_us,
                     /*BN*/1,
                     /*max_pdu*/p_rx->pdu_size,
                     /*framing*/ISO_UNFRAMED_MODE,
                     /*mic_present*/false,
                     /*am0*/true);
        }
    }

    DBG_SWDIAG(AM0, START, 0);
    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_con_am0_stop(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_AM0_STOP_1_FUNC_BIT, uint8_t, link_id);

    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    GLOBAL_INT_DISABLE();
    DBG_SWDIAG(AM0, STOP, 1);

    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    if (con_par != NULL)
    {
        if (con_par->am0.enable)
        {
            // Data information
            struct lld_am0_data *p_tx  = &(con_par->am0.tx);
            struct lld_am0_data *p_rx  = &(con_par->am0.rx);
            // Counter
            uint8_t  cnt;

            // stop ISOAL
            lld_isoal_stop(link_id, CO_ERROR_NO_ERROR);

            for (cnt = 0; cnt < p_tx->buf_nb; cnt++)
            {
                // Free TX ISO descriptors
                ble_util_isodesc_free(p_tx->iso_descs_idx[cnt]);
                // Free TX ISO buffers
                if (p_tx->pdu_size > 0)
                {
                    ble_util_buf_iso_free(p_tx->iso_bufs_idx[cnt]);
                    p_tx->pdu_size = 0;
                }
            }

            for (cnt = 0; cnt < p_rx->buf_nb; cnt++)
            {
                // Free RX ISO descriptors
                ble_util_isodesc_free(p_rx->iso_descs_idx[cnt]);
                // Free TX ISO buffers
                if (p_rx->pdu_size > 0)
                {
                    ble_util_buf_iso_free(p_rx->iso_bufs_idx[cnt]);
                    p_rx->pdu_size = 0;
                }
            }

            // Set parameters
            con_par->am0.enable = false;
            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
    }

    DBG_SWDIAG(AM0, STOP, 0);
    GLOBAL_INT_RESTORE();

    return (status);
}
#endif //(BLE_ISO_MODE_0)

void lld_con_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_INIT_1_FUNC_BIT, is_reset);

    if (is_reset)
    {
        for (int8_t link_id = (BLE_ACTIVITY_MAX - 1); link_id >= 0 ; link_id--)
        {
            // Check if connection is ongoing
            if (lld_con_env[link_id] != NULL)
            {
                // clean-up allocated memory
                lld_con_cleanup(link_id, false, CO_ERROR_CON_TERM_BY_LOCAL_HOST);
            }
        }
    }
    // No break

    // Initialize environment
    memset(&lld_con_env, 0, sizeof(lld_con_env));
}

#if (BLE_CIS)
uint8_t lld_con_info_for_cis_get(uint8_t link_id, uint16_t evt_cnt, struct lld_con_info_for_cis *info)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_INFO_FOR_CIS_GET_1_FUNC_BIT, uint8_t, link_id, evt_cnt, info);

    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    if (con_par != NULL)
    {
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

        uint32_t con_interval  = 0;
        uint32_t con_timestamp = 0;
        uint16_t con_evt_cnt   = 0;
        int16_t  evt_cnt_diff;

        con_evt_cnt          = con_par->evt_cnt + con_par->evt_inc;
        con_interval         = con_par->interval;
        con_timestamp        = con_par->next_ts;

        // Compute connection timestamp according to provided connection event counter
        evt_cnt_diff         = BLE_UTIL_EVT_CNT_DIFF(evt_cnt, con_evt_cnt);
        info->ref_timestamp  = CLK_ADD_2(con_timestamp, evt_cnt_diff * con_interval);

        info->last_sync_ts   = con_par->last_sync_ts;
        info->ref_bit_off    = con_par->last_sync_bit_off;
        info->sync_drift_acc = con_par->sync_drift_acc;
        info->master_sca     = con_par->master_sca;

        // load encryption settings
        info->encrypted      = GETB(con_par->link_info, LLD_CON_TX_ENCRYPT);

        if (info->encrypted)
        {
            uint8_t reg_idx;
            uint8_t key_idx;

            // Retrieve initialization vector
            for (reg_idx = 0, key_idx = 0; reg_idx < EM_BLE_IV_COUNT; reg_idx++)
            {
                co_write16p(&(info->iv.vect[key_idx]), em_ble_iv_getf(cs_idx, reg_idx));
                key_idx += 2;
            }

            // Retrieve session key
            for (reg_idx = 0, key_idx = (KEY_LEN - 1); reg_idx < EM_BLE_SK_COUNT; reg_idx++)
            {
                uint16_t key = em_ble_sk_getf(cs_idx, reg_idx);

                info->sk.ltk[key_idx - 1] = (key >> 8) & 0xFF;
                info->sk.ltk[key_idx]   = (key >> 0) & 0xFF;

                key_idx -= 2;
            }
        }

        // Retrieve channel map
        co_write16p(&(info->ch_map.map[0]), em_ble_chmap0_get(cs_idx));
        co_write16p(&(info->ch_map.map[2]), em_ble_chmap1_get(cs_idx));
        co_write16p(&(info->ch_map.map[4]), em_ble_chmap2_get(cs_idx));

        // Retrieve CRC Init
        info->crcinit.crc[0] = (em_ble_crcinit0_get(cs_idx) >> 0) & 0xFF;
        info->crcinit.crc[1] = (em_ble_crcinit0_get(cs_idx) >> 8) & 0xFF;
        info->crcinit.crc[2] = (em_ble_crcinit1_get(cs_idx) >> 0) & 0xFF;

        status = CO_ERROR_NO_ERROR;
    }

    return (status);
}

uint32_t lld_con_sup_to_get(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_SUP_TO_GET_2_FUNC_BIT, uint32_t, link_id);

    // Point to connection parameters
    struct lld_con_env_tag *p_con_par = lld_con_env[link_id];
    uint32_t sup_to = 0;

    if (p_con_par != NULL)
    {
        sup_to = p_con_par->timeout;
    }

    return (sup_to);
}

#if (BLE_PERIPHERAL)
void lld_con_sync_time_update(uint8_t link_id, uint32_t last_sync_ts, int32_t sync_drift_acc)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_SYNC_TIME_UPDATE_2_FUNC_BIT, link_id, last_sync_ts, sync_drift_acc);

    // Point to parameters
    struct lld_con_env_tag *con_par = lld_con_env[link_id];

    // Check that connection exists
    if (con_par != NULL)
    {
        // Get drift since last drift value
        int32_t drift_val = sync_drift_acc - con_par->sync_drift_acc;
        // Connection anchor timestamp - half microseconds part
        int32_t con_bit_off = con_par->last_sync_bit_off + drift_val;

        // Update connection event position
        if (con_bit_off < 0)
        {
            con_bit_off = -con_bit_off - 1;
            con_par->next_ts = CLK_SUB(con_par->next_ts, 1 + (con_bit_off / HALF_SLOT_SIZE));
            con_par->last_sync_bit_off = HALF_SLOT_SIZE - 1 - CO_MOD(con_bit_off, HALF_SLOT_SIZE);
        }
        else
        {
            con_par->next_ts = CLK_ADD_2(con_par->next_ts, con_bit_off / HALF_SLOT_SIZE);
            con_par->last_sync_bit_off = CO_MOD(con_bit_off, HALF_SLOT_SIZE);
        }

        con_par->last_sync_ts = last_sync_ts;
        con_par->sync_drift_acc = sync_drift_acc;

        // If event is scheduled, update timings, assuming the sync window reduces thanks to new synchronization infos
        if (con_par->state == CON_EVT_WAIT)
        {
            // Get event parameters
            struct sch_arb_elt_tag *p_evt = &con_par->evt;

            // Expected next anchor point (half microsecond part)
            int32_t bit_off = con_par->last_sync_bit_off;
            // Expected next anchor point (half slot part)
            uint32_t target_clock = con_par->next_ts;

            // Initialize RX window size
            uint32_t rx_win_size = 0;

            // Add transmitWindowSize if used for synchronizing to the master at connection update instant
            if ((con_par->instant_proc.type != INSTANT_PROC_CON_PAR_UPD) || con_par->instant_proc.data.con_par.params_updated)
            {
                rx_win_size += con_par->winsize * HALF_SLOT_SIZE;
            }

            // Compute RX timings
            rx_win_size = lld_rx_timing_compute(last_sync_ts, &target_clock, &bit_off, con_par->master_sca, con_par->rx_rate, rx_win_size);

            // Store the RX window size
            con_par->sync_win_size = rx_win_size;

            // Update the activity arbitration element
            p_evt->time.hs      = target_clock;
            p_evt->time.hus     = bit_off;
            p_evt->duration_min = con_par->duration_min + rx_win_size / 2;
        }
    }
}
#endif //(BLE_PERIPHERAL)
#endif // (BLE_CIS)


#if BLE_CON_CTE_REQ
void lld_con_cte_rx_en(uint8_t link_id, uint8_t slot_dur, uint8_t switching_pattern_len)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_CTE_RX_EN_1_FUNC_BIT, link_id, slot_dur, switching_pattern_len);

    if (lld_con_env[link_id] != NULL)
    {
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

        // Set the Antenna ID pointer
        em_ble_rxdfantswptr_set(cs_idx, EM_BLE_RX_ANTENNA_ID_OFFSET >> 2);

        // Set the length of the switching pattern
        em_ble_rxdfantpattcntl_rx_ant_patt_length_setf(cs_idx, switching_pattern_len);

        // max I&Q samples per CTE
        em_ble_crcinit1_rxmaxctebuf_setf(cs_idx, LLD_MAX_CTE_IQ_SAMPLES);

        // max sampled CTE per event - 0x1F - no limit
        em_ble_rxdfantpattcntl_max_samp_cte_setf(cs_idx, 0x1F);

        // Set RXDFCNTL (both AoA and AoD supported)
        em_ble_rxdfcntl_pack(cs_idx, /*DFRSPEN*/ 0, /*DFSWCNTL*/ slot_dur, /*DFSAMPCNTL*/ slot_dur, /*DFTYPE*/ 0x3, /*DFFILTEREN*/ 0, /*DFEN*/ 1);
    }
}

void lld_con_cte_rx_dis(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_CTE_RX_DIS_1_FUNC_BIT, link_id);

    if (lld_con_env[link_id] != NULL)
    {
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

        // Disable antenna switching
        em_ble_rxdfantpattcntl_rx_ant_patt_length_setf(cs_idx, 0);
        // By default, CTE reception is enabled, IQ sampling is disabled
        em_ble_rxdfcntl_set(cs_idx, EM_BLE_DFEN_BIT);
    }
}
#endif // BLE_CON_CTE_REQ

#if BLE_CON_CTE_RSP
void lld_con_cte_tx_ant_switch_config(uint8_t link_id, uint8_t switching_pattern_len, uint8_t cte_types)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_CTE_TX_ANT_SWITCH_CONFIG_1_FUNC_BIT, link_id, switching_pattern_len, cte_types);

    if (lld_con_env[link_id] != NULL)
    {
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(link_id);

        // If multiple antennae are used
        if (cte_types & (CTE_TYPES_AOD_1US_BIT | CTE_TYPES_AOD_2US_BIT))
        {
            // Set the Antenna ID pointer
            em_ble_txdfantswptr_set(cs_idx, EM_BLE_TX_ANTENNA_ID_OFFSET >> 2);

            // Set the length of the switching pattern
            em_ble_txdfantpattcntl_tx_ant_patt_length_setf(cs_idx, switching_pattern_len);
        }
        else
        {
            // Disable antenna switching
            em_ble_txdfantpattcntl_tx_ant_patt_length_setf(cs_idx, 0);
        }
    }
}

void lld_con_cte_tx_param_set(uint8_t link_id, uint8_t cte_len, uint8_t cte_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_CTE_TX_PARAM_SET_1_FUNC_BIT, link_id, cte_len, cte_type);

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        con_par->cte_len = cte_len;
        con_par->cte_type = cte_type;
    }
}
#endif // BLE_CON_CTE_RSP

void lld_con_peer_sca_set(uint8_t link_id, uint8_t peer_sca)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_PEER_SCA_SET_2_FUNC_BIT, link_id, peer_sca);

    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        if (GETF(con_par->link_info, LLD_CON_ROLE) == SLAVE_ROLE)
        {
            con_par->master_sca = co_sca2ppm[peer_sca];
        }
    }
}

uint8_t lld_con_time_get(uint8_t link_id, uint16_t *con_evt_cnt, uint32_t *con_evt_time_hs, uint16_t *con_evt_time_hus)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_CON_2_PATCH_TYPE, LLD_CON_TIME_GET_2_FUNC_BIT, uint8_t, link_id, con_evt_cnt, con_evt_time_hs, con_evt_time_hus);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();
    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        if (GETB(con_par->link_info, LLD_CON_ESTABLISHED))
        {
            // Get time of current event
            *con_evt_cnt = con_par->last_sync_evt_cnt;
            *con_evt_time_hs = con_par->anchor_ts;

            if (con_evt_time_hus != NULL)
            {
                *con_evt_time_hus = con_par->last_sync_bit_off;
            }
        }
        else
        {
            *con_evt_cnt = con_par->evt_cnt + con_par->evt_inc;
            *con_evt_time_hs = con_par->next_ts;

            if (con_evt_time_hus != NULL)
            {
                *con_evt_time_hus = con_par->next_bit_off;
            }
        }

        status = CO_ERROR_NO_ERROR;
    }
    GLOBAL_INT_RESTORE();

    return (status);
}

#if (AUDIO_SYNC_SUPPORT)
uint8_t lld_con_event_tx_time_get(uint8_t link_id, uint16_t con_evt_cnt,  uint32_t *con_evt_time_hs, uint16_t *con_evt_time_hus)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_CON_1_PATCH_TYPE, LLD_CON_EVENT_TX_TIME_GET_1_FUNC_BIT, uint8_t, link_id, con_evt_cnt, con_evt_time_hs, con_evt_time_hus);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();
    if (lld_con_env[link_id] != NULL)
    {
        // Point to parameters
        struct lld_con_env_tag *con_par = lld_con_env[link_id];

        // Next event count, which corresponds to latest connection event scheduled at next_ts
        uint16_t next_evt_cnt = (con_par->evt_cnt + con_par->evt_inc);
        uint32_t next_ts = con_par->next_ts;

        // Calculate time of connection event relative to latest scheduled
        if (con_evt_cnt == next_evt_cnt)
        {
            // Requested event count is the latest scheduled
            *con_evt_time_hs = next_ts;
        }
        else if (con_evt_cnt > next_evt_cnt)
        {
            // Requested event count is in future, add N * interval
            *con_evt_time_hs = CLK_ADD_2(next_ts, ((con_evt_cnt - next_evt_cnt) * con_par->interval));
        }
        else
        {
            // Requested event count is in past, subtract N * interval
            *con_evt_time_hs = CLK_SUB(next_ts, ((next_evt_cnt - con_evt_cnt) * con_par->interval));
        }

        *con_evt_time_hus = lld_tx_path_delay[lld_con_env[link_id]->tx_rate] << 1;

        status = CO_ERROR_NO_ERROR;
    }
    GLOBAL_INT_RESTORE();

    return (status);
}

#endif //(AUDIO_SYNC_SUPPORT)

uint8_t lld_con_prio_elevation_set(uint8_t link_id, uint8_t elevation)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CON_3_PATCH_TYPE, LLD_CON_PRIO_ELEVATION_SET_3_FUNC_BIT, uint8_t, link_id, elevation);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    uint8_t prio_inc = rwip_priority[RWIP_PRIO_CONNECT_DFT_IDX].increment;

    GLOBAL_INT_DISABLE();

    // Check if can be configured, and within a configurable range
    if ((lld_con_env[link_id] != NULL) && (((uint16_t)elevation * prio_inc) < (RWIP_PRIO_MAX - rwip_priority[RWIP_PRIO_CONNECT_DFT_IDX].value)))
    {
        struct sch_arb_elt_tag *evt = &(lld_con_env[link_id]->evt);

        // Set the priority elevation
        lld_con_env[link_id]->prio_elevation = elevation * prio_inc;

        // Update immediately
        evt->current_prio = co_max(evt->current_prio, RWIP_PRIO_ADD_2(rwip_priority[RWIP_PRIO_CONNECT_DFT_IDX].value, lld_con_env[link_id]->prio_elevation));

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#if (RW_DEBUG)
void lld_con_evt_trace(uint16_t con_hdl, uint8_t nb_events)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CON_3_PATCH_TYPE, LLD_CON_EVT_TRACE_3_FUNC_BIT, con_hdl, nb_events);
    uint16_t link_id = BLE_CONHDL_TO_LINKID(con_hdl);

    GLOBAL_INT_DISABLE();

    // Check if can be configured
    if (lld_con_env[link_id] != NULL)
    {
        lld_con_env[link_id]->trc_nb_events = nb_events;
        lld_con_env[link_id]->p_trc_data = &lld_con_env[link_id]->trc_data[0];

        memset(&lld_con_env[link_id]->trc_data[0], 0, HCI_CON_MAX_TRACE_NB_EVENT * sizeof(uint16_t));
    }

    GLOBAL_INT_RESTORE();
}
#endif // (RW_DEBUG)

#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

///@} LLDCON
