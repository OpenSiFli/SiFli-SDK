/**
****************************************************************************************
*
* @file ld_acl.c
*
* @brief LD ACL source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDACL
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#include "rwip.h"           // IP definitions
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

#include "co_math.h"
#include "co_utils.h"
#include "co_bt.h"

#include "arch.h"
#include "rwip.h"
#include "ke_mem.h"

#include "ld.h"             // link driver API
#include "ld_util.h"        // link driver utilities
#include "ld_int.h"         // link driver internal
#include "ld_util.h"        // link driver utilities

#include "lc.h"
#include "lb.h"
#include "lm.h"

#include "dbg.h"

#if (EAVESDROPPING_SUPPORT)
    #include "ed.h"
#endif // EAVESDROPPING_SUPPORT

#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_alarm.h"          // Scheduling Alarm
#include "sch_prog.h"           // Scheduling Programmer
#include "sch_slice.h"          // Scheduling Slicer

#include "reg_ipcore.h"         // IP core registers
#include "reg_btcore.h"         // BT core registers
#include "reg_btcore_esco.h"    // eSCO registers
#include "reg_btcore_audio.h"   // Audio registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors
#include "reg_em_bt_txdesc.h"   // BT EM TX descriptors
#include "btdm_patch.h"

#undef __STATIC
#define __STATIC


/*
 * DEFINES
 *****************************************************************************************
 */

/// Number on correctly received packets for interpreting the RSSI average
#define LD_ACL_RSSI_AVG_NB_PKT     150

/// Maximum drop below the RSSI low threshold before max power is requested
#define LD_ACL_RSSI_BELOW_LOW_THR  3

/// The maximum size the Rx window can reach before continuous programming is disabled (in us)
#define LD_ACL_RX_WIN_SIZE_PROG_LIM         900

/// Event delay threshold where 1 half-slot is added to the stop notification delay (in half-us)
#define LD_ACL_STOP_NOTIF_BIT_OFF_THR       400

/// Minimum scheduling delay threshold where SW tries to advance the schedule (in BT slots)
#define LD_ACL_EVT_DELAY_RESCHED_MIN        10

/// ACL event duration min default (in us)
#define LD_ACL_EVT_DUR_MIN_DFT     (2*SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN)
/// ACL event duration min when the traffic is ongoing (in us)
#define LD_ACL_EVT_DUR_MIN_TRAFFIC  (4*SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN)
/// ACL event automatic rescheduling attempts
#define LD_ACL_EVT_AUTO_RESCHED_ATT 4
/// ACL event stop threshold (considering 1 slot for TX and 1 slot for RX) (in BT half-slots)
#define LD_ACL_EVT_STOP_THR_MIN    (2)

/// ACL event duration for  single Poll-Null exchange (in us)
#define LD_ACL_EVT_DUR_POLL_NULL   (SLOT_SIZE + 126)
/// ACL event priority increment on data exchange
#define LD_ACL_DATA_XCHG_PRIO_INC  8

/// Maximum value for RX/TX traffic weight indicator
#define LD_ACL_TRAFFIC_MAX              120

/// Default RX traffic value after receiving a packet successfully
#define LD_ACL_RX_TRAFFIC_DFT           1

/// Factor to apply to traffic weight for increasing event duration (increased by 1 every 2^n missed transmissions/receptions)
#define LD_ACL_TRAFFIC_DUR_INCR_FACTOR  4

/// Factor to apply to traffic weight for increasing event priority (increased by 1 every 2^n missed transmissions/receptions)
#define LD_ACL_TRAFFIC_PRIO_INCR_FACTOR  4

/// ACL Role Switch preparation delay (in BT slots)
#define LD_ACL_RSW_PREPARE_DELAY    10

/// ACL Role Switch event duration min (in BT slots)
#define LD_ACL_RSW_EVT_DUR_MIN         (2*SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN)

/// ACL sniff event duration min (in BT slots)
#define LD_ACL_SNIFF_EVT_DUR_MIN       (2*SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN)

/// Protection delay to stop programming ACL before sniff event in sniff transition mode (in BT half-slots)
#define LD_ACL_SNIFF_TRANS_ANC_PROTEC_DELAY  (8)

/// Default duration before switching to FPOLL (in 312.5 us half-slots)
#define LD_ACL_FPOLL_LIMIT_DFT         200

/// Margin before switching to FPOLL in sniff mode (in number of sniff intervals prior to LSTO)
#define LD_ACL_SNIFF_FPOLL_MARGIN      4

/// Minimum synchronization window when switching to a new piconet (in us)
#define LD_ACL_RSW_SYNC_WIN_SIZE_MIN   50
/// Maximum synchronization window when switching to a new piconet (in us)
#define LD_ACL_RSW_SYNC_WIN_SIZE_MAX   150

/// Minimum drift to notify peer's for new slot offset, when waiting for slave->master role switch (in us)
#define LD_ACL_RSW_DRIFT_NOTIF_THR     2

/// ACL Role Switch drift notification delay (in BT slots)
#define LD_ACL_RSW_DRIFT_NOTIF_DELAY   (LD_ACL_RSW_PREPARE_DELAY + 40)

/// Macro to enable a Basic Rate packet type in LD local bit field, from HCI bit field
#define LD_ACL_EN_BR_PKT_TYPE(br_types, pkt_types, pkt_type)       br_types  |= (packet_types & PACKET_TYPE_##pkt_type##_FLAG)  ? (1 << pkt_type##_IDX) : 0;

/// Macro to enable a Enhanced Data Rate packet type in LD local bit field, from HCI bit field
#define LD_ACL_EN_EDR_PKT_TYPE(edr_types, pkt_types, pkt_type, rate)     edr_types |= (packet_types & PACKET_TYPE_NO_##rate##_##pkt_type##_FLAG) ? 0 : (1 << pkt_type##_##rate##_IDX);

#if MAX_NB_SYNC

    /// Voice coding configurations
    #define AULAW_CODE_POS    0
    #define AULAW_CODE_MSK    0xF
    #define CVSD_EN_POS       4
    #define CVSD_EN_MSK       0x10
    #define AULAW_EN_POS      5
    #define AULAW_EN_MSK      0x20
    #define VOICE_CODE_MSK    0xF
    #define LINEAR_CVSD       0x10
    #define LINEAR_ALAW       0x22
    #define LINEAR_ULAW       0x2A
    #define ALAW_CVSD         0x31
    #define ALAW_ALAW         0x00
    #define ALAW_ULAW         0x2B
    #define ULAW_CVSD         0x35
    #define ULAW_ALAW         0x27
    #define ULAW_ULAW         0x00
    #define TRANSP            0x00

    /// Linear format
    #define LINEAR_FORMAT_8_BITS    0x00
    #define LINEAR_FORMAT_13_BITS   0x01
    #define LINEAR_FORMAT_14_BITS   0x02
    #define LINEAR_FORMAT_16_BITS   0x03

    /// Threshold on eSCO interval to skip one interval when switching to new parameters (in slots)
    #define LD_SCO_UPD_INTV_SKIP_THR     6

    #if VOICE_OVER_HCI
        /// Audio interrupt delay (time before an (e)SCO instant where the audio IRQ is raised)
        #define LD_SCO_AUDIO_IRQ_DELAY     rwip_prog_delay
    #endif //VOICE_OVER_HCI
#endif //MAX_NB_SYNC

#if (EAVESDROPPING_SUPPORT)
    /// Bit field definitions for monitoring AES-CCM counter reports, when binaural ACK is enabled
    #define ED_AES_CCM_CNT_LINK_ID_LSB           0
    #define ED_AES_CCM_CNT_LINK_ID_MASK          0x0F
    #define ED_AES_CCM_CNT_CON_TYPE_POS          6
    #define ED_AES_CCM_CNT_CON_TYPE_BIT          0x40
    #define ED_AES_CCM_CNT_BLOCK_POS             7
    #define ED_AES_CCM_CNT_BLOCK_BIT             0x80

    #define BIT27 0x08000000L
#endif // EAVESDROPPING_SUPPORT

/// Enable HW slot blocking for SAM
#define LD_ACL_SAM_SLOT_BLOCKING_EN    1


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// ACL mode
enum ACL_MODE
{
    ACL_MODE_NORMAL,
    ACL_MODE_SNIFF_TRANS,
    ACL_MODE_SNIFF_ENTER,
    ACL_MODE_SNIFF,
    ACL_MODE_ROLE_SWITCH,
};

/// ACL event states
enum ACL_EVT_STATE
{
    ACL_EVT_WAIT,
    ACL_EVT_ACTIVE,
    ACL_EVT_END,
};

/// ACL Role Switch steps
enum ACL_RSW_STEP
{
    ACL_RSW_STEP_PENDING,
    ACL_RSW_STEP_TDD_SWITCH,
    ACL_RSW_STEP_PICONET_SWITCH,
};

/// SAM Switch steps
enum ACL_SAM_STEP
{
    ACL_SAM_STEP_NONE,
    ACL_SAM_STEP_CHANGE_PENDING,
    ACL_SAM_STEP_CHANGE_PROG,
};

/// Test mode action
enum ACL_TEST_MODE_ACTION
{
    TEST_MODE_NO_ACTION,
    TEST_MODE_ENTER,
    TEST_MODE_EXIT,
};

/// AFH state
enum ACL_AFH_STATE
{
    AFH_NOT_PENDING,
    AFH_WAIT_INSTANT_WAIT_ACK,
    AFH_WAIT_INSTANT,
    AFH_WAIT_ACK,
};

#if MAX_NB_SYNC
/// SCO event states
enum SCO_EVT_STATE
{
    SCO_EVT_WAIT,
    SCO_EVT_ACTIVE,
    SCO_EVT_END,
};

/// SCO update
enum SCO_UPDATE
{
    SCO_NO_UPDATE,
    SCO_UPDATE_STEP1,
    SCO_UPDATE_STEP2,
    SCO_UPDATE_STEP3,
};

/// SCO packet types indexes
enum SCO_IDX
{
    IDX_SCO_PACKET_HV1,
    IDX_SCO_PACKET_HV3,
    IDX_SCO_PACKET_NULL,
    IDX_SCO_PACKET_EV3,
    IDX_SCO_PACKET_EV4,
    IDX_SCO_PACKET_EV5,
    IDX_SCO_PACKET_EV3_2,
    IDX_SCO_PACKET_EV3_3,
    IDX_SCO_PACKET_EV5_2,
    IDX_SCO_PACKET_EV5_3,
};
#endif //MAX_NB_SYNC

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// Test mode environment structure
struct ld_acl_test_mode_tag
{
    /// Test mode parameters (provided by higher layers)
    struct ld_acl_test_mode_params params;
    /// Buffer used for sending the test data
    uint16_t em_buf;
    /// Action (enter, update, exit)
    uint8_t action;
};

/// LD ACL Role Switch event parameters structure
struct ld_acl_rsw_evt_params
{
    /// Switch instant (in 312.5us BT half-slots)
    uint32_t instant;
    /// New connection end timestamp (in 312.5us BT half-slots)
    uint32_t new_con_end_ts;
    /// Slot offset reception TS (for master->slave switch only)
    uint32_t slot_offset_ts;
    /// FHS packet buffer (for slave->master switch only)
    uint16_t fhs_buf;
    /// Role Switch step
    uint8_t step;
    /// New LT address (slave->master: given by Higher layers | master->slave: received in FHS packet)
    uint8_t new_lt_addr;
    /// Temporary buffer for saving descriptor content during Role Switch
    uint8_t txdesc_save_buf[REG_EM_BT_TXDESC_SIZE];
    /// Temporary buffer for saving CS content during Role Switch
    uint8_t cs_save_buf[REG_EM_BT_CS_SIZE];
    /// TX descriptor pointer (saved from CS)
    uint16_t txdescptr;
};

/// LD ACL event sniff subrating parameters structure
struct ld_acl_evt_ssr_params
{
    /// Sniff subrating instant (in BT half-slots)
    uint32_t ssr_instant;
    /// Timestamp indicating when the SSR timer was last started (in BT half-slots)
    uint32_t ssr_restart_ts;
    /// Sniff subrating timeout (in BT slots)
    uint16_t ssr_to;
    /// Sniff subrating value
    uint8_t ssr;
    /// Indicates whether the instant has been reached
    bool ssr_instant_passed;
    /// Indicates whether the SSR timer is active (counting)
    bool ssr_timer;
    /// Indicates whether SSR is being used (true for sniff subrating, false for sniff mode)
    bool ssr_active;
};

/// LD ACL event sniff parameters structure
struct ld_acl_evt_sniff_params
{
    /// Value of last_sync_ts used for the calculation of sched_rx_win_size, sched_clock_off and sched_bit_off
    uint32_t sched_last_sync_ts_used;
    /// Scheduled RX window size (in half-us)
    uint32_t sched_rx_win_size;
    /// Scheduled clock offset (in half-slots)
    uint32_t sched_clock_off;
    /// Anchor point in local clock domain (in half-slots)
    uint32_t anchor_point;
    /// Scheduled bit offset (in half-us)
    int16_t sched_bit_off;
    /// Sniff offset (in BT slots)
    uint16_t offset;
    /// Sniff interval (in BT slots)
    uint16_t intv;
    /// Number of requested sniff attempts
    uint16_t att;
    /// Number of scheduled sniff attempts
    uint16_t sched_att;
    /// Sniff timeout (in BT slots)
    uint16_t to;
    /// Sniff subrating parameters structure
    struct ld_acl_evt_ssr_params ssr;
    /// Initialization method (0: init 1 / 1: init 2), from timing control flags
    uint8_t init;
};

/// SAM configuration
struct ld_acl_sam_info_tag
{
    /// SAM anchor point (CLK[27:1])
    uint32_t anchor_pt;
    /// pending SAM anchor point (CLK[27:1])
    uint32_t anchor_pt_pending;
    /// SAM instant after which new map can be activated (CLK[27:1])
    uint32_t instant;
    /// pending SAM map from peer
    uint8_t peer_map[RW_PEER_SAM_MAP_MAX_LEN];
    /// active map length (slots, 0=disabled)
    uint8_t map_length;
    /// pending map length (slots, 0=disable)
    uint8_t map_length_pending;
    /// SAM instant pending (peer map)
    bool peer_map_pending;
#if RW_BT_MWS_COEX
    /// active local SAM index (0-2, 0xFF=disabled)
    uint8_t loc_idx;
    /// SAM instant pending (local map)
    bool local_map_pending;
#endif //RW_BT_MWS_COEX
    /// Indicate sam has changed (@see enum ACL_SAM_STEP)
    uint8_t sam_change;
};

/// LD ACL environment structure
struct ld_acl_env_tag
{
    /// ACL event
    struct sch_arb_elt_tag evt;

    /// Alarm for AFH switch
    struct sch_alarm_tag afh_alarm;

    /// Temporary storage of new AFH maps
    struct bt_ch_map afh_map;

    /// LMP TX queue
    struct co_list queue_lmp_tx;
    /// ACL TX queue
    struct co_list queue_acl_tx;

    /// Current value of the Rx window size (in half-us)
    uint32_t rx_win_size;

#if (EAVESDROPPING_SUPPORT)
    /// Alarm for Rx statistics indication
    struct sch_alarm_tag stats_alarm;

    /// Rx statistics indication timestamp (corresponds to the master clock in half-us)
    uint32_t stats_ts;

    /// Indicate to update AES-CCM counter
    bool aes_ccm_cnt_upd;

    /// Indicates if the AES-CCM counter needs correction
    bool aes_ccm_cnt_correction;

    /// Indicates if the AES-CCM counter correction flag needs to be set
    bool aes_ccm_cnt_correction_upd;

    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t con_type;

    /// New AES-CCM counter
    uint8_t aes_ccm_cnt[5];

    /// Custom event duration for scheduling (in half-us, 0=disable)
    uint16_t cust_evt_dur_min;

    /// MSB of the piconet clock (needed for clock wrap notification when slave)
    bool piconet_msb;

    /// eSCO binary ack enable
    bool esco_bin_ack_en;

    /// LRD enable
    bool lrd_en;

    /// Indicates if Tx programming is controlled by the host or not (LRD)
    bool control_tx_prog;

    /// Indicates if Tx programming is enabled or not (LRD)
    bool tx_prog_en;

    /// Ignore SEQN Errors (DIS = 0x00, EN = 0x01)
    uint8_t ignore_seqn_errors;

    /// L2CAP channel ID
    uint16_t l2cap_cid;

    /// NULL filtering (DIS = 0x00, EN = 0x01)
    bool null_flt;

    /// Indicates if corrupted ACL payload reporting is enabled or not
    bool corr_acl_payl_en;

    /// Indicates if Rx stats reporting is enabled or not
    bool rx_stats_rep_en;

    /// Rx stats reporting interval in slots (625 us)
    uint8_t rx_stats_rep_intv;

    /// Total number of reception attempts
    uint8_t rx_tot;

    /// Number of synchronized reception attempts
    uint8_t rx_sync;

    /// Rx stats array
    uint8_t rx_stats[HCI_ECI_RX_STATS_LEN_MAX];

    /// Indicates whether the ECI_MIC_Invalid VS event is generated or not
    bool gen_mic_invalid_event;

    /// Indicates whether the ECI_MIC_Invalid VS event is blocked or not
    bool mic_invalid_event_block;

    /// Indicates whether the ECI_AES_CCM_Counter VS event is generated or not
    bool gen_aes_ccm_counter_event;

    /// Indicates whether the Rx AES-CCM counter is increased after reception of an invalid MIC or not
    bool increase_counter;

    /// Indicates whether special handling for invalid MIC is enabled or not
    uint8_t tolerate_invalid_mic;

    /// Number of invalid MIC receptions before the encryption key is refreshed
    uint8_t n_refr;

    /// Number of invalid MIC receptions before the link is disconnected
    uint8_t n_disc;

    /// Number of consecutive MIC errors
    uint8_t mic_err_cnt;

#endif // EAVESDROPPING_SUPPORT

#if PCA_SUPPORT
    /// Alarm for clock adjust (slave)
    struct sch_alarm_tag clk_adj_alarm;
    /// Temporary storage of new clock offset adjustment (in BT slots)
    uint32_t clk_off_adj;
    /// Temporary storage of new bit offset adjustment (in us)
    int16_t bit_off_adj;
    /// Clock recovery requested flag for ongoing rx timeout
    bool sscan_clk_recov;
#endif // PCA_SUPPORT

    /// Last value of master clock checked for monitoring day counter (in BT half-slots)
    uint32_t last_master_clock;

    /// Current L2CAP packet start timestamp (in BT half-slots)
    uint32_t curr_l2cap_start_ts;

    /// Value of CLKN when the last sync was detected  (in BT half-slots)
    uint32_t last_sync_ts;

    /// Value of CLKN when the last correct packet header is received (in BT half-slots)
    uint32_t last_hec_ok_ts;

    /// Clock offset (in BT half-slots)
    uint32_t last_sync_clk_off;

    /// Current clock offset (in BT half-slots)
    uint32_t clk_off;

    /// Clock offset in BT half-slots (slave->master: clock offset value when bit offset was sent to the peer)
    uint32_t rsw_clk_off;

    /// Value of the day counter used for AES-CCM encryption (in ~ 23.3h unit)
    uint16_t day_count;

    /// Slot offset (master->slave: value received from peer | slave->master: bit offset value sent to peer)
    uint16_t rsw_bit_off;

    /// Current bit offset (in half-us)
    int16_t bit_off;

    /// Current bit offset (in half-us)
    int16_t last_sync_bit_off;

    /**
     * Link Supervision TO
     * 0x0000: No Link_Supervision_Timeout.
     * N = 0xXXXX:
     *  Size: 2 Octets
     *  Range: 0x0001 to 0xFFFF
     *  Default: 0x7D00
     *  Mandatory Range: 0x0190 to 0xFFFF
     *  Time = N * 0.625 msec
     *  Time Range: 0.625 msec to 40.9 sec
     *  Time Default: BR/EDR 20 sec
     */
    uint16_t lsto;

    /**
     * Flush TO
     * 0: infinite - No Automatic Flush
     * N = 0xXXXX
     * Flush Timeout = N * 0.625 msec
     * Size: 11 bits
     * Range: 0x0001 to 0x07FF
     */
    uint16_t flush_to;

    /// Buffer element of the packet currently under transmission
    struct bt_em_acl_buf_elt *next_buf_elt;
    /// Buffer element of the packet to transmit next (1st segment already pending for transmission)
    struct bt_em_acl_buf_elt *curr_buf_elt;
    /// EM position of the next data to transmit
    uint16_t data_ptr;
    /// Length of data remaining to transmit from the current or next buffer
    uint16_t data_len;

    /// Accumulated RSSI (to compute an average value)
    int16_t rssi_acc;
    /// Counter of received packets used in RSSI average
    uint8_t rssi_avg_cnt;
    /**
     *  Difference between the measured Received Signal Strength Indication (RSSI) and the limits of the Golden Receive
     *  Power Range (in dBm).
     *  Any positive RSSI value returned by the Controller indicates how many dB the RSSI is above the upper limit, any
     *  negative value indicates how many dB the RSSI is below the lower limit. The value zero indicates that the RSSI
     *  is inside the Golden Receive Power Range.
     *  Range: -128 <= N <= 127 (signed integer)
     */
    int8_t last_rssi_delta;

    /**
     * Allowed Basic Rate packet types: bit=1 => allowed | bit=0 => not allowed
     *  bit 0: DM1
     *  bit 1: DH1
     *  bit 2: DM3
     *  bit 3: DH3
     *  bit 4: DM5
     *  bit 5: DH5
     */
    uint8_t allowed_br_packet_types;

    /**
     * Allowed Enhanced Data Rate packet types: bit=1 => allowed | bit=0 => not allowed
     *  bit 0: DM1
     *  bit 1: DH1_2
     *  bit 2: DH1_3
     *  bit 3: DH3_2
     *  bit 4: DH3_3
     *  bit 5: DH5_2
     *  bit 6: DH5_3
     */
    uint8_t allowed_edr_packet_types;

    /// Link identifier
    uint8_t link_id;
    /// Role (0: Master | 1: Slave)
    uint8_t role;
    /// State
    uint8_t state;
    /// Mode (0: Normal | 1: Sniff transition | 2: Sniff)
    uint8_t mode;
    /// Maximum value of the peer clock drift (measured in ppm)
    uint8_t peer_drift;
    /// Maximum value of the peer clock jitter (measured in us)
    uint8_t peer_jitter;
    /// Polling interval (in slots)
    uint16_t t_poll;
    /// Phase of the ACL frames (every 2 slots), compared to local clock (half-slots) (0, 1, 2 or 3)
    uint8_t phase;
    /// RX traffic flag (0: no traffic | >0: traffic, increase with failing reception attempts, reloaded to initial value after successful reception)
    uint8_t rx_traffic;
    /// TX traffic flag (0: no traffic | >0: traffic, increase with failing transmission attempts)
    uint8_t tx_traffic;
    /// Number of programmed frames
    uint8_t nb_prog;
    /// Programming enable/disable
    bool prog_en;
    /// Enhanced data rate enable/disable
    bool edr_en;
    /// TX flow ON/OFF
    bool tx_flow;
    /// Index of TX descriptor currently used by HW [0:1]
    uint8_t txdesc_index_hw;
    /// Index of TX descriptor currently used by SW [0:1]
    uint8_t txdesc_index_sw;
    /// Number of prepared TX descriptors (0, 1, or 2)sam
    uint8_t txdesc_cnt;
    /// PTI update control
    uint8_t pti;

    /// Maximum number of slots per packet peer device is allowed to use
    uint8_t rx_max_slot;

    /// AFH current state
    uint8_t afh_state;

    /**
     * SCO HV1 index (SCO link ID + 1)
     *  0: No HV1
     *  1: HV1 on SCO link ID 0
     *  2: HV1 on SCO link ID 1
     * Note: can be used as a flag to check the presence of a SCO HV1, or as an index to retrieve SCO link ID
     */
    uint8_t sco_hv1_idx;

    /**
     * SCO EV3 with reTx index (SCO link ID + 1)
     *  0: No EV3
     *  1: EV3 on SCO link ID 0
     *  2: EV3 on SCO link ID 1
     * Note: can be used as a flag to check the presence of a SCO EV3 with reTx, or as an index to retrieve SCO link ID
     */
    uint8_t sco_ev3_retx_idx;

    /**
     * Phase change
     *  Flag indicating that a phase change has been detected on the slave ACL link
     *  The phase change is a critical point for the slave synchronization algorithm, then on phase change, the
     *  programming is blocked and the next sync algorithm is not executed. Things restart normally after that.
     */
    bool phase_chg;

    /// Sniff parameters
    struct ld_acl_evt_sniff_params sniff_par;

    /// Role Switch parameters
    struct ld_acl_rsw_evt_params *rsw_par;

    /// Test mode environment
    struct ld_acl_test_mode_tag *test_mode;

    /// SAM configuration
    struct ld_acl_sam_info_tag *sam_info;

};

#if MAX_NB_SYNC
/// LD SCO environment structure
struct ld_sco_env_tag
{
    /// SCO event
    struct sch_arb_elt_tag evt;

    /// Alarm for rescheduling
    struct sch_alarm_tag resched_alarm;

    /// Anchor point in local clock domain (in half-slots)
    uint32_t anchor_point;

    /// RX packet length (in bytes)
    uint16_t rx_pkt_len;
    /// TX packet length (in bytes)
    uint16_t tx_pkt_len;
    /// eSCO Tx EM pointer 1, saved/restored when loopback mode is enabled/disabled
    uint16_t escoptrtx1;
    /// eSCO Tx EM pointer 0, saved/restored when loopback mode is enabled/disabled
    uint16_t escoptrtx0;
    /// eSCO Rx EM pointer 1, saved/restored when loopback mode is enabled/disabled
    uint16_t escoptrrx1;
    /// eSCO Rx EM pointer 0, saved/restored when loopback mode is enabled/disabled
    uint16_t escoptrrx0;
    /// End of SCO
    uint8_t end;
    /// eSCO LT address
    uint8_t lt_addr;
    /// Link type (0: SCO | 2: ESCO)
    uint8_t sync_type;
    /// RX packet type (as defined in BT specification LMP:5.2)
    uint8_t rx_pkt_type;
    /// TX packet type (as defined in BT specification LMP:5.2)
    uint8_t tx_pkt_type;
    /// Synchronous link offset (in slots)
    uint8_t d_esco;
    /// Synchronous link interval (in slots)
    uint8_t t_esco;
    /// Initialization method (0: init 1 / 1: init 2), from timing control flags
    uint8_t init;
    /// Number of retransmission attempts (0, 1 or 2) (eSCO only)
    uint8_t retx_att_nb;
    /// Number of reserved slots
    uint8_t rsvd_slots;
    /// Link identifier
    uint8_t link_id;
    /// SCO Link identifier
    uint8_t sco_link_id;
    /// Number of programmed frames (0, 1 or 2 (in case of full bandwidth))
    uint8_t nb_prog;
    /// Flag indicating full bandwidth (e.g. eSCO EV3 / T=6 with 2 reTx)
    bool bw_full;
    /// eSCO parameters update requested by HL
    uint8_t update;

    /// Audio data path (Voice over HCI, PCM, etc ...)
    uint8_t data_path;

#if VOICE_OVER_HCI
    /// Buffer TX queue
    struct co_list queue_tx;
    /// Sample size at the entry of the audio path (True: 16-bits - False: 8 bits)
    bool sample_size_16_bits;
#endif //VOICE_OVER_HCI

#if (EAVESDROPPING_SUPPORT)

    /// Sampling of eSCO Tx SEQN is pending
    bool sample_tx_seqn;

#endif // EAVESDROPPING_SUPPORT
};
#endif //MAX_NB_SYNC


/*
 * CONSTANTS DEFINITIONS
 *****************************************************************************************
 */

/// Table of the first 64 bytes of pseudo-random data
__STATIC const uint8_t prbs_64bytes[64] =
{
    0xFF, 0x83, 0xDF, 0x17, 0x32, 0x09, 0x4E, 0xD1, 0xE7, 0xCD, 0x8A, 0x91, 0xC6, 0xD5,
    0xC4, 0xC4, 0x40, 0x21, 0x18, 0x4E, 0x55, 0x86, 0xF4, 0xDC, 0x8A, 0x15, 0xA7, 0xEC,
    0x92, 0xDF, 0x93, 0x53, 0x30, 0x18, 0xCA, 0x34, 0xBF, 0xA2, 0xC7, 0x59, 0x67, 0x8F,
    0xBA, 0x0D, 0x6D, 0xD8, 0x2D, 0x7D, 0x54, 0x0A, 0x57, 0x97, 0x70, 0x39, 0xD2, 0x7A,
    0xEA, 0x24, 0x33, 0x85, 0xED, 0x9A, 0x1D, 0xE1
};


#if MAX_NB_SYNC

/// Indexes for SCO packet types
__STATIC const uint8_t ld_sco_pkt_idx[][2] =
{
    {SCO_PACKET_HV1, IDX_SCO_PACKET_HV1},
    {SCO_PACKET_HV3, IDX_SCO_PACKET_HV3},
};

/// Indexes for ESCO packet types
__STATIC const uint8_t ld_esco_pkt_idx[][2] =
{
    {ESCO_PACKET_NULL, IDX_SCO_PACKET_NULL},
    {ESCO_PACKET_EV3, IDX_SCO_PACKET_EV3},
    {ESCO_PACKET_EV4, IDX_SCO_PACKET_EV4},
    {ESCO_PACKET_EV5, IDX_SCO_PACKET_EV5},
    {ESCO_PACKET_EV3_2, IDX_SCO_PACKET_EV3_2},
    {ESCO_PACKET_EV3_3, IDX_SCO_PACKET_EV3_3},
    {ESCO_PACKET_EV5_2, IDX_SCO_PACKET_EV5_2},
    {ESCO_PACKET_EV5_3, IDX_SCO_PACKET_EV5_3},
};


/// Setting for SCO/ESCO packet types
__STATIC const uint8_t ld_sco_pkt_types[][3] =
{
    {HV1_TYPE, 0, 1},
    {HV3_TYPE, 0, 1},
    {ID_NUL_TYPE, 0, 1},
    {EV3_TYPE, 0, 1},
    {EV4_TYPE, 0, 3},
    {EV5_TYPE, 0, 3},
    {EV3_2_TYPE, 1, 1},
    {EV3_3_TYPE, 1, 1},
    {EV5_2_TYPE, 1, 3},
    {EV5_3_TYPE, 1, 3},
};

/// Audio Voice Coding
__STATIC const uint8_t ld_sco_voice_coding[][6] =
{
//Air  |    ULAW     |    ALAW     |    CVSD     | Transp |  | Linear PCM |  | mSBC |
    { ULAW_ULAW, ULAW_ALAW, ULAW_CVSD, TRANSP, TRANSP, TRANSP },           // ULAW
    { ALAW_ULAW, ALAW_ALAW, ALAW_CVSD, TRANSP, TRANSP, TRANSP },           // ALAW
    { TRANSP, TRANSP, TRANSP, TRANSP, TRANSP, TRANSP },                    // CVSD, not possible
    { TRANSP, TRANSP, TRANSP, TRANSP, TRANSP, TRANSP },                    // Transp, not possible
    { LINEAR_ULAW, LINEAR_ALAW, LINEAR_CVSD, TRANSP, TRANSP, TRANSP },     // Linear PCM
};
#endif //MAX_NB_SYNC


/*
 * VARIABLES DEFINITIONS
 *****************************************************************************************
 */

/// LD ACL environment variable
__STATIC struct ld_acl_env_tag *ld_acl_env[MAX_NB_ACTIVE_ACL];

#if MAX_NB_SYNC
    /// LD SCO environment variable
    __STATIC struct ld_sco_env_tag *ld_sco_env[MAX_NB_SYNC];
#endif //MAX_NB_SYNC

#if (EAVESDROPPING_SUPPORT)
    /**
    * Bit field for monitoring AES-CCM counter reports, when binaural ACK is enabled
    * |   7   |    6     |  6..4  |    3..0    |
    * | BLOCK | CON_TYPE |  RFU   |  LINK_ID   |
    * Note: it is assumed that only 1 link can have binaural ACK enabled
    * Note: it is assumed that EDMICERR IRQ happens only when binaural ACK is enabled, no protection against spurious IRQ
    */
    uint8_t ld_aes_ccm_cnt;
#endif // EAVESDROPPING_SUPPORT

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_acl_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
__STATIC void ld_acl_clk_isr(uint32_t clock, uint8_t id);
__STATIC void ld_acl_rsw_end(uint8_t link_id, uint8_t reason, uint32_t timestamp);
__STATIC void ld_acl_rsw_start(uint8_t link_id, uint32_t instant_ts);
__STATIC void ld_acl_rsw_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
__STATIC void ld_acl_rsw_evt_canceled_cbk(struct sch_arb_elt_tag *evt);
__STATIC void ld_acl_rsw_evt_start_cbk(struct sch_arb_elt_tag *evt);
__STATIC void ld_acl_sniff_enter(uint8_t link_id, uint32_t clock);
__STATIC void ld_acl_sniff_exit(uint8_t link_id);
__STATIC void ld_acl_sniff_sched(uint8_t link_id);
__STATIC void ld_acl_sniff_trans_sched(uint8_t link_id, uint32_t clock);
__STATIC void ld_acl_sniff_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
__STATIC void ld_acl_sniff_evt_start_cbk(struct sch_arb_elt_tag *evt);
__STATIC void ld_acl_sniff_evt_canceled_cbk(struct sch_arb_elt_tag *evt);
#if MAX_NB_SYNC
    __STATIC void ld_sco_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
    __STATIC void ld_sco_sched(uint8_t sco_link_id);
#endif //MAX_NB_SYNC


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

#if (EAVESDROPPING_SUPPORT)
__STATIC void ld_acl_apply_aes_ccm_cnt(uint8_t link_id, uint8_t *src_ptr)
{
    if (ld_acl_env[link_id]->con_type == ED_CON_TYPE_ESCO)
    {
        uint32_t sco_day_cntr;

        // TODO: Check byte and bit order
        memcpy(&sco_day_cntr, &src_ptr[0], sizeof(uint32_t));
        bt_e_scodaycnt_set(link_id, sco_day_cntr);
    }
    else
    {
        uint16_t rx_ccm_pld_cntr0, rx_ccm_pld_cntr1, rx_ccm_pld_cntr2;

        // TODO: Check byte and bit order
        rx_ccm_pld_cntr0 = 256 * src_ptr[3] + src_ptr[4];
        rx_ccm_pld_cntr1 = 256 * src_ptr[1] + src_ptr[2];
        rx_ccm_pld_cntr2 = src_ptr[0] & 0x0F;

        em_bt_rxccmpldcnt0_set(link_id, rx_ccm_pld_cntr0);
        em_bt_rxccmpldcnt1_set(link_id, rx_ccm_pld_cntr1);
        em_bt_rxccmpldcnt2_set(link_id, rx_ccm_pld_cntr2);
    }
}

__STATIC void ld_ed_afh_channel_map_changed_ind(uint8_t                  link_id,
        uint8_t                  enabled,
        uint32_t                 time,
        const struct bt_ch_map  *ch_map)
{
    // Allocate indication message
    struct ed_afh_ch_map_chg_ind *ind = KE_MSG_ALLOC(ED_AFH_CH_MAP_CHG_IND, TASK_ED, TASK_NONE, ed_afh_ch_map_chg_ind);

    // Fill data
    ind->link_id  = link_id;
    ind->enabled  = enabled;
    ind->time     = time;
    memcpy(&ind->ch_map.map[0], &ch_map->map[0], BT_CH_MAP_LEN);

    // Send message
    ke_msg_send(ind);
}

__STATIC void ld_ed_max_slot_changed_ind(uint8_t link_id, uint8_t max_slot)
{
    // Allocate indication message
    struct ed_max_slot_chg_ind *ind = KE_MSG_ALLOC(ED_MAX_SLOT_CHG_IND, TASK_ED, TASK_NONE, ed_max_slot_chg_ind);

    // Fill data
    ind->link_id  = link_id;
    ind->max_slot = max_slot;

    // Send message
    ke_msg_send(ind);
}

__STATIC void ld_ed_ptt_changed_ind(uint8_t link_id, uint8_t ptt)
{
    // Allocate indication message
    struct ed_ptt_chg_ind *ind = KE_MSG_ALLOC(ED_PTT_CHG_IND, TASK_ED, TASK_NONE, ed_ptt_chg_ind);

    // Fill data
    ind->link_id = link_id;
    ind->table   = ptt;

    // Send message
    ke_msg_send(ind);
}

__STATIC void ld_ed_sco_changed_ind(uint8_t link_id, uint8_t status, uint8_t type,  uint8_t lt_addr,
                                    uint8_t flags,   uint8_t d,      uint8_t t,     uint8_t w,
                                    uint8_t type_m,  uint8_t type_s, uint8_t len_m, uint8_t len_s,
                                    uint8_t air_mode)
{
    // Allocate indication message
    struct ed_sco_con_chg_ind *ind = KE_MSG_ALLOC(ED_SCO_CON_CHG_IND, TASK_ED, TASK_NONE, ed_sco_con_chg_ind);

    // Fill data
    ind->link_id      = link_id;
    ind->status       = status;
    ind->type         = type;
    ind->lt_addr      = lt_addr;
    ind->timing_flags = flags;
    ind->d            = d;
    ind->t            = t;
    ind->w            = w;
    ind->type_m       = type_m;
    ind->type_s       = type_s;
    ind->len_m        = len_m;
    ind->len_s        = len_s;
    ind->air_mode     = air_mode;

    // Send message
    ke_msg_send(ind);
}

/**
 ****************************************************************************************
 * @brief Handle Rx statistics indication alarm
 ****************************************************************************************
 */
__STATIC void ld_acl_rx_stats_ind_cbk(struct sch_alarm_tag *elt)
{
    struct ld_acl_env_tag *acl_par = CONTAINER_OF(elt, struct ld_acl_env_tag, stats_alarm);

    if (acl_par->rx_stats_rep_en)
    {
        // Allocate indication message
        struct ed_rx_stats_ind *ind = KE_MSG_ALLOC(ED_RX_STATS_IND, TASK_ED, TASK_NONE, ed_rx_stats_ind);

        // Fill data
        ind->link_id  = acl_par->link_id;
        ind->clkn = acl_par->stats_ts >> 1;
        ASSERT_ERR(co_abs(CLK_DIFF(CLK_ADD_2(elt->time.hs, acl_par->clk_off), acl_par->stats_ts)) < 3);
        ind->rx_tot = acl_par->rx_tot;
        ind->rx_sync = acl_par->rx_sync;
        memcpy(&ind->rx_stats[0], &acl_par->rx_stats[0], acl_par->rx_sync);

        // Send message
        ke_msg_send(ind);

        // Clear the statistics
        acl_par->rx_tot = 0;
        acl_par->rx_sync = 0;

        // Program the reschedule via an alarm
        {
            uint32_t clock = ld_read_clock();
            uint32_t master_clock = CLK_ADD_2(clock, acl_par->clk_off);
            uint8_t intv = acl_par->rx_stats_rep_intv;
            uint32_t instant;
            int32_t diff;

            // Make sure that the master clock considered is even (corresponds to a BT slot)
            if (master_clock & 0x1)
            {
                master_clock &= ~0x1;
                clock = CLK_SUB(clock, 1);
            }

            // Compute the next instant
            diff = intv - CO_MOD((master_clock >> 1), intv);

            instant = CLK_ADD_2(clock, 2 * diff);
            acl_par->stats_ts = CLK_ADD_2(master_clock, 2 * diff);
            ASSERT_ERR(CO_MOD(acl_par->stats_ts, 2 * intv) == 0);

            // Program the reschedule via an alarm
            acl_par->stats_alarm.time.hs = instant;
            acl_par->stats_alarm.time.hus = acl_par->bit_off;
            sch_alarm_set(&acl_par->stats_alarm);
        }
    }
}

#endif // EAVESDROPPING_SUPPORT

#if MAX_NB_SYNC
/**
 ****************************************************************************************
 * @brief Update day counter if needed
 ****************************************************************************************
 */
__STATIC void ld_acl_day_count_update(uint8_t link_id, uint32_t master_clock)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_DAY_COUNT_UPDATE_BIT, link_id, master_clock);

    // If day counter has already been initialized
    if (acl_par->last_master_clock != RWIP_INVALID_TARGET_TIME)
    {
        // If master clock wraps, increment day counter
        if (((acl_par->last_master_clock & (1 << 27)) != 0) && ((master_clock & (1 << 27)) == 0))
        {
            acl_par->day_count++;
        }

        // Save master clock
        acl_par->last_master_clock = master_clock;
    }
}

/**
 ****************************************************************************************
 * @brief Check if a timestamp correspond to a SCO slot of the ACL link (reserved slot or retransmission window)
 ****************************************************************************************
 */
__STATIC uint8_t ld_acl_sco_check(uint8_t link_id, uint32_t timestamp, bool *reserved)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint8_t vxchan = MAX_NB_SYNC;
    *reserved = true;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SCO_CHECK_3_FUNC_BIT, uint8_t, link_id, timestamp, reserved);

    // If test mode SCO loopback is active, every slot is treated as SCO reserved
    if ((acl_par->test_mode != NULL)
            && ((acl_par->test_mode->params.test_scenario == SCOLOOP_MODE) || (acl_par->test_mode->params.test_scenario == SCONOWHIT_MODE)))
    {
        vxchan = 0;
    }
    else if (acl_par->sco_hv1_idx)
    {
        vxchan = acl_par->sco_hv1_idx - 1;
    }
    else
    {
        // Check if the frame coincides with a SCO reserved slot
        for (int sco_link_id = (MAX_NB_SYNC - 1) ; sco_link_id >= 0 ; sco_link_id--)
        {
            if (ld_sco_env[sco_link_id] != NULL)
            {
                // Point to parameters
                struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

                if ((sco_par->link_id == link_id) && (sco_par->update == SCO_NO_UPDATE) && (!sco_par->end))
                {
                    uint8_t slot_position = CO_MOD((CLK_SUB(timestamp + 2 * sco_par->t_esco, sco_par->anchor_point) >> 1), sco_par->t_esco);
                    if (slot_position < (sco_par->rsvd_slots + (1 + (acl_par->role == SLAVE_ROLE))*sco_par->retx_att_nb))
                    {
                        vxchan = sco_link_id;

                        // Clear the retransmission attempts
                        bt_e_scoltcntl_retxnb_setf(sco_link_id, 1);

                        // Update day counter if needed
                        ld_acl_day_count_update(link_id, CLK_ADD_2(timestamp, acl_par->last_sync_clk_off));

                        // Program day counter for AES-CCM encryption
                        bt_e_scodaycnt_set(sco_link_id, acl_par->day_count);

                        // If first slot of the Tesco, indicate reserved slot, else retransmission slot
                        *reserved = (slot_position == 0);

                        break;
                    }
                }
            }
        }
    }

    return vxchan;
}
#endif //MAX_NB_SYNC

/**
 ****************************************************************************************
 * @brief Check if DV allowed for this LMP on HV1
 *
 * @param[in]   buf_ptr          Pointer to the LMP payload buffer (in EM)
 * @param[in]   length           PDU length (in bytes)
 * @return      txdv             True if DV allowed on HV1, False otherwise
 ****************************************************************************************
 */
__INLINE bool ld_acl_lmp_txdv(uint16_t buf_ptr, uint8_t length)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ACL_4_PATCH_TYPE, LD_ACL_LMP_TXDV_4_FUNC_BIT, bool, buf_ptr, length);
    bool txdv = false;

    /*
     * LMP messages shall be transmitted using DM1 packets, however if an HV1
     * SCO link is in use and the length of the payload is no greater than 9 bytes then
     * DV packets may be used. vol2c 2.3 Packet Format
     */
    if (length <= DV_ACL_PACKET_SIZE)
    {
        /*
         * Check opcode exceptions. Note some PDUs (LMP_SETUP_COMPLETE and LMP_SWITCH_REQ) are listed as DM1 only
         * as cannot be used when HV1 active. No need to explicitly check for these.
         */
        uint8_t opcode = GETF(em_rd8p(buf_ptr), LMP_OPCODE);

        if (opcode == LMP_ACCEPTED_OPCODE)
        {
            /*
             * For LMP_SCO_LINK_REQ PDU, if the SCO packet type is HV1 the LMP_ACCEPTED shall be
             * sent using the DM1 packet. vol2c 4.6.1 SCO logical transport
             */
            txdv = (LMP_SCO_LINK_REQ_OPCODE != em_rd8p(buf_ptr + 1));
        }
        else if (opcode == LMP_ESC4_OPCODE)
        {
            uint8_t ext_opcode = em_rd8p(buf_ptr + 1);

            /*
             * All extended opcode PDUs, with the exception of Power Control Req/Res shall be
             * sent using the DM1 packet. vol2c 5.1 PDU Summary Table review
             */
            txdv = ((LMP_PWR_CTRL_REQ_EXTOPCODE == ext_opcode) || (LMP_PWR_CTRL_RES_EXTOPCODE == ext_opcode));
        }
        else
        {
            txdv = true;
        }
    }

    return txdv;
}

/**
 ****************************************************************************************
 * @brief Report the end of the activity
 ****************************************************************************************
 */
__STATIC void ld_acl_end(uint8_t link_id, uint8_t status)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    struct bt_em_lmp_buf_elt *lmp_buf_elt;
    struct bt_em_acl_buf_elt *acl_buf_elt;

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_END_BIT, link_id, status);

    // Check if test mode was enabled
    if (acl_par->test_mode != NULL)
    {
        // Disable direct loopback
        bt_rftestfreq_directloopbacken_setf(0);
        // Disable test mode
        bt_rftestfreq_testmodeen_setf(0);

        // Release TX buffer
        bt_util_buf_acl_tx_free(acl_par->test_mode->em_buf);

        // Release memory
        ke_free(acl_par->test_mode);
        acl_par->test_mode = NULL;
    }

    // Free pending LMP TX buffers
    do
    {
        // Extract a buffer
        lmp_buf_elt = (struct bt_em_lmp_buf_elt *) co_list_pop_front(&acl_par->queue_lmp_tx);

        if (lmp_buf_elt != NULL)
        {
            // Free buffer
            bt_util_buf_lmp_tx_free(lmp_buf_elt->buf_ptr);
        }

    }
    while (lmp_buf_elt != NULL);

    // Check if any LMP is currently programmed in TX descriptors
    for (int i = 0 ; i < 2 ; i++)
    {
        uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, i);

        if (em_bt_txlmbufptr_getf(txdesc_idx))
        {
            // Free buffer
            bt_util_buf_lmp_tx_free(em_bt_txlmbufptr_getf(txdesc_idx));
        }
    }

    // Free pending ACL TX buffers
    do
    {
        // Extract a buffer
        acl_buf_elt = (struct bt_em_acl_buf_elt *) co_list_pop_front(&acl_par->queue_acl_tx);

        if (acl_buf_elt != NULL)
        {
            // Free buffer
            bt_util_buf_acl_tx_free(acl_buf_elt->buf_ptr);
        }

    }
    while (acl_buf_elt != NULL);

    // Free current and next ACL buffers
    if (acl_par->curr_buf_elt)
    {
        // Free buffer
        bt_util_buf_acl_tx_free(acl_par->curr_buf_elt->buf_ptr);

        if (acl_par->next_buf_elt)
        {
            // Free buffer
            bt_util_buf_acl_tx_free(acl_par->next_buf_elt->buf_ptr);
        }
    }

    // Report ACL disconnection to LC
    ke_msg_send_basic(LC_ACL_DISC_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);

    // Clear a potential AFH alarm
    sch_alarm_clear(&acl_par->afh_alarm);

#if (EAVESDROPPING_SUPPORT)
    // Clear a potential statistics alarm
    if (acl_par->rx_stats_rep_en)
    {
        sch_alarm_clear(&acl_par->stats_alarm);
    }
#endif // EAVESDROPPING_SUPPORT

    // Unregister ACL from active links
    ld_active_mode_set(LD_ACT_TYPE_ACL, link_id, 0, LD_ACT_MODE_OFF);

    // Unregister ACL link from scheduling parameters
    sch_slice_per_remove(BT_SNIFF, link_id);

    // Check if Role Switch memory has been allocated
    if (acl_par->rsw_par != NULL)
    {
        // Free Role Switch memory
        ke_free(acl_par->rsw_par);
    }

    // Check if SAM memory has been allocated
    if (acl_par->sam_info != NULL)
    {
        ke_free(acl_par->sam_info);
    }

#if defined(CFG_MEM_PROTECTION)
    {
#if MAX_NB_SYNC
        bool sco_active = false;

        // Look for SCO connections on the link
        for (int sco_link_id = (MAX_NB_SYNC - 1) ; sco_link_id >= 0 ; sco_link_id--)
        {
            if (ld_sco_env[sco_link_id] != NULL)
            {
                // Point to parameters
                struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

                if ((sco_par->link_id == link_id) && (sco_par->nb_prog > 0))
                {
                    sco_active = true;
                }
            }
        }

        if (!sco_active)
#endif // MAX_NB_SYNC
        {
            // Remove permission/status of CS as now unused
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_ACL_INDEX(link_id))), REG_EM_BT_CS_SIZE, false, false, false);
        }
    }
#endif // defined(CFG_MEM_PROTECTION)

    // Free event memory
    ke_free(ld_acl_env[link_id]);
    ld_acl_env[link_id] = NULL;
}

/**
 ****************************************************************************************
 * @brief Program the SAM offset configuration
 ****************************************************************************************
 */
__STATIC void ld_acl_sam_offset_set(uint8_t cs_idx, struct ld_acl_env_tag *acl_par, uint32_t clk_prg)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_SAM_OFFSET_SET_2_FUNC_BIT, cs_idx, acl_par, clk_prg);
    if (acl_par->sam_info->sam_change == ACL_SAM_STEP_CHANGE_PENDING)
    {
        // Update state to indicate SAM map is programmed
        acl_par->sam_info->sam_change = ACL_SAM_STEP_CHANGE_PROG;
    }
    else if (acl_par->sam_info->sam_change == ACL_SAM_STEP_CHANGE_PROG)
    {
        // Notify the SAM map has changed
        ke_msg_send_basic(LC_SAM_CHANGE_IND, KE_BUILD_ID(TASK_LC, acl_par->link_id), TASK_NONE);

        acl_par->sam_info->sam_change = ACL_SAM_STEP_NONE;
    }

    // If Remote SAM is enabled, calculate and apply the SAM offset for this event
    if (acl_par->sam_info->map_length)
    {
        DBG_SWDIAG(SAM, RMAP_OFFSET_CFG, 1);

        uint32_t master_clk = CLK_ADD_2(clk_prg, acl_par->last_sync_clk_off);
        uint32_t sam_offset;

        // update anchor_pt
        int32_t clk_diff = CLK_DIFF((acl_par->sam_info->anchor_pt << 1), master_clk);
        acl_par->sam_info->anchor_pt += ((clk_diff >> 1) / acl_par->sam_info->map_length) * acl_par->sam_info->map_length;

        // determine the SAM offset from ACL start
        sam_offset = CO_MOD((CLK_SUB(master_clk, (acl_par->sam_info->anchor_pt << 1)) >> 1), acl_par->sam_info->map_length);

        // Write the new SAM offset & length
        em_bt_sam_cntl_pack(cs_idx, acl_par->sam_info->map_length, sam_offset);

        DBG_SWDIAG(SAM, RMAP_OFFSET_CFG, 0);
    }

    // If neither peer or local SAM is enabled or pending update, can delete the SAM parameters
    if (!acl_par->sam_info->map_length && !acl_par->sam_info->peer_map_pending && (ACL_SAM_STEP_NONE == acl_par->sam_info->sam_change)
#if RW_BT_MWS_COEX
            && (acl_par->sam_info->loc_idx == SAM_DISABLED) && !acl_par->sam_info->local_map_pending
#endif //RW_BT_MWS_COEX
       )
    {
        // Delete SAM parameters
        ke_free(acl_par->sam_info);
        acl_par->sam_info = NULL;
    }
}


/**
 ****************************************************************************************
 * @brief Determines if a SAM update is pending
 ****************************************************************************************
 */
__STATIC bool ld_acl_sam_update_pending(struct ld_acl_env_tag *acl_par)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SAM_UPDATE_PENDING_3_FUNC_BIT, bool, acl_par);

    // check if a SAM configuraiton update is pending for peer map or local map
    return (acl_par->sam_info && (acl_par->sam_info->peer_map_pending
#if RW_BT_MWS_COEX
                                  || acl_par->sam_info->local_map_pending
#endif // RW_BT_MWS_COEX
                                 ));
}

/**
 ****************************************************************************************
 * @brief Program the SAM configuration
 ****************************************************************************************
 */
__STATIC void ld_acl_sam_config(uint8_t cs_idx, struct ld_acl_env_tag *acl_par, uint32_t clk_prg)
{
    uint32_t master_clk = CLK_ADD_2(clk_prg, acl_par->last_sync_clk_off);

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_SAM_CONFIG_2_FUNC_BIT, cs_idx, acl_par, clk_prg);

    // Pending peer map and instant reached
    if (acl_par->sam_info->peer_map_pending && (CLK_DIFF(master_clk, (acl_par->sam_info->instant << 1)) <= 0))
    {
        DBG_SWDIAG(SAM, CFG, 1);

        acl_par->sam_info->map_length = acl_par->sam_info->map_length_pending;

        if (acl_par->sam_info->map_length)
        {
            uint16_t sam_ptr = EM_BT_PEER_SAM_MAP_OFF(acl_par->link_id);
            ASSERT_ERR((sam_ptr & 0x03) == 0);

            // Activate pending configurations
            acl_par->sam_info->anchor_pt = acl_par->sam_info->anchor_pt_pending;

            // Write the SAM Map to EM
            em_wr((void *)&acl_par->sam_info->peer_map[0], sam_ptr, ((acl_par->sam_info->map_length + 3) >> 2));

            // Set pointer, and enable peer SAM on this link
            em_bt_sam_ptr_set(cs_idx, sam_ptr >> 2);

            em_bt_frcntl_sam_en_setf(cs_idx, LD_ACL_SAM_SLOT_BLOCKING_EN);
        }
        else
        {
            // Disable peer SAM in HW
            em_bt_frcntl_sam_en_setf(cs_idx, 0);
        }

        acl_par->sam_info->sam_change = ACL_SAM_STEP_CHANGE_PENDING;

        DBG_SWDIAG(SAM, RMAP_EN, ((acl_par->sam_info->map_length) ? 1 : 0));

        acl_par->sam_info->peer_map_pending = false;
    }

#if RW_BT_MWS_COEX
    // Pending local map and instant reached
    if (acl_par->sam_info->local_map_pending && (CLK_DIFF(master_clk, (acl_par->sam_info->instant << 1)) <= 0))
    {
        DBG_SWDIAG(SAM, CFG, 1);

        if (acl_par->sam_info->loc_idx != SAM_DISABLED)
        {
            //0xx: Let MWS_PATTERN input selection running
            //100: Force MWS Pattern register set 0 usage
            //101: Force MWS Pattern register set 1 usage
            //110: Force MWS Pattern register set 2 usage
            ASSERT_ERR(acl_par->sam_info->loc_idx < SAM_INDEX_MAX);
            em_bt_frcntl_fwmspatt_setf(cs_idx, 0x4 | acl_par->sam_info->loc_idx);

            // Enable local SAM on this link
            em_bt_frcntl_lsam_dsb_setf(cs_idx, RWIP_COEX_GET(CONNECT, SAMEN) ? 0 : 1);
        }
        else
        {
            // Disable local SAM on this link
            em_bt_frcntl_lsam_dsb_setf(cs_idx, 1);
        }

        acl_par->sam_info->sam_change = ACL_SAM_STEP_CHANGE_PENDING;

        DBG_SWDIAG(SAM, LMAP_EN, (acl_par->sam_info->loc_idx != SAM_DISABLED));

        acl_par->sam_info->local_map_pending = false;
    }
#endif //RW_BT_MWS_COEX

    DBG_SWDIAG(SAM, CFG, 0);
}

/**
 ****************************************************************************************
 * @brief Schedule next ACL activity
 ****************************************************************************************
 */
__STATIC void ld_acl_sched(uint8_t link_id)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint32_t limit = LD_CLOCK_UNDEF;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SCHED_3_FUNC_BIT, link_id);

    // Configure as default PTI priority for on connection
    acl_par->pti = BT_PTI_CONNECT_IDX;

    // If master and no traffic
    if ((acl_par->role == MASTER_ROLE) && !(acl_par->rx_traffic || acl_par->tx_traffic || acl_par->sco_hv1_idx))
    {
        // Set to active ACL priority as Tpoll is used
        evt->current_prio = rwip_priority[RWIP_PRIO_ACL_ACT_IDX].value;

        // Event priority and duration are set to the minimum
        evt->duration_min = 2 * LD_ACL_EVT_DUR_MIN_DFT;

        // Insert a duration of inactivity (Tpoll)
        evt->time.hs = CO_ALIGN4_LO(CLK_ADD_2(evt->time.hs, 2 * acl_par->t_poll));

        // Configure as poll interval PTI priority for on connection
        acl_par->pti = BT_PTI_POLLINT_IDX;
    }
    // If slave / traffic or SCO-HV1 => needs to fulfill the bandwidth
    else
    {
        /*
         * Duration and priority increased in case of traffic or slave operations
         * Take the event duration recommended by scheduling parameters block
         */
#if (EAVESDROPPING_SUPPORT)
        if (acl_par->cust_evt_dur_min)
        {
            // cust_evt_dur_min is in us
            evt->duration_min = 2 * acl_par->cust_evt_dur_min;
        }
        else
#endif // EAVESDROPPING_SUPPORT
        {
            evt->duration_min = sch_slice_params.acl_evt_dur;
        }

        /*
         * In case of slave needing to transmit/receive a packet, the event duration is increased for:
         *  - help transmitting the packet and receive the ACK on the subsequent frame
         *  - help transmitting/receiving a multi-slot packet
         */
        if ((acl_par->role == SLAVE_ROLE) && !acl_par->sco_ev3_retx_idx)
        {
            evt->duration_min = co_max(evt->duration_min, 2 * SLOT_SIZE * ((acl_par->tx_traffic + acl_par->rx_traffic) >> LD_ACL_TRAFFIC_DUR_INCR_FACTOR));
        }
    }

    // Check at what time a role switch may need to be prepared
    if (acl_par->mode == ACL_MODE_ROLE_SWITCH)
    {
        limit = CLK_SUB(acl_par->rsw_par->instant, acl_par->clk_off + 2 * LD_ACL_RSW_PREPARE_DELAY);
    }
    else if (acl_par->lsto != LSTO_OFF)
    {
        // Schedule ACL if currently before LSTO
        limit = CLK_ADD_3(acl_par->last_hec_ok_ts, 2 * acl_par->lsto, CO_DIVIDE_CEIL(evt->duration_min, HALF_SLOT_SIZE) + CO_ALIGN4_HI(rwip_prog_delay));
    }

    if (limit == LD_CLOCK_UNDEF)
    {
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, acl_par->phase, LD_ACL_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));
    }
    else
    {
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, acl_par->phase, LD_ACL_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));
        evt->asap_limit = limit;
    }

    // SAM Support
    if (NULL != acl_par->sam_info)
    {
        ld_acl_sam_config(EM_BT_CS_ACL_INDEX(link_id), acl_par, evt->time.hs);
    }

    // Try scheduling ACL before LSTO or Role Switch
    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
    {
        acl_par->state = ACL_EVT_WAIT;
    }
    else
    {
        // Check if a role switch has to be prepared
        if (acl_par->mode == ACL_MODE_ROLE_SWITCH)
        {
            // Prepare Role Switch
            ld_acl_rsw_start(link_id, CLK_ADD_2(limit, 2 * LD_ACL_RSW_PREPARE_DELAY));
        }
        else
        {
            // Report ACL end due to Connection Timeout
            ld_acl_end(link_id, CO_ERROR_CON_TIMEOUT);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Rechedule next ACL activity
 ****************************************************************************************
 */
__STATIC void ld_acl_resched(uint8_t link_id, uint32_t clock)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RESCHED_2_FUNC_BIT, link_id, clock);
    switch (acl_par->mode)
    {
    case ACL_MODE_NORMAL:
    case ACL_MODE_ROLE_SWITCH:
    {
        // Schedule normal ACL frame
        ld_acl_sched(link_id);
    }
    break;

    case ACL_MODE_SNIFF_ENTER:
    {
        // Enter sniff mode
        ld_acl_sniff_enter(link_id, clock);

        // Schedule the 1st sniff event
        ld_acl_sniff_sched(link_id);
    }
    break;

    case ACL_MODE_SNIFF_TRANS:
    {
        evt->current_prio = rwip_priority[RWIP_PRIO_ACL_SNIFF_TRANS_IDX].value;
        // Schedule with sniff transition mode rules
        ld_acl_sniff_trans_sched(link_id, clock);
    }
    break;

    default:
    {
        ASSERT_INFO_FORCE(0, link_id, acl_par->mode);
    }
    break;
    }
}

/**
 ****************************************************************************************
 * @brief Flush current buffers if expired
 ****************************************************************************************
 */
__STATIC void ld_acl_automatic_data_flush(uint8_t link_id, uint32_t clock)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_AUTOMATIC_DATA_FLUSH_BIT, link_id, clock);

    // Check the current buffer element
    if (acl_par->curr_buf_elt != NULL)
    {
        // Check the next buffer element
        if (acl_par->next_buf_elt != NULL)
        {
            if ((acl_par->next_buf_elt->l2cap_start_ts != LD_CLOCK_UNDEF) && (CLK_DIFF(acl_par->next_buf_elt->l2cap_start_ts, clock) >= 2 * acl_par->flush_to))
            {
                // Check if there is Tx buffer data coming from this element
                uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, !acl_par->txdesc_index_hw);
                uint16_t data_len = GETF(acl_par->next_buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);
                uint16_t txbufptr = em_bt_txaclbufptr_getf(txdesc_idx);
                if ((txbufptr >= acl_par->next_buf_elt->buf_ptr) && (txbufptr < (acl_par->next_buf_elt->buf_ptr + data_len)))
                {
                    em_bt_txpheader_txlength_setf(txdesc_idx, 0);
                    em_bt_txpheader_txllid_setf(txdesc_idx, LLID_CONTINUE);

                    if ((acl_par->state == ACL_EVT_WAIT) || (acl_par->mode == ACL_MODE_NORMAL))
                    {
                        // Set txdone
                        em_bt_txptr_txdone_setf(txdesc_idx, 1);
                        // Clear descriptor fields
                        em_bt_txheader_txtype_setf(txdesc_idx, 0);

                        // Update TX pointer and counter
                        acl_par->txdesc_cnt--;
                        DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);
                        acl_par->txdesc_index_sw = !acl_par->txdesc_index_sw;
                        DBG_SWDIAG(TX, SW_IDX, acl_par->txdesc_index_sw);
                    }
                }

                // Flush the buffer
                struct lc_acl_tx_cfm *msg = KE_MSG_ALLOC(LC_ACL_TX_CFM, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_acl_tx_cfm);
                msg->flushed = true;
                ke_msg_send(msg);

                // Free the buffer
                bt_util_buf_acl_tx_free(acl_par->next_buf_elt->buf_ptr);

                acl_par->next_buf_elt = NULL;
                acl_par->data_len = 0;
            }
        }

        if ((acl_par->curr_buf_elt->l2cap_start_ts != LD_CLOCK_UNDEF) && (CLK_DIFF(acl_par->curr_buf_elt->l2cap_start_ts, clock) >= 2 * acl_par->flush_to))
        {
            // Check if there is Tx buffer data coming from this element
            uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, acl_par->txdesc_index_hw);
            uint16_t data_len = GETF(acl_par->curr_buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);
            for (uint8_t i = 0; i < 2; i++)
            {
                uint16_t txbufptr = em_bt_txaclbufptr_getf(txdesc_idx);
                if ((txbufptr >= acl_par->curr_buf_elt->buf_ptr) && (txbufptr < (acl_par->curr_buf_elt->buf_ptr + data_len)))
                {
                    em_bt_txpheader_txlength_setf(txdesc_idx, 0);
                    em_bt_txpheader_txllid_setf(txdesc_idx, LLID_CONTINUE);

                    if ((acl_par->state == ACL_EVT_WAIT) || (acl_par->mode == ACL_MODE_NORMAL))
                    {
                        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);
                        bool waitack = em_bt_acltxstat_waitack_getf(cs_idx);

                        if ((i == 1) || !waitack)
                        {
                            // Set txdone
                            em_bt_txptr_txdone_setf(txdesc_idx, 1);
                            // Clear descriptor fields
                            em_bt_txheader_txtype_setf(txdesc_idx, 0);

                            // Update TX pointer and counter
                            acl_par->txdesc_cnt--;
                            DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);
                            acl_par->txdesc_index_sw = !acl_par->txdesc_index_sw;
                            DBG_SWDIAG(TX, SW_IDX, acl_par->txdesc_index_sw);
                        }
                    }
                }

                // Check next descriptor
                txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, !acl_par->txdesc_index_hw);
            }

            // Flush the buffer
            struct lc_acl_tx_cfm *msg = KE_MSG_ALLOC(LC_ACL_TX_CFM, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_acl_tx_cfm);
            msg->flushed = true;
            ke_msg_send(msg);

            // Free the buffer
            bt_util_buf_acl_tx_free(acl_par->curr_buf_elt->buf_ptr);

            if (acl_par->next_buf_elt == NULL)
            {
                acl_par->data_len = 0;
            }

            // Shift next pointer to current pointer
            acl_par->curr_buf_elt = acl_par->next_buf_elt;
            acl_par->next_buf_elt = NULL;
        }
    }
    else
    {
        ASSERT_ERR(acl_par->next_buf_elt == NULL);
    }
}

/**
 ****************************************************************************************
 * @brief Update test mode
 ****************************************************************************************
 */
__STATIC void ld_acl_test_mode_update(uint8_t link_id)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_TEST_MODE_UPDATE_3_FUNC_BIT, link_id);

    ASSERT_ERR(acl_par->test_mode != NULL);

    switch (acl_par->test_mode->action)
    {
    case TEST_MODE_NO_ACTION:
    {
        // Nothing to do
    }
    break;
    case TEST_MODE_ENTER:
    {
        if (acl_par->test_mode->em_buf == 0)
        {
            // Get a new buffer from the TX pool
            uint16_t buf_ptr = bt_util_buf_acl_tx_alloc();

            if (buf_ptr != 0)
            {
                // Save test buffer pointer
                acl_par->test_mode->em_buf = buf_ptr;

                // Point all common RX descriptors to the test buffer
                for (int i = 0 ; i < EM_NUM_BT_RXDESC ; i++)
                {
                    // Point to test buffer
                    em_bt_rxaclbufptr_setf(i, acl_par->test_mode->em_buf);
                }
            }
        }

        if (acl_par->test_mode->em_buf != 0)
        {
            // Disable direct loopback
            bt_rftestfreq_directloopbacken_setf(0);

            // Apply new static configuration
            switch (acl_par->test_mode->params.test_scenario)
            {
            case TXTEST0_MODE   :
            {
                em_set(0x00, acl_par->test_mode->em_buf, acl_par->test_mode->params.data_length);
                em_bt_linkcntl_whdsb_setf(cs_idx, 1);
            }
            break;
            case TXTEST1_MODE   :
            {
                em_set(0xFF, acl_par->test_mode->em_buf, acl_par->test_mode->params.data_length);
                em_bt_linkcntl_whdsb_setf(cs_idx, 1);
            }
            break;
            case TXTEST10_MODE  :
            {
                em_set(0x55, acl_par->test_mode->em_buf, acl_par->test_mode->params.data_length);
                em_bt_linkcntl_whdsb_setf(cs_idx, 1);
            }
            break;
            case TXTEST1100_MODE:
            {
                em_set(0x0F, acl_par->test_mode->em_buf, acl_par->test_mode->params.data_length);
                em_bt_linkcntl_whdsb_setf(cs_idx, 1);
            }
            break;
            case PRAND_MODE     :
            {
                // First initialize buffer with the 64 first bytes
                em_wr(prbs_64bytes, acl_par->test_mode->em_buf, 64);

                for (int i = 64 ; i < acl_par->test_mode->params.data_length ; i++)
                {
                    uint8_t temp1 = em_rd8p(acl_par->test_mode->em_buf + i - 64);
                    uint8_t temp2 = em_rd8p(acl_par->test_mode->em_buf + i - 63);

                    uint8_t temp = ((temp1 << 1) & 0xFF) + (temp2 >> 7);

                    em_wr8p(acl_par->test_mode->em_buf + i, temp);
                }
                for (int i = 0 ; i < acl_par->test_mode->params.data_length ; i++)
                {
                    uint8_t temp = em_rd8p(acl_par->test_mode->em_buf + i);

                    temp = ((temp & 0x01) << 7) |
                           ((temp & 0x02) << 5) |
                           ((temp & 0x04) << 3) |
                           ((temp & 0x08) << 1) |
                           ((temp & 0x10) >> 1) |
                           ((temp & 0x20) >> 3) |
                           ((temp & 0x40) >> 5) |
                           ((temp & 0x80) >> 7);

                    em_wr8p(acl_par->test_mode->em_buf + i, temp);
                }

                em_bt_linkcntl_whdsb_setf(cs_idx, 1);
            }
            break;
            case ACLNOWHIT_MODE :
            case ACLLOOP_MODE   :
            {
                if (acl_par->test_mode->params.test_scenario == ACLNOWHIT_MODE)
                {
                    em_bt_linkcntl_whdsb_setf(cs_idx, 1);
                }
                else
                {
                    em_bt_linkcntl_whdsb_setf(cs_idx, 0);
                }

                // Enable direct loopback
                bt_rftestfreq_directloopbacken_setf(1);
            }
            break;
            case SCONOWHIT_MODE :
            case SCOLOOP_MODE   :
            {
                uint8_t lt_addr = em_bt_linkcntl_aclltaddr_getf(cs_idx);
                uint8_t pkt_type = (acl_par->test_mode->params.packet_type & LMP_TEST_CTRL_PKT_TYPE_CODE_MSK) >> LMP_TEST_CTRL_PKT_TYPE_CODE_POS;
                uint8_t link_type = (acl_par->test_mode->params.packet_type & LMP_TEST_CTRL_PKT_TYPE_LINK_MSK) >> LMP_TEST_CTRL_PKT_TYPE_LINK_POS;
                uint8_t esco = ((link_type == TEST_ESCO) || (link_type == TEST_EDRESCO));
                uint8_t edresco = (link_type == TEST_EDRESCO);
                uint8_t voice_offset = 0;
                uint16_t data_length = acl_par->test_mode->params.data_length;

                if (acl_par->test_mode->params.test_scenario == SCONOWHIT_MODE)
                {
                    em_bt_linkcntl_whdsb_setf(cs_idx, 1);
                }
                else
                {
                    em_bt_linkcntl_whdsb_setf(cs_idx, 0);
                }

                // Disable reserved slot slave tx on error
                bt_rftestcntl_sserrren_setf(0);
                bt_rftestcntl_herrren_setf(0);

                // Configure (e)SCO Logical Transport
                bt_e_scoltcntl_pack(acl_par->link_id, 1, edresco, edresco, esco, lt_addr);

                // According to the spec, the length of the test sequence in not applicable to ACL and SCO loopback tests (set to 0)
                // Therefore, it has to be set by the SW for SCO loopback tests
                if (link_type == TEST_ACLSCO)
                {
                    switch (pkt_type)
                    {
                    case DV_TYPE  :
                    {
                        pkt_type = HV1_TYPE;
                        data_length = HV1_PACKET_SIZE;
                        // An offset of 32 is used for the voice data of the DV packet
                        voice_offset = 32;
                    }
                    break;
                    case HV1_TYPE :
                    {
                        data_length = HV1_PACKET_SIZE;
                    }
                    break;
                    case HV2_TYPE :
                    {
                        data_length = HV2_PACKET_SIZE;
                    }
                    break;
                    case HV3_TYPE :
                    {
                        data_length = HV3_PACKET_SIZE;
                    }
                    break;
                    default       :
                    {
                        ASSERT_ERR_FORCE(0);
                    }
                    break;
                    }
                }

                // Initialize and configure reception and transmission
                bt_e_scotrcntl_pack(acl_par->link_id, 0, data_length, pkt_type, data_length, pkt_type);

                // Set Tx and Rx pointers to the same buffer
                ASSERT_ERR(((acl_par->test_mode->em_buf + voice_offset) & 0x03) == 0);
                bt_e_scocurrenttxptr_pack(acl_par->link_id, (acl_par->test_mode->em_buf + voice_offset) >> 2, (acl_par->test_mode->em_buf + voice_offset) >> 2);
                bt_e_scocurrentrxptr_pack(acl_par->link_id, (acl_par->test_mode->em_buf + voice_offset) >> 2, (acl_par->test_mode->em_buf + voice_offset) >> 2);

                // Mute audio part
                bt_e_scomutecntl_pack(acl_par->link_id,
#if (EAVESDROPPING_SUPPORT)
                                      0, 0, 0, 0,
#endif // EAVESDROPPING_SUPPORT
                                      0,
                                      1,
                                      2,
                                      2,
                                      0);

                // Enable Voice channel
                bt_e_scochancntl_pack(acl_par->link_id, 0, 1, 0, 0, 2);

                // Enable direct loopback
                bt_rftestfreq_directloopbacken_setf(1);
            }
            break;
            default:
            {
                ASSERT_ERR_FORCE(0);
            }
            break;
            }

            switch ((acl_par->test_mode->params.packet_type & LMP_TEST_CTRL_PKT_TYPE_LINK_MSK) >> LMP_TEST_CTRL_PKT_TYPE_LINK_POS)
            {
            case TEST_ACLSCO:
            case TEST_ESCO:
            {
                em_bt_linkcntl_acledr_setf(cs_idx, 0);
            }
            break;
            case TEST_EDRACL:
            case TEST_EDRESCO:
            {
                em_bt_linkcntl_acledr_setf(cs_idx, 1);
            }
            break;

            default:
            {
                ASSERT_ERR_FORCE(0);
            }
            break;
            }

            if (acl_par->test_mode->params.hopping_mode == HOPSINGLE)
            {
                em_bt_pwrcntl_fh_en_setf(cs_idx, 0);

                /* Set used TX and RX frequencies in the RFTESTFREQ register        */
                /* The TXFREQ/RXFREQ field is the index in the frequency table.     */
                /* So the index programmed is:                                      */
                /*          * if freq is odd , index = 40 + freq/2                  */
                /*          * if freq is even, index = freq/2                       */
                if (acl_par->test_mode->params.tx_freq & 0x01)
                {
                    em_bt_pwrcntl_pack(cs_idx, 0,  40 + (acl_par->test_mode->params.tx_freq / 2), rwip_rf.txpwr_max);
                    bt_rftestfreq_txfreq_setf(40 + (acl_par->test_mode->params.tx_freq / 2));
                }
                else
                {
                    em_bt_pwrcntl_pack(cs_idx, 0, (acl_par->test_mode->params.tx_freq / 2), rwip_rf.txpwr_max);
                    bt_rftestfreq_txfreq_setf(acl_par->test_mode->params.tx_freq / 2);
                }
                if (acl_par->test_mode->params.rx_freq & 0x01)
                {
                    em_bt_pwrcntl_pack(cs_idx, 0,  40 + (acl_par->test_mode->params.tx_freq / 2), rwip_rf.txpwr_max);
                    bt_rftestfreq_rxfreq_setf(40 + (acl_par->test_mode->params.rx_freq / 2));
                }
                else
                {
                    em_bt_pwrcntl_pack(cs_idx, 0, (acl_par->test_mode->params.tx_freq / 2), rwip_rf.txpwr_max);
                    bt_rftestfreq_rxfreq_setf(acl_par->test_mode->params.rx_freq / 2);
                }
                rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rwip_rf.txpwr_max_mod, rwip_rf.txpwr_max);
            }
            else
            {
                em_bt_pwrcntl_fh_en_setf(cs_idx, 1);
            }
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }

        // Enable test mode
        bt_rftestfreq_testmodeen_setf(1);

        // Clear action flag
        acl_par->test_mode->action = TEST_MODE_NO_ACTION;
    }
    break;
    case TEST_MODE_EXIT:
    {
        // Disable direct loopback
        bt_rftestfreq_directloopbacken_setf(0);
        // Disable test mode
        bt_rftestfreq_testmodeen_setf(0);

        // Restore normal configuration
        em_bt_pwrcntl_fh_en_setf(cs_idx, 1);
        em_bt_linkcntl_whdsb_setf(cs_idx, 0);

        // Flush current test ACL data
        for (int i = 0 ; i < 2 ; i++)
        {
            uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, i);
            if (em_bt_txaclbufptr_getf(txdesc_idx))
            {
                em_bt_txpheader_txlength_setf(txdesc_idx, 0);
            }
        }

        // Release TX buffer
        bt_util_buf_acl_tx_free(acl_par->test_mode->em_buf);

        // Release memory
        if (acl_par->test_mode != NULL)
        {
            ke_free(acl_par->test_mode);
            acl_par->test_mode = NULL;
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

/**
 ****************************************************************************************
 * @brief Select a packet type for TX data
 ****************************************************************************************
 */
__STATIC uint8_t ld_acl_tx_packet_type_select(uint8_t link_id, uint16_t max_data_len,  uint16_t *tx_data_len)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint8_t idx = 0;
    uint8_t select = DM1_IDX;
    uint8_t tx_type;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_TX_PACKET_TYPE_SELECT_3_FUNC_BIT, uint8_t, link_id, max_data_len, tx_data_len);

    if (acl_par->edr_en)
    {
        // Find the smallest allowed EDR packet type that is large enough for the data to send
        for (idx = 0; idx < ARRAY_LEN(ld_acl_edr_sizes); idx++)
        {
            // Check if packet type is enabled
            if (acl_par->allowed_edr_packet_types & (1 << idx))
            {
                // Replace the selected index by the new one
                select = idx;

                // Check if the packet size is enough for the data to send
                if (ld_acl_edr_sizes[idx] >= max_data_len)
                {
                    break;
                }
            }
        }

        // Get packet type
        tx_type = ld_acl_edr_types[select];
        // Get packet size
        *tx_data_len = co_min(max_data_len, ld_acl_edr_sizes[select]);
    }
    else
    {
        // Find the smallest allowed BR packet type that is large enough for the data to send
        for (idx = 0; idx < ARRAY_LEN(ld_acl_br_sizes); idx++)
        {
            // Check if packet type is enabled
            if (acl_par->allowed_br_packet_types & (1 << idx))
            {
                // Replace the selected index by the new one
                select = idx;

                // Check if the packet size is enough for the data to send
                if (ld_acl_br_sizes[idx] >= max_data_len)
                {
                    break;
                }
            }
        }

        // Get packet type
        tx_type = ld_acl_br_types[select];
        // Get packet size
        *tx_data_len = co_min(max_data_len, ld_acl_br_sizes[select]);
    }

    return tx_type;
}

/**
 ****************************************************************************************
 * @brief Adjust the slave's anchor points and event timings to new phase
 *
 * Note: Phase is re-aligned at both no_sync and sync updates, while notifications
 * to LC are sent only after phase updates are confirmed on a sync update.
 ****************************************************************************************
 */
__STATIC void ld_acl_phase_align(uint8_t link_id, uint32_t old_clk_off, bool refresh_phase, uint32_t new_clk_off, int16_t new_bit_off, bool notify)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_PHASE_ALIGN_2_FUNC_BIT, 6, link_id, old_clk_off, refresh_phase, new_clk_off, new_bit_off, notify);

    // Update sniff anchor point if needed
    if (acl_par->mode == ACL_MODE_SNIFF)
    {
        // Update timings if required
        if (refresh_phase)
        {
            acl_par->sniff_par.anchor_point = CLK_SUB(acl_par->sniff_par.anchor_point, new_clk_off);
            acl_par->sniff_par.anchor_point = CLK_ADD_2(acl_par->sniff_par.anchor_point, old_clk_off);
        }

        // Send the sniff offset update indication
        if (notify)
        {
            struct lc_sniff_offset_upd_ind *msg = KE_MSG_ALLOC(LC_SNIFF_OFFSET_UPD_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_sniff_offset_upd_ind);
            msg->sniff_offset = CO_MOD(acl_par->sniff_par.anchor_point, 2 * acl_par->sniff_par.intv);
            ke_msg_send(msg);
        }
    }

#if MAX_NB_SYNC
    // Look for SCO connections on the link
    for (int sco_link_id = (MAX_NB_SYNC - 1) ; sco_link_id >= 0 ; sco_link_id--)
    {
        if (ld_sco_env[sco_link_id] != NULL)
        {
            // Point to parameters
            struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

            if (sco_par->link_id == link_id)
            {
                // Update timings if required
                if (refresh_phase)
                {
                    struct sch_arb_elt_tag *sco_evt = &(sco_par->evt);

                    // Adjust the anchor point by the clock offsets difference
                    sco_par->anchor_point = CLK_SUB(sco_par->anchor_point, new_clk_off);
                    sco_par->anchor_point = CLK_ADD_2(sco_par->anchor_point, old_clk_off);

                    // Remove event
                    sch_arb_remove(sco_evt, false);

                    // Update event timings used for scheduling
                    sco_evt->time.hs = CLK_SUB(sco_evt->time.hs, new_clk_off);
                    sco_evt->time.hs = CLK_ADD_2(sco_evt->time.hs, old_clk_off);
                    sco_evt->time.hus = new_bit_off;

                    if (sco_par->t_esco > 2)
                    {
                        // Directly reschedule the next interval
                        ld_sco_sched(sco_par->sco_link_id);
                    }
                }

                // Send the SCO offset update indication
                if (notify)
                {
                    struct lc_sco_offset_upd_ind *msg = KE_MSG_ALLOC(LC_SCO_OFFSET_UPD_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_sco_offset_upd_ind);
                    msg->sco_link_id = sco_par->sco_link_id;
                    msg->sco_offset = CO_MOD(sco_par->anchor_point, 2 * sco_par->t_esco);
                    ke_msg_send(msg);
                }
            }
        }
    }
#endif //MAX_NB_SYNC
}

/**
 ****************************************************************************************
 * @brief Synchronize slave after correct reception
 *
 * Note: this algorithm supports both large sync windows and multi-attempts
 ****************************************************************************************
 */
__STATIC bool ld_acl_rx_sync(uint8_t link_id, uint32_t timestamp, uint16_t rxbit)
{
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RX_SYNC_2_FUNC_BIT, bool, link_id, timestamp, rxbit);

    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    uint8_t old_phase = acl_par->phase;
    uint32_t old_rx_win_size = acl_par->rx_win_size;
    uint32_t old_clk_off = acl_par->clk_off;

    // Read clock value where sync has been found (in half-slots)
    uint32_t rxclkn = (em_bt_rxclkn1_getf(cs_idx) << 16) | em_bt_rxclkn0_getf(cs_idx);
    // Get estimated clock offset (in half-slots)
    uint32_t est_clk_off = (em_bt_clkoff1_getf(cs_idx) << 16) + em_bt_clkoff0_getf(cs_idx);

    uint32_t new_clk_off;
    int16_t new_bit_off;

    // Check if there can be multiple attempts
    // If so, move the timestamp close to the rxclkn in steps of 2 slots (4 half-slots)
    // The clock offset does not change during this operation
    if (old_rx_win_size < 2 * HALF_SLOT_SIZE)
    {
        ASSERT_ERR(CLK_DIFF(timestamp, rxclkn) >= 0);
        int32_t diff = CLK_SUB(rxclkn, timestamp) / 4;

        if (diff > 0)
        {
            timestamp = CLK_ADD_2(timestamp, diff * 4);
        }
    }

    // Compute new clock offset
    new_clk_off = CLK_SUB(CLK_ADD_2(est_clk_off, timestamp), rxclkn);

    // Compute new bit offset
    new_bit_off = (rxbit - 2 * ld_env.exp_sync_pos);

    // Adjust bit offset and clock offset if needed
    if (new_bit_off < 0)
    {
        new_bit_off += HALF_SLOT_SIZE;
        new_clk_off = CLK_ADD_2(new_clk_off, 1);
    }
    else if (new_bit_off >= HALF_SLOT_SIZE)
    {
        new_bit_off -= HALF_SLOT_SIZE;
        new_clk_off = CLK_SUB(new_clk_off, 1);
    }

    // Update timing values
    acl_par->clk_off = new_clk_off;
    acl_par->bit_off = new_bit_off;
    acl_par->phase = (4 - (acl_par->clk_off & 0x03)) & 0x03;
    DBG_SWDIAG(ACL, PARITY, acl_par->phase);
    acl_par->rx_win_size = 2 * NORMAL_WIN_SIZE;

    // Update event delay used for scheduling
    evt->time.hus = new_bit_off;

    // Update stop latencies based on rx_win_size reduct
    if ((old_rx_win_size > 2 * NORMAL_WIN_SIZE) && (evt->stop_latency != 0))
    {
        evt->stop_latency = LD_ACL_EVT_STOP_THR_MIN + (evt->time.hus > LD_ACL_STOP_NOTIF_BIT_OFF_THR);
    }

    // Check if clock offset has changed
    bool notify = (new_clk_off != acl_par->last_sync_clk_off);

    bool refresh_phase = ((old_phase != acl_par->phase) || (old_rx_win_size >= 2 * HALF_SLOT_SIZE));
    if (refresh_phase || notify)
    {
        ld_acl_phase_align(link_id, old_clk_off, refresh_phase, new_clk_off, new_bit_off, notify);
    }

    // Save the values corresponding the last detected sync
    acl_par->last_sync_clk_off = new_clk_off;
    acl_par->last_sync_bit_off = new_bit_off;

#if PCA_SUPPORT && RW_BT_MWS_COEX
    // Update slave target offset based on realigned offset
    ld_pca_update_target_offset(acl_par->link_id);
#endif //PCA_SUPPORT && RW_BT_MWS_COEX

    // Check if link is waiting for role switch
    if (acl_par->mode == ACL_MODE_ROLE_SWITCH)
    {
        int16_t bit_offset_diff = co_abs(acl_par->rsw_bit_off - new_bit_off);

        if (bit_offset_diff > ((HALF_SLOT_SIZE + 1) >> 1))
        {
            bit_offset_diff = HALF_SLOT_SIZE - bit_offset_diff;
        }

        // Check if the new bit offset drift is above notification threshold
        if ((bit_offset_diff >= 2 * LD_ACL_RSW_DRIFT_NOTIF_THR) && (CLK_DIFF(CLK_ADD_2(timestamp, acl_par->clk_off), acl_par->rsw_par->instant) > 2 * LD_ACL_RSW_DRIFT_NOTIF_DELAY))
        {
            ke_msg_send_basic(LC_RSW_SLOT_OFFSET_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
        }
    }

#if PCA_SUPPORT
    // In synchronization scan recovery mode, but sync is recovered
    if (acl_par->sscan_clk_recov)
    {
        // Stop synchronization scan request
        ke_msg_send_basic(LC_PCA_SSCAN_STOP_REQ, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
        acl_par->sscan_clk_recov = false;
    }
#endif // PCA_SUPPORT

    return (old_phase != acl_par->phase);
}

/**
 ****************************************************************************************
 * @brief Adjust the slave's Rx window size if no sync has been found
 ****************************************************************************************
 */
__STATIC bool ld_acl_rx_no_sync(uint8_t link_id, uint32_t timestamp)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint8_t old_phase = acl_par->phase;
    uint32_t old_clk_off = acl_par->clk_off;

    // Calculate the new Rx window size
    int16_t new_bit_off = acl_par->last_sync_bit_off;
    uint32_t new_clk_off = acl_par->last_sync_clk_off;
    uint32_t clock_diff = CLK_SUB(timestamp, acl_par->last_sync_ts);

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RX_NO_SYNC_2_FUNC_BIT, bool, link_id, timestamp);

#if PCA_SUPPORT
    // slave scans for Syncs Train when it does not receive any communications from the Master
    if ((clock_diff > LD_ACL_TO_SYNC_SCAN) && !acl_par->sscan_clk_recov)
    {
        // Start synchronization scan request
        ke_msg_send_basic(LC_PCA_SSCAN_START_REQ, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
        acl_par->sscan_clk_recov = true;
    }
#endif // PCA_SUPPORT

    // Compute the time uncertainty considering maximum drift between 2 devices
    uint32_t max_drift = clock_diff * (rwip_current_drift_get() + BT_MAX_DRIFT_ACTIVE) * SLOT_SIZE / 1000000;

    // The RX window size include the uncertainty window on each side of the theoretical sync position (before and after)
    uint32_t rx_win_size = 2 * max_drift + 2 * NORMAL_WIN_SIZE;

    // The RX window hslot offset extension due to RX window size.
    uint8_t rx_win_hslot_off = 0;
    // The normal window size is automatically applied by the hardware, so it needs to be subtracted from the calculated Rx window size
    new_bit_off -= (rx_win_size - 2 * NORMAL_WIN_SIZE) / 2;
    if (new_bit_off < 0)
    {
        uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
        new_bit_off += num_hs * HALF_SLOT_SIZE;
        new_clk_off = CLK_ADD_2(new_clk_off, num_hs);
        rx_win_hslot_off += num_hs;
    }

    // Update timing values
    acl_par->clk_off = new_clk_off;
    acl_par->bit_off = new_bit_off;
    acl_par->phase = (4 - (acl_par->clk_off & 0x03)) & 0x03;
    acl_par->rx_win_size = rx_win_size;
    DBG_SWDIAG(ACL, PARITY, acl_par->phase);

    // Update event delay used for scheduling
    evt->time.hus = new_bit_off;

    // Update stop latencies based on rx_win_size extend
    if ((rx_win_hslot_off != 0) && (evt->stop_latency != 0))
    {
        evt->stop_latency = LD_ACL_EVT_STOP_THR_MIN + rx_win_hslot_off + (evt->time.hus > LD_ACL_STOP_NOTIF_BIT_OFF_THR);
    }

    // Check if clock offset has changed
    if (old_phase != acl_par->phase)
    {
        ld_acl_phase_align(link_id, old_clk_off, true, new_clk_off, new_bit_off, false);
    }

    return (old_phase != acl_par->phase);
}

/**
 ****************************************************************************************
 * @brief Check the reception during activity
 ****************************************************************************************
 */
__STATIC void ld_acl_rx(uint8_t link_id, uint32_t clock, uint8_t nb_rx)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RX_2_FUNC_BIT, link_id, clock, nb_rx);

    // Check if a packet has been received
    while ((nb_rx-- > 0) && ld_rxdesc_check(EM_BT_CS_ACL_INDEX(link_id)))
    {
        // Retrieve RX status
        uint16_t rxstat = em_bt_rxstat_get(ld_env.curr_rxdesc_index);
        uint16_t rxheader = em_bt_rxheader_get(ld_env.curr_rxdesc_index);
        uint8_t rxtype = (rxheader & EM_BT_RXTYPE_MASK) >> EM_BT_RXTYPE_LSB;
        uint16_t rxpheader = em_bt_rxpheader_get(ld_env.curr_rxdesc_index);
        uint8_t rxllid = (rxpheader & EM_BT_RXLLID_MASK) >> EM_BT_RXLLID_LSB;
        uint16_t rxchass = em_bt_rxchass_get(ld_env.curr_rxdesc_index);
        uint8_t rxrssi = (rxchass & EM_BT_RXRSSI_MASK) >> EM_BT_RXRSSI_LSB;
        uint8_t rxchannel = (rxchass & EM_BT_RXCHANNEL_MASK) >> EM_BT_RXCHANNEL_LSB;
        uint8_t rxltaddr = (rxheader & EM_BT_RXLTADDR_MASK) >> EM_BT_RXLTADDR_LSB;

        uint8_t rx_status = RWIP_RX_OTHER_ERROR;
        bool rx_audio = ((rxchass & (EM_BT_IS_AUDIO_BIT)) != 0x0000);

        do
        {
#if (EAVESDROPPING_SUPPORT)
            // If Rx statistics are required
            if (acl_par->rx_stats_rep_en)
            {
                // If the configured connection type matches the received packet type (ACL or eSCO)
                if (((acl_par->con_type == ED_CON_TYPE_ACL) && !rx_audio)
                        || ((acl_par->con_type == ED_CON_TYPE_ESCO) && rx_audio))
                {
                    // If there is no sync error
                    if (!(rxstat & EM_BT_RXSYNCERR_BIT))
                    {
                        int8_t rx_rssi = LD_RSSI_CONVERT(rxrssi);
                        uint8_t rx_succ = 0;
                        bool no_crc_type = ((rxtype == POLL_TYPE) || (rxtype == ID_NUL_TYPE));

                        // Packet received with HEC error or CRC error
                        if ((rxstat & EM_BT_RXHECERR_BIT) || (!no_crc_type && (rxstat & EM_BT_RXCRCERR_BIT)))
                        {
                            rx_succ = 0;
                        }
                        // Non-CRC packet received with correct HEC
                        else if (!(rxstat & EM_BT_RXHECERR_BIT) && no_crc_type)
                        {
                            rx_succ = 1;
                        }
                        // CRC packet received with correct HEC and correct CRC
                        else if (!(rxstat & (EM_BT_RXHECERR_BIT | EM_BT_RXCRCERR_BIT)))
                        {
                            // If the packet has already been received
                            rx_succ = (rxstat & (EM_BT_RXSEQERR_BIT | EM_BT_RXDUPERR_BIT)) ? 2 : 3;
                        }
                        else
                        {
                            ASSERT_ERR_FORCE(0);
                        }

                        // RSSI values within [-35, -98] dBm are translated to rx_rssi values within [0, 63]
                        // Values outside that range are saturated to either 0 or 63
                        {
                            int max = ((-rx_rssi) > 35) ? (-rx_rssi) : 35;
                            rx_rssi = co_min(max, 98) - 35;
                        }

                        // Save rx_stats[i]
                        acl_par->rx_stats[acl_par->rx_sync] = (rx_succ << 6) | rx_rssi;

                        // Increment rx_sync
                        acl_par->rx_sync++;
                    }

                    // Increment total number of reception attempts
                    acl_par->rx_tot++;
                    ASSERT_ERR(acl_par->rx_tot <= HCI_ECI_RX_STATS_LEN_MAX);
                }
            }
#endif // EAVESDROPPING_SUPPORT

            // Check synchronization status
            if (rxstat & EM_BT_RXSYNCERR_BIT)
            {
                rx_status = RWIP_RX_NO_SYNC;
                break;
            }

            // Update last sync timestamp
            acl_par->last_sync_ts = clock;

            if (rxltaddr != LT_ADDR_BCST) // Do not accumulate RSSI for broadcast receive
            {
                // Accumulate RSSI
                acl_par->rssi_acc += LD_RSSI_CONVERT(rxrssi);
                acl_par->rssi_avg_cnt++;
            }

            // Check packet header status
            if (rxstat & EM_BT_RXHECERR_BIT)
                break;

            // Update last HEC OK timestamp
            acl_par->last_hec_ok_ts = clock;

            // Check packet header is targeting local device
            if ((rxstat & EM_BT_RXBADLT_BIT) && (rxltaddr != LT_ADDR_BCST))
            {
                rx_status = RWIP_RX_OK;
                break;
            }

            // Check POLL/NULL reception
            if (rxtype == POLL_TYPE || rxtype == ID_NUL_TYPE)
            {
                // Set rx_traffic to 0
                acl_par->rx_traffic = 0;
                DBG_SWDIAG(ACL_RX, TRAFFIC, acl_par->rx_traffic);

                rx_status = RWIP_RX_OK;
                break;
            }

            /* If ACL or control packet under reception, increase RX traffic weight, to get more attention to this
             * link up to successful reception */
            if (!rx_audio && (acl_par->rx_traffic < LD_ACL_TRAFFIC_MAX))
            {
                acl_par->rx_traffic++;
                DBG_SWDIAG(ACL_RX, TRAFFIC, acl_par->rx_traffic);
            }

            // Check if packet has been already received
            if (rxstat & EM_BT_RXSEQERR_BIT)
            {
                rx_status = RWIP_RX_OK;
                break;
            }

            // Check payload status
            if (rxstat & (EM_BT_RXFECERR_BIT | EM_BT_RXGUARDERR_BIT))
                break;

            // Check CRC status
            if (rxstat & EM_BT_RXCRCERR_BIT)
            {
#if (EAVESDROPPING_SUPPORT)
                // Check ACL reception
                if ((acl_par->test_mode == NULL) && acl_par->corr_acl_payl_en && ((rxllid == LLID_CONTINUE) || (rxllid == LLID_START)))
                {
                    {
                        struct ed_acl_corr_payl_rx_ind *msg = KE_MSG_ALLOC(ED_ACL_CORR_PAYL_RX_IND, TASK_ED, TASK_NONE, ed_acl_corr_payl_rx_ind);
                        msg->link_id = link_id;
                        msg->em_buf = em_bt_rxaclbufptr_getf(ld_env.curr_rxdesc_index);
                        msg->data_len_flags = 0;
                        SETF(msg->data_len_flags, BT_EM_ACL_DATA_LEN, GETF(rxpheader, EM_BT_RXLENGTH));
                        SETF(msg->data_len_flags, BT_EM_ACL_PBF, rxllid);
                        SETF(msg->data_len_flags, BT_EM_ACL_BF, (rxltaddr == LT_ADDR_BCST) ? BCF_ACTIVE_SLV_BCST : BCF_P2P);
                        ke_msg_send(msg);
                    }

                    // Clear ACL buffer pointer
                    em_bt_rxaclbufptr_setf(ld_env.curr_rxdesc_index, 0);
                }
#endif // EAVESDROPPING_SUPPORT

                rx_status = RWIP_RX_CRC_ERROR;
                break;
            }

            // Check encryption status (valid only for AES-CCM encryption)
            if (rxstat & (EM_BT_RXMICERR_BIT))
            {
#if EAVESDROPPING_SUPPORT
                if (!(rxstat & EM_BT_RXDUPERR_BIT))
                {
                    // Increment number of consecutive MIC errors
                    acl_par->mic_err_cnt++;
                    if (!acl_par->tolerate_invalid_mic)
                    {
                        // Report the failure to LC
                        ke_msg_send_basic(LC_ACL_RX_MIC_ERR_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
                    }

                    if (!rx_audio)
                    {
                        if (acl_par->gen_mic_invalid_event && !acl_par->mic_invalid_event_block)
                        {
                            acl_par->mic_invalid_event_block = true;

                            // Report the failure to ED task
                            {
                                struct ed_mic_invalid_ind *ind = KE_MSG_ALLOC(ED_MIC_INVALID_IND, TASK_ED, TASK_NONE, ed_mic_invalid_ind);
                                ind->link_id = link_id;
                                ke_msg_send(ind);
                            }
                        }

                        if (acl_par->increase_counter && !acl_par->aes_ccm_cnt_correction)
                        {
                            bool carry = false;

                            uint16_t rx_ccm_pld_cntr0 = em_bt_rxccmpldcnt0_get(link_id);
                            carry = (rx_ccm_pld_cntr0 == 0xFFFF);
                            rx_ccm_pld_cntr0++;
                            em_bt_rxccmpldcnt0_set(link_id, rx_ccm_pld_cntr0);
                            if (carry)
                            {
                                uint16_t rx_ccm_pld_cntr1 = em_bt_rxccmpldcnt1_get(link_id);
                                carry = (rx_ccm_pld_cntr1 == 0xFFFF);
                                rx_ccm_pld_cntr1++;
                                em_bt_rxccmpldcnt1_set(link_id, rx_ccm_pld_cntr1);
                                if (carry)
                                {
                                    uint16_t rx_ccm_pld_cntr2 = em_bt_rxccmpldcnt2_get(link_id);
                                    rx_ccm_pld_cntr2 = (rx_ccm_pld_cntr2 + 1) & 0x00FF;
                                    em_bt_rxccmpldcnt2_set(link_id, (rx_ccm_pld_cntr2 + 1));
                                }
                            }
                        }

                        if (acl_par->tolerate_invalid_mic)
                        {
                            if ((acl_par->n_refr != 0) && (acl_par->mic_err_cnt == acl_par->n_refr))
                            {
                                // Request to refresh encryption key
                                ke_msg_send_basic(LC_MIC_ENC_KEY_REFRESH, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
                            }
                            else if ((acl_par->n_disc != 0) && (acl_par->mic_err_cnt == acl_par->n_disc))
                            {
                                // Request to disconnect
                                ke_msg_send_basic(LC_MIC_DISC, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
                            }
                        }
                    }

                    // If the aes_ccm_cnt_correction flag is set and MIC error without duplicate error
                    if (acl_par->aes_ccm_cnt_correction)
                    {
                        bool carry = false;

                        // Decrement counter
                        uint16_t rx_ccm_pld_cntr0 = em_bt_rxccmpldcnt0_get(link_id);
                        carry = (rx_ccm_pld_cntr0 == 0);
                        rx_ccm_pld_cntr0--;
                        em_bt_rxccmpldcnt0_set(link_id, rx_ccm_pld_cntr0);
                        if (carry)
                        {
                            uint16_t rx_ccm_pld_cntr1 = em_bt_rxccmpldcnt1_get(link_id);
                            carry = (rx_ccm_pld_cntr1 == 0);
                            rx_ccm_pld_cntr1--;
                            em_bt_rxccmpldcnt1_set(link_id, rx_ccm_pld_cntr1);
                            if (carry)
                            {
                                uint16_t rx_ccm_pld_cntr2 = em_bt_rxccmpldcnt2_get(link_id);
                                rx_ccm_pld_cntr2 = (rx_ccm_pld_cntr2 - 1) & 0x00FF;
                                em_bt_rxccmpldcnt2_set(link_id, rx_ccm_pld_cntr2);
                            }
                        }

                        // Clear flag
                        acl_par->aes_ccm_cnt_correction = false;
                    }
                }
                else // (rxstat & EM_BT_RXDUPERR_BIT)
                {
                    // If the aes_ccm_cnt_correction flag is set and MIC error with duplicate error
                    if (acl_par->aes_ccm_cnt_correction)
                    {
                        // Clear flag
                        acl_par->aes_ccm_cnt_correction = false;
                    }
                }
#else
                // Report the failure to LC
                ke_msg_send_basic(LC_ACL_RX_MIC_ERR_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
#endif //EAVESDROPPING_SUPPORT

                break;
            }

            rx_status = RWIP_RX_OK;

#if EAVESDROPPING_SUPPORT
            acl_par->mic_invalid_event_block = false;

            // Reset number of consecutive MIC errors
            acl_par->mic_err_cnt = 0;

            // Check if the link is currently under binaural ACK
            if (link_id == GETF(ld_aes_ccm_cnt, ED_AES_CCM_CNT_LINK_ID))
            {
                // Unblock AES-CCM counter reporting
                SETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_BLOCK, 0);
            }

            if (acl_par->gen_aes_ccm_counter_event)
            {
                // Packet correctly received, read AES CCM counter and send the event to the ED task
                uint8_t link_id = GETF(ld_aes_ccm_cnt, ED_AES_CCM_CNT_LINK_ID);
                uint8_t con_type = GETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_CON_TYPE);

                struct ed_aes_ccm_cntr_ind *msg = KE_MSG_ALLOC(ED_AES_CCM_CNTR_IND, TASK_ED, TASK_NONE, ed_aes_ccm_cntr_ind);
                ld_acl_get_aes_ccm_payload_counter(con_type, link_id, &msg->cntr[0]);
                msg->link_id = link_id;
                msg->con_type = con_type;
                msg->mic_ok = true;
                ke_msg_send(msg);
            }

            // If the aes_ccm_cnt_correction flag is set and MIC ok
            if (acl_par->aes_ccm_cnt_correction)
            {
                // Clear flag
                acl_par->aes_ccm_cnt_correction = false;
            }

            // No counter update needed since reception was ok
            if (acl_par->aes_ccm_cnt_upd)
            {
                acl_par->aes_ccm_cnt_upd = false;
            }
#endif //EAVESDROPPING_SUPPORT

            // Audio is not processed here
            if (rx_audio)
                break;

            // If ACL or control packet received, reset RX traffic weight to the default value
            if (!rx_audio && (acl_par->rx_traffic < LD_ACL_TRAFFIC_MAX))
            {
                acl_par->rx_traffic = LD_ACL_RX_TRAFFIC_DFT;
                DBG_SWDIAG(ACL_RX, TRAFFIC, acl_par->rx_traffic);
            }

            DBG_SWDIAG(BCST, PDU_RX, (rxltaddr == LT_ADDR_BCST));

            // Check LMP reception
            if ((rxtype == DM1_TYPE || rxtype == DV_TYPE) && (rxllid == LLID_CNTL))
            {
                DBG_SWDIAG(DATA, LMP_RX, 1);

                uint16_t em_buf = em_bt_rxlmbufptr_getf(ld_env.curr_rxdesc_index);
                uint16_t length = (rxpheader & (EM_BT_RXLENGTH_MASK)) >> EM_BT_RXLENGTH_LSB;

                struct lc_lmp_rx_ind *msg = KE_MSG_ALLOC_DYN(LC_LMP_RX_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_lmp_rx_ind, length);
                msg->lmp_len_flags = length & BT_RX_LMP_LEN_MASK;
                SETB(msg->lmp_len_flags, BT_RX_LMP_BF, (rxltaddr == LT_ADDR_BCST));
                em_rd(msg->pdu, em_buf, length);
                ke_msg_send(msg);

                DBG_SWDIAG(DATA, LMP_RX, 0);

                break;
            }

            // Check ACL reception
            if ((acl_par->test_mode == NULL) && ((rxllid == LLID_CONTINUE) || (rxllid == LLID_START)))
            {
                DBG_SWDIAG(DATA, ACL_RX, 1);

#if (EAVESDROPPING_SUPPORT)
                if (!acl_par->ignore_seqn_errors || !em_bt_rxstat_rxduperr_getf(ld_env.curr_rxdesc_index))
#endif // EAVESDROPPING_SUPPORT
                {
                    struct lc_acl_rx_ind *msg = KE_MSG_ALLOC(LC_ACL_RX_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_acl_rx_ind);
                    msg->em_buf = em_bt_rxaclbufptr_getf(ld_env.curr_rxdesc_index);
                    msg->data_len_flags = 0;
                    SETF(msg->data_len_flags, BT_EM_ACL_DATA_LEN, GETF(rxpheader, EM_BT_RXLENGTH));
                    SETF(msg->data_len_flags, BT_EM_ACL_PBF, rxllid);
                    SETF(msg->data_len_flags, BT_EM_ACL_BF, (rxltaddr == LT_ADDR_BCST) ? BCF_ACTIVE_SLV_BCST : BCF_P2P);
                    ke_msg_send(msg);

                    // Clear ACL buffer pointer
                    em_bt_rxaclbufptr_setf(ld_env.curr_rxdesc_index, 0);
                }

                DBG_SWDIAG(DATA, ACL_RX, 0);

                break;
            }

        }
        while (0);

        DBG_SWDIAG(BCST, PDU_RX, 0);

        // Free RX descriptor
        ld_rxdesc_free();

        rwip_channel_assess_bt(rxchannel, rx_status, rxrssi, clock);
    }

    // If slave not synced since a while, master needs to insist on that link
    if ((CLK_SUB(clock, acl_par->last_sync_ts) > LD_ACL_FPOLL_LIMIT_DFT) && (acl_par->rx_traffic < LD_ACL_TRAFFIC_MAX) && (acl_par->mode == ACL_MODE_NORMAL))
    {
        acl_par->rx_traffic++;
        DBG_SWDIAG(ACL_RX, TRAFFIC, acl_par->rx_traffic);
    }

#if (BT_PWR_CTRL)
    // If the number RSSI accumulated is sufficient to interpret
    if ((acl_par->rssi_avg_cnt > LD_ACL_RSSI_AVG_NB_PKT) && (acl_par->test_mode == NULL))
    {
        // Clear the last RSSI delta value
        acl_par->last_rssi_delta = 0;

        // Compare the average RSSI with thresholds (RF dependent)
        if (acl_par->rssi_acc < (rwip_rf.rssi_low_thr * acl_par->rssi_avg_cnt))
        {
            if (acl_par->rssi_acc < ((rwip_rf.rssi_low_thr - LD_ACL_RSSI_BELOW_LOW_THR) * acl_par->rssi_avg_cnt))
            {
                // Request to go to max power
                ke_msg_send_basic(LC_PWR_MAX_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
            }
            else
            {
                // Request to increase power
                ke_msg_send_basic(LC_PWR_INCR_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
            }

            // Compute the delta from low threshold
            acl_par->last_rssi_delta = (int8_t)(acl_par->rssi_acc / acl_par->rssi_avg_cnt) - rwip_rf.rssi_low_thr;
        }
        else if (acl_par->rssi_acc > (rwip_rf.rssi_high_thr * acl_par->rssi_avg_cnt))
        {
            // Request to decrease power
            ke_msg_send_basic(LC_PWR_DECR_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);

            // Compute the delta from high threshold
            acl_par->last_rssi_delta = (int8_t)(acl_par->rssi_acc / acl_par->rssi_avg_cnt) - rwip_rf.rssi_high_thr;
        }

        // Clear RSSI average data
        acl_par->rssi_avg_cnt = 0;
        acl_par->rssi_acc = 0;
    }
#endif //(BT_PWR_CTRL)
}

/**
 ****************************************************************************************
 * @brief Program new transmissions (LMP or ACL data)
 ****************************************************************************************
 */
__STATIC void ld_acl_tx_prog(uint8_t link_id, uint32_t clock)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_TX_PROG_3_FUNC_BIT, link_id, clock);
#if (EAVESDROPPING_SUPPORT)
    if (!acl_par->control_tx_prog || (acl_par->control_tx_prog && acl_par->tx_prog_en))
#endif // EAVESDROPPING_SUPPORT
    {
        // Try to fill all available descriptors with new data
        while ((acl_par->txdesc_cnt < 2) && ((acl_par->mode != ACL_MODE_ROLE_SWITCH) || (acl_par->rsw_par->step == ACL_RSW_STEP_PENDING)))
        {
            uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, acl_par->txdesc_index_sw);
            uint8_t new_txllid = LLID_CNTL;

            // Check LMP TX queue
            struct bt_em_lmp_buf_elt *lmp_buf_elt = (struct bt_em_lmp_buf_elt *) co_list_pop_front(&acl_par->queue_lmp_tx);
            if (lmp_buf_elt)
            {
                // Fill descriptor
                em_bt_txheader_txtype_setf(txdesc_idx, DM1_TYPE);
                em_bt_txheader_txdv_setf(txdesc_idx, (acl_par->sco_hv1_idx && ld_acl_lmp_txdv(lmp_buf_elt->buf_ptr, lmp_buf_elt->length)));
                em_bt_txpheader_pack(txdesc_idx, 0, lmp_buf_elt->length, 1, LLID_CNTL);
                em_bt_txlmbufptr_setf(txdesc_idx, lmp_buf_elt->buf_ptr);

                // Release descriptor
                em_bt_txptr_txdone_setf(txdesc_idx, 0);

                // Update TX pointer and counter
                acl_par->txdesc_index_sw = !acl_par->txdesc_index_sw;
                DBG_SWDIAG(TX, SW_IDX, acl_par->txdesc_index_sw);
                acl_par->txdesc_cnt++;
                DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);

                continue;
            }

            // Check pending ACL data
            new_txllid = LLID_CONTINUE;

            // Test mode
            if ((acl_par->test_mode != NULL) && (acl_par->test_mode->action == TEST_MODE_NO_ACTION))
            {
                // If not in loopback mode, set available descriptors for sending the test data
                if ((acl_par->test_mode->params.test_scenario != ACLLOOP_MODE)
                        && (acl_par->test_mode->params.test_scenario != SCOLOOP_MODE)
                        && (acl_par->test_mode->params.test_scenario != ACLNOWHIT_MODE)
                        && (acl_par->test_mode->params.test_scenario != SCONOWHIT_MODE))
                {
                    uint16_t txlength = acl_par->test_mode->params.data_length;
                    uint8_t txtype = (acl_par->test_mode->params.packet_type & LMP_TEST_CTRL_PKT_TYPE_CODE_MSK) >> LMP_TEST_CTRL_PKT_TYPE_CODE_POS;

                    // Fill descriptor
                    em_bt_txheader_txtype_setf(txdesc_idx, txtype);
                    em_bt_txpheader_pack(txdesc_idx, 0, txlength, 1, LLID_CONTINUE);
                    em_bt_txaclbufptr_setf(txdesc_idx, acl_par->test_mode->em_buf);

                    // Release descriptor
                    em_bt_txptr_txdone_setf(txdesc_idx, 0);

                    // Update TX pointer and counter
                    acl_par->txdesc_index_sw = !acl_par->txdesc_index_sw;
                    DBG_SWDIAG(TX, SW_IDX, acl_par->txdesc_index_sw);
                    acl_par->txdesc_cnt++;
                    DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);

                    continue;
                }

                break;
            }

            // Check if ACL flow is ON
            if (acl_par->tx_flow)
            {
                // If there is no data to send from the current ACL buffer
                if (acl_par->data_len == 0)
                {
                    struct bt_em_acl_buf_elt *acl_buf_elt;

                    do
                    {
                        // Get new buffer
                        acl_buf_elt = (struct bt_em_acl_buf_elt *) co_list_pop_front(&acl_par->queue_acl_tx);

                        if (acl_buf_elt == NULL)
                        {
                            break;
                        }
                        else
                        {
                            // Check if the data is still valid
                            if ((acl_par->flush_to == AUTO_FLUSH_TIMEOUT_OFF) || (acl_buf_elt->l2cap_start_ts == LD_CLOCK_UNDEF)
                                    || CLK_DIFF(acl_buf_elt->l2cap_start_ts, clock) <= (2 * acl_par->flush_to))
                            {
                                break;
                            }
                            else
                            {
                                // Flush the buffer
                                struct lc_acl_tx_cfm *msg = KE_MSG_ALLOC(LC_ACL_TX_CFM, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_acl_tx_cfm);
                                msg->flushed = true;
                                ke_msg_send(msg);

                                // Free the buffer
                                bt_util_buf_acl_tx_free(acl_buf_elt->buf_ptr);
                            }
                        }
                    }
                    while (1);

                    // Check if a new buffer has been found
                    if (acl_buf_elt != NULL)
                    {
                        uint8_t pbf = GETF(acl_buf_elt->data_len_flags, BT_EM_ACL_PBF);
                        uint16_t data_len = GETF(acl_buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);

                        // Check if there is a current buffer under transmission
                        if (acl_par->curr_buf_elt == NULL)
                        {
                            // Save the buffer as the current one
                            acl_par->curr_buf_elt = acl_buf_elt;
                        }
                        else
                        {
                            ASSERT_ERR(acl_par->next_buf_elt == NULL);

                            // Save the buffer as the next one
                            acl_par->next_buf_elt = acl_buf_elt;
                        }

                        // Set data length and pointer
                        acl_par->data_len = data_len;
                        acl_par->data_ptr = acl_buf_elt->buf_ptr;

                        // Check packet boundary flag
                        if (pbf != PBF_CONT_HL_FRAG)
                        {
                            new_txllid = LLID_START;
                        }
                    }
                }

                // Check new ACL data to send
                if (acl_par->data_len != 0)
                {
                    uint16_t new_txlength;
                    uint8_t new_txtype;

                    // Find packet type and length
                    new_txtype = ld_acl_tx_packet_type_select(link_id, acl_par->data_len, &new_txlength);

                    // Fill descriptor
                    em_bt_txheader_txtype_setf(txdesc_idx, new_txtype);
                    em_bt_txheader_txdv_setf(txdesc_idx, 1);
                    em_bt_txpheader_pack(txdesc_idx, 0, new_txlength, 1, new_txllid);
                    em_bt_txaclbufptr_setf(txdesc_idx, acl_par->data_ptr);

                    // Release descriptor
                    em_bt_txptr_txdone_setf(txdesc_idx, 0);

                    // Update remaining data length
                    acl_par->data_len -= new_txlength;
                    // Update data pointer
                    acl_par->data_ptr += new_txlength;

                    // Update TX pointer and counter
                    acl_par->txdesc_index_sw = !acl_par->txdesc_index_sw;
                    DBG_SWDIAG(TX, SW_IDX, acl_par->txdesc_index_sw);
                    acl_par->txdesc_cnt++;
                    DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);

                    continue;
                }
            }

            // Nothing to send, exit loop
            break;
        }
    }

    // Indicate TX traffic
    if ((acl_par->txdesc_cnt > 0) && (acl_par->tx_traffic == 0))
    {
        acl_par->tx_traffic = 1;
        DBG_SWDIAG(ACL_TX, TRAFFIC, acl_par->tx_traffic);
    }
}

/**
 ****************************************************************************************
 * @brief Check the transmission during activity
 ****************************************************************************************
 */
__STATIC void ld_acl_tx(uint8_t link_id, uint32_t clock)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, acl_par->txdesc_index_hw);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_TX_3_FUNC_BIT, link_id, clock);
    // Initialize TX traffic by default
    if ((acl_par->txdesc_cnt > 0) && (acl_par->tx_traffic < LD_ACL_TRAFFIC_MAX))
    {
        acl_par->tx_traffic++;
        DBG_SWDIAG(ACL_TX, TRAFFIC, acl_par->tx_traffic);
    }

    // Check if packets are acknowledged
    while ((acl_par->txdesc_cnt > 0) && em_bt_txptr_txdone_getf(txdesc_idx))
    {
        // Check if packet is LMP or ACL
        if (em_bt_txlmbufptr_getf(txdesc_idx)) // LMP
        {
            DBG_SWDIAG(DATA, LMP_ACK, 1);

            // Report LMP TX confirmation
            struct lc_lmp_tx_cfm *msg = KE_MSG_ALLOC(LC_LMP_TX_CFM, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_lmp_tx_cfm);
            msg->em_buf = em_bt_txlmbufptr_getf(txdesc_idx);
            ke_msg_send(msg);

            // Clear LMP buffer pointer
            em_bt_txlmbufptr_setf(txdesc_idx, 0);

            DBG_SWDIAG(DATA, LMP_ACK, 0);
        }
        else if (em_bt_txaclbufptr_getf(txdesc_idx) && (acl_par->test_mode == NULL)) // ACL
        {
            DBG_SWDIAG(DATA, ACL_ACK, 1);

            uint16_t txlength = em_bt_txpheader_txlength_getf(txdesc_idx);
            if (txlength > 0)
            {
                uint16_t txaclbufptr = em_bt_txaclbufptr_getf(txdesc_idx);
                uint16_t data_len = GETF(acl_par->curr_buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);

                // Check if current ACL buffer has been completely sent
                if ((txaclbufptr + txlength) >= (acl_par->curr_buf_elt->buf_ptr + data_len))
                {
                    // Report ACL TX confirmation
                    struct lc_acl_tx_cfm *msg = KE_MSG_ALLOC(LC_ACL_TX_CFM, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_acl_tx_cfm);
                    msg->flushed = false;
                    ke_msg_send(msg);

                    // Free the buffer
                    bt_util_buf_acl_tx_free(acl_par->curr_buf_elt->buf_ptr);

                    // Shift next pointer to current pointer
                    acl_par->curr_buf_elt = acl_par->next_buf_elt;
                    acl_par->next_buf_elt = NULL;
                }
            }

            ASSERT_ERR(acl_par->next_buf_elt == NULL);

            // Clear ACL buffer pointer
            em_bt_txaclbufptr_setf(txdesc_idx, 0);

            DBG_SWDIAG(DATA, ACL_ACK, 0);
        }
        else if (acl_par->test_mode == NULL)
        {
            ASSERT_ERR_FORCE(0);
        }

        // Clear descriptor fields
        em_bt_txheader_txtype_setf(txdesc_idx, 0);
        em_bt_txheader_txdv_setf(txdesc_idx, 0);
        em_bt_txpheader_txlength_setf(txdesc_idx, 0);

        // Update TX pointer and counter
        acl_par->txdesc_cnt--;
        DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);
        acl_par->txdesc_index_hw = !acl_par->txdesc_index_hw;
        DBG_SWDIAG(TX, HW_IDX, acl_par->txdesc_index_hw);

#if (EAVESDROPPING_SUPPORT)
        if (acl_par->control_tx_prog && (acl_par->txdesc_cnt == 0) && co_list_is_empty(&acl_par->queue_acl_tx) && co_list_is_empty(&acl_par->queue_lmp_tx))
        {
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);

            // Notify ED task that no transmission is pending
            struct ed_acl_tx_pending_ind *msg = KE_MSG_ALLOC(ED_ACL_TX_PENDING_IND, TASK_ED, TASK_NONE, ed_acl_tx_pending_ind);

            msg->link_id = link_id;
            msg->tx_pending = false;
            msg->tx_seqn = em_bt_acltxstat_lasttxseqn_getf(cs_idx);

            ke_msg_send(msg);

            // Disable Tx programming
            acl_par->tx_prog_en = false;
        }
#endif // EAVESDROPPING_SUPPORT

        // Clear TX traffic to 0 if no packet, or 1 if still a packet to send
        acl_par->tx_traffic = (acl_par->txdesc_cnt > 0);
        DBG_SWDIAG(ACL_TX, TRAFFIC, acl_par->tx_traffic);

        ASSERT_ERR(acl_par->next_buf_elt == NULL);

        // Check opposite descriptor
        txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, acl_par->txdesc_index_hw);
    }

    if (acl_par->flush_to != AUTO_FLUSH_TIMEOUT_OFF)
    {
        // Flush automatically flushable packets if the flush timeout has expired
        ld_acl_automatic_data_flush(link_id, clock);
    }

    // Program new transmissions
    ld_acl_tx_prog(link_id, clock);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_acl_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(ACL, EVT_START, 1);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_EVT_START_CBK_BIT, evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = (struct ld_acl_env_tag *) evt;
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);

        uint8_t frm_type = SCH_BT_FRAME_TYPE_NORMAL;
        uint8_t vxchan = 0;

#if MAX_NB_SYNC
        bool reserved = false;
        vxchan = ld_acl_sco_check(acl_par->link_id, evt->time.hs, &reserved);
        if (vxchan < MAX_NB_SYNC)
            frm_type = reserved ? SCH_BT_FRAME_TYPE_ESCO : SCH_BT_FRAME_TYPE_ESCO_RETX;
#endif //MAX_NB_SYNC

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk         = &ld_acl_frm_cbk;
            prog_par.time.hs         = evt->time.hs;
            prog_par.time.hus        = evt->time.hus;
            prog_par.cs_idx          = cs_idx;
            prog_par.dummy           = cs_idx;
            prog_par.bandwidth       = 2 * LD_ACL_EVT_DUR_POLL_NULL;
            prog_par.prio_1          = evt->current_prio;
            prog_par.prio_2          = RWIP_PRIO_ADD_2(evt->current_prio, LD_ACL_DATA_XCHG_PRIO_INC);
            prog_par.prio_3          = 0;
            prog_par.pti_prio        = acl_par->pti;
            prog_par.add.bt.frm_type = frm_type;
            prog_par.add.bt.vxchan   = vxchan;
            prog_par.mode            = SCH_PROG_BT;
            sch_prog_push(&prog_par);
        }

        acl_par->nb_prog++;

        // For slave role, update timings for the next frame
        if (acl_par->role == SLAVE_ROLE)
        {
            uint32_t rx_win_size_us = (acl_par->rx_win_size + 1) >> 1;

            em_bt_clkoff0_setf(cs_idx, acl_par->clk_off & EM_BT_CLKOFF0_MASK);
            em_bt_clkoff1_setf(cs_idx, (acl_par->clk_off >> 16));

            // Set the RX window size. Check if the wide-open mode should be used
            if (rx_win_size_us > EM_BT_CS_RXWINSZ_MAX)
            {
                if (CO_MOD(rx_win_size_us, SLOT_SIZE))
                {
                    em_bt_wincntl_pack(cs_idx, 1, rx_win_size_us / SLOT_SIZE + 1);
                }
                else
                {
                    em_bt_wincntl_pack(cs_idx, 1, rx_win_size_us / SLOT_SIZE);
                }
            }
            else
            {
                em_bt_wincntl_pack(cs_idx, 0, (rx_win_size_us + 1) >> 1);
            }
        }

        // SAM Support
        if (NULL != acl_par->sam_info)
        {
            ld_acl_sam_offset_set(cs_idx, acl_par, evt->time.hs);
        }

        /*
         * Continuous programming used when master has traffic or slave has a small sync window
         */
        if (((acl_par->role == MASTER_ROLE) && (acl_par->rx_traffic || acl_par->tx_traffic || acl_par->sco_hv1_idx))
                || ((acl_par->role == SLAVE_ROLE) && (acl_par->rx_win_size < 2 * LD_ACL_RX_WIN_SIZE_PROG_LIM)))
        {
            // Enable programming
            sch_prog_enable(&ld_acl_clk_isr, acl_par->link_id, 2, ((acl_par->phase + 4 - rwip_prog_delay) & 0x3));
            acl_par->prog_en = true;
            DBG_SWDIAG(ACL, PROG_EN, 1);
        }

        // Program control structure
        em_bt_loopcntl_att_nb_setf(cs_idx, 1);

        // Force a POLL packet if needed (rx_traffic is used as an indicator)
        // Do not Force POLL if SAM update pending, workaround HW-SW race on CS-FRCNTL update
        if ((acl_par->role == MASTER_ROLE) && acl_par->rx_traffic && !acl_par->tx_traffic && !ld_acl_sam_update_pending(acl_par))
        {
            em_bt_frcntl_fpoll_setf(cs_idx, 1);
        }

        // Move state
        acl_par->state = ACL_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(ACL, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_acl_evt_stop_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(ACL, EVT_STOP, 1);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_EVT_STOP_CBK_BIT, evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = (struct ld_acl_env_tag *) evt;

        // Check if programming is enabled
        if (acl_par->prog_en)
        {
            // Disable programming
            sch_prog_disable(acl_par->link_id);
            acl_par->prog_en = false;
            DBG_SWDIAG(ACL, PROG_EN, 0);
        }

        // If no frame is currently programmed
        if (acl_par->nb_prog == 0)
        {
            if (acl_par->state == ACL_EVT_END)
            {
                // Report ACL end
                ld_acl_end(acl_par->link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST);
            }
            else
            {
                // Reschedule according to the current mode
                ld_acl_resched(acl_par->link_id, evt->time.hs);
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(ACL, EVT_STOP, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_acl_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(ACL, EVT_CANCELED, 1);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_EVT_CANCELED_CBK_BIT, evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = (struct ld_acl_env_tag *) evt;

        switch (acl_par->mode)
        {
        case ACL_MODE_NORMAL:
        case ACL_MODE_ROLE_SWITCH:
        {
            // Increment event priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));
            evt->time.hs += 2 * ((2 + ((co_rand_byte() + ip_finetimtgt_get() + (evt->time.hs >> 1)) & 0x07)) & 0x0E); // Delay with a random number between 2 and 8 slots

            // Reload the automatic rescheduling attempts
            SCH_ARB_ASAP_STG_RESCHED_ATT_SET(evt, LD_ACL_EVT_AUTO_RESCHED_ATT);

            // Try scheduling ACL before LSTO or Role Switch
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                acl_par->state = ACL_EVT_WAIT;
            }
            else
            {
                // Check if a role switch has to be prepared
                if (acl_par->mode == ACL_MODE_ROLE_SWITCH)
                {
                    // Prepare Role Switch
                    ld_acl_rsw_start(acl_par->link_id, CLK_ADD_2(evt->asap_limit, 2 * LD_ACL_RSW_PREPARE_DELAY));
                }
                else
                {
                    // Report ACL end due to Connection Timeout
                    ld_acl_end(acl_par->link_id, CO_ERROR_CON_TIMEOUT);
                }
            }
        }
        break;

        case ACL_MODE_SNIFF_TRANS:
        {
            uint32_t clock = ld_read_clock();

            // Increment event priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));

            // Schedule with sniff transition mode rules
            ld_acl_sniff_trans_sched(acl_par->link_id, clock);
        }
        break;

        case ACL_MODE_SNIFF_ENTER:
        {
            // Enter sniff mode
            ld_acl_sniff_enter(acl_par->link_id, evt->time.hs);

            // Schedule the 1st sniff event
            ld_acl_sniff_sched(acl_par->link_id);
        }
        break;

        default:
        {
            ASSERT_INFO_FORCE(0, acl_par->link_id, acl_par->mode);
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(ACL, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_acl_frm_isr(uint8_t link_id, uint32_t timestamp)
{
    DBG_SWDIAG(ACL, FRM_ISR, 1);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_FRM_ISR_BIT, link_id, timestamp);

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Decrement number of programmed frames
        ASSERT_ERR(acl_par->nb_prog > 0);
        acl_par->nb_prog--;

        // Check ACL end
        if ((acl_par->state == ACL_EVT_END) && (acl_par->nb_prog == 0))
        {
            // Remove event
            sch_arb_remove(evt, true);

            if (acl_par->mode == ACL_MODE_ROLE_SWITCH)
            {
                // Exit Role Switch
                ld_acl_rsw_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST, LD_CLOCK_UNDEF);
            }

            // Report ACL end
            ld_acl_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST);

            // Flush a potentially consumed descriptor
            if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(link_id)))
            {
                // Free RX descriptor
                ld_rxdesc_free();
            }
        }
        else
        {
            uint32_t clock = ld_read_clock();

            // Check reception
            ld_acl_rx(link_id, clock, 1);
            // Check transmission
            ld_acl_tx(link_id, clock);

#if (EAVESDROPPING_SUPPORT)
            // Apply new AES-CCM counter if needed
            if (acl_par->aes_ccm_cnt_upd)
            {
                ld_acl_apply_aes_ccm_cnt(link_id, &acl_par->aes_ccm_cnt[0]);
                acl_par->aes_ccm_cnt_upd = false;
                acl_par->mic_invalid_event_block = false;

                if (acl_par->aes_ccm_cnt_correction_upd)
                {
                    acl_par->aes_ccm_cnt_correction_upd = false;
                    acl_par->aes_ccm_cnt_correction = true;
                }
            }
#endif // (EAVESDROPPING_SUPPORT)
            // Handle slave synchronization
            if (acl_par->role == SLAVE_ROLE)
            {
                // Check if phase has changed from last sync calculation
                if (!acl_par->phase_chg)
                {
                    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

                    uint8_t  frrxok;
                    uint16_t rxbit;

                    // Read together cs_frrxok status and bit position where sync has been found (in half-us)
                    em_bt_rxbit_unpack(cs_idx, &frrxok, &rxbit);
                    // Convert to SW representation
                    rxbit = HALF_SLOT_TIME_MAX - rxbit;

                    // Check if a sync has been found in the frame
                    if (frrxok)
                    {
                        // Refresh slave synchronization as per new reception
                        acl_par->phase_chg = ld_acl_rx_sync(link_id, timestamp, rxbit);
                    }
                    else
                    {
                        // Widen the reception window if needed
                        acl_par->phase_chg = ld_acl_rx_no_sync(link_id, clock);
                    }
                }
                else
                {
                    // Clear phase change flag to restart the sync algorithm
                    acl_par->phase_chg = false;
                }

#if (EAVESDROPPING_SUPPORT)
                bool current_piconet_msb = (CLK_ADD_2(clock, acl_par->last_sync_clk_off) & BT_CLOCK_MSB) != 0;
                if (acl_par->piconet_msb && !current_piconet_msb)
                {
                    // Inform LM of clock wrap (piconet clock when slave)
                    struct lm_clock_wrap_ind *msg;
                    msg = KE_MSG_ALLOC(LM_CLOCK_WRAP_IND, TASK_LM, TASK_NONE, lm_clock_wrap_ind);
                    memcpy(&msg->addr.addr[0], lm_bd_addr_get(link_id), BD_ADDR_LEN);
                    ke_msg_send(msg);
                }
                acl_par->piconet_msb = current_piconet_msb;
#endif // EAVESDROPPING_SUPPORT
            }

            // Check if programming is enabled
            if (acl_par->prog_en)
            {
                // Find any reason to stop programming
                if (((acl_par->lsto != LSTO_OFF) && (CLK_DIFF(acl_par->last_hec_ok_ts, clock) >= (2 * (acl_par->lsto))))
                        || ((acl_par->mode == ACL_MODE_ROLE_SWITCH) && (CLK_DIFF(CLK_ADD_2(clock, acl_par->clk_off), acl_par->rsw_par->instant) <= 2 * LD_ACL_RSW_PREPARE_DELAY))
                        || (!(acl_par->rx_traffic || acl_par->tx_traffic || acl_par->sco_hv1_idx) && (acl_par->role == MASTER_ROLE))
                        || (acl_par->mode == ACL_MODE_SNIFF_ENTER)
                        || ((acl_par->role == SLAVE_ROLE) && (acl_par->rx_win_size > 2 * LD_ACL_RX_WIN_SIZE_PROG_LIM))
                        || ((acl_par->test_mode != NULL) && (acl_par->test_mode->action != TEST_MODE_NO_ACTION))
                        || (acl_par->phase_chg))
                {
                    // Disable programming
                    sch_prog_disable(acl_par->link_id);
                    acl_par->prog_en = false;
                    DBG_SWDIAG(ACL, PROG_EN, 0);
                }
            }

            // Reset event priority
            evt->current_prio = RWIP_PRIO_ADD_2(rwip_priority[RWIP_PRIO_ACL_DFT_IDX].value, ((acl_par->tx_traffic >> LD_ACL_TRAFFIC_PRIO_INCR_FACTOR) + (acl_par->rx_traffic >> LD_ACL_TRAFFIC_PRIO_INCR_FACTOR)) * LD_ACL_DATA_XCHG_PRIO_INC);

            // If no more frame will happen
            if ((!acl_par->prog_en) && (acl_par->nb_prog == 0))
            {
                // Remove event
                sch_arb_remove(evt, true);

                // Check if a test mode update if requested
                if (acl_par->test_mode != NULL)
                {
                    // Update test mode (if needed)
                    ld_acl_test_mode_update(link_id);
                }

                // Reschedule
                evt->time.hs = clock;

                // Reschedule according to the current mode
                ld_acl_resched(link_id, clock);
            }
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(ACL, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle SkipET interrupt
 ****************************************************************************************
 */
__STATIC void ld_acl_sket_isr(uint8_t link_id, uint32_t timestamp)
{
    DBG_SWDIAG(ACL, SKET_ISR, 1);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SKET_ISR_3_FUNC_BIT, link_id, timestamp);

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Decrement number of programmed frames
        ASSERT_ERR(acl_par->nb_prog > 0);
        acl_par->nb_prog--;

        if (acl_par->nb_prog == 0)
        {
            // Check ACL end
            if (acl_par->state == ACL_EVT_END)
            {
                // Remove event
                sch_arb_remove(evt, true);

                if (acl_par->mode == ACL_MODE_ROLE_SWITCH)
                {
                    // Exit Role Switch
                    ld_acl_rsw_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST, LD_CLOCK_UNDEF);
                }

                // Report ACL end
                ld_acl_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST);
            }
            else if (!acl_par->prog_en)
            {
                uint32_t clock = ld_read_clock();

                // Remove event
                sch_arb_remove(evt, true);

                // Increment the priority and reinsert if skipped event corresponded to a Tpoll
                if ((acl_par->mode == ACL_MODE_NORMAL) && (acl_par->role == MASTER_ROLE) && (timestamp == evt->time.hs)
                        && !(acl_par->rx_traffic || acl_par->tx_traffic || acl_par->sco_hv1_idx))
                {
                    // Increment event priority
                    evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));

                    // Reload the automatic rescheduling attempts
                    SCH_ARB_ASAP_STG_RESCHED_ATT_SET(evt, LD_ACL_EVT_AUTO_RESCHED_ATT);

                    // Try scheduling ACL before LSTO
                    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    {
                        acl_par->state = ACL_EVT_WAIT;
                    }
                    else
                    {
                        // Report ACL end due to Connection Timeout
                        ld_acl_end(acl_par->link_id, CO_ERROR_CON_TIMEOUT);
                    }
                }
                else
                {
                    // Reschedule according to the current mode
                    ld_acl_resched(link_id, clock);
                }
            }
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(ACL, SKET_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle RX interrupt
 ****************************************************************************************
 */
__STATIC void ld_acl_rx_isr(uint8_t link_id, uint32_t timestamp)
{
    DBG_SWDIAG(SNIFF, RX_ISR, 1);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RX_ISR_2_FUNC_BIT, link_id, timestamp);

    if (ld_acl_env[link_id] != NULL)
    {
        uint32_t clock = ld_read_clock();

        // Check reception
        ld_acl_rx(link_id, clock, 1);

        // Check transmission
        ld_acl_tx(link_id, clock);
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(SNIFF, RX_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_acl_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    uint8_t cs_idx = dummy;
    ASSERT_INFO(EM_BT_CS_IDX_TO_LID(cs_idx) < MAX_NB_ACTIVE_ACL, cs_idx, irq_type);

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_FRM_CBK_BIT, timestamp, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_acl_frm_isr(EM_BT_CS_IDX_TO_LID(cs_idx), timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        ld_acl_sket_isr(EM_BT_CS_IDX_TO_LID(cs_idx), timestamp);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        ld_acl_rx_isr(EM_BT_CS_IDX_TO_LID(cs_idx), timestamp); // The RX IRQ is not used in normal ACL operations (sniff mode only)
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, cs_idx, irq_type);
    }
    break;
    }
}

/**
 ****************************************************************************************
 * @brief Handle clock interrupt
 ****************************************************************************************
 */
__STATIC void ld_acl_clk_isr(uint32_t clock, uint8_t id)
{
    DBG_SWDIAG(ACL, CLK_ISR, 1);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_CLK_ISR_BIT, clock, id);

    if (ld_acl_env[id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[id];
        struct sch_arb_elt_tag *evt = &acl_par->evt;
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(id);
        uint32_t clock_prg = CLK_ADD_2(clock, ((acl_par->phase - clock) & 0x3));

        // The first frame is programmed from event start callback, not from clock ISR
        if (CLK_DIFF(evt->time.hs, clock_prg) > 2)
        {
            uint8_t frm_type = SCH_BT_FRAME_TYPE_NORMAL;
            uint8_t vxchan = 0;

#if MAX_NB_SYNC
            bool reserved = false;
            vxchan = ld_acl_sco_check(id, clock_prg, &reserved);
            if (vxchan < MAX_NB_SYNC)
                frm_type = reserved ? SCH_BT_FRAME_TYPE_ESCO : SCH_BT_FRAME_TYPE_ESCO_RETX;
#endif //MAX_NB_SYNC

            {
                // Push the programming to SCH PROG
                struct sch_prog_params prog_par;
                prog_par.frm_cbk         = &ld_acl_frm_cbk;
                prog_par.time.hs         = clock_prg;
                prog_par.time.hus        = evt->time.hus;
                prog_par.cs_idx          = cs_idx;
                prog_par.dummy           = cs_idx;
                prog_par.bandwidth       = 2 * LD_ACL_EVT_DUR_POLL_NULL;
                prog_par.prio_1          = evt->current_prio;
                prog_par.prio_2          = RWIP_PRIO_ADD_2(evt->current_prio, LD_ACL_DATA_XCHG_PRIO_INC);
                prog_par.prio_3          = 0;
                prog_par.pti_prio        = BT_PTI_CONNECT_IDX;
                prog_par.add.bt.frm_type = frm_type;
                prog_par.add.bt.vxchan   = vxchan;
                prog_par.mode            = SCH_PROG_BT;
                sch_prog_push(&prog_par);
            }

            acl_par->nb_prog++;

            // Force a POLL packet if needed (rx_traffic is used as an indicator)
            // Do not Force POLL if SAM update pending, workaround HW-SW race on CS-FRCNTL update
            if ((acl_par->role == MASTER_ROLE) && acl_par->rx_traffic && !acl_par->tx_traffic && !ld_acl_sam_update_pending(acl_par))
            {
                em_bt_frcntl_fpoll_setf(cs_idx, 1);
            }

            // For slave role, update timings for the next frame
            if (acl_par->role == SLAVE_ROLE)
            {
                em_bt_clkoff0_setf(cs_idx, acl_par->clk_off & EM_BT_CLKOFF0_MASK);
                em_bt_clkoff1_setf(cs_idx, (acl_par->clk_off >> 16));

                /*
                 * The continuous programming via CLK IRQ is done only when the RX window is small, no need to
                 * check for the wide open mode
                 */
                em_bt_wincntl_pack(cs_idx, 0, (acl_par->rx_win_size >> 2));
            }

            // SAM Support
            if (NULL != acl_par->sam_info)
            {
                ld_acl_sam_config(cs_idx, acl_par, clock_prg);
                ld_acl_sam_offset_set(cs_idx, acl_par, clock_prg);
            }
        }
    }

    DBG_SWDIAG(ACL, CLK_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Apply new AFH map
 ****************************************************************************************
 */
__STATIC void ld_acl_afh_apply(uint8_t link_id, const struct bt_ch_map *ch_map)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_AFH_APPLY_BIT, link_id, ch_map);

    /*
     * The update is done on-the-fly. HW may be fetching the channel map at the same time SW writes it. The acceptable
     * consequence is to do 1 frame on wrong channel. Nevertheless the HW needs at any time to fetch a channel map with
     * at least 20 valid channels.
     * To prevent the map being fetched with not enough valid channels, the SW pre-writes the channel map with all
     * channels enabled (32 channels enabled). Then writes the new map.
     */
    em_bt_chmap0_setf(cs_idx, EM_BT_CHMAP0_MASK);
    em_bt_chmap1_setf(cs_idx, EM_BT_CHMAP1_MASK);
    em_bt_chmap2_setf(cs_idx, EM_BT_CHMAP2_MASK);
    em_bt_chmap3_setf(cs_idx, EM_BT_CHMAP3_MASK);
    em_bt_chmap4_setf(cs_idx, EM_BT_CHMAP4_MASK);

    // Write channel map to CS
    em_bt_chmap0_setf(cs_idx, co_read16p(&ch_map->map[0]));
    em_bt_chmap1_setf(cs_idx, co_read16p(&ch_map->map[2]));
    em_bt_chmap2_setf(cs_idx, co_read16p(&ch_map->map[4]));
    em_bt_chmap3_setf(cs_idx, co_read16p(&ch_map->map[6]));
    em_bt_chmap4_setf(cs_idx, co_read16p(&ch_map->map[8]) & EM_BT_CHMAP4_MASK);

    // Enable AFH
    em_bt_linkcntl_afhena_setf(cs_idx, 1);
}

/**
 ****************************************************************************************
 * @brief Handle AFH switch ON alarm
 ****************************************************************************************
 */
__STATIC void ld_acl_afh_switch_on_cbk(struct sch_alarm_tag *elt)
{
    struct ld_acl_env_tag *acl_par = CONTAINER_OF(elt, struct ld_acl_env_tag, afh_alarm);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_AFH_SWITCH_ON_CBK_BIT, elt);

    DBG_SWDIAG(AFH, HSSI, 1);

    switch (acl_par->afh_state)
    {
    case AFH_WAIT_INSTANT_WAIT_ACK:
    {
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);
        struct bt_ch_map ch_map;

        /*
         * Build recovery map
         *
         * The recovery map is a combination of the channels enabled in the old or new map.
         */

        // Read old map from CS

        co_write16(&ch_map.map[0], em_bt_chmap0_getf(cs_idx));
        co_write16(&ch_map.map[2], em_bt_chmap1_getf(cs_idx));
        co_write16(&ch_map.map[4], em_bt_chmap2_getf(cs_idx));
        co_write16(&ch_map.map[6], em_bt_chmap3_getf(cs_idx));
        co_write16(&ch_map.map[8], em_bt_chmap4_getf(cs_idx));

        // Add channels from the new map
        for (int i = 0 ; i < BT_CH_MAP_LEN ; i++)
        {
            ch_map.map[i] |= acl_par->afh_map.map[i];
        }

        // Apply recovery map
        ld_acl_afh_apply(acl_par->link_id, &ch_map);

        // Move AFH state
        acl_par->afh_state = AFH_WAIT_ACK;
    }
    break;

    case AFH_WAIT_INSTANT:
    {
        // Apply new map
        ld_acl_afh_apply(acl_par->link_id, &acl_par->afh_map);

        // Clear AFH state
        acl_par->afh_state = AFH_NOT_PENDING;
    }
    break;

    default:
    {
        ASSERT_ERR_FORCE(0);
    }
    break;
    }

    DBG_SWDIAG(AFH, HSSI, 0);
}

/**
 ****************************************************************************************
 * @brief Handle AFH switch OFF alarm
 ****************************************************************************************
 */
__STATIC void ld_acl_afh_switch_off_cbk(struct sch_alarm_tag *elt)
{
    struct ld_acl_env_tag *acl_par = CONTAINER_OF(elt, struct ld_acl_env_tag, afh_alarm);
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_AFH_SWITCH_OFF_CBK_BIT, elt);

    DBG_SWDIAG(AFH, HSSI, 1);

    // Disable AFH
    em_bt_linkcntl_afhena_setf(cs_idx, 0);

    // Clear AFH state
    acl_par->afh_state = AFH_NOT_PENDING;

    DBG_SWDIAG(AFH, HSSI, 0);
}

/**
 ****************************************************************************************
 * @brief End of a Role Switch
 ****************************************************************************************
 */
__STATIC void ld_acl_rsw_end(uint8_t link_id, uint8_t reason, uint32_t timestamp)
{
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);
    uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(link_id, 0);
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_END_2_FUNC_BIT, link_id, reason, timestamp);

    // Set normal mode
    acl_par->mode = ACL_MODE_NORMAL;

    // Indicate traffic in order to schedule ASAP
    if (acl_par->rx_traffic < LD_ACL_TRAFFIC_MAX)
    {
        acl_par->rx_traffic++;
    }

    if (reason != CO_ERROR_CON_TERM_BY_LOCAL_HOST)
    {
        // Report Role Switch end to LC
        struct lc_rsw_end_ind *ind = KE_MSG_ALLOC(LC_RSW_END_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_rsw_end_ind);
        ind->status = reason;
        ke_msg_send(ind);

        // Restore the first TX descriptor content
        em_wr(&acl_par->rsw_par->txdesc_save_buf[0], REG_EM_BT_TXDESC_ADDR_GET(txdesc_idx), REG_EM_BT_TXDESC_SIZE);

        if (reason == CO_ERROR_NO_ERROR)
        {
            uint8_t lt_addr = em_bt_linkcntl_aclltaddr_getf(cs_idx);

            // Switch ACL connection to the new piconet
            acl_par->role = !acl_par->role;

            // Set the new timings values
            if (acl_par->role == SLAVE_ROLE)
            {
                // Read bit position where sync has been found (in half-us)
                uint16_t rxbit = HALF_SLOT_TIME_MAX - em_bt_rxbit_rxbit_getf(cs_idx);

                // Run the sync algorithm as the sync has just been found
                acl_par->last_sync_bit_off = evt->time.hus;
                acl_par->last_sync_clk_off = (em_bt_clkoff1_getf(cs_idx) << 16) | em_bt_clkoff0_getf(cs_idx);
                ld_acl_rx_sync(link_id, timestamp, rxbit);
            }
            else
            {
                acl_par->bit_off     = 0;
                acl_par->clk_off     = 0;
                acl_par->phase      = 0;
                acl_par->rx_win_size = 2 * NORMAL_WIN_SIZE;
                acl_par->last_sync_clk_off = 0;
                acl_par->last_sync_bit_off = 0;
            }

            DBG_SWDIAG(ACL, PARITY, acl_par->phase);
            acl_par->rx_max_slot = MAX_SLOT_DFT;
            acl_par->t_poll      = POLL_INTERVAL_DFT;
            acl_par->lsto        = LSTO_DFT;
            acl_par->afh_state   = AFH_NOT_PENDING;
            acl_par->allowed_br_packet_types  &= ((1 << DM1_IDX) | (1 << DH1_IDX));
            acl_par->allowed_edr_packet_types &= ((1 << DM1_IDX) | (1 << DH1_2_IDX) | (1 << DH1_3_IDX));
            acl_par->last_master_clock = RWIP_INVALID_TARGET_TIME;

            // Update Tx descriptors
            for (int i = 0 ; i < 2 ; i++)
            {
                uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(link_id, i);
                em_bt_txheader_txltaddr_setf(txdesc_idx, lt_addr);
            }

            // Restore CS fields
            em_bt_txdescptr_setf(cs_idx, acl_par->rsw_par->txdescptr);
            em_bt_loopcntl_att_nb_setf(cs_idx, 1);

            // Clear a potential AFH alarm
            sch_alarm_clear(&acl_par->afh_alarm);

            if (NULL != acl_par->sam_info)
            {
                // SAM is disabled after successful role switch
                em_bt_frcntl_lsam_dsb_setf(cs_idx, 1);
                em_bt_frcntl_sam_en_setf(cs_idx, 0);

                // Delete SAM paramaters
                ke_free(acl_par->sam_info);
                acl_par->sam_info = NULL;
            }
        }
        else
        {
            // Restore the CS content
            em_wr(&acl_par->rsw_par->cs_save_buf[0], REG_EM_BT_CS_ADDR_GET(cs_idx), REG_EM_BT_CS_SIZE);

            // Restore event delay used for scheduling
            evt->time.hus = acl_par->bit_off;
        }
    }

    if (acl_par->rsw_par->fhs_buf != 0)
    {
        // Free FHS buffer
        bt_util_buf_lmp_tx_free(acl_par->rsw_par->fhs_buf);
    }

    // Delete Role Switch parameters
    ke_free(acl_par->rsw_par);
    acl_par->rsw_par = NULL;
    DBG_SWDIAG(RSW, STEP, 0);

    // Initialize event parameters (common part)
    evt->cb_cancel        = &ld_acl_evt_canceled_cbk;
    evt->cb_start         = &ld_acl_evt_start_cbk;
    evt->cb_stop          = &ld_acl_evt_stop_cbk;
    evt->stop_latency        = LD_ACL_EVT_STOP_THR_MIN;

    // Set to active ACL priority as the frames right after role switch may carry important information
    evt->current_prio        = rwip_priority[RWIP_PRIO_ACL_ACT_IDX].value;
}

/**
 ****************************************************************************************
 * @brief Prepare a Role Switch
 ****************************************************************************************
 */
__STATIC void ld_acl_rsw_start(uint8_t link_id, uint32_t instant_ts)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint8_t new_phase;

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_START_2_FUNC_BIT, link_id, instant_ts);

    // Initialize event parameters (common part)
    evt->cb_cancel        = &ld_acl_rsw_evt_canceled_cbk;
    evt->cb_start         = &ld_acl_rsw_evt_start_cbk;
    evt->cb_stop          = NULL;
    evt->stop_latency        = 0;
    evt->current_prio        = rwip_priority[RWIP_PRIO_ACL_RSW_IDX].value;
    evt->duration_min        = 2 * LD_ACL_RSW_EVT_DUR_MIN;

    // Compute new bit offset (take into account the Tx path delay for TDD switch)
    int16_t new_bit_off = (acl_par->role == MASTER_ROLE) ? (acl_par->bit_off + 2 * ld_env.tx_path_delay) : (acl_par->rsw_bit_off - 2 * ld_env.tx_path_delay);

    // Adjust bit offset and clock offset if needed
    uint32_t new_clk_off = (acl_par->role == MASTER_ROLE) ? 0 : acl_par->rsw_clk_off;
    if (new_bit_off < 0)
    {
        new_bit_off += HALF_SLOT_SIZE;
        new_clk_off = CLK_ADD_2(new_clk_off, 1);
        instant_ts  = CLK_ADD_2(instant_ts, 1);
    }
    else if (new_bit_off >= HALF_SLOT_SIZE)
    {
        new_bit_off -= HALF_SLOT_SIZE;
        new_clk_off = CLK_SUB(new_clk_off, 1);
        instant_ts = CLK_SUB(instant_ts, 1);
    }

    new_phase = (4 - (new_clk_off & 0x03)) & 0x03;
    DBG_SWDIAG(RSW, TDD_PARITY, new_phase);

    // Set the new connection timeout (32 slots)
    acl_par->rsw_par->new_con_end_ts = CLK_ADD_2(instant_ts, 2 * (NEW_CONNECTION_TO + 1));

    // Try scheduling Role Switch before new connection TO
    evt->time.hs = CO_ALIGN4_LO(instant_ts) + new_phase;
    evt->time.hus     = new_bit_off;
    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, new_phase, 0, 0);
    evt->asap_limit = acl_par->rsw_par->new_con_end_ts;
    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
    {
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);
        uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, 0);

        // Save the first descriptor content (will be used in TDD switch (FHS) and piconet switch (POLL/NULL))
        em_rd(&acl_par->rsw_par->txdesc_save_buf[0], REG_EM_BT_TXDESC_ADDR_GET(txdesc_idx), REG_EM_BT_TXDESC_SIZE);

        // Save the CS content
        DBG_MEM_GRANT_CTRL((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), true);
        em_rd(&acl_par->rsw_par->cs_save_buf[0], REG_EM_BT_CS_ADDR_GET(cs_idx), REG_EM_BT_CS_SIZE);
        DBG_MEM_GRANT_CTRL((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), false);

        // Save TX descriptor pointer
        acl_par->rsw_par->txdescptr = em_bt_txdescptr_get(cs_idx);

        // Prepare CS
        em_bt_acltxstat_rswitch_setf(cs_idx, 1);
        em_bt_frcntl_format_setf(cs_idx, (acl_par->role == MASTER_ROLE) ? EM_BT_CS_FMT_SLV_CONNECT : EM_BT_CS_FMT_MST_CONNECT);
        em_bt_clkoff0_setf(cs_idx, new_clk_off & EM_BT_CLKOFF0_MASK);
        em_bt_clkoff1_setf(cs_idx, (new_clk_off >> 16));
        // SAM disabled for role switch
        em_bt_frcntl_sam_en_setf(cs_idx, 0);
#if RW_BT_MWS_COEX
        em_bt_frcntl_dnabort_setf(cs_idx, RWIP_COEX_GET(MSSWITCH, DNABORT));
#endif //RW_BT_MWS_COEX

        // Clear descriptor fields
        em_bt_txaclbufptr_setf(txdesc_idx, 0);
        em_bt_txptr_pack(txdesc_idx, 0, 0);
        // Clear other descriptor fields (master->slave) or prepare descriptor for FHS TX (slave->master)
        em_bt_txheader_txtype_setf(txdesc_idx, (acl_par->role == SLAVE_ROLE) ? FHS_TYPE : ID_NUL_TYPE);
        em_bt_txpheader_txlength_setf(txdesc_idx, (acl_par->role == SLAVE_ROLE) ? FHS_PACKET_SIZE : 0);
        em_bt_txlmbufptr_setf(txdesc_idx, (acl_par->role == SLAVE_ROLE) ? acl_par->rsw_par->fhs_buf : 0);

        // Point CS to the descriptor
        em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, txdesc_idx));

        // Indicate role switch has started
        acl_par->rsw_par->step = ACL_RSW_STEP_TDD_SWITCH;
    }
    else
    {
        // Report Role Switch end to LC
        struct lc_rsw_end_ind *ind = KE_MSG_ALLOC(LC_RSW_END_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE, lc_rsw_end_ind);
        ind->status = CO_ERROR_ROLE_SWITCH_FAIL;
        ke_msg_send(ind);

        // Set normal mode
        acl_par->mode = ACL_MODE_NORMAL;

        if (acl_par->rsw_par->fhs_buf != 0)
        {
            // Free FHS buffer
            bt_util_buf_lmp_tx_free(acl_par->rsw_par->fhs_buf);
        }

        // Delete Role Switch parameters
        ke_free(acl_par->rsw_par);
        acl_par->rsw_par = NULL;
        DBG_SWDIAG(RSW, STEP, 0);

        // Initialize event parameters (common part)
        evt->cb_cancel        = &ld_acl_evt_canceled_cbk;
        evt->cb_start         = &ld_acl_evt_start_cbk;
        evt->cb_stop          = &ld_acl_evt_stop_cbk;
        evt->stop_latency        = LD_ACL_EVT_STOP_THR_MIN;
        evt->current_prio        = rwip_priority[RWIP_PRIO_ACL_DFT_IDX].value;

        // Restore event delay used for scheduling
        evt->time.hus = acl_par->bit_off;

        // Reschedule normal ACL
        ld_acl_sched(acl_par->link_id);
    }
}

/**
 ****************************************************************************************
 * @brief Check FHS packet reception during TDD switch (master->slave)
 ****************************************************************************************
 */
__STATIC bool ld_acl_rsw_fhs_rx(uint8_t link_id, uint8_t *bch_ptr, struct bd_addr *bd_addr_ptr, uint8_t *lt_addr_ptr, uint32_t *fhs_clk_frm_ptr)
{
    bool fhs_received = false;

    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_FHS_RX_2_FUNC_BIT, bool, 5, link_id, bch_ptr, bd_addr_ptr, lt_addr_ptr, fhs_clk_frm_ptr);

    // Check if FHS has been received
    if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(link_id)))
    {
        // Get current RX descriptor index
        uint8_t index_fhs = ld_env.curr_rxdesc_index;
        // Retrieve RX status and type
        uint16_t rxstat_fhs = em_bt_rxstat_get(index_fhs);

        // Check if FHS reception is correct
        if ((rxstat_fhs & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT | EM_BT_RXCRCERR_BIT)) == 0)
        {
            // Point to FHS payload buffer
            uint16_t fhs_buf_ptr = em_bt_rxlmbufptr_getf(index_fhs);

            ASSERT_INFO(em_bt_rxheader_rxtype_getf(index_fhs) == FHS_TYPE, em_bt_rxheader_rxtype_getf(index_fhs), 0);

            // Extract fields from packet
            ld_util_fhs_unpk(fhs_buf_ptr, bch_ptr, bd_addr_ptr, NULL, lt_addr_ptr, fhs_clk_frm_ptr, NULL);

            // Indicate successful reception
            fhs_received = true;
        }

        // Free RX descriptor
        ld_rxdesc_free();
    }

    return (fhs_received);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_acl_rsw_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(RSW, EVT_START, 1);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_EVT_START_CBK_2_FUNC_BIT, evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = (struct ld_acl_env_tag *) evt;
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);
        uint8_t att_nb = (CLK_SUB(acl_par->rsw_par->new_con_end_ts, evt->time.hs) >> 1) + 1;

        // Prepare CS
        em_bt_loopcntl_att_nb_setf(cs_idx, att_nb / 2);

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk         = &ld_acl_rsw_frm_cbk;
            prog_par.time.hs         = evt->time.hs;
            prog_par.time.hus        = evt->time.hus;
            prog_par.cs_idx          = cs_idx;
            prog_par.dummy           = cs_idx;
            prog_par.bandwidth       = evt->duration_min;
            prog_par.prio_1          = evt->current_prio;
            prog_par.prio_2          = evt->current_prio;
            prog_par.prio_3          = 0;
            prog_par.pti_prio        = BT_PTI_MSSWITCH_IDX;
            prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_NORMAL;
            prog_par.mode            = SCH_PROG_BT;
            sch_prog_push(&prog_par);
        }

        // Move state
        acl_par->state = ACL_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(RSW, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_acl_rsw_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(RSW, EVT_CANCELED, 1);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_EVT_CANCELED_CBK_2_FUNC_BIT, evt);

    if (evt != NULL)
    {
        // Increment event priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_RSW_IDX));

        // Try to re-schedule TDD switch or piconet switch before the new connection TO
        if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
        {
            // Point to parameters
            struct ld_acl_env_tag *acl_par = (struct ld_acl_env_tag *) evt;

            // Exit Role Switch
            ld_acl_rsw_end(acl_par->link_id, CO_ERROR_ROLE_SWITCH_FAIL, LD_CLOCK_UNDEF);

            // Reschedule normal ACL
            ld_acl_sched(acl_par->link_id);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(RSW, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_acl_rsw_frm_isr(uint8_t link_id, uint32_t timestamp)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_FRM_ISR_2_FUNC_BIT, link_id, timestamp);

    DBG_SWDIAG(RSW, FRM_ISR, 1);

    // Remove event
    sch_arb_remove(evt, true);

    if (acl_par->mode == ACL_MODE_ROLE_SWITCH)
    {
        // Check Role Switch end
        if (acl_par->state == ACL_EVT_END)
        {
            // Exit Role Switch
            ld_acl_rsw_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST, LD_CLOCK_UNDEF);

            // Exit ACL connection
            ld_acl_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST);

            // Flush a potentially consumed descriptor
            if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(link_id)))
            {
                // Free RX descriptor
                ld_rxdesc_free();
            }
        }
        else
        {
            uint32_t clock = ld_read_clock();

            // Restore original Role Switch priority
            evt->current_prio = rwip_priority[RWIP_PRIO_ACL_RSW_IDX].value;

            // Update timestamp to current clock
            evt->time.hs = clock;

            // Move to wait state
            acl_par->state = ACL_EVT_WAIT;

            // Check Role Switch step
            switch (acl_par->rsw_par->step)
            {
            case ACL_RSW_STEP_TDD_SWITCH:
            {
                uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);
                bool frame_ok = false;
                struct bd_addr peer_bd_addr;
                uint8_t peer_bch[5];
                uint32_t fhs_clk_frm = 0;

                if (acl_par->role == MASTER_ROLE)
                {
                    // Check if FHS has been received in the TDD switch frame
                    frame_ok = ld_acl_rsw_fhs_rx(link_id, &peer_bch[0], &peer_bd_addr, &acl_par->rsw_par->new_lt_addr, &fhs_clk_frm);
                }
                else
                {
                    // Check if ID has been received in the TDD switch frame
                    frame_ok = (em_bt_rxbit_frrxok_getf(cs_idx) && em_bt_aclrxstat_rxid_getf(cs_idx));
                }

                // Check if ID or FHS has been received in the TDD switch frame
                if (frame_ok)
                {
                    uint8_t new_phase = 0;
                    struct bd_addr *bd_addr = (acl_par->role == MASTER_ROLE) ? &peer_bd_addr : &ld_env.local_bd_addr;
                    uint8_t *bch = (acl_par->role == MASTER_ROLE) ? &peer_bch[0] : &ld_env.local_bch[0];
                    uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, 0);

                    // Set the new connection timeout (32 slots)
                    acl_par->rsw_par->new_con_end_ts = CLK_ADD_2(clock, 2 * (NEW_CONNECTION_TO + 1));

                    // Prepare descriptor for TX / Poll (slave->master) or Null (master->slave)
                    em_bt_txheader_txltaddr_setf(txdesc_idx, acl_par->rsw_par->new_lt_addr);
                    em_bt_txptr_txdone_setf(txdesc_idx, 1);

                    // Set control structure fields for the new piconet
                    em_bt_linkcntl_aclltaddr_setf(cs_idx, acl_par->rsw_par->new_lt_addr);
                    em_bt_bdaddr_setf(cs_idx, 0, (bd_addr->addr[1] << 8) | bd_addr->addr[0]);
                    em_bt_bdaddr_setf(cs_idx, 1, (bd_addr->addr[3] << 8) | bd_addr->addr[2]);
                    em_bt_bdaddr_setf(cs_idx, 2, (bd_addr->addr[5] << 8) | bd_addr->addr[4]);
                    em_bt_bch0_setf(cs_idx, (bch[1] << 8) | bch[0]);
                    em_bt_bch1_setf(cs_idx, (bch[3] << 8) | bch[2]);
                    em_bt_rxmaxbuf_bch2_pack(cs_idx, ACL_DATA_BUF_SIZE, 0, bch[4] & EM_BT_BCH2_MASK);
                    em_bt_acltxstat_fnak_setf(cs_idx, 1);
                    em_bt_acltxstat_fcrc_setf(cs_idx, 1);
                    em_bt_frcntl_fpoll_setf(cs_idx, 1);
                    em_bt_aclrxstat_lastrxseqn_setf(cs_idx, 0);
                    em_bt_linkcntl_afhena_setf(cs_idx, 0);
                    em_bt_txrxcntl_txcrypt_setf(cs_idx, 0);
                    em_bt_txrxcntl_rxcrypt_setf(cs_idx, 0);
                    em_bt_txrxcntl_txbcry_setf(cs_idx, 0);
                    em_bt_txrxcntl_rxbcry_setf(cs_idx, 0);

                    if (acl_par->role == SLAVE_ROLE)
                    {
                        // Release FHS buffer
                        if (acl_par->rsw_par->fhs_buf != 0)
                        {
                            bt_util_buf_lmp_tx_free(acl_par->rsw_par->fhs_buf);
                            acl_par->rsw_par->fhs_buf = 0;
                        }

                        // Clear FHS pointer from descriptor
                        em_bt_txpheader_txlength_setf(txdesc_idx, 0);
                        em_bt_txlmbufptr_setf(txdesc_idx, 0);

                        // Clear clock offset and bit offset
                        em_bt_clkoff0_setf(cs_idx, 0);
                        em_bt_clkoff1_setf(cs_idx, 0);

                        // Update event delay used for scheduling
                        evt->time.hus = 0;
                    }
                    else
                    {
                        uint32_t new_clk_off;
                        int16_t new_bit_off;

                        // Read clock value where sync has been found
                        uint32_t rxclkn = (em_bt_rxclkn1_getf(cs_idx) << 16) | em_bt_rxclkn0_getf(cs_idx);

                        // Compute synchronization window size
                        uint32_t sync_win_size = CLK_SUB(clock, acl_par->rsw_par->slot_offset_ts);
                        sync_win_size = sync_win_size * HALF_SLOT_SIZE * (rwip_current_drift_get() + BT_MAX_DRIFT_ACTIVE) * 2 / 1000000;
                        sync_win_size += 2 * LD_ACL_RSW_SYNC_WIN_SIZE_MIN;
                        sync_win_size = co_min(sync_win_size, 2 * LD_ACL_RSW_SYNC_WIN_SIZE_MAX);
                        em_bt_wincntl_pack(cs_idx, 0, (sync_win_size >> 2));

                        // Compute bit offset and clock offset
                        new_clk_off = CLK_SUB((fhs_clk_frm << 2), rxclkn);
                        new_bit_off = 0;
                        if (acl_par->rsw_bit_off != 0)
                        {
                            new_bit_off = acl_par->rsw_bit_off - (4 * HALF_SLOT_SIZE);
                        }
                        new_bit_off += NORMAL_WIN_SIZE - sync_win_size / 2;

                        // Adjust bit offset and clock offset if needed
                        if (new_bit_off >= HALF_SLOT_SIZE)
                        {
                            uint8_t num_hs = new_bit_off / HALF_SLOT_SIZE;
                            new_bit_off -= num_hs * HALF_SLOT_SIZE ;
                            new_clk_off = CLK_SUB(new_clk_off, num_hs);
                        }
                        else if (new_bit_off < 0)
                        {
                            uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
                            new_bit_off += num_hs * HALF_SLOT_SIZE;
                            new_clk_off = CLK_ADD_2(new_clk_off, num_hs);
                        }

                        // Set clock offset and bit offset
                        em_bt_clkoff0_setf(cs_idx, new_clk_off & EM_BT_CLKOFF0_MASK);
                        em_bt_clkoff1_setf(cs_idx, (new_clk_off >> 16));

                        // Update the clock phase
                        new_phase = (4 - (new_clk_off & 0x03)) & 0x03;

                        // Update event delay used for scheduling
                        evt->time.hus = new_bit_off;
                    }

                    // Move step to 1st Packet
                    acl_par->rsw_par->step = ACL_RSW_STEP_PICONET_SWITCH;
                    DBG_SWDIAG(RSW, STEP, 1);

                    // Change event settings
                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, new_phase, 0, 0);
                    evt->asap_limit = acl_par->rsw_par->new_con_end_ts;

                    // Try to schedule piconet switch before the new connection TO
                    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    {
                        break;
                    }
                }
                else
                {
                    // Try to schedule TDD switch before the new connection TO
                    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    {
                        break;
                    }
                }

                // Exit Role Switch
                ld_acl_rsw_end(link_id, CO_ERROR_ROLE_SWITCH_FAIL, LD_CLOCK_UNDEF);

                // Reschedule normal ACL
                ld_acl_sched(link_id);
            }
            break;
            case ACL_RSW_STEP_PICONET_SWITCH:
            {
                bool packet_received = false;

                // Check if a descriptor is consumed
                if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(link_id)))
                {
                    // Retrieve RX status and type
                    uint16_t rxstat = em_bt_rxstat_get(ld_env.curr_rxdesc_index);

                    // Check if packet reception is correct
                    if ((rxstat & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT)) == 0)
                    {
                        // Indicate successful reception
                        packet_received = true;
                    }

                    // Process reception
                    ld_acl_rx(link_id, clock, 1);
                }

                // Check if 1st packet of the new piconet has been received
                if (packet_received)
                {
                    // Exit Role Switch (completed successfully)
                    ld_acl_rsw_end(link_id, CO_ERROR_NO_ERROR, timestamp);
                }
                else
                {
                    // Try to re-schedule piconet switch before the new connection TO
                    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    {
                        break;
                    }

                    // Exit Role Switch
                    ld_acl_rsw_end(link_id, CO_ERROR_ROLE_SWITCH_FAIL, LD_CLOCK_UNDEF);
                }

                // Reschedule normal ACL
                ld_acl_sched(link_id);
            }
            break;
            default:
            {
                // Nothing to do
                ASSERT_INFO_FORCE(0, acl_par->rsw_par->step, 0);
            }
            break;
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(RSW, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_acl_rsw_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    uint8_t cs_idx = dummy;

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_FRM_CBK_2_FUNC_BIT, timestamp, dummy, irq_type);

    ASSERT_INFO(EM_BT_CS_IDX_TO_LID(cs_idx) < MAX_NB_ACTIVE_ACL, cs_idx, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_acl_rsw_frm_isr(EM_BT_CS_IDX_TO_LID(cs_idx), timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        // Remove event
        sch_arb_remove(&(ld_acl_env[EM_BT_CS_IDX_TO_LID(cs_idx)]->evt), true);

        ld_acl_rsw_evt_canceled_cbk(&(ld_acl_env[EM_BT_CS_IDX_TO_LID(cs_idx)]->evt));
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, cs_idx, irq_type);
    }
    break;
    }
}

/**
 ****************************************************************************************
 * @brief Compute 1st sniff anchor point
 * @param[in]    offset   Sniff offset (in BT slots)
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_first_anchor_compute(uint8_t link_id, uint8_t offset)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_FIRST_ANCHOR_COMPUTE_3_FUNC_BIT, link_id, offset);

    uint32_t clock = ld_read_clock();
    int32_t diff;
    uint32_t master_clock = CLK_ADD_2(clock, acl_par->last_sync_clk_off);

    // Find the first sniff anchor point
    diff = 2 * acl_par->sniff_par.offset - CO_MOD((master_clock ^ (acl_par->sniff_par.init << 27)), 2 * acl_par->sniff_par.intv);
    acl_par->sniff_par.anchor_point = CLK_ADD_2(clock, diff);
}

/**
 ****************************************************************************************
 * @brief Enter sniff mode
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_enter(uint8_t link_id, uint32_t clock)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_ENTER_3_FUNC_BIT, link_id, clock);

    // Initialize event parameters (common part)
    evt->cb_cancel        = &ld_acl_sniff_evt_canceled_cbk;
    evt->cb_start         = &ld_acl_sniff_evt_start_cbk;
    evt->cb_stop          = NULL;
    evt->stop_latency        = 0;
    evt->current_prio        = rwip_priority[RWIP_PRIO_ACL_SNIFF_DFT_IDX].value;
    evt->duration_min        = 2 * LD_ACL_EVT_DUR_MIN_DFT;

    // Unregister ACL from active links
    ld_active_mode_set(LD_ACT_TYPE_ACL, link_id, 0, LD_ACT_MODE_OFF);

    // Register the sniff to scheduling parameters
    sch_slice_per_add(BT_SNIFF, link_id, 2 * acl_par->sniff_par.intv, (acl_par->sniff_par.att * 4 * HALF_SLOT_SIZE), false);

    // Set mode to Sniff
    acl_par->mode = ACL_MODE_SNIFF;

    // Set Rx threshold to 1
    em_bt_txrxcntl_rxthr_setf(cs_idx, 1);

    // SAM Support
    // Sniff overrides the SAM slot maps BB:4.1.15.6
    if (NULL != acl_par->sam_info)
    {
        // Disable peer SAM on this link
        em_bt_frcntl_sam_en_setf(cs_idx, 0);
#if RW_BT_MWS_COEX
        // Disable local SAM on this link
        em_bt_frcntl_lsam_dsb_setf(cs_idx, 1);
#endif //RW_BT_MWS_COEX
    }
}

/**
 ****************************************************************************************
 * @brief Exit sniff mode
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_exit(uint8_t link_id)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_EXIT_3_FUNC_BIT, link_id);

    // Initialize event parameters (common part)
    evt->cb_cancel        = &ld_acl_evt_canceled_cbk;
    evt->cb_start         = &ld_acl_evt_start_cbk;
    evt->cb_stop          = &ld_acl_evt_stop_cbk;
    evt->current_prio        = rwip_priority[RWIP_PRIO_ACL_DFT_IDX].value;
    evt->duration_min       = 2 * LD_ACL_EVT_DUR_MIN_DFT;
    evt->stop_latency       = LD_ACL_EVT_STOP_THR_MIN;

    // Set Rx threshold to 0 (no Rx interrupt)
    em_bt_txrxcntl_rxthr_setf(cs_idx, 0);

    // Clear sniff sub rating parameters
    acl_par->sniff_par.ssr.ssr = 0;

    // Register ACL link as active link
    ld_active_mode_set(LD_ACT_TYPE_ACL, link_id, 0, LD_ACT_MODE_ON);

    // Unregister the sniff from scheduling parameters
    sch_slice_per_remove(BT_SNIFF, link_id);

    // SAM Support
    // Sniff overrides the SAM slot maps BB:4.1.15.6
    if (NULL != acl_par->sam_info)
    {
        if (acl_par->sam_info->map_length)
        {
            // Re-enable peer SAM on this link
            em_bt_frcntl_sam_en_setf(cs_idx, LD_ACL_SAM_SLOT_BLOCKING_EN);
        }
#if RW_BT_MWS_COEX
        if (acl_par->sam_info->loc_idx != SAM_DISABLED)
        {
            // Re-enable local SAM on this link
            em_bt_frcntl_lsam_dsb_setf(cs_idx, RWIP_COEX_GET(CONNECT, SAMEN) ? 0 : 1);
        }
#endif //RW_BT_MWS_COEX
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_EVT_START_CBK_3_FUNC_BIT, evt);

    DBG_SWDIAG(SNIFF, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = (struct ld_acl_env_tag *) evt;
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);
        int32_t diff_sched_prog = 0;

        if (acl_par->role == SLAVE_ROLE)
        {
            uint32_t rx_win_size;
            uint32_t rx_win_size_us;
            uint32_t new_clk_off;
            int16_t new_bit_off;

            // Calculate new values only if last_sync_ts has changed
            if (acl_par->sniff_par.sched_last_sync_ts_used != acl_par->last_sync_ts)
            {
                new_bit_off = acl_par->last_sync_bit_off;
                new_clk_off = acl_par->last_sync_clk_off;

                // Compute maximum drift
                uint32_t clock_diff = CLK_SUB(evt->time.hs, acl_par->last_sync_ts);

                uint8_t local_drift = rwip_current_drift_get();
                uint8_t peer_drift = ld_active_link_check(acl_par->link_id) ? BT_MAX_DRIFT_ACTIVE : acl_par->peer_drift;

                uint32_t max_drift = clock_diff * (local_drift + peer_drift) * HALF_SLOT_SIZE / 1000000;

                // Calculate the new Rx window size
                rx_win_size = 2 * max_drift + 2 * NORMAL_WIN_SIZE;

                // Compute the new bit offset / clock offset
                new_bit_off -= (rx_win_size - 2 * NORMAL_WIN_SIZE) / 2;
                if (new_bit_off < 0)
                {
                    uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
                    new_bit_off += num_hs * HALF_SLOT_SIZE;
                    new_clk_off = CLK_ADD_2(new_clk_off, num_hs);
                }

                /*
                 * Compute the difference between the scheduling and the programming. The values might have changed if a
                 * parallel activity has synced in the meantime.
                 */
                diff_sched_prog = CLK_DIFF(new_clk_off, acl_par->sniff_par.sched_clock_off);

                ASSERT_ERR(diff_sched_prog >= 0);
                ASSERT_ERR(diff_sched_prog < 4);
            }
            else // Use the values already calculated by ld_acl_sniff_sched since last_sync_ts has not changed
            {
                rx_win_size = acl_par->sniff_par.sched_rx_win_size;
                new_clk_off = acl_par->sniff_par.sched_clock_off;
                new_bit_off = acl_par->sniff_par.sched_bit_off;
            }

            // Refresh the RX win size so that it reflects the latest computed value
            acl_par->rx_win_size = rx_win_size;

            // Apply the values calculated for the clock offset, bit offset and Rx window size
            em_bt_clkoff0_setf(cs_idx, new_clk_off & EM_BT_CLKOFF0_MASK);
            em_bt_clkoff1_setf(cs_idx, (new_clk_off >> 16));
            // Check if the HW needs to continue looping after the anchor point
            if (rx_win_size < 2 * HALF_SLOT_SIZE)
            {
                em_bt_linkcntl_laap_setf(cs_idx, 1);
            }
            else
            {
                em_bt_linkcntl_laap_setf(cs_idx, 0);
            }

            rx_win_size_us = (acl_par->rx_win_size + 1) >> 1;
            // Check if the wide-open mode should be used
            if (rx_win_size_us > EM_BT_CS_RXWINSZ_MAX)
            {
                if (CO_MOD(rx_win_size_us, SLOT_SIZE))
                {
                    em_bt_wincntl_pack(cs_idx, 1, rx_win_size_us / SLOT_SIZE + 1);
                }
                else
                {
                    em_bt_wincntl_pack(cs_idx, 1, rx_win_size_us / SLOT_SIZE);
                }
            }
            else
            {
                em_bt_wincntl_pack(cs_idx, 0, (rx_win_size_us + 1) >> 1);
            }
        }

        // Program control structure
        em_bt_loopcntl_to_nb_setf(cs_idx, acl_par->sniff_par.to);
        em_bt_loopcntl_att_nb_setf(cs_idx, acl_par->sniff_par.sched_att); // Program the number of scheduled attempts
        em_bt_maxfrmtime_setf(cs_idx, co_min(acl_par->sniff_par.intv / 2, EM_BT_MAXFRMTIME_MASK));

        // Force a POLL packet if needed
        if (acl_par->role == MASTER_ROLE)
        {
            // FPOLL limit in 312.5 us half-slots
            uint32_t fpoll_lim = LD_ACL_FPOLL_LIMIT_DFT;
            if (acl_par->lsto != LSTO_OFF)
            {
                // FPOLL margin in slots, from N * sniff interval * subrate factor, where N is the configurable number of active sniff
                // anchor points prior to link supervision timeout, where NULL packet is replaced by POLL packet until a response is received.
                uint32_t fpoll_margin = LD_ACL_SNIFF_FPOLL_MARGIN * acl_par->sniff_par.intv * (acl_par->sniff_par.ssr.ssr + 1);

                // Apply the new FPOLL limit if it extends on the default value.
                if (acl_par->lsto > (fpoll_margin + HS_TO_S(LD_ACL_FPOLL_LIMIT_DFT)))
                {
                    fpoll_lim = S_TO_HS(acl_par->lsto - fpoll_margin);
                }
            }

            if (CLK_SUB(evt->time.hs, acl_par->last_sync_ts) > fpoll_lim)
            {
                em_bt_frcntl_fpoll_setf(cs_idx, 1);
            }
            else
            {
                em_bt_frcntl_fpoll_setf(cs_idx, 0);
            }
        }

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk         = &ld_acl_sniff_frm_cbk;
            prog_par.time.hs         = CLK_ADD_2(evt->time.hs, diff_sched_prog);
            prog_par.time.hus        = evt->time.hus;
            prog_par.cs_idx          = cs_idx;
            prog_par.dummy           = cs_idx;
            prog_par.bandwidth       = evt->duration_min;
            prog_par.prio_1          = evt->current_prio;
            prog_par.prio_2          = 0;
            prog_par.prio_3          = 0;
            prog_par.pti_prio        = BT_PTI_SNIFFATT_IDX;
            prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_SNIFF;
            prog_par.add.bt.vxchan   = 0;
            prog_par.mode            = SCH_PROG_BT;
            sch_prog_push(&prog_par);
        }

        // Move state
        acl_par->state = ACL_EVT_ACTIVE;

        // Reset sniff event priority
        evt->current_prio = RWIP_PRIO_ADD_2(rwip_priority[RWIP_PRIO_ACL_SNIFF_DFT_IDX].value, (acl_par->tx_traffic >> LD_ACL_TRAFFIC_PRIO_INCR_FACTOR) * LD_ACL_DATA_XCHG_PRIO_INC);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(SNIFF, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_EVT_CANCELED_CBK_3_FUNC_BIT, evt);

    DBG_SWDIAG(SNIFF, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = (struct ld_acl_env_tag *) evt;

        // Reschedule the next interval
        ld_acl_sniff_sched(acl_par->link_id);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(SNIFF, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_frm_isr(uint8_t link_id, uint32_t timestamp)
{

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_FRM_ISR_3_FUNC_BIT, link_id, timestamp);

    DBG_SWDIAG(SNIFF, FRM_ISR, 1);

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Remove event
        sch_arb_remove(evt, true);

        // Check ACL end
        if (acl_par->state == ACL_EVT_END)
        {
            // Report ACL end
            ld_acl_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        else
        {
            uint32_t clock = ld_read_clock();

            // Handle slave synchronization
            if (acl_par->role == SLAVE_ROLE)
            {
                uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

                uint8_t  frrxok;
                uint16_t rxbit;

                // Read together cs_frrxok status and bit position where sync has been found (in half-us)
                em_bt_rxbit_unpack(cs_idx, &frrxok, &rxbit);
                // Convert to SW representation
                rxbit = HALF_SLOT_TIME_MAX - rxbit;

                // Check if a sync has been found in the frame
                if (frrxok)
                {
                    // Refresh slave synchronization as per new reception
                    ld_acl_rx_sync(link_id, timestamp, rxbit);
                }
            }

            // Reschedule
            evt->time.hs = timestamp;

            switch (acl_par->mode)
            {
            case ACL_MODE_SNIFF_ENTER:
            {
                // Exit sniff mode
                ld_acl_sniff_exit(link_id);

                // Enter sniff mode
                ld_acl_sniff_enter(link_id, clock);
            }
            // no break

            case ACL_MODE_SNIFF:
            {
                // Schedule sniff
                ld_acl_sniff_sched(link_id);
            }
            break;

            case ACL_MODE_SNIFF_TRANS:
            {
                // Exit sniff mode
                ld_acl_sniff_exit(link_id);

                // Try to schedule
                ld_acl_sniff_trans_sched(link_id, clock);
            }
            break;

            case ACL_MODE_NORMAL:
            {
                // Exit sniff mode
                ld_acl_sniff_exit(link_id);

                // Try to schedule
                ld_acl_sched(link_id);
            }
            break;

            default:
            {
                ASSERT_INFO_FORCE(0, link_id, acl_par->mode);
            }
            break;
            }
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(SNIFF, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    uint8_t cs_idx = dummy;

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_FRM_CBK_3_FUNC_BIT, timestamp, dummy, irq_type);

    ASSERT_INFO(EM_BT_CS_IDX_TO_LID(cs_idx) < MAX_NB_ACTIVE_ACL, cs_idx, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_acl_sniff_frm_isr(EM_BT_CS_IDX_TO_LID(cs_idx), timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        // Remove event
        sch_arb_remove(&(ld_acl_env[EM_BT_CS_IDX_TO_LID(cs_idx)]->evt), true);

        ld_acl_sniff_evt_canceled_cbk(&(ld_acl_env[EM_BT_CS_IDX_TO_LID(cs_idx)]->evt));
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        ld_acl_rx_isr(EM_BT_CS_IDX_TO_LID(cs_idx), timestamp);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, cs_idx, irq_type);
    }
    break;
    }
}

/**
 ****************************************************************************************
 * @brief Schedule sniff mode anchor point
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_sched(uint8_t link_id)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint32_t lim;
    bool found = false;
    int16_t new_bit_off = 0;
    uint32_t new_clk_off = 0;
    uint32_t rx_win_size = 2 * NORMAL_WIN_SIZE;
    uint32_t clock = ld_read_clock();

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_SCHED_3_FUNC_BIT, link_id);

    DBG_SWDIAG(SNIFF, SCHED, 1);

    // Check if SSR parameters have been set
    if (acl_par->sniff_par.ssr.ssr > 0)
    {
        uint32_t master_clock = CLK_ADD_2(acl_par->sniff_par.anchor_point, acl_par->last_sync_clk_off);

        // Check if the SSR instant has occurred
        if (!acl_par->sniff_par.ssr.ssr_instant_passed && (CLK_LOWER_EQ(acl_par->sniff_par.ssr.ssr_instant, master_clock)))
        {
            acl_par->sniff_par.ssr.ssr_instant_passed = true;
            ke_msg_send_basic(LC_SSR_INST_IND, KE_BUILD_ID(TASK_LC, link_id), TASK_NONE);
        }

        if (acl_par->sniff_par.ssr.ssr_instant_passed)
        {
            // Check if SSR can be enabled
            if ((acl_par->rx_traffic < LD_ACL_RX_TRAFFIC_DFT) && (!acl_par->tx_traffic))
            {
                if ((!acl_par->sniff_par.ssr.ssr_timer) || (acl_par->sniff_par.ssr.ssr_timer && (CLK_DIFF(acl_par->sniff_par.ssr.ssr_restart_ts, acl_par->sniff_par.anchor_point) >= 2 * acl_par->sniff_par.ssr.ssr_to)))
                {
                    acl_par->sniff_par.ssr.ssr_timer = false;
                    acl_par->sniff_par.ssr.ssr_active = true;
                }
            }
            else // Disable SSR and restart the SSR timer
            {
                acl_par->sniff_par.ssr.ssr_active = false;
                acl_par->sniff_par.ssr.ssr_timer = true;
                acl_par->sniff_par.ssr.ssr_restart_ts = acl_par->sniff_par.anchor_point;
            }
        }
    }

    // Find the next sniff anchor point
    while (CLK_LOWER_EQ(acl_par->sniff_par.anchor_point, clock))
    {
        acl_par->sniff_par.anchor_point = CLK_ADD_2(acl_par->sniff_par.anchor_point, 2 * acl_par->sniff_par.intv);
    }

    // Check if SSR rules need to be applied
    if (acl_par->sniff_par.ssr.ssr_active)
    {
        acl_par->sniff_par.anchor_point = CLK_ADD_2(acl_par->sniff_par.anchor_point, 2 * (acl_par->sniff_par.ssr.ssr - 1) * acl_par->sniff_par.intv);
    }

    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, 0, 0, 0);
    acl_par->sniff_par.sched_att = acl_par->sniff_par.att;

    lim = (acl_par->lsto != LSTO_OFF) ? 2 * acl_par->lsto : RWIP_MAX_CLOCK_TIME;
    // Try to schedule and adjust the Rx window size accordingly if needed
    while (((int32_t) CLK_DIFF(acl_par->last_sync_ts, acl_par->sniff_par.anchor_point) < (int32_t) lim) && (!found))
    {
        // The scheduling timestamp is initialized to the targeted clock value of the sync
        uint32_t sched_ts = acl_par->sniff_par.anchor_point;

        if (acl_par->role == SLAVE_ROLE)
        {
            new_bit_off = acl_par->last_sync_bit_off;
            new_clk_off = acl_par->last_sync_clk_off;

            // Compute maximum drift
            uint32_t clock_diff = CLK_SUB(sched_ts, acl_par->last_sync_ts);

            uint8_t local_drift = rwip_current_drift_get();
            uint8_t peer_drift = ld_active_link_check(link_id) ? BT_MAX_DRIFT_ACTIVE : acl_par->peer_drift;

            uint32_t max_drift = clock_diff * (local_drift + peer_drift) / 1600; // slots * ppm * 625us / 1000000;

            // Calculate the new Rx window size
            rx_win_size = 2 * max_drift + 2 * NORMAL_WIN_SIZE;

            // Compute the new bit offset / clock offset
            new_bit_off -= (rx_win_size - 2 * NORMAL_WIN_SIZE) / 2;
            if (new_bit_off < 0)
            {
                uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
                sched_ts = CLK_SUB(sched_ts, num_hs);
                new_bit_off += num_hs * HALF_SLOT_SIZE;
                new_clk_off = CLK_ADD_2(new_clk_off, num_hs);
            }

            // Set new bit offset as event delay for scheduling
            evt->time.hus = new_bit_off;
        }

        // Load maximum the number of attempts for the next sniff event
        acl_par->sniff_par.sched_att = acl_par->sniff_par.att;

        // Load minimum duration including all attempts
        evt->duration_min = 2 * ((2 * acl_par->sniff_par.att * SLOT_SIZE) - LD_ACL_INTERFRAME_MARGIN);

        // Ensure enough time for receiving a multi-slot packet that fails multiple consecutive receptions
        if ((acl_par->rx_max_slot > 1) && (acl_par->rx_traffic >> LD_ACL_TRAFFIC_DUR_INCR_FACTOR))
        {
            evt->duration_min = co_max(evt->duration_min, 2 * ((acl_par->rx_max_slot + 1) * SLOT_SIZE));
        }

        // Try to schedule at least one of the attempts
        while (acl_par->sniff_par.sched_att > 0)
        {
            evt->time.hs = sched_ts;
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                found = true;
                break;
            }
            else
            {
                // Move timestamp to the next attempt
                acl_par->sniff_par.sched_att--;
                evt->duration_min -= (4 * HALF_SLOT_SIZE);
                sched_ts = CLK_ADD_2(sched_ts, 4);
            }
        }

        if (found)
            break;

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_SNIFF_DFT_IDX));
        // Move timestamp to the next anchor point
        acl_par->sniff_par.anchor_point = CLK_ADD_2(acl_par->sniff_par.anchor_point, 2 * acl_par->sniff_par.intv);
    }

    if (found)
    {
        acl_par->state = ACL_EVT_WAIT;

        if (acl_par->role == SLAVE_ROLE)
        {
            // Store the values calculated for the clock offset, bit offset and Rx window size
            acl_par->sniff_par.sched_last_sync_ts_used = acl_par->last_sync_ts;
            acl_par->sniff_par.sched_bit_off = new_bit_off;
            acl_par->sniff_par.sched_clock_off = new_clk_off;
            acl_par->sniff_par.sched_rx_win_size = rx_win_size;
        }
    }
    else
    {
        // Report ACL end
        ld_acl_end(link_id, CO_ERROR_CON_TIMEOUT);
    }

    DBG_SWDIAG(SNIFF, SCHED, 0);
}

/**
 ****************************************************************************************
 * @brief Schedule next sniff transition event
 ****************************************************************************************
 */
__STATIC void ld_acl_sniff_trans_sched(uint8_t link_id, uint32_t clock)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_TRANS_SCHED_3_FUNC_BIT, link_id, clock);

    acl_par->state = ACL_EVT_WAIT;

    // Find the next sniff anchor point
    while (CLK_DIFF(clock, acl_par->sniff_par.anchor_point) < (LD_ACL_SNIFF_TRANS_ANC_PROTEC_DELAY + 2 * rwip_prog_delay))
    {
        acl_par->sniff_par.anchor_point = CLK_ADD_2(acl_par->sniff_par.anchor_point, 2 * acl_par->sniff_par.intv);
    }

    evt->time.hs = clock;
    while (1)
    {
        SCH_ARB_ASAP_STG_TYPE_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT);
        // Try to schedule normal ACL before next sniff anchor point
        evt->asap_limit = CLK_SUB(acl_par->sniff_par.anchor_point, (LD_ACL_SNIFF_TRANS_ANC_PROTEC_DELAY + rwip_prog_delay));
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            break;
        }

        // Check link supervision timeout
        if ((acl_par->lsto != LSTO_OFF) && (CLK_SUB(evt->time.hs, acl_par->last_hec_ok_ts) > 2 * acl_par->lsto))
        {
            // Report ACL end
            ld_acl_end(link_id, CO_ERROR_CON_TIMEOUT);
            break;
        }

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));
        // Move timestamp to next sniff anchor point
        evt->time.hs = acl_par->sniff_par.anchor_point;

        // Try to schedule normal ACL before next sniff anchor point
        SCH_ARB_ASAP_STG_TYPE_SET(evt, SCH_ARB_FLAG_NO_ASAP);
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            SCH_ARB_ASAP_STG_TYPE_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT);
            break;
        }

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));
        // Move timestamp after next sniff anchor point
        evt->time.hs = CLK_ADD_2(acl_par->sniff_par.anchor_point, 4);

        // Get the after next sniff anchor point
        acl_par->sniff_par.anchor_point = CLK_ADD_2(acl_par->sniff_par.anchor_point, 2 * acl_par->sniff_par.intv);
    }
}

#if MAX_NB_SYNC
/**
 ****************************************************************************************
 * @brief End of a SCO link
 ****************************************************************************************
 */
__STATIC void ld_sco_end(uint8_t sco_link_id)
{
    // Point to parameters
    struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_SCO_END_3_FUNC_BIT, sco_link_id);
#if (EAVESDROPPING_SUPPORT)
    if (sco_par->sample_tx_seqn)
    {
        // eSCO Tx SEQN sampling still pending: notify ED task that it could not be sampled
        struct ed_esco_tx_seqn_rd_ind *msg = KE_MSG_ALLOC(ED_ESCO_TX_SEQN_RD_IND, TASK_ED, TASK_NONE, ed_esco_tx_seqn_rd_ind);

        msg->link_id = sco_par->link_id;
        msg->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        msg->esco_tx_seqn = 0;
        msg->clkn = 0;

        ke_msg_send(msg);

        sco_par->sample_tx_seqn = false;
    }
#endif // EAVESDROPPING_SUPPORT

    //Check if LSTO happened on ACL link
    if (ld_acl_env[sco_par->link_id] != NULL)
    {
        struct ld_acl_env_tag *acl_par = ld_acl_env[sco_par->link_id];
        // Clear HV1 flag
        acl_par->sco_hv1_idx = 0;
        acl_par->sco_ev3_retx_idx = 0;
    }
#if defined(CFG_MEM_PROTECTION)
    else
    {
        // Remove permission/status of CS as now unused
        DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_ACL_INDEX(sco_par->link_id))), REG_EM_BT_CS_SIZE, false, false, false);
    }
#endif //defined(CFG_MEM_PROTECTION)

#if VOICE_OVER_HCI
    if (sco_par->data_path == AUDIO_DATA_PATH_HCI)
    {
        bt_util_buf_sync_clear(sco_link_id);
    }
#endif //VOICE_OVER_HCI

    // Un-register SCO link from active links
    ld_active_mode_set(LD_ACT_TYPE_SCO, sco_par->link_id, sco_link_id, LD_ACT_MODE_OFF);

    // Free event memory
    ke_free(ld_sco_env[sco_link_id]);
    ld_sco_env[sco_link_id] = NULL;

    // Disable PCM
    bt_pcmgencntl_set((co_read16p(&ld_env.pcm_settings[0]) | (sco_link_id << BT_VXCHSEL_LSB)) & (~BT_PCMEN_BIT));

    // Disable Audio interrupt
    bt_intcntl0_set(bt_intcntl0_get() & ~(BT_AUDIOINT0MSK_BIT << sco_link_id));

    // Turn off voice channel
    bt_e_scochancntl_pack(sco_link_id, 0, 0, 0, 0, 0);
}
/**
 ****************************************************************************************
 * @brief Modify of a SCO link
 ****************************************************************************************
 */
__STATIC void ld_sco_modify(uint8_t sco_link_id)
{
    // Point to parameters
    struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];
    uint8_t rxind = IDX_SCO_PACKET_NULL, txind = IDX_SCO_PACKET_NULL;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_MODIFY_4_FUNC_BIT, sco_link_id);

    if (sco_par->sync_type == SCO_TYPE)
    {
        // Find TX/RX packet index
        for (uint16_t i = 0 ; i < ARRAY_LEN(ld_sco_pkt_idx) ; i++)
        {
            if (ld_sco_pkt_idx[i][0] == sco_par->rx_pkt_type)
            {
                rxind = ld_sco_pkt_idx[i][1];
                txind = rxind;
                break;
            }
        }
    }
    else
    {
        // Find RX packet index
        for (uint16_t i = 0 ; i < ARRAY_LEN(ld_esco_pkt_idx) ; i++)
        {
            if (ld_esco_pkt_idx[i][0] == sco_par->rx_pkt_type)
            {
                rxind = ld_esco_pkt_idx[i][1];
                break;
            }
        }
        // Find TX packet index
        for (uint16_t i = 0 ; i < ARRAY_LEN(ld_esco_pkt_idx) ; i++)
        {
            if (ld_esco_pkt_idx[i][0] == sco_par->tx_pkt_type)
            {
                txind = ld_esco_pkt_idx[i][1];
                break;
            }
        }
    }

    // Turn off voice channel
    bt_e_scochancntl_e_scochanen_setf(sco_link_id, 0);

    // Update EDR settings
    bt_e_scoltcntl_e_scoedrtx_setf(sco_link_id, ld_sco_pkt_types[txind][1]);
    bt_e_scoltcntl_e_scoedrrx_setf(sco_link_id, ld_sco_pkt_types[rxind][1]);

    // Reconfigure reception and transmission
    bt_e_scotrcntl_pack(sco_link_id, 0, sco_par->tx_pkt_len, ld_sco_pkt_types[txind][0], sco_par->rx_pkt_len, ld_sco_pkt_types[rxind][0]);

    // Change Tesco
    bt_e_scochancntl_te_sco_setf(sco_link_id, sco_par->t_esco);

#if VOICE_OVER_HCI
    if (sco_par->data_path == AUDIO_DATA_PATH_HCI)
    {
        uint8_t intdelay = (2 * sco_par->t_esco - LD_SCO_AUDIO_IRQ_DELAY + 1) >> 1;

        // Compute RX/TX buffer sizes (depending on input coding format (8-bits or 16-bits))
        uint8_t tx_buf_size = sco_par->tx_pkt_len * (1 + sco_par->sample_size_16_bits);
        uint8_t rx_buf_size = sco_par->rx_pkt_len * (1 + sco_par->sample_size_16_bits);

        // Initialize TX queue
        co_list_init(&sco_par->queue_tx);

        // Initialize audio buffer allocation system
        bt_util_buf_sync_clear(sco_link_id);
        bt_util_buf_sync_init(sco_link_id, SYNC_TX_BUF_NB + (sco_par->bw_full), tx_buf_size, SYNC_RX_BUF_NB + (sco_par->bw_full), rx_buf_size);

        // Initialize RX/TX pointers
        bt_e_scocurrenttxptr_pack(sco_link_id, 0x0000, 0x0000);
        bt_e_scocurrentrxptr_pack(sco_link_id, 0x0000, 0x0000);

        // Enable Audio interrupt
        bt_intcntl0_set(bt_intcntl0_get() | (BT_AUDIOINT0MSK_BIT << sco_link_id));

        // Work-around for INTDELAY field: saturate value so that it fits within 5 bits
        intdelay = co_min(intdelay, ((1 << BT_INTDELAY_WIDTH) - 1));

        // Update interrupt delay
        bt_e_scochancntl_intdelay_setf(sco_link_id, intdelay);
    }
#endif // VOICE_OVER_HCI

    // Turn on voice channel
    bt_e_scochancntl_e_scochanen_setf(sco_link_id, 1);

    // Update the SCO to scheduling parameters
    sch_slice_per_add(BT_SCO, sco_link_id, 2 * sco_par->t_esco, 4 * HALF_SLOT_SIZE, (sco_par->retx_att_nb > 0));
}

/**
 ****************************************************************************************
 * @brief Schedule next SCO activity
 ****************************************************************************************
 */
__STATIC void ld_sco_sched(uint8_t sco_link_id)
{
    // Point to parameters
    struct sch_arb_elt_tag *sco_evt = &(ld_sco_env[sco_link_id]->evt);
    struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];
    struct ld_acl_env_tag *acl_par = ld_acl_env[sco_par->link_id];
    uint8_t sco_evt_res_slot = sco_par->rsvd_slots;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_SCHED_4_FUNC_BIT, sco_link_id);

    // Find the next SCO anchor point
    sco_par->anchor_point = CLK_ADD_2(sco_par->anchor_point, 2 * sco_par->t_esco);

    // Update event position with latest bit offset
    sco_evt->time.hus = acl_par->bit_off;

    /* Reserve one more slot for retransmission attempts in case there is no scanning activity ongoing
     * and ACL activity is slave or master polling */
    if (sch_slice_params.sco_retx_allowed && (sco_par->retx_att_nb > 0)
            && (((acl_par->role == SLAVE_ROLE) && !acl_par->tx_traffic)
                || ((acl_par->role == MASTER_ROLE) && !acl_par->rx_traffic && !acl_par->tx_traffic)))
    {
        sco_evt_res_slot += 1;
    }

    while (1)
    {
        // Try to schedule next entire SCO event
        sco_evt->time.hs    = sco_par->anchor_point;
        sco_evt->duration_min = 2 * (sco_evt_res_slot * SLOT_SIZE - LD_SCO_INTERFRAME_MARGIN);
        if (sch_arb_insert(sco_evt) == SCH_ARB_ERROR_OK)
            break;

        // If some additional duration for reTx was included
        if (sco_evt_res_slot > sco_par->rsvd_slots)
        {
            // Reduce event duration and try to schedule only the reserved slots
            sco_evt->duration_min = 2 * (sco_par->rsvd_slots * SLOT_SIZE - LD_SCO_INTERFRAME_MARGIN);
            if (sch_arb_insert(sco_evt) == SCH_ARB_ERROR_OK)
                break;
        }

        // If reTx is available on this eSCO link
        if (sco_par->retx_att_nb > 0)
        {
            // Shift timestamp and try to schedule the first retransmission attempt
            sco_evt->time.hs = CLK_ADD_2(sco_par->anchor_point, 2 * sco_par->rsvd_slots);
            if (sch_arb_insert(sco_evt) == SCH_ARB_ERROR_OK)
                break;
        }

        // Increment priority
        sco_evt->current_prio = RWIP_PRIO_ADD_2(sco_evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_SCO_DFT_IDX));
        // Move timestamp
        sco_par->anchor_point = CLK_ADD_2(sco_par->anchor_point, 2 * sco_par->t_esco);
    }
}

/**
 ****************************************************************************************
 * @brief Handle SCO reschedule alarm
 ****************************************************************************************
 */
__STATIC void ld_sco_resched_cbk(struct sch_alarm_tag *elt)
{
    struct ld_sco_env_tag *sco_par = CONTAINER_OF(elt, struct ld_sco_env_tag, resched_alarm);
    struct sch_arb_elt_tag *sco_evt = &(sco_par->evt);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_RESCHED_CBK_4_FUNC_BIT, elt);

    DBG_SWDIAG(SCO, RESCHED, 1);

    // Remove event
    sch_arb_remove(sco_evt, false);

    // Check if ACL link still exists
    if (ld_acl_env[sco_par->link_id] != NULL)
    {
        // Check if the SCO link parameters have changed
        if (sco_par->update == SCO_UPDATE_STEP1)
        {
            struct ld_acl_env_tag *acl_par = ld_acl_env[sco_par->link_id];
            int32_t diff;
            uint32_t clock = ld_read_clock();
            uint32_t master_clock = CLK_ADD_2(clock, acl_par->clk_off);

            // Make sure that the master clock considered is even (corresponds to a BT slot)
            if (master_clock & 0x1)
            {
                master_clock &= ~0x1;
                clock = CLK_SUB(clock, 1);
            }

            // Reposition on the new SCO anchor point
            diff = sco_par->d_esco - CO_MOD(((master_clock >> 1) ^ (sco_par->init << 26)), sco_par->t_esco);
            if (diff <= 0)
            {
                diff += sco_par->t_esco;
            }
            // When switching to a small eSCO interval, 1 interval is skipped for safety, otherwise the next SCO anchor point is scheduled
            if (sco_par->t_esco > LD_SCO_UPD_INTV_SKIP_THR)
            {
                diff -= sco_par->t_esco;
            }
            sco_par->anchor_point = CLK_ADD_2(clock, 2 * diff);

            // Update step 2: wait for the ongoing SCO frames to complete
            sco_par->update = SCO_UPDATE_STEP2;
            DBG_SWDIAG(SCO, UPDATE, sco_par->update);
        }

        if (sco_par->t_esco > 2)
        {
            // Directly reschedule the next interval
            ld_sco_sched(sco_par->sco_link_id);
        }
    }

    DBG_SWDIAG(SCO, RESCHED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_sco_evt_start_cbk(struct sch_arb_elt_tag *evt)
{

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_EVT_START_CBK_4_FUNC_BIT, evt);

    DBG_SWDIAG(SCO, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_sco_env_tag *sco_par = (struct ld_sco_env_tag *) evt;

        // Check if ACL link still exists
        if (ld_acl_env[sco_par->link_id] != NULL)
        {
            struct ld_acl_env_tag *acl_par = ld_acl_env[sco_par->link_id];
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(sco_par->link_id);
            uint8_t att_nb = 1;

#if (EAVESDROPPING_SUPPORT)
            if ((sco_par->sample_tx_seqn) && (sco_par->update == SCO_NO_UPDATE))
            {
                // Ensure that the HW is already toggling the eSCO Tx SEQN
                if (bt_e_scochancntl_tog_getf(sco_par->sco_link_id) == 1)
                {
                    // Sample eSCO Tx SEQN and provide it to ED task
                    struct ed_esco_tx_seqn_rd_ind *msg = KE_MSG_ALLOC(ED_ESCO_TX_SEQN_RD_IND, TASK_ED, TASK_NONE, ed_esco_tx_seqn_rd_ind);
                    msg->link_id = sco_par->link_id;
                    msg->status = CO_ERROR_NO_ERROR;
                    msg->esco_tx_seqn = bt_e_scotrcntl_txseqn_getf(sco_par->sco_link_id);
                    // The current eSCO Tx SEQN corresponds to the previous anchor point
                    msg->clkn = CLK_ADD_2(CLK_SUB(sco_par->anchor_point, 2 * sco_par->t_esco), acl_par->clk_off) >> 1;
                    ke_msg_send(msg);

                    sco_par->sample_tx_seqn = false;
                }
            }
#endif // EAVESDROPPING_SUPPORT

            // Program the reschedule via an alarm, at event start position (after programming delay)
            sco_par->resched_alarm.time.hs = CLK_SUB(evt->time.hs, 2);
            sco_par->resched_alarm.time.hus = 0;
            sch_alarm_set(&sco_par->resched_alarm);

            /*
             *  Verify that the target timestamp is a SCO reserved slot
             *  In case the slave phase just changed since scheduling, this event is obsolete, it is simply skipped,
             *  will be rescheduled with the new bit/clock offset
             */
            if ((evt->time.hs == sco_par->anchor_point) || (evt->time.hs == CLK_ADD_2(sco_par->anchor_point, 2 * sco_par->rsvd_slots)))
            {
                // Push the programming to SCH PROG
                struct sch_prog_params prog_par;
                prog_par.frm_cbk         = &ld_sco_frm_cbk;
                prog_par.time.hs         = evt->time.hs;
                prog_par.time.hus        = evt->time.hus;
                prog_par.cs_idx          = cs_idx;
                prog_par.dummy           = cs_idx | (sco_par->sco_link_id << 4);
                prog_par.bandwidth       = evt->duration_min;
                prog_par.prio_1          = evt->current_prio;
                prog_par.prio_2          = 0;
                prog_par.prio_3          = 0;
                prog_par.pti_prio        = (evt->time.hs == sco_par->anchor_point) ? BT_PTI_SCORSVD_IDX : BT_PTI_SCORETX_IDX;
                prog_par.add.bt.frm_type = (evt->time.hs == sco_par->anchor_point) ? SCH_BT_FRAME_TYPE_ESCO : SCH_BT_FRAME_TYPE_ESCO_RETX;
                prog_par.add.bt.vxchan   = sco_par->sco_link_id;
                prog_par.mode            = SCH_PROG_BT;
                sch_prog_push(&prog_par);

                sco_par->nb_prog++;

                // Retransmission attempts are discarded if the synchronization window is too large or an update is ongoing
                if ((acl_par->rx_win_size < 2 * HALF_SLOT_SIZE) && (sco_par->update == SCO_NO_UPDATE))
                {
                    att_nb += sco_par->retx_att_nb;
                }
                // Program number of retransmission attempts
                bt_e_scoltcntl_retxnb_setf(sco_par->sco_link_id, att_nb);

                // Update day counter if needed
                ld_acl_day_count_update(sco_par->link_id, CLK_ADD_2(evt->time.hs, acl_par->last_sync_clk_off));

#if  (EAVESDROPPING_SUPPORT)
                // Configure eSCOBINACKENA
                em_bt_linkcntl_escobinackena_setf(cs_idx, acl_par->esco_bin_ack_en);
#endif //(EAVESDROPPING_SUPPORT)

                // Program day counter for AES-CCM encryption
                bt_e_scodaycnt_set(sco_par->sco_link_id, acl_par->day_count);

                // For slave role, update timings for the next frame
                if (acl_par->role == SLAVE_ROLE)
                {
                    uint32_t rx_win_size_us = (acl_par->rx_win_size + 1) >> 1;

                    // Update timings for next frame
                    em_bt_clkoff0_setf(cs_idx, acl_par->clk_off & EM_BT_CLKOFF0_MASK);
                    em_bt_clkoff1_setf(cs_idx, (acl_par->clk_off >> 16));

                    // Set the RX window size. Check if the wide-open mode should be used
                    if (rx_win_size_us > EM_BT_CS_RXWINSZ_MAX)
                    {
                        if (CO_MOD(rx_win_size_us, SLOT_SIZE))
                        {
                            em_bt_wincntl_pack(cs_idx, 1, rx_win_size_us / SLOT_SIZE + 1);
                        }
                        else
                        {
                            em_bt_wincntl_pack(cs_idx, 1, rx_win_size_us / SLOT_SIZE);
                        }
                    }
                    else
                    {
                        em_bt_wincntl_pack(cs_idx, 0, (rx_win_size_us + 1) >> 1);
                    }
                }
            }

            // Reset SCO event priority
            evt->current_prio        = rwip_priority[RWIP_PRIO_SCO_DFT_IDX].value;

            if (sco_par->update == SCO_UPDATE_STEP3)
            {
                // Clear Update state
                sco_par->update = SCO_NO_UPDATE;
                DBG_SWDIAG(SCO, UPDATE, sco_par->update);
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(SCO, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_sco_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_EVT_CANCELED_CBK_4_FUNC_BIT, evt);

    DBG_SWDIAG(SCO, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_sco_env_tag *sco_par = (struct ld_sco_env_tag *) evt;

        do
        {
            // Check if ACL link still exists
            if (ld_acl_env[sco_par->link_id] == NULL)
                break;

            if (evt->duration_min > (uint32_t)(2 * (sco_par->rsvd_slots * SLOT_SIZE - LD_SCO_INTERFRAME_MARGIN)))
            {
                // Reduce event duration and try to schedule only the reserved slots
                evt->duration_min = 2 * (sco_par->rsvd_slots * SLOT_SIZE - LD_SCO_INTERFRAME_MARGIN);
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    break;

                // Shift timestamp and try to schedule the first retransmission attempt
                evt->time.hs = CLK_ADD_2(sco_par->anchor_point, 2 * sco_par->rsvd_slots);
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    break;
            }

            // Increment event priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_SCO_DFT_IDX));

            // Reschedule the next interval
            ld_sco_sched(sco_par->sco_link_id);
        }
        while (0);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(SCO, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_sco_frm_isr(uint8_t sco_link_id, uint32_t timestamp)
{

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_FRM_ISR_4_FUNC_BIT, sco_link_id, timestamp);

    DBG_SWDIAG(SCO, FRM_ISR, 1);

    do
    {
        // Point to parameters
        struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];
        struct ld_acl_env_tag *acl_par = ld_acl_env[sco_par->link_id];
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(sco_par->link_id);
        uint8_t nb_rx = em_bt_rxdesccnt_aclrxdesccnt_getf(cs_idx) + em_bt_rxdesccnt_audiorxdesccnt_getf(cs_idx);

        if (ld_sco_env[sco_link_id] == NULL)
        {
            ASSERT_INFO_FORCE(0, sco_link_id, 0);
            break;
        }

        // Decrement number of programmed frames
        sco_par->nb_prog--;

        // Check if ACL link still exists
        if (ld_acl_env[sco_par->link_id] == NULL)
        {
            // Flush potentially consumed descriptors
            while (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(sco_par->link_id)))
            {
                // Free RX descriptor
                ld_rxdesc_free();
            }
        }
        else
        {
            // Check receptions during the frame
            ld_acl_rx(sco_par->link_id, timestamp, nb_rx);

            // Check transmissions during the frame
            ld_acl_tx(sco_par->link_id, timestamp);
        }

        // End of SCO
        if (sco_par->end && (sco_par->nb_prog == 0))
        {
            ld_sco_end(sco_link_id);
            break;
        }

        // Check if ACL link still exists
        if (ld_acl_env[sco_par->link_id] == NULL)
            break;

        // Handle slave synchronization
        if (acl_par->role == SLAVE_ROLE)
        {
            // Check if phase has changed from last sync calculation
            if (!acl_par->phase_chg)
            {
                uint8_t  frrxok;
                uint16_t rxbit;

                // Read together cs_frrxok status and bit position where sync has been found (in half-us)
                em_bt_rxbit_unpack(cs_idx, &frrxok, &rxbit);
                // Convert to SW representation
                rxbit = HALF_SLOT_TIME_MAX - rxbit;

                // Check if a sync has been found in the frame
                if (frrxok)
                {
                    // Refresh slave synchronization as per new reception
                    acl_par->phase_chg = ld_acl_rx_sync(sco_par->link_id, timestamp, rxbit);
                }
                else
                {
                    // Widen the reception window if needed
                    acl_par->phase_chg = ld_acl_rx_no_sync(sco_par->link_id, CLK_ADD_2(timestamp, 4));
                }

                // Disable ACL programming if phase has changed
                if (acl_par->phase_chg && acl_par->prog_en)
                {
                    // Disable programming
                    sch_prog_disable(acl_par->link_id);
                    acl_par->prog_en = false;
                    DBG_SWDIAG(ACL, PROG_EN, 0);

                    // If no frame is currently programmed
                    if (acl_par->nb_prog == 0)
                    {
                        // Remove event
                        sch_arb_remove(&acl_par->evt, true);

                        // Reschedule according to the current mode
                        ld_acl_resched(acl_par->link_id, timestamp);
                    }
                }
            }
            else
            {
                // Clear phase change flag to restart the sync algorithm
                acl_par->phase_chg = false;
            }
        }

        if ((sco_par->nb_prog == 0) && (sco_par->update == SCO_UPDATE_STEP2))
        {
            // Update SCO
            ld_sco_modify(sco_link_id);

            if (sco_par->t_esco > 2)
            {
                // Update step 3: block the ACL->SCO frame replacement up to the first new SCO frame
                sco_par->update = SCO_UPDATE_STEP3;
            }
            else
            {
                // Clear Update state
                sco_par->update = SCO_NO_UPDATE;
            }
            DBG_SWDIAG(SCO, UPDATE, sco_par->update);
        }

    }
    while (0);

    DBG_SWDIAG(SCO, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle SkipET interrupt
 ****************************************************************************************
 */
__STATIC void ld_sco_sket_isr(uint8_t sco_link_id, uint32_t timestamp)
{

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_SKET_ISR_4_FUNC_BIT, sco_link_id, timestamp);

    if (ld_sco_env[sco_link_id] != NULL)
    {
        // Point to parameters
        struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

        // Decrement number of programmed frames
        sco_par->nb_prog--;

        if (sco_par->nb_prog == 0)
        {
            if (sco_par->end)
            {
                // End of SCO link
                ld_sco_end(sco_link_id);
            }
            else if (sco_par->update == SCO_UPDATE_STEP2)
            {
                // Update SCO
                ld_sco_modify(sco_link_id);

                if (sco_par->t_esco > 2)
                {
                    // Update step 3: block the ACL->SCO frame replacement up to the first new SCO frame
                    sco_par->update = SCO_UPDATE_STEP3;
                }
                else
                {
                    // Clear Update state
                    sco_par->update = SCO_NO_UPDATE;
                }
                DBG_SWDIAG(SCO, UPDATE, sco_par->update);
            }
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, sco_link_id, 0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_sco_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    uint8_t vxchan = dummy >> 4;
    dummy &= 0xF;

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_FRM_CBK_4_FUNC_BIT, timestamp, dummy, irq_type);

    ASSERT_INFO(EM_BT_CS_IDX_TO_LID(dummy) < MAX_NB_ACTIVE_ACL, dummy, irq_type);
    ASSERT_INFO(vxchan < MAX_NB_SYNC, vxchan, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_sco_frm_isr(vxchan, timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        ld_sco_sket_isr(vxchan, timestamp);
    }
    break;
    case SCH_FRAME_IRQ_RX:
        break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }
}
#endif //MAX_NB_SYNC



/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_acl_init(void)
{

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_INIT_2_FUNC_BIT);

    memset(&ld_acl_env[0], 0, sizeof(ld_acl_env));
#if (MAX_NB_SYNC > 0)
    memset(&ld_sco_env[0], 0, sizeof(ld_sco_env));
#endif // (MAX_NB_SYNC > 0)
}

void ld_acl_reset(void)
{

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RESET_2_FUNC_BIT);

    for (int link_id = (MAX_NB_ACTIVE_ACL - 1) ; link_id >= 0 ; link_id--)
    {
        if (ld_acl_env[link_id] != NULL)
        {
            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
            struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

            // Check if test mode was enabled
            if (acl_par->test_mode != NULL)
            {
                // Disable direct loopback
                bt_rftestfreq_directloopbacken_setf(0);
                // Disable test mode
                bt_rftestfreq_testmodeen_setf(0);

                // Release memory
                ke_free(acl_par->test_mode);
            }

            // Check if Role Switch memory has been allocated
            if (acl_par->rsw_par != NULL)
            {
                // Free Role Switch memory
                ke_free(acl_par->rsw_par);
            }

            // Check if SAM memory has been allocated
            if (acl_par->sam_info != NULL)
            {
                // Free SAM memory
                ke_free(acl_par->sam_info);

                DBG_SWDIAG(SAM, RMAP_EN, 0);
                DBG_SWDIAG(SAM, LMAP_EN, 0);
            }

            // Delete event
            ke_free(evt);
        }
    }
    memset(&ld_acl_env[0], 0, sizeof(ld_acl_env));

#if MAX_NB_SYNC
    for (int sco_link_id = (MAX_NB_SYNC - 1) ; sco_link_id >= 0 ; sco_link_id--)
    {
        if (ld_sco_env[sco_link_id] != NULL)
        {
            // Free event memory
            ke_free(ld_sco_env[sco_link_id]);
        }
    }
    memset(&ld_sco_env[0], 0, sizeof(ld_sco_env));
#endif //MAX_NB_SYNC
}

uint8_t ld_acl_start(uint8_t link_id, uint8_t role, struct ld_slave_timing_info *slave_timing_info_ptr)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_START_3_FUNC_BIT, uint8_t, link_id, role, slave_timing_info_ptr);

    // Check if ACL is inactive
    if (ld_acl_env[link_id] == NULL)
    {
        // Allocate event
        ld_acl_env[link_id] = LD_ALLOC_EVT(ld_acl_env_tag);

        if (ld_acl_env[link_id] != NULL)
        {
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);
            uint32_t clock = ld_read_clock();

            // Set permission/status of CS as R/W, retain initialize status
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, false);

            uint8_t lt_addr = em_bt_linkcntl_aclltaddr_getf(cs_idx);

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
            struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

            LD_INIT_EVT(evt, ld_acl_env_tag);

            // Initialize event parameters (common part)
            evt->cb_cancel       = &ld_acl_evt_canceled_cbk;
            evt->cb_start        = &ld_acl_evt_start_cbk;
            evt->cb_stop         = &ld_acl_evt_stop_cbk;
            evt->current_prio       = rwip_priority[RWIP_PRIO_ACL_DFT_IDX].value;
            evt->duration_min       = 2 * LD_ACL_EVT_DUR_MIN_DFT;
            evt->stop_latency       = LD_ACL_EVT_STOP_THR_MIN;

            // Initialize event parameters (ACL part)
            acl_par->link_id           = link_id;
            acl_par->role              = role;
            acl_par->lsto              = LSTO_DFT;
            acl_par->last_sync_ts      = clock;
            acl_par->last_hec_ok_ts    = clock;
            // Initialize values for clock offset, bit offset, phase etc.
            if (role == SLAVE_ROLE)
            {
                acl_par->last_sync_ts      = slave_timing_info_ptr->last_sync_ts;
                acl_par->last_sync_clk_off = slave_timing_info_ptr->last_sync_clk_off;
                acl_par->last_sync_bit_off = slave_timing_info_ptr->last_sync_bit_off;
                acl_par->clk_off           = acl_par->last_sync_clk_off;
                acl_par->bit_off           = acl_par->last_sync_bit_off;
                acl_par->phase            = (4 - (acl_par->clk_off & 0x03)) & 0x03;
                acl_par->rx_win_size       = 2 * NORMAL_WIN_SIZE;
            }
            acl_par->mode              = ACL_MODE_NORMAL;
            acl_par->t_poll            = POLL_INTERVAL_DFT;
            acl_par->flush_to          = AUTO_FLUSH_TIMEOUT_DFT;
            acl_par->peer_drift        = BT_MAX_DRIFT_SLEEP;
            acl_par->rx_max_slot       = MAX_SLOT_DFT;
            acl_par->allowed_br_packet_types  = ((1 << DM1_IDX) | (1 << DH1_IDX));
            acl_par->allowed_edr_packet_types = ((1 << DM1_IDX) | (1 << DH1_2_IDX) | (1 << DH1_3_IDX));
#if  (EAVESDROPPING_SUPPORT)
            acl_par->cust_evt_dur_min = 0;
            acl_par->piconet_msb = false;
            acl_par->esco_bin_ack_en = false;
#endif // EAVESDROPPING_SUPPORT
            acl_par->sam_info = NULL;
            acl_par->last_master_clock = RWIP_INVALID_TARGET_TIME;
            DBG_SWDIAG(ACL, PROG_EN, 0);
            DBG_SWDIAG(ACL, PARITY, acl_par->phase);
            DBG_SWDIAG(TX, SW_IDX, 0);
            DBG_SWDIAG(TX, HW_IDX, 0);
            DBG_SWDIAG(TX, TX_CNT, 0);
            co_list_init(&acl_par->queue_lmp_tx);
            co_list_init(&acl_par->queue_acl_tx);

            // Prepare CS
            em_bt_txrxcntl_nullflt_setf(cs_idx, 1);
            em_bt_loopcntl_att_nb_setf(cs_idx, 1);
            em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, 0)));
            em_bt_rxmaxbuf_bch2_rxmaxbuf_setf(cs_idx, ACL_DATA_BUF_SIZE);

            em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(CONNECT, TXBSY), RWIP_COEX_GET(CONNECT, RXBSY), RWIP_COEX_GET(CONNECT, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, em_bt_frcntl_format_getf(cs_idx));
            //TODO: Adjust the position to set tx power.
            em_bt_pwrcntl_txpwr_setf(cs_idx, rwip_rf.txpwr_max);

#if EAVESDROPPING_SUPPORT
            // Initialize binaural ACK, CRC duplicate filtering, eSCO force NACK to disable
            em_bt_linkcntl_aclbinackena_setf(cs_idx, 0);
            em_bt_linkcntl_escobinackena_setf(cs_idx, 0);
            em_bt_linkcntl_duplfltena_setf(cs_idx, 0);
            em_bt_txrxcntl_escofnak_setf(cs_idx, 0);
#endif //EAVESDROPPING_SUPPORT

            // Prepare Tx descriptors
            for (int i = 0 ; i < 2 ; i++)
            {
                uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, i);
                em_bt_txheader_txtype_setf(txdesc_idx, ((role == MASTER_ROLE) ? POLL_TYPE : ID_NUL_TYPE));
                em_bt_txheader_txltaddr_setf(txdesc_idx, lt_addr);
                em_bt_txheader_txdv_setf(txdesc_idx, 0);
                em_bt_txpheader_txlength_setf(txdesc_idx, 0);
                em_bt_txaclbufptr_setf(txdesc_idx, 0);
                em_bt_txlmbufptr_setf(txdesc_idx, 0);
                em_bt_txptr_pack(txdesc_idx, 1, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, (i ^ 0x1))));
            }

            // Default clear other control
            for (uint16_t i = 0 ; i < (sizeof(struct initialization_vector) / sizeof(uint16_t)) ; i++)
            {
                em_bt_iv_setf(cs_idx, i, 0);
            }
            em_bt_txccmpldcnt0_setf(cs_idx, 0);
            em_bt_txccmpldcnt1_setf(cs_idx, 0);
            em_bt_txccmpldcnt2_setf(cs_idx, 0);
            em_bt_rxccmpldcnt0_setf(cs_idx, 0);
            em_bt_rxccmpldcnt1_setf(cs_idx, 0);
            em_bt_rxccmpldcnt2_setf(cs_idx, 0);
#if (EAVESDROPPING_SUPPORT)
            em_bt_cidcntl_set(cs_idx, 0);
            em_bt_rxcrc_set(cs_idx, 0);
#endif // EAVESDROPPING_SUPPORT

            rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rwip_rf.txpwr_max_mod, rwip_rf.txpwr_max);



            // Try to schedule ACL before LSTO
            evt->time.hs = clock;
            evt->time.hus = acl_par->bit_off;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, acl_par->phase, 0, 0);
            evt->asap_limit = CLK_ADD_2(acl_par->last_hec_ok_ts, 2 * acl_par->lsto);

            GLOBAL_INT_DISABLE();

            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                acl_par->state = ACL_EVT_WAIT;

                // Register ACL link as active link
                ld_active_mode_set(LD_ACT_TYPE_ACL, link_id, 0, LD_ACT_MODE_ON);

                status = CO_ERROR_NO_ERROR;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }

            GLOBAL_INT_RESTORE();
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }

    return (status);
}

uint8_t ld_acl_stop(uint8_t link_id)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_STOP_3_FUNC_BIT, uint8_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        switch (acl_par->state)
        {
        case ACL_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(&(ld_acl_env[link_id]->evt), false);

            if (acl_par->mode == ACL_MODE_ROLE_SWITCH)
            {
                // Exit Role Switch
                ld_acl_rsw_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST, LD_CLOCK_UNDEF);
            }

            // Report ACL end
            ld_acl_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        break;

        case ACL_EVT_ACTIVE:
        {
            // Check if programming is enabled
            if (acl_par->prog_en)
            {
                // Disable programming
                sch_prog_disable(acl_par->link_id);
                acl_par->prog_en = false;
                DBG_SWDIAG(ACL, PROG_EN, 0);
            }

            // Check if a frame is ongoing
            if (acl_par->nb_prog == 0)
            {
                // Remove event
                sch_arb_remove(&acl_par->evt, true);

                // Report ACL end
                ld_acl_end(link_id, CO_ERROR_CON_TERM_BY_LOCAL_HOST);
            }
            else
            {
                // Move state
                acl_par->state = ACL_EVT_END;
            }
        }
        break;

        default:
        {
            // Nothing to do
            ASSERT_INFO_FORCE(0, acl_par->state, 0);
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_flow_off(uint8_t link_id)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_FLOW_OFF_BIT, uint8_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Disable ACl flow
        acl_par->tx_flow = false;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_flow_on(uint8_t link_id)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_FLOW_ON_BIT, uint8_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        uint32_t clock = ld_read_clock();

        // Enable ACl flow
        acl_par->tx_flow = true;

        // Program new transmissions
        ld_acl_tx_prog(link_id, clock);
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_data_tx(uint8_t link_id, struct bt_em_acl_buf_elt *buf_elt)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_DATA_TX_BIT, uint8_t, link_id, buf_elt);
#if (EAVESDROPPING_SUPPORT)
    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        if (acl_par->control_tx_prog && !acl_par->tx_prog_en && (acl_par->txdesc_cnt == 0) && co_list_is_empty(&acl_par->queue_acl_tx) && co_list_is_empty(&acl_par->queue_lmp_tx))
        {
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

            // Notify ED task that a transmission is pending
            struct ed_acl_tx_pending_ind *msg = KE_MSG_ALLOC(ED_ACL_TX_PENDING_IND, TASK_ED, TASK_NONE, ed_acl_tx_pending_ind);

            msg->link_id = link_id;
            msg->tx_pending = true;
            msg->tx_seqn = em_bt_acltxstat_lasttxseqn_getf(cs_idx);

            ke_msg_send(msg);
        }
    }
#endif // EAVESDROPPING_SUPPORT

    DBG_SWDIAG(DATA, ACL_TX, 1);
    DBG_SWDIAG(TX, ACL_TX, 1);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        uint8_t pbf = GETF(buf_elt->data_len_flags, BT_EM_ACL_PBF);

        uint32_t clock = ld_read_clock();

        // Check Packet boundary flag
        switch (pbf)
        {
        case PBF_1ST_NF_HL_FRAG:
        {
            // Clear L2CAP start timestamp of the new buffer
            buf_elt->l2cap_start_ts = LD_CLOCK_UNDEF;
            // Clear current L2CAP start timestamp
            acl_par->curr_l2cap_start_ts = LD_CLOCK_UNDEF;
        }
        break;
        case PBF_CONT_HL_FRAG:
        {
            // Store current L2CAP start timestamp in the new buffer
            buf_elt->l2cap_start_ts = acl_par->curr_l2cap_start_ts;
        }
        break;
        case PBF_1ST_HL_FRAG:
        case PBF_CMP_PDU:
        {
            // Store current time as L2CAP start timestamp of the new buffer
            buf_elt->l2cap_start_ts = clock;
            // Store current time as L2CAP start timestamp
            acl_par->curr_l2cap_start_ts = clock;
        }
        break;
        default:
        {
            ASSERT_ERR_FORCE(0);
        }
        break;
        }

        // Push at the end of ACL TX queue
        co_list_push_back(&acl_par->queue_acl_tx, &buf_elt->hdr);

        // Check if TX flow is ON
        if (acl_par->tx_flow)
        {
            // Program new transmissions
            ld_acl_tx_prog(link_id, clock);

            // If event is scheduled far in the future, schedule it asap
            if ((acl_par->state == ACL_EVT_WAIT) && ((acl_par->mode == ACL_MODE_NORMAL) || ((acl_par->mode == ACL_MODE_SNIFF) && (acl_par->sniff_par.ssr.ssr_active))))
            {
                struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);

                if (CLK_DIFF(clock, evt->time.hs) > 2 * LD_ACL_EVT_DELAY_RESCHED_MIN)
                {
                    // Extract event from schedule
                    sch_arb_remove(evt, false);

                    if (acl_par->mode == ACL_MODE_NORMAL)
                    {
                        // Reschedule ACL
                        evt->time.hs = clock;
                        ld_acl_sched(link_id);
                    }
                    else
                    {
                        // Find the previous anchor point
                        while (CLK_LOWER_EQ(clock, acl_par->sniff_par.anchor_point))
                        {
                            // Advance the anchor point by one interval
                            acl_par->sniff_par.anchor_point = CLK_SUB(acl_par->sniff_par.anchor_point, 2 * acl_par->sniff_par.intv);
                        }
                        // Reschedule next sniff anchor point
                        ld_acl_sniff_sched(link_id);
                    }
                }
            }
        }

        status = CO_ERROR_NO_ERROR;
    }

    DBG_SWDIAG(DATA, ACL_TX, 0);
    DBG_SWDIAG(TX, ACL_TX, 0);

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_data_flush(uint8_t link_id, uint8_t *nb_flushed, bool all)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_DATA_FLUSH_BIT, uint8_t, link_id, nb_flushed, all);

    *nb_flushed = 0;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        struct bt_em_acl_buf_elt *buf_elt = (struct bt_em_acl_buf_elt *) co_list_pick(&acl_par->queue_acl_tx);
        struct bt_em_acl_buf_elt *prev = NULL;

        while (buf_elt != NULL)
        {
            // Save the pointer to the next element
            struct bt_em_acl_buf_elt *next = (struct bt_em_acl_buf_elt *) co_list_next(&buf_elt->hdr);

            if (all || (buf_elt->l2cap_start_ts != LD_CLOCK_UNDEF))
            {
                // Extract the element from the list
                co_list_extract_after(&acl_par->queue_acl_tx, (struct co_list_hdr *) prev, (struct co_list_hdr *) buf_elt);

                // Free the buffer
                bt_util_buf_acl_tx_free(buf_elt->buf_ptr);

                // Increment number of flushed packets
                (*nb_flushed)++;
            }
            else
            {
                // Update the pointer to the previous element only if the item has not been removed from the list
                prev = buf_elt;
            }

            // Jump to the next element
            buf_elt = next;
        }

        // Check the current buffer element
        if (acl_par->curr_buf_elt != NULL)
        {
            // Check the next buffer element
            if (acl_par->next_buf_elt != NULL)
            {
                if (all || (acl_par->next_buf_elt->l2cap_start_ts != LD_CLOCK_UNDEF))
                {
                    // Check if there is Tx buffer data coming from this element
                    uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, !acl_par->txdesc_index_hw);
                    uint16_t data_len = GETF(acl_par->next_buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);
                    uint8_t txllid = em_bt_txpheader_txllid_getf(txdesc_idx);
                    uint16_t txbufptr = em_bt_txaclbufptr_getf(txdesc_idx);
                    if ((txllid == LLID_CONTINUE || txllid == LLID_START)
                            && ((txbufptr >= acl_par->next_buf_elt->buf_ptr) && (txbufptr < (acl_par->next_buf_elt->buf_ptr + data_len))))
                    {
                        em_bt_txpheader_txlength_setf(txdesc_idx, 0);
                        if (txllid != LLID_CONTINUE)
                        {
                            em_bt_txpheader_txllid_setf(txdesc_idx, LLID_CONTINUE);
                        }

                        if (acl_par->state == ACL_EVT_WAIT)
                        {
                            // Set txdone
                            em_bt_txptr_txdone_setf(txdesc_idx, 1);
                            // Clear descriptor fields
                            em_bt_txheader_txtype_setf(txdesc_idx, 0);
                            em_bt_txheader_txdv_setf(txdesc_idx, 0);

                            // Update TX pointer and counter
                            acl_par->txdesc_cnt--;
                            DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);
                            acl_par->txdesc_index_sw = !acl_par->txdesc_index_sw;
                            DBG_SWDIAG(TX, SW_IDX, acl_par->txdesc_index_sw);
                        }
                    }

                    // Free the buffer
                    bt_util_buf_acl_tx_free(acl_par->next_buf_elt->buf_ptr);

                    // Increment number of flushed packets
                    (*nb_flushed)++;

                    acl_par->next_buf_elt = NULL;
                    acl_par->data_len = 0;
                }
            }

            if (all || (acl_par->curr_buf_elt->l2cap_start_ts != LD_CLOCK_UNDEF))
            {
                // Check if there is Tx buffer data coming from this element
                uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, acl_par->txdesc_index_hw);
                uint16_t data_len = GETF(acl_par->curr_buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);
                for (uint8_t i = 0; i < 2; i++)
                {
                    uint8_t txllid = em_bt_txpheader_txllid_getf(txdesc_idx);
                    uint16_t txbufptr = em_bt_txaclbufptr_getf(txdesc_idx);
                    if ((txllid == LLID_CONTINUE || txllid == LLID_START)
                            && ((txbufptr >= acl_par->curr_buf_elt->buf_ptr) && (txbufptr < (acl_par->curr_buf_elt->buf_ptr + data_len))))
                    {
                        em_bt_txpheader_txlength_setf(txdesc_idx, 0);
                        if (txllid != LLID_CONTINUE)
                        {
                            em_bt_txpheader_txllid_setf(txdesc_idx, LLID_CONTINUE);
                        }

                        if (acl_par->state == ACL_EVT_WAIT)
                        {
                            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);
                            bool waitack = em_bt_acltxstat_waitack_getf(cs_idx);

                            if ((i == 1) || !waitack)
                            {
                                // Set txdone
                                em_bt_txptr_txdone_setf(txdesc_idx, 1);
                                // Clear descriptor fields
                                em_bt_txheader_txtype_setf(txdesc_idx, 0);
                                em_bt_txheader_txdv_setf(txdesc_idx, 0);

                                // Update TX pointer and counter
                                acl_par->txdesc_cnt--;
                                DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);
                                acl_par->txdesc_index_sw = !acl_par->txdesc_index_sw;
                                DBG_SWDIAG(TX, SW_IDX, acl_par->txdesc_index_sw);
                            }
                        }
                    }

                    // Check next descriptor
                    txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, !acl_par->txdesc_index_hw);
                }

                // Free the buffer
                bt_util_buf_acl_tx_free(acl_par->curr_buf_elt->buf_ptr);

                // Increment number of flushed packets
                (*nb_flushed)++;

                if (acl_par->next_buf_elt == NULL)
                {
                    acl_par->data_len = 0;
                }

                // Shift next pointer to current pointer
                acl_par->curr_buf_elt = acl_par->next_buf_elt;
                acl_par->next_buf_elt = NULL;
            }
        }
        else
        {
            ASSERT_ERR(acl_par->next_buf_elt == NULL);
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_lmp_tx(uint8_t link_id, struct bt_em_lmp_buf_elt *buf_elt)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_FLOW_OFF_BIT, uint8_t, link_id, buf_elt);
#if (EAVESDROPPING_SUPPORT)
    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        if (acl_par->control_tx_prog && !acl_par->tx_prog_en && (acl_par->txdesc_cnt == 0) && co_list_is_empty(&acl_par->queue_acl_tx) && co_list_is_empty(&acl_par->queue_lmp_tx))
        {
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);

            // Notify ED task that a transmission is pending
            struct ed_acl_tx_pending_ind *msg = KE_MSG_ALLOC(ED_ACL_TX_PENDING_IND, TASK_ED, TASK_NONE, ed_acl_tx_pending_ind);

            msg->link_id = link_id;
            msg->tx_pending = true;
            msg->tx_seqn = em_bt_acltxstat_lasttxseqn_getf(cs_idx);

            ke_msg_send(msg);
        }
    }
#endif // EAVESDROPPING_SUPPORT

    DBG_SWDIAG(DATA, LMP_TX, 1);
    DBG_SWDIAG(TX, LMP_TX, 1);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Check if a new packet can be prepared
        if (
#if (EAVESDROPPING_SUPPORT)
            (!acl_par->control_tx_prog || (acl_par->control_tx_prog && acl_par->tx_prog_en)) &&
#endif // EAVESDROPPING_SUPPORT
            ((acl_par->txdesc_cnt < 2) && ((acl_par->mode != ACL_MODE_ROLE_SWITCH) || (acl_par->rsw_par->step == ACL_RSW_STEP_PENDING))))
        {
            uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(acl_par->link_id, acl_par->txdesc_index_sw);

            // Fill descriptor
            em_bt_txheader_txtype_setf(txdesc_idx, DM1_TYPE);
            em_bt_txheader_txdv_setf(txdesc_idx, (acl_par->sco_hv1_idx && ld_acl_lmp_txdv(buf_elt->buf_ptr, buf_elt->length)));
            em_bt_txpheader_pack(txdesc_idx, 0, buf_elt->length, 1, LLID_CNTL);
            em_bt_txlmbufptr_setf(txdesc_idx, buf_elt->buf_ptr);

            // Release descriptor
            em_bt_txptr_txdone_setf(txdesc_idx, 0);

            // Update TX pointer and counter
            acl_par->txdesc_index_sw = !acl_par->txdesc_index_sw;
            DBG_SWDIAG(TX, SW_IDX, acl_par->txdesc_index_sw);
            acl_par->txdesc_cnt++;
            DBG_SWDIAG(TX, TX_CNT, acl_par->txdesc_cnt);

            // Indicate TX traffic
            if (acl_par->tx_traffic == 0)
            {
                acl_par->tx_traffic = 1;
                DBG_SWDIAG(ACL_TX, TRAFFIC, acl_par->tx_traffic);
            }

            // Check if event is waiting
            if ((acl_par->state == ACL_EVT_WAIT) && ((acl_par->mode == ACL_MODE_NORMAL) || ((acl_par->mode == ACL_MODE_SNIFF) && (acl_par->sniff_par.ssr.ssr_active))))
            {
                struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
                uint32_t clock = ld_read_clock();

                if (CLK_DIFF(clock, evt->time.hs) > 2 * LD_ACL_EVT_DELAY_RESCHED_MIN)
                {
                    // Extract event from schedule
                    sch_arb_remove(evt, false);

                    if (acl_par->mode == ACL_MODE_NORMAL)
                    {
                        // Reschedule ACL
                        evt->time.hs = clock;
                        ld_acl_sched(link_id);
                    }
                    else
                    {
                        // Find the previous anchor point
                        while (CLK_LOWER_EQ(clock, acl_par->sniff_par.anchor_point))
                        {
                            // Advance the anchor point by one interval
                            acl_par->sniff_par.anchor_point = CLK_SUB(acl_par->sniff_par.anchor_point, 2 * acl_par->sniff_par.intv);
                        }
                        // Reschedule next sniff anchor point
                        ld_acl_sniff_sched(link_id);
                    }
                }
            }
        }
        else
        {
            // Push at the end of LMP TX queue
            co_list_push_back(&acl_par->queue_lmp_tx, &buf_elt->hdr);
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(DATA, LMP_TX, 0);
    DBG_SWDIAG(TX, LMP_TX, 0);

    return (status);
}

uint8_t ld_acl_rsw_req(uint8_t link_id, uint8_t lt_addr, uint16_t slot_offset,  uint32_t instant, uint8_t page_scan_rep_mode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_REQ_2_FUNC_BIT, uint8_t, 5, link_id, lt_addr, slot_offset, instant, page_scan_rep_mode);

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Check that the ACl link is in normal mode
        if (acl_par->mode == ACL_MODE_NORMAL)
        {
            uint32_t clock = ld_read_clock();

            if (CLK_DIFF(clock + acl_par->clk_off, (instant << 1)) > 2 * LD_ACL_RSW_PREPARE_DELAY)
            {
                // Allocate memory buffer for storing data during Role Switch
                acl_par->rsw_par = ke_malloc_system(sizeof(struct ld_acl_rsw_evt_params), KE_MEM_KE_MSG);

                if (acl_par->rsw_par != NULL)
                {
                    // Initialize event parameters (ACL part)
                    acl_par->rsw_par->instant           = instant << 1;
                    acl_par->rsw_par->new_lt_addr       = lt_addr;
                    acl_par->rsw_par->slot_offset_ts    = clock;
                    acl_par->rsw_par->step              = ACL_RSW_STEP_PENDING;
                    acl_par->rsw_par->fhs_buf           = 0;

                    // Prepare FHS packet in case of slave->master switch
                    if (acl_par->role == SLAVE_ROLE)
                    {
                        // Allocate a buffer for FHS TX packet
                        struct bt_em_lmp_buf_elt *buf_elt = bt_util_buf_lmp_tx_alloc();

                        if (buf_elt != NULL)
                        {
                            // Save buffer address
                            acl_par->rsw_par->fhs_buf = buf_elt->buf_ptr;

                            // Pack FHS
                            ld_util_fhs_pk(acl_par->rsw_par->fhs_buf, &ld_env.local_bch[0], 0, page_scan_rep_mode, &ld_env.local_bd_addr, &ld_env.class_of_dev, lt_addr, MANDATORY_PAGE_SCAN_MODE);
                        }
                        else
                        {
                            ASSERT_ERR_FORCE(0);
                        }
                    }
                    else
                    {
                        // Store peer's slot offset value
                        acl_par->rsw_bit_off = slot_offset << 1;
                    }

                    // Set mode to Role Switch
                    acl_par->mode = ACL_MODE_ROLE_SWITCH;

                    status = CO_ERROR_NO_ERROR;
                }
                else
                {
                    status = CO_ERROR_HARDWARE_FAILURE;
                    ASSERT_ERR_FORCE(0);
                }
            }
            else
            {
                status = CO_ERROR_INSTANT_PASSED;
            }
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    return (status);
}

uint16_t ld_acl_flush_timeout_get(uint8_t link_id)
{
    uint16_t flush_to = 0;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_FLUSH_TIMEOUT_GET_BIT, uint16_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Get flush TO
        flush_to = acl_par->flush_to;
    }

    GLOBAL_INT_RESTORE();

    return flush_to;
}

void ld_acl_flush_timeout_set(uint8_t link_id, uint16_t flush_to)
{

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_FLUSH_TIMEOUT_SET_BIT, link_id, flush_to);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Set flush TO
        acl_par->flush_to = flush_to;
    }

    GLOBAL_INT_RESTORE();
}

uint16_t ld_acl_t_poll_get(uint8_t link_id)
{
    uint16_t t_poll = 0;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_T_POLL_GET_3_FUNC_BIT, uint16_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Get Tpoll
        t_poll = acl_par->t_poll;
    }

    GLOBAL_INT_RESTORE();

    return t_poll;
}

void ld_acl_t_poll_set(uint8_t link_id, uint16_t t_poll)
{

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_T_POLL_SET_3_FUNC_BIT, link_id, t_poll);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Set Tpoll
        acl_par->t_poll = t_poll;
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();
}

uint8_t ld_acl_sniff_trans(uint8_t link_id, uint16_t offset, uint16_t intv, uint16_t att, uint16_t to, uint8_t init)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_TRANS_3_FUNC_BIT, uint8_t, 6, link_id, offset, intv, att, to, init);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Check the current mode
        if (acl_par->mode == ACL_MODE_NORMAL || acl_par->mode == ACL_MODE_SNIFF_TRANS || acl_par->mode == ACL_MODE_SNIFF || acl_par->mode == ACL_MODE_SNIFF_ENTER)
        {
            // Initialize/clear sniff subrating parameters
            memset(&acl_par->sniff_par.ssr, 0, sizeof(struct ld_acl_evt_ssr_params));

            evt->current_prio = rwip_priority[RWIP_PRIO_ACL_SNIFF_TRANS_IDX].value;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, acl_par->phase, 0, 0);

            if (acl_par->mode == ACL_MODE_NORMAL || acl_par->mode == ACL_MODE_SNIFF_TRANS)
            {
                // If link is active, the new sniff parameters are stored
                acl_par->sniff_par.offset = offset;
                acl_par->sniff_par.intv = intv;
                acl_par->sniff_par.att = att;
                acl_par->sniff_par.to = to;
                acl_par->sniff_par.init = init;

                // Compute 1st anchor point
                ld_acl_sniff_first_anchor_compute(link_id, offset);
            }

            acl_par->mode = ACL_MODE_SNIFF_TRANS;

            status = CO_ERROR_NO_ERROR;
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_sniff(uint8_t link_id, uint16_t offset, uint16_t intv, uint16_t att, uint16_t to, uint8_t init)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SNIFF_3_FUNC_BIT, uint8_t, 6, link_id, offset, intv, att, to, init);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Check the current mode
        if (acl_par->mode == ACL_MODE_NORMAL || acl_par->mode == ACL_MODE_SNIFF_TRANS)
        {
            acl_par->sniff_par.offset = offset;
            acl_par->sniff_par.intv = intv;
            acl_par->sniff_par.att = att;
            acl_par->sniff_par.to = to;
            acl_par->sniff_par.init = init;
            // Initialize sniff subrating parameters
            memset(&acl_par->sniff_par.ssr, 0, sizeof(struct ld_acl_evt_ssr_params));

            if (acl_par->mode == ACL_MODE_NORMAL)
            {
                // Compute 1st anchor point
                ld_acl_sniff_first_anchor_compute(link_id, offset);
            }

            acl_par->mode = ACL_MODE_SNIFF_ENTER;

            status = CO_ERROR_NO_ERROR;
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_unsniff(uint8_t link_id)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_UNSNIFF_3_FUNC_BIT, uint8_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Check the current mode
        if (acl_par->mode == ACL_MODE_SNIFF_TRANS || acl_par->mode == ACL_MODE_SNIFF || acl_par->mode == ACL_MODE_SNIFF_ENTER)
        {
            if (acl_par->state == ACL_EVT_WAIT)
            {
                uint32_t clock = ld_read_clock();

                if (CLK_DIFF(clock, evt->time.hs) > 2 * LD_ACL_EVT_DELAY_RESCHED_MIN)
                {
                    // Extract event from schedule
                    sch_arb_remove(evt, false);

                    if (acl_par->mode == ACL_MODE_SNIFF)
                    {
                        // Exit sniff mode
                        ld_acl_sniff_exit(link_id);
                    }

                    // Reschedule with normal ACL rules
                    evt->time.hs = clock;
                    ld_acl_sched(link_id);
                }
            }

            acl_par->mode = ACL_MODE_NORMAL;

            status = CO_ERROR_NO_ERROR;
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_ssr_set(uint8_t link_id, uint8_t sub_rate, uint16_t to, uint32_t instant)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_SSR_SET_3_FUNC_BIT, uint8_t, link_id, sub_rate, to, instant);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Check the current mode
        if ((acl_par->mode == ACL_MODE_SNIFF) || (acl_par->mode == ACL_MODE_SNIFF_ENTER))
        {
            ASSERT_ERR((acl_par->sniff_par.ssr.ssr == 0) || acl_par->sniff_par.ssr.ssr_instant_passed);

            acl_par->sniff_par.ssr.ssr = sub_rate;
            acl_par->sniff_par.ssr.ssr_to = to;
            acl_par->sniff_par.ssr.ssr_instant = instant << 1;
            acl_par->sniff_par.ssr.ssr_instant_passed = false;
            acl_par->sniff_par.ssr.ssr_timer = false;
            acl_par->sniff_par.ssr.ssr_active = false;
            acl_par->sniff_par.ssr.ssr_restart_ts = 0;

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

void ld_acl_tx_enc(uint8_t link_id, uint8_t mode)
{
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_TX_ENC_3_FUNC_BIT, link_id, mode);

    // Set encryption mode for TX packets
    em_bt_txrxcntl_txcrypt_setf(cs_idx, mode);
}

void ld_acl_rx_enc(uint8_t link_id, uint8_t mode)
{
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RX_ENC_2_FUNC_BIT, link_id, mode);

    // Set encryption mode for RX packets
    em_bt_txrxcntl_rxcrypt_setf(cs_idx, mode);

#if EAVESDROPPING_SUPPORT
    {
        // Send the MSG to ED TASK
        struct ed_enc_chg_ind *msg = KE_MSG_ALLOC(ED_ENC_CHG_IND, TASK_ED, TASK_NONE, ed_enc_chg_ind);

        msg->link_id = link_id;
        msg->type    = mode;

        ke_msg_send(msg);
    }
#endif // EAVESDROPPING_SUPPORT
}

#if BCAST_ENC_SUPPORT
void ld_acl_bcst_rx_dec(uint8_t link_id, uint8_t mode)
{
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_BCST_RX_DEC_BIT, link_id, mode);

    // Enable/disable decryption of RX broadcast payload
    em_bt_txrxcntl_rxbcry_setf(cs_idx, mode);
}
#endif // BCAST_ENC_SUPPORT

void ld_acl_enc_key_load(uint8_t link_id, struct ltk *key, struct initialization_vector *iv)
{
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_ENC_KEY_LOAD_BIT, link_id, key, iv);

    for (uint16_t i = 0 ; i < (sizeof(struct ltk) / sizeof(uint16_t)) ; i++)
    {
        em_bt_sk_setf(cs_idx, i, co_read16p(&key->ltk[i * 2]));
    }

    if (iv != NULL)
    {
        for (uint16_t i = 0 ; i < (sizeof(struct initialization_vector) / sizeof(uint16_t)) ; i++)
        {
            em_bt_iv_setf(cs_idx, i, co_read16p(&iv->vect[i * 2]));
        }

        em_bt_txccmpldcnt0_setf(cs_idx, 0);
        em_bt_txccmpldcnt1_setf(cs_idx, 0);
        em_bt_txccmpldcnt2_setf(cs_idx, 0);
        em_bt_rxccmpldcnt0_setf(cs_idx, 0);
        em_bt_rxccmpldcnt1_setf(cs_idx, 0);
        em_bt_rxccmpldcnt2_setf(cs_idx, 0);
    }
}

rwip_time_t ld_acl_clock_offset_get(uint8_t link_id)
{
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    rwip_time_t res = { 0, 0, 0};

    //FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN

    GLOBAL_INT_DISABLE();

    if (acl_par != NULL)
    {
        // Fetch the clock offset from last known sync
        res.hs = acl_par->last_sync_clk_off;
        res.hus = acl_par->last_sync_bit_off;
    }

    GLOBAL_INT_RESTORE();

    return (res);
}

uint16_t ld_acl_rsw_slot_offset_get(uint8_t link_id)
{
    int32_t slot_offset = 0;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_SLOT_OFFSET_GET_2_FUNC_BIT, uint16_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        uint8_t phase = (4 - (acl_par->last_sync_clk_off & 0x03)) & 0x03;

        DBG_SWDIAG(RSW, TX_SLOTOFF, 1);

        // Get bit offset
        slot_offset = (acl_par->last_sync_bit_off + phase * HALF_SLOT_SIZE) >> 1;
        if (slot_offset < 0)
        {
            slot_offset += (2 * SLOT_SIZE) ;
        }
        else if (slot_offset >= (2 * SLOT_SIZE))
        {
            slot_offset -= (2 * SLOT_SIZE) ;
        }

        // Complement to 1250us
        if (slot_offset != 0)
        {
            slot_offset = (2 * SLOT_SIZE) - slot_offset;
        }

        // Store the values sent to the peer
        // The driver assumes that these values will be received by the peer, and used at the role switch
        acl_par->rsw_bit_off = acl_par->last_sync_bit_off;
        acl_par->rsw_clk_off = acl_par->last_sync_clk_off;
        DBG_SWDIAG(RSW, TX_PARITY, phase);

        DBG_SWDIAG(RSW, TX_SLOTOFF, 0);
    }

    GLOBAL_INT_RESTORE();

    return ((uint16_t)(slot_offset));
}

void ld_acl_rsw_slot_offset_set(uint8_t link_id, uint16_t slot_offset)
{
    GLOBAL_INT_DISABLE();

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSW_SLOT_OFFSET_SET_2_FUNC_BIT, link_id, slot_offset);

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Check if Role Switch is ongoing
        if (acl_par->rsw_par != NULL)
        {
            // Store slot offset and its reception timestamp
            acl_par->rsw_par->slot_offset_ts = ld_read_clock();
            acl_par->rsw_bit_off    = slot_offset << 1;
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();
}

uint16_t ld_acl_lsto_get(uint8_t link_id)
{
    uint16_t lsto = 0;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_LSTO_GET_2_FUNC_BIT, uint16_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Get Link Supervision Timeout
        lsto = acl_par->lsto;
    }

    GLOBAL_INT_RESTORE();

    return lsto;
}

void ld_acl_lsto_set(uint8_t link_id, uint16_t lsto)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_LSTO_SET_2_FUNC_BIT, link_id, lsto);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Set Link Supervision Timeout
        acl_par->lsto = lsto;
    }

    GLOBAL_INT_RESTORE();
}

void ld_acl_timing_accuracy_set(uint8_t link_id, uint8_t drift, uint8_t jitter)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_TIMING_ACCURACY_SET_3_FUNC_BIT, link_id, drift, jitter);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Set drift and jitter
        acl_par->peer_drift = drift;
        acl_par->peer_jitter = jitter;
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();
}

void ld_acl_edr_set(uint8_t link_id, bool en)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_EDR_SET_BIT, link_id, en);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

#if (EAVESDROPPING_SUPPORT)
        uint8_t ptt = en ? PACKET_TABLE_2_3MBPS : PACKET_TABLE_1MBPS;
        uint8_t old_ptt = acl_par->edr_en ? PACKET_TABLE_2_3MBPS : PACKET_TABLE_1MBPS;
#endif // EAVESDROPPING_SUPPORT

        // Set EDR enable flag
        acl_par->edr_en = en;

        // Configure HW to new data rate
        em_bt_linkcntl_acledr_setf(cs_idx, en);

#if (EAVESDROPPING_SUPPORT)
        if (ptt != old_ptt)
        {
            ld_ed_ptt_changed_ind(link_id, ptt);
        }
#endif // EAVESDROPPING_SUPPORT
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();
}

void ld_acl_allowed_tx_packet_types_set(uint8_t link_id, uint16_t packet_types)
{
    uint8_t br_types = (1 << DM1_IDX);
    uint8_t edr_types = (1 << DM1_IDX);

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_ALLOWED_TX_PACKET_TYPES_SET_BIT, link_id, packet_types);

    // Check basic rate packet types
    LD_ACL_EN_BR_PKT_TYPE(br_types, packet_types, DH1);
    LD_ACL_EN_BR_PKT_TYPE(br_types, packet_types, DM3);
    LD_ACL_EN_BR_PKT_TYPE(br_types, packet_types, DH3);
    LD_ACL_EN_BR_PKT_TYPE(br_types, packet_types, DM5);
    LD_ACL_EN_BR_PKT_TYPE(br_types, packet_types, DH5);

    // Check enhanced data rate packet types
    LD_ACL_EN_EDR_PKT_TYPE(edr_types, packet_types, DH1, 2);
    LD_ACL_EN_EDR_PKT_TYPE(edr_types, packet_types, DH1, 3);
    LD_ACL_EN_EDR_PKT_TYPE(edr_types, packet_types, DH3, 2);
    LD_ACL_EN_EDR_PKT_TYPE(edr_types, packet_types, DH3, 3);
    LD_ACL_EN_EDR_PKT_TYPE(edr_types, packet_types, DH5, 2);
    LD_ACL_EN_EDR_PKT_TYPE(edr_types, packet_types, DH5, 3);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Set allowed packet types
        acl_par->allowed_br_packet_types = br_types;
        acl_par->allowed_edr_packet_types = edr_types;
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();
}

void ld_acl_dm1_packet_type_dis(uint8_t link_id, bool disable)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_DM1_PACKET_TYPE_DIS_BIT, link_id, disable);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        if (disable)
        {
            // Set allowed packet types
            if (acl_par->allowed_br_packet_types & (~(1 << DM1_IDX)))
            {
                acl_par->allowed_br_packet_types &= ~(1 << DM1_IDX);
            }
            if (acl_par->allowed_edr_packet_types & (~(1 << DM1_IDX)))
            {
                acl_par->allowed_edr_packet_types &= ~(1 << DM1_IDX);
            }
        }
        else
        {
            acl_par->allowed_br_packet_types |= (1 << DM1_IDX);
            acl_par->allowed_edr_packet_types |= (1 << DM1_IDX);
        }
    }

    GLOBAL_INT_RESTORE();
}

#if MAX_NB_SYNC
void ld_esco_loopback_mode_en(uint8_t link_id, bool enable)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ESCO_LOOPBACK_MODE_EN_3_FUNC_BIT, link_id, enable);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        struct ld_sco_env_tag *sco_par;
        uint8_t sco_link_id;

        // Search for an eSCO corresponding to this ACL
        for (sco_link_id = 0; sco_link_id < MAX_NB_SYNC; sco_link_id++)
        {
            if (ld_sco_env[sco_link_id] != NULL)
            {
                sco_par = ld_sco_env[sco_link_id];
                if ((sco_par->link_id == link_id) && (sco_par->sync_type == ESCO_TYPE))
                {
                    break;
                }
            }
        }

        // If eSCO link is present
        if (sco_link_id < MAX_NB_SYNC)
        {
            if (enable)
            {
                // Set Tx and Rx pointers to the same buffers for eSCO loopback mode
                ASSERT_ERR((EM_BT_AUDIOBUF_OFF(sco_link_id) & 0x03) == 0);
                bt_e_scocurrenttxptr_pack(sco_link_id, EM_BT_AUDIOBUF_OFF(sco_link_id) >> 2, EM_BT_AUDIOBUF_OFF(sco_link_id) >> 2);
                bt_e_scocurrentrxptr_pack(sco_link_id, EM_BT_AUDIOBUF_OFF(sco_link_id) >> 2, EM_BT_AUDIOBUF_OFF(sco_link_id) >> 2);
            }
            else
            {
                // Restore Tx and Rx pointers
                bt_e_scocurrenttxptr_pack(sco_link_id, sco_par->escoptrtx1, sco_par->escoptrtx0);
                bt_e_scocurrentrxptr_pack(sco_link_id, sco_par->escoptrrx1, sco_par->escoptrrx0);
            }
        }
    }

    GLOBAL_INT_RESTORE();
}
#endif

uint8_t ld_acl_current_tx_power_get(uint8_t link_id, uint8_t mod)
{
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);
    uint8_t tx_pwr = 0;
    uint8_t rf_mod;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_CURRENT_TX_POWER_GET_BIT, uint8_t, link_id, mod);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        rf_mod = em_bt_frcntl_rf_modtype_getf(cs_idx);
        // Get TX power from CS
        if (rf_mod == MOD_IQ)
        {
            tx_pwr = em_bt_linkcntl_iqval_getf(cs_idx);
        }
        else
        {
            tx_pwr = em_bt_pwrcntl_txpwr_getf(cs_idx);
        }

        // Convert to dbM
        tx_pwr = LD_TXPWR_DBM_GET(tx_pwr, mod, rf_mod);
    }

    GLOBAL_INT_RESTORE();

    return tx_pwr;
}

#if PCA_SUPPORT
uint8_t ld_acl_clk_set(uint8_t link_id, uint32_t clk_off, int16_t bit_off)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_CLK_SET_BIT, uint8_t, link_id, clk_off, bit_off);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

        acl_par->clk_off = clk_off;
        acl_par->bit_off = bit_off;
        acl_par->last_sync_bit_off = acl_par->bit_off;
        acl_par->last_sync_clk_off = acl_par->clk_off;

        em_bt_clkoff0_setf(cs_idx, acl_par->clk_off & EM_BT_CLKOFF0_MASK);
        em_bt_clkoff1_setf(cs_idx, (acl_par->clk_off >> 16));

        // Prevent sync algorithm overwriting bit_off at end of current frame
        acl_par->phase_chg = true;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

__STATIC void ld_acl_clk_adj_instant_cbk(struct sch_alarm_tag *elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_CLK_ADJ_INSTANT_CBK_BIT, elt);

    DBG_SWDIAG(PCA, ALARM_INSTANT, 1);

    if (elt != NULL)
    {
        struct ld_acl_env_tag *acl_par = CONTAINER_OF(elt, struct ld_acl_env_tag, clk_adj_alarm);
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(acl_par->link_id);

        int16_t new_bit_off = acl_par->bit_off + (acl_par->bit_off_adj << 1);
        uint32_t new_clk_off = acl_par->clk_off + (acl_par->clk_off_adj << 1);

        // Adjust bit offset and clock offset if needed
        if (new_bit_off < 0)
        {
            uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
            new_bit_off += num_hs * HALF_SLOT_SIZE;
            new_clk_off = CLK_ADD_2(new_clk_off, num_hs);
        }
        else if (new_bit_off >= HALF_SLOT_SIZE)
        {
            uint8_t num_hs = new_bit_off / HALF_SLOT_SIZE;
            new_bit_off -= num_hs * HALF_SLOT_SIZE;
            new_clk_off = CLK_SUB(new_clk_off, num_hs);
        }

        acl_par->clk_off = new_clk_off;
        acl_par->bit_off = new_bit_off;
        acl_par->last_sync_bit_off = acl_par->bit_off;
        acl_par->last_sync_clk_off = acl_par->clk_off;

        em_bt_clkoff0_setf(cs_idx, new_clk_off & EM_BT_CLKOFF0_MASK);
        em_bt_clkoff1_setf(cs_idx, new_clk_off >> 16);

        // Prevent sync algorithm overwriting bit_off at end of current frame
        acl_par->phase_chg = true;

#if RW_BT_MWS_COEX
        // Allow immediate re-enable frame sync interrupt for monitoring external frame alignment
        ld_pca_update_target_offset(acl_par->link_id);
        ld_pca_reporting_enable(true);
#endif //RW_BT_MWS_COEX
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(PCA, ALARM_INSTANT, 0);
}

uint8_t ld_acl_clk_adj_set(uint8_t link_id, uint32_t clk_adj_slots, int16_t clk_adj_us, uint32_t clk_adj_instant)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_CLK_ADJ_SET_BIT, uint8_t, link_id, clk_adj_slots, clk_adj_us, clk_adj_instant);

    GLOBAL_INT_DISABLE();

    DBG_SWDIAG(PCA, COARSE_CLK_ADJ, 1);

    if (ld_acl_env[link_id] != NULL)
    {
        uint32_t clock = ld_read_clock();

        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Program clk_adj via alarm
        acl_par->clk_adj_alarm.time.hs = CLK_SUB((clk_adj_instant << 1), acl_par->clk_off + rwip_prog_delay);
        acl_par->clk_adj_alarm.cb_alarm = &ld_acl_clk_adj_instant_cbk;

        // If instant passed (due to clk_adj_mode), adjust the alarm timestamp accordingly -
        if (CLK_DIFF(clock, acl_par->clk_adj_alarm.time.hs) < rwip_prog_delay)
        {
            acl_par->clk_adj_alarm.time.hs = CLK_ADD_2(ld_read_clock(), rwip_prog_delay);
        }
        acl_par->clk_adj_alarm.time.hus = 0;

        acl_par->clk_off_adj = clk_adj_slots; // HW CLKOFF expressed in slots
        acl_par->bit_off_adj = 0 - clk_adj_us; //  logic inversion for bit_off representation

        sch_alarm_set(&acl_par->clk_adj_alarm);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(PCA, COARSE_CLK_ADJ, 0);

    GLOBAL_INT_RESTORE();

    return status;
}

#endif // PCA_SUPPORT

#if (EAVESDROPPING_SUPPORT || PCA_SUPPORT)

uint8_t ld_acl_role_get(uint8_t link_id)
{
    uint8_t role = UNKNOWN_ROLE;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        role = acl_par->role;
    }

    GLOBAL_INT_RESTORE();

    return role;
}

#endif // (0 || PCA_SUPPORT)

uint8_t ld_acl_afh_set(uint8_t link_id, uint32_t hssi, bool en, const struct bt_ch_map *ch_map)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_AFH_SET_BIT, uint8_t, link_id, hssi, en, ch_map);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

        ASSERT_ERR(acl_par->role == SLAVE_ROLE);

        switch (acl_par->afh_state)
        {
        case AFH_WAIT_INSTANT:
        {
            // Previous instant not passed, apply settings anyway
            if (en)
            {
                // Apply previous map
                ld_acl_afh_apply(acl_par->link_id, &acl_par->afh_map);
            }
            else
            {
                // Disable AFH
                em_bt_linkcntl_afhena_setf(cs_idx, 0);
            }
        } // No break

        case AFH_NOT_PENDING:
        {
            // Program HSSI via an alarm
            acl_par->afh_alarm.time.hs = CLK_SUB((hssi << 1), acl_par->clk_off + rwip_prog_delay);
            acl_par->afh_alarm.time.hus = 0;
            acl_par->afh_alarm.cb_alarm = (en) ? &ld_acl_afh_switch_on_cbk : &ld_acl_afh_switch_off_cbk;
            sch_alarm_set(&acl_par->afh_alarm);

            // Store channel map
            memcpy(&acl_par->afh_map.map[0], &ch_map->map[0], BT_CH_MAP_LEN);

            // Set state
            acl_par->afh_state = AFH_WAIT_INSTANT;

            status = CO_ERROR_NO_ERROR;
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, acl_par->afh_state, link_id);
        }
        break;
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();

#if (EAVESDROPPING_SUPPORT)
    ld_ed_afh_channel_map_changed_ind(link_id, en, hssi, ch_map);
#endif // EAVESDROPPING_SUPPORT

    return (status);
}

uint8_t ld_acl_afh_prepare(uint8_t link_id, uint32_t hssi, bool en, const struct bt_ch_map *ch_map)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_AFH_PREPARE_BIT, uint8_t, link_id, hssi, en, ch_map);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        ASSERT_ERR(acl_par->role == MASTER_ROLE);

        switch (acl_par->afh_state)
        {
        case AFH_NOT_PENDING:
        {
            // Program HSSI via an alarm
            acl_par->afh_alarm.time.hs = CLK_SUB((hssi << 1), acl_par->clk_off + rwip_prog_delay);
            acl_par->afh_alarm.time.hus = 0;
            acl_par->afh_alarm.cb_alarm = (en) ? &ld_acl_afh_switch_on_cbk : &ld_acl_afh_switch_off_cbk;
            sch_alarm_set(&acl_par->afh_alarm);

            // Store channel map
            memcpy(&acl_par->afh_map.map[0], &ch_map->map[0], BT_CH_MAP_LEN);

            // Set state
            acl_par->afh_state = AFH_WAIT_INSTANT_WAIT_ACK;

            status = CO_ERROR_NO_ERROR;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    GLOBAL_INT_RESTORE();

#if (EAVESDROPPING_SUPPORT)
    ld_ed_afh_channel_map_changed_ind(link_id, en, hssi, ch_map);
#endif // EAVESDROPPING_SUPPORT

    return (status);
}

uint8_t ld_acl_afh_confirm(uint8_t link_id)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_AFH_CONFIRM_BIT, uint8_t, link_id);

    GLOBAL_INT_DISABLE();

    DBG_SWDIAG(AFH, CFM, 1);

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        switch (acl_par->afh_state)
        {
        case AFH_WAIT_INSTANT_WAIT_ACK:
        {
            acl_par->afh_state = AFH_WAIT_INSTANT;
        }
        break;

        case AFH_WAIT_ACK:
        {
            // Apply new map
            ld_acl_afh_apply(acl_par->link_id, &acl_par->afh_map);

            // Clear AFH state
            acl_par->afh_state = AFH_NOT_PENDING;
        }
        break;

        case AFH_NOT_PENDING:
        case AFH_WAIT_INSTANT:
        default:
        {
            ASSERT_ERR_FORCE(0);
        }
        break;
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    DBG_SWDIAG(AFH, CFM, 0);

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_acl_active_hop_types_get(void)
{
    uint8_t active_hop_types = 0;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_ACL_1_PATCH_TYPE, LD_ACL_ACTIVE_HOP_TYPES_GET_BIT, uint8_t);

    for (int link_id = (MAX_NB_ACTIVE_ACL - 1); (link_id >= 0) && (active_hop_types != (LD_AFH_HOP_USED | LD_STD_HOP_USED)); link_id--)
    {
        if (ld_acl_env[link_id] != NULL)
        {
            if (em_bt_linkcntl_afhena_getf(EM_BT_CS_ACL_INDEX(link_id)))
            {
                active_hop_types |= LD_AFH_HOP_USED;
            }
            else
            {
                active_hop_types |= LD_STD_HOP_USED;
            }
        }
    }

    return active_hop_types;
}

uint8_t ld_acl_rx_max_slot_get(uint8_t link_id)
{
    uint8_t max_slot = 0;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RX_MAX_SLOT_GET_2_FUNC_BIT, uint8_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Get maximum number of slots peer device is allowed to use
        max_slot = acl_par->rx_max_slot;
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();

    return max_slot;
}

void ld_acl_rx_max_slot_set(uint8_t link_id, uint8_t max_slot)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RX_MAX_SLOT_SET_2_FUNC_BIT, link_id, max_slot);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Set maximum number of slots peer device is allowed to use
        acl_par->rx_max_slot = max_slot;
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();

#if (EAVESDROPPING_SUPPORT)
    ld_ed_max_slot_changed_ind(link_id, max_slot);
#endif // EAVESDROPPING_SUPPORT
}

void ld_acl_test_mode_set(uint8_t link_id, struct ld_acl_test_mode_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_ACL_TEST_MODE_SET_3_FUNC_BIT, link_id, params);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_acl_env[link_id]->evt);
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Check test mode scenario
        switch (params->test_scenario)
        {
        case PAUSE_MODE:
        {
            ASSERT_ERR(acl_par->test_mode != NULL);
        }
        break;
        case EXITTEST_MODE:
        {
            // Check test mode state
            if (acl_par->test_mode != NULL)
            {
                acl_par->test_mode->action = TEST_MODE_EXIT;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
        break;
        case TXTEST0_MODE   :
        case TXTEST1_MODE   :
        case TXTEST10_MODE  :
        case PRAND_MODE     :
        case ACLLOOP_MODE   :
        case SCOLOOP_MODE   :
        case ACLNOWHIT_MODE :
        case SCONOWHIT_MODE :
        case TXTEST1100_MODE:
        {
            // Check test mode state
            if (acl_par->test_mode == NULL)
            {
                // Allocate test mode parameters memory
                acl_par->test_mode = ke_malloc_system(sizeof(struct ld_acl_test_mode_tag), KE_MEM_ENV);

                // Initialize memory
                memset(acl_par->test_mode, 0, sizeof(struct ld_acl_test_mode_tag));
            }

            if (acl_par->test_mode != NULL)
            {
                acl_par->test_mode->action = TEST_MODE_ENTER;

                // Store new test mode parameters
                memcpy(&acl_par->test_mode->params, params, sizeof(struct ld_acl_test_mode_params));
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
        break;
        default:
        {
            ASSERT_ERR_FORCE(0);
        }
        break;
        }

        // Check if event is waiting (then the HW/SW interface can be accessed securely)
        if (acl_par->state == ACL_EVT_WAIT)
        {
            // Extract event from schedule
            sch_arb_remove(evt, false);

            // Apply test mode
            ld_acl_test_mode_update(link_id);

            // Reschedule with appropriate ACL rules
            ld_acl_sched(link_id);
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();
}

int8_t ld_acl_rssi_delta_get(uint8_t link_id)
{
    int8_t rssi_delta = 0;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_RSSI_DELTA_GET_2_FUNC_BIT, int8_t, link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Get last RSSI delta value computed over the accumulated samples
        rssi_delta = acl_par->last_rssi_delta;
    }

    GLOBAL_INT_RESTORE();

    return rssi_delta;
}

#if MAX_NB_SYNC
__STATIC void ld_sco_pcm_init(uint8_t sco_link_id, uint16_t audio_len, uint16_t interval)
{
    uint32_t Pyld = 1, Fpcm = 0, R = 0, RL = 0, n = 0, l = 0, W = 0, A = 0, OLC = 0;
    uint32_t Tpcm_frame = 0, Npcm_frame = 0, Npcm = 0, /*like FSYNCH = STEREO*/FSH = 0, pcmclklimit = 0, pcmclkval = 0;
    uint32_t audio_interv = interval;
    uint8_t slotnb = 1, samptype = 1, sampsz = 1;
    uint16_t rsamppad = 0, lsamppad = 0;

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_PCM_INIT_4_FUNC_BIT, sco_link_id, audio_len, interval);

    /*
     * Get the payload length, assuming the link is symmetric or completely
     * asymmetric (i.e. zero data on one way)
     */
    Pyld = audio_len;

    // For the following equations see FS
    Fpcm = bt_rwbtconf_clk_sel_getf();

    R = ((625 * Fpcm * audio_interv) << 8) / Pyld;
    //Floor(R)
    RL = R >> 8;

    n = (1 << 10) / Pyld;

    l = (Pyld * (R - (RL << 8))) >> 8 ;
    W = Pyld * n;
    A = W - (n * l);
    OLC = (uint16_t)(CO_MOD((69 * Fpcm), W) & 0xFFFF);

    // Configure PCM physical IF
    //Remove 1 clock cycle to keep some margin
    Tpcm_frame = ((625 * audio_interv)) / Pyld;
    Npcm_frame = Fpcm * Tpcm_frame;
    Npcm = slotnb * (FSH + 8 * (1 + samptype) * (1 + sampsz));

    pcmclkval = Npcm_frame / Npcm;
    pcmclklimit = (Npcm * (1 + pcmclkval)) - Npcm_frame;

    // Set PCM Pointer
    bt_pcmsourceptr_set(bt_e_scocurrentrxptr_get(sco_link_id));
    bt_pcmsinkptr_set(bt_e_scocurrenttxptr_get(sco_link_id));

    // Set PCM Pll
    bt_pcmpllcntl0_rl_setf(RL);
    bt_pcmpllcntl1_pack(OLC, A);
    bt_pcmpllcntl2_w_setf(W);

    // Set PCM Control
    bt_pcmpadding_pack(rsamppad, lsamppad);
    bt_pcmphyscntl0_set(co_read32p(&ld_env.pcm_settings[2]));
    bt_pcmphyscntl1_set((co_read16p(&ld_env.pcm_settings[6]) << 16) | (pcmclklimit << BT_PCMCLKLIMIT_LSB) | pcmclkval);
    bt_pcmgencntl_set(co_read16p(&ld_env.pcm_settings[0]) | (sco_link_id << BT_VXCHSEL_LSB) | BT_PCMEN_BIT);
}

uint8_t ld_sco_start(uint8_t link_id, uint8_t sco_link_id, struct ld_sco_params *params, bool remote_loopback)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct ld_sco_env_tag *sco_env = NULL;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_START_4_FUNC_BIT, uint8_t, link_id, sco_link_id, params, remote_loopback);

    GLOBAL_INT_DISABLE();

    do
    {
        if (ld_acl_env[link_id] == NULL)
        {
            ASSERT_ERR_FORCE(0);
            break;
        }

        if (ld_sco_env[sco_link_id] != NULL)
        {
            ASSERT_ERR_FORCE(0);
            break;
        }

        // Allocate SCO event
        sco_env = LD_ALLOC_EVT(ld_sco_env_tag);

        if (sco_env == NULL)
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
            break;
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    GLOBAL_INT_RESTORE();

    if (status == CO_ERROR_NO_ERROR)
    {
        // Point to parameters
        struct ld_sco_env_tag *sco_par = sco_env;
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        uint8_t rxind = IDX_SCO_PACKET_NULL, txind = IDX_SCO_PACKET_NULL;
        int32_t diff;
        uint8_t esco_frame_duration;
        uint8_t retx_att_nb;
        uint16_t rx_buf_size;
        uint16_t tx_buf_size;
        uint16_t mute_pattern = 0x0000;
        uint8_t linear_format = LINEAR_FORMAT_8_BITS;
        uint8_t sample_type = 0x00;
        uint8_t aulaw_code = 0x00;
        bool aulaw_en = false;
        bool cvsd_en = false;
        bool aircode_trans = (params->audio.air_coding == CODING_FORMAT_TRANSP);
        bool sample_size_16_bits = ((aircode_trans == false) &&
                                    ((params->audio.data_path == AUDIO_DATA_PATH_PCM) || (params->audio.in_sample_size > 8)));

        uint32_t clock = ld_read_clock();
        uint32_t master_clock = CLK_ADD_2(clock, acl_par->clk_off);

        // If day counter has not already been initialized
        if (acl_par->last_master_clock == RWIP_INVALID_TARGET_TIME)
        {
            // Initialize day counter
            acl_par->day_count = ((master_clock & (1 << 27)) == 0) && params->init;

            // Save master clock
            acl_par->last_master_clock = master_clock;
        }

        LD_INIT_EVT(sco_env, ld_sco_env_tag);

        if (params->sync_type == SCO_TYPE)
        {
            // Find TX/RX packet index
            for (uint16_t i = 0 ; i < ARRAY_LEN(ld_sco_pkt_idx) ; i++)
            {
                if (ld_sco_pkt_idx[i][0] == params->rx_pkt_type)
                {
                    rxind = ld_sco_pkt_idx[i][1];
                    txind = rxind;
                    break;
                }
            }
        }
        else
        {
            // Find RX packet index
            for (uint16_t i = 0 ; i < ARRAY_LEN(ld_esco_pkt_idx) ; i++)
            {
                if (ld_esco_pkt_idx[i][0] == params->rx_pkt_type)
                {
                    rxind = ld_esco_pkt_idx[i][1];
                    break;
                }
            }
            // Find TX packet index
            for (uint16_t i = 0 ; i < ARRAY_LEN(ld_esco_pkt_idx) ; i++)
            {
                if (ld_esco_pkt_idx[i][0] == params->tx_pkt_type)
                {
                    txind = ld_esco_pkt_idx[i][1];
                    break;
                }
            }
        }

        ASSERT_ERR((rxind < ARRAY_LEN(ld_sco_pkt_types)) && (txind < ARRAY_LEN(ld_sco_pkt_types)));

        // Compute number of retransmission attempts
        esco_frame_duration = ld_sco_pkt_types[rxind][2] + ld_sco_pkt_types[txind][2];
        retx_att_nb = params->w_esco / esco_frame_duration;

        // Initialize event parameters (common part)
        sco_env->evt.cb_cancel        = &ld_sco_evt_canceled_cbk;
        sco_env->evt.cb_start         = &ld_sco_evt_start_cbk;
        sco_env->evt.current_prio     = rwip_priority[RWIP_PRIO_SCO_DFT_IDX].value;
        SCH_ARB_ASAP_STG_SET((&(sco_env->evt)), SCH_ARB_FLAG_NO_ASAP, 0, 0, 0);

        // Initialize event parameters (SCO part)
        sco_par->link_id     = link_id;
        sco_par->sco_link_id = sco_link_id;
        sco_par->rx_pkt_len  = params->rx_pkt_len  ;
        sco_par->tx_pkt_len  = params->tx_pkt_len  ;
        sco_par->lt_addr     = params->lt_addr     ;
        sco_par->sync_type   = params->sync_type   ;
        sco_par->rx_pkt_type = params->rx_pkt_type ;
        sco_par->tx_pkt_type = params->tx_pkt_type ;
        sco_par->d_esco      = params->d_esco      ;
        sco_par->t_esco      = params->t_esco      ;
        sco_par->init        = params->init        ;
        sco_par->retx_att_nb = retx_att_nb;
        sco_par->resched_alarm.cb_alarm = &ld_sco_resched_cbk;
        sco_par->data_path   = params->audio.data_path;
        sco_par->bw_full     = (params->t_esco == (esco_frame_duration + params->w_esco));
        sco_par->rsvd_slots  = esco_frame_duration;
#if VOICE_OVER_HCI
        sco_par->sample_size_16_bits = sample_size_16_bits;
#endif //VOICE_OVER_HCI
        DBG_SWDIAG(SCO, UPDATE, sco_par->update);

        // Register SCO link as active link
        ld_active_mode_set(LD_ACT_TYPE_SCO, link_id, sco_link_id, LD_ACT_MODE_ON);

        GLOBAL_INT_DISABLE();

        // Link the SCO environment
        ld_sco_env[sco_link_id] = sco_env;

        GLOBAL_INT_RESTORE();

        if (!remote_loopback)
        {
            /*
             *  Configure Baseband Core
             */

            // Compute RX/TX buffer sizes (depending on input coding format (8-bits or 16-bits))
            tx_buf_size = params->tx_pkt_len * (1 + sample_size_16_bits);
            rx_buf_size = params->rx_pkt_len * (1 + sample_size_16_bits);

            // Select mute pattern
            mute_pattern = 0x0000;
            if ((params->audio.in_sample_size > LINEAR_FORMAT_8_BITS) && (params->audio.in_data_format == PCM_FORMAT_UNSIGNED))
            {
                if (params->audio.in_sample_size == LINEAR_FORMAT_13_BITS)
                {
                    mute_pattern = 0x0FFF;
                }
                else if (params->audio.in_sample_size == LINEAR_FORMAT_14_BITS)
                {
                    mute_pattern = 0x1FFF;
                }
                else
                {
                    mute_pattern = 0x7FFF;
                }
            }

            // Select voice coding
            if ((params->audio.air_coding < ARRAY_NB_COLUMNS(ld_sco_voice_coding)) && (params->audio.in_coding < ARRAY_LEN(ld_sco_voice_coding)))
            {
                uint8_t voice_coding = ld_sco_voice_coding[params->audio.in_coding][params->audio.air_coding];
                aulaw_code = (voice_coding & AULAW_CODE_MSK) >> AULAW_CODE_POS;
                aulaw_en = (voice_coding & AULAW_EN_MSK) >> AULAW_EN_POS;
                cvsd_en = (voice_coding & CVSD_EN_MSK) >> CVSD_EN_POS;
            }
            else
            {
                ASSERT_INFO_FORCE(0, params->audio.air_coding, params->audio.in_coding);
            }

            // Select linear format
            switch (params->audio.in_sample_size)
            {
            case 8:
                break;
            case 13:
            {
                linear_format = LINEAR_FORMAT_13_BITS;
            }
            break;
            case 14:
            {
                linear_format = LINEAR_FORMAT_14_BITS;
            }
            break;
            case 16:
            {
                linear_format = LINEAR_FORMAT_16_BITS;
            }
            break;
            default:
            {
                ASSERT_ERR_FORCE(0);
            }
            break;
            }

            // Select sample type
            switch (params->audio.in_data_format)
            {
            case PCM_FORMAT_1SCOMP:
            case PCM_FORMAT_2SCOMP:
            case PCM_FORMAT_SIGNMAG:
            case PCM_FORMAT_UNSIGNED:
            {
                sample_type = params->audio.in_data_format - 1;
            }
            break;
            default:
            {
                ASSERT_ERR_FORCE(0);
            }
            break;
            }

            // Enable reserved slot slave tx on error
            bt_rftestcntl_sserrren_setf(1);
            bt_rftestcntl_herrren_setf(1);

            // Configure muting
            bt_e_scomutecntl_pack(sco_link_id,
#if (EAVESDROPPING_SUPPORT)
                                  0, 0, 0, 0,
#endif // EAVESDROPPING_SUPPORT
                                  1,
                                  0,
                                  1,
                                  1,
                                  mute_pattern);

            // Configure (e)SCO Logical Transport
            bt_e_scoltcntl_pack(sco_link_id, 1, ld_sco_pkt_types[rxind][1], ld_sco_pkt_types[txind][1], (params->sync_type == ESCO_TYPE), params->lt_addr);

            // Initialize and configure reception and transmission
            bt_e_scotrcntl_pack(sco_link_id, 0, params->tx_pkt_len, ld_sco_pkt_types[txind][0], params->rx_pkt_len, ld_sco_pkt_types[rxind][0]);

            // Audio coding/decoding
            bt_audiocntl_pack(sco_link_id, linear_format, sample_type, aulaw_en, aulaw_code, cvsd_en, 0);


#if VOICE_OVER_HCI
            if (sco_par->data_path == AUDIO_DATA_PATH_HCI)
            {
                struct bt_em_sync_buf_elt *buf_elt_rx_new;
                uint8_t intdelay = (2 * sco_par->t_esco - LD_SCO_AUDIO_IRQ_DELAY + 1) >> 1;

                // Initialize TX queue
                co_list_init(&sco_par->queue_tx);

                // Initialize audio buffer allocation system
                bt_util_buf_sync_init(sco_link_id, SYNC_TX_BUF_NB + (sco_par->bw_full), (uint8_t) tx_buf_size, SYNC_RX_BUF_NB + (sco_par->bw_full), (uint8_t) rx_buf_size);

                // Allocate an RX buffer for this first reception
                buf_elt_rx_new = bt_util_buf_sync_rx_alloc(sco_link_id, 0);

                ASSERT_ERR(buf_elt_rx_new != NULL);

                // Initialize RX/TX pointers
                bt_e_scocurrenttxptr_pack(sco_link_id, 0x0000, 0x0000);
                bt_e_scocurrentrxptr_pack(sco_link_id, (buf_elt_rx_new->buf_ptr >> 2), 0x0000);

                // Enable Audio interrupt
                bt_intcntl0_set(bt_intcntl0_get() | (BT_AUDIOINT0MSK_BIT << sco_link_id));

                // Work-around for INTDELAY field: saturate value so that it fits within 5 bits
                intdelay = co_min(intdelay, ((1 << BT_INTDELAY_WIDTH) - 1));

                // Enable Voice channel (with SW transport mode)
                bt_e_scochancntl_pack(sco_link_id, aircode_trans, 1, 1, intdelay, params->t_esco);
            }
            else
#endif // VOICE_OVER_HCI
            {
                uint32_t Pyld = 1;

                // Configure fixed buffers for RX/TX (voice buffers are aligned to 4-bytes EM addresses)
                ASSERT_ERR((EM_BT_AUDIOBUF_OFF(sco_link_id) & 0x03) == 0);
                sco_par->escoptrtx0 = (EM_BT_AUDIOBUF_OFF(sco_link_id)) >> 2;
                sco_par->escoptrtx1 = (EM_BT_AUDIOBUF_OFF(sco_link_id) + CO_ALIGN4_HI(tx_buf_size)) >> 2;
                sco_par->escoptrrx0 = (EM_BT_AUDIOBUF_OFF(sco_link_id) + 2 * CO_ALIGN4_HI(tx_buf_size)) >> 2;
                sco_par->escoptrrx1 = (EM_BT_AUDIOBUF_OFF(sco_link_id) + 2 * CO_ALIGN4_HI(tx_buf_size) + CO_ALIGN4_HI(rx_buf_size)) >> 2;
                bt_e_scocurrenttxptr_pack(sco_link_id, sco_par->escoptrtx1, sco_par->escoptrtx0);
                bt_e_scocurrentrxptr_pack(sco_link_id, sco_par->escoptrrx1, sco_par->escoptrrx0);

                // Enable Voice channel
                bt_e_scochancntl_pack(sco_link_id, 0, 1, 0, 0, params->t_esco);

                /*
                 * Get the payload length, assuming the link is symmetric or completely
                 * asymmetric (i.e. zero data on one way)
                 */
                if (params->tx_pkt_len != 0)
                {
                    Pyld = params->tx_pkt_len;
                }
                else if (params->rx_pkt_len != 0)
                {
                    Pyld = params->rx_pkt_len;
                }

                // PCM is configured only for voice channel 0
                if (sco_link_id == 0)
                {
                    ld_sco_pcm_init(sco_link_id, Pyld, params->t_esco);
                }
            }
        }
        else // Remote Loopback Mode
        {
            // Configure (e)SCO Logical Transport
            bt_e_scoltcntl_pack(sco_link_id, 1, ld_sco_pkt_types[rxind][1], ld_sco_pkt_types[txind][1], (params->sync_type == ESCO_TYPE), params->lt_addr);

            // Initialize and configure reception and transmission
            bt_e_scotrcntl_pack(sco_link_id, 0, params->tx_pkt_len, ld_sco_pkt_types[txind][0], params->rx_pkt_len, ld_sco_pkt_types[rxind][0]);

            // Set Tx and Rx pointers to the same buffers for Remote Loopback Mode
            ASSERT_ERR((EM_BT_AUDIOBUF_OFF(sco_link_id) & 0x03) == 0);
            bt_e_scocurrenttxptr_pack(sco_link_id, EM_BT_AUDIOBUF_OFF(sco_link_id) >> 2, EM_BT_AUDIOBUF_OFF(sco_link_id) >> 2);
            bt_e_scocurrentrxptr_pack(sco_link_id, EM_BT_AUDIOBUF_OFF(sco_link_id) >> 2, EM_BT_AUDIOBUF_OFF(sco_link_id) >> 2);

            // Enable Voice channel
            bt_e_scochancntl_pack(sco_link_id, 0, 1, 0, 0, params->t_esco);
        }

        GLOBAL_INT_DISABLE();

        // Link the SCO environment
        //ld_sco_env[sco_link_id] = sco_env;

        // Register the SCO to scheduling parameters
        sch_slice_per_add(BT_SCO, sco_link_id, 2 * sco_par->t_esco, esco_frame_duration * 2 * HALF_SLOT_SIZE, (sco_par->retx_att_nb > 0));

        // Find the first eSCO anchor point
        diff = 2 * sco_par->d_esco - CO_MOD((master_clock ^ (sco_par->init << 27)), 2 * sco_par->t_esco);
        sco_par->anchor_point = CLK_ADD_2(clock, diff);

        if (sco_par->t_esco > 2)
        {
            // Re-schedule SCO according to new parameters
            ld_sco_sched(sco_link_id);

            // Special case for ACL scheduling: T_esco = 6 slots with retransmission window
            if ((sco_par->t_esco <= 6) && (sco_par->retx_att_nb > 0))
            {
                // Point to parameters
                struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

                acl_par->sco_ev3_retx_idx = sco_link_id + 1;
            }
        }
        else
        {
            // Point to parameters
            struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

            acl_par->sco_hv1_idx = sco_link_id + 1;
        }

        GLOBAL_INT_RESTORE();

#if (EAVESDROPPING_SUPPORT)
        {
            uint8_t   type_m, type_s, len_m, len_s, lt_addr;
            struct ld_acl_env_tag *acl_par = ld_acl_env[sco_par->link_id];

            if (acl_par->role == MASTER_ROLE)
            {
                type_m = params->tx_pkt_type;
                type_s = params->rx_pkt_type;
                len_m  = params->tx_pkt_len;
                len_s  = params->rx_pkt_len;
            }
            else
            {
                type_m = params->rx_pkt_type;
                type_s = params->tx_pkt_type;
                len_m  = params->rx_pkt_len;
                len_s  = params->tx_pkt_len;
            }

            if (params->sync_type == ESCO_TYPE)
            {
                lt_addr = params->lt_addr;
            }
            else
            {
                lt_addr = em_bt_linkcntl_aclltaddr_getf(EM_BT_CS_ACL_INDEX(link_id));
            }

            ld_ed_sco_changed_ind(link_id, ED_STATUS_CONNECTED,     params->sync_type, lt_addr,
                                  (params->init << 1),   params->d_esco, params->t_esco,    params->w_esco,
                                  type_m,  type_s,         len_m,             len_s,
                                  params->audio.air_coding);
        }
#endif // EAVESDROPPING_SUPPORT
    }

    return status;
}

uint8_t ld_sco_update(uint8_t sco_link_id, struct ld_sco_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_UPDATE_4_FUNC_BIT, uint8_t, sco_link_id, params);

    GLOBAL_INT_DISABLE();

    if (ld_sco_env[sco_link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *sco_evt = &(ld_sco_env[sco_link_id]->evt);
        struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];
        struct sch_arb_elt_tag *evt = &(ld_acl_env[sco_par->link_id]->evt);
        struct ld_acl_env_tag *acl_par = ld_acl_env[sco_par->link_id];
        uint8_t rxind = IDX_SCO_PACKET_NULL, txind = IDX_SCO_PACKET_NULL;
        uint8_t esco_frame_duration;
        uint8_t retx_att_nb;

        if (params->sync_type == SCO_TYPE)
        {
            // Find TX/RX packet index
            for (uint16_t i = 0 ; i < ARRAY_LEN(ld_sco_pkt_idx) ; i++)
            {
                if (ld_sco_pkt_idx[i][0] == params->rx_pkt_type)
                {
                    rxind = ld_sco_pkt_idx[i][1];
                    txind = rxind;
                    break;
                }
            }
        }
        else
        {
            // Find RX packet index
            for (uint16_t i = 0 ; i < ARRAY_LEN(ld_esco_pkt_idx) ; i++)
            {
                if (ld_esco_pkt_idx[i][0] == params->rx_pkt_type)
                {
                    rxind = ld_esco_pkt_idx[i][1];
                    break;
                }
            }
            // Find TX packet index
            for (uint16_t i = 0 ; i < ARRAY_LEN(ld_esco_pkt_idx) ; i++)
            {
                if (ld_esco_pkt_idx[i][0] == params->tx_pkt_type)
                {
                    txind = ld_esco_pkt_idx[i][1];
                    break;
                }
            }
        }

        // Compute number of retransmission attempts
        esco_frame_duration = ld_sco_pkt_types[rxind][2] + ld_sco_pkt_types[txind][2];
        retx_att_nb = params->w_esco / esco_frame_duration;

        // Update event parameters (common part)
        sco_evt->duration_min        = 2 * (esco_frame_duration * SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN);

        // Update event parameters (SCO part)
        sco_par->rx_pkt_len  = params->rx_pkt_len  ;
        sco_par->tx_pkt_len  = params->tx_pkt_len  ;
        sco_par->rx_pkt_type = params->rx_pkt_type ;
        sco_par->tx_pkt_type = params->tx_pkt_type ;
        sco_par->d_esco      = params->d_esco      ;
        sco_par->t_esco      = params->t_esco      ;
        sco_par->init        = params->init        ;
        sco_par->retx_att_nb = retx_att_nb;
        sco_par->bw_full     = (params->t_esco == (esco_frame_duration + params->w_esco));
        sco_par->rsvd_slots  = esco_frame_duration;

        if (acl_par->sco_hv1_idx)
        {
            // Compute the first SCO anchor point
            uint32_t clock = ld_read_clock();
            uint32_t master_clock = CLK_ADD_2(clock, acl_par->last_sync_clk_off);
            int32_t diff = sco_par->d_esco - CO_MOD(((master_clock >> 1) ^ (sco_par->init << 26)), sco_par->t_esco);
            if (diff <= 0)
            {
                diff += sco_par->t_esco;
            }
            diff -= sco_par->t_esco;
            sco_par->anchor_point = CLK_ADD_2(clock, 2 * diff);

            // Directly schedule the next SCO frame
            ld_sco_sched(sco_link_id);

            // Clear HV1 flag
            acl_par->sco_hv1_idx = 0;
        }
        else if (sco_par->t_esco == 2)
        {
            acl_par->sco_hv1_idx = sco_link_id + 1;

            if (acl_par->state == ACL_EVT_WAIT)
            {
                uint32_t clock = ld_read_clock();

                if (CLK_DIFF(clock, evt->time.hs) > 2 * LD_ACL_EVT_DELAY_RESCHED_MIN)
                {
                    // Extract event from schedule
                    sch_arb_remove(evt, false);

                    // Reschedule with appropriate ACL rules
                    evt->time.hs = clock;
                    ld_acl_sched(sco_par->link_id);
                }
            }
        }

        // Update step 1: wait for the next reschedule
        sco_par->update      = SCO_UPDATE_STEP1;
        DBG_SWDIAG(SCO, UPDATE, sco_par->update);

#if VOICE_OVER_HCI
        // Disable Audio interrupt to prevent sending packet to the host with non correct size
        bt_intcntl0_set(bt_intcntl0_get() & ~(BT_AUDIOINT0MSK_BIT << sco_link_id));
#endif // VOICE_OVER_HCI

        status = CO_ERROR_NO_ERROR;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();



#if (EAVESDROPPING_SUPPORT)
    {
        uint8_t   type_m, type_s, len_m, len_s, lt_addr;

        struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];
        struct ld_acl_env_tag *acl_par = ld_acl_env[sco_par->link_id];

        if (params->sync_type == ESCO_TYPE)
        {
            lt_addr = params->lt_addr;
        }
        else
        {
            lt_addr = em_bt_linkcntl_aclltaddr_getf(EM_BT_CS_ACL_INDEX(sco_par->link_id));
        }

        if (acl_par->role == MASTER_ROLE)
        {
            type_m = params->tx_pkt_type;
            type_s = params->rx_pkt_type;
            len_m  = params->tx_pkt_len;
            len_s  = params->rx_pkt_len;
        }
        else
        {
            type_m = params->rx_pkt_type;
            type_s = params->tx_pkt_type;
            len_m  = params->rx_pkt_len;
            len_s  = params->tx_pkt_len;
        }

        ld_ed_sco_changed_ind(acl_par->link_id, ED_STATUS_CONNECTED,     params->sync_type, lt_addr,
                              (params->init << 1),   params->d_esco, params->t_esco,    params->w_esco,
                              type_m,  type_s,         len_m,             len_s,
                              params->audio.air_coding);
    }
#endif // EAVESDROPPING_SUPPORT

    return status;
}

uint8_t ld_sco_stop(uint8_t sco_link_id)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
#if (EAVESDROPPING_SUPPORT)
    uint8_t   sync_type, sco_status, flags, link_id, lt_addr;
    uint8_t   type_m, type_s, len_m, len_s;
    uint8_t   air_mode, d_esco, t_esco, w_esco;
#endif // EAVESDROPPING_SUPPORT
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_4_PATCH_TYPE, LD_SCO_STOP_4_FUNC_BIT, uint8_t, sco_link_id);

    GLOBAL_INT_DISABLE();

    if (ld_sco_env[sco_link_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *sco_evt = &(ld_sco_env[sco_link_id]->evt);
        struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

#if (EAVESDROPPING_SUPPORT)
        link_id      = sco_par->link_id;
        lt_addr      = sco_par->lt_addr;
        sync_type    = sco_par->sync_type;
        sco_status   = ED_STATUS_NOT_CONNECTED;
        flags        = 0;
        d_esco       = sco_par->d_esco;
        t_esco       = sco_par->t_esco;
        w_esco       = 0;
        type_m       = 0;
        type_s       = 0;
        len_m        = 0;
        len_s        = 0;
        air_mode     = 0;
#endif // EAVESDROPPING_SUPPORT

        // Extract event from schedule
        sch_arb_remove(sco_evt, false);

        // Extract alarm from schedule
        sch_alarm_clear(&sco_par->resched_alarm);

        // Check potential ongoing SCO frame
        if (sco_par->nb_prog == 0)
        {
            // End of SCO
            ld_sco_end(sco_link_id);
        }
        else
        {
            // End SCO after ongoing frame
            sco_par->end = true;
        }

        // Unregister the SCO from scheduling parameters
        sch_slice_per_remove(BT_SCO, sco_link_id);

#if (EAVESDROPPING_SUPPORT)
        ld_ed_sco_changed_ind(link_id, sco_status, sync_type, lt_addr,
                              flags,   d_esco,     t_esco,    w_esco,
                              type_m,  type_s,     len_m,     len_s,
                              air_mode);

#endif // EAVESDROPPING_SUPPORT
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}

#if VOICE_OVER_HCI
void ld_sco_audio_isr(uint8_t sco_link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ACL_3_PATCH_TYPE, LD_SCO_AUDIO_ISR_3_FUNC_BIT, sco_link_id);

    if (ld_sco_env[sco_link_id] != NULL)
    {
        DBG_SWDIAG(SCO, AUDIO_ISR, 1);

        // Point to parameters
        struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

        struct lc_sync_rx_ind *msg;
        struct bt_em_sync_buf_elt *buf_elt_rx_new, *buf_elt_tx_new;
        uint16_t buf_ptr_rx_old = 0, buf_ptr_tx_old = 0;
        uint8_t rx_invalid = 0;
        uint8_t tog = 0;

        ASSERT_ERR(sco_par->data_path == AUDIO_DATA_PATH_HCI);

        // Get new TX buffer from queue
        buf_elt_tx_new = (struct bt_em_sync_buf_elt *) co_list_pop_front(&sco_par->queue_tx);

        // Allocate new RX buffer
        buf_elt_rx_new = bt_util_buf_sync_rx_alloc(sco_link_id, 0);

        // Make the buffer switch in HW registers

        // Get current tog
        tog = bt_e_scochancntl_tog_getf(sco_link_id);

        /*
         * Get/clear old buffers pointers
         *   - bandwidth not full => current tog, as current event is considered finished
         *   - bandwidth full     => opposite tog, as current event is considered ongoing
         */
        if (tog ^ !sco_par->bw_full)
        {
            // Get old buffers pointers
            buf_ptr_tx_old = bt_e_scocurrenttxptr_e_scoptrtx_0_getf(sco_link_id) << 2;
            buf_ptr_rx_old = bt_e_scocurrentrxptr_e_scoptrrx_0_getf(sco_link_id) << 2;

            // Check reception status
            rx_invalid = bt_e_scomutecntl_invl_0_getf(sco_link_id);

            // Clear old buffer pointers
            bt_e_scocurrenttxptr_e_scoptrtx_0_setf(sco_link_id, 0);
            bt_e_scocurrentrxptr_e_scoptrrx_0_setf(sco_link_id, 0);
        }
        else
        {
            // Get old buffers pointers
            buf_ptr_tx_old = bt_e_scocurrenttxptr_e_scoptrtx_1_getf(sco_link_id) << 2;
            buf_ptr_rx_old = bt_e_scocurrentrxptr_e_scoptrrx_1_getf(sco_link_id) << 2;

            // Check reception status
            rx_invalid = bt_e_scomutecntl_invl_1_getf(sco_link_id);

            // Clear old buffer pointers
            bt_e_scocurrenttxptr_e_scoptrtx_1_setf(sco_link_id, 0);
            bt_e_scocurrentrxptr_e_scoptrrx_1_setf(sco_link_id, 0);
        }

        /*
         * Set new buffers pointers
         *    - always use opposite tog, to prepare the buffers used in the next SCO event
         */
        if (buf_elt_tx_new)
        {
            ASSERT_ERR((buf_elt_tx_new->buf_ptr & 0x03) == 0);
            if (!tog)
            {
                bt_e_scocurrenttxptr_e_scoptrtx_1_setf(sco_link_id, buf_elt_tx_new->buf_ptr >> 2);
            }
            else
            {
                bt_e_scocurrenttxptr_e_scoptrtx_0_setf(sco_link_id, buf_elt_tx_new->buf_ptr >> 2);
            }
        }
        if (buf_elt_rx_new)
        {
            ASSERT_ERR(((buf_elt_rx_new->buf_ptr) & 0x03) == 0);
            if (!tog)
            {
                bt_e_scocurrentrxptr_e_scoptrrx_1_setf(sco_link_id, buf_elt_rx_new->buf_ptr >> 2);
            }
            else
            {
                bt_e_scocurrentrxptr_e_scoptrrx_0_setf(sco_link_id, buf_elt_rx_new->buf_ptr >> 2);
            }
        }

        if (buf_ptr_tx_old)
        {
            // Report packet transmission confirmation
            struct lc_sync_tx_cfm *cfm = KE_MSG_ALLOC(LC_SYNC_TX_CFM, KE_BUILD_ID(TASK_LC, sco_par->link_id), TASK_NONE, lc_sync_tx_cfm);
            cfm->sync_link_id = sco_link_id;
            ke_msg_send(cfm);

            // Free previously transmitted buffer
            bt_util_buf_sync_tx_free(sco_link_id, buf_ptr_tx_old);
        }

        if (buf_ptr_rx_old)
        {
            // Report previously received packet
            msg = KE_MSG_ALLOC(LC_SYNC_RX_IND, KE_BUILD_ID(TASK_LC, sco_par->link_id), TASK_NONE, lc_sync_rx_ind);
            msg->em_buf = buf_ptr_rx_old;
            msg->data_len = sco_par->rx_pkt_len * (1 + sco_par->sample_size_16_bits);
            msg->packet_status_flag = rx_invalid;
            msg->sync_link_id = sco_link_id;
            ke_msg_send(msg);
        }

        DBG_SWDIAG(SCO, AUDIO_ISR, 0);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

uint8_t ld_sco_data_tx(uint8_t sco_link_id, struct bt_em_sync_buf_elt *buf_elt)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ACL_3_PATCH_TYPE, LD_SCO_DATA_TX_3_FUNC_BIT, uint8_t, sco_link_id, buf_elt);

    GLOBAL_INT_DISABLE();

    if (ld_sco_env[sco_link_id] != NULL)
    {
        // Point to parameters
        struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

        ASSERT_ERR(sco_par->data_path == AUDIO_DATA_PATH_HCI);

        // Push at the end of SCO TX queue
        co_list_push_back(&sco_par->queue_tx, &buf_elt->hdr);

        status = CO_ERROR_NO_ERROR;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();

    return status;
}
#endif // VOICE_OVER_HCI

#if EAVESDROPPING_SUPPORT
uint8_t ld_sco_lrd_set(uint8_t link_id, bool en, bool null_flt)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Set eSCO binary ack enable
        acl_par->esco_bin_ack_en = en;

        // Save NULL filtering setting
        acl_par->null_flt = (en) ? null_flt : true;

        // Check if the link is currently under binaural ACK
        if (en || (MAX_NB_ACTIVE_ACL != GETF(ld_aes_ccm_cnt, ED_AES_CCM_CNT_LINK_ID)))
        {
            // Set eavesdropping interface enable control
            bt_binackinfo_edifen_setf(en);
        }

        // Remember which link has Bin ack enabled
        if (en)
        {
            SETF(ld_aes_ccm_cnt, ED_AES_CCM_CNT_LINK_ID, link_id);
            SETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_CON_TYPE, ED_CON_TYPE_ESCO);
            SETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_BLOCK, 0);
        }
        else
        {
            SETF(ld_aes_ccm_cnt, ED_AES_CCM_CNT_LINK_ID, MAX_NB_ACTIVE_ACL);
            SETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_BLOCK, 1);
        }

        // Set the CS
        {
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);
            em_bt_txrxcntl_nullflt_setf(cs_idx, acl_par->null_flt);
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}
#endif //EAVESDROPPING_SUPPORT

#endif //MAX_NB_SYNC

#if RW_BT_MWS_COEX
void ld_acl_local_sam_index_set(uint8_t link_id, uint8_t sam_idx, uint32_t instant)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_LOCAL_SAM_INDEX_SET_2_FUNC_BIT, link_id, sam_idx, instant);

    DBG_SWDIAG(SAM, LMAP_SET, 1);

    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    ASSERT_ERR((acl_par != NULL) && ((sam_idx == SAM_DISABLED) || (sam_idx < SAM_INDEX_MAX)));

    GLOBAL_INT_DISABLE();

    if (NULL == acl_par->sam_info)
    {
        // Allocate memory buffer for storing SAM data
        acl_par->sam_info = ke_malloc_system(sizeof(struct ld_acl_sam_info_tag), KE_MEM_KE_MSG);
        acl_par->sam_info->map_length = 0;
        acl_par->sam_info->peer_map_pending = false;
        acl_par->sam_info->sam_change = ACL_SAM_STEP_NONE;

        DBG_SWDIAG(SAM, RMAP_EN, 0);
        DBG_SWDIAG(SAM, LMAP_EN, 0);
    }

    acl_par->sam_info->instant = instant;
    acl_par->sam_info->local_map_pending = true;

    // Set the active local SAM index
    acl_par->sam_info->loc_idx = sam_idx;

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(SAM, LMAP_SET, 0);
}

#endif //RW_BT_MWS_COEX

void ld_acl_remote_sam_map_set(uint8_t link_id, uint8_t *sam_ptr, uint16_t t_sam, uint32_t instant, uint8_t d_sam, bool flags)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_REMOTE_SAM_MAP_SET_2_FUNC_BIT, 6, link_id, sam_ptr, t_sam, instant, d_sam, flags);

    DBG_SWDIAG(SAM, RMAP_SET, 1);

    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    ASSERT_ERR(acl_par != NULL);

    GLOBAL_INT_DISABLE();

    if (NULL == acl_par->sam_info)
    {
        // Allocate memory buffer for storing SAM data
        acl_par->sam_info = ke_malloc_system(sizeof(struct ld_acl_sam_info_tag), KE_MEM_KE_MSG);
        acl_par->sam_info->map_length = 0;
#if RW_BT_MWS_COEX
        acl_par->sam_info->loc_idx = SAM_DISABLED;
        acl_par->sam_info->local_map_pending = false;
#endif //RW_BT_MWS_COEX
        acl_par->sam_info->sam_change = ACL_SAM_STEP_NONE;

        DBG_SWDIAG(SAM, RMAP_EN, 0);
        DBG_SWDIAG(SAM, LMAP_EN, 0);
    }

    if (sam_ptr)
    {
        uint32_t master_clk = (SLAVE_ROLE == acl_par->role) ? CLK_ADD_2(ld_read_clock(),
                              ld_acl_clock_offset_get(link_id).hs) : ld_read_clock();

        // Find the first anchor point after master_clk
        uint32_t anchor_pt = (master_clk >> 1) + d_sam - CO_MOD(((master_clk >> 1) ^ (flags << 25)), t_sam);

        if (CLK_DIFF(master_clk, (anchor_pt << 1)) < 0)
        {
            anchor_pt += t_sam;
        }

        // Point to the peer SAM Map
        acl_par->sam_info->map_length_pending = t_sam;
        acl_par->sam_info->anchor_pt_pending = anchor_pt;

        // MAPPING
        // LMP SAM_Submap definition -> HW definition of SAM Map dibit:
        // 0: The slot is not available for either tx or rx => Stop both Tx and Rx = 0
        // 1: The slot is available for tx but not rx on PEER => Do not stop Rx / Stop only Tx = 2
        // 2: The slot is available for rx but not tx on PEER => Stop only Rx / Do not stop Tx = 1
        // 3: The slot is available for tx and rx => Do not stop Tx / Do not stop Rx = 3

        for (int i = 0; i < RW_PEER_SAM_MAP_MAX_LEN; i++)
        {
            // Convert mapping by simply swapping all odd and even bits
            uint8_t dibits = *sam_ptr++;
            acl_par->sam_info->peer_map[i] = ((dibits & 0xAA) >> 1) | ((dibits & 0x55) << 1);
        }
    }
    else
    {
        // SAM length 0 indicates SAM disabled
        acl_par->sam_info->map_length_pending = 0;
    }

    acl_par->sam_info->instant = instant;
    acl_par->sam_info->peer_map_pending = true;

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(SAM, RMAP_SET, 0);
}

void ld_acl_remote_sam_map_refresh(uint8_t link_id, uint8_t *sam_ptr, uint8_t t_sam_sm)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_REMOTE_SAM_MAP_REFRESH_2_FUNC_BIT, link_id, sam_ptr, t_sam_sm);

    DBG_SWDIAG(SAM, RMAP_REFRESH, 1);

    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

    GLOBAL_INT_DISABLE();

    ASSERT_ERR((acl_par != NULL) && (acl_par->sam_info != NULL));

    // MAPPING
    // LMP SAM_Submap definition -> HW definition of SAM Map dibit:
    // 0: The slot is not available for either tx or rx => Stop both Tx and Rx = 0
    // 1: The slot is available for tx but not rx on PEER => Do not stop Rx / Stop only Tx = 2
    // 2: The slot is available for rx but not tx on PEER => Stop only Rx / Do not stop Tx = 1
    // 3: The slot is available for tx and rx => Do not stop Tx / Do not stop Rx = 3

    for (int i = 0; i < RW_PEER_SAM_MAP_MAX_LEN; i++)
    {
        // Convert mapping by simply swapping all odd and even bits
        uint8_t dibits = *sam_ptr++;
        acl_par->sam_info->peer_map[i] = ((dibits & 0xAA) >> 1) | ((dibits & 0x55) << 1);
    }

    // update at next t_sam_sm (or immediate if t_sam_sm set to 0)
    acl_par->sam_info->instant = acl_par->sam_info->anchor_pt + t_sam_sm;
    acl_par->sam_info->peer_map_pending = true;

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(SAM, RMAP_REFRESH, 0);
}

uint32_t ld_acl_sam_ts_get(uint8_t link_id)
{
    // Point to parameters
    struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
    uint32_t sam_ts = 0;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ACL_2_PATCH_TYPE, LD_ACL_SAM_TS_GET_2_FUNC_BIT, uint32_t, link_id);

    GLOBAL_INT_DISABLE();

    ASSERT_ERR(acl_par != NULL);
    ASSERT_ERR(acl_par->sam_info != NULL);

    // If a new SAM map is pending, use this one instead
    if (acl_par->sam_info->peer_map_pending)
    {
        sam_ts = CLK_ADD_2(acl_par->sam_info->anchor_pt_pending, acl_par->sam_info->map_length_pending);
    }
    else
    {
        sam_ts = CLK_ADD_2(acl_par->sam_info->anchor_pt, acl_par->sam_info->map_length);
    }

    GLOBAL_INT_RESTORE();

    return sam_ts;
}

#if (EAVESDROPPING_SUPPORT)
enum co_error ld_acl_get_aes_ccm_payload_counter(uint8_t con_type, uint8_t link_id, uint8_t *dst_ptr)
{
    uint16_t   rx_ccm_pld_cntr0, rx_ccm_pld_cntr1, rx_ccm_pld_cntr2;
    uint32_t   sco_day_cntr;

    switch (con_type)
    {
    case ED_CON_TYPE_ACL:
        rx_ccm_pld_cntr0 = em_bt_rxccmpldcnt0_get(link_id);
        rx_ccm_pld_cntr1 = em_bt_rxccmpldcnt1_get(link_id);
        rx_ccm_pld_cntr2 = em_bt_rxccmpldcnt2_get(link_id);

        if (ld_acl_env[link_id] != NULL)
        {
            // Point to parameters
            struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

            if (acl_par->state != ACL_EVT_WAIT)
            {
                bool finished = false;
                uint8_t att = 0;
                uint8_t max_att = 4;
                uint16_t cntr0, cntr1, cntr2;

                while ((!finished) && (att < max_att))
                {
                    // Read the counter again
                    cntr0 = em_bt_rxccmpldcnt0_get(link_id);
                    cntr1 = em_bt_rxccmpldcnt1_get(link_id);
                    cntr2 = em_bt_rxccmpldcnt2_get(link_id);

                    // If the counters match then stop
                    if ((cntr0 == rx_ccm_pld_cntr0) && (cntr1 == rx_ccm_pld_cntr1) && (cntr2 == rx_ccm_pld_cntr2))
                    {
                        finished = true;
                    }
                    else
                    {
                        rx_ccm_pld_cntr0 = cntr0;
                        rx_ccm_pld_cntr1 = cntr1;
                        rx_ccm_pld_cntr2 = cntr2;
                    }

                    att++;
                    ASSERT_ERR(att < max_att);
                }
            }
        }

        dst_ptr[0] = (uint8_t)((rx_ccm_pld_cntr2 >> 0) & 0x000F); // ED_LOBYTE(rx_ccm_pld_cntr2) & 0x0F;
        dst_ptr[1] = (uint8_t)((rx_ccm_pld_cntr1 >> 8) & 0x00FF); // ED_HIBYTE(rx_ccm_pld_cntr1);
        dst_ptr[2] = (uint8_t)((rx_ccm_pld_cntr1 >> 0) & 0x00FF); // ED_LOBYTE(rx_ccm_pld_cntr1);
        dst_ptr[3] = (uint8_t)((rx_ccm_pld_cntr0 >> 8) & 0x00FF); // ED_HIBYTE(rx_ccm_pld_cntr0);
        dst_ptr[4] = (uint8_t)((rx_ccm_pld_cntr0 >> 0) & 0x00FF); // ED_LOBYTE(rx_ccm_pld_cntr0);
        break;
    case ED_CON_TYPE_ESCO:
        sco_day_cntr = bt_e_scodaycnt_get(link_id);

        memcpy(dst_ptr, &sco_day_cntr, sizeof(uint32_t));
        break;
    default:
        break;
    }

    return (CO_ERROR_NO_ERROR);
}

enum co_error ld_acl_update_aes_ccm_payload_cntr(uint8_t con_type, uint8_t link_id, const uint8_t *src_ptr)
{
    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        if (ld_acl_env[link_id]->con_type == ED_CON_TYPE_ACL)
        {
            uint16_t new_cntr0 = 256 * src_ptr[3] + src_ptr[4];
            uint16_t new_cntr1 = 256 * src_ptr[1] + src_ptr[2];
            uint16_t new_cntr2 = src_ptr[0] & 0x0F;

            uint16_t curr_cntr0 = em_bt_rxccmpldcnt0_get(link_id);
            uint16_t curr_cntr1 = em_bt_rxccmpldcnt1_get(link_id);
            uint16_t curr_cntr2 = em_bt_rxccmpldcnt2_get(link_id);

            // If the new counter is greater set the aes_ccm_cnt_correction flag
            if (((new_cntr2 > curr_cntr2) && (curr_cntr2 < 0x000F))
                    || ((new_cntr2 == curr_cntr2) && (new_cntr1 > curr_cntr1))
                    || ((new_cntr2 == curr_cntr2) && (new_cntr1 == curr_cntr1) && (new_cntr0 > curr_cntr0))
                    // If the counter has wrapped around
                    || ((new_cntr2 == 0) && (new_cntr1 == 0) && (curr_cntr2 == 0x000F) && (curr_cntr1 == 0xFFFF)))
            {
                acl_par->aes_ccm_cnt_correction_upd = true;
            }
        }

        if ((ld_acl_env[link_id]->con_type == ED_CON_TYPE_ESCO) || acl_par->aes_ccm_cnt_correction_upd)
        {
            if (acl_par->state == ACL_EVT_WAIT)
            {
                // Apply the new value
                ld_acl_apply_aes_ccm_cnt(link_id, (uint8_t *) src_ptr);

                acl_par->mic_invalid_event_block = false;

                if (acl_par->aes_ccm_cnt_correction_upd)
                {
                    acl_par->aes_ccm_cnt_correction_upd = false;
                    acl_par->aes_ccm_cnt_correction = true;
                }
            }
            else
            {
                // Store the value to apply
                memcpy(&acl_par->aes_ccm_cnt[0], src_ptr, 5);

                // Save the connection type
                acl_par->con_type = con_type;

                // Indicate to update the counter on next frame interrupt
                acl_par->aes_ccm_cnt_upd = true;
            }
        }
    }

    GLOBAL_INT_RESTORE();

    return (CO_ERROR_NO_ERROR);
}

void ld_acl_micerr_isr(void)
{
    // Check if counter has not been already reported
    if (!GETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_BLOCK))
    {
        uint8_t link_id = GETF(ld_aes_ccm_cnt, ED_AES_CCM_CNT_LINK_ID);
        uint8_t con_type = GETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_CON_TYPE);

        if (ld_acl_env[link_id] != NULL)
        {
            // Point to parameters
            struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

            if (acl_par->gen_aes_ccm_counter_event)
            {
                // Read AES CCM counter and send the event to the ED task.
                struct ed_aes_ccm_cntr_ind *msg = KE_MSG_ALLOC(ED_AES_CCM_CNTR_IND, TASK_ED, TASK_NONE, ed_aes_ccm_cntr_ind);
                ld_acl_get_aes_ccm_payload_counter(con_type, link_id, &msg->cntr[0]);
                msg->link_id = link_id;
                msg->con_type = con_type;
                msg->mic_ok = false;
                ke_msg_send(msg);
            }
        }

        // Block following reports
        SETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_BLOCK, 1);
    }
}

void ld_acl_timing_info_get(uint8_t link_id, uint32_t *clk_off, uint16_t *bit_off)
{
    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        // Get bit offset
        *bit_off = acl_par->last_sync_bit_off;
        *clk_off = acl_par->last_sync_clk_off;
    }

    GLOBAL_INT_RESTORE();
}

uint8_t ld_acl_lt_addr_get(uint8_t link_id)
{
    uint8_t lt_addr = 0;
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        lt_addr = em_bt_linkcntl_aclltaddr_getf(cs_idx);
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, 0);
    }

    GLOBAL_INT_RESTORE();

    return lt_addr;
}

enum co_error ld_acl_set_cust_evt_dur(uint8_t link_id, uint8_t evt_dur)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // check evt_dur is within supported range
        if ((evt_dur >= 2) && (evt_dur <= (0xFFFF / SLOT_SIZE)))
        {
            // Set cust_evt_duration (in half-us) to evt_dur (in slots) to override using default value
            acl_par->cust_evt_dur_min = 2 * HALF_SLOT_SIZE * evt_dur;

            status = CO_ERROR_NO_ERROR;
        }
    }

    GLOBAL_INT_RESTORE();

    return status;
}

enum co_error ld_acl_clr_cust_evt_dur(uint8_t link_id)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        /// clear duration to disable custom setting, and revert to using default value
        acl_par->cust_evt_dur_min = 0;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}

enum co_error ld_acl_set_lrd(uint8_t link_id, bool lrd_en, bool control_tx_prog, bool ignore_seqn_errors, uint16_t l2cap_cid, bool null_flt)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(link_id);

        // LRD enabled/disabled
        acl_par->lrd_en = lrd_en;

        if (acl_par->lrd_en)
        {
            acl_par->control_tx_prog = control_tx_prog;
            acl_par->ignore_seqn_errors = ignore_seqn_errors;
            acl_par->l2cap_cid = l2cap_cid;
            acl_par->null_flt = null_flt;
        }
        else
        {
            acl_par->control_tx_prog = false;
            acl_par->ignore_seqn_errors = false;
            acl_par->l2cap_cid = 0;
            acl_par->null_flt = true;
        }

        // Set eavesdropping interface enable control
        bt_binackinfo_edifen_setf(acl_par->lrd_en);

        // Set control structure
        em_bt_linkcntl_aclbinackena_setf(cs_idx, (acl_par->lrd_en && !acl_par->ignore_seqn_errors));
        em_bt_linkcntl_duplfltena_setf(cs_idx, (acl_par->lrd_en && acl_par->ignore_seqn_errors));
        em_bt_cidcntl_cid_setf(cs_idx, acl_par->l2cap_cid);
        em_bt_txrxcntl_nullflt_setf(cs_idx, acl_par->null_flt);

        // Remember which link has Bin ack enabled
        if (acl_par->lrd_en)
        {
            SETF(ld_aes_ccm_cnt, ED_AES_CCM_CNT_LINK_ID, link_id);
            SETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_CON_TYPE, ED_CON_TYPE_ACL);
            SETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_BLOCK, 0);
        }
        else
        {
            SETF(ld_aes_ccm_cnt, ED_AES_CCM_CNT_LINK_ID, MAX_NB_ACTIVE_ACL);
            SETB(ld_aes_ccm_cnt, ED_AES_CCM_CNT_BLOCK, 1);
        }

        status = CO_ERROR_NO_ERROR;

        if (acl_par->control_tx_prog && (acl_par->txdesc_cnt == 0) && co_list_is_empty(&acl_par->queue_acl_tx) && co_list_is_empty(&acl_par->queue_lmp_tx))
        {
            // Notify ED task that no transmission is pending
            struct ed_acl_tx_pending_ind *msg = KE_MSG_ALLOC(ED_ACL_TX_PENDING_IND, TASK_ED, TASK_NONE, ed_acl_tx_pending_ind);

            msg->link_id = link_id;
            msg->tx_pending = false;
            msg->tx_seqn = em_bt_acltxstat_lasttxseqn_getf(cs_idx);

            ke_msg_send(msg);

            // Disable Tx programming
            acl_par->tx_prog_en = false;
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

enum co_error ld_acl_allow_tx_prog(uint8_t link_id)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        /// Tx programming has been enabled
        acl_par->tx_prog_en = true;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

enum co_error ld_acl_corr_payl_en(uint8_t link_id, bool en)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        acl_par->corr_acl_payl_en = en;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

enum co_error ld_acl_rx_stats_reporting(uint8_t link_id, bool en, uint8_t con_type, uint8_t intv)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Save parameters
        acl_par->rx_stats_rep_en = en;
        acl_par->con_type = con_type;
        acl_par->rx_stats_rep_intv = intv;
        acl_par->rx_tot = 0;
        acl_par->rx_sync = 0;

        if (en)
        {
            uint32_t clock = ld_read_clock();
            uint32_t master_clock = CLK_ADD_2(clock, acl_par->clk_off);
            uint32_t instant;
            int32_t diff;

            // Make sure that the master clock considered is even (corresponds to a BT slot)
            if (master_clock & 0x1)
            {
                master_clock &= ~0x1;
                clock = CLK_SUB(clock, 1);
            }

            // Compute the first instant
            diff = intv - CO_MOD((master_clock >> 1), intv);

            instant = CLK_ADD_2(clock, 2 * diff);
            acl_par->stats_ts = CLK_ADD_2(master_clock, 2 * diff);
            ASSERT_ERR(CO_MOD(acl_par->stats_ts, 2 * intv) == 0);

            // Program Rx statistics indication instant via an alarm
            acl_par->stats_alarm.time.hs = instant;
            acl_par->stats_alarm.time.hus = acl_par->bit_off;
            acl_par->stats_alarm.cb_alarm = &ld_acl_rx_stats_ind_cbk;
            sch_alarm_set(&acl_par->stats_alarm);
        }
        else
        {
            // Clear the statistics alarm
            sch_alarm_clear(&acl_par->stats_alarm);
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

enum co_error ld_acl_handle_mic_invalid(uint8_t link_id, bool gen_mic_invalid_event, bool gen_aes_ccm_counter_event, bool increase_counter, bool tolerate_invalid_mic, uint8_t n_refr, uint8_t n_disc)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (ld_acl_env[link_id] != NULL)
    {
        // Point to parameters
        struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];

        // Save parameters
        acl_par->gen_mic_invalid_event = gen_mic_invalid_event;
        acl_par->gen_aes_ccm_counter_event = gen_aes_ccm_counter_event;
        acl_par->increase_counter = increase_counter;
        acl_par->tolerate_invalid_mic = tolerate_invalid_mic;
        acl_par->n_refr = n_refr;
        acl_par->n_disc = n_disc;

        acl_par->mic_invalid_event_block = false;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

enum co_error ld_acl_sample_esco_tx_seqn(uint8_t link_id)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    // Look for SCO connections on the link
    for (int sco_link_id = (MAX_NB_SYNC - 1) ; sco_link_id >= 0 ; sco_link_id--)
    {
        if (ld_sco_env[sco_link_id] != NULL)
        {
            // Point to parameters
            struct ld_acl_env_tag *acl_par = ld_acl_env[link_id];
            struct ld_sco_env_tag *sco_par = ld_sco_env[sco_link_id];

            if (sco_par->link_id == link_id)
            {
                if ((acl_par->role == SLAVE_ROLE) && (sco_par->update == SCO_NO_UPDATE) && (!sco_par->end) &&
                        (sco_par->sync_type == ESCO_TYPE) && (!sco_par->sample_tx_seqn))
                {
                    // Mark to sample the eSCO Tx SEQN
                    sco_par->sample_tx_seqn = true;
                    status = CO_ERROR_NO_ERROR;
                }
                break;
            }
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#endif // EAVESDROPPING_SUPPORT
///@} LDACL
