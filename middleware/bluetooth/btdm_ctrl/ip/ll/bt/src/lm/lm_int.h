/**
 ****************************************************************************************
 *
 * @file lm.h
 *
 * @brief Link manager internal definition
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef LM_INT_H_
#define LM_INT_H_

/**
 ****************************************************************************************
 * @addtogroup LM Link Manager
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>          // standard integer
#include <stdbool.h>         // standard boolean
#include "lm.h"              // Link manager API

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of instances of the LM task
#define LM_IDX_MAX                       1

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// States of a link
enum LM_LINK_STATE
{
    LM_FREE,
    LM_PAGE,
    LM_PAGE_STOPPING,
    LM_PAGE_SCAN,
    LM_PAGE_SCAN_STOPPING,
    LM_CONNECTED,
    LM_SWITCH,
};

/// Possible states of the LM task
enum LM_STATE
{
    LM_IDLE,
};

/// Inquiry states
enum LM_INQ_STATE
{
    LM_INQ_OFF,
    LM_INQ_NORMAL,
    LM_INQ_PERIODIC,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for HCI configuration parameters (non-connection oriented)
struct lm_hci_cfg_params
{
    /**
     * Scan Enable
     *
     * 0x00 No Scans enabled.
     * 0x01 Inquiry Scan enabled. Page Scan always disabled.
     * 0x02 Inquiry Scan disabled. Page Scan enabled.
     * 0x03 Inquiry Scan enabled. Page Scan enabled.
     * 0x04-0xFF Reserved [0x04 limited page scan removed]
     */
    uint8_t scan_en;

    /**
     * Inquiry Scan Interval
     *
     * Size: 2 Octets
     * Range: 0x0012 to 0x1000; only even values are valid
     * Default: 0x1000
     * Mandatory Range: 0x0012 to 0x1000
     * Time = N * 0.625 msec
     * Time Range: 11.25 to 2560 msec
     * Time Default: 2.56 sec
     */
    uint16_t inq_scan_intv;

    /**
     * Inquiry Scan Window
     *
     * Size: 2 Octets
     * Range: 0x0011 to 0x1000
     * Default: 0x0012
     * Mandatory Range: 0x0011 to Inquiry Scan Interval
     * Time = N * 0.625 msec
     * Time Range: 10.625 msec to 2560 msec
     * Time Default: 11.25 msec
     */
    uint16_t inq_scan_win;

    /**
     * Inquiry Scan Type
     *
     * 0x00 Mandatory: Standard Scan (default)
     * 0x01 Optional: Interlaced Scan
     * 0x02-0xFF Reserved
     */
    uint8_t inq_scan_type;

    /**
     * Inquiry Mode
     *
     * 0x00 Standard Inquiry Result event format
     * 0x01 Inquiry Result format with RSSI
     * 0x02 Inquiry Result with RSSI format or Extended Inquiry Result format
     * 0x03-0xFF Reserved
     */
    uint8_t inq_mode;

    /**
     * Page Timeout
     *
     * Size: 2 Octets
     * Range: 0x0001 to 0xFFFF
     * Default: 0x2000
     * Mandatory Range: 0x0016 to 0xFFFF
     * Time = N * 0.625 msec
     * Time Range: 0.625 msec to 40.9 sec
     * Time Default: 5.12 sec
     */
    uint16_t page_to;

    /**
     * Page Scan Interval
     *
     * Size: 2 Octets
     * Range: 0x0012 to 0x1000; only even values are valid
     * Default: 0x0800
     * Mandatory Range: 0x0012 to 0x1000
     * Time = N * 0.625 msec
     * Time Range: 11.25 msec to 2560 msec
     * Time Default: 1.28 sec
     */
    uint16_t page_scan_intv;

    /**
     * Page Scan Window
     *
     * Size: 2 Octets
     * Range: 0x0011 to 0x1000
     * Default: 0x0012
     * Mandatory Range: 0x0011 to Page Scan Interval
     * Time = N * 0.625 msec
     * Time Range: 10.625 msec to Page Scan Interval
     * Time Default: 11.25 msec
     */
    uint16_t page_scan_win;

    /**
     * Page scan repetition mode (depends on page scan window and interval)
     * 0x00 R0
     * 0x01 R1
     * 0x02 R2
     * 0x03 – 0xFF Reserved.
     */
    uint8_t page_scan_rep_mode;

    /**
     * Page Scan Type
     *
     * 0x00 Mandatory: Standard Scan (default)
     * 0x01 Optional: Interlaced Scan
     * 0x02-0xFF Reserved
     */
    uint8_t page_scan_type;

    /**
     * Voice Setting
     *
     * 00XXXXXXXX Input Coding: Linear
     * 01XXXXXXXX Input Coding: u-law Input Coding
     * 10XXXXXXXX Input Coding: A-law Input Coding
     * 11XXXXXXXX Reserved for future use
     * XX00XXXXXX Input Data Format: 1's complement
     * XX01XXXXXX Input Data Format: 2's complement
     * XX10XXXXXX Input Data Format: Sign-Magnitude
     * XX11XXXXXX Input Data Format: Unsigned
     * XXXX0XXXXX Input Sample Size: 8-bit (only for linear PCM)
     * XXXX1XXXXX Input Sample Size: 16-bit (only for linear PCM)
     * XXXXXnnnXX Linear_PCM_Bit_Pos: # bit positions that MSB of sample is away from starting at MSB (only for Linear PCM).
     * XXXXXXXX00 Air Coding Format: CVSD
     * XXXXXXXX01 Air Coding Format: u-law
     * XXXXXXXX10 Air Coding Format: A-law
     * XXXXXXXX11 Air Coding Format: Transparent Data
     */
    uint16_t voice_stg;

    /**
     * Pin Type
     *
     * 0x00 Variable PIN.
     * 0x01 Fixed PIN.
     */
    uint8_t pin_type;

    /**
     * Authentication Enable
     *
     * 0x00 Authentication not required.
     * 0x01 Authentication required for all connections.
     */
    uint8_t auth_en;

    /**
     * Extended Page Timeout
     *
     * Range: 0x0000 to 0xFFFF Default: 0x0000
     * Mandatory Range: 0x0000 to 0xFFFF Time = N * 0.625 msec
     * Time Range: 0 to 40.9 sec
     * Time Default: 0 sec
     */
    uint16_t ext_page_to;

    /**
     * Extended Inquiry Length
     *
     * Range: 0x0000 to 0xFFFF Default: 0x0000
     * Mandatory Range: 0x0000 to 0xFFFF Time = N * 0.625 msec
     * Time Range: 0 to 40.9 sec
     * Time Default: 0 sec
     */
    uint16_t ext_inq_len;

    /**
     * Inquiry TX power level
     *
     * Range: –70 <= N <= 20
     * Units: dBm
     */
    int8_t inq_tx_pwr_lvl;

    /**
     * IAC_LAP
     *
     * LAPs used to create the IAC which is currently in use by the local BR/EDR
     * Controller to simultaneously listen for during an Inquiry Scan.
     *
     * Range: 0x9E8B00-0x9E8B3F
     *
     * Note: only 1 IAC LAP supported (can scan only 1 simultaneously due to correlator limitations)
     */
    struct lap iac_lap;

    /**
     * Link Policy Settings (Default value used for new connections)
     *
     * 0x0000 Disable All LM Modes Default
     * 0x0001 Enable Role switch.
     * 0x0002 Enable Hold Mode.
     * 0x0004 Enable Sniff Mode.
     * 0x0008 Enable Park State.
     * 0x0010–0x8000 Reserved for future use.
     */
    uint16_t link_pol_stg;

    /**
     * Simple Pairing Mode
     *
     * 0x00 Simple Pairing not set (default)
     * 0x01 Simple pairing enabled
     * 0x02 - 0xFF Reserved for future use
     */
    uint8_t sp_mode;

    /**
     * Secure Connections Host Support
     * 0x00 Secure_Conections_Host_Support is ‘disabled’. Host does not support Secure Connections (default)
     * 0x01 Secure_Connections_Host_Support is ‘enabled’. Host supports Secure Connections
     * 0x02 – 0xFF Reserved
     */
    uint8_t sec_con_host_supp;

    /**
     * LE SUPPORTED HOST
     *
     * 0x00 LE Supported (Host) disabled (default)
     * 0x01 LE Supported (Host) enabled
     * 0x02 – 0xFF Reserved
     */
    uint8_t le_supported_host;

    /**
     * SIMULTANEOUS LE HOST
     *
     * 0x00 Simultaneous LE and BR/EDR to Same Device Capable (Host) disabled (default). This value shall be ignored.
     * 0x01 – 0xFF Reserved
     */
    uint8_t simultaneous_le_host;

    /**
     * Loopback mode
     *
     * 0x00 No Loopback mode enabled (default).
     * 0x01 Enable Local Loopback.
     * 0x02 Enable Remote Loopback. (Not Supported by AMPs)
     * 0x03-0xFF Reserved for future use.
     */
    uint8_t loopback_mode;

    /**
     * Simple pairing debug mode
     *
     * 0x00 Simple Pairing debug mode disabled (default)
     * 0x01 Simple pairing debug mode enabled
     * 0x02 - 0xFF Reserved for future use
     */
    uint8_t sp_debug_mode;

    /**
     * Erroneous Data reporting
     *
     * 0x00 Erroneous Data reporting disabled (default).
     * 0x01 Erroneous data reporting enabled.
     * 0x02 - 0xFF Reserved.
     */
    uint8_t err_data_rep;

#if (EAVESDROPPING_SUPPORT)
    /**
     * Host max slot value (1, 3 or 5)
     */
    uint8_t host_max_slot;
#endif // EAVESDROPPING_SUPPORT

#if (BT_53)
    /**
     * Minimum encryption key size
     */
    uint8_t min_enc_key_size;
#if (RW_DEBUG)
    /**
     * Maximum encryption key size
     */
    uint8_t max_enc_key_size;
#endif  // (RW_DEBUG)
#endif // (BT_53)

    /**
     * Enable device under test mode
     */
    bool dut_mode_en;

    /**
     * Synchronous flow control enable
     */
    bool sync_flow_ctrl_en;

#if (EAVESDROPPING_SUPPORT)
    /**
     * SCO link request delay
     */
    uint16_t sco_link_request_delay;

    /**
     * Clock wrap notification
     */
    bool clock_wrap_notification_enabled;
    struct bd_addr clock_wrap_notification_bd_addr;

    /**
     * Notification of sniff events
     */
    bool notify_sniff_events;
#endif // EAVESDROPPING_SUPPORT
};

/// AFH parameters structure
struct lm_afh_tag
{
    /// Channel classification from Host
    struct bt_ch_map host_ch_class;

#if (EAVESDROPPING_SUPPORT)
    /// Channel classification from eavesdropper (given from ED to EDE)
    struct bt_ch_map ed_ch_class;
#endif // EAVESDROPPING_SUPPORT

    /// AFH state
    uint8_t active;
};

struct lm_sam_pattern
{
    ///Tsam-sm (slots)
    uint8_t t_sam_sm;
    ///Nsam-sm
    uint8_t n_sam_sm;
    ///SAM Submaps
    struct sam_submaps submaps;
    ///Num slots available for tx
    uint16_t n_tx_slots;
    ///Num slots available for rx
    uint16_t n_rx_slots;
    ///Nsam_sm defined by submap
    uint8_t n_ex_sm;
};

/// Slot availability map parameters structure
struct lm_sam_info_tag
{
    /// type0 submap
    uint8_t type0submap[SAM_TYPE0_SUBMAP_LEN];

    /// defined patterns
    struct lm_sam_pattern pattern[SAM_INDEX_MAX];

    /// submap frame length (slots)
    uint8_t frame_len;

    /// frame offset (us)
    int16_t frame_offset;

    /// active index
    uint8_t active_index;
};


/// Inquiry procedure parameters structure
struct lm_inq_tag
{
    /// Flag indicating periodic inquiry mode
    bool periodic;
};

/// State for all connections
struct lm_con_info
{
    /// Peer BD Address
    struct bd_addr bd_addr;

    /// State
    uint8_t state;

    /// Role (0x00: MASTER / 0x01: SLAVE)
    uint8_t role;

    /// LT Address (Valid only for master ACL links)
    uint8_t lt_addr;
};

/// Inquiry duplicate filtering information structure
struct inq_dup_filt_info
{
    /// Inquiry result filtering list
    struct bd_addr dev[INQ_FILT_LEN];

    /// Current position in the table for placing the next BD address
    uint8_t curr_pos;
};

/// LM environment structure
struct lm_env_tag
{
    /// Pointer to HCI_Create_Con_Cmd associated to the ongoing Page procedure
    const struct hci_create_con_cmd *create_con_cmd;

    /// Pointer to HCI_Remote_Name_Req associated to the ongoing Page procedure
    const struct hci_rem_name_req_cmd *rem_name_req_cmd;

    /// Pointer to HCI_Trunc_Page_Cmd associated to the ongoing Page procedure
    const struct hci_trunc_page_cmd *trunc_page_cmd;

    /// HCI configuration parameters (non-connection oriented)
    struct lm_hci_cfg_params hci;

    /// State for all connections
    struct lm_con_info con_info[MAX_NB_ACTIVE_ACL];

    /// Pointer for generated OOB data information - valid for only one pairing
    lm_oob_data_t *p_oob_data;

    /// LT Address bitmap
    uint8_t lt_addr_bitmap;

    /// Local name
    char *local_name;

    /// Inquiry state (Off, Normal, Periodic)
    uint8_t inq_state;

    /// Number of stored link keys
    uint8_t nb_stored_link_keys;

    /// AFH parameters
    struct lm_afh_tag afh;

#if RW_BT_MWS_COEX
    /// MWS External frame configuration set
    bool local_ext_fr_config;
#endif //RW_BT_MWS_COEX

    /// SAM parameters (local)
    struct lm_sam_info_tag sam_info;

    /// Inquiry result filtering list
    struct inq_dup_filt_info inq_filt;

#if (MAX_NB_SYNC > 0)
    /// SCO move enable
    bool sco_move_en;
#endif //(MAX_NB_SYNC > 0)
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 *****************************************************************************************
 */

/// LM task descriptor
extern const struct ke_task_desc TASK_DESC_LM;

/// LM task message handlers table
extern const struct ke_state_handler lm_default_handler;

/// LM task state
extern ke_state_t lm_state[LM_IDX_MAX];

/// LM environment variable
extern struct lm_env_tag lm_env;

/// LM local supported features
extern uint8_t const lm_local_supp_feats[FEATURE_PAGE_MAX][FEATS_LEN];
/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is used to generate OOB data
 *
 * @param[in] gen_oob_256     True if P256 OOB data must be generated, only P192 otherwise
 ****************************************************************************************
 */
void lm_gen_local_oob_data(bool gen_oob_256);

/// @} LM

#endif // LM_INT_H_
