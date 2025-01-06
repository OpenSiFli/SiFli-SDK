/**
 ****************************************************************************************
 *
 * @file llc_int.h
 *
 * @brief Link layer controller internal definition
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

#ifndef LLC_INT_H_
#define LLC_INT_H_

/**
 ****************************************************************************************
 * @addtogroup LLC Link layer Controller
 * @{
 ****************************************************************************************
 */
#include "rwip_task.h" // Task definitions


/*
 * INSTANCES
 ****************************************************************************************
 */
/// Maximum number of instances of the LLC task
#define LLC_IDX_MAX  BLE_ACTIVITY_MAX


/*
 * DEFINE
 ****************************************************************************************
 */

/// The master should allow a minimum of 6 connection events that the slave will be listening for before the instant occurs
#define LLC_PROC_SWITCH_INSTANT_DELAY   (8)

/// Number of connection events for the connection event count used in negotiating CIS anchor point
#define LLC_PROC_CIS_NEGO_INSTANT_DELAY (2)

/*
 * MESSAGES
 ****************************************************************************************
 */

/// Message API of the LLC task
/*@TRACE
 * llc_encrypt_ind = aes_func_res_ind */
enum llc_msg_id
{
    LLC_MSG_ID_MSG_FIRST = TASK_FIRST_MSG(TASK_ID_LLC),

    /*
     * ************** Msg LLC->LLC****************
     */
    /// Local procedure LLCP Time out
    LLC_LOC_LLCP_RSP_TO = LLC_MSG_ID_MSG_FIRST,
    /// Remote procedure LLCP Time out
    LLC_REM_LLCP_RSP_TO,
    /// Authenticated Payload timeout event
    LLC_AUTH_PAYL_REAL_TO,
    /// Authenticated Payload Near timeout event
    LLC_AUTH_PAYL_NEARLY_TO,
    /// Link is stopped, clean-up environment
    LLC_STOPPED_IND,
    /// Triggered when encryption has been performed
    LLC_ENCRYPT_IND,
#if (BLE_CON_CTE_REQ)
    /// Constant Tone Extension request timeout
    LLC_CTE_REQ_TO,
#endif // (BLE_CON_CTE_REQ)
#if (BLE_CIS) && (BLE_PERIPHERAL)
    /// CIS connection accept timeout
    LLC_CIS_ACCEPT_TO,
#endif // (BLE_CIS) && (BLE_PERIPHERAL)
#if (BT_53)
    /// Channel classification reporting timeout
    LLC_CH_CLASS_TO,
#endif // (BT_53)

    /*
     * ************ Local Procedures *************
     */
    /// Version exchange procedure
    LLC_OP_VER_EXCH_IND,
    /// Features exchange procedure
    LLC_OP_FEATS_EXCH_IND,
    /// Disconnection procedure
    LLC_OP_DISCONNECT_IND,
    /// Encryption procedure
    LLC_OP_ENCRYPT_IND,
    /// Data Length update procedure
    LLC_OP_DL_UPD_IND,
    /// Connection update procedure
    LLC_OP_CON_UPD_IND,
    /// LE Ping exchange
    LLC_OP_LE_PING_IND,
    /// Channel map update procedure
    LLC_OP_CH_MAP_UPD_IND,
    /// PHY update procedure
    LLC_OP_PHY_UPD_IND,
#if (BLE_CON_CTE_REQ | BLE_CON_CTE_RSP)
    /// Constant Tone Extension procedure
    LLC_OP_CTE_IND,
#endif // (BLE_CON_CTE_REQ | BLE_CON_CTE_RSP)
    /// Clock accuracy exchange procedure
    LLC_OP_CLK_ACC_IND,
#if (BLE_PAST)
    /// Periodic Advertising Sync Transfer procedure
    LLC_OP_PAST_IND,
#endif // (BLE_PAST)
#if (BLE_CIS)
    /// CIS create procedure
    LLC_OP_CIS_CREATE_IND,
    /// CIS stop procedure
    LLC_OP_CIS_STOP_IND,
#endif // (BLE_CIS)
#if BLE_PWR_CTRL
    /// Power control procedure
    LLC_OP_PWR_CTRL_IND,
#endif // BLE_PWR_CTRL
#if (BT_53)
    /// Subrate procedure
    LLC_OP_SUBRATE_IND,
    /// Channel classification enable procedure
    LLC_OP_LE_CH_CLASS_EN_IND,
    /// Channel classification reporting procedure
    LLC_OP_CH_CLASS_REP_IND,
#endif // (BT_53)
};




/*
 * STATES
 ****************************************************************************************
 */
/// Procedures type definition
enum llc_proc_type
{
    /** Local initiated procedure
     *
     * @startuml llc_local_proc_init.png
     * participant HCI
     * participant LLC
     * alt Procedure initiated by host
     *     HCI --> LLC : HCI_XXX_CMD
     *     LLC -> LLC: hci_xxx_cmd_handler()
     *     activate LLC
     *         note over LLC #aqua: Allocate and fill procedure message
     *         LLC --> LLC : LLC_OP_XXX_IND
     *         note right: send message to local task instance \nto flow control procedure
     *         LLC --> HCI : HCI_CMD_STAT_EVENT
     *     deactivate LLC
     * else Procedure initiated by controller
     *     ... -> LLC: llc_xxx_proc_start()
     *     activate LLC
     *         note over LLC #aqua: Allocate and fill procedure message
     *         LLC --> LLC : LLC_OP_XXX_IND
     *         note right: send message to local task instance \nto flow control procedure
     *     deactivate LLC
     * end
     * @enduml
     *
     * Example of simple local procedure
     *
     * @startuml llc_local_proc_msc.png
     * participant HCI
     * participant ...
     * participant LLC
     * participant LLD
     * ... --> LLC : LLC_OP_XXX_IND
     * LLC -> LLC: llc_op_xxx_ind_handler()
     * activate LLC
     *     hnote over LLC #lightgreen: Register Procedure\nLOCAL_PROC = busy
     *     LLC -> LLC: llc_op_xxx_continue(START)
     *     activate LLC
     *         note over LLC #aqua: state = WAIT_XXX_RSP\nStart transaction timer
     *         LLC --> LLD: LLCP_XXX_REQ
     *     deactivate LLC
     * deactivate LLC
     * ... wait for LLCP response ...
     * LLD --> LLC: LLCP_XXX_RSP
     * LLC -> LLC: llcp_xxx_rsp_handler()
     * activate LLC
     *     note over LLC: copy information
     *     LLC -> LLC: llc_op_xxx_continue(XXX_RSP)
     *     activate LLC
     *         note over LLC #aqua: state = Done\nStop transaction timer
     *         hnote over LLC #lightgreen: Unregister Procedure\nLOCAL_PROC = idle
     *         LLC --> HCI: HCI_XXX_INFO_CMP_EVT
     *     deactivate LLC
     * deactivate LLC
     * @enduml
     *
     */
    LLC_PROC_LOCAL,
    /** Remote initiated procedure
     *
     *
     * Example of simple remote procedure
     *
     * @startuml llc_remote_proc_msc.png
     * participant HCI
     * participant LLC
     * participant LLC_LLCP
     * participant LLD
     * LLD --> LLC: LLCP_XXX_REQ
     * LLC -> LLC: llcp_xxx_req_handler()
     * activate LLC
     *     hnote over LLC #lightgreen: Allocate procedure message\nRegister Procedure\nREMOTE_PROC = busy
     *     LLC -> LLC: llc_op_xxx_continue(START)
     *     activate LLC
     *         note over LLC #aqua: state = WAIT_HCI_XXX_RSP
     *         LLC --> HCI: HCI_XXX_REQ_EVT
     *     deactivate LLC
     * deactivate LLC
     * ... wait for HCI response ...
     * HCI --> LLC: HCI_XXX_REQ_REPLY_CMD
     * LLC -> LLC: hci_xxx_req_reply_cmd_handler()
     * activate LLC
     *     note over LLC: copy information
     *     LLC --> HCI: HCI_CMD_CMP_EVT
     *     LLC -> LLC: llc_op_xxx_continue(HCI_XXX_RSP)
     *     activate LLC
     *         note over LLC #aqua: state = WAIT_LLCP_ACK
     *         LLC --> LLD: LLCP_XXX_RSP
     *     deactivate LLC
     * deactivate LLC
     * ... wait for LLCP Baseband ACK ...
     * LLD --> LLC_LLCP: LLD_LLCP_TX_CFM
     * LLC_LLCP -> LLC_LLCP: lld_llcp_tx_cfm_handler()
     * activate LLC_LLCP
     *     LLC_LLCP -\ LLC: llcp_xxx_rsp_ack_handler()
     *     activate LLC
     *         LLC -> LLC: llc_op_xxx_continue(LLCP_ACK)
     *         activate LLC
     *             note over LLC #aqua: state = Done
     *             hnote over LLC #lightgreen: Unregister Procedure\nREMOTE_PROC = idle
     *         LLC -/ LLC_LLCP
     *         deactivate LLC
     *     deactivate LLC
     * deactivate LLC_LLCP
     * @enduml
     *
     */
    LLC_PROC_REMOTE,
    /// Max number of operations
    LLC_PROC_MAX
};

/// Possible states of the LLC task
enum llc_state_id
{
    /// Connection ready state
    LLC_IDLE            = 0x00,
    /// Local procedure on-going
    LLC_LOC_PROC_BUSY   = (1 << LLC_PROC_LOCAL),
    /// Remote procedure on-going
    LLC_REM_PROC_BUSY   = (1 << LLC_PROC_REMOTE),

    /// Not connected
    LLC_FREE            = 0xFF,
};
//



/// Procedure identifier
/*@TRACE*/
enum llc_proc_id
{
    /// No procedure on-going
    LLC_PROC_NONE,
    /// Disconnection procedure
    LLC_PROC_DISCONNECT,
    /// Version Exchange procedure
    LLC_PROC_VERSION_EXCHANGE,
    /// Pause Encryption or Encryption Start procedure
    LLC_PROC_ENCRYPT,
    /// Feature Exchange procedure
    LLC_PROC_FEATURE_EXCHANGE,
    /// Connection update
    LLC_PROC_CON_UPDATE,
    /// Channel map update
    LLC_PROC_CH_MAP_UPDATE,
    /// Data Length  update
    LLC_PROC_DL_UPDATE,
    /// LE Ping exchange
    LLC_PROC_LE_PING,
    /// PHY Update procedure
    LLC_PROC_PHY_UPDATE,
#if (BLE_CON_CTE_REQ | BLE_CON_CTE_RSP)
    /// Constant Tone Extension procedure
    LLC_PROC_CTE,
#endif // (BLE_CON_CTE_REQ | BLE_CON_CTE_RSP)
    /// Clock accuracy Exchange procedure
    LLC_PROC_CLK_ACC,
#if (BLE_CIS)
    /// CIS Creation procedure
    LLC_PROC_CIS_CREATE,
    /// CIS Stop procedure
    LLC_PROC_CIS_STOP,
#endif //(BLE_CIS)
#if BLE_PWR_CTRL
    /// Power Control procedure
    LLC_PROC_PWR_CTRL,
#endif // BLE_PWR_CTRL
#if (BT_53)
    LLC_PROC_SUBRATE,
    /// Channel classification procedure
    LLC_PROC_CH_CLASS,
#endif // (BT_53)
};


/// List of unexpected error received during a procedure
enum llc_error_type
{
    /// Disconnection detected
    LLC_ERR_DISCONNECT,
    /// LLCP Unknown PDU received
    LLC_ERR_LLCP_UNKNOWN_RSP,
    /// LLCP Reject PDU received
    LLC_ERR_LLCP_REJECT_IND,
    /// LLCP Reject Extended PDU received
    LLC_ERR_LLCP_REJECT_EXT_IND,
};

/// Options for restoring subrating after procedures with instants
enum llc_subrate_restore
{
    /// Do not restore. For cleanup only.
    LLC_SUBRATE_NO_RESTORE,
    /// Partial restore of subrating configurations - excluding connection latency and timeout.
    LLC_SUBRATE_PARTIAL_RESTORE,
    /// Full restore of subrating configurations - including connection latency and timeout.
    LLC_SUBRATE_FULL_RESTORE,
};

/** LLCP state used for some operation flow control
 *
 * @startuml llc_llcp_state.png
 * state "IDLE" as idle
 * state "PAUSE_ENC" as pause
 * state "START_ENC" as enc
 * state "TERMINATE" as terminate
 * idle   -down->  terminate : disconnect
 * enc    -down->  terminate : disconnect
 * pause  -down->  terminate : disconnect
 * idle   -right-> pause     : re-encrypt
 * idle   -->      enc       : encrypt
 * pause  -->      enc       : paused
 * enc    -->      idle      : encrypted
 * @enduml
 *
 */
enum llc_llcp_state
{
    /// No specific on-going procedure
    LLC_LLCP_IDLE,
    /// Pause encryption procedure on-going
    LLC_LLCP_PAUSE_ENC,
    /// Start encryption procedure on-going
    LLC_LLCP_START_ENC,
    /// Link termination procedure on-going
    LLC_LLCP_TERMINATE,
};

/// LLCP direction used to configure the flow control
enum llc_llcp_dir
{
    LLC_LLCP_DIR_RX           = 0,
    LLC_LLCP_DIR_TX           = 1,
    LLC_LLCP_DIR_BOTH         = 2,
};

/// Bit field definitions for LLCP states
///    7    6    5    4    3    2    1    0
/// +----+----+----+----+----+----+----+----+
/// |        RFU        |    TX   |    RX   |
/// +----+----+----+----+----+----+----+----+
enum llc_llcp_state_bf
{
    /// RX field position
    LLC_LLCP_RX_LSB           = 0,
    LLC_LLCP_RX_MASK          = (0x3),

    /// TX field position
    LLC_LLCP_TX_LSB           = 2,
    LLC_LLCP_TX_MASK          = (0xC),
};


/// Flow control bit field to flow control procedures requested by host and controller (autonomous)
///
///    15     14     13     12     11     10     9      8       7    6    5    4    3    2    1    0
/// +------+------+------+------+------+------+------+------+-------+----+----+----+----+----+----+----+
/// |      |      |      |      |      |      | SUB  | PWR  | L_C_UP| PHY|S_DL|C_UP|ENC |FEAT| VER|DISC|
/// +------+------+------+------+------+------+------+------+-------+----+----+----+----+----+----+----+
///
enum llc_flow_ctrl
{
    /// HCI Disconnect requested by host
    LLC_HCI_DISCONNECT_REQ_POS       = 0,
    LLC_HCI_DISCONNECT_REQ_BIT       = (1 << 0),

    /// HCI Read Remote Version requested by host
    LLC_HCI_RD_REM_VER_REQ_POS       = 1,
    LLC_HCI_RD_REM_VER_REQ_BIT       = (1 << 1),

    /// HCI Read Remote features requested by host
    LLC_HCI_RD_REM_FEAT_REQ_POS      = 2,
    LLC_HCI_RD_REM_FEAT_REQ_BIT      = (1 << 2),

    /// HCI Start Encryption requested by host
    LLC_HCI_START_ENCRYPT_REQ_POS    = 3,
    LLC_HCI_START_ENCRYPT_REQ_BIT    = (1 << 3),

    /// HCI Connection Update requested by host
    LLC_HCI_CON_UPDATE_REQ_POS       = 4,
    LLC_HCI_CON_UPDATE_REQ_BIT       = (1 << 4),

    /// HCI Set Data Length requested by host
    LLC_HCI_SET_DATA_LEN_REQ_POS     = 5,
    LLC_HCI_SET_DATA_LEN_REQ_BIT     = (1 << 5),

    /// HCI Set PHY requested by host
    LLC_HCI_SET_PHY_REQ_POS          = 6,
    LLC_HCI_SET_PHY_REQ_BIT          = (1 << 6),

    /// Local Connection Update requested by controller
    LLC_LOC_CON_UPDATE_REQ_POS       = 7,
    LLC_LOC_CON_UPDATE_REQ_BIT       = (1 << 7),

#if BLE_PWR_CTRL
    /// HCI Read Remote Transmit Power Level requested by host
    LLC_HCI_PWR_CTRL_REQ_POS         = 8,
    LLC_HCI_PWR_CTRL_REQ_BIT         = (1 << 8),
#endif // BLE_PWR_CTRL

#if (BT_53)
    /// Local Subrate requested by controller
    LLC_LOC_SUBRATE_REQ_POS          = 9,
    LLC_LOC_SUBRATE_REQ_BIT          = (1 << 9),
#endif // (BT_53)
};




/// Link information bit field
///
///    15     14     13     12     11     10     9      8      7      6      5      4      3      2      1      0
/// +------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+
/// |      |      |      |      |      | CK_A | ENC_S|CH UPD| PF_R | LDIS | ENC  | INST | LV_S | PV_R | TX_O | ROLE |
/// +------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+
///
enum llc_info
{
    /// Device is master of the link
    LLC_INFO_MASTER_ROLE_POS            = 0,
    LLC_INFO_MASTER_ROLE_BIT            = (1 << 0),

    /// LLCP buffer TX on-going
    LLC_LLCP_TX_ONGOING_POS             = 1,
    LLC_LLCP_TX_ONGOING_BIT             = (1 << 1),

    /// Peer version information received
    LLC_INFO_PEER_VERSION_RECV_POS      = 2,
    LLC_INFO_PEER_VERSION_RECV_BIT      = (1 << 2),

    /// Local version information sent
    LLC_INFO_LOCAL_VERSION_SENT_POS     = 3,
    LLC_INFO_LOCAL_VERSION_SENT_BIT     = (1 << 3),

    /// Procedure with instant on-going
    LLC_INFO_INSTANT_PROC_POS           = 4,
    LLC_INFO_INSTANT_PROC_BIT           = (1 << 4),

    /// Current Link is encrypted
    LLC_INFO_LINK_ENCRYPTED_POS         = 5,
    LLC_INFO_LINK_ENCRYPTED_BIT         = (1 << 5),

    /// Discard LLCP reception debug feature
    LLC_INFO_DBG_LLCP_DISCARD_POS       = 6,
    LLC_INFO_DBG_LLCP_DISCARD_BIT       = (1 << 6),

    /// Peer feature information received
    LLC_INFO_PEER_FEATURE_RECV_POS      = 7,
    LLC_INFO_PEER_FEATURE_RECV_BIT      = (1 << 7),

    /// Local channel map update ongoing
    LLC_INFO_LOC_CH_MAP_UPD_POS         = 8,
    LLC_INFO_LOC_CH_MAP_UPD_BIT         = (1 << 8),

    /// Encryption start ongoing
    LLC_INFO_ENC_START_POS              = 9,
    LLC_INFO_ENC_START_BIT              = (1 << 9),

    /// Accurate clock indicated
    LLC_INFO_CLK_ACC_POS                = 10,
    LLC_INFO_CLK_ACC_BIT                = (1 << 10),
};

#if (BT_53 && BLE_PERIPHERAL)
enum llc_ch_class_rep
{
    /// Bit indicating channel classification reporting is enable
    LLC_CH_CLASS_REP_EN_POS         = 0,
    LLC_CH_CLASS_REP_EN_BIT         = 0x1,

    /// Bit indicating channel classification reporting interval is custom
    LLC_CH_CLASS_REP_INTV_CUST_POS  = 1,
    LLC_CH_CLASS_REP_INTV_CUST_BIT  = 0x2,
};
#endif // (BT_53 && BLE_PERIPHERAL)


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/**
 * Local or Remote procedure callback used to inform if an unexpected error is raised during procedure execution
 *
 * @param[in] link_id     Link Identifier
 * @param[in] error_type  Error type (@see enum llc_error_type)
 * @param[in] param       Parameter according to error type:
 *   - LLC_ERR_DISCONNECT:          reason
 *   - LLC_ERR_LLCP_UNKNOWN_RSP:    struct ll_unknown_rsp*
 *   - LLC_ERR_LLCP_REJECT_IND:     struct ll_reject_ind*
 *   - LLC_ERR_LLCP_REJECT_IND_EXT: struct ll_reject_ext_ind*
 */
/*@TRACE*/
typedef void (*llc_proc_error_cb)(uint8_t link_id, uint8_t error_type, void *param);



/// Version information structure
typedef struct
{
    /// LMP version
    uint8_t  lmp_vers;
    /// Manufacturer ID
    uint16_t compid;
    /// LMP subversion
    uint16_t subvers;
} llc_version_t;


/// Information about on-going procedure
/*@TRACE*/
typedef struct
{
    /// Unexpected Error callback
    llc_proc_error_cb   error_cb;
    /// Procedure identifier
    //@trc_ref llc_proc_id
    uint8_t             id;
    /// Current procedure state
    uint8_t             state;
    /// Timer procedure state, shall not be modified by procedure (@see enum llc_timer_state)
    //@trc_ref llc_timer_state
    uint8_t             timer_state;
} llc_procedure_t;


/// LE Ping information
typedef struct
{
    /// Authenticated payload timeout (N*10ms)
    uint16_t auth_payl_to;
    /// Authenticated payload timeout (N*10ms), Near timer duration
    uint16_t auth_near_payl_to;
} llc_le_ping_t;


#if (BLE_CON_CTE_REQ)
/// CTE information
typedef struct
{
    /// RX parameters received
    bool param_received;
    /// Sampling enable
    bool sampl_en;
    /// Request active
    bool req_active;
    /// CTE request interval (in number of connection events)
    uint16_t    cte_req_intv;
    /// Requested CTE length (in 8us unit)
    uint8_t     req_cte_len;
    /// Requested CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t     req_cte_type;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t     slot_dur;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     switching_pattern_len;
} llc_cte_rx_t;
#endif // (BLE_CON_CTE_REQ)

#if (BLE_CON_CTE_RSP)
/// CTE information
typedef struct
{
    /// RX parameters received
    bool param_received;
    /// CTE TX enable status (@see enum co_error)
    /// CO_ERROR_NO_ERROR: CTE TX enabled
    /// (!= CO_ERROR_NO_ERROR): Error code representing the reason why CTE TX is disabled
    uint8_t en_status;
    /// Used CTE types (bit0: AOA | bit1: AOD-1us | bit2: AOD-2us)
    uint8_t     cte_types;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     switching_pattern_len;
} llc_cte_tx_t;
#endif // (BLE_CON_CTE_RSP)

/// Information about connection parameters
typedef struct
{
    /// Current channel map used for connection
    struct le_ch_map    ch_map;

    /// Interval (in units of 1,25 ms, i.e. 2 slots)
    uint16_t            interval;
    /// Latency (in units of event)
    uint16_t            latency;
    /// Link supervision timeout (in units of 10 ms)
    uint16_t            timeout;

#if (BT_53)
    /// Subrate factor (in units of connection event)
    uint16_t            subrate;
    /// Subrate continuation number (in units of connection event)
    uint16_t            cont_num;
#endif //(BT_53)

    // **** Data Length Management ****
    ///the maximum number of octets that the local device will send to the remote device
    uint16_t            max_tx_octets;
    ///the maximum number of octets that the local device is able to receive from the remote device
    uint16_t            max_rx_octets;

    ///the maximum number of microsecond that the local device will take to transmit to the remote
    uint16_t            max_tx_time;
    ///the maximum number of microsecond that the local device can take to receive from the remote device
    uint16_t            max_rx_time;

    ///the maximum number of octets that the remote device will send to the local device
    uint16_t            rem_max_tx_octets;
    ///the maximum number of octets that the remote device is able to receive from the local device
    uint16_t            rem_max_rx_octets;

    ///the maximum number of microsecond that the remote device will take to transmit to the local device
    uint16_t            rem_max_tx_time;
    ///the maximum number of microsecond that the remote device can take to receive from the local device
    uint16_t            rem_max_rx_time;

    ///effective max Tx octets
    uint16_t            eff_max_tx_octets;
    ///effective max Rx octets
    uint16_t            eff_max_rx_octets;

    ///effective max Tx time in us
    uint16_t            eff_max_tx_time;
    ///effective max Rx time in us
    uint16_t            eff_max_rx_time;

    // ****    PHY  Management     ****
    /// Current TX PHY  (@see enum le_phy_value)
    uint8_t             tx_phy;
    /// Current RX PHY  (@see enum le_phy_value)
    uint8_t             rx_phy;

    // **** Min Used Channels Management ****
    /// PHY(s) for which the slave has a minimum number of used channels requirement
    uint8_t    min_used_ch_phys;
    /// minimum number of channels to be used on the specified PHY
    uint8_t    min_used_ch;

    /// Peer's sleep clock accuracy (only for slave) (@see enum SCA)
    uint8_t sca;

} llc_con_params_t;

#if (BLE_PAST)
/// Information received from LL_PER_SYNC_IND
typedef struct
{
    /// SyncInfo
    struct sync_info syncinfo;
    /// ID (provided by the Host)
    uint16_t       id;
    /// Connection event counter of the LLCP reception
    uint16_t       con_evt_cnt_rx;
    /// Connection event counter
    uint16_t       con_evt_cnt;
    /// Last periodic advertising event counter
    uint16_t       last_pa_evt_cnt;
    /// Advertising Set ID, advertiser address type and sleep clock accuracy
    uint8_t        sid_atype_sca;
    /// Periodic advertising PHY (@see enum le_phy_mask)
    uint8_t        phy;
    /// Periodic advertiser address
    struct bd_addr adva;
    /// Periodic advertiser RPA (unresolved RPA if advertiser address is a resolved identity address, 0 otherwise)
    struct bd_addr adv_rpa;
    /// Sync Connection event counter
    uint16_t       sync_con_evt_cnt;
} llc_past_rx_info_t;

/// Periodic advertising sync transfer parameters
typedef struct
{
    /// Information received from LL_PER_SYNC_IND (temporarily stored during address resolution)
    llc_past_rx_info_t rx_info;

    // ****    PAST RX     ****
    /// Mode (@see enum per_adv_sync_info_rec_mode)
    uint8_t   mode;
    /// The number of periodic advertising packets that can be skipped after a successful receive
    uint16_t  skip;
    /// Sync timeout (Time=N*10ms)
    uint16_t  sync_to;
    /// CTE type (@see enum sync_cte_type)
    uint8_t   cte_type;
} llc_past_param_t;
#endif // (BLE_PAST)

#if BLE_PWR_CTRL
/// Power control information
typedef struct
{
    /// Local device transmit power level (in dBm)
    int8_t loc_tx_pwr[CO_RATE_MAX];
    /// Local tx power reports enabled
    bool loc_tx_pwr_rep_en;
    /// Remote tx power reports enabled
    bool rem_tx_pwr_rep_en;
    /// Local power change indications enabled
    bool loc_tx_pwr_chg_ind_en;
    /// Remote power change indications enabled
    bool rem_tx_pwr_chg_ind_en;
    /// Path loss monitoring configured
    bool path_loss_cfg;
    /// PHY mask currently in use by CIS
    uint8_t cis_phy_msk;
} llc_pwr_params_t;
#endif // BLE_PWR_CTRL

#if (BT_53)
/// Channel classification information
typedef struct
{
#if (BLE_PERIPHERAL)
    /// Channel classification reporting interval (in ms)
    uint16_t    ch_class_rep_intv;
    /// Bit field (@see enum llc_ch_class_rep)
    uint8_t    en;
#endif // (BLE_PERIPHERAL)
    /// Channel classification map
    struct le_ch_class ch_class_map;
} llc_ch_class_t;
#endif // (BT_53)

#if (BT_53) && (BLE_CENTRAL)
/// Subrate information
typedef struct
{
    /// Subrate min (in units of connection event)
    uint16_t subrate_min;
    /// Subrate max ((in units of connection event)
    uint16_t subrate_max;
    /// Max latency (in units of connection event)
    uint16_t max_latency;
    /// Continuation number (in units of connection event)
    uint16_t cont_num;
    /// Supervision timeout (N x 10ms)
    uint16_t timeout;

    /// suspended Subrate factor (in units of connection event, 0 if not suspended)
    uint16_t            subrate_ss;
    /// suspended Latency (in units of connection event, 0 if not suspended)
    uint16_t            latency_ss;
    /// suspended Continuation number (in units of connection event, 0 if not suspended)
    uint16_t            cont_num_ss;

} llc_subrate_info_t;
#endif // (BT53) && (BLE_CENTRAL)

/// LLC environment structure
struct llc_env_tag
{
    /// Procedure Kernel message saved
    llc_procedure_t        *procedure[LLC_PROC_MAX];
    /// Current connection parameters
    llc_con_params_t        con_params;
    /// Queue of LLCP to be sent
    struct co_list          llcp_tx_queue;
    /// Remote device feature exchange and used by the stack
    struct le_features      rem_feats;
    ///version exchange env
    llc_version_t           rem_ver;
    /// LE Ping information
    llc_le_ping_t           le_ping;
#if (BLE_CON_CTE_REQ)
    /// CTE RX information
    llc_cte_rx_t            cte_rx;
#endif // (BLE_CON_CTE_REQ)
#if (BLE_CON_CTE_RSP)
    /// CTE TX information
    llc_cte_tx_t            cte_tx;
#endif // (BLE_CON_CTE_RSP)

#if (BLE_PAST)
    /// Periodic Advertising Sync Transfer parameters
    llc_past_param_t        past;
#endif // (BLE_PAST)

#if BLE_PWR_CTRL
    /// Power control information per PHY rate
    llc_pwr_params_t        pwr;
#endif // BLE_PWR_CTRL

#if (BT_53)
    /// Channel classification information
    llc_ch_class_t          ch_class;
#endif // (BT_53)

#if (BT_53) && (BLE_CENTRAL)
    /// Subrate information
    llc_subrate_info_t      subrate_info;
#endif // (BT_53) && (BLE_CENTRAL)

    // PHY  Management
    /// Phy options indicated by Host (@see enum le_phy_opt) (by default 0 if never set by Host)
    uint16_t   phy_opt;
    /// Default TX preferred PHY to use (@see enum le_phy_mask)
    uint8_t    tx_phys;
    /// Default RX preferred PHY to use (@see enum le_phy_mask)
    uint8_t    rx_phys;

    /// Information about current link status (@see enum llc_info)
    uint16_t link_info;
    /// Information about the current link state (@see enum llc_llcp_state)
    uint8_t  llcp_state;
    /// Flow control information of HCI commands and local (autonomous) procedures (see enum llc_flow_ctrl)
    uint16_t  flow_ctrl;
    /// Disconnection reason
    uint8_t  disc_reason;
    /// Indicate L2CAP start fragment is pending
    bool  l2cap_start;
    /// Indicate connection driver is stopping or has already stopped
    bool  con_stop;
};

/// AES function execution result
struct llc_encrypt_ind
{
    /// Status of AES execution
    uint8_t status;
    /// Result of the
    uint8_t result[KEY_LEN];
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 *****************************************************************************************
 */

/// LLC task descriptor
extern const struct ke_task_desc TASK_DESC_LLC;

/// LLM environment variable
extern struct llc_env_tag *llc_env[];


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Perform a Link environment variable clean-up due to disconnection or reset
 *
 * @param[in] link_id Link Identifier
 ****************************************************************************************
 */
void llc_cleanup(uint8_t link_id, bool reset);


/**
 ****************************************************************************************
 * @brief Stop the link to ensure that all host request are correctly answered.
 *
 * It restore the message queue to force finishing postponed operations and then
 * send a message to fully stop the link.
 *
 * @param[in] link_id Link Identifier
 ****************************************************************************************
 */
void llc_stop(uint8_t link_id);

#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Check if the feature is supported
 *
 * @param[in] link_id  Link identifier
 * @param[in] feature  Feature to check (@see enum ble_feature)
 *
 * @return True if peer device feature support is expected, False if not supported
 ****************************************************************************************
 */
__INLINE bool llc_le_feature_check(uint8_t link_id, uint8_t feature)
{
    bool ret = false;

    if (link_id < BLE_ACTIVITY_MAX)
    {
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];
        if (llc_env_ptr != NULL)
        {
            ret = ((llc_env_ptr->rem_feats.feats[feature >> 3] & (1 << (feature & 0x7))) != 0) ? true : false;
        }
    }
    return (ret);
}

/**
 ****************************************************************************************
 * @brief Set/Force if peer feature is supported
 *
 * @param[in] link_id  Link identifier
 * @param[in] feature  Feature to check (@see enum ble_feature)
 * @param[in] enable   True to enable the feature, false else
 ****************************************************************************************
 */
__INLINE void llc_le_feature_set(uint8_t link_id, uint8_t feature, bool enable)
{
    if (link_id < BLE_ACTIVITY_MAX)
    {
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];
        if (llc_env_ptr != NULL)
        {
            if (enable)
            {
                llc_env_ptr->rem_feats.feats[feature >> 3] |= (1 << (feature & 0x7));
            }
            else
            {
                llc_env_ptr->rem_feats.feats[feature >> 3] &= ~(1 << (feature & 0x7));
            }
        }
    }
}

/**
 ****************************************************************************************
 * @brief Selects an instant based on procedures with instants rules.
 *
 * @param[in] link_id  Link identifier
 *
 * @return Connection event count for Instant
 ****************************************************************************************
 */
uint16_t llc_inst_select(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Check if the link is disconnecting or not established
 *
 * @param[in] link_id  Link identifier
 *
 * @return True if a link is disconnecting or already in not connected state, False else
 ****************************************************************************************
 */
__INLINE bool llc_is_disconnecting(uint8_t link_id)
{
    bool res = true;
    if (link_id < BLE_ACTIVITY_MAX)
    {
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];
        if (llc_env_ptr != NULL)
        {
            res = (GETF(llc_env_ptr->llcp_state, LLC_LLCP_RX) == LLC_LLCP_TERMINATE);
        }
    }
    return (res);
}
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Update the link state according to on-going activity
 *
 * When the state goes to terminate, it cannot return to another state.
 * Also when LLCP TX state is updated, LLCP module in informed to update the TX Queue list
 * if some LLCP transmission are paused.
 *
 * LLC can set asymmetric RX/TX configuration during encryption procedures.
 * When setting Idle or Terminate states, both directions shall be configured at the same time.
 *
 * @param[in] link_id Link identifier
 * @param[in] dir     Direction (0:RX | 1:TX | 2:both)
 * @param[in] state   New state to apply
 ****************************************************************************************
 */
void llc_llcp_state_set(uint8_t link_id, uint8_t dir, uint8_t state);


/* ************************************************************************************ */
/* ********************            PROCEDURE MANAGEMENT          ********************** */
/* ************************************************************************************ */

/**
 ****************************************************************************************
 * @brief Register/Start a procedure
 *
 * @param[in] link_id   Link identifier
 * @param[in] proc_type Local or Remote Procedure (@see enum llc_proc_type)
 * @param[in] params    Procedure parameters
 ****************************************************************************************
 */
void llc_proc_reg(uint8_t link_id, uint8_t proc_type, llc_procedure_t *params);

/**
 ****************************************************************************************
 * @brief Unregister/finish a procedure
 *
 * @param[in] link_id   Link identifier
 * @param[in] proc_type Local or Remote Procedure (@see enum llc_proc_type)
 ****************************************************************************************
 */
void llc_proc_unreg(uint8_t link_id, uint8_t proc_type);

/**
 ****************************************************************************************
 * @brief Retrieve current procedure
 *
 * @param[in] link_id   Link identifier
 * @param[in] proc_type Local or Remote Procedure (@see enum llc_proc_type)
 *
 * @return current procedure type or LLC_PROC_NONE if not started (@see enum llc_proc_id_type)
 ****************************************************************************************
 */
uint8_t llc_proc_id_get(uint8_t link_id, uint8_t proc_type);

/**
 ****************************************************************************************
 * @brief Initialize the procedure state
 *
 * @param[in] proc          procedure
 * @param[in] proc_id       procedure ID
 * @param[in] errro_cb      error callback
 ****************************************************************************************
 */
void llc_proc_init(llc_procedure_t *proc, uint8_t proc_id, llc_proc_error_cb error_cb);

/**
 ****************************************************************************************
 * @brief Retrieve procedure state
 *
 * @param[in] proc   procedure
 *
 * @return procedure state
 ****************************************************************************************
 */
uint8_t llc_proc_state_get(llc_procedure_t *proc);

/**
 ****************************************************************************************
 * @brief Set the procedure state
 *
 * @param[in] proc          procedure
 * @param[in] link_id       link identifier
 * @param[in] proc_state    new procedure state
 ****************************************************************************************
 */
void llc_proc_state_set(llc_procedure_t *proc, uint8_t link_id, uint8_t proc_state);

/**
 ****************************************************************************************
 * @brief Retrieve pointer on the current procedure
 *
 * @param[in] link_id   Link identifier
 * @param[in] proc_type Local or Remote Procedure (@see enum llc_proc_type)
 *
 * @return Pointer on the current procedure type or NULL if not used
 *****************************************************************************************
 */
llc_procedure_t *llc_proc_get(uint8_t link_id, uint8_t proc_type);


/**
 ****************************************************************************************
 * @brief Inform on-going procedure that an unexpected error event happens
 *
 * @param[in] link_id     Link identifier
 * @param[in] proc_type   Local or Remote Procedure (@see enum llc_proc_type)
 * @param[in] error_type  Type of procedure error detected (@see enum llc_error_type)
 * @param[in] param       Parameter according to error type (@see llc_proc_error_cb)
 *****************************************************************************************
 */
void llc_proc_err_ind(uint8_t link_id, uint8_t proc_type, uint8_t error_type, void *param);

/**
 ****************************************************************************************
 * @brief Enable or Disable Local or Remote procedure LLCP transaction timer
 *
 * @param[in] link_id     Link identifier
 * @param[in] proc_type   Local or Remote Procedure (@see enum llc_proc_type)
 * @param[in] enable      True to enable/restart the LLCP transaction timer, False to clear it
 *****************************************************************************************
 */
void llc_proc_timer_set(uint8_t link_id, uint8_t proc_type, bool enable);

/**
 ****************************************************************************************
 * @brief Pause or Resume Local or Remote procedure LLCP transaction timer
 *
 * (if a timer is already started, it will be paused until resume is requested, see state
 * machine described in @see enum llc_timer_state)
 *
 * @param[in] link_id     Link identifier
 * @param[in] proc_type   Local or Remote Procedure (@see enum llc_proc_type)
 * @param[in] enable      True to pause the LLCP transaction timer, False to resume it
 *****************************************************************************************
 */
void llc_proc_timer_pause_set(uint8_t link_id, uint8_t proc_type, bool enable);
/**
 ****************************************************************************************
 * @brief Check if a procedure is not entering in collision with another one.
 *
 * @param[in] link_id       Link Identifier
 * @param[in] proc_id       New procedure to start
 *
 * @return CO_ERROR_NO_ERROR if no collision;
 *         CO_ERROR_LMP_COLLISION or CO_ERROR_DIFF_TRANSACTION_COLLISION else.
 ****************************************************************************************
 */
uint8_t llc_proc_collision_check(uint8_t link_id, uint8_t proc_id);




/* ************************************************************************************ */
/* *************************             HCI HELPERS             ********************** */
/* ************************************************************************************ */


/**
 ****************************************************************************************
 * @brief Send HCI CC event returning a status only
 *
 * @param[in] link_id Link Identifier
 * @param[in] opcode  HCI Operation code
 * @param[in] status  HCI CC status code
 ****************************************************************************************
 */
void llc_cmd_cmp_send(uint8_t link_id, uint16_t opcode, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send HCI CS event
 *
 * @param[in] link_id Link Identifier
 * @param[in] opcode  HCI Operation code
 * @param[in] status  HCI CS status code
 ****************************************************************************************
 */
void llc_cmd_stat_send(uint8_t link_id, uint16_t opcode, uint8_t status);



/* ************************************************************************************ */
/* *************************           Procedure start           ********************** */
/* ************************************************************************************ */

/**
 ****************************************************************************************
 * @brief Initiate the termination procedure
 *
 * @param[in] link_id   Link Identifier
 * @param[in] reason    Reason of disconnection
 ****************************************************************************************
 */
void llc_init_term_proc(uint8_t link_id, uint8_t reason);

/**
 ****************************************************************************************
 * @brief Set the authenticated payload timeout and compute the timer margin.
 *
 * @param[in] link_id       Link Identifier
 * @param[in] auth_payl_to  Authenticated payload timeout
 *
 * @return CO_ERROR_NO_ERROR if succeed; CO_ERROR_INVALID_HCI_PARAM else.
 ****************************************************************************************
 */
uint8_t llc_le_ping_set(uint8_t link_id, uint16_t auth_payl_to);

/**
 ****************************************************************************************
 * @brief Restart the authenticated payload timeout timer (and its margin timer)
 * to check if encrypted link is still alive
 *
 * @param[in] link_id       Link Identifier
 * @param[in] auth_payl_to  Authenticated payload timeout
 ****************************************************************************************
 */
void llc_le_ping_restart(uint8_t link_id);

/**
 ****************************************************************************************
 * Start PHY update procedure
 *
 * @param[in] link_id Link identifier
 ****************************************************************************************
 */
void llc_phy_upd_proc_start(uint8_t link_id);

/**
 ****************************************************************************************
 * Start data length update procedure
 *
 * @param[in] link_id Link identifier
 ****************************************************************************************
 */
void llc_dl_upd_proc_start(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Sends the data length effective values to the host.
 *
 * @param[in] link_id           Link identifier on which the pdu will be sent
 * @param[in] status            Status of the host request
 ****************************************************************************************
 */
void llc_hci_dl_upd_info_send(uint8_t link_id, uint8_t status);

/**
 ****************************************************************************************
 * Compute and store effective values
 *
 * @param[in] link_id Link identifier
 ****************************************************************************************
 */
void llc_dl_compute_eff(uint8_t link_id);

#if (BT_53)
    /**
    ****************************************************************************************
    * Start feature exchange procedure if needed
    *
    * @param[in] link_id Link identifier
    ****************************************************************************************
    */
    void llc_feat_exch_proc_start(uint8_t link_id);

    #if (BLE_CENTRAL)
        /**
        ****************************************************************************************
        * Start channel classification enable procedure
        *
        * @param[in] link_id Link identifier
        ****************************************************************************************
        */
        void llc_ch_class_en_proc_start(uint8_t link_id);

        /**
        ****************************************************************************************
        * @brief Suspend subrating for procedures with instants if needed
        *
        * When performing a Link Layer procedure that has an instant, the Central's Link
        * Layer may (particularly for large values of the subrate factor) use the
        * Connection Subrate Update procedure to set the subrate factor to 1 and the
        * Peripheral latency to 0 before performing the procedure (and then use it again
        * to restore the previous values afterwards). vol 6b.5.5 procedures with instants.
        *
        * @param[in] link_id       Link identifier
        * @return    suspended     True if subrating is suspended, false otherwise.
        ****************************************************************************************
        */
        bool llc_subrate_suspend(uint8_t link_id);
    #endif //(BLE_CENTRAL)

    /**
    ****************************************************************************************
    * @brief Restore subrating after procedures with instants if needed
    *
    * This restores previously saved subrating configuration settings, which was suspended
    * for the duration of such procedures.
    *
    * @param[in] link_id       Link identifier
    * @param[in] restore_lvl   Restore level (@see enum llc_subrate_restore)
    ****************************************************************************************
    */
    void llc_subrate_restore(uint8_t link_id, uint8_t restore_lvl);
#endif //(BT_53)

#if BLE_CON_CTE_REQ
    /**
    ****************************************************************************************
    * @brief Disable CTE request
    *
    * @param[in] link_id       Link Identifier
    ****************************************************************************************
    */
    void llc_cte_req_dis(uint8_t link_id);
#endif // BLE_CON_CTE_REQ
#if BLE_CON_CTE_RSP
    /**
    ****************************************************************************************
    * @brief Disable CTE response
    *
    * @param[in] link_id       Link Identifier
    ****************************************************************************************
    */
    void llc_cte_rsp_dis(uint8_t link_id);
#endif // BLE_CON_CTE_RSP

/// @} LLC

#endif // LLC_INT_H_
