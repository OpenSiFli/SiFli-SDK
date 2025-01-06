/**
 ****************************************************************************************
 *
 * @file lc_int.h
 *
 * @brief Link controller internal definition
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef LC_INT_H_
#define LC_INT_H_

/**
 ****************************************************************************************
 * @addtogroup LC Link Controller
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>          // standard integer
#include "rwip_config.h"     // stack configuration
#include "co_bt.h"           // BT standard definitions
#include "co_utils.h"        // BT utility definitions
#include "co_math.h"        // BT utility definitions
#include "ke_msg.h"          // kernel message declaration
#include "ke_task.h"         // kernel task definitions
#include "lc_sco.h"     // SCO definitions


/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of instances of the LC task
#define LC_IDX_MAX  MAX_NB_ACTIVE_ACL

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

///Transition collision state or Extended opcode used
enum
{
    /// internal: lm_util not used
    LC_UTIL_NOT_USED,
    /// internal: lm_util used
    LC_UTIL_INUSED,
};

/// Possible states of the LC task
/*@TRACE*/
enum LC_STATE
{
    LC_FREE,
    LC_CONNECTED,
#if BCAST_ENC_SUPPORT
    LC_MST_WAIT_VERS,
    LC_MST_WAIT_ENC_SIZE_MSK,
#endif // BCAST_ENC_SUPPORT
    LC_SCO_NEGO_ONGOING,
    LC_SCO_DISC_ONGOING,
    LC_WAIT_CLK_OFF,
    LC_WAIT_TIM_ACC_RSP,
    LC_WAIT_QOS_CFM,
    LC_WAIT_ACL_ACC,
    LC_WAIT_TEST_TX_CFM,            //10
    LC_WAIT_MAX_SLOT_CFM,
    LC_PUB_KEY_HEADER_INIT_LOC,
    LC_PUB_KEY_PAYLOAD_INIT_LOC,
    LC_PUB_KEY_HEADER_RSP_CFM,
    LC_PUB_KEY_PAYLOAD_RSP_PEER_CFM,
    LC_REM_NAME_PAGE_CANCEL,
    LC_WAIT_REM_HL_CON,
    LC_WAIT_REM_NAME,
    LC_WAIT_REM_VERS,
    LC_WAIT_REM_FEATS,              //20
    LC_WAIT_REM_EXT_FEATS,
    LC_WAIT_DETACH_REQ_TX_CFM,
    LC_WAIT_DISC_TO,
    LC_WAIT_DISC_CFM,
    LC_WAIT_DETACH_TO,
    LC_WAIT_PAIR_CFM_INIT,
    LC_WAIT_PAIR_CFM_RSP,
    LC_WAIT_AUTH_SRES,
    LC_WAIT_KEY_EXCH,
    LC_WAIT_KEY_EXCH_RSP,           //30
    LC_WAIT_AU_RAND_RSP,
    LC_WAIT_AU_RAND_PEER_INIT,
    LC_WAIT_LINK_KEY,
    LC_WAIT_PIN_CODE,
    LC_WAIT_SRES_RSP,
    LC_WAIT_SRES_INIT,
    LC_WAIT_ENC_MODE_CFM,
    LC_WAIT_ENC_SIZE_CFM,
    LC_WAIT_ENC_STOP_CFM_MST,
    LC_WAIT_ENC_START_CFM,          //40
    LC_WAIT_ENC_SLV_SIZE,
    LC_WAIT_ENC_SLV_RESTART,
    LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT,
    LC_WAIT_ENC_START_REQ_SLV,
    LC_WAIT_EPR_ENC_PAUSE_REQ_MST_INIT,
    LC_WAIT_SNIFF_REQ,
    LC_WAIT_SNIFF_ACC_TX_CFM,
    LC_WAIT_SNIFF_SUB_RSP,
    LC_WAIT_SNIFF_SUB_RSP_TX_CFM,
    LC_WAIT_UNSNIFF_ACC,            //50
    LC_WAIT_UNSNIFF_ACC_TX_CFM,
    LC_WAIT_SWITCH_CFM,
    LC_WAIT_SWITCH_CMP,
#if BCAST_ENC_SUPPORT
    LC_WAIT_MST_SEMI_ACC,
#endif // BCAST_ENC_SUPPORT
    LC_WAIT_PKT_TBL_TYP_ACC_CFM,
    LC_WAIT_HL_IO_CAP_INIT,
    LC_WAIT_HL_IO_CAP_RSP,
    LC_WAIT_IO_CAP_INIT_CFM,
    LC_WAIT_PUB_KEY_HEADER_INIT_PEER,
    LC_WAIT_PUB_KEY_PAYLOAD_INIT_PEER, //60
    LC_WAIT_PUB_KEY_HEADER_RSP_PEER,
    LC_WAIT_PUB_KEY_PAYLOAD_RSP_PEER,
    LC_WAIT_NUM_COMP_COMM_INIT_CONF,
    LC_WAIT_NUM_COMP_COMM_INIT_RANDN_PEER,
    LC_WAIT_NUM_COMP_COMM_INIT_RANDN_CFM,
    LC_WAIT_NUM_COMP_USER_CONF_INIT,
    LC_WAIT_NUM_COMP_USER_CONF_RSP,
    LC_WAIT_NUM_COMP_RANDN_RSP_PEER,
    LC_WAIT_NUM_COMP_RANDN_RSP_PEER_CFM,
    LC_WAIT_DHKEY_RSP,                  //70
    LC_WAIT_DHKEY_CHECK_INIT_PEER,
    LC_WAIT_DHKEY_CHECK_INIT_CFM,
    LC_WAIT_DHKEY_CHECK_RSP_PEER_CFM,
    LC_WAIT_PASSKEY_HL_RPLY,
    LC_WAIT_PASSKEY_COMM_RSP_PEER,
    LC_WAIT_PASSKEY_COMM_INIT_PEER,
    LC_WAIT_PASSKEY_RANDN_INIT_CFM,
    LC_WAIT_PASSKEY_RANDN_INIT_PEER,
    LC_WAIT_PASSKEY_RANDN_RSP_PEER,
    LC_WAIT_PASSKEY_RANDN_RSP_PEER_CFM, //80
    LC_WAIT_HL_OOB_DATA,
    LC_WAIT_OOB_RANDN_RSP_PEER,
    LC_WAIT_OOB_RANDN_RSP_PEER_CFM,
    LC_WAIT_OOB_RANDN_INIT_PEER,
    LC_WAIT_OOB_RANDN_INIT_CFM,
    LC_WAIT_EPR_RSP,
    LC_WAIT_EPR_ENC_RESUME_REQ_MST_RSP,
    LC_WAIT_PWR_CTRL_RES,
    LC_WAIT_PWR_CTRL_TX_CFM,
    LC_WAIT_DHKEY_COMPUTING,            //90
    LC_WAIT_AUTH_STG1_NC_RANDN,
    LC_WAIT_AUTH_STG1_PK_RANDN,
    LC_WAIT_DHKEY_RSP_FAIL,
    LC_WAIT_EPR_PEER_REQ_MST,
    LC_WAIT_EPR_PEER_REQ_SLV,
    LC_WAIT_EPR_ENC_PAUSE_REQ_MST_RSP,
    LC_WAIT_EPR_ENC_STOP_REQ_SLV_RSP,
    LC_WAIT_RESTART_ENC,
    LC_WAIT_AU_RAND_SEC_AUTH_INIT,
    LC_WAIT_SRES_SEC_AUTH_INIT,        //100
    LC_WAIT_PING_RES,
    LC_WAIT_SSR_INSTANT,
    LC_WAIT_PTT_ACC_TX_CFM,

#if PCA_SUPPORT
    LC_WAIT_CLK_ADJ_REQ_ACC_CFM,
    LC_WAIT_CLK_ADJ_CFM,
#endif // PCA_SUPPORT

#if RW_BT_MWS_COEX
    LC_WAIT_SAM_SET_TYPE0_CFM,
    LC_WAIT_SAM_DEFINE_MAP_CFM,
    LC_WAIT_SAM_SWITCH_CFM,
#endif //RW_BT_MWS_COEX
    LC_WAIT_FLOW_SPEC_CFM,

#if RW_DEBUG
    LC_WAIT_DBG_SEND_LMP_CFM,
#endif // RW_DEBUG

    LC_WAIT_PRIV_PUB_KEY_GEN,

    // Number of states.
    LC_STATE_MAX
};

/// state of DH-KEY computation
enum lc_dh_key_compute_state
{
    LC_DH_KEY_COMP_SUCCESS,
    LC_DH_KEY_COMP_PENDING,
    LC_DH_KEY_COMP_FAILED,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// LC Link Structure
struct lc_link_tag
{
    /// token rate
    uint32_t TokenRate;
    /// peak bandwidth
    uint32_t PeakBandwidth;
    /// latency
    uint32_t Latency;
    /// delay variation
    uint32_t DelayVariation;
    /// access latency
    uint32_t AccessLatency;
    /// token bucket size
    uint32_t TokenBucketSize;
    /// switch instant
    uint32_t SwitchInstant;
    /// link timeout value
    uint16_t LinkTimeout;
    /// ACL basic packet type
    uint16_t AclPacketType;
    /// current packet type
    uint16_t CurPacketType;
    /// settings for link policy
    uint16_t LinkPolicySettings;
    /// polling interval
    uint16_t PollInterval;
    /// slot offset
    uint16_t SlotOffset;
    /// failed contact
    uint16_t FailedContact;
    /// Rx preferred rate
    uint16_t RxPreferredRate;
    /// Authenticated payload time out (expressed in units of 10 ms)
    uint16_t auth_payl_to;
    /// Authenticated payload time out margin (expressed in units of 10 ms)
    uint16_t auth_payl_to_margin;
    /// device class
    struct devclass Class;
    /// transmit current max slot
    uint8_t TxMaxSlotCur;
    /// received max slot
    uint8_t MaxSlotReceived;
    /// allowed role switch indication
    uint8_t AllowRoleSwitch;
    /// device role
    uint8_t Role;
    /// reason
    uint8_t Reason;
    /// logical transport address
    uint8_t LtAddr;
    /// Rx transaction id server
    uint8_t RxTrIdServer;
    /// RX transaction id client
    uint8_t RxTrIdClient;
    /// RX packet type ID
    uint8_t RxPktTypeID;
    /// current mode
    uint8_t CurrentMode;
    /// flow direction
    uint8_t FlowDirection;
    /// service type
    uint8_t ServiceType;
    /// current packet type table
    uint8_t CurPacketTypeTable;
    /// temporary packet type table
    uint8_t ptt_tmp;
    /// saved link state
    uint8_t saved_state;
    /// flag
    uint8_t Flags;
    /// MIC error counter
    uint8_t micerr_cnt;
    /// initiator flag
    bool    Initiator;
    /// connected state flag
    bool    ConnectedState;
    /// setup complete flag
    bool    SetupComplete;
    /// setup complete Rx flag
    bool    SetupCompRx;
    /// setup complete Tx flag
    bool    SetupCompTx;
    /// connection complete sent flag
    bool    ConnectionCompleteSent;
    /// host connected flag
    bool    HostConnected;
    /// flush continue flag
    bool    FlushContinue;
    /// minimum power received flag
    bool    MinPowerRcv;
    /// maximum power received flag
    bool    MaxPowerRcv;
    /// packet type table 2 MB flag
    bool    PacketTypeTable2Mb;
    /// quality of service notified flag
    bool    QOSNotified;
    /// Enhanced Power Control supported by peer
    bool epc_supported;
    /// Indicate L2CAP start fragment is pending
    bool    l2cap_start;
    /// eSCO loopback mode (False: disabled | True: enabled)
    bool    esco_loopback_mode;
#if RW_DEBUG
    /// LMP Packet Discard
    bool    LMPPacketDiscard;
#endif //RW_DEBUG
};

/// Structure for the version indication
struct lc_vers_tag
{
    /// company identification
    uint16_t compid;
    /// sub version
    uint16_t subvers;
    /// version
    uint8_t  vers;
};

/// LC Information Structure
struct lc_info_tag
{
    /// remote features (including extended)
    struct features     RemoteFeatures[FEATURE_PAGE_MAX];
    /// local bd address
    struct bd_addr      LocalBdAddr;
    /// bd address
    struct bd_addr      BdAddr;
    /// remote version
    struct lc_vers_tag  RemVers;
    /// remote name
    struct bd_name name_rem;
    /// boolean for send remote name confirm
    bool                SendRemoteNameCfm;
    /// Bit field indicating remote features pages received (bit0: page 0 | bit1: page 1 | bit2: page 2)
    uint8_t             RemFeatureRec;
    /// boolean for received version
    bool                RecvRemVerRec;
};

/// LC Encryption Structure
struct lc_enc_tag
{
    /// Key given from Host (or generated via pairing)
    uint8_t         key_from_host;
#if BCAST_ENC_SUPPORT
    /// key size mask
    uint16_t        KeySizeMask;
    /// key flag
    uint8_t         KeyFlag;
    /// new key flag
    uint8_t         NewKeyFlag;
#endif // BCAST_ENC_SUPPORT
    /// key type
    uint8_t         KeyType;
    /// status of the key
    uint8_t         KeyStatus;
    /// status of pin
    uint8_t         PinStatus;
    /// length of the pin
    uint8_t         PinLength;
    /// encryption size
    uint8_t         EncSize;
    /// encryption mode
    uint8_t         EncMode;
    /// new encryption mode
    uint8_t         NewEncMode;
    /// encryption enable
    uint8_t         EncEnable;
#if (BT_53)
    /// minimum encryption key size
    uint8_t         min_enc_key_size;
#if (RW_DEBUG)
    /// maximum encryption key size
    uint8_t         max_enc_key_size;
#endif // (RW_DEBUG)
#endif // (BT_53)
    /// Authentication key (secure connections)
    struct ltk      auth_key;
    /// long term key
    struct ltk      LTKey;
    /// semi permanent key
    struct ltk      SemiPermanentKey;
    /// overlay key
    struct ltk      Overlay;
    /// random rx key
    struct ltk      RandomRx;
    /// random tx key
    struct ltk      RandomTx;
    /// encryption key
    struct ltk      EncKey;
#if (EAVESDROPPING_SUPPORT)
    /// encryption IV
    struct initialization_vector EncIV;
#endif // EAVESDROPPING_SUPPORT
    /// signed response
    struct sres_nb  Sres;
    /// signed response expected from peer
    struct sres_nb  Sres_expected;
    /// aco value
    struct aco      Aco;
    /// pin code
    struct pin_code PinCode;
    /// boolean link key valid
    bool            LinkKeyValid ;
    /// Prevent encryption change event
    bool PreventEncEvt;
};
/// LC Request owner Structure
struct lc_req_tag
{
    // Request from local device
    /// local name request
    bool LocNameReq;
    /// local request for extended features
    bool LocRemoteExtendedReq;
    /// disconnection request
    bool LocDetachReq;
    /// change packet type request
    bool LocCPTReq;
    /// encryption request
    bool LocEncReq;
    /// authentication request
    bool LocAuthReq;
    /// key exchange request
    bool LocKeyExchangeReq;
    /// encryption key refresh
    bool LocEncKeyRefresh;
    /// switch request
    bool LocSwitchReq;
    /// version request
    bool LocVersReq;

    // Request from the Peer device
    /// switch request
    bool PeerSwitchReq;
    /// authentication request
    bool PeerAuthReq;
    /// encryption request
    bool PeerEncReq;
    /// disconnection request
    bool PeerDetachReq;
    /// encryption key refresh
    bool PeerEncKeyRefresh;

    // Request Internal
    /// restart encryption request
    bool RestartEncReq;
#if BCAST_ENC_SUPPORT
    /// master key request
    bool MasterKeyReq;
#endif // BCAST_ENC_SUPPORT
};

/// LC Test Mode Structure
struct lc_test_mode_tag
{
    /// Data length
    uint16_t DataLength;
    /// Test scenario
    uint8_t  TestScenario;
    /// Hopping mode
    uint8_t  HoppingMode;
    /// Transmit frequency
    uint8_t  TxFreq;
    /// Reception frequency
    uint8_t  RxFreq;
    /// Power control test
    uint8_t  PowerControl;
    /// Polling period
    uint8_t  PollPeriod;
    /// Packet type
    uint8_t  PacketType;
    /// Activation state
    bool activated;
};

/// LC AFH Structure
struct lc_afh_tag
{
    /// AFH channel map - defined by the master
    struct bt_ch_map   ch_map;
    /// AFH channel classification - computed by the slave
    struct bt_ch_class ch_class;
    /// Channel classification from peer
    struct bt_ch_map peer_ch_class;
    /// AFH reporting interval (in slots)
    uint32_t reporting_interval;
    /// AFH enable
    bool     en;
    /// AFH temporary state (state saved during Role switch)
    bool     temp_en;
    /// AFH reporting enable
    bool     reporting_en;
    /// AFH lmp_ch_class PDU waiting for Tx confirmation - used to control the flow of lmp_ch_class PDUs
    bool     lmp_ch_class_pending;
};

/// Simple pairing data
/// P192 keys only used 24 first bytes of each arrays
typedef struct lc_sp_data
{
    /// Private key used for pairing
    /// for P192 - only first 24 bytes of key are valid
    priv_key_256_t    priv_key;
    /// Public key used for pairing
    /// for P192 - only first 24 bytes of each coordinate are valid
    pub_key_256_t     local_pub_key;
    /// Received remote public key
    /// for P192 - only first 24 bytes of each coordinate are valid
    pub_key_256_t     remote_pub_key;
    /// Computed DH Key
    /// for P192 - only first 24 bytes of DH-Key are valid
    dh_key_256_t      dh_key;
    /// Local OOB rand data generated pairing
    struct randomizer local_oob_rand;
} lc_sp_data_t;

/// LC Simple Pairing Structure
struct lc_simple_pairing_tag
{
    /// Environment to perform DH Key computations
    lc_sp_data_t *p_data;
    /// pass key
    uint32_t      Passkey;
    /// local random number
    struct byte16 LocRandN;
    /// remote random number
    struct byte16 RemRandN;
    /// local commitment value
    struct byte16 LocCommitment;
    /// remote commitment value
    struct byte16 RemCommitment;
    /// DH key check key
    struct ltk    DHKeyCheck;
    /// local IO capability
    struct io_capability IOCap_loc;
    /// remote IO capability
    struct io_capability IOCap_rem;
    /// encapsulated PDU counter
    uint8_t EncapPduCtr;
    /// simple pairing phase1 failure
    uint8_t SPPhase1Failed;
    /// Used to know state of DH-KEY computation (see #lc_dh_key_compute_state)
    uint8_t dh_key_compute_state;
    /// simple pairing transaction id
    uint8_t SpTId;
    /// simple pairing initiator
    bool SPInitiator;
    /// Secure Connections (true if supported by both controllers and both Hosts)
    bool sec_con;
};

/// LC EPR Structure
struct lc_epr_tag
{
    /// epr enable
    bool    on;
    /// epr role switch
    bool    rsw;
    /// role switch error code
    uint8_t rswerror;
    /// change connection link key epr enable
    bool    cclk;
};

/// LC util local transaction detail
struct lc_local_trans_details_tag
{
    /// lmp opcode
    uint8_t Opcode;
    /// lmp extended opcode
    uint8_t OpcodeExt;
    /// boolean if in used
    uint8_t InUse;
};

#if PCA_SUPPORT
/// LC piconet clock adjust detail
struct lc_pca_tag
{
    bool activated;
    uint8_t clk_adj_id;
};
#endif // PCA_SUPPORT

/// LC SAM Pattern detail
struct lc_sam_pattern
{
    ///Num slots available for tx (slots)
    uint16_t n_tx_slots;
    ///Num slots available for rx (slots)
    uint16_t n_rx_slots;
    ///SAM Submaps
    struct sam_submaps submaps;
    ///Tsam-sm (slots)
    uint8_t t_sam_sm;
    ///Nsam-sm (slots)
    uint8_t n_sam_sm;
    ///Num ext submap
    uint8_t n_ex_sm;
};

/// LC SAM Structure
struct lc_sam_tag
{
    /// SAM update instant for pending SAM switch (CLK[27:1])
    uint32_t instant;
    /// Tsam interval in a pending SAM pattern (slots)
    uint16_t t_sam;
    /// Num tx slots in a pending SAM pattern (slots)
    uint16_t n_tx_slots;
    /// Num rx slots in a pending SAM pattern (slots)
    uint16_t n_rx_slots;
    /// active local Tsam interval (slots)
    uint16_t loc_t_sam_av;
    /// active remote Tsam interval (slots)
    uint16_t rem_t_sam_av;
    /// SAM remote type0 submap
    struct sam_type0_submap rem_submap;
    /// defined patterns from remote device
    struct lc_sam_pattern rem_pattern[SAM_INDEX_MAX];
    /// Remote SAM index
    uint8_t rem_idx;
    /// local SAM index
    uint8_t loc_idx;
    /// local SAM index pending acceptance
    uint8_t loc_idx_wait_cfm;
    /// Tsam_sm submap interval in a pending SAM pattern (slots)
    uint8_t t_sam_sm;
    /// Local SAM tx availability (proportion of 255 max)
    uint8_t loc_tx_av;
    /// Local SAM rx availability (proportion of 255 max)
    uint8_t loc_rx_av;
    /// Remote SAM tx availability (proportion of 255 max)
    uint8_t rem_tx_av;
    /// Remote SAM tx availability (proportion of 255 max)
    uint8_t rem_rx_av;
    /// Remote type0 submap is available
    bool loc_submap0_av;
    /// SAM Patterns available
    bool loc_pattern_av[SAM_INDEX_MAX];
    /// Remote type0 submap is available
    bool rem_submap0_av;
    /// SAM Patterns available
    bool rem_pattern_av[SAM_INDEX_MAX];
    /// a configuration sequence of SAM maps is ongoing
    bool config_mode;
};

/// LC environment structure
struct lc_env_tag
{
    /// link data structure
    struct  lc_link_tag link;
    /// information data structure
    struct  lc_info_tag info;
    /// encryption data structure
    struct  lc_enc_tag enc;
    /// test mode data structure
    struct  lc_test_mode_tag tst_mode;
    /// afh data structure
    struct  lc_afh_tag afh;
    /// simple pairing data structure
    struct  lc_simple_pairing_tag sp;
    /// encryption pause resume data structure
    struct  lc_epr_tag epr;
    /// request data structure
    struct  lc_req_tag req;
    /// local transaction details
    struct lc_local_trans_details_tag local_trans_details;
#if (MAX_NB_SYNC > 0)
    /// Host parameters for SCO setup
    struct lc_sco_host_params_tag sco;
#endif // (MAX_NB_SYNC > 0)
#if PCA_SUPPORT
    /// Piconet clock adjust details
    struct lc_pca_tag pca;
#endif // PCA_SUPPORT
    /// SAM configuration
    struct lc_sam_tag sam_info;
};

/// Format of a LMP message handler function
typedef int (*lc_lmp_msg_hdl_func_t)(void const *param, ke_task_id_t const dest_id);

/// LMP descriptor structure
struct lmp_desc_tag
{
    /// LMP handler function
    lc_lmp_msg_hdl_func_t handler;

    /// Parameters format string
    void *fmt;

    /// Information bit field (@see enum lmp_info_fields)
    uint8_t info;
};


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LMP packets descriptors
extern const struct lmp_desc_tag lmp_desc_tab[];
extern const struct lmp_desc_tag lmp_ext_desc_tab[];

/// LC task descriptor
extern const struct ke_task_desc TASK_DESC_LC;

/// LC task message handlers table
extern const struct ke_state_handler lc_default_handler;

/// LC task state
extern ke_state_t lc_state[LC_IDX_MAX];

/// LC environment variable
extern struct lc_env_tag *lc_env[MAX_NB_ACTIVE_ACL];


/*
 * INLINE FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Convert a negotiated offset to a local offset for an activity
 *
 * @param[in] offset   Offset negotiated at LMP (in half-slots)
 * @param[in] interval Interval of the activity (in half-slots)
 * @param[in] clk      Local clock (in half-slots)
 * @param[in] clk_off  Clock offset between local clock and master clock (in half-slots)
 * @param[in] init     Initialization method (0: init 1 / 1: init 2), from timing control flags
 *
 * @return Offset of the local clock (in half-slots)
 ****************************************************************************************
 */
__INLINE uint16_t lc_offset_lmp_to_local(uint16_t offset, uint16_t interval, uint32_t clk, uint32_t clk_off, uint8_t init)
{
    // Current master clock
    uint32_t master_clock = CLK_ADD_2(clk, clk_off);
    int32_t diff = offset - CO_MOD((master_clock ^ (init << 27)), interval);

    // Find an anchor point on local clock
    uint32_t anchor = CLK_ADD_2(clk, diff);

    // Derive the offset
    return (CO_MOD(anchor, interval));
}

/**
 ****************************************************************************************
 * @brief Convert a local offset to a negotiated offset for an activity
 *
 * @param[in] offset   Offset of the local clock (in half-slots)
 * @param[in] interval Interval of the activity (in half-slots)
 * @param[in] clk      Local clock (in half-slots)
 * @param[in] clk_off  Clock offset between local clock and master clock (in half-slots)
 * @param[in] init     Initialization method (0: init 1 / 1: init 2), from timing control flags
 *
 * @return Offset negotiated at LMP (in half-slots)
 ****************************************************************************************
 */
__INLINE uint16_t lc_offset_local_to_lmp(uint16_t offset, uint16_t interval, uint32_t clk, uint32_t clk_off, uint8_t init)
{
    // Find an anchor point on local clock
    int32_t diff = offset - CO_MOD(clk, interval);
    uint32_t anchor = CLK_ADD_2(clk, diff);

    // Anchor point on master clock
    uint32_t master_clock = CLK_ADD_2(anchor, clk_off);

    // Derive the LMP offset
    return (CO_MOD((master_clock ^ (init << 27)), interval));
}


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function sets the local transaction value
 *
 * @param[in] LinkId            Linked identifier
 * @param[in] Opcode            Opcode
 * @param[in] OpcodeExt         Extended opcode
 * @param[in] Mode              Mode
 ****************************************************************************************
 */
void lc_util_set_loc_trans_coll(uint8_t LinkId, uint8_t Opcode, uint8_t OpcodeExt, uint8_t Mode);

/// Send HCI CS event
void lc_cmd_stat_send(uint16_t opcode, uint8_t status);

/**
 ****************************************************************************************
 * @brief This function updates the channel map
 *
 * @param[in] link_id    Link identifier
 * @param[in] ch_map     Channel map: combination of local channel map and HOST classification
 ****************************************************************************************
 */
void lc_ch_map_update(uint8_t link_id, struct bt_ch_map *ch_map);

/**
****************************************************************************************
* @brief Get the SAM interval on a link.
* If both local and remote SAM maps are in operation, the larger interval is returned.
*
* @param[in] link_id    Link identifier
*
* @return interval (slots)
****************************************************************************************
*/
uint16_t lc_sam_intv_get(uint8_t link_id);

/**
****************************************************************************************
* @brief Get the local SAM offset active on a link.
*
* @param[in] link_id    Link identifier
*
* @return d_sam offset (slots)
****************************************************************************************
*/
uint16_t lc_sam_loc_offset_get(uint8_t link_id);

/**
****************************************************************************************
* @brief Get the remote SAM offset active on a link.
*
* @param[in] link_id    Link identifier
*
* @return d_sam offset (slots)
****************************************************************************************
*/
uint16_t lc_sam_rem_offset_get(uint8_t link_id);

/**
****************************************************************************************
* @brief Gets an available slot from the SAM map within specified range.
* The offsets refer to a period which is equal to or a multiple of the SAM interval.
* If a suitable offset is not found, the lowest value of specified range is returned.
*
* @param[in] link_id        Link identifier
* @param[in] offset_min     Lower bound of d_sam offsets to search (slots)
* @param[in] offset_max     Upper bound of d_sam offsets to search (slots)
*
* @return d_sam offset (slots)
****************************************************************************************
*/
uint16_t lc_sam_slot_av_get(uint8_t link_id, uint16_t offset_min, uint16_t offset_max);

/**
 ****************************************************************************************
 * @brief Handle authentication completion.
 *
 * @param[in] pid        process identifier.
 * @param[in] reason     authentication process status.
 *
 ****************************************************************************************
 */
void lc_auth_cmp(ke_task_id_t pid, uint8_t reason);

/**
 ****************************************************************************************
 * @brief Handle link key calculation.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_calc_link_key(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief Handle change packet type continuation.
 *
 * @param[in] pid        process identifier.
 * @param[in] status     authentication process status.
 *
 ****************************************************************************************
 */
void lc_chg_pkt_type_cont(ke_task_id_t pid, uint8_t status);

/**
 ****************************************************************************************
 * @brief Handle change of packet type complete.
 *
 * @param[in] pid        process identifier.
 * @param[in] status     change packet type status.
 *
 ****************************************************************************************
 */
void lc_chg_pkt_type_cmp(ke_task_id_t pid, uint8_t status);

/**
 ****************************************************************************************
 * @brief Handle change of packet type retry.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_chg_pkt_type_retry(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief Start AFH on the link (master only).
 *
 * @param[in] pid        process identifier.
 ****************************************************************************************
 */
void lc_afh_start(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief Handle combination key server.
 *
 * @param[in] pid        process identifier.
 * @param[in] key        pointer to the unit key
 *
 ****************************************************************************************
 */
void lc_comb_key_svr(ke_task_id_t pid, struct ltk *key);

/**
 ****************************************************************************************
 * @brief Handle connection completion.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_con_cmp(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle sending of connection complete event.
 *
 * @param[in] pid        process identifier.
 * @param[in] status     status
 *
 ****************************************************************************************
 */
void lc_con_cmp_evt_send(ke_task_id_t pid, uint8_t status);
/**
 ****************************************************************************************
 * @brief Handle DHKey SP stage.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_try_start_auth_stage2(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Process encryption key refresh request.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_enc_key_refresh(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Check transaction collision.
 *
 * @param[in] pid        process identifier.
 * @param[in] enc_mode   encryption mode.
 *
 ****************************************************************************************
 */
void lc_end_chk_colli(ke_task_id_t pid, uint8_t enc_mode);
/**
 ****************************************************************************************
 * @brief Handle encryption completion.
 *
 * @param[in] pid        process identifier.
 * @param[in] reason     reason
 *
 ****************************************************************************************
 */
void lc_enc_cmp(ke_task_id_t pid, uint8_t reason);
/**
 ****************************************************************************************
 * @brief Handle encryption pause and resume completion.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_epr_cmp(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief Handle encryption pause and resume on change connection link key.
 *
 * @param[in] pid        process identifier.
 * @param[in] reason     reason
 *
 ****************************************************************************************
 */
void lc_epr_change_lk(ke_task_id_t pid, uint8_t reason);
/**
 ****************************************************************************************
 * @brief Handle encryption pause and resume as responder.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_epr_resp(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Clean up phase for epr during role switch.
 *
 * @param[in] pid        process identifier.
 * @param[in] status     status
 *
 ****************************************************************************************
 */
void lc_rsw_done(ke_task_id_t pid, uint8_t status);
/**
 ****************************************************************************************
 * @brief Handle successful encryption pause and resume during role switch
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_epr_rsw_cmp(ke_task_id_t pid, uint8_t status);
/**
 ****************************************************************************************
 * @brief Initializes calculation of F3.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_init_calc_f3(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle start of mutual authentication for initiator.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_init_start_mutual_auth(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle passkey loop for initiator.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_init_passkey_loop(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle encryption pause and resume initiator side.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_initiator_epr(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle key exchange.
 *
 * @param[in] pid        process identifier.
 * @param[in] reason     reason
 *
 ****************************************************************************************
 */
void lc_key_exch_end(ke_task_id_t pid, uint8_t reason);
/**
 ****************************************************************************************
 * @brief Handle legacy pairing - non-simple pairing
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_legacy_pair(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle local role switch processing.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_local_switch(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Manage maximum packet slot.
 *
 * @param[in] pid        process identifier.
 * @param[in] max_slot   maximum slot
 *
 ****************************************************************************************
 */
void lc_max_slot_mgt(ke_task_id_t pid, uint8_t max_slot);
/**
 ****************************************************************************************
 * @brief Handle mutual authentication last part.
 *
 * @param[in] pid        process identifier.
 * @param[in] reason     reason
 *
 ****************************************************************************************
 */
void lc_mutual_auth_end2(ke_task_id_t pid, uint8_t reason);
/**
 ****************************************************************************************
 * @brief Handle mutual authentication first part.
 *
 * @param[in] pid        process identifier.
 * @param[in] reason     reason.
 *
 ****************************************************************************************
 */
void lc_mutual_auth_end(ke_task_id_t pid, uint8_t reason);
/**
 ****************************************************************************************
 * @brief Handle master key process.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_mst_key(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Function that sends master key to peer.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_mst_send_mst_key(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle master QOS.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_mst_qos_done(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle pairing procedure
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_pairing_cont(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle passkey.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_passkey_comm(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle packet type table request procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_ptt(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Completion of the packet type table request.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_ptt_cmp(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle quality of service setup.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_qos_setup(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Read remote name processing.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_rd_rem_name(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle remote switch procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_rem_switch(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle remote encryption procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_rem_enc(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle remote name request processing.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_rem_name_cont(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief Compute signed responses and ACO
 *
 * @param[in] pid        process identifier.
 ****************************************************************************************
 */
void lc_sec_auth_compute_sres(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief Handle responder secure authentication
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_resp_sec_auth(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle responder authentication
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_resp_auth(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle responder oob wait nonce
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_resp_oob_wait_nonce(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle responder oob nonce.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_resp_oob_nonce(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle responder calculate F3.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_resp_calc_f3(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle connection release
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_release(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_restart_enc(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle restart of encryption procedure.
 *
 * @param[in] pid        process identifier.
 * @param[in] reason     reason
 *
 ****************************************************************************************
 */
void lc_restart_enc_cont(ke_task_id_t pid, uint8_t reason);
/**
 ****************************************************************************************
 * @brief Handle restoration of link supervision time out.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_restore_to(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle role switch for devices that does not support EPR.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_rsw_non_epr_back(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle start of LMP time out.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_start_lmp_to(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle start of encryption.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_start_enc(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle start of encryption key size.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_start_enc_key_size(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle start of key exchange procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_start_key_exch(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle start of pass key procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_start_passkey(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle start of pass key loop procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_start_passkey_loop(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle start of out-of-band procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_start_oob(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle numerical comparison for responder.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_resp_num_comp(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle stop of adaptive frequency hopping reporting.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_stop_afh_reporting(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle restoration of adaptive frequency reporting.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_restore_afh_reporting(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle stop encryption procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_stop_enc(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief Skip higher layer OOB request
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_skip_hl_oob_req(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Send encryption mode request.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_send_enc_mode(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Send semi-permanent key complete event.
 *
 * @param[in] pid        process identifier.
 * @param[in] reason     reason
 *
 ****************************************************************************************
 */
void lc_semi_key_cmp(ke_task_id_t pid, uint8_t reason);
/**
 ****************************************************************************************
 * @brief Start detachment procedure.
 *
 * @param[in] reason     reason.
 * @param[in] pid        process identifier.
 ****************************************************************************************
 */
void lc_detach(ke_task_id_t pid, uint8_t reason);
/**
 ****************************************************************************************
 * @brief Handle switch complete procedure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_switch_cmp(ke_task_id_t pid, uint8_t status);
/**
 ****************************************************************************************
 * @brief Handle simple pairing failure.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
void lc_sp_fail(ke_task_id_t pid);
/**
 ****************************************************************************************
 * @brief Handle packet type check.
 *
 * @param[in] pid        process identifier.
 * @param[in] idx        index
 *
 ****************************************************************************************
 */
void lc_packet_type(ke_task_id_t pid, int idx);
/**
 ****************************************************************************************
 * @brief Handle local authentication procedure.
 *
 * @param[in] pid        process identifier.
 * @param[in] idx        index instance
 *
 ****************************************************************************************
 */
void lc_loc_auth(ke_task_id_t pid, int idx);
/**
 ****************************************************************************************
 * @brief Handle pairing procedure.
 *
 * @param[in] pid        process identifier.
 * @param[in] idx        index instance
 *
 ****************************************************************************************
 */
void lc_pair(ke_task_id_t pid, int idx);
/**
 ****************************************************************************************
 * @brief Handle exchange of extended features.
 *
 * @param[in] pid        process identifier.
 * @param[in] idx        index instance
 * @param[in] page_nb    page number
 *
 ****************************************************************************************
 */
void lc_ext_feat(ke_task_id_t pid, int idx, uint8_t page_nb);
/**
 ****************************************************************************************
 * @brief Function to check the connection sequence. Ensure optional procedures
 *        during connection establishment are executed before sending setup
 *        complete to peer.
 *
 * @param[in] pid        process identifier.
 *
 ****************************************************************************************
 */
bool lc_conn_seq_done(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief Send encryption change event indication.
 *
 * @param[in] idx          Link identifier
 * @param[in] status       status
 ****************************************************************************************
 */
void lc_send_enc_chg_evt(uint8_t idx, uint8_t status);

void lc_version(ke_task_id_t pid, int idx);
void lc_locepr_rsw(ke_task_id_t pid);
void lc_locepr_lkref(ke_task_id_t pid);
void lc_resp_pair(ke_task_id_t pid, int idx);
void lc_feat(ke_task_id_t pid, int idx);
void lc_hl_connect(ke_task_id_t pid, int idx);

/**
 ****************************************************************************************
 * @brief This functions is used to free the buffer pointed to by the index.
 *
 * @param[in] link_id            ACL Link Identifier
 * @param[in] reason             Reason of the simple pairing end
 ****************************************************************************************
 */
void lc_sp_end(uint8_t link_id, uint8_t reason);


/**
****************************************************************************************
* @brief This function disables SAM if enabled on the specified link
*
 * @param[in] link_id            ACL Link Identifier
****************************************************************************************
*/
void lc_sam_disable(uint8_t link_id);

/// @} LC

#endif // LC_INT_H_
