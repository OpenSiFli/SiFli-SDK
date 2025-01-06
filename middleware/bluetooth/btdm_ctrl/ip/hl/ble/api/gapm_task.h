/**
 ****************************************************************************************
 *
 * @file gapm_task.h
 *
 * @brief Generic Access Profile Manager Task Header.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


#ifndef _GAPM_TASK_H_
#define _GAPM_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup GAPM_TASK Generic Access Profile Manager Task
 * @ingroup GAPM
 * @brief  Handles ALL messages to/from GAP Manager block.
 *
 * It handles messages from lower and higher layers not related to an ongoing connection.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_task.h" // Task definitions
#include "gap.h"
#include "co_math.h"
#include "ke_msg.h"
/*
 * DEFINES
 ****************************************************************************************
 */

/// Offset for List Management Operation Codes
#define GAPM_OP_OFFSET_LIST_MGMT  (0x90)
/// Offset for Extended Air Operation Codes
#define GAPM_OP_OFFSET_EXT_AIR    (0xA0)

#define KEY_ID_LEN 4
/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// GAP Manager Message Interface
/*@TRACE*/
enum gapm_msg_id
{
    /* Default event */
    /// Command Complete event
    GAPM_CMP_EVT = TASK_FIRST_MSG(TASK_ID_GAPM),

    /* Default commands */
    /// Reset link layer and the host command
    GAPM_RESET_CMD = GAPM_CMP_EVT + 2,

    /* Device Configuration */
    /// Set device configuration command
    GAPM_SET_DEV_CONFIG_CMD = GAPM_RESET_CMD + 2,
    /// Set device channel map
    GAPM_SET_CHANNEL_MAP_CMD,

    /* Local device information */
    /// Get local device info command
    GAPM_GET_DEV_INFO_CMD,
    /// Local device version indication event
    GAPM_DEV_VERSION_IND,
    /// Local device BD Address indication event
    GAPM_DEV_BDADDR_IND,
    /// Advertising channel Tx power level
    GAPM_DEV_ADV_TX_POWER_IND,
    /// Indication containing information about memory usage.
    GAPM_DBG_MEM_INFO_IND,

    /// Name of peer device indication
    GAPM_PEER_NAME_IND = GAPM_DBG_MEM_INFO_IND + 8,

    /* Security / Encryption Toolbox */
    /// Resolve address command
    GAPM_RESOLV_ADDR_CMD = GAPM_PEER_NAME_IND + 2,
    /// Indicate that resolvable random address has been solved
    GAPM_ADDR_SOLVED_IND,
    /// Generate a random address.
    GAPM_GEN_RAND_ADDR_CMD,
    /// Use the AES-128 block in the controller
    GAPM_USE_ENC_BLOCK_CMD,
    ///  AES-128 block result indication
    GAPM_USE_ENC_BLOCK_IND,
    /// Generate a 8-byte random number
    GAPM_GEN_RAND_NB_CMD,
    /// Random Number Indication
    GAPM_GEN_RAND_NB_IND,

    /* Profile Management */
    /// Create new task for specific profile
    GAPM_PROFILE_TASK_ADD_CMD,
    /// Inform that profile task has been added.
    GAPM_PROFILE_ADDED_IND,

    /// Indicate that a message has been received on an unknown task
    GAPM_UNKNOWN_TASK_IND,

    /* Data Length Extension */
    /// Suggested Default Data Length indication
    GAPM_SUGG_DFLT_DATA_LEN_IND,
    /// Maximum Data Length indication
    GAPM_MAX_DATA_LEN_IND,

    /// Link RSSI and channl map notify periodically indicaiton
    GAPM_DBG_RSSI_NOTIFY_IND,
    /* Resolving list for controller-based privacy*/
    /// Resolving address list address indication
    GAPM_RAL_ADDR_IND = GAPM_MAX_DATA_LEN_IND + 3,

    /* Set new IRK */
    /// Modify current IRK
    GAPM_SET_IRK_CMD,

    /* LE Protocol/Service Multiplexer Management */
    /// Register a LE Protocol/Service Multiplexer command
    GAPM_LEPSM_REGISTER_CMD,
    /// Unregister a LE Protocol/Service Multiplexer command
    GAPM_LEPSM_UNREGISTER_CMD,

    /* LE Test Mode */
    /// Control of the test mode command
    GAPM_LE_TEST_MODE_CTRL_CMD,
    /// Indicate end of test mode
    GAPM_LE_TEST_END_IND,

    /// Provide statistic information about ISO exchange
    GAPM_ISO_STAT_IND,

    /* Secure Connections */
    /// Request to provide DH Key
    GAPM_GEN_DH_KEY_CMD,
    /// Indicates the DH Key computation is complete and available
    GAPM_GEN_DH_KEY_IND,
    /// Retrieve Public Key
    GAPM_GET_PUB_KEY_CMD,
    /// Indicates the Public Key Pair value
    GAPM_PUB_KEY_IND,

    /* ************************************************ */
    /* ------------ NEW COMMANDS FOR BLE 5 ------------ */
    /* ************************************************ */

    /* List Management Operations */
    /// Get local or peer address
    /// @see struct gapm_get_ral_addr_cmd
    GAPM_GET_RAL_ADDR_CMD = TASK_FIRST_MSG(TASK_ID_GAPM) + GAPM_OP_OFFSET_LIST_MGMT,
    /// Set content of either white list or resolving list or periodic advertiser list
    /// @see struct gapm_list_set_wl_cmd
    /// @see struct gapm_list_set_ral_cmd
    /// @see struct gapm_list_set_pal_cmd
    GAPM_LIST_SET_CMD,
    /// Indicate size of list indicated in GAPM_GET_DEV_CONFIG_CMD message
    /// @see struct gapm_list_size_ind
    GAPM_LIST_SIZE_IND,

    /* Extended Air Operations */
    /// Create an advertising, a scanning, an initiating or a periodic synchronization activity
    /// @see struct gapm_activity_create_cmd
    /// @see struct gapm_activity_create_adv_cmd
    GAPM_ACTIVITY_CREATE_CMD = TASK_FIRST_MSG(TASK_ID_GAPM) + GAPM_OP_OFFSET_EXT_AIR,
    /// Start a previously created activity
    /// @see struct gapm_activity_start_cmd
    GAPM_ACTIVITY_START_CMD,
    /// Stop either a given activity or all existing activities
    /// @see struct gapm_activity_stop_cmd
    GAPM_ACTIVITY_STOP_CMD,
    /// Delete either a given activity or all existing activities
    /// @see struct gapm_activity_delete_cmd
    GAPM_ACTIVITY_DELETE_CMD,
    /// Indicate that an activity has well been created
    /// @see struct gapm_activity_create_ind
    GAPM_ACTIVITY_CREATED_IND,
    /// Indicate that an activity has been stopped and can be restarted
    /// @see struct gapm_activity_stopped_ind
    GAPM_ACTIVITY_STOPPED_IND,
    /// Set either advertising data or scan response data or periodic advertising data
    /// @see struct gapm_set_adv_data_cmd
    GAPM_SET_ADV_DATA_CMD,
    /// Indicate reception of an advertising report (periodic or not), a scan response report
    /// @see struct gapm_ext_adv_report_ind
    GAPM_EXT_ADV_REPORT_IND,
    /// Indicate reception of a scan request
    /// @see struct gapm_scan_request_ind
    GAPM_SCAN_REQUEST_IND,
    /// Indicate that synchronization has been successfully established with a periodic advertiser
    /// @see struct gapm_sync_established_ind
    GAPM_SYNC_ESTABLISHED_IND,
    /// Indicate maximum advertising data length supported by controller
    /// @see struct gapm_max_adv_data_len_ind
    GAPM_MAX_ADV_DATA_LEN_IND,
    /// Indicate number of available advertising sets
    /// @see struct gapm_nb_adv_sets_ind
    GAPM_NB_ADV_SETS_IND,
    /// Indicate the transmit powers supported by the controller
    /// @see struct gapm_dev_tx_power_ind
    GAPM_DEV_TX_PWR_IND,
    /// Indicate the RF path compensation values
    /// @see struct gapm_dev_rf_path_comp_ind
    GAPM_DEV_RF_PATH_COMP_IND,
    /// Indication to the task that sends the unknown message
    /// @see struct gapm_unknown_msg_ind
    GAPM_UNKNOWN_MSG_IND,
    /* ************************************************ */
    /* -------------- Internal usage only ------------- */
    /* ************************************************ */
    /// Message received to unknown task received
    GAPM_UNKNOWN_TASK_MSG,

    /* Internal messages for timer events, not part of API */
    /// Address renewal timeout indication
    GAPM_ADDR_RENEW_TO_IND,
    /// Automatic connection establishment timeout indication
    GAPM_AUTO_CONN_TO_IND,

    GAPM_AES_H6_CMD,
    GAPM_AES_H6_IND,
    /* Addresses Management */
    /// Renew random private addresses
    /// @see struct gapm_addr_renew_cmd
    GAPM_ADDR_RENEW_CMD,

    GAPM_ASSERT_IND,

    GAPM_SET_DEFAULT_REQUESTER,

    GAPM_CONFIG_RESOLUTION,
};


/// GAP Manager operation type - application interface
/*@TRACE*/
enum gapm_operation
{
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation.
    GAPM_NO_OP                                     = 0x00,

    /* Default operations                               */
    /* ************************************************ */
    /// Reset BLE subsystem: LL and HL.
    GAPM_RESET,

    /* Configuration operations                         */
    /* ************************************************ */
    /// Set device configuration
    GAPM_SET_DEV_CONFIG = GAPM_RESET + 2,
    /// Set device channel map
    GAPM_SET_CHANNEL_MAP,

    /* Retrieve device information                      */
    /* ************************************************ */
    /// Get Local device version
    GAPM_GET_DEV_VERSION,
    /// Get Local device BD Address
    GAPM_GET_DEV_BDADDR,
    /// Get device advertising power level
    GAPM_GET_DEV_ADV_TX_POWER,
    /// Get White List Size.
    GAPM_GET_WLIST_SIZE,

    /* Security / Encryption Toolbox                    */
    /* ************************************************ */
    /// Resolve device address
    GAPM_RESOLV_ADDR = GAPM_GET_WLIST_SIZE + 15,
    /// Generate a random address
    GAPM_GEN_RAND_ADDR,
    /// Use the controller's AES-128 block
    GAPM_USE_ENC_BLOCK,
    /// Generate a 8-byte random number
    GAPM_GEN_RAND_NB,

    /* Profile Management                               */
    /* ************************************************ */
    /// Create new task for specific profile
    GAPM_PROFILE_TASK_ADD,

    /* DEBUG                                            */
    /* ************************************************ */
    /// Get memory usage
    GAPM_DBG_GET_MEM_INFO,
    /// Perform a platform reset
    GAPM_PLF_RESET,

    /* Data Length Extension                            */
    /* ************************************************ */
    /// Set Suggested Default LE Data Length
    GAPM_SET_SUGGESTED_DFLT_LE_DATA_LEN,
    /// Get Suggested Default LE Data Length
    GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN,
    /// Get Maximum LE Data Length
    GAPM_GET_MAX_LE_DATA_LEN,

    /* Operation on Resolving List                      */
    /* ************************************************ */
    /// Get resolving address list size
    GAPM_GET_RAL_SIZE,
    /// Get resolving local address
    GAPM_GET_RAL_LOC_ADDR,
    /// Get resolving peer address
    GAPM_GET_RAL_PEER_ADDR,

    /* Change current IRK                               */
    /* ************************************************ */
    /// Set IRK
    GAPM_SET_IRK = GAPM_GET_RAL_PEER_ADDR + 5,

    /* LE Protocol/Service Multiplexer management       */
    /* ************************************************ */
    /// Register a LE Protocol/Service Multiplexer
    GAPM_LEPSM_REG,
    /// Unregister a LE Protocol/Service Multiplexer
    GAPM_LEPSM_UNREG,

    /* LE Direct Test Mode                              */
    /* ************************************************ */
    /// Stop the test mode
    GAPM_LE_TEST_STOP,
    /// Start RX Test Mode
    GAPM_LE_TEST_RX_START,
    /// Start TX Test Mode
    GAPM_LE_TEST_TX_START,

    /* Secure Connection                                */
    /* ************************************************ */
    /// Generate DH_Key
    GAPM_GEN_DH_KEY,
    /// Retrieve Public Key
    GAPM_GET_PUB_KEY,

    /* List Management                                  */
    /* ************************************************ */
    /// Set content of white list
    GAPM_SET_WL = GAPM_NO_OP + GAPM_OP_OFFSET_LIST_MGMT,
    /// Set content of resolving list
    GAPM_SET_RAL,
    /// Set content of periodic advertiser list
    GAPM_SET_PAL,
    /// Get white list size
    //GAPM_GET_WHITE_LIST_SIZE,
    /// Get resolving list size
    //GAPM_GET_RAL_SIZE,
    /// Get periodic advertiser list size
    GAPM_GET_PAL_SIZE = GAPM_SET_PAL + 3,

    /* Air Operations                                   */
    /* ************************************************ */
    /// Create advertising activity
    GAPM_CREATE_ADV_ACTIVITY = GAPM_NO_OP + GAPM_OP_OFFSET_EXT_AIR,
    /// Create scanning activity
    GAPM_CREATE_SCAN_ACTIVITY,
    /// Create initiating activity
    GAPM_CREATE_INIT_ACTIVITY,
    /// Create periodic synchronization activity
    GAPM_CREATE_PERIOD_SYNC_ACTIVITY,
    /// Start an activity
    GAPM_START_ACTIVITY,
    /// Stop an activity
    GAPM_STOP_ACTIVITY,
    /// Stop all activities
    GAPM_STOP_ALL_ACTIVITIES,
    /// Delete an activity
    GAPM_DELETE_ACTIVITY,
    /// Delete all activities
    GAPM_DELETE_ALL_ACTIVITIES,
    /// Set advertising data
    GAPM_SET_ADV_DATA,
    /// Set scan response data
    GAPM_SET_SCAN_RSP_DATA,
    /// Set periodic advertising data
    GAPM_SET_PERIOD_ADV_DATA,
    /// Get number of available advertising sets
    GAPM_GET_NB_ADV_SETS,
    /// Get maximum advertising data length supported by the controller
    GAPM_GET_MAX_LE_ADV_DATA_LEN,
    /// Get minimum and maximum transmit powers supported by the controller
    GAPM_GET_DEV_TX_PWR,
    /// Get the RF Path Compensation values used in the TX Power Level and RSSI calculation
    GAPM_GET_DEV_RF_PATH_COMP,
    /// INTERNAL OPERATION - Renew random addresses
    GAPM_RENEW_ADDR,
    /// INTERNAL OPERATION -  Start all activities
    GAPM_START_ALL_ACTIVITIES,
};

/// Own BD address source of the device
enum gapm_own_addr
{
    /// Public or Private Static Address according to device address configuration
    GAPM_STATIC_ADDR,
    /// Generated resolvable private random address
    GAPM_GEN_RSLV_ADDR,
    /// Generated non-resolvable private random address
    GAPM_GEN_NON_RSLV_ADDR,
};

/// Device Attribute write permission requirement
enum gapm_write_att_perm
{
    /// Disable write access
    GAPM_WRITE_DISABLE     = 0,
    /// Enable write access - no authentication required
    GAPM_WRITE_NO_AUTH     = 1,
    /// Write access requires unauthenticated link
    GAPM_WRITE_UNAUTH      = 2,
    /// Write access requires authenticated link
    GAPM_WRITE_AUTH        = 3,
    /// Write access requires secure connected link
    GAPM_WRITE_SEC_CON     = 4
};

/// Attribute database configuration
///   15   14   13   12   11   10    9    8    7    6    5    4    3    2    1    0
/// +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
/// | DBG|DBGT|          RFU                | SC |PCP |   APP_PERM   |   NAME_PERM  |
/// +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
/// - Bit [0-2] : Device Name write permission requirements for peer device (@see gapm_write_att_perm)
/// - Bit [3-5] : Device Appearance write permission requirements for peer device (@see gapm_write_att_perm)
/// - Bit [6]   : Slave Preferred Connection Parameters present
/// - Bit [7]   : Service change feature present in GATT attribute database.
/// - Bit [8-13]: Reserved
/// - Bit [14]  : if Debug Mode enabled, forward all L2cap traffic to application
/// - Bit [15]  : Enable Debug Mode
enum gapm_att_cfg_flag
{
    /// Device Name write permission requirements for peer device (@see gapm_write_att_perm)
    GAPM_MASK_ATT_NAME_PERM            = 0x0007,
    GAPM_POS_ATT_NAME_PERM             = 0x00,
    /// Device Appearance write permission requirements for peer device (@see gapm_write_att_perm)
    GAPM_MASK_ATT_APPEARENCE_PERM      = 0x0038,
    GAPM_POS_ATT_APPEARENCE_PERM       = 0x03,
    /// Slave Preferred Connection Parameters present in GAP attribute database.
    GAPM_MASK_ATT_SLV_PREF_CON_PAR_EN  = 0x0040,
    GAPM_POS_ATT_SLV_PREF_CON_PAR_EN   = 0x06,
    /// Service change feature present in GATT attribute database.
    GAPM_MASK_ATT_SVC_CHG_EN           = 0x0080,
    GAPM_POS_ATT_SVC_CHG_EN            = 0x07,
    /// if Debug Mode enabled, forward all L2cap traffic to application
    GAPM_MASK_ATT_DBG_L2CAP_TRAFFIC_EN = 0x4000,
    GAPM_POS_ATT_DBG_L2CAP_TRAFFIC_EN  = 0x0E,
    /// Enable Debug Mode
    GAPM_MASK_ATT_DBG_MODE_EN          = 0x8000,
    GAPM_POS_ATT_DBG_MODE_EN           = 0x0F,
};

/// Pairing mode authorized on the device
///    7    6    5    4    3    2    1    0
/// +----+----+----+----+----+----+----+----+
/// |KGEN|          RFU           | SCP| LP |
/// +----+----+----+----+----+----+----+----+
enum gapm_pairing_mode
{
    /// No pairing authorized
    GAPM_PAIRING_DISABLE  = 0,
    /// Legacy pairing Authorized
    GAPM_PAIRING_LEGACY   = (1 << 0),
    /// Secure Connection pairing Authorized
    GAPM_PAIRING_SEC_CON  = (1 << 1),


    /// Force re-generation of P256 private and public keys
    GAPM_PAIRING_FORCE_P256_KEY_GEN = (1 << 7),
};

/// LE Audio Mode Configuration
///   15   14   13   12   11   10    9    8    7    6    5    4    3    2    1    0
/// +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
/// |                                 RFU                                      | AM0|
/// +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
enum gapm_audio_cfg_flag
{
    /// LE Audio Mode 0 Supported
    GAPM_MASK_AUDIO_AM0_SUP           = 0x0001,
    GAPM_POS_AUDIO_AM0_SUP            = 0x00,
};

/// Security level
///   7   6   5   4   3   2   1   0
/// +---+---+---+---+---+---+---+---+
/// |MI |      RFU      |EKS|SEC_LVL|
/// +---+---+---+---+---+---+---+---+
/// bit[0-1]: Security level requirement (0=NO_AUTH, 1=UNAUTH, 2=AUTH, 3=SEC_CON)
/// bit[2]  : Encryption Key Size length must have 16 bytes
/// bit[7]  : Multi-instantiated task
enum gapm_le_psm_sec_mask
{
    /// bit[0-1]: Security level requirement (0=NO_AUTH, 1=UNAUTH, 2=AUTH, 3=SEC_CON)
    /// bit[2]  : Encryption Key Size length must have 16 bytes
    GAPM_LE_PSM_SEC_LVL_MASK  = 0x07,
    /// bit[7]  : Multi-instantiated task
    GAPM_LE_PSM_MI_TASK_MASK  = 0x80,
};

/// Type of activities that can be created
/*@TRACE*/
enum gapm_actv_type
{
    /// Advertising activity
    GAPM_ACTV_TYPE_ADV = 0,
    /// Scanning activity
    GAPM_ACTV_TYPE_SCAN,
    /// Initiating activity
    GAPM_ACTV_TYPE_INIT,
    /// Periodic synchronization activity
    GAPM_ACTV_TYPE_PER_SYNC,
};

/// Type of advertising that can be created
enum gapm_adv_type
{
    /// Legacy advertising
    GAPM_ADV_TYPE_LEGACY = 0,
    /// Extended advertising
    GAPM_ADV_TYPE_EXTENDED,
    /// Periodic advertising
    GAPM_ADV_TYPE_PERIODIC,
};

/// Advertising report type
enum gapm_adv_report_type
{
    /// Extended advertising report
    GAPM_REPORT_TYPE_ADV_EXT = 0,
    /// Legacy advertising report
    GAPM_REPORT_TYPE_ADV_LEG,
    /// Extended scan response report
    GAPM_REPORT_TYPE_SCAN_RSP_EXT,
    /// Legacy scan response report
    GAPM_REPORT_TYPE_SCAN_RSP_LEG,
    /// Periodic advertising report
    GAPM_REPORT_TYPE_PER_ADV,
};

/// Advertising properties bit field bit positions
enum gapm_adv_prop_pos
{
    /// Indicate that advertising is connectable, reception of CONNECT_REQ or AUX_CONNECT_REQ
    /// PDUs is accepted. Not applicable for periodic advertising.
    GAPM_ADV_PROP_CONNECTABLE_POS     = 0,
    /// Indicate that advertising is scannable, reception of SCAN_REQ or AUX_SCAN_REQ PDUs is
    /// accepted
    GAPM_ADV_PROP_SCANNABLE_POS,
    /// Indicate that advertising targets a specific device. Only apply in following cases:
    ///   - Legacy advertising: if connectable
    ///   - Extended advertising: connectable or (non connectable and non discoverable)
    GAPM_ADV_PROP_DIRECTED_POS,
    /// Indicate that High Duty Cycle has to be used for advertising on primary channel
    /// Apply only if created advertising is not an extended advertising
    GAPM_ADV_PROP_HDC_POS,
    /// Bit 4 is reserved
    GAPM_ADV_PROP_RESERVED_4_POS,
    /// Enable anonymous mode. Device address won't appear in send PDUs
    /// Valid only if created advertising is an extended advertising
    GAPM_ADV_PROP_ANONYMOUS_POS,
    /// Include TX Power in the extended header of the advertising PDU.
    /// Valid only if created advertising is not a legacy advertising
    GAPM_ADV_PROP_TX_PWR_POS,
    /// Include TX Power in the periodic advertising PDU.
    /// Valid only if created advertising is a periodic advertising
    GAPM_ADV_PROP_PER_TX_PWR_POS,
    /// Indicate if application must be informed about received scan requests PDUs
    GAPM_ADV_PROP_SCAN_REQ_NTF_EN_POS,
};

/// Advertising properties bit field bit value
enum gapm_adv_prop
{
    /// Indicate that advertising is connectable, reception of CONNECT_REQ or AUX_CONNECT_REQ
    /// PDUs is accepted. Not applicable for periodic advertising.
    GAPM_ADV_PROP_CONNECTABLE_BIT     = CO_BIT(GAPM_ADV_PROP_CONNECTABLE_POS),
    /// Indicate that advertising is scannable, reception of SCAN_REQ or AUX_SCAN_REQ PDUs is
    /// accepted
    GAPM_ADV_PROP_SCANNABLE_BIT       = CO_BIT(GAPM_ADV_PROP_SCANNABLE_POS),
    /// Indicate that advertising targets a specific device. Only apply in following cases:
    ///   - Legacy advertising: if connectable
    ///   - Extended advertising: connectable or (non connectable and non discoverable)
    GAPM_ADV_PROP_DIRECTED_BIT        = CO_BIT(GAPM_ADV_PROP_DIRECTED_POS),
    /// Indicate that High Duty Cycle has to be used for advertising on primary channel
    /// Apply only if created advertising is not an extended advertising
    GAPM_ADV_PROP_HDC_BIT             = CO_BIT(GAPM_ADV_PROP_HDC_POS),
    /// Bit 4 is reserved
    GAPM_ADV_PROP_RESERVED_4_BIT      = CO_BIT(GAPM_ADV_PROP_RESERVED_4_POS),
    /// Enable anonymous mode. Device address won't appear in send PDUs
    /// Valid only if created advertising is an extended advertising
    GAPM_ADV_PROP_ANONYMOUS_BIT       = CO_BIT(GAPM_ADV_PROP_ANONYMOUS_POS),
    /// Include TX Power in the extended header of the advertising PDU.
    /// Valid only if created advertising is not a legacy advertising
    GAPM_ADV_PROP_TX_PWR_BIT          = CO_BIT(GAPM_ADV_PROP_TX_PWR_POS),
    /// Include TX Power in the periodic advertising PDU.
    /// Valid only if created advertising is a periodic advertising
    GAPM_ADV_PROP_PER_TX_PWR_BIT      = CO_BIT(GAPM_ADV_PROP_PER_TX_PWR_POS),
    /// Indicate if application must be informed about received scan requests PDUs
    GAPM_ADV_PROP_SCAN_REQ_NTF_EN_BIT = CO_BIT(GAPM_ADV_PROP_SCAN_REQ_NTF_EN_POS),
};

/// Advertising discovery mode
enum gapm_adv_disc_mode
{
    /// Mode in non-discoverable
    GAPM_ADV_MODE_NON_DISC = 0,
    /// Mode in general discoverable
    GAPM_ADV_MODE_GEN_DISC,
    /// Mode in limited discoverable
    GAPM_ADV_MODE_LIM_DISC,
    /// Broadcast mode without presence of AD_TYPE_FLAG in advertising data
    GAPM_ADV_MODE_BEACON,
    // for dual mode one-click connection
    GAPM_ADV_MODE_DULMODE,
    // user can set flags in this mode
    GAPM_ADV_MODE_CUSTOMIZE,
    GAPM_ADV_MODE_MAX,
};

/// Scanning Types
enum gapm_scan_type
{
    /// General discovery
    GAPM_SCAN_TYPE_GEN_DISC = 0,
    /// Limited discovery
    GAPM_SCAN_TYPE_LIM_DISC,
    /// Observer
    GAPM_SCAN_TYPE_OBSERVER,
    /// Selective observer
    GAPM_SCAN_TYPE_SEL_OBSERVER,
    /// Connectable discovery
    GAPM_SCAN_TYPE_CONN_DISC,
    /// Selective connectable discovery
    GAPM_SCAN_TYPE_SEL_CONN_DISC,
};

/// Scanning properties bit field bit value
enum gapm_scan_prop
{
    /// Scan advertisement on the LE 1M PHY
    GAPM_SCAN_PROP_PHY_1M_BIT       = (1 << 0),
    /// Scan advertisement on the LE Coded PHY
    GAPM_SCAN_PROP_PHY_CODED_BIT    = (1 << 1),
    /// Active scan on LE 1M PHY (Scan Request PDUs may be sent)
    GAPM_SCAN_PROP_ACTIVE_1M_BIT    = (1 << 2),
    /// Active scan on LE Coded PHY (Scan Request PDUs may be sent)
    GAPM_SCAN_PROP_ACTIVE_CODED_BIT = (1 << 3),
    /// Accept directed advertising packets if we use a RPA and target address cannot be solved by the
    /// controller
    GAPM_SCAN_PROP_ACCEPT_RPA_BIT   = (1 << 4),
    /// Filter truncated advertising or scan response reports
    GAPM_SCAN_PROP_FILT_TRUNC_BIT   = (1 << 5),
};

/// Initiating Types
enum gapm_init_type
{
    /// Direct connection establishment, establish a connection with an indicated device
    GAPM_INIT_TYPE_DIRECT_CONN_EST = 0,
    /// Automatic connection establishment, establish a connection with all devices whose address is
    /// present in the white list
    GAPM_INIT_TYPE_AUTO_CONN_EST,
    /// Name discovery, Establish a connection with an indicated device in order to read content of its
    /// Device Name characteristic. Connection is closed once this operation is stopped.
    GAPM_INIT_TYPE_NAME_DISC,
};

/// Initiating Properties
enum gapm_init_prop
{
    /// Scan connectable advertisements on the LE 1M PHY. Connection parameters for the LE 1M PHY are provided
    GAPM_INIT_PROP_1M_BIT       = (1 << 0),
    /// Connection parameters for the LE 2M PHY are provided
    GAPM_INIT_PROP_2M_BIT       = (1 << 1),
    /// Scan connectable advertisements on the LE Coded PHY. Connection parameters for the LE Coded PHY are provided
    GAPM_INIT_PROP_CODED_BIT    = (1 << 2),
};

/// Advertising report information
enum gapm_adv_report_info
{
    /// Report Type
    GAPM_REPORT_INFO_REPORT_TYPE_MASK    = 0x07,
    /// Report is complete
    GAPM_REPORT_INFO_COMPLETE_BIT        = (1 << 3),
    /// Connectable advertising
    GAPM_REPORT_INFO_CONN_ADV_BIT        = (1 << 4),
    /// Scannable advertising
    GAPM_REPORT_INFO_SCAN_ADV_BIT        = (1 << 5),
    /// Directed advertising
    GAPM_REPORT_INFO_DIR_ADV_BIT         = (1 << 6),
};

/// Filtering policy for duplicated packets
enum gapm_dup_filter_pol
{
    /// Disable filtering of duplicated packets
    GAPM_DUP_FILT_DIS = 0,
    /// Enable filtering of duplicated packets
    GAPM_DUP_FILT_EN,
    /// Enable filtering of duplicated packets, reset for each scan period
    GAPM_DUP_FILT_EN_PERIOD,
};

/// Periodic synchronization types
enum gapm_per_sync_type
{
    /// Do not use periodic advertiser list for synchronization. Use advertiser information provided
    /// in the GAPM_ACTIVITY_START_CMD.
    GAPM_PER_SYNC_TYPE_GENERAL = 0,
    /// Use periodic advertiser list for synchronization
    GAPM_PER_SYNC_TYPE_SELECTIVE,
};

/// PHY Type
enum gapm_phy_type
{
    /// LE 1M
    GAPM_PHY_TYPE_LE_1M = 0,
    /// LE 2M
    GAPM_PHY_TYPE_LE_2M,
    /// LE Coded
    GAPM_PHY_TYPE_LE_CODED,
};

/// -------------------------------------------------------------------------------------
/// Masks for advertising properties
/// -------------------------------------------------------------------------------------

/// Advertising properties configurations for legacy advertising
enum gapm_leg_adv_prop
{
    /// Non connectable non scannable advertising
    GAPM_ADV_PROP_NON_CONN_NON_SCAN_MASK  = 0x0000,
    /// Broadcast non scannable advertising - Discovery mode must be Non Discoverable
    GAPM_ADV_PROP_BROADCAST_NON_SCAN_MASK = GAPM_ADV_PROP_NON_CONN_NON_SCAN_MASK,
    /// Non connectable scannable advertising
    GAPM_ADV_PROP_NON_CONN_SCAN_MASK      = GAPM_ADV_PROP_SCANNABLE_BIT,
    /// Broadcast non scannable advertising - Discovery mode must be Non Discoverable
    GAPM_ADV_PROP_BROADCAST_SCAN_MASK     = GAPM_ADV_PROP_NON_CONN_SCAN_MASK,
    /// Undirected connectable advertising
    GAPM_ADV_PROP_UNDIR_CONN_MASK         = GAPM_ADV_PROP_CONNECTABLE_BIT | GAPM_ADV_PROP_SCANNABLE_BIT,
    /// Directed connectable advertising
    GAPM_ADV_PROP_DIR_CONN_MASK           = GAPM_ADV_PROP_DIRECTED_BIT | GAPM_ADV_PROP_CONNECTABLE_BIT,
    /// Directed connectable with Low Duty Cycle
    GAPM_ADV_PROP_DIR_CONN_LDC_MASK       = GAPM_ADV_PROP_DIR_CONN_MASK,
    /// Directed connectable with High Duty Cycle
    GAPM_ADV_PROP_DIR_CONN_HDC_MASK       = GAPM_ADV_PROP_DIR_CONN_MASK | GAPM_ADV_PROP_HDC_BIT,
};

/// Advertising properties configurations for extended advertising
enum gapm_ext_adv_prop
{
    /// Non connectable non scannable extended advertising
    GAPM_EXT_ADV_PROP_NON_CONN_NON_SCAN_MASK = 0x0000,
    /// Non connectable scannable extended advertising
    GAPM_EXT_ADV_PROP_NON_CONN_SCAN_MASK     = GAPM_ADV_PROP_SCANNABLE_BIT,
    /// Non connectable scannable directed extended advertising
    GAPM_EXT_ADV_PROP_NON_CONN_SCAN_DIR_MASK = GAPM_ADV_PROP_SCANNABLE_BIT | GAPM_ADV_PROP_DIRECTED_BIT,
    /// Non connectable anonymous directed extended advertising
    GAPM_EXT_ADV_PROP_ANONYM_DIR_MASK        = GAPM_ADV_PROP_ANONYMOUS_BIT | GAPM_ADV_PROP_DIRECTED_BIT,
    /// Undirected connectable extended advertising
    GAPM_EXT_ADV_PROP_UNDIR_CONN_MASK        = GAPM_ADV_PROP_CONNECTABLE_BIT,
    /// Directed connectable extended advertising
    GAPM_EXT_ADV_PROP_DIR_CONN_MASK          = GAPM_ADV_PROP_CONNECTABLE_BIT | GAPM_ADV_PROP_DIRECTED_BIT,
};

/// Advertising properties configurations for periodic advertising
enum gapm_per_adv_prop
{
    /// Undirected periodic advertising
    GAPM_PER_ADV_PROP_UNDIR_MASK = 0x0000,
    /// Directed periodic advertising
    GAPM_PER_ADV_PROP_DIR_MASK   = GAPM_ADV_PROP_DIRECTED_BIT,
};

/// Clock accuracy values
enum gapm_clk_acc
{
    /// 500 ppm
    GAPM_CLK_ACC_500 = 0,
    /// 250 ppm
    GAPM_CLK_ACC_250,
    /// 150 ppm
    GAPM_CLK_ACC_150,
    /// 100 ppm
    GAPM_CLK_ACC_100,
    /// 75 ppm
    GAPM_CLK_ACC_75,
    /// 50 ppm
    GAPM_CLK_ACC_50,
    /// 30 ppm
    GAPM_CLK_ACC_30,
    /// 20 ppm
    GAPM_CLK_ACC_20,
};

/// Privacy configuration
enum gapm_priv_cfg
{
    /// Indicate if identity address is a public (0) or static private random (1) address
    GAPM_PRIV_CFG_PRIV_ADDR_BIT = (1 << 0),
    /// Reserved
    GAPM_PRIV_CFG_RSVD          = (1 << 1),
    /// Indicate if controller privacy is enabled
    GAPM_PRIV_CFG_PRIV_EN_BIT   = (1 << 2),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Configuration for advertising on primary channel
/*@TRACE*/
struct gapm_adv_prim_cfg
{
    /// Minimum advertising interval (in unit of 625us). Must be greater than 20ms
    uint32_t adv_intv_min;
    /// Maximum advertising interval (in unit of 625us). Must be greater than 20ms
    uint32_t adv_intv_max;
    /// Bit field indicating the channel mapping
    uint8_t chnl_map;
    /// Indicate on which PHY primary advertising has to be performed (@see enum gapm_phy_type)
    /// Note that LE 2M PHY is not allowed and that legacy advertising only support LE 1M PHY
    uint8_t phy;
};

/// Configuration for advertising on secondary channel
struct gapm_adv_second_cfg
{
    /// Maximum number of advertising events the controller can skip before sending the
    /// AUX_ADV_IND packets. 0 means that AUX_ADV_IND PDUs shall be sent prior each
    /// advertising events
    uint8_t max_skip;
    /// Indicate on which PHY secondary advertising has to be performed (@see enum gapm_phy_type)
    uint8_t phy;
    /// Advertising SID
    uint8_t adv_sid;
};

/// Configuration for periodic advertising
struct gapm_adv_period_cfg
{
    /// Minimum advertising interval (in unit of 1.25ms). Must be greater than 20ms
    uint16_t adv_intv_min;
    /// Maximum advertising interval (in unit of 1.25ms). Must be greater than 20ms
    uint16_t adv_intv_max;
};

/// Advertising parameters for advertising creation
struct gapm_adv_create_param
{
    /// Advertising type (@see enum gapm_adv_type)
    uint8_t type;
    /// Discovery mode (@see enum gapm_adv_disc_mode)
    uint8_t disc_mode;
    /// Bit field value provided advertising properties (@see enum gapm_adv_prop for bit signification)
    uint16_t prop;
    /// Maximum power level at which the advertising packets have to be transmitted
    /// (between -127 and 126 dBm)
    int8_t max_tx_pwr;
    /// Advertising filtering policy (@see enum adv_filter_policy)
    uint8_t filter_pol;
    /// Peer address configuration (only used in case of directed advertising)
    struct gap_bdaddr peer_addr;
    /// Configuration for primary advertising
    struct gapm_adv_prim_cfg prim_cfg;
    /// Configuration for secondary advertising (valid only if advertising type is
    /// GAPM_ADV_TYPE_EXTENDED or GAPM_ADV_TYPE_PERIODIC)
    struct gapm_adv_second_cfg second_cfg;
    /// Configuration for periodic advertising (valid only if advertising type os
    /// GAPM_ADV_TYPE_PERIODIC)
    struct gapm_adv_period_cfg period_cfg;
};

/// Additional advertising parameters
/*@TRACE*/
struct gapm_adv_param
{
    /// Advertising duration (in unit of 10ms). 0 means that advertising continues
    /// until the host disable it
    uint16_t duration;
    /// Maximum number of extended advertising events the controller shall attempt to send prior to
    /// terminating the extending advertising
    /// Valid only if extended advertising
    uint8_t max_adv_evt;
};

/// Scan Window operation parameters
/*@TRACE*/
struct gapm_scan_wd_op_param
{
    /// Scan interval
    uint16_t scan_intv;
    /// Scan window
    uint16_t scan_wd;
};

/// Scanning parameters
/*@TRACE*/
struct gapm_scan_param
{
    /// Type of scanning to be started (@see enum gapm_scan_type)
    uint8_t type;
    /// Properties for the scan procedure (@see enum gapm_scan_prop for bit signification)
    uint8_t prop;
    /// Duplicate packet filtering policy
    uint8_t dup_filt_pol;
    /// Reserved for future use
    uint8_t rsvd;
    /// Scan window opening parameters for LE 1M PHY
    struct gapm_scan_wd_op_param scan_param_1m;
    /// Scan window opening parameters for LE Coded PHY
    struct gapm_scan_wd_op_param scan_param_coded;
    /// Scan duration (in unit of 10ms). 0 means that the controller will scan continuously until
    /// reception of a stop command from the application
    uint16_t duration;
    /// Scan period (in unit of 1.28s). Time interval betweem two consequent starts of a scan duration
    /// by the controller. 0 means that the scan procedure is not periodic
    uint16_t period;
};

/// Connection parameters
/*@TRACE*/
struct gapm_conn_param
{
    /// Minimum value for the connection interval (in unit of 1.25ms). Shall be less than or equal to
    /// conn_intv_max value. Allowed range is 7.5ms to 4s.
    uint16_t conn_intv_min;
    /// Maximum value for the connection interval (in unit of 1.25ms). Shall be greater than or equal to
    /// conn_intv_min value. Allowed range is 7.5ms to 4s.
    uint16_t conn_intv_max;
    /// Slave latency. Number of events that can be missed by a connected slave device
    uint16_t conn_latency;
    /// Link supervision timeout (in unit of 10ms). Allowed range is 100ms to 32s
    uint16_t supervision_to;
    /// Recommended minimum duration of connection events (in unit of 625us)
    uint16_t ce_len_min;
    /// Recommended maximum duration of connection events (in unit of 625us)
    uint16_t ce_len_max;
};

/// Initiating parameters
/*@TRACE*/
struct gapm_init_param
{
    /// Initiating type (@see enum gapm_init_type)
    uint8_t type;
    /// Properties for the initiating procedure (@see enum gapm_init_prop for bit signification)
    uint8_t prop;
    /// Timeout for automatic connection establishment (in unit of 10ms). Cancel the procedure if not all
    /// indicated devices have been connected when the timeout occurs. 0 means there is no timeout
    uint16_t conn_to;
    /// Scan window opening parameters for LE 1M PHY
    struct gapm_scan_wd_op_param scan_param_1m;
    /// Scan window opening parameters for LE Coded PHY
    struct gapm_scan_wd_op_param scan_param_coded;
    /// Connection parameters for LE 1M PHY
    struct gapm_conn_param conn_param_1m;
    /// Connection parameters for LE 2M PHY
    struct gapm_conn_param conn_param_2m;
    /// Connection parameters for LE Coded PHY
    struct gapm_conn_param conn_param_coded;
    /// Address of peer device in case white list is not used for connection
    struct gap_bdaddr peer_addr;
};

/// Periodic advertising information
/*@TRACE*/
struct gapm_period_adv_addr_cfg
{
    /// Advertiser address information
    struct gap_bdaddr addr;
    /// Advertising SID
    uint8_t adv_sid;
};

/// Periodic synchronization parameters
/*@TRACE*/
struct gapm_per_sync_param
{
    /// Number of periodic advertising that can be skipped after a successful receive. Maximum authorized
    /// value is 499
    uint16_t skip;
    /// Synchronization timeout for the periodic advertising (in unit of 10ms between 100ms and 163.84s)
    uint16_t sync_to;
    /// Periodic synchronization type (@see enum gapm_per_sync_type)
    uint8_t type;
    /// Reserved for future use
    uint8_t rsvd;
    /// Address of advertiser with which synchronization has to be established (used only if use_pal is false)
    struct gapm_period_adv_addr_cfg adv_addr;
};

/// Operation command structure in order to keep requested operation.
struct gapm_operation_cmd
{
    /// GAP request type
    uint8_t operation;
};

/// Command complete event data structure
/*@TRACE*/
struct gapm_cmp_evt
{
    /// GAP requested operation
    uint8_t operation;
    /// Status of the request
    uint8_t status;
};

///  Reset link layer and the host command
/*@TRACE*/
struct gapm_reset_cmd
{
    /// GAPM requested operation:
    /// - GAPM_RESET: Reset BLE subsystem: LL and HL.
    uint8_t operation;
};

/// Set device configuration command
/*@TRACE*/
struct gapm_set_dev_config_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_SET_DEV_CONFIG: Set device configuration
    uint8_t operation;
    /// Device Role: Central, Peripheral, Observer, Broadcaster or All roles.
    uint8_t role;

    /// -------------- Privacy Config -----------------------
    /// Duration before regenerate device address when privacy is enabled. - in seconds
    uint16_t renew_dur;
    /// Provided own static private random address
    bd_addr_t addr;
    /// Device IRK used for resolvable random BD address generation (LSB first)
    struct gap_sec_key irk;
    /// Privacy configuration bit field (@see enum gapm_priv_cfg for bit signification)
    uint8_t privacy_cfg;

    /// -------------- Security Config -----------------------

    /// Pairing mode authorized (@see enum gapm_pairing_mode)
    uint8_t pairing_mode;

    /// -------------- ATT Database Config -----------------------

    /// GAP service start handle
    uint16_t gap_start_hdl;
    /// GATT service start handle
    uint16_t gatt_start_hdl;

    /// Attribute database configuration (@see enum gapm_att_cfg_flag)
    uint16_t  att_cfg;

    /// -------------- LE Data Length Extension -----------------------
    ///Suggested value for the Controller's maximum transmitted number of payload octets to be used
    uint16_t sugg_max_tx_octets;
    ///Suggested value for the Controller's maximum packet transmission time to be used
    uint16_t sugg_max_tx_time;

    /// --------------- L2CAP Configuration ---------------------------
    /// Maximal MTU acceptable for device
    uint16_t max_mtu;
    /// Maximal MPS Packet size acceptable for device
    uint16_t max_mps;
    /// Maximum number of LE Credit based connection that can be established
    uint8_t  max_nb_lecb;

    /// --------------- LE Audio Mode Supported -----------------------
    ///
    /// LE Audio Mode Configuration (@see gapm_audio_cfg_flag)
    uint16_t  audio_cfg;

    /// ------------------ LE PHY Management  -------------------------
    /// Preferred LE PHY for data transmission (@see enum gap_phy)
    uint8_t tx_pref_phy;
    /// Preferred LE PHY for data reception (@see enum gap_phy)
    uint8_t rx_pref_phy;

    /// ------------------ Miscellaneous 2 ----------------------------
    /// RF TX Path Compensation value (from -128dB to 128dB, unit is 0.1dB)
    uint16_t tx_path_comp;
    /// RF RX Path Compensation value (from -128dB to 128dB, unit is 0.1dB)
    uint16_t rx_path_comp;
};

/// Set new IRK
/*@TRACE*/
struct gapm_set_irk_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_SET_IRK: Set device configuration
    uint8_t operation;
    /// Device IRK used for resolvable random BD address generation (LSB first)
    struct gap_sec_key irk;
};

/// Set device channel map
/*@TRACE*/
struct gapm_set_channel_map_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_SET_CHANNEL_MAP: Set device channel map.
    uint8_t operation;
    /// Channel map
    le_chnl_map_t chmap;
};

/// Get local device info command
/*@TRACE*/
struct gapm_get_dev_info_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_GET_DEV_VERSION: Get Local device version
    ///  - GAPM_GET_DEV_BDADDR: Get Local device BD Address
    ///  - GAPM_GET_DEV_ADV_TX_POWER: Get device advertising power level
    ///  - GAPM_DBG_GET_MEM_INFO: Get memory usage (debug only)
    uint8_t operation;
};

/// Local device version indication event
/*@TRACE*/
struct gapm_dev_version_ind
{
    /// HCI version
    uint8_t hci_ver;
    /// LMP version
    uint8_t lmp_ver;
    /// Host version
    uint8_t host_ver;
    /// HCI revision
    uint16_t hci_subver;
    /// LMP subversion
    uint16_t lmp_subver;
    /// Host revision
    uint16_t host_subver;
    /// Manufacturer name
    uint16_t manuf_name;
};

/// Local device BD Address indication event
/*@TRACE*/
struct gapm_dev_bdaddr_ind
{
    /// Local device address information
    struct gap_bdaddr addr;
    /// Activity index
    uint8_t actv_idx;
};

struct gapm_aes_h6_ind
{
    uint8_t aes_res[GAP_KEY_LEN];
    uint32_t metainfo;
};
/// Advertising channel Tx power level indication event
/*@TRACE*/
struct gapm_dev_adv_tx_power_ind
{
    /// Advertising channel Tx power level
    int8_t     power_lvl;
};

/// Resolving Address indication event
/*@TRACE*/
struct gapm_ral_addr_ind
{
    /// Peer or local read operation
    uint8_t operation;
    /// Resolving List address
    struct gap_bdaddr addr;
};

/// Resolve Address command
/*@TRACE*/
struct gapm_resolv_addr_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_RESOLV_ADDR: Resolve device address
    uint8_t operation;
    /// Number of provided IRK (sahlle be > 0)
    uint8_t nb_key;
    /// Resolvable random address to solve
    bd_addr_t addr;
    /// Array of IRK used for address resolution (MSB -> LSB)
    struct gap_sec_key irk[__ARRAY_EMPTY];
};

/// Indicate that resolvable random address has been solved
/*@TRACE*/
struct gapm_addr_solved_ind
{
    /// Resolvable random address solved
    bd_addr_t addr;
    /// IRK that correctly solved the random address
    struct gap_sec_key irk;
};

/// Name of peer device indication
/*@TRACE*/
struct gapm_peer_name_ind
{
    /// peer device bd address
    bd_addr_t addr;
    /// peer device address type
    uint8_t addr_type;
    /// peer device name length
    uint8_t name_len;
    /// peer device name
    uint8_t name[__ARRAY_EMPTY];
};

/// Generate a random address.
/*@TRACE*/
struct gapm_gen_rand_addr_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_GEN_RAND_ADDR: Generate a random address
    uint8_t  operation;
    /// Dummy parameter used to store the prand part of the address
    uint8_t  prand[GAP_ADDR_PRAND_LEN];
    /// Random address type @see gap_rnd_addr_type
    ///  - GAP_STATIC_ADDR: Static random address
    ///  - GAP_NON_RSLV_ADDR: Private non resolvable address
    ///  - GAP_RSLV_ADDR: Private resolvable address
    uint8_t rnd_type;
};

/// Parameters of the @ref GAPM_USE_ENC_BLOCK_CMD message
/*@TRACE*/
struct gapm_use_enc_block_cmd
{
    /// Command Operation Code (shall be GAPM_USE_ENC_BLOCK)
    uint8_t operation;
    /// Operand 1
    uint8_t operand_1[GAP_KEY_LEN];
    /// Operand 2
    uint8_t operand_2[GAP_KEY_LEN];
};

/// Parameters of the @ref GAPM_USE_ENC_BLOCK_IND message
/*@TRACE*/
struct gapm_use_enc_block_ind
{
    /// Result (16 bytes)
    uint8_t result[GAP_KEY_LEN];
};

/// Parameters of the @ref GAPM_GEN_DH_KEY_CMD message
/*@TRACE*/
struct gapm_gen_dh_key_cmd
{
    /// Command Operation Code (shall be GAPM_GEN_DH_KEY)
    uint8_t operation;
    /// X coordinate
    uint8_t operand_1[GAP_P256_KEY_LEN];
    /// Y coordinate
    uint8_t operand_2[GAP_P256_KEY_LEN];
};

/// Parameters of the @ref GAPM_GEN_DH_KEY_IND message
/*@TRACE*/
struct gapm_gen_dh_key_ind
{
    /// Result (32 bytes)
    uint8_t result[GAP_P256_KEY_LEN];
};

/// Parameters of the @ref GAPM_GET_PUB_KEY_CMD message
/*@TRACE*/
struct gapm_get_pub_key_cmd
{
    /// Command Operation Code (shall be GAPM_GET_PUB_KEY)
    uint8_t operation;
    /// 1 to renew, 0 to read current value
    uint8_t renew;
};

/// Parameters of the @ref GAPM_PUB_KEY_IND message
/*@TRACE*/
struct gapm_pub_key_ind
{
    /// X coordinate
    uint8_t pub_key_x[GAP_P256_KEY_LEN];
    /// Y coordinate
    uint8_t pub_key_y[GAP_P256_KEY_LEN];
};

/// Parameters of the @ref GAPM_GEN_RAND_NB_CMD message
/*@TRACE*/
struct gapm_gen_rand_nb_cmd
{
    /// Command Operation Code (shall be GAPM_GEN_RAND_NB)
    uint8_t operation;
};

/// Parameters of the @ref GAPM_GEN_RAND_NB_IND message
/*@TRACE*/
struct gapm_gen_rand_nb_ind
{
    /// Generation Random Number (8 bytes)
    rand_nb_t randnb;
};

/// Create new task for specific profile
/*@TRACE*/
struct gapm_profile_task_add_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_PROFILE_TASK_ADD: Add new profile task
    uint8_t  operation;
    /// Security Level :
    ///  7    6    5    4    3    2    1    0
    /// +----+----+----+----+----+----+----+----+
    /// |   Reserved   |DIS |  AUTH   |EKS | MI |
    /// +----+----+----+----+----+----+----+----+
    ///
    /// - MI: 1 - Application task is a Multi-Instantiated task, 0 - Mono-Instantiated
    /// Only applies for service - Ignored by collectors:
    /// - EKS: Service needs a 16 bytes encryption key
    /// - AUTH: 0 - Disable, 1 - Enable, 2 - Unauth, 3 - Auth
    /// - DIS: Disable the service
    uint8_t  sec_lvl;
    /// Profile task identifier
    uint16_t prf_task_id;
    /// Application task number
    uint16_t app_task;
    /// Service start handle
    /// Only applies for services - Ignored by collectors
    /// 0: dynamically allocated in Attribute database
    uint16_t start_hdl;
    /// 32 bits value that contains value to initialize profile (database parameters, etc...)
    uint32_t param[__ARRAY_EMPTY];
};

/// Inform that profile task has been added.
/*@TRACE*/
struct gapm_profile_added_ind
{
    /// Profile task identifier
    uint16_t prf_task_id;
    /// Profile task number allocated
    uint16_t prf_task_nb;
    /// Service start handle
    /// Only applies for services - Ignored by collectors
    uint16_t start_hdl;
};

/// Indicate that a message has been received on an unknown task
/*@TRACE*/
struct gapm_unknown_task_ind
{
    /// Message identifier
    uint16_t msg_id;
    /// Task identifier
    uint16_t task_id;
};

/// Indicates suggested default data length
/*@TRACE*/
struct gapm_sugg_dflt_data_len_ind
{
    ///Host's suggested value for the Controller's maximum transmitted number of payload octets
    uint16_t suggted_max_tx_octets;
    ///Host's suggested value for the Controller's maximum packet transmission time
    uint16_t suggted_max_tx_time;
};

/// Indicates maximum data length
/*@TRACE*/
struct gapm_max_data_len_ind
{
    ///Maximum number of payload octets that the local Controller supports for transmission
    uint16_t suppted_max_tx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for transmission
    uint16_t suppted_max_tx_time;
    ///Maximum number of payload octets that the local Controller supports for reception
    uint16_t suppted_max_rx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for reception
    uint16_t suppted_max_rx_time;
};

/// Register a LE Protocol/Service Multiplexer command
/*@TRACE*/
struct gapm_lepsm_register_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_LEPSM_REG: Register a LE Protocol/Service Multiplexer
    uint8_t  operation;
    /// LE Protocol/Service Multiplexer
    uint16_t le_psm;
    /// Application task number
    uint16_t app_task;
    /// Security level
    ///   7   6   5   4   3   2   1   0
    /// +---+---+---+---+---+---+---+---+
    /// |MI |      RFU      |EKS|SEC_LVL|
    /// +---+---+---+---+---+---+---+---+
    /// bit[0-1]: Security level requirement (0=NO_AUTH, 1=UNAUTH, 2=AUTH, 3=SEC_CON)
    /// bit[2]  : Encryption Key Size length must have 16 bytes
    /// bit[7]  : Does the application task is multi-instantiated or not
    uint8_t sec_lvl;
};

/// Unregister a LE Protocol/Service Multiplexer command
/*@TRACE*/
struct gapm_lepsm_unregister_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_LEPSM_UNREG: Unregister a LE Protocol/Service Multiplexer
    uint8_t  operation;
    /// LE Protocol/Service Multiplexer
    uint16_t le_psm;
};

/// Control LE Test Mode command
struct gapm_le_test_mode_ctrl_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_LE_TEST_STOP: Unregister a LE Protocol/Service Multiplexer
    ///  - GAPM_LE_TEST_RX_START: Start RX Test Mode
    ///  - GAPM_LE_TEST_TX_START: Start TX Test Mode
    uint8_t operation;
    /// Tx or Rx Channel (Range 0x00 to 0x27)
    uint8_t channel;
    /// Length in bytes of payload data in each packet (only valid for TX mode, range 0x00-0xFF)
    uint8_t tx_data_length;
    /// Packet Payload type (only valid for TX mode @see enum gap_pkt_pld_type)
    uint8_t tx_pkt_payload;
    /// Test PHY rate (@see enum gap_test_phy)
    uint8_t phy;
    /// Modulation Index (only valid for RX mode @see enum gap_modulation_idx)
    uint8_t modulation_idx;
};

/// Indicate end of test mode event
struct gapm_le_test_end_ind
{
    /// Number of received packets
    uint16_t nb_packet_received;
};

/// Provide statistic information about ISO exchange
struct gapm_iso_stat_ind
{
    /// ISO Handle of the isochronous channel (Range 0x0000-0x0EFF)
    uint16_t iso_hdl;

    /// Statistics - Number of transmission attempts
    uint32_t nb_tx;
    /// Statistics - Number of transmission attempts that succeed
    uint32_t nb_tx_ok;
    /// Statistics - Number of Not granted packet packets
    uint32_t nb_tx_not_granted;

    /// Statistics - Number of reception attempt
    uint32_t nb_rx;
    /// Statistics - Number of reception attempts that succeed
    uint32_t nb_rx_ok;
    /// Statistics - Number of Not granted packet packets
    uint32_t nb_rx_not_granted;
    /// Statistics - Number of wrongly received packet (invalid data)
    uint32_t nb_rx_data_err;
    /// Statistics - Number of CRC Errors
    uint32_t nb_rx_crc_err;
    /// Statistics - Number of SYNC Errors
    uint32_t nb_rx_sync_err;
    /// Statistics - Number of received empty packets
    uint32_t nb_rx_empty;
};

/// Create an advertising, a scanning, an initiating, a periodic synchonization activity command (common)
/*@TRACE*/
struct gapm_activity_create_cmd
{
    /// GAPM request operation:
    ///  - GAPM_CREATE_ADV_ACTIVITY: Create advertising activity
    ///  - GAPM_CREATE_SCAN_ACTIVITY: Create scanning activity
    ///  - GAPM_CREATE_INIT_ACTIVITY: Create initiating activity
    ///  - GAPM_CREATE_PERIOD_SYNC_ACTIVITY: Create periodic synchronization activity
    uint8_t operation;
    /// Own address type (@see enum gapm_own_addr)
    uint8_t own_addr_type;
};

/// Create an advertising activity command
struct gapm_activity_create_adv_cmd
{
    /// GAPM request operation:
    ///  - GAPM_CREATE_ADV_ACTIVITY: Create advertising activity
    uint8_t operation;
    /// Own address type (@see enum gapm_own_addr)
    uint8_t own_addr_type;
    /// Advertising parameters (optional, shall be present only if operation is GAPM_CREATE_ADV_ACTIVITY)
    /// For prop parameter, @see enum gapm_leg_adv_prop, @see enum gapm_ext_adv_prop and @see enum gapm_per_adv_prop for help
    struct gapm_adv_create_param adv_param;
};

/// Activity parameters
/*@TRACE
 @trc_ref gapm_actv_type
 */
union gapm_u_param
{
    /// Additional advertising parameters (for advertising activity)
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_ADV
    struct gapm_adv_param adv_add_param;
    /// Scan parameters (for scanning activity)
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_SCAN
    struct gapm_scan_param scan_param;
    /// Initiating parameters (for initiating activity)
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_INIT
    struct gapm_init_param init_param;
    /// Periodic synchronization parameters (for periodic synchronization activity)
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_PER_SYNC
    struct gapm_per_sync_param per_sync_param;
};

/// Start a given activity command
/*@TRACE*/
struct gapm_activity_start_cmd
{
    /// GAPM request operation:
    ///  - GAPM_START_ACTIVITY: Start a given activity
    uint8_t operation;
    /// Activity identifier
    uint8_t actv_idx;
    /// Activity parameters
    union gapm_u_param u_param;
};

/// Stop one or all activity(ies) command
/*@TRACE*/
struct gapm_activity_stop_cmd
{
    /// GAPM request operation:
    ///  - GAPM_STOP_ACTIVITY: Stop a given activity
    ///  - GAPM_STOP_ALL_ACTIVITIES: Stop all existing activities
    uint8_t operation;
    /// Activity identifier - used only if operation is GAPM_STOP_ACTIVITY
    uint8_t actv_idx;
};

/// Delete one or all activity(ies) command
/*@TRACE*/
struct gapm_activity_delete_cmd
{
    /// GAPM request operation:
    ///  - GAPM_DELETE_ACTIVITY: Delete a given activity
    ///  - GAPM_DELETE_ALL_ACTIVITIES: Delete all existing activities
    uint8_t operation;
    /// Activity identifier - used only if operation is GAPM_STOP_ACTIVITY
    uint8_t actv_idx;
};

/// Indicate creation of an activity
/*@TRACE
 @trc_exec activity_map[$actv_idx] = $actv_type
 activity_map = {}*/
struct gapm_activity_created_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Activity type (@see enum gapm_actv_type)
    uint8_t actv_type;
    /// Selected TX power for advertising activity
    int8_t  tx_pwr;
};

/// Indicate that an activity has been stopped
/*@TRACE*/
struct gapm_activity_stopped_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Activity type (@see enum gapm_actv_type)
    uint8_t actv_type;
    /// Activity stop reason
    uint8_t reason;
    /// In case of periodic advertising, indicate if periodic advertising has been stopped
    uint8_t per_adv_stop;
};

/// Set advertising, scan response or periodic advertising data command
/*@TRACE*/
struct gapm_set_adv_data_cmd
{
    /// GAPM request operation:
    ///  - GAPM_SET_ADV_DATA: Set advertising data
    ///  - GAPM_SET_SCAN_RSP_DATA: Set scan response data
    ///  - GAPM_SET_PERIOD_ADV_DATA: Set periodic advertising data
    uint8_t operation;
    /// Activity identifier
    uint8_t actv_idx;
    /// Data length
    uint16_t length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// Indicate reception of scan request
/*@TRACE*/
struct gapm_scan_request_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Transmitter device address
    struct gap_bdaddr trans_addr;
};

/// Indicate reception of advertising, scan response or periodic advertising data
/*@TRACE*/
struct gapm_ext_adv_report_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Bit field providing information about the received report (@see enum gapm_adv_report_info)
    uint8_t info;
    /// Transmitter device address
    struct gap_bdaddr trans_addr;
    /// Target address (in case of a directed advertising report)
    struct gap_bdaddr target_addr;
    /// TX power (in dBm)
    int8_t tx_pwr;
    /// RSSI (between -127 and +20 dBm)
    int8_t rssi;
    /// Primary PHY on which advertising report has been received
    uint8_t phy_prim;
    /// Secondary PHY on which advertising report has been received
    uint8_t phy_second;
    /// Advertising SID
    /// Valid only for periodic advertising report
    uint8_t adv_sid;
    /// Periodic advertising interval (in unit of 1.25ms, min is 7.5ms)
    /// Valid only for periodic advertising report
    uint16_t period_adv_intv;
    /// Report length
    uint16_t length;
    /// Report
    uint8_t data[__ARRAY_EMPTY];
};

/// Indicate that synchronization has been established with a periodic advertiser
/*@TRACE*/
struct gapm_sync_established_ind
{
    /// Activity identifier
    uint8_t actv_idx;
    /// PHY on which synchronization has been established (@see gap_phy_type)
    uint8_t phy;
    /// Periodic advertising interval (in unit of 1.25ms, min is 7.5ms)
    uint16_t intv;
    /// Advertising SID
    uint8_t adv_sid;
    /// Advertiser clock accuracy (@see enum gapm_clk_acc)
    uint8_t clk_acc;
    /// Advertiser address
    struct gap_bdaddr addr;
};

/// Read local or peer address
/*@TRACE*/
struct gapm_get_ral_addr_cmd
{
    /// GAPM request operation:
    ///  - GAPM_GET_RAL_LOC_ADDR: Set white list content
    ///  - GAPM_GET_RAL_PEER_ADDR: Set resolving list content
    uint8_t operation;
    /// Peer device identity
    struct gap_bdaddr peer_identity;
};

/// Set content of either white list or resolving list or periodic advertiser list command (common part)
struct gapm_list_set_cmd
{
    /// GAPM request operation:
    ///  - GAPM_SET_WHITE_LIST: Set white list content
    ///  - GAPM_SET_RAL: Set resolving list content
    ///  - GAPM_SET_PAL: Set periodic advertiser list content
    uint8_t operation;
    /// Number of entries to be added in the list. 0 means that list content has to be cleared
    uint8_t size;
};

/// Set content of white list
/*@TRACE*/
struct gapm_list_set_wl_cmd
{
    /// GAPM request operation:
    ///  - GAPM_SET_WHITE_LIST: Set white list content
    uint8_t operation;
    /// Number of entries to be added in the list. 0 means that list content has to be cleared
    uint8_t size;
    /// List of entries to be added in the list
    struct gap_bdaddr wl_info[__ARRAY_EMPTY];
};

/// Set content of resolving list command
/*@TRACE*/
struct gapm_list_set_ral_cmd
{
    /// GAPM request operation:
    ///  - GAPM_SET_RAL: Set resolving list content
    uint8_t operation;
    /// Number of entries to be added in the list. 0 means that list content has to be cleared
    uint8_t size;
    /// List of entries to be added in the list
    struct gap_ral_dev_info ral_info[__ARRAY_EMPTY];
};

/// Set content of periodic advertiser list command
/*@TRACE*/
struct gapm_list_set_pal_cmd
{
    /// GAPM request operation:
    ///  - GAPM_SET_PAL: Set periodic advertiser list content
    uint8_t operation;
    /// Number of entries to be added in the list. 0 means that list content has to be cleared
    uint8_t size;
    /// List of entries to be added in the list
    struct gapm_period_adv_addr_cfg pal_info[__ARRAY_EMPTY];
};

struct gapm_aes_h6_cmd
{
    uint8_t   w[GAP_KEY_LEN];
    uint8_t   key_id[KEY_ID_LEN];
    uint32_t  cb_request;
};
/*@TRACE
 @trc_ref gapm_operation
 gapm_list_set_cmd = gapm_u_list_set
*/
union gapm_u_list_set
{
    uint8_t operation;

    //@trc_union operation == GAPM_SET_WL
    struct gapm_list_set_wl_cmd list_set_wl_cmd;
    //@trc_union operation == GAPM_SET_RAL
    struct gapm_list_set_ral_cmd list_set_ral_cmd;
    //@trc_union operation == GAPM_SET_PAL
    struct gapm_list_set_pal_cmd list_set_pal_cmd;
};

/// List Size indication event
/*@TRACE*/
struct gapm_list_size_ind
{
    /// Operation code, indicate list for which size has been read
    ///     - GAPM_SET_WHITE_LIST
    ///     - GAPM_SET_RAL
    ///     - GAPM_SET_PAL
    uint8_t operation;
    /// List size
    uint8_t size;
};

/// Maximum advertising data length indication event
/*@TRACE*/
struct gapm_max_adv_data_len_ind
{
    /// Maximum advertising data length supported by controller
    uint16_t length;
};

/// Number of available advertising sets indication event
/*@TRACE*/
struct gapm_nb_adv_sets_ind
{
    /// Number of available advertising sets
    uint8_t nb_adv_sets;
};

/// Indicate the transmit powers supported by the controller
/*@TRACE*/
struct gapm_dev_tx_pwr_ind
{
    /// Minimum TX power
    int8_t min_tx_pwr;
    /// Maximum TX power
    int8_t max_tx_pwr;
};

/// Indicate the RF path compensation values
/*@TRACE*/
struct gapm_dev_rf_path_comp_ind
{
    /// RF TX path compensation
    uint16_t tx_path_comp;
    /// RF RX path compensation
    uint16_t rx_path_comp;
};

/// Request to renew all currently used random private addresses (non-resolvable or resolvable)
/// For internal use only
/*@TRACE*/
struct gapm_addr_renew_cmd
{
    /// GAPM request operation:
    ///  - GAPM_RENEW_ADDR: Renew random private addresses
    uint8_t operation;
    /// Activity index, used by GAPM ADDR state machine in order to remind for which activity
    /// a new address has been generated
    uint8_t actv_idx;
    /// Index of first created initiating/scanning for which address has been renewed,
    /// any initiating/scanning met after shall use the same address if it uses a random
    /// address
    uint8_t init_scan_actv_idx;
};

/// Indicate that an unknown message has been received
/*@TRACE*/
struct gapm_unknown_msg_ind
{
    /// Unknown message id
    ke_msg_id_t unknown_msg_id;
};


struct gapm_dbg_rssi_notify_ind
{
    uint8_t is_bt;
    int8_t channel_asset[79];
    uint8_t rssi_num;
    int8_t rssi_ave[__ARRAY_EMPTY];
};

struct gapm_dbg_assert_ind
{
    uint8_t type;
};

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @} GAPM_TASK

#endif /* _GAPM_TASK_H_ */
