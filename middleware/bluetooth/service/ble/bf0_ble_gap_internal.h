/**
  ******************************************************************************
  * @file   bf0_ble_gap_internal.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "bf0_ble_gap.h"
#include "bf0_sibles_internal.h"

/// Length of resolvable random address prand part
#define GAP_ADDR_PRAND_LEN            (3)

#ifndef KEY_ID_LEN
    #define KEY_ID_LEN 4
#endif

/// device information data
/*@TRACE
 @trc_ref gapc_dev_info*/
union gapc_set_dev_info
{
    /// Device name
    //@trc_union parent.req == GAPC_DEV_NAME
    ble_gap_dev_name_t name;
    /// Appearance Icon
    //@trc_union parent.req == GAPC_DEV_APPEARANCE
    uint16_t appearance;
    /// Slave preferred parameters
    //@trc_union parent.req == GAPC_DEV_SLV_PREF_PARAMS
    ble_gap_slave_prf_param_t slv_pref_params;
};


/// Peer device request to modify local device info such as name or appearance
/*@TRACE*/
struct gapc_set_dev_info_req_ind
{
    /// Requested information
    /// - GAPC_DEV_NAME: Device Name
    /// - GAPC_DEV_APPEARANCE: Device Appearance Icon
    uint8_t req;

    /// device information data
    union gapc_set_dev_info info;
};

/// List of device info that should be provided by application
/*@TRACE*/
enum gapc_dev_info
{
    /// Device Name
    GAPC_DEV_NAME,
    /// Device Appearance Icon
    GAPC_DEV_APPEARANCE,
    /// Device Slave preferred parameters
    GAPC_DEV_SLV_PREF_PARAMS,
    /// Device Central address resolution
    GAPC_DEV_CTL_ADDR_RESOL,
    /// maximum device info parameter
    GAPC_DEV_INFO_MAX,
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

/// request operation type - application interface
/*@TRACE*/
enum gapc_operation
{
    /*                 Operation Flags                  */
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation
    GAPC_NO_OP                                    = 0x00,

    /* Connection management */
    /// Disconnect link
    GAPC_DISCONNECT,

    /* Connection information */
    /// Retrieve name of peer device.
    GAPC_GET_PEER_NAME,
    /// Retrieve peer device version info.
    GAPC_GET_PEER_VERSION,
    /// Retrieve peer device features.
    GAPC_GET_PEER_FEATURES,
    /// Get Peer device appearance
    GAPC_GET_PEER_APPEARANCE,
    /// Get Peer device Slaved Preferred Parameters
    GAPC_GET_PEER_SLV_PREF_PARAMS,
    /// Retrieve connection RSSI.
    GAPC_GET_CON_RSSI,
    /// Retrieve Connection Channel MAP.
    GAPC_GET_CON_CHANNEL_MAP,

    /* Connection parameters update */
    /// Perform update of connection parameters.
    GAPC_UPDATE_PARAMS,

    /* Security procedures */
    /// Start bonding procedure.
    GAPC_BOND,
    /// Start encryption procedure.
    GAPC_ENCRYPT,
    /// Start security request procedure
    GAPC_SECURITY_REQ,

    /* Deprecated */
    /// Deprecated operation
    GAPC_OP_DEPRECATED_0,
    GAPC_OP_DEPRECATED_1,
    GAPC_OP_DEPRECATED_2,
    GAPC_OP_DEPRECATED_3,
    GAPC_OP_DEPRECATED_4,

    /* LE Ping*/
    /// get timer timeout value
    GAPC_GET_LE_PING_TO,
    /// set timer timeout value
    GAPC_SET_LE_PING_TO,

    /* LE Data Length extension*/
    /// LE Set Data Length
    GAPC_SET_LE_PKT_SIZE,

    /* Central Address resolution supported*/
    GAPC_GET_ADDR_RESOL_SUPP,

    /* Secure Connections */
    /// Request to inform the remote device when keys have been entered or erased
    GAPC_KEY_PRESS_NOTIFICATION,

    /* PHY Management */
    /// Set the PHY configuration for current active link
    GAPC_SET_PHY,
    /// Retrieve PHY configuration of active link
    GAPC_GET_PHY,

    /* Channel Selection Algorithm */
    /// Retrieve Channel Selection Algorithm
    GAPC_GET_CHAN_SEL_ALGO,

    /* Preferred slave latency */
    /// Set the preferred slave latency (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_LATENCY,
    /// Set the preferred slave event duration (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_EVT_DUR,

    // ---------------------- INTERNAL API ------------------------
    /* Packet signature */
    /// sign an attribute packet
    GAPC_SIGN_PACKET,
    /// Verify signature or an attribute packet
    GAPC_SIGN_CHECK,
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
    ble_gap_addr_t wl_info[__ARRAY_EMPTY];
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
    ble_gap_ral_dev_info_t ral_info[__ARRAY_EMPTY];
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
    ble_gap_addr_t peer_identity;
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

/// Generic Security key structure
/*@TRACE*/
struct gap_sec_key
{
    /// Key value MSB -> LSB
    uint8_t key[GAP_KEY_LEN];
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
    ble_gap_addr_t peer_addr;
    /// Configuration for primary advertising
    gapm_adv_prim_cfg_t prim_cfg;
    /// Configuration for secondary advertising (valid only if advertising type is
    /// GAPM_ADV_TYPE_EXTENDED or GAPM_ADV_TYPE_PERIODIC)
    gapm_adv_second_cfg_t second_cfg;
    /// Configuration for periodic advertising (valid only if advertising type os
    /// GAPM_ADV_TYPE_PERIODIC)
    struct gapm_adv_period_cfg period_cfg;
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
    ble_gap_addr_t addr;
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
    ble_gap_addr_t peer_addr;
};


/// Periodic advertising information
/*@TRACE*/
struct gapm_period_adv_addr_cfg
{
    /// Advertiser address information
    ble_gap_addr_t addr;
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

/// Request disconnection of current link command.
/*@TRACE*/
struct gapc_disconnect_cmd
{
    /// GAP request type:
    /// - GAPC_DISCONNECT: Disconnect link.
    uint8_t operation;

    /// Reason of disconnection
    uint8_t reason;
};

/// Retrieve information command
/*@TRACE*/
struct gapc_get_info_cmd
{
    /// GAP request type:
    /// - GAPC_GET_PEER_NAME: Retrieve name of peer device.
    /// - GAPC_GET_PEER_VERSION: Retrieve peer device version info.
    /// - GAPC_GET_PEER_FEATURES: Retrieve peer device features.
    /// - GAPC_GET_CON_RSSI: Retrieve connection RSSI.
    /// - GAPC_GET_CON_CHANNEL_MAP: Retrieve Connection Channel MAP.
    /// - GAPC_GET_PEER_APPEARANCE: Get Peer device appearance
    /// - GAPC_GET_PEER_SLV_PREF_PARAMS: Get Peer device Slaved Preferred Parameters
    /// - GAPC_GET_ADDR_RESOL_SUPP: Address Resolution Supported
    /// - GAPC_GET_LE_PING_TIMEOUT: Retrieve LE Ping Timeout Value
    uint8_t operation;
};

/// Pairing parameters
/*@TRACE*/
struct gapc_pairing
{
    /// IO capabilities (@see gap_io_cap)
    uint8_t iocap;
    /// OOB information (@see gap_oob)
    uint8_t oob;
    /// Authentication (@see gap_auth)
    /// Note in BT 4.1 the Auth Field is extended to include 'Key Notification' and
    /// and 'Secure Connections'.
    uint8_t auth;
    /// Encryption key size (7 to 16)
    uint8_t key_size;
    ///Initiator key distribution (@see gap_kdist)
    uint8_t ikey_dist;
    ///Responder key distribution (@see gap_kdist)
    uint8_t rkey_dist;

    /// Device security requirements (minimum security level). (@see gap_sec_req)
    uint8_t sec_req;
};


/// Start Bonding command procedure
/*@TRACE*/
struct gapc_bond_cmd
{
    /// GAP request type:
    /// - GAPC_BOND:  Start bonding procedure.
    uint8_t operation;
    /// Pairing information
    struct gapc_pairing pairing;
};


/// Perform update of connection parameters command
/*@TRACE*/
struct gapc_param_update_cmd
{
    /// GAP request type:
    /// - GAPC_UPDATE_PARAMS: Perform update of connection parameters.
    uint8_t operation;
    /// Internal parameter used to manage internally l2cap packet identifier for signaling
    uint8_t pkt_id;
    /// Connection interval minimum
    uint16_t intv_min;
    /// Connection interval maximum
    uint16_t intv_max;
    /// Latency
    uint16_t latency;
    /// Supervision timeout
    uint16_t time_out;
    /// Minimum Connection Event Duration
    uint16_t ce_len_min;
    /// Maximum Connection Event Duration
    uint16_t ce_len_max;
};

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


/// Confirm requested bond information.
/*@TRACE*/
struct gapc_bond_cfm
{
    /// Bond request type (@see gapc_bond)
    uint8_t request;
    /// Request accepted
    uint8_t accept;

    /// Bond procedure information data
    ble_gap_bond_cfm_data_t data;
};

/// Confirm requested Encryption information.
/*@TRACE*/
struct gapc_encrypt_cfm
{
    /// Indicate if a LTK has been found for the peer device
    uint8_t found;
    /// Long Term Key
    ble_gap_sec_key_t ltk;
    /// LTK Key Size
    uint8_t key_size;
};

/// Start Security Request command procedure
/*@TRACE*/
struct gapc_security_cmd
{
    /// GAP request type:
    /// - GAPC_SECURITY_REQ: Start security request procedure
    uint8_t operation;
    /// Authentication level (@see gap_auth)
    uint8_t auth;
};

/// Set the PHY configuration for current active link
/*@TRACE*/
struct gapc_set_phy_cmd
{
    /// GAP request type:
    /// - GAPC_SET_PHY : Set the PHY configuration for current active link
    uint8_t operation;
    /// Supported LE PHY for data transmission (@see enum gap_phy)
    uint8_t tx_phy;
    /// Supported LE PHY for data reception (@see enum gap_phy)
    uint8_t rx_phy;
    /// PHY options (@see enum gapc_phy_option)
    uint8_t phy_opt;
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


/// Event struct

/// Command complete event data structure
/*@TRACE*/
struct gapm_cmp_evt
{
    /// GAP requested operation
    uint8_t operation;
    /// Status of the request
    uint8_t status;
};

/// Command complete event data structure
/*@TRACE*/
struct gattc_cmp_evt
{
    /// GATT request type
    uint8_t operation;
    /// Status of the request
    uint8_t status;
    /// operation sequence number - provided when operation is started
    uint16_t seq_num;
};


/// Resolving Address indication event
/*@TRACE*/
struct gapm_ral_addr_ind
{
    /// Peer or local read operation
    uint8_t operation;
    /// Resolving List address
    ble_gap_addr_t addr;
};

/// Indicate that a connection has been established
/*@TRACE*/
struct gapc_connection_req_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection interval
    uint16_t con_interval;
    /// Connection latency
    uint16_t con_latency;
    /// Link supervision timeout
    uint16_t sup_to;
    /// Clock accuracy
    uint8_t clk_accuracy;
    /// Peer address type
    uint8_t peer_addr_type;
    /// Peer BT address
    bd_addr_t peer_addr;
    /// Role of device in connection (0 = Master / 1 = Slave)
    uint8_t role;
};

/// Set specific link data configuration.
/*@TRACE*/
struct gapc_connection_cfm
{
    /// Local CSRK value
    struct gap_sec_key lcsrk;
    /// Local signature counter value
    uint32_t lsign_counter;

    /// Remote CSRK value
    struct gap_sec_key rcsrk;
    /// Remote signature counter value
    uint32_t rsign_counter;

    /// Authentication (@see gap_auth)
    uint8_t auth;
    /// Service Changed Indication enabled
    uint8_t svc_changed_ind_enable;
    /// LTK exchanged during pairing.
    bool ltk_present;
};


/// Indicate that a link has been disconnected
/*@TRACE*/
struct gapc_disconnect_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Reason of disconnection
    uint8_t reason;
};


/// Indication of peer version info
/*@TRACE*/
struct gapc_peer_version_ind
{
    /// Manufacturer name
    uint16_t compid;
    /// LMP subversion
    uint16_t lmp_subvers;
    /// LMP version
    uint8_t  lmp_vers;
};

struct gapc_peer_features_ind
{
    /// 8-byte array for LE features
    uint8_t features[GAP_LE_FEATS_LEN];
};

/// Indication of ongoing connection RSSI
/*@TRACE*/
struct gapc_con_rssi_ind
{
    /// RSSI value
    int8_t rssi;
};

/// Active link PHY configuration. Triggered when configuration is read or during an update.
/*@TRACE*/
struct gapc_le_phy_ind
{
    /// LE PHY for data transmission (@see enum gap_phy)
    uint8_t tx_phy;
    /// LE PHY for data reception (@see enum gap_phy)
    uint8_t rx_phy;
};

/// Connection parameters updated indication
/*@TRACE*/
struct gapc_param_updated_ind
{
    ///Connection interval value
    uint16_t            con_interval;
    ///Connection latency value
    uint16_t            con_latency;
    ///Supervision timeout
    uint16_t            sup_to;
};





/// Bonding requested by peer device indication message.
/*@TRACE*/
struct gapc_bond_req_ind
{
    /// Bond request type (@see gapc_bond)
    uint8_t request;

    /// Bond procedure requested information data
    ble_gap_bond_req_data_t data;
};



/// Encryption requested by peer device indication message.
/*@TRACE*/
struct gapc_encrypt_req_ind
{
    /// Encryption Diversifier
    uint16_t ediv;
    /// Random Number
    ble_gap_rand_nb_t rand_nb;
};

/// Bonding information indication message
/*@TRACE*/
struct gapc_bond_ind
{
    /// Bond information type (@see gapc_bond)
    uint8_t info;

    /// Bond procedure information data
    ble_gap_bond_data_t data;
};


/// Encryption information indication message
/*@TRACE*/
struct gapc_encrypt_ind
{
    /// Authentication  level (@see gap_auth)
    uint8_t auth;
};


/// Parameters of the @ref GAPC_SET_LE_PKT_SIZE_CMD message
/*@TRACE*/
struct gapc_set_le_pkt_size_cmd
{
    /// GAP request type:
    /// - GAPC_SET_LE_PKT_SIZE : Set the LE Data length value
    uint8_t operation;
    ///Preferred maximum number of payload octets that the local Controller should include
    ///in a single Link Layer Data Channel PDU.
    uint16_t tx_octets;
    ///Preferred maximum number of microseconds that the local Controller should use to transmit
    ///a single Link Layer Data Channel PDU
    uint16_t tx_time;
};

/// Parameters of the @ref GAPC_LE_PKT_SIZE_IND message
/*@TRACE*/
struct gapc_le_pkt_size_ind
{
    ///The maximum number of payload octets in TX
    uint16_t max_tx_octets;
    ///The maximum time that the local Controller will take to TX
    uint16_t max_tx_time;
    ///The maximum number of payload octets in RX
    uint16_t max_rx_octets;
    ///The maximum time that the local Controller will take to RX
    uint16_t max_rx_time;
};



///Channel map structure
/*@TRACE*/
typedef struct
{
    ///5-byte channel map array
    uint8_t map[GAP_LE_CHNL_MAP_LEN];
} le_chnl_map_t;


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

struct gapm_aes_h6_cmd
{
    uint8_t   w[GAP_KEY_LEN];
    uint8_t   key_id[KEY_ID_LEN];
    uint32_t  cb_request;
};
/// Indication of ongoing connection Channel Map
/*@TRACE*/
struct gapc_con_channel_map_ind
{
    /// channel map value
    le_chnl_map_t ch_map;
};


/// Parameters of the @ref GAPC_SIGN_COUNTER_IND message
/*@TRACE*/
struct gapc_sign_counter_ind
{
    /// Local SignCounter value
    uint32_t local_sign_counter;
    /// Peer SignCounter value
    uint32_t peer_sign_counter;
};

/// Request of updating connection parameters indication
/*@TRACE*/
struct gapc_param_update_req_ind
{
    /// Connection interval minimum
    uint16_t intv_min;
    /// Connection interval maximum
    uint16_t intv_max;
    /// Latency
    uint16_t latency;
    /// Supervision timeout
    uint16_t time_out;
};


/// Master confirm or not that parameters proposed by slave are accepted or not
/*@TRACE*/
struct gapc_param_update_cfm
{
    /// True to accept slave connection parameters, False else.
    bool accept;
    /// Minimum Connection Event Duration
    uint16_t ce_len_min;
    /// Maximum Connection Event Duration
    uint16_t ce_len_max;
};

/// Peer device request local device info such as name, appearance or slave preferred parameters
/*@TRACE*/
struct gapc_get_dev_info_req_ind
{
    /// Requested information
    /// - GAPC_DEV_NAME: Device Name
    /// - GAPC_DEV_APPEARANCE: Device Appearance Icon
    /// - GAPC_DEV_SLV_PREF_PARAMS: Device Slave preferred parameters
    uint8_t req;
};


/// Device name
/*@TRACE*/
struct gap_dev_name
{
    /// name length
    uint16_t length;
    /// name value
    uint8_t value[__ARRAY_EMPTY];
};

/// Slave preferred connection parameters
/*@TRACE*/
struct gap_slv_pref
{
    /// Connection interval minimum
    uint16_t con_intv_min;
    /// Connection interval maximum
    uint16_t con_intv_max;
    /// Slave latency
    uint16_t slave_latency;
    /// Connection supervision timeout multiplier
    uint16_t conn_timeout;
};

/// device information data
/*@TRACE
 @trc_ref gapc_dev_info*/
union gapc_dev_info_val
{
    /// Device name
    //@trc_union parent.req == GAPC_DEV_NAME
    struct gap_dev_name name;
    /// Appearance Icon
    //@trc_union parent.req == GAPC_DEV_APPEARANCE
    uint16_t appearance;
    /// Slave preferred parameters
    //@trc_union parent.req == GAPC_DEV_SLV_PREF_PARAMS
    struct gap_slv_pref slv_pref_params;
    /// Central address resolution
    //@trc_union parent.req == GAPC_DEV_CTL_ADDR_RESOL
    uint8_t ctl_addr_resol;
};


/// Send requested info to peer device
/*@TRACE*/
struct gapc_get_dev_info_cfm
{
    /// Requested information
    /// - GAPC_DEV_NAME: Device Name
    /// - GAPC_DEV_APPEARANCE: Device Appearance Icon
    /// - GAPC_DEV_SLV_PREF_PARAMS: Device Slave preferred parameters
    uint8_t req;

    /// Peer device information data
    union gapc_dev_info_val info;
};


struct gattc_disc_cmd
{
    /// GATT request type
    uint8_t  operation;
    /// UUID length
    uint8_t  uuid_len;
    /// operation sequence number
    uint16_t seq_num;
    /// start handle range
    uint16_t start_hdl;
    /// start handle range
    uint16_t end_hdl;
    /// UUID
    uint8_t  uuid[__ARRAY_EMPTY];
};

/// Simple Read (GATTC_READ or GATTC_READ_LONG)
/*@TRACE
 gattc_read = gattc_read_simple
 gattc_read_long = gattc_read_simple*/
struct gattc_read_simple
{
    /// attribute handle
    uint16_t handle;
    /// start offset in data payload
    uint16_t offset;
    /// Length of data to read (0 = read all)
    uint16_t length;
};

/// Read by UUID: search UUID and read it's characteristic value (GATTC_READ_BY_UUID)
/// Note: it doesn't perform an automatic read long.
/*@TRACE*/
struct gattc_read_by_uuid
{
    /// Start handle
    uint16_t start_hdl;
    /// End handle
    uint16_t end_hdl;
    /// Size of UUID
    uint8_t uuid_len;
    /// UUID value
    uint8_t uuid[__ARRAY_EMPTY];
};

/// Read Multiple short characteristic (GATTC_READ_MULTIPLE)
/*@TRACE*/
struct gattc_read_multiple
{
    /// attribute handle
    uint16_t handle;
    /// Known Handle length (shall be != 0)
    uint16_t len;
};

/// request union according to read type
/*@TRACE
 @trc_ref gattc_operation
 */
union gattc_read_req
{
    /// Simple Read (GATTC_READ or GATTC_READ_LONG)
    //@trc_union parent.operation == GATTC_READ or parent.operation == GATTC_READ_LONG
    struct gattc_read_simple simple;
    /// Read by UUID (GATTC_READ_BY_UUID)
    //@trc_union parent.operation == GATTC_READ_BY_UUID
    struct gattc_read_by_uuid by_uuid;
    /// Read Multiple short characteristic (GATTC_READ_MULTIPLE)
    //@trc_union parent.operation == GATTC_READ_MULTIPLE
    struct gattc_read_multiple multiple[1];
};

/// Read command (Simple, Long, Multiple, or by UUID)
/*@TRACE*/
struct gattc_read_cmd
{
    /// request type
    uint8_t operation;
    /// number of read (only used for multiple read)
    uint8_t nb;
    /// operation sequence number
    uint16_t seq_num;
    /// request union according to read type
    union gattc_read_req req;
};

/// Parameters for @ref GATTC_SEND_SVC_CHANGED_CMD message
/*@TRACE*/
struct gattc_send_svc_changed_cmd
{
    /// Request Type
    uint8_t operation;
    /// operation sequence number
    uint16_t seq_num;
    /// Start of Affected Attribute Handle Range
    uint16_t svc_shdl;
    /// End of Affected Attribute Handle Range
    uint16_t svc_ehdl;
};

/// Write peer attribute value command
/*@TRACE*/
struct gattc_write_cmd
{
    /// Request type
    uint8_t operation;
    /// Perform automatic execution
    /// (if false, an ATT Prepare Write will be used this shall be use for reliable write)
    bool auto_execute;
    /// operation sequence number
    uint16_t seq_num;
    /// Attribute handle
    uint16_t handle;
    /// Write offset
    uint16_t offset;
    /// Write length
    uint16_t length;
    /// Internal write cursor shall be initialized to 0
    uint16_t cursor;
    /// Value to write
    uint8_t value[__ARRAY_EMPTY];
};

/// Write peer attribute value command
/*@TRACE*/
struct gattc_execute_write_cmd
{
    /// Request type
    uint8_t operation;

    /// [True = perform/False cancel] pending write operations
    bool execute;
    /// operation sequence number
    uint16_t seq_num;
};

/// Confirm reception of event (trigger a confirmation message)
/*@TRACE*/
struct gattc_event_cfm
{
    /// Attribute handle
    uint16_t handle;
};

struct gapc_encrypt_cmd
{
    /// GAP request type:
    /// - GAPC_ENCRYPT:  Start encryption procedure.
    uint8_t operation;
    /// Long Term Key information
    ble_gap_ltk_t ltk;
};

typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// True to accept slave connection parameters, False else.
    bool accept;
    /// Minimum Connection Event Duration
    uint16_t ce_len_min;
    /// Maximum Connection Event Duration
    uint16_t ce_len_max;
} ble_gap_update_conn_response_t;

/// Security requested by peer device indication message
/*@TRACE*/
struct gapc_security_ind
{
    /// Authentication level (@see gap_auth)
    uint8_t auth;
};

struct gapm_aes_h6_ind
{
    uint8_t aes_res[GAP_KEY_LEN];
    uint32_t metainfo;
};
struct gattc_svc_changed_cfg
{
    /**
     * Current value of the Client Characteristic Configuration descriptor for the Service
     * Changed characteristic
     */
    uint16_t ind_cfg;
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



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
