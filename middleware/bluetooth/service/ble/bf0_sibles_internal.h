/**
  ******************************************************************************
  * @file   bf0_sibles_internal.h
  * @author Sifli software development team
  * @brief Header file - Sibles interface exposed by BCPU.
 *
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

#ifndef BF0_SIBLES_INTERNAL_H_
#define BF0_SIBLES_INTERNAL_H_

#define APP_ST_BLE_RWIP


#include "bf0_sibles.h"
#include "bf0_sibles_util.h"
#include "ble_stack.h"
#include "ipc_queue.h"
#include "bluetooth_int.h"

/// Build the first message ID of a task. (in fact a ke_msg_id_t)
#define TASK_FIRST_MSG(task) ((uint16_t)((task) << 8))

/// Builds the task identifier from the type and the index of that task.
#define TASK_BUILD(type, index) ((uint16_t)(((index) << 8)|(type)) )

/// Retrieves task type from task id.
#define TASK_TYPE_GET(sifli_task_id) (((uint16_t)sifli_task_id) & 0xFF)

/// Retrieves task index number from task id.
#define TASK_IDX_GET(sifli_task_id) ((((uint16_t)sifli_task_id) >> 8) & 0xFF)

#define TASK_BUILD_ID(type, index) ( (sifli_task_id_t)(((index) << 8)|(type)) )

#ifndef __ARRAY_EMPTY
    #define __ARRAY_EMPTY
#endif

/// Offset for List Management Operation Codes
#define GAPM_OP_OFFSET_LIST_MGMT  (0x90)
/// Offset for Extended Air Operation Codes
#define GAPM_OP_OFFSET_EXT_AIR    (0xA0)
// double of number of completed packets
#define MAX_NUM_OF_TX_PKT 8

#define TRC_HCI_MASK (TRC_HCI_BIT)
#define TRC_ALL_MASK (0x1FFFFF)

/**
 ****************************************************************************************
 * @addtogroup BF0_Sibles_Driver Sibles
 * @ingroup BF0_HAL_Driver
 * @brief Sifli BLE Server Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
extern void sifli_mbox_bcpu2hcpu(uint8_t *param);


/// Tasks types definition, this value shall be in [0-254] range
/*@TRACE*/
enum TASK_API_ID
{
    // -----------------------------------------------------------------------------------
    // --------------------- BLE HL TASK API Identifiers ---------------------------------
    // ---------------------     SHALL NOT BE CHANGED    ---------------------------------
    // -----------------------------------------------------------------------------------

    TASK_ID_L2CC         = 10,   // L2CAP Controller Task
    TASK_ID_GATTM        = 11,   // Generic Attribute Profile Manager Task
    TASK_ID_GATTC        = 12,   // Generic Attribute Profile Controller Task
    TASK_ID_GAPM         = 13,   // Generic Access Profile Manager
    TASK_ID_GAPC         = 14,   // Generic Access Profile Controller
    TASK_ID_APP          = 15,   // Application API

    // -----------------------------------------------------------------------------------
    // --------------------- TRANSPORT AND PLATFORM TASKS --------------------------------
    // -----------------------------------------------------------------------------------
    TASK_ID_AHI          = 16,   // Application Host Interface
    TASK_ID_HCI          = 17,   //!< TASK_ID_HCI
    TASK_ID_AHI_INT      = 18,   // Application Host Interface in the same core
    TASK_ID_DISS         = 20,   // Device Information Service Server Task
    TASK_ID_SIBLES         = 71,   // Sifli BLE Server Task
    TASK_ID_COMMON         = 72,  // For system handles.

    TASK_ID_INVALID      = 0xFF, // Invalid Task Identifier
};



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

/// request operation type - application interface
/*@TRACE*/
enum gattc_operation
{
    /*              Attribute Client Flags              */
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation
    GATTC_NO_OP                                    = 0x00,

    /* Operation flags for MTU Exchange                 */
    /* ************************************************ */
    /// Perform MTU exchange
    GATTC_MTU_EXCH,

    /*      Operation flags for discovery operation     */
    /* ************************************************ */
    /// Discover all services
    GATTC_DISC_ALL_SVC,
    /// Discover services by UUID
    GATTC_DISC_BY_UUID_SVC,
    /// Discover included services
    GATTC_DISC_INCLUDED_SVC,
    /// Discover all characteristics
    GATTC_DISC_ALL_CHAR,
    /// Discover characteristic by UUID
    GATTC_DISC_BY_UUID_CHAR,
    /// Discover characteristic descriptor
    GATTC_DISC_DESC_CHAR,

    /* Operation flags for reading attributes           */
    /* ************************************************ */
    /// Read attribute
    GATTC_READ,
    /// Read long attribute
    GATTC_READ_LONG,
    /// Read attribute by UUID
    GATTC_READ_BY_UUID,
    /// Read multiple attribute
    GATTC_READ_MULTIPLE,

    /* Operation flags for writing/modifying attributes */
    /* ************************************************ */
    /// Write attribute
    GATTC_WRITE,
    /// Write no response
    GATTC_WRITE_NO_RESPONSE,
    /// Write signed
    GATTC_WRITE_SIGNED,
    /// Execute write
    GATTC_EXEC_WRITE,

    /* Operation flags for registering to peer device   */
    /* events                                           */
    /* ************************************************ */
    /// Register to peer device events
    GATTC_REGISTER,
    /// Unregister from peer device events
    GATTC_UNREGISTER,

    /* Operation flags for sending events to peer device*/
    /* ************************************************ */
    /// Send an attribute notification
    GATTC_NOTIFY,
    /// Send an attribute indication
    GATTC_INDICATE,
    /// Send a service changed indication
    GATTC_SVC_CHANGED,

    /* Service Discovery Procedure                      */
    /* ************************************************ */
    /// Search specific service
    GATTC_SDP_DISC_SVC,
    /// Search for all services
    GATTC_SDP_DISC_SVC_ALL,
    /// Cancel Service Discovery Procedure
    GATTC_SDP_DISC_CANCEL,
};


/// GAP Controller Task messages
/*@TRACE*/
enum gapc_msg_id
{
    /* Default event */
    /// Command Complete event
    GAPC_CMP_EVT = TASK_FIRST_MSG(TASK_ID_GAPC),//!< GAPC_CMP_EVT

    /* Connection state information */
    /// Indicate that a connection has been established
    GAPC_CONNECTION_REQ_IND,                    //!< GAPC_CONNECTION_REQ_IND
    /// Set specific link data configuration.
    GAPC_CONNECTION_CFM,                        //!< GAPC_CONNECTION_CFM

    /// Indicate that a link has been disconnected
    GAPC_DISCONNECT_IND,                        //!< GAPC_DISCONNECT_IND

    /* Link management command */
    /// Request disconnection of current link command.
    GAPC_DISCONNECT_CMD,                        //!< GAPC_DISCONNECT_CMD

    /* Peer device info */
    /// Retrieve information command
    GAPC_GET_INFO_CMD,                          //!< GAPC_GET_INFO_CMD
    /// Peer device attribute DB info such as Device Name, Appearance or Slave Preferred Parameters
    GAPC_PEER_ATT_INFO_IND,                     //!< GAPC_PEER_ATT_INFO_IND
    /// Indication of peer version info
    GAPC_PEER_VERSION_IND,                      //!< GAPC_PEER_VERSION_IND
    /// Indication of peer features info
    GAPC_PEER_FEATURES_IND,                     //!< GAPC_PEER_FEATURES_IND
    /// Indication of ongoing connection RSSI
    GAPC_CON_RSSI_IND,                          //!< GAPC_CON_RSSI_IND

    /* Device Name Management */
    /// Peer device request local device info such as name, appearance or slave preferred parameters
    GAPC_GET_DEV_INFO_REQ_IND,                  //!< GAPC_GET_DEV_INFO_REQ_IND
    /// Send requested info to peer device
    GAPC_GET_DEV_INFO_CFM,                      //!< GAPC_GET_DEV_INFO_CFM
    /// Peer device request to modify local device info such as name or appearance
    GAPC_SET_DEV_INFO_REQ_IND,                  //!< GAPC_SET_DEV_INFO_REQ_IND
    /// Local device accept or reject device info modification
    GAPC_SET_DEV_INFO_CFM,                      //!< GAPC_SET_DEV_INFO_CFM

    /* Connection parameters update */
    /// Perform update of connection parameters command
    GAPC_PARAM_UPDATE_CMD,                      //!< GAPC_PARAM_UPDATE_CMD
    /// Request of updating connection parameters indication
    GAPC_PARAM_UPDATE_REQ_IND,                  //!< GAPC_PARAM_UPDATE_REQ_IND
    /// Master confirm or not that parameters proposed by slave are accepted or not
    GAPC_PARAM_UPDATE_CFM,                      //!< GAPC_PARAM_UPDATE_CFM
    /// Connection parameters updated indication
    GAPC_PARAM_UPDATED_IND,                     //!< GAPC_PARAM_UPDATED_IND

    /* Bonding procedure */
    /// Start Bonding command procedure
    GAPC_BOND_CMD,                              //!< GAPC_BOND_CMD
    /// Bonding requested by peer device indication message.
    GAPC_BOND_REQ_IND,                          //!< GAPC_BOND_REQ_IND
    /// Confirm requested bond information.
    GAPC_BOND_CFM,                              //!< GAPC_BOND_CFM
    /// Bonding information indication message
    GAPC_BOND_IND,                              //!< GAPC_BOND_IND

    /* Encryption procedure */
    /// Start Encryption command procedure
    GAPC_ENCRYPT_CMD,                           //!< GAPC_ENCRYPT_CMD
    /// Encryption requested by peer device indication message.
    GAPC_ENCRYPT_REQ_IND,                       //!< GAPC_ENCRYPT_REQ_IND
    /// Confirm requested Encryption information.
    GAPC_ENCRYPT_CFM,                           //!< GAPC_ENCRYPT_CFM
    /// Encryption information indication message
    GAPC_ENCRYPT_IND,                           //!< GAPC_ENCRYPT_IND

    /* Security request procedure */
    /// Start Security Request command procedure
    GAPC_SECURITY_CMD,                          //!< GAPC_SECURITY_CMD
    /// Security requested by peer device indication message
    GAPC_SECURITY_IND,                          //!< GAPC_SECURITY_IND

    /* Signature procedure */
    /// Indicate the current sign counters to the application
    GAPC_SIGN_COUNTER_IND,                      //!< GAPC_SIGN_COUNTER_IND

    /* Device information */
    /// Indication of ongoing connection Channel Map
    GAPC_CON_CHANNEL_MAP_IND,                   //!< GAPC_CON_CHANNEL_MAP_IND

    /* Deprecated */
    /// Deprecated messages
    GAPC_DEPRECATED_0,                          //!< GAPC_DEPRECATED_0
    GAPC_DEPRECATED_1,                          //!< GAPC_DEPRECATED_1
    GAPC_DEPRECATED_2,                          //!< GAPC_DEPRECATED_2
    GAPC_DEPRECATED_3,                          //!< GAPC_DEPRECATED_3
    GAPC_DEPRECATED_4,                          //!< GAPC_DEPRECATED_4
    GAPC_DEPRECATED_5,                          //!< GAPC_DEPRECATED_5
    GAPC_DEPRECATED_6,                          //!< GAPC_DEPRECATED_6
    GAPC_DEPRECATED_7,                          //!< GAPC_DEPRECATED_7
    GAPC_DEPRECATED_8,                          //!< GAPC_DEPRECATED_8
    GAPC_DEPRECATED_9,                          //!< GAPC_DEPRECATED_9

    /* LE Ping */
    /// Update LE Ping timeout value
    GAPC_SET_LE_PING_TO_CMD,                    //!< GAPC_SET_LE_PING_TO_CMD
    /// LE Ping timeout indication
    GAPC_LE_PING_TO_VAL_IND,                    //!< GAPC_LE_PING_TO_VAL_IND
    /// LE Ping timeout expires indication
    GAPC_LE_PING_TO_IND,                        //!< GAPC_LE_PING_TO_IND

    /* LE Data Length extension*/
    /// LE Set Data Length Command
    GAPC_SET_LE_PKT_SIZE_CMD,                   //!< GAPC_SET_LE_PKT_SIZE_CMD
    /// LE Set Data Length Indication
    GAPC_LE_PKT_SIZE_IND,                       //!< GAPC_LE_PKT_SIZE_IND

    /* Secure Connections */
    /// Request to inform the remote device when keys have been entered or erased
    GAPC_KEY_PRESS_NOTIFICATION_CMD,            //!< GAPC_KEY_PRESS_NOTIFICATION_CMD
    /// Indication that a KeyPress has been performed on the peer device.
    GAPC_KEY_PRESS_NOTIFICATION_IND,            //!< GAPC_KEY_PRESS_NOTIFICATION_IND

    /* PHY Management */
    /// Set the PHY configuration for current active link
    GAPC_SET_PHY_CMD,                           //!< GAPC_SET_PHY_CMD
    /// Active link PHY configuration. Triggered when configuration is read or during an update.
    GAPC_LE_PHY_IND,                            //!< GAPC_LE_PHY_IND

    /* Channel Selection Algorithm */
    /// Indication of currently used channel selection algorithm
    /// @see struct gapc_chan_sel_algo_ind
    GAPC_CHAN_SEL_ALGO_IND,                     //!< GAPC_CHAN_SEL_ALGO_IND

    /* Preferred Slave Latency */
    /// Set the preferred slave latency (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_LATENCY_CMD,            //!< GAPC_SET_PREF_SLAVE_LATENCY_CMD
    /// Set the preferred slave event duration (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_EVT_DUR_CMD,            //!< GAPC_SET_PREF_SLAVE_EVT_DUR_CMD

    /// Indication to the task that sends the unknown message
    GAPC_UNKNOWN_MSG_IND,                       //!< GAPC_UNKNOWN_MSG_IND
    // ---------------------- INTERNAL API ------------------------
    /* Internal messages for timer events, not part of API*/
    /// Signature procedure
    GAPC_SIGN_CMD,                              //!< GAPC_SIGN_CMD
    /// Signature result
    GAPC_SIGN_IND,                              //!< GAPC_SIGN_IND

    /// Parameter update procedure timeout indication
    GAPC_PARAM_UPDATE_TO_IND,                   //!< GAPC_PARAM_UPDATE_TO_IND
    /// Pairing procedure timeout indication
    GAPC_SMP_TIMEOUT_TIMER_IND,                 //!< GAPC_SMP_TIMEOUT_TIMER_IND
    /// Pairing repeated attempts procedure timeout indication
    GAPC_SMP_REP_ATTEMPTS_TIMER_IND,            //!< GAPC_SMP_REP_ATTEMPTS_TIMER_IND

    GAPC_PARAM_UPDATE_L2CAP_CMD,
};


/// Messages for Device Information Service Server
/*@TRACE*/
enum sibles_msg_id
{
    /// APP Write value to Sibles
    ///Set the value of an attribute - Request
    SIBLES_SET_VALUE_REQ = TASK_FIRST_MSG(TASK_ID_SIBLES),  //!< SIBLES_SET_VALUE_REQ
    ///Set the value of an attribute - Response
    SIBLES_SET_VALUE_RSP,                                   //!< SIBLES_SET_VALUE_RSP

    /// Sibles ask APP for data if not initialized, when peer device try to read.
    /// Peer device request to get profile attribute value
    SIBLES_VALUE_REQ_IND,                                   //!< SIBLE_VALUE_REQ_IND
    /// Peer device confirm value of requested attribute
    SIBLES_VALUE_REQ_CFM,                                   //!< SIBLES_VALUE_REQ_CFM

    /// Sibles report to APP that a value has changed, when peer device wrote a attribute.
    /// Indicate a value changes, the change is NOT saved in BLE core. Use SIBLES_VALUE_REQ_IND
    /// to update the firmware cache.
    SIBLES_VALUE_WRITE_IND,                                 //!< SIBLES_VALUE_WRITE_IND
    /// APP confirm got the changed value
    SIBLES_VALUE_WRITE_CFM,                                 //!< SIBLES_VALUE_WRITE_CFM

    /// Application report to Peer that a value has changed
    SIBLES_VALUE_IND_REQ,                                   //!< SIBLES_VALUE_IND_REQ
    /// GATTC Inform application when sending of Service Changed indications has been enabled or disabled
    SIBLES_VALUE_IND_RSP,                                   //!< SIBLES_VALUE_IND_RSP
    /// Messages for init and de-init
    /// Register a service
    SIBLES_SVC_REG_REQ,                                     //!< SIBLES_SVC_REG_REQ
    SIBLES_SVC_REG128_REQ,                                  //!< SIBLES_SVC_REG128_REQ
    /// Register confirmation.
    SIBLES_SVC_RSP,                                         //!< SIBLES_SVC_CFM

    /// GAP commands
    /// Set Adv data
    SIBLES_ADV_DATA_REQ,                                    // !<SIBLES_ADV_DATA_REQ
    /// Set Scan response data
    SIBLES_SCAN_RSP_REQ,                                    // !<SIBLES_SCAN_RSP_REQ
    /// Toggle ADV state
    SIBLES_ADV_CMD_REQ,                                     // !<SIBLES_ADV_CMD_REQ
    /// Update device name
    SIBLES_NAME_REQ,                                        //!<SIBLES_NAME_REQ
    /// Response to GAP commands
    SIBLES_CMD_RSP,                                         //!<SIBLES_CMD_RSP

    SIBLES_SVC_SEARCH_REQ,
    SIBLES_DISC_SVC_IND,
    SIBLES_DISC_CHAR_IND,
    SIBLES_DISC_CHAR_DESC_IND,
    SIBLES_SVC_SEARCH_RSP,
    SIBLES_REGISTER_NOTIFY_REQ,
    SIBLES_REGISTER_NOTIFY_RSP,
    SIBLES_VALUE_WRITE_REQ,
    SIBLES_EVENT_IND,
    SIBLES_VALUE_READ_REQ,
    SIBLES_VALUE_READ_RSP,

    SIBLES_CONNECTED_IND,

    SIBLES_VALUE_NTF_IND,

    SIBLES_SET_STATIC_RANDOM_ADDR_REQ,

    SIBLES_WLAN_COEX_ENABLE_REQ,

    SIBLES_TRC_CFG_REQ,

    SIBLES_WRTIE_VALUE_RSP,

    SIBLES_ENABLE_DBG,

    SIBLES_CH_BD_ADDR,
    SIBLES_CH_BD_ADDR_RSP,

    SIBLES_UNREGISTER_NOTIFY_REQ,

    SIBLES_UPDATE_ATT_PERM_REQ,
    SIBLES_UPDATE_ATT_PERM_RSP,

    //  Sibles ready
    SIBLES_SVC_READY_IND = 0xFF,                            //!< SIBLES_SVC_READY_IND
};


enum app_msg_id
{
    APPM_DUMMY_MSG = TASK_FIRST_MSG(TASK_ID_APP),

    APP_SIFLI_NVDS_GET_REQ,
    APP_SIFLI_NVDS_GET_CNF,

    APP_SIFLI_NVDS_SET_REQ,
    APP_SIFLI_NVDS_SET_CNF,

};

enum gattc_msg_id
{
    /* Default event */
    /// Command Complete event
    GATTC_CMP_EVT = TASK_FIRST_MSG(TASK_ID_GATTC),

    /* ATTRIBUTE CLIENT */
    /// Server configuration request
    GATTC_EXC_MTU_CMD,
    /// Indicate that the ATT MTU has been updated (negotiated)
    GATTC_MTU_CHANGED_IND,

    /*Discover All Services */
    /*Discover Services by Service UUID*/
    /*Find Included Services*/
    /*Discover Characteristics by UUID*/
    /*Discover All Characteristics of a Service*/
    /*Discover All Characteristic Descriptors*/
    /// Discovery command
    GATTC_DISC_CMD,
    /* GATT -> HL: Events to Upper layer */
    /*Discover All Services*/
    /// Discovery services indication
    GATTC_DISC_SVC_IND,
    /*Find Included Services*/
    /// Discover included services indication
    GATTC_DISC_SVC_INCL_IND,
    /*Discover All Characteristics of a Service*/
    /// Discover characteristic indication
    GATTC_DISC_CHAR_IND,
    /*Discover All Characteristic Descriptors*/
    /// Discovery characteristic descriptor indication
    GATTC_DISC_CHAR_DESC_IND,

    /*Read Value*/
    /*Read Using UUID*/
    /*Read Long Value*/
    /*Read Multiple Values*/
    /// Read command
    GATTC_READ_CMD,
    /// Read response
    GATTC_READ_IND,

    /*Write without response*/
    /*Write without response with Authentication*/
    /*Write Characteristic Value*/
    /*Signed Write Characteristic Value*/
    /*Write Long Characteristic Value*/
    /*Characteristic Value Reliable Write*/
    /*Write Characteristic Descriptors*/
    /*Write Long Characteristic Descriptors*/
    /*Characteristic Value Reliable Write*/
    /// Write command request
    GATTC_WRITE_CMD,

    /* Cancel / Execute pending write operations */
    /// Execute write characteristic request
    GATTC_EXECUTE_WRITE_CMD,

    /* Reception of an indication or notification from peer device. */
    /// peer device triggers an event (notification)
    GATTC_EVENT_IND,
    /// peer device triggers an event that requires a confirmation (indication)
    GATTC_EVENT_REQ_IND,
    /// Confirm reception of event (trigger a confirmation message)
    GATTC_EVENT_CFM,

    /// Registration to peer device events (Indication/Notification).
    GATTC_REG_TO_PEER_EVT_CMD,

    /* -------------------------- ATTRIBUTE SERVER ------------------------------- */
    /*Notify Characteristic*/
    /*Indicate Characteristic*/
    /// send an event to peer device
    GATTC_SEND_EVT_CMD,

    /* Service Changed Characteristic Indication */
    /**
     * Send a Service Changed indication to a device
     * (message structure is struct gattm_svc_changed_ind_req)
     */
    GATTC_SEND_SVC_CHANGED_CMD,
    /**
     * Inform the application when sending of Service Changed indications has been
     * enabled or disabled
     */
    GATTC_SVC_CHANGED_CFG_IND,

    /* Indicate that read operation is requested. */
    /// Read command indicated to upper layers.
    GATTC_READ_REQ_IND,
    /// REad command confirmation from upper layers.
    GATTC_READ_CFM,

    /* Indicate that write operation is requested. */
    /// Write command indicated to upper layers.
    GATTC_WRITE_REQ_IND,
    /// Write command confirmation from upper layers.
    GATTC_WRITE_CFM,

    /* Indicate that write operation is requested. */
    /// Request Attribute info to upper layer - could be trigger during prepare write
    GATTC_ATT_INFO_REQ_IND,
    /// Attribute info from upper layer confirmation
    GATTC_ATT_INFO_CFM,

    /* ----------------------- SERVICE DISCOVERY PROCEDURE  --------------------------- */
    /// Service Discovery command
    GATTC_SDP_SVC_DISC_CMD,
    /// Service Discovery indicate that a service has been found.
    GATTC_SDP_SVC_IND,

    /* -------------------------- TRANSACTION ERROR EVENT ----------------------------- */
    /// Transaction Timeout Error Event no more transaction will be accepted
    GATTC_TRANSACTION_TO_ERROR_IND,

    /// Indication to the task that sends the unknown message
    GATTC_UNKNOWN_MSG_IND,

    /* ------------------------------- Internal API ----------------------------------- */
    /// Client Response timeout indication
    GATTC_CLIENT_RTX_IND,
    /// Server indication confirmation timeout indication
    GATTC_SERVER_RTX_IND,
};

enum diss_msg_id
{
    ///Set the value of an attribute - Request
    DISS_SET_VALUE_REQ = TASK_FIRST_MSG(TASK_ID_DISS),//!< DISS_SET_VALUE_REQ
    ///Set the value of an attribute - Response
    DISS_SET_VALUE_RSP,                               //!< DISS_SET_VALUE_RSP

    /// Peer device request to get profile attribute value
    DISS_VALUE_REQ_IND,                               //!< DISS_VALUE_REQ_IND
    /// Peer device confirm value of requested attribute
    DISS_VALUE_CFM,                                   //!< DISS_VALUE_CFM

    // set dis value by upper user - request.
    DISS_APP_SET_VALUE_REQ,
    // set dis value by upper user - response.
    DISS_APP_SET_VALUE_RSP,
};

enum comm_msg_id
{
    /// Enter BT test mode
    COMM_BT_TEST_MODE_CTRL_CMD = TASK_FIRST_MSG(TASK_ID_COMMON),
    /// Indicate end of BT test mode
    COMM_BT_TEST_MODE_CTRL_RSP,
};

enum sifli_task_evt_id
{
    SIFLI_TASK_RECV_EVT = 0x01,
    SIFLI_TASK_TRAN_EVT = 0x02,
#ifdef SIBLES_TRANSPARENT_ENABLE
    SIFLI_TASK_TRAN_OUTPUT_EVT = 0x04,
#endif
    SIFLI_TASK_EVT_ALL = SIFLI_TASK_RECV_EVT | SIFLI_TASK_TRAN_EVT
#ifdef SIBLES_TRANSPARENT_ENABLE
                         | SIFLI_TASK_TRAN_OUTPUT_EVT
#endif
    ,
};


enum dbg_trc_cfg_fields
{
    /// Kernel message
    TRC_KE_MSG_POS              = 0,
    TRC_KE_MSG_BIT              = 0x00000001,

    /// Kernel timer
    TRC_KE_TMR_POS              = 1,
    TRC_KE_TMR_BIT              = 0x00000002,

    /// Kernel event
    TRC_KE_EVT_POS              = 2,
    TRC_KE_EVT_BIT              = 0x00000004,

    /// Memory allocation and deallocation
    TRC_MEM_POS                 = 3,
    TRC_MEM_BIT                 = 0x00000008,

    /// Sleep mode
    TRC_SLEEP_POS               = 4,
    TRC_SLEEP_BIT               = 0x00000010,

    /// Software Assert
    TRC_SW_ASS_POS              = 5,
    TRC_SW_ASS_BIT              = 0x00000020,

    /// Programming of the exchange table, updating of the event counter and handling of "End of Event" interrupt
    TRC_PROG_POS                = 6,
    TRC_PROG_BIT                = 0x00000040,

    /// BLE Control structure
    TRC_CS_BLE_POS              = 7,
    TRC_CS_BLE_BIT              = 0x00000080,

    /// BT Control structure
    TRC_CS_BT_POS               = 8,
    TRC_CS_BT_BIT               = 0x00000100,

    /// Processing of RX descriptors at each LLD driver
    TRC_RX_DESC_POS             = 9,
    TRC_RX_DESC_BIT             = 0x00000200,

    /// LLCP transmission, reception and acknowledgment
    TRC_LLCP_POS                = 10,
    TRC_LLCP_BIT                = 0x00000400,

    /// LMP transmission, reception and acknowledgment
    TRC_LMP_POS                 = 11,
    TRC_LMP_BIT                 = 0x00000800,

    /// L2CAP transmission, reception and acknowledgment
    TRC_L2CAP_POS               = 12,
    TRC_L2CAP_BIT               = 0x00001000,

    /// Scheduling request, cancellation, shift and remove
    TRC_ARB_POS                 = 13,
    TRC_ARB_BIT                 = 0x00002000,

    /// LLC state transition
    TRC_LLC_STATE_TRANS_POS     = 14,
    TRC_LLC_STATE_TRANS_BIT     = 0x00004000,

    /// LC state transition
    TRC_LC_STATE_TRANS_POS      = 15,
    TRC_LC_STATE_TRANS_BIT      = 0x00008000,

    /// HCI messages (in Full embedded mode)
    TRC_HCI_POS                 = 16,
    TRC_HCI_BIT                 = 0x00010000,

    /// Advertising pdu
    TRC_ADV_POS                 = 17,
    TRC_ADV_BIT                 = 0x00020000,

    /// ACL pdu
    TRC_ACL_POS                 = 18,
    TRC_ACL_BIT                 = 0x00040000,

    /// Custom trace
    TRC_CUSTOM_POS              = 19,
    TRC_CUSTOM_BIT              = 0x00080000,

    /// ACL truncate trace
    TRC_HCI_ACL_TRUNCATE_POS    = 21,
    TRC_HCI_ACL_TRUNCATE_BIT    = 0x00200000,
};

///Attribute Table Indexes
enum diss_info
{
    /// Manufacturer Name
    DIS_MANUFACTURER_NAME_CHAR,
    /// Model Number
    DIS_MODEL_NB_STR_CHAR,
    /// Serial Number
    DIS_SERIAL_NB_STR_CHAR,
    /// HW Revision Number
    DIS_HARD_REV_STR_CHAR,
    /// FW Revision Number
    DIS_FIRM_REV_STR_CHAR,
    /// SW Revision Number
    DIS_SW_REV_STR_CHAR,
    /// System Identifier Name
    DIS_SYSTEM_ID_CHAR,
    /// IEEE Certificate
    DIS_IEEE_CHAR,
    /// Plug and Play Identifier
    DIS_PNP_ID_CHAR,

    DIS_CHAR_MAX,
};

/*
 * API MESSAGES STRUCTURES
 ****************************************************************************************
 * For value name, if not set, will use lower 16bits of attribute UUID as name.
 * Please set other name bytes to 0.
 *
 */


///SIBLES_SET_VALUE_REQ: Set the value of an attribute - Request
///SIBLES_VALUE_REQ_CFM: Peer device  value of requested attribute
struct sibles_value
{
    /// Value to Set
    uint8_t hdl;
    /// Value length
    uint16_t length;
    /// Value data
    uint8_t data[__ARRAY_EMPTY];
};


///SIBLES_SET_VALUE_RSP:Set the value of an attribute - Response
///SIBLES_VALUE_WRITE_CFM: APP confirm the value changes
struct sibles_value_ack
{
    /// Value Set
    uint8_t hdl;
    /// status of the request
    uint8_t status;
};


/// SIBLES_VALUE_REQ_IND: Peer device request to get profile attribute value
struct sibles_value_req_ind
{
    /// Requested value
    uint8_t hdl;
};

///SIBLES_VALUE_WRITE_IND:Peer set a value change to APP
struct sibles_value_write_ind
{
    /// Value Set
    uint16_t hdl;
    /// Value offset
    uint16_t offset;
    /// Value length
    uint16_t length;
    /// Value data
    uint8_t is_cmd;
    uint8_t data[__ARRAY_EMPTY];
};

/// SIBLES_SVC_REG_REQ: Register one BLE service.
struct sibles_svc_reg_req
{
    uint16_t svc_uuid;
    uint8_t sec_lvl;
    uint8_t attm_entries;
    struct attm_desc att_db[__ARRAY_EMPTY];
};

/// SIBLES_SVC_REG128_REQ: Register one BLE service with 128 bit UUID.
struct sibles_svc_reg128_req
{
    uint8_t svc_uuid[ATT_UUID_128_LEN];
    uint8_t sec_lvl;
    uint8_t attm_entries;
    uint16_t reserved;
    struct attm_desc_128 att_db[__ARRAY_EMPTY];
};

struct sibles_update_att_perm_req
{
    uint16_t handle;
    uint16_t access_mask;
    uint16_t perm;
};

struct sibles_update_att_perm_rsp
{
    uint16_t handle;
    uint8_t status;
};

typedef struct
{
    uint8_t      hdl;               /*!< Service record handle */
    uint16_t     msg;               /*!< Message ID */
    uint16_t     len;               /*!< Length of value*/
    uint8_t     *value;             /*!< Value content */
} sibles_send_value_t;

/// SIBLES_SVC_RSP: Confirm on SVC register.
struct sibles_svc_rsp
{
    uint8_t status;
    uint8_t start_hdl;
};

///
struct sibles_svc_search_req
{
    /// Search all if uuid is zero.
    uint8_t conn_idx;
    uint8_t len;
    uint8_t svc_uuid[ATT_UUID_128_LEN];
};

struct sibles_svc_search_rsp
{
    uint8_t status;
    uint8_t len;
    uint8_t svc_uuid[ATT_UUID_128_LEN];
};


/// Discover Service indication Structure
/*@TRACE*/
struct sibles_disc_svc_ind
{
    /// start handle
    uint16_t start_hdl;
    /// end handle
    uint16_t end_hdl;
    /// UUID length
    uint8_t  uuid_len;
    /// service UUID
    uint8_t  uuid[ATT_UUID_128_LEN];
};

/// Discovery All Characteristic indication Structure
/*@TRACE*/
struct sibles_disc_char_ind
{
    /// database element handle
    uint16_t attr_hdl;
    /// pointer attribute handle to UUID
    uint16_t pointer_hdl;
    /// properties
    uint8_t prop;
    /// UUID length
    uint8_t uuid_len;
    /// characteristic UUID
    uint8_t uuid[ATT_UUID_128_LEN];
};




struct sibles_register_notify_req_t
{
    uint16_t hdl_start;
    uint16_t hdl_end;
    uint16_t seq_num;
};

struct sibles_register_notify_rsp_t
{
    uint8_t status;
    uint16_t seq_num;
};

struct sibles_value_write_req_t
{
    uint8_t write_type;
    uint16_t seq_num;
    /// Attribute handle
    uint16_t handle;
    /// Write length
    uint16_t length;
    /// Value to write
    uint8_t value[__ARRAY_EMPTY];
};

struct sibles_value_write_req_content_t
{
    uint8_t conn_idx;
    uint8_t write_type;
    uint16_t seq_num;
    /// Attribute handle
    uint16_t handle;
    /// Write length
    uint16_t length;
    /// Value to write
    uint8_t value[__ARRAY_EMPTY];
};

/// peer device triggers an event (notification)
/*@TRACE*/
struct gattc_event_ind
{
    /// Event Type
    uint8_t type;
    /// Data length
    uint16_t length;
    /// Attribute handle
    uint16_t handle;
    /// Event Value
    uint8_t value[__ARRAY_EMPTY];
};

struct sibles_value_read_req_t
{
    uint8_t read_type;
    uint16_t seq_num;
    uint16_t handle;
    uint16_t offset;
    uint16_t length;
};

/// Attribute value read indication
/*@TRACE*/
struct gattc_read_ind
{
    /// Attribute handle
    uint16_t handle;
    /// Read offset
    uint16_t offset;
    /// Read length
    uint16_t length;
    /// Handle value
    uint8_t value[__ARRAY_EMPTY];
};

/// Service Discovery Command Structure
/*@TRACE*/
struct gattc_exc_mtu_cmd
{
    /// GATT request type
    uint8_t operation;
    /// operation sequence number
    uint16_t seq_num;
};


/// Indicate that the ATT MTU has been updated (negotiated)
/*@TRACE*/
struct gattc_mtu_changed_ind
{
    /// Exchanged MTU value
    uint16_t mtu;
    /// operation sequence number
    uint16_t seq_num;
};

struct sibles_random_addr
{
    uint8_t addr_type;
    uint8_t addr[BD_ADDR_LEN];
};

struct sibles_wlan_coex_enable_req_t
{
    uint8_t enable;
};

struct sibles_trc_cfg_req_t
{
    uint32_t mask;
};

struct gapm_dbg_rssi_notify_ind
{
    uint8_t is_bt;
    int8_t channel_asset[79];
    uint8_t rssi_num;
    int8_t rssi_ave[__ARRAY_EMPTY];
};

struct sibles_ch_bd_addr_t
{
    uint8_t addr_type;
    uint8_t addr_method;
    uint8_t addr[BD_ADDR_LEN];
};

struct sibles_ch_bd_addr_rsp_t
{
    uint8_t status;
};

// device infomation
typedef struct
{
    ///@ enum diss_info
    uint8_t value;
    uint8_t len;
    ///device information string
    uint8_t data[__ARRAY_EMPTY];
} app_dis_set_req_t;

// device infomation
typedef struct
{
    ///@ enum diss_info
    uint8_t value;
    uint8_t status;
} app_dis_set_rsp_t;

/// Possible states of the SIBLES task
enum
{
    /// Not USED state
    SIBLES_EMPTY,
    /// Idle state
    SIBLES_IDLE,
    /// Ready state
    SIBLES_READY,
    /// Busy state
    SIBLES_BUSY,
    /// Number of defined states.
    SIBLES_STATE_MAX
};

#define SIBLE_CFM_FLAG 0x4000       // Use this bit in len to indicate this setvalue is a confirmation or not. 
#define SIBLE_FREE_FLAG 0x8000      // Use this bit in len to indicate sibles to free the buffer.  
#define SIBLE_FLAG_ALL 0xC000

#define MAX_MAIL_SIZE 1024

char *msg_str(uint16_t msg);

ipc_queue_handle_t sifli_get_mbox_read_dev(void);

ipc_queue_handle_t sifli_get_mbox_write_dev(void);

#ifdef APP_ST_BLE_RWIP
    void sifli_AHI_process(void);
#endif

uint8_t *sifli_get_mbox_buff(void);

rt_err_t sifli_mbox_send(uint8_t *data);

rt_err_t silfi_mbox_notify(uint32_t evt);

void sifli_env_init(void);


void sifli_sem_take(void);

void sifli_sem_release(void);

void sifli_mbox_process(sibles_msg_para_t *header, uint8_t *data_ptr, uint16_t param_len);


void ble_gap_event_process(sibles_msg_para_t *header, uint8_t *data_ptr, uint16_t param_len);

void ble_nvds_config_prepare();

/// @} BF0_HAL_Driver
#endif // BF0_SIBLES_H_

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
