/**
  ******************************************************************************
  * @file   bf0_ble_gap.h
  * @author Sifli software development team
  * @brief Header file - Bluetooth GAP protocol interface.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2020 - 2021,  Sifli Technology
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

#ifndef __BF0_BLE_GAP_H
#define __BF0_BLE_GAP_H

/*
 * INCLUDE FILES
 ****************************************************************************************
    */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#ifdef BSP_USING_PC_SIMULATOR
    #include <stdbool.h>
#endif

#include "bf0_ble_err.h"
#include "bf0_ble_common.h"
#include "bf0_sibles.h"


/**
  ****************************************************************************************
  * @addtogroup GAP GAP Interface
  * @ingroup middleware
  * @brief Generic Access Profile defines the device discovery , link management and related security procedures.
  * @{
  */



/*
 * DEFINES
 ****************************************************************************************
 */


#define CO_BIT(pos) (1UL<<(pos))         /**< return Value with one bit set.  There is no return type since this is a macro and this
                                          **  will be resolved by the compiler upon assignment to an l-value. */


#define GAP_KEY_LEN           (16)       /**< Key length. */
#define GAP_ACTV_INVALID_IDX   (0xFF)    /**< Invalid activity identifier. */
#define GAP_LE_FEATS_LEN      (0x08)     /**< LE Feature Flags Length. */
#define GAP_RAND_NB_LEN       (0x08)     /**< Random number length. */
#define GAP_ADV_DATA_LEN      (0x1F)     /**< The maximum ADV Data length. */
#define GAP_SCAN_RSP_DATA_LEN (0x1F)     /**< The maximum scan response length. */
#define GAP_MIN_OCTETS  (27)             /**< minimum data size in octets. */
#define GAP_MIN_TIME    (328)            /**< minimum data transfer time in us. */
#define GAP_MAX_OCTETS  (251)            /**< maximum data size in octets. */
#define GAP_MAX_TIME    (17040)          /**< maximum data size in octets. */
#define GAP_LE_CHNL_MAP_LEN   (0x05)     /**< LE Channel map length. */
#ifndef SOC_SF32LB55X
    #define GAP_MAX_LOCAL_NAME_LEN (0x1A)    /**< Maximum Local device name. */
#else
    #define GAP_MAX_LOCAL_NAME_LEN (0x12)    /**< Maximum Local device name. */
#endif

#define GAP_DEFAULT_LOCAL_NAME "SIFLI-BLE-DEV"  /**< Default Local device name. */



/// Random Address type.
enum gap_rnd_addr_type
{
    /// Static random address           - 11 (MSB->LSB)
    GAP_STATIC_ADDR     = 0xC0,
    /// Private non resolvable address  - 01 (MSB->LSB)
    GAP_NON_RSLV_ADDR   = 0x00,
    /// Private resolvable address      - 01 (MSB->LSB)
    GAP_RSLV_ADDR       = 0x40,
};



/**
 * @brief GAP events that notify user.
 */
enum ble_gap_event_t
{

    BLE_GAP_LOCAL_VER_IND = BLE_GAP_TYPE,    /**< This event indicates local version. */
    BLE_GAP_LOCAL_BD_ADDR_IND,               /**< This event indicates local bd address. */
    BLE_GAP_LOCAL_ADV_TX_POWER_IND,          /**< This event indicates advertising channel TX power */
    BLE_GAP_LOCAL_MAX_DATA_LEN_IND,          /**< This event indicates mamimum data len that controller supported. */
    BLE_GAP_NUMBER_ADV_SETS_IND,             /**< This event indicates maximum number of adv sets. */
    BLE_GAP_LOCAL_MAX_ADV_DATA_LEN_IND,      /**< This event indicates maximum adv data len and scan rsp len that controller
                                                  supported. */
    BLE_GAP_SET_WHITE_LIST_CNF,              /**< This event indicates the result of set white list. */
    BLE_GAP_SET_RESOLVING_LIST_CNF,          /**< This event indicates the result of set resolving list. */
    BLE_GAP_RAL_ADDR_IND,                    /**< This event indicates current resolvable address. */
    BLE_GAP_SET_IRK_CNF,                     /**< This event indicates the result of set IRK. */
    BLE_GAP_ADV_CREATED_IND,                 /**< This event indicates advertising created. */
    BLE_GAP_CREATE_ADV_CNF,                  /**< This event indicates the result of created advertising. */
    BLE_GAP_SET_ADV_DATA_CNF,                /**< This event indicates the result of set advertising data. */
    BLE_GAP_SET_SCAN_RSP_DATA_CNF,           /**< This event indicates the result of set scan rsp data. */
    BLE_GAP_START_ADV_CNF,                   /**< This event indicates the result of start advertising. */
    BLE_GAP_ADV_STOPPED_IND,                 /**< This event indicates advertising stopped. */
    BLE_GAP_STOP_ADV_CNF,                    /**< This event indicates the result of stop advertising. */
    BLE_GAP_DELETE_ADV_CNF,                  /**< This event indicates the result of delete advertising. */
    BLE_GAP_SCAN_START_CNF,                  /**< This event indicates the result of scan start. */
    BLE_GAP_SCAN_STOPPED_IND,                /**< This event indicates scan stopped. */
    BLE_GAP_SCAN_STOP_CNF,                   /**< This event indicates the result of scan stop. */
    BLE_GAP_CREATE_CONNECTION_CNF,           /**< This event indicates the result of start connection. */
    BLE_GAP_CANCEL_CREATE_CONNECTION_CNF,    /**< This event indicates the result of cancel connection. */
    BLE_GAP_CONNECTED_IND,                   /**< This event indicates connection setup. */
    BLE_GAP_DISCONNECT_CNF,                  /**< This event indicates the result of disconnect. */
    BLE_GAP_DISCONNECTED_IND,                /**< This event indicates link disconnected. */
    BLE_GAP_REMOTE_VER_IND,                  /**< This event indicates peer device bluetooth version. */
    BLE_GAP_REMOTE_FEATURE_IND,              /**< This event indicates peer device supported BLE features. */
    BLE_GAP_REMOTE_RSSI_IND,                 /**< This event indicates on-going link RSSI. */
    BLE_GAP_REMOTE_PHY_IND,                  /**< This event indicates on-goling link PHY configuration. */
    BLE_GAP_UPDATE_CONN_PARAM_IND,           /**< This event indicates on-going link connection parameter updatad */
    BLE_GAP_UPDATE_CONN_PARAM_CNF,           /**< This event indicates the result of update connection parameter. */
    BLE_GAP_BOND_CNF,                        /**< This event indicates the result of bond command. */
    BLE_GAP_BOND_REQ_IND,                    /**< This event indicates bond request by peer device. */
    BLE_GAP_BOND_IND,                        /**< This event indicates bond information. */
    BLE_GAP_ENCRYPT_REQ_IND,                 /**< This event indicates encrypt request by peer device. */
    BLE_GAP_ENCRYPT_IND,                     /**< This event indicates encrypt information. */
    BLE_GAP_SECURITY_REQUEST_CNF,            /**< This event indicates the result of security request. */
    BLE_GAP_SECURITY_REQUEST_IND,            /**< This event indicates security request by peer deivce in peripherial role. */
    BLE_GAP_PASSKEY_REQ_IND,                 /**< This event indicates passkey request by peer device. */
    BLE_GAP_PHY_UPDATE_IND,                  /**< This event indicates PHY updated. */
    BLE_GAP_EXT_ADV_REPORT_IND,              /**< This evnet indicates extended advertising report which including
                                                  either legacy or extended advertising PDUs. */
    BLE_GAP_UPDATE_DATA_LENGTH_CNF,          /**< This event indicates the result of update data length  */
    BLE_GAP_UPDATE_DATA_LENGTH_IND,          /**< This event indicates the on-going link data length updated */
    BLE_GAP_UPDATE_CHANNEL_MAP_CNF,          /**< This event indicates the result of channel map updated */
    BLE_GAP_UPDATE_CHANNEL_MAP_IND,          /**< This event indicates the on-going link channel map updated */
    BLE_GAP_SIGN_COUNTER_UPDATE_IND,         /**< This event indicates the local or peer sign counter updated */
    BLE_GAP_RESOLVE_ADDRESS_CNF,             /**< This event indicates the address resolved result */
    BLE_GAP_SOLVED_ADDRESS_IND,              /**< This event indicates the solved address with dedicated key */
    BLE_GAP_PUBLIC_KEY_GEN_IND,              /**< This event indicates public key has generated */
    BT_DBG_RSSI_NOTIFY_IND,                  /**< This debug event indicates the link rssi and channel assessment */
    BT_DBG_ASSERT_NOTIFY_IND,                /**< This debug event indicates the controller is crash */
    BLE_GAP_SET_PERIODIC_ADV_DATA_CNF,       /**< This event indicates the result of set periodic rsp data. */
    BLE_GAP_CREATE_PERIODIC_ADV_SYNC_CNF,    /**< This event indicates the result of create periodic advertising sync. */
    BLE_GAP_START_PERIODIC_ADV_SYNC_CNF,     /**< This event indicates the result of start periodic advertising sync. */
    BLE_GAP_STOP_PERIODIC_ADV_SYNC_CNF,      /**< This event indicates the result of stop periodic advertising sync. */
    BLE_GAP_DELETE_PERIODIC_ADV_SYNC_CNF,    /**< This event indicates the result of delete periodic advertising sync. */
    BLE_GAP_PERIODIC_ADV_SYNC_CREATED_IND,   /**< This event indicates periodic advertising sync created. */
    BLE_GAP_PERIODIC_ADV_SYNC_STOPPED_IND,   /**< This event indicates periodic advertising sync stopped. */
    BLE_GAP_PERIODIC_ADV_SYNC_ESTABLISHED_IND, /**< This event indicates periodic advertising sync established. */
};

/**
 * @brief GAP Advertising Flags.
 */
enum ble_gap_adv_type
{
    /// Flag
    BLE_GAP_AD_TYPE_FLAGS                      = 0x01, /**< Flag. */
    BLE_GAP_AD_TYPE_MORE_16_BIT_UUID           = 0x02, /**< Use of more than 16 bits UUID. */
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID  = 0x03, /**< Complete list of 16 bit UUID. */
    BLE_GAP_AD_TYPE_MORE_32_BIT_UUID           = 0x04, /**< Use of more than 32 bit UUD. */
    BLE_GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID  = 0x05, /**< Complete list of 32 bit UUID. */
    BLE_GAP_AD_TYPE_MORE_128_BIT_UUID          = 0x06, /**< Use of more than 128 bit UUID. */
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID = 0x07, /**< Complete list of 128 bit UUID. */
    BLE_GAP_AD_TYPE_SHORTENED_NAME             = 0x08, /**< Shortened device name. */
    BLE_GAP_AD_TYPE_COMPLETE_NAME              = 0x09, /**< Complete device name. */
    BLE_GAP_AD_TYPE_TRANSMIT_POWER             = 0x0A, /**< Transmit power. */
    BLE_GAP_AD_TYPE_CLASS_OF_DEVICE            = 0x0D, /**< Class of device. */
    BLE_GAP_AD_TYPE_SP_HASH_C                  = 0x0E, /**< Simple Pairing Hash C. */
    BLE_GAP_AD_TYPE_SP_RANDOMIZER_R            = 0x0F, /**< Simple Pairing Randomizer. */
    BLE_GAP_AD_TYPE_TK_VALUE                   = 0x10, /**< Temporary key value. */
    BLE_GAP_AD_TYPE_OOB_FLAGS                  = 0x11, /**< Out of Band Flag. */
    BLE_GAP_AD_TYPE_SLAVE_CONN_INT_RANGE       = 0x12, /**< Slave connection interval range. */
    BLE_GAP_AD_TYPE_RQRD_16_BIT_SVC_UUID       = 0x14, /**< Require 16 bit service UUID. */
    BLE_GAP_AD_TYPE_RQRD_32_BIT_SVC_UUID       = 0x1F, /**< Require 32 bit service UUID. */
    BLE_GAP_AD_TYPE_RQRD_128_BIT_SVC_UUID      = 0x15, /**< Require 128 bit service UUID. */
    BLE_GAP_AD_TYPE_SERVICE_16_BIT_DATA        = 0x16, /**< Service data 16-bit UUID. */
    BLE_GAP_AD_TYPE_SERVICE_32_BIT_DATA        = 0x20, /**< Service data 32-bit UUID. */
    BLE_GAP_AD_TYPE_SERVICE_128_BIT_DATA       = 0x21, /**< Service data 128-bit UUID. */
    BLE_GAP_AD_TYPE_PUB_TGT_ADDR               = 0x17, /**< Public Target Address. */
    BLE_GAP_AD_TYPE_RAND_TGT_ADDR              = 0x18, /**< Random Target Address. */
    BLE_GAP_AD_TYPE_APPEARANCE                 = 0x19, /**< Appearance. */
    BLE_GAP_AD_TYPE_ADV_INTV                   = 0x1A, /**< Advertising Interval. */
    BLE_GAP_AD_TYPE_LE_BT_ADDR                 = 0x1B, /**< LE Bluetooth Device Address. */
    BLE_GAP_AD_TYPE_LE_ROLE                    = 0x1C, /**< LE Role. */
    BLE_GAP_AD_TYPE_SPAIR_HASH                 = 0x1D, /**< Simple Pairing Hash C-256. */
    BLE_GAP_AD_TYPE_SPAIR_RAND                 = 0x1E, /**< Simple Pairing Randomizer R-256. */
    BLE_GAP_AD_TYPE_3D_INFO                    = 0x3D, /**< 3D Information Data. */

    /// Manufacturer specific data
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA         = 0xFF, /**< Manufacturer specific data. */
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


///Advertising filter policy
enum adv_filter_policy
{
    ///Allow both scan and connection requests from anyone
    ADV_ALLOW_SCAN_ANY_CON_ANY    = 0x00,
    ///Allow both scan req from White List devices only and connection req from anyone
    ADV_ALLOW_SCAN_WLST_CON_ANY,
    ///Allow both scan req from anyone and connection req from White List devices only
    ADV_ALLOW_SCAN_ANY_CON_WLST,
    ///Allow scan and connection requests from White List devices only
    ADV_ALLOW_SCAN_WLST_CON_WLST,
};


///Advertising channels enables
enum adv_channel_map
{
    ///Byte value for advertising channel map for channel 37 enable
    ADV_CHNL_37_EN                = 0x01,
    ///Byte value for advertising channel map for channel 38 enable
    ADV_CHNL_38_EN                = 0x02,
    ///Byte value for advertising channel map for channel 39 enable
    ADV_CHNL_39_EN                = 0x04,
    ///Byte value for advertising channel map for channel 37, 38 and 39 enable
    ADV_ALL_CHNLS_EN              = 0x07,
};


/// PHY Type
enum gap_phy_type
{
    /// LE 1M
    GAP_PHY_TYPE_LE_1M = 1,
    /// LE 2M
    GAP_PHY_TYPE_LE_2M,
    /// LE Coded
    GAP_PHY_TYPE_LE_CODED,
};

/// Periodic synchronization types
enum gap_per_sync_type
{
    /// Do not use periodic advertiser list for synchronization. Use advertiser information provided
    /// in the ble_gap_start_periodic_advertising_sync().
    GAP_PER_SYNC_TYPE_GENERAL = 0,
    /// Use periodic advertiser list for synchronization
    GAP_PER_SYNC_TYPE_SELECTIVE,
};



/// Security Defines
enum gap_sec_req
{
    /// No security (no authentication and encryption)
    GAP_NO_SEC = 0x00,
    /// Unauthenticated pairing with encryption
    GAP_SEC1_NOAUTH_PAIR_ENC,
    /// Authenticated pairing with encryption
    GAP_SEC1_AUTH_PAIR_ENC,
    /// Unauthenticated pairing with data signing
    GAP_SEC2_NOAUTH_DATA_SGN,
    /// Authentication pairing with data signing
    GAP_SEC2_AUTH_DATA_SGN,
    /// Secure Connection pairing with encryption
    GAP_SEC1_SEC_CON_PAIR_ENC,
};

/// Key Distribution Flags
enum gap_kdist
{
    /// No Keys to distribute
    GAP_KDIST_NONE = 0x00,
    /// Encryption key in distribution
    GAP_KDIST_ENCKEY = (1 << 0),
    /// IRK (ID key)in distribution
    GAP_KDIST_IDKEY  = (1 << 1),
    /// CSRK(Signature key) in distribution
    GAP_KDIST_SIGNKEY = (1 << 2),
    /// LTK in distribution
    GAP_KDIST_LINKKEY = (1 << 3),

    GAP_KDIST_LAST = (1 << 4)
};

/// IO Capability Values
enum gap_io_cap
{
    /// Display Only
    GAP_IO_CAP_DISPLAY_ONLY = 0x00,
    /// Display Yes No
    GAP_IO_CAP_DISPLAY_YES_NO,
    /// Keyboard Only
    GAP_IO_CAP_KB_ONLY,
    /// No Input No Output
    GAP_IO_CAP_NO_INPUT_NO_OUTPUT,
    /// Keyboard Display
    GAP_IO_CAP_KB_DISPLAY,
    GAP_IO_CAP_LAST
};

/// OOB Data Present Flag Values
enum gap_oob
{
    /// OOB Data not present
    GAP_OOB_AUTH_DATA_NOT_PRESENT = 0x00,
    /// OOB data present
    GAP_OOB_AUTH_DATA_PRESENT,
    GAP_OOB_AUTH_DATA_LAST
};

/// Authentication mask
enum gap_auth_mask
{
    /// No Flag set
    GAP_AUTH_NONE    = 0,
    /// Bond authentication
    GAP_AUTH_BOND    = (1 << 0),
    /// Man In the middle protection
    GAP_AUTH_MITM    = (1 << 2),
    /// Secure Connection
    GAP_AUTH_SEC_CON = (1 << 3),
    /// Key Notification
    GAP_AUTH_KEY_NOTIF = (1 << 4)
};


/// Authentication Requirements
enum gap_auth
{
    /// No MITM No Bonding
    GAP_AUTH_REQ_NO_MITM_NO_BOND  = (GAP_AUTH_NONE),
    /// No MITM Bonding
    GAP_AUTH_REQ_NO_MITM_BOND     = (GAP_AUTH_BOND),
    /// MITM No Bonding
    GAP_AUTH_REQ_MITM_NO_BOND     = (GAP_AUTH_MITM),
    /// MITM and Bonding
    GAP_AUTH_REQ_MITM_BOND        = (GAP_AUTH_MITM | GAP_AUTH_BOND),
    /// SEC_CON and No Bonding
    GAP_AUTH_REQ_SEC_CON_NO_BOND  = (GAP_AUTH_SEC_CON),
    /// SEC_CON and Bonding
    GAP_AUTH_REQ_SEC_CON_BOND     = (GAP_AUTH_SEC_CON | GAP_AUTH_BOND),
    /// SEC CON and Bonding and MITM
    GAP_AUTH_REQ_SEC_CON_MITM_BOND = (GAP_AUTH_SEC_CON | GAP_AUTH_BOND | GAP_AUTH_MITM),

    GAP_AUTH_REQ_LAST,

    /// Mask of  authentication features without reserved flag
    GAP_AUTH_REQ_MASK             = 0x1F,
};

/// Bond event type.
/*@TRACE*/
enum gapc_bond
{
    /// Bond Pairing request
    GAPC_PAIRING_REQ,
    /// Respond to Pairing request
    GAPC_PAIRING_RSP,

    /// Pairing Finished information
    GAPC_PAIRING_SUCCEED,
    /// Pairing Failed information
    GAPC_PAIRING_FAILED,

    /// Used to retrieve pairing Temporary Key
    GAPC_TK_EXCH,
    /// Used for Identity Resolving Key exchange
    GAPC_IRK_EXCH,
    /// Used for Connection Signature Resolving Key exchange
    GAPC_CSRK_EXCH,
    /// Used for Long Term Key exchange
    GAPC_LTK_EXCH,

    /// Bond Pairing request issue, Repeated attempt
    GAPC_REPEATED_ATTEMPT,

    /// Out of Band - exchange of confirm and rand.
    GAPC_OOB_EXCH,

    /// Numeric Comparison - Exchange of Numeric Value -
    GAPC_NC_EXCH
};

/// TK Type
enum gap_tk_type
{
    ///  TK get from out of band method
    GAP_TK_OOB         = 0x00,
    /// TK generated and shall be displayed by local device
    GAP_TK_DISPLAY,
    /// TK shall be entered by user using device keyboard
    GAP_TK_KEY_ENTRY
};


/// Bit field use to select the preferred TX or RX LE PHY. 0 means no preferences
enum gap_phy
{
    /// No preferred PHY
    GAP_PHY_ANY               = 0x00,
    /// LE 1M PHY preferred for an active link
    GAP_PHY_LE_1MBPS          = (1 << 0),
    /// LE 2M PHY preferred for an active link
    GAP_PHY_LE_2MBPS          = (1 << 1),
    /// LE Coded PHY preferred for an active link
    GAP_PHY_LE_CODED          = (1 << 2),
};

/// Option for PHY configuration
enum gapc_phy_option
{
    /// No preference for rate used when transmitting on the LE Coded PHY
    GAPC_PHY_OPT_LE_CODED_ALL_RATES     = (1 << 0),
    /// 500kbps rate preferred when transmitting on the LE Coded PHY
    GAPC_PHY_OPT_LE_CODED_500K_RATE     = (1 << 1),
    /// 125kbps  when transmitting on the LE Coded PHY
    GAPC_PHY_OPT_LE_CODED_125K_RATE     = (1 << 2),
};

/// Packet Payload type for test mode
enum gap_pkt_pld_type
{
    /// PRBS9 sequence "11111111100000111101..." (in transmission order)
    GAP_PKT_PLD_PRBS9,
    /// Repeated "11110000" (in transmission order)
    GAP_PKT_PLD_REPEATED_11110000,
    /// Repeated "10101010" (in transmission order)
    GAP_PKT_PLD_REPEATED_10101010,
    /// PRBS15 sequence
    GAP_PKT_PLD_PRBS15,
    /// Repeated "11111111" (in transmission order) sequence
    GAP_PKT_PLD_REPEATED_11111111,
    /// Repeated "00000000" (in transmission order) sequence
    GAP_PKT_PLD_REPEATED_00000000,
    /// Repeated "00001111" (in transmission order) sequence
    GAP_PKT_PLD_REPEATED_00001111,
    /// Repeated "01010101" (in transmission order) sequence
    GAP_PKT_PLD_REPEATED_01010101,
};

/// Enumeration of TX/RX PHY used for Test Mode
enum gap_test_phy
{
    /// LE 1M PHY (TX or RX)
    GAP_TEST_PHY_1MBPS        = 1,
    /// LE 2M PHY (TX or RX)
    GAP_TEST_PHY_2MBPS        = 2,
    /// LE Coded PHY (RX Only)
    GAP_TEST_PHY_CODED        = 3,
    /// LE Coded PHY with S=8 data coding (TX Only)
    GAP_TEST_PHY_125KBPS      = 3,
    /// LE Coded PHY with S=2 data coding (TX Only)
    GAP_TEST_PHY_500KBPS      = 4,
};

/// Modulation index
enum gap_modulation_idx
{
    /// Assume transmitter will have a standard modulation index
    GAP_MODULATION_STANDARD,
    /// Assume transmitter will have a stable modulation index
    GAP_MODULATION_STABLE,
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


/**
 * @brief BD address structure
 */
typedef struct
{
    /// BD Address of device
    bd_addr_t addr;
    /// Address type of the device 0=public/1=private random
    uint8_t addr_type;

} ble_gap_addr_t;

/// Resolving list device information
/*@TRACE*/
typedef struct
{
    /// Device identity
    ble_gap_addr_t addr;
    /// Privacy Mode
    uint8_t priv_mode;
    /// Peer IRK
    uint8_t peer_irk[GAP_KEY_LEN];
    /// Local IRK
    uint8_t local_irk[GAP_KEY_LEN];
} ble_gap_ral_dev_info_t;


/**
 * @brief White list structure.
 */
typedef struct
{
    uint8_t size;                               /**< Number of entries in white list. */
    ble_gap_addr_t addr[__ARRAY_EMPTY];         /**< Bd address list of white list. */
} ble_gap_white_list_t;

/**
 * @brief The struture of set resolving list request.
 */
typedef struct
{
    uint8_t size;                               /**< Number of entries in resovling list. */
    ble_gap_ral_dev_info_t ral[__ARRAY_EMPTY];  /**< Resovling inforamtion list of resovling list. */
} ble_gap_resolving_list_t;


typedef struct
{
    ble_gap_addr_t peer_addr;
} ble_gap_ral_addr_t;

/**
 * @brief The struture of security key.
 */
typedef struct
{
    /// Key value MSB -> LSB
    uint8_t key[GAP_KEY_LEN];
} ble_gap_sec_key_t;

/**
 * @brief The struture of random number.
 */
typedef struct
{
    ///8-byte array for random number
    uint8_t     nb[GAP_RAND_NB_LEN];
} ble_gap_rand_nb_t;

/// Long Term Key information
/*@TRACE*/

typedef struct
{
    /// Long Term Key
    ble_gap_sec_key_t ltk;
    /// Encryption Diversifier
    uint16_t ediv;
    /// Random Number
    ble_gap_rand_nb_t randnb;
    /// Encryption key size (7 to 16)
    uint8_t key_size;
} ble_gap_ltk_t;

/// Identity Resolving Key information
/*@TRACE*/
typedef struct
{
    /// Identity Resolving Key
    ble_gap_sec_key_t irk;
    /// Device BD Identity Address
    ble_gap_addr_t addr;
} ble_gap_irk_t;


/// Out of Band Information
/*@TRACE*/
typedef struct
{
    /// Confirm Value
    uint8_t conf[GAP_KEY_LEN];
    /// Random Number
    uint8_t rand[GAP_KEY_LEN];
} ble_gap_oob_t;


/**
 * @brief The struture of number comparison data.
 */
typedef struct
{
    uint8_t value[4];               /**< number conparison value. */
} ble_gap_nc_t;


/**
 * @brief The struture of set local device name request.
 */
typedef struct
{
    uint16_t len;                     /**< Name length. */
    uint8_t name[__ARRAY_EMPTY];     /**< Deivce name. */
} ble_gap_dev_name_t;


/**
 * @brief The struture of set slave preference parameter request.
 */
typedef struct
{
    /// Connection interval minimum
    uint16_t con_intv_min;
    /// Connection interval maximum
    uint16_t con_intv_max;
    /// Slave latency
    uint16_t slave_latency;
    /// Connection supervision timeout multiplier
    uint16_t conn_timeout;
} ble_gap_slave_prf_param_t;

/// Configuration for advertising on primary channel
/*@TRACE*/
typedef struct
{
    /// Minimum advertising interval (in unit of 625us). Must be greater than 20ms
    uint32_t adv_intv_min;
    /// Maximum advertising interval (in unit of 625us). Must be greater than 20ms
    uint32_t adv_intv_max;
    /// Bit field indicating the channel mapping (@see enum adv_channel_map).
    uint8_t chnl_map;
    /// Indicate on which PHY primary advertising has to be performed (@see enum gap_phy_type)
    /// Note that LE 2M PHY is not allowed and that legacy advertising only support LE 1M PHY
    uint8_t phy;
} gapm_adv_prim_cfg_t;


/// Configuration for advertising on secondary channel
typedef struct
{
    /// Maximum number of advertising events the controller can skip before sending the
    /// AUX_ADV_IND packets. 0 means that AUX_ADV_IND PDUs shall be sent prior each
    /// advertising events
    uint8_t max_skip;
    /// Indicate on which PHY secondary advertising has to be performed (@see enum gap_phy_type)
    uint8_t phy;
    /// Advertising SID
    uint8_t adv_sid;
} gapm_adv_second_cfg_t;

/// Configuration for periodic advertising
typedef struct
{
    /// Minimum advertising interval (in unit of 1.25ms). Must be greater than 20ms
    uint16_t adv_intv_min;
    /// Maximum advertising interval (in unit of 1.25ms). Must be greater than 20ms
    uint16_t adv_intv_max;
} gapm_adv_period_cfg_t;



/**
 * @brief The struture of advertising parmeter.
 */
typedef struct
{
    /// Own address type (@see enum gapm_own_addr)
    uint8_t own_addr_type;
    /// Advertising type (@see enum gapm_adv_type)
    uint8_t type;
    /// Discovery mode (@see enum gapm_adv_disc_mode)
    uint8_t disc_mode;
    /// Bit field value provided advertising properties (@see enum gapm_adv_prop for bit signification)
    uint16_t prop;
    /// Maximum power level at which the advertising packets have to be transmitted
    /// (between -127 and 126 dBm, 127 dbm use default value in stack)
    int8_t max_tx_pwr;
    /// Advertising filtering policy (@see enum adv_filter_policy)
    uint8_t filter_pol;
    /// Peer address configuration (only used in case of directed advertising)
    ble_gap_addr_t peer_addr;
    /// Configuration for primary advertising
    gapm_adv_prim_cfg_t prim_cfg;
    /// Configuration for secondary advertising (valid only if advertising type is
    /// GAPM_ADV_TYPE_EXTENDED)
    gapm_adv_second_cfg_t second_cfg;
    /// Configuration for periodic advertising (valid only if advertising type os
    /// GAPM_ADV_TYPE_PERIODIC)
    gapm_adv_period_cfg_t period_cfg;
} ble_gap_adv_parameter_t;


/**
 * @brief The struture of advertising data and scan response data.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Data length
    uint16_t length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
} ble_gap_adv_data_t;

/**
 * @brief The struture of advertising start parameter.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Advertising duration (in unit of 10ms). 0 means that advertising continues
    /// until the host disable it
    /// (between 10 to 655350 ms)
    uint16_t duration;
    /// Maximum number of extended advertising events the controller shall attempt to send prior to
    /// terminating the extending advertising
    /// Valid only if extended advertising
    uint8_t max_adv_evt;
} ble_gap_adv_start_t;

/**
 * @brief The struture of advertising stop parameter.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
} ble_gap_adv_stop_t;


/**
 * @brief The struture of advertising delete parameter.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
} ble_gap_adv_delete_t;

/**
 * @brief The struture of scan window parameter.
 */
typedef struct
{
    /// Scan interval in unit of 0.625ms
    /// (between 2.5ms to 40.959375s)
    uint16_t scan_intv;
    /// Scan window in unit of 0.625ms
    /// (between 2.5ms to 40.959375s)
    uint16_t scan_wd;
} ble_gap_scan_param_t;

/**
 * @brief The struture of scan start parameter.
 */
typedef struct
{
    /// own addr type. (@see enum gapm_own_addr)
    uint8_t own_addr_type;
    /// Type of scanning to be started (@see enum gapm_scan_type)
    uint8_t type;
    /// Properties for the scan procedure (@see enum gapm_scan_prop for bit signification)
    uint8_t prop;
    /// Duplicate packet filtering policy
    uint8_t dup_filt_pol;
    /// Reserved for future use
    uint8_t rsvd;
    /// Scan window opening parameters for LE 1M PHY
    ble_gap_scan_param_t scan_param_1m;
    /// Scan window opening parameters for LE Coded PHY
    /// reception of a stop command from the application
    uint16_t duration;
    /// Scan period (in unit of 1.28s). Time interval betweem two consequent starts of a scan duration
    /// by the controller. 0 means that the scan procedure is not periodic
    /// (between 1.28s to 83884.8s)
    uint16_t period;
} ble_gap_scan_start_t;

/**
 * @brief The struture of scan stop parameter.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
} ble_gap_scan_stop_t;


/**
 * @brief The struture of connection parameter.
 */
typedef struct
{

    /// Scan interval (in unit of 625us)
    /// (between 2.5ms to 40.959375s)
    uint16_t scan_intv;
    /// Scan window (in unit of 0.625us)
    /// (between 2.5ms to 40.959375s)
    uint16_t scan_wd;
    /// Minimum value for the connection interval (in unit of 1.25ms). Shall be less than or equal to
    /// conn_intv_max value. Allowed range is 7.5ms to 4s.
    uint16_t conn_intv_min;
    /// Maximum value for the connection interval (in unit of 1.25ms). Shall be greater than or equal to
    /// conn_intv_min value. Allowed range is 7.5ms to 4s.
    uint16_t conn_intv_max;
    /// Slave latency. Number of events that can be missed by a connected slave device.
    /// Allowed range is 0 to ((supervision_to / (conn_intv)) - 1)
    uint16_t conn_latency;
    /// Link supervision timeout (in unit of 10ms). Allowed range is 100ms to 32s
    uint16_t supervision_to;
    /// Recommended minimum duration of connection events (in unit of 625us)
    uint16_t ce_len_min;
    /// Recommended maximum duration of connection events (in unit of 625us)
    uint16_t ce_len_max;
} ble_gap_conn_param_t;

/**
 * @brief The struture of create connection parameter.
 */
typedef struct
{
    /// own addr type. (@see enum gapm_own_addr)
    uint8_t own_addr_type;
    /// Initiating type (@see enum gapm_init_type)
    uint8_t type;
    /// Properties for the initiating procedure (@see enum gapm_init_prop for bit signification)
    //uint8_t prop;
    /// Timeout for automatic connection establishment (in unit of 10ms). Cancel the procedure if not all
    /// indicated devices have been connected when the timeout occurs. 0 means there is no timeout
    uint16_t conn_to;
    /// Connection parameters for LE 1M PHY
    ble_gap_conn_param_t conn_param_1m;
    /// Address of peer device in case white list is not used for connection
    ble_gap_addr_t peer_addr;

} ble_gap_connection_create_param_t;

/**
 * @brief The struture of start periodic advertising sync parameter.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Periodic synchronization type (@see enum gap_per_sync_type)
    uint8_t type;
    /// Advertiser address information
    ble_gap_addr_t addr;
    /// Advertising SID
    uint8_t adv_sid;
    /// value is 499
    uint16_t skip;
    /// Synchronization timeout for the periodic advertising (in unit of 10ms between 100ms and 163.84s)
    uint16_t sync_to;
} ble_gap_periodic_advertising_sync_start_t;

/**
 * @brief The struture of stop periodic advertising sync parameter.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
} ble_gap_eriodic_advertising_sync_stop_t;


/**
 * @brief The struture of delete periodic advertising sync parameter.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
} ble_gap_eriodic_advertising_sync_delete_t;


typedef struct
{
    uint8_t conn_idx;  /**< Connection index. */
    /// Local CSRK value
    ble_gap_sec_key_t lcsrk;
    /// Local signature counter value that acquired from #BLE_GAP_SIGN_COUNTER_UPDATE_IND
    uint32_t lsign_counter;

    /// Remote CSRK value that distributed from #BLE_GAP_BOND_IND (if info == GAPC_CSRK_EXCH)
    ble_gap_sec_key_t rcsrk;
    /// Remote signature counter value that acquired from #BLE_GAP_SIGN_COUNTER_UPDATE_IND
    uint32_t rsign_counter;

    /// Authentication (@see gap_auth) from #BLE_GAP_BOND_IND (if info == GAPC_PAIRING_SUCCEED)
    uint8_t auth;
    /// LTK exchanged from #BLE_GAP_BOND_IND (if info == GAPC_PAIRING_SUCCEED)
    bool ltk_present;
} ble_gap_connection_response_t;

/**
 * @brief The structure of disconnect request.
 */
typedef struct
{
    uint8_t conn_idx;                        /**< Connection index. */
    uint8_t reason;                          /**< Disconnect Reason(@see enum co_error). */
} ble_gap_disconnect_t;

/**
 * @brief The structure of get remote version  request.
 */
typedef struct
{
    uint8_t conn_idx;                        /**< Connection index. */
} ble_gap_get_remote_version_t;

/**
 * @brief The structure of get remote feature  request.
 */
typedef struct
{
    uint8_t conn_idx;                        /**< Connection index. */
} bt_gap_get_remote_feature_t;

/**
 * @brief The structure of get RSSI request.
 */
typedef struct
{
    uint8_t conn_idx;                        /**< Connection index. */
} ble_gap_get_rssi_t;

/**
 * @brief The structure of get remote physical request.
 */
typedef struct
{
    uint8_t conn_idx;                        /**< Connection index. */
} ble_gap_get_phy_t;

/**
 * @brief The structure of update connection parameter request.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Connection interval minimum. Shall be less than or equal to
    /// intv_max value. Allowed range is 7.5ms to 4s
    uint16_t intv_min;
    /// Connection interval maximum. Shall be greater than or equal to
    /// intv_min value. Allowed range is 7.5ms to 4s
    uint16_t intv_max;
    /// Latency. Allowed range is 0 to ((time_out / (conn_intv)) - 1)
    uint16_t latency;
    /// Supervision timeout (in unit of 10ms). Allowed range is 100ms to 32s
    uint16_t time_out;
    /// Minimum Connection Event Duration (in unit of 625us)
    uint16_t ce_len_min;
    /// Maximum Connection Event Duration (in unit of 625us)
    uint16_t ce_len_max;

} ble_gap_update_conn_param_t;


/** @brief Initialize update connection parameters
 *
 *  @param _conn_idx  conn_idx for dedicated BLE connection.
 *  @param _intv_min  minimum interval (N * 1.25 ms).
 *  @param _intv_max   maximum interval (N * 1.25 ms).
 *  @param _latency   maximum peripherial latency in connection (N * 0.625 ms).
 *  @param _timeout   Create connection scan window (N * 100 ms).
 */
#define BLE_GAP_UPDATE_CONN_PARAM_INIT(_conn_idx, _intv_min, _intv_max, _latency, _timeout) \
{ \
    .conn_idx = _conn_idx, \
    .intv_min = _intv_min, \
    .intv_max = _intv_max, \
    .latency = _latency, \
    .time_out = _timeout, \
    .ce_len_min = 0, \
    .ce_len_max = 12, \
}


/** Helper to declare update connection parameters inline
 *
 *  @param _conn_idx  conn_idx for dedicated BLE connection.
 *  @param _intv_min  minimum interval (N * 1.25 ms).
 *  @param _intv_max   maximum interval (N * 1.25 ms).
 *  @param _latency   maximum peripherial latency in connection (N * 0.625 ms).
 *  @param _timeout   Create connection scan window (N * 100 ms).
 */
#define BLE_GAP_CREATE_UPDATE_CONN_PARA(_conn_idx, _intv_min, _intv_max, _latency, _timeout) \
    ((ble_gap_update_conn_param_t[]) { \
        BLE_GAP_UPDATE_CONN_PARAM_INIT(_conn_idx, _intv_min, _intv_max, _latency, _timeout) \
    })


/** Default fast connection parameters.
 * It's for scenarios which need high throughput
 */
#define BLE_GAP_UPDATE_CONN_PARA_FAST(_conn_idx) \
    BLE_GAP_CREATE_UPDATE_CONN_PARA(_conn_idx, 24, 24, 0, 500)


/** Default slow connection parameters.
 * It's for scenarios which need low power
 */
#define BLE_GAP_UPDATE_CONN_PARA_SLOW(_conn_idx) \
    BLE_GAP_CREATE_UPDATE_CONN_PARA(_conn_idx, 96, 120, 4, 500)


/**
 * @brief The structure of pair information.
 */
typedef struct
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
} ble_gap_pairing_t;

/**
 * @brief The structure of bond request.
 */
typedef struct
{
    uint8_t conn_idx;                       /**< Connection index. */
    ble_gap_pairing_t pair_info;            /**< Pair info struct. */
} ble_gap_bond_t;

/*@TRACE
 @trc_ref gapc_bond
*/
typedef union
{
    /// Pairing Features (request = GAPC_PAIRING_RSP)
    //@trc_union parent.request == GAPC_PAIRING_RSP
    ble_gap_pairing_t pairing_feat;
    /// LTK (request = GAPC_LTK_EXCH)
    //@trc_union parent.request == GAPC_LTK_EXCH
    ble_gap_ltk_t ltk;
    /// CSRK (request = GAPC_CSRK_EXCH)
    //@trc_union parent.request == GAPC_CSRK_EXCH
    ble_gap_sec_key_t csrk;
    /// TK (request = GAPC_TK_EXCH)
    //@trc_union parent.request == GAPC_TK_EXCH
    ble_gap_sec_key_t tk;
    /// IRK (request = GAPC_IRK_EXCH)
    //@trc_union parent.request == GAPC_IRK_EXCH
    ble_gap_irk_t irk;
    /// OOB Confirm and Random from the peer (request = GAPC_OOB_EXCH)
    //@trc_union parent.request == GAPC_OOB_EXCH
    ble_gap_oob_t oob;
} ble_gap_bond_cfm_data_t;

/**
 * @brief The structure of bond confirm.
 */
typedef struct
{
    uint8_t conn_idx;                           /**< Connection index. */
    uint8_t request;                            /**< Bond request type (@see gapc_bond). */
    uint8_t accept;                             /**< Request accepted. */
    ble_gap_bond_cfm_data_t cfm_data;           /**< bond confirm data structure. */
} ble_gap_bond_confirm_t;


/**
 * @brief The structure of bond confirm.
 */
typedef struct
{
    uint8_t conn_idx;                           /**< Connection index. */
    uint8_t found;                              /**< Indicate if a LTK has been found for the peer device. */
    ble_gap_sec_key_t ltk;                      /**< Long Term Key. */
    uint8_t key_size;                           /**< LTK Key Size. */
} ble_gap_encrypt_confirm_t;

/**
 * @brief The structure of security request.
 */
typedef struct
{
    uint8_t conn_idx;                           /**< Connection index. */
    uint8_t auth;                               /**< Authentication level (@see gap_auth). */
} ble_gap_sec_req_t;

typedef struct
{
    uint8_t conn_idx;
    /// Supported LE PHY for data transmission (@see enum gap_phy)
    uint8_t tx_phy;
    /// Supported LE PHY for data reception (@see enum gap_phy)
    uint8_t rx_phy;
    /// PHY options (@see enum gapc_phy_option)
    uint8_t phy_opt;

} ble_gap_update_phy_t;

typedef struct
{
    uint8_t conn_idx;
    ///Preferred maximum number of payload octets that the local Controller should include
    ///in a single Link Layer Data Channel PDU.
    uint16_t tx_octets;
    ///Preferred maximum number of microseconds that the local Controller should use to transmit
    ///a single Link Layer Data Channel PDU
    uint16_t tx_time;
} ble_gap_update_data_len_t;


typedef struct
{
    /// Number of provided IRK (sahlle be > 0)
    uint8_t nb_key;
    /// Resolvable random address to solve
    bd_addr_t addr;
    /// Array of IRK used for address resolution (MSB -> LSB)
    ble_gap_sec_key_t irk[__ARRAY_EMPTY];
} ble_gap_resolve_address_t;

/// Event

/// Local device version indication event. The structure of #BLE_GAP_LOCAL_VER_IND.
/*@TRACE*/
typedef struct
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
}  ble_gap_local_version_ind_t;


/// Local device BD Address indication event. The structure of #BLE_GAP_LOCAL_BD_ADDR_IND.
/*@TRACE*/
typedef struct
{
    /// Local device address information
    ble_gap_addr_t addr;
    /// Activity index
    uint8_t actv_idx;
} ble_gap_dev_bdaddr_ind_t;


/// Advertising channel Tx power level indication event. The structure of #BLE_GAP_LOCAL_ADV_TX_POWER_IND.
/*@TRACE*/
typedef struct
{
    /// Advertising channel Tx power level
    int8_t     power_lvl;
} ble_gap_dev_adv_tx_power_ind_t;


/// Indicates maximum data length. The structure of #BLE_GAP_LOCAL_MAX_DATA_LEN_IND.
/*@TRACE*/
typedef struct
{
    ///Maximum number of payload octets that the local Controller supports for transmission
    uint16_t suppted_max_tx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for transmission
    uint16_t suppted_max_tx_time;
    ///Maximum number of payload octets that the local Controller supports for reception
    uint16_t suppted_max_rx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for reception
    uint16_t suppted_max_rx_time;
} ble_gap_max_data_len_ind_t;

/// Number of available advertising sets indication event. The structure of #BLE_GAP_NUMBER_ADV_SETS_IND.
/*@TRACE*/
typedef struct
{
    /// Number of available advertising sets
    uint8_t nb_adv_sets;
} ble_gap_nb_adv_sets_ind_t;


/// Maximum advertising data length indication event. The structure of #BLE_GAP_LOCAL_MAX_ADV_DATA_LEN_IND.
/*@TRACE*/
typedef struct
{
    /// Maximum advertising data length supported by controller.
    uint16_t length;
} ble_gap_max_adv_data_len_ind_t;




/**
 * @brief The structure of channel map.
 */
typedef struct
{
    uint8_t channel_map[GAP_LE_CHNL_MAP_LEN];
} ble_gap_update_channel_map_t;

/**
 * @brief The structure of #BLE_GAP_UPDATE_CHANNEL_MAP_IND.
 */
typedef struct
{
    uint8_t channel_map[GAP_LE_CHNL_MAP_LEN];
} ble_gap_update_channel_map_ind_t;



/**
 * @brief The structure of #BLE_GAP_ADV_CREATED_IND.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Selected TX power for advertising activity
    int8_t  tx_pwr;
} ble_gap_adv_created_ind_t;

/**
 * @brief The structure of #BLE_GAP_ADV_STOPPED_IND.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Stopped reason
    uint8_t reason;
    /// In case of periodic advertising, indicate if periodic advertising has been stopped
    int8_t  per_adv_stop;
} ble_gap_adv_stopped_ind_t;

/**
 * @brief The structure of #BLE_GAP_SCAN_STOPPED_IND.
 */
typedef struct
{
    /// Stopped reason
    uint8_t reason;
} ble_gap_scan_stopped_ind_t;



typedef struct
{
    uint8_t status;
} ble_gap_status_t;

typedef ble_gap_status_t ble_gap_set_white_list_cnf_t;                   /**< The struture of #BLE_GAP_SET_WHITE_LIST_CNF. */

typedef ble_gap_status_t ble_gap_set_resolve_list_cnf_t;                 /**< The struture of #BLE_GAP_SET_RESOLVING_LIST_CNF. */

typedef ble_gap_status_t ble_gap_set_irk_cnf_t;                          /**< The struture of #BLE_GAP_SET_IRK_CNF. */

typedef ble_gap_status_t ble_gap_create_adv_cnf_t;                       /**< The struture of #BLE_GAP_CREATE_ADV_CNF. */

typedef ble_gap_status_t ble_gap_set_adv_data_cnf_t;                     /**< The strcture of #BLE_GAP_SET_ADV_DATA_CNF. */

typedef ble_gap_status_t ble_gap_set_scan_rsp_data_cnf_t;                 /**< The strcture of #BLE_GAP_SET_SCAN_RSP_DATA_CNF. */

typedef ble_gap_status_t ble_gap_set_periodic_adv_data_cnf_t;             /**< The strcture of #BLE_GAP_SET_PERIODIC_ADV_DATA_CNF. */

typedef ble_gap_status_t ble_gap_start_scan_cnf_t;                          /**< The struture of #BLE_GAP_SCAN_START_CNF. */

typedef ble_gap_status_t ble_gap_stop_scan_cnf_t;                          /**< The struture of #BLE_GAP_SCAN_STOP_CNF. */

typedef ble_gap_status_t ble_gap_create_connection_cnf_t;                  /**< The strucutre of #BLE_GAP_CREATE_CONNECTION_CNF. */

typedef ble_gap_status_t ble_gap_cancel_create_connection_cnf_t;            /**< The struture of #BLE_GAP_CANCEL_CREATE_CONNECTION_CNF. */

typedef ble_gap_status_t ble_gap_resolve_address_cnf_t;  /**< The struture of #BLE_GAP_RESOLVE_ADDRESS_CNF. */

typedef ble_gap_status_t ble_gap_create_per_adv_sync_cnf_t;                /**< The struture of #BLE_GAP_CREATE_PERIODIC_ADV_SYNC_CNF. */


/**
 * @brief The struture of #BLE_GAP_START_ADV_CNF.
 */
typedef struct
{
    /// Activity identifier.
    uint8_t actv_index;
    /// Start result.
    uint8_t status;
} ble_gap_start_adv_cnf_t;


/**
 * @brief The struture of #BLE_GAP_STOP_ADV_CNF.
 */
typedef struct
{
    /// Activity identifier.
    uint8_t actv_index;
    /// Start result.
    uint8_t status;
} ble_gap_stop_adv_cnf_t;

/**
 * @brief The struture of #BLE_GAP_DELETE_ADV_CNF.
 */
typedef struct
{
    /// Activity identifier.
    uint8_t actv_index;
    /// Delete result.
    uint8_t status;
} ble_gap_delete_adv_cnf_t;

/**
 * @brief The struture of #BLE_GAP_UPDATE_CONN_PARAM_CNF.
 */
typedef struct
{
    uint8_t conn_idx;               /**< Connection index. */
    uint8_t status;                 /**< Update request result  */
} ble_gap_update_conn_param_cnf_t;

/**
 * @brief The struture of #BLE_GAP_DISCONNECT_CNF.
 */
typedef struct
{
    uint8_t conn_idx;               /**< Connection index. */
    uint8_t status;                 /**< Disconnect request result. */
} ble_gap_disconnect_cnf_t;



/// Resolving Address indication event. The structure of #BLE_GAP_RAL_ADDR_IND. */
/*@TRACE*/
typedef struct
{
    /// Resolving List address
    ble_gap_addr_t addr;
} ble_gap_ral_addr_ind_t;

/**
 * @brief Provide link configrations.
 */
typedef struct
{
    /// Local CSRK value
    ble_gap_sec_key_t lcsrk;
    /// Local signature counter value that acquired from #BLE_GAP_SIGN_COUNTER_UPDATE_IND
    uint32_t lsign_counter;

    /// Remote CSRK value that distributed from #BLE_GAP_BOND_IND (if info == GAPC_CSRK_EXCH)
    ble_gap_sec_key_t rcsrk;
    /// Remote signature counter value that acquired from #BLE_GAP_SIGN_COUNTER_UPDATE_IND
    uint32_t rsign_counter;

    /// Authentication (@see gap_auth) from #BLE_GAP_BOND_IND (if info == GAPC_PAIRING_SUCCEED)
    uint8_t auth;
    /// LTK exchanged from #BLE_GAP_BOND_IND (if info == GAPC_PAIRING_SUCCEED)
    bool ltk_present;
    /// CFG not respond directly
    uint8_t not_respond;
} ble_gap_connect_configure_t;

/**
 * @brief The structure of #BLE_GAP_CONNECTED_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Connection interval, time = N * 1.25ms
    uint16_t con_interval;
    /// Connection latency
    uint16_t con_latency;
    /// Link supervision timeout, time = N * 10 ms
    uint16_t sup_to;
    /// Peer address type
    uint8_t peer_addr_type;
    /// Peer BT address
    bd_addr_t peer_addr;
    /// Role of device in connection (0 = Master / 1 = Slave)
    uint8_t role;
    /// Need one and only one user to fill this filed to provide link configurations.
    ble_gap_connect_configure_t config_info;
} ble_gap_connect_ind_t;


/**
 * @brief The structure of #BLE_GAP_DISCONNECTED_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Reason of disconnection(@see enum co_error)
    uint8_t reason;
} ble_gap_disconnected_ind_t;


/**
 * @brief The structure of #BLE_GAP_PERIODIC_ADV_SYNC_CREATED_IND.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
} ble_gap_per_adv_sync_created_ind_t;

/**
 * @brief The structure of #BLE_GAP_PERIODIC_ADV_SYNC_STOPPED_IND.
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Stopped reason
    uint8_t reason;
} ble_gap_per_adv_sync_stopped_ind_t;


/**
 * @brief The struture of #BLE_GAP_START_PERIODIC_ADV_SYNC_CNF.
 */
typedef struct
{
    /// Activity identifier.
    uint8_t actv_index;
    /// Start result.
    uint8_t status;
} ble_gap_start_per_adv_sync_cnf_t;


/**
 * @brief The struture of #BLE_GAP_STOP_PERIODIC_ADV_SYNC_CNF.
 */
typedef struct
{
    /// Activity identifier.
    uint8_t actv_index;
    /// Start result.
    uint8_t status;
} ble_gap_stop_per_adv_sync_cnf_t;

/**
 * @brief The struture of #BLE_GAP_DELETE_PERIODIC_ADV_SYNC_CNF.
 */
typedef struct
{
    /// Activity identifier.
    uint8_t actv_index;
    /// Delete result.
    uint8_t status;
} ble_gap_delete_per_adv_sync_cnf_t;

/**
 * @brief The struture of #BLE_GAP_PERIODIC_ADV_SYNC_ESTABLISHED_IND.
 */
typedef struct
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
} ble_gap_per_adv_sync_established_t;


/**
 * @brief The structure of #BLE_GAP_REMOTE_VER_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Manufacturer name
    uint16_t compid;
    /// LMP subversion
    uint16_t lmp_subvers;
    /// LMP version
    uint8_t  lmp_vers;

} ble_gap_remote_version_ind_t;

/**
 * @brief The structure of #BLE_GAP_REMOTE_FEATURE_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// 8-byte array for LE features
    uint8_t features[GAP_LE_FEATS_LEN];
} ble_gap_remote_features_ind_t;


/**
 * @brief The structure of #BLE_GAP_REMOTE_RSSI_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// RSSI value
    int8_t rssi;
} ble_gap_remote_rssi_ind_t;


/**
 * @brief The structure of #BLE_GAP_REMOTE_PHY_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// LE PHY for data transmission (@see enum gap_phy)
    uint8_t tx_phy;
    /// LE PHY for data reception (@see enum gap_phy)
    uint8_t rx_phy;
} ble_gap_remote_phy_ind_t;



/**
 * @brief The structure of #BLE_GAP_UPDATE_CONN_PARAM_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    ///Connection interval value
    uint16_t            con_interval;
    ///Connection latency value
    uint16_t            con_latency;
    ///Supervision timeout
    uint16_t            sup_to;
} ble_gap_update_conn_param_ind_t;

/// Bond procedure requested information data
/*@TRACE
 @trc_ref gapc_bond*/
typedef union
{
    /// Authentication level (@see gap_auth) (if request = GAPC_PAIRING_REQ)
    //@trc_union parent.request == GAPC_PAIRING_REQ
    uint8_t auth_req;
    /// LTK Key Size (if request = GAPC_LTK_EXCH)
    //@trc_union parent.request == GAPC_LTK_EXCH
    uint8_t key_size;
    /// Device IO used to get TK: (if request = GAPC_TK_EXCH)
    ///  - GAP_TK_OOB:       TK get from out of band method
    ///  - GAP_TK_DISPLAY:   TK generated and shall be displayed by local device
    ///  - GAP_TK_KEY_ENTRY: TK shall be entered by user using device keyboard
    //@trc_union parent.request == GAPC_TK_EXCH
    uint8_t tk_type;

    /// Addition OOB Data for the OOB Conf and Rand values
    //@trc_union parent.request == GAPC_OOB_EXCH
    ble_gap_oob_t  oob_data;
    /// Numeric Comparison Data
    //@trc_union parent.request == GAPC_NC_EXCH
    ble_gap_nc_t   nc_data;
} ble_gap_bond_req_data_t;



/**
 * @brief The struture of #BLE_GAP_BOND_CNF.
 */
typedef struct
{
    uint8_t conn_idx;                 /**< Connection index. */
    uint8_t status;                   /**< Bond command result. */
} ble_gap_bond_cnf_t;


/**
 * @brief The structure of #BLE_GAP_BOND_REQ_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Bond request type (@see gapc_bond)
    uint8_t request;
    /// Bond parameter
    ble_gap_bond_req_data_t data;
} ble_gap_bond_req_ind_t;


/**
 *  Authentication information
 */
/*@TRACE*/
typedef struct
{
    /// Authentication information (@see gap_auth)
    uint8_t info;
    /// LTK exchanged during pairing.
    bool ltk_present;
} ble_gap_bond_auth_t;


/// Bond procedure information data
/*@TRACE
 @trc_ref gapc_bond*/
typedef union
{
    /// Authentication information (@see gap_auth)
    /// (if info = GAPC_PAIRING_SUCCEED)
    //@trc_union parent.info == GAPC_PAIRING_SUCCEED
    ble_gap_bond_auth_t auth;
    /// Pairing failed reason  (if info = GAPC_PAIRING_FAILED)
    //@trc_union parent.info == GAPC_PAIRING_FAILED
    uint8_t reason;
    /// Long Term Key information (if info = GAPC_LTK_EXCH)
    //@trc_union parent.info == GAPC_LTK_EXCH
    ble_gap_ltk_t ltk;
    /// Identity Resolving Key information (if info = GAPC_IRK_EXCH)
    //@trc_union parent.info == GAPC_IRK_EXCH
    ble_gap_irk_t irk;
    /// Connection Signature Resolving Key information (if info = GAPC_CSRK_EXCH)
    //@trc_union parent.info == GAPC_CSRK_EXCH
    ble_gap_sec_key_t csrk;
} ble_gap_bond_data_t;


/**
 * @brief The structure of #BLE_GAP_BOND_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Bond information type (@see gapc_bond)
    uint8_t info;
    ble_gap_bond_data_t data;
} ble_gap_bond_ind_t;

/**
 * @brief The structure of #BLE_GAP_ENCRYPT_REQ_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Encryption Diversifier
    uint16_t ediv;
    /// Random Number
    ble_gap_rand_nb_t rand_nb;
} ble_gap_encrypt_req_ind_t;


/**
 * @brief The structure of #BLE_GAP_ENCRYPT_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Authentication  level (@see gap_auth)
    uint8_t auth;
} ble_gap_encrypt_ind_t;



/**
 * @brief The structure of #BLE_GAP_SECURITY_REQUEST_IND.
 */
typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Authentication  level (@see gap_auth)
    uint8_t auth;
} ble_gap_security_request_ind_t;


/**
 * @brief The struture of #BLE_GAP_SECURITY_REQUEST_CNF.
 */
typedef struct
{
    uint8_t conn_idx;                 /**< Connection index. */
    uint8_t status;                   /**< Security request result. */
} ble_gap_security_request_cnf_t;


/**
 * @brief The struture of #BLE_GAP_UPDATE_DATA_LENGTH_CNF.
 */
typedef struct
{
    uint8_t conn_idx;                 /**< Connection index. */
    uint8_t status;                   /**< Update data length result. */
} ble_gap_update_data_length_cnf_t;

/**
 * @brief The structure of #BLE_GAP_UPDATE_DATA_LENGTH_IND.
 */
typedef struct
{
    uint8_t conn_idx;                /**< Connection index. */
    uint16_t max_tx_octets;          /**< The maximum number of payload octets in TX. */
    uint16_t max_tx_time;            /**< The maximum time that the local Controller will take to TX. */
    uint16_t max_rx_octets;          /**< The maximum number of payload octets in RX. */
    uint16_t max_rx_time;            /**< The maximum time that the local Controller will take to RX. */
} ble_gap_update_data_length_ind_t;


/**
 * @brief The struture of #BLE_GAP_UPDATE_CHANNEL_MAP_CNF.
 */
typedef struct
{
    uint8_t conn_idx;                 /**< Connection index. */
    uint8_t status;                   /**< Update data length result. */
} ble_gap_update_channel_map_cnf_t;


/**
 * @brief The structure of #BLE_GAP_SIGN_COUNTER_UPDATE_IND.
 */
typedef struct
{
    uint32_t local_sign_counter;     /**< Local SignCounter value. */
    uint32_t peer_sign_counter;      /**< Peer SignCounter value. */
} ble_gap_sign_counter_update_ind_t;



/**
 * @ Indicate reception of advertising, scan response or periodic advertising data. The structure of #BLE_GAP_EXT_ADV_REPORT_IND
 */
typedef struct
{
    /// Activity identifier
    uint8_t actv_idx;
    /// Combined with report type (@see enum gapm_adv_report_type) and bit field providing information (@see enum gapm_adv_report_info)
    uint8_t info;
    /// Advertising device address
    ble_gap_addr_t addr;
    /// Directed advertising target address (in case of a directed advertising report)
    ble_gap_addr_t direct_addr;
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
} ble_gap_ext_adv_report_ind_t;

/**
 * @ Indicate rssi and channel assesement of connected link . The structure of #BT_DBG_RSSI_NOTIFY_IND
 */
typedef struct
{
    /// BT link or BLE link.
    uint8_t is_bt;
    /// All channel assesement. 0 is no prefered. Negative is good channel while positive is bad channel.
    int8_t channel_asset[79];
    /// Average RSSI for all Exsted link of BT or BLE
    uint8_t rssi_num;
    int8_t rssi_ave[__ARRAY_EMPTY];
} bt_dbg_rssi_notify_ind_t;


typedef struct
{
    /// Resolvable random address solved
    bd_addr_t addr;
    /// IRK that correctly solved the random address
    ble_gap_sec_key_t irk;
} ble_gap_solved_addr_ind_t;

typedef struct
{
    uint8_t type;
} ble_gap_assert_ind_t;

// Local info


/**
 * @brief Set local device name. The event
   @param[in] dev local device name.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_dev_name(ble_gap_dev_name_t *dev);

/**
 * @brief Set local appearance for GAP service
   @param[in] appearance local appearance.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_appearance(uint16_t appearance);


/**
 * @brief Set local slave prefer param for GAP service
   @param[in] param local slave prefer param.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_slave_prefer_param(ble_gap_slave_prf_param_t *param);

/**
 * @brief Configure local public address
   @param[in] addr public address.
   @retval The status of configure.
 */
uint8_t ble_gap_configure_public_addr(bd_addr_t *addr);

/**
 * @brief Get the local version supported by controller. The event #BLE_GAP_LOCAL_VER_IND will indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_local_version(void);

/**
 * @brief Get the local address. The event #BLE_GAP_LOCAL_BD_ADDR_IND will indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_local_bdaddr(void);

/**
 * @brief Get the local advertising channel tx power. The event #BLE_GAP_LOCAL_ADV_TX_POWER_IND will indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_local_adv_tx_power(void);

/**
 * @brief Get the local maximum data length support by controller. The event #BLE_GAP_LOCAL_MAX_DATA_LEN_IND will indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_local_max_data_len(void);

/**
 * @brief Get the maximum number of adv sets.The event #BLE_GAP_NUMBER_ADV_SETS_IND will indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_number_of_adv_sets(void);

/**
 * @brief Get the maximum adv data len and scan rsp len supported by controller.The event #BLE_GAP_LOCAL_MAX_ADV_DATA_LEN_IND will indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_local_max_adv_data_len(void);

/**
 * @brief Set the white list.The event #BLE_GAP_SET_WHITE_LIST_CNF will indicate the result.
   @param[in] white_list Devices in white list.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_white_list(ble_gap_white_list_t *white_list);

/**
 * @brief Set the resolving list.The event #BLE_GAP_SET_RESOLVING_LIST_CNF will indicate the result.
   @param[in] rl_list Devices in resolving list.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_resolving_list(ble_gap_resolving_list_t *rl_list);

/**
 * @brief Get local ral address via peer ral address. The event #BLE_GAP_RAL_ADDR_IND will indicate the result.
   @param[in] addr Peer ral address.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_local_ral_addr(ble_gap_addr_t *addr);

/**
 * @brief Generate a random address by rand type. The event #BLE_GAP_LOCAL_BD_ADDR_IND will indicate the result.
   @param[in] rand_type address random type.(@see enum gapm_own_addr)
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_generate_rand_addr(uint8_t rand_type);


/**
 * @brief Set new irk. The event #BLE_GAP_SET_IRK_CNF will indicate the result.
   @param[in] new_irk new irk to set.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_irk(ble_gap_sec_key_t *new_irk);

/**
 * @brief Create a new advertising set. The event #BLE_GAP_ADV_CREATED_IND will receive if created successful. #BLE_GAP_CREATE_ADV_CNF will indicate the
          created compelted
   @param[in] para advertising parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_create_advertising(ble_gap_adv_parameter_t *para);

/**
 * @brief Set advertising data, it should be set before advertising start. The event #BLE_GAP_SET_ADV_DATA_CNF will indicate the result.
   @param[in] data advertising data.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_adv_data(ble_gap_adv_data_t *data);

/**
 * @brief Set periodic advertising data. The event #BLE_GAP_SET_ADV_DATA_CNF will indicate the result.
   @param[in] data advertising data.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_periodic_adv_data(ble_gap_adv_data_t *data);

/**
 * @brief Set scan rsp data, it should be set before advertising start. The event #BLE_GAP_SET_SCAN_RSP_DATA_CNF will indicate the result.
   @param[in] data scan response data.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_set_scan_rsp_data(ble_gap_adv_data_t *data);

/**
 * @brief Start advertising. The event #BLE_GAP_START_ADV_CNF will indicate the result.
   @param[in] adv_start start advertising parameter.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_start_advertising(ble_gap_adv_start_t *adv_start);

/**
 * @brief Stop advertising. The event #BLE_GAP_STOP_ADV_CNF will indicate the result.
   @param[in] adv_stop stop advertising parameter.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_stop_advertising(ble_gap_adv_stop_t *adv_stop);

/**
 * @brief Delete advertising set. The event #BLE_GAP_DELETE_ADV_CNF will indicate the result.
   @param[in] del delete advertising parameter.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_delete_advertising(ble_gap_adv_delete_t *del);

/**
 * @brief Start scan. The event #BLE_GAP_SCAN_START_CNF will indicate the result.
   @param[in] scan_param Scan start parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_scan_start(ble_gap_scan_start_t *scan_param);

/**
 * @brief Stop scan.The event #BLE_GAP_SCAN_STOP_CNF will indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_scan_stop(void);


/**
 * @brief Start connect peer device. The event #BLE_GAP_CREATE_CONNECTION_CNF will indicate connect operation result. #BLE_GAP_CONNECTED_IND will indicate
          peer device connected.
   @param[in] conn_param connection parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_create_connection(ble_gap_connection_create_param_t *conn_param);


/**
 * @brief Provide link information for the connected request.
   @param[in] rsp connection information.
   @retval The status of send to BLE subsystem.
 */

uint8_t ble_gap_connect_response(ble_gap_connection_response_t *rsp);


/**
 * @brief Start connect peer device. The event #BLE_GAP_CANCEL_CREATE_CONNECTION_CNF indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_cancel_create_connection(void);


/**
 * @brief Disconnect peer device. The event #BLE_GAP_DISCONNECT_CNF will indicate disconnect operation result.
          #BLE_GAP_DISCONNECTED_IND will indicate peer device disconnected.
   @param[in] conn disconnect parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_disconnect(ble_gap_disconnect_t *conn);


/**
 * @brief Create a new periodic advertising sync set to get activity idx. The event #BLE_GAP_CREATE_PERIODIC_ADV_SYNC_CNF indicate the result.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_create_periodic_advertising_sync(void);

/**
 * @brief Start periodic advertising sync to dedicated periodic advertising. The event #BLE_GAP_START_PERIODIC_ADV_SYNC_CNF indicate the result.
   @param[in] Sync start parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_start_periodic_advertising_sync(ble_gap_periodic_advertising_sync_start_t *sync_param);

/**
 * @brief Stop periodic advertising sync. The event #BLE_GAP_STOP_PERIODIC_ADV_SYNC_CNF indicate the result.
   @param[in] Sync stop parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_stop_periodic_advertising_sync(ble_gap_eriodic_advertising_sync_stop_t *sync_stop);

/**
 * @brief Delete periodic advertising sync set. The event #BLE_GAP_DELETE_PERIODIC_ADV_SYNC_CNF indicate the result.
   @param[in] Sync delete parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_delete_periodic_advertising_sync(ble_gap_eriodic_advertising_sync_delete_t *del);



/**
 * @brief Get the peer device Bluetooth version. The event #BLE_GAP_REMOTE_VER_IND will indicate the result.
   @param[in] ver version parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_remote_version(ble_gap_get_remote_version_t *ver);

/**
 * @brief Get the peer device supported features. The event #BLE_GAP_REMOTE_FEATURE_IND will indicate the result.
   @param[in] feature version parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_remote_feature(bt_gap_get_remote_feature_t *feature);

/**
 * @brief Get the peer device RSSI. The event #BLE_GAP_REMOTE_RSSI_IND will indicate the result.
   @param[in] rssi parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_remote_rssi(ble_gap_get_rssi_t *rssi);

/**
 * @brief Get the peer device physical configuration. The event #BLE_GAP_REMOTE_PHY_IND will indicate the result.
   @param[in] phy parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_get_remote_physical(ble_gap_get_phy_t *phy);

/**
 * @brief Update the connection parameters with peer device. This API will first try to use HCI command to update connection parameter and if only
          remote device not supported then use l2cap to update.
          The event #BLE_GAP_UPDATE_CONN_PARAM_IND will indicate the result.
   @param[in] conn_para Connection update parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_update_conn_param(ble_gap_update_conn_param_t *conn_para);

/**
 * @brief Update the connection parameters with peer device .This API will try to used l2cap to update connection parameter. Only if current role is
          master, then use HCI command to update.
          The event #BLE_GAP_UPDATE_CONN_PARAM_IND will indicate the result.
   @param[in] conn_para Connection update parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_update_conn_param_on_l2cap(ble_gap_update_conn_param_t *conn_para);

uint8_t ble_gap_lepsm_register(void);

/**
 * @brief Resolve RPA address with IRKs. The event #BLE_GAP_RESOLVE_ADDRESS_CNF will indicate the result.
          #BLE_GAP_SOLVED_ADDRESS_IND will indicate address is solved with the correct IRK.
   @param[in] req resolve address and possible IRKs.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_resolve_address(ble_gap_resolve_address_t *req);

/**
 * @brief Bond with peer device. The API could  be only called via master device. Slave device can use  #ble_gap_security_request().
          The event #BLE_GAP_BOND_CNF indicate the bond command result. The event #BLE_GAP_BOND_IND will indicate the bond process result.
   @param[in] bond bond information.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_bond(ble_gap_bond_t *bond);

/**
 * @brief Reponse with bond request from peer device. Should call this API after received #BLE_GAP_BOND_REQ_IND. The event #BLE_GAP_BOND_IND will indicate the
          bond result.
   @param[in] cfm confirm parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_bond_confirm(ble_gap_bond_confirm_t *cfm);

/**
 * @brief Reponse with encrypt request from peer device. Should call this API after received #BLE_GAP_ENCRYPT_REQ_IND. The event #BLE_GAP_ENCRYPT_IND will indicate the result.
   @param[in] cfm encrypt confirm parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_encrypt_confirm(ble_gap_encrypt_confirm_t *cfm);


/**
 * @brief  Request master initiates a security with associated parameters which only could be called as slave role.
           Master device can use #ble_gap_bond(). The event #BLE_GAP_SECURITY_REQUEST_CNF will indicate the result of request.
   @param[in] req Security properties.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_security_request(ble_gap_sec_req_t *req);

//uint8_t ble_gap_set_passkey(ble_gap_set_passkey_t *key);

/**
 * @brief Update phy configuration with peer device. The event #BLE_GAP_PHY_UPDATE_IND will indicate the result.
   @param[in] phy Phy updated parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_update_phy(ble_gap_update_phy_t *phy);

/**
 * @brief Update data length with peer device. The event #BLE_GAP_UPDATE_DATA_LENGTH_IND will indicate the result.
   @param[in] req Updated parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_update_data_len(ble_gap_update_data_len_t *req);

/**
 * @brief Update channel map to avoid air collision. The event  #BLE_GAP_UPDATE_CHANNEL_MAP_CNF and
          #BLE_GAP_UPDATE_CHANNEL_MAP_IND will indicate the result.
   @param[in] map Channel updated parameters.
   @retval The status of send to BLE subsystem.
 */
uint8_t ble_gap_update_channel_map(ble_gap_update_channel_map_t *map);



/**
 * @brief Get local irk.
   @param[out] local_irk Local irk.
   @retval 0 if local irk could get.
 */
uint8_t ble_get_local_irk(ble_gap_sec_key_t *local_irk);


/**
 * @brief User implementation function. Stack will call this function to get local irk.
   @param[out] local_irk Local irk.
   @retval  #BLE_UPDATE_NO_UPDATE No need to update.
            #BLE_UPDATE_ONCE Only update if public address not existed.
            #BLE_UPDATE_ALWAYS Always use this public address.
 */
ble_common_update_type_t ble_request_local_irk(ble_gap_sec_key_t *local_irk);

//extern void connection_manager_h6_result_cb(uint8_t *aes_res, uint32_t metainfo);

uint8_t ble_gap_aes_h6(uint8_t *w, uint8_t *key_id, uint32_t cb_request);

uint8_t ble_gap_aes_h7(uint8_t *salt, uint8_t *w, uint32_t metainfo);



// Central related APIs
//   a. Initial scan/conneciton
//   b. Initial encrypt
//   c. Set channl mapping/selection.

// Periodic related APIs

// LE credit based connection


/*
*/


/**
* @}
*/

#endif // __BF0_BLE_GAP_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
