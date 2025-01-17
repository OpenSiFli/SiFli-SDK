/**
  ******************************************************************************
  * @file   bts2_app_interface_type.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
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

#ifndef _BTS2_APP_INTERFACE_TYPE_H_
#define _BTS2_APP_INTERFACE_TYPE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BT_NOTIFY_PBAP_MAX_VCARD_ENTRY_HANDLE_SIZE    (42)
#define BT_NOTIFY_PBAP_MAX_VCARD_CONTACT_NAME_SIZE    (80)
#define BT_NOTIFY_MAX_FRIENDLY_NAME_LEN               (60)

/**
 * @defgroup BT_interface_type BT interface type define
 * @ingroup  BT_interface
 * @{
 *
 */

///  these type are bt callback type_id
typedef enum
{
    ///  BT common event notify type
    BT_NOTIFY_COMMON,
    ///  HFP HF profle event notify type
    BT_NOTIFY_HFP_HF,
    ///  HFP AG profle event notify type
    BT_NOTIFY_HFP_AG,
    ///  A2DP profle event notify type
    BT_NOTIFY_A2DP,
    ///  AVRCP profle event notify type
    BT_NOTIFY_AVRCP,
    ///  HID profle event notify type
    BT_NOTIFY_HID,
    ///  PAN profle event notify type
    BT_NOTIFY_PAN,
    ///  SPP profle event notify type
    BT_NOTIFY_SPP,
    ///  BT GATT profle event notify type
    BT_NOTIFY_GATT,
    ///  PBAP profle event notify type
    BT_NOTIFY_PBAP
} bt_notify_type_id_t;

///  these type are BT_NOTIFY_COMMON event id
typedef enum
{
    ///  BT stack ready event
    BT_NOTIFY_COMMON_BT_STACK_READY,
    ///  close bt function complete event
    BT_NOTIFY_COMMON_CLOSE_COMPLETE,
    ///  inquiry response event
    BT_NOTIFY_COMMON_DISCOVER_IND,
    ///  inquiry complete event
    BT_NOTIFY_COMMON_INQUIRY_CMP,
    ///  read local name complete event
    BT_NOTIFY_COMMON_LOCAL_NAME_RSP,
    ///  read remote name complete event
    BT_NOTIFY_COMMON_REMOTE_NAME_RSP,
    ///  read local RSSI complete event
    BT_NOTIFY_COMMON_LOCAL_RSSI_RSP,
    ///  read local address complete event
    BT_NOTIFY_COMMON_LOCAL_ADDR_RSP,
    ///  change local address complete event
    BT_NOTIFY_COMMON_CHANGE_ADDR_RSP,
    ///  read remote version complete event
    BT_NOTIFY_COMMON_RMT_VERSION_IND,
    ///  encryption complete event
    BT_NOTIFY_COMMON_ENCRYPTION,
    ///  notift link key missing event
    BT_NOTIFY_COMMON_KEY_MISSING,
    ///  notift link key overlaid event
    BT_NOTIFY_COMMON_KEY_OVERLAID,
    ///  pair complete event
    BT_NOTIFY_COMMON_PAIR_IND,
    ///  receive pair request event
    BT_NOTIFY_COMMON_PAIR_STATE,
    ///  receive acl connect request from remote device
    BT_NOTIFY_COMMON_ACL_CONNECT_IND,
    ///  acl connection complete event
    BT_NOTIFY_COMMON_ACL_CONNECTED,
    ///  acl disconnection complete event
    BT_NOTIFY_COMMON_ACL_DISCONNECTED,
    ///  sco connection complete event
    BT_NOTIFY_COMMON_SCO_CONNECTED,
    ///  sco disconnection complete event
    BT_NOTIFY_COMMON_SCO_DISCONNECTED,
    ///  user confirm event
    BT_NOTIFY_COMMON_USER_CONFIRM_IND,
    ///  receive io capability request event
    BT_NOTIFY_COMMON_IO_CAPABILITY_IND,
} bt_notify_common_event_id_t;

///  these type are BT_NOTIFY_HFP_HF event id
typedef enum
{
    ///  HFP HF profile connection complete event
    BT_NOTIFY_HF_PROFILE_CONNECTED,
    ///  HFP HF profile disconnection complete event
    BT_NOTIFY_HF_PROFILE_DISCONNECTED,
    ///  HFP HF profile voice recognition capability status update event
    BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE,
    ///  HFP HF profile voice recognition status change event
    BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE,
    ///  HFP HF profile local phone number event
    BT_NOTIFY_HF_LOCAL_PHONE_NUMBER,
    ///  HFP HF profile remote phone information event
    BT_NOTIFY_HF_REMOTE_CALL_INFO_IND,
    ///  HFP HF profile voice speaker volume change event
    BT_NOTIFY_HF_VOLUME_CHANGE,
    ///  HFP HF profile call status change event
    BT_NOTIFY_HF_CALL_STATUS_UPDATE,
    ///  HFP HF profile indicator status change event
    BT_NOTIFY_HF_INDICATOR_UPDATE,
    ///  HFP HF profile at cmd send result event
    BT_NOTIFY_HF_AT_CMD_CFM,
} bt_notify_hfp_hf_event_id_t;

///  these type are BT_NOTIFY_HFP_AG event id
typedef enum
{
    ///  HFP AG profile connection complete event
    BT_NOTIFY_AG_PROFILE_CONNECTED,
    ///  HFP AG profile disconnection complete event
    BT_NOTIFY_AG_PROFILE_DISCONNECTED,
    ///  HFP AG profile receive make call request event
    BT_NOTIFY_AG_MAKE_CALL_REQ,
    ///  HFP AG profile receive answer call request event
    BT_NOTIFY_AG_ANSWER_CALL_REQ,
    ///  HFP AG profile receive hangup call request event
    BT_NOTIFY_AG_HANGUP_CALL_REQ,
    ///  HFP AG profile receive dtmf key request event
    BT_NOTIFY_AG_RECV_DTMF_KEY,
    ///  HFP AG profile receive speaker volume change event
    BT_NOTIFY_AG_VOLUME_CHANGE,
    ///  HFP AG profile receive get indicator status request event
    BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ,
    ///  HFP AG profile receive get all remote call information request event
    BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ,
    ///  HFP AG profile receive get local phone number request event
    BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ,
    ///  HFP AG profile receive extern at cmd event
    BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ,
} bt_notify_hfp_ag_event_id_t;

///  these type are BT_NOTIFY_A2DP event id
typedef enum
{
    ///  A2DP sink function close complete event
    BT_NOTIFY_AVSNK_CLOSE_COMPLETE,
    ///  A2DP sink function open complete event
    BT_NOTIFY_AVSNK_OPEN_COMPLETE,
    ///  A2DP profile connection complete event
    BT_NOTIFY_A2DP_PROFILE_CONNECTED,
    ///  A2DP profile disconnection complete event
    BT_NOTIFY_A2DP_PROFILE_DISCONNECTED,
    ///  receive a2dp start command from remote device indication event
    BT_NOTIFY_A2DP_START_IND,
} bt_notify_a2dp_event_id_t;

///  these type are BT_NOTIFY_AVRCP event id
typedef enum
{
    ///  AVRCP function close complete event
    BT_NOTIFY_AVRCP_CLOSE_COMPLETE,
    ///  AVRCP function open complete event
    BT_NOTIFY_AVRCP_OPEN_COMPLETE,
    ///  AVRCP profile connection complete event
    BT_NOTIFY_AVRCP_PROFILE_CONNECTED,
    ///  AVRCP profile disconnection complete event
    BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED,
    ///  get music information event
    BT_NOTIFY_AVRCP_MP3_DETAIL_INFO,
    ///  peer device register the absolute volume change notify event
    BT_NOTIFY_AVRCP_VOLUME_CHANGED_REGISTER,
    ///  peer device adjusts the absolute volume event
    BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME,
    ///  play status changed event
    BT_NOTIFY_AVRCP_PLAY_STATUS,
    ///  play progress change event
    BT_NOTIFY_AVRCP_SONG_PROGREAS_STATUS,
    ///  play song change event
    BT_NOTIFY_AVRCP_TRACK_CHANGE_STATUS,
} bt_notify_avrcp_event_id_t;

///  these type are BT_NOTIFY_HID event id
typedef enum
{
    ///  HID function close complete event
    BT_NOTIFY_HID_CLOSE_COMPLETE,
    ///  HID function open complete event
    BT_NOTIFY_HID_OPEN_COMPLETE,
    ///  HID profile connection complete event
    BT_NOTIFY_HID_PROFILE_CONNECTED,
    ///  HID profile disconnection complete event
    BT_NOTIFY_HID_PROFILE_DISCONNECTED,
} bt_notify_hid_event_id_t;

///  these type are BT_NOTIFY_SPP event id
typedef enum
{
    ///  SPP profile connection complete event
    BT_NOTIFY_SPP_PROFILE_CONNECTED,
    ///  SPP profile disconnection complete event
    BT_NOTIFY_SPP_PROFILE_DISCONNECTED,
    ///  receive spp connect request event
    BT_NOTIFY_SPP_CONN_IND,
    ///  receive spp data event
    BT_NOTIFY_SPP_DATA_IND,
    ///  send spp response event
    BT_NOTIFY_SPP_DATA_CFM,
    ///  certain spp disconnection event
    BT_NOTIFY_SPP_DISC_IND,
} bt_notify_spp_event_id_t;

///  these type are BT_NOTIFY_PAN event id
typedef enum
{
    ///  PAN profile connection complete event
    BT_NOTIFY_PAN_PROFILE_CONNECTED,
    ///  PAN profile disconnection complete event
    BT_NOTIFY_PAN_PROFILE_DISCONNECTED,
} bt_notify_pan_event_id_t;

/// these type are BT_NOTIFY_GATT event id
typedef enum
{
    ///  GATT profile connection complete event
    BT_NOTIFY_GATT_PROFILE_CONNECTED,
    ///  GATT profile disconnection complete event
    BT_NOTIFY_GATT_PROFILE_DISCONNECTED,
    ///  GATT profile register status update event
    BT_NOTIFY_GATT_REGISTER_RESPONSE,
    ///  GATT profile unregister status update event
    BT_NOTIFY_GATT_UNREGISTER_RESPONSE,
    ///  GATT profile change mtu value result event
    BT_NOTIFY_GATT_CHANGE_MTU_RESPONSE,
} bt_notify_gatt_event_id_t;

///  these type are BT_NOTIFY_PBAP event id
typedef enum
{
    ///  PBAP profile connection complete event
    BT_NOTIFY_PBAP_PROFILE_CONNECTED,
    ///  PBAP profile disconnection complete event
    BT_NOTIFY_PBAP_PROFILE_DISCONNECTED,
    ///  PBAP profile vCard list event
    BT_NOTIFY_PBAP_VCARD_LIST_ITEM_IND,
    ///  PBAP profile vCard list complete event
    BT_NOTIFY_PBAP_VCARD_LIST_CMPL,
} bt_notify_pbap_event_id_t;



/// Inquiry parameters
typedef struct
{
    /// Device class mask, A device with a specific Class of Device responded to the Inquiry process.
    /// 0 is used for unlimited class of device
    uint32_t dev_cls_mask;
    /// Maxium amount of time specified before the inquiry procedure halted
    /// with SECOND. 0 is used for unlimited time
    uint16_t max_timeout;
    /// Maxium number of responses from the inquiry before the inquiry is halted.
    /// 0 is used for unlimited number of responses
    uint8_t max_rsp;
} bt_start_inquiry_ex_t;


///  bt device mac information
typedef struct
{
    ///  bt device mac
    uint8_t addr[6];
} bt_notify_device_mac_t;

///  bt device pair confirm information
typedef struct
{
    ///  bt device mac
    bt_notify_device_mac_t mac;
    ///  passkey val
    uint32_t num_val;
} bt_notify_pair_confirm_t;

///  bt remote device version information
typedef struct
{
    ///  read remote device version status
    uint8_t    res;
    ///  remote LMP version status
    uint8_t    lmp_version;
    ///  remote device manufacturer mame
    uint16_t   manufacturer_name;
    ///  remote device lmp subversion
    uint16_t   lmp_subversion;
} bt_notify_rmt_version_t;

///  bt inquiry remote device information
typedef struct
{
    ///  remote bt device mac
    bt_notify_device_mac_t mac;
    ///  remote bt class of device
    uint32_t dev_cls;
    ///  remote device name size
    uint8_t name_size;
    ///  remote device name,utf8
    char bt_name[BT_NOTIFY_MAX_FRIENDLY_NAME_LEN + 1];
} bt_notify_remote_device_info_t;

///  bt read remote device name information
typedef struct
{
    ///  remote bt device mac
    bt_notify_device_mac_t mac;
    ///  read remote bt device name result
    uint8_t res;
    /// remote device name
    char bt_name[BT_NOTIFY_MAX_FRIENDLY_NAME_LEN + 1];
} bt_notify_rmt_name_t;

///  bt remote device acl connection state base information
typedef struct
{
    ///  remote device mac
    bt_notify_device_mac_t mac;
    ///  reason
    uint8_t res;
} bt_notify_device_base_info_t;

///  bt profile connection state information
typedef struct
{
    ///  remote device mac
    bt_notify_device_mac_t mac;
    ///  profile type
    uint8_t profile_type;
    ///  reason
    uint8_t res;
} bt_notify_profile_state_info_t;

///  bt acl connection state information
typedef struct
{
    ///  remote device mac
    bt_notify_device_mac_t mac;
    ///  reason
    uint8_t res;
    ///  acl connect direction
    uint8_t acl_dir;
    ///  remote device class of device
    uint32_t dev_cls;
} bt_notify_device_acl_conn_info_t;

///  bt sco connection state information
typedef struct
{
    ///  bt sco connection type:AG or HF or L2CAP
    uint8_t sco_type;
    ///  reason
    uint8_t sco_res;
} bt_notify_device_sco_info_t;

///  HFP HF at cmd send result
typedef struct
{
    ///  at cmd identify
    uint8_t  at_cmd_id;
    ///  result
    uint8_t  res;
} bt_notify_at_cmd_cfm_t;

///  HFP HF indicator status
typedef struct
{
    ///  indicator type
    uint8_t type;
    ///  indicator value
    uint8_t val;
} bt_notify_cind_ind_t;

///  HFP HF call information
typedef struct
{
    /// call status
    uint8_t  call_status;
    /// callsetup status
    uint8_t  callsetup_status;
    /// call held status
    uint8_t  callheld_status;
} bt_notify_all_call_status;

///  HFP HF remote call detail information
typedef struct
{
    /// phone number type
    uint16_t phone_number_type;
    /// Current call index, counting from 1
    uint8_t  idx;
    ///  Telephone direction 0:outgoing 1:incoming
    uint8_t  dir;      ///  Telephone direction 0:outgoing 1:incoming
    uint8_t  st;       /* clcc status
                        0: Active
                        1: held
                        2: Dialing(outgoing calls only)
                        3: Alerting(outgoing calls only)
                        4: Incoming (incoming calls only)
                        5: Waiting (incoming calls only)
                        6: Call held by Response and Hold */
    ///  Telephone mode£¬0 (Voice), 1 (Data), 2 (FAX)
    uint8_t  mode;
    ///  Multiparty call identification 0: not multiparty call 1:multiparty call
    uint8_t  mpty;
    ///  phone number size
    uint8_t  number_size;
    ///  phone number
    uint8_t  number[23];
} bt_notify_clcc_ind_t;

///  GATT register or unregister result
typedef struct
{
    ///  sdp record handler
    uint32_t sdp_rec_hdl;
    /// result
    uint8_t res;
} bt_notify_gatt_sdp_info_t;

///  PABP vcard name result
typedef struct
{
    ///  vcard handle length
    uint8_t vcard_handle_len;
    ///  vcard name length
    uint8_t name_len;
    ///  vcard handle
    char vcard_handle[BT_NOTIFY_PBAP_MAX_VCARD_ENTRY_HANDLE_SIZE + 1];
    /// vcard name
    char vcard_name[BT_NOTIFY_PBAP_MAX_VCARD_CONTACT_NAME_SIZE + 1];
} bt_notify_pbap_vcard_listing_item_t;


typedef union
{
    uint8_t uuid_16[2];
    uint8_t uuid_32[4];
    uint8_t uuid_128[16];
} SPP_UUID;

///  received spp data information
typedef struct
{
    ///  the address of the device that sent the data
    bt_notify_device_mac_t mac;
    ///  the service channel of spp connection that sent the data
    uint8_t  srv_chl;
    ///  the uuid of spp connection that sent the data
    SPP_UUID uuid;
    ///  the uuid length of spp connection that sent the data
    uint8_t uuid_len;
    ///  payload length
    uint16_t  payload_len;
    ///  payload pointer
    uint8_t  payload[1];
} bt_notify_spp_data_ind_t;

///  spp connection information
typedef struct
{
    ///  the address of the connected device
    bt_notify_device_mac_t mac;
    ///  the service channel of the connected device
    uint8_t  srv_chl;
    ///  the uuid of spp connection that sent the data
    SPP_UUID uuid;
    ///  the uuid length of spp connection that sent the data
    uint8_t uuid_len;
    ///  the mtu size of the connected device
    uint16_t  mfs;
} bt_notify_spp_conn_ind_t;

///  spp data response confirm information
typedef struct
{
    ///  the address of the connected device
    bt_notify_device_mac_t mac;
    ///  the service channel of the connected device
    uint8_t  srv_chl;
    ///  the uuid of the connected device
    SPP_UUID uuid;
    ///  the uuid length of spp connection that sent the data
    uint8_t uuid_len;
} bt_notify_spp_data_cfm_t;

///  spp disconnection information
typedef struct
{
    ///  the address of the disconnected device
    bt_notify_device_mac_t mac;
    ///  the service channel of the disconnected device
    uint8_t  srv_chl;
    ///  the uuid of the disconnected device
    SPP_UUID uuid;
    ///  the uuid length of spp connection that sent the data
    uint8_t uuid_len;
} bt_notify_spp_disc_ind_t;

/// @}  BT_interface_type
#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

