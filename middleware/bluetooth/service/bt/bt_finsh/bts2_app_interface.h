/**
  ******************************************************************************
  * @file   bts2_app_interface.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
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

#ifndef _BTS2_APP_INTERFACE_H_
#define _BTS2_APP_INTERFACE_H_

/**
 ****************************************************************************************
 * @addtogroup BT_interface BT interface
 * @ingroup middleware
 * @brief Provided standard blutooth interface.
 * @{
 */

#include "drivers/bt_device.h"
#include "bts2_app_interface_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BT_COMMON  BT device interfaces
  * @{
  */
/**
 * @brief           Performs chip power on and kickstarts OS scheduler
 *
 **/
void bt_interface_open_bt(void);

/**
 * @brief           Inititates shutdown of Bluetooth system.
 *                  Any active links will be dropped and device entering
 *                  non connectable/discoverable mode
 **/
void bt_interface_close_bt(void);

/**
 * @brief           Start device discovery/inquiry for BT_DEVCLS_AUDIO device
 *
 **/
void bt_interface_start_inquiry(void);

/**
 * @brief           Start device discovery/inquiry with more parameters
 * @param[in] param Parameters for inquiry condition
 *
 * @return int8_t
 **/
int8_t bt_interface_start_inquiry_ex(bt_start_inquiry_ex_t *param);

/**
 * @brief           Stop device discovery/inquiry
 *
 **/
void bt_interface_stop_inquiry(void);

/**
 * @brief            Initiate connect with the specified device and profile(hf sink)
 * @param[in] mac    Remote device address
 * @param[in] ext_profile   Profile value
 *
 * @return           bt_err_t
 **/
bt_err_t bt_interface_conn_ext(unsigned char *mac, bt_profile_t ext_profile);

/**
 * @brief            Initiate connect with the specified device and profile(ag source)
 * @param[in] mac    Remote device address
 * @param[in] ext_profile   Profile value
 *
 * @return           bt_err_t
 **/
bt_err_t bt_interface_conn_to_source_ext(unsigned char *mac, bt_profile_t ext_profile);

/**
 * @brief                   Disconnect with the specified profile
 * @param[in] mac    Remote device address
 * @param[in] ext_profile : Profile value
 *
 * @return           bt_err_t
 *
 *                   BT_ERROR_INPARAM            = 0x10000001,  input param error
 *                   BT_ERROR_UNSUPPORTED        = 0x10000002,  unsupported function
 *                   BT_ERROR_TIMEOUT            = 0x10000003,  error timout
 *                   BT_ERROR_DISCONNECTED       = 0x10000004,  the bt device is disconnected
 *                   BT_ERROR_STATE              = 0x10000005,  current state  unsupported this function
 *                   BT_ERROR_PARSING            = 0x10000006,  parsing at response error
 *                   BT_ERROR_POWER_OFF          = 0x10000007,  current bt device has been power off
 *                   BT_ERROR_NOTIFY_CB_FULL     = 0x10000008,  register notify cb is more than BT_MAX_EVENT_NOTIFY_CB_NUM
 *                   BT_ERROR_DEVICE_EXCEPTION   = 0x10000009,  current bt device has happend exception
 *                   BT_ERROR_RESP_FAIL          = 0x10000010,  at cmd response fail
 *                   BT_ERROR_AVRCP_NO_REG       = 0x10000011,  set absolute volume, but remote device dont register the event
 *                   BT_ERROR_IN_PROGRESS        = 0x10000012,  for non-blocking processing, wait until the corresponding event is reported
 *                   BT_ERROR_OUT_OF_MEMORY      = 0x10000013,  System heap is not enough
 **/
bt_err_t bt_interface_disc_ext(unsigned char *mac, bt_profile_t ext_profile);

/**
 * @brief                   Cancel with the specified device
 * @param[in] mac           Remote device address
 *
 * @return                  int8_t
 **/
int8_t bt_interface_cancel_connect_req(unsigned char *mac);

/**
 * @brief                   Disconnect with the specified device
 * @param[in] mac           Remote device address
 *
 * @return                  int8_t
 **/
int8_t bt_interface_disconnect_req(unsigned char *mac);

/**
 * @brief            Change local name
 * @param[in] len    Name length
 * @param[in] data   Name
 *
 **/
void bt_interface_set_local_name(int len, void *data);

/**
 * @brief            Read local name request
 *
 **/
void bt_interface_rd_local_name(void);

/**
 * @brief            Read local address request
 *
 **/
void bt_interface_rd_local_bd_addr(void);

/**
 * @brief            exit sniff mode
 *
 **/
void bt_interface_exit_sniff_mode(void);

/**
 * @brief            Read rssi with the specified device
 * @param[in] mac   Remote device address
 *
 **/
void bt_interface_rd_local_rssi(unsigned char *mac);

/**
 * @brief            Read bluetooth's current scan mode
 *
 * @return           Bluetooth scan mode
 **/
U8 bt_interface_get_current_scan_mode(void);

/**
 * @brief               Bluetooth address covert
 * @param[in] src_addr  BTS2S_BD_ADDR to uint8_t
 * @param[out] addr     Device mac address (eg:uint8_t mac[6] = {11,22,33,44,55,66})
 * @return              uint8_t
 **/
uint8_t bt_addr_convert(BTS2S_BD_ADDR *src_addr, uint8_t *addr);

/**
 * @brief            Read connected address with the type of BTS2S_BD_ADDR
 * @param[in] mac    Device mac address(eg:char mac[6] = {11,22,33,44,55,66})
 * @return           BTS2S_BD_ADDR
 **/
BTS2S_BD_ADDR *bt_interface_this_connect_addr(unsigned char *mac);

/**
 * @brief            Set the local role when acl connection is received
 * @param[in] role   Role to be set when acl connection is received
 **/
void bt_interface_acl_accept_role_set(U8 role);

/**
 * @brief            Set the local link policy
 * @param[in] lp_in  Link policy to be set
 * @param[in] lp_out Link policy to be set
 **/
void bt_interface_set_linkpolicy(U16 lp_in, U16 lp_out);

/**
 * @brief            Set whether sniff mode is turned on
 * @param[in] enable Whether the flag of sniff mode is enabled or not
 **/
void bt_interface_set_sniff_enable(BOOL enable);
/// @}  BT_COMMON

/** @defgroup BT_A2DP_SRV  A2DP profile interfaces
  * @{
  */

/**
 * @brief            Open the function of a2dp sink
 *
 **/
void bt_interface_open_avsink(void);

/**
 * @brief            Close the function of a2dp sink
 * @return           Return whether closing succeeded
 *
 **/
bt_err_t bt_interface_close_avsink(void);

/**
 * @brief            Set a2dp bqb test model flag
 * @param[in] value  A2dp bqb test-case number
 **/
void bt_interface_set_a2dp_bqb_test(U8 value);

/**
 * @brief            Set audio device type
 * @param[in] device_type  Audio device type
 **/
void bt_interface_set_audio_device(U8 device_type);

/**
 * @brief            Get current audio device type
 * @return           Return current audio device type
 *
 **/
U8 bt_interface_get_current_audio_device(void);

/**
 * @brief            Check whether the a2dp start command sent by the mobile phone has been received
 * @return           Return results
 *
 **/
U8 bt_interface_get_receive_a2dp_start(void);


/**
 * @brief            release a2dp media channel
 *
 **/
void bt_interface_release_a2dp(void);

/**
 * @brief            unregister a2dp sink sdp record
 *
 **/
void bt_interface_unregister_av_snk_sdp(void);

/**
 * @brief            register a2dp sink sdp record
 *
 **/
void bt_interface_register_av_snk_sdp(void);

#ifdef CFG_AV_SRC
/**
 * @brief            Get current a2dp stream state
 * @return           Return the state of a2dp stream
 *
 **/
U8 bt_interface_get_a2dp_stream_state(void);
#else
#define bt_interface_get_a2dp_stream_state() AVRCP_PLAY_STATUS_STOP
#endif
/// @}  BT_A2DP_SRV

/** @defgroup BT_AVRCP_SRV  AVRCP profile interfaces
  * @{
  */

/**
 * @brief            Open the function of avrcp profile
 *
 **/
void bt_interface_open_avrcp(void);

/**
 * @brief            Close the function of avrcp profile
 *
 **/
void bt_interface_close_avrcp(void);

/**
 * @brief            Control the phone to switch to the next music
 *
 **/
void bt_interface_avrcp_next(void);

/**
 * @brief            Control the mobile phone to play music
 *
 **/
void bt_interface_avrcp_play(void);

/**
 * @brief            Control the mobile phone to suspend music
 *
 **/
void bt_interface_avrcp_pause(void);

/**
 * @brief            Control the mobile phone to stop music
 *
 **/
void bt_interface_avrcp_stop(void);

/**
 * @brief            Control the phone to turn up the volume
 *
 **/
void bt_interface_avrcp_volume_up(void);

/**
 * @brief            Control the phone to turn down the volume
 *
 **/
void bt_interface_avrcp_volume_down(void);

/**
 * @brief            Control the phone to switch to the previous music
 *
 **/
void bt_interface_avrcp_previous(void);

/**
 * @brief            Control the phone rewind
 *
 **/
void bt_interface_avrcp_rewind(void);

/**
 * @brief            As CT role to adjust the volume of remote device through avrcp
 * @param[in] volume The volume value you want to adjust
 * @return           The result of adjusting the volume
 *
 **/
bt_err_t bt_interface_avrcp_set_absolute_volume_as_ct_role(U8 volume);

/**
 * @brief            As TG role to adjust the volume of remote device through avrcp
 * @param[in] volume The volume value you want to adjust
 *
 **/
bt_err_t bt_interface_avrcp_set_absolute_volume_as_tg_role(U8 volume);

/**
 * @brief            playback status register request
 *
 **/
void bt_interface_avrcp_playback_register_request(void);

/**
 * @brief            playback pos change register request
 *
 **/
void bt_interface_avrcp_playback_pos_register_request(void);

/**
 * @brief            track change register request
 *
 **/
void bt_interface_avrcp_track_change_register_request(void);

/**
 * @brief            volume change register request
 *
 **/
void bt_interface_avrcp_volume_change_register_request(void);

/**
 * @brief            track change register request
 * @param[in] media_attribute The media attribute value you want to get
 *
 **/
void bt_interface_avrcp_get_element_attributes_request(U8 media_attribute);

/**
 * @brief            play status register request
 *
 **/
void bt_interface_avrcp_get_play_status_request(void);

/**
 * @brief            Set the playback status of avrcp
 * @param[in] playback_status The playback status of avrcp
 *
 **/
void bt_interface_avrcp_set_playback_status(U8 playback_status);

/**
 * @brief            Get the playback status of avrcp
 *
 **/
void bt_interface_avrcp_set_can_play(void);

/**
 * @brief            Check the avrcp role valid
 * @param[in] role   Avrcp role
 * @return           Is the role valid?
 *
 **/
BOOL bt_interface_check_avrcp_role_valid(U8 role);

/**
 * @brief            Set the avrcp role
 * @param[in] bd_addr    The pointer of bd address
 * @param[in] role    Avrcp role
 * @return           The results of send spp data
 *
 **/
bt_err_t bt_interface_set_avrcp_role(BTS2S_BD_ADDR *bd_addr, U8 role);

/**
 * @brief            Get the playback status of avrcp
 * @return           The playback status of avrcp
 *
 **/
U8 bt_interface_avrcp_get_playback_status(void);
/// @}  BT_AVRCP_SRV

/** @defgroup BT_HID_SRV  HID profile interfaces
  * @{
  */

/**
 * @brief            Open the function of hid profile
 *
 **/
void bt_interface_open_hid(void);

/**
 * @brief            Close the function of hid profile
 *
 **/
void bt_interface_close_hid(void);

/**
 * @brief            Set whether the peer device is an ios device
 * @param[in] is_ios IOS device flag
 *
 **/
void bt_interface_set_hid_device(U8 is_ios);

/**
 * @brief            Control the mobile phone to page up
 *
 **/
void bt_interface_phone_drag_up(void);

/**
 * @brief            Control the mobile phone to page down
 *
 **/
void bt_interface_phone_drag_down(void);

/**
 * @brief            Control mobile phone click once
 *
 **/
void bt_interface_phone_once_click(void);

/**
 * @brief            Control mobile phone click twice
 *
 **/
void bt_interface_phone_double_click(void);

/**
 * @brief            Control mobile phone take a picture
 *
 **/
void bt_interface_phone_take_picture(void);

/**
 * @brief            Control the phone to lower the volume
 *
 **/
void bt_interface_phone_volume_down(void);

/**
 * @brief            Add hid descriptor
 * @param[in] data   The pointer of hid descriptor
 * @param[in] len    The length of hid descriptor
 *
 **/
void bt_interface_add_hid_descriptor(U8 *data, U8 len);
/// @}  BT_HID_SRV

/** @defgroup BT_SPP_SRV  SPP profile interfaces
  * @{
  */

/**
 * @brief            Send data through spp
 * @param[in] data   The pointer of spp data
 * @param[in] len    The length of spp data
 * @param[in] bd_addr    The pointer of bd address
 * @param[in] srv_chl    The service channel of spp
 * @return           The results of send spp data
 *
 **/
bt_err_t bt_interface_spp_send_data(U8 *data, U16 len, BTS2S_BD_ADDR *bd_addr, U8 srv_chl);

/**
 * @brief            Add spp uuid
 * @param[in] uuid   The pointer of spp uuid
 * @param[in] uuid_len    The length of spp uuid
 * @param[in] srv_name    The service name of spp uuid
 *
 **/
bt_err_t bt_interface_add_spp_uuid(U8 *uuid, U8 uuid_len, char *srv_name);

/**
 * @brief            Send spp data response
 * @param[in] bd_addr    The pointer of bd address
 * @param[in] srv_chl    The service channel of spp
 * @return           The results of send spp data response
 *
 **/
bt_err_t bt_interface_spp_srv_data_rsp(BTS2S_BD_ADDR *bd_addr, U8 srv_chl);

/**
 * @brief            Disconnect spp by bd address and service channel
 * @param[in] bd_addr    The pointer of bd address
 * @param[in] srv_chl    The service channel of spp
 * @return           The results of disconnect spp
 *
 **/
bt_err_t bt_interface_dis_spp_by_addr_and_chl(BTS2S_BD_ADDR *bd_addr, U8 srv_chl);

/**
 * @brief            Disconnect all connected spp
 * @return           The results of disconnect spp
 *
 **/
bt_err_t bt_interface_dis_spp_all(void);
/// @}  BT_SPP_SRV

#if defined(CFG_BR_GATT_SRV) || defined(_SIFLI_DOXYGEN_)

/** @defgroup BR_GATT_SRV  GATT over BR profile interfaces
  * @{
  */

/**
 * @brief                        This function register gatt's sdp_info
 * @param[in] sdp_info           BLE gatt UUID information
 *
 * @return                       void
 **/
void bt_interface_bt_gatt_reg(br_att_sdp_data_t *sdp_info);

/**
 * @brief                        This function unregister gatt's sdp_info
 * @param[in] sdp_hdl            Sdp_hdl is the value when gatt_sdp_register callback
 *
 * @return                       void
 **/
void bt_interface_bt_gatt_unreg(U32 sdp_hdl);

/**
 * @brief                        Change l2cap config MTU para
 * @param[in] mtu                MTU value
 *
 * @return                       void
 **/
void bt_interface_bt_gatt_mtu_changed(U16 mtu);
/// @}  BR_GATT_SRV
#endif


#if defined(BT_USING_AG) || defined(_SIFLI_DOXYGEN_)

/** @defgroup HFP_AG  HFP_AG profile interfaces
  * @{
  */

/**
 * @brief                        Update phone call state
 * @param[in] call_info          Call information
 *
 * @return                       void
 **/
void bt_interface_phone_state_changed(HFP_CALL_INFO_T *call_info);

/**
 * @brief                        Send subscriber number response for AT+CNUM
 * @param[in] local_phone_num    Local phone number information
 *
 * @return                       void
 **/
void bt_interface_local_phone_info_res(hfp_phone_number_t *local_phone_num);

/**
 * @brief                        Response for all call info request(AT+CLCC)
 * @param[in] calls_info         All call information
 *
 * @return                       void
 **/
void bt_interface_remote_call_info_res(hfp_remote_calls_info_t *calls_info);

/**
 * @brief                        Response all indicators information for remote device
 * @param[in] cind_status        All indicators information
 *
 * @return                       void
 **/
void bt_interface_get_all_indicator_info_res(hfp_cind_status_t *cind_status);

/**
 * @brief                        Update indicator status
 * @param[in] ind_info           indicator information
 *
 * @return                       void
 **/
void bt_interface_indicator_status_changed(HFP_IND_INFO_T *ind_info);

/**
 * @brief                        Update speaker volume
 * @param[in] vol                Volume value
 *
 * @return                       void
 **/
void bt_interface_spk_vol_change_req(U8 vol);

/**
 * @brief                        Update microphone volume
 * @param[in] vol                Volume value
 *
 * @return                       void
 **/
void bt_interface_mic_vol_change_req(U8 vol);

/**
 * @brief                        The response of make_call request
 * @param[in] res                OK (0)/ERROR (1)
 *
 * @return                       void
 **/
void bt_interface_make_call_res(U8 res);

/**
 * @brief                       Create/Close an audio connection
 * @param[in] bt_hfp_audio_switch_t
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_ag_audio_switch(bt_hfp_audio_switch_t *audio);
/// @}  HFP_AG
#endif

#if defined(CFG_HFP_HF) || defined(_SIFLI_DOXYGEN_)

/** @defgroup HFP_HF  HFP_HF profile interfaces
  * @{
  */

/**
 * @brief                        Connect hf profile request
 * @param[in] mac                Connect with the specified device address
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_hfp_hf_start_connecting(unsigned char *mac);

/**
 * @brief                        Get subscriber information request
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_get_ph_num(void);

/**
 * @brief                        Get all call information request
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_get_remote_ph_num(void);

/**
 * @brief                        Get current call status (only CIEV) request
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_get_remote_call_status(void);

/**
 * @brief                        Place a call using number
 * @param[in] len                Phone number length
 * @param[in] data               Phone number
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_hf_out_going_call(int len, void *data);

/**
 * @brief                        Place call to last dialed number
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_start_last_num_dial_req_send(void);

/**
 * @brief                        Answer a call
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_start_hf_answer_req_send(void);

/**
 * @brief                        Hang up a call
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_handup_call(void);

/**
 * @brief                        Transmit DTMF tone
 * @param[in] key                DTMF tone
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_start_dtmf_req_send(char key);

/**
 * @brief                        Call hold and multiparty handling AT command.
 * @param[in] cmd                this specification only covers values for (n） of 0, 1, 1(idx),
 *                               2, 2(idx), 3 and 4, where:
 *                                 0 = Releases all held calls or sets User Determined User Busy
 *                                 (UDUB) for a waiting call.
 *
 *                                 1 = Releases all active calls (if any exist) and accepts the other
 *                                 (held or waiting) call.
 *                                 1(idx) = Releases specified active call only ((idx)).
 *
 *                                 2 = Places all active calls (if any exist) on hold and accepts
 *                                 the other (held or waiting) call.
 *                                 2(idx) = Request private consultation mode with specified call
 *                                ((idx)). (Place all calls on hold EXCEPT the call indicated by (idx).)
 *
 *                                 3 = Adds a held call to the conversation.
 *
 *                                 4 = Connects the two calls and disconnects the subscriber from
 *                                 both calls (Explicit Call Transfer). Support for this value
 *                                 and its associated functionality is optional for the HF.
 *
 * @param[in] idx                current call index
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_hf_3way_hold(bt_3way_coded_t cmd, int idx);

/**
 * @brief                        Query response and hold status.
 * @param[in] cmd                cmd
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_hf_3way_btrh(bt_3way_incom_t cmd);

/**
 * @brief                        Call waiting notification control
 * @param[in] enable             Enable（1）/Disable（0）call waiting notification
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_hf_3way_ccwa(unsigned int enable);

/**
 * @brief                        Voice recognition control
 * @param[in] flag               Enable（1）/Disable（0）voice recognition
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_voice_recog(uint8_t flag);


/**
 * @brief                        Create/Close an audio connection
 * @param[in] type               Create(0)/Close（1） an audio connection
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_audio_switch(U8 type);

/**
 * @brief                        Speaker volume control
 * @param[in] volume             Volume value
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_set_speaker_volume(int volume);

/**
 * @brief                        Update device battery value
 * @param[in] batt_val           Current battery value
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_hf_update_battery(U8 batt_val);

/**
 * @brief                         Codec negotiation control
 *
 * @param[in] status              Enable（1）/Disable（0）codec negotiation
 *
 * @return                       bt_err_t
 **/
bt_err_t bt_interface_set_wbs_status(U8 status);

/// @}  HFP_HF

#endif

#ifdef BT_FINSH_PAN
/** @defgroup BT_PAN_SRV  BT PAN profile interfaces
  * @{
  */

/**
 * @brief                         update the address used by pan
 *
 * @param[in] bd_addr              the address used by pan
 *
 **/
void bt_interface_update_pan_addr(BTS2S_BD_ADDR *bd_addr);

/**
 * @brief                         check whether pan profile is in sniff mode
 *
 * @return                        is it in sniff mode?
 *
 **/
BOOL bt_interface_check_pan_in_sniff(void);

/// @}  BT_PAN_SRV
#endif

/*****************************************************notify start********************************************************************/

/** @defgroup bt_notify  BT Result Callback interfaces
 * @{
 */

typedef enum
{
    BT_INTERFACE_STATUS_OK,
    BT_INTERFACE_STATUS_NO_MEMORY,
    BT_INTERFACE_STATUS_ALREADY_EXIST,
    BT_INTERFACE_STATUS_NOT_EXIST,
} bt_interface_status_t;

typedef int (*register_bt_notify_func_t)(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len);

typedef struct bt_notify_register_callback_t
{
    register_bt_notify_func_t function;
    struct bt_notify_register_callback_t *next;
} bt_notify_register_callback_t;

/**
 * @brief                         BT event notify callback function
 *
 * @param[in] type              Event module_id
 * @param[in] event_id          Event identifier
 * @param[in] data              Event result
 * @param[in] data_len          Event result data length
 *
 * @return                       void
 **/
void bt_interface_bt_event_notify(uint16_t type, uint16_t event_id, void *data, uint16_t data_len);


/**
 * @brief                        This function is called to register to bt notify callback
 *
 * @param[in] func               Need register function
 *
 * @return                       bt_interface_status_t
 **/
bt_interface_status_t bt_interface_register_bt_event_notify_callback(register_bt_notify_func_t func);

/**
 * @brief                        This function is called to unregister to bt notify callback
 *
 * @param[in] func               Need unregister function
 *
 * @return                       bt_interface_status_t
 **/
bt_interface_status_t bt_interface_unregister_bt_event_notify_callback(register_bt_notify_func_t func);
/// @}  bt_notify
/******************************************************notify end*******************************************************************/

/// @}  BT_interface

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

