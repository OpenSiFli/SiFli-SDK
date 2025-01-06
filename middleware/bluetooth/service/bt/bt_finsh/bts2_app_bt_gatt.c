/**
  ******************************************************************************
  * @file   bts2_app_bt_gatt.c
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

#include "bts2_global.h"
#ifdef CFG_BR_GATT_SRV

#define LOG_TAG         "btapp_gatt"
#include "log.h"

#include "bts2_app_inc.h"

static void bt_gatt_device_state_change(bts2_app_stru *bts2_app_data, BTS2S_BT_GATT_RES *con_msg)
{
    if (con_msg->device_state == BT_GATT_CONNECTED)
    {
        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&con_msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_GATT;
        profile_state.res = BTS2_SUCC;
        bt_interface_bt_event_notify(BT_NOTIFY_GATT, BT_NOTIFY_GATT_PROFILE_CONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));
    }
    else if (con_msg->device_state == BT_GATT_DISCONNECTED)
    {
        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&con_msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_GATT;
        profile_state.res = con_msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_GATT, BT_NOTIFY_GATT_PROFILE_DISCONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));
    }
}

void bt_bt_gatt_msg_hdl(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    msg_type = (U16 *)bts2_app_data->recv_msg;
    USER_TRACE("bt_bt_gatt_msg_hdl *msg_type %x\n", *msg_type);
    switch (*msg_type)
    {
    case BTS2MU_BT_GATT_SDP_RES:
    {
        bt_notify_gatt_sdp_info_t sdp_reg_info;
        bt_gatt_sdp_reg_info *sdp_info;
        sdp_info = (bt_gatt_sdp_reg_info *)bts2_app_data->recv_msg;
        sdp_reg_info.sdp_rec_hdl = sdp_info->sdp_rec_hdl;
        sdp_reg_info.res = sdp_info->res;
        bt_interface_bt_event_notify(BT_NOTIFY_GATT, BT_NOTIFY_GATT_REGISTER_RESPONSE,
                                     &sdp_reg_info, sizeof(bt_notify_gatt_sdp_info_t));
        break;
    }
    case BTS2MU_BT_GATT_SDP_UNRES:
    {
        bt_notify_gatt_sdp_info_t sdp_unreg_info;
        bt_gatt_sdp_reg_info *sdp_info;
        sdp_info = (bt_gatt_sdp_reg_info *)bts2_app_data->recv_msg;
        sdp_unreg_info.sdp_rec_hdl = sdp_info->sdp_rec_hdl;
        sdp_unreg_info.res = sdp_info->res;
        bt_interface_bt_event_notify(BT_NOTIFY_GATT, BT_NOTIFY_GATT_UNREGISTER_RESPONSE,
                                     &sdp_unreg_info, sizeof(bt_notify_gatt_sdp_info_t));
        break;
    }
    case BTS2MU_BT_GATT_MTU_RES:
    {
        bt_gatt_mtu_res_info *mtu_info;
        mtu_info = (bt_gatt_mtu_res_info *)bts2_app_data->recv_msg;
        bt_interface_bt_event_notify(BT_NOTIFY_GATT, BT_NOTIFY_GATT_CHANGE_MTU_RESPONSE,
                                     &mtu_info->res, sizeof(uint8_t));
        break;
    }
    case BTS2MD_BT_GATT_CONN_RES:
    {
        break;
    }
    case BTS2MD_BT_GATT_DISC_RES:
    {
        break;
    }

    case BTS2MU_BT_GATT_CONN_STATE:
    {
        BTS2S_BT_GATT_RES *con_msg;
        con_msg = (BTS2S_BT_GATT_RES *)bts2_app_data->recv_msg;
        bt_gatt_device_state_change(bts2_app_data, con_msg);
        break;
    }
    }

}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/


