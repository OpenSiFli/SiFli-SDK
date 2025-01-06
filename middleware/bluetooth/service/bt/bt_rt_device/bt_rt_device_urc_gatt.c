/**
  ******************************************************************************
  * @file   bt_rt_device_urc_gatt.c
  * @author Sifli software development team
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2022,  Sifli Technology
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

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
//#include "utf8_unicode.h"
#include "bt_rt_device.h"
#include "bts2_global.h"
#include "bts2_app_inc.h"

#define DBG_TAG               "bt_rt_device.urc_gatt"
//#define DBG_LVL               DBG_INFO
#include <log.h>


void urc_func_bt_gatt_reg_res(U32 sdp_hdl, U8 res)
{
    bt_notify_t args;
    args.event = BT_EVENT_BT_GATT_REG_RES;
    args.args = &sdp_hdl;
    rt_bt_event_notify(&args);
}

void urc_func_bt_gatt_unreg_res(U32 sdp_hdl, U8 res)
{
    bt_notify_t args;
    args.event = BT_EVENT_BT_GATT_UNREG_RES;
    args.args = &res;
    rt_bt_event_notify(&args);
}

void urc_func_bt_gatt_mtu_res(U8 res)
{
    bt_notify_t args;
    args.event = BT_EVENT_BT_GATT_MTU_RES;
    args.args = &res;
    rt_bt_event_notify(&args);
}

int bt_sifli_notify_gatt_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_GATT_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_conn_sifli(profile_info->mac.addr, BT_PROFILE_BT_GATT);
        break;
    }
    case BT_NOTIFY_GATT_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_disc_sifli(profile_info->mac.addr, BT_PROFILE_BT_GATT, profile_info->res);
        break;
    }
    case BT_NOTIFY_GATT_REGISTER_RESPONSE:
    {
        bt_notify_gatt_sdp_info_t *sdp_info = (bt_notify_gatt_sdp_info_t *)data;
        urc_func_bt_gatt_reg_res(sdp_info->sdp_rec_hdl, sdp_info->res);
        break;
    }
    case BT_NOTIFY_GATT_UNREGISTER_RESPONSE:
    {
        bt_notify_gatt_sdp_info_t *sdp_info = (bt_notify_gatt_sdp_info_t *)data;
        urc_func_bt_gatt_unreg_res(sdp_info->sdp_rec_hdl, sdp_info->res);
        break;
    }
    case BT_NOTIFY_GATT_CHANGE_MTU_RESPONSE:
    {
        urc_func_bt_gatt_mtu_res(data[0]);
        break;
    }
    default:
        return -1;
    }

    return 0;
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

