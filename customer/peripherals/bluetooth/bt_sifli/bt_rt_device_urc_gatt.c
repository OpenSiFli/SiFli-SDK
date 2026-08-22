/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
    LOG_I("URC gatt reg_hdl:%d res:%d", sdp_hdl, res);
}

void urc_func_bt_gatt_unreg_res(U32 sdp_hdl, U8 res)
{
    bt_notify_t args;
    args.event = BT_EVENT_BT_GATT_UNREG_RES;
    args.args = &res;
    rt_bt_event_notify(&args);
    LOG_I("URC gatt unreg_hdl:%d res:%d", sdp_hdl, res);
}

void urc_func_bt_gatt_mtu_res(U8 res)
{
    bt_notify_t args;
    args.event = BT_EVENT_BT_GATT_MTU_RES;
    args.args = &res;
    rt_bt_event_notify(&args);
    LOG_I("URC gatt mtu res:%d", res);
}

int bt_sifli_notify_gatt_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_GATT_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_conn_sifli(profile_info, BT_PROFILE_BT_GATT);
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

