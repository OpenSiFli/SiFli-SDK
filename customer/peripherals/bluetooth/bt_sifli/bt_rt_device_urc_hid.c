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

#define DBG_TAG               "bt_rt_device.urc_a2dp"
//#define DBG_LVL               DBG_INFO
#include <log.h>


void urc_func_bt_hid_open_complete_sifli(void)
{
    bt_notify_t args;
    args.event = BT_EVENT_HID_OPEN_COMPLETE;
    args.args = RT_NULL;
    if (bt_sifli_check_bt_event(BT_SET_HID_OPEN_EVENT))
    {
        LOG_I("URC BT hid open complete ind %x", bt_sifli_get_bt_event());
        bt_sifli_reset_bt_event(BT_SET_HID_OPEN_EVENT);
        rt_bt_event_notify(&args);
    }
    return;
}

void urc_func_bt_hid_close_complete_sifli(void)
{
    bt_notify_t args;
    args.event = BT_EVENT_HID_CLOSE_COMPLETE;
    args.args = RT_NULL;
    if (bt_sifli_check_bt_event(BT_SET_HID_CLOSE_EVENT))
    {
        LOG_I("URC BT hid close complete ind %x", bt_sifli_get_bt_event());
        bt_sifli_reset_bt_event(BT_SET_HID_CLOSE_EVENT);
        rt_bt_event_notify(&args);
    }
    return;
}

int bt_sifli_notify_hid_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_HID_CLOSE_COMPLETE:
    {
        urc_func_bt_hid_close_complete_sifli();
        break;
    }
    case BT_NOTIFY_HID_OPEN_COMPLETE:
    {
        urc_func_bt_hid_open_complete_sifli();
        break;
    }
    case BT_NOTIFY_HID_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_conn_sifli(profile_info, BT_PROFILE_HID);
        break;
    }
    case BT_NOTIFY_HID_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_disc_sifli(profile_info->mac.addr, BT_PROFILE_HID, profile_info->res);
        break;
    }
    default:
        return -1;
    }

    return 0;
}

