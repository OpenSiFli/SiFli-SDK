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
#include "bt_rt_device.h"
#include "bts2_global.h"
#include "bts2_app_inc.h"
#include "bf0_ble_common.h"
#include "bf0_sibles.h"
#include "bt_connection_manager.h"

#define DBG_TAG               "bt_rt_device.control_hid"
//#define DBG_LVL               DBG_INFO
#include <log.h>

bt_err_t bt_sifli_control_hid(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case BT_CONTROL_OPEN_HID:
    {
        LOG_I("open hid ,event %x", bt_sifli_get_bt_event());
        if (bt_sifli_check_bt_event(BT_SET_HID_CLOSE_EVENT))
        {
            LOG_I("during hid close porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_HID_OPEN_EVENT);
            bt_interface_open_hid();
        }
    }
    break;

    case BT_CONTROL_CLOSE_HID:
    {
        LOG_I("close hid ,event %x", bt_sifli_get_bt_event());
        if (bt_sifli_check_bt_event(BT_SET_HID_OPEN_EVENT))
        {
            LOG_I("during hid open porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_HID_CLOSE_EVENT);
            bt_interface_close_hid();
        }
    }
    break;

    case BT_CONTROL_SET_HID_DEVICE:
    {
        U8 *is_ios = (U8 *)args;
        LOG_I("set hid device is ios %d", *is_ios);
        bt_interface_set_hid_device(*is_ios);
    }
    break;

    case BT_CONTROL_PHONE_DRAG_UP:
    {
        bt_interface_hid_mouse_drag_up();
    }
    break;

    case BT_CONTROL_PHONE_DRAG_DOWN:
    {
        bt_interface_hid_mouse_drag_down();
    }
    break;

    case BT_CONTROL_PHONE_ONCE_CLICK:
    {
        bt_interface_hid_mouse_once_left_click();
    }
    break;

    case BT_CONTROL_PHONE_DOUBLE_CLICK:
    {
        bt_interface_hid_mouse_double_left_click();
    }
    break;

    case BT_CONTROL_PHONE_VOLUME_UP:
    case BT_CONTROL_PHONE_TAKE_PICTURE:
    {
        bt_interface_hid_consumer_take_picture();
    }
    break;

    case BT_CONTROL_PHONE_VOLUME_DOWN:
    {
        bt_interface_hid_consumer_volume_down();
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

