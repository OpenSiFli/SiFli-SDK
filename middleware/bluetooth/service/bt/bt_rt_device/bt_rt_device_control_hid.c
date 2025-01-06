/**
  ******************************************************************************
  * @file   bt_rt_device_control_hid.c
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
        bt_interface_phone_drag_up();
    }
    break;

    case BT_CONTROL_PHONE_DRAG_DOWN:
    {
        bt_interface_phone_drag_down();
    }
    break;

    case BT_CONTROL_PHONE_ONCE_CLICK:
    {
        bt_interface_phone_once_click();
    }
    break;

    case BT_CONTROL_PHONE_DOUBLE_CLICK:
    {
        bt_interface_phone_double_click();
    }
    break;

    case BT_CONTROL_PHONE_VOLUME_UP:
    case BT_CONTROL_PHONE_TAKE_PICTURE:
    {
        bt_interface_phone_take_picture();
    }
    break;

    case BT_CONTROL_PHONE_VOLUME_DOWN:
    {
        bt_interface_phone_volume_down();
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

