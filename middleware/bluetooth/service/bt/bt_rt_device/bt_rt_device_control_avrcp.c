/**
  ******************************************************************************
  * @file   bt_rt_device_control_avrcp.c
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

#define DBG_TAG               "bt_rt_device.control_avrcp"
//#define DBG_LVL               DBG_INFO
#include <log.h>

extern uint8_t a2dp_set_speaker_volume(uint8_t volume);

#ifdef BT_USING_AVRCP
bt_err_t bt_sifli_set_avrcp_volume(rt_bt_device_t *dev, bt_volume_set_t *set)
{
    int volume, temp_volume;
    temp_volume = set->volume.media_volume;
    bt_err_t ret = BT_EOK;
    uint8_t volume_conversion[16] = {0, 8, 16, 25, 34, 43, 52, 61, 70, 79, 87, 96, 104, 112, 120, 127};

    if (temp_volume < 16)
    {
        volume = volume_conversion[temp_volume];
    }
    else
    {
        volume = 127;
    }
#ifndef BT_CONNECT_SUPPORT_MULTI_LINK
    if (BT_STATE_CONNECTED == rt_bt_get_connect_state(dev, BT_PROFILE_AVRCP))
#endif
    {
        if (BT_ROLE_MASTER == dev->role)
        {
            ret = bt_interface_set_absolute_volume(volume);
        }
        else
        {
            ret = bt_interface_avrcp_volume_changed(volume);
        }
    }

    if (BT_EOK == ret)
    {
        a2dp_set_speaker_volume(volume);
    }

    return ret;
}
#endif

bt_err_t bt_sifli_control_avrcp(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case BT_CONTROL_OPEN_AVRCP:
    {
        LOG_I("open avrcp ,event %x", bt_sifli_get_bt_event());
        if (bt_sifli_check_bt_event(BT_SET_AVRCP_CLOSE_EVENT))
        {
            LOG_I("during avrcp close porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_AVRCP_OPEN_EVENT);
            bt_interface_open_avrcp();
        }
    }
    break;

    case BT_CONTROL_CLOSE_AVRCP:
    {
        LOG_I("close avrcp ,event %x", bt_sifli_get_bt_event());
        if (bt_sifli_check_bt_event(BT_SET_AVRCP_OPEN_EVENT))
        {
            LOG_I("during avrcp open porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_AVRCP_CLOSE_EVENT);
            bt_interface_close_avrcp();
        }
    }
    break;
    case BT_CONTROL_PHONE_PLAY_NEXT:
    {
        bt_interface_phone_play_next();
    }
    break;

    case BT_CONTROL_PHONE_PLAY:
    {
        bt_interface_phone_play();
    }
    break;

    case BT_CONTROL_PHONE_PLAY_SUSPEND:
    {
        bt_interface_phone_play_pause();
    }
    break;

    case BT_CONTROL_PHONE_PLAY_STOP:
    {
        bt_interface_phone_play_stop();
    }
    break;

    case BT_CONTROL_AVRCP_VOLUME_UP:
    {
        bt_interface_avrcp_volume_up();
    }
    break;

    case BT_CONTROL_AVRCP_VOLUME_DOWN:
    {
        bt_interface_avrcp_volume_down();
    }
    break;

    case BT_CONTROL_PHONE_PLAY_PREVIOUS:
    {
        bt_interface_phone_play_previous();
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

