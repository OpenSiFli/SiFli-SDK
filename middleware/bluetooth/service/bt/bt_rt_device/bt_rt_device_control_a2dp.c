/**
  ******************************************************************************
  * @file   bt_rt_device_control_a2dp.c
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

#define DBG_TAG               "bt_rt_device.control_a2dp"
//#define DBG_LVL               DBG_INFO
#include <log.h>

bt_err_t bt_sifli_control_a2dp(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case BT_CONTROL_OPEN_AVSINK:
    {
        if (bt_sifli_check_bt_event(BT_SET_AVSNK_CLOSE_EVENT))
        {
            LOG_I("during a2dp close porcess");
            return BT_ERROR_STATE;
        }
        else if (bt_sifli_check_bt_event(BT_SET_AVSNK_OPEN_EVENT))
        {
            LOG_I("during a2dp open porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_AVSNK_OPEN_EVENT);
            bt_interface_open_avsink();
        }
    }
    break;

    case BT_CONTROL_CLOSE_AVSINK:
    {
        if (bt_sifli_check_bt_event(BT_SET_AVSNK_OPEN_EVENT))
        {
            LOG_I("during a2dp open porcess");
            return BT_ERROR_STATE;
        }
        else if (bt_sifli_check_bt_event(BT_SET_AVSNK_CLOSE_EVENT))
        {
            LOG_I("during a2dp close porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_AVSNK_CLOSE_EVENT);
            ret = bt_interface_close_avsink();
        }
    }
    break;

    case BT_CONTROL_UNREGISTER_AVSINK_SDP:
    {
        bt_interface_unregister_av_snk_sdp();
    }
    break;

    case BT_CONTROL_REGISTER_AVSINK_SDP:
    {
        bt_interface_register_av_snk_sdp();
    }
    break;

    case BT_CONTROL_SET_A2DP_BQB_TEST:
    {
        int *set = (int *)args;
        bt_interface_set_a2dp_bqb_test(*set);
    }
    break;

    case BT_CONTROL_SET_A2DP_SRC_AUDIO_DEVICE:
    {
        U8 *device_type = (U8 *)args;
        bt_interface_set_audio_device(*device_type);
    }
    break;

    case BT_CONTROL_RELEASE_A2DP:
    {
        bt_interface_release_a2dp();
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

