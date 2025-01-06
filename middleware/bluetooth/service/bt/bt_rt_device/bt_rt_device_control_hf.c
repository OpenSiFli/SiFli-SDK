/**
  ******************************************************************************
  * @file   bt_rt_device_control_hfp.c
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

#define DBG_TAG               "bt_rt_device.control_hfp"
//#define DBG_LVL               DBG_INFO
#include <log.h>

#ifdef BT_USING_SIRI
#define BT_SIRI_TIMEOUT       (15000)
rt_timer_t  siri_timer_hdl = NULL;

static void bt_siri_timeout(void *parameter)
{
    LOG_I("siri timeout");
    if (NULL != siri_timer_hdl)
    {
        urc_func_bt_hf_voice_recog_sifli(BTS2_TIMEOUT);
    }
}
#endif


rt_timer_t  audio_transfer_timer_hdl = NULL;
static int  audio_transfer_state;

static void bt_sifli_control_clcc_get(void)
{
    bt_err_t ret;
    if (bt_sifli_check_bt_event(BT_SET_CLCC_EVENT))
    {
        return;
    }
    else
    {
        bt_sifli_set_bt_event(BT_SET_CLCC_EVENT);
    }
    ret = bt_interface_get_remote_ph_num();
    LOG_D("%s: ret:%d\n", __func__, ret);
    if (BT_EOK != ret) bt_sifli_reset_bt_event(BT_SET_CLCC_EVENT);
    return;
}

static void bt_sifli_audio_transfer_timeout(void *parameter)
{
    LOG_I("audio transfer timeout:%d", audio_transfer_state);
    bt_interface_audio_switch(audio_transfer_state);
    return;
}

static void bt_sifli_audio_transfer(void)
{
    if (NULL == audio_transfer_timer_hdl)
    {
        audio_transfer_timer_hdl = rt_timer_create("audio_transfer", bt_sifli_audio_transfer_timeout, NULL,
                                   rt_tick_from_millisecond(300), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }
    RT_ASSERT(audio_transfer_timer_hdl);
    rt_timer_stop(audio_transfer_timer_hdl);
    rt_timer_start(audio_transfer_timer_hdl);
    return;
}

bt_err_t bt_sifli_control_hf(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case BT_CONTROL_MAKE_CALL:
    {
        if (bt_sifli_check_bt_event(BT_SET_DIAL_EVENT))
        {
            LOG_I("during dial process");
            return BT_ERROR_STATE;
        }
        else
        {
            phone_number_t   *p_args;
            p_args = (phone_number_t *)args;
            bt_sifli_set_bt_event(BT_SET_DIAL_EVENT);
            bt_interface_hf_out_going_call(p_args->size, (void *)(p_args->number));
        }
    }
    break;

    case BT_CONTROL_PHONE_CONNECT:
    {
        bt_interface_start_hf_answer_req_send();
    }
    break;
    case BT_CONTROL_PHONE_HANDUP:
    {
        bt_interface_handup_call();
        //fix bug2012
        //urc_func_link_down_sifli();
    }
    break;

    case BT_CONTROL_DIAL_BACK:
    {
        bt_interface_start_last_num_dial_req_send();
    }
    break;

    case BT_CONTROL_3WAY_HOLD:
    {
        bt_3way_hold_t   *p_args;
        p_args = (bt_3way_hold_t *)args;
        bt_interface_hf_3way_hold(p_args->cmdCode, p_args->index);
    }
    break;

    case BT_CONTROL_3WAY_BTRH:
    {
        bt_3way_incom_t  *cmd;
        cmd = (bt_3way_incom_t *)args;
        bt_interface_hf_3way_btrh(*cmd);
    }
    break;

    case BT_CONTROL_3WAY_CCWA:
    {
        bt_interface_hf_3way_ccwa(*(unsigned int *)args);
    }

#ifdef BT_USING_DTMF
    case BT_CONTROL_DTMF_DIAL:
    {
        bt_dtmf_key_t *dial_key = (bt_dtmf_key_t *)args;
        char key_str[2] = {0};
        switch (*dial_key)
        {
        case BT_DTMF_KEY_0:
            key_str[0] = '0';
            break;

        case BT_DTMF_KEY_1:
            key_str[0] = '1';
            break;

        case BT_DTMF_KEY_2:
            key_str[0] = '2';
            break;

        case BT_DTMF_KEY_3:
            key_str[0] = '3';
            break;

        case BT_DTMF_KEY_4:
            key_str[0] = '4';
            break;

        case BT_DTMF_KEY_5:
            key_str[0] = '5';
            break;

        case BT_DTMF_KEY_6:
            key_str[0] = '6';
            break;

        case BT_DTMF_KEY_7:
            key_str[0] = '7';
            break;

        case BT_DTMF_KEY_8:
            key_str[0] = '8';
            break;

        case BT_DTMF_KEY_9:
            key_str[0] = '9';
            break;

        case BT_DTMF_KEY_STAR:
            key_str[0] = '*';
            LOG_D("key_str %x \n", key_str[0]);
            break;

        case BT_DTMF_KEY_HASH:
            key_str[0] = '#';
            break;

        default:
            return BT_ERROR_INPARAM;
        }

        if (bt_sifli_check_bt_event(BT_SET_DTMF_EVENT) || (BT_STATE_CONNECTED != rt_bt_get_connect_state(bt_handle, BT_PROFILE_HFP)))
        {
            LOG_I("set DMTF failed");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_DTMF_EVENT);
            bt_interface_start_dtmf_req_send(key_str[0]);
        }
    }
    break;
#endif
    case BT_CONTROL_UPDATE_BATT_BY_HFP:
    {
        U8 *batt_val = (U8 *)args;
        if (*batt_val > 9)
        {
            return BT_ERROR_INPARAM;
        }
        bt_interface_hf_update_battery(*batt_val);
        break;
    }
    case BT_CONTROL_GET_REMOTE_CALL_STATUS:
    {
        bt_interface_get_remote_call_status();
        break;
    }
    case BT_CONTROL_SET_INBAND_RING:
    {
    }
    break;

    case BT_CONTROL_AUDIO_TRANSFER:
    {
        audio_transfer_state = *((int *)args);
        bt_sifli_audio_transfer();
    }
    break;

#ifdef BT_USING_SIRI
    case BT_CONTROL_SIRI_ON:
    {
#ifndef BT_CONNECT_SUPPORT_MULTI_LINK
        if (rt_bt_get_connect_state(bt_handle, BT_PROFILE_HFP) != BT_STATE_CONNECTED)
        {
            return BT_ERROR_DISCONNECTED;
        }
#endif
        if (bt_sifli_check_bt_event(BT_SET_SIRI_OFF_EVENT))
        {
            LOG_I("during siri off porcess");
            return BT_ERROR_STATE;
        }
        else if (bt_sifli_check_bt_event(BT_SET_SIRI_ON_EVENT))
        {
            LOG_I("during siri on porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_SIRI_ON_EVENT);
            bt_interface_voice_recog(TRUE);
            if (NULL == siri_timer_hdl)
            {
                siri_timer_hdl = rt_timer_create("bt_siri", bt_siri_timeout, NULL,
                                                 rt_tick_from_millisecond(BT_SIRI_TIMEOUT), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
            }
            rt_timer_start(siri_timer_hdl);
            LOG_I("sirion timer start");
        }
    }
    break;

    case BT_CONTROL_SIRI_OFF:
    {
#ifndef BT_CONNECT_SUPPORT_MULTI_LINK
        if (rt_bt_get_connect_state(bt_handle, BT_PROFILE_HFP) != BT_STATE_CONNECTED)
        {
            return BT_ERROR_DISCONNECTED;
        }
#endif
        if (bt_sifli_check_bt_event(BT_SET_SIRI_ON_EVENT))
        {
            LOG_I("during siri on porcess");
            return BT_ERROR_STATE;
        }
        else if (bt_sifli_check_bt_event(BT_SET_SIRI_OFF_EVENT))
        {
            LOG_I("during siri off porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_SIRI_OFF_EVENT);
            bt_interface_voice_recog(FALSE);
            if (NULL == siri_timer_hdl)
            {
                siri_timer_hdl = rt_timer_create("bt_siri", bt_siri_timeout, NULL,
                                                 rt_tick_from_millisecond(BT_SIRI_TIMEOUT), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
            }
            rt_timer_start(siri_timer_hdl);
            LOG_I("sirioff timer start");
        }
    }
    break;
#endif

    case BT_CONTROL_GET_PHONE_NUMBER:
    {
        bt_interface_get_ph_num();
    }
    break;

    case BT_CONTROL_GET_REMOTE_PHONE_NUMER:
    {
        bt_sifli_control_clcc_get();
    }
    break;

    case BT_CONTROL_SET_WBS_STATUS:
    {
        U8 status = *(U8 *)args;
        bt_interface_set_wbs_status(status);
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;

}









