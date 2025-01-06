/**
  ******************************************************************************
  * @file   bt_rt_device_control_pbap.c
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

#define DBG_TAG               "bt_rt_device.control_pbap"
//#define DBG_LVL               DBG_INFO
#include <log.h>

bt_err_t bt_sifli_control_pbap(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case BT_CONTROL_PBAP_PULL_PB:
    {
        bt_pbap_pb_info *info = (bt_pbap_pb_info *)args;
        bt_pbap_client_pull_pb((BTS2E_PBAP_PHONE_REPOSITORY)info->repos, info->phone_book, info->max_size);
        break;
    }
    case BT_CONTROL_PBAP_SET_PB:
    {
        bt_pbap_pb_set_t *info = (bt_pbap_pb_set_t *)args;
        bt_pbap_client_set_pb((BTS2E_PBAP_PHONE_REPOSITORY)info->repos, info->phone_book);
        break;
    }

    case BT_CONTROL_PBAP_PULL_VCARD_LIST:
    {
        // bt_pbap_client_pull_vcard_list(NULL); to do
        break;
    }

    case BT_CONTROL_PBAP_PULL_VCARD_ENTRY:
    {
        //bt_pbap_client_pull_vcard(1,1); to do
        break;
    }

    case BT_CONTROL_PBAP_GET_NAME_BY_NUMBER:
    {
        phone_number_t   *p_args;
        p_args = (phone_number_t *)args;
        bt_pbap_client_get_name_by_number((char *)p_args->number, (U16)p_args->size);
        break;
    }

    case BT_CONTROL_PBAP_AUTH_RSP:
    {
        bt_pbap_auth_info *auth_info = (bt_pbap_auth_info *)args;
        bt_pbap_client_auth(auth_info->password, auth_info->password_len);
        break;
    }

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

