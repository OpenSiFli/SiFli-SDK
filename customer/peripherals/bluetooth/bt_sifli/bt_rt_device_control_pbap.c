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
        ret = bt_pbap_client_pull_pb((BTS2E_PBAP_PHONE_REPOSITORY)info->repos, info->phone_book, info->max_size);
        break;
    }
    case BT_CONTROL_PBAP_SET_PB:
    {
        bt_pbap_pb_set_t *info = (bt_pbap_pb_set_t *)args;
        ret = bt_pbap_client_set_pb((BTS2E_PBAP_PHONE_REPOSITORY)info->repos, info->phone_book);
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
        ret = bt_pbap_client_get_name_by_number((char *)p_args->number, (U16)p_args->size);
        break;
    }

    case BT_CONTROL_PBAP_AUTH_RSP:
    {
        bt_pbap_auth_info *auth_info = (bt_pbap_auth_info *)args;
        ret = bt_pbap_client_auth(auth_info->password, auth_info->password_len);
        break;
    }

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

