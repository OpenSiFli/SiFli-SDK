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

#define DBG_TAG               "bt_rt_device.urc_pbap"
//#define DBG_LVL               DBG_INFO
#include <log.h>


static void urc_func_pbap_vcard_list_notify(pbap_vcard_listing_item_t *msg)
{
    bt_notify_t args;
    args.event = BT_EVENT_VCARD_LIST_ITEM_NOTIFY;
    args.args = msg;
    rt_bt_event_notify(&args);
    LOG_I("URC bt vcard_list notify %s len:%d", msg->vcard_name, msg->name_len);
}

static void urc_func_pbap_vcard_list_cmp(uint8_t res)
{
    bt_notify_t args;
    args.event = BT_EVENT_VCARD_LIST_CMP;
    args.args = &res;
    rt_bt_event_notify(&args);
    LOG_I("URC bt vcard_list notify cmp %d", res);
}

static void urc_func_pbap_pull_vcard_cmp(uint8_t res)
{
    bt_notify_t args;
    args.event = BT_EVENT_PULL_VCARD_CMP;
    args.args = &res;
    rt_bt_event_notify(&args);
    LOG_I("URC bt pull vcard cmp %d", res);
}

static void urc_func_pbap_pull_pb_cmp(bt_pbap_vcard_item_cmpl_t *msg)
{
    bt_notify_t args;
    args.event = BT_EVENT_PULL_PB_CMP;
    args.args = msg;
    rt_bt_event_notify(&args);
    LOG_I("URC bt pull pb (%d) cmp result:%d", msg->phone_book, msg->res);
}

static void urc_func_pbap_set_path_cfm(bt_pbap_vcard_item_cmpl_t *msg)
{
    bt_notify_t args;
    args.event = BT_EVENT_SET_PATH_CFM;
    args.args = msg;
    rt_bt_event_notify(&args);
    LOG_I("URC bt set path(%d) cfm %d", msg->phone_book, msg->res);
}

static void urc_func_pbap_vcard_item_notify(bt_pbap_vcard_item_t *msg)
{
    bt_notify_t args;
    args.event = BT_EVENT_VCARD_ITEM_NOTIFY;
    args.args = msg;
    rt_bt_event_notify(&args);
    // LOG_I("URC bt vcard item name %s number:%s time:%s", msg->vcard_name, msg->vcard_number, msg->vcard_time);
}

int bt_sifli_notify_pbap_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_PBAP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_conn_sifli(profile_info, BT_PROFILE_PBAP);
        break;
    }
    case BT_NOTIFY_PBAP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_disc_sifli(profile_info->mac.addr, BT_PROFILE_PBAP, profile_info->res);
        break;
    }
    case BT_NOTIFY_PBAP_VCARD_LIST_ITEM_IND:
    {
        pbap_vcard_listing_item_t *list_item = (pbap_vcard_listing_item_t *)data;
        urc_func_pbap_vcard_list_notify(list_item);
        break;
    }
    case BT_NOTIFY_PBAP_VCARD_LIST_CMPL:
    {
        urc_func_pbap_vcard_list_cmp(data[0]);
        break;
    }
    case BT_NOTIFY_PBAP_PULL_VCARD_CMPL:
    {
        urc_func_pbap_pull_vcard_cmp(data[0]);
        break;
    }
    ///  PBAP profile pull phone book complete event
    case BT_NOTIFY_PBAP_PULL_PB_CMPL:
    {
        urc_func_pbap_pull_pb_cmp((bt_pbap_vcard_item_cmpl_t *)data);
        break;
    }
    ///  PBAP profile set path cfm
    case BT_NOTIFY_PBAP_SET_PATH_CFM:
    {
        urc_func_pbap_set_path_cfm((bt_pbap_vcard_item_cmpl_t *)data);
        break;
    }
    case BT_NOTIFY_PBAP_VCARD_ITEM_IND:
    {
        bt_pbap_vcard_item_t *vcard_item = (bt_pbap_vcard_item_t *)data;
        urc_func_pbap_vcard_item_notify(vcard_item);
        break;
    }
    case BT_NOTIFY_PBAP_VCARD_TOTAL_NUM_IND:
    {
        bt_notify_pbap_vcard_num_check_t *vcard_num_check = (bt_notify_pbap_vcard_num_check_t *)data;
        bt_notify_t args;
        args.event = BT_EVENT_VCARD_TOTAL_NUM_IND;
        args.args = vcard_num_check;
        rt_bt_event_notify(&args);
        LOG_I("URC bt phone book %d total num %d", vcard_num_check->phone_book, vcard_num_check->total_num);
        break;
    }
    default:
        return -1;
    }
    return 0;
}

