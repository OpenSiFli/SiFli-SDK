/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "bt_profile_channel_fsm"
//#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

void bt_profile_channel_fsm_handle(rt_bt_device_t *dev, int event_type, void *args)
{
    switch (event_type)
    {
    case BT_EVENT_PROFILE_DISCONNECT:
    {
        bt_disconnect_info_t *info = (bt_disconnect_info_t *)args;
        if (0xFF != info->conn_idx)
        {
            dev->fsm.profile_channel[info->conn_idx][info->profile] = 0;
        }
    }
    break;

    case BT_EVENT_CLOSE_COMPLETE:
    {
        for (uint8_t index = 0; index < BT_MAX_ACL_NUM; index++)
        {
            for (uint8_t i = 0; i < BT_PROFILE_MAX; i++)
            {
                dev->fsm.profile_channel[index][i] = 0;
            }
        }
    }
    break;

    case BT_EVENT_DISCONNECT:
    {
        bt_acl_disconnect_info_t *info = (bt_acl_disconnect_info_t *)args;
        if (0xFF != info->conn_idx)
        {
            for (uint8_t i = 0; i < BT_PROFILE_MAX; i++)
            {
                dev->fsm.profile_channel[info->conn_idx][i] = 0;
            }
        }
    }
    break;

    case BT_EVENT_CONNECT_COMPLETE:
    {
        bt_connect_info_t *info = (bt_connect_info_t *)args;
        if (0xFF != info->conn_idx)
        {
            dev->fsm.profile_channel[info->conn_idx][info->profile] = info->profile_channel;
        }
    }
    break;

    default:
        break;
    }
}

int bt_profile_channel_fsm_init(void)
{
    rt_bt_device_t *bt_device = (rt_bt_device_t *) rt_device_find(BT_DEVICE_NAME);
    if (RT_NULL == bt_device)
    {
        LOG_E("init bt profile channel fsm fail");
        return RT_ERROR;
    }

    for (uint8_t index = 0; index < BT_MAX_ACL_NUM; index++)
    {
        for (uint8_t profile = 0; profile < BT_PROFILE_MAX; profile++)
        {
            bt_device->fsm.profile_channel[index][profile] = 0;
        }
    }
    return BT_EOK;
}
