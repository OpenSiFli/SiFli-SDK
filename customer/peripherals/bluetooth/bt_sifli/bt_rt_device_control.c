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
#include "bt_rt_device_control_common.h"
#ifdef BT_USING_HF
    #include "bt_rt_device_control_hf.h"
#endif
#ifdef BT_USING_A2DP
    #include "bt_rt_device_control_a2dp.h"
#endif
#ifdef BT_USING_AVRCP
    #include "bt_rt_device_control_avrcp.h"
#endif
#ifdef BT_USING_AG
    #include "bt_rt_device_control_ag.h"
#endif
#ifdef BT_USING_HID
    #include "bt_rt_device_control_hid.h"
#endif
#ifdef BT_USING_SPP
    #include "bt_rt_device_control_spp.h"
#endif
#ifdef BT_USING_PBAP
    #include "bt_rt_device_control_pbap.h"
#endif
#ifdef BT_USING_GATT
    #include "bt_rt_device_control_gatt.h"
#endif

#define DBG_TAG               "bt_rt_device.control"
//#define DBG_LVL               DBG_INFO
#include <log.h>

volatile uint32_t bt_set_event = 0x00000000;

U8 bt_sifli_check_bt_event(uint32_t event)
{
    if (bt_set_event & event)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

U8 bt_sifli_set_bt_event(uint32_t event)
{
    if (bt_sifli_check_bt_event(event))
    {
        return 0;
    }
    else
    {
        bt_set_event |= event;
        return 1;
    }
}

U8 bt_sifli_reset_bt_event(uint32_t event)
{
    if (bt_sifli_check_bt_event(event))
    {
        bt_set_event &= (~event);
    }
    return 1;
}

uint32_t bt_sifli_get_bt_event()
{
    return bt_set_event;
}

bt_err_t bt_sifli_control(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;
    switch (cmd >> 8)
    {
    case BT_COMMON_TYPE_ID:
    {
        ret = bt_sifli_control_common(bt_handle, cmd, args);
    }
    break;

#ifdef BT_USING_HF
    case BT_HF_TYPE_ID:
    {
        ret = bt_sifli_control_hf(bt_handle, cmd, args);
    }
    break;
#endif

#ifdef BT_USING_AG
    case BT_AG_TYPE_ID:
    {
        ret = bt_sifli_control_ag(bt_handle, cmd, args);
    }
    break;
#endif

#ifdef BT_USING_A2DP
    case BT_A2DP_TYPE_ID:
    {
        ret = bt_sifli_control_a2dp(bt_handle, cmd, args);
    }
    break;
#endif

#ifdef BT_USING_AVRCP
    case BT_AVRCP_TYPE_ID:
    {
        ret = bt_sifli_control_avrcp(bt_handle, cmd, args);
    }
    break;
#endif

#ifdef BT_USING_HID
    case BT_HID_TYPE_ID:
    {
        ret = bt_sifli_control_hid(bt_handle, cmd, args);
    }
    break;
#endif

#ifdef BT_USING_SPP
    case BT_SPP_TYPE_ID:
    {
        ret = bt_sifli_control_spp(bt_handle, cmd, args);
    }
    break;
#endif

#ifdef BT_USING_GATT
    case BT_GATT_TYPE_ID:
    {
        ret = bt_sifli_control_gatt(bt_handle, cmd, args);
    }
    break;
#endif

#ifdef BT_USING_PBAP
    case BT_PBAP_TYPE_ID:
    {
        ret = bt_sifli_control_pbap(bt_handle, cmd, args);
    }
    break;
#endif
    default:
        break;
    }
    return ret;
}







