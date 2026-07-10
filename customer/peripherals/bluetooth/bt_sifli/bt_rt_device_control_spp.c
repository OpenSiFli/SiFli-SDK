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

#define DBG_TAG               "bt_rt_device.control_spp"
//#define DBG_LVL               DBG_INFO
#include <log.h>

bt_err_t bt_sifli_control_spp(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
#ifdef CFG_SPP_SRV
    case BT_CONTROL_SEND_SPP_DATA:
    {
        spp_data_t   *p_args;

        p_args = (spp_data_t *)args;
        bt_mac_t *mac_addr = &p_args->mac_addr;
        ret = bt_interface_spp_send_data_ext(p_args->data, p_args->len, (bt_notify_device_mac_t *)mac_addr, p_args->srv_chl);
    }
    break;

    case BT_CONTROL_SEND_SPP_RSP:
    {
        spp_common_t *p_args;

        p_args = (spp_common_t *)args;

        bt_mac_t *mac_addr = &p_args->mac_addr;
        ret = bt_interface_spp_srv_data_rsp_ext((bt_notify_device_mac_t *)mac_addr, p_args->srv_chl);
    }
    break;

    case BT_CONTROL_SEND_SPP_DISC_REQ:
    {
        spp_common_t *p_args;

        p_args = (spp_common_t *)args;

        bt_mac_t *mac_addr = &p_args->mac_addr;
        ret = bt_interface_dis_spp_by_addr_and_chl_ext((bt_notify_device_mac_t *)mac_addr, p_args->srv_chl);
    }
    break;
#endif

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

