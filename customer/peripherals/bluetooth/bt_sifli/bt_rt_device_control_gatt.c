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

#define DBG_TAG               "bt_rt_device.control_gatt"
//#define DBG_LVL               DBG_INFO
#include <log.h>

bt_err_t bt_sifli_control_gatt(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case BT_CONTROL_BT_GATT_SDP_REG_REQ:
    {
        br_att_sdp_data_t *sdp_info;
        sdp_info = (br_att_sdp_data_t *)args;
        bt_interface_bt_gatt_reg(sdp_info);
    }
    break;

    case BT_CONTROL_BT_GATT_SDP_UNREG_REQ:
    {
        U32 sdp_hdl = *(U32 *)args;
        bt_interface_bt_gatt_unreg(sdp_hdl);
    }
    break;

    case BT_CONTROL_BT_GATT_MTU_CHANGE_REQ:
    {
        U16 mtu = *(U16 *)args;
        bt_interface_bt_gatt_mtu_changed(mtu);
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

