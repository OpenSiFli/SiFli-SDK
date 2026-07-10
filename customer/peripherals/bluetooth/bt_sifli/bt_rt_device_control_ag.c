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

#define DBG_TAG               "bt_rt_device.control_ag"
//#define DBG_LVL               DBG_INFO
#include <log.h>

bt_err_t bt_sifli_control_ag(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;
    switch (cmd)
    {
    case BT_CONTROL_AG_PHONE_CALL_STATUS_CHANGED:
    {
        hfp_call_info_t   *call_info;
        call_info = (hfp_call_info_t *)args;
        bt_interface_phone_state_changed((HFP_CALL_INFO_T *)call_info);
    }
    break;

    case BT_CONTROL_AG_LOCAL_PHONE_INFO_RES:
    {
        hfp_local_num_t *local_phone = (hfp_local_num_t *)args;
        bt_interface_local_phone_info_res(local_phone->profile_channel, (hfp_phone_number_t *)&local_phone->phone_num);
    }
    break;

    case BT_CONTROL_AG_REMOTE_CALL_INFO_RES:
    {
        hfp_local_call_info_t *calls_info = (hfp_local_call_info_t *)args;
        bt_interface_remote_call_info_res(calls_info->profile_channel, (hfp_remote_calls_info_t *)&calls_info->call_info);
    }
    break;

    case BT_CONTROL_AG_ALL_INDICATOR_INFO_RES:
    {
        hfp_indicator_info_t *all_ind_info = (hfp_indicator_info_t *)args;
        bt_interface_get_all_indicator_info_res(all_ind_info->profile_channel, (hfp_cind_status_t *)&all_ind_info->cind_state);
    }
    break;

    case BT_CONTROL_AG_INDICATOR_CHANGED_RES:
    {
        hfp_indicator_status_t *ind_info = (hfp_indicator_status_t *)args;
        bt_interface_indicator_status_changed(ind_info->profile_channel, (HFP_IND_INFO_T *)&ind_info->ind_info);
    }
    break;

    case BT_CONTROL_AG_SPK_VOL_CHANGE_REQ:
    {
        U8 vol = *(U8 *)args;
        bt_interface_spk_vol_change_req(0, vol);
    }
    break;

    case BT_CONTROL_AG_MIC_VOL_CHANGE_REQ:
    {
        U8 vol = *(U8 *)args;
        bt_interface_mic_vol_change_req(0, vol);
    }
    break;

    case BT_CONTROL_AG_CMD_RESULT_RES:
    {
        U8 res = *(U8 *)args;
        bt_interface_make_call_res(0, res);
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;

}


