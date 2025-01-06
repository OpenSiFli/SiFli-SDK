/**
  ******************************************************************************
  * @file   bt_rt_device_control.c
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
#ifdef BT_USING_AVRCP
    #include "bt_rt_device_control_avrcp.h"
#endif

#define DBG_TAG               "bt_rt_device.control_comm"
//#define DBG_LVL               DBG_INFO
#include <log.h>


static bd_addr_t g_custom_bt_addr = {0};

extern void set_speaker_volume(uint8_t volume);
static bt_err_t bt_sifli_set_speaker_volume(rt_bt_device_t *dev, bt_volume_set_t *set)
{
    bt_err_t ret = BT_ERROR_STATE;
    int volume;
    if (set->volume.call_volume > 15)
    {
        volume = 15;
    }
    else
    {
        volume =  set->volume.call_volume;
    }
#ifdef BT_CONNECT_SUPPORT_MULTI_LINK
    if (!bt_sifli_check_bt_event(BT_SET_VGS_EVENT))
#else
    if (!(bt_sifli_check_bt_event(BT_SET_VGS_EVENT)) && BT_STATE_CONNECTED == rt_bt_get_connect_state(dev, BT_PROFILE_HFP))
#endif
    {
        bt_sifli_set_bt_event(BT_SET_VGS_EVENT);
        bt_interface_set_speaker_volume(volume);
        ret = BT_EOK;
    }

    set_speaker_volume(volume);
    return ret;
}

static bt_err_t bt_sifli_control_close(int cmd)
{
    bt_err_t ret = BT_EOK;
    //gap_wr_scan_enb_req(bts2_task_get_app_task_id(), FALSE, FALSE);
    bt_sifli_set_bt_event(BT_SET_CLOSE_EVENT);
    if (BT_CONTROL_DISCONNECT == cmd)
    {
        bt_sifli_set_bt_event(BT_SET_DIS_GAP_EVENT);
        ret = BT_ERROR_IN_PROGRESS;
    }

    if (BT_CONTROL_CANCEL_PAGE == cmd)
    {
        bt_sifli_set_bt_event(BT_SET_CANCEL_PAGE_EVENT);
        ret = BT_ERROR_IN_PROGRESS;
    }

    bt_interface_close_bt();
    //shutdown stop waiting paring complete to avoid timeout erro
    //bt_sifli_wait_pairing_complete();
    LOG_I("BT_CONTROL_CLOSE_DEVICE cmd:%x", cmd);
    return ret;
}

static bt_err_t bt_sifli_control_open(void)
{
    bt_err_t ret = BT_EOK;

    bt_sifli_set_bt_event(BT_SET_OPEN_EVENT);
    if (bt_sifli_check_bt_event(BT_SET_CLOSE_EVENT))
    {
        LOG_I("BT is not closed and needs to wait");
        ret = BT_ERROR_STATE;
    }
    else
    {
        bt_interface_open_bt();
        bt_sifli_reset_bt_event(BT_SET_OPEN_EVENT);
        LOG_I("BT_CONTROL_OPEN_DEVICE");
    }
    return ret;
}

bt_err_t bt_sifli_control_common(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case BT_CONTROL_CLOSE_DEVICE:
    {
        bt_cm_disconnect_req();
        bt_sifli_reset_bt_event(BT_SET_OPEN_EVENT | BT_SET_DIS_GAP_EVENT | BT_SET_CANCEL_PAGE_EVENT);
        ret = bt_sifli_control_close(cmd);
    }
    break;

    case BT_CONTROL_OPEN_DEVICE:
    {
        ret = bt_sifli_control_open();
    }
    break;

    case BT_CONTROL_SET_SCAN:
    {
        if (app_bt_get_non_signaling_test_status())
            return BT_ERROR_STATE;
        bt_scan_con_t *scan = (bt_scan_con_t *)args;
        gap_wr_scan_enb_req(bts2_task_get_app_task_id(), scan->inquiry_scan, scan->page_scan);
    }
    break;

    case BT_CONTROL_GET_SCAN:
    {
        uint8_t *p_args = (uint8_t *)args;
        *p_args = bt_interface_get_current_scan_mode();
    }
    break;

    case BT_CONTROL_EXIT_SNIFF:
    {
        bt_interface_exit_sniff_mode();
    }
    break;

    case BT_CONTROL_DEVICE_INIT:
    {
    }
    break;

    case BT_CONTROL_DEVICE_DEINIT:
    {
        /*TODO*/
    }
    break;

    case BT_CONTROL_SWITCH_TO_SOURCE:
    case BT_CONTROL_SWITCH_TO_SINK:

        break;

    case BT_CONTROL_SEARCH_EQUIPMENT:
    {
        bt_sifli_set_bt_event(BT_SET_INQ_EVENT);
        bt_interface_start_inquiry();
        LOG_D("BT_CONTROL_SEARCH_EQUIPMENT\n");
    }
    break;

    case BT_CONTROL_CANCEL_SEARCH:
    {
        bt_sifli_reset_bt_event(BT_SET_INQ_EVENT);
        bt_interface_stop_inquiry();
        LOG_D("BT_CONTROL_CANCEL_SEARCH\n");
    }
    break;

    case BT_CONTROL_CONNECT_DEVICE:
    {
        bt_mac_t *mac = (bt_mac_t *)args;

        if (BT_ROLE_MASTER == bt_handle->role)
        {
            /*@TODO*/
        }
        else
        {
            ret = bt_interface_hfp_hf_start_connecting((unsigned char *)(mac->addr));
        }
    }
    break;

    case BT_CONTROL_CONNECT_DEVICE_EX:
    {
        bt_connect_info_t *info = (bt_connect_info_t *)args;
        bt_mac_t *mac = &(info->mac);
        gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
        bt_cm_conn_info_t *conn = bt_cm_find_bonded_dev_by_addr((uint8_t *)&info->mac);
        LOG_D("[%s] conn:%p role:%d", __func__, conn, conn->role);
        if (conn)
        {
            if (BT_CM_MASTER == conn->role)
                ret =  bt_interface_conn_to_source_ext((unsigned char *)(mac->addr), info->profile);
            else
                ret = bt_interface_conn_ext((unsigned char *)(mac->addr), info->profile);
        }
        else
        {
            if (BT_ROLE_MASTER == bt_handle->role)
                ret =  bt_interface_conn_to_source_ext((unsigned char *)(mac->addr), info->profile);
            else
                ret = bt_interface_conn_ext((unsigned char *)(mac->addr), info->profile);
        }
        info->conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)info->mac.addr);
    }
    break;

    case BT_CONTROL_DISCONNECT:
    {
        ret = bt_sifli_control_close(cmd);
    }
    break;

    case BT_CONTROL_DISCONNECT_EX:
    {
        bt_profile_t *profile = (bt_profile_t *)args;
        ret = bt_interface_disc_ext(NULL, *profile);
    }
    break;

    case BT_CONTROL_QUERY_STATE_EX:

        break;

    case BT_CONTROL_QUERY_STATE_NONBLOCK:
    {
        //ret = fr508x_query_state_nonblock();
    }
    break;

    case BT_CONTROL_AUDIO_TRANSFER_EX:
    {
        bt_hfp_audio_switch_t *set = (bt_hfp_audio_switch_t *)args;
        LOG_I("role 0x%x 0x%0x flag:%d", bt_handle->role, BT_ROLE_MASTER, set->type);
#ifdef BT_USING_AG
        if (BT_ROLE_MASTER == bt_handle->role)
        {
            BTS2S_BD_ADDR dest_addr;
            bt_addr_convert_to_bts((bd_addr_t *)set->peer_addr.addr, &dest_addr);
            if (set->type)
                bt_hfp_disconnect_audio(&dest_addr);
            else
                bt_hfp_connect_audio(&dest_addr);
        }
#endif
    }
    break;

    case BT_CONTROL_SET_VOLUME:
    {
        bt_volume_set_t *set = (bt_volume_set_t *)args;
        if (BT_VOLUME_CALL == set->mode)
        {
            ret = bt_sifli_set_speaker_volume(bt_handle, set);
        }
        else if (BT_VOLUME_MEDIA == set->mode)
        {
#ifdef BT_USING_AVRCP
            ret = bt_sifli_set_avrcp_volume(bt_handle, set);
#else
            ret = BT_ERROR_STATE;
#endif
        }
    }
    break;

    case BT_CONTROL_SET_LOCAL_NAME:
    {
        set_name_t   *p_args;
        p_args = (set_name_t *)args;
        bt_interface_set_local_name(p_args->size, (void *)(p_args->name));
    }
    break;

    case BT_CONTROL_RD_LOCAL_NAME:
    {
        bt_sifli_set_bt_event(BT_SET_RD_LOCAL_NAME_EVENT);
        bt_interface_rd_local_name();
    }
    break;

    case BT_CONTROL_RD_LOCAL_RSSI:
    {
        bt_mac_t *mac = (bt_mac_t *)args;
        bt_sifli_set_bt_event(BT_SET_RD_LOCAL_RSSI_EVENT);
        bt_interface_rd_local_rssi((unsigned char *)(mac->addr));
    }
    break;

    case BT_CONTROL_GET_BT_MAC:
    {
        bt_mac_t *p_args = (bt_mac_t *)args;
        bd_addr_t addr = {0};
        if (rt_memcmp(&g_custom_bt_addr, &addr, sizeof(bd_addr_t)))
        {
            rt_memcpy(p_args->addr, g_custom_bt_addr.addr, BT_MAX_MAC_LEN);
            return ret;
        }
        ret = ble_get_public_address(&addr);
        if (ret == 0)
        {
            rt_memcpy(p_args->addr, addr.addr, BT_MAX_MAC_LEN);
        }
        else
        {
            ret = BT_ERROR_RESP_FAIL;
        }
    }
    break;
#if defined(BT_USING_MIC_MUTE) && defined(AUDIO_USING_MANAGER)
    case BT_CONTROL_SET_MIC_MUTE:
    {
        bt_mic_mute_t *p_args = (bt_mic_mute_t *)args;
        audio_server_set_public_mic_mute(*p_args);
    }
    break;
    case BT_CONTROL_GET_MIC_MUTE:
    {
        bt_mic_mute_t *p_args = (bt_mic_mute_t *)args;
        *p_args = audio_server_get_public_mic_mute();
    }
    break;
#endif

    case BT_CONTROL_CHANGE_BD_ADDR:
    {
        if (RT_NULL == args)
        {
            bd_addr_t addr = {0};
            ble_get_public_address(&addr);
            sibles_change_bd_addr(SIBLES_CH_BD_TYPE_BT, SIBLES_CH_BD_METHOD_CUSTOMIZE, &addr);
#ifdef BT_FINSH_PAN
            BTS2S_BD_ADDR     bd_addr;
            bt_addr_convert_to_bts(&addr, &bd_addr);
            bt_interface_update_pan_addr(&bd_addr);
#endif
        }
        else
        {
            rt_memcpy(&g_custom_bt_addr, args, sizeof(bd_addr_t));
            sibles_change_bd_addr(SIBLES_CH_BD_TYPE_BT, SIBLES_CH_BD_METHOD_CUSTOMIZE, args);
#ifdef BT_FINSH_PAN
            BTS2S_BD_ADDR     bd_addr;
            bt_addr_convert_to_bts(&g_custom_bt_addr, &bd_addr);
            bt_interface_update_pan_addr(&bd_addr);
#endif
        }
    }
    break;

    case BT_CONTROL_CANCEL_PAGE:
    {
        ret = bt_sifli_control_close(cmd);
    }
    break;

    case BT_CONTROL_CANCEL_PAGE_BY_ADDR:
    {
        BTS2S_BD_ADDR addr = {0};
        bt_mac_t *mac = (bt_mac_t *)args;
        bt_addr_convert_to_bts((bd_addr_t *)mac, &addr);
        gap_cancel_connect_req(&addr);
    }
    break;

    case BT_CONTROL_GET_RMT_VERSION:
    {
        BTS2S_BD_ADDR bd_addr;
        bt_mac_t *mac_addr = (bt_mac_t *)args;
        bt_addr_convert_to_bts((bd_addr_t *)mac_addr, &bd_addr);
        gap_rd_rmt_version_req(bts2_task_get_app_task_id(), bd_addr);
    }
    break;

#ifdef BT_USING_PAIRING_CONFIRMATION
    case BT_CONTROL_IO_CAPABILITY_RES:
    {
        bt_io_capability_rsp_t *res = (bt_io_capability_rsp_t *)args;
        BTS2S_BD_ADDR bd_addr;
        bt_addr_convert_to_bts((bd_addr_t *)&res->mac, &bd_addr);
        sc_io_capability_rsp(&bd_addr, (BTS2E_SC_IO_CAPABILITY)res->io_capability, res->mitm, res->bonding);
    }
    break;

    case BT_CONTROL_USER_CONFIRM_RES:
    {
        bt_user_confirm_rsp_t *res = (bt_user_confirm_rsp_t *)args;
        BTS2S_BD_ADDR bd_addr;
        bt_addr_convert_to_bts((bd_addr_t *)&res->mac, &bd_addr);
        sc_user_cfm_rsp(&bd_addr, res->confirm);
        if (!res->confirm)
        {
            ret = bt_sifli_control_close(BT_CONTROL_DISCONNECT);
        }
    }
    break;
#endif

    case BT_CONTROL_SWITCH_ON:
    {
        if (app_bt_get_non_signaling_test_status())
            return BT_ERROR_STATE;
        gap_wr_scan_enb_req(bts2_task_get_app_task_id(), TRUE, TRUE);
    }
    break;

    case BT_CONTROL_SWITCH_OFF:
    {
        if (app_bt_get_non_signaling_test_status())
            return BT_ERROR_STATE;
        gap_wr_scan_enb_req(bts2_task_get_app_task_id(), FALSE, FALSE);
    }
    break;

    case BT_CONTROL_DISCONNECT_BY_CONNIDX:
    {
        BTS2S_BD_ADDR addr = {0};
        uint8_t *conn_idx = (uint8_t *)args;
        bt_cm_find_addr_by_conn_index(*conn_idx, &addr);

        gap_disconnect_req(&addr);
    }
    break;

    case BT_CONTROL_GET_RMT_NAME:
    {
        bt_mac_t *mac = (bt_mac_t *)args;
        BTS2S_BD_ADDR addr;
        bt_addr_convert_to_bts((bd_addr_t *)mac, &addr);
        gap_rd_rmt_name_req(bts2_task_get_app_task_id(), addr);
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/







