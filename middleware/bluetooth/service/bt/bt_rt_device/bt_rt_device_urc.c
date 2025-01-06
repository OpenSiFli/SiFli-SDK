/**
  ******************************************************************************
  * @file   bt_rt_device_urc.c
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
//#include "utf8_unicode.h"
#include "bt_rt_device.h"
#include "bts2_global.h"
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#define DBG_TAG               "bt_rt_device.urc"
//#define DBG_LVL               DBG_INFO
#include <log.h>


void urc_func_inq_sifli(uint8_t *adrr, uint32_t nameSize, char *name, uint32_t dev_cls)
{
    bt_serached_device_info_t bt_device_info = {0};
    char bt_name[250];

    if (bt_sifli_check_bt_event(BT_SET_INQ_EVENT))
    {
        //bt_sifli_reset_bt_event(BT_SET_INQ_EVENT);
        memcpy(bt_device_info.mac_addr.addr, adrr, BT_MAX_MAC_LEN);

        /*bt_device_info.mac_addr.addr[0] = adrr[0]>>8;
        bt_device_info.mac_addr.addr[1] = adrr[0]&0xff;
        bt_device_info.mac_addr.addr[2] = adrr[1]>>8;
        bt_device_info.mac_addr.addr[3] = adrr[1]&0xff;
        bt_device_info.mac_addr.addr[4] = adrr[2]>>8;
        bt_device_info.mac_addr.addr[5] = adrr[2]&0xff;*/
        LOG_I("URC inq data : namesize:%d ", nameSize);

        LOG_I("URC inq data : adrr[0]:%x  addr[1]:%x addr[2]:%x adrr[3]:%x  addr[4]:%x addr[5]:%x "\
              , bt_device_info.mac_addr.addr[0], bt_device_info.mac_addr.addr[1], bt_device_info.mac_addr.addr[2]
              , bt_device_info.mac_addr.addr[3], bt_device_info.mac_addr.addr[4], bt_device_info.mac_addr.addr[5]);

        bt_device_info.name_size = nameSize;
        LOG_I("URC inq data 11: namesize:%d ", nameSize);
        memcpy(bt_name, name, nameSize);
        bt_device_info.bt_name = bt_name;
        bt_device_info.rssi = 0xff;//temp

        bt_notify_t args;
        args.event = BT_EVENT_INQ;
        args.args = &bt_device_info;
        rt_bt_event_notify(&args);
        LOG_I("URC inq data : rssi:%d  name_size:%d name:%s ", bt_device_info.rssi, bt_device_info.name_size, bt_device_info.bt_name);
    }
    return;
}

void urc_func_inq_finished_sifli(void)
{
    bt_notify_t args;
    args.event = BT_EVENT_INQ_FINISHED;
    args.args = NULL;
    bt_sifli_reset_bt_event(BT_SET_INQ_EVENT);
    rt_bt_event_notify(&args);
    return;
}


void urc_func_profile_conn_sifli(uint8_t *addr, uint8_t profile)
{
    bt_notify_t args;
    bt_connect_info_t info;
    rt_memcpy(info.mac.addr, addr, BT_MAX_MAC_LEN);
    info.profile = profile;
    info.conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)info.mac.addr);
    args.event = BT_EVENT_CONNECT_COMPLETE;
    args.args = &info;
    rt_bt_event_notify(&args);
    LOG_I("URC profile  %x conn idx:%d", profile, info.conn_idx);

    return;
}

void urc_func_profile_disc_sifli(uint8_t *addr, bt_profile_t profile, uint8_t reason)
{
    bt_notify_t args;
    bt_disconnect_info_t info;
    args.event = BT_EVENT_PROFILE_DISCONNECT;
    info.reason = reason;
    info.profile = profile;
    rt_memcpy(info.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    info.conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)info.peer_addr.addr);
    args.args = &info;
    rt_bt_event_notify(&args);

#ifdef BT_USING_AVRCP
    if ((BT_PROFILE_A2DP == profile) || (BT_PROFILE_MAX == profile))
    {
        urc_func_bt_avrcp_playback_status_sifli(0x01);
    }
#endif

    if (BT_PROFILE_HFP == profile)
    {
        bt_sifli_reset_bt_event(BT_SET_DIAL_EVENT | BT_SET_CLCC_EVENT);
    }

    LOG_I("URC profile disc data %x disc idx:%d, reason:%d", profile, info.conn_idx, reason);
    return;
}

void urc_func_disc_sifli(uint8_t *addr, uint8_t reason)
{
    bt_notify_t args;
    bt_acl_disconnect_info_t info;
    info.reason = reason;
    rt_memcpy(info.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    info.conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)info.peer_addr.addr);
    args.event = BT_EVENT_DISCONNECT;
    args.args = &info;
    rt_bt_event_notify(&args);
    LOG_I("URC disc ind:%d idx:%d", reason, info.conn_idx);
    return;
}

void urc_func_key_missing_sifli(uint8_t *addr)
{
    bt_notify_t args;
    bt_key_missing_info_t info;
    rt_memcpy(info.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    info.conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)info.peer_addr.addr);
    args.event = BT_EVENT_KEY_MISSING;
    args.args = &info;
    rt_bt_event_notify(&args);
    LOG_I("URC keymissing ind:%d", info.conn_idx);
    return;
}

void urc_func_encryption_sifli(uint8_t *addr)
{
    bt_notify_t args;
    bt_encryption_info_t info;
    rt_memcpy(info.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    info.conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)info.peer_addr.addr);
    args.event = BT_EVENT_ENCRYPTION;
    args.args = &info;
    rt_bt_event_notify(&args);
    LOG_I("URC encryption ind:%d", info.conn_idx);
    return;
}



void urc_func_call_link_ested_sifli(uint8_t res)
{
    bt_notify_t args;
    //LOG_I("sifli state%x", bt_state_sifli);
    args.event = BT_EVENT_CALL_lINK_ESTABLISHED;
    args.args = &res;
    rt_bt_event_notify(&args);
    //bt_profile_state_set(BT_PROFILE_HFP, BT_STATE_ON_CALL);
    LOG_I("URC call link established");
    return;
}

void urc_func_link_down_sifli(void)
{
    bt_notify_t args;
    //LOG_I("sifli state%x", bt_state_sifli);
    args.event = BT_EVENT_CALL_LINK_DOWN;
    args.args = RT_NULL;
    rt_bt_event_notify(&args);
    LOG_I("URC call link down");
    return;
}

void urc_func_bt_stack_ready_sifli(void)
{
    bt_notify_t args;
    args.event = BT_EVENT_BT_STACK_READY;
    args.args = RT_NULL;

    rt_bt_event_notify(&args);
    //LOG_I("URC BT STACK ready");
    return;
}

void urc_func_bt_addr_sifli(uint8_t *addr)
{
    bt_notify_t args;
    bt_mac_t    bt_addr;

    memcpy(&bt_addr, addr, BT_MAX_MAC_LEN);
    args.event = BT_EVENT_BT_ADDR_IND;
    args.args = &bt_addr;
    rt_bt_event_notify(&args);
    LOG_I("URC BT addr ind");
    return;
}

uint8_t urc_func_bt_close_complete_sifli(void)
{
    bt_notify_t args;
    uint8_t ret = 0;
    args.event = BT_EVENT_CLOSE_COMPLETE;
    args.args = RT_NULL;
    urc_func_bt_pairing_state(1);
    LOG_I("urc bt close:%d", bt_sifli_get_bt_event());
    if (bt_sifli_check_bt_event(BT_SET_DIS_GAP_EVENT | BT_SET_CANCEL_PAGE_EVENT))
    {
        //urc_func_disc_sifli(0x16);//0x16:local host terminate
        bt_interface_open_bt();
        ret = 1;
    }

    if (bt_sifli_check_bt_event(BT_SET_DIS_GAP_EVENT))
    {
        bt_sifli_reset_bt_event(BT_SET_DIS_GAP_EVENT | BT_SET_CLOSE_EVENT);
    }

    if (bt_sifli_check_bt_event(BT_SET_CANCEL_PAGE_EVENT))
    {
        bt_sifli_reset_bt_event(BT_SET_CANCEL_PAGE_EVENT | BT_SET_CLOSE_EVENT);
        args.event = BT_EVENT_CANCEL_PAGE_IND;
        rt_bt_event_notify(&args);
    }

    if (bt_sifli_check_bt_event(BT_SET_CLOSE_EVENT))
    {
        uint8_t open_bt = 0;
        bt_sifli_reset_bt_event(BT_SET_CLOSE_EVENT);
        args.event = BT_EVENT_CLOSE_COMPLETE;
        if (bt_sifli_check_bt_event(BT_SET_OPEN_EVENT))
        {
            open_bt = 1;
        }
        args.args = &open_bt;
        rt_bt_event_notify(&args);
    }

    return ret;
}

void urc_func_rd_local_name_sifli(BTS2S_DEV_NAME local_name)
{
    bt_notify_t args;
    args.event = BT_EVENT_RD_LOCAL_NAME;
    args.args = RT_NULL;

    if (bt_sifli_check_bt_event(BT_SET_RD_LOCAL_NAME_EVENT))
    {
        LOG_I("URC rd local name ind");
        bt_sifli_reset_bt_event(BT_SET_RD_LOCAL_NAME_EVENT);
        strcpy((char *)args.args, (const char *)local_name);
        rt_bt_event_notify(&args);
    }

    return;
}

void urc_func_rd_local_rssi_sifli(S8 rssi)
{
    bt_notify_t args;
    args.event = BT_EVENT_RD_LOCAL_RSSI;
    args.args = RT_NULL;

    if (bt_sifli_check_bt_event(BT_SET_RD_LOCAL_RSSI_EVENT))
    {
        LOG_I("URC rd local rssi ind:%d", rssi);
        bt_sifli_reset_bt_event(BT_SET_RD_LOCAL_RSSI_EVENT);
        args.args = &rssi;
        rt_bt_event_notify(&args);
    }

    return;
}

void urc_func_acl_opened_ind_sifli(uint8_t *addr, uint8_t res, uint8_t incoming, uint32_t dev_cls)
{
    bt_notify_t args;
    bt_acl_opened_t ind;

    ind.res = res;//0:success other_value:error code
    ind.incoming = incoming;
    rt_memcpy(ind.mac_addr.addr, addr, BT_MAX_MAC_LEN);
    ind.dev_cls = dev_cls;  //0x001f00 audio box
    ind.conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)ind.mac_addr.addr);
    args.event = BT_EVENT_ACL_OPENED_IND;
    args.args = &ind;
    if (HCI_SUCC != res)
    {
        urc_func_disc_sifli(addr, res);
    }
    rt_bt_event_notify(&args);
    LOG_I("URC BT acl opened ind:%d", ind.conn_idx);
    return;
}

void urc_func_pair_ind_sifli(uint8_t *addr, uint8_t result)
{
    bt_notify_t args;
    bt_pair_ind_t ind;
    ind.res = result;
    rt_memcpy(ind.mac_addr.addr, addr, BT_MAX_MAC_LEN);
    ind.conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)ind.mac_addr.addr);
    args.event = BT_EVENT_PAIR_IND;
    args.args = &ind;
    rt_bt_event_notify(&args);
    LOG_I("URC pair ind:%d idx:%d", result, ind.conn_idx);
    return;
}

void urc_func_change_bd_addr_sifli(uint32_t state)
{
    bt_notify_t args;
    args.event = BT_EVENT_CHANGE_BD_ADDR;
    args.args = &state;
    rt_bt_event_notify(&args);
    LOG_I("URC bd addr change state %d", state);
}

void urc_func_bt_pairing_state(uint8_t state)
{
    return;
}

void urc_func_rmt_version_inq(bt_rmt_version_t *version)
{
    bt_notify_t args;
    args.event = BT_EVENT_RMT_VERSION_IND;
    args.args = version;
    rt_bt_event_notify(&args);
}


#ifdef BT_USING_PAIRING_CONFIRMATION
void urc_func_io_capability_sifli(bt_mac_t *mac)
{
    bt_notify_t args;
    args.event = BT_EVENT_IO_CAPABILITY_IND;
    args.args = mac;
    rt_bt_event_notify(&args);
    LOG_I("URC IO capability bd:%02x:%02x:%02x:%02x:%02x:%02x",
          mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3], mac->addr[4], mac->addr[5]);
}

void urc_func_confirm_sifli(bt_pair_confirm_t *msg)
{
    bt_notify_t args;
    args.event = BT_EVENT_USER_CONFIRM_IND;
    args.args = msg;
    rt_bt_event_notify(&args);
    LOG_I("URC bt user confirm %d", msg->num_val);
}
#endif

void urc_func_key_overlaid_sifli(uint8_t *addr)
{
    bt_notify_t args;
    bt_key_missing_info_t info;
    rt_memcpy(info.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    info.conn_idx = bt_cm_find_conn_index_by_addr((uint8_t *)info.peer_addr.addr);
    args.event = BT_EVENT_KEY_OVERLAID;
    args.args = &info;
    rt_bt_event_notify(&args);
    LOG_I("URC key overlaidind:%d", info.conn_idx);
    return;
}

void urc_func_bt_remote_name_sifli(uint8_t *addr, uint8_t res, BTS2S_DEV_NAME *name)
{
    bt_notify_t args;
    bt_rmt_name_t rmt_name = {0};
    rt_memset(&rmt_name, 0, sizeof(bt_rmt_name_t));
    rt_memcpy(&rmt_name.addr, addr, BT_MAX_MAC_LEN);
    rmt_name.res = res;
    rt_memcpy(rmt_name.name, name, MAX_FRIENDLY_NAME_LEN);
    args.event = BT_EVENT_RMT_NAME;
    args.args = &rmt_name;
    rt_bt_event_notify(&args);
    LOG_I("URC rmt mac:%02x:%02x:%02x:%02x:%02x:%02x ret:%d name:%s", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], res, name);
    return;
}

static int bt_sifli_notify_common_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        urc_func_bt_stack_ready_sifli();
        break;
    }
    case BT_NOTIFY_COMMON_CLOSE_COMPLETE:
    {
        urc_func_bt_close_complete_sifli();
        break;
    }
    case BT_NOTIFY_COMMON_DISCOVER_IND:
    {
        bt_notify_remote_device_info_t *remote_info = (bt_notify_remote_device_info_t *)data;
        urc_func_inq_sifli(remote_info->mac.addr, remote_info->name_size, (char *)remote_info->bt_name, remote_info->dev_cls);
        break;
    }
    case BT_NOTIFY_COMMON_INQUIRY_CMP:
    {
        urc_func_inq_finished_sifli();
        break;
    }
    case BT_NOTIFY_COMMON_LOCAL_NAME_RSP:
    {
        BTS2S_DEV_NAME local_name;
        strcpy((char *)&local_name, (const char *)data);
        urc_func_rd_local_name_sifli(local_name);
        break;
    }
    case BT_NOTIFY_COMMON_REMOTE_NAME_RSP:
    {
        bt_notify_rmt_name_t *device_name = (bt_notify_rmt_name_t *)data;
        urc_func_bt_remote_name_sifli(device_name->mac.addr, device_name->res, (BTS2S_DEV_NAME *)device_name->bt_name);
        break;
    }
    case BT_NOTIFY_COMMON_LOCAL_RSSI_RSP:
    {
        int8_t rssi = data[0];
        urc_func_rd_local_rssi_sifli(rssi);
        break;
    }
    case BT_NOTIFY_COMMON_LOCAL_ADDR_RSP:
    {
        urc_func_bt_addr_sifli(data);
        break;
    }
    case BT_NOTIFY_COMMON_CHANGE_ADDR_RSP:
    {
        uint32_t *state = (uint32_t *)data;
        urc_func_change_bd_addr_sifli(*state);
        break;
    }
    case BT_NOTIFY_COMMON_RMT_VERSION_IND:
    {
        bt_rmt_version_t *version = (bt_rmt_version_t *)data;
        urc_func_rmt_version_inq(version);
        break;
    }
    case BT_NOTIFY_COMMON_ENCRYPTION:
    {
        urc_func_encryption_sifli(data);
        break;
    }
    case BT_NOTIFY_COMMON_KEY_MISSING:
    {
        urc_func_key_missing_sifli(data);
        break;
    }
    case BT_NOTIFY_COMMON_KEY_OVERLAID:
    {
        urc_func_key_overlaid_sifli(data);
        break;
    }
    case BT_NOTIFY_COMMON_PAIR_IND:
    {
        bt_notify_device_base_info_t *device_info = (bt_notify_device_base_info_t *)data;
        urc_func_pair_ind_sifli(device_info->mac.addr, device_info->res);
        break;
    }
    case BT_NOTIFY_COMMON_PAIR_STATE:
    {
        urc_func_bt_pairing_state(data[0]);
        break;
    }
    case BT_NOTIFY_COMMON_ACL_CONNECT_IND:
    {
        break;
    }
    case BT_NOTIFY_COMMON_ACL_CONNECTED:
    {
        bt_notify_device_acl_conn_info_t *acl_info = (bt_notify_device_acl_conn_info_t *) data;
        urc_func_acl_opened_ind_sifli(acl_info->mac.addr, acl_info->res, acl_info->acl_dir, acl_info->dev_cls);
        break;
    }
    case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
    {
        bt_notify_device_base_info_t *device_info = (bt_notify_device_base_info_t *)data;
        urc_func_disc_sifli(device_info->mac.addr, device_info->res);
        break;
    }
    case BT_NOTIFY_COMMON_SCO_CONNECTED:
    {
        bt_notify_device_sco_info_t *sco_info = (bt_notify_device_sco_info_t *)data;
        urc_func_call_link_ested_sifli(sco_info->sco_res);
        break;
    }
    case BT_NOTIFY_COMMON_SCO_DISCONNECTED:
    {
        urc_func_link_down_sifli();
        break;
    }
#ifdef BT_USING_PAIRING_CONFIRMATION
    case BT_NOTIFY_COMMON_USER_CONFIRM_IND:
    {
        bt_pair_confirm_t *info = (bt_pair_confirm_t *) data;
        urc_func_confirm_sifli(info);
        break;
    }
    case BT_NOTIFY_COMMON_IO_CAPABILITY_IND:
    {
        urc_func_io_capability_sifli(data);
        break;
    }
#endif
    default:
        return -1;
    }
    return 0;
}

static int bt_sifli_notify_pan_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_PAN_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_conn_sifli(profile_info->mac.addr, BT_PROFILE_PAN);
        break;
    }
    case BT_NOTIFY_PAN_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_disc_sifli(profile_info->mac.addr, BT_PROFILE_PAN, profile_info->res);
        break;
    }
    default:
        return -1;
    }
    return 0;
}

static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_COMMON:
    {
        ret = bt_sifli_notify_common_event_hdl(event_id, data, data_len);
    }
    break;

#ifdef BT_USING_HF
    case BT_NOTIFY_HFP_HF:
    {
        bt_sifli_notify_hfp_hf_event_hdl(event_id, data, data_len);
    }
    break;
#endif

#ifdef BT_USING_AG
    case BT_NOTIFY_HFP_AG:
    {
        bt_sifli_notify_hfp_ag_event_hdl(event_id, data, data_len);
    }
    break;
#endif

#ifdef BT_USING_A2DP
    case BT_NOTIFY_A2DP:
    {
        bt_sifli_notify_a2dp_event_hdl(event_id, data, data_len);
    }
    break;
#endif

#ifdef BT_USING_AVRCP
    case BT_NOTIFY_AVRCP:
    {
        bt_sifli_notify_avrcp_event_hdl(event_id, data, data_len);
    }
    break;
#endif

#ifdef BT_USING_HID
    case BT_NOTIFY_HID:
    {
        bt_sifli_notify_hid_event_hdl(event_id, data, data_len);
    }
    break;
#endif

#ifdef BT_USING_SPP
    case BT_NOTIFY_SPP:
    {
        bt_sifli_notify_spp_event_hdl(event_id, data, data_len);
    }
    break;
#endif

#ifdef BT_USING_GATT
    case BT_NOTIFY_GATT:
    {
        bt_sifli_notify_gatt_event_hdl(event_id, data, data_len);
    }
    break;
#endif

#ifdef BT_USING_PBAP
    case BT_NOTIFY_PBAP:
    {
        bt_sifli_notify_pbap_event_hdl(event_id, data, data_len);
    }
    break;
#endif
    case BT_NOTIFY_PAN:
    {
        bt_sifli_notify_pan_event_hdl(event_id, data, data_len);
    }
    break;

    default:
        break;
    }

    return 0;
}

int app_bt_notify_init(void)
{
    bt_interface_register_bt_event_notify_callback(bt_notify_handle);
    return BT_EOK;
}

INIT_ENV_EXPORT(app_bt_notify_init);
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/



