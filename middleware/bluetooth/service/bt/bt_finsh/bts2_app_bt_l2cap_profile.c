/**
  ******************************************************************************
  * @file   bts2_app_bt_l2cap_profile.c
  * @author Sifli software development team
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
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

#include "bts2_global.h"
#ifdef CFG_BT_L2CAP_PROFILE
#include "bts2_app_inc.h"
#include "bts2_app_bt_l2cap_profile.h"
#include "log.h"
#ifdef AUDIO_USING_MANAGER
    #include "audio_server.h"
#endif

static bts2_bt_l2cap_inst_data_t *bt_l2cap_profile_app_get_context()
{
    bts2_app_stru *bts2_app_data = getApp();
    return &bts2_app_data->bt_l2cap_profile_inst;
}

/*************************************BT_L2CAP device_manager start ************************************/
bts2_bt_l2cap_device_info_inst *bt_l2cap_profile_app_get_empty_device_inst()
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    bts2_bt_l2cap_device_info_inst *device_info = l2cap_profile_info->device;

    for (int idx = 0; idx < CFG_MAX_ACL_CONN_NUM; idx++)
    {
        if (!device_info[idx].is_use)
        {
            return &device_info[idx];
        }
    }
    return NULL;
}

bts2_bt_l2cap_device_info_inst *bt_l2cap_profile_app_get_device_inst_by_cid(U16 cid)
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    bts2_bt_l2cap_device_info_inst *device_info = l2cap_profile_info->device;

    for (int idx = 0; idx < CFG_MAX_ACL_CONN_NUM; idx++)
    {
        if (device_info[idx].cid == cid)
        {
            return &device_info[idx];
        }
    }
    return NULL;
}

bts2_bt_l2cap_device_info_inst *bt_l2cap_profile_app_get_device_inst_by_addr(BTS2S_BD_ADDR *bd)
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    bts2_bt_l2cap_device_info_inst *device_info = l2cap_profile_info->device;

    for (int idx = 0; idx < CFG_MAX_ACL_CONN_NUM; idx++)
    {
        if (device_info[idx].is_use && (bd_eq(&device_info[idx].bd, bd) == TRUE))
        {
            return &device_info[idx];
        }
    }
    return NULL;
}

uint8_t bt_l2cap_profile_get_cur_sco_num()
{
    uint8_t num = 0;
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    bts2_bt_l2cap_device_info_inst *device_info = l2cap_profile_info->device;

    for (int idx = 0; idx < CFG_MAX_ACL_CONN_NUM; idx++)
    {
        if (device_info[idx].is_use && (device_info[idx].sco_hdl != 0))
        {
            num++;
        }
    }
    return num;
}

/*************************************BT_L2CAP device_manager end *************************************/

static void bt_l2cap_profile_app_recv_data(U16 cid, U16 payload_len, char *payload)
{

}

static void bt_l2cap_profile_app_send_data_status(U16 cid, U8 res)
{

}

static void bt_l2cap_profile_app_update_reg_state(BTS2S_L2CAP_PROFILE_REG_RES *status_info)
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();

    LOG_I("bt_l2cap_profile_app_update_reg_state status_info 0x%2x\n", status_info->reg_state);

    if ((status_info->local_psm == l2cap_profile_info->local_psm) && (status_info->reg_state == (U8)BT_L2CAP_PROFILE_REG_STATE))
    {
        l2cap_profile_info->reg_status = 1;
    }
    else if ((status_info->local_psm == l2cap_profile_info->local_psm) && (status_info->reg_state == (U8)BT_L2CAP_PROFILE_UNREG_STATE))
    {
        l2cap_profile_info->local_psm = 0x00;
        l2cap_profile_info->reg_status = 0;
    }
}

static void bt_l2cap_profile_app_update_device_state(BTS2S_BT_L2CAP_CONN_RES *status_info)
{
    LOG_I("bt_l2cap_profile update device state:0x%2x res:0x%2x bd:%04X:%04X:%04X",
          status_info->device_state, status_info->res, status_info->bd.lap, status_info->bd.uap, status_info->bd.nap);

    switch (status_info->device_state)
    {
    case BT_L2CAP_PROFILE_DEVICE_DISCONNECTED:
    {
        bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(&status_info->bd);
        if (device)
        {
            bmemset(device, 0x00, (sizeof(bts2_bt_l2cap_device_info_inst)));
        }
        break;
    }
    case BT_L2CAP_PROFILE_DEVICE_CONNECTING:
    {
        break;
    }
    case BT_L2CAP_PROFILE_DEVICE_CONNECTED:
    {
        bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(&status_info->bd);
        if (!device)
        {
            device = bt_l2cap_profile_app_get_empty_device_inst();
            if (device && (status_info->res == BTS2_SUCC))
            {
                device->is_use = 1;
                device->cid = status_info->cid;
                device->bd = status_info->bd;
                device->remote_mtu = status_info->remote_mtu;
                device->device_state = status_info->device_state;
            }
            else
            {
                // to do disconnect because no inst to save
            }
        }
        break;
    }
    case BT_L2CAP_PROFILE_DEVICE_DISCONNECTING:
    {

        break;
    }
    }

}

__WEAK void bt_l2cap_profile_app_conn_ind(BTS2S_BT_L2CAP_CONN_IND *conn_info)
{
    bt_l2cap_profile_send_conn_res(1, conn_info);  // 1:accept 0:reject
}


void bt_l2cap_profile_app_msg_hdl(bts2_app_stru *bts2_app_data)
{
    U16 msg_type;
    msg_type = *(U16 *)bts2_app_data->recv_msg;

    LOG_I("bt_l2cap_profile_app_msg_hdl msg_type 0x%2x\n", msg_type);

    switch (msg_type)
    {
    case BTS2MU_BT_L2CAP_PROFILE_REG_RES:
    {
        BTS2S_L2CAP_PROFILE_REG_RES *msg = (BTS2S_L2CAP_PROFILE_REG_RES *)bts2_app_data->recv_msg;
        bt_l2cap_profile_app_update_reg_state(msg);
        break;
    }
    case BTS2MU_BT_L2CAP_PROFILE_MTU_RES:
    {
        BTS2S_BT_L2CAP_MTU_RES *msg = (BTS2S_BT_L2CAP_MTU_RES *)bts2_app_data->recv_msg;
        break;
    }
    case BTS2MU_BT_L2CAP_PROFILE_CONN_STATE:
    {
        BTS2S_BT_L2CAP_CONN_RES *msg = (BTS2S_BT_L2CAP_CONN_RES *)bts2_app_data->recv_msg;
        bt_l2cap_profile_app_update_device_state(msg);
        break;
    }
    case BTS2MU_BT_L2CAP_PROFILE_CONN_IND:
    {
        BTS2S_BT_L2CAP_CONN_IND *msg = (BTS2S_BT_L2CAP_CONN_IND *)bts2_app_data->recv_msg;
        bt_l2cap_profile_app_conn_ind(msg);
        break;
    }
    case BTS2MU_BT_L2CAP_PROFILE_DATA_IND:
    {
        // bt_l2cap_profile_app_recv_data
        BTS2S_BT_L2CAP_DATA_IND *msg = (BTS2S_BT_L2CAP_DATA_IND *)bts2_app_data->recv_msg;
        bt_l2cap_profile_app_recv_data(msg->cid, msg->len, (char *)msg->payload);
        bfree(msg->payload);
        break;
    }
    case BTS2MU_BT_L2CAP_PROFILE_DATA_CFM:
    {
        BTS2S_BT_L2CAP_DATA_CFM *msg = (BTS2S_BT_L2CAP_DATA_CFM *)bts2_app_data->recv_msg;
        bt_l2cap_profile_app_send_data_status(msg->cid, msg->res);
        break;
    }
    }
}

void bt_l2cap_profile_app_init_data(bts2_app_stru *bts2_app_data)
{
    bmemset(&bts2_app_data->bt_l2cap_profile_inst, 0x00, (sizeof(bts2_bt_l2cap_inst_data_t)));
}

/*************************************BT_L2CAP PROFILE CMD *************************************/
U8 bt_l2cap_profile_app_reg_service(U16 psm, U16 flag)
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    if (!l2cap_profile_info->reg_status)
    {
        l2cap_profile_info->local_psm = psm;
        hcia_sync_reg_req(bts2_task_get_app_task_id(), 0);
        bt_l2cap_profile_reg_req(bts2_task_get_app_task_id(), psm, flag);
    }
    else
    {
        return 1;
    }
    return 0;
}

U8 bt_l2cap_profile_app_unreg_service(U16 psm)
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    if (l2cap_profile_info->reg_status)
    {
        l2cap_profile_info->local_psm = 0x00;
        bt_l2cap_profile_unreg_req(bts2_task_get_app_task_id(), psm);
    }
    else
    {
        return 1;
    }
    return 0;
}

U8 bt_l2cap_profile_app_connect_req(BTS2S_BD_ADDR *bd, U16 local_psm, U16 remote_psm)
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    if (l2cap_profile_info->reg_status)
    {
        bt_l2cap_profile_conn_req(bd, local_psm, remote_psm);
    }
    else
    {
        return 1;
    }
    return 0;
}

U8 bt_l2cap_profile_app_disconnect_req(BTS2S_BD_ADDR *bd, U16 psm)
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    if (l2cap_profile_info->reg_status)
    {
        bt_l2cap_profile_disconn_req(bd, psm);
    }
    else
    {
        return 1;
    }
    return 0;
}

U8 bt_l2cap_profile_app_send_data_req(U16 cid, U16 payload_len, char *payload)
{
    bts2_bt_l2cap_inst_data_t *l2cap_profile_info = bt_l2cap_profile_app_get_context();
    if (l2cap_profile_info->reg_status)
    {
        bt_l2cap_profile_send_data_req(cid, payload, payload_len);
    }
    else
    {
        return 1;
    }
    return 0;
}
/*************************************BT_L2CAP PROFILE CMD *************************************/

/*************************************BT_L2CAP PROFILE audio processing start *************************************/
#ifdef AUDIO_USING_MANAGER
audio_client_t   g_bt_3sco_audio_hdl = NULL;
RT_WEAK void bt_l2cap_3sco_audio_open(uint8_t air_mod, uint8_t tx_invl)
{
    audio_parameter_t param = {0};

    param.codec = air_mod;
    param.tsco = tx_invl;//no used, must be 12(7.5ms)
    param.read_channnel_num = 1;
    param.read_bits_per_sample = 16;
    param.write_bits_per_sample = 16;
    param.write_channnel_num = 1;
    if (air_mod == 3)//msbc
    {
        param.read_samplerate = 16000;
        param.write_samplerate = 16000;
    }
    else if (air_mod == 2)//cvsd
    {
        param.read_samplerate = 8000;
        param.write_samplerate = 8000;
    }
    else
    {
        RT_ASSERT(0);
    }

    RT_ASSERT(g_bt_3sco_audio_hdl == NULL);
    g_bt_3sco_audio_hdl = audio_open(AUDIO_TYPE_BT_VOICE, AUDIO_TXRX,  &param, NULL, (void *)AUDIO_TYPE_BT_VOICE);

}
RT_WEAK void bt_l2cap_audio_open(uint8_t air_mod, uint8_t tx_invl)
{
    uint8_t sco_num;
    BTS2S_HF_AUDIO_INFO hf_audio_para;

    sco_num = bt_l2cap_profile_get_cur_sco_num();

    if (sco_num == 1)
    {
        {
            void bt_voice_trans_disable();
            bt_voice_trans_disable();
        }
        bt_l2cap_3sco_audio_open(air_mod, tx_invl);
    }
}
RT_WEAK void bt_l2cap_audio_close(void)
{
    uint8_t sco_num;
    BTS2S_HF_AUDIO_INFO hf_audio_para;

    sco_num = bt_l2cap_profile_get_cur_sco_num();

    if (sco_num == 0)
    {
        int res;
        res = audio_close(g_bt_3sco_audio_hdl);
        g_bt_3sco_audio_hdl = NULL;
    }
}
#else
RT_WEAK void bt_l2cap_audio_open(uint8_t air_mod, uint8_t tx_invl)
{
}
RT_WEAK void bt_l2cap_audio_close(void)
{
}
#endif
/*************************************BT_L2CAP PROFILE audio processing end *************************************/


/*************************************BT_L2CAP PROFILE SCO start *************************************/
void bt_l2cap_profile_hci_msg_hdl(bts2_app_stru *bts2_app_data)
{
    BTS2U_HCI_MSG *dm_msg;
    DBG_IN("gap_dm_arrival_hdl")
    dm_msg = (BTS2U_HCI_MSG *)bts2_app_data->recv_msg;

    switch (dm_msg->type)
    {
    case DM_SYNC_CONN_CFM:
    {
        BTS2S_DM_SYNC_CONN_CFM *msg;
        msg = (BTS2S_DM_SYNC_CONN_CFM *)bts2_app_data->recv_msg;
        LOG_I("BTS2S_DM_SYNC_CONN_CFM msg sco_hdl:0x%2x res:0x%2x", msg->hdl, msg->st);
        if (msg->st == BTS2_SUCC)
        {
            bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(&msg->bd);
            if (device)
            {
                device->sco_hdl = msg->hdl;
            }
            bt_l2cap_audio_open(msg->air_mode, msg->tx_intvl);
        }
        break;
    }
    case DM_SYNC_CONN_COMP_IND:
    {
        BTS2S_DM_SYNC_CONN_COMP_IND *msg;
        msg = (BTS2S_DM_SYNC_CONN_COMP_IND *)bts2_app_data->recv_msg;
        LOG_I("DM_SYNC_CONN_COMP_IND sco_hdl:0x%2x res:0x%2x", msg->hdl, msg->st);
        if (msg->st == BTS2_SUCC)
        {
            bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(&msg->bd);
            if (device)
            {
                device->sco_hdl = msg->hdl;
            }
            bt_l2cap_audio_open(msg->air_mode, msg->tx_intvl);
        }
        break;
    }
    case DM_SYNC_CONN_IND:
    {
        BTS2S_DM_SYNC_CONN_IND *msg;
        msg = (BTS2S_DM_SYNC_CONN_IND *)bts2_app_data->recv_msg;
        LOG_I("DM_SYNC_CONN_IND---------");
        bt_l2cap_profile_app_sco_request_res(&msg->bd, 1);
        break;
    }
    case DM_SYNC_DISC_IND:
    {
        BTS2S_DM_SYNC_DISC_IND *msg;
        msg = (BTS2S_DM_SYNC_DISC_IND *)bts2_app_data->recv_msg;
        LOG_I("DM_SYNC_DISC_IND sco_hdl:0x%2x res:0x%2x", msg->hdl, msg->st);
        bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(&msg->bd);
        if (device)
        {
            device->sco_hdl = 0;
        }
        bt_l2cap_audio_close();
        break;
    }
    case DM_SYNC_DISC_CFM:
    {
        BTS2S_DM_SYNC_DISC_CFM *msg;
        msg = (BTS2S_DM_SYNC_DISC_CFM *)bts2_app_data->recv_msg;
        LOG_I("DM_SYNC_DISC_CFM sco_hdl:0x%2x res:0x%2x", msg->hdl, msg->st);
        bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(&msg->bd);
        if (device)
        {
            device->sco_hdl = 0;
        }
        bt_l2cap_audio_close();
        break;
    }
    default:
    {
        break;
    }
    }
}

U8 bt_l2cap_profile_app_connect_sco_req(BTS2S_BD_ADDR *bd)
{
    bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(bd);
    U8 ret = 1;

    if (device && (device->device_state == BT_L2CAP_PROFILE_DEVICE_CONNECTED) && (!device->sco_hdl))
    {
        hcia_sync_conn_req(bts2_task_get_app_task_id(), 0, bd, BT_SYNC_3SCO_TX_BANDWIDTH,
                           BT_SYNC_3SCO_RX_BANDWIDTH,
                           BT_SYNC_3SCO_MAX_LATENCY,
                           BT_SYNC_3SCO_VOICE_SETTING,
                           BT_SYNC_3SCO_RE_TX_EFFORT,
                           BT_SYNC_3SCO_PACKET_TYPE);
        ret = 0;
    }
    return ret;
}

U8 bt_l2cap_profile_app_disconnect_sco_req(BTS2S_BD_ADDR *bd)
{
    U8 ret = 1;
    bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(bd);
    if (device && device->sco_hdl)
    {
        hcia_sync_disc_req(device->sco_hdl, HCI_ERR_OETC_USER);
        ret = 0;
    }
    return ret;
}

U8 bt_l2cap_profile_app_sco_request_res(BTS2S_BD_ADDR *bd, U8 acpt)
{
    U8 ret = 1;
    bts2_bt_l2cap_device_info_inst *device = bt_l2cap_profile_app_get_device_inst_by_addr(bd);

    if (device && device->device_state == BT_L2CAP_PROFILE_DEVICE_CONNECTED)
    {
        hcia_sync_conn_res(bd, acpt, BT_SYNC_3SCO_TX_BANDWIDTH,
                           BT_SYNC_3SCO_RX_BANDWIDTH,
                           BT_SYNC_3SCO_MAX_LATENCY,
                           BT_SYNC_3SCO_VOICE_SETTING,
                           BT_SYNC_3SCO_RE_TX_EFFORT,
                           BT_SYNC_3SCO_PACKET_TYPE,
                           BT_HCI_CONN_REJED_DUE_TO_LIMITED_RESRCS);
        ret = 0;
    }
    return ret;
}


/*************************************BT_L2CAP PROFILE SCO end *************************************/

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/


