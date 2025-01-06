/**
  ******************************************************************************
  * @file   bt_rt_device_urc_spp.c
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

#define DBG_TAG               "bt_rt_device.urc_spp"
//#define DBG_LVL               DBG_INFO
#include <log.h>


void urc_func_spp_conn_ind_sifli(uint8_t *addr, U8 srv_chl, U8 *uuid, U8 uuid_len, U16 mfs)
{
    bt_notify_t args;
    bt_spp_conn_ind_t spp_conn_ind = {0};
    rt_memcpy(spp_conn_ind.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    spp_conn_ind.uuid = uuid;
    spp_conn_ind.uuid_len = uuid_len;
    spp_conn_ind.srv_chl = srv_chl;
    spp_conn_ind.mtu_size = mfs;
    args.event = BT_EVENT_SPP_CONN_IND;
    args.args = &spp_conn_ind;
    rt_bt_event_notify(&args);
    LOG_I("URC BT spp conn ind");
}

void urc_func_spp_data_ind_sifli(U8 *payload, U16 payload_len, uint8_t *addr, U8 srv_chl, U8 *uuid, U8 uuid_len)
{
    bt_notify_t args;
    bt_spp_data_t data_ind = {0};
    data_ind.payload = payload;
    data_ind.payload_len = payload_len;
    rt_memcpy(data_ind.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    data_ind.srv_chl = srv_chl;
    data_ind.uuid = uuid;
    data_ind.uuid_len = uuid_len;
    args.event = BT_EVENT_SPP_DATA_IND;
    args.args = &data_ind;
    rt_bt_event_notify(&args);
    LOG_I("URC BT spp data ind");
}

void urc_func_spp_data_cfm_sifli(uint8_t *addr, U8 srv_chl, U8 *uuid, U8 uuid_len)
{
    bt_notify_t args;
    bt_spp_data_cfm_t data_cfm = {0};
    rt_memcpy(data_cfm.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    data_cfm.srv_chl = srv_chl;
    data_cfm.uuid = uuid;
    data_cfm.uuid_len = uuid_len;

    args.event = BT_EVENT_SPP_DATA_CFM;
    args.args = &data_cfm;
    rt_bt_event_notify(&args);
    LOG_I("URC BT spp data cfm");
    return;
}

void urc_func_spp_disconn_ind_sifli(uint8_t *addr, U8 srv_chl, U8 *uuid, U8 uuid_len)
{
    bt_notify_t args;
    bt_spp_disconn_ind_t spp_disconn_ind = {0};
    rt_memcpy(spp_disconn_ind.peer_addr.addr, addr, BT_MAX_MAC_LEN);
    spp_disconn_ind.uuid = uuid;
    spp_disconn_ind.uuid_len = uuid_len;
    spp_disconn_ind.srv_chl = srv_chl;
    args.event = BT_EVENT_SPP_DISCONN_IND;
    args.args = &spp_disconn_ind;
    rt_bt_event_notify(&args);
    LOG_I("URC BT spp disconn ind");
}

int bt_sifli_notify_spp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_SPP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_conn_sifli(profile_info->mac.addr, BT_PROFILE_SPP);
        break;
    }
    case BT_NOTIFY_SPP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_disc_sifli(profile_info->mac.addr, BT_PROFILE_SPP, profile_info->res);
        break;
    }
    case BT_NOTIFY_SPP_CONN_IND:
    {
        bt_notify_spp_conn_ind_t *conn_ind = (bt_notify_spp_conn_ind_t *)data;
        if (conn_ind->uuid_len == 2)
        {
            urc_func_spp_conn_ind_sifli(conn_ind->mac.addr, conn_ind->srv_chl, conn_ind->uuid.uuid_16, conn_ind->uuid_len, conn_ind->mfs);
        }
        else if (conn_ind->uuid_len == 4)
        {
            urc_func_spp_conn_ind_sifli(conn_ind->mac.addr, conn_ind->srv_chl, conn_ind->uuid.uuid_32, conn_ind->uuid_len, conn_ind->mfs);
        }
        else if (conn_ind->uuid_len == 16)
        {
            urc_func_spp_conn_ind_sifli(conn_ind->mac.addr, conn_ind->srv_chl, conn_ind->uuid.uuid_128, conn_ind->uuid_len, conn_ind->mfs);
        }
        break;
    }
    case BT_NOTIFY_SPP_DATA_IND:
    {
        bt_notify_spp_data_ind_t *data_info = (bt_notify_spp_data_ind_t *)data;
        if (data_info->uuid_len == 2)
        {
            urc_func_spp_data_ind_sifli(data_info->payload,
                                        data_info->payload_len,
                                        data_info->mac.addr,
                                        data_info->srv_chl,
                                        data_info->uuid.uuid_16,
                                        data_info->uuid_len);
        }
        else if (data_info->uuid_len == 4)
        {
            urc_func_spp_data_ind_sifli(data_info->payload,
                                        data_info->payload_len,
                                        data_info->mac.addr,
                                        data_info->srv_chl,
                                        data_info->uuid.uuid_32,
                                        data_info->uuid_len);
        }
        else if (data_info->uuid_len == 16)
        {
            urc_func_spp_data_ind_sifli(data_info->payload,
                                        data_info->payload_len,
                                        data_info->mac.addr,
                                        data_info->srv_chl,
                                        data_info->uuid.uuid_128,
                                        data_info->uuid_len);
        }
        break;
    }
    case BT_NOTIFY_SPP_DATA_CFM:
    {
        bt_notify_spp_data_cfm_t *data_cfm = (bt_notify_spp_data_cfm_t *)data;
        if (data_cfm->uuid_len == 2)
        {
            urc_func_spp_data_cfm_sifli(data_cfm->mac.addr,
                                        data_cfm->srv_chl,
                                        data_cfm->uuid.uuid_16,
                                        data_cfm->uuid_len);
        }
        else if (data_cfm->uuid_len == 4)
        {
            urc_func_spp_data_cfm_sifli(data_cfm->mac.addr,
                                        data_cfm->srv_chl,
                                        data_cfm->uuid.uuid_32,
                                        data_cfm->uuid_len);
        }
        else if (data_cfm->uuid_len == 16)
        {
            urc_func_spp_data_cfm_sifli(data_cfm->mac.addr,
                                        data_cfm->srv_chl,
                                        data_cfm->uuid.uuid_128,
                                        data_cfm->uuid_len);
        }
        break;
    }
    case BT_NOTIFY_SPP_DISC_IND:
    {
        bt_notify_spp_disc_ind_t *disc_ind = (bt_notify_spp_disc_ind_t *)data;
        if (disc_ind->uuid_len == 2)
        {
            urc_func_spp_disconn_ind_sifli(disc_ind->mac.addr,
                                           disc_ind->srv_chl,
                                           disc_ind->uuid.uuid_16,
                                           disc_ind->uuid_len);
        }
        else if (disc_ind->uuid_len == 4)
        {
            urc_func_spp_disconn_ind_sifli(disc_ind->mac.addr,
                                           disc_ind->srv_chl,
                                           disc_ind->uuid.uuid_32,
                                           disc_ind->uuid_len);
        }
        else if (disc_ind->uuid_len == 16)
        {
            urc_func_spp_disconn_ind_sifli(disc_ind->mac.addr,
                                           disc_ind->srv_chl,
                                           disc_ind->uuid.uuid_128,
                                           disc_ind->uuid_len);
        }
        break;
    }
    default:
        return -1;
    }
    return 0;
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

