/**
  ******************************************************************************
  * @file   bts2_app_bt_dis.c
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
#ifdef CFG_BT_DIS
#include "bts2_app_inc.h"

#define LOG_TAG         "btapp_dis"
#include "log.h"


bt_dis_device_info_t g_dis_info =
{
    .prim_rec = 1,
    .spec_id = 0x0102,
    .prod_id = DIS_PRODUCT_ID,
    .prod_ver = DIS_PRODUCT_VERSION,
    .vend_id = DIS_VENDOR_ID,
    .vend_id_source = DIS_VENDID_SOURCE_BTSIG
};

uint32_t g_sdp_rec_hdl = 0xffff;


void bt_dis_app_set_prim_rec(uint8_t prim_rec)
{
    g_dis_info.prim_rec = prim_rec;
}

void bt_dis_app_set_vendor_id(uint16_t vend_id, uint16_t vend_id_source)
{
    g_dis_info.vend_id = vend_id;
    g_dis_info.vend_id_source = vend_id_source;
}

void bt_dis_app_set_prod_id(uint16_t prod_id)
{
    g_dis_info.prod_id = prod_id;
}

void bt_dis_app_set_prod_ver(uint16_t prod_ver)
{
    g_dis_info.prod_ver = prod_ver;
}

void bt_dis_app_set_spec_id(uint16_t spec_id)
{
    g_dis_info.spec_id = spec_id;
}

void bt_dis_app_set_sdp_rec_hdl(uint32_t sdp_rec_hdl)
{
    g_sdp_rec_hdl = sdp_rec_hdl;
}

bt_dis_device_info_t *bt_dis_app_get_dis_info()
{
    return &g_dis_info;
}

uint8_t bt_dis_app_get_prim_rec()
{
    return g_dis_info.prim_rec;
}

uint16_t bt_dis_app_get_vendor_id()
{
    return g_dis_info.vend_id;
}

uint16_t bt_dis_app_get_vend_id_source()
{
    return g_dis_info.vend_id_source;
}

uint16_t bt_dis_app_get_prod_id()
{
    return g_dis_info.prod_id;
}

uint16_t bt_dis_app_get_prod_ver()
{
    return g_dis_info.prod_ver;
}

uint16_t bt_dis_app_get_spec_id()
{
    return g_dis_info.spec_id;
}

uint32_t bt_dis_app_get_sdp_rec_hdl()
{
    return g_sdp_rec_hdl;
}


bt_err_t bt_dis_app_sdp_reg()
{
    bt_dis_sdp_req_api(bt_dis_app_get_dis_info());
    return BT_EOK;
}

bt_err_t bt_dis_app_sdp_unreg()
{
    bt_dis_sdp_unreq_api(bt_dis_app_get_sdp_rec_hdl());
    return BT_EOK;
}


void bt_dis_app_msg_hdl(bts2_app_stru *bts2_app_data)
{
    bt_dis_sdp_reg_cfm_t *msg;
    msg = (bt_dis_sdp_reg_cfm_t *)bts2_app_data->recv_msg;

    USER_TRACE("bt_dis_app_msg_hdl *msg_type %x\n", msg->type);

    switch (msg->type)
    {
    case BTS2MU_BT_DIS_SDP_RES:
    {
        bt_dis_app_set_sdp_rec_hdl(msg->sdp_sec_hdl);
        break;
    }

    case BTS2MU_BT_DIS_SDP_UNRES:
    {
        bt_dis_app_set_sdp_rec_hdl(msg->sdp_sec_hdl);
        break;
    }

    }
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
