/**
  ******************************************************************************
  * @file   bt_gatt_api.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
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

#ifndef _BT_GATT_API_H_
#define _BT_GATT_API_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "bts2_bt.h"
#include "bts2_msg.h"

typedef enum
{
    BTS2MD_BT_GATT_SDP_REG_REQ = BTS2MD_START,//0x00
    BTS2MD_BT_GATT_SDP_UNREG_REQ,
    BTS2MD_BT_GATT_CONN_REQ,
    BTS2MD_BT_GATT_DISC_REQ,
    BTS2MD_BT_GATT_SEND_DATA_REQ,
    BTS2MD_BT_GATT_CHANGE_MTU_REQ,
    BTS2MD_BT_GATT_REQ_MAX
} BT_GATT_EVENT_REQ_T;

enum
{
    BTS2MU_BT_GATT_SDP_RES = BTS2MU_START,
    BTS2MU_BT_GATT_SDP_UNRES,
    BTS2MU_BT_GATT_MTU_RES,
    BTS2MD_BT_GATT_CONN_RES,
    BTS2MD_BT_GATT_DISC_RES,
    BTS2MU_BT_GATT_CONN_STATE,
    BTS2MD_BT_GATT_RES_MAX
};

typedef enum
{
    BT_GATT_DISCONNECTED = 0,
    BT_GATT_CONNECTING,
    BT_GATT_CONNECTED,
    BT_GATT_DISCONNECTING,
} bt_gatt_device_state_t;

typedef struct
{
    U16          type;
    BTS2S_BD_ADDR bd;
} BTS2S_BT_GATT_CONN_INFO;

typedef struct
{
    U16 type;
    U16 payload_len;
    char *payload;
} BTS2S_BT_GATT_DATA;

typedef struct
{
    U16 type;
    U16 mtu;
} BTS2S_BT_GATT_MTU;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U8 device_state;
    U8 res;
} BTS2S_BT_GATT_RES;

typedef struct
{
    U16 type;
    U32 sdp_rec_hdl;
    U8 res;
} bt_gatt_sdp_reg_info;

typedef struct
{
    U16 type;
    U16 mtu;
    U8 res;
} bt_gatt_mtu_res_info;

typedef struct
{
    U16 gatt_start_handle;
    U16 gatt_end_handle;
    U8 att_uuid_len;
    U8 *att_uuid;
} br_att_sdp_data_t;

void bt_gatt_conn_req(BTS2S_BD_ADDR *bd);
void bt_gatt_disconn_req(BTS2S_BD_ADDR *bd);
void bt_gatt_send_data_req(char *payload, U16 payload_len);
void bt_gatt_change_mtu_req(U16 mtu);
void bt_gatt_create_sdp_record(U8 att_uuid_len, U8 *att_uuid, U16 gatt_start_handle, U16 gatt_end_handle);
void bt_gatt_create_sdp_unreg(U32 svc_rec_hdl);

#ifdef __cplusplus
}
#endif
#endif

