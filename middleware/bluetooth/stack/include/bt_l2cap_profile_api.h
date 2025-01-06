/**
  ******************************************************************************
  * @file   bt_l2cap_profile_api.h
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

#ifndef _BT_L2CAP_PROFILE_API_H_
#define _BT_L2CAP_PROFILE_API_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BT_L2CAP_PROFILE_DEVICE_DISCONNECTED,
    BT_L2CAP_PROFILE_DEVICE_CONNECTING,
    BT_L2CAP_PROFILE_DEVICE_CONNECTED,
    BT_L2CAP_PROFILE_DEVICE_DISCONNECTING
} BT_L2CAP_PROFILE_DEVICE_ST;

typedef enum
{
    BT_L2CAP_PROFILE_UNREG_STATE,
    BT_L2CAP_PROFILE_REGING_STATE,
    BT_L2CAP_PROFILE_REG_STATE,
    BT_L2CAP_PROFILE_UNREGING_STATE
} BT_L2CAP_PROFILE_REG__STATE;

typedef enum
{
    BTS2MD_BT_L2CAP_PROFILE_REG_REQ = BTS2MD_START,//0x00
    BTS2MD_BT_L2CAP_PROFILE_UNREG_REQ,
    BTS2MD_BT_L2CAP_PROFILE_CONN_REQ,
    BTS2MD_BT_L2CAP_PROFILE_DISC_REQ,
    BTS2MD_BT_L2CAP_PROFILE_SEND_DATA_REQ,
    BTS2MD_BT_L2CAP_PROFILE_CHANGE_MTU_REQ,
    BTS2MD_BT_L2CAP_PROFILE_REQ_MAX
} BT_L2CAP_PROFILE_EVENT_REQ_T;

typedef enum
{
    BTS2MU_BT_L2CAP_PROFILE_REG_RES = BTS2MU_START,
    BTS2MU_BT_L2CAP_PROFILE_UNREG_RES,
    BTS2MU_BT_L2CAP_PROFILE_MTU_RES,
    BTS2MU_BT_L2CAP_PROFILE_CONN_IND,
    BTS2MU_BT_L2CAP_PROFILE_CONN_RES,
    BTS2MU_BT_L2CAP_PROFILE_DISC_RES,
    BTS2MU_BT_L2CAP_PROFILE_CONN_STATE,
    BTS2MU_BT_L2CAP_PROFILE_DATA_IND,
    BTS2MU_BT_L2CAP_PROFILE_DATA_CFM,
    BTS2MU_BT_L2CAP_PROFILE_RES_MAX
} BT_L2CAP_PROFILE_EVENT_RES_T;

typedef struct
{
    U16 type;
    U16 local_psm;
    U16 remote_psm;
    BTS2S_BD_ADDR bd;
} BTS2S_BT_L2CAP_PROFILE_CONN_INFO;

typedef struct
{
    U16 type;
    U16 cid;
    U16 payload_len;
    char *payload;
} BTS2S_L2CAP_PROFILE_DATA;

typedef struct
{
    U16 type;
    U16 hdl_id;
    U16 psm;
    U16 flag;
} BTS2S_L2CAP_PROFILE_REG_INFO;

typedef struct
{
    U16 type;
    U16 hdl_id;
    U16 psm;
} BTS2S_L2CAP_PROFILE_UNREG_INFO;

typedef struct
{
    U16 type;
    U16 flow_mode;
    U16 local_mtu_in;
    U16 local_mtu_out;
    U16 local_flush_in;
    U16 local_flush_out;
} BTS2S_L2CAP_PROFILE_CONFIG_PARA;

typedef struct
{
    U16 type;
    U16 local_psm;
    U16 reg_state;
    U16 res;
} BTS2S_L2CAP_PROFILE_REG_RES;

typedef struct
{
    U16 type;
    U16 cid;
    U16 local_psm;
    U16 remote_mtu;
    U8 device_state;
    U8 res;
    BTS2S_BD_ADDR bd;
} BTS2S_BT_L2CAP_CONN_RES;

typedef struct
{
    U16          type; /* always L2CA_CONN_IND */
    U8           id;   /* used to identify the connection signal */
    U16          cid;  /* channel id */
    U16          psm;  /* the PSM on the local device */
    BTS2S_BD_ADDR bd;  /* bluetooth addr of remote device */
} BTS2S_BT_L2CAP_CONN_IND;

typedef struct
{
    U16 type;
    U16 mtu;
    U8 res;
} BTS2S_BT_L2CAP_MTU_RES;

typedef struct
{
    U16 type;
    U16 cid;
    U16 len;
    U8 *payload;
} BTS2S_BT_L2CAP_DATA_IND;

typedef struct
{
    U16 type;
    U16 cid;
    U8  res;
} BTS2S_BT_L2CAP_DATA_CFM;

void bt_l2cap_profile_reg_req(U16 hdl, U16 psm, U16 flag);
void bt_l2cap_profile_unreg_req(U16 hdl, U16 psm);
void bt_l2cap_profile_conn_req(BTS2S_BD_ADDR *bd, U16 local_psm, U16 remote_psm);
void bt_l2cap_profile_send_conn_res(U8 accept, BTS2S_BT_L2CAP_CONN_IND *msg);
void bt_l2cap_profile_disconn_req(BTS2S_BD_ADDR *bd, U16 psm);
void bt_l2cap_profile_send_data_req(U16 cid, char *payload, U16 payload_len);


#ifdef __cplusplus
}
#endif
#endif

