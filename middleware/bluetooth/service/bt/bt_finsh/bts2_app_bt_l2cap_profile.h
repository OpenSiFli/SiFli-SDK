/**
  ******************************************************************************
  * @file   bts2_app_bt_l2cap_profile.h
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
#ifndef _BTS2_APP_BT_L2CAP_PROFILE_H_
#define _BTS2_APP_BT_L2CAP_PROFILE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bts2_global.h"
#ifdef CFG_BT_L2CAP_PROFILE
#include "bts2_app_demo.h"

#define BT_SYNC_3SCO_PACKET_TYPE              0x0380  // only 2EV3
#define BT_SYNC_3SCO_TX_BANDWIDTH             (8000)
#define BT_SYNC_3SCO_RX_BANDWIDTH             (8000)
#define BT_SYNC_3SCO_VOICE_SETTING            (BT_INPUT_CODING_LINEAR | BT_INPUT_DATA_FMT_2COMPLEMENT | BT_INPUT_SAMPLE_SIZE_16BIT | BT_AIR_CODING_CVSD)
#define BT_SYNC_3SCO_MAX_LATENCY              (0xFFFF)//Don't care
#define BT_SYNC_3SCO_RE_TX_EFFORT             (0x00)//no retansmission, 3sco don't support retansmission

void bt_l2cap_profile_app_init_data(bts2_app_stru *bts2_app_data);
void bt_l2cap_profile_app_msg_hdl(bts2_app_stru *bts2_app_data);
void bt_l2cap_profile_hci_msg_hdl(bts2_app_stru *bts2_app_data);
/*************************************BT_L2CAP PROFILE CMD *************************************/
U8 bt_l2cap_profile_app_reg_service(U16 psm, U16 flag);
U8 bt_l2cap_profile_app_unreg_service(U16 psm);
U8 bt_l2cap_profile_app_connect_req(BTS2S_BD_ADDR *bd, U16 local_psm, U16 remote_psm);
void bt_l2cap_profile_app_conn_ind(BTS2S_BT_L2CAP_CONN_IND *conn_info);
U8 bt_l2cap_profile_app_disconnect_req(BTS2S_BD_ADDR *bd, U16 psm);
U8 bt_l2cap_profile_app_send_data_req(U16 cid, U16 payload_len, char *payload);
/*************************************BT_L2CAP PROFILE CMD *************************************/

/*************************************BT_L2CAP PROFILE SCO start *************************************/
U8 bt_l2cap_profile_app_connect_sco_req(BTS2S_BD_ADDR *bd);
U8 bt_l2cap_profile_app_disconnect_sco_req(BTS2S_BD_ADDR *bd);
U8 bt_l2cap_profile_app_sco_request_res(BTS2S_BD_ADDR *bd, U8 acpt);
/*************************************BT_L2CAP PROFILE SCO end *************************************/

#endif
#endif

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

