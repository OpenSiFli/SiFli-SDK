/**
  ******************************************************************************
  * @file   bts2_app_hfp_ag.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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
#ifndef _BTS2_APP_HFP_AG_H_
#define _BTS2_APP_HFP_AG_H_




#ifdef __cplusplus
extern "C" {
#endif

#include "bts2_global.h"
#ifdef CFG_HFP_AG

/*************************************func *************************************/
void bt_hfp_ag_app_init(bts2_app_stru *bts2_app_data);
void bt_hfp_ag_msg_hdl(bts2_app_stru *bts2_app_data);
void bt_hfp_start_profile_service(bts2_app_stru *bts2_app_data);
void bt_hfp_stop_profile_service(bts2_app_stru *bts2_app_data);
void bt_hfp_connect_profile(BTS2S_BD_ADDR *bd);
void bt_hfp_disconnect_profile(BTS2S_BD_ADDR *bd);
void bt_hfp_connect_audio(BTS2S_BD_ADDR *bd);
void bt_hfp_disconnect_audio(BTS2S_BD_ADDR *bd);
void bt_hfp_ag_call_state_update_listener(HFP_CALL_INFO_T *call_info);
void bt_hfp_ag_remote_calls_res_hdl(hfp_remote_calls_info_t *call_info);
void bt_hfp_ag_app_call_status_change(char *phone_num, uint8_t phone_len, uint8_t active, uint8_t callsetup_state);




/*************************************AT CMD *************************************/
void bt_hfp_ag_spk_vol_control(U8 vol);
void bt_hfp_ag_mic_vol_control(U8 vol);
void bt_hfp_ag_cind_response(hfp_cind_status_t *cind_status);
void bt_hfp_ag_ind_status_update(U8 type, U8 val);
void bt_hfp_ag_brva_response(U8 val);
void bt_hfp_ag_set_inband(U8 val);
void bt_hfp_ag_cnum_response(hfp_phone_number_t *local_phone_num);
void bt_hfp_ag_set_btrh(U8 val);
void bt_hfp_ag_clcc_response(hfp_phone_call_info_t *call_info);
void bt_hfp_ag_cops_response(char *payload, U8 payload_len);
void bt_hfp_ag_set_bcs(U8 code_id);
void bt_hfp_ag_clip_response(hfp_phone_number_t *remote_phone_num);
void bt_hfp_ag_at_result_res(U8 res);

#endif

#endif
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

