/**
  ******************************************************************************
  * @file   bts2_app_hfp_hf.h
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

#ifndef _BTS2_APP_HFP_HF_H_
#define _BTS2_APP_HFP_HF_H_

#include "drivers/bt_device.h"


#ifdef __cplusplus
extern "C" {
#endif

#ifdef CFG_HFP_HF
void bt_hfp_hf_init(bts2_app_stru *bts2_app_data);
bt_err_t bt_hfp_hf_start_enb(bts2_app_stru *bts2_app_data);
bt_err_t bt_hfp_hf_start_disb(bts2_app_stru *bts2_app_data);
void bt_hfp_hf_clean_flag();
U8 bt_hfp_hf_get_ring_type(void);
bt_err_t bt_hfp_hf_start_connecting(BTS2S_BD_ADDR *bd);
bt_err_t bt_hfp_hf_start_disc(BTS2S_BD_ADDR *bd);
bt_err_t bt_hfp_hf_audio_transfer(U8 type);
bt_err_t bt_hfp_hf_voice_recog_send(U8 active);
bt_err_t bt_hfp_hf_dial_by_mem_send(U16 memory);
bt_err_t bt_hfp_hf_last_num_dial_send(void);
bt_err_t bt_hfp_hf_make_call_by_number_send(U8 *payload, U8 payload_len);
bt_err_t bt_hfp_hf_answer_call_send(void);
bt_err_t bt_hfp_hf_hangup_call_send(void);
bt_err_t bt_hfp_hf_update_spk_vol(U8 vol);
bt_err_t bt_hfp_hf_update_mic_vol(U8 vol);
bt_err_t bt_hfp_hf_at_btrh_query_send(void);
bt_err_t bt_hfp_hf_at_btrh_cmd_send(U8 mode);
bt_err_t bt_hfp_hf_at_binp_send(void);
bt_err_t bt_hfp_hf_at_clip_send(U8 enable);
bt_err_t bt_hfp_hf_at_cmee_send(BOOL val);
bt_err_t bt_hfp_hf_at_cnum_send(void);
bt_err_t bt_hfp_hf_at_ccwa_send(BOOL val);
bt_err_t bt_hfp_hf_at_chld_send(U8 *payload, U8 payload_len);
bt_err_t bt_hfp_hf_at_clcc_send(void);
bt_err_t bt_hfp_hf_at_cops_cmd_send(void);
bt_err_t bt_hfp_hf_at_dtmf_send(char key);
bt_err_t bt_hfp_hf_at_nrec_send(void);
bt_err_t bt_hfp_hf_update_batt_send(U8 batt_val);

void bt_hfp_hf_rfc_conn_accept_hdl(void);
void bt_hfp_hf_rfc_conn_rej_hdl(void);
U8 bt_hfp_hf_get_ciev_info(BTS2S_HF_CIEV_IND *msg);
void bt_hfp_hf_msg_hdl(bts2_app_stru *bts2_app_data);

#else
#define bt_hfp_hf_start_enb(bts2_app_data)  (-BT_ERROR_INPARAM)
#define bt_hfp_hf_start_disb(bts2_app_data) (-BT_ERROR_INPARAM)
#define bt_hfp_hf_clean_flag()
#define bt_hfp_hf_get_ring_type()           0
#define bt_hfp_hf_start_connecting(bd)      (-BT_ERROR_INPARAM)
#define bt_hfp_hf_start_disc(bd)            (-BT_ERROR_INPARAM)
#define bt_hfp_hf_audio_transfer(type)      (-BT_ERROR_INPARAM)
#define bt_hfp_hf_voice_recog_send(active)  (-BT_ERROR_INPARAM)
#define bt_hfp_hf_dial_by_mem_send(memory)  (-BT_ERROR_INPARAM)
#define bt_hfp_hf_last_num_dial_send()      (-BT_ERROR_INPARAM)
#define bt_hfp_hf_make_call_by_number_send(payload,payload_len) (-BT_ERROR_INPARAM)
#define bt_hfp_hf_answer_call_send()        (-BT_ERROR_INPARAM)
#define bt_hfp_hf_hangup_call_send()        (-BT_ERROR_INPARAM)
#define bt_hfp_hf_update_spk_vol(vol)       (-BT_ERROR_INPARAM)
#define bt_hfp_hf_update_mic_vol(vol)       (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_btrh_query_send()      (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_btrh_cmd_send(mode)    (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_binp_send()            (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_clip_send(enable)      (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_cmee_send(val)         (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_cnum_send()            (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_ccwa_send(val)         (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_chld_send(payload,payload_len) (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_clcc_send()            (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_cops_cmd_send()        (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_dtmf_send(key)         (-BT_ERROR_INPARAM)
#define bt_hfp_hf_at_nrec_send()            (-BT_ERROR_INPARAM)
#define bt_hfp_hf_update_batt_send(batt_val)    (-BT_ERROR_INPARAM)
#define bt_hfp_hf_start_connecting(bd)      (-BT_ERROR_INPARAM)

#endif




#ifdef __cplusplus
}
#endif
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
