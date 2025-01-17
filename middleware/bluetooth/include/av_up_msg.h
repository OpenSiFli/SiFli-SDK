/**
  ******************************************************************************
  * @file   av_up_msg.h
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

#ifndef _AV_UPMSG_H_
#define _AV_UPMSG_H_


#ifdef __cplusplus
extern "C" {
#endif

void avs_av_secu_ind(BTS2S_AV_INST_DATA *clt_data);
void avs_av_secu_cfm_rej(BTS2S_AV_INST_DATA *clt_data);
void avs_av_secu_cfm_acpt(BTS2S_AV_INST_DATA *clt_data);

void avs_av_abort_ind(BTS2S_AV_INST_DATA *clt_data);
void avs_av_abort_cfm_acpt(BTS2S_AV_INST_DATA *clt_data);

void avs_av_close_ind(BTS2S_AV_INST_DATA *clt_data);
void avs_av_close_cfm_rej(BTS2S_AV_INST_DATA *clt_data);
void avs_av_close_cfm_acpt(BTS2S_AV_INST_DATA *clt_data);



void avs_av_suspend_ind(BTS2S_AV_INST_DATA *clt_data);
void avs_av_suspend_cfm_rej(BTS2S_AV_INST_DATA *clt_data);
void avs_av_suspend_cfm_acpt(BTS2S_AV_INST_DATA *clt_data);

void avs_av_start_ind(BTS2S_AV_INST_DATA *clt_data);
void avs_av_start_cfm_rej(BTS2S_AV_INST_DATA *clt_data);
void avs_av_start_cfm_acpt(BTS2S_AV_INST_DATA *clt_data);

void avs_av_open_ind(BTS2S_AV_INST_DATA *clt_data);
void avs_av_open_cfm_rej(BTS2S_AV_INST_DATA *clt_data);
void avs_av_open_cfm_acpt(BTS2S_AV_INST_DATA *clt_data);

void avs_av_recfgure_ind(BTS2S_AV_INST_DATA *clt_data);
void avs_av_recfgure_cfm_rej(BTS2S_AV_INST_DATA *clt_data);
void avs_av_recfgure_cfm_acpt(BTS2S_AV_INST_DATA *clt_data);

void avs_av_get_cfguration_ind(BTS2S_AV_INST_DATA *clt_data);
void avs_av_get_cfguration_cfm_rej(BTS2S_AV_INST_DATA *clt_data);
void avs_av_get_cfguration_cfm_acpt(BTS2S_AV_INST_DATA *clt_data);

void avs_av_set_cfguration_ind(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_set_cfguration_cfm_rej(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_set_cfguration_cfm_acpt(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);

void avs_av_get_capabilities_ind(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_get_capabilities_cfm_rej(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_get_capabilities_cfm_acpt(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);

void avs_av_discover_ind(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_discover_cfm_rej(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_discover_cfm_acpt(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);


void avs_av_close_srch_ind(BTS2S_AV_INST_DATA *clt_data);


void avs_av_stream_data_ind(BTS2S_AV_INST_DATA *clt_data);

void avs_av_get_all_capabilities_cfm_acpt(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_get_all_capabilities_cfm_rej(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_get_all_capabilities_ind(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_delay_report_cfm_acpt(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_delay_report_cfm_rej(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);
void avs_av_delay_report_ind(BTS2S_AV_INST_DATA *clt_data, U16 conn_id);

void avs_av_conn_cfm(U16 apphdl,
                     U8 res,
                     U16 conn_id,
                     BTS2S_BD_ADDR bd,
                     U16 mtu_size);
void avs_av_conn_ind(U16 apphdl, U16 conn_id, BTS2S_BD_ADDR bd);


void avs_av_enb_cfm(U16 apphdl, U16 enable_role, U8 res);
void avs_av_disb_cfm(U16 apphdl, U8 res);

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
