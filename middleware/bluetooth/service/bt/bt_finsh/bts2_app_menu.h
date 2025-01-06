/**
  ******************************************************************************
  * @file   bts2_app_menu.h
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

#ifndef _BTS2_APP_MENU_H_
#define _BTS2_APP_MENU_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    menu_cfg = 0x00,
    menu_start,
    menu_main,

    menu_gen,
    menu_gen_3,
    menu_gen_4,
    menu_gen_4_3,
    menu_gen_5,
    menu_gen_5_1,
    menu_gen_5_4,
    menu_gen_5_7,
    menu_gen_5_8,
    menu_gen_5_a,
    menu_gen_6,
    menu_gen_6_1,
    menu_gen_6_2,
    menu_gen_6_3,
    menu_gen_7,
    menu_gen_8,
    menu_gen_8_1,
    menu_gen_8_2,
    menu_gen_8_4,
    menu_gen_8_5,
    menu_gen_8_7,
    menu_gen_8_8,
    menu_gen_8_9,
    menu_gen_8_b,
    menu_gen_9,
    menu_gen_a,
    menu_gen_b,

    menu_sc_pair,
    menu_sc_passkey_notifi,
    menu_sc_input,
    menu_sc_yesno,
    menu_sc_oobdata,
    menu_hfp_hf,
    menu_hfp_hf_5,

    menu_hfp_hf_a,
    menu_hfp_hf_a_1,
    menu_hfp_hf_a_2,
    menu_hfp_hf_h,
    menu_hfp_hf_l,
    menu_hfp_hf_p,

#ifdef CFG_HFP_AG
    menu_hfp_ag,
    menu_hfp_ag_1,
#endif

#ifdef CFG_AV_SNK
    menu_av_snk,
#endif

#ifdef CFG_AV_SRC
    menu_av_src,
    menu_av_src_bqb,
#endif

#ifdef CFG_VDP_SNK
    menu_vdp_snk,
#endif

#ifdef CFG_VDP_SRC
    menu_vdp_src,
#endif

#ifdef CFG_AVRCP
    menu_avrcp,
    menu_avrcp_a,
#endif

#ifdef CFG_HID
    menu_hid,
#endif

#ifdef CFG_SPP_CLT
    menu_spp_clt,
    menu_spp_clt_0,
    menu_spp_clt_3,
    menu_spp_clt_4,
    menu_spp_clt_5,
#endif

#ifdef CFG_SPP_SRV
    menu_spp_srv,
    menu_spp_srv_0,
    menu_spp_srv_3,
    menu_spp_srv_4,
    menu_spp_srv_5,
    menu_spp_srv_6,
#endif
    menu_l2cap_bqb,
    menu_l2cap_bqb_1,
    menu_l2cap_bqb_2,

    menu_pan_g,

#ifdef CFG_PBAP_CLT
    menu_pbap_c,
    menu_pbap_c_1,
    menu_pbap_c_3,
    menu_pbap_c_4,
    menu_pbap_c_6,
#endif

#ifdef CFG_BR_GATT_SRV
    menu_bt_gatt_srv,
#endif

#ifdef CFG_BT_DIS
    menu_bt_dis,
#endif

#ifdef CFG_BT_L2CAP_PROFILE
    menu_bt_l2cap_profile,
#endif

} menu_st;

/*fn */
void bt_disply_menu(bts2_app_stru *bts2_app_data);
void bt_hdl_menu(bts2_app_stru *bts2_app_data);

#ifdef __cplusplus
}
#endif
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
