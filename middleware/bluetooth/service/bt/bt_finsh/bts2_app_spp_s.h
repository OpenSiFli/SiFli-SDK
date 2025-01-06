/**
  ******************************************************************************
  * @file   bts2_app_spp_s.h
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

#ifndef _BTS2_APP_SPP_B_H_
#define _BTS2_APP_SPP_B_H_
#ifdef __cplusplus
extern "C" {
#endif

#ifdef CFG_SPP_SRV

BOOL bt_spp_srv_test_enable(void);


void bt_spp_srv_add_uuid_list(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_set_instance_index(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_init(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_start_enb(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_start_disb(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_mode_change_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chnl, U8 mode);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_select_file_to_send(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_open_the_selected_file(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BOOL bt_spp_srv_check_addr_is_connected(bts2_app_stru *bts2_app_data, BTS2S_BD_ADDR *bd, U8 *idx);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 bt_spp_srv_get_available_sub_inst(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bts2_spp_service_list *bt_spp_srv_add_new_service_list(bts2_spp_srv_inst_data *spp_srv_inst_ptr, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_delete_service_list_by_srv_chl(bts2_spp_srv_inst_data *spp_srv_inst_ptr, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bts2_spp_service_list *bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst_data *spp_srv_inst_ptr, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      send data to special channel
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_sending_data_by_device_id_and_srv_chnl(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl, U8 *payload, U16 payload_len);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      get mtu size of special channel
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U16 bt_spp_srv_get_mtu_size(bts2_app_stru *bts2_app_data, U8 device_id);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_sending_the_selected_file(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_sending_next_pieceof_file(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      U16 U16:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_time_out(U8 spp_id, bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_esc_file_transfer(bts2_app_stru *bts2_app_data, U8 spp_id);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_data_ind_hdl(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_msg_hdl(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_rfc_conn_accept_hdl(bts2_app_stru *bts2_app_data, U8 srv_chl, BTS2S_BD_ADDR bd);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_rfc_conn_rej_hdl(bts2_app_stru *bts2_app_data, U8 srv_chl, BTS2S_BD_ADDR bd);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 bt_spp_srv_get_srv_chl_by_uuid(U8 *uuid, U8 uuid_len);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_dump_all_spp_connect_information(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_test_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_linest_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_switch_role(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_portneg_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_disc_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);

void bt_spp_srv_disc_req_all(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_sending_random_data(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 bt_spp_srv_sending_random_data_next(bts2_spp_service_list *spp_service_list, U8 device_id);

void bt_spp_srv_set_write_into_file(bts2_app_stru *bts2_app_data, U8 on_or_off);

void bt_spp_srv_sending_data_to_peer(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl);
#else
#define bt_spp_srv_set_instance_index(bts2_app_data)
#define bt_spp_srv_init(bts2_app_data)
#define bt_spp_srv_start_enb(bts2_app_data)
#define bt_spp_srv_start_disb(bts2_app_data)
#define bt_spp_srv_mode_change_req(bts2_app_data)
#define bt_spp_srv_select_file_to_send(bts2_app_data)
#define bt_spp_srv_open_the_selected_file(bts2_app_data)
#define bt_spp_srv_sending_data_to_peer(bts2_app_data,payload,payload_len)
#define bt_spp_srv_get_mtu_size(bts2_app_data) 0
#define bt_spp_srv_sending_the_selected_file(bts2_app_data,inst_data)
#define bt_spp_srv_sending_next_pieceof_file(bts2_app_data,device_id,srv_chl)
#define bt_spp_srv_esc_file_transfer(bts2_app_data)
#define bt_spp_srv_data_ind_hdl(bts2_app_data)
#define bt_spp_srv_msg_hdl(bts2_app_data)
#define bt_spp_srv_rfc_conn_accept_hdl(bts2_app_data)
#define bt_spp_srv_rfc_conn_rej_hdl(bts2_app_data)
#define bt_spp_srv_test_req(bts2_app_data)
#define bt_spp_srv_linest_req(bts2_app_data)
#define bt_spp_srv_switch_role(bts2_app_data)
#define bt_spp_srv_portneg_req(bts2_app_data)
#define bt_spp_srv_disc_req(bts2_app_data)
#define bt_spp_srv_sending_random_data(bts2_app_data)
#endif
#ifdef __cplusplus
}
#endif
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
