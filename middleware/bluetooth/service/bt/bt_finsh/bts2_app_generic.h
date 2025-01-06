/**
  ******************************************************************************
  * @file   bts2_app_generic.h
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

#ifndef _BTS2_APP_GENERIC_H_
#define _BTS2_APP_GENERIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BT_FINSH_PAN
//add for tcpip route
extern BTS2S_ETHER_ADDR   bts2_local_ether_addr;
#endif

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_start_inquiry(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_stop_inquiry(bts2_app_stru *bts2_app_data);

void bt_register_receive_earphone_connect_req_handler(BOOL (*cb)(BTS2S_BD_ADDR *p_bd));

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_pincode_indi(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_start_pairing(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_start_unpairing(bts2_app_stru *bts2_app_data);


void bt_io_capability_rsp(BTS2S_BD_ADDR *bd,
                          BTS2E_SC_IO_CAPABILITY io_capability,
                          U8 mitm,
                          U8 bonding);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_init_local_ctrller(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_set_local_name(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_wr_dev_cls(bts2_app_stru *bts2_app_data);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 * User-implemenation API.
 *
 * INPUT:
 *      void
 *
 * OUTPUT:
 *      class of device.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
uint32_t bt_get_class_of_device(void);

void bt_sc_io_capability_rsp(BTS2S_BD_ADDR *bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_wr_page_timeout(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_set_connable(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_get_rmt_dev_name(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_get_role_discov(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_get_rmt_version(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_wr_link_supvisn_timeout(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_rd_transmit_power_level(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_get_rssi(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_wr_link_policy(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_rd_clock(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_wr_afh_chnl_assessment_mode(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_set_secu_mode(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_hdl_gap_msg(bts2_app_stru *bts2_app_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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

#ifdef CFG_BQB_SDC_SSA
void bt_hdl_sdp_msg(bts2_app_stru *bts2_app_data);
#endif


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_hdl_sc_msg(bts2_app_stru *bts2_app_data);
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
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
void bt_close_kb_thread(void);

U8  bt_get_oob_data(bts2_app_stru *app_data);
void  bt_get_sele_yesno(bts2_app_stru *app_data);
U8  bt_get_user_psky_value(bts2_app_stru *app_data);
U8  bt_get_passkey_notifi(bts2_app_stru *app_data);
U8  bt_hdl_sc_input(bts2_app_stru *bts2_app_data);
U8  bt_hdl_sc_passkey_notifi(bts2_app_stru *bts2_app_data);
U8  bt_hdl_sc_yesno(bts2_app_stru *bts2_app_data);
U8  bt_hdl_sc_oobdata(bts2_app_stru *bts2_app_data);


#ifdef RT_USING_BT
void bt_sifli_interface_stop_inquiry(void);
void bt_sifli_interface_start_inquiry(void);
#endif
void bt_etner_sniff_mode(bts2_app_stru *bts2_app_data);
void bt_exit_sniff_mode(bts2_app_stru *bts2_app_data);




#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
