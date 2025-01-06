/**
  ******************************************************************************
  * @file   hci_api.h
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

#ifndef _HCI_API_H_
#define _HCI_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hci_spec.h"




/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_AM_REG_REQ message to the HCI command task.
 *
 *
 * INPUT:
 *     U16 phdl: handle.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_gap_mgm_reg_req(U16 phdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_ACL_REG_REQ message to the HCI ACL task.
 *
 *
 * INPUT:
 *     U16 phdl: handle.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_acl_reg_req(U16 phdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_ACL_CONN_REQ message.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U16 pkt_type: packet type.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_acl_conn_req(BTS2S_BD_ADDR *bd, U16 pkt_type);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_ACL_DISC_REQ message.
 *
 *
 * INPUT:
 *     U16 phdl: handle.
 *     U8 reason: disconnection reason.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_acl_disc_req(U16 hdl, U8 reason);
int8_t hcia_acl_disc_req_by_addr(BTS2S_BD_ADDR *bd, U8 reason);
int8_t hcia_acl_close_bt_req_by_addr(BTS2S_BD_ADDR *bd, U8 reason);
int8_t hcia_acl_cancel_conn_req_by_addr(BTS2S_BD_ADDR *bd);
int8_t hcia_acl_open_bt_req(void);



/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_CONNLESS_CH_REG_REQ message.
 *
 *
 * INPUT:
 *     U16 phdl: handle.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_connless_ch_reg_req(U16 hdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_WR_CACHED_PAGE_MODE_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U8 page_scan_mode: page scan mode.
 *     U8 page_scan_rep_mode: page scan repetition mode.
 *     BTS2U_HCI_MSG **pmsg: HCI message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_cached_page_mode_req(BTS2S_BD_ADDR *bd,
                                  U8 page_scan_mode,
                                  U8 page_scan_rep_mode,
                                  BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_WR_CACHED_CLOCK_OFFSET_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U16 clock_offset: clock offset.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_cached_clock_offset_req(BTS2S_BD_ADDR *bd,
                                     U16 clock_offset,
                                     BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_WR_CACHED_CLOCK_OFFSET_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_clear_param_cache_req(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_ACL_OPEN_REQ message.if pmsg != NULL, message is returned,
 *     not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_acl_open_req(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_ACL_CLOSE_REQ message.if pmsg != NULL,
 *     message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_acl_close_req(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SET_DEFAULT_LINK_POLICY message.if pmsg != NULL,
 *     message is returned, not sent.
 *
 *
 * INPUT:
 *     U16 bts2s_default_lp_in: in default link policy.
 *     U16 bts2s_default_lp_out: out default link policy.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_set_default_link_policy(U16 bts2s_default_lp_in,
                                  U16 bts2s_default_lp_out,
                                  BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_SET_DEFAULT_SECU_REQ message. if pmsg != NULL,
 *     message is returned, not sent.
 *
 *
 * INPUT:
 *     U16 default_secu_level: default security level.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_set_default_secu_req(U16 default_secu_level,
                                  BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_REG_REQ message.if pmsg != NULL,
 *     message is returned, not sent.
 *
 *
 * INPUT:
 *     U32 prot_id: protocol id.
 *     U32 chnl: channel.
 *     BOOL outgoing_ok: is outgoing ok?
 *     U16 secu_level: security level.
 *     U16 psm: protocol/service multiplexer.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_reg_req(U32 prot_id,
                     U32 chnl,
                     BOOL outgoing_ok,
                     U16 secu_level,
                     U16 psm, /* zero if don't care about connectionless security */
                     BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_REG_OUTGOING_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U32 prot_id: protocol id.
 *     U32 rmt_chnl: remote channel.
 *     U16 outgoing_secu_level: outgoing security level.
 *     U16 psm: protocol/service multiplexer.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_reg_outgoing_req(BTS2S_BD_ADDR *bd,
                              U32 prot_id,
                              U32 rmt_chnl,
                              U16 outgoing_secu_level,
                              U16 psm, /* zero if don't care about connectless secu */
                              BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_UNREG_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     U32 prot_id: protocol id.
 *     U32 chnl: channel.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_unreg_req(U32 prot_id,
                       U32 chnl,
                       BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_UNREG_OUTGOING_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U32 prot_id: protocol id.
 *     U32 rmt_chnl: remote channel.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_unreg_outgoing_req(BTS2S_BD_ADDR *bd,
                                U32 prot_id,
                                U32 rmt_chnl,
                                BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_ACCESS_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     U16 phdl
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U32 prot_id: protocol id.
 *     U32 chnl: channel.
 *     BOOL incoming: is incoming?
 *     U32 pv_context:
 *     U32 context:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_access_req(U16 phdl,
                        BTS2S_BD_ADDR *bd,
                        U32 prot_id,
                        U32 chnl,
                        BOOL incoming,
                        U32 pv_context,
                        U32 context,
                        BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_SET_SEC_MODE_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     U8 mode: mode.
 *     U8 bts2s_mode3_enc: mode 3 encryption.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_set_sec_mode_req(U8 mode,
                              U8 bts2s_mode_enc,
                              BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_ADD_DEV_REQ message.
 *     if pmsg != NULL, message is returned, not sent
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BOOL update_trust_level: update trust level.
 *     BOOL trusted: is trusted?
 *     BOOL update_link_key: update link key.
 *     U8 link_key[LINK_KEY_SIZE]: link key.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_add_dev_req(BTS2S_BD_ADDR *bd,
                         BOOL update_trust_level,
                         BOOL trusted,
                         BOOL update_link_key,
                         U8 link_key[LINK_KEY_SIZE],
                         BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_REMOVE_DEV_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_remove_dev_req(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_LINK_KEY_REQ_RES message.
 *     if pmsg != NULL, message is returned, not sent
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U8 key[LINK_KEY_SIZE]: link key.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message. duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_link_key_req_res(BTS2S_BD_ADDR *bd,
                              U8 key[LINK_KEY_SIZE], /* NULL to reject */
                              BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_PIN_REQ_RES message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U8 pin_len: pin length.
 *     U8 *p_pin: pointer to pin.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_pin_req_res(BTS2S_BD_ADDR *bd,
                         U8 pin_len,
                         U8 *p_pin,
                         BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_AUTHORISE_RES message.
 *     if pmsg != NULL, message is returned, not sent
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U32 prot_id: protocol id.
 *     U32 chnl: channel.
 *     BOOL incoming: is incoming?
 *     BOOL authorised: is authorised?
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_authorise_res(BTS2S_BD_ADDR *bd,
                           U32 prot_id,
                           U32 chnl,
                           BOOL incoming,
                           BOOL authorised,
                           BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_AUTH_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_auth_req(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_ENCRYPT_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BOOL encrypt: is encrypted?
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_encrypt_req(BTS2S_BD_ADDR *bd,
                         BOOL encrypt,
                         BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SM_L2C_REG_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     U16 phdl: handle.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sm_l2c_reg_req(U16 phdl, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_INQUIRY message.
 *     if pmsg != NULL, message is returned, not sent
 *
 *
 * INPUT:
 *     U24 lap: lap(BLUETOOTH address).
 *     U8 inquiry_len: inquiry length.
 *     U8 rsp_num: response number.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_inquiry(U24 lap,
                  U8 inquiry_len,
                  U8 rsp_num,
                  BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_INQUIRY_ESC message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 *
 * INPUT:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_inquiry_esc(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_PERIODIC_INQUIRY message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    U16 max_period_len: max period length.
 *    U16 min_period_len: min period length.
 *    U24 lap: part of the BLUETOOTH address.
 *    U8 inquiry_len: inquiry length.
 *    U8 rsp_num: response number.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_periodic_inquiry(U16 max_period_len,
                           U16 min_period_len,
                           U24 lap,
                           U8 inquiry_len,
                           U8 rsp_num,
                           BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_PERIODIC_INQUIRY message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_exit_periodic_inquiry(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_CHANGE_PKT_TYPE message for an ACL conn.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    U16 hdl: handle.
 *    U16 pkt_type: packet type.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_change_pkt_type_acl(U16 hdl, U16 pkt_type);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_CHANGE_PKT_TYPE message for a SCO connection.
 *
 * INPUT:
 *    U16 hdl: handle.
 *    U16 pkt_type: packet type.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_change_pkt_type_sco(U16 hdl, U16 pkt_type);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_CHANGE_LINK_KEY message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: pointer to BLUETOOTH address.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_change_link_key(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_CHANGE_LINK_KEY message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    U8 link_key_type: link key type.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_master_link_key(U8 link_key_type, /* 0 = regular link key, 1 = tmp link key */
                          BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RMT_NAME_REQ message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_rmt_name_req(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_RMT_FEATR message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_rd_rmt_featr(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_CREATE_CONN_ESC message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_create_conn_esc(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RMT_NAME_REQ_ESC message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_rmt_name_req_esc(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_RMT_EXT_FEATR message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    U8 page_num: page number.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_rd_rmt_ext_featr(BTS2S_BD_ADDR *bd,
                           U8 page_num,
                           BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_LMP_HANDLE message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     U16 hdl: handle.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_rd_lmp_hdl(U16 hdl, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_RMT_VERSION message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     U16 hdl: handle.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_rd_rmt_version(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_RD_CLOCK_OFFSET message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_rd_clock_offset(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_HOLD_MODE message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void hcia_hold_mode(BTS2S_BD_ADDR *bd,
                    U16 max_intvl,
                    U16 min_intvl,
                    BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_SNIFF_MODE message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U16 max_intvl: max interval.
 *     U16 min_intvl: min interval.
 *     U16 attmpt: attempt.
 *     U16 timeout: timeout time.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_sniff_mode(BTS2S_BD_ADDR *bd,
                     U16 max_intvl,
                     U16 min_intvl,
                     U16 attmpt,
                     U16 timeout,
                     BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_EXIT_SNIFF_MODE message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_exit_sniff_mode(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_QOS_SETUP_REQ message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U8 flag: flag.
 *     U8 svc_type: service type.
 *     U32 token_rate: token rate.
 *     U32 peak_bandwidth: peak bandwidth.
 *     U32 latency: latency.
 *     U32 delay_variation: delay variation.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_qos_setup_req(BTS2S_BD_ADDR *bd,
                        U8 flag, /* rsv */
                        U8 svc_type,
                        U32 token_rate, /* in bytes per second */
                        U32 peak_bandwidth, /* peak bandwidth in bytes per sec */
                        U32 latency, /* in microseconds */
                        U32 delay_variation, /* in microseconds */
                        BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_ROLE_DISCOV message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_role_discov(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_SWITCH_ROLE message.
 *    if pmsg != NULL, message is returned, not sent
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    U8 role: role.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_switch_role(BTS2S_BD_ADDR *bd, U8 role, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_LP_SETTINGS message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_lp_settings(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_LP_SETTINGS message, changing only the LM
 *    link policy settings.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_lp_settings(BTS2S_BD_ADDR *bd, U16 link_policy_setting, BTS2U_HCI_MSG **pmsg);

#ifdef CFG_OPEN_FULL_HCI_FUNC
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_LP_SETTINGS message, setting the HCI link
 *    policy to DM_LINK_POLICY_DEFAULT.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    U16 link_policy_setting: link policy setting.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_lp_settings_default(BTS2S_BD_ADDR *bd,
                                 U16 link_policy_setting,
                                 BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_LP_SETTINGS message, setting the HCI link
 *    policy to DM_LINK_POLICY_KEEP_ACT.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    U16 link_policy_setting: link policy setting.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_lp_settings_keep_act(BTS2S_BD_ADDR *bd,
                                  U16 link_policy_setting,
                                  BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_LP_SETTINGS message, setting the HCI link
 *    policy to DM_LINK_POLICY_KEEP_SNIFF.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    U16 link_policy_setting: link policy setting.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_lp_settings_keep_sniff(BTS2S_BD_ADDR *bd,
                                    U16 link_policy_setting,
                                    U16 max_intvl,
                                    U16 min_intvl,
                                    U16 attmpt,
                                    U16 timeout,
                                    BTS2U_HCI_MSG **pmsg);
#endif // CFG_OPEN_FULL_HCI_FUNC

void hcia_wr_lp_settings_keep_sniff_interval(BTS2S_BD_ADDR *bd,
        U16 link_policy_setting,
        U16 sniff_idle_time,
        U16 max_intvl,
        U16 min_intvl,
        U16 attmpt,
        U16 timeout,
        BTS2U_HCI_MSG **pmsg);

#ifdef CFG_OPEN_FULL_HCI_FUNC

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_LP_SETTINGS message, setting the HCI link
 *    policy to DM_LINK_POLICY_TRANSPARENT.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *    U16 link_policy_setting: link policy setting.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_lp_settings_transparent(BTS2S_BD_ADDR *bd,
                                     U16 link_policy_setting,
                                     BTS2U_HCI_MSG **pmsg);
#endif //CFG_OPEN_FULL_HCI_FUNC
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_default_link_policy_setting(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS message.
 *    If pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    U16 default_lps: default link policy.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_default_link_policy_setting(U16 default_lps, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_FLOW_SPEC message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    U16 default_lps: default link policy.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_flow_specification(U16 hdl,
                             BTS2S_BD_ADDR *bd,
                             U8 flag,
                             U8 flow_direction,
                             U8 svc_type,
                             U32 token_rate,
                             U32 token_bucket_size,
                             U32 peak_bandwidth,
                             U32 access_latency,
                             BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RESET message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_reset(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_SET_EV_FILTER message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U8 filter_type: filter type.
 *      U8 filter_condition_type: filter condition type.
 *      CONDITION *p_condition: condition.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_set_ev_filter(U8 filter_type,
                        U8 filter_condition_type,
                        CONDITION *p_condition,
                        BTS2U_HCI_MSG **pmsg);
void hcia_set_ev_msk(U32 ev_mask_low, U32 ev_mask_high,
                     BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_FLUSH message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_flush(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_PIN_TYPE message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_pin_type(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_PIN_TYPE message.
 *    if pmsg != NULL, message is returned, not sent
 *
 * INPUT:
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_pin_type(U8 pin_type, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_CREATE_NEW_UNIT_KEY message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_create_new_unit_key(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_STORED_LINK_KEY message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address, optional, can be NULL.
 *     U8 rd_all: read all.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_stored_link_key(BTS2S_BD_ADDR *bd, U8 rd_all, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_STORED_LINK_KEY message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *    U8 key_num: number of keys.
 *    LINK_KEYBD_ADDR *ap_link_key_bd[]: array of bmalloc LINK_KEY_BD_ADDR_T pointers.
 *    BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_stored_link_key(U8 key_num,
                             LINK_KEYBD_ADDR *ap_link_key_bd,
                             BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_DEL_STORED_LINK_KEY message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address,optional, can be NULL
 *     U8 flag: flag.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_del_stored_link_key(BTS2S_BD_ADDR *bd, U8 flag, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_CHANGE_LOCAL_NAME message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     U8 *sz_name: null - terminated name str.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_change_local_name(U8 *sz_name, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_LOCAL_NAME message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_local_name(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_CONN_ACPT_TO message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_conn_u8o(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_CONN_ACPT_TO message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_conn_u8o(U16 conn_acc_timeout, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_RD_PAGE_TO message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_page_to(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send a DM_HCI_WR_PAGE_TO message.
 *    if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_page_to(U16 page_timeout, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_RD_SCAN_ENB message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_scan_enb(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_WR_SCAN_ENB message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     U8 scan_enb: scan enable.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_scan_enb(U8 scan_enb, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_RD_PAGESCAN_ACTIVITY message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     U8 scan_enb: scan enabled.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_pagescan_activity(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_WR_PAGESCAN_ACTIVITY message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     U8 scan_enb: scan enabled.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_pagescan_activity(U16 pagescan_intvl,
                               U16 pagescan_window,
                               BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_HCI_RD_INQUIRYSCAN_ACTIVITY message.
 *     if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *     U8 scan_enb: scan enabled.
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_rd_inquiryscan_activity(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_INQUIRYSCAN_ACTIVITY message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U16 inqscan_intvl: inquiry scan interval.
 *      U16 inqscan_window: inquiry scan window.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_inquiryscan_activity(U16 inqscan_intvl,
                                  U16 inqscan_window,
                                  BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_AUTH_ENB message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_auth_enb(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_ENCRYPTION_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_encryption_mode(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_DEV_CLS message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_dev_cls(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_DEV_CLS message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U24 dev_cls: device class.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_dev_cls(U24 dev_cls, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_VOICE_SETTING message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_voice_setting(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_VOICE_SETTING message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U16 voice_setting: voice setting.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_voice_setting(U16 voice_setting, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_AUTO_FLUSH_TIMEOUT message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_auto_flush_timeout(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_AUTO_FLUSH_TIMEOUT message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      U16 timeout: time out(how long time will execute).
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_auto_flush_timeout(BTS2S_BD_ADDR *bd,
                                U16 timeout,
                                BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_NUM_BCAST_TXS message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_num_bcast_txs(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_NUM_BCAST_TXS message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_num_bcast_txs(U8 num, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      build and send a DM_HCI_RD_HOLD_MODE_ACTIVITY message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_hold_mode_activity(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_HOLD_MODE_ACTIVITY message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U8 activity: activity.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_hold_mode_activity(U8 activity, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_TX_POWER_LEVEL message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      U8 type: what type.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_tx_power_level(BTS2S_BD_ADDR *bd,
                            U8 type,
                            BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_SCO_FLOW_CTRL_ENB message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_sco_flow_ctrl_enb(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_SCO_FLOW_CTRL_ENB message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U8 enb
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_sco_flow_ctrl_enb(U8 enb, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_SET_HC_TO_HOST_FLOW message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U8 enb
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_set_hc_to_host_flow(U8 enb, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a HCI_HOST_NUM_COMP_PKT message for an
 *      ACL conn.
 *
 * INPUT:
 *      U8 num_hdls: handle numbers.
 *      BTS2S_HANDLE_COMP *ap_hdl_comps[]: completed application handles.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_host_num_compd_pkt_acl(U8 num_hdls, BTS2S_HANDLE_COMP *ap_hdl_comps[]);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a HCI_HOST_NUM_COMP_PKT message for an
 *      SCO conn.
 *
 * INPUT:
 *      U8 num_hdls: handle numbers.
 *      BTS2S_HANDLE_COMP *ap_hdl_comps[]: completed application handles.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_host_num_compd_pkt_sco(U8 num_hdls, BTS2S_HANDLE_COMP *ap_hdl_comps[]);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_LINK_SUPERV_TIMEOUT message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_link_superv_timeout(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_LINK_SUPERV_TIMEOUT message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_link_superv_timeout(BTS2S_BD_ADDR *bd,
                                 U16 timeout,
                                 BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_NUM_SUPP_IAC message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_num_supp_iac(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_CUR_IAC_LAP message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_cur_iac_lap(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_CUR_IAC_LAP message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_cur_iac_lap(U8 num_iac,
                         U24 *a_iacs,
                         BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_PAGESCAN_PERIOD_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_pagescan_period_mode(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_PAGESCAN_PERIOD_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_pagescan_period_mode(U8 mode, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_PAGESCAN_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_pagescan_mode(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_PAGESCAN_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *      U8 mode: what mode.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_pagescan_mode(U8 mode, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_SET_AFH_CHNL_CLS message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *      U8 mode: what mode.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_set_afh_chnl_cls(U8 *map, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_INQUIRY_SCAN_TYPE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_inquiry_scan_type(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_INQUIRY_SCAN_TYPE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_inquiry_scan_type(U8 mode, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_INQUIRY_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_inquiry_mode(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_INQUIRY_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *      U8 mode: what mode.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_inquiry_mode(U8 mode, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_PAGE_SCAN_TYPE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_page_scan_type(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_WR_PAGE_SCAN_TYPE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_page_scan_type(U8 mode, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a dm_hci_rd_afh_chnl_cls_m message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_afh_chnl_cls_m(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a dm_hci_wr_afh_chnl_cls_m message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_afh_chnl_cls_m(U8 cls_mode, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_LOCAL_VERSION message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_local_version(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *       Build and send a DM_HCI_RD_LOCAL_FEATR message.
 *       if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_local_featr(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *       Build and send a DM_HCI_RD_LOCAL_EXT_FEATR message.
 *       if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_local_ext_featr(U8 page_num, BTS2U_HCI_MSG **pmsg);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_BD_ADDR message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_bd(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_FAILED_CONTACT_COUNTER message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_failed_contact_counter(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RESET_CONTACT_COUNTER message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_reset_contact_counter(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_GET_LINK_QA message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_get_link_qa(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_RSSI message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_rssi(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_AFH_CHNL_MAP message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_afh_chnl_map(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_CLOCK message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U8 which_clock
 *      BTS2S_BD_ADDR *bd: BLUETOOTH address.
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_clock(U8 which_clock, BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_LOOPBACK_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_rd_loopback_mode(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_RD_LOOPBACK_MODE message.
 *      if pmsg != NULL, message is returned, not sent.
 *
 * INPUT:
 *      U8 mode
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_wr_loopback_mode(U8 mode, BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_HCI_ENB_DEV_UT_MODE message.
 *      if pmsg != NULL, message is returned, not sent
 *
 * INPUT:
 *      BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_enb_dev_ut_mode(BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      build and send a DM_SYNC_REG_REQ message.
 *
 * INPUT:
 *      U16 phdl: handle.
 *      U32 pv_cbarg:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_sync_reg_req(U16 phdl, U32 pv_cbarg);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      build and send a DM_SYNC_UNREG_REQ message.
 *
 * INPUT:
 *      U16 phdl: handle.
 *      U32 pv_cbarg:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_sync_unreg_req(U16 phdl, U32 pv_cbarg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send a DM_SYNC_CONN_REQ message.
 *
 * INPUT:
 *     U16 phdl: handle.
 *     U32 pv_cbarg:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     U32 tx_bdw: transfer bandwidth.
 *     U32 rx_bdw: receive bandwidth.
 *     U16 max_latency: max latency.
 *     U16 voice_setting: voice setting.
 *     U8 retx_effort: retransfer effort.
 *     U16 pkt_type: packet type.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_sync_conn_req(U16 phdl,
                        U32 pv_cbarg,
                        BTS2S_BD_ADDR *bd,
                        U32 tx_bdw,
                        U32 rx_bdw,
                        U16 max_latency,
                        U16 voice_setting,
                        U8 retx_effort,
                        U16 pkt_type);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SYNC_CONN_RES message.
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BOOL acpt: accept.
 *     U32 tx_bdw: transfer bandwidth.
 *     U32 rx_bdw: receiver bandwidth.
 *     U16 max_latency: max latency.
 *     U16 voice_setting: voice setting.
 *     U8 retx_effort: retransfer effort.
 *     U16 pkt_type: packet type.
 *     U8 reason: reason.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_sync_conn_res(BTS2S_BD_ADDR *bd,
                        BOOL acpt,
                        U32 tx_bdw,
                        U32 rx_bdw,
                        U16 max_latency,
                        U16 voice_setting,
                        U8 retx_effort,
                        U16 pkt_type,
                        U8 reason);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SYNC_RENEGOTIATE_REQ message.
 *
 * INPUT:
 *     U16 hdl: handle.
 *     U16 max_latency: max latency.
 *     U8 retx_effort: retransfer effort.
 *     U16 pkt_type: packet type.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_sync_renegotiate_req(U16 hdl,
                               U16 max_latency,
                               U8 retx_effort,
                               U16 pkt_type);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SYNC_RENEGOTIATE_REQ message.
 *
 * INPUT:
 *
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_sync_rfc_renegotiate_req(U16 sco_hdl,
                                   U16 audio_qa,
                                   U32 tx_bandwidth,
                                   U32 rx_bandwidth,
                                   U16 max_latency,
                                   U16 voice_setting,
                                   U8 re_tx_effort);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SYNC_DISC_REQ message.
 *
 * INPUT:
 *     U16 hdl: handle.
 *     U8 reason: reason.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_sync_disc_req(U16 hdl, U8 reason);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a DM_SYNC_DISC_REQ message.
 *
 * INPUT:
 *     BTS2U_HCI_MSG *umsg: union, all kinds of HCI messages.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/
void hcia_free_umsg_msg(BTS2U_HCI_MSG *umsg);

void hcia_register_receive_earphone_connect_req_handler(BOOL (*cb)(BTS2S_BD_ADDR *p_bd));

void hcia_set_sniff_mode_enable(BOOL enable);
BOOL hcia_get_sniff_mode_enable(void);

void hcia_set_utest_mode_enable(BOOL enable);
BOOL hcia_get_utest_mode_enable(void);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *     BTS2S_BD_ADDR *bd: remote BLUETOOTH address.
 *     BTS2E_IO_CAPABILITY io_capability: io capability
 *     BTS2E_DM_OOB_DATA_PRESENT oob_data_present:
 *     BTS2E_DM_IO_AUTH_REQUIRE auth_require:
 *     BTS2U_HCI_MSG **pmsg: duple pointer to message.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_hci_io_capability_request_reply(BTS2S_BD_ADDR *bd,
        BTS2E_IO_CAPABILITY io_capability,
        BTS2E_DM_OOB_DATA_PRESENT oob_data_present,
        BTS2E_DM_IO_AUTH_REQUIRE auth_require,
        BTS2U_HCI_MSG **pmsg);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void hcia_wr_ext_inpuire_resp(U8 uflag, U8 *eir_data, U32 len);

void hcia_hci_io_capability_req_neg_reply(BTS2S_BD_ADDR *bd, U8 reason, BTS2U_HCI_MSG **pmsg);

void hcia_hci_user_cfm_req_reply(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

void hcia_hci_user_cfm_req_neg_reply(BTS2S_BD_ADDR *bd, BTS2U_HCI_MSG **pmsg);

void bt_acl_accept_role_set(U8 role);
void bt_acl_set_default_link_policy(U16 lp_in, U16 lp_out);

// Just default parameter, could change via hcia_wr_pagescan_activity or hcia_wr_pagescan_mode
void hcia_set_default_pscan_efficient(U8 is_quick_pscan);
U8 hcia_get_default_pscan_efficient(void);

// Just default parameter, could change via hcia_wr_inquiryscan_activity
void hcia_set_default_iscan_efficient(U8 is_quick_iscan);
U8 hcia_get_default_iscan_efficient(void);




#ifdef __cplusplus
}
#endif

#endif


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
