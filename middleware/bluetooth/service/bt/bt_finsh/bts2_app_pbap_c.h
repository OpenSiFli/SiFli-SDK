/**
  ******************************************************************************
  * @file   bts2_app_pbap_c.h
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

#ifndef _BTS2_APP_PBAP_CLT_H_
#define _BTS2_APP_PBAP_CLT_H_


#ifdef __cplusplus
extern "C" {
#endif

#ifdef CFG_PBAP_CLT

#define PBAP_VCARD_NAME_LEN         (0x20)

#define PBAP_CLT_PATH_LEN   (sizeof("test/pbap/sim1/tetecom/ich/") + 15)

/* PBAP Client Currently running Command */
typedef enum
{
    BT_PBAP_CLT_IDLE,
    BT_PBAP_CLT_SETPHONEBOOK,
    BT_PBAP_CLT_PULLVCARDLIST,
    BT_PBAP_CLT_PULLVCARD,
    BT_PBAP_CLT_PULLPHONEBOOK,
    BT_PBAP_CLT_END
} BT_PBAP_CLT_RUN_CMD;


typedef enum
{
    BT_PBAP_CLT_VCARD_IDLE,
    BT_PBAP_CLT_FN,
    BT_PBAP_CLT_N,
    BT_PBAP_CLT_PHOTO,
    BT_PBAP_CLT_BDAY,
    BT_PBAP_CLT_ADR,
    BT_PBAP_CLT_LABEL,
    BT_PBAP_CLT_TEL,
    BT_PBAP_CLT_EMAIL,
    BT_PBAP_CLT_TZ,
    BT_PBAP_CLT_GEO,
    BT_PBAP_CLT_TITLE,
    BT_PBAP_CLT_ROLE,
    BT_PBAP_CLT_LOGO,
    BT_PBAP_CLT_MAX = 0xff
} BT_PBAP_CLT_VCARD_MEMBER;


typedef enum
{
    BT_PBAP_ELEM_VCARD_IDLE = 0x00,
    BT_PBAP_ELEM_FN         = 0x01,
    BT_PBAP_ELEM_TEL        = 0x02,
    BT_PBAP_ELEM_MAX        = 0xff
} BT_PBAP_ELEM_VCARD;



typedef struct
{
    char *name;
    U8 length;
} vcard_name;


typedef struct
{
    void *next_struct;
    char *tel;
    U8 length;
    U8 num;
} vcard_tel_list;


typedef struct
{
    vcard_name v_name;
    vcard_tel_list *v_tel;
} BT_PBAP_VCARD;

struct pbap_key_value_t
{
    const char *value_start;
    const char *value_end;
};

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Free global instance.
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
void bt_pbap_clt_free_inst();
void bt_pbap_clt_init(bts2_app_stru *bts2_app_data);
void bt_pbap_clt_rel(bts2_app_stru *bts2_app_data, void *msg_data);
void bt_pbap_clt_hdl_msg(bts2_app_stru *bts2_app_data);
void bt_pbapc_hdl_vcardlist(BTS2S_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND *msg);

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
bt_err_t bt_pbap_client_connect(BTS2S_BD_ADDR *bd, BOOL auth_flag);
bt_err_t bt_pbap_client_disconnect(BTS2S_BD_ADDR *bd);
bt_err_t bt_pbap_client_pull_pb(BTS2E_PBAP_PHONE_REPOSITORY repos, U8 phone_book, U8 max_size);
bt_err_t bt_pbap_client_set_pb(BTS2E_PBAP_PHONE_REPOSITORY repos, U8 phone_book);
bt_err_t bt_pbap_client_pull_vcard_list(bts2_app_stru *bts2_app_data);
bt_err_t bt_pbap_client_pull_vcard(U8 *p, U8 len);
bt_err_t bt_pbap_client_get_name_by_number(char *phone_number, U16 phone_len);
bt_err_t bt_pbap_client_auth(U8 *password, U8 len);

#endif
#ifdef __cplusplus
}
#endif
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
