/**
  ******************************************************************************
  * @file   pbap_clt_api.h
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

#ifndef _PBAP_CLT_API_H_
#define _PBAP_CLT_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#define PBAPC_MAX_PASSWORD_LEN           0x20
#define PBAPC_MAX_PASSWORD_USERID_LEN    0x20

enum
{
    BTS2MU_PBAP_CLT_CONN_CFM = BTS2MU_START,
    BTS2MU_PBAP_CLT_DISC_IND,
    BTS2MU_PBAP_CLT_AUTH_IND,
    BTS2MU_PBAP_CLT_ABORT_CFM,

    BTS2MU_PBAP_CLT_SET_PB_CFM,
    BTS2MU_PBAP_CLT_PULL_PB_BEGIN_IND,
    BTS2MU_PBAP_CLT_PULL_PB_NEXT_IND,
    BTS2MU_PBAP_CLT_PULL_PB_COMPLETE_IND,
    BTS2MU_PBAP_CLT_PULL_VCARD_BEGIN_IND,
    BTS2MU_PBAP_CLT_PULL_VCARD_NEXT_IND,
    BTS2MU_PBAP_CLT_PULL_VCARD_COMPLETE_IND,
    BTS2MU_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND,
    BTS2MU_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND,
    BTS2MU_PBAP_CLT_PULL_VCARD_LIST_COMPLETE_IND
};

typedef enum
{
    PBAPC_SUCCESS = 0,
    PBAPC_FAIL,
    PBAPC_ABORT,
    PBAPC_NOT_IDLE,
    PBAPC_WRONG_STATE,
    PBAPC_SDP_FAIL,
    PBAPC_REMOTE_DISCONNECT,
    PBAPC_UNAUTHORISED,
    PBAPC_PULL_BADREQUEST,
    PBAPC_PULL_FORBIDDEN,
    PBAPC_SPB_NO_REPOSITORY,
    PBAPC_SPB_NOT_FOUND,
    PBAPC_VCL_NO_PARAM_RESOURCES,
    PBAPC_VCL_NO_PBOOK_FOLDER,
    PBAPC_VCL_INVALID_PBOOK,
    PBAPC_VCE_NO_PARAM_RESOURCES,
    PBAPC_VCE_NO_NAME_RESOURCES,
    PBAPC_PPB_NO_PARAM_RESOURCES,
    PBAPC_PPB_NO_NAME_RESOURCES,
    PBAPC_PPB_NO_REQUIRED_NAME,
    PBAPC_PPB_NO_REPOSITORY
} BTS2E_PBAPC_RESULT_CODE;

typedef struct
{
    U16          type;          /* message identity */
    U16          mfs;           /* maximum size of packet transferable during this connection */
    U8           res;           /* connection result */
    U8           supp_repos;    /* remote side supported repositories */
    BTS2S_BD_ADDR bd;           /* remote side Bluetooth address */
} BTS2S_PBAP_CLT_CONN_CFM;

typedef struct
{
    U16 type;                   /* message identity */
    U8 res;                     /* disconnection reason */
    BTS2S_BD_ADDR bd;
} BTS2S_PBAP_CLT_DISC_IND;

typedef struct
{
    U16 type;                   /* message identity */
} BTS2S_PBAP_CLT_AUTH_IND;

typedef struct
{
    U16 type;                   /* message identity */
    U8  res;                    /* abort reason */
} BTS2S_PBAP_CLT_ABORT_CFM;

typedef struct
{
    U16 type;                   /* BTS2MU_PBAP_CLT_SET_PB_CFM message identity */
    U8 res;                     /* operation result */
    BTS2S_BD_ADDR bd;
} BTS2S_PBAP_CLT_SET_PB_CFM;

typedef struct
{
    U16 type;                   /* message identity */
    U16 pbook_size;             /* the number of indexes in the device phone book */
    U32 totalLength;            /* the data total length */
    U16 dataLen;                /* the length of payload */
    U16 dataOffset;             /* data offset from the packet */
    BOOL more_data;             /* is last packet */
    U8 new_missed;              /* the number of missed calls */
    U8 *data;                   /* payload */
    BTS2S_BD_ADDR bd;
} BTS2S_PBAP_CLT_PULL_PB_BEGIN_IND;

typedef struct
{
    U16 type;                   /* message identity */
    U16 dataLen;                /* the data total length */
    U16 dataOffset;             /* data offset from the packet */
    BOOL more_data;             /* is last packet */
    U8 *data;                   /* payload */
} BTS2S_PBAP_CLT_PULL_PB_NEXT_IND;

typedef struct
{
    U16 type;                   /* message identity */
    U32 totalLength;            /* the data total length */
    U16 dataLen;                /* the length of data */
    U16 dataOffset;             /* the length of payload */
    BOOL more_data;             /* is last packet */
    U8 *data;                   /* payload */
    BTS2S_BD_ADDR bd;
} BTS2S_PBAP_CLT_PULL_VCARD_BEGIN_IND;

typedef struct
{
    U16 type;                   /* message identity */
    U16 dataLen;                /* the length of data  */
    U16 dataOffset;             /* the length of payload */
    BOOL more_data;             /* is last packet */
    U8 *data;                   /* payload */
} BTS2S_PBAP_CLT_PULL_VCARD_NEXT_IND;

typedef struct
{
    U16 type;                   /* message identity */
    U16 pbook_size;             /* the number of indexes in the device phone book */
    U32 totalLength;            /* the data total length */
    U16 dataLen;                /* the length of data */
    U16 dataOffset;             /* the length of payload */
    BOOL more_data;             /* is last packet */
    U8 new_missed;              /* the number of missed calls */
    U8 *data;                   /* payload */
    BTS2S_BD_ADDR bd;
} BTS2S_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND;

typedef struct
{
    U16 type;                   /* message identity */
    U16 dataLen;                /* the length of data */
    U16 dataOffset;             /* the length of payload */
    BOOL more_data;             /* is last packet */
    U8 *data;                   /* payload */
} BTS2S_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND;

typedef struct
{
    U16 type;                   /* message identity */
    U8  res;                    /* result code */
    BTS2S_BD_ADDR bd;
} BTS2S_PBAP_CLT_PULL_CMPT_IND;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Initiate a connect towards the peer device.
 *
 * INPUT:
 *      U16 max_pkt_size: indicates the maximum packet size that local device can receive
 *      BOOL auth_flag: whether or not authenticate remote device.
 *      const BTS2S_BD_ADDR *bd: destination Bluetooth address.
 *      const U8 *pass_word: authenticate password, use if auth_flag set.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_CONN_CFM with structure BTS2S_PBAP_CLT_CONN_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_conn_req(U16 max_pkt_size,
                       BOOL auth_flag,
                       BTS2S_BD_ADDR *bd,
                       U8 *pass_word);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send disconnect command with the server.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_DISC_CFM with structure
 *      BTS2S_PBAP_CLT_DISC_CFM will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_disc_req(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      PBAP client send authentication response.
 *
 * INPUT:
 *      const U8 *rsp_password: a pointer to the response password.
 *      U16 rsp_password_len: response password length.
 *      const char *rsp_user_id: user id.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_AUTH_IND with structure BTS2S_PBAP_CLT_AUTH_IND
 *      will be received as a hint to call this function.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_auth_rsp(const U8 *rsp_password,
                       U16 rsp_password_len,
                       const char *rsp_user_id);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request to abort the current multi-packet operation.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_ABORT_CFM with structure
 *      BTS2S_PBAP_CLT_ABORT_CFM will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_abort_req(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Change the current active phonebook.
 *
 * INPUT:
 *      U8 repository: phone book repositories, see BTS2E_PBAP_PHONE_REPOSITORY
 *                     in pbap_public.h.
 *      U8 phonebook: phone book objects, see BTS2E_PBAP_PHONE_BOOK in pbap_public.h.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      1. To change to a different phonebook in the same repository,
 *         param repository may use pbap_current.
 *      2. BTS2S_PBAP_CLT_SET_PB_CFM message will be received by the application.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_set_pb_req(U8 repository, U8 phonebook);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Retrieve an entire phone book object from the specific repositories.
 *
 * INPUT:
 *      U8 repository: phone book repositories, see BTS2E_PBAP_PHONE_REPOSITORY
 *                     in pbap_public.h.
 *      U8 phonebook:  phone book objects.
 *      U32 filter_lo: PBAP Parameter filter mask low 32 bits. see pbap_public.h.
 *                     Use zero to not sent the parameter.
 *      U32 filter_hi: PBAP Parameter filter mask high 32 bits.
 *                     Use zero to not sent the parameter.
 *      U8 format: indicate the requested format (vCard 2.1 or 3.0) to be returned
 *                 in the operation. The format vCard 2.1 shall be the default format.
 *      U16 max_list:  indicate the maximum number of entries that the PCE can handle.
 *      U16 list_start_offset: the offset of the first entry of the phonebook, the
 *      first entry of the Phonebook object that would be returned if this parameter
 *      was not specified in the request.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_PULL_PB_BEGIN_IND with structure
 *      BTS2S_PBAP_CLT_PULL_PB_BEGIN_IND will be received by the application
 *      when the first packet arrives.
 *
 *      Message BTS2MU_PBAP_CLT_PULL_PB_COMPLETE_IND with structure
 *      BTS2S_PBAP_CLT_PULL_CMPT_IND will be received after the last packet.
 *
 *      If remote side reject our request, application will receive message
 *      BTS2MU_PBAP_CLT_PULL_PB_COMPLETE_IND only.
 *
 *      If param max_list is 0 and the pbook is MCH, then NewMissedCalls will
 *      be returned in structure BTS2S_PBAP_CLT_PULL_CMPT_IND.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_pull_pb_req(U8 repository,
                          U8 phonebook,
                          U32 filter_lo,
                          U32 filter_hi,
                          U8 format,
                          U16 max_list,
                          U16 list_start_offset);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Start to get a specific vCard from the current phonebook.
 *
 * INPUT:
 *
 *      U8 *vcard_name: VCARD name(*.vcf) is a null terminated Unicode text string describing
 *                      the name of the object.
 *      U32 filter_lo: PBAP Parameter filter mask low 32 bits.
 *                     Use zero to not sent the parameter. see pbap_public.h.
 *      U32 filter_hi: PBAP Parameter filter mask high 32 bits.
 *                     Use zero to not sent the parameter. see pbap_public.h.
 *      U8 format: indicate the requested format (vCard 2.1 or 3.0) to be returned in the operation.
 *                 The format vCard 2.1 shall be the default format.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      1. If the filter does not include the mandatory fields as required
 *         by the specified format, these will be added.
 *
 *      2. Message BTS2MU_PBAP_CLT_PULL_VCARD_BEGIN_IND with structure
 *         BTS2S_PBAP_CLT_PULL_VCARD_BEGIN_IND will be received by the application
 *         when the first packet arrives.
 *
 *      3. Message BTS2MU_PBAP_CLT_PULL_VCARD_COMPLETE_IND message with structure
 *         BTS2S_PBAP_CLT_PULL_CMPT_IND will be received after the last packet arrived.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_pull_vcard_req(U8 *vcard_name,
                             U32 filter_lo,
                             U32 filter_hi,
                             U8 format);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Start to get the vCard listing for the current phonebook.
 *
 * INPUT:
 *      U8 order: indicate to the Server, which sorting order shall be used.
 *                Use 'pbap_order_default' for default order, the default order is "Indexed".
 *      U8 pbook: PhoneBook folder to retrieve. Use 'pbap_b_unknown' for the current folder.
 *      U8 srchAttr: Attribute to search. Use 'pbap_a_unknown' for default search attribute.
 *      U8 srchVal: a UTF-8 text string, which matches the value of the attribute indicated
 *                  using the Search/Attribute.
 *                  All the vCards shall be returned if this header is not specified.
 *      U16 size_srchVal: Length of the search value.
 *      U16 maxList: Maximum number of entries that the PCE can handle.
 *      U16 list_start_offset: the offset of the first entry of the phonebook, the first entry of the Phonebook
 *      object that would be returned if the ListStartOffset parameter was not specified in the request.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND with structure
 *      BTS2S_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND will be received by the application
 *      when the first packet arrives.
 *
 *      Message BTS2MU_PBAP_CLT_PULL_VCARD_LIST_COMPLETE_IND with structure
 *      BTS2S_PBAP_CLT_PULL_CMPT_IND will be received after the last packet.
 *
 *      If remote side reject our request, application will receive message
 *      BTS2MU_PBAP_CLT_PULL_PB_COMPLETE_IND only.
 *
 *      If param max_list is 0 and the pbook is MCH, then NewMissedCalls will
 *      be returned in structure BTS2S_PBAP_CLT_PULL_CMPT_IND.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_pull_vcard_list_req(U8 order,
                                  U8 pbook,
                                  U8 srchAttr,
                                  U8 *srchVal,
                                  U16 size_srchVal,
                                  U16 max_list,
                                  U16 list_start_offset);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *       Get the next vCard listing packet for the current phonebook.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_PULL_PB_NEXT_IND with structure
 *      BTS2S_PBAP_CLT_PULL_PB_NEXT_IND will be received as a confirmation if
 *      continuation packets exist.
 *
 *      Message BTS2MU_PBAP_CLT_PULL_PB_COMPLETE_IND with structure
 *      BTS2S_PBAP_CLT_PULL_CMPT_IND will be received after the last packet arrived.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_pull_pb_next_req(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Get the next vCard entry packet from the current phonebook.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_PULL_VCARD_NEXT_IND with structure
 *      BTS2S_PBAP_CLT_PULL_VCARD_NEXT_IND will be received by the application
 *      for each packet after the first.
 *
 *      Message BTS2MU_PBAP_CLT_PULL_VCARD_COMPLETE_IND with structure
 *      BTS2S_PBAP_CLT_PULL_CMPT_IND will be received after the last packet arrived.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_pull_vcard_next_req(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Get the next vCard listing packet for the current phonebook.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND with structure
 *      BTS2S_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND will be received by the application
 *      for each packet after the first.
 *
 *      Message BTS2MU_PBAP_CLT_PULL_VCARD_LIST_COMPLETE_IND with structure
 *      BTS2S_PBAP_CLT_PULL_CMPT_IND will be received after the last packet arrived.
 *
 *----------------------------------------------------------------------------*/
void pbap_clt_pull_vcard_list_next_req(void);

U16 pbap_clt_get_max_mtu(void);


#ifdef __cplusplus
}
#endif

#endif





/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
