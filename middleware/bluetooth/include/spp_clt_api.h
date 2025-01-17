/**
  ******************************************************************************
  * @file   spp_clt_api.h
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

#ifndef _SPP_CLT_API_H_
#define _SPP_CLT_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DCE
#define DCE                                      (1)
#define DTE                                      (2)
#endif

#ifndef SPP_CLT_ROLE
#define SPP_CLT_ROLE                    (DTE)
#define SPP_SRV_ROLE                    (DCE)
#endif

#ifndef MDM_CTS_MASK
#define MDM_CTS_MASK                    0x01
#define MDM_RTS_MASK                    0x02
#define MDM_DSR_MASK                    0x04
#define MDM_DTR_MASK                    0x08
#define MDM_RI_MASK                     0x10
#define MDM_DCD_MASK                    0x20
#endif

enum
{
    BTS2MD_SPP_CLT_CONN_REQ = BTS2MD_START,
    BTS2MD_SPP_CLT_DISC_REQ,
    BTS2MD_SPP_CLT_DATA_REQ,
    BTS2MD_SPP_CLT_DATA_RSP,
    BTS2MD_SPP_CLT_TEST_REQ,
    BTS2MD_SPP_CLT_CTRL_REQ,
    BTS2MD_SPP_CLT_PORT_NEG_REQ,
    BTS2MD_SPP_CLT_LINE_ST_REQ,
    BTS2MD_SPP_CLT_MODE_CHANGE_REQ,

    BTS2MU_SPP_CLT_CONN_CFM = BTS2MU_START,
    BTS2MU_SPP_CLT_DISC_CFM,
    BTS2MU_SPP_CLT_DATA_CFM,
    BTS2MU_SPP_CLT_DATA_IND,
    BTS2MU_SPP_CLT_CTRL_IND,
    BTS2MU_SPP_CLT_PORTNEG_IND,
    BTS2MU_SPP_CLT_MODE_CHANGE_IND
};

#define BTS2MD_SPP_CLT_MSG_NUM (BTS2MD_SPP_CLT_MODE_CHANGE_REQ - BTS2MD_START + 1)

typedef struct
{
    U16 type;
    U8 rmt_srv_chnl;
    BOOL vld_port_par;
    BOOL req_port_par;
    BTS2S_BD_ADDR bd;
    BTS2S_PORT_PAR port_par;
} BTS2S_SPP_CLT_CONN_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 rmt_srv_chnl;
    U8 res;
    U16 mfs;
    BOOL vld_port_par;
    BTS2S_BD_ADDR bd;
    BTS2S_PORT_PAR port_par;
} BTS2S_SPP_CLT_CONN_CFM;

typedef struct
{
    U16 type;
    U8 spp_id;
    U16 payload_len;
    U8  *payload;
} BTS2S_SPP_CLT_DATA_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U16 credit;
} BTS2S_SPP_CLT_DATA_CFM;

typedef struct
{
    U16 type;
    U8 spp_id;
    U16 payload_len;
    U8 *payload;
} BTS2S_SPP_CLT_DATA_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
} BTS2S_SPP_CLT_DATA_RSP;

typedef struct
{
    U16 type;
    U8 spp_id;
    U16 test_data_len;
    U8 *test_data;
} BTS2S_SPP_CLT_TEST_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 mdm_st;
    U8 break_sig;
} BTS2S_SPP_CLT_CTRL_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 mdm_st;
    U8 break_sig;
} BTS2S_SPP_CLT_CTRL_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
    BOOL req;
    BTS2S_PORT_PAR *port_par;
} BTS2S_SPP_CLT_PORT_NEG_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    BOOL req;
    BTS2S_PORT_PAR port_par;
} BTS2S_SPP_CLT_PORTNEG_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 line_st;
    BOOL err_flag;
} BTS2S_SPP_CLT_LINE_ST_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 mode;
} BTS2S_SPP_CLT_MODE_CHANGE_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 res;
    U8 mode;
} BTS2S_SPP_CLT_MODE_CHANGE_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
} BTS2S_SPP_CLT_DISC_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 res;
    BTS2S_BD_ADDR bd;
} BTS2S_SPP_CLT_DISC_CFM;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      SPP client send a connect request to remote device which Bluetooth Address
 *      is bd.
 *
 * INPUT:
 *      bd: Remote device BD to be connected.
 *      rmt_srv_chnl: Remote device server channel,default 0xFF
 *      req_port_par: TRUE or FALSE
 *                    TRUE: Connect with user's port parameter
 *                    FALSE: Connect with BTS2 default port parameter.
 *      port_par: Port parameter
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      If req_port_par is FALSE,then port_par will be NULL.
 *      Message BTS2MU_SPP_CLT_CONN_CFM with structure BTS2S_SPP_CLT_CONN_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void spp_clt_conn_req(BTS2S_BD_ADDR *bd,
                      U8 rmt_srv_chnl,
                      BOOL req_port_par,
                      BTS2S_PORT_PAR *port_par);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send a disconnect request to remote device.
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SPP_CLT_DISC_CFM with structure BTS2S_SPP_CLT_DISC_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void spp_clt_disc_req(U8 spp_id);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send data to remote device
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      payload_len: Number of data bytes in data area
 *      payload: Pointer to allocated data
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SPP_CLT_DATA_CFM with structure BTS2S_SPP_CLT_DATA_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void spp_clt_data_req(U8 spp_id,
                      U16 payload_len,
                      U8 *payload);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      It is indicated that the application is ready to receive next piece of data
 *
 * INPUT:
 *      none.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void spp_clt_data_rsp(U8 spp_id);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     This function is used to send a test command to remote device
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      test_data_len: Number of data bytes in data area
 *      test_data: Pointer to allocated data
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void spp_clt_test_req(U8 spp_id,
                      U16 test_data_len,
                      U8 *test_data);
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send modem status information to remote device.
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      mdm_st: modem status.
 *      break_sig: modem break signal.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *
 *----------------------------------------------------------------------------*/
void spp_clt_ctrl_req(U8 spp_id,
                      U8 mdm_st,
                      U8 break_sig);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send port negotiation request to remote device.
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      req: TRUE or FAULSE
 *           TURE means using user port setting
 *           FAULSE means using BTS2 default port setting
 *      port_part: Port parameter
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *
 *
 *----------------------------------------------------------------------------*/
void spp_clt_port_neg_req(U8 spp_id,
                          BOOL req,
                          BTS2S_PORT_PAR *port_par);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used for indicate remote port line status.
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      err_flag: Error flag
 *      line_st: Line status
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void spp_clt_line_st_req(U8 spp_id,
                         BOOL err_flag,
                         U8 line_st);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Change current SPP mode.
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      mode: One of ACT_MODE, HOLD_MODE, SNIFF_MODE and PARK_MODE.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *
 *----------------------------------------------------------------------------*/
void spp_clt_mode_change_req(U8 spp_id, U8 mode);

#ifdef __cplusplus
}
#endif


#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
