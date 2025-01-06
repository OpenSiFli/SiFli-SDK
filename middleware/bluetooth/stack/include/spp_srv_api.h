/**
  ******************************************************************************
  * @file   spp_srv_api.h
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

#ifndef _SPP_SRV_API_H_
#define _SPP_SRV_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DCE
#define DCE                             (1)
#define DTE                             (2)
#endif




#define SPP_DEBUG    BT_DBG_D


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

#define    SPP_DISC_INIT_NUM              0x00
#define    SPP_DISC_APP_NUM               0x01
#define    SPP_DISC_TO_NUM                0x02


#define    SPP_NO_ENABLE_ST                    0x00
#define    SPP_ENABLE_PENDING_ST               0x01
#define    SPP_ENABLED_ST                      0x02


typedef enum
{
    SPP_OK = 0,
    /* general error code */
    SPP_ERROR_INPARAM            = 0x10000001,
    SPP_ERROR_UNSUPPORTED        = 0x10000002,
} spp_err_t;

enum
{
    BTS2MD_SPP_SRV_ENB_REQ = BTS2MD_START,
    BTS2MD_SPP_SRV_DISB_REQ,
    BTS2MD_SPP_SRV_CONN_RSP,
    BTS2MD_SPP_SRV_DISC_REQ,
    BTS2MD_SPP_SRV_DATA_REQ,
    BTS2MD_SPP_SRV_DATA_RSP,
    BTS2MD_SPP_SRV_TEST_REQ,
    BTS2MD_SPP_SRV_CTRL_REQ,
    BTS2MD_SPP_SRV_PORTNEG_REQ,
    BTS2MD_SPP_SRV_LINE_ST_REQ,
    BTS2MD_SPP_SRV_MODE_CHANGE_REQ,

    BTS2MD_SPP_SRV_DISC_REQ_EXT,
    BTS2MD_SPP_SRV_DATA_REQ_EXT,
    BTS2MD_SPP_SRV_DATA_RSP_EXT,
    BTS2MD_SPP_SRV_TEST_REQ_EXT,
    BTS2MD_SPP_SRV_CTRL_REQ_EXT,
    BTS2MD_SPP_SRV_PORTNEG_REQ_EXT,
    BTS2MD_SPP_SRV_LINE_ST_REQ_EXT,
    BTS2MD_SPP_SRV_MODE_CHANGE_REQ_EXT,

    //New messages must be added before this
    BTS2MD_SPP_SRV_MAX_MSG_NUM,

    BTS2MU_SPP_SRV_ENB_CFM = BTS2MU_START,
    BTS2MU_SPP_SRV_DISB_CFM,
    BTS2MU_SPP_SRV_CONN_IND,
    BTS2MU_SPP_SRV_CONN_CFM,
    BTS2MU_SPP_SRV_DISC_CFM,
    BTS2MU_SPP_SRV_DATA_CFM,
    BTS2MU_SPP_SRV_DATA_IND,
    BTS2MU_SPP_SRV_CTRL_IND,
    BTS2MU_SPP_SRV_PORTNEG_IND,
    BTS2MU_SPP_SRV_MODE_CHANGE_IND
};

#define BTS2MD_SPP_SRV_MSG_NUM          (BTS2MD_SPP_SRV_MAX_MSG_NUM - BTS2MD_START)
#define SPP_SRV_SVC_NAME_MAX_LEN        (50)



typedef struct
{
    U8                      srv_chl;
    U8                      st;
    U8                      uuid_len;
    BOOL                    sds_unreg_in_progress;
    BOOL                    bts2s_svc_record_ext;
    BOOL                    sds_record_obtain;
    U16                     retry_num;
    U16                     bts2s_svc_record_size;
    U16                     bts2s_svc_record_srv_ch_idx;
    char                    *srv_name;
    U8                      *bts2s_svc_record;
    U32                     sds_rec_hdl;
    void                    *next_struct;
    U8                      *uuid;
} BTS2S_SPP_UUID_LIST_EXT;

typedef struct
{
    U16 type;
} BTS2S_SPP_SRV_RESET;

typedef struct
{
    U16 type;
    U8  spp_id;
    U16 timeout;
    char svc_name[SPP_SRV_SVC_NAME_MAX_LEN + 1];
} BTS2S_SPP_SRV_ENB_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 res;
} BTS2S_SPP_SRV_ENB_CFM;

typedef struct
{
    U16 type;
    U8 spp_id;
} BTS2S_SPP_SRV_DISB_REQ;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 res;
} BTS2S_SPP_SRV_DISB_CFM;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 srv_chnl;
    BTS2S_BD_ADDR bd;
} BTS2S_SPP_SRV_CONN_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8  res;
    U8  srv_chnl;
    BTS2S_BD_ADDR bd;
} BTS2S_SPP_SRV_CONN_RSP;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 res;
    U8 srv_chnl;
    BTS2S_BD_ADDR bd;
    U16 mfs;
    BOOL vld_port_par;
    BTS2S_PORT_PAR port_par;
    U8 *uuid;
    U8 uuid_len;
} BTS2S_SPP_SRV_CONN_CFM;

typedef struct
{
    U16 type;
    U8 spp_id;
    U16 payload_len;
    U8 *payload;
} BTS2S_SPP_SRV_DATA_REQ;


typedef struct
{
    U16 type;
    U8  device_id;
    U8  srv_chl;
    U16 payload_len;
    U8 *payload;
} BTS2S_SPP_SRV_DATA_REQ_EXT;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 credit;
    U8 device_id;
    U8 srv_chl;
    U8 *uuid;
    U8 uuid_len;
} BTS2S_SPP_SRV_DATA_CFM;

typedef struct
{
    U16 type;
    U8 spp_id;
    U16 payload_len;
    U8 *payload;
    U8 device_id;
    U8 srv_chl;
    U8 *uuid;
    U8 uuid_len;
} BTS2S_SPP_SRV_DATA_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
} BTS2S_SPP_SRV_DATA_RSP;


typedef struct
{
    U16 type;
    U8 device_id;
    U8 srv_chl;
} BTS2S_SPP_SRV_DATA_RSP_EXT;

typedef struct
{
    U16 type;
    U8 spp_id;
    U16 test_data_len;
    U8 *test_data;
} BTS2S_SPP_SRV_TEST_REQ;


typedef struct
{
    U16 type;
    U8 device_id;
    U8 srv_chl;
    U16 test_data_len;
    U8 *test_data;
} BTS2S_SPP_SRV_TEST_REQ_EXT;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 mdm_st;
    U8 break_sig;
} BTS2S_SPP_SRV_CTRL_REQ;


typedef struct
{
    U16 type;
    U8 device_id;
    U8 srv_chl;
    U8 mdm_st;
    U8 break_sig;
} BTS2S_SPP_SRV_CTRL_REQ_EXT;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 mdm_st;
    U8 break_sig;
} BTS2S_SPP_SRV_CTRL_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
    BOOL req;
    BTS2S_PORT_PAR *port_par;
} BTS2S_SPP_SRV_PORTNEG_REQ;


typedef struct
{
    U16 type;
    U8 device_id;
    U8 srv_chl;
    BOOL req;
    BTS2S_PORT_PAR *port_par;
} BTS2S_SPP_SRV_PORTNEG_REQ_EXT;

typedef struct
{
    U16 type;
    U8 spp_id;
    BTS2S_PORT_PAR port_par;
    BOOL req;
} BTS2S_SPP_SRV_PORTNEG_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 line_st;
    BOOL err_flag;
} BTS2S_SPP_SRV_LINE_ST_REQ;

typedef struct
{
    U16 type;
    U8 device_id;
    U8 srv_chl;
    U8 line_st;
    BOOL err_flag;
} BTS2S_SPP_SRV_LINE_ST_REQ_EXT;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 mode;
} BTS2S_SPP_SRV_MODE_CHANGE_REQ;


typedef struct
{
    U16 type;
    U8 device_id;
    U8 srv_chl;
    U8 mode;
} BTS2S_SPP_SRV_MODE_CHANGE_REQ_EXT;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 res;
    U8 mode;
    U8 device_id;
    U8 srv_chl;
} BTS2S_SPP_SRV_MODE_CHANGE_IND;

typedef struct
{
    U16 type;
    U8 spp_id;
} BTS2S_SPP_SRV_DISC_REQ;

typedef struct
{
    U16 type;
    U8 device_id;
    U8 srv_chl;
} BTS2S_SPP_SRV_DISC_REQ_EXT;

typedef struct
{
    U16 type;
    U8 spp_id;
    U8 res;
    BTS2S_BD_ADDR bd;
    U8 device_id;
    U8 srv_chl;
    U8 *uuid;
    U8 uuid_len;
} BTS2S_SPP_SRV_DISC_IND;

/*
 * DESCRIPTION:
 *      This function is used to enable SPP service and make it accessible.
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      svc_name: Service name
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SPP_SRV_ENB_CFM with structure BTS2S_SPP_SRV_ENB_CFM
 *      will be received as a confirmation.
 *
*/
void spp_srv_enb_req(U8 spp_id, char *svc_name);


/*
Description:
    add a new spp uuid list node
Input:
    uuid:Universally Unique Identifier(16bit,32bit,128bit)
    uuid_len:length of Universally Unique Identifier(16bit,32bit,128bit)
    srv_name:spp service name
Time:2024/07/23 17:33:07

Author:zhengyu

Modify:
*/
spp_err_t spp_add_uuid_list_node(U8 *uuid, U8 uuid_len, char *srv_name);


/*
Description:
    add spp uuid list node through incoming sdp record
Input:
    bts2s_svc_record_size:size of sdp record
    bts2s_svc_record_srv_ch_idx:the service channel idx in the sdp record
    bts2s_svc_record:pointer of sdp record
Time:2024/07/23 17:36:07

Author:zhengyu

Modify:
*/
spp_err_t spp_add_uuid_list_node_ext(U16 bts2s_svc_record_size, U16 bts2s_svc_record_srv_ch_idx, U8 *bts2s_svc_record);


BTS2S_SPP_UUID_LIST_EXT *spp_get_uuid_list_by_uuid(U8 *uuid, U8 uuid_len, U8 *srv_chnl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Disable SPP service
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SPP_SRV_DISB_CFM with structure BTS2S_SPP_SRV_DISB_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void spp_srv_disb_req(U8 spp_id);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Give a response for SPP accept/reject connection.
 *
 * INPUT:
 *      spp_id: SPP instance identification.
 *      srv_chnl: Local server channel registered.
 *      bd: Remote device Bluetooth Address.
 *      res: TRUE or FALSE
 *           TRUE means accept current connection.
 *           FALSE means reject current connection.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void spp_srv_conn_rsp(U8 spp_id,
                      U8 srv_chnl,
                      BTS2S_BD_ADDR bd,
                      U8 res);

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
*      Message BTS2MU_SPP_SRV_DISC_CFM with structure BTS2S_SPP_SRV_DISC_CFM
*      will be received as a confirmation.
*
*----------------------------------------------------------------------------*/
void spp_srv_disc_req(U8 spp_id);
void spp_srv_disc_req_ext(U8 device_id, U8 srv_chnl);

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
 *      Message BTS2MU_SPP_SRV_DATA_CFM with structure BTS2S_SPP_SRV_DATA_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void spp_srv_data_req(U8 spp_id,
                      U16 payload_len,
                      U8 *payload);

void spp_srv_data_req_ext(U8 device_id, U8 srv_chnl,
                          U16 payload_len,
                          U8 *payload);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      It is indicated that the application is ready to receive next piece of data
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void spp_srv_data_rsp(U8 spp_id);
void spp_srv_data_rsp_ext(U8 device_id, U8 srv_chnl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     This function is used to send a test command to remote device.
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
void spp_srv_test_req(U8 spp_id,
                      U16 test_data_len,
                      U8 *test_data);

void spp_srv_test_req_ext(U8 device_id, U8 srv_chnl,
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
void spp_srv_ctrl_req(U8 spp_id,
                      U8 mdm_st,
                      U8 break_sig);

void spp_srv_ctrl_req_ext(U8 device_id, U8 srv_chnl,
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
 *----------------------------------------------------------------------------*/
void spp_srv_portneg_req(U8 spp_id,
                         BOOL req,
                         BTS2S_PORT_PAR *port_par);

void spp_srv_portneg_req_ext(U8 device_id, U8 srv_chnl,
                             BOOL req,
                             BTS2S_PORT_PAR *port_par);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used for indicate remote port line status.
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      err_flag: error flag
 *      line_st: line status
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void spp_srv_line_st_req(U8 spp_id,
                         BOOL err_flag,
                         U8 line_st);

void spp_srv_line_st_req_ext(U8 device_id, U8 srv_chnl,
                             BOOL err_flag,
                             U8 line_st);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Change current SPP mode.
 *
 * INPUT:
 *      spp_id: SPP instance identification
 *      mode: ACT_MODE, HOLD_MODE, SNIFF_MODE or PARK_MODE.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *
 *----------------------------------------------------------------------------*/
void spp_srv_mode_change_req(U8 spp_id, U8 mode);

void spp_srv_mode_change_req_ext(U8 device_id, U8 srv_chnl, U8 mode);

#ifdef __cplusplus
}
#endif


#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
