/**
  ******************************************************************************
  * @file   sdp_api.h
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

#ifndef _SDA_API_H_
#define _SDA_API_H_

#include "gap_sdc_type.h"

#define SDC_OPEN_SRCH_OK         0x00  /* open srch succeeded */
#define SDC_OPEN_SRCH_BUSY       0x01  /* curly performing srch */
#define SDC_OPEN_SRCH_FAILED     0x02  /* rmt dev refused conn */
#define SDC_OPEN_SRCH_OPEN       0x03  /* SDC alrdy has conn open */
#define SDC_OPEN_DISCED          0x04  /* rmt service dev disced */
#define SDC_RSP_SUCC             0x0000/* srch succeeded */
#define SDC_ERR_RSP_PDU          0x0010/* SDP_err_rsp PDU */
#define SDC_NO_RSP_DATA          0x0011/* empty rsp - no ress */
#define SDC_CON_DISCED           0x0012/* rmt dev disced */
#define SDC_CONN_ERR             0x0013/* rmt dev refused */
#define SDC_CFG_ERR              0x0014/* L2C cfg failed */
#define SDC_SRCH_DATA_ERR        0x0015/* srch data is invld */
#define SDC_DATA_CFM_ERR         0x0016/* failed to transmit PDU */
#define SDC_SRCH_BUSY            0x0017/* srch is busy */
#define SDC_RSP_PDU_HEADER_ERR   0x0018/* the rsp from sds had a */
#define SDC_RSP_PDU_SIZE_ERR     0x0019/* the rsp from sds had a */
#define SDC_RSP_TIMEOUT_ERR      0x001a/* the rsp from sds has */
#define SDC_SRCH_SIZE_TO_BIG     0x001b/* the size of the srch will */
#define SDC_RSP_OUT_OF_MEM       0x001c/* the rsp was too big to fit*in mem */
#define SDC_RSP_TERMINATED       0x001d/* the srch was terminated */
#define PDU_ERR_VERSION          0x0001
#define PDU_ERR_SR_HANDLE        0x0002
#define PDU_ERR_SYNTAX           0x0003
#define PDU_ERR_PDU_SIZE         0x0004
#define PDU_ERR_CONT_ST          0x0005
#define PDU_ERR_INSUFF_RES       0x0006
#define SDC_SDS_DISCED           0x00  /* the conn was dicsonnected */
#define SDC_SDC_DISCED           0x01  /* the conn was dicsonnected */

#define SDP_ERR_RSP              0x01
#define SDP_SVC_SRCH_REQ         0x02
#define SDP_SVC_SRCH_RSP         0x03
#define SDP_SVC_ATTRUTE_REQ      0x04
#define SDP_SVC_ATTRUTE_RSP      0x05
#define SDP_SVC_SRCH_ATTRUTE_REQ 0x06
#define SDP_SVC_SRCH_ATTRUTE_RSP 0x07

#define SDC_MSG_BASE             0x0000

enum
{
    BTS2M_SDC_SVC_SRCH_REQ = BTS2MD_START + 1,
    BTS2M_SDC_SVC_SRCH_CFM,
    BTS2M_SDC_SVC_ATTRUTE_REQ,
    BTS2M_SDC_SVC_ATTRUTE_CFM,
    BTS2M_SDC_SVC_SRCH_ATTRUTE_REQ,
    BTS2M_SDC_SVC_SRCH_ATTRUTE_CFM,
    BTS2M_SDC_TERMINATE_MSGITIVE_REQ,
    BTS2M_SDC_OPEN_SRCH_REQ,
    BTS2M_SDC_OPEN_SRCH_CFM,
    BTS2M_SDC_CLOSE_SRCH_REQ,
    BTS2M_SDC_CFG_REQ,
    BTS2M_SDC_CLOSE_SRCH_CFM
};

#define SDC_BASE                    (0x01)
#define SDC_MAX                     (0x0c)

typedef struct
{
    U16           type;           /* always SDC_SVC_SRCH_REQ */
    U16           tid;            /* routing hdl */
    BTS2S_BD_ADDR bd;             /* addr of bluetooth dev */
    U16          size_srch_pttrn; /* size of the search pattern */
    U8          *srch_pttrn;      /* pointer to search pattern */
    U16          max_num_rec;     /* maximum num of svc */
} BTS2S_SDC_SVC_SRCH_REQ;

typedef struct
{
    U16 type;          /* always SDC_SVC_SRCH_CFM */
    U16 tid;           /* as supplied in REQ */
    U16 num_recs_ret;  /* num of svc record */
    U16 size_rec_list; /* size of the rec_list in bytes */
    U8 *rec_list;      /* pointer to list of */
    U16 rsp;           /* indicates whether or not the */
    U16 err_code;      /* err code if one has occurred */
    U16 size_err_info; /* size of the err_info */
    U8 *err_info;      /* err - specific information - not */
    BTS2S_BD_ADDR bd;  /* addr of bluetooth dev */
} BTS2S_SDC_SVC_SRCH_CFM;

typedef struct
{
    U16           type;         /* always */
    U16           tid;          /* routing hdl */
    BTS2S_BD_ADDR bd;           /* addr of bluetooth dev */
    U32          svc_rec_hndl;  /* bts2s_svc_record_hdl of */
    U16          size_attr_list;/* size of the attr_list in bytes */
    U8          *attr_list;     /* pointer to attribute list */
    U16          max_num_attr;
} BTS2S_SDC_SVC_ATTRUTE_REQ;


typedef struct
{
    U16 type;           /* always */
    U16 tid;            /* as supplied in REQ */
    U16 size_attr_list; /* size of the attr_list in bytes */
    U8 *attr_list;      /* the attribute list, described as a data elem sequence */
    U16 rsp;            /* indicates whether or not the */
    U16 err_code;       /* err code if one has occurred */
    U16 size_err_info;  /* size of the err_info */
    U8 *err_info;       /* err - specific information - */
    BTS2S_BD_ADDR bd;   /* addr of bluetooth dev */
} BTS2S_SDC_SVC_ATTRUTE_CFM;

typedef struct
{
    U16           type;           /* always */
    U16           tid;            /* routing hdl */
    BTS2S_BD_ADDR bd;             /* addr of bluetooth dev */
    U16          size_srch_pttrn; /* size of the search pattern - bytes */
    U8          *srch_pttrn;      /* pointer to search pattern */
    U16          size_attr_list;  /* size of the attribute list - bytes */
    U8          *attr_list;       /* pointer to attribute list */
    U16          max_num_attr;
} BTS2S_SDC_SVC_SRCH_ATTRUTE_REQ;

typedef struct
{
    U16   type;          /* always */
    U16   tid;           /* as supplied in REQ */
    U16   total_rsp_size;
    U16   size_attr_list;
    U8    *attr_list;
    BOOL  more_to_come;
    U16   rsp;            /* indicates whether or not the */
    U16   err_code;       /* err code if one has occurred */
    U16   size_err_info;  /* size of the err info */
    U8   *err_info;       /* err - specific information */
    BTS2S_BD_ADDR bd;            /* addr of bluetooth dev */
} BTS2S_SDC_SVC_SRCH_ATTRUTE_CFM;

typedef struct
{
    U16           type;         /* always SDC_TERMINATE_MSGITIVE_REQ */
    U16           tid;          /* routing hdl */
} BTS2S_SDC_TERMINATE_MSGITIVE_REQ;

typedef struct
{
    U16          type;             /* always SDC_OPEN_SRCH_REQ */
    U16          tid;              /* routing task id */
    BTS2S_BD_ADDR bd;              /* addr of bluetooth dev to be */
} BTS2S_SDC_OPEN_SRCH_REQ;

typedef struct
{
    U16 type;  /* always SDC_OPEN_SRCH_CFM */
    U16 tid;   /* routing hdl */
    U8  rsp;   /* res from open search req */
    BTS2S_BD_ADDR bd;              /* addr of bluetooth dev to be */
} BTS2S_SDC_OPEN_SRCH_CFM;

typedef struct
{
    U16 type; /* always SDC_CLOSE_SRCH_REQ */
    U16 tid;  /* routing hdl */
    BTS2S_BD_ADDR bd;              /* addr of bluetooth dev */
} BTS2S_SDC_CLOSE_SRCH_REQ;

typedef struct
{
    U16 type; /* always SDC_CFG_REQ */
    U16 mtu;  /* L2C MTU val */
} BTS2S_SDC_CFG_REQ;

typedef struct
{
    U16 type; /* always SDC_CLOSE_SRCH_IND */
    U16 tid;  /* routing hdl */
    U8  rsp;  /* res conn err */
    BTS2S_BD_ADDR bd;              /* addr of bluetooth dev */
} BTS2S_SDC_CLOSE_SRCH_CFM;

typedef struct
{
    U16 type; /* whatever */
    U16 tid;  /* routing hdl */
} BTS2S_SDC_UP_MSG_COMMON;

#define REQ_ERR         0x0001
#define REQ_PASS        0x0000
#define SDS_MSG_BASE    0x000c

typedef enum BTS2E_SDS_MSG_TAG
{
    ENUM_SEP_SDS_FIRST_MSG = SDS_MSG_BASE,
    ENUM_SDS_REG_REQ,
    ENUM_SDS_REG_CFM,
    ENUM_SDS_UNREG_REQ,
    ENUM_SDS_UNREG_CFM,
    ENUM_SDS_CFG_REQ,
    ENUM_SEP_SDS_LAST_MSG
} BTS2E_SDS_MSG;

#define SDS_REG_REQ              ((U16)(ENUM_SDS_REG_REQ))
#define SDS_REG_CFM              ((U16)(ENUM_SDS_REG_CFM))
#define SDS_UNREG_REQ            ((U16)(ENUM_SDS_UNREG_REQ))
#define SDS_UNREG_CFM            ((U16)(ENUM_SDS_UNREG_CFM))
#define SDS_CFG_REQ              ((U16)(ENUM_SDS_CFG_REQ))

#define SDS_BASE                 (ENUM_SEP_SDS_FIRST_MSG + 1)
#define SDS_MAX                  (ENUM_SEP_SDS_LAST_MSG - SDS_MSG_BASE - 1)

typedef struct
{
    U16 type;          /* always SDS_REG_REQ */
    U16 tid;           /* routing hdl */
    U8  *svc_rec;      /* pointer to svc record data */
    U16 num_rec_byte;  /* num of bytes in the svc */
} BTS2S_SDS_REG_REQ;

typedef struct
{
    U16 type;          /* always SDS_REG_CFM */
    U16 tid;           /* routing hdl */
    U32 svc_rec_hndl;  /* bts2s_svc_record_hdl of the */
    U16 res;           /* succ or fail */
} BTS2S_SDS_REG_CFM;

typedef struct
{
    U16 type;          /* always SDS_UNREG_REQ */
    U16 tid;           /* routing hdl */
    U32 svc_rec_hndl;  /* bts2s_svc_record_hdl of the */

} BTS2S_SDS_UNREG_REQ;

typedef struct
{
    U16 type;         /* always SDS_UNREG_CFM */
    U16 tid;          /* routing hdl */
    U32 svc_rec_hndl; /* bts2s_svc_record_hdl of the */
    U16 res;          /* succ or fail */
} BTS2S_SDS_UNREG_CFM;

typedef struct
{
    U16 type;          /* always SDS_CFG_REQ */
    U16 mtu;           /* L2C MTU val */
} BTS2S_SDS_CFG_REQ;

typedef struct
{
    U16 type;          /* whatever */
    U16 tid;           /* routing hdl */
} BTS2S_SDS_UP_MSG_COMMON;

typedef union
{
    U16                    type;
    BTS2S_SDS_UP_MSG_COMMON sds_umsg_common;
    BTS2S_SDS_REG_REQ       sdpm_sds_reg_req;
    BTS2S_SDS_REG_CFM       sds_reg_cfm;
    BTS2S_SDS_UNREG_REQ     sdpm_sds_unreg_req;
    BTS2S_SDS_UNREG_CFM     sdpm_sds_unreg_cfm;
    BTS2S_SDS_CFG_REQ       sds_cfg_req;

} SDS_UMSG;

typedef union
{
    U16                                 type;
    BTS2S_SDC_UP_MSG_COMMON             sdc_umsg_common;
    BTS2S_SDC_SVC_SRCH_REQ              sdpm_sdc_svc_srch_req;
    BTS2S_SDC_SVC_SRCH_CFM              sdc_svc_srch_cfm;
    BTS2S_SDC_SVC_ATTRUTE_REQ           sdpm_sdc_svc_attrute_req;
    BTS2S_SDC_SVC_ATTRUTE_CFM           sdc_svc_attrute_cfm;
    BTS2S_SDC_SVC_SRCH_ATTRUTE_REQ      sdpm_sdc_svc_srch_attrute_req;
    BTS2S_SDC_SVC_SRCH_ATTRUTE_CFM      sdc_svc_srch_attrute_cfm;
    BTS2S_SDC_TERMINATE_MSGITIVE_REQ    sdpm_sdc_terminate_msg_req;
    BTS2S_SDC_OPEN_SRCH_REQ             sdpm_sdc_open_srch_req;
    BTS2S_SDC_OPEN_SRCH_CFM             sdc_open_srch_cfm;
    BTS2S_SDC_CLOSE_SRCH_REQ            sdpm_sdc_close_srch_req;
    BTS2S_SDC_CFG_REQ                   sdpm_sdc_cfg_req;
    BTS2S_SDC_CLOSE_SRCH_CFM            sdc_close_srch_ind;

} SDC_UMSG;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send an SDC_CFG_REQ msg to SDP.
 *
 * INPUT:
 *     U16 mtu : maximum transaction unit
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *
 *----------------------------------------------------------------------------*/
void sdpa_sdc_cfg_req(U16 mtu);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send an SDC_OPEN_SRCH_REQ msg to SDP.
 *
 *
 * INPUT:
 *     U16 tid : routing hdl
 *     BTS2S_BD_ADDR *bd : remote dev
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *
 *----------------------------------------------------------------------------*/
void sdpa_sdc_open_srch_req(U16 tid, BTS2S_BD_ADDR *bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send an SDC_CLOSE_SRCH_REQ msg to SDP.
 *
 * INPUT:
 *     U16 tid : routing hdl
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *
 *----------------------------------------------------------------------------*/
void sdpa_sdc_close_srch_req(U16 tid, BTS2S_BD_ADDR *bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send a SDC_SVC_ATTRUTE_REQ msg to SDP.
 *
 * INPUT:
 *    U16 tid : routing hdl
 *    BTS2S_BDADDR *p_bd : remote dev
 *    U32 svc_rec_hdl : remote service hdl
 *    U16 num_attrs : num of attribute list
 *    U16  *attr_list : pointer to the attribute list
 *    U16 max_num_attr : maximum byte per rsp
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void sdpa_sdc_svc_attrute_req(U16 tid,
                              BTS2S_BD_ADDR *bd,
                              U32 svc_rec_hndl,
                              U16 num_attr,
                              U16 *attr_list,
                              U16 max_num_attr);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send an SDC_SVC_SRCH_ATTRUTE_REQ msg to SDP.
 *
 * INPUT:
 *    BTS2S_BD_ADDR *p_bd : remote device
 *    U16 num_uuid : num of search pattern
 *    U8  *uuid_list : pointer to the search pattern
 *    U16 num_attrs : num of attribute list
 *    U16  *attr_list : pointer to the attribute list
 *    U16  max_num_attr : maximum bytes per rsp
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    none.
 *----------------------------------------------------------------------------*/
void sdpa_sdc_svc_srch_attrute_req(U16 tid,
                                   BTS2S_BD_ADDR *bd,
                                   U16 num_uuid,
                                   GAP_BT_UUID *uuid_list,
                                   U16 num_attr,
                                   U16 *attr_list,
                                   U16 max_num_attr);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send an SDC_SVC_SRCH_REQ msg to SDP.
 *
 *
 * INPUT:
 *     U16 tid : routing hdl
 *     BTS2S_BD_ADDR *p_bd : remote dev
 *     U16 size_srch_pttrn : size of search pattern
 *     U8 *srch_pttrn : pointer to the search pattern
 *     U16 max_num_rec : maximum record to return
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void sdpa_sdc_svc_srch_req(U16 tid,
                           BTS2S_BD_ADDR *bd,
                           U16 num_uuid,
                           GAP_BT_UUID *uuid_list,
                           U16 max_num_rec);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Build and send an SDC_TERMINATE_MSGITIVE_REQ msg to SDP.
 *
 *
 * INPUT:
 *     U16 tid routing hdl
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *
 *----------------------------------------------------------------------------*/
void sdpa_sdc_terminate_msg_req(U16 tid);

/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*    Build and send an SDS_CFG_REQ msg to SDP.
*
*
* INPUT:
*    U16 mtu : maximum transcation unit
*
*
* OUTPUT:
*     void.
*
* NOTE:
*     none.
*
*----------------------------------------------------------------------------*/
void sdpa_sds_cfg_req(U16 mtu);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send an SDS_REG_REQ msg to SDP.
 *
 *
 * INPUT:
 *    U16 tid : routing hdl
 *    U8 *svc_rec :pointer to svc record data
 *    U16 rec_byte_num : num of byte in the service record data
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *
 *----------------------------------------------------------------------------*/
void sdpa_sds_reg_req(U16 tid, U8 *svc_rec, U16 rec_byte_num);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *    Build and send an SDS_UNREG_REQ msg to SDP.
 *
 *
 * INPUT:
 *    U16 tid : routing hdl
 *    U32 svc_rec_hndl : svc record hdl
 *
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *
 *----------------------------------------------------------------------------*/
void sdpa_sds_unreg_req(U16 tid,  U32 svc_rec_hndl);


#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
