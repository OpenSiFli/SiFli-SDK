/**
  ******************************************************************************
  * @file   av_api.h
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

#ifndef _AV_API_H_
#define _AV_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#define AV_ACPT                       ((U8)0x00)
#define AV_BAD_HEADER_FMT             ((U8)0x01)
#define AV_BAD_LEN                    ((U8)0x11)
#define AV_BAD_ACP_SEID               ((U8)0x12)
#define AV_SEP_IN_USE                 ((U8)0x13)
#define AV_SEP_NOT_IN_USE             ((U8)0x14)
#define AV_BAD_SERV_CATEGORY          ((U8)0x17)
#define AV_BAD_PAYLOAD_FMT            ((U8)0x18)
#define AV_NOT_SUPP_CMD               ((U8)0x19)
#define AV_INVLD_CAPABILITY           ((U8)0x1A)
#define AV_BAD_RECOVERY_TYPE          ((U8)0x22)
#define AV_BAD_MEDIA_TRS_FMT          ((U8)0x23)
#define AV_BAD_RECOVERY_FMT           ((U8)0x25)
#define AV_BAD_ROHC_FMT               ((U8)0x26)
#define AV_BAD_CP_FMT                 ((U8)0x27)
#define AV_BAD_MULTIPLEXING_FMT       ((U8)0x28)
#define AV_UNSUPP_CFG                 ((U8)0x29)
#define AV_BAD_ST                     ((U8)0x31)
#define AV_BAD_SVC                    ((U8)0x80)
#define AV_INSUFFICIENT_RESRCS        ((U8)0x81)
#define AV_OPEN_STREAM_FAIL           ((U8)0x90)
#define AV_L2CA_CONN_ACPT_ERR         ((U8)0x91)
#define AV_INVLD_ROLE                 ((U8)0x92)
#define AV_SDS_REG_FAILED             ((U8)0x93)
#define AV_SDS_UNREG_FAILED           ((U8)0x94)
#define AV_MAX_CONN_NUM               ((U8)0x95)
#define AV_MAX_AV_CONN_NUM            ((U8)0x96)
#define AV_CONN_ATTMPT_FAILED         ((U8)0x97)
#define AV_DATA_SEND_FAILED           ((U8)0x98)
#define AV_MAX_APP_CLT_NUM            ((U8)0x99)
#define AV_DUPLICATE_CONNECTING       ((U8)0x9A)
#define AV_CONNECT_CONFLICT           ((U8)0x9B)
#define AV_SIG_TIMEOUT                ((U8)0xC0)

#define AV_INVLD_CODEC_TYPE           ((U8)0xC1)
#define AV_NOT_SUPP_CODEC_TYPE        ((U8)0xC2)
#define AV_INVLD_SAMPLING_FREQ        ((U8)0xC3)
#define AV_NOT_SUPP_SAMPLING_FREQ     ((U8)0xC4)
#define AV_INVLD_CHNL_MODE            ((U8)0xC5)
#define AV_NOT_SUPP_CHNL_MODE         ((U8)0xC6)
#define AV_INVLD_SUBBANDS             ((U8)0xC7)
#define AV_NOT_SUPP_SUBBANDS          ((U8)0xC8)
#define AV_INVLD_ALLOC_METHOD         ((U8)0xC9)
#define AV_NOT_SUPP_ALLOC_METHOD      ((U8)0xCA)
#define AV_INVLD_MIN_BITPOOL          ((U8)0xCB)
#define AV_NOT_SUPP_MIN_BITPOOL       ((U8)0xCC)
#define AV_INVLD_MAX_BITPOOL          ((U8)0xCD)
#define AV_NOT_SUPP_MAX_BITPOOL       ((U8)0xCE)
#define AV_INVLD_LAYER                ((U8)0xCF)
#define AV_NOT_SUPP_LAYER             ((U8)0xD0)
#define AV_NOT_SUPP_CRC               ((U8)0xD1)
#define AV_NOT_SUPP_MPF               ((U8)0xD2)
#define AV_NOT_SUPP_VBR               ((U8)0xD3)
#define AV_INVLD_BIT_RATE             ((U8)0xD4)
#define AV_NOT_SUPP_BIT_RATE          ((U8)0xD5)
#define AV_INVLD_OBJ_TYPE             ((U8)0xD6)
#define AV_NOT_SUPP_OBJ_TYPE          ((U8)0xD7)
#define AV_INVLD_CHNLS                ((U8)0xD8)
#define AV_NOT_SUPP_CHNLS             ((U8)0xD9)
#define AV_INVLD_VERSION              ((U8)0xDA)
#define AV_NOT_SUPP_VERSION           ((U8)0xDB)
#define AV_NOT_SUPP_MAX_SUL           ((U8)0xDC)
#define AV_INVLD_BLOCK_LEN            ((U8)0xDD)
#define AV_INVLD_CP_TYPE              ((U8)0xE0)
#define AV_INVLD_CP_FMT               ((U8)0xE1)

/* media types */
#define AV_AUDIO      ((U8)0x00)
#define AV_VIDEO      ((U8)0x01)
#define AV_MULTIMEDIA ((U8)0x10)

/* stream end - point type */
#define AV_SRC ((U8)0x00)
#define AV_SNK ((U8)0x01)

/* media codecs */
#define AV_SBC            ((U8)0x00)
#define AV_MPEG12_AUDIO   ((U8)0x01)
#define AV_MPEG24_AAC     ((U8)0x02)
#define AV_ATRAC          ((U8)0x04)
#define AV_NON_A2DP_CODEC ((U8)0xFF)

/* video codecs: (assigned numbers) */
#define AV_H263_BASELINE           ((U8)0x01)
#define AV_MPEG_VISUAL_SIMPEL_PROF ((U8)0x02)
#define AV_H263_PROF3              ((U8)0x03)
#define AV_H263_PROF8              ((U8)0x04)
#define AV_NON_VDP_CODEC           ((U8)0xFF)

/* service cap header sizes for video codecs */
#define H263_MEDIA_CODEC_SC_SIZE 5

/* defined in spec, same for all H263 profs */
#define H263_CAP_LEVEL_10 ((U8)0x80)
#define H263_CAP_LEVEL_20 ((U8)0x40)
#define H263_CAP_LEVEL_30 ((U8)0x20)

/* ther bts2g_bits in H263 service capabilities */
#define MPEG_CAP_LEVEL_0 ((U8)0x80)
#define MPEG_CAP_LEVEL_1 ((U8)0x40)
#define MPEG_CAP_LEVEL_2 ((U8)0x20)
#define MPEG_CAP_LEVEL_3 ((U8)0x10)

/* av roles */
#define AV_AUDIO_SRC ((U16)0x00)
#define AV_AUDIO_SNK ((U16)0x01)
#define AV_VIDEO_SRC ((U16)0x02)
#define AV_VIDEO_SNK ((U16)0x03)
#define AV_AUDIO_NO_ROLE ((U16)0xFF)

#define AV_FIXED_MEDIA_PKT_HDR_SIZE    12
//Todo This macro presentation cannot be modified.
#define AV_MTU_SIZE      (1005)          /* max frm size for L2C pkt */

typedef enum
{
    AV_SC_MEDIA_TRS = 1,
    AV_SC_REPORTING,
    AV_SC_RECOVERY,
    AV_SC_CONT_PROTECTION,
    AV_SC_HDR_COMPRESSION,
    AV_SC_MULTIPLEXING,
    AV_SC_MEDIA_CODEC,
    AV_SC_DELAY_REPORTING,
    AV_SC_NEXT = 0xFFFF
} BTS2E_AV_SERV_CAP;

typedef enum BTS2E_AV_MSG_TAG
{
    BTS2MD_AV_ENB_REQ = BTS2MD_START,
    BTS2MD_AV_DISB_REQ,
    BTS2MD_AV_CONN_REQ,
    BTS2MD_AV_DISC_REQ,
    BTS2MD_AV_DISCOVER_REQ,
    BTS2MD_AV_GET_CAPABILITIES_REQ,
    BTS2MD_AV_SET_CFG_REQ,
    BTS2MD_AV_GET_CFG_REQ,
    BTS2MD_AV_CFG_REQ,
    BTS2MD_AV_OPEN_REQ,
    BTS2MD_AV_START_REQ,
    BTS2MD_AV_CLOSE_REQ,
    BTS2MD_AV_SUSPEND_REQ,
    BTS2MD_AV_ABORT_REQ,
    BTS2MD_AV_SECU_CTRL_REQ,
    BTS2MD_AV_STREAM_DATA_REQ,
    BTS2MD_AV_GET_ALL_CAPABILITIES_REQ,
    BTS2MD_AV_DELAY_REPORT_REQ,
    BTS2MD_AV_DISCOVER_RSP,
    BTS2MD_AV_GET_CAPABILITIES_RSP,
    BTS2MD_AV_SET_CFG_RSP,
    BTS2MD_AV_GET_CFG_RSP,
    BTS2MD_AV_CFG_RSP,
    BTS2MD_AV_OPEN_RSP,
    BTS2MD_AV_START_RSP,
    BTS2MD_AV_CLOSE_RSP,
    BTS2MD_AV_SUSPEND_RSP,
    BTS2MD_AV_ABORT_RSP,
    BTS2MD_AV_SECU_CTRL_RSP,
    BTS2MD_AV_GET_ALL_CAPABILITIES_RSP,
    BTS2MD_AV_DELAY_REPORT_RSP,
    BTS2MD_AV_RESET_REQ,
    BTS2MD_AV_UNREGISTER_SDP_REQ,
    BTS2MD_AV_REGISTER_SDP_REQ,

    BTS2MU_AV_CONN_IND = BTS2MU_START,
    BTS2MU_AV_DISC_IND,
    BTS2MU_AV_DISCOVER_IND,
    BTS2MU_AV_GET_CAPABILITIES_IND,
    BTS2MU_AV_SET_CFG_IND,
    BTS2MU_AV_GET_CFG_IND,
    BTS2MU_AV_CFG_IND,
    BTS2MU_AV_OPEN_IND,
    BTS2MU_AV_START_IND,
    BTS2MU_AV_CLOSE_IND,
    BTS2MU_AV_SUSPEND_IND,
    BTS2MU_AV_ABORT_IND,
    BTS2MU_AV_SECU_CTRL_IND,
    BTS2MU_AV_STREAM_DATA_IND,
    BTS2MU_AV_QOS_IND,
    BTS2MU_AV_STREAM_MTU_SIZE_IND,
    BTS2MU_AV_STS_IND,
    BTS2MU_AV_GET_ALL_CAPABILITIES_IND,
    BTS2MU_AV_DELAY_REPORT_IND,

    BTS2MU_AV_ENB_CFM,
    BTS2MU_AV_DISB_CFM,
    BTS2MU_AV_CONN_CFM,
    BTS2MU_AV_DISCOVER_CFM,
    BTS2MU_AV_GET_CAPABILITIES_CFM,
    BTS2MU_AV_SET_CFG_CFM,
    BTS2MU_AV_GET_CFG_CFM,
    BTS2MU_AV_CFG_CFM,
    BTS2MU_AV_OPEN_CFM,
    BTS2MU_AV_START_CFM,
    BTS2MU_AV_CLOSE_CFM,
    BTS2MU_AV_SUSPEND_CFM,
    BTS2MU_AV_ABORT_CFM,
    BTS2MU_AV_SECU_CTRL_CFM,
    BTS2MU_AV_GET_ALL_CAPABILITIES_CFM,
    BTS2MU_AV_STREAM_DATA_CFM,
    BTS2MU_AV_DELAY_REPORT_CFM,
    BTS2MU_AV_ERROR_IND,

    //New messages must be added before this
    BTS2M_AV_MAX_MSG_NUM
} BTS2E_AV_MSG;

#define BTS2MD_HIGHEST_AV_RECV_MSG_NUM  (BTS2MD_AV_REGISTER_SDP_REQ + 1)
#define AV_RECV_MSG_NUM   (BTS2MD_AV_REGISTER_SDP_REQ - BTS2MD_START + 1)
#define AV_SEND_MSG_NUM   (BTS2M_AV_MAX_MSG_NUM - BTS2MU_START)

typedef struct
{
    U8 acp_seid;
    BOOL in_use;
    U8 media_type;
    U8 sep_type;
    U8 codec;
} BTS2S_AV_SEID_INFO;


typedef struct
{
    U16 type;
    U16 tid;
    U16 local_role;
} BTS2S_AV_ENB_REQ;

typedef struct
{
    U16 type;
    U16 enable_role;
    U8  res;
} BTS2S_AV_ENB_CFM;

typedef struct
{
    U16 type;
} BTS2S_AV_DISB_REQ;

typedef struct
{
    U16 type;
    U16 local_role;
} BTS2S_AV_UNREGISTER_SDP_REQ;

typedef struct
{
    U16 type;
    U16 local_role;
} BTS2S_AV_REGISTER_SDP_REQ;

typedef struct
{
    U16 type;
    U8  res;
} BTS2S_AV_DISB_CFM;

typedef struct
{
    U16           type;
    U16           tid;
    BTS2S_BD_ADDR bd;
    U16           rmt_role;
    U16           local_role;
} BTS2S_AV_CONN_REQ;

typedef struct
{
    U16              type;
    U8               res;
    U16              conn_id;
    U16              max_frm_size;
    U16              local_role;
    BTS2S_BD_ADDR    bd;
} BTS2S_AV_CONN_CFM;

typedef struct
{
    U16          type;
    U16          conn_id;
    BTS2S_BD_ADDR bd;
} BTS2S_AV_CONN_IND;

typedef struct
{
    U16 type;
    U16 conn_id;
} BTS2S_AV_DISC_REQ;

typedef struct
{
    U16 type;
    U16 conn_id;
    U8  res;
    BTS2S_BD_ADDR bd;
} BTS2S_AV_DISC_IND;


typedef struct
{
    U16 type;
    U16 shdl;    /* stream handle */
    U16 len;     /* len of data (in bytes) */
    U8  *data;   /* pointer to media pkt data */
} BTS2S_AV_STREAM_DATA_REQ;

typedef BTS2S_AV_STREAM_DATA_REQ BTS2S_AV_STREAM_DATA_IND;

typedef struct
{
    U16 type;     /* this indi will be send for every 100 samples */
    U16 shdl;     /* except if buff runs full > 8 */
    U16 buff_st;  /* 0 for buf empty - 10 for full */
} BTS2S_AV_QOS_IND;

typedef struct
{
    U16 type;         /*message identity */
    U8  shdl;
    U16 rmt_mtu_size; /*holds the remote mtu size. */
} BTS2S_AV_STREAM_MTU_SIZE_IND;

typedef struct
{
    U16 type;
    U16 conn_id;
    U16 st_type;
    U16 role_type;
    U16 app_hdl;
} BTS2S_AV_STS_IND;

typedef struct
{
    U16  type;
    U16  conn_id;
    U8   tlabel;
} BTS2S_AV_DISCOVER_REQ;

typedef BTS2S_AV_DISCOVER_REQ BTS2S_AV_DISCOVER_IND;

typedef struct
{
    U16               type;
    U16               conn_id;
    U8                tlabel;
    U8                res;
    U8                list_len;
    BTS2S_AV_SEID_INFO *first_seid_info;
} BTS2S_AV_DISCOVER_RSP;

typedef BTS2S_AV_DISCOVER_RSP BTS2S_AV_DISCOVER_CFM;

typedef struct
{
    U16 type;
    U16 conn_id;
    U8  tlabel;
    U8  acp_seid;
} BTS2S_AV_GET_CAPABILITIES_REQ;

typedef BTS2S_AV_GET_CAPABILITIES_REQ BTS2S_AV_GET_CAPABILITIES_IND;

typedef struct
{
    U16 type;
    U16 conn_id;
    U8  tlabel;
    U8  res;
    U16 serv_cap_len;
    U8  *serv_cap_data; /* only bmalloc if not error */
} BTS2S_AV_GET_CAPABILITIES_RSP;

typedef BTS2S_AV_GET_CAPABILITIES_RSP BTS2S_AV_GET_CAPABILITIES_CFM;

typedef struct
{
    U16 type;
    U16 conn_id;
    U8  tlabel;
    U8  acp_seid;
    U8  int_seid;
    U16 app_serv_cap_len;
    U16 rmt_serv_cap_len;
    U8  *app_serv_cap_data;
    U8  *rmt_serv_cap_data;
} BTS2S_AV_SET_CFG_REQ;

typedef struct
{
    U16 type;
    U16 conn_id;
    U8  tlabel;
    U8  shdl;
    U8  acp_seid;
    U8  int_seid;
    U16 serv_cap_len;
    U8  *serv_cap_data;
} BTS2S_AV_SET_CFG_IND;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U8  res;
    U8  serv_category;
} BTS2S_AV_SET_CFG_RSP;

typedef struct
{
    U16 type;
    U16 conn_id;
    U8  tlabel;
    U8  res;
    U8  shdl;
    U8  serv_category;
} BTS2S_AV_SET_CFG_CFM;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
} BTS2S_AV_GET_CFG_REQ;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
} BTS2S_AV_GET_CFG_IND;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  res;
    U16 serv_cap_len;
    U8  *serv_cap_data;
} BTS2S_AV_GET_CFG_CFM;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U8  res;
    U16 serv_cap_len;
    U8  *serv_cap_data;
} BTS2S_AV_GET_CFG_RSP;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U16 serv_cap_len;
    U8  *serv_cap_data;
} BTS2S_AV_CFG_REQ;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U16 serv_cap_len;
    U8  *serv_cap_data;
} BTS2S_AV_CFG_IND;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  res;
    U8  serv_category;
} BTS2S_AV_CFG_CFM;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U8  res;
    U8  serv_category;
} BTS2S_AV_CFG_RSP;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
} BTS2S_AV_OPEN_REQ;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
} BTS2S_AV_OPEN_IND;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U8  res;
} BTS2S_AV_OPEN_RSP;

typedef BTS2S_AV_OPEN_RSP BTS2S_AV_OPEN_CFM;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  list_len;
    U8  first_shdl;
} BTS2S_AV_START_REQ;

typedef struct
{
    U16 type;
    U8  tlabel;
    U16 list_len;
    U8  first_shdl;
} BTS2S_AV_START_IND;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  res;
    U8  rej_shdl;
    U8  list_len;
    U8  first_shdl;
} BTS2S_AV_START_RSP;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  res;
    U8  rej_shdl;
} BTS2S_AV_START_CFM;

typedef struct
{
    U16  type;
    U8   shdl;
    U8   tlabel;
} BTS2S_AV_CLOSE_REQ;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
} BTS2S_AV_CLOSE_IND;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U8  res;
} BTS2S_AV_CLOSE_RSP;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  res;
    U8  shdl;
} BTS2S_AV_CLOSE_CFM;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  list_len;
    U8  first_shdl;
} BTS2S_AV_SUSPEND_REQ;

typedef struct
{
    U16 type;
    U8  tlabel;
    U16 list_len;
    U8  first_shdl;
} BTS2S_AV_SUSPEND_IND;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  res;
    U8  rej_shdl;
} BTS2S_AV_SUSPEND_CFM;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  res;
    U16 list_len;
    U8  rej_shdl;
    U8  first_shdl;
} BTS2S_AV_SUSPEND_RSP;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
} BTS2S_AV_ABORT_REQ;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
} BTS2S_AV_ABORT_IND;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
} BTS2S_AV_ABORT_RSP;

typedef BTS2S_AV_ABORT_RSP BTS2S_AV_ABORT_CFM;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U16 cont_prot_method_len;
    U8  *cont_prot_method_data;
} BTS2S_AV_SECU_CTRL_REQ;

typedef struct
{
    U16 type;
    U16 conn_id;
    U8  tlabel;
    U8  acp_seid;
} BTS2S_AV_GET_ALL_CAPABILITIES_REQ;

typedef BTS2S_AV_GET_ALL_CAPABILITIES_REQ BTS2S_AV_GET_ALL_CAPABILITIES_IND;

typedef struct
{
    U16 type;
    U16 conn_id;
    U8  tlabel;
    U8  res;
    U16 serv_cap_len;
    U8  *serv_cap_data; /* only bmalloc if not error */
} BTS2S_AV_GET_ALL_CAPABILITIES_RSP;

typedef BTS2S_AV_GET_ALL_CAPABILITIES_RSP BTS2S_AV_GET_ALL_CAPABILITIES_CFM;

typedef struct
{
    U16 type;
    U8  tlabel;
    U8  shdl;
    U8  acp_seid;
    U16 delay; // 1/10 milliseconds
} BTS2S_AV_DELAY_REPORT_REQ;

typedef struct
{
    U16 error_type;
    U16 type;
} BTS2S_AV_ERROR_IND;

typedef BTS2S_AV_DELAY_REPORT_REQ BTS2S_AV_DELAY_REPORT_IND;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U8  res;
} BTS2S_AV_DELAY_REPORT_RSP;

typedef BTS2S_AV_DELAY_REPORT_RSP BTS2S_AV_DELAY_REPORT_CFM;

typedef struct
{
    U16 type;
    U8  shdl;
    U8  tlabel;
    U16 cont_prot_method_len;
    U8  *cont_prot_method_data;
} BTS2S_AV_SECU_CTRL_IND;

typedef struct
{
    U16  type;
    U8   shdl;
    U8   tlabel;
    U8   res;
    U16  cont_prot_method_len;
    U8   *cont_prot_method_data;
} BTS2S_AV_SECU_CTRL_RSP;

typedef BTS2S_AV_SECU_CTRL_RSP BTS2S_AV_SECU_CTRL_CFM;

typedef struct
{
    U16 type;
} BTS2S_AV_LP_SUPVISN_TIMER;

typedef struct
{
    U16 type;
} BTS2S_A2DP_RESET_REQ;


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This signal is used to enable a service and make it accessible from a
 *      remote device.
 *
 * INPUT:
 *      U16 tid: task id
 *      U16 local_role: the role of this device
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_enb_req(U16 tid, U16 local_role);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This signal is used to disable a service and make in inaccessible from
 *      other devices.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_disb_req(void);
void av_unregister_sdp(U16 local_role);
void av_register_sdp(U16 local_role);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      An AV_CONN_REQ will initiate a connection towards a device specified
 *      by the blueTooth device address. the AV will send an AV_CONN_IND back
 *      to the initiator with the result of the connect attempt.
 *
 * INPUT:
 *      U16 tid: task id.
 *      BTS2S_BD_ADDR bd: address of device to connect to.
 *      U16 rmt_role: wanted role of the device to connect to.
 *      U16 local_role: the device's own role.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      a confirmation message should be received after sent out this request.
 *
 *----------------------------------------------------------------------------*/
void av_conn_req(U16 tid,
                 BTS2S_BD_ADDR bd,
                 U16 rmt_role,
                 U16 local_role);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request for disconnect of connection previously established.
 *
 * INPUT:
 *      U16 conn_id: connection id.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_disc_req(U16 conn_id);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request for discover the seid informations of the connected device .
 *
 * INPUT:
 *      U16 conn_id: connect id.
 *      U8 tlabel: transaction label.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_discover_req(U16 conn_id, U8 tlabel);

void av_delay_report_rsp_acp(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Response to reject the stream discover request.
 *
 * INPUT:
 *      U16 conn_id: connect id.
 *      U8 tlabel: transaction label.
 *      U8 error: cause of the rejection.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      discover information will be received .
 *
 *----------------------------------------------------------------------------*/
void av_discover_rsp_rej(U16 conn_id, U8 tlabel, U8 res);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Response to accept the stream discover request.
 *
 * INPUT:
 *      U16 conn_id: connect id.
 *      U8 tlabel: transaction label.
 *      U8 list_len: num of list entries.
 *      BTS2S_AV_SEID_INFO seid_list: pointer to stream end - point information structure(s).
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_discover_rsp_acp(U16 conn_id,
                         U8 tlabel,
                         U8 list_len,
                         BTS2S_AV_SEID_INFO *seid_list);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request to get the stream's capabilities.
 *
 * INPUT:
 *      U16 conn_id: conn id
 *      U8 acp_seid: acceptor stream end - point id
 *      S8 tlabel: transaction label
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_get_capabilities_req(U16 conn_id, U8 acp_seid, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject the get stream's capabilities request.
 *
 * INPUT:
 *      U16 conn_id: conn id
 *      U8 tlabel: transaction label
 *      U8 res: cause of rejection
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      capabilities information will be received after this request sent out.
 *
 *----------------------------------------------------------------------------*/
void av_get_capabilities_rsp_rej(U16 id, U8 tlabel, U8 err);
void av_get_all_capabilities_req(U16 conn_id, U8 acp_seid, U8 tlabel);

void av_get_all_capabilities_rsp_rej(U16 id, U8 tlabel, U8 err);
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept the get stream's capabilities request.
 *
 * INPUT:
 *      U16 conn_id: conn id
 *      U8 tlabel: transaction label
 *      U16 app_serv_cap_len: len of app service capabilities
 *      U8 app_serv_cap_data: pointer to app. service capabilities
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_get_capabilities_rsp_acp(U16 conn_id,
                                 U8 tlabel,
                                 U16 serv_cap_len,
                                 U8 *serv_cap_data);

void av_get_all_capabilities_rsp_acp(U16 conn_id,
                                     U8 tlabel,
                                     U16 serv_cap_len,
                                     U8 *serv_cap_data);

void av_delay_report_req(U8 shdl, U8 tlabel, U8 acp_seid, U16 delay);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      request to  set stream's configuration.
 *
 * INPUT:
 *      U16 conn_id: conn id
 *      U8 acp_seid: acceptor stream end - point id
 *      U8 int_seid: initiator stream end - point id
 *      U16 app_serv_cap_len: len of app service capabilities
 *      U8 app_serv_cap_data: pointer to app. service capabilities
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      set configuration confirmation will be received.
 *
 *----------------------------------------------------------------------------*/
void av_set_cfg_req(U16 conn_id,
                    U8 tlabel,
                    U8 acp_seid,
                    U8 int_seid,
                    U16 serv_cap_len,
                    U8 *serv_cap_data,
                    U16 rmt_serv_cap_len,
                    U8 *rmt_serv_cap_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept the stream set configuration request.
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      U8 tlabel: transaction label
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_set_cfg_rsp_acp(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject to set configuration to the stream
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      U8 tlabel: transaction label
 *      U8 res: cause of rejection
 *      U8 serv_category: failing service category
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_set_cfg_rsp_rej(U8 shdl,
                        U8 tlabel,
                        U8 res,
                        U8 serv_category);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request for cur stream configuration.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Get configuration confirmation will be received.
 *
 *----------------------------------------------------------------------------*/
void av_get_cfg_req(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject response to a stream get configuration request.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *      U8 res: cause of rejection.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_get_cfg_rsp_rej(U8 shdl, U8 tlabel, U8 res);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept response to a stream get configuration request.
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      U8 tlabel: transaction label
 *      U16 serv_cap_len: len of app service capabilities
 *      U8 serv_cap_data: pointer to app. service capabilities
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_get_cfg_rsp_acp(U8 shdl,
                        U8 tlabel,
                        U16 serv_cap_len,
                        U8 *serv_cap_data);
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request for a stream reconfiguration.
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      U8 tlabel: transaction label
 *      U16 serv_cap_len: len of app service capabilities
 *      U8 serv_cap_data: pointer to app. service capabilities
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *    According reconfigure confirmation will be received.
 *
 *----------------------------------------------------------------------------*/
void av_recfg_req(U8 shdl,
                  U8 tlabel,
                  U16 serv_cap_len,
                  U8 *serv_cap_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept response to a stream reconfiguration request.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_recfg_rsp_acp(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject response to a stream reconfiguration request.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *      U8 res: cause of rejection.
 *      U8 serv_category: failing service category.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_recfg_rsp_rej(U8 shdl,
                      U8 tlabel,
                      U8 res,
                      U8 serv_category);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request to open a stream.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Open stream confirmation will be received.
 *
 *----------------------------------------------------------------------------*/
void av_open_req(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject to open a stream.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction labe.l
 *      U8 res: cause of rejection.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_open_rsp_rej(U8 shdl, U8 tlabel, U8 res);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept to open the stream.
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      U8 tlabel: transaction label
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_open_rsp_acp(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request to start stream.
 *
 * INPUT:
 *      U8 list_len: num of list entries.
 *      U8 tlabel: transaction label.
 *      U8 first_shdl: pointer to list of stream handle.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Start stream confirmation will be received.
 *
 *----------------------------------------------------------------------------*/
void av_start_req(U8 list_len, U8 tlabel, U8 *first_shdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject to start a stream.
 *
 * INPUT:
 *      U8 shdl: stream handle for the first stream that fails.
 *      U8 tlabel: transaction label.
 *      U8 res: cause of rejection.
 *      U8 list_len: num of list entries.
 *      U8 first_shdl: pointer to list of stream handles.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_start_rsp_rej(U8 shdl,
                      U8 tlabel,
                      U8 res,
                      U8 list_len,
                      U8 *first_shdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept to start a stream.
 *
 * INPUT:
 *      U8 tlabel: transaction label.
 *      U8 list_len: num of list entries.
 *      U8 first_shdl: pointer to list of stream handles.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_start_rsp_acp(U8 tlabel, U8 list_len, U8 *first_shdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request to close a stream.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Close stream confirmation will be received.
 *
 *----------------------------------------------------------------------------*/
void av_close_req(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject to close stream.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *      U8 res:  cause of rejection.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_close_rsp_rej(U8 shdl, U8 tlabel, U8 res);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept to close the stream.
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      U8 tlabel: transaction label
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_close_rsp_acp(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request to suspend stream.
 *
 * INPUT:
 *      U8 list_len: num of list entries.
 *      U8 tlabel: transaction label.
 *      U8 first_shdl: pointer to list of stream handles.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Suspend stream confirmation will be received.
 *
 *----------------------------------------------------------------------------*/
void av_suspend_req(U8 list_len, U8 tlabel, U8 *first_shdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject to suspend the stream.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *      U8 res: cause of rejection.
 *      U8 list_len: num of list entries.
 *      U8 first_shdl: pointer to list of stream handles.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_suspend_rsp_rej(U8 shdl,
                        U8 tlabel,
                        U8 res,
                        U16 list_len,
                        U8 *first_shdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept to suspend thr stream.
 *
 * INPUT:
 *      U8 tlabel: transaction label.
 *      U8 list_len: num of list entries.
 *      U8 first_shdl: pointer to list of stream handles.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_suspend_rsp_acp(U8 tlabel, U16 list_len, U8 *first_shdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      request to abort a stream.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      abort stream confirmation will be received after send out the request .
 *
 *----------------------------------------------------------------------------*/
void av_abort_req(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Response to abort a stream request.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_abort_rsp(U8 shdl, U8 tlabel);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request to security control a stream.
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      U8 tlabel: transaction label
 *      U16 cont_protect_method_data_len: len of cont protection method data
 *      U8 cont_protect_method_data: pointer to cont protection method data
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Confirmation will be received after the request.
 *
 *----------------------------------------------------------------------------*/
void av_secu_ctrl_req(U8 shdl,
                      U8 tlabel,
                      U16 cont_protect_method_data_len,
                      U8 *cont_protect_method_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Accept to security control a stream.
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      U8 tlabel: transaction label
 *      U16 cont_protect_method_data_len: len of cont protection method data
 *      U8 cont_protect_method_data: pointer to cont protection method data
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_secu_ctrl_rsp_acp(U8 shdl,
                          U8 tlabel,
                          U16 cont_protect_method_data_len,
                          U8 *cont_protect_method_data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reject to security control a stream.
 *
 * INPUT:
 *      U8 shdl: stream handle.
 *      U8 tlabel: transaction label.
 *      U8 err: cause for rejection.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_secu_ctrl_rsp_rej(U8 shdl, U8 tlabel, U8 res);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request to sending stream data.
 *
 * INPUT:
 *      U8 shdl: stream handle
 *      BOOL padding: padding, pkt is padded (media pkt header bit field)
 *      BOOL marker: marker (media pkt header bit field)
 *      U8 payload_type: payload type (media pkt header field)
 *      U32 timestamp: pkt timestamp (media pkt header field)
 *      U16 data_len: len of data to be sent
 *      U8 data: pointer to data
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void av_stream_data_req(U8 shdl,
                        BOOL padding,
                        BOOL marker,
                        U8 payload_type,
                        U32 timestamp,
                        U16 data_len,
                        U8 *data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Get service capability form the list.
 *
 * INPUT:
 *      BTS2E_AV_SERV_CAP svc_cap:
 *      U8 *list:
 *      U16 len:
 *      U16 *idx:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 *av_get_svc_cap(BTS2E_AV_SERV_CAP svc_cap,
                   U8 *list,
                   U16 len,
                   U16 *idx);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Validate the service capability.
 *
 * INPUT:
 *      U8 *svc_cap_ptr: pointer to service capability.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 av_vldate_svc_cap(U8 *svc_cap_ptr);

void av_set_stream_buffize(U16 stream_buffize);

U16 av_get_stream_buffize(void);

U16 av_get_max_stream_buffer_cnt(void);

void a2dp_reset_req(void);

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
