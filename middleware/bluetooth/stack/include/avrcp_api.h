/**
  ******************************************************************************
  * @file   avrcp_api.h
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

#ifndef _AVRCP_API_H_
#define _AVRCP_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#define AVRCP_FIXED_MEDIA_PKT_HDR_SIZE    12

/* service capabilities */
typedef enum
{
    AVRCP_SC_MEDIA_TRS = 1,
    AVRCP_SC_REPORTING,
    AVRCP_SC_RECOVERY,
    AVRCP_SC_CONT_PROTECTION,
    AVRCP_SC_HDR_COMPRESSION,
    AVRCP_SC_MULTIPLEXING,
    AVRCP_SC_MEDIA_CODEC,
    AVRCP_SC_NEXT = 0xFFFF
} BTS2E_AVRCP_SERV_CAP;

/* response / error codes */
#define AVRCP_OPEN_STREAM_FAIL   ((U8)0x01)
#define AVRCP_L2CA_CONN_ACPT_ERR ((U8)0x02) /* not possible to make conn_acpt */
#define AVRCP_INVLD_ROLE         ((U8)0x03)

/* command / response types */
#define AVRCP_CR_CTRL            (0x00)
#define AVRCP_CR_STS             (0x01)
#define AVRCP_CR_SPECIFIC_INUIRY (0x02)
#define AVRCP_CR_NOTIFY          (0x03)
#define AVRCP_CR_GENERAL_INQUIRY (0x04)
#define AVRCP_CR_ACPT            (0x09)
#define AVRCP_CR_STABLE          (0x0c)
#define AVRCP_CR_CHANGED         (0x0d)
#define AVRCP_CR_INTERIM         (0x0f)
#define AVRCP_CR_INVLD_PID       (0x10) /* used internally by AVRCP */
#define AVRCP_CR_REJECT          (0x0A)
#define AVRCP_REJECT_ERROR_INVALID_COMMAND 0x00
#define AVRCP_REJECT_ERROR_INVALID_PARAMETER 0x01

/* sub_unit types */
#define AVRCP_UNIT_INFO_SUBUNIT_TYPE    (0x1f)
#define AVRCP_SUBUNIT_INFO_SUBUNIT_TYPE (0x1f)
#define AVRCP_PASS_THROUGH_SUBUNIT_TYPE (0x09)
#define AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE (0x09)

/* sub_unit ID */
#define AVRCP_UNIT_INFO_SUBUNIT_ID    (0x07)
#define AVRCP_SUBUNIT_INFO_SUBUNIT_ID (0x07)
#define AVRCP_PASS_THROUGH_SUBUNIT_ID (0x00)
#define AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID (0x00)

/* opp_cltodes */
#define AVRCP_VENDOR_TYPE       (0x00)
#define AVRCP_UNIT_INFO_TYPE    (0x30)
#define AVRCP_SUBUNIT_INFO_TYPE (0x31)
#define AVRCP_PASS_THROUGH_TYPE (0x7c)


/* AVRCP device roles */
#define AVRCP_TG   ((U16)0x00)
#define AVRCP_CT   ((U16)0x01)

#define AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION 0x31
#define AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_CAPABILITIES 0x10
#define AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_ELEMENT_ATTRIBUTES 0x20
#define AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_PLAY_STATUS 0x30
#define AVRCP_VENDOR_DEPENDENT_PDU_ID_REQUEST_CONTINUING_RESPONSE 0x40
#define AVRCP_VENDOR_DEPENDENT_PDU_ID_ABORT_CONTINUING_RESPONSE 0x41
#define AVRCP_VENDOR_DEPENDENT_PDU_ID_SET_ABSOLUTE_VOLUME 0x50

#define AVRCP_VENDOR_DEPENDENT_EVENT_VOLUME_CHANGED 0x0D

#define AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_COMPANY_ID 0x02
#define AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_FOR_EVENTS 0x03
#define AVRCP_PLAY_STATUS_STOP 0x00
#define AVRCP_PLAY_STATUS_PLAYING 0x01
#define AVRCP_PLAY_STATUS_PAUSED 0x02

// notification events id
#define AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_STATUS_CHANGED 0x01
#define AVRCP_VENDOR_DEPENDENT_EVENT_TRACK_CHANGED 0x02
#define AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_POS_CHANGED 0x05


#define AVRCP_MEDIA_ATTRIBUTES_TITLE 0x01
#define AVRCP_MEDIA_ATTRIBUTES_ARTIST 0x02
#define AVRCP_MEDIA_ATTRIBUTES_ALBUM 0x03
#define AVRCP_MEDIA_ATTRIBUTES_GENRE 0x06
#define AVRCP_MEDIA_ATTRIBUTES_PLAYTIME 0x07

typedef enum BTS2E_AVRCP_MSG_TAG
{
    BTS2MD_AVRCP_ENB_REQ = BTS2MD_START,
    BTS2MD_AVRCP_DISB_REQ,
    BTS2MD_AVRCP_CONN_REQ,
    BTS2MD_AVRCP_DISC_REQ,
    BTS2MD_AVRCP_CMD_FRM_REQ,
    BTS2MD_AVRCP_CMD_FRM_RSP,
    BTS2MD_AVRCP_RESET_REQ,

    BTS2MU_AVRCP_CONN_IND = BTS2MU_START,
    BTS2MU_AVRCP_DISC_IND,
    BTS2MU_AVRCP_UNIT_INFO_CMD_IND,
    BTS2MU_AVRCP_SUBUNIT_INFO_CMD_IND,
    BTS2MU_AVRCP_VENDOR_DEPEND_CMD_IND,
    BTS2MU_AVRCP_PASS_THROUGH_CMD_IND,
    BTS2MU_AVRCP_ENB_CFM,
    BTS2MU_AVRCP_DISB_CFM,
    BTS2MU_AVRCP_CONN_CFM,
    BTS2MU_AVRCP_UNIT_INFO_CMD_CFM,
    BTS2MU_AVRCP_SUBUNIT_INFO_CMD_CFM,
    BTS2MU_AVRCP_VENDOR_DEPEND_CMD_CFM,
    BTS2MU_AVRCP_PASS_THROUGH_CMD_CFM,
    BTS2MU_AVRCP_ERROR_IND,

    //New messages must be added before this
    BTS2M_AVRCP_MAX_MSG_NUM
} AVRCPMSG;

#define BTS2MD_HIGHEST_AVRCP_RECV_MSG_NUM (BTS2MD_AVRCP_RESET_REQ + 1)
#define AVRCP_RECV_MSG_NUM    (BTS2MD_AVRCP_RESET_REQ - BTS2MD_START + 1)
#define AVRCP_SEND_MSG_NUM    (BTS2M_AVRCP_MAX_MSG_NUM - BTS2MU_START)

typedef struct
{
    U16 type;
    U16 tid;
    U16 local_role;
} BTS2S_AVRCP_ENB_REQ;

typedef struct
{
    U16 type;
    U8  res;
} BTS2S_AVRCP_ENB_CFM;

typedef struct
{
    U16 type;
} BTS2S_AVRCP_DISB_REQ;

typedef struct
{
    U16 type;
    U8  res;
} BTS2S_AVRCP_DISB_CFM;

typedef struct
{
    U16          type;
    U16          tid;
    BTS2S_BD_ADDR bd;
    U16          rmt_role;
    U16          local_role;
} BTS2S_AVRCP_CONN_REQ;

typedef struct
{
    U16          type;
    U8           res;
    BTS2S_BD_ADDR bd;
    U16          mfs;
} BTS2S_AVRCP_CONN_CFM;

typedef struct
{
    U16          type;
    BTS2S_BD_ADDR bd;
} BTS2S_AVRCP_CONN_IND;

typedef struct
{
    U16 type;
} BTS2S_AVRCP_DISC_REQ;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U8      res;
} BTS2S_AVRCP_DISC_IND;

typedef struct
{
    U16 type;
    U16 tid;
    U16 cmd_len;
    U8  *cmd_ptr;
} BTS2S_AVRCP_CMD_FRM_REQ;


typedef struct
{
    U16 type;
} BTS2S_AVRCP_RESET_REQ;


typedef BTS2S_AVRCP_CMD_FRM_REQ BTS2S_AVRCP_CMD_FRM_RSP;

typedef struct
{
    U16   type;
    U8    tlabel;
    BOOL  invld_prof;
    U16   prof_id;
    U8    c_type;
    U8    subunit_type;
    U8    subunit_id;
    U16   data_len;
    U8    *data;
} BTS2S_AVRCP_UNIT_INFO_CMD_IND;

typedef BTS2S_AVRCP_UNIT_INFO_CMD_IND BTS2S_AVRCP_SUBUNIT_INFO_CMD_IND;
typedef BTS2S_AVRCP_UNIT_INFO_CMD_IND BTS2S_AVRCP_VENDOR_DEPEND_CMD_IND;
typedef BTS2S_AVRCP_UNIT_INFO_CMD_IND BTS2S_AVRCP_PASS_THROUGH_CMD_IND;
typedef BTS2S_AVRCP_UNIT_INFO_CMD_IND BTS2S_AVRCP_UNIT_INFO_CMD_CFM;
typedef BTS2S_AVRCP_UNIT_INFO_CMD_IND BTS2S_AVRCP_SUBUNIT_INFO_CMD_CFM;
typedef BTS2S_AVRCP_UNIT_INFO_CMD_IND BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM;
typedef BTS2S_AVRCP_UNIT_INFO_CMD_IND BTS2S_AVRCP_PASS_THROUGH_CMD_CFM;

typedef struct
{
    U16 type;
    BOOL  conn;
} BTS2S_AVRCP_STS_IND;

typedef struct
{
    U16 error_type;
    U16 type;
} BTS2S_AVRCP_ERROR_IND;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This signal is used to enable a service and make it accessible from
 *      a remote device.
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
void avrcp_enb_req(U16 tid, U16 local_role);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This signal is used to disable a service and make in inaccessible from
 *      other device.
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
void avrcp_disb_req(void);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      An AVRCP_CONN_REQ will initiate a connect towards a device specified
 *      by the bluetooth device address. the AVRCP will send an AVRCP_CONN_IND back
 *      to the initiator with the res of the connect attmpt.
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
 *      none.
 *
 *----------------------------------------------------------------------------*/
void avrcp_conn_req(U16 tid,
                    BTS2S_BD_ADDR bd,
                    U16 rmt_role,
                    U16 local_role);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request for disconnect of connection previously established.
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
void avrcp_disc_req(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request for sending a unit information
 *
 * INPUT:
 *      U8 tid: task id.
 *      U16 tlable: holds a unit num for this command.
 *      U8 prof_id: 16 bit UUID for the profile addressed.
 *      U8 c_type: CTYPE_CTRL, CTYPE_STS, CTYPE_ACPT.
 *      U8 subunit_type: remote subunit types address.
 *      U8 subunit_id: remote subunit ID types within a subunit type.
 *      U16 data_len: length of data to be sent.
 *      U8 *data: pointer to data.
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void avrcp_cmd_data_req(U16 tid,
                        U8 tlable,
                        U16 prof_id,
                        U8 c_type,
                        U8 subunit_type,
                        U8 subunit_id,
                        U16 data_len,
                        U8 *data);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      U16 msg: message.
 *      U8 tid: task id.
 *      U16 tlable: holds a unit num for this command.
 *      U8 prof_id: 16 bit UUID for the profile addressed.
 *      U8 rps: response type.
 *      U8 subunit_type: remote subunit types address.
 *      U8 subunit_id: remote subunit ID types within a subunit type.
 *      U16 data_len: length of data to be sent.
 *      U8 *data: pointer to data.
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void avrcp_cmd_data_rsp(U16 tid,
                        U8 tlable,
                        U16 prof_id,
                        U8 rsp,
                        U8 subunit_type,
                        U8 subunit_id,
                        U16 data_len,
                        U8 *data);


void avrcp_reset_req(void);

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
