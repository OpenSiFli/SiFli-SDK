/**
  ******************************************************************************
  * @file   hfp_hf_api.h
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

#ifndef _HFP_HF_API_H_
#define _HFP_HF_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_VGS_VAL            (15)
#define MAX_VGM_VAL            (15)
#define HF_MSG_BASE            0x0000


#define CIEV_NAME_LEN         20
#define CHLD_LEN              20

enum
{
    BTS2MU_HF_ENB_CFM = BTS2MU_START,
    BTS2MU_HF_DISB_CFM,
    BTS2MU_HF_CONN_IND,
    BTS2MU_HF_CONN_CFM,
    BTS2MU_HF_DISC_CFM,
    BTS2MU_HF_DISC_IND,
    BTS2MU_HF_AUDIO_CFM,
    BTS2MU_HF_AUDIO_IND,
    BTS2MU_HF_AT_CMD_CFM,
    BTS2MU_HF_REQ_ACCEPT_CFM,
    BTS2MU_HF_SCO_RENEGOTIATE_IND,
    BTS2MU_HF_SCO_RENEGOTIATE_CFM,
    BTS2MU_HF_SPK_GAIN_IND,
    BTS2MU_HF_MIC_GAIN_IND,
    BTS2MU_HF_AT_DATA_IND,
    BTS2MU_HF_CHLD_IND,             /*AT+CHLD?*/
    BTS2MU_HF_COPP_SRV_QUERY_IND,   /*AT+COPS?*/
    BTS2MU_HF_CLCC_IND,
    BTS2MU_HF_CNUM_IND,
    BTS2MU_HF_CLIP_IND,
    BTS2MU_HF_BINP_IND,
    BTS2MU_HF_BTRH_IND,             /*AT+BTRH=0,1,2... and AT_BTRH?*/
    BTS2MU_HF_VOICE_RECOG_IND,      /*AT+BVRA=0,1*/
    BTS2MU_HF_CCWA_IND,
    BTS2MU_HF_RING_IND,
    BTS2MU_HF_STS_IND,
    BTS2MU_HF_CIEV_IND,
    BTS2MU_HF_CMEE_IND,
    BTS2MU_HF_AG_EVENT_IND,
    BTS2MU_HF_BRSF_IND,
    BTS2MU_HF_CIND_IND,
    BTS2MU_HF_BSIR_IND,
};

/* AT Command enum */
enum
{
    HFP_HF_AT_NONE = 0x00,
    HFP_HF_AT_BRSF,
    HFP_HF_AT_BAC,
    HFP_HF_AT_CIND,
    HFP_HF_AT_CIND_STATUS,
    HFP_HF_AT_CMER,
    HFP_HF_AT_CHLD,
    HFP_HF_AT_CHLD_CMD,
    HFP_HF_AT_CMEE,
    HFP_HF_AT_BIA,
    HFP_HF_AT_CLIP,
    HFP_HF_AT_CCWA,
    HFP_HF_AT_COPS,
    HFP_HF_AT_COPS_CMD,
    HFP_HF_AT_CLCC,
    HFP_HF_AT_BVRA,
    HFP_HF_AT_VGS,
    HFP_HF_AT_VGM,
    HFP_HF_AT_ATD,
    HFP_HF_AT_BLDN,
    HFP_HF_AT_ATA,
    HFP_HF_AT_CHUP,
    HFP_HF_AT_BTRH,
    HFP_HF_AT_BTRH_MODE,
    HFP_HF_AT_VTS,
    HFP_HF_AT_BCC,
    HFP_HF_AT_BCS,
    HFP_HF_AT_CNUM,
    HFP_HF_AT_NREC,
    HFP_HF_AT_BINP,
    HFP_HF_AT_CBC,
    HFP_HF_AT_BIND,
    HFP_HF_AT_BIEV,
    HFP_HF_AT_BATT_UPDATE,
    HFP_HS_AT_CKPD,
    HFP_AT_EXTERN_AT_CMD
};

typedef struct
{
    U16 type;
    U8  at_cmd_id;
    U8  res;
} BTS2S_HF_AT_CMD_CFM;

typedef struct
{
    U16 type;
    S16 command_id;
    S16 val;
    U8  payload[1];
} BTS2S_HF_AT_CMD;

typedef struct
{
    U16 type;
    U16 payload_len;
    U8  cmd_id;
    U8 *payload;
} HFP_HF_AT_CMD_INFO;

typedef enum
{
    HF_CONN,
    HS_CONN,
    NO_CONN
} BTS2E_HF_CONN_TYPE;

typedef struct
{
    U16 type;
    U8 res;
    U8 conn_type;
} BTS2S_HF_ENB_CFM;

typedef struct
{
    U16 type;
} BTS2S_HF_DISB_CFM;

typedef struct
{
    U16 type;
    U8 srv_chnl;
    BTS2S_BD_ADDR bd;
} BTS2S_HF_CONN_IND;

typedef struct
{
    U16 type;
    U8 res;
    U8 srv_chnl;
    BTS2S_BD_ADDR bd;
} BTS2S_HF_CONN_RSP;

typedef struct
{
    U16 type;
    U8 conn_type;
    U8 res;
    U8 rfcomm_channel;
    U32 supp_featr;
    BTS2S_DEV_NAME svc_name;
    BTS2S_BD_ADDR bd;
} BTS2S_HF_CONN_CFM;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR cur_bd;
    U8 res;
} BTS2S_HF_DISC_CFM;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR cur_bd;
    U8 res;
} BTS2S_HF_DISC_IND;

typedef struct
{
    U16 type;
    BOOL audio_on;
} BTS2S_HF_AUDIO_CFM;

typedef struct
{
    U16 type;
    U16 type_req;  // request id not accept
    U8 res;
} BTS2S_HF_REQ_CFM;

typedef struct
{
    U16 type;
    U16 sco_hdl;
    /* 0x00 Connection successfully reconfigured. */
    U8 res;
    /* Time between two consecutive SCO/eSCO instants measured in slots. */
    U8 tx_intvl;
    /* The size of the retransmission window measured in slots. Shall be zero for SCO links. */
    U8 we_sco;
    /* Length in bytes of the SCO/eSCO payload in the receive direction. */
    U16 rx_pkt_len;
    /* Length in bytes of the SCO/eSCO payload in the transmit direction. */
    U16 tx_pkt_len;
} BTS2S_HF_SCO_RENEGOTIATE_IND;

typedef struct
{
    U16 type;
    U16 sco_hdl;
    /* 0x00 Connection successfully reconfigured. */
    U8 res;
    /* Time between two consecutive SCO/eSCO instants measured in slots. */
    U8 tx_intvl;
    /* The size of the retransmission window measured in slots. Shall be zero for SCO links. */
    U8 we_sco;
    /* Length in bytes of the SCO/eSCO payload in the receive direction. */
    U16 rx_pkt_len;
    /* Length in bytes of the SCO/eSCO payload in the transmit direction. */
    U16 tx_pkt_len;
} BTS2S_HF_SCO_RENEGOTIATE_CFM;


typedef struct
{
    U16 type;
    U8 gain;
} BTS2S_HF_SPK_GAIN_IND;

typedef struct
{
    U16 type;
    U8  gain;
} BTS2S_HF_MIC_GAIN_IND;

typedef struct
{
    U16 type;
    U16 payload_len;
    U8  *payload;
} BTS2S_HF_AT_DATA_IND;

typedef struct
{
    U16  type;
    char chld_str[CHLD_LEN + 1];
    U8   supp;
} BTS2S_HF_CHLD_IND;

typedef struct
{
    U16 type;
    U8  res;
    U8  mode;
    U8  fmt;
    U8  data_len;
    U8  *data;
} BTS2MD_HF_COPP_SRV_QUERY_IND;

typedef struct
{
    U16 type;
    U8  idx;
    U8  dir;
    U8  st;
    U8  mode;
    U8  mpty;
    U32 data_len;
    U8  *data;
    U32 body_len;
    U8  *body;
} BTS2S_HF_CLCC_IND;

typedef struct
{
    U16 type;
    U16 phone_len;
    U8 phone_type;
    U8 *phone_number;
} BTS2S_HF_CNUM_IND;

typedef struct
{
    U16 type;
    U16 phone_len;
    U8 phone_type;
    U8 *phone_number;
} BTS2S_HF_CLIP_IND;

typedef struct
{
    U16 type;
    U8 res;
} BTS2S_HF_BINP_CFM;

typedef struct
{
    U16 type;
    U8 res;
    U16 phone_len;
    U8 phone_type;
    U8 *phone_number;
} BTS2S_HF_BINP_IND;

typedef struct
{
    U16 type;
    U8  val;
} BTS2S_HF_BTRH_IND;

typedef struct
{
    U16 type;
    U8  val;
} BTS2S_HF_VOICE_RECOG_IND;


typedef struct
{
    U16 type;
    U8 res;
    U8 phone_type;
    U16 phone_len;
    U8 *phone_number;
} BTS2S_HF_CCWA_IND;

typedef struct
{
    U16 type;
    U8 res;
} BTS2S_HF_COMMON_CFM;

typedef struct
{
    U16 type;
} BTS2S_HF_RING_IND;

typedef struct
{
    U16 type;
    U8  st_ev;
} BTS2S_HF_ST_IND;

typedef struct
{
    U16  type;
    char name[CIEV_NAME_LEN + 1];
    U8   val;
} BTS2S_HF_CIEV_IND;

typedef struct
{
    U16  type;
    bts2_hfp_hf_cind cind_status;
} BTS2S_HF_CALL_STATUS_IND;

typedef struct
{
    U16 type;
    U8  cmee_err_code;
} BTS2S_HF_CMEE_IND;

typedef struct
{
    U16 type;
    U16  supp_feature;
} BTS2S_HF_BRSF_IND;


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Activate the Hands-Free service. This signal is used to allowed remote
 *      device to find and discover the Hands-Free service and subsequently connect
 *      to it.
 *
 * INPUT:
 *      U32 supp_featr: announce the feature of local device.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_ENB_CFM with structure BTS2S_HF_ENB_CFM will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_enb_req(U32 supp_featr);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Deactivate the Hands-Free service. This signal deactivate the Hands-Free
 *      service and remove the service records in SDP database.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_DISB_CFM with structure BTS2S_HF_DISB_CFM will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_disb_req(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send a connection request to a device specified by the bluetooth address.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: address of device to connect to.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_CONN_CFM with structure BTS2S_HF_CONN_CFM will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_conn_req(BTS2S_BD_ADDR *bd, U8 conn_type);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Answer/Reject the incoming connection.
 *
 * INPUT:
 *      BTS2S_BD_ADDR bd: Remote Bluetooth address.
 *      U8 srv_chnl: Local server channel.
 *      U8 res: TRUE indicate answer,False indicate reject.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_CONN_RSP with structure BTS2S_HF_CONN_RSP will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_conn_rsp(BTS2S_BD_ADDR *bd, U8 srv_chnl, U8 res);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send a disconnect request to remote device.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_DISC_CFM with structure BTS2S_HF_DISC_CFM will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_disc_req();

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Connect/disconnect the sco path on the established service level connection
 *
 * INPUT:
 *      BOOL audio_on: TRUE/FALSE to indicate whether audio should be turned on or off
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AUDIO_CFM with structure BTS2S_HF_AUDIO_CFM will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_audio_transfer_req(BOOL audio_on);


/*-------------------------------------------------------------------------------------------AT_CMD FUNC------------------------------------------------------------------------*/




/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      get remote cind status about call_status callsetup_status and so on
 *
 * INPUT:

 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with cmd_id HFP_HF_AT_CIND_STATUS will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_cind_status_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      indicator update status enable or not
 *
 * INPUT:
 *       active: 1 or TRUE ; enable indicator update status
 *       active: 0 or FALSE ; disenable indicator update status
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with cmd_id HFP_HF_AT_CMER will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/

void hfp_hf_send_at_cmer_api(BOOL active);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Call hold and multiparty handling AT command.
 *
 * INPUT:
 *      U8 *payload: this specification only covers values for <n> of 0, 1, 1<idx>,
 *              2, 2<idx>, 3 and 4, where:
 *
 *              0 = Releases all held calls or sets User Determined User Busy
 *                  (UDUB) for a waiting call.
 *              1 = Releases all active calls (if any exist) and accepts the other
 *                  (held or waiting) call.
 *              1<idx> = Releases specified active call only (<idx>).
 *              2 = Places all active calls (if any exist) on hold and accepts
 *                  the other (held or waiting) call.
 *              2<idx> = Request private consultation mode with specified call
 *                  (<idx>). (Place all calls on hold EXCEPT the call indicated by <idx>.)
 *              3 = Adds a held call to the conversation.
 *              4 = Connects the two calls and disconnects the subscriber from
 *                  both calls (Explicit Call Transfer). Support for this value
 *                  and its associated functionality is optional for the HF.
 *     payload_len: payload len
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with cmd_id HFP_HF_AT_CHLD_CMD will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_chld_control_api(U8 *payload, U8 payload_len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The HF shall execute this procedure to enable/disable Extended Audio
 *      Gateway Error result codes in the AG.
 * INPUT:
 *      val: 1 (enable cmee) 0 (disable cmee)
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_CMEE will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_cmee_api(BOOL val);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Enable calling Line Identification (CLI) Notification.
 *
 * INPUT:
 *      active: 1 (enable clip) 0 (disable clip)
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_CLIP will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_clip_api(U8 active);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Enable calling Line Identification (CLI) Notification.
 *
 * INPUT:
 *      active: 1 (enable ccwa) 0 (disable ccwa)
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_CCWA will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_ccwa_api(U8 active);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Set Network operator name format to long alphanumeric.
 *
 * INPUT:
 *     payload: "3,0"
 *       1.mode: The first parameter, always 3, indicates that this command is
 *               only affecting the format parameter.
 *       2.fmt: The second parameter, always 0, is the value required to set the
 *              format to long alphanumeric.
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_COPS_CMD will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_cops_cmd_api(U8 *payload, U8 payload_len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Query List of Current Calls in AG.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_CLCC will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_clcc_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Voice Recognition Activation.
 *
 * INPUT:
 *      U8 flag: TRUE:enable,FALSE:disable
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_BVRA will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_bvra_api(U8 active);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *       Send a request to change the speaker gain.
 *
 * INPUT:
 *      U8 vol: new speak gain. vol must 0~15
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_VGS will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_vgs_api(U8 vol);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send a request to change the microphone gain.
 *
 * INPUT:
 *      U8 vol: new microphone gain. vol must 0~15
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_VGM will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_vgm_api(U8 vol);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Dial a phone number out.
 *
 * INPUT:
 *      U8 *payload:phone numer + ";" or idx + ";"
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_ATD will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/

void hfp_hf_send_at_atd_api(U8 *payload, U8 payload_len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      last number redial
 *
 * INPUT:
 *      void
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_BLDN will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/

void hfp_hf_send_at_bldn_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      accept or answer phone call
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_ATA will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_ata_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      reject phone call
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_CHUP will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_chup_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Query Response and Hold Status.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_BTRH will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_btrh_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Bluetooth Response and Hold Feature.
 *
 * INPUT:
 *      U8 mode: 0, 1, 2 entered as integer values, where
 *               0 = Put Incoming call on hold
 *               1 = Accept a held incoming call
 *               2 = Reject a held incoming call
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_BTRH_MODE will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_btrh_mode_api(U8 mode);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Transmit DTMF Codes.
 *
 * INPUT:
 *      U8 val: 0 ~ 9,and '*','#' are effected.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_VTS will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_vts_api(char payload);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *       Retrieve AG Subscriber Number Information.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_CNUM will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/

void hfp_hf_send_at_cnum_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The HF Requests Turning Off the AG's EC and NR.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_NREC will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_nrec_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Attach a Phone Number to a Voice Tag.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_BINP will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_send_at_binp_api(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      update battery val to remote device
 *
 * INPUT:
 *      U8 payload: 1,1,%d(1,1,: iphone spec define; %d: batt_level:0---9)
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HF_AT_BATT_UPDATE will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/

void hfp_hf_send_at_batt_update_api(U8 *payload, U8 payload_len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Head set at cmd AT+CKPD=200/r.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_HS_AT_CKPD will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hs_send_at_ckpd_api(void);


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send extern AT command.
 *
 * INPUT:
 *      U8 *payload: Point to at command string.
 *      U16 len: The length of payload.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_HF_AT_CMD_CFM with structure HFP_AT_EXTERN_AT_CMD will
 *      be received as a confirmation.
 *----------------------------------------------------------------------------*/
void hfp_hf_at_data_req(U8 *payload, U16 payload_len);

void hfp_hf_send_at_bia_api(void);//reserve
void hfp_hf_set_wbs(U8 wbs_flag);


#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
