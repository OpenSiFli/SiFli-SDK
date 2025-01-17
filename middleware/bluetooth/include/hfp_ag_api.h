
/**
  ******************************************************************************
  * @file   hfp_ag_api.h
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

#ifndef _HFP_AG_API_H_
#define _HFP_AG_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hfp_type_api.h"


/****************************************enum define*************************************************/
typedef enum
{
    HFP_AG_SPK_EVT      =  7,       /* Speaker volume changed */
    HFP_AG_MIC_EVT      =  8,       /* Microphone volume changed */
    HFP_AG_AT_CKPD_EVT  =  9,       /* CKPD from the HS */
    HFP_AG_AT_A_EVT     = 10,       /* Answer a call */
    HFP_AG_AT_D_EVT     = 11,       /* Place a call using number or memory dial */
    HFP_AG_AT_CHLD_EVT  = 12,       /* Call hold */
    HFP_AG_AT_CHUP_EVT  = 13,       /* Hang up a call */
    HFP_AG_AT_CIND_EVT  = 14,       /* Read indicator settings */
    HFP_AG_AT_VTS_EVT   = 15,       /* Transmit DTMF tone */
    HFP_AG_AT_BINP_EVT  = 16,       /* Retrieve number from voice tag */
    HFP_AG_AT_BLDN_EVT  = 17,       /* Place call to last dialed number */
    HFP_AG_AT_BVRA_EVT  = 18,       /* Enable/disable voice recognition */
    HFP_AG_AT_NREC_EVT  = 19,       /* Disable echo canceling */
    HFP_AG_AT_CNUM_EVT  = 20,       /* Retrieve subscriber number */
    HFP_AG_AT_BTRH_EVT  = 21,       /* CCAP-style incoming call hold */
    HFP_AG_AT_CLCC_EVT  = 22,       /* Query list of current calls */
    HFP_AG_AT_COPS_EVT  = 23,       /* Query list of current calls */
    HFP_AG_AT_UNAT_EVT  = 24,       /* Unknown AT command */
    HFP_AG_AT_CBC_EVT   = 25,       /* Battery Level report from HF */
    HFP_AG_AT_BAC_EVT   = 26,       /* avablable codec */
    HFP_AG_AT_BCS_EVT   = 27,       /* Codec select */
    HFP_AG_AT_BIND_EVT  = 28,       /* HF indicator */
    HFP_AG_AT_BIEV_EVT  = 29,       /* HF indicator updates from peer */
    HFP_AG_DISABLE_EVT  = 30,       /* AG disabled */
    HFP_AG_WBS_EVT      = 31,       /* SCO codec info */
    HFP_AG_BATT_EVT     = 32,       /* BATT from hf*/
} hfp_ag_at_command_evt;

enum
{
    BTS2MU_AG_ENB_CFM = BTS2MU_START,
    BTS2MU_AG_DISB_CFM,
    BTS2MU_AG_CONN_RES,
    BTS2MU_AG_DISC_RES,
    BTS2MU_AG_CONN_STATE,
    BTS2MU_AG_AUDIO_CFM,
    BTS2MU_AG_AUDIO_IND,
    BTS2MU_AG_SCO_RENEGOTIATE_IND,
    BTS2MU_AG_SCO_RENEGOTIATE_CFM,
    BTS2MU_AG_AT_CMD_EVENT,
};

/****************************************struct define*************************************************/
typedef struct
{
    U16 type;
} BTS2S_HFP_CMD_TYPE;

typedef struct
{
    U16 type;
    U32 supp_featr;
} BTS2S_HFP_ENB_REQ;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U16 rfcomm_chnl; //for reconnect not need sdp search
} BTS2S_AG_CONN_REQ;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
} BTS2S_AG_CONN_INFO;

typedef struct
{
    U16 type;
    S16 command_id;
    S16 val;
    char str[1];
} BTS2S_AG_AT_CMD_INFO;

typedef struct
{
    U16 type;
    U8 res;
} BTS2S_AG_CFM;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U8 device_state;
    U8 res;
} BTS2S_AG_CONN_RES;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U8 audio_on;
    U8 res;
} BTS2S_AG_AUDIO_CONN_CFM;
/****************************************func define*************************************************/
/*******************************************************************************
 *
 * Function         hfp_ag_register
 *
 * Description     Initialize the bluetooth HF AG module and init local features
 *
 * Returns         void
 *
 ******************************************************************************/
void hfp_ag_register(U32 local_features);

/*******************************************************************************
 *
 * Function         hfp_ag_deregister
 *
 * Description     De-initialize for HF AG module
 *
 * Returns         void
 *
 ******************************************************************************/
void hfp_ag_deregister(void);

/*******************************************************************************
 *
 * Function         hfp_ag_connect
 *
 * Description      Opens a connection to a headset or hands-free device.
 *                  When connection is open callback function is called
 *                  with a HFP_AG_OPEN_EVT. Only the data connection is
 *                  opened. The audio connection is not opened.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_connect(BTS2S_BD_ADDR *bd, U16 rfcomm_chnl);

/*******************************************************************************
 *
 * Function         hfp_ag_disconnect
 *
 * Description      To establish a Service Level Connection to remote
 *                  bluetooth HFP client device.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_disconnect(BTS2S_BD_ADDR *bd);

/*******************************************************************************
*
* Function         hfp_ag_connect_audio
*
* Description      Opens an audio connection to the currently connected
*                  headset or hnadsfree.
*
*
* Returns          void
*
******************************************************************************/
void hfp_ag_connect_audio(BTS2S_BD_ADDR *bd);

/*******************************************************************************
*
* Function         hfp_ag_disconnect_audio
*
* Description      Close the currently active audio connection to a headset
*                  or hnadsfree. The data connection remains open
*
*
* Returns          void
*
******************************************************************************/
void hfp_ag_disconnect_audio(BTS2S_BD_ADDR *bd);

/*******************************************************************************
 *
 * Function         start_voice_recognition
 *
 * Description      start voice recognition
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_start_voice_recognition(BTS2S_BD_ADDR *bd);

/*******************************************************************************
 *
 * Function         stop_voice_recognition
 *
 * Description      stop_voice_recognition
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_stop_voice_recognition(BTS2S_BD_ADDR *bd);

/*******************************************************************************
 *
 * Function         phone_state_change
 *
 * Description      notify of a call state change
 *                  number & type: valid only for incoming & waiting call
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_phone_call_status_changed_api(HFP_CALL_INFO_T *p_call_info);


/****************************************AT cmd func define*************************************************/
/*******************************************************************************
 *
 * Function         volume_control
 *
 * Description      speaker volume control
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_spk_volume_control(S8 volume);

/*******************************************************************************
 *
 * Function         volume_control
 *
 * Description      microphone volume control
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_mic_volume_control(S8 volume);

/*******************************************************************************
 *
 * Function         cind_response
 *
 * Description      Response to device individual indicators to HFP Client.
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_cind_response(char *payload, U8 payload_len);

/*******************************************************************************
 *
 * Function         send_indicator_update
 *
 * Description      Send indicator report ��+CIEV: <ind> <value>�� to HFP Client. (CIEV)
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_ind_status_update(char *payload, U8 payload_len);

/*******************************************************************************
 *
 * Function         Enable/disable voice recognition update
 *
 * Description      Enable/disable voice recognition update
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_brva_response(U8 val);

/*******************************************************************************
 *
 * Function         hfp_ag_set_inband
 *
 * Description      Determine whether in-band ring can be used.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_set_inband(U8 val);

/*******************************************************************************
*
* Function        Response for AT+CNUM command from HF Client.
*                 As a precondition to use this API, Service
*                 Level Connection shall exist with HFP client.
*
* Description      registration number
*                  phone type national or international
*
*
*
* Returns          void
*
******************************************************************************/
void hfp_ag_cnum_response(char *payload, U8 payload_len);

/*******************************************************************************
 *
 * Function         hfp_ag_set_btrh
 *
 * Description      Reponse for AT+BRTH command from HF Client.
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_set_btrh(U8 val);

/*******************************************************************************
*
* Function         clcc_response
*
* Description      Response to AT+CLCC command from HFP Client.
*                  Can be iteratively called for each call index. Call index
*                  of 0 will be treated as NULL termination (Completes
*                  response)
*
* Returns          void
*
******************************************************************************/
void hfp_ag_clcc_response(char *payload, U8 payload_len);

/*******************************************************************************
 *
 * Function         cops_response
 *
 * Description      Reponse for AT+COPS command from HF Client.
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_cops_response(char *payload, U8 payload_len);

/*******************************************************************************
 *
 * Function         hfp_ag_set_codec_id
 *
 * Description      Specify the codec type to be used for the subsequent
 *                  audio connection.
 *
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_set_codec_id(U8 code_id);

/*******************************************************************************
 *
 * Function         hfp_ag_clip_response
 *
 * Description      when call is coming send +clip:phone_info to hf
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_clip_response(char *payload, U8 payload_len);

/*******************************************************************************
 *
 * Function         hfp_ag_app_send_ring
 *
 * Description      sned ring
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_app_send_ring(void);

/*******************************************************************************
 *
 * Function         send_at_result
 *
 * Description      Send AT result code (OK/ERROR)
 *
 * Returns          void
 *
 ******************************************************************************/
void hfp_ag_at_cmd_result(U8 res);


void hfp_set_sco_retry_flag(U8 enable);
U8 hfp_get_sco_retry_flag(void);

#ifdef __cplusplus
}
#endif

#endif/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
