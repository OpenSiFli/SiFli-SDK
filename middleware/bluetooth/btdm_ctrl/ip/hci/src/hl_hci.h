/**
  ******************************************************************************
  * @file   hl_hci.h
  * @author Sifli software development team
  * @brief Host HCI header file.
  * @{
  ******************************************************************************
*/
/**
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


#ifndef HL_HCI_H
#define HL_HCI_H


/// Message API of the HCI task
/*@TRACE*/
enum hl_hci_msg_id
{
    HL_HCI_INQ_CMP_EVT,
    HL_HCI_INQ_RES_EVT,
    HL_HCI_CON_CMP_EVT,
    HL_HCI_CON_REQ_EVT,
    HL_HCI_DISC_CMP_EVT,
    HL_HCI_AUTH_CMP_EVT,
    HL_HCI_REM_NAME_REQ_CMP_EVT,
    HL_HCI_ENC_CHG_EVT,
    HL_HCI_CHG_CON_LK_CMP_EVT,
    HL_HCI_MASTER_LK_CMP_EVT,
    HL_HCI_RD_REM_SUPP_FEATS_CMP_EVT,
    HL_HCI_RD_REM_VER_INFO_CMP_EVT,
    HL_HCI_QOS_SETUP_CMP_EVT,
    HL_HCI_CMD_CMP_EVT,
    HL_HCI_CMD_STAT_EVT,
    HL_HCI_HW_ERR_EVT,
    HL_HCI_FLUSH_OCCURRED_EVT,
    HL_HCI_ROLE_CHG_EVT,
    HL_HCI_NB_CMP_PKTS_EVT,
    HL_HCI_MODE_CHG_EVT,
    HL_HCI_RETURN_LINK_KEYS_EVT,
    HL_HCI_PIN_CODE_REQ_EVT,
    HL_HCI_LK_REQ_EVT,
    HL_HCI_LK_NOTIF_EVT,
    HL_HCI_DATA_BUF_OVFLW_EVT,
    HL_HCI_MAX_SLOT_CHG_EVT,
    HL_HCI_RD_CLK_OFF_CMP_EVT,
    HL_HCI_CON_PKT_TYPE_CHG_EVT,
    HL_HCI_QOS_VIOL_EVT,
    HL_HCI_PAGE_SCAN_REPET_MODE_CHG_EVT,
    HL_HCI_FLOW_SPEC_CMP_EVT,
    HL_HCI_INQ_RES_WITH_RSSI_EVT,
    HL_HCI_RD_REM_EXT_FEATS_CMP_EVT,
    HL_HCI_SYNC_CON_CMP_EVT,
    HL_HCI_SYNC_CON_CHG_EVT,
    HL_HCI_SNIFF_SUB_EVT,
    HL_HCI_EXT_INQ_RES_EVT,
    HL_HCI_ENC_KEY_REFRESH_CMP_EVT,
    HL_HCI_IO_CAP_REQ_EVT,
    HL_HCI_IO_CAP_RSP_EVT,
    HL_HCI_USER_CFM_REQ_EVT,
    HL_HCI_USER_PASSKEY_REQ_EVT,
    HL_HCI_REM_OOB_DATA_REQ_EVT,
    HL_HCI_SP_CMP_EVT,
    HL_HCI_LINK_SUPV_TO_CHG_EVT,
    HL_HCI_ENH_FLUSH_CMP_EVT,
    HL_HCI_USER_PASSKEY_NOTIF_EVT,
    HL_HCI_KEYPRESS_NOTIF_EVT,
    HL_HCI_REM_HOST_SUPP_FEATS_NOTIF_EVT,
    HL_HCI_LE_META_EVT,
    HL_HCI_MAX_EVT_MSK_PAGE_1,
    HL_HCI_TRIGGERED_CLOCK_CAPTURE,
    HL_HCI_SYNC_TRAIN_CMP_EVT,
    HL_HCI_SYNC_TRAIN_REC_EVT,
    HL_HCI_CON_SLV_BCST_REC_EVT,
    HL_HCI_CON_SLV_BCST_TO_EVT,
    HL_HCI_TRUNC_PAGE_CMP_EVT,
    HL_HCI_SLV_PAGE_RSP_TO_EVT,
    HL_HCI_CON_SLV_BCST_CH_MAP_CHG_EVT,
    HL_HCI_AUTH_PAYL_TO_EXP_EVT,
    HL_HCI_SAM_STATUS_CHANGE_EVT,
    HL_HCI_MAX_EVT_MSK_PAGE_2,
    HL_HCI_DBG_META_EVT,
    HL_HCI_EVT,
    HL_HCI_LE_EVT,
    HL_HCI_ISO_DATA,
    HL_HCI_ACL_DATA,
    HL_HCI_VOHCI_DATA,
    HL_HCI_DBG_EVT,
    HL_HCI_DBG_ASSERT_EVT,
    HL_HCI_DBG_LE_CON_EVT_TRACE_EVT,
    HL_HCI_DBG_BT_RSSI_NOTIFY_EVT,
    HL_HCI_UNDEF = 0xff,
};


enum hl_hci_le_evt_subcode
{
    /// LE Events Subcodes
    HL_HCI_LE_CON_CMP_EVT                 = 0x01,
    HL_HCI_LE_ADV_REPORT_EVT              = 0x02,
    HL_HCI_LE_CON_UPDATE_CMP_EVT          = 0x03,
    HL_HCI_LE_RD_REM_FEATS_CMP_EVT        = 0x04,
    HL_HCI_LE_LTK_REQUEST_EVT             = 0x05,
    HL_HCI_LE_REM_CON_PARAM_REQ_EVT       = 0x06,
    HL_HCI_LE_DATA_LEN_CHG_EVT            = 0x07,
    HL_HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT = 0x08,
    HL_HCI_LE_GEN_DHKEY_CMP_EVT           = 0x09,
    HL_HCI_LE_ENH_CON_CMP_EVT             = 0x0A,
    HL_HCI_LE_DIR_ADV_REP_EVT             = 0x0B,
    HL_HCI_LE_PHY_UPD_CMP_EVT             = 0x0C,
    HL_HCI_LE_EXT_ADV_REPORT_EVT          = 0x0D,
    HL_HCI_LE_PER_ADV_SYNC_EST_EVT        = 0x0E,
    HL_HCI_LE_PER_ADV_REPORT_EVT          = 0x0F,
    HL_HCI_LE_PER_ADV_SYNC_LOST_EVT       = 0x10,
    HL_HCI_LE_SCAN_TIMEOUT_EVT            = 0x11,
    HL_HCI_LE_ADV_SET_TERMINATED_EVT      = 0x12,
    HL_HCI_LE_SCAN_REQ_RCVD_EVT           = 0x13,
    HL_HCI_LE_CH_SEL_ALGO_EVT             = 0x14,
    HL_HCI_LE_CONLESS_IQ_REPORT_EVT       = 0x15,
    HL_HCI_LE_CON_IQ_REPORT_EVT           = 0x16,
    HL_HCI_LE_CTE_REQ_FAILED_EVT          = 0x17,
    HL_HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT = 0x18,
    HL_HCI_LE_CIS_ESTABLISHED_EVT         = 0x19,
    HL_HCI_LE_CIS_REQUEST_EVT             = 0x1A,
    HL_HCI_LE_CREATE_BIG_CMP_EVT          = 0x1B,
    HL_HCI_LE_TERMINATE_BIG_CMP_EVT       = 0x1C,
    HL_HCI_LE_BIG_SYNC_ESTABLISHED_EVT    = 0x1D,
    HL_HCI_LE_BIG_SYNC_LOST_EVT           = 0x1E,
    HL_HCI_LE_REQ_PEER_SCA_CMP_EVT        = 0x1F,
    HL_HCI_LE_PATH_LOSS_THRESHOLD_EVT     = 0x20,
    HL_HCI_LE_TX_POWER_REPORTING_EVT      = 0x21,
    HL_HCI_LE_BIG_INFO_ADV_REPORT_EVT     = 0x22,
};

#define HL_HCI_INVALID_LID 0xff
#define HL_HCI_NOT_TL 0xffff

typedef void hci_done_callback(void);
uint16_t em_addr_get(uint8_t *p_em_buf);

/**
    * @brief Get host transport dest from connection handle.
    * @param conhdl connection handle.
    * @retval host transport dest, see HCI_MSG_HOST_TL_DEST.
*/
uint8_t hl_hci_get_host_tl_dest_from_conhdl(uint16_t conhdl);

/**
    * @brief Get host transport dest from hci event code.
    * @param opcode event code
    * @retval host transport dest, see HCI_MSG_HOST_TL_DEST.
*/
uint8_t hl_hci_cmd_evt_get_host_tl_dest(uint16_t opcode);

/*
    * @brief Send unpacked event directly to host stack..
    * @param evt_type event type, see hl_hci_msg_id
    * @param opcode LE subcode if it is HCI_LE_EVENT
    * @param param Parameter for event.
    * @retval host transport dest, see HCI_MSG_HOST_TL_DEST.
*/
void hl_hci_send_cmd_evt_to_host(uint8_t dest, uint8_t evt_type, uint16_t opcode, uint16_t evt_len, void *param);

/*
    * @brief Send Packed event/data to host stack..
    * @param type hci message type, see hci_msg_type
    * @param p_buf HCI data buffer
    * @param len length of HCI data buffer.
    * @param cbk Callback when sent.
*/
void hl_hci_pk_send_to_host(uint8_t type, uint8_t *p_buf, uint16_t len, hci_done_callback cbk);


/*
    * @brief Send event to host stack..
    * @param evt_type event type, see hl_hci_msg_id
    * @param opcode LE subcode if it is HCI_LE_EVENT
    * @param host_lid Local identifier used to select the hci handler。
    * @param param Parameter for event.
*/
void hl_hci_send_evt_to_host(uint8_t evt_type, uint16_t opcode, uint16_t conn_hdl, uint16_t evt_len, void *param);

/*
    * @brief Send data to host stack..
    * @param evt_type event type, see hl_hci_msg_id
    * @param param Parameter for event.
*/
void hl_hci_send_data_to_host(uint8_t evt_type, void *param, uint8_t is_tl);

/*
    * @brief Mark next command/event with opcode for bt host
    * @param opcode Command/event to mark.
    * @retval 1: Mark success  0:Failed.
*/
int hl_hci_mark_host_bt(uint16_t opcode);


#endif //HL_HCI_H
