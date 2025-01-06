/**
  ******************************************************************************
  * @file   gap_api.h
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

#ifndef _GAP_API_H_
#define _GAP_API_H_
//add for pan project start
#include "bts2_msg.h"
//add for pan project stop
#ifdef __cplusplus
extern "C" {
#endif

#define ACT_MODE                (0x0000)
#define HOLD_MODE               (0x0001)
#define SNIFF_MODE              (0x0002)
#define PARK_MODE               (0x0003)

#define GAP_RECV_MSG_BASE       0x0000
#define GAP_SEND_MSG_BASE       BTS2MU_START
#define BTS2MU_GAP_MSG_EXT      0x8100
enum
{
    BTS2MU_GAP_SET_LOCAL_NAME_CFM = BTS2MU_START, // 0x8000
    BTS2MU_GAP_RD_LOCAL_BD_ADDR_CFM,
    BTS2MU_GAP_WR_LINK_SUPERV_TIMEOUT_CFM,
    BTS2MU_GAP_RD_LOCAL_NAME_CFM,
    BTS2MU_GAP_RD_TX_POWER_LEVEL_CFM,
    BTS2MU_GAP_GET_LINK_QA_CFM, // 0x8005
    BTS2MU_GAP_RD_RSSI_CFM,
    BTS2MU_GAP_WR_COD_CFM,
    BTS2MU_GAP_RD_COD_CFM,
    BTS2MU_GAP_RD_LOCAL_VERSION_CFM,
    BTS2MU_GAP_RD_RMT_VERSION_CFM, // 0x800A
    BTS2MU_GAP_WR_SCAN_ENB_CFM,
    BTS2MU_GAP_RD_SCAN_ENB_CFM,

    BTS2MU_GAP_RD_RMT_EXT_FEATR_CFM,
    BTS2MU_GAP_RD_CLOCK_CFM,
    BTS2MU_GAP_ENB_DUT_MODE_CFM,
    BTS2MU_GAP_ROLE_DISCOV_CFM, // 0x8010
    BTS2MU_GAP_RD_LINK_POLICY_CFM,
    BTS2MU_GAP_WR_LINK_POLICY_ERR_IND,
    BTS2MU_GAP_SDC_SVC_RECORD,
    BTS2MU_GAP_APP_INIT_CFM,
    GAP_MSG_NUM, // 0x8015

    BTS2MU_GAP_DISCOV_RES_IND = GAP_MSG_NUM,
    BTS2MU_GAP_DISCOV_CFM, // 0x8015
    BTS2MU_GAP_ESC_DISCOV_CFM,
    BTS2MU_GAP_RD_RMT_NAME_CFM,
    BTS2MU_GAP_WR_PAGE_TO_CFM,
    GAP_DISCOV_MSG_MAX_NUM,
    BTS2MU_GAP_KEYMISSING_IND, // 0x801A
    BTS2MU_GAP_ENCRYPTION_IND,
    BTS2MU_GAP_ACL_OPEN_IND,
    BTS2MU_GAP_SDC_SERVICE_RECORD_HANDLE,
    BTS2MU_GAP_SDC_SRCH_CFM,  // ofsset 30
    BTS2MU_GAP_SDC_CLOSE_IND,

    BTS2MU_GAP_MODE_CHANGED_IND, // 0x8020
    GAP_API_MSG_MAX_NUM, // 0x8021
    BTS2MU_GAP_SCO_CONNECT_REQ_IND = BTS2MU_GAP_MSG_EXT + 1,
};

/* HCI pin code len */
#ifndef HCI_MIN_PIN_LEN
#define HCI_MIN_PIN_LEN  ((U8)0x01)
#define HCI_MAX_PIN_LEN  ((U8)0x10)
#endif

#ifndef SEC_MODE0_OFF
#define SEC_MODE0_OFF                   ((U8)0x00)
#define SEC_MODE1_NON_SECURE            ((U8)0x01)
#define SEC_MODE2_SVC                   ((U8)0x02)
#define SEC_MODE3_LINK                  ((U8)0x03)
#endif

#define SEC_MODE4_SVC                   ((U8)0x04)


#ifndef BTS2S_PORT_PART_DEF
#define BTS2S_PORT_PART_DEF
typedef struct
{
    U8   port_speed;
    U8   data_bit;
    U8   stop_bit;
    U8   parity;
    U8   parity_type;
    U8   flow_ctrl_mask;
    U8   xon;
    U8   xoff;
    U16  par_mask;
} BTS2S_PORT_PAR;
#endif

typedef struct
{
    U16               type;
    U8                res;
    U16               vld_rsp;
    U16               total_rsp;
} BTS2S_GAP_DISCOV_CFM;

typedef struct
{
    U16               type;
    BTS2S_BD_ADDR     bd;
    U32               dev_cls;
    BTS2S_CONN_INFO   conn_info;
    BTS2S_DEV_NAME    dev_disp_name;
    U8                device_service_len;
    U8                device_service[100];
} BTS2S_GAP_DISCOV_RES_IND;

typedef struct
{
    U16               type;
    U8                res;
} BTS2S_GAP_ESC_DISCOV_CFM;

typedef struct
{
    U16               type;
    BTS2S_BD_ADDR     bd;
} BTS2S_GAP_KEYMISSING_IND;

typedef struct
{
    U16               type;
    BTS2S_BD_ADDR     bd;
} BTS2S_GAP_ENCRYPTION_IND;



typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U24 cod;
} BTS2S_GAP_ACL_OPEN_IND;

typedef struct
{
    U16               type;
    U8                res;
} BTS2S_GAP_SET_LOCAL_NAME_CFM;

typedef struct
{
    U16               type;
    BTS2S_BD_ADDR     bd;
    U8                res;
    U8                power_level;
} BTS2S_GAP_RD_TX_POWER_LEVEL_CFM;

typedef struct
{
    U16               type;
    BTS2S_BD_ADDR     bd;
    U8                res;
    U8                link_qa;
} BTS2S_GAP_GET_LINK_QA_CFM;

typedef struct
{
    U16          type;
    BTS2S_BD_ADDR bd;
    U8           res;
    S8           rssi; /* -128 ~ 127 */
} BTS2S_GAP_RD_RSSI_CFM;

typedef struct
{
    U16                 type;
    U8                  res;
    BTS2S_BD_ADDR       bd;
    BTS2S_DEV_NAME      dev_disp_name;
} BTS2S_GAP_RD_RMT_NAME_CFM;

typedef struct
{
    U16                 type;
    U8                  res;
    BTS2S_BD_ADDR       bd;
    U8                  lmp_version;
    U16                 manufacturer_name;
    U16                 lmp_subversion;
} BTS2S_GAP_RD_RMT_VERSION_CFM;

typedef struct
{
    U16                 type;
    BTS2S_BD_ADDR       bd;
} BTS2S_GAP_RD_LOCAL_BD_ADDR_CFM;


typedef struct
{
    U16                 type;
    BTS2S_DEV_NAME      local_name;
} BTS2S_GAP_RD_LOCAL_NAME_CFM;

typedef struct
{
    U16                 type;
    U8                  res;
} BTS2S_GAP_WR_COD_CFM;

typedef struct
{
    U16             type;
    U8              res;
    U24             dev_cls;
} BTS2S_GAP_RD_COD_CFM;

typedef struct
{
    U16             type;
    BTS2S_BD_ADDR   bd;
    U8              role;
} BTS2S_GAP_ROLE_DISCOV_CFM;


typedef struct
{
    U16             type;
    U32             svc;
    U32             *svc_hdl_list;
    U16             svc_num;
    U8              local_srv_chnl;
    BTS2S_BD_ADDR   bd;
} BTS2S_GAP_SDC_SRCH_IND;

typedef struct
{
    U16             type;
    U8              local_srv_chnl;
    BTS2S_BD_ADDR   bd;
} BTS2S_GAP_SDC_SRCH_CFM;

typedef struct
{
    U16             type;
    U8              res;
    U8              local_srv_chnl;
    BTS2S_BD_ADDR   bd;
} BTS2S_GAP_SDC_CLOSE_IND;

typedef struct
{
    U16             type;
    U16             uuid_num;
    U32             svc_hdl;
    U32            *uuid_list;
    U32             svc_name_len;
    U8             *svc_name;
    U8              rfc_chl;
    BTS2S_BD_ADDR   bd;
} BTS2S_GAP_SDC_SVC_RECORD;

typedef struct
{
    U16             type;
    U8              res;
    U8              step_num;
} BTS2S_GAP_ENB_DUT_MODE_CFM;

typedef struct
{
    U16             type;
    U8              res; /* HCI result code, defined in bts2_bt.h */
    U8              inquiry_scan;
    U8              page_scan;
} BTS2S_GAP_WR_SCAN_ENB_CFM;

typedef struct
{
    U16             type;
    U8              res;
    U8              scan_enb;
} BTS2S_GAP_RD_SCAN_ENB_CFM;

typedef struct
{
    U16             type;
    U8              res;
    BTS2S_BD_ADDR   bd;
} BTS2S_GAP_WR_LINK_SUPERV_TIMEOUT_CFM;

typedef struct
{
    U16             type;
} BTS2S_GAP_WR_PAGE_TO_CFM;

typedef struct
{
    U16               type;
    U8                res;
    BTS2S_BD_ADDR     bd;
    U32               clock;
    U16               accuracy;
} BTS2S_GAP_RD_CLOCK_CFM;

typedef struct
{
    U16                 type;
    U8                  res;
    U8                  lmp_version;
    U16                 manufacturer_name;
    U16                 lmp_subversion;
} BTS2S_GAP_RD_LOCAL_VERSION_CFM;

#ifndef BTS2S_HOLD_SETTINGS_DEF
#define BTS2S_HOLD_SETTINGS_DEF
typedef struct
{
    U16 max_intvl;
    U16 min_intvl;
} BTS2S_HOLD_SETTINGS;
#endif

#ifndef BTS2S_SNIFF_SETTINGS_DEF
#define BTS2S_SNIFF_SETTINGS_DEF
typedef struct
{
    U16 max_intvl;
    U16 min_intvl;
    U16 attmpt;
    U16 timeout;
} BTS2S_SNIFF_SETTINGS;
#endif

#ifndef BTS2S_PARK_SETTINGS_DEF
#define BTS2S_PARK_SETTINGS_DEF
typedef struct
{
    U16 max_intvl;
    U16 min_intvl;
    U32 park_idle_time;
} BTS2S_PARK_SETTINGS;
#endif

typedef struct
{
    U16                         type;
    BTS2S_BD_ADDR               bd;
    U8                          res;
    U8                          actual_mode;
    U16                         link_policy_setting;
    BTS2S_SNIFF_SETTINGS        sniff_setting;
    BTS2S_HOLD_SETTINGS         hold_setting;
} BTS2S_GAP_RD_LINK_POLICY_CFM;

typedef struct
{
    U16                         type;
    BTS2S_BD_ADDR               bd;
    U8                          res;
} BTS2S_GAP_WR_LINK_POLICY_ERR_IND;


typedef struct
{
    U16                         type;
    BTS2S_BD_ADDR               bd;
    U8 st;
    U8 mode;
    U16 interval;
} BTS2MU_GAP_MODE_CHANGED_IND_t;

typedef struct
{
    U16                         type;
    BTS2S_BD_ADDR               bd;
    U8                          code_id;
} BTS2S_GAP_SCO_REMOTE_CONN_REQ;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to get the completed event for stack.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_LOCAL_NAME_CFM with structure BTS2S_GAP_RD_LOCAL_NAME_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_app_init_req(U16 tid);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function will activate a Bluetooth Inquire process.
 *
 * INPUT:
 *      tid:      Task identification.
 *      max_rsp:  Maxium number of responses from the inquiry before the inquiry is halted.
 *                0 is used for unlimited number of responses.
 *      timeout:  Maxium amount of time specified before the inquiry procedure halted
 *                with SECOND. 0 is used for unlimited time.
 *      filter:   Set the filtering of responses or not. Once set, a device with a specific
 *                Class of Device responded to the Inquiry process.
 *      name_flag:if the reading name of the discoveried remote device should be performed.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_DISCOV_RES_IND with structure BTS2S_GAP_DISCOV_RES_IND will be receive.
 *      Main members in this structure include Bluetooth address,Device Class,
 *      Device display name,etc.When finishing the inquire process, the caller will
 *      recieve message BTS2MU_GAP_DISCOV_CFM which indicate "Inquire finish".
 *
 *----------------------------------------------------------------------------*/
void gap_discov_req(U16 tid,
                    U8 max_rsp,
                    U16 timeout,
                    BTS2S_CPL_FILTER *filter,
                    BOOL name_flag);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to end the inquiry process.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      BTS2MU_GAP_ESC_DISCOV_CFM with structure BTS2S_GAP_ESC_DISCOV_CFM will be
 *      receive.
 *
 *----------------------------------------------------------------------------*/
void gap_esc_discov_req(U16 tid);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to set local Bluetooth device name.
 *
 * INPUT:
 *      tid: Task identification.
 *      name: New name of local device.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_SET_LOCAL_NAME_CFM with structure BTS2S_GAP_SET_LOCAL_NAME_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_set_local_name_req(U16 tid, BTS2S_DEV_NAME name);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read local Bluetooth device name.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_LOCAL_NAME_CFM with structure BTS2S_GAP_RD_LOCAL_NAME_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_local_name_req(U16 tid);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read remote Bluetooth device name.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of remote device.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_RMT_NAME_CFM with structure BTS2S_GAP_RD_RMT_NAME_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_rmt_name_req(U16 tid, BTS2S_BD_ADDR bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read remote Bluetooth version.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of remote device.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_RMT_VERSION_CFM with structure BTS2S_GAP_RD_RMT_VERSION_CFM
 *      will be received. Main members in this structure include LMP version,LMP subversion
 *      and manufacturer name.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_rmt_version_req(U16 tid, BTS2S_BD_ADDR bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read local Bluetooth device address.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_LOCAL_BD_ADDR_CFM with structure BTS2S_GAP_RD_LOCAL_BD_ADDR_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_local_bd_req(U16 tid);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to write link supervision timeout.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of the device.
 *      timeout: Link supervision timeout value.
 *               0x0000: No link supervision timeout.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_WR_LINK_SUPERV_TIMEOUT_CFM with structure
 *      BTS2S_GAP_WR_LINK_SUPERV_TIMEOUT_CFM will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_wr_super_vision_timeout_req(U16 phdl,
                                     BTS2S_BD_ADDR bd,
                                     U16 timeout);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read Transmit Power Level.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of the device to read.
 *      level_type: The maximum power level as defined in the Bluetooth HCI specification.
 *                  0X00 means Read Current Transmit Power Lever.
 *                  0x01 means Read Maximum Transmit Power Lever.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_TX_POWER_LEVEL_CFM with structure BTS2S_GAP_RD_TX_POWER_LEVEL_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_tx_power_level_req(U16 tid,
                               BTS2S_BD_ADDR bd,
                               U8 level_type);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read Link Quality.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of the device to read.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_GET_LINK_QA_CFM with structure BTS2S_GAP_GET_LINK_QA_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_get_link_qa_req(U16 tid, BTS2S_BD_ADDR bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read rssi.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of the device to read.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_RSSI_CFM with structure BTS2S_GAP_RD_RSSI_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_rssi_req(U16 tid, BTS2S_BD_ADDR bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to write device class.
 *
 * INPUT:
 *      tid: Task identification.
 *      dev_cls: Class of Device for the device.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_WR_COD_CFM with structure BTS2S_GAP_WR_COD_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_wr_dev_cls_req(U16 tid, U24 dev_cls);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read device class.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_COD_CFM with structure BTS2S_GAP_RD_COD_CFM
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_dev_cls_req(U16 tid);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to write scan enable.
 *
 * INPUT:
 *      tid: Task identification.
 *      inquiry_scan: TRUE enable inquiry scan,FALSE disable inquiry scan.
 *      page_scan: TRUE enable page scan,FALSE disable page scan.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_WR_SCAN_ENB_CFM with structure
 *      BTS2S_GAP_WR_SCAN_ENB_CFM will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_wr_scan_enb_req(U16 tid, BOOL inquiry_scan, BOOL page_scan);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read scan enable.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_SCAN_ENB_CFM with structure
 *      BTS2S_GAP_RD_SCAN_ENB_CFM will be received.
 *
*----------------------------------------------------------------------------*/
void gap_rd_scan_enb_req(U16 tid);
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to close the SDP search request.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_SDC_CLOSE_IND with structure BTS2S_GAP_SDC_CLOSE_IND
 *      will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_sdc_close_req(U16 tid, BTS2S_BD_ADDR bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to enable Device Under Test Mode.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_ENB_DUT_MODE_CFM with structure
 *      BTS2S_GAP_ENB_DUT_MODE_CFM will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_enb_dut_mode_req(U16 tid);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to write page timeout.
 *
 * INPUT:
 *      tid: Task identification.
 *      page_timeout: the timeout value.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_WR_PAGE_TO_CFM with structure
 *      BTS2S_GAP_WR_PAGE_TO_CFM will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_wr_page_timeout_req(U16 tid, U16 page_timeout);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read Clock.
 *
 * INPUT:
 *      tid: Task identification.
 *      clock: the clock which should be read.
 *             0x00: Local Clock (Connection Handle does not have to be a valid handle).
 *             0x01: Piconet Clock (Connection Handle shall be a valid ACL Handle).
 *             0x02 to 0xFF: Reserved.
 *      bd: Bluetooth address of the device to read.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_CLOCK_CFM with structure
 *      BTS2S_GAP_RD_CLOCK_CFM will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_clock_req(U16 tid, U8 clock, BTS2S_BD_ADDR bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read local version.
 *
 * INPUT:
 *      tid: Task identification.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_LOCAL_VERSION_CFM with structure
 *      BTS2S_GAP_RD_LOCAL_VERSION_CFM will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_local_version_req(U16 tid);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to role discovery.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of the device.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_ROLE_DISCOV_CFM with structure
 *      BTS2S_GAP_ROLE_DISCOV_CFM will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_role_discov_req(U16 app_hdl, BTS2S_BD_ADDR bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to write link policy.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of the device to write link policy.
 *      link_policy_setting: Link setting policy.
 *                           0x0000: Disable all LM Modes.
 *                           0x0001: Enable role switch.
 *                           0x0002: Enable hold mode.
 *                           0x0003: Enable sniff mode.
 *                           0x0004: Enable park mode.
 *                           0x0010-0x8000: Reserved for future used.
 *      park_setting: park mode setting.
 *      setup_link_policy_setting: TRUE enable link policy setting,False disable.
 *      sniff_setting: sniff mode setting.
 *      hold_setting: hold mode setting.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void gap_wr_link_policy_req(U16 app_hdl,
                            BTS2S_BD_ADDR bd,
                            U16 link_policy_setting,
                            BOOL setup_link_policy_setting,
                            BTS2S_SNIFF_SETTINGS *sniff_setting,
                            BTS2S_HOLD_SETTINGS *hold_setting);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to read link policy.
 *
 * INPUT:
 *      tid: Task identification.
 *      bd: Bluetooth address of the device to read.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_GAP_RD_LINK_POLICY_CFM with structure
 *      BTS2S_GAP_RD_LINK_POLICY_CFM will be received.
 *
 *----------------------------------------------------------------------------*/
void gap_rd_link_policy_req(U16 app_hdl, BTS2S_BD_ADDR bd);

void gap_user_passkey_request_reply(U8 bFlag, BTS2S_BD_ADDR *bd, U32 passky_val);
void gap_rmt_oob_data_request_reply(U8 bFlag, BTS2S_BD_ADDR *bd, U8 *hash_c, U8 *rand_r);
void gap_wr_eir_req(U8 uflag, U8 *eir_data, U32 len);


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
void gap_switch_role_req(BTS2S_BD_ADDR bd, U8 role);
int8_t gap_disconnect_req(BTS2S_BD_ADDR *bd);
int8_t gap_close_req(BTS2S_BD_ADDR *bd);
int8_t gap_cancel_connect_req(BTS2S_BD_ADDR *bd);
void gap_open_req(void);
int8_t gap_sync_conn_res(BTS2S_BD_ADDR *bd, U8 code_id, U8 accept);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send Baseband reset HCI command (03 0C 00) to controller hardware.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      No message can be received, application should handle it by itself.
 *
 *----------------------------------------------------------------------------*/
void gap_hw_reset(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Register a callback function.
 *
 * INPUT:
 *      start: 1 to start o to stop
 *      cb_fn: function pointer.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      The function is used to send Vendor command when BTS2 in startup process..
 *
 *----------------------------------------------------------------------------*/
typedef void(vendor_cmd_cb_fn_type)(void);

void gap_reg_hci_vendor_cmd_callback(BOOL start, vendor_cmd_cb_fn_type *cb_fn);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Register a callback function.
 *
 * INPUT:
 *      cb_fn: function pointer.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
typedef void(vendor_cb_fn_type)(void *, U8);

void gap_reg_hci_vendor_ev_callback(vendor_cb_fn_type *cb_fn);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send vendor-specific command.
 *
 * INPUT:
 *      chnl: chnl id of sending command.
 *      cmd: point to memory address of data.
 *      len: the length of data.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void gap_send_hci_verdor_cmd(U8 chnl, U8 *cmd, U16 len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Register sco callback function.
 *
 * INPUT:
 *      sco_hdl: sco task id.
 *      fn: function pointer.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
typedef void (sco_cb_fn_type)(U16, U8, U8 *);

BOOL gap_reg_sco_callback(U16 sco_hdl, sco_cb_fn_type *fn);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Un-egister sco callback function.
 *
 * INPUT:
 *      sco_hdl: sco task id.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void gap_unreg_sco_callback(U16 sco_hdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Send audio data to hardware.
 *
 * INPUT:
 *      U16 sco_hdl, U8 sco_len, U8 *data
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      the data should be in packet format of audio.
 *
 *----------------------------------------------------------------------------*/
void gap_send_audio_data(U16 sco_hdl, U8 sco_len, U8 *data);

/*----------------------------------------------------------------------------*
 * PAR:
 *      U8 dir
 *          1 tx
 *          0 rx
 *      U8 type
 *          1 command
 *          2 acl
 *          3 sco
 *          4 event
 *      U16 len
 *      U8* data
 *      BTS2S_BD_ADDR* Bluetooth device address, only used for ACL data
 *----------------------------------------------------------------------------*/
typedef void (bts2_data_info_cb_fn_type)(U8 dir, U8 type, U16 len, const U8 *data, BTS2S_BD_ADDR *bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Register BTS2 data information callback function.
 *
 * INPUT:
 *      fn: function pointer.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Once the function pointer is NULL which can be used as unregister.
 *      The data pointer MUST not be free or modified which still be used by BTS2.
 *      The bd is only provided for ACL data type not SCO type currently.
 *      The callback function MUST be in high performance.
 *----------------------------------------------------------------------------*/
void gap_reg_bts2_data_info_callback(bts2_data_info_cb_fn_type *fn);


void gap_set_receive_avdtp_connect_request(BOOL flag);
void gap_set_receive_avdtp_connect_addr(BTS2S_BD_ADDR bd);
void gap_reset_receive_avdtp_connect_addr(void);
BOOL gap_get_receive_avdtp_connect_request(void);
BOOL gap_check_receive_avdtp_connect_addr(BTS2S_BD_ADDR *bd);
void gap_set_sco_res_by_app_flag(U8 flag);
void gap_set_receive_avrcp_connect_request(BOOL flag);
BOOL gap_get_receive_avrcp_connect_request(void);
#ifdef __cplusplus
}
#endif
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
