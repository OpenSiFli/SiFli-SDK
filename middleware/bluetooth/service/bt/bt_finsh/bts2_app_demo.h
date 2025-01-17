/**
  ******************************************************************************
  * @file   bts2_app_demo.h
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

#ifndef _BTS2_APP_DEMO_H_
#define _BTS2_APP_DEMO_H_

#include <sys/stat.h>
#include <stdio.h>
#include "rtthread.h"
//#include "log.h"

//add for pan project start
#include "bts2_type.h"
#include "bts2_bt.h"
#include "gap_api.h"


#include "hfp_type_api.h"


//#if defined(BSP_BTS2_PAN) || defined(BT_FINSH_PAN)
//#define CFG_PAN
//#endif
//add for pan project end

#ifdef __cplusplus
extern "C" {
#endif

#define RETURN_KEY        0x0D
#define RETURN_ENTER      0x0A
#define BACKSPACE_KEY     0x08
#define CLS               "cls"
#define ESCAPE_KEY        0x1B

//#define SPP_MAX_NUM 2
#define SPP_INSTANCE_UNDEFINED 0xFF

/*default link device */
//#define CFG_BD_NAP                  (0x0018)
//#define CFG_BD_UAP                  (0x11)
//#define CFG_BD_LAP                  (0x920182)

//20220727
#define CFG_BD_NAP                  (0xffff)
#define CFG_BD_UAP                  (0xff)
#define CFG_BD_LAP                  (0xffffff)

#define CBR_230400                  230400
#define CBR_460800                  460800
#define CBR_691200                  691200
#define CBR_921600                  921600

#define KEY_MSG                     0x0CBA

#define MAX_ENB_TIME                (0)
#define KEYB_CHK_TIMEOUT            (200*MILLISECOND)

#define MAX_DISCOV_RESS             (10)

#define MAX_ONLY_FILE_NAME_LEN      (260)

#define TIME_WITH_NO_DATA           (3000*MILLISECOND)

#ifndef CFG_TL_USB
#define CFG_COM_PORT                "COM1"
#define CFG_COM_PORT_BAUD_RATE      115200
#endif


#define CFG_PATH_SYB            '\\'


#define CFG_PIN_CODE               "0000"

typedef enum
{
    BTS_APP_IDLE,
    BTS_APP_STACK_READY,
    BTS_APP_PROFILE_READY,
} bts_app_state_t;


#ifdef CFG_SPP_CLT
typedef enum
{
    spp_clt_idle,
    spp_clt_conned
} bts2_spp_clt_st;
#endif

#ifdef CFG_SPP_SRV
typedef enum
{
    spp_srv_idle,
    spp_srv_enbd,
    spp_srv_conned
} bts2_spp_srv_st;
#endif

#ifdef CFG_SPP_CLT
typedef struct
{
    bts2_spp_clt_st  spp_clt_st;
    U8               spp_id;/*Record current spp instance id*/
    U16              mfs;
    U8               sending;
    BTS2S_BD_ADDR    bd_addr;
    FILE             *cur_file_hdl;
    FILE             *wr_file_hdl;
    U32              cur_file_pos;
    U32              cur_file_size;
    char             file_name[MAX_ONLY_FILE_NAME_LEN];
    char             rd_file_name[20];
    U32              start_timer;
    U32              last_time;
    U32              time_id;
    U8               timer_flag;
    U32              counter;
} bts2_spp_inst_data;
#endif
#ifdef CFG_SPP_SRV
typedef struct
{
    U8               sending;
    U8               srv_chnl;
    FILE             *cur_file_hdl;
    FILE             *wr_file_hdl;
    U32              cur_file_pos;
    U32              cur_file_size;
    char             file_name[MAX_ONLY_FILE_NAME_LEN];
    char             rd_file_name[20];
    U32              start_timer;
    U32              last_time;
    U32              time_id;
    U8               timer_flag;
    U32              counter;
    U32         rand_total_len;
    U16         rand_packet_len;
    U8          *rand_buf;
    U8          is_rand_going;
    U8          receive_first_packet;
    U16         mtu_size;
    void                    *next_struct;
} bts2_spp_service_list;

typedef struct
{
    BTS2S_BD_ADDR           bd_addr;
    U8                      device_id;
    U8                      cur_link_mode;
    BOOL                    exit_sniff_pending;
    U16                     spp_srv_mfs;
    U32                     cod;
    U32                     service_list;
    bts2_spp_service_list *spp_service_list;
} bts2_spp_srv_inst_data;
#endif

#ifdef CFG_HFP_HF
typedef enum
{
    hfp_idle,       /* Initializing state */
    hfp_enbd,       /* Enable service state */
    hfp_disb,       /* Disable service state */
    hfp_conned,     /* SLC connected */
    hfp_calling     /* An active call */
} bts2_hfp_st;
#endif

#ifdef CFG_HFP_AG
typedef enum
{
    HFP_AG_APP_INIT,       /* Initializing state */
    HFP_AG_APP_OPENING,       /* Enable service state */
    HFP_AG_APP_OPEN,       /* Disable service state */
    HFP_AG_APP_CLOSING,     /* SLC connected */
} bts2_ag_st;

typedef struct
{
    bts2_ag_st st;
    bts2_ag_st old_st;
    hfp_device_state_t profile_state;
    hfp_device_state_t pre_profile_state;
    hfp_call_state_t call_state;
    U8 srv_chnl;
} bts2_hfp_ag_inst_data;
#endif

#ifdef CFG_HFP_HF
typedef struct
{
    bts2_hfp_st st;
    /*
      conn_type 0 indicate hf connect
      conn_type 1 indicate hs connect
     */
    U8 conn_type;
    U8 srv_chnl;
    U8 esco_flag;
    BOOL voice_flag;

    U16 peer_features;
    U16 sco_hdl;
    BTS2S_BD_ADDR         hfp_bd;
    bts2_hfp_hf_cind cind_status;
} bts2_hfp_hf_inst_data;
#endif

#ifdef BT_FINSH_PAN
#define PAN_MAX_NUM 2
typedef enum
{
    PAN_IDLE_ST,
    PAN_REG_ST,
    PAN_SDS_REG_ST,
    PAN_BUSY_ST
} bts2_pan_st;

typedef struct
{
    BTS2S_BD_ADDR bd_addr;
    U8 gn_sdp_pending;
    U8 gn_sdp_fail;
    U8 nap_sdp_pending;
    U8 nap_sdp_fail;
} bts2_pan_sdp_service;

typedef struct
{
    bts2_pan_st pan_st;
    U16 id;
    BTS2S_BD_ADDR bd_addr;
    U16 local_role;
    U16 rmt_role;
    bts2_pan_sdp_service pan_sdp[PAN_MAX_NUM];
    U8 mode;
} bts2_pan_inst_data;
#endif
#ifdef CFG_AVRCP
typedef enum
{
    avrcp_idle,
    avrcp_conned
} bts2_avrcp_st;

typedef struct
{
    bts2_avrcp_st  st;
    BTS2S_BD_ADDR  rmt_bd;
    U8             release_type;
    rt_timer_t     avrcp_time_handle;
    rt_timer_t     avrcp_vol_time_handle;
    rt_sem_t       volume_change_sem;
    U8             tgRegStatus;//0:TG has not register absolute volume; 1:TG has register absolute volume.
    U8             tgRegStatus1;
    U8             ab_volume;//the absolute volue be set.
    U8             tgTlable;
    U8             tgTlable_1;
    U8             abs_volume_pending;
    U8             playback_status;
} bts2_avrcp_inst_data;
#endif


#ifdef CFG_HID
typedef enum
{
    hid_idle,
    hid_conned
} bts2_hid_st;

typedef struct
{
    bts2_hid_st  st;
    BTS2S_BD_ADDR  rmt_bd;
    BOOL is_hid_device_role;
    rt_timer_t     hid_time_handle;
    rt_timer_t     hid_time_handle_drag_up;
    rt_timer_t     hid_time_handle_drag_down;
    rt_timer_t     hid_time_handle_reset_at_middle1;
    rt_timer_t     hid_time_handle_reset_at_middle2;
    rt_timer_t     hid_time_handle_reset_at_middle_ios;
    rt_timer_t     hid_time_handle_reset_at_middle_ios1;
    rt_timer_t     hid_time_handle_reset_report;
    U8        local_protocol_mode;
    U8        mode;
    void     *device_input_data;
    void     *device_output_data;
    void     *device_feature_data;
} bts2_hid_inst_data;
#endif
#ifdef CFG_BT_L2CAP_PROFILE
typedef struct
{
    U8 is_use;
    U8 device_state;
    U16 cid;
    U16 remote_mtu;
    U16 sco_hdl;
    BTS2S_BD_ADDR bd;
} bts2_bt_l2cap_device_info_inst;

typedef struct
{
    U8 conn_num;
    U8 reg_status;
    U16 local_psm;
    U16 local_mtu;
    bts2_bt_l2cap_device_info_inst device[CFG_MAX_ACL_CONN_NUM];
} bts2_bt_l2cap_inst_data_t;
#endif

/* app structure */
typedef struct
{
    /* name should prefix with app_ */
    U8                menu_id;
    U8                input_str_len;
    U8                input_str[512];
    U8                dev_idx;
    bts_app_state_t   state;

    void              *recv_msg;
    BOOL              inquiry_flag;
    U8                bd_list_num;
    U8                inquiry_list_num;
    BTS2S_BD_ADDR     bd_list[MAX_DISCOV_RESS];
    BTS2S_BD_ADDR     inquiry_list[MAX_DISCOV_RESS];
    //20220727
    BTS2S_BD_ADDR     last_conn_bd;
    BTS2S_BD_ADDR     pair_bd;
    BTS2S_BD_ADDR     local_bd;
    U8                pin_code_len;
    U8                pin_code[HCI_MAX_PIN_LEN];
    //U16               sdc_app_hdl;
    U16               phdl;

    /* HFP HF */
#ifdef CFG_HFP_HF
    bts2_hfp_hf_inst_data *hfp_hf_ptr;
    bts2_hfp_hf_inst_data hfp_hf_inst;
    U8                    esco_flag;
#endif
#ifdef CFG_HFP_AG
    bts2_hfp_ag_inst_data  hfp_ag_inst;
#endif
    /* SPP */
    /* name should prefix with spp_ */
#ifdef CFG_SPP_CLT
    bts2_spp_inst_data *inst_ptr;
    bts2_spp_inst_data spp_inst[SPP_CLT_MAX_CONN_NUM];
    U8 select_spp_clt_id;
    U8 spp_conn_nums;
    int file_name_len;
#endif

#ifdef CFG_SPP_SRV
    bts2_spp_srv_inst_data *spp_srv_inst_ptr;
    bts2_spp_srv_inst_data spp_srv_inst[CFG_MAX_ACL_CONN_NUM];
    U8 select_device_id;
    U8 select_srv_chnl;
    U8 spp_srv_conn_nums;
#endif

#ifdef CFG_AVRCP
    bts2_avrcp_inst_data avrcp_inst;
#endif

#ifdef CFG_HID
    bts2_hid_inst_data hid_inst;
#endif

#ifdef BT_FINSH_PAN
    bts2_pan_inst_data pan_inst;
    bts2_pan_inst_data *pan_inst_ptr;
#endif

#ifdef CFG_BT_L2CAP_PROFILE
    bts2_bt_l2cap_inst_data_t  bt_l2cap_profile_inst;
#endif
    //****************COMMOND SETTINGS*******************
    U8                 pre_menu_id;
    //U8                 conn_count;    //connections count
    //U8                 connect_mode;  //connect  mode
    //U8                 sec_mode;      //security  mode
    //U8                 sec_auth;      //security  authortiy
    //U8                 sec_encry;     //security  encription

    //U16                local_cls;           //local device services class
    //BTS2S_DEV_NAME     local_dmname[20];    //local device name
    //BTS2S_BD_ADDR      local_db;            //local device services class
    //U8                 cfg_fname[MAX_ONLY_FILE_NAME_LEN];       //setting filename
    //U8                 cfg_hist_fname[MAX_ONLY_FILE_NAME_LEN];  //history filename
    //U8                 discover_mode;       //discove mode
    //U8                 manuafacture[20];    //bluetooth chip manuafacture

    //U8                 lmp_version;
    //U8                 hci_version;
    //U8                 l2cap_version;
    U8                 scan_mode;
} bts2_app_stru;

void key_msg_svc(void);
void hdl_keyb_msg(U8 thechar, bts2_app_stru *bts2_app_data);
void bt_init_profile(bts2_app_stru *bts2_app_data);
void app_fn_init(void **pp);
void app_fn_rel(void **pp);
void app_fn_hdl(void **pp);
void app_fn_hdl_ext(void **pp);

extern void bt_log_output(uint8_t tag, char *fmt, ...);

/* used by upper layer to get application global structure */
bts2_app_stru *getApp(void);

#if defined(BSP_BT_CUSTOMIZE_LOG) && !defined(BSP_BT_LOG_OFF)
#define USER_TRACE_ON
#define INFO_TRACE_ON
// #define BT_SPP_DEBUG_ENABLE
#endif

/* used to trace menu/key information have to displayed to the end user */
#if defined(USER_TRACE_ON)
#define USER_TRACE(...)  bt_log_output(BT_TAG_I, __VA_ARGS__)
#else // !USER_TRACE_ON
#ifdef BSP_BT_LOG_OFF
#define USER_TRACE(...)
#else // !BSP_BT_LOG_OFF
#define USER_TRACE       LOG_I
#endif // BSP_BT_LOG_OFF
#endif // USER_TRACE_ON



#if defined(BT_SPP_DEBUG_ENABLE)
#define BT_SPP_DEBUG(...)    bt_log_output(BT_TAG_D, __VA_ARGS__)
#else // !BT_SPP_DEBUG_ENABLE
#ifdef BSP_BT_LOG_OFF
#define BT_SPP_DEBUG(...)
#else // !BSP_BT_LOG_OFF
#define BT_SPP_DEBUG         LOG_D
#endif // BSP_BT_LOG_OFF
#endif // BT_SPP_DEBUG_ENABLE

/* used to trace debug information helpful for the developer */
#if defined(INFO_TRACE_ON)
#define INFO_TRACE(...)  bt_log_output(BT_TAG_D, __VA_ARGS__)
#else // !INFO_TRACE_ON
#ifdef BSP_BT_LOG_OFF
#define INFO_TRACE(...)
#else // !BSP_BT_LOG_OFF
#define INFO_TRACE       LOG_D
#endif // BSP_BT_LOG_OFF
#endif // INFO_TRACE_ON


#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
