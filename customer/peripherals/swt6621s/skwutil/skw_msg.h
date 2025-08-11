/* SPDX-License-Identifier: GPL-2.0 */

/******************************************************************************
 *
 * Copyright (C) 2020 SeekWave Technology Co.,Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 ******************************************************************************/

#ifndef __SKW_MSG_H__
#define __SKW_MSG_H__

//#include <net/cfg80211.h>
#include "skw_sdio.h"
#include "skw_boot.h"
#include "skw_sdio_core.h"
//#include "skw_type_msg.h"


#define SKW_CMD_MAX_LEN                      1588
#define SKW_CMD_TIMEOUT                      2000

#define SKW_TXBA_COUNT_MASK                  0x3
#define SKW_TXBA_DONE                        BIT(4)
#define SKW_TXBA_WAIT_EVENT                  BIT(5)
#define SKW_TXBA_DISABLED                    BIT(6)

#define SKW_CMD_FLAG_NO_WAKELOCK             0
#define SKW_CMD_FLAG_NO_ACK                  1
#define SKW_CMD_FLAG_IGNORE_BLOCK_TX         2
#define SKW_CMD_FLAG_DONE                    3
#define SKW_CMD_FLAG_XMIT                    4
#define SKW_CMD_FLAG_ACKED                   5
#define SKW_CMD_FLAG_DISABLE_IRQ             6

#define SKW_WLAN_STATUS_SAE_HASH_TO_ELEMENT 126


typedef int (*skw_event_fn)(struct skw_core *, struct skw_iface *, void *, int);

enum SKW_BA_ACTION_CMD
{
    SKW_ADD_TX_BA,
    SKW_DEL_TX_BA,
    SKW_ADD_RX_BA,
    SKW_DEL_RX_BA,
    SKW_REQ_RX_BA,
};

enum SKW_TX_BA_STATUS_CODE
{
    SKW_TX_BA_SETUP_SUCC,
    SKW_TX_BA_REFUSED,
    SKW_TX_BA_TIMEOUT,
    SKW_TX_BA_DEL_EVENT,
    SKW_TX_BA_DEL_SUCC,
};

// struct skw_ba_action {
//  u8 action;
//  u8 lmac_id;
//  u8 peer_idx;
//  u8 tid;
//  u8 status_code;
//  u8 resv[3];
//  u16 ssn;
//  u16 win_size;
// }  ;

struct skw_event_func
{
    int id;
    char *name;
    skw_event_fn func;
};

enum SKW_CSA_COMMAND_MODE
{
    SKW_CSA_DONE,
    SKW_CSA_START,
};

struct skw_event_csa_param
{
    u8 mode;
    u8 bss_type;  /* reference SKW_CAPA_ */
    u8 chan;
    u8 center_chan1;
    u8 center_chan2;
    u8 band_width;
    u8 band;
    u8 oper_class;      /* Only valid for csa start */
    u8 switch_mode;     /* Only valid for csa start */
    u8 switch_count;    /* Only valid for csa start */
    u8 hw_id;
};

enum SKW_EVENT_LOCAL_ID
{
    SKW_EVENT_LOCAL_STA_AUTH_ASSOC_TIMEOUT,
    SKW_EVENT_LOCAL_AP_AUTH_TIMEOUT,
    SKW_EVENT_LOCAL_STA_CONNECT,
    SKW_EVENT_LOCAL_IBSS_CONNECT,

    SKW_EVENT_LOCAL_MAX
};

// enum SKW_MSG_TYPE {
//  SKW_MSG_CMD,
//  SKW_MSG_CMD_ACK,
//  SKW_MSG_EVENT,
//  SKW_MSG_EVENT_LOCAL
// };
#ifndef u_int8_t
    #define u_int8_t unsigned char
#endif
#ifndef u_int16_t
    #define u_int16_t uint16_t
#endif
#ifndef s8
    #define s8 unsigned char
#endif


#define ETH_ALEN    6

struct skw_mgmt_hdr
{
    u8 chan;
    u8 band;
    u16 signal;
    u16 mgmt_len;
    u16 resv;
    u8 mgmt[0];
}  ;

struct skw_tx_mgmt_status
{
    u64 cookie;
    u16 ack;
    u16 payload_len;
    const u8 mgmt;
}  ;

struct skw_enter_roc
{
    u8 inst;
    u8 chn;
    u8 band;
    u64 cookie;
    u32 duration;
}  ;

struct skw_cancel_roc
{
    u8 inst;
    u8 chn;
    u8 band;
    u64 cookie;
}  ;

struct skw_mc_list
{
    u16 count;
    //struct mac_address mac[];
}  ;

struct skw_discon_event_params
{
    u16 reason;
    u8 bssid[ETH_ALEN];
}  ;

/*
 * IEEE 802.2 Link Level Control headers, for use in conjunction with
 * 802.{3,4,5} media access control methods.
 *
 * Headers here do not use bit fields due to shortcommings in many
 * compilers.
 */


struct llc
{
    u_int8_t llc_dsap;
    u_int8_t llc_ssap;
    union
    {
        struct
        {
            u_int8_t control;
            u_int8_t format_id;
            u_int8_t class;
            u_int8_t window_x2;
        }   type_u;
        struct
        {
            u_int8_t num_snd_x2;
            u_int8_t num_rcv_x2;
        }   type_i;
        struct
        {
            u_int8_t control;
            u_int8_t num_rcv_x2;
        }   type_s;
        struct
        {
            u_int8_t control;
            /*
             * We cannot put the following fields in a structure because
             * the structure rounding might cause padding.
             */
            u_int8_t frmr_rej_pdu0;
            u_int8_t frmr_rej_pdu1;
            u_int8_t frmr_control;
            u_int8_t frmr_control_ext;
            u_int8_t frmr_cause;
        }   type_frmr;
        struct
        {
            u_int8_t  control;
            u_int8_t  org_code[3];
            u_int16_t ether_type;
        }   type_snap;
        struct
        {
            u_int8_t control;
            u_int8_t control_ext;
        }   type_raw;
    } llc_un /* XXX   ??? */;
}  ;

struct skw_frame_tx_status
{
    u8 status;
    u8 resv;
    u16 mgmt_len;
    //struct ieee80211_mgmt mgmt[0];
}  ;


enum SKW_OFFLOAD_PAT_OP_TYPE
{
    PAT_OP_TYPE_SAME = 0,
    PAT_OP_TYPE_DIFF = 1,
};

enum SKW_OFFLOAD_PAT_OFFSET
{
    PAT_TYPE_ETH = 0,
    PAT_TYPE_IPV4 = 1,
    PAT_TYPE_UDP = 2,
    PAT_TYPE_TCP = 3,
};

struct skw_pkt_pattern
{
    u8 op;
    u8 type_offset;
    u16 offset;
    u8 len;
    u8 val[0];
}  ;

#define SKW_MAX_WOW_RULE_NUM           2
struct skw_wow_rule
{
    u16 len;
    u8 rule[64];
}  ;

struct skw_wow_input_param
{
    u32 wow_flags;
    u8 rule_num;
    struct skw_wow_rule rules[0];
}  ;

enum SKW_SPD_ACTION_SUBCMD
{
    ACTION_DIS_ALL = 0,
    ACTION_DIS_WOW = 1,
    ACTION_DIS_KEEPALIVE = 2,
    ACTION_EN_WOW = 3,
    ACTION_EN_KEEPALIVE = 4,
    ACTION_EN_ALWAYS_KEEPALIVE = 5,
    ACTION_DIS_ALWAYS_KEEPALIVE = 6,
    ACTION_DIS_ALL_KEEPALIVE = 7,
    ACTION_WOW_KEEPALIVE = 16,
};

struct skw_spd_action_param
{
    u16 sub_cmd;
    u16 len;
}  ;

struct skw_set_monitor_param
{
    u8 mode;
    u8 chan_num;
    u8 bandwidth;
    u8 center_chn1;
    u8 center_chn2;
}  ;

struct skw_rssi_mointor
{
    u32 req_id;
    s8 curr_rssi;
    u8 curr_bssid[ETH_ALEN];
}  ;
#if 0

static inline int skw_cmd_locked(struct semaphore *sem)
{
    unsigned long flags;
    int count;

    raw_spin_lock_irqsave(&sem->lock, flags);
    count = sem->count - 1;
    raw_spin_unlock_irqrestore(&sem->lock, flags);

    return (count < 0);
}
#endif


#define SKW_MAX_INST_NUM                        4
struct skw_inst_nss_info
{
    u8 valid_id;
    u8 inst_rx_nss: 4;
    u8 inst_tx_nss: 4;
}  ;

struct skw_mac_nss_info
{
    u8 mac_nss[SKW_MAX_LMAC_SUPPORT];
    u8 is_dbdc_mode;
    u8 max_inst_num;
    struct skw_inst_nss_info inst_nss[SKW_MAX_INST_NUM];
}  ;

#define SKW_MAX_W_LEVEL                        5
struct skw_hw_w_debug
{
    u16 w_total_cnt;
    u16 w_cnt[SKW_MAX_W_LEVEL];
    u16 w_ctrl_thresh[SKW_MAX_W_LEVEL - 1];
}  ;

struct skw_debug_info_w
{
    struct skw_hw_w_debug hw_w[SKW_MAX_LMAC_SUPPORT];
}  ;

struct skw_debug_info_s
{
    u16 tlv;
    u8 val[1];
}  ;

/* TLV: SKW_MIB_GET_HW_TX_INFO_E */
/* No val data for debug info cmd */
/** Rsp of this TLV **/
struct skw_hw_tx_info_rsp
{
    u16 status;
    u16 tlv;
    u16 len;
    u8 val[0];
}  ;

/*** val[0] start***/
struct tx_hmac_per_hw_rpt_info
{
    u32 hmac_tx_stat[7];
}  ;

struct tx_mib_acc_info
{
    u32 acc_tx_psdu_cnt;
    u16 acc_tx_wi_ack_to_cnt;
    u16 acc_tx_wi_ack_fail_cnt;
    u32 acc_rts_wo_cts_cnt;
    u16 acc_tx_post_busy_cnt;
    u8 tx_en_perct;
}  ;

struct tx_mib_tb_acc_info
{
    /*from tx reg*/
    u32 rx_basic_trig_cnt;
    u32 rx_basic_trig_succ_cnt;

    u32 tb_gen_invoke_cnt;
    u32 tb_gen_abort_cnt;

    u32 tb_cs_fail_acc_cnt;
    u32 all_tb_pre_tx_cnt;

    u32 tb_tx_acc_cnt;
    u32 tb_tx_acc_scucc_cnt;
    u32 tb_tx_suc_ratio;

    /*from rx req*/
    u32 rx_trig_reg_cnt;
    u32 rx_trig_start_tx_cnt;
    u32 rx_trig_suc_ratio;
} ;
/* val definition */
struct skw_get_hw_tx_info_s
{
    u32 hif_tx_all_cnt;
    u32 hif_rpt_all_credit_cnt;
    u32 cur_free_buf_cnt;
    u32 pendding_tx_pkt_cnt;
    u32 lst_cmd_seq;
    u32 lst_evt_seq;
    u32 lmac_tx_bc_cnt;
    u32 lmac_tx_uc_cnt[5];
    struct tx_hmac_per_hw_rpt_info hmac_tx_stat;
    struct tx_mib_acc_info tx_mib_acc_info;
    struct tx_mib_tb_acc_info tx_mib_tb_acc_info;
}  ;
/*** val[0] end***/

/* TLV: SKW_MIB_GET_INST_INFO_E */
/* No val data for debug info cmd */
/** Rsp of this TLV **/
struct skw_inst_info_rsp
{
    u16 status;
    u16 tlv;
    u16 len;
    u8 val[0];
}  ;
/*** val[0] start***/
struct skw_cs_monitor_rpt
{
    u8 p20_lst_busy_perctage;
    u8 p40_lst_busy_perctage;
    u8 p80_lst_busy_perctage;
    u8 p20_lst_nav_perctage;
    u8 p40_lst_nav_perctage;
    u8 p80_lst_nav_perctage;
}  ;

#define RPI_MAX_LEVEL 6
struct skw_rpi_monitor_rpt
{
    int8_t no_uc_direct_rpi_level;
    int8_t uc_direct_rpi_level;
    int8_t rx_ba_rssi;
    u8 rx_idle_percent_in_rpi;
    u8 rx_percent_in_rpi;
    /* measure dur*/
    u32 rpi_class_dur[RPI_MAX_LEVEL];
}  ;

#define NOISE_MAX_LEVEL 6
struct skw_noise_monitor_rpt
{
    u32 noise_class_dur[RPI_MAX_LEVEL];
}  ;

/*val[0]*/
struct skw_get_inst_info_s
{
    u8 inst_mode;
    u8 hw_id;
    u8 enable;
    u8 mac_id;
    u32 asoc_peer_map;
    u32 peer_ps_state;
    struct skw_cs_monitor_rpt cs_info;
    struct skw_rpi_monitor_rpt rpi_info;
    struct skw_noise_monitor_rpt noise_info;
}  ;
/*** val[0] end***/

/* TLV: SKW_MIB_GET_PEER_INFO_E */
/* No val data for debug info cmd */
/** Rsp of this TLV **/
struct skw_peer_info_rsp
{
    u16 status;
    u16 tlv;
    u16 len;
    u8 val[0];
}  ;

/*** val[0] start***/

struct skw_tx_hmac_per_peer_rpt_info
{
    u32 hmac_tx_stat[5];
    u32 txc_isr_read_dscr_fifo_cnt;
}  ;

struct skw_rate_struct
{
    u8 flags;
    u8 mcs_idx;
    u16 legacy_rate;
    u8 nss;
    u8 bw;
    u8 gi;
    u8 ru;
    u8 dcm;
}  ;

struct skw_rx_msdu_info_rpt
{
    u8 ppdu_mode;
    u8 data_rate;
    u8 gi_type;
    u8 sbw;
    u8 dcm;
    u8 nss;
}  ;

struct skw_tx_dbg_stats_per_peer
{
    u32 rts_fail_cnt;
    u32 psdu_ack_timeout_cnt;
    u32 tx_mpdu_cnt;
    u32 tx_wait_time;
}  ;

/*val*/
struct skw_get_peer_info
{
    u8 is_valid;
    u8 inst_id;
    u8 hw_id;
    int8_t rx_rsp_rssi;
    u16 avg_add_delay_in_ms;
    u32 tx_max_delay_time;
    struct skw_tx_hmac_per_peer_rpt_info hmac_tx_stat;
    struct skw_rate_struct tx_rate_info;
    struct skw_rx_msdu_info_rpt rx_msdu_info_rpt;
    struct skw_tx_dbg_stats_per_peer tx_stats_info;
    u16 tx_pending_pkt_cnt[4];
}  ;

/*** val[0] end***/


/* TLV: SKW_MIB_GET_HW_RX_INFO_E */
/* No val data for debug info cmd */
/** Rsp of this TLV **/
struct skw_hw_rx_info_rsp
{
    u16 status;
    u16 tlv;
    u16 len;
    u8 val[0];
}  ;

/*** val[0] start***/

/*val*/
struct skw_get_hw_rx_info
{
    u16 phy_rx_all_cnt;
    u16 phy_rx_11b_cnt;
    u16 phy_rx_11g_cnt;
    u16 phy_rx_11n_cnt;
    u16 phy_rx_11ac_cnt;
    u16 phy_rx_11ax_cnt;
    u16 mac_rx_mpdu_cnt;
    u16 mac_rx_mpdu_fcs_pass_cnt;
    u16 mac_rx_mpdu_fcs_pass_dir_cnt;
}  ;

/*** val[0] end***/

/* TLV: SKW_MIB_GET_INST_TSF_E */
/* No val data for debug info cmd */
/** Rsp of this TLV **/
struct skw_inst_tsf
{
    u32 tsf_l;
    u32 tsf_h;
}  ;

/*** val[0] start***/

/*val*/
struct skw_get_inst_tsf_rsp
{
    u16 status;
    u16 tlv;
    u16 len;
    u8 val[0];
}  ;

static inline void skw_abort_cmd(struct skw_core *skw)
{
#if 0
    if (skw_cmd_locked(&skw->cmd.lock) &&
            !test_and_set_bit(SKW_CMD_FLAG_ACKED, &skw->cmd.flags))
        skw->cmd.callback(skw);
#endif
}
#if 0

int skw_msg_xmit_timeout(struct wiphy *wiphy, int dev_id, int cmd,
                         void *buf, int len, void *arg, int size,
                         char *name, unsigned long timeout,
                         unsigned long extra_flags);

#define skw_msg_xmit(wiphy, inst, cmd, buf, len, arg, size)                     \
    skw_msg_xmit_timeout(wiphy, inst, cmd, buf, len,                        \
            arg, size, #cmd,                                        \
            msecs_to_jiffies(SKW_CMD_TIMEOUT), 0)

//#define SKW_NDEV_ID(d) (((struct skw_iface *)netdev_priv(d))->id)

#define skw_send_msg(wiphy, dev, cmd, buf, len, arg, size)                      \
    skw_msg_xmit_timeout(wiphy, dev ? SKW_NDEV_ID(dev) : -1,                \
            cmd, buf, len, arg, size, #cmd,                         \
            msecs_to_jiffies(SKW_CMD_TIMEOUT), 0)

#endif
int skw_msg_try_send(struct skw_core *skw, int dev_id,
                     int cmd, void *data, int data_len,
                     void *arg, int arg_size, char *name);

//void skw_default_event_work(struct work_struct *work);
int skw_cmd_ack_handler(struct skw_core *skw, void *data, int data_len);
void skw_event_handler(struct skw_core *skw, struct skw_iface *iface,
                       struct skw_msg *msg_hdr, void *data, size_t data_len);
//int skw_queue_local_event(struct wiphy *wiphy, struct skw_iface *iface,
//            int event_id, void *data, size_t data_len);
void skw_del_sta_event(struct skw_iface *iface, const u8 *addr, u16 reason);

void skw_cqm_scan_timeout(void *data);
#endif
