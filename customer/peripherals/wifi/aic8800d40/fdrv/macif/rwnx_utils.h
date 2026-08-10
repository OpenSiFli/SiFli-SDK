/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * rwnx_utils.h
 */
#ifndef _RWNX_UTILS_H_
#define _RWNX_UTILS_H_

#include "aic_plat_types.h"
#include "aic_plat_log.h"
#include "fhost.h"

/// Macro defining an invalid VIF index
#define INVALID_VIF_IDX 0xFF
#define INVALID_STA_IDX 0xFF

#define AICWF_RX_REORDER    1
#define AICWF_RWNX_TIMER_EN 1
#define REORDER_DEBUG       0

#if !REORDER_DEBUG
#define DBG_REORD(...)  do {} while (0)
#define WRN_REORD(...)  do {} while (0)
#define DBG_TIMER(...)  do {} while (0)
#define WRN_TIMER(...)  do {} while (0)
#else
#define DBG_REORD(...)  DBG_MACIF_VRB(__VA_ARGS__)
#define WRN_REORD(...)  DBG_MACIF_WRN(__VA_ARGS__)
#define DBG_TIMER(...)  DBG_MACIF_VRB(__VA_ARGS__)
#define WRN_TIMER(...)  DBG_MACIF_WRN(__VA_ARGS__)
#endif
#define ERR_REORD(...)  DBG_MACIF_ERR(__VA_ARGS__)
#define ERR_TIMER(...)  DBG_MACIF_ERR(__VA_ARGS__)

#ifdef CONFIG_RWNX_DBG
#define RWNX_DBG        AIC_LOG_PRINTF
#else
#define RWNX_DBG(a...)  do {} while (0)
#endif

#define RWNX_FN_ENTRY_STR ">>> %s()\n", __func__

struct rwnx_hw;
typedef uint32_t dma_addr_t;

/// 802.11 Status Code
#define MAC_ST_SUCCESSFUL                   0
#define MAC_ST_FAILURE                      1
#define MAC_ST_RESERVED                     2
#define MAC_ST_CAPA_NOT_SUPPORTED           10
#define MAC_ST_REASSOC_NOT_ASSOC            11
#define MAC_ST_ASSOC_DENIED                 12
#define MAC_ST_AUTH_ALGO_NOT_SUPPORTED      13
#define MAC_ST_AUTH_FRAME_WRONG_SEQ         14
#define MAC_ST_AUTH_CHALLENGE_FAILED        15
#define MAC_ST_AUTH_TIMEOUT                 16
#define MAC_ST_ASSOC_TOO_MANY_STA           17
#define MAC_ST_ASSOC_RATES_NOT_SUPPORTED    18
#define MAC_ST_ASSOC_PREAMBLE_NOT_SUPPORTED 19

#define MAC_ST_ASSOC_SPECTRUM_REQUIRED      22
#define MAC_ST_ASSOC_POWER_CAPA             23
#define MAC_ST_ASSOC_SUPPORTED_CHANNEL      24
#define MAC_ST_ASSOC_SLOT_NOT_SUPPORTED     25

#define MAC_ST_REFUSED_TEMPORARILY          30
#define MAC_ST_INVALID_MFP_POLICY           31

#define MAC_ST_INVALID_IE                   40             // draft 7.0 extention
#define MAC_ST_GROUP_CIPHER_INVALID         41             // draft 7.0 extention
#define MAC_ST_PAIRWISE_CIPHER_INVALID      42             // draft 7.0 extention
#define MAC_ST_AKMP_INVALID                 43             // draft 7.0 extention
#define MAC_ST_UNSUPPORTED_RSNE_VERSION     44             // draft 7.0 extention
#define MAC_ST_INVALID_RSNE_CAPA            45             // draft 7.0 extention
#define MAC_ST_CIPHER_SUITE_REJECTED        46             // draft 7.0 extention


#if (AICWF_RWNX_TIMER_EN)
typedef void (*rwnx_timer_cb_t)(void * arg,void * arg1);

enum rwnx_timer_state_e {
    RWNX_TIMER_STATE_FREE   = 0,
    RWNX_TIMER_STATE_POST   = 1,
    RWNX_TIMER_STATE_STOP   = 2,
};

enum rwnx_timer_action_e {
    RWNX_TIMER_ACTION_CREATE    = 0,
    RWNX_TIMER_ACTION_START     = 1,
    RWNX_TIMER_ACTION_RESTART   = 2,
    RWNX_TIMER_ACTION_STOP      = 3,
    RWNX_TIMER_ACTION_DELETE    = 4,
    RWNX_TIMER_ACTION_NONE      = 5,
};

struct rwnx_timer_node_s {
    struct co_list_hdr hdr;
    rwnx_timer_cb_t cb;
    void *arg;
	void *arg1;
    uint32_t expired_ms; // remained
    enum rwnx_timer_state_e state;
    enum rwnx_timer_action_e action;
    bool periodic;
    bool auto_load;
};

typedef struct rwnx_timer_node_s * rwnx_timer_handle;
#endif

#if (AICWF_RX_REORDER)
#define MAX_REORD_RXFRAME       250
#define REORDER_UPDATE_TIME     50
#define AICWF_REORDER_WINSIZE   64
#define SN_LESS(a, b)           (((a-b)&0x800) != 0)
#define SN_EQUAL(a, b)          (a == b)

struct reord_ctrl {
    uint8_t enable;
    uint8_t wsize_b;
    uint16_t ind_sn;
    uint16_t list_cnt;
    struct co_list reord_list;
    rtos_mutex reord_list_lock;
    #if (AICWF_RWNX_TIMER_EN)
    rwnx_timer_handle reord_timer;
    #else
    rtos_timer reord_timer;
    #endif
};

struct reord_ctrl_info {
    struct co_list_hdr hdr;
    uint8_t mac_addr[6];
    struct reord_ctrl preorder_ctrl[8];
};

struct fhost_rx_buf_tag;

struct recv_msdu {
    struct co_list_hdr hdr;
    struct fhost_rx_buf_tag *rx_buf;
    #ifdef CONFIG_RX_NOCOPY
    void *lwip_msg;
    #endif
    uint16_t buf_len;
    uint16_t seq_num;
    uint8_t  tid;
    uint8_t  forward;
    //struct reord_ctrl *preorder_ctrl;
};

int rwnx_rxdataind_aicwf(struct fhost_rx_buf_tag *buf);
void rwnx_reord_init(void);
void rwnx_reord_deinit(void);
int reord_single_frame_ind(struct recv_msdu *prframe);
void reord_deinit_sta_by_mac(const uint8_t *mac_addr);
#endif

int8_t data_pkt_rssi_get(uint8_t *mac_addr);
#endif /* _RWNX_UTILS_H_ */
