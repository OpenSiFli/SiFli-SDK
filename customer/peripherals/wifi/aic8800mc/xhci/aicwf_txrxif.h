/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _AICWF_TXRXIF_H_
#define _AICWF_TXRXIF_H_

#include "lmac_types.h"
#include "lmac_msg.h"
#include "osal_service.h"
#include "aicwf_types.h"
#include "aicwf_config.h"
//#include "aicwf_sdio.h"
#if 0
    #include "aicwf_usb.h"
#endif
#include "co_list.h"

#define CMD_BUF_MAX                 1536
#define TXPKT_BLOCKSIZE             512
#define MAX_AGGR_TXPKT_LEN          (1536*8)//(1536*64)
#define CMD_TX_TIMEOUT              5000
#define TX_ALIGNMENT                4

#define RX_HWHRD_LEN                60 //58->60 word allined
#define CCMP_OR_WEP_INFO            8
#define MAX_RXQLEN                  2000
#define RX_ALIGNMENT                4


enum aicwf_bus_state
{
    BUS_DOWN_ST,
    BUS_UP_ST
};

struct aicwf_bus_ops
{
    int (*start)(void *bus_data);
    void (*stop)(void *bus_data);
#ifdef CONFIG_DATA_TXRX
    int (*txdata)(void *bus_data, u8 *pkt, uint len);
#endif
    int (*txmsg)(void *bus_data, u8 *msg, uint len);
    int (*rxtask)(void *bus_data);
    int (*txtask)(void *bus_data);
};

struct frame_queue
{
    u16              num_prio;
    u16              hi_prio;
    u16              qmax;      /* max number of queued frames */
    u16              qcnt;
    //struct sk_buff_head queuelist[8];
};

struct aicwf_buf_node_s
{
    struct co_list_hdr hdr;
    uint8_t *buf_raw; // static alloced
    uint8_t *buf; // 64B aligned, rx in-use
    uint16_t buf_len;
    uint16_t pad_len;
};

struct aicwf_buf_list_s
{
    struct co_list list;
    rtos_mutex mutex;
    rtos_semaphore ava_node_sema;
};

struct aic_sdio_dev;

struct aicwf_bus
{
    union
    {
        void *xhci;
        struct aic_sdio_dev *sdio;
#if 0
        struct aic_usb_dev *usb;
#endif
    } bus_priv;
    //struct device *dev;
    struct aicwf_bus_ops *ops;
    enum aicwf_bus_state state;
    u8 *cmd_buf;
    //struct completion bustx_trgg;
    //struct completion busrx_trgg;
    //struct task_struct *bustx_thread;
    //struct task_struct *busrx_thread;
    rtos_semaphore bustx_trgg;
    rtos_semaphore busrx_trgg;
    rtos_task_handle bustx_thread;
    rtos_task_handle busrx_thread;
};

struct aicwf_tx_priv
{
#ifdef AICWF_SDIO_SUPPORT
    struct aic_sdio_dev *sdiodev;
    wait_queue_head_t cmd_txdone_wait;

    //struct frame_queue txq;
    spinlock_t txqlock;
    struct semaphore txctl_sema;
#endif
#ifdef AICWF_USB_SUPPORT
    struct aic_usb_dev *usbdev;
#endif
    //for cmd tx
    u8 *cmd_buf;
    uint cmd_len;
    bool cmd_txstate;
    bool cmd_tx_succ;
    rtos_mutex cmd_txsema;
#ifdef CONFIG_DATA_TXRX
    //for data tx
    struct aicwf_buf_list_s txq;
    int fw_avail_bufcnt;
    rtos_atomic_t tx_pktcnt;
    u8 *aggr_buf;
    //u8 *aggr_buf_align;
    u32 aggr_count;
    u32 len;
    u8 *head;
    u8 *tail;
#endif
};


#ifdef AICWF_RX_REORDER
#define MAX_REORD_RXFRAME       250
#define REORDER_UPDATE_TIME     50
#define AICWF_REORDER_WINSIZE   64
#define SN_LESS(a, b)           (((a-b)&0x800)!=0)
#define SN_EQUAL(a, b)          (a == b)

struct reord_ctrl
{
    struct aicwf_rx_priv *rx_priv;
    u8 enable;
    u16 ind_sn;
    u8 wsize_b;
    spinlock_t reord_list_lock;
    struct list_head reord_list;
    struct timer_list reord_timer;
    struct work_struct reord_timer_work;
};

struct reord_ctrl_info
{
    u8 mac_addr[6];
    struct reord_ctrl preorder_ctrl[8];
    struct list_head list;
};

struct recv_msdu
{
    //struct sk_buff  *pkt;
    u8  tid;
    u16 seq_num;
    uint len;
    u8 *rx_data;
    //for pending rx reorder list
    struct list_head reord_pending_list;
    //for total frame list, when rxframe from busif, dequeue, when submit frame to net, enqueue
    struct list_head rxframe_list;
    struct reord_ctrl *preorder_ctrl;
};
#endif

struct aicwf_rx_priv
{
#if 0
#ifdef AICWF_SDIO_SUPPORT
    struct aic_sdio_dev *sdiodev;
#endif
#ifdef AICWF_USB_SUPPORT
    struct aic_usb_dev *usbdev;
#endif
#endif
    void *rwnx_vif;
    //atomic_t rx_cnt;
    u32 data_len;
    //spinlock_t rxqlock;
    //struct frame_queue rxq;
    struct aicwf_buf_list_s rxq;

#ifdef AICWF_RX_REORDER
    spinlock_t freeq_lock;
    struct list_head rxframes_freequeue;
    struct list_head stas_reord_list;
    spinlock_t stas_reord_lock;
    struct recv_msdu *recv_frames;
#endif
};

typedef struct
{
    u16_l  plen;
    u8_l   ptype;
    u8_l   reserved;
    u32_l  payload[1];
} host_data_t;

#define HOST_DATA_HDR_LEN       offsetof_b(host_data_t, payload)

static inline int aicwf_bus_start(struct aicwf_bus *bus)
{
    return bus->ops->start(bus);
}

static inline void aicwf_bus_stop(struct aicwf_bus *bus)
{
    bus->ops->stop(bus);
}

#ifdef CONFIG_DATA_TXRX
static inline int aicwf_bus_txdata(struct aicwf_bus *bus, u8 *pkt, uint len)
{
    return bus->ops->txdata(bus, pkt, len);
}
#endif

static inline int aicwf_bus_txmsg(struct aicwf_bus *bus, u8 *msg, uint len)
{
    return bus->ops->txmsg(bus, msg, len);
}

#if 0
static inline void aicwf_sched_timeout(u32 millisec)
{
    ulong timeout = 0, expires = 0;
    expires = jiffies + msecs_to_jiffies(millisec);
    timeout = millisec;

    while (timeout)
    {
        timeout = schedule_timeout(timeout);
        if (time_after(jiffies, expires))
            break;
    }
}
#endif

void aicwf_busrx_buf_init(void);
struct aicwf_buf_node_s *aicwf_busrx_buf_alloc(uint16_t buf_len);
void aicwf_busrx_buf_free(struct aicwf_buf_node_s *node);
#ifdef CONFIG_DATA_TXRX
    void aicwf_bustx_buf_init(void);
    struct aicwf_buf_node_s *aicwf_bustx_buf_alloc(uint16_t buf_len);
    void aicwf_bustx_buf_free(struct aicwf_buf_node_s *node);
#endif

int aicwf_bus_init(struct aicwf_bus *bus_if);
void aicwf_bus_deinit(struct aicwf_bus *bus_if);
void aicwf_tx_deinit(struct aicwf_tx_priv *tx_priv);
void aicwf_rx_deinit(struct aicwf_rx_priv *rx_priv);
struct aicwf_tx_priv *aicwf_tx_init(void *arg);
struct aicwf_rx_priv *aicwf_rx_init(void *arg);
void aicwf_frame_queue_init(struct aicwf_buf_list_s *buf_list);
void aicwf_frame_queue_deinit(struct aicwf_buf_list_s *buf_list);
void aicwf_frame_enqueue(struct aicwf_buf_list_s *buf_list, struct aicwf_buf_node_s *node);
struct aicwf_buf_node_s *aicwf_frame_dequeue(struct aicwf_buf_list_s *buf_list);
void aicwf_frame_queue_flush(struct aicwf_buf_list_s *buf_list);
bool aicwf_is_framequeue_empty(struct aicwf_buf_list_s *buf_list);
#ifdef CONFIG_DATA_TXRX
    int aicwf_frame_tx(struct aicwf_bus *bus_if, uint8_t *pkt, uint32_t len);
#endif
void aicwf_msg_tx(void *dev,  u8 *msg, uint len);
int aicwf_bus_msg_tx(struct aicwf_bus *bus_if, struct lmac_msg *msg, uint16_t len);

#endif /* _AICWF_TXRXIF_H_ */
