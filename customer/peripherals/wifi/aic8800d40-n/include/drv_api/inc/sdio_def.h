/*
 * Copyright (C) 2018-2020 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SDIO_DEF_H_
#define _SDIO_DEF_H_

#include "sdio_port.h"
#include "aic_porting_types.h"

/* -------------- h/w register ------------------- */
#define SDIOWIFI_FUNC_BLOCKSIZE             512

#define SDIOWIFI_BYTEMODE_LEN_REG           0x02
#define SDIOWIFI_INTR_CONFIG_REG            0x04
#define SDIOWIFI_SLEEP_REG                  0x05
#define SDIOWIFI_WAKEUP_REG                 0x09
#define SDIOWIFI_FLOW_CTRL_REG              0x0A
#define SDIOWIFI_REGISTER_BLOCK             0x0B
#define SDIOWIFI_BYTEMODE_ENABLE_REG        0x11
#define SDIOWIFI_BLOCK_CNT_REG              0x12
#define SDIOWIFI_FLOWCTRL_MASK_REG          0x7F
#define SDIOWIFI_WR_FIFO_ADDR               0x07
#define SDIOWIFI_RD_FIFO_ADDR               0x08
#define SDIOWIFI_FBR_FUNC1_BLK_SIZE_REG     0x110
#define SDIOWIFI_FBR_FUNC2_BLK_SIZE_REG     0x210

#define SDIOWIFI_INTR_ENABLE_REG_V3         0x00
#define SDIOWIFI_INTR_PENDING_REG_V3        0x01
#define SDIOWIFI_INTR_TO_DEVICE_REG_V3      0x02
#define SDIOWIFI_FLOW_CTRL_Q1_REG_V3        0x03
#define SDIOWIFI_MISC_INT_STATUS_REG_V3     0x04
#define SDIOWIFI_BYTEMODE_LEN_REG_V3        0x05
#define SDIOWIFI_BYTEMODE_LEN_MSB_REG_V3    0x06
#define SDIOWIFI_BYTEMODE_ENABLE_REG_V3     0x07
#define SDIOWIFI_MISC_CTRL_REG_V3           0x08
#define SDIOWIFI_FLOW_CTRL_Q2_REG_V3        0x09
#define SDIOWIFI_CLK_TEST_RESULT_REG_V3     0x0A
#define SDIOWIFI_RD_FIFO_ADDR_V3            0x0F
#define SDIOWIFI_WR_FIFO_ADDR_V3            0x10

#define SDIOCLK_FREE_RUNNING_BIT            (1 << 6)
#define FEATURE_SDIO_CLOCK_V3               150000000

#define SDIOWIFI_PWR_CTRL_INTERVAL          30
#define FLOW_CTRL_RETRY_COUNT               50
#define BUFFER_SIZE                         1536
#define TAIL_LEN                            32//4
#define TXQLEN                              (2048*4)

#define TXPKT_BLOCKSIZE                     512
#ifndef CONFIG_SDIO_ADMA
    #define MAX_AGGR_TXPKT_CNT                  28
#else
    #define MAX_AGGR_TXPKT_CNT                  64
#endif /* CONFIG_SDIO_ADMA */
#define TX_ALIGNMENT                        4
#define RX_ALIGNMENT                        4
#define AGGR_TXPKT_ALIGN_SIZE               64

#define DATA_FLOW_CTRL_THRESH               2
#define MAX_AGGR_TXPKT_LEN                  ((1500 + 44 + 4) * MAX_AGGR_TXPKT_CNT)
#define TX_AGGR_COUNTER                     (MAX_AGGR_TXPKT_CNT + DATA_FLOW_CTRL_THRESH)

#define SDIO_OTHER_INTERRUPT                (0x1ul << 7)


/* SDIO Device ID */
#define SDIO_VENDOR_ID_AIC8801              0x5449
#define SDIO_VENDOR_ID_AIC8800DC            0xc8a1
#define SDIO_VENDOR_ID_AIC8800D80           0xc8a1
#define SDIO_VENDOR_ID_AIC8800D80N          0xc8a1
#define SDIO_VENDOR_ID_AIC8800D80X2         0xc8a1

#define SDIO_DEVICE_ID_AIC8801              0x0145
#define SDIO_DEVICE_ID_AIC8800DC            0xc08d
#define SDIO_DEVICE_ID_AIC8800D80           0x0082
#define SDIO_DEVICE_ID_AIC8800D80N          0x9081
#define SDIO_DEVICE_ID_AIC8800D80LN         0x9082
#define SDIO_DEVICE_ID_AIC8800D80WN         0x9083
#define SDIO_DEVICE_ID_AIC8800D40N          0x9084
#define SDIO_DEVICE_ID_AIC8800D40LN         0x9085
#define SDIO_DEVICE_ID_AIC8800D40WN         0x9086
#define SDIO_DEVICE_ID_AIC8800D80X2         0x2082


//-------------------------------------------------------------------
// Driver Typedef Variables
//-------------------------------------------------------------------
typedef void (*sdio_adma_free)(void *pbuf);


//-------------------------------------------------------------------
// Driver Struct Define
//-------------------------------------------------------------------
struct aic_sdio_reg
{
    uint8_t bytemode_len_reg;
    uint8_t intr_config_reg;
    uint8_t sleep_reg;
    uint8_t wakeup_reg;
    uint8_t flow_ctrl_reg;
    uint8_t flowctrl_mask_reg;
    uint8_t register_block;
    uint8_t bytemode_enable_reg;
    uint8_t block_cnt_reg;
    uint8_t misc_int_status_reg;
    uint8_t rd_fifo_addr;
    uint8_t wr_fifo_addr;
    uint32_t fun1_blk_size_reg;
    uint32_t fun2_blk_size_reg;
};

struct sdio_pwrctl_s
{
    uint32_t bus_status;
    uint32_t init_status;
    uint32_t active_duration;
    rtos_mutex bus_wakeup_sema;
    rtos_mutex pwrctl_lock;
    rtos_semaphore pwrctl_trgg;
    rtos_timer pwrctl_timer;
    rtos_timer_status timer_status;
    rtos_task_handle pwrctl_task_hdl;
};

struct aic_sdio_dev
{
    /* sdio base */
    uint16_t chipid;
    struct aic_sdio_reg sdio_reg;

    /* sdio buffer config (set by platform before aicwf_sdio_host_init;
     * 0 means "use default", apply_default is called in tx_init/buf_init).
     * rx buffer size is fixed at compile time (SDIO_RX_BUF_SIZE) */
    uint32_t rx_buf_count;
    uint32_t max_aggr_txpkt_cnt;

    /* sdio buffer */
    uint8_t *aggr_buf;
    uint8_t *aggr_buf_align;
    uint32_t aggr_count;
    uint32_t aggr_end;
    uint8_t *head;
    uint8_t *tail;
    uint32_t len;
    int32_t fw_avail_bufcnt;
    rtos_mutex bus_txrx;

    /* sdio task */
    rtos_semaphore sdio_rx_sema;
    rtos_task_handle sdio_task_hdl;

    /* sdio flowctrl */
    uint32_t fc_th;
    uint32_t stamp;
    int tx_aggr_counter;

#ifdef CONFIG_OOB
    uint32_t oob_enable;
    uint32_t sdio_gpio_num;
    rtos_semaphore sdio_oobirq_sema;
    rtos_task_handle sdio_oobirq_hdl;
#endif

#ifdef CONFIG_SDIO_BUS_PWRCTRL
    uint32_t rx_cnt;
    uint32_t tx_pktcnt;
    uint32_t cmd_txstate;
    rtos_mutex tx_sema;
    rtos_mutex rx_sema;
    rtos_mutex cmdtx_sema;
    struct sdio_pwrctl_s pwrctl;
#endif

#ifdef CONFIG_SDIO_ADMA
    sdio_sg_list sg_list[SDIO_TX_SLIST_MAX];
    void *desc_buf[SDIO_TX_SLIST_MAX];
    uint8_t desc_free[SDIO_TX_SLIST_MAX];
    sdio_adma_free sfree;
    uint32_t aggr_segcnt;
    uint8_t fill;
#endif

    uint32_t wr_fail;
};

struct sdio_buf_node_s
{
    struct co_list_hdr hdr;
    uint8_t *buf_raw;  // static alloced
    uint8_t *buf;      // 64B aligned, rx in-use
    uint16_t buf_len;
    uint16_t pad_len;
#ifdef CONFIG_RX_NOCOPY
    /* [PLATFORM] RX 零拷贝: 聚合 node 含多个子帧，数据帧经 custom pbuf 持有
     * 额外引用，ref 归零才回池。alloc 置 1(基础引用)，每建一个 custom pbuf +1
     */
    uint8_t ref;
#endif /* CONFIG_RX_NOCOPY */
};

struct sdio_buf_list_s
{
    struct co_list list;
    rtos_mutex mutex;
    rtos_semaphore sdio_rx_node_sema;
    uint16_t free_node_cnt;
    uint16_t alloc_fail_cnt;
    uint16_t allocwait_fail_cnt;
    uint16_t unexcept_evt;
    uint8_t *tmpbuf;
};

#endif /* _SDIO_DEF_H_ */
