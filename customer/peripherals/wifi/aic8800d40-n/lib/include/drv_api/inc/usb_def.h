/*
 * Copyright (C) 2018-2025 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB driver definitions (Plan B stub).
 * Mirrors sdio_def.h structures for compile-time bus abstraction.
 * USB transport has no CMD52/CMD53; registers are accessed via
 * vendor control transfers (stubbed here).
 */

#ifndef _USB_DEF_H_
#define _USB_DEF_H_

#include "usb_port.h"
#include "aic_porting_types.h"

/* -------------- USB endpoint / transfer constants ------------------- */
#define USBWIFI_FUNC_BLOCKSIZE             512

#define USBWIFI_BYTEMODE_LEN_REG           0x02
#define USBWIFI_INTR_CONFIG_REG            0x04
#define USBWIFI_SLEEP_REG                  0x05
#define USBWIFI_WAKEUP_REG                 0x09
#define USBWIFI_FLOW_CTRL_REG              0x0A
#define USBWIFI_REGISTER_BLOCK             0x0B
#define USBWIFI_BYTEMODE_ENABLE_REG        0x11
#define USBWIFI_BLOCK_CNT_REG              0x12
#define USBWIFI_FLOWCTRL_MASK_REG          0x7F
#define USBWIFI_WR_FIFO_ADDR               0x07
#define USBWIFI_RD_FIFO_ADDR               0x08

#define USBWIFI_INTR_ENABLE_REG_V3         0x00
#define USBWIFI_INTR_PENDING_REG_V3        0x01
#define USBWIFI_INTR_TO_DEVICE_REG_V3      0x02
#define USBWIFI_FLOW_CTRL_Q1_REG_V3        0x03
#define USBWIFI_MISC_INT_STATUS_REG_V3     0x04
#define USBWIFI_BYTEMODE_LEN_REG_V3        0x05
#define USBWIFI_BYTEMODE_LEN_MSB_REG_V3    0x06
#define USBWIFI_BYTEMODE_ENABLE_REG_V3     0x07
#define USBWIFI_MISC_CTRL_REG_V3           0x08
#define USBWIFI_FLOW_CTRL_Q2_REG_V3        0x09
#define USBWIFI_CLK_TEST_RESULT_REG_V3     0x0A
#define USBWIFI_RD_FIFO_ADDR_V3            0x0F
#define USBWIFI_WR_FIFO_ADDR_V3            0x10

#define USBCLK_FREE_RUNNING_BIT            (1 << 6)
#define FEATURE_USB_CLOCK_V3               150000000

#define USBWIFI_PWR_CTRL_INTERVAL          30
#define USB_FLOW_CTRL_RETRY_COUNT          50
#define USB_BUFFER_SIZE                    1536
#define USB_TAIL_LEN                       32
#define USB_TXQLEN                         (2048*4)

#define USB_TXPKT_BLOCKSIZE                512
#ifndef CONFIG_USB_ADMA
    #define USB_MAX_AGGR_TXPKT_CNT             28
#else
    #define USB_MAX_AGGR_TXPKT_CNT             64
#endif /* CONFIG_USB_ADMA */
#define USB_TX_ALIGNMENT                   4
#define USB_RX_ALIGNMENT                   4
#define USB_AGGR_TXPKT_ALIGN_SIZE          64

#define USB_DATA_FLOW_CTRL_THRESH          2
#define USB_MAX_AGGR_TXPKT_LEN             ((1500 + 44 + 4) * USB_MAX_AGGR_TXPKT_CNT)
#define USB_TX_AGGR_COUNTER                (USB_MAX_AGGR_TXPKT_CNT + USB_DATA_FLOW_CTRL_THRESH)

/* ---- Aliases for SDIO-named constants used by upper layers ---- *
 * fhost_rx.c / fhost_tx.c reference RX_ALIGNMENT / TX_ALIGNMENT /
 * DATA_FLOW_CTRL_THRESH directly. Provide aliases so the same upper
 * layer code compiles under both bus backends without #ifdef churn.
 */
#define RX_ALIGNMENT                       USB_RX_ALIGNMENT
#define TX_ALIGNMENT                       USB_TX_ALIGNMENT
#define DATA_FLOW_CTRL_THRESH              USB_DATA_FLOW_CTRL_THRESH
#define MAX_AGGR_TXPKT_CNT                 USB_MAX_AGGR_TXPKT_CNT
#define MAX_AGGR_TXPKT_LEN                 USB_MAX_AGGR_TXPKT_LEN
#define AGGR_TXPKT_ALIGN_SIZE              USB_AGGR_TXPKT_ALIGN_SIZE

#define USB_OTHER_INTERRUPT                (0x1ul << 7)

/*
 * USB vendor/product IDs are defined in aic_usb_types.h (included
 * via usb_port.h). Do not redefine here — see that file for the
 * canonical values sourced from the Linux reference driver.
 */

/* -------------------------------------------------------------------- */
typedef void (*usb_adma_free)(void *pbuf);

/* USB register map (mirrors aic_sdio_reg) */
struct aic_usb_reg
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

struct usb_pwrctl_s
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

/* USB device struct (mirrors aic_sdio_dev) */
struct aic_usb_dev
{
    /* usb base */
    uint16_t chipid;
    struct aic_usb_reg usb_reg;

    /* usb buffer */
    uint8_t *aggr_buf;
    uint8_t *aggr_buf_align;
    uint32_t aggr_count;
    uint32_t aggr_end;
    uint8_t *head;
    uint8_t *tail;
    uint32_t len;
    int32_t fw_avail_bufcnt;
    rtos_mutex bus_txrx;

    /* usb task */
    rtos_semaphore usb_rx_sema;
    rtos_task_handle usb_task_hdl;

    /* usb flowctrl */
    uint32_t fc_th;
    uint32_t stamp;
    int tx_aggr_counter;

#ifdef CONFIG_OOB
    uint32_t oob_enable;
    uint32_t usb_gpio_num;
    rtos_semaphore usb_oobirq_sema;
    rtos_task_handle usb_oobirq_hdl;
#endif

#ifdef CONFIG_USB_BUS_PWRCTRL
    uint32_t rx_cnt;
    uint32_t tx_pktcnt;
    uint32_t cmd_txstate;
    rtos_mutex tx_sema;
    rtos_mutex rx_sema;
    rtos_mutex cmdtx_sema;
    struct usb_pwrctl_s pwrctl;
#endif

#ifdef CONFIG_USB_ADMA
    usb_sg_list sg_list[USB_TX_SLIST_MAX];
    void *desc_buf[USB_TX_SLIST_MAX];
    uint8_t desc_free[USB_TX_SLIST_MAX];
    usb_adma_free sfree;
    uint32_t aggr_segcnt;
    uint8_t fill;
#endif

    uint32_t wr_fail;
};

/* USB buffer node (mirrors sdio_buf_node_s) */
struct usb_buf_node_s
{
    struct co_list_hdr hdr;
    uint8_t *buf_raw;  // static alloced
    uint8_t *buf;      // 64B aligned, rx in-use
    uint16_t buf_len;
    uint16_t pad_len;
#ifdef CONFIG_RX_NOCOPY
    uint8_t ref;  /* [PLATFORM] RX 零拷贝对称实现, 与 sdio_buf_node_s.ref 同义
                   */
#endif /* CONFIG_RX_NOCOPY */
};

struct usb_buf_list_s
{
    struct co_list list;
    rtos_mutex mutex;
    rtos_semaphore usb_rx_node_sema;
    uint16_t free_node_cnt;
    uint16_t alloc_fail_cnt;
    uint16_t allocwait_fail_cnt;
    uint16_t unexcept_evt;
    uint8_t *tmpbuf;
};

#endif /* _USB_DEF_H_ */
