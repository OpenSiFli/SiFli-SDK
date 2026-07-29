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

#ifndef _AICWF_SDIO_H_
#define _AICWF_SDIO_H_

#include "sdio_func.h"
#include "aicwf_txrxif.h"

#if 1//def AICWF_SDIO_SUPPORT

#define AICWF_SDIO_NAME                     "aicwf_sdio"
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

#define SDIOCLK_FREE_RUNNING_BIT        (1 << 6)

#define SDIOWIFI_CLOCK_V2               30000000 // default 20MHz
#define SDIOWIFI_CLOCK_V3               20000000 // default 20MHz
#define SDIOWIFI_PHASE_V2               0        // 0: default, 2: 180°

#define SDIOWIFI_PWR_CTRL_INTERVAL      30
#define FLOW_CTRL_RETRY_COUNT           50
#define BUFFER_SIZE                     1536
#define TAIL_LEN                        4
#define TXQLEN                          (2048*4)

#define SDIO_SLEEP_ST                   0
#define SDIO_ACTIVE_ST                  1

#define DATA_FLOW_CTRL_THRESH 2
#ifdef CONFIG_TX_NETIF_FLOWCTRL
    #define AICWF_SDIO_TX_LOW_WATER         100
    #define AICWF_SDIO_TX_HIGH_WATER        500
#endif

#define CFG_SDIO_BUF_IN_DCACHE          0

typedef enum
{
    SDIO_TYPE_DATA          = 0X00,
    SDIO_TYPE_DATA_AHB2SDIO = 0X01,
    SDIO_TYPE_CFG           = 0X10,
    SDIO_TYPE_CFG_CMD_RSP   = 0X11,
    SDIO_TYPE_CFG_DATA_CFM  = 0X12
} sdio_type;

/* SDIO Device ID */
#define SDIO_VENDOR_ID_AIC8800          0x5449
#define SDIO_VENDOR_ID_AIC8800MC        0xc8a1
#define SDIO_VENDOR_ID_AIC8800M40       0xc8a1

#define SDIO_DEVICE_ID_AIC8800          0x0145
#define SDIO_DEVICE_ID_AIC8800MC        0xc08d
#define SDIO_DEVICE_ID_AIC8800M40       0x0082

struct rwnx_hw;

struct aic_sdio_reg
{
    u8 bytemode_len_reg;
    u8 intr_config_reg;
    u8 sleep_reg;
    u8 wakeup_reg;
    u8 flow_ctrl_reg;
    u8 flowctrl_mask_reg;
    u8 register_block;
    u8 bytemode_enable_reg;
    u8 block_cnt_reg;
    u8 misc_int_status_reg;
    u8 rd_fifo_addr;
    u8 wr_fifo_addr;
};

struct aic_sdio_dev
{
    struct rwnx_hw *rwnx_hw;
    struct sdio_func *func;
    //struct device *dev;
    struct aicwf_bus *bus_if;

    struct aicwf_rx_priv *rx_priv;
    struct aicwf_tx_priv *tx_priv;
    u32 state;
#ifdef CONFIG_TX_NETIF_FLOWCTRL
    u8 flowctrl;
    spinlock_t tx_flow_lock;
#endif

#if defined(CONFIG_SDIO_PWRCTRL)
    //for sdio pwr ctrl
    struct timer_list timer;
    uint active_duration;
    struct completion pwrctrl_trgg;
    struct task_struct *pwrctl_tsk;
    spinlock_t pwrctl_lock;
    struct semaphore pwrctl_wakeup_sema;
#endif

    u16 chipid;
    struct aic_sdio_reg sdio_reg;
};
int aicwf_sdio_writeb(struct aic_sdio_dev *sdiodev, uint regaddr, u8 val);
void aicwf_sdio_hal_irqhandler(struct sdio_func *func);
#if defined(CONFIG_SDIO_PWRCTRL)
    void aicwf_sdio_pwrctl_timer(struct aic_sdio_dev *sdiodev, uint duration);
    int aicwf_sdio_pwr_stctl(struct  aic_sdio_dev *sdiodev, uint target);
#endif
void aicwf_sdio_reg_init(struct aic_sdio_dev *sdiodev);
int aicwf_sdio_func_init(struct aic_sdio_dev *sdiodev);
int aicwf_sdiov3_func_init(struct aic_sdio_dev *sdiodev);
void aicwf_sdio_func_deinit(struct aic_sdio_dev *sdiodev);
#ifdef CONFIG_TX_NETIF_FLOWCTRL
    void aicwf_sdio_tx_netif_flowctrl(struct net_device *ndev, bool state);
#endif
int aicwf_sdio_flow_ctrl(struct aic_sdio_dev *sdiodev);
int aicwf_sdio_flow_ctrl_msg(struct aic_sdio_dev *sdiodev);
int aicwf_sdio_recv_pkt(struct aic_sdio_dev *sdiodev, u32 *buf, u32 size);
bool aicwf_another_ptk(uint8_t *data, uint32_t len);
int aicwf_sdio_send_pkt(struct aic_sdio_dev *sdiodev, u8 *buf, uint count);
void *aicwf_sdio_bus_init(struct aic_sdio_dev *sdiodev);
void aicwf_sdio_release(struct aic_sdio_dev *sdiodev);
void aicwf_sdio_exit(void);
void aicwf_sdio_register(void);
#ifdef CONFIG_DATA_TXRX
    int aicwf_sdio_txpkt(struct aic_sdio_dev *sdiodev, uint8_t *pkt, uint16_t pkt_len);
#endif
int sdio_bustx_thread(void *data);
int sdio_busrx_thread(void *data);
#ifdef CONFIG_DATA_TXRX
    int aicwf_sdio_aggr(struct aic_sdio_dev *sdiodev, uint8_t *pkt, uint16_t len);
    uint8_t crc8_ponl_107(uint8_t *p_buffer, uint16_t cal_size);
    int aicwf_sdio_send(struct aic_sdio_dev *sdiodev, u8 txnow);
    void aicwf_sdio_aggr_send(struct aic_sdio_dev *sdiodev);
    void aicwf_sdio_aggrbuf_reset(struct aicwf_tx_priv *tx_priv);
#endif
//extern void aicwf_hostif_ready(void);

#endif /* AICWF_SDIO_SUPPORT */

int aicwf_rwnx_sdio_platform_init(struct aic_sdio_dev *sdiodev);
int aicwf_rwnx_sdio_platform_deinit(struct aic_sdio_dev *sdiodev);

int aicwf_sdio_probe(struct sdio_func *func);
void aicwf_sdio_remove(struct sdio_func *func);

#endif /*_AICWF_SDIO_H_*/
