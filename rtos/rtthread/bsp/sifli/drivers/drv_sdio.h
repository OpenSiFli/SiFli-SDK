/**
  ******************************************************************************
  * @file   drv_sdio.h
  * @author Sifli software development team
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

#ifndef _DRV_SDIO_H
#define _DRV_SDIO_H
#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>
#include <string.h>
#include <drivers/mmcsd_core.h>
#include <drivers/sdio.h>

#define SDCARD_INSTANCE_TYPE              SD_TypeDef
//#define SDCARD_INSTANCE_TYPE              SD_TypeDef

#define SDIO_BUFF_SIZE       4096
#define SDIO_MAX_FREQ        24000000
#define SDIO_MIN_FREQ        400000
#define SDIO_ALIGN_LEN       32

#ifdef SOC_SF32LB52X
    #define SDCARD_INSTANCE     SDIO1
    #define SDIO_AHB_BASE       (HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET+QSPI2_MEM_BASE)
    #define SDIO_IRQn           SDMMC1_IRQn
#else
    #define SDCARD_INSTANCE     SDIO2
    #define SDIO_AHB_BASE       (HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET+QSPI3_MEM_BASE)
    #define SDIO_IRQn           SDMMC2_IRQn
#endif

#ifndef SDIO_BASE_ADDRESS
    #define SDIO_BASE_ADDRESS    (SDCARD_INSTANCE)
#endif

#ifndef SDIO_CLOCK_FREQ
    #define SDIO_CLOCK_FREQ      (48U * 1000 * 1000)
#endif

#ifndef SDIO_BUFF_SIZE
    #define SDIO_BUFF_SIZE       (4096)
#endif

#ifndef SDIO_ALIGN_LEN
    #define SDIO_ALIGN_LEN       (32)
#endif

#ifndef SDIO_MAX_FREQ
    #define SDIO_MAX_FREQ        (24 * 1000 * 1000)
#endif

#define HW_SDIO_IT_CCRCFAIL                    (SD_IER_CMD_RSP_CRC_MASK)
#define HW_SDIO_IT_DCRCFAIL                    (SD_IER_DATA_CRC_MASK)
#define HW_SDIO_IT_CTIMEOUT                    (SD_IER_CMD_TIMEOUT_MASK)
#define HW_SDIO_IT_DTIMEOUT                    (SD_IER_DATA_TIMEOUT_MASK)
#define HW_SDIO_IT_TXUNDERR                    (SD_IER_FIFO_UNDERRUN_MASK)
#define HW_SDIO_IT_RXOVERR                     (SD_IER_FIFO_OVERRUN_MASK)
#define HW_SDIO_IT_CMDREND                     (SD_IER_CMD_DONE_MASK)
#define HW_SDIO_IT_CMDSENT                     (SD_IER_CMD_SENT_MASK)
#define HW_SDIO_IT_DATAEND                     (SD_IER_DATA_DONE_MASK)
//#define HW_SDIO_IT_STBITERR                    (0x01U << 9)     //
//#define HW_SDIO_IT_DBCKEND                     (0x01U << 10)    //
//#define HW_SDIO_IT_CMDACT                      (0x01U << 11)    //
#define HW_SDIO_IT_TXACT                       (SD_SR_DATA_BUSY)    //
#define HW_SDIO_IT_RXACT                       (SD_SR_DATA_BUSY)    //
//#define HW_SDIO_IT_TXFIFOHE                    (0x01U << 14)    //
//#define HW_SDIO_IT_RXFIFOHF                    (0x01U << 15)    //
//#define HW_SDIO_IT_TXFIFOF                     (0x01U << 16)    //
//#define HW_SDIO_IT_RXFIFOF                     (0x01U << 17)    //
//#define HW_SDIO_IT_TXFIFOE                     (0x01U << 18)    //
//#define HW_SDIO_IT_RXFIFOE                     (0x01U << 19)    //
//#define HW_SDIO_IT_TXDAVL                      (0x01U << 20)    //
//#define HW_SDIO_IT_RXDAVL                      (0x01U << 21)    //
#define HW_SDIO_IT_SDIOIT                      (SD_IER_CMD_DONE_MASK)    //

#define HW_SDIO_ERRORS \
    (HW_SDIO_IT_CCRCFAIL | HW_SDIO_IT_CTIMEOUT | \
     HW_SDIO_IT_DCRCFAIL | HW_SDIO_IT_DTIMEOUT | \
     HW_SDIO_IT_RXOVERR  | HW_SDIO_IT_TXUNDERR)

#define HW_SDIO_POWER_OFF                      (0x00U)
#define HW_SDIO_POWER_ONX                      (0x01U<<SD_PCR_PWR_Pos)
#define HW_SDIO_POWER_UP                       (0x02U)
#define HW_SDIO_POWER_ON                       (0x03U)

#define HW_SDIO_FLOW_ENABLE                    (0x01U << 14)
#define HW_SDIO_BUSWIDE_1B                     (0x00U << SD_DCR_WIRE_MODE_Pos)
#define HW_SDIO_BUSWIDE_4B                     (0x01U << SD_DCR_WIRE_MODE_Pos)
#define HW_SDIO_BUSWIDE_8B                     (0x02U << SD_DCR_WIRE_MODE_Pos)
#define HW_SDIO_BYPASS_ENABLE                  (0x01U << 10)
#define HW_SDIO_IDLE_ENABLE                    (0x01U << 9)
#define HW_SDIO_CLK_ENABLE                     (0x01U << 8)

#define HW_SDIO_SUSPEND_CMD                    (0x01U << 11)
//#define HW_SDIO_CPSM_ENABLE                    (0x01U << 10)
#define HW_SDIO_CPSM_ENABLE                    SD_CCR_CMD_TX_EN

#define HW_SDIO_WAIT_END                       (0x01U << 9)
#define HW_SDIO_WAIT_INT                       (0x01U << 8)
//#define HW_SDIO_RESPONSE_NO                    (0x00U << 6)
//#define HW_SDIO_RESPONSE_SHORT                 (0x01U << 6)
//#define HW_SDIO_RESPONSE_LONG                  (0x03U << 6)
#define HW_SDIO_RESPONSE_NO                    (0x00U << SD_CCR_CMD_HAS_RSP_Pos)
#define HW_SDIO_RESPONSE_SHORT                 (0x01U << SD_CCR_CMD_HAS_RSP_Pos)
#define HW_SDIO_RESPONSE_LONG                  (0x03U << SD_CCR_CMD_HAS_RSP_Pos)


#define HW_SDIO_DATA_LEN_MASK                  (0x01FFFFFFU)

#define HW_SDIO_IO_ENABLE                      (0x01U << 11)
#define HW_SDIO_RWMOD_CK                       (0x01U << 10)
#define HW_SDIO_RWSTOP_ENABLE                  (0x01U << 9)
#define HW_SDIO_RWSTART_ENABLE                 (0x01U << 8)
#define HW_SDIO_DBLOCKSIZE_1                   (0x00U << 4)
#define HW_SDIO_DBLOCKSIZE_2                   (0x01U << 4)
#define HW_SDIO_DBLOCKSIZE_4                   (0x02U << 4)
#define HW_SDIO_DBLOCKSIZE_8                   (0x03U << 4)
#define HW_SDIO_DBLOCKSIZE_16                  (0x04U << 4)
#define HW_SDIO_DBLOCKSIZE_32                  (0x05U << 4)
#define HW_SDIO_DBLOCKSIZE_64                  (0x06U << 4)
#define HW_SDIO_DBLOCKSIZE_128                 (0x07U << 4)
#define HW_SDIO_DBLOCKSIZE_256                 (0x08U << 4)
#define HW_SDIO_DBLOCKSIZE_512                 (0x09U << 4)
#define HW_SDIO_DBLOCKSIZE_1024                (0x0AU << 4)
#define HW_SDIO_DBLOCKSIZE_2048                (0x0BU << 4)
#define HW_SDIO_DBLOCKSIZE_4096                (0x0CU << 4)
#define HW_SDIO_DBLOCKSIZE_8192                (0x0DU << 4)
#define HW_SDIO_DBLOCKSIZE_16384               (0x0EU << 4)
#define HW_SDIO_DMA_ENABLE                     (0x01U << 3)
#define HW_SDIO_STREAM_ENABLE                  (0x01U << 2)
#define HW_SDIO_TO_HOST                        (0x01U << 1)
#define HW_SDIO_DPSM_ENABLE                    SD_DCR_DATA_START

#define HW_SDIO_DATATIMEOUT                    (0x7F000000U)

typedef rt_err_t (*dma_txconfig)(rt_uint32_t *src, rt_uint32_t *dst, int size);
typedef rt_err_t (*dma_rxconfig)(rt_uint32_t *src, rt_uint32_t *dst, int size);
typedef rt_uint32_t (*sdio_clk_get)(SD_TypeDef *hw_sdio);

struct sifli_sdio_des
{
    SD_TypeDef *hw_sdio;
    dma_txconfig txconfig;
    dma_rxconfig rxconfig;
    sdio_clk_get clk_get;
};

struct sifli_sdio_config
{
    SDCARD_INSTANCE_TYPE *Instance;
    struct dma_config dma_rx, dma_tx;
};

/* sifli sdio dirver class */
struct sifli_sdio_class
{
    struct sifli_sdio_des *des;
    const struct sifli_sdio_config *cfg;
    struct rt_mmcsd_host host;
    struct
    {
        DMA_HandleTypeDef handle_rx;
        DMA_HandleTypeDef handle_tx;
    } dma;
};

int rt_sdio_enable_ahb(uint32_t enable_sd_ahb);

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
