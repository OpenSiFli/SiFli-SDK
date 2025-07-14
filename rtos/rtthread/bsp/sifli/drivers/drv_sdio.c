/**
  ******************************************************************************
  * @file   drv_sdio.c
  * @author Sifli software development team
  * @brief SDIO BSP driver
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

#include "board.h"
#include "drv_sdio.h"
#include "drv_config.h"

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_sdio SDIO
  * @brief SDIO BSP driver
  * @{
  */

#ifdef BSP_USING_SD_LINE

//#define DRV_DEBUG
#define LOG_TAG             "drv.sdio"
#include <drv_log.h>

#ifdef SOC_SF32LB52X
    #ifdef SDMMC1_DMA_INSTANCE
        #define SDIO_USING_DMA          (1)
    #endif  //SDMMC2_DMA_INSTANCE
#else
    #ifdef SDMMC2_DMA_INSTANCE
        #define SDIO_USING_DMA          (1)
    #endif  //SDMMC2_DMA_INSTANCE
#endif  //SOC_SF32LB52X
#ifndef SDIO_USING_DMA
    #error "SDIO_USING_DMA must be defined,the DMA function of sdio must be enabled"
#endif
int rt_hw_sdio_init(void);

static struct sifli_sdio_config sdio_config = SDIO_BUS_CONFIG;
static struct sifli_sdio_class sdio_obj;
static struct rt_mmcsd_host *sdio_host;
#ifdef RT_USING_PM
    static struct rt_device rt_sdio_device;
#endif /* RT_USING_PM */

#define SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS    (100000)

#define RTHW_SDIO_LOCK(_sdio)   rt_mutex_take(&_sdio->mutex, RT_WAITING_FOREVER)
#define RTHW_SDIO_UNLOCK(_sdio) rt_mutex_release(&_sdio->mutex);

struct sdio_pkg
{
    struct rt_mmcsd_cmd *cmd;
    void *buff;
    rt_uint32_t flag;
};

struct rthw_sdio
{
    struct rt_mmcsd_host *host;
    struct sifli_sdio_des sdio_des;
    struct rt_event event;
    struct rt_mutex mutex;
    struct sdio_pkg *pkg;
    uint32_t ahb_en;        // flag to check if ahb access enabled
    uint32_t cmd_to;        // command time out value, it should related with freq
    uint32_t part_offset;   // start offset for read/write with device interface
    uint32_t cur_freq;      // current sd frequency
};

ALIGN(SDIO_ALIGN_LEN)
//static rt_uint8_t cache_buf[SDIO_BUFF_SIZE];
HAL_RETM_BSS_SECT(cache_buf, static rt_uint8_t cache_buf[SDIO_BUFF_SIZE]);


static rt_uint32_t sifli_sdio_clk_get(SD_TypeDef *hw_sdio)
{
    return SDIO_CLOCK_FREQ;
}

/**
  * @brief  This function get order from sdio.
  * @param  data
  * @retval sdio  order
  */
static int get_order(rt_uint32_t data)
{
    int order = 0;

    switch (data)
    {
    case 1:
        order = 0;
        break;
    case 2:
        order = 1;
        break;
    case 4:
        order = 2;
        break;
    case 8:
        order = 3;
        break;
    case 16:
        order = 4;
        break;
    case 32:
        order = 5;
        break;
    case 64:
        order = 6;
        break;
    case 128:
        order = 7;
        break;
    case 256:
        order = 8;
        break;
    case 512:
        order = 9;
        break;
    case 1024:
        order = 10;
        break;
    case 2048:
        order = 11;
        break;
    case 4096:
        order = 12;
        break;
    case 8192:
        order = 13;
        break;
    case 16384:
        order = 14;
        break;
    default :
        order = 0;
        break;
    }

    return order;
}


#define _DUMP_REG_DEBUG         (0)
#define _SDHCI_DUMP_RCNT        (23)
static uint32_t sdhci_reg_arr[_SDHCI_DUMP_RCNT];
static void dump_sdio_reg(void)
{
    int i;
    uint32_t *sdhci_base_reg;
    sdhci_base_reg = (uint32_t *)SDCARD_INSTANCE;;

    for (i = 0; i < _SDHCI_DUMP_RCNT; i++)
    {
        sdhci_reg_arr[i] = *sdhci_base_reg++;
#if _DUMP_REG_DEBUG
        rt_kprintf("%08x ", sdhci_reg_arr[i]);
        if ((i + 1) % 8 == 0)
        {
            rt_kprintf("\n");
        }
        rt_kprintf("\n");
#endif
    }
}

static void recov_sdio_reg(void)
{
    int i;
    uint32_t *sdhci_base_reg;

    sdhci_base_reg = (uint32_t *)SDCARD_INSTANCE;

    for (i = 0; i < _SDHCI_DUMP_RCNT; i++)
    {
        *sdhci_base_reg = sdhci_reg_arr[i];
        // read only reg: 0x10, 0x14, 0x18, 0x1c for response
        // 0x24 for sr, 0x30 for clear sr, w1c;
        // 0x40 , 0x44, 0x48 for capbility
        // 0x54 for adma error status
#if _DUMP_REG_DEBUG
        rt_kprintf("%08x ", *sdhci_base_reg);
        if ((i + 1) % 8 == 0)
        {
            rt_kprintf("\n");
        }
        rt_kprintf("\n");
#endif
        sdhci_base_reg++;
    }
}

void rt_hw_sdio_timeout_handle(void)
{
    dump_sdio_reg();
    HAL_RCC_ResetModule(RCC_MOD_SDMMC1);
    rt_thread_mdelay(1);
    recov_sdio_reg();
}


/**
  * @brief  This function wait sdio completed.
  * @param  sdio  rthw_sdio
  * @retval None
  */
static void rthw_sdio_wait_completed(struct rthw_sdio *sdio)
{
    rt_uint32_t status, rci;
    struct rt_mmcsd_cmd *cmd = sdio->pkg->cmd;
    struct rt_mmcsd_data *data = cmd->data;
    SD_TypeDef *hw_sdio = sdio->sdio_des.hw_sdio;

    if (rt_event_recv(&sdio->event, 0xffffffff, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      rt_tick_from_millisecond(500), &status) != RT_EOK)
    {
        LOG_E("wait %d completed timeout 0x%08x,arg 0x%08x\n", cmd->cmd_code, HAL_SDMMC_GET_STA(hw_sdio), cmd->arg);
        cmd->err = -RT_ETIMEOUT;
        rt_hw_sdio_timeout_handle();
        return;
    }

    if (sdio->pkg == RT_NULL)
    {
        LOG_E("sdio->pkg NULL");
        return;
    }

    rci = HAL_SDMMC_GET_RCI(hw_sdio);
    if ((resp_type(cmd) == RESP_R1) || (resp_type(cmd) == RESP_R1B))
    {
        while (rci != cmd->cmd_code)
            rci = HAL_SDMMC_GET_RCI(hw_sdio);
    }

    //cmd->resp[0] = hw_sdio->resp1;
    //cmd->resp[1] = hw_sdio->resp2;
    //cmd->resp[2] = hw_sdio->resp3;
    //cmd->resp[3] = hw_sdio->resp4;
    HAL_SDMMC_GET_RESP(hw_sdio, (uint32_t *)cmd->resp);
    if (resp_type(cmd) == RESP_R2)
    {
        // FOR R2, it need 128 bits response, high/low words should switch.
        // least 8 bit has been removed, so need fill 8 bits at least bits
        uint32_t temp;
        // switch for [0] as highest
        temp = cmd->resp[0];
        cmd->resp[0] = cmd->resp[3];
        cmd->resp[3] = temp;
        temp = cmd->resp[1];
        cmd->resp[1] = cmd->resp[2];
        cmd->resp[2] = temp;

        // << 8
        cmd->resp[0] = (cmd->resp[0] << 8) | (cmd->resp[1] >> 24);
        cmd->resp[1] = (cmd->resp[1] << 8) | (cmd->resp[2] >> 24);
        cmd->resp[2] = (cmd->resp[2] << 8) | (cmd->resp[3] >> 24);
        cmd->resp[3] = (cmd->resp[3] << 8) ;
        LOG_D("Respones 4 words, switch order");
    }

    if (status & HW_SDIO_ERRORS)
    {
        if ((status & HW_SDIO_IT_CCRCFAIL) && (resp_type(cmd) & (RESP_R3 | RESP_R4)))
        {
            cmd->err = RT_EOK;
        }
        else
        {
            cmd->err = -RT_ERROR;
        }

        if (status & HW_SDIO_IT_CTIMEOUT)
        {
            cmd->err = -RT_ETIMEOUT;
            rt_hw_sdio_timeout_handle();
        }

        if ((status & HW_SDIO_IT_DCRCFAIL) && (data != NULL))
        {
            data->err = -RT_ERROR;
        }

        if ((status & HW_SDIO_IT_DTIMEOUT) && (data != NULL))
        {
            data->err = -RT_ETIMEOUT;
            rt_hw_sdio_timeout_handle();
        }

        if (cmd->err == RT_EOK)
        {
            LOG_D("sta0 %d:0x%08X [%08X %08X %08X %08X], tick %d\n", rci, status, cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3], rt_tick_get());
        }
        else
        {
            LOG_D("err:0x%08x, %s%s%s%s%s%s%s cmd:%d arg:0x%08x rw:%c len:%d blksize:%d, rci:%d, tick %d\n",
                  status,
                  status & HW_SDIO_IT_CCRCFAIL  ? "CCRCFAIL "    : "",
                  status & HW_SDIO_IT_DCRCFAIL  ? "DCRCFAIL "    : "",
                  status & HW_SDIO_IT_CTIMEOUT  ? "CTIMEOUT "    : "",
                  status & HW_SDIO_IT_DTIMEOUT  ? "DTIMEOUT "    : "",
                  status & HW_SDIO_IT_TXUNDERR  ? "TXUNDERR "    : "",
                  status & HW_SDIO_IT_RXOVERR   ? "RXOVERR "     : "",
                  status == 0                   ? "NULL"         : "",
                  cmd->cmd_code,
                  cmd->arg,
                  data ? (data->flags & DATA_DIR_WRITE ?  'w' : 'r') : '-',
                  data ? data->blks * data->blksize : 0,
                  data ? data->blksize : 0,
                  rci, rt_tick_get()
                 );
        }

    }
    else
    {
        cmd->err = RT_EOK;
        if (data != NULL)
            data->err = RT_EOK;
        LOG_D("sta %d:0x%08X [%08X %08X %08X %08X] tick %d\n", rci, status, cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3], rt_tick_get());
    }
}

/**
  * @brief  This function transfer data by dma.
  * @param  sdio  rthw_sdio
  * @param  pkg   sdio package
  * @retval None
  */
static void rthw_sdio_transfer_by_dma(struct rthw_sdio *sdio, struct sdio_pkg *pkg)
{
    struct rt_mmcsd_data *data;
    int size;
    void *buff;
    SD_TypeDef *hw_sdio;

    if ((RT_NULL == pkg) || (RT_NULL == sdio))
    {
        LOG_E("rthw_sdio_transfer_by_dma invalid args");
        return;
    }

    data = pkg->cmd->data;
    if (RT_NULL == data)
    {
        LOG_E("rthw_sdio_transfer_by_dma invalid args");
        return;
    }

    buff = pkg->buff;
    if (RT_NULL == buff)
    {
        LOG_E("rthw_sdio_transfer_by_dma invalid args");
        return;
    }
    hw_sdio = sdio->sdio_des.hw_sdio;
    size = data->blks * data->blksize;

    if (data->flags & DATA_DIR_WRITE)
    {
        mpu_dcache_clean(buff, (uint32_t)size);
        sdio->sdio_des.txconfig((rt_uint32_t *)buff, (rt_uint32_t *)&hw_sdio->FIFO, size);
        //hw_sdio->DCTR |= HW_SDIO_DMA_ENABLE;
        // use ext-dma to replace it, sram to sdcard?
    }
    else if (data->flags & DATA_DIR_READ)
    {
        if (IS_DCACHED_RAM(buff))
            SCB_InvalidateDCache_by_Addr(buff, size);
        sdio->sdio_des.rxconfig((rt_uint32_t *)&hw_sdio->FIFO, (rt_uint32_t *)buff, size);
        //hw_sdio->dctrl |= HW_SDIO_DMA_ENABLE | HW_SDIO_DPSM_ENABLE;
        // use ext-dma to replace, sdcard to ram?
    }
}

/**
  * @brief  This function transfer data by cpu.
  * @param  sdio  rthw_sdio
  * @param  pkg   sdio package
  * @retval None
  */
static void rthw_sdio_transfer_by_cpu(struct rthw_sdio *sdio, struct sdio_pkg *pkg)
{
    struct rt_mmcsd_data *data;
    int size;
    void *buff;
    SD_TypeDef *hw_sdio;

    if ((RT_NULL == pkg) || (RT_NULL == sdio))
    {
        LOG_E("rthw_sdio_transfer_by_cpu invalid args");
        return;
    }

    data = pkg->cmd->data;
    if (RT_NULL == data)
    {
        LOG_E("rthw_sdio_transfer_by_cpu invalid args");
        return;
    }

    buff = pkg->buff;
    if (RT_NULL == buff)
    {
        LOG_E("rthw_sdio_transfer_by_cpu invalid args");
        return;
    }
    hw_sdio = sdio->sdio_des.hw_sdio;
    size = data->blks * data->blksize;

    if (data->flags & DATA_DIR_WRITE)
    {
        //HAL_SDMMC_SET_DATA_START(hw_sdio, HW_SDIO_DPSM_ENABLE);
        HAL_SDMMC_WIRTE(hw_sdio, (uint32_t *)buff, size);
    }
    else if (data->flags & DATA_DIR_READ)
    {
        //HAL_SDMMC_SET_DATA_START(hw_sdio, HW_SDIO_DPSM_ENABLE);
        HAL_SDMMC_READ(hw_sdio, (uint32_t *)buff, size);
    }
#if 0
    {
        uint32_t *tbuf = (uint32_t *)buff;
        int i;
        rt_kprintf("RW flag 0x%x :\n", data->flags);
        for (i = 0; i < size / 4; i++)
        {
            rt_kprintf(" 0x%08x ", *tbuf++);
            if ((i & 7) == 7)
                rt_kprintf("\n");
        }
        rt_kprintf("\n");
    }
#endif
}


/**
  * @brief  This function send command.
  * @param  sdio  rthw_sdio
  * @param  pkg   sdio package
  * @retval None
  */
static void rthw_sdio_send_command(struct rthw_sdio *sdio, struct sdio_pkg *pkg)
{
    struct rt_mmcsd_cmd *cmd = pkg->cmd;
    struct rt_mmcsd_data *data = cmd->data;
    SD_TypeDef *hw_sdio = sdio->sdio_des.hw_sdio;
    rt_uint32_t reg_cmd;

    /* save pkg */
    sdio->pkg = pkg;

    LOG_D("CMD:%d ARG:0x%08x RES:%s%s%s%s%s%s%s%s%s rw:%c len:%d blksize:%d, tick %d",
          cmd->cmd_code,
          cmd->arg,
          resp_type(cmd) == RESP_NONE ? "NONE"  : "",
          resp_type(cmd) == RESP_R1  ? "R1"  : "",
          resp_type(cmd) == RESP_R1B ? "R1B"  : "",
          resp_type(cmd) == RESP_R2  ? "R2"  : "",
          resp_type(cmd) == RESP_R3  ? "R3"  : "",
          resp_type(cmd) == RESP_R4  ? "R4"  : "",
          resp_type(cmd) == RESP_R5  ? "R5"  : "",
          resp_type(cmd) == RESP_R6  ? "R6"  : "",
          resp_type(cmd) == RESP_R7  ? "R7"  : "",
          data ? (data->flags & DATA_DIR_WRITE ?  'w' : 'r') : '-',
          data ? data->blks * data->blksize : 0,
          data ? data->blksize : 0, rt_tick_get()
         );
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#ifdef BSP_PM_FREQ_SCALING
    rt_pm_hw_device_start();
#endif
#endif

    // switch to normal command mode before set command
    if (sdio->ahb_en)
        HAL_SDMMC_SWITCH_NORMAL(hw_sdio);

    /* config data reg */
    if (data != RT_NULL)
    {
        rt_uint32_t dir = 0;
        rt_uint32_t size = data->blks * data->blksize;
        int res;
        rt_uint32_t wire;

        if (sdio->host->io_cfg.bus_width == MMCSD_BUS_WIDTH_8)
        {
            wire = HW_SDIO_BUSWIDE_8B;
        }
        else if (sdio->host->io_cfg.bus_width == MMCSD_BUS_WIDTH_4)
        {
            wire = HW_SDIO_BUSWIDE_4B;
        }
        else
        {
            wire = HW_SDIO_BUSWIDE_1B;
        }

        //hw_sdio->dctrl = 0;
        res = HAL_SDMMC_CLR_DATA_CTRL(hw_sdio);
        //hw_sdio->dtimer = HW_SDIO_DATATIMEOUT;
        res |= HAL_SDMMC_SET_TIMEOUT(hw_sdio, sdio->cmd_to * 5);
        //hw_sdio->dlen = size;
        res |= HAL_SDMMC_SET_DATALEN(hw_sdio, size);
        //order = get_order(data->blksize);
        dir = (data->flags & DATA_DIR_READ) ? HAL_SDMMC_DATA_CARD2HOST : HAL_SDMMC_DATA_HOST2CARD;
        //hw_sdio->dctrl = HW_SDIO_IO_ENABLE | (order << 4) | dir;
        //res |= HAL_SDMMC_SET_DATA_CTRL(hw_sdio, data->blksize, dir, HAL_SDMMC_WIRE_SINGLE, HAL_SDMMC_DATA_BLOCK_MODE);
        res |= HAL_SDMMC_SET_DATA_CTRL(hw_sdio, data->blksize, dir, wire, HAL_SDMMC_DATA_BLOCK_MODE);
        if (res != HAL_OK)
            LOG_D("HAL set error %d\n", res);
        if (data->flags & DATA_DIR_READ)
            HAL_SDMMC_SET_DATA_START(hw_sdio, HW_SDIO_DPSM_ENABLE);

#ifdef SDIO_USING_DMA
        /* transfer config */
        rthw_sdio_transfer_by_dma(sdio, pkg);
#endif
        /* wait completed */
        //rthw_sdio_wait_completed(sdio);
        LOG_D("dir: %d, start %d, length %d, blksize %d\n", dir, cmd->arg, size, data->blksize);
    }
    else
    {
        HAL_SDMMC_SET_TIMEOUT(hw_sdio, sdio->cmd_to);
    }

    /* open irq */
    rt_uint32_t mask = HAL_SDMMC_GET_IRQ_MASK(hw_sdio);
    mask |= (HW_SDIO_IT_CMDSENT | HW_SDIO_IT_CMDREND | HW_SDIO_ERRORS);
    if (data != RT_NULL)
    {
        mask |= HW_SDIO_IT_DATAEND;
    }
    if (resp_type(cmd) & (RESP_R3))
        mask &= ~(HW_SDIO_IT_CCRCFAIL);

    HAL_SDMMC_SET_IRQ_MASK(hw_sdio, mask);

    /* config cmd response */
    if (resp_type(cmd) == RESP_NONE)
        reg_cmd = 0; //HW_SDIO_RESPONSE_NO;
    else if (resp_type(cmd) == RESP_R2)
        reg_cmd = 3; //HW_SDIO_RESPONSE_LONG;
    else
        reg_cmd = 1; //HW_SDIO_RESPONSE_SHORT;

    /* send cmd */
    //hw_sdio->arg = cmd->arg;
    //hw_sdio->cmd = reg_cmd;
    HAL_SDMMC_SET_CMD(hw_sdio, cmd->cmd_code, reg_cmd, cmd->arg);

    /* wait completed */
    rthw_sdio_wait_completed(sdio);

    /* Waiting for data to be sent to completion */
    if (data != RT_NULL)
    {
        volatile rt_uint32_t count = SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS;
        //LOG_D("before data: 0x%08x\n",HAL_SDMMC_GET_STA(hw_sdio));
        int dma_res;
#ifndef SDIO_USING_DMA
        rthw_sdio_transfer_by_cpu(sdio, pkg);

        if (data->flags & DATA_DIR_WRITE)   // for write, start in irq, need wait data done
        {
            // open error and data end irq
            mask = HAL_SDMMC_GET_IRQ_MASK(hw_sdio);
            mask |= (HW_SDIO_IT_DATAEND | HW_SDIO_IT_CMDREND | HW_SDIO_ERRORS);
            HAL_SDMMC_SET_IRQ_MASK(hw_sdio, mask);
            rthw_sdio_wait_completed(sdio);
        }
#else
        if (data->flags & DATA_DIR_WRITE)
            dma_res = HAL_DMA_PollForTransfer(&sdio_obj.dma.handle_tx, HAL_DMA_FULL_TRANSFER, 1000);
        else if (data->flags & DATA_DIR_READ)
            dma_res = HAL_DMA_PollForTransfer(&sdio_obj.dma.handle_rx, HAL_DMA_FULL_TRANSFER, 1000);
        RT_ASSERT(dma_res == HAL_OK);
#endif
        //LOG_D("after data: 0x%08x\n",HAL_SDMMC_GET_STA(hw_sdio));
        while (count && (HAL_SDMMC_GET_STA(hw_sdio) & (HW_SDIO_IT_TXACT)))
        {
            count--;
        }

        if ((count == 0) || (HAL_SDMMC_GET_STA(hw_sdio) & HW_SDIO_ERRORS))
        {
            cmd->err = -RT_ERROR;
            LOG_D("count = %d, status 0x%x\n", count, HAL_SDMMC_GET_STA(hw_sdio));
        }
        //LOG_D("0x%08x\n",HAL_SDMMC_GET_STA(hw_sdio));
    }

    /* close irq, keep sdio irq ??? */
    //hw_sdio->mask = hw_sdio->mask & HW_SDIO_IT_SDIOIT ? HW_SDIO_IT_SDIOIT : 0x00;
    mask = HAL_SDMMC_GET_IRQ_MASK(hw_sdio);
    mask = mask & HW_SDIO_IT_SDIOIT ? HW_SDIO_IT_SDIOIT : 0x00;
    HAL_SDMMC_SET_IRQ_MASK(hw_sdio, mask);

    //HAL_SDMMC_CLR_DATA_CTRL(hw_sdio);

    /* clear pkg */
    sdio->pkg = RT_NULL;

    // recover to AHB mode if it enabled
    if (sdio->ahb_en)
    {
        if ((data != RT_NULL) && (data->flags & DATA_DIR_WRITE))
        {
            uint32_t addr, len;
            len = data->blksize * data->blks;
            if ((sdio->host->card->flags & CARD_FLAG_SDHC) || (sdio->host->card->flags & CARD_FLAG_SDXC)) // SDHC/SDXC based on block
            {
                addr = cmd->arg * data->blksize + SDIO_AHB_BASE;
            }
            else
            {
                addr = cmd->arg + SDIO_AHB_BASE;
            }
            SCB_InvalidateDCache_by_Addr((void *)addr, len);
            LOG_I("DCache by addr 0x%x with size 0x%x\n", addr, len);
        }
        HAL_SDMMC_SWITCH_AHB(hw_sdio);
    }
#ifdef RT_USING_PM
#ifdef BSP_PM_FREQ_SCALING
    rt_pm_hw_device_stop();
#endif
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif

    //LOG_I("set comd func done\n");
}

/**
  * @brief  This function send sdio request.
  * @param  sdio  rthw_sdio
  * @param  req   request
  * @retval None
  */
static void rthw_sdio_request(struct rt_mmcsd_host *host, struct rt_mmcsd_req *req)
{
    struct sdio_pkg pkg;
    struct rthw_sdio *sdio = host->private_data;
    struct rt_mmcsd_data *data;

    RTHW_SDIO_LOCK(sdio);

    if (req->cmd != RT_NULL)
    {
        memset(&pkg, 0, sizeof(pkg));
        data = req->cmd->data;
        pkg.cmd = req->cmd;

        if (data != RT_NULL)
        {
            rt_uint32_t size = data->blks * data->blksize;

            RT_ASSERT(size <= SDIO_BUFF_SIZE);

            //pkg.buff = data->buf;
            //if ((rt_uint32_t)data->buf & (SDIO_ALIGN_LEN - 1))
            // replace buffer any way for SRAM buffer and aligned issue
            {
                SCB_InvalidateDCache_by_Addr(cache_buf, size);
                pkg.buff = cache_buf;
                if (data->flags & DATA_DIR_WRITE)
                {
                    memcpy(cache_buf, data->buf, size);
                }
            }
        }

        rthw_sdio_send_command(sdio, &pkg);

        //if ((data != RT_NULL) && (data->flags & DATA_DIR_READ) && ((rt_uint32_t)data->buf & (SDIO_ALIGN_LEN - 1)))
        if ((data != RT_NULL) && (data->flags & DATA_DIR_READ)) // always do copy when buffer replaced.
        {
            memcpy(data->buf, cache_buf, data->blksize * data->blks);
        }
    }

    if (req->stop != RT_NULL)
    {
        memset(&pkg, 0, sizeof(pkg));
        pkg.cmd = req->stop;
        rthw_sdio_send_command(sdio, &pkg);
    }

    RTHW_SDIO_UNLOCK(sdio);

    mmcsd_req_complete(sdio->host);
}

static int rthw_sdio_set_clk(struct rt_mmcsd_host *host, uint32_t clk)
{
    rt_uint32_t div, clk_src;
    struct rthw_sdio *sdio = host->private_data;
    SD_TypeDef *hw_sdio = sdio->sdio_des.hw_sdio;

    clk_src = sdio->sdio_des.clk_get(sdio->sdio_des.hw_sdio);
    if (clk_src < 400 * 1000)
    {
        LOG_E("The clock rate is too low! rata:%d", clk_src);
        return 1;
    }

    if (clk > host->freq_max)
        clk = host->freq_max;

    if (clk > clk_src)
    {
        LOG_W("Setting rate is greater than clock source rate.");
        clk = clk_src;
    }

    if (clk != 0)
        div = clk_src / clk;
    else
        div = 1;

    if (clk / 10 > HAL_SDMMC_DEFAULT_TIMEOUT)
        sdio->cmd_to = clk / 10;
    else
        sdio->cmd_to = HAL_SDMMC_DEFAULT_TIMEOUT;

    LOG_D("SDIO CLK src %d, dst %d, div %d\n", clk_src, clk, div);

    HAL_SDMMC_CLK_SET(hw_sdio, div, 1);

    return 0;
}

void sdio_update_clk(void)
{
    rthw_sdio_set_clk(sdio_host, SDIO_MAX_FREQ);
}

/**
  * @brief  This function config sdio.
  * @param  host    rt_mmcsd_host
  * @param  io_cfg  rt_mmcsd_io_cfg
  * @retval None
  */
static void rthw_sdio_iocfg(struct rt_mmcsd_host *host, struct rt_mmcsd_io_cfg *io_cfg)
{
    rt_uint32_t clk = io_cfg->clock;
    struct rthw_sdio *sdio = (struct rthw_sdio *)host->private_data;
    SD_TypeDef *hw_sdio = sdio->sdio_des.hw_sdio;

    LOG_D("clk:%d width:%s%s%s power:%s%s%s",
          clk,
          io_cfg->bus_width == MMCSD_BUS_WIDTH_8 ? "8" : "",
          io_cfg->bus_width == MMCSD_BUS_WIDTH_4 ? "4" : "",
          io_cfg->bus_width == MMCSD_BUS_WIDTH_1 ? "1" : "",
          io_cfg->power_mode == MMCSD_POWER_OFF ? "OFF" : "",
          io_cfg->power_mode == MMCSD_POWER_UP ? "UP" : "",
          io_cfg->power_mode == MMCSD_POWER_ON ? "ON" : ""
         );

    RTHW_SDIO_LOCK(sdio);

    // set clock divider
    sdio->cur_freq = clk;
    rthw_sdio_set_clk(host, clk);

    switch (io_cfg->power_mode)
    {
    case MMCSD_POWER_OFF:
        //hw_sdio->power = HW_SDIO_POWER_OFF;
        HAL_SDMMC_POWER_SET(hw_sdio, HW_SDIO_POWER_OFF);
        break;
    case MMCSD_POWER_UP:
        //hw_sdio->power = HW_SDIO_POWER_UP;
        HAL_SDMMC_POWER_SET(hw_sdio, HW_SDIO_POWER_UP);
        break;
    case MMCSD_POWER_ON:
        //hw_sdio->power = HW_SDIO_POWER_ON;
        HAL_SDMMC_POWER_SET(hw_sdio, HW_SDIO_POWER_ON);
        break;
    default:
        LOG_W("unknown power_mode %d", io_cfg->power_mode);
        break;
    }

    RTHW_SDIO_UNLOCK(sdio);
}

/**
  * @brief  This function update sdio interrupt.
  * @param  host    rt_mmcsd_host
  * @param  enable
  * @retval None
  */
void rthw_sdio_irq_update(struct rt_mmcsd_host *host, rt_int32_t enable)
{
    struct rthw_sdio *sdio = host->private_data;
    SD_TypeDef *hw_sdio = sdio->sdio_des.hw_sdio;

    if (enable)
    {
        LOG_D("enable sdio irq");
        //hw_sdio->mask |= HW_SDIO_IT_SDIOIT;
        rt_uint32_t mask = HAL_SDMMC_GET_IRQ_MASK(hw_sdio);
        mask |= HW_SDIO_IT_SDIOIT;
        HAL_SDMMC_SET_IRQ_MASK(hw_sdio, mask);
    }
    else
    {
        LOG_D("disable sdio irq");
        //hw_sdio->mask &= ~HW_SDIO_IT_SDIOIT;
        rt_uint32_t mask = HAL_SDMMC_GET_IRQ_MASK(hw_sdio);
        mask &= ~HW_SDIO_IT_SDIOIT;
        HAL_SDMMC_SET_IRQ_MASK(hw_sdio, mask);
    }
}

/**
  * @brief  This function delect sdcard.
  * @param  host    rt_mmcsd_host
  * @retval 0x01
  */
static rt_int32_t rthw_sd_delect(struct rt_mmcsd_host *host)
{
    LOG_D("try to detect device");
    return 0x01;
}

/**
  * @brief  This function interrupt process function.
  * @param  host  rt_mmcsd_host
  * @retval None
  */
void rthw_sdio_irq_process(struct rt_mmcsd_host *host)
{
    int complete = 0;
    struct rthw_sdio *sdio = host->private_data;
    SD_TypeDef *hw_sdio = sdio->sdio_des.hw_sdio;
    rt_uint32_t intstatus = HAL_SDMMC_GET_STA(hw_sdio);

    //rt_kprintf("SD IRQ 0x%x\n",intstatus);
    if (intstatus & HW_SDIO_ERRORS)
    {
        //hw_sdio->icr = HW_SDIO_ERRORS;
        HAL_SDMMC_CLR_INT(hw_sdio, HW_SDIO_ERRORS);
        complete = 1;
    }
    else
    {
        if (intstatus & HW_SDIO_IT_CMDREND)
        {
            //hw_sdio->icr = HW_SDIO_IT_CMDREND;
            HAL_SDMMC_CLR_INT(hw_sdio, HW_SDIO_IT_CMDREND);
            //complete = 1;

            if (sdio->pkg != RT_NULL)
            {
#if 0
                if (!(intstatus & HW_SDIO_IT_RXACT))
                    complete = 1;
#else
                if (!sdio->pkg->cmd->data)
                {
                    complete = 1;
                }
                else if ((sdio->pkg->cmd->data->flags & DATA_DIR_WRITE))
                {
                    // enable data, set complete to let fifo write, and wait next irq for data done
                    HAL_SDMMC_SET_DATA_START(hw_sdio, HW_SDIO_DPSM_ENABLE);
#ifndef SDIO_USING_DMA
                    complete = 1;
#endif
                }
                else if (sdio->pkg->cmd->data->flags & DATA_DIR_READ)
                {
                    // enable data, wait data done(fill to fifo), read fifo after that.
                    //HAL_SDMMC_SET_DATA_START(hw_sdio, HW_SDIO_DPSM_ENABLE);
                    //complete = 1;
                }
#endif
            }
        }

        if (intstatus & HW_SDIO_IT_CMDSENT)
        {
            //hw_sdio->icr = HW_SDIO_IT_CMDSENT;
            HAL_SDMMC_CLR_INT(hw_sdio, HW_SDIO_IT_CMDSENT);

            if (resp_type(sdio->pkg->cmd) == RESP_NONE)
            {
                complete = 1;
            }
        }

        if (intstatus & HW_SDIO_IT_DATAEND)
        {
            //hw_sdio->icr = HW_SDIO_IT_DATAEND;
            HAL_SDMMC_CLR_INT(hw_sdio, HW_SDIO_IT_DATAEND);
            complete = 1;
        }
    }

    if ((intstatus & HW_SDIO_IT_SDIOIT) && (HAL_SDMMC_GET_IRQ_MASK(hw_sdio) & HW_SDIO_IT_SDIOIT))
    {
        //hw_sdio->icr = HW_SDIO_IT_SDIOIT;
        HAL_SDMMC_CLR_INT(hw_sdio, HW_SDIO_IT_SDIOIT);
        //sdio_irq_wakeup(host);
    }

    if (complete)
    {
        //hw_sdio->mask &= ~HW_SDIO_ERRORS;
        rt_uint32_t mask = HAL_SDMMC_GET_IRQ_MASK(hw_sdio);
        mask &= ~HW_SDIO_ERRORS;
        HAL_SDMMC_SET_IRQ_MASK(hw_sdio, mask);
        rt_event_send(&sdio->event, intstatus);
    }
}

static const struct rt_mmcsd_host_ops ops =
{
    rthw_sdio_request,
    rthw_sdio_iocfg,
    rthw_sd_delect,
    rthw_sdio_irq_update,
};

/**
  * @brief  This function create mmcsd host.
  * @param  sdio_des  sifli_sdio_des
  * @retval rt_mmcsd_host
  */
struct rt_mmcsd_host *sdio_host_create(struct sifli_sdio_des *sdio_des)
{
    struct rt_mmcsd_host *host;
    struct rthw_sdio *sdio = RT_NULL;

    if ((sdio_des == RT_NULL) || (sdio_des->txconfig == RT_NULL) || (sdio_des->rxconfig == RT_NULL))
    {
        LOG_E("L:%d F:%s %s %s %s",
              (sdio_des == RT_NULL ? "sdio_des is NULL" : ""),
              (sdio_des ? (sdio_des->txconfig ? "txconfig is NULL" : "") : ""),
              (sdio_des ? (sdio_des->rxconfig ? "rxconfig is NULL" : "") : "")
             );
        return RT_NULL;
    }

    sdio = rt_malloc(sizeof(struct rthw_sdio));
    if (sdio == RT_NULL)
    {
        LOG_E("L:%d F:%s malloc rthw_sdio fail");
        return RT_NULL;
    }
    rt_memset(sdio, 0, sizeof(struct rthw_sdio));

    host = mmcsd_alloc_host();
    if (host == RT_NULL)
    {
        LOG_E("L:%d F:%s mmcsd alloc host fail");
        rt_free(sdio);
        return RT_NULL;
    }

    rt_memcpy(&sdio->sdio_des, sdio_des, sizeof(struct sifli_sdio_des));
    sdio->sdio_des.hw_sdio = (sdio_des->hw_sdio == RT_NULL ? (SD_TypeDef *)SDIO_BASE_ADDRESS : sdio_des->hw_sdio);
    sdio->sdio_des.clk_get = (sdio_des->clk_get == RT_NULL ? sifli_sdio_clk_get : sdio_des->clk_get);

    rt_event_init(&sdio->event, "sdio", RT_IPC_FLAG_FIFO);
    rt_mutex_init(&sdio->mutex, "sdio", RT_IPC_FLAG_FIFO);


    /* set host defautl attributes */
    host->ops = &ops;
    host->freq_min = SDIO_MIN_FREQ;
    host->freq_max = SDIO_MAX_FREQ; //SDIO_MAX_FREQ; // ??
    host->valid_ocr = 0X00FFFF80;/* The voltage range supported is 1.65v-3.6v */

    // set 1 bit only, config it when 4 bits ready
    host->flags = MMCSD_MUTBLKWRITE | MMCSD_SUP_SDIO_IRQ | MMCSD_BUSWIDTH_4;

    host->max_seg_size = SDIO_BUFF_SIZE;
    host->max_dma_segs = 1;
    host->max_blk_size = 512;
    host->max_blk_count = 512;

    /* link up host and sdio */
    sdio->host = host;
    host->private_data = sdio;

    sdio->ahb_en = 0;
    sdio->cmd_to = HAL_SDMMC_DEFAULT_TIMEOUT;
    sdio->part_offset = 0;

    rthw_sdio_irq_update(host, 1);

    /* ready to change */
    mmcsd_change(host);

    return host;
}

/**
  * @brief  This function configures the DMATX.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_TxConfig(uint32_t *src, uint32_t *dst, uint32_t BufferSize)
{
    static uint32_t size = 0;
    size += BufferSize * 4;
    sdio_obj.cfg = &sdio_config;
    sdio_obj.dma.handle_tx.Instance = sdio_config.dma_tx.Instance;
    sdio_obj.dma.handle_tx.Init.Request             = sdio_config.dma_tx.request;
    sdio_obj.dma.handle_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    sdio_obj.dma.handle_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    sdio_obj.dma.handle_tx.Init.MemInc              = DMA_MINC_ENABLE;
    sdio_obj.dma.handle_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    sdio_obj.dma.handle_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    sdio_obj.dma.handle_tx.Init.Mode                = DMA_NORMAL;
    sdio_obj.dma.handle_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    sdio_obj.dma.handle_tx.Init.BurstSize           = 1;

    HAL_DMA_DeInit(&sdio_obj.dma.handle_tx);
    HAL_DMA_Init(&sdio_obj.dma.handle_tx);

    HAL_DMA_Start(&sdio_obj.dma.handle_tx, (uint32_t)src, (uint32_t)dst, BufferSize);
}

/**
  * @brief  This function configures the DMARX.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_RxConfig(uint32_t *src, uint32_t *dst, uint32_t BufferSize)
{
    sdio_obj.cfg = &sdio_config;
    sdio_obj.dma.handle_rx.Instance = sdio_config.dma_rx.Instance;
    sdio_obj.dma.handle_rx.Init.Request             = sdio_config.dma_rx.request;
    sdio_obj.dma.handle_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    sdio_obj.dma.handle_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    sdio_obj.dma.handle_rx.Init.MemInc              = DMA_MINC_ENABLE;
    sdio_obj.dma.handle_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    sdio_obj.dma.handle_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    sdio_obj.dma.handle_rx.Init.Mode                = DMA_NORMAL;
    sdio_obj.dma.handle_rx.Init.Priority            = DMA_PRIORITY_LOW;
    sdio_obj.dma.handle_rx.Init.BurstSize           = 1;

    HAL_DMA_DeInit(&sdio_obj.dma.handle_rx);
    HAL_DMA_Init(&sdio_obj.dma.handle_rx);

    HAL_DMA_Start(&sdio_obj.dma.handle_rx, (uint32_t)src, (uint32_t)dst, BufferSize);
}

/**
  * @brief  This function get sdio clock.
  * @param  hw_sdio: sifli sdio hardware block
  * @retval PCLK2Freq
  */
static rt_uint32_t sifli_sdio_clock_get(SD_TypeDef *hw_sdio)
{
    UNUSED(hw_sdio);
    uint32_t sclk = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);
    LOG_D("SDIO source clock %d Hz \n", sclk);
    return sclk;//48 * 1000 * 1000; //HAL_RCC_GetPCLK2Freq();
}

static rt_err_t DMA_TxConfig(rt_uint32_t *src, rt_uint32_t *dst, int Size)
{
    SD_LowLevel_DMA_TxConfig((uint32_t *)src, (uint32_t *)dst, Size / 4);
    return RT_EOK;
}

static rt_err_t DMA_RxConfig(rt_uint32_t *src, rt_uint32_t *dst, int Size)
{
    SD_LowLevel_DMA_RxConfig((uint32_t *)src, (uint32_t *)dst, Size / 4);
    return RT_EOK;
}

#ifdef SOC_SF32LB52X
void SDMMC1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* Process All SDIO Interrupt Sources */
    rthw_sdio_irq_process(sdio_host);

    /* leave interrupt */
    rt_interrupt_leave();
}
#else
void SDMMC2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* Process All SDIO Interrupt Sources */
    rthw_sdio_irq_process(sdio_host);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifndef SD_BOOT
    #include "dfs_fs.h"
#endif

static int rt_sdio_get_offset(int part_id)
{
    struct dfs_partition part;
    int status;
    struct rt_mmcsd_req req;
    struct rt_mmcsd_data  data;
    struct rt_mmcsd_cmd   cmd;
    struct rt_mmcsd_cmd   stop;
    uint8_t *buf;
    int offset;

    memset(&req, 0, sizeof(struct rt_mmcsd_req));
    memset(&data, 0, sizeof(struct rt_mmcsd_data));
    memset(&cmd, 0, sizeof(struct rt_mmcsd_cmd));
    memset(&stop, 0, sizeof(struct rt_mmcsd_cmd));
    req.cmd = &cmd;
    req.data = &data;
    req.stop = NULL; //&stop;

    buf = rt_malloc(512);
    if (buf == NULL)
    {
        LOG_E("Malloc buf fail for SD read\n");
        return 0;
    }
    data.blks = 1;
    data.blksize = 512;
    data.buf = (rt_uint32_t *)buf;
    data.flags = DATA_DIR_READ;
    data.timeout_clks = 0;
    data.timeout_ns = 10000000;
    data.mrq = &req;
    data.stop = NULL;

    cmd.arg = 0;

    cmd.cmd_code = READ_SINGLE_BLOCK;
    req.stop = NULL;

    cmd.flags = RESP_SPI_R1 | RESP_R1 | CMD_ADTC;
    cmd.retries = 1;
    cmd.data = &data;
    cmd.mrq = &req;

    rthw_sdio_request(sdio_host, &req);
    if (cmd.err || data.err)
    {
        rt_free(buf);
        LOG_I("SD Read error with %d, %d\n", cmd.err, data.err);
        return 0;
    }

    status = dfs_filesystem_get_partition(&part, buf, part_id);
    if (status == RT_EOK)
    {
        LOG_I("Part %d offset %d\n", part_id, part.offset);
        offset = part.offset * 512;
    }
    else
    {
        offset = 0;
    }
    rt_free(buf);

    return offset;
}
int rt_sdio_enable_ahb(uint32_t enable_sd_ahb)
{
    HAL_StatusTypeDef res;
    struct rthw_sdio *sdio = sdio_host->private_data;
    SD_TypeDef *hw_sdio = sdio->sdio_des.hw_sdio;
    struct rt_mmcsd_card *card = sdio->host->card;
    uint8_t blk_mode;
    if (card == NULL || hw_sdio == NULL)
        return -1;

    if ((card->flags & CARD_FLAG_SDHC) || (card->flags & CARD_FLAG_SDXC)) // for card larger than 2GB, use block for read/write
        blk_mode = 1;
    else
        blk_mode = 0;

    LOG_D("capacity %d KB, flag 0x%x, type %d, block flag %d\n", card->card_capacity, card->flags, card->card_type, blk_mode);

    if (enable_sd_ahb)
    {
        sdio->part_offset = rt_sdio_get_offset(0);
        if (blk_mode)   // for sdhc, sdxc, offset is block based
        {
            HAL_SDMMC_SET_CAOFFSET(hw_sdio, sdio->part_offset / 512);
        }
        else
        {
            HAL_SDMMC_SET_CAOFFSET(hw_sdio, sdio->part_offset);
        }
        HAL_SDMMC_SWITCH_AHB(hw_sdio);

        HAL_SDMMC_SELECT_VERSION(hw_sdio,   blk_mode);

        HAL_SDMMC_ENABLE_AHB_MAP(hw_sdio, 1);

        HAL_SDMMC_VOID_FIFO(hw_sdio, 0);    // stop sd clock when fifo under flow, it should not work for AHB

        //HAL_SDMMC_CACHE_TO_EN(hw_sdio, 0);  // for some special card, their latency too large, close timeout function

        sdio->ahb_en = 1;
    }
    else
    {

        HAL_SDMMC_SWITCH_NORMAL(hw_sdio);

        //HAL_SDMMC_SELECT_VERSION(hw_sdio,   blk_mode);

        HAL_SDMMC_ENABLE_AHB_MAP(hw_sdio, 0);

        HAL_SDMMC_VOID_FIFO(hw_sdio, 1);

        sdio->ahb_en = 0;
    }

    return 0;
}

#ifdef RT_USING_PM

static int rt_sdio_freq_chg(const struct rt_device *device, uint8_t mode)
{
    struct rt_mmcsd_host *host = (struct rt_mmcsd_host *)device;
    struct rthw_sdio *sdio = host->private_data;

    // TODO: for PM_RUN_MODE_HIGH_SPEED/PM_RUN_MODE_NORMAL_SPEED open clock,
    //       for PM_RUN_MODE_MEDIUM_SPEED/PM_RUN_MODE_LOW_SPEED close clock?

    HAL_SDMMC_CLK_SET(sdio->sdio_des.hw_sdio, 1, 0);
    rthw_sdio_set_clk(host, sdio->cur_freq);
    return 0;
}

static const struct rt_device_pm_ops sdio_pm_op =
{
    .suspend = NULL,
    .resume = NULL,
    .frequency_change = rt_sdio_freq_chg,
};

static int sifli_sdio_pm_register(void)
{
    rt_device_t device = &rt_sdio_device;
    device->user_data = (void *)sdio_host;
    rt_pm_device_register(device, &sdio_pm_op);
    return 0;
}

static rt_err_t rt_sdio_control(struct rt_device *dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    uint8_t mode = (uint8_t)((uint32_t)args);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RESUME:
    {

        if (PM_SLEEP_MODE_STANDBY == mode)
            rt_hw_sdio_init();
        else
        {
            rthw_sdio_irq_update(sdio_host, 1);
            /* ready to change */
            mmcsd_change(sdio_host);
        }
        break;
    }
    case RT_DEVICE_CTRL_SUSPEND:
    {
        struct rthw_sdio *sdio = (struct rthw_sdio *)sdio_host->private_data;
        if ((PM_SLEEP_MODE_STANDBY == mode) && (sdio_host != NULL))
        {
            //rt_kprintf("SD suspend\n");
            mmcsd_host_lock(sdio_host);
            HAL_SDMMC_CLK_SET(sdio->sdio_des.hw_sdio, 1, 0);
            rt_mmcsd_blk_remove(sdio_host->card);
            rt_free(sdio_host->card);
            if (sdio)
                rt_free(sdio);
            sdio_host->card = RT_NULL;
            mmcsd_free_host(sdio_host);
            sdio_host = NULL;
        }
        else
            HAL_SDMMC_CLK_SET(sdio->sdio_des.hw_sdio, 1, 0);

        break;
    }
    default:
    {
        break;
    }
    }
    return result;
}

#ifdef RT_USING_DEVICE_OPS
static const rt_device_ops sdio_device_ops =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    rt_sdio_control,
};
#endif

static void rt_sdio_register_rt_device(void)
{
    rt_err_t err = RT_EOK;
    rt_device_t device;

    device = &rt_sdio_device;

    device->type        = RT_Device_Class_Miscellaneous;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &sdio_device_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_sdio_control;
#endif
    device->user_data = (void *)sdio_host;

    err = rt_device_register(device, "sdio0", RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);
    RT_ASSERT(RT_EOK == err);

}

#endif  /* RT_USING_PM */

int rt_hw_sdio_init(void)
{
    struct sifli_sdio_des sdio_des;
#if 0       // FIX ME: With read HW setting
    {
        rt_uint32_t tmpreg = 0x00U;
        SET_BIT(RCC->AHB1ENR, sdio_config.dma_rx.dma_rcc);
        /* Delay after an RCC peripheral clock enabling */
        tmpreg = READ_BIT(RCC->AHB1ENR, sdio_config.dma_rx.dma_rcc);
        UNUSED(tmpreg); /* To avoid compiler warnings */
    }
#endif

    sdio_des.clk_get = sifli_sdio_clock_get;
    sdio_des.hw_sdio = (SD_TypeDef *)SDCARD_INSTANCE;
    sdio_des.rxconfig = DMA_RxConfig;
    sdio_des.txconfig = DMA_TxConfig;

    HAL_SDMMC_INIT(sdio_des.hw_sdio);

    sdio_host = sdio_host_create(&sdio_des);
    if (sdio_host == RT_NULL)
    {
        LOG_E("sdio_host create fail");
        return -1;
    }

    HAL_NVIC_SetPriority(SDIO_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
#ifdef RT_USING_PM
    //sifli_sdio_pm_register();
    rt_sdio_register_rt_device();
#endif
    //HAL_SD_MspInit(&hsd);
#ifdef SDIO_USING_DMA
    LOG_I("SDIO USING DMA MODE !\n");
#else
    LOG_I("SDIO USING POLLING MODE !\n");
#endif

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_sdio_init);

//#define DRV_SDIO_TEST
#ifdef DRV_SDIO_TEST
int cmd_sdcard(int argc, char *argv[])
{
    if (strcmp(argv[1], "-ahb") == 0)
    {
        rt_device_t dev = rt_device_find("sd0");    // make sure sd card exist
        {
            uint32_t ahb = atoi(argv[2]);
            int res = rt_sdio_enable_ahb(ahb);
            rt_kprintf("Enable SD AHB (%d) with res %d\n", ahb, res);
        }
        return 0;
    }
    if ((strcmp(argv[1], "-r") && strcmp(argv[1], "-w")) || (argc < 4))
    {
        LOG_I("Invalid parameter\n");
        LOG_I("-r for read, -w for write; with addr length (and value if needed)\n");
        return 2;
    }
    rt_uint32_t *buf = (rt_uint32_t *)rt_malloc(1024 * 4);
    if (buf == NULL)
    {
        LOG_I("Malloc 4KB fail\n");
        return 3;
    }
    memset(buf, 0, 4096);
    rt_uint32_t addr = atoi(argv[2]);
    rt_uint32_t len = atoi(argv[3]);
    rt_uint32_t value = 0;
    if (argc >= 5)
        value = atoi(argv[4]);
    if (len > 4096)
    {
        LOG_I("lenght too large, change to 4KB\n");
        len = 4096;
    }
    else if (len < 512)
    {
        LOG_I("lenght too small, change to 512\n");
        len = 512;
    }

    rt_device_t dev = rt_device_find("sd0");    // get block device
    if (dev)
    {
        if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
        {
            LOG_I("Open device sd0 fail\n");
            return 1;
        }
        if ((dev->read == NULL) || (dev->write == NULL))
        {
            LOG_I("SD0 device read/write function empty!\n");
            return 1;
        }

        int i, res, blk;
        blk = len >> 9;
        if (strcmp(argv[1], "-r") == 0)
        {
            res = rt_device_read(dev, addr, (void *)buf, blk);
            if (res > 0)
            {
                LOG_I("Read Data :\n");
                for (i = 0; i < len / 4; i++)
                {
                    LOG_RAW(" 0x%08x ", *(buf + i));
                    if ((i & 7) == 7)
                        LOG_RAW("\n");
                }
            }
            else
            {
                LOG_I("read data fail %d\n", res);
            }
        }
        if (strcmp(argv[1], "-w") == 0)
        {
            // initial write data
            if (value == 0)
            {
                for (i = 0; i < len / 4; i++)
                    *(buf + i) = (i << 16) | (i + 1);
            }
            else
            {
                for (i = 0; i < len / 4; i++)
                    *(buf + i) = value;
            }
            res = rt_device_write(dev, addr, buf, blk);
            if (res > 0)
                LOG_I("write done %d\n", res);
            else
                LOG_I("write fail %d\n", res);

        }

        rt_device_close(dev);
    }
    else
    {
        LOG_I("find device sd0 fail\n");
    }

    rt_free(buf);

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sdcard, __cmd_sdcard, Test hw sdcard);

#endif  // DRV_SDIO_TEST 

#endif

/// @} drv_sdio
/// @} bsp_driver
/// @} file

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
