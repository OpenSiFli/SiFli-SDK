/**
  ******************************************************************************
  * @file   drv_sdhci.c
  * @author Sifli software development team
  * @brief SD/MMC HCI Controller BSP driver
  This driver is validated by using MSH command 'date'.
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

#if defined (BSP_USING_SDHCI) || defined(_SIFLI_DOXYGEN_)

#include "drv_sdhci.h"
#include "drv_config.h"
#include "drv_io.h"

//extern int dbg_message(uint8_t debug_level, const char *fmt, ...);
//#define DRIVER_NAME "sdhci"

//#define DRV_DEBUG
#define LOG_TAG                "drv.sdhci"
#include <drv_log.h>
#include <stdlib.h>

/** TODO: lite mode only for controller save/restor registers when sleep, CARD SHOULD NOT power down.
**      : full initial sequence for sd if lite=0.
**      : move it to menuconfig if needed
**/
#define SDHCI_SLEEP_LITE_MODE       (0)

static struct sdhci_host sdhci_ctx[2];
//static uint32_t sdhci_irq_flag = 0;

#ifdef RT_USING_PM
    static struct rt_device rt_sdhci_device[2];
#endif /* RT_USING_PM */


static void sdhci_prepare_data(struct sdhci_host *host, struct rt_mmcsd_data *data);
static void sdhci_finish_data(struct sdhci_host *host);

static void sdhci_send_command(struct sdhci_host *host, struct rt_mmcsd_cmd *cmd);
static void sdhci_finish_command(struct sdhci_host *host);
static int sdhci_wait_completed(struct sdhci_host *sdio, int flag);
static void sdhci_cmd_irq(struct sdhci_host *host, uint32_t intmask);
static void sdhci_data_irq(struct sdhci_host *host, uint32_t intmask);
static void sdhci_set_ios(struct rt_mmcsd_host *mmc, struct rt_mmcsd_io_cfg *ios);
static void sdhci_set_ddr(struct sdhci_host *host, unsigned int ddr);

#ifdef SDIO_PM_MODE
    static int sdmmc_pm_resume_init(uint8_t id);
    uint32_t sd_send_cmd(uint8_t cmd_idx, uint32_t cmd_arg);
    uint32_t sd_clr_status();
    SDHCI_HandleTypeDef sdhci_handle;
#endif

int rt_hw_sdmmc_init(void);
int rt_sdhci_init_instance(uint8_t id);

struct rt_sdhci_configuration
{
    uint32_t max_freq;
    uint32_t irqn;
    uint8_t dma_mode;
    uint8_t card_mode;
};

static struct rt_sdhci_configuration rt_sdhci_cfg_def[2] =
{
#ifdef BSP_USING_SDHCI1
    {
        SD_MAX_FREQ,       //uint32_t max_freq;
        SDMMC1_IRQn,       //uint32_t irqn;
        SD_DMA_MODE,    //uint8_t dma_mode;
        SDIO_CARD_MODE   //uint8_t card_mode;
    },
#else
    {
        0,       //uint32_t max_freq;
        0,       //uint32_t irqn;
        0,    //uint8_t dma_mode;
        0   //uint8_t card_mode;
    },
#endif  // BSP_USING_SDHCI1
#ifdef BSP_USING_SDHCI2
    {
        SD2_MAX_FREQ,       //uint32_t max_freq;
        SDMMC2_IRQn,       //uint32_t irqn;
        SD2_DMA_MODE,    //uint8_t dma_mode;
        SDIO2_CARD_MODE   //uint8_t card_mode;
    },
#else
    {
        0,       //uint32_t max_freq;
        0,       //uint32_t irqn;
        0,    //uint8_t dma_mode;
        0   //uint8_t card_mode;
    },
#endif  // BSP_USING_SDHCI2
};

#ifdef RT_USING_PM
static void sdhci_pm_set_ddr(uint8_t id)
{
    if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_BUS_WIDTH_4)
    {
        sdhci_set_ddr(&sdhci_ctx[id], 0);
    }
    else if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_BUS_WIDTH_8)
    {
        sdhci_set_ddr(&sdhci_ctx[id], 0);
    }
    else if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_DDR_BUS_WIDTH_4)
    {
        sdhci_set_ddr(&sdhci_ctx[id], 1);
    }
    else if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_DDR_BUS_WIDTH_8)
    {
        sdhci_set_ddr(&sdhci_ctx[id], 1);
    }
}

#define _DUMP_REG_DEBUG         (0)
#define _SDHCI_DUMP_RCNT        (23)
static uint32_t sdhci_reg_arr[_SDHCI_DUMP_RCNT];
static void dump_sdhci_reg(int id)
{
    int i;
    uint32_t *sdhci_base_reg;
    if (id == 0)
    {
        sdhci_base_reg = (uint32_t *)SDMMC1_BASE;
    }
    else
    {
        sdhci_base_reg = (uint32_t *)SDMMC2_BASE;
    }
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

static void recov_sdhci_reg(int id)
{
    int i;
    uint32_t *sdhci_base_reg;
    if (id == 0)
    {
        sdhci_base_reg = (uint32_t *)SDMMC1_BASE;
    }
    else
    {
        sdhci_base_reg = (uint32_t *)SDMMC2_BASE;
    }
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
    sdhci_pm_set_ddr(id);//this is bug only 56X ddr mode
}
#endif

static void sdhci_init(struct sdhci_host *host, int soft)
{
    LOG_D("SDDEBUG: sdhci.c sdhci_init \n");

    hal_sdhci_init(&host->handle, soft);
    if (soft)
    {
        /* force clock reconfiguration */
        host->clock = 0;
        //printk("Test Soft\n");
        sdhci_set_ios(host->mmc, &host->mmc->io_cfg);
    }
}

static void sdhci_reinit(struct sdhci_host *host)
{

    hal_sdhci_init(&host->handle, 0);
    //sdhci_enable_card_detection(host);
}

static uint8_t sdhci_calc_timeout(struct sdhci_host *host, struct rt_mmcsd_data *data)
{
    uint8_t count;
    unsigned int target_timeout, current_timeout;

    /*
     * If the host controller provides us with an incorrect timeout
     * value, just skip the check and use 0xE.  The hardware may take
     * longer to time out, but that's much better than having a too-short
     * timeout value.
     */
    //if (host->quirks & SDHCI_QUIRK_BROKEN_TIMEOUT_VAL)
    //    return 0xE;

    /* timeout in us */
    target_timeout = data->timeout_ns / 1000 +
                     data->timeout_clks / host->clock;

    //if (host->quirks & SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK)
    //    host->timeout_clk = host->clock / 1000;

    /*
     * Figure out needed cycles.
     * We do this in steps in order to fit inside a 32 bit int.
     * The first step is the minimum timeout, which will have a
     * minimum resolution of 6 bits:
     * (1) 2^13*1000 > 2^22,
     * (2) host->timeout_clk < 2^16
     *     =>
     *     (1) / (2) > 2^6
     */
    count = 0;
    current_timeout = (1 << 13) * 1000 / host->timeout_clk;
    while (current_timeout < target_timeout)
    {
        count++;
        current_timeout <<= 1;
        if (count >= 0xF)
            break;
    }

    if (count >= 0xF)
    {
        count = 0xE;
    }

    return count;
}


static void sdhci_prepare_data(struct sdhci_host *host, struct rt_mmcsd_data *data)
{
    uint8_t count;
    uint8_t ctrl;
    int ret;

    //ASSERT(host->data);
    //LOG_D("SDDEBUG: sdhci.c sdhci_prepare_data\n");
    host->data = data;
    if (data == NULL)
        return;

    /* Sanity checks */
    //BUG_ON(data->blksz * data->blocks > 524288);
    //BUG_ON(data->blksz > host->mmc->max_blk_size);
    //BUG_ON(data->blocks > 65535);

    //host->data = data;
    host->data_early = 0;
    //LOG_D("SDDEBUG:data->blksz*data->blocks:%u \n", data->blksize * data->blks);
    LOG_D("SDDEBUG:hosemaxblksize:%u\n", host->mmc->max_blk_size);


    count = sdhci_calc_timeout(host, data);
    count = 0xE;
    hal_sdhci_set_timeout(&host->handle, count);
    if (host->handle.Init.flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA))
        host->handle.Init.flags |= SDHCI_REQ_USE_DMA;


    if (host->handle.Init.flags & SDHCI_REQ_USE_DMA)
    {
        host->org_buf = NULL;
        host->cache_buf = NULL;
        if (data->flags & DATA_DIR_WRITE)
        {
            if (IS_BUF_ACCROSS_512K_BOUNDARY((uint32_t)data->buf, (uint32_t)data->blksize * data->blks))
            {
                // buffer accross 512kb boundary, need copy to local buffer to avoid dma limited
                host->cache_buf = malloc((uint32_t)data->blksize * data->blks);
                if (host->cache_buf == NULL || IS_BUF_ACCROSS_512K_BOUNDARY((uint32_t)host->cache_buf, (uint32_t)data->blksize * data->blks))
                {
                    // alloc fail or new buffer also accross boundary?
                    LOG_E("Alloc new SD buffer can not used %p, %d\n", host->cache_buf, (uint32_t)data->blksize * data->blks);
                    RT_ASSERT(0);
                }
                memcpy(host->cache_buf, data->buf, (uint32_t)data->blksize * data->blks);
                host->org_buf = data->buf;
                data->buf = host->cache_buf;
            }
            mpu_dcache_clean(data->buf, (uint32_t)data->blksize * data->blks);
        }
        else
        {
            if (IS_BUF_ACCROSS_512K_BOUNDARY((uint32_t)data->buf, (uint32_t)data->blksize * data->blks))
            {
                // buffer accross 512kb boundary, need copy to local buffer to avoid dma limited
                host->cache_buf = malloc((uint32_t)data->blksize * data->blks);
                if (host->cache_buf == NULL || IS_BUF_ACCROSS_512K_BOUNDARY((uint32_t)host->cache_buf, (uint32_t)data->blksize * data->blks))
                {
                    // alloc fail or new buffer also accross boundary?
                    LOG_E("Alloc new SD buffer can not used %p, %d\n", host->cache_buf, (uint32_t)data->blksize * data->blks);
                    RT_ASSERT(0);
                }
                host->org_buf = data->buf;
                data->buf = host->cache_buf;
            }
            if (IS_DCACHED_RAM(data->buf))
                SCB_InvalidateDCache_by_Addr(data->buf, (uint32_t)data->blksize * data->blks);
        }
        if (host->handle.Init.flags & SDHCI_USE_ADMA)
        {
            LOG_D("sdhci_prepare_data ADMA, blk size %d\n", data->blksize);
            ret = hal_sdhci_adma_table_pre(&host->handle, (uint8_t *)data->buf, (uint32_t)data->blksize * data->blks);
            if (ret)
            {
                /*
                 * This only happens when someone fed
                 * us an invalid request.
                 */
                //WARN_ON(1);
                host->handle.Init.flags &= ~SDHCI_REQ_USE_DMA;
            }
            else
            {
                //sdhci_writel(host, host->adma_addr,
                //             SDHCI_ADMA_ADDRESS);
                hal_sdhic_set_adma_addr(&host->handle, host->adma_addr);
                LOG_D("ADMA 0x%0x, addr 0x%08x, len %d\n", host->adma_addr, *((uint32_t *)(host->adma_desc + 4)), *((uint16_t *)(host->adma_desc + 2)));
            }
        }
        else
        {
            LOG_D("sdhci_prepare_data SDMA");
            //sdhci_writel(host, (uint32_t)data->buf,   SDHCI_DMA_ADDRESS);
            hal_sdhci_set_dma_addr(&host->handle, (uint32_t)data->buf);
        }
    }

    /*
     * Always adjust the DMA selection as some controllers
     * (e.g. JMicron) can't do PIO properly when the selection
     * is ADMA.
     */
#if 0    //  ??? sdma NOT work anymore, always ADMA?
    if (host->version >= SDHCI_SPEC_200)
    {
        hal_sdhci_set_dma_mode(&host->handle, host->handle.Init.flags);
    }
#endif
    if (!(host->handle.Init.flags & SDHCI_REQ_USE_DMA))
    {
        int flags;

        //flags = SG_MITER_ATOMIC;
        //if (host->data->flags & DATA_DIR_READ)
        //    flags |= SG_MITER_TO_SG;
        //else
        //    flags |= SG_MITER_FROM_SG;
        //sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
        host->handle.Init.blocks = data->blks;
    }

    hal_sdhci_set_transfer_irqs(&host->handle);
    //LOG_D("data.no of blocks =%d\n", data->blks);
    /* We do not handle DMA boundaries, so set it to max (512 KiB) */
    //sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, data->blksize), SDHCI_BLOCK_SIZE);
    //sdhci_writew(host, data->blks, SDHCI_BLOCK_COUNT);
    hal_sdhci_set_blk(&host->handle, data->blksize, data->blks);
    LOG_D("SDDEBUG: Number of Blocks: 0x%x Block Size:0x%x\n",  data->blks, SDHCI_MAKE_BLKSZ(7, data->blksize));
}

static void sdhci_set_transfer_mode(struct sdhci_host *host,
                                    struct rt_mmcsd_data *data)
{
    uint16_t mode = 0;

    if (data == NULL)
        return;

    //ASSERT(!host->data);
    //if(data->stop)
    //mode = 1<<2;
    //check the version it supports and enable Autocmd23 if it is SDXC card
    //if (s == 1)
    //{
    //    mode = 1 << 3;
    //    sdhci_writew(host, data->blks, 0x00);//AUTOCMD 23 for 0x00
    //}
    mode = SDHCI_TRNS_BLK_CNT_EN | mode;
    if (data->blks > 1)
    {
        mode |= SDHCI_TRNS_MULTI;
        //mode |= (SDHCI_TRNS_MULTI | SDHCI_TRNS_ACMD12);
        if (SDHCI_SDIO != host->usr_cfg.sdio_mode)
        {
            mode |= SDHCI_TRNS_ACMD12;
        }
    }
    if (data->flags & DATA_DIR_READ)
        mode |= SDHCI_TRNS_READ;
    //if (host->handle.Init.flags & SDHCI_REQ_USE_DMA)
    //    mode |= SDHCI_TRNS_DMA;
//  printk("transfer mode value = %x\n",mode);
    //sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
    hal_sdhci_set_transfer_mode(&host->handle, mode);
    LOG_D("SDDEBUG:Transfer mode: 0x%x\n", mode);
}

static void sdhci_adma_table_post(struct sdhci_host *host,
                                  struct rt_mmcsd_data *data)
{
}

static void sdhci_finish_data(struct sdhci_host *host)
{
    struct rt_mmcsd_data *data;
    //int err = 0;

    //ASSERT(!host->data);

    LOG_D("SDDEBUG: sdhci.c sdhci_finish_data \n");

    data = host->data;
    host->data = NULL;

    if (host->handle.Init.flags & SDHCI_REQ_USE_DMA)
    {
        if (host->handle.Init.flags & SDHCI_USE_ADMA)
            sdhci_adma_table_post(host, data);

        if (host->cache_buf != NULL && host->org_buf != NULL)
        {
            // Use local buffer replaced
            if (data->flags & DATA_DIR_WRITE)
                data->buf = host->org_buf;
            else
            {
                memcpy(host->org_buf, host->cache_buf, (uint32_t)data->blksize * data->blks);
                data->buf = host->org_buf;
            }
            free(host->cache_buf);
            host->cache_buf = NULL;
            host->org_buf = NULL;
        }
    }
#if 0
    else // for PIO
    {
        int i;
        int cnt = data->blks * data->blksize;
        uint32_t *buf = (uint32_t *)data->buf;
        if (data->flags & DATA_DIR_READ)
        {
            for (i = 0; i < cnt / 4; i++)
                *(buf++) = sdhci_readl(host, SDHCI_BUFFER);
            LOG_D("Received %d data\n", cnt);
        }
        else if (data->flags & DATA_DIR_WRITE)
        {
            for (i = 0; i < cnt / 4; i++)
                sdhci_writel(host, *(buf++), SDHCI_BUFFER);

            // wait for data complete
            LOG_D("Wait write \n");
            sdhci_wait_completed(host, 0, NULL);
            LOG_I("Wait write done\n");
        }
    }
#endif
    /*
     * The specification states that the block count register must
     * be updated, but it does not specify at what point in the
     * data flow. That makes the register entirely useless to read
     * back so we have to assume that nothing made it to the card
     * in the event of an error.
     */
    if (data->err)
        data->bytes_xfered = 0;
    else
        data->bytes_xfered = data->blksize * data->blks;
    // dbg_message(SDIO_USR,"SDDEBUG: blocks: %x bytes_xfered: %x\n",data->blocks,data->bytes_xfered);
    if (data->stop)
    {
        /*
         * The controller needs a reset of internal state machines
         * upon error conditions.
         */
        if (data->err)
        {
            hal_sdhci_reset(&host->handle, SDHCI_RESET_CMD);
            hal_sdhci_reset(&host->handle, SDHCI_RESET_DATA);
        }

        //sdhci_send_command(host, data->stop);
        //if(SDXC)
        //{
        //call retuning;
        //  retuning(host);
        //}


    }
    //else
    //tasklet_schedule(&host->finish_tasklet);
}

static void sdhci_send_command(struct sdhci_host *host, struct rt_mmcsd_cmd *cmd)
{
    int flags;
    uint32_t mask, status;
    unsigned long timeout;
    SDHCI_CmdArgTypeDef mcmd;
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif

    //ASSERT(host->cmd);
    LOG_D("SDDEBUG: sdhci.c sdhci_send_command \n");
    //printk("send command \n");
    /* Wait max 10 ms */
    timeout = 10;

    mask = SDHCI_CMD_INHIBIT;
    if ((cmd->data != NULL) || (cmd->flags == RESP_R1B))
        mask |= SDHCI_DATA_INHIBIT;

    /* We shouldn't wait for data inihibit for stop commands, even
       though they might use busy signaling */
    if (host->mrq->data && (cmd == host->mrq->data->stop))
        mask &= ~SDHCI_DATA_INHIBIT;
    if (1)
    {
        volatile uint32_t val;
        while ((val = hal_sdhci_get_present_state(&host->handle)) & mask)
        {
            if (timeout == 0)
            {
                LOG_E("wait busy timout, stat = 0x%x, mask = 0x%x, arg 0x%x\n", val, mask, cmd->arg);
                //printk("value of mask in present state register = %x\n",mask);
                //printk(KERN_ERR "%s: Controller never released "
                //      "inhibit bit(s).\n", mmc_hostname(host->mmc));
                cmd->err = -EIO;
                //tasklet_schedule(&host->finish_tasklet);
                return;
            }
            timeout--;
            rt_thread_delay(1);
        }
    }
    //mod_timer(&host->timer, jiffies + 12 * HZ);

    host->cmd = cmd;

    sdhci_prepare_data(host, cmd->data);

    //sdhci_writel(host, cmd->arg, SDHCI_ARGUMENT);
    //  dbg_message(SDIO_WRITE,"SDDEBUG: PID:0x%p Argument:0x%x\n",current,cmd->arg);

    sdhci_set_transfer_mode(host, cmd->data);


    //if (!(cmd->flags == RESP_R1))
    //    flags = SDHCI_CMD_RESP_NONE;
    //else if (cmd->flags == RESP_R2)
    //    flags = SDHCI_CMD_RESP_LONG;
    //else if (cmd->flags == RESP_R1B)
    //    flags = SDHCI_CMD_RESP_SHORT_BUSY;
    //else
    //    flags = SDHCI_CMD_RESP_SHORT;
    if (resp_type(cmd) == RESP_NONE)
        flags = SDHCI_CMD_RESP_NONE;
    else if (resp_type(cmd) == RESP_R2)
        flags = SDHCI_CMD_RESP_LONG;
    else if (resp_type(cmd) == RESP_R1B)
        flags = SDHCI_CMD_RESP_SHORT_BUSY;
    else
        flags = SDHCI_CMD_RESP_SHORT;


    if ((resp_type(cmd) != RESP_R3) && (resp_type(cmd) != RESP_R4) && (resp_type(cmd) != RESP_NONE))
        flags |= SDHCI_CMD_CRC;
    if ((resp_type(cmd) != RESP_R2) && (resp_type(cmd) != RESP_R3) && (resp_type(cmd) != RESP_R4) && (resp_type(cmd) != RESP_NONE))
        flags |= SDHCI_CMD_INDEX;
    if (cmd->data)
        flags |= SDHCI_CMD_DATA;
    LOG_D("cmd flag = 0x%x\n", flags);

//  do_gettimeofday(&stv);
//  printk("Start time sec %ld microseconds %ld\n",stv.tv_sec,stv.tv_usec);
    //  printk("written before the command register\n");
//  printk("%x",SDHCI_MAKE_CMD(cmd->opcode, flags));
    struct rt_mmcsd_data *data = cmd->data;

    LOG_D("CMD:%s(%d) ARG:0x%08x RES:%s%s%s%s%s%s%s%s%s rw:%c len:%d blksize:%d\n",
          sd_cmd_name(cmd->cmd_code),
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
          data ? data->blksize : 0
         );
    //LOG_D("Before send cmd 0x%08x\n",sdhci_readl(host, SDHCI_INT_STATUS));

    hal_sdhci_clear_int(&host->handle, 0xffffffff);
    host->irq_flag = 0; // clear irq flag before command start, todo: add mutex or signal to protect
    rt_event_control(&host->event, RT_IPC_CMD_RESET, NULL);
    //sdhci_writew(host, SDHCI_MAKE_CMD(cmd->cmd_code, flags), SDHCI_COMMAND);
    mcmd.Arg = (uint32_t)cmd->arg;
    mcmd.CmdIndex = (uint32_t)cmd->cmd_code;
    mcmd.RespType = flags;

    hal_sdhci_send_command(&host->handle, &mcmd);
    //  if(cmd->opcode == 19)
    //  {
    //  sdxc=1;
    //if(cmd->opcode==18)
    //  rd_flag=1;
    //  }

    int res = sdhci_wait_completed(host, flags);
    if (res == 0)
    {
        LOG_D("Wait cmd %d done\n", cmd->cmd_code);
    }
    else
    {
        LOG_D("Wait cmd %d fail with %d\n", cmd->cmd_code, res);
    }
#ifdef RT_USING_PM
    rt_pm_hw_device_stop();
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif
}

static void sdhci_finish_command(struct sdhci_host *host)
{
    int i;

    RT_ASSERT(host->cmd);

    //dbg_message(SDIO_DBG,"SDDEBUG: sdhci.c sdhci_finish_command \n");
    if (resp_type(host->cmd) != RESP_NONE)
    {
        if (resp_type(host->cmd) == RESP_R2)  // ???
        {
#if 1
            uint32_t buf[4];
            for (i = 0; i < 4; i++)
            {
                buf[i] = hal_sdhci_get_response(&host->handle, i);
                LOG_D("REP%d: 0x%x", i, buf[i]);
            }

            host->cmd->resp[0] = (buf[3] << 8) | (buf[2] >> 24);
            host->cmd->resp[1] = (buf[2] << 8) | (buf[1] >> 24);
            host->cmd->resp[2] = (buf[1] << 8) | (buf[0] >> 24);
            host->cmd->resp[3] = (buf[0] << 8);
#else
            /* CRC is stripped so we need to do some shifting. */
            for (i = 0; i < 4; i++)
            {
                host->cmd->resp[i] = sdhci_readl(host,
                                                 SDHCI_RESPONSE + (3 - i) * 4) << 8;
                if (i != 3)
                    host->cmd->resp[i] |=
                        sdhci_readb(host,
                                    SDHCI_RESPONSE + (3 - i) * 4 - 1);
            }
#endif
        }
        else
        {

            host->cmd->resp[0] = hal_sdhci_get_response(&host->handle, 0);
            //printk("host cmd resp[0] = %x\n",host->cmd->resp[0]);

        }
    }

    host->cmd->err = 0;

    if (host->data && host->data_early)
        sdhci_finish_data(host);

    //if (!host->cmd->data)
    //  tasklet_schedule(&host->finish_tasklet);

    host->cmd = NULL;
}

static void sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
    int div, div1;
    //  int err;
    int version;
    uint16_t clk = 0;
    unsigned long timeout;
    LOG_D("SDDEBUG: sdhci.c sdhci_set_clock %d\n", clock);

    if (clock == host->clock)
        return;

    // TODO: for FPGA, it can not work fater than 12mhz
    //if (clock > 12000000)
    //    clock = 12000000;
    if (host->cfg_flag & SDHCI_USER_CONFIG_CLK) // set clock to cal divider
    {
        if (clock > host->usr_cfg.max_clk)
            clock = host->usr_cfg.max_clk;
    }

    if (clock > host->max_clk)
    {
        LOG_I("Input clock %d larger than max support clock %d\n", clock, host->max_clk);
        clock = host->max_clk;
    }

    LOG_I("sdhci_set_clock2 %d\n", clock);

    if (host->ops->set_clock)
    {
        host->ops->set_clock(host, clock);
    }

    hal_sdhci_set_clk(&host->handle, clock, host->max_clk);

    host->clock = clock;
}

void sdhci_update_clk(void)
{
#ifdef BSP_USING_SDHCI1
    LOG_D("sdhci_ctx[0]->clock=%d \n", sdhci_ctx[0].clock);
    sdhci_set_clock(&sdhci_ctx[0], sdhci_ctx[0].clock);
#endif

#ifdef BSP_USING_SDHCI2
    LOG_D("SDDEBUG: sdhci.c sdhci_request \n");
    sdhci_set_clock(&sdhci_ctx[1], sdhci_ctx[1].clock);
#endif
}

static void sdhci_set_ddr(struct sdhci_host *host, unsigned int ddr)
{
//    if (host->usr_cfg.ddr_mode == ddr)
//        return;

    LOG_I("Enable DDR mode %d\n", ddr);
    host->usr_cfg.ddr_mode = ddr;
    hal_sdhci_set_ddr(&host->handle, ddr);
}

/*****************************************************************************\
 *                                                                           *
 * MMC callbacks                                                             *
 *                                                                           *
 \*****************************************************************************/

static void sdhci_request(struct rt_mmcsd_host *mmc, struct rt_mmcsd_req *mrq)
{
    struct sdhci_host *host;
    bool present;
    unsigned long flags;

    host = (struct sdhci_host *)(mmc->private_data);

    LOG_D("SDDEBUG: sdhci.c sdhci_request \n");
    //spin_lock_irqsave(&host->lock, flags);

    //ASSERT(host->mrq != NULL);


    host->mrq = mrq;

    /* If polling, assume that the card is always present. */

    present = hal_sdhci_get_present_state(&host->handle) & SDHCI_CARD_PRESENT;
    //LOG_D("present %d state register value = %x \n", present, sdhci_readl(host, SDHCI_PRESENT_STATE));

    present = true; // card detect not work, make it always true

    if (!present || host->handle.Init.flags & SDHCI_DEVICE_DEAD)
    {
        host->mrq->cmd->err = -ENODEV;
        LOG_D("SDHCI Busy\n");
        //tasklet_schedule(&host->finish_tasklet);
    }
    else
    {
        LOG_D("Send cmd %d\n", mrq->cmd->cmd_code);
        sdhci_send_command(host, mrq->cmd);
    }


    mmcsd_req_complete(mmc);
    //mmiowb();
    //spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_set_ios(struct rt_mmcsd_host *mmc, struct rt_mmcsd_io_cfg *ios)
{
    struct sdhci_host *host;
    unsigned long flags;
    uint8_t ctrl;
    LOG_D("SDDEBUG: sdhci_set_ios : busmode %d, width %d, clk %d, pwer_mode %d, vdd %d\n",
          ios->bus_mode, ios->bus_width, ios->clock, ios->power_mode, ios->vdd);
    host = (struct sdhci_host *)(mmc->private_data);

    //spin_lock_irqsave(&host->lock, flags);

    if (host->handle.Init.flags & SDHCI_DEVICE_DEAD)
        return ; //goto out;

    /*
     * Reset the chip on each power off.
     * Should clear out any weird states.
     */
    if (ios->power_mode == MMCSD_POWER_OFF)
    {
        hal_sdhci_clear_mask_irqs(&host->handle, 0);
        sdhci_reinit(host);
    }

    sdhci_set_clock(host, ios->clock);

    if (ios->power_mode == MMCSD_POWER_OFF)
        hal_sdhci_set_power(&host->handle, -1);
    else
    {
//#if (SDIO_CARD_MODE == 1)    // emmc
        if (SDHCI_SDEMMC == host->usr_cfg.sdio_mode)
        {
            if (ios->clock > 400000) // identify finish, push pull
                hal_sdhci_set_power(&host->handle, ios->vdd);
            else // identify , open drain,
                hal_sdhci_set_power(&host->handle, ios->vdd | 0x100);
        }
//#else
        else
        {
            hal_sdhci_set_power(&host->handle, ios->vdd);   // sdcard alway push pull
        }
//#endif
    }
    //printk("c HOST CONTROL  register = %x\n",ctrl);
    //ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
    ctrl = 0;

    if (ios->bus_width == MMCSD_BUS_WIDTH_4)
    {
        ctrl = 4;
        sdhci_set_ddr(host, 0);
        LOG_I("Set to 4 line mode\n");
    }
    else if (ios->bus_width == MMCSD_BUS_WIDTH_8)
    {
        ctrl = 8;
        sdhci_set_ddr(host, 0);
        LOG_I("Set to 8 line mode\n");
    }
    else if (ios->bus_width == MMCSD_DDR_BUS_WIDTH_4)
    {
        ctrl = 4;
        sdhci_set_ddr(host, 1);
        LOG_I("Set to 4 line DDR mode\n");
    }
    else if (ios->bus_width == MMCSD_DDR_BUS_WIDTH_8)
    {
        ctrl = 8;
        sdhci_set_ddr(host, 1);
        LOG_I("Set to 8 line DDR mode\n");
    }
    else
        ctrl = 1;

    //if (ios->timing == MMC_TIMING_SD_HS)
    //  ctrl |= SDHCI_CTRL_HISPD;
    //else
    //ctrl &= ~SDHCI_CTRL_HISPD;
    //printk("a HOST CONTROL 1 REGISTER = %x\n",ctrl);
    //sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
    hal_sdhci_set_bus_width(&host->handle, ctrl);
    LOG_D("BITWIDTH 4bit mode %d\n", ctrl);
}

static long sdhci_get_ro(struct rt_mmcsd_host *mmc)
{
    struct sdhci_host *host;
    unsigned long flags;
    int present;

    host = (struct sdhci_host *)(mmc->private_data);
    LOG_D("SDDEBUG: sdhci.c sdhci_get_ro \n");
    //spin_lock_irqsave(&host->lock, flags);

    present = hal_sdhci_get_present_state(&host->handle);

    return !(present & SDHCI_WRITE_PROTECT);
}

static void sdhci_enable_sdio_irq(struct rt_mmcsd_host *mmc, long enable)
{
    struct sdhci_host *host;
    unsigned long flags;

    host = (struct sdhci_host *)(mmc->private_data);

    //spin_lock_irqsave(&host->lock, flags);

    //if (host->handle.Init.flags & SDHCI_DEVICE_DEAD)
    //    return ; //goto out;

    if (enable)
        hal_sdhci_unmask_irqs(&host->handle, SDHCI_INT_CARD_INT);
    else
        hal_sdhci_mask_irqs(&host->handle, SDHCI_INT_CARD_INT);
}

static const struct rt_mmcsd_host_ops sdhci_host_ops =
{
    .request    = sdhci_request,
    .set_iocfg  = sdhci_set_ios,
    .get_card_status    = sdhci_get_ro,
    .enable_sdio_irq = sdhci_enable_sdio_irq,
};

/**
  * @brief  This function get sdio clock.
  * @param  hw_sdio: sifli sdio hardware block
  * @retval PCLK2Freq
  */
static unsigned int sifli_sdio_clock_get(struct sdhci_host *host)
{
    uint32_t freq = 48000000;
    // this define max input clock , before internal divider!
#ifdef FPGA
    freq = 48000000;
#else
#ifdef  SF32LB55X
    freq = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);//48 * 1000 * 1000; //HAL_RCC_GetPCLK2Freq();
#else
    int src = HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SDMMC);
    switch (src)
    {
    case RCC_CLK_MPI_SD_SYSCLK:
        freq = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
        break;
    case RCC_CLK_MPI_SD_DLL2:
        freq = HAL_RCC_HCPU_GetDLL2Freq();
        break;
    case RCC_CLK_MPI_SD_DLL3:
        freq = HAL_RCC_HCPU_GetDLL3Freq();
        break;
    default:
        rt_kprintf("Invalid SDMMC clock source\n");
        RT_ASSERT(0);
        break;
    }
#endif  //SF32LB55X
#endif  // fpga
    //if(host->cfg_flag & SDHCI_USER_CONFIG_CLK)
    //     freq = host->usr_cfg.max_clk;
    LOG_I("SDHCI clock %d\n", freq);
    return freq;
}

static unsigned int sifli_sdio_min_clock_get(struct sdhci_host *host)
{
    //uint32_t freq = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);//48 * 1000 * 1000; //HAL_RCC_GetPCLK2Freq();
    uint32_t freq = 400000;
    LOG_D("SDHCI clock %d\n", freq);
    return freq;
}

static unsigned int sifli_get_timeout_clock(struct sdhci_host *host)
{
    return 1000;
}


static const struct sdhci_ops_t  sdhci_ops =
{
    .set_clock = NULL,
    .enable_dma = NULL,
    .get_max_clock = sifli_sdio_clock_get,
    .get_min_clock = sifli_sdio_min_clock_get,
    .get_timeout_clock = sifli_get_timeout_clock,
};

/*****************************************************************************\
 *                                                                           *
 * Interrupt handling                                                        *
 *                                                                           *
 \*****************************************************************************/

static void sdhci_cmd_irq(struct sdhci_host *host, uint32_t intmask)
{
    RT_ASSERT(intmask);
    //dbg_message(SDIO_DBG,"SDDEBUG: sdhci.c sdhci_cmd_irq \n");
    if (!host->cmd)
    {
        //printk(KERN_ERR "%s: Got command interrupt 0x%08x even "
        //      "though no command operation was in progress.\n",
        //      mmc_hostname(host->mmc), (unsigned)intmask);
        return;
    }

    if (intmask & SDHCI_INT_TIMEOUT)
        host->cmd->err = -ETIMEDOUT;
    else if (intmask & (SDHCI_INT_CRC | SDHCI_INT_END_BIT |
                        SDHCI_INT_INDEX))
        host->cmd->err = -EILSEQ;

    if (host->cmd->err)
    {
        //tasklet_schedule(&host->finish_tasklet);
        return;
    }

    /*
     * The host can send and interrupt when the busy state has
     * ended, allowing us to wait without wasting CPU cycles.
     * Unfortunately this is overloaded on the "data complete"
     * interrupt, so we need to take some care when handling
     * it.
     *
     * Note: The 1.0 specification is a bit ambiguous about this
     *       feature so there might be some problems with older
     *       controllers.
     */
    if (resp_type(host->cmd) == RESP_R1B)
    {
        if (host->cmd->data)
            LOG_D("Cannot wait for busy signal when also "
                  "doing a data transfer");
        else
            return;

        /* The controller does not support the end-of-busy IRQ,
         * fall through and take the SDHCI_INT_RESPONSE */
    }

    if (intmask & SDHCI_INT_RESPONSE)
        sdhci_finish_command(host);
}

static void sdhci_show_adma_error(struct sdhci_host *host) { }

static void sdhci_data_irq(struct sdhci_host *host, uint32_t intmask)
{
    //RT_ASSERT(intmask);
    //dbg_message(SDIO_DBG,"SDDEBUG: sdhci.c sdhci_data_irq\n");
    //  dbg_message(SDIO_READ,"SDDEBUG:PID:0x%p Data Interrupt Status:0x%x\n",current,intmask);
    if (!host->data)
    {
        /*
         * The "data complete" interrupt is also used to
         * indicate that a busy state has ended. See comment
         * above in sdhci_cmd_irq().
         */
        if (host->cmd && (host->cmd->flags == RESP_R1B))
        {
            if (intmask & SDHCI_INT_DATA_END)
            {
                sdhci_finish_command(host);
                return;
            }
        }

        //printk(KERN_ERR "%s: Got data interrupt 0x%08x even "
        //      "though no data operation was in progress.\n",
        //      mmc_hostname(host->mmc), (unsigned)intmask);

        return;
    }
    if (intmask & 0x8000)
    {
        //printk("error interrupt occured\n");
    }
    if (intmask & SDHCI_INT_DATA_TIMEOUT)
        host->data->err = -ETIMEDOUT;
    else if (intmask & (SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_END_BIT))
        host->data->err = -EILSEQ;
    else if (intmask & SDHCI_INT_ADMA_ERROR)
    {
        //printk(KERN_ERR "%s: ADMA error\n", mmc_hostname(host->mmc));
        sdhci_show_adma_error(host);
        host->data->err = -EIO;
    }

    if (host->data->err)
        sdhci_finish_data(host);
    else
    {
        uint8_t is_read = host->data->flags & DATA_DIR_READ ? 1 : 0;
        if (intmask & (SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL))
        {
            hal_sdhci_transfer_pio(&host->handle, is_read, (uint8_t *)host->data->buf, (uint32_t)host->data->blksize);
        }

        /*
         * We currently don't do anything fancy with DMA
         * boundaries, but as we can't disable the feature
         * we need to at least restart the transfer.
         */
        if (intmask & SDHCI_INT_DMA_END)
            //sdhci_writel(host, sdhci_readl(host, SDHCI_DMA_ADDRESS),
            //             SDHCI_DMA_ADDRESS);
            hal_sdhci_reset_dma_addr(&host->handle);

        if (intmask & SDHCI_INT_DATA_END)
        {
            if (host->cmd)
            {
                /*
                 * Data managed to finish before the
                 * command completed. Make sure we do
                 * things in the proper order.
                 */
                host->data_early = 1;
            }
            else
            {
                sdhci_finish_data(host);
            }
        }
    }
}

//static void sdhci_isr(void *handle)
//{
//    struct sdhci_host *host = handle;
//    uint32_t intstatus = sdhci_readl(host, SDHCI_INT_STATUS);
//    LOG_D("sdhci_isr 0x%x\n",intstatus);
//    rt_event_send(&host->event, intstatus);
//}

static int sdhci_irq(void *dev)
{
    struct sdhci_host *host = dev;
    uint32_t intmask, val;
    // dbg_message(SDIO_DBG,"SDDEBUG: sdhci.c sdhci_irq\n");
    //spin_lock(&host->lock);


    intmask = hal_sdhci_get_int_value(&host->handle);
    LOG_D("sdhci_irq 0x%x, flag = %x\n", intmask, host->irq_flag);
    host->irq_flag |= intmask; // save all bits when previous irq not process to avoid lose irq
    //LOG_D("flag2 = %x\n",  host->irq_flag);
    if (intmask & 0x8000)
    {
        //data_crc_read(host);
        //if(intmask & 0x208000)
        //sdhci_read_block_pio(host);
        //else
        //printk("err interrupt occurs= %x\n",intmask);
    }
    if (!intmask || intmask == 0xffffffff)
    {
        //result = 0; //IRQ_NONE;
        return 0;
    }
    if (intmask & (SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE))
    {
        hal_sdhci_clear_int(&host->handle, intmask & (SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE));
        //tasklet_schedule(&host->card_tasklet);
    }

    //DBG("*** %s got interrupt: 0x%08x\n",
    //      mmc_hostname(host->mmc), intmask);
//  if(intmask & (SDHCI_DATA_INHIBIT))
//  {
    //  do_gettimeofday(&etv);
    //  printk("end time sec %ld microseconds %ld\n",etv.tv_sec,etv.tv_usec);
    //}
//
    //dbg_message(SDIO_DBG,"SDDEBUG*** %s got interrupt: 0x%08x\n",mmc_hostname(host->mmc), intmask);
    //dbg_message(SDIO_READ,"\nSDDEBUG:PID:0x%p Normal Int Status: 0x%08x\n",current,intmask);
    if (intmask & eMMC_BOOT_ACK_INT)
    {
        hal_sdhci_clear_int(&host->handle, intmask & eMMC_BOOT_ACK_INT);
        //printk("Boot Acknowledge interrupt occured \n");
    }
    if (intmask & eMMC_BOOT_DONE_INT)
    {
        //val = SDHCI_TRANSFER_MODE & (~(1 << 15));//stop boot operation
        hal_sdhci_clear_int(&host->handle, intmask & eMMC_BOOT_DONE_INT);
        //sdhci_writew(host, val, SDHCI_TRANSFER_MODE); //to terminate the boot operation.
        hal_sdhci_set_boot_mode(&host->handle, 0);
    }
    if (intmask & SDHCI_TUNING_ERROR)
    {
        LOG_I("TUNING ERROR OCCURED STARTS RETUNING\n");
        //retuning(host);
    }

    if (intmask & 0x1000)
    {
        //printk("Host requires retuning \n");
    }
    if (intmask & (SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE))
    {
        hal_sdhci_clear_int(&host->handle, intmask & (SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE));
        //tasklet_schedule(&host->card_tasklet);
    }

    intmask &= ~(SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE);

    if (intmask & SDHCI_INT_CMD_MASK)
    {
        hal_sdhci_clear_int(&host->handle, intmask & SDHCI_INT_CMD_MASK);
        //sdhci_cmd_irq(host, intmask & SDHCI_INT_CMD_MASK);
    }

    if (intmask & SDHCI_INT_DATA_MASK)
    {

        hal_sdhci_clear_int(&host->handle, intmask & SDHCI_INT_DATA_MASK);
        //sdhci_data_irq(host, intmask & SDHCI_INT_DATA_MASK);
    }

    intmask &= ~(SDHCI_INT_CMD_MASK | SDHCI_INT_DATA_MASK);

    intmask &= ~SDHCI_INT_ERROR;

    if (intmask & SDHCI_INT_BUS_POWER)
    {
        //printk(KERN_ERR "%s: Card is consuming too much power!\n",
        //      mmc_hostname(host->mmc));
        hal_sdhci_clear_int(&host->handle, SDHCI_INT_BUS_POWER);
    }

    intmask &= ~SDHCI_INT_BUS_POWER;

    if (intmask & SDHCI_INT_CARD_INT)
    {
        sdio_irq_wakeup(host->mmc);
        intmask &= ~SDHCI_INT_CARD_INT;
    }

    if (intmask)
    {
        //printk(KERN_ERR "%s: Unexpected interrupt 0x%08x.\n",
        //      mmc_hostname(host->mmc), intmask);

        hal_sdhci_clear_int(&host->handle, intmask);
    }

    if (host->data != NULL)  // if data valid, interrupt bit should include data mask
    {
        if ((host->irq_flag & SDHCI_INT_DATA_MASK) != 0)
        {
            rt_event_send(&host->event, host->irq_flag);
        }
        else
        {
            LOG_D("Wait more irq 0x%x\n", host->irq_flag);
        }
    }
    else
        rt_event_send(&host->event, host->irq_flag);

    //rt_event_send(&host->event, flag);

    return 0;
}

static int sdhci_wait_completed(struct sdhci_host *sdio, int flag)
{
    int res = 0;
    rt_uint32_t status, rci;
    struct rt_mmcsd_cmd *cmd = sdio->cmd;
    //struct rt_mmcsd_data *data = cmd->data;
    //SDMMC_TypeDef *hw_sdio = (SDMMC_TypeDef *)sdio->ioaddr;

    if (rt_event_recv(&sdio->event, 0xffffffff, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      rt_tick_from_millisecond(5000), &status) != RT_EOK)
    {
        LOG_E("wait %d completed timeout 0x%08x,arg 0x%08x\n", cmd->cmd_code, hal_sdhci_get_int_value(&sdio->handle), cmd->arg);
        //LOG_D("Int EN 0x%08x, Mask 0x%08x\n", sdhci_readl(sdio, SDHCI_INT_ENABLE), sdhci_readl(sdio, SDHCI_SIGNAL_ENABLE));
        cmd->err = -RT_ETIMEOUT;
        return -1;
    }
    LOG_D("sdhci_wait_completed cmd %d:  0x%x, flag 0x%x\n", cmd->cmd_code, status, flag);

    //status = host->irq_flag;

    if (status & SDHCI_INT_ERROR_MASK)
    {
        if ((((flag & SDHCI_CMD_CRC) == 0) && (status & SDHCI_INT_CRC)) || (((flag & SDHCI_CMD_INDEX) == 0) && (status & SDHCI_INT_INDEX)))
        {
            // do not care this error
            cmd->err = RT_EOK;
            res = 0;
            LOG_D("Response do not care CRC and index\n");
        }
        else
        {
            LOG_D("Wait sdhci error 0x%x\n", status);
            cmd->err = -RT_ERROR;
            res = -2;
        }
    }
    // how to make sure all irq come ? add a delay?
    //if (psta)
    //    *psta = host->irq_flag;

    if (status & SDHCI_INT_CMD_MASK)
    {
        sdhci_cmd_irq(sdio, status & SDHCI_INT_CMD_MASK);
    }

    if (status & SDHCI_INT_DATA_MASK)
    {
        sdhci_data_irq(sdio, status & SDHCI_INT_DATA_MASK);
    }

    if (resp_type(cmd) != RESP_NONE)
    {
        LOG_D("Resp: 0x%08x, 0x%08x,0x%08x,0x%08x\n", cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);
    }

    return res;
}

/*****************************************************************************\
 *                                                                           *
 * Device allocation/registration                                            *
 *                                                                           *
 \*****************************************************************************/
static struct rt_mmcsd_host *emmc_host = NULL;
void sdhci_set_emmchost(struct rt_mmcsd_host *host)
{
    emmc_host = host;
}
struct rt_mmcsd_host *sdhci_get_emmchost(void)
{
    return emmc_host;
}

int sdhci_add_host(struct sdhci_host *host)
{
    int ret;

    struct rt_mmcsd_host *mmc;
    unsigned int caps;

    //host->flags |= SDHCI_USE_SDMA;
    if (host == NULL)
        return -1;

    mmc = mmcsd_alloc_host();
    if (mmc) sdhci_set_emmchost(mmc);
    if (host == RT_NULL)
    {
        LOG_E("mmcsd alloc host fail");
        return -2;
    }

    /* link up host and sdio */
    host->mmc = mmc;
    mmc->private_data = host;

    //mmc = host->mmc;

    hal_sdhci_reset(&host->handle, SDHCI_RESET_ALL);

    host->version = hal_sdhci_get_host_version(&host->handle);
    LOG_D("Reg VERSION 0x%x\n", host->version);
    host->version = (host->version & SDHCI_SPEC_VER_MASK)
                    >> SDHCI_SPEC_VER_SHIFT;
    LOG_I("host version = %x\n", host->version);

    caps = hal_sdhci_get_host_cap(&host->handle);
    LOG_D("SDHCI caps 0x%x\n", caps);

    if (!(caps & SDHCI_CAN_DO_SDMA))
        LOG_I("Controller doesn't have SDMA capability\n");
    else
        host->handle.Init.flags |= SDHCI_USE_SDMA;
    //  printk("value of host ->flags =%x\n",host->flags);

    //if ((host->version >= SDHCI_SPEC_200) && (caps & SDHCI_CAN_DO_ADMA2))
    //    host->handle.Init.flags |= SDHCI_USE_ADMA;

#if 1
    if (host->cfg_flag & SDHCI_USER_CONFIG_DMA)
    {
        switch (host->usr_cfg.dma_mode)
        {
        case SDHCI_DMA_PIO:
            host->handle.Init.flags &= ~(SDHCI_USE_ADMA | SDHCI_USE_SDMA | SDHCI_REQ_USE_DMA);
            break;
        case SDHCI_SDMA:
            host->handle.Init.flags |= (SDHCI_USE_SDMA | SDHCI_REQ_USE_DMA);
            break;
        case SDHCI_ADMA:
            host->handle.Init.flags |= (SDHCI_USE_ADMA | SDHCI_REQ_USE_DMA);
            break;
        default:
            LOG_E("Error DMA mode %d, flag 0x%x\n", host->usr_cfg.dma_mode, host->cfg_flag);
            break;
        }
    }
#endif

    if (host->handle.Init.flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA))
    {
        if (host->ops->enable_dma)
        {
            if (host->ops->enable_dma(host))
            {
                LOG_I(" No suitable DMA available. Falling back to PIO.\n");
                host->handle.Init.flags &=
                    ~(SDHCI_USE_SDMA | SDHCI_USE_ADMA);
            }
        }
    }

    if (host->handle.Init.flags & SDHCI_USE_ADMA)
    {
        LOG_D("SDHCI USE ADMA\n");
        /*
         * We need to allocate descriptors for all sg entries
         * (128) and potentially one alignment transfer for
         * each of those entries.
         */
        host->handle.Init.adma_desc = (uint8_t *)malloc((128 * 2 + 1) * 4); //??
        if (!host->handle.Init.adma_desc)
        {
            LOG_I(" Unable to allocate ADMA buffers. Falling back to standard DMA.\n");
            host->handle.Init.flags &= ~SDHCI_USE_ADMA;
        }
        else
        {
            host->adma_addr = (uint32_t)host->handle.Init.adma_desc;
            if (host->adma_addr & 3) // not 4byte alinged, assert ! rt_malloc always 4 aligned!
            {
                RT_ASSERT(0);
            }
        }
    }

    /*
     * If we use DMA, then it's up to the caller to set the DMA
     * mask, but PIO does not need the hw shim so we set a new
     * mask here in that case.
     */
    //if (!(host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA)))
    //{
    //    host->dma_mask = 0xffffffff; //0xffffffffffffffff; //DMA_BIT_MASK(64);
    //}
    if (host->version == SDHCI_SPEC_300)
    {
        //host->max_clk =
        //(caps & SDHCI_CLOCK_BASE_3 ) >> SDHCI_CLOCK_BASE_SHIFT;
        if (host->ops->get_max_clock)
            host->max_clk = host->ops->get_max_clock(host) / 1000000;
        else
            host->max_clk = 48; //200;  // for FPGA, max 48MHZ
    }
    else
    {
        host->max_clk =
            (caps & SDHCI_CLOCK_BASE_MASK) >> SDHCI_CLOCK_BASE_SHIFT;
    }
    LOG_I("Maximum Clock Supported by HOST : %d MHz \n", host->max_clk);
    host->max_clk *= 1000000;
    if (host->max_clk == 0)
    {
        if (!host->ops->get_max_clock)
        {
            LOG_I("Hardware doesn't specify base clock frequency.\n");
            return -1;
        }
        host->max_clk = host->ops->get_max_clock(host);
        LOG_D("Clock Supported by HOST : %d  \n", host->max_clk);
    }

    host->timeout_clk =
        (caps & SDHCI_TIMEOUT_CLK_MASK) >> SDHCI_TIMEOUT_CLK_SHIFT;
    //host->timeout_clk = 12;
    LOG_D("Timeout Clock from  Capability Reg: %d MHz \n", host->timeout_clk);
    if (host->timeout_clk == 0)
    {
        if (host->ops->get_timeout_clock)
        {
            host->timeout_clk = host->ops->get_timeout_clock(host);
        }
        else
        {
            LOG_I(" Hardware doesn't specify timeout clock frequency.\n");
            return -2;
        }
    }
    if (caps & SDHCI_TIMEOUT_CLK_UNIT)
        host->timeout_clk *= 1000;

    //printk("resp[126] =%d\n",test.resp[126]);


    /*
     * Set host parameters.
     */
    mmc->ops = &sdhci_host_ops;

    if (host->ops->get_min_clock)
    {
        mmc->freq_min = host->ops->get_min_clock(host);
    }
    else
    {
        mmc->freq_min = host->max_clk / 256;
        if (host->version == SDHCI_SPEC_300)
        {
            //host->max_clk =
            //(caps & SDHCI_CLOCK_BASE_3 ) >> SDHCI_CLOCK_BASE_SHIFT;
            //mmc->freq_min = host->max_clk / 2048;
            //mmc->freq_min = SDIO_MIN_FREQ;
            mmc->freq_min = host->max_clk / 128;
        }
    }
    mmc->freq_max = host->max_clk;
    LOG_I("host minclock %d  host maxclock %d  \n", mmc->freq_min, mmc->freq_max);
    mmc->flags = MMCSD_SUP_SDIO_IRQ;

    mmc->flags |= MMCSD_BUSWIDTH_4;

    if (caps & SDHCI_CAN_DO_HISPD)
        mmc->flags |= MMCSD_SUP_HIGHSPEED;

    // TODO
    //if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)
    //    mmc->flags |= MMC_CAP_NEEDS_POLL;

    mmc->valid_ocr = 0;
    if (caps & SDHCI_CAN_VDD_330)
        mmc->valid_ocr |= (VDD_32_33 | VDD_33_34);  //MMC_VDD_32_33 | MMC_VDD_33_34;
    if (caps & SDHCI_CAN_VDD_300)
        mmc->valid_ocr |= (VDD_29_30 | VDD_30_31); //MMC_VDD_29_30 | MMC_VDD_30_31;
    if (caps & SDHCI_CAN_VDD_180)
        mmc->valid_ocr |= VDD_165_195; //MMC_VDD_165_195;

    if (mmc->valid_ocr == 0)
    {
        LOG_I("Hardware doesn't report any support voltages.\n");
        mmc->valid_ocr |= (VDD_32_33 | VDD_33_34);
        //return -3;
    }
    mmc->valid_ocr = 0X00FFFF80;/* The voltage range supported is 1.65v-3.6v */

    //spin_lock_init(&host->lock);

    /*
     * Maximum number of segments. Depends on if the hardware
     * can do scatter/gather or not.
     */
    //if (host->flags & SDHCI_USE_ADMA)
    //    mmc->max_hw_segs = 128;
    //else if (host->flags & SDHCI_USE_SDMA)
    //    mmc->max_hw_segs = 1;
    //else /* PIO */
    //    mmc->max_hw_segs = 128;
    //mmc->max_phys_segs = 1;
    mmc->max_dma_segs = 1;

    /*
     * Maximum number of sectors in one transfer. Limited by DMA boundary
     * size (512KiB).
     */
    //mmc->max_req_size = 524288;
    //mmc->max_blk_size = 512;
    //mmc->max_blk_count = 512;

    /*
     * Maximum segment size. Could be one segment with the maximum number
     * of bytes. When doing hardware scatter/gather, each entry cannot
     * be larger than 64 KiB though.
     */
    if (host->handle.Init.flags & SDHCI_USE_ADMA)
        mmc->max_seg_size = SDHCI_ADMA_MAX_SIZE;
    else
        mmc->max_seg_size = 65536; //512; //SDIO_BUFF_SIZE; //mmc->max_req_size;

    /*
     * Maximum block size. This varies from controller to controller and
     * is specified in the capabilities register.
     */
    {
        mmc->max_blk_size = (caps & SDHCI_MAX_BLOCK_MASK) >>
                            SDHCI_MAX_BLOCK_SHIFT;
        if (mmc->max_blk_size >= 3)
        {
            LOG_I(" Invalid maximum block size, assuming 512 bytes\n");
            mmc->max_blk_size = 0;
        }
    }

    mmc->max_blk_size = 512 << mmc->max_blk_size;

    /*
     * Maximum block count.
     */
    mmc->max_blk_count = 65535;

    sdhci_init(host, 0);


    LOG_I("SDHCI controller on %s using %s\n",
          host->hw_name,
          (host->handle.Init.flags & SDHCI_USE_ADMA) ? "ADMA" :
          (host->handle.Init.flags & SDHCI_USE_SDMA) ? "DMA" : "PIO");

    hal_sdhci_enable_card_detection(&host->handle);

    // clear buffer before start
    //sdhci_readl(host, SDHCI_BUFFER);

    // set debug port
#if 0
//#define SD_DEBUG_PORT          (1)
    /* FOR USB DEBUG PORT */
    {
        volatile uint32_t *hdbg = (volatile uint32_t *)0x4000001C;
        *hdbg = 0XFF01FF01;
        //*hdbg = (0xff << 24) | (1 << 16) | (0xff << 8) | (1 << 0);
        //hwp_hpsys_rcc->DBGR = (0xff << 24) | (SD_DEBUG_PORT << 16) | (0xff << 8) | (SD_DEBUG_PORT << 0);

        // hpcd->Instance->DBG_OUT_SEL |= ((1 << 15) | (1 << 14) | (1 << 13) | (1 << 12) ;

    }
#endif

    if (PM_STANDBY_BOOT != SystemPowerOnModeGet())  // standby do noct destory event/mutex, can not init again
    {
        // enable irq
        //rthw_sdio_irq_update(host, 1);
        rt_event_init(&host->event, "sdhci", RT_IPC_FLAG_FIFO);
        rt_mutex_init(&host->mutex, "sdhci", RT_IPC_FLAG_FIFO);
    }
    /* ready to change */
    mmcsd_change(mmc);
    LOG_I("Add host success\n");

    return 0;
}

#ifndef SF32LB56X
void SDMMC2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* Process All SDIO Interrupt Sources */
    //rthw_sdio_irq_process(host);
    sdhci_irq(&sdhci_ctx[1]);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif //SF32LB56X

void SDMMC1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* Process All SDIO Interrupt Sources */
    //rthw_sdio_irq_process(host);
    sdhci_irq(&sdhci_ctx[0]);

    /* leave interrupt */
    rt_interrupt_leave();
}


// add some output interface for sd test.
int rt_sdmmc_set_clock(int id, uint32_t clk)
{
    struct sdhci_host *host = &sdhci_ctx[id];

    if (clk <= 0)
        return 0;

    host->cfg_flag |= SDHCI_USER_CONFIG_CLK;
    host->usr_cfg.max_clk = clk;

    return 0;
}

int rt_sdmmc_enable_ddr(int id, uint8_t en_ddr)
{
    struct sdhci_host *host = &sdhci_ctx[id];
    // need send command 6 to card to change mode
#if 0
    host->cfg_flag |= SDHCI_USER_CONFIG_DDR;
    host->usr_cfg.ddr_mode = en_ddr;
#else
    //sdhci_set_ddr(host, en_ddr);
#endif
    return 0;
}

// mode 0 = PIO, mode 1 = SDMA, mode 2 = ADMA, set a enum
int rt_sdmmc_set_dma_mode(int id, uint8_t mode)
{
    struct sdhci_host *host = &sdhci_ctx[id];

    if (mode > 0)
        return 0;

    host->cfg_flag |= SDHCI_USER_CONFIG_DMA;
    host->usr_cfg.dma_mode = mode;
    return 0;
}

uint32_t rt_sdmmc_get_clock(int id)
{
    struct sdhci_host *host = &sdhci_ctx[id];

    if (host == NULL)
        return 0;

    return host->clock;
}

#ifdef RT_USING_PM
__weak void BSP_SD_PowerUp(void)
{
}
__weak void BSP_SD_PowerDown(void)
{
}
static rt_err_t rt_sdhci_control(struct rt_device *dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    uint8_t mode = (uint8_t)((uint32_t)args);
    uint8_t id = (uint8_t)((uint32_t)dev->user_data);
#ifdef SDIO_PM_MODE
    if (id == 0)
        sdhci_handle.Instance = (uint32_t)SDIO1;
    else
        sdhci_handle.Instance = (uint32_t)SDIO2;
#endif
    if (id > 1)
        return RT_ERROR;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RESUME:
    {
        if (PM_SLEEP_MODE_STANDBY == mode)
        {
            rt_kprintf("Resume from stand by id=%d\n", id);
            BSP_SD_PowerUp();
#ifdef SDIO_POWER_PM_UP
            recov_sdhci_reg(id);
#elif SDIO_POWER_PM_ALL_DOWN
            sdmmc_pm_resume_init(id);
#elif SDIO_POWER_PM_VCC_DOWN
            recov_sdhci_reg(id);
            sd_send_cmd(5, 0);
            sd_clr_status();
#else
            recov_sdhci_reg(id);
#endif
        }
        break;
    }
    case RT_DEVICE_CTRL_SUSPEND:
    {
        if ((PM_SLEEP_MODE_STANDBY == mode) && (sdhci_ctx[id].mmc != NULL))
        {
            rt_kprintf("SD suspend id=%d\n", id);

#ifdef SDIO_POWER_PM_UP
            dump_sdhci_reg(id);
#elif SDIO_POWER_PM_ALL_DOWN
            //do nothing
#elif SDIO_POWER_PM_VCC_DOWN
            sd_send_cmd(5, 0);
            sd_clr_status();
            dump_sdhci_reg(id);
#else
            dump_sdhci_reg(id);
#endif
            BSP_SD_PowerDown();
        }
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
static const rt_device_ops sdhci_device_ops =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    rt_sdhci_control,
};
#endif

static void rt_sdhci_register_rt_device(uint8_t id)
{
    rt_err_t err = RT_EOK;
    rt_device_t device;
    char devname[8] = {0};

    if (id > 1)
        return;

    device = &rt_sdhci_device[id];

    device->type        = RT_Device_Class_Miscellaneous;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &sdhci_device_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_sdhci_control;
#endif
    device->user_data   = (void *)((uint32_t)id);

    rt_snprintf(devname, 8, "sdhci%d",  id);
    err = rt_device_register(device, devname, RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);
    RT_ASSERT(RT_EOK == err);

}
#endif  /* RT_USING_PM */

int rt_sdhci_init_instance(uint8_t id)
{
    int ret = 0;

    LOG_I("rt_hw_sdmmc_init %d begin\n", id + 1);

    if (id > 1)
    {
        LOG_E("Invalid SD instance ID %d\n", id + 1);
        return -1;
    }

    if (rt_sdhci_cfg_def[id].max_freq == 0)
    {
        LOG_E("Invalid freq setting 0 for SD%d\n", id + 1);
        return -1;
    }

    memset(&(sdhci_ctx[id].usr_cfg), 0, sizeof(sdhci_user_config_t));
    HAL_SDHCI_MspInit(&sdhci_ctx[id].handle);

    rt_sdmmc_set_clock(id, rt_sdhci_cfg_def[id].max_freq);

    rt_sdmmc_set_dma_mode(id, rt_sdhci_cfg_def[id].dma_mode);

    sdhci_ctx[id].usr_cfg.sdio_mode = rt_sdhci_cfg_def[id].card_mode;

    if (id == 0)
        HAL_RCC_ResetModule(RCC_MOD_SDMMC1);
    else
        HAL_RCC_ResetModule(RCC_MOD_SDMMC2);

    HAL_Delay(20);

    ret = sdhci_add_host(&sdhci_ctx[id]);
    if (ret != 0)
    {
        LOG_E("host create fail with sd%d\n", id + 1);
        return -1;
    }


    HAL_NVIC_SetPriority((IRQn_Type)rt_sdhci_cfg_def[id].irqn, 2, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)rt_sdhci_cfg_def[id].irqn);

#ifdef SDIO_PM_MODE
    if (PM_STANDBY_BOOT != SystemPowerOnModeGet())
    {
        rt_sdhci_register_rt_device(id);
    }
#endif /* RT_USING_PM */

    LOG_I("rt_hw_sdmmc_init %d done\n", id + 1);

    return 0;
}

int rt_hw_sdmmc_init(void)
{
    int ret = 0;

#ifdef BSP_USING_SDHCI1
    HAL_RCC_EnableModule(RCC_MOD_SDMMC1);
    sdhci_ctx[0].hw_name = "sdmmc";
    sdhci_ctx[0].ops = &sdhci_ops;
    sdhci_ctx[0].handle.Instance = (uint32_t)SDIO1;
    sdhci_ctx[0].irq_flag = 0;
    ret = rt_sdhci_init_instance(0);
#endif

#ifdef BSP_USING_SDHCI2
    HAL_RCC_EnableModule(RCC_MOD_SDMMC2);
    sdhci_ctx[1].hw_name = "sdmmc2";
    sdhci_ctx[1].ops = &sdhci_ops;
    sdhci_ctx[1].handle.Instance = (uint32_t)SDIO2;
    sdhci_ctx[1].irq_flag = 0;
    ret = rt_sdhci_init_instance(1);
#endif

    return ret;
}

INIT_DEVICE_EXPORT(rt_hw_sdmmc_init);

int rt_hw_sdmmc_num_init(uint8_t sdhci_num)
{
    int ret = 0;
    switch (sdhci_num)
    {
    case 0:
#ifdef BSP_USING_SDHCI1
        HAL_RCC_EnableModule(RCC_MOD_SDMMC1);
        sdhci_ctx[0].hw_name = "sdmmc";
        sdhci_ctx[0].ops = &sdhci_ops;
        sdhci_ctx[0].handle.Instance = (uint32_t)SDIO1;
        sdhci_ctx[0].irq_flag = 0;
        ret = rt_sdhci_init_instance(0);
#endif
        break;
    case 1:
#ifdef BSP_USING_SDHCI2
        HAL_RCC_EnableModule(RCC_MOD_SDMMC2);
        sdhci_ctx[1].hw_name = "sdmmc2";
        sdhci_ctx[1].ops = &sdhci_ops;
        sdhci_ctx[1].handle.Instance = (uint32_t)SDIO2;
        sdhci_ctx[1].irq_flag = 0;
        ret = rt_sdhci_init_instance(1);
#endif
        break;
    }
    return ret;
}

int rt_hw_sdmmc_deinit(uint8_t id)
{
    mmcsd_host_lock(sdhci_ctx[id].mmc);
    sdio_unregister_card(sdhci_ctx[id].mmc->card);
    rt_mmcsd_blk_remove(sdhci_ctx[id].mmc->card);
    rt_free(sdhci_ctx[id].mmc->card);
    sdhci_ctx[id].mmc->card = RT_NULL;
    mmcsd_free_host(sdhci_ctx[id].mmc);
    sdhci_ctx[id].mmc = RT_NULL;
    if ((sdhci_ctx[id].handle.Init.flags & SDHCI_USE_ADMA) && sdhci_ctx[id].handle.Init.adma_desc)
    {
        free(sdhci_ctx[id].handle.Init.adma_desc);
    }
    return 0;
}

int sdhci_reinit_host(uint8_t host_index)
{
    struct rt_mmcsd_host *mmc;
    struct sdhci_host *host;

    if (host_index > 1)
        return -1;

    host = &sdhci_ctx[host_index];
    mmc = host->mmc;

    if (mmc == NULL)
        return 0;

    if (host_index == 0)
        HAL_RCC_ResetModule(RCC_MOD_SDMMC1);
    else
        HAL_RCC_ResetModule(RCC_MOD_SDMMC2);
    HAL_Delay(20);

    mmcsd_host_lock(sdhci_ctx[host_index].mmc);
    sdio_unregister_card(sdhci_ctx[host_index].mmc->card);
    rt_mmcsd_blk_remove(sdhci_ctx[host_index].mmc->card);
    rt_free(sdhci_ctx[host_index].mmc->card);
    sdhci_ctx[host_index].mmc->card = RT_NULL;
    mmcsd_host_unlock(sdhci_ctx[host_index].mmc);

    hal_sdhci_reset(&host->handle, SDHCI_RESET_ALL);

    sdhci_init(host, 0);

    hal_sdhci_enable_card_detection(&host->handle);

    mmcsd_change(mmc);

    return 0;
}
#ifdef SDIO_PM_MODE
#define read_memory(addr)        (*(volatile unsigned int *)((addr)))
#define write_memory(addr,value) (*(volatile unsigned int *)((addr))) = (value)
#define read_byte(addr)          (*(volatile unsigned char *)((addr)))
#define write_byte(addr,value)   (*(volatile unsigned char *)((addr))) = (value)
#define read_hword(addr)         (*(volatile unsigned short *)((addr)))
#define write_hword(addr,value)  (*(volatile unsigned short *)((addr))) = (value)
#define RESP_R1b     (2 << 0)
#define RESP_R5b     (7 << 0)
SDHCI_HandleTypeDef sdhci_handle;
void sd_wr_word(uint8_t reg_addr, uint32_t data)
{
    write_memory(sdhci_handle.Instance + (uint32_t)reg_addr, data);
    //HAL_SDHCI_WRITE_LONG(data, sdhci_handle.Instance + reg_addr);
}

void sd_wr_hword(uint8_t reg_addr, uint16_t data)
{
    write_hword(sdhci_handle.Instance + (uint32_t)reg_addr, data);
    //HAL_SDHCI_WRITE_SHORT(data, sdhci_handle.Instance + reg_addr);
}

void sd_wr_byte(uint8_t reg_addr, uint8_t data)
{
    write_byte(sdhci_handle.Instance + (uint32_t)reg_addr, data);
    //HAL_SDHCI_WRITE_BYTE(data, sdhci_handle.Instance + reg_addr);
}

uint32_t sd_rd_word(uint8_t reg_addr)
{
    return read_memory(sdhci_handle.Instance + (uint32_t)reg_addr);
}

uint16_t sd_rd_hword(uint8_t reg_addr)
{
    return read_hword(sdhci_handle.Instance + (uint32_t)reg_addr);
}

uint8_t sd_rd_byte(uint8_t reg_addr)
{
    return read_hword(sdhci_handle.Instance + (uint32_t)reg_addr);
}
uint32_t sd_wait_cmd()
{
    while ((sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_CMD_MASK) == 0);
    return (sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_CMD_MASK);
}
uint32_t sd_clr_status()
{
    sd_wr_word(SDHCI_INT_STATUS, SDHCI_INT_ALL_MASK);
    return 0;
}

void sd_get_rsp(uint32_t *rsp_arg1, uint32_t *rsp_arg2, uint32_t *rsp_arg3, uint32_t *rsp_arg4)
{
    *rsp_arg1 = sd_rd_word(SDHCI_RESPONSE);
    *rsp_arg2 = sd_rd_word(SDHCI_RESPONSE + 0x4);
    *rsp_arg3 = sd_rd_word(SDHCI_RESPONSE + 0x8);
    *rsp_arg4 = sd_rd_word(SDHCI_RESPONSE + 0xc);
}

uint32_t sd_send_cmd(uint8_t cmd_idx, uint32_t cmd_arg)
{
    SDHCI_CmdArgTypeDef mcmd;
    mcmd.Arg = (uint32_t)cmd_arg;
    mcmd.CmdIndex = (uint32_t)cmd_idx;

    uint32_t resp_type;
    uint8_t with_data;
    uint8_t cmd_para;
    switch (cmd_idx)
    {
    case  0:
    case  4:
    case  5:
        resp_type = RESP_R4;
        break;
    case 15:
        resp_type = RESP_NONE;
        break;
    case  2:
    case  9:
    case 10:
        resp_type = RESP_R2;
        break;
    case 41:
        resp_type = RESP_R3;
        break; //ACMD41
    case  3:
        resp_type = RESP_R6;
        break;
    case  8:
        resp_type = RESP_R7;
        break;
    case  6:
    case  7:
        resp_type = RESP_R1b;
        break;
    default:
        resp_type = RESP_R1;
        break;
    }

    switch (cmd_idx)
    {
    case 24:
    case 25:
    case 21:
    case 17:
    case 18:
        with_data = 1;
        break;
    default:
        with_data = 0;
        break;
    }
    switch (resp_type)
    {
    case RESP_NONE:
        cmd_para = SDHCI_CMD_RESP_NONE;
        break;
    case RESP_R2:
        cmd_para = SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC;
        break;
    case RESP_R3:
    case RESP_R4:
        cmd_para = SDHCI_CMD_RESP_SHORT;
        break;
    case RESP_R1:
    case RESP_R5:
    case RESP_R6:
    case RESP_R7:
        cmd_para = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;
        break;
    case RESP_R1b:
        cmd_para = SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;
        break;
    default:
        cmd_para = SDHCI_CMD_RESP_SHORT;
        break;
    }

    if (with_data) cmd_para |= SDHCI_CMD_DATA;
    mcmd.RespType = (uint32_t)cmd_para;
    hal_sdhci_send_command(&sdhci_handle, &mcmd);
    return sd_wait_cmd();
}
uint32_t sd_send_acmd(uint8_t cmd_idx, uint32_t cmd_arg, uint16_t rca)
{
    uint8_t cmd_result;

    cmd_result = sd_send_cmd(55, (uint32_t)rca << 16);
    if (cmd_result != SDHCI_INT_RESPONSE)
        return cmd_result;
    sd_clr_status();

    cmd_result = sd_send_cmd(cmd_idx, cmd_arg);
    return cmd_result;
}

uint32_t sd_wait_read()
{
    while ((sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_DATA_MASK) == 0);
    return (sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_DATA_MASK);
}

void sdmmc_set_clock_line(uint8_t id)
{
    rt_uint32_t arg = 0, i = 0;
    hal_sdhci_set_clk(&sdhci_handle, sdhci_ctx[id].clock, sdhci_ctx[id].max_clk);
    HAL_Delay(1);

    if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_DDR_BUS_WIDTH_4 || sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_DDR_BUS_WIDTH_8)
    {
        arg = (0x03 << 24) | (185 << 16) | (1 << 8) | 1;
        sd_send_cmd(SWITCH, arg);
        sd_clr_status();
    }
    arg = (0x03 << 24) | (183 << 16) | (sdhci_ctx[id].mmc->io_cfg.bus_width << 8) | 1;
    sd_send_cmd(SWITCH, arg);
    sd_clr_status();
    hal_sdhci_set_power(&sdhci_handle, sdhci_ctx[id].mmc->io_cfg.vdd);

    uint8_t ctrl = 0;

    if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_BUS_WIDTH_4)
    {
        ctrl = 4;
        sdhci_set_ddr(&sdhci_ctx[id], 0);
        sd_clr_status();
    }
    else if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_BUS_WIDTH_8)
    {
        ctrl = 8;
        sdhci_set_ddr(&sdhci_ctx[id], 0);
        sd_clr_status();
    }
    else if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_DDR_BUS_WIDTH_4)
    {
        ctrl = 4;
        sdhci_set_ddr(&sdhci_ctx[id], 1);
        sd_clr_status();
    }
    else if (sdhci_ctx[id].mmc->io_cfg.bus_width == MMCSD_DDR_BUS_WIDTH_8)
    {
        ctrl = 8;
        sdhci_set_ddr(&sdhci_ctx[id], 1);
        sd_clr_status();
    }
    else
        ctrl = 1;

    hal_sdhci_set_bus_width(&sdhci_handle, ctrl);
    sd_clr_status();
}

static int sdmmc_pm_resume_init(uint8_t id)
{
    uint32_t rsp_arg1;
    uint32_t rsp_arg2;
    uint32_t rsp_arg3;
    uint32_t rsp_arg4;
    uint32_t cmd_arg;
    uint16_t i;
    uint16_t rca;
    uint8_t  cmd_result;
    if (id == 0)
        sdhci_handle.Instance = (uint32_t)SDIO1;
    else
        sdhci_handle.Instance = (uint32_t)SDIO2;

    hal_sdhci_reset(&sdhci_handle, SDHCI_RESET_ALL);
    hal_sdhci_set_clk(&sdhci_handle, 400 * 1000, sdhci_ctx[id].max_clk);
    hal_sdhci_set_power(&sdhci_handle, sdhci_ctx[id].mmc->io_cfg.vdd | 0x100);
    hal_sdhci_init(&sdhci_handle, 0);
    hal_sdhci_enable_card_detection(&sdhci_handle);

    rca = 0x1;
    //reset eMMC
    cmd_result = sd_send_cmd(0, 0); //CMD0
    sd_clr_status();

    //CMD1
    cmd_arg = 0x40000080;
    while (1)  //wait for card busy status
    {
        cmd_result = sd_send_cmd(1, cmd_arg);
        sd_clr_status();
        sd_get_rsp(&rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);
        if (rsp_arg1 == 0xC0FF8080)
        {
            break; //card power up done
        }
        for (i = 0; i < 1000; i++) {} //add some delay
    }

    //CMD2
    cmd_arg = 0x0;
    cmd_result = sd_send_cmd(2, cmd_arg); //CMD2
    sd_clr_status();
    sd_get_rsp(&rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);

    //CMD3
    cmd_arg = rca << 16;
    cmd_result = sd_send_cmd(3, cmd_arg); //CMD3
    sd_clr_status();
    sd_get_rsp(&rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);

    sd_wr_byte(SDHCI_POWER_CONTROL, 0x10); //push-pull mode for cmd line

    //start card transfer
    //CMD9
    cmd_arg = (uint32_t)rca << 16;
    cmd_result = sd_send_cmd(9, cmd_arg);
    sd_clr_status();
    sd_get_rsp(&rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);

    //CMD7 (SELECT_CARD)
    cmd_arg = (uint32_t)rca << 16;
    cmd_result = sd_send_cmd(7, cmd_arg);
    sd_clr_status();

    //step 1, CMD8
    cmd_arg = 0x000001aa; //VHS=1
    cmd_result = sd_send_cmd(8, cmd_arg); //CMD8
    sd_clr_status();

    sdmmc_set_clock_line(id);
    for (i = 0; i < 1000; i++) {} //add some delay
    return 0;
}

#endif

#define DRV_SDHCI_TEST
#ifdef DRV_SDHCI_TEST
int cmd_sdhci(int argc, char *argv[])
{
    if (strcmp(argv[1], "-init") == 0)
    {
        rt_hw_sdmmc_init();
    }
    if (argc < 3)
    {
        LOG_I("Not enough parameter\n");
        LOG_I("sdcar [-r/-w/-t] start_addr length\n");
        return 1;
    }
    if (strcmp(argv[1], "-r") && strcmp(argv[1], "-w") && strcmp(argv[1], "-t"))
    {
        LOG_I("Invalid parameter\n");
        LOG_I("-r for read, -w for write\n");
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
    rt_uint32_t len = 512; // = atoi(argv[3]);
    rt_uint32_t value = 0;
    if (argc >= 5)
        value = atoi(argv[4]);
    if (argc >= 4)
        len = atoi(argv[3]);
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
        rt_err_t ret = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
        if ((ret != RT_EOK) && (ret != -RT_EBUSY))
        {
            LOG_I("Open device sd0 fail %d\n", ret);
            if (buf != NULL)
            {
                free(buf);
                buf = NULL;
            }
            return 1;
        }
        if ((dev->read == NULL) || (dev->write == NULL))
        {
            LOG_I("SD0 device read/write function empty!\n");
            if (buf != NULL)
            {
                free(buf);
                buf = NULL;
            }
            return 1;
        }

        int i, res, blk;
        blk = len >> 9;
        if (strcmp(argv[1], "-r") == 0)
        {
            res = rt_device_read(dev, addr, (void *)buf, blk);
            if (res > 0)
            {
                for (i = 0; i < len / 4; i++)
                {
                    LOG_RAW("0x%08x ", *(buf + i));
                    if ((i & 7) == 7)
                        LOG_RAW("\n");
                }
            }
            else
            {
                LOG_I("read data fail %d\n", res);
            }
        }
        else if (strcmp(argv[1], "-w") == 0)
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
        else if (strcmp(argv[1], "-t") == 0)
        {
            int start, end, fail, total;
            int clk, dma, ddr;
            int id;

#ifdef BSP_USING_SDHCI1
            id = 0;
            clk = rt_sdmmc_get_clock(id);
#ifdef SD_DMA_MODE
            dma = SD_DMA_MODE;
#else
            dma = 0;
#endif

#else   // SDHCI2
            id = 1;
            clk = rt_sdmmc_get_clock(id);
#ifdef SD2_DMA_MODE
            dma = SD2_DMA_MODE;
#else
            dma = 0;
#endif

#endif
            struct sdhci_host *host = &sdhci_ctx[id];
            rt_uint32_t *tbuf = (rt_uint32_t *)(0x62200000);//buf;
            total = 0x1000000;
            len = 65536; //4096;
            blk = len >> 9;
            addr = addr >> 9;
            // initial write data
            if (value == 0)
            {
                for (i = 0; i < len / 4; i++)
                    *(tbuf + i) = (i << 16) | (i + 1);
            }
            else
            {
                for (i = 0; i < len / 4; i++)
                    *(tbuf + i) = value;
            }
#ifdef SD_MMC_DDR_SUPPORT
            ddr = 1;
#else
            ddr = 0;
#endif

            LOG_I("SD max clk %d, DMA %d, DDR(MMC only) %d\n", clk, dma, ddr);
            fail = 0;
            start = rt_tick_get();
            for (i = 0; i < total / len; i++)
            {
                res = rt_device_write(dev, addr + i * blk, tbuf, blk);
                if (res <= 0)
                {
                    fail = 1;
                    LOG_I("write fail %d\n", res);
                    break;
                }
            }
            end = rt_tick_get();
            if (fail == 0)
            {
                LOG_I("Write %d kbytes use %d tick, speed %d kbps", total / 1024, end - start, (total / 1024 * 8 * 1000) / (end - start));
            }

            // test read
            fail = 0;
            start = rt_tick_get();
            for (i = 0; i < total / len; i++)
            {
                res = rt_device_read(dev, addr + i * blk, (void *)tbuf, blk);
                if (res <= 0)
                {
                    fail = 1;
                    LOG_I("read fail %d\n", res);
                    break;
                }
            }
            end = rt_tick_get();
            if (fail == 0)
            {
                LOG_I("Read %d Kbytes use %d tick, speed %d kbps", total / 1024, end - start, (total / 1024 * 8 * 1000) / (end - start));
            }

        }
        if (ret != -RT_EBUSY)
            rt_device_close(dev);
    }
    else
    {
        LOG_I("find device sd0 fail\n");
    }

    rt_free(buf);

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sdhci, __cmd_sdhci, Test hw sdhci);

//#if DRV_DEBUG
struct sd_cmds_stru
{
    const char *name;
    uint16_t index;
    uint16_t class;
    const char *othername;
};

static const struct sd_cmds_stru sd_cmds[] =
{
    {"GO_IDLE_STATE",       0, 1, NULL}, /* bc                          */
    {"SEND_OP_COND",        1, 1, NULL}, /* bcr  [31:0] OCR         R3  */
    {"ALL_SEND_CID",        2, 1, NULL}, /* bcr                     R2  */
    {"SET_RELATIVE_ADDR",   3, 1, NULL}, /* ac   [31:16] RCA        R1  */
    {"SET_DSR",             4, 1, NULL}, /* bc   [31:16] RCA            */
    {"SWITCH",              6, 1, "SD_APP_SET_BUS_WIDTH"}, /* ac   [31:0] See below   R1b */
    {"SELECT_CARD",         7, 1, NULL}, /* ac   [31:16] RCA        R1  */
    {"SEND_EXT_CSD",        8, 1, "SD_SEND_IF_COND"}, /* adtc                    R1  */
    {"SEND_CSD",            9, 1, NULL}, /* ac   [31:16] RCA        R2  */
    {"SEND_CID",            10, 1, NULL}, /* ac   [31:16] RCA        R2  */
    {"READ_DAT_UNTIL_STOP", 11, 1, NULL}, /* adtc [31:0] dadr        R1  */
    {"STOP_TRANSMISSION",   12, 1, NULL}, /* ac                      R1b */
    {"SEND_STATUS",         13, 1, NULL}, /* ac   [31:16] RCA        R1  */
    {"GO_INACTIVE_STATE",   15, 1, NULL}, /* ac   [31:16] RCA            */
    {"SPI_READ_OCR",        58, 1, NULL}, /* spi                  spi_R3 */
    {"SPI_CRC_ON_OFF",      59, 1, NULL}, /* spi  [0:0] flag      spi_R1 */

    /* class 2 */
    {"SET_BLOCKLEN",        16, 2, NULL}, /* ac   [31:0] block len   R1  */
    {"READ_SINGLE_BLOCK",   17, 2, NULL}, /* adtc [31:0] data addr   R1  */
    {"READ_MULTIPLE_BLOCK", 18, 2, NULL}, /* adtc [31:0] data addr   R1  */

    /* class 3 */
    {"WRITE_DAT_UNTIL_STOP", 20, 3, NULL}, /* adtc [31:0] data addr   R1  */

    /* class 4 */
    {"SET_BLOCK_COUNT",      23, 4, NULL}, /* adtc [31:0] data addr   R1  */
    {"WRITE_BLOCK",          24, 4, NULL}, /* adtc [31:0] data addr   R1  */
    {"WRITE_MULTIPLE_BLOCK", 25, 4, NULL}, /* adtc                    R1  */
    {"PROGRAM_CID",          26, 4, NULL}, /* adtc                    R1  */
    {"PROGRAM_CSD",          27, 4, NULL}, /* adtc                    R1  */

    /* class 6 */
    {"SET_WRITE_PROT",       28, 6, NULL}, /* ac   [31:0] data addr   R1b */
    {"CLR_WRITE_PROT",       29, 6, NULL}, /* ac   [31:0] data addr   R1b */
    {"SEND_WRITE_PROT",      30, 6, NULL}, /* adtc [31:0] wpdata addr R1  */

    /* class 5 */
    {"ERASE_GROUP_START",    35, 5, NULL},  /* ac   [31:0] data addr   R1  */
    {"ERASE_GROUP_END",      36, 5, NULL},  /* ac   [31:0] data addr   R1  */
    {"ERASE",                38, 5, NULL},  /* ac                      R1b */

    /* class 9 */
    {"FAST_IO",              39, 9, NULL}, /* ac   <Complex>          R4  */
    {"GO_IRQ_STATE",         40, 9, NULL}, /* bcr                     R5  */

    /* class 7 */
    {"LOCK_UNLOCK",          42, 7, NULL}, /* adtc                    R1b */

    /* class 8 */
    {"APP_CMD",              55, 8, NULL}, /* ac   [31:16] RCA        R1  */
    {"GEN_CMD",              56, 8, NULL}, /* adtc [0] RD/WR          R1  */


    /* SD commands                           type  argument     response */
    /* class 0 */
    /* This is basically the same command as for MMC with some quirks. */

    /* Application commands */
    {"SD_APP_SEND_NUM_WR_BLKS",  22, 0x10, NULL}, /* adtc                    R1  */
    {"SD_APP_OP_COND",       41, 0x10, NULL}, /* bcr  [31:0] OCR         R3  */
    {"SD_APP_SEND_SCR",      51, 0x10, NULL}, /* adtc                    R1  */

    /* SDIO commands                                          type  argument     response */
    {"SD_IO_SEND_OP_COND",   5, 0x11, NULL}, /* bcr  [23:0] OCR         R4  */
    {"SD_IO_RW_DIRECT",      52, 0x11, NULL}, /* ac   [31:0] See below   R5  */
    {"SD_IO_RW_EXTENDED",    53, 0x11, NULL}, /* adtc [31:0] See below   R5  */
    {NULL, 0, 0, NULL},                    /* End*/
};

const char *sd_cmd_name(int index)
{
    int i;
    for (i = 0;; i++)
    {
        if (sd_cmds[i].name == NULL)
            break;
        if (sd_cmds[i].index == index)
            return sd_cmds[i].name;
    }
    return "Unknown";
}
//#endif

#endif  // DRV_SDHCI_TEST 

#endif  // BSP_USING_SDHCI

