/**
  ******************************************************************************
  * @file   drv_ext_dma.c
  * @author Sifli software development team
  * @brief EXT DMA Interrupt handler.
  *
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

#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include "drv_ext_dma.h"
#include "bf0_hal_ext_dma.h"


pCallback full_cb = NULL;
pCallback err_cb = NULL;

EXT_DMA_HandleTypeDef gExtDma = {0};

rt_sem_t ExtDma_sema = NULL;

static void EXT_DMA_CPLT_CB(EXT_DMA_HandleTypeDef *_hdma);
static void EXT_DMA_ERR_CB(EXT_DMA_HandleTypeDef *_hdma);


static int EXT_DMA_Init(void)
{
    if (!ExtDma_sema)
    {
        ExtDma_sema = rt_sem_create("drv_eDma", 1, 0);
        RT_ASSERT(ExtDma_sema != NULL);
    }

    return 0;
}

static void EXT_DMA_Lock(void)
{
    RT_ASSERT(ExtDma_sema != NULL);

    rt_err_t err;
    err = rt_sem_take(ExtDma_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
}

static void EXT_DMA_Unlock(void)
{
    RT_ASSERT(ExtDma_sema != NULL);

    rt_err_t err;
    err = rt_sem_release(ExtDma_sema);
    RT_ASSERT(RT_EOK == err);
}

/**
 * @brief EXT DMA Interrupt handler.
 */
void EXTDMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_EXT_DMA_IRQHandler(&gExtDma);

    /* leave interrupt */
    rt_interrupt_leave();
}


rt_err_t EXT_DMA_ConfigCmpr(uint8_t src_inc, uint8_t dst_inc, const EXT_DMA_CmprTypeDef *cmpr)
{
    rt_err_t res = 0;
    if (!cmpr)
    {
        return -RT_ERROR;
    }

    EXT_DMA_Lock();

    if (src_inc)
        gExtDma.Init.SrcInc = HAL_EXT_DMA_SRC_INC | HAL_EXT_DMA_SRC_BURST16;
    else
        gExtDma.Init.SrcInc = HAL_EXT_DMA_SRC_BURST1;

    if (dst_inc)
        gExtDma.Init.DstInc = HAL_EXT_DMA_DST_INC | HAL_EXT_DMA_DST_BURST16;
    else
        gExtDma.Init.DstInc = HAL_EXT_DMA_DST_BURST1;


    gExtDma.Init.cmpr_en = cmpr->cmpr_en;
    gExtDma.Init.src_format = cmpr->src_format;
    gExtDma.Init.cmpr_rate = cmpr->cmpr_rate;
    gExtDma.Init.col_num = cmpr->col_num;
    gExtDma.Init.row_num = cmpr->row_num;

    return res;
}

rt_err_t EXT_DMA_Config(uint8_t src_inc, uint8_t dst_inc)
{
    const EXT_DMA_CmprTypeDef cmpr_none =
    {
        .cmpr_en = false,
    };
    if (ExtDma_sema == NULL)
        return -RT_ERROR;
    return EXT_DMA_ConfigCmpr(src_inc, dst_inc, &cmpr_none);
}

rt_err_t EXT_DMA_START_ASYNC(uint32_t src, uint32_t dst, uint32_t len)
{
    HAL_StatusTypeDef res;


#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif  /* RT_USING_PM */

    /* reset extdma to make CMPRDR.MAXBUF is updated if overflow happens */
    HAL_RCC_ResetModule(RCC_MOD_EXTDMA);
    res = HAL_EXT_DMA_Init(&gExtDma);
    if (HAL_OK == res)
    {
        /* NVIC configuration for DMA transfer complete interrupt */
        HAL_NVIC_SetPriority(EXTDMA_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTDMA_IRQn);
        HAL_EXT_DMA_RegisterCallback(&gExtDma, HAL_EXT_DMA_XFER_CPLT_CB_ID, EXT_DMA_CPLT_CB);
        HAL_EXT_DMA_RegisterCallback(&gExtDma, HAL_EXT_DMA_XFER_ERROR_CB_ID, EXT_DMA_ERR_CB);

        res = HAL_EXT_DMA_Start_IT(&gExtDma, src, dst, len);
    }
    else
    {
        //Act like EXTDMA error: err irq + cmplt irq.
        EXT_DMA_ERR_CB(&gExtDma);
        EXT_DMA_CPLT_CB(&gExtDma);
    }

    return HAL_OK == res ?  RT_EOK : (rt_err_t)res;
}

rt_err_t EXT_DMA_TRANS_SYNC(uint32_t src, uint32_t dst, uint32_t len, uint32_t timeout)
{
    HAL_StatusTypeDef res = 0;

    /* reset extdma to make CMPRDR.MAXBUF is updated if overflow happens */
    HAL_RCC_ResetModule(RCC_MOD_EXTDMA);
    res = HAL_EXT_DMA_Init(&gExtDma);

    res = HAL_EXT_DMA_Start(&gExtDma, src, dst, len);
    if (HAL_OK == res)
    {
        res = HAL_EXT_DMA_PollForTransfer(&gExtDma, HAL_EXT_DMA_FULL_TRANSFER, timeout);
    }
    if (res != HAL_OK)
    {
        res = HAL_EXT_DMA_GetError(&gExtDma);
    }

    EXT_DMA_Unlock();

    return HAL_OK == res ?  RT_EOK : (rt_err_t)res;
}

uint32_t EXT_DMA_GetError(void)
{
    return HAL_EXT_DMA_GetError(&gExtDma);
}

void EXT_DMA_Register_Callback(EXT_DMA_CallbackIDTypeDef cid, pCallback cb)
{
    if (cid == EXT_DMA_XFER_CPLT_CB_ID)
    {
        full_cb = cb;
    }

    if (cid == EXT_DMA_XFER_ERROR_CB_ID)
    {
        err_cb = cb;
    }
}

void EXT_DMA_Wait_ASYNC_Done(void)
{
    EXT_DMA_Lock();
    EXT_DMA_Unlock();
}

static void EXT_DMA_CPLT_CB(EXT_DMA_HandleTypeDef *_hdma)
{
    rt_err_t err;

    HAL_NVIC_DisableIRQ(EXTDMA_IRQn);

#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_stop();
#endif  /* RT_USING_PM */

    if (full_cb)
        full_cb();

    EXT_DMA_Unlock();
}

static void EXT_DMA_ERR_CB(EXT_DMA_HandleTypeDef *_hdma)
{
#if 0//def RT_USING_PM  The CPLT_CB always come, although there are error occurs
    rt_pm_release(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_stop();
#endif  /* RT_USING_PM */
    if (err_cb)
        err_cb();
}


INIT_BOARD_EXPORT(EXT_DMA_Init);

//#define DRV_EXT_DMA_TEST
#ifdef DRV_EXT_DMA_TEST

#include "drv_flash.h"
#include "drv_psram.h"

#define EXT_DMA_IT

#define FLASH_TEST_ADDR         FLASH_BASE_ADDR
#define PSRAM_TEST_ADDR         PSRAM_BASE

#ifdef EXT_DMA_IT

static uint32_t endflag = 0;
void psram_dma_done_cb()
{
    endflag = 1;
    LOG_I("psram with ext dma interrupt done\n");
}

void psram_dma_err_cb()
{
    endflag = 2;
    LOG_I("psram with ext dma interrupt error\n");
}

#endif

/*****************************************************************
 ***** support memory to memory, it should include 4 cases ****
 *
 * 1. sram to psram
 * 2. psram to sram
 * 3. flash to sram
 * 4. flash to psram
******************************************************************/

void edma_help()
{
    LOG_I("*** edma test command parameter: ***\n");
    LOG_I("*** 1 ---- sram to psram, sram use fixed address\n");
    LOG_I("*** 2 ---- psram to sram, sram use fixed address\n");
    LOG_I("*** 3 ---- flash to sram, sram use fixed address\n");
    LOG_I("*** 4 ---- flash to psram, address auto increased\n");
    LOG_I("*** 5 ---- flash to psram without dma\n");
}

int cmd_edma(int argc, char *argv[])
{
    int cmd, i;
    rt_uint32_t start, end;
    if (argc >= 2)
    {
        cmd = atoi(argv[1]);
        switch (cmd)
        {
        case 1: // sram use fixed address, psram address increase auto
        {
            EXT_DMA_Config(0, 1);

            HAL_StatusTypeDef res = HAL_OK;
            rt_uint32_t sram_data = 0x5aa5a55a;
            rt_uint32_t psram_addr = PSRAM_TEST_ADDR;
            endflag = 0;

            EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, psram_dma_done_cb);
            EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, psram_dma_err_cb);
            start = rt_tick_get();
            res = EXT_DMA_START_ASYNC((rt_uint32_t)(&sram_data), psram_addr, 0x80000);
            if (res != 0)
            {
                LOG_I("EXT_DMA_START_ASYNC fail with %d\n", res);
                return 1;
            }
            i = 0;
            while (endflag == 0)
            {
                rt_thread_delay(10);
                i++;
                if (i > 1000)
                {
                    LOG_I("WRITE psram dma with it time out!\n");
                    break;
                }
            }
            if (endflag == 1)
            {
                LOG_I("transfer done\n");
                end = rt_tick_get();
                LOG_I("sram to psram with 0x%x data use %d tick, speed %d kbps\n", 0x80000 * 4, end - start, (0x80000 * 4 * 8) / (end - start));
            }
            else if (endflag == 2)
                LOG_I("transfer error\n");
            else
                LOG_I("tranfer timeout\n");
            break;
        }
        case 2: // psram address increased, to fixed sram address
        {
            EXT_DMA_Config(1, 0);

            HAL_StatusTypeDef res = HAL_OK;
            rt_uint32_t sram_data = 0xdeadbeaf;
            rt_uint32_t psram_addr = PSRAM_TEST_ADDR;
            endflag = 0;

            EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, psram_dma_done_cb);
            EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, psram_dma_err_cb);
            start = rt_tick_get();
            res = EXT_DMA_START_ASYNC(psram_addr, (rt_uint32_t)(&sram_data), 0x80000);
            if (res != 0)
            {
                LOG_I("EXT_DMA_START_ASYNC fail with %d\n", res);
                return 1;
            }
            i = 0;
            while (endflag == 0)
            {
                rt_thread_delay(10);
                i++;
                if (i > 1000)
                {
                    LOG_I("READ psram dma with it time out!\n");
                    break;
                }
            }
            if (endflag == 1)
            {
                LOG_I("transfer done\n");
                end = rt_tick_get();
                LOG_I("psram to sram with 0x%x data use %d tick, speed %d kbps\n", 0x80000 * 4, end - start, (0x80000 * 4 * 8) / (end - start));
            }
            else if (endflag == 2)
                LOG_I("transfer error\n");
            else
                LOG_I("tranfer timeout\n");
            break;
        }
        case 3: // flash read, flash address increased to fixed sram address
        {
            EXT_DMA_Config(1, 0);

            HAL_StatusTypeDef res = HAL_OK;
            rt_uint32_t sram_data = 0xdeadbeaf;
            rt_uint32_t flash_addr = FLASH_TEST_ADDR;
            rt_uint8_t tbuf[256];
            endflag = 0;

            // initial flash data to randam
            for (i = 0; i < 256; i++)
                tbuf[i] = (i + 0x3721 * rt_tick_get()) & 0xff;

            start = rt_tick_get();
            rt_flash_erase(0x10000000, 0x200000);
            end = rt_tick_get();
            LOG_I("Flash full chip erase used %d tick\n", end - start);
            for (i = 0; i < 0x200000 / 256; i++)
                rt_flash_write(flash_addr + i * 256, tbuf, 256);
            start = rt_tick_get();
            LOG_I("Flash full chip page write used %d tick, speed %d kbps\n", (start - end) * 2, (0x200000 * 8) / (start - end));

            EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, psram_dma_done_cb);
            EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, psram_dma_err_cb);
            start = rt_tick_get();
            res = EXT_DMA_START_ASYNC(flash_addr, (rt_uint32_t)(&sram_data), 0x80000);
            if (res != 0)
            {
                LOG_I("EXT_DMA_START_ASYNC fail with %d\n", res);
                return 1;
            }
            i = 0;
            while (endflag == 0)
            {
                rt_thread_delay(10);
                i++;
                if (i > 1000)
                {
                    LOG_I("READ flash dma with it time out!\n");
                    break;
                }
            }
            if (endflag == 1)
            {
                LOG_I("transfer done\n");
                end = rt_tick_get();
                LOG_I("flash to sram with 0x%x data use %d tick, speed %d kbps\n", 0x80000 * 4, end - start, (0x80000 * 4 * 8) / (end - start));
                // data check
                LOG_I("src 0x%02x%02x%02x%02x\n", tbuf[255], tbuf[254], tbuf[253], tbuf[252]);
                LOG_I("dst 0x%08x\n", sram_data);
            }
            else if (endflag == 2)
                LOG_I("transfer error\n");
            else
                LOG_I("tranfer timeout\n");
            break;
        }
        case 4: // flash to psram, all address auto increased
        {
            EXT_DMA_Config(1, 1);

            HAL_StatusTypeDef res = HAL_OK;
            rt_uint32_t psram_data = PSRAM_TEST_ADDR;
            rt_uint32_t flash_addr = FLASH_TEST_ADDR;
            rt_uint8_t tbuf[256];
            endflag = 0;

            // initial flash data to randam
            for (i = 0; i < 256; i++)
                tbuf[i] = (i + 0x3721 * rt_tick_get()) & 0xff;

            start = rt_tick_get();
            rt_flash_erase(0x10000000, 0x200000);
            end = rt_tick_get();
            LOG_I("Flash full chip erase used %d tick\n", end - start);
            for (i = 0; i < 0x200000 / 256; i++)
                rt_flash_write(flash_addr + i * 256, tbuf, 256);
            start = rt_tick_get();
            LOG_I("Flash full chip page write used %d tick, speed %d kbps\n", (start - end) * 2, (0x200000 * 8) / (start - end));

            EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, psram_dma_done_cb);
            EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, psram_dma_err_cb);
            start = rt_tick_get();
            res = EXT_DMA_START_ASYNC(flash_addr, psram_data, 0x80000);
            if (res != 0)
            {
                LOG_I("EXT_DMA_START_ASYNC fail with %d\n", res);
                return 1;
            }
            i = 0;
            while (endflag == 0)
            {
                rt_thread_delay(10);
                i++;
                if (i > 1000)
                {
                    LOG_I("Flash to Psram time out!\n");
                    break;
                }
            }
            if (endflag == 1)
            {
                LOG_I("transfer done\n");
                end = rt_tick_get();
                LOG_I("flash to psram with 0x%x data use %d tick, speed %d kbps\n", 0x80000 * 4, end - start, (0x80000 * 4 * 8) / (end - start));
                // data check
                rt_uint32_t *fptr = (rt_uint32_t *)FLASH_TEST_ADDR;
                rt_uint32_t *pptr = (rt_uint32_t *)PSRAM_TEST_ADDR;
                start = rt_tick_get();
                for (i = 0; i < 0x80000; i++)
                {
                    if (*(fptr + i) != *(pptr + i))
                    {
                        LOG_I("data check fail at pos %d: 0x%x vs 0x%s\n", i, *(fptr + i), *(pptr + i));
                        break;
                    }
                }
                end = rt_tick_get();
                if (i == 0x80000)
                    LOG_I("flash to psram data check pass, flash/psram read speed %d kbps\n", (0x80000 * 4 * 8) / (end - start));
            }
            else if (endflag == 2)
                LOG_I("transfer error\n");
            else
                LOG_I("tranfer timeout\n");
            break;
        }
        case 5:
        {
            HAL_StatusTypeDef res = HAL_OK;
            //rt_uint32_t psram_data = PSRAM_TEST_ADDR;
            rt_uint32_t flash_addr = FLASH_TEST_ADDR;
            rt_uint32_t *fptr = (rt_uint32_t *)FLASH_TEST_ADDR;
            rt_uint32_t *pptr = (rt_uint32_t *)PSRAM_TEST_ADDR;
            rt_uint8_t tbuf[256];

            // initial flash data to randam
            for (i = 0; i < 256; i++)
                tbuf[i] = (i + 0x3721 * rt_tick_get()) & 0xff;

            rt_flash_erase(0x10000000, 0x200000);
            for (i = 0; i < 0x200000 / 256; i++)
                rt_flash_write(flash_addr + i * 256, tbuf, 256);
            start = rt_tick_get();
            for (i = 0; i < 0x80000; i++)
                *(pptr + i) = *(fptr + i);
            end = rt_tick_get();
            LOG_I("CPU: flash to psram with 0x%x data use %d tick, speed %d kbps\n", 0x80000 * 4, end - start, (0x80000 * 4 * 8) / (end - start));
            break;
        }
        default:
            LOG_I("Invalid parameter %s\n", argv[1]);
            edma_help();
        }
    }
    else
    {
        LOG_I("Invalid parameter\n");
        edma_help();
    }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_edma, __cmd_edma, Test ext_dma driver);


#endif  // DRV_EXT_DMA_TEST

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
