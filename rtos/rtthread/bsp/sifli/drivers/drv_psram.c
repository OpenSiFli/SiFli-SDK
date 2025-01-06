/**
  ******************************************************************************
  * @file   drv_psram.c
  * @author Sifli software development team
  * @brief PSRAM Controller BSP driver
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
#include "module_record.h"
/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_psram PSRAM
  * @brief PSRAM Controller BSP driver
  This driver is validated by using MSH command 'date'.
  * @{
  */

#if defined(BSP_USING_PSRAM) || defined(_SIFLI_DOXYGEN_)

#include "drv_config.h"
#include "drv_psram.h"
#include "string.h"
#include <stdlib.h>
#include "drv_io.h"

#ifdef RT_USING_ULOG
    //#define DRV_DEBUG
    #define LOG_TAG                "drv.psram"
    #include <drv_log.h>
#endif


/* -----------------output apis --------------------------------*/

/**
* @brief  psrame initial.
* @retval 0.
*/
int rt_psram_init(void)
{
    return bsp_psramc_init();
}

uint32_t rt_psram_get_clk(uint32_t addr)
{
    return bsp_psram_get_clk(addr);
}

int rt_psram_enter_low_power(char *name)
{
    int res;
    sifli_record_module(RECORD_PSRAM_ENTER_LOW_POWER_BEGIN);

    res = bsp_psram_enter_low_power(name);

    sifli_record_psram_half_status(1);
    sifli_record_module(RECORD_PSRAM_ENTER_LOW_POWER_END);

    return res;
}

int rt_psram_deep_power_down(char *name)
{
    int res;
    sifli_record_module(RECORD_PSRAM_EXIT_LOW_POWER_BEGIN);

    res = bsp_psram_deep_power_down(name);

    sifli_record_psram_half_status(0);
    sifli_record_module(RECORD_PSRAM_EXIT_LOW_POWER_END);

    return res;
}

int rt_psram_exit_low_power(char *name)
{
    int res;
    sifli_record_module(RECORD_PSRAM_EXIT_LOW_POWER_BEGIN);

    res = bsp_psram_exit_low_power(name);

    sifli_record_psram_half_status(0);
    sifli_record_module(RECORD_PSRAM_EXIT_LOW_POWER_END);

    return res;
}

int rt_psram_set_pasr(char *name, uint8_t top, uint8_t deno)
{
    return bsp_psram_set_pasr(name, top, deno);
}

int rt_psram_auto_calib(char *name, uint8_t *sck, uint8_t *dqs)
{
    int res;
    int level = rt_hw_interrupt_disable();
    res = bsp_psram_auto_calib(name, sck, dqs);
    rt_hw_interrupt_enable(level);

    return res;
}

void rt_psram_wait_idle(char *name)
{
    bsp_psram_wait_idle(name);
}


//#define DRV_PSRAM_TEST
#ifdef DRV_PSRAM_TEST
#define TEST_VALUE          (0x5a5aa5a5)
#define EXT_DMA_IT
#include "drv_ext_dma.h"

#ifndef RT_USING_ULOG
    #include "rtdbg.h"
#endif

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

int cmd_psram(int argc, char *argv[])
{
    rt_uint32_t value, addr, loop, i;
    rt_uint32_t buf[256];
    rt_uint32_t *ptr;

    if (strcmp(argv[1], "-cp") == 0)
    {
        rt_uint32_t src, dst, size;

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            src = strtoul(argv[2], 0, 16);
        else
            src = strtoul(argv[2], 0, 10);

        if ((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
            dst = strtoul(argv[3], 0, 16);
        else
            dst = strtoul(argv[3], 0, 10);

        if ((strncmp(argv[4], "0x", 2) == 0) || (strncmp(argv[4], "0X", 2) == 0))
            size = strtoul(argv[4], 0, 16);
        else
            size = strtoul(argv[4], 0, 10);

        rt_uint32_t start = rt_tick_get();
        memcpy((void *)dst, (void *)src, size);
        rt_uint32_t end = rt_tick_get();
        if (end > start)
            LOG_I("memcpy %d data use %d tick, speed %d KBps\n", size, end - start, size / (end - start));
        for (i = 0; i < size / 4; i++)
            if (*(uint32_t *)(src + 4 * i) != *(uint32_t *)(dst + 4 * i))
            {
                LOG_I("Copy compare fail at pos %d, src 0x%x, dst 0x%x\n", i, src + 4 * i, dst + 4 * i);
                LOG_I(" src 0x%x, dst 0x%x\n", *(uint32_t *)(src + 4 * i), *(uint32_t *)(dst + 4 * i));
                break;
            }
        if (i >= size / 4)
            LOG_I("COPY DONE\n");
        else
            LOG_I("COPY FAIL\n");

    }
    else if (strcmp(argv[1], "-cp2") == 0)
    {
        rt_uint32_t src, dst, size;

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            src = strtoul(argv[2], 0, 16);
        else
            src = strtoul(argv[2], 0, 10);

        if ((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
            dst = strtoul(argv[3], 0, 16);
        else
            dst = strtoul(argv[3], 0, 10);

        if ((strncmp(argv[4], "0x", 2) == 0) || (strncmp(argv[4], "0X", 2) == 0))
            size = strtoul(argv[4], 0, 16);
        else
            size = strtoul(argv[4], 0, 10);

        rt_uint32_t *src_ptr = (rt_uint32_t *)src;
        rt_uint32_t *dst_ptr = (rt_uint32_t *)dst;
        rt_uint32_t start = rt_tick_get();
        for (i = 0; i < size / 4; i++)
        {
            *dst_ptr = *src_ptr;
            dst_ptr++;
            src_ptr++;
        }
        rt_uint32_t end = rt_tick_get();
        if (end > start)
            LOG_I("word copy %d data use %d tick, speed %d KBps\n", size, end - start, size / (end - start));

        src_ptr = (rt_uint32_t *)src;
        dst_ptr = (rt_uint32_t *)dst;
        for (i = 0; i < size / 4; i++)
        {
            if (*src_ptr != *dst_ptr)
            {
                LOG_I("Copy compare fail at pos %d, src %p, dst %p\n", i, src_ptr, dst_ptr);
                LOG_I(" src 0x%x, dst 0x%x\n", *src_ptr, *dst_ptr);
                break;
            }
            src_ptr++;
            dst_ptr++;
        }
        if (i >= size / 4)
            LOG_I("COPY DONE\n");
        else
            LOG_I("COPY FAIL\n");

    }
    else if (strcmp(argv[1], "-read") == 0)
    {
        rt_uint32_t dst, size;

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            dst = strtoul(argv[2], 0, 16) & 0xfffffffc;
        else
            dst = strtoul(argv[2], 0, 10) & 0xfffffffc;

        if ((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
            size = strtoul(argv[3], 0, 16);
        else
            size = strtoul(argv[3], 0, 10);

        rt_uint32_t *dst_ptr = (rt_uint32_t *)dst;
        for (i = 0; i < size / 4; i++)
        {
            rt_kprintf("0x%08x ", *dst_ptr);
            dst_ptr++;
            if ((i & 7) == 7)
                rt_kprintf("\n");
        }
    }
    else if (strcmp(argv[1], "-write") == 0)
    {
        rt_uint32_t dst, size;

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            dst = strtoul(argv[2], 0, 16) & 0xfffffffc;
        else
            dst = strtoul(argv[2], 0, 10) & 0xfffffffc;

        if ((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
            value = strtoul(argv[3], 0, 16);
        else
            value = strtoul(argv[3], 0, 10);

        if ((strncmp(argv[4], "0x", 2) == 0) || (strncmp(argv[4], "0X", 2) == 0))
            size = strtoul(argv[4], 0, 16);
        else
            size = strtoul(argv[4], 0, 10);

        rt_uint32_t *dst_ptr = (rt_uint32_t *)dst;
        for (i = 0; i < size / 4; i++)
        {
            *dst_ptr = value;
            //rt_kprintf("Addr %p, value 0x%08x\n", dst_ptr, value);
            dst_ptr++;
        }
        SCB_InvalidateDCache_by_Addr((void *)dst, size);
        SCB_InvalidateICache_by_Addr((void *)dst, size);
    }
    else if (!strcmp(argv[1], "-loop"))
    {
        rt_uint32_t data = TEST_VALUE;
        rt_uint32_t start, end, j, fail;
        rt_uint32_t test_base, test_length;

        uint32_t freq;

        test_base = PSRAM_BASE_ADDR + 0x200000;
        test_length = PSRAM_SIZE - 0x200000;
        freq = rt_psram_get_clk(PSRAM_SIZE);
        //test_base = HCPU_MPI_SBUS_ADDR(QSPI2_MEM_BASE);
        //test_length = FLASH2_SIZE; // * 0x100000;
        //freq = rt_psram_get_clk(test_base);

        loop = atoi(argv[2]);
        fail = 0;

        for (j = 0; j < loop; j++)
        {
            ptr = (rt_uint32_t *)(test_base);
            data = TEST_VALUE;
            start = rt_tick_get();
            data = start;
            if (j % 3 == 0) // word access
            {
                start = rt_tick_get();
                for (i = 0; i < test_length / 4; i++)
                    *(ptr + i) = data + i;
                end = rt_tick_get();
                LOG_I("write full chip to 0x%x use %d tick, speed %d KBps\n", data, end - start, test_length / (end - start));

                start = rt_tick_get();
                for (i = 0; i < test_length / 4; i++)
                    if (*(ptr + i) != data + i)
                    {
                        LOG_I("test fail with pos 0x%x, buf 0x%x, expect 0x%x\n", test_base + 4 * i, *(ptr + i), data + i);
                        break;
                    }
                end = rt_tick_get();
                if (i == test_length / 4)
                    LOG_I("FULL WORD 0x%08x read use %d tick, size %dMB, speed %d KBps with %d hz\n", test_base, end - start, test_length >> 20, test_length / (end - start), freq);
                else
                {
                    LOG_I("!!!!!!!!!!!!! psram test fail with WORD mode with %d hz !!!!!!!!!!!!!!!!!!!!!!!!\n", freq);
                    fail++;
                }
            }
            else if (j % 3 == 1) // half word access
            {
                uint16_t *ptr16 = (uint16_t *)(test_base);
                start = rt_tick_get();
                for (i = 0; i < test_length / 2; i++)
                    *(ptr16 + i) = (uint16_t)((data + i) & 0xffff);
                end = rt_tick_get();
                LOG_I("write full chip to 0x%x use %d tick, speed %d KBps\n", data & 0xffff, end - start, test_length / (end - start));

                start = rt_tick_get();
                for (i = 0; i < test_length / 2; i++)
                    if (*(ptr16 + i) != (uint16_t)((data + i) & 0xffff))
                    {
                        LOG_I("test fail with pos 0x%x, buf 0x%x, expect 0x%x\n", test_base + 2 * i, *(ptr16 + i), (uint16_t)((data + i) & 0xffff));
                        break;
                    }
                end = rt_tick_get();
                if (i == test_length / 2)
                    LOG_I("FULL HW read 0x%08x use %d tick, size %dMB, speed %d KBps with %d hz\n", test_base, end - start, test_length >> 20, test_length / (end - start), freq);
                else
                {
                    LOG_I("!!!!!!!!!!!!! psram test fail at HALF WORD mode with %d hz !!!!!!!!!!!!!!!!!!!!!!!!\n", freq);
                    fail++;
                }
            }
            else // byte access
            {
                uint8_t *ptr8 = (uint8_t *)(test_base);
                start = rt_tick_get();
                for (i = 0; i < test_length; i++)
                    *(ptr8 + i) = (uint8_t)((data + i) & 0xff);
                end = rt_tick_get();
                LOG_I("write full chip to 0x%x use %d tick, speed %d KBps\n", data & 0xff, end - start, test_length / (end - start));

                start = rt_tick_get();
                for (i = 0; i < test_length; i++)
                    if (*(ptr8 + i) != (uint8_t)((data + i) & 0xff))
                    {
                        LOG_I("test fail with pos 0x%x, buf 0x%x, expect 0x%x\n", test_base +  i, *(ptr8 + i), (uint8_t)((data + i) & 0xff));
                        break;
                    }
                end = rt_tick_get();
                if (i == test_length)
                    LOG_I("FULL BYTE read 0x%08x use %d tick, size %dMB, speed %d KBps with %d hz\n", test_base, end - start, test_length >> 20, test_length / (end - start), freq);
                else
                {
                    LOG_I("!!!!!!!!!!!!! psram test fail at BYTE mode with %d hz !!!!!!!!!!!!!!!!!!!!!!!!\n", freq);
                    fail++;
                }
            }

            LOG_I("TEST COUNTER %d , fail %d times\n", j + 1, fail);
        }
    }
    else if (!strcmp(argv[1], "-bid"))
    {
        uint32_t pid = (hwp_hpsys_cfg->IDR & HPSYS_CFG_IDR_PID) >> HPSYS_CFG_IDR_PID_Pos;
        rt_kprintf("pid = 0x%x\n", pid);
    }
    else if (!strcmp(argv[1], "-pasr"))
    {
        int top = atoi(argv[3]);
        int deno = atoi(argv[4]);
        int res = rt_psram_set_pasr(argv[2], top, deno);

        LOG_I("set PSRAM %s PASR with top %d, parg 1/%d res %d\n", argv[2], top, deno, res);
    }
    else if (argc >= 3)
    {
        for (i = 0; i < argc; i++)
            LOG_I("%s ", argv[i]);
        LOG_I("\n");

        // get parameters
        loop = 1;
        addr = atoi(argv[1]) & 0xfffffffc;
        value = atoi(argv[2]);
        if (argc >= 4)
            loop = atoi(argv[3]);
        // initial data
        ptr = (rt_uint32_t *)(PSRAM_BASE_ADDR + addr);
        if (loop > 256)
            loop = 256;
        for (i = 0; i < loop; i++)
            buf[i] = value + i;

        // write value or memory copy
        if (loop > 1)
        {
            memcpy(ptr, buf, loop * sizeof(rt_uint32_t));
        }
        else
        {
            *ptr = value;
        }

        // check result
        for (i = 0; i < loop; i++)
            if (*(ptr + i) != buf[i])
            {
                LOG_I("%d value fail: 0x%x : 0x%x\n", i, *(ptr + i), buf[i]);
                break;
            }
        if (i == loop)
            LOG_I("psram test pass\n");
        else
            LOG_I("psram test fail\n");

    }
    else if (argc == 2)
    {
        LOG_I("full chip test\n");
        if (!strcmp(argv[1], "-dma"))
        {
#ifdef PSRAM_USING_DMA

#ifdef EXT_DMA_IT
            // write from sram to psram, source(sram) use fixed address
            EXT_DMA_Config(0, 1);

            HAL_StatusTypeDef res = HAL_OK;
            rt_uint32_t data = rt_tick_get(); //0xdeadbeaf;
            addr = PSRAM_BASE_ADDR;
            endflag = 0;

            EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, psram_dma_done_cb);
            EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, psram_dma_err_cb);
            rt_uint32_t total = PSRAM_SIZE > 0x400000 ? 0x400000 : PSRAM_SIZE;      // EXTDMA max count 0x100000
            rt_uint32_t start = rt_tick_get();
            res = EXT_DMA_START_ASYNC((rt_uint32_t)(&data), addr, total / 4 - 1);
            if (res != 0)
            {
                LOG_I("EXT DMA START fail with %d\n", res);
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
                LOG_I("transfer done\n");
            else if (endflag == 2)
                LOG_I("transfer error\n");

            rt_uint32_t end = rt_tick_get();
            LOG_I("FULL write with DMA IT use %d tick, speed %d kbps\n", end - start, total * 8 / (end - start));

            // read from psram to sram, dst(sram) use fixed address
            EXT_DMA_Config(1, 0);

            addr = PSRAM_BASE_ADDR;
            data = 0;
            endflag = 0;

            EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, psram_dma_done_cb);
            EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, psram_dma_err_cb);

            start = rt_tick_get();
            res = EXT_DMA_START_ASYNC(addr, (rt_uint32_t)(&data), total / 4 - 1);
            if (res != HAL_OK)
            {
                LOG_I("EXT DMA start read fail with %d\n", res);
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
                LOG_I("transfer done\n");
            else if (endflag == 2)
                LOG_I("transfer error\n");

            end = rt_tick_get();
            LOG_I("FULL read with DMA IT use %d tick, size %dMB, speed %d KBps\n", end - start, total >> 20, total / (end - start));
            LOG_I("data = 0x%x\n", data);


#else
            // write from sram to psram, source(sram) use fixed address
            EXT_DMA_Config(0, 1);

            HAL_StatusTypeDef res = HAL_OK;
            rt_uint32_t data = rt_tick_get(); //0xdeadbeaf;
            addr = PSRAM_BASE_ADDR;
            rt_uint32_t total = PSRAM_SIZE > 0x400000 ? 0x400000 : PSRAM_SIZE;      // EXTDMA max count 0x100000

            rt_uint32_t start = rt_tick_get();
            res = EXT_DMA_TRANS_SYNC((rt_uint32_t)(&data), addr, total / 4 - 1, 8000);
            if (res != 0)
            {
                LOG_I("EXT DMA transfer fail with %d\n", res);
                return 1;
            }

            rt_uint32_t end = rt_tick_get();
            LOG_I("FULL write use %d tick\n", end - start);

            // read from psram to sram, dst(sram) use fixed address
            EXT_DMA_Config(1, 0);

            addr = PSRAM_BASE_ADDR;
            data = 0;

            start = rt_tick_get();
            res = EXT_DMA_TRANS_SYNC(addr, (rt_uint32_t)(&data), total / 4 - 1, 8000);
            if (res != HAL_OK)
            {
                LOG_I("EXT DMA start read fail with %d\n", res);
                return 1;
            }

            end = rt_tick_get();
            LOG_I("FULL read with DMA use %d tick, size %dMB, speed %d KBps\n", end - start, total >> 20, total  / (end - start));
            LOG_I("data = 0x%x\n", data);
#endif //EXT_DMA_IT
#else   // not defined PSRAM_USING_DMA
            LOG_I("DMA for psram not enable!\n");
#endif  // PSRAM_USING_DMA
        }
        else if (!strcmp(argv[1], "-a"))
        {
            ptr = (rt_uint32_t *)(PSRAM_BASE_ADDR);
            rt_uint32_t data = TEST_VALUE;
            rt_uint32_t start = rt_tick_get();
            data = start;
            for (i = 0; i < PSRAM_SIZE / 4; i++)
                *(ptr + i) = data + i;
            rt_uint32_t end = rt_tick_get();
            LOG_I("write full chip to 0x%x use %d tick, speed %d KBps\n", data, end - start, PSRAM_SIZE / (end - start));

            start = rt_tick_get();
            for (i = 0; i < PSRAM_SIZE / 4; i++)
                if (*(ptr + i) != data + i)
                {
                    LOG_I("test fail with pos 0x%x, buf 0x%x, expect 0x%x\n", PSRAM_BASE_ADDR + 4 * i, *(ptr + i), data + i);
                    break;
                }
            end = rt_tick_get();
            if (i == PSRAM_SIZE / 4)
                LOG_I("FULL read use %d tick, size %dMB, speed %d KBps\n", end - start, PSRAM_SIZE >> 20, PSRAM_SIZE / (end - start));
            else
                LOG_I("psram test fail\n");
        }
    }
    else
    {
        LOG_I("Invalid parameter:\n");
        LOG_I("-a for full chip test\n");
        LOG_I("-dma for full chip test with ext dma\n");
        LOG_I("2 parameters to address and value\n");
        LOG_I("3 parameters to address, value and test length\n");
    }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_psram, __cmd_psram, Test psram driver);

#endif  // DRV_PSRAM_TEST


#endif

/// @} drv_psram
/// @} bsp_driver

/** @addtogroup bsp_sample BSP driver sample commands.
  * @{
  */

/** @defgroup bsp_sample_psram psram sample commands
  * @brief FLASH sample commands
  *
  * FLASH driver could be tested with MSH command 'date',
  *
  * @{
  */
/// @} bsp_sample_psram
/// @} bsp_sample


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
