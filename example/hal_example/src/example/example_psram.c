/**
  ******************************************************************************
  * @file   example_psram.c
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

#include <string.h>
#include <stdlib.h>
#include "utest.h"
#include "bf0_hal.h"

#ifdef HAL_PSRAM_MODULE_ENABLED

static PSRAM_HandleTypeDef psram_Handle = {0};

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}



static void testcase(int argc, char **argv)
{
    uint32_t *pSrc = (uint32_t *)PSRAM_BASE;
    uint32_t *pDst = (uint32_t *)(PSRAM_BASE + 2 * 1024);

    psram_Handle.Instance = hwp_psramc;

#ifdef BSP_USING_DUAL_PSRAM
#ifndef BSP_USING_XCCELA_PSRAM
#error "XCCELA must be enabled for Dual PSRAM"
#endif

    psram_Handle.dual_psram = true;
#else
    psram_Handle.dual_psram = false;
#endif

#ifdef BSP_USING_XCCELA_PSRAM
    psram_Handle.is_xccela = true;
#else
    psram_Handle.is_xccela = false;
#endif

    HAL_PSRAM_Init(&psram_Handle);

    for (int m = 0; m < 200; m++)
    {
        pSrc[m] = m + 100;
    }

    memcpy(pDst, pSrc, 200 * sizeof(uint32_t));
    uassert_buf_equal(pSrc, pDst, 200 * sizeof(uint32_t));

    for (int m = 0; m < 200; m++)
    {
        if (pDst[m] !=  m + 100)
        {
            uassert_true(false);
        }
    }


}

UTEST_TC_EXPORT(testcase, "example_psram", utest_tc_init, utest_tc_cleanup, 10);

#elif defined(HAL_MPI_MODULE_ENABLED)

static FLASH_HandleTypeDef local_psramctx;

// mpi2 use psram, or change it

static rt_err_t utest_tc_init(void)
{
    FLASH_HandleTypeDef *handle;
    uint32_t sys_clk;
    qspi_configure_t qspi_cfg;
    uint8_t r_lat, w_lat;
    uint32_t mode = SPI_MODE_OPSRAM;

    // psram defined by macro and has been initialized?
    if (PSRAM_SIZE > 0)
    {
        local_psramctx.base = PSRAM_BASE;
        local_psramctx.size = PSRAM_SIZE;
        return RT_EOK;
    }

    rt_memset(&qspi_cfg, 0, sizeof(qspi_configure_t));

    handle = &local_psramctx;
#if defined(BSP_ENABLE_QSPI2) && ((BSP_QSPI2_MODE==SPI_MODE_OPSRAM) || (BSP_QSPI2_MODE==SPI_MODE_HPSRAM))
    qspi_cfg.Instance = hwp_qspi2;
    qspi_cfg.SpiMode = BSP_QSPI2_MODE;
    qspi_cfg.msize = BSP_QSPI2_MEM_SIZE;
    qspi_cfg.base = QSPI2_MEM_BASE + HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET;

#if (BSP_QSPI2_MODE==SPI_MODE_OPSRAM)
    mode = SPI_MODE_OPSRAM;
#elif (BSP_QSPI2_MODE==SPI_MODE_HPSRAM)
    mode = SPI_MODE_HPSRAM;
#else
    mode = SPI_MODE_PSRAM;
#endif

#elif defined(BSP_ENABLE_QSPI1) && \
    (BSP_QSPI1_MODE==SPI_MODE_OPSRAM||BSP_QSPI1_MODE==SPI_MODE_HPSRAM||BSP_QSPI1_MODE==SPI_MODE_LEGPSRAM)
    qspi_cfg.Instance = hwp_qspi1;
    qspi_cfg.SpiMode = BSP_QSPI1_MODE;
    qspi_cfg.msize = BSP_QSPI1_MEM_SIZE;
    qspi_cfg.base = QSPI1_MEM_BASE + HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET;
#if (BSP_QSPI1_MODE==SPI_MODE_OPSRAM)
    mode = SPI_MODE_OPSRAM;
#elif (BSP_QSPI1_MODE==SPI_MODE_HPSRAM)
    mode = SPI_MODE_HPSRAM;
#elif (BSP_QSPI1_MODE==SPI_MODE_LEGPSRAM)
    mode = SPI_MODE_LEGPSRAM;
#else
    mode = SPI_MODE_PSRAM;
#endif

#else
    //HAL_ASSERT(0);  // NO PSRAM ENABLED
    // or set a default, do not care result?
    qspi_cfg.Instance = hwp_qspi2;
    qspi_cfg.SpiMode = SPI_MODE_OPSRAM; //SPI_MODE_PSRAM;
    qspi_cfg.msize = 4;
    qspi_cfg.base = QSPI2_MEM_BASE + HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET;
    mode = SPI_MODE_OPSRAM; //SPI_MODE_PSRAM;
#endif
    uint32_t fix_lat = 1;
    HAL_StatusTypeDef res;

    if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
    {
        handle->wakeup = 1;
    }
    else
    {
        handle->wakeup = 0;
    }

    if (mode == SPI_MODE_LEGPSRAM)
        res = HAL_LEGACY_PSRAM_Init(handle, &qspi_cfg, 1);
    else
        res = HAL_OPI_PSRAM_Init(handle, &qspi_cfg, 1);
    RT_ASSERT(HAL_OK == res);

    if (mode == SPI_MODE_OPSRAM)
    {
        HAL_MPI_MR_WRITE(handle, 8, 3);
    }
    else if (mode == SPI_MODE_HPSRAM)
    {
        HAL_MPI_MR_WRITE(handle, 8, 0x43);
        HAL_FLASH_SET_X16_MODE(handle, 1);
    }
    else if (mode == SPI_MODE_LEGPSRAM)
    {
        HAL_LEGACY_CFG_READ(handle);
        HAL_LEGACY_CFG_WRITE(handle);
        return RT_EOK;
    }
    else
    {
        // 4 line or legacy?
        HAL_MPI_MR_WRITE(handle, 8, 3);
    }


    // TODO: modify read/write delay to meed psram frequency
    sys_clk = HAL_QSPI_GET_CLK(handle);
    sys_clk /= 2;
    if (sys_clk < 66 * 1000000)
        w_lat = 3;
    else if (sys_clk < 109 * 1000000)
        w_lat = 4;
    else if (sys_clk < 133 * 1000000)
        w_lat = 5;
    else if (sys_clk < 166 * 1000000)
        w_lat = 6;
    else if (sys_clk < 200 * 1000000)
        w_lat = 7;
    else
        ASSERT(0);

    if (fix_lat)
        r_lat = w_lat * 2; //10;
    else
        r_lat = w_lat; // = 6; //5;

    /* configure AHB command */
    HAL_FLASH_CFG_AHB_RCMD(handle, 7, r_lat - 1, 0, 0, 3, 7, 7);
    HAL_FLASH_SET_AHB_RCMD(handle, 0);  //OPSRAM_RD   = 0x00,   // Read
    HAL_FLASH_CFG_AHB_WCMD(handle, 7, w_lat - 1, 0, 0, 3, 7, 7);
    HAL_FLASH_SET_AHB_WCMD(handle, 0x80);   //OPSRAM_WR   = 0x80,   // Write

    HAL_MPI_SET_FIXLAT(handle, fix_lat, r_lat, w_lat);

    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void testcase(int argc, char **argv)
{
    uint32_t *pSrc = (uint32_t *)(local_psramctx.base + 0x100000);
    uint32_t *pDst = (uint32_t *)(local_psramctx.base + 0x100000 + 2 * 1024);

    //HAL_OPI_PSRAM_Init();
    for (int m = 0; m < 200; m++)
    {
        pSrc[m] = m + 100;
    }

    memcpy(pDst, pSrc, 200 * sizeof(uint32_t));
    uassert_buf_equal(pSrc, pDst, 200 * sizeof(uint32_t));

    for (int m = 0; m < 200; m++)
    {
        if (pDst[m] !=  m + 100)
        {
            uassert_true(false);
        }
    }
    rt_kprintf("finish psram test case\n");

}

UTEST_TC_EXPORT(testcase, "example_psram", utest_tc_init, utest_tc_cleanup, 10);

#endif  //HAL_PSRAM_MODULE_ENABLED

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
