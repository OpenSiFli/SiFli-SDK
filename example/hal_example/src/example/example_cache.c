/**
  ******************************************************************************
  * @file   example_cache.c
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
#include "tc_utils.h"


/* Example Description:
 *
 * 1. Read 32kbyte data from flash. Invalidate cache each time, loop for 10 times. D-Cache miss rate is same every time.
 * 2. Read 32kbyte data from flash. Don't invalidate cache, loop for 10 times.
 *    The first D-Cache miss rate is largest, others are smaller than the first one as some data is already in cache.
 */

#ifdef HAL_CACHE_MODULE_ENABLED

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
    uint32_t cnt;
    uint8_t *data;
    uint32_t rd_size;
    uint8_t *rd_addr;
    float irate;
    float drate;

#if defined(SOC_SF32LB55X)
    HAL_CACHE_Enable(HAL_CACHE_DCACHE_QSPI1_4, HAL_CACHE_DCACHE_QSPI1_4);
#elif defined (SOC_SF32LB52X)
    HAL_CACHE_Enable(HAL_CACHE_DCACHE_MPI1, HAL_CACHE_DCACHE_MPI1);
#else
    HAL_CACHE_Enable(HAL_CACHE_DCACHE_MPI5, HAL_CACHE_DCACHE_MPI5);
#endif

    cnt = 0;
    rd_size = 32 * 1024;
    rd_addr = (uint8_t *)FLASH_BASE_ADDR;
    data = rt_malloc(rd_size);
    if (data == NULL)
    {
        LOG_I("Malloc buffer fail with lenght %d\n", rd_size);
        return;
    }

    LOG_I("step1 %p", data);
    while (cnt < 10)
    {
        /* Invalidate D-Cache */
        SCB_InvalidateDCache_by_Addr(rd_addr, rd_size);
        HAL_CACHE_RESET();
        /* Flash data is not in cache */
        memcpy(data, rd_addr, rd_size);
        HAL_CACHE_GetMissRate(&irate, &drate, true);
        LOG_I("[%d]icache miss rate: %f \%, dcache miss rate: %f \%", cnt, irate, drate);
        rt_thread_mdelay(1000);
        cnt++;
    }

    cnt = 0;
    rd_addr = (uint8_t *)FLASH_BASE_ADDR;
    LOG_I("step2");
    SCB_InvalidateDCache_by_Addr(rd_addr, rd_size);
    while (cnt < 10)
    {
        HAL_CACHE_RESET();
        /* Some flash data is in cache after the first loop */
        memcpy(data, rd_addr, rd_size);
        HAL_CACHE_GetMissRate(&irate, &drate, true);
        LOG_I("[%d]icache miss rate: %f \%, dcache miss rate: %f \%", cnt, irate, drate);
        rt_thread_mdelay(1000);
        cnt++;
    }

    rt_free(data);
}

UTEST_TC_EXPORT(testcase, "example_cache", utest_tc_init, utest_tc_cleanup, 10);

#endif /*HAL_GPIO_MODULE_ENABLED*/
