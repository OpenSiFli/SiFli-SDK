/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
#elif defined (SOC_SF32LB57X)
    HAL_CACHE_Enable(HAL_CACHE_DCACHE_MPI1, HAL_CACHE_DCACHE_MPI1);
    HAL_CACHE_Enable(HAL_CACHE_DCACHE_MPI2, HAL_CACHE_DCACHE_MPI2);
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
