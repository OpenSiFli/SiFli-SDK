/**
  ******************************************************************************
  * @file   example_busmon.c
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

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"
#include "tc_utils.h"

#if defined(HAL_BUSMON_MODULE_ENABLED)&&!defined(BSP_USING_BUSMON)

/*
    This example demo:
     - Monitor the bus activity for PSRAM Access, get total access times.
     - When access times reach half of count expected, generate PTC interrupt
     - PTC will clear the OF flag.
*/

static BUSMON_HandleTypeDef   BusmonHandle;
static PTC_HandleTypeDef      PtcHandle;
static int g_count = 0;
static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

void PTC1_IRQHandler(void)
{
    ENTER_INTERRUPT();
    HAL_PTC_IRQHandler(&PtcHandle);
    rt_kprintf("PTC1 Interrupt at %d\n", g_count);
    LEAVE_INTERRUPT();
}

#define TOTAL_STEPS 128

static void test_busmon(void)
{
    uint32_t address, min_addr = 0x60200000, max_addr = 0x60201000;
    int i;
    volatile uint32_t temp = 0x5AA55AA5;

#if defined(SOC_SF32LB58X)||defined(SOC_SF32LB56X)
    HAL_RCC_EnableModule(RCC_MOD_PTC1);
    HAL_RCC_EnableModule(RCC_MOD_BUSMON1);
#endif

#ifdef BSP_USING_BOARD_EC_LB567XXX
    min_addr = 0x60a00000;
    max_addr = 0x60a01000;
#endif

#ifdef BSP_USING_BOARD_EC_LB585XXX
    min_addr = 0x62a00000;
    max_addr = 0x62a01000;
#endif
    /*##-1- Initialize Bus Monitor #######################################*/
    BusmonHandle.Instance = hwp_busmon1;                        // HCPU using busmon 1
    BusmonHandle.Init.Flags = BUSMON_OPFLAG_RW;                 // Monitor for both read  and write access

#ifdef SOC_SF32LB58X
    BusmonHandle.Init.Channel = HAL_BUSMON_CHN8;                // Use Channel 8 available for MPI1
    BusmonHandle.Init.SelFunc = HAL_BUSMON_MPI1;                // Monitor for MPI1
#elif defined(SOC_SF32LB56X)
    BusmonHandle.Init.Channel = HAL_BUSMON_CHN5;                // Use Channel 5 available for MPI1
    BusmonHandle.Init.SelFunc = HAL_BUSMON_MPI1;                // Monitor for MPI1
#else
    BusmonHandle.Init.Channel = HAL_BUSMON_CHN5;                // Use Channel 5 available for OPSRAM
    BusmonHandle.Init.SelFunc = HAL_BUSMON_OPSRAM;              // Monitor for OPSRAM
#endif
    BusmonHandle.Init.Max = max_addr;                           // Address range max
    BusmonHandle.Init.Min = min_addr;                           // Address range min
    BusmonHandle.Init.count = TOTAL_STEPS / 2;                  // Count to generate interrupt

    if (HAL_BUSMON_Init(&BusmonHandle) != HAL_OK)               // Initialize Busmon
    {
        /* Initialization Error */
        uassert_true(RT_FALSE);
    }

    PtcHandle.Instance = hwp_ptc1;
    PtcHandle.Init.Channel = 0;                                 // Use PTC Channel 0
    PtcHandle.Init.Address = (uint32_t) & (hwp_busmon1->CCR);   // Bus monitor clear register

#ifdef SOC_SF32LB58X
    PtcHandle.Init.data = (1 << HAL_BUSMON_CHN8);               // data to handle with value in Address.
    PtcHandle.Init.Sel = PTC_HCPU_BUSMON1_OF8;                  // Busmon OF channel 8.
#else
    PtcHandle.Init.data = (1 << HAL_BUSMON_CHN5);               // data to handle with value in Address.
    PtcHandle.Init.Sel = PTC_HCPU_BUSMON1_OF5;                  // Busmon OF channel 5.
#endif
    PtcHandle.Init.Operation = PTC_OP_OR;                       // PTC Or and write back bus monitor register(clear bus monitor event)
    NVIC_EnableIRQ(PTC1_IRQn);
    if (HAL_PTC_Init(&PtcHandle) != HAL_OK)                     // Initialize PTC
    {
        /* Initialization Error */
        uassert_true(RT_FALSE);
    }
    NVIC_EnableIRQ(PTC1_IRQn);
    HAL_PTC_Enable(&PtcHandle, 1);                              // Enable PTC

    HAL_BUSMON_Enable(&BusmonHandle, 1);                        // Enable busmon

    for (address = min_addr, g_count = 1; g_count <= TOTAL_STEPS; g_count++)       // Access PSRAM TOTAL_STEPS times
    {
        address = (address + 3) & 0xFFFFFFFC;
        if (g_count % 2)
            temp = *(uint32_t *)address;                        // Read in odd time
        else
            *(uint32_t *)address = temp;                        // Write in  even time
        address += (max_addr - min_addr) / TOTAL_STEPS;
        rt_thread_delay(1);                                     // Delay for interrupt generation before g_count increase.
    }

    HAL_BUSMON_GetCount(&BusmonHandle, (int32_t *)&temp);       // Get total counts
    LOG_I("Total count=%d", temp);
    HAL_BUSMON_Enable(&BusmonHandle, 0);                        // Disable busmon
    // uassert_true(temp == 0);                                 // PSRAM now have cache, total counts could be non-zero.

#if defined(SOC_SF32LB58X)||defined(SOC_SF32LB56X)
    HAL_RCC_DisableModule(RCC_MOD_PTC1);
    HAL_RCC_DisableModule(RCC_MOD_BUSMON1);
#endif
}

static void testcase(int argc, char **argv)
{
    UTEST_UNIT_RUN(test_busmon);
}

UTEST_TC_EXPORT(testcase, "example_busmon", utest_tc_init, utest_tc_cleanup, 10);

#endif /*HAL_BUSMON_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
