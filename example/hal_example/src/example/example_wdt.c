/**
  ******************************************************************************
  * @file   example_wdt.c
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

#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"

#ifdef HAL_WDT_MODULE_ENABLED

static volatile uint32_t irq_count;
static uint32_t g_tmout;
static WDT_HandleTypeDef   WdtHandle;

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    __HAL_WDT_STOP(&WdtHandle);
    LOG_I("Stop WDT");
    return RT_EOK;
}

// WDT interrupt (only for hwp_wdt1/hwp_wdt2) is NMI_Interrupt, could not be masked.
void WDT_IRQHandler(void)
{
    irq_count++;
    LOG_I("IRQ Fired");
    __HAL_WDT_CLEAR(&WdtHandle);            // Clear interrupt will reset timer 1
}



static void testcase(int argc, char **argv)
{

    g_tmout = 3; //in seconds

    /* WDT handler declaration */
    HAL_StatusTypeDef   status;
    int i;
    int freq;

    /*##-1- Initialize RNG peripheral #######################################*/

    WdtHandle.Instance = hwp_wdt1;                          // HCPU use hwp_wdt1, LCPU use hwp_wdt2, both of them could use hwp_iwdt

#ifdef SOC_SF32LB55X
    if (HAL_PMU_LXT_ENABLED())
        freq = LXT_FREQ;
    else
#elif defined(SOC_SF32LB52X)
    if (HAL_PMU_LXT_ENABLED())
        freq = RC32K_FREQ;
    else
#endif
        freq = RC10K_FREQ;                                        // For 58x and 56x, WDT is always using RC10K

    // Counter is based on 32K clock/RC10K, g_tmout is in seconds
    // If use RC10K, LXT_FREQ should be around 8K-9K, depends on chipset, temperature etc, timeout might not be accurate.
    WdtHandle.Init.Reload = (uint32_t)g_tmout * freq;       // first Reload timeout will generate WDT interrupt is enabled (only apply to hwp_wdt1/hwp_wdt2).
    WdtHandle.Init.Reload2 = WdtHandle.Init.Reload * 2;     // Second Reload2 timeout after frist one will reset HCPU/LCPU if use hwp_wdt1/hwp_wdt2
    // reset whole system if use hwp_iwdt


    __HAL_WDT_STOP(&WdtHandle);                             // Stop previous watch dog
    while ((WdtHandle.Instance->WDT_SR & WDT_WDT_SR_WDT_ACTIVE) != 0);  // Make sure watch stop is done.
    if (HAL_WDT_Init(&WdtHandle) != HAL_OK)                 // Initializ watch dog.
    {
        /* Initialization Error */
        uassert_true(RT_FALSE);
    }

    // No interrupt generated
    __HAL_WDT_INT(&WdtHandle, 0);                           // Disable WDT interrupt for Reload timeout
    __HAL_WDT_START(&WdtHandle);                            // Start Watch dog
    for (i = 0; i < 5; i++)
    {
        rt_thread_mdelay(g_tmout * 1000 / 2);
        HAL_WDT_Refresh(&WdtHandle);                        // Pet watchdog
        LOG_I("Pet WDT");
    }
    uassert_true((WdtHandle.Instance->WDT_SR & WDT_WDT_SR_INT_ASSERT) == 0);        // Should not generate any interrupt
    __HAL_WDT_STOP(&WdtHandle);                             // Stop watchdog
    while ((WdtHandle.Instance->WDT_SR & WDT_WDT_SR_WDT_ACTIVE) != 0);  // Make sure watch stop is done.

    // Interrupt generate for first counter Init.Reload
    __HAL_WDT_INT(&WdtHandle, 1);                           // Enable  WDT interrupt for Reload timeout
    __HAL_WDT_START(&WdtHandle);
    irq_count = 0;
    for (i = 0; i < 5; i++)
    {
        LOG_I("Pet WDT");
        HAL_WDT_Refresh(&WdtHandle);
        rt_thread_mdelay(g_tmout * 1000 * 3 / 2); // Init.Reload < delay < Init.Reload2, will generate interrupt
    }
    LOG_I("IRQ: %d, expect 5", irq_count);
    __HAL_WDT_STOP(&WdtHandle);
    while ((WdtHandle.Instance->WDT_SR & WDT_WDT_SR_WDT_ACTIVE) != 0);  // Make sure watch stop is done.
    uassert_true(irq_count == 5);

    __HAL_WDT_PROTECT(&WdtHandle, 1);               // Pretect Watchdog
    uint32_t cnt = WdtHandle.Init.Reload;
    WdtHandle.Init.Reload = 0x345678;
    //WdtHandle.Init.Reload2 = 0x345678;
    __HAL_WDT_RELOAD_COUNTER(&WdtHandle);           // Reload will fail
    LOG_I("counter=%x", WdtHandle.Instance->WDT_CVR0);
    uassert_true(WdtHandle.Init.Reload != WdtHandle.Instance->WDT_CVR0);

    __HAL_WDT_PROTECT(&WdtHandle, 0);               // Un-protect watchdog
    WdtHandle.Init.Reload = 0x345678;
    //WdtHandle.Init.Reload2 = 0x345678;
    __HAL_WDT_RELOAD_COUNTER(&WdtHandle);           // Reload counter will success.
    LOG_I("counter 2=%x", WdtHandle.Instance->WDT_CVR0);
    uassert_true(WdtHandle.Init.Reload == WdtHandle.Instance->WDT_CVR0);
}

#ifndef HAL_USING_HTOL
    UTEST_TC_EXPORT(testcase, "example_wdt", utest_tc_init, utest_tc_cleanup, 10);
#endif


#endif /*HAL_CRC_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
