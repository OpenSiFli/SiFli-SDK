/**
  ******************************************************************************
  * @file   app_pm.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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

#include "app_common.h"
#include "mem_section.h"
#define LOG_TAG "app_pm"
#include "log.h"

static void pm_shutdown_config(void)
{
#ifdef SOC_BF0_HCPU
#if defined(SF32LB52X)

    //rtc:  0:PE disable; 1: PE enable; PA35~PA44 Enable PE, pull down; PA34 disable pull down;
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_NOPULL, 1);
    for (int i = PAD_PA35; i <= PAD_PA44; i++)
    {
        HAL_PIN_Set(PAD_PA35 + i, GPIO_A35 + i, PIN_PULLDOWN, 1);
    }
    //Hibernate Wakeup support PIN0 and PIN1;
#define PIN0    0
#define PIN1    1
#if defined(BSP_KEY1_PIN) &&  (BSP_KEY1_PIN != -1)
    HAL_PMU_SelectWakeupPin(PIN0, HAL_HPAON_QueryWakeupPin(hwp_gpio1, BSP_KEY1_PIN)); //PIN0 select:PA34 as pin0  PA24=>0 PA30=>6  PA34=>10  PA35=>11
    HAL_PMU_EnablePinWakeup(PIN0, AON_PIN_MODE_HIGH);  //PIN0 wake mode: AS 0: high level
#endif
    HAL_PMU_EnableRtcWakeup();
#endif
#endif
}

L1_RET_CODE_SECT(pm_shutdown, void pm_shutdown(void))
{
    rt_kprintf("%s, hibernate", __func__);
    pm_shutdown_config();
    HAL_Delay_us(3000000);  /**/
    rt_hw_interrupt_disable();
#if defined(SF32LB52X)
    HAL_PMU_ConfigPeriLdo(PMUC_PERI_LDO_EN_VDD33_LDO2_Pos, false, false);
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8, false, false);
#endif
    HAL_PMU_EnterHibernate();
    while (1);
}
#ifdef RT_USING_FINSH
static int cmd_pwr_off(int argc, char **argv)
{
    pm_shutdown();
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwr_off, __cmd_pwr_off, power off);
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

