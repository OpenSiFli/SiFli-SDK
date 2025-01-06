/**
  ******************************************************************************
  * @file   example_lpcomp.c
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

#ifdef HAL_COMP_MODULE_ENABLED

/*
    This example demo:
        1. Configure lpcomp parameters
            InternalVRef for internal Voltage reference , support 0.6, 1.2, 1.8. 2.4 v
            InvertingInput for compare with internal or external reference voltage
            TriggerMode for triger interrupt mode, polling mode do not use it
            WorkingPin for channel
        2. Polling lpcomp value or check voltage changed
           Note : for interrupt mode, only compare result status changed can triger interrupt !!

NOTE:  for eh-ss6600_551, J0301 and J0303 need to be disconnected, which ensure PB01 and PB03 are floating
*/

static int irq_flag = 0;
static COMP_HandleTypeDef hcomp;

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}


void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    /* Prevent unused argument(s) compilation warning */
    //UNUSED(hcomp);
    irq_flag = 1;
    LOG_I("Get comp result %d\n", HAL_COMP_GetOutputLevel(hcomp));
}

void LPCOMP_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    //LOG_I("LPCOMP1_IRQHandler\n");

    HAL_COMP_IRQHandler(&hcomp);

    /* leave interrupt */
    rt_interrupt_leave();
}

static void testcase(int argc, char **argv)
{
    COMP_ConfigTypeDef cfg;
    hcomp.Instance = hwp_lpcomp;
    hcomp.ErrorCode = 0;
    hcomp.Lock = 0;
    hcomp.State = 0;
#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
    hcomp.MspDeInitCallback = NULL;
    hcomp.MspInitCallback = NULL;
    hcomp.TriggerCallback = NULL;
#endif
    hcomp.Init.Mode = COMP_POWERMODE_HIGHSPEED;
    hcomp.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
    hcomp.Init.InvertingInput = COMP_INPUT_MINUS_VREF;
    hcomp.Init.Hysteresis = COMP_HYSTERESIS_NONE;
    hcomp.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
    hcomp.Init.InternalVRef = COMP_VREFINT_0D6V;
    hcomp.Init.WorkingPin = 0;

    HAL_StatusTypeDef res = HAL_COMP_Init(&hcomp);
    uassert_true_ret(res == HAL_OK);

    // config
    cfg.InternalVRef = COMP_VREFINT_0D6V;
    cfg.InvertingInput = COMP_INPUT_MINUS_VREF;
    cfg.Mode = COMP_POWERMODE_HIGHSPEED;
    cfg.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
#ifdef BSP_USING_BOARD_EC_LB557XXX
    cfg.WorkingPin = 1;
#else //BSP_USING_BOARD_EC_LB555XXX
    cfg.WorkingPin = 0;
#endif  //
    HAL_COMP_Config(&hcomp, &cfg);
    uassert_true_ret(res == HAL_OK);

    /* get comp result with polling mode */
    //res = HAL_COMP_PollForComp(&hcomp, 0, 1000);
    //LOG_I("ADC reg value %d\n", res);

    /* Get comp result with interrupt , only voltage change can triger it */
    irq_flag = 0;
    res = HAL_COMP_Start(&hcomp);
    uassert_true_ret(res == HAL_OK);
    HAL_NVIC_SetPriority(LPCOMP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPCOMP_IRQn);

    rt_thread_mdelay(5000);

    uassert_true_ret(irq_flag == 1);

    HAL_COMP_Stop(&hcomp);

}


UTEST_TC_EXPORT(testcase, "example_lpcomp", utest_tc_init, utest_tc_cleanup, 10);

#endif /*HAL_COMP_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
