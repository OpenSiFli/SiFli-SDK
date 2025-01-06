/**
  ******************************************************************************
  * @file   example_lptimer.c
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
#include "board.h"
#include "bf0_hal_lptim.h"
#include "bf0_hal_lrc_cal.h"
#include "tc_utils.h"

#ifdef HAL_LPTIM_MODULE_ENABLED
/*
    This example demo:
        1. Using LPTIM1 (hwp_lptim1) to generate a one-time timeout interrupt in 3.5 seconds
*/

static LPTIM_HandleTypeDef LPTIM_Handle = {0};

static HAL_StatusTypeDef lp_timer_init()
{
    HAL_StatusTypeDef r = HAL_OK;

    LPTIM_Handle.Instance = hwp_lptim1;
    LPTIM_Handle.Mode = HAL_LPTIM_ONESHOT;
    LPTIM_Handle.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV2; // Set prescaler divider, freqency is LXT_FREQ/2
    if (HAL_LPTIM_Init(&LPTIM_Handle) != HAL_OK)
        r = HAL_ERROR;
    else
    {
        HAL_NVIC_SetPriority(LPTIM1_IRQn, 3, 0);    /* set the TIMx priority */
        HAL_NVIC_EnableIRQ(LPTIM1_IRQn);            /* enable the TIMx global Interrupt */
    }

    return r;
}

void LPTIM1_IRQHandler(void)
{
    ENTER_INTERRUPT();
    LOG_I("LPTIM1 timeout");
    HAL_LPTIM_IRQHandler(&LPTIM_Handle);
    LEAVE_INTERRUPT();
}

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
    lp_timer_init();
    LOG_I("Start LPTIMER for 3.5 seconds");

    HAL_LPTIM_Counter_Start_IT(&LPTIM_Handle, 3500 * (HAL_LPTIM_GetFreq() / 2) / 1000); // Set counter, timeout in 3.5seconds
    rt_thread_mdelay(4000);
}

UTEST_TC_EXPORT(testcase, "example_lptimer", utest_tc_init, utest_tc_cleanup, 30);
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
