/**
  ******************************************************************************
  * @file   example_tsen.c
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
#include "tc_utils.h"

#ifdef HAL_TSEN_MODULE_ENABLED

/*
    This example demo:
        1. Read temperature in sync mode
        2. Read temperature in Async mode
*/

static TSEN_HandleTypeDef   TsenHandle;

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    HAL_TSEN_DeInit(&TsenHandle);
    LOG_I("Stop TSEN");
    return RT_EOK;
}

void TSEN_IRQHandler(void)
{
    ENTER_INTERRUPT();
    LOG_I("IRQ Fired");
    HAL_TSEN_IRQHandler(&TsenHandle);
    LEAVE_INTERRUPT();
}


static void testcase(int argc, char **argv)
{
    HAL_StatusTypeDef   status;
    int temperature;

    /*##-1- Initialize TSEN peripheral #######################################*/
    TsenHandle.Instance = hwp_tsen;
    if (HAL_TSEN_Init(&TsenHandle) == HAL_OK)
    {
        temperature = HAL_TSEN_Read(&TsenHandle);                                   /* Read synchronized*/
        LOG_I("Sync: Current temperature is %d degree\n", temperature);

        HAL_NVIC_SetPriority(TSEN_IRQn, 5, 0);                                      /* Set interrupt priority*/
        if (HAL_TSEN_Read_IT(&TsenHandle) == HAL_TSEN_STATE_BUSY)                   /* Read Async, interrupt will be enabled*/
            while (HAL_TSEN_GetState(&TsenHandle) != HAL_TSEN_STATE_READY);
        LOG_I("Async: Current temperature is %d degree\n", TsenHandle.temperature);
        uassert_true((TsenHandle.temperature > -40) && TsenHandle.temperature < 125);
    }
}

UTEST_TC_EXPORT(testcase, "example_tsen", utest_tc_init, utest_tc_cleanup, 10);

#endif /*HAL_CRC_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
