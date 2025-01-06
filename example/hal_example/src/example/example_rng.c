/**
  ******************************************************************************
  * @file   example_rng.c
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

#ifdef HAL_RNG_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define BUFFER_SIZE    114


static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void test_rng(void)
{
    /* RNG handler declaration */
    RNG_HandleTypeDef   RngHandle;
    HAL_StatusTypeDef   status;
    uint32_t            value;


    /*##-1- Initialize RNG peripheral #######################################*/
    RngHandle.Instance = hwp_trng;

    if (HAL_RNG_Init(&RngHandle) != HAL_OK)
    {
        /* Initialization Error */
        uassert_true(RT_FALSE);
    }

    /*##-2- Generate seed ###################################*/
    value = 0;
    status = HAL_RNG_Generate(&RngHandle, &value,  1);
    LOG_I("Generate seed value=%x, status=%d", value, status);
    uassert_true(value != 0);

    /*##-3- Generate random ###################################*/
    value = 0;
    status = HAL_RNG_Generate(&RngHandle, &value, 0);
    LOG_I("Generate Random value=%x, status=%d", value, status);
    uassert_true(value != 0);
}

static void testcase(int argc, char **argv)
{
    UTEST_UNIT_RUN(test_rng);
}


UTEST_TC_EXPORT(testcase, "example_rng", utest_tc_init, utest_tc_cleanup, 10);

#endif /*HAL_CRC_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
