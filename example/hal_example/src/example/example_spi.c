/**
  ******************************************************************************
  * @file   example_spi.c
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

#if defined(HAL_SPI_MODULE_ENABLED) && !defined(SOC_SF32LB52X)

/*
    This example demo:
        1. USE SPI1 CONTROL lsm6dsl sensor for example.
           The sensor bord should be connected to the EVB bord.
        2. For A0 and Z0 evb, it may connect to different SPI and pin mux set.
*/


#define LSM6DSL_ID            0x6AU
#define LSM6DSL_WHO_AM_I      0x0FU

static SPI_HandleTypeDef spi_Handle = {0};


static void spi2_test_init(int lineNum, int mode)  //mode 0:sync mode    mode 1: INT mode   mode 2: DMA mode
{

}


static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void gpio_set(uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_PIN_Set(PAD_PB00 + pin, GPIO_B0 + pin, PIN_PULLDOWN, 0);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio2, &GPIO_InitStruct);

    HAL_GPIO_WritePin(hwp_gpio2, pin, 1);
}


static void testcase(int argc, char **argv)
{
    uint32_t baundRate = 6000000;
    uint16_t cmd = ((uint16_t)LSM6DSL_WHO_AM_I | 0x80) << 8;   //SPI_DATASIZE_16BIT
    uint16_t pid = 0;
    HAL_StatusTypeDef ret;

    //----------------------------------------------
    // 1. pin mux

    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_PULLUP, 0);             // SPI3_EN
    HAL_PIN_Set(PAD_PB13, SPI3_CLK, PIN_NOPULL, 0);             // SPI3
    HAL_PIN_Set(PAD_PB16, SPI3_DO, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB19, SPI3_DI, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB23, SPI3_CS, PIN_NOPULL, 0);

    //gpio_set(30);

    //----------------------------------------------
    // 2. spi init

    spi_Handle.Instance = SPI3;
    spi_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    spi_Handle.Init.Mode = SPI_MODE_MASTER;
    spi_Handle.Init.DataSize = SPI_DATASIZE_16BIT;
    spi_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi_Handle.Init.BaudRatePrescaler = (HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 1) + baundRate / 2) / baundRate;
    spi_Handle.Init.FrameFormat = SPI_FRAME_FORMAT_SPI;
    spi_Handle.Init.SFRMPol = SPI_SFRMPOL_HIGH;
    spi_Handle.State = HAL_SPI_STATE_RESET;
    if (HAL_SPI_Init(&spi_Handle) != HAL_OK)
    {
        uassert_false(RT_TRUE);
        return;
    }

    //----------------------------------------------
    // 3.1. spi sync rtx
    ret = HAL_SPI_TransmitReceive(&spi_Handle, (uint8_t *)&cmd, (uint8_t *)&pid, 1, 1000);
    uassert_true(ret == HAL_OK);
    pid &= 0xff;

    LOG_I("get pid %d, expect %d", pid, LSM6DSL_ID);
    uassert_true(pid == LSM6DSL_ID);

}

/*PARAM DESCRIPTION:
    see enum ARG_IDX ahead
*/
UTEST_TC_EXPORT(testcase, "example_spi", utest_tc_init, utest_tc_cleanup, 10);

#endif /* HAL_SPI_MODULE_ENABLED */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
