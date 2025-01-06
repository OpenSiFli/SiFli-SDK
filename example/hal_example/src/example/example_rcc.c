/**
  ******************************************************************************
  * @file   example_rcc.c
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

#ifdef HAL_RCC_MODULE_ENABLED
/*
    This example demo:
    1. Reset HCPU on-chip peripherals
    2. Switch system to use 48M external Crystal.
    3. Enable DLL1 to use 192M Hz
    4. Switch system clock to DLL1
*/


static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void aes_async_cbk(void)
{
    LOG_I("Aes complete\n");
}

#ifdef SOC_SF32LB55X
#define HCPU_RESET_MODULES  \
    (HPSYS_RCC_RSTR1_MAILBOX1|\
     HPSYS_RCC_RSTR1_USART2|\
     HPSYS_RCC_RSTR1_EZIP|\
     HPSYS_RCC_RSTR1_EPIC|\
     HPSYS_RCC_RSTR1_LCDC1|\
     HPSYS_RCC_RSTR1_I2S1|\
     HPSYS_RCC_RSTR1_I2S2|\
     HPSYS_RCC_RSTR1_CRC |\
     HPSYS_RCC_RSTR1_TRNG|\
     HPSYS_RCC_RSTR1_GPTIM1|\
     HPSYS_RCC_RSTR1_GPTIM2|\
     HPSYS_RCC_RSTR1_BTIM1|\
     HPSYS_RCC_RSTR1_BTIM2|\
     HPSYS_RCC_RSTR1_SPI1|\
     HPSYS_RCC_RSTR1_SPI2|\
     HPSYS_RCC_RSTR1_PSRAMC|\
     HPSYS_RCC_RSTR1_NNACC|\
     HPSYS_RCC_RSTR1_PDM1|\
     HPSYS_RCC_RSTR1_PDM2|\
     HPSYS_RCC_RSTR1_I2C1|\
     HPSYS_RCC_RSTR1_I2C2|\
     HPSYS_RCC_RSTR1_DSIHOST|\
     HPSYS_RCC_RSTR1_DSIPHY|\
     HPSYS_RCC_RSTR1_PTC1\
    )

#define HCPU_RESET_MODULES2  \
    (HPSYS_RCC_RSTR2_I2C3|\
     HPSYS_RCC_RSTR2_BUSMON1|\
     HPSYS_RCC_RSTR2_USBC|\
     HPSYS_RCC_RSTR2_SDMMC2|\
     HPSYS_RCC_RSTR2_SDMMC1|\
     HPSYS_RCC_RSTR2_QSPI3\
     )
#elif defined(SOC_SF32LB56X)
#define HCPU_RESET_MODULES  \
    (HPSYS_RCC_RSTR1_MAILBOX1|\
     HPSYS_RCC_RSTR1_USART2|\
     HPSYS_RCC_RSTR1_EZIP1|\
     HPSYS_RCC_RSTR1_EPIC|\
     HPSYS_RCC_RSTR1_LCDC1|\
     HPSYS_RCC_RSTR1_I2S1|\
     HPSYS_RCC_RSTR1_CRC1 |\
     HPSYS_RCC_RSTR1_TRNG|\
     HPSYS_RCC_RSTR1_GPTIM1|\
     HPSYS_RCC_RSTR1_GPTIM2|\
     HPSYS_RCC_RSTR1_BTIM1|\
     HPSYS_RCC_RSTR1_BTIM2|\
     HPSYS_RCC_RSTR1_SPI1|\
     HPSYS_RCC_RSTR1_SPI2|\
     HPSYS_RCC_RSTR1_NNACC1|\
     HPSYS_RCC_RSTR1_PDM1|\
     HPSYS_RCC_RSTR1_PDM2|\
     HPSYS_RCC_RSTR1_I2C1|\
     HPSYS_RCC_RSTR1_I2C2|\
     HPSYS_RCC_RSTR1_PTC1\
    )

#define HCPU_RESET_MODULES2  \
    (HPSYS_RCC_RSTR2_I2C3|\
     HPSYS_RCC_RSTR2_BUSMON1|\
     HPSYS_RCC_RSTR2_USBC|\
     HPSYS_RCC_RSTR2_SDMMC2|\
     HPSYS_RCC_RSTR2_SDMMC1|\
     HPSYS_RCC_RSTR2_BUSMON1|\
     HPSYS_RCC_RSTR2_ATIM1|\
     HPSYS_RCC_RSTR2_FFT1|\
     HPSYS_RCC_RSTR2_FACC1|\
     HPSYS_RCC_RSTR2_SCI|\
     HPSYS_RCC_RSTR2_CAN1|\
     HPSYS_RCC_RSTR2_AUDCODEC|\
     HPSYS_RCC_RSTR2_AUDPRC|\
     HPSYS_RCC_RSTR2_I2C4\
     )

#elif defined(SOC_SF32LB58X)
#define HCPU_RESET_MODULES  \
    (HPSYS_RCC_RSTR1_MAILBOX1|\
     HPSYS_RCC_RSTR1_USART2|\
     HPSYS_RCC_RSTR1_EZIP1|\
     HPSYS_RCC_RSTR1_EPIC|\
     HPSYS_RCC_RSTR1_LCDC1|\
     HPSYS_RCC_RSTR1_I2S1|\
     HPSYS_RCC_RSTR1_I2S2|\
     HPSYS_RCC_RSTR1_CRC1 |\
     HPSYS_RCC_RSTR1_TRNG|\
     HPSYS_RCC_RSTR1_GPTIM1|\
     HPSYS_RCC_RSTR1_GPTIM2|\
     HPSYS_RCC_RSTR1_BTIM1|\
     HPSYS_RCC_RSTR1_BTIM2|\
     HPSYS_RCC_RSTR1_SPI1|\
     HPSYS_RCC_RSTR1_SPI2|\
     HPSYS_RCC_RSTR1_NNACC1|\
     HPSYS_RCC_RSTR1_PDM1|\
     HPSYS_RCC_RSTR1_PDM2|\
     HPSYS_RCC_RSTR1_I2C1|\
     HPSYS_RCC_RSTR1_I2C2|\
     HPSYS_RCC_RSTR1_DSIHOST|\
     HPSYS_RCC_RSTR1_DSIPHY|\
     HPSYS_RCC_RSTR1_PTC1\
    )

#define HCPU_RESET_MODULES2  \
    (HPSYS_RCC_RSTR2_I2C3|\
     HPSYS_RCC_RSTR2_BUSMON1|\
     HPSYS_RCC_RSTR2_USBC|\
     HPSYS_RCC_RSTR2_SDMMC2|\
     HPSYS_RCC_RSTR2_SDMMC1|\
     HPSYS_RCC_RSTR2_JENC|\
     HPSYS_RCC_RSTR2_JDEC|\
     HPSYS_RCC_RSTR2_I2C4|\
     HPSYS_RCC_RSTR2_ACPU|\
     )
#elif defined(SOC_SF32LB52X)
#define HCPU_RESET_MODULES  \
    (HPSYS_RCC_RSTR1_MAILBOX1|\
     HPSYS_RCC_RSTR1_USART2|\
     HPSYS_RCC_RSTR1_EZIP1|\
     HPSYS_RCC_RSTR1_EPIC|\
     HPSYS_RCC_RSTR1_LCDC1|\
     HPSYS_RCC_RSTR1_I2S1|\
     HPSYS_RCC_RSTR1_CRC1 |\
     HPSYS_RCC_RSTR1_TRNG|\
     HPSYS_RCC_RSTR1_GPTIM1|\
     HPSYS_RCC_RSTR1_GPTIM2|\
     HPSYS_RCC_RSTR1_BTIM1|\
     HPSYS_RCC_RSTR1_BTIM2|\
     HPSYS_RCC_RSTR1_SPI1|\
     HPSYS_RCC_RSTR1_SPI2|\
     HPSYS_RCC_RSTR1_PDM1|\
     HPSYS_RCC_RSTR1_I2C1|\
     HPSYS_RCC_RSTR1_I2C2|\
     HPSYS_RCC_RSTR1_PTC1\
    )

#define HCPU_RESET_MODULES2  \
    (HPSYS_RCC_RSTR2_I2C3|\
     HPSYS_RCC_RSTR2_USBC|\
     HPSYS_RCC_RSTR2_SDMMC1|\
     HPSYS_RCC_RSTR2_I2C4\
     )
#endif
static void testcase(int argc, char **argv)
{
    HAL_StatusTypeDef ret;

    LOG_I("Reset peripherals on HCPU");
    HAL_RCC_HCPU_reset(HCPU_RESET_MODULES, 1);                          // Reset HCPU on-chip peripherals
#ifdef SF32LB55X
    HAL_RCC_HCPU_reset2(HCPU_RESET_MODULES2, 1);                          // Reset HCPU on-chip peripherals
#endif
    HAL_HPAON_EnableXT48();
    HAL_RCC_HCPU_reset(HCPU_RESET_MODULES, 0);
#ifdef SF32LB55X
    HAL_RCC_HCPU_reset2(HCPU_RESET_MODULES2, 0);                          // Reset HCPU on-chip peripherals
#endif

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);      // Switch system to use 48M external Crystal.
    ret = HAL_RCC_HCPU_EnableDLL1(192000000);                         // Enable DLL1 to use 192M Hz
    RT_ASSERT(HAL_OK == ret);
    LOG_I("Set system clock to use clock of DLL1 192M");
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);         // Switch system clock to DLL1
    LOG_I("Now system clock is 192M");
}

UTEST_TC_EXPORT(testcase, "example_rcc", utest_tc_init, utest_tc_cleanup, 10);
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
