/**
  ******************************************************************************
  * @file   bf0_hal_rcc.c
  * @author Sifli software development team
  * @brief   RCC HAL module driver.
  *          This file provides firmware functions to manage the following
  ******************************************************************************
*/
/**
 *
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

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup RCC Reset and control
  * @brief RCC HAL module driver
  * @{
  */

#if defined(HAL_RCC_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#define DLL_STG_STEP    (24000000)
#ifdef SF32LB55X
    #define DLL_MAX_FREQ    (288000000)
#else
    /*in_div2=1,out_div2=0 */
    #define DLL_MIN_FREQ    (24000000)
    #define DLL_MAX_FREQ    (384000000)
#endif /* SF32LB55X */
#define DLL_DIV2_MAX     192000000


#if defined(SF32LB52X) && defined(SOC_BF0_HCPU)

#define HPSYS_DVFS_MODE_D0_LIMIT (24)
#define HPSYS_DVFS_MODE_D1_LIMIT (48)
#define HPSYS_DVFS_MODE_S0_LIMIT (144)
#define HPSYS_DVFS_MODE_S1_LIMIT (240)

typedef struct
{
    int8_t ldo_offset;
    uint8_t ldo;
    uint8_t buck;
    uint32_t ulpmcr;
} HPSYS_DvfsConfigTypeDef;

const static HPSYS_DvfsConfigTypeDef hpsys_dvfs_config[HPSYS_DVFS_MODE_NUM] =
{
    //                         LDO,  BUCK,  ULPMCR
    //                         0.9V, 1.0V
    [HPSYS_DVFS_MODE_D0] = {-5, 0x6, 0x9, 0x00100330},
    //                         1.0V, 1.1V
    [HPSYS_DVFS_MODE_D1] = {-3, 0x8, 0xA, 0x00110331},
    //                         1.1V, 1.25V
    [HPSYS_DVFS_MODE_S0] = {0,  0xB, 0xD, 0x00130213},
    //                         1.2V, 1.35V
    [HPSYS_DVFS_MODE_S1] = {2,  0xD, 0xF, 0x00130213},
};

/* maximum DLL2 frequency(Hz) of each dvfs mode */
const static uint32_t hpsys_dll2_limit[HPSYS_DVFS_MODE_NUM] =
{
    [HPSYS_DVFS_MODE_D0] = 0,
    [HPSYS_DVFS_MODE_D1] = 0,
    [HPSYS_DVFS_MODE_S0] = 288000000,
    [HPSYS_DVFS_MODE_S1] = 288000000,
};

//TODO: will restored to default S0 mode when wakeup from standby
static HPSYS_DvfsModeTypeDef curr_dvfs_mode = HPSYS_DVFS_MODE_S0;
static uint32_t curr_freq_in_mhz;

#endif /* SF32LB52X && SOC_BF0_HCPU */


#if defined(SF32LB52X) && defined(SOC_BF0_LCPU)

#define LPSYS_DVFS_MODE_D_LIMIT (24)
#define LPSYS_DVFS_MODE_S_LIMIT (48)

typedef struct
{
    uint8_t ldo;
    uint8_t buck;
    uint32_t ulpmcr;
} LPSYS_DvfsConfigTypeDef;

const static LPSYS_DvfsConfigTypeDef lpsys_dvfs_config[LPSYS_DVFS_MODE_NUM] =
{
    //                     LDO, BUCK,  ULPMCR
    //                     0.9V, 1.0V
    [LPSYS_DVFS_MODE_D] = {0x6, 0x9, 0x00120330},
    //                     1.0V, 1.1V
    [LPSYS_DVFS_MODE_S] = {0x8, 0xA, 0x00120331},
};

//TODO: will restored to default D mode when wakeup from standby
static LPSYS_DvfsModeTypeDef curr_dvfs_mode = LPSYS_DVFS_MODE_D;
static uint32_t curr_freq_in_mhz;

#endif /* SF32LB52X && SOC_BF0_HCPU */



__STATIC_INLINE uint32_t HAL_RCC_DisableInterrupt(void)
{
    uint32_t mask;

    mask = __get_PRIMASK();
    __set_PRIMASK(1);
    return mask;
}

__STATIC_INLINE void HAL_RCC_EnableInterrupt(uint32_t mask)
{
    __set_PRIMASK(mask);
}

#ifdef SF32LB55X
static inline __IO uint32_t *RCC_GetModuleMask(RCC_MODULE_TYPE module, uint32_t *group, uint32_t *mask)
{
    __IO uint32_t *base_rcc = 0;
    if ((!mask) || (!group))
    {
        return NULL;
    }
    base_rcc = NULL;
    switch (module)
    {
#if 1//(CORE_ID_CURRENT == CORE_ID_HCPU)
    case RCC_MOD_DMAC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_DMAC1;
        break;
    case RCC_MOD_MAILBOX1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_MAILBOX1;
        break;
    case RCC_MOD_PINMUX1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PINMUX1;
        break;
    case RCC_MOD_USART1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_USART1;
        break;
    case RCC_MOD_USART2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_USART2;
        break;
    case RCC_MOD_EZIP:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EZIP;
        break;
    case RCC_MOD_EPIC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EPIC;
        break;
    case RCC_MOD_LCDC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_LCDC1;
        break;
    case RCC_MOD_I2S1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2S1;
        break;
    case RCC_MOD_I2S2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2S2;
        break;
    case RCC_MOD_SYSCFG1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SYSCFG1;
        break;
    case RCC_MOD_EFUSEC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EFUSEC;
        break;
    case RCC_MOD_AES:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_AES;
        break;
    case RCC_MOD_CRC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_CRC;
        break;
    case RCC_MOD_TRNG:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_TRNG;
        break;
    case RCC_MOD_GPTIM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_GPTIM1;
        break;
    case RCC_MOD_GPTIM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_GPTIM2;
        break;
    case RCC_MOD_BTIM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_BTIM1;
        break;
    case RCC_MOD_BTIM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_BTIM2;
        break;
    case RCC_MOD_WDT1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_WDT1;
        break;
    case RCC_MOD_SPI1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SPI1;
        break;
    case RCC_MOD_SPI2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SPI2;
        break;
    case RCC_MOD_EXTDMA:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EXTDMA;
        break;
    case RCC_MOD_PSRAMC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PSRAMC;
        break;
    case RCC_MOD_NNACC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_NNACC;
        break;
    case RCC_MOD_PDM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PDM1;
        break;
    case RCC_MOD_PDM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PDM2;
        break;
    case RCC_MOD_I2C1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2C1;
        break;
    case RCC_MOD_I2C2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2C2;
        break;
    case RCC_MOD_DSIHOST:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_DSIHOST;
        break;
    case RCC_MOD_DSIPHY:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_DSIPHY;
        break;
    case RCC_MOD_PTC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PTC1;
        break;

    case RCC_MOD_GPIO1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_GPIO1;
        break;
    case RCC_MOD_QSPI1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_QSPI1;
        break;
    case RCC_MOD_QSPI2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_QSPI2;
        break;
    case RCC_MOD_QSPI3:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_QSPI3;
        break;
    case RCC_MOD_SDMMC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_SDMMC1;
        break;
    case RCC_MOD_SDMMC2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_SDMMC2;
        break;
    case RCC_MOD_USBC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_USBC;
        break;
    case RCC_MOD_BUSMON1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_BUSMON1;
        break;
    case RCC_MOD_I2C3:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_I2C3;
        break;
#endif
#if 1//(CORE_ID_CURRENT == CORE_ID_LCPU)
    case RCC_MOD_BUSMON2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BUSMON2;
        break;
    case RCC_MOD_LCDC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_LCDC2;
        break;
    case RCC_MOD_PTC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PTC2;
        break;
    case RCC_MOD_TSEN:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_TSEN;
        break;
    case RCC_MOD_LPCOMP:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_LPCOMP;
        break;
    case RCC_MOD_SDADC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SDADC;
        break;
    case RCC_MOD_GPADC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPADC;
        break;
    case RCC_MOD_WDT2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_WDT2;
        break;
    case RCC_MOD_BTIM4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BTIM4;
        break;
    case RCC_MOD_BTIM3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BTIM3;
        break;
    case RCC_MOD_GPTIM5:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPTIM5;
        break;
    case RCC_MOD_GPTIM4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPTIM4;
        break;
    case RCC_MOD_GPTIM3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPTIM3;
        break;
    case RCC_MOD_SYSCFG2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SYSCFG2;
        break;
    case RCC_MOD_I2C6:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_I2C6;
        break;
    case RCC_MOD_I2C5:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_I2C5;
        break;
    case RCC_MOD_I2C4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_I2C4;
        break;
    case RCC_MOD_SPI4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SPI4;
        break;
    case RCC_MOD_SPI3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SPI3;
        break;
    case RCC_MOD_USART5:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_USART5;
        break;
    case RCC_MOD_USART4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_USART4;
        break;
    case RCC_MOD_USART3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_USART3;
        break;
    case RCC_MOD_PATCH:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PATCH;
        break;
    case RCC_MOD_PINMUX2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PINMUX2;
        break;
    case RCC_MOD_MAILBOX2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_MAILBOX2;
        break;
    case RCC_MOD_DMAC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_DMAC2;
        break;

    case RCC_MOD_MAC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_MAC;
        break;
    case RCC_MOD_PHY:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_PHY;
        break;
    case RCC_MOD_RFC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_RFC;
        break;
    case RCC_MOD_QSPI4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_QSPI4;
        break;
    case RCC_MOD_GPIO2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_GPIO2;
        break;
#endif
    default:
        HAL_ASSERT(0);
        break;

    }

    return base_rcc;
}
#elif defined(SF32LB52X)
//TODO: need optimize, replaced by macro
static inline __IO uint32_t *RCC_GetModuleMask(RCC_MODULE_TYPE module, uint32_t *group, uint32_t *mask)
{
    __IO uint32_t *base_rcc = 0;
    if ((!mask) || (!group))
    {
        return NULL;
    }
    base_rcc = NULL;
    switch (module)
    {
#if (CORE_ID_CURRENT == CORE_ID_HCPU)
    case RCC_MOD_DMAC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_DMAC1;
        break;
    case RCC_MOD_MAILBOX1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_MAILBOX1;
        break;
    case RCC_MOD_PINMUX1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PINMUX1;
        break;
    case RCC_MOD_USART2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_USART2;
        break;
    case RCC_MOD_EZIP:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EZIP1;
        break;
    case RCC_MOD_EPIC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EPIC;
        break;
    case RCC_MOD_LCDC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_LCDC1;
        break;
    case RCC_MOD_I2S1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2S1;
        break;
    case RCC_MOD_SYSCFG1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SYSCFG1;
        break;
    case RCC_MOD_EFUSEC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EFUSEC;
        break;
    case RCC_MOD_AES:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_AES;
        break;
    case RCC_MOD_CRC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_CRC1;
        break;
    case RCC_MOD_CRC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR1_CRC2;
        break;
    case RCC_MOD_TRNG:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_TRNG;
        break;
    case RCC_MOD_GPTIM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_GPTIM1;
        break;
    case RCC_MOD_GPTIM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_GPTIM2;
        break;
    case RCC_MOD_BTIM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_BTIM1;
        break;
    case RCC_MOD_BTIM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_BTIM2;
        break;
    case RCC_MOD_SPI1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SPI1;
        break;
    case RCC_MOD_SPI2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SPI2;
        break;
    case RCC_MOD_EXTDMA:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EXTDMA;
        break;
    case RCC_MOD_PDM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PDM1;
        break;
    case RCC_MOD_I2C1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2C1;
        break;
    case RCC_MOD_I2C2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2C2;
        break;
    case RCC_MOD_PTC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PTC1;
        break;

    case RCC_MOD_GPIO1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_GPIO1;
        break;
    case RCC_MOD_MPI1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_MPI1;
        break;
    case RCC_MOD_MPI2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_MPI2;
        break;
    case RCC_MOD_SDMMC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_SDMMC1;
        break;
    case RCC_MOD_USBC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_USBC;
        break;
    case RCC_MOD_I2C3:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_I2C3;
        break;
    case RCC_MOD_ATIM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_ATIM1;
        break;
    case RCC_MOD_USART3:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_USART3;
        break;
    case RCC_MOD_AUDCODEC_HP:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_AUDCODEC;
        break;
    case RCC_MOD_AUDPRC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_AUDPRC;
        break;
    case RCC_MOD_I2C4:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_I2C4;
        break;
    case RCC_MOD_TSEN:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_TSEN;
        break;
    case RCC_MOD_AUDCODEC_LP:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_AUDCODEC;
        break;
    case RCC_MOD_GPADC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_GPADC;
        break;
    case RCC_MOD_SECU1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SECU1;
        break;
    case RCC_MOD_AUDCODEC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_AUDCODEC;
        break;
#endif
#if 1//(CORE_ID_CURRENT == CORE_ID_LCPU)
    case RCC_MOD_PTC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PTC2;
        break;
    case RCC_MOD_BTIM4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BTIM4;
        break;
    case RCC_MOD_BTIM3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BTIM3;
        break;
    case RCC_MOD_SYSCFG2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SYSCFG2;
        break;
    case RCC_MOD_USART5:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_USART5;
        break;
    case RCC_MOD_USART4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_USART4;
        break;
    case RCC_MOD_PATCH:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PATCH;
        break;
    case RCC_MOD_PINMUX2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PINMUX2;
        break;
    case RCC_MOD_MAILBOX2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_MAILBOX2;
        break;
    case RCC_MOD_DMAC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_DMAC2;
        break;
    case RCC_MOD_MAC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_MAC;
        break;
    case RCC_MOD_PHY:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PHY;
        break;
    case RCC_MOD_RFC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_RFC;
        break;
    case RCC_MOD_GPIO2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPIO2;
        break;
    case RCC_MOD_SECU2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SECU2;
        break;

#endif
    default:
        HAL_ASSERT(0);
        break;

    }

    return base_rcc;
}

#else
static inline __IO uint32_t *RCC_GetModuleMask(RCC_MODULE_TYPE module, uint32_t *group, uint32_t *mask)
{
    __IO uint32_t *base_rcc = 0;
    if ((!mask) || (!group))
    {
        return NULL;
    }
    base_rcc = NULL;
    switch (module)
    {
#if 1//(CORE_ID_CURRENT == CORE_ID_HCPU)
    case RCC_MOD_DMAC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_DMAC1;
        break;
    case RCC_MOD_MAILBOX1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_MAILBOX1;
        break;
    case RCC_MOD_PINMUX1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PINMUX1;
        break;
    case RCC_MOD_USART1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_USART1;
        break;
    case RCC_MOD_USART2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_USART2;
        break;
    case RCC_MOD_EZIP:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EZIP1;
        break;
    case RCC_MOD_EPIC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EPIC;
        break;
    case RCC_MOD_LCDC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_LCDC1;
        break;
    case RCC_MOD_I2S1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2S1;
        break;
#ifdef SF32LB58X
    case RCC_MOD_I2S2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2S2;
        break;
#endif
    case RCC_MOD_SYSCFG1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SYSCFG1;
        break;
    case RCC_MOD_EFUSEC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EFUSEC;
        break;
    case RCC_MOD_AES:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_AES;
        break;
    case RCC_MOD_CRC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_CRC1;
        break;
    case RCC_MOD_CRC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_CRC2;
        break;
    case RCC_MOD_TRNG:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_TRNG;
        break;
    case RCC_MOD_GPTIM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_GPTIM1;
        break;
    case RCC_MOD_GPTIM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_GPTIM2;
        break;
    case RCC_MOD_BTIM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_BTIM1;
        break;
    case RCC_MOD_BTIM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_BTIM2;
        break;
    case RCC_MOD_SPI1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SPI1;
        break;
    case RCC_MOD_SPI2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_SPI2;
        break;
    case RCC_MOD_EXTDMA:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_EXTDMA;
        break;
#ifdef HPSYS_RCC_ENR1_NNACC1
    case RCC_MOD_NNACC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_NNACC1;
        break;
#endif
    case RCC_MOD_PDM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PDM1;
        break;

#ifdef HPSYS_RCC_ENR1_PDM2
    case RCC_MOD_PDM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PDM2;
        break;
#endif
    case RCC_MOD_I2C1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2C1;
        break;
    case RCC_MOD_I2C2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_I2C2;
        break;
#ifdef SF32LB58X
    case RCC_MOD_DSIHOST:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_DSIHOST;
        break;
    case RCC_MOD_DSIPHY:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_DSIPHY;
        break;
#endif
    case RCC_MOD_PTC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_PTC1;
        break;

    case RCC_MOD_GPIO1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_GPIO1;
        break;
    case RCC_MOD_MPI1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_MPI1;
        break;
    case RCC_MOD_MPI2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_MPI2;
        break;
#ifdef HPSYS_RCC_ENR2_MPI3
    case RCC_MOD_MPI3:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_MPI3;
        break;
#endif
    case RCC_MOD_SDMMC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_SDMMC1;
        break;
#ifdef HPSYS_RCC_ENR2_SDMMC2
    case RCC_MOD_SDMMC2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_SDMMC2;
        break;
#endif
    case RCC_MOD_USBC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_USBC;
        break;
#ifdef HPSYS_RCC_ENR2_BUSMON1
    case RCC_MOD_BUSMON1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_BUSMON1;
        break;
#endif
    case RCC_MOD_I2C3:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_I2C3;
        break;
    case RCC_MOD_ATIM1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_ATIM1;
        break;
#ifdef SF32LB58X
    case RCC_MOD_ATIM2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_ATIM2;
        break;
    case RCC_MOD_MPI4:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_MPI4;
        break;
#endif
    case RCC_MOD_USART3:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_USART3;
        break;
#ifdef HPSYS_RCC_ENR2_FFT1
    case RCC_MOD_FFT1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_FFT1;
        break;
#endif
#ifdef HPSYS_RCC_ENR2_FACC1
    case RCC_MOD_FACC1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_FACC1;
        break;
#endif
#ifdef SF32LB58X
    case RCC_MOD_DMAC2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR1_DMAC2;
        break;
#endif
#ifdef HPSYS_RCC_ENR2_CAN1
    case RCC_MOD_CAN1:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_CAN1;
        break;
#endif
#ifdef SF32LB58X
    case RCC_MOD_CAN2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_CAN2;
        break;
#endif
    case RCC_MOD_AUDCODEC_HP:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_AUDCODEC;
        break;
    case RCC_MOD_AUDPRC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_AUDPRC;
        break;
#ifdef SF32LB58X
    case RCC_MOD_GPU:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_GPU;
        break;
    case RCC_MOD_JENC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_JENC;
        break;
    case RCC_MOD_JDEC:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_JDEC;
        break;
#endif
    case RCC_MOD_I2C4:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_I2C4;
        break;
#ifdef SF32LB58X
    case RCC_MOD_ACPU:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_ACPU;
        break;
#endif
#ifdef HPSYS_RCC_ENR2_SCI
    case RCC_MOD_SCI:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 1;
        *mask = HPSYS_RCC_ENR2_SCI;
        break;
#endif
#endif
#if 1//(CORE_ID_CURRENT == CORE_ID_LCPU)
#ifdef SF32LB58X
    case RCC_MOD_FACC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_FACC2;
        break;
    case RCC_MOD_FFT2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_FFT2;
        break;
    case RCC_MOD_BUSMON2:
        base_rcc = (uint32_t *)hwp_hpsys_rcc;
        *group = 0;
        *mask = HPSYS_RCC_ENR2_BUSMON2;
        break;
    case RCC_MOD_BUSMON3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BUSMON3;
        break;
#else
    case RCC_MOD_BUSMON2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BUSMON2;
        break;
#endif

#if !defined(SF32LB56X)&&!defined(SF32LB52X)
    case RCC_MOD_LCDC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_LCDC2;
        break;
#endif
    case RCC_MOD_PTC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PTC2;
        break;
    case RCC_MOD_TSEN:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_TSEN;
        break;
    case RCC_MOD_LPCOMP:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_LPCOMP;
        break;
        break;
    case RCC_MOD_AUDCODEC_LP:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_AUDCODEC;
        break;
        break;
#if !defined(SF32LB56X)&&!defined(SF32LB52X)
    case RCC_MOD_SDADC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SDADC;
        break;
#endif
    case RCC_MOD_GPADC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPADC;
        break;
#if !defined(SF32LB56X)&&!defined(SF32LB52X)
    case RCC_MOD_NNACC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_NNACC2;
        break;
#endif
    case RCC_MOD_BTIM4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BTIM4;
        break;
    case RCC_MOD_BTIM3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_BTIM3;
        break;
    case RCC_MOD_GPTIM5:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPTIM5;
        break;
    case RCC_MOD_GPTIM4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPTIM4;
        break;
    case RCC_MOD_GPTIM3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_GPTIM3;
        break;
    case RCC_MOD_SYSCFG2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SYSCFG2;
        break;
    case RCC_MOD_I2C7:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_I2C7;
        break;
    case RCC_MOD_I2C6:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_I2C6;
        break;
    case RCC_MOD_I2C5:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_I2C5;
        break;
    case RCC_MOD_SPI4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SPI4;
        break;
    case RCC_MOD_SPI3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_SPI3;
        break;
#if !defined(SF32LB56X)&&!defined(SF32LB52X)
    case RCC_MOD_I2S3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_I2S3;
        break;
#endif
    case RCC_MOD_USART6:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_USART6;
        break;
    case RCC_MOD_USART5:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_USART5;
        break;
    case RCC_MOD_USART4:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_USART4;
        break;
    case RCC_MOD_PATCH:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PATCH;
        break;
    case RCC_MOD_PINMUX2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_PINMUX2;
        break;
    case RCC_MOD_MAILBOX2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_MAILBOX2;
        break;
#if defined(SF32LB56X)||defined(SF32LB52X)
    case RCC_MOD_DMAC2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_DMAC2;
        break;
#else
    case RCC_MOD_DMAC3:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 0;
        *mask = LPSYS_RCC_ENR1_DMAC3;
        break;
#endif
    case RCC_MOD_MAC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_MAC;
        break;
    case RCC_MOD_PHY:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_PHY;
        break;
    case RCC_MOD_RFC:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_RFC;
        break;
    case RCC_MOD_MPI5:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_MPI5;
        break;
    case RCC_MOD_GPIO2:
        base_rcc = (uint32_t *)hwp_lpsys_rcc;
        *group = 1;
        *mask = LPSYS_RCC_ENR2_GPIO2;
        break;
#endif
    default:
        HAL_ASSERT(0);
        break;

    }

    return base_rcc;
}

#endif

#ifdef SF32LB55X
__HAL_ROM_USED void HAL_RCC_HCPU_ClockSelect(int clk_module, int src)
{
    uint32_t csr = hwp_hpsys_rcc->CSR;
    uint32_t mask = (clk_module <= RCC_CLK_MOD_FLASH3) ? 3 : 1;

    csr &= ~(mask << clk_module);
    hwp_hpsys_rcc->CSR = csr | ((src & mask) << clk_module);
    if (clk_module == RCC_CLK_MOD_SYS)
        SystemCoreClock = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);
}

__HAL_ROM_USED int HAL_RCC_HCPU_GetClockSrc(int clk_module)
{
    uint32_t csr = hwp_hpsys_rcc->CSR;
    uint32_t mask = (clk_module <= RCC_CLK_MOD_FLASH3) ? 3 : 1;
    uint8_t src;

    mask <<= clk_module;
    src = GET_REG_VAL(csr, mask, clk_module);

    return src;
}
#else
__HAL_ROM_USED void HAL_RCC_HCPU_ClockSelect(int clk_module, int src)
{
    uint32_t csr = hwp_hpsys_rcc->CSR;
    uint32_t mask;

    if ((HPSYS_RCC_CSR_SEL_SYS_Pos == clk_module)
            || ((clk_module >= RCC_CLK_MOD_FLASH1) && (clk_module < RCC_CLK_MOD_HP_PERI))
#ifdef   HPSYS_RCC_CSR_SEL_SDMMC_Pos
            || (HPSYS_RCC_CSR_SEL_SDMMC_Pos == clk_module)
#endif
#ifdef   RCC_CLK_MOD_HP_TICK
            || (RCC_CLK_MOD_HP_TICK == clk_module)
#endif /* RCC_CLK_MOD_HP_TICK */
       )
    {
        mask = 3;
    }
    else
    {
        mask = 1;
    }

    csr &= ~(mask << clk_module);
    hwp_hpsys_rcc->CSR = csr | ((src & mask) << clk_module);
    if (clk_module == RCC_CLK_MOD_SYS)
        SystemCoreClock = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);
}

__HAL_ROM_USED int HAL_RCC_HCPU_GetClockSrc(int clk_module)
{
    uint32_t csr = hwp_hpsys_rcc->CSR;
    uint32_t mask;
    uint8_t src;

    if ((HPSYS_RCC_CSR_SEL_SYS_Pos == clk_module)
            || ((clk_module >= RCC_CLK_MOD_FLASH1) && (clk_module < RCC_CLK_MOD_HP_PERI))
#ifdef   HPSYS_RCC_CSR_SEL_SDMMC_Pos
            || (HPSYS_RCC_CSR_SEL_SDMMC_Pos == clk_module)
#endif
#ifdef   RCC_CLK_MOD_HP_TICK
            || (RCC_CLK_MOD_HP_TICK == clk_module)
#endif /* RCC_CLK_MOD_HP_TICK */
       )
    {
        mask = 3;
    }
    else
    {
        mask = 1;
    }

    mask <<= clk_module;
    src = GET_REG_VAL(csr, mask, clk_module);

    return src;
}

#endif /* SF32LB55X */

__HAL_ROM_USED uint32_t HAL_RCC_HCPU_GetDLLFreq(int dll)
{
    uint32_t cr;
    uint32_t freq;

    switch (dll)
    {
    case 1:
        cr = hwp_hpsys_rcc->DLL1CR;
        break;
    case 2:
        cr = hwp_hpsys_rcc->DLL2CR;
        break;

#ifndef SF32LB52X
    case 3:
        cr = hwp_hpsys_rcc->DLL3CR;
        break;
#endif
    default:
        cr = 0;
    }
    if (cr)
    {
        uint32_t stg;
        if (cr & HPSYS_RCC_DLL1CR_EN)
        {
            stg = GET_REG_VAL(cr, HPSYS_RCC_DLL1CR_STG_Msk, HPSYS_RCC_DLL1CR_STG_Pos);
#ifdef SF32LB55X
            if (DLL_DIV2_MAX >= stg * DLL_STG_STEP)
            {
                freq = DLL_DIV2_MAX - stg * DLL_STG_STEP;
                if ((cr & HPSYS_RCC_DLL1CR_OUT_DIV2_EN) == 0)
                    freq <<= 1;
            }
            else
            {
                freq = 0;
            }
#else
            freq = stg * DLL_STG_STEP + DLL_MIN_FREQ;
#endif
        }
        else
        {
            freq = 0;
        }
    }
    else
        freq = 0;

    return freq;
}


__HAL_ROM_USED uint32_t HAL_RCC_HCPU_GetDLL1Freq(void)
{
    return (HAL_RCC_HCPU_GetDLLFreq(1));
}

__HAL_ROM_USED uint32_t HAL_RCC_HCPU_GetDLL2Freq(void)
{
    return (HAL_RCC_HCPU_GetDLLFreq(2));
}

__HAL_ROM_USED uint32_t HAL_RCC_HCPU_GetDLL3Freq(void)
{
    return (HAL_RCC_HCPU_GetDLLFreq(3));
}

#ifdef SOC_BF0_HCPU
__HAL_ROM_USED void HAL_RCC_HCPU_SetDiv(int div, int pdiv1, int pdiv2)
{
    uint32_t mask = 0, reg = 0;

    if (div > 0)
    {
        mask = HPSYS_RCC_CFGR_HDIV_Msk;
        reg = div;
    }
    if (pdiv1 >= 0)
    {
        mask |= HPSYS_RCC_CFGR_PDIV1_Msk;
        reg |= (pdiv1 << HPSYS_RCC_CFGR_PDIV1_Pos);
    }
    if (pdiv2 >= 0)
    {
        mask |= HPSYS_RCC_CFGR_PDIV2_Msk;
        reg |= (pdiv2 << HPSYS_RCC_CFGR_PDIV2_Pos);
    }
    if (mask)
    {
        MODIFY_REG(hwp_hpsys_rcc->CFGR, mask, reg);
    }
    SystemCoreClock = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);

}

void HAL_RCC_HCPU_GetDiv(int *div, int *pdiv1, int *pdiv2)
{
    uint32_t val;

    val = hwp_hpsys_rcc->CFGR;
    if (div)
    {
        *div = GET_REG_VAL(val, HPSYS_RCC_CFGR_HDIV_Msk, HPSYS_RCC_CFGR_HDIV_Pos);
    }
    if (pdiv1)
    {
        *pdiv1 = GET_REG_VAL(val, HPSYS_RCC_CFGR_PDIV1_Msk, HPSYS_RCC_CFGR_PDIV1_Pos);
    }
    if (pdiv2)
    {
        *pdiv2 = GET_REG_VAL(val, HPSYS_RCC_CFGR_PDIV2_Msk, HPSYS_RCC_CFGR_PDIV2_Pos);
    }
}


#ifdef SF32LB55X
__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_HCPU_EnableDLL(int dll, uint32_t freq)
{
    HAL_StatusTypeDef status;
    uint32_t stg;
    __IO uint32_t *cr;
    uint32_t val;

    if ((0 == freq) || (freq > DLL_MAX_FREQ))
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    if (freq > DLL_DIV2_MAX)
    {
        stg = (DLL_DIV2_MAX * 2 - freq) / (DLL_STG_STEP * 2); // Need to set out div to 1.
    }
    else
    {
        // default out div is 2. HPSYS_RCC_DLL1CR_OUT_DIV2_EN is set
        stg = (DLL_DIV2_MAX - freq) / DLL_STG_STEP;
        if (DLL_DIV2_MAX != (freq + (stg * DLL_STG_STEP)))
        {
            status = HAL_ERROR;
            goto __EXIT;
        }
    }
    stg = MAKE_REG_VAL(stg, HPSYS_RCC_DLL1CR_STG_Msk, HPSYS_RCC_DLL1CR_STG_Pos);
    switch (dll)
    {
    case 1:
        cr = &(hwp_hpsys_rcc->DLL1CR);
        break;
    case 2:
        cr = &(hwp_hpsys_rcc->DLL2CR);
        break;
    case 3:
        cr = &(hwp_hpsys_rcc->DLL3CR);
        break;
    default:
        HAL_ASSERT(0);
        break;

    }

    (*cr) &= ~HPSYS_RCC_DLL1CR_EN;
    val = (*cr);
    MODIFY_REG(val, HPSYS_RCC_DLL1CR_STG_Msk, stg);
    if (freq > DLL_DIV2_MAX)  // If max freq > 192M(ie, 240M), need set out div to 1.
        MODIFY_REG(val, HPSYS_RCC_DLL1CR_OUT_DIV2_EN, 0);
    else
        MODIFY_REG(val, HPSYS_RCC_DLL1CR_OUT_DIV2_EN, HPSYS_RCC_DLL1CR_OUT_DIV2_EN);
    (*cr) = (HPSYS_RCC_DLL1CR_EN | val);

    HAL_Delay_us(0);
    /** wait for DLL ready, 5us at least */
    HAL_Delay_us(10);
    while (0 == ((*cr) & HPSYS_RCC_DLL1CR_READY))
    {
    }

    status = HAL_OK;

__EXIT:
    return status;
}
#else

__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_HCPU_EnableDLL(int dll, uint32_t freq)
{
    HAL_StatusTypeDef status;
    uint32_t stg;
    __IO uint32_t *cr;
    uint32_t val;

    if ((freq < DLL_MIN_FREQ) || (freq > DLL_MAX_FREQ))
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    switch (dll)
    {
    case 1:
        cr = &(hwp_hpsys_rcc->DLL1CR);
        break;
    case 2:
        cr = &(hwp_hpsys_rcc->DLL2CR);
        break;
#ifndef SF32LB52X
    case 3:
        cr = &(hwp_hpsys_rcc->DLL3CR);
        break;
#endif
    default:
        HAL_ASSERT(0);
        break;
    }

    if (0 == (hwp_hpsys_cfg->CAU2_CR & HPSYS_CFG_CAU2_CR_HPBG_EN))
    {
        hwp_hpsys_cfg->CAU2_CR |= HPSYS_CFG_CAU2_CR_HPBG_EN;
    }
    if (0 == (hwp_hpsys_cfg->CAU2_CR & HPSYS_CFG_CAU2_CR_HPBG_VDDPSW_EN))
    {
        hwp_hpsys_cfg->CAU2_CR |= HPSYS_CFG_CAU2_CR_HPBG_VDDPSW_EN;
    }

    (*cr) &= ~HPSYS_RCC_DLL1CR_EN;
    val = (*cr);

    MODIFY_REG(val, HPSYS_RCC_DLL1CR_IN_DIV2_EN, HPSYS_RCC_DLL1CR_IN_DIV2_EN);
    MODIFY_REG(val, HPSYS_RCC_DLL1CR_OUT_DIV2_EN, 0);

    stg = (freq - DLL_MIN_FREQ) / DLL_STG_STEP;
    stg = MAKE_REG_VAL(stg, HPSYS_RCC_DLL1CR_STG_Msk, HPSYS_RCC_DLL1CR_STG_Pos);
    MODIFY_REG(val, HPSYS_RCC_DLL1CR_STG_Msk, stg);
    (*cr) = (HPSYS_RCC_DLL1CR_EN | val);

    HAL_Delay_us(0);
    /** wait for DLL ready, 5us at least */
    HAL_Delay_us(10);
    while (0 == ((*cr) & HPSYS_RCC_DLL1CR_READY))
    {
    }

    status = HAL_OK;

__EXIT:
    return status;
}

#endif


__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_HCPU_EnableDLL1(uint32_t freq)
{
    return HAL_RCC_HCPU_EnableDLL(1, freq);
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_HCPU_EnableDLL2(uint32_t freq)
{
    return HAL_RCC_HCPU_EnableDLL(2, freq);
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_HCPU_EnableDLL3(uint32_t freq)
{
    return HAL_RCC_HCPU_EnableDLL(3, freq);
}

__USED __HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_HCPU_DisableDLL1(void)
{
    hwp_hpsys_rcc->DLL1CR &= ~HPSYS_RCC_DLL1CR_EN;
    return HAL_OK;
}
__USED __HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_HCPU_DisableDLL2(void)
{
    hwp_hpsys_rcc->DLL2CR &= ~HPSYS_RCC_DLL1CR_EN;
    return HAL_OK;
}

#ifndef SF32LB52X
__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_HCPU_DisableDLL3(void)
{
    hwp_hpsys_rcc->DLL3CR &= ~HPSYS_RCC_DLL1CR_EN;
    return HAL_OK;
}
#endif

#endif

#if defined(SOC_BF0_LCPU) || defined(SOC_BF0_HCPU)
__HAL_ROM_USED void HAL_RCC_LCPU_SetDiv(int div, int pdiv1, int pdiv2)
{
    uint32_t mask = 0, reg = 0;

    if (div > 0)
    {
        mask = LPSYS_RCC_CFGR_HDIV1_Msk;
        reg = (div << LPSYS_RCC_CFGR_HDIV1_Pos);
    }

    if (pdiv1 >= 0)
    {
        if ((1 == div) && (0 == pdiv1))
        {
            /* PCLK cannot exceed 24MHz */
            pdiv1 = 1;
        }

        mask |= LPSYS_RCC_CFGR_PDIV1_Msk;
        reg |= (pdiv1 << LPSYS_RCC_CFGR_PDIV1_Pos);
    }
    if (pdiv2 >= 0)
    {
        mask |= LPSYS_RCC_CFGR_PDIV2_Msk;
        reg |= (pdiv2 << LPSYS_RCC_CFGR_PDIV2_Pos);
    }
    if (mask)
    {
#ifndef SF32LB52X
        if (div == 1)
        {
            MODIFY_REG(hwp_lpsys_rcc->CFGR, LPSYS_RCC_CFGR_HDIV2_Msk, (1UL << LPSYS_RCC_CFGR_HDIV2_Pos));
        }
#endif /* SF32LB52X */

        MODIFY_REG(hwp_lpsys_rcc->CFGR, mask, reg);

#ifndef SF32LB52X
        if (div != 1 && div >= 0)
        {
            MODIFY_REG(hwp_lpsys_rcc->CFGR, LPSYS_RCC_CFGR_HDIV2_Msk, (0UL << LPSYS_RCC_CFGR_HDIV2_Pos));
        }
#endif /* SF32LB52X */
    }
    SystemCoreClock = HAL_RCC_GetHCLKFreq(CORE_ID_LCPU);
}

void HAL_RCC_LCPU_GetDiv(int *div, int *pdiv1, int *pdiv2)
{
    uint32_t val;

    val = hwp_lpsys_rcc->CFGR;
    if (div)
    {
        *div = GET_REG_VAL(val, LPSYS_RCC_CFGR_HDIV1_Msk, LPSYS_RCC_CFGR_HDIV1_Pos);
    }
    if (pdiv1)
    {
        *pdiv1 = GET_REG_VAL(val, LPSYS_RCC_CFGR_PDIV1_Msk, LPSYS_RCC_CFGR_PDIV1_Pos);
    }
    if (pdiv2)
    {
        *pdiv2 = GET_REG_VAL(val, LPSYS_RCC_CFGR_PDIV2_Msk, LPSYS_RCC_CFGR_PDIV2_Pos);
    }
}


#endif


__HAL_ROM_USED int HAL_RCC_LCPU_GetClockSrc(int clk_module)
{
    uint32_t csr = hwp_lpsys_rcc->CSR;
    uint32_t mask;
    uint8_t src;

    if ((RCC_CLK_MOD_SYS == clk_module)
#ifdef   RCC_CLK_MOD_LP_TICK
            || (RCC_CLK_MOD_LP_TICK == clk_module)
#endif /* RCC_CLK_MOD_LP_TICK */
       )
    {
        mask = 3;
    }
    else
    {
        mask = 1;
    }

    mask <<= clk_module;
    src = GET_REG_VAL(csr, mask, clk_module);

    return src;
}


__HAL_ROM_USED uint32_t HAL_RCC_GetSysCLKFreq(int core_id)
{
    uint32_t r = 48000000;

    if (core_id == CORE_ID_HCPU)
    {
        switch (hwp_hpsys_rcc->CSR & 3)
        {
        case RCC_SYSCLK_DLL1:
            r = HAL_RCC_HCPU_GetDLL1Freq();
            break;
        }
    }
    return r;
}

__HAL_ROM_USED uint32_t HAL_RCC_GetHCLKFreq(int core_id)
{
    uint32_t r;
    int div = 1;

    if (CORE_ID_DEFAULT == core_id)
        core_id = CORE_ID_CURRENT;
    r = HAL_RCC_GetSysCLKFreq(core_id);

    switch (core_id)
    {
    case CORE_ID_HCPU:
        div = (hwp_hpsys_rcc->CFGR & HPSYS_RCC_CFGR_HDIV_Msk);
        break;
    case CORE_ID_LCPU:
        // TODO: HDIV2 is not used.
        div = (hwp_lpsys_rcc->CFGR & LPSYS_RCC_CFGR_HDIV1_Msk);
        break;
    default:
        HAL_ASSERT(0);
    }
    if (div == 0)
        div = 1;
    return r / div;
}

__HAL_ROM_USED uint32_t HAL_RCC_GetPCLKFreq(int core_id, int is_pclk1)
{
    uint32_t hclk;
    uint32_t pdiv;
    uint32_t r = 0;

    if (CORE_ID_DEFAULT == core_id)
        core_id = CORE_ID_CURRENT;
    hclk = HAL_RCC_GetHCLKFreq(core_id);
    switch (core_id)
    {
    case CORE_ID_HCPU:
        if (is_pclk1)
            pdiv = GET_REG_VAL(hwp_hpsys_rcc->CFGR, HPSYS_RCC_CFGR_PDIV1_Msk, HPSYS_RCC_CFGR_PDIV1_Pos);
        else
            pdiv = GET_REG_VAL(hwp_hpsys_rcc->CFGR, HPSYS_RCC_CFGR_PDIV2_Msk, HPSYS_RCC_CFGR_PDIV2_Pos);
        r = hclk >> pdiv;
        break;
    case CORE_ID_LCPU:
        if (is_pclk1)
            pdiv = GET_REG_VAL(hwp_lpsys_rcc->CFGR, LPSYS_RCC_CFGR_PDIV1_Msk, LPSYS_RCC_CFGR_PDIV1_Pos);
        else
            pdiv = GET_REG_VAL(hwp_lpsys_rcc->CFGR, LPSYS_RCC_CFGR_PDIV2_Msk, LPSYS_RCC_CFGR_PDIV2_Pos);
        r = hclk >> pdiv;
        break;
    default:
        HAL_ASSERT(0);
    }
    return r;
}


__HAL_ROM_USED void HAL_RCC_ResetBluetoothRF(void)
{
#ifdef SF32LB52X
    hwp_lpsys_rcc->RSTR1 |= LPSYS_RCC_RSTR1_RFC;
    while (!(hwp_lpsys_rcc->RSTR1 & LPSYS_RCC_RSTR1_RFC));
    hwp_lpsys_rcc->RSTR1 &= ~LPSYS_RCC_RSTR1_RFC;
#else
    hwp_lpsys_rcc->RSTR2 |= LPSYS_RCC_RSTR2_RFC;
    while (!(hwp_lpsys_rcc->RSTR2 & LPSYS_RCC_RSTR2_RFC));
    hwp_lpsys_rcc->RSTR2 &= ~LPSYS_RCC_RSTR2_RFC;
#endif
}


// RSTR1 is default 0 and LCPU only run when hwp_lpsys_aon->PWR is 0
__HAL_ROM_USED void HAL_RCC_ReleaseLCPU(void)
{
    hwp_lpsys_aon->PMR &= ~LPSYS_AON_PMR_CPUWAIT;
}

__HAL_ROM_USED void HAL_RCC_ResetLCPU(void)
{
    if (hwp_lpsys_aon->PMR & LPSYS_AON_PMR_CPUWAIT)
        HAL_RCC_ReleaseLCPU();
    else
    {
#ifdef SF32LB52X
        hwp_lpsys_rcc->RSTR1 = LPSYS_RCC_RSTR1_MAC;
#else
        hwp_lpsys_rcc->RSTR2 = LPSYS_RCC_RSTR2_MAC;
#endif
        hwp_lpsys_rcc->RSTR1 = LPSYS_RCC_RSTR1_LCPU;
        while (!hwp_lpsys_rcc->RSTR1);
#ifdef SF32LB52X
        hwp_lpsys_rcc->RSTR1 &= ~LPSYS_RCC_RSTR1_MAC;
#else
        hwp_lpsys_rcc->RSTR2 &= ~LPSYS_RCC_RSTR2_MAC;
#endif
        hwp_lpsys_rcc->RSTR1 &= ~LPSYS_RCC_RSTR1_LCPU_Msk;
    }
}

__HAL_ROM_USED void HAL_RCC_Reset_and_Halt_LCPU(uint8_t is_init)
{
    if ((hwp_lpsys_aon->PMR & LPSYS_AON_PMR_CPUWAIT) == 0)
    {
        uint32_t rst_flag = is_init ? 0xFFFFFFFF : LPSYS_RCC_RSTR1_LCPU;

        hwp_lpsys_aon->PMR |= LPSYS_AON_PMR_CPUWAIT;

#ifdef SF32LB52X
        rst_flag |= LPSYS_RCC_RSTR1_MAC;
#endif
        hwp_lpsys_rcc->RSTR1 = rst_flag ;
        while (!hwp_lpsys_rcc->RSTR1);

#ifndef SF32LB52X
        hwp_lpsys_rcc->RSTR2 = LPSYS_RCC_RSTR2_MAC;
        while (!hwp_lpsys_rcc->RSTR2);
#endif
        if ((hwp_lpsys_aon->SLP_CTRL & LPSYS_AON_SLP_CTRL_SLEEP_STATUS) != 0)
        {
            hwp_lpsys_aon->SLP_CTRL |= LPSYS_AON_SLP_CTRL_WKUP_REQ;
            while ((hwp_lpsys_aon->SLP_CTRL & LPSYS_AON_SLP_CTRL_SLEEP_STATUS) != 0);
        }
#ifndef SF32LB52X
        hwp_lpsys_rcc->RSTR2 &= ~LPSYS_RCC_RSTR2_MAC;
#endif
        hwp_lpsys_rcc->RSTR1 &= ~rst_flag;
    }
}


#if defined(SF32LB58X) && defined(SOC_BF0_HCPU)
void HAL_RCC_ReleaseACPU(void)
{
    hwp_hpsys_rcc->RSTR2 &= ~HPSYS_RCC_RSTR2_ACPU;
}

void HAL_RCC_ResetACPU(void)
{
    hwp_hpsys_rcc->RSTR2 |= HPSYS_RCC_RSTR2_ACPU;
}
#endif /* SF32LB58X && SOC_BF0_HCPU */


__HAL_ROM_USED void HAL_RCC_SetMacFreq(void)
{
    uint32_t sys_clk;
    uint32_t mac_clk = 8000000;
    uint32_t mac_div;
#if 0
    if (!(hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
    {
        hwp_lpsys_aon->ACR |= LPSYS_AON_ACR_HXT48_REQ;
        while (0 == (hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
        {
            /* wait until HXT48 ready */
        }
    }

    // switch system clock to HXT48
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, 1);
#endif
    sys_clk = HAL_RCC_GetHCLKFreq(CORE_ID_LCPU);
    mac_div = sys_clk / mac_clk;

    MODIFY_REG(hwp_lpsys_rcc->CFGR, LPSYS_RCC_CFGR_MACDIV_Msk | LPSYS_RCC_CFGR_MACFREQ_Msk,
               (mac_div << LPSYS_RCC_CFGR_MACDIV_Pos) | (0x08 << LPSYS_RCC_CFGR_MACFREQ_Pos));
}


__HAL_ROM_USED void HAL_RCC_ResetModule(RCC_MODULE_TYPE module)
{
    __IO uint32_t *base_rcc, *rstr;
    uint32_t mask, group;

    base_rcc = RCC_GetModuleMask(module, &group, &mask);

    if (((uint32_t *)hwp_hpsys_rcc == base_rcc) && (0 == group))
        rstr = (uint32_t *)&hwp_hpsys_rcc->RSTR1;
    else if (((uint32_t *)hwp_hpsys_rcc == base_rcc) && (1 == group))
        rstr = (uint32_t *)&hwp_hpsys_rcc->RSTR2;
    else if (((uint32_t *)hwp_lpsys_rcc == base_rcc) && (0 == group))
        rstr = (uint32_t *)&hwp_lpsys_rcc->RSTR1;
#ifndef SF32LB52X
    else if (((uint32_t *)hwp_lpsys_rcc == base_rcc) && (1 == group))
        rstr = (uint32_t *)&hwp_lpsys_rcc->RSTR2;
#endif
    else
    {
        HAL_ASSERT(0);
        return;
    }

    /*Rest module*/
    *rstr |= mask;
    *rstr &= ~mask;
}



__HAL_ROM_USED void HAL_RCC_EnableModule(RCC_MODULE_TYPE module)
{
    __IO uint32_t *base_rcc, *enr;
    uint32_t mask, group;
    uint32_t level;

    base_rcc = RCC_GetModuleMask(module, &group, &mask);

    if (((uint32_t *)hwp_hpsys_rcc == base_rcc) && (0 == group))
        enr = (uint32_t *)&hwp_hpsys_rcc->ENR1;
    else if (((uint32_t *)hwp_hpsys_rcc == base_rcc) && (1 == group))
        enr = (uint32_t *)&hwp_hpsys_rcc->ENR2;
    else if (((uint32_t *)hwp_lpsys_rcc == base_rcc) && (0 == group))
        enr = (uint32_t *)&hwp_lpsys_rcc->ENR1;
#ifndef SF32LB52X
    else if (((uint32_t *)hwp_lpsys_rcc == base_rcc) && (1 == group))
        enr = (uint32_t *)&hwp_lpsys_rcc->ENR2;
#endif
    else
    {
        HAL_ASSERT(0);
        return;
    }

    level = HAL_RCC_DisableInterrupt();
    *enr |= mask;
    HAL_RCC_EnableInterrupt(level);
}

__HAL_ROM_USED void HAL_RCC_DisableModule(RCC_MODULE_TYPE module)
{
    __IO uint32_t *base_rcc, *enr;
    uint32_t mask, group;
    uint32_t level;

    base_rcc = RCC_GetModuleMask(module, &group, &mask);

    if (((uint32_t *)hwp_hpsys_rcc == base_rcc) && (0 == group))
        enr = (uint32_t *)&hwp_hpsys_rcc->ENR1;
    else if (((uint32_t *)hwp_hpsys_rcc == base_rcc) && (1 == group))
        enr = (uint32_t *)&hwp_hpsys_rcc->ENR2;
    else if (((uint32_t *)hwp_lpsys_rcc == base_rcc) && (0 == group))
        enr = (uint32_t *)&hwp_lpsys_rcc->ENR1;
#ifndef SF32LB52X
    else if (((uint32_t *)hwp_lpsys_rcc == base_rcc) && (1 == group))
        enr = (uint32_t *)&hwp_lpsys_rcc->ENR2;
#endif
    else
    {
        HAL_ASSERT(0);
        return;
    }

    level = HAL_RCC_DisableInterrupt();
    *enr &= ~mask;
    HAL_RCC_EnableInterrupt(level);
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_SetModuleFreq(RCC_MODULE_TYPE module, uint32_t freq)
{
    switch (module)
    {
#ifdef SF32LB55X
#if (CORE_ID_CURRENT == CORE_ID_HCPU)

    case RCC_MOD_I2S1:
    case RCC_MOD_I2S2:
    {
        uint32_t pclk1, div;

        pclk1 = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
        div = pclk1 / freq; // i2s mclk alway keep 12M Hz

        HAL_ASSERT(div > 0);
        if (div > 0)
        {
            // for this register, it need write twice to avoid shadow
            hwp_hpsys_rcc->I2SR = div;
            hwp_hpsys_rcc->I2SR = div;
        }
        else
        {
            return HAL_ERROR;
        }
    }
    break;
#endif
#endif /* SF32LB55X */
    default:
        HAL_ASSERT(0);
        return HAL_ERROR;
        break;

    }
    return HAL_OK;
}


__HAL_ROM_USED uint32_t HAL_RCC_GetModuleFreq(RCC_MODULE_TYPE module)
{
    uint32_t freq = 0;
    switch (module)
    {
#ifdef SF32LB55X
#if (CORE_ID_CURRENT == CORE_ID_HCPU)
    case RCC_MOD_I2S1:
    case RCC_MOD_I2S2:
    {
        uint32_t pclk1, div;

        pclk1 = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
        div = hwp_hpsys_rcc->I2SR;

        if (div > 0)
        {
            freq = pclk1 / div;
        }
    }
    break;
#endif
#endif /* SF32LB55X */
    default:
        HAL_ASSERT(0);
        break;

    }

    return freq;
}


#ifdef SF32LB52X
__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_CalibrateRC48(void)
{
    uint32_t hxt_cnt;
    uint32_t hrc_cnt;
    uint16_t  cnt_diff;
    uint16_t step_unit;
    uint16_t ct_val ;
    uint32_t i;

    if (0 == (hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY))
    {
        return HAL_ERROR;
    }

    /* set cal length */
    MODIFY_REG(hwp_hpsys_rcc->HRCCAL1, HPSYS_RCC_HRCCAL1_CAL_LENGTH_Msk,
               MAKE_REG_VAL(0x3fff, HPSYS_RCC_HRCCAL1_CAL_LENGTH_Msk, HPSYS_RCC_HRCCAL1_CAL_LENGTH_Pos));

    MODIFY_REG(hwp_pmuc->HRC_CR, PMUC_HRC_CR_CLKHP_SEL_Msk,
               MAKE_REG_VAL(3, PMUC_HRC_CR_CLKHP_SEL_Msk, PMUC_HRC_CR_CLKHP_SEL_Pos));
    MODIFY_REG(hwp_pmuc->HRC_CR, PMUC_HRC_CR_CLKLP_SEL_Msk,
               MAKE_REG_VAL(3, PMUC_HRC_CR_CLKLP_SEL_Msk, PMUC_HRC_CR_CLKLP_SEL_Pos));


    /* set start cfg to mid value */
    ct_val    = 0x200;
    step_unit = 0x100 ;

    /* binary search  */
    for (i = 0; i < 11; i++)
    {
        MODIFY_REG(hwp_pmuc->HRC_CR, PMUC_HRC_CR_FREQ_TRIM_Msk,
                   MAKE_REG_VAL(ct_val, PMUC_HRC_CR_FREQ_TRIM_Msk, PMUC_HRC_CR_FREQ_TRIM_Pos));

        HAL_Delay_us(3);

        /* enable hrc cal */
        hwp_hpsys_rcc->HRCCAL1 |= HPSYS_RCC_HRCCAL1_CAL_EN;

        /* wait for cal done */
        while (!(hwp_hpsys_rcc->HRCCAL1 & HPSYS_RCC_HRCCAL1_CAL_DONE))
        {
        }

        /* read out cal result */
        hxt_cnt = GET_REG_VAL(hwp_hpsys_rcc->HRCCAL2, HPSYS_RCC_HRCCAL2_HXT_CNT_Msk, HPSYS_RCC_HRCCAL2_HXT_CNT_Pos);
        hrc_cnt = GET_REG_VAL(hwp_hpsys_rcc->HRCCAL2, HPSYS_RCC_HRCCAL2_HRC_CNT_Msk, HPSYS_RCC_HRCCAL2_HRC_CNT_Pos);


        /* disable hrc cal */
        hwp_hpsys_rcc->HRCCAL1 &= ~HPSYS_RCC_HRCCAL1_CAL_EN ;

        if (hxt_cnt > hrc_cnt)
        {
            ct_val += step_unit;
            cnt_diff = hxt_cnt - hrc_cnt;
        }
        else
        {
            ct_val -= step_unit;
            cnt_diff = hrc_cnt - hxt_cnt;
        }
        if (cnt_diff < 64)
        {
            break;
        }
        step_unit >>= 1 ;
    }
    //printf("Calibration result : hxt_cnt = %d, hrc_cnt = %d, cnt_diff=%d\n",hxt_cnt, hrc_cnt, cnt_diff  );
    if (cnt_diff > 160)
    {
        return HAL_ERROR;
    }
    else
    {
        return HAL_OK;
    }
}

#else
__HAL_ROM_USED HAL_StatusTypeDef HAL_RCC_CalibrateRC48(void)
{
    uint32_t hxt_cnt;
    uint32_t hrc_cnt;
    uint16_t cnt_diff;
    uint16_t step_unit;
    uint16_t ct_val ;
    uint32_t i;

    if (0 == (hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY))
    {
        return HAL_ERROR;
    }

    /* set cal length */
    MODIFY_REG(hwp_hpsys_rcc->HRCCAL1, HPSYS_RCC_HRCCAL1_CAL_LENGTH_Msk,
               MAKE_REG_VAL(0x3fff, HPSYS_RCC_HRCCAL1_CAL_LENGTH_Msk, HPSYS_RCC_HRCCAL1_CAL_LENGTH_Pos));

    /* set start cfg to mid value */
    ct_val    = 0x400;
    step_unit = 0x200 ;

    /* binary search  */
    for (i = 0; i < 11; i++)
    {
        MODIFY_REG(hwp_pmuc->HRC_CR, PMUC_HRC_CR_CT_Msk,
                   MAKE_REG_VAL(ct_val, PMUC_HRC_CR_CT_Msk, PMUC_HRC_CR_CT_Pos));

        HAL_Delay_us(3);

        /* enable hrc cal */
        hwp_hpsys_rcc->HRCCAL1 |= HPSYS_RCC_HRCCAL1_CAL_EN;

        /* wait for cal done */
        while (!(hwp_hpsys_rcc->HRCCAL1 & HPSYS_RCC_HRCCAL1_CAL_DONE))
        {
        }

        /* read out cal result */
        hxt_cnt = GET_REG_VAL(hwp_hpsys_rcc->HRCCAL2, HPSYS_RCC_HRCCAL2_HXT_CNT_Msk, HPSYS_RCC_HRCCAL2_HXT_CNT_Pos);
        hrc_cnt = GET_REG_VAL(hwp_hpsys_rcc->HRCCAL2, HPSYS_RCC_HRCCAL2_HRC_CNT_Msk, HPSYS_RCC_HRCCAL2_HRC_CNT_Pos);


        /* disable hrc cal */
        hwp_hpsys_rcc->HRCCAL1 &= ~HPSYS_RCC_HRCCAL1_CAL_EN ;

        if (hxt_cnt > hrc_cnt)
        {
            ct_val -= step_unit;
            cnt_diff = hxt_cnt - hrc_cnt;
        }
        else
        {
            ct_val += step_unit;
            cnt_diff = hrc_cnt - hxt_cnt;
        }
        if (cnt_diff < 64)
        {
            break;
        }
        step_unit >>= 1 ;
    }
    //printf("Calibration result : hxt_cnt = %d, hrc_cnt = %d, cnt_diff=%d\n",hxt_cnt, hrc_cnt, cnt_diff  );
    if (cnt_diff > 160)
    {
        return HAL_ERROR;
    }
    else
    {
        return HAL_OK;
    }
}
#endif /* SF32LB52X */

__weak void HAL_RCC_MspInit(void)
{
}


void HAL_RCC_Init(void)
{
#ifdef SOC_BF0_HCPU
    HAL_RCC_DisableModule(RCC_MOD_EPIC);
    HAL_RCC_DisableModule(RCC_MOD_EZIP);

#ifdef SF32LB58X
    HAL_RCC_DisableModule(RCC_MOD_ACPU);
    HAL_RCC_DisableModule(RCC_MOD_GPU);
#endif /* SF32LB58X */

#ifdef SF32LB55X
    HAL_RCC_DisableModule(RCC_MOD_NNACC);
#endif /* SF32LB55X */

#if defined(SF32LB52X)
    HAL_RCC_HCPU_DeepWFIClockSelect(true, RCC_SYSCLK_HRC48);
    HAL_RCC_HCPU_SetDeepWFIDiv(12, 0, 0);

    /* select RC48 as clock source, RC48 has been calibrated */
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HRC48);
#endif /* SF32LB52X */

#else
#endif /* SOC_BF0_HCPU */

    HAL_RCC_MspInit();
}

#ifndef SF32LB55X
void HAL_RCC_HCPU_SetDeepWFIDiv(int8_t div, int8_t pdiv1, int8_t pdiv2)
{
    uint32_t mask = 0;
    uint32_t reg = 0;

    if (div > 0)
    {
        mask = HPSYS_RCC_DWCFGR_HDIV_Msk;
        reg = MAKE_REG_VAL(div, HPSYS_RCC_DWCFGR_HDIV_Msk, HPSYS_RCC_DWCFGR_HDIV_Pos);
    }
    if (pdiv1 >= 0)
    {
        mask |= HPSYS_RCC_DWCFGR_PDIV1_Msk;
        reg |= MAKE_REG_VAL(pdiv1, HPSYS_RCC_DWCFGR_PDIV1_Msk, HPSYS_RCC_CFGR_PDIV1_Pos);
    }
    if (pdiv2 >= 0)
    {
        mask |= HPSYS_RCC_DWCFGR_PDIV2_Msk;
        reg |= MAKE_REG_VAL(pdiv2, HPSYS_RCC_DWCFGR_PDIV2_Msk, HPSYS_RCC_CFGR_PDIV2_Pos);
    }
    if (mask)
    {
        MODIFY_REG(hwp_hpsys_rcc->DWCFGR, mask, reg);
    }
}

void HAL_RCC_HCPU_GetDeepWFIDiv(int *div, int *pdiv1, int *pdiv2)
{
    uint32_t val;

    val = hwp_hpsys_rcc->DWCFGR;
    if (div)
    {
        *div = GET_REG_VAL(val, HPSYS_RCC_DWCFGR_HDIV_Msk, HPSYS_RCC_DWCFGR_HDIV_Pos);
    }
    if (pdiv1)
    {
        *pdiv1 = GET_REG_VAL(val, HPSYS_RCC_DWCFGR_PDIV1_Msk, HPSYS_RCC_DWCFGR_PDIV1_Pos);
    }
    if (pdiv2)
    {
        *pdiv2 = GET_REG_VAL(val, HPSYS_RCC_DWCFGR_PDIV2_Msk, HPSYS_RCC_DWCFGR_PDIV2_Pos);
    }
}


__HAL_ROM_USED void HAL_RCC_HCPU_DeepWFIClockSelect(bool sys_clk, uint32_t sys_clk_src)
{

    hwp_hpsys_rcc->DWCFGR &= ~HPSYS_RCC_DWCFGR_SEL_SYS_LP;
    MODIFY_REG(hwp_hpsys_rcc->DWCFGR, HPSYS_RCC_DWCFGR_SEL_SYS_Msk,
               MAKE_REG_VAL(sys_clk_src, HPSYS_RCC_DWCFGR_SEL_SYS_Msk, HPSYS_RCC_DWCFGR_SEL_SYS_Pos));

}
#endif /* SF32LB55X */

#if defined(SF32LB56X)
void HAL_RCC_Reset_DMAC2_and_MPI5()
{
    hwp_lpsys_rcc->RSTR1 = LPSYS_RCC_RSTR1_DMAC2;
    while (!hwp_lpsys_rcc->RSTR1)
        hwp_lpsys_rcc->RSTR1 &= ~LPSYS_RCC_RSTR1_DMAC2;

    hwp_lpsys_rcc->RSTR2 = LPSYS_RCC_RSTR2_MPI5;
    while (!hwp_lpsys_rcc->RSTR2)
        hwp_lpsys_rcc->RSTR2 &= ~LPSYS_RCC_RSTR2_MPI5;
}
#endif

#if defined(SF32LB58X)
void HAL_RCC_Reset_DMAC3_and_MPI5()
{
    hwp_lpsys_rcc->RSTR1 = LPSYS_RCC_RSTR1_DMAC3;
    while (!hwp_lpsys_rcc->RSTR1)
        hwp_lpsys_rcc->RSTR1 &= ~LPSYS_RCC_RSTR1_DMAC3;

    hwp_lpsys_rcc->RSTR2 = LPSYS_RCC_RSTR2_MPI5;
    while (!hwp_lpsys_rcc->RSTR2)
        hwp_lpsys_rcc->RSTR2 &= ~LPSYS_RCC_RSTR2_MPI5;
}
#endif

#if defined(SF32LB52X) && defined(SOC_BF0_HCPU)
__STATIC_INLINE void HAL_RCC_HCPU_ConfigSxModeVolt(HPSYS_DvfsModeTypeDef new_dvfs_mode)
{
    uint8_t vout_ref;
    HAL_StatusTypeDef res;

    /* configure BUCK voltage */
    MODIFY_REG(hwp_pmuc->BUCK_VOUT, PMUC_BUCK_VOUT_VOUT_Msk,
               MAKE_REG_VAL(hpsys_dvfs_config[new_dvfs_mode].buck, PMUC_BUCK_VOUT_VOUT_Msk, PMUC_BUCK_VOUT_VOUT_Pos));

    /* configure LDO voltage */
    if (HPSYS_DVFS_MODE_S0 == new_dvfs_mode)
    {
        res = HAL_PMU_GetHpsysVoutRef(&vout_ref);
        if (HAL_OK != res)
        {
            vout_ref = hpsys_dvfs_config[HPSYS_DVFS_MODE_S0].ldo;
        }
    }
    else
    {
        res = HAL_PMU_GetHpsysVoutRef2(&vout_ref);
        if (HAL_OK != res)
        {
            vout_ref = hpsys_dvfs_config[HPSYS_DVFS_MODE_S1].ldo;
        }
    }
    MODIFY_REG(hwp_pmuc->HPSYS_VOUT, PMUC_HPSYS_VOUT_VOUT_Msk,
               MAKE_REG_VAL(vout_ref, PMUC_HPSYS_VOUT_VOUT_Msk, PMUC_HPSYS_VOUT_VOUT_Pos));
}

static HAL_StatusTypeDef HAL_RCC_HCPU_SwitchDvfsD2S(HPSYS_DvfsModeTypeDef new_dvfs_mode, uint32_t freq_in_mhz)
{
    HAL_StatusTypeDef r = HAL_OK;
    uint8_t div;

    HAL_RCC_HCPU_ConfigSxModeVolt(new_dvfs_mode);

    /* switch to S mode */
    hwp_hpsys_cfg->SYSCR &= ~HPSYS_CFG_SYSCR_LDO_VSEL_Msk;

    /* buck need 250us to settle */
    HAL_Delay_us(250);

    if (freq_in_mhz <= 48)
    {
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
        div = 48 / freq_in_mhz;
        HAL_ASSERT(div > 0);
        HAL_RCC_HCPU_SetDiv(div, 0, 4);
    }
    else
    {
        HAL_HPAON_EnableXT48();
        /* configure memory param */
        hwp_hpsys_cfg->ULPMCR = hpsys_dvfs_config[new_dvfs_mode].ulpmcr;
        r = HAL_RCC_HCPU_EnableDLL1(freq_in_mhz * 1000000);
        if (HAL_OK != r)
        {
            goto __EXIT;
        }
        /* update pdiv first to avoid pclk exceeding 120MHz */
        HAL_RCC_HCPU_SetDiv(1, 1, 6);
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
    }

__EXIT:
    return r;
}

static HAL_StatusTypeDef HAL_RCC_HCPU_SwitchDvfsS2D(HPSYS_DvfsModeTypeDef new_dvfs_mode, uint32_t freq_in_mhz)
{
    uint8_t div;
    uint8_t vout_ref;
    HAL_StatusTypeDef res;
    int32_t vref;

    /* configure BUCK voltage */
    MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
               MAKE_REG_VAL(hpsys_dvfs_config[new_dvfs_mode].buck, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
    /* configure LDO voltage */
    res = HAL_PMU_GetHpsysVoutRef(&vout_ref);
    if (HAL_OK != res)
    {
        vout_ref = hpsys_dvfs_config[HPSYS_DVFS_MODE_S0].ldo;
    }
    vref = (int32_t)hpsys_dvfs_config[new_dvfs_mode].ldo_offset + (int32_t)vout_ref;
    MODIFY_REG(hwp_pmuc->HPSYS_LDO, PMUC_HPSYS_LDO_VREF_Msk,
               MAKE_REG_VAL(vref, PMUC_HPSYS_LDO_VREF_Msk, PMUC_HPSYS_LDO_VREF_Pos));

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
    div = 48 / freq_in_mhz;
    HAL_ASSERT(div > 0);
    HAL_RCC_HCPU_SetDiv(div, 0, 4);

    HAL_RCC_HCPU_DisableDLL1();
    HAL_HPAON_DisableXT48();

    /* configure memory param */
    hwp_hpsys_cfg->ULPMCR = hpsys_dvfs_config[new_dvfs_mode].ulpmcr;
    /* switch to D mode */
    hwp_hpsys_cfg->SYSCR |= HPSYS_CFG_SYSCR_LDO_VSEL_Msk;

    return HAL_OK;
}

static HAL_StatusTypeDef HAL_RCC_HCPU_ConfigDvfs(uint32_t freq_in_mhz, HPSYS_DvfsModeTypeDef new_dvfs_mode)
{
    HAL_StatusTypeDef r = HAL_OK;
    uint8_t div;
    uint32_t dll2_limit;

    switch (curr_dvfs_mode)
    {
    case HPSYS_DVFS_MODE_S0:
    case HPSYS_DVFS_MODE_S1:
    {
        dll2_limit = hpsys_dll2_limit[new_dvfs_mode];
        /* check whether DLL2 freq exceeds limit of target dvfs mode */
        if (HAL_RCC_HCPU_GetDLL2Freq() > dll2_limit)
        {
            r = HAL_ERROR;
            goto __EXIT;
        }

        if (HPSYS_DVFS_MODE_D1 >= new_dvfs_mode)
        {

            HAL_RCC_HCPU_SwitchDvfsS2D(new_dvfs_mode, freq_in_mhz);
        }
        else
        {
            /* switch between different S mode */
            HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);

            HAL_RCC_HCPU_ConfigSxModeVolt(new_dvfs_mode);

            /* buck need 250us to settle */
            HAL_Delay_us(250);

            HAL_RCC_HCPU_DisableDLL1();

            if (freq_in_mhz <= 48)
            {
                HAL_HPAON_DisableXT48();
                HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
                div = 48 / freq_in_mhz;
                HAL_ASSERT(div > 0);
                HAL_RCC_HCPU_SetDiv(div, 0, 4);
            }
            else
            {
                HAL_HPAON_EnableXT48();
                r = HAL_RCC_HCPU_EnableDLL1(freq_in_mhz * 1000000);
                if (HAL_OK != r)
                {
                    goto __EXIT;
                }

                /* update pdiv first to avoid pclk exceeding 120MHz */
                HAL_RCC_HCPU_SetDiv(1, 1, 6);
                HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
            }
        }
        break;
    }
    case HPSYS_DVFS_MODE_D0:
    case HPSYS_DVFS_MODE_D1:
    {
        if (HPSYS_DVFS_MODE_S0 <= new_dvfs_mode)
        {
            r = HAL_RCC_HCPU_SwitchDvfsD2S(new_dvfs_mode, freq_in_mhz);
        }
        else if (new_dvfs_mode == curr_dvfs_mode)
        {
            HAL_RCC_HCPU_SetDiv((48 / freq_in_mhz), -1, -1);
        }
        else
        {
            r = HAL_RCC_HCPU_SwitchDvfsD2S(HPSYS_DVFS_MODE_S0, HPSYS_DVFS_MODE_S0_LIMIT);
            if (HAL_OK != r)
            {
                goto __EXIT;
            }

            r = HAL_RCC_HCPU_SwitchDvfsS2D(new_dvfs_mode, freq_in_mhz);
        }
        break;
    }
    default:
    {
        r = HAL_ERROR;
        goto __EXIT;
    }
    }

    curr_dvfs_mode = new_dvfs_mode;
    curr_freq_in_mhz = freq_in_mhz;
    HAL_Delay_us(0);

__EXIT:
    return r;
}


HAL_StatusTypeDef HAL_RCC_HCPU_ConfigHCLK(uint32_t freq_in_mhz)
{
    HPSYS_DvfsModeTypeDef new_dvfs_mode;
    HAL_StatusTypeDef r = HAL_OK;

    if (freq_in_mhz > HPSYS_DVFS_MODE_S1_LIMIT)
    {
        return HAL_ERROR;
    }

    if (freq_in_mhz > HPSYS_DVFS_MODE_S0_LIMIT)
    {
        new_dvfs_mode = HPSYS_DVFS_MODE_S1;
    }
    else if (freq_in_mhz > HPSYS_DVFS_MODE_D1_LIMIT)
    {
        new_dvfs_mode = HPSYS_DVFS_MODE_S0;
    }
    else if (freq_in_mhz > HPSYS_DVFS_MODE_D0_LIMIT)
    {
        new_dvfs_mode = HPSYS_DVFS_MODE_D1;
    }
    else
    {
        new_dvfs_mode = HPSYS_DVFS_MODE_D0;
    }

    r = HAL_RCC_HCPU_ConfigDvfs(freq_in_mhz, new_dvfs_mode);

    return r;
}

HAL_StatusTypeDef HAL_RCC_HCPU_ConfigHCLKByMode(uint32_t freq_in_mhz, HPSYS_DvfsModeTypeDef mode)
{
    HAL_StatusTypeDef r = HAL_OK;
    uint32_t limit;

    if (HPSYS_DVFS_MODE_D0 == mode)
    {
        limit = HPSYS_DVFS_MODE_D0_LIMIT;
    }
    else if (HPSYS_DVFS_MODE_D1 == mode)
    {
        limit = HPSYS_DVFS_MODE_D1_LIMIT;
    }
    else if (HPSYS_DVFS_MODE_S0 == mode)
    {
        limit = HPSYS_DVFS_MODE_S0_LIMIT;
    }
    else if (HPSYS_DVFS_MODE_S1 == mode)
    {
        limit = HPSYS_DVFS_MODE_S1_LIMIT;
    }
    else
    {
        r = HAL_ERROR;
        goto __EXIT;
    }

    if (freq_in_mhz > limit)
    {
        r = HAL_ERROR;
        goto __EXIT;
    }

    r = HAL_RCC_HCPU_ConfigDvfs(freq_in_mhz, mode);

__EXIT:
    return r;
}

HPSYS_DvfsModeTypeDef HAL_RCC_HCPU_GetCurrentDvfsMode(void)
{
    return curr_dvfs_mode;
}

#endif /* SF32LB52X && SOC_BF0_HCPU */

#if defined(SF32LB56X) && defined(SOC_BF0_HCPU)

HAL_StatusTypeDef HAL_RCC_HCPU_ConfigHCLK(uint32_t freq_in_mhz)
{
    HAL_StatusTypeDef r = HAL_OK;
    PMU_HpsysLdoVoltTypeDef volt;
    uint32_t level;

    level = HAL_RCC_DisableInterrupt();
    if ((240 != freq_in_mhz) && (288 != freq_in_mhz) && (312 != freq_in_mhz))
    {
        HAL_ASSERT(0);
        r = HAL_ERROR;
        goto __EXIT;
    }

    if (SysTick->CTRL & SYSTICK_CLKSOURCE_HCLK)
    {
        /* Update systick config as hclk changes */
        if (0 != HAL_SYSTICK_Config(freq_in_mhz * 1000000 / HAL_TICK_PER_SECOND))
        {
            HAL_ASSERT(0);
        }
    }

    if (freq_in_mhz > 240)
    {
        volt = PMU_HPSYS_LDO_VOLT_HIGH;
    }
    else
    {
        volt = PMU_HPSYS_LDO_VOLT_NORMAL;
    }

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);

    HAL_PMU_ConfigHpsysLdoVolt(volt);
    /* TODO:  buck need 250us to settle */
    HAL_Delay_us(250);

    HAL_RCC_HCPU_DisableDLL1();
    r = HAL_RCC_HCPU_EnableDLL1(freq_in_mhz * 1000000);
    if (HAL_OK != r)
    {
        goto __EXIT;
    }

    HAL_RCC_HCPU_SetDiv(1, -1, -1);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);

    HAL_Delay_us(0);

__EXIT:

    HAL_RCC_EnableInterrupt(level);

    return r;
}

#endif /* SF32LB56X && SOC_BF0_HCPU */


#if defined(SF32LB52X) && defined(SOC_BF0_LCPU)

static HAL_StatusTypeDef HAL_RCC_LCPU_ConfigDvfs(uint32_t freq_in_mhz, LPSYS_DvfsModeTypeDef new_dvfs_mode)
{
    uint8_t div;

    if (LPSYS_DVFS_MODE_D == curr_dvfs_mode)
    {
        if (new_dvfs_mode != curr_dvfs_mode)
        {
            hwp_lpsys_cfg->SYSCR |= LPSYS_CFG_SYSCR_LDO_VSEL_Msk;
            hwp_lpsys_cfg->ULPMCR = lpsys_dvfs_config[new_dvfs_mode].ulpmcr;
        }
        div = 48 / freq_in_mhz;
        HAL_ASSERT(div > 0);
        HAL_RCC_LCPU_SetDiv(div, -1, -1);
    }
    else
    {
        div = 48 / freq_in_mhz;
        HAL_ASSERT(div > 0);
        HAL_RCC_LCPU_SetDiv(div, -1, -1);

        if (new_dvfs_mode != curr_dvfs_mode)
        {
            hwp_lpsys_cfg->ULPMCR = lpsys_dvfs_config[new_dvfs_mode].ulpmcr;
            hwp_lpsys_cfg->SYSCR &= ~LPSYS_CFG_SYSCR_LDO_VSEL_Msk;
        }
    }

    curr_dvfs_mode = new_dvfs_mode;
    curr_freq_in_mhz = freq_in_mhz;
    HAL_Delay_us(0);

    return HAL_OK;
}


HAL_StatusTypeDef HAL_RCC_LCPU_ConfigHCLK(uint32_t freq_in_mhz)
{
    LPSYS_DvfsModeTypeDef new_dvfs_mode;
    HAL_StatusTypeDef r = HAL_OK;

    if (freq_in_mhz > LPSYS_DVFS_MODE_D_LIMIT)
    {
        new_dvfs_mode = LPSYS_DVFS_MODE_S;
    }
    else
    {
        new_dvfs_mode = LPSYS_DVFS_MODE_D;
    }

    r = HAL_RCC_LCPU_ConfigDvfs(freq_in_mhz, new_dvfs_mode);

    return r;
}

HAL_StatusTypeDef HAL_RCC_LCPU_ConfigHCLKByMode(uint32_t freq_in_mhz, LPSYS_DvfsModeTypeDef mode)
{
    HAL_StatusTypeDef r = HAL_OK;
    uint32_t limit;

    if (LPSYS_DVFS_MODE_D == mode)
    {
        limit = LPSYS_DVFS_MODE_D_LIMIT;
    }
    else if (LPSYS_DVFS_MODE_S == mode)
    {
        limit = LPSYS_DVFS_MODE_S_LIMIT;
    }
    else
    {
        r = HAL_ERROR;
        goto __EXIT;
    }

    if (freq_in_mhz > limit)
    {
        r = HAL_ERROR;
        goto __EXIT;
    }

    r = HAL_RCC_LCPU_ConfigDvfs(freq_in_mhz, mode);

__EXIT:

    return r;
}

LPSYS_DvfsModeTypeDef HAL_RCC_LCPU_GetCurrentDvfsMode(void)
{
    return curr_dvfs_mode;
}

#endif /* SF32LB52X && SOC_BF0_LCPU */


#endif /* HAL_RCC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
