/**
  ******************************************************************************
  * @file   bf0_hal_rcc.h
  * @author Sifli software development team
  * @brief   Header file of RCC HAL module.
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_RCC_H
#define __BF0_HAL_RCC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup RCC Reset and control
  * @ingroup BF0_HAL_Driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup RCC_Exported_Types RCC Exported Types
  * @{
  *
  * @brief  RCC System, lock configuration definition
  */

// Core ID
#define CORE_ID_DEFAULT 0
/** HCPU */
#define CORE_ID_HCPU    1
/** LCPU */
#define CORE_ID_LCPU    2
#ifdef SF32LB58X
/** ACPU */
#define CORE_ID_ACPU    3
#endif /* SF32LB58X */

#ifdef SOC_BF0_HCPU
#define CORE_ID_CURRENT CORE_ID_HCPU
#elif defined(SOC_BF0_LCPU)
#define CORE_ID_CURRENT CORE_ID_LCPU
#else
#error "Core ID is not defined."
#endif
// Sysclock
#define  RCC_SYSCLK_HRC48   0
#define  RCC_SYSCLK_HXT48   1
#ifdef SF32LB55X
#define  RCC_SYSCLK_CLK_LP  2
#else
/* HPSYS clock source */
#define  RCC_SYSCLK_CLK_HRC100  2
/* LPSYS clock source */
#define  RCC_SYSCLK_DBL96       2
#endif
#define  RCC_SYSCLK_DLL1    3   // 192M

// Flash clock
#define  RCC_CLK_FLASH_SYSCLK   0   // 0/1 all referred to sysclock
#define  RCC_CLK_SRC_SYS      0
#define  RCC_CLK_SRC_DLL1     1
#define  RCC_CLK_SRC_DLL2     2
#define  RCC_CLK_SRC_DLL3     3
#define  RCC_CLK_FLASH_DLL2   RCC_CLK_SRC_DLL2      // Backward comaptible to Z0  
#define  RCC_CLK_FLASH_DLL3   RCC_CLK_SRC_DLL3      // Backward comaptible to Z0  


// PSRAM clock
#define  RCC_CLK_PSRAM_SYSCLK   (0)
#define  RCC_CLK_PSRAM_DLL2     (RCC_CLK_SRC_DLL2)
#define  RCC_CLK_PSRAM_DLL3     (RCC_CLK_SRC_DLL3)

#ifdef SF32LB55X
// USART clock
#define  RCC_CLK_USART_HRC48    0
#define  RCC_CLK_USART_HXT48    1
#else
#define  RCC_CLK_PERI_HRC48     0
#define  RCC_CLK_PERI_HXT48     1
#endif /* SF32LB55X */

// I2S clock
#define  RCC_CLK_I2S_SYSCLK     0
#define  RCC_CLK_I2S_DLL2       1

// MPI / SDMMC clock
#define RCC_CLK_MPI_SD_SYSCLK       0
#define RCC_CLK_MPI_SD_DLL2         2
#define RCC_CLK_MPI_SD_DLL3         3

// For All  core
#define  RCC_CLK_MOD_SYS        0

// For HCPU
#if defined(SF32LB55X)
#define  RCC_CLK_MOD_PSRAM      HPSYS_RCC_CSR_SEL_PSRAMC_Pos
#define  RCC_CLK_MOD_FLASH1     HPSYS_RCC_CSR_SEL_QSPI1_Pos
#define  RCC_CLK_MOD_FLASH2     HPSYS_RCC_CSR_SEL_QSPI2_Pos
#define  RCC_CLK_MOD_FLASH3     HPSYS_RCC_CSR_SEL_QSPI3_Pos
#else
#define  RCC_CLK_MOD_FLASH1     HPSYS_RCC_CSR_SEL_MPI1_Pos
#define  RCC_CLK_MOD_FLASH2     HPSYS_RCC_CSR_SEL_MPI2_Pos
#define  RCC_CLK_MOD_FLASH3     HPSYS_RCC_CSR_SEL_MPI3_Pos
#ifdef HPSYS_RCC_CSR_SEL_MPI4_Pos
#define  RCC_CLK_MOD_FLASH4     HPSYS_RCC_CSR_SEL_MPI4_Pos
#endif

#define  RCC_CLK_MOD_PSRAM1     HPSYS_RCC_CSR_SEL_MPI1_Pos
#define  RCC_CLK_MOD_PSRAM2     HPSYS_RCC_CSR_SEL_MPI2_Pos
#ifdef HPSYS_RCC_CSR_SEL_MPI3_Pos
#define  RCC_CLK_MOD_PSRAM3     HPSYS_RCC_CSR_SEL_MPI3_Pos
#endif /* HPSYS_RCC_CSR_SEL_MPI3_Pos */
#ifdef HPSYS_RCC_CSR_SEL_MPI4_Pos
#define  RCC_CLK_MOD_PSRAM4     HPSYS_RCC_CSR_SEL_MPI4_Pos
#endif
#define  RCC_CLK_MOD_PSRAM      RCC_CLK_MOD_PSRAM1

#define  RCC_CLK_MOD_SDMMC      HPSYS_RCC_CSR_SEL_SDMMC_Pos

#endif

// For LCPU

#if defined(SF32LB55X)
#define  RCC_CLK_MOD_UART1      HPSYS_RCC_CSR_SEL_USART1_Pos
#define  RCC_CLK_MOD_UART2      HPSYS_RCC_CSR_SEL_USART2_Pos
#define  RCC_CLK_MOD_UART3      LPSYS_RCC_CSR_SEL_USART3_Pos
#define  RCC_CLK_MOD_UART4      LPSYS_RCC_CSR_SEL_USART4_Pos
#define  RCC_CLK_MOD_UART5      LPSYS_RCC_CSR_SEL_USART5_Pos
#else
#define RCC_CLK_MOD_HP_PERI     HPSYS_RCC_CSR_SEL_PERI_Pos
#define RCC_CLK_MOD_LP_PERI     LPSYS_RCC_CSR_SEL_PERI_Pos
#endif

#define RCC_CLK_TICK_CLK_LP     0
#define RCC_CLK_TICK_HRC48      2
#define RCC_CLK_TICK_HXT48      3

#ifdef HPSYS_RCC_CSR_SEL_TICK_Pos
#define RCC_CLK_MOD_HP_TICK        HPSYS_RCC_CSR_SEL_TICK_Pos
#endif /* HPSYS_RCC_CSR_SEL_TICK_Pos */

#ifdef LPSYS_RCC_CSR_SEL_TICK_Pos
#define RCC_CLK_MOD_LP_TICK        LPSYS_RCC_CSR_SEL_TICK_Pos
#endif /* LPSYS_RCC_CSR_SEL_TICK_Pos */


#ifdef SF32LB55X
typedef enum
{
    RCC_MOD_DMAC1,
    RCC_MOD_DMAC2,
    RCC_MOD_MAILBOX1,
    RCC_MOD_MAILBOX2,
    RCC_MOD_PINMUX1,
    RCC_MOD_PINMUX2,
    RCC_MOD_USART1,
    RCC_MOD_USART2,
    RCC_MOD_USART3,
    RCC_MOD_USART4,
    RCC_MOD_USART5,
    RCC_MOD_EZIP,
    RCC_MOD_EPIC,
    RCC_MOD_LCDC1,
    RCC_MOD_LCDC2,
    RCC_MOD_I2S1,
    RCC_MOD_I2S2,
    RCC_MOD_SYSCFG1,
    RCC_MOD_SYSCFG2,
    RCC_MOD_EFUSEC,
    RCC_MOD_AES,
    RCC_MOD_CRC,
    RCC_MOD_TRNG,
    RCC_MOD_GPTIM1,
    RCC_MOD_GPTIM2,
    RCC_MOD_GPTIM3,
    RCC_MOD_GPTIM4,
    RCC_MOD_GPTIM5,
    RCC_MOD_BTIM1,
    RCC_MOD_BTIM2,
    RCC_MOD_BTIM3,
    RCC_MOD_BTIM4,
    RCC_MOD_WDT1,
    RCC_MOD_WDT2,
    RCC_MOD_SPI1,
    RCC_MOD_SPI2,
    RCC_MOD_SPI3,
    RCC_MOD_SPI4,
    RCC_MOD_EXTDMA,
    RCC_MOD_PSRAMC,
    RCC_MOD_NNACC,
    RCC_MOD_PDM1,
    RCC_MOD_PDM2,
    RCC_MOD_I2C1,
    RCC_MOD_I2C2,
    RCC_MOD_I2C3,
    RCC_MOD_I2C4,
    RCC_MOD_I2C5,
    RCC_MOD_I2C6,
    RCC_MOD_DSIHOST,
    RCC_MOD_DSIPHY,
    RCC_MOD_DSI,
    RCC_MOD_PTC1,
    RCC_MOD_PTC2,
    RCC_MOD_GPIO1,
    RCC_MOD_GPIO2,
    RCC_MOD_QSPI1,
    RCC_MOD_QSPI2,
    RCC_MOD_QSPI3,
    RCC_MOD_QSPI4,
    RCC_MOD_SDMMC1,

    RCC_MOD_SDMMC2,
    RCC_MOD_USBC,
    RCC_MOD_BUSMON1,
    RCC_MOD_BUSMON2,
    RCC_MOD_KEYPAD,
    RCC_MOD_GPDAC,
    RCC_MOD_ATIM1,
    RCC_MOD_ATIM2,
    RCC_MOD_GAU,
    RCC_MOD_FLASHC1,
    RCC_MOD_FLASHC2,
    RCC_MOD_FLASHC3,


    RCC_MOD_TSEN,
    RCC_MOD_LPCOMP,
    RCC_MOD_SDADC,
    RCC_MOD_GPADC,

    RCC_MOD_PATCH,
    RCC_MOD_MAC,
    RCC_MOD_PHY,
    RCC_MOD_RFC,
} RCC_MODULE_TYPE;
#else
typedef enum
{
    RCC_MOD_DMAC1,
    RCC_MOD_DMAC2,
    RCC_MOD_DMAC3,
    RCC_MOD_MAILBOX1,
    RCC_MOD_MAILBOX2,
    RCC_MOD_PINMUX1,
    RCC_MOD_PINMUX2,
    RCC_MOD_USART1,
    RCC_MOD_USART2,
    RCC_MOD_USART3,
    RCC_MOD_USART4,
    RCC_MOD_USART5,
    RCC_MOD_USART6,
    RCC_MOD_EZIP,
    RCC_MOD_RESERVED, // Reserved for ROM compatible
    RCC_MOD_EPIC,
    RCC_MOD_LCDC1,
    RCC_MOD_LCDC2,
    RCC_MOD_I2S1,
    RCC_MOD_I2S2,
    RCC_MOD_I2S3,
    RCC_MOD_SYSCFG1,
    RCC_MOD_SYSCFG2,
    RCC_MOD_EFUSEC,
    RCC_MOD_AES,
    RCC_MOD_CRC1,
    RCC_MOD_TRNG,
    RCC_MOD_GPTIM1,
    RCC_MOD_GPTIM2,
    RCC_MOD_GPTIM3,
    RCC_MOD_GPTIM4,
    RCC_MOD_GPTIM5,
    RCC_MOD_BTIM1,
    RCC_MOD_BTIM2,
    RCC_MOD_BTIM3,
    RCC_MOD_BTIM4,
    RCC_MOD_SPI1,
    RCC_MOD_SPI2,
    RCC_MOD_SPI3,
    RCC_MOD_SPI4,
    RCC_MOD_EXTDMA,
    RCC_MOD_NNACC1,
    RCC_MOD_NNACC2,
    RCC_MOD_PDM1,
    RCC_MOD_PDM2,
    RCC_MOD_I2C1,
    RCC_MOD_I2C2,
    RCC_MOD_I2C3,
    RCC_MOD_I2C4,
    RCC_MOD_I2C5,
    RCC_MOD_I2C6,
    RCC_MOD_I2C7,
    RCC_MOD_DSIHOST,
    RCC_MOD_DSIPHY,
    RCC_MOD_DSI,
    RCC_MOD_PTC1,
    RCC_MOD_PTC2,
    RCC_MOD_GPIO1,
    RCC_MOD_GPIO2,
    RCC_MOD_MPI1,
    RCC_MOD_MPI2,
    RCC_MOD_MPI3,
    RCC_MOD_MPI4,
    RCC_MOD_MPI5,
    RCC_MOD_SDMMC1,
    RCC_MOD_SDMMC2,
    RCC_MOD_USBC,
    RCC_MOD_BUSMON1,
    RCC_MOD_BUSMON2,
    RCC_MOD_BUSMON3,
    RCC_MOD_ATIM1,
    RCC_MOD_ATIM2,
    RCC_MOD_FFT1,
    RCC_MOD_FFT2,
    RCC_MOD_FACC1,
    RCC_MOD_FACC2,
    RCC_MOD_CAN1,
    RCC_MOD_CAN2,
    RCC_MOD_AUDCODEC_HP,
    RCC_MOD_AUDCODEC_LP,
    RCC_MOD_AUDPRC,
    RCC_MOD_GPU,
    RCC_MOD_JENC,
    RCC_MOD_JDEC,
    RCC_MOD_ACPU,
    RCC_MOD_TSEN,
    RCC_MOD_LPCOMP,
    RCC_MOD_SDADC,
    RCC_MOD_GPADC,
    RCC_MOD_PATCH,
    RCC_MOD_MAC,
    RCC_MOD_PHY,
    RCC_MOD_RFC,
    RCC_MOD_CRC2,
    RCC_MOD_SCI,
    RCC_MOD_SECU1,
    RCC_MOD_SECU2,
    RCC_MOD_AUDCODEC,
} RCC_MODULE_TYPE;
#endif

#define RCC_MOD_I2S_ALL     RCC_MOD_I2S1

#if defined(SF32LB52X)
typedef enum
{
    HPSYS_DVFS_MODE_D0,
    HPSYS_DVFS_MODE_D1,
    HPSYS_DVFS_MODE_S0,
    HPSYS_DVFS_MODE_S1,
    HPSYS_DVFS_MODE_NUM,
} HPSYS_DvfsModeTypeDef;

typedef enum
{
    LPSYS_DVFS_MODE_D,
    LPSYS_DVFS_MODE_S,
    LPSYS_DVFS_MODE_NUM,
} LPSYS_DvfsModeTypeDef;
#endif /* SF32LB52X */

///@} RCC_Exported_Types

/***********************************For HCPU****************************************/
/**
  * @brief  Reset RCC module for HCPU.
  * @param  modules modules to be reset, defined in hpsys_rcc.h, HPSYS_RCC_RSTR_XXX
  * @param  reset 1: reset(bit set), 0:Release reset(bit clear).
  * @retval None
  */

#define HAL_RCC_HCPU_reset(modules, reset) \
{ \
    if (reset) \
        hwp_hpsys_rcc->RSTR1|=(modules);\
    else \
        hwp_hpsys_rcc->RSTR1&=~(modules); \
}

#define HAL_RCC_HCPU_reset2(modules, reset) \
{ \
    if (reset) \
        hwp_hpsys_rcc->RSTR2|=(modules);\
    else \
        hwp_hpsys_rcc->RSTR2&=~(modules); \
}

/**
  * @brief  enable/disable RCC module for HCPU.
  * @param  modules modules to be enabled/disabled. defined in hpsys_rcc.h, HPSYS_RCC_ENR_XXX
  * @param  enabled 1: enable(bit set), 0:disable(bit clear).
  * @retval None
  */

#define HAL_RCC_HCPU_enable(modules,enabled) {\
    if (enabled) \
        hwp_hpsys_rcc->ENR1|=(modules);\
    else \
        hwp_hpsys_rcc->ENR1&=~(modules);\
}

#define HAL_RCC_HCPU_enable2(modules,enabled) {\
    if (enabled) \
        hwp_hpsys_rcc->ENR2|=(modules);\
    else \
        hwp_hpsys_rcc->ENR2&=~(modules);\
}

/**
  * @brief  Release all HPSYS modules
  * @retval None
  */
#define HAL_RCC_HCPU_RELEASE_ALL()  \
    do                              \
    {                               \
        hwp_hpsys_rcc->RSTR1 = 0;   \
        hwp_hpsys_rcc->RSTR2 = 0;   \
    }                               \
    while (0)


/**
  * @brief  Select clock source for HCPU HW module.
  * @param  clk_module module for clock, RCC_CLK_MOD_XXX
  * @param  src clock source.
  * @retval None
  */
void HAL_RCC_HCPU_ClockSelect(int clk_module, int src);

/**
 * @brief  Get clock source of HCPU HW module.
 * @param  clk_module module for clock, RCC_CLK_MOD_XXX
 * @retval clock source,
 */
int HAL_RCC_HCPU_GetClockSrc(int clk_module);


/**
 * @brief  Set divider for clock of HCPU.
 * @param  div main divider HCLK=HPCLK/div
 * @param  pdiv1 divider for pclk1= HCLK/ (2^PDIV1) default 1
 * @param  pdiv2 divider for pclk2= HCLK/ (2^PDIV2) default 5
 * @retval None
 */
void HAL_RCC_HCPU_SetDiv(int div, int pdiv1, int pdiv2);

/**
 * @brief  Get divider for clock of HCPU.
 * @param  div pointer to main divider, HCLK=HPCLK/div
 * @param  pdiv1 pointer to divider for pclk1= HCLK/ (2^PDIV1) default 1
 * @param  pdiv2 pointer to divider for pclk2= HCLK/ (2^PDIV2) default 5
 * @retval None
 */
void HAL_RCC_HCPU_GetDiv(int *div, int *pdiv1, int *pdiv2);


/**
  * @brief  Enable DLL1
  * @param  freq Freqency of DLL1
  * @retval HAL_OK if successful, Otherwise HAL_ERROR
  */
HAL_StatusTypeDef HAL_RCC_HCPU_EnableDLL1(uint32_t freq);

/**
  * @brief  Disable DLL1
  * @retval HAL_OK if successful, Otherwise HAL_ERROR
  */
HAL_StatusTypeDef HAL_RCC_HCPU_DisableDLL1(void);

/**
  * @brief  Get DLL1 freqency
  * @retval Frequency of DLL1, 0 if not enabled
  */
uint32_t HAL_RCC_HCPU_GetDLL1Freq(void);

/**
  * @brief  Enable DLL2
  * @param  freq Freqency of DLL2
  * @retval HAL_OK if successful, Otherwise HAL_ERROR
  */
HAL_StatusTypeDef HAL_RCC_HCPU_EnableDLL2(uint32_t freq);

/**
  * @brief  Disable DLL2
  * @retval HAL_OK if successful, Otherwise HAL_ERROR
  */
HAL_StatusTypeDef HAL_RCC_HCPU_DisableDLL2(void);

/**
  * @brief  Get DLL2 freqency
  * @retval Frequency of DLL2, 0 if not enabled
  */
uint32_t HAL_RCC_HCPU_GetDLL2Freq(void);


uint32_t HAL_RCC_HCPU_GetDLL3Freq(void);

/**
  * @brief  Enable DLL3
  * @param  freq Freqency of DLL3
  * @retval HAL_OK if successful, Otherwise HAL_ERROR
  */
HAL_StatusTypeDef HAL_RCC_HCPU_EnableDLL3(uint32_t freq);

/**
  * @brief  Disable DLL3
  * @retval HAL_OK if successful, Otherwise HAL_ERROR
  */
HAL_StatusTypeDef HAL_RCC_HCPU_DisableDLL3(void);



/***********************************For LCPU****************************************/

/**
  * @brief  Reset RCC module for LCPU.
  * @param  modules modules to be reset, defined in lpsys_rcc.h, LPSYS_RCC_RSTR_XXX
  * @param  reset 1: reset(bit set), 0:Release reset(bit clear).
  * @retval None
  */

#define HAL_RCC_LCPU_reset(modules, reset) { \
    if (reset) \
        hwp_lpsys_rcc->RSTR1|=(modules);\
    else \
        hwp_lpsys_rcc->RSTR1&=~(modules); \
}

/**
  * @brief  Release all LPSYS modules
  * @retval void
  */
#define HAL_RCC_LCPU_RELEASE_ALL()  (hwp_lpsys_rcc->RSTR1 = 0)


/**
  * @brief  enable/disable RCC module for LCPU.
  * @param  modules modules to be enabled/disabled. defined in lpsys_rcc.h, LPSYS_RCC_ENR_XXX
  * @param  enabled 1: enable(bit set), 0:disable(bit clear).
  * @retval None
  */
#define HAL_RCC_LCPU_enable(modules,enabled) {\
    if (enabled) \
        hwp_lpsys_rcc->ENR1|=(modules);\
    else \
        hwp_lpsys_rcc->ENR1&=~(modules);\
}

#define HAL_RCC_LCPU_enable2(modules,enabled) {\
    if (enabled) \
        hwp_lpsys_rcc->ENR2|=(modules);\
    else \
        hwp_lpsys_rcc->ENR2&=~(modules);\
}


/**
  * @brief  Select clock source for LCPU HW module.
  * @param  clk_module module for clock, RCC_CLK_MOD_XXX
  * @param  src clock 0:RCC_SYSCLK_HRC48, 1:RCC_SYSCLK_HXT48
  * @retval None
  */
__STATIC_INLINE void HAL_RCC_LCPU_ClockSelect(int clk_module, int src)
{
    uint32_t csr = hwp_lpsys_rcc->CSR;
    uint32_t mask;

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

    csr &= ~(mask << clk_module);
    hwp_lpsys_rcc->CSR = csr | ((src & mask) << clk_module);
}

#ifdef RCC_SYSCLK_DBL96
/**
 * @brief  LPSYS clock source switch to DBL96
 * @retval None
 */
#define HAL_RCC_LCPU_ClockSelectDBL96()   \
    MODIFY_REG(hwp_lpsys_rcc->CSR, LPSYS_RCC_CSR_SEL_SYS_Msk, MAKE_REG_VAL(RCC_SYSCLK_DBL96, LPSYS_RCC_CSR_SEL_SYS_Msk, LPSYS_RCC_CSR_SEL_SYS_Pos));
#endif /* RCC_SYSCLK_DBL96 */

/**
 * @brief  Get clock source of LCPU HW module.
 * @param  clk_module module for clock, RCC_CLK_MOD_XXX
 * @retval clock source,
 */
int HAL_RCC_LCPU_GetClockSrc(int clk_module);

/**
  * @brief  Set divider for clock of LCPU.
  * @param  div main divider HCLK=LPCLK/div
  * @param  pdiv1 divider for pclk1= HCLK/ (2^PDIV1) default 1
  * @param  pdiv2 divider for pclk2= HCLK/ (2^PDIV2) default 3
  * @retval None
  */
void HAL_RCC_LCPU_SetDiv(int div, int pdiv1, int pdiv2);

/**
 * @brief  Get divider for clock of LCPU.
 * @param  div pointer to hclk divider
 * @param  pdiv1 pointer to pclk1 divider
 * @param  pdiv2 pointer to pclk2 divider
 * @retval None
 */
void HAL_RCC_LCPU_GetDiv(int *div, int *pdiv1, int *pdiv2);

/***********************************For BCPU****************************************/
/**
  * @brief  Reset RCC module for BCPU.
  * @param  modules modules to be reset, defined in ble_rcc.h, BLE_RCC_RSTR_XXX
  * @retval None
  */
#define HAL_RCC_BCPU_reset(modules) hwp_ble_rcc->RSTR|=(modules)

/**
  * @brief  enable/disable RCC module for BCPU.
  * @param  modules modules to be enabled/disabled. defined in ble_rcc.h, BLE_RCC_ENR_XXX
  * @param  enabled 1: enable(bit set), 0:disable(bit clear).
  * @retval None
  */
#define HAL_RCC_BCPU_enable(modules,enabled) {\
    if (enabled) \
        hwp_ble_rcc->ENR|=(modules);\
    else \
        hwp_ble_rcc->ENR&=~(modules); \
}

/**
  * @brief  Select clock source for BCPU HW module.
  * @param  clk_module module for clock
  * @param  src clock 0:RCC_SYSCLK_HRC48, 1:RCC_SYSCLK_HXT48
  * @retval None
  */
#define HAL_RCC_BCPU_ClockSelect(clk_module,src) hwp_ble_rcc->CSR=src

/**
  * @brief  Set divider for clock of BCPU.
  * @param  div main divider HCLK=BLECLK(48M)/div
  * @param  pdiv divider for pclk,
            HCLK not divided; 100 - HCLK divided by 2; 101 - divided by 4; 110 - divided by 8; 111 - divided by 16
  * @param  macdiv divider for MACCLK = HCLK / MACDIV
  * @param  macfreq frequency of BLE MAC master clock
  * @retval None
  */
void HAL_RCC_BCPU_SetDiv(int div, int pdiv, int macdiv, int macfreq);

/********************************For all cores***********************************************/

/**
 * @brief  Get HCLK freq.
 * @param  core_id Core ID
 * @retval HCLK Frequency for Core ID
 */
uint32_t HAL_RCC_GetHCLKFreq(int core_id);

/**
 * @brief  Get SYSCLK freq.
 * @param  core_id Core ID,
 * @retval SYSCLK Frequency for Core ID
 */
uint32_t HAL_RCC_GetSysCLKFreq(int core_id);

/**
 * @brief  Get PCLK freq.
 * @param  core_id Core ID
 * @param  is_pclk1 1: Get pclk1, 0: not PCLK1
 * @retval PCLK freq for specific CORE.
 */
uint32_t HAL_RCC_GetPCLKFreq(int core_id, int is_pclk1);

/**
 * @brief  Release LCPU
 * @retval void
 */
void HAL_RCC_ReleaseLCPU(void);

/**
 * @brief  Reset LCPU
 * @retval void
 */
void HAL_RCC_ResetLCPU(void);

/**
 * @brief  Reset and halt LCPU
 * @param is_init Is in init state.
 * @retval void
 */
void HAL_RCC_Reset_and_Halt_LCPU(uint8_t is_init);


/**
 * @brief  Reset BLE RF
 * @retval void
 */
void HAL_RCC_ResetBluetoothRF(void);


/**
 * @brief  Set Mac freq.
 * @retval void
 */
void HAL_RCC_SetMacFreq(void);

/**
 * @brief  Reset module
 * @param  module module name
 * @retval void
 */
void HAL_RCC_ResetModule(RCC_MODULE_TYPE module);

/**
 * @brief  Enable module if it's disabled
 *
 * If module is already enabled, it has no effect
 *
 * @param  module module name
 * @retval void
 */
void HAL_RCC_EnableModule(RCC_MODULE_TYPE module);

/**
 * @brief  Disable module
 *
 * If module is already disabled, it has no effect
 *
 * @param  module module name
 * @retval void
 */
void HAL_RCC_DisableModule(RCC_MODULE_TYPE module);

/**
 * @brief  Calibrate RC48
 *
 * Must be called after XTAL48 is ready
 *
 * @retval HAL_OK if successful, Otherwise HAL_ERROR
 */
HAL_StatusTypeDef HAL_RCC_CalibrateRC48(void);


HAL_StatusTypeDef HAL_RCC_SetModuleFreq(RCC_MODULE_TYPE module, uint32_t freq);

uint32_t HAL_RCC_GetModuleFreq(RCC_MODULE_TYPE module);

/**
 * @brief  RCC Init
 *
 * Disable some modules by default.
 * If the function is called by HCPU, HPSYS_RCC is configured and HPSYS modules are disabled.
 * If the function is called by LCPU, LPSYS_RCC is configured and LPSYS modules are disabled.
 *
 * @return void
 */
void HAL_RCC_Init(void);

/**
 * @brief  Chip specific RCC initialization function.
 *
 * It's called by HAL_RCC_Init. It's implemented as weak symbol internally, so can be re-implemented by user as required.
 *
 * @return void
 */
void HAL_RCC_MspInit(void);

#if defined(SF32LB58X) && defined(SOC_BF0_HCPU)
/**
 * @brief  Release ACPU
 * @retval void
 */
void HAL_RCC_ReleaseACPU(void);

/**
 * @brief  Reset ACPU, ACPU would be reset and halted
 * @retval void
 */
void HAL_RCC_ResetACPU(void);
#endif /* SF32LB58X && SOC_BF0_HCPU */

#ifndef SF32LB55X

/**
 * @brief  Set divider for clock of HCPU in deep WFI mode.
 * @param  div main divider HCLK=HPCLK/div
 * @param  pdiv1 divider for pclk1= HCLK/ (2^PDIV1) default 1
 * @param  pdiv2 divider for pclk2= HCLK/ (2^PDIV2) default 5
 * @retval None
 */
void HAL_RCC_HCPU_SetDeepWFIDiv(int8_t div, int8_t pdiv1, int8_t pdiv2);

/**
 * @brief  Get deepWFI divider for HCPU clock
 * @param  div pointer to main divider, HCLK=HPCLK/div
 * @param  pdiv1 pointer to divider for pclk1= HCLK/ (2^PDIV1) default 1
 * @param  pdiv2 pointer to divider for pclk2= HCLK/ (2^PDIV2) default 5
 * @retval None
 */
void HAL_RCC_HCPU_GetDeepWFIDiv(int *div, int *pdiv1, int *pdiv2);

/**
 * @brief  Select clock source for HCPU in deep WFI mode.
 * @param  sys_clk whether use sys clock, true: use sys clock, false: use LP clock
 * @param  sys_clk_src sys clock source, such as RCC_SYSCLK_HXT48
 * @retval None
 */
void HAL_RCC_HCPU_DeepWFIClockSelect(bool sys_clk, uint32_t sys_clk_src);

#endif /* SF32LB55X */

#if defined(SF32LB56X)
void HAL_RCC_Reset_DMAC2_and_MPI5();
#endif

#if defined(SF32LB58X)
void HAL_RCC_Reset_DMAC3_and_MPI5();
#endif

#if defined(SF32LB52X) && defined(SOC_BF0_HCPU)
/**
 * @brief  Config HPSYS HCLK
 *
 * If freq is greater than 48MHz, DLL1 would be used as SYSCLK.
 * If freq is less than or equal to 48MHz, all DLLs except DLL1 should be disabled before calling this function,
 * DLL1 would be disabled in this function automatically.
 *
 * @param  freq_in_mhz Frequency in MHz
 *
 * @retval HAL_OK if successful, Otherwise HAL_ERROR
 */
HAL_StatusTypeDef HAL_RCC_HCPU_ConfigHCLK(uint32_t freq_in_mhz);

/**
 * @brief  Config HPSYS HCLK by DVFS mode
 *
 * @param  freq_in_mhz Frequency in MHz
 * @param  mode        DVFS mode
 *
 * @retval HAL_OK if successful, Otherwise HAL_ERROR
 */
HAL_StatusTypeDef HAL_RCC_HCPU_ConfigHCLKByMode(uint32_t freq_in_mhz, HPSYS_DvfsModeTypeDef mode);

/**
 * @brief  Get HPSYS current DVFS mode
 *
 *
 * @return dvfs mode
 */
HPSYS_DvfsModeTypeDef HAL_RCC_HCPU_GetCurrentDvfsMode(void);

#endif /* SF32LB52X && SOC_BF0_HCPU*/


#if defined(SF32LB52X) && defined(SOC_BF0_LCPU)
/**
 * @brief  Config LPSYS HCLK
 *
 * @param  freq_in_mhz Frequency in MHz
 *
 * @retval HAL_OK if successful, Otherwise HAL_ERROR
 */
HAL_StatusTypeDef HAL_RCC_LCPU_ConfigHCLK(uint32_t freq_in_mhz);

/**
 * @brief  Config LPSYS HCLK by DVFS mode
 *
 * @param  freq_in_mhz Frequency in MHz
 * @param  mode        DVFS mode
 *
 * @retval HAL_OK if successful, Otherwise HAL_ERROR
 */
HAL_StatusTypeDef HAL_RCC_LCPU_ConfigHCLKByMode(uint32_t freq_in_mhz, LPSYS_DvfsModeTypeDef mode);

/**
 * @brief  Get LPSYS current DVFS mode
 *
 *
 * @return dvfs mode
 */
LPSYS_DvfsModeTypeDef HAL_RCC_LCPU_GetCurrentDvfsMode(void);

#endif /* SF32LB52X && SOC_BF0_LCPU */

#if defined(SF32LB56X) && defined(SOC_BF0_HCPU)
/**
 * @brief  Config HPSYS HCLK
 *
 * Only 240MHz, 288MHz and 312MHz are supported. DLL1 is used as clock source
 *
 * @param  freq_in_mhz Frequency in MHz
 *
 * @retval HAL_OK if successful, Otherwise HAL_ERROR
 */
HAL_StatusTypeDef HAL_RCC_HCPU_ConfigHCLK(uint32_t freq_in_mhz);

#endif /* SF32LB56X && SOC_BF0_HCPU */

///@} RCC

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_RCC_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

