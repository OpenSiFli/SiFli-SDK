/**
  ******************************************************************************
  * @file   bf0_hal_conf_hcpu.h
  * @author Sifli software development team
  * @brief   HAL configuration file.
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

#ifndef __BF0_HAL_CONF_HCPU_H
#define __BF0_HAL_CONF_HCPU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver
  */
#define HAL_ADC_MODULE_ENABLED
#define HAL_AES_MODULE_ENABLED
#define HAL_AON_MODULE_ENABLED
#define HAL_CACHE_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_CRC_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EFUSE_MODULE_ENABLED
#define HAL_EPIC_MODULE_ENABLED
#define HAL_EZIP_MODULE_ENABLED
#define HAL_EXTDMA_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_GPT_MODULE_ENABLED
#define HAL_HCD_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_I2S_MODULE_ENABLED
#define HAL_LCD_MODULE_ENABLED
#define HAL_LCPU_CONFIGURE_ENABLED
#define HAL_LCPU_PATCH_MODULE
#define HAL_LRC_CAL_ENABLED
#define HAL_LPTIM_MODULE_ENABLED
#define HAL_MAILBOX_MODULE_ENABLED
#define HAL_MMC_MODULE_ENABLED
#define HAL_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PDM_MODULE_ENABLED
#define HAL_PINMUX_MODULE_ENABLED
#define HAL_PMU_MODULE_ENABLED
#define HAL_PTC_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RNG_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SDHCI_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED

#define HAL_SYSTEM_CONFIG_ENABLED

#define HAL_TSEN_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_WDT_MODULE_ENABLED

#ifdef SF32LB52X
#define HAL_SECU_MODULE_ENABLED
#else
#define HAL_COMP_MODULE_ENABLED
#define HAL_BUSMON_MODULE_ENABLED
#define HAL_NNACC_MODULE_ENABLED
#endif /* SF32LB52X */

#ifdef SF32LB55X
#define HAL_QSPI_MODULE_ENABLED
#define HAL_PSRAM_MODULE_ENABLED
#else
#define HAL_ATIM_MODULE_ENABLED
#define HAL_MPI_MODULE_ENABLED
#define HAL_MATH_MODULE_ENABLED
#endif /* SF32LB55X */

#ifdef SF32LB58X
#define HAL_V2D_GPU_MODULE_ENABLED
#endif /* SF32LB58X */

#if defined(SF32LB56X) || defined(SF32LB58X)
#define HAL_AUDCODEC_MODULE_ENABLED
#define HAL_AUDPRC_MODULE_ENABLED
#define HAL_FACC_MODULE_ENABLED
#define HAL_FFT_MODULE_ENABLED
#define HAL_ATOMIC_FIX_ENABLED
#endif

#if defined(SF32LB52X)
#define HAL_AUDPRC_MODULE_ENABLED
#define HAL_AUDCODEC_MODULE_ENABLED
#endif

#if defined(SF32LB56X) || defined(SF32LB52X)
#define HAL_HASH_MODULE_ENABLED
#define HAL_SD_MODULE_ENABLED
#else
#define HAL_DSI_MODULE_ENABLED
#define HAL_SDADC_MODULE_ENABLED
#endif



/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSE_VALUE)
#define HSE_VALUE    ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

/**
  * @brief In the following line adjust the External High Speed oscillator (HSE) Startup
  *        Timeout value
  */
#if !defined  (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    ((uint32_t)100)   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSI_VALUE)
#define HSI_VALUE    ((uint32_t)8000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief In the following line adjust the Internal High Speed oscillator (HSI) Startup
  *        Timeout value
  */
#if !defined  (HSI_STARTUP_TIMEOUT)
#define HSI_STARTUP_TIMEOUT   ((uint32_t)5000) /*!< Time out for HSI start up */
#endif /* HSI_STARTUP_TIMEOUT */

/**
  * @brief Internal High Speed oscillator for ADC (HSI14) value.
  */
#if !defined  (HSI14_VALUE)
#define HSI14_VALUE ((uint32_t)14000000) /*!< Value of the Internal High Speed oscillator for ADC in Hz.
                                             The real value may vary depending on the variations
                                             in voltage and temperature.  */
#endif /* HSI14_VALUE */

/**
  * @brief Internal High Speed oscillator for USB (HSI48) value.
  */
#if !defined  (HSI48_VALUE)
#define HSI48_VALUE ((uint32_t)48000000) /*!< Value of the Internal High Speed oscillator for USB in Hz.
                                             The real value may vary depending on the variations
                                             in voltage and temperature.  */
#endif /* HSI48_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE)
#define LSI_VALUE  ((uint32_t)40000)
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
The real value may vary depending on the variations
in voltage and temperature.  */
/**
  * @brief External Low Speed oscillator (LSI) value.
  */
#if !defined  (LSE_VALUE)
#define LSE_VALUE  ((uint32_t)32768)    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT    ((uint32_t)5000)   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */
#define  VDD_VALUE                    ((uint32_t)3300) /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY            ((uint32_t)0)    /*!< tick interrupt priority (lowest by default)  */
/*  Warning: Must be set to higher priority for HAL_Delay()  */
/*  and HAL_GetTick() usage under interrupt context          */
#define  PREFETCH_ENABLE              0
#define  INSTRUCTION_CACHE_ENABLE     0
#define  DATA_CACHE_ENABLE            0
/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "HAL_ASSERT" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT   1U */

/* ################## SPI peripheral configuration ########################## */

/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
* Activated: CRC code is present inside driver
* Deactivated: CRC code cleaned from driver
*/

#define USE_SPI_CRC                     0U

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file
  */

#ifdef HAL_RCC_MODULE_ENABLED
#include "bf0_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
#include "bf0_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
#include "bf0_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
#include "bf0_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_EXTDMA_MODULE_ENABLED
#include "bf0_hal_ext_dma.h"
#endif /* HAL_EXTDMA_MODULE_ENABLED */


#ifdef HAL_CORTEX_MODULE_ENABLED
#include "bf0_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
#include "bf0_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_SDADC_MODULE_ENABLED
#include "bf0_hal_sdadc.h"
#endif /* HAL_SDADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
#include "bf0_hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_CEC_MODULE_ENABLED
#include "bf0_hal_cec.h"
#endif /* HAL_CEC_MODULE_ENABLED */

#ifdef HAL_COMP_MODULE_ENABLED
#include "bf0_hal_lpcomp.h"
#endif /* HAL_COMP_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
#include "bf0_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
#include "bf0_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_QSPI_MODULE_ENABLED
#include "bf0_hal_qspi_ex.h"
#endif /* HAL_QSPI_MODULE_ENABLED */

#ifdef HAL_MPI_MODULE_ENABLED
#include "bf0_hal_mpi_ex.h"
#endif  /* HAL_MPI_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
#include "bf0_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
#include "bf0_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
#include "bf0_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
#include "bf0_hal_pcd.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "bf0_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
#include "bf0_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
#include "bf0_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_SMBUS_MODULE_ENABLED
#include "bf0_hal_smbus.h"
#endif /* HAL_SMBUS_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
#include "bf0_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_GPT_MODULE_ENABLED
#include "bf0_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_TSC_MODULE_ENABLED
#include "bf0_hal_tsc.h"
#endif /* HAL_TSC_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
#include "bf0_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
//#include "bf0_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_WDT_MODULE_ENABLED
#include "bf0_hal_wdt.h"
#endif /* HAL_WDT_MODULE_ENABLED */

#ifdef HAL_EZIP_MODULE_ENABLED
#include "bf0_hal_ezip.h"
#endif

#ifdef HAL_EPIC_MODULE_ENABLED
#include "bf0_hal_epic.h"
#endif /* HAL_EPIC_MODULE_ENABLED */

#ifdef HAL_NNACC_MODULE_ENABLED
#include "bf0_hal_nn_acc.h"
#endif /* HAL_NNACC_MODULE_ENABLED */

#ifdef HAL_LCD_MODULE_ENABLED
#include "bf0_hal_lcdc.h"
#endif /* HAL_LCD_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
#include "bf0_hal_sdmmc.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_SDHCI_MODULE_ENABLED
#include "bf0_hal_sdhci.h"
#endif /* HAL_SDHCI_MODULE_ENABLED */

#ifdef HAL_AES_MODULE_ENABLED
#include "bf0_hal_aes.h"
#endif /* HAL_AES_MODULE_ENABLED */

#ifdef HAL_EFUSE_MODULE_ENABLED
#include "bf0_hal_efuse.h"
#endif /* HAL_EFUSE_MODULE_ENABLED */

#ifdef HAL_MAILBOX_MODULE_ENABLED
#include "bf0_hal_mailbox.h"
#endif /* HAL_MAILBOX_MODULE_ENABLED */

#ifdef HAL_PINMUX_MODULE_ENABLED
#include "bf0_hal_pinmux.h"
#endif /* HAL_PINMUX_MODULE_ENABLED */

#ifdef HAL_SYSTEM_CONFIG_ENABLED
#include "bf0_sys_cfg.h"
#endif

#ifdef HAL_PMU_MODULE_ENABLED
#include "bf0_hal_pmu.h"
#endif /* HAL_PMU_MODULE_ENABLED */

#ifdef HAL_AON_MODULE_ENABLED
#include "bf0_hal_aon.h"
#endif

#ifdef HAL_DSI_MODULE_ENABLED
#include "bf0_hal_dsi.h"
#endif /* HAL_DSI_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
#include "bf0_hal_rng.h"
#endif

#ifdef HAL_PSRAM_MODULE_ENABLED
#include "bf0_hal_psram.h"
#endif

#ifdef HAL_CACHE_MODULE_ENABLED
#include "bf0_hal_cache.h"
#endif

#ifdef HAL_BUSMON_MODULE_ENABLED
#include "bf0_hal_busmon.h"
#endif

#ifdef HAL_PTC_ENABLED
#include "bf0_hal_ptc.h"
#endif

#ifdef HAL_PDM_MODULE_ENABLED
#include "bf0_hal_pdm.h"
#endif

#ifdef HAL_LCPU_PATCH_MODULE
#include "bf0_hal_patch.h"
#endif

#ifdef HAL_TSEN_MODULE_ENABLED
#include "bf0_hal_tsen.h"
#endif

#ifdef HAL_LCPU_CONFIGURE_ENABLED
#include "bf0_hal_lcpu_config.h"
#endif

#ifdef HAL_LRC_CAL_ENABLED
#include "bf0_hal_lrc_cal.h"
#endif

#ifdef HAL_FFT_MODULE_ENABLED
#include "bf0_hal_fft.h"
#endif


#ifdef HAL_MATH_MODULE_ENABLED
#include "bf0_hal_math.h"
#endif

#ifdef HAL_FACC_MODULE_ENABLED
#include "bf0_hal_facc.h"
#endif

#ifdef HAL_AUDPRC_MODULE_ENABLED
#include "bf0_hal_audprc.h"
#endif

#ifdef HAL_HCD_MODULE_ENABLED
#include "bf0_hal_hcd.h"
#endif

#ifdef HAL_AUDCODEC_MODULE_ENABLED
#include "bf0_hal_audcodec.h"
#endif

#ifdef HAL_SECU_MODULE_ENABLED
#include "bf0_hal_secu.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_CONF_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
