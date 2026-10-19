/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_CONF_HCPU_H
#define __BF0_HAL_CONF_HCPU_H

#ifdef __cplusplus
extern "C" {
#endif


#ifndef _REGISTER_H_
#error "Don't include bf0_hal.h without including register.h first. \
You can just include register.h which includes bf0_hal.h also"
#endif /* _REGISTER_H_ */


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
#ifdef EXTDMA_BASE
#define HAL_EXTDMA_MODULE_ENABLED
#endif  /* EXTDMA_BASE */
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
#ifdef PTM1_BASE
#define HAL_PTM_MODULE_ENABLED
#endif /* PTM1_BASE */
#define HAL_RCC_MODULE_ENABLED
#define HAL_RNG_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SDHCI_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED

#define HAL_SYSTEM_CONFIG_ENABLED

#define HAL_TSEN_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_WDT_MODULE_ENABLED
#ifdef PWM1_BASE
#define HAL_PWM_MODULE_ENABLED
#endif /* PWM1_BASE */

//TODO:
#ifdef SECU1_BASE
#define HAL_SECU_MODULE_ENABLED
#endif /* SECU1_BASE */

#ifdef LPCOMP_BASE
#define HAL_COMP_MODULE_ENABLED
#endif /* LPCOMP_BASE */

#ifdef BUSMON1_BASE
#define HAL_BUSMON_MODULE_ENABLED
#endif /* BUSMON1_BASE */

#if defined(NNACC_BASE) || defined(NNACC1_BASE)
#define HAL_NNACC_MODULE_ENABLED
#endif /* NNACC_BASE || NNACC1_BASE */

#ifdef QSPI1_BASE
#define HAL_QSPI_MODULE_ENABLED
#endif /* QSPI1_BASE */

#ifdef PSRAMC_BASE
#define HAL_PSRAM_MODULE_ENABLED
#endif /* PSRAMC_BASE */

#ifdef ATIM1_BASE
#define HAL_ATIM_MODULE_ENABLED
#endif /* ATIM1_BASE */

#ifdef MPI1_BASE
#define HAL_MPI_MODULE_ENABLED
#endif /* MPI1_BASE */

#ifndef SF32LB55X
#if defined(__CC_ARM) || defined(__CLANG_ARM) || (defined(__GNUC__) && (__GNUC__ > 9))
#define HAL_MATH_MODULE_ENABLED
#endif /* defined(__CC_ARM) || defined(__CLANG_ARM) || (defined(__GNUC__) && (__GNUC__ > 9)) */
#endif /* SF32LB55X */

#ifdef V2D_GPU_BASE
#define HAL_V2D_GPU_MODULE_ENABLED
#endif /* V2D_GPU_BASE */

#if defined(FACC1_BASE)
#define HAL_FACC_MODULE_ENABLED
#endif /* FACC1_BASE */

#if defined(FFT1_BASE) || defined(FFT_BASE)
#define HAL_FFT_MODULE_ENABLED
#endif /* FFT_BASE */

#ifndef SF32LB55X
#define HAL_ATOMIC_FIX_ENABLED
#endif /* !SF32LB55X */

#ifdef AUDPRC_BASE
#define HAL_AUDPRC_MODULE_ENABLED
#endif /* AUDPRC_BASE */

#if defined(AUDCODEC_HP_BASE) || defined(AUDCODEC_BASE)
#define HAL_AUDCODEC_MODULE_ENABLED
#endif /* AUDCODEC_HP_BASE */

#ifdef CAN1_BASE
#define HAL_CAN_MODULE_ENABLED
#endif /* CAN1_BASE */

#ifdef HASH_ACC_BASE
#define HAL_HASH_MODULE_ENABLED
#endif /* HASH_ACC_BASE */

#if !defined(SF32LB55X) && !defined(SF32LB58X)
#define HAL_SD_MODULE_ENABLED
#endif /* !SF32LB55X && !SF32LB58X */

#ifdef DSI_HOST_BASE
#define HAL_DSI_MODULE_ENABLED
#endif /* DSI_HOST_BASE */

#ifdef SDADC_BASE
#define HAL_SDADC_MODULE_ENABLED
#endif /* SDADC_BASE */

#ifdef JPEGD_BASE
#define HAL_JPEGD_MODULE_ENABLED
#endif

#ifdef DCMI_BASE
#define HAL_DCMI_MODULE_ENABLED
#endif /* SDADC_BASE */



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
#endif /* HAL_EZIP_MODULE_ENABLED */

#ifdef HAL_JPEGD_MODULE_ENABLED
#include "bf0_hal_jpegd.h"
#endif /* HAL_JPEGD_MODULE_ENABLED */

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

#ifdef HAL_PTM_MODULE_ENABLED
#include "bf0_hal_ptm.h"
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

#ifdef HAL_DCMI_MODULE_ENABLED
#include "bf0_hal_dcmi.h"
#endif

#ifdef HAL_LPTIM_MODULE_ENABLED
#include "bf0_hal_lptim.h"
#endif

#ifdef HAL_PWM_MODULE_ENABLED
#include "bf0_hal_pwm.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_CONF_H */