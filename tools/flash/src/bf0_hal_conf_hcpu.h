/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Minimal HAL configuration for tools/flash loaders.
 * Keep this deliberately narrow so unrelated subsystems do not leak in.
 */

#ifndef __BF0_HAL_CONF_HCPU_H
#define __BF0_HAL_CONF_HCPU_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED

#define HAL_AON_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EFUSE_MODULE_ENABLED
#define HAL_EXTDMA_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PINMUX_MODULE_ENABLED
#define HAL_PMU_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SDHCI_MODULE_ENABLED
#define HAL_SYSTEM_CONFIG_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_WDT_MODULE_ENABLED

#if !defined(HSE_VALUE)
#define HSE_VALUE ((uint32_t)8000000)
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT ((uint32_t)100)
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE ((uint32_t)8000000)
#endif

#if !defined(HSI_STARTUP_TIMEOUT)
#define HSI_STARTUP_TIMEOUT ((uint32_t)5000)
#endif

#if !defined(HSI14_VALUE)
#define HSI14_VALUE ((uint32_t)14000000)
#endif

#if !defined(HSI48_VALUE)
#define HSI48_VALUE ((uint32_t)48000000)
#endif

#if !defined(LSI_VALUE)
#define LSI_VALUE ((uint32_t)40000)
#endif

#if !defined(LSE_VALUE)
#define LSE_VALUE ((uint32_t)32768)
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT ((uint32_t)5000)
#endif

#define VDD_VALUE ((uint32_t)3300)
#define TICK_INT_PRIORITY ((uint32_t)0)
#define PREFETCH_ENABLE 0
#define INSTRUCTION_CACHE_ENABLE 0
#define DATA_CACHE_ENABLE 0
#define USE_SPI_CRC 0U

#ifdef SF32LB55X
#define HAL_QSPI_MODULE_ENABLED
#else
#define HAL_MPI_MODULE_ENABLED
#endif

#include "bf0_hal_rcc.h"
#include "bf0_hal_gpio.h"
#include "bf0_hal_dma.h"
#include "bf0_hal_ext_dma.h"
#include "bf0_hal_cortex.h"
#include "bf0_hal_rtc.h"
#include "bf0_hal_uart.h"
#include "bf0_hal_efuse.h"
#include "bf0_hal_sdhci.h"
#include "bf0_hal_pinmux.h"
#include "bf0_sys_cfg.h"
#include "bf0_hal_pmu.h"
#include "bf0_hal_aon.h"
#include "bf0_hal_wdt.h"

#ifdef SF32LB55X
#include "bf0_hal_qspi_ex.h"
#else
#include "bf0_hal_mpi_ex.h"
#endif

#ifdef __cplusplus
}
#endif

#endif
