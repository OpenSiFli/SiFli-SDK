/**
  ******************************************************************************
  * @file   register.h
  * @author Sifli software development team
  * @brief  SiFli chipset register definition
  *          This file provides register definition for SiFli chipset
  * @{
  ******************************************************************************
*/
/**
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

#ifndef _REGISTER_H_
#define _REGISTER_H_

/** @addtogroup CMSIS_Device SiFli CMSIS device interface
  * @{
  */


#ifdef __cplusplus
extern "C" {
#endif


#ifdef SOC_BF0_HCPU

/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum IRQn
{
    /* -------------------  Processor Exceptions Numbers  ----------------------------- */
    NonMaskableInt_IRQn           = -14,     /*  2 Non Maskable Interrupt */
    HardFault_IRQn                = -13,     /*  3 HardFault Interrupt */
    MemoryManagement_IRQn         = -12,     /*  4 Memory Management Interrupt */
    BusFault_IRQn                 = -11,     /*  5 Bus Fault Interrupt */
    UsageFault_IRQn               = -10,     /*  6 Usage Fault Interrupt */
    SecureFault_IRQn              =  -9,     /*  7 Secure Fault Interrupt */
    SVCall_IRQn                   =  -5,     /* 11 SV Call Interrupt */
    DebugMonitor_IRQn             =  -4,     /* 12 Debug Monitor Interrupt */
    PendSV_IRQn                   =  -2,     /* 14 Pend SV Interrupt */
    SysTick_IRQn                  =  -1,     /* 15 System Tick Interrupt */

    /* -------------------  Processor Interrupt Numbers  ------------------------------ */
    AON_IRQn                      =   0,
    BLE_MAC_IRQn                  =   1,
    DMAC2_CH1_IRQn                =   2,
    DMAC2_CH2_IRQn                =   3,
    DMAC2_CH3_IRQn                =   4,
    DMAC2_CH4_IRQn                =   5,
    DMAC2_CH5_IRQn                =   6,
    DMAC2_CH6_IRQn                =   7,
    DMAC2_CH7_IRQn                =   8,
    DMAC2_CH8_IRQn                =   9,
    PATCH_IRQn                    =  10,
    Interrupt11_IRQn              =  11,
    USART3_IRQn                   =  12,
    USART4_IRQn                   =  13,
    USART5_IRQn                   =  14,
    Interrupt15_IRQn              =  15,
    SPI3_IRQn                     =  16,
    SPI4_IRQn                     =  17,
    Interrupt18_IRQn              =  18,
    I2C4_IRQn                     =  19,
    I2C5_IRQn                     =  20,
    I2C6_IRQn                     =  21,
    GPTIM3_IRQn                   =  22,
    GPTIM4_IRQn                   =  23,
    GPTIM5_IRQn                   =  24,
    BTIM3_IRQn                    =  25,
    BTIM4_IRQn                    =  26,
    Interrupt27_IRQn              =  27,
    GPADC_IRQn                    =  28,
    SDADC_IRQn                    =  29,
    Interrupt30_IRQn              =  30,
    Interrupt31_IRQn              =  31,
    TSEN_IRQn                     =  32,
    PTC2_IRQn                     =  33,
    LCDC2_IRQn                    =  34,
    GPIO2_IRQn                    =  35,
    QSPI4_IRQn                    =  36,
    Interrupt37_IRQn              =  37,
    Interrupt38_IRQn              =  38,
    Interrupt39_IRQn              =  39,
    Interrupt40_IRQn              =  40,
    LPCOMP_IRQn                   =  41,
    LPTIM2_IRQn                   =  42,
    LPTIM3_IRQn                   =  43,
    Interrupt44_IRQn              =  44,
    Interrupt45_IRQn              =  45,
    LPTIM1_IRQn                   =  46,
    Interrupt47_IRQn              =  47,
    IWDT_IRQn                     =  48,
    RTC_IRQn                      =  49,
    DMAC1_CH1_IRQn                =  50,
    DMAC1_CH2_IRQn                =  51,
    DMAC1_CH3_IRQn                =  52,
    DMAC1_CH4_IRQn                =  53,
    DMAC1_CH5_IRQn                =  54,
    DMAC1_CH6_IRQn                =  55,
    DMAC1_CH7_IRQn                =  56,
    DMAC1_CH8_IRQn                =  57,
    LCPU2HCPU_IRQn                =  58,
    USART1_IRQn                   =  59,
    SPI1_IRQn                     =  60,
    I2C1_IRQn                     =  61,
    EPIC_IRQn                     =  62,
    LCDC1_IRQn                    =  63,
    I2S1_IRQn                     =  64,
    I2S2_IRQn                     =  65,
    EFUSEC_IRQn                   =  66,
    AES_IRQn                      =  67,
    PTC1_IRQn                     =  68,
    TRNG_IRQn                     =  69,
    GPTIM1_IRQn                   =  70,
    GPTIM2_IRQn                   =  71,
    BTIM1_IRQn                    =  72,
    BTIM2_IRQn                    =  73,
    USART2_IRQn                   =  74,
    SPI2_IRQn                     =  75,
    I2C2_IRQn                     =  76,
    EXTDMA_IRQn                   =  77,
    PSRAMC_IRQn                   =  78,
    SDMMC1_IRQn                   =  79,
    SDMMC2_IRQn                   =  80,
    NNACC_IRQn                    =  81,
    PDM1_IRQn                     =  82,
    DSIHOST_IRQn                  =  83,
    GPIO1_IRQn                    =  84,
    QSPI1_IRQn                    =  85,
    QSPI2_IRQn                    =  86,
    QSPI3_IRQn                    =  87,
    EZIP_IRQn                     =  88,
    PDM2_IRQn                     =  89,
    USBC_IRQn                     =  90,
    I2C3_IRQn                     =  91,
    Interrupt92_IRQn              =  92,
    Interrupt93_IRQn              =  93,
    Interrupt94_IRQn              =  94,
    Interrupt95_IRQn              =  95,
    HCPU2LCPU_IRQn                =  -1,
    /* Interrupts 96 .. 479 are left out */
} IRQn_Type;

#else       /*LCPU*/
typedef enum IRQn
{
    /* -------------------  Processor Exceptions Numbers  ----------------------------- */
    NonMaskableInt_IRQn           = -14,     /*  2 Non Maskable Interrupt */
    HardFault_IRQn                = -13,     /*  3 HardFault Interrupt */
    MemoryManagement_IRQn         = -12,     /*  4 Memory Management Interrupt */
    BusFault_IRQn                 = -11,     /*  5 Bus Fault Interrupt */
    UsageFault_IRQn               = -10,     /*  6 Usage Fault Interrupt */
    SecureFault_IRQn              =  -9,     /*  7 Secure Fault Interrupt */
    SVCall_IRQn                   =  -5,     /* 11 SV Call Interrupt */
    DebugMonitor_IRQn             =  -4,     /* 12 Debug Monitor Interrupt */
    PendSV_IRQn                   =  -2,     /* 14 Pend SV Interrupt */
    SysTick_IRQn                  =  -1,     /* 15 System Tick Interrupt */

    /* -------------------  Processor Interrupt Numbers  ------------------------------ */
    AON_IRQn                      =   0,
    BLE_MAC_IRQn                  =   1,
    DMAC2_CH1_IRQn                =   2,
    DMAC2_CH2_IRQn                =   3,
    DMAC2_CH3_IRQn                =   4,
    DMAC2_CH4_IRQn                =   5,
    DMAC2_CH5_IRQn                =   6,
    DMAC2_CH6_IRQn                =   7,
    DMAC2_CH7_IRQn                =   8,
    DMAC2_CH8_IRQn                =   9,
    PATCH_IRQn                    =  10,
    Interrupt11_IRQn              =  11,
    USART3_IRQn                   =  12,
    USART4_IRQn                   =  13,
    USART5_IRQn                   =  14,
    Interrupt15_IRQn              =  15,
    SPI3_IRQn                     =  16,
    SPI4_IRQn                     =  17,
    Interrupt18_IRQn              =  18,
    I2C4_IRQn                     =  19,
    I2C5_IRQn                     =  20,
    I2C6_IRQn                     =  21,
    GPTIM3_IRQn                   =  22,
    GPTIM4_IRQn                   =  23,
    GPTIM5_IRQn                   =  24,
    BTIM3_IRQn                    =  25,
    BTIM4_IRQn                    =  26,
    Interrupt27_IRQn              =  27,
    GPADC_IRQn                    =  28,
    SDADC_IRQn                    =  29,
    Interrupt30_IRQn              =  30,
    Interrupt31_IRQn              =  31,
    TSEN_IRQn                     =  32,
    PTC2_IRQn                     =  33,
    LCDC2_IRQn                    =  34,
    GPIO2_IRQn                    =  35,
    QSPI4_IRQn                    =  36,
    Interrupt37_IRQn              =  37,
    Interrupt38_IRQn              =  38,
    Interrupt39_IRQn              =  39,
    Interrupt40_IRQn              =  40,
    LPCOMP_IRQn                   =  41,
    LPTIM2_IRQn                   =  42,
    LPTIM3_IRQn                   =  43,
    Interrupt44_IRQn              =  44,
    Interrupt45_IRQn              =  45,
    HCPU2LCPU_IRQn                =  46,
    RTC_IRQn                      =  47,

// Warning: Those IRQ will no work in LCPU, put here for compile purpose
// TODO: Add more if needed.
    USART1_IRQn                   =  -1,
    LCPU2HCPU_IRQn                =  -1,
    /* Interrupts 48 .. 479 are left out */
} IRQn_Type;


#endif /* SOC_BF0_HCPU */

/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* -------  Start of section using anonymous unions and disabling warnings  ------- */
#if   defined (__CC_ARM)
#pragma push
#pragma anon_unions
#elif defined (__ICCARM__)
#pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc11-extensions"
#pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning 586
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif


/* --------  Configuration of Core Peripherals  ----------------------------------- */
#define __CM33_REV                0x0000U   /* Core revision r0p1 */
#define __SAUREGION_PRESENT       0U        /* SAU regions present */
#define __MPU_PRESENT             1U        /* MPU present */
#define __VTOR_PRESENT            1U        /* VTOR present */
#define __NVIC_PRIO_BITS          3U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */
#ifndef __FPU_PRESENT
#define __FPU_PRESENT             1U        /* no FPU present */
#endif /* __FPU_PRESENT */
#ifndef __DSP_PRESENT
#define __DSP_PRESENT             1U        /* no DSP extension present */
#endif /* __DSP_PRESENT */

#include "core_cm33.h"                      /* Processor and core peripherals */
#include "system_bf0_ap.h"                 /* System Header */


#ifdef SOC_BF0_HCPU
#ifndef __ICACHE_PRESENT
#define __ICACHE_PRESENT          1U
#endif
#ifndef __DCACHE_PRESENT
#define __DCACHE_PRESENT          1U
#endif
#else
#ifndef __ICACHE_PRESENT
#define __ICACHE_PRESENT          1U
#endif
#ifndef __DCACHE_PRESENT
#define __DCACHE_PRESENT          1U
#endif

#endif /* SOC_BF0_HCPU */

#ifdef SOC_BF0_HCPU
#define MPU_REGION_NUM  12
#else
#define MPU_REGION_NUM   8
#endif /* SOC_BF0_HCPU */


#include "core_mstar.h"                     /* cache related functions */

/* --------  End of section using anonymous unions and disabling warnings  -------- */
#if   defined (__CC_ARM)
#pragma pop
#elif defined (__ICCARM__)
/* leave anonymous unions enabled */
#elif (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
#pragma clang diagnostic pop
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning restore
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif

#include <stdint.h>
#include "mem_map.h"
#include "ble_mac.h"
#include "ble_phy.h"
#include "ble_rf_dig.h"
#include "hpsys_rcc.h"
#include "lpsys_rcc.h"
#include "dmac.h"
#include "extdma.h"
#include "usart.h"
#include "epic.h"
#include "spi.h"
#include "gpt.h"
#include "btim.h"
#include "mailbox.h"
#include "rtc.h"
#include "psramc.h"
#include "qspi.h"
#include "nn_acc.h"
#include "dsi_host.h"
#include "dsi_phy.h"
#include "lptim.h"
#include "i2c.h"
#include "hpsys_cfg.h"
#include "lpsys_cfg.h"
#include "efusec.h"
#include "i2s.h"
#include "crc.h"
#include "lcd_if.h"
#include "sdmmc.h"
#include "aes_acc.h"
#include "gpio1.h"
#include "gpio2.h"
#include "hpsys_pinmux.h"
#include "lpsys_pinmux.h"
#include "hpsys_aon.h"
#include "lpsys_aon.h"
#include "pmuc.h"
#include "gpadc.h"
#include "sdadc.h"
#include "tsen.h"
#include "trng.h"
#include "ptc.h"
#include "ezip.h"
#include "patch.h"
#include "wdt.h"
#include "pdm.h"
#include "busmon.h"
#include "lpcomp.h"
#include "usbc_x.h"
#include "cache.h"

/** @addtogroup Peripheral_memory_map
  * @{
  */

/******************* Base Addresss Definition ******************/
//================== CORE ===================
#define CACHE_BASE          0xE0080000

//================== MCU_HPSYS ===================
#define HPSYS_RCC_BASE      0x40000000
#define DMAC1_BASE          0x40001000
#define MAILBOX1_BASE       0x40002000
#define PINMUX1_BASE        0x40003000
#define USART1_BASE         0x40004000
#define USART2_BASE         0x40005000
#define EZIP_BASE           0x40006000
#define EPIC_BASE           0x40007000
#define LCDC1_BASE          0x40008000
#define I2S1_BASE           0x40009000
#define I2S2_BASE           0x4000a000
#define HPSYS_CFG_BASE      0x4000b000
#define EFUSEC_BASE         0x4000c000
#define AES_BASE            0x4000d000
#define CRC_BASE            0x4000e000
#define TRNG_BASE           0x4000f000
//------------------------------------
#define GPTIM1_BASE         0x40010000
#define GPTIM2_BASE         0x40011000
#define BTIM1_BASE          0x40012000
#define BTIM2_BASE          0x40013000
#define WDT1_BASE           0x40014000
#define SPI1_BASE           0x40015000
#define SPI2_BASE           0x40016000
#define EXTDMA_BASE         0x40017000
#define PSRAMC_BASE         0x40018000
#define NNACC_BASE          0x40019000
#define PDM1_BASE           0x4001a000
#define PDM2_BASE           0x4001b000
#define I2C1_BASE           0x4001c000
#define I2C2_BASE           0x4001d000
#define DSI_HOST_BASE       0x4001e000
#define DSI_PHY_BASE        0x4001f000
#define PTC1_BASE           0x40020000
#define BUSMON1_BASE        0x40021000
#define I2C3_BASE           0x40022000
//------------------------------------
#define HPSYS_AON_BASE      0x40030000
#define LPTIM1_BASE         0x40031000
//------------------------------------
#define GPIO1_BASE          0x50000000
#define QSPI1_BASE          0x50001000
#define QSPI2_BASE          0x50002000
#define QSPI3_BASE          0x50003000
#define SDMMC1_BASE         0x50004000
#define SDMMC2_BASE         0x50005000
#define USBC_BASE           0x50006000
#define EPIC_RAM_BASE       0x50010000
//------------------------------------

//================== MCU_LPSYS ===================
#define LPSYS_RCC_BASE      0x40040000
#define DMAC2_BASE          0x40041000
#define MAILBOX2_BASE       0x40042000
#define PINMUX2_BASE        0x40043000
#define PATCH_BASE          0x40044000
#define USART3_BASE         0x40045000
#define USART4_BASE         0x40046000
#define USART5_BASE         0x40047000
// RSVD                     0x40048000
#define SPI3_BASE           0x40049000
#define SPI4_BASE           0x4004a000
// RSVD                     0x4004b000
#define I2C4_BASE           0x4004c000
#define I2C5_BASE           0x4004d000
#define I2C6_BASE           0x4004e000
#define LPSYS_CFG_BASE      0x4004f000
//------------------------------------
#define GPTIM3_BASE         0x40050000
#define GPTIM4_BASE         0x40051000
#define GPTIM5_BASE         0x40052000
#define BTIM3_BASE          0x40053000
#define BTIM4_BASE          0x40054000
#define WDT2_BASE           0x40055000
#define GPADC_BASE          0x40056000
#define SDADC_BASE          0x40057000
//reserved                  0x40058000
#define LPCOMP_BASE         0x40059000
#define TSEN_BASE           0x4005a000
#define PTC2_BASE           0x4005b000
#define LCDC2_BASE          0x4005c000
#define BUSMON2_BASE        0x4005d000
//reserved                  0x4005e000
//reserved                  0x4005f000
//------------------------------------
#define LPSYS_AON_BASE      0x40070000
#define LPTIM2_BASE         0x40071000
#define LPTIM3_BASE         0x40072000
//reserved
#define PMUC_BASE           0x4007a000
#define RTC_BASE            0x4007b000
#define IWDT_BASE           0x4007c000
//reserved
//------------------------------------
#define GPIO2_BASE          0x50040000
#define QSPI4_BASE          0x50041000
#define BLE_RFC_BASE        0x50042000
#define BLE_PHY_BASE        0x50044000
#define BLE_MAC_BASE        0x50050000
#define HMAILBOX_BASE       MAILBOX1_BASE
#define LMAILBOX_BASE       MAILBOX2_BASE

#define USBC_X_BASE         (USBC_BASE)          // TODO: Change address to match chipset

/****************** Header Pointers Definition *****************/
#define hwp_cache       ((CACHE_TypeDef         *)    CACHE_BASE)
#define hwp_hpsys_rcc   ((HPSYS_RCC_TypeDef     *)    HPSYS_RCC_BASE)
#define hwp_lpsys_rcc   ((LPSYS_RCC_TypeDef     *)    LPSYS_RCC_BASE)
#define hwp_dmac1       ((DMAC_TypeDef          *)    DMAC1_BASE)
#define hwp_dmac2       ((DMAC_TypeDef          *)    DMAC2_BASE)
#define hwp_gptim1      ((GPT_TypeDef           *)    GPTIM1_BASE)
#define hwp_gptim2      ((GPT_TypeDef           *)    GPTIM2_BASE)
#define hwp_gptim3      ((GPT_TypeDef           *)    GPTIM3_BASE)
#define hwp_gptim4      ((GPT_TypeDef           *)    GPTIM4_BASE)
#define hwp_gptim5      ((GPT_TypeDef           *)    GPTIM5_BASE)
#define hwp_btim1       ((BTIM_TypeDef          *)    BTIM1_BASE)
#define hwp_btim2       ((BTIM_TypeDef          *)    BTIM2_BASE)
#define hwp_btim3       ((BTIM_TypeDef          *)    BTIM3_BASE)
#define hwp_btim4       ((BTIM_TypeDef          *)    BTIM4_BASE)
/** EPIC instance */
#define hwp_epic        ((EPIC_TypeDef          *)    EPIC_BASE)
#define hwp_spi1        ((SPI_TypeDef           *)    SPI1_BASE)
#define hwp_spi2        ((SPI_TypeDef           *)    SPI2_BASE)
#define hwp_spi3        ((SPI_TypeDef           *)    SPI3_BASE)
#define hwp_spi4        ((SPI_TypeDef           *)    SPI4_BASE)
#define hwp_usart1      ((USART_TypeDef         *)    USART1_BASE)
#define hwp_usart2      ((USART_TypeDef         *)    USART2_BASE)
#define hwp_usart3      ((USART_TypeDef         *)    USART3_BASE)
#define hwp_usart4      ((USART_TypeDef         *)    USART4_BASE)
#define hwp_usart5      ((USART_TypeDef         *)    USART5_BASE)
#define hwp_i2c1        ((I2C_TypeDef           *)    I2C1_BASE)
#define hwp_i2c2        ((I2C_TypeDef           *)    I2C2_BASE)
#define hwp_i2c3        ((I2C_TypeDef           *)    I2C3_BASE)
#define hwp_i2c4        ((I2C_TypeDef           *)    I2C4_BASE)
#define hwp_i2c5        ((I2C_TypeDef           *)    I2C5_BASE)
#define hwp_i2c6        ((I2C_TypeDef           *)    I2C6_BASE)
#define hwp_mailbox1    ((MAILBOX_TypeDef       *)    MAILBOX1_BASE)
#define hwp_mailbox2    ((MAILBOX_TypeDef       *)    MAILBOX2_BASE)

#define hwp_hmailbox    ((MAILBOX_TypeDef       *)    MAILBOX1_BASE)
#define hwp_lmailbox    ((MAILBOX_TypeDef       *)    MAILBOX2_BASE)

#define hwp_nnacc       ((NN_ACC_TypeDef        *)    NNACC_BASE)
#define hwp_dsi_host    ((DSI_HOST_TypeDef      *)    DSI_HOST_BASE)
#define hwp_dsi_phy     ((DSI_PHY_TypeDef       *)    DSI_PHY_BASE)
#define hwp_ptc1        ((PTC_TypeDef           *)    PTC1_BASE)
#define hwp_ptc2        ((PTC_TypeDef           *)    PTC2_BASE)
#define hwp_busmon1     ((BUSMON_TypeDef        *)    BUSMON1_BASE)
#define hwp_busmon2     ((BUSMON_TypeDef        *)    BUSMON2_BASE)
/** EZIP instance */
#define hwp_ezip        ((EZIP_TypeDef          *)    EZIP_BASE)
#define hwp_efusec      ((EFUSEC_TypeDef        *)    EFUSEC_BASE)
#define hwp_rtc         ((RTC_TypeDef           *)    RTC_BASE)
#define hwp_pmuc        ((PMUC_TypeDef          *)    PMUC_BASE)
#define hwp_qspi1       ((QSPI_TypeDef          *)    QSPI1_BASE)
#define hwp_qspi2       ((QSPI_TypeDef          *)    QSPI2_BASE)
#define hwp_qspi3       ((QSPI_TypeDef          *)    QSPI3_BASE)
#define hwp_qspi4       ((QSPI_TypeDef          *)    QSPI4_BASE)
#define hwp_psramc      ((PSRAMC_TypeDef        *)    PSRAMC_BASE)
#define hwp_lptim1      ((LPTIM_TypeDef         *)    LPTIM1_BASE)
#define hwp_lptim2      ((LPTIM_TypeDef         *)    LPTIM2_BASE)
#define hwp_lptim3      ((LPTIM_TypeDef         *)    LPTIM3_BASE)
#define hwp_hpsys_cfg   ((HPSYS_CFG_TypeDef     *)    HPSYS_CFG_BASE)
#define hwp_lpsys_cfg   ((LPSYS_CFG_TypeDef     *)    LPSYS_CFG_BASE)
#define hwp_i2s1        ((I2S_TypeDef           *)    I2S1_BASE)
#define hwp_i2s2        ((I2S_TypeDef           *)    I2S2_BASE)
#define hwp_pdm1        ((PDM_TypeDef           *)    PDM1_BASE)
#define hwp_pdm2        ((PDM_TypeDef           *)    PDM2_BASE)
#define hwp_crc         ((CRC_TypeDef           *)    CRC_BASE)
#define hwp_trng        ((TRNG_TypeDef          *)    TRNG_BASE)
#define hwp_lcdc1       ((LCD_IF_TypeDef        *)    LCDC1_BASE)
#define hwp_lcdc2       ((LCD_IF_TypeDef        *)    LCDC2_BASE)
#define hwp_extdma      ((EXTDMA_TypeDef        *)    EXTDMA_BASE)
#define hwp_sdmmc1      ((SDMMC_TypeDef         *)    SDMMC1_BASE)
#define hwp_sdmmc2      ((SDMMC_TypeDef         *)    SDMMC2_BASE)
#define hwp_aes_acc     ((AES_ACC_TypeDef       *)    AES_BASE)
/** GPIO1 */
#define hwp_gpio1       ((GPIO_TypeDef          *)    GPIO1_BASE)
/** GPIO2 */
#define hwp_gpio2       ((GPIO_TypeDef          *)    GPIO2_BASE)
#define hwp_usbc        ((USBC_X_Typedef        *)    USBC_BASE)
/** PINMUX1 */
#define hwp_pinmux1     ((HPSYS_PINMUX_TypeDef  *)    PINMUX1_BASE)
/** PINMUX2 */
#define hwp_pinmux2     ((LPSYS_PINMUX_TypeDef  *)    PINMUX2_BASE)
/** HPSYS AON */
#define hwp_hpsys_aon   ((HPSYS_AON_TypeDef     *)    HPSYS_AON_BASE)
/** LPSYS AON */
#define hwp_lpsys_aon   ((LPSYS_AON_TypeDef     *)    LPSYS_AON_BASE)
#define hwp_gpadc1       ((GPADC_TypeDef         *)    GPADC_BASE)
#define hwp_sdadc       ((SDADC_TypeDef         *)    SDADC_BASE)
#define hwp_lpcomp      ((LPCOMP_TypeDef        *)    LPCOMP_BASE)
#define hwp_tsen        ((TSEN_TypeDef          *)    TSEN_BASE)
#define hwp_patch       ((PATCH_TypeDef         *)    PATCH_BASE)
#define hwp_ble_rfc     ((BLE_RF_DIG_TypeDef    *)    BLE_RFC_BASE)
#define hwp_ble_phy     ((BLE_PHY_TypeDef       *)    BLE_PHY_BASE)
#define hwp_ble_mac     ((BLE_MAC_TypeDef       *)    BLE_MAC_BASE)
#define hwp_wdt1        ((WDT_TypeDef           *)    WDT1_BASE)
#define hwp_wdt2        ((WDT_TypeDef           *)    WDT2_BASE)
#define hwp_iwdt        ((WDT_TypeDef           *)    IWDT_BASE)
#define hwp_usbc_x      ((USBC_X_Typedef        *)    USBC_X_BASE))

#define USART1        hwp_usart1
#define USART2        hwp_usart2
#define USART3        hwp_usart3
#define USART4        hwp_usart4
#define USART5        hwp_usart5

#define DMA1          hwp_dmac1
#define DMA2          hwp_dmac2

#define FLASH1        hwp_qspi1
#define FLASH2        hwp_qspi2
#define FLASH3        hwp_qspi3
#define FLASH4        hwp_qspi4

#define SDIO1          hwp_sdmmc1
#define SDIO2          hwp_sdmmc2

#define SPI1          hwp_spi1
#define SPI2          hwp_spi2
#define SPI3          hwp_spi3
#define SPI4          hwp_spi4

#define GPTIM1        hwp_gptim1
#define GPTIM2        hwp_gptim2
#define GPTIM3        hwp_gptim3
#define GPTIM4        hwp_gptim4
#define GPTIM5        hwp_gptim5
#define BTIM1         hwp_btim1
#define BTIM2         hwp_btim2
#define BTIM3         hwp_btim3
#define BTIM4         hwp_btim4
#define LPTIM1        hwp_lptim1
#define LPTIM2        hwp_lptim2
#define LPTIM3        hwp_lptim3
#define TRNG          hwp_trng


#define PSRAM         hwp_psramc

/** HCPU2LCPU mailbox instance */
#define H2L_MAILBOX   ((MAILBOX_CH_TypeDef *)HMAILBOX_BASE)
/** HCPU mutex instance channel1 */
#define HMUTEX_CH1    ((MUTEX_CH_TypeDef *)&hwp_hmailbox->C1EXR)
/** HCPU mutex instance channel2 */
#define HMUTEX_CH2    ((MUTEX_CH_TypeDef *)&hwp_hmailbox->C2EXR)

/** LCPU2HCPU mailbox instance */
#define L2H_MAILBOX   ((MAILBOX_CH_TypeDef *)LMAILBOX_BASE)
/** LCPU mutex instance channel1 */
#define LMUTEX_CH1    ((MUTEX_CH_TypeDef *)&hwp_lmailbox->C1EXR)
/** LCPU mutex instance channel2 */
#define LMUTEX_CH2    ((MUTEX_CH_TypeDef *)&hwp_lmailbox->C2EXR)

/** EPIC instance */
#define EPIC            hwp_epic
#define LCDC1           hwp_lcdc1
#define LCDC2           hwp_lcdc2


#define I2C1           hwp_i2c1
#define I2C2           hwp_i2c2
#define I2C3           hwp_i2c3
#define I2C4           hwp_i2c4
#define I2C5           hwp_i2c5
#define I2C6           hwp_i2c6

#define CRC            hwp_crc
/** EZIP instance */
#define EZIP           hwp_ezip


#define DMA1_Channel1       ((DMA_Channel_TypeDef *) &DMA1->CCR1)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) &DMA1->CCR2)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) &DMA1->CCR3)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) &DMA1->CCR4)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) &DMA1->CCR5)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) &DMA1->CCR6)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) &DMA1->CCR7)
#define DMA1_Channel8       ((DMA_Channel_TypeDef *) &DMA1->CCR8)
#define DMA1_CSELR          ((DMA_Request_TypeDef *) &DMA1->CSELR1)
#define DMA1_CHANNEL_NUM    (8)
#define DMA2_Channel1       ((DMA_Channel_TypeDef *) &DMA2->CCR1)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *) &DMA2->CCR2)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *) &DMA2->CCR3)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *) &DMA2->CCR4)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *) &DMA2->CCR5)
#define DMA2_Channel6       ((DMA_Channel_TypeDef *) &DMA2->CCR6)
#define DMA2_Channel7       ((DMA_Channel_TypeDef *) &DMA2->CCR7)
#define DMA2_Channel8       ((DMA_Channel_TypeDef *) &DMA2->CCR8)
#define DMA2_CSELR          ((DMA_Request_TypeDef *) &DMA2->CSELR1)
#define DMA2_CHANNEL_NUM    (8)

/**
 *
 * @} Peripheral_memory_map
 */


#define HCPU2LCPU_OFFSET      (0x0A000000)
#define LCPUROM2HCPU_OFFSET   (0x0B000000)
#define LCPUITCM2HCPU_OFFSET  (0x0B000000)
#define LCPUDTCM2HCPU_OFFSET  (0x0B000000)


typedef enum
{
    RESET = 0,
    SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
    SF_ERROR = 0,
    SF_SUCCESS = !SF_ERROR
} ErrorStatus;


#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define IS_LPUART_INSTANCE(INSTANCE)    (0)

#define HCPU_IS_SRAM_ADDR(addr)  (((uint32_t)(addr) >= HPSYS_RAM0_BASE) && ((uint32_t)(addr) < HPSYS_RAM_END))

/**
  * @brief  Convert HCPU SRAM address which can be used by LCPU
  * @param  addr HCPU SRAM address
  * @retval address which can be accessed by LCPU
*/
#define HCPU_ADDR_2_LCPU_ADDR(addr)    (HCPU_IS_SRAM_ADDR((addr)) ? (uint32_t)((addr) + HCPU2LCPU_OFFSET) : (uint32_t)(addr))


/**
  * @brief  Convert LCPU SRAM address which can be used by HCPU
  * @param  addr LCPU SRAM address
  * @retval address which can be accessed by HCPU
*/
#define LCPU_ADDR_2_HCPU_ADDR(addr) (addr)
/**
  * @brief  Convert LCPU ROM address which can be used by HCPU
  * @param  addr LCPU ROM address
  * @retval address which can be accessed by HCPU
*/
#define LCPU_ROM_ADDR_2_HCPU_ADDR(addr) ((addr) + LCPUROM2HCPU_OFFSET)
/**
  * @brief  Convert LCPU ITCM address which can be used by HCPU
  * @param  addr LCPU ITCM address
  * @retval address which can be accessed by HCPU
*/
#define LCPU_ITCM_ADDR_2_HCPU_ADDR(addr) ((addr) + LCPUITCM2HCPU_OFFSET)
/**
  * @brief  Convert LCPU DTCM address which can be used by HCPU
  * @param  addr LCPU ITCM address
  * @retval address which can be accessed by HCPU
*/
#define LCPU_DTCM_ADDR_2_HCPU_ADDR(addr) ((addr) + LCPUDTCM2HCPU_OFFSET)


#define HCPU_MPI_SBUS_ADDR(addr)       (addr)

#define GPADC_CALIB_FLOW_VERSION        (1)

#ifndef LCPU_BOOT_ADDR
#define LCPU_BOOT_ADDR          (LCPU_RAM_DATA_START_ADDR+LCPU_RAM_DATA_SIZE-4)
#endif

#define IS_LCPU(id)  ((*id)&1)


#if defined (USE_HAL_DRIVER)
#include "bf0_hal.h"
#endif /* USE_HAL_DRIVER */


#ifdef __cplusplus
}
#endif /* __cplusplus */

/**
  * @} CMSIS_Device
  */

/**
  * @}
  */

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
