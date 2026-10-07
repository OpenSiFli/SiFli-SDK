/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    register.h
 * @brief   QEMU Cortex-M33 device header.
 *
 * Provides the CMSIS device prerequisites (IRQn_Type, __NVIC_PRIO_BITS, ...)
 * that the real SoC register.h supplies, so core_cm33.h and the RT-Thread
 * Cortex-M33 port compile without any SiFli chip register map.
 */

#ifndef __QEMU_REGISTER_H__
#define __QEMU_REGISTER_H__

/* IRQ numbers: core exceptions (negative) + emulated device IRQs (>= 0) */
typedef enum
{
    Reset_IRQn             = -15,  /*  1 Reset */
    NonMaskableInt_IRQn    = -14,  /*  2 Non maskable interrupt */
    HardFault_IRQn         = -13,  /*  3 Hard fault */
    MemoryManagement_IRQn  = -12,  /*  4 Memory management */
    BusFault_IRQn          = -11,  /*  5 Bus fault */
    UsageFault_IRQn        = -10,  /*  6 Usage fault */
    SecureFault_IRQn       =  -9,  /*  7 Secure fault */
    SVCall_IRQn            =  -5,  /* 11 System service call */
    DebugMonitor_IRQn      =  -4,  /* 12 Debug monitor */
    PendSV_IRQn            =  -2,  /* 14 Pend SV */
    SysTick_IRQn           =  -1,  /* 15 System tick */
    /* device IRQs (MPS2-AN505 CMSDK UART is among these; see QEMU version) */
    UART0_IRQn             =   1,
    UART1_IRQn             =   2,
    UART2_IRQn             =   3,
    UART3_IRQn             =   4,
    UART4_IRQn             =   5,
} IRQn_Type;

/* Core revision r0p1 */
#define __CM33_REV          (0x0000U)

/* Number of priority bits implemented: Cortex-M33 = 3 */
#define __NVIC_PRIO_BITS    (3U)

/* CMSIS feature flags */
#define __Vendor_SysTickConfig (0U)  /* use standard CMSIS SysTick_Config */
#define __MPU_PRESENT       (1U)
#define __VTOR_PRESENT      (1U)
#define __FPU_PRESENT       (1U)
#define __DSP_PRESENT       (1U)
/* The QEMU cortex-m33 model does not implement L1 caches */
#define __ICACHE_PRESENT    (0U)
#define __DCACHE_PRESENT    (0U)

#include "core_cm33.h"

#endif /* __QEMU_REGISTER_H__ */
