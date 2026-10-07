/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    system_qemu.c
 * @brief   Minimal system init for the QEMU Cortex-M33 virtual board.
 *
 * Replaces the real SoC system_bf0_ap.c: only core Cortex-M33 setup is done
 * (vector table relocation + FPU access). No real clock tree / power / cache
 * initialization, since the QEMU machine models no SiFli peripherals.
 */

#include <stdint.h>
#include "register.h"

extern uint32_t __Vectors;

/* Core clock used for SysTick calibration. This must match the QEMU machine's
 * virtual clock so that SysTick_Config() yields a real 1 ms tick.
 * MPS2-AN505: SYSCLK_FRQ = 20 MHz in QEMU hw/arm/mps2-tz.c, and the SysTick is
 * clocked by the CPU MAINCLK, so it runs at 20 MHz. (Not 48 MHz — that value
 * was a placeholder; measured 2.4 ms/tick with a 48000 reload confirmed 20 MHz.)
 * If you switch to another machine, re-check its SYSCLK. */
uint32_t SystemCoreClock = 20000000U;

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 48000000U;
}

void SystemInit(void)
{
#if defined(__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
    /* Set vector table base address (code region @ 0x10000000) */
    SCB->VTOR = (uint32_t)&__Vectors;
#endif

    /* Enable CP10/CP11 (FPU) full access */
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}
