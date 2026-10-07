/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    board.c
 * @brief   Board-level init for the QEMU Cortex-M33 virtual board.
 *
 * Replaces the real SoC board init: no clock tree / pinmux / power. Only the
 * things QEMU models are initialized -- the SysTick as RT-Thread tick source
 * and the console device.
 *
 * The console is the emulated CMSDK UART0 wired to the host by `-serial stdio`,
 * not semihosting. This comment used to say otherwise, and the mistake was
 * worth more than a typo: semihosting output works but its SYS_READC blocks the
 * vCPU, so it cannot back an interactive shell -- see the note at the top of
 * uart_console.c, which is the file that actually registers the device.
 */

#include "register.h"

#include <rthw.h>
#include <rtthread.h>

#include <string.h>
#include <stdint.h>

#include "board.h"
#include "uart_console.h"

extern uint32_t SystemCoreClock;

/* SysTick: RT-Thread 1ms tick */
void SysTick_Handler(void)
{
    rt_tick_increase();
}

/**
 * This function will initialize the QEMU virtual board.
 */
void rt_hw_board_init(void)
{
    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif

    /* Console device (semihosting) */
    uart_console_init();

    /* SysTick 1ms */
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /* Set the shell console output device */
#if defined(RT_USING_CONSOLE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
}

/* Weak board hooks expected by the SDK; nothing to do under QEMU */
void BSP_IO_Init(void)
{
}

void print_sysinfo(char *buf, uint32_t buf_len)
{
    if (buf)
    {
        memset(buf, 0, buf_len);
        rt_snprintf(buf, buf_len, "System info on QEMU Cortex-M33");
    }
}
