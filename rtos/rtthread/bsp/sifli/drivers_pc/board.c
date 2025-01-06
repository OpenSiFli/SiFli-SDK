/**
  ******************************************************************************
  * @file   board.c
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

#include <rthw.h>
#include <rtthread.h>

#include <stdlib.h>

#include "board.h"
#include "uart_console.h"
#include "stdint.h"

/**
 * @addtogroup simulator on win32
 */
rt_uint8_t *heap;

rt_uint8_t *rt_hw_sram_init(void)
{
    rt_uint8_t *heap;
    heap = malloc(RT_HEAP_SIZE);
    if (heap == RT_NULL)
    {
        rt_kprintf("there is no memory in pc.");
#ifdef _WIN32
        _exit(1);
#else
        exit(1);
#endif
    }
    return heap;
}

#ifdef _WIN32
    #include <windows.h>
#endif

void rt_hw_win32_low_cpu(void)
{
#ifdef _WIN32
    /* in windows */
    Sleep(1000);
#else
    /* in linux */
    sleep(1);
#endif
}

#ifdef _MSC_VER
    #ifndef _CRT_TERMINATE_DEFINED
        #define _CRT_TERMINATE_DEFINED
        _CRTIMP __declspec(noreturn) void __cdecl exit(__in int _Code);
        _CRTIMP __declspec(noreturn) void __cdecl _exit(__in int _Code);
        _CRTIMP void __cdecl abort(void);
    #endif
#endif

void rt_hw_exit(void)
{
    rt_kprintf("RT-Thread, bye\n");
#if !defined(_WIN32) && defined(__GNUC__)
    /* *
     * getchar reads key from buffer, while finsh need an non-buffer getchar
     * in windows, getch is such an function, in linux, we had to change
     * the behaviour of terminal to get an non-buffer getchar.
     * in usart_sim.c, set_stty is called to do this work
     * */
    {
        extern void restore_stty(void);
        restore_stty();
    }
#endif
    exit(0);
}

#if defined(RT_USING_FINSH)
    #include <finsh.h>
    FINSH_FUNCTION_EXPORT_ALIAS(rt_hw_exit, exit, exit rt - thread);
    FINSH_FUNCTION_EXPORT_ALIAS(rt_hw_exit, __cmd_quit, exit rt - thread);
#endif /* RT_USING_FINSH */

int rt_in_system_heap(void *ptr)
{
    if (((uint8_t *)ptr >= heap) && ((uint8_t *)ptr - (uint8_t *)heap) < RT_HEAP_SIZE)
        return 1;
    else
        return 0;
}

void rt_simu_heap_init(void)
{
    static int initialized = 0;

    if (initialized)
        return;
    initialized = 1;
    /* init system memory */
    heap = rt_hw_sram_init();

#ifdef RT_USING_HEAP
    /* init memory system */
    rt_system_heap_init((void *)heap, (void *)&heap[RT_HEAP_SIZE - 1]);
#endif
}

/**
 * This function will initial win32
 */
void rt_hw_board_init(void)
{
    rt_simu_heap_init();
    uart_console_init();
    {
        extern int uart_pc_init(void);
        uart_pc_init();
    }
#if defined(CFG_EMB)
    {
        extern int uart_pc_ahi_init(void);
        uart_pc_ahi_init();
    }
#endif //CFG_EMB
    //lcd_dummy_hw_init();
#ifdef LCD_SDL2

#endif

#ifdef _WIN32
    //rt_thread_idle_sethook(rt_hw_win32_low_cpu);
#endif

#if defined(RT_USING_CONSOLE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
    //rt_console_set_device("h4tl");
#endif
}

void print_sysinfo(char *buf, uint32_t buf_len)
{
    if (buf)
    {
        memset(buf, 0, buf_len);

        rt_snprintf(buf, buf_len, "System info on simulator");
    }
}

void BSP_IO_Init(void)
{
}

/*@}*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
