/**
  ******************************************************************************
  * @file   uart_console.c
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

#include <stdio.h>

#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <rtdevice.h>

/* uart driver */
struct console_uart
{
    int rx_ready;

    struct rt_ringbuffer rb;
    rt_uint8_t rx_buffer[2048];
    struct rt_semaphore  sem; /*API lock semaphore*/
} _console_uart;
static struct rt_serial_device _serial;

#define SAVEKEY(key)  do { char ch = key; rt_ringbuffer_put_force(&(_console_uart.rb), &ch, 1); } while (0)

#ifdef _WIN32
#include  <windows.h>
#include  <mmsystem.h>
#include  <conio.h>

/*
 * Handler for OSKey Thread
 */
static HANDLE       OSKey_Thread;
static DWORD        OSKey_ThreadID;

static DWORD WINAPI ThreadforKeyGet(LPVOID lpParam);
void console_lowlevel_init(void)
{
    /*
     * create serial thread that receive key input from keyboard
     */

    OSKey_Thread = CreateThread(NULL,
                                0,
                                (LPTHREAD_START_ROUTINE)ThreadforKeyGet,
                                0,
                                CREATE_SUSPENDED,
                                &OSKey_ThreadID);
    if (OSKey_Thread == NULL)
    {
        //Display Error Message
        return;
    }

    SetThreadPriority(OSKey_Thread,
                      THREAD_PRIORITY_NORMAL);
    SetThreadPriorityBoost(OSKey_Thread,
                           TRUE);
    SetThreadAffinityMask(OSKey_Thread,
                          0x01);
    /*
     * Start OS get key Thread
     */
    ResumeThread(OSKey_Thread);
}

static DWORD WINAPI ThreadforKeyGet(LPVOID lpParam)
#else /* POSIX version */

#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <signal.h>
#include <termios.h> /* for tcxxxattr, ECHO, etc */
#include <unistd.h> /* for STDIN_FILENO */

static void *ThreadforKeyGet(void *lpParam);
static pthread_t OSKey_Thread;

void console_lowlevel_init(void)
{
    int res;

    res = pthread_create(&OSKey_Thread, NULL, &ThreadforKeyGet, NULL);
    if (res)
    {
        printf("pthread create faild, <%d>\n", res);
        exit(EXIT_FAILURE);
    }
}

static struct termios oldt, newt;
/*simulate windows' getch(), it works!!*/
static void set_stty(void)
{
    /* get terminal input's attribute */
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    /* set termios' local mode */
    newt.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

void restore_stty(void)
{
    /* recover terminal's attribute */
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

#define getch  getchar
static void *ThreadforKeyGet(void *lpParam)
#endif /* not _WIN32*/
{
    /*
    * left  key(��)�� 0xe04b
    * up    key(��)�� 0xe048
    * right key(��)�� 0xe04d
    * down  key(��)�� 0xe050
    */
    unsigned char key;

#ifndef _WIN32
    sigset_t  sigmask, oldmask;
    /* set the getchar without buffer */
    sigfillset(&sigmask);
    pthread_sigmask(SIG_BLOCK, &sigmask, &oldmask);
    set_stty();
#endif

    (void)lpParam;              //prevent compiler warnings

    for (;;)
    {
        key = getch();
#ifdef _WIN32
        if (key == 0xE0)
        {
            key = getch();

            if (key == 0x48) //up key , 0x1b 0x5b 0x41
            {
                SAVEKEY(0x1b);
                SAVEKEY(0x5b);
                SAVEKEY(0x41);
            }
            else if (key == 0x50)//0x1b 0x5b 0x42
            {
                SAVEKEY(0x1b);
                SAVEKEY(0x5b);
                SAVEKEY(0x42);
            }
            else if (key == 0x4b)//<- 0x1b 0x5b 0x44
            {
                SAVEKEY(0x1b);
                SAVEKEY(0x5b);
                SAVEKEY(0x44);
            }
            else if (key == 0x4d)//<- 0x1b 0x5b 0x43
            {
                SAVEKEY(0x1b);
                SAVEKEY(0x5b);
                SAVEKEY(0x43);
            }

            continue;
        }
#endif
        SAVEKEY(key);

        /* Notfiy serial ISR */
        rt_hw_serial_isr(&_serial, RT_SERIAL_EVENT_RX_IND);
    }
} /*** ThreadforKeyGet ***/

static rt_err_t console_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    /* no baudrate, nothing */

    return RT_EOK;
}

static rt_err_t console_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct console_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct console_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        uart->rx_ready = 0;
        break;
    case RT_DEVICE_CTRL_SET_INT:
        uart->rx_ready = 1;
        break;
    }

    return RT_EOK;
}

static int console_putc(struct rt_serial_device *serial, char c)
{
    struct console_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct console_uart *)serial->parent.user_data;

#if 0 /* Enable it if you want to save the console log */
    {
        static FILE *fp = NULL;

        if (fp == NULL)
            fp = fopen("log.txt", "wb+");

        if (fp != NULL)
            fwrite(buffer, size, 1, fp);
    }
#endif
    rt_sem_take(&uart->sem, RT_WAITING_FOREVER);
    fputc(c, stdout);
    rt_sem_release(&uart->sem);
#if 0
    int level = rt_hw_interrupt_disable();
    fwrite(&c, 1, 1, stdout);
    fflush(stdout);
    rt_hw_interrupt_enable(level);
#endif
    return 1;
}

static int console_getc(struct rt_serial_device *serial)
{
    char ch;
    struct console_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct console_uart *)serial->parent.user_data;

    if (rt_ringbuffer_getchar(&(uart->rb), &ch)) return ch;

    return -1;
}

rt_size_t console_dma(struct rt_serial_device *console, rt_uint8_t *buf, rt_size_t size, int direction)
{
    DWORD  dNoOfBytesWritten = 0;

    if (direction == RT_SERIAL_DMA_TX)
    {
        struct console_uart *uart;
        RT_ASSERT(console != RT_NULL);
        uart = (struct console_uart *)console->parent.user_data;

        rt_sem_take(&uart->sem, RT_WAITING_FOREVER);
        fwrite(buf, 1, size, stdout);
        rt_sem_release(&uart->sem);
        rt_hw_serial_isr(console, RT_SERIAL_EVENT_TX_DMADONE);
    }
    return dNoOfBytesWritten;
}


static const struct rt_uart_ops console_uart_ops =
{
    console_configure,
    console_control,
    console_putc,
    console_getc,
    console_dma,
};

int uart_console_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    struct console_uart *uart;
    struct rt_serial_device *serial;

    uart = &_console_uart;
    serial = &_serial;

    uart->rx_ready = 0;

    serial->ops    = &console_uart_ops;
    serial->config = config;
    /* initialize ring buffer */
    rt_ringbuffer_init(&uart->rb, uart->rx_buffer, sizeof(uart->rx_buffer));

    /* register UART device */
    rt_hw_serial_register(serial, "console",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX
#ifdef CONSOLE_UART_DMA
                          | RT_DEVICE_FLAG_DMA_TX | RT_DEVICE_FLAG_DMA_RX
#endif
                          ,
                          uart);

    console_lowlevel_init();

    rt_sem_init(&_console_uart.sem,     "uart_console", 1, RT_IPC_FLAG_FIFO);
    return 0;
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
