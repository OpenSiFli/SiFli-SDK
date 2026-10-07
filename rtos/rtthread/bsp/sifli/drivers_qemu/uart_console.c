/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    uart_console.c
 * @brief   RT-Thread console on the MPS2's CMSDK APB UART.
 *
 * The console is the emulated UART0, wired to the host through QEMU's
 * `-serial stdio`. Both directions are polled -- no interrupts -- because the
 * device is simple enough that polling costs nothing here and it avoids
 * depending on this board's IRQ numbering, which is not obvious and fails
 * silently when wrong.
 *
 * The obvious alternative, semihosting SYS_WRITEC/SYS_READC, was tried first
 * and only half works. Output is fine, but SYS_READC **blocks the vCPU** until a
 * character arrives, so a reader that calls it during startup freezes the whole
 * guest -- including the shell that was about to print its prompt. The symptom
 * is a board that boots, prints its banner and then does nothing, with no fault
 * to explain it. Semihosting input cannot back an interactive console.
 *
 * Register layout (CMSDK APB UART):
 *   0x00 DATA        bits 0..7
 *   0x04 STATE       bit0 TX full, bit1 RX full, bit2 TX overrun, bit3 RX overrun
 *   0x08 CTRL        bit0 TX enable, bit1 RX enable, bit2 TX irq, bit3 RX irq
 *   0x0C INTSTATUS   bit0 TX, bit1 RX
 *   0x10 BAUDDIV
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "register.h"
#include "uart_console.h"

/* UART0 on MPS2-AN505.
 *
 * Note the base: the AN505 (IoTKit) puts its UARTs at 0x40200000..0x40204000,
 * not at the 0x40004000 of the older CMSDK MPS2 images (AN385 and friends).
 * Using the wrong one is not a fault -- the address reads back as an
 * unimplemented region, the transmit-full bit never clears, and uart_putc_raw
 * spins forever. The board then boots and prints nothing at all, which looks
 * like a dead console rather than a wrong address.
 *
 * QEMU will tell you the truth: start it with "-monitor stdio -serial none" and
 * run "info mtree -f". */
#define CMSDK_UART0_BASE    0x40200000UL
#define UART_DATA           (CMSDK_UART0_BASE + 0x00UL)
#define UART_STATE          (CMSDK_UART0_BASE + 0x04UL)
#define UART_CTRL           (CMSDK_UART0_BASE + 0x08UL)
#define UART_BAUDDIV        (CMSDK_UART0_BASE + 0x10UL)

#define UART_STATE_TX_FULL  (1UL << 0)
#define UART_STATE_RX_FULL  (1UL << 1)
#define UART_CTRL_TX_EN     (1UL << 0)
#define UART_CTRL_RX_EN     (1UL << 1)

/* 16 is the CMSDK UART's reset value and what QEMU models happily. */
#define UART_BAUDDIV_VALUE  16UL

#define RX_BUFFER_SIZE      256
#define RX_THREAD_STACK     1024
#define RX_THREAD_PRIORITY  25
#define RX_THREAD_TICK      20
#define RX_POLL_TICKS       2

#define UART_REG(addr)      (*(volatile rt_uint32_t *)(addr))

/* Device state. Declared before the ops because console_getc reads the buffer
 * the poller fills. */
static struct rt_serial_device _console_serial;

static struct
{
    struct rt_ringbuffer rb;
    rt_uint8_t buffer[RX_BUFFER_SIZE];
} _console_rx;

static void uart_putc_raw(char c)
{
    while (UART_REG(UART_STATE) & UART_STATE_TX_FULL)
    {
        /* wait for the transmit holding register */
    }
    UART_REG(UART_DATA) = (rt_uint32_t)(rt_uint8_t)c;
}

static rt_err_t console_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    return RT_EOK;
}

static rt_err_t console_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    return RT_EOK;
}

static int console_putc(struct rt_serial_device *serial, char c)
{
    uart_putc_raw(c);
    return 1;
}

static int console_getc(struct rt_serial_device *serial)
{
    rt_uint8_t ch;

    if (rt_ringbuffer_getchar(&_console_rx.rb, &ch))
    {
        return (int)ch;
    }

    return -1;
}

static rt_size_t console_dma(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    if (direction == RT_SERIAL_DMA_TX)
    {
        rt_size_t i;

        for (i = 0; i < size; i++)
        {
            uart_putc_raw((char)buf[i]);
        }
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);
    }
    return 0;
}

static const struct rt_uart_ops console_uart_ops =
{
    console_configure,
    console_control,
    console_putc,
    console_getc,
    console_dma,
};

/**
 * @brief  Move host input into the serial framework.
 *
 * A dedicated thread, polling. Both other options were tried and rejected:
 *
 * - A thread reading semihosting SYS_READC: that call blocks the whole vCPU,
 *   so the guest freezes the moment the thread parks -- including the shell
 *   that was about to print its prompt.
 * - The idle hook: it looks allocation-free and therefore safe on this board's
 *   small heap, but it only runs when the system is genuinely idle, and it
 *   dropped characters under a burst.
 *
 * A thread on the real UART has neither problem: the read never blocks the
 * guest, and the loop runs at a priority that keeps up with the host.
 */
static void console_rx_thread_entry(void *parameter)
{
    while (1)
    {
        while (UART_REG(UART_STATE) & UART_STATE_RX_FULL)
        {
            rt_uint8_t ch = (rt_uint8_t)(UART_REG(UART_DATA) & 0xFFu);

            rt_ringbuffer_putchar_force(&_console_rx.rb, ch);

            /*
             * Only notify once something has opened the device.
             * rt_hw_serial_isr(RX_IND) drains console_getc() into the rx fifo
             * that rt_serial_open allocated, and asserts if it does not exist
             * yet -- and this thread starts long before the shell opens its
             * input. Bytes that arrive early stay in the ring buffer and are
             * drained by the first notification afterwards.
             */
            if (_console_serial.serial_rx != RT_NULL)
            {
                rt_hw_serial_isr(&_console_serial, RT_SERIAL_EVENT_RX_IND);
            }
        }

        rt_thread_mdelay(RX_POLL_TICKS);
    }
}

int uart_console_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    struct rt_serial_device *serial = &_console_serial;

    rt_ringbuffer_init(&_console_rx.rb, _console_rx.buffer, sizeof(_console_rx.buffer));

    /* Bring the UART up: divider first, then enable both directions. */
    UART_REG(UART_BAUDDIV) = UART_BAUDDIV_VALUE;
    UART_REG(UART_CTRL) = UART_CTRL_TX_EN | UART_CTRL_RX_EN;

    serial->ops    = &console_uart_ops;
    serial->config = config;

    /* INT_RX is required, not optional: the shell opens its input with that
     * flag, and rt_serial_open rejects the open outright if the device was
     * registered without it. The shell then holds a NULL device and faults on
     * its first read. */
    rt_hw_serial_register(serial, "console",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, RT_NULL);

    return 0;
}

/**
 * @brief  Start the input polling thread.
 *
 * Deliberately NOT done in uart_console_init: that runs from rt_hw_board_init,
 * which RT-Thread calls before it has initialized the scheduler and the object
 * system. Creating and starting a thread there corrupts the ready list. An
 * INIT_APP_EXPORT runs from rt_components_init, by which point the kernel is up.
 */
static int console_rx_start(void)
{
    rt_thread_t rx_thread = rt_thread_create("qemu-rx", console_rx_thread_entry, RT_NULL,
                            RX_THREAD_STACK, RX_THREAD_PRIORITY,
                            RX_THREAD_TICK);

    if (rx_thread != RT_NULL)
    {
        rt_thread_startup(rx_thread);
    }

    return 0;
}
INIT_APP_EXPORT(console_rx_start);
