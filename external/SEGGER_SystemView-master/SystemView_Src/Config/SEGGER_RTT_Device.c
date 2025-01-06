/*
 * File      : kservice.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-07-20     FlyLu        the first version
 * 2018-07-20     Tanek        use idle hook as receive
 */

#include <rtdevice.h>
#include <rthw.h>
#include "SEGGER_RTT.h"

static struct rt_serial_device segger_serial;

static void serial1_idle(void)
{
    rt_hw_serial_isr(&segger_serial, RT_SERIAL_EVENT_RX_IND);
}

static rt_err_t segger_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    static rt_bool_t sethook = RT_FALSE;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(arg != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        if (sethook)
        {
            rt_thread_idle_delhook(serial1_idle);
            sethook = RT_FALSE;
        }
        break;

    case RT_DEVICE_CTRL_SET_INT:
        if (!sethook)
        {
            rt_thread_idle_sethook(serial1_idle);
            sethook = RT_TRUE;
        }
        break;
    }

    return RT_EOK;
}

static int segger_putc(struct rt_serial_device *serial, char c)
{
    RT_ASSERT(serial != RT_NULL);

    return SEGGER_RTT_Write(0, &c, 1);
}

static int segger_getc(struct rt_serial_device *serial)
{
    char ch;

    RT_ASSERT(serial != RT_NULL);

    if (SEGGER_RTT_Read(0, &ch, 1) == 1)
    {
        return ch;
    }
    else
    {
        return -1;
    }
}

int hw_segger_init(void)
{
    static const struct rt_uart_ops segger_uart_ops =
    {
        NULL,
        segger_control,
        segger_putc,
        segger_getc,
    };

    segger_serial.ops = &segger_uart_ops;
    segger_serial.config.bufsz = RT_SERIAL_RB_BUFSZ;

    /* register segger rtt as serial device */
    rt_hw_serial_register(&segger_serial,
                          PKG_SERIAL_DEVICE_NAME,
                          RT_DEVICE_FLAG_RDWR  | RT_DEVICE_FLAG_INT_RX,
                          NULL);

    return 0;
}
//INIT_BOARD_EXPORT(hw_segger_init);
