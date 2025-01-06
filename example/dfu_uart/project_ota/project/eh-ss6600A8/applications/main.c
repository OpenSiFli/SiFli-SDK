/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */
#include <stdbool.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include "dfu_uart.h"

int main(void)
{
    dfu_uart_reset_handler();
    int count = 0;

    while (1)
    {
        count++;
        //rt_kprintf("test\n");
        rt_thread_mdelay(3600000);
    }
}



