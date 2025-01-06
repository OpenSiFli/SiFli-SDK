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
#include "register.h"

int main(void)
{
    while (1)
    {
#ifndef SF32LB52X
        if (HAL_PMU_LXT_DISABLED())
        {
            HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
            rt_thread_mdelay(15 * 1000);
        }
        else
        {
            //rt_thread_mdelay(3600000);
            break;
        }
        //rt_thread_mdelay(5000);
        //rt_kprintf("lcpu test \n");
#else
        break;
#endif
    }
}
