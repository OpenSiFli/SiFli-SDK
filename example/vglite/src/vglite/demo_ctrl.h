/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DEMO_CTRL_H__
#define __DEMO_CTRL_H__

#include <rtthread.h>

int demo_should_exit(void);
void demo_request_exit(void);
void demo_clear_exit(void);

static inline rt_bool_t demo_delay_ms(rt_uint32_t timeout_ms)
{
    const rt_uint32_t step_ms = 20;

    while (timeout_ms > 0)
    {
        if (demo_should_exit())
        {
            return RT_FALSE;
        }

        if (timeout_ms > step_ms)
        {
            rt_thread_mdelay(step_ms);
            timeout_ms -= step_ms;
        }
        else
        {
            rt_thread_mdelay(timeout_ms);
            timeout_ms = 0;
        }
    }

    return demo_should_exit() ? RT_FALSE : RT_TRUE;
}

#endif
