/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "lvgl.h"
#include "demo_mulroller.h"

/**
  * @brief  Main program: init LVGL, build the mulroller demo, run the task loop.
  *
  * On the PC simulator (RT_USING_USER_MAIN=n) the CRT/startup owns main(), and
  * RT-Thread's main thread calls app_main() instead; on a board, main() is the
  * RT-Thread entry. Use the right symbol for each.
  */
#ifdef BSP_USING_PC_SIMULATOR
int app_main(void)
#else
int main(void)
#endif
{
    rt_err_t ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }
    lv_ex_data_pool_init();

    demo_mulroller_init();

    while (1)
    {
        rt_uint32_t ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;
}
