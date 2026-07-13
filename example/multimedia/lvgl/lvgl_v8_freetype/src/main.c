/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "stdio.h"

#ifndef _WIN32
#include "drv_flash.h"
#include "dfs_file.h"

int mnt_init(void)
{
    rt_kprintf("===auto_mnt_init===\n");

    const char *name = "flash0";
    
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, (char *)name);

    if (dfs_mount(name, "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        rt_kprintf("mount fs on flash to root fail\n");
    }

    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif /* _WIN32 */

extern void lv_example_font_switch_demo(void);

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }
    lv_ex_data_pool_init();

    lv_example_font_switch_demo();
    
    while (1)
    {
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;

}
