/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "vg_lite.h"

#ifdef BSP_USING_TOUCHD
    #include "drv_touch.h"
#endif
#include "mem_section.h"
#include "vglite/tc_vglite.h"

extern void tc_vglite_linear_grad_entry(void);
extern void tc_vglite_glyphs_entry(void);
extern void tc_vglite_rotate_entry(void);
extern void tc_vglite_coverflow_entry_public(void);

static rt_thread_t demo_thread = RT_NULL;
static volatile int demo_exit_flag = 0;

int demo_should_exit(void)
{
    return demo_exit_flag;
}

void demo_request_exit(void)
{
    demo_exit_flag = 1;
}

void demo_clear_exit(void)
{
    demo_exit_flag = 0;
}

static void demo_thread_entry(void *parameter)
{
    void (*entry)(void) = (void (*)(void))parameter;
    entry();
    demo_thread = RT_NULL;
}

int demo_run(int demo_id)
{
    void (*entry)(void) = RT_NULL;
    const char *name = "";
    rt_err_t init_err = RT_ERROR;

    switch (demo_id)
    {
    case 0:
        entry = tc_vglite_linear_grad_entry;
        name = "linear_grad";
        break;
    case 1:
        entry = tc_vglite_glyphs_entry;
        name = "glyphs";
        break;
    case 2:
        entry = tc_vglite_rotate_entry;
        name = "rotate";
        break;
    case 3:
        entry = tc_vglite_coverflow_entry_public;
        name = "coverflow";
        break;
    default:
        rt_kprintf("Invalid demo id: %d\n", demo_id);
        return -1;
    }

    if (demo_thread != RT_NULL)
    {
        rt_kprintf("Stopping current demo...\n");
        demo_request_exit();
        for (int i = 0; i < 50 && demo_thread != RT_NULL; i++)
        {
            rt_thread_mdelay(100);
        }
        if (demo_thread != RT_NULL)
        {
            rt_thread_delete(demo_thread);
            demo_thread = RT_NULL;
        }
    }

    demo_clear_exit();
    tc_vglite_cleanup();
    rt_thread_mdelay(1000);

    rt_kprintf("Initializing...\n");
    rt_thread_mdelay(500);
    for (int retry = 0; retry < 3; retry++)
    {
        init_err = tc_vglite_init();
        if (init_err == RT_EOK)
        {
            break;
        }
        rt_kprintf("Init failed, retry %d...\n", retry + 1);
        rt_thread_mdelay(500);
    }

    if (init_err != RT_EOK)
    {
        rt_kprintf("Demo '%s' start aborted.\n", name);
        return init_err;
    }

    demo_thread = rt_thread_create(name,
                                   demo_thread_entry,
                                   (void *)entry,
                                   8192,
                                   10,
                                   10);
    if (demo_thread != RT_NULL)
    {
        rt_thread_startup(demo_thread);
        rt_kprintf("Demo '%s' started\n", name);
    }

    return 0;
}

void demo_list(void)
{
    rt_kprintf("\n========== Available Demos ==========\n");
    rt_kprintf("0: linear_grad  - Linear gradient demo\n");
    rt_kprintf("1: glyphs       - Glyphs render demo\n");
    rt_kprintf("2: rotate       - Rotate image demo\n");
    rt_kprintf("3: coverflow    - Coverflow effect demo\n");
    rt_kprintf("======================================\n\n");
    rt_kprintf("Usage: demo_run_handler <0-3>\n");
    rt_kprintf("Example: demo_run_handler 0\n\n");
}

static void demo_run_handler(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: demo_run_handler <0-3>\n");
        return;
    }
    demo_run(atoi(argv[1]));
}

    MSH_CMD_EXPORT(demo_run_handler, Run demo: demo_run_handler <0-3>);

int main(void)
{
    demo_list();

    while (1)
    {
        rt_thread_mdelay(5000);
        rt_kprintf("__main loop__\r\n");
    }

    return RT_EOK;
}
