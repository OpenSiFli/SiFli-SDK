/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "lvgl.h"

/*
 * Exactly one widget example is selected via menuconfig:
 *   LVSF v8 widget examples  --->  Select widget example
 * The matching demo sources under src/widgets/<name>/ are compiled by the
 * SConscript; every other demo folder is left out of the build.
 */
#if defined(LV_WIDGET_DEMO_BASECHART)
    #include "demo_basechart.h"
    #define widget_example_init   demo_basechart_init
#elif defined(LV_WIDGET_DEMO_BASELABEL)
    #include "demo_baselabel.h"
    #define widget_example_init   demo_baselabel_init
#elif defined(LV_WIDGET_DEMO_FOLLOW)
    #include "demo_follow.h"
    #define widget_example_init   demo_follow_init
#elif defined(LV_WIDGET_DEMO_IMGARRAY)
    #include "demo_imgarray.h"
    #define widget_example_init   demo_imgarray_init
#elif defined(LV_WIDGET_DEMO_IMGBAR)
    #include "demo_imgbar.h"
    #define widget_example_init   demo_imgbar_init
#elif defined(LV_WIDGET_DEMO_MULROLLER)
    #include "demo_mulroller.h"
    #define widget_example_init   demo_mulroller_init
#elif defined(LV_WIDGET_DEMO_MULTANIM)
    #include "lv_example_multanim.h"
    #define widget_example_init   lv_example_multanim
#elif defined(LV_WIDGET_DEMO_MULTLIST)
    /* multlist is built on the gui_app_fwk application framework */
    #include "gui_app_fwk.h"
    #include "demo_multlist.h"
#elif defined(LV_WIDGET_DEMO_MULTROLLER)
    #include "demo_multroller.h"
    #define widget_example_init   demo_multroller_init
#elif defined(LV_WIDGET_DEMO_MULTSLIDER)
    #include "demo_multslider.h"
    #define widget_example_init   demo_multslider_init
#elif defined(LV_WIDGET_DEMO_SECTOR)
    #include "demo_sector.h"
    #define widget_example_init   demo_sector_init
#elif defined(LV_WIDGET_DEMO_SELECT)
    #include "demo_select.h"
    #define widget_example_init   demo_select_init
#elif defined(LV_WIDGET_DEMO_TIMELINE)
    #include "demo_timeline.h"
    #define widget_example_init   demo_timeline_init
#else
    #error "Please select one widget example in menuconfig (LVSF v8 widget examples)"
#endif

/**
  * @brief  Main program: init LVGL, start the selected widget demo, run the loop.
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

#if defined(LV_WIDGET_DEMO_MULTLIST)
    /* gui_app_fwk based demo: register apps and run the main page */
    gui_app_init(1);
    gui_app_run(DEMO_MULTLIST_MAIN_ID);
#else
    widget_example_init();
#endif

    while (1)
    {
        rt_uint32_t ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;
}
