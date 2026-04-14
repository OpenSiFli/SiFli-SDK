/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*********************
 *      INCLUDES
 *********************/
#include "littlevgl2rtt.h"
#include "lv_ext_resource_manager.h"
#include <rtdevice.h>
#ifndef _WIN32
    #include "drv_lcd.h"
#endif
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ex_data.h"
#include "app_mem.h"
#include "log.h"
#include "lvsf/lv_freetype.h"



#define APP_WATCH_GUI_TASK_STACK_SIZE 16*1024

#define LCD_DEVICE_NAME  "lcd"
#define IDLE_TIME_LIMIT  (10000)

static struct rt_thread watch_thread;

ALIGN(RT_ALIGN_SIZE)
static uint8_t watch_thread_stack[APP_WATCH_GUI_TASK_STACK_SIZE];
static rt_device_t lcd_device;

/*Compatible with private lib*/
uint32_t g_mainmenu[2];

extern void ui_datac_init(void);

void app_watch_entry(void *parameter)
{
    uint8_t first_loop = 1;
#ifdef _MSC_VER
    {
        extern int wait_platform_init_done(void);
        wait_platform_init_done();
    }
#else
    {
        set_date(2022, 7, 1);
        set_time(9, 0, 0);
    }
#endif /* _MSC_VER */

    lcd_device = rt_device_find(LCD_DEVICE_NAME);

    /* init littlevGL */
    {
        rt_err_t r = littlevgl2rtt_init(LCD_DEVICE_NAME);
        RT_ASSERT(RT_EOK == r);
    }

    lv_ex_data_pool_init();
    resource_init();
#if LV_USING_FREETYPE_ENGINE
    lv_freetype_open_font(true);                                /* open freetype */
#endif
    gui_app_init();

    gui_app_run("clock");
    lv_disp_trig_activity(NULL);
#if defined(GUI_APP_FRAMEWORK)&&(!defined (APP_TRANS_ANIMATION_NONE))
    lvsf_gesture_init(lv_layer_top());
#endif /* defined(GUI_APP_FRAMEWORK)&&(!defined (APP_TRANS_ANIMATION_NONE)) */

    while (1)
    {
        int ms;

        // rt_pm_request(PM_SLEEP_MODE_IDLE);
        ms = lv_timer_handler();
        // rt_pm_release(PM_SLEEP_MODE_IDLE);
        {
            //EventStartB(0);
            if (ms > 0)
                rt_thread_mdelay(ms);       /* Just to let the system breathe */
            //EventStopB(0);
        }

        if (first_loop)
        {
#ifndef WIN32
            //Turn on lcd backlight after power on
            uint8_t brightness = 100;
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &brightness);
#endif /* WIN32 */
            first_loop = 0;
        }
    }

}

void app_register(void)
{
}

int app_watch_init(void)
{
    rt_err_t ret = RT_EOK;
    rt_thread_t thread = RT_NULL;

    ret = rt_thread_init(&watch_thread, "app_watch", app_watch_entry, RT_NULL, watch_thread_stack, APP_WATCH_GUI_TASK_STACK_SIZE,
                         RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);

    if (RT_EOK != ret)
    {
        return RT_ERROR;
    }
    rt_thread_startup(&watch_thread);
    return RT_EOK;
}

INIT_APP_EXPORT(app_watch_init);
