#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "lvgl.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "events_init.h"
#include "generated/gui_guider.h"


lv_ui guider_ui;

int main(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    /* init littlevGL */
    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }
    lv_ex_data_pool_init();

    rt_kprintf("EEZ Studio LVGL Image Example\n");
    setup_ui(&guider_ui);
    events_init(&guider_ui);
    /* Infinite loop */
    while (1)
    {
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return 0;
}

