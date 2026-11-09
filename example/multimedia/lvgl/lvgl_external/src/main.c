/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "board.h"
#include "lvgl.h"
#include "lv_port.h"

#ifdef EXAMPLE_LVGL_BENCHMARK
#include "lv_demos.h"
#endif

static uint32_t tick_get_cb(void)
{
    return (uint32_t)rt_tick_get_millisecond();
}

#if LV_USE_LOG
static void lv_log_print(lv_log_level_t level, const char *buf)
{
    LV_UNUSED(level);

    /* The upstream EPIC adapter logs from its IRQ. LV_OS_NONE leaves that IRQ
     * unregistered with RT-Thread, so ulog would see thread context and take
     * its mutex recursively on top of the render thread's log; that trips
     * ulog's own assertion. A log line is never worth stopping the image for,
     * so output from an exception context is dropped. */
    if (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk)
    {
        return;
    }

    rt_kprintf("%s", buf);
}
#endif

static void ui_init(void)
{
#ifdef EXAMPLE_LVGL_BENCHMARK
    lv_demo_benchmark();
#else
    /* Fallback UI when the benchmark is disabled. */
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *box = lv_obj_create(scr);
    lv_obj_remove_style_all(box);
    lv_obj_set_size(box, 200, 120);
    lv_obj_center(box);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x00C853), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(box);
    lv_label_set_text(label, "LVGL external OK");
    lv_obj_center(label);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
#endif
}

int main(void)
{
    lv_init();
#if LV_USE_LOG
    lv_log_register_print_cb(lv_log_print);
#endif
    lv_tick_set_cb(tick_get_cb);

    lv_port_disp_init();
    lv_port_indev_init();

    rt_kprintf("LVGL %d.%d.%d\n", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR,
               LVGL_VERSION_PATCH);

    ui_init();

    while (1)
    {
        uint32_t delay_ms = lv_timer_handler();

        /* Keep input polling responsive even when LVGL has no active timer. */
        if (delay_ms > 20)
        {
            delay_ms = 20;
        }
        if (delay_ms == 0)
        {
            delay_ms = 1;
        }
        rt_thread_mdelay(delay_ms);
    }
}
