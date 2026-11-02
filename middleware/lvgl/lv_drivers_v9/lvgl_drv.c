/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "board.h"

extern void lv_sifli_img_decoder(void);
extern void lv_lcd_init(const char *name);
extern lv_indev_t *lv_touchscreen_create(const char *dev_path);
extern void keypad_init(void);
extern void wheel_init(void);

static uint32_t tick_count_callback(void)
{
    return (uint32_t)rt_tick_get_millisecond();
}

void lv_hal_init(const char *name)
{

    lv_tick_set_cb(tick_count_callback);

    /* LCD Device Init */
    lv_lcd_init(name);
    littlevgl2rtt_fps_init(lv_display_get_default());

    lv_sifli_img_decoder();

    /* littlevGL Input device interface */
    lv_touchscreen_create("touch");

    // Init keyboard driver
    keypad_init();

    //wheel_init();

}

