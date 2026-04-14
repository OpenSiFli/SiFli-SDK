/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lvsf.h"
#include "app_clock_main.h"
#include "app_mem.h"
#include "log.h"


/* Image decalration */
LV_IMG_DECLARE(apng_dice);

typedef struct
{
    lv_obj_t *apng;
} app_clock_apng_t;

static app_clock_apng_t *p_clk_apng = NULL;

static rt_int32_t resume_callback(void)
{
    /* Resume apng animation refresh */
    lv_ezipa_resume(p_clk_apng->apng);
    return RT_EOK;
}

static rt_int32_t pause_callback(void)
{
    /* Pause apng animation refresh */
    lv_ezipa_pause(p_clk_apng->apng);
    return RT_EOK;
}

static rt_int32_t init(lv_obj_t *parent)
{
    p_clk_apng = (app_clock_apng_t *)rt_malloc(sizeof(app_clock_apng_t));
    RT_ASSERT(p_clk_apng);

    lv_obj_t *dice_img = lv_ezipa_create(parent);
    RT_ASSERT(dice_img);

    lv_ezipa_set_src(dice_img, apng_dice.data);
    // lv_ezipa_set_surface(dice_img, xxx);
    lv_ezipa_set_interval(dice_img, 60);
    lv_obj_set_pos(dice_img, lv_disp_get_hor_res(NULL) / 2, lv_disp_get_ver_res(NULL) / 2);
    p_clk_apng->apng = dice_img;

    return RT_EOK;
}

static rt_int32_t deinit(void)
{
    rt_free(p_clk_apng);
    p_clk_apng = NULL;

    return RT_EOK;
}

static const app_clock_ops_t ops =
{
    .init = init,
    .pause = pause_callback,
    .resume = resume_callback,
    .deinit = deinit,
};

void app_clock_apng_register(void)
{
    app_clock_register("apng", &ops);
}
