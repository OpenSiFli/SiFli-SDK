#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"

#include "app_clock_main.h"
#include "app_mem.h"


#define CACHE_CLOCK_HANDS 1
#ifndef SF32LB55X
    // TODO: Check for LVGL V9
    #ifndef DISABLE_LVGL_V8
        #define ENABLE_MASKED_IMAGE
    #endif
#endif /* SF32LB55X */
typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *second_hand;
    lv_obj_t *minute_hand;
    lv_obj_t *hour_hand;
    lv_timer_t *redraw_task;
    app_clock_time_t last_redraw_time;

#if CACHE_CLOCK_HANDS
    lv_img_dsc_t *sec_cache;
    lv_img_dsc_t *min_cache;
    lv_img_dsc_t *hor_cache;
#endif
#ifdef ENABLE_MASKED_IMAGE
    lv_obj_t *masked_img_obj;
#endif /* ENABLE_MASKED_IMAGE */
} app_clock_simple_t;

LV_IMG_DECLARE(clock_simple_bg);
LV_IMG_DECLARE(clock_simple_second_hand);
LV_IMG_DECLARE(clock_simple_minute_hand);
LV_IMG_DECLARE(clock_simple_hour_hand);


#ifdef ENABLE_MASKED_IMAGE
    #include "a8_mask_300x200.h"
#endif /* ENABLE_MASKED_IMAGE */




static app_clock_simple_t *p_clk_simple = NULL;

static void app_clock_simple_redraw(lv_timer_t *task)
{
    app_clock_time_t current_time;
    rt_uint8_t hours, minutes, seconds;
    rt_uint16_t second_angle, minute_angle, hour_angle, milli_seconds;

    app_clock_main_get_current_time(&current_time);
    hours = current_time.h;
    minutes = current_time.m;
    seconds = current_time.s;
    milli_seconds = current_time.ms;

    hour_angle = ((hours * 300) + (minutes * 5) + 10);
    minute_angle = ((minutes * 60) + (seconds) + 10);
    second_angle = (seconds * 60) + (milli_seconds * 360 * 10 / 60000) + 10;


    lv_img_set_angle(p_clk_simple->second_hand, second_angle);
    lv_img_set_angle(p_clk_simple->minute_hand, minute_angle);
    lv_img_set_angle(p_clk_simple->hour_hand, hour_angle);
#ifdef ENABLE_MASKED_IMAGE
    //lv_img_set_zoom(p_clk_simple->masked_img_obj, LV_IMG_ZOOM_NONE + (seconds * 3));
    lv_img_set_offset_x(p_clk_simple->masked_img_obj, -(((seconds * 1000 + milli_seconds) / 33) % LV_HOR_RES_MAX));
    lv_img_set_offset_y(p_clk_simple->masked_img_obj, -(((seconds * 1000 + milli_seconds) / 33) % LV_VER_RES_MAX));
#endif /* ENABLE_MASKED_IMAGE */
    memcpy(&p_clk_simple->last_redraw_time, &current_time, sizeof(app_clock_time_t));
}

static void init_clock_hands_img(void)
{
#if CACHE_CLOCK_HANDS
    if (NULL == p_clk_simple->hor_cache)
    {
        p_clk_simple->hor_cache = app_cache_copy_alloc(LV_EXT_IMG_GET(clock_simple_hour_hand), ROTATE_MEM);
        RT_ASSERT(p_clk_simple->hor_cache != NULL);
        lv_img_set_src(p_clk_simple->hour_hand, p_clk_simple->hor_cache);
    }


    if (NULL == p_clk_simple->min_cache)
    {
        p_clk_simple->min_cache = app_cache_copy_alloc(LV_EXT_IMG_GET(clock_simple_minute_hand), ROTATE_MEM);
        RT_ASSERT(p_clk_simple->min_cache != NULL);
        lv_img_set_src(p_clk_simple->minute_hand, p_clk_simple->min_cache);
    }


    if (NULL == p_clk_simple->sec_cache)
    {
        p_clk_simple->sec_cache = app_cache_copy_alloc(LV_EXT_IMG_GET(clock_simple_second_hand), ROTATE_MEM);
        RT_ASSERT(p_clk_simple->sec_cache != NULL);
        lv_img_set_src(p_clk_simple->second_hand, p_clk_simple->sec_cache);
    }

#endif
}

static void deinit_clock_hands_img(void)
{
#if CACHE_CLOCK_HANDS

    lv_img_set_src(p_clk_simple->hour_hand, LV_EXT_IMG_GET(clock_simple_hour_hand));
    lv_img_set_src(p_clk_simple->minute_hand, LV_EXT_IMG_GET(clock_simple_minute_hand));
    lv_img_set_src(p_clk_simple->second_hand, LV_EXT_IMG_GET(clock_simple_second_hand));
    //Not to rotate image while it's src on flash
    lv_img_set_angle(p_clk_simple->hour_hand, 0);
    lv_img_set_angle(p_clk_simple->minute_hand, 0);
    lv_img_set_angle(p_clk_simple->second_hand, 0);


    if (p_clk_simple->hor_cache != NULL)
    {
        app_cache_copy_free(p_clk_simple->hor_cache);
        p_clk_simple->hor_cache = NULL;
    }

    if (p_clk_simple->min_cache != NULL)
    {
        app_cache_copy_free(p_clk_simple->min_cache);
        p_clk_simple->min_cache = NULL;
    }

    if (p_clk_simple->sec_cache != NULL)
    {
        app_cache_copy_free(p_clk_simple->sec_cache);
        p_clk_simple->sec_cache = NULL;
    }


#endif
}



static rt_int32_t resume_callback(void)
{
    init_clock_hands_img();

    if (NULL == p_clk_simple->redraw_task)
    {
        p_clk_simple->redraw_task = lv_timer_create(app_clock_simple_redraw, 30, (void *)0);
    }


    return RT_EOK;

}

static rt_int32_t pause_callback(void)
{
    lv_timer_del(p_clk_simple->redraw_task);
    p_clk_simple->redraw_task = NULL;

    deinit_clock_hands_img();

    return RT_EOK;

}
#ifdef ENABLE_MASKED_IMAGE
static void add_mask_event_cb(lv_event_t *e)
{
    static lv_draw_mask_map_param_t m;
    static int16_t mask_id;

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    if (code == LV_EVENT_COVER_CHECK)
    {
        lv_event_set_cover_res(e, LV_COVER_RES_MASKED);
    }
    else if (code == LV_EVENT_DRAW_MAIN_BEGIN)
    {
        lv_area_t mask_coords;

        mask_coords.x1 = obj->coords.x1;
        mask_coords.y1 = obj->coords.y1;

        mask_coords.x2 = mask_coords.x1 + a8_mask_300x200.header.w - 1;
        mask_coords.y2 = mask_coords.y1 + a8_mask_300x200.header.h - 1;

        lv_draw_mask_map_init(&m, &mask_coords, a8_mask_300x200.data);
        mask_id = lv_draw_mask_add(&m, NULL);
    }
    else if (code == LV_EVENT_DRAW_MAIN_END)
    {
        lv_draw_mask_free_param(&m);
        lv_draw_mask_remove_id(mask_id);
    }
    else if (code == LV_EVENT_CLICKED)
    {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }

}

static void bg_img_event_cb(lv_event_t *e)
{
    static lv_draw_mask_map_param_t m;
    static int16_t mask_id;

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    if (code == LV_EVENT_CLICKED)
    {
        if (lv_obj_is_visible(p_clk_simple->masked_img_obj))
            lv_obj_add_flag(p_clk_simple->masked_img_obj, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(p_clk_simple->masked_img_obj, LV_OBJ_FLAG_HIDDEN);
    }
}

#endif /* ENABLE_MASKED_IMAGE */
static rt_int32_t init(lv_obj_t *parent)
{
    p_clk_simple = (app_clock_simple_t *) rt_malloc(sizeof(app_clock_simple_t));
    memset(p_clk_simple, 0, sizeof(app_clock_simple_t));

    p_clk_simple->bg     = lv_img_create(parent);
    lv_img_set_src(p_clk_simple->bg, LV_EXT_IMG_GET(clock_simple_bg));
    lv_obj_align(p_clk_simple->bg, LV_ALIGN_CENTER, 0, 0);

#ifdef ENABLE_MASKED_IMAGE
    p_clk_simple->masked_img_obj   = lv_img_create(p_clk_simple->bg);
    lv_img_set_src(p_clk_simple->masked_img_obj, LV_EXT_IMG_GET(clock_simple_bg));
    lv_img_set_size_mode(p_clk_simple->masked_img_obj, LV_IMG_SIZE_MODE_REAL);
    //Set same size as mask
    lv_obj_set_size(p_clk_simple->masked_img_obj, a8_mask_300x200.header.w, a8_mask_300x200.header.h);
    lv_obj_clear_flag(p_clk_simple->masked_img_obj, LV_OBJ_FLAG_SCROLLABLE);

    lv_img_set_pivot(p_clk_simple->masked_img_obj,
                     a8_mask_300x200.header.w / 2,
                     a8_mask_300x200.header.h / 2);

    lv_obj_align(p_clk_simple->masked_img_obj, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_add_event_cb(p_clk_simple->masked_img_obj, add_mask_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(p_clk_simple->masked_img_obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p_clk_simple->bg, bg_img_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(p_clk_simple->bg, LV_OBJ_FLAG_CLICKABLE);


#endif /* ENABLE_MASKED_IMAGE */

    //lv_obj_set_snapshot(p_clk_simple->bg, true);
    p_clk_simple->hour_hand   = lv_img_create(p_clk_simple->bg);
    p_clk_simple->minute_hand   = lv_img_create(p_clk_simple->bg);
    p_clk_simple->second_hand   = lv_img_create(parent);

    lv_img_set_src(p_clk_simple->hour_hand, LV_EXT_IMG_GET(clock_simple_hour_hand));
    lv_img_set_src(p_clk_simple->minute_hand, LV_EXT_IMG_GET(clock_simple_minute_hand));
    lv_img_set_src(p_clk_simple->second_hand, LV_EXT_IMG_GET(clock_simple_second_hand));

    lv_obj_align(p_clk_simple->hour_hand, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(p_clk_simple->minute_hand, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(p_clk_simple->second_hand, LV_ALIGN_CENTER, 0, 0);
    p_clk_simple->redraw_task = NULL;

    return RT_EOK;
}


static rt_int32_t deinit(void)
{
    if (p_clk_simple)
    {
        if (p_clk_simple->redraw_task)
        {
            lv_timer_del(p_clk_simple->redraw_task);
        }
        rt_free(p_clk_simple);
        p_clk_simple = NULL;
    }

    return RT_EOK;
}




static const app_clock_ops_t ops =
{
    .init = init,
    .pause = pause_callback,
    .resume = resume_callback,
    .deinit = deinit,
};

void app_clock_simple_register(void)
{
    app_clock_register("simple", &ops);
}

