#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"

#include "app_clock_main.h"
#include "app_mem.h"

#define  CACHE_ROTATE_BG  1//memory insufficient, rotate bg size 340*340, 346KiB with alpha  , 231KiB no alpha 


typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *minute_hand;
    lv_obj_t *hour_hand;
    lv_timer_t *redraw_task;
    app_clock_time_t last_redraw_time;
#if CACHE_ROTATE_BG
    lv_img_dsc_t *bg_cache;
#endif

} app_clock_rotate_bg_t;



#if (IMAGE_CACHE_IN_SRAM_SIZE >= 258*1000)
    #define ROTATE_BG_IMG_VARIABLE clock_rotate_bg_bg   //Image size 257766 bytes, 359x359
#elif (IMAGE_CACHE_IN_SRAM_SIZE >= 81*1000)
    #define ROTATE_BG_IMG_VARIABLE clock_rotate_bg_bg_small //Image size 80000 bytes, 200x200
#else
    #define ROTATE_BG_IMG_VARIABLE clock_rotate_bg_bg_tiny //Image size 45000 bytes, 150x150
#endif

LV_IMG_DECLARE(ROTATE_BG_IMG_VARIABLE);
LV_IMG_DECLARE(clock_rotate_bg_minute_hand);
LV_IMG_DECLARE(clock_rotate_bg_hour_hand);


static app_clock_rotate_bg_t *p_clk_rotate_bg = NULL;

static void app_clock_rotate_bg_redraw(lv_timer_t *task)
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


    //rt_kprintf("app_clock_rotate_bg_redraw %d:%d:%d\n",current_time.h,current_time.m,current_time.s);

    {
        static int16_t bg_angle = 0;
        static int32_t scale_percent = 2;
        static uint32_t scale = 256;

        const uint32_t scale_down_min = LV_IMG_ZOOM_NONE / 10;

        const uint32_t scale_up_max = LV_IMG_ZOOM_NONE * 20;

        if ((scale > scale_up_max) || (scale < scale_down_min))
        {
            scale_percent = -scale_percent;
        }


        scale = LV_MIN(scale_up_max, scale);
        scale = LV_MAX(scale_down_min, scale);

        int16_t l = (ROTATE_BG_IMG_VARIABLE.header.w * 256 / scale) * 3 /* PI */;

        int16_t d_angle = 3600 / l;


        bg_angle = (bg_angle + d_angle) % 3600;

        //rt_kprintf("app_clock_rotate_bg scale= %d, %d%%\n",scale,scale_percent);
        lv_img_set_angle(p_clk_rotate_bg->bg, bg_angle);
        lv_img_set_zoom(p_clk_rotate_bg->bg, scale);

        scale = (100 + scale_percent) * ((int32_t)scale) / 100;

        scale += (scale_percent > 0) ? 1 : -1; //In case round down

    }

    lv_img_set_angle(p_clk_rotate_bg->minute_hand, minute_angle);
    lv_img_set_angle(p_clk_rotate_bg->hour_hand, hour_angle);


    memcpy(&p_clk_rotate_bg->last_redraw_time, &current_time, sizeof(app_clock_time_t));
}

static rt_int32_t resume_callback(void)
{
    if (NULL == p_clk_rotate_bg->redraw_task)
    {
        p_clk_rotate_bg->redraw_task = lv_timer_create(app_clock_rotate_bg_redraw, 30, (void *)0);
    }

#if CACHE_ROTATE_BG
    if (NULL == p_clk_rotate_bg->bg_cache)
    {
        //cache rotate_bg
        p_clk_rotate_bg->bg_cache = app_cache_copy_alloc(LV_EXT_IMG_GET(ROTATE_BG_IMG_VARIABLE), ROTATE_MEM);

        lv_img_set_src(p_clk_rotate_bg->bg, p_clk_rotate_bg->bg_cache);
        lv_obj_align(p_clk_rotate_bg->bg, LV_ALIGN_CENTER, 0, 0);
    }
#endif

    return RT_EOK;

}

static rt_int32_t pause_callback(void)
{
    if (p_clk_rotate_bg->redraw_task)
    {
        lv_timer_del(p_clk_rotate_bg->redraw_task);
        p_clk_rotate_bg->redraw_task = NULL;
    }

#if CACHE_ROTATE_BG
    if (p_clk_rotate_bg->bg_cache != NULL)
    {
        lv_img_set_src(p_clk_rotate_bg->bg, LV_EXT_IMG_GET(ROTATE_BG_IMG_VARIABLE));
        lv_img_set_angle(p_clk_rotate_bg->bg, 0);
        lv_img_set_zoom(p_clk_rotate_bg->bg, LV_IMG_ZOOM_NONE);
        app_cache_copy_free(p_clk_rotate_bg->bg_cache);
        p_clk_rotate_bg->bg_cache = NULL;
    }
#endif

    return RT_EOK;
}

static rt_int32_t init(lv_obj_t *parent)
{
    p_clk_rotate_bg = (app_clock_rotate_bg_t *) rt_malloc(sizeof(app_clock_rotate_bg_t));
    memset(p_clk_rotate_bg, 0, sizeof(app_clock_rotate_bg_t));

    p_clk_rotate_bg->bg     = lv_img_create(parent);


    lv_img_set_src(p_clk_rotate_bg->bg, LV_EXT_IMG_GET(ROTATE_BG_IMG_VARIABLE));
    lv_obj_align(p_clk_rotate_bg->bg, LV_ALIGN_CENTER, 0, 0);
    uint32_t img_h = lv_obj_get_self_height(p_clk_rotate_bg->bg);
    uint32_t img_w = lv_obj_get_self_width(p_clk_rotate_bg->bg);

    rt_kprintf("app_clock_rotate_bg img_size:h:%d,w:%d \n", img_h, img_w);

    p_clk_rotate_bg->hour_hand   = lv_img_create(parent);
    lv_img_set_src(p_clk_rotate_bg->hour_hand, LV_EXT_IMG_GET(clock_rotate_bg_hour_hand));
    lv_obj_align(p_clk_rotate_bg->hour_hand, LV_ALIGN_TOP_MID, 0, 0);

    p_clk_rotate_bg->minute_hand   = lv_img_create(parent);
    lv_img_set_src(p_clk_rotate_bg->minute_hand, LV_EXT_IMG_GET(clock_rotate_bg_minute_hand));
    lv_obj_align(p_clk_rotate_bg->minute_hand, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_add_flag(p_clk_rotate_bg->minute_hand, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_clk_rotate_bg->hour_hand, LV_OBJ_FLAG_HIDDEN);

    //lv_obj_set_snapshot(p_clk_rotate_bg->bg, true);
    p_clk_rotate_bg->redraw_task = NULL;

    return RT_EOK;
}


static rt_int32_t deinit(void)
{
    if (p_clk_rotate_bg)
    {
        if (p_clk_rotate_bg->redraw_task)
        {
            lv_timer_del(p_clk_rotate_bg->redraw_task);
            p_clk_rotate_bg->redraw_task = NULL;
        }
#if CACHE_ROTATE_BG

        if (p_clk_rotate_bg->bg_cache != NULL)
        {
            app_cache_copy_free(p_clk_rotate_bg->bg_cache);
            p_clk_rotate_bg->bg_cache = NULL;
        }
#endif
        rt_free(p_clk_rotate_bg);
        p_clk_rotate_bg = NULL;
    }

    return RT_EOK;
}



static const app_clock_ops_t ops =
{
    .init   = init,
    .pause  = pause_callback,
    .resume = resume_callback,
    .deinit = deinit,
};

void app_clock_rotate_bg_register(void)
{
    app_clock_register("rotate_bg", &ops);
}


