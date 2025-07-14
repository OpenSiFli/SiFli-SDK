#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"

#include "app_clock_main.h"
#include "app_mem.h"

#define CACHE_CLOCK_HANDS 1
typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *second_hand;
    lv_obj_t *minute_hand;
    lv_obj_t *hour_hand;
    lv_timer_t *redraw_task;
    app_clock_time_t last_redraw_time;

#if CACHE_CLOCK_HANDS
    lv_image_dsc_t *sec_cache;
    lv_image_dsc_t *min_cache;
    lv_image_dsc_t *hor_cache;
#endif
} app_clock_dial_t;

LV_IMG_DECLARE(img_dial_bg);
LV_IMG_DECLARE(clock_dial_point_s);
LV_IMG_DECLARE(clock_dial_point_m);
LV_IMG_DECLARE(clock_dial_point_h);


static app_clock_dial_t *p_clk_dial = NULL;

static void app_clock_dial_redraw(lv_timer_t *task)
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


    lv_img_set_angle(p_clk_dial->second_hand, second_angle);
    lv_img_set_angle(p_clk_dial->minute_hand, minute_angle);
    lv_img_set_angle(p_clk_dial->hour_hand, hour_angle);
    memcpy(&p_clk_dial->last_redraw_time, &current_time, sizeof(app_clock_time_t));
}

static void init_clock_hands_img(void)
{
#if CACHE_CLOCK_HANDS
    if (NULL == p_clk_dial->hor_cache)
    {
        p_clk_dial->hor_cache = app_cache_copy_alloc(LV_EXT_IMG_GET(clock_dial_point_h), ROTATE_MEM);
        RT_ASSERT(p_clk_dial->hor_cache != NULL);
        lv_image_set_src(p_clk_dial->hour_hand, p_clk_dial->hor_cache);
    }


    if (NULL == p_clk_dial->min_cache)
    {
        p_clk_dial->min_cache = app_cache_copy_alloc(LV_EXT_IMG_GET(clock_dial_point_m), ROTATE_MEM);
        RT_ASSERT(p_clk_dial->min_cache != NULL);
        lv_image_set_src(p_clk_dial->minute_hand, p_clk_dial->min_cache);
    }


    if (NULL == p_clk_dial->sec_cache)
    {
        p_clk_dial->sec_cache = app_cache_copy_alloc(LV_EXT_IMG_GET(clock_dial_point_s), ROTATE_MEM);
        RT_ASSERT(p_clk_dial->sec_cache != NULL);
        lv_image_set_src(p_clk_dial->second_hand, p_clk_dial->sec_cache);
    }

#endif

    lv_img_set_pivot(p_clk_dial->hour_hand, 17, 108);
    lv_img_set_pivot(p_clk_dial->minute_hand, 20, 140);
    lv_img_set_pivot(p_clk_dial->second_hand, 8, 160);

    lv_obj_align(p_clk_dial->hour_hand, LV_ALIGN_CENTER, 0, -55);
    lv_obj_align(p_clk_dial->minute_hand, LV_ALIGN_CENTER, 0, -65);
    lv_obj_align(p_clk_dial->second_hand, LV_ALIGN_CENTER, 0, -70);

}

static void deinit_clock_hands_img(void)
{
#if CACHE_CLOCK_HANDS

    lv_image_set_src(p_clk_dial->hour_hand, LV_EXT_IMG_GET(clock_dial_point_h));
    lv_image_set_src(p_clk_dial->minute_hand, LV_EXT_IMG_GET(clock_dial_point_m));
    lv_image_set_src(p_clk_dial->second_hand, LV_EXT_IMG_GET(clock_dial_point_s));
    //Not to rotate image while it's src on flash
    lv_img_set_angle(p_clk_dial->hour_hand, 0);
    lv_img_set_angle(p_clk_dial->minute_hand, 0);
    lv_img_set_angle(p_clk_dial->second_hand, 0);


    if (p_clk_dial->hor_cache != NULL)
    {
        app_cache_copy_free(p_clk_dial->hor_cache);
        p_clk_dial->hor_cache = NULL;
    }

    if (p_clk_dial->min_cache != NULL)
    {
        app_cache_copy_free(p_clk_dial->min_cache);
        p_clk_dial->min_cache = NULL;
    }

    if (p_clk_dial->sec_cache != NULL)
    {
        app_cache_copy_free(p_clk_dial->sec_cache);
        p_clk_dial->sec_cache = NULL;
    }


#endif
}
static rt_int32_t resume_callback(void)
{
    init_clock_hands_img();

    if (NULL == p_clk_dial->redraw_task)
    {
        p_clk_dial->redraw_task = lv_timer_create(app_clock_dial_redraw, 30, (void *)0);
    }


    return RT_EOK;

}

static rt_int32_t pause_callback(void)
{
    lv_timer_del(p_clk_dial->redraw_task);
    p_clk_dial->redraw_task = NULL;

    deinit_clock_hands_img();

    return RT_EOK;

}
static void lv_obj_img_png_set_zoom(lv_obj_t *obj_img, const char *src, uint32_t obj_width, uint32_t obj_height)
{
    uint32_t img_width = 0, img_height = 0, zoom_factor = 0;
    lv_img_header_t header;
    if (lv_img_decoder_get_info(src, &header) != LV_RES_OK)
    {
        rt_kprintf("lv_img_decoder_get_info error\n");
        return;
    }

    img_width = header.w;
    img_height = header.h;

    rt_kprintf("img_widtg:%u, img_height%u, obj_width:%u, obj_height%u\n", img_width, img_height, obj_width, obj_height);
    if (img_width != 0 && img_height != 0)
    {
        uint32_t y_a = obj_height;
        uint32_t x_b = obj_width;

        if (x_b >= y_a)
        {

            uint32_t x = obj_height * 256;
            zoom_factor = x / img_height;
            lv_img_set_zoom(obj_img, zoom_factor);

        }
        else
        {

            uint32_t x = obj_width * 256;
            zoom_factor = x / img_width;
            lv_img_set_zoom(obj_img, zoom_factor);

        }
    }
}
static rt_int32_t init(lv_obj_t *parent)
{
    p_clk_dial = (app_clock_dial_t *) rt_malloc(sizeof(app_clock_dial_t));
    memset(p_clk_dial, 0, sizeof(app_clock_dial_t));


    lv_obj_t *my_parent = lv_obj_create(parent);

    lv_obj_set_size(my_parent, lv_obj_get_width(parent), lv_obj_get_height(parent));
    lv_obj_clear_flag(my_parent, LV_OBJ_FLAG_SCROLLABLE);

    p_clk_dial->bg  = lv_image_create(my_parent);
    lv_image_set_src(p_clk_dial->bg, LV_EXT_IMG_GET(img_dial_bg));
    lv_obj_align(p_clk_dial->bg, LV_ALIGN_CENTER, 0, 0);

    //unsigned int obj_width = 410,obj_height = 494;
    //lv_obj_img_png_set_zoom(p_clk_dial->bg,LV_EXT_IMG_GET(img_dial_bg),obj_width,obj_height);

    lv_obj_img_png_set_zoom(p_clk_dial->bg, LV_EXT_IMG_GET(img_dial_bg), lv_obj_get_width(parent), lv_obj_get_height(parent));
    p_clk_dial->hour_hand   = lv_image_create(p_clk_dial->bg);
    p_clk_dial->minute_hand   = lv_image_create(p_clk_dial->bg);
    p_clk_dial->second_hand   = lv_image_create(p_clk_dial->bg);

    lv_image_set_src(p_clk_dial->hour_hand, LV_EXT_IMG_GET(clock_dial_point_h));
    lv_image_set_src(p_clk_dial->minute_hand, LV_EXT_IMG_GET(clock_dial_point_m));
    lv_image_set_src(p_clk_dial->second_hand, LV_EXT_IMG_GET(clock_dial_point_s));
    p_clk_dial->redraw_task = NULL;

    return RT_EOK;
}

static rt_int32_t deinit(void)
{
    if (p_clk_dial)
    {
        if (p_clk_dial->redraw_task)
        {
            lv_timer_del(p_clk_dial->redraw_task);
        }
        rt_free(p_clk_dial);
        p_clk_dial = NULL;
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

void app_clock_dial_register(void)
{
    app_clock_register("dial", &ops);
}