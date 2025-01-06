/**
 * @file lvsf_rlottie.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"

#include "lvgl.h"
#include "lvsf.h"

#if LVSF_USE_ANALOGCLK!=0

#include "time.h"
#if defined(ROTATE_MEM_IN_PSRAM)|| defined(ROTATE_MEM_IN_SRAM)
    #include "app_mem.h"
#endif

#ifdef WIN32
    extern time_t time(time_t *t);
    struct tm *localtime(const time_t *t);
#endif
/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_analogclk_class

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
    lv_obj_t obj_ext;
    lv_obj_t *clock_bg;
    lv_obj_t *hour_hand;
    lv_obj_t *minute_hand;
    lv_obj_t *second_hand;
    lv_timer_t *clock_redraw_task;
    uint8_t hoff;
    uint8_t moff;
    uint8_t soff;
    uint16_t refresh_interval;
} lv_analogclk_ext_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void refresh_task_cb(lv_timer_t *t);
static void lv_analogclk_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_analogclk_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);


/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_analogclk_class =
{
    .constructor_cb = lv_analogclk_constructor,
    .destructor_cb = lv_analogclk_destructor,
    .instance_size = sizeof(lv_analogclk_ext_t),
    .base_class = &lv_obj_class
};

/**********************
 *      MACROS
 **********************/


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_analogclk_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

}

static void lv_analogclk_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)obj;

    if (ext->clock_redraw_task)
        lv_timer_del(ext->clock_redraw_task);
#if 0
//#ifdef CACHE_CLOCK_HANDS_SIMPLE
    if (ext->hor_cache != NULL)
    {
        app_cache_copy_free(ext->hor_cache);
        ext->hor_cache = NULL;
    }

    if (ext->min_cache != NULL)
    {
        app_cache_copy_free(ext->min_cache);
        ext->min_cache = NULL;
    }

    if (ext->sec_cache != NULL)
    {
        app_cache_copy_free(ext->sec_cache);
        ext->sec_cache = NULL;
    }
#endif
}

static void refresh_task_cb(lv_timer_t *t)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)t->user_data;

#ifdef WIN32
    time_t  cur = _time64(RT_NULL);
    struct tm *tmb = _localtime64(&cur);
#else
    time_t  cur = time(RT_NULL);
    struct tm *tmb = localtime(&cur);
#endif

    if (ext->second_hand)
    {
        static uint32_t ms_start;
        static int delta = 60;

        if (delta == 60)
        {
            ms_start = rt_tick_get_millisecond();
            delta = (rt_tick_get_millisecond() / 1000) % 60 - tmb->tm_sec;
        }

        int32_t ms_now = rt_tick_get_millisecond();
        uint16_t second_angle = ((ms_now / 1000 + delta) * 60) + ((ms_now % 1000) * 360 * 10 / 60000) + 10;
        lv_img_set_angle(ext->second_hand, second_angle);
    }
    if (ext->minute_hand)
    {
        uint16_t minute_angle = (tmb->tm_min * 60) + (tmb->tm_sec) + 10;
        lv_img_set_angle(ext->minute_hand, minute_angle);
    }
    if (ext->hour_hand)
    {
        uint16_t hour_angle = (tmb->tm_hour * 300) + (tmb->tm_min * 5) + 10;
        lv_img_set_angle(ext->hour_hand, hour_angle);
    }
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_obj_t *lv_analogclk_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    lv_obj_set_width(obj, lv_pct(100));
    lv_obj_set_height(obj, lv_pct(100));
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    return obj;
}

int lv_analogclk_img(lv_obj_t *aclk, const char *bg, const char *hour, const char *min, const char *second)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;
    lv_obj_t *parent = aclk;
    lv_img_t *img;

    if (bg)
    {
        if (ext->clock_bg == NULL)
            ext->clock_bg = lv_img_create(aclk);
        lv_img_set_src(ext->clock_bg, bg);
        lv_obj_align(ext->clock_bg, LV_ALIGN_CENTER, 0, 0);
        lv_obj_clear_flag(ext->clock_bg, LV_OBJ_FLAG_CLICKABLE);
    }
    if (hour)
    {
        if (ext->hour_hand == NULL)
            ext->hour_hand = lv_img_create(parent);
        img = (lv_img_t *)ext->hour_hand;
        lv_img_set_src(ext->hour_hand, hour);
        lv_obj_align(ext->hour_hand, LV_ALIGN_CENTER, 0, -(img->h >> 1) + ext->hoff);
        lv_img_set_pivot(ext->hour_hand, img->pivot.x, img->h - ext->hoff);
        lv_obj_clear_flag(ext->hour_hand, LV_OBJ_FLAG_CLICKABLE);
    }

    if (min)
    {
        if (ext->minute_hand == NULL)
            ext->minute_hand = lv_img_create(parent);
        img = (lv_img_t *)ext->minute_hand;
        lv_img_set_src(ext->minute_hand, min);
        lv_obj_align(ext->minute_hand,  LV_ALIGN_CENTER, 0, - (img->h >> 1) + ext->moff);
        lv_img_set_pivot(ext->minute_hand, img->pivot.x, img->h - ext->moff);
        lv_obj_clear_flag(ext->minute_hand, LV_OBJ_FLAG_CLICKABLE);
    }
    if (second)
    {
        if (ext->second_hand == NULL)
            ext->second_hand = lv_img_create(parent);
        img = (lv_img_t *)ext->second_hand;
        lv_img_set_src(ext->second_hand, second);
        lv_obj_align(ext->second_hand,  LV_ALIGN_CENTER, 0, - (img->h >> 1) + ext->soff);
        lv_img_set_pivot(ext->second_hand, img->pivot.x, img->h - ext->soff);
        lv_obj_clear_flag(ext->second_hand, LV_OBJ_FLAG_CLICKABLE);
    }

    return LV_RES_OK;
}

int lv_analogclk_set_bg(lv_obj_t *aclk, const char *bg)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;

    if (ext->clock_bg == NULL)
        ext->clock_bg = lv_img_create(aclk);
    lv_img_set_src(ext->clock_bg, bg);
    lv_obj_align(ext->clock_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(ext->clock_bg, LV_OBJ_FLAG_CLICKABLE);

    return LV_RES_OK;
}

int lv_analogclk_set_hour(lv_obj_t *aclk, const char *hour)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;
    lv_obj_t *parent = aclk;
    lv_img_t *img;

    if (ext->hour_hand == NULL)
        ext->hour_hand = lv_img_create(parent);
    img = (lv_img_t *)ext->hour_hand;
    lv_img_set_src(ext->hour_hand, hour);
    lv_obj_align(ext->hour_hand, LV_ALIGN_CENTER, 0, -(img->h >> 1) + ext->hoff);
    lv_img_set_pivot(ext->hour_hand, img->pivot.x, img->h - ext->hoff);
    lv_obj_clear_flag(ext->hour_hand, LV_OBJ_FLAG_CLICKABLE);


    return LV_RES_OK;
}

int lv_analogclk_set_min(lv_obj_t *aclk, const char *min)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;
    lv_obj_t *parent = aclk;
    lv_img_t *img;

    if (ext->minute_hand == NULL)
        ext->minute_hand = lv_img_create(parent);
    img = (lv_img_t *)ext->minute_hand;
    lv_img_set_src(ext->minute_hand, min);
    lv_obj_align(ext->minute_hand,  LV_ALIGN_CENTER, 0, - (img->h >> 1) + ext->moff);
    lv_img_set_pivot(ext->minute_hand, img->pivot.x, img->h - ext->moff);
    lv_obj_clear_flag(ext->minute_hand, LV_OBJ_FLAG_CLICKABLE);


    return LV_RES_OK;
}

int lv_analogclk_set_second(lv_obj_t *aclk, const char *second)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;
    lv_obj_t *parent = aclk;
    lv_img_t *img;

    if (ext->second_hand == NULL)
        ext->second_hand = lv_img_create(parent);
    img = (lv_img_t *)ext->second_hand;
    lv_img_set_src(ext->second_hand, second);
    lv_obj_align(ext->second_hand,  LV_ALIGN_CENTER, 0, - (img->h >> 1) + ext->soff);
    lv_img_set_pivot(ext->second_hand, img->pivot.x, img->h - ext->soff);
    lv_obj_clear_flag(ext->second_hand, LV_OBJ_FLAG_CLICKABLE);

    return LV_RES_OK;
}

void lv_analogclk_pos_off(lv_obj_t *aclk, uint8_t hoff, uint8_t moff, uint8_t soff)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;
    ext->hoff = hoff;
    ext->moff = moff;
    ext->soff = soff;
}

void lv_analogclk_set_hoff(lv_obj_t *aclk, uint8_t hoff)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;
    ext->hoff = hoff;
}

void lv_analogclk_set_moff(lv_obj_t *aclk, uint8_t moff)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;
    ext->hoff = moff;
}

void lv_analogclk_set_soff(lv_obj_t *aclk, uint8_t soff)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;
    ext->hoff = soff;
}

void lv_analogclk_refr_inteval(lv_obj_t *aclk, uint16_t ms)
{
    lv_analogclk_ext_t *ext = (lv_analogclk_ext_t *)aclk;

    if (ms != ext->refresh_interval)
    {
        if (ext->clock_redraw_task)
        {
            lv_timer_del(ext->clock_redraw_task);
            ext->clock_redraw_task = NULL;
        }
        if (ms)
            ext->clock_redraw_task = lv_timer_create(refresh_task_cb, ms, (void *)aclk);
        ext->refresh_interval = ms;
    }
}

#endif

