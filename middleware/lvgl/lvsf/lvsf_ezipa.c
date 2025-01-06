/**
 * @file lvsf_ezipa.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf_ezipa.h"

#ifdef USING_EZIPA_DEC
#include "app_mem.h"
#include "ezipa_dec.h"


/*********************
 *      DEFINES
 *********************/
#define LV_OBJX_NAME "lvsf_ezipa"
#define MY_CLASS &lv_lvsfezipa_class

#if(16 == LV_COLOR_DEPTH)
    #define EZIPA_OUTPUT_CF EZIPA_RGB565
    #define CANVAS_CF      EPIC_OUTPUT_RGB565
#elif (24 == LV_COLOR_DEPTH)
    #define EZIPA_OUTPUT_CF EZIPA_RGB888
    #define CANVAS_CF      EPIC_OUTPUT_RGB888

#else
    #error "Unsupport format!"
#endif

/**********************
 *      TYPEDEFS
 **********************/

typedef enum
{
    LV_EZIPA_STOP,
    LV_EZIPA_CURR,
    LV_EZIPA_NEXT,
} lv_ezipa_status_t;

typedef struct
{
    lv_obj_t img;
    lv_anim_t anim;
    ezipa_obj_t *ezipa_dec;
    lv_ezipa_status_t status;
    /** interval in millisecond, if the value is greater than 0,
        use this value rather than interval extracted from apng data */
    int32_t interval;
} lvsf_ezipa_ext_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ezipa_start_timer(lv_obj_t *ezipa);
static void ezipa_stop_timer(lv_obj_t *ezipa);
static void ezipa_timer_cb(struct _lv_anim_t *a);
static void ezipa_event(const lv_obj_class_t *class_p, lv_event_t *e);


/**********************
 *  STATIC VARIABLES
 **********************/
static void lv_lvsfezipa_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfezipa_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);

static const lv_obj_class_t lv_lvsfezipa_class =
{
    .constructor_cb = lv_lvsfezipa_constructor,
    .destructor_cb = lv_lvsfezipa_destructor,
    .event_cb = ezipa_event,
    .instance_size = sizeof(lvsf_ezipa_ext_t),
    .base_class = &lv_obj_class
};

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

static void lv_lvsfezipa_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{

    LV_UNUSED(class_p);
}

static void lv_lvsfezipa_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    lvsf_ezipa_ext_t *ext = (lvsf_ezipa_ext_t *)obj;

    lv_anim_del(obj, NULL);

    if (ext->ezipa_dec)
    {
        ezipa_close(ext->ezipa_dec);
        ext->ezipa_dec = NULL;
    }

}

extern int32_t gpu_ezipa_draw(ezipa_obj_t *obj, const lv_area_t *src_area, const lv_area_t *dst_area, bool next);


lv_obj_t *lv_lvsfezipa_create(lv_obj_t *parent)
{
    lvsf_ezipa_ext_t *obj = (lvsf_ezipa_ext_t *) lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj((lv_obj_t *)obj);
    lv_anim_init(&obj->anim);
    lv_anim_set_var(&obj->anim, obj);
    lv_anim_set_ready_cb(&obj->anim, ezipa_timer_cb);
    return (lv_obj_t *)obj;
}

void lv_lvsfezipa_set_src(lv_obj_t *ezipa, const char *src_ezipa)
{
    lvsf_ezipa_ext_t *ext = (lvsf_ezipa_ext_t *)ezipa;

    ext->ezipa_dec = ezipa_open((const uint8_t *)src_ezipa, EZIPA_OUTPUT_CF);
    LV_ASSERT_NULL(ext->ezipa_dec);

    lv_obj_set_size(ezipa, (lv_coord_t)ext->ezipa_dec->header.width,
                    (lv_coord_t)ext->ezipa_dec->header.height);

    ext->status = LV_EZIPA_STOP;
}

void lv_lvsfezipa_play(lv_obj_t *ezipa)
{
    ezipa_start_timer(ezipa);
}

void lv_lvsfezipa_stop(lv_obj_t *ezipa)
{
    ezipa_stop_timer(ezipa);
}

void lv_lvsfezipa_set_interval(lv_obj_t *ezipa, int32_t interval)
{
    lvsf_ezipa_ext_t *ext = (lvsf_ezipa_ext_t *)ezipa;

    ext->interval = interval;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static uint32_t get_next_delay_ms(ezipa_obj_t *ezipa_dec)
{
    if (0 == ezipa_dec->next_frame.delay_den)
    {
        ezipa_dec->next_frame.delay_den = 100;
    }
    return ezipa_dec->next_frame.delay_num * 1000 / ezipa_dec->next_frame.delay_den;
}


static void ezipa_timer_cb(struct _lv_anim_t *a)
{
    lv_obj_t *ezipa = (lv_obj_t *) a->var;
    lvsf_ezipa_ext_t *ext = (lvsf_ezipa_ext_t *)ezipa;

    ext->status = LV_EZIPA_NEXT;

    lv_obj_invalidate(ezipa);

}

static void ezipa_start_timer(lv_obj_t *ezipa)
{
    lvsf_ezipa_ext_t *ext = (lvsf_ezipa_ext_t *)ezipa;

    if (ext->interval > 0)
    {
        lv_anim_set_time(&ext->anim, ext->interval);
    }
    else
    {
        lv_anim_set_time(&ext->anim, get_next_delay_ms(ext->ezipa_dec));
    }

    lv_anim_start(&ext->anim);
}

static void ezipa_stop_timer(lv_obj_t *ezipa)
{
    lvsf_ezipa_ext_t *ext = (lvsf_ezipa_ext_t *)ezipa;

    lv_anim_del(ezipa, NULL);
}

static void ezipa_event(const lv_obj_class_t *class_p, lv_event_t *e)
{
    LV_UNUSED(class_p);

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    /*Call the ancestor's event handler*/
    lv_res_t res = lv_obj_event_base(MY_CLASS, e);
    if (res != LV_RES_OK)
    {
        return;
    }

    if (code == LV_EVENT_COVER_CHECK)
    {
        lv_cover_check_info_t *info;

        info = lv_event_get_param(e);
        info->res = LV_COVER_RES_COVER;
    }
    else if (code == LV_EVENT_DRAW_MAIN)
    {
        int32_t r;
#ifndef DISABLE_LVGL_V8
        lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
        const lv_area_t *clip_area = draw_ctx->clip_area;
#else
        lv_layer_t *layer = lv_event_get_layer(e);
        const lv_area_t *clip_area = &layer->_clip_area;
#endif /* DISABLE_LVGL_V8 */

        lvsf_ezipa_ext_t *ext = (lvsf_ezipa_ext_t *)obj;

        if (LV_EZIPA_STOP != ext->status)
        {
            r = gpu_ezipa_draw(ext->ezipa_dec, &obj->coords, clip_area, (LV_EZIPA_NEXT == ext->status));

            if (LV_EZIPA_NEXT == ext->status)
            {
                /*Start next frame update timer*/
                ezipa_start_timer(obj);
            }
            ext->status = LV_EZIPA_CURR;
        }
    }
    else
    {
        /* Do nothing */
    }
}


#else
lv_obj_t *lv_lvsfezipa_create(lv_obj_t *parent)
{
    return NULL;
}
void lv_lvsfezipa_set_src(lv_obj_t *ezipa, const char *src_ezipa)
{

}

void lv_lvsfezipa_play(lv_obj_t *ezipa)
{

}

void lv_lvsfezipa_stop(lv_obj_t *ezipa)
{

}
#endif

