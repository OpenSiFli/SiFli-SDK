#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"
#include "lv_ext_resource_manager.h"

//TODO
#if (LVSF_USE_CURVE != 0) && defined(LV_USING_EXT_RESOURCE_MANAGER)
#include "app_mem.h"

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_lvsfcurve_class

/**********************
 *      TYPEDEFS
 **********************/
/*struct of curve*/
typedef struct
{
    lv_canvas_t canvas;
    lv_coord_t pivot_x;
    lv_coord_t pivot_y;
    void *buf;
} lv_lvsfcurve_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_lvsfcurve_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfcurve_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);

/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_lvsfcurve_class =
{
    .constructor_cb = lv_lvsfcurve_constructor,
    .destructor_cb = lv_lvsfcurve_destructor,
    .instance_size = sizeof(lv_lvsfcurve_t),
    .base_class = &lv_canvas_class
};


/**********************
 *      MACROS
 **********************/


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_lvsfcurve_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

    lv_obj_set_style_bg_opa(obj, LV_OPA_0, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void lv_lvsfcurve_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    lv_lvsfcurve_t *ext = (lv_lvsfcurve_t *)obj;

    LV_UNUSED(class_p);
    if (ext && ext->buf)
        app_canvas_mem_free(ext->buf);
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_obj_t *lv_lvsfcurve_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

void lv_lvsfcurve_set_buf(lv_obj_t *curve, uint16_t txt_width, uint16_t txt_height)
{
    lv_lvsfcurve_t *ext = (lv_lvsfcurve_t *)curve;
    void *buff = (void *)app_canvas_mem_alloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(txt_width, txt_height));

    RT_ASSERT(buff);
    memset(buff, 0x00, LV_CANVAS_BUF_SIZE_TRUE_COLOR(txt_width, txt_height));
    RT_ASSERT(!ext->buf);
    lv_canvas_set_buffer(curve, buff, txt_width, txt_height, LV_IMG_CF_TRUE_COLOR);
    ext->buf = buff;
}

void lv_lvsfcurve_set_pivot(lv_obj_t *curve, lv_coord_t x, lv_coord_t y)
{
    lv_lvsfcurve_t *ext = (lv_lvsfcurve_t *)curve;

    ext->pivot_x = x;
    ext->pivot_y = y;
}

void lv_lvsfcurve_draw_arc(lv_obj_t *curve,          lv_coord_t r, int32_t start_angle, int32_t end_angle, lv_color_t color, lv_coord_t width)
{
    lv_lvsfcurve_t *ext = (lv_lvsfcurve_t *)curve;
    lv_draw_arc_dsc_t style;

    memset(&style, 0, sizeof(style));
    style.color = color;
    style.width = width;
#ifdef DISABLE_LVGL_V9
    lv_canvas_draw_arc(curve, ext->pivot_x, ext->pivot_y, r, start_angle, end_angle, &style);
#endif
}

void lv_lvsfcurve_text(lv_obj_t *curve, char *text, int angle, int r, lv_color_t color, int size)
{
    lv_lvsfcurve_t *ext = (lv_lvsfcurve_t *)curve;

    lvsf_curve_text(curve, text, ext->pivot_x, ext->pivot_y, angle, r, color, size);
}


#endif
