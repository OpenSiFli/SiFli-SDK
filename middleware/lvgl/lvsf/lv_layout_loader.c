/**
 * @file lv_layout_loader.c
 *
 *
 *
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvgl.h"
#include "widget_factory.h"
#include "lv_layout_loader.h"
//#include "app_clock_main.h"
#include "ui_loader_default.h"
#include "ui_builder_default.h"
#include "widget_factory.h"
#include "widget_consts.h"
#include "value.h"
#ifdef SOLUTION_WATCH
    #include "lvsf_wf.h"
    #include "lvsf_roundbar.h"
    #include "lvsf_details.h"
    #include "lvsf_weather.h"
#endif

/*********************
 *      DEFINES
 *********************/
SECTION_DEF(APP_WATCHFACE_LAYOUT_SECTION_NAME, ui_layout_desc_t);


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

static lv_color_t color_str_2_lv_color(const char *color_val)
{
    char color_str[3];
    uint8_t r, g, b;

    color_str[2] = 0;

    color_str[0] = color_val[0];
    color_str[1] = color_val[1];
    r = strtol(color_str, NULL, 16);

    color_str[0] = color_val[2];
    color_str[1] = color_val[3];
    g = strtol(color_str, NULL, 16);

    color_str[0] = color_val[4];
    color_str[1] = color_val[5];
    b = strtol(color_str, NULL, 16);

    return lv_color_make(r, g, b);
}



static const ui_layout_desc_t *find_watchface_layout(const char *id)
{
    ui_layout_desc_t *layout_desc;
    uint32_t *end;
    uint32_t *temp;

    end = (uint32_t *)SECTION_END_ADDR(APP_WATCHFACE_LAYOUT_SECTION_NAME);;
    temp = (uint32_t *)SECTION_START_ADDR(APP_WATCHFACE_LAYOUT_SECTION_NAME);;
    while (temp < end)
    {
        layout_desc = (ui_layout_desc_t *)temp;
        if (layout_desc->id_str
                && layout_desc->data)
        {
            if (0 == rt_strcmp(id, layout_desc->id_str))
            {
                break;
            }
            temp += (sizeof(ui_layout_desc_t) >> 2);
        }
        else
        {
            temp++;
        }
    }
    if (temp >= end)
    {
        layout_desc = NULL;
    }
    return layout_desc;
}



static lv_obj_t *load_layout(lv_obj_t *parent, const ui_layout_desc_t *layout)
{
    ui_loader_t *loader = default_ui_loader();
    ui_builder_t *builder = ui_builder_default(layout->id_str);

    builder->widget = parent;

    ui_loader_load(loader, layout->data, layout->data_size, builder);

    return builder->root;
}


static lv_obj_t *screen_obj;
static rt_int32_t on_init(lv_obj_t *parent)
{
    const char *watchface_id;
    const ui_layout_desc_t *layout;

    watchface_id = app_clock_change_context();
    layout = find_watchface_layout(watchface_id);
    if (!layout)
    {
        LOG_W("watchface layout %s not found", watchface_id);
        return RT_ERROR;
    }

    load_layout(parent, layout);
    screen_obj = parent;

    return RT_EOK;
}

static rt_int32_t on_pause(void)
{
    lv_obj_pause(screen_obj);

    return RT_EOK;
}

static rt_int32_t on_resume(void)
{
    lv_obj_resume(screen_obj);

    return RT_EOK;
}

static rt_int32_t on_deinit(void)
{
    screen_obj = NULL;
    return RT_EOK;
}

const app_clock_ops_t g_app_watchface_layout_ops =
{
    .init = on_init,
    .pause = on_pause,
    .resume = on_resume,
    .deinit = on_deinit,

};


static lv_res_t lv_analogclk_set_prop(lv_obj_t *obj, widget_prop_t prop_name, const value_t *val, const void *ui)
{
    lv_res_t res = LV_RES_OK;

#ifdef WIDGET_ENUM_TYPE_ENABLED
    if (prop_name == WIDGET_ANACLOCK_PROP_HOFF)
    {
        lv_analogclk_set_hoff(obj, value_uint8(val));
    }
    else if (prop_name == WIDGET_ANACLOCK_PROP_MOFF)
    {
        lv_analogclk_set_moff(obj, value_uint8(val));
    }
    else if (prop_name == WIDGET_ANACLOCK_PROP_SOFF)
    {
        lv_analogclk_set_soff(obj, value_uint8(val));
    }
    else if (prop_name == WIDGET_ANACLOCK_PROP_BG)
    {
        lv_analogclk_set_bg(obj, value_str(val));
    }
    else if (prop_name == WIDGET_ANACLOCK_PROP_HOUR)
    {
        lv_analogclk_set_hour(obj, value_str(val));
    }
    else if (prop_name == WIDGET_ANACLOCK_PROP_MIN)
    {
        lv_analogclk_set_min(obj, value_str(val));
    }
    else if (prop_name == WIDGET_ANACLOCK_PROP_SEC)
    {
        lv_analogclk_set_second(obj, value_str(val));
    }
    else if (prop_name == WIDGET_ANACLOCK_PROP_INTERVAL)
    {
        lv_analogclk_refr_inteval(obj, value_uint16(val));
    }
    else
    {
        res = LV_RES_INV;
    }
#else
    if (0 == rt_strcmp(prop_name, "hoff"))
    {
        lv_analogclk_set_hoff(obj, value_uint8(val));
    }
    else if (0 == rt_strcmp(prop_name, "moff"))
    {
        lv_analogclk_set_moff(obj, value_uint8(val));
    }
    else if (0 == rt_strcmp(prop_name, "soff"))
    {
        lv_analogclk_set_soff(obj, value_uint8(val));
    }
    else if (0 == rt_strcmp(prop_name, "bg"))
    {
        lv_analogclk_set_bg(obj, value_str(val));
    }
    else if (0 == rt_strcmp(prop_name, "hour"))
    {
        lv_analogclk_set_hour(obj, value_str(val));
    }
    else if (0 == rt_strcmp(prop_name, "min"))
    {
        lv_analogclk_set_min(obj, value_str(val));
    }
    else if (0 == rt_strcmp(prop_name, "second"))
    {
        lv_analogclk_set_second(obj, value_str(val));
    }
    else if (0 == rt_strcmp(prop_name, "refr_inteval"))
    {
        lv_analogclk_refr_inteval(obj, value_uint16(val));
    }
    else
    {
        res = LV_RES_INV;
    }
#endif


    return res;
}


static FONT_SIZES lv_get_font_size_from_name(const char *font_size)
{
    FONT_SIZES size;

    if (0 == rt_strcmp(font_size, "SMALL"))
    {
        size = FONT_SMALL;
    }
    else if (0 == rt_strcmp(font_size, "NORMAL"))
    {
        size = FONT_NORMAL;
    }
    else if (0 == rt_strcmp(font_size, "SUBTITLE"))
    {
        size = FONT_SUBTITLE;
    }
    else if (0 == rt_strcmp(font_size, "TITLE"))
    {
        size = FONT_TITLE;
    }
    else if (0 == rt_strcmp(font_size, "BIGL"))
    {
        size = FONT_BIGL;
    }
    else if (0 == rt_strcmp(font_size, "HUGE"))
    {
        size = FONT_HUGE;
    }
    else if (0 == rt_strcmp(font_size, "SUPER"))
    {
        size = FONT_SUPER;
    }
    else
    {
        size = FONT_NORMAL;
    }

    return size;
}


static lv_res_t lv_label_set_prop(lv_obj_t *obj, widget_prop_t prop_name, const value_t *val, const void *ui)
{
    lv_res_t res = LV_RES_OK;

#ifdef WIDGET_ENUM_TYPE_ENABLED
    if (prop_name == WIDGET_LABEL_PROP_TEXT)
    {
        lv_label_set_text(obj, value_str(val));
    }
    else if (prop_name == WIDGET_LABEL_PROP_COLOR)
    {
        lv_obj_set_style_text_color(obj, color_str_2_lv_color(value_str(val)), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else if (prop_name == WIDGET_LABEL_PROP_FONT_SIZE)
    {
        lv_font_t *font = (lv_font_t *)LV_EXT_FONT_GET(lv_get_font_size_from_name(value_str(val)));
        lv_obj_set_style_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
        res = LV_RES_INV;
    }

#else

    if (0 == rt_strcmp(prop_name, "text"))
    {
        lv_label_set_text(obj, value_str(val));
    }
    else if (0 == rt_strcmp(prop_name, "color"))
    {
        lv_ext_set_local_text_color(obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, color_str_2_lv_color(value_str(val)));
    }
    else if (0 == rt_strcmp(prop_name, "font_size"))
    {
        lv_font_t *font = (lv_font_t *)LV_EXT_FONT_GET(lv_get_font_size_from_name(value_str(val)));
        lv_ext_set_local_text_font(obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, font);
    }
    else
    {
        res = LV_RES_INV;
    }
#endif

    return res;
}


static int str_split(char *s, rt_size_t length, char *item_list[], uint8_t item_list_len)
{
    int cnt = 0;
    char *ptr = s;

    while ((ptr - s) < (int)length)
    {
        /* strip bank and tab */
        while ((*ptr == ' ' || *ptr == '\t') && (ptr - s) < (int)length)
            *ptr++ = '\0';
        /* check whether it's the end of line */
        if ((ptr - s) >= (int)length) break;

        item_list[cnt++] = ptr;
        while ((*ptr != ' ' && *ptr != '\t') && (ptr - s) < (int)length)
        {
            ptr++;
        }

        if (cnt >= item_list_len) break;
    }

    return cnt;
}






#ifdef SOLUTION_WATCH
static lv_res_t lv_wf_set_prop(lv_obj_t *obj, widget_prop_t prop_name, const value_t *val, const void *ui)
{
#define WF_DIG_IMG_NUM  (10)
    lv_res_t res = LV_RES_OK;
    char **img_list;
    const char *s;
    int img_cnt;

#ifdef WIDGET_ENUM_TYPE_ENABLED

    if (prop_name == WIDGET_WATCHFACE_PROP_DIG_HOUR_IMG)
    {
        s = rt_strdup(value_str(val));
        img_list = rt_malloc(sizeof(*img_list) * WF_DIG_IMG_NUM);
        RT_ASSERT(img_list);
        img_cnt = str_split(s, strlen(s), img_list, WF_DIG_IMG_NUM);
        lv_wf_set_digital_hour_img(obj, (const lv_img_dsc_t **)img_list, img_cnt);
        rt_free(img_list);
        rt_free(s);
    }
    else if (prop_name == WIDGET_WATCHFACE_PROP_DIG_MIN_IMG)
    {
        s = rt_strdup(value_str(val));
        img_list = rt_malloc(sizeof(*img_list) * WF_DIG_IMG_NUM);
        RT_ASSERT(img_list);
        img_cnt = str_split(s, strlen(s), img_list, WF_DIG_IMG_NUM);
        lv_wf_set_digital_min_img(obj, (const lv_img_dsc_t **)img_list, img_cnt);
        rt_free(img_list);
        rt_free(s);
    }
    else if (prop_name == WIDGET_WATCHFACE_PROP_DIG_SEC_IMG)
    {
        s = rt_strdup(value_str(val));
        img_list = rt_malloc(sizeof(*img_list) * WF_DIG_IMG_NUM);
        RT_ASSERT(img_list);
        img_cnt = str_split(s, strlen(s), img_list, WF_DIG_IMG_NUM);
        lv_wf_set_digital_sec_img(obj, (const lv_img_dsc_t **)img_list, img_cnt);
        rt_free(img_list);
        rt_free(s);
    }
    else if (prop_name == WIDGET_WATCHFACE_PROP_DIG_SEP_IMG)
    {
        s = rt_strdup(value_str(val));
        img_list = rt_malloc(sizeof(*img_list) * WF_DIG_IMG_NUM);
        RT_ASSERT(img_list);
        img_cnt = str_split(s, strlen(s), img_list, WF_DIG_IMG_NUM);
        if (img_cnt >= 2)
        {
            lv_wf_set_digital_sep_img(obj, img_list[0], img_list[1]);
        }
        rt_free(img_list);
        rt_free(s);
    }
    else if (prop_name == WIDGET_WATCHFACE_PROP_TYPE)
    {
        lv_wf_set_type(obj, WF_TYPE_DIGITAL_IMG);
        lv_wf_invalidate(obj);
        lv_wf_refresh_open(obj);
    }
    else
    {
        res = LV_RES_INV;
    }
#else
    if (0 == rt_strcmp(prop_name, "dig_hour_img"))
    {
        s = rt_strdup(value_str(val));
        img_list = rt_malloc(sizeof(*img_list) * WF_DIG_IMG_NUM);
        RT_ASSERT(img_list);
        img_cnt = str_split(s, strlen(s), img_list, WF_DIG_IMG_NUM);
        lv_wf_set_digital_hour_img(obj, (const lv_img_dsc_t **)img_list, img_cnt);
        rt_free(img_list);
        rt_free(s);
    }
    else if (0 == rt_strcmp(prop_name, "dig_min_img"))
    {
        s = rt_strdup(value_str(val));
        img_list = rt_malloc(sizeof(*img_list) * WF_DIG_IMG_NUM);
        RT_ASSERT(img_list);
        img_cnt = str_split(s, strlen(s), img_list, WF_DIG_IMG_NUM);
        lv_wf_set_digital_min_img(obj, (const lv_img_dsc_t **)img_list, img_cnt);
        rt_free(img_list);
        rt_free(s);
    }
    else if (0 == rt_strcmp(prop_name, "dig_sec_img"))
    {
        s = rt_strdup(value_str(val));
        img_list = rt_malloc(sizeof(*img_list) * WF_DIG_IMG_NUM);
        RT_ASSERT(img_list);
        img_cnt = str_split(s, strlen(s), img_list, WF_DIG_IMG_NUM);
        lv_wf_set_digital_sec_img(obj, (const lv_img_dsc_t **)img_list, img_cnt);
        rt_free(img_list);
        rt_free(s);
    }
    else if (0 == rt_strcmp(prop_name, "dig_sep_img"))
    {
        s = rt_strdup(value_str(val));
        img_list = rt_malloc(sizeof(*img_list) * WF_DIG_IMG_NUM);
        RT_ASSERT(img_list);
        img_cnt = str_split(s, strlen(s), img_list, WF_DIG_IMG_NUM);
        if (img_cnt >= 2)
        {
            lv_wf_set_digital_sep_img(obj, img_list[0], img_list[1]);
        }
        rt_free(img_list);
        rt_free(s);
    }
    else if (0 == rt_strcmp(prop_name, "type"))
    {
        lv_wf_set_type(obj, WF_TYPE_DIGITAL_IMG);
        lv_wf_invalidate(obj);
        lv_wf_refresh_open(obj);
    }
    else
    {
        res = LV_RES_INV;
    }

#endif

    return res;
}

static lv_res_t lv_details_set_prop(lv_obj_t *obj, widget_prop_t prop_name, const value_t *val, const void *ui)
{
    lv_res_t res = LV_RES_INV;


    return res;
}


static lv_res_t lv_roundbar_set_prop(lv_obj_t *obj, widget_prop_t prop_name, const value_t *val, const void *ui)
{
    lv_res_t res = LV_RES_OK;

#ifdef WIDGET_ENUM_TYPE_ENABLED
    if (prop_name == WIDGET_ROUNDBAR_PROP_BG_COLOR)
    {
        lv_roundbar_set_bg_color(obj, color_str_2_lv_color(value_str(val)));
    }
    else if (prop_name == WIDGET_ROUNDBAR_PROP_INDIC_COLOR)
    {
        lv_roundbar_set_indic_color(obj, color_str_2_lv_color(value_str(val)));
    }
    else if (prop_name == WIDGET_ROUNDBAR_PROP_LINE_WIDTH)
    {
        lv_roundbar_set_line_width(obj, value_int(val));
    }
    else if (prop_name == WIDGET_ROUNDBAR_PROP_START_ANGLE)
    {
        lv_coord_t start;
        lv_coord_t end;

        lv_roundbar_get_angle(obj, &start, &end);
        lv_roundbar_set_angle(obj, value_int(val), end);
    }
    else if (prop_name == WIDGET_ROUNDBAR_PROP_END_ANGLE)
    {
        lv_coord_t start;
        lv_coord_t end;

        lv_roundbar_get_angle(obj, &start, &end);
        lv_roundbar_set_angle(obj, start, value_int(val));
    }
    else if (prop_name == WIDGET_ROUNDBAR_PROP_VALUE)
    {
        lv_roundbar_set_value(obj, value_int(val));
    }
    else
    {
        res = LV_RES_INV;
    }

#else

    if (0 == rt_strcmp(prop_name, "bg_color"))
    {
        lv_roundbar_set_bg_color(obj, color_str_2_lv_color(value_str(val)));
    }
    else if (0 == rt_strcmp(prop_name, "indic_color"))
    {
        lv_roundbar_set_indic_color(obj, color_str_2_lv_color(value_str(val)));
    }
    else if (0 == rt_strcmp(prop_name, "line_width"))
    {
        lv_roundbar_set_line_width(obj, value_int(val));
    }
    else if (0 == rt_strcmp(prop_name, "start_angle"))
    {
        lv_coord_t start;
        lv_coord_t end;

        lv_roundbar_get_angle(obj, &start, &end);
        lv_roundbar_set_angle(obj, value_int(val), end);
    }
    else if (0 == rt_strcmp(prop_name, "end_angle"))
    {
        lv_coord_t start;
        lv_coord_t end;

        lv_roundbar_get_angle(obj, &start, &end);
        lv_roundbar_set_angle(obj, start, value_int(val));
    }
    else if (0 == rt_strcmp(prop_name, "value"))
    {
        lv_roundbar_set_value(obj, value_int(val));

    }
    else
    {
        res = LV_RES_INV;
    }
#endif

    return res;
}


#endif

static void obj_resume_core(lv_obj_t *obj)
{
//TODO
#if 0
    /*Recursively resume the children*/
    lv_obj_t *i;
    lv_obj_t *i_next;
    i = _lv_ll_get_head(&(obj->child_ll));
    while (i != NULL)
    {
        /*Get the next object before resume this*/
        i_next = _lv_ll_get_next(&(obj->child_ll), i);

        /*Call the recursive resume to the child too*/
        obj_resume_core(i);

        /*Set i to the next node*/
        i = i_next;
    }

    /* All children paused.
     * Now resume the object specific data*/
    obj->signal_cb(obj, LV_SIGNAL_RESUME, NULL);
#endif
}



/**
 * Pause 'obj' and all of its children
 * @param obj pointer to an object to resume
 * @return LV_RES_OK
 */
lv_res_t lv_obj_resume(lv_obj_t *obj)
{
//    LV_ASSERT_OBJ(obj, LV_OBJX_NAME);

    obj_resume_core(obj);

    return LV_RES_OK;
}



static void obj_pause_core(lv_obj_t *obj)
{
//TODO
#if 0
    /*Recursively pause the children*/
    lv_obj_t *i;
    lv_obj_t *i_next;
    i = _lv_ll_get_head(&(obj->child_ll));
    while (i != NULL)
    {
        /*Get the next object before pause this*/
        i_next = _lv_ll_get_next(&(obj->child_ll), i);

        /*Call the recursive pause to the child too*/
        obj_pause_core(i);

        /*Set i to the next node*/
        i = i_next;
    }

    /* All children paused.
     * Now pause the object specific data*/
    obj->signal_cb(obj, LV_SIGNAL_PAUSE, NULL);
#endif
}



/**
 * Pause 'obj' and all of its children
 * @param obj pointer to an object to pause
 * @return LV_RES_OK
 */
lv_res_t lv_obj_pause(lv_obj_t *obj)
{
//    LV_ASSERT_OBJ(obj, LV_OBJX_NAME);

    obj_pause_core(obj);

    return LV_RES_OK;
}

static lv_res_t lv_obj_set_prop(lv_obj_t *obj, widget_prop_t prop_name, const value_t *val, const void *ui)
{
    lv_res_t res = LV_RES_OK;

#ifdef WIDGET_ENUM_TYPE_ENABLED
    if (prop_name == WIDGET_OBJ_PROP_ALIGN)
    {
        char *s = rt_strdup(value_str(val));
        char *aligned_obj_name;
        lv_obj_t *aligned_obj = NULL;
        lv_align_t align = LV_ALIGN_DEFAULT;

        aligned_obj_name = strstr(s, ":");
        if (aligned_obj_name)
        {
            aligned_obj = ui_builder_default_get_widget(ui, aligned_obj_name + 1);
            *aligned_obj_name = 0;
        }

        if (0 == rt_strcmp(s, "TOP_LEFT"))
        {
            align = LV_ALIGN_TOP_LEFT;
        }
        else if (0 == rt_strcmp(s, "TOP_MID"))
        {
            align = LV_ALIGN_TOP_MID;
        }
        else if (0 == rt_strcmp(s, "TOP_RIGHT"))
        {
            align = LV_ALIGN_TOP_RIGHT;
        }
        else if (0 == rt_strcmp(s, "BOTTOM_LEFT"))
        {
            align = LV_ALIGN_BOTTOM_LEFT;
        }
        else if (0 == rt_strcmp(s, "BOTTOM_MID"))
        {
            align = LV_ALIGN_BOTTOM_MID;
        }
        else if (0 == rt_strcmp(s, "BOTTOM_RIGHT"))
        {
            align = LV_ALIGN_BOTTOM_RIGHT;
        }
        else if (0 == rt_strcmp(s, "LEFT_MID"))
        {
            align = LV_ALIGN_LEFT_MID;
        }
        else if (0 == rt_strcmp(s, "RIGHT_MID"))
        {
            align = LV_ALIGN_RIGHT_MID;
        }
        else if (0 == rt_strcmp(s, "CENTER"))
        {
            align = LV_ALIGN_CENTER;
        }
        else if (0 == rt_strcmp(s, "OUT_TOP_LEFT"))
        {
            align = LV_ALIGN_OUT_TOP_LEFT;
        }
        else if (0 == rt_strcmp(s, "OUT_TOP_MID"))
        {
            align = LV_ALIGN_OUT_TOP_MID;
        }
        else if (0 == rt_strcmp(s, "OUT_TOP_RIGHT"))
        {
            align = LV_ALIGN_OUT_TOP_RIGHT;
        }
        else if (0 == rt_strcmp(s, "OUT_BOTTOM_LEFT"))
        {
            align = LV_ALIGN_OUT_BOTTOM_LEFT;
        }
        else if (0 == rt_strcmp(s, "OUT_BOTTOM_MID"))
        {
            align = LV_ALIGN_OUT_BOTTOM_MID;
        }
        else if (0 == rt_strcmp(s, "OUT_BOTTOM_RIGHT"))
        {
            align = LV_ALIGN_OUT_BOTTOM_RIGHT;
        }
        else if (0 == rt_strcmp(s, "OUT_LEFT_TOP"))
        {
            align = LV_ALIGN_OUT_LEFT_TOP;
        }
        else if (0 == rt_strcmp(s, "OUT_LEFT_MID"))
        {
            align = LV_ALIGN_OUT_LEFT_MID;
        }
        else if (0 == rt_strcmp(s, "OUT_LEFT_BOTTOM"))
        {
            align = LV_ALIGN_OUT_LEFT_BOTTOM;
        }
        else if (0 == rt_strcmp(s, "OUT_RIGHT_TOP"))
        {
            align = LV_ALIGN_OUT_RIGHT_TOP;
        }
        else if (0 == rt_strcmp(s, "OUT_RIGHT_MID"))
        {
            align = LV_ALIGN_OUT_RIGHT_MID;
        }
        else if (0 == rt_strcmp(s, "OUT_RIGHT_BOTTOM"))
        {
            align = LV_ALIGN_OUT_RIGHT_BOTTOM;
        }
        else
        {
            rt_kprintf("unknown align:%s\n", s);
        }

        if (LV_ALIGN_DEFAULT != align)
        {
            if (!aligned_obj)
            {
                lv_obj_align(obj, align, lv_obj_get_x(obj), lv_obj_get_y(obj));
            }
            else
            {
                lv_obj_align_to(obj, aligned_obj, align, lv_obj_get_x(obj), lv_obj_get_y(obj));
            }
        }

        rt_free(s);

    }
    else
    {
        res = LV_RES_INV;
    }
#else
    res = LV_RES_INV;
#endif


    return res;
}


int lv_widget_factory_init(void)
{
    widget_factory_t *factory;
    ret_t r;

    factory = widget_factory_create();
    RT_ASSERT(factory);
    r = widget_factory_set(factory);
    RT_ASSERT(RET_OK == r);

#ifdef WIDGET_ENUM_TYPE_ENABLED
    widget_factory_register(factory, WIDGET_TYPE_ENUM_OBJ,      lv_obj_create,       lv_obj_set_prop);

    widget_factory_register(factory, WIDGET_TYPE_ENUM_ANACLOCK, lv_analogclk_create, lv_analogclk_set_prop);
    widget_factory_register(factory, WIDGET_TYPE_ENUM_IMAGE,    lv_img_create,       NULL);
    widget_factory_register(factory, WIDGET_TYPE_ENUM_IDXIMG,   lv_idximg_create,    NULL);
    widget_factory_register(factory, WIDGET_TYPE_ENUM_LABEL,    lv_label_create,     lv_label_set_prop);
#ifdef SOLUTION_WATCH
    widget_factory_register(factory, WIDGET_TYPE_ENUM_WATCHFACE, lv_wf_create,       lv_wf_set_prop);
    widget_factory_register(factory, WIDGET_TYPE_ENUM_DETAILS,   lv_details_create,  lv_details_set_prop);
    widget_factory_register(factory, WIDGET_TYPE_ENUM_ROUNDBAR,  lv_roundbar_create, lv_roundbar_set_prop);
    widget_factory_register(factory, WIDGET_TYPE_ENUM_WEATHER,   lv_weather_create,  NULL);
    widget_factory_register(factory, WIDGET_TYPE_ENUM_ICON,      lv_icon_create,  NULL);

#endif /* SOLUTION_WATCH */

#else
    widget_factory_register(factory, WIDGET_TYPE_ANACLOCK, lv_analogclk_create, lv_analogclk_set_prop);
    widget_factory_register(factory, WIDGET_TYPE_IMAGE,    lv_img_create,       NULL);
    widget_factory_register(factory, WIDGET_TYPE_IDXIMG,   lv_idximg_create,    NULL);
    widget_factory_register(factory, WIDGET_TYPE_LABEL,    lv_label_create,     lv_label_set_prop);
#ifdef SOLUTION_WATCH
    widget_factory_register(factory, WIDGET_TYPE_WATCHFACE, lv_wf_create,       lv_wf_set_prop);
    widget_factory_register(factory, WIDGET_TYPE_DETAILS,   lv_details_create,  lv_details_set_prop);
    widget_factory_register(factory, WIDGET_TYPE_ROUNDBAR,  lv_roundbar_create, lv_roundbar_set_prop);
    widget_factory_register(factory, WIDGET_TYPE_WEATHER,   lv_weather_create,  NULL);
    widget_factory_register(factory, WIDGET_TYPE_ICON,      lv_icon_create,  NULL);

#endif /* SOLUTION_WATCH */
#endif /* WIDGET_ENUM_TYPE_ENABLED */
    return 0;
}
INIT_ENV_EXPORT(lv_widget_factory_init);



void *lv_load_layout(lv_obj_t *parent, const ui_layout_desc_t *layout)
{
    ui_loader_t *loader = default_ui_loader();
    ui_builder_t *builder = ui_builder_default(layout->id_str);
    ret_t ret;

    builder->widget = parent;

    ret = ui_loader_load(loader, layout->data, layout->data_size, builder);
    RT_ASSERT(RET_OK == ret);

    return builder->ui;
}

void lv_free_layout(void *ui)
{
    ui_builder_t *builder = ui_builder_default("");

    ui_builder_on_destroy(builder, ui);

}



lv_obj_t *lv_find_obj_in_layout(void *ui, const char *name)
{
    return ui_builder_default_get_widget(ui, name);
}


