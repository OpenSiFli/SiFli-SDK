#include "app_clock_status_bar.h"
#include "app_mem.h"
#include "lv_tiny_ttf.h"
typedef struct
{
    lv_color_t title_color;
    lv_color_t content_color;
    int32_t content_height;
    const uint8_t *p_title;
    const uint8_t *p_content;
    uint8_t dynamic_title_font_size;
    uint8_t dynamic_content_font_size;
} notification_msg_cntxt_t;



typedef struct
{
    uint8_t font_size;
    lv_font_t *font;
} font_cache_t;

#define MAX_FONTS 10
static font_cache_t title_font_cache[MAX_FONTS];
static font_cache_t content_font_cache[MAX_FONTS];
static uint8_t title_font_count = 0;
static uint8_t content_font_count = 0;

static lv_obj_t *app_clock_main_status_bar;
static lv_obj_t *status_bar_area_up;
static lv_obj_t *status_bar_area_down;

static rt_bool_t alarm_enabled = RT_TRUE;
static rt_bool_t timer_enabled = RT_TRUE;


static lv_obj_t *app_clock_tileview;


static const lv_btnmatrix_ctrl_t btnm_ctrl_map[] =
{
    1 | LV_BTNMATRIX_CTRL_DISABLED,  1 | LV_BTNMATRIX_CTRL_DISABLED,
    1 | LV_BTNMATRIX_CTRL_CHECKABLE, 1 | LV_BTNMATRIX_CTRL_CHECKABLE,
};

static const char *btnm_map[] = {"BT", "GPS", "\n",
                                 "ALARM", "TIMER", ""
                                };
static const notification_msg_cntxt_t notify_msgs[] =
{
    {
        LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
        LV_COLOR_MAKE(0x00, 0xFF, 0x00),  LV_PCT(20),
        "思澈科技欢迎您",
        "思澈科技欢迎您",
        18,
        16
    },
    {
        LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
        LV_COLOR_MAKE(0x00, 0xFF, 0x00),  LV_PCT(20),
        "思澈科技欢迎您",
        "思澈科技欢迎您",
        22,
        20
    },
    {
        LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
        LV_COLOR_MAKE(0x00, 0xFF, 0x00),  LV_PCT(20),
        "思澈科技欢迎您",
        "思澈科技欢迎您",
        24,
        22
    },
    {
        LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
        LV_COLOR_MAKE(0x00, 0x00, 0x00),  LV_PCT(80),
        "关于我们",
        "思澈科技是一家专注于物联网技术的公司，提供一站式的物联网解决方案。最先提出嵌入式MCU+GPU的物联网解决方案，为客户提供更高性能、更低功耗的物联网产品。",
        26,
        24
    },

    {
        LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
        LV_COLOR_MAKE(0x00, 0x00, 0xFF),  LV_PCT(80),
        "我们是谁",
        "思澈科技成立於2019年3月，總部位於上海張江高科技園區，在重慶、北京、深圳、蘇州均設有分子公司，團隊成員均來自於美國、中國的一線電晶體設計企業，包括Marvell、 Broadcom、Amazon、 紫光展銳、聯發科等，碩士以上學歷占比超過80%； 團隊骨幹具有豐富的產品定義->自主研發->大規模量產的全流程經驗，由這些骨幹成員主導研發的晶片累計出貨超過10億顆。",
        28,
        26
    },

};
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define PX_5mm LV_DPX(32) //160 is 1 inch(about 2.5cm)
#define PX_1cm LV_DPX(64) //160 is 1 inch(about 2.5cm)

#ifndef SF32LB55X
    #if (LV_USE_LABEL && LV_USE_CANVAS && LV_DRAW_COMPLEX) && defined(BSP_USING_PSRAM)
        #define ENABLE_GRADIENT_LABEL
    #endif
#endif /* SF32LB55X */
static void app_clock_main_press_to_show_status_bar(lv_event_t *event)
{

    if (LV_EVENT_PRESSED == event->code)
    {
        //rt_kprintf("app_clock_main_press_to_show_status_bar\n");

        lv_obj_set_tile_id(app_clock_main_status_bar, 0, 1, false);
        lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
        //Bring status bar to foreground
        lv_obj_move_foreground(app_clock_main_status_bar);
    }
}

/**
 * 获取当前活跃 tile 所在的行号
 */
rt_uint16_t get_active_tile_row(lv_obj_t *tileview)
{
    lv_tileview_t *tv = (lv_tileview_t *)tileview;
    lv_obj_t *active_tile = tv->tile_act; // 获取当前活跃的 tile 对象

    if (!active_tile)
    {
        return 0; // 默认返回第一行
    }

    int32_t h = lv_obj_get_content_height(tileview); // 获取 Tileview 单个 tile 的高度
    int32_t y = lv_obj_get_y(active_tile); // 获取当前活跃 tile 的 Y 坐标

    // 计算当前 tile 所在行
    rt_uint16_t row_id = (y + h / 2) / h;

    return row_id;
}

static void app_clock_main_status_bar_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);


    switch (event->code)
    {
    case LV_EVENT_RELEASED:
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint32_t active_pos = (rt_uint32_t)get_active_tile_row(lv_event_get_current_target(event));
        rt_kprintf("LV_EVENT_VALUE_CHANGED  %d\n", active_pos);

        if (1 == active_pos)
            lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);

        // if (1 == active_pos) lv_ext_font_reset();


        break;
    }
    case LV_EVENT_SHORT_CLICKED:
    case LV_EVENT_LONG_PRESSED:
    case LV_EVENT_CLICKED:

    case LV_EVENT_FOCUSED:
    default:
        //printf("Released\n");

        break;
    }
}

static void btnm_event_handler(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);

    if (event->code == LV_EVENT_VALUE_CHANGED)
    {
        const char *txt = lv_btnmatrix_get_btn_text(obj, lv_btnmatrix_get_selected_btn(obj));

        if (txt)
        {
            rt_kprintf("%s was pressed\n", txt);

            if (0 == strcmp(txt, "ALARM"))
                alarm_enabled = !alarm_enabled;

            if (0 == strcmp(txt, "TIMER"))
                timer_enabled = !timer_enabled;
        }
    }
}

#ifdef ENABLE_GRADIENT_LABEL
/*
    pull up hidden control panel
*/
#define MASK_WIDTH 200
#define MASK_HEIGHT 45

static void add_mask_event_cb(lv_event_t *e)
{
    static lv_draw_mask_map_param_t m;
    static int16_t mask_id;

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    lv_opa_t *mask_map = lv_event_get_user_data(e);
    if (code == LV_EVENT_COVER_CHECK)
    {
        lv_event_set_cover_res(e, LV_COVER_RES_MASKED);
    }
    else if (code == LV_EVENT_DRAW_MAIN_BEGIN)
    {
        lv_draw_mask_map_init(&m, &obj->coords, mask_map);
        mask_id = lv_draw_mask_add(&m, NULL);
    }
    else if (code == LV_EVENT_DRAW_MAIN_END)
    {
        lv_draw_mask_free_param(&m);
        lv_draw_mask_remove_id(mask_id);
    }
    else if (code == LV_EVENT_DELETE)
    {
        app_cache_free(mask_map);
    }
}

static lv_obj_t *gradient_label(lv_obj_t *parent, const char *text)
{
    /* Create the mask of a text by drawing it to a canvas*/
    lv_opa_t *mask_map = app_cache_alloc(MASK_WIDTH * MASK_HEIGHT, IMAGE_CACHE_PSRAM);

    LV_ASSERT(mask_map);

    /*Create a "8 bit alpha" canvas and clear it*/
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(canvas, mask_map, MASK_WIDTH, MASK_HEIGHT, LV_IMG_CF_ALPHA_8BIT);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_TRANSP);

    /*Draw a label to the canvas. The result "image" will be used as mask*/
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.align = LV_TEXT_ALIGN_CENTER;
    lv_canvas_draw_text(canvas, 5, 5, MASK_WIDTH, &label_dsc, text);

    /*The mask is reads the canvas is not required anymore*/
    lv_obj_del(canvas);

    /* Create an object from where the text will be masked out.
     * Now it's a rectangle with a gradient but it could be an image too*/
    lv_obj_t *grad = lv_obj_create(parent);
    lv_obj_set_size(grad, MASK_WIDTH, MASK_HEIGHT);
    lv_obj_center(grad);
    lv_obj_set_style_bg_color(grad, lv_color_hex(0xff0000), 0);
    lv_obj_set_style_bg_grad_color(grad, lv_color_hex(0x0000ff), 0);
    lv_obj_set_style_bg_grad_dir(grad, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_radius(grad, 0, 0);
    lv_obj_add_event_cb(grad, add_mask_event_cb, LV_EVENT_ALL, mask_map);

    return grad;
}
#endif /* ENABLE_GRADIENT_LABEL */

static void control_panel_content_init(lv_obj_t *par)
{
    lv_obj_t *redraw_interval_slider, *clock_step_ms_slider;
    lv_obj_t *label1, *label2;

#ifdef ENABLE_GRADIENT_LABEL
    label1 = gradient_label(par, "Functioooooooooooooooons");
#else
    label1 = lv_label_create(par);
    lv_label_set_text(label1, "Functions");
#endif
    lv_obj_align(label1, LV_ALIGN_TOP_MID, 0, PX_5mm);

    lv_obj_t *btnm1 = lv_btnmatrix_create(par);
    lv_btnmatrix_set_map(btnm1, btnm_map);
    lv_obj_set_width(btnm1, LV_PCT(75));
    //lv_btnm_set_btn_width(btnm1, 10, 2);        /*Make "Action1" twice as wide as "Action2"*/
    lv_obj_add_event_cb(btnm1, btnm_event_handler, LV_EVENT_ALL, NULL);
    lv_btnmatrix_set_ctrl_map(btnm1, btnm_ctrl_map);
    lv_obj_align_to(btnm1, label1, LV_ALIGN_OUT_BOTTOM_MID, 0, PX_5mm);



    label2 = lv_label_create(par);
    lv_label_set_text(label2, "clock redraw time & steps");
    lv_obj_align_to(label2, btnm1, LV_ALIGN_OUT_BOTTOM_MID, 0, PX_1cm);

    redraw_interval_slider = lv_slider_create(par);
    lv_bar_set_range(redraw_interval_slider, 1, 3000);
    //lv_bar_set_value(redraw_interval_slider, CLOCK_MIN_REDRAW_INTERVAL_MS, LV_ANIM_ON);
    lv_obj_set_width(redraw_interval_slider, LV_PCT(75));
    lv_obj_align_to(redraw_interval_slider, label2, LV_ALIGN_OUT_BOTTOM_MID, 0, PX_5mm);

    //lv_obj_set_event_cb(redraw_interval_slider, redraw_interval_slider_event_handler);


    clock_step_ms_slider = lv_slider_create(par);
    lv_bar_set_range(clock_step_ms_slider, 1, 6000);
    //lv_bar_set_value(clock_step_ms_slider, CLOCK_MIN_REDRAW_INTERVAL_MS, LV_ANIM_ON);
    lv_obj_set_width(clock_step_ms_slider, LV_PCT(75));
    lv_obj_align_to(clock_step_ms_slider, redraw_interval_slider, LV_ALIGN_OUT_BOTTOM_MID, 0, PX_1cm);

    //lv_obj_set_event_cb(clock_step_ms_slider, clock_step_ms_event_handler);
}

/*
    drop down hidden msg list
*/

extern const unsigned char droid_sans_fallback_font[];
extern const int droid_sans_fallback_font_size;
static lv_font_t *chinese_font = NULL;
void app_clock_status_bar_init_font(void)
{

    if (!chinese_font)
    {
        chinese_font = lv_tiny_ttf_create_data(droid_sans_fallback_font, droid_sans_fallback_font_size, 24);
    }


    for (uint32_t i = 0; i < ARRAY_SIZE(notify_msgs); i++)
    {
        uint8_t size = notify_msgs[i].dynamic_title_font_size;

        // Check if the font size already exists in the cache
        bool found = false;
        for (int j = 0; j < title_font_count; j++)
        {
            if (title_font_cache[j].font_size == size)
            {
                found = true;
                break;
            }
        }

        if (!found && title_font_count < MAX_FONTS)
        {
            title_font_cache[title_font_count].font_size = size;
            title_font_cache[title_font_count].font =
                lv_tiny_ttf_create_data(droid_sans_fallback_font, droid_sans_fallback_font_size, size);
            title_font_count++;
        }
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(notify_msgs); i++)
    {
        uint8_t size = notify_msgs[i].dynamic_content_font_size;

        bool found = false;
        for (int j = 0; j < content_font_count; j++)
        {
            if (content_font_cache[j].font_size == size)
            {
                found = true;
                break;
            }
        }
        // If the font size is not found in the cache, create a new font
        if (!found && content_font_count < MAX_FONTS)
        {
            content_font_cache[content_font_count].font_size = size;
            content_font_cache[content_font_count].font =
                lv_tiny_ttf_create_data(droid_sans_fallback_font, droid_sans_fallback_font_size, size);
            content_font_count++;
        }
    }

}

static void msg_list_content_init(lv_obj_t *par)
{
    app_clock_status_bar_init_font();
    lv_obj_t *label_header;
    lv_obj_t *align_base;

    label_header = lv_label_create(par);
    lv_label_set_text(label_header, "Notifications");
    lv_obj_set_style_text_font(label_header, chinese_font, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_header, LV_COLOR_WHITE, LV_PART_MAIN);
    lv_obj_set_size(label_header, LV_PCT(100), LV_PCT(5));
    lv_obj_align(label_header, LV_ALIGN_TOP_MID, 0, LV_PCT(5));

    align_base = label_header;
    for (uint32_t i = 0; i < sizeof(notify_msgs) / sizeof(notify_msgs[0]); i++)
    {


        lv_font_t *title_font = NULL;
        for (int j = 0; j < title_font_count; j++)
        {
            if (title_font_cache[j].font_size == notify_msgs[i].dynamic_title_font_size)
            {
                title_font = title_font_cache[j].font;
                break;
            }
        }

        lv_obj_t *title_label = lv_label_create(par);
        lv_obj_set_width(title_label, LV_PCT(80));
        lv_label_set_long_mode(title_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_style_text_font(title_label, title_font, LV_PART_MAIN);
        lv_obj_set_style_text_color(title_label, notify_msgs[i].title_color, LV_PART_MAIN);
        lv_label_set_text(title_label, notify_msgs[i].p_title);
        lv_obj_align_to(title_label, align_base, LV_ALIGN_OUT_BOTTOM_LEFT, 0, PX_5mm);

        lv_font_t *content_font = NULL;
        for (int j = 0; j < content_font_count; j++)
        {
            if (content_font_cache[j].font_size == notify_msgs[i].dynamic_content_font_size)
            {
                content_font = content_font_cache[j].font;
                break;
            }
        }


        lv_obj_t *content_label_container = lv_obj_create(par);
        lv_obj_set_size(content_label_container, LV_PCT(100), notify_msgs[i].content_height);
        lv_ext_set_local_bg(content_label_container, LV_COLOR_GRAY, LV_OPA_80);


        lv_obj_t *content_label = lv_label_create(content_label_container);
        lv_obj_set_width(content_label, LV_PCT(100));
        lv_label_set_long_mode(content_label, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(content_label, content_font, LV_PART_MAIN);
        lv_obj_set_style_text_color(content_label, notify_msgs[i].content_color, LV_PART_MAIN);
        lv_label_set_text(content_label, notify_msgs[i].p_content);
        lv_obj_set_layout(content_label, LV_LAYOUT_FLEX);


        lv_obj_align_to(title_label, align_base, LV_ALIGN_OUT_BOTTOM_LEFT, 0, PX_5mm);
        lv_obj_align_to(content_label_container, title_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
        align_base = content_label_container;

    }
}

void app_clock_main_status_bar_init(lv_obj_t *par, lv_obj_t *clock_tileview)
{
    rt_uint16_t i;
    lv_obj_t *tileview;
    lv_obj_t *pages[3];
    lv_obj_t *status_bar_area;

    //create a invisible object at top of parent, and shown status bar when press it
    for (i = 0; i < 2; i++)
    {
        status_bar_area = lv_obj_create(par);
        lv_obj_set_size(status_bar_area, LV_HOR_RES_MAX, (LV_VER_RES_MAX >> 4));
        lv_obj_set_style_border_opa(status_bar_area, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_scrollbar_mode(status_bar_area, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(status_bar_area, LV_OBJ_FLAG_PRESS_LOCK); //Allow press event to tileview
        lv_obj_add_event_cb(status_bar_area, app_clock_main_press_to_show_status_bar, LV_EVENT_ALL, NULL);
        lv_obj_set_style_bg_opa(status_bar_area, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);


        if (0 == i)
        {
            lv_obj_align(status_bar_area, LV_ALIGN_BOTTOM_MID, 0, 0);
            status_bar_area_down = status_bar_area;
        }
        else if (1 == i)
        {
            lv_obj_align(status_bar_area, LV_ALIGN_TOP_MID, 0, 0);
            status_bar_area_up = status_bar_area;
        }

    }

    //create tile view , page 0  for content, page 1 is transparent
    tileview = lv_tileview_create(par);
    app_clock_main_status_bar = tileview;
    lv_obj_add_flag(tileview, LV_OBJ_FLAG_SCROLL_ONE);
    lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_opa(tileview, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);


    for (i = 0; i < 3; i++)
    {
        pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_VER);

        if (i == 1)
        {
            lv_obj_set_style_bg_opa(pages[i], LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        else
        {
            lv_color_t color = LV_COLOR_BLACK;
            lv_obj_set_style_bg_opa(pages[i], LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(pages[i], color, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        lv_obj_set_size(pages[i], LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_pos(pages[i], 0, (LV_VER_RES_MAX * i));
        lv_obj_set_scrollbar_mode(pages[i], LV_SCROLLBAR_MODE_OFF);
    }

    msg_list_content_init(pages[0]);
    control_panel_content_init(pages[2]);

    //scroll to page[1]
    lv_obj_set_tile_id(tileview, 0, 1, false);
    lv_obj_add_event_cb(tileview, app_clock_main_status_bar_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(tileview, LV_OBJ_FLAG_HIDDEN);

    app_clock_tileview = clock_tileview;
}

void app_clock_main_status_bar_deinit(void)
{

    for (int i = 0; i < title_font_count; i++)
    {
        if (title_font_cache[i].font)
        {
            lv_tiny_ttf_destroy(title_font_cache[i].font);
            title_font_cache[i].font = NULL;
        }
    }
    title_font_count = 0;

    for (int i = 0; i < content_font_count; i++)
    {
        if (content_font_cache[i].font)
        {
            lv_tiny_ttf_destroy(content_font_cache[i].font);
            content_font_cache[i].font = NULL;
        }
    }
    content_font_count = 0;

    if (chinese_font)
    {
        lv_tiny_ttf_destroy(chinese_font);
        chinese_font = NULL;
    }


    lv_obj_del(app_clock_main_status_bar);
    lv_obj_del(status_bar_area_up);
    lv_obj_del(status_bar_area_down);

    app_clock_main_status_bar = NULL;
    status_bar_area_up = NULL;
    status_bar_area_down = NULL;

}
