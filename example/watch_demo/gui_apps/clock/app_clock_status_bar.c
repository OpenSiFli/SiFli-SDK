#include "app_clock_status_bar.h"
#include "app_mem.h"

static lv_obj_t *app_clock_main_status_bar;
static lv_obj_t *status_bar_area_up;
static lv_obj_t *status_bar_area_down;

static rt_bool_t alarm_enabled = RT_TRUE;
static rt_bool_t timer_enabled = RT_TRUE;
static lv_obj_t *bar_label1, *bar_label2;

static lv_obj_t *app_clock_tileview;


static const lv_btnmatrix_ctrl_t btnm_ctrl_map[] =
{
    1 | LV_BTNMATRIX_CTRL_DISABLED,  1 | LV_BTNMATRIX_CTRL_DISABLED,
    1 | LV_BTNMATRIX_CTRL_CHECKABLE, 1 | LV_BTNMATRIX_CTRL_CHECKABLE,
};

static const char *btnm_map[] = {"BT", "GPS", "\n",
                                 "ALARM", "TIMER", ""
                                };


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



static void app_clock_main_status_bar_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);

    //if (LV_EVENT_PRESSING != event->code)
    //    rt_kprintf("app_clock_main_status_bar_event_cb %p, got event %s\n", obj, lv_event_to_name(event->code));

    switch (event->code)
    {
    case LV_EVENT_SCROLL_BEGIN:
    {
        static int num = 2;
        num++;
        {
            lv_color_t color = LV_COLOR_RED;
            lv_ext_set_local_font(bar_label1, FONT_TITLE, color);
        }
        {
            lv_color_t color = LV_COLOR_GRAY;
            lv_ext_set_local_font(bar_label2, FONT_TITLE, color);
        }
        //lv_obj_realign(bar_label2);

        if (1 == num % 3)
        {
            lv_label_set_text(bar_label1, "20:08 中文简体");// 陆标、台标之正体字差异 ");
            lv_label_set_text(bar_label2,
                              "界面初始化时读取文本或\n"
                              "进制格式的xml文件，读取\n"
                              "一个标签后，即根据标签\n"
                              "建对应的控件，接着读取\n"
                              "该标签的所有属性，调用\n"
                              "接口设置属性值，遍历所\n"
                              "标签就能创建出界面上的\n");
        }
        else if (2 == num % 3)
        {
            lv_color_t color = LV_COLOR_GREEN;
            lv_label_set_text(bar_label1, "中文繁体,小号字体");
            lv_ext_set_local_font(bar_label2, FONT_SMALL, color);
            lv_label_set_text(bar_label2,
                              "有很多的吧友列過這樣的\n"
                              "單獨整理一下，打在電腦\n"
                              "來了，何不分享到吧裡說\n"
                              "也能給沒收藏那些老帖子\n"
                              "細微的幫助。爭論全民應\n"
                              "也能給沒收藏那些老帖子\n"
                              "也能給沒收藏那些老帖子\n"
                              "細微的幫助。爭論全民應\n"
                              "也能給沒收藏那些老帖子\n"
                              "細微的幫助。爭論全民應\n"
                              "也能給沒收藏那些老帖子\n"
                              "細微的幫助。爭論全民應\n"
                              "個更正這樣的話題其實是\n"
                              "所以然，不如效仿老吧友\n"
                              "」							  \n");

        }
        else //if(3 == num++ % 1)
        {
            lv_color_t color = LV_COLOR_GREEN;

            lv_label_set_text(bar_label1, "超大号字体");
            lv_ext_set_local_font(bar_label2, FONT_SUPER, color);
            lv_label_set_text(bar_label2, "20:08 思澈科技欢迎您");
        }

        lv_obj_scroll_to_view(bar_label1, LV_ANIM_ON);
        lv_obj_scroll_to_view(bar_label2, LV_ANIM_ON);

        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(event);
        rt_kprintf("LV_EVENT_VALUE_CHANGED  %d\n", active_pos);

        if (1 == active_pos)
            lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);

        if (1 == active_pos) lv_ext_font_reset();


        break;
    }
    case LV_EVENT_SHORT_CLICKED:
    case LV_EVENT_LONG_PRESSED:
    case LV_EVENT_CLICKED:
    case LV_EVENT_SCROLL_END:
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
static void msg_list_content_init(lv_obj_t *par)
{
    lv_obj_t *label1;


    label1 = lv_label_create(par);
    lv_label_set_text(label1, "notification");
    lv_obj_align(label1, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_height(label1, LV_PCT(5));


    bar_label1 = lv_label_create(par);
    lv_obj_set_width(bar_label1, LV_PCT(75));         /*Set the label width to max value to not show hor. scroll bars*/
    lv_obj_set_height(bar_label1, LV_PCT(10));         /*Set the label width to max value to not show hor. scroll bars*/
    lv_ext_set_local_bg(bar_label1, LV_COLOR_BLUE, LV_OPA_30);
    lv_obj_align_to(bar_label1, label1, LV_ALIGN_OUT_BOTTOM_MID, 0, PX_5mm);


    //lv_label_set_long_mode(bar_label1, LV_LABEL_LONG_DOT);
    //lv_ext_set_local_font(bar_label1, FONT_TITLE, LV_COLOR_RED);
    //lv_label_set_text(bar_label1, "陸標");// 陸標、台標之正體字差異 ")
    //lv_label_set_long_mode(bar_label1, LV_LABEL_LONG_DOT);

    //lv_obj_t *
    lv_obj_t *page = lv_obj_create(par);
    lv_ext_set_local_bg(page, LV_COLOR_WHITE, LV_OPA_80);
    lv_obj_set_size(page, LV_PCT(75), LV_PCT(50));
    lv_obj_align_to(page, bar_label1, LV_ALIGN_OUT_BOTTOM_MID, 0, PX_5mm);


    /*Create a label on the page*/
    bar_label2 = lv_label_create(page);

    lv_label_set_long_mode(bar_label2, LV_LABEL_LONG_WRAP);            /*Automatically break long lines*/
    lv_obj_set_width(bar_label2, LV_PCT(100));            /*Set the label width to max value to not show hor. scroll bars*/
    lv_obj_align(bar_label2, LV_ALIGN_TOP_MID, 0, 0);
    //lv_obj_set_auto_realign(bar_label2, true);
    //lv_ext_text_bg_sytel_set(bar_label2, LV_COLOR_GRAY, 255);
    //lv_obj_set_height(bar_label2, lv_page_get_height_fit(page));      /*Set the label height to max value to not show hor. scroll bars*/

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
    lv_obj_clear_flag(tileview, LV_OBJ_FLAG_SCROLL_ONE);
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
            lv_obj_set_style_bg_opa(pages[i], LV_OPA_80, LV_PART_MAIN | LV_STATE_DEFAULT);
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
    lv_obj_del(app_clock_main_status_bar);
    lv_obj_del(status_bar_area_up);
    lv_obj_del(status_bar_area_down);

    app_clock_main_status_bar = NULL;
    status_bar_area_up = NULL;
    status_bar_area_down = NULL;
    lv_ext_font_reset();
}
/*
lvsf_msg_content_create(lv_obj_t *par, lv_obj *icon, char *header, char *text)
{
    bar_label1 = lv_label_create(par, NULL);
    lv_obj_set_pos(bar_label1, 0, 60);
    lv_obj_set_width(bar_label1, LV_VER_RES_MAX - 100);
    lv_obj_align(bar_label1, pages[0], LV_ALIGN_IN_TOP_MID, 0, 60);
    lv_obj_set_auto_realign(bar_label1, true);

    lv_obj_t *page = lv_page_create(par, NULL);
    lv_obj_set_pos(page, 0, 120);
    lv_obj_set_size(page, LV_HOR_RES_MAX, LV_VER_RES_MAX - 160);

}

*/
