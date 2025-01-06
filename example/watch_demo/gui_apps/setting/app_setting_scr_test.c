#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "custom_trans_anim.h"


enum
{
    TYPE_COLOR_TEST_E,
    TYPE_COLOR_BAR_TEST_E,
    TYPE_GRADIENT_COLOR_TEST_E,
    TYPE_TE_TEST_E,
    TYPE_MAX_TEST_E
};


typedef struct
{
    uint32_t test_type;
    uint32_t test_v;
    lv_obj_t *btn;
    lv_timer_t *redraw_task;
} src_test_t;

static src_test_t *p_scr_test = NULL;
static uint32_t last_test_type = TYPE_MAX_TEST_E;
const uint32_t color_bar[8] = {0xFFFFFF, 0x777777, 0xFF0000, 0x00FF00, 0x0000FF, 0x00FFFF, 0xFF00FF, 0xFFFF00};

static void task_cb(lv_timer_t *task)
{
    switch (p_scr_test->test_type)
    {

    case TYPE_COLOR_TEST_E://Color test
    {
        if (p_scr_test->test_v > 7)
            p_scr_test->test_v = 0;
        else
            p_scr_test->test_v ++;

        lv_obj_set_style_bg_color(p_scr_test->btn, lv_color_hex(color_bar[p_scr_test->test_v]), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_timer_set_period(p_scr_test->redraw_task, 5000);
    }
    break;

    case TYPE_GRADIENT_COLOR_TEST_E:
    {
        if (p_scr_test->test_v > 7)
            p_scr_test->test_v = 0;
        else
            p_scr_test->test_v ++;

        lv_obj_set_style_bg_color(p_scr_test->btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(p_scr_test->btn, lv_color_hex(color_bar[p_scr_test->test_v]), LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_timer_set_period(p_scr_test->redraw_task, 5000);
    }
    break;


    case TYPE_TE_TEST_E://TE test
    {
        if (p_scr_test->test_v > 300)
        {
            lv_obj_set_size(p_scr_test->btn, rand() % LV_HOR_RES_MAX, rand() % LV_VER_RES_MAX);
            lv_obj_set_y(p_scr_test->btn, rand() % (LV_VER_RES_MAX - lv_obj_get_height(p_scr_test->btn)));

            lv_obj_set_style_bg_color(p_scr_test->btn, lv_color_make(rand() % 255, rand() % 255, rand() % 255), LV_PART_MAIN | LV_STATE_DEFAULT);
            p_scr_test->test_v = 0;
        }
        p_scr_test->test_v ++;

        lv_obj_set_x(p_scr_test->btn, rand() % (LV_HOR_RES_MAX - lv_obj_get_width(p_scr_test->btn)));
    }
    break;

    default:
        break;
    }


}

static void btn_click_event_callback(lv_event_t *e)
{
    switch (p_scr_test->test_type)
    {
    case TYPE_TE_TEST_E:
    {

    }
    break;

    default:
        break;
    }
}

static void on_start(void)
{
    p_scr_test = (src_test_t *) rt_malloc(sizeof(src_test_t));
    memset(p_scr_test, 0, sizeof(src_test_t));



    if (last_test_type + 1 >= TYPE_MAX_TEST_E)
        last_test_type = 0;
    else
        last_test_type++;

    p_scr_test->test_type = last_test_type;

    switch (p_scr_test->test_type)
    {
    case TYPE_COLOR_TEST_E:
    {
        lv_obj_t *btn = lv_obj_create(lv_scr_act());
        lv_obj_set_size(btn, LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_style_bg_color(btn, lv_color_hex(color_bar[0]), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_align(btn, LV_ALIGN_TOP_LEFT);
        lv_obj_set_style_radius(btn, 0, 0);


        p_scr_test->btn = btn;
    }
    break;

    case TYPE_TE_TEST_E://Te test
    {
        lv_obj_t *btn = lv_obj_create(lv_scr_act());
        lv_obj_set_size(btn, LV_HOR_RES_MAX / 3, LV_VER_RES_MAX / 2);
        lv_obj_set_style_bg_color(btn, lv_color_make(255, 2, 34), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_align(btn, LV_ALIGN_TOP_LEFT);
        lv_obj_set_style_radius(btn, 0, 0);

        p_scr_test->btn = btn;
    }
    break;

    case TYPE_COLOR_BAR_TEST_E:
    {
        lv_obj_t *btn[8];
        int32_t btn_height = LV_VER_RES_MAX / 8;

        for (int32_t i = 0; i < 8; i++)
        {
            btn[i] = lv_obj_create(lv_scr_act());
            lv_obj_set_size(btn[i], LV_HOR_RES_MAX, btn_height);
            lv_obj_set_style_bg_grad_dir(btn[i], LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_width(btn[i], 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(btn[i],      lv_color_hex(0x000000),       LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_grad_color(btn[i], lv_color_hex(color_bar[i]), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(btn[i], 0, 0);

            lv_obj_set_y(btn[i], btn_height * i);
        }
    }
    break;

    case TYPE_GRADIENT_COLOR_TEST_E:
    {
        lv_obj_t *btn = lv_obj_create(lv_scr_act());
        lv_obj_set_size(btn, LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, lv_color_hex(color_bar[0]), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_align(btn, LV_ALIGN_TOP_LEFT);
        lv_obj_set_style_radius(btn, 0, 0);


        p_scr_test->btn = btn;
    }
    break;

    default:
        break;
    }




    if (p_scr_test->btn) lv_obj_add_event_cb(p_scr_test->btn, btn_click_event_callback, LV_EVENT_CLICKED, 0);
}

static void on_resume(void)
{
    if (!p_scr_test->redraw_task) p_scr_test->redraw_task = lv_timer_create(task_cb, 16, (void *)0);
}

static void on_pause(void)
{
    if (p_scr_test->redraw_task) lv_timer_del(p_scr_test->redraw_task);
    p_scr_test->redraw_task = NULL;
}

static void on_stop(void)
{
    if (p_scr_test)
    {
        rt_free(p_scr_test);
    }
    p_scr_test = NULL;
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

void app_setting_scr_test_main(void)
{
    gui_app_create_page("scr_test", msg_handler);
}
