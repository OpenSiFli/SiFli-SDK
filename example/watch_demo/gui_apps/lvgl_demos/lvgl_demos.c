/*********************
 *      INCLUDES
 *********************/


#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "app_mem.h"


#ifdef LV_USE_DEMO

typedef void (*FuncPtr)(void);

extern int lvgl_demo_run(void (*entry)(void), void (*exit)(void));


LV_IMG_DECLARE(airplane);

#if (LV_USE_DEMO_MUSIC == 1)
    extern void lv_demo_music(void);
    extern void lv_demo_music_close(void);
#endif /* LV_USE_DEMO_MUSIC */

#if (LV_USE_DEMO_BENCHMARK == 1)
    extern void lv_demo_benchmark(void);
    extern void lv_demo_benchmark_close(void);
#endif /* LV_USE_DEMO_BENCHMARK */

#if (LV_USE_DEMO_WIDGETS == 1)
    extern void lv_demo_widgets(void);
    extern void lv_demo_widgets_close(void);
#endif /* LV_USE_DEMO_WIDGETS */

typedef struct
{

    const char *name;
    FuncPtr entry_func;
    FuncPtr exit_func;
} lvgl_demo_t;


static lvgl_demo_t all_demos[] =
{
#if (LV_USE_DEMO_MUSIC == 1)
    {"Music", lv_demo_music, lv_demo_music_close},
#endif /* LV_USE_DEMO_MUSIC */

#if (LV_USE_DEMO_BENCHMARK == 1)
    {"Benchmark", lv_demo_benchmark, lv_demo_benchmark_close},
#endif /* LV_USE_DEMO_BENCHMARK */

#if (LV_USE_DEMO_WIDGETS == 1)
    {"Widgets", lv_demo_widgets, lv_demo_widgets_close},
#endif /* LV_USE_DEMO_WIDGETS */



};


static void btn_go_back_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();

    }
}

static void btn_run_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lvgl_demo_t *p = lv_event_get_user_data(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        lvgl_demo_run(p->entry_func, p->exit_func);

    }
}



static void on_start(void)
{
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);
    lv_obj_update_layout(cont);

    // header
    lv_obj_t *cont_title = lv_lvsfheader_create(cont);
    lv_lvsfheader_set_title(cont_title, "LVGL Demos");
    lv_lvsfheader_set_visible_item(cont_title, LVSF_HEADER_DEFAULT);
    lv_lvsfheader_back_event_cb(cont_title, btn_go_back_callback);

    // Demo list
    lv_obj_t *list1 = lv_list_create(cont);
    lv_obj_set_size(list1, LV_HOR_RES_MAX, LV_VER_RES_MAX - lv_obj_get_height(cont_title));
    lv_obj_align_to(list1, cont_title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);



    /*Add buttons to the list*/
    for (uint32_t i = 0; i < sizeof(all_demos) / sizeof(all_demos[0]); i++)
    {
        lv_obj_t *list_btn;

        list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), all_demos[i].name);
        lv_obj_add_event_cb(list_btn, btn_run_callback, LV_EVENT_ALL, &all_demos[i]);
    }


}

static void on_pause(void)
{

}

static void on_resume(void)
{

}

static void on_stop(void)
{
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

#define APP_ID "lvgl_demos"
static int app_main(intent_t i)
{

    uint32_t active = intent_get_uint32(i, "active", 0xFFFF);
    if (active != 0xFFFF)
    {
        uint32_t len = sizeof(all_demos) / sizeof(all_demos[0]);
        if (active < len)
        {
            lvgl_demo_run(all_demos[active].entry_func, all_demos[active].exit_func);
            return 0;
        }
    }
    gui_app_regist_msg_handler(APP_ID, msg_handler);

    return 0;
}



BUILTIN_APP_EXPORT(LV_EXT_STR_ID(lvgl_demos), LV_EXT_IMG_GET(airplane), APP_ID, app_main);


#endif
