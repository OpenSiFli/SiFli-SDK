/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lv_ext_resource_manager.h"
#include "gui_app_fwk.h"
#include "custom_trans_anim.h"

#define CANVAS_WIDTH (LV_HOR_RES_MAX)
#define CANVAS_HEIGHT (LV_VER_RES_MAX)
#define APP_ID "setting"


LV_IMG_DECLARE(dn);
LV_IMG_DECLARE(bluetooth);
LV_IMG_DECLARE(airplane);
LV_IMG_DECLARE(clock_40);
LV_IMG_DECLARE(clock_80);
LV_IMG_DECLARE(clock_100);


LV_IMG_DECLARE(img_stocks);
LV_IMG_DECLARE(img_settings);


/**
 *  description of app setting
 *
 */
typedef struct
{
    lv_obj_t *main_window;
    lv_timer_t *task;
    lv_obj_t *list1;
    lv_obj_t *selected_lang;
} app_setting_t;

static app_setting_t *p_app_setting = NULL;

//static uint8_t canvas_buffer[LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(CANVAS_WIDTH, CANVAS_HEIGHT)];

static lv_obj_t *get_main_win(void);


//static lv_group_t *p_app_setting->list_group = NULL;

extern void lv_refr_set_mask_area_debug(bool en);
extern bool lv_refr_get_mask_area_debug(void);
extern void app_setting_display_main(void);
void app_setting_sys_main(void);
void app_setting_scr_test_main(void);


#ifdef WIN32
void lv_gpu_set_enable(bool en) {};
bool lv_gpu_is_enabled(void)
{
    return false;
};
#else
extern void lv_gpu_set_enable(bool en);
extern bool lv_gpu_is_enabled(void);
#endif


#if defined(SF32LB52X) && (!BSP_USING_PC_SIMULATOR)
    #define EN_SWITCH_JLINK_DBGUART
    #ifdef FINSH_USING_MSH
        #include "msh.h"
    #endif /* FINSH_USING_MSH */
#endif /* SF32LB52X */

#if 0
static void img_animation(lv_task_t *task)
{
    static uint32_t scale = 1000;
    static int32_t scale_dir = 1;

    if (p_app_setting->canvas)
    {
        lv_img_dsc_t *canvas_img = lv_canvas_get_img(p_app_setting->canvas);
        memset((void *)canvas_img->data, 0, canvas_img->data_size);
        lvsf_canvas_rotate(p_app_setting->canvas, (lv_img_dsc_t *)LV_EXT_IMG_GET(clock_80), 0, scale, 60, 60, 40, 40);

        scale += scale_dir * 10;

        if (scale > 1500)
        {
            scale_dir = -1;
        }
        else if (scale < 1000)
        {
            scale_dir = 1;
        }
    }
}
#endif

#ifdef EN_SWITCH_JLINK_DBGUART
static uint8_t app_setting_dvlp_get_jlink_status(void)
{
    uint32_t *pa18_pinmux = (uint32_t *)0x5000307c; //PAD_PA18
    uint32_t *pa19_pinmux = (uint32_t *)0x50003080; //PAD_PA19

    if ((0x02 == ((*pa18_pinmux) & 0xF)) && (0x02 == ((*pa19_pinmux) & 0xF)))
        return 1;
    else
        return 0;
}

static void app_setting_dlvp_set_jlink_uart_switch(lv_event_t *e)
{
    uint32_t *pa18_pinmux = (uint32_t *)0x5000307c; //PAD_PA18
    uint32_t *pa19_pinmux = (uint32_t *)0x50003080; //PAD_PA19

    rt_kprintf("app_setting_dlvp_set_jlink_uart_switch(PA18=%x,PA19=%x)", *pa18_pinmux, *pa19_pinmux);


    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_VALUE_CHANGED == event)
    {
        if ((lv_obj_get_state(obj)&LV_STATE_CHECKED)) // ON
        {
            rt_kprintf("Switch to Jlink");
            //HAL_PIN_Set(PAD_PA18, SWDIO, PIN_PULLDOWN, 1);
            //HAL_PIN_Set(PAD_PA19, SWCLK, PIN_PULLDOWN, 1);
            *pa18_pinmux = 0x2D2;
            *pa19_pinmux = 0x2D2;

#if 0//def FINSH_USING_MSH
            msh_set_console("segger");
#endif /* FINSH_USING_MSH */
        }
        else //off
        {
            rt_kprintf("Switch to DBG UART");
            //HAL_PIN_Set(PAD_PA18, USART1_RXD, PIN_PULLUP, 1);
            //HAL_PIN_Set(PAD_PA19, USART1_TXD, PIN_PULLUP, 1);
            *pa18_pinmux = 0x2F4;
            *pa19_pinmux = 0x2B4;

#if 0//def FINSH_USING_MSH
            msh_set_console("uart1");
#endif /* FINSH_USING_MSH */
        }


        if (lv_obj_get_state(obj)&LV_STATE_CHECKED) lv_obj_add_state(obj, LV_STATE_PRESSED);
    }
}

#endif /* EN_SWITCH_JLINK_DBGUART */


static void btn_close_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();

    }
}

static void btn_time_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();

    }
    if (LV_EVENT_FOCUSED == event)
    {
        //TODO:
        //lv_group_focus_obj(obj);
        //lv_list_focus(obj, LV_ANIM_ON);
    }

}

static void lang_btn_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    //rt_kprintf("event:%d,%d\n", event, lv_btn_get_state(obj));
    if (event == LV_EVENT_VALUE_CHANGED)
    {
        lv_state_t new_state = lv_obj_get_state(obj);
        //rt_kprintf("lang:%s,%d\n", lv_list_get_btn_text(obj), new_state);
        if ((p_app_setting->selected_lang != obj) && (0 == (LV_STATE_PRESSED & new_state)))
        {
            if (p_app_setting->selected_lang)
            {
                //TODO:
                lv_obj_clear_state(p_app_setting->selected_lang, LV_STATE_CHECKED);
                //lv_btn_toggle(p_app_setting->selected_lang);
            }
            p_app_setting->selected_lang = obj;
            lv_ext_set_locale(NULL, lv_list_get_btn_text(NULL, obj));
        }
        else if ((p_app_setting->selected_lang == obj) && (0 == (LV_STATE_PRESSED & new_state)))
        {
            //TODO
            //lv_btn_toggle(obj);
            if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
            {
                lv_obj_clear_state(obj, LV_STATE_CHECKED);
                lv_obj_clear_state(obj, LV_STATE_PRESSED);
            }
            else
            {
                lv_obj_add_state(obj, LV_STATE_CHECKED);
                lv_obj_add_state(obj, LV_STATE_PRESSED);
            }
        }


        gui_app_self_exit();
        gui_app_run(APP_ID);;
    }
}


static void lang_subpage_msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        // header
        lv_obj_t *cont_title = lv_lvsfheader_create(lv_scr_act());
        lv_lvsfheader_set_title(cont_title, LV_EXT_STR_GET_BY_KEY(setting_language, "Language"));
        lv_lvsfheader_set_visible_item(cont_title, LVSF_HEADER_DEFAULT);
        lv_lvsfheader_back_event_cb(cont_title, btn_close_event_callback);



        // lang list
        lv_obj_t *list1 = lv_list_create(lv_scr_act());
        lv_obj_set_size(list1, LV_PCT(100), LV_VER_RES_MAX - lv_obj_get_height(cont_title));
        lv_obj_align_to(list1, cont_title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);



        /*Add buttons to the list*/
        lv_obj_t *list_btn;
        const char *curr_locale = lv_ext_get_locale();
        p_app_setting->selected_lang = NULL;
        LV_EXT_LANG_PACK_LIST_ITER_DEF(iter);

        LV_EXT_LANG_PACK_LIST_ITER(NULL, iter)
        {
            LV_EXT_LANG_PACK_ITER(iter, lang_pack)
            {
                list_btn = lv_list_add_btn(list1, NULL, LV_EXT_LANG_PACK_ITER_GET_NAME(lang_pack));
                lv_obj_add_flag(list_btn, LV_OBJ_FLAG_CHECKABLE);
                lv_obj_add_event_cb(list_btn, lang_btn_event_handler, LV_EVENT_ALL, 0);
                if (0 == strcmp(LV_EXT_LANG_PACK_ITER_GET_NAME(lang_pack), curr_locale))
                {
                    lv_obj_add_state(list_btn, LV_STATE_CHECKED);
                    p_app_setting->selected_lang = list_btn;
                }
            }
        }
    }
    break;

    case GUI_APP_MSG_ONRESUME:
        break;

    case GUI_APP_MSG_ONPAUSE:
        break;

    case GUI_APP_MSG_ONSTOP:
        rt_kprintf("lang_setting exit\n");
        p_app_setting->selected_lang = NULL;
        break;
    default:
        break;
    }
}


static void btn_lang_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_create_page("lang_setting", lang_subpage_msg_handler);

    }
    if (LV_EVENT_FOCUSED == event)
    {
        //TODO:
        //lv_group_focus_obj(obj);
        //lv_list_focus(obj, LV_ANIM_ON);
    }

}



static void list_display_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        app_setting_display_main();
    }
}

static void list_item_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (lv_obj_check_type(obj, &lv_list_btn_class)) // Align the container object.
        ;//obj = obj->parent;

    if (LV_EVENT_FOCUSED == event)
    {
        //TODO:
        //lv_group_focus_obj(obj);
        //lv_list_focus(obj, LV_ANIM_ON);
    }

}

static void btn_scrtest_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        app_setting_scr_test_main();
    }
}


static void btn_sysinfo_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        app_setting_sys_main();

    }
    if (LV_EVENT_FOCUSED == event)
    {
        //TODO:
        //lv_group_focus_obj(obj);
        //lv_list_focus(obj, LV_ANIM_ON);
    }
}


static void fps_cpu_load_switch_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_VALUE_CHANGED == event)
    {
        //switch display fps and cpu load
        set_display_fps_and_cpu_load(lv_obj_get_state(obj)&LV_STATE_CHECKED);

        if (lv_obj_get_state(obj)&LV_STATE_CHECKED) lv_obj_add_state(obj, LV_STATE_PRESSED);
    }
    if (LV_EVENT_FOCUSED == event)
    {
        //TODO:
        //lv_group_focus_obj(obj);
        //lv_list_focus(obj, LV_ANIM_ON);
    }

}

static void EPIC_switch_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_VALUE_CHANGED == event)
    {
        //switch object refresh mask
        lv_gpu_set_enable(lv_obj_get_state(obj)&LV_STATE_CHECKED);

        if (lv_obj_get_state(obj)&LV_STATE_CHECKED) lv_obj_add_state(obj, LV_STATE_PRESSED);
    }
    if (LV_EVENT_FOCUSED == event)
    {
        //TODO:
        //lv_group_focus_obj(obj);
        //lv_list_focus(obj, LV_ANIM_ON);
    }

}

#if 0
static void btn_bluetooth_event_callback(struct _lv_obj_t *obj, lv_event_t event)
{
    if (LV_EVENT_CLICKED == event)
    {
        p_app_setting->canvas = lv_canvas_create(get_main_win(), NULL);
        lv_canvas_set_buffer(p_app_setting->canvas, canvas_buffer, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR_ALPHA);
        memset(canvas_buffer, 0, sizeof(canvas_buffer));

        p_app_setting->task = lv_task_create(img_animation, 50, LV_TASK_PRIO_MID, (void *)0);
    }
}
#endif

static void add_childrens_to_group(const lv_obj_t *parent)
{
    lv_group_t *list_group = lv_group_get_default();

    for (int32_t id = 0; id < lv_obj_get_child_cnt(parent); id++)
    {
        lv_group_add_obj(list_group, lv_obj_get_child(parent, id));
    }
}

static void rm_childrens_from_group(const lv_obj_t *parent)
{
    lv_group_t *list_group = lv_group_get_default();

    for (int32_t id = 0; id < lv_obj_get_child_cnt(parent); id++)
    {
        lv_group_remove_obj(lv_obj_get_child(parent, id));
    }
}


void app_setting_init(void *param)
{

    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    p_app_setting->main_window = cont;
    lv_obj_set_size(cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);
    lv_obj_update_layout(cont);

    // header
    lv_obj_t *cont_title = lv_lvsfheader_create(cont);
    lv_lvsfheader_set_title(cont_title, LV_EXT_STR_GET_BY_KEY(setting_title, "Setting"));
    lv_lvsfheader_set_visible_item(cont_title, LVSF_HEADER_DEFAULT);
    lv_lvsfheader_back_event_cb(cont_title, btn_close_event_callback);

    // setting list
    lv_obj_t *list1 = lv_list_create(cont);
    lv_obj_set_size(list1, LV_HOR_RES_MAX, LV_VER_RES_MAX - lv_obj_get_height(cont_title));
    lv_obj_align_to(list1, cont_title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    p_app_setting->list1 = list1;
    //TODO:
    //lv_list_set_scrollbar_mode(list1, LV_SCROLLBAR_MODE_OFF);

    /*Add buttons to the list*/
    lv_obj_t *list_btn;

    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(clock_40), LV_EXT_STR_GET_BY_KEY(setting_time, "Time"));
    lv_obj_add_event_cb(list_btn, btn_time_event_callback, LV_EVENT_ALL, 0);

    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(bluetooth), LV_EXT_STR_GET_BY_KEY(setting_language, "Language"));
    lv_obj_add_event_cb(list_btn, btn_lang_event_callback, LV_EVENT_ALL, 0);

    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(dn), LV_EXT_STR_GET_BY_KEY(setting_display, "Display"));
    lv_obj_add_event_cb(list_btn, list_display_event_callback, LV_EVENT_ALL, 0);

    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(setting_airplane_mode, "Airplane Mode"));
    lv_obj_add_event_cb(list_btn, list_item_event_callback, LV_EVENT_ALL, 0);

    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(system_info, "Syttem Info"));
    lv_obj_add_event_cb(list_btn, btn_sysinfo_event_callback, LV_EVENT_ALL, 0);

    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), "Screen Test");
    lv_obj_add_event_cb(list_btn, btn_scrtest_event_callback, LV_EVENT_ALL, 0);


    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(setting_no_disturb, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));
    list_btn = lv_list_add_btn(list1, LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(test_str, "Test"));


    /*add some debug switches to the list*/

    list_btn = lv_list_add_btn(list1, NULL, "FPS & CPU load");
    lv_obj_add_flag(list_btn, LV_OBJ_FLAG_CHECKABLE);
    if (get_display_fps_and_cpu_load())
        lv_obj_add_state(list_btn, LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_add_event_cb(list_btn, fps_cpu_load_switch_event_callback, LV_EVENT_VALUE_CHANGED, 0);


    list_btn = lv_list_add_btn(list1, NULL, "GPU");
    lv_obj_add_flag(list_btn, LV_OBJ_FLAG_CHECKABLE);
#ifdef WIN32
    lv_obj_add_state(list_btn, LV_STATE_DISABLED);
#else
    if (lv_gpu_is_enabled())
        lv_obj_add_state(list_btn, LV_STATE_PRESSED);
#endif
    lv_obj_add_event_cb(list_btn, EPIC_switch_event_callback, LV_EVENT_VALUE_CHANGED, 0);


#ifdef EN_SWITCH_JLINK_DBGUART
    list_btn = lv_list_add_btn(list1, NULL, "Switch DbgUart & H-Jlink");
    lv_obj_add_flag(list_btn, LV_OBJ_FLAG_CHECKABLE);
    if (app_setting_dvlp_get_jlink_status())
        lv_obj_add_state(list_btn, LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_add_event_cb(list_btn, app_setting_dlvp_set_jlink_uart_switch, LV_EVENT_VALUE_CHANGED, 0);
#endif /* EN_SWITCH_JLINK_DBGUART */
}

static void on_start(void)
{
    RT_ASSERT(NULL == p_app_setting);
    p_app_setting = (app_setting_t *) lv_mem_alloc(sizeof(app_setting_t));
    memset(p_app_setting, 0, sizeof(app_setting_t));

    app_setting_init(NULL);

#if 0
    cust_trans_anim_config(CUST_ANIM_TYPE_0, NULL);
#else
    gui_app_trans_anim_t enter_anim_cfg, exit_anim_cfg;

    gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
    gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_NONE);




    gui_app_set_enter_trans_anim(&enter_anim_cfg);
    gui_app_set_exit_trans_anim(&exit_anim_cfg);

    gui_app_set_trans_anim_prio(1, -1);
#endif
}
static void on_resume(void)
{
    add_childrens_to_group(p_app_setting->list1);
}

static void on_pause(void)
{
    rm_childrens_from_group(p_app_setting->list1);
}

static void on_stop(void)
{

    if (p_app_setting)
    {
        //p_app_setting->canvas = NULL;
        if (p_app_setting->task)
        {
            lv_timer_del(p_app_setting->task);
            p_app_setting->task = NULL;
        }

        lv_obj_del(p_app_setting->main_window);
        p_app_setting->main_window = NULL;

        lv_mem_free(p_app_setting);
        p_app_setting = NULL;
    }

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


static int app_main(intent_t i)
{
    const char *active = intent_get_string(i, "active");

    if ((active) && (0 == strncmp(active, "scr_test", sizeof("scr_test"))))
        app_setting_scr_test_main();
    else
        gui_app_regist_msg_handler(APP_ID, msg_handler);

    return 0;
}


BUILTIN_APP_EXPORT(LV_EXT_STR_ID(setting), LV_EXT_IMG_GET(img_settings), APP_ID, app_main);

