#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "custom_trans_anim.h"

extern void print_sysinfo(char *buf, uint32_t buf_len);


static void back_btn_event_callback(lv_event_t *e)
{
    //lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();

    }
}

static void on_start(void)
{
    char *buf;

#define BUF_SIZE  (512)

    // header
    lv_obj_t *title = lv_lvsfheader_create(lv_scr_act());
    lv_lvsfheader_set_title(title, "System Info");
    lv_lvsfheader_set_visible_item(title, LVSF_HEADER_BRANCH);
    lv_lvsfheader_back_event_cb(title, back_btn_event_callback);


    lv_obj_t *page = lv_obj_create(lv_scr_act());
    lv_obj_set_size(page, LV_HOR_RES, LV_VER_RES - lv_obj_get_height(title) - 10);
    lv_obj_align_to(page, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_obj_t *lbl_sys_info = lv_label_create(page);
    lv_obj_align_to(lbl_sys_info, title, LV_ALIGN_OUT_BOTTOM_LEFT, 30, 10);
    //lv_obj_set_size(lbl_sys_info, LV_HOR_RES, LV_VER_RES - lv_obj_get_height(title));

    buf = lv_mem_alloc(BUF_SIZE);
    LV_ASSERT_MALLOC(buf);

#ifndef _WIN32
    print_sysinfo(buf, BUF_SIZE);
#endif

    lv_label_set_text(lbl_sys_info, buf);
    lv_mem_free(buf);



    cust_trans_anim_config(CUST_ANIM_TYPE_2, NULL);
}

static void on_resume(void)
{

}

static void on_pause(void)
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

void app_setting_sys_main(void)
{
    gui_app_create_page("sysinfo", msg_handler);
}
