/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"

static lv_style_t style_for_opaque;


static void generic_event_callback(lv_obj_t *obj, lv_event_t event)
{
    if (LV_EVENT_CLICKED == event)
        gui_app_self_exit();
}

void payment_init(void)
{
    lv_obj_t *main_window;
    static lv_obj_t *qr;
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);
    char *text = "0123456789012345678901234567890123456789";

    //setup style_for_opaque
    rt_kprintf("payment_init 1\n");
    lv_style_copy(&style_for_opaque, &lv_style_plain);
    style_for_opaque.body.border.opa = LV_OPA_TRANSP;
    style_for_opaque.body.main_color = LV_COLOR_BLACK;
    style_for_opaque.body.grad_color = LV_COLOR_BLACK;
    memset(&style_for_opaque.body.padding, 0, sizeof(style_for_opaque.body.padding));
    style_for_opaque.glass = 1;

    rt_kprintf("payment_init 2\n");
    //create container
    main_window = lv_cont_create(scr, NULL);
    lv_obj_set_style(main_window, &style_for_opaque);
    lv_obj_set_size(main_window, 240, 240);
    lv_obj_align(main_window, scr, LV_ALIGN_CENTER, 0, 0);
    //lv_cont_set_fit(app_clock_container, LV_FIT_FILL);

    rt_kprintf("payment_init 3\n");
    qr = lvsf_qrcode_create(main_window, text, 2);
    rt_kprintf("payment_init 3.1\n");
    lv_obj_align(qr, main_window, LV_ALIGN_CENTER, 0, 0);
    rt_kprintf("payment_init 3.2\n");
    lv_obj_set_event_cb(main_window, generic_event_callback);

    rt_kprintf("payment_init 4\n");
}

static void on_start(void)
{
    rt_kprintf("app_payment on_start\n");
    payment_init();
}

static void on_resume(void)
{
}

static void on_pause(void)
{
}

static void on_stop(void)
{
    rt_kprintf("app_payment on_stop\n");
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


int app_main(int argc, char *argv[])
{
    rt_kprintf("Register app\n");

    gui_app_regist_msg_handler(argv[0], msg_handler);
    rt_kprintf("Finish\n");

    return 0;
}


int module_init(void)
{

    rt_kprintf("module_init\n");
    return 0;
}


int module_cleanup(void)
{

    rt_kprintf("module_cleanup\n");
    return 0;
}


