/*********************
 *      INCLUDES
 *********************/


#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"


#ifdef LV_USE_DEMO
static void (*entry_func)(void) = NULL;
static void (*exit_func)(void) = NULL;
static void on_start(void)
{
}


static void on_resume(void)
{
    if (entry_func) entry_func();
}

static void on_pause(void)
{
    if (exit_func) exit_func();
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



int lvgl_demo_run(void (*entry)(void), void (*exit)(void))
{
    entry_func = entry;
    exit_func = exit;


    gui_app_create_page("lvgl_demo_run", msg_handler);

    return 0;
}





#endif
