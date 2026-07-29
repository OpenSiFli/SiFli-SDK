/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtconfig.h"
#include <rtthread.h>
#include "lvsf_resource.h"
#include "mod_installer.h"

#include "gui_app_fwk.h"

LV_IMG_DECLARE(img_red_heart);
LV_FONT_DECLARE(lv_font_montserrat_28_ext);

static void on_start(void)
{
    lv_obj_t *label_obj = lv_label_create(lv_scr_act());
    /* use bitmap font */
    lv_obj_set_style_text_font(label_obj, &lv_font_montserrat_28_ext, 0);
#ifdef LV_USING_EXT_RESOURCE_MANAGER
    lv_label_set_text(label_obj, LV_EXT_STR_GET(LV_EXT_STR_ID(demo_text)));
#else
    lv_label_set_text(label_obj, "This is dlmodule demo.");
#endif
    lv_obj_align(label_obj, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *img_obj = lv_img_create(lv_scr_act());
    lv_img_set_src(img_obj, LV_EXT_IMG_GET(img_red_heart));
    lv_obj_align_to(img_obj, label_obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
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

/**
 * @brief Module initialization function.
 */
static void app_init_func(void)
{
    rt_kprintf("%s create demo page.\n", __func__);
    gui_app_create_page("demo_p", msg_handler);
}

MODULE_INIT_DEF(app_init_func)

MODULE_CLEANUP_DEF(NULL)