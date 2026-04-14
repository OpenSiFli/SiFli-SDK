/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/

#include "global.h"
#include "demo_multlist.h"
#include "lvsf_multlist.h"
#include "lvsf_txtimg.h"
#define _MODULE_NAME_ "multlist"
#include "app_module.h"

static lv_obj_t *p_list = NULL;
static void demo_set_txt_ext(comm_msg_t *msg);
static void demo_insert_txt_ext(comm_msg_t *msg);

lv_obj_t *app_create_item(lv_obj_t *parent, const char *str, uint16_t index)
{
    lv_obj_t *item_bg = lv_obj_create(parent);
    lv_obj_remove_style_all(item_bg);
    lv_obj_set_style_bg_opa(item_bg, LV_OPA_0, LV_STATE_DEFAULT);
    lv_obj_add_flag(item_bg, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(item_bg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name = lv_txtimg_create(item_bg);
    lv_obj_set_style_radius(name, 12, 0);
    lv_obj_set_style_pad_hor(name, 10, 0);
    lv_obj_set_style_pad_ver(name, 5, 0);
    lv_obj_set_style_bg_color(name, lv_color_make(255, 255, 255), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(name, LV_OPA_100, LV_STATE_DEFAULT);
    lv_obj_set_width(name, LV_HOR_RES_MAX * 3 / 4);
    lv_txtimg_set_txt(name, str);
    lv_obj_add_flag(name, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_coord_t h_self = lv_obj_get_self_height(name) + 40;
    lv_obj_set_size(item_bg, LV_HOR_RES_MAX, h_self);
    lv_obj_refr_size(item_bg);

    if (index % 2)
        lv_obj_align_to(name, item_bg, LV_ALIGN_LEFT_MID, 0, 0);
    else
        lv_obj_align_to(name, item_bg, LV_ALIGN_RIGHT_MID, 0, 0);
    return item_bg;
}

lv_obj_t *app_create_item_cb(lv_obj_t *parent, lv_multlist_item_t *item)
{
    return app_create_item(parent, item->info, item->index);
}

void app_delete_item_cb(lv_multlist_item_t *item)
{
    if (item && item->info)
    {
        rt_free(item->info);
        item->info = NULL;
    }
}

/* When using the solution application architecture, the functions that must be defined */
static void on_start(void)
{
    lv_obj_t *parent = lv_scr_act();
    p_list = lv_multlist_create(parent);
    lv_obj_remove_style_all(p_list);
    lv_obj_set_size(p_list, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(p_list, LV_COLOR_BLACK, 0);
    lv_obj_set_style_bg_opa(p_list, 255, 0);
    lv_obj_center(p_list);
    lv_obj_refr_size(p_list);

    lv_multlist_set_gap(p_list, 0);
    lv_multlist_set_scrl_pad(p_list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);
    lv_multlist_set_dir(p_list, LV_MULTLIST_DIR_VER);

    lv_multlist_set_item_cb(p_list, app_create_item_cb, NULL, app_delete_item_cb);

    lv_multlist_set_springback(p_list, 0, 0);
    comm_msg_t msg;
    msg.data = "you can enter fish cmd to interactive.";
    demo_set_txt_ext(&msg);
    msg.data = "\"demo_insert_txt string \" for adding based on the previous dialogue.";
    demo_set_txt_ext(&msg);
    msg.data = "\"demo_set_txt string \" for set a new dialogue.";
    demo_set_txt_ext(&msg);
    demo_multlist_create_back_button(parent);
}

/* When using the solution application architecture, the functions that must be defined */
static void on_resume(void)
{
    lv_multlist_on_resume(p_list);
    lv_multlist_enable_encoder(p_list, 5, 400, false);

}

/* When using the solution application architecture, the functions that must be defined */
static void on_pause(void)
{
    /* Call the lv_seqframe_pause interface to pause the playback of sequence frames. */
    lv_multlist_on_pause(p_list);
    lv_multlist_disable_encoder(p_list);
}

/* When using the solution application architecture, the functions that must be defined */
static void on_stop(void)
{
    p_list = NULL;
}

/**
 * Register this application into the application framework,
 * on the premise that GUI_APP_FRAMEWORK in menuconfig enables the application architecture of the solution.
 */
APP_PAGE_REGISTER(DEMO_MULTLIST_ID, DEMO_MULTLIST_DIALOGUE_ID, 0);


#if defined(RT_USING_FINSH) &&!defined(PY_GEN)
#include <finsh.h>

static void demo_insert_txt_ext(comm_msg_t *msg)
{
    if (!p_list)return;
    lv_multlist_item_t *item = lv_multlist_get_item_by_index(p_list, -1);
    if (item && item->info)
    {
        uint32_t len0 = strlen(item->info);
        uint32_t len1 = strlen(msg->data);
        char *buf = (char *)rt_malloc(len0 + len1 + 1);
        LV_ASSERT(buf);
        lv_memcpy(buf, item->info, len0);
        lv_memcpy(buf + len0, msg->data, len1 + 1);
        rt_free(item->info);
        item->info = buf;
        if (item->element)
        {
            lv_obj_t *txt = lv_obj_get_child(item->element, -1);
            lv_txtimg_ins_txt(txt, msg->data);
            lv_coord_t w = lv_obj_get_width(txt);
            lv_coord_t h = lv_obj_get_height(txt);
            lv_coord_t w_self = lv_obj_get_self_width(txt);
            lv_coord_t h_self = lv_obj_get_self_height(txt);
            rt_kprintf("w:%d h:%d w_self:%d h_self:%d\n", w, h, w_self, h_self);
            lv_obj_set_height(item->element, lv_obj_get_height(txt));
        }
        lv_multlist_updata_element(p_list, item, false);
        lv_multlist_align_to(p_list, LV_MULTLIST_ALIGN_TAIL, -1, 0, 200);
    }
}

static int demo_insert_txt(int argc, char **argv)
{
    char *str = "it is demo string for inserting";
    if (argc > 1)
        str = argv[1];
    send_msg_to_gui_thread(str, strlen(str) + 1, demo_insert_txt_ext, 0, NEED_WAKEUP_UI);
    return 0;
}
MSH_CMD_EXPORT_REL(demo_insert_txt, demo_insert_txt, demo insert string);

static void demo_set_txt_ext(comm_msg_t *msg)
{
    if (!p_list)return;
    uint32_t len0 = strlen(msg->data);
    char *str = (char *)rt_malloc(len0 + 1);
    LV_ASSERT(str);
    lv_memcpy(str, msg->data, len0 + 1);
    lv_multlist_item_t *item = lv_multlist_add_info(p_list, LV_HOR_RES_MAX, 200, str, NULL);
    lv_multlist_updata_element(p_list, item, true);
    if (item->element)
    {
        lv_obj_t *txt = lv_obj_get_child(item->element, -1);
        lv_coord_t w = lv_obj_get_width(txt);
        lv_coord_t h = lv_obj_get_height(txt);
        lv_coord_t w_self = lv_obj_get_self_width(txt);
        lv_coord_t h_self = lv_obj_get_self_height(txt);
        rt_kprintf("w:%d h:%d w_self:%d h_self:%d\n", w, h, w_self, h_self);
        lv_obj_set_size(txt, w, h);
    }
    lv_multlist_align_to(p_list, LV_MULTLIST_ALIGN_TAIL, -1, 0, 200);
}

static int demo_set_txt(int argc, char **argv)
{
    const char def_str[] = "it is demo string for setting.";
    char *str = (char *)def_str;
    if (argc > 1)
        str = argv[1];
    send_msg_to_gui_thread(str, strlen(str) + 1, demo_set_txt_ext, 0, NEED_WAKEUP_UI);
    return 0;
}

MSH_CMD_EXPORT_REL(demo_set_txt, demo_set_txt, demo set string);

#endif
