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
#include "lvsf_multobj.h"
#include "lvsf_multlist.h"
#define _MODULE_NAME_ "multlist"
#include "app_module.h"
static lv_obj_t *p_list = NULL;

typedef struct
{
    const char *name;
    uint32_t strid;
    uint32_t flag;
} item_info_t;

static item_info_t info_arr[] =
{
    {DEMO_MULTLIST_LIST_ID, app_get_strid(key_list_hor, "horizontal list"), MULTLIST_DIR_HOR << MULTLIST_DIR_OFFSET},
    {DEMO_MULTLIST_LIST_ID, app_get_strid(key_list_ver, "vertical list"), MULTLIST_DIR_VER << MULTLIST_DIR_OFFSET},
    {DEMO_MULTLIST_PAGE_ID, app_get_strid(key_page_hor, "horizontal page"), MULTLIST_DIR_HOR << MULTLIST_DIR_OFFSET},
    {DEMO_MULTLIST_PAGE_ID, app_get_strid(key_page_ver, "vertical page"), MULTLIST_DIR_VER << MULTLIST_DIR_OFFSET},
    {DEMO_MULTLIST_ANIM_ID, app_get_strid(key_anim_hor, "horizontal aniamtion"), MULTLIST_DIR_HOR << MULTLIST_DIR_OFFSET},
    {DEMO_MULTLIST_ANIM_ID, app_get_strid(key_anim_ver, "vertical aniamtion"), MULTLIST_DIR_VER << MULTLIST_DIR_OFFSET},
    {DEMO_MULTLIST_DIALOGUE_ID, app_get_strid(key_list_intercom, "Intercom list")},

};

void demo_item_click_cb(lv_event_t *e)
{
    item_info_t *info = (item_info_t *)lv_event_get_user_data(e);
    if (info && info->name)
    {
        gui_app_run_subpage(DEMO_MULTLIST_ID, info->name, (void *)info->flag);
    }
}

static lv_obj_t *demo_create_item(lv_obj_t *parent, lv_multlist_item_t *item)
{
    item_info_t *info = (item_info_t *)item->info;
    lv_obj_t *item_bg = lv_multobj_create(parent);
    lv_obj_remove_style_all(item_bg);
    lv_obj_set_style_radius(item_bg, 20, 0);
    lv_obj_set_size(item_bg, item->org_w, item->org_h);
    lv_obj_set_style_bg_color(item_bg, lv_color_make(189, 189, 189), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(item_bg, LV_OPA_100, LV_STATE_DEFAULT);
    lv_obj_add_flag(item_bg, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(item_bg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name = lv_label_create(item_bg);
    lv_ext_set_local_font(name, FONT_BIGL, LV_COLOR_WHITE);
    lv_label_set_text(name, app_get_str_from_id(info->strid));
    lv_obj_center(name);
    lv_obj_add_flag(name, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_add_event_cb(item_bg, demo_item_click_cb, LV_EVENT_SHORT_CLICKED, info);
    return item_bg;
}

/* When using the solution application architecture, the functions that must be defined */
static void on_start(void)
{
    p_list = lv_multlist_create(lv_scr_act());
    lv_obj_remove_style_all(p_list);
    lv_obj_set_size(p_list, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(p_list, lv_color_make(82, 93, 118), 0);
    lv_obj_set_style_bg_opa(p_list, LV_OPA_COVER, 0);
    lv_obj_center(p_list);
    lv_obj_refr_size(p_list);

    lv_multlist_set_gap(p_list, DEMO_GAP);
    lv_multlist_set_scrl_pad(p_list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);
    lv_multlist_set_dir(p_list, LV_MULTLIST_DIR_VER);

    lv_multlist_set_item_cb(p_list, demo_create_item, NULL, NULL);

    lv_multlist_set_springback(p_list, DEMO_GAP >> 1, DEMO_GAP >> 1);

    lv_multlist_set_first_align(p_list, LV_MULTLIST_ALIGN_HEAD, -DEMO_GAP >> 1, 0);

    for (uint32_t i = 0; i < sizeof(info_arr) / sizeof(info_arr[0]); i++)
    {
        lv_multlist_add_info(p_list, LV_HOR_RES, 80, (void *)&info_arr[i], NULL);
    }
    lv_obj_set_scrollbar_mode(p_list, LV_SCROLLBAR_MODE_AUTO);
    demo_multlist_create_back_button(lv_scr_act());
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
APPLICATION_REGISTER_PATH(LV_EXT_STR_ID(multlist), APP_GET_IMG(menu_multlist), DEMO_MULTLIST_ID, 0);
