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
#include "demo_multlist_cards.h"
#include "lvsf_multlist.h"

#define _MODULE_NAME_ "multlist"
#include "app_module.h"
static lv_obj_t *p_list = NULL;
static uint32_t flag = 0;

static lv_obj_t *demo_create_item(lv_obj_t *parent, lv_multlist_item_t *item)
{
    void *scr = item->info;
    lv_obj_t *item_bg = lv_img_create(parent);
    lv_img_set_src(item_bg, scr);
    lv_obj_clear_flag(item_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(item_bg, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    return item_bg;
}

/* When using the solution application architecture, the functions that must be defined */
static void on_start(void)
{
    lv_obj_t *parent = lv_scr_act();
    flag = (uint32_t)APP_GET_PAGE_USERDATA_PTR;
    p_list = lv_multlist_create(parent);
    lv_obj_remove_style_all(p_list);
    lv_obj_set_size(p_list, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(p_list, lv_color_make(82, 93, 118), 0);
    lv_obj_set_style_bg_opa(p_list, LV_OPA_COVER, 0);
    lv_obj_center(p_list);
    lv_obj_refr_size(p_list);
    lv_multlist_add_flag(p_list, LV_MULTLIST_FLAG_LOOP);
    lv_multlist_set_gap(p_list, 0);
    lv_multlist_set_scrl_pad(p_list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);
    if (MULTLIST_IS_VER(flag))
        lv_multlist_set_dir(p_list, LV_MULTLIST_DIR_VER);
    else
        lv_multlist_set_dir(p_list, LV_MULTLIST_DIR_HOR);

    lv_multlist_set_item_cb(p_list, demo_create_item, NULL, NULL);
    lv_multlist_set_springback(p_list, 0, 0);
    //lv_multlist_set_first_align(p_list, LV_MULTLIST_ALIGN_CENTER, 0, 0);

    for (uint32_t i = 0; i < demo_multlist_get_image_count(); i++)
    {
        lv_multlist_add_info(p_list, LV_HOR_RES, LV_VER_RES,
                             (void *)demo_multlist_get_image_src(i), NULL);
    }
    lv_multlist_focus_near(p_list, 0, 0, 0);
    demo_multlist_create_back_button(parent);
}

/* When using the solution application architecture, the functions that must be defined */
static void on_resume(void)
{
    if (!MULTLIST_IS_VER(flag))
        lv_gesture_disable();

    lv_multlist_on_resume(p_list);
    lv_multlist_enable_encoder(p_list, 5, 400, false);
}

/* When using the solution application architecture, the functions that must be defined */
static void on_pause(void)
{
    if (!MULTLIST_IS_VER(flag))
        lv_gesture_enable();

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
APP_PAGE_REGISTER(DEMO_MULTLIST_ID, DEMO_MULTLIST_PAGE_ID, 0);
