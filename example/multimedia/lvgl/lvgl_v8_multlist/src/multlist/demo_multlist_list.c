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
#define ITEM_BG_COLOR lv_color_make(82, 93, 118)
#define _MODULE_NAME_ "multlist"
#include "app_module.h"
static lv_obj_t *p_list = NULL;
static uint32_t flag = 0;

static void demo_item_click_cb(lv_event_t *e)
{
    lv_multlist_item_t *item = lv_event_get_user_data(e);
    lv_obj_t *target = lv_event_get_target(e);
    lv_obj_t *multlist = lv_obj_get_parent(target);
    int32_t pos;
    int32_t offset;

    if(item == NULL || multlist == NULL) return;

    pos = lv_multlist_get_focus_pos(multlist, LV_MULTLIST_ALIGN_CENTER, item);
    offset = pos - lv_multlist_get_pos(multlist);
    lv_multlist_focus_near(multlist, offset, false, LV_ABS(offset));
}

static lv_obj_t *demo_create_item(lv_obj_t *parent, lv_multlist_item_t *item)
{
    lv_coord_t card_w;
    lv_coord_t card_h;
    lv_coord_t gap;
    lv_coord_t start_x;
    lv_coord_t start_y;
    lv_obj_t *item_bg = lv_obj_create(parent);
    lv_obj_remove_style_all(item_bg);
    lv_obj_set_size(item_bg, item->org_w, item->org_h);
    lv_obj_set_style_bg_color(item_bg, ITEM_BG_COLOR, LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(item_bg, LV_OPA_100, LV_STATE_DEFAULT);
    lv_obj_add_flag(item_bg, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(item_bg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *item_obj = lv_obj_create(item_bg);
    lv_obj_remove_style_all(item_obj);
    lv_obj_set_style_radius(item_obj, 12, 0);
    lv_obj_set_size(item_obj, item->org_w, item->org_h);
    lv_obj_set_style_bg_color(item_obj, lv_color_make(189, 189, 189), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(item_obj, LV_OPA_100, LV_STATE_DEFAULT);
    lv_obj_add_flag(item_obj, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(item_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(item_obj);

    demo_multlist_get_card_size(DEMO_MULTLIST_CARD_STYLE_LIST, &card_w, &card_h);

    if (MULTLIST_IS_VER(flag))
    {
        gap = LV_MAX((item->org_w - card_w * 3) / 4, 6);
        start_x = LV_MAX((item->org_w - (card_w * 3 + gap * 2)) / 2, 0);
        start_y = LV_MAX((item->org_h - card_h) / 2, 0);
    }
    else
    {
        gap = LV_MAX((item->org_h - card_h * 3) / 4, 6);
        start_x = LV_MAX((item->org_w - card_w) / 2, 0);
        start_y = LV_MAX((item->org_h - (card_h * 3 + gap * 2)) / 2, 0);
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        lv_obj_t *img = lv_img_create(item_obj);
        lv_img_set_src(img, demo_multlist_get_card_snapshot(DEMO_MULTLIST_CARD_STYLE_LIST,
                                                            item->index + i));
        if (MULTLIST_IS_VER(flag))
            lv_obj_set_pos(img, start_x + i * (card_w + gap), start_y);
        else
            lv_obj_set_pos(img, start_x, start_y + i * (card_h + gap));
    }
    lv_obj_update_layout(item_bg);
    return item_bg;
}

/* When using the solution application architecture, the functions that must be defined */
static void on_start(void)
{
    lv_obj_t *parent = lv_scr_act();
    flag = (uint32_t)APP_GET_PAGE_USERDATA_PTR;
    lv_coord_t hor = LV_HOR_RES;
    lv_coord_t ver = LV_VER_RES;
    demo_multlist_card_cache_init();
    for(uint32_t i = 0; i < demo_multlist_get_image_count(); i++)
    {
        demo_multlist_get_card_snapshot(DEMO_MULTLIST_CARD_STYLE_LIST, i);
    }
    p_list = lv_multlist_create(parent);
    lv_obj_remove_style_all(p_list);
    lv_obj_set_size(p_list, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(p_list, lv_color_make(82, 93, 118), 0);
    lv_obj_set_style_bg_opa(p_list, LV_OPA_COVER, 0);
    lv_obj_center(p_list);
    lv_obj_refr_size(p_list);
    lv_multlist_add_flag(p_list, LV_MULTLIST_FLAG_SNAPSHOT);
    lv_multlist_set_snapshot(p_list, demo_item_click_cb, LV_IMG_CF_TRUE_COLOR);

    float para[] = { 0, 0, 0, 0.1f, 0.3f };
    lv_multlist_set_bezier_para(p_list, LV_VER_RES_MAX, para, para);

    lv_multlist_set_gap(p_list, 20);
    lv_multlist_set_scrl_pad(p_list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);
    lv_multlist_set_item_cb(p_list, demo_create_item, NULL, NULL);

    if (MULTLIST_IS_VER(flag))
    {
        lv_multlist_set_dir(p_list, LV_MULTLIST_DIR_VER);
        for (uint32_t i = 0; i < 100; i++)
        {
            lv_multlist_add_info(p_list, hor, 110, NULL, NULL);
        }
    }
    else
    {
        lv_multlist_set_dir(p_list, LV_MULTLIST_DIR_HOR);
        for (uint32_t i = 0; i < 100; i++)
        {
            lv_multlist_add_info(p_list, 110, LV_VER_RES, NULL, NULL);
        }
    }

    lv_multlist_set_springback(p_list, 0, 0);
    lv_multlist_align_to(p_list, LV_MULTLIST_ALIGN_HEAD, 0, 0, 0);
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
    demo_multlist_card_cache_deinit();
    p_list = NULL;
}


/**
 * Register this application into the application framework,
 * on the premise that GUI_APP_FRAMEWORK in menuconfig enables the application architecture of the solution.
 */
APP_PAGE_REGISTER(DEMO_MULTLIST_ID, DEMO_MULTLIST_LIST_ID, 0);
