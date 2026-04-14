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
#define ITEM_CNT 10

#define _MODULE_NAME_ "multlist"
#include "app_module.h"

static lv_obj_t *p_list = NULL;
static uint32_t flag = 0;

static void demo_item_click_cb(lv_event_t *e)
{
    lv_multlist_item_t *item = lv_event_get_user_data(e);
    lv_obj_t *multlist = lv_obj_get_parent(item->org_element);
    int32_t pos = lv_multlist_get_focus_pos(multlist, LV_MULTLIST_ALIGN_CENTER, item);
    int32_t offset = pos - lv_multlist_get_pos(multlist);
    lv_multlist_focus_near(multlist, offset, false, LV_ABS(offset));
}

static void demo_get_item_display_size(const lv_img_dsc_t *src, uint16_t zoom, lv_coord_t *w, lv_coord_t *h)
{
    if(w) *w = 0;
    if(h) *h = 0;

    if(src == NULL) return;

    if(w) *w = (lv_coord_t)LV_MAX(((uint32_t)src->header.w * zoom) / LV_IMG_ZOOM_NONE, 1);
    if(h) *h = (lv_coord_t)LV_MAX(((uint32_t)src->header.h * zoom) / LV_IMG_ZOOM_NONE, 1);
}

int demo_tranform_cb(lv_obj_t *multlist, lv_multlist_item_t *item, int32_t offset, lv_coord_t zoom)
{
    int32_t size_max = LV_HOR_RES;
    const lv_img_dsc_t *src = demo_multlist_get_card_snapshot(DEMO_MULTLIST_CARD_STYLE_ANIM,
                                                               item->index);
    if (MULTLIST_IS_VER(flag))
        size_max = LV_VER_RES;
    int32_t cent_off = offset - (size_max >> 1);
    if (cent_off > -size_max && cent_off < size_max)
    {
        if (!item->element)
        {
            if (!item->org_element)
            {
                item->org_element = lv_img_create(multlist);
                lv_img_set_size_mode(item->org_element, LV_IMG_SIZE_MODE_REAL);
                lv_img_set_src(item->org_element, src);
                lv_obj_add_flag(item->org_element, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_EVENT_BUBBLE);
                lv_obj_add_event_cb(item->org_element, demo_item_click_cb, LV_EVENT_SHORT_CLICKED, item);
            }
            item->element = item->org_element;
            lv_obj_clear_flag(item->element, LV_OBJ_FLAG_HIDDEN);
        }
        int32_t val = lv_map(LV_ABS(cent_off), 0, size_max, 0, 1024);
        val = lv_bezier3(val, 0, 500, 900, 1024);
        lv_coord_t zoom = lv_map(val, 0, 1024, LV_IMG_ZOOM_NONE, LV_MAX(LV_IMG_ZOOM_NONE / 3, 1));
        lv_coord_t opa = lv_map(val, 0, 1024, LV_OPA_100, LV_OPA_10);
        lv_coord_t disp_w;
        lv_coord_t disp_h;
        lv_img_set_zoom(item->element, zoom);
        lv_obj_set_style_img_opa(item->element, opa, 0);
        demo_get_item_display_size(src, zoom, &disp_w, &disp_h);
        if (val < 128)
            lv_obj_move_foreground(item->element);
        val = lv_map(val, 0, 1024, 0, size_max / 2);
        if (cent_off < 0)
            offset = -val + (size_max >> 1);
        else
            offset = val + (size_max >> 1);

        if (MULTLIST_IS_VER(flag))
            lv_obj_set_pos(item->element, (lv_obj_get_width(multlist) - disp_w) >> 1, offset - (disp_h >> 1));
        else
            lv_obj_set_pos(item->element, offset - (disp_w >> 1), (lv_obj_get_height(multlist) - disp_h) >> 1);
    }
    else
    {
        if (item->element)
        {
            lv_obj_add_flag(item->element, LV_OBJ_FLAG_HIDDEN);
            item->element = NULL;
        }
    }
    return 0;
}

/* When using the solution application architecture, the functions that must be defined */
static void on_start(void)
{
    lv_obj_t *parent = lv_scr_act();
    flag = (uint32_t)APP_GET_PAGE_USERDATA_PTR;
    lv_coord_t card_w;
    lv_coord_t card_h;

    demo_multlist_card_cache_init();
    for(uint32_t i = 0; i < demo_multlist_get_image_count(); i++)
    {
        demo_multlist_get_card_snapshot(DEMO_MULTLIST_CARD_STYLE_ANIM, i);
    }
    demo_multlist_get_card_size(DEMO_MULTLIST_CARD_STYLE_ANIM, &card_w, &card_h);

    p_list = lv_multlist_create(parent);
    lv_obj_remove_style_all(p_list);
    lv_obj_set_size(p_list, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(p_list, lv_color_make(82, 93, 118), 0);
    lv_obj_set_style_bg_opa(p_list, LV_OPA_COVER, 0);
    lv_obj_center(p_list);
    lv_obj_refr_size(p_list);
    lv_multlist_add_flag(p_list, LV_MULTLIST_FLAG_LOOP);
    lv_multlist_set_tranform_cb(p_list, demo_tranform_cb);
    lv_multlist_set_gap(p_list, 20);
    lv_multlist_set_scrl_pad(p_list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);
    //lv_multlist_set_item_cb(p_list, demo_create_item, NULL, NULL);

    if (MULTLIST_IS_VER(flag))
    {
        lv_multlist_set_dir(p_list, LV_MULTLIST_DIR_VER);
    }
    else
    {
        lv_multlist_set_dir(p_list, LV_MULTLIST_DIR_HOR);
    }
    for (uint32_t i = 0; i < ITEM_CNT; i++)
    {
        lv_multlist_add_info(p_list, card_w, card_h, NULL, NULL);
    }
    lv_multlist_set_springback(p_list, 0, 0);
    //lv_multlist_align_to(p_list, LV_MULTLIST_ALIGN_HEAD, 0, 0, 0);
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
APP_PAGE_REGISTER(DEMO_MULTLIST_ID, DEMO_MULTLIST_ANIM_ID, 0);
