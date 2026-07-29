/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_FOLLOW_H
#define LVSF_FOLLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#if 1//LVSF_USING_CRASHMENU != 0

#define FOLLOW_TYPE_DEFAULT 0
#define FOLLOW_TYPE_SIMILAR 1
#define FOLLOW_TYPE_STANDARDS 2

#define FOLLOW_STATUS_NORMAL    (1U << 0)
#define FOLLOW_STATUS_ORDER     (1U << 1)
#define FOLLOW_STATUS_CHANGE    (1U << 2)
#define FOLLOW_STATUS_EDIT      (1U << 3)

#define MMENU_ICON_LAYER_CNT 3

struct lv_follow_item_info;
typedef struct lv_follow_item_info lv_follow_item_info_t;
typedef struct lv_follow_item_draw lv_follow_item_draw_t;

typedef lv_obj_t *(*lv_follow_create_item_cb)(lv_obj_t *parent, uint16_t index, uint16_t type, void *user_data);
typedef lv_obj_t *(*lv_follow_set_border_cb)(lv_obj_t *parent, uint16_t index, void *user_data);
typedef void(*lv_follow_delete_info_cb)(lv_follow_item_info_t          *item);

typedef struct
{
    float gravity;
    float friction;
    lv_coord_t opa_r;
    lv_coord_t opa_min;
    uint16_t v_max;
    uint8_t margin;
    uint8_t collision_type;
    int16_t icon_r;
    float hor_rate;
    float ver_rate;
    float speed_ratio;
    float black_ratio;
    int16_t square_r; //Square screen fillet radius
    int16_t target_r[MMENU_ICON_LAYER_CNT];
    int16_t offset_r[MMENU_ICON_LAYER_CNT]; //The center point of the image in polar coordinates for each layer
    int16_t start_angle[MMENU_ICON_LAYER_CNT]; //The start angle of icon for each layer
    int16_t gap_angle[MMENU_ICON_LAYER_CNT];
    bool custom_align;
    bool is_square;
} lv_follow_cfg_t;

struct lv_follow_item_draw
{
    lv_obj_t *icon;
    lv_obj_t *border;
    float v_x;
    float v_y;
    lv_coord_t start_r;
    lv_coord_t target_r;
    lv_coord_t start_angle;
    lv_coord_t target_angle;
    lv_coord_t target_x;
    lv_coord_t target_y;
    lv_coord_t r;
    int16_t layer;
    lv_follow_item_info_t *item_info;
};

struct lv_follow_item_info
{
    uint16_t index;
    uint16_t type;
    void *user_data;
    lv_follow_item_draw_t *item_draw;
};

typedef struct
{
    lv_obj_t obj;
    lv_ll_t item_info;
    lv_ll_t item_draw;
    lv_obj_t *mask;
    lv_point_t pre_point;
    float g_r;
    float g_x;
    float g_y;
    uint16_t resv_item_cnt;
    lv_follow_item_draw_t *other_item;
    lv_follow_item_draw_t *press_item;
    lv_follow_item_info_t *next_item;
    lv_follow_create_item_cb create_item_cb;
    lv_follow_delete_info_cb delete_info_cb;
    lv_follow_set_border_cb draw_border_cb;
    lv_timer_t *refresh_task;
    lv_follow_cfg_t cfg;
    uint8_t status;
    uint8_t dis_status;
} lv_follow_ext_t;


lv_obj_t *lv_follow_create(lv_obj_t *parent);

void lv_follow_add_item_info(lv_obj_t *crashmenu, uint16_t type, void *user_data);

void lv_follow_set_gravity(lv_obj_t *crashmenu, float g, lv_coord_t angle);

lv_follow_cfg_t *lv_follow_get_cfg_param(lv_obj_t *crashmenu);

void lv_follow_disable_status(lv_obj_t *crashmenu, uint8_t status);

uint8_t lv_follow_get_status(lv_obj_t *crashmenu);

void lv_follow_set_item_cb(lv_obj_t *crashmenu, lv_follow_create_item_cb create_cb, lv_follow_delete_info_cb delete_cb);

void lv_follow_enter_order_status(lv_obj_t *crashmenu);

void lv_follow_enter_normal_status(lv_obj_t *crashmenu);

void lv_follow_enter_change_status(lv_obj_t *crashmenu);

void lv_follow_enter_edit_status(lv_obj_t *crashmenu, int8_t layer);

void lv_follow_on_start(lv_obj_t *parent);

void lv_follow_on_resume(lv_obj_t *parent);

void lv_follow_on_pause(lv_obj_t *parent);

void lv_follow_on_stop(lv_obj_t *parent);

#endif /*LVSF_FOLLOW_H*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif