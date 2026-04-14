/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_MULTOBJ_H
#define LVSF_MULTOBJ_H

#include "lvsf_conf_internal.h"

#if LVSF_USE_MULTOBJ != 0

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

typedef void(*lv_multobj_label_pos_cb)(lv_obj_t *label, float zoom, lv_coord_t x, lv_coord_t y);
typedef struct
{
    lv_obj_t *obj;
    lv_obj_t *snapshort;
    lv_coord_t h;
    lv_coord_t w;
    lv_coord_t x;
    lv_coord_t y;
    lv_coord_t radius;
    lv_ll_t list;
    lv_coord_t ori_opa;
    uint16_t type;
    uint16_t zoom;
} lv_obj_hier_t;


typedef struct
{
    lv_obj_t obj_base;
    float zoom;
    float pre_zoom;
    lv_obj_hier_t hier;
    lv_multobj_label_pos_cb label_cb;
    uint8_t is_snapshot : 1;
    uint8_t is_initial : 1;
    uint8_t hor_align : 3;
    uint8_t ver_align : 3;
} lv_multobj_ext_t;

enum
{
    LABEL_ALIGN_LEFT = 0,
    LABEL_ALIGN_UP = 0,
    LABEL_ALIGN_CENTER = 1,
    LABEL_ALIGN_RIGHT = 2,
    LABEL_ALIGN_DOWN = 2,
};
typedef uint8_t lv_label_align_type;

extern const lv_obj_class_t lv_multobj_class;

/*
create a multobj
    * @param multobj pointer to parent
    * @param copy set to NULL
    * return none
*/
lv_obj_t *lv_multobj_create(lv_obj_t *parent);

/*
Scale OBJ according to saved hierarchy
    * @param multobj pointer to multobj
    * @param zoom Scaling factor
    [0,1)   make smaller
    1       Do not shrink
    (1,x)   Proportional amplification
    * return none
*/
void lv_multobj_set_zoom(lv_obj_t *multobj, float zoom);

/*
reset the hierarchical structure and location information of child multobj
    * @param obj pointer to multobj
    * return the hierarchical structure
*/
void lv_multobj_reset_hier(lv_obj_t *multobj);

/*
enable snapshot the label in multobj
    * @param obj pointer to multobj
    * @param is_enable true:for enable
    * return void
*/
void lv_multobj_set_snapshot(lv_obj_t *multobj, bool is_enable);

/*
    register a callback function for label
    * @param obj pointer to multobj
    * @param is_enable true:for enable
    * return void
*/
void lv_multobj_set_label_pos_cb(lv_obj_t *multobj, lv_multobj_label_pos_cb callback);

/*
    set label default align type
    * @param obj pointer to multobj
    * @param hor_align horizontal alignment method
    * @param ver_align vertical alignment method
    * return void
*/
void lv_multobj_set_label_align(lv_obj_t *multobj, lv_label_align_type hor_align, lv_label_align_type ver_align);

#ifdef __cplusplus
}
#endif

#endif /* LVSF_USE_MULTOBJ */

#endif /* LVSF_MULTOBJ_H */

