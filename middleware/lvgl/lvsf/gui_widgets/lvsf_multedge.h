/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_MULTEDGE_H
#define _LVSF_MULTEDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#if LVSF_USE_MULTEDGE != 0

#include "lvsf_multlist.h"

/*********************
*      INCLUDES
*********************/

/*********************
*      DEFINES
*********************/

enum
{
    LV_MULTEDGE_HIDDEN = 0,
    LV_MULTEDGE_MOVING,
    LV_MULTEDGE_END,
    LV_MULTEDGE_GOBACK
};

typedef int(*lv_multedge_check_item)(lv_multlist_item_t *item);

typedef struct
{
    lv_obj_t obj;
    lv_multlist_edge_type_t type;
    lv_multedge_check_item check_cb;
    int32_t process;
    uint16_t state;
    lv_coord_t start_pos;
    lv_coord_t end_pos;
    lv_coord_t start_thres;
    lv_coord_t end_thres;
    int16_t  index;
} lv_multedge_ext_t;

extern const lv_obj_class_t lv_multedge_class;

/**
* set the flage
* @param parent the parent of multedge to be created.
* @return the created multedge
*/
lv_obj_t *lv_multedge_create(lv_obj_t *parent);

/**
* set the flage
* @param edge the type of edge
            LV_EDGE_LEFT,
            LV_EDGE_RIGHT,
            LV_EDGE_TOP,
            LV_EDGE_BOTTOM,
* @return none
*/
void lv_multedge_set_type(lv_obj_t *edge, lv_multlist_edge_type_t type);

/**
* set check callbak function to determine which item the edge is used for
* @param edge the type of edge.
* @param check_cb check item callback function.
* @return none
*/
void lv_multedge_set_check_cb(lv_obj_t *edge, lv_multedge_check_item check_cb);

/**
* set the index if item which the edge is used for
* @param edge the type of edge.
* @param index the index of item.
* @return none
*/
void lv_multedge_set_item_index(lv_obj_t *edge, int16_t index);

/**
* set threshold where the edge unfold and fold
* @param edge the type of edge.
* @param start the type of edge.
* @param end the type of edge.
* @return none
*/
void lv_multedge_set_threshold(lv_obj_t *edge, lv_coord_t start, lv_coord_t end);

/**
* get the state of edge
* @param edge the type of edge.
* @return the state of edge
 eg.LV_MULTEDGE_HIDDEN when edge is hidden,out of screen.
    LV_MULTEDGE_MOVING,when edge is moveing to end.
    LV_MULTEDGE_END,,when edge is puout screen.
    LV_MULTEDGE_GOBACK
*/
uint16_t lv_multedge_get_state(lv_obj_t *edge);

#endif

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif
