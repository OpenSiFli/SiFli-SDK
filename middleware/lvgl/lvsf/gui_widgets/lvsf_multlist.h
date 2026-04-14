/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 ******************************************************************************
 * @file   lvsf_multlist.h
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _LVSF_MULTLIST_H
#define _LVSF_MULTLIST_H

#include "lvsf_conf_internal.h"

#if LVSF_USE_MULTLIST != 0

#ifdef __cplusplus
extern "C" {
#endif

/*********************
*      INCLUDES
*********************/
#include "lvgl.h"

/*********************
*      DEFINES
*********************/
#define LV_MULTLIST_EDGE_PRO 1024
#define LV_MULTLIST_DRAGE_CNT 8
enum
{
    LV_MULTLIST_FLAG_THROW                  = (1U << 0),  // enable drage throw.
    LV_MULTLIST_FLAG_LOOP                   = (1U << 1),  // enable item loop mode.
    LV_MULTLIST_FLAG_RESIDENCY              = (1U << 2),  // enable item residency mode.that item will not delete even if the item is out off screen
    LV_MULTLIST_FLAG_BEZIER_ALG             = (1U << 3),  // enable bezier algorithm for item zoom.
    LV_MULTLIST_FLAG_ELLIPSE_ALG            = (1U << 4),  // enable ellipse algorithm for item offset.
    LV_MULTLIST_FLAG_EDGE                   = (1U << 5),  // when scrl focus anim end, list will send LV_EVENT_LIST_FOCUS event.
    LV_MULTLIST_FLAG_SCROLLBAR              = (1U << 6),  // when scrl scrolling, list will send LV_EVENT_LIST_SCROLLBAR event to update scrollbar
    LV_MULTLIST_FLAG_BOUNDARY               = (1U << 7),  // when scroll anim end, focus the dividing line between tow item to the center point
    LV_MULTLIST_FLAG_LOCK_SCRL              = (1U << 8),  // Prohibit scr sliding,prohibit scr sliding
    LV_MULTLIST_FLAG_SNAPSHOT               = (1U << 9),  //Dynamically implement photography，items that are not on the screen will be deleted
    LV_MULTLIST_FLAG_SNAPSHOT_ALL           = (1U << 10), //Take photos of all items and keep them without deleting them
    LV_MULTLIST_FLAG_TOW_PAGE               = (1U << 11),
    LV_MULTLIST_FLAG_THREE_PAGE             = (1U << 12),
    LV_MULTLIST_FLAG_INFINTE                = (1U << 13),
    LV_MULTLIST_FLAG_ANIM_BUF               = (1U << 14),
    LV_MULTLIST_FLAG_SHOW_ALL               = (1U << 15), //Alignment method when element length is shorter than the length of the multlist
    LV_MULTLIST_FLAG_ALIGN_HEAD             = (1U << 16), //Alignment method when element length is shorter than the length of the multlist
};
typedef uint32_t lv_multlist_flag_t;
#define LV_MULTLIST_FLAG_PAGE (LV_MULTLIST_FLAG_THREE_PAGE | LV_MULTLIST_FLAG_TOW_PAGE)
enum
{
    LV_EVENT_EDGE_DRAGE_REQ = _LV_EVENT_LAST + 1,
    LV_EVENT_EDGE_DRAGE_RES,
    LV_EVENT_EDGE_DRAGE_BEGIN,
    LV_EVENT_EDGE_DRAGING,
    LV_EVENT_EDGE_DRAGE_END,
    LV_EVENT_EDGE_CHANGE_DIR,
    LV_EVENT_LIST_SCROLL_STRAT,
    LV_EVENT_LIST_SCROLLING,
    LV_EVENT_LIST_SCROLL_END,
    LV_EVENT_LIST_FOCUS,
    LV_EVENT_LIST_FOCUS_LOSS,
    LV_EVENT_LIST_SCROLLBAR,
    LV_EVENT_SWIPE_DELETE,
    LV_EVENT_PAGE_FOUCE,
    LV_EVENT_LIST_LAST
};

enum
{
    LV_EDGE_NONE,
    LV_EDGE_LEFT,
    LV_EDGE_RIGHT,
    LV_EDGE_TOP,
    LV_EDGE_BOTTOM,
};
typedef  uint8_t lv_multlist_edge_type_t;


enum
{
    LV_MULTLIST_ANIM_NONE = 0,
    LV_MULTLIST_ANIM_DEL,
    LV_MULTLIST_ANIM_FLY,
    LV_MULTLIST_ANIM_SALCE,
    LV_MULTLIST_ANIM_ZOOM,
};
typedef  uint8_t lv_multlist_anim_type_t;

enum
{
    LV_MULTLIST_ALIGN_NONE,
    LV_MULTLIST_ALIGN_CENTER,
    LV_MULTLIST_ALIGN_HEAD,
    LV_MULTLIST_ALIGN_TAIL,
} ;
typedef  uint8_t lv_multlist_align_t;

enum
{
    LV_MULTLIST_FLY_LEFT_TOP,
    LV_MULTLIST_FLY_RIGHT_TOP,
    LV_MULTLIST_FLY_LEFT_MID,
    LV_MULTLIST_FLY_RIGHT_MID,
    LV_MULTLIST_FLY_LEFT_BOTTOM,
    LV_MULTLIST_FLY_RIGHT_BOTTOM,
};
typedef  uint8_t lv_multlist_fly_type_t;

/**********************
*      TYPEDEFS
**********************/

typedef struct
{
    int32_t                     process;
    lv_multlist_edge_type_t     trige_type;
    uint8_t                     state;

} lv_multlist_edge_t;

typedef struct
{
    int32_t                     cur_pos;
    uint32_t                    total_len;
    uint32_t                    show_len;
    int16_t                     cur_index;
    uint16_t                    total_num;

} lv_multlist_proc_t;

enum
{
    LV_MULTLIST_DIR_VER,
    LV_MULTLIST_DIR_HOR,
};
typedef uint8_t lv_multlist_dir_t;

typedef struct
{
    lv_obj_t                    *element;
    lv_obj_t                    *snapshot;
    lv_obj_t                    *org_element;
    int32_t                     org_x;
    int32_t                     org_y;
    lv_coord_t                  org_w;
    lv_coord_t                  org_h;
    int32_t                     major_pos;
    lv_coord_t                  major_size;
    lv_coord_t                  minor_pos;
    int32_t                     anim_pos_start;
    int32_t                     anim_pos_end;
    int32_t                     delay;
    void                        *info;
    void                        *user_data;
    void                        *swipe;
    int16_t                     index;
    uint16_t                     flag : 14;
    uint16_t                     ignore : 1;
    uint16_t                     tail : 1;
} lv_multlist_item_t;

typedef void (*lv_multlist_delete_info_cb)(lv_multlist_item_t *);
typedef lv_multlist_delete_info_cb lv_multlist_remove_item_cb;
typedef lv_obj_t *(*lv_multlist_create_item_cb)(lv_obj_t *parent, lv_multlist_item_t *item);
typedef void(*lv_multlist_refresh_cb)(lv_obj_t *multlist, int32_t scrl_pos);
typedef void(*lv_multlist_page_cb)(lv_obj_t *multlist, lv_multlist_item_t *item, int32_t offset);
typedef int(*lv_multlist_tranform_cb)(lv_obj_t *multlist, lv_multlist_item_t *item, int32_t offset, lv_coord_t zoom);
typedef struct
{
    float                       *table;
    float                       pos_para[5];          //algorithm parameter
    float                       neg_para[5];          //algorithm parameter
    uint16_t                    range;               //the range of the bezier
    lv_coord_t                  offset_x;
    lv_coord_t                  offset_y;
    bool                        en_pos;
    bool                        en_neg;
} lv_multlist_bezier_param_t;

typedef struct
{
    uint16_t                    r;
    uint16_t                    virt_r;
    int16_t                     offset_angle;
    int16_t                     start_angle;
    int16_t                     end_angle;
} lv_multlist_ring_param_t;

typedef struct
{
    lv_point_t                  vect;
    lv_point_t                  dist;
    lv_point_t                  dist_abs;
    int32_t                     pre_pos;
    uint32_t                    ref_tick;
    uint32_t                    vect_tick;
    int32_t                     pos;
    lv_coord_t                  offset_x;
    lv_coord_t                  offset_y;
    lv_coord_t                  head_edge;
    lv_coord_t                  tail_edge;
    lv_coord_t                  head_offset;
    lv_coord_t                  tail_offset;
    lv_coord_t                  head_pad;
    lv_coord_t                  tail_pad;
    lv_multlist_align_t         align;
    lv_multlist_align_t         first_align;
    lv_coord_t                  align_offset;
    lv_coord_t                  first_offset;
    lv_coord_t                  first_index;
    uint8_t                     vect_cnt;
} lv_multlist_scrl_t;

typedef struct
{
    lv_obj_t                    *obj;
    uint32_t                    pre_tick;
    int32_t                     cur_vect;
    uint32_t                    interval;
    int32_t                     ratio;
    int32_t                     vect_max;
    uint8_t                     reverse;
} lv_multlist_encoder_t;

typedef struct
{
    uint16_t vect_max;
    uint16_t time_max;
    uint16_t px_per;
} lv_multlist_anim_t;

typedef struct
{
    lv_obj_t obj;
    lv_ll_t item_list;
    lv_multlist_delete_info_cb info_delete_cb;
    lv_multlist_create_item_cb create_cb;
    lv_multlist_remove_item_cb remove_cb;
    lv_multlist_refresh_cb     refresh_cb;
    lv_event_cb_t              snapshot_cb;
    lv_multlist_page_cb        page_anim_cb;
    lv_multlist_tranform_cb    tranform_cb;
    lv_multlist_scrl_t         scrl;
    lv_multlist_bezier_param_t bezier;
    lv_multlist_ring_param_t   ring;
    lv_multlist_encoder_t      encoder;
    lv_multlist_item_t         *focus_item;
    lv_multlist_item_t         *del_item;
    lv_coord_t                 vect_drag;
    lv_multlist_anim_t         anim_para;
    int32_t                    scrl_size;
    uint16_t                   min_pad;
    uint16_t                   max_pad;
    int32_t                    items_dist;
    uint32_t                   flags;
    lv_coord_t                 ellipse_x;
    lv_coord_t                 ellipse_y;
    lv_coord_t                 gap;
    uint16_t                   dis_threshhold;
    uint16_t                   vect_threshhold;
    lv_img_cf_t                snapshot_cf;
    lv_dir_t                   gesture_type;
    uint8_t                    draw_cnt;
    uint8_t                    overlap_cnt;
    lv_multlist_dir_t          scroll_dir;
    uint8_t                    focus_en         : 1;
    uint8_t                    refr_en          : 1;
    uint8_t                    scrolling        : 1;
    uint8_t                    moving           : 1;
} lv_multlist_ext_t;

typedef struct
{
    float throw_rate; //default

} lv_multlist_cfg_t;

extern const lv_obj_class_t lv_multlist_class;
/**********************
*      MACROS
**********************/


/**********************
* GLOBAL PROTOTYPES
**********************/

/**
* Create an heap menu page
* @param par pointer to an object, it will be the parent of the new page
* @param row_cnt the row count only support 1/2/3 row
* @return pointer to the created page
*/
lv_obj_t *lv_multlist_create(lv_obj_t *parent);

/**
* Get touch points gestures
* @param multlist pointer to multlist object
* @return the gesture
*/
lv_dir_t lv_multlist_get_gesture(lv_obj_t *multlist);

/**
* set the flage
* @param multlist pointer to multlist object,
* @return pointer to the created page
*/
int lv_multlist_has_flag(lv_obj_t *multlist, uint32_t flag);

/**
* get the flage state
* @param multlist pointer to multlist object,
* @param flag the state flage,can be LV_MULTLIST_LOOP | LV_MULTLIST_DYNAMIC | LV_MULTLIST_HOR_DIR ...
*/
void lv_multlist_add_flag(lv_obj_t *multlist, uint32_t flag);

/**
* remove the flage from state
* @param multlist pointer to multlist object,
* @param flag the state flage,can be LV_MULTLIST_LOOP | LV_MULTLIST_DYNAMIC | LV_MULTLIST_HOR_DIR ...
*/
void lv_multlist_clear_flag(lv_obj_t *multlist, uint32_t flag);

/**
* refresh items for current srcl pos,when scrl pos has change.
* @param multlist pointer to multlist object,
*/
void lv_multlist_refresh(lv_obj_t *multlist);

/**
* set the srcl pos to the given position.
* @param multlist pointer to multlist object,
* @param position the given position.
*/
void lv_multlist_set_pos(lv_obj_t *multlist, int32_t position);

/**
* set the srcl pos to the given position.
* @param multlist pointer to multlist object,
* @param position the given position.
*/
void lv_multlist_set_snapshot(lv_obj_t *multlist, lv_event_cb_t cb, lv_img_cf_t cf);

/**
* get the srcl pos to the given position.
* @param multlist pointer to multlist object,
* return the srcl current position.
*/
int32_t lv_multlist_get_pos(lv_obj_t *multlist);

/**
* get the srcl pos which the item will align to target align posion.
* @param multlist pointer to multlist object,
* @param align the align type can be one of
        LV_MULTLIST_ALIGN_CENTER/LV_MULTLIST_ALIGN_HEAD/LV_MULTLIST_ALIGN_TAIL
* @param item the item to align,
* return the srcl position.
*/
int32_t lv_multlist_get_focus_pos(lv_obj_t *multlist, lv_multlist_align_t align, lv_multlist_item_t *item);

/**
* set the srcl pos to offset distance and focus the near item or the element based on the direction.
* @param multlist the pointer to multlist object,
* @param offset the relative offset position.
* @param dir_en true :Align to the next element based on the direction of the offset
                false:Align to the nearest element
* @param time the time of animation, 0 for colose animation, other enable animation
*/
void lv_multlist_focus_near(lv_obj_t *multlist, int32_t offset, uint8_t dir_en, uint32_t time);

/**
* Align the given index element to the top with edge spacing.
* @param multlist pointer to multlist object,
* @param index the index of element to align,
* @param edge the edge space from the top,
*/
void lv_multlist_align_head_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset);

/**
* Align the given index element to the tail with edge spacing.
* @param multlist pointer to multlist object,
* @param index the index of element to align,
* @param edge the edge space from the top,
*/
void lv_multlist_align_tail_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset);

/**
* Align the given index element to the center.
* @param multlist pointer to multlist object,
* @param index the index of element to align,
*/
void lv_multlist_align_center_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset);

/**
* Align the given index element to target pos with animation.
* @param multlist pointer to multlist object,
* @param align the align type can be one of LV_MULTLIST_ALIGN_CENTER/LV_MULTLIST_ALIGN_HEAD/LV_MULTLIST_ALIGN_TAIL
* @param index the index of element to align,
* @param offset the offset of element to target pos,
* @param time the time for a anim,
*/
void lv_multlist_align_to(lv_obj_t *multlist, lv_multlist_align_t align, int16_t index, lv_coord_t offset, uint32_t time);

/**
* add item info to list.Afterwards,itemwill be  dynamic loading during the sliding process
* @param multlist the pointer to multlist object,
* @param w the width of item,
* @param h the height of item,
* @param type the type of data used by user self,
* @param user the user data used by user self,
return the instance handle of item
*/
lv_multlist_item_t *lv_multlist_add_info(lv_obj_t *multlist, lv_coord_t w, lv_coord_t h, void *info, void *user_data);

/**
* When the attributes of an element change, a forced refresh is required such as info data
* @param multlist the pointer to multlist object,
* @param item the item to update,Can be obtained through by lv_multlist_get_item_by_index interface
*/
void lv_multlist_updata_element(lv_obj_t *multlist, lv_multlist_item_t *item, bool delete);

/**
* remove item with delete animation
* @param multlist the pointer to multlist object,
* @param delete_item the item to delete
* @param type  the type of remove animation type LV_MULTLIST_ANIM_NONE:for close animation
*             LV_MULTLIST_ANIM_DEL: Directly delete the control
*             LV_MULTLIST_ANIM_FLY: Fly out from the side
*             LV_MULTLIST_ANIM_SALCE: Compressed animation
*             LV_MULTLIST_ANIM_ZOOM:Extended XY axis compression animation
* @param free remove the node and free memory
return 1:remove sucessfully 0:failed
*/
uint32_t lv_multlist_item_remove(lv_obj_t *multlist, lv_multlist_item_t *delete_item, lv_multlist_anim_type_t type, uint8_t free);

/**
* moving the order of item, and animation
* @param multlist the pointer to multlist object,
* @param insert_item the item to insert
* @param item_ref the item to refer
* @param en_anim 1:enable anim 0:disable anim
return 1:remove sucessfully 0:failed
*/
uint32_t lv_multlist_item_move_before(lv_obj_t *multlist, lv_multlist_item_t *insert_item, lv_multlist_item_t *item_ref, uint8_t en_anim);

/**
* insert the order of item
* @param multlist the pointer to multlist object,
* @param insert_item the item to insert
* @param item_ref the item to refer
return 1:remove sucessfully 0:failed
*/
uint8_t lv_multlist_insert_info(lv_obj_t *multlist, lv_multlist_item_t *item, lv_multlist_item_t *ref);

/**
* moving the order of item, and animation
* @param multlist the pointer to multlist object,
* @param type fly type eg.LV_MULTLIST_FLY_LEFT_TOP/LV_MULTLIST_FLY_RIGHT_TOP/LV_MULTLIST_FLY_LEFT_BOTTOM/LV_MULTLIST_FLY_RIGHT_BOTTOM
* @param delay the delay time [0,1024]
* @param time the time animation
* @param fly_in is fly in animation,true for fly, flase for fly out.
* @param ready_cb callback function for animation end.
return NONE
*/
void lv_multlist_fly_anim(lv_obj_t *multlist, lv_multlist_fly_type_t type, uint16_t delay, uint32_t time, uint8_t fly_in, lv_anim_ready_cb_t ready_cb);

/**
* remove all items info which add by lv_multlist_add_info
* @param multlist the pointer to multlist object,
return void
*/
void lv_multlist_remove_info_all(lv_obj_t *multlist);

/**
* Load all items according to the added node information
* @param multlist the pointer to multlist object,
return void
*/
void lv_multlist_load_all_item(lv_obj_t *multlist);

/**
* get the count of info node
* @param multlist the pointer to multlist object,
return info node count
*/
uint32_t lv_multlist_get_info_cnt(lv_obj_t *multlist);

/**
* get the center item
* @param multlist the pointer to multlist object,
return the center item
*/
lv_multlist_item_t *lv_multlist_get_center_item(lv_obj_t *multlist);

/**
* Obtain information about the currently aligned nodes
* @param multlist the pointer to multlist object,
* @param offset   the offset of align point,
return the instance handle of aligned item
*/
lv_multlist_item_t *lv_multlist_get_focus_item(lv_obj_t *multlist, lv_coord_t offset);

/**
* get the instance handle of item by the index
* @param multlist the pointer to multlist object,
* @param index the index of item to get
return the instance handle of item
*/
lv_multlist_item_t *lv_multlist_get_item_by_index(lv_obj_t *multlist, int16_t index);

/**
* set the property of postion which the srcl springback to ,when scrl move beyond range.
* @param multlist pointer to multlist object,
* @param edge_head the head springback postion which the first element offset from top.
* @param edge_tail the tail springback postion which the last element offset from bottom.
*/
void lv_multlist_set_springback(lv_obj_t *multlist, lv_coord_t edge_head, int16_t edge_tail);

/**
* Calculate the head position of the springback area
* @param multlist pointer to multlist object,
* return the head postion
*/
int32_t lv_multlist_get_springback_head(lv_obj_t *multlist);

/**
* Calculate the tail position of the springback area.
* @param multlist pointer to multlist object
* return the tail postion
*/
int32_t lv_multlist_get_springback_tail(lv_obj_t *multlist);
/**
* The offset of the element that can be dragged from edge.
* @param multlist pointer to multlist object,
* @param head the max offset distance which item can be dragged from top.
* @param tail the max offset distance which item can be dragged from bottom.
*/
void lv_multlist_set_scrl_pad(lv_obj_t *multlist, lv_coord_t head, lv_coord_t tail);

/**
* Set the MULTILIST additional area, beyond which the element will be removed, with a default value of 0
* @param multlist pointer to multlist object,
* @param head the head additional area
* @param tail the tail additional area
*/
void lv_multlist_set_show_pad(lv_obj_t *multlist, uint16_t head, uint16_t tail);

/**
* set the gap of tow items.
* @param multlist pointer to multlist object,
* @param gap the gap of tow item.
*/
void lv_multlist_set_gap(lv_obj_t *multlist, uint16_t gap);

/**
* set the gap of tow items.
* @param multlist pointer to multlist object.
* @param offset_x the offset of pivot to conter in x axis
* @param offset_y the offset of pivot to conter in y axis
*/
void lv_multlist_set_pivot_offset(lv_obj_t *multlist, lv_coord_t offset_x, lv_coord_t offset_y);

/**
* Set the threshold for alignment, and start the next page alignment
* when the sliding distance is greater than the specified value or
* the sliding speed is greater than the specified value.
* The default alignment method is sliding trend alignment
* @param multlist pointer to multlist object.
* @param dis the threshold of distance
* @param vect the threshold of speed
*/
void lv_multlist_set_focus_threshold(lv_obj_t *multlist, uint16_t dis, uint16_t vect);

/**
* set the list dir.
* @param multlist pointer to multlist object,
* @param type the type of multllist. only suport LV_MULTLIST_TYPE_ARC/LV_MULTLIST_TYPE_HOR/LV_MULTLIST_TYPE_VER.
*/
void lv_multlist_set_dir(lv_obj_t *multlist, lv_multlist_dir_t dir);

/**
* get the list dir.
* @param multlist pointer to multlist object,
* return LV_MULTLIST_TYPE_HOR/LV_MULTLIST_TYPE_VER/LV_MULTLIST_TYPE_ARC
*/
lv_multlist_dir_t lv_multlist_get_dir(lv_obj_t *multlist);
/**
* when list is in arc dir, can set angles
* @param multlist pointer to multlist object,
* @param offset_angle the angle to rotate all item,
* @param start_angle the start angle to show item ,defaule 0,
* @param end_angle the max angle to show item ,defaule 360,
*/
void lv_multlist_set_angles(lv_obj_t *multlist, int16_t offset_angle, int16_t start_angle, int16_t end_angle);

/**
* when list is in arc dir, can set radius
* @param multlist pointer to multlist object,
* @param r the radius of items are layout
* @param virt_r the virtual radius when drage distance will be convered
*/
void lv_multlist_set_radius(lv_obj_t *multlist, uint16_t r, uint16_t virt_r);

/**
* call this interface will overlap item
* @param multlist pointer to multlist object,
* @param overlap_cnt the count of will be overlaped
*/
void lv_multlist_set_overlap_cnt(lv_obj_t *multlist, uint16_t overlap_cnt);

/**
* Set the alignment of items
* @param multlist pointer to multlist object,
* @param align only suport LV_MULTLIST_TYPE_CIRCLE/LV_MULTLIST_TYPE_HOR/LV_MULTLIST_TYPE_VER.
* @param offset the offset of align point
*/
void lv_multlist_set_align(lv_obj_t *multlist, lv_multlist_align_t align, lv_coord_t offset);

/**
* Set the alignment of items for the first time, when resume
* @param multlist pointer to multlist object,
* @param align only suport LV_MULTLIST_TYPE_CIRCLE/LV_MULTLIST_TYPE_HOR/LV_MULTLIST_TYPE_VER.
* @param offset the offset of align point
* @param index the index of item
*/
void lv_multlist_set_first_align(lv_obj_t *multlist, lv_multlist_align_t align, lv_coord_t offset, int16_t index);


/**
 * @brief   Check if the total height of all children items exceeds the height of the multilist object.
 * @param   multlist    pointer to the multi list object to check
 * @return  true        the total height of items is greater than the height of multlist (items are full/overflow)
 *          false       the total height of items is less than or equal to the height of multlist (enough space)
 * @note
 */
int lv_multlist_is_item_full(lv_obj_t *multlist);

/**
* User defined callback for refresh
* @param multlist pointer to multlist object,
* @param refresh_cb the callback function.
*/
void lv_multlist_set_refresh_cb(lv_obj_t *multlist, lv_multlist_refresh_cb refresh_cb);

/**
* set animation callback function in page mode
* @param multlist pointer to multlist object,
* @param anim_cb the callback function.
*/
void lv_multlist_set_page_anim_cb(lv_obj_t *multlist, lv_multlist_page_cb anim_cb);

/**
* set animation callback function which will be call tranform
* @param multlist pointer to multlist object,
* @param tranform_cb the callback function.
*/
void lv_multlist_set_tranform_cb(lv_obj_t *multlist, lv_multlist_tranform_cb tranform_cb);

/**
* Set relevant callback functions
* @param multlist pointer to multlist object,
* @param create_cb the function will be call when item is created
* @param remove_cb the function will be call when item's obj is deleted
* @param delete_cb the function will be call when item's info is deleted
*/
void lv_multlist_set_item_cb(lv_obj_t *multlist, lv_multlist_create_item_cb create_cb, lv_multlist_remove_item_cb remove_cb, lv_multlist_delete_info_cb delete_cb);

/**
* Set bezier parameters for item zoom
* @param multlist pointer to multlist object,
* @param range the farthest offset from the align point
* @param pos_arr pos parameter for bezier
* @param new_arr neg parameter for bezier
*/
void lv_multlist_set_bezier_para(lv_obj_t *multlist, uint32_t range, float *pos_arr, float *neg_arr);

/**
* Set ellipse parameters for item offset
* @param multlist pointer to multlist object,
* @param range the farthest offset from the align point
* @param x_axis wheelbase in the X direction
* @param y_axis wheelbase in the Y direction
*/
void lv_multlist_set_ellipse_para(lv_obj_t *multlist, lv_coord_t x_axis, lv_coord_t y_axis);

/**
* Set animation parameters
* @param multlist pointer to multlist object,
* @param rate the time of each pixel moves
* @param min_time the shortest duration of animation
* @param max_time the longest duration of animation
*/
void lv_multlist_set_anim_para(lv_obj_t *multlist, lv_coord_t rate, lv_coord_t min_time, lv_coord_t max_time);

/**
* Enable encoder which move items by encoder rotate
* @param multlist pointer to multlist object,
* @param ratio the increase in speed for each rotation of a scale
* @param vect_max maximum speed
* @param reverse if turn the rotation direction
*/
void lv_multlist_enable_encoder(lv_obj_t *multlist, uint32_t ratio, uint32_t vect_max, uint8_t reverse);

/**
* disable encoder
* @param multlist pointer to multlist object
*/
void lv_multlist_disable_encoder(lv_obj_t *multlist);

/**
* call when on pause,this function will remove all show item in screen,and pause refresh
* @param multlist pointer to multlist object
*/
void lv_multlist_on_pause(lv_obj_t *multlist);

/**
* disable encoder
* @param multlist pointer to multlist object
*/
void lv_multlist_on_resume(lv_obj_t *multlist);

/**
* multlist animation interface
* @param multlist pointer to multlist object
* @param start the start value of anim
* @param end the end value of anim
* @param pred animation execution duration
* @param exe_cb animation execution function callback
* @param ready_cb animation end function callback
* @param path_cb animation path function callback
*/
lv_anim_t *lv_multlist_anim(void *multlist, int32_t start, int32_t end, uint32_t pred, lv_anim_exec_xcb_t exe_cb, lv_anim_ready_cb_t ready_cb, lv_anim_path_cb_t path_cb);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LVSF_USE_MULTLIST */

#endif /* _LVSF_MULTLIST_H */
