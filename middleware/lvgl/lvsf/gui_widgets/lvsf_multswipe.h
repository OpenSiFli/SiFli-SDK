/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_MULTSIWPE_H
#define _LVSF_MULTSIWPE_H

#include "lvsf_multlist.h"
#include "lvsf_conf_internal.h"

#if LVSF_USE_MULTSWIPE != 0

#ifdef __cplusplus
extern "C" {
#endif

/*********************
*      DEFINES
*********************/

enum
{
    LV_MULTSIWPE_STATE_INIT = 0,
    LV_MULTSWIPE_STATE_DRAGING,
};

enum
{
    LV_MULTSIWPE_STYLE_NONE = 0,
    LV_MULTSWIPE_STYLE_DEFAULT,
    LV_MULTSWIPE_STYLE_WITH_BTN,
    LV_MULTSWIPE_STYLE_WITH_EXPAN,
};
typedef uint8_t lv_multswipe_state;
typedef uint8_t lv_multswipe_style;
typedef void(*process_cb)(lv_multlist_item_t *item, lv_obj_t *content, lv_obj_t *del_btn, int32_t proc);
typedef struct
{
    lv_obj_t            *content;
    lv_obj_t            *del_btn;
    lv_obj_t            *snapshot;
    const void          *src;
    process_cb          process;
    lv_coord_t          thres_val;
    lv_coord_t          springback;
    lv_coord_t          offset;
    lv_multswipe_style  type;
    lv_multswipe_state  state;
    uint16_t            neg_dir  : 1;
    uint16_t            focus_en : 1;
} lv_multswipe_t;


/**
* enable the function of drage delete
* @param item the item of multlist
* @param content the content of item to add cover.
* @param style the style swipe.eg.LV_MULTSWIPE_STYLE_DEFAULT/LV_MULTSWIPE_STYLE_WITH_BTN.
* @return element swiped cover
*/
lv_obj_t *lv_multswipe_enable(lv_multlist_item_t *item, lv_obj_t *content, lv_multswipe_style style);

/**
* set check callbak function to determine which item the edge is used for
* @param item the item of multlist
* @param process the callback function for drag process
* @return none
*/
void lv_multswipe_set_process_cb(lv_multlist_item_t *item, process_cb process);

/**
* set delete botton for remove item,it is only effective for Type LV_MULTSWIPE_STYLE_WITH_BTN
* @param item the item of multlist
* @param del_btn set the delete botton for click delete
* @return none
*/
void lv_multswipe_set_del_btn(lv_multlist_item_t *item, lv_obj_t *del_btn);

/**
* Set delete icon dsc,it is only effective for Type LV_MULTSWIPE_STYLE_WITH_EXPAN
* @param item the item of multlist
* @param del_src the source of a image
* @return none
*/
void lv_multswipe_set_src(lv_multlist_item_t *item, const void *del_src);

/**
* Only when aligned can it be enabled to slide and delete
* @param item the item of multlist
* @param en true or false for enable/disable delete in focus status.
* @return none
*/
void lv_multswipe_enbale_focus(lv_multlist_item_t *item, uint8_t en);

/**
* set threshold value, which the item focus to, or when item drag exceeding this range will be deleted
* @param item the item of multlist
* @param thres_val the threshold value
* @return none
*/
void lv_multswipe_set_thres(lv_multlist_item_t *item, uint16_t thres_val);

/**
* set the max range where the item can de draged  maximum distance
* @param item the item of multlist
* @param springback the max range
* @return none
*/
void lv_multswipe_set_springback(lv_multlist_item_t *item, uint16_t springback);

/**
* set the drag dir
* @param item the item of multlist
* @param dir the direction that can be dragged,
        1 positive direction,
        0 negative direction.
* @return none
*/
void lv_multswipe_set_dir(lv_multlist_item_t *item, uint8_t dir);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LVSF_USE_MULTSWIPE */

#endif /* _LVSF_MULTSWIPE_H */
