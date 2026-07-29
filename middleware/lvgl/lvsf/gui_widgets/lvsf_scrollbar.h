/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_SCROLLBAR_H
#define _LVSF_SCROLLBAR_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef LVSF_USE_SCROLLBAR

/*********************
*      INCLUDES
*********************/

/*********************
*      DEFINES
*********************/
#define LV_SCROLLBAR_SQUARE_TYPE 0
#define LV_SCROLLBAR_CIRCLE_TYPE 1
/**********************
*      TYPEDEFS
**********************/

/**********************
*      MACROS
**********************/

/**********************
* GLOBAL PROTOTYPES
**********************/

lv_obj_t *lv_scrollbar_create(lv_obj_t *parent, uint16_t type);

void lv_scrollbar_update(lv_obj_t *scrollbar, lv_coord_t cur_pos, uint16_t ind_len, uint16_t tatol_len);

void lv_scrollbar_set_anim_time(lv_obj_t *parent, uint32_t hold_time, uint32_t disappear_time);

void lv_scrollbar_set_hidden(lv_obj_t *scrollbar, bool is_hidden);

void lv_scrollbar_set_always_visible(lv_obj_t *scrollbar, bool is_always_visible);

#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LVSF_MULTLIST_H */
