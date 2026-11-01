/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_SCROLLBAR_H
#define _LVSF_SCROLLBAR_H

#include "lvgl.h"
#include "lvsf_conf_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

#if LVSF_USE_SCROLLBAR != 0

/*********************
*      INCLUDES
*********************/

/*********************
*      DEFINES
*********************/
/**
 * @brief Square scrollbar type.
 */
#define LV_SCROLLBAR_SQUARE_TYPE 0
/**
 * @brief Circular scrollbar type.
 */
#define LV_SCROLLBAR_CIRCLE_TYPE 1
/**
 * @brief Image-based scrollbar type.
 */
#define LV_SCROLLBAR_IMAGE_TYPE 2

/**********************
*      TYPEDEFS
**********************/

/**********************
*      MACROS
**********************/

/**********************
* GLOBAL PROTOTYPES
**********************/
/**
 * @brief Create a scrollbar object.
 * @param parent Parent object.
 * @param type Scrollbar type, see LV_SCROLLBAR_*_TYPE.
 * @return Pointer to the created scrollbar.
 */
lv_obj_t *lv_scrollbar_create(lv_obj_t *parent, uint16_t type);

/**
 * @brief Update scrollbar progress according to the current scroll status.
 * @param scrollbar Scrollbar object.
 * @param cur_pos Current scroll position.
 * @param ind_len Visible indicator length.
 * @param tatol_len Total content length.
 */
void lv_scrollbar_update(lv_obj_t *scrollbar, lv_coord_t cur_pos, uint16_t ind_len, uint16_t tatol_len);

/**
 * @brief Set scrollbar fade timing.
 * @param parent Scrollbar object or its owner object.
 * @param hold_time Delay before fade-out starts.
 * @param disappear_time Fade-out animation duration.
 */
void lv_scrollbar_set_anim_time(lv_obj_t *parent, uint32_t hold_time, uint32_t disappear_time);

/**
 * @brief Hide or show the scrollbar immediately.
 * @param scrollbar Scrollbar object.
 * @param is_hidden true to hide the scrollbar.
 */
void lv_scrollbar_set_hidden(lv_obj_t *scrollbar, bool is_hidden);

/**
 * @brief Enable or disable always-visible mode.
 * @param scrollbar Scrollbar object.
 * @param is_always_visible true to keep the scrollbar visible.
 */
void lv_scrollbar_set_always_visible(lv_obj_t *scrollbar, bool is_always_visible);

/**
 * @brief Set scrollbar background and indicator colors.
 * @param scrollbar Scrollbar object.
 * @param bg_color Background color.
 * @param ind_color Indicator color.
 */
void lv_scrollbar_set_color(lv_obj_t *scrollbar, lv_color_t bg_color, lv_color_t ind_color);

/**
 * @brief Set scrollbar alignment and offset.
 * @param scrollbar Scrollbar object.
 * @param align Alignment relative to the parent.
 * @param x_ofs Horizontal offset.
 * @param y_ofs Vertical offset.
 */
void lv_scrollbar_set_pos(lv_obj_t *scrollbar, lv_align_t align, lv_coord_t x_ofs, lv_coord_t y_ofs);

#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LVSF_SCROLLBAR_H */
