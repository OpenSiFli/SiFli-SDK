/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_ARCIMG_H
#define LVSF_ARCIMG_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "lvsf_conf_internal.h"

#if LVSF_USE_ARCIMG != 0

#if LV_USE_IMG == 0
#error "lv_arcimg: lv_img is required. Enable it in lv_conf.h (LV_USE_IMG 1)"
#endif

/**
 * @brief Arc image widget instance data
 */
typedef struct
{
    lv_img_t    base;
    lv_coord_t  cent_x;
    lv_coord_t  cent_y;
    uint16_t    r;
    uint16_t    w;
    float       start_angle;
    float       end_angle;
    float       cur_angle;
    float       draw_angle;
    uint16_t    sram_cnt;
    uint8_t     clockwise;
    uint8_t     *sram_buf;
    uint32_t    sram_size;
    uint8_t     sram_index;
} lv_arcimg_ext_t;

/**
 * @brief Arc image widget class descriptor
 */
extern const lv_obj_class_t lv_arcimg_class;

/**
 * @brief Create an arc image widget
 * @param parent Parent object
 * @return Pointer to the created widget
 */
lv_obj_t *lv_arcimg_create(lv_obj_t *parent);

/**
 * @brief Set arc geometry parameters and SRAM mask buffer count
 * @param arcimg Arc image object
 * @param cent_x Circle center X coordinate
 * @param cent_y Circle center Y coordinate
 * @param r Arc outer radius
 * @param w Arc line width
 * @param buf_cnt Number of SRAM mask buffers
 */
void lv_arcimg_set_param(lv_obj_t *arcimg, lv_coord_t cent_x, lv_coord_t cent_y, uint16_t r, uint16_t w, uint8_t buf_cnt);

/**
 * @brief Set the background arc angle range
 * @param arcimg Arc image object
 * @param start_angle Arc start angle in degrees
 * @param end_angle Arc end angle in degrees
 * @param clockwise Direction flag, 1 for clockwise and 0 for counterclockwise
 */
void lv_arcimg_set_bg_angles(lv_obj_t *arcimg, lv_coord_t start_angle, lv_coord_t end_angle, uint8_t clockwise);

/**
 * @brief Set the current displayed angle with floating-point precision
 * @param arcimg Arc image object
 * @param angle Current arc span angle in degrees, supports floating-point values
 * @param time Animation time in milliseconds,0 means apply immediately
 */
void lv_arcimg_set_angle(lv_obj_t *arcimg, float angle, uint32_t time);

/**
 * @brief Set the current displayed angle using integer input
 * @param arcimg Arc image object
 * @param angle Current arc span angle in degrees as an integer value
 * @param time Animation time in milliseconds,0 means apply immediately
 */
void lv_arcimg_set_angle_int(lv_obj_t *arcimg, lv_coord_t angle, uint32_t time);

/**
 * @brief Get the current displayed angle
 * @param arcimg Arc image object
 * @return Current arc span angle in degrees
 */
float lv_arcimg_get_angle(lv_obj_t *arcimg);

#endif /*LVSF_USE_ARCIMG*/

#ifdef __cplusplus
}
#endif

#endif
