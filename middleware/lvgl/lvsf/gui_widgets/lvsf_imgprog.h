/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_IMGPROG_H
#define LVSF_IMGPROG_H

#include "lvgl.h"
#include "lvsf_conf_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#if LVSF_USE_IMGPROG != 0

#if LV_USE_IMG == 0
#error "lv_imgprog: lv_img is required. Enable it in lv_conf.h (LV_USE_IMG 1)"
#endif

/**
 * @brief LVGL class descriptor for the image progress widget.
 */
extern const lv_obj_class_t lv_imgprog_class;

/**
 * @brief Horizontal alignment of the image layers.
 */
typedef enum
{
    LV_ARC_PROGRESS_ALIGN_LEFT = 0,  /**< Align image layers to the left side. */
    LV_ARC_PROGRESS_ALIGN_RIGHT,     /**< Align image layers to the right side. */
} lv_imgprog_align_t;

/**
 * @brief Configuration parameters for an image progress widget.
 * @note Angles are expressed in tenths of a degree (0.1 degree).
 */
typedef struct
{
    const void *bg_src;       /**< Background image source. */
    const void *indic_src;    /**< Indicator image source. */
    const void *mask_src;     /**< Optional mask image source. */
    int16_t start_angle;      /**< Progress start angle in 0.1 degree. */
    int16_t end_angle;        /**< Progress end angle in 0.1 degree. */
    int16_t indic_angle;      /**< Indicator image angular width in 0.1 degree. */
    int16_t min_value;        /**< Minimum progress value. */
    int16_t max_value;        /**< Maximum progress value. */
    int16_t default_value;    /**< Initial value, clamped to the configured range. */
    lv_imgprog_align_t align; /**< Horizontal alignment of the image layers. */
    lv_coord_t offset;        /**< Horizontal offset from the selected side. */
    lv_coord_t radius;        /**< Widget radius in pixels. */
} lv_imgprog_cfg_t;

/**
 * @brief Image progress widget instance data.
 */
typedef struct
{
    lv_obj_t obj;             /**< Base LVGL object. */
    lv_obj_t *bg_img;         /**< Background image object. */
    lv_obj_t *indic_img;      /**< Rotating indicator image object. */
    lv_obj_t *mask_img;       /**< Optional mask image object. */
    lv_imgprog_cfg_t cfg;     /**< Effective widget configuration. */
    int16_t curr_angle;       /**< Current progress angle in 0.1 degree. */
    int16_t value;             /**< Current progress value. */
} lv_imgprog_t;

/**
 * @brief Get the process-wide default image progress configuration.
 * @return Pointer to the mutable default configuration.
 */
const lv_imgprog_cfg_t *lv_imgprog_get_default_cfg(void);

/**
 * @brief Set the process-wide default image progress configuration.
 * @param cfg Configuration to copy; ignored when NULL.
 */
void lv_imgprog_set_default_cfg(const lv_imgprog_cfg_t *cfg);

/**
 * @brief Create an image progress widget.
 * @param parent Parent object. May be NULL when supported by LVGL.
 * @return Pointer to the newly created widget.
 */
lv_obj_t *lv_imgprog_create(lv_obj_t *parent);

/**
 * @brief Apply a complete configuration to an image progress widget.
 * @param arc_progress Image progress widget.
 * @param cfg Configuration to copy; ignored when NULL.
 * @note Angles are normalized and values are clamped to the configured range.
 */
void lv_imgprog_set_cfg(lv_obj_t *arc_progress, const lv_imgprog_cfg_t *cfg);

/**
 * @brief Set the background, indicator, and mask image sources.
 * @param arc_progress Image progress widget.
 * @param bg_src Background image source, or NULL to clear it.
 * @param indic_src Indicator image source, or NULL to clear it.
 * @param mask_src Mask image source, or NULL to clear it.
 */
void lv_imgprog_set_images(lv_obj_t *arc_progress, const void *bg_src, const void *indic_src, const void *mask_src);

/**
 * @brief Set the background image source.
 * @param arc_progress Image progress widget.
 * @param bg_src Background image source, or NULL to clear it.
 */
void lv_imgprog_set_bg_src(lv_obj_t *arc_progress, const void *bg_src);

/**
 * @brief Set the indicator image source.
 * @param arc_progress Image progress widget.
 * @param indic_src Indicator image source, or NULL to clear it.
 */
void lv_imgprog_set_indicator_src(lv_obj_t *arc_progress, const void *indic_src);

/**
 * @brief Set the mask image source.
 * @param arc_progress Image progress widget.
 * @param mask_src Mask image source, or NULL to clear it.
 */
void lv_imgprog_set_mask_src(lv_obj_t *arc_progress, const void *mask_src);

/**
 * @brief Set the progress start and end angles.
 * @param arc_progress Image progress widget.
 * @param start_angle Start angle in 0.1 degree.
 * @param end_angle End angle in 0.1 degree.
 */
void lv_imgprog_set_angles(lv_obj_t *arc_progress, int16_t start_angle, int16_t end_angle);

/**
 * @brief Set the angular width of the indicator image.
 * @param arc_progress Image progress widget.
 * @param indic_angle Indicator angular width in 0.1 degree; negative values become zero.
 */
void lv_imgprog_set_indicator_angle(lv_obj_t *arc_progress, int16_t indic_angle);

/**
 * @brief Set the minimum and maximum progress values.
 * @param arc_progress Image progress widget.
 * @param min_value Minimum progress value.
 * @param max_value Maximum progress value. The two values are reordered when necessary.
 */
void lv_imgprog_set_range(lv_obj_t *arc_progress, int16_t min_value, int16_t max_value);

/**
 * @brief Set the horizontal alignment of the image layers.
 * @param arc_progress Image progress widget.
 * @param align Alignment side; unsupported values default to left alignment.
 */
void lv_imgprog_set_align(lv_obj_t *arc_progress, lv_imgprog_align_t align);

/**
 * @brief Set the horizontal offset of the image layers.
 * @param arc_progress Image progress widget.
 * @param offset Offset from the selected alignment side in pixels.
 */
void lv_imgprog_set_offset(lv_obj_t *arc_progress, lv_coord_t offset);

/**
 * @brief Set the widget radius.
 * @param arc_progress Image progress widget.
 * @param radius Widget radius in pixels.
 */
void lv_imgprog_set_radius(lv_obj_t *arc_progress, lv_coord_t radius);

/**
 * @brief Set the current progress value.
 * @param arc_progress Image progress widget.
 * @param value Progress value, clamped to the configured range.
 */
void lv_imgprog_set_value(lv_obj_t *arc_progress, int16_t value);

/**
 * @brief Set the opacity of all image layers.
 * @param arc_progress Image progress widget.
 * @param opa Image opacity.
 */
void lv_imgprog_set_opa(lv_obj_t *arc_progress, lv_opa_t opa);

/**
 * @brief Set the current progress from an angle.
 * @param arc_progress Image progress widget.
 * @param angle Current angle in 0.1 degree.
 * @note The angle is normalized and clamped to the configured progress arc.
 */
void lv_imgprog_set_value_by_angle(lv_obj_t *arc_progress, int16_t angle);

/**
 * @brief Get the background image object.
 * @param arc_progress Image progress widget.
 * @return Background image object, or NULL when unavailable.
 */
lv_obj_t *lv_imgprog_get_bg_img(lv_obj_t *arc_progress);

/**
 * @brief Get the indicator image object.
 * @param arc_progress Image progress widget.
 * @return Indicator image object, or NULL when unavailable.
 */
lv_obj_t *lv_imgprog_get_indicator_img(lv_obj_t *arc_progress);

/**
 * @brief Get the mask image object.
 * @param arc_progress Image progress widget.
 * @return Mask image object, or NULL when unavailable.
 */
lv_obj_t *lv_imgprog_get_mask_img(lv_obj_t *arc_progress);

/**
 * @brief Get the current progress value.
 * @param arc_progress Image progress widget.
 * @return Current progress value.
 */
int16_t lv_imgprog_get_value(lv_obj_t *arc_progress);

/**
 * @brief Get the current progress angle.
 * @param arc_progress Image progress widget.
 * @return Current angle in 0.1 degree.
 */
int16_t lv_imgprog_get_value_angle(lv_obj_t *arc_progress);

/**
 * @brief Get the effective widget configuration.
 * @param arc_progress Image progress widget.
 * @return Pointer to the widget's internal configuration.
 */
const lv_imgprog_cfg_t *lv_imgprog_get_cfg(lv_obj_t *arc_progress);

#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
