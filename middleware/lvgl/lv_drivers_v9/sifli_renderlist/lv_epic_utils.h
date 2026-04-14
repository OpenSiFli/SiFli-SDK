/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef LV_EPIC_UTILS_H
#define LV_EPIC_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "../../lv_conf_internal.h"

#if LV_USE_DRAW_EPIC
#include "../sw/lv_draw_sw.h"
#include "lv_draw_sw_private.h"

#include "../../stdlib/lv_string.h"


/*********************
 *      DEFINES
 *********************/
/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
uint32_t lv_img_2_epic_cf(lv_color_format_t cf);
EPIC_ColorDef lv_color_to_epic_color(lv_color_t color, lv_opa_t opa);

void lv_epic_get_coord_offset(const lv_draw_task_t *draw_task, int32_t *offset_x, int32_t *offset_y, bool *is_sub_layer);

void lv_epic_area_to_render_coords(const lv_area_t *src_area, int32_t offset_x, int32_t offset_y, lv_area_t *dst_area);

void lv_epic_point_to_render_coords(int32_t src_x, int32_t src_y, int32_t offset_x, int32_t offset_y, int32_t *dst_x, int32_t *dst_y);

bool lv_epic_clip_area_by_render_buf(lv_area_t *area, const drv_epic_render_buf *render_buf);

uint32_t lv_epic_setup_bg_and_output_layer(EPIC_LayerConfigTypeDef *epic_bg_layer,
        EPIC_LayerConfigTypeDef *epic_output_layer,
        lv_draw_task_t *draw_task, const lv_area_t *coords);
void lv_epic_print_area_info(const char *prefix, const lv_area_t *p_area);
void lv_epic_print_layer_info(lv_draw_task_t *draw_task);
/**********************
 *      MACROS
 **********************/
#endif /*LV_USE_DRAW_EPIC*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_EPIC_UTILS_H*/
