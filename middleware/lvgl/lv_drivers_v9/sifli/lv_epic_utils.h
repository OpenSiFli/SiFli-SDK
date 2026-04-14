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
// typedef  int32_t lv_coord_t;
/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
uint32_t lv_img_2_epic_cf(lv_color_format_t cf);
EPIC_ColorDef lv_color_to_epic_color(lv_color_t color, lv_opa_t opa);
uint32_t lv_epic_setup_bg_and_output_layer(EPIC_LayerConfigTypeDef *epic_bg_layer,
        EPIC_LayerConfigTypeDef *epic_output_layer,
        lv_draw_task_t *draw_task, const lv_area_t *coords);
void lv_epic_print_area_info(const char *prefix, const lv_area_t *p_area);
void lv_epic_print_layer_info(lv_draw_task_t *draw_task);
void lv_epic_draw_blend(lv_draw_task_t *draw_task, const lv_draw_sw_blend_dsc_t *blend_dsc);
/**********************
 *      MACROS
 **********************/
#endif /*LV_USE_DRAW_EPIC*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_EPIC_UTILS_H*/
