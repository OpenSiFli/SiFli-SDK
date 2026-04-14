/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef LV_DRAW_EPIC_H
#define LV_DRAW_EPIC_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "../../lv_conf_internal.h"

#if LV_USE_DRAW_EPIC
#include "lv_draw_sw.h"

#include "drv_epic.h"
/*********************
 *      DEFINES
 *********************/
#define GPU_BLEND_EXP_MS     100
#define LV_EPIC_LOG     LV_LOG_INFO
/**********************
 *      TYPEDEFS
 **********************/

typedef lv_layer_t lv_epic_layer_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lv_draw_buf_epic_init_handlers(void);

void lv_draw_epic_init(void);

void lv_draw_epic_deinit(void);

void lv_draw_epic_fill(lv_draw_task_t *draw_task, const lv_draw_fill_dsc_t *dsc,
                       const lv_area_t *coords);

void lv_draw_epic_img(lv_draw_task_t *draw_task, const lv_draw_image_dsc_t *dsc,
                      const lv_area_t *coords);

void lv_draw_epic_layer(lv_draw_task_t *draw_task, const lv_draw_image_dsc_t *draw_dsc,
                        const lv_area_t *coords);

void lv_draw_epic_label(lv_draw_task_t *draw_task, const lv_draw_label_dsc_t *dsc,
                        const lv_area_t *coords);

void lv_draw_epic_border(lv_draw_task_t *draw_task, const lv_draw_border_dsc_t *dsc,
                         const lv_area_t *coords);
void lv_draw_epic_arc(lv_draw_task_t *draw_task, const lv_draw_arc_dsc_t *dsc, const lv_area_t *coords);
/**********************
 *      MACROS
 **********************/
#endif /*LV_USE_DRAW_EPIC*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRAW_EPIC_H*/
