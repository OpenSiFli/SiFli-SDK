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
void lv_draw_epic_line(lv_draw_task_t *draw_task, const lv_draw_line_dsc_t *dsc);

bool lv_draw_epic_retain_glyph_data(void *glyph_data);

bool lv_draw_epic_retain_layer_draw_buf(lv_draw_buf_t *draw_buf);

void lv_draw_epic_release_deferred_resources(void);

void render_list_set_display_pixel_align(lv_display_t *disp, uint32_t pixel_align);
drv_epic_render_buf *render_list_get_current_buf(void);

int render_list_create_main_frame(lv_display_t *disp);
int render_list_submit_main_frame(void);
drv_epic_render_list_t render_list_get_current_frame_handle(void);
int render_list_release_by_handle(drv_epic_render_list_t rl);
/**********************
 *      MACROS
 **********************/
#endif /*LV_USE_DRAW_EPIC*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRAW_EPIC_H*/
