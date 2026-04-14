/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/*********************
 *      INCLUDES
 *********************/

#include "lv_draw_epic.h"

#if LV_USE_DRAW_EPIC
#include "lv_epic_utils.h"
#include "../../misc/lv_area_private.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_draw_epic_line(lv_draw_task_t *draw_task, const lv_draw_line_dsc_t *dsc)
{
    if (dsc->width == 0) return;
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN) return;

    lv_point_t p1 = lv_point_from_precise(&dsc->p1);
    lv_point_t p2 = lv_point_from_precise(&dsc->p2);
    if (p1.x == p2.x && p1.y == p2.y) return;

    drv_epic_render_buf *render_buf = render_list_get_current_buf();
    if (render_buf == NULL)
    {
        LV_LOG_ERROR("EPIC: No render buffer available for line drawing\n");
        return;
    }

    lv_layer_t *layer = draw_task->target_layer;
    lv_area_t clip_line_abs;
    clip_line_abs.x1 = LV_MIN(p1.x, p2.x) - dsc->width / 2;
    clip_line_abs.x2 = LV_MAX(p1.x, p2.x) + dsc->width / 2;
    clip_line_abs.y1 = LV_MIN(p1.y, p2.y) - dsc->width / 2;
    clip_line_abs.y2 = LV_MAX(p1.y, p2.y) + dsc->width / 2;

    if (!lv_area_intersect(&clip_line_abs, &clip_line_abs, &draw_task->clip_area)) return;
    if (layer && !lv_area_intersect(&clip_line_abs, &clip_line_abs, &layer->buf_area)) return;

    int32_t coord_offset_x = 0;
    int32_t coord_offset_y = 0;
    bool is_sub_layer = false;
    lv_epic_get_coord_offset(draw_task, &coord_offset_x, &coord_offset_y, &is_sub_layer);

    lv_area_t clip_line_local;
    lv_epic_area_to_render_coords(&clip_line_abs, coord_offset_x, coord_offset_y, &clip_line_local);
    if (!lv_epic_clip_area_by_render_buf(&clip_line_local, render_buf))
    {
        LV_LOG("EPIC: Line skipped after coord conversion (%s), area=[%d,%d,%d,%d]\n",
               is_sub_layer ? "sub-layer" : "main-layer",
               clip_line_local.x1, clip_line_local.y1, clip_line_local.x2, clip_line_local.y2);
        return;
    }

    int32_t p1_x_local = 0;
    int32_t p1_y_local = 0;
    int32_t p2_x_local = 0;
    int32_t p2_y_local = 0;
    lv_epic_point_to_render_coords(p1.x, p1.y, coord_offset_x, coord_offset_y, &p1_x_local, &p1_y_local);
    lv_epic_point_to_render_coords(p2.x, p2.y, coord_offset_x, coord_offset_y, &p2_x_local, &p2_y_local);

    drv_epic_operation *op = drv_epic_alloc_op(render_buf);
    if (op == NULL)
    {
        LV_LOG_ERROR("EPIC: Failed to allocate line operation\n");
        return;
    }

    op->op = DRV_EPIC_DRAW_LINE;
    op->clip_area.x0 = clip_line_local.x1;
    op->clip_area.y0 = clip_line_local.y1;
    op->clip_area.x1 = clip_line_local.x2;
    op->clip_area.y1 = clip_line_local.y2;

    op->desc.line.p1.x = (int16_t)p1_x_local;
    op->desc.line.p1.y = (int16_t)p1_y_local;
    op->desc.line.p2.x = (int16_t)p2_x_local;
    op->desc.line.p2.y = (int16_t)p2_y_local;
    op->desc.line.width = (uint16_t)dsc->width;
    op->desc.line.dash_width = dsc->dash_width;
    op->desc.line.dash_gap = dsc->dash_gap;
    op->desc.line.round_start = dsc->round_start ? 1 : 0;
    op->desc.line.round_end = dsc->round_end ? 1 : 0;
    op->desc.line.raw_end = dsc->raw_end ? 1 : 0;
    op->desc.line.argb8888 = ((uint32_t)dsc->opa << 24)
                           | ((uint32_t)dsc->color.red << 16)
                           | ((uint32_t)dsc->color.green << 8)
                           | (uint32_t)dsc->color.blue;

    drv_epic_commit_op(op);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif /*LV_USE_DRAW_EPIC*/
