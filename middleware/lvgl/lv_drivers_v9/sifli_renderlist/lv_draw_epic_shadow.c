/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lv_draw_epic.h"

#if LV_USE_DRAW_EPIC
#include "lv_epic_utils.h"
#include "../lv_draw_private.h"
#include "../../misc/lv_area_private.h"

void lv_draw_epic_shadow(lv_draw_task_t *draw_task, const lv_draw_box_shadow_dsc_t *dsc,
                         const lv_area_t *coords)
{
    if (dsc == NULL || coords == NULL || dsc->opa <= LV_OPA_MIN || dsc->width <= 0)
        return;

    lv_layer_t *layer = draw_task->target_layer;
    if (layer == NULL)
        return;

    lv_area_t clip;
    if (!lv_area_intersect(&clip, &draw_task->clip_area, &layer->buf_area))
        return;

    drv_epic_render_buf *render_buf = render_list_get_current_buf();
    if (render_buf == NULL)
        return;

    int32_t offset_x = 0;
    int32_t offset_y = 0;
    bool is_sub_layer = false;
    lv_epic_get_coord_offset(draw_task, &offset_x, &offset_y, &is_sub_layer);
    LV_UNUSED(is_sub_layer);

    lv_area_t clip_local;
    lv_area_t shadow_area_local;
    lv_epic_area_to_render_coords(&clip, offset_x, offset_y, &clip_local);
    lv_epic_area_to_render_coords(coords, offset_x, offset_y, &shadow_area_local);
    if (!lv_epic_clip_area_by_render_buf(&clip_local, render_buf))
        return;

    drv_epic_operation *op = drv_epic_alloc_op(render_buf);
    if (op == NULL)
        return;

    op->op = DRV_EPIC_DRAW_SHADOW;
    op->clip_area.x0 = clip_local.x1;
    op->clip_area.y0 = clip_local.y1;
    op->clip_area.x1 = clip_local.x2;
    op->clip_area.y1 = clip_local.y2;

    op->desc.shadow.area.x0 = shadow_area_local.x1;
    op->desc.shadow.area.y0 = shadow_area_local.y1;
    op->desc.shadow.area.x1 = shadow_area_local.x2;
    op->desc.shadow.area.y1 = shadow_area_local.y2;
    op->desc.shadow.radius = (uint16_t)LV_MAX(0, LV_MIN(dsc->radius, UINT16_MAX));
    op->desc.shadow.width = (uint16_t)LV_MAX(0, LV_MIN(dsc->width, UINT16_MAX));
    op->desc.shadow.spread = (int16_t)LV_MAX(INT16_MIN, LV_MIN(dsc->spread, INT16_MAX));
    op->desc.shadow.ofs_x = (int16_t)LV_MAX(INT16_MIN, LV_MIN(dsc->ofs_x, INT16_MAX));
    op->desc.shadow.ofs_y = (int16_t)LV_MAX(INT16_MIN, LV_MIN(dsc->ofs_y, INT16_MAX));
    op->desc.shadow.argb8888 = ((uint32_t)dsc->opa << 24) |
                               ((uint32_t)dsc->color.red << 16) |
                               ((uint32_t)dsc->color.green << 8) |
                               (uint32_t)dsc->color.blue;

    drv_epic_commit_op(op);
}

#endif /* LV_USE_DRAW_EPIC */
