/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lv_draw_epic.h"

#if LV_USE_DRAW_EPIC
#include "app_mem.h"
#include "lv_epic_utils.h"
#include "../lv_draw_private.h"
#include "../lv_draw_mask_private.h"
#include "lv_draw_sw_mask_private.h"
#include "../../misc/lv_area_private.h"
#include "../../stdlib/lv_string.h"

void lv_draw_epic_mask_rect(lv_draw_task_t *draw_task, const lv_draw_mask_rect_dsc_t *dsc)
{
#ifdef EPIC_SUPPORT_MASK
    lv_layer_t *layer = draw_task->target_layer;
    if (layer->draw_buf == NULL || layer->color_format != LV_COLOR_FORMAT_ARGB8888)
    {
        LV_LOG_WARN("EPIC: Rectangle masks require an ARGB8888 layer");
        return;
    }
    lv_area_t clip;
    if (!lv_area_intersect(&clip, &draw_task->clip_area, &layer->buf_area)) return;
    drv_epic_render_buf *render_buf = render_list_get_current_buf();
    if (render_buf == NULL) return;

    int32_t offset_x = 0, offset_y = 0;
    bool sub_layer = false;
    lv_epic_get_coord_offset(draw_task, &offset_x, &offset_y, &sub_layer);
    lv_area_t local_clip;
    lv_epic_area_to_render_coords(&clip, offset_x, offset_y, &local_clip);
    if (!lv_epic_clip_area_by_render_buf(&local_clip, render_buf)) return;
    /* A render buffer can narrow the task's clip; keep A8 stride and geometry
     * consistent with the actual operation clip. */
    clip = local_clip;
    lv_area_move(&clip, offset_x, offset_y);
    uint32_t width = lv_area_get_width(&clip);
    uint32_t height = lv_area_get_height(&clip);
    if (height > SIZE_MAX / width) return;
    uint8_t *coverage = app_cache_alloc((size_t)width * height, IMAGE_CACHE_PSRAM);
    if (coverage == NULL)
    {
        LV_LOG_ERROR("EPIC: Failed to allocate rectangle mask");
        return;
    }

    lv_draw_sw_mask_radius_param_t radius_mask;
    lv_draw_sw_mask_radius_init(&radius_mask, &dsc->area, dsc->radius, false);
    void *masks[2] = {&radius_mask, NULL};
    for (int32_t y = clip.y1; y <= clip.y2; y++)
    {
        uint8_t *line = coverage + (size_t)(y - clip.y1) * width;
        lv_memset(line, 255, width);
        int32_t left = clip.x1;
        uint32_t count = width;
        if (dsc->keep_outside)
        {
            if (y < dsc->area.y1 || y > dsc->area.y2) continue;
            left = LV_MAX(left, dsc->area.x1);
            int32_t right = LV_MIN(clip.x2, dsc->area.x2);
            if (left > right) continue;
            count = right - left + 1;
        }
        uint8_t *part = line + left - clip.x1;
        if (lv_draw_sw_mask_apply(masks, part, left, y, count) == LV_DRAW_SW_MASK_RES_TRANSP)
            lv_memzero(part, count);
    }
    lv_draw_sw_mask_free_param(&radius_mask);
    if (!lv_draw_epic_retain_mask_data(coverage))
    {
        app_cache_free(coverage);
        return;
    }

    drv_epic_operation *op = drv_epic_alloc_op(render_buf);
    if (op == NULL) return;
    op->op = DRV_EPIC_DRAW_IMAGE;
    op->clip_area = (EPIC_AreaTypeDef){local_clip.x1, local_clip.y1, local_clip.x2, local_clip.y2};
    EPIC_LayerConfigTypeDef *source = &op->desc.blend.layer;
    HAL_EPIC_LayerConfigInit(source);
    source->data = layer->draw_buf->data;
    source->data_size = layer->draw_buf->data_size;
    source->color_mode = EPIC_INPUT_ARGB8888;
    source->width = lv_area_get_width(&layer->buf_area);
    source->height = lv_area_get_height(&layer->buf_area);
    source->total_width = layer->draw_buf->header.stride / 4;
    source->x_offset = layer->buf_area.x1 - offset_x;
    source->y_offset = layer->buf_area.y1 - offset_y;
    op->desc.blend.use_dest_as_bg = EPIC_BLEND_MODE_OVERWRITE;
    HAL_EPIC_LayerConfigInit(&op->mask);
    op->mask.data = coverage;
    op->mask.data_size = (size_t)width * height;
    op->mask.color_mode = EPIC_INPUT_A8;
    op->mask.ax_mode = ALPHA_BLEND_MASK;
    op->mask.width = op->mask.total_width = width;
    op->mask.height = height;
    op->mask.x_offset = local_clip.x1;
    op->mask.y_offset = local_clip.y1;
    /* Read the preceding operations' result and overwrite its alpha in queue
     * order. Blending over the old destination would preserve masked pixels. */
    drv_epic_commit_op(op);
#else
    LV_UNUSED(draw_task);
    LV_UNUSED(dsc);
    LV_LOG_WARN("EPIC: Rectangle masks are not supported on this chip");
#endif
}

#endif /*LV_USE_DRAW_EPIC*/
