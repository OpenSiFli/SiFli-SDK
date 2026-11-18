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
#include "../../stdlib/lv_mem.h"
#include <stdint.h>

typedef struct
{
    EPIC_LayerConfigTypeDef *source;
    drv_epic_rect_image_t images[];
} bg_image_resource_t;

void lv_draw_epic_release_bg_image(void *image)
{
    bg_image_resource_t *resource = image;
    if (resource == NULL) return;
    lv_draw_epic_image_release(resource->source);
    lv_free(resource);
}

/* Only rounded image clipping needs the backend rectangle mask. Keep square
 * images, including tiled backgrounds, on the existing image path to avoid
 * splitting compressed images into small masked hardware operations. */
bool lv_draw_epic_bg_img(lv_draw_task_t *draw_task, const lv_draw_image_dsc_t *dsc,
                         const lv_area_t *coords)
{
    if (draw_task->type != LV_DRAW_TASK_TYPE_IMAGE || dsc->clip_radius <= 0 ||
        dsc->rotation || dsc->scale_x != LV_SCALE_NONE || dsc->scale_y != LV_SCALE_NONE ||
        dsc->skew_x || dsc->skew_y || dsc->recolor_opa > LV_OPA_MIN ||
        dsc->blend_mode != LV_BLEND_MODE_NORMAL || dsc->colorkey || dsc->bitmap_mask_src)
        return false;
    if (dsc->opa <= LV_OPA_MIN) return true;

    lv_area_t rectangle = dsc->image_area;
    if (rectangle.x2 == LV_COORD_MIN) rectangle = *coords;
    lv_area_t clip;
    if (!lv_area_intersect(&clip, coords, &rectangle) ||
        !lv_area_intersect(&clip, &clip, &draw_task->clip_area) ||
        !lv_area_intersect(&clip, &clip, &draw_task->target_layer->buf_area))
        return true;

    drv_epic_render_buf *render_buf = render_list_get_current_buf();
    if (render_buf == NULL) return true;
    int32_t offset_x = 0, offset_y = 0;
    bool sub_layer = false;
    lv_epic_get_coord_offset(draw_task, &offset_x, &offset_y, &sub_layer);
    lv_epic_area_to_render_coords(&clip, offset_x, offset_y, &clip);
    lv_epic_area_to_render_coords(&rectangle, offset_x, offset_y, &rectangle);
    if (!lv_epic_clip_area_by_render_buf(&clip, render_buf)) return true;
    if (rectangle.x1 < INT16_MIN || rectangle.y1 < INT16_MIN ||
        rectangle.x2 > INT16_MAX || rectangle.y2 > INT16_MAX)
        return false;

    EPIC_LayerConfigTypeDef mask;
    HAL_EPIC_LayerConfigInit(&mask);
    EPIC_LayerConfigTypeDef *source = lv_draw_epic_image_prepare(dsc->src, dsc->opa, dsc->recolor, &mask);
    if (source == NULL) return false;

    int32_t first_x = dsc->tile ? rectangle.x1 : coords->x1 - offset_x;
    int32_t first_y = dsc->tile ? rectangle.y1 : coords->y1 - offset_y;
    int32_t image_w = source->width;
    int32_t image_h = source->height;
    uint32_t columns = 1, rows = 1;
    /* An independent RGB565A8 alpha plane must move with each tile. Other
     * formats can use the backend's tile walker in one operation. */
    bool split_tiles = dsc->tile && mask.data != NULL;
    if (split_tiles)
    {
        first_x += (clip.x1 - first_x) / image_w * image_w;
        first_y += (clip.y1 - first_y) / image_h * image_h;
        columns = (clip.x2 - first_x) / image_w + 1;
        rows = (clip.y2 - first_y) / image_h + 1;
    }
    if (first_x < INT16_MIN || first_y < INT16_MIN ||
        first_x + image_w - 1 > INT16_MAX || first_y + image_h - 1 > INT16_MAX ||
        rows > (SIZE_MAX - sizeof(bg_image_resource_t)) / sizeof(drv_epic_rect_image_t) / columns)
    {
        lv_draw_epic_image_release(source);
        return false;
    }

    bg_image_resource_t *resource = lv_malloc(sizeof(*resource) +
                                             (size_t)columns * rows * sizeof(resource->images[0]));
    if (resource == NULL)
    {
        lv_draw_epic_image_release(source);
        LV_LOG_ERROR("EPIC: Failed to allocate background image descriptors");
        return true;
    }
    resource->source = source;
    if (!lv_draw_epic_retain_bg_image(resource))
    {
        lv_draw_epic_release_bg_image(resource);
        LV_LOG_ERROR("EPIC: Failed to retain background image");
        return true;
    }

    int32_t radius = dsc->clip_radius;
    int32_t max_radius = LV_MIN(lv_area_get_width(&rectangle), lv_area_get_height(&rectangle)) / 2;
    radius = LV_CLAMP(0, radius, max_radius);
    for (uint32_t row = 0; row < rows; row++)
    {
        for (uint32_t column = 0; column < columns; column++)
        {
            drv_epic_rect_image_t *image = &resource->images[row * columns + column];
            image->layer = *source;
            image->layer.x_offset = first_x + (int32_t)column * image_w;
            image->layer.y_offset = first_y + (int32_t)row * image_h;
            image->tiled = dsc->tile && !split_tiles;
            drv_epic_operation *op = drv_epic_alloc_op(render_buf);
            if (op == NULL)
            {
                LV_LOG_ERROR("EPIC: Failed to allocate background image operation");
                return true;
            }
            op->op = DRV_EPIC_DRAW_RECT;
            op->clip_area = (EPIC_AreaTypeDef){clip.x1, clip.y1, clip.x2, clip.y2};
            op->desc.rectangle.area = (EPIC_AreaTypeDef){rectangle.x1, rectangle.y1,
                                                        rectangle.x2, rectangle.y2};
            op->desc.rectangle.radius = radius;
            op->desc.rectangle.top_fillet = radius != 0;
            op->desc.rectangle.bot_fillet = radius != 0;
            op->desc.rectangle.grad_color_en = 0;
            op->desc.rectangle.argb8888 = 0; /* LVGL emits the fill as a separate task. */
            op->desc.rectangle.bg_image = image;
            op->mask = mask;
            if (mask.data)
            {
                op->mask.x_offset = image->layer.x_offset;
                op->mask.y_offset = image->layer.y_offset;
            }
            drv_epic_commit_op(op);
        }
    }
    return true;
}

#endif /*LV_USE_DRAW_EPIC*/
