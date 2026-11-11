/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
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
#include "lv_draw_sw_mask_private.h"
#include "blend/lv_draw_sw_blend_private.h"
#include "../lv_image_decoder_private.h"
#include "lv_draw_sw.h"



#include "../../misc/lv_math.h"
#include "../../misc/lv_log.h"
#include "../../stdlib/lv_mem.h"
#include "../../stdlib/lv_string.h"
#include "../lv_draw_private.h"
#include <stdint.h>

/*********************
 *      DEFINES
 *********************/
#define SPLIT_RADIUS_LIMIT 10
#define SPLIT_ANGLE_GAP_LIMIT 60

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static EPIC_LayerConfigTypeDef *arc_image_prepare(const lv_draw_arc_dsc_t *dsc,
        int32_t center_x, int32_t center_y, EPIC_LayerConfigTypeDef *mask);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_draw_epic_arc(lv_draw_task_t *draw_task, const lv_draw_arc_dsc_t *dsc, const lv_area_t *coords)
{
    if (dsc->opa <= LV_OPA_MIN) return;
    if (dsc->width == 0) return;
    if (dsc->start_angle == dsc->end_angle) return;

    int32_t width = dsc->width;
    if (width > dsc->radius) width = dsc->radius;

    lv_area_t area_out = *coords;
    lv_area_t clipped_area;
    if (!lv_area_intersect(&clipped_area, &area_out, &draw_task->clip_area)) return;

    int32_t coord_offset_x = 0;
    int32_t coord_offset_y = 0;
    bool is_sub_layer = false;
    lv_epic_get_coord_offset(draw_task, &coord_offset_x, &coord_offset_y, &is_sub_layer);

    drv_epic_render_buf *render_buf = render_list_get_current_buf();
    if (render_buf == NULL) {
        LV_LOG("EPIC: No render buffer available for arc drawing\n");
        return;
    }

    lv_area_t arc_clip_local;
    lv_epic_area_to_render_coords(&clipped_area, coord_offset_x, coord_offset_y, &arc_clip_local);
    if (!lv_epic_clip_area_by_render_buf(&arc_clip_local, render_buf)) {
        return;
    }

    int32_t center_x_local = 0;
    int32_t center_y_local = 0;
    lv_epic_point_to_render_coords(dsc->center.x, dsc->center.y,
                                   coord_offset_x, coord_offset_y,
                                   &center_x_local, &center_y_local);

    EPIC_LayerConfigTypeDef *image = NULL;
    EPIC_LayerConfigTypeDef image_mask;
    HAL_EPIC_LayerConfigInit(&image_mask);
    if (dsc->img_src != NULL)
    {
        if (width <= 0 || dsc->radius == 0 ||
                center_x_local - dsc->radius < INT16_MIN ||
                center_x_local + dsc->radius - 1 > INT16_MAX ||
                center_y_local - dsc->radius < INT16_MIN ||
                center_y_local + dsc->radius - 1 > INT16_MAX)
        {
            LV_LOG_WARN("EPIC: Arc image geometry out of range\n");
            return;
        }
        image = arc_image_prepare(dsc, center_x_local, center_y_local, &image_mask);
        if (image == NULL)
        {
            LV_LOG_WARN("EPIC: Arc image unavailable or unsupported\n");
            return;
        }
        if (!lv_draw_epic_retain_arc_image(image))
        {
            lv_draw_epic_release_arc_image(image);
            LV_LOG_ERROR("EPIC: Failed to retain arc image until render-list completion\n");
            return;
        }
    }

    drv_epic_operation *op = drv_epic_alloc_op(render_buf);
    if (op == NULL) {
        LV_LOG("EPIC: Failed to allocate operation\n");
        return;
    }

    op->op = DRV_EPIC_DRAW_ARC;
    op->clip_area.x0 = arc_clip_local.x1;
    op->clip_area.y0 = arc_clip_local.y1;
    op->clip_area.x1 = arc_clip_local.x2;
    op->clip_area.y1 = arc_clip_local.y2;

    op->desc.arc.center_x = center_x_local;
    op->desc.arc.center_y = center_y_local;
    op->desc.arc.start_angle = (uint16_t)dsc->start_angle;
    op->desc.arc.end_angle = (uint16_t)dsc->end_angle;
    op->desc.arc.width = (uint16_t)width;
    op->desc.arc.radius = (uint16_t)dsc->radius;
    op->desc.arc.round_start = (uint8_t)dsc->rounded;
    op->desc.arc.round_end = (uint8_t)dsc->rounded;
    op->desc.arc.image = image;
    op->mask = image_mask;

    uint32_t alpha = dsc->opa;
    op->desc.arc.argb8888 = (alpha << 24) | (dsc->color.red << 16) | (dsc->color.green << 8) | dsc->color.blue;

    drv_epic_commit_op(op);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/* ARC resources share the same decoder lifetime as rectangle images. */
void lv_draw_epic_release_arc_image(void *arc_image)
{
    lv_draw_epic_image_release(arc_image);
}

static EPIC_LayerConfigTypeDef *arc_image_prepare(const lv_draw_arc_dsc_t *dsc,
                                                  int32_t center_x,
                                                  int32_t center_y,
                                                  EPIC_LayerConfigTypeDef *mask)
{
    EPIC_LayerConfigTypeDef *image =
        lv_draw_epic_image_prepare(dsc->img_src, dsc->opa, dsc->color, mask);
    if (image == NULL) return NULL;

    int32_t left = center_x - image->width / 2;
    int32_t top = center_y - image->height / 2;
    if (left < INT16_MIN || left + image->width - 1 > INT16_MAX ||
        top < INT16_MIN || top + image->height - 1 > INT16_MAX)
    {
        lv_draw_epic_image_release(image);
        return NULL;
    }
    image->x_offset = left;
    image->y_offset = top;
    if (mask->data)
    {
        mask->x_offset = left;
        mask->y_offset = top;
    }
    return image;
}

#endif /*LV_USE_DRAW_EPIC*/
