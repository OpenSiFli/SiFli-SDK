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

    if (dsc->img_src != NULL)
    {
        LV_LOG_ERROR("EPIC: Arc with image source is not supported in render-list mode\n");
        return;
    }

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

    uint32_t alpha = dsc->opa;
    op->desc.arc.argb8888 = (alpha << 24) | (dsc->color.red << 16) | (dsc->color.green << 8) | dsc->color.blue;

    drv_epic_commit_op(op);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif /*LV_USE_DRAW_EPIC*/
