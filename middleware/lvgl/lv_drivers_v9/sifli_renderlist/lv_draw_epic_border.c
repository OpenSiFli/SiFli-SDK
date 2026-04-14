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
#include "../../lvgl_private.h"
#include "lv_api_map_v8.h"


#include "../../misc/lv_area_private.h"
#include "lv_draw_sw_mask_private.h"
#include "../lv_draw_private.h"
#include "../lv_draw_private.h"
#include "lv_draw_sw.h"

#include "blend/lv_draw_sw_blend_private.h"
#include "../../misc/lv_math.h"
#include "../../misc/lv_text_ap.h"
#include "../../core/lv_refr.h"
#include "../../misc/lv_assert.h"
#include "../../stdlib/lv_string.h"
#include "../lv_draw_mask.h"


/*********************
 *      DEFINES
 *********************/
#define SPLIT_LIMIT             50


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

void lv_draw_epic_border(lv_draw_task_t *draw_task, const lv_draw_border_dsc_t *dsc, const lv_area_t *coords)
{
    if (dsc->opa <= LV_OPA_MIN) return;
    if (dsc->width == 0) return;
    if (dsc->side == LV_BORDER_SIDE_NONE) return;

    int32_t coords_w = lv_area_get_width(coords);
    int32_t coords_h = lv_area_get_height(coords);
    int32_t rout = dsc->radius;
    int32_t short_side = LV_MIN(coords_w, coords_h);
    if (rout > short_side >> 1) rout = short_side >> 1;

    drv_epic_render_buf *render_buf = render_list_get_current_buf();
    if (render_buf == NULL) {
        LV_LOG_ERROR("EPIC: No render buffer available for border operation\n");
        return;
    }

    int32_t coord_offset_x = 0;
    int32_t coord_offset_y = 0;
    bool is_sub_layer = false;
    lv_epic_get_coord_offset(draw_task, &coord_offset_x, &coord_offset_y, &is_sub_layer);

    lv_area_t border_clip_abs;
    if (!lv_area_intersect(&border_clip_abs, coords, &draw_task->clip_area)) return;

    lv_area_t border_clip_local;
    lv_area_t border_area_local;
    lv_epic_area_to_render_coords(&border_clip_abs, coord_offset_x, coord_offset_y, &border_clip_local);
    lv_epic_area_to_render_coords(coords, coord_offset_x, coord_offset_y, &border_area_local);
    if (!lv_epic_clip_area_by_render_buf(&border_clip_local, render_buf)) {
        return;
    }

    drv_epic_operation *op = drv_epic_alloc_op(render_buf);
    if (op == NULL)
    {
        LV_LOG("EPIC: Failed to allocate EPIC operation for border drawing\n");
        return;
    }

    op->op = DRV_EPIC_DRAW_BORDER;
    op->clip_area.x0 = border_clip_local.x1;
    op->clip_area.y0 = border_clip_local.y1;
    op->clip_area.x1 = border_clip_local.x2;
    op->clip_area.y1 = border_clip_local.y2;

    op->desc.border.area.x0 = border_area_local.x1;
    op->desc.border.area.y0 = border_area_local.y1;
    op->desc.border.area.x1 = border_area_local.x2;
    op->desc.border.area.y1 = border_area_local.y2;

    op->desc.border.radius = rout;
    op->desc.border.width = dsc->width;
    op->desc.border.top_side = (dsc->side & LV_BORDER_SIDE_TOP) ? 1 : 0;
    op->desc.border.bot_side = (dsc->side & LV_BORDER_SIDE_BOTTOM) ? 1 : 0;
    op->desc.border.left_side = (dsc->side & LV_BORDER_SIDE_LEFT) ? 1 : 0;
    op->desc.border.right_side = (dsc->side & LV_BORDER_SIDE_RIGHT) ? 1 : 0;

    uint32_t alpha = dsc->opa;
    op->desc.border.argb8888 = (alpha << 24) | (dsc->color.red << 16) | (dsc->color.green << 8) | dsc->color.blue;

    drv_epic_commit_op(op);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif /*LV_USE_DRAW_EPIC*/
