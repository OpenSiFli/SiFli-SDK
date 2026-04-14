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
#include "../lv_draw_private.h"
#include "lv_draw_sw.h"


#include "blend/lv_draw_sw_blend_private.h"
#include "lv_draw_sw_grad.h"

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

void lv_draw_epic_fill(lv_draw_task_t *draw_task, const lv_draw_fill_dsc_t *dsc,
                       const lv_area_t *coords)
{
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN)
        return;

    lv_layer_t *layer = draw_task->target_layer;

    lv_area_t blend_area;
    if (!lv_area_intersect(&blend_area, coords, &draw_task->clip_area))
        return;

    if (!lv_area_intersect(&blend_area, &blend_area, &layer->buf_area))
        return;

    lv_grad_dir_t grad_dir = (lv_grad_dir_t) dsc->grad.dir;

    drv_epic_render_buf *render_buf = render_list_get_current_buf();
    if (render_buf == NULL) {
        LV_LOG_ERROR("EPIC: No render buffer available for fill operation\n");
        return;
    }

    int32_t coord_offset_x = 0;
    int32_t coord_offset_y = 0;
    bool is_sub_layer = false;
    lv_epic_get_coord_offset(draw_task, &coord_offset_x, &coord_offset_y, &is_sub_layer);

    lv_area_t op_blend_area;
    lv_epic_area_to_render_coords(&blend_area, coord_offset_x, coord_offset_y, &op_blend_area);

    if (!lv_epic_clip_area_by_render_buf(&op_blend_area, render_buf))
    {
        return;
    }

    if (LV_GRAD_DIR_NONE == grad_dir)
    {
        drv_epic_operation *op = drv_epic_alloc_op(render_buf);
        if (op == NULL)
        {
            LV_LOG("EPIC: Failed to allocate fill operation\n");
            return;
        }

        op->clip_area.x0 = op_blend_area.x1;
        op->clip_area.y0 = op_blend_area.y1;
        op->clip_area.x1 = op_blend_area.x2;
        op->clip_area.y1 = op_blend_area.y2;

        if (dsc->radius != 0 || dsc->opa < LV_OPA_MAX)
        {
            lv_area_t rect_area_local;
            uint32_t alpha = dsc->opa;

            lv_epic_area_to_render_coords(coords, coord_offset_x, coord_offset_y, &rect_area_local);

            op->op = DRV_EPIC_DRAW_RECT;
            op->desc.rectangle.area.x0 = rect_area_local.x1;
            op->desc.rectangle.area.y0 = rect_area_local.y1;
            op->desc.rectangle.area.x1 = rect_area_local.x2;
            op->desc.rectangle.area.y1 = rect_area_local.y2;
            op->desc.rectangle.radius = dsc->radius;
            op->desc.rectangle.top_fillet = (dsc->radius != 0) ? 1 : 0;
            op->desc.rectangle.bot_fillet = (dsc->radius != 0) ? 1 : 0;
            op->desc.rectangle.argb8888 = (alpha << 24) | (dsc->color.red << 16)
                                         | (dsc->color.green << 8) | dsc->color.blue;
        }
        else
        {
            op->op = DRV_EPIC_DRAW_FILL;
            op->desc.fill.r = dsc->color.red;
            op->desc.fill.g = dsc->color.green;
            op->desc.fill.b = dsc->color.blue;
            op->desc.fill.opa = dsc->opa;
        }

        drv_epic_commit_op(op);
    }
    else
    {
        LV_LOG_ERROR("EPIC: Gradient fill is not supported in render-list mode\n");
        return;
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


#endif /*LV_USE_DRAW_EPIC*/
