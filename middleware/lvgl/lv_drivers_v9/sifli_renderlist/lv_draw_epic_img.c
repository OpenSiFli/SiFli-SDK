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
#include "blend/lv_draw_sw_blend_private.h"
#include "../lv_image_decoder_private.h"
#include "../lv_draw_image_private.h"
#include "../lv_draw_private.h"
#include "lv_draw_sw.h"

#include "../../display/lv_display.h"
#include "../../display/lv_display_private.h"
#include "../../misc/lv_log.h"
#include "../../core/lv_refr_private.h"
#include "../../stdlib/lv_mem.h"
#include "../../misc/lv_math.h"
#include "../../misc/lv_color.h"
#include "../../stdlib/lv_string.h"
#include "../../core/lv_global.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void img_draw_core(lv_draw_task_t *draw_task, const lv_draw_image_dsc_t *draw_dsc,
                          const lv_image_decoder_dsc_t *decoder_dsc, lv_draw_image_sup_t *sup,
                          const lv_area_t *img_coords, const lv_area_t *clipped_img_area);
static bool img_cf_can_overwrite_dst(lv_color_format_t cf);

/**********************
 *  STATIC VARIABLES
 **********************/
#define _draw_info LV_GLOBAL_DEFAULT()->draw_info

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_draw_epic_layer(lv_draw_task_t *draw_task, const lv_draw_image_dsc_t *draw_dsc, const lv_area_t *coords)
{
    if (draw_dsc->blend_mode != LV_BLEND_MODE_NORMAL)
    {
        LV_LOG_WARN("EPIC: Skip layer draw in render-list mode for non-normal blend mode %d", draw_dsc->blend_mode);
        return;
    }

    lv_layer_t *layer_to_draw = (lv_layer_t *)draw_dsc->src;

    if (layer_to_draw->draw_buf == NULL)
    {
        return;
    }

    lv_draw_image_dsc_t new_draw_dsc = *draw_dsc;
    lv_draw_buf_t *retained_draw_buf = layer_to_draw->draw_buf;
    new_draw_dsc.src = layer_to_draw->draw_buf;
    lv_draw_epic_img(draw_task, &new_draw_dsc, coords);
    if (retained_draw_buf && layer_to_draw->draw_buf == retained_draw_buf)
    {
        if (lv_draw_epic_retain_layer_draw_buf(retained_draw_buf))
        {
            layer_to_draw->draw_buf = NULL;
        }
        else
        {
            LV_LOG_ERROR("EPIC: Failed to retain layer draw buffer before LVGL cleanup");
        }
    }
#if LV_USE_LAYER_DEBUG || LV_USE_PARALLEL_DRAW_DEBUG
    lv_area_t area_rot;
    lv_area_copy(&area_rot, coords);
    if (draw_dsc->rotation || draw_dsc->scale_x != LV_SCALE_NONE || draw_dsc->scale_y != LV_SCALE_NONE)
    {
        int32_t w = lv_area_get_width(coords);
        int32_t h = lv_area_get_height(coords);

        _lv_image_buf_get_transformed_area(&area_rot, w, h, draw_dsc->rotation, draw_dsc->scale_x, draw_dsc->scale_y,
                                           &draw_dsc->pivot);

        area_rot.x1 += coords->x1;
        area_rot.y1 += coords->y1;
        area_rot.x2 += coords->x1;
        area_rot.y2 += coords->y1;
    }
    lv_area_t draw_area;
    if (!lv_area_intersect(&draw_area, &area_rot, &draw_task->clip_area)) return;
#endif

#if LV_USE_LAYER_DEBUG
    lv_draw_fill_dsc_t fill_dsc;
    lv_draw_fill_dsc_init(&fill_dsc);
    fill_dsc.color = lv_color_hex(layer_to_draw->color_format == LV_COLOR_FORMAT_ARGB8888 ? 0xff0000 : 0x00ff00);
    fill_dsc.opa = LV_OPA_20;
    lv_draw_sw_fill(draw_task, &fill_dsc, &area_rot);

    lv_draw_border_dsc_t border_dsc;
    lv_draw_border_dsc_init(&border_dsc);
    border_dsc.color = fill_dsc.color;
    border_dsc.opa = LV_OPA_60;
    border_dsc.width = 2;
    lv_draw_sw_border(draw_task, &border_dsc, &area_rot);

#endif

#if LV_USE_PARALLEL_DRAW_DEBUG
    uint32_t idx = 0;
    lv_draw_task_t *draw_task_tmp = _draw_info.unit_head;
    while (draw_task_tmp != draw_task)
    {
        draw_task_tmp = draw_task_tmp->next;
        idx++;
    }

    lv_draw_fill_dsc_t fill_dsc;
    lv_draw_rect_dsc_init(&fill_dsc);
    fill_dsc.color = lv_palette_main(idx % _LV_PALETTE_LAST);
    fill_dsc.opa = LV_OPA_10;
    lv_draw_sw_fill(draw_task, &fill_dsc, &area_rot);

    lv_draw_border_dsc_t border_dsc;
    lv_draw_border_dsc_init(&border_dsc);
    border_dsc.color = lv_palette_main(idx % _LV_PALETTE_LAST);
    border_dsc.opa = LV_OPA_100;
    border_dsc.width = 2;
    lv_draw_sw_border(draw_task, &border_dsc, &area_rot);

    lv_point_t txt_size;
    lv_text_get_size(&txt_size, "W", LV_FONT_DEFAULT, 0, 0, 100, LV_TEXT_FLAG_NONE);

    lv_area_t txt_area;
    txt_area.x1 = draw_area.x1;
    txt_area.x2 = draw_area.x1 + txt_size.x - 1;
    txt_area.y2 = draw_area.y2;
    txt_area.y1 = draw_area.y2 - txt_size.y + 1;

    lv_draw_fill_dsc_init(&fill_dsc);
    fill_dsc.color = lv_color_black();
    lv_draw_sw_fill(draw_task, &fill_dsc, &txt_area);

    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d", idx);
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.text = buf;
    lv_draw_sw_label(draw_task, &label_dsc, &txt_area);
#endif
}
void lv_draw_epic_img(lv_draw_task_t *draw_task, const lv_draw_image_dsc_t *draw_dsc,
                      const lv_area_t *coords)
{
    if (draw_dsc->blend_mode != LV_BLEND_MODE_NORMAL)
    {
        LV_LOG_WARN("EPIC: Skip image draw in render-list mode for non-normal blend mode %d", draw_dsc->blend_mode);
        return;
    }

    if (!draw_dsc->tile)
    {
        lv_draw_image_normal_helper(draw_task, draw_dsc, coords, img_draw_core);
    }
    else
    {
        lv_draw_image_tiled_helper(draw_task, draw_dsc, coords, img_draw_core);
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void img_draw_core(lv_draw_task_t *draw_task, const lv_draw_image_dsc_t *draw_dsc,
                          const lv_image_decoder_dsc_t *decoder_dsc, lv_draw_image_sup_t *sup,
                          const lv_area_t *img_coords, const lv_area_t *clipped_img_area)
{
    const lv_draw_buf_t *decoded = decoder_dsc->decoded;
    const uint8_t *src_buf = decoded->data;
    lv_color_format_t cf = decoded->header.cf;
    uint32_t img_total_width;

    if (decoded->header.flags & LV_IMAGE_FLAGS_EZIP)
        img_total_width = decoded->header.w;
    else
        img_total_width = decoded->header.stride / lv_color_format_get_size(cf);

    if (src_buf)
    {
        drv_epic_render_buf *render_buf = render_list_get_current_buf();
        if (render_buf == NULL) {
            LV_LOG_ERROR("EPIC: No render buffer available for image operation\n");
            return;
        }

        int32_t coord_offset_x = 0;
        int32_t coord_offset_y = 0;
        bool is_sub_layer = false;
        lv_epic_get_coord_offset(draw_task, &coord_offset_x, &coord_offset_y, &is_sub_layer);

        lv_area_t op_clip_area;
        lv_epic_area_to_render_coords(clipped_img_area, coord_offset_x, coord_offset_y, &op_clip_area);

        if (!lv_epic_clip_area_by_render_buf(&op_clip_area, render_buf)) 
        {
            return;
        }

        drv_epic_operation *op = drv_epic_alloc_op(render_buf);
        if (op == NULL) {
            LV_LOG("EPIC: Failed to allocate EPIC operation, returning\n");
            return;
        }

        op->op = DRV_EPIC_DRAW_IMAGE;
        op->clip_area.x0 = op_clip_area.x1;
        op->clip_area.y0 = op_clip_area.y1;
        op->clip_area.x1 = op_clip_area.x2;
        op->clip_area.y1 = op_clip_area.y2;

        HAL_EPIC_LayerConfigInit(&op->desc.blend.layer);

        op->desc.blend.layer.transform_cfg.angle   = (draw_dsc->rotation + 3600) % 3600;
        op->desc.blend.layer.transform_cfg.pivot_x = draw_dsc->pivot.x;
        op->desc.blend.layer.transform_cfg.pivot_y = draw_dsc->pivot.y;
        op->desc.blend.layer.transform_cfg.scale_x = LV_SCALE_NONE * EPIC_INPUT_SCALE_NONE / (uint32_t)draw_dsc->scale_x;
        op->desc.blend.layer.transform_cfg.scale_y = LV_SCALE_NONE * EPIC_INPUT_SCALE_NONE / (uint32_t)draw_dsc->scale_y;

        op->desc.blend.layer.alpha = draw_dsc->opa;
        op->desc.blend.layer.x_offset = img_coords->x1 - coord_offset_x;
        op->desc.blend.layer.y_offset = img_coords->y1 - coord_offset_y;

        if (decoded->header.flags & LV_IMAGE_FLAGS_USER1)
            op->desc.blend.layer.color_mode = EPIC_INPUT_EZIP;
        else
            op->desc.blend.layer.color_mode = lv_img_2_epic_cf(cf);

        op->desc.blend.layer.data = (uint8_t *)src_buf;

        op->desc.blend.layer.width = lv_area_get_width(img_coords);
        op->desc.blend.layer.height = lv_area_get_height(img_coords);
        op->desc.blend.layer.total_width = img_total_width;

        if (EPIC_INPUT_A8 == op->desc.blend.layer.color_mode)
        {
            op->desc.blend.layer.color_en = true;
            op->desc.blend.layer.color_r = draw_dsc->recolor.red;
            op->desc.blend.layer.color_g = draw_dsc->recolor.green;
            op->desc.blend.layer.color_b = draw_dsc->recolor.blue;
        }
        else if (EPIC_INPUT_L8 == op->desc.blend.layer.color_mode)
        {
            op->desc.blend.layer.lookup_table = (uint8_t *)decoder_dsc->palette;
        }
        else
        {
            op->desc.blend.layer.color_en = false;
        }

        bool no_transform = (op->desc.blend.layer.transform_cfg.angle == 0)
                            && (op->desc.blend.layer.transform_cfg.scale_x == EPIC_INPUT_SCALE_NONE)
                            && (op->desc.blend.layer.transform_cfg.scale_y == EPIC_INPUT_SCALE_NONE);
        bool can_overwrite_dst = no_transform
                                 && (draw_dsc->opa >= LV_OPA_MAX)
                                 && !(decoded->header.flags & LV_IMAGE_FLAGS_USER1)
                                 && img_cf_can_overwrite_dst(cf);
        op->desc.blend.use_dest_as_bg = can_overwrite_dst ? EPIC_BLEND_MODE_OVERWRITE : EPIC_BLEND_MODE_NORMAL;

        drv_epic_commit_op(op);
    }
}

static bool img_cf_can_overwrite_dst(lv_color_format_t cf)
{
    switch (cf)
    {
    case LV_COLOR_FORMAT_RGB565:
    case LV_COLOR_FORMAT_RGB888:
    case LV_COLOR_FORMAT_XRGB8888:
        return true;
    default:
        return false;
    }
}




#endif /*LV_USE_DRAW_EPIC*/
