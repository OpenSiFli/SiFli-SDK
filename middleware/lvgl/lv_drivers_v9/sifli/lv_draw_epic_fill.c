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
        return; /*Fully clipped, nothing to do*/

    if (!lv_area_intersect(&blend_area, &blend_area, &layer->buf_area))
        return; /*Fully clipped, nothing to do*/


    lv_color_format_t dest_cf = layer->color_format;
    uint8_t *dest_blend_area_buf = lv_draw_layer_go_to_xy(layer, blend_area.x1 - layer->buf_area.x1,
                                   blend_area.y1 - layer->buf_area.y1);

    /*Call epic**/
    lv_grad_dir_t grad_dir = (lv_grad_dir_t) dsc->grad.dir;

    if (LV_GRAD_DIR_NONE == grad_dir)
    {
        EPIC_LayerConfigTypeDef input_layers[1];
        EPIC_LayerConfigTypeDef output_canvas;

        int ret;
        if (lv_epic_setup_bg_and_output_layer(&input_layers[0], &output_canvas, draw_task, coords))
            return;/*Fully clipped, nothing to do*/

        output_canvas.color_r = dsc->color.red;
        output_canvas.color_g = dsc->color.green;
        output_canvas.color_b = dsc->color.blue;
        output_canvas.color_en = true;

        if (dsc->opa < LV_OPA_MAX)
        {
            input_layers[0].alpha = (0 == dsc->opa) ? 255 : (256 - dsc->opa);

            ret = drv_epic_fill_ext(input_layers, 1, &output_canvas, NULL);
        }
        else
        {
            ret = drv_epic_fill_ext(NULL, 0, &output_canvas, NULL);
        }
        LV_ASSERT(0 == ret);
    }
    else
    {
        uint32_t epic_cf = lv_img_2_epic_cf(dest_cf);
        
        LV_ASSERT_MSG(EPIC_SUPPROT_OUT_FORMAT(epic_cf), "EPIC: Color format not supported for gradient fill");
        
        EPIC_GradCfgTypeDef param;
        HAL_EPIC_FillGradDataInit(&param);

        param.start = dest_blend_area_buf;
        param.color_mode = epic_cf;
        param.width = lv_area_get_width(&blend_area);
        param.height = lv_area_get_height(&blend_area);
        param.total_width = lv_area_get_width(&layer->buf_area);

        if (LV_GRAD_DIR_VER == grad_dir)
        {
            LV_ASSERT(2 == dsc->grad.stops_count);
            EPIC_ColorDef epic_color1 = lv_color_to_epic_color(dsc->grad.stops[0].color, LV_OPA_MIX2(dsc->grad.stops[0].opa, dsc->opa));
            EPIC_ColorDef epic_color2 = lv_color_to_epic_color(dsc->grad.stops[1].color, LV_OPA_MIX2(dsc->grad.stops[1].opa, dsc->opa));

            param.color[0][0] = epic_color1;
            param.color[0][1] = epic_color1;
            param.color[1][0] = epic_color2;
            param.color[1][1] = epic_color2;
        }
        else if (LV_GRAD_DIR_HOR == grad_dir)
        {
            LV_ASSERT(2 == dsc->grad.stops_count);
            EPIC_ColorDef epic_color1 = lv_color_to_epic_color(dsc->grad.stops[0].color, LV_OPA_MIX2(dsc->grad.stops[0].opa, dsc->opa));
            EPIC_ColorDef epic_color2 = lv_color_to_epic_color(dsc->grad.stops[1].color, LV_OPA_MIX2(dsc->grad.stops[1].opa, dsc->opa));

            param.color[0][0] = epic_color1;
            param.color[0][1] = epic_color2;
            param.color[1][0] = epic_color1;
            param.color[1][1] = epic_color2;
        }
        else
        {
            EPIC_ColorDef epic_color = lv_color_to_epic_color(dsc->color, dsc->opa);

            param.color[0][0] = epic_color;
            param.color[0][1] = epic_color;
            param.color[1][0] = epic_color;
            param.color[1][1] = epic_color;
        }



        int ret_v = drv_epic_fill_grad(&param, NULL);
        LV_ASSERT(0 == ret_v);

    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


#endif /*LV_USE_DRAW_EPIC*/
