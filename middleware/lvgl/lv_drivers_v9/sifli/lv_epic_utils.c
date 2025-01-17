/**
  ******************************************************************************
  * @file   lv_epic_utils.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
#include "string.h"

/*********************
 *      DEFINES
 *********************/
#define GPU_OPERATION_MAX_PIXELS (1000*1000)

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
uint32_t lv_img_2_epic_cf(lv_color_format_t cf)
{
    uint32_t color_mode;

    switch (cf)
    {
    /*2 byte (+alpha) formats*/
    case LV_COLOR_FORMAT_RGB565:
        color_mode = EPIC_INPUT_RGB565;
        break;
    case LV_COLOR_FORMAT_RGB565A8:
        color_mode = EPIC_INPUT_RGB565;
        break;

    /*3 byte (+alpha) formats*/
    case LV_COLOR_FORMAT_RGB888:
        color_mode = EPIC_INPUT_RGB888;
        break;
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
        color_mode = EPIC_INPUT_ARGB8888;
        break;

    case LV_COLOR_FORMAT_A8:
        color_mode = EPIC_INPUT_A8;
        break;

    case LV_COLOR_FORMAT_ARGB8565:
        color_mode = EPIC_INPUT_ARGB8565;
        break;

    default:
        LV_LOG_USER("format %d \r\n", cf);
        LV_ASSERT_MSG(false, "Unsupported color format");
        break;
    }

    return color_mode;
}

EPIC_ColorDef lv_color_to_epic_color(lv_color_t color, lv_opa_t opa)
{
    EPIC_ColorDef c;
    c.ch.color_r = color.red;
    c.ch.color_g = color.green;
    c.ch.color_b = color.blue;
    c.ch.alpha = opa;
    return c;
}

uint32_t lv_epic_setup_bg_and_output_layer(EPIC_LayerConfigTypeDef *epic_bg_layer,
        EPIC_LayerConfigTypeDef *epic_output_layer,
        lv_draw_unit_t *draw_unit, const lv_area_t *coords)
{
    lv_layer_t *layer = draw_unit->target_layer;

    lv_area_t blend_area;
    if (!lv_area_intersect(&blend_area, coords, draw_unit->clip_area))
        return 1; /*Fully clipped, nothing to do*/

    lv_color_format_t dest_cf = layer->color_format;

    /*Setup bg layer*/
    HAL_EPIC_LayerConfigInit(epic_bg_layer);
    epic_bg_layer->color_en = false;
    epic_bg_layer->data = (uint8_t *)lv_draw_layer_go_to_xy(layer, 0, 0);
    epic_bg_layer->color_mode = lv_img_2_epic_cf(dest_cf);
    epic_bg_layer->width = lv_area_get_width(&layer->buf_area);
    epic_bg_layer->total_width = lv_area_get_width(&layer->buf_area);
    epic_bg_layer->height = lv_area_get_height(&layer->buf_area);
    epic_bg_layer->x_offset = layer->buf_area.x1;
    epic_bg_layer->y_offset = layer->buf_area.y1;


    /*Setup output layer*/
    memcpy(epic_output_layer, epic_bg_layer, sizeof(EPIC_LayerConfigTypeDef));
    epic_output_layer->width = lv_area_get_width(&blend_area);
    epic_output_layer->height = lv_area_get_height(&blend_area);
    epic_output_layer->x_offset = blend_area.x1;
    epic_output_layer->y_offset = blend_area.y1;

    epic_output_layer->data = (uint8_t *)lv_draw_layer_go_to_xy(layer,
                              blend_area.x1 - layer->buf_area.x1,
                              blend_area.y1 - layer->buf_area.y1);


    LV_ASSERT((epic_output_layer->height * epic_output_layer->width) <= GPU_OPERATION_MAX_PIXELS);

    return 0;
}


void lv_epic_draw_blend(lv_draw_unit_t *draw_unit, const lv_draw_sw_blend_dsc_t *blend_dsc)
{
    /*Do not draw transparent things*/
    if (blend_dsc->opa <= LV_OPA_MIN) return;
    if (blend_dsc->mask_buf && blend_dsc->mask_res == LV_DRAW_SW_MASK_RES_TRANSP) return;

    lv_area_t blend_area;
    if (!lv_area_intersect(&blend_area, blend_dsc->blend_area, draw_unit->clip_area)) return;


    EPIC_LayerConfigTypeDef input_layers[3];
    EPIC_LayerConfigTypeDef output_canvas;

    int ret;


    if (lv_epic_setup_bg_and_output_layer(&input_layers[0], &output_canvas, draw_unit, blend_dsc->blend_area))
        return;/*Fully clipped, nothing to do*/



    if (blend_dsc->src_buf == NULL)
    {
        output_canvas.color_r = blend_dsc->color.red;
        output_canvas.color_g = blend_dsc->color.green;
        output_canvas.color_b = blend_dsc->color.blue;

        LV_ASSERT((output_canvas.height * output_canvas.width) <= GPU_OPERATION_MAX_PIXELS);
        output_canvas.color_en = true;


        if ((blend_dsc->mask_buf) && (blend_dsc->mask_res != LV_DRAW_SW_MASK_RES_FULL_COVER))
        {
#if defined(EPIC_SUPPORT_MONOCHROME_LAYER)&&defined(EPIC_SUPPORT_MASK)

            HAL_EPIC_LayerConfigInit(&input_layers[1]);
            input_layers[1].data = (uint8_t *)blend_dsc->mask_buf;
            input_layers[1].x_offset = blend_dsc->mask_area->x1;
            input_layers[1].y_offset = blend_dsc->mask_area->y1;
            input_layers[1].width = lv_area_get_width(blend_dsc->mask_area);
            input_layers[1].total_width = lv_area_get_width(blend_dsc->mask_area);
            input_layers[1].height = lv_area_get_height(blend_dsc->mask_area);
            input_layers[1].color_mode = EPIC_INPUT_A8;
            input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;
            input_layers[1].color_r = blend_dsc->color.red;
            input_layers[1].color_g = blend_dsc->color.green;
            input_layers[1].color_b = blend_dsc->color.blue;
            input_layers[1].alpha = blend_dsc->opa;
            input_layers[1].color_en = true;


            output_canvas.color_en = false;

            ret = drv_epic_fill_ext(input_layers, 2, &output_canvas, NULL);
#else
            LV_ASSERT(0);
#endif /* EPIC_SUPPORT_MONOCHROME_LAYER&&EPIC_SUPPORT_MASK */
        }
        else if (blend_dsc->opa < LV_OPA_MAX)
        {
            input_layers[0].alpha = (0 == blend_dsc->opa) ? 255 : (256 - blend_dsc->opa);

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
        if (!lv_area_intersect(&blend_area, &blend_area, blend_dsc->src_area)) return;

        /*Setup fg layer*/
        HAL_EPIC_LayerConfigInit(&input_layers[1]);

        input_layers[1].alpha = blend_dsc->opa;
        input_layers[1].x_offset = blend_dsc->src_area->x1;
        input_layers[1].y_offset = blend_dsc->src_area->y1;

        input_layers[1].color_mode = lv_img_2_epic_cf(blend_dsc->src_color_format);
        input_layers[1].data = (uint8_t *)blend_dsc->src_buf;
        input_layers[1].width = lv_area_get_width(blend_dsc->src_area);
        input_layers[1].height = lv_area_get_height(blend_dsc->src_area);
        input_layers[1].total_width = blend_dsc->src_stride / lv_color_format_get_size(blend_dsc->src_color_format);


        if ((blend_dsc->mask_buf) && (blend_dsc->mask_res != LV_DRAW_SW_MASK_RES_FULL_COVER))
        {
#if defined(EPIC_SUPPORT_MONOCHROME_LAYER)&&defined(EPIC_SUPPORT_MASK)

            HAL_EPIC_LayerConfigInit(&input_layers[2]);
            input_layers[2].data = (uint8_t *)blend_dsc->mask_buf;
            input_layers[2].x_offset = blend_dsc->mask_area->x1;
            input_layers[2].y_offset = blend_dsc->mask_area->y1;
            input_layers[2].width = lv_area_get_width(blend_dsc->mask_area);
            input_layers[2].total_width = lv_area_get_width(blend_dsc->mask_area);
            input_layers[2].height = lv_area_get_height(blend_dsc->mask_area);
            input_layers[2].color_mode = EPIC_INPUT_A8;
            input_layers[2].ax_mode = ALPHA_BLEND_MASK;


            ret = drv_epic_blend(input_layers, 3, &output_canvas, NULL);
#else
            LV_ASSERT(0);
#endif /* EPIC_SUPPORT_MONOCHROME_LAYER&&EPIC_SUPPORT_MASK */
        }
        else
        {
            ret = drv_epic_blend(input_layers, 2, &output_canvas, NULL);
        }

        LV_ASSERT(0 == ret);
    }
}

void lv_epic_print_area_info(const char *prefix, const lv_area_t *p_area)
{
    LV_EPIC_LOG("%s[%d,%d,%d,%d]", prefix, p_area->x1, p_area->y1, p_area->x2, p_area->y2);
}
void lv_epic_print_layer_info(lv_draw_unit_t *draw_unit)
{
    lv_layer_t *layer = draw_unit->target_layer;
    LV_EPIC_LOG("format %d, buf=%p, stride=%u", layer->color_format,
                layer->draw_buf->data, layer->draw_buf->header.stride);

    lv_epic_print_area_info("buf_area", &layer->buf_area);
    lv_epic_print_area_info("clip_area", draw_unit->clip_area);
}
/**********************
 *   STATIC FUNCTIONS
 **********************/


#endif /*LV_USE_DRAW_EPIC*/
