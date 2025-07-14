/**
  ******************************************************************************
  * @file   lv_draw_epic_img.c
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

static void img_draw_core(lv_draw_unit_t *draw_unit, const lv_draw_image_dsc_t *draw_dsc,
                          const lv_image_decoder_dsc_t *decoder_dsc, lv_draw_image_sup_t *sup,
                          const lv_area_t *img_coords, const lv_area_t *clipped_img_area);

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

void lv_draw_epic_layer(lv_draw_unit_t *draw_unit, const lv_draw_image_dsc_t *draw_dsc, const lv_area_t *coords)
{
    lv_layer_t *layer_to_draw = (lv_layer_t *)draw_dsc->src;

    /*It can happen that nothing was draw on a layer and therefore its buffer is not allocated.
     *In this case just return. */
    if (layer_to_draw->draw_buf == NULL) return;

    lv_draw_image_dsc_t new_draw_dsc = *draw_dsc;
    new_draw_dsc.src = layer_to_draw->draw_buf;
    lv_draw_epic_img(draw_unit, &new_draw_dsc, coords);
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
    if (!lv_area_intersect(&draw_area, &area_rot, draw_unit->clip_area)) return;
#endif

#if LV_USE_LAYER_DEBUG
    lv_draw_fill_dsc_t fill_dsc;
    lv_draw_fill_dsc_init(&fill_dsc);
    fill_dsc.color = lv_color_hex(layer_to_draw->color_format == LV_COLOR_FORMAT_ARGB8888 ? 0xff0000 : 0x00ff00);
    fill_dsc.opa = LV_OPA_20;
    lv_draw_sw_fill(draw_unit, &fill_dsc, &area_rot);

    lv_draw_border_dsc_t border_dsc;
    lv_draw_border_dsc_init(&border_dsc);
    border_dsc.color = fill_dsc.color;
    border_dsc.opa = LV_OPA_60;
    border_dsc.width = 2;
    lv_draw_sw_border(draw_unit, &border_dsc, &area_rot);

#endif

#if LV_USE_PARALLEL_DRAW_DEBUG
    uint32_t idx = 0;
    lv_draw_unit_t *draw_unit_tmp = _draw_info.unit_head;
    while (draw_unit_tmp != draw_unit)
    {
        draw_unit_tmp = draw_unit_tmp->next;
        idx++;
    }

    lv_draw_fill_dsc_t fill_dsc;
    lv_draw_rect_dsc_init(&fill_dsc);
    fill_dsc.color = lv_palette_main(idx % _LV_PALETTE_LAST);
    fill_dsc.opa = LV_OPA_10;
    lv_draw_sw_fill(draw_unit, &fill_dsc, &area_rot);

    lv_draw_border_dsc_t border_dsc;
    lv_draw_border_dsc_init(&border_dsc);
    border_dsc.color = lv_palette_main(idx % _LV_PALETTE_LAST);
    border_dsc.opa = LV_OPA_100;
    border_dsc.width = 2;
    lv_draw_sw_border(draw_unit, &border_dsc, &area_rot);

    lv_point_t txt_size;
    lv_text_get_size(&txt_size, "W", LV_FONT_DEFAULT, 0, 0, 100, LV_TEXT_FLAG_NONE);

    lv_area_t txt_area;
    txt_area.x1 = draw_area.x1;
    txt_area.x2 = draw_area.x1 + txt_size.x - 1;
    txt_area.y2 = draw_area.y2;
    txt_area.y1 = draw_area.y2 - txt_size.y + 1;

    lv_draw_fill_dsc_init(&fill_dsc);
    fill_dsc.color = lv_color_black();
    lv_draw_sw_fill(draw_unit, &fill_dsc, &txt_area);

    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d", idx);
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.text = buf;
    lv_draw_sw_label(draw_unit, &label_dsc, &txt_area);
#endif
}
void lv_draw_epic_img(lv_draw_unit_t *draw_unit, const lv_draw_image_dsc_t *draw_dsc,
                      const lv_area_t *coords)
{
    if (!draw_dsc->tile)
    {
        lv_draw_image_normal_helper(draw_unit, draw_dsc, coords, img_draw_core);
    }
    else
    {
        lv_draw_image_tiled_helper(draw_unit, draw_dsc, coords, img_draw_core);
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void img_draw_core(lv_draw_unit_t *draw_unit, const lv_draw_image_dsc_t *draw_dsc,
                          const lv_image_decoder_dsc_t *decoder_dsc, lv_draw_image_sup_t *sup,
                          const lv_area_t *img_coords, const lv_area_t *clipped_img_area)
{
    const lv_draw_buf_t *decoded = decoder_dsc->decoded;
    const uint8_t *src_buf = decoded->data;
    //const lv_image_header_t * header = &decoded->header;
    lv_color_format_t cf = decoded->header.cf;
    uint32_t img_total_width;

    if (decoded->header.flags & LV_IMAGE_FLAGS_EZIP)
        img_total_width = decoded->header.w;
    else
        img_total_width = decoded->header.stride / lv_color_format_get_size(cf);

    /*The whole image is available, just draw it*/
    if (src_buf)
    {
        EPIC_LayerConfigTypeDef input_layers[3];
        EPIC_LayerConfigTypeDef output_canvas;

        uint8_t input_layer_cnt = 2;

        if (lv_epic_setup_bg_and_output_layer(&input_layers[0], &output_canvas, draw_unit, clipped_img_area))
            return; /*Fully clipped, nothing to do*/


        /*Setup fg layer*/
        HAL_EPIC_LayerConfigInit(&input_layers[1]);

        input_layers[1].transform_cfg.angle   = (draw_dsc->rotation + 3600) % 3600;
        input_layers[1].transform_cfg.pivot_x = draw_dsc->pivot.x;
        input_layers[1].transform_cfg.pivot_y = draw_dsc->pivot.y;
        input_layers[1].transform_cfg.scale_x = LV_SCALE_NONE * EPIC_INPUT_SCALE_NONE / (uint32_t)draw_dsc->scale_x;
        input_layers[1].transform_cfg.scale_y = LV_SCALE_NONE * EPIC_INPUT_SCALE_NONE / (uint32_t)draw_dsc->scale_y;

        input_layers[1].alpha = draw_dsc->opa;
        input_layers[1].x_offset = img_coords->x1;
        input_layers[1].y_offset = img_coords->y1;

        if (decoded->header.flags & LV_IMAGE_FLAGS_USER1)
            input_layers[1].color_mode = EPIC_INPUT_EZIP;
        else
            input_layers[1].color_mode = lv_img_2_epic_cf(cf);

        input_layers[1].data = (uint8_t *)src_buf;

        input_layers[1].width = lv_area_get_width(img_coords);
        input_layers[1].height = lv_area_get_height(img_coords);
        input_layers[1].total_width = img_total_width;

        if (EPIC_INPUT_A8 == input_layers[1].color_mode)
        {
            input_layers[1].color_en = true;
            input_layers[1].color_r = draw_dsc->recolor.red;
            input_layers[1].color_g = draw_dsc->recolor.green;
            input_layers[1].color_b = draw_dsc->recolor.blue;
        }
        else if (EPIC_INPUT_L8 == input_layers[1].color_mode)
        {
            input_layers[1].lookup_table = (uint8_t *)decoder_dsc->palette;
        }
        else
        {
            input_layers[1].color_en = false;
        }

        if (LV_COLOR_FORMAT_RGB565A8 == cf)
        {

            /*Setup alpha layer*/
            HAL_EPIC_LayerConfigInit(&input_layers[input_layer_cnt]);

            input_layers[input_layer_cnt].alpha = draw_dsc->opa;
            input_layers[input_layer_cnt].x_offset = img_coords->x1;
            input_layers[input_layer_cnt].y_offset = img_coords->y1;
            input_layers[input_layer_cnt].color_mode = EPIC_INPUT_A8;
            input_layers[input_layer_cnt].data = ((uint8_t *)src_buf) + lv_area_get_size(img_coords) * 2;
            input_layers[input_layer_cnt].ax_mode = ALPHA_BLEND_OVERWRITE;
            input_layers[input_layer_cnt].width = lv_area_get_width(img_coords);
            input_layers[input_layer_cnt].height = lv_area_get_height(img_coords);
            input_layers[input_layer_cnt].total_width = img_total_width;
            input_layer_cnt++;
        }

        lv_image_decoder_dsc_t mask_decoder_dsc;
        lv_result_t decoder_res = LV_RESULT_INVALID;

        if (draw_dsc->bitmap_mask_src != NULL)
        {
            lv_area_t mask_area;
            const lv_image_dsc_t *mask_dsc = draw_dsc->bitmap_mask_src;
            lv_image_src_t mask_src_type = lv_image_src_get_type(mask_dsc);
            // Determine whether it is a variable type and in A8 format.
            if (lv_image_src_get_type(mask_dsc) == LV_IMAGE_SRC_VARIABLE && mask_dsc->header.cf == LV_COLOR_FORMAT_A8)
            {


                const lv_image_header_t *header = &mask_dsc->header;

                const lv_area_t *image_area;
                if (lv_area_get_width(&draw_dsc->image_area) < 0) image_area = img_coords;
                else image_area = &draw_dsc->image_area;

                lv_area_set(&mask_area, 0, 0, header->w - 1, header->h - 1);
                lv_area_align(image_area, &mask_area, LV_ALIGN_CENTER, 0, 0);

                HAL_EPIC_LayerConfigInit(&input_layers[input_layer_cnt]);

                input_layers[input_layer_cnt].alpha = 255;
                input_layers[input_layer_cnt].x_offset = mask_area.x1;
                input_layers[input_layer_cnt].y_offset = mask_area.y1;
                input_layers[input_layer_cnt].color_mode = EPIC_INPUT_A8;
                input_layers[input_layer_cnt].data = (uint8_t *)mask_dsc->data;
                input_layers[input_layer_cnt].ax_mode = ALPHA_BLEND_OVERWRITE;
                input_layers[input_layer_cnt].width = lv_area_get_width(&mask_area);
                input_layers[input_layer_cnt].height = lv_area_get_height(&mask_area);
                input_layers[input_layer_cnt].total_width = header->stride;


                input_layer_cnt++;
            }
            else
            {

                decoder_res = lv_image_decoder_open(&mask_decoder_dsc, draw_dsc->bitmap_mask_src, NULL);
                if (decoder_res == LV_RESULT_OK && mask_decoder_dsc.decoded)
                {
                    const lv_draw_buf_t *mask_img = mask_decoder_dsc.decoded;

                    if (mask_img->header.cf == LV_COLOR_FORMAT_A8 ||
                            mask_img->header.cf == LV_COLOR_FORMAT_L8)
                    {
                        const lv_area_t *image_area;
                        if (lv_area_get_width(&draw_dsc->image_area) < 0) image_area = img_coords;
                        else image_area = &draw_dsc->image_area;

                        lv_area_set(&mask_area, 0, 0, mask_img->header.w - 1, mask_img->header.h - 1);
                        lv_area_align(image_area, &mask_area, LV_ALIGN_CENTER, 0, 0);

                        // Setup mask layer
                        HAL_EPIC_LayerConfigInit(&input_layers[input_layer_cnt]);

                        input_layers[input_layer_cnt].alpha = 255;
                        input_layers[input_layer_cnt].x_offset = mask_area.x1;
                        input_layers[input_layer_cnt].y_offset = mask_area.y1;
                        input_layers[input_layer_cnt].color_mode = EPIC_INPUT_A8;
                        input_layers[input_layer_cnt].data = (uint8_t *)mask_img->data;
                        input_layers[input_layer_cnt].ax_mode = ALPHA_BLEND_OVERWRITE;
                        input_layers[input_layer_cnt].width = lv_area_get_width(&mask_area);
                        input_layers[input_layer_cnt].height = lv_area_get_height(&mask_area);
                        input_layers[input_layer_cnt].total_width = mask_img->header.stride;

                        input_layer_cnt++;
                    }
                    else
                    {
                        LV_LOG_WARN("The mask image is not A8/L8 format. Drawing the image without mask.\n");
                    }

                    lv_image_decoder_close(&mask_decoder_dsc);
                }
                else
                {
                    LV_LOG_WARN("Couldn't decode the mask image. Drawing the image without mask.\n");
                }
            }
        }

        int ret = drv_epic_blend(input_layers, input_layer_cnt, &output_canvas, NULL);
        LV_ASSERT(0 == ret);

        if (decoder_res == LV_RESULT_OK)
        {
            lv_image_decoder_close(&mask_decoder_dsc);
        }
    }
}




#endif /*LV_USE_DRAW_EPIC*/
