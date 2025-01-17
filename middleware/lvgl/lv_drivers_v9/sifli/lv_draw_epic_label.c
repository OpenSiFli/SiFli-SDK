/**
  ******************************************************************************
  * @file   lv_draw_epic_label.c
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

#include "blend/lv_draw_sw_blend_private.h"
#include "../lv_draw_label_private.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void _draw_epic_letter(lv_draw_unit_t *draw_unit, lv_draw_glyph_dsc_t *glyph_draw_dsc,
                              lv_draw_fill_dsc_t *fill_draw_dsc, const lv_area_t *fill_area);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *  GLOBAL VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/
//#define ENABLE_PRINT_LETTER_MAP
/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_draw_epic_label(lv_draw_unit_t *draw_unit, const lv_draw_label_dsc_t *dsc,
                        const lv_area_t *coords)
{
    if (dsc->opa <= LV_OPA_MIN) return;

    lv_draw_label_iterate_characters(draw_unit, dsc, coords, _draw_epic_letter);

    drv_epic_cont_blend_reset();
}
#ifdef ENABLE_PRINT_LETTER_MAP
#if COMPATIBLE_WITH_SIFLI_EPIC_Ax
void print_letter_map(uint32_t letter, const uint8_t *map_p, uint16_t box_w, uint16_t box_h, uint32_t bpp)
{

    uint32_t bitmask_init;
    uint32_t bitmask;
    int32_t col, row;
    uint32_t col_bit_max = 8 - bpp;
    uint32_t col_bit_row_ofs = 0;

    uint32_t col_bit = 0;

    switch (bpp)
    {
    case 1:
        bitmask_init  = 0x01;
        break;
    case 2:
        bitmask_init  = 0x03;
        break;
    case 4:
        bitmask_init  = 0x0F;
        break;
    case 8:
        bitmask_init  = 0xFF;
        break;       /*No opa table, pixel value will be used directly*/
    default:
        LV_LOG_WARN("lv_draw_letter: invalid bpp");
        return; /*Invalid bpp. Can't render the letter*/
    }


    rt_kprintf("print_letter_map[%c], bpp=%d, wh=%d,%d map=%x \r\n", letter, bpp, box_w, box_h, map_p);

    box_w = ((box_w + (8 / bpp) - 1) / (8 / bpp)) * (8 / bpp); //Align to 1 byte

    for (row = 0 ; row < box_h; row++)
    {

        const uint8_t *col_map_p = map_p;
        bitmask = bitmask_init << col_bit;
        for (col = 0; col < box_w; col++)
        {
            /*Load the pixel's opacity into the mask*/
            uint8_t letter_px = (*map_p & bitmask) >> col_bit;

            if (letter_px)
                if (8 == bpp)
                    rt_kprintf("%x", letter_px >> 4);
                else
                    rt_kprintf("%x", letter_px);
            else
                rt_kprintf(".");



            /*Go to the next column*/
            if (col_bit < col_bit_max)
            {
                col_bit += bpp;
                bitmask = bitmask << bpp;
            }
            else
            {
                col_bit = 0;
                bitmask = bitmask_init;
                map_p++;
            }
        }


        rt_kprintf("(0x%x:", col_map_p);
        do
        {
            rt_kprintf("%02x,", *col_map_p);
            col_map_p++;
        }
        while (col_map_p < map_p);
        rt_kprintf(")");


        rt_kprintf("\r\n");
        col_bit += col_bit_row_ofs;
        map_p += (col_bit >> 3);
        col_bit = col_bit & 0x7;
    }

}
#else
void print_letter_map(uint32_t letter, const uint8_t *map_p, uint16_t box_w, uint16_t box_h, uint32_t bpp)
{

    uint32_t bitmask_init;
    uint32_t bitmask;
    int32_t col, row;
    uint32_t col_bit_max = 8 - bpp;
    uint32_t col_bit_row_ofs = 0;

    uint32_t col_bit = 0;

    switch (bpp)
    {
    case 1:
        bitmask_init  = 0x80;
        break;
    case 2:
        bitmask_init  = 0xC0;
        break;
    case 4:
        bitmask_init  = 0xF0;
        break;
    case 8:
        bitmask_init  = 0xFF;
        break;       /*No opa table, pixel value will be used directly*/
    default:
        LV_LOG_WARN("lv_draw_letter: invalid bpp");
        return; /*Invalid bpp. Can't render the letter*/
    }


    rt_kprintf("print_letter_map[%c], bpp=%d, wh=%d,%d map=%x \r\n", letter, bpp, box_w, box_h, map_p);

    for (row = 0 ; row < box_h; row++)
    {

        const uint8_t *col_map_p = map_p;
        bitmask = bitmask_init >> col_bit;
        for (col = 0; col < box_w; col++)
        {
            /*Load the pixel's opacity into the mask*/
            uint8_t letter_px = (*map_p & bitmask) >> (col_bit_max - col_bit);

            if (letter_px)
                if (8 == bpp)
                    rt_kprintf("%x", letter_px >> 4);
                else
                    rt_kprintf("%x", letter_px);
            else
                rt_kprintf(".");



            /*Go to the next column*/
            if (col_bit < col_bit_max)
            {
                col_bit += bpp;
                bitmask = bitmask >> bpp;
            }
            else
            {
                col_bit = 0;
                bitmask = bitmask_init;
                map_p++;
            }
        }


        rt_kprintf("(0x%x:", col_map_p);
        do
        {
            rt_kprintf("%02x,", *col_map_p);
            col_map_p++;
        }
        while (col_map_p <= map_p);
        rt_kprintf(")");


        rt_kprintf("\r\n");
        col_bit += col_bit_row_ofs;

        col_bit = RT_ALIGN(col_bit, 8);//Align to byte per line

        map_p += (col_bit >> 3);
        col_bit = col_bit & 0x7;
    }

}

#endif /* COMPATIBLE_WITH_SIFLI_EPIC_Ax */
#endif /*ENABLE_PRINT_LETTER_MAP*/
/**********************
 *   STATIC FUNCTIONS
 **********************/

static void _draw_epic_letter(lv_draw_unit_t *draw_unit, lv_draw_glyph_dsc_t *glyph_draw_dsc,
                              lv_draw_fill_dsc_t *fill_draw_dsc, const lv_area_t *fill_area)
{
    if (glyph_draw_dsc)
    {
        if (glyph_draw_dsc->format == LV_FONT_GLYPH_FORMAT_NONE)
        {
#if LV_USE_FONT_PLACEHOLDER
            /* Draw a placeholder rectangle*/
            lv_draw_border_dsc_t border_draw_dsc;
            lv_draw_border_dsc_init(&border_draw_dsc);
            border_draw_dsc.opa = glyph_draw_dsc->opa;
            border_draw_dsc.color = glyph_draw_dsc->color;
            border_draw_dsc.width = 1;
            lv_draw_epic_border(draw_unit, &border_draw_dsc, glyph_draw_dsc->bg_coords);
#endif
        }
        else if ((glyph_draw_dsc->format == LV_FONT_GLYPH_FORMAT_A8)
                 || (glyph_draw_dsc->format == LV_FONT_GLYPH_FORMAT_A4)
                 || (glyph_draw_dsc->format == LV_FONT_GLYPH_FORMAT_A2)
                 || (glyph_draw_dsc->format == LV_FONT_GLYPH_FORMAT_A1))
        {
            /*Do not draw transparent things*/
            if (glyph_draw_dsc->opa <= LV_OPA_MIN)
                return;

            if (LV_FONT_GLYPH_FORMAT_A1 == glyph_draw_dsc->format
#if (0 ==COMPATIBLE_WITH_SIFLI_EPIC_Ax)
                    || LV_FONT_GLYPH_FORMAT_A2 == glyph_draw_dsc->format
                    || LV_FONT_GLYPH_FORMAT_A4 == glyph_draw_dsc->format
#endif /* COMPATIBLE_WITH_SIFLI_EPIC_Ax */
               )
            {
                lv_area_t mask_area = *glyph_draw_dsc->letter_coords;
                mask_area.x2 = mask_area.x1 + lv_draw_buf_width_to_stride(lv_area_get_width(&mask_area), LV_COLOR_FORMAT_A8) - 1;
                lv_draw_sw_blend_dsc_t blend_dsc;
                lv_memzero(&blend_dsc, sizeof(blend_dsc));
                blend_dsc.color = glyph_draw_dsc->color;
                blend_dsc.opa = glyph_draw_dsc->opa;
                lv_draw_buf_t *draw_buf = glyph_draw_dsc->glyph_data;
                blend_dsc.mask_buf = draw_buf->data;
                blend_dsc.mask_area = &mask_area;
                blend_dsc.mask_stride = draw_buf->header.stride;
                blend_dsc.blend_area = glyph_draw_dsc->letter_coords;
                blend_dsc.mask_res = LV_DRAW_SW_MASK_RES_CHANGED;

                lv_draw_sw_blend(draw_unit, &blend_dsc);
            }
            else
            {

                EPIC_LayerConfigTypeDef input_layers[2];
                EPIC_LayerConfigTypeDef output_canvas;


                uint8_t input_layer_cnt = 2;
                lv_draw_buf_t *draw_buf = glyph_draw_dsc->glyph_data;

                LV_EPIC_LOG("glyph_draw_dsc->bitmap %p, color=%u  \r\n", draw_buf->data, lv_color_to_u32(glyph_draw_dsc->color));
                lv_epic_print_area_info("bitmap", glyph_draw_dsc->letter_coords);

#ifdef ENABLE_PRINT_LETTER_MAP
                print_letter_map('c', draw_buf->data,
                                 lv_area_get_width(glyph_draw_dsc->letter_coords),
                                 lv_area_get_height(glyph_draw_dsc->letter_coords),
                                 FT_BPP);
#endif /*ENABLE_PRINT_LETTER_MAP*/
                if (lv_epic_setup_bg_and_output_layer(&input_layers[0], &output_canvas, draw_unit, glyph_draw_dsc->letter_coords))
                    return;/*Fully clipped, nothing to do*/


                /*Setup fg layer*/
                HAL_EPIC_LayerConfigInit(&input_layers[1]);

                input_layers[1].alpha = glyph_draw_dsc->opa;
                input_layers[1].x_offset = glyph_draw_dsc->letter_coords->x1;
                input_layers[1].y_offset = glyph_draw_dsc->letter_coords->y1;


                input_layers[1].data = (uint8_t *)draw_buf->data;
                input_layers[1].color_en = true;
                input_layers[1].color_r = glyph_draw_dsc->color.red;
                input_layers[1].color_g = glyph_draw_dsc->color.green;
                input_layers[1].color_b = glyph_draw_dsc->color.blue;
                input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;
                input_layers[1].width = lv_area_get_width(glyph_draw_dsc->letter_coords);
                input_layers[1].height = lv_area_get_height(glyph_draw_dsc->letter_coords);

                switch (glyph_draw_dsc->format)
                {
                case LV_FONT_GLYPH_FORMAT_A2:
                    input_layers[1].color_mode = EPIC_INPUT_A2;
                    input_layers[1].total_width = RT_ALIGN(input_layers[1].width, 4);
                    break;
                case LV_FONT_GLYPH_FORMAT_A4:
                    input_layers[1].color_mode = EPIC_INPUT_A4;
                    input_layers[1].total_width = RT_ALIGN(input_layers[1].width, 2);
                    break;
                default:
                    input_layers[1].color_mode = EPIC_INPUT_A8;
                    input_layers[1].total_width = input_layers[1].width;
                    break;
                }

                int ret = drv_epic_cont_blend(input_layers, input_layer_cnt, &output_canvas);
                LV_ASSERT(0 == ret);
            }

        }
        else if (glyph_draw_dsc->format == LV_FONT_GLYPH_FORMAT_IMAGE)
        {
#if LV_USE_IMGFONT
            lv_draw_img_dsc_t img_dsc;
            lv_draw_img_dsc_init(&img_dsc);
            img_dsc.rotation = 0;
            img_dsc.scale_x = LV_SCALE_NONE;
            img_dsc.scale_y = LV_SCALE_NONE;
            img_dsc.opa = glyph_draw_dsc->opa;
            img_dsc.src = glyph_draw_dsc->glyph_data;
            lv_draw_epic_img(draw_unit, &img_dsc, glyph_draw_dsc->letter_coords);
#endif
        }
    }

    if (fill_draw_dsc && fill_area)
    {
        lv_draw_epic_fill(draw_unit, fill_draw_dsc, fill_area);
    }
}

#endif /*LV_USE_DRAW_EPIC*/
