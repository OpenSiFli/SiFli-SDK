/**
  ******************************************************************************
  * @file   lv_draw_epic_arc.c
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
#include "../lv_image_decoder_private.h"
#include "lv_draw_sw.h"



#include "../../misc/lv_math.h"
#include "../../misc/lv_log.h"
#include "../../stdlib/lv_mem.h"
#include "../../stdlib/lv_string.h"
#include "../lv_draw_private.h"

static void add_circle(const lv_opa_t *circle_mask, const lv_area_t *blend_area, const lv_area_t *circle_area,
                       lv_opa_t *mask_buf,  int32_t width);
static void get_rounded_area(int16_t angle, int32_t radius, uint8_t thickness, lv_area_t *res_area);

/*********************
 *      DEFINES
 *********************/
#define SPLIT_RADIUS_LIMIT 10  /*With radius greater than this the arc will drawn in quarters. A quarter is drawn only if there is arc in it*/
#define SPLIT_ANGLE_GAP_LIMIT 60  /*With small gaps in the arc don't bother with splitting because there is nothing to skip.*/


//#define MULTI_LINE_MASK_ENABLE
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

void lv_draw_epic_arc(lv_draw_unit_t *draw_unit, const lv_draw_arc_dsc_t *dsc, const lv_area_t *coords)
{
    if (dsc->opa <= LV_OPA_MIN) return;
    if (dsc->width == 0) return;
    if (dsc->start_angle == dsc->end_angle) return;

    int32_t width = dsc->width;
    if (width > dsc->radius) width = dsc->radius;

    lv_area_t area_out = *coords;
    lv_area_t clipped_area;
    if (!lv_area_intersect(&clipped_area, &area_out, draw_unit->clip_area)) return;

    /*Draw a full ring*/
    if (dsc->img_src == NULL &&
            (dsc->start_angle + 360 == dsc->end_angle || dsc->start_angle == dsc->end_angle + 360))
    {
        lv_draw_border_dsc_t cir_dsc;
        lv_draw_border_dsc_init(&cir_dsc);
        cir_dsc.opa = dsc->opa;
        cir_dsc.color = dsc->color;
        cir_dsc.width = width;
        cir_dsc.radius = LV_RADIUS_CIRCLE;
        cir_dsc.side = LV_BORDER_SIDE_FULL;
        lv_draw_sw_border(draw_unit, &cir_dsc, &area_out);
        return;
    }

    lv_area_t area_in;
    lv_area_copy(&area_in, &area_out);
    area_in.x1 += dsc->width;
    area_in.y1 += dsc->width;
    area_in.x2 -= dsc->width;
    area_in.y2 -= dsc->width;

    int32_t start_angle = (int32_t)dsc->start_angle;
    int32_t end_angle = (int32_t)dsc->end_angle;
    while (start_angle >= 360) start_angle -= 360;
    while (end_angle >= 360) end_angle -= 360;

    void *mask_list[4] = {0};
    /*Create an angle mask*/
    lv_draw_sw_mask_angle_param_t mask_angle_param;
    lv_draw_sw_mask_angle_init(&mask_angle_param, dsc->center.x, dsc->center.y, start_angle, end_angle);
    mask_list[0] = &mask_angle_param;

    /*Create an outer mask*/
    lv_draw_sw_mask_radius_param_t mask_out_param;
    lv_draw_sw_mask_radius_init(&mask_out_param, &area_out, LV_RADIUS_CIRCLE, false);
    mask_list[1] = &mask_out_param;

    /*Create inner the mask*/
    lv_draw_sw_mask_radius_param_t mask_in_param;
    bool mask_in_param_valid = false;
    if (lv_area_get_width(&area_in) > 0 && lv_area_get_height(&area_in) > 0)
    {
        lv_draw_sw_mask_radius_init(&mask_in_param, &area_in, LV_RADIUS_CIRCLE, true);
        mask_list[2] = &mask_in_param;
        mask_in_param_valid = true;
    }

    int32_t blend_h = lv_area_get_height(&clipped_area);
    int32_t blend_w = lv_area_get_width(&clipped_area);
    int32_t h;

#ifdef MULTI_LINE_MASK_ENABLE
    //Allocate maximum 4KB ping-pong buffer for CPU and GPU paralell
    int32_t  mask_buf_h = LV_MIN(2048 / blend_w, blend_h);
    lv_opa_t *mask_buf = lv_mem_alloc(blend_w * mask_buf_h * 2);
    lv_opa_t *mask_buf_ping_pong = mask_buf;
#else
    lv_opa_t *mask_buf = lv_malloc(blend_w);
#endif /* MULTI_LINE_MASK_ENABLE */

    lv_area_t blend_area = clipped_area;
    lv_area_t img_area;
    lv_draw_sw_blend_dsc_t blend_dsc = {0};
    blend_dsc.mask_buf = mask_buf;
    blend_dsc.opa = dsc->opa;
    blend_dsc.blend_area = &blend_area;
    blend_dsc.mask_area = &blend_area;

    const uint8_t *img_mask = NULL;
    lv_image_decoder_dsc_t decoder_dsc;
    if (dsc->img_src == NULL)
    {
        blend_dsc.color = dsc->color;
    }
    else
    {
        lv_result_t res = lv_image_decoder_open(&decoder_dsc, dsc->img_src, NULL);
        if (res == LV_RESULT_INVALID || decoder_dsc.decoded == NULL)
        {
            LV_LOG_WARN("Can't decode the background image");
            blend_dsc.color = dsc->color;
        }
        else
        {
            img_area.x1 = 0;
            img_area.y1 = 0;
            img_area.x2 = decoder_dsc.decoded->header.w - 1;
            img_area.y2 = decoder_dsc.decoded->header.h - 1;
            int32_t ofs = decoder_dsc.decoded->header.w / 2;
            lv_area_move(&img_area, dsc->center.x - ofs, dsc->center.y - ofs);
            blend_dsc.src_area = &img_area;
            blend_dsc.src_buf = decoder_dsc.decoded->data;
            blend_dsc.src_stride = decoder_dsc.decoded->header.stride;
            blend_dsc.src_color_format = decoder_dsc.decoded->header.cf;
            if (blend_dsc.src_color_format == LV_COLOR_FORMAT_RGB565A8)
            {
                blend_dsc.src_color_format = LV_COLOR_FORMAT_RGB565;
                img_mask = (uint8_t *)blend_dsc.src_buf + blend_dsc.src_stride * lv_area_get_height(blend_dsc.src_area);
            }
        }
    }

    lv_opa_t *circle_mask = NULL;
    lv_area_t round_area_1;
    lv_area_t round_area_2;
    if (dsc->rounded)
    {
        circle_mask = lv_malloc(width * width);
        lv_memset(circle_mask, 0xff, width * width);
        lv_area_t circle_area = {0, 0, width - 1, width - 1};
        lv_draw_sw_mask_radius_param_t circle_mask_param;
        lv_draw_sw_mask_radius_init(&circle_mask_param, &circle_area, width / 2, false);
        void *circle_mask_list[2] = {&circle_mask_param, NULL};

        lv_opa_t *circle_mask_tmp = circle_mask;
        for (h = 0; h < width; h++)
        {
            lv_draw_sw_mask_res_t res = lv_draw_sw_mask_apply(circle_mask_list, circle_mask_tmp, 0, h, width);
            if (res == LV_DRAW_SW_MASK_RES_TRANSP)
            {
                lv_memzero(circle_mask_tmp, width);
            }

            circle_mask_tmp += width;
        }
        get_rounded_area(start_angle, dsc->radius, width, &round_area_1);
        lv_area_move(&round_area_1, dsc->center.x, dsc->center.y);
        get_rounded_area(end_angle, dsc->radius, width, &round_area_2);
        lv_area_move(&round_area_2, dsc->center.x, dsc->center.y);

    }

#ifdef MULTI_LINE_MASK_ENABLE
    lv_area_t mask_area = clipped_area;

    for (int32_t f = 0; f < blend_h;)
    {
        int32_t blend_sub_h = LV_MIN(mask_buf_h, blend_h - f);
        lv_opa_t *mask_buf_sub;
        lv_draw_mask_res_t mask_res = LV_DRAW_MASK_RES_TRANSP;


        mask_area.y1 = blend_area.y1;
        mask_area.y2 = blend_area.y1;

        mask_buf_sub = mask_buf_ping_pong;
        for (h = 0; h < blend_sub_h; h++)
        {
            lv_memset(mask_buf_sub, 0xff, blend_w);
            lv_draw_mask_res_t mask_res_sub = lv_draw_mask_apply(mask_buf_sub, mask_area.x1, mask_area.y1, blend_w);

            if (dsc->rounded)
            {
                if (mask_area.y1 >= round_area_1.y1 && mask_area.y1 <= round_area_1.y2)
                {
                    if (mask_res_sub == LV_DRAW_MASK_RES_TRANSP)
                    {
                        lv_memzero(mask_buf_sub, blend_w);
                        mask_res_sub = LV_DRAW_MASK_RES_CHANGED;
                    }
                    add_circle(circle_mask, &mask_area, &round_area_1, mask_buf_sub, width);
                }
                if (mask_area.y1 >= round_area_2.y1 && mask_area.y1 <= round_area_2.y2)
                {
                    if (mask_res_sub == LV_DRAW_MASK_RES_TRANSP)
                    {
                        lv_memzero(mask_buf_sub, blend_w);
                        mask_res_sub = LV_DRAW_MASK_RES_CHANGED;
                    }
                    add_circle(circle_mask, &mask_area, &round_area_2, mask_buf_sub, width);
                }
            }


            /*If it was an RGB565A8 image use consider its A8 part on the mask*/
            if (img_mask && mask_res_sub != LV_DRAW_SW_MASK_RES_TRANSP)
            {
                const uint8_t *img_mask_tmp = img_mask;
                img_mask_tmp += blend_dsc.src_stride / 2 * (blend_area.y1 - blend_dsc.src_area->y1);
                img_mask_tmp += blend_area.x1 - blend_dsc.src_area->x1;

                int32_t i;
                for (i = 0; i < blend_w; i++)
                {
                    mask_buf[i] = LV_OPA_MIX2(mask_buf[i], img_mask_tmp[i]);
                }
                if (mask_res_sub == LV_DRAW_SW_MASK_RES_FULL_COVER)
                {
                    mask_res_sub = LV_DRAW_SW_MASK_RES_CHANGED;
                }
            }

            if (mask_res_sub != LV_DRAW_MASK_RES_TRANSP)
                mask_res = mask_res_sub;
            else
                lv_memzero(mask_buf_sub, blend_w);

            mask_area.y1 ++;
            mask_area.y2 ++;
            mask_buf_sub += blend_w;
        }



        blend_area.y2 =  blend_area.y1 + blend_sub_h - 1;
        if (mask_res != LV_DRAW_MASK_RES_TRANSP)
        {
            _lv_blend_cont_fill(clip_area, &blend_area,
                                dsc->color, mask_buf_ping_pong, mask_res, dsc->opa, LV_BLEND_MODE_NORMAL);

            if (mask_buf_ping_pong == mask_buf)
                mask_buf_ping_pong = mask_buf + (blend_w * mask_buf_h);
            else
                mask_buf_ping_pong = mask_buf;
        }

        blend_area.y1 += blend_sub_h;
        blend_area.y2 += blend_sub_h;
        f += blend_sub_h;
    }

    _lv_blend_cont_fill_reset();

#else
    blend_area.y2 = blend_area.y1;
    for (h = 0; h < blend_h; h++)
    {
        lv_memset(mask_buf, 0xff, blend_w);
        blend_dsc.mask_res = lv_draw_sw_mask_apply(mask_list, mask_buf, blend_area.x1, blend_area.y1, blend_w);

        if (dsc->rounded)
        {
            if (blend_area.y1 >= round_area_1.y1 && blend_area.y1 <= round_area_1.y2)
            {
                if (blend_dsc.mask_res == LV_DRAW_SW_MASK_RES_TRANSP)
                {
                    lv_memzero(mask_buf, blend_w);
                    blend_dsc.mask_res = LV_DRAW_SW_MASK_RES_CHANGED;
                }
                add_circle(circle_mask, &blend_area, &round_area_1, mask_buf, width);
            }
            if (blend_area.y1 >= round_area_2.y1 && blend_area.y1 <= round_area_2.y2)
            {
                if (blend_dsc.mask_res == LV_DRAW_SW_MASK_RES_TRANSP)
                {
                    lv_memzero(mask_buf, blend_w);
                    blend_dsc.mask_res = LV_DRAW_SW_MASK_RES_CHANGED;
                }
                add_circle(circle_mask, &blend_area, &round_area_2, mask_buf, width);
            }
        }

        /*If it was an RGB565A8 image use consider its A8 part on the mask*/
        if (img_mask && blend_dsc.mask_res != LV_DRAW_SW_MASK_RES_TRANSP)
        {
            const uint8_t *img_mask_tmp = img_mask;
            img_mask_tmp += blend_dsc.src_stride / 2 * (blend_area.y1 - blend_dsc.src_area->y1);
            img_mask_tmp += blend_area.x1 - blend_dsc.src_area->x1;

            int32_t i;
            for (i = 0; i < blend_w; i++)
            {
                mask_buf[i] = LV_OPA_MIX2(mask_buf[i], img_mask_tmp[i]);
            }
            if (blend_dsc.mask_res == LV_DRAW_SW_MASK_RES_FULL_COVER)
            {
                blend_dsc.mask_res = LV_DRAW_SW_MASK_RES_CHANGED;
            }
        }

        lv_epic_draw_blend(draw_unit, &blend_dsc);

        blend_area.y1 ++;
        blend_area.y2 ++;
    }
#endif /* MULTI_LINE_MASK_ENABLE */

    lv_draw_sw_mask_free_param(&mask_angle_param);
    lv_draw_sw_mask_free_param(&mask_out_param);
    if (mask_in_param_valid)
    {
        lv_draw_sw_mask_free_param(&mask_in_param);
    }

    lv_free(mask_buf);
    if (dsc->img_src) lv_image_decoder_close(&decoder_dsc);
    if (circle_mask) lv_free(circle_mask);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void add_circle(const lv_opa_t *circle_mask, const lv_area_t *blend_area, const lv_area_t *circle_area,
                       lv_opa_t *mask_buf,  int32_t width)
{
    lv_area_t circle_common_area;
    if (lv_area_intersect(&circle_common_area, circle_area, blend_area))
    {
        const lv_opa_t *circle_mask_tmp = circle_mask + width * (circle_common_area.y1 - circle_area->y1);
        circle_mask_tmp += circle_common_area.x1 - circle_area->x1;

        lv_opa_t *mask_buf_tmp = mask_buf + circle_common_area.x1 - blend_area->x1;

        uint32_t x;
        uint32_t w = lv_area_get_width(&circle_common_area);
        for (x = 0; x < w; x++)
        {
            uint32_t res = mask_buf_tmp[x] + circle_mask_tmp[x];
            mask_buf_tmp[x] = res > 255 ? 255 : res;
        }
    }

}

static void get_rounded_area(int16_t angle, int32_t radius, uint8_t thickness, lv_area_t *res_area)
{
    int32_t thick_half = thickness / 2;
    uint8_t thick_corr = (thickness & 0x01) ? 0 : 1;

    int32_t cir_x;
    int32_t cir_y;

    cir_x = ((radius - thick_half) * lv_trigo_cos(angle)) >> (LV_TRIGO_SHIFT - 8);
    cir_y = ((radius - thick_half) * lv_trigo_sin(angle)) >> (LV_TRIGO_SHIFT - 8);

    /*The center of the pixel need to be calculated so apply 1/2 px offset*/
    if (cir_x > 0)
    {
        cir_x = (cir_x - 128) >> 8;
        res_area->x1 = cir_x - thick_half + thick_corr;
        res_area->x2 = cir_x + thick_half;
    }
    else
    {
        cir_x = (cir_x + 128) >> 8;
        res_area->x1 = cir_x - thick_half;
        res_area->x2 = cir_x + thick_half - thick_corr;
    }

    if (cir_y > 0)
    {
        cir_y = (cir_y - 128) >> 8;
        res_area->y1 = cir_y - thick_half + thick_corr;
        res_area->y2 = cir_y + thick_half;
    }
    else
    {
        cir_y = (cir_y + 128) >> 8;
        res_area->y1 = cir_y - thick_half;
        res_area->y2 = cir_y + thick_half - thick_corr;
    }
}

#endif /*LV_USE_DRAW_SW*/
