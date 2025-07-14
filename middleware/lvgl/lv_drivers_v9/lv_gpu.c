/**
  ******************************************************************************
  * @file   lvsf_gpu.c
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
#include "rtconfig.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "board.h"
//#include "EventRecorder.h"
#include "drv_io.h"
#include "drv_flash.h"
#include "drv_epic.h"
#include "log.h"
#include "bf0_pm.h"
#include "section.h"
#include "cpu_usage_profiler.h"
#include "lv_draw_sw.h"
#include "lvsf_perf.h"
#include "lv_draw_epic.h"
#include "lv_epic_utils.h"
#include "lv_conf_sifli.h"
#ifdef USING_EZIPA_DEC
#include "ezipa_dec.h"

static void invalidate_screen(void *data)
{
    lv_obj_invalidate(lv_scr_act());
}






void img_transform(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                   const lv_area_t *src_coords, const lv_area_t *dst_coords,
                   const lv_area_t *output_coords, lv_opa_t opa, lv_color_t ax_color,
                   lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom,
                   lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords,
                   uint8_t type)
{
    EPIC_LayerConfigTypeDef input_layers[3];
    EPIC_LayerConfigTypeDef output_canvas;
    rt_err_t err;

    uint8_t pixel_size;
    uint32_t color_bpp;
    uint8_t input_layer_cnt = 2;


    RT_ASSERT((RT_NULL != src_coords) && (RT_NULL != dst_coords) && (RT_NULL != output_coords));


    /*Setup mask layer*/
    if ((mask_map) && (lv_area_is_on(mask_coords, src_coords)))
    {
#ifdef EPIC_SUPPORT_MASK
        HAL_EPIC_LayerConfigInit(&input_layers[2]);
        input_layers[2].data = (uint8_t *)mask_map;
        input_layers[2].x_offset = mask_coords->x1;
        input_layers[2].y_offset = mask_coords->y1;
        input_layers[2].width = lv_area_get_width(mask_coords);
        input_layers[2].total_width = input_layers[2].width;
        input_layers[2].height = lv_area_get_height(mask_coords);
        if (LV_COLOR_FORMAT_A8 == mask_cf)
            input_layers[2].color_mode = EPIC_INPUT_A8;
        else if (LV_COLOR_FORMAT_A4 == mask_cf)
            input_layers[2].color_mode = EPIC_INPUT_A4;
        else
            LV_ASSERT(false);
        pixel_size = HAL_EPIC_GetColorDepth(input_layers[2].color_mode);
        input_layers[2].data_size = ((pixel_size * input_layers[2].total_width * input_layers[2].height) + 7) >> 3;

        input_layers[2].ax_mode = ALPHA_BLEND_MASK;
        input_layer_cnt = 3;
#else
        LV_ASSERT(0);
#endif /* EPIC_SUPPORT_MASK */
    }


    /*Setup fg layer*/
    HAL_EPIC_LayerConfigInit(&input_layers[1]);



    input_layers[1].x_offset = src_coords->x1;
    input_layers[1].y_offset = src_coords->y1;

    input_layers[1].data = (uint8_t *)src->data;
    input_layers[1].color_mode = lv_img_2_epic_cf(src->header.cf);
    color_bpp = HAL_EPIC_GetColorDepth(input_layers[1].color_mode);

    input_layers[1].width = lv_area_get_width(src_coords);
    input_layers[1].total_width = (color_bpp > 8) ? src->header.w : RT_ALIGN(src->header.w, 8 / color_bpp);
    input_layers[1].height = lv_area_get_height(src_coords);


    input_layers[1].data_size = src->data_size;

#if 0
    if ((LV_IMG_CF_RAW == src->header.cf)
            || (LV_IMG_CF_RAW_ALPHA == src->header.cf))
    {
        /* png image cannot be rotated */
    }
    else if ((LV_IMG_CF_ALPHA_2BIT == src->header.cf)
             || (LV_IMG_CF_ALPHA_4BIT == src->header.cf)
             || (LV_IMG_CF_ALPHA_8BIT == src->header.cf))
    {

        lv_color32_t ax_color_u32;

        ax_color_u32.full = lv_color_to32(ax_color);
        input_layers[1].color_en = true;
        input_layers[1].color_r = ax_color_u32.ch.red;
        input_layers[1].color_g = ax_color_u32.ch.green;
        input_layers[1].color_b = ax_color_u32.ch.blue;
    }
    else if (LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED == src->header.cf)
    {
        lv_color32_t chroma_u32;

        chroma_u32.full = lv_color_to32(LV_COLOR_CHROMA_KEY);
        input_layers[1].color_en = true;
        input_layers[1].color_r = chroma_u32.ch.red;
        input_layers[1].color_g = chroma_u32.ch.green;
        input_layers[1].color_b = chroma_u32.ch.blue;

    }
#if LV_USE_L8_GPU
    else if (LV_IMG_CF_INDEXED_8BIT == src->header.cf)
    {
        input_layers[1].lookup_table = (uint8_t *)src->data;
        input_layers[1].data = (uint8_t *)src->data + (1 << lv_img_cf_get_px_size(LV_IMG_CF_INDEXED_8BIT)) * sizeof(lv_color32_t);
    }
#endif /*LV_USE_L8_GPU == 1*/
#ifdef EPIC_SUPPORT_YUV
    else if (LV_IMG_CF_YUV420_PLANAR == src->header.cf)
    {
        uint32_t xres = RT_ALIGN(src->header.w, 2);
        uint32_t yres = RT_ALIGN(src->header.h, 2);

        input_layers[1].yuv.y_buf = input_layers[1].data;
        input_layers[1].yuv.u_buf = input_layers[1].yuv.y_buf + xres * yres;
        input_layers[1].yuv.v_buf = input_layers[1].yuv.u_buf + xres * yres / 4;
    }
    else if (LV_IMG_CF_YUV420_PLANAR2 == src->header.cf)
    {
        uint8_t **yuv = (uint8_t **)input_layers[1].data;
        input_layers[1].yuv.y_buf = yuv[0];
        input_layers[1].yuv.u_buf = yuv[1];
        input_layers[1].yuv.v_buf = yuv[2];
    }
    else if ((LV_IMG_CF_YUV422_PACKED_YUYV == src->header.cf) || (LV_IMG_CF_YUV422_PACKED_UYVY == src->header.cf))
    {
        input_layers[1].yuv.y_buf = input_layers[1].data;
    }
#endif /* EPIC_SUPPORT_YUV */
#endif
    input_layers[1].transform_cfg.type = type;
    input_layers[1].transform_cfg.angle_adv = angle;
    input_layers[1].transform_cfg.pivot_z = pivot_z;
    input_layers[1].transform_cfg.z_offset = src_z;
    input_layers[1].transform_cfg.pivot_x = pivot->x;
    input_layers[1].transform_cfg.pivot_y = pivot->y;
    //To epic scale
    input_layers[1].transform_cfg.scale_x = LV_IMG_ZOOM_NONE * EPIC_INPUT_SCALE_NONE / (uint32_t)src_zoom;
    input_layers[1].transform_cfg.scale_y = input_layers[1].transform_cfg.scale_x;

    input_layers[1].transform_cfg.vp_x_offset = input_layers[1].transform_cfg.pivot_x;
    input_layers[1].transform_cfg.vp_y_offset = input_layers[1].transform_cfg.pivot_y;
    input_layers[1].transform_cfg.dst_z_offset = input_layers[1].transform_cfg.z_offset;


    /*Setup bg layer*/
    HAL_EPIC_LayerConfigInit(&input_layers[0]);
    input_layers[0].color_en = false;
    input_layers[0].data = (uint8_t *)dest->data;
    input_layers[0].color_mode = lv_img_2_epic_cf(dest->header.cf);
    pixel_size = HAL_EPIC_GetColorDepth(input_layers[0].color_mode);
    RT_ASSERT(0 == (pixel_size % 8));
    pixel_size = pixel_size >> 3;
    input_layers[0].width = lv_area_get_width(dst_coords);
    input_layers[0].total_width = dest->header.w;
    input_layers[0].height = lv_area_get_height(dst_coords);
    input_layers[0].x_offset = dst_coords->x1;
    input_layers[0].y_offset = dst_coords->y1;
    input_layers[0].data_size = pixel_size * input_layers[0].total_width * input_layers[0].height;


    /*Setup output layer*/
    HAL_EPIC_LayerConfigInit(&output_canvas);
    output_canvas.color_en = false;
    output_canvas.data = (uint8_t *)(dest->data + pixel_size * (input_layers[0].total_width * (output_coords->y1 - dst_coords->y1) + (output_coords->x1 - dst_coords->x1)));
    output_canvas.color_mode = input_layers[0].color_mode;
    output_canvas.width = lv_area_get_width(output_coords);
    output_canvas.total_width = input_layers[0].total_width;
    output_canvas.height = lv_area_get_height(output_coords);
    output_canvas.x_offset = output_coords->x1;
    output_canvas.y_offset = output_coords->y1;
    output_canvas.data_size = pixel_size * output_canvas.total_width * output_canvas.height;


    if (1)
        err = drv_epic_transform(&input_layers[0], input_layer_cnt, &output_canvas, NULL);
    else
        err = drv_epic_transform(&input_layers[1], input_layer_cnt - 1, &output_canvas, NULL);

    if (RT_EOK != err)
    {
        lv_async_call(invalidate_screen, NULL);
    }
}
void img_rotate_adv1(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                     const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                     const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                     lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom)
{
    img_transform(dest, src, angle, p_src_coords, p_dst_coords,
                  p_output_coords, opa, ax_color, pivot, pivot_z, src_z, src_zoom,
                  0, NULL, NULL, 1);
}

void img_rotate_adv2(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                     const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                     const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                     lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom)
{
    img_transform(dest, src, angle, p_src_coords, p_dst_coords,
                  p_output_coords, opa, ax_color, pivot, pivot_z, src_z, src_zoom,
                  0, NULL, NULL, 2);
}

extern void HAL_EPIC_Adv_Log(uint32_t level);
void lv_gpu_adv_log(uint32_t level)
{
    HAL_EPIC_Adv_Log(level);
}
int32_t gpu_ezipa_draw(ezipa_obj_t *obj, const lv_area_t *src_area, const lv_area_t *dst_area, bool next)
{
    return 0;
}

#endif /* USING_EAZIP_DEC */

