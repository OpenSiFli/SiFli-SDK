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
#include "log.h"
#include "bf0_pm.h"
#include "section.h"
#include "cpu_usage_profiler.h"
#include "lv_draw_sw.h"
#include "lvsf_perf.h"
#include "lv_gpu_sifli_epic.h"




#if LV_USE_GPU
#include "drv_epic.h"


#define GPU_BLEND_EXP_MS     500





#ifdef RT_USING_PM
    #define GPU_DEVICE_START()   rt_pm_hw_device_start()
    #define GPU_DEVICE_STOP()     rt_pm_hw_device_stop()
#else
    #define GPU_DEVICE_START()
    #define GPU_DEVICE_STOP()
#endif  /* RT_USING_PM */


void my_gpu_wait(lv_draw_ctx_t *draw_ctx);


bool EPIC_SUPPORTED_CF(lv_img_cf_t cf)
{
    bool ret;

    switch (cf)
    {
#ifdef HAL_EZIP_MODULE_ENABLED
    case LV_IMG_CF_RAW:
    case LV_IMG_CF_RAW_ALPHA:
#endif /* HAL_EZIP_MODULE_ENABLED */
    case LV_IMG_CF_TRUE_COLOR:
    case LV_IMG_CF_TRUE_COLOR_ALPHA:
    case LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED:
    case LV_IMG_CF_RGB565:
    case LV_IMG_CF_RGB888:
    case LV_IMG_CF_RGBA5658:
    case LV_IMG_CF_RGBA8888:
#ifdef EPIC_SUPPORT_A2
    case LV_IMG_CF_ALPHA_2BIT:
#endif /* EPIC_SUPPORT_A2 */
#ifdef EPIC_SUPPORT_A4
    case LV_IMG_CF_ALPHA_4BIT:
#endif /* EPIC_SUPPORT_A4 */
#ifdef EPIC_SUPPORT_A8
    case LV_IMG_CF_ALPHA_8BIT:
#endif /* EPIC_SUPPORT_A8 */
#ifdef EPIC_SUPPORT_L8
    case LV_IMG_CF_INDEXED_8BIT:
#endif /* EPIC_SUPPORT_L8 */
#ifdef EPIC_SUPPORT_YUV
    case LV_IMG_CF_YUV422_PACKED_YUYV:
    case LV_IMG_CF_YUV422_PACKED_UYVY:
    case LV_IMG_CF_YUV420_PLANAR:
    case LV_IMG_CF_YUV420_PLANAR2:
#endif /* EPIC_SUPPORT_YUV */
        ret = true;
        break;

    default:
        ret = false;
        break;
    }

    return ret;
}

static uint32_t lv_img_2_epic_cf2(uint32_t cf)
{
    uint32_t color_mode;


    if (LV_IMG_CF_TRUE_COLOR_ALPHA == cf)
    {

#if LV_COLOR_DEPTH == 16
        color_mode = EPIC_INPUT_ARGB8565;
#elif LV_COLOR_DEPTH == 24
        color_mode = EPIC_INPUT_ARGB8888;
#else
        color_mode = EPIC_INPUT_ARGB8888;
#endif
    }
    else if ((LV_IMG_CF_RAW == cf) || (LV_IMG_CF_RAW_ALPHA == cf))
    {
        color_mode = EPIC_INPUT_EZIP;
    }
#if LV_USE_L8_GPU
    else if (LV_IMG_CF_INDEXED_8BIT == cf)
    {
        color_mode = EPIC_INPUT_L8;
    }
#endif /* LV_USE_L8_GPU==1 */
    else if (LV_IMG_CF_ALPHA_2BIT == cf)
    {
        color_mode = EPIC_INPUT_A2;
    }
    else if (LV_IMG_CF_ALPHA_4BIT == cf)
    {
        color_mode = EPIC_INPUT_A4;
    }
    else if (LV_IMG_CF_ALPHA_8BIT == cf)
    {
        color_mode = EPIC_INPUT_A8;
    }
    else if ((LV_IMG_CF_TRUE_COLOR == cf)  || (LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED == cf))
    {
#if LV_COLOR_DEPTH == 16
        color_mode = EPIC_INPUT_RGB565;
#elif LV_COLOR_DEPTH == 24
        color_mode = EPIC_INPUT_RGB888;
#else
        color_mode = EPIC_INPUT_ARGB8888;
#endif
    }
    else if (LV_IMG_CF_RGB565 == cf)
    {
        color_mode = EPIC_INPUT_RGB565;
    }
    else if (LV_IMG_CF_RGBA5658 == cf)
    {
        color_mode = EPIC_INPUT_ARGB8565;
    }
    else if (LV_IMG_CF_RGB888 == cf)
    {
        color_mode = EPIC_INPUT_RGB888;
    }
    else if (LV_IMG_CF_RGBA8888 == cf)
    {
        color_mode = EPIC_INPUT_ARGB8888;
    }
#ifdef EPIC_SUPPORT_YUV   //NOT supported YUV format on lvgl v8
    else if (LV_IMG_CF_YUV422_PACKED_YUYV == cf)
    {
        color_mode = EPIC_INPUT_YUV422_PACKED_YUYV;
    }
    else if (LV_IMG_CF_YUV422_PACKED_UYVY == cf)
    {
        color_mode = EPIC_INPUT_YUV422_PACKED_UYVY;
    }
    else if (LV_IMG_CF_YUV420_PLANAR == cf || LV_IMG_CF_YUV420_PLANAR2 == cf)
    {
        color_mode = EPIC_INPUT_YUV420_PLANAR;
    }
#endif /* EPIC_SUPPORT_YUV */
    else
    {
        rt_kprintf("Unknown format %d\r\n", cf);
        RT_ASSERT(0);
    }

    return color_mode;
}

static uint32_t lv_img_2_epic_cf(const lv_img_dsc_t *src)
{
    uint32_t color_mode;


    if (LV_IMG_CF_TRUE_COLOR_ALPHA == src->header.cf)
    {
        if (3 == (src->data_size / src->header.w / src->header.h))
        {
            color_mode = EPIC_INPUT_ARGB8565;
        }
        else if (4 == (src->data_size / src->header.w / src->header.h))
        {
            color_mode = EPIC_INPUT_ARGB8888;
        }
        else //Use default
        {
            color_mode = lv_img_2_epic_cf2(src->header.cf);
        }
    }
    else if ((LV_IMG_CF_TRUE_COLOR == src->header.cf)  || (LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED == src->header.cf))
    {
        if (2 == (src->data_size / src->header.w / src->header.h))
        {
            color_mode = EPIC_INPUT_RGB565;
        }
        else if (3 == (src->data_size / src->header.w / src->header.h))
        {
            color_mode = EPIC_INPUT_RGB888;
        }
        else  //Use default
        {
            color_mode = lv_img_2_epic_cf2(src->header.cf);
        }
    }
    else
    {
        color_mode = lv_img_2_epic_cf2(src->header.cf);
    }


    return color_mode;
}



static void lv_area_to_EPIC_area(const lv_area_t *lv_a, EPIC_AreaTypeDef *epic_a)
{
    epic_a->x0 = (int16_t)lv_a->x1;
    epic_a->x1 = (int16_t)lv_a->x2;
    epic_a->y0 = (int16_t)lv_a->y1;
    epic_a->y1 = (int16_t)lv_a->y2;
}

static void invalidate_screen(void *data)
{
    lv_obj_invalidate(lv_scr_act());
}


void check_gpu_done2(void)
{
    rt_err_t err;

    err = drv_gpu_check_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err)
    {
        lv_async_call(invalidate_screen, NULL);
    }
}

void img_rotate_opa_frac2(lv_img_dsc_t *dest, lv_img_dsc_t *src, int16_t angle, uint32_t pitch_x, uint32_t pitch_y,
                          const lv_area_t *src_coords, const lv_area_t *dst_coords,
                          const lv_area_t *output_coords, lv_point_t *pivot, lv_opa_t opa, lv_color_t ax_color,
                          uint16_t src_off_x_frac, uint16_t src_off_y_frac, uint8_t use_dest_as_bg,
                          lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords)
{
    EPIC_LayerConfigTypeDef input_layers[3];
    EPIC_LayerConfigTypeDef output_canvas;
    HAL_StatusTypeDef ret;
    rt_err_t err;
    EPIC_HandleTypeDef *epic;
    uint8_t pixel_size;
    uint32_t color_bpp;
    uint8_t input_layer_cnt = 2;

    RT_ASSERT(RT_NULL != pivot);

    RT_ASSERT((RT_NULL != src_coords) && (RT_NULL != dst_coords) && (RT_NULL != output_coords));


    angle = angle % 3600;
    if (angle < 0)
    {
        angle += 3600;
    }



    /*Setup mask layer*/
    if ((mask_map) && (_lv_area_is_on(mask_coords, src_coords)))
    {
#ifdef EPIC_SUPPORT_MASK
        HAL_EPIC_LayerConfigInit(&input_layers[2]);
        input_layers[2].data = (uint8_t *)mask_map;
        input_layers[2].x_offset = mask_coords->x1;
        input_layers[2].y_offset = mask_coords->y1;
        input_layers[2].width = lv_area_get_width(mask_coords);
        input_layers[2].total_width = input_layers[2].width;
        input_layers[2].height = lv_area_get_height(mask_coords);
        if (LV_IMG_CF_ALPHA_8BIT == mask_cf)
            input_layers[2].color_mode = EPIC_INPUT_A8;
        else if (LV_IMG_CF_ALPHA_4BIT == mask_cf)
            input_layers[2].color_mode = EPIC_INPUT_A4;
        else
            LV_ASSERT(false);

        input_layers[2].ax_mode = ALPHA_BLEND_MASK;
        pixel_size = HAL_EPIC_GetColorDepth(input_layers[2].color_mode);
        input_layers[2].data_size = ((pixel_size * input_layers[2].total_width * input_layers[2].height) + 7) >> 3;

        input_layer_cnt = 3;
#else
        LV_ASSERT(0);
#endif /* EPIC_SUPPORT_MASK */
    }


    /*Setup fg layer*/
    HAL_EPIC_LayerConfigInit(&input_layers[1]);


    input_layers[1].transform_cfg.angle   = angle;
    input_layers[1].transform_cfg.pivot_x = pivot->x;
    input_layers[1].transform_cfg.pivot_y = pivot->y;
    input_layers[1].transform_cfg.scale_x = pitch_x;
    input_layers[1].transform_cfg.scale_y = pitch_y;

    input_layers[1].alpha = opa;
    input_layers[1].x_offset = src_coords->x1;
    input_layers[1].y_offset = src_coords->y1;

    input_layers[1].data = (uint8_t *)src->data;
    input_layers[1].color_mode = lv_img_2_epic_cf(src);
    color_bpp = HAL_EPIC_GetColorDepth(input_layers[1].color_mode);

    input_layers[1].width = lv_area_get_width(src_coords);
    input_layers[1].total_width = (color_bpp > 8) ? src->header.w : RT_ALIGN(src->header.w, 8 / color_bpp);
    input_layers[1].height = lv_area_get_height(src_coords);
    input_layers[1].x_offset_frac = src_off_x_frac;
    input_layers[1].y_offset_frac = src_off_y_frac;

    input_layers[1].data_size = src->data_size;
    if ((LV_IMG_CF_RAW == src->header.cf)
            || (LV_IMG_CF_RAW_ALPHA == src->header.cf))
    {
        /* png image cannot be rotated */
        RT_ASSERT(0 == angle);
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
#endif /* LV_USE_L8_GPU==1 */
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



    /*Setup bg layer*/
    HAL_EPIC_LayerConfigInit(&input_layers[0]);
    input_layers[0].color_en = false;
    input_layers[0].data = (uint8_t *)dest->data;
    input_layers[0].color_mode = lv_img_2_epic_cf(dest);
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

    if (use_dest_as_bg)
        err = drv_epic_blend(&input_layers[0], input_layer_cnt, &output_canvas, NULL);
    else
        err = drv_epic_blend(&input_layers[1], input_layer_cnt - 1, &output_canvas, NULL);

    if (RT_EOK != err)
    {
        lv_async_call(invalidate_screen, NULL);
    }
}


void img_transform(lv_img_dsc_t *dest, const lv_img_dsc_t *src,
                   const lv_area_t *src_coords, const lv_area_t *dst_coords,
                   const lv_area_t *output_coords, lv_color_t ax_color,
                   uint8_t use_dest_as_bg,
                   lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords,
                   EPIC_TransPath hor_path, EPIC_TransPath ver_path, void *user_data)
{
    EPIC_LayerConfigTypeDef input_layers[3];
    EPIC_LayerConfigTypeDef output_canvas;
    rt_err_t err;

    uint8_t pixel_size;
    uint32_t color_bpp;
    uint8_t input_layer_cnt = 2;


    RT_ASSERT((RT_NULL != src_coords) && (RT_NULL != dst_coords) && (RT_NULL != output_coords));


    /*Setup mask layer*/
    if ((mask_map) && (_lv_area_is_on(mask_coords, src_coords)))
    {
#ifdef EPIC_SUPPORT_MASK
        HAL_EPIC_LayerConfigInit(&input_layers[2]);
        input_layers[2].data = (uint8_t *)mask_map;
        input_layers[2].x_offset = mask_coords->x1;
        input_layers[2].y_offset = mask_coords->y1;
        input_layers[2].width = lv_area_get_width(mask_coords);
        input_layers[2].total_width = input_layers[2].width;
        input_layers[2].height = lv_area_get_height(mask_coords);
        if (LV_IMG_CF_ALPHA_8BIT == mask_cf)
            input_layers[2].color_mode = EPIC_INPUT_A8;
        else if (LV_IMG_CF_ALPHA_4BIT == mask_cf)
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
    input_layers[1].color_mode = lv_img_2_epic_cf(src);
    color_bpp = HAL_EPIC_GetColorDepth(input_layers[1].color_mode);

    input_layers[1].width = lv_area_get_width(src_coords);
    input_layers[1].total_width = (color_bpp > 8) ? src->header.w : RT_ALIGN(src->header.w, 8 / color_bpp);
    input_layers[1].height = lv_area_get_height(src_coords);


    input_layers[1].data_size = src->data_size;
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
#endif /* LV_USE_L8_GPU==1 */
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



    /*Setup bg layer*/
    HAL_EPIC_LayerConfigInit(&input_layers[0]);
    input_layers[0].color_en = false;
    input_layers[0].data = (uint8_t *)dest->data;
    input_layers[0].color_mode = lv_img_2_epic_cf(dest);
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


    if (use_dest_as_bg)
        err = drv_epic_transform(&input_layers[0], input_layer_cnt, &output_canvas, hor_path, ver_path, user_data, NULL);
    else
        err = drv_epic_transform(&input_layers[1], input_layer_cnt - 1, &output_canvas, hor_path, ver_path, user_data, NULL);

    if (RT_EOK != err)
    {
        lv_async_call(invalidate_screen, NULL);
    }
}


void img_rotate_opa_frac(lv_img_dsc_t *dest, lv_img_dsc_t *src, int16_t angle, uint32_t zoom,
                         const lv_area_t *src_coords, const lv_area_t *dst_coords,
                         const lv_area_t *output_coords, lv_point_t *pivot, lv_opa_t opa, lv_color_t ax_color,
                         uint16_t src_off_x_frac, uint16_t src_off_y_frac,
                         lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords)
{
    uint32_t pitch;

    //To epic scale
    pitch = LV_IMG_ZOOM_NONE * EPIC_INPUT_SCALE_NONE / (uint32_t)zoom;

    img_rotate_opa_frac2(dest, src, angle, pitch, pitch,
                         src_coords, dst_coords,
                         output_coords, pivot, opa, ax_color,
                         src_off_x_frac, src_off_y_frac, 1,
                         mask_cf, mask_map, mask_coords);
}

static void fill_color_opa(lv_img_cf_t cf, lv_color_t *dest_buf, lv_coord_t dest_width,
                           const lv_area_t *fill_area, lv_color_t color, lv_opa_t opa,
                           lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords)
{
    rt_err_t err;
    EPIC_AreaTypeDef epic_dst_area, epic_fill_area, epic_mask_area;
    lv_color32_t c;

    uint32_t dst_cf = lv_img_2_epic_cf2(cf);



    uint32_t epic_mask_cf = lv_img_2_epic_cf2(mask_cf);

    c.full = lv_color_to32(color);
    c.ch.alpha = opa;

    lv_area_t mask_area_r = *mask_coords;

    lv_area_move(&mask_area_r, - fill_area->x1, - fill_area->y1);
    lv_area_to_EPIC_area(&mask_area_r, &epic_mask_area);
    lv_area_to_EPIC_area(fill_area, &epic_fill_area);
    epic_dst_area.x0 = 0;
    epic_dst_area.y0 = 0;
    epic_dst_area.x1 = dest_width - 1;
    epic_dst_area.y1 = lv_area_get_height(fill_area) - 1;

    err = drv_epic_fill(dst_cf, (uint8_t *)dest_buf,
                        &epic_dst_area, &epic_fill_area, c.full,
                        epic_mask_cf, (uint8_t *)mask_map, &epic_mask_area, NULL);


    if (RT_EOK != err)
    {
        lv_async_call(invalidate_screen, NULL);
    }

}

void lv_gpu_img_copy(lv_img_dsc_t *dst, lv_img_dsc_t *src,
                     const lv_area_t *src_coords, const lv_area_t *dst_coords,
                     const lv_area_t *output_coords)
{
    rt_err_t err;

    RT_ASSERT(dst->header.cf == src->header.cf);
    RT_ASSERT(src->header.w == lv_area_get_width(src_coords));
    RT_ASSERT(src->header.h == lv_area_get_height(src_coords));
    RT_ASSERT(dst->header.w == lv_area_get_width(dst_coords));
    RT_ASSERT(dst->header.h == lv_area_get_height(dst_coords));

    uint32_t epic_cf = lv_img_2_epic_cf(src);
    EPIC_AreaTypeDef src_area, dst_area, copy_area;


    lv_area_to_EPIC_area(src_coords, &src_area);
    lv_area_to_EPIC_area(dst_coords, &dst_area);
    lv_area_to_EPIC_area(output_coords, &copy_area);

    err = drv_epic_copy(src->data, (uint8_t *)dst->data,
                        &src_area, &dst_area, &copy_area,
                        epic_cf, epic_cf, NULL);

    if (RT_EOK != err)
    {
        lv_async_call(invalidate_screen, NULL);
    }
}


static void draw_img(struct _lv_draw_ctx_t *draw_ctx,
                     const lv_draw_img_dsc_t *draw_dsc,
                     const lv_area_t *src_area, //Src buf coordinates
                     lv_img_decoder_dsc_t *src_dsc)

{
    lv_img_cf_t cf = src_dsc->header.cf;

    //lv_area_t clipped_coords;
    // if (!_lv_area_intersect(&clipped_coords, &src_area, draw_ctx->clip_area)) return;

    lv_draw_mask_map_param_t mask_param;
    lv_img_cf_t mask_cf;
    const lv_opa_t *mask_map;
    lv_area_t mask_coords;
    if (lv_draw_mask_is_only_map_mask(draw_ctx->buf_area, &mask_param))
    {
        LV_ASSERT(LV_DRAW_MASK_TYPE_MAP == mask_param.dsc.type);

        mask_coords = mask_param.cfg.coords;

        //lv_area_move(&mask_coords, src_area->x1, src_area->y1);

        PRINT_AREA("scr_buf", draw_ctx->buf_area);
        PRINT_AREA("scr_clip", draw_ctx->clip_area);
        PRINT_AREA("src_area", src_area);
        PRINT_AREA("msk_area", &mask_coords);

        mask_map = mask_param.cfg.map;
        mask_cf  = LV_IMG_CF_ALPHA_8BIT;
    }
    else
    {
        mask_map = NULL;
    }

#if LV_USE_GPU
    if ((draw_dsc->recolor_opa == LV_OPA_TRANSP)
            //&& (0 == other_mask_cnt)
            //&& disp->driver.gpu_rotate_cb
            //&& disp->driver.gpu_rotate_frac_cb
            && EPIC_SUPPORTED_CF(cf)
       )
    {
        lv_img_dsc_t src;
        lv_img_dsc_t dest;


        //my_gpu_wait(draw_ctx);

        src.data = src_dsc->img_data;
        src.data_size = src_dsc->img_data_size;
        memcpy(&src.header, &src_dsc->header, sizeof(lv_img_header_t));



        dest.data = draw_ctx->buf;
        dest.header.w = lv_area_get_width(draw_ctx->buf_area);
        dest.header.h = lv_area_get_height(draw_ctx->buf_area);
        dest.data_size = dest.header.w * dest.header.h * sizeof(lv_color_t);
        dest.header.cf = LV_IMG_CF_TRUE_COLOR;
        dest.header.always_zero = 0;
#ifdef SOC_SF32LB55X
#define gpu_min_zoom  33

        //LOG_I(">>>>>draw_dsc->zoom %d",draw_dsc->zoom);

        if (draw_dsc->zoom < gpu_min_zoom) //Scale multi times to approaching target zoom
        {
            if ((cf == LV_IMG_CF_RAW_ALPHA) || (cf == LV_IMG_CF_RAW))
            {
                lv_img_dsc_t *tmp_dst_img;
                lv_img_dsc_t *tmp_src_img;
                /*
                                //Target zoom info
                                rotate_print_info(&dest, &src, draw_dsc->angle, (uint32_t)draw_dsc->zoom,
                                                           map_area, disp_area,
                                                           clip_area, &(draw_dsc->pivot), draw_dsc->opa);
                */

                tmp_dst_img = lv_img_buf_alloc(lv_area_get_width(src_area) * gpu_min_zoom / 256,
                                               lv_area_get_height(src_area) * gpu_min_zoom / 256,
                                               (cf == LV_IMG_CF_RAW) ? LV_IMG_CF_TRUE_COLOR : LV_IMG_CF_TRUE_COLOR_ALPHA);

                if (tmp_dst_img)
                {
                    uint16_t pre_zoom = 256;
                    uint16_t left_zoom = draw_dsc->zoom;
                    lv_area_t tmp_src_area = {0, 0, src.header.w - 1, src.header.h - 1};
                    lv_area_t tmp_dst_area = {0, 0, tmp_dst_img->header.w - 1, tmp_dst_img->header.h - 1};
                    lv_point_t tmp_pivot   = {0, 0}; //Pre zoom always use TL pivot
                    tmp_src_img = &src;

                    //Pre scale down image util left_zoom > gpu_min_zoom
                    while (left_zoom < gpu_min_zoom)
                    {
                        img_rotate_opa_frac(tmp_dst_img, tmp_src_img, 0, (uint32_t)gpu_min_zoom,
                                            &tmp_src_area, &tmp_dst_area,
                                            &tmp_dst_area, &tmp_pivot, LV_OPA_COVER, LV_COLOR_CHROMA_KEY,
                                            0, 0, 0, 0, 0);

                        /*

                                                rotate_print_info(tmp_dst_img, tmp_src_img, 0, (uint32_t)gpu_min_zoom,
                                                                               &tmp_src_area, &tmp_dst_area,
                                                                               &tmp_dst_area, &tmp_pivot, LV_OPA_COVER);
                        */
                        my_gpu_wait(draw_ctx);

                        //Prepare next 'gpu_min_zoom' parameters
                        memcpy(&tmp_src_area, &tmp_dst_area, sizeof(tmp_src_area));
                        tmp_dst_area.x2 = (lv_area_get_width(&tmp_dst_area) * gpu_min_zoom / 256) - 1;
                        tmp_dst_area.y2 = (lv_area_get_height(&tmp_dst_area) * gpu_min_zoom / 256) - 1;
                        tmp_src_img = tmp_dst_img; //Use same img to scale down, EPIC supported!

                        left_zoom = left_zoom * 256 / gpu_min_zoom;
                        pre_zoom  = pre_zoom * gpu_min_zoom / 256;
                        //LOG_I("pre_zoom %d, left %d",pre_zoom,left_zoom);
                    }


                    //Scale and blend to target vdb
                    tmp_src_area.x1 = src_area->x1 + (draw_dsc->pivot.x) * (256 - pre_zoom) / 256;
                    tmp_src_area.y1 = src_area->y1 + (draw_dsc->pivot.y) * (256 - pre_zoom) / 256;
                    tmp_src_area.x2 = tmp_src_area.x1 + tmp_dst_img->header.w - 1;
                    tmp_src_area.y2 = tmp_src_area.y1 + tmp_dst_img->header.h - 1;

                    tmp_pivot.x = draw_dsc->pivot.x * pre_zoom / 256;
                    tmp_pivot.y = draw_dsc->pivot.y * pre_zoom / 256;
                    /*

                                        rotate_print_info(&dest, tmp_dst_img, draw_dsc->angle, (uint32_t)left_zoom,
                                                                       &tmp_src_area, disp_area,
                                                                       clip_area, &tmp_pivot, draw_dsc->opa);
                    */
                    img_rotate_opa_frac(&dest, tmp_dst_img, draw_dsc->angle, (uint32_t)left_zoom,
                                        &tmp_src_area, draw_ctx->buf_area,
                                        draw_ctx->clip_area, &tmp_pivot, draw_dsc->opa, LV_COLOR_CHROMA_KEY,
                                        0, 0, mask_cf, mask_map, &mask_coords);

                    my_gpu_wait(draw_ctx);

                    lv_img_buf_free(tmp_dst_img);

                    return;

                }
                else
                {
                    ;//Allocate temporary image fail use software scale anyway, do not to return here.
                }
            }
            else
            {
                ;//Not EZIP image use software scale instead, do not to return here.
            }

        }
        else
#endif /* SOC_BF_A0 */
        {
            img_rotate_opa_frac(&dest, &src, draw_dsc->angle, (uint32_t)draw_dsc->zoom,
                                src_area, draw_ctx->buf_area,
                                draw_ctx->clip_area, (lv_point_t *) & (draw_dsc->pivot), draw_dsc->opa, LV_COLOR_CHROMA_KEY,
                                draw_dsc->coord_x_frac, draw_dsc->coord_y_frac,
                                mask_cf, mask_map, &mask_coords);

            return;
        }


    }
#endif  /* LV_USE_GPU */

}

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
/**
* Draw a letter in the Virtual Display Buffer
* @param pos_p left-top coordinate of the latter
* @param mask_p the letter will be drawn only on this area  (truncated to draw_buf area)
* @param font_p pointer to font
* @param letter a letter to draw
* @param color color of letter
* @param opa opacity of letter (0..255)
*/
static void draw_letter(lv_draw_ctx_t *draw_ctx, const lv_draw_label_dsc_t *dsc,  const lv_point_t *pos_p,
                        uint32_t letter)
{
    lv_font_glyph_dsc_t g;
    bool g_ret = lv_font_get_glyph_dsc(dsc->font, &g, letter, '\0');
    if (g_ret == false)
    {
        /*Add warning if the dsc is not found
         *but do not print warning for non printable ASCII chars (e.g. '\n')*/
        if (letter >= 0x20 &&
                letter != 0xf8ff && /*LV_SYMBOL_DUMMY*/
                letter != 0x200c)   /*ZERO WIDTH NON-JOINER*/
        {
            LV_LOG_WARN("lv_draw_letter: glyph dsc. not found for U+%" PRIX32, letter);

#if LV_USE_FONT_PLACEHOLDER
            /* draw placeholder */
            lv_area_t glyph_coords;
            lv_draw_rect_dsc_t glyph_dsc;
            lv_coord_t begin_x = pos_p->x + g.ofs_x;
            lv_coord_t begin_y = pos_p->y + g.ofs_y;
            lv_area_set(&glyph_coords, begin_x, begin_y, begin_x + g.box_w, begin_y + g.box_h);
            lv_draw_rect_dsc_init(&glyph_dsc);
            glyph_dsc.bg_opa = LV_OPA_MIN;
            glyph_dsc.outline_opa = LV_OPA_MIN;
            glyph_dsc.shadow_opa = LV_OPA_MIN;
            glyph_dsc.bg_img_opa = LV_OPA_MIN;
            glyph_dsc.border_color = dsc->color;
            glyph_dsc.border_width = 1;
            draw_ctx->draw_rect(draw_ctx, &glyph_dsc, &glyph_coords);
#endif
        }
        return;
    }

    /*Don't draw anything if the character is empty. E.g. space*/
    if ((g.box_h == 0) || (g.box_w == 0)) return;

    lv_point_t gpos;
    gpos.x = pos_p->x + g.ofs_x;
    gpos.y = pos_p->y + (dsc->font->line_height - dsc->font->base_line) - g.box_h - g.ofs_y;

    /*If the letter is completely out of mask don't draw it*/
    if (gpos.x + g.box_w < draw_ctx->clip_area->x1 ||
            gpos.x > draw_ctx->clip_area->x2 ||
            gpos.y + g.box_h < draw_ctx->clip_area->y1 ||
            gpos.y > draw_ctx->clip_area->y2)
    {
        return;
    }

    const uint8_t *map_p = lv_font_get_glyph_bitmap(g.resolved_font, letter);
    if (map_p == NULL)
    {
        LV_LOG_WARN("lv_draw_letter: character's bitmap not found");
        return;
    }





    uint32_t bpp = g.bpp;
    if (bpp == 3) bpp = 4;



#if defined(SOC_SF32LB56X)||defined(SOC_SF32LB58X)
    const uint32_t gpu_support_min_bpp = 4;
#else
    const uint32_t gpu_support_min_bpp = 2;
#endif /* SOC_SF32LB56X || SOC_SF32LB58X*/

    lv_disp_t *disp = _lv_refr_get_disp_refreshing();

    if ((lv_draw_mask_get_cnt() < 2)
            && (LV_BLEND_MODE_NORMAL == dsc->blend_mode)
            && (bpp >= gpu_support_min_bpp)
            && (0 == g.resolved_font->subpx)
            && ((NULL == disp->driver->set_px_cb) || (set_px_true_color_alpha == disp->driver->set_px_cb))
       )
    {
        lv_img_cf_t mask_cf;
        const lv_opa_t *mask_map;
        lv_area_t mask_coords;
        lv_draw_mask_map_param_t mask_param;

        lv_area_t letter_area;

        letter_area.x1 = gpos.x;
        letter_area.y1 = gpos.y;
        letter_area.x2 = letter_area.x1 + g.box_w - 1;
        letter_area.y2 = letter_area.y1 + g.box_h - 1;

        if (lv_draw_mask_is_only_map_mask(draw_ctx->clip_area, &mask_param))
        {
            LV_DEBUG_ASSERT((LV_DRAW_MASK_TYPE_MAP == mask_param.dsc.type), "type:%d", mask_param.dsc.type);

            mask_coords = mask_param.cfg.coords;

            //lv_area_move(&mask_coords, src_area->x1, src_area->y1);
//
//            PRINT_AREA("scr_buf", disp_area);
//            PRINT_AREA("scr_clip", clip_area);
//            PRINT_AREA("src_area", letter_area);
//            PRINT_AREA("msk_area", &mask_coords);

            mask_map = mask_param.cfg.map;
            mask_cf  = LV_IMG_CF_ALPHA_8BIT;
        }
        else
        {
            mask_map = NULL;
        }


        lv_img_dsc_t src;
        lv_img_dsc_t dest;
        lv_point_t pivot = {0, 0};



        switch (bpp)
        {
        case 2:
            src.header.cf = LV_IMG_CF_ALPHA_2BIT;
            break;
        case 4:
            src.header.cf = LV_IMG_CF_ALPHA_4BIT;
            break;
        case 8:
            src.header.cf = LV_IMG_CF_ALPHA_8BIT;
            break;       /*No opa table, pixel value will be used directly*/

        default:
            RT_ASSERT(0);
        }
//print_letter_map(letter, map_p,g.box_w,g.box_h,bpp);

// #undef PRINT_AREA
//#define PRINT_AREA(s,area) rt_kprintf("%s \t x1y1=%d,%d  x2y2=%d,%d  \n",s,(area)->x1,(area)->y1,(area)->x2,(area)->y2)


//PRINT_AREA("src_area", &letter_area);
        src.data = map_p;
        src.header.w = g.box_w;
        src.header.h = g.box_h;
        src.data_size = (RT_ALIGN(src.header.w * bpp, 8) >> 3)  * src.header.h;
        src.header.always_zero = 0;

        dest.data = draw_ctx->buf;
        dest.header.w = lv_area_get_width(draw_ctx->buf_area);
        dest.header.h = lv_area_get_height(draw_ctx->buf_area);
        dest.data_size = dest.header.w * dest.header.h * sizeof(lv_color_t);
        dest.header.cf = (set_px_true_color_alpha == disp->driver->set_px_cb) ? LV_IMG_CF_TRUE_COLOR_ALPHA : LV_IMG_CF_TRUE_COLOR;
        dest.header.always_zero = 0;

        img_rotate_opa_frac(&dest, &src, 0, (uint32_t)LV_IMG_ZOOM_NONE,
                            &letter_area, draw_ctx->buf_area,
                            draw_ctx->clip_area, &pivot, dsc->opa, dsc->color, 0, 0,
                            mask_cf, mask_map, &mask_coords);

        return;
    }







//Not support by EPIC, call sw rendering
    epic_draw_ctx_t *my_draw_ctx = (epic_draw_ctx_t *)draw_ctx;


    my_draw_ctx->sw_ctx_backup.base_draw.draw_letter(draw_ctx, dsc,  pos_p, letter);
}


static void my_draw_cleanup(_lv_img_cache_entry_t *cache)
{
    /*Automatically close images with no caching*/
#if LV_IMG_CACHE_DEF_SIZE == 0
    lv_img_decoder_close(&cache->dec_dsc);
#else
    LV_UNUSED(cache);
#endif
}

/*
    This function is copied from decode_and_draw() at lv_draw_img.c
*/
static lv_res_t my_decode_and_draw(lv_draw_ctx_t *draw_ctx, const lv_draw_img_dsc_t *draw_dsc,
                                   const lv_area_t *coords, const void *src)
{
    if (draw_dsc->opa <= LV_OPA_MIN) return LV_RES_OK;

    _lv_img_cache_entry_t *cdsc = _lv_img_cache_open(src, draw_dsc->recolor, draw_dsc->frame_id);

    if (cdsc == NULL || (!EPIC_SUPPORTED_CF(cdsc->dec_dsc.header.cf))) return LV_RES_INV;

    /*
        lv_img_cf_t cf;
        if(lv_img_cf_is_chroma_keyed(cdsc->dec_dsc.header.cf)) cf = LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED;
        else if(lv_img_cf_has_alpha(cdsc->dec_dsc.header.cf)) cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
        else cf = LV_IMG_CF_TRUE_COLOR;
    */
    if (cdsc->dec_dsc.error_msg != NULL)
    {
        LV_LOG_WARN("Image draw error");

        //show_error(draw_ctx, coords, cdsc->dec_dsc.error_msg);
    }
    /*The decoder could open the image and gave the entire uncompressed image.
     *Just draw it!*/
    else if (cdsc->dec_dsc.img_data)
    {
        lv_area_t map_area_rot;
        lv_area_copy(&map_area_rot, coords);
        if (draw_dsc->angle || draw_dsc->zoom != LV_IMG_ZOOM_NONE)
        {
            int32_t w = lv_area_get_width(coords);
            int32_t h = lv_area_get_height(coords);

            _lv_img_buf_get_transformed_area(&map_area_rot, w, h, draw_dsc->angle, draw_dsc->zoom, &draw_dsc->pivot);

            map_area_rot.x1 += coords->x1;
            map_area_rot.y1 += coords->y1;
            map_area_rot.x2 += coords->x1;
            map_area_rot.y2 += coords->y1;
        }

        lv_area_t clip_com; /*Common area of mask and coords*/
        bool union_ok;
        union_ok = _lv_area_intersect(&clip_com, draw_ctx->clip_area, &map_area_rot);
        /*Out of mask. There is nothing to draw so the image is drawn successfully.*/
        if (union_ok == false)
        {
            my_draw_cleanup(cdsc);
            return LV_RES_OK;
        }

        draw_img(draw_ctx, draw_dsc, coords, &cdsc->dec_dsc);
    }


    my_draw_cleanup(cdsc);
    return LV_RES_OK;
}


static void draw_blend(lv_draw_ctx_t *draw_ctx, const lv_draw_sw_blend_dsc_t *dsc)
{
    /*Let's get the blend area which is the intersection of the area to fill and the clip area.*/
    lv_area_t blend_area;
    if (!_lv_area_intersect(&blend_area, dsc->blend_area, draw_ctx->clip_area)) return; /*Fully clipped, nothing to do*/


    lv_draw_mask_map_param_t mask_param;
    lv_img_cf_t mask_cf;
    const lv_opa_t *mask_map;
    lv_area_t *mask_coords;
    bool only_1map_mask;
    if (lv_draw_mask_is_only_map_mask(&blend_area, &mask_param))
    {
        LV_ASSERT(LV_DRAW_MASK_TYPE_MAP == mask_param.dsc.type);

        mask_coords = &mask_param.cfg.coords;
        mask_map = mask_param.cfg.map;
        mask_cf  = LV_IMG_CF_ALPHA_8BIT;

        if (_lv_area_is_in(&blend_area, mask_coords, 0))
        {
            only_1map_mask = true;
        }
        else
        {
            //Mask is small than blend area, NOT supported.
            mask_map = NULL;
            mask_coords = &mask_param.cfg.coords;
            only_1map_mask = false;
        }
    }
    else
    {
        mask_map = NULL;
        mask_coords = &mask_param.cfg.coords;
        only_1map_mask = false;
    }


    if ((dsc->src_buf != NULL) && (dsc->mask_buf == NULL || only_1map_mask) &&
            dsc->blend_mode == LV_BLEND_MODE_NORMAL && lv_area_get_size(&blend_area) > 100)
    {
        lv_point_t pivot = {0, 0};
        lv_img_dsc_t src;
        lv_img_dsc_t dest;

        src.data = (const uint8_t *) dsc->src_buf;
        src.header.w = lv_area_get_width(dsc->blend_area);
        src.header.h = lv_area_get_height(dsc->blend_area);
        src.data_size = src.header.w * src.header.h * sizeof(lv_color_t);
        src.header.cf = LV_IMG_CF_TRUE_COLOR;
        src.header.always_zero = 0;

        dest.data = draw_ctx->buf;
        dest.header.w = lv_area_get_width(draw_ctx->buf_area);
        dest.header.h = lv_area_get_height(draw_ctx->buf_area);
        dest.data_size = dest.header.w * dest.header.h * sizeof(lv_color_t);
        dest.header.cf = LV_IMG_CF_TRUE_COLOR;
        dest.header.always_zero = 0;

        img_rotate_opa_frac(&dest, &src, 0, (uint32_t)LV_IMG_ZOOM_NONE,
                            dsc->blend_area, draw_ctx->buf_area,
                            draw_ctx->clip_area, &pivot, dsc->opa,
                            LV_COLOR_CHROMA_KEY, 0, 0,
                            mask_cf, mask_map, mask_coords);

    }
    /*Fill only non masked, fully opaque, normal blended and not too small areas*/
    else if (dsc->src_buf == NULL && (dsc->mask_buf == NULL || only_1map_mask) &&
             dsc->blend_mode == LV_BLEND_MODE_NORMAL && lv_area_get_size(&blend_area) > 100)
    {

        /*Got the first pixel on the buffer*/
        lv_coord_t dest_stride = lv_area_get_width(draw_ctx->buf_area); /*Width of the destination buffer*/

        /*Make the blend area relative to the buffer*/
        lv_area_move(&blend_area, -draw_ctx->buf_area->x1, -draw_ctx->buf_area->y1);

        /*Call your custom gou fill function to fill blend_area, on dest_buf with dsc->color*/
        fill_color_opa(LV_IMG_CF_TRUE_COLOR, draw_ctx->buf, dest_stride,
                       &blend_area, dsc->color, dsc->opa,
                       mask_cf, mask_map, mask_coords);
    }
    /*Fallback: the GPU doesn't support these settings. Call the SW renderer.*/
    else
    {
        lv_draw_sw_blend_basic(draw_ctx, dsc);
    }
}

void my_gpu_wait(lv_draw_ctx_t *draw_ctx)
{

    check_gpu_done2();

    /*Call SW renderer's wait callback too*/
    lv_draw_sw_wait_for_finish(draw_ctx);
}




void lv_gpu_epic_ctx_init(lv_disp_drv_t *drv, lv_draw_ctx_t *draw_ctx)
{
    /*Initialize the parent type first */
    lv_draw_sw_init_ctx(drv, draw_ctx);


    /*Change some callbacks*/
    epic_draw_ctx_t *my_draw_ctx = (epic_draw_ctx_t *)draw_ctx;

    memcpy(&my_draw_ctx->sw_ctx_backup, &my_draw_ctx->sw_ctx, sizeof(lv_draw_sw_ctx_t));

    my_draw_ctx->sw_ctx.blend = draw_blend;
    my_draw_ctx->sw_ctx.base_draw.draw_img = my_decode_and_draw;//draw_img;
    my_draw_ctx->sw_ctx.base_draw.wait_for_finish = my_gpu_wait;
#if !defined(SOC_SF32LB55X) && LV_USING_FREETYPE_ENGINE
    my_draw_ctx->sw_ctx.base_draw.draw_letter = draw_letter;
#endif /* SOC_SF32LB55X */
}


void lv_gpu_epic_ctx_deinit(lv_disp_drv_t *drv, lv_draw_ctx_t *draw_ctx)
{
    LV_UNUSED(drv);
    LV_UNUSED(draw_ctx);
}


#endif /* LV_USE_GPU */


void lv_gpu_init(lv_disp_drv_t *drv)
{
    /*New draw ctx settings*/
    drv->draw_ctx_init   = lv_gpu_epic_ctx_init;
    drv->draw_ctx_deinit = lv_gpu_epic_ctx_deinit;
    drv->draw_ctx_size   = sizeof(epic_draw_ctx_t);
    /*
    #elif LV_USE_GPU_ARM2D
        driver->draw_ctx_init = lv_draw_arm2d_ctx_init;
        driver->draw_ctx_deinit = lv_draw_arm2d_ctx_init;
        driver->draw_ctx_size = sizeof(lv_draw_arm2d_ctx_t);
    */
}


void lv_gpu_set_enable(bool en)
{
#if LV_USE_GPU
    epic_draw_ctx_t *my_draw_ctx = (epic_draw_ctx_t *)(lv_disp_get_default()->driver->draw_ctx);
    lv_draw_sw_ctx_t *draw_ctx = &my_draw_ctx->sw_ctx;

    if (draw_ctx->base_draw.wait_for_finish)
        draw_ctx->base_draw.wait_for_finish(&draw_ctx->base_draw);

    if (en)
    {
        draw_ctx->blend = draw_blend;
        draw_ctx->base_draw.draw_img = my_decode_and_draw;//draw_img;
        draw_ctx->base_draw.wait_for_finish = my_gpu_wait;
    }
    else
    {
        draw_ctx->blend = lv_draw_sw_blend_basic;
        draw_ctx->base_draw.wait_for_finish = lv_draw_sw_wait_for_finish;
        draw_ctx->base_draw.draw_img = NULL;
    }
#endif
}

bool lv_gpu_is_enabled(void)
{
#if LV_USE_GPU
    epic_draw_ctx_t *my_draw_ctx = (epic_draw_ctx_t *)(lv_disp_get_default()->driver->draw_ctx);
    lv_draw_sw_ctx_t *draw_ctx = &my_draw_ctx->sw_ctx;


    return ((draw_blend == draw_ctx->blend) && (my_decode_and_draw == draw_ctx->base_draw.draw_img)
            && (my_gpu_wait == draw_ctx->base_draw.wait_for_finish));
#else
    return false;
#endif
}



#ifdef USING_EZIPA_DEC
#include "ezipa_dec.h"


int32_t gpu_ezipa_draw(ezipa_obj_t *obj, const lv_area_t *src_area, const lv_area_t *dst_area, bool next)
{
    int32_t r;

    lv_disp_t *disp    = _lv_refr_get_disp_refreshing();
    lv_draw_ctx_t *draw_ctx = disp->driver->draw_ctx;
    const lv_area_t *disp_area = draw_ctx->buf_area;

    my_gpu_wait(draw_ctx);

    ezipa_canvas_t canvas;

    canvas.width  = lv_area_get_width(disp_area);
    canvas.height = lv_area_get_height(disp_area);
    canvas.buf    = draw_ctx->buf;

#if(16 == LV_COLOR_DEPTH)
    canvas.color_fmt =       EPIC_OUTPUT_RGB565;
#elif (24 == LV_COLOR_DEPTH)
    canvas.color_fmt =       EPIC_OUTPUT_RGB888;
#else
#error "Unsupport format!"
#endif

    canvas.x_offset = src_area->x1 - disp_area->x1;
    canvas.y_offset = src_area->y1 - disp_area->y1;

    canvas.mask_x_offset = dst_area->x1 - disp_area->x1;
    canvas.mask_y_offset = dst_area->y1 - disp_area->y1;
    canvas.mask_width  = lv_area_get_width(dst_area);
    canvas.mask_height = lv_area_get_height(dst_area);

    //gpu_lock(DRV_EPIC_COLOR_BLEND, &src2, &src1, &output, NULL);

    GPU_DEVICE_START();

    r = ezipa_draw(obj, &canvas, next);
    RT_ASSERT(0 == r);

    GPU_DEVICE_STOP();

    //gpu_unlock();

    my_gpu_wait(draw_ctx);

    return r;
}

#endif /* USING_EAZIP_DEC */



#ifdef FINSH_USING_MSH
#include <finsh.h>
static rt_err_t gpu_cfg(int argc, char **argv)
{

    if (argc < 2)
    {
        rt_kprintf("gpu_cfg [OPTION] [VALUE]\n");
        rt_kprintf("    OPTION:enable\n");
        return RT_EOK;
    }

    if (strcmp(argv[1], "enable") == 0)
    {
        if (argc > 2) //write
        {
            uint32_t v = strtoul(argv[2], 0, 10);
            lv_gpu_set_enable(v);
        }

        if (lv_gpu_is_enabled())
            rt_kprintf("gpu is enable.\n");
        else
            rt_kprintf("gpu is disable.\n");
    }
    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(gpu_cfg, gpu configuration: gpu switch | compress rate etc.);
MSH_CMD_EXPORT(gpu_cfg, gpu configurattion);
#endif
