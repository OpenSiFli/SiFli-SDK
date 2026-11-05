/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
#ifdef DRV_EPIC_NEW_API
#include "lv_display_private.h"
#endif
#ifdef USING_EZIPA_DEC
#include "ezipa_dec.h"
#endif

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
#ifndef DRV_EPIC_NEW_API
    EPIC_LayerConfigTypeDef output_canvas;
#endif
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

#ifdef DRV_EPIC_NEW_API
    input_layers[1].alpha = opa;
    err = lv_draw_epic_transform(lv_refr_get_disp_refreshing()->layer_head, &input_layers[1],
                                 input_layer_cnt == 3 ? &input_layers[2] : NULL,
                                 output_coords);
#else

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
#endif

    if (RT_EOK != err)
    {
        lv_async_call(invalidate_screen, NULL);
    }
}
#if defined(DRV_EPIC_NEW_API) && defined(EPIC_SUPPORT_TRANS_MATRIX)
/* Match the v8 matrix projection, using the display as the viewport even when
 * v9 renders into a partial draw buffer. */
static void img_rotate_matrix(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                              const lv_area_t *src_coords, const lv_area_t *dst_coords,
                              const lv_area_t *output_coords, lv_opa_t opa,
                              lv_coord_t pivot_z, lv_coord_t src_z, uint16_t zoom, bool horizontal)
{
    const float fov = 40.0f;
    lv_display_t *display = lv_display_get_default();
    float dst_w = lv_display_get_horizontal_resolution(display);
    float dst_h = lv_display_get_vertical_resolution(display);
    float src_w = lv_area_get_width(src_coords);
    float src_h = lv_area_get_height(src_coords);
    lv_area_t clip;
    if (!zoom || !lv_area_intersect(&clip, dst_coords, output_coords)) return;

    float scale = sifli_get_perspective_scaling_factor(fov);
    float distance = scale * LV_IMG_ZOOM_NONE / zoom;
    float radius = LV_ABS(pivot_z - src_z) / dst_h * 2.0f;
    sifli_matrix_4x4_t matrix;
    sifli_identity_4x4(&matrix);
    sifli_translate_4x4(0, 0, -distance, &matrix);

    sifli_matrix_4x4_t rotation;
    sifli_identity_4x4(&rotation);
    sifli_translate_4x4(0, 0, -radius, &rotation);
    if (horizontal)
        sifli_rotate_y(-angle / 10.0f, &rotation);
    else
        sifli_rotate_x(-angle / 10.0f, &rotation);
    sifli_translate_4x4(0, 0, radius, &rotation);
    sifli_multiply_4x4(&matrix, &rotation);

    float x = (dst_w - src_coords->x1 - src_coords->x2) / dst_h;
    float y = (dst_h - src_coords->y1 - src_coords->y2) / dst_h;
    float w = src_w / dst_h;
    float h = src_h / dst_h;
    sifli_quad_3d_t quad =
    {
        {x - w, y + h, 0}, {x - w, y - h, 0},
        {x + w, y - h, 0}, {x + w, y + h, 0},
    };
    for (uint32_t i = 0; i < 4; i++)
        sifli_mat_vertex_multiply_4x4(&matrix, &quad[i], &quad[i]);

    sifli_matrix_3x3_t transform;
    if (SIFLI_MATRIX_SUCCESS != sifli_get_texture_map_matrix(fov, src_w, src_h,
            dst_w, dst_h, quad, &transform)) return;

    EPIC_LayerConfigTypeDef layer;

    HAL_EPIC_LayerConfigInit(&layer);
    layer.data = (uint8_t *)src->data;
    layer.color_mode = lv_img_2_epic_cf(src->header.cf);
    layer.width = src->header.w;
    layer.height = src->header.h;
    layer.total_width = src->header.w;
    layer.data_size = src->data_size;
    layer.alpha = opa;
    layer.transform_cfg.trans_matrix = &transform;

    if (RT_EOK != lv_draw_epic_transform(lv_refr_get_disp_refreshing()->layer_head, &layer, NULL, &clip))
        lv_async_call(invalidate_screen, NULL);
}
#endif /* DRV_EPIC_NEW_API && EPIC_SUPPORT_TRANS_MATRIX */

void img_rotate_adv1(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                     const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                     const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                     lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom)
{
#if defined(DRV_EPIC_NEW_API) && defined(EPIC_SUPPORT_TRANS_MATRIX)
    img_rotate_matrix(dest, src, angle, p_src_coords, p_dst_coords, p_output_coords,
                      opa, pivot_z, src_z, src_zoom, false);
#else
    img_transform(dest, src, angle, p_src_coords, p_dst_coords,
                  p_output_coords, opa, ax_color, pivot, pivot_z, src_z, src_zoom,
                  0, NULL, NULL, 1);
#endif
}

void img_rotate_adv2(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                     const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                     const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                     lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom)
{
#if defined(DRV_EPIC_NEW_API) && defined(EPIC_SUPPORT_TRANS_MATRIX)
    img_rotate_matrix(dest, src, angle, p_src_coords, p_dst_coords, p_output_coords,
                      opa, pivot_z, src_z, src_zoom, true);
#else
    img_transform(dest, src, angle, p_src_coords, p_dst_coords,
                  p_output_coords, opa, ax_color, pivot, pivot_z, src_z, src_zoom,
                  0, NULL, NULL, 2);
#endif
}

extern void HAL_EPIC_Adv_Log(uint32_t level);
void lv_gpu_adv_log(uint32_t level)
{
    HAL_EPIC_Adv_Log(level);
}

#ifdef USING_EZIPA_DEC
#ifdef RT_USING_PM
    #define GPU_DEVICE_START()   rt_pm_hw_device_start()
    #define GPU_DEVICE_STOP()     rt_pm_hw_device_stop()
#else
    #define GPU_DEVICE_START()
    #define GPU_DEVICE_STOP()
#endif  /* RT_USING_PM */

int32_t gpu_ezipa_draw(ezipa_obj_t *obj, const lv_area_t *src_area, const lv_layer_t *layer, bool next)
{
    int32_t r;

    const lv_area_t *dst_area = &layer->_clip_area;
    lv_draw_buf_t *buf_act = layer->draw_buf;
    const lv_area_t *disp_area = &layer->buf_area;

    // my_gpu_wait(draw_ctx);
    drv_epic_wait_done();

    ezipa_canvas_t canvas;
    canvas.width  = buf_act->header.w;
    canvas.height = buf_act->header.h;
    canvas.buf    = buf_act->data;

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

    // my_gpu_wait(draw_ctx);
    drv_epic_wait_done();

    return r;
}

#endif /* USING_EZIPA_DEC */

