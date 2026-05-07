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
#define LV_AREA_TO_EPIC_AREA(p_epic_area, p_lv_area) \
        (p_epic_area)->x0 = (p_lv_area)->x1; \
        (p_epic_area)->y0 = (p_lv_area)->y1; \
        (p_epic_area)->x1 = (p_lv_area)->x2; \
        (p_epic_area)->y1 = (p_lv_area)->y2;

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

uint32_t lv_img_2_epic_cf2(uint32_t cf)
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
    else if (LV_IMG_CF_INDEXED_8BIT == cf)
    {
        color_mode = EPIC_INPUT_L8;
    }
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
extern void set_px_cb_assert(lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                             lv_color_t color, lv_opa_t opa);
static uint32_t get_refreshing_disp_buf_cf(void)
{
    lv_disp_t *disp    = _lv_refr_get_disp_refreshing();
    lv_disp_drv_t *disp_drv = disp->driver;

    if (set_px_cb_rgb565 == disp_drv->set_px_cb)
        return LV_IMG_CF_RGB565;
    else if (set_px_true_color_alpha == disp_drv->set_px_cb)
        return LV_IMG_CF_TRUE_COLOR_ALPHA;
    else if ((NULL == disp_drv->set_px_cb) || (set_px_cb_assert == disp_drv->set_px_cb))
        return LV_IMG_CF_TRUE_COLOR;
    else
        return LV_IMG_CF_UNKNOWN;
}
static void setup_render_buf(drv_epic_render_buf *p_render_buf, struct _lv_draw_ctx_t *draw_ctx)
{
    p_render_buf->cf = lv_img_2_epic_cf2(get_refreshing_disp_buf_cf());
    p_render_buf->data = (uint8_t *)draw_ctx->buf;
    p_render_buf->area.x0 = draw_ctx->buf_area->x1;
    p_render_buf->area.y0 = draw_ctx->buf_area->y1;
    p_render_buf->area.x1 = draw_ctx->buf_area->x2;
    p_render_buf->area.y1 = draw_ctx->buf_area->y2;
}


static void setup_mask_layer(EPIC_LayerConfigTypeDef *p_mask_layer,
                             lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords)
{
#if defined(EPIC_SUPPORT_MONOCHROME_LAYER)&&defined(EPIC_SUPPORT_MASK)
    p_mask_layer->data = (uint8_t *)mask_map;
    p_mask_layer->x_offset = mask_coords->x1;
    p_mask_layer->y_offset = mask_coords->y1;
    p_mask_layer->width = lv_area_get_width(mask_coords);
    p_mask_layer->total_width = p_mask_layer->width;
    p_mask_layer->height = lv_area_get_height(mask_coords);
    if (LV_IMG_CF_ALPHA_8BIT == mask_cf)
        p_mask_layer->color_mode = EPIC_INPUT_A8;
    else if (LV_IMG_CF_ALPHA_4BIT == mask_cf)
        p_mask_layer->color_mode = EPIC_INPUT_A4;
    else
        LV_ASSERT(false);

    p_mask_layer->ax_mode = ALPHA_BLEND_MASK;
    uint8_t pixel_size;
    pixel_size = HAL_EPIC_GetColorDepth(p_mask_layer->color_mode);
    p_mask_layer->data_size = ((pixel_size * p_mask_layer->total_width * p_mask_layer->height) + 7) >> 3;
#else
    LV_ASSERT(0);
#endif /* EPIC_SUPPORT_MONOCHROME_LAYER&&EPIC_SUPPORT_MASK */
}

static void setup_fg_layer(EPIC_LayerConfigTypeDef *p_fg_layer,
                           const lv_img_dsc_t *src, const lv_area_t *src_coords,
                           int16_t angle, int32_t pitch_x, int32_t pitch_y,
                           lv_point_t *pivot, lv_opa_t opa, lv_color_t ax_color,
                           uint16_t src_off_x_frac, uint16_t src_off_y_frac
                          )
{
    uint32_t color_bpp;

    /*
        if (0 == gpu_fraction)
        {
            src_off_x_frac = 0;
            src_off_y_frac = 0;
        }
    */
    angle = angle % 3600;
    if (angle < 0)
    {
        angle += 3600;
    }


    if (pitch_x < 0)
        p_fg_layer->transform_cfg.h_mirror = 1;
    if (pitch_y < 0)
        p_fg_layer->transform_cfg.v_mirror = 1;
    pitch_x = pitch_x > 0 ? pitch_x : -pitch_x;
    pitch_y = pitch_y > 0 ? pitch_y : -pitch_y;
    p_fg_layer->transform_cfg.angle   = angle;
    p_fg_layer->transform_cfg.pivot_x = pivot->x;
    p_fg_layer->transform_cfg.pivot_y = pivot->y;
    p_fg_layer->transform_cfg.scale_x = pitch_x;
    p_fg_layer->transform_cfg.scale_y = pitch_y;

    p_fg_layer->alpha = opa;
    p_fg_layer->x_offset = src_coords->x1;
    p_fg_layer->y_offset = src_coords->y1;

    p_fg_layer->data = (uint8_t *)src->data;
    p_fg_layer->color_mode = lv_img_2_epic_cf(src);
    color_bpp = HAL_EPIC_GetColorDepth(p_fg_layer->color_mode);

    p_fg_layer->width = lv_area_get_width(src_coords);
    p_fg_layer->total_width = (color_bpp > 8) ? src->header.w : RT_ALIGN(src->header.w, 8 / color_bpp);
    p_fg_layer->height = lv_area_get_height(src_coords);
    p_fg_layer->x_offset_frac = src_off_x_frac;
    p_fg_layer->y_offset_frac = src_off_y_frac;

    p_fg_layer->data_size = src->data_size;
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
        p_fg_layer->color_en = true;
        p_fg_layer->color_r = ax_color_u32.ch.red;
        p_fg_layer->color_g = ax_color_u32.ch.green;
        p_fg_layer->color_b = ax_color_u32.ch.blue;
    }
    else if (LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED == src->header.cf)
    {
        lv_color32_t chroma_u32;
        chroma_u32.full = lv_color_to32(LV_COLOR_CHROMA_KEY);

        p_fg_layer->color_en = true;

        p_fg_layer->color_r = chroma_u32.ch.red;
        p_fg_layer->color_g = chroma_u32.ch.green;
        p_fg_layer->color_b = chroma_u32.ch.blue;

    }
#ifdef EPIC_SUPPORT_L8
    else if (LV_IMG_CF_INDEXED_8BIT == src->header.cf)
    {
        const uint8_t *p = src->data;
        p_fg_layer->lookup_table_size = (uint16_t)(*((uint32_t *)p));
        p += 4;
        p_fg_layer->lookup_table = (uint8_t *)p;
        p += p_fg_layer->lookup_table_size * sizeof(lv_color32_t);
        p_fg_layer->data = (uint8_t *)p;
    }
#endif /* EPIC_SUPPORT_L8 */
#ifdef EPIC_SUPPORT_YUV
    else if (LV_IMG_CF_YUV420_PLANAR == src->header.cf)
    {
        uint32_t xres = RT_ALIGN(src->header.w, 2);
        uint32_t yres = RT_ALIGN(src->header.h, 2);

        p_fg_layer->yuv.y_buf = p_fg_layer->data;
        p_fg_layer->yuv.u_buf = p_fg_layer->yuv.y_buf + xres * yres;
        p_fg_layer->yuv.v_buf = p_fg_layer->yuv.u_buf + xres * yres / 4;

        p_fg_layer->dither_level = EPIC_YUV420_DITHER_LEVEL_DEFAULT;
    }
    else if (LV_IMG_CF_YUV420_PLANAR2 == src->header.cf)
    {
        uint8_t **yuv = (uint8_t **)p_fg_layer->data;
        p_fg_layer->yuv.y_buf = yuv[0];
        p_fg_layer->yuv.u_buf = yuv[1];
        p_fg_layer->yuv.v_buf = yuv[2];

        p_fg_layer->dither_level = EPIC_YUV420_DITHER_LEVEL_DEFAULT;
    }
    else if ((LV_IMG_CF_YUV422_PACKED_YUYV == src->header.cf) || (LV_IMG_CF_YUV422_PACKED_UYVY == src->header.cf))
    {
        p_fg_layer->yuv.y_buf = p_fg_layer->data;
    }
#endif /* EPIC_SUPPORT_YUV */

}


static drv_epic_operation *letter_op = NULL;
static void letter_blend_reset(void)
{
    if (letter_op)
    {
        drv_epic_commit_op(letter_op);
        letter_op = NULL;
    }
}


static void letter_blend(lv_img_dsc_t *dest, lv_img_dsc_t *src,
                         const lv_area_t *src_coords, const lv_area_t *dst_coords,
                         const lv_area_t *output_coords, lv_opa_t opa, lv_color_t ax_color,
                         lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords)
{
    lv_area_t clipped_output; //Clip output_coords by src_coords
    lv_color32_t ax_color_u32;



    RT_ASSERT((RT_NULL != src_coords) && (RT_NULL != dst_coords) && (RT_NULL != output_coords));

    if (!_lv_area_intersect(&clipped_output, output_coords, src_coords)) return;

    ax_color_u32.full = lv_color_to32(ax_color);

    if ((letter_op != NULL)
            && (letter_op->desc.label.r == ax_color_u32.ch.red)
            && (letter_op->desc.label.g == ax_color_u32.ch.green)
            && (letter_op->desc.label.b == ax_color_u32.ch.blue))
    {
        EPIC_AreaTypeDef epic_output_area;


        drv_epic_letter_type_t *p_letter = drv_epic_op_alloc_letter(letter_op);
        RT_ASSERT(p_letter);

        p_letter->data = src->data;
        LV_AREA_TO_EPIC_AREA(&p_letter->area, src_coords);
        LV_AREA_TO_EPIC_AREA(&epic_output_area, output_coords);
        HAL_EPIC_AreaJoin(&letter_op->clip_area, &letter_op->clip_area, &epic_output_area);
    }
    else
    {
        letter_blend_reset();

        RT_ASSERT(NULL == letter_op);

        drv_epic_render_buf dst_buf;
        dst_buf.cf = lv_img_2_epic_cf2(dest->header.cf);
        dst_buf.data = (uint8_t *)dest->data;
        dst_buf.area.x0 = dst_coords->x1;
        dst_buf.area.y0 = dst_coords->y1;
        dst_buf.area.x1 = dst_buf.area.x0 + dest->header.w - 1;
        dst_buf.area.y1 = dst_buf.area.y0 + dest->header.h - 1;

        drv_epic_operation *o = drv_epic_alloc_op(&dst_buf);
        RT_ASSERT(o != NULL);
        letter_op = o;

        o->op = DRV_EPIC_DRAW_LETTERS;
        LV_AREA_TO_EPIC_AREA(&o->clip_area, output_coords);


        HAL_EPIC_LayerConfigInit(&o->mask);

        //Setup once only
        if (mask_map)
        {
#if defined(EPIC_SUPPORT_MONOCHROME_LAYER)&&defined(EPIC_SUPPORT_MASK)
            o->mask.data = (uint8_t *)mask_map;
            o->mask.x_offset = mask_coords->x1;
            o->mask.y_offset = mask_coords->y1;
            o->mask.width = lv_area_get_width(mask_coords);
            o->mask.total_width = o->mask.width;
            o->mask.height = lv_area_get_height(mask_coords);
            if (LV_IMG_CF_ALPHA_8BIT == mask_cf)
                o->mask.color_mode = EPIC_INPUT_A8;
            else if (LV_IMG_CF_ALPHA_4BIT == mask_cf)
                o->mask.color_mode = EPIC_INPUT_A4;
            else
                LV_ASSERT(false);

            o->mask.ax_mode = ALPHA_BLEND_MASK;
            uint8_t pixel_size;
            pixel_size = HAL_EPIC_GetColorDepth(o->mask.color_mode);
            o->mask.data_size = ((pixel_size * o->mask.total_width * o->mask.height) + 7) >> 3;
#else
            LV_ASSERT(0);
#endif /* EPIC_SUPPORT_MONOCHROME_LAYER&&EPIC_SUPPORT_MASK */

        }


        o->desc.label.p_letters = NULL;
        o->desc.label.letter_num = 0;



        o->desc.label.opa = opa;
        o->desc.label.color_mode = lv_img_2_epic_cf(src);
        if ((LV_IMG_CF_ALPHA_2BIT == src->header.cf)
                || (LV_IMG_CF_ALPHA_4BIT == src->header.cf)
                || (LV_IMG_CF_ALPHA_8BIT == src->header.cf))
        {
            o->desc.label.r = ax_color_u32.ch.red;
            o->desc.label.g = ax_color_u32.ch.green;
            o->desc.label.b = ax_color_u32.ch.blue;
        }
        else
        {
            LV_ASSERT(0);
        }



        drv_epic_letter_type_t *p_letter = drv_epic_op_alloc_letter(o);
        RT_ASSERT(p_letter);

        p_letter->data = src->data;
        LV_AREA_TO_EPIC_AREA(&p_letter->area, src_coords);
    }
}


void img_rotate_opa_frac2(lv_img_dsc_t *dest, lv_img_dsc_t *src, int16_t angle, int32_t pitch_x, int32_t pitch_y,
                          const lv_area_t *src_coords, const lv_area_t *dst_coords,
                          const lv_area_t *output_coords, lv_point_t *pivot, lv_opa_t opa, lv_color_t ax_color,
                          uint16_t src_off_x_frac, uint16_t src_off_y_frac, uint8_t use_dest_as_bg,
                          lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords)
{
    RT_ASSERT(RT_NULL != pivot);

    RT_ASSERT((RT_NULL != src_coords) && (RT_NULL != dst_coords) && (RT_NULL != output_coords));

    lv_area_t res;
    if (!_lv_area_intersect(&res, dst_coords, output_coords)) return;


    drv_epic_render_buf dst_buf;
    dst_buf.cf = lv_img_2_epic_cf2(dest->header.cf);
    dst_buf.data = (uint8_t *)dest->data;
    dst_buf.area.x0 = dst_coords->x1;
    dst_buf.area.y0 = dst_coords->y1;
    dst_buf.area.x1 = dst_buf.area.x0 + dest->header.w - 1;
    dst_buf.area.y1 = dst_buf.area.y0 + dest->header.h - 1;

    letter_blend_reset();

    drv_epic_operation *o = drv_epic_alloc_op(&dst_buf);
    RT_ASSERT(o != NULL);


    o->op = DRV_EPIC_DRAW_IMAGE;
    LV_AREA_TO_EPIC_AREA(&o->clip_area, output_coords);

    HAL_EPIC_LayerConfigInit(&o->mask);
    if ((mask_map) && (_lv_area_is_on(mask_coords, src_coords)))
    {
        setup_mask_layer(&o->mask, mask_cf, mask_map, mask_coords);
    }

    /*Setup fg layer*/
    HAL_EPIC_LayerConfigInit(&o->desc.blend.layer);
    setup_fg_layer(&o->desc.blend.layer,
                   src, src_coords,
                   angle, pitch_x, pitch_y,
                   pivot, opa, ax_color,
                   src_off_x_frac, src_off_y_frac);


    LV_AREA_TO_EPIC_AREA(&o->clip_area, &res);
    o->desc.blend.use_dest_as_bg = use_dest_as_bg;

    drv_epic_commit_op(o);
}


static void img_transform(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                          const lv_area_t *src_coords, const lv_area_t *dst_coords,
                          const lv_area_t *output_coords, lv_opa_t opa, lv_color_t ax_color,
                          lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom,
                          lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords,
                          uint8_t type, lv_coord_t dst_z, lv_point_t *viewpoint)
{
    RT_ASSERT(RT_NULL != pivot);

    RT_ASSERT((RT_NULL != src_coords) && (RT_NULL != dst_coords) && (RT_NULL != output_coords));

    lv_area_t res;
    if (!_lv_area_intersect(&res, dst_coords, output_coords)) return;


    drv_epic_render_buf dst_buf;
    dst_buf.cf = lv_img_2_epic_cf2(dest->header.cf);
    dst_buf.data = (uint8_t *)dest->data;
    dst_buf.area.x0 = dst_coords->x1;
    dst_buf.area.y0 = dst_coords->y1;
    dst_buf.area.x1 = dst_buf.area.x0 + dest->header.w - 1;
    dst_buf.area.y1 = dst_buf.area.y0 + dest->header.h - 1;

    letter_blend_reset();

    drv_epic_operation *o = drv_epic_alloc_op(&dst_buf);
    RT_ASSERT(o != NULL);


    o->op = DRV_EPIC_DRAW_IMAGE;
    LV_AREA_TO_EPIC_AREA(&o->clip_area, output_coords);

    HAL_EPIC_LayerConfigInit(&o->mask);
    if ((mask_map) && (_lv_area_is_on(mask_coords, src_coords)))
    {
        setup_mask_layer(&o->mask, mask_cf, mask_map, mask_coords);
    }

    /*Setup fg layer*/
    EPIC_LayerConfigTypeDef *p_fg_layer = &o->desc.blend.layer;
    int32_t pitch_x = LV_IMG_ZOOM_NONE * EPIC_INPUT_SCALE_NONE / (uint32_t)src_zoom;
    HAL_EPIC_LayerConfigInit(p_fg_layer);
    setup_fg_layer(p_fg_layer,
                   src, src_coords,
                   0, pitch_x, pitch_x,
                   pivot, opa, ax_color,
                   0, 0);

    p_fg_layer->transform_cfg.type = type;
    p_fg_layer->transform_cfg.angle_adv = angle;
    p_fg_layer->transform_cfg.pivot_z = pivot_z;
    p_fg_layer->transform_cfg.z_offset = src_z;

    if (!viewpoint)
    {
        p_fg_layer->transform_cfg.vp_x_offset = p_fg_layer->transform_cfg.pivot_x;
        p_fg_layer->transform_cfg.vp_y_offset = p_fg_layer->transform_cfg.pivot_y;
        p_fg_layer->transform_cfg.dst_z_offset = dst_z;
    }
    else
    {
        p_fg_layer->transform_cfg.vp_x_offset = viewpoint->x;
        p_fg_layer->transform_cfg.vp_y_offset = viewpoint->y;
        p_fg_layer->transform_cfg.dst_z_offset = dst_z;
    }


    LV_AREA_TO_EPIC_AREA(&o->clip_area, &res);
    o->desc.blend.use_dest_as_bg = true;;

    drv_epic_commit_op(o);
}

void img_rotate_adv1(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                     const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                     const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                     lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom)
{
    img_transform(dest, src, angle, p_src_coords, p_dst_coords,
                  p_output_coords, opa, ax_color, pivot, pivot_z, src_z, src_zoom,
                  0, NULL, NULL, 1, src_z, NULL);
}

void img_rotate_adv2(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                     const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                     const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                     lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom)
{
    img_transform(dest, src, angle, p_src_coords, p_dst_coords,
                  p_output_coords, opa, ax_color, pivot, pivot_z, src_z, src_zoom,
                  0, NULL, NULL, 2, src_z, NULL);
}

void img_rotate_adv2_vp(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                        const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                        const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                        lv_point_t *pivot, lv_coord_t pivot_z, lv_coord_t src_z, uint16_t src_zoom,
                        lv_coord_t dst_z, lv_point_t *viewpoint, lv_area_t *src_new_area)
{
    LV_UNUSED(src_new_area);
    img_transform(dest, src, angle, p_src_coords, p_dst_coords,
                  p_output_coords, opa, ax_color, pivot, pivot_z, src_z, src_zoom,
                  0, NULL, NULL, 2, dst_z, viewpoint);
}



extern void HAL_EPIC_Adv_Log(uint32_t level);
void lv_gpu_adv_log(uint32_t level)
{
    HAL_EPIC_Adv_Log(level);
}

static void img_rotate_opa_frac(lv_img_dsc_t *dest, lv_img_dsc_t *src, int16_t angle, uint32_t zoom,
                                const lv_area_t *src_coords, const lv_area_t *dst_coords,
                                const lv_area_t *output_coords, lv_point_t *pivot, lv_opa_t opa, lv_color_t ax_color,
                                uint16_t src_off_x_frac, uint16_t src_off_y_frac,
                                lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords)
{
    int32_t pitch;

    //To epic scale
    pitch = LV_IMG_ZOOM_NONE * EPIC_INPUT_SCALE_NONE / (uint32_t)zoom;

    img_rotate_opa_frac2(dest, src, angle, pitch, pitch,
                         src_coords, dst_coords,
                         output_coords, pivot, opa, ax_color,
                         src_off_x_frac, src_off_y_frac, 1,
                         mask_cf, mask_map, mask_coords);
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
    if (EPIC_SUPPORTED_CF(cf))
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
        dest.header.cf = get_refreshing_disp_buf_cf();
        dest.data_size = lv_img_buf_get_img_size(dest.header.w, dest.header.h, dest.header.cf);
        dest.header.always_zero = 0;

        lv_opa_t opa = (cf == LV_IMG_CF_ALPHA_1BIT || \
                        cf == LV_IMG_CF_ALPHA_2BIT || \
                        cf == LV_IMG_CF_ALPHA_4BIT || \
                        cf == LV_IMG_CF_ALPHA_8BIT) ? draw_dsc->recolor_opa : draw_dsc->opa;

        {
            img_rotate_opa_frac(&dest, &src, draw_dsc->angle, (uint32_t)draw_dsc->zoom,
                                src_area, draw_ctx->buf_area,
                                draw_ctx->clip_area, (lv_point_t *) & (draw_dsc->pivot), opa, draw_dsc->recolor,
                                draw_dsc->coord_x_frac, draw_dsc->coord_y_frac,
                                mask_cf, mask_map, &mask_coords);

            return;
        }


    }
#endif  /* LV_USE_GPU */

}


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
            LV_DEBUG_ASSERT((LV_DRAW_MASK_TYPE_MAP == mask_param.dsc.type), "type:%llu", mask_param.dsc.type);

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
        dest.header.always_zero = 0;
        dest.header.w = lv_area_get_width(draw_ctx->buf_area);
        dest.header.h = lv_area_get_height(draw_ctx->buf_area);
        dest.header.cf = get_refreshing_disp_buf_cf();
        dest.data_size = lv_img_buf_get_img_size(dest.header.w, dest.header.h, dest.header.cf);

        letter_blend(&dest, &src,
                     &letter_area, draw_ctx->buf_area,
                     draw_ctx->clip_area, dsc->opa, dsc->color,
                     mask_cf, mask_map, &mask_coords);



        return;
    }







//Not support by EPIC, call sw rendering
    LV_ASSERT(0);
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


    if (cdsc == NULL || (!EPIC_SUPPORTED_CF(cdsc->dec_dsc.header.cf)) || (cdsc->dec_dsc.error_msg != NULL))
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

    LV_ASSERT(0);
}

static void draw_bg(lv_draw_ctx_t *draw_ctx, const lv_draw_rect_dsc_t *dsc, const lv_area_t *coords)
{
#if LV_COLOR_SCREEN_TRANSP && LV_COLOR_DEPTH == 32
    RT_ASSERT(0);//Not supported now.
#endif

    /*Draw bg*/
    if (dsc->bg_opa >= LV_OPA_MIN)
    {
        drv_epic_render_buf dst_buf;
        setup_render_buf(&dst_buf, draw_ctx);

        lv_area_t bg_coords;
        lv_area_copy(&bg_coords, coords);

        letter_blend_reset();
        drv_epic_operation *o = drv_epic_alloc_op(&dst_buf);
        RT_ASSERT(o != NULL);
        o->op = DRV_EPIC_DRAW_RECT;

        lv_draw_mask_map_param_t mask_param;
        HAL_EPIC_LayerConfigInit(&o->mask);
        if (lv_draw_mask_is_only_map_mask(&bg_coords, &mask_param))
        {
            LV_ASSERT(LV_DRAW_MASK_TYPE_MAP == mask_param.dsc.type);
            setup_mask_layer(&o->mask, LV_IMG_CF_ALPHA_8BIT, mask_param.cfg.map, &mask_param.cfg.coords);
        }

        LV_AREA_TO_EPIC_AREA(&o->clip_area, draw_ctx->clip_area);
        LV_AREA_TO_EPIC_AREA(&o->desc.rectangle.area, coords);
        o->desc.rectangle.radius = dsc->radius;
        lv_color32_t ax_color_u32;
        ax_color_u32.full = lv_color_to32(dsc->bg_color);
        ax_color_u32.ch.alpha = dsc->bg_opa;
        o->desc.rectangle.argb8888 = ax_color_u32.full;
        drv_epic_commit_op(o);
    }
}



static void draw_border(lv_draw_ctx_t *draw_ctx,
                        const lv_draw_rect_dsc_t *dsc, const lv_area_t *coords)
{
    if (dsc->border_opa <= LV_OPA_MIN) return;
    if (dsc->border_width == 0) return;
    if (dsc->border_side == LV_BORDER_SIDE_NONE) return;
    if (dsc->border_post) return;

    drv_epic_render_buf dst_buf;
    setup_render_buf(&dst_buf, draw_ctx);

    letter_blend_reset();
    drv_epic_operation *o = drv_epic_alloc_op(&dst_buf);
    RT_ASSERT(o != NULL);
    o->op = DRV_EPIC_DRAW_BORDER;

    lv_draw_mask_map_param_t mask_param;
    HAL_EPIC_LayerConfigInit(&o->mask);
    if (lv_draw_mask_is_only_map_mask(coords, &mask_param))
    {
        LV_ASSERT(LV_DRAW_MASK_TYPE_MAP == mask_param.dsc.type);
        setup_mask_layer(&o->mask, LV_IMG_CF_ALPHA_8BIT, mask_param.cfg.map, &mask_param.cfg.coords);
    }

    LV_AREA_TO_EPIC_AREA(&o->clip_area, draw_ctx->clip_area);
    LV_AREA_TO_EPIC_AREA(&o->desc.border.area, coords);
    o->desc.border.radius = dsc->radius;
    o->desc.border.width  = dsc->border_width;
    o->desc.border.top_side = (dsc->border_side & LV_BORDER_SIDE_TOP) ? 1 : 0;
    o->desc.border.bot_side = (dsc->border_side & LV_BORDER_SIDE_BOTTOM) ? 1 : 0;
    o->desc.border.left_side = (dsc->border_side & LV_BORDER_SIDE_LEFT) ? 1 : 0;
    o->desc.border.right_side = (dsc->border_side & LV_BORDER_SIDE_RIGHT) ? 1 : 0;

    lv_color32_t ax_color_u32;
    ax_color_u32.full = lv_color_to32(dsc->border_color);
    ax_color_u32.ch.alpha = dsc->border_opa;
    o->desc.border.argb8888 = ax_color_u32.full;
    drv_epic_commit_op(o);
}

static void draw_rect(struct _lv_draw_ctx_t *draw_ctx, const lv_draw_rect_dsc_t *dsc, const lv_area_t *coords)
{
    /*Draw bg*/
    draw_bg(draw_ctx, dsc, coords);

    /*Draw border*/
    draw_border(draw_ctx, dsc, coords);
}


static void draw_arc(struct _lv_draw_ctx_t *draw_ctx, const lv_draw_arc_dsc_t *dsc, const lv_point_t *center,
                     uint16_t radius,  uint16_t start_angle, uint16_t end_angle)

{
    drv_epic_render_buf dst_buf;
    setup_render_buf(&dst_buf, draw_ctx);

    letter_blend_reset();
    drv_epic_operation *o = drv_epic_alloc_op(&dst_buf);
    RT_ASSERT(o != NULL);

    o->op = DRV_EPIC_DRAW_ARC;
    LV_AREA_TO_EPIC_AREA(&o->clip_area, draw_ctx->clip_area);

    o->desc.arc.center_x = center->x;
    o->desc.arc.center_y = center->y;
    o->desc.arc.start_angle = start_angle;
    o->desc.arc.end_angle = end_angle;
    o->desc.arc.radius = radius;

    lv_color32_t ax_color_u32;
    ax_color_u32.full = lv_color_to32(dsc->color);
    ax_color_u32.ch.alpha = dsc->opa;
    o->desc.arc.argb8888 = ax_color_u32.full;

    o->desc.arc.width = dsc->width;
    o->desc.arc.round_start = dsc->rounded;
    o->desc.arc.round_end = dsc->rounded;

    drv_epic_commit_op(o);
}



static void draw_line(struct _lv_draw_ctx_t *draw_ctx, const lv_draw_line_dsc_t *dsc, const lv_point_t *point1,
                      const lv_point_t *point2)
{
    drv_epic_render_buf dst_buf;
    setup_render_buf(&dst_buf, draw_ctx);
    letter_blend_reset();
    drv_epic_operation *o = drv_epic_alloc_op(&dst_buf);
    RT_ASSERT(o != NULL);

    o->op = DRV_EPIC_DRAW_LINE;
    LV_AREA_TO_EPIC_AREA(&o->clip_area, draw_ctx->clip_area);


    lv_color32_t ax_color_u32;
    ax_color_u32.full = lv_color_to32(dsc->color);
    ax_color_u32.ch.alpha = dsc->opa;
    o->desc.line.argb8888 = ax_color_u32.full;
    o->desc.line.width = dsc->width;
    o->desc.line.dash_width = dsc->dash_width;
    o->desc.line.dash_gap = dsc->dash_gap;
    o->desc.line.round_start = dsc->round_start;
    o->desc.line.round_end = dsc->round_end;
    o->desc.line.raw_end = dsc->raw_end;
    o->desc.line.p1.x = point1->x;
    o->desc.line.p1.y = point1->y;
    o->desc.line.p2.x = point2->x;
    o->desc.line.p2.y = point2->y;

    drv_epic_commit_op(o);
}


static void draw_polygon(struct _lv_draw_ctx_t *draw_ctx, const lv_draw_rect_dsc_t *draw_dsc,
                         const lv_point_t *points, uint16_t point_cnt)
{
    if (points == NULL || DRV_EPIC_POLYGON_POINT_MAX < point_cnt) return;

    /*Join adjacent points if they are on the same coordinate*/
    lv_point_t p[DRV_EPIC_POLYGON_POINT_MAX];
    uint16_t i;
    uint16_t pcnt = 0;
    p[0] = points[0];
    for (i = 0; i < point_cnt - 1; i++)
    {
        if (points[i].x != points[i + 1].x || points[i].y != points[i + 1].y)
        {
            p[pcnt] = points[i];
            pcnt++;
        }
    }
    /*The first and the last points are also adjacent*/
    if (points[0].x != points[point_cnt - 1].x || points[0].y != points[point_cnt - 1].y)
    {
        p[pcnt] = points[point_cnt - 1];
        pcnt++;
    }

    point_cnt = pcnt;
    if (point_cnt < 3) return;

    lv_area_t poly_coords = {.x1 = LV_COORD_MAX, .y1 = LV_COORD_MAX, .x2 = LV_COORD_MIN, .y2 = LV_COORD_MIN};

    for (i = 0; i < point_cnt; i++)
    {
        poly_coords.x1 = LV_MIN(poly_coords.x1, p[i].x);
        poly_coords.y1 = LV_MIN(poly_coords.y1, p[i].y);
        poly_coords.x2 = LV_MAX(poly_coords.x2, p[i].x);
        poly_coords.y2 = LV_MAX(poly_coords.y2, p[i].y);
    }

    lv_area_t clip_area;
    if (!_lv_area_intersect(&clip_area, &poly_coords, draw_ctx->clip_area))
        return;

    drv_epic_render_buf dst_buf;
    setup_render_buf(&dst_buf, draw_ctx);
    letter_blend_reset();
    drv_epic_operation *o = drv_epic_alloc_op(&dst_buf);
    RT_ASSERT(o != NULL);

    o->op = DRV_EPIC_DRAW_POLYGON;
    LV_AREA_TO_EPIC_AREA(&o->clip_area, &clip_area);

    lv_color32_t ax_color_u32;
    ax_color_u32.full = lv_color_to32(draw_dsc->bg_color);
    ax_color_u32.ch.alpha = draw_dsc->bg_opa;
    o->desc.polygon.argb8888 = ax_color_u32.full;
    o->desc.polygon.point_cnt = point_cnt;
    for (uint16_t i = 0; i < point_cnt; i++)
    {
        o->desc.polygon.points[i].x = points[i].x;
        o->desc.polygon.points[i].y = points[i].y;
    }
    drv_epic_commit_op(o);

}


void my_gpu_wait(lv_draw_ctx_t *draw_ctx)
{

    letter_blend_reset();

    // check_gpu_done2();

    // /*Call SW renderer's wait callback too*/
    // lv_draw_sw_wait_for_finish(draw_ctx);
}

static void draw_ctx_setup_gpu(lv_draw_ctx_t *p_draw_ctx, bool en)
{
#if LV_USE_GPU
    epic_draw_ctx_t *my_draw_ctx = (epic_draw_ctx_t *)p_draw_ctx;
    lv_draw_sw_ctx_t *draw_ctx = &my_draw_ctx->sw_ctx;

    if (draw_ctx->base_draw.wait_for_finish)
        draw_ctx->base_draw.wait_for_finish(&draw_ctx->base_draw);

    if (en)
    {
        my_draw_ctx->sw_ctx.blend = draw_blend;
        my_draw_ctx->sw_ctx.base_draw.draw_rect = draw_rect;
        my_draw_ctx->sw_ctx.base_draw.draw_arc = draw_arc;
        my_draw_ctx->sw_ctx.base_draw.draw_line = draw_line;
        my_draw_ctx->sw_ctx.base_draw.draw_polygon = draw_polygon;
        my_draw_ctx->sw_ctx.base_draw.draw_img = my_decode_and_draw;//draw_img;
        my_draw_ctx->sw_ctx.base_draw.wait_for_finish = my_gpu_wait;
        my_draw_ctx->sw_ctx.base_draw.draw_letter = draw_letter;
        my_draw_ctx->sw_ctx.base_draw.draw_bg = draw_bg;
    }
    else
    {
        my_draw_ctx->sw_ctx.blend = my_draw_ctx->sw_ctx_backup.blend;
        my_draw_ctx->sw_ctx.base_draw.draw_rect = my_draw_ctx->sw_ctx_backup.base_draw.draw_rect;
        my_draw_ctx->sw_ctx.base_draw.draw_arc = my_draw_ctx->sw_ctx_backup.base_draw.draw_arc;
        my_draw_ctx->sw_ctx.base_draw.draw_line = my_draw_ctx->sw_ctx_backup.base_draw.draw_line;
        my_draw_ctx->sw_ctx.base_draw.draw_polygon = my_draw_ctx->sw_ctx_backup.base_draw.draw_polygon;
        my_draw_ctx->sw_ctx.base_draw.draw_img = my_draw_ctx->sw_ctx_backup.base_draw.draw_img;
        my_draw_ctx->sw_ctx.base_draw.wait_for_finish = my_draw_ctx->sw_ctx_backup.base_draw.wait_for_finish;
        my_draw_ctx->sw_ctx.base_draw.draw_letter = my_draw_ctx->sw_ctx_backup.base_draw.draw_letter;
        my_draw_ctx->sw_ctx.base_draw.draw_bg = my_draw_ctx->sw_ctx_backup.base_draw.draw_bg;
    }
#endif
}


uint32_t lv_gpu_render_start(lv_disp_drv_t *disp_drv)
{
    uint32_t draw_buf_cf;
    if (set_px_cb_rgb565 == disp_drv->set_px_cb)
        draw_buf_cf = LV_IMG_CF_RGB565;
    else if (set_px_true_color_alpha == disp_drv->set_px_cb)
        draw_buf_cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    else if (NULL == disp_drv->set_px_cb)
        draw_buf_cf = LV_IMG_CF_TRUE_COLOR;
    else
        draw_buf_cf = LV_IMG_CF_UNKNOWN;

    if (LV_IMG_CF_UNKNOWN != draw_buf_cf)
    {
        drv_epic_render_buf render_buf;
        EPIC_AreaTypeDef ow_area;
        drv_epic_render_list_t p_rd_list;

        render_buf.cf = lv_img_2_epic_cf2(draw_buf_cf);
        render_buf.data = disp_drv->draw_buf->buf_act;
        lv_area_to_EPIC_area(disp_drv->draw_ctx->buf_area, &render_buf.area);
        p_rd_list = drv_epic_alloc_render_list(&render_buf, &ow_area);
        RT_ASSERT(p_rd_list != NULL);

        RT_ASSERT(!HAL_EPIC_AreaIsValid(&ow_area));

        return (uint32_t)p_rd_list;
    }
    else //Unsupported destination format, use software render
    {
        draw_ctx_setup_gpu(disp_drv->draw_ctx, false);
        return 0;
    }
}
/*
    p_area - the real area of 'rl->dst'
*/
void lv_gpu_render_end(lv_disp_drv_t *disp_drv, uint32_t rl, lv_area_t *p_area)
{
    if (rl != 0)
    {
        EPIC_MsgTypeDef msg;

        letter_blend_reset(); //Commit last letter blend if present

        msg.id = EPIC_MSG_RENDER_TO_BUF;
        msg.render_list = (drv_epic_render_list_t) rl;
        if (!p_area) p_area = disp_drv->draw_ctx->buf_area; //Use buf area if no scaling
        lv_area_to_EPIC_area(p_area, &msg.content.r2b.dst_area);
        msg.content.r2b.done_cb = NULL;
        msg.content.r2b.usr_data = NULL;
        drv_epic_render_msg_commit(&msg);
    }
    else//Restore GPU render
    {
        draw_ctx_setup_gpu(disp_drv->draw_ctx, true);
    }

}


void lv_gpu_epic_ctx_init(lv_disp_drv_t *drv, lv_draw_ctx_t *draw_ctx)
{
    /*Initialize the parent type first */
    lv_draw_sw_init_ctx(drv, draw_ctx);


    /*Change some callbacks*/
    epic_draw_ctx_t *my_draw_ctx = (epic_draw_ctx_t *)draw_ctx;

    memcpy(&my_draw_ctx->sw_ctx_backup, &my_draw_ctx->sw_ctx, sizeof(lv_draw_sw_ctx_t));

    draw_ctx_setup_gpu(draw_ctx, true);
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
    draw_ctx_setup_gpu(lv_disp_get_default()->driver->draw_ctx, en);
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
