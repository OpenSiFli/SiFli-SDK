/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "drv_epic.h"
#if defined(DRV_EPIC_NEW_API) && defined(BSP_USING_EPIC)
#include "drv_epic_private.h"
#include "drv_epic_mask.h"
#include "string.h"

#ifdef DRV_EPIC_NEW_API_DUAL_CORE_ACPU
    EPIC_DrvTypeDef *gp_drv_epic = NULL;
#endif

static void inline _to_mask_area_type(drv_epic_mask_area_t *dst, const EPIC_AreaTypeDef *src)
{
    dst->x1 = src->x0;
    dst->x2 = src->x1;
    dst->y1 = src->y0;
    dst->y2 = src->y1;
}

static bool _layer_IntersectArea(const EPIC_LayerConfigTypeDef *layer0,
                                 const EPIC_AreaTypeDef *p0_area, EPIC_AreaTypeDef *res_area)
{
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;

    /* calculate the intersect region */
    x0 = HAL_MAX(layer0->x_offset, p0_area->x0);
    y0 = HAL_MAX(layer0->y_offset, p0_area->y0);
    x1 = HAL_MIN(layer0->x_offset + layer0->width - 1, p0_area->x1);
    y1 = HAL_MIN(layer0->y_offset + layer0->height - 1, p0_area->y1);

    if ((x0 <= x1) && (y0 <= y1))
    {
        if (res_area != NULL)
        {
            res_area->x0 = x0;
            res_area->y0 = y0;
            res_area->x1 = x1;
            res_area->y1 = y1;
        }
        return true;
    }
    else
    {
        return false;
    }
}

static void _clip_layer(EPIC_LayerConfigTypeDef *p_layer, const EPIC_AreaTypeDef *clip_area)
{
    //Clip output_layer
    if ((clip_area->x0 >= p_layer->x_offset + p_layer->width)
            || (clip_area->y0 >= p_layer->y_offset + p_layer->height))
        return; //No intersection.

    int16_t x_off = clip_area->x0 - p_layer->x_offset;
    int16_t y_off = clip_area->y0 - p_layer->y_offset;
    uint32_t dst_color_bytes = HAL_EPIC_GetColorDepth(p_layer->color_mode) >> 3;

    p_layer->data     += (y_off * p_layer->total_width + x_off) * dst_color_bytes;
    p_layer->x_offset += x_off;
    p_layer->y_offset += y_off;
    p_layer->width    = clip_area->x1 - clip_area->x0 + 1;
    p_layer->height   = clip_area->y1 - clip_area->y0 + 1;
}

static void overwrite_with_circle(const drv_epic_mask_opa_t *circle_mask, const EPIC_AreaTypeDef *blend_area, const EPIC_AreaTypeDef *circle_area,
                                  drv_epic_mask_opa_t *mask_buf,  int32_t width)
{
    EPIC_AreaTypeDef circle_common_area;
    if (HAL_EPIC_AreaIntersect(&circle_common_area, circle_area, blend_area))
    {
        const drv_epic_mask_opa_t *circle_mask_tmp = circle_mask + width * (circle_common_area.y0 - circle_area->y0);
        circle_mask_tmp += circle_common_area.x0 - circle_area->x0;

        drv_epic_mask_opa_t *mask_buf_tmp = mask_buf + circle_common_area.x0 - blend_area->x0;

        uint32_t x;
        uint32_t w = HAL_EPIC_AreaWidth(&circle_common_area);
        memcpy(mask_buf_tmp, circle_mask_tmp, w);
    }

}

static void add_circle(const drv_epic_mask_opa_t *circle_mask, const EPIC_AreaTypeDef *blend_area, const EPIC_AreaTypeDef *circle_area,
                       drv_epic_mask_opa_t *mask_buf,  int32_t width)
{
    EPIC_AreaTypeDef circle_common_area;
    if (HAL_EPIC_AreaIntersect(&circle_common_area, circle_area, blend_area))
    {
        const drv_epic_mask_opa_t *circle_mask_tmp = circle_mask + width * (circle_common_area.y0 - circle_area->y0);
        circle_mask_tmp += circle_common_area.x0 - circle_area->x0;

        drv_epic_mask_opa_t *mask_buf_tmp = mask_buf + circle_common_area.x0 - blend_area->x0;

        uint32_t x;
        uint32_t w = HAL_EPIC_AreaWidth(&circle_common_area);
        for (x = 0; x < w; x++)
        {
            uint32_t res = mask_buf_tmp[x] + circle_mask_tmp[x];
            mask_buf_tmp[x] = res > 255 ? 255 : res;
        }
    }

}



//Fill the area without blending with 'dst' layer always
static void draw_fill(EPIC_LayerConfigTypeDef *dst, EPIC_LayerConfigTypeDef *p_mask_layer,
                      const EPIC_AreaTypeDef *p_fill_area, uint32_t argb8888)
{
    HAL_StatusTypeDef ret;
    EPIC_LayerConfigTypeDef output_layer;
    EPIC_AreaTypeDef com_area;
    uint8_t output_has_alpha = (EPIC_OUTPUT_ARGB8565 == dst->color_mode)
                               || (EPIC_OUTPUT_ARGB8888 == dst->color_mode);

    uint8_t opa = (uint8_t)((argb8888 >> 24) & 0xFF);


    if (!_layer_IntersectArea(dst, p_fill_area, &com_area))
        return;

    if (NULL == p_mask_layer->data) p_mask_layer = NULL; //Disable mask layer if no data

    //Clip output_layer
    memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));
    _clip_layer(&output_layer, &com_area);

    {
        output_layer.color_r = (uint8_t)((argb8888 >> 16) & 0xFF);
        output_layer.color_g = (uint8_t)((argb8888 >> 8) & 0xFF);
        output_layer.color_b = (uint8_t)((argb8888 >> 0) & 0xFF);

        if (p_mask_layer)
        {
            EPIC_LayerConfigTypeDef input_layers[2];

            memcpy(&input_layers[1], p_mask_layer, sizeof(EPIC_LayerConfigTypeDef));

            input_layers[0] = output_layer;
            input_layers[0].data = (uint8_t *) mono_layer_addr;
            input_layers[0].color_mode = EPIC_INPUT_MONO;
            input_layers[0].alpha = opa;
            input_layers[0].ax_mode = ALPHA_BLEND_RGBCOLOR;
            input_layers[0].color_en = true;


            ret =  Call_Hal_Api(HAL_API_BLEND_EX, input_layers, (void *)((uint32_t)2), &output_layer);
        }
#ifndef SF32LB55X
        else if (opa != 255)
        {
            EPIC_LayerConfigTypeDef input_layers[1];

            input_layers[0] = output_layer;
            input_layers[0].data = (uint8_t *) mono_layer_addr;
            input_layers[0].color_mode = EPIC_INPUT_MONO;
            input_layers[0].alpha = opa;
            input_layers[0].ax_mode = ALPHA_BLEND_RGBCOLOR;
            input_layers[0].color_en = true;

            ret =  Call_Hal_Api(HAL_API_BLEND_EX, input_layers, (void *)((uint32_t)1), &output_layer);
        }
#else /* !SF32LB55X */
        else if (opa != 255)
        {
            if (output_has_alpha)
            {
                RT_ASSERT(0 == argb8888);//Only support fill with 0x00000000 on SF32LB55X

                uint32_t dst_color_bytes = HAL_EPIC_GetColorDepth(dst->color_mode) >> 3;
                uint32_t dst_layer_stride = dst->total_width * dst_color_bytes;

                uint16_t output_new_width = output_layer.width * dst_color_bytes / 2;

                //Software fills the bytes that are not 2-bytes aligned
                if (0 != output_layer.width - output_new_width)
                {
                    for (uint32_t y = 0; y < output_layer.height; y++)
                    {
                        uint8_t *p_dst = output_layer.data + y * dst_layer_stride + output_new_width * 2;
                        *p_dst = 0x00;
                    }
                }


                //EPIC fills the bytes that are 2 bytes aligned
                output_layer.color_mode = EPIC_OUTPUT_RGB565;
                output_layer.total_width = dst_layer_stride / 2;
                output_layer.width = output_new_width;

                output_layer.color_en = true;
                ret =  Call_Hal_Api(HAL_API_BLEND_EX, NULL, (void *)0, &output_layer);
            }
            else
            {
                EPIC_LayerConfigTypeDef input_layer = output_layer;

                input_layer.alpha = (0 == opa) ? 255 : (256 - opa);

                output_layer.color_en = true;
                ret =  Call_Hal_Api(HAL_API_BLEND_EX, &input_layer, (void *)1, &output_layer);
            }
        }
#endif /*SF32LB55X */
        else
        {
            output_layer.color_en = true;
            ret =  Call_Hal_Api(HAL_API_BLEND_EX, NULL, (void *)0, &output_layer);
        }
        DRV_EPIC_ASSERT(HAL_OK == ret);

    }
}

static void inline setup_grad_rect_layer(EPIC_LayerConfigTypeDef *p_layer, draw_rect_dsc_t *p_rect_desc)
{
    int32_t area_w = HAL_EPIC_AreaWidth(p_rect_desc->p_rect_area);
    int32_t area_h = HAL_EPIC_AreaHeight(p_rect_desc->p_rect_area);
    area_w = MIN(area_w, EPIC_INPUT_SCALE_NONE);
    area_h = MIN(area_h, EPIC_INPUT_SCALE_NONE);

    HAL_EPIC_LayerConfigInit(p_layer);
    p_layer->data = (uint8_t *)p_rect_desc->p_grad_color;
    p_layer->color_mode = EPIC_COLOR_ARGB8888;
    p_layer->width = 2;
    p_layer->height = 2;
    p_layer->x_offset = p_rect_desc->p_rect_area->x0;
    p_layer->y_offset = p_rect_desc->p_rect_area->y0;
    p_layer->total_width = 2;
    p_layer->color_en = false;
    p_layer->transform_cfg.scale_x = EPIC_INPUT_SCALE_NONE / area_w;
    p_layer->transform_cfg.scale_y = EPIC_INPUT_SCALE_NONE / area_h;
    p_layer->alpha = EPIC_OPA_MAX;
    p_layer->data_size = 2 * 2 * 4;
}

static void draw_rect(EPIC_LayerConfigTypeDef *dst, EPIC_LayerConfigTypeDef *p_mask_layer,
                      const EPIC_AreaTypeDef *p_fill_area, draw_rect_dsc_t *p_rect_desc)
{
    HAL_StatusTypeDef ret;
    EPIC_LayerConfigTypeDef output_layer;
    EPIC_AreaTypeDef com_area;

    uint8_t opa = (uint8_t)((p_rect_desc->argb8888 >> 24) & 0xFF);
    if ((opa <= EPIC_OPA_MIN) && (!p_rect_desc->is_grad)) return;

    if (!_layer_IntersectArea(dst, p_fill_area, &com_area))
        return;

    if (NULL == p_mask_layer->data) p_mask_layer = NULL; //Disable mask layer if no data

    //Clip output_layer
    memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));
    _clip_layer(&output_layer, &com_area);

    {


        output_layer.color_r = (uint8_t)((p_rect_desc->argb8888 >> 16) & 0xFF);
        output_layer.color_g = (uint8_t)((p_rect_desc->argb8888 >> 8) & 0xFF);
        output_layer.color_b = (uint8_t)((p_rect_desc->argb8888 >> 0) & 0xFF);

        if (p_mask_layer)
        {
            EPIC_LayerConfigTypeDef input_layers[3];

            memcpy(&input_layers[2], p_mask_layer, sizeof(EPIC_LayerConfigTypeDef));

            if (p_rect_desc->is_grad)
            {
                setup_grad_rect_layer(&input_layers[1], p_rect_desc);
            }
            else
            {
                input_layers[1] = output_layer;
                input_layers[1].data = (uint8_t *) mono_layer_addr;
                input_layers[1].color_mode = EPIC_INPUT_MONO;
                input_layers[1].alpha = opa;
                input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;
                input_layers[1].color_en = true;
            }


            memcpy(&input_layers[0], dst, sizeof(EPIC_LayerConfigTypeDef));
            ret =  Call_Hal_Api(HAL_API_BLEND_EX, input_layers, (void *)((uint32_t)3), &output_layer);
        }
#ifndef SF32LB55X
        else if ((opa != 255) || p_rect_desc->is_grad)
        {
            EPIC_LayerConfigTypeDef input_layers[2];

            if (p_rect_desc->is_grad)
            {
                setup_grad_rect_layer(&input_layers[1], p_rect_desc);
            }
            else
            {
                input_layers[1] = output_layer;
                input_layers[1].data = (uint8_t *) mono_layer_addr;
                input_layers[1].color_mode = EPIC_INPUT_MONO;
                input_layers[1].alpha = opa;
                input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;
                input_layers[1].color_en = true;
            }

            memcpy(&input_layers[0], dst, sizeof(EPIC_LayerConfigTypeDef));
            ret =  Call_Hal_Api(HAL_API_BLEND_EX, input_layers, (void *)((uint32_t)2), &output_layer);
        }
#else /* !SF32LB55X */
        else if (opa != 255)
        {

            EPIC_LayerConfigTypeDef input_layer = output_layer;

            input_layer.alpha = (0 == opa) ? 255 : (256 - opa);

            output_layer.color_en = true;
            ret =  Call_Hal_Api(HAL_API_BLEND_EX, &input_layer, (void *)1, &output_layer);
        }
#endif /*SF32LB55X */
        else
        {
            output_layer.color_en = true;
            ret =  Call_Hal_Api(HAL_API_BLEND_EX, NULL, (void *)0, &output_layer);
        }
        DRV_EPIC_ASSERT(HAL_OK == ret);

    }
}

static inline void draw_rect2(EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area,
                              EPIC_LayerConfigTypeDef *p_mask_layer,
                              const EPIC_AreaTypeDef *p_fill_area, uint32_t argb8888)
{
    EPIC_AreaTypeDef com_area;
    if (HAL_EPIC_AreaIntersect(&com_area, p_fill_area, p_clip_area))
    {
        draw_rect_dsc_t rect_desc;
        rect_desc.is_grad = 0;
        rect_desc.p_grad_color = NULL;
        rect_desc.p_rect_area = NULL;
        rect_desc.argb8888 = argb8888;

        draw_rect(dst, p_mask_layer, &com_area, &rect_desc);
    }
}

static inline void draw_rect3(EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area,
                              EPIC_LayerConfigTypeDef *p_mask_layer,
                              const EPIC_AreaTypeDef *p_fill_area, draw_rect_dsc_t *p_rect_desc)
{
    EPIC_AreaTypeDef com_area;
    if (HAL_EPIC_AreaIntersect(&com_area, p_fill_area, p_clip_area))
        draw_rect(dst, p_mask_layer, &com_area, p_rect_desc);
}

static rt_err_t draw_masked_rect(EPIC_LayerConfigTypeDef *dst, EPIC_LayerConfigTypeDef *p_extra_mask_layer,
                                 void *masks[], const EPIC_AreaTypeDef *p_draw_area, draw_rect_dsc_t *p_rect_desc,
                                 const EPIC_AreaTypeDef *p_circle_mask_area1,
                                 const EPIC_AreaTypeDef *p_circle_mask_area2,
                                 drv_epic_mask_opa_t *p_circle_mask_buf,
                                 uint32_t circle_mask_width
                                )
{
    HAL_StatusTypeDef ret;
    EPIC_LayerConfigTypeDef *p_rect_color_layer, *p_mask_layer;
    EPIC_LayerConfigTypeDef input_layers[3];
    EPIC_LayerConfigTypeDef output_layer;
    EPIC_AreaTypeDef com_area;

    if (!_layer_IntersectArea(dst, p_draw_area, &com_area))
        return RT_EOK;

    int32_t blend_h = HAL_EPIC_AreaHeight(&com_area);
    int32_t blend_w = HAL_EPIC_AreaWidth(&com_area);

    if (blend_h <= 0 || blend_w <= 0) return RT_EEMPTY;
    if (p_extra_mask_layer && (NULL == p_extra_mask_layer->data)) p_extra_mask_layer = NULL; //Disable mask layer if no data

    //ping-pong buffer for CPU and GPU paralell
    int32_t  mask_buf_h = MIN((gp_drv_epic->dbg_mask_buf_pool_max / 2) / blend_w, blend_h);
    RT_ASSERT(mask_buf_h > 0);
    drv_epic_mask_opa_t *mask_buf = (drv_epic_mask_opa_t *)gp_drv_epic->mask_buf_pool;
    drv_epic_mask_opa_t *mask_buf_ping_pong = mask_buf;

    EPIC_AreaTypeDef mask_area = com_area;
    EPIC_AreaTypeDef fill_area = com_area;



    /*Inital mask layer*/
    p_mask_layer = &input_layers[2];
    HAL_EPIC_LayerConfigInit(p_mask_layer);
    p_mask_layer->color_mode = EPIC_INPUT_A8;
    p_mask_layer->width  = blend_w;
    p_mask_layer->total_width = p_mask_layer->width;
    p_mask_layer->x_offset = fill_area.x0;

    if (p_rect_desc->is_grad)
    {
        p_mask_layer->ax_mode = ALPHA_BLEND_MASK;

        //Setup independed gradient layer
        setup_grad_rect_layer(&input_layers[1], p_rect_desc);
    }
    else
    {
        //Append RGB to mask layer
        p_mask_layer->color_en = true;
        p_mask_layer->ax_mode = ALPHA_BLEND_RGBCOLOR;
        p_mask_layer->alpha = (uint8_t)((p_rect_desc->argb8888 >> 24) & 0xFF);
        p_mask_layer->color_r = (uint8_t)((p_rect_desc->argb8888 >> 16) & 0xFF);
        p_mask_layer->color_g = (uint8_t)((p_rect_desc->argb8888 >> 8) & 0xFF);
        p_mask_layer->color_b = (uint8_t)((p_rect_desc->argb8888 >> 0) & 0xFF);
    }




    /*Inital output layer*/
    memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));
    uint32_t dst_color_bytes = HAL_EPIC_GetColorDepth(dst->color_mode) >> 3;
    output_layer.width    = blend_w;


    /*
        Avoid 'mask_buf_ping_pong' is still using by previous blending,
        befre we overwrite it.
    */
    ret =  Call_Hal_Api(HAL_API_ALL_STOP, NULL, NULL, NULL);
    DRV_EPIC_ASSERT(HAL_OK == ret);


    for (int32_t f = 0; f < blend_h;)
    {
        int32_t fill_h = MIN(mask_buf_h, blend_h - f);
        drv_epic_mask_opa_t *mask_buf_sub;
        drv_epic_mask_res_t mask_res = DRAW_MASK_RES_TRANSP;


        mask_area.y0 = fill_area.y0;
        mask_area.y1 = fill_area.y0;

        mask_buf_sub = mask_buf_ping_pong;
        memset(mask_buf_sub, 0xff, blend_w * fill_h);
        for (int32_t h = 0; h < fill_h; h++)
        {
            drv_epic_mask_res_t mask_res_sub = drv_epic_mask_apply(masks, mask_buf_sub, mask_area.x0, mask_area.y0, blend_w);


            if (p_circle_mask_area1 && p_circle_mask_buf)
                if (mask_area.y0 >= p_circle_mask_area1->y0 && mask_area.y0 <= p_circle_mask_area1->y1)
                {
                    if (mask_res_sub == DRAW_MASK_RES_TRANSP)
                    {
                        memset(mask_buf_sub, 0x00, blend_w);
                        overwrite_with_circle(p_circle_mask_buf, &mask_area, p_circle_mask_area1, mask_buf_sub, circle_mask_width);
                        mask_res_sub = DRAW_MASK_RES_CHANGED;
                    }
                    else
                        add_circle(p_circle_mask_buf, &mask_area, p_circle_mask_area1, mask_buf_sub, circle_mask_width);
                }



            if (p_circle_mask_area2 && p_circle_mask_buf)
                if (mask_area.y0 >= p_circle_mask_area2->y0 && mask_area.y0 <= p_circle_mask_area2->y1)
                {
                    if (mask_res_sub == DRAW_MASK_RES_TRANSP)
                    {
                        memset(mask_buf_sub, 0x00, blend_w);
                        overwrite_with_circle(p_circle_mask_buf, &mask_area, p_circle_mask_area2, mask_buf_sub, circle_mask_width);
                        mask_res_sub = DRAW_MASK_RES_CHANGED;
                    }
                    else
                        add_circle(p_circle_mask_buf, &mask_area, p_circle_mask_area2, mask_buf_sub, circle_mask_width);
                }


            if (mask_res_sub != DRAW_MASK_RES_TRANSP)
                mask_res = mask_res_sub;
            else
                memset(mask_buf_sub, 0x00, blend_w);

            mask_area.y0 ++;
            mask_area.y1 ++;
            mask_buf_sub += blend_w;
        }

        fill_area.y1 =  fill_area.y0 + fill_h - 1;
        if (mask_res != DRAW_MASK_RES_TRANSP)
        {
            //setup mask layer
            p_mask_layer->data = (uint8_t *)mask_buf_ping_pong;
            p_mask_layer->height = fill_h;
            p_mask_layer->y_offset = fill_area.y0;


            //setup output_layer
            {
                int16_t x_off = fill_area.x0 - dst->x_offset;
                int16_t y_off = fill_area.y0 - dst->y_offset;

                output_layer.x_offset = dst->x_offset + x_off;
                output_layer.y_offset = dst->y_offset + y_off;
                output_layer.data     = dst->data + (y_off * dst->total_width + x_off) * dst_color_bytes;
                output_layer.height   = fill_h;
            }


            if (p_rect_desc->is_grad)
            {
                if (p_extra_mask_layer)
                {
                    RT_ASSERT(0); //TODO, merge extra mask into p_mask_layer
                }

                input_layers[0] = output_layer; //Setup bg layer
                ret =  Call_Hal_Api(HAL_API_BLEND_EX, input_layers, (void *)((uint32_t)3), &output_layer);
                DRV_EPIC_ASSERT(HAL_OK == ret);
            }
            else
            {
                ret =  Call_Hal_Api(HAL_API_CONT_BLEND, p_mask_layer, p_extra_mask_layer, &output_layer);
                DRV_EPIC_ASSERT(HAL_OK == ret);
            }



            if (mask_buf_ping_pong == mask_buf)
                mask_buf_ping_pong = mask_buf + (blend_w * mask_buf_h);
            else
                mask_buf_ping_pong = mask_buf;
        }
        fill_area.y0 += fill_h;
        fill_area.y1 += fill_h;
        f += fill_h;
    }

    return RT_EOK;
}

static inline rt_err_t draw_masked_rect2(EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area,
        EPIC_LayerConfigTypeDef *p_mask_layer,
        void *masks[], const EPIC_AreaTypeDef *p_draw_area, uint32_t argb8888,
        const EPIC_AreaTypeDef *p_circle_mask_area1,
        const EPIC_AreaTypeDef *p_circle_mask_area2,
        drv_epic_mask_opa_t *p_circle_mask_buf,
        uint32_t circle_mask_width)
{
    EPIC_AreaTypeDef com_area;
    if (HAL_EPIC_AreaIntersect(&com_area, p_draw_area, p_clip_area))
    {
        draw_rect_dsc_t rect_desc;
        rect_desc.is_grad = 0;
        rect_desc.p_grad_color = NULL;
        rect_desc.p_rect_area = NULL;
        rect_desc.argb8888 = argb8888;
        return draw_masked_rect(dst, p_mask_layer, masks, &com_area, &rect_desc,
                                p_circle_mask_area1, p_circle_mask_area2, p_circle_mask_buf, circle_mask_width);
    }

    return RT_EOK;
}

static inline rt_err_t draw_masked_rect3(EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area,
        EPIC_LayerConfigTypeDef *p_mask_layer,
        void *masks[], const EPIC_AreaTypeDef *p_draw_area, draw_rect_dsc_t *p_rect_desc,
        const EPIC_AreaTypeDef *p_circle_mask_area1,
        const EPIC_AreaTypeDef *p_circle_mask_area2,
        drv_epic_mask_opa_t *p_circle_mask_buf,
        uint32_t circle_mask_width)
{
    EPIC_AreaTypeDef com_area;
    if (HAL_EPIC_AreaIntersect(&com_area, p_draw_area, p_clip_area))
        return draw_masked_rect(dst, p_mask_layer, masks, &com_area, p_rect_desc,
                                p_circle_mask_area1, p_circle_mask_area2, p_circle_mask_buf, circle_mask_width);

    return RT_EOK;
}

static rt_err_t draw_masked_rect4(EPIC_LayerConfigTypeDef *dst, EPIC_LayerConfigTypeDef *p_extra_mask_layer,
                                  void *masks[], const EPIC_AreaTypeDef *p_draw_area, uint32_t argb8888)
{
    draw_rect_dsc_t rect_desc;
    rect_desc.is_grad = 0;
    rect_desc.p_grad_color = NULL;
    rect_desc.p_rect_area = NULL;
    rect_desc.argb8888 = argb8888;

    return draw_masked_rect(dst, p_extra_mask_layer, masks, p_draw_area, &rect_desc,
                            NULL, NULL, NULL, 0);
}

static void get_rounded_area(int16_t angle, int32_t radius, uint8_t thickness, EPIC_AreaTypeDef *res_area)
{
    int32_t thick_half = thickness / 2;
    uint8_t thick_corr = (thickness & 0x01) ? 0 : 1;

    int32_t cir_x;
    int32_t cir_y;


    cir_x = ((radius - thick_half) * EPIC_TrigoCos(angle)) >> (EPIC_TRIGO_SHIFT - 8);
    cir_y = ((radius - thick_half) * EPIC_TrigoSin(angle)) >> (EPIC_TRIGO_SHIFT - 8);

    /*The center of the pixel need to be calculated so apply 1/2 px offset*/
    if (cir_x > 0)
    {
        cir_x = (cir_x - 128) >> 8;
        res_area->x0 = cir_x - thick_half + thick_corr;
        res_area->x1 = cir_x + thick_half;
    }
    else
    {
        cir_x = (cir_x + 128) >> 8;
        res_area->x0 = cir_x - thick_half;
        res_area->x1 = cir_x + thick_half - thick_corr;
    }

    if (cir_y > 0)
    {
        cir_y = (cir_y - 128) >> 8;
        res_area->y0 = cir_y - thick_half + thick_corr;
        res_area->y1 = cir_y + thick_half;
    }
    else
    {
        cir_y = (cir_y + 128) >> 8;
        res_area->y0 = cir_y - thick_half;
        res_area->y1 = cir_y + thick_half - thick_corr;
    }
}


static rt_err_t render_arc(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    int16_t center_x = p_operation->desc.arc.center_x;
    int16_t center_y = p_operation->desc.arc.center_y;
    uint16_t radius = p_operation->desc.arc.radius;
    uint16_t start_angle = p_operation->desc.arc.start_angle;
    uint16_t end_angle = p_operation->desc.arc.end_angle;
    uint8_t opa = (uint8_t)((p_operation->desc.arc.argb8888 >> 24) & 0xFF);

    if (opa <= EPIC_OPA_MIN) return RT_EOK;
    if (p_operation->desc.arc.width == 0) return RT_EOK;
    if (start_angle == end_angle) return RT_EOK;

    int32_t width = p_operation->desc.arc.width;
    if (width > radius) width = radius;

    EPIC_AreaTypeDef area_out;
    area_out.x0 = center_x - radius;
    area_out.y0 = center_y - radius;
    area_out.x1 = center_x + radius - 1;  /*-1 because the center already belongs to the left/bottom part*/
    area_out.y1 = center_y + radius - 1;



    EPIC_AreaTypeDef clipped_area;
    if (!HAL_EPIC_AreaIntersect(&clipped_area, &area_out, p_clip_area)) return RT_EOK;


    EPIC_AreaTypeDef area_in;
    HAL_EPIC_AreaCopy(&area_in, &area_out);
    area_in.x0 += p_operation->desc.arc.width;
    area_in.y0 += p_operation->desc.arc.width;
    area_in.x1 -= p_operation->desc.arc.width;
    area_in.y1 -= p_operation->desc.arc.width;

    while (start_angle >= 360) start_angle -= 360;
    while (end_angle >= 360) end_angle -= 360;




    //Prepare round mask
    drv_epic_mask_opa_t *circle_mask = NULL;
    EPIC_AreaTypeDef round_area_1;
    EPIC_AreaTypeDef round_area_2;
    if ((p_operation->desc.arc.round_start || p_operation->desc.arc.round_end)
            && (start_angle != end_angle) //Not draw round end points if it is an circle
       )
    {
        RT_ASSERT(width * width <= mask_buf2_max_bytes);
        circle_mask = (drv_epic_mask_opa_t *)gp_drv_epic->mask_buf2_pool;

        /*
            Avoid 'mask_buf2_pool' is still using by previous blending,
            befre we overwrite it.
        */
        HAL_StatusTypeDef ret =  Call_Hal_Api(HAL_API_ALL_STOP, NULL, NULL, NULL);
        DRV_EPIC_ASSERT(HAL_OK == ret);

        memset(circle_mask, 0xff, width * width);

        drv_epic_mask_area_t circle_area = {0, 0, width - 1, width - 1};
        drv_epic_mask_radius_param_t circle_mask_param;
        drv_epic_mask_radius_init(&circle_mask_param, &circle_area, width / 2, false);
        void *circle_mask_list[2] = {&circle_mask_param, NULL};


        drv_epic_mask_opa_t *circle_mask_tmp = circle_mask;
        for (int32_t h = 0; h < width; h++)
        {
            drv_epic_mask_res_t res = drv_epic_mask_apply(circle_mask_list, circle_mask_tmp, 0, h, width);
            if (res == DRAW_MASK_RES_TRANSP)
            {
                memset(circle_mask_tmp, 0x00, width);
            }

            circle_mask_tmp += width;
        }

        drv_epic_mask_free_param(&circle_mask_param);

        get_rounded_area(start_angle, radius, width, &round_area_1);
        HAL_EPIC_AreaMove(&round_area_1, center_x, center_y);
        get_rounded_area(end_angle, radius, width, &round_area_2);
        HAL_EPIC_AreaMove(&round_area_2, center_x, center_y);

    }


    void *mask_list[4] = {0, 0, 0, 0};

    int32_t  mask_counts = 0;
    drv_epic_mask_angle_param_t mask_angle_param;
    /*Create an angle mask*/
    if (start_angle != end_angle)
    {
        drv_epic_mask_angle_init(&mask_angle_param, center_x, center_y, start_angle, end_angle);
        mask_list[mask_counts] = &mask_angle_param;
        mask_counts++;
    }

    /*Create an outer mask*/
    drv_epic_mask_radius_param_t mask_out_param;
    drv_epic_mask_radius_init2(&mask_out_param, &area_out, radius_circle, false);
    mask_list[mask_counts] = &mask_out_param;
    mask_counts++;

    /*Create inner the mask*/
    drv_epic_mask_radius_param_t mask_in_param;
    if (HAL_EPIC_AreaWidth(&area_in) > 0 && HAL_EPIC_AreaHeight(&area_in) > 0)
    {
        drv_epic_mask_radius_init2(&mask_in_param, &area_in, radius_circle, true);
        mask_list[mask_counts] = &mask_in_param;
        mask_counts++;
    }

    draw_rect_dsc_t rect_desc;
    rect_desc.is_grad = 0;
    rect_desc.p_grad_color = NULL;
    rect_desc.p_rect_area = NULL;
    rect_desc.argb8888 = p_operation->desc.arc.argb8888;

    draw_masked_rect(dst, &p_operation->mask,
                     mask_list, &clipped_area, &rect_desc,
                     &round_area_1, &round_area_2,
                     circle_mask,
                     width
                    );


    for (int32_t i = 0; i < mask_counts; i++) drv_epic_mask_free_param(mask_list[i]);

    return RT_EOK;
}

static rt_err_t render_rectangle(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    const int32_t SPLIT_LIMIT = 100;
    EPIC_AreaTypeDef *p_rect_area = &p_operation->desc.rectangle.area;
    uint32_t color = p_operation->desc.rectangle.argb8888;
    int32_t coords_w = HAL_EPIC_AreaWidth(p_rect_area);
    int32_t coords_h = HAL_EPIC_AreaHeight(p_rect_area);
    EPIC_AreaTypeDef draw_area;
    draw_rect_dsc_t rect_desc;

    if (!HAL_EPIC_AreaIntersect(&draw_area, p_rect_area, p_clip_area))  return RT_EOK;

    rect_desc.is_grad = p_operation->desc.rectangle.grad_color_en;
    rect_desc.p_grad_color = &p_operation->desc.rectangle.grad_color;
    rect_desc.p_rect_area = p_rect_area;
    rect_desc.argb8888 = p_operation->desc.rectangle.argb8888;

    /*Get the real radius*/
    int32_t rout = p_operation->desc.rectangle.radius;
    int32_t short_side = MIN(coords_w, coords_h);
    if (rout > short_side >> 1) rout = short_side >> 1;

    if (rout <= 0)
    {
        draw_rect(dst, &p_operation->mask, &draw_area, &rect_desc);
    }
    else
    {
        void *mask_list[2] = {0, 0};
        int top_fillet = p_operation->desc.rectangle.top_fillet;
        int bot_fillet = p_operation->desc.rectangle.bot_fillet;

        drv_epic_mask_radius_param_t mask_out_param;

        drv_epic_mask_radius_init2(&mask_out_param, p_rect_area, rout, false);
        mask_list[0] = &mask_out_param;

        uint8_t split = 0;

        if ((coords_w > coords_h) && (coords_w - 2 * rout > SPLIT_LIMIT))
            split = 1;
        else if ((coords_w <= coords_h) && (coords_h - 2 * rout > SPLIT_LIMIT))
            split = 2;

        EPIC_AreaTypeDef fill_area;
        fill_area.x0 = p_rect_area->x0;
        fill_area.x1 = p_rect_area->x1;

        if (top_fillet && !bot_fillet)
        {
            //Draw round rect area
            fill_area.y0 = p_rect_area->y0;
            fill_area.y1 = p_rect_area->y0 + rout;
            draw_masked_rect3(dst, p_clip_area, &p_operation->mask,
                              mask_list, &fill_area, &rect_desc,
                              NULL, NULL, NULL, 0);


            //Draw NONE round rect area
            fill_area.y0 = p_rect_area->y0 + rout + 1;
            fill_area.y1 = p_rect_area->y1;
            draw_rect3(dst, p_clip_area, &p_operation->mask, &fill_area, &rect_desc);
        }
        else if (!top_fillet && bot_fillet)
        {
            //Draw round rect area
            fill_area.y0 = p_rect_area->y1 - rout;
            fill_area.y1 = p_rect_area->y1;
            draw_masked_rect3(dst, p_clip_area, &p_operation->mask,
                              mask_list, &fill_area, &rect_desc,
                              NULL, NULL, NULL, 0);



            //Draw NONE round rect area
            fill_area.y0 = p_rect_area->y0;
            fill_area.y1 = p_rect_area->y1 - rout - 1;
            draw_rect3(dst, p_clip_area, &p_operation->mask, &fill_area, &rect_desc);
        }
        else if (2 == split)
        {
            //Draw round rect area
            fill_area.y0 = p_rect_area->y0;
            fill_area.y1 = p_rect_area->y0 + rout;
            //Top
            draw_masked_rect3(dst, p_clip_area, &p_operation->mask,
                              mask_list, &fill_area, &rect_desc,
                              NULL, NULL, NULL, 0);

            fill_area.y0 = p_rect_area->y1 - rout;
            fill_area.y1 = p_rect_area->y1;
            //Bottom
            draw_masked_rect3(dst, p_clip_area, &p_operation->mask,
                              mask_list, &fill_area, &rect_desc,
                              NULL, NULL, NULL, 0);




            //Draw NONE round rect area
            fill_area.y0 = p_rect_area->y0 + rout + 1;
            fill_area.y1 = p_rect_area->y1 - rout - 1;
            draw_rect3(dst, p_clip_area, &p_operation->mask, &fill_area, &rect_desc);
        }
        else if (1 == split)
        {
            fill_area.y0 = p_rect_area->y0;
            fill_area.y1 = p_rect_area->y1;

            //Draw round rect area
            fill_area.x0 = p_rect_area->x0;
            fill_area.x1 = p_rect_area->x0 + rout - 1;
            //Left
            draw_masked_rect3(dst, p_clip_area, &p_operation->mask,
                              mask_list, &fill_area, &rect_desc,
                              NULL, NULL, NULL, 0);


            fill_area.x0 = p_rect_area->x1 - rout + 1;
            fill_area.x1 = p_rect_area->x1;
            //Right
            draw_masked_rect3(dst, p_clip_area, &p_operation->mask,
                              mask_list, &fill_area, &rect_desc,
                              NULL, NULL, NULL, 0);




            //Draw NONE round rect area
            fill_area.x0 = p_rect_area->x0 + rout;
            fill_area.x1 = p_rect_area->x1 - rout;
            draw_rect3(dst, p_clip_area, &p_operation->mask, &fill_area, &rect_desc);
        }
        else
        {
            fill_area.y0 = p_rect_area->y0;
            fill_area.y1 = p_rect_area->y1;
            draw_masked_rect3(dst, p_clip_area, &p_operation->mask,
                              mask_list, &fill_area, &rect_desc,
                              NULL, NULL, NULL, 0);
        }

        drv_epic_mask_free_param(&mask_out_param);
    }
    return RT_EOK;
}

static rt_err_t render_border_complex(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst,  const EPIC_AreaTypeDef *p_clip_area,
                                      const EPIC_AreaTypeDef *outer_area, const EPIC_AreaTypeDef *inner_area,
                                      int32_t rout, int32_t rin, uint32_t argb8888)
{
    const int32_t SPLIT_LIMIT = 100;
    /*Get clipped draw area which is the real draw area.
     *It is always the same or inside `coords`*/
    EPIC_AreaTypeDef draw_area;
    if (!HAL_EPIC_AreaIntersect(&draw_area, outer_area, p_clip_area)) return RT_EOK;
    int32_t draw_area_w = HAL_EPIC_AreaWidth(&draw_area);



    EPIC_AreaTypeDef blend_area;


    /*Calculate the x and y coordinates where the straight parts area is*/
    EPIC_AreaTypeDef core_area;
    core_area.x0 = MAX(outer_area->x0 + rout, inner_area->x0);
    core_area.x1 = MIN(outer_area->x1 - rout, inner_area->x1);
    core_area.y0 = MAX(outer_area->y0 + rout, inner_area->y0);
    core_area.y1 = MIN(outer_area->y1 - rout, inner_area->y1);
    int32_t core_w = HAL_EPIC_AreaWidth(&core_area);

    bool top_side = outer_area->y0 <= inner_area->y0;
    bool bottom_side = outer_area->y1 >= inner_area->y1;

    /*No masks*/
    bool left_side = outer_area->x0 <= inner_area->x0;
    bool right_side = outer_area->x1 >= inner_area->x1;

    bool split_hor = true;
    if (left_side && right_side && top_side && bottom_side &&
            core_w < SPLIT_LIMIT)
    {
        split_hor = false;
    }

    drv_epic_mask_res_t blend_dsc_mask_res = DRAW_MASK_RES_FULL_COVER;
    /*Draw the straight lines first if they are long enough*/
    if (top_side && split_hor)
    {
        blend_area.x0 = core_area.x0;
        blend_area.x1 = core_area.x1;
        blend_area.y0 = outer_area->y0;
        blend_area.y1 = inner_area->y0 - 1;
        draw_rect2(dst, p_clip_area, &p_operation->mask, &blend_area, argb8888);
    }

    if (bottom_side && split_hor)
    {
        blend_area.x0 = core_area.x0;
        blend_area.x1 = core_area.x1;
        blend_area.y0 = inner_area->y1 + 1;
        blend_area.y1 = outer_area->y1;
        draw_rect2(dst, p_clip_area, &p_operation->mask, &blend_area, argb8888);
    }

    /*If the border is very thick and the vertical sides overlap horizontally draw a single rectangle*/
    if (inner_area->x0 >= inner_area->x1 && left_side && right_side)
    {
        blend_area.x0 = outer_area->x0;
        blend_area.x1 = outer_area->x1;
        blend_area.y0 = core_area.y0;
        blend_area.y1 = core_area.y1;
        draw_rect2(dst, p_clip_area, &p_operation->mask, &blend_area, argb8888);
    }
    else
    {
        if (left_side)
        {
            blend_area.x0 = outer_area->x0;
            blend_area.x1 = inner_area->x0 - 1;
            blend_area.y0 = core_area.y0;
            blend_area.y1 = core_area.y1;
            draw_rect2(dst, p_clip_area, &p_operation->mask, &blend_area, argb8888);
        }

        if (right_side)
        {
            blend_area.x0 = inner_area->x1 + 1;
            blend_area.x1 = outer_area->x1;
            blend_area.y0 = core_area.y0;
            blend_area.y1 = core_area.y1;
            draw_rect2(dst, p_clip_area, &p_operation->mask, &blend_area, argb8888);
        }
    }

    /*Draw the corners*/
    int32_t blend_w;
    void *mask_list[3] = {0, 0, 0};

    /*Create mask for the inner mask*/
    drv_epic_mask_radius_param_t mask_rin_param;
    drv_epic_mask_radius_init2(&mask_rin_param, inner_area, rin, true);
    mask_list[0] = &mask_rin_param;

    /*Create mask for the outer area*/
    drv_epic_mask_radius_param_t mask_rout_param;
    if (rout > 0)
    {
        drv_epic_mask_radius_init2(&mask_rout_param, outer_area, rout, false);
        mask_list[1] = &mask_rout_param;
    }

    /*Left and right corner together if they are close to each other*/
    if (!split_hor)
    {
        /*Calculate the top corner and mirror it to the bottom*/
        blend_area.x0 = draw_area.x0;
        blend_area.x1 = draw_area.x1;

        /*Top Left&Right corner*/
        blend_area.y0 = draw_area.y0;
        blend_area.y1 = core_area.y0 - 1;
        draw_masked_rect2(dst, p_clip_area, &p_operation->mask,
                          mask_list, &blend_area, argb8888,
                          NULL, NULL, NULL, 0);

        /*Bottom left&right corner*/
        blend_area.y0 = core_area.y1 + 1;
        blend_area.y1 = draw_area.y1;
        draw_masked_rect2(dst, p_clip_area, &p_operation->mask,
                          mask_list, &blend_area, argb8888,
                          NULL, NULL, NULL, 0);
    }
    else
    {
        /*Left corners*/
        blend_area.x0 = draw_area.x0;
        blend_area.x1 = MIN(draw_area.x1, core_area.x0 - 1);
        blend_w = HAL_EPIC_AreaWidth(&blend_area);
        if (blend_w > 0)
        {
            if (left_side || top_side)
            {
                blend_area.y0 = draw_area.y0;
                blend_area.y1 = core_area.y0 - 1;
                draw_masked_rect2(dst, p_clip_area, &p_operation->mask,
                                  mask_list, &blend_area, argb8888,
                                  NULL, NULL, NULL, 0);
            }

            if (left_side || bottom_side)
            {
                blend_area.y0 = core_area.y1 + 1;
                blend_area.y1 = draw_area.y1;
                draw_masked_rect2(dst, p_clip_area, &p_operation->mask,
                                  mask_list, &blend_area, argb8888,
                                  NULL, NULL, NULL, 0);
            }
        }

        /*Right corners*/
        blend_area.x0 = MAX(draw_area.x0, blend_area.x1 + 1);    /*To not overlap with the left side*/
        blend_area.x0 = MAX(draw_area.x0, core_area.x1 + 1);

        blend_area.x1 = draw_area.x1;
        blend_w = HAL_EPIC_AreaWidth(&blend_area);

        if (blend_w > 0)
        {
            if (right_side || top_side)
            {
                blend_area.y0 = draw_area.y0;
                blend_area.y1 = core_area.y0 - 1;
                draw_masked_rect2(dst, p_clip_area, &p_operation->mask,
                                  mask_list, &blend_area, argb8888,
                                  NULL, NULL, NULL, 0);
            }

            if (right_side || bottom_side)
            {
                blend_area.y0 = core_area.y1 + 1;
                blend_area.y1 = draw_area.y1;
                draw_masked_rect2(dst, p_clip_area, &p_operation->mask,
                                  mask_list, &blend_area, argb8888,
                                  NULL, NULL, NULL, 0);
            }
        }
    }

    drv_epic_mask_free_param(&mask_rin_param);
    if (rout > 0) drv_epic_mask_free_param(&mask_rout_param);

    return RT_EOK;
}

static rt_err_t render_border_simple(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst,  const EPIC_AreaTypeDef *p_clip_area,
                                     const EPIC_AreaTypeDef *outer_area, const EPIC_AreaTypeDef *inner_area,
                                     uint32_t argb8888)
{
    EPIC_AreaTypeDef a;

    bool top_side = outer_area->y0 <= inner_area->y0;
    bool bottom_side = outer_area->y1 >= inner_area->y1;
    bool left_side = outer_area->x0 <= inner_area->x0;
    bool right_side = outer_area->x1 >= inner_area->x1;

    /*Top*/
    a.x0 = outer_area->x0;
    a.x1 = outer_area->x1;
    a.y0 = outer_area->y0;
    a.y1 = inner_area->y0 - 1;
    if (top_side)
    {
        draw_rect2(dst, p_clip_area, &p_operation->mask, &a, argb8888);
    }

    /*Bottom*/
    a.y0 = inner_area->y1 + 1;
    a.y1 = outer_area->y1;
    if (bottom_side)
    {
        draw_rect2(dst, p_clip_area, &p_operation->mask, &a, argb8888);
    }

    /*Left*/
    a.x0 = outer_area->x0;
    a.x1 = inner_area->x0 - 1;
    a.y0 = (top_side) ? inner_area->y0 : outer_area->y0;
    a.y1 = (bottom_side) ? inner_area->y1 : outer_area->y1;
    if (left_side)
    {
        draw_rect2(dst, p_clip_area, &p_operation->mask, &a, argb8888);
    }

    /*Right*/
    a.x0 = inner_area->x1 + 1;
    a.x1 = outer_area->x1;
    if (right_side)
    {
        draw_rect2(dst, p_clip_area, &p_operation->mask, &a, argb8888);
    }

    return RT_EOK;
}

static rt_err_t render_border(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    uint8_t opa = (p_operation->desc.border.argb8888 >> 24) & 0xFF;
    if (p_operation->desc.border.width == 0) return RT_EOK;
    if (opa <= EPIC_OPA_MIN) return RT_EOK;
    if ((0 == p_operation->desc.border.top_side)
            && (0 == p_operation->desc.border.bot_side)
            && (0 == p_operation->desc.border.left_side)
            && (0 == p_operation->desc.border.right_side))
        return RT_EOK;

    int32_t border_w = p_operation->desc.border.width;
    int32_t coords_w = HAL_EPIC_AreaWidth(&p_operation->desc.border.area);
    int32_t coords_h = HAL_EPIC_AreaHeight(&p_operation->desc.border.area);
    int32_t rout = p_operation->desc.border.radius;
    int32_t short_side = MIN(coords_w, coords_h);
    if (rout > short_side >> 1) rout = short_side >> 1;

    /*Get the inner area*/
    EPIC_AreaTypeDef area_inner = p_operation->desc.border.area;
    area_inner.x0 += ((p_operation->desc.border.left_side) ? border_w : - (border_w + rout));
    area_inner.x1 -= ((p_operation->desc.border.right_side) ? border_w : - (border_w + rout));
    area_inner.y0 += ((p_operation->desc.border.top_side) ? border_w : - (border_w + rout));
    area_inner.y1 -= ((p_operation->desc.border.bot_side) ? border_w : - (border_w + rout));

    int32_t rin = rout - border_w;
    if (rin < 0) rin = 0;

    if (rout == 0 && rin == 0)
    {
        render_border_simple(p_operation, dst, p_clip_area, &p_operation->desc.border.area, &area_inner, p_operation->desc.border.argb8888);
    }
    else
    {
        render_border_complex(p_operation, dst, p_clip_area, &p_operation->desc.border.area, &area_inner, rout, rin, p_operation->desc.border.argb8888);
    }

    return RT_EOK;
}



static rt_err_t render_line_hor(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    int32_t w = p_operation->desc.line.width - 1;
    int32_t w_half0 = w >> 1;
    int32_t w_half1 = w_half0 + (w & 0x1); /*Compensate rounding error*/

    EPIC_AreaTypeDef blend_area;
    int16_t dash_origin;
    blend_area.x0 = (int32_t)MIN(p_operation->desc.line.p1.x, p_operation->desc.line.p2.x);
    blend_area.x1 = (int32_t)MAX(p_operation->desc.line.p1.x, p_operation->desc.line.p2.x)  - 1;
    blend_area.y0 = (int32_t)p_operation->desc.line.p1.y - w_half1;
    blend_area.y1 = (int32_t)p_operation->desc.line.p1.y + w_half0;
    dash_origin = blend_area.x0 - w_half1;

    bool is_common;
    is_common = HAL_EPIC_AreaIntersect(&blend_area, &blend_area, p_clip_area);
    if (!is_common) return RT_EOK;

    bool dashed = p_operation->desc.line.dash_gap && p_operation->desc.line.dash_width;

    /*If there is no mask then simply draw a rectangle*/
    if (!dashed)
    {
        draw_rect_dsc_t rect_desc;
        rect_desc.is_grad = 0;
        rect_desc.p_grad_color = NULL;
        rect_desc.p_rect_area = NULL;
        rect_desc.argb8888 = p_operation->desc.line.argb8888;
        draw_rect(dst, &p_operation->mask, &blend_area, &rect_desc);
    }
    else
    {
        HAL_StatusTypeDef ret;
        EPIC_LayerConfigTypeDef fg_layer, output_layer;

        uint32_t dst_color_bytes = HAL_EPIC_GetColorDepth(dst->color_mode) >> 3;
        uint32_t argb8888 = p_operation->desc.line.argb8888;

        /*Inital fg layer*/
        HAL_EPIC_LayerConfigInit(&fg_layer);
        fg_layer.alpha = (uint8_t)((argb8888 >> 24) & 0xFF);
        fg_layer.color_en = true;
        fg_layer.color_r = (uint8_t)((argb8888 >> 16) & 0xFF);
        fg_layer.color_g = (uint8_t)((argb8888 >> 8) & 0xFF);
        fg_layer.color_b = (uint8_t)((argb8888 >> 0) & 0xFF);
        fg_layer.color_mode = EPIC_INPUT_MONO;
        fg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;
        fg_layer.data = (uint8_t *)mono_layer_addr;
        fg_layer.width  = p_operation->desc.line.dash_width;
        fg_layer.height = HAL_EPIC_AreaHeight(&blend_area);
        fg_layer.total_width = fg_layer.width;


        /*Inital output layer*/
        memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));

        int32_t dash_width = p_operation->desc.line.dash_width;
        int32_t dash_gap  = p_operation->desc.line.dash_gap;
        EPIC_AreaTypeDef dash_area = blend_area;
        int16_t dash_offset = (blend_area.x0 - dash_origin) % (dash_gap + dash_width);

        for (dash_area.x0 = blend_area.x0 - dash_offset;
                dash_area.x0 <= blend_area.x1; dash_area.x0 += dash_width + dash_gap)
        {
            dash_area.x1 = dash_area.x0 + dash_width - 1;
            if (dash_area.x1 < blend_area.x0) continue;

            EPIC_AreaTypeDef final_area;

            if (HAL_EPIC_AreaIntersect(&final_area, &blend_area, &dash_area))
            {
                //setup dash fg
                {
                    fg_layer.x_offset = dash_area.x0;
                    fg_layer.y_offset = dash_area.y0;
                }
                //setup output_layer according to final_area
                {

                    int16_t x_off = final_area.x0 - dst->x_offset;
                    int16_t y_off = final_area.y0 - dst->y_offset;

                    output_layer.data     = dst->data + (y_off * dst->total_width + x_off) * dst_color_bytes;
                    output_layer.x_offset = dst->x_offset + x_off;
                    output_layer.y_offset = dst->y_offset + y_off;
                    output_layer.width    = final_area.x1 - final_area.x0 + 1;
                    output_layer.height   = final_area.y1 - final_area.y0 + 1;
                }



                ret =  Call_Hal_Api(HAL_API_CONT_BLEND, &fg_layer,
                                    (p_operation->mask.data != NULL) ? &p_operation->mask : NULL,
                                    &output_layer);
                DRV_EPIC_ASSERT(HAL_OK == ret);
            }
        }
    }
    return RT_EOK;
}

static rt_err_t render_line_ver(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    int32_t w = p_operation->desc.line.width - 1;
    int32_t w_half0 = w >> 1;
    int32_t w_half1 = w_half0 + (w & 0x1); /*Compensate rounding error*/

    EPIC_AreaTypeDef blend_area;
    int16_t dash_origin;
    blend_area.x0 = (int32_t)p_operation->desc.line.p1.x - w_half1;
    blend_area.x1 = (int32_t)p_operation->desc.line.p1.x + w_half0;
    blend_area.y0 = (int32_t)MIN(p_operation->desc.line.p1.y, p_operation->desc.line.p2.y);
    blend_area.y1 = (int32_t)MAX(p_operation->desc.line.p1.y, p_operation->desc.line.p2.y) - 1;
    dash_origin = blend_area.y0 - w_half1;

    bool is_common;
    is_common = HAL_EPIC_AreaIntersect(&blend_area, &blend_area, p_clip_area);
    if (!is_common) return RT_EOK;

    bool dashed = p_operation->desc.line.dash_gap && p_operation->desc.line.dash_width;

    /*If there is no mask then simply draw a rectangle*/
    if (!dashed)
    {
        draw_rect_dsc_t rect_desc;
        rect_desc.is_grad = 0;
        rect_desc.p_grad_color = NULL;
        rect_desc.p_rect_area = NULL;
        rect_desc.argb8888 = p_operation->desc.line.argb8888;
        draw_rect(dst, &p_operation->mask, &blend_area, &rect_desc);
    }
    else
    {
        HAL_StatusTypeDef ret;
        EPIC_LayerConfigTypeDef fg_layer, output_layer;

        uint32_t dst_color_bytes = HAL_EPIC_GetColorDepth(dst->color_mode) >> 3;
        uint32_t argb8888 = p_operation->desc.line.argb8888;

        /*Inital fg layer*/
        HAL_EPIC_LayerConfigInit(&fg_layer);
        fg_layer.alpha = (uint8_t)((argb8888 >> 24) & 0xFF);
        fg_layer.color_en = true;
        fg_layer.color_r = (uint8_t)((argb8888 >> 16) & 0xFF);
        fg_layer.color_g = (uint8_t)((argb8888 >> 8) & 0xFF);
        fg_layer.color_b = (uint8_t)((argb8888 >> 0) & 0xFF);
        fg_layer.color_mode = EPIC_INPUT_MONO;
        fg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;
        fg_layer.data = (uint8_t *)mono_layer_addr;
        fg_layer.width  = HAL_EPIC_AreaWidth(&blend_area);
        fg_layer.height = p_operation->desc.line.dash_width;
        fg_layer.total_width = fg_layer.width;


        /*Inital output layer*/
        memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));

        int32_t dash_width = p_operation->desc.line.dash_width;
        int32_t dash_gap  = p_operation->desc.line.dash_gap;
        EPIC_AreaTypeDef dash_area = blend_area;
        int16_t dash_offset = (blend_area.y0 - dash_origin) % (dash_gap + dash_width);



        for (dash_area.y0 = blend_area.y0 - dash_offset;
                dash_area.y0 <= blend_area.y1; dash_area.y0 += dash_width + dash_gap)
        {
            dash_area.y1 = dash_area.y0 + dash_width - 1;
            if (dash_area.y1 < blend_area.y0) continue;

            EPIC_AreaTypeDef final_area;

            if (HAL_EPIC_AreaIntersect(&final_area, &blend_area, &dash_area))
            {
                //setup dash fg
                {
                    fg_layer.x_offset = dash_area.x0;
                    fg_layer.y_offset = dash_area.y0;
                }
                //setup output_layer according to final_area
                {

                    int16_t x_off = final_area.x0 - dst->x_offset;
                    int16_t y_off = final_area.y0 - dst->y_offset;

                    output_layer.data     = dst->data + (y_off * dst->total_width + x_off) * dst_color_bytes;
                    output_layer.x_offset = dst->x_offset + x_off;
                    output_layer.y_offset = dst->y_offset + y_off;
                    output_layer.width    = final_area.x1 - final_area.x0 + 1;
                    output_layer.height   = final_area.y1 - final_area.y0 + 1;
                }



                ret =  Call_Hal_Api(HAL_API_CONT_BLEND, &fg_layer,
                                    (p_operation->mask.data != NULL) ? &p_operation->mask : NULL,
                                    &output_layer);
                DRV_EPIC_ASSERT(HAL_OK == ret);
            }
        }
    }

    return RT_EOK;
}

static rt_err_t render_line_skew(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    /*Keep the great y in p1*/
    EPIC_PointTypeDef p1;
    EPIC_PointTypeDef p2;

    if (p_operation->desc.line.p1.y < p_operation->desc.line.p2.y)
    {
        p1 = p_operation->desc.line.p1;
        p2 = p_operation->desc.line.p2;
    }
    else
    {
        p1 = p_operation->desc.line.p2;
        p2 = p_operation->desc.line.p1;
    }

    int32_t xdiff = p2.x - p1.x;
    int32_t ydiff = p2.y - p1.y;
    bool flat = ABS(xdiff) > ABS(ydiff);

    static const uint8_t wcorr[] =
    {
        128, 128, 128, 129, 129, 130, 130, 131,
        132, 133, 134, 135, 137, 138, 140, 141,
        143, 145, 147, 149, 151, 153, 155, 158,
        160, 162, 165, 167, 170, 173, 175, 178,
        181,
    };

    int32_t w = p_operation->desc.line.width;
    int32_t wcorr_i = 0;
    if (flat) wcorr_i = (ABS(ydiff) << 5) / ABS(xdiff);
    else wcorr_i = (ABS(xdiff) << 5) / ABS(ydiff);

    w = (w * wcorr[wcorr_i] + 63) >> 7;     /*+ 63 for rounding*/
    int32_t w_half0 = w >> 1;
    int32_t w_half1 = w_half0 + (w & 0x1); /*Compensate rounding error*/

    EPIC_AreaTypeDef blend_area;
    blend_area.x0 = MIN(p1.x, p2.x) - w;
    blend_area.x1 = MAX(p1.x, p2.x) + w;
    blend_area.y0 = MIN(p1.y, p2.y) - w;
    blend_area.y1 = MAX(p1.y, p2.y) + w;

    /*Get the union of `coords` and `clip`*/
    /*`clip` is already truncated to the `draw_buf` size
     *in 'lv_refr_area' function*/
    bool is_common = HAL_EPIC_AreaIntersect(&blend_area, &blend_area, p_clip_area);
    if (is_common == false) return RT_EOK;

    drv_epic_mask_line_param_t mask_left_param;
    drv_epic_mask_line_param_t mask_right_param;
    drv_epic_mask_line_param_t mask_top_param;
    drv_epic_mask_line_param_t mask_bottom_param;

    void *masks[5] = {&mask_left_param, & mask_right_param, NULL, NULL, NULL};

    if (flat)
    {
        if (xdiff > 0)
        {
            drv_epic_mask_line_points_init(&mask_left_param, p1.x, p1.y - w_half0, p2.x, p2.y - w_half0,
                                           DRAW_MASK_LINE_SIDE_LEFT);
            drv_epic_mask_line_points_init(&mask_right_param, p1.x, p1.y + w_half1, p2.x, p2.y + w_half1,
                                           DRAW_MASK_LINE_SIDE_RIGHT);
        }
        else
        {
            drv_epic_mask_line_points_init(&mask_left_param, p1.x, p1.y + w_half1, p2.x, p2.y + w_half1,
                                           DRAW_MASK_LINE_SIDE_LEFT);
            drv_epic_mask_line_points_init(&mask_right_param, p1.x, p1.y - w_half0, p2.x, p2.y - w_half0,
                                           DRAW_MASK_LINE_SIDE_RIGHT);
        }
    }
    else
    {
        drv_epic_mask_line_points_init(&mask_left_param, p1.x + w_half1, p1.y, p2.x + w_half1, p2.y,
                                       DRAW_MASK_LINE_SIDE_LEFT);
        drv_epic_mask_line_points_init(&mask_right_param, p1.x - w_half0, p1.y, p2.x - w_half0, p2.y,
                                       DRAW_MASK_LINE_SIDE_RIGHT);

    }

    /*Use the normal vector for the endings*/

    if (!p_operation->desc.line.raw_end)
    {
        drv_epic_mask_line_points_init(&mask_top_param, p1.x, p1.y, p1.x - ydiff, p1.y + xdiff,
                                       DRAW_MASK_LINE_SIDE_BOTTOM);
        drv_epic_mask_line_points_init(&mask_bottom_param, p2.x, p2.y, p2.x - ydiff, p2.y + xdiff,
                                       DRAW_MASK_LINE_SIDE_TOP);
        masks[2] = &mask_top_param;
        masks[3] = &mask_bottom_param;
    }



    draw_masked_rect4(dst, &p_operation->mask,
                      masks, &blend_area, p_operation->desc.line.argb8888);


    drv_epic_mask_free_param(&mask_left_param);
    drv_epic_mask_free_param(&mask_right_param);
    if (!p_operation->desc.line.raw_end)
    {
        drv_epic_mask_free_param(&mask_top_param);
        drv_epic_mask_free_param(&mask_bottom_param);
    }

    return RT_EOK;
}

static rt_err_t render_line(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    uint8_t opa = (p_operation->desc.line.argb8888 >> 24) & 0xFF;
    if (p_operation->desc.line.width == 0) return RT_EOK;
    if (opa <= EPIC_OPA_MIN) return RT_EOK;

    if (p_operation->desc.line.p1.x == p_operation->desc.line.p2.x && p_operation->desc.line.p1.y == p_operation->desc.line.p2.y) return RT_EOK;

    EPIC_AreaTypeDef clip_line;
    clip_line.x0 = (int32_t)MIN(p_operation->desc.line.p1.x, p_operation->desc.line.p2.x) - p_operation->desc.line.width / 2;
    clip_line.x1 = (int32_t)MAX(p_operation->desc.line.p1.x, p_operation->desc.line.p2.x) + p_operation->desc.line.width / 2;
    clip_line.y0 = (int32_t)MIN(p_operation->desc.line.p1.y, p_operation->desc.line.p2.y) - p_operation->desc.line.width / 2;
    clip_line.y1 = (int32_t)MAX(p_operation->desc.line.p1.y, p_operation->desc.line.p2.y) + p_operation->desc.line.width / 2;

    bool is_common;
    is_common = HAL_EPIC_AreaIntersect(&clip_line, &clip_line, p_clip_area);
    if (!is_common) return RT_EOK;

    uint8_t line_type; //0: hor, 1: ver, 2: skew
    if (p_operation->desc.line.p1.y == p_operation->desc.line.p2.y)
    {
        render_line_hor(p_operation, dst, p_clip_area);
        line_type = 0;
    }
    else if (p_operation->desc.line.p1.x == p_operation->desc.line.p2.x)
    {
        render_line_ver(p_operation, dst, p_clip_area);
        line_type = 1;
    }
    else
    {
        render_line_skew(p_operation, dst, p_clip_area);
        line_type = 2;
    }

    if (p_operation->desc.line.round_end || p_operation->desc.line.round_start)
    {
        int32_t r = (p_operation->desc.line.width >> 1);
        int32_t r_corr = (p_operation->desc.line.width & 1) ? 0 : 1;
        if (r > 1)
        {
            EPIC_AreaTypeDef cir_area;
            bool dashed = p_operation->desc.line.dash_gap && p_operation->desc.line.dash_width;

            if (p_operation->desc.line.round_start)
            {
                cir_area.x0 = (int32_t)p_operation->desc.line.p1.x - r;
                cir_area.y0 = (int32_t)p_operation->desc.line.p1.y - r;
                cir_area.x1 = (int32_t)p_operation->desc.line.p1.x + r - r_corr;
                cir_area.y1 = (int32_t)p_operation->desc.line.p1.y + r - r_corr ;


                void *mask_list[3] = {0};
                /*Create an outer mask*/
                drv_epic_mask_radius_param_t mask_out_param;
                drv_epic_mask_radius_init2(&mask_out_param, &cir_area, r, false);
                mask_list[0] = &mask_out_param;

                if (((opa < EPIC_OPA_MAX) || dashed) && (line_type < 2))
                {
                    drv_epic_mask_line_param_t mask_line_param;
                    drv_epic_mask_line_side_t side;
                    EPIC_PointTypeDef p1_ortho = p_operation->desc.line.p1;
                    if (line_type == 0) //hor
                    {
                        if (p_operation->desc.line.p2.x > p_operation->desc.line.p1.x) side = DRAW_MASK_LINE_SIDE_LEFT;
                        else side = DRAW_MASK_LINE_SIDE_RIGHT;

                        p1_ortho.y -= (p_operation->desc.line.p2.x - p_operation->desc.line.p1.x);
                    }
                    else //ver
                    {
                        if (p_operation->desc.line.p2.y > p_operation->desc.line.p1.y) side = DRAW_MASK_LINE_SIDE_TOP;
                        else side = DRAW_MASK_LINE_SIDE_BOTTOM;

                        p1_ortho.x -= (p_operation->desc.line.p2.y - p_operation->desc.line.p1.y);
                    }


                    drv_epic_mask_line_points_init(&mask_line_param, p_operation->desc.line.p1.x, p_operation->desc.line.p1.y, p1_ortho.x, p1_ortho.y, side);

                    mask_list[1] = &mask_line_param;
                }

                draw_masked_rect4(dst, NULL,
                                  mask_list, &clip_line, p_operation->desc.line.argb8888);
                drv_epic_mask_free_param(&mask_out_param);

            }

            if (p_operation->desc.line.round_end)
            {
                cir_area.x0 = (int32_t)p_operation->desc.line.p2.x - r;
                cir_area.y0 = (int32_t)p_operation->desc.line.p2.y - r;
                cir_area.x1 = (int32_t)p_operation->desc.line.p2.x + r - r_corr;
                cir_area.y1 = (int32_t)p_operation->desc.line.p2.y + r - r_corr ;



                void *mask_list[3] = {0};
                /*Create an outer mask*/
                drv_epic_mask_radius_param_t mask_out_param;
                drv_epic_mask_radius_init2(&mask_out_param, &cir_area, r, false);
                mask_list[0] = &mask_out_param;

                if (((opa < EPIC_OPA_MAX) || dashed) && (line_type < 2))
                {
                    drv_epic_mask_line_param_t mask_line_param;
                    drv_epic_mask_line_side_t side;
                    EPIC_PointTypeDef p2_ortho = p_operation->desc.line.p2;
                    if (line_type == 0) //hor
                    {
                        if (p_operation->desc.line.p2.x > p_operation->desc.line.p1.x) side = DRAW_MASK_LINE_SIDE_RIGHT;
                        else side = DRAW_MASK_LINE_SIDE_LEFT;

                        p2_ortho.y -= (p_operation->desc.line.p2.x - p_operation->desc.line.p1.x);
                    }
                    else //ver
                    {
                        if (p_operation->desc.line.p2.y > p_operation->desc.line.p1.y) side = DRAW_MASK_LINE_SIDE_BOTTOM;
                        else side = DRAW_MASK_LINE_SIDE_TOP;

                        p2_ortho.x -= (p_operation->desc.line.p2.y - p_operation->desc.line.p1.y);
                    }


                    drv_epic_mask_line_points_init(&mask_line_param, p_operation->desc.line.p2.x, p_operation->desc.line.p2.y, p2_ortho.x, p2_ortho.y, side);

                    mask_list[1] = &mask_line_param;
                }

                draw_masked_rect4(dst, NULL,
                                  mask_list, &clip_line, p_operation->desc.line.argb8888);
                drv_epic_mask_free_param(&mask_out_param);

            }

        }
    }

    return RT_EOK;
}

static rt_err_t render_letters(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    HAL_StatusTypeDef ret;
    EPIC_LayerConfigTypeDef fg_layer, mask_layer;
    EPIC_LayerConfigTypeDef output_layer;

    uint8_t input_layer_cnt = 2;
    uint32_t dst_color_bytes = HAL_EPIC_GetColorDepth(dst->color_mode) >> 3;
    uint32_t mask_addr = 0;
    uint32_t fg_addr = 0;
    uint32_t bg_addr = 0;

    if (p_operation->mask.data)
    {
        //Setup mask layer
        memcpy(&mask_layer, &p_operation->mask, sizeof(EPIC_LayerConfigTypeDef));
        mask_addr = (uint32_t) p_operation->mask.data;
        input_layer_cnt++;
    }

    /*Inital fg layer*/
    HAL_EPIC_LayerConfigInit(&fg_layer);
    fg_layer.alpha = p_operation->desc.label.opa;
    fg_layer.color_en = true;
    fg_layer.color_r = p_operation->desc.label.r;
    fg_layer.color_g = p_operation->desc.label.g;
    fg_layer.color_b = p_operation->desc.label.b;
    fg_layer.color_mode = p_operation->desc.label.color_mode;
    fg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;
    /*Inital output layer*/
    memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));

    for (uint32_t i = 0; i < p_operation->desc.label.letter_num; i++)
    {
        drv_epic_letter_type_t *p_letter = p_operation->desc.label.p_letters + i;
        EPIC_AreaTypeDef final_area;

        if (HAL_EPIC_AreaIntersect(&final_area, p_clip_area, &p_letter->area))
        {
            //setup fg
            fg_layer.data = (uint8_t *)p_letter->data;
            fg_layer.width  = HAL_EPIC_AreaWidth(&p_letter->area);
            fg_layer.height = HAL_EPIC_AreaHeight(&p_letter->area);
            fg_layer.total_width = fg_layer.width;
            fg_layer.x_offset = p_letter->area.x0;
            fg_layer.y_offset = p_letter->area.y0;
            fg_addr = (uint32_t)fg_layer.data;


            //setup output_layer
            {

                int16_t x_off = final_area.x0 - dst->x_offset;
                int16_t y_off = final_area.y0 - dst->y_offset;


                output_layer.data     = dst->data + (y_off * dst->total_width + x_off) * dst_color_bytes;
                output_layer.x_offset = dst->x_offset + x_off;
                output_layer.y_offset = dst->y_offset + y_off;
                output_layer.width    = final_area.x1 - final_area.x0 + 1;
                output_layer.height   = final_area.y1 - final_area.y0 + 1;
            }



            ret =  Call_Hal_Api(HAL_API_CONT_BLEND, &fg_layer,
                                (3 == input_layer_cnt) ? &mask_layer : NULL,
                                &output_layer);
            DRV_EPIC_ASSERT(HAL_OK == ret);
        }
    }

    return RT_ERROR;
}
#if (rotate_buf_bytes > 0)
static bool visible_after_transformed(EPIC_LayerConfigTypeDef *layer, const EPIC_AreaTypeDef *p_clip_area)
{
    EPIC_AreaTypeDef transformed_area;
    EPIC_PointTypeDef pivot;
    pivot.x = layer->transform_cfg.pivot_x;
    pivot.y = layer->transform_cfg.pivot_y;
    EPIC_GetTransformedArea(&transformed_area, layer->width, layer->height, layer->transform_cfg.angle,
                            layer->transform_cfg.scale_x, layer->transform_cfg.scale_y, &pivot);
    HAL_EPIC_AreaMove(&transformed_area, layer->x_offset, layer->y_offset);
    return HAL_EPIC_AreaIntersect(&transformed_area, &transformed_area, p_clip_area);
}
#ifdef EPIC_SUPPORT_TRANS_MATRIX
static bool visible_after_matrix_transformed(EPIC_LayerConfigTypeDef *layer, const EPIC_AreaTypeDef *p_clip_area)
{

    sifli_matrix_3x3_t *p_matrix = layer->transform_cfg.trans_matrix;
    RT_ASSERT(p_matrix);

    float pos_x = layer->width - 1;
    float pos_y = layer->height - 1;
    EPIC_AreaTypeDef transformed_area = {0, 0, pos_x, pos_y};
    EPIC_GetMatrixTransfromedArea(p_matrix, &transformed_area);
    HAL_EPIC_AreaMove(&transformed_area, layer->x_offset, layer->y_offset);

    return HAL_EPIC_AreaIntersect(&transformed_area, &transformed_area, p_clip_area);

}
#endif /* EPIC_SUPPORT_TRANS_MATRIX */
#endif
static void render_image(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    HAL_StatusTypeDef ret;
    EPIC_LayerConfigTypeDef output_layer;


    //Clip output_layer
    memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));
    _clip_layer(&output_layer, p_clip_area);

    EPIC_LayerConfigTypeDef input_layers[3];
    HAL_API_TypeDef api;

    uint32_t input_layer_cnt = 2;

    memcpy(&input_layers[0], dst, sizeof(EPIC_LayerConfigTypeDef));
    memcpy(&input_layers[1], &p_operation->desc.blend.layer, sizeof(EPIC_LayerConfigTypeDef));
    if (p_operation->mask.data)
    {
        memcpy(&input_layers[2], &p_operation->mask, sizeof(EPIC_LayerConfigTypeDef));
        input_layer_cnt++;
    }
    if (p_operation->desc.blend.layer.transform_cfg.type != 0)
    {
        api = HAL_API_TRANSFORM;
        RT_ASSERT((EPIC_BLEND_MODE_NORMAL == p_operation->desc.blend.use_dest_as_bg));
    }
    else
    {
        api = HAL_API_BLEND_EX;
    }

#if (rotate_buf_bytes > 0)
    if ((p_operation->desc.blend.layer.transform_cfg.angle != 0)
            && (0 == p_operation->desc.blend.layer.transform_cfg.type)
            && (!HCPU_IS_SRAM_ADDR(p_operation->desc.blend.layer.data)))
    {
        const EPIC_LayerConfigTypeDef *src_layer = &p_operation->desc.blend.layer;
        EPIC_LayerConfigTypeDef *dst_layer = &input_layers[1];

        uint32_t color_bytes = HAL_EPIC_GetColorDepth(src_layer->color_mode) >> 3;
        uint32_t buf_size = rotate_buf_bytes >> 1;
        int16_t angle = dst_layer->transform_cfg.angle;
        int16_t w = src_layer->total_width;
        int16_t h = src_layer->height;
        int16_t x = src_layer->x_offset;
        int16_t y_start = src_layer->y_offset;
        int16_t y_end = y_start + h;
        uint32_t size_line = color_bytes * src_layer->total_width;
        int16_t h_align = buf_size / size_line;
        RT_ASSERT(h_align > 2);
        bool pre_show = false;
        uint8_t rotate_buf_i = 0;
        /*Avoid 'rotate_buf' is still using by previous blending, befre we overwrite it.        */
        HAL_StatusTypeDef ret =  Call_Hal_Api(HAL_API_ALL_STOP, NULL, NULL, NULL);
        DRV_EPIC_ASSERT(HAL_OK == ret);
        for (int16_t row = y_start; row < y_end; row += h_align - 2)
        {
            int16_t rev_line = y_end - row;
            dst_layer->height = h_align > rev_line ? rev_line : h_align;
            dst_layer->y_offset = row;
            dst_layer->transform_cfg.pivot_y = src_layer->transform_cfg.pivot_y - row + y_start;

            if (visible_after_transformed(dst_layer, p_clip_area))
            {
                pre_show = true;
                uint32_t offset = (row - y_start) * size_line;
                dst_layer->data_size = dst_layer->height * size_line;
                RT_ASSERT(dst_layer->data_size <= buf_size);
                dst_layer->data = (0 == rotate_buf_i) ? &gp_drv_epic->rotate_buf[0] : &gp_drv_epic->rotate_buf[rotate_buf_bytes >> 1];
                rotate_buf_i = !rotate_buf_i;
                memcpy(dst_layer->data, src_layer->data + offset, dst_layer->data_size);
                ret =  Call_Hal_Api(api, input_layers, (void *)input_layer_cnt, &output_layer);
                DRV_EPIC_ASSERT(HAL_OK == ret);
            }
            else if (pre_show)
                break;
        }
    }
#ifdef EPIC_SUPPORT_TRANS_MATRIX
    else if (p_operation->desc.blend.layer.transform_cfg.trans_matrix
             && (!HCPU_IS_SRAM_ADDR(p_operation->desc.blend.layer.data)))
    {
        sifli_matrix_3x3_t *p_matrix = p_operation->desc.blend.layer.transform_cfg.trans_matrix;
        const EPIC_LayerConfigTypeDef *src_layer = &p_operation->desc.blend.layer;
        EPIC_LayerConfigTypeDef *dst_layer = &input_layers[1];
        sifli_matrix_3x3_t chunk_matrix;
        sifli_matrix_3x3_t translate_matrix;

        uint32_t color_bytes = HAL_EPIC_GetColorDepth(src_layer->color_mode) >> 3;
        uint32_t buf_size = rotate_buf_bytes >> 1;
        int16_t h = src_layer->height;
        int16_t y_start = src_layer->y_offset;
        int16_t y_end = y_start + h;
        uint32_t size_line = color_bytes * src_layer->total_width;
        int16_t h_align = buf_size / size_line;
        RT_ASSERT(h_align > 2);
        bool pre_show = false;
        uint8_t rotate_buf_i = 0;

        /* Avoid overwrite when previous blending is still using rotate_buf. */
        ret = Call_Hal_Api(HAL_API_ALL_STOP, NULL, NULL, NULL);
        DRV_EPIC_ASSERT(HAL_OK == ret);

        for (int16_t row = y_start; row < y_end; row += h_align - 2)
        {
            int16_t rev_line = y_end - row;
            int16_t row_offset = row - y_start;
            uint32_t offset = row_offset * size_line;

            dst_layer->height = h_align > rev_line ? rev_line : h_align;
#if 1
            dst_layer->y_offset = y_start;
            memcpy(&chunk_matrix, p_matrix, sizeof(chunk_matrix));
            sifli_mat_translate_3x3(0.0f, (float)row_offset, &chunk_matrix);
            dst_layer->transform_cfg.trans_matrix = &chunk_matrix;
#else
            dst_layer->y_offset = row;//row_offset + y_start = row
#endif

            if (visible_after_matrix_transformed(dst_layer, p_clip_area))
            {
                pre_show = true;
                dst_layer->data_size = dst_layer->height * size_line;
                RT_ASSERT(dst_layer->data_size <= buf_size);
                dst_layer->data = (0 == rotate_buf_i) ? &gp_drv_epic->rotate_buf[0] : &gp_drv_epic->rotate_buf[rotate_buf_bytes >> 1];
                rotate_buf_i = !rotate_buf_i;

                memcpy(dst_layer->data, src_layer->data + offset, dst_layer->data_size);

                ret = Call_Hal_Api(api, input_layers, (void *)input_layer_cnt, &output_layer);
                DRV_EPIC_ASSERT(HAL_OK == ret);
            }
            else if (pre_show)
                break;
        }
        dst_layer->transform_cfg.trans_matrix = p_matrix;
    }
#endif
    else
#endif
    {
        if (EPIC_BLEND_MODE_NORMAL != p_operation->desc.blend.use_dest_as_bg)
        {
            output_layer.color_en = (EPIC_BLEND_MODE_FIXED_BG == p_operation->desc.blend.use_dest_as_bg);
            output_layer.color_r  = p_operation->desc.blend.r;
            output_layer.color_g  = p_operation->desc.blend.g;
            output_layer.color_b  = p_operation->desc.blend.b;
            ret =  Call_Hal_Api(api, &input_layers[1], (void *)(input_layer_cnt - 1), &output_layer);
        }
        else
        {
            ret =  Call_Hal_Api(api, input_layers, (void *)input_layer_cnt, &output_layer);
        }
        DRV_EPIC_ASSERT(HAL_OK == ret);
    }

}


static void render_fill(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    uint32_t argb8888 = (((uint32_t)p_operation->desc.fill.opa) << 24)
                        | (((uint32_t)p_operation->desc.fill.r) << 16)
                        | (((uint32_t)p_operation->desc.fill.g) << 8)
                        | (((uint32_t)p_operation->desc.fill.b) << 0);
    draw_fill(dst, &p_operation->mask, p_clip_area, argb8888);
}


static void render_layer(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    if (gp_drv_epic->dbg_flag_print_exe_detail)
    {
        print_operation("EXECUTION>> <<render_layer", p_operation);
        LOG_E(FORMATED_LAYER_INFO(dst, "DST"));
    }

    if (HAL_EPIC_AreaWidth(p_clip_area) <= EPIC_COORDINATES_MAX)
    {
        render_image(p_operation, dst, p_clip_area);
    }
    else
    {
        EPIC_AreaTypeDef final_layer_clip_area;

        final_layer_clip_area.y0 = p_clip_area->y0;
        final_layer_clip_area.y1 = p_clip_area->y1;

        /* Horizontal block rendering */
        for (int16_t start_column = p_clip_area->x0; start_column <= p_clip_area->x1; start_column += EPIC_COORDINATES_MAX)
        {
            final_layer_clip_area.x0 = start_column;
            if (start_column + EPIC_COORDINATES_MAX - 1 >= p_clip_area->x1)
                final_layer_clip_area.x1 = p_clip_area->x1;
            else
                final_layer_clip_area.x1 = start_column + EPIC_COORDINATES_MAX - 1;

            render_image(p_operation, dst, &final_layer_clip_area);
        }
    }
}

static rt_err_t render_polygon(drv_epic_operation *p_operation, EPIC_LayerConfigTypeDef *dst, const EPIC_AreaTypeDef *p_clip_area)
{
    uint32_t point_cnt = p_operation->desc.polygon.point_cnt;

    EPIC_AreaTypeDef clip_area;
    if (!HAL_EPIC_AreaIntersect(&clip_area, &p_operation->clip_area, p_clip_area)) return RT_EOK;

    void **mask_list = epic_malloc((point_cnt + 1) * sizeof(void *));
    drv_epic_mask_line_param_t *mp = epic_malloc(point_cnt * sizeof(drv_epic_mask_line_param_t));
    drv_epic_mask_line_param_t *mp_next = mp;
    RT_ASSERT(mask_list);
    RT_ASSERT(mp);
    EPIC_PointTypeDef *points = p_operation->desc.polygon.points;
    /*Find the lowest point*/
    int16_t y_min = points[0].y;
    int16_t y_min_i = 0;

    for (int16_t i = 1; i < point_cnt; i++)
    {
        if (points[i].y < y_min)
        {
            y_min = points[i].y;
            y_min_i = i;
        }
    }

    int32_t i_prev_left = y_min_i;
    int32_t i_prev_right = y_min_i;
    int32_t i_next_left;
    int32_t i_next_right;
    uint32_t mask_cnt = 0;

    /*Get the index of the left and right points*/
    i_next_left = y_min_i - 1;
    if (i_next_left < 0) i_next_left = point_cnt + i_next_left;

    i_next_right = y_min_i + 1;
    if (i_next_right > point_cnt - 1) i_next_right = 0;

    /**
     * Check if the order of points is inverted or not.
     * The normal case is when the left point is on `y_min_i - 1`
     * Explanation:
     *   if angle(p_left) < angle(p_right) -> inverted
     *   dy_left/dx_left < dy_right/dx_right
     *   dy_left * dx_right < dy_right * dx_left
     */
    int16_t dxl = points[i_next_left].x - points[y_min_i].x;
    int16_t dxr = points[i_next_right].x - points[y_min_i].x;
    int16_t dyl = points[i_next_left].y - points[y_min_i].y;
    int16_t dyr = points[i_next_right].y - points[y_min_i].y;

    bool inv = false;
    if (dyl * dxr < dyr * dxl) inv = true;
    uint16_t make_index = 0;
    uint16_t i = 0;
    for (i = 0; i <  point_cnt; i ++)
    {
        if (!inv)
        {
            i_next_left = i_prev_left - 1;
            if (i_next_left < 0) i_next_left = point_cnt + i_next_left;

            i_next_right = i_prev_right + 1;
            if (i_next_right > point_cnt - 1) i_next_right = 0;
        }
        else
        {
            i_next_left = i_prev_left + 1;
            if (i_next_left > point_cnt - 1) i_next_left = 0;

            i_next_right = i_prev_right - 1;
            if (i_next_right < 0) i_next_right = point_cnt + i_next_right;
        }

        if (points[i_next_left].y >= points[i_prev_left].y)
        {
            if (points[i_next_left].y != points[i_prev_left].y &&
                    points[i_next_left].x != points[i_prev_left].x)
            {
                drv_epic_mask_line_points_init(mp_next, points[i_prev_left].x, points[i_prev_left].y,
                                               points[i_next_left].x, points[i_next_left].y,
                                               DRAW_MASK_LINE_SIDE_RIGHT);
                mask_list[make_index++] = mp_next;
                mp_next++;
            }
            mask_cnt++;
            i_prev_left = i_next_left;
        }

        if (mask_cnt == point_cnt) break;

        if (points[i_next_right].y >= points[i_prev_right].y)
        {
            if (points[i_next_right].y != points[i_prev_right].y &&
                    points[i_next_right].x != points[i_prev_right].x)
            {

                drv_epic_mask_line_points_init(mp_next, points[i_prev_right].x, points[i_prev_right].y,
                                               points[i_next_right].x, points[i_next_right].y,
                                               DRAW_MASK_LINE_SIDE_LEFT);
                mask_list[make_index++] = mp_next;
                mp_next++;
            }
            mask_cnt++;
            i_prev_right = i_next_right;
        }
        if (mask_cnt == point_cnt) break;
    }

    mask_list[make_index] = NULL;

    if (point_cnt != i)
        draw_masked_rect4(dst, &p_operation->mask,
                          mask_list, &clip_area, p_operation->desc.polygon.argb8888);

    for (int16_t i = 0; i < make_index; i++) drv_epic_mask_free_param(mask_list[i]);

    epic_free(mask_list);
    epic_free(mp);

    return RT_EOK;
}

static rt_err_t render(drv_epic_render_list_t list)
{
    EPIC_AreaTypeDef intersect_area;
    priv_render_list_t *rl = (priv_render_list_t *) list;
    EPIC_LayerConfigTypeDef *dst = &rl->dst;
    rt_err_t ret_v = RT_EEMPTY;
    if (gp_drv_epic->dbg_flag_print_exe_detail)
    {
        LOG_E("\n");
        LOG_E(FORMATED_LAYER_INFO(dst, "--Render DST--"));
    }

    if (rl->letter_pool_free > gp_drv_epic->letter_pool_used_max)
        gp_drv_epic->letter_pool_used_max = rl->letter_pool_free;

    uint16_t i = 0;
    rt_list_t *pos;
    rt_list_for_each(pos, (&rl->src_list))
    {
        drv_epic_operation *p_operation = rt_list_entry(pos, drv_epic_operation, list);
        i++;
        if (_layer_IntersectArea(dst, &p_operation->clip_area, &intersect_area))
        {
            if (0 == (gp_drv_epic->dbg_flag_dis_operations & (1 << p_operation->op)))
            {
#if !defined(EPIC_SUPPORT_MASK)
                RT_ASSERT(NULL == p_operation->mask.data);
#endif

                if (gp_drv_epic->dbg_flag_print_exe_detail)
                {
                    char idex_str[32];
                    rt_sprintf(&idex_str[0], "EXECUTE>> << %d", i);
                    print_operation(&idex_str[0], p_operation);
                }

                uint32_t start_tick = HAL_DBG_DWT_GetCycles();
                uint32_t epic_sync_wait_cnt = gp_drv_epic->epic_handle.WaitCnt;
                uint32_t epic_async_wait_cnt = gp_drv_epic->rd_epic_async_wait;

#if debug_rl_hist_num > 0
                priv_render_hist_t *p_cur_hist;
                gp_drv_epic->hist_idx = (gp_drv_epic->hist_idx + 1) % debug_rl_hist_num;
                p_cur_hist = &gp_drv_epic->hist[gp_drv_epic->hist_idx];
                p_cur_hist->rl = rl;
                p_cur_hist->src_idx = i;
                p_cur_hist->operation = p_operation;
                p_cur_hist->start_tick = start_tick;
                p_cur_hist->cost_us = 0;
#endif /* debug_rl_hist_num > 0 */

                switch (p_operation->op)
                {
                case DRV_EPIC_DRAW_LETTERS:
                    render_letters(p_operation, dst, &intersect_area);
                    break;

                case DRV_EPIC_DRAW_ARC:
                    render_arc(p_operation, dst, &intersect_area);
                    break;

                case DRV_EPIC_DRAW_RECT:
                    render_rectangle(p_operation, dst, &intersect_area);
                    break;

                case DRV_EPIC_DRAW_IMAGE:
                    render_image(p_operation, dst, &intersect_area);
                    break;

                case DRV_EPIC_DRAW_FILL:
                    render_fill(p_operation, dst, &intersect_area);
                    break;

                case DRV_EPIC_DRAW_LINE:
                {
                    render_line(p_operation, dst, &intersect_area);
                    for (uint32_t i = 0; i < p_operation->desc.line.point_cnt;)
                    {
                        drv_epic_operation op = *p_operation;
                        op.desc.line.p1 = op.desc.line.p_points[i++];
                        op.desc.line.p2 = op.desc.line.p_points[i++];
                        render_line(&op, dst, &intersect_area);
                    }
                    break;
                }
                case DRV_EPIC_DRAW_BORDER:
                    render_border(p_operation, dst, &intersect_area);
                    break;

                case DRV_EPIC_DRAW_POLYGON:
                    render_polygon(p_operation, dst, &intersect_area);
                    break;

                default:
                    RT_ASSERT(0);
                    break;
                }


                //Notify the continue blend operations should be restarted next time
                HAL_StatusTypeDef ret =  Call_Hal_Api(HAL_API_CONT_BLEND_ASYNC_STOP, NULL, NULL, NULL);
                DRV_EPIC_ASSERT(HAL_OK == ret);

                uint32_t epic_sync_wait_us   = GetElapsedUs(epic_sync_wait_cnt, gp_drv_epic->epic_handle.WaitCnt);
                uint32_t epic_async_wait_us  = (gp_drv_epic->rd_epic_async_wait - epic_async_wait_cnt) * 1000;
                uint32_t sw_cost_us = GetElapsedUs(start_tick, HAL_DBG_DWT_GetCycles()) - epic_sync_wait_us - epic_async_wait_us;

#if debug_rl_hist_num > 0
                p_cur_hist->cost_us = sw_cost_us;
#endif
                gp_drv_epic->rd_operations_detail[p_operation->op].sw += sw_cost_us;
                gp_drv_epic->rd_operations_detail[p_operation->op].async_wait += epic_async_wait_us;
                gp_drv_epic->rd_operations_detail[p_operation->op].sync_wait += epic_sync_wait_us;
                gp_drv_epic->p_last_rd_operation = &gp_drv_epic->rd_operations_detail[p_operation->op];
            }

            ret_v = RT_EOK;

        }
    }



    return ret_v;
}

static rt_err_t render_list(priv_render_list_t *rl)
{
    __DEBUG_RENDER_LIST_START__;

    rt_err_t ret;

    /* If both the width and height do not exceed the maximum values, render directly. */
    if (rl->dst.width <= EPIC_COORDINATES_MAX && rl->dst.height <= EPIC_COORDINATES_MAX)
    {
        return render((drv_epic_render_list_t)rl);
    }
    else
    {
        /* The current rendering area, that is, the rendering area after block division */
        EPIC_AreaTypeDef render_area;
        /* Before block rendering, the original data should be backed up first. After the block division is completed,
           the data will be restored, and these data will be used for subsequent operations.*/
        EPIC_LayerConfigTypeDef dst = rl->dst;

        /* Calculate the maximum value of rows/columns */
        int16_t max_columns = rl->dst.x_offset + rl->dst.width - 1;
        int16_t max_rows = rl->dst.y_offset + rl->dst.height - 1;

        /* Vertical block rendering */
        for (int16_t start_rows = dst.y_offset; start_rows <= max_rows; start_rows += EPIC_COORDINATES_MAX)
        {
            render_area.y0 = start_rows;

            if (start_rows + EPIC_COORDINATES_MAX - 1 >= max_rows)
                render_area.y1 = max_rows;
            else
                render_area.y1 = start_rows + EPIC_COORDINATES_MAX - 1;

            /* Horizontal block rendering */
            for (int16_t start_columns = dst.x_offset; start_columns <= max_columns; start_columns += EPIC_COORDINATES_MAX)
            {
                render_area.x0 = start_columns;

                if (start_columns + EPIC_COORDINATES_MAX - 1 >= max_columns)
                    render_area.x1 = max_columns;
                else
                    render_area.x1 = start_columns + EPIC_COORDINATES_MAX - 1;

                clip_layer_to_area((EPIC_BlendingDataType *)&rl->dst, (const uint8_t *)dst.data, dst.x_offset, dst.y_offset, &render_area);

                ret = render((drv_epic_render_list_t)rl);
            }
        }
        rl->dst = dst;
    }

    __DEBUG_RENDER_LIST_END__;

    return ret;
}



rt_err_t drv_epic_render_list(void *p_drv_epic, void *list)
{
    if (!p_drv_epic) return RT_EINVAL;

    gp_drv_epic = (EPIC_DrvTypeDef *)p_drv_epic;
    DRV_EPIC_ASSERT(EPIC_DRV_MAGIC_NUM == gp_drv_epic->magic_num);
    rt_err_t ret;

    ret = render_list(list);

    //Wait all blending operations to stop
    HAL_StatusTypeDef hal_ret =  Call_Hal_Api(HAL_API_ALL_STOP, NULL, NULL, NULL);
    DRV_EPIC_ASSERT(HAL_OK == hal_ret);

    return ret;
}

/*
    p_scaled_area - the real destination area of the `rl->dst`
*/
rt_err_t drv_epic_render_list_scale(void *p_drv_epic, void *list, void *p_scaled_area)
{
    if (!p_drv_epic) return RT_EINVAL;
    gp_drv_epic = (EPIC_DrvTypeDef *)p_drv_epic;
    DRV_EPIC_ASSERT(EPIC_DRV_MAGIC_NUM == gp_drv_epic->magic_num);

    rt_err_t ret;
    priv_render_list_t *rl = (priv_render_list_t *)list;


    EPIC_LayerConfigTypeDef *p_dst = &rl->dst;
    EPIC_AreaTypeDef invalid_area;
    _get_layer_area(&invalid_area, &rl->dst);

    //Get scaling values
    uint32_t scale_x, scale_y;
    scale_x = EPIC_INPUT_SCALE_NONE * p_dst->width / HAL_EPIC_AreaWidth(p_scaled_area);
    scale_y = EPIC_INPUT_SCALE_NONE * p_dst->height / HAL_EPIC_AreaHeight(p_scaled_area);

    /*
        Firstly, render the image to the 'gp_drv_epic->buf1&2' buffer,
        then scale the image to the rl->dst layer
    */

    EPIC_LayerConfigTypeDef final_layer;
    EPIC_AreaTypeDef *p_final_area = p_scaled_area;

    //1.Save original dst layer info to final_layer
    memcpy(&final_layer, &rl->dst, sizeof(EPIC_LayerConfigTypeDef));
    /*
        Restore real size of final layer from 'p_scaled_area',
        the rl->dst layer's size is the invalid_area of rl.
    */
    final_layer.x_offset = p_final_area->x0;
    final_layer.y_offset = p_final_area->y0;
    final_layer.width = HAL_EPIC_AreaWidth(p_final_area);
    final_layer.height = HAL_EPIC_AreaHeight(p_final_area);
    final_layer.total_width = final_layer.width;

    //2. Replace dst layer with 'gp_drv_epic->buf1&2'
    uint32_t color_bytes = HAL_EPIC_GetColorDepth(p_dst->color_mode) >> 3;
    p_dst->width = HAL_EPIC_AreaWidth(&invalid_area);
    p_dst->total_width = p_dst->width;
    uint32_t max_buf = MIN(gp_drv_epic->dbg_render_buf_max, gp_drv_epic->buf_bytes);
    uint32_t max_row = (uint32_t)(max_buf / color_bytes) / p_dst->total_width;
    DRV_EPIC_ASSERT(max_row > 0);
    p_dst->height = max_row;
    p_dst->data_size = color_bytes * p_dst->total_width * p_dst->height;
    p_dst->data = (uint8_t *)gp_drv_epic->cur_buf;


    //Setup scaling image operation
    drv_epic_operation draw_img_op;
    draw_img_op.op = DRV_EPIC_DRAW_IMAGE;
    draw_img_op.clip_area = *p_final_area;
    HAL_EPIC_LayerConfigInit(&draw_img_op.mask);
    draw_img_op.desc.blend.layer = *p_dst;
    draw_img_op.desc.blend.layer.transform_cfg.pivot_x = 0;
    draw_img_op.desc.blend.layer.transform_cfg.scale_x = scale_x;
    draw_img_op.desc.blend.layer.transform_cfg.scale_y = scale_y;
    draw_img_op.desc.blend.use_dest_as_bg = EPIC_BLEND_MODE_OVERWRITE;

    EPIC_AreaTypeDef final_layer_clip_area;
    //The maximum copied rows per time
    uint32_t final_layer_max_row = max_row * EPIC_INPUT_SCALE_NONE / scale_y;
    final_layer_clip_area.x0 = p_final_area->x0;
    final_layer_clip_area.x1 = p_final_area->x1;

    //3. Render the image one by one
    int16_t line_cnt_pre = scale_y == EPIC_INPUT_SCALE_NONE ? final_layer_max_row : (final_layer_max_row - 2);
    DRV_EPIC_ASSERT(line_cnt_pre > 0);
    /*In the case of scaling, the last drawing may be drawn twice,
    resulting in the callback function being called twice with two errors*/
    uint32_t last = 0;
    for (int16_t start_row = p_final_area->y0; start_row <= p_final_area->y1 && !last; start_row += line_cnt_pre)
    {
        final_layer_clip_area.y0 = start_row;
        if (start_row + final_layer_max_row - 1 >= p_final_area->y1)
        {
            final_layer_clip_area.y1 = p_final_area->y1;
            last = 1;
        }
        else
        {
            final_layer_clip_area.y1 = start_row + final_layer_max_row - 1;
            last = 0;
        }


        p_dst->y_offset = p_final_area->y0 + (start_row - p_final_area->y0) * scale_y / EPIC_INPUT_SCALE_NONE;


        if (p_dst->y_offset + p_dst->height - 1 >= invalid_area.y1)
        {
            p_dst->height = invalid_area.y1 - p_dst->y_offset + 1;
        }
        else
        {
            ; //Keep the height as max_row
        }

        if (RT_EOK == render_list(rl))
        {
            //Update the changes from p_dst
            draw_img_op.desc.blend.layer.data = p_dst->data;
            draw_img_op.desc.blend.layer.y_offset = p_dst->y_offset;
            draw_img_op.desc.blend.layer.height  = p_dst->height;
            draw_img_op.desc.blend.layer.transform_cfg.pivot_y = - p_dst->y_offset + p_final_area->y0;

            render_layer(&draw_img_op, &final_layer, &final_layer_clip_area);
        }
        else
        {
            ;//Ignore EMPTY rendering
        }

        if (gp_drv_epic->cur_buf == (uint8_t *)gp_drv_epic->buf1)
            gp_drv_epic->cur_buf = (uint8_t *)gp_drv_epic->buf2;
        else
            gp_drv_epic->cur_buf = (uint8_t *)gp_drv_epic->buf1;

        p_dst->data = gp_drv_epic->cur_buf;
    }

    //Wait all blending operations to stop
    HAL_StatusTypeDef hal_ret =  Call_Hal_Api(HAL_API_ALL_STOP, NULL, NULL, NULL);
    DRV_EPIC_ASSERT(HAL_OK == hal_ret);

    return ret;
}

#endif /*DRV_EPIC_NEW_API*/