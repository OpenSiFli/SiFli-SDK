/**
  ******************************************************************************
  * @file   drv_epic.h
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

#ifndef __DRV_EPIC_H__
#define __DRV_EPIC_H__

#include <rtdevice.h>
#include "bf0_hal.h"

//#define DRV_EPIC_NEW_API

typedef enum
{
    DRV_EPIC_COLOR_BLEND, //0
    DRV_EPIC_COLOR_FILL,  //1
    DRV_EPIC_IMG_ROT,     //2
    DRV_EPIC_IMG_COPY,    //3
    DRV_EPIC_LETTER_BLEND, //4
    DRV_EPIC_TRANSFORM,
    DRV_EPIC_FILL_GRAD,
    DRV_EPIC_INVALID = 0xFFFF,      //Invalid
} drv_epic_op_type_t;

#define DRV_EPIC_TIMEOUT_MS 500

#ifndef DRV_EPIC_NEW_API


typedef void (*drv_epic_cplt_cbk)(EPIC_HandleTypeDef *);


rt_err_t drv_epic_fill_ext(EPIC_LayerConfigTypeDef *input_layers,
                           uint8_t input_layer_cnt,
                           EPIC_LayerConfigTypeDef *output_canvas,
                           drv_epic_cplt_cbk cbk);

rt_err_t drv_epic_fill(uint32_t dst_cf, uint8_t *dst,
                       const EPIC_AreaTypeDef *dst_area,
                       const EPIC_AreaTypeDef *fill_area,
                       uint32_t fill_color,
                       uint32_t mask_cf, const uint8_t *mask,
                       const EPIC_AreaTypeDef *mask_area,
                       drv_epic_cplt_cbk cbk);

rt_err_t drv_epic_fill_grad(EPIC_GradCfgTypeDef *param,
                            drv_epic_cplt_cbk cbk);

rt_err_t drv_epic_copy(const uint8_t *src, uint8_t *dst,
                       const EPIC_AreaTypeDef *src_area,
                       const EPIC_AreaTypeDef *dst_area,
                       const EPIC_AreaTypeDef *copy_area,
                       uint32_t src_cf, uint32_t dst_cf,
                       drv_epic_cplt_cbk cbk);

rt_err_t drv_epic_blend(EPIC_LayerConfigTypeDef *input_layers,
                        uint8_t input_layer_cnt,
                        EPIC_LayerConfigTypeDef *output_canvas,
                        drv_epic_cplt_cbk cbk);


rt_err_t drv_epic_transform(EPIC_LayerConfigTypeDef *input_layers,
                            uint8_t input_layer_cnt,
                            EPIC_LayerConfigTypeDef *output_canvas,
                            EPIC_TransPath hor_path,
                            EPIC_TransPath ver_path,
                            void *user_data,
                            drv_epic_cplt_cbk cbk);


rt_err_t drv_epic_cont_blend(EPIC_LayerConfigTypeDef *input_layers,
                             uint8_t input_layer_cnt,
                             EPIC_LayerConfigTypeDef *output_canvas);

void drv_epic_cont_blend_reset(void);

#else /*DRV_EPIC_NEW_API*/


typedef struct
{
    const uint8_t *data;
    EPIC_AreaTypeDef area;
} drv_epic_letter_type_t;


typedef struct
{
    drv_epic_op_type_t  op;
    EPIC_AreaTypeDef clip_area;
    EPIC_LayerConfigTypeDef mask;

    union
    {
        struct
        {
            EPIC_LayerConfigTypeDef layer;
            uint8_t use_dest_as_bg;
            uint8_t r;
            uint8_t g;
            uint8_t b;
        } blend;
        struct
        {
            uint8_t r;
            uint8_t g;
            uint8_t b;
            uint8_t opa;
        } fill;
        struct
        {
            uint32_t color_mode;
            uint8_t r;
            uint8_t g;
            uint8_t b;
            uint8_t opa;

            uint32_t letter_num;
            drv_epic_letter_type_t *p_letters;
        } label;
    } desc;

    //Offset to specified dst buf, internal use only
    int16_t offset_x;
    int16_t offset_y;
} drv_epic_operation;


typedef struct
{
    uint32_t cf; /**< color mode, refer to EPIC_COLOR_XXX, like #EPIC_COLOR_RGB565 */
    uint8_t *data;
    EPIC_AreaTypeDef area;
} drv_epic_render_buf;

typedef enum
{
    EPIC_MSG_RENDER_DRAW,
} EPIC_MsgIdDef;

typedef uint32_t *drv_epic_render_list_t;
typedef struct
{
    uint32_t cf;
    uint8_t *buf1;
    uint8_t *buf2;
    EPIC_AreaTypeDef area;
    uint32_t buf_bytes;

    drv_epic_render_list_t render_list;
    rt_device_t lcd_dev;
    uint8_t pixel_align;
} drv_epic_render_draw_cfg;


typedef struct
{
    EPIC_MsgIdDef id;
    uint32_t tick;

    union
    {
        drv_epic_render_draw_cfg  rd;
    } content;


} EPIC_MsgTypeDef;



typedef void (*drv_epic_render_cb)(EPIC_LayerConfigTypeDef *dst);


drv_epic_render_list_t drv_epic_alloc_render_list(drv_epic_render_buf *p_buf, EPIC_AreaTypeDef *p_ow_area);

drv_epic_operation *drv_epic_alloc_op(drv_epic_render_buf *p_buf);
drv_epic_letter_type_t *drv_epic_op_alloc_letter(drv_epic_operation *op);
rt_err_t drv_epic_commit_op(drv_epic_operation *op);


rt_err_t drv_epic_render_draw_commit(drv_epic_render_draw_cfg *cfg);


#endif /* DRV_EPIC_NEW_API */



EPIC_HandleTypeDef *drv_get_epic_handle(void);

#ifdef HAL_EZIP_MODULE_ENABLED
    EZIP_HandleTypeDef *drv_get_ezip_handle(void);
    #ifdef SF32LB58X
        EZIP_HandleTypeDef *drv_get_ezip2_handle(void);
    #endif
#endif


void drv_gpu_open(void);
void drv_gpu_close(void);

rt_err_t drv_gpu_take(rt_int32_t ms);
rt_err_t drv_gpu_release(void);
rt_err_t drv_gpu_check_done(rt_int32_t ms);
#define drv_epic_take      drv_gpu_take
#define drv_epic_release   drv_gpu_release
#define drv_epic_wait_done() drv_gpu_check_done(DRV_EPIC_TIMEOUT_MS)
/**
 * @brief Get the specified ram block cache state
 * @param start   start address
 * @param len     ram length
 * @return        0 - if not cached  1 - cached
 */
uint8_t drv_gpu_is_cached_ram(uint32_t start, uint32_t len);

#endif /* __DRV_EPIC_H__ */





/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
