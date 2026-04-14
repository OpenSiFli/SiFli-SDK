/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_EPIC_H__
#define __DRV_EPIC_H__

#include <rtdevice.h>
#include "bf0_hal.h"

#define DRV_EPIC_TIMEOUT_MS 500

#define DRV_EPIC_POLYGON_POINT_MAX 16
#ifndef DRV_EPIC_NEW_API

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

typedef void (*drv_epic_cplt_cbk)(EPIC_HandleTypeDef *);

rt_err_t drv_epic_fill_ext(EPIC_LayerConfigTypeDef *input_layers,
                           uint8_t input_layer_cnt,
                           EPIC_LayerConfigTypeDef *output_canvas,
                           drv_epic_cplt_cbk cbk);

rt_err_t drv_epic_fill(uint32_t dst_cf, uint8_t *dst,
                       const EPIC_AreaTypeDef *dst_area,
                       const EPIC_AreaTypeDef *fill_area,
                       uint32_t argb8888,
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
                            drv_epic_cplt_cbk cbk);

rt_err_t drv_epic_cont_blend(EPIC_LayerConfigTypeDef *input_layers,
                             uint8_t input_layer_cnt,
                             EPIC_LayerConfigTypeDef *output_canvas);

void drv_epic_cont_blend_reset(void);

#else /*DRV_EPIC_NEW_API*/

typedef enum
{
    DRV_EPIC_DRAW_MIN = 0,
    DRV_EPIC_DRAW_IMAGE = DRV_EPIC_DRAW_MIN,
    DRV_EPIC_DRAW_FILL,
    DRV_EPIC_DRAW_LETTERS,
    DRV_EPIC_DRAW_ARC,
    DRV_EPIC_DRAW_RECT,
    DRV_EPIC_DRAW_LINE,
    DRV_EPIC_DRAW_BORDER,
    DRV_EPIC_DRAW_POLYGON,
    DRV_EPIC_DRAW_MAX,
    DRV_EPIC_INVALID = 0xFFFF,      //Invalid
} drv_epic_op_type_t;

typedef enum
{
    DRV_EPIC_ROT_NONE = 0,
    DRV_EPIC_ROT_90,
    DRV_EPIC_ROT_180,
    DRV_EPIC_ROT_270
} drv_epic_rotate_t;

typedef struct
{
    const uint8_t *data;
    EPIC_AreaTypeDef area;
} drv_epic_letter_type_t;

typedef enum
{
    EPIC_BLEND_MODE_FIXED_BG = 0,       //Fixed background color, no blending with destination buffer
    EPIC_BLEND_MODE_NORMAL  = 1,        //Normal mode, blending with destination buffer
    EPIC_BLEND_MODE_OVERWRITE  = 2,       //Overwrite mode, no background
} drv_epic_blend_mode_type_t;

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
            uint8_t use_dest_as_bg; // See drv_epic_blend_mode_type_t
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
        struct
        {
            int16_t center_x;
            int16_t center_y;

            uint16_t start_angle;
            uint16_t end_angle;

            uint16_t width;
            uint16_t radius;
            uint8_t round_start;
            uint8_t round_end;

            uint32_t argb8888;
        } arc;
        struct
        {
            EPIC_AreaTypeDef area;

            uint16_t radius;
            uint8_t top_fillet;
            uint8_t bot_fillet;

            uint32_t argb8888;
        } rectangle;
        struct
        {
            EPIC_PointTypeDef p1;
            EPIC_PointTypeDef p2;

            uint16_t width;
            int32_t dash_width;
            int32_t dash_gap;
            uint32_t argb8888;
            uint8_t round_start;
            uint8_t round_end;
            uint8_t raw_end;
        } line;
        struct
        {
            EPIC_AreaTypeDef area;

            uint16_t radius;
            uint16_t width;

            uint8_t top_side : 1;
            uint8_t bot_side : 1;
            uint8_t left_side : 1;
            uint8_t right_side : 1;
            uint8_t reserved : 4;

            uint32_t argb8888;
        } border;
        struct
        {
            EPIC_PointTypeDef points[DRV_EPIC_POLYGON_POINT_MAX];
            uint16_t point_cnt;
            uint32_t argb8888;
        } polygon;
    } desc;

    //Offset to specified dst buf, internal use only
    int16_t offset_x;
    int16_t offset_y;
    rt_list_t list; /**< list link */
} drv_epic_operation;

typedef struct
{
    uint32_t cf; /**< color mode, refer to EPIC_COLOR_XXX, like #EPIC_COLOR_RGB565 */
    uint8_t *data;
    EPIC_AreaTypeDef area;
} drv_epic_render_buf;

typedef enum
{
    /*Render and save results in render list's dest buffer*/
    EPIC_MSG_RENDER_TO_BUF,
    /*Render and draw results on LCD,
        has NOT effect with render list's dest buffer*/
    EPIC_MSG_RENDER_DRAW,
} EPIC_MsgIdDef;

typedef void *drv_epic_render_list_t;
typedef void (*drv_epic_render_cb)(drv_epic_render_list_t rl, EPIC_LayerConfigTypeDef *p_dst, void *usr_data, uint32_t last);
typedef void (*drv_epic_render_trav_cb)(drv_epic_operation *op, void *usr_data);

typedef struct
{
    EPIC_AreaTypeDef area;//render part of 'render_list'

    uint32_t pixel_align; //partial render pixel alignments
    drv_epic_render_cb partial_done_cb;
    void *usr_data;
} drv_epic_render_draw_cfg;

typedef struct
{
    /*
        The real size of 'render_list->dst'
        if 'dst_area' is not equal to 'render_list->dst',
        we'll draw 'render_list->dst' sized area first then scale to 'dst_area'
    */
    EPIC_AreaTypeDef dst_area;
    drv_epic_render_cb done_cb;
    void *usr_data;
} drv_epic_render_to_buf_cfg;

typedef struct
{
    EPIC_MsgIdDef id;
    drv_epic_render_list_t render_list;

    union
    {
        drv_epic_render_draw_cfg  rd;
        drv_epic_render_to_buf_cfg  r2b;
    } content;

    uint32_t tick; //Internal use
} EPIC_MsgTypeDef;

rt_err_t drv_epic_setup_render_buffer(uint8_t *buf1, uint8_t *buf2, uint32_t buf_bytes);

drv_epic_render_list_t drv_epic_alloc_render_list(drv_epic_render_buf *p_buf, EPIC_AreaTypeDef *p_ow_area);

drv_epic_operation *drv_epic_alloc_op(drv_epic_render_buf *p_buf);
drv_epic_letter_type_t *drv_epic_op_alloc_letter(drv_epic_operation *op);
rt_err_t drv_epic_commit_op(drv_epic_operation *op);

rt_err_t drv_epic_render_msg_commit(EPIC_MsgTypeDef *p_msg);

rt_err_t drv_epic_render_trav(drv_epic_render_list_t list, drv_epic_render_trav_cb cb, void *usr_data);

rt_err_t drv_epic_set_rotation(drv_epic_rotate_t rotate);
drv_epic_rotate_t drv_epic_get_rotation(void);
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
bool drv_epic_is_busy(void);
#endif /* __DRV_EPIC_H__ */

