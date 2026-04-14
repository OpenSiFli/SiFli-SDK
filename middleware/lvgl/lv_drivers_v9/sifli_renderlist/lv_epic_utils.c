/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "string.h"

/*********************
 *      DEFINES
 *********************/
#define GPU_OPERATION_MAX_PIXELS (1000*1000)

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
uint32_t lv_img_2_epic_cf(lv_color_format_t cf)
{
    uint32_t color_mode;

    switch (cf)
    {
    case LV_COLOR_FORMAT_RGB565:
        color_mode = EPIC_INPUT_RGB565;
        break;
#ifdef EPIC_SUPPORT_MASK
    case LV_COLOR_FORMAT_RGB565A8:
        color_mode = EPIC_INPUT_RGB565;
        break;
#endif

    case LV_COLOR_FORMAT_RGB888:
        color_mode = EPIC_INPUT_RGB888;
        break;
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
        color_mode = EPIC_INPUT_ARGB8888;
        break;
    case LV_COLOR_FORMAT_ARGB8565:
        color_mode = EPIC_INPUT_ARGB8565;
        break;

#ifdef EPIC_SUPPORT_A8
    case LV_COLOR_FORMAT_A8:
        color_mode = EPIC_INPUT_A8;
        break;
#endif

#ifdef EPIC_SUPPORT_A4
    case LV_COLOR_FORMAT_A4:
        color_mode = EPIC_INPUT_A4;
        break;
#endif

#ifdef EPIC_SUPPORT_A2
    case LV_COLOR_FORMAT_A2:
        color_mode = EPIC_INPUT_A2;
        break;
#endif

#ifdef EPIC_SUPPORT_L8
    case LV_COLOR_FORMAT_I8:
    case LV_COLOR_FORMAT_L8:
        color_mode = EPIC_INPUT_L8;
        break;
#endif

#ifdef EPIC_SUPPORT_YUV
    case LV_COLOR_FORMAT_I420:
        color_mode = EPIC_INPUT_YUV420_PLANAR;
        break;
    case LV_COLOR_FORMAT_YUY2:
        color_mode = EPIC_INPUT_YUV422_PACKED_YUYV;
        break;
    case LV_COLOR_FORMAT_UYVY:
        color_mode = EPIC_INPUT_YUV422_PACKED_UYVY;
        break;
#endif

    default:
        LV_LOG_USER("format %d \r\n", cf);
        LV_ASSERT_MSG(false, "Unsupported color format");
        break;
    }

    return color_mode;
}

EPIC_ColorDef lv_color_to_epic_color(lv_color_t color, lv_opa_t opa)
{
    EPIC_ColorDef c;
    c.ch.color_r = color.red;
    c.ch.color_g = color.green;
    c.ch.color_b = color.blue;
    c.ch.alpha = opa;
    return c;
}

void lv_epic_get_coord_offset(const lv_draw_task_t *draw_task, int32_t *offset_x, int32_t *offset_y, bool *is_sub_layer)
{
    if (offset_x) *offset_x = 0;
    if (offset_y) *offset_y = 0;
    if (is_sub_layer) *is_sub_layer = false;
    if (draw_task == NULL) return;

    lv_layer_t *target_layer = draw_task->target_layer;
    bool sub = (target_layer && target_layer->parent != NULL);
    if (is_sub_layer) *is_sub_layer = sub;

    if (sub)
    {
        if (offset_x) *offset_x = target_layer->buf_area.x1;
        if (offset_y) *offset_y = target_layer->buf_area.y1;
    }
}

void lv_epic_area_to_render_coords(const lv_area_t *src_area, int32_t offset_x, int32_t offset_y, lv_area_t *dst_area)
{
    if (src_area == NULL || dst_area == NULL) return;

    dst_area->x1 = src_area->x1 - offset_x;
    dst_area->y1 = src_area->y1 - offset_y;
    dst_area->x2 = src_area->x2 - offset_x;
    dst_area->y2 = src_area->y2 - offset_y;
}

void lv_epic_point_to_render_coords(int32_t src_x, int32_t src_y, int32_t offset_x, int32_t offset_y, int32_t *dst_x, int32_t *dst_y)
{
    if (dst_x) *dst_x = src_x - offset_x;
    if (dst_y) *dst_y = src_y - offset_y;
}

bool lv_epic_clip_area_by_render_buf(lv_area_t *area, const drv_epic_render_buf *render_buf)
{
    if (area == NULL || render_buf == NULL) return false;

    lv_area_t render_area;
    render_area.x1 = render_buf->area.x0;
    render_area.y1 = render_buf->area.y0;
    render_area.x2 = render_buf->area.x1;
    render_area.y2 = render_buf->area.y1;

    return lv_area_intersect(area, area, &render_area);
}

uint32_t lv_epic_setup_bg_and_output_layer(EPIC_LayerConfigTypeDef *epic_bg_layer,
        EPIC_LayerConfigTypeDef *epic_output_layer,
        lv_draw_task_t *draw_task, const lv_area_t *coords)
{
    lv_layer_t *layer = draw_task->target_layer;

    lv_area_t blend_area;
    if (!lv_area_intersect(&blend_area, coords, &draw_task->clip_area))
        return 1;

    lv_color_format_t dest_cf = layer->color_format;

    HAL_EPIC_LayerConfigInit(epic_bg_layer);
    epic_bg_layer->color_en = false;
    epic_bg_layer->data = (uint8_t *)lv_draw_layer_go_to_xy(layer, 0, 0);
    epic_bg_layer->color_mode = lv_img_2_epic_cf(dest_cf);
    epic_bg_layer->width = lv_area_get_width(&layer->buf_area);
    epic_bg_layer->total_width = lv_area_get_width(&layer->buf_area);
    epic_bg_layer->height = lv_area_get_height(&layer->buf_area);
    epic_bg_layer->x_offset = layer->buf_area.x1;
    epic_bg_layer->y_offset = layer->buf_area.y1;


    memcpy(epic_output_layer, epic_bg_layer, sizeof(EPIC_LayerConfigTypeDef));
    epic_output_layer->width = lv_area_get_width(&blend_area);
    epic_output_layer->height = lv_area_get_height(&blend_area);
    epic_output_layer->x_offset = blend_area.x1;
    epic_output_layer->y_offset = blend_area.y1;

    epic_output_layer->data = (uint8_t *)lv_draw_layer_go_to_xy(layer,
                              blend_area.x1 - layer->buf_area.x1,
                              blend_area.y1 - layer->buf_area.y1);


    LV_ASSERT((epic_output_layer->height * epic_output_layer->width) <= GPU_OPERATION_MAX_PIXELS);

    return 0;
}

void lv_epic_print_area_info(const char *prefix, const lv_area_t *p_area)
{
    LV_EPIC_LOG("%s[%d,%d,%d,%d]", prefix, p_area->x1, p_area->y1, p_area->x2, p_area->y2);
}
void lv_epic_print_layer_info(lv_draw_task_t *draw_task)
{
    lv_layer_t *layer = draw_task->target_layer;
    LV_EPIC_LOG("format %d, buf=%p, stride=%u", layer->color_format,
                layer->draw_buf->data, layer->draw_buf->header.stride);

    lv_epic_print_area_info("buf_area", &layer->buf_area);
    lv_epic_print_area_info("clip_area", &draw_task->clip_area);
}
/**********************
 *   STATIC FUNCTIONS
 **********************/


#endif /*LV_USE_DRAW_EPIC*/
