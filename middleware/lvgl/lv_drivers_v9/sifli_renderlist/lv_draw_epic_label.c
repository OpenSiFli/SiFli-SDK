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

    #include "blend/lv_draw_sw_blend_private.h"
    #include "../lv_draw_label_private.h"
    #include <string.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
    drv_epic_render_buf *render_buf;
    drv_epic_operation *pending_op;
    lv_area_t clip_area;
    lv_color_t color;
    lv_opa_t opa;
    int32_t coord_offset_x;
    int32_t coord_offset_y;
} label_draw_ctx_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void _draw_epic_letter(lv_draw_task_t *draw_task,
                              lv_draw_glyph_dsc_t *glyph_draw_dsc,
                              lv_draw_fill_dsc_t *fill_draw_dsc,
                              const lv_area_t *fill_area);
static void label_draw_ctx_reset(void);
static void label_draw_ctx_flush_pending_op(void);
static drv_epic_operation *
label_draw_ctx_get_pending_op(const lv_draw_glyph_dsc_t *glyph_draw_dsc);
/*
 * Copy a glyph bitmap from a strided source buffer into a tightly packed destination.
 *
 * Font engines often store glyph bitmaps with per-line padding (stride > width) for
 * memory alignment. EPIC hardware requires tightly packed A8 data where each row is
 * exactly 'width' bytes with no gap between rows.
 *
 * @param src    Source bitmap data (may have stride padding at end of each row)
 * @param width  Glyph width in pixels (bytes per row in the output)
 * @param height Glyph height in pixels (number of rows)
 * @param stride Bytes per row in the source buffer (>= width)
 * @return       Newly allocated tightly packed bitmap, or NULL on failure.
 *               Caller is responsible for freeing the returned buffer.
 */
static uint8_t *copy_glyph_bitmap_tightly(const uint8_t *src, uint32_t width,
                                          uint32_t height, uint32_t stride);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *  GLOBAL VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

static label_draw_ctx_t g_label_ctx;

void lv_draw_epic_label(lv_draw_task_t *draw_task,
                        const lv_draw_label_dsc_t *dsc, const lv_area_t *coords)
{
    if (dsc->opa <= LV_OPA_MIN)
        return;

    label_draw_ctx_reset();

    drv_epic_render_buf *render_buf = render_list_get_current_buf();
    if (render_buf == NULL)
    {
        LV_LOG_ERROR("EPIC: No render buffer available for label operation\n");
        LV_ASSERT(0);
        return;
    }

    bool is_sub_layer = false;
    lv_epic_get_coord_offset(draw_task, &g_label_ctx.coord_offset_x,
                             &g_label_ctx.coord_offset_y, &is_sub_layer);

    lv_area_t label_clip_area;
    lv_epic_area_to_render_coords(coords, g_label_ctx.coord_offset_x,
                                  g_label_ctx.coord_offset_y, &label_clip_area);
    if (!lv_epic_clip_area_by_render_buf(&label_clip_area, render_buf))
    {
        label_draw_ctx_reset();
        return;
    }

    g_label_ctx.render_buf = render_buf;
    g_label_ctx.clip_area = label_clip_area;

    lv_draw_label_iterate_characters(draw_task, dsc, coords, _draw_epic_letter);
    label_draw_ctx_flush_pending_op();
    label_draw_ctx_reset();
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void _draw_epic_letter(lv_draw_task_t *draw_task,
                              lv_draw_glyph_dsc_t *glyph_draw_dsc,
                              lv_draw_fill_dsc_t *fill_draw_dsc,
                              const lv_area_t *fill_area)
{
    if (glyph_draw_dsc)
    {
        switch (glyph_draw_dsc->format)
        {
        case LV_FONT_GLYPH_FORMAT_NONE:
        {
    #if LV_USE_FONT_PLACEHOLDER
            if (glyph_draw_dsc->bg_coords == NULL)
                break;
            label_draw_ctx_flush_pending_op();
            lv_draw_border_dsc_t border_draw_dsc;
            lv_draw_border_dsc_init(&border_draw_dsc);
            border_draw_dsc.opa = glyph_draw_dsc->opa;
            border_draw_dsc.color = glyph_draw_dsc->color;
            border_draw_dsc.width = 1;
            lv_draw_epic_border(draw_task, &border_draw_dsc,
                                glyph_draw_dsc->bg_coords);
    #endif
        }
        break;

        case LV_FONT_GLYPH_FORMAT_A1:
        case LV_FONT_GLYPH_FORMAT_A2:
        case LV_FONT_GLYPH_FORMAT_A3:
        case LV_FONT_GLYPH_FORMAT_A4:
        case LV_FONT_GLYPH_FORMAT_A8:
        case LV_FONT_GLYPH_FORMAT_IMAGE:
        {
            if (glyph_draw_dsc->rotation % 3600 == 0 &&
                glyph_draw_dsc->format != LV_FONT_GLYPH_FORMAT_IMAGE)
            {
                if (glyph_draw_dsc->opa <= LV_OPA_MIN)
                    return;

                if (glyph_draw_dsc->g == NULL ||
                    glyph_draw_dsc->letter_coords == NULL)
                {
                    LV_EPIC_LOG("ERROR: glyph meta or letter coords is NULL");
                    return;
                }

                uint32_t glyph_width =
                    lv_area_get_width(glyph_draw_dsc->letter_coords);
                uint32_t glyph_height =
                    lv_area_get_height(glyph_draw_dsc->letter_coords);
                if (glyph_width == 0 || glyph_height == 0)
                {
                    LV_EPIC_LOG("ERROR: glyph width/height is zero");
                    return;
                }

                lv_draw_buf_t *draw_buf = NULL;
                const uint8_t *bitmap_data = NULL;
                const uint8_t *letter_data = NULL;
                uint8_t *owned_bitmap = NULL;
                uint32_t mask_stride = 0;

                if (lv_font_has_static_bitmap(
                        glyph_draw_dsc->g->resolved_font) &&
                    glyph_draw_dsc->g->format == LV_FONT_GLYPH_FORMAT_A8)
                {
                    glyph_draw_dsc->g->req_raw_bitmap = 1;
                    bitmap_data =
                        (const uint8_t *)lv_font_get_glyph_static_bitmap(
                            glyph_draw_dsc->g);
                    mask_stride = glyph_draw_dsc->g->stride;
                    if (bitmap_data == NULL)
                    {
                        LV_EPIC_LOG("ERROR: failed to get static glyph bitmap");
                        return;
                    }
                    /*
                     * EPIC letter renderer sets total_width == width (no stride gap),
                     * so glyph data must be tightly packed. Check if font stride has
                     * per-line padding:
                     *   stride == width: zero-copy, use Flash pointer directly
                     *   stride >  width: copy row-by-row to strip padding
                     */
                    if (mask_stride == glyph_width)
                    {
                        letter_data = bitmap_data;
                    }
                    else
                    {
                        owned_bitmap = copy_glyph_bitmap_tightly(
                            bitmap_data, glyph_width, glyph_height,
                            mask_stride);
                        if (owned_bitmap == NULL)
                        {
                            LV_LOG_ERROR("EPIC: Failed to pack static glyph "
                                         "bitmap, size=%u bytes",
                                         glyph_width * glyph_height);
                            return;
                        }
                        /*
                         * Register the allocated bitmap with EPIC's glyph data pool.
                         * This transfers ownership of owned_bitmap to the EPIC engine,
                         * which will free it after the hardware finishes rendering.
                         * If retain fails, we must free it ourselves to avoid leak.
                         */
                        if (!lv_draw_epic_retain_glyph_data(owned_bitmap))
                        {
                            LV_LOG_ERROR("EPIC: Failed to retain packed static "
                                         "glyph bitmap");
                            lv_free(owned_bitmap);
                            return;
                        }
                        letter_data = owned_bitmap;
                    }
                }
                else
                {
                    if (glyph_draw_dsc->_draw_buf == NULL)
                    {
                        glyph_draw_dsc->_draw_buf = lv_draw_buf_create(
                            glyph_width, glyph_height, LV_COLOR_FORMAT_A8, 0);
                    }
                    if (glyph_draw_dsc->_draw_buf == NULL)
                    {
                        LV_EPIC_LOG("ERROR: failed to create draw buf");
                        return;
                    }
                    glyph_draw_dsc->glyph_data = lv_font_get_glyph_bitmap(
                        glyph_draw_dsc->g, glyph_draw_dsc->_draw_buf);
                    if (glyph_draw_dsc->glyph_data == NULL)
                    {
                        LV_EPIC_LOG("ERROR: failed to get glyph bitmap");
                        if (glyph_draw_dsc->_draw_buf)
                            lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                        glyph_draw_dsc->_draw_buf = NULL;
                        return;
                    }
                    draw_buf = (lv_draw_buf_t *)glyph_draw_dsc->glyph_data;
                    mask_stride = draw_buf->header.stride;
                    owned_bitmap = copy_glyph_bitmap_tightly(
                        (const uint8_t *)draw_buf->data, glyph_width,
                        glyph_height, mask_stride);
                    if (owned_bitmap == NULL)
                    {
                        LV_LOG_ERROR("EPIC: Failed to allocate memory for "
                                     "glyph data, size=%u bytes",
                                     glyph_width * glyph_height);
                        if (glyph_draw_dsc->_draw_buf)
                        {
                            lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                            glyph_draw_dsc->_draw_buf = NULL;
                        }
                        return;
                    }
                    if (!lv_draw_epic_retain_glyph_data(owned_bitmap))
                    {
                        LV_LOG_ERROR(
                            "EPIC: Failed to retain glyph data, size=%u bytes",
                            glyph_width * glyph_height);
                        lv_free(owned_bitmap);
                        if (glyph_draw_dsc->_draw_buf)
                        {
                            lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                            glyph_draw_dsc->_draw_buf = NULL;
                        }
                        return;
                    }
                    letter_data = owned_bitmap;
                }

                drv_epic_operation *label_op = label_draw_ctx_get_pending_op(glyph_draw_dsc);
                if (label_op == NULL)
                {
                    if (glyph_draw_dsc->_draw_buf)
                    {
                        lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                        glyph_draw_dsc->_draw_buf = NULL;
                    }
                    return;
                }

                drv_epic_letter_type_t *letter =
                    drv_epic_op_alloc_letter(label_op);
                letter->data = letter_data;

                lv_area_t letter_area_local;
                lv_epic_area_to_render_coords(
                    glyph_draw_dsc->letter_coords, g_label_ctx.coord_offset_x,
                    g_label_ctx.coord_offset_y, &letter_area_local);
                letter->area.x0 = letter_area_local.x1;
                letter->area.y0 = letter_area_local.y1;
                letter->area.x1 = letter_area_local.x2;
                letter->area.y1 = letter_area_local.y2;

                if (glyph_draw_dsc->_draw_buf)
                {
                    lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                    glyph_draw_dsc->_draw_buf = NULL;
                }
            }
            else
            {
                label_draw_ctx_flush_pending_op();
                glyph_draw_dsc->glyph_data = lv_font_get_glyph_bitmap(
                    glyph_draw_dsc->g, glyph_draw_dsc->_draw_buf);
                if (glyph_draw_dsc->glyph_data == NULL)
                {
                    LV_EPIC_LOG(
                        "ERROR: failed to get glyph bitmap for rotation/image");
                    if (glyph_draw_dsc->_draw_buf)
                    {
                        lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                        glyph_draw_dsc->_draw_buf = NULL;
                    }
                    break;
                }

                lv_draw_image_dsc_t img_dsc;
                lv_draw_image_dsc_init(&img_dsc);
                img_dsc.rotation = glyph_draw_dsc->rotation;
                img_dsc.scale_x = LV_SCALE_NONE;
                img_dsc.scale_y = LV_SCALE_NONE;
                img_dsc.opa = glyph_draw_dsc->opa;
                img_dsc.src = glyph_draw_dsc->glyph_data;
                img_dsc.recolor = glyph_draw_dsc->color;
                img_dsc.pivot = (lv_point_t){
                    .x = glyph_draw_dsc->pivot.x,
                    .y = glyph_draw_dsc->g ? (glyph_draw_dsc->g->box_h +
                                              glyph_draw_dsc->g->ofs_y)
                                           : 0};

                lv_draw_epic_img(draw_task, &img_dsc,
                                 glyph_draw_dsc->letter_coords);

                if (glyph_draw_dsc->_draw_buf)
                {
                    lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                    glyph_draw_dsc->_draw_buf = NULL;
                }
            }
            break;
        }
        break;

        default:
            LV_EPIC_LOG("WARNING: unsupported glyph format %d",
                        glyph_draw_dsc->format);
            break;
        }
    }

    if (fill_draw_dsc && fill_area)
    {
        label_draw_ctx_flush_pending_op();
        lv_draw_epic_fill(draw_task, fill_draw_dsc, fill_area);
    }
}

static void label_draw_ctx_reset(void)
{
    memset(&g_label_ctx, 0, sizeof(g_label_ctx));
}

static void label_draw_ctx_flush_pending_op(void)
{
    if (g_label_ctx.pending_op == NULL)
    {
        return;
    }

    rt_err_t err = drv_epic_commit_op(g_label_ctx.pending_op);
    if (err != RT_EOK)
    {
        LV_LOG_ERROR("EPIC: Failed to commit label operation, err=%d", err);
    }

    g_label_ctx.pending_op = NULL;
}

static drv_epic_operation *
label_draw_ctx_get_pending_op(const lv_draw_glyph_dsc_t *glyph_draw_dsc)
{
    if (glyph_draw_dsc == NULL || g_label_ctx.render_buf == NULL)
    {
        LV_LOG_ERROR("EPIC: Label context is not ready for letter drawing");
        return NULL;
    }

    if (g_label_ctx.pending_op != NULL &&
        g_label_ctx.opa == glyph_draw_dsc->opa &&
        g_label_ctx.color.red == glyph_draw_dsc->color.red &&
        g_label_ctx.color.green == glyph_draw_dsc->color.green &&
        g_label_ctx.color.blue == glyph_draw_dsc->color.blue)
    {
        return g_label_ctx.pending_op;
    }

    label_draw_ctx_flush_pending_op();

    drv_epic_operation *label_op = drv_epic_alloc_op(g_label_ctx.render_buf);
    if (label_op == NULL)
    {
        LV_LOG_ERROR("EPIC: Failed to allocate label operation");
        return NULL;
    }

    label_op->op = DRV_EPIC_DRAW_LETTERS;
    label_op->desc.label.color_mode = EPIC_INPUT_A8;
    label_op->desc.label.r = glyph_draw_dsc->color.red;
    label_op->desc.label.g = glyph_draw_dsc->color.green;
    label_op->desc.label.b = glyph_draw_dsc->color.blue;
    label_op->desc.label.opa = glyph_draw_dsc->opa;
    label_op->desc.label.letter_num = 0;
    label_op->clip_area.x0 = g_label_ctx.clip_area.x1;
    label_op->clip_area.y0 = g_label_ctx.clip_area.y1;
    label_op->clip_area.x1 = g_label_ctx.clip_area.x2;
    label_op->clip_area.y1 = g_label_ctx.clip_area.y2;

    g_label_ctx.pending_op = label_op;
    g_label_ctx.color = glyph_draw_dsc->color;
    g_label_ctx.opa = glyph_draw_dsc->opa;
    return label_op;
}

static uint8_t *copy_glyph_bitmap_tightly(const uint8_t *src, uint32_t width,
                                          uint32_t height, uint32_t stride)
{
    if (src == NULL || width == 0U || height == 0U || stride < width)
    {
        return NULL;
    }

    uint32_t data_size = width * height;
    uint8_t *dst = (uint8_t *)lv_malloc(data_size);
    if (dst == NULL)
    {
        return NULL;
    }

    if (stride == width)
    {
        memcpy(dst, src, data_size);
        return dst;
    }

    for (uint32_t row = 0; row < height; row++)
    {
        memcpy(dst + row * width, src + row * stride, width);
    }

    return dst;
}

#endif /*LV_USE_DRAW_EPIC*/
