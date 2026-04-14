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

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void _draw_epic_letter(lv_draw_task_t *draw_task, lv_draw_glyph_dsc_t *glyph_draw_dsc,
                              lv_draw_fill_dsc_t *fill_draw_dsc, const lv_area_t *fill_area);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *  GLOBAL VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/
//#define ENABLE_PRINT_LETTER_MAP
/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_draw_epic_label(lv_draw_task_t *draw_task, const lv_draw_label_dsc_t *dsc,
                        const lv_area_t *coords)
{
    if (dsc->opa <= LV_OPA_MIN) return;

    lv_draw_label_iterate_characters(draw_task, dsc, coords, _draw_epic_letter);

    drv_epic_cont_blend_reset();
}
#ifdef ENABLE_PRINT_LETTER_MAP
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
#endif /*ENABLE_PRINT_LETTER_MAP*/
/**********************
 *   STATIC FUNCTIONS
 **********************/static void _draw_epic_letter(lv_draw_task_t *draw_task, lv_draw_glyph_dsc_t *glyph_draw_dsc,
                              lv_draw_fill_dsc_t *fill_draw_dsc, const lv_area_t *fill_area)
{
    if (glyph_draw_dsc)
    {
        switch (glyph_draw_dsc->format)
        {
            case LV_FONT_GLYPH_FORMAT_NONE: {
#if LV_USE_FONT_PLACEHOLDER
                if (glyph_draw_dsc->bg_coords == NULL) break;
                lv_draw_border_dsc_t border_draw_dsc;
                lv_draw_border_dsc_init(&border_draw_dsc);
                border_draw_dsc.opa = glyph_draw_dsc->opa;
                border_draw_dsc.color = glyph_draw_dsc->color;
                border_draw_dsc.width = 1;
                lv_draw_epic_border(draw_task, &border_draw_dsc, glyph_draw_dsc->bg_coords);
#endif
            }
            break;

            case LV_FONT_GLYPH_FORMAT_A1:
            case LV_FONT_GLYPH_FORMAT_A2:
            case LV_FONT_GLYPH_FORMAT_A3:
            case LV_FONT_GLYPH_FORMAT_A4:
            case LV_FONT_GLYPH_FORMAT_A8:
            case LV_FONT_GLYPH_FORMAT_IMAGE: {
                if (glyph_draw_dsc->rotation % 3600 == 0 && glyph_draw_dsc->format != LV_FONT_GLYPH_FORMAT_IMAGE) {
                    /*Do not draw transparent things*/
                    if (glyph_draw_dsc->opa <= LV_OPA_MIN)
                        return;

                    if (glyph_draw_dsc->g == NULL || glyph_draw_dsc->letter_coords == NULL) {
                        LV_EPIC_LOG("ERROR: glyph meta or letter coords is NULL");
                        return;
                    }

                    uint32_t glyph_width = lv_area_get_width(glyph_draw_dsc->letter_coords);
                    uint32_t glyph_height = lv_area_get_height(glyph_draw_dsc->letter_coords);
                    if (glyph_width == 0 || glyph_height == 0) {
                        LV_EPIC_LOG("ERROR: glyph width/height is zero");
                        return;
                    }

                    lv_draw_buf_t *draw_buf = NULL;
                    const void *static_bitmap = NULL;
                    uint32_t mask_stride = 0;

                    if (lv_font_has_static_bitmap(glyph_draw_dsc->g->resolved_font) &&
                        glyph_draw_dsc->g->format == LV_FONT_GLYPH_FORMAT_A8) {
                        glyph_draw_dsc->g->req_raw_bitmap = 1;
                        static_bitmap = lv_font_get_glyph_static_bitmap(glyph_draw_dsc->g);
                        mask_stride = glyph_draw_dsc->g->stride;
                        if (static_bitmap == NULL) {
                            LV_EPIC_LOG("ERROR: failed to get static glyph bitmap");
                            return;
                        }
                        LV_EPIC_LOG("Use static glyph bitmap: addr=%p, stride=%u", static_bitmap, mask_stride);
                    }
                    else {
                        if (glyph_draw_dsc->_draw_buf == NULL) {
                            glyph_draw_dsc->_draw_buf = lv_draw_buf_create(glyph_width, glyph_height, LV_COLOR_FORMAT_A8, 0);
                        }
                        if (glyph_draw_dsc->_draw_buf == NULL) {
                            LV_EPIC_LOG("ERROR: failed to create draw buf");
                            return;
                        }
                        glyph_draw_dsc->glyph_data = lv_font_get_glyph_bitmap(glyph_draw_dsc->g, glyph_draw_dsc->_draw_buf);
                        if (glyph_draw_dsc->glyph_data == NULL) {
                            LV_EPIC_LOG("ERROR: failed to get glyph bitmap");
                            if (glyph_draw_dsc->_draw_buf) lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                            glyph_draw_dsc->_draw_buf = NULL;
                            return;
                        }
                        draw_buf = (lv_draw_buf_t *)glyph_draw_dsc->glyph_data;
                        mask_stride = draw_buf->header.stride;
                    }

                    EPIC_LayerConfigTypeDef input_layers[2];
                    EPIC_LayerConfigTypeDef output_canvas;
                    uint8_t input_layer_cnt = 2;

                    if (lv_epic_setup_bg_and_output_layer(&input_layers[0], &output_canvas, draw_task, glyph_draw_dsc->letter_coords)) {
                        if (glyph_draw_dsc->_draw_buf) {
                            lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                            glyph_draw_dsc->_draw_buf = NULL;
                        }
                        return;
                    }

                    HAL_EPIC_LayerConfigInit(&input_layers[1]);
                    input_layers[1].alpha = glyph_draw_dsc->opa;
                    input_layers[1].x_offset = glyph_draw_dsc->letter_coords->x1;
                    input_layers[1].y_offset = glyph_draw_dsc->letter_coords->y1;
                    input_layers[1].data = (uint8_t *)(static_bitmap ? static_bitmap : draw_buf->data);
                    input_layers[1].color_en = true;
                    input_layers[1].color_r = glyph_draw_dsc->color.red;
                    input_layers[1].color_g = glyph_draw_dsc->color.green;
                    input_layers[1].color_b = glyph_draw_dsc->color.blue;
                    input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;
                    input_layers[1].width = glyph_width;
                    input_layers[1].height = glyph_height;
                    input_layers[1].color_mode = EPIC_INPUT_A8;
                    input_layers[1].total_width = mask_stride;

                    LV_EPIC_LOG("glyph_draw_dsc->bitmap %p, color=%u, w=%u, h=%u, stride=%u",
                               input_layers[1].data, lv_color_to_u32(glyph_draw_dsc->color),
                               glyph_width, glyph_height, mask_stride);
                    lv_epic_print_area_info("bitmap", glyph_draw_dsc->letter_coords);

#ifdef ENABLE_PRINT_LETTER_MAP
                    print_letter_map('c', input_layers[1].data, glyph_width, glyph_height, FT_BPP);
#endif

                    int ret = drv_epic_cont_blend(input_layers, input_layer_cnt, &output_canvas);
                    LV_ASSERT(0 == ret);

                    if (glyph_draw_dsc->_draw_buf) {
                        lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                        glyph_draw_dsc->_draw_buf = NULL;
                    }
                }
                else {
                    glyph_draw_dsc->glyph_data = lv_font_get_glyph_bitmap(glyph_draw_dsc->g, glyph_draw_dsc->_draw_buf);
                    if (glyph_draw_dsc->glyph_data == NULL) {
                        LV_EPIC_LOG("ERROR: failed to get glyph bitmap for rotation/image");
                        if (glyph_draw_dsc->_draw_buf) {
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
                    img_dsc.pivot = (lv_point_t) {
                        .x = glyph_draw_dsc->pivot.x,
                        .y = glyph_draw_dsc->g ? (glyph_draw_dsc->g->box_h + glyph_draw_dsc->g->ofs_y) : 0
                    };

                    lv_draw_epic_img(draw_task, &img_dsc, glyph_draw_dsc->letter_coords);

                    if (glyph_draw_dsc->_draw_buf) {
                        lv_draw_buf_destroy(glyph_draw_dsc->_draw_buf);
                        glyph_draw_dsc->_draw_buf = NULL;
                    }
                }
                break;
            }
            break;

            default:
                LV_EPIC_LOG("WARNING: unsupported glyph format %d", glyph_draw_dsc->format);
                break;
        }
    }

    if (fill_draw_dsc && fill_area)
    {
        lv_draw_epic_fill(draw_task, fill_draw_dsc, fill_area);
    }
}

#endif /*LV_USE_DRAW_EPIC*/
