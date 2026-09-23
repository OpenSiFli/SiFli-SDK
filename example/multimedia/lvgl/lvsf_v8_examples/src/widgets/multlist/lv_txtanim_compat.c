/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl.h"

static lv_opa_t lv_txtanim_px_to_opa(uint8_t letter_px, uint32_t bpp)
{
    switch(bpp)
    {
    case 1:
        return letter_px ? LV_OPA_COVER : LV_OPA_TRANSP;
    case 2:
        return (lv_opa_t)(letter_px * 85U);
    case 4:
        return (lv_opa_t)(letter_px * 17U);
    case 8:
        return (lv_opa_t)letter_px;
    default:
        return LV_OPA_TRANSP;
    }
}

void lv_txtanim_draw_txt(lv_img_dsc_t *dsc,
                         lv_coord_t x,
                         lv_coord_t y,
                         lv_font_glyph_dsc_t *glyph,
                         const uint8_t *src,
                         lv_color_t color,
                         uint8_t is_hollow)
{
    LV_UNUSED(is_hollow);

    if(dsc == NULL || glyph == NULL || src == NULL) return;

    uint32_t bpp = glyph->bpp;
    if(bpp == 3) bpp = 4;
    if(bpp == LV_IMGFONT_BPP) return;

    uint32_t bitmask_init = 0;
    switch(bpp)
    {
    case 1:
        bitmask_init = 0x80;
        break;
    case 2:
        bitmask_init = 0xC0;
        break;
    case 4:
        bitmask_init = 0xF0;
        break;
    case 8:
        bitmask_init = 0xFF;
        break;
    default:
        return;
    }

    lv_coord_t box_w = glyph->box_w;
    lv_coord_t box_h = glyph->box_h;
    uint32_t width_bit = (uint32_t)box_w * bpp;
    uint32_t col_bit_max = 8U - bpp;

    for(lv_coord_t row = 0; row < box_h; row++)
    {
        const uint8_t *map_p = src + (((uint32_t)row * width_bit) >> 3);
        uint32_t col_bit = 0;
        uint32_t bitmask = bitmask_init;

        for(lv_coord_t col = 0; col < box_w; col++)
        {
            uint8_t letter_px = (uint8_t)((*map_p & bitmask) >> (col_bit_max - col_bit));

            if(letter_px != 0)
            {
                lv_coord_t draw_x = x + col;
                lv_coord_t draw_y = y + row;

                if(draw_x >= 0 && draw_y >= 0 &&
                   draw_x < (lv_coord_t)dsc->header.w &&
                   draw_y < (lv_coord_t)dsc->header.h)
                {
                    lv_opa_t opa = lv_txtanim_px_to_opa(letter_px, bpp);

                    if(dsc->header.cf == LV_IMG_CF_TRUE_COLOR_ALPHA ||
                       dsc->header.cf == LV_IMG_CF_TRUE_COLOR)
                    {
                        lv_img_buf_set_px_color(dsc, draw_x, draw_y, color);
                    }

                    lv_img_buf_set_px_alpha(dsc, draw_x, draw_y, opa);
                }
            }

            if(col_bit < col_bit_max)
            {
                col_bit += bpp;
                bitmask >>= bpp;
            }
            else
            {
                col_bit = 0;
                bitmask = bitmask_init;
                map_p++;
            }
        }
    }
}
