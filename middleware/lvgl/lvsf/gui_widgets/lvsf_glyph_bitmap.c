/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvsf_glyph_bitmap.h"

#include <string.h>

/* Match the FreeType bitmap layout selected by the consuming firmware. */
#if COMPATIBLE_WITH_SIFLI_EPIC_Ax && defined(LV_USING_FREETYPE_ENGINE)
    #define IS_BIG_ALIGN 1
#else
    #define IS_BIG_ALIGN 0
#endif

uint32_t lv_txtanim_draw_flag = ((LV_COLOR_DEPTH == 24) << 1) | IS_BIG_ALIGN;

const uint8_t *lvsf_glyph_get_epic_bitmap(const lv_font_t *font, uint32_t letter)
{
    return lv_font_get_bitmap_fmt_txt(font, letter);
}

void lvsf_glyph_bitmap_read_a8(const lv_font_glyph_dsc_t *glyph, const uint8_t *bitmap,
                              uint32_t x, uint32_t y,
                              uint8_t *out, uint32_t count)
{
    uint32_t bpp = glyph->bpp;
    if (bpp == 3) bpp = 4; /* LVGL v8 stores 3bpp glyphs in nibbles. */
    if (bpp == 8)
    {
        memcpy(out, bitmap + y * glyph->box_w + x, count);
        return;
    }
    if (bpp != 1 && bpp != 2 && bpp != 4)
    {
        memset(out, 0, count);
        return;
    }

    uint32_t row_bits = glyph->box_w * bpp;
    if (IS_BIG_ALIGN) row_bits = (row_bits + 7) & ~7U;
    uint32_t bit = y * row_bits + x * bpp;
    uint32_t mask = (1U << bpp) - 1;

    for (uint32_t i = 0; i < count; i++, bit += bpp)
    {
        uint32_t shift = bit & 7;
        if (!IS_BIG_ALIGN) shift = 8 - bpp - shift;
        uint32_t pixel = (bitmap[bit >> 3] >> shift) & mask;
        out[i] = pixel ? (pixel << (8 - bpp)) + mask : 0;
    }
}
