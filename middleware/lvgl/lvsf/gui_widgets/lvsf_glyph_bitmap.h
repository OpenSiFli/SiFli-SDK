/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef LVSF_GLYPH_BITMAP_H
#define LVSF_GLYPH_BITMAP_H

#include "lvgl.h"

const uint8_t *lvsf_glyph_get_epic_bitmap(const lv_font_t *font, uint32_t letter);

/* Decode a span within the visible glyph bounds into one opacity byte per pixel. */
void lvsf_glyph_bitmap_read_a8(const lv_font_glyph_dsc_t *glyph, const uint8_t *bitmap,
                              uint32_t x, uint32_t y,
                              uint8_t *out, uint32_t count);

#endif
