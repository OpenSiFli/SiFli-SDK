/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emoji font for notify_msgs demo.
 *
 * A single fixed-size image is kept for every emoji. The source PNGs carry a
 * 4px transparent border (80x80 px with the original 72x72 glyph centered),
 * so EPIC downscales drop transparent pixels instead of clipping the glyph
 * edge. When the line height of the font that needs the emoji differs from
 * the scaled image size, the image is further scaled by LVGL/EPIC instead of
 * shipping a pre-scaled copy per size.
 *
 * For each font size used by notify_msgs a small custom lv_font_t is created
 * and attached as fallback of the shared freetype font of the same size, so
 * emoji unicodes resolve to the emoji images while other characters still come
 * from the freetype font. Image drawing inside draw_letter is accelerated by
 * EPIC (see lv_gpu_new_api.c).
 *
 * NOTE: unlike LVGL's built-in lv_imgfont, whose glyph box equals the native
 * image size, this font reports the metrics of the *scaled* glyph box so the
 * emoji fills the ascent space (line_height - base_line) of the surrounding
 * text. Such a scaled imgfont
 * is only rendered correctly by the EPIC draw_letter path in lv_gpu_new_api.c,
 * which is compiled when CONFIG_DRV_EPIC_NEW_API is enabled. Builds that fall
 * back to the software letter renderer (lv_gpu.c, e.g. the sf32lb55x and
 * PC-simulator targets) keep lv_draw_sw_letter's "box_w/box_h == image size"
 * assumption and would draw the emoji incorrectly, so emoji must not be
 * enabled on those targets.
 */

#include "app_clock_emoji.h"
#include "lvsf_font.h"

#if LV_USE_IMGFONT

/* Font sizes must match the fonts created by lvsf_get_font_from_size(). */
static const uint16_t emoji_font_sizes[] =
{
    FONT_SMALL,     /* 16 */
    FONT_NORMAL,    /* 20 */
    FONT_SUBTITLE,  /* 24 */
    FONT_BIGL,      /* 36 */
    FONT_HUGE,      /* 56 */
    FONT_SUPER,     /* 72 */
};
#define EMOJI_SIZE_CNT  (sizeof(emoji_font_sizes) / sizeof(emoji_font_sizes[0]))

/* Fixed-size emoji images, generated from twemoji PNGs by the ezip tool. */
LV_IMG_DECLARE(emoji_u1f600_72);
LV_IMG_DECLARE(emoji_u1f602_72);
LV_IMG_DECLARE(emoji_u1f60d_72);
LV_IMG_DECLARE(emoji_u1f44d_72);
LV_IMG_DECLARE(emoji_u1f525_72);
LV_IMG_DECLARE(emoji_u1f496_72);

typedef struct
{
    uint32_t unicode;               /* emoji unicode codepoint */
    const lv_img_dsc_t *img;        /* single fixed-size image for this emoji */
} emoji_glyph_t;

static const emoji_glyph_t emoji_glyphs[] =
{
    {0x1F600, &emoji_u1f600_72}, /* grinning face */
    {0x1F602, &emoji_u1f602_72}, /* face with tears of joy */
    {0x1F60D, &emoji_u1f60d_72}, /* smiling face with heart-eyes */
    {0x1F44D, &emoji_u1f44d_72}, /* thumbs up */
    {0x1F525, &emoji_u1f525_72}, /* fire */
    {0x1F496, &emoji_u1f496_72}, /* sparkling heart */
};
#define EMOJI_GLYPH_CNT (sizeof(emoji_glyphs) / sizeof(emoji_glyphs[0]))

/* Custom font handle per font size slot, created once. The freetype base font
 * each one is attached to mirrors its line metrics onto this font in
 * app_clock_emoji_font_init(): line_height and base_line are copied from the
 * base font, so the ascent space (line_height - base_line) used by
 * get_glyph_dsc() can be derived from this font alone without a separate
 * base-font lookup that could silently miss. */
static lv_font_t *emoji_fonts[EMOJI_SIZE_CNT];

static const emoji_glyph_t *emoji_find(uint32_t unicode)
{
    for (uint32_t i = 0; i < EMOJI_GLYPH_CNT; i++)
    {
        if (emoji_glyphs[i].unicode == unicode) return &emoji_glyphs[i];
    }
    return NULL;
}

static bool emoji_font_get_glyph_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *dsc_out,
                                     uint32_t unicode, uint32_t unicode_next)
{
    LV_UNUSED(unicode_next);

    const emoji_glyph_t *glyph = emoji_find(unicode);
    if (NULL == glyph) return false;

    /* Report the metrics of the *scaled* glyph box. The renderers position the
     * glyph box bottom-aligned at line_top + (dsc->font->line_height -
     * dsc->font->base_line), where dsc->font is the label's *base* freetype
     * font (a fallback resolved glyph does not change dsc->font). So the box
     * height must equal the base font's ascent space line_height - base_line:
     * a full-line-height box would be pushed base_line pixels above the line
     * top where it gets clipped. Because init() mirrors the base metrics onto
     * this font, that ascent is simply this font's line_height - base_line.
     * Reporting the scaled box instead of the native 72x72 image size deviates
     * from the built-in lv_imgfont convention (box_w/box_h == image size) and
     * is only valid for the EPIC draw path; see the file header. */
    lv_coord_t font_h = (font->line_height > font->base_line)
                        ? (font->line_height - font->base_line)
                        : font->line_height;
    lv_coord_t img_h = (lv_coord_t)glyph->img->header.h;
    lv_coord_t img_w = (lv_coord_t)glyph->img->header.w;
    lv_coord_t box_w = (img_h > 0) ? (lv_coord_t)(((int32_t)img_w * font_h + img_h / 2) / img_h) : font_h;
    if (box_w <= 0) box_w = font_h;

    dsc_out->is_placeholder = 0;
    dsc_out->adv_w = (uint16_t)box_w;
    dsc_out->box_w = box_w;
    dsc_out->box_h = font_h;
    dsc_out->ofs_x = 0;
    dsc_out->ofs_y = 0;
    dsc_out->bpp   = LV_IMGFONT_BPP;    /* is image identifier */

    return true;
}

static const uint8_t *emoji_font_get_glyph_bitmap(const lv_font_t *font, uint32_t unicode)
{
    LV_UNUSED(font);

    const emoji_glyph_t *glyph = emoji_find(unicode);
    if (NULL == glyph) return NULL;

    /* Return the stable per-emoji descriptor: unlike the built-in imgfont this
     * never aliases different glyphs onto one shared path buffer. */
    return (const uint8_t *)glyph->img;
}

static lv_font_t *emoji_font_create(uint16_t height)
{
    lv_font_t *font = lv_mem_alloc(sizeof(lv_font_t));
    if (NULL == font) return NULL;

    lv_memset_00(font, sizeof(lv_font_t));
    font->get_glyph_dsc = emoji_font_get_glyph_dsc;
    font->get_glyph_bitmap = emoji_font_get_glyph_bitmap;
    font->line_height = height;
    font->base_line = 0;
    font->subpx = LV_FONT_SUBPX_NONE;
    return font;
}

void app_clock_emoji_font_init(void)
{
    for (uint32_t i = 0; i < EMOJI_SIZE_CNT; i++)
    {
        uint16_t size = emoji_font_sizes[i];

        if (NULL == emoji_fonts[i])
            emoji_fonts[i] = emoji_font_create(size);
        if (NULL == emoji_fonts[i]) continue;

        lv_font_t *base = lvsf_get_font_from_size(size);
        if (base)
        {
            /* The nominal size differs from the real freetype line metrics by
             * ~15-20%; mirror both base metrics onto the emoji font so that
             * get_glyph_dsc() can derive the ascent space (line_height -
             * base_line) directly from this font. */
            emoji_fonts[i]->line_height = base->line_height;
            emoji_fonts[i]->base_line = base->base_line;
            if (NULL == base->fallback)
                base->fallback = emoji_fonts[i];
        }
    }
}

#else

void app_clock_emoji_font_init(void)
{
}

#endif /* LV_USE_IMGFONT */
