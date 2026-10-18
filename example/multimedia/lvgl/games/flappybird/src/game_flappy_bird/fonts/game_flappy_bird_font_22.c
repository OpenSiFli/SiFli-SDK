/*******************************************************************************
 * Size: 22 px
 * Bpp: 1
 * Opts: --bpp 1 --size 22 --no-compress --font 包图小白体.ttf --symbols 0123456789SCOREBTA --format lvgl -o game_flappy_bird_font_22.c
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifndef GAME_FLAPPY_BIRD_FONT_22
#define GAME_FLAPPY_BIRD_FONT_22 1
#endif

#if GAME_FLAPPY_BIRD_FONT_22

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0030 "0" */
    0xff, 0xe4, 0x19, 0xe6, 0x49, 0x92, 0x64, 0x99,
    0x26, 0x49, 0x92, 0x64, 0x99, 0x26, 0x79, 0x82,
    0xdf, 0xf0,

    /* U+0031 "1" */
    0x69, 0x99, 0x99, 0x99, 0x99, 0x99, 0x97,

    /* U+0032 "2" */
    0x7f, 0x40, 0x62, 0x2e, 0x90, 0x4b, 0xe6, 0x13,
    0x9, 0x9f, 0xc8, 0x24, 0x13, 0xe8, 0xb, 0xfc,
    0xf0,

    /* U+0033 "3" */
    0xff, 0x81, 0x79, 0x9, 0x9, 0xf9, 0x89, 0x79,
    0x9, 0x9, 0x79, 0x89, 0x93, 0x7f,

    /* U+0034 "4" */
    0xee, 0x26, 0x49, 0x92, 0x64, 0x99, 0x26, 0x49,
    0x8a, 0x2, 0x61, 0x86, 0x40, 0x90, 0x24, 0x9,
    0x1, 0xc0,

    /* U+0035 "5" */
    0x7f, 0xd0, 0x67, 0xf2, 0x9, 0x4, 0xfa, 0x3,
    0x1, 0x7c, 0x82, 0x41, 0x2f, 0x98, 0xf, 0xfc,

    /* U+0036 "6" */
    0xff, 0xc0, 0x67, 0xf2, 0x9, 0x5, 0x7e, 0x83,
    0x79, 0xa4, 0xda, 0x6f, 0x30, 0x98, 0x4b, 0xfc,

    /* U+0037 "7" */
    0x7f, 0x40, 0x5f, 0x20, 0x90, 0x88, 0x44, 0x22,
    0x13, 0x9, 0x84, 0xc2, 0x61, 0x20, 0x90, 0x38,

    /* U+0038 "8" */
    0xff, 0x48, 0x67, 0xb2, 0x59, 0x2c, 0xf6, 0x3,
    0xd, 0x9e, 0xc9, 0x64, 0xb3, 0xd8, 0x4b, 0xfc,

    /* U+0039 "9" */
    0xff, 0x58, 0x67, 0x32, 0x99, 0x4c, 0xe6, 0x13,
    0x9, 0x7c, 0x82, 0x41, 0x3f, 0x18, 0xf, 0xfc,

    /* U+0041 "A" */
    0xf, 0x0, 0x98, 0x10, 0x81, 0x48, 0x16, 0xc2,
    0x64, 0x22, 0x42, 0x16, 0x4f, 0x24, 0x92, 0x49,
    0x39, 0x89, 0x90, 0x97, 0x7,

    /* U+0042 "B" */
    0x7f, 0xa4, 0x29, 0xca, 0x72, 0x9c, 0xa8, 0x1a,
    0x6, 0x79, 0x9a, 0x66, 0x99, 0xa6, 0x19, 0x82,
    0x5f, 0xf0,

    /* U+0043 "C" */
    0x7f, 0xe8, 0x19, 0xfe, 0x40, 0x90, 0x24, 0x9,
    0x2, 0x40, 0x90, 0x24, 0x9, 0xee, 0x1, 0x41,
    0xcf, 0xf0,

    /* U+0045 "E" */
    0xff, 0x40, 0x6f, 0xf6, 0xb, 0x5, 0x7a, 0x85,
    0x7e, 0xa0, 0x50, 0x29, 0xd3, 0x18, 0xb, 0xfc,

    /* U+004F "O" */
    0xff, 0xe4, 0x19, 0xe6, 0x49, 0x92, 0x64, 0x99,
    0x26, 0x49, 0x92, 0x64, 0x99, 0x26, 0x39, 0x82,
    0xdf, 0xf0,

    /* U+0052 "R" */
    0x7f, 0xa8, 0x19, 0xe6, 0x49, 0x92, 0x63, 0x98,
    0x26, 0x7f, 0x92, 0x26, 0x49, 0x8a, 0x51, 0x92,
    0xdc, 0x60,

    /* U+0053 "S" */
    0x7f, 0xc0, 0x67, 0xf2, 0x9, 0x4, 0x7a, 0x2,
    0xf9, 0x4, 0x82, 0x5f, 0x30, 0x18, 0x1f, 0xfc,

    /* U+0054 "T" */
    0xff, 0xf0, 0x7, 0xff, 0xb9, 0x1, 0x30, 0x26,
    0x4, 0xc0, 0x98, 0x13, 0x2, 0x60, 0x4c, 0x9,
    0x81, 0x30, 0x1c, 0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 183, .box_w = 10, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 18, .adv_w = 91, .box_w = 4, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 25, .adv_w = 166, .box_w = 9, .box_h = 15, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 42, .adv_w = 164, .box_w = 8, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 56, .adv_w = 180, .box_w = 10, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 74, .adv_w = 168, .box_w = 9, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 90, .adv_w = 168, .box_w = 9, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 106, .adv_w = 162, .box_w = 9, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 122, .adv_w = 176, .box_w = 9, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 138, .adv_w = 168, .box_w = 9, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 154, .adv_w = 202, .box_w = 12, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 175, .adv_w = 169, .box_w = 10, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 193, .adv_w = 168, .box_w = 10, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 211, .adv_w = 155, .box_w = 9, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 227, .adv_w = 172, .box_w = 10, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 245, .adv_w = 165, .box_w = 10, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 263, .adv_w = 153, .box_w = 9, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 279, .adv_w = 177, .box_w = 11, .box_h = 14, .ofs_x = 0, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_1[] = {
    0x0, 0x1, 0x2, 0x4, 0xe, 0x11, 0x12, 0x13
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 48, .range_length = 10, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    },
    {
        .range_start = 65, .range_length = 20, .glyph_id_start = 11,
        .unicode_list = unicode_list_1, .glyph_id_ofs_list = NULL, .list_length = 8, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 2,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t game_flappy_bird_font_22 = {
#else
lv_font_t game_flappy_bird_font_22 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 15,          /*The maximum line height required by the font*/
    .base_line = 1,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -4,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if GAME_FLAPPY_BIRD_FONT_22*/

