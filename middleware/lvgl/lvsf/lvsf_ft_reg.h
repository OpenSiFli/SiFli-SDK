/**
 * @file lv_font_reg.h
 *
 */
#ifndef _LVSF_FONT_REG_H
#define _LVSF_FONT_REG_H

#include "lvgl.h"
#ifdef LV_USING_FREETYPE_ENGINE

#include "lv_freetype.h"

#define LVSF_GET_FT(lib) (const void*)&(RES_PATH "font" "/" STRINGIFY(lib) TTF_SUFFIX)

typedef struct
{
    uint32_t        font_lib_size;      /* handle to face object */
    const char      *font_lib_data;     /* font height size */
    const char      *font_lib_name;     /* font name*/
} lv_font_freetype_lib_dsc_t;


typedef struct
{
    const char                  *font_name;
    lv_font_freetype_lib_dsc_t  *font_lib;
    uint16_t                     font_size;
} font_desc_t;

typedef struct
{
    font_desc_t                 *font_desc;
    lv_font_t                   *ft_font;
    lv_font_t                   *font;
    lv_freetype_font_fmt_dsc_t  *fmt_desc;
    rt_list_t                    list;
    rt_list_t                    name_list;
} font_node_t;

//Note: The font size must be arranged from small to large.
//Because font is searched from small to large
#ifndef USING_FREETYPE_ANY_SIZE
#define LVSF_FREETYPE_FONT_REGISTER(freetype_font)          \
    extern lv_font_freetype_lib_dsc_t CONCAT_2(freetype_font, _lib);    \
    LVSF_FONT_REGISTER(freetype_font, FONT_SMALL);          \
    LVSF_FONT_REGISTER(freetype_font, FONT_NORMAL);         \
    LVSF_FONT_REGISTER(freetype_font, FONT_SUBTITLE);       \
    LVSF_FONT_REGISTER(freetype_font, FONT_TITLE);          \
    LVSF_FONT_REGISTER(freetype_font, FONT_BIGL);           \
    LVSF_FONT_REGISTER(freetype_font, FONT_HUGE);           \
    LVSF_FONT_REGISTER(freetype_font, FONT_SUPER);
#else
#define LVSF_FREETYPE_FONT_REGISTER(freetype_font)          \
    extern lv_font_freetype_lib_dsc_t CONCAT_2(freetype_font, _lib); \
    LVSF_FONT_REGISTER(freetype_font, 12);                  \
    LVSF_FONT_REGISTER(freetype_font, 13);                  \
    LVSF_FONT_REGISTER(freetype_font, 14);                  \
    LVSF_FONT_REGISTER(freetype_font, 15);                  \
    LVSF_FONT_REGISTER(freetype_font, 16);                  \
    LVSF_FONT_REGISTER(freetype_font, 17);                  \
    LVSF_FONT_REGISTER(freetype_font, 18);                  \
    LVSF_FONT_REGISTER(freetype_font, 19);                  \
    LVSF_FONT_REGISTER(freetype_font, 20);                  \
    LVSF_FONT_REGISTER(freetype_font, 21);                  \
    LVSF_FONT_REGISTER(freetype_font, 22);                  \
    LVSF_FONT_REGISTER(freetype_font, 23);                  \
    LVSF_FONT_REGISTER(freetype_font, 24);                  \
    LVSF_FONT_REGISTER(freetype_font, 25);                  \
    LVSF_FONT_REGISTER(freetype_font, 26);                  \
    LVSF_FONT_REGISTER(freetype_font, 27);                  \
    LVSF_FONT_REGISTER(freetype_font, 28);                  \
    LVSF_FONT_REGISTER(freetype_font, 29);                  \
    LVSF_FONT_REGISTER(freetype_font, 30);                  \
    LVSF_FONT_REGISTER(freetype_font, 31);                  \
    LVSF_FONT_REGISTER(freetype_font, 32);                  \
    LVSF_FONT_REGISTER(freetype_font, 33);                  \
    LVSF_FONT_REGISTER(freetype_font, 34);                  \
    LVSF_FONT_REGISTER(freetype_font, 35);                  \
    LVSF_FONT_REGISTER(freetype_font, 36);                  \
    LVSF_FONT_REGISTER(freetype_font, FONT_HUGE);           \
    LVSF_FONT_REGISTER(freetype_font, FONT_SUPER);
#endif

#define LVSF_FONT_REGISTER(name, size) \
        SECTION_ITEM_REGISTER(FONT_SECTION_NAME, static const font_desc_t CONCAT_2(CONCAT_2(CONCAT_2(name, _), size), _var)) = \
        { \
        .font_name  = #name, \
        .font_lib   = &CONCAT_2(name, _lib), \
        .font_size  = size, \
        }

#endif
#endif
