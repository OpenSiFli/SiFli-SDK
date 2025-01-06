/**
  ******************************************************************************
  * @file   lv_ext_resource_manager.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef LVSF_FONT_H
#define LVSF_FONT_H

#include "lvgl.h"
#include "section.h"
#ifdef LV_USING_FREETYPE_ENGINE
    #include "lv_freetype.h"
#endif /* LV_USING_FREETYPE_ENGINE */

typedef enum
{
    FONT_INTERVAL_EQUAL_WIDTH = 0,
    FONT_INTERVAL_COMPACT_WIDTH,
} lvsf_font_interval_t;


//LVSF_FONT_SIZES only used in lvsf_xxx control.
typedef enum
{
    LVSF_FONT_SMALL = 0,
    LVSF_FONT_NORMAL,
    LVSF_FONT_SUBTITLE,
    LVSF_FONT_TITLE,
    LVSF_FONT_BIG,
    LVSF_FONT_HUGE,
    LVSF_FONT_SUPER,
} LVSF_FONT_SIZES;

//If the customer modifies the font size, the following mapping needs to be modified accordingly
#define lvsf_convert_font_size(size)                            \
  switch(size)                                                  \
  {                                                             \
    case LVSF_FONT_SMALL:     size = FONT_SMALL;      break;    \
    case LVSF_FONT_NORMAL:    size = FONT_NORMAL;     break;    \
    case LVSF_FONT_SUBTITLE:  size = FONT_SUBTITLE;   break;    \
    case LVSF_FONT_TITLE:     size = FONT_TITLE;      break;    \
    case LVSF_FONT_BIG:       size = FONT_BIGL;       break;    \
    case LVSF_FONT_HUGE:      size = FONT_HUGE;       break;    \
    case LVSF_FONT_SUPER:     size = FONT_SUPER;      break;    \
  }

void lv_ext_set_local_font_bitmap(lv_obj_t *obj, lv_color_t color, lv_font_t *font);

#define lv_ext_set_local_bitmap_font(obj, color, font) \
extern lv_font_t font;\
lv_ext_set_local_font_bitmap(obj, color, &font);

//if freetype is used, FONT_SIZES will be used to register in lvsf_ft_reg.h
typedef enum
{
#if LV_HOR_RES_MAX > 350
    FONT_SMALL      = 16,
    FONT_NORMAL     = 20,
    FONT_SUBTITLE   = 24,
    FONT_TITLE      = 28,
    FONT_BIGL       = 36,
    FONT_HUGE       = 56,
    FONT_SUPER      = 72,
#else
    FONT_SMALL      = 12,
    FONT_NORMAL     = 16,
    FONT_SUBTITLE   = 20,
    FONT_TITLE      = 24,
    FONT_BIGL       = 28,
    FONT_HUGE       = 56,
    FONT_SUPER      = 72,
#endif
} FONT_SIZES;

#ifndef LV_USING_FREETYPE_ENGINE

static inline const lv_font_t *LV_EXT_FONT_GET(uint8_t size)
{
    lvsf_convert_font_size(size);

    const lv_font_t *font;

    if (FONT_BIGL <= size)
    {
        font = lv_theme_get_font_bigl(NULL);
    }
    else if (FONT_TITLE <= size)
    {
        font = lv_theme_get_font_title(NULL);
    }
    else if (FONT_SUBTITLE <= size)
    {
        font = lv_theme_get_font_subtitle(NULL);
    }
    else if (FONT_NORMAL <= size)
    {
        font = lv_theme_get_font_normal(NULL);
    }
    else //if (FONT_SMALL == size)
    {
        font = lv_theme_get_font_small(NULL);
    }

    return font;
}
#else

#define FONT_SECTION_NAME app_font

void lvsf_font_inital(uint32_t cache_size, bool init);
void lvsf_font_deinit(void);
void lvsf_font_load(uint32_t cache_size);
void lvsf_font_unload(void);

extern uint16_t font_pixel_size[];
void lv_freetype_set_font_size(lv_font_t *font, uint16_t size);
lv_font_t *lvsf_traverse_font_from_size(void **font_node, uint16_t size);
lv_font_t *lvsf_get_font_from_size(uint16_t size);
void lvsf_reset_font_size_by_name(char *font_name, int *size);
lv_font_t *lvsf_get_font_by_name(char *font_name, int size);

static inline const lv_font_t *LV_EXT_FONT_GET(uint8_t size)
{
    lvsf_convert_font_size(size);
    lv_font_t *font = lvsf_get_font_from_size(size);
    return (const lv_font_t *) font;
}
#endif

uint16_t lvsf_get_size_from_font(lv_font_t *font);
void lv_ext_set_local_font(lv_obj_t *obj, uint16_t size, lv_color_t color);
void lv_ext_lable_set_fixed_font(lv_obj_t *obj, uint16_t size, lv_color_t color, const char *font_name);
void lv_ext_font_reset(void);
#define  lv_ext_get_font_size(size) size


#endif /* LV_EXT_RESOURCE_MANAGER_H_ */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
