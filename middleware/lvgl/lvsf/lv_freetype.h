/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lv_freetype.h
 *
 */
#ifndef _LV_FREETYPE_H
#define _LV_FREETYPE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "lvgl.h"

#ifndef LV_FREETYPE_CACHE_FT_FACES
    #ifdef CONFIG_LV_FREETYPE_CACHE_FT_FACES
        #define LV_FREETYPE_CACHE_FT_FACES CONFIG_LV_FREETYPE_CACHE_FT_FACES
    #else
        #define LV_FREETYPE_CACHE_FT_FACES 0
    #endif
#endif

#ifndef LV_FREETYPE_CACHE_FT_SIZES
    #ifdef CONFIG_LV_FREETYPE_CACHE_FT_SIZES
        #define LV_FREETYPE_CACHE_FT_SIZES CONFIG_LV_FREETYPE_CACHE_FT_SIZES
    #else
        #define LV_FREETYPE_CACHE_FT_SIZES 0
    #endif
#endif

#if defined (LV_USING_FREETYPE_ENGINE) && !defined(PKG_SCHRIFT)

#include "ft2build.h"
#include FT_FREETYPE_H
#include FT_GLYPH_H
#include FT_CACHE_H

/*********************
 *      DEFINES
 *********************/

/**
 * the following setting can't change, because lib use these config
 */

#define USE_CACHE_MANGER    1

#ifndef FT_CACHE_SIZE
#define FT_CACHE_SIZE (80 * 1000)
#endif


//#define FREETYPE_EXTERN_CACHE_AGAIN 1

typedef enum
{
    EXTERN_CACHE_NONE,
    EXTERN_CACHE_AGINE
} lv_freetype_extern_cache_t;

/**
 * the above setting can't change, because lib use these config
 */

/**********************
 *      TYPEDEFS
 **********************/

typedef struct
{
    FT_Face         face;      /* handle to face object */
    uint16_t        font_size;     /*font height size */
    const char      *font_lib_addr;
    int             font_lib_size;
    void            *buf;
    void            *face_source;  /* shared per-font-file FTC face id (see lv_freetype.c) */
} lv_freetype_font_fmt_dsc_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
int lv_freetype_init(uint16_t max_faces, uint16_t max_sizes, uint32_t max_cache_size);
int lv_freetype_font_init(lv_font_t *font, const char *font_lib_addr, int font_lib_size, uint16_t font_size, const char *font_name);
void lv_freetype_font_deinit(lv_font_t *font);
void lv_freetype_close_font(void);
void lv_freetype_open_font(bool init);

/* Look up an FT_Size through the cache. Opens the face first: FTC recycles a
 * size node by freeing its FT_Size before opening the face of the new scaler,
 * and a face that fails to open there takes the half-freed node down with it.
 * Use this instead of FTC_Manager_LookupSize. */
FT_Error lv_freetype_lookup_size(FTC_Scaler scaler, FT_Size *asize);

typedef enum
{
    FT_CACHE_QUAD_CLEAN,
    FT_CACHE_HALF_CLEAN,
    FT_CACHE_WHOLE_CLEAN
} lv_freetype_cache_clean_t;

void lv_freetype_clean_cache(uint8_t clean_type);
void lv_freetype_set_parameter(uint16_t bpp, uint16_t cache_max_font_size, lv_freetype_extern_cache_t extern_cache);

/**********************
 *      MACROS
 **********************/
#elif defined (LV_USING_FREETYPE_ENGINE) && defined(PKG_SCHRIFT)
#include "lv_schrift.h"
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
