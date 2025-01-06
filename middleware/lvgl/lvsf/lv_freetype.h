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
    void            *buf;
} lv_freetype_font_fmt_dsc_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
int lv_freetype_init(uint8_t max_faces, uint32_t max_cache_size);
int lv_freetype_font_init(lv_font_t *font, const char *font_lib_addr, int font_lib_size, uint16_t font_size, const char *font_name);
void lv_freetype_close_font(void);
void lv_freetype_open_font(bool init);

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
