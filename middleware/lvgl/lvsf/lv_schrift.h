/**
 * @file lv_freetype.h
 *
 */
#ifndef _LV_SCHRIFT_H
#define _LV_SCHRIFT_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "lvgl.h"

#if defined (LV_USING_FREETYPE_ENGINE) && defined(PKG_SCHRIFT)

/**
 * the following setting can't change, because lib use these config
 */

#define FT_EXTERN_CACHE

#ifndef FT_CACHE_SIZE
#define FT_CACHE_SIZE (40 * 1000)
#endif


#if defined (PKG_SCHRIFT) && !defined (FREETYPE_NORMAL_FONT)
#error "schrift is used to replace freetype engine, must use normal font for sckrift now, menuconfig-->Select freetype normal ttf"
#endif

/**
 * the above setting can't change, because lib use these config
 */

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
#if defined (PKG_SCHRIFT) || defined (FT_EXTERN_CACHE)
struct SFT_Font
{
    const uint8_t *memory;
    uint_fast32_t  size;
    int            source;
    uint_least16_t unitsPerEm;
    int_least16_t  locaFormat;
    uint_least16_t numLongHmtx;
};

typedef struct SFT_Font     SFT_Font;

struct SFT
{
    SFT_Font *font;
    float    xScale;
    float    yScale;
    float    xOffset;
    float    yOffset;
    int      flags;
};

typedef struct SFT          SFT;

typedef struct
{
    SFT             sft;  //non freetype engine used
    uint8_t         *buf; //handle to image(face object)
    uint16_t        font_size;     /*font height size */
} lv_freetype_font_fmt_dsc_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
int lv_freetype_init(uint8_t max_faces, uint32_t max_cache_size);
int lv_freetype_font_init(lv_font_t *font, const char *font_lib_addr, int font_lib_size, uint16_t font_size);
void lv_freetype_close_font(void);
void lv_freetype_open_font(bool init);

typedef enum
{
    FT_CACHE_QUAD_CLEAN,
    FT_CACHE_HALF_CLEAN,
    FT_CACHE_WHOLE_CLEAN
} ft_cache_clean_t;

void lv_freetype_clean_cache(uint8_t clean_type);

#endif

#ifdef __cplusplus
} /* extern "C" */

#endif
#endif

#endif
