/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file app_mem.h
 *
 */

#ifndef APP_MEM_H
#define APP_MEM_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <rtthread.h>
#include "lvgl.h"
#ifdef DISABLE_LVGL_V8
#include "src/misc/cache/instance/lv_image_cache.h"
#endif
//#include "lv_img_buf.h"
#include "mem_section.h"

/*********************
 *      DEFINES
 *********************/
typedef enum
{
    IMAGE_CACHE_HEAP,
    IMAGE_CACHE_SRAM,
    IMAGE_CACHE_PSRAM,
    IMAGE_CACHE_MSG,                      /**< Apply memory block in msg heap.              */
} image_cache_t;

#ifndef CACHE_HEAP
#define CACHE_HEAP IMAGE_CACHE_HEAP
#endif

#ifndef CACHE_SRAM
#define CACHE_SRAM IMAGE_CACHE_SRAM
#endif

#ifndef CACHE_PSRAM
#define CACHE_PSRAM IMAGE_CACHE_PSRAM
#endif

#ifndef CACHE_MSG
#define CACHE_MSG IMAGE_CACHE_MSG
#endif

typedef enum
{
    FIX_MODE_NONE,
    FIX_MODE_REUSE_ANIM_BUF,
} svg_mem_mode_t;


/**
@brief apply cache mem for solution applicaiton.
@param[in] size Size of cache mem
@param[in] cache_type Cache type of cache mem be applied
@retval Pointer of the successsful applicaiton.
*/
void *app_cache_alloc(size_t size, image_cache_t cache_type);

/**
@brief re-apply cache mem and preserve the original memory domain.
@param[in] Original memory
@param[in] new_size New size of cache mem
@retval Pointer of the successsful applicaiton.
*/
void *app_cache_realloc(void *memory, size_t new_size);

/**
@brief re-apply cache mem using an explicit memory domain.
@param[in] memory Original memory
@param[in] new_size New size of cache mem
@param[in] cache_type Cache type used when a new block is required
@retval Pointer of the successful allocation.
*/
void *app_cache_realloc_ex(void *memory, size_t new_size, image_cache_t cache_type);

/**
@brief free cache mem which successsful apply by app_cache_alloc.
@param[in] p Pointer of free mem
*/
void app_cache_free(void *p);

/**
@brief allocate zeroed cache mem.
@param[in] nmemb Number of objects
@param[in] size Size of each object
@param[in] cache_type Cache type
@retval Pointer to allocated zeroed memory, or NULL on failure.
*/
void *app_cache_calloc(size_t nmemb, size_t size, image_cache_t cache_type);

void *app_malloc(uint32_t size);

void *app_calloc(uint32_t count, uint32_t size);

char *app_strdup(const char *src);

void *app_realloc(void *memory, uint32_t new_size);

void app_free(void *p);


/**
@brief apply cache mem for message(including short message and notificaiton).
@param[in] size Size of cache mem
@retval Pointer of the successsful applicaiton.
*/
void *app_message_alloc(size_t size);

/**
@brief free cache mem which successsful apply by app_message_alloc.
@param[in] p Pointer of free mem
*/
void app_message_free(void *p);

/**
@brief apply cache mem for image(esp for canvas).
@param[in] w Width of image
@param[in] h Heigh of image
@param[in] cf Color fomat of image
@param[in] data_size Data size to be applied
@param[in] cache_type Cache type of cache mem be applied
@retval LV Image Pointer of the successsful applicaiton.
*/
lv_img_dsc_t *app_cache_img_alloc(lv_coord_t w, lv_coord_t h, lv_img_cf_t cf, uint32_t data_size, image_cache_t cache_type);

/**
@brief free image cache mem.
@param[in] dsc LV Image Pointer to be free, which successsful apply by app_cache_img_alloc
*/
void app_cache_img_free(lv_img_dsc_t *dsc);

/**
@brief apply cache mem of image copy(esp for rotating image).
@param[in] copy The origin image
@param[in] cache_type Cache type of cache mem be applied
@retval LV Image Pointer of the successsful applicaiton. (origin image will copy to this poionter)
*/
lv_img_dsc_t *app_cache_copy_alloc(const void *copy, image_cache_t cache_type);

/**
@brief free image copy cache mem.
@param[in] rel_mem LV Image Pointer to be free, which successsful apply by app_cache_copy_alloc
*/
void app_cache_copy_free(lv_img_dsc_t *rel_mem);

/**
@brief apply cache mem for animation playing(esp for gif).
@param[in] size Size of cache mem
@param[in] anim_data Application type: 0 - anim control; 1 - anim data
@retval Pointer of the successsful applicaiton.
*/
void *app_anim_mem_alloc(rt_size_t size, bool anim_data);

/**
@brief realloc cache mem for animation playing(esp for gif).
@param[in] p animation pointer to be reallocated, also, p must allocated by app_anim_alloc
@param[in] new_size Size of cache mem
@retval Pointer of the successsful applicaiton.
*/
void *app_anim_mem_realloc(void *p, size_t new_size);

/**
@brief free anim mem which successsful apply by app_anim_mem_alloc or app_anim_mem_realloc.
@param[in] p Pointer of free mem
*/
void app_anim_mem_free(void *p);

/**
@brief temporarily reuse transition animation buffers as large memheaps.
@param[in] enable 1: enable reuse; 0: disable reuse
@retval 0 when success, negative value when no reusable buffer is available or
        the animation heap is still busy.
*/
int app_anim_buf_set_as_memheap(uint8_t enable);

/**
@brief allocate temporary memory from reusable animation memheap first, then
       fallback to the normal PSRAM cache path.
*/
void *app_anim_alloc(size_t size);

/**
@brief reallocate temporary memory allocated by app_anim_alloc.
*/
void *app_anim_realloc(void *ptr, size_t size);

/**
@brief free memory allocated by app_anim_alloc/app_anim_realloc.
*/
void app_anim_free(void *ptr);

/**
@brief duplicate string with app_anim_alloc.
*/
char *app_anim_strdup(const char *s);

/**
@brief query allocation size for memory allocated by app_cache_xxx/app_anim_xxx.
*/
uint32_t app_mem_get_size(void *ptr);

/**
@brief initialize app memory heaps. Called by SOLUTION board startup path.
*/
int app_memheap_init(void);

/**
@brief get the total size of app memheap (PSRAM heap).
*/
uint32_t app_memheap_get_size(void);

/**
@brief check app memory status. Reserved for SOLUTION finsh hook.
*/
void app_mem_check(void);

/**
@brief allocate memory from SRAM cache heap.
*/
void *app_sram_alloc(rt_size_t size);

/**
@brief allocate zeroed memory from SRAM cache heap.
*/
void *app_sram_calloc(rt_size_t count, rt_size_t size);

/**
@brief reallocate memory from SRAM cache heap.
*/
void *app_sram_realloc(void *ptr, rt_size_t newsize);

/**
@brief free memory allocated by app_sram_alloc/calloc/realloc.
*/
void app_sram_free(void *ptr);

void *app_anim_calloc(uint32_t count, uint32_t size);

/**
@brief get a sapshot buffer, which size equal to screen size. the buffer only valid when PSRAM exist
@param[in] p Pointer of free mem. (if no psram, NULL will be returned)
*/
char *app_snapshot_get_buf(void);

/**
@brief refre cache to psram when psram cacheable
*/
void app_mem_flush_cache(void *data, uint32_t size);

/**
 * @brief  I-Cache Invalid by address
 * @param  data Address
 * @param  size Size of memory block (in number of bytes)
*/
void app_mem_invalid_icache(void *data, uint32_t size);

/**
@brief get a app memory type: PSRAM_HEAP , SRAM_HEAP ...
@retval app memory type.
*/
uint8_t app_get_mem_type(void *data);

typedef enum
{
    MEM_ASYN_IMG,
    MEM_ASYN_FONT,
} mem_aysnc_type_t;

void app_mem_set_ref_count(void *ptr, int ref_count, int type);
void app_mem_insert_asyn_node(void *ptr, void (*free_fun)(void *));
void app_mem_free_asyn_node(void);

/**
@brief      alloc mem for switch_animation's snapshot. use fixed length.
@param[in]  nbytes Size of snapshot mem
@param[in]  index  snapshot index
*/
void       *app_anim_buf_alloc(size_t nbytes, uint8_t index);

/**
@brief      alloc mem for switch_animation's snapshot. _ext use variable length.
@param[in]  nbytes Size of snapshot mem
@param[in]  index  snapshot index
*/
void       *app_anim_buf_alloc_ex(size_t nbytes, uint8_t index);

#ifndef app_anim_buf_alloc_ext
    #define app_anim_buf_alloc_ext app_anim_buf_alloc_ex
#endif

/**
@brief      free snapshot mem.
@param[in]  ptr snapshot Pointer to be free, which successsful apply by app_anim_buf_alloc_ex
*/
void       *app_anim_buf_free(void *ptr);

/**
@brief SVG fixed-block memory configuration.
*/
void        app_svg_memheap_config(svg_mem_mode_t mode, size_t max_num, size_t fixed_size);
void       *app_svg_alloc(size_t nbytes);
void        app_svg_free(void *ptr);

#if LV_USE_TINY_TTF
/**
@brief allocate/free tiny ttf draw buffer memory from the tiny heap
*/
void *app_tiny_ttf_mem_alloc(size_t size);
void *app_tiny_ttf_mem_free(void *buf);
#endif

#ifdef LV_USING_FREETYPE_ENGINE
/**
@brief get the allocated size of freetype cache.
*/
uint32_t ft_cache_alloc_size(void);

/**
@brief allocate memory for hindi shaper.
*/
void *hindi_shaper_malloc(size_t size);

/**
@brief free memory allocated by hindi_shaper_malloc.
*/
void hindi_shaper_free(void *ptr);
#endif

/**
@brief allocate memory for QuickJS engine.
*/
void *qjs_alloc(size_t nbytes);

/**
@brief free memory allocated by qjs_alloc.
*/
void qjs_free(void *ptr);

/**
@brief reallocate memory for QuickJS engine.
*/
void *qjs_realloc(void *ptr, size_t nbytes);

/**
@brief allocate memory for ulog ram backend.
*/
void *ulog_ram_mem_malloc(uint32_t size);

/**
@brief free memory allocated by ulog_ram_mem_malloc.
*/
void ulog_ram_mem_free(void *ptr);

/**
@brief reallocate memory for ulog ram backend.
*/
void *ulog_ram_mem_realloc(void *ptr, rt_size_t size);

#if PKG_USING_FFMPEG
/**
@brief initialize ffmpeg memory heap
*/
void ffmpeg_heap_init(void);

/**
@brief enable or disable animation buffer path for ffmpeg.
*/
void ffmpeg_enable_anim_buf(uint8_t en);

/**
@brief allocate memory from ffmpeg heap with alignment support for EPIC (>64K)
@param nbytes size in bytes to allocate
@retval pointer to allocated memory, NULL on failure
*/
void *ffmpeg_alloc(size_t nbytes);

/**
@brief free memory allocated by ffmpeg_alloc
@param p pointer previously returned by ffmpeg_alloc
*/
void ffmpeg_free(void *p);

/**
@brief reallocate memory from ffmpeg heap
@param p pointer previously returned by ffmpeg_alloc, NULL for new allocation
@param new_size new size in bytes
@retval pointer to reallocated memory, NULL on failure
*/
void *ffmpeg_realloc(void *p, size_t new_size);

/**
@brief allocate audio memory from ffmpeg heap
@param size size in bytes to allocate
@retval pointer to allocated memory
*/
void *audio_mem_malloc(uint32_t size);

/**
@brief free audio memory allocated by audio_mem_malloc
@param ptr pointer to free
*/
void audio_mem_free(void *ptr);

/**
@brief allocate zeroed audio memory from ffmpeg heap
@param count number of elements
@param size size of each element in bytes
@retval pointer to zeroed allocated memory
*/
void *audio_mem_calloc(uint32_t count, uint32_t size);

/**
@brief reallocate audio memory (uses rt_realloc).
*/
void *audio_mem_realloc(void *mem_address, unsigned int newsize);
#endif


/**********************
 *      MACROS
 **********************/
/********************************************************************
*
*  L1 non-retained section
*
********************************************************************/
/** L1 non-retained bss section begin*/
#define APP_L1_NON_RET_BSS_SECT_BEGIN(section_name)    L1_NON_RET_BSS_SECT_BEGIN(section_name)
/** L1 non-retained bss section end*/
#define APP_L1_NON_RET_BSS_SECT_END                    L1_NON_RET_BSS_SECT_END
/** L1 non-retained bss section*/
#define APP_L1_NON_RET_BSS_SECT(section_name, var)     L1_NON_RET_BSS_SECT(section_name, var)


/********************************************************************
 *
 *  L1 retained section
 *
 ********************************************************************/
/** L1 retained bss section begin */
#define APP_L1_RET_BSS_SECT_BEGIN(section_name)        L1_RET_BSS_SECT_BEGIN(section_name)
/** L1 retained bss section end */
#define APP_L1_RET_BSS_SECT_END                        L1_RET_BSS_SECT_END
/** L1 retained bss section */
#define APP_L1_RET_BSS_SECT(section_name, var)         L1_RET_BSS_SECT(section_name, var)


/********************************************************************
 *
 *  L2 non-cachable non-retained section
 *
 ********************************************************************/
/** L2 non-retained bss section begin */
#define APP_L2_NON_RET_BSS_SECT_BEGIN(section_name)    L2_NON_RET_BSS_SECT_BEGIN(section_name)
/** L2 non-retained bss section end */
#define APP_L2_NON_RET_BSS_SECT_END                    L2_NON_RET_BSS_SECT_END
/** L2 non-retained bss section */
#define APP_L2_NON_RET_BSS_SECT(section_name, var)     L2_NON_RET_BSS_SECT(section_name, var)


/********************************************************************
 *
 *  L2 non-cachable retained section
 *
 ********************************************************************/
/** L2 retained bss section begin */
#define APP_L2_RET_BSS_SECT_BEGIN(section_name)        L2_RET_BSS_SECT_BEGIN(section_name)
/** L2 retained bss section end */
#define APP_L2_RET_BSS_SECT_END                        L2_RET_BSS_SECT_END
/** L2 retained bss section */
#ifdef _MSC_VER
#define APP_L2_RET_BSS_SECT(section_name, var)         var
#else
#define APP_L2_RET_BSS_SECT(section_name, var)         var L2_RET_BSS_SECT(section_name)
#endif

/********************************************************************
 *
 *  L2 cachable non-retained section
 *
 ********************************************************************/
/** L2 cachable non-retained bss section begin*/
#define APP_L2_CACHE_NON_RET_BSS_SECT_BEGIN(section_name)    L2_CACHE_NON_RET_BSS_SECT_BEGIN(section_name)
/** L2 cachable non-retained bss section */
#define APP_L2_CACHE_NON_RET_BSS_SECT_END                    L2_CACHE_NON_RET_BSS_SECT_END
/** L2 cachable non-retained bss section */
#define APP_L2_CACHE_NON_RET_BSS_SECT(section_name)          L2_CACHE_NON_RET_BSS_SECT(section_name)

/********************************************************************
 *
 *  L2 cachable retained section
 *
 ********************************************************************/
/** L2 cachable retained bss section begin*/
#define APP_L2_CACHE_RET_BSS_SECT_BEGIN(section_name)        L2_CACHE_RET_BSS_SECT_BEGIN(section_name)
/** L2 cachable retained bss section end*/
#define APP_L2_CACHE_RET_BSS_SECT_END                        L2_CACHE_RET_BSS_SECT_END
/** L2 cachable retained bss section */
#define APP_L2_CACHE_RET_BSS_SECT(section_name)              L2_CACHE_RET_BSS_SECT(section_name)


#ifndef IMAGE_CACHE_IN_PSRAM_SIZE
#define IMAGE_CACHE_IN_PSRAM_SIZE 0
#endif

#if defined (ROTATE_MEM_IN_PSRAM) && IMAGE_CACHE_IN_PSRAM_SIZE > 0
#define ROTATE_MEM IMAGE_CACHE_PSRAM
#else //if define (ROTATE_MEM_IN_SRAM)
#define ROTATE_MEM IMAGE_CACHE_SRAM
#endif

#if defined (BSP_USING_PC_SIMULATOR) || (defined (ROTATE_MEM_IN_PSRAM) && IMAGE_CACHE_IN_PSRAM_SIZE > 0) || (defined (ROTATE_MEM_IN_SRAM) && IMAGE_CACHE_IN_SRAM_SIZE > 0)
#define CACHE_CLOCK_HANDS_COMPOSITE
#define CACHE_CLOCK_HANDS_COMPACT
#define CACHE_CLOCK_HANDS_KALEI
#define CACHE_ORIGIN_IMG_KALEI
#define CACHE_CLOCK_HANDS_MICKEY
#define CACHE_CLOCK_HANDS_SPACEMAN
#define CACHE_CLOCK_HANDS_SIMPLE
#define CACHE_ORIGIN_IMG_ROTATE
#define CACHE_COMPASS_ARROW
#else
#undef CACHE_CLOCK_HANDS_COMPOSITE
#undef CACHE_CLOCK_HANDS_COMPACT
#undef CACHE_CLOCK_HANDS_KALEI
#undef CACHE_ORIGIN_IMG_KALEI
#undef CACHE_CLOCK_HANDS_MICKEY
#undef CACHE_CLOCK_HANDS_SPACEMAN
#undef CACHE_CLOCK_HANDS_SIMPLE
#undef CACHE_ORIGIN_IMG_ROTATE
#undef CACHE_COMPASS_ARROW
#endif

/**
 * @brief  sifli_memxxx is DMA operation function.
 */
#if !defined(BSP_USING_PC_SIMULATOR) && !defined(MEMCPY_NON_DMA)
extern void *sifli_memcpy(void *dst, const void *src, rt_ubase_t count);
extern void *sifli_memset(void *s, int c, rt_ubase_t count);
#define app_memcpy sifli_memcpy
#define app_memset sifli_memset
#else
#define app_memcpy rt_memcpy
#define app_memset rt_memset
#endif

#ifdef BSP_USING_PC_SIMULATOR
#define RET_ADDR _ReturnAddress()
#else
#define RET_ADDR __builtin_return_address(0)
#endif

/**
 * @brief  RET_ADDR_TRACE used for memory trace.
 *         SDK app_mem has not migrated solution's header metadata write-back
 *         path yet, so keep a solution-compatible no-op hook here first.
 */
#define RET_ADDR_TRACE(p) do { (void)(p); } while (0)

//#define SIMULATOR_MEM_LEAKAGE
#if defined(BSP_USING_PC_SIMULATOR) && defined(SIMULATOR_MEM_LEAKAGE)
typedef enum
{
    LEAK_APP      = 0x01,
    LEAK_CACHE    = 0x02,
    LEAK_FT       = 0x04,
    LEAK_ANIM     = 0x08,
    LEAK_FFMPEG   = 0x10,
    LEAK_EPUB     = 0x20,
    LEAK_QJS      = 0x40,
} simulator_mem_leakage_t;

#define SIMULATOR_MEM_LEAKAGE_MODE                      LEAK_FT

#include <stdlib.h>
#define SIMULATOR_MEM_LEAKAGE_MALLOC(mode, x)          if ((mode) & SIMULATOR_MEM_LEAKAGE_MODE) return malloc(x);
#define SIMULATOR_MEM_LEAKAGE_CALLOC(mode, x, y)       if ((mode) & SIMULATOR_MEM_LEAKAGE_MODE) return calloc(x, y);
#define SIMULATOR_MEM_LEAKAGE_REALLOC(mode, x, y)      if ((mode) & SIMULATOR_MEM_LEAKAGE_MODE) return realloc(x, y);
#define SIMULATOR_MEM_LEAKAGE_FREE(mode, x)            if ((mode) & SIMULATOR_MEM_LEAKAGE_MODE) {free(x); return;}
#else
#define SIMULATOR_MEM_LEAKAGE_MALLOC(mode, x)
#define SIMULATOR_MEM_LEAKAGE_CALLOC(mode, x, y)
#define SIMULATOR_MEM_LEAKAGE_REALLOC(mode, x, y)
#define SIMULATOR_MEM_LEAKAGE_FREE(mode, x)
#endif

//for corner
#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
#define app_canvas_mem_alloc(size) app_cache_alloc(size, IMAGE_CACHE_PSRAM)
#define app_canvas_mem_free(p) app_cache_free(p)
#elif IMAGE_CACHE_IN_SRAM_SIZE > 0
#define app_canvas_mem_alloc(size) app_cache_alloc(size, IMAGE_CACHE_SRAM)
#define app_canvas_mem_free(p) app_cache_free(p)
#else
#define app_canvas_mem_alloc(size) lv_mem_alloc(size)
#define app_canvas_mem_free(p) lv_mem_free(p)
#endif

#ifndef FT_CACHE_SIZE
#define FT_CACHE_SIZE 0
#endif

#ifndef TINY_TTF_CACHE_SIZE
#define TINY_TTF_CACHE_SIZE 0
#endif

#ifndef FREETYPE_ACT_CACHE_SIZE
#define FREETYPE_ACT_CACHE_SIZE (FT_CACHE_SIZE) // * 75 / 100)
#endif

#ifndef MEDIA_CACHE_SIZE
#define MEDIA_CACHE_SIZE 0
#endif

#if defined(USING_EZIPA_DEC) && defined(EZIPA_CUSTOM_INCLUDE_FILE)
#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
#define EZIPA_LARGE_BUF_MALLOC(size) app_cache_alloc(size, IMAGE_CACHE_PSRAM)
#define EZIPA_LARGE_BUF_FREE(p) app_cache_free(p)
#elif IMAGE_CACHE_IN_SRAM_SIZE > 0
#define EZIPA_LARGE_BUF_MALLOC(size) app_cache_alloc(size, IMAGE_CACHE_SRAM)
#define EZIPA_LARGE_BUF_FREE(p) app_cache_free(p)
#else
#define EZIPA_LARGE_BUF_MALLOC(size) lv_mem_alloc(size)
#define EZIPA_LARGE_BUF_FREE(p) lv_mem_free(p)
#endif
#endif


#define FORCE_REUSE_ANIM_BUF        0x00010000

/**
 * @brief  Check if p is in the sysheap.
 */
#ifndef app_mem_is_sysheap
#define app_mem_is_sysheap(p)       rt_mem_is_sysheap(p)
#endif

/**
 * @brief  Check if p is in the memheap(heap).
 */
#ifndef app_mem_is_in_memheap
#define app_mem_is_in_memheap(heap, ptr) ((heap)->start_addr <= (void *)(ptr) && (uint8_t *)(ptr) < (uint8_t *)(heap)->start_addr + (heap)->pool_size)
#endif

#if defined(USING_BLOCK_MEM)
    #define app_mem_is_bmem(p)      mem_is_bmem(p)
#else
    #define app_mem_is_bmem(p)      0
#endif

#if defined(RT_USING_MEMHEAP_AS_HEAP) || defined(APP_USING_TLSF_MEM)
    #define sys_heap_get_ie(p, ie) (((struct rt_memheap_item *)((uint32_t)(p) - sizeof(struct rt_memheap_item)))->ie)
#else
    #define sys_heap_get_ie(p, ie) (((struct heap_mem *)((uint32_t)(p) - sizeof(struct heap_mem)))->ie)
#endif

#ifndef MEM_SET_HEADER_IE
#define MEM_SET_HEADER_IE(p, ie, x)                                                                                 \
        if (p)                                                                                                      \
        {                                                                                                           \
            if (app_mem_is_bmem(p))                                                                                 \
                ((bmem_item_t *)((uint32_t)(p) - sizeof(bmem_item_t)))->ie = (uint32_t)(x);                        \
            else if (app_mem_is_sysheap(p))                                                                         \
                sys_heap_get_ie(p, ie) = (rt_uint32_t)(x);                                                         \
            else                                                                                                    \
                ((struct rt_memheap_item *)((uint32_t)(p) - sizeof(struct rt_memheap_item)))->ie = (uint32_t)(x);   \
        }
#endif

#ifndef MEM_GET_HEADER_IE
#define MEM_GET_HEADER_IE(p, ie)                                                                                    \
        (app_mem_is_bmem(p) ? ((bmem_item_t *)((uint32_t)(p) - sizeof(bmem_item_t)))->ie :                          \
        app_mem_is_sysheap(p) ? sys_heap_get_ie(p, ie) :                                                            \
        ((struct rt_memheap_item *)((uint32_t)(p) - sizeof(struct rt_memheap_item)))->ie)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_IMG_BUF_H*/
