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
//#include "lv_img_buf.h"
#include "mem_section.h"

/*********************
 *      DEFINES
 *********************/
typedef enum
{
    IMAGE_CACHE_HEAP,
    IMAGE_CACHE_SRAM,
    IMAGE_CACHE_PSRAM
} image_cache_t;


/**
@brief apply cache mem for solution applicaiton.
@param[in] size Size of cache mem
@param[in] cache_type Cache type of cache mem be applied
@retval Pointer of the successsful applicaiton.
*/
void *app_cache_alloc(size_t size, image_cache_t cache_type);

/**
@brief re-apply cache mem for solution applicaiton.
@param[in] Original memory
@param[in] new_size New size of cache mem
@param[in] cache_type Cache type of cache mem be applied
@retval Pointer of the successsful applicaiton.
*/
void *app_cache_realloc(void *memory, size_t new_size, image_cache_t cache_type);

/**
@brief free cache mem which successsful apply by app_cache_alloc.
@param[in] p Pointer of free mem
*/
void app_cache_free(void *p);


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
@brief get a sapshot buffer, which size equal to screen size. the buffer only valid when PSRAM exist
@param[in] p Pointer of free mem. (if no psram, NULL will be returned)
*/
char *app_snapshot_get_buf(void);

/**
@brief refre cache to psram when psram cacheable
*/
void app_mem_flush_cache(void *data, uint32_t size);

/**
@brief get a app memory type: PSRAM_HEAP , SRAM_HEAP ...
@retval app memory type.
*/
uint8_t app_get_mem_type(void *data);


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

#define FREETYPE_ACT_CACHE_SIZE (FT_CACHE_SIZE) // * 75 / 100)

#if defined(USING_EZIPA_DEC)
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


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_IMG_BUF_H*/
