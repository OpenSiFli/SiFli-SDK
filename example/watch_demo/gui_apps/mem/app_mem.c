/**
 * @file app_mem.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <stddef.h>
#include <string.h>
#include "app_mem.h"

#ifndef WIN32
    #include "register.h"
#endif

extern rt_size_t rt_heapmem_size(void *rmem);

enum
{
    SYS_HEAP,
    SRAM_HEAP,
    PSRAM_HEAP,
} ;


#ifdef RT_USING_FINSH
#include <finsh.h>
static uint8_t mem_log = 0;
int app_mem_log(void)
{
    mem_log = (mem_log + 1) & 0x01;
    rt_kprintf("app_mem_log: %d\n", mem_log);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(app_mem_log, app_mem, app_mem: open or close app_mem log);
#endif


/**
    Note: the following MACRO defined in menuconfig.
*/

/**
    if no pram exists. disable all MARCO relative to PSRAM.
*/
#if !defined(BSP_USING_PSRAM) && !defined(BSP_USING_PC_SIMULATOR)
    #undef IMAGE_CACHE_IN_PSRAM_SIZE
    #undef MESSAGE_CACHE_IN_PSRAM
    #undef FT_CACHE_IN_PSRAM
    #define IMAGE_CACHE_IN_PSRAM_SIZE 0
#endif


#if defined(BSP_USING_PSRAM) || defined(BSP_USING_PC_SIMULATOR)
    /**
    for the case when pram exists.
    Note:
    1. for message/image(gif/rotate)/ft, it can't exsit both in SRAM and PSRAM. customer can choose the configuration.
    2. for image cache, it can be configured both in PSRAM and in SRAM
    */

    /**
    for L1_MEM(SRAM)
    */
    APP_L1_NON_RET_BSS_SECT_BEGIN(app_sram_non_ret_cache)

    #ifdef MESSAGE_CACHE_IN_SRAM_STANDALONE
        APP_L1_NON_RET_BSS_SECT(app_sram_non_ret_cache, ALIGN(4) static uint8_t app_message_cache[MESSAGE_CACHE_SIZE]);
    #endif

    #if IMAGE_CACHE_IN_SRAM_SIZE > 0
        APP_L1_NON_RET_BSS_SECT(app_sram_non_ret_cache, ALIGN(4) static uint8_t app_image_sram_cache[IMAGE_CACHE_IN_SRAM_SIZE]);
    #endif

    #ifdef FREETYPE_CACHE_IN_SRAM_STANDALONE
        APP_L1_NON_RET_BSS_SECT(app_sram_non_ret_cache, ALIGN(4) static uint8_t app_ft_cache[FT_CACHE_SIZE]);
    #endif

    APP_L1_NON_RET_BSS_SECT_END

    /**
    for L2_MEM(PSRAM).
    */
    APP_L2_RET_BSS_SECT_BEGIN(app_psram_ret_cache)

    #if IMAGE_CACHE_IN_PSRAM_SIZE > 0
        APP_L2_RET_BSS_SECT(app_psram_ret_cache, ALIGN(4) static uint8_t app_image_psram_cache[IMAGE_CACHE_IN_PSRAM_SIZE]);
    #endif

    #ifdef MESSAGE_CACHE_IN_PSRAM
        APP_L2_RET_BSS_SECT(app_psram_ret_cache, ALIGN(4) static uint8_t app_message_cache[MESSAGE_CACHE_SIZE]);
    #endif


    #ifdef FREETYPE_CACHE_IN_PSRAM
        APP_L2_RET_BSS_SECT(app_psram_ret_cache, ALIGN(4) static uint8_t app_ft_cache[FT_CACHE_SIZE]);
    #endif

    #ifdef QUICKJS_PSRAM_SIZE
        APP_L2_RET_BSS_SECT(app_psram_ret_cache, ALIGN(4) static uint8_t app_qjs_cache[QUICKJS_PSRAM_SIZE]);
    #endif

    #if defined(SNAPSHOT_CACHE_IN_PSRAM)
        APP_L2_RET_BSS_SECT(app_psram_ret_cache, ALIGN(4) static char app_snapshot_cache[LV_HOR_RES_MAX * LV_VER_RES_MAX * LV_COLOR_SIZE / 8]);
    #endif

    APP_L2_RET_BSS_SECT_END

#else
    /**
    for no pram case. customer can config message/gif/ft cache in SRAM.
    */

    APP_L1_NON_RET_BSS_SECT_BEGIN(app_sram_non_ret_cache)
    #ifdef MESSAGE_CACHE_IN_SRAM_STANDALONE
        APP_L1_NON_RET_BSS_SECT(app_sram_non_ret_cache, ALIGN(4) static uint8_t app_message_cache[MESSAGE_CACHE_SIZE]);
    #endif

    #if IMAGE_CACHE_IN_SRAM_SIZE > 0
        APP_L1_NON_RET_BSS_SECT(app_sram_non_ret_cache, ALIGN(4) static uint8_t app_image_sram_cache[IMAGE_CACHE_IN_SRAM_SIZE]);
    #endif

    #ifdef FREETYPE_CACHE_IN_SRAM_STANDALONE
        APP_L1_NON_RET_BSS_SECT(app_sram_non_ret_cache, ALIGN(4) static uint8_t app_ft_cache[FT_CACHE_SIZE]);
    #endif

    APP_L1_NON_RET_BSS_SECT_END

#endif



#ifdef APP_TRANS_ANIMATION_SCALE_NEXT
    L2_NON_RET_BSS_SECT_BEGIN(anim_frambuf)
    APP_L2_NON_RET_BSS_SECT(anim_frambuf, static char app_trans_anim_buf_a[LV_HOR_RES_MAX * LV_VER_RES_MAX * LV_COLOR_SIZE / 8]);
    L2_NON_RET_BSS_SECT_END
#elif defined(APP_TRANS_ANIMATION_SCALE)
    L2_NON_RET_BSS_SECT_BEGIN(anim_frambuf)
    APP_L2_NON_RET_BSS_SECT(anim_frambuf, static char app_trans_anim_buf_a[LV_HOR_RES_MAX * LV_VER_RES_MAX * LV_COLOR_SIZE / 8]);
    APP_L2_NON_RET_BSS_SECT(anim_frambuf, static char app_trans_anim_buf_b[LV_HOR_RES_MAX * LV_VER_RES_MAX * LV_COLOR_SIZE / 8]);
    L2_NON_RET_BSS_SECT_END
#elif defined(APP_TRANS_ANIMATION_OVERWRITE) || defined(APP_TRANS_ANIMATION_NONE)
    /*No trans animtion buf need*/
#else
    #error "Need trans animtion buf?"
#endif /* APP_TRANS_ANIMATION_SCALE_NEXT */



#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
    struct rt_memheap app_image_psram_memheap;
#endif

#if IMAGE_CACHE_IN_SRAM_SIZE > 0
    struct rt_memheap app_image_sram_memheap;
#endif

#ifndef FREETYPE_CACHE_IN_SRAM
    struct rt_memheap app_ft_memheap;
#endif

#ifdef QUICKJS_PSRAM_SIZE
    struct rt_memheap app_qjs_memheap;
#endif

#ifndef MESSAGE_BUFFER_IN_SRAM
    struct rt_memheap app_message_memheap;
#endif

/**********************
 *   GLOBAL FUNCTIONS
 **********************/


static int app_cahe_memheap_init(void)
{

#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
    rt_memheap_init(&app_image_psram_memheap, "app_image_psram_memheap", (void *)app_image_psram_cache, IMAGE_CACHE_IN_PSRAM_SIZE);
#endif

#if IMAGE_CACHE_IN_SRAM_SIZE > 0
    rt_memheap_init(&app_image_sram_memheap, "app_image_sram_memheap", (void *)app_image_sram_cache, IMAGE_CACHE_IN_SRAM_SIZE);
#ifdef RT_USING_MEMHEAP_AS_HEAP
    {
        rt_err_t err = rt_memheap_add_to_sys(&app_image_sram_memheap);
        RT_ASSERT(RT_EOK == err);
    }
#endif
#endif


#if defined (MESSAGE_CACHE_IN_SRAM_STANDALONE) || defined (MESSAGE_CACHE_IN_PSRAM)
    rt_memheap_init(&app_message_memheap, "app_message_memheap", (void *)app_message_cache, MESSAGE_CACHE_SIZE);
#endif

#if defined (FREETYPE_CACHE_IN_SRAM_STANDALONE) || defined (FREETYPE_CACHE_IN_PSRAM)
    rt_memheap_init(&app_ft_memheap, "app_ft_memheap", (void *)app_ft_cache, FT_CACHE_SIZE);
#ifdef RT_USING_MEMHEAP_AS_HEAP
    {
#if IMAGE_CACHE_IN_SRAM_SIZE > 0
        rt_err_t err = rt_memheap_add_to_sys(&app_image_sram_memheap);
        RT_ASSERT(RT_EOK == err);
#endif
    }
#endif
#endif

#ifdef QUICKJS_PSRAM_SIZE
    rt_memheap_init(&app_qjs_memheap, "app_qjs_memheap", (void *)app_qjs_cache, QUICKJS_PSRAM_SIZE);
#endif

    return 0;
}
INIT_PREV_EXPORT(app_cahe_memheap_init);


void *app_cache_alloc(size_t size, image_cache_t cache_type)
{
    uint8_t *p = NULL;

    size += 4;
    /* Allocate raw buffer */

    //reused the mem with GIF
    if (IMAGE_CACHE_SRAM == cache_type)
    {
#if IMAGE_CACHE_IN_SRAM_SIZE > 0
        p = (uint8_t *)rt_memheap_alloc(&app_image_sram_memheap, size);
        if (p)((uint32_t *) p)[0] = SRAM_HEAP;
#else
        p = (uint8_t *)rt_malloc(size);
        if (p)((uint32_t *) p)[0] = SYS_HEAP;
#endif
    }

#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
    if (!p)
    {
        p = (uint8_t *)rt_memheap_alloc(&app_image_psram_memheap, size);
        if (p)((uint32_t *) p)[0] = PSRAM_HEAP;
    }
#endif

    if (!p)
    {
        p = (uint8_t *)rt_malloc(size);
        if (p)((uint32_t *) p)[0] = SYS_HEAP;
    }

    //RT_ASSERT(p);
#ifdef RT_USING_FINSH
    if (mem_log) rt_kprintf("app_cache_alloc: size %d p %p. \n", size, p + 4);
#endif

    if (!p)
    {
        rt_kprintf("app_cache_alloc: size %d failed!", size);
        return 0;
    }

    return p + 4;
}

void app_cache_free(void *p)
{
    uint8_t *temp_p = p;

    temp_p -= 4;
    if (PSRAM_HEAP == ((uint32_t *) temp_p)[0] || SRAM_HEAP == ((uint32_t *) temp_p)[0])
    {
        rt_memheap_free(temp_p);
    }
    else
    {
        rt_free(temp_p);
    }

#ifdef RT_USING_FINSH
    if (mem_log) rt_kprintf("app_cache_free: p %p. \n", p);
#endif
}

void *app_message_alloc(size_t size)
{
    uint8_t *p = NULL;

    size += 4;

#if (MESSAGE_CACHE_SIZE > 0 && defined(MESSAGE_CACHE_IN_PSRAM))
    p = rt_memheap_alloc(&app_message_memheap, size);
    if (p)((uint32_t *) p)[0] = PSRAM_HEAP;
#elif (MESSAGE_CACHE_SIZE > 0 && defined(MESSAGE_CACHE_IN_SRAM_STANDALONE))
    p = rt_memheap_alloc(&app_message_memheap, size);
    if (p)((uint32_t *) p)[0] = SRAM_HEAP;
#endif

    if (!p)
    {
        p = rt_malloc(size);
        if (p)((uint32_t *) p)[0] = SYS_HEAP;
    }

#ifdef RT_USING_FINSH
    if (mem_log) rt_kprintf("app_message_alloc: size %d p %p. \n", size, p + 4);
#endif

    if (!p)
    {
        rt_kprintf("app_cache_alloc: size %d failed!", size);
        return 0;
    }

    return p + 4;
}

void app_message_free(void *p)
{
    uint8_t *temp_p = p;

    temp_p -= 4;
    if (PSRAM_HEAP == ((uint32_t *) temp_p)[0] || SRAM_HEAP == ((uint32_t *) temp_p)[0])
    {
        rt_memheap_free(temp_p);
    }
    else
    {
        rt_free(temp_p);
    }

#ifdef RT_USING_FINSH
    if (mem_log) rt_kprintf("app_message_free: %p. \n", p);
#endif
}


#ifdef DISABLE_LVGL_V8
/**
 * Get the memory consumption of a raw bitmap, given color format and dimensions.
 * @param w width
 * @param h height
 * @param cf color format
 * @return size in bytes
 */
uint32_t lv_img_buf_get_img_size(lv_coord_t w, lv_coord_t h, lv_img_cf_t cf)
{
    switch (cf)
    {
    case LV_IMG_CF_TRUE_COLOR:
        return LV_IMAGE_BUF_SIZE_TRUE_COLOR(w, h);
    case LV_IMG_CF_TRUE_COLOR_ALPHA:
        return LV_IMAGE_BUF_SIZE_TRUE_COLOR_ALPHA(w, h);
    case LV_COLOR_FORMAT_A1:
        return LV_IMAGE_BUF_SIZE_ALPHA_1BIT(w, h);
    case LV_COLOR_FORMAT_A2:
        return LV_IMAGE_BUF_SIZE_ALPHA_2BIT(w, h);
    case LV_COLOR_FORMAT_A4:
        return LV_IMAGE_BUF_SIZE_ALPHA_4BIT(w, h);
    case LV_IMG_CF_ALPHA_8BIT:
        return LV_IMAGE_BUF_SIZE_ALPHA_8BIT(w, h);
    case LV_COLOR_FORMAT_I1:
        return LV_IMAGE_BUF_SIZE_INDEXED_1BIT(w, h);
    case LV_COLOR_FORMAT_I2:
        return LV_IMAGE_BUF_SIZE_INDEXED_2BIT(w, h);
    case LV_COLOR_FORMAT_I4:
        return LV_IMAGE_BUF_SIZE_INDEXED_4BIT(w, h);
    case LV_COLOR_FORMAT_I8:
        return LV_IMAGE_BUF_SIZE_INDEXED_8BIT(w, h);
    default:
        return 0;
    }
}
#endif

lv_img_dsc_t *app_cache_img_alloc(lv_coord_t w, lv_coord_t h, lv_img_cf_t cf, uint32_t data_size, image_cache_t cache_type)
{
    /* Allocate image descriptor */
    lv_img_dsc_t *dsc = rt_malloc(sizeof(lv_img_dsc_t));
    if (dsc == NULL)
        return NULL;

    RT_ASSERT(dsc);
    memset(dsc, 0x00, sizeof(lv_img_dsc_t));

    /* Get image data size */

    //for A0, data_size can't compute by w*h
    if (data_size > 0)
    {
        dsc->data_size = data_size;
    }
    else
    {
        dsc->data_size = lv_img_buf_get_img_size(w, h, cf);
    }
    if (dsc->data_size == 0)
    {
        rt_free(dsc);
        return NULL;
    }

    dsc->data = (uint8_t *)app_cache_alloc(dsc->data_size, cache_type);

    //RT_ASSERT(dsc->data);

    if (dsc->data == NULL)
    {
        rt_free(dsc);
        return NULL;
    }

    /* Fill in header */
    dsc->header.always_zero = 0;
    dsc->header.w = w;
    dsc->header.h = h;
    dsc->header.cf = cf;



    return dsc;
}


void app_cache_img_free(lv_img_dsc_t *p_img)
{
    if (p_img && p_img->data)
    {
        lv_img_cache_invalidate_src(p_img);
        app_cache_free((void *)p_img->data);
        rt_free(p_img);
    }
}

/**
 * duplicate an image to SRAM/or PSRAM to improve drawn performance
 * \n
 *
 * @return
 * @param copy
 * \n
 * @see
 */
lv_img_dsc_t *app_cache_copy_alloc(const void *copy, image_cache_t cache_type)
{
    lv_img_dsc_t img_dsc_temp;
    lv_img_dsc_t *dsc;

    if (NULL == copy) return NULL;

    img_dsc_temp = *(lv_img_dsc_t *) copy;

    dsc = app_cache_img_alloc(img_dsc_temp.header.w, img_dsc_temp.header.h, img_dsc_temp.header.cf, img_dsc_temp.data_size, cache_type);

    RT_ASSERT(dsc);
    RT_ASSERT(img_dsc_temp.data);
    if (img_dsc_temp.data_size != dsc->data_size)
        rt_kprintf("warnning: app_cache_img_alloc diff size, cache %d, copy->data_size %d", dsc->data_size, img_dsc_temp.data_size);
    memcpy((uint8_t *)dsc->data, (uint8_t *)img_dsc_temp.data, dsc->data_size);

    return dsc;
}

void app_cache_copy_free(lv_img_dsc_t *rel_mem)
{
    app_cache_img_free(rel_mem);
}

void *app_anim_mem_alloc(rt_size_t size, bool anim_data)
{
    uint8_t *p = NULL;

    size += 4;

#if IMAGE_CACHE_IN_SRAM_SIZE > 0
    p = rt_memheap_alloc(&app_image_sram_memheap, size);
    if (p)((uint32_t *) p)[0] = SRAM_HEAP;
#endif

    if (anim_data)
    {
        if (!p)
        {
#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
            p  = (uint8_t *)rt_memheap_alloc(&app_image_psram_memheap, size);
            if (p)((uint32_t *) p)[0] = PSRAM_HEAP;
#endif
        }

        if (!p)
        {
            p = rt_malloc(size);
            if (p)((uint32_t *) p)[0] = SYS_HEAP;
        }
    }
    else
    {
        if (!p)
        {
            p = rt_malloc(size);
            if (p)((uint32_t *) p)[0] = SYS_HEAP;
        }

        if (!p)
        {
#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
            p  = (uint8_t *)rt_memheap_alloc(&app_image_psram_memheap, size);
            if (p)((uint32_t *) p)[0] = PSRAM_HEAP;
#endif
        }
    }

    if (mem_log) rt_kprintf("app_anim_mem_alloc: %p %d. \n", p, size);

    if (!p)
    {
        rt_kprintf("app_cache_alloc: size %d failed!", size);
        return 0;
    }

    return p + 4;
}

void *app_anim_mem_realloc(void *p, size_t new_size)
{
    if (!p)
        return p;

    uint8_t *ret = NULL;

    uint8_t *temp_p = p;
    new_size += 4;
    temp_p -= 4;
#if IMAGE_CACHE_IN_SRAM_SIZE > 0
    if (SRAM_HEAP == ((uint32_t *) temp_p)[0])
    {
        ret  = rt_memheap_realloc(&app_image_sram_memheap, temp_p, new_size);
        if (ret)
            ((uint32_t *) ret)[0] = SRAM_HEAP;
        else
        {
#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
            ret = (uint8_t *)rt_memheap_alloc(&app_image_psram_memheap, new_size);
            if (ret)
            {
                ((uint32_t *)ret)[0] = PSRAM_HEAP;
                rt_kprintf("realloc sram 2 psram\n");
                size_t old_size = rt_heapmem_size(temp_p) ;
                if (old_size > new_size)
                    old_size = new_size;
                rt_memcpy(ret + 4, p, old_size - 4);
                rt_memheap_free(temp_p);
            }
#endif
        }
    }
    else
#endif
#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
        if (PSRAM_HEAP == ((uint32_t *) temp_p)[0])
        {
            ret  = rt_memheap_realloc(&app_image_psram_memheap, temp_p, new_size);
            if (ret)((uint32_t *) ret)[0] = PSRAM_HEAP;
        }
        else
#endif
        {
            ret = rt_realloc(temp_p, new_size);
            if (ret)((uint32_t *) ret)[0] = SYS_HEAP;
        }


    if (mem_log) rt_kprintf("app_anim_mem_realloc: old:%p->new:%p, new_size:%d. \n", temp_p, ret, new_size);

    if (!p)
    {
        rt_kprintf("app_anim_mem_realloc: size %d failed!", new_size);
        return 0;
    }

    if (ret)
        ret += 4;
    return ret;
}

void app_anim_mem_free(void *p)
{

    uint8_t *temp_p = p;
    if (!p)
        return;
    temp_p -= 4;
    if (mem_log) rt_kprintf("app_anim_mem_free: %p. \n", temp_p);
    if (PSRAM_HEAP == ((uint32_t *) temp_p)[0] || SRAM_HEAP == ((uint32_t *) temp_p)[0])
    {
        rt_memheap_free(temp_p);
    }
    else
    {
        rt_free(temp_p);
    }


}


void *app_anim_buf_alloc(size_t nbytes, uint8_t index)
{
    void *ptr = NULL;

#ifdef APP_TRANS_ANIMATION_SCALE_NEXT
    ptr = &app_trans_anim_buf_a;
#elif defined(APP_TRANS_ANIMATION_SCALE)
    if (0 == index) ptr = &app_trans_anim_buf_a;
    if (1 == index) ptr = &app_trans_anim_buf_b;
#elif defined(APP_TRANS_ANIMATION_OVERWRITE) || defined(APP_TRANS_ANIMATION_NONE)
    ptr = NULL;
#else

#endif /* APP_TRANS_ANIMATION_SCALE_NEXT */

    rt_kprintf("app_anim_buf_alloc: %p index %d size %d\n", ptr, index, nbytes);
    return ptr;
}

void *app_anim_buf_free(void *ptr)
{
    rt_kprintf("app_anim_buf_free: %p\n", ptr);

    return NULL;
}

uint8_t app_get_mem_type(void *data)
{
    uint8_t *temp_p = data;
    temp_p -= 4;

    return ((uint32_t *) temp_p)[0];
}

void app_mem_flush_cache(void *data, uint32_t size)
{
#ifndef WIN32
    if (PSRAM_HEAP == app_get_mem_type(data))
        SCB_CleanDCache_by_Addr(data, (size + 3) >> 2 << 2);
#endif
}

#if (defined(BSP_USING_PSRAM) || defined(BSP_USING_PC_SIMULATOR)) && defined(SNAPSHOT_CACHE_IN_PSRAM)
char *app_snapshot_get_buf(void)
{
    return &app_snapshot_cache[0];
}
#else
char *app_snapshot_get_buf(void)
{
    return NULL;
}

#endif

#if FT_CACHE_SIZE > 0
#ifndef FREETYPE_CACHE_IN_SRAM
void ft_get_mem_info(uint32_t *available_size, uint32_t *memheap_size, uint32_t *act_cache_size)
{
    extern struct rt_memheap app_ft_memheap;
    *memheap_size = app_ft_memheap.pool_size;
    *available_size = app_ft_memheap.available_size;
    *act_cache_size = FREETYPE_ACT_CACHE_SIZE;
}

#else
void ft_get_mem_info(uint32_t *available_size, uint32_t *memheap_size, uint32_t *act_cache_size)
{
    uint32_t max_used_size;
    rt_memory_info((rt_uint32_t *)memheap_size, (rt_uint32_t *)available_size, (rt_uint32_t *)&max_used_size);
    *available_size = *memheap_size - *available_size;
    *act_cache_size = FREETYPE_ACT_CACHE_SIZE;
}

#endif

#else
void ft_get_mem_info(uint32_t *available_size, uint32_t *memheap_size, uint32_t *act_cache_size)
{
    *memheap_size = FREETYPE_ACT_CACHE_SIZE;
    *available_size = FREETYPE_ACT_CACHE_SIZE;
    *act_cache_size = FREETYPE_ACT_CACHE_SIZE;
}

#endif

uint32_t app_mem_get_ft_cache_size(void)

{
    return FREETYPE_ACT_CACHE_SIZE;
}


#if PKG_USING_FFMPEG
typedef struct _ffmpeg_mem_header
{
    uint32_t magic;
    uint32_t offset;//Offset between 'ffmpeg_alloc' returned value and 'app_anim_mem_(re)alloc' returned value
    uint32_t size;
} ffmpeg_mem_header;
//Assumed that > 64K memory area used by EPIC
#define ALIGN64_SIZE_THRESHOLD 65536
#define FFMPEG_MEM_HEADER sizeof(ffmpeg_mem_header)
#define FFMPEG_MEM_MAGIC  0xFF3E63E3
#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

void *ffmpeg_alloc(size_t nbytes)
{
    uint8_t *p;
    ffmpeg_mem_header *header_p;

    if (nbytes > ALIGN64_SIZE_THRESHOLD)
    {
        size_t header_size = 63 + FFMPEG_MEM_HEADER;
        p = app_anim_mem_alloc(nbytes + header_size, 1);
        if (!p) return NULL;

        header_p = (ffmpeg_mem_header *)(RT_ALIGN_DOWN((uint32_t)(p + header_size), 64) - FFMPEG_MEM_HEADER);

        RT_ASSERT(((uint32_t)header_p) >= ((uint32_t)p));
    }
    else
    {
        p = app_anim_mem_alloc(nbytes + FFMPEG_MEM_HEADER, 1);
        if (!p) return NULL;

        header_p = (ffmpeg_mem_header *) p;
    }


    header_p->magic = FFMPEG_MEM_MAGIC;
    header_p->offset = ((uint32_t)header_p) + sizeof(ffmpeg_mem_header) - ((uint32_t)p);
    header_p->size = nbytes;

    return (uint8_t *)(header_p + 1);
}

void ffmpeg_free(void *p)
{
    if (!p) return;

    ffmpeg_mem_header *header_p = ((ffmpeg_mem_header *)p) - 1;

    RT_ASSERT(FFMPEG_MEM_MAGIC == header_p->magic);
    app_anim_mem_free(((uint8_t *)p) - header_p->offset);
}

void *ffmpeg_realloc(void *p, size_t new_size)
{
    if (!p) return ffmpeg_alloc(new_size);
    if (!new_size)
    {
        ffmpeg_free(p);
        return NULL;
    }

    uint8_t *new_p = ffmpeg_alloc(new_size);
    if (new_p)
    {
        ffmpeg_mem_header *header_p = ((ffmpeg_mem_header *)p) - 1;
        RT_ASSERT(FFMPEG_MEM_MAGIC == header_p->magic);
        memcpy(new_p, p, MIN(new_size, header_p->size));
        ffmpeg_free(p);
    }

    return new_p;
}
#endif

#ifdef LV_USING_FREETYPE_ENGINE
#if (defined (FREETYPE_CACHE_IN_SRAM_STANDALONE) || defined (FREETYPE_CACHE_IN_PSRAM))
extern struct rt_memheap app_ft_memheap;
void *ft_smalloc(size_t nbytes)
{
    return rt_memheap_alloc(&app_ft_memheap, nbytes);
}

void ft_sfree(void *ptr)
{
    rt_memheap_free(ptr);
}

void *ft_srealloc(void *ptr, size_t nbytes)
{
    return rt_memheap_realloc(&app_ft_memheap, ptr, nbytes);
}

void *ft_scalloc(size_t count, size_t size)
{
    return rt_memheap_calloc(&app_ft_memheap, count, size);
}
uint32_t app_mem_get_ft_cache_avail_size(void)
{

    return FT_CACHE_SIZE - app_ft_memheap.available_size;
}
#else
uint32_t ft_alloc_size = 0;
void *ft_smalloc(size_t nbytes)
{
    uint8_t *p;
#ifdef USING_MEM_BLOCK
    p = block_mem_alloc(nbytes);
    if (p) ft_alloc_size += block_mem_size(p);
#else
    p = rt_malloc(nbytes);
    if (p)  ft_alloc_size += rt_mem_size(p);
#endif
    return p;
}

void ft_sfree(void *ptr)
{
    if (!ptr) return;
#ifdef USING_MEM_BLOCK
    ft_alloc_size -= block_mem_size(ptr);
    block_mem_free(ptr);
#else
    ft_alloc_size -= rt_mem_size(ptr);
    rt_free(ptr);
#endif
}

void *ft_srealloc(void *ptr, size_t nbytes)
{
    uint8_t *p;
#ifdef USING_MEM_BLOCK
    ft_alloc_size -= block_mem_size(ptr);
    p = block_mem_realloc(ptr, nbytes);
    if (p) ft_alloc_size += block_mem_size(p);
#else
    ft_alloc_size -= rt_mem_size(ptr);
    p = rt_realloc(ptr, nbytes);
    if (p) ft_alloc_size += rt_mem_size(p);
#endif
    return p;

}

void *ft_scalloc(size_t count, size_t size)
{
    uint8_t *p;
#ifdef USING_MEM_BLOCK
    p = block_mem_calloc(count, size);
    if (p) ft_alloc_size += block_mem_size(p);
#else
    p = rt_calloc(count, size);
    if (p) ft_alloc_size += rt_mem_size(p);
#endif
    return p;
}

uint32_t app_mem_get_ft_cache_avail_size(void)
{
    rt_uint32_t total_size;
    rt_memory_info(&total_size, NULL, NULL);
    return total_size - ft_alloc_size;
}
#endif
#endif


/**********************
 *   STATIC FUNCTIONS
 **********************/

