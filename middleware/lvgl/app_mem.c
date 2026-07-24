/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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

#ifdef MEM_ASYN_FREE
#define APP_MEMHEAP_MAGIC 0x1ea01ea0U
#define APP_MEMHEAP_MASK  0xfffffffeU
#define APP_MEMHEAP_SIZE  RT_ALIGN(sizeof(struct rt_memheap_item), RT_ALIGN_SIZE)

typedef struct
{
    rt_list_t list;
    void *ptr;
    void (*free)(void *);
} mem_async_node_t;

static rt_list_t app_mem_async_list;
static struct rt_mutex mem_asyn_mutex;

static struct rt_memheap_item *app_mem_get_async_header(void *ptr)
{
    struct rt_memheap_item *header;

    /* Validate the allocator header magic number instead of relying on a fixed address-window 
    allowlist — SBus remap addresses may fall outside the PSRAM base/size range. */
    if (!ptr)
    {
        return NULL;
    }

    header = (struct rt_memheap_item *)((uint8_t *)ptr - APP_MEMHEAP_SIZE);
    if (((header->magic & APP_MEMHEAP_MASK) == APP_MEMHEAP_MAGIC) &&
            (header->ref_count_magic == REF_COUNT_MAGIC))
    {
        return header;
    }

    return NULL;
}

#endif


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

/*
 * Backward compatibility: the reusable transition-buffer feature started in
 * reader project code. Keep accepting the old project-scoped macro while the
 * SDK migrates to the generic app_mem naming.
 */
#if defined(SIFLI_READER_REUSE_TRANS_ANIM_BUF) && !defined(SIFLI_APP_MEM_REUSE_TRANS_ANIM_BUF)
    #define SIFLI_APP_MEM_REUSE_TRANS_ANIM_BUF
#endif

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
    1. for message/image(gif/rotate)/ft/tiny_ttf, it can't exsit both in SRAM and PSRAM. customer can choose the configuration.
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

    #if LV_USE_TINY_TTF && defined(TINY_TTF_CACHE_IN_SRAM_STANDALONE)
        APP_L1_NON_RET_BSS_SECT(app_sram_non_ret_cache, ALIGN(4) static uint8_t app_tiny_ttf_cache[TINY_TTF_CACHE_SIZE]);
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

    #if LV_USE_TINY_TTF && defined(TINY_TTF_CACHE_IN_PSRAM)
        APP_L2_RET_BSS_SECT(app_psram_ret_cache, ALIGN(4) static uint8_t app_tiny_ttf_cache[TINY_TTF_CACHE_SIZE]);
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

    #if LV_USE_TINY_TTF && defined(TINY_TTF_CACHE_IN_SRAM_STANDALONE)
        APP_L1_NON_RET_BSS_SECT(app_sram_non_ret_cache, ALIGN(4) static uint8_t app_tiny_ttf_cache[TINY_TTF_CACHE_SIZE]);
    #endif

    APP_L1_NON_RET_BSS_SECT_END

#endif



#ifdef APP_TRANS_ANIMATION_SCALE_NEXT
    L2_NON_RET_BSS_SECT_BEGIN(anim_frambuf)
    APP_L2_NON_RET_BSS_SECT(anim_frambuf, static char app_trans_anim_buf_a[LV_HOR_RES_MAX * LV_VER_RES_MAX * LV_COLOR_SIZE / 8]);
    L2_NON_RET_BSS_SECT_END
#elif defined(APP_TRANS_ANIMATION_SCALE) || defined(SIFLI_APP_MEM_REUSE_TRANS_ANIM_BUF)
    /*
     * Keep a pair of frame buffers when app_mem needs to reuse them as large
     * temporary memheaps. This is independent from UI transition style:
     * APP_TRANS_ANIMATION_NONE may still be selected while upper layers borrow
     * these buffers for document/image decode work.
     */
    L2_NON_RET_BSS_SECT_BEGIN(anim_frambuf)
    APP_L2_NON_RET_BSS_SECT(anim_frambuf, static char app_trans_anim_buf_a[LV_HOR_RES_MAX * LV_VER_RES_MAX * LV_COLOR_SIZE / 8]);
    APP_L2_NON_RET_BSS_SECT(anim_frambuf, static char app_trans_anim_buf_b[LV_HOR_RES_MAX * LV_VER_RES_MAX * LV_COLOR_SIZE / 8]);
    L2_NON_RET_BSS_SECT_END
#elif defined(APP_TRANS_ANIMATION_OVERWRITE) || defined(APP_TRANS_ANIMATION_NONE)
    /*No trans animtion buf need*/
#elif defined(GUI_APP_FRAMEWORK)
    #error "Need trans animtion buf?"
#else
    /* Non-GUI_APP_FRAMEWORK projects don't need transition buffers. */
#endif /* APP_TRANS_ANIMATION_SCALE_NEXT */

#if PKG_USING_FFMPEG && (MEDIA_CACHE_SIZE > 0)
    APP_L2_RET_BSS_SECT_BEGIN(app_ffmpeg_ret_cache)
    APP_L2_RET_BSS_SECT(app_ffmpeg_ret_cache, ALIGN(4) static uint8_t app_ffmpeg_cache[MEDIA_CACHE_SIZE]);
    APP_L2_RET_BSS_SECT_END
#endif

#if IMAGE_CACHE_IN_PSRAM_SIZE > 0
    struct rt_memheap app_image_psram_memheap;
#endif

#if IMAGE_CACHE_IN_SRAM_SIZE > 0
    struct rt_memheap app_image_sram_memheap;
#endif

#ifndef FREETYPE_CACHE_IN_SRAM
    struct rt_memheap app_ft_memheap;
#endif

#if LV_USE_TINY_TTF && (defined(TINY_TTF_CACHE_IN_SRAM_STANDALONE) || defined(TINY_TTF_CACHE_IN_PSRAM))
    struct rt_memheap app_tiny_ttf_memheap;
#endif

#ifdef QUICKJS_PSRAM_SIZE
    struct rt_memheap app_qjs_memheap;
#endif

#ifndef MESSAGE_BUFFER_IN_SRAM
    struct rt_memheap app_message_memheap;
#endif

#define APP_ANIM_BUF_BYTES ((size_t)LV_HOR_RES_MAX * LV_VER_RES_MAX * LV_COLOR_SIZE / 8U)
#define APP_ANIM_RT_MEMHEAP_SIZE RT_ALIGN(sizeof(struct rt_memheap_item), RT_ALIGN_SIZE)

static struct rt_memheap app_anim_buf_memheap[2];
static void *app_anim_buf_heap_start[2];
static size_t app_anim_buf_heap_size[2];
static bool app_anim_buf_heap_enabled;
static size_t app_anim_buf_alloc_ex_offset;

#if PKG_USING_FFMPEG && (MEDIA_CACHE_SIZE > 0)
static struct rt_memheap app_ffmpeg_memheap;
static bool app_ffmpeg_memheap_ready = false;

static void app_ffmpeg_memheap_init_once(void)
{
    if (app_ffmpeg_memheap_ready)
    {
        return;
    }

    rt_memheap_init(&app_ffmpeg_memheap, "ffmpeg_memheap", (void *)app_ffmpeg_cache, MEDIA_CACHE_SIZE);
    app_ffmpeg_memheap_ready = true;
}
#endif

static bool app_cache_memheap_ready;

static bool app_anim_ptr_is_local(const void *ptr)
{
    uint32_t i;

    if (!app_anim_buf_heap_enabled || !ptr)
    {
        return false;
    }

    for (i = 0; i < 2; i++)
    {
        const uint8_t *start = (const uint8_t *)app_anim_buf_heap_start[i];
        const uint8_t *end;

        if (!start || !app_anim_buf_heap_size[i])
        {
            continue;
        }

        end = start + app_anim_buf_heap_size[i];
        if ((const uint8_t *)ptr >= start && (const uint8_t *)ptr < end)
        {
            return true;
        }
    }

    return false;
}

static int app_anim_ptr_heap_index(const void *ptr)
{
    uint32_t i;

    if (!app_anim_buf_heap_enabled || !ptr)
    {
        return -1;
    }

    for (i = 0; i < 2; i++)
    {
        const uint8_t *start = (const uint8_t *)app_anim_buf_heap_start[i];
        const uint8_t *end;

        if (!start || !app_anim_buf_heap_size[i])
        {
            continue;
        }

        end = start + app_anim_buf_heap_size[i];
        if ((const uint8_t *)ptr >= start && (const uint8_t *)ptr < end)
        {
            return (int)i;
        }
    }

    return -1;
}

static uint32_t app_anim_ptr_size(const void *ptr)
{
    struct rt_memheap_item *header_ptr;

    if (!ptr)
    {
        return 0;
    }

    header_ptr = (struct rt_memheap_item *)((uint8_t *)ptr - APP_ANIM_RT_MEMHEAP_SIZE);
    return (uint32_t)((uint8_t *)header_ptr->next - (uint8_t *)header_ptr - APP_ANIM_RT_MEMHEAP_SIZE);
}

static size_t app_cache_ptr_size(const void *ptr)
{
    uint8_t mem_type;
    uint8_t *raw_ptr;
    size_t raw_size;

    if (!ptr)
    {
        return 0;
    }

    raw_ptr = ((uint8_t *)ptr) - 4;
    mem_type = app_get_mem_type((void *)ptr);
    if (PSRAM_HEAP == mem_type || SRAM_HEAP == mem_type)
    {
        raw_size = rt_heapmem_size(raw_ptr);
    }
    else
    {
        raw_size = rt_mem_size(raw_ptr);
    }

    return raw_size >= 4 ? raw_size - 4 : 0;
}

static bool app_anim_heap_has_used(void)
{
    uint32_t i;

    for (i = 0; i < 2; i++)
    {
        if (app_anim_buf_heap_start[i] && app_anim_buf_memheap[i].actual_used_size)
        {
            return true;
        }
    }

    return false;
}

static uint32_t app_anim_heap_used_size(void)
{
    uint32_t i;
    uint32_t used = 0;

    for (i = 0; i < 2; i++)
    {
        if (app_anim_buf_heap_start[i])
        {
            used += app_anim_buf_memheap[i].actual_used_size;
        }
    }

    return used;
}

static void *app_anim_heap_alloc(size_t size)
{
    uint32_t i;

    for (i = 0; i < 2; i++)
    {
        void *ptr;

        if (!app_anim_buf_heap_start[i])
        {
            continue;
        }

        ptr = rt_memheap_alloc(&app_anim_buf_memheap[i], size);
        if (ptr)
        {
            return ptr;
        }
    }

    return NULL;
}

static void *app_anim_heap_realloc(void *ptr, size_t size)
{
    int idx = app_anim_ptr_heap_index(ptr);

    if (idx < 0)
    {
        return NULL;
    }

    return rt_memheap_realloc(&app_anim_buf_memheap[idx], ptr, size);
}

static void app_anim_heap_free(void *ptr)
{
    if (app_anim_ptr_heap_index(ptr) < 0)
    {
        return;
    }

    rt_memheap_free(ptr);
}

static int app_anim_memheap_enable(void)
{
    uint32_t i;

    if (app_anim_buf_heap_enabled)
    {
        return 0;
    }

    for (i = 0; i < 2; i++)
    {
        void *buf = app_anim_buf_alloc(APP_ANIM_BUF_BYTES, (uint8_t)i);
        char heap_name[10];

        if (!buf)
        {
            break;
        }

        rt_snprintf(heap_name, sizeof(heap_name), "anim%u", (unsigned int)i);
        rt_memheap_init(&app_anim_buf_memheap[i], heap_name, buf, APP_ANIM_BUF_BYTES);
        app_anim_buf_heap_start[i] = buf;
        app_anim_buf_heap_size[i] = APP_ANIM_BUF_BYTES;
    }

    if (!app_anim_buf_heap_start[0])
    {
        rt_kprintf("app_anim_buf_set_as_memheap: no animation buffer available\n");
        return -RT_ERROR;
    }

    app_anim_buf_heap_enabled = true;
    rt_kprintf("app_anim_buf_set_as_memheap: enable start0=%p size0=%u start1=%p size1=%u\n",
               app_anim_buf_heap_start[0],
               (unsigned int)app_anim_buf_heap_size[0],
               app_anim_buf_heap_start[1],
               (unsigned int)app_anim_buf_heap_size[1]);
    return 0;
}

static int app_anim_memheap_disable(void)
{
    uint32_t i;
    uint32_t wait_count = 5;

    if (!app_anim_buf_heap_enabled)
    {
        rt_kprintf("app_anim_buf_set_as_memheap: animation buffer not initialized\n");
        return -RT_ERROR;
    }

    while (app_anim_heap_has_used() && wait_count > 0)
    {
        rt_thread_mdelay(LV_DISP_DEF_REFR_PERIOD);
        wait_count--;
    }

    if (app_anim_heap_has_used())
    {
        rt_kprintf("app_anim_buf_set_as_memheap: disable busy used=%u\n",
                   (unsigned int)app_anim_heap_used_size());
        return -RT_EBUSY;
    }

    for (i = 0; i < 2; i++)
    {
        if (app_anim_buf_heap_start[i])
        {
            rt_memheap_detach(&app_anim_buf_memheap[i]);
        }
        app_anim_buf_heap_start[i] = NULL;
        app_anim_buf_heap_size[i] = 0;
    }

    app_anim_buf_heap_enabled = false;
    rt_kprintf("app_anim_buf_set_as_memheap: disable\n");
    return 0;
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/


static int app_cahe_memheap_init(void)
{
    if (app_cache_memheap_ready)
    {
        return 0;
    }

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

#if LV_USE_TINY_TTF && (defined(TINY_TTF_CACHE_IN_SRAM_STANDALONE) || defined(TINY_TTF_CACHE_IN_PSRAM))
    rt_memheap_init(&app_tiny_ttf_memheap, "app_tiny_ttf_memheap", (void *)app_tiny_ttf_cache, TINY_TTF_CACHE_SIZE);
#endif

#ifdef QUICKJS_PSRAM_SIZE
    rt_memheap_init(&app_qjs_memheap, "app_qjs_memheap", (void *)app_qjs_cache, QUICKJS_PSRAM_SIZE);
#endif

#if PKG_USING_FFMPEG && (MEDIA_CACHE_SIZE > 0)
    app_ffmpeg_memheap_init_once();
#endif

    app_cache_memheap_ready = true;
    return 0;
}
INIT_PREV_EXPORT(app_cahe_memheap_init);

int app_memheap_init(void)
{
    return app_cahe_memheap_init();
}

void app_mem_check(void)
{
}


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

void *app_cache_realloc(void *memory, size_t new_size, image_cache_t cache_type)
{
    void *new_memory;
    uint32_t old_size;

    if (!memory)
    {
        return app_cache_alloc(new_size, cache_type);
    }

    if (new_size == 0)
    {
        app_cache_free(memory);
        return NULL;
    }

    old_size = app_mem_get_size(memory);
    if (old_size >= new_size)
    {
        return memory;
    }

    new_memory = app_cache_alloc(new_size, cache_type);
    if (new_memory)
    {
        rt_memcpy(new_memory, memory, old_size);
        app_cache_free(memory);
    }

    return new_memory;
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
static inline uint32_t lv_img_buf_get_img_size(lv_coord_t w, lv_coord_t h, uint8_t cf)
{
    return lv_draw_buf_width_to_stride(w, cf) * h;
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

int app_anim_buf_set_as_memheap(uint8_t enable)
{
    if (enable)
    {
        return app_anim_memheap_enable();
    }

    return app_anim_memheap_disable();
}

void *app_anim_alloc(size_t size)
{
    void *ptr = NULL;

    if (size == 0)
    {
        return NULL;
    }

    if (app_anim_buf_heap_enabled)
    {
        ptr = app_anim_heap_alloc(size);
    }

    if (!ptr)
    {
        ptr = app_cache_alloc(size, CACHE_PSRAM);
    }

    return ptr;
}

void *app_anim_realloc(void *ptr, size_t size)
{
    void *new_ptr;
    size_t old_size;

    if (!ptr)
    {
        return app_anim_alloc(size);
    }

    if (size == 0)
    {
        app_anim_free(ptr);
        return NULL;
    }

    old_size = app_mem_get_size(ptr);
    if (old_size >= size)
    {
        return ptr;
    }

    if (app_anim_ptr_is_local(ptr) && app_anim_buf_heap_enabled)
    {
        new_ptr = app_anim_heap_realloc(ptr, size);
        if (new_ptr)
        {
            return new_ptr;
        }
    }

    new_ptr = app_anim_alloc(size);
    if (new_ptr)
    {
        rt_memcpy(new_ptr, ptr, old_size);
        app_anim_free(ptr);
    }

    return new_ptr;
}

void app_anim_free(void *ptr)
{
    if (!ptr)
    {
        return;
    }

    if (app_anim_ptr_is_local(ptr))
    {
        app_anim_heap_free(ptr);
    }
    else
    {
        app_cache_free(ptr);
    }
}

char *app_anim_strdup(const char *s)
{
    char *ptr = NULL;

    if (s)
    {
        ptr = app_anim_alloc(strlen(s) + 1);
        if (ptr)
        {
            strcpy(ptr, s);
        }
    }

    return ptr;
}

uint32_t app_mem_get_size(void *ptr)
{
    if (!ptr)
    {
        return 0;
    }

    if (app_anim_ptr_is_local(ptr))
    {
        return app_anim_ptr_size(ptr);
    }

    return (uint32_t)app_cache_ptr_size(ptr);
}

void app_mem_insert_asyn_node(void *ptr, void (*free_fun)(void *))
{
#ifdef MEM_ASYN_FREE
    mem_async_node_t *node;

    if (!ptr || !free_fun)
    {
        return;
    }

    node = lv_mem_alloc(sizeof(mem_async_node_t));
    RT_ASSERT(node);

    rt_mutex_take(&mem_asyn_mutex, RT_WAITING_FOREVER);
    node->ptr = ptr;
    node->free = free_fun;
    rt_list_insert_before(&app_mem_async_list, &node->list);
    rt_mutex_release(&mem_asyn_mutex);
#else
    (void)ptr;
    (void)free_fun;
#endif
}

void app_mem_free_asyn_node(void)
{
#ifdef MEM_ASYN_FREE
    rt_list_t *pos, *n;

    rt_mutex_take(&mem_asyn_mutex, RT_WAITING_FOREVER);
    rt_list_for_each_safe(pos, n, (&app_mem_async_list))
    {
        mem_async_node_t *node = rt_list_entry(pos, mem_async_node_t, list);
        struct rt_memheap_item *header = app_mem_get_async_header(node->ptr);

        if (!header || 0 == header->ref_count)
        {
            rt_list_remove(pos);
            node->free(node->ptr);
            lv_mem_free(node);
        }
    }
    rt_mutex_release(&mem_asyn_mutex);
#endif
}

void app_mem_set_ref_count(void *ptr, int ref_count, int type)
{
#ifdef MEM_ASYN_FREE
    struct rt_memheap_item *header;
    int16_t next_count;

    if (MEM_ASYN_FONT != type)
    {
        return;
    }

    header = app_mem_get_async_header(ptr);
    if (!header)
    {
        return;
    }

    rt_mutex_take(&mem_asyn_mutex, RT_WAITING_FOREVER);
    next_count = (int16_t)header->ref_count + ref_count;
    RT_ASSERT(next_count >= 0);
    header->ref_count = (rt_uint16_t)next_count;
    rt_mutex_release(&mem_asyn_mutex);
#else
    (void)ptr;
    (void)ref_count;
    (void)type;
#endif
}

static int mem_asyn_list_init(void)
{
#ifdef MEM_ASYN_FREE
    rt_list_init(&app_mem_async_list);
    rt_mutex_init(&mem_asyn_mutex, "mem_asyn", RT_IPC_FLAG_FIFO);
#endif
    return 0;
}
INIT_PREV_EXPORT(mem_asyn_list_init);


void *app_anim_buf_alloc(size_t nbytes, uint8_t index)
{
    void *ptr = NULL;

    if (nbytes == 0 || nbytes > APP_ANIM_BUF_BYTES)
    {
        rt_kprintf("app_anim_buf_alloc: invalid size %d max %d\n", nbytes, APP_ANIM_BUF_BYTES);
        return NULL;
    }

#ifdef APP_TRANS_ANIMATION_SCALE_NEXT
    if (0 == index) ptr = &app_trans_anim_buf_a;
#elif defined(APP_TRANS_ANIMATION_SCALE) || defined(SIFLI_APP_MEM_REUSE_TRANS_ANIM_BUF)
    if (0 == index) ptr = &app_trans_anim_buf_a;
    if (1 == index) ptr = &app_trans_anim_buf_b;
#elif defined(APP_TRANS_ANIMATION_OVERWRITE) || defined(APP_TRANS_ANIMATION_NONE)
    ptr = NULL;
#else

#endif /* APP_TRANS_ANIMATION_SCALE_NEXT */

    rt_kprintf("app_anim_buf_alloc: %p index %d size %d\n", ptr, index, nbytes);
    return ptr;
}

void *app_anim_buf_alloc_ex(size_t nbytes, uint8_t first)
{
    uint8_t *buf_a;
    uint8_t *buf_b;
    size_t offset;
    size_t total_size;

    if (first)
    {
        app_anim_buf_alloc_ex_offset = 0;
    }

    if (nbytes == 0)
    {
        return NULL;
    }

    buf_a = app_anim_buf_alloc(APP_ANIM_BUF_BYTES, 0);
    buf_b = app_anim_buf_alloc(APP_ANIM_BUF_BYTES, 1);
    total_size = APP_ANIM_BUF_BYTES;
    if (buf_b)
    {
        total_size += APP_ANIM_BUF_BYTES;
    }

    offset = app_anim_buf_alloc_ex_offset;
    if (offset < APP_ANIM_BUF_BYTES)
    {
        if (offset + nbytes <= APP_ANIM_BUF_BYTES)
        {
            app_anim_buf_alloc_ex_offset = offset + nbytes;
            return buf_a + offset;
        }
        offset = APP_ANIM_BUF_BYTES;
    }

    if (!buf_b || offset + nbytes > total_size)
    {
        rt_kprintf("app_anim_buf_alloc_ex: size %d offset %d total %d\n",
                   nbytes,
                   offset,
                   total_size);
        return NULL;
    }

    app_anim_buf_alloc_ex_offset = offset + nbytes;
    return buf_b + (offset - APP_ANIM_BUF_BYTES);
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

void app_mem_invalid_icache(void *data, uint32_t size)
{
#ifndef WIN32
    if (PSRAM_HEAP == app_get_mem_type(data))
        SCB_InvalidateICache_by_Addr(data, (size + 3) >> 2 << 2);
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

void ffmpeg_heap_init(void)
{
#if MEDIA_CACHE_SIZE > 0
    app_ffmpeg_memheap_init_once();
#else
    /* app_mem heaps are initialized during app_cahe_memheap_init(). */
#endif
}

void *ffmpeg_alloc(size_t nbytes)
{
    uint8_t *p;
    ffmpeg_mem_header *header_p;

#if MEDIA_CACHE_SIZE > 0
    app_ffmpeg_memheap_init_once();
#endif

    if (nbytes > ALIGN64_SIZE_THRESHOLD)
    {
        size_t header_size = 63 + FFMPEG_MEM_HEADER;
#if MEDIA_CACHE_SIZE > 0
        p = (uint8_t *)rt_memheap_alloc(&app_ffmpeg_memheap, nbytes + header_size);
#else
        p = app_anim_mem_alloc(nbytes + header_size, 1);
#endif
        if (!p) return NULL;

        header_p = (ffmpeg_mem_header *)(RT_ALIGN_DOWN((uint32_t)(p + header_size), 64) - FFMPEG_MEM_HEADER);

        RT_ASSERT(((uint32_t)header_p) >= ((uint32_t)p));
    }
    else
    {
#if MEDIA_CACHE_SIZE > 0
        p = (uint8_t *)rt_memheap_alloc(&app_ffmpeg_memheap, nbytes + FFMPEG_MEM_HEADER);
#else
        p = app_anim_mem_alloc(nbytes + FFMPEG_MEM_HEADER, 1);
#endif
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
#if MEDIA_CACHE_SIZE > 0
    rt_memheap_free(((uint8_t *)p) - header_p->offset);
#else
    app_anim_mem_free(((uint8_t *)p) - header_p->offset);
#endif
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

void *audio_mem_malloc(uint32_t size)
{
    void *ptr = ffmpeg_alloc(size);
    RT_ASSERT(ptr);
    return ptr;
}

void audio_mem_free(void *ptr)
{
    ffmpeg_free(ptr);
}

void *audio_mem_calloc(uint32_t count, uint32_t size)
{
    void *ptr = ffmpeg_alloc(count * size);
    RT_ASSERT(ptr);
    memset(ptr, 0, count * size);
    return ptr;
}

void *audio_mem_recalloc(void *p, size_t new_size)
{
    return ffmpeg_realloc(p, new_size);
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

#if LV_USE_TINY_TTF
void * app_tiny_ttf_mem_alloc(size_t size)
{
#if defined(TINY_TTF_CACHE_IN_SRAM_STANDALONE) || defined(TINY_TTF_CACHE_IN_PSRAM)
    return rt_memheap_alloc(&app_tiny_ttf_memheap, size);
#else
    return rt_malloc(size);
#endif
}

void app_tiny_ttf_mem_free(void *buf)
{
#if defined(TINY_TTF_CACHE_IN_SRAM_STANDALONE) || defined(TINY_TTF_CACHE_IN_PSRAM)
    rt_memheap_free(buf);
#else
    rt_free(buf);
#endif
}
#endif


/**********************
 *   STATIC FUNCTIONS
 **********************/
