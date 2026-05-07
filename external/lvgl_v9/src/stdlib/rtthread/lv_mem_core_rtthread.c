/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lv_malloc_core_rtthread.c
 */

/*********************
 *      INCLUDES
 *********************/
#include "../lv_mem.h"
#if LV_USE_STDLIB_MALLOC == LV_STDLIB_RTTHREAD
#include "../../stdlib/lv_mem.h"
#include <rtthread.h>
#if defined(BSP_USING_PSRAM)
    #include "bf0_hal.h"
    #include "mem_section.h"
#endif

#ifndef RT_USING_HEAP
    #error "lv_mem_core_rtthread: RT_USING_HEAP is required. Define it in rtconfig.h"
#endif

#if defined(BSP_USING_PSRAM) && defined(LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP) && \
    defined(LV_RTTHREAD_PSRAM_HEAP_SIZE_KILOBYTES) && \
    (LV_RTTHREAD_PSRAM_HEAP_SIZE_KILOBYTES > 0)
    #define LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME 1
#else
    #define LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME 0
#endif

/*********************
 *      DEFINES
 *********************/
#if LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME
    #define LV_PSRAM_HEAP_BYTES ((rt_ubase_t)LV_RTTHREAD_PSRAM_HEAP_SIZE_KILOBYTES * 1024U)
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
#if LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME
    L2_NON_RET_BSS_SECT_BEGIN(lvgl_heap)
    L2_NON_RET_BSS_SECT(lvgl_heap, ALIGN(RT_ALIGN_SIZE) static uint8_t lv_psram_heap_buffer[LV_PSRAM_HEAP_BYTES]);
    L2_NON_RET_BSS_SECT_END

    static struct rt_memheap lv_psram_heap;
    static rt_bool_t lv_psram_heap_ready = RT_FALSE;
    static rt_ubase_t lv_psram_heap_begin = 0;
    static rt_ubase_t lv_psram_heap_end = 0;
    static uint32_t lv_psram_alloc_cnt = 0;
    static uint32_t lv_psram_fallback_cnt = 0;
    static rt_bool_t lv_psram_fallback_logged = RT_FALSE;
#endif

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_mem_init(void)
{
#if LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME
    if (LV_PSRAM_HEAP_BYTES >= 1024U)
    {
        lv_psram_heap_begin = (rt_ubase_t)lv_psram_heap_buffer;
        lv_psram_heap_end = lv_psram_heap_begin + LV_PSRAM_HEAP_BYTES;

        if (rt_memheap_init(&lv_psram_heap, "lv_psram", lv_psram_heap_buffer, LV_PSRAM_HEAP_BYTES) == RT_EOK)
        {
            lv_psram_heap_ready = RT_TRUE;
            rt_kprintf("LV_MEM_RT: psram heap enabled [%08x,%08x), size=%uKB\n",
                       (unsigned int)lv_psram_heap_begin, (unsigned int)lv_psram_heap_end,
                       (unsigned int)(LV_PSRAM_HEAP_BYTES >> 10));
        }
    }

    if (!lv_psram_heap_ready)
    {
        rt_kprintf("LV_MEM_RT: psram heap disabled at runtime, buf=%08x, size=%uKB\n",
                   (unsigned int)lv_psram_heap_begin, (unsigned int)(LV_PSRAM_HEAP_BYTES >> 10));
    }
#else
    rt_kprintf("LV_MEM_RT: using rt_malloc path (psram private heap build condition not met)\n");
#endif
    return;
}

void lv_mem_deinit(void)
{
#if LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME
    if (lv_psram_heap_ready)
    {
        rt_kprintf("LV_MEM_RT: psram summary used=%u/%u\n",
                   (unsigned int)(lv_psram_heap.pool_size - lv_psram_heap.available_size),
                   (unsigned int)lv_psram_heap.pool_size);
    }
#endif
    return; /*Nothing to deinit*/
}

lv_mem_pool_t lv_mem_add_pool(void * mem, size_t bytes)
{
    /*Not supported*/
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    /*Not supported*/
    LV_UNUSED(pool);
    return;
}

void * lv_malloc_core(size_t size)
{
#if LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME
    if (lv_psram_heap_ready)
    {
        void *p = rt_memheap_alloc(&lv_psram_heap, (rt_size_t)size);
        if (p != RT_NULL)
        {
            lv_psram_alloc_cnt++;
            if ((lv_psram_alloc_cnt & 0x1FFU) == 0U)
            {
                // rt_kprintf("LV_MEM_RT: psram used=%u/%u fallback=%u\n",
                //            (unsigned int)(lv_psram_heap.pool_size - lv_psram_heap.available_size),
                //            (unsigned int)lv_psram_heap.pool_size, (unsigned int)lv_psram_fallback_cnt);
            }
            return p;
        }

        lv_psram_fallback_cnt++;
        if (!lv_psram_fallback_logged)
        {
            rt_kprintf("LV_MEM_RT: psram alloc fallback to rt_malloc, req=%u\n", (unsigned int)size);
            lv_psram_fallback_logged = RT_TRUE;
        }
    }
#endif
    return rt_malloc(size);
}

void * lv_realloc_core(void * p, size_t new_size)
{
#if LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME
    if (p == RT_NULL) return lv_malloc_core(new_size);

    if (lv_psram_heap_ready)
    {
        rt_ubase_t addr = (rt_ubase_t)p;
        if ((addr >= lv_psram_heap_begin) && (addr < lv_psram_heap_end))
        {
            return rt_memheap_realloc(&lv_psram_heap, p, (rt_size_t)new_size);
        }
    }
#endif
    return rt_realloc(p, new_size);
}

void lv_free_core(void * p)
{
#if LV_RTTHREAD_USE_PSRAM_PRIVATE_HEAP_RUNTIME
    if ((p != RT_NULL) && lv_psram_heap_ready)
    {
        rt_ubase_t addr = (rt_ubase_t)p;
        if ((addr >= lv_psram_heap_begin) && (addr < lv_psram_heap_end))
        {
            rt_memheap_free(p);
            return;
        }
    }
#endif
    rt_free(p);
}

void lv_mem_monitor_core(lv_mem_monitor_t * mon_p)
{
    /*Not supported*/
    LV_UNUSED(mon_p);
    return;
}

lv_result_t lv_mem_test_core(void)
{
    /*Not supported*/
    return LV_RESULT_OK;
}

#endif /* LV_USE_STDLIB_MALLOC == LV_STDLIB_RTTHREAD */
