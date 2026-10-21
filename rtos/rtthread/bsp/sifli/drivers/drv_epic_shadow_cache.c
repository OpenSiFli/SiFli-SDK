/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "drv_epic.h"
#include "drv_epic_shadow_cache.h"
#include "drv_epic_private.h"
#include "mem_section.h"
#include <stdbool.h>
#include "string.h"

/* Set to 1 to enable counters and the epic_shadow_cache_stat FinSH command. */
#ifndef DRV_EPIC_SHADOW_CACHE_DEBUG
#define DRV_EPIC_SHADOW_CACHE_DEBUG 0
#endif

#if DRV_EPIC_SHADOW_CACHE_DEBUG && defined(RT_USING_FINSH)
#include <finsh.h>
#endif

#define SHADOW_CACHE_ENTRY_COUNT 8       /* Maximum number of cached shapes. */
#define SHADOW_CACHE_MAX_BYTES   (16 * 1024) /* Fixed SRAM pool size. */

/* A cache entry stores an offset because the static pool may be compacted. */
typedef struct
{
    drv_epic_shadow_cache_key_t key;
    size_t offset;
    size_t bytes;
    uint32_t ref_count;
    uint32_t age;
    bool valid;
} shadow_cache_entry_t;

static shadow_cache_entry_t cache[SHADOW_CACHE_ENTRY_COUNT];
static struct rt_mutex cache_mutex;
static size_t cache_bytes;
static uint32_t cache_age;

#if DRV_EPIC_SHADOW_CACHE_DEBUG
typedef struct
{
    uint32_t hits;                  /* Requests served by an existing cached mask. */
    uint32_t inserts;               /* Masks generated and stored in the static pool. */
    uint32_t fallback_oversize;     /* Heap fallbacks for masks larger than the static pool. */
    uint32_t fallback_busy;         /* Heap fallbacks while cached masks are in use. */
    uint32_t fallback_capacity;     /* Heap fallbacks when no static cache slot is available. */
    uint32_t heap_alloc_failures;   /* Failed heap allocations for temporary masks. */
    uint32_t generate_failures;     /* Failed attempts to generate a shadow mask. */
} shadow_cache_stats_t;

static shadow_cache_stats_t cache_stats;
#endif

/* Keep generated masks in a fixed SRAM pool, independent of render-list lifetime. */
L1_NON_RET_BSS_SECT_BEGIN(drv_epic_shadow_cache)
L1_NON_RET_BSS_SECT(drv_epic_shadow_cache, ALIGN(RT_ALIGN_SIZE) static uint8_t cache_data[SHADOW_CACHE_MAX_BYTES]);
L1_NON_RET_BSS_SECT_END

static bool key_equal(const drv_epic_shadow_cache_key_t *a, const drv_epic_shadow_cache_key_t *b)
{
    return a->width == b->width && a->radius == b->radius &&
           a->core_width == b->core_width && a->core_height == b->core_height &&
           a->size == b->size;
}

/* Remove an unused entry; its storage is reclaimed by compact_cache(). */
static void evict_entry(shadow_cache_entry_t *entry)
{
    RT_ASSERT(entry->ref_count == 0);
    cache_bytes -= entry->bytes;
    memset(entry, 0, sizeof(*entry));
}

/* Pack live entries so a new mask can occupy one contiguous tail region. */
static void compact_cache(void)
{
    size_t scan_offset = 0;
    size_t write_offset = 0;

    for (size_t n = 0; n < SHADOW_CACHE_ENTRY_COUNT; n++)
    {
        shadow_cache_entry_t *next = NULL;
        for (size_t i = 0; i < SHADOW_CACHE_ENTRY_COUNT; i++)
        {
            shadow_cache_entry_t *entry = &cache[i];
            if (entry->valid && entry->offset >= scan_offset &&
                    (!next || entry->offset < next->offset))
                next = entry;
        }
        if (!next) break;

        size_t old_offset = next->offset;
        memmove(&cache_data[write_offset], &cache_data[old_offset], next->bytes);
        next->offset = write_offset;
        write_offset += next->bytes;
        scan_offset = old_offset + next->bytes;
    }
    cache_bytes = write_offset;
}

/* Initialize the mutex before the first cache request. */
static int shadow_cache_init(void)
{
    rt_err_t err = rt_mutex_init(&cache_mutex, "epicshdw", RT_IPC_FLAG_FIFO);
    RT_ASSERT(err == RT_EOK);
    return err;
}
INIT_APP_EXPORT(shadow_cache_init);

/* Acquire a shared cached mask or a temporary heap-backed mask. */
rt_err_t drv_epic_shadow_cache_acquire(const drv_epic_shadow_cache_key_t *key,
                                       size_t bytes,
                                       drv_epic_shadow_cache_generate_cb generate,
                                       void *user_data,
                                       uint8_t **mask)
{
    RT_ASSERT(key != NULL && generate != NULL && mask != NULL && bytes > 0);

    rt_err_t err = rt_mutex_take(&cache_mutex, RT_WAITING_FOREVER);
    RT_ASSERT(err == RT_EOK);

    for (size_t i = 0; i < SHADOW_CACHE_ENTRY_COUNT; i++)
    {
        shadow_cache_entry_t *entry = &cache[i];
        if (entry->valid && entry->bytes == bytes && key_equal(&entry->key, key))
        {
#if DRV_EPIC_SHADOW_CACHE_DEBUG
            cache_stats.hits++;
#endif
            entry->ref_count++;
            entry->age = ++cache_age;
            *mask = &cache_data[entry->offset];
            rt_mutex_release(&cache_mutex);
            return RT_EOK;
        }
    }

    bool cache_in_use = false;
    for (size_t i = 0; i < SHADOW_CACHE_ENTRY_COUNT; i++)
    {
        if (cache[i].valid && cache[i].ref_count != 0)
        {
            cache_in_use = true;
            break;
        }
    }

#if DRV_EPIC_SHADOW_CACHE_DEBUG
    if (bytes > SHADOW_CACHE_MAX_BYTES)
        cache_stats.fallback_oversize++;
    else if (cache_in_use)
        cache_stats.fallback_busy++;
#endif

    if (!cache_in_use && bytes <= SHADOW_CACHE_MAX_BYTES)
    {
        shadow_cache_entry_t *slot = NULL;
        while (cache_bytes + bytes > SHADOW_CACHE_MAX_BYTES || !slot)
        {
            slot = NULL;
            shadow_cache_entry_t *oldest = NULL;
            for (size_t i = 0; i < SHADOW_CACHE_ENTRY_COUNT; i++)
            {
                shadow_cache_entry_t *entry = &cache[i];
                if (!entry->valid)
                {
                    if (!slot) slot = entry;
                }
                else if (entry->ref_count == 0 && (!oldest || entry->age < oldest->age))
                {
                    oldest = entry;
                }
            }

            if (cache_bytes + bytes <= SHADOW_CACHE_MAX_BYTES && slot) break;
            if (!oldest) break;
            evict_entry(oldest);
        }

        if (slot && cache_bytes + bytes <= SHADOW_CACHE_MAX_BYTES)
        {
            compact_cache();
            slot = NULL;
            for (size_t i = 0; i < SHADOW_CACHE_ENTRY_COUNT; i++)
            {
                if (!cache[i].valid)
                {
                    slot = &cache[i];
                    break;
                }
            }
            RT_ASSERT(slot != NULL);
            uint8_t *data = &cache_data[cache_bytes];
            err = generate(data, user_data);
            if (err == RT_EOK)
            {
                slot->key = *key;
                slot->offset = cache_bytes;
                slot->bytes = bytes;
                slot->ref_count = 1;
                slot->age = ++cache_age;
                slot->valid = true;
                cache_bytes += bytes;
#if DRV_EPIC_SHADOW_CACHE_DEBUG
                cache_stats.inserts++;
#endif
                *mask = data;
                rt_mutex_release(&cache_mutex);
                return RT_EOK;
            }
#if DRV_EPIC_SHADOW_CACHE_DEBUG
            cache_stats.generate_failures++;
#endif
            rt_mutex_release(&cache_mutex);
            return err;
        }
    }

#if DRV_EPIC_SHADOW_CACHE_DEBUG
    if (!cache_in_use && bytes <= SHADOW_CACHE_MAX_BYTES)
        cache_stats.fallback_capacity++;
#endif
    uint8_t *temporary = epic_malloc(bytes);
    if (!temporary)
    {
#if DRV_EPIC_SHADOW_CACHE_DEBUG
        cache_stats.heap_alloc_failures++;
#endif
        rt_mutex_release(&cache_mutex);
        return -RT_ENOMEM;
    }
    rt_mutex_release(&cache_mutex);

    err = generate(temporary, user_data);
    if (err != RT_EOK)
    {
        epic_free(temporary);
#if DRV_EPIC_SHADOW_CACHE_DEBUG
        rt_err_t lock_err = rt_mutex_take(&cache_mutex, RT_WAITING_FOREVER);
        RT_ASSERT(lock_err == RT_EOK);
        cache_stats.generate_failures++;
        rt_mutex_release(&cache_mutex);
#endif
        return err;
    }

    *mask = temporary;
    return RT_EOK;
}

/* Drop a cache reference or free a temporary mask. */
void drv_epic_shadow_cache_release(uint8_t *mask)
{
    if (!mask) return;

    rt_err_t err = rt_mutex_take(&cache_mutex, RT_WAITING_FOREVER);
    RT_ASSERT(err == RT_EOK);

    for (size_t i = 0; i < SHADOW_CACHE_ENTRY_COUNT; i++)
    {
        shadow_cache_entry_t *entry = &cache[i];
        if (entry->valid && &cache_data[entry->offset] == mask)
        {
            RT_ASSERT(entry->ref_count > 0);
            entry->ref_count--;
            rt_mutex_release(&cache_mutex);
            return;
        }
    }

    rt_mutex_release(&cache_mutex);
    epic_free(mask);
}

#if DRV_EPIC_SHADOW_CACHE_DEBUG && defined(RT_USING_FINSH)
/* Print cache hits, insertions, heap fallback reasons, and current SRAM use. */
static void epic_shadow_cache_stat(void)
{
    shadow_cache_stats_t stats;
    size_t used_bytes;
    uint32_t entries = 0;
    rt_err_t err = rt_mutex_take(&cache_mutex, RT_WAITING_FOREVER);
    RT_ASSERT(err == RT_EOK);

    stats = cache_stats;
    used_bytes = cache_bytes;
    for (size_t i = 0; i < SHADOW_CACHE_ENTRY_COUNT; i++)
        if (cache[i].valid) entries++;

    rt_mutex_release(&cache_mutex);

    rt_kprintf("EPIC shadow cache: %u/%u bytes, %u/%u entries\n",
               (unsigned)used_bytes, (unsigned)SHADOW_CACHE_MAX_BYTES,
               (unsigned)entries, (unsigned)SHADOW_CACHE_ENTRY_COUNT);
    rt_kprintf("hits=%u inserts=%u heap_fallback(oversize=%u busy=%u capacity=%u) "
               "heap_alloc_fail=%u generate_fail=%u\n",
               (unsigned)stats.hits, (unsigned)stats.inserts,
               (unsigned)stats.fallback_oversize, (unsigned)stats.fallback_busy,
               (unsigned)stats.fallback_capacity, (unsigned)stats.heap_alloc_failures,
               (unsigned)stats.generate_failures);
}
MSH_CMD_EXPORT(epic_shadow_cache_stat, show EPIC shadow mask cache statistics);
#endif
