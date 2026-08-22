/**
 * @file dfu_engine_flash.c
 * @brief DFU V2 Engine — Flash Abstraction Layer Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include "dfu_engine_flash.h"

/* SDK HAL flash driver */
#include "board.h"
#include "drv_flash.h"

#define LOG_TAG "dfu.ef"
#include <log.h>

/*============================================================================
 * Constants
 *============================================================================*/

#define NAND_PAGESIZE   2048
#define NAND_BLK_SIZE   0x20000     /* 128KB */

/*============================================================================
 * NAND Page Cache
 *============================================================================*/

typedef struct
{
    uint32_t page_addr;             /**< Aligned page address */
    uint8_t  buf[NAND_PAGESIZE];    /**< Page buffer */
    uint32_t used;                  /**< Bytes filled in buf */
    bool     dirty;                 /**< Has unflushed data */
} nand_cache_t;

/*============================================================================
 * Internal State
 *============================================================================*/

static struct
{
    bool         initialized;
    bool         target_is_nand;
    bool         cache_active;
    nand_cache_t cache;
} g_ef;

/*============================================================================
 * Flash Type Detection
 *============================================================================*/

/* Addr2Handle: returns non-NULL for NOR, NULL for NAND */
extern void *Addr2Handle(uint32_t addr);

static bool is_nand(uint32_t addr)
{
    return (Addr2Handle(addr) == NULL);
}

/*============================================================================
 * Internal: NAND Cache
 *============================================================================*/

static rt_err_t cache_flush(void)
{
    nand_cache_t *c = &g_ef.cache;
    if (!c->dirty)
        return RT_EOK;

    /* Pad remainder with 0xFF */
    if (c->used < NAND_PAGESIZE)
        memset(c->buf + c->used, 0xFF, NAND_PAGESIZE - c->used);

    int ret = rt_nand_write_page(c->page_addr, c->buf, NAND_PAGESIZE, NULL, 0);
    if (ret != NAND_PAGESIZE)
    {
        LOG_E("NAND write_page failed: addr=0x%08x ret=%d", c->page_addr, ret);
        return -RT_ERROR;
    }

    c->dirty = false;
    c->used  = 0;
    return RT_EOK;
}

static int nand_cached_write(uint32_t addr, const uint8_t *data, uint32_t size)
{
    nand_cache_t *c = &g_ef.cache;
    uint32_t written = 0;

    while (written < size)
    {
        uint32_t page_start = (addr + written) & ~(NAND_PAGESIZE - 1);

        /* If targeting a different page, flush current */
        if (c->dirty && c->page_addr != page_start)
        {
            if (cache_flush() != RT_EOK)
                return -1;
        }

        /* Start fresh page if needed */
        if (!c->dirty)
        {
            c->page_addr = page_start;
            c->used = 0;
            memset(c->buf, 0xFF, NAND_PAGESIZE);
        }

        uint32_t space = NAND_PAGESIZE - c->used;
        uint32_t chunk = (size - written > space) ? space : (size - written);

        memcpy(c->buf + c->used, data + written, chunk);
        c->used += chunk;
        c->dirty = true;
        written += chunk;

        /* Auto-flush full page */
        if (c->used == NAND_PAGESIZE)
        {
            if (cache_flush() != RT_EOK)
                return -1;
        }
    }

    return (int)written;
}

/*============================================================================
 * Lifecycle
 *============================================================================*/

dfu_engine_err_t dfu_ef_init(void)
{
    if (g_ef.initialized)
        return DFU_ENGINE_ERR_NONE;

    memset(&g_ef, 0, sizeof(g_ef));
    g_ef.initialized = true;
    return DFU_ENGINE_ERR_NONE;
}

void dfu_ef_deinit(void)
{
    if (!g_ef.initialized)
        return;

    if (g_ef.cache_active)
    {
        cache_flush();
        g_ef.cache_active = false;
    }

    g_ef.initialized = false;
}

/*============================================================================
 * Flash Operations
 *============================================================================*/

dfu_engine_err_t dfu_ef_erase(uint32_t addr, uint32_t size)
{
    LOG_I("erase: addr=0x%08x size=%u", addr, size);

    if (is_nand(addr))
    {
        /* NAND: block-aligned erase */
        uint32_t aligned_addr = addr & ~(NAND_BLK_SIZE - 1);
        int erase_align = rt_flash_get_erase_alignment(addr);
        if (erase_align <= 0)
            erase_align = NAND_BLK_SIZE;
        uint32_t aligned_size = ((size + erase_align - 1) / erase_align) * erase_align;
        rt_nand_erase(aligned_addr, aligned_size);
    }
    else
    {
        /* NOR: sector-aligned erase */
        int erase_align = rt_flash_get_erase_alignment(addr);
        if (erase_align <= 0)
            erase_align = 4096;
        uint32_t aligned_size = ((size + erase_align - 1) / erase_align) * erase_align;
        rt_err_t ret = rt_flash_erase(addr, aligned_size);
        if (ret != RT_EOK)
        {
            LOG_E("NOR erase failed: addr=0x%08x ret=%d", addr, ret);
            return DFU_ENGINE_ERR_FLASH_ERASE;
        }
    }

    return DFU_ENGINE_ERR_NONE;
}

dfu_engine_err_t dfu_ef_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (is_nand(addr))
    {
        if (!g_ef.cache_active)
        {
            LOG_E("NAND write without cache_init!");
            return DFU_ENGINE_ERR_FLASH_WRITE;
        }
        int ret = nand_cached_write(addr, data, len);
        if (ret < 0)
            return DFU_ENGINE_ERR_FLASH_WRITE;
    }
    else
    {
        int ret = rt_flash_write(addr, data, len);
        if (ret < 0 || ret != (int)len)
        {
            LOG_E("NOR write failed: addr=0x%08x len=%u ret=%d", addr, len, ret);
            return DFU_ENGINE_ERR_FLASH_WRITE;
        }
    }

    return DFU_ENGINE_ERR_NONE;
}

dfu_engine_err_t dfu_ef_read(uint32_t addr, uint8_t *data, uint32_t len)
{
    int ret;

    if (is_nand(addr))
        ret = rt_nand_read(addr, data, len);
    else
        ret = rt_flash_read(addr, data, len);

    if (ret < 0 || ret != (int)len)
    {
        LOG_E("read failed: addr=0x%08x len=%u ret=%d", addr, len, ret);
        return DFU_ENGINE_ERR_FLASH_READ;
    }

    return DFU_ENGINE_ERR_NONE;
}

/*============================================================================
 * NAND Cache Management
 *============================================================================*/

dfu_engine_err_t dfu_ef_cache_init(uint32_t addr)
{
    g_ef.target_is_nand = is_nand(addr);

    if (!g_ef.target_is_nand)
        return DFU_ENGINE_ERR_NONE;

    if (g_ef.cache_active)
        return DFU_ENGINE_ERR_NONE;

    memset(&g_ef.cache, 0, sizeof(g_ef.cache));
    memset(g_ef.cache.buf, 0xFF, NAND_PAGESIZE);
    g_ef.cache_active = true;

    LOG_I("NAND cache init for addr=0x%08x", addr);
    return DFU_ENGINE_ERR_NONE;
}

dfu_engine_err_t dfu_ef_cache_flush(void)
{
    if (!g_ef.cache_active)
        return DFU_ENGINE_ERR_NONE;

    rt_err_t ret = cache_flush();
    if (ret != RT_EOK)
        return DFU_ENGINE_ERR_FLASH_WRITE;

    return DFU_ENGINE_ERR_NONE;
}

void dfu_ef_cache_deinit(void)
{
    if (!g_ef.cache_active)
        return;

    g_ef.cache_active = false;
    memset(&g_ef.cache, 0, sizeof(g_ef.cache));
}

bool dfu_ef_is_nand(uint32_t addr)
{
    return is_nand(addr);
}