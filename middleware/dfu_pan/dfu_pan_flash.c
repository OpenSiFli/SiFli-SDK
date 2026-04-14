/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"
#include <rtthread.h>
#include "dfu_pan_flash.h"
#include "dfu_pan_macro.h"
#include "drv_flash.h"
#include "log.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Forward declarations from drv_spi_flash.c
extern void *Addr2Handle(uint32_t addr);

// Debug log control macro - set to 0 to disable debug logs, 1 to enable
#define DFU_PAN_FLASH_DEBUG 0

// Default NAND page size (2048 bytes)
#define DEFAULT_NAND_PAGESIZE 2048

// Debug log wrapper macros
#if DFU_PAN_FLASH_DEBUG
#define DFU_LOG_D(...) LOG_D(__VA_ARGS__)
#define DFU_LOG_E(...) LOG_E(__VA_ARGS__)
#define DFU_LOG_W(...) LOG_W(__VA_ARGS__)
#else
#define DFU_LOG_D(...) do {} while(0)
#define DFU_LOG_E(...) LOG_E(__VA_ARGS__)
#define DFU_LOG_W(...) LOG_W(__VA_ARGS__)
#endif

// Cache structure definition
typedef enum {
    CACHE_EMPTY = 0,
    CACHE_PARTIAL,
    CACHE_FULL
} dfu_cache_state_t;

// Cache structure
typedef struct {
    uint32_t cache_addr;          // Cache target flash address
    uint8_t cache_buf[DEFAULT_NAND_PAGESIZE];  // Cache buffer
    uint32_t cache_size;          // Current cached data size
    dfu_cache_state_t state;      // Cache state
} dfu_nand_cache_t;

// Global cache instance
static dfu_nand_cache_t g_nand_cache = {
    .cache_addr = 0,
    .cache_size = 0,
    .state = CACHE_EMPTY
};

/**
 * @brief Erase flash with NAND support
 * @param addr Flash address to erase
 * @param size Size to erase in bytes
 * @return RT_EOK on success, error code on failure
 */
rt_err_t dfu_pan_erase(uint32_t addr, uint32_t size)
{
    // Default NAND block size (128KB)
    #define DEFAULT_NAND_BLK_SIZE 0x20000
    
    // Get erase alignment
    int erase_alignment = rt_flash_get_erase_alignment(addr);
    if (erase_alignment <= 0) {
        // Use default NAND block size if alignment is invalid
        erase_alignment = DEFAULT_NAND_BLK_SIZE;
        DFU_LOG_W("Invalid erase alignment, using default NAND block size: %d\n", erase_alignment);
    }
    
    // Calculate aligned size
    int aligned_size = ((size + erase_alignment - 1) / erase_alignment) * erase_alignment;
    DFU_LOG_D("Erase addr=0x%08X, size=%d, aligned_size=%d, alignment=%d\n", 
          addr, size, aligned_size, erase_alignment);
    
    // Get flash handle
    void *fhandle = Addr2Handle(addr);
    int result = RT_EOK;
    
    if (fhandle == NULL) {
        // For NAND flash, use rt_nand_erase with aligned address
        DFU_LOG_D("Using NAND erase for address 0x%08X\n", addr);
        
        // Calculate aligned address for NAND flash
        uint32_t aligned_addr = (addr & ~(DEFAULT_NAND_BLK_SIZE - 1));
        
        // Call rt_nand_erase with aligned address and size
        DFU_LOG_D("NAND Erase: orig_addr=0x%08X, orig_size=%d, aligned_addr=0x%08X, aligned_size=%d, blksize=0x%x\n", 
              addr, size, aligned_addr, aligned_size, DEFAULT_NAND_BLK_SIZE);
        
        rt_nand_erase(aligned_addr, aligned_size);
        // rt_nand_erase doesn't return error code, assume success
    } else {
        // For NOR flash, use rt_flash_erase
        DFU_LOG_D("Using NOR erase for address 0x%08X\n", addr);
        result = rt_flash_erase(addr, aligned_size);
    }
    
    return result;
}

/**
 * @brief Initialize NAND flash write cache
 */
void dfu_pan_cache_init(void)
{
    g_nand_cache.cache_addr = 0;
    g_nand_cache.cache_size = 0;
    memset(g_nand_cache.cache_buf, 0xFF, DEFAULT_NAND_PAGESIZE);
    g_nand_cache.state = CACHE_EMPTY;
    DFU_LOG_D("NAND cache initialized\n");
}

/**
 * @brief Deinitialize NAND flash write cache
 */
void dfu_pan_cache_deinit(void)
{
    g_nand_cache.cache_addr = 0;
    g_nand_cache.cache_size = 0;
    g_nand_cache.state = CACHE_EMPTY;
    DFU_LOG_D("NAND cache deinitialized\n");
}

/**
 * @brief Flush NAND flash write cache to actual flash
 * @return RT_EOK on success, error code on failure
 */
rt_err_t dfu_pan_flush_cache(void)
{
    int result = RT_EOK;
    
    if (g_nand_cache.state != CACHE_EMPTY) {
        DFU_LOG_D("Flushing NAND cache, addr=0x%08X, size=%d\n", 
              g_nand_cache.cache_addr, g_nand_cache.cache_size);
        
        // If cache has partial data, fill the rest with 0xFF
        if (g_nand_cache.cache_size < DEFAULT_NAND_PAGESIZE) {
            memset(g_nand_cache.cache_buf + g_nand_cache.cache_size, 
                   0xFF, DEFAULT_NAND_PAGESIZE - g_nand_cache.cache_size);
        }
        
        // Write the full page to NAND flash
        result = rt_nand_write_page(g_nand_cache.cache_addr, 
                                   g_nand_cache.cache_buf, 
                                   DEFAULT_NAND_PAGESIZE, 
                                   NULL, 0);
        
        if (result == DEFAULT_NAND_PAGESIZE) {
            DFU_LOG_D("Cache flush successful\n");
            // Reset cache state
            g_nand_cache.cache_size = 0;
            g_nand_cache.state = CACHE_EMPTY;
            result = RT_EOK;
        } else {
            LOG_E("Cache flush failed, result=%d\n", result);
            result = -1;
        }
    }
    
    return result;
}

/**
 * @brief Internal function to write data to cache
 * @param addr Flash address to write to
 * @param data Pointer to data buffer
 * @param size Size of data to write in bytes
 * @return Number of bytes written to cache, or -1 on failure
 */
static int dfu_pan_write_to_cache(uint32_t addr, const uint8_t *data, uint32_t size)
{
    uint32_t page_start = addr & ~(DEFAULT_NAND_PAGESIZE - 1);
    uint32_t page_offset = addr & (DEFAULT_NAND_PAGESIZE - 1);
    uint32_t remaining = size;
    uint32_t written = 0;
    const uint8_t *current_data = data;
    uint32_t current_addr = addr;
    
    DFU_LOG_D("Writing to cache: addr=0x%08X, size=%d\n", addr, size);
    
    while (remaining > 0) {
        // Check if we need to start a new cache page
        if (g_nand_cache.state == CACHE_EMPTY || g_nand_cache.cache_addr != page_start) {
            // If cache is not empty, flush it first
            if (g_nand_cache.state != CACHE_EMPTY) {
                if (dfu_pan_flush_cache() != RT_EOK) {
                    LOG_E("Failed to flush cache before new write\n");
                    return -1;
                }
            }
            
            // Initialize new cache page
            g_nand_cache.cache_addr = page_start;
            g_nand_cache.cache_size = 0;
            memset(g_nand_cache.cache_buf, 0xFF, DEFAULT_NAND_PAGESIZE);
            g_nand_cache.state = CACHE_PARTIAL;
            DFU_LOG_D("Initialized new cache page at addr=0x%08X\n", page_start);
        }
        
        // Calculate how much data we can write to current cache
        uint32_t cache_space = DEFAULT_NAND_PAGESIZE - g_nand_cache.cache_size;
        uint32_t write_size = remaining > cache_space ? cache_space : remaining;
        
        // Write data to cache
        memcpy(g_nand_cache.cache_buf + g_nand_cache.cache_size, current_data, write_size);
        g_nand_cache.cache_size += write_size;
        written += write_size;
        remaining -= write_size;
        current_data += write_size;
        current_addr += write_size;
        
        // Update cache state
        if (g_nand_cache.cache_size == DEFAULT_NAND_PAGESIZE) {
            g_nand_cache.state = CACHE_FULL;
            // If cache is full, flush it immediately
            if (dfu_pan_flush_cache() != RT_EOK) {
                LOG_E("Failed to flush full cache\n");
                return -1;
            }
        }
        
        // Update page info for next iteration
        page_start = current_addr & ~(DEFAULT_NAND_PAGESIZE - 1);
        page_offset = current_addr & (DEFAULT_NAND_PAGESIZE - 1);
    }
    
    return written;
}

// Override the original dfu_pan_write function with cache support
int dfu_pan_write(uint32_t addr, const uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0) {
        LOG_E("Invalid parameters for flash write\n");
        return -1;
    }
    
    DFU_LOG_D("Write addr=0x%08X, size=%d\n", addr, size);
    
    // Get flash handle
    void *fhandle = Addr2Handle(addr);
    int result = size;
    
    if (fhandle == NULL) {
        // For NAND flash, use cached write
        DFU_LOG_D("Using NAND write with cache for address 0x%08X\n", addr);
        
        result = dfu_pan_write_to_cache(addr, data, size);
        if (result < 0) {
            LOG_E("NAND Write with cache failed\n");
            return -1;
        }
        
        // Smart flush strategy for firmware info write
        // Firmware info is written to FIRMWARE_INFO_BASE_ADDR (near DFU_PAN loader)
        // and its size is usually much smaller than a page (e.g., 996 bytes)
        if ((addr == FIRMWARE_INFO_BASE_ADDR) || 
            (addr > FIRMWARE_INFO_BASE_ADDR && addr < FIRMWARE_INFO_BASE_ADDR + 0x1000)) {
            DFU_LOG_D("Detected firmware info write, checking flush strategy\n");
            
            // Flush immediately if cache is not full
            // This ensures firmware info is written to flash immediately
            // even if it's smaller than a page
            if (g_nand_cache.cache_size < DEFAULT_NAND_PAGESIZE) {
                DFU_LOG_D("Cache not full, flushing firmware info immediately\n");
                if (dfu_pan_flush_cache() != RT_EOK) {
                    LOG_E("Failed to flush cache for firmware info write\n");
                    return -1;
                }
            }
        }
    } else {
        // For NOR flash, use direct write
        DFU_LOG_D("Using NOR write for address 0x%08X\n", addr);
        result = rt_flash_write(addr, data, size);
        
        // Check result
        if (result < 0 || result != size) {
            LOG_E("NOR Write: Failed, addr=0x%08X, expected=%d, actual=%d\n", 
                  addr, size, result);
            return -1;
        }
    }
    
    return result;
}

/**
 * @brief Read data from flash with NAND support
 * @param addr Flash address to read from
 * @param data Pointer to data buffer
 * @param size Size of data to read in bytes
 * @return Number of bytes read, or -1 on failure
 */
int dfu_pan_read(uint32_t addr, uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0) {
        LOG_E("Invalid parameters for flash read\n");
        return -1;
    }
    
    DFU_LOG_D("Read addr=0x%08X, size=%d\n", addr, size);
    
    // Get flash handle
    void *fhandle = Addr2Handle(addr);
    int result = size;
    
    if (fhandle == NULL) {
        // For NAND flash, use rt_nand_read directly
        DFU_LOG_D("Using NAND read for address 0x%08X\n", addr);
        result = rt_nand_read(addr, data, size);
    } else {
        // For NOR flash, use rt_flash_read
        DFU_LOG_D("Using NOR read for address 0x%08X\n", addr);
        result = rt_flash_read(addr, data, size);
    }
    
    if (result < 0 || result != size) {
        LOG_E("Flash read failed, addr=0x%08X, expected=%d, actual=%d\n", 
              addr, size, result);
        return -1;
    }
    
    return result;
}


