/**
 * @file dfu_engine_flash.h
 * @brief DFU V2 Engine — Flash Abstraction Layer (internal)
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_ENGINE_FLASH_H__
#define __DFU_ENGINE_FLASH_H__

#include <stdint.h>
#include <stdbool.h>
#include "dfu_engine_types.h"    /* dfu_engine_err_t */

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Initialization
 *============================================================================*/

/**
 * @brief Initialize the flash abstraction layer
 *
 * Called once by dfu_engine_init(). Verifies that the underlying
 * flash driver function pointers (g_flash_read/write/erase) are valid.
 *
 * @return DFU_ENGINE_ERR_NONE on success
 */
dfu_engine_err_t dfu_ef_init(void);

/**
 * @brief Deinitialize the flash abstraction layer
 *
 * Called by dfu_engine_deinit(). Ensures NAND cache is flushed
 * and deinitialized if it was active.
 */
void dfu_ef_deinit(void);

/*============================================================================
 * Flash Operations
 *============================================================================*/

/**
 * @brief Erase a flash region
 *
 * Automatically handles NOR (sector-aligned) and NAND (block-aligned)
 * differences via the underlying dfu_flash_erase().
 *
 * @param addr  Start address (must be aligned to erase unit)
 * @param size  Size in bytes to erase
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_FLASH_ERASE on failure
 */
dfu_engine_err_t dfu_ef_erase(uint32_t addr, uint32_t size);

/**
 * @brief Write data to flash
 *
 * For NAND targets, writes go through the page cache (dfu_flash_write
 * handles this internally). For NOR targets, writes are direct.
 *
 * @param addr  Target flash address
 * @param data  Source data buffer
 * @param len   Number of bytes to write
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_FLASH_WRITE on failure
 */
dfu_engine_err_t dfu_ef_write(uint32_t addr, const uint8_t *data, uint32_t len);

/**
 * @brief Read data from flash
 *
 * @param addr  Source flash address
 * @param data  Destination buffer
 * @param len   Number of bytes to read
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_FLASH_READ on failure
 */
dfu_engine_err_t dfu_ef_read(uint32_t addr, uint8_t *data, uint32_t len);

/*============================================================================
 * NAND Cache Management
 *============================================================================*/

/**
 * @brief Initialize NAND page write cache
 *
 * Must be called before writing to a NAND flash region.
 * Safe to call when target is NOR — becomes a no-op.
 * Safe to call multiple times.
 *
 * @param addr  Target flash address (used to detect NOR/NAND)
 * @return DFU_ENGINE_ERR_NONE always
 */
dfu_engine_err_t dfu_ef_cache_init(uint32_t addr);

/**
 * @brief Flush NAND page write cache
 *
 * Forces any buffered data to be written to NAND flash.
 * Must be called before CRC verification. No-op for NOR.
 *
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_FLASH_WRITE if flush fails
 */
dfu_engine_err_t dfu_ef_cache_flush(void);

/**
 * @brief Deinitialize NAND page write cache
 *
 * Releases cache resources. Does NOT flush — call dfu_ef_cache_flush()
 * first if needed. No-op for NOR or if not initialized.
 */
void dfu_ef_cache_deinit(void);

/**
 * @brief Check if an address is on NAND flash
 *
 * @param addr  Flash address to check
 * @return true if NAND, false if NOR
 */
bool dfu_ef_is_nand(uint32_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_ENGINE_FLASH_H__ */