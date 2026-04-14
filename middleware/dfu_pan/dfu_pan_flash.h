/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_PAN_FLASH_H__
#define __DFU_PAN_FLASH_H__

#include <stdint.h>
#include <rtthread.h>

/**
 * @brief Erase flash with both NOR and NAND support
 * @param addr Flash address to erase
 * @param size Size to erase in bytes
 * @return RT_EOK on success, error code on failure
 */
rt_err_t dfu_pan_erase(uint32_t addr, uint32_t size);

/**
 * @brief Write data to flash with both NOR and NAND support
 * @param addr Flash address to write to
 * @param data Pointer to data buffer
 * @param size Size to write in bytes
 * @return Number of bytes written, or -1 on failure
 */
int dfu_pan_write(uint32_t addr, const uint8_t *data, uint32_t size);

/**
 * @brief Read data from flash with both NOR and NAND support
 * @param addr Flash address to read from
 * @param data Pointer to data buffer
 * @param size Size of data to read in bytes
 * @return Number of bytes read, or -1 on failure
 */
int dfu_pan_read(uint32_t addr, uint8_t *data, uint32_t size);

/**
 * @brief Initialize NAND flash write cache
 */
void dfu_pan_cache_init(void);

/**
 * @brief Deinitialize NAND flash write cache
 */
void dfu_pan_cache_deinit(void);

/**
 * @brief Flush NAND flash write cache to actual flash
 * @return RT_EOK on success, error code on failure
 */
rt_err_t dfu_pan_flush_cache(void);

#endif /* __DFU_PAN_FLASH_H__ */
