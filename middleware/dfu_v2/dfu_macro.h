/**
 * @file dfu_macro.h
 * @brief DFU V2 — Shared macro and structure definitions
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_MACRO_H__
#define __DFU_MACRO_H__

#include <stdint.h>
#include <stddef.h>
#include "ptab.h"

/*============================================================================
 * Partition Address
 *============================================================================*/

#define DFU_FLASH_UNINIT_32  0xFFFFFFFF

#ifndef DFU_V2_LOADER_START_ADDR
    #define DFU_V2_LOADER_START_ADDR  0xFFFFFFFF
#endif

#ifndef DFU_V2_LOADER_SIZE
    #define DFU_V2_LOADER_SIZE        0xFFFFFFFF
#endif

/** Firmware info is stored at the last 4KB of the DFU_V2_LOADER partition */
#define DFU_FWINFO_BASE_ADDR \
    (DFU_V2_LOADER_START_ADDR + DFU_V2_LOADER_SIZE - 0x1000)

/*============================================================================
 * Firmware File Info Structure
 *
 * Layout MUST NOT change — bootloader reads raw flash by offset.
 *============================================================================*/

/** Maximum firmware file entries */
#define DFU_MAX_FW_FILES     3

/** Magic: ("dfu\0" << 16) | ("pan\0" & 0xFFFF) — kept for compatibility */
#define DFU_FW_MAGIC \
    ((uint32_t)(0x64667500u << 16) | (0x70616E00u & 0xFFFF))

struct dfu_fw_info
{
    char     name[48];          /**< File name                          off=0   */
    char     url[256];          /**< Download URL (reserved)            off=48  */
    uint32_t addr;              /**< Flash target address               off=304 */
    uint32_t size;              /**< File size                          off=308 */
    uint32_t crc32;             /**< CRC32 checksum                     off=312 */
    uint32_t region_size;       /**< Flash region size                  off=316 */
    uint32_t file_id;           /**< File/image ID                      off=320 */
    uint32_t needs_update;      /**< 1 = pending install                off=324 */
    uint32_t magic;             /**< Must be DFU_FW_MAGIC               off=328 */
};

/*============================================================================
 * Struct Size & Field Offsets (for bootloader raw flash reads)
 *============================================================================*/

#define DFU_FWINFO_SIZE          sizeof(struct dfu_fw_info)
#define DFU_NEEDS_UPDATE_OFFSET  offsetof(struct dfu_fw_info, needs_update)
#define DFU_MAGIC_OFFSET         offsetof(struct dfu_fw_info, magic)

#endif /* __DFU_MACRO_H__ */