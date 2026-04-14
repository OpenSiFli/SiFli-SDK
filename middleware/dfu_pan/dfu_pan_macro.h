/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _OTA_MACRO_H_
#define _OTA_MACRO_H_

#include <stddef.h>

// Modify the initial address and size of OTA as needed
// DFU_PAN_LOADER_START_ADDR and DFU_PAN_LOADER_SIZE are defined based on
// the OTA dedicated partition space configured in ptab.json
// The end portion of this space is used to store OTA-related data structures
#include "ptab.h"
#define DFU_PAN_FLASH_UNINIT_32 0xFFFFFFFF

#ifndef DFU_PAN_LOADER_START_ADDR
    #define DFU_PAN_LOADER_START_ADDR 0xFFFFFFFF
#endif

#ifndef DFU_PAN_LOADER_SIZE
    #define DFU_PAN_LOADER_SIZE 0xFFFFFFFF
#endif

#define FIRMWARE_INFO_BASE_ADDR                                                \
    (DFU_PAN_LOADER_START_ADDR + DFU_PAN_LOADER_SIZE - 0x1000)

// Structure for storing firmware file information
struct firmware_file_info
{
    char name[48];         // File name
    char url[256];         // Download URL
    uint32_t addr;         // Flash address
    uint32_t size;         // File size
    uint32_t crc32;        // CRC32 checksum
    uint32_t region_size;  // Region size
    uint32_t file_id;      // File ID
    uint32_t needs_update; // Flag indicating whether an update is needed
    uint32_t magic;        // Magic number
};

// Device registration request parameters structure
typedef struct {
    const char* mac;          // Device MAC address (required)
    const char* model;        // Device model (required)
    const char* solution;     // Solution name (required)
    const char* version;      // Device current version (required)
    const char* ota_version;  // Device OTA version (required)
    const char* screen_width; // Device screen width (optional)
    const char* screen_height;// Device screen height (optional)
    const char* flash_type;   // Device flash type (optional)
    const char* chip_id;      // Device chip ID (required)
} device_register_params_t;

// Define maximum number of firmware files
#define MAX_FIRMWARE_FILES 3

// Magic number definitions
#define FIRMWARE_INFO_MAGIC                                                    \
    0x64667500 // ASCII value of "dfu", ensuring 4-byte alignment
#define FIRMWARE_INFO_MAGIC_PAN                                                \
    0x70616E00 // ASCII value of "pan", ensuring 4-byte alignment
#define FIRMWARE_MAGIC_DFU_PAN                                                 \
    ((uint32_t)FIRMWARE_INFO_MAGIC << 16 | (FIRMWARE_INFO_MAGIC_PAN & 0xFFFF))

// Size of firmware info structure
#define FIRMWARE_INFO_SIZE sizeof(struct firmware_file_info)

// Calculate offset of needs_update field using offsetof macro
#define NEEDS_UPDATE_OFFSET offsetof(struct firmware_file_info, needs_update)
#define NEEDS_MAGIC_OFFSET offsetof(struct firmware_file_info, magic)
#endif