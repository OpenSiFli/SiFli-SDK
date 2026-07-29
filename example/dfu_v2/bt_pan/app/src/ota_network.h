/**
 * @file ota_network.h
 * @brief PAN OTA V2 — Server Registration & Version Query
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __OTA_NETWORK_H__
#define __OTA_NETWORK_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get device MAC address as string "xx:xx:xx:xx:xx:xx"
 * @return Static string pointer
 */
char *ota_get_mac_address(void);

/**
 * @brief Get device chip ID (SHA256 of MAC, UUID format)
 * @return Static string pointer
 */
char *ota_get_chip_id(void);

/**
 * @brief Register device with OTA server
 *
 * POSTs device info (MAC, model, chip_id, version, etc.) to the
 * registration endpoint. Server records the device for firmware
 * distribution.
 *
 * @return 0 on success, -1 on failure
 */
int ota_register_device(void);

/**
 * @brief Build OTA version query URL with chip_id
 *
 * Returns a board-specific URL pointing to the OTA server's version
 * query endpoint, with chip_id as a query parameter.
 *
 * @param chip_id  Device chip ID string
 * @return Static URL string pointer
 */
char *ota_build_query_url(const char *chip_id);

/**
 * @brief Query OTA server for latest firmware version
 *
 * Sends HTTP GET to query_url, parses the JSON response to extract
 * version info and firmware file list. If a newer version is found,
 * writes firmware_file_info entries to flash via dfu_fwinfo_set().
 *
 * @param query_url          Full query URL (from ota_build_query_url)
 * @param current_version    Current firmware version string (e.g. "V1.0")
 * @param latest_version_out Buffer to receive latest version name
 * @param name_size          Buffer size
 * @return 1 if newer version available (info written to flash),
 *         0 if current is latest,
 *         -1 on error
 */
int ota_query_latest_version(const char *query_url,
                             const char *current_version,
                             char *latest_version_out,
                             size_t name_size);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_NETWORK_H__ */
