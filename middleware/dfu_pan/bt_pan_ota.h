/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_PAN_OTA_H_
#define _BT_PAN_OTA_H_

#include "bts2_app_inc.h"
#include <stdint.h>
#include "dfu_pan_macro.h"
#include "drv_flash.h"

/**
 * @brief Bluetooth application environment for OTA
 *
 * This structure holds the Bluetooth connection state and related information
 * needed for over-the-air updates.
 */
typedef struct
{
    BOOL bt_connected;              /**< Bluetooth connection status */
    bt_notify_device_mac_t bd_addr; /**< Bluetooth device MAC address */
    rt_timer_t pan_connect_timer;   /**< Timer for PAN connection management */
} bt_app_t_ota;

extern bt_app_t_ota g_bt_app_env_ota;

/**
 * @brief Print OTA files command
 *
 * Prints information about the OTA firmware files to the console.
 */
void dfu_pan_print_files(void);

/**
 * @brief Download firmware based on stored firmware information
 *
 * Downloads and stores firmware files to flash based on the provided firmware
 * file information.
 *
 * @param firmware_file_info Array of firmware file information structures
 * @param file_count Number of firmware files to download
 * @return 0 on success, -1 on failure
 */
int dfu_pan_download_firmware(struct firmware_file_info *firmware_file_info,
                              int file_count);


/**
 * @brief Query latest version information
 *
 * Queries the server for the latest firmware version information.
 * After successful execution, firmware file information can be retrieved
 * using dfu_pan_get_firmware_file_info().
 *
 * @param server_url Server URL, uses default URL if NULL
 * @param current_version Current version number
 * @param latest_version_name Buffer to return latest version name
 * @param name_size Buffer size
 * @return 1 if newer version available, 0 if current is latest, -1 on error
 */
int dfu_pan_query_latest_version(const char *server_url,
                                 const char *current_version,
                                 char *latest_version_name, size_t name_size);

/**
 * @brief Clear all firmware information
 *
 * Clears all stored firmware information.
 */
void dfu_pan_clear_files(void);

/**
 * @brief Get firmware file information
 *
 * Retrieves firmware file information from flash storage.
 *
 * @param index Index of the firmware file
 * @param firmware_file_info Pointer to firmware file information structure
 * @return 0 on success, -1 on failure
 */
int dfu_pan_get_firmware_file_info(
    int index, struct firmware_file_info *firmware_file_info);

/**
 * @brief Test OTA update flags
 *
 * Performs a test of the OTA update flag functionality.
 */
void dfu_pan_test_update_flags(void);

/**
 * @brief Set firmware update flags
 *
 * Sets update flags and magic values for firmware files to indicate they need
 * updating.
 *
 * @return 0 on success, -1 on failure
 */
int dfu_pan_set_update_flags(void);

/**
 * @brief Device registration interface
 *
 * Registers the device with the OTA server using the provided parameters.
 *
 * @param server_url Server URL for device registration
 * @param params Device registration parameters including MAC address, model,
 * etc.
 * @return 0 on success, -1 on failure
 */
int dfu_pan_register_device(const char *server_url,
                            const device_register_params_t *params);

#endif