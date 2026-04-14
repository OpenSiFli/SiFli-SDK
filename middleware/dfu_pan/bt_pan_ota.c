/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include <webclient.h>
#include <cJSON.h>
#include <ulog.h>
#include <lwip/sys.h>
#include "lwip/tcpip.h"
#include "string.h"
#include <stdlib.h>
#include <ctype.h>
#include "register.h"
#include "bt_pan_ota.h"
#include "dfu_pan_ui.h"
#include "dfu_pan_macro.h"
#include "dfu_pan_flash.h"

#define OTA_WORKER_THREAD_STACK_SIZE 8192
static struct rt_thread ota_worker_thread;

bt_app_t_ota g_bt_app_env_ota;
static struct firmware_file_info s_temp_firmware_files[MAX_FIRMWARE_FILES];

#define PAN_OTA_HEADER_BUFSZ 2048
#define OTA_RECV_BUFF_SIZE 2048

// Definitions related to CRC32 checksum
#define CRC32_POLY 0xEDB88320
static uint32_t crc32_table[256];

// Initialize the CRC32 table
static void init_crc32_table(void)
{
    for (int i = 0; i < 256; i++)
    {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ CRC32_POLY;
            else
                crc >>= 1;
        }
        crc32_table[i] = crc;
    }
}

// Make sure to initialize the table before using CRC.
static uint32_t calculate_crc32(const uint8_t *data, size_t length,
                                uint32_t crc)
{
    static int crc_table_initialized = 0;

    // Ensure that the CRC table is initialized only once.
    if (!crc_table_initialized)
    {
        init_crc32_table();
        crc_table_initialized = 1;
    }

    crc = crc ^ 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFF];
    }
    return crc ^ 0xFFFFFFFF;
}

// Exit Bluetooth Sniff Mode
static void exist_sniff_mode(void)
{
    rt_kprintf("exit sniff mode\n");
    bt_interface_exit_sniff_mode(
        &g_bt_app_env_ota.bd_addr); // exit sniff mode
    bt_interface_wr_link_policy_setting(
        &g_bt_app_env_ota.bd_addr,
        BT_NOTIFY_LINK_POLICY_ROLE_SWITCH); // close role switch
}

/**
 * @brief Compare version strings
 *
 * Compares two version strings numerically, handling versions with prefixes
 * like 'v' or 'V' and multiple components separated by dots (e.g., "1.2.3").
 *
 * @param v1 First version string to compare
 * @param v2 Second version string to compare
 * @return 1 if v1 > v2, -1 if v1 < v2, 0 if equal, -2 on error (invalid input)
 */
// Compare version number strings
static int compare_version_strings(const char *v1, const char *v2)
{
    if (!v1 || !v2)
    {
        return -2;
    }

    BOOL v1_has_prefix = (v1[0] == 'v' || v1[0] == 'V');
    BOOL v2_has_prefix = (v2[0] == 'v' || v2[0] == 'V');

    if (v1_has_prefix != v2_has_prefix)
    {
        return -2;
    }

    char version1[32];
    char version2[32];

    const char *ver1_ptr = v1;
    const char *ver2_ptr = v2;

    if (v1_has_prefix)
    {
        ver1_ptr++;
    }

    if (v2_has_prefix)
    {
        ver2_ptr++;
    }

    if (*ver1_ptr == '\0' || *ver2_ptr == '\0')
    {
        return -2;
    }

    strncpy(version1, ver1_ptr, sizeof(version1) - 1);
    version1[sizeof(version1) - 1] = '\0';

    strncpy(version2, ver2_ptr, sizeof(version2) - 1);
    version2[sizeof(version2) - 1] = '\0';

    char *temp1 = version1;
    char *temp2 = version2;
    char *token1, *token2;

    while (1)
    {
        token1 = strtok_r(temp1, ".", &temp1);
        token2 = strtok_r(temp2, ".", &temp2);

        if (token1 == NULL && token2 == NULL)
        {
            return 0;
        }

        int num1 = token1 ? atoi(token1) : 0;
        int num2 = token2 ? atoi(token2) : 0;

        if (num1 > num2)
            return 1;
        if (num1 < num2)
            return -1;

        if (token1 == NULL || token2 == NULL)
        {
            continue;
        }
    }
}

// Set the firmware update flag and the magic value, which are used to set the
// update before a jump is required.
int dfu_pan_set_update_flags(void)
{
    LOG_I("Marking versions for update...\n");

    // Read the existing version information
    struct firmware_file_info temp_version_list[MAX_FIRMWARE_FILES];
    int data_size = sizeof(struct firmware_file_info) * MAX_FIRMWARE_FILES;

    int read_result = dfu_pan_read(FIRMWARE_INFO_BASE_ADDR,
                                    (uint8_t *)temp_version_list, data_size);
    if (read_result != data_size)
    {
        LOG_E("Failed to read version info from flash");
        return -1;
    }

    // Mark all versions for update
    LOG_I("Marking all versions for update");
    for (int i = 0; i < MAX_FIRMWARE_FILES; i++)
    {
        // Check if it is a valid version information
        if (temp_version_list[i].name[0] == '\0')
        {
            continue;
        }

        temp_version_list[i].magic = FIRMWARE_MAGIC_DFU_PAN;
        temp_version_list[i].needs_update = 1;
        LOG_I("Marked version[%d] %s for update", i, temp_version_list[i].name);
    }

    // Erase the Flash area
    // Use dfu_pan_erase which handles NAND/NOR automatically
    if (dfu_pan_erase(FIRMWARE_INFO_BASE_ADDR, data_size) != RT_EOK)
    {
        LOG_E("Failed to erase flash at 0x%08X", FIRMWARE_INFO_BASE_ADDR);
        return -1;
    }

    // Write version information with update flags to flash
    int write_result = dfu_pan_write(FIRMWARE_INFO_BASE_ADDR, (uint8_t *)temp_version_list,
                                     data_size);
    if (write_result < 0)
    {
        LOG_E("Failed to write version info to flash, result=%d", write_result);
        return -1;
    }

    LOG_I("Successfully marked versions for update");
    return 0;
}

// Read the firmware file information
int dfu_pan_get_firmware_file_info(
    int index, struct firmware_file_info *firmware_file_info)
{
    if (index < 0 || index >= MAX_FIRMWARE_FILES || firmware_file_info == NULL)
    {
        return -1;
    }

    uint32_t addr =
        FIRMWARE_INFO_BASE_ADDR + index * sizeof(struct firmware_file_info);
    int result = dfu_pan_read(addr, (uint8_t *)firmware_file_info,
                               sizeof(struct firmware_file_info));

    if (result != sizeof(struct firmware_file_info))
    {
        LOG_E("Failed to read firmware file info from flash at 0x%08X, result: "
              "%d",
              addr, result);
        return -1;
    }

    return 0;
}
// Clear all information
void dfu_pan_clear_files(void)
{
    LOG_I("Clearing all firmware...\n");
    struct firmware_file_info temp_version_files[MAX_FIRMWARE_FILES];

    // Initialize the entire array to 0 and clear all version information.
    memset(temp_version_files, 0, sizeof(temp_version_files));

    // Use dfu_pan_erase which handles NAND/NOR automatically
    int data_size = sizeof(struct firmware_file_info) * MAX_FIRMWARE_FILES;
    
    // Erase flash
    if (dfu_pan_erase(FIRMWARE_INFO_BASE_ADDR, data_size) != RT_EOK)
    {
        LOG_E("Failed to erase firmware info flash at 0x%08X\n", FIRMWARE_INFO_BASE_ADDR);
        return;
    }
    
    // Write cleared firmware info
    int write_result = dfu_pan_write(FIRMWARE_INFO_BASE_ADDR, (uint8_t *)temp_version_files,
                                     sizeof(struct firmware_file_info) * MAX_FIRMWARE_FILES);
    
    if (write_result < 0)
    {
        LOG_E("Failed to write cleared firmware info to flash, result=%d\n", write_result);
        return;
    }

    LOG_I("firmware cleared successfully.\n");
}

// Querying the version information stores the firmware information in the
// flash.
int dfu_pan_query_latest_version(const char *server_url,
                                 const char *current_version,
                                 char *latest_version_name, size_t name_size)
{
    // dfu_pan_clear_files(); // Clear existing firmware information
    struct webclient_session *session = RT_NULL;
    char *buffer = RT_NULL;
    int content_pos = 0;
    int resp_status = 0;
    char query_url[512];
    int ret = 0;

    LOG_I("dfu_pan_query_latest_version\n");
    
    // Initialize flash write cache
    dfu_pan_cache_init();

    if (server_url == NULL)
    {
        // If no URL is provided, an error will be returned.
        LOG_E("No server URL provided");
        return -1;
    }
    else
    {
        // Use the complete URL passed in directly
        strncpy(query_url, server_url, sizeof(query_url) - 1);
        query_url[sizeof(query_url) - 1] = '\0';
    }

    rt_kputs(query_url);
    LOG_I("\n");

    LOG_I("exist_sniff_mode\n");
    exist_sniff_mode();

    session = webclient_session_create(PAN_OTA_HEADER_BUFSZ);
    if (!session)
    {
        LOG_E("Create session failed!");
        ret = -1;
        goto __exit;
    }

    buffer = rt_calloc(1, OTA_RECV_BUFF_SIZE);
    if (!buffer)
    {
        LOG_E("No memory for buffer!");
        ret = -1;
        goto __exit;
    }

    resp_status = webclient_get(session, query_url);
    if (resp_status != 200)
    {
        LOG_E("GET request failed, response code: %d", resp_status);
        ret = -1;
        goto __exit;
    }

    // Check whether the length of the response content exceeds the buffer size.
    if (session->content_length > OTA_RECV_BUFF_SIZE)
    {
        LOG_E("Response content length (%d) exceeds buffer size (%d)",
              session->content_length, OTA_RECV_BUFF_SIZE);
        ret = -1;
        goto __exit;
    }

    while (content_pos < session->content_length)
    {
        int bytes_to_read =
            session->content_length - content_pos > OTA_RECV_BUFF_SIZE
                ? OTA_RECV_BUFF_SIZE
                : session->content_length - content_pos;

        // Add additional security checks
        if (content_pos + bytes_to_read > OTA_RECV_BUFF_SIZE)
        {
            bytes_to_read = OTA_RECV_BUFF_SIZE - content_pos;
        }

        if (bytes_to_read <= 0)
        {
            LOG_E("Buffer overflow prevented. Would read %d bytes to offset %d",
                  bytes_to_read, content_pos);
            ret = -1;
            goto __exit;
        }

        int bytes_read =
            webclient_read(session, buffer + content_pos, bytes_to_read);
        if (bytes_read <= 0)
        {
            break;
        }
        content_pos += bytes_read;
    }

    if (content_pos > 0)
    {
        cJSON *root = cJSON_Parse(buffer);
        if (!root)
        {
            LOG_E("Failed to parse JSON: [%s]", cJSON_GetErrorPtr());
            ret = -1;
            goto __exit;
        }

        LOG_I("Latest version check response:\n");
        rt_kputs(buffer);
        LOG_I("\n");

        cJSON *result_item = cJSON_GetObjectItem(root, "result");
        if (!result_item || result_item->valueint != 200)
        {
            LOG_E("Server returned error result: %d",
                  result_item ? result_item->valueint : -1);
            cJSON_Delete(root);
            ret = -1;
            goto __exit;
        }

        cJSON *data_item = cJSON_GetObjectItem(root, "data");
        // Check if the 'data' is an array and retrieve the first element
        cJSON *version_object = NULL;
        if (data_item && cJSON_IsArray(data_item))
        {
            // If it is an array, obtain the first element
            version_object = cJSON_GetArrayItem(data_item, 0);
            if (!version_object)
            {
                LOG_E("Empty data array in response");
                cJSON_Delete(root);
                ret = -1;
                goto __exit;
            }
        }
        else if (data_item && cJSON_IsObject(data_item))
        {
            // If it is an object, simply use it directly.
            version_object = data_item;
        }
        else
        {
            LOG_E("Invalid data format in response - not array or object");
            cJSON_Delete(root);
            ret = -1;
            goto __exit;
        }

        cJSON *name_item = cJSON_GetObjectItem(version_object, "name");
        if (!name_item || !cJSON_IsString(name_item))
        {
            LOG_E("Invalid name format in response");
            cJSON_Delete(root);
            ret = -1;
            goto __exit;
        }

        // Compare version numbers
        int version_compare_result =
            compare_version_strings(name_item->valuestring, current_version);
        LOG_I("Version comparison: server='%s', current='%s', result=%d",
              name_item->valuestring, current_version, version_compare_result);

        if (version_compare_result > 0)
        {
            // Server version update
            LOG_I("Newer version available: %s (current: %s)",
                  name_item->valuestring, current_version);

            // If a buffer is provided, the version name will be copied.
            if (latest_version_name && name_size > 0)
            {
                rt_strncpy(latest_version_name, name_item->valuestring,
                           name_size - 1);
                latest_version_name[name_size - 1] = '\0';
            }

            // 2. Examine firmware details
            cJSON *files_array = cJSON_GetObjectItem(version_object, "files");
            if (!files_array || !cJSON_IsArray(files_array))
            {
                LOG_E("Invalid files format in response");
                cJSON_Delete(root);
                ret = -1;
                goto __exit;
            }

            // 3. Store in Flash
            memset(s_temp_firmware_files, 0, sizeof(s_temp_firmware_files));

            int file_count = 0;
            cJSON *file_item = NULL;
            cJSON_ArrayForEach(file_item, files_array)
            {
                if (file_count >= MAX_FIRMWARE_FILES)
                {
                    LOG_W("Too many firmware files, only process first %d",
                          MAX_FIRMWARE_FILES);
                    break;
                }

                // Check if there is a "file_id" field. If not, skip.
                cJSON *file_id_item = cJSON_GetObjectItem(file_item, "file_id");
                if (!file_id_item)
                {
                    LOG_I("Skipping file without file_id");
                    continue;
                }

                cJSON *file_name_item =
                    cJSON_GetObjectItem(file_item, "file_name");
                cJSON *url_item = cJSON_GetObjectItem(file_item, "url");
                cJSON *addr_item = cJSON_GetObjectItem(file_item, "addr");
                cJSON *size_item = cJSON_GetObjectItem(file_item, "file_size");
                cJSON *crc32_item = cJSON_GetObjectItem(file_item, "crc32");
                cJSON *region_size_item =
                    cJSON_GetObjectItem(file_item, "region_size");

                if (!file_name_item || !cJSON_IsString(file_name_item) ||
                    !url_item || !cJSON_IsString(url_item))
                {
                    LOG_W("Skipping invalid file entry");
                    continue;
                }

                // Copy file name
                rt_strncpy(s_temp_firmware_files[file_count].name,
                           file_name_item->valuestring,
                           sizeof(s_temp_firmware_files[file_count].name) - 1);

                // Copy URL
                rt_strncpy(s_temp_firmware_files[file_count].url,
                           url_item->valuestring,
                           sizeof(s_temp_firmware_files[file_count].url) - 1);

                // Parsing address
                if (addr_item && cJSON_IsString(addr_item))
                {
                    s_temp_firmware_files[file_count].addr =
                        strtoul(addr_item->valuestring, NULL, 0);
                }

                // Analysis of size
                if (size_item)
                {
                    if (cJSON_IsNumber(size_item))
                    {
                        s_temp_firmware_files[file_count].size =
                            size_item->valueint;
                    }
                    else if (cJSON_IsString(size_item))
                    {
                        s_temp_firmware_files[file_count].size =
                            atoi(size_item->valuestring);
                    }
                }

                // Analyzing CRC32
                if (crc32_item && cJSON_IsString(crc32_item))
                {
                    s_temp_firmware_files[file_count].crc32 =
                        strtoul(crc32_item->valuestring, NULL, 0);
                }

                // Analyze the size of the area
                if (region_size_item && cJSON_IsString(region_size_item))
                {
                    s_temp_firmware_files[file_count].region_size =
                        strtoul(region_size_item->valuestring, NULL, 0);
                }

                // Set file ID
                if (file_id_item)
                {
                    if (cJSON_IsNumber(file_id_item))
                    {
                        s_temp_firmware_files[file_count].file_id =
                            file_id_item->valueint;
                    }
                    else if (cJSON_IsString(file_id_item))
                    {
                        s_temp_firmware_files[file_count].file_id =
                            atoi(file_id_item->valuestring);
                    }
                }

                LOG_I("Parsed firmware file[%d]: %s", file_count,
                      s_temp_firmware_files[file_count].name);
                LOG_I("  URL: %s", s_temp_firmware_files[file_count].url);
                LOG_I("  Addr: 0x%08X", s_temp_firmware_files[file_count].addr);
                LOG_I("  Size: %d", s_temp_firmware_files[file_count].size);
                LOG_I("  CRC32: 0x%08X", s_temp_firmware_files[file_count].crc32);
                LOG_I("  Region Size: 0x%08X",
                      s_temp_firmware_files[file_count].region_size);
                LOG_I("  File ID: %d", s_temp_firmware_files[file_count].file_id);

                file_count++;
            }

            // Write to Flash

            int data_size =
                sizeof(struct firmware_file_info) * MAX_FIRMWARE_FILES;
            if (dfu_pan_erase(FIRMWARE_INFO_BASE_ADDR, data_size) != RT_EOK)
            {
                LOG_E("Failed to erase flash at 0x%08X",
                      FIRMWARE_INFO_BASE_ADDR);
                cJSON_Delete(root);
                ret = -1;
                goto __exit;
            }

            int write_result = dfu_pan_write(FIRMWARE_INFO_BASE_ADDR,
                                            (uint8_t *)s_temp_firmware_files,
                                            data_size);
            if (write_result < 0)
            {
                LOG_E("Failed to write firmware info to flash, result=%d", write_result);
                cJSON_Delete(root);
                ret = -1;
                goto __exit;
            }

            LOG_I("Successfully saved %d firmware files info to flash",
                  file_count);
            ret = 1; // Indicates the existence of a new version
        }
        else if (version_compare_result == 0)
        {
            LOG_I("Current version is latest: %s", current_version);
            ret = 0; // Indicates that this version is the latest.
        }
        else
        {
            LOG_I(
                "Current version is newer than server version: %s (server: %s)",
                current_version, name_item->valuestring);
            ret = 0; // Indicates that this version is the latest.
        }

        cJSON_Delete(root);
    }
    else
    {
        LOG_E("Failed to read response data. Content pos: %d", content_pos);
        LOG_E("Session content length: %d", session->content_length);
        ret = -1;
    }

__exit:
    // Deinitialize flash write cache
    dfu_pan_cache_deinit();
    
    if (session)
    {
        LOCK_TCPIP_CORE();
        webclient_close(session);
        UNLOCK_TCPIP_CORE();
    }

    if (buffer)
    {
        rt_free(buffer);
    }

    return ret;
}

// Based on the contents of the firmware structure, download and store the
// firmware file into the Flash.
int dfu_pan_download_firmware(struct firmware_file_info *firmware_file_info,
                              int file_count)
{
    struct webclient_session *session = RT_NULL;
    char *buffer = RT_NULL;
    int ret = 0;

    if (firmware_file_info == NULL || file_count <= 0)
    {
        LOG_E("Invalid parameters for firmware download");
        return -1;
    }

    LOG_I("Starting to download and store %d firmware files to flash\n",
          file_count);

    // Allocate receive buffer
    buffer = rt_calloc(1, OTA_RECV_BUFF_SIZE);
    if (!buffer)
    {
        LOG_E("No memory for buffer!\n");
        ret = -1;
        goto __exit;
    }
    
    // Initialize NAND flash write cache
    dfu_pan_cache_init();

    // Process each firmware file
    for (int i = 0; i < file_count; i++)
    {
        // Check if the file name is empty. If it is empty, skip it.
        if (firmware_file_info[i].name[0] == '\0')
        {
            LOG_W("Skipping empty firmware entry at index %d", i);
            continue;
        }

        uint32_t remaining_length = firmware_file_info[i].size;
        uint32_t data_written = 0;
        int last_progress = -1;

        LOG_I("Processing firmware %s, size %d bytes\n",
              firmware_file_info[i].name, remaining_length);

        // Send the name of the firmware that is being updated.
        dfu_pan_ui_update_message(UI_MSG_UPDATE_FILES,
                                  firmware_file_info[i].name);

        // Check whether the firmware size is reasonable
        if (remaining_length == 0)
        {
            LOG_W("Warning: Firmware %s has zero length\n",
                  firmware_file_info[i].name);
            continue;
        }

        // Erase the target Flash area
        LOG_I("Erasing flash region for %s at 0x%08X, size 0x%08X\n",
              firmware_file_info[i].name, firmware_file_info[i].addr,
              firmware_file_info[i].region_size);

        // Use dfu_pan_erase which handles NAND/NOR automatically
        if (dfu_pan_erase(firmware_file_info[i].addr, firmware_file_info[i].region_size) != RT_EOK)
        {
            LOG_E("Failed to erase flash for %s at 0x%08X\n",
                  firmware_file_info[i].name, firmware_file_info[i].addr);
            ret = -1;
            goto __exit;
        }

        LOG_I("exist_sniff_mode\n");
        exist_sniff_mode();

        // Create network session for this file
        session = webclient_session_create(PAN_OTA_HEADER_BUFSZ);
        if (!session)
        {
            LOG_E("Create session failed!\n");
            ret = -1;
            goto __exit;
        }

        // Send GET request to download firmware
        int resp_status = webclient_get(session, firmware_file_info[i].url);
        LOG_I("HTTP response status for %s: %d\n", firmware_file_info[i].name,
              resp_status);

        // Check HTTP response status
        if (resp_status != 200)
        {
            LOG_E("GET request failed for %s, response code: %d\n",
                  firmware_file_info[i].name, resp_status);
            ret = -1;
            goto __exit;
        }

        // Stream processing firmware data
        // Read data in chunks and write to flash immediately to avoid memory
        // overflow
        while (remaining_length > 0)
        {
            // Calculate chunk size, not exceeding buffer size
            int chunk_size = (remaining_length > OTA_RECV_BUFF_SIZE)
                                 ? OTA_RECV_BUFF_SIZE
                                 : remaining_length;
            int bytes_read = webclient_read(session, buffer, chunk_size);

            if (bytes_read <= 0)
            {
                LOG_E("Failed to read firmware data for %s\n",
                      firmware_file_info[i].name);
                ret = -1;
                goto __exit;
            }

            // Write data to flash immediately after reading
            int write_result = dfu_pan_write(firmware_file_info[i].addr + data_written,
                                            (uint8_t *)buffer, bytes_read);
            if (write_result < 0)
            {
                LOG_E("Failed to write firmware data to flash for %s\n",
                      firmware_file_info[i].name);
                ret = -1;
                goto __exit;
            }

            remaining_length -= bytes_read;
            data_written += bytes_read;

            // Display progress
            if (firmware_file_info[i].size > 0)
            {
                int progress =
                    ((firmware_file_info[i].size - remaining_length) * 100) /
                    firmware_file_info[i].size;

                if (progress != last_progress)
                {
                    LOG_I("Writing %s: %d%% (%d/%d bytes)\n",
                          firmware_file_info[i].name, progress,
                          firmware_file_info[i].size - remaining_length,
                          firmware_file_info[i].size);

                    char progress_str[8];
                    snprintf(progress_str, sizeof(progress_str), "%d",
                             progress);
                    // Update progress
                    dfu_pan_ui_update_message(UI_MSG_UPDATE_PROGRESS,
                                              progress_str);

                    last_progress = progress;
                }
            }
        }

        LOG_I("Successfully wrote firmware %s to flash, %d bytes\n",
              firmware_file_info[i].name, data_written);

        // Flush any remaining data in NAND flash write cache before CRC verification
        if (dfu_pan_flush_cache() != RT_EOK)
        {
            LOG_E("Failed to flush NAND flash write cache before CRC verification\n");
            ret = -1;
            goto __exit;
        }

        // Verify data integrity by calculating CRC if needed
        LOG_I("Verifying CRC for %s...\n", firmware_file_info[i].name);
        uint8_t *verify_buffer = rt_malloc(OTA_RECV_BUFF_SIZE);
        if (verify_buffer)
        {
            uint32_t calculated_crc = 0xffffffff;
            uint32_t verify_remaining = data_written; // Use actual written size
            uint32_t verify_offset = 0;

            // Verify data integrity by calculating CRC
            while (verify_remaining > 0)
            {
                int verify_chunk = (verify_remaining > OTA_RECV_BUFF_SIZE)
                                       ? OTA_RECV_BUFF_SIZE
                                       : verify_remaining;

                if (dfu_pan_read(firmware_file_info[i].addr + verify_offset,
                                  verify_buffer, verify_chunk) != verify_chunk)
                {
                    LOG_E("Failed to read flash for CRC verification\n");
                    rt_free(verify_buffer);
                    ret = -1;
                    goto __exit;
                }

                calculated_crc = calculate_crc32(verify_buffer, verify_chunk,
                                                 calculated_crc);

                verify_remaining -= verify_chunk;
                verify_offset += verify_chunk;
            }

            // Compare calculated CRC with stored CRC
            if (calculated_crc != firmware_file_info[i].crc32)
            {
                LOG_E("CRC verification failed for %s. Expected: 0x%08X, Got: "
                      "0x%08X\n",
                      firmware_file_info[i].name, firmware_file_info[i].crc32,
                      calculated_crc);
                rt_free(verify_buffer);
                ret = -1;
                goto __exit;
            }

            LOG_I("CRC verification passed for %s (calculated: 0x%08X)\n",
                  firmware_file_info[i].name, calculated_crc);
            rt_free(verify_buffer);
        }
        else
        {
            LOG_W("No memory for CRC verification buffer, skipping "
                  "verification\n");
        }

        // Close session for this file
        if (session)
        {
            LOCK_TCPIP_CORE();
            webclient_close(session);
            UNLOCK_TCPIP_CORE();
            session = RT_NULL;
        }
    }

__exit:
    if (session)
    {
        LOCK_TCPIP_CORE();
        webclient_close(session);
        UNLOCK_TCPIP_CORE();
    }

    // Flush any remaining data in NAND flash write cache
    if (dfu_pan_flush_cache() != RT_EOK)
    {
        LOG_E("Failed to flush NAND flash write cache\n");
        if (ret == 0) // Only set error if no previous error
        {
            ret = -1;
        }
    }
    
    // Deinitialize NAND flash write cache
    dfu_pan_cache_deinit();
    
    if (buffer)
    {
        rt_free(buffer);
    }

    LOG_I("dfu_pan_download_firmware finished with result: %d\n", ret);

    return ret;
}

// Print all firmware file information
void dfu_pan_print_files(void)
{
    LOG_I("----------------OTA version_list Files Status:*******\n");
    LOG_I("==========================\n");
    LOG_I("OTA version_list Files Address: 0x%08X\n", FIRMWARE_INFO_BASE_ADDR);

    for (int i = 0; i < MAX_FIRMWARE_FILES; i++)
    {
        struct firmware_file_info firmware_file_info;
        if (dfu_pan_get_firmware_file_info(i, &firmware_file_info) == 0)
        {
            LOG_I("File %d:\n", i);
            LOG_I("  Name: %s\n", firmware_file_info.name);
            LOG_I("url: \n", firmware_file_info.url);
            rt_kputs(firmware_file_info.url);
            LOG_I("\n");
            LOG_I("  Address: 0x%08X\n", firmware_file_info.addr);
            LOG_I("  file Size: %d bytes\n", firmware_file_info.size);
            LOG_I("  CRC: 0x%08X\n", firmware_file_info.crc32);
            LOG_I("  Region Size: %d bytes\n", firmware_file_info.region_size);
            LOG_I("  file_id: %d\n", firmware_file_info.file_id);
            LOG_I("  needs_update: %d\n", firmware_file_info.needs_update);
            LOG_I("  magic: %d\n", firmware_file_info.magic);
            LOG_I("  ------------------------\n");
        }
        else
        {
            LOG_E("Failed to read firmware info at index %d\n", i);
        }
    }
}

void dfu_pan_test_update_flags(void)
{
    LOG_I("=== OTA Update Flags Test ===\n");

    // 1. Print initial state
    LOG_I("1. Initial state:\n");
    for (int i = 0; i < MAX_FIRMWARE_FILES; i++)
    {
        struct firmware_file_info temp_version;
        if (dfu_pan_get_firmware_file_info(i, &temp_version) == 0)
        {
            uint32_t needs_update_addr =
                FIRMWARE_INFO_BASE_ADDR +
                i * sizeof(struct firmware_file_info) +
                offsetof(struct firmware_file_info, needs_update);
            LOG_I("  Entry %d: needs_update=%d (addr: 0x%08x)", i,
                  temp_version.needs_update, needs_update_addr);
        }
        else
        {
            uint32_t needs_update_addr =
                FIRMWARE_INFO_BASE_ADDR +
                i * sizeof(struct firmware_file_info) +
                offsetof(struct firmware_file_info, needs_update);
            LOG_I("  Entry %d: read failed (addr: 0x%08x)", i,
                  needs_update_addr);
        }
    }

    // 2. Set all the flags to 1
    LOG_I("2. Setting all flags to 1:\n");
    struct firmware_file_info temp_version_files[MAX_FIRMWARE_FILES];

    // Initialize all entries directly, and only set the necessary fields.
    for (int i = 0; i < MAX_FIRMWARE_FILES; i++)
    {
        memset(&temp_version_files[i], 0, sizeof(struct firmware_file_info));
        temp_version_files[i].needs_update = 1;
        LOG_I("  Setting entry %d needs_update = 1", i);
    }

    // Write to flash memory
    int data_size = sizeof(struct firmware_file_info) * MAX_FIRMWARE_FILES;
    
    // Use dfu_pan_erase which handles NAND/NOR automatically
    if (dfu_pan_erase(FIRMWARE_INFO_BASE_ADDR, data_size) != RT_EOK)
    {
        LOG_E("Failed to erase flash at 0x%08X", FIRMWARE_INFO_BASE_ADDR);
        return;
    }

    int write_result =
        dfu_pan_write(FIRMWARE_INFO_BASE_ADDR, (uint8_t *)temp_version_files,
                       sizeof(struct firmware_file_info) * MAX_FIRMWARE_FILES);

    if (write_result < 0)
    {
        LOG_E("Failed to write to flash. Result: %d", write_result);
        return;
    }
    LOG_I("All flags set to 1 successfully.\n");

    // 3. Verify whether all the flags are set to 1
    LOG_I("3. Verifying all flags are set to 1:\n");
    for (int i = 0; i < MAX_FIRMWARE_FILES; i++)
    {
        struct firmware_file_info temp_version;
        if (dfu_pan_get_firmware_file_info(i, &temp_version) == 0)
        {
            uint32_t needs_update_addr =
                FIRMWARE_INFO_BASE_ADDR +
                i * sizeof(struct firmware_file_info) +
                offsetof(struct firmware_file_info, needs_update);
            LOG_I("  Entry %d: needs_update=%d (addr: 0x%08x) %s", i,
                  temp_version.needs_update, needs_update_addr,
                  (temp_version.needs_update == 1) ? "OK" : "ERROR");
        }
        else
        {
            uint32_t needs_update_addr =
                FIRMWARE_INFO_BASE_ADDR +
                i * sizeof(struct firmware_file_info) +
                offsetof(struct firmware_file_info, needs_update);
            LOG_E("  Entry %d: read failed (addr: 0x%08x)", i,
                  needs_update_addr);
        }
    }

    // 4. Clear all the flags that are set to 0
    LOG_I("4. Clearing all flags to 0:\n");
    // Reinitialize the array
    for (int i = 0; i < MAX_FIRMWARE_FILES; i++)
    {
        memset(&temp_version_files[i], 0, sizeof(struct firmware_file_info));
        temp_version_files[i].needs_update = 0;
        LOG_I("  Setting entry %d needs_update = 0", i);
    }

    // Write to flash memory (reuse existing data_size variable)
    
    // Use dfu_pan_erase which handles NAND/NOR automatically
    if (dfu_pan_erase(FIRMWARE_INFO_BASE_ADDR, data_size) != RT_EOK)
    {
        LOG_E("Failed to erase flash at 0x%08X", FIRMWARE_INFO_BASE_ADDR);
        return;
    }

    write_result = 
        dfu_pan_write(FIRMWARE_INFO_BASE_ADDR, (uint8_t *)temp_version_files,
                       data_size);

    if (write_result < 0)
    {
        LOG_E("Failed to write to flash. Result: %d", write_result);
        return;
    }
    LOG_I("All flags cleared to 0 successfully.\n");

    // 5. Verify whether all the flags are zero.
    LOG_I("5. Verifying all flags are cleared to 0:\n");
    for (int i = 0; i < MAX_FIRMWARE_FILES; i++)
    {
        struct firmware_file_info temp_version;
        if (dfu_pan_get_firmware_file_info(i, &temp_version) == 0)
        {
            uint32_t needs_update_addr =
                FIRMWARE_INFO_BASE_ADDR +
                i * sizeof(struct firmware_file_info) +
                offsetof(struct firmware_file_info, needs_update);
            LOG_I("  Entry %d: needs_update=%d (addr: 0x%08x) %s", i,
                  temp_version.needs_update, needs_update_addr,
                  (temp_version.needs_update == 0) ? "OK" : "ERROR");
        }
        else
        {
            uint32_t needs_update_addr =
                FIRMWARE_INFO_BASE_ADDR +
                i * sizeof(struct firmware_file_info) +
                offsetof(struct firmware_file_info, needs_update);
            LOG_E("  Entry %d: read failed (addr: 0x%08x)", i,
                  needs_update_addr);
        }
    }
    // Clear all information
    dfu_pan_clear_files();
    LOG_I("=== OTA Update Flags Test Complete ===\n");
}

// Device registration interface implementation
int dfu_pan_register_device(const char *server_url,
                            const device_register_params_t *params)
{
    struct webclient_session *session = RT_NULL;
    char *request_body = RT_NULL;
    char *response_buffer = RT_NULL;
    int resp_status = 0;
    int ret = 0;
    char register_url[512];

    LOG_I("dfu_pan_register_device\n");

    // Validate input parameters
    if (server_url == NULL || params == NULL)
    {
        LOG_E("Invalid parameters for device registration");
        return -1;
    }

    // Validate required parameters
    if (!params->mac || !params->model || !params->solution ||
        !params->version || !params->ota_version || !params->chip_id)
    {
        LOG_E("Missing required parameters for device registration");
        return -1;
    }

    // Build registration URL
    rt_snprintf(register_url, sizeof(register_url), "%s/register", server_url);

    // Build request JSON body
    int body_size = 512; // Adjust according to actual needs
    request_body = rt_calloc(1, body_size);
    if (!request_body)
    {
        LOG_E("No memory for request body!");
        ret = -1;
        goto __exit;
    }

    // Build JSON request body with all required parameters
    int len =
        rt_snprintf(request_body, body_size,
                    "{"
                    "\"mac\":\"%s\","
                    "\"model\":\"%s\","
                    "\"solution\":\"%s\","
                    "\"version\":\"%s\","
                    "\"ota_version\":\"%s\","
                    "\"chip_id\":\"%s\"",
                    params->mac, params->model, params->solution,
                    params->version, params->ota_version, params->chip_id);

    // Add optional parameters
    if (params->screen_width)
    {
        len += rt_snprintf(request_body + len, body_size - len,
                           ",\"screen_width\":\"%s\"", params->screen_width);
    }
    if (params->screen_height)
    {
        len += rt_snprintf(request_body + len, body_size - len,
                           ",\"screen_height\":\"%s\"", params->screen_height);
    }
    if (params->flash_type)
    {
        len += rt_snprintf(request_body + len, body_size - len,
                           ",\"flash_type\":\"%s\"", params->flash_type);
    }

    len += rt_snprintf(request_body + len, body_size - len, "}");

    LOG_I("Register request body: \n");
    rt_kputs(request_body);
    LOG_I("\n");

    // Exit Bluetooth sniff mode
    exist_sniff_mode();

    // Create network session
    session = webclient_session_create(PAN_OTA_HEADER_BUFSZ);
    if (!session)
    {
        LOG_E("Create session failed!");
        ret = -1;
        goto __exit;
    }

    // Set request headers
    webclient_header_fields_add(session, "Content-Type: application/json\r\n");
    webclient_header_fields_add(session, "Content-Length: %d\r\n",
                                strlen(request_body));

    // Send POST request
    resp_status = webclient_post(session, register_url, request_body,
                                 strlen(request_body));
    rt_kputs(register_url);
    LOG_I("\n");
    LOG_I("Device registration response status: %d\n", resp_status);

    // Check response status
    if (resp_status != 200)
    {
        LOG_E("Device registration failed, response code: %d", resp_status);
        ret = -1;
        goto __exit;
    }

    // Read response
    response_buffer = rt_calloc(1, OTA_RECV_BUFF_SIZE);
    if (!response_buffer)
    {
        LOG_E("No memory for response buffer!");
        ret = -1;
        goto __exit;
    }

    int content_length = webclient_content_length_get(session);
    if (content_length > 0 && content_length <= OTA_RECV_BUFF_SIZE)
    {
        int bytes_read =
            webclient_read(session, response_buffer, content_length);
        if (bytes_read > 0)
        {
            LOG_I("Registration response: %s\n", response_buffer);

            // Parse response JSON to check registration success
            cJSON *root = cJSON_Parse(response_buffer);
            if (root)
            {
                cJSON *result_item = cJSON_GetObjectItem(root, "result");
                if (result_item && result_item->valueint == 200)
                {
                    LOG_I("Device registration successful");
                    ret = 0; // Success
                }
                else
                {
                    LOG_E("Server returned error for registration");
                    ret = -1; // Failure
                }
                cJSON_Delete(root);
            }
            else
            {
                LOG_E("Failed to parse registration response JSON");
                ret = -1;
            }
        }
    }

__exit:
    // Clean up resources
    if (session)
    {
        LOCK_TCPIP_CORE();
        webclient_close(session);
        UNLOCK_TCPIP_CORE();
    }

    if (request_body)
    {
        rt_free(request_body);
    }

    if (response_buffer)
    {
        rt_free(response_buffer);
    }

    return ret;
}
 