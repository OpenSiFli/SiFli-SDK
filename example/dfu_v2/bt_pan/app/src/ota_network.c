/**
 * @file ota_network.c
 * @brief PAN OTA V2 — Server Registration & Version Query Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <webclient.h>
#include <cJSON.h>

#include "bf0_hal.h"
#include "bts2_app_inc.h"
#include "ble_connection_manager.h"
#include "bt_connection_manager.h"
#include "bts2_app_pan.h"

/* lwip — for LOCK_TCPIP_CORE / UNLOCK_TCPIP_CORE */
#include "lwip/tcpip.h"

/* DFU V2 API */
#include "dfu_macro.h"
#include "dfu_fwinfo.h"

#include "ota_network.h"

#define LOG_TAG "ota_net"
#include <ulog.h>

/*============================================================================
 * Configuration
 *============================================================================*/

#define OTA_SERVER_URL      "https://ota.sifli.com"
#define OTA_HEADER_BUFSZ    1536
#define OTA_RECV_BUFSZ      1536

/*============================================================================
 * Internal: MAC address and chip ID
 *============================================================================*/

static char s_mac_str[20];
static char s_chip_id[40];
ALIGN(4) static uint8_t s_sha256_result[32] = {0};

static void hex_to_ascii(uint8_t n, char *str)
{
    uint8_t hi = (n >> 4);
    str[0] = (hi >= 10) ? (hi + 'a' - 10) : (hi + '0');
    uint8_t lo = (n & 0x0F);
    str[1] = (lo >= 10) ? (lo + 'a' - 10) : (lo + '0');
}

char *ota_get_mac_address(void)
{
    if (s_mac_str[0] == '\0')
    {
        BTS2S_ETHER_ADDR addr = bt_pan_get_mac_address(NULL);
        uint8_t *p = (uint8_t *)&addr;
        rt_snprintf(s_mac_str, sizeof(s_mac_str),
                    "%02x:%02x:%02x:%02x:%02x:%02x",
                    *(p + 1), *p, *(p + 3), *(p + 2), *(p + 5), *(p + 4));
    }
    return s_mac_str;
}

char *ota_get_chip_id(void)
{
    if (s_chip_id[0] == '\0')
    {
        BTS2S_ETHER_ADDR addr = bt_pan_get_mac_address(NULL);
        HAL_HASH_reset();
        HAL_HASH_init(NULL, HASH_ALGO_SHA256, 0);
        HAL_HASH_run((uint8_t *)&addr, sizeof(addr), 1);
        HAL_HASH_result(s_sha256_result);

        int j = 0;
        for (int i = 0; i < 16; i++, j += 2)
        {
            /* UUID format: 12345678-1234-1234-1234-123456789012 */
            if (i == 4 || i == 6 || i == 8 || i == 10)
                s_chip_id[j++] = '-';
            hex_to_ascii(s_sha256_result[i], &s_chip_id[j]);
        }
    }
    return s_chip_id;
}

/*============================================================================
 * Internal: Version string comparison
 *============================================================================*/

static int compare_version_strings(const char *v1, const char *v2)
{
    if (!v1 || !v2)
        return -2;

    BOOL v1_prefix = (v1[0] == 'v' || v1[0] == 'V');
    BOOL v2_prefix = (v2[0] == 'v' || v2[0] == 'V');

    if (v1_prefix != v2_prefix)
        return -2;

    const char *p1 = v1_prefix ? v1 + 1 : v1;
    const char *p2 = v2_prefix ? v2 + 1 : v2;

    if (*p1 == '\0' || *p2 == '\0')
        return -2;

    char buf1[32], buf2[32];
    strncpy(buf1, p1, sizeof(buf1) - 1); buf1[sizeof(buf1) - 1] = '\0';
    strncpy(buf2, p2, sizeof(buf2) - 1); buf2[sizeof(buf2) - 1] = '\0';

    char *t1 = buf1, *t2 = buf2;
    char *tok1, *tok2;

    while (1)
    {
        tok1 = strtok_r(t1, ".", &t1);
        tok2 = strtok_r(t2, ".", &t2);

        if (!tok1 && !tok2) return 0;

        int n1 = tok1 ? atoi(tok1) : 0;
        int n2 = tok2 ? atoi(tok2) : 0;

        if (n1 > n2) return 1;
        if (n1 < n2) return -1;
    }
}

/*============================================================================
 * Public API: Device Registration
 *============================================================================*/

int ota_register_device(void)
{
    struct webclient_session *session = NULL;
    char *request_body = NULL;
    char *response_buf = NULL;
    int ret = 0;

    const char *mac = ota_get_mac_address();
    const char *chip_id = ota_get_chip_id();

    /* Build registration URL */
    char reg_url[512];
    rt_snprintf(reg_url, sizeof(reg_url), "%s/register", OTA_SERVER_URL);

    /* Build JSON body */
    request_body = rt_calloc(1, 512);
    if (!request_body) { ret = -1; goto __exit; }

    const char *model = "unknown";
    const char *solution = "unknown";

#ifdef BSP_USING_BOARD_SF32LB52_LCD_N16R8
    model = "sf32lb52-lcd-n16r8";
    solution = "SF32LB52_LCD_N16R8_TFT_CO5300";
#elif defined(BSP_USING_BOARD_SF32LB52_LCHSPI_ULP)
    model = "sf32lb52-lchspi-ulp";
    solution = "SF32LB52_ULP_NOR_TFT_CO5300";
#elif defined(BSP_USING_BOARD_SF32LB52_NANO_A128R16)
    model = "sf32lb52-nano-a128r16";
    solution = "SF32LB52_NANO_A128R16_TFT_CO5300";
#endif

    rt_snprintf(request_body, 512,
        "{\"mac\":\"%s\",\"model\":\"%s\",\"solution\":\"%s\","
        "\"version\":\"v1.0\",\"ota_version\":\"v1.0\",\"chip_id\":\"%s\"}",
        mac, model, solution, chip_id);

    LOG_I("Register: %s", reg_url);

    session = webclient_session_create(OTA_HEADER_BUFSZ);
    if (!session) { ret = -1; goto __exit; }

    webclient_header_fields_add(session, "Content-Type: application/json\r\n");
    webclient_header_fields_add(session, "Content-Length: %d\r\n",
                                strlen(request_body));

    int status = webclient_post(session, reg_url, request_body,
                                strlen(request_body));
    if (status != 200)
    {
        LOG_E("Register failed, HTTP %d", status);
        ret = -1;
        goto __exit;
    }

    /* Parse response */
    response_buf = rt_calloc(1, OTA_RECV_BUFSZ);
    if (response_buf)
    {
        int content_len = webclient_content_length_get(session);
        if (content_len > 0 && content_len <= OTA_RECV_BUFSZ)
        {
            webclient_read(session, response_buf, content_len);
            cJSON *root = cJSON_Parse(response_buf);
            if (root)
            {
                cJSON *result = cJSON_GetObjectItem(root, "result");
                if (!result || result->valueint != 200)
                    ret = -1;
                cJSON_Delete(root);
            }
        }
    }

__exit:
    if (session) { LOCK_TCPIP_CORE(); webclient_close(session); UNLOCK_TCPIP_CORE(); }
    if (request_body) rt_free(request_body);
    if (response_buf) rt_free(response_buf);
    return ret;
}

/*============================================================================
 * Public API: Build Query URL
 *============================================================================*/

char *ota_build_query_url(const char *chip_id)
{
    static char url[512] = {0};

#ifdef BSP_USING_BOARD_SF32LB52_LCD_N16R8
    rt_snprintf(url, sizeof(url),
        "%s/v2/example/pan_ota/SF32LB52_LCD_N16R8_TFT_CO5300/"
        "sf32lb52-lcd-n16r8?chip_id=%s&version=latest",
        OTA_SERVER_URL, chip_id);
#elif defined(BSP_USING_BOARD_SF32LB52_LCHSPI_ULP)
    rt_snprintf(url, sizeof(url),
        "%s/v2/example/pan_ota/SF32LB52_ULP_NOR_TFT_CO5300/"
        "sf32lb52-lchspi-ulp?chip_id=%s&version=latest",
        OTA_SERVER_URL, chip_id);
#elif defined(BSP_USING_BOARD_SF32LB52_NANO_A128R16)
    rt_snprintf(url, sizeof(url),
        "%s/v2/example/pan_ota/SF32LB52_NANO_A128R16_TFT_CO5300/"
        "sf32lb52-nano-a128r16?chip_id=%s&version=latest",
        OTA_SERVER_URL, chip_id);
#else
    rt_snprintf(url, sizeof(url),
        "%s/v2/example/pan_ota/unknown/unknown?chip_id=%s&version=latest",
        OTA_SERVER_URL, chip_id);
#endif

    return url;
}

/*============================================================================
 * Public API: Query Latest Version
 *============================================================================*/

int ota_query_latest_version(const char *query_url,
                             const char *current_version,
                             char *latest_version_out,
                             size_t name_size)
{
    struct webclient_session *session = NULL;
    char *buffer = NULL;
    int ret = 0;
    int content_pos = 0;

    if (!query_url) { LOG_E("NULL URL"); return -1; }

    LOG_I("Query: %s", query_url);

    session = webclient_session_create(OTA_HEADER_BUFSZ);
    if (!session) { return -1; }

    buffer = rt_calloc(1, OTA_RECV_BUFSZ);
    if (!buffer) { ret = -1; goto __exit; }

    int status = webclient_get(session, query_url);
    if (status != 200)
    {
        LOG_E("HTTP %d", status);
        ret = -1;
        goto __exit;
    }

    if (session->content_length > OTA_RECV_BUFSZ)
    {
        LOG_E("Response too large: %d", session->content_length);
        ret = -1;
        goto __exit;
    }

    /* Read response body */
    while (content_pos < session->content_length)
    {
        int to_read = session->content_length - content_pos;
        if (content_pos + to_read > OTA_RECV_BUFSZ)
            to_read = OTA_RECV_BUFSZ - content_pos;
        if (to_read <= 0) break;

        int n = webclient_read(session, buffer + content_pos, to_read);
        if (n <= 0) break;
        content_pos += n;
    }

    if (content_pos <= 0) { ret = -1; goto __exit; }

    /* Parse JSON */
    cJSON *root = cJSON_Parse(buffer);
    if (!root)
    {
        LOG_E("JSON parse failed");
        ret = -1;
        goto __exit;
    }

    cJSON *result_item = cJSON_GetObjectItem(root, "result");
    if (!result_item || result_item->valueint != 200)
    {
        LOG_E("Server error: %d", result_item ? result_item->valueint : -1);
        cJSON_Delete(root);
        ret = -1;
        goto __exit;
    }

    /* Extract version object (data may be array or object) */
    cJSON *data_item = cJSON_GetObjectItem(root, "data");
    cJSON *version_obj = NULL;
    if (data_item && cJSON_IsArray(data_item))
        version_obj = cJSON_GetArrayItem(data_item, 0);
    else if (data_item && cJSON_IsObject(data_item))
        version_obj = data_item;

    if (!version_obj)
    {
        LOG_E("No version data");
        cJSON_Delete(root);
        ret = -1;
        goto __exit;
    }

    cJSON *name_item = cJSON_GetObjectItem(version_obj, "name");
    if (!name_item || !cJSON_IsString(name_item))
    {
        LOG_E("No version name");
        cJSON_Delete(root);
        ret = -1;
        goto __exit;
    }

    /* Compare versions */
    int cmp = compare_version_strings(name_item->valuestring, current_version);
    LOG_I("Server: %s, Current: %s, cmp=%d",
          name_item->valuestring, current_version, cmp);

    if (cmp > 0)
    {
        /* Newer version available — parse firmware files */
        if (latest_version_out && name_size > 0)
        {
            strncpy(latest_version_out, name_item->valuestring, name_size - 1);
            latest_version_out[name_size - 1] = '\0';
        }

        cJSON *files_array = cJSON_GetObjectItem(version_obj, "files");
        if (!files_array || !cJSON_IsArray(files_array))
        {
            LOG_E("No files array");
            cJSON_Delete(root);
            ret = -1;
            goto __exit;
        }

        /* Clear old entries before writing new ones */
        dfu_fwinfo_clear();

        int file_count = 0;
        cJSON *file_item = NULL;
        cJSON_ArrayForEach(file_item, files_array)
        {
            if (file_count >= DFU_MAX_FW_FILES)
            {
                LOG_W("Too many files, max %d", DFU_MAX_FW_FILES);
                break;
            }

            cJSON *file_id_item = cJSON_GetObjectItem(file_item, "file_id");
            if (!file_id_item) continue;

            cJSON *fn   = cJSON_GetObjectItem(file_item, "file_name");
            cJSON *url  = cJSON_GetObjectItem(file_item, "url");
            cJSON *addr = cJSON_GetObjectItem(file_item, "addr");
            cJSON *sz   = cJSON_GetObjectItem(file_item, "file_size");
            cJSON *crc  = cJSON_GetObjectItem(file_item, "crc32");
            cJSON *rsz  = cJSON_GetObjectItem(file_item, "region_size");

            if (!fn || !cJSON_IsString(fn) || !url || !cJSON_IsString(url))
                continue;

            struct dfu_fw_info info;
            memset(&info, 0, sizeof(info));

            strncpy(info.name, fn->valuestring, sizeof(info.name) - 1);
            strncpy(info.url, url->valuestring, sizeof(info.url) - 1);

            if (addr && cJSON_IsString(addr))
                info.addr = strtoul(addr->valuestring, NULL, 0);

            if (sz)
            {
                if (cJSON_IsNumber(sz))    info.size = sz->valueint;
                else if (cJSON_IsString(sz)) info.size = atoi(sz->valuestring);
            }

            if (crc && cJSON_IsString(crc))
                info.crc32 = strtoul(crc->valuestring, NULL, 0);

            if (rsz && cJSON_IsString(rsz))
                info.region_size = strtoul(rsz->valuestring, NULL, 0);

            if (cJSON_IsNumber(file_id_item))
                info.file_id = file_id_item->valueint;
            else if (cJSON_IsString(file_id_item))
                info.file_id = atoi(file_id_item->valuestring);

            LOG_I("fw[%d]: %s addr=0x%08X size=%u crc=0x%08X",
                  file_count, info.name, info.addr, info.size, info.crc32);

            /* Write to flash via V2 fwinfo API */
            if (dfu_fwinfo_set(file_count, &info) != 0)
            {
                LOG_E("Failed to write fwinfo[%d]", file_count);
                cJSON_Delete(root);
                ret = -1;
                goto __exit;
            }

            file_count++;
        }

        LOG_I("Saved %d firmware file(s) to flash", file_count);
        ret = 1; /* newer version available */
    }
    else
    {
        ret = 0; /* current is latest */
    }

    cJSON_Delete(root);

__exit:
    if (session) { LOCK_TCPIP_CORE(); webclient_close(session); UNLOCK_TCPIP_CORE(); }
    if (buffer) rt_free(buffer);
    return ret;
}