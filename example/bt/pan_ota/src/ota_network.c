/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "bts2_app_inc.h"
#include "ble_connection_manager.h"
#include "bt_connection_manager.h"
#include "bts2_app_pan.h"
#include "dfu_pan_macro.h"
#include "bt_pan_ota.h"
char mac_address_string[20];
char client_id_string[40];
ALIGN(4) uint8_t g_sha256_result[32] = {0};

void hash_run(uint8_t algo, uint8_t *raw_data, uint32_t raw_data_len,
              uint8_t *result, uint32_t result_len)
{
    /* Rest hash block. */
    HAL_HASH_reset();
    /* Initialize AES Hash hardware block. */
    HAL_HASH_init(NULL, algo, 0);
    /* Do hash. HAL_HASH_run will block until hash finish. */
    HAL_HASH_run(raw_data, raw_data_len, 1);
    /* Get hash result. */
    HAL_HASH_result(result);
}

void hex_2_asc(uint8_t n, char *str)
{
    uint8_t i = (n >> 4);
    if (i >= 10)
        *str = i + 'a' - 10;
    else
        *str = i + '0';
    str++, i = n & 0xf;
    if (i >= 10)
        *str = i + 'a' - 10;
    else
        *str = i + '0';
}
char *get_mac_address()
{
    if (mac_address_string[0] == '\0')
    {
        BTS2S_ETHER_ADDR addr = bt_pan_get_mac_address(NULL);
        uint8_t *p = (uint8_t *)&(addr);

        rt_snprintf((char *)mac_address_string, 20,
                    "%02x:%02x:%02x:%02x:%02x:%02x", *(p + 1),*p, *(p + 3),
                    *(p + 2), *(p + 5), *(p + 4));
    }
    return (&(mac_address_string[0]));
}

char *get_client_id()
{
    if (client_id_string[0] == '\0')
    {
        int i, j = 0;
        BTS2S_ETHER_ADDR addr = bt_pan_get_mac_address(NULL);
        hash_run(HASH_ALGO_SHA256, (uint8_t *)&addr, sizeof(addr),
                 g_sha256_result, sizeof(g_sha256_result));
        for (i = 0; i < 16; i++, j += 2)
        {
            // 12345678-1234-1234-1234-123456789012
            if (i == 4 || i == 6 || i == 8 || i == 10)
            {
                client_id_string[j++] = '-';
            }
            hex_2_asc(g_sha256_result[i], &client_id_string[j]);
        }
        rt_kprintf(client_id_string);
    }
    return (&(client_id_string[0]));
}

int register_device_with_server(void)
{
    device_register_params_t reg_params = {0};
    
    // 填充注册参数
    reg_params.mac = get_mac_address();

    #ifdef BSP_USING_BOARD_SF32LB52_LCD_N16R8
    reg_params.model = "sf32lb52-lcd-n16r8";
    reg_params.solution = "SF32LB52_LCD_N16R8_TFT_CO5300";
    #elif defined(BSP_USING_BOARD_SF32LB52_LCHSPI_ULP)
    reg_params.model = "sf32lb52-lchspi-ulp";
    reg_params.solution = "SF32LB52_ULP_NOR_TFT_CO5300";
    #elif defined(BSP_USING_BOARD_SF32LB52_NANO_A128R16)
    reg_params.model = "sf32lb52-nano-a128r16";
    reg_params.solution = "SF32LB52_NANO_A128R16_TFT_CO5300";
    #endif

    reg_params.version = "v1.0"; // 当前固件版本
    reg_params.ota_version = "v1.0"; // 当前OTA版本
    reg_params.chip_id = get_client_id();
    
    // 服务器注册设备URL
    const char* ota_server_url = "https://ota.sifli.com";
    
    // 执行设备注册
    int result = dfu_pan_register_device(ota_server_url, &reg_params);
    
    if (result == 0) {
        rt_kprintf("Device registered successfully with chip_id: %s\n", get_client_id());
    } else {
        rt_kprintf("Device registration failed\n");
    }
    
    return result;
}

// Construct the OTA query URL
char* build_ota_query_url(const char* chip_id)
{
    static char query_url[512] = {0};

    #ifdef BSP_USING_BOARD_SF32LB52_LCD_N16R8
    snprintf(query_url, sizeof(query_url), 
             "https://ota.sifli.com/v2/example/pan_ota/SF32LB52_LCD_N16R8_TFT_CO5300/sf32lb52-lcd-n16r8?chip_id=%s&version=latest",
             chip_id);

    #elif defined(BSP_USING_BOARD_SF32LB52_LCHSPI_ULP)
    snprintf(query_url, sizeof(query_url), 
             "https://ota.sifli.com/v2/example/pan_ota/SF32LB52_ULP_NOR_TFT_CO5300/sf32lb52-lchspi-ulp?chip_id=%s&version=latest",
             chip_id);
    #elif defined(BSP_USING_BOARD_SF32LB52_NANO_A128R16)
    snprintf(query_url, sizeof(query_url), 
             "https://ota.sifli.com/v2/example/pan_ota/SF32LB52_NANO_A128R16_TFT_CO5300/sf32lb52-nano-a128r16?chip_id=%s&version=latest",
             chip_id);

    #endif

    return query_url;
}