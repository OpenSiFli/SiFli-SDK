/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OTA_NETWORK_H__
#define OTA_NETWORK_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取设备唯一标识符
 * @return 设备ID字符串指针
 */
char* get_client_id(void);

/**
 * @brief 构建OTA查询URL
 * @param chip_id 设备ID
 * @return OTA查询URL字符串指针
 */
char* build_ota_query_url(const char* chip_id);

/**
 * @brief 向服务器注册设备
 * @return 注册结果，0-成功，其他-失败
 */
int register_device_with_server(void);

#ifdef __cplusplus
}
#endif

#endif // OTA_NETWORK_H__