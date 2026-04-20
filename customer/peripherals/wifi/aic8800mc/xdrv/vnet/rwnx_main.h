/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _RWNX_MAIN_H_
#define _RWNX_MAIN_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "aicwf_config.h"
#include "osal_service.h"
#include "rwnx_defs.h"
#include "rwnx_platform.h"

typedef struct
{
    int8_t  ota_status;
    int8_t  wifi_proc_fail;
    int8_t  aic_mode_status;
    uint8_t wlan_status;
    uint8_t wlan_conn_fail_result;
    uint8_t mac_addr[MAC_ADDR_LEN];
    uint32_t ip;
    uint32_t mk;
    uint32_t gw;
    uint32_t dns1;
    uint32_t dns2;
} aicwf_custom_msg_vnet_t;

extern aicwf_custom_msg_vnet_t g_custom_msg_vnet;

int rwnx_vnet_init(struct rwnx_plat *rwnx_plat, void **platform_data);
void rwnx_vnet_deinit(struct rwnx_hw *rwnx_hw);

#endif // _RWNX_MAIN_H_
