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

#ifndef _NETAL_SERVICE_H_
#define _NETAL_SERVICE_H_

#include "aicwf_types.h"

/** @addtogroup RTOS
 *  @{
 */

#ifdef __cplusplus
extern "C" {
#endif


/* DEFINITIONS
 ****************************************************************************************
 */

typedef int (*netal_eth_send_func_t)(uint8_t *buff, int len);

typedef struct
{
    netal_eth_send_func_t eth_send;
} netal_service_ops_t;

typedef struct
{
    void *eth_dev;
    netal_service_ops_t ops;
    uint8_t mac_addr[6];
} netal_eth_env_t;

extern netal_eth_env_t *p_netal_eth_env;

/*
 * FUNCTIONS
 ****************************************************************************************
 */

int netal_eth_protocol_init(uint8_t *init_mac);
int netal_eth_protocol_recv(void *buff, int len);
int netal_eth_protocol_linkchange(int link_up);
int netal_eth_protocol_addrinfoset(uint32_t *ip, uint32_t *nm, uint32_t *gw);

#ifdef __cplusplus
}
#endif

/*\@}*/

#endif /* _NETAL_SERVICE_H_ */
