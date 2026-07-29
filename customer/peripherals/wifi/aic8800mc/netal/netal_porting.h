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

#ifndef _NETAL_PORTING_H_
#define _NETAL_PORTING_H_

#include "aicwf_types.h"

#include "netal_service.h"

/** @addtogroup RTOS
 *  @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/* DEFINITIONS
****************************************************************************************
*/

typedef int (*netal_eth_init_t)(netal_eth_env_t *net_env);
typedef int (*netal_eth_recv_t)(netal_eth_env_t *net_env, void *buff, int len);
typedef int (*netal_eth_linkchange_t)(netal_eth_env_t *net_env, int link_up);
typedef int (*netal_eth_addrinfoset_t)(netal_eth_env_t *net_env, uint32_t *ip, uint32_t *nm, uint32_t *gw);

/*
 * FUNCTIONS
 ****************************************************************************************
 */

typedef struct
{
    netal_eth_init_t eth_init;
    netal_eth_recv_t eth_recv;
    netal_eth_linkchange_t eth_linkchange;
    netal_eth_addrinfoset_t eth_addrinfoset;
} netal_porting_ops_t;

extern const netal_porting_ops_t netal_porting_ops;

void netif_lwip_reset(void);

#ifdef __cplusplus
}
#endif

/*\@}*/

#endif  //#ifndef _NETAL_PORTING_H_
