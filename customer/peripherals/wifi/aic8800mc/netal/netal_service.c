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

#include <string.h>
#include "osal_debug.h"
#include "netal_service.h"
#include "netal_porting.h"
#include "rwnx_platform.h"
#include "aicwf_txrxif.h"

netal_eth_env_t netal_eth_env = {NULL,};
netal_eth_env_t *p_netal_eth_env = &netal_eth_env;


static int netal_eth_protocol_send(uint8_t *buff, int len)
{
    int ret = 0;
    if (g_rwnx_plat && g_rwnx_plat->sdiodev && g_rwnx_plat->sdiodev->bus_if)
    {
        ret = aicwf_frame_tx(g_rwnx_plat->sdiodev->bus_if, buff, (uint32_t)len);
        if (ret)
        {
            DBG_NETAL_ERR("aicwf_frame_tx err:%d\r\n", ret);
        }
    }
    return ret;
}

int netal_eth_protocol_init(uint8_t *init_mac)
{
    int ret = -1;
    const uint8_t default_mac[] = {0x88, 0x00, 0xA6, 0x9C, 0x5A, 0x78};
    p_netal_eth_env->ops.eth_send = netal_eth_protocol_send;
    if (init_mac)
    {
        rtos_memcpy(&p_netal_eth_env->mac_addr[0], init_mac, 6);
    }
    else
    {
        rtos_memcpy(&p_netal_eth_env->mac_addr[0], default_mac, 6);
    }
    if (netal_porting_ops.eth_init)
    {
        ret = netal_porting_ops.eth_init(p_netal_eth_env);
    }
    else
    {
        DBG_NETAL_ERR("not implement ops: eth_init\r\n");
    }
    return ret;
}

int netal_eth_protocol_recv(void *buff, int len)
{
    int ret = 0;
    do
    {
        if (p_netal_eth_env->eth_dev == NULL)
        {
            DBG_NETAL_ERR("eth_dev uninited\r\n");
            ret = -1;
            break;
        }
        if (netal_porting_ops.eth_recv == NULL)
        {
            DBG_NETAL_ERR("not implement ops: eth_recv\r\n");
            ret = -2;
            break;
        }
        ret = netal_porting_ops.eth_recv(p_netal_eth_env, buff, len);
    }
    while (0);
    return ret;
}

int netal_eth_protocol_linkchange(int link_up)
{
    int ret = 0;
    do
    {
        if (p_netal_eth_env->eth_dev == NULL)
        {
            DBG_NETAL_ERR("eth_dev uninited\r\n");
            ret = -1;
            break;
        }
        if (netal_porting_ops.eth_linkchange == NULL)
        {
            DBG_NETAL_ERR("not implement ops: eth_linkchange\r\n");
            ret = -2;
            break;
        }
        ret = netal_porting_ops.eth_linkchange(p_netal_eth_env, link_up);
    }
    while (0);
    return ret;
}

int netal_eth_protocol_addrinfoset(uint32_t *ip, uint32_t *nm, uint32_t *gw)
{
    int ret = 0;
    do
    {
        if (p_netal_eth_env->eth_dev == NULL)
        {
            DBG_NETAL_ERR("eth_dev uninited\r\n");
            ret = -1;
            break;
        }
        if (netal_porting_ops.eth_addrinfoset == NULL)
        {
            DBG_NETAL_ERR("not implement ops: eth_addrinfoset\r\n");
            ret = -2;
            break;
        }
        ret = netal_porting_ops.eth_addrinfoset(p_netal_eth_env, ip, nm, gw);
    }
    while (0);
    return ret;
}
