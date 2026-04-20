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

#include <rtthread.h>
#include <rtdevice.h>
#include "sdio_func.h"
#include "aicwf_types.h"
#include "osal_debug.h"
#include "sdio_porting.h"


extern struct rt_mmcsd_card *g_wifi_if_sdio;
extern struct sdio_func g_wifi_if_sdio_func1;

int xhci_sdio_enable_func(struct sdio_func *func)
{
    int ret = -1;
    int func_num = func->num;
    DBG_SDIO_INF("%s\n", __func__);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        struct rt_sdio_function *sdio_func_ptr = g_wifi_if_sdio->sdio_function[func_num];
        ret = (int)sdio_enable_func(sdio_func_ptr);
    }
    return ret;
}

int xhci_sdio_disable_func(struct sdio_func *func)
{
    int ret = -1;
    int func_num = func->num;
    DBG_SDIO_INF("%s\n", __func__);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        struct rt_sdio_function *sdio_func_ptr = g_wifi_if_sdio->sdio_function[func_num];
        ret = (int)sdio_disable_func(sdio_func_ptr);
    }
    return ret;
}

void xhci_sdio_claim_host(struct sdio_func *func)
{
    if (g_wifi_if_sdio)
    {
        //DBG_SDIO_INF("claim host=%x\n", g_wifi_if_sdio->host);
        mmcsd_host_lock(g_wifi_if_sdio->host);
    }
}

void xhci_sdio_release_host(struct sdio_func *func)
{
    if (g_wifi_if_sdio)
    {
        mmcsd_host_unlock(g_wifi_if_sdio->host);
        //DBG_SDIO_INF("release host=%x\n", g_wifi_if_sdio->host);
    }
}

void rtt_sdio_irq_handler(struct rt_sdio_function *rt_func)
{
    int func_num = rt_func->num;
    struct sdio_func *func1 = &g_wifi_if_sdio_func1;
    DBG_SDIO_VRB("%s, %d\n", __func__, func_num);
    if ((func_num == func1->num) && func1->irq_handler)
    {
        func1->irq_handler(func1);
    }
}

int xhci_sdio_claim_irq(struct sdio_func *func, aicwf_sdio_irq_handler_t handler)
{
    int ret = 0;
    int func_num = func->num;
    if (func->irq_handler == NULL)
    {
        func->irq_handler = handler;
    }
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        ret = (int)sdio_attach_irq(g_wifi_if_sdio->sdio_function[func_num], rtt_sdio_irq_handler);
    }
    return ret;
}

int xhci_sdio_release_irq(struct sdio_func *func)
{
    int ret = 0;
    int func_num = func->num;
    if (func->irq_handler)
    {
        func->irq_handler = NULL;
    }
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        ret = (int)sdio_detach_irq(g_wifi_if_sdio->sdio_function[func_num]);
    }
    return ret;
}

unsigned char xhci_sdio_readb(struct sdio_func *func, unsigned int addr, int *err_ret)
{
    rt_int32_t ret;
    rt_uint8_t result = 0;
    int func_num = func->num;
    DBG_SDIO_VRB("%s addr:0x%x\n", __func__, addr);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        result = sdio_io_readb(g_wifi_if_sdio->sdio_function[func_num], addr, &ret);
        if (ret != 0)
        {
            DBG_SDIO_ERR("%s [0x%x, 0x%x] error: %x\n", __func__, func, addr, ret);
        }
        else
        {
            DBG_SDIO_VRB("%s res=0x%x\n", __func__, result);
        }
        if (err_ret)
        {
            *err_ret = ret;
        }
    }
    return result;
}


void xhci_sdio_writeb(struct sdio_func *func, unsigned char b, unsigned int addr, int *err_ret)
{
    rt_uint8_t result = 0;
    int func_num = func->num;
    DBG_SDIO_VRB("%s addr=0x%x, b = %x\n", __func__, addr, b);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        result = sdio_io_writeb(g_wifi_if_sdio->sdio_function[func_num], addr, b);
        if (result != 0)
        {
            DBG_SDIO_ERR("%s [0x%x, 0x%x] error: %x\n", __func__, func, addr, result);
        }
        if (err_ret)
        {
            *err_ret = result;
        }
    }
}

int xhci_sdio_readsb(struct sdio_func *func, void *dst, unsigned int addr, int count)
{
    int ret = 0;
    int func_num = func->num;
    DBG_SDIO_VRB("%s, %d, cnt=%d\n", __func__, func_num, count);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        ret = sdio_io_read_multi_fifo_b(g_wifi_if_sdio->sdio_function[func_num], (rt_uint32_t)addr, dst, count);
    }
    return ret;
}

int xhci_sdio_writesb(struct sdio_func *func, unsigned int addr, void *src, int count)
{
    int ret = 0;
    int func_num = func->num;
    DBG_SDIO_VRB("%s, %d, cnt=%d\n", __func__, func_num, count);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        ret = sdio_io_write_multi_fifo_b(g_wifi_if_sdio->sdio_function[func_num], (rt_uint32_t)addr, src, count);
    }
    return ret;
}

int xhci_sdio_set_block_size(struct sdio_func *func, unsigned int blk_sz)
{
    int ret = 0;
    int func_num = func->num;
    DBG_SDIO_INF("%s, %d, blksz=%d\n", __func__, func_num, blk_sz);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        ret = (int)sdio_set_block_size(g_wifi_if_sdio->sdio_function[func_num], (rt_uint32_t)blk_sz);
    }
    return ret;
}


const sdio_porting_ops_t sdio_porting_ops =
{
    .enable = xhci_sdio_enable_func,
    .disable = xhci_sdio_disable_func,
};

