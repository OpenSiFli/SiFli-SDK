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
#ifndef _SDIO_PORTING_H_
#define _SDIO_PORTING_H_

#include "sdio_func.h"

typedef void(*aicwf_sdio_irq_handler_t)(struct sdio_func *);

int xhci_sdio_enable_func(struct sdio_func *func);
int xhci_sdio_disable_func(struct sdio_func *func);
void xhci_sdio_claim_host(struct sdio_func *func);
void xhci_sdio_release_host(struct sdio_func *func);
int xhci_sdio_claim_irq(struct sdio_func *func, aicwf_sdio_irq_handler_t handler);
int xhci_sdio_release_irq(struct sdio_func *func);
unsigned char xhci_sdio_readb(struct sdio_func *func, unsigned int addr, int *err_ret);
void xhci_sdio_writeb(struct sdio_func *func, unsigned char b, unsigned int addr, int *err_ret);
int xhci_sdio_readsb(struct sdio_func *func, void *dst, unsigned int addr, int count);
int xhci_sdio_writesb(struct sdio_func *func, unsigned int addr, void *src, int count);
int xhci_sdio_set_block_size(struct sdio_func *func, unsigned int blk_sz);

typedef int (*sdio_enable_t)(struct sdio_func *func);
typedef int (*sdio_disable_t)(struct sdio_func *func);
typedef void (*sdio_claim_host_t)(struct sdio_func *func);
typedef void (*sdio_release_host_t)(struct sdio_func *func);
typedef int (*sdio_claim_irq_t)(struct sdio_func *func, aicwf_sdio_irq_handler_t handler);
typedef int (*sdio_release_irq_t)(struct sdio_func *func);
typedef unsigned char (*sdio_readb_t)(struct sdio_func *func, unsigned int addr, int *err_ret);
typedef void (*sdio_writeb_t)(struct sdio_func *func, unsigned char b, unsigned int addr, int *err_ret);
typedef int (*sdio_readsb_t)(struct sdio_func *func, void *dst, unsigned int addr, int count);
typedef int (*sdio_writesb_t)(struct sdio_func *func, unsigned int addr, void *src, int count);
typedef int (*sdio_set_block_size_t)(struct sdio_func *func, unsigned int blk_sz);

typedef struct
{
    sdio_enable_t enable;
    sdio_disable_t disable;
    sdio_claim_host_t claim_host;
    sdio_release_host_t release_host;
    sdio_claim_irq_t claim_irq;
    sdio_release_irq_t release_irq;
    sdio_readb_t readb;
    sdio_writeb_t writeb;
    sdio_readsb_t readsb;
    sdio_writesb_t writesb;
    sdio_set_block_size_t set_block_size;
} sdio_porting_ops_t;

extern const sdio_porting_ops_t sdio_porting_ops;

#endif /* _SDIO_PORTING_H_ */
