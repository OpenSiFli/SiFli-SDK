/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SDIO_CONFIG_H__
#define __SDIO_CONFIG_H__

#include <rtconfig.h>
#include "bf0_hal.h"
#include "dma_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SDIO1_BUS_CONFIG                                 \
    {                                                    \
        .Instance = SDIO1,                               \
        .name     = "sd0",                               \
        .irqn = SDMMC1_IRQn,                             \
        .rcc_mod = RCC_MOD_SDMMC1,                      \
        .mem_base = SDMMC1_MEM_BASE,                     \
        .dma_rx.dma_irq_prio = SDMMC1_DMA_IRQ_PRIO,      \
        .dma_tx.dma_irq_prio = SDMMC1_DMA_IRQ_PRIO,      \
        .dma_rx.Instance = SDMMC1_DMA_INSTANCE,          \
        .dma_rx.request = SDMMC1_DMA_REQUEST,            \
        .dma_rx.dma_irq = SDMMC1_DMA_IRQ,                \
        .dma_tx.Instance = SDMMC1_DMA_INSTANCE,          \
        .dma_tx.request = SDMMC1_DMA_REQUEST,            \
        .dma_tx.dma_irq = SDMMC1_DMA_IRQ,                \
    }


#define SDIO2_BUS_CONFIG                                 \
    {                                                    \
        .Instance = SDIO2,                               \
        .name     = "sd1",                               \
        .irqn = SDMMC2_IRQn,                             \
        .rcc_mod = RCC_MOD_SDMMC2,                      \
        .mem_base = SDMMC2_MEM_BASE,                     \
        .dma_rx.dma_irq_prio = SDMMC2_DMA_IRQ_PRIO,      \
        .dma_tx.dma_irq_prio = SDMMC2_DMA_IRQ_PRIO,      \
        .dma_rx.Instance = SDMMC2_DMA_INSTANCE,          \
        .dma_rx.request = SDMMC2_DMA_REQUEST,            \
        .dma_rx.dma_irq = SDMMC2_DMA_IRQ,                \
        .dma_tx.Instance = SDMMC2_DMA_INSTANCE,          \
        .dma_tx.request = SDMMC2_DMA_REQUEST,            \
        .dma_tx.dma_irq = SDMMC2_DMA_IRQ,                \
    }

#ifdef __cplusplus
}
#endif

#endif /*__SDIO_CONFIG_H__ */

