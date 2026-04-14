/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SDIO_CONFIG_H__
#define __SDIO_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SDIO_BUS_CONFIG                                  \
    {                                                    \
        .Instance = SDIO2,                               \
        .name     = "sd1",                               \
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

