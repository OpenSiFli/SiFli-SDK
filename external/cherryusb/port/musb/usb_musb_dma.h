/*
 * Copyright (c) 2026, SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DMA engine API for the Mentor/Inventra MUSB controller.
 * Only available when CONFIG_USB_MUSB_DMA is defined (see usb_dma_musb.c).
 */
#ifndef __USB_MUSB_DMA_H__
#define __USB_MUSB_DMA_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_USB_MUSB_DMA

#ifndef CONFIG_USB_MUSB_DMA_CH_NUM
//TODO: actually there're only 7 channels available for DMA 0~6, not 1~7
#define CONFIG_USB_MUSB_DMA_CH_NUM 8
#endif

#ifndef CONFIG_USB_MUSB_DMA_MIN_LEN
#define CONFIG_USB_MUSB_DMA_MIN_LEN 32
#endif

/* DMAINTR bit n = channel n. Channel 0 is unused (EP0 is PIO). */
#define USB_DMA_CH_MASK 0xFE

bool musb_dma_can_use(const uint8_t *buf, uint32_t len);
bool musb_dma_ch_is_active(uint8_t ch);
void musb_dma_ch_free(uint32_t base, uint8_t ch);
bool musb_dma_program_tx(uint32_t base, uint8_t ch, uint8_t ep, uint8_t *buf, uint32_t len, uint8_t multi);
bool musb_dma_program_rx(uint32_t base, uint8_t ch, uint8_t ep, uint8_t *buf, uint32_t len);
uint32_t musb_dma_read_intr(uint32_t base);
uint32_t musb_dma_irq_process(uint32_t base, uint32_t dmaintr, uint32_t *err_map);
void musb_dma_deinit(uint32_t base);

void musb_dma_clean_dcache(uint8_t busid, const void *addr, uint32_t len);
void musb_dma_invalidate_dcache(uint8_t busid, void *addr, uint32_t len);

#endif /* CONFIG_USB_MUSB_DMA */

#endif /* __USB_MUSB_DMA_H__ */
