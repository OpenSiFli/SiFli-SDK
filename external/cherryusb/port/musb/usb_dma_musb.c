/*
 * Copyright (c) 2026, SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared DMA engine for the CherryUSB MUSB port.
 *
 * The Mentor/Inventra MUSB controller embeds a dedicated DMA controller with
 * 8 channels (register block at 0x200..0x27C: DMAINTR + DMACTL/ADDR/COUNT per
 * channel). This file programs those channels for bulk TX/RX transfers and
 * reports completion through DMAINTR.
 *
 * Everything here is guarded by CONFIG_USB_MUSB_DMA so non-SiFli MUSB glue
 * platforms (bk/es/sunxi/ti) that share usb_dc_musb.c / usb_hc_musb.c keep
 * their pure PIO path.
 *
 * Channel n is bound to endpoint/pipe n (EP0 is always PIO). The channel
 * ENABLE bit is cleared by the hardware when the transfer completes, which
 * also clears the DMAINTR bit, so the ISR only reads DMAINTR and never needs
 * to write it back.
 */
#include "usbd_core.h"
#include "usbh_core.h"
#include "usb_musb_reg.h"
#include "usb_musb_dma.h"

#ifdef CONFIG_USB_MUSB_DMA

#define HWREG(x) (*((volatile uint32_t *)(x)))

/* Each DMA channel occupies 0x10 bytes starting at DMACTL0. */
#define USB_DMA_REG_SIZE        (USB_O_DMACTL1 - USB_O_DMACTL0)
#define USB_DMA_CNTL(base, ch)  ((base) + USB_O_DMACTL0 + USB_DMA_REG_SIZE * (ch))
#define USB_DMA_ADDR(base, ch)  ((base) + USB_O_DMAADDR0 + USB_DMA_REG_SIZE * (ch))
#define USB_DMA_COUNT(base, ch) ((base) + USB_O_DMACOUNT0 + USB_DMA_REG_SIZE * (ch))
#define USB_DMA_INTR(base)      ((base) + USB_O_DMAINTR)

struct musb_dma_ch {
    uint8_t in_use; /* 1 = channel programmed, awaiting DMA completion */
    uint8_t ep;     /* endpoint / host pipe index this channel serves */
    uint8_t dir;    /* 1 = TX (device IN / host OUT), 0 = RX (device OUT / host IN) */
    uint8_t multi;  /* 1 = MODE1 multi-packet, 0 = single-packet */
    uint32_t len;   /* programmed byte count (RX: packet rxcount) */
};

static struct musb_dma_ch s_dma_ch[CONFIG_USB_MUSB_DMA_CH_NUM];

bool musb_dma_can_use(const uint8_t *buf, uint32_t len)
{
    if (!buf || (len == 0)) {
        return false;
    }
    /* DMAADDR low 2 bits must be zero. */
    if (((uint32_t)buf & 0x03)) {
        return false;
    }
    if (len < CONFIG_USB_MUSB_DMA_MIN_LEN) {
        return false;
    }
    return true;
}

bool musb_dma_ch_is_active(uint8_t ch)
{
    if (ch >= CONFIG_USB_MUSB_DMA_CH_NUM) {
        return false;
    }
    return s_dma_ch[ch].in_use != 0;
}

void musb_dma_ch_free(uint32_t base, uint8_t ch)
{
    if (ch >= CONFIG_USB_MUSB_DMA_CH_NUM) {
        return;
    }
    /* Disable the channel. On a completed transfer the hardware already
     * cleared ENABLE (and the DMAINTR bit); this write also covers aborts. */
    HWREG(USB_DMA_CNTL(base, ch)) = 0;
    s_dma_ch[ch].in_use = 0;
    s_dma_ch[ch].ep = 0;
    s_dma_ch[ch].dir = 0;
    s_dma_ch[ch].multi = 0;
    s_dma_ch[ch].len = 0;
}

bool musb_dma_program_tx(uint32_t base, uint8_t ch, uint8_t ep, uint8_t *buf, uint32_t len, uint8_t multi)
{
    uint32_t cntl;

    if (ch >= CONFIG_USB_MUSB_DMA_CH_NUM || s_dma_ch[ch].in_use) {
        return false;
    }

    HWREG(USB_DMA_ADDR(base, ch)) = (uint32_t)buf;
    HWREG(USB_DMA_COUNT(base, ch)) = len;

    cntl = USB_DMACTL0_DIR |                          /* transmit */
           ((ep & 0x0f) << USB_DMACTL0_EP_S) |
           USB_DMACTL0_IE |
           USB_DMACTL0_BRSTM_INC16;
    if (multi) {
        cntl |= USB_DMACTL0_MODE;
    }
    /* ENABLE last: hardware starts on this write. */
    HWREG(USB_DMA_CNTL(base, ch)) = cntl | USB_DMACTL0_ENABLE;

    s_dma_ch[ch].in_use = 1;
    s_dma_ch[ch].ep = ep;
    s_dma_ch[ch].dir = 1;
    s_dma_ch[ch].multi = multi;
    s_dma_ch[ch].len = len;
    return true;
}

bool musb_dma_program_rx(uint32_t base, uint8_t ch, uint8_t ep, uint8_t *buf, uint32_t len)
{
    uint32_t cntl;

    if (ch >= CONFIG_USB_MUSB_DMA_CH_NUM || s_dma_ch[ch].in_use) {
        return false;
    }

    HWREG(USB_DMA_ADDR(base, ch)) = (uint32_t)buf;
    HWREG(USB_DMA_COUNT(base, ch)) = len;

    /* DIR = 0 selects receive. Single-packet mode. */
    cntl = ((ep & 0x0f) << USB_DMACTL0_EP_S) |
           USB_DMACTL0_IE |
           USB_DMACTL0_BRSTM_INC16;
    HWREG(USB_DMA_CNTL(base, ch)) = cntl | USB_DMACTL0_ENABLE;

    s_dma_ch[ch].in_use = 1;
    s_dma_ch[ch].ep = ep;
    s_dma_ch[ch].dir = 0;
    s_dma_ch[ch].multi = 0;
    s_dma_ch[ch].len = len;
    return true;
}

uint32_t musb_dma_read_intr(uint32_t base)
{
    return HWREG(USB_DMA_INTR(base));
}

/* Returns a bitmap (bits 1..7) of channels whose DMA transfer completed
 * normally. Channels that report a bus error are reported through *err_map
 * instead (their ENABLE is left for the caller to clean up). Stale interrupts
 * on freed channels are ignored. */
uint32_t musb_dma_irq_process(uint32_t base, uint32_t dmaintr, uint32_t *err_map)
{
    uint32_t done = 0;
    uint32_t ch;

    *err_map = 0;
    dmaintr &= USB_DMA_CH_MASK;

    for (ch = 1; ch < CONFIG_USB_MUSB_DMA_CH_NUM && dmaintr; ch++) {
        if (!(dmaintr & (1u << ch))) {
            continue;
        }
        if (s_dma_ch[ch].in_use) {
            if (HWREG(USB_DMA_CNTL(base, ch)) & USB_DMACTL0_ERR) {
                *err_map |= (1u << ch);
            } else {
                done |= (1u << ch);
            }
        }
        dmaintr &= ~(1u << ch);
    }
    return done;
}

void musb_dma_deinit(uint32_t base)
{
    uint32_t ch;

    for (ch = 0; ch < CONFIG_USB_MUSB_DMA_CH_NUM; ch++) {
        HWREG(USB_DMA_CNTL(base, ch)) = 0;
        s_dma_ch[ch].in_use = 0;
        s_dma_ch[ch].ep = 0;
        s_dma_ch[ch].dir = 0;
        s_dma_ch[ch].multi = 0;
        s_dma_ch[ch].len = 0;
    }
}

__WEAK void musb_dma_clean_dcache(uint8_t busid, const void *addr, uint32_t len)
{
    (void)busid;
    (void)addr;
    (void)len;
}

__WEAK void musb_dma_invalidate_dcache(uint8_t busid, void *addr, uint32_t len)
{
    (void)busid;
    (void)addr;
    (void)len;
}

#endif /* CONFIG_USB_MUSB_DMA */
