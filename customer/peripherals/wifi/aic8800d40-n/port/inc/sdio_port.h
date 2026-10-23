/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SDIO port header: ops-dispatch macros.
 */

#ifndef _SDIO_PORT_H_
#define _SDIO_PORT_H_

#include "rtos_port.h"
#include "co_list.h"
#include "aic_sdio_types.h"

/*
 * Runtime-configurable SDIO behavior flags.
 *
 * These were originally compile-time #define macros. However, since host
 * layer code (sdio_co.c) references them via #if directives, changing the
 * macros after the host library (.a) is built had no effect. Converting to
 * runtime variables allows platform code to set them at init time without
 * recompiling the host library.
 *
 * Platform defines and initializes these variables (typically in sdio_port.c).
 * Host code reads them at runtime via the extern declarations below.
 *
 * Default values are provided as _DEFAULT macros for platform convenience.
 */
/*
 * Default policy: RX is handled in a separate task + semaphore by default (=0,
 * reference implementation, see 2a5bf32 "SDIO RX default separate task mode");
 * set to 1 only when the platform SDIO driver handles RX directly in the ISR.
 * The runtime value is decided by the config_* variables below; the compile-time
 * defaults here are only a reference for platform initialization.
 * See docs/aic8800-porting, aic8800-release-adaptation, aic8800-sync SKILL.md.
 */
/* All three defaults are wrapped in #ifndef: a platform may override them at
 * compile time with -DCONFIG_*_DEFAULT=<v> (previously a bare #define, which
 * conflicted with -D and defeated the override -- reported from the goke
 * platform);
 * runtime override goes through config_rxtask_insdio / config_irq_sus /
 * aicwifi_sdio_rxdata_mode below. */
#ifndef CONFIG_RXTASK_INSDIO_DEFAULT
    #define CONFIG_RXTASK_INSDIO_DEFAULT    0   /**< RX in SDIO ISR context (1=in-ISR, 0=separate task, see strategy above) */
#endif
#ifndef CONFIG_RX_POLLING_DEFAULT
    #define CONFIG_RX_POLLING_DEFAULT       0   /**< RX data mode (0=interrupt-driven, 1=polling) */
#endif
#ifndef CONFIG_IRQ_SUS_DEFAULT
    #define CONFIG_IRQ_SUS_DEFAULT          0   /**< Re-enable SDIO IRQ after RX (0=no, 1=yes) */
#endif

extern uint8_t config_rxtask_insdio;    /**< runtime flag: CONFIG_RXTASK_INSDIO_DEFAULT */
extern uint8_t config_irq_sus;          /**< runtime flag: CONFIG_IRQ_SUS_DEFAULT */
extern unsigned char aicwifi_sdio_rxdata_mode; /**< runtime flag: CONFIG_RX_POLLING_DEFAULT (0=interrupt, 1=polling) */

/* Export variables (provided by platform or stubs) */
extern int msgcfm_poll_en;
extern uint8_t sdio_unexcept_test;
extern struct aic_sdio_dev sdio_dev;

#ifdef CONFIG_SDIO_ADMA
    extern unsigned char sdio_tx_buf_fill[512];
    extern unsigned char sdio_tx_buf_dummy[SDIO_TX_SLIST_MAX][8];
#endif

/* ---- SDIO abstraction — platform must implement ALL functions ---- */

/* Host control */
void aicwf_sdio_claim_host(uint32_t func_num);
void aicwf_sdio_release_host(uint32_t func_num);

/* ------------------------------------------------------------------
 * Bulk data transfer (CMD53) -- the ADMA contract between host and platform
 *
 * Non-ADMA (CONFIG_SDIO_ADMA undefined):
 *   src/dst = contiguous buffer, count = byte count; the platform must
 *   CacheClean before write and CacheInvalidate after read
 *   (or do it inside the platform HAL).
 *
 * ADMA (CONFIG_SDIO_ADMA defined):
 *   aicwf_sdio_writesb(func, addr, sg_list, segcnt):
 *     src   = const sdio_sg_list *  (host has filled buf/len of each segment, incl. 8B SDIO header and tail fill segments)
 *     count = segment count (NOT byte count!)
 *     platform duty: hand the sg list to the platform DMA/descriptor engine, and do CacheClean + sync barrier per segment;
 *   aicwf_sdio_readsb(func, addr, dst, count):
 *     single-buffer read under ADMA (platform may implement it as one sg entry), platform must CacheInvalidate;
 *   aicwf_sdio_writesb_msg(): control frame, always contiguous buffer + CacheClean (no ADMA aggregation).
 *
 * The platform must also provide (under CONFIG_SDIO_ADMA, declarations below):
 *   sdio_tx_buf_fill[512] / sdio_tx_buf_dummy[SDIO_TX_SLIST_MAX][8] -- cache aligned, host only fills the content;
 *   PROTECT_TX_ADMA -- ensure the buffer is not recycled/overwritten before TX completes
 *   (reference implementation: poll the DMA done flag after sending before returning; without this protection the host reusing the aggregation buffer clobbers in-flight data).
 *
 * sdio_sg_list type source: port/inc/aic_sdio_types.h (a platform may define
 * AICWF_SDIO_SG_LIST_EXTERNAL to reuse its platform HAL type). For the platform-side
 * ADMA integration checklist and reference implementation see
 * aidoc/adma-platform-porting-reference.md.
 * ------------------------------------------------------------------ */
int  aicwf_sdio_readsb(uint32_t func_num, void *dst, unsigned int addr, int count);
int  aicwf_sdio_writesb(uint32_t func_num, unsigned int addr, void *src, int count);
int  aicwf_sdio_writesb_msg(uint32_t func_num, unsigned int addr, void *src, int count);

/* Register access (CMD52) */
bool aicwf_sdio_readb_cmd52(uint32_t addr, uint8_t *data);
bool aicwf_sdio_writeb_cmd52(uint32_t addr, uint8_t data);
bool aicwf_sdio_readb_cmd52_func2(uint32_t addr, uint8_t *data);
bool aicwf_sdio_writeb_cmd52_func2(uint32_t addr, uint8_t data);

/* write intr_config_reg again after firmware loading to enable the WiFi chip
 * side SDIO interrupt (sdio_port.c) */
int aicwf_sdio_enable_irq_after_fw_boot(void);

/* Initialization */
int  aicwf_sdio_func_init(uint16_t chipid, struct aic_sdio_dev *sdiodev);
int  aicwf_sdiov3_func_init(uint16_t chipid, struct aic_sdio_dev *sdiodev);
void aicwf_sdio_release_func2(void);

/* Interrupt */
bool aicwf_sdio_host_enable_isr(bool enable);
void aicwf_sdio_set_irq_handler(void (*cb)(void));
int  aicwf_sdio_interrupt_init(void);
void aicwf_sdio_isr(uint8_t en);

/* Driver registration */
int  aicwf_sdio_drv_register(void);

#ifdef CONFIG_OOB
    void aicwf_sdio_oob_enable(void);
#endif

#endif /* _SDIO_PORT_H_ */
