/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _SDIO_PORT_H_
#define _SDIO_PORT_H_

//-------------------------------------------------------------------
// Driver Header Files
//-------------------------------------------------------------------
/* AIC RTOS Driver Header Files */
#include "rtos_port.h"
#include "co_list.h"
#include "aic_plat_hal.h"
#include "sdio_func.h"

#if 0
//rtthread sdio
#include <drivers/sdio.h>
#include <drivers/mmcsd_core.h>
#include <drivers/mmcsd_card.h>

//sdio function define
typedef struct rt_sdio_function    sdio_func;
#endif
//-------------------------------------------------------------------
// Driver Macros Define
//-------------------------------------------------------------------
/* sdio func define */
#define FUNC_0                  0
#define FUNC_1                  1
#define FUNC_2                  2
#define SDIOM_MAX_FUNCS         3

/* sdio mode define */
#define CONFIG_RXTASK_INSDIO    1
#define	SDIO_TX_SLIST_MAX       136

// sdio clk phase
#define CONFIG_SDIO_PAHSE       0

//-------------------------------------------------------------------
// Driver Export Variables
//-------------------------------------------------------------------
extern int msgcfm_poll_en;
extern bool_l func_flag_tx;
extern bool_l func_flag_rx;
extern uint8_t sdio_unexcept_test;
extern struct aic_sdio_dev sdio_dev;
extern struct sdio_func *sdio_function[SDIOM_MAX_FUNCS];

#ifdef	CONFIG_SDIO_ADMA
extern unsigned char sdio_tx_buf_fill[512];
extern unsigned char sdio_tx_buf_dummy[SDIO_TX_SLIST_MAX][8];
#endif /* CONFIG_SDIO_ADMA */

//-------------------------------------------------------------------
// Driver SDIO Export Functions Declare
//-------------------------------------------------------------------
typedef void(*aicwf_sdio_irq_handler_t)(struct sdio_func *);

bool sdio_host_enable_isr(bool enable);
void sdio_set_irq_handler(void (*cb)(void));

void sdio_release_func2(void);

void aic_sdio_set_clock(uint32_t clk);
int aic_sdio_enable_func(struct sdio_func *func);
int xhci_sdio_disable_func(struct sdio_func *func);
void sdio_claim_host(struct sdio_func *func);
void sdio_release_host(struct sdio_func *func);
unsigned char sdio_readb(struct sdio_func *func, unsigned int addr, int *err_ret);
void sdio_writeb(struct sdio_func *func, unsigned char b, unsigned int addr, int *err_ret);
int sdio_readsb(struct sdio_func *func, void *dst, unsigned int addr, int count);
int sdio_writesb(struct sdio_func *func, unsigned int addr, void *src, int count);
bool sdio_readb_cmd52(uint32_t addr, uint8_t *data);
bool sdio_writeb_cmd52(uint32_t addr, uint8_t data);
bool sdio_readb_cmd52_func2(uint32_t addr, uint8_t *data);

#ifdef CONFIG_OOB
void aicwf_sdio_oob_enable(void);
#endif /* CONFIG_OOB */
int sdio_interrupt_init(struct aic_sdio_dev *sdiodev);
int sdio_interrupt_deinit(struct aic_sdio_dev *sdiodev);
int aicwf_sdio_func_init(uint16_t chipid, struct aic_sdio_dev *sdiodev);
int aicwf_sdiov3_func_init(uint16_t chipid, struct aic_sdio_dev *sdiodev);
int aicwf_sdio_probe(struct sdio_func *func);
int aicwf_sdio_remove(struct sdio_func *func);
#endif /* _SDIO_H_ */