/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _SDIO_LAYER_H_
#define _SDIO_LAYER_H_

#include "rwnx_defs.h"
#include "sdio_def.h"

#define ALIGN4_ADJ_LEN(x)   ((4-(x&3))&3)

/* sdio buffer define */
#define SDIO_RX_BUF_COUNT       40//20
#define SDIO_RX_BUF_SIZE        2048

void sdio_rx_task(void *argv);
int sdio_host_init(uint16_t chipid);
void sdio_host_predeinit(void);
int sdio_host_deinit(struct rwnx_hw *rwnx_hw);
void aicwf_sdio_reg_init(struct rwnx_hw *rwnx_hw, struct aic_sdio_dev *sdiodev);

void sdio_buf_init(void);
void sdio_buf_free(struct sdio_buf_node_s *node);
void aicwf_sdio_tx_init(void);
#ifdef CONFIG_SDIO_ADMA
void aicwf_sdio_adma_desc(void *desc,sdio_adma_free func,uint8_t needfree);
#endif /* CONFIG_SDIO_ADMA */

#ifdef CONFIG_SDIO_BUS_PWRCTRL
int aicwf_sdio_bus_pwrctrl_preinit(void);
void aicwf_sdio_bus_pwrctl_timer_task_init(void);
void aicwf_sdio_bus_pwrctl_timer_task_deinit(void);
int aicwf_sdio_bus_pwr_stctl(uint target);
void aicwf_sdio_rxcnt_increase(void);
void aicwf_sdio_rxcnt_decrease(void);
void aicwf_sdio_txpktcnt_increase(void);
void aicfw_sdio_txpktcnt_decrease(void);
void aicfw_sdio_txpktcnt_decrease_cnt(uint32_t cnt);
void aicwf_sdio_cmdstatus_set(void);
void aicwf_sdio_cmdstatus_clr(void);
#endif /* CONFIG_SDIO_BUS_PWRCTRL */


int aicwf_sdio_send_check(void);
int aicwf_sdio_flow_ctrl(void);
int aicwf_sdio_tx_msg(u8 *buf, uint count, u8 *out);
int aicwf_sdio_send(u8 *pkt_data, u32 pkt_len, bool last);
uint8_t crc8_ponl_107(uint8_t *p_buffer, uint16_t cal_size);



#endif /* _SDIO_LAYER_H_ */