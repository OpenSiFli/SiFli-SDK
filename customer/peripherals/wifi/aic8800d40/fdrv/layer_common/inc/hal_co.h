/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _HAL_CO_H_
#define _HAL_CO_H_

#include "lmac_msg.h"
#ifdef CONFIG_SDIO_SUPPORT
#include "sdio_def.h"
#endif /* CONFIG_SDIO_SUPPORT */


#ifdef CONFIG_SDIO_SUPPORT
typedef struct sdio_buf_list_s hal_buf_list_s;
typedef struct sdio_buf_node_s hal_buf_node_s;
#endif /* CONFIG_SDIO_SUPPORT */

extern uint32_t hal_rx_buf_count;

int aicwf_hal_send_check(void);
int aicwf_hal_send(u8 *pkt_data, u32 pkt_len, bool last);
int aicwf_hal_msg_push(uint8_t *buffer, struct lmac_msg *msg, uint16_t len);
bool aicwf_hal_another_ptk(uint8_t *data, uint32_t len);
//__INLINE
uint8_t *aicwf_hal_get_node_pkt_data(void *node);
//__INLINE
uint32_t aicwf_hal_get_node_pkt_len(void *node);
//__INLINE
int aicwf_hal_node_pkt_is_aggr(void);
void aicwf_hal_buf_free(void *node);

#ifdef CONFIG_SDIO_BUS_PWRCTRL
int aicwf_hal_bus_pwr_stctl(uint target);
//__INLINE
void aicwf_hal_rxcnt_decrease(void);
//__INLINE
void aicwf_hal_txpktcnt_increase(void);
//__INLINE
void aicfw_hal_txpktcnt_decrease(void);
//__INLINE
void aicfw_hal_txpktcnt_decrease_cnt(uint32_t cnt);
#endif /* CONFIG_SDIO_BUS_PWRCTRL */

#ifdef CONFIG_SDIO_ADMA
void aicwf_hal_adma_desc(void *desc, void (*func)(void *), uint8_t needfree);
#endif /* CONFIG_SDIO_ADMA */

#endif /* _HAL_CO_H_ */