/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SDIO_API_H_
#define _SDIO_API_H_

#include "sdio_def.h"

void aic_sdio_tx_init(void);
void aic_sdio_buf_init(void);
void aic_sdio_rx_task(void *argv);

/* Platform-configurable SDIO buffer parameters (call before aicwf_sdio_host_init).
 * rx buf count / tx aggr packet count; rx buf size is fixed at compile time.
 * Passing a direct pointer to sdio_dev, compatible with both extern-link and
 * registration-based platform architectures. */
void aic_sdio_rx_buf_config(struct aic_sdio_dev *sdiodev, uint32_t count);
void aic_sdio_tx_aggr_config(struct aic_sdio_dev *sdiodev, uint32_t max_aggr_cnt);

#ifdef CONFIG_SDIO_BUS_PWRCTRL
    int aic_sdio_bus_pwrctrl_preinit(void);
#endif
void aic_sdio_host_predeinit(void);

#endif /* _SDIO_API_H_ */

