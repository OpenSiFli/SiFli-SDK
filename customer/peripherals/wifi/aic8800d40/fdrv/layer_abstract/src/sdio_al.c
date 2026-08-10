/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "sdio_co.h"

void aic_sdio_tx_init(void)
{
    aicwf_sdio_tx_init();
}

void aic_sdio_rx_task(void *argv)
{
    sdio_rx_task(argv);
}

void aic_sdio_buf_init(void)
{
    sdio_buf_init();
}

#ifdef CONFIG_SDIO_BUS_PWRCTRL
int aic_sdio_bus_pwrctrl_preinit(void)
{
    return  aicwf_sdio_bus_pwrctrl_preinit();
}
#endif