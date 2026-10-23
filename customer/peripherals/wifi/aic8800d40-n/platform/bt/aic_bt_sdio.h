/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform HCI entry for SDIO mailbox (alongside aic_bt_uart / aic_btusb).
 * Does not open SDIO; send is one complete H4 frame.
 */

#ifndef AIC_BT_SDIO_H
#define AIC_BT_SDIO_H

#include <stdint.h>
#include "aic_hci_tr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef aic_hci_tr_rx_cb_t aic_bt_sdio_rx_cb_t;

int aic_bt_sdio_start(void);
int aic_bt_sdio_send(const uint8_t *h4, uint32_t len);
void aic_bt_sdio_set_rx_cb(aic_bt_sdio_rx_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* AIC_BT_SDIO_H */
