/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AIC_BT_USB_H
#define AIC_BT_USB_H

#include <stdint.h>
#include "aic_hci_tr.h"

#ifdef __cplusplus
extern "C" {
#endif

int aic_bt_usb_start(void);
int aic_bt_usb_send(const uint8_t *h4, uint32_t len);
void aic_bt_usb_set_rx_cb(aic_hci_tr_rx_cb_t cb);
int aic_bt_usb_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* AIC_BT_USB_H */
