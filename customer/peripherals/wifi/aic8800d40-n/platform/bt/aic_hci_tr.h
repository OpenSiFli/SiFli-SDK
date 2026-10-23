/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Common HCI transport: one complete H4 frame in and out.
 * UART keeps the type byte on the wire. SDIO mailbox payload is H4.
 * USB Bluetooth class has no H4 on the bus; aic_bt_usb_* adds/strips it.
 */

#ifndef AIC_HCI_TR_H
#define AIC_HCI_TR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*aic_hci_tr_rx_cb_t)(uint8_t *h4, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* AIC_HCI_TR_H */
