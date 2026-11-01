/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AIC_UART_H
#define AIC_UART_H
#include "aic_porting_types.h"
#include <stdio.h>
#include <string.h>

#define BT_SOCKET_BUFFER_SIZE 1024
#define BT_DRIVER_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

#define HCI_CMD_PKT 0x01UL
#define HCI_ACL_PKT 0x02UL
#define HCI_SCO_PKT 0x03UL
#define HCI_EVT_PKT 0x04UL

#define HCI_TYPE_LEN 1
#define HCI_LE_EVT_HEADER_LEN 2
#define HCI_LE_ACL_HEADER_LEN 4
#define HCI_SCO_HEADER_LEN 4

typedef enum
{
    HCI_MSGTYPE_ST,
    HCI_LEN_ST,
    HCI_DATA_ST,
    HCI_DONE_ST,
} HCI_RXST_T;

typedef void (*aic_bt_uart_rx_cb_t)(uint8_t *h4, uint32_t len);

int aic_bt_uart_hci_start(void);
int aic_bt_uart_legacy_start(void);
int aic_bt_uart_send(const uint8_t *h4, uint32_t len);
void aic_bt_uart_set_rx_cb(aic_bt_uart_rx_cb_t cb);
int aic_bt_uart_ready(void);
#endif
