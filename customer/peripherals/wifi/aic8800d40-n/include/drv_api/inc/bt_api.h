/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HCI transport facade for AIC8800. Does not expose rwnx_hw or SDIO registers.
 * SDIO mailbox implementation stays in drv_bt / drv_macif.
 */

#ifndef _BT_API_H_
#define _BT_API_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Must match mm_bt_send_req.bt_data[] in lmac_msg.h */
#define AIC_BT_HCI_H4_MAX  1024u

typedef void (*aic_bt_hci_rx_cb_t)(uint8_t *data, uint32_t len);

int aic_bt_hci_send(const uint8_t *h4, uint32_t len);
void aic_bt_hci_set_rx_cb(aic_bt_hci_rx_cb_t cb);

/* 1 = BT controller fw (patch) loaded OK by aicbt_init; 0 otherwise */
int aic_bt_fw_ready(void);

/* 复位 BT 通路就绪状态 (控制器掉电/驱动关闭时调用), 使 aic_bt_fw_ready() 归 0 */
void aic_bt_fw_reset(void);



#ifdef __cplusplus
}
#endif

#endif /* _BT_API_H_ */
