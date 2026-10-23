/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SDIO HCI transport for platform/bt. Calls drv_api only (no rwnx_*, no sdio_port).
 */

#include "aic_bt_sdio.h"
#include "bt_api.h"

int aic_bt_sdio_start(void)
{
    if (aic_bt_fw_ready() != 1)
    {
        return -1;
    }
    return 0;
}

int aic_bt_sdio_send(const uint8_t *h4, uint32_t len)
{
    return aic_bt_hci_send(h4, len);
}

void aic_bt_sdio_set_rx_cb(aic_bt_sdio_rx_cb_t cb)
{
    aic_bt_hci_set_rx_cb(cb);
}

