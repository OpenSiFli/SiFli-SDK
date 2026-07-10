/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_URC_GATT_H
#define _BT_RT_DEVICE_URC_GATT_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bts2_bt.h"


void urc_func_bt_gatt_reg_res(U32 sdp_hdl, U8 res);
void urc_func_bt_gatt_unreg_res(U32 sdp_hdl, U8 res);
void urc_func_bt_gatt_mtu_res(U8 res);
int bt_sifli_notify_gatt_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len);

#endif /* _BT_RT_DEVICE_URC_GATT_H */

