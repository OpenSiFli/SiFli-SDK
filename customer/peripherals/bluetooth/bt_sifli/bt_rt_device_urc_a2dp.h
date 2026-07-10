/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_URC_A2DP_H
#define _BT_RT_DEVICE_URC_A2DP_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bts2_bt.h"


extern    void urc_func_bt_avsnk_close_complete_sifli(void);
extern    void urc_func_bt_avsnk_open_complete_sifli(void);
//extern    void urc_func_a2dp_start_ind_sifli(void);
extern    int bt_sifli_notify_a2dp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len);

#endif /* _BT_RT_DEVICE_URC_A2DP_H */
