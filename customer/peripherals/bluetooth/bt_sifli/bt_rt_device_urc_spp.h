/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_URC_SPP_H
#define _BT_RT_DEVICE_URC_SPP_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bts2_bt.h"


extern    void urc_func_spp_conn_ind_sifli(uint8_t *addr, U8 srv_chl, U8 *uuid, U8 uuid_len, U16 mfs);
extern    void urc_func_spp_sdp_cfm_sifli(uint8_t *addr, uint8_t reason, U8 *uuid, U8 uuid_len);
extern    void urc_func_spp_data_ind_sifli(U8 *payload, U16 payload_len, uint8_t *addr, U8 srv_chl, U8 *uuid, U8 uuid_len);
extern    void urc_func_spp_data_cfm_sifli(uint8_t *addr, U8 srv_chl, U8 *uuid, U8 uuid_len);
extern    void urc_func_spp_disconn_ind_sifli(uint8_t *addr, U8 srv_chl, U8 *uuid, U8 uuid_len);
extern    int bt_sifli_notify_spp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len);
#endif /* _BT_RT_DEVICE_URC_SPP_H */
