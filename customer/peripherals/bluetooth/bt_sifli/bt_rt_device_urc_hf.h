/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_URC_HF_H
#define _BT_RT_DEVICE_URC_HF_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bts2_bt.h"


#ifdef BT_USING_SIRI
    extern    void urc_func_bt_hf_voice_recog_sifli(uint8_t res);
#endif

extern    int bt_sifli_notify_hfp_hf_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len);

#endif /* _BT_RT_DEVICE_URC_H */

