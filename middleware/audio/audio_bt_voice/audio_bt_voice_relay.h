/*
 * SPDX-FileCopyrightText: 2020-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __relay_H__
#define __relay_H__

#include <stdint.h>
#include <stddef.h>
#include "ipc/ringbuffer.h"
#include "ipc/dataqueue.h"
#include "audio_msbc_plc.h"
#include "audio_cvsd.h"

#ifdef __cplusplus
extern "C" {
#endif

void bt_voice_relay_open(uint16_t sco_hdl, uint32_t samplerate);
void bt_voice_relay_close(uint16_t sco_hdl);
void bt_voice_relay_downlink_process(uint8_t is_ready);
uint8_t bt_voice_relay_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* __relay_H__ */