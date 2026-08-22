/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AUDIO_BT_VOICE_LC3SWB_H
#define AUDIO_BT_VOICE_LC3SWB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_BT_VOICE_LC3SWB_SAMPLE_RATE     32000
#define AUDIO_BT_VOICE_LC3SWB_FRAME_US        7500
#define AUDIO_BT_VOICE_LC3SWB_FRAME_BYTES     58

int audio_bt_voice_lc3swb_open(uint32_t samplerate, uint32_t frame_us, uint16_t frame_bytes);
int audio_bt_voice_lc3swb_open_default(void);
void audio_bt_voice_lc3swb_close(void);
int audio_bt_voice_lc3swb_encode(const int16_t *pcm, uint8_t *frame, uint16_t frame_size);
int audio_bt_voice_lc3swb_decode(const uint8_t *frame, uint16_t frame_size, int16_t *pcm);
int audio_bt_voice_lc3swb_plc(int16_t *pcm);
uint16_t audio_bt_voice_lc3swb_get_pcm_samples(void);
uint16_t audio_bt_voice_lc3swb_get_frame_bytes(void);

#ifdef __cplusplus
}
#endif

#endif