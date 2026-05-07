/*
 * SPDX-FileCopyrightText: 2020-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AUDIO_3A_ANYKA_H__
#define __AUDIO_3A_ANYKA_H__

#include <string.h>
#include <stdlib.h>

#include "sdfilter.h"
#include "sdParamFactory.h"


#ifdef __cplusplus
extern "C" {
#endif

#define ANYKA_FRAME_SIZE        320 /* must same as CODEC_DATA_UNIT_LEN in audio_server.c */
#define ANYKA_CACHED_FRMAES     4

/*
    The final algorithm determines how many samples the MIC signal lags behind the reference signal,
    then adjusts the g_mic_delay_ref to ensure the measured value matches this value
*/
#define DELAY_SAMPLE    10


typedef struct _vad_instance
{
    T_U32 vadType;
    char *name;
    int32_t ts_active_start; // -1: not start, other: start ts
} t_vad_instance;

typedef struct
{
    char        *const_far;
    char        *const_near;
    T_AUDIO_FILTER_TS   ts_far;
    T_AUDIO_FILTER_TS   ts_dac_stream;
    uint16_t    samplerate;
    uint8_t     is_bt_voice;
    uint8_t     all_mic_channels;
} acpu_audio_3a_open_parameter_t;

typedef struct
{
    uint8_t     *data_in;
    uint8_t     *data_out;
    uint32_t    tick;
} acpu_audio_3a_downlink_parameter_t;

typedef struct
{
    uint8_t     *refframe;
    uint64_t    ts;
    uint8_t     *fifo;
    uint8_t     *result;
    T_AUDIO_FILTER_TS   ts_far;
    T_AUDIO_FILTER_TS   ts_dac_stream;
    uint16_t    samplerate;
} acpu_audio_3a_uplink_parameter_t;

int acpu_audio_3a_open(acpu_audio_3a_open_parameter_t *arg);
int acpu_audio_3a_uplink(acpu_audio_3a_uplink_parameter_t *arg);
int acpu_audio_3a_downlink(acpu_audio_3a_downlink_parameter_t *arg);
int acpu_audio_3a_close();

#ifdef __cplusplus
}
#endif

#endif


