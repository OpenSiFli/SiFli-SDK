/*
 * SPDX-FileCopyrightText: 2020-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AUDIO_3A_WEBRTC_H__
#define __AUDIO_3A_WEBRTC_H__

#include <string.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"
#include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
#include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"
#include "webrtc/modules/audio_processing/dc_correction/dc_correction.h"
#include "webrtc/modules/audio_processing/ramp_in/ramp_in.h"
#include "webrtc/modules/audio_processing/ramp_out/ramp_out.h"

typedef enum  ANS_LEVEL_TAG
{
    ANS_LOW_LEVEL,
    ANS_MODERATE_LEVEL,
    ANS_HIGH_LEVEL,
    ANS_VERY_HIGH_LEVEL,
} ANS_LEVEL;

enum AEC_MODE_TAG
{
    kQuietEarpieceOrHeadset = 0,
    kEarpiece,
    kLoudEarpiece,
    kSpeakerphone,
    kLoudSpeakerphone
};

typedef struct aec_input_para_tag
{
    int16_t *nearframe;
    int16_t *nearframe_clean;
    int16_t *outframe;
} aec_input_para_t;

typedef struct audio_3a_tag
{
    uint8_t      state;
    uint8_t      is_bt_voice;
    uint8_t      disable_uplink_agc;
    volatile uint8_t is_far_putted;
    volatile uint8_t is_aecm_mic_putted;
    uint16_t     frame_len; //byte
    uint16_t     samplerate;
    struct rt_ringbuffer *rbuf_out;
    struct rt_ringbuffer *rbuf_far;
    struct rt_ringbuffer *rbuf_dwlink;
    NsxHandle *pNS_inst;
    void      *aecmInst;
    void      *agcInst;
    void      *dwlink_agcInst;
    void      *dcInst;
    void      *rampInInst;
    void      *rampOutInst;
} audio_3a_t;

typedef struct
{
    audio_3a_t *thiz;
    uint32_t samplerate;
} acpu_webrtc_open_parameter_t;

typedef struct
{
    audio_3a_t *thiz;
    int16_t    *data_out;
    int16_t    *outframe;
    int16_t    *outframe2;
    uint8_t    ans1_disabled;
    uint8_t    aecm_enable;
    uint8_t    uplink_agc_enable;
    uint8_t    is_far_putted;
} acpu_webrtc_uplink_parameter_t;

typedef struct
{
    audio_3a_t  *thiz;
    int16_t     *in;
    int16_t     *out;
    uint32_t    samplerate;
} acpu_webrtc_downlink_parameter_t;

typedef struct
{
    audio_3a_t *thiz;
} acpu_webrtc_close_parameter_t;

typedef struct
{
    void        *aecm;
    uint16_t    *data;
    uint32_t    data_len;
} acpu_webrtc_farput_parameter_t;

int acpu_webrtc_open(audio_3a_t *thiz, uint32_t samplerate);
int acpu_webrtc_uplink(acpu_webrtc_uplink_parameter_t *uplink, int16_t *outframe, int16_t *outframe2, uint8_t ans_disable, uint8_t aec_enable, uint8_t uplink_agc_enable);
int acpu_webrtc_downlink(void *agcInst, int16_t spframe[160], int16_t outframe[160], uint32_t samplerate);
int acpu_webrtc_farput(void *aecm, uint8_t *data, uint32_t data_len);
int acpu_webrtc_close(audio_3a_t *thiz);

#ifdef __cplusplus
}
#endif

#endif


