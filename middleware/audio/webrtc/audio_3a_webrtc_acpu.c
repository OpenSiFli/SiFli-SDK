/*
 * SPDX-FileCopyrightText: 2020-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include "acpu_ctrl.h"
#include "audio_3a_webrtc.h"

void acpu_printf(const char *fmt, ...);

void *acpu_call_hcpu_malloc(uint32_t size);
void acpu_call_hcpu_free(void *p);

#define g_cngMode   AecmFalse
#define g_echoMode  kSpeakerphone
#define minLevel    0
#define maxLevel    255
#define ulagcMode   kAgcModeFixedDigital
#define dlagcMode   kAgcModeFixedDigital
#define aec_delay   10

#define g_ul_agc_compression_gain_db        18
#define g_ul_agc_target_level_dbfs          3
#define g_ul_agc_thrhold                    14
#define g_dl_agc_target_level_dbfs          6
#define g_dl_agc_compression_gain_db        10
#define g_dl_agc_thrhold                    14


int acpu_webrtc_open(audio_3a_t *thiz, uint32_t samplerate)
{
    uint8_t nMode = ANS_HIGH_LEVEL;
    AecmConfig config;
    WebRtcAgcConfig agcConfig;
    int ret = -1;
    do
    {
        thiz->pNS_inst = WebRtcNsx_Create();
        if (!thiz->pNS_inst)
        {
            break;
        }

        WebRtcNsx_Init(thiz->pNS_inst, samplerate);
        WebRtcNsx_set_policy(thiz->pNS_inst, nMode);

        thiz->aecmInst = WebRtcAecm_Create();
        if (!thiz->aecmInst)
        {
            break;
        }

        config.cngMode = g_cngMode;
        config.echoMode = g_echoMode;

        WebRtcAecm_Init(thiz->aecmInst, samplerate);
        WebRtcAecm_set_config(thiz->aecmInst, config);

        agcConfig.compressionGaindB = g_ul_agc_compression_gain_db;
        agcConfig.limiterEnable = 1;
        agcConfig.targetLevelDbfs = g_ul_agc_target_level_dbfs;
        agcConfig.thrhold = g_ul_agc_thrhold;

        thiz->agcInst = WebRtcAgc_Create();
        if (!thiz->agcInst)
        {
            break;
        }

        WebRtcAgc_Init(thiz->agcInst, minLevel, maxLevel, ulagcMode, samplerate);
        WebRtcAgc_set_config(thiz->agcInst, agcConfig);

        agcConfig.compressionGaindB = g_dl_agc_compression_gain_db;
        agcConfig.limiterEnable = 1;
        agcConfig.targetLevelDbfs = g_dl_agc_target_level_dbfs;
        agcConfig.thrhold = g_dl_agc_thrhold;
        thiz->dwlink_agcInst = WebRtcAgc_Create();
        if (!thiz->dwlink_agcInst)
        {
            break;
        }

        WebRtcAgc_Init(thiz->dwlink_agcInst, minLevel, maxLevel, dlagcMode, samplerate);
        WebRtcAgc_set_config(thiz->dwlink_agcInst, agcConfig);
        ret = 0;
    }
    while (0);

    if (ret < 0)
    {
        acpu_webrtc_close(thiz);
    }

    return ret;
}

int acpu_webrtc_close(audio_3a_t *thiz)
{
    WebRtcNsx_Free(thiz->pNS_inst);
    thiz->pNS_inst = NULL;
    WebRtcAecm_Free(thiz->aecmInst);
    thiz->aecmInst = NULL;
    WebRtcAgc_Free(thiz->agcInst);
    thiz->agcInst = NULL;
    WebRtcAgc_Free(thiz->dwlink_agcInst);
    thiz->dwlink_agcInst = NULL;
    return 0;
}

int acpu_webrtc_downlink(void *agcInst, int16_t spframe[160], int16_t outframe[160], uint32_t samplerate)
{
    int32_t micLevelIn = 0;
    int32_t micLevelOut = 0;
    uint8_t saturationWarning;
    uint16_t u16_frame_len = 80;
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};

    if (16000 <= samplerate)
    {
        u16_frame_len = 160;
    }

    WebRtcAgc_Process(agcInst, (const int16_t *const *)spframe_p, 1, u16_frame_len, (int16_t *const *)outframe_p, micLevelIn, &micLevelOut, 0, &saturationWarning);

    return 0;
}

static void audio_ans_proc(NsxHandle *p_ns_inst, int16_t spframe[160], int16_t outframe[160])
{
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
    WebRtcNsx_Process(p_ns_inst, (const int16_t *const *)spframe_p, 1, outframe_p);
}

static void audio_aec_proc(void *aecmInst, aec_input_para_t *pt_input_para, uint16_t samplerate)
{
    uint16_t u16_frame_len = 80;

    if (16000 <= samplerate)
    {
        u16_frame_len = 160;
    }

    WebRtcAecm_Process(aecmInst, pt_input_para->nearframe, pt_input_para->nearframe_clean, pt_input_para->outframe, u16_frame_len, aec_delay);
}

void audio_agc_proc(void *agcInst, int16_t spframe[160], int16_t outframe[160], uint16_t samplerate)
{
    int32_t micLevelIn = 0;
    int32_t micLevelOut = 0;
    uint8_t saturationWarning;
    uint16_t u16_frame_len = 80;
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};

    if (16000 <= samplerate)
    {
        u16_frame_len = 160;
    }

    WebRtcAgc_Process(agcInst, (const int16_t *const *)spframe_p, 1, u16_frame_len, (int16_t *const *)outframe_p, micLevelIn, &micLevelOut, 0, &saturationWarning);
}

int acpu_webrtc_uplink(acpu_webrtc_uplink_parameter_t *uplink, int16_t *outframe, int16_t *outframe2, uint8_t ans_disable, uint8_t aec_enable, uint8_t uplink_agc_enable)
{
    int16_t  *data_in;
    int16_t  *data_in2;
    int16_t  *data_out;
    int16_t  *temp;

    audio_3a_t *thiz = uplink->thiz;
    data_out = uplink->data_out;

    data_in = data_out;
    data_out = outframe;
    data_in2 = NULL;

    if (!ans_disable)
    {
        audio_ans_proc(thiz->pNS_inst, data_in, data_out);
    }
    else
    {
        memcpy(data_out, data_in, thiz->frame_len);
    }

    data_in2 = data_out;
    if (thiz->is_far_putted && aec_enable)
    {
        aec_input_para_t input_para;
        data_out = outframe2;
        input_para.nearframe = outframe;
        input_para.nearframe_clean = NULL;
        input_para.outframe = data_out;
        if (thiz->is_aecm_mic_putted == 0)
        {
            thiz->is_aecm_mic_putted = 1;
        }

        audio_aec_proc(thiz->aecmInst, &input_para, thiz->samplerate);
    }
    else
    {
        memcpy(data_out, data_in, thiz->frame_len);
    }

    data_in = data_out;
    data_out = outframe;
    memcpy(data_out, data_in, thiz->frame_len);

    temp = data_in;
    data_in = data_out;
    data_out = temp;

    if (uplink_agc_enable && !thiz->disable_uplink_agc)
    {
        audio_agc_proc(thiz->agcInst, data_in, data_out, thiz->samplerate);
    }
    else
    {
        memcpy(data_out, data_in, thiz->frame_len);
    }
    return 0;
}

int acpu_webrtc_farput(void *aecm, uint8_t *farframe, uint32_t data_len)
{
    WebRtcAecm_BufferFarend(aecm, (const int16_t *)farframe,  data_len);
    return 0;
}


