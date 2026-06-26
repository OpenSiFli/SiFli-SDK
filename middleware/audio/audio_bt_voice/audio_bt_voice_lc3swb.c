/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifdef SOC_BF0_HCPU

#include <string.h>

#include "audio_bt_voice_lc3swb.h"
#include "audio_mem.h"
#include "lc3.h"
#include "rtthread.h"

#define DBG_TAG           "lc3swb"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

typedef struct
{
    lc3_encoder_t encoder;
    lc3_decoder_t decoder;
    void *encoder_mem;
    void *decoder_mem;
    uint32_t samplerate;
    uint32_t frame_us;
    uint16_t frame_bytes;
    uint16_t pcm_samples;
} audio_bt_voice_lc3swb_env_t;

static audio_bt_voice_lc3swb_env_t g_lc3swb_env;

static int audio_bt_voice_lc3swb_check(uint32_t samplerate, uint32_t frame_us, uint16_t frame_bytes)
{
    if (!LC3_CHECK_SR_HZ(samplerate) || !LC3_CHECK_DT_US(frame_us))
    {
        return -1;
    }

    if ((frame_bytes < LC3_MIN_FRAME_BYTES) || (frame_bytes > LC3_MAX_FRAME_BYTES))
    {
        return -1;
    }

    return 0;
}

int audio_bt_voice_lc3swb_open(uint32_t samplerate, uint32_t frame_us, uint16_t frame_bytes)
{
    unsigned encoder_size;
    unsigned decoder_size;
    int pcm_samples;

    if (g_lc3swb_env.encoder || g_lc3swb_env.decoder)
    {
        audio_bt_voice_lc3swb_close();
    }

    if (audio_bt_voice_lc3swb_check(samplerate, frame_us, frame_bytes) != 0)
    {
        LOG_E("invalid lc3swb param sr=%d, frame_us=%d, bytes=%d", samplerate, frame_us, frame_bytes);
        return -1;
    }

    pcm_samples = lc3_frame_samples(frame_us, samplerate);
    encoder_size = lc3_encoder_size(frame_us, samplerate);
    decoder_size = lc3_decoder_size(frame_us, samplerate);
    if ((pcm_samples <= 0) || (encoder_size == 0) || (decoder_size == 0))
    {
        LOG_E("invalid lc3swb size samples=%d, enc=%d, dec=%d", pcm_samples, encoder_size, decoder_size);
        return -1;
    }

    g_lc3swb_env.encoder_mem = audio_mem_malloc(encoder_size);
    g_lc3swb_env.decoder_mem = audio_mem_malloc(decoder_size);
    if ((g_lc3swb_env.encoder_mem == NULL) || (g_lc3swb_env.decoder_mem == NULL))
    {
        audio_bt_voice_lc3swb_close();
        LOG_E("lc3swb alloc fail enc=%d, dec=%d", encoder_size, decoder_size);
        return -1;
    }

    memset(g_lc3swb_env.encoder_mem, 0, encoder_size);
    memset(g_lc3swb_env.decoder_mem, 0, decoder_size);

    g_lc3swb_env.encoder = lc3_setup_encoder(frame_us, samplerate, 0, g_lc3swb_env.encoder_mem);
    g_lc3swb_env.decoder = lc3_setup_decoder(frame_us, samplerate, 0, g_lc3swb_env.decoder_mem);
    if ((g_lc3swb_env.encoder == NULL) || (g_lc3swb_env.decoder == NULL))
    {
        audio_bt_voice_lc3swb_close();
        LOG_E("lc3swb setup fail");
        return -1;
    }

    g_lc3swb_env.samplerate = samplerate;
    g_lc3swb_env.frame_us = frame_us;
    g_lc3swb_env.frame_bytes = frame_bytes;
    g_lc3swb_env.pcm_samples = (uint16_t)pcm_samples;
    LOG_I("lc3swb open sr=%d, frame_us=%d, bytes=%d, samples=%d", samplerate, frame_us, frame_bytes, pcm_samples);

    return 0;
}

int audio_bt_voice_lc3swb_open_default(void)
{
    return audio_bt_voice_lc3swb_open(AUDIO_BT_VOICE_LC3SWB_SAMPLE_RATE,
                                      AUDIO_BT_VOICE_LC3SWB_FRAME_US,
                                      AUDIO_BT_VOICE_LC3SWB_FRAME_BYTES);
}

void audio_bt_voice_lc3swb_close(void)
{
    if (g_lc3swb_env.encoder_mem)
    {
        audio_mem_free(g_lc3swb_env.encoder_mem);
    }

    if (g_lc3swb_env.decoder_mem)
    {
        audio_mem_free(g_lc3swb_env.decoder_mem);
    }

    memset(&g_lc3swb_env, 0, sizeof(g_lc3swb_env));
}

int audio_bt_voice_lc3swb_encode(const int16_t *pcm, uint8_t *frame, uint16_t frame_size)
{
    if ((g_lc3swb_env.encoder == NULL) || (pcm == NULL) || (frame == NULL) || (frame_size < g_lc3swb_env.frame_bytes))
    {
        return -1;
    }

    return lc3_encode(g_lc3swb_env.encoder, LC3_PCM_FORMAT_S16, pcm, 1, g_lc3swb_env.frame_bytes, frame);
}

int audio_bt_voice_lc3swb_decode(const uint8_t *frame, uint16_t frame_size, int16_t *pcm)
{
    if ((g_lc3swb_env.decoder == NULL) || (frame == NULL) || (pcm == NULL) || (frame_size < g_lc3swb_env.frame_bytes))
    {
        return -1;
    }

    return lc3_decode(g_lc3swb_env.decoder, frame, AUDIO_BT_VOICE_LC3SWB_FRAME_BYTES, LC3_PCM_FORMAT_S16, pcm, 1);
}

int audio_bt_voice_lc3swb_plc(int16_t *pcm)
{
    if ((g_lc3swb_env.decoder == NULL) || (pcm == NULL))
    {
        return -1;
    }

    return lc3_decode(g_lc3swb_env.decoder, NULL, g_lc3swb_env.frame_bytes, LC3_PCM_FORMAT_S16, pcm, 1);
}

uint16_t audio_bt_voice_lc3swb_get_pcm_samples(void)
{
    return g_lc3swb_env.pcm_samples;
}

uint16_t audio_bt_voice_lc3swb_get_frame_bytes(void)
{
    return g_lc3swb_env.frame_bytes;
}

#endif