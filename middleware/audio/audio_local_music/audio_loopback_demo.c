/**
  ******************************************************************************
  * @file   audio_loopback_demo.c
  * @author Sifli software development team
  * @brief SIFLI audio loopback demo.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include "os_adaptor.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "audio_server.h"

#define DBG_TAG           "audio"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

#define MIC_RECORD_FILE "/mic16k.pcm"
//#define MIC_RECORD_FILE "/ramfs/mic16k.pcm"  //using ramfs if mounted ramfs
#define RECORD_USING_WEBRTC 1

#ifndef PKG_USING_WEBRTC
    #undef RECORD_USING_WEBRTC
#endif

#if !defined(SOLUTION_WATCH) && !defined(SOLUTION) && defined(RT_USING_DFS)


#if RECORD_USING_WEBRTC
#include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"
#include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"
static NsxHandle               *pNS_inst;
static void                    *agcInst;
static uint8_t *frame0;
static uint8_t *frame1;
static uint8_t *in;
static uint8_t *out;
static void app_recorder_ans_proc(NsxHandle *h, int16_t spframe[160], int16_t outframe[160])
{
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
    if (h)
    {
        WebRtcNsx_Process(h, (const int16_t *const *)spframe_p, 1, outframe_p);
    }
}
static void app_recorder_agc_proc(void *h, int16_t spframe[160], int16_t outframe[160])
{
    int32_t micLevelIn = 0;
    int32_t micLevelOut = 0;
    uint8_t saturationWarning;
    uint16_t u16_frame_len = 160;
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
    if (h && 0 != WebRtcAgc_Process(h, (const int16_t *const *)spframe_p, 1, u16_frame_len, (int16_t *const *)outframe_p, micLevelIn, &micLevelOut, 0, &saturationWarning))
    {
        LOG_W("WebRtcAgc_Process error !\n");
    }
}
static void webrtc_process_frame(const uint8_t *p, uint32_t data_len)
{
    app_recorder_ans_proc(pNS_inst, (int16_t *)p, (int16_t *)frame0);
    app_recorder_agc_proc(agcInst, (int16_t *)frame0, (int16_t *)frame1);
}

static void webrtc_open()
{
    pNS_inst = WebRtcNsx_Create();
    RT_ASSERT(pNS_inst);
    if (0 != WebRtcNsx_Init(pNS_inst, 16000))
    {
        RT_ASSERT(0);
    }
    else if (0 != WebRtcNsx_set_policy(pNS_inst, 2))
    {
        RT_ASSERT(0);
    }
    WebRtcAgcConfig agcConfig;
    agcConfig.compressionGaindB = 19;
    agcConfig.limiterEnable = 1;
    agcConfig.targetLevelDbfs = 3;
    agcConfig.thrhold = 14;
    agcInst = WebRtcAgc_Create();
    RT_ASSERT(agcInst);
    if (0 != WebRtcAgc_Init(agcInst, 0, 255, 3, 16000)) // 3 --> kAgcModeFixedDigital
    {
        RT_ASSERT(0);
    }
    if (0 != WebRtcAgc_set_config(agcInst, agcConfig))
    {
        RT_ASSERT(0);
    }
    frame0 = malloc(320);
    RT_ASSERT(frame0);
    frame1 = malloc(320);
    RT_ASSERT(frame1);
}

static void webrtc_close()
{
    if (pNS_inst)
        WebRtcNsx_Free(pNS_inst);
    if (agcInst)
        WebRtcAgc_Free(agcInst);

    if (frame0)
        free(frame0);

    if (frame1)
        free(frame1);

    frame0   = NULL;
    frame1   = NULL;
    pNS_inst = NULL;
    agcInst  = NULL;
}

#endif

static int mic2speaker_callback(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    audio_client_t client = *((audio_client_t *)callback_userdata);
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
#if RECORD_USING_WEBRTC
        RT_ASSERT(p->data_len == 320);
        webrtc_process_frame(p->data, p->data_len);
        audio_write(client, frame1, 320);
#else
        auido_gain_pcm((int16_t *)p->data, p->data_len, 4); //pcm data left shift 4 bits
        audio_write(client, (uint8_t *)p->data, p->data_len);
#endif
    }
    return 0;
}
static void mic2speaker(uint8_t argc, char **argv)
{
    uint32_t record_seconds = 0;
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.read_cache_size = 0;
    pa.write_cache_size = 2048;
#if RECORD_USING_WEBRTC
    webrtc_open();
#endif

    /*
      client must set to null before audio_open(),
      mic2speaker_callback() may be called before audio_open() return,
      and in mic2speaker_callback() will call audio_write(client).
      audio_write(client) can using null client
     */

    audio_client_t client = NULL;

    client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TXRX, &pa, mic2speaker_callback, &client);
    RT_ASSERT(client);

    while (record_seconds < 10)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(client);
#if RECORD_USING_WEBRTC
    webrtc_close();
#endif

}

MSH_CMD_EXPORT(mic2speaker, mic2speaker test);

static uint16_t *pcm;
static audio_client_t g_client;
static int cache_full;
static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
#if RECORD_USING_WEBRTC
        RT_ASSERT(p->data_len == 320);
        webrtc_process_frame(p->data, p->data_len);
        write(fd, frame1, 320);
#else
        auido_gain_pcm((int16_t *)p->data, p->data_len, 4); //pcm data left shift 4 bits
        write(fd, p->data, p->data_len);
#endif
    }
    return 0;
}
static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (fd >= 0 && pcm && g_client)
        {
            read(fd, (void *)pcm, 2048);
            int writted = audio_write(g_client, (uint8_t *)pcm, 2048);
            if (writted == 0)
            {
                cache_full = 1;
            }
        }
    }
    return 0;
}
static void mic2file(uint8_t argc, char **argv)
{
    int fd;
    uint32_t record_seconds = 0;
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.read_cache_size = 0;
    pa.write_cache_size = 2048;
    pcm = NULL;
    cache_full = 0;
    pcm = malloc(4096);
    RT_ASSERT(pcm);
    fd = open(MIC_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(fd >= 0);
#if RECORD_USING_WEBRTC
    webrtc_open();
#endif
    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, audio_callback_record, (void *)fd);
    RT_ASSERT(client);

    while (record_seconds < 5)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(client);
    close(fd);
#if RECORD_USING_WEBRTC
    webrtc_close();
#endif


    //play now
    pa.write_cache_size = 4096;
    fd = open(MIC_RECORD_FILE, O_RDONLY | O_BINARY);
    RT_ASSERT(fd >= 0);

    g_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &pa, audio_callback_play, (void *)fd);
    RT_ASSERT(g_client >= 0);
    record_seconds = 0;
    while (record_seconds < 5)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }

    audio_close(g_client);
    close(fd);
    unlink(MIC_RECORD_FILE);
    free(pcm);
}

MSH_CMD_EXPORT(mic2file, mic2file test);

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
