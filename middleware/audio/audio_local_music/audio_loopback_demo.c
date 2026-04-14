/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if 0

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

#define MIC_RECORD_FILE "/music/mic16k.wav"
//#define MIC_RECORD_FILE "/ramfs/mic16k.pcm"  //using ramfs if mounted ramfs
#define RECORD_USING_WEBRTC 1

#ifndef PKG_USING_WEBRTC
    #undef RECORD_USING_WEBRTC
#endif


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
    pa.write_cache_size = 4096;
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
    audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 15);
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
static uint32_t record_len;
static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        //LOG_I("recording data");
        RT_ASSERT(p->data_len == 320);
        record_len += p->data_len;
        //audio_write(g_client, (uint8_t *)p->data, p->data_len);

#if RECORD_USING_WEBRTC
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
        //LOG_I("playing...%d %p %p", fd, pcm, g_client);
        if (fd >= 0 && pcm && g_client)
        {
            int len = read(fd, (void *)pcm, 2048);
            int writted = audio_write(g_client, (uint8_t *)pcm, len);
            if (writted == 0)
            {
                cache_full = 1;
            }
            //LOG_I("writed=%d", writted);
        }
    }
    return 0;
}

typedef struct
{
    uint8_t riff[4];
    uint32_t lenth;
    uint8_t wave[4];
    uint8_t fmt[4];
    uint32_t size1;
    uint16_t fmt_tag;
    uint16_t channel;
    uint32_t sampleRate;
    uint32_t bytePerSec;
    uint16_t blockAlign;
    uint16_t bitPerSample;
    uint8_t data[4];
    uint32_t size2;
} AUD_WAV_HDR_T;

static void fill_wav_header(int fd, uint32_t pcm_len)
{
    AUD_WAV_HDR_T hdr;
    hdr.riff[0] = 'R';
    hdr.riff[1] = 'I';
    hdr.riff[2] = 'F';
    hdr.riff[3] = 'F';
    hdr.lenth = pcm_len + 36;
    hdr.wave[0] = 'W';
    hdr.wave[1] = 'A';
    hdr.wave[2] = 'V';
    hdr.wave[3] = 'E';
    hdr.fmt[0] = 'f';
    hdr.fmt[1] = 'm';
    hdr.fmt[2] = 't';
    hdr.fmt[3] = ' ';
    hdr.size1 = 16;
    hdr.fmt_tag = 1;
    hdr.channel = 1;
    hdr.sampleRate = 16000;
    hdr.blockAlign = 2;
    hdr.bitPerSample = 16;
    hdr.bytePerSec = hdr.sampleRate * hdr.channel * hdr.bitPerSample / 8;
    hdr.data[0] = 'd';
    hdr.data[1] = 'a';
    hdr.data[2] = 't';
    hdr.data[3] = 'a';
    hdr.size2 = pcm_len;
    lseek(fd, 0, SEEK_SET);
    write(fd, &hdr, sizeof(hdr));
    lseek(fd, 0, SEEK_END);
}

/*
    mic2file <1>
example:
1. record to file and playing, then delete it
    mic2file
1. record to file and playing, then reserved it for debug
    can using mp3 command to play it again, mp3 command is in audio_mp3ctrl.c
    mic2file 1
*/
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
    record_len = 0;
    pcm = NULL;
    cache_full = 0;
    pcm = malloc(4096);
    RT_ASSERT(pcm);
    fd = open(MIC_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(fd >= 0);
    fill_wav_header(fd, 0);
#if RECORD_USING_WEBRTC
    webrtc_open();
#endif
    g_client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_TXRX, &pa, audio_callback_record, (void *)fd);
    RT_ASSERT(g_client);

    while (record_seconds < 5)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(g_client);
    fill_wav_header(fd, record_len);
    close(fd);
#if RECORD_USING_WEBRTC
    webrtc_close();
#endif


    //play now
    LOG_I("mic2file play now.");
    pa.write_cache_size = 4096;
    fd = open(MIC_RECORD_FILE, O_RDONLY | O_BINARY);
    RT_ASSERT(fd >= 0);
    lseek(fd, sizeof(AUD_WAV_HDR_T), SEEK_SET);

    audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 15);
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
    if (argc == 1)
    {
        unlink(MIC_RECORD_FILE);
    }
    free(pcm);
    LOG_I("mic2file play end.");
}

MSH_CMD_EXPORT(mic2file, mic2file test);

#endif

