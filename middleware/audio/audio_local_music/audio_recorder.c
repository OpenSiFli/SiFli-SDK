/**
  ******************************************************************************
  * @file   audio_recorder.c
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
#include "dfs_file.h"
#include "dfs_posix.h"
#include "audio_server.h"
#ifdef PKG_USING_TINYMP3
    #include "shine_mp3.h"
#endif

#define DBG_TAG           "recorder"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"
#ifndef SOLUTION_WATCH

typedef enum file_format
{
    FILE_FMT_PCM    = 0,
    FILE_FMT_MP3    = 1,
    FILE_FMT_WAV    = 2,
} E_FILE_FMT;

typedef enum
{
    REC_STATE_IDLE      = 0,
    REC_STATE_RUNNING   = 1,
} E_REC_STATE;

typedef struct aud_recorder
{
    E_FILE_FMT fmt;
    char file_name[40];
    rt_uint8_t chs;
    rt_uint32_t rate;
    rt_uint32_t time;
    int fd;
    struct rt_ringbuffer *pcm_rbf;
    rt_uint8_t *temp_buf;
    audio_client_t client;
#ifdef PKG_USING_TINYMP3
    shine_t shine;
    rt_uint32_t per_pass_size;
#endif
    rt_uint8_t state;
} RECORDER;

#define AUDIO_REC_PCM_SIZE          (1024 * 2)
#define AUDIO_REC_TEMP_SIZE         (512)
#define AUDIO_REC_OPEN_EVENT        (1 << 0)
#define AUDIO_REC_CLOSE_EVENT       (1 << 1)
#define AUDIO_REC_PCM_EVENT         (1 << 2)
#define AUDIO_REC_MP3_EVENT         (1 << 3)
#define AUDIO_REC_WAV_EVENT         (1 << 4)

#define AUDIO_REC_ALL_EVENT ( \
                            AUDIO_REC_OPEN_EVENT | \
                            AUDIO_REC_CLOSE_EVENT | \
                            AUDIO_REC_PCM_EVENT | \
                            AUDIO_REC_MP3_EVENT | \
                            AUDIO_REC_WAV_EVENT \
                            )

typedef struct aud_recorder *RECORDER_T;
static struct aud_recorder recorder;
static rt_event_t rec_event;
static audio_client_t g_client;


static int audio_record_pcm(RECORDER_T rec)
{
    rt_uint32_t data_len = 0, wrt_len = 0;

    RT_ASSERT(rec);
    /*write data into pcm file*/
    data_len = rt_ringbuffer_data_len(rec->pcm_rbf);
    while (data_len)
    {
        wrt_len = data_len > AUDIO_REC_TEMP_SIZE ? AUDIO_REC_TEMP_SIZE : data_len;
        rt_ringbuffer_get(rec->pcm_rbf, rec->temp_buf, wrt_len);
        if (write(rec->fd, rec->temp_buf, wrt_len) != wrt_len)
            LOG_E("audio_record_pcm write err\n");
        data_len -= wrt_len;
        //rt_kprintf("audio_record_pcm %d.\n", wrt_len);
    }

    return 0;
}
#ifdef PKG_USING_TINYMP3
static void set_defaults(shine_config_t *config)
{
    shine_set_config_mpeg_defaults(&config->mpeg);
}

static void check_config(shine_config_t *config)
{
    static char *version_names[4] = {"2.5", "reserved", "II", "I"};
    static char *mode_names[4] = {"stereo", "joint-stereo", "dual-channel", "mono"};
    static char *demp_names[4] = {"none", "50/15us", "", "CITT"};

    LOG_I("MPEG-%s layer III, %s  Psychoacoustic Model: Shine\n",
          version_names[shine_check_config(config->wave.samplerate, config->mpeg.bitr)],
          mode_names[config->mpeg.mode]);
    LOG_I("Bitrate: %d kbps  ", config->mpeg.bitr);
    LOG_I("De-emphasis: %s   %s %s\n",
          demp_names[config->mpeg.emph],
          ((config->mpeg.original) ? "Original" : ""),
          ((config->mpeg.copyright) ? "(C)" : ""));
}

static int audio_tinymp3_init(RECORDER_T rec)
{
    shine_config_t config = {0};
    int samples_per_pass = 0;

    RT_ASSERT(rec);
    /*1.set config*/
    set_defaults(&config);
    config.wave.channels = rec->chs;
    config.wave.samplerate = rec->rate;
    config.mpeg.mode = (rec->chs == 1) ? MONO : STEREO;
    config.mpeg.bitr = 64;
    /*2.check config samplerate bitrate*/
    if (shine_check_config(config.wave.samplerate, config.mpeg.bitr) < 0)
    {
        LOG_E("error unsupported samplerate/bitrate configuration\n");
        return -1;
    }
    check_config(&config);
    /*3.shine init*/
    rec->shine = shine_initialise(&config);
    /*4.get sample point num in every frame which shine process one time*/
    samples_per_pass = shine_samples_per_pass(rec->shine) * rec->chs;
    rec->per_pass_size = samples_per_pass * sizeof(rt_uint16_t);
    LOG_I("samples_per_pass:%d %d\n", samples_per_pass, samples_per_pass * sizeof(rt_uint16_t));
    /*malloc temp buf*/
    rec->temp_buf = rt_calloc(1, rec->per_pass_size);
    RT_ASSERT(rec->temp_buf);

    return 0;
}


int audio_record_mp3(RECORDER_T rec, rt_uint8_t last)
{
    rt_uint8_t *out_data = RT_NULL;
    int written = 0;

    RT_ASSERT(rec);
    while (rt_ringbuffer_data_len(rec->pcm_rbf) >= rec->per_pass_size)
    {
        rt_ringbuffer_get(rec->pcm_rbf, rec->temp_buf, rec->per_pass_size);
        out_data = shine_encode_buffer_interleaved(rec->shine, (int16_t *)rec->temp_buf, &written);
        if (write(rec->fd, out_data, written) != written)
            LOG_E("audio_record_mp3 write err1\n");
    }

    if (last)
    {
        if (rt_ringbuffer_data_len(rec->pcm_rbf))
        {
            rt_memset(rec->temp_buf, 0, rec->per_pass_size);
            rt_ringbuffer_get(rec->pcm_rbf, rec->temp_buf, rt_ringbuffer_data_len(rec->pcm_rbf));
            out_data = shine_encode_buffer_interleaved(rec->shine, (int16_t *)rec->temp_buf, &written);
            if (write(rec->fd, out_data, written) != written)
                LOG_E("audio_record_mp3 write err2\n");
        }

        out_data = shine_flush(rec->shine, &written);
        if (write(rec->fd, out_data, written) != written)
            LOG_E("audio_record_mp3 write err3\n");
    }

    return 0;
}
#endif

static int audio_record_wav(RECORDER_T rec)
{
    return 0;
}

static int audio_record_callback(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    RECORDER_T rec = (RECORDER_T)callback_userdata;
    rt_int32_t putlen = 0;

    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;

        /*audio data alg process, gain dc ramp nr*/
        auido_gain_pcm((int16_t *)p->data, p->data_len, 4); //pcm data left shift 4 bits

        /*put data to pcm ringbuf*/
        if (rt_ringbuffer_space_len(rec->pcm_rbf) > p->data_len)
        {
            putlen = rt_ringbuffer_put(rec->pcm_rbf, p->data, p->data_len);
            RT_ASSERT(putlen == p->data_len);
        }
        else
        {
            LOG_E("pcm rbf full, lost data!\n");
        }
        /*send record event by fmt*/
        if (rec->fmt == FILE_FMT_MP3)
        {
            rt_event_send(rec_event, AUDIO_REC_MP3_EVENT);
        }
        else if (rec->fmt == FILE_FMT_WAV)
        {
            rt_event_send(rec_event, AUDIO_REC_WAV_EVENT);
        }
        else
        {
            rt_event_send(rec_event, AUDIO_REC_PCM_EVENT);
        }
        //rt_kprintf("audio_record_callback %d %d\n", putlen, rec->fmt);
    }

    return 0;
}

static int audio_record_open(RECORDER_T rec)
{
    audio_parameter_t pa = {0};

    RT_ASSERT(rec);
    /*create and init ringbuf*/
    rec->pcm_rbf = rt_ringbuffer_create(AUDIO_REC_PCM_SIZE);
    RT_ASSERT(rec->pcm_rbf);
    /*create audio file*/
    if (access("/recorder", 0))
    {
        RT_ASSERT(mkdir("/recorder", 0x777) == 0)
        LOG_I("canot find dir recorder, make it.\n");
    }
    rec->fd = open(rec->file_name, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(rec->fd >= 0);

    /*init mp3 or wav encoder*/
    if (rec->fmt == FILE_FMT_MP3)
    {
#ifdef PKG_USING_TINYMP3
        if (audio_tinymp3_init(rec))
            goto ERR;
#endif
    }
    else if (rec->fmt == FILE_FMT_WAV)
    {
        /*malloc temp buf*/
        rec->temp_buf = rt_calloc(1, AUDIO_REC_TEMP_SIZE);
        RT_ASSERT(rec->temp_buf);
    }
    else
    {
        /*malloc temp buf*/
        rec->temp_buf = rt_calloc(1, AUDIO_REC_TEMP_SIZE);
        RT_ASSERT(rec->temp_buf);
    }

    /*open audio record*/
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = rec->chs;
    pa.write_samplerate = rec->rate;
    pa.write_cache_size = 2048;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = rec->chs;
    pa.read_samplerate = rec->rate;
    pa.read_cache_size = 0;

    rec->client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_RX, &pa, audio_record_callback, (void *)rec);
    RT_ASSERT(rec->client);
    rec->state = REC_STATE_RUNNING;
    /*start a timer*/

    LOG_I("audio_record_open success\n");

    return 0;
ERR:
    close(rec->fd);
    unlink(rec->file_name);
    rt_ringbuffer_destroy(rec->pcm_rbf);
    rt_memset(rec, 0, sizeof(RECORDER));
    rec->state = REC_STATE_IDLE;
    LOG_E("audio_record_open failed\n");

    return -1;
}

static int audio_record_close(RECORDER_T rec)
{
    RT_ASSERT(rec);

    audio_close(rec->client);
    /*process ringbuf remaining data*/
    if (rec->fmt == FILE_FMT_MP3)
    {
#ifdef PKG_USING_TINYMP3
        audio_record_mp3(rec, 1);
        /*close shine*/
        shine_close(rec->shine);
#endif
    }
    else if (rec->fmt == FILE_FMT_WAV)
    {

    }
    else
    {
        audio_record_pcm(rec);
    }
    /*close file*/
    close(rec->fd);
    /*free temp buf, distroy ringbuf*/
    rt_free(rec->temp_buf);
    rt_ringbuffer_destroy(rec->pcm_rbf);
    rt_memset(rec, 0, sizeof(RECORDER));
    LOG_I("audio_record_close.\n");

    return 0;
}

static void audio_record_entry(void *parameter)
{
    rt_uint32_t evt = 0;

    while (1)
    {
        if (rt_event_recv(rec_event, AUDIO_REC_ALL_EVENT, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &evt) == RT_EOK)
        {
            if (evt & AUDIO_REC_OPEN_EVENT)
            {
                audio_record_open(&recorder);
                continue;
            }

            if (evt & AUDIO_REC_CLOSE_EVENT)
            {
                audio_record_close(&recorder);
                continue;
            }

            if (evt & AUDIO_REC_PCM_EVENT)
            {
                audio_record_pcm(&recorder);
            }

            if (evt & AUDIO_REC_MP3_EVENT)
            {
#ifdef PKG_USING_TINYMP3
                audio_record_mp3(&recorder, 0);
#endif
            }

            if (evt & AUDIO_REC_WAV_EVENT)
            {
                audio_record_wav(&recorder);
            }
        }
    }
}

static void record(uint8_t argc, char **argv)
{
    RECORDER_T rec = &recorder;
    char *str = NULL;
    rt_thread_t tid;
    char *fmt = NULL;
    rt_uint8_t record_seconds = 0;

    if (argc < 5)
    {
        LOG_E("[Error]: Invalid argument input!\n \
            [Format]: {cmd} {fmt} {rate} {time} {name}.\n \
            [Example]: record mp3 16000 10s mp3_1.\n");
        return;
    }

    if (rec->state == REC_STATE_RUNNING)
    {
        LOG_E("now is recording!\n");
        return;
    }
    rec->chs = 1; //default mono

    if (!strncmp(argv[1], "mp3", 3))
    {
#ifdef PKG_USING_TINYMP3
        rec->fmt = FILE_FMT_MP3;
        fmt = ".mp3";
#else
        LOG_W("mp3 encoder not open, save as pcm data!\n");
        rec->fmt = FILE_FMT_PCM;
        fmt = ".pcm";
#endif
    }
    else if (!strncmp(argv[1], "wav", 3))
    {
        rec->fmt = FILE_FMT_WAV;
        fmt = ".wav";
        LOG_W("wav encoder not support now!\n");
        return;
    }
    else
    {
        rec->fmt = FILE_FMT_PCM;
        fmt = ".pcm";
    }

    rec->rate = strtoul(argv[2], &str, 10);
    if ((rec->rate != 16000) && (rec->rate != 44100) && (rec->rate != 48000))
    {
        LOG_E("record err sample rate, should be 16000/44100/48000!\n");
        return;
    }

    sscanf(argv[3], "%lds", &rec->time);
    if (!rec->time)
    {
        LOG_E("record err time, at least 1 second!\n");
        return;
    }

    if (strlen(argv[4]) >= 21)
    {
        LOG_E("record err file name, contains a maximum of 20 characters!\n");
        return;
    }
    strcpy(rec->file_name, "/recorder/");
    strcat(rec->file_name, argv[4]);
    strcat(rec->file_name, fmt);

    LOG_I("name:%s, fmt:%s, rate:%d, time:%ds.\n", rec->file_name,
          !rec->fmt ? "pcm" : rec->fmt == 1 ? "mp3" : "wav", rec->rate, rec->time);

    rec_event = rt_event_create("recorder", RT_IPC_FLAG_FIFO);
    RT_ASSERT(rec_event);
    /*create rec process thread*/
    tid = rt_thread_create("recorder", audio_record_entry, NULL, 1536, 16, 10);
    rt_thread_startup(tid);
    /*open audio record*/
    rt_event_send(rec_event, AUDIO_REC_OPEN_EVENT);
    LOG_I("recording start.\n");
    while ((record_seconds < rec->time))
    {
        rt_thread_mdelay(1000);
        record_seconds++;
        LOG_I("recording %ds.\n", record_seconds);
    }
    if (rec->state == REC_STATE_RUNNING)
        rt_event_send(rec_event, AUDIO_REC_CLOSE_EVENT);

    while (rec->state == REC_STATE_RUNNING)
    {
        rt_thread_mdelay(5);
    }
    rt_thread_delete(tid);
    rt_event_delete(rec_event);
    LOG_I("recording end.\n");
}

MSH_CMD_EXPORT(record, audio recorder);
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
