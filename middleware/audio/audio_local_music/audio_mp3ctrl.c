/**
  ******************************************************************************
  * @file   audio_mp3ctrl.c
  * @author Sifli software development team
  * @brief SIFLI audio play mp3 or wave music.
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
#include "audio_mp3ctrl.h"
#include "sifli_resample.h"
#ifdef SOLUTION_WATCH
    #include "app_mem.h"
#endif

#define PUBLIC_API


#include "mp3dec.h"

#define DBG_TAG           "audio"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

#define MP3_HANDLE_MAGIC        0x33333333

#define CACHE_BUF_SIZE          (3*MAINBUF_SIZE + 100)

#define FADE_OUT_TIME_MS      1000

#define MP3_ONE_STEREO_FRAME_SIZE (MAX_NCHAN * MAX_NGRAN * MAX_NSAMP * 2)
#define MP3_FRAME_CACHE_COUNT (4) // if not a2dp source, 2 is enough
#define MP3_FRAME_CACHE_SIZE  (MP3_ONE_STEREO_FRAME_SIZE * MP3_FRAME_CACHE_COUNT + 10)

//event flag
#define MP3_EVENT_FLAG_PLAY             (1 << 0)
#define MP3_EVENT_FLAG_PAUSE            (1 << 1)
#define MP3_EVENT_FLAG_SEEK             (1 << 2)
#define MP3_EVENT_FLAG_CLOSE            (1 << 3)
#define MP3_EVENT_FLAG_DECODE           (1 << 4)
#define MP3_EVENT_FLAG_NEXT             (1 << 5)
#define MP3_EVENT_FLAG_RESUME           (1 << 6)

// api event
#define API_EVENT_PLAY             (1 << 0)
#define API_EVENT_PAUSE            (1 << 1)
#define API_EVENT_CLOSE            (1 << 3)
#define API_EVENT_NEXT             (1 << 4)
#define API_EVENT_SEEK             (1 << 5)
#define API_EVENT_RESUME           (1 << 6)


#define MP3_EVENT_ALL (MP3_EVENT_FLAG_PLAY|MP3_EVENT_FLAG_PAUSE| \
                       MP3_EVENT_FLAG_SEEK|MP3_EVENT_FLAG_CLOSE| \
                       MP3_EVENT_FLAG_DECODE|MP3_EVENT_FLAG_NEXT|MP3_EVENT_FLAG_RESUME)

typedef enum
{
    MP3_PLAY    = 0,
    MP3_PAUSE   = 1,
    MP3_CLOSE   = 2,
    MP3_SEEK    = 3,
    MP3_NEXT    = 4,
} mp3_cmd_e;

typedef struct
{
    rt_slist_t          snode;
    mp3_cmd_e           cmd;
    void                *cmd_paramter1;

    /*for switch to next mp3*/
    int                 next_fd;
    uint8_t             *next_buffer;
    uint8_t             next_is_file;
} mp3_cmt_t;

struct mp3ctrl_t
{
    uint32_t        magic;
    sifli_resample_t *resample;
    audio_type_t    type;
    HMP3Decoder     handle;
    rt_thread_t     thread;
    uint32_t        loop_times;
    uint32_t        frame_index;
    rt_event_t      event;
    rt_event_t      api_event;
    rt_slist_t      cmd_slist;
    rt_mutex_t      cmd_slist_mutex;

    audio_server_callback_func callback;
    void                      *userdata;

    uint8_t         *cache_ptr;
    uint8_t         *cache_read_ptr;
    int             cache_bytesLeft;
    mp3_info_t      frameinfo;
    //source
    audio_client_t  client;
    const char      *filename; //filename or mp3 data
    uint32_t        mp3_data_len;
    uint32_t        tag_len;
    int             fd;        //file fd or mp3 data offset
    uint32_t        total_time_in_seconds;
    uint32_t        last_display_seconds;
    uint32_t        bitrate;
    uint8_t         is_file;
    uint8_t         is_suspended;
    uint8_t         is_wave;
    uint8_t         is_file_end;
    //wave
    uint32_t        wave_bytes_per_second;
    uint32_t        wave_samplerate;
    uint8_t         wave_channels;
    uint8_t         is_record;
#if defined(SYS_HEAP_IN_PSRAM)
    uint8_t        *stack_addr;
#endif
};

typedef struct  ID3v1
{
    char header[3];     // TAG
    char title[30];     // title
    char artist[30];    // author
    char album[30];     // album
    char year[4];       //
    char comment[28];   //
    char reserved;      // 0 has genra, !=0 comment[28] is 30 bytes
    char genra;         //
} id3v1_t;

typedef struct ID3v2
{
    char header[3];     // ID3
    char ver;           //
    char revision;      //
    char flag;          //
    char size[4];       // size, not include id3v2_t
} id3v2_t;

#undef audio_mem_malloc
#undef audio_mem_free
#undef audio_mem_calloc

#ifdef SOLUTION_WATCH
    #define audio_mem_malloc    app_malloc
    #define audio_mem_free      app_free
    #define audio_mem_calloc    app_calloc
#else
    #define audio_mem_malloc    rt_malloc
    #define audio_mem_free      rt_free
    #define audio_mem_calloc    rt_calloc
#endif

static uint32_t wav_read_header(mp3ctrl_handle ctrl);
static int get_frame_info(mp3ctrl_handle ctrl, MP3FrameInfo *mp3FrameInfo);

#if defined (SOLUTION_RING_BUILT_IN) || defined (RING_USING_FLASH_B)
    #include "lv_conf.h"
    #include "drv_flash.h"
    #include "mem_map.h"
#endif

static int buf_read(mp3ctrl_handle handle, void *buf, int len)
{
    if ((uint32_t)(handle->fd + len) >= handle->mp3_data_len)
    {
        len = handle->mp3_data_len - (uint32_t)handle->fd;
    }
    if (len == 0)
        return 0;
#if defined (SOLUTION_RING_BUILT_IN) || defined (RING_USING_FLASH_B)
    if (LV_IS_RING_ON_FLASH(handle->filename + handle->fd))
    {
#if defined (RING_USING_FLASH_B)
        rt_flash_read((uint32_t)(handle->filename + handle->fd), buf, len);
#else
        rt_nand_read((uint32_t)(handle->filename + handle->fd), buf, len);
#endif
    }
    else
#endif
    {
        memcpy(buf, handle->filename + handle->fd, len);
    }
    handle->fd = handle->fd + (int)len;
    return len;
}
static int buf_seek(mp3ctrl_handle handle, int offset)
{
    if (offset > (int)handle->mp3_data_len)
        offset = (int)handle->mp3_data_len;
    handle->fd = offset;
    return offset;
}

static void wave_seek(mp3ctrl_handle ctrl, uint32_t offset)
{
#if RT_USING_DFS
    if (ctrl->is_file)
        lseek(ctrl->fd, offset, SEEK_SET);
    else
#endif
        buf_seek(ctrl, offset);
}

static void wave_read(mp3ctrl_handle ctrl, void *buffer, uint32_t size)
{
#if RT_USING_DFS
    if (ctrl->is_file)
        read(ctrl->fd, buffer, size);
    else
#endif
        buf_read(ctrl, buffer, size);
}

static uint32_t audio_parse_mp3_id3v2(mp3ctrl_handle handle)
{
    uint32_t    tag_len;
    id3v2_t     id3;
#if RT_USING_DFS
    if (handle->is_file)
        read(handle->fd, (char *)&id3, 10);
    else
#endif
        buf_read(handle, (uint8_t *)&id3, 10);
    if (strncmp((const char *)&id3.header, "ID3", 3) != 0)
    {
        LOG_I("no ID3");
        tag_len = 0;
        if (!memcmp((const char *)&id3.header, "RIFF", 4))
        {
            handle->is_wave = 1;
            wave_seek(handle, 0);
            tag_len = wav_read_header(handle);
            LOG_I("wav len=%d", tag_len);
        }
    }
    else
    {
        tag_len = (((id3.size[0] & 0x7F) << 21) | ((id3.size[1] & 0x7F) << 14) |
                   ((id3.size[2] & 0x7F) << 7) | (id3.size[3] & 0x7F));

        LOG_I("ID3 len=0x%x", tag_len);
        tag_len += 10;
    }
#if RT_USING_DFS
    if (handle->is_file)
        lseek(handle->fd, tag_len, SEEK_SET);
    else
#endif
        buf_seek(handle, tag_len);
    return tag_len;
}


inline static void mp3_slist_lock(mp3ctrl_handle handle)
{
    rt_mutex_take(handle->cmd_slist_mutex, RT_WAITING_FOREVER);
}

inline static void mp3_slist_unlock(mp3ctrl_handle handle)
{
    rt_mutex_release(handle->cmd_slist_mutex);
}

static int load_file_to_cache(mp3ctrl_handle ctrl)
{
    int readed = 0;
    if (ctrl->cache_bytesLeft < 2 * MAINBUF_SIZE && !ctrl->is_file_end)
    {
        if (ctrl->cache_bytesLeft > 0)
        {
            memcpy(ctrl->cache_ptr, ctrl->cache_read_ptr, ctrl->cache_bytesLeft);
        }
#if RT_USING_DFS
        if (ctrl->is_file)
            readed = read(ctrl->fd, ctrl->cache_ptr + ctrl->cache_bytesLeft, CACHE_BUF_SIZE - ctrl->cache_bytesLeft);
        else
#endif
            readed = buf_read(ctrl, ctrl->cache_ptr + ctrl->cache_bytesLeft, CACHE_BUF_SIZE - ctrl->cache_bytesLeft);

        ctrl->cache_read_ptr = ctrl->cache_ptr;

        /* zero-pad to avoid finding false sync word after last frame (from old data in readBuf) */
        if (readed >= 0 && readed < CACHE_BUF_SIZE - ctrl->cache_bytesLeft)
        {
            memset(ctrl->cache_ptr + ctrl->cache_bytesLeft + readed, 0, CACHE_BUF_SIZE - ctrl->cache_bytesLeft - readed);
        }
        if (readed <= 0)
        {
            LOG_I("mp3 read end");
            ctrl->is_file_end = 1;
        }
        else
        {
            ctrl->cache_bytesLeft += readed;
        }

    }
    return 0;
}

static int find_sync_in_cache(mp3ctrl_handle ctrl)
{
    int offset = 0;
    while (1)
    {
        load_file_to_cache(ctrl);
        offset = MP3FindSyncWord(ctrl->cache_read_ptr, ctrl->cache_bytesLeft);
        if (offset >= 0)
        {
            break;
        }
        return -1;
    }

    ctrl->cache_read_ptr += offset;
    ctrl->cache_bytesLeft -= offset;

    return 0;
}

static mp3ctrl_handle g_handle1 = NULL;
static mp3ctrl_handle g_handle2 = NULL;

static int mp3_audio_server_callback(audio_server_callback_cmt_t cmd, void *userdata, uint32_t reverved)
{
    //ctrl->client may be null, for audio_open is async
    mp3ctrl_handle ctrl  = (mp3ctrl_handle)userdata;
    RT_ASSERT(ctrl);
    RT_ASSERT(ctrl->magic == MP3_HANDLE_MAGIC || ctrl->magic == ~MP3_HANDLE_MAGIC);
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        LOG_D("mp3 empty: ctrl=0x%x client=0x%x", ctrl, ctrl->client);
        rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
        return 0;
    }
    if (cmd == as_callback_cmd_suspended)
    {
        LOG_I("mp3 suspend: ctrl=0x%x client=0x%x", ctrl, ctrl->client);
        ctrl->is_suspended = 1;
    }
    else if (cmd == as_callback_cmd_resumed)
    {
        LOG_I("mp3 resume: ctrl=0x%x client=0x%x", ctrl, ctrl->client);
        ctrl->is_suspended = 0;
        rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
    }
    else if (cmd == as_callback_cmd_play_resume)
    {
        LOG_I("mp3 resume by sink: ctrl=0x%x client=0x%x", ctrl, ctrl->client);
        if (g_handle1)
        {
            LOG_I("resume by sink 1");
            mp3ctrl_resume(g_handle1);
        }
        else if (g_handle2)
        {
            LOG_I("resume by sink 2");
            mp3ctrl_resume(g_handle2);
        }
        // ctrl->is_suspended = 0;
        // rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
    }
    else if (cmd == as_callback_cmd_play_pause)
    {
        LOG_I("mp3 suspend by sink: ctrl=0x%x client=0x%x", ctrl, ctrl->client);
        if (g_handle1)
        {
            LOG_I("pause by sink 1");
            mp3ctrl_pause(g_handle1);
        }
        else if (g_handle2)
        {
            LOG_I("pause by sink 2");
            mp3ctrl_pause(g_handle2);
        }
        // ctrl->is_suspended = 1;
    }

    //callback to user
    if (ctrl->callback)
    {
        LOG_I("mp3 user callback");
        ctrl->callback(cmd, ctrl->userdata, 0);  //mp3ctrl_open()'s callback
    }
    return 0;
}

static void replace(mp3ctrl_handle ctrl)
{
    MP3FrameInfo frameinfo;
    uint32_t file_size;
    mp3_cmt_t  *p_cmd;
#if RT_USING_DFS
    if (ctrl->is_file && ctrl->fd >= 0)
    {
        close(ctrl->fd);
        ctrl->fd = -1;
    }
#endif
    ctrl->cache_bytesLeft = 0;
    ctrl->is_file_end = 0;
    rt_slist_t *first = rt_slist_first(&ctrl->cmd_slist);
    RT_ASSERT(first);
    p_cmd = rt_slist_entry(first, mp3_cmt_t, snode);
    rt_slist_remove(&ctrl->cmd_slist, first);
    ctrl->is_file = p_cmd->next_is_file;
#if RT_USING_DFS
    if (ctrl->is_file)
    {
        ctrl->fd = p_cmd->next_fd;
        file_size = (uint32_t)p_cmd->cmd_paramter1;
        ctrl->mp3_data_len = 0;
    }
    else
#endif
    {
        ctrl->filename = (const char *)p_cmd->next_buffer;
        ctrl->fd = 0; //data offset
        file_size = (uint32_t)p_cmd->cmd_paramter1;
        ctrl->mp3_data_len = file_size;
    }
    ctrl->tag_len = audio_parse_mp3_id3v2(ctrl);
    if (ctrl->tag_len >= file_size)
    {
        ctrl->mp3_data_len = 0;
    }
    else
    {
        ctrl->mp3_data_len = file_size;
    }
    if (ctrl->is_wave == 0 && get_frame_info(ctrl, &frameinfo) == 0)
    {
        ctrl->total_time_in_seconds = (ctrl->mp3_data_len - ctrl->tag_len) * 8 / frameinfo.bitrate;
        ctrl->bitrate = frameinfo.bitrate;
        LOG_I("repalce time=%d channel=%d samprate=%d samps=%d",
              ctrl->total_time_in_seconds, frameinfo.nChans, frameinfo.samprate, frameinfo.outputSamps);
    }
    else if (ctrl->is_wave == 1 && ctrl->wave_bytes_per_second != 0)
    {
        ctrl->total_time_in_seconds = (ctrl->mp3_data_len - ctrl->tag_len) / ctrl->wave_bytes_per_second;
    }
    else
    {
        LOG_I("get frame info error");
    }
#if RT_USING_DFS
    if (ctrl->is_file)
        lseek(ctrl->fd, ctrl->tag_len, SEEK_SET);
    else
#endif
        buf_seek(ctrl, ctrl->tag_len);

    audio_mem_free(p_cmd);
    ctrl->cache_bytesLeft = 0;
}

static inline void mono2stereo(int16_t *mono, uint32_t samples, int16_t *stereo)
{
    for (int i = 0; i < samples; i++)
    {
        *stereo++ = *mono;
        *stereo++ = *mono++;
    }
}
static void callback_playing_progress(mp3ctrl_handle ctrl)
{
    off_t pos;
    if (ctrl->callback)
    {
#if RT_USING_DFS
        if (ctrl->is_file)
        {
            pos = lseek(ctrl->fd, 0, SEEK_CUR);
        }
        else
#endif
        {
            pos = ctrl->fd;
        }
#if 1
        uint32_t senconds = ctrl->frame_index * ctrl->frameinfo.one_channel_sampels / ctrl->frameinfo.samplerate;
#else
        uint32_t senconds = ctrl->total_time_in_seconds * (pos - ctrl->tag_len) / (ctrl->mp3_data_len - ctrl->tag_len);
#endif
        if (ctrl->last_display_seconds != senconds)
        {
            ctrl->callback(as_callback_cmd_user, (void *)senconds, (uint32_t)ctrl->userdata);
            ctrl->last_display_seconds = senconds;
        }
    }
}
static void mp3ctrl_thread_entry_file(void *parameter)
{
#if PKG_USING_LIBHELIX
    MP3FrameInfo mp3FrameInfo;
    uint32_t frame_err = 0;
    uint32_t old_samplerate = -1;
    uint8_t  old_channels = -1;
    uint8_t  is_paused = 0;
    uint8_t  is_closing = 0;
    uint8_t  cache_full_occured = 0;
    mp3ctrl_handle ctrl = (mp3ctrl_handle)parameter;
    HMP3Decoder hMP3Decoder = MP3InitDecoder();
    RT_ASSERT(hMP3Decoder);
    short *outBuf = audio_mem_malloc(sizeof(short) * MAX_NCHAN * MAX_NGRAN * MAX_NSAMP);
    short *outBuf2 = audio_mem_malloc(sizeof(short) * MAX_NCHAN * MAX_NGRAN * MAX_NSAMP);
    RT_ASSERT(outBuf);
    RT_ASSERT(outBuf2);
    rt_tick_t start = 0;
    int nFrames = 0;
    rt_uint32_t evt;
    LOG_I("mp3 run...ctrl=0x%x", ctrl);
    while (1)
    {
        if (is_closing)
        {
            evt = 0;
            //BT may disconnected, no event send to here
            if (RT_EOK != rt_event_recv(ctrl->event, MP3_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 500, NULL))
            {
                break;
            }
        }
        else
        {
            rt_event_recv(ctrl->event, MP3_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        }

        if (evt & MP3_EVENT_FLAG_CLOSE)
        {
            LOG_I("mp3 close end=%d d=%d", ctrl->is_file_end, get_server_current_device());
            if (ctrl->is_file_end
                    || is_paused
                    || ctrl->is_suspended
                    || (get_server_current_device() != AUDIO_DEVICE_SPEAKER))
            {
                break;
            }
            else if (is_closing == 0)
            {
                LOG_I("mp3 fade");
                is_closing = 1;
                start = rt_tick_get_millisecond();
                audio_ioctl(ctrl->client, -1, 0); //fade out
            }
        }
        if (is_closing)
        {
            if (ctrl->is_suspended
                    || !audio_ioctl(ctrl->client, 2, NULL)
                    || (rt_tick_get_millisecond() - start) > FADE_OUT_TIME_MS)
            {
                LOG_I("mp3 fade done");
                rt_thread_mdelay(50);
                break;
            }
        }
        if (evt & MP3_EVENT_FLAG_PAUSE)
        {
            LOG_I("mp3-->pause p=%d s=%d", is_paused, ctrl->is_suspended);
            if (is_paused == 0)
            {
                is_paused = 1; //pause
                if (ctrl->client)
                {
#if 0 //flush cache data
                    uint32_t cache_time_ms = 150;
                    audio_ioctl(ctrl->client, 1, &cache_time_ms);
                    rt_thread_mdelay(cache_time_ms + 20);
#endif
                    audio_close(ctrl->client);
                    ctrl->client = NULL;
                }
            }
            cache_full_occured = 0;
            rt_event_send(ctrl->api_event, API_EVENT_PAUSE);
            continue;
        }
        if (evt & MP3_EVENT_FLAG_RESUME)
        {
            LOG_I("mp3-->play p=%d s=%d", is_paused, ctrl->is_suspended);
            ctrl->is_suspended = 0;
            if (is_paused == 1)
            {
                is_paused = 0; //resume
            }
            rt_event_send(ctrl->api_event, API_EVENT_RESUME);
            rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
            continue;
        }
        if (evt & MP3_EVENT_FLAG_PLAY)
        {
            if (is_paused == 1)
            {
                is_paused = 0; //resume
            }
            ctrl->is_suspended = 0;
            rt_event_send(ctrl->api_event, API_EVENT_PLAY);
        }

        if (evt & MP3_EVENT_FLAG_SEEK)
        {
            uint32_t offset;
            is_paused = 0; //pause
            mp3_slist_lock(ctrl);
            mp3_cmt_t  *p_cmd;
            rt_slist_t *first = rt_slist_first(&ctrl->cmd_slist);
            RT_ASSERT(first);
            p_cmd = rt_slist_entry(first, mp3_cmt_t, snode);
            rt_slist_remove(&ctrl->cmd_slist, first);
            mp3_slist_unlock(ctrl);
            offset = (uint32_t)p_cmd->cmd_paramter1 * (uint64_t)ctrl->bitrate / 8;
#if RT_USING_DFS
            if (ctrl->is_file)
                lseek(ctrl->fd, ctrl->tag_len + offset, SEEK_SET);
            else
#endif
                buf_seek(ctrl, ctrl->tag_len + offset);
            ctrl->cache_bytesLeft = 0;
            ctrl->is_file_end = 0;
            audio_mem_free(p_cmd);
            rt_event_send(ctrl->api_event, API_EVENT_SEEK);
        }

        if (evt & MP3_EVENT_FLAG_NEXT)
        {
            frame_err = 0;
            ctrl->frame_index = 0;
            ctrl->last_display_seconds = -1;
            mp3_slist_lock(ctrl);
            replace(ctrl);
            MP3FreeDecoder(hMP3Decoder);
            hMP3Decoder = MP3InitDecoder();
            RT_ASSERT(hMP3Decoder);
            mp3_slist_unlock(ctrl);
            if (is_paused == 1)
            {
                is_paused = 0; //resume
            }
            rt_event_send(ctrl->api_event, API_EVENT_NEXT);
        }

        if (is_paused == 1 || ctrl->is_suspended)
        {
            LOG_D("mp3 paused=%d suspend=%d", is_paused, ctrl->is_suspended);
            rt_thread_mdelay(20);
            continue;
        }

        if (evt & MP3_EVENT_FLAG_DECODE)
        {
            int ret;
            if (cache_full_occured)
            {
                LOG_D("mp3 try write again");
                MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
                if (audio_device_is_a2dp_sink())
                {
                    uint32_t bytes;
                    if (mp3FrameInfo.samprate != 44100)
                    {
                        if (ctrl->resample)
                        {
                            ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), ctrl->resample->dst_bytes);
                            goto look_write_result;
                        }
                        else
                        {
                            ctrl->resample = sifli_resample_open(2, mp3FrameInfo.samprate, 44100);
                            RT_ASSERT(ctrl->resample);
                        }
                        if (mp3FrameInfo.nChans == 2)
                        {
                            bytes = sifli_resample_process(ctrl->resample, (int16_t *)outBuf, mp3FrameInfo.outputSamps * 2, 0);
                            ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), bytes);
                        }
                        else
                        {
                            mono2stereo((int16_t *)outBuf, mp3FrameInfo.outputSamps, (int16_t *)outBuf2);
                            bytes = sifli_resample_process(ctrl->resample, (int16_t *)outBuf2, mp3FrameInfo.outputSamps * 4, 0);
                            ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), bytes);
                        }
                    }
                    else if (mp3FrameInfo.nChans == 1)
                    {
                        mono2stereo((int16_t *)outBuf, mp3FrameInfo.outputSamps, (int16_t *)outBuf2);
                        ret = audio_write(ctrl->client, (uint8_t *)outBuf2, mp3FrameInfo.outputSamps * 4);
                    }
                    else
                    {
                        ret = audio_write(ctrl->client, (uint8_t *)outBuf, mp3FrameInfo.outputSamps * 2);
                    }
                }
                else
                {
                    ret = audio_write(ctrl->client, (uint8_t *)outBuf, mp3FrameInfo.outputSamps * 2);
                }

look_write_result:
                if (ret == 0)
                {
                    LOG_D("mp3 cache full");
                    continue;
                }
                else if (ret == -1)
                {
                    continue;
                }
                cache_full_occured = 0;
                callback_playing_progress(ctrl);
            }
        }

        if (find_sync_in_cache(ctrl) < 0)
        {
            uint32_t cache_time_ms = 150;
            audio_ioctl(ctrl->client, 1, &cache_time_ms);
            rt_thread_mdelay(cache_time_ms + 20);

            if (ctrl->loop_times > 0)
            {
                ctrl->is_file_end = 0;

                ctrl->loop_times--;
#if RT_USING_DFS
                if (ctrl->is_file)
                    lseek(ctrl->fd, ctrl->tag_len + 0, SEEK_SET);
                else
#endif
                    buf_seek(ctrl, ctrl->tag_len + 0);
                LOG_I("mp3--loo frame=%d", nFrames);

                ctrl->cache_read_ptr = ctrl->cache_ptr;
                cache_full_occured = 0;
                ctrl->cache_bytesLeft = 0;
                ctrl->is_file_end = 0;
                nFrames = 0;
                MP3FreeDecoder(hMP3Decoder);
                hMP3Decoder = MP3InitDecoder();
                RT_ASSERT(hMP3Decoder);
                rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
                continue;
            }

            is_paused = 1;

            LOG_I("mp3--end frame=%d", nFrames);
            nFrames = 0;
            if (ctrl->callback && 0 == is_closing)
                ctrl->callback(as_callback_cmd_play_to_end, ctrl->userdata, 0);
            continue;
        }

        int err = MP3Decode(hMP3Decoder, &ctrl->cache_read_ptr, &ctrl->cache_bytesLeft, outBuf, 0);

        nFrames++;
        ctrl->frame_index++;
        //LOG_I("nFrames=%d", nFrames);
        if (err)
        {
            LOG_I("mp3 err=%d", err);
            frame_err++;
            if (frame_err > 128
                    || (ERR_MP3_OUT_OF_MEMORY == err)
                    || (ERR_MP3_NULL_POINTER == err))
            {
                rt_thread_mdelay(100);
                ctrl->is_file_end = 1;
                ctrl->cache_bytesLeft = 0;
                LOG_I("too many errors, seek to file end");
            }
            rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
        }
        else
        {
            /* no error */
            int ret;
            frame_err = 0;

            MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
            if (mp3FrameInfo.nChans != old_channels || mp3FrameInfo.samprate != old_samplerate)
            {
                if (ctrl->client)
                {
                    audio_close(ctrl->client);
                    ctrl->client = NULL;
                }
                if (ctrl->resample)
                {
                    sifli_resample_close(ctrl->resample);
                    ctrl->resample = NULL;
                }
                LOG_I("resample should open %d", mp3FrameInfo.samprate);
            }
            if (!ctrl->client)
            {
                audio_parameter_t pa = {0};
                pa.write_bits_per_sample = 16;
                pa.write_channnel_num = mp3FrameInfo.nChans;
                pa.write_samplerate = mp3FrameInfo.samprate;
                pa.write_cache_size = MP3_FRAME_CACHE_SIZE;
                old_channels = mp3FrameInfo.nChans;
                old_samplerate = mp3FrameInfo.samprate;
                ctrl->frameinfo.samplerate = mp3FrameInfo.samprate;
                ctrl->frameinfo.channels = mp3FrameInfo.nChans;
                ctrl->frameinfo.one_channel_sampels = mp3FrameInfo.outputSamps / mp3FrameInfo.nChans;
                ctrl->client = audio_open(ctrl->type, AUDIO_TX, &pa, mp3_audio_server_callback, (void *)ctrl);
                RT_ASSERT(ctrl->client);
                LOG_I("mp3 open ctrl=0x%x, client=0x%x, c=%d samrate=%d samples=%d",
                      ctrl, ctrl->client, mp3FrameInfo.nChans, mp3FrameInfo.samprate, mp3FrameInfo.outputSamps);
            }
            LOG_D("nFrames=%d", nFrames);
            if (audio_device_is_a2dp_sink())
            {
                uint32_t bytes;
                if (mp3FrameInfo.samprate != 44100)
                {
                    if (!ctrl->resample)
                    {
                        LOG_I("resample open %d", mp3FrameInfo.samprate);
                        ctrl->resample = sifli_resample_open(2, mp3FrameInfo.samprate, 44100);
                        RT_ASSERT(ctrl->resample);
                    }
                    if (mp3FrameInfo.nChans == 2)
                    {
                        bytes = sifli_resample_process(ctrl->resample, (int16_t *)outBuf, mp3FrameInfo.outputSamps * 2, 0);
                        ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), bytes);
                    }
                    else
                    {
                        mono2stereo((int16_t *)outBuf, mp3FrameInfo.outputSamps, (int16_t *)outBuf2);
                        bytes = sifli_resample_process(ctrl->resample, (int16_t *)outBuf2, mp3FrameInfo.outputSamps * 4, 0);
                        ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), bytes);
                    }
                }
                else if (mp3FrameInfo.nChans == 1)
                {
                    mono2stereo((int16_t *)outBuf, mp3FrameInfo.outputSamps, (int16_t *)outBuf2);
                    ret = audio_write(ctrl->client, (uint8_t *)outBuf2, mp3FrameInfo.outputSamps * 4);
                }
                else
                {
                    ret = audio_write(ctrl->client, (uint8_t *)outBuf, mp3FrameInfo.outputSamps * 2);
                }
            }
            else
            {
                ret = audio_write(ctrl->client, (uint8_t *)outBuf, mp3FrameInfo.outputSamps * 2);
            }

            if (ret == -1)
            {
                LOG_I("mp3 suspend");
                rt_thread_mdelay(20);
            }
            if (ret == 0)
            {
                LOG_D("mp3 cache full");
                cache_full_occured = 1;
                rt_thread_mdelay(20);
            }
            else
            {
                cache_full_occured = 0;
                callback_playing_progress(ctrl);
                rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
            }
        }
    }
    if (ctrl->client)
        audio_close(ctrl->client);
    ctrl->client = NULL;
    LOG_I("mp3 exit..nFrames=%d", nFrames);
    MP3FreeDecoder(hMP3Decoder);
#if RT_USING_DFS
    if (ctrl->is_file)
        close(ctrl->fd);
#endif
    audio_mem_free(ctrl->cache_ptr);
    ctrl->cache_ptr = NULL;
    audio_mem_free(outBuf);
    audio_mem_free(outBuf2);

    mp3_slist_lock(ctrl);
    while (1)
    {
        mp3_cmt_t  *p_cmd;
        rt_slist_t *first = rt_slist_first(&ctrl->cmd_slist);
        if (!first)
            break;
        p_cmd = rt_slist_entry(first, mp3_cmt_t, snode);
        rt_slist_remove(&ctrl->cmd_slist, first);

        audio_mem_free(p_cmd);
    }
    mp3_slist_unlock(ctrl);
    rt_mutex_delete(ctrl->cmd_slist_mutex);
    rt_event_delete(ctrl->event);
    rt_event_send(ctrl->api_event, API_EVENT_CLOSE);
    audio_mem_free(ctrl);

#endif //PKG_USING_LIBHELIX
    LOG_I("mp3 exit done");
}


#define WAV_FRAME_SIZE   1024
static void wave_thread_entry_file(void *parameter)
{
    uint16_t last_frame_len = WAV_FRAME_SIZE;
    uint8_t  is_paused = 0;
    uint8_t  is_closing = 0;
    uint8_t  cache_full_occured = 0;
    uint8_t  old_channels = -1;;
    uint32_t old_samplerate = -1;
    rt_tick_t start = 0;
    mp3ctrl_handle ctrl = (mp3ctrl_handle)parameter;
    short *outBuf = audio_mem_malloc(WAV_FRAME_SIZE);
    RT_ASSERT(outBuf);
    short *outBuf2 = audio_mem_malloc(WAV_FRAME_SIZE * 2);
    RT_ASSERT(outBuf2);

    int nFrames = 0;
    rt_uint32_t evt;
    LOG_I("wav run...ctrl=0x%x", ctrl);
    while (1)
    {
        if (is_closing)
        {
            //BT may disconnected, no event send to here
            evt = 0;
            if (RT_EOK != rt_event_recv(ctrl->event, MP3_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 500, NULL))
            {
                break;
            }
        }
        else
        {
            rt_event_recv(ctrl->event, MP3_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        }
        if (evt & MP3_EVENT_FLAG_CLOSE)
        {
            LOG_I("wav close end=%d d=%d", ctrl->is_file_end, get_server_current_device());
            if (ctrl->is_file_end
                    || is_paused
                    || ctrl->is_suspended
                    || (get_server_current_device() != AUDIO_DEVICE_SPEAKER))
            {
                break;
            }
            else if (is_closing == 0)
            {
                LOG_I("wav fade");
                is_closing = 1;
                start = rt_tick_get_millisecond();
                audio_ioctl(ctrl->client, -1, 0); //fade out
            }
        }
        if (is_closing)
        {
            if (ctrl->is_suspended
                    || !audio_ioctl(ctrl->client, 2, NULL)
                    || (rt_tick_get_millisecond() - start) > FADE_OUT_TIME_MS)
            {
                LOG_I("wav fade done");
                break;
            }
        }
        if (evt & MP3_EVENT_FLAG_PAUSE)
        {
            LOG_I("wav-->pause %d %d", is_paused, ctrl->is_suspended);
            if (is_paused == 0)
            {
                is_paused = 1;
                if (ctrl->client)
                {
                    audio_close(ctrl->client);
                    ctrl->client = NULL;
                }
            }
            cache_full_occured = 0;
            rt_event_send(ctrl->api_event, API_EVENT_PAUSE);
            continue;
        }
        if (evt & MP3_EVENT_FLAG_RESUME)
        {
            ctrl->is_suspended = 0;
            LOG_I("wav-->play p=%d s=%d", is_paused, ctrl->is_suspended);
            if (is_paused == 1)
            {
                is_paused = 0; //resume
            }
            rt_event_send(ctrl->api_event, API_EVENT_RESUME);
            rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
            continue;
        }
        if (evt & MP3_EVENT_FLAG_PLAY)
        {
            ctrl->is_suspended = 0;
            if (is_paused == 1)
            {
                is_paused = 0; //resume
            }
            rt_event_send(ctrl->api_event, API_EVENT_PLAY);
        }

        if (evt & MP3_EVENT_FLAG_SEEK)
        {
            uint32_t offset  = -1;
            is_paused = 0; //pause
            ctrl->last_display_seconds = -1;
            mp3_slist_lock(ctrl);
            mp3_cmt_t  *p_cmd;
            rt_slist_t *first = rt_slist_first(&ctrl->cmd_slist);
            RT_ASSERT(first);
            p_cmd = rt_slist_entry(first, mp3_cmt_t, snode);
            rt_slist_remove(&ctrl->cmd_slist, first);
            mp3_slist_unlock(ctrl);
            offset = (uint32_t)p_cmd->cmd_paramter1 * (uint64_t)ctrl->wave_bytes_per_second;
            if (offset != -1)
            {
#if RT_USING_DFS
                if (ctrl->is_file)
                    lseek(ctrl->fd, ctrl->tag_len + offset, SEEK_SET);
                else
#endif
                    buf_seek(ctrl, ctrl->tag_len + offset);
                ctrl->cache_bytesLeft = 0;
                ctrl->is_file_end = 0;
            }
            audio_mem_free(p_cmd);
            rt_event_send(ctrl->api_event, API_EVENT_SEEK);
        }

        if (evt & MP3_EVENT_FLAG_NEXT)
        {
            ctrl->last_display_seconds = -1;
            ctrl->frame_index = 0;
            mp3_slist_lock(ctrl);
            replace(ctrl);
            mp3_slist_unlock(ctrl);
            if (is_paused == 1)
            {
                is_paused = 0; //resume
            }
            rt_event_send(ctrl->api_event, API_EVENT_NEXT);
        }

        if (is_paused == 1 || ctrl->is_suspended)
        {
            LOG_D("wav paused=%d suspend=%d", is_paused, ctrl->is_suspended);
            rt_thread_mdelay(20);
            continue;
        }

        if (evt & MP3_EVENT_FLAG_DECODE)
        {
            int ret;
            if (cache_full_occured)
            {
                if (audio_device_is_a2dp_sink())
                {
                    uint32_t bytes;
                    if (ctrl->wave_samplerate != 44100)
                    {
                        if (ctrl->resample)
                        {
                            ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), ctrl->resample->dst_bytes);
                            goto check_write_result;
                        }
                        else
                        {
                            ctrl->resample = sifli_resample_open(2, ctrl->wave_samplerate, 44100);
                            RT_ASSERT(ctrl->resample);
                        }
                        if (ctrl->wave_channels == 2)
                        {
                            bytes = sifli_resample_process(ctrl->resample, (int16_t *)outBuf, last_frame_len, 0);
                            ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), bytes);
                        }
                        else
                        {
                            mono2stereo((int16_t *)outBuf, last_frame_len / 2, (int16_t *)outBuf2);
                            bytes = sifli_resample_process(ctrl->resample, (int16_t *)outBuf2, last_frame_len * 2, 0);
                            ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), bytes);
                        }
                    }
                    else if (ctrl->wave_channels == 1)
                    {
                        mono2stereo((int16_t *)outBuf, last_frame_len / 2, (int16_t *)outBuf2);
                        ret = audio_write(ctrl->client, (uint8_t *)outBuf2, last_frame_len * 2);
                    }
                    else
                    {
                        ret = audio_write(ctrl->client, (uint8_t *)outBuf, last_frame_len);
                    }
                }
                else
                {
                    ret = audio_write(ctrl->client, (uint8_t *)outBuf, last_frame_len);
                }
check_write_result:
                if (ret == 0)
                {
                    LOG_D("wav cache full");
                    continue;
                }
                else if (ret == -1)
                {
                    continue;
                }
                cache_full_occured = 0;
                callback_playing_progress(ctrl);
            }
        }
        int len;
#if RT_USING_DFS
        if (ctrl->is_file)
            len = read(ctrl->fd, outBuf, WAV_FRAME_SIZE);
        else
#endif
            len = buf_read(ctrl, outBuf, WAV_FRAME_SIZE);
        if (len <= 0)
        {
            //wait cache out to speaker
            uint32_t cache_time_ms = 150;
            audio_ioctl(ctrl->client, 1, &cache_time_ms);
            rt_thread_mdelay(cache_time_ms + 20);

            if (ctrl->loop_times > 0)
            {
                ctrl->loop_times--;
                ctrl->is_file_end = 0;
#if RT_USING_DFS
                if (ctrl->is_file)
                    lseek(ctrl->fd, ctrl->tag_len + 0, SEEK_SET);
                else
#endif
                    buf_seek(ctrl, ctrl->tag_len + 0);
                LOG_I("wav--loop");
                last_frame_len = WAV_FRAME_SIZE;
                continue;
            }

            LOG_I("wav--end");
            is_paused = 1;
            if (ctrl->callback &&  0 == is_closing)
                ctrl->callback(as_callback_cmd_play_to_end, ctrl->userdata, 0);
            continue;
        }
        else
        {
            last_frame_len = len;
        }
        ctrl->frame_index++;
        nFrames++;
        if (ctrl->wave_channels != old_channels || ctrl->wave_samplerate != old_samplerate)
        {
            if (ctrl->client)
            {
                audio_close(ctrl->client);
                ctrl->client = NULL;
            }
            if (ctrl->resample)
            {
                sifli_resample_close(ctrl->resample);
                ctrl->resample = NULL;
            }

        }
        if (!ctrl->client)
        {
            audio_parameter_t pa = {0};
            pa.write_bits_per_sample = 16;
            pa.write_channnel_num = ctrl->wave_channels;
            pa.write_samplerate = ctrl->wave_samplerate;
            pa.write_cache_size = WAV_FRAME_SIZE * 2 + 10;
            old_channels = ctrl->wave_channels;
            old_samplerate = ctrl->wave_samplerate;
            if (ctrl->wave_samplerate > 16000)
            {
                pa.write_cache_size = WAV_FRAME_SIZE * 8 + 10;
            }
            ctrl->frameinfo.samplerate = ctrl->wave_samplerate;
            ctrl->frameinfo.channels = old_channels;
            ctrl->frameinfo.one_channel_sampels = WAV_FRAME_SIZE / 2 / old_channels;
            ctrl->client = audio_open(ctrl->type, AUDIO_TX, &pa, mp3_audio_server_callback, (void *)ctrl);
            RT_ASSERT(ctrl->client);
            LOG_I("wav open ctrl=0x%x, client=0x%x, c=%d samrate=%d",
                  ctrl, ctrl->client, ctrl->wave_channels, ctrl->wave_samplerate);
        }
        LOG_D("nFrames=%d", nFrames);
        int ret;
        if (audio_device_is_a2dp_sink())
        {
            uint32_t bytes;
            if (ctrl->wave_samplerate != 44100 && ctrl->wave_samplerate)
            {
                if (!ctrl->resample)
                {
                    ctrl->resample = sifli_resample_open(2, ctrl->wave_samplerate, 44100);
                    RT_ASSERT(ctrl->resample);
                }
                if (ctrl->wave_channels == 2)
                {
                    bytes = sifli_resample_process(ctrl->resample, (int16_t *)outBuf, last_frame_len, 0);
                    ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), bytes);
                }
                else
                {
                    mono2stereo((int16_t *)outBuf, last_frame_len / 2, (int16_t *)outBuf2);
                    bytes = sifli_resample_process(ctrl->resample, (int16_t *)outBuf2, last_frame_len * 2, 0);
                    ret = audio_write(ctrl->client, (uint8_t *)sifli_resample_get_output(ctrl->resample), bytes);
                }
            }
            else if (ctrl->wave_channels == 1)
            {
                mono2stereo((int16_t *)outBuf, last_frame_len / 2, (int16_t *)outBuf2);
                ret = audio_write(ctrl->client, (uint8_t *)outBuf2, last_frame_len * 2);
            }
            else
            {
                ret = audio_write(ctrl->client, (uint8_t *)outBuf, last_frame_len);
            }
        }
        else
        {
            ret = audio_write(ctrl->client, (uint8_t *)outBuf, len);
        }

        if (ret == -1)
        {
            LOG_I("wav suspend");
            rt_thread_mdelay(20);
        }
        if (ret == 0)
        {
            LOG_D("wav cache full");
            cache_full_occured = 1;
            rt_thread_mdelay(10);
        }
        else
        {
            cache_full_occured = 0;
            callback_playing_progress(ctrl);
            rt_event_send(ctrl->event, MP3_EVENT_FLAG_DECODE);
        }
    }
    if (ctrl->client)
        audio_close(ctrl->client);
    ctrl->client = NULL;
    LOG_I("wav exit..");
#if RT_USING_DFS
    if (ctrl->is_file)
        close(ctrl->fd);
#endif
    if (ctrl->cache_ptr)
    {
        audio_mem_free(ctrl->cache_ptr);
        ctrl->cache_ptr = NULL;
    }
    audio_mem_free(outBuf);
    audio_mem_free(outBuf2);

    mp3_slist_lock(ctrl);
    while (1)
    {
        mp3_cmt_t  *p_cmd;
        rt_slist_t *first = rt_slist_first(&ctrl->cmd_slist);
        if (!first)
            break;
        p_cmd = rt_slist_entry(first, mp3_cmt_t, snode);
        rt_slist_remove(&ctrl->cmd_slist, first);

        audio_mem_free(p_cmd);
    }
    mp3_slist_unlock(ctrl);
    rt_mutex_delete(ctrl->cmd_slist_mutex);
    rt_event_delete(ctrl->event);
    rt_event_send(ctrl->api_event, API_EVENT_CLOSE);
    audio_mem_free(ctrl);
    LOG_I("wav exit done");
}


static int get_frame_info(mp3ctrl_handle ctrl, MP3FrameInfo *mp3FrameInfo)
{
    int ret = -1;
#if PKG_USING_LIBHELIX
    int offset = 0;
    int sync_cnt = 0;
    int err_cnt = 0;
    MP3FrameInfo temp = {0};
    HMP3Decoder hMP3Decoder = MP3InitDecoder();
    temp.nChans = 1;
    //used MP3Decode & MP3GetLastFrameInfo?
    while (1)
    {
        load_file_to_cache(ctrl);
        offset = MP3FindSyncWord(ctrl->cache_read_ptr, ctrl->cache_bytesLeft);
        if (offset >= 0)
        {
            sync_cnt = 0;
            ctrl->cache_read_ptr += offset;
            ctrl->cache_bytesLeft -= offset;
            load_file_to_cache(ctrl);
            memcpy(mp3FrameInfo, &temp, sizeof(MP3FrameInfo));
            if (ERR_MP3_NONE == MP3GetNextFrameInfo(hMP3Decoder, &temp, ctrl->cache_read_ptr))
            {
                err_cnt = 0;
                memcpy(mp3FrameInfo, &temp, sizeof(MP3FrameInfo));
                LOG_I("mp3: B=%d L=%d C=%d V=%d O=%d S=%d", temp.bitsPerSample, temp.layer, temp.nChans, temp.version, temp.outputSamps, temp.samprate);
                ret = 0;
                break;
            }
            else
            {
                err_cnt++;
                if (err_cnt > 100 && !ctrl->is_record) //record mp3 file compute time error
                {
                    LOG_I("mp3: unknow file");
                    break;
                }
            }
        }
        else if (ctrl->is_file_end)
        {
            ctrl->is_file_end = 0;
            break;
        }
        else
        {
            sync_cnt++;
            if (sync_cnt > 100)
            {
                break;
            }
        }
        ctrl->cache_read_ptr++;
        ctrl->cache_bytesLeft--;
    }
    MP3FreeDecoder(hMP3Decoder);
#endif
    return ret;
}
static mp3ctrl_handle mp3ctrl_open_real(audio_type_t type,
                                        const char *filename,
                                        uint32_t len,
                                        audio_server_callback_func callback,
                                        void *callback_userdata)
{
    MP3FrameInfo frameinfo;
    uint32_t     file_size;
    if (!filename)
    {
        LOG_E("mp3 parameter error");
        return NULL;
    }
    LOG_I("mp3 open %s callback=0x%x", filename, (uint32_t)callback);
    mp3ctrl_handle handle = audio_mem_calloc(1, sizeof(struct mp3ctrl_t));
    RT_ASSERT(handle);
    handle->last_display_seconds = -1;
    handle->is_file = (len == -1);
#if RT_USING_DFS
    if (handle->is_file)
    {
        LOG_I("mp3 open %s", filename);
        struct stat stat_buf;
        stat(filename, &stat_buf);
        file_size = stat_buf.st_size;
        handle->mp3_data_len = 0; //fileszie ?
        handle->fd = open(filename, O_RDONLY | O_BINARY);
        if (handle->fd < 0)
        {
            LOG_E("mp3 open %s error fd=%d", filename, handle->fd);
            audio_mem_free(handle);
            return NULL;
        }
    }
    else
#endif
    {
        LOG_I("mp3 open buffer");
        handle->filename = filename;
        handle->fd = 0;
        file_size = len;
        handle->mp3_data_len = file_size;
    }
    handle->tag_len = audio_parse_mp3_id3v2(handle);
    if (handle->tag_len >= file_size)
    {
        handle->mp3_data_len = 0;
    }
    else
    {
        handle->mp3_data_len = file_size;
    }

    if (handle->is_wave == 0)
    {
        handle->cache_ptr = audio_mem_malloc(CACHE_BUF_SIZE);
        RT_ASSERT(handle->cache_ptr);
    }
    handle->cache_bytesLeft = 0;
    if (handle->is_wave == 0 && get_frame_info(handle, &frameinfo) == 0)
    {
        handle->total_time_in_seconds = (handle->mp3_data_len - handle->tag_len) * 8 / frameinfo.bitrate;
        handle->bitrate = frameinfo.bitrate;
        LOG_I("f=%d d=%d time=%d channel=%d samprate=%d samps=%d", file_size, handle->mp3_data_len,
              handle->total_time_in_seconds, frameinfo.nChans, frameinfo.samprate, frameinfo.outputSamps);
    }
    else if (handle->is_wave == 1 && handle->wave_bytes_per_second != 0)
    {
        handle->total_time_in_seconds = (handle->mp3_data_len - handle->tag_len) / handle->wave_bytes_per_second;
    }
    else
    {
        LOG_I("get frame info error");
    }
#if RT_USING_DFS
    if (handle->is_file)
        lseek(handle->fd, handle->tag_len, SEEK_SET);
    else
#endif
        buf_seek(handle, handle->tag_len);
    handle->cache_bytesLeft = 0;
    handle->is_file_end = 0;
    handle->type = type;
    handle->filename = filename;
    handle->callback = callback;
    handle->userdata = callback_userdata;
    handle->api_event = rt_event_create("mp3sync", RT_IPC_FLAG_FIFO);
    RT_ASSERT(handle->api_event);
    handle->magic = MP3_HANDLE_MAGIC;
    rt_slist_init(&handle->cmd_slist);
    handle->cmd_slist_mutex = rt_mutex_create("mp3", RT_IPC_FLAG_FIFO);
    handle->event = rt_event_create("mp3", RT_IPC_FLAG_FIFO);
#if !defined(SYS_HEAP_IN_PSRAM)
    if (handle->is_wave)
        handle->thread = rt_thread_create("wave", wave_thread_entry_file, handle, 2048, RT_THREAD_PRIORITY_HIGH + 1, 10);
    else
        handle->thread = rt_thread_create("mp3", mp3ctrl_thread_entry_file, handle, 2048, RT_THREAD_PRIORITY_HIGH + 1, 10);
    RT_ASSERT(handle->thread);
#else
    rt_err_t err;
    handle->stack_addr = (uint8_t *)app_sram_alloc(2048);
    RT_ASSERT(handle->stack_addr);
    handle->thread = audio_mem_malloc(sizeof(struct rt_thread));
    RT_ASSERT(handle->thread);
    rt_memset(handle->thread, 0, sizeof(struct rt_thread));
    if (handle->is_wave)
        err = rt_thread_init(handle->thread, "wave", wave_thread_entry_file, handle, handle->stack_addr, 2048, RT_THREAD_PRIORITY_HIGH + 1, 10);
    else
        err = rt_thread_init(handle->thread, "mp3", mp3ctrl_thread_entry_file, handle, handle->stack_addr, 2048, RT_THREAD_PRIORITY_HIGH + 1, 10);
    RT_ASSERT(RT_EOK == err);
#endif
    rt_thread_startup(handle->thread);
    return handle;
}

PUBLIC_API mp3ctrl_handle mp3ctrl_open(audio_type_t type,
                                       const char *filename,
                                       audio_server_callback_func callback,
                                       void *callback_userdata)
{
    LOG_I("%s call by thread id=%p name=%s", __FUNCTION__, rt_thread_self(), rt_thread_self()->name);

    return mp3ctrl_open_real(type, filename, -1, callback, callback_userdata);
}
PUBLIC_API mp3ctrl_handle mp3ctrl_open_buffer(audio_type_t type,
        const char *buf,
        uint32_t buf_len,
        audio_server_callback_func callback,
        void *callback_userdata)
{
    return mp3ctrl_open_real(type, buf, buf_len, callback, callback_userdata);
}
PUBLIC_API int mp3ctrl_close(mp3ctrl_handle handle)
{
    rt_uint32_t evt;
    rt_event_t event;
    LOG_I("%s call by thread id=%p name=%s", __FUNCTION__, rt_thread_self(), rt_thread_self()->name);
    if (!handle || handle->magic != MP3_HANDLE_MAGIC)
    {
        LOG_I("mp3ctrl_close error h=0x%x\n", (uint32_t)handle);
        return -1;
    }
    sifli_resample_t *resample = handle->resample;

    LOG_I("mp3ctrl_close h=0x%x\n", (uint32_t)handle);
#if defined(SYS_HEAP_IN_PSRAM)
    void *stack_addr = handle->stack_addr;
    rt_thread_t thread = (rt_thread_t)handle->thread;
#endif
    event = handle->api_event;
    handle->magic = ~MP3_HANDLE_MAGIC; //now allow use handle lator
    rt_event_send(handle->event, MP3_EVENT_FLAG_CLOSE);
    rt_event_recv(event, API_EVENT_CLOSE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
    rt_event_delete(event);
#if defined(SYS_HEAP_IN_PSRAM)
    while (1)
    {
        if ((thread->stat & RT_THREAD_STAT_MASK) == RT_THREAD_CLOSE)
        {
            break;
        }
        rt_thread_mdelay(2);
        LOG_I("wait mp3 exit");
    }
    app_sram_free(stack_addr);
    audio_mem_free(thread);
#endif
    if (resample)
    {
        sifli_resample_close(resample);
    }
    return 0;
}

PUBLIC_API int mp3ctrl_resume(mp3ctrl_handle handle)
{
    if (!handle || handle->magic != MP3_HANDLE_MAGIC)
    {
        return -1;
    }
    rt_event_send(handle->event, MP3_EVENT_FLAG_RESUME);
    rt_uint32_t evt;
    rt_event_recv(handle->api_event, API_EVENT_RESUME, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
    return 0;
}

PUBLIC_API int mp3ctrl_play(mp3ctrl_handle handle)
{
    if (!handle || handle->magic != MP3_HANDLE_MAGIC)
    {
        LOG_I("mp3ctrl_play error h=0x%x\n", (uint32_t)handle);
        return -1;
    }
    LOG_I("mp3ctrl_play h=0x%x\n", (uint32_t)handle);
    rt_uint32_t evt;
    rt_event_send(handle->event, MP3_EVENT_FLAG_PLAY);
    rt_event_recv(handle->api_event, API_EVENT_PLAY, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
    LOG_I("mp3ctrl_play ok");
    return 0;
}

PUBLIC_API int mp3ctrl_pause(mp3ctrl_handle handle)
{
    if (!handle || handle->magic != MP3_HANDLE_MAGIC)
    {
        LOG_I("mp3ctrl_pause error h=0x%x\n", (uint32_t)handle);
        return -1;
    }
    LOG_I("mp3ctrl_pause h=0x%x\n", (uint32_t)handle);
    rt_event_send(handle->event, MP3_EVENT_FLAG_PAUSE);
    rt_uint32_t evt;
    rt_event_recv(handle->api_event, API_EVENT_PAUSE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
    LOG_I("mp3ctrl_pause ok");
    return 0;
}

PUBLIC_API int mp3ctrl_ioctl(mp3ctrl_handle handle, int cmd, uint32_t param)
{
    if (!handle || handle->magic != MP3_HANDLE_MAGIC)
        return -1;
    if (cmd == 0)
    {
        handle->loop_times = param;
        return 0;
    }
    if (cmd == 1)
    {
        mp3_ioctl_cmd_param_t *p = (mp3_ioctl_cmd_param_t *)param;
        if (!p)
            return -1;

        if ((p->len == -1 && handle->is_file == 0) || (p->len != -1 && handle->is_file))
        {
            LOG_E("error switch between file and buf");
            return -1;
        }

        mp3_cmt_t *cmd_msg = audio_mem_calloc(1, sizeof(mp3_cmt_t));
        RT_ASSERT(cmd_msg);
        cmd_msg->cmd = MP3_NEXT;
#if RT_USING_DFS
        if (p->len == -1)
        {
            cmd_msg->next_is_file = 1;
            LOG_I("mp3 next %s", p->filename);
            struct stat stat_buf;
            stat(p->filename, &stat_buf);
            cmd_msg->cmd_paramter1 = (void *)stat_buf.st_size;
            cmd_msg->next_fd = open(p->filename, O_RDONLY | O_BINARY);
            if (cmd_msg->next_fd < 0)
            {
                LOG_E("mp3 next %s error fd=%d", p->filename, cmd_msg->next_fd);
                audio_mem_free(cmd_msg);
                return -1;
            }
            char riff[4] = {0};
            read(cmd_msg->next_fd, riff, 4);
            lseek(cmd_msg->next_fd, 0, SEEK_SET);
            if ((!memcmp((const char *)riff, "RIFF", 4) && !handle->is_wave)
                    || (memcmp((const char *)riff, "RIFF", 4) && handle->is_wave))
            {
                close(cmd_msg->next_fd);
Error:
                audio_mem_free(cmd_msg);
                LOG_E("error switch between mp3 and wav");
                return -1;
            }

        }
        else
#endif
        {
            LOG_I("mp3 next buffer");
            if ((!memcmp((const char *)p->filename, "RIFF", 4) && !handle->is_wave)
                    || (memcmp((const char *)p->filename, "RIFF", 4) && handle->is_wave))
            {
#if RT_USING_DFS
                goto Error;
#else
                audio_mem_free(cmd_msg);
                LOG_E("error switch between mp3 and wav");
                return -1;
#endif
            }
            cmd_msg->next_buffer = (uint8_t *)p->filename;
            cmd_msg->cmd_paramter1 = (void *)p->len;
        }
        mp3_slist_lock(handle);
        rt_slist_append(&handle->cmd_slist, &cmd_msg->snode);
        mp3_slist_unlock(handle);
        rt_event_send(handle->event, MP3_EVENT_FLAG_NEXT);
        rt_uint32_t evt;
        rt_event_recv(handle->api_event, API_EVENT_NEXT, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        LOG_I("mp3ctrl next ok");
        return 0;
    }
    if (cmd == 2)
    {
        uint32_t priority = param;
        rt_thread_control(handle->thread, RT_THREAD_CTRL_CHANGE_PRIORITY, &priority);
        return 0;
    }
    return -1;
}

int mp3ctrl_getinfo(const char *filename, mp3_info_t *info)
{
#if RT_USING_DFS
    int ret = 0;
    MP3FrameInfo frameinfo;
    uint32_t     file_size;
    if (!filename || !info)
    {
        LOG_E("mp3 parameter error");
        return -1;
    }
    LOG_I("mp3 open %s", filename);
    mp3ctrl_handle handle = audio_mem_calloc(1, sizeof(struct mp3ctrl_t));
    RT_ASSERT(handle);

    handle->is_file = 1;

    LOG_I("mp3 open %s", filename);
    struct stat stat_buf;
    stat(filename, &stat_buf);
    file_size = stat_buf.st_size;
    handle->mp3_data_len = 0; //fileszie ?
    handle->fd = open(filename, O_RDONLY | O_BINARY);
    if (handle->fd < 0)
    {
        LOG_E("mp3 open %s error", filename);
        ret = -1;
        goto Exit;
    }

    handle->tag_len = audio_parse_mp3_id3v2(handle);
    if (handle->tag_len >= file_size)
    {
        handle->mp3_data_len = 0;
    }
    else
    {
        handle->mp3_data_len = file_size;
    }
    if (handle->is_wave == 0)
    {
        handle->cache_ptr = audio_mem_malloc(CACHE_BUF_SIZE);
        RT_ASSERT(handle->cache_ptr);
    }
    if (handle->is_wave == 0 && get_frame_info(handle, &frameinfo) == 0)
    {
        info->total_time_in_seconds = (handle->mp3_data_len - handle->tag_len) * 8 / frameinfo.bitrate;
        info->samplerate = frameinfo.samprate;
        info->channels = frameinfo.nChans;
        info->one_channel_sampels = frameinfo.outputSamps / frameinfo.nChans;
        ret = 0;
    }
    else if (handle->is_wave == 1 && handle->wave_bytes_per_second != 0)
    {
        handle->total_time_in_seconds = (handle->mp3_data_len - handle->tag_len) / handle->wave_bytes_per_second;
        info->total_time_in_seconds = handle->total_time_in_seconds;
        info->samplerate = handle->wave_samplerate;
        info->channels = handle->wave_channels;
    }
    else
    {
        LOG_I("get frame info error");
        ret = -1;
    }
    close(handle->fd);
Exit:
    if (handle->cache_ptr)
    {
        audio_mem_free(handle->cache_ptr);
    }
    audio_mem_free(handle);
    return 0;
#else
    return -1;
#endif
}

PUBLIC_API int mp3ctrl_seek(mp3ctrl_handle handle, off_t frame_offset)
{

    if (!handle || handle->magic != MP3_HANDLE_MAGIC)
    {
        LOG_I("mp3 seek h=0x%x", (uint32_t)handle);
        return -1;
    }
    LOG_I("mp3 seek h=0x%x o=%d", (uint32_t)handle, frame_offset);
    mp3_cmt_t *cmd = audio_mem_calloc(1, sizeof(mp3_cmt_t));
    RT_ASSERT(cmd);
    cmd->cmd = MP3_SEEK;
    cmd->cmd_paramter1 = (void *)frame_offset;
    mp3_slist_lock(handle);
    rt_slist_append(&handle->cmd_slist, &cmd->snode);
    mp3_slist_unlock(handle);
    rt_event_send(handle->event, MP3_EVENT_FLAG_SEEK);
    rt_uint32_t evt;
    rt_event_recv(handle->api_event, API_EVENT_SEEK, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

    return 0;
}


/* wav input */
#define MKTAG(a,b,c,d) ((a) | ((b) << 8) | ((c) << 16) | ((unsigned)(d) << 24))

static uint32_t wav_read_header(mp3ctrl_handle ctrl)
{
    uint32_t got_fmt = 0;
    uint32_t tag = 0;
    uint32_t size;
    uint32_t next_tag_ofs = 0;
    uint32_t wave_samplerate;
    uint32_t data_size;
    wave_read(ctrl, &tag, 4);    //RIFF
    wave_read(ctrl, &size, 4);
    if (tag != MKTAG('R', 'I', 'F', 'F'))
        return -1;

    wave_read(ctrl, &tag, 4);

    if (tag != MKTAG('W', 'A', 'V', 'E'))
    {
        LOG_I("not WAVE");
        return -1;
    }
    next_tag_ofs = 12;

    for (;;)
    {
        wave_seek(ctrl, next_tag_ofs);
        wave_read(ctrl, &tag, 4);
        wave_read(ctrl, &size, 4);

        switch (tag)
        {
        case MKTAG('f', 'm', 't', ' '):
            got_fmt = 1;
            tag = 0;
            wave_read(ctrl, &tag, 2); //format
            if (tag != 1)
            {
                LOG_I("wave not pcm");
                return -1;
            }
            tag = 0;
            wave_read(ctrl, &tag, 2); //channels
            ctrl->wave_channels = (uint8_t)tag;
            wave_read(ctrl, &tag, 4); //samplerate
            ctrl->wave_samplerate = tag;
            wave_read(ctrl, &tag, 4); //byte_rates
            ctrl->wave_bytes_per_second = tag;
            tag = 0;
            wave_read(ctrl, &tag, 2);
            tag = 0;
            wave_read(ctrl, &tag, 2);
            if (tag != 16)
            {
                LOG_I("wave bits not 16");
                return -1;
            }

            break;
        case MKTAG('d', 'a', 't', 'a'):
            if (!got_fmt)
            {
                LOG_I("wave not fmt");
                return -1;
            }
            if (size != -1)
            {
                data_size = size;
                return next_tag_ofs + 8;
            }
            else
            {
                return -1;
            }
        default:
            break;
        }
        next_tag_ofs += 8 + size;
    }

    return -1;
}

#define MP3_TEST_CMD 0

#if defined(RT_USING_UTEST)
    #undef  MP3_TEST_CMD
    #define MP3_TEST_CMD 1
#endif

#ifdef RT_USING_FINSH

#if MP3_TEST_CMD

/*
    most play two diffrent type
    mp3 open  filename.mp3 <music/alarm/ring/notify>
    mp3 play 1/2
    mp3 pause 1/2
    mp3 close 1/2
    mp3 open filename.mp3
example1:
    mp3 open a.mp3 music //open a.mp3 as music for first, default is music
    mp3 open b.mp3 ring  //open b.mp3 as ring for second
    mp3 play 1           //play first, play a.mp3
    mp3 play 2           //play second, play b.mp3,  ring has high priority than music
    mp3 close 1
    mp3 close 2

simple example:
    mp3 open a.mp3
    mp3 play 1
    mp3 close 1
*/
static audio_client_t re;
static int audio_record_callback(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_data_coming)
    {
        LOG_I("record...\n");
    }
    return 0;
}
static void mp3(uint8_t argc, char **argv)
{
    char *value = NULL;

    if (argc < 3)
    {
        rt_kprintf("usage: \r\n    mp3 open  filename.mp3 type\r\n    mp3 pause 1/2\r\n    mp3 close 1/2\r\n");
        return;
    }
    if (strcmp(argv[1], "stress") == 0)
    {
        int i = 0x80000;
        value = argv[2];
again:
        LOG_I("1......open1 %s", value);
        g_handle1 = mp3ctrl_open(AUDIO_TYPE_LOCAL_MUSIC, (void *)value, NULL, NULL);
        LOG_I("2......g_handle1=0x%x", g_handle1);
        LOG_I("3. ....play 1");
        RT_ASSERT(g_handle1);
        mp3ctrl_play(g_handle1);
        rt_thread_mdelay(2000);
        LOG_I("4.     close");
        mp3ctrl_close(g_handle1);
        i--;
        if (i > 0)
            goto again;
        return;
    }
    if (strcmp(argv[1], "ro") == 0)
    {
        /*open audio record*/
        audio_parameter_t pa = {0};
        pa.write_bits_per_sample = 16;
        pa.write_channnel_num = 1;
        pa.write_samplerate = 16000;
        pa.write_cache_size = 2048;
        pa.read_bits_per_sample = 16;
        pa.read_channnel_num = 1;
        pa.read_samplerate = 16000;
        pa.read_cache_size = 0;

        re = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, audio_record_callback, (void *)NULL);
        return;
    }
    if (strcmp(argv[1], "rc") == 0)
    {
        audio_close(re);
        return;
    }

    if (strcmp(argv[1], "open") == 0)
    {
        audio_type_t type = AUDIO_TYPE_LOCAL_MUSIC;
        value = argv[2];
        if (argc > 3)
        {
            if (argv[3][0] == 'a')
                type = AUDIO_TYPE_ALARM;
            else if (argv[3][0] == 'r')
                type = AUDIO_TYPE_LOCAL_RING;
            else if (argv[3][0] == 'n')
                type = AUDIO_TYPE_NOTIFY;
            else
                type = AUDIO_TYPE_LOCAL_MUSIC;
        }
        if (g_handle1 == NULL)
        {
            LOG_I("open1 %s type=%d", value, type);
            g_handle1 = mp3ctrl_open(type, (void *)value, NULL, NULL);
            mp3ctrl_ioctl(g_handle1, 0, -1);
            LOG_I("g_handle1=0x%x", g_handle1);
        }
        else if (g_handle2 == NULL)
        {
            LOG_I("open2 %s type=%d", value, type);
            g_handle2 = mp3ctrl_open(type, (void *)value, NULL, NULL);
            LOG_I("g_handle2=0x%x", g_handle2);
        }
        else
        {
            rt_kprintf("busynow two mp3 in playing\r\n");
            return;
        }
    }
    else if (strcmp(argv[1], "play") == 0)
    {
        if (g_handle1 && argv[2][0] == '1')
        {
            LOG_I("play 1");
            mp3ctrl_play(g_handle1);
            return;
        }
        if (g_handle2 && argv[2][0] == '2')
        {
            LOG_I("play 2");
            mp3ctrl_play(g_handle2);
            return;
        }
        LOG_I("play none");
    }
    else if (strcmp(argv[1], "pause") == 0)
    {
        if (g_handle1 && argv[2][0] == '1')
        {
            LOG_I("pause 1");
            mp3ctrl_pause(g_handle1);
            return;
        }
        if (g_handle2 && argv[2][0] == '2')
        {
            LOG_I("pause 2");
            mp3ctrl_pause(g_handle2);
            return;
        }
        LOG_I("pause none");
    }
    else if (strcmp(argv[1], "resume") == 0)
    {
        if (g_handle1 && argv[2][0] == '1')
        {
            LOG_I("resume 1");
            mp3ctrl_resume(g_handle1);
            return;
        }
        if (g_handle2 && argv[2][0] == '2')
        {
            LOG_I("resume 2");
            mp3ctrl_resume(g_handle2);
            return;
        }
        LOG_I("resume none");
    }
    else if (strcmp(argv[1], "close") == 0)
    {
        if (g_handle1 && argv[2][0] == '1')
        {
            LOG_I("close 1");
            mp3ctrl_close(g_handle1);
            g_handle1 = NULL;
            return;
        }
        if (g_handle2 && argv[2][0] == '2')
        {
            LOG_I("close 2");
            mp3ctrl_close(g_handle2);
            g_handle2 = NULL;
            return;
        }
        LOG_I("stop none");
    }
    else if (strcmp(argv[1], "next") == 0)
    {
        if (g_handle1)
        {
            mp3_ioctl_cmd_param_t para;
            para.filename = argv[2];
            para.len = -1;
            mp3ctrl_ioctl(g_handle1, 1, (uint32_t)&para);
            return;
        }
        LOG_I("next none");
    }
}

MSH_CMD_EXPORT(mp3, mp3 commnad);

#if 0

static void print_buffer(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        rt_kprintf("print_buffer data == NULL || len == 0 \r\n");
        return;
    }

    rt_kprintf("buffer len=%d\r\n[", len);
    for (int i = 0; i < len; i++)
    {
        rt_kprintf("%02X ", data[i]);
        if ((i + 1) % 16 == 0)
            rt_kprintf("\r\n");
    }
    rt_kprintf("] \r\n");
}
static void id3(uint8_t argc, char **argv)
{
    mp3_id3_info_t info = {0};

    if (argc < 2)
    {
        return;
    }
    rt_kprintf("--start=%d\r\n", rt_tick_get());
    mp3_get_id3_start(argv[1], &info);
    rt_kprintf("%s\r\n", info.title);
    rt_kprintf("%s\r\n", info.artist);
    rt_kprintf("%s\r\n", info.album);
    rt_kprintf("--end=%d\r\n", rt_tick_get());
    mp3_info_t info2;
    mp3ctrl_getinfo(argv[1], &info2);
    rt_kprintf("c=%d, s=%d, t=%d\r\n", info2.channels, info2.samplerate, info2.total_time_in_seconds);
    rt_kprintf("--end2=%d\r\n", rt_tick_get());
    mp3_get_id3_end(&info);
}

MSH_CMD_EXPORT(id3, id3 commnad);
#endif

#endif //SOLUTION_WATCH

#endif //RT_USING_FINSH


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
