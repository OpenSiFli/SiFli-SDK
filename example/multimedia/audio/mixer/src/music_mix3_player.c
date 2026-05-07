/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include "os_adaptor.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif

#include "sifli_resample.h"

#define PUBLIC_API
#ifndef PKG_USING_LIBHELIX
    #error "should config PKG_USING_LIBHELIX"
#endif

#include "audio_server.h"
#include "mp3dec.h"


#define MIX_EVENT_MSG           (1<<0)
#define MIX_EVENT_STOP          (1<<1)
#define MIX_EVENT_DECODE        (1<<2)

#define MIX_EVENT_ALL           (MIX_EVENT_MSG|MIX_EVENT_STOP|MIX_EVENT_DECODE)

#define MP3_ONE_MONO_FRAME_SIZE     (MAX_NGRAN * MAX_NSAMP * 2)

#define RESAMPLE_FRAME_SIZE     64
#define RESAMPLE_RING_SIZE      (48000 / 8000 * RESAMPLE_FRAME_SIZE)

#define CACHE_BUF_SIZE          (3 * MAINBUF_SIZE + 100)
typedef struct
{
    HMP3Decoder             hMP3Decoder;
    short                   *mp3_decode_out;
    sifli_resample_t        *resample;
    uint32_t                samplerate;
    uint32_t                ch;
    int                     fd;
    uint32_t                mp3_data_len;
    uint32_t                tag_len;
    char                    filename[256];
    uint32_t                frame_index;
    struct rt_ringbuffer    *rb_resampled;
    struct rt_ringbuffer    decoded_ring;
    uint8_t                 ring_pool[MP3_ONE_MONO_FRAME_SIZE * 2 + 16];
    uint8_t                 cache_ptr[CACHE_BUF_SIZE];
    uint8_t                 *cache_read_ptr;
    int                     cache_bytesLeft;
    uint8_t                 is_file_end;
    uint8_t                 is_wave;
    uint8_t                 is_wav_pcm;
    uint8_t                 bits;
    uint8_t                 is_working;

} mix_stream_t;

typedef enum
{
    MIX_CMD_START   = 0,
    MIX_CMD_STOP    = 1,
} mix_cmd_e;

typedef struct
{
    rt_slist_t  node;
    mix_cmd_e   cmd_type;
    char        file_name[256];
} mix_cmd_t;

typedef struct
{
    uint32_t            magic;
    rt_thread_t         thread;
    rt_event_t          event;
    rt_slist_t          cmd_slist;
    mix_stream_t        stream[3];
    audio_client_t      client;
} mix_player_t;

static int find_sync_in_cache(mix_stream_t *ctrl);
static uint32_t wav_read_header(mix_stream_t *ctrl);
static uint32_t audio_parse_mp3_id3v2(mix_stream_t *s);
static int load_file_to_cache(mix_stream_t *ctrl);

static inline void stereo2mono(int16_t *stereo, uint32_t samples, int16_t *mono)
{
    for (int i = 0; i < samples / 2; i++)
    {
        *mono++ = *stereo++;
        stereo++;
    }
}

static int ensure_resample_ready(mix_stream_t *s)
{
    if (s->samplerate == 48000)
    {
        return 0;
    }
    if (!s->rb_resampled)
    {
        s->rb_resampled = rt_ringbuffer_create(RESAMPLE_RING_SIZE);
        RT_ASSERT(s->rb_resampled);
    }
    if (!s->resample)
    {
        s->resample = sifli_resample_open(1, s->samplerate, 48000);
        RT_ASSERT(s->resample);
    }
    return 0;
}

static void deode_mp3(mix_stream_t *s)
{
    if (find_sync_in_cache(s) < 0)
    {
        lseek(s->fd, s->tag_len + 0, SEEK_SET);
        s->cache_read_ptr = s->cache_ptr;
        s->cache_bytesLeft = 0;
        s->is_file_end = 0;
        if (s->hMP3Decoder)
        {
            MP3FreeDecoder(s->hMP3Decoder);
            s->hMP3Decoder = MP3InitDecoder();
            RT_ASSERT(s->hMP3Decoder);
        }
        load_file_to_cache(s);
    }

    int err = MP3Decode(s->hMP3Decoder, &s->cache_read_ptr, &s->cache_bytesLeft, s->mp3_decode_out, 0, 0);
    if (err)
    {
        rt_kprintf("mp3 err=%d\n", err);
    }
    else
    {
        /* no error */
        int ret;
        uint32_t bytes, putted;
        MP3FrameInfo mp3FrameInfo;
        MP3GetLastFrameInfo(s->hMP3Decoder, &mp3FrameInfo);
        s->samplerate = mp3FrameInfo.samprate;
        s->ch = mp3FrameInfo.nChans;
        if (!s->samplerate)
        {
            rt_kprintf("mp3 error, not layer3\n");
            return;
        }
        if (mp3FrameInfo.samprate != 48000)
        {
            ensure_resample_ready(s);
        }
        bytes = mp3FrameInfo.outputSamps * 2;
        if (mp3FrameInfo.nChans == 2)
        {
            stereo2mono(s->mp3_decode_out, mp3FrameInfo.outputSamps, s->mp3_decode_out);
            bytes >>= 1;
        }
        memset((uint8_t *)s->mp3_decode_out, 0, bytes);
        putted = rt_ringbuffer_put(&s->decoded_ring, (uint8_t *)s->mp3_decode_out, bytes);
        if (putted != bytes)
        {
            rt_kprintf("mp3 ring full = %d\n", bytes);
        }
    }
}

static inline void float2pcm(float *fl, uint32_t samples, int16_t *pcm)
{
    int32_t v;
    for (int i = 0; i < samples; i++)
    {
        v = (int32_t)(*fl * 32767);
        if (v > 32767)
        {
            v = 32767;
        }
        else if (v < -32768)
        {
            v = -32768;
        }
        *pcm++ = (int16_t)v;
        fl++;
    }
}

static inline int16_t pcm_u8_to_s16(uint8_t v)
{
    return (int16_t)(((int32_t)v - 128) << 8);
}

static void decode_wav(mix_stream_t *s)
{
    uint32_t bytes, putted;
    float read_buf[MP3_ONE_MONO_FRAME_SIZE];
    uint32_t r;
    if (s->ch == 2)
    {
        if (s->is_wav_pcm)
        {
            if (s->bits == 16)
            {
                r = MP3_ONE_MONO_FRAME_SIZE * 2;
            }
            else if (s->bits == 8)
            {
                r = MP3_ONE_MONO_FRAME_SIZE;
            }
            else
            {
                rt_kprintf("unsupported wav pcm bits=%d\n", s->bits);
                return;
            }
        }
        else
        {
            r = MP3_ONE_MONO_FRAME_SIZE * sizeof(float);
        }
    }
    else
    {
        if (s->is_wav_pcm)
        {
            if (s->bits == 16)
            {
                r = MP3_ONE_MONO_FRAME_SIZE;
            }
            else if (s->bits == 8)
            {
                r = MP3_ONE_MONO_FRAME_SIZE / 2;
            }
            else
            {
                rt_kprintf("unsupported wav pcm bits=%d\n", s->bits);
                return;
            }
        }
        else
        {
            r = (MP3_ONE_MONO_FRAME_SIZE / 2) * sizeof(float);
        }
    }
    RT_ASSERT(r <= sizeof(read_buf));

    int len = read(s->fd, read_buf, r);
    if (len < 0)
    {
        return;
    }

    if (s->samplerate != 48000)
    {
        ensure_resample_ready(s);
    }

    if (len < r)
    {
        lseek(s->fd, s->tag_len + 0, SEEK_SET);
        s->is_file_end = 0;
        read(s->fd, read_buf, r);
    }

    if (s->is_wav_pcm)
    {
        if (s->bits == 16)
        {
            bytes = r;
            if (s->ch == 2)
            {
                bytes >>= 1;
                stereo2mono((int16_t *)read_buf, bytes, (int16_t *)read_buf);
            }
        }
        else
        {
            uint8_t *src = (uint8_t *)read_buf;
            int16_t *dst = (int16_t *)read_buf;
            uint32_t mono_samples = MP3_ONE_MONO_FRAME_SIZE / 2;
            if (s->ch == 2)
            {
                for (uint32_t i = 0; i < mono_samples; i++)
                {
                    int16_t l = pcm_u8_to_s16(src[2 * i]);
                    int16_t rch = pcm_u8_to_s16(src[2 * i + 1]);
                    dst[i] = (int16_t)(((int32_t)l + (int32_t)rch) / 2);
                }
            }
            else
            {
                for (int32_t i = (int32_t)mono_samples - 1; i >= 0; i--)
                {
                    dst[i] = pcm_u8_to_s16(src[i]);
                }
            }
            bytes = MP3_ONE_MONO_FRAME_SIZE;
        }
    }
    else
    {
        //rt_kprintf("wav float--------------\n");
        r >>= 2; //r= samples, r = MP3_ONE_MONO_FRAME_SIZE
        if (s->ch == 2)
        {
            float2pcm(read_buf, r, (int16_t *)read_buf);
            bytes = r;
            stereo2mono((int16_t *)read_buf, bytes, (int16_t *)read_buf);
        }
        else
        {
            float2pcm(read_buf, r, (int16_t *)read_buf);
            bytes = (r << 1);
        }
    }
    RT_ASSERT(bytes == MP3_ONE_MONO_FRAME_SIZE);
    putted = rt_ringbuffer_put(&s->decoded_ring, (uint8_t *)read_buf, bytes);
    if (bytes != putted)
    {
        rt_kprintf("wav full %d/%d\n", putted, bytes);
    }
}

static void decode_stream(mix_stream_t *s)
{
    if (!s->is_working)
    {
        return;
    }
    while (1)
    {
        if (rt_ringbuffer_data_len(&s->decoded_ring) >= MP3_ONE_MONO_FRAME_SIZE)
        {
            return;
        }
        if (s->is_wave)
        {
            decode_wav(s);
        }
        else
        {
            deode_mp3(s);
        }
    }
}

static int load_file_to_cache(mix_stream_t *ctrl)
{
    int readed = 0;
    if (ctrl->cache_bytesLeft < 2 * MAINBUF_SIZE && !ctrl->is_file_end)
    {
        if (ctrl->cache_bytesLeft > 0)
        {
            memcpy(ctrl->cache_ptr, ctrl->cache_read_ptr, ctrl->cache_bytesLeft);
        }
        readed = read(ctrl->fd, ctrl->cache_ptr + ctrl->cache_bytesLeft, CACHE_BUF_SIZE - ctrl->cache_bytesLeft);

        ctrl->cache_read_ptr = ctrl->cache_ptr;

        /* zero-pad to avoid finding false sync word after last frame (from old data in readBuf) */
        if (readed >= 0 && readed < CACHE_BUF_SIZE - ctrl->cache_bytesLeft)
        {
            memset(ctrl->cache_ptr + ctrl->cache_bytesLeft + readed, 0, CACHE_BUF_SIZE - ctrl->cache_bytesLeft - readed);
        }
        if (readed <= 0)
        {
            rt_kprintf("mp3 read end\n");
            ctrl->is_file_end = 1;
        }
        else
        {
            ctrl->cache_bytesLeft += readed;
        }

    }
    return 0;
}

static int find_sync_in_cache(mix_stream_t *ctrl)
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

static void stop_stream(mix_stream_t *s)
{
    if (s->resample)
    {
        sifli_resample_close(s->resample);
        s->resample = NULL;
    }
    if (s->fd >= 0)
    {
        close(s->fd);
        s->fd = -1;
    }
    if (s->hMP3Decoder)
    {
        MP3FreeDecoder(s->hMP3Decoder);
        s->hMP3Decoder = NULL;
    }
    if (s->mp3_decode_out)
    {
        audio_mem_free(s->mp3_decode_out);
        s->mp3_decode_out = NULL;
    }
    if (s->rb_resampled)
    {
        rt_ringbuffer_destroy(s->rb_resampled);
        s->rb_resampled = NULL;
    }
    s->is_working = 0;
    rt_kprintf("stop %s\n", s->filename);
}

static void find_and_stop_stream(mix_player_t *p, mix_cmd_t *cmd)
{
    for (int i = 0; i < 3; i++)
    {
        mix_stream_t *s = &p->stream[i];
        if (!strcmp(cmd->file_name, s->filename) && s->fd >= 0)
        {
            stop_stream(s);
        }
    }
}

static int start_stream(mix_player_t *p, mix_cmd_t *cmd)
{
    mix_stream_t *s = NULL;
    for (int i = 0; i < 3; i++)
    {
        s = &p->stream[i];
        if (!s->is_working)
        {
            strncpy(s->filename, cmd->file_name, sizeof(s->filename));
            break;
        }
    }
    if (s->is_working)
        return -2;

    s->is_working = 1;
    s->cache_bytesLeft = 0;
    s->cache_read_ptr = s->cache_ptr;

    uint32_t file_size;
    struct stat stat_buf;
    stat(s->filename, &stat_buf);
    file_size = stat_buf.st_size;
    s->fd = open(s->filename, O_RDONLY | O_BINARY);
    if (s->fd < 0)
    {
        rt_kprintf("mp3 open %s error fd=%d\n", s->filename, s->fd);
        stop_stream(s);
        return -1;
    }
    s->tag_len = audio_parse_mp3_id3v2(s);
    if (s->tag_len == -1)
    {
        rt_kprintf("audio parse error tag_len=%d\n", s->tag_len);
        stop_stream(s);
        return -3;
    }
    if (s->tag_len >= file_size)
    {
        s->mp3_data_len = 0;
    }
    else
    {
        s->mp3_data_len = file_size;
    }

    lseek(s->fd, s->tag_len + 0, SEEK_SET);

    if (!s->is_wave)
    {
        s->hMP3Decoder = MP3InitDecoder();
        RT_ASSERT(s->hMP3Decoder);
        s->mp3_decode_out = audio_mem_malloc(sizeof(short) * MAX_NCHAN * MAX_NGRAN * MAX_NSAMP);
        RT_ASSERT(s->mp3_decode_out);
    }
    rt_ringbuffer_init(&s->decoded_ring, s->ring_pool, sizeof(s->ring_pool));
    rt_kprintf("start %s\n", s->filename);
    return 0;
}

static void process_msg(mix_player_t *p, uint8_t is_exit)
{
    rt_slist_t *first;
    while (1)
    {
        rt_enter_critical();
        first = rt_slist_first(&p->cmd_slist);
        if (first)
        {
            rt_slist_remove(&p->cmd_slist, first);
        }
        rt_exit_critical();
        if (!first)
        {
            break;
        }

        mix_cmd_t *cmd = rt_container_of(first, mix_cmd_t, node);
        rt_kprintf("cmd=%d\n", cmd->cmd_type);
        if (!is_exit)
        {
            if (cmd->cmd_type == MIX_CMD_STOP)
            {
                find_and_stop_stream(p, cmd);
            }
            else if (cmd->cmd_type == MIX_CMD_START)
            {
                find_and_stop_stream(p, cmd);
                start_stream(p, cmd);
            }
        }
        audio_mem_free(cmd);
    }
}

static int audio_callback_func(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    mix_player_t *p = (mix_player_t *)callback_userdata;
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        rt_event_send(p->event, MIX_EVENT_DECODE);
    }
    return 0;
}

static bool is_can_mix(mix_player_t *p)
{
    mix_stream_t *s;
    bool can_mix = true;
    for (int i = 0; i < 3; i++)
    {
        s = &p->stream[i];
        if (s->is_working && rt_ringbuffer_data_len(&s->decoded_ring) < RESAMPLE_FRAME_SIZE)
        {
            can_mix = false;
            break;
        }
    }
    return can_mix;
}
static void mix_stream(mix_player_t *p)
{
    mix_stream_t *s;
    int16_t mix[3][RESAMPLE_FRAME_SIZE / 2];
    int16_t mixed[RESAMPLE_FRAME_SIZE / 2];

    memset(mix, 0, sizeof(mix));
    for (int i = 0; i < 3; i++)
    {
        s = &p->stream[i];
        if (!s->is_working)
        {
            continue;
        }
        if (s->samplerate == 48000)
        {
            rt_ringbuffer_get(&s->decoded_ring, (uint8_t *)&mix[i][0], RESAMPLE_FRAME_SIZE);
        }
        else
        {
            if (rt_ringbuffer_data_len(s->rb_resampled) < RESAMPLE_FRAME_SIZE)
            {
                uint32_t bytes, putted;
                uint8_t get[RESAMPLE_FRAME_SIZE];
                rt_ringbuffer_get(&s->decoded_ring, &get[0], RESAMPLE_FRAME_SIZE);
                RT_ASSERT(s->resample);
                bytes = sifli_resample_process(s->resample, (int16_t *)&get[0], RESAMPLE_FRAME_SIZE, 0);
                putted = rt_ringbuffer_put(s->rb_resampled, (const uint8_t *)s->resample->dst, bytes);
                RT_ASSERT(bytes == putted);
            }
            rt_ringbuffer_get(s->rb_resampled, (uint8_t *)&mix[i][0], RESAMPLE_FRAME_SIZE);
        }
    }
    for (int i = 0; i < RESAMPLE_FRAME_SIZE / 2; i++)
    {
        int t = (int)mix[0][i] + (int)mix[1][i] + (int)mix[2][i];
        mixed[i] = t / 3;
    }

    while (0 == audio_write(p->client, (uint8_t *)&mixed[0], RESAMPLE_FRAME_SIZE))
    {
        os_delay(5);
    }
}

static void thread_entry_file(void *parameter)
{
    mix_player_t *p = (mix_player_t *)parameter;
    {
        audio_parameter_t pa = {0};
        pa.write_bits_per_sample = 16;
        pa.write_channnel_num = 1;
        pa.read_bits_per_sample = 16;
        pa.read_channnel_num = 1;
        pa.write_samplerate = 48000;
        pa.read_samplerate = 16000;
        pa.read_cache_size = 0;
        pa.write_cache_size = 32000;
        p->client = audio_open(AUDIO_TYPE_BT_MUSIC, AUDIO_TX, &pa, audio_callback_func, p);
        RT_ASSERT(p->client);
    }

    rt_kprintf("wait start\n");

    while (1)
    {
        rt_uint32_t evt;
        rt_event_recv(p->event, MIX_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, OS_WAIT_FORVER, &evt);

        if (evt & MIX_EVENT_STOP)
        {
            rt_kprintf("event stop\n");
            process_msg(p, 1);
            break;
        }

        if (evt & MIX_EVENT_MSG)
        {
            process_msg(p, 0);
        }
        if (evt & MIX_EVENT_DECODE)
        {
            //rt_kprintf("msg decode\n");
            for (int i = 0; i < 3; i++)
            {
                decode_stream(&p->stream[i]);
            }
            while (is_can_mix(p))
            {
                mix_stream(p);
            }
        }
    }
    audio_close(p->client);
    p->client = NULL;
    rt_event_delete(p->event);
    p->event = NULL;
    audio_mem_free(p);
    rt_kprintf("thread exit\n");
}

#define THREAD_NAME_MIX3    "app_mix3"
PUBLIC_API mix_player_t *music_mix_app_start()
{
    MP3FrameInfo frameinfo;
    mix_player_t *p = audio_mem_calloc(1, sizeof(mix_player_t));
    RT_ASSERT(p);
    for (int i = 0; i < 3; i++)
    {
        mix_stream_t *s = &p->stream[i];
        s->fd = -1;
    }
    rt_slist_init(&p->cmd_slist);
    p->event = rt_event_create("mix", RT_IPC_FLAG_FIFO);
    RT_ASSERT(p->event);
    p->magic = 0;
    p->thread = rt_thread_create(THREAD_NAME_MIX3, thread_entry_file, p, MP3_ONE_MONO_FRAME_SIZE * sizeof(float) + 2048, RT_THREAD_PRIORITY_HIGH + 1, 10);
    RT_ASSERT(p->thread);
    rt_thread_startup(p->thread);
    return p;
}

PUBLIC_API void music_mix_app_stop(mix_player_t *p)
{
    rt_event_send(p->event, MIX_EVENT_STOP);
    while (rt_thread_find(THREAD_NAME_MIX3))
    {
        os_delay(1000);
        rt_kprintf("wait thread exit\n");
    }
}

PUBLIC_API void music_mix_app_add(mix_player_t *p, const char *filename)
{
    if (!p || p->magic)
    {
        return;
    }
    uint8_t *c;
    mix_cmd_t *cmd = audio_mem_malloc(sizeof(mix_cmd_t));
    RT_ASSERT(cmd);
    RT_ASSERT(strlen(filename) < 256);
    cmd->cmd_type = MIX_CMD_START;
    strcpy(cmd->file_name, filename);
    rt_enter_critical();
    rt_slist_append(&p->cmd_slist, &cmd->node);
    rt_exit_critical();
    rt_event_send(p->event, MIX_EVENT_MSG);
}

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

#define MKTAG(a,b,c,d) ((a) | ((b) << 8) | ((c) << 16) | ((unsigned)(d) << 24))


static inline void wave_seek(mix_stream_t *s, uint32_t offset)
{
    lseek(s->fd, offset, SEEK_SET);
}

static inline void wave_read(mix_stream_t *s, void *buffer, uint32_t size)
{
    read(s->fd, buffer, size);
}
static uint32_t wav_read_header(mix_stream_t *ctrl)
{
    uint32_t got_fmt = 0;
    uint32_t tag = 0;
    uint32_t size;
    uint32_t next_tag_ofs = 0;
    uint32_t data_size;
    wave_read(ctrl, &tag, 4);    //RIFF
    wave_read(ctrl, &size, 4);
    if (tag != MKTAG('R', 'I', 'F', 'F'))
        return -1;

    wave_read(ctrl, &tag, 4);

    if (tag != MKTAG('W', 'A', 'V', 'E'))
    {
        rt_kprintf("not WAVE\n");
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
            ctrl->is_wav_pcm = 1;
            wave_read(ctrl, &tag, 2); //format
            if (tag != 1)
            {
                ctrl->is_wav_pcm = 0;
            }
            tag = 0;
            wave_read(ctrl, &tag, 2); //channels
            ctrl->ch = (uint8_t)tag;
            wave_read(ctrl, &tag, 4); //samplerate
            ctrl->samplerate = tag;
            wave_read(ctrl, &tag, 4); //byte_rates
            //ctrl->wave_bytes_per_second = tag;
            tag = 0;
            wave_read(ctrl, &tag, 2);
            tag = 0;
            wave_read(ctrl, &tag, 2);
            {
                rt_kprintf("wave bits=%d\n", tag);
                ctrl->bits = tag;
            }

            break;
        case MKTAG('d', 'a', 't', 'a'):
            if (!got_fmt)
            {
                rt_kprintf("wave not fmt\n");
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

static uint32_t audio_parse_mp3_id3v2(mix_stream_t *s)
{
    uint32_t    tag_len;
    id3v2_t     id3;
    read(s->fd, (char *)&id3, 10);
    s->is_wave = 0;
    if (strncmp((const char *)&id3.header, "ID3", 3) != 0)
    {
        rt_kprintf("no ID3\n");
        tag_len = 0;
        if (!memcmp((const char *)&id3.header, "RIFF", 4))
        {
            s->is_wave = 1;
            lseek(s->fd, 0, SEEK_SET);
            tag_len = wav_read_header(s);
            rt_kprintf("wav len=%d\n", tag_len);
        }
    }
    else
    {
        tag_len = (((id3.size[0] & 0x7F) << 21) | ((id3.size[1] & 0x7F) << 14) |
                   ((id3.size[2] & 0x7F) << 7) | (id3.size[3] & 0x7F));

        rt_kprintf("ID3 len=0x%x\n", tag_len);
        tag_len += 10;
    }
    lseek(s->fd, tag_len, SEEK_SET);
    return tag_len;
}

void mix_main()
{
    mix_player_t *p = music_mix_app_start();
    RT_ASSERT(p);
    music_mix_app_add(p, "/1.mp3");
    music_mix_app_add(p, "/2.mp3");
    music_mix_app_add(p, "/3.wav");
    for (int i = 0; i < 50; i++)
    {
        rt_thread_mdelay(1000);
    }
    music_mix_app_stop(p);
    rt_kprintf("app exit\n");
}