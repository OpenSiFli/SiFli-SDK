/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#include "media_dec.h"
#include "media_internal.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif

#if PKG_USING_LIBHELIX
    #include "mp3dec.h"
#endif

#define DBG_TAG           "audio"
#define DBG_LVL           LOG_LVL_DBG
#include "log.h"

static inline void mono2stereo(int16_t *mono, uint32_t samples, int16_t *stereo)
{
    for (int i = 0; i < samples; i++)
    {
        *stereo++ = *mono;
        *stereo++ = *mono++;
    }
}

void ezip_audio_cache_init(ffmpeg_handle thiz)
{
    ezip_audio_cache_t *cache = &thiz->ezip_audio_cache;
    memset(cache, 0, sizeof(ezip_audio_cache_t));

    rt_slist_init(&cache->empty_audio_slist);
    rt_slist_init(&cache->readed_audio_slist);
    cache->main_ptr = &cache->main_buf[0];
    cache->main_left = 0;

    for (int i = 0; i < AUDIO_PACKETS_MEMORY_POOL; i++)
    {
        rt_slist_append(&cache->empty_audio_slist, &cache->cache[i].snode);
    }
}
void ezip_audio_cache_deinit(ffmpeg_handle thiz)
{
    ezip_audio_cache_t *cache = &thiz->ezip_audio_cache;
    for (int i = 0; i < AUDIO_PACKETS_MEMORY_POOL; i++)
    {
        if (cache->cache[i].buf)
        {
            thiz->cfg.mem_free(cache->cache[i].buf);
            cache->cache[i].buf = NULL;
        }
    }
#if PKG_USING_LIBHELIX
    if (cache->decode_handle)
    {
        MP3FreeDecoder((HMP3Decoder)cache->decode_handle);
        cache->decode_handle = NULL;
    }
#endif
    if (cache->decode_out)
    {
        thiz->cfg.mem_free(cache->decode_out);
        cache->decode_out = NULL;
    }
}

ezip_audio_packet_t *ezip_audio_read_packet(ffmpeg_handle thiz, uint32_t size, uint32_t paddings)
{
    ezip_audio_packet_t *p;
    rt_slist_t *empty;

#if 0
    for (int i = 0; i < AUDIO_PACKETS_MEMORY_POOL; i++)
    {
        LOG_I("audio read in cache %d=buf=%p, s=%d len=%d",
              i,
              thiz->ezip_audio_cache.cache[i].buf,
              thiz->ezip_audio_cache.cache[i].buf_size,
              thiz->ezip_audio_cache.cache[i].data_len);
    }
#endif
    while (1)
    {
        rt_enter_critical();
        empty = rt_slist_first(&thiz->ezip_audio_cache.empty_audio_slist);
        if (empty)
        {
            rt_slist_remove(&thiz->ezip_audio_cache.empty_audio_slist, empty);
        }
        rt_exit_critical();
        if (empty)
        {
            break;
        }
        rt_thread_mdelay(5);
    }



    p = rt_container_of(empty, ezip_audio_packet_t, snode);
    //LOG_I("empty size=%d size=%d paddings=%d", p->buf_size, size, paddings);
    if (p->buf_size < size)
    {
        if (p->buf)
        {
            thiz->cfg.mem_free(p->buf);
        }
        p->buf = thiz->cfg.mem_malloc(size);
        RT_ASSERT(p->buf);
        p->buf_size = size;
    }
    //LOG_I("empty size=%d size=%d paddings=%d", p->buf_size, size, paddings);
    int len;
    if (thiz->is_nand)
    {
        len = ezip_flash_read(thiz, p->buf, size);
    }
    if (thiz->is_ram)
    {
        len = ezip_ram_read(thiz, p->buf, size);
    }
    else
    {
        len = read(thiz->ezip_fd, p->buf, size);
    }
    p->data_len = size - paddings;
    //LOG_I("audio slist=%p, readlen=%d buf=%p, len=%d\n", empty, len, p->buf, p->data_len);
    //LOG_HEX("audio pkt", 16, p->buf, p->data_len);
    rt_enter_critical();
    rt_slist_append(&thiz->ezip_audio_cache.readed_audio_slist, empty);
    rt_exit_critical();
#if 0
    for (int i = 0; i < AUDIO_PACKETS_MEMORY_POOL; i++)
    {
        LOG_I("audio read in cache %d=buf=%p, s=%d len=%d",
              i,
              thiz->ezip_audio_cache.cache[i].buf,
              thiz->ezip_audio_cache.cache[i].buf_size,
              thiz->ezip_audio_cache.cache[i].data_len);
    }
#endif
    return p;
}

void ezip_audio_decode(ffmpeg_handle thiz, audio_server_callback_func callback, void *mp3parser)
{
    AVCodecParserContext *parser = (AVCodecParserContext *)mp3parser;
    ezip_audio_cache_t *cache = &thiz->ezip_audio_cache;
#if EZIP_DECODE_AUDIO_USING_FFMPEG
    AVPacket packet;
#elif PKG_USING_LIBHELIX
    UNUSED(parser);
    HMP3Decoder hMP3Decoder = (HMP3Decoder)cache->decode_handle;
    if (!hMP3Decoder)
    {
        HMP3Decoder hMP3Decoder = MP3InitDecoder();
        cache->decode_handle = hMP3Decoder;
        if (!hMP3Decoder)
        {
            return;
        }
        cache->decode_out = thiz->cfg.mem_malloc(sizeof(short) * MAX_NCHAN * MAX_NGRAN * MAX_NSAMP);
        RT_ASSERT(cache->decode_out);
        LOG_I("mp3 decoder=%p", hMP3Decoder);
    }
#endif

#if 0
    for (int i = 0; i < AUDIO_PACKETS_MEMORY_POOL; i++)
    {
        LOG_I("decode audio cache %d=buf=%p, s=%d len=%d",
              i,
              thiz->ezip_audio_cache.cache[i].buf,
              thiz->ezip_audio_cache.cache[i].buf_size,
              thiz->ezip_audio_cache.cache[i].data_len);
    }
#endif
    rt_slist_t *readed;
    rt_enter_critical();
    readed = rt_slist_first(&cache->readed_audio_slist);
    rt_slist_remove(&cache->readed_audio_slist, readed);
    rt_exit_critical();
    if (!readed)
    {
        LOG_I("%s no readed", __FUNCTION__);
        return;
    }
    ezip_audio_packet_t *p = rt_container_of(readed, ezip_audio_packet_t, snode);
    if (!p->data_len)
    {
        LOG_E("audio error pkt");
    }

    //LOG_I("got slist=%p, buf=%p size=%d len=%d", readed, p->buf, p->buf_size, p->data_len);
    //LOG_HEX("mp3 data:\n", 16, p->buf, p->data_len);
#if EZIP_DECODE_AUDIO_USING_FFMPEG
    uint8_t *data = NULL;
    uint8_t *pp = p->buf;
    uint32_t size = 0;
    int bytes_used;

    while (p->data_len > 0)
    {
        bytes_used = av_parser_parse2(parser, thiz->audio_dec_ctx, &data, &size, pp, p->data_len, 0, 0, AV_NOPTS_VALUE);
        p->data_len -= bytes_used;
        pp += bytes_used;
        if (size == 0)
        {
            continue;
        }
        // We have data of one packet, decode it; or decode whatever when ending
        av_init_packet(&packet);
        packet.data = data;
        packet.size = size;
        int got_frame = 0;
        int ret = avcodec_decode_audio4(thiz->audio_dec_ctx, thiz->audio_frame, &got_frame, &packet);
        if (ret < 0)
        {
            LOG_I("Decode frame error\n");
            continue;
        }
        if (got_frame)
        {
            if (thiz->audio_data == NULL)
            {
                thiz->audio_data_size = (thiz->audio_frame->nb_samples * thiz->audio_frame->channels * sizeof(uint16_t));

                if (thiz->audio_data_size < 1152 * 4)
                {
                    thiz->audio_data_size = 1152 * 4;
                }

                thiz->audio_data = (uint16_t *)thiz->cfg.mem_malloc(thiz->audio_data_size);
                RT_ASSERT(thiz->audio_data != NULL);
            }
            if (thiz->audio_handle == NULL && !thiz->is_wait_for_resume)
            {
                audio_parameter_t arg = {0};
                arg.write_bits_per_sample = 16;
                arg.write_samplerate = thiz->audio_samplerate;
                arg.write_channnel_num = thiz->audio_channel < 2 ? 1 : 2;
                arg.write_cache_size = EZIP_AUDIO_CACHE_SIZE;
                thiz->audio_data_period = thiz->audio_data_size / (thiz->audio_samplerate * arg.write_channnel_num * (arg.write_bits_per_sample >> 3) / 1000);

                LOG_I("audio_frame_size=%d, sr=%d", thiz->audio_data_size, thiz->audio_samplerate);
                LOG_I("audio_data_period=%d", thiz->audio_data_period);
                thiz->audio_handle = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &arg, callback, thiz);
                RT_ASSERT(thiz->audio_handle);
            }

            uint32_t new_size = thiz->audio_frame->nb_samples * thiz->audio_frame->channels * sizeof(uint16_t);
            thiz->audio_data_period = new_size / (thiz->audio_samplerate * thiz->audio_frame->channels * 2 / 1000);

            uint8_t data_size_changed = 0;
            if (new_size > thiz->audio_data_size)
            {
                data_size_changed = 1;
                thiz->cfg.mem_free(thiz->audio_data);
                thiz->audio_data_size = new_size;
                thiz->audio_data = (uint16_t *)thiz->cfg.mem_malloc(new_size);
                RT_ASSERT(thiz->audio_data != NULL);
            }

            media_audio_get(thiz->audio_frame,  thiz->audio_data, thiz->audio_data_size);
            av_frame_unref(thiz->audio_frame);

            uint32_t write_bytes = new_size;
            uint8_t *write_ptr = thiz->audio_data;
#if !TWS_MIX_ENABLE
            if (audio_device_is_a2dp_sink())
            {
                if (data_size_changed || !thiz->audio_stereo)
                {
                    if (thiz->audio_stereo)
                    {
                        thiz->cfg.mem_free(thiz->audio_stereo);
                        thiz->audio_stereo = NULL;
                    }
                    if (thiz->audio_channel != 2)
                    {
                        thiz->audio_stereo = (uint16_t *)thiz->cfg.mem_malloc(thiz->audio_data_size * 2);
                        RT_ASSERT(thiz->audio_stereo);
                    }
                }
                if (thiz->audio_samplerate != 44100)
                {
                    if (!thiz->resample)
                    {
                        thiz->resample = sifli_resample_open(2, thiz->audio_samplerate, 44100);
                        RT_ASSERT(thiz->resample);

                    }
                }
                if (thiz->audio_channel == 2)
                {
                    if (thiz->audio_samplerate != 44100)
                    {
                        write_bytes = sifli_resample_process(thiz->resample, (int16_t *)thiz->audio_data, new_size, 0);
                        write_ptr = (uint8_t *)sifli_resample_get_output(thiz->resample);
                    }
                }
                else
                {
                    mono2stereo((int16_t *)thiz->audio_data, new_size / 2, thiz->audio_stereo);
                    if (thiz->audio_samplerate != 44100)
                    {
                        write_bytes = sifli_resample_process(thiz->resample, (int16_t *)thiz->audio_stereo, new_size * 2, 0);
                        write_ptr = (uint8_t *)sifli_resample_get_output(thiz->resample);
                    }
                    else
                    {
                        write_bytes = new_size * 2;
                        write_ptr = thiz->audio_stereo;
                    }
                }
            }
#endif
            while (!thiz->is_wait_for_resume && 0 == audio_write(thiz->audio_handle, write_ptr, write_bytes))
            {
                uint32_t    evt = 0;
                uint32_t    wait_ticks = rt_tick_from_millisecond(thiz->audio_data_period);
                os_event_flags_wait(thiz->evt_audio, 1, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, wait_ticks, &evt);
                if (thiz->is_ok == 0)
                {
                    LOG_I("ezip exit@audio");
                    break;
                }
            }
        }
    }
    rt_enter_critical();
    rt_slist_append(&cache->empty_audio_slist, readed);
    p->data_len = 0;
    rt_exit_critical();

    return;

#elif PKG_USING_LIBHELIX
    RT_ASSERT(cache->main_left < 2 * MAINBUF_SIZE);
    RT_ASSERT(cache->main_left + p->data_len < MP3_MAIN_BUFFER_SIZE);
    RT_ASSERT(cache->main_ptr == &cache->main_buf[0]);
    memcpy(cache->main_ptr + cache->main_left, p->buf, p->data_len);
    cache->main_left += p->data_len;
    do
    {
        if (cache->main_left < MAINBUF_SIZE)
        {
            if (cache->main_ptr != &cache->main_buf[0])
            {
                memcpy(&cache->main_buf[0], cache->main_ptr, cache->main_left);
                cache->main_ptr = &cache->main_buf[0];
            }
            break;
        }

        int offset = MP3FindSyncWord(cache->main_ptr, cache->main_left);
        if (offset >= 0)
        {
            cache->main_ptr += offset;
            cache->main_left -= offset;
            //LOG_I("offset=%d", offset);
            int err = MP3Decode(hMP3Decoder, &cache->main_ptr, &cache->main_left, (short *)cache->decode_out, 0, 0);
            if (err)
            {
                LOG_I("mp3 decode err=%d", err);
                continue;
            }

            if (thiz->audio_handle == NULL)
            {
                MP3FrameInfo mp3FrameInfo;
                MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
                thiz->audio_data_size = mp3FrameInfo.outputSamps * sizeof(uint16_t);
                thiz->audio_samplerate = mp3FrameInfo.samprate;
                thiz->audio_channel = mp3FrameInfo.nChans;
                audio_parameter_t arg = {0};
                arg.write_bits_per_sample = 16;
                arg.write_samplerate = thiz->audio_samplerate;
                arg.write_channnel_num = thiz->audio_channel < 2 ? 1 : 2;
                arg.write_cache_size = AUDIO_CACHE_SIZE;
                thiz->audio_data_period = thiz->audio_data_size * 1000 / (thiz->audio_samplerate * arg.write_channnel_num * 2);
                LOG_I("audio_frame_size=%d, sr=%d", thiz->audio_data_size, thiz->audio_samplerate);
                LOG_I("audio_data_period=%d", thiz->audio_data_period);
                thiz->audio_handle = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &arg, callback, thiz);
                RT_ASSERT(thiz->audio_handle);
            }
            TRACE_MARK_START(TRACEID_AUDIO_WRITE);

            uint32_t write_bytes = thiz->audio_data_size;
            uint8_t *write_ptr = (int16_t *)cache->decode_out;
#if !TWS_MIX_ENABLE
            if (audio_device_is_a2dp_sink())
            {
                if (!thiz->audio_stereo)
                {
                    if (thiz->audio_stereo)
                    {
                        thiz->cfg.mem_free(thiz->audio_stereo);
                        thiz->audio_stereo = NULL;
                    }
                    if (thiz->audio_channel != 2)
                    {
                        thiz->audio_stereo = (uint16_t *)thiz->cfg.mem_malloc(thiz->audio_data_size * 2);
                        RT_ASSERT(thiz->audio_stereo);
                    }
                }
                if (thiz->audio_samplerate != 44100 && !thiz->resample)
                {
                    thiz->resample = sifli_resample_open(2, thiz->audio_samplerate, 44100);
                    RT_ASSERT(thiz->resample);
                }

                if (thiz->audio_channel == 2 && thiz->audio_samplerate != 44100)
                {
                    write_bytes = sifli_resample_process(thiz->resample, (int16_t *)cache->decode_out, thiz->audio_data_size, 0);
                    write_ptr = (uint8_t *)sifli_resample_get_output(thiz->resample);
                }
                else
                {
                    mono2stereo((int16_t *)thiz->audio_data, thiz->audio_data_size / 2, thiz->audio_stereo);
                    write_bytes = sifli_resample_process(thiz->resample, (int16_t *)thiz->audio_stereo, thiz->audio_data_size * 2, 0);
                    write_ptr = (uint8_t *)sifli_resample_get_output(thiz->resample);
                }
            }
#endif
            while (!thiz->is_wait_for_resume && 0 == audio_write(thiz->audio_handle, write_ptr, write_bytes))
            {
                uint32_t    evt = 0;
                uint32_t    wait_ticks = rt_tick_from_millisecond(thiz->audio_data_period);
                os_event_flags_wait(thiz->evt_audio, 1, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, wait_ticks, &evt);
                if (thiz->is_ok == 0)
                {
                    LOG_I("ezip_audio_decode exit2");
                    break;
                }
            }

            TRACE_MARK_STOP(TRACEID_AUDIO_WRITE);
        }
        else
        {
            ASSERT(0);
        }
    }
    while (thiz->is_ok);
    rt_enter_critical();
    rt_slist_append(&cache->empty_audio_slist, readed);
    p->data_len = 0;
    rt_exit_critical();
#endif
}

