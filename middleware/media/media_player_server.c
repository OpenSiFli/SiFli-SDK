#include <rtthread.h>
#include "media_dec.h"
#include "media_internal.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif

#define DBG_TAG           "h264"
#define DBG_LVL           LOG_LVL_INFO
#define _MODULE_NAME_ "h264"
#include "log.h"

#define MEDIAPLAYER_USING_ACPU  0
#define DEBUG_FFMPEG_FPS        1
#define USING_FAST_YUV2RGB      1

#define NETWORK_READ_STACK_SIZE (13 * 1024)
#define FFMPEG_OPEN_AYNC        1
#define refcount                1

#define SIFLI_MEDIA_MAGIC       "siflizip"

#undef lock
#undef unlock
#define lock()      _lock(thiz->network_queue)
#define unlock()    _unlock(thiz->network_queue)

/*
    ffmpeg_thread_stack in sram for fast decode speed, see link sct file
*/
uint32_t ffmpeg_thread_stack[4 * 1024]; //video & audio stack
#define ffmpeg_audio_dec_thread_stack_size  (sizeof(ffmpeg_thread_stack)/2)
#define ffmpeg_video_dec_thread_stack_size  (sizeof(ffmpeg_thread_stack)/2)
#define ffmpeg_audio_dec_thread_stack   &ffmpeg_thread_stack[0]
#define ffmpeg_video_dec_thread_stack   &ffmpeg_thread_stack[ffmpeg_audio_dec_thread_stack_size/4]

#define EVT_INIT_OK         (1<<0)
#define EVT_INIT_FAILED     (1<<1)

static ffmpeg_handle g_player = NULL;

static void drop_all_avpacket(os_message_queue_t q);
static void clean_up(ffmpeg_handle thiz);
RT_WEAK uint32_t lv_img_decode_flash_read(uint32_t addr, uint8_t *buf, int size)
{
    RT_ASSERT(0);
    return 0;
}

int ezip_flash_read(ffmpeg_handle thiz, void *buf, int len)
{
    if ((uint32_t)(thiz->ezip_fd + len) >= thiz->src_in_nand_len)
    {
        len = thiz->src_in_nand_len - (uint32_t)thiz->ezip_fd;
    }
    if (len == 0)
        return 0;

    lv_img_decode_flash_read((uint32_t)(thiz->src_in_nand_address + thiz->ezip_fd), buf, len);
    thiz->ezip_fd = thiz->ezip_fd + (int)len;
    return len;
}
static int ezip_flash_seek(ffmpeg_handle thiz, int size, int where)
{
    if (where == SEEK_SET)
    {
        if (size < 0)
            size = 0;
        if (size > (int)thiz->src_in_nand_len)
            size = (int)thiz->src_in_nand_len;
        thiz->ezip_fd = size;
    }
    else if (where == SEEK_CUR)
    {
        thiz->ezip_fd += size;
    }
    else if (where == SEEK_END)
    {
        thiz->ezip_fd = (int)thiz->src_in_nand_len + size;
    }

    if (thiz->ezip_fd < 0)
        thiz->ezip_fd = 0;

    if (thiz->ezip_fd > (int)thiz->src_in_nand_len)
        thiz->ezip_fd = (int)thiz->src_in_nand_len;

    return thiz->ezip_fd;
}

static int mediaplayer_start(ffmpeg_handle thiz, bool is_file);

static int open_codec_context(int *stream_idx,
                              AVFormatContext *fmt_ctx, enum AVMediaType type)
{
    int ret, stream_index;
    AVStream *st;
    AVCodecContext *dec_ctx = NULL;
    AVCodec *dec = NULL;
    AVDictionary *opts = NULL;

    ret = av_find_best_stream(fmt_ctx, type, -1, -1, NULL, 0);
    if (ret < 0)
    {
        LOG_E("Could not find %s stream", av_get_media_type_string(type));
        return ret;
    }
    else
    {
        stream_index = ret;
        st = fmt_ctx->streams[stream_index];

        /* find decoder for the stream */
        dec_ctx = st->codec;
        dec = avcodec_find_decoder(dec_ctx->codec_id);
        if (!dec)
        {
            LOG_E("Failed to find %s codec\n",
                  av_get_media_type_string(type));
            return AVERROR(EINVAL);
        }

        /* Init the decoders, with or without reference counting */
        av_dict_set(&opts, "refcounted_frames", refcount ? "1" : "0", 0);
        if ((ret = avcodec_open2(dec_ctx, dec, &opts)) < 0)
        {
            LOG_E("Failed to open %s codec\n",
                  av_get_media_type_string(type));
            return ret;
        }
        *stream_idx = stream_index;
    }

    return 0;
}

/*
  1. for local file or network mp4 stream, decoding speed is same as
     video period if no audio stream  or audio stream is disabled by user.
  2. for demuxed packet stream, decoding speed is sames as network packet speed
 */
static void video_period_handle(void *p)
{
    ffmpeg_handle thiz = p;
    if (thiz->evt_video_decode)
    {
        os_event_flags_set(thiz->evt_video_decode, 1);
    }
}
static void video_decode_thread(void *p)
{
    int got_frame = 0;
    AVPacket                pkt;
    ffmpeg_handle thiz = p;
    rt_time_t  timer = RT_NULL;
    LOG_I("video decode task run\n");
    /*readed frame must decode it, or memory leak. so look thiz->is_ok first*/
    while (thiz->is_ok)
    {
        if (!thiz->audio_dec_ctx || !thiz->cfg.audio_enable)
        {
            if (!media_video_need_decode(&thiz->video_cache))
            {
                os_delay(5);
                continue;
            }
        }

        os_message_get(thiz->av_pkt_queue, &pkt, sizeof(pkt), OS_WAIT_FORVER);

        AVPacket orig_pkt = pkt;

        do
        {
            int ret;
            if (!thiz->audio_dec_ctx || !thiz->cfg.audio_enable)
            {
                if (!media_video_need_decode(&thiz->video_cache))
                {
                    os_delay(5);
                    continue;
                }
            }
            TRACE_MARK_START(TRACEID_VIDEO_DECODE_TOTAL);
            got_frame = 0;
            ret = media_decode_video(thiz,
                                     &got_frame,
                                     &pkt);
            TRACE_MARK_STOP(TRACEID_VIDEO_DECODE_TOTAL);

            if (ret < 0)
            {
                break;
            }
            pkt.data += ret;
            pkt.size -= ret;
        }
        while (pkt.size > 0);
        av_packet_unref(&orig_pkt);
    }

    /* flush cached frames, TODO: Still need this??? */
    pkt.data = NULL;
    pkt.size = 0;
    do
    {
        TRACE_MARK_START(TRACEID_VIDEO_DECODE_TOTAL);
        got_frame = 0;
        media_decode_video(thiz,
                           &got_frame,
                           &pkt);

        TRACE_MARK_STOP(TRACEID_VIDEO_DECODE_TOTAL);
    }
    while (got_frame);

    rt_thread_mdelay(20);
    LOG_I("video decode_thread exit");
}

static int audio_callback_func(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    ffmpeg_handle thiz = callback_userdata;
    RT_ASSERT(thiz->magic == FFMPEG_HANDLE_MAGIC);
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (thiz->evt_audio)
            os_event_flags_set(thiz->evt_audio, 1);
    }
    else if (cmd == as_callback_cmd_suspended)
    {
        thiz->is_suspended = 1;
        if (thiz->cfg.notify && !thiz->is_closing)
            thiz->cfg.notify(thiz->user_data, e_ffmpeg_suspended, 0);
    }
    else if (cmd == as_callback_cmd_resumed)
    {
        thiz->is_suspended = 0;
        os_event_flags_set(thiz->evt_pause, 1);

        if (thiz->cfg.notify && !thiz->is_closing)
            thiz->cfg.notify(thiz->user_data, e_ffmpeg_resumed, 0);
    }

    return 0;
}

static void decode_audio_packet(ffmpeg_handle thiz, AVPacket *orig, AVPacket *cur_pkt)
{
    int got_frame;
    do
    {
        int ret;
        TRACE_MARK_START(TRACEID_AUDIO_DECODE_TOTAL);
        got_frame = 0;
        TRACE_MARK_START(TRACEID_AUDIO_DECODE);
        ret = avcodec_decode_audio4(thiz->audio_dec_ctx, thiz->audio_frame, &got_frame, cur_pkt);
        TRACE_MARK_STOP(TRACEID_AUDIO_DECODE);
        if (ret < 0)
        {
            LOG_E("audio decode_thread exiting");
            break;
        }
        if (got_frame)
        {
            if (thiz->audio_data == NULL)
            {
                thiz->audio_data_size = (thiz->audio_frame->nb_samples * thiz->audio_frame->channels * sizeof(uint16_t));
                thiz->audio_data = (uint16_t *)thiz->cfg.mem_malloc(thiz->audio_data_size);
                RT_ASSERT(thiz->audio_data != NULL);
            }
            if (thiz->audio_handle == NULL)
            {
                audio_parameter_t arg = {0};
                arg.write_bits_per_sample = 16;
                arg.write_samplerate = thiz->audio_samplerate;
                arg.write_channnel_num = thiz->audio_channel < 2 ? 1 : 2;
                arg.write_cache_size = AUDIO_CACHE_SIZE;
                thiz->audio_data_period = thiz->audio_data_size / (thiz->audio_samplerate * thiz->audio_frame->channels * (arg.write_bits_per_sample >> 3) / 1000);
                LOG_I("audio_frame_size=%d, sr=%d", thiz->audio_data_size, thiz->audio_samplerate);
                LOG_I("audio_data_period=%d", thiz->audio_data_period);
                thiz->audio_handle = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &arg, audio_callback_func, thiz);
                RT_ASSERT(thiz->audio_handle);
            }

            TRACE_MARK_START(TRACEID_AUDIO_CONVERT);
            media_audio_get(thiz->audio_frame,  thiz->audio_data);
            TRACE_MARK_STOP(TRACEID_AUDIO_CONVERT);

            if (refcount)
                av_frame_unref(thiz->audio_frame);

            TRACE_MARK_START(TRACEID_AUDIO_WRITE);
            while (0 == audio_write(thiz->audio_handle, (uint8_t *)thiz->audio_data, thiz->audio_data_size))
            {
                uint32_t    evt = 0;
                uint32_t    wait_ticks = rt_tick_from_millisecond(thiz->audio_data_period);
                os_event_flags_wait(thiz->evt_audio, 1, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, wait_ticks, &evt);
                if (thiz->is_ok == 0)
                {
                    LOG_I("media_decode_packet exit @audio");
                    break;
                }
            }
            TRACE_MARK_STOP(TRACEID_AUDIO_WRITE);

        }
        TRACE_MARK_STOP(TRACEID_AUDIO_DECODE_TOTAL);
        cur_pkt->data += ret;
        cur_pkt->size -= ret;
    }
    while (cur_pkt->size > 0);
    av_packet_unref(orig);
}

static void audio_decode_thread(void *p)
{
    int got_frame;
    AVPacket                pkt;
    ffmpeg_handle thiz = p;
    LOG_I("audio decode task run\n");
    /*readed frame must decode it, or memory leak. so look thiz->is_ok first*/
    while (thiz->is_ok)
    {
        //can't call audio_close() if thiz->is_suspended. otherwie audio can't be resumed
        if (thiz->is_paused || !thiz->cfg.audio_enable)
        {
            //may has no pkt coming, close audio device to save power
            if (thiz->audio_handle)
            {
                audio_close(thiz->audio_handle);
                thiz->audio_handle = NULL;
            }
        }
        os_message_get(thiz->av_pkt_queue_audio, &pkt, sizeof(pkt), OS_WAIT_FORVER);

        AVPacket orig_pkt = pkt;

        if (thiz->is_paused || thiz->is_suspended)
        {
            //@pause status, read thread will not send pkt to this thread
            av_packet_unref(&orig_pkt);
            continue;
        }

        decode_audio_packet(thiz, &orig_pkt, &pkt);

    }

    /* flush cached frames */
    pkt.data = NULL;
    pkt.size = 0;
    do
    {
        TRACE_MARK_START(TRACEID_AUDIO_DECODE_TOTAL);
        got_frame = 0;
        avcodec_decode_audio4(thiz->audio_dec_ctx, thiz->audio_frame, &got_frame, &pkt);

        if (got_frame && refcount)
            av_frame_unref(thiz->audio_frame);

        TRACE_MARK_STOP(TRACEID_AUDIO_DECODE_TOTAL);
    }
    while (got_frame);
    if (thiz->audio_handle)
    {
        audio_close(thiz->audio_handle);
        thiz->audio_handle = NULL;
    }
    rt_thread_mdelay(20);
    LOG_I("audio decode_thread exit");
}

static void video_audio_decode_thread(void *p)
{
    int got_frame = 0;
    AVPacket pkt;
    ffmpeg_handle thiz = p;
    rt_time_t  timer = RT_NULL;
    LOG_I("video decode task run\n");
    /*readed frame must decode it, or memory leak. so look thiz->is_ok first*/
    while (thiz->is_ok)
    {
        //can't call audio_close() if thiz->is_suspended. otherwie audio can't be resumed
        if (thiz->is_paused || !thiz->cfg.audio_enable)
        {
            //may has no pkt coming, close audio device to save power
            if (thiz->audio_handle)
            {
                audio_close(thiz->audio_handle);
                thiz->audio_handle = NULL;
            }
        }

        os_message_get(thiz->av_pkt_queue, &pkt, sizeof(pkt), OS_WAIT_FORVER);

        AVPacket orig_pkt = pkt;

        if (thiz->is_paused || thiz->is_suspended)
        {
            //@pause status, read thread will not send pkt to this thread
            av_packet_unref(&orig_pkt);
            continue;
        }

        if (pkt.stream_index == thiz->video_stream_idx)
        {
            do
            {
                int ret;

                TRACE_MARK_START(TRACEID_VIDEO_DECODE_TOTAL);
                got_frame = 0;
                ret = media_decode_video(thiz,
                                         &got_frame,
                                         &pkt);
                TRACE_MARK_STOP(TRACEID_VIDEO_DECODE_TOTAL);

                if (ret < 0)
                {
                    break;
                }
                pkt.data += ret;
                pkt.size -= ret;
            }
            while (pkt.size > 0);
            av_packet_unref(&orig_pkt);
        }
        else if (pkt.stream_index == thiz->audio_stream_idx && thiz && thiz->cfg.audio_enable)
        {
            decode_audio_packet(thiz, &orig_pkt, &pkt);
        }
        else
        {
            av_packet_unref(&orig_pkt);
        }
    }

    /* flush cached frames, TODO: Still need this??? */
    pkt.data = NULL;
    pkt.size = 0;
    do
    {
        TRACE_MARK_START(TRACEID_VIDEO_DECODE_TOTAL);
        got_frame = 0;
        media_decode_video(thiz,
                           &got_frame,
                           &pkt);

        TRACE_MARK_STOP(TRACEID_VIDEO_DECODE_TOTAL);
    }
    while (got_frame);

    pkt.data = NULL;
    pkt.size = 0;
    do
    {
        TRACE_MARK_START(TRACEID_AUDIO_DECODE_TOTAL);
        got_frame = 0;
        avcodec_decode_audio4(thiz->audio_dec_ctx, thiz->audio_frame, &got_frame, &pkt);

        if (got_frame && refcount)
            av_frame_unref(thiz->audio_frame);

        TRACE_MARK_STOP(TRACEID_AUDIO_DECODE_TOTAL);
    }
    while (got_frame);
    if (thiz->audio_handle)
    {
        audio_close(thiz->audio_handle);
        thiz->audio_handle = NULL;
    }
    rt_thread_mdelay(20);
    LOG_I("%s exit", __FUNCTION__);
}

//#define DEBUG_DOWNLOAD_SPEED
//#define DEBUG_IO_SPEED
static void media_read_thread(void *p)
{
#ifdef DEBUG_DOWNLOAD_SPEED
    uint64_t debug_download_size = 0;
    rt_tick_t  tick_start = 0, tick_cur;
#endif
#ifdef DEBUG_IO_SPEED
    rt_tick_t  io_start, io_ticks = 0;
    uint32_t io_read_size = 0;
#endif /*DEBUG_IO_SPEED*/
    ffmpeg_handle thiz = p;
    LOG_I("%s", __FUNCTION__);

#if FFMPEG_OPEN_AYNC
    int ret;
    if (thiz->cfg.src == e_src_localfile)
    {
        LOG_E("mediaplayer_start %s fmt=%d", thiz->cfg.file_path, thiz->cfg.fmt);
        ffmeg_mem_init();
        ret = mediaplayer_start(thiz, 1);
    }
    else if (thiz->cfg.src == e_network_frames_stream)
    {
        ret = mediaplayer_start(thiz, 0);
    }
    else
    {
        ret = -1;
        LOG_E("under development");
    }
    if (ret != 0)
    {
        LOG_E("media start error");
        os_event_flags_set(thiz->evt_init, EVT_INIT_FAILED);
        return;
    }
    os_event_flags_set(thiz->evt_init, EVT_INIT_OK);
#endif

    while (thiz->is_ok)
    {
        if (thiz->is_paused || thiz->is_suspended)
        {
            LOG_I("read paused=%d suspend=%d", thiz->is_paused, thiz->is_suspended);
            os_event_flags_wait(thiz->evt_pause, 1, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, OS_WAIT_FORVER, NULL);
            LOG_I("read resumed");
            continue; //may exit after resume
        }
        AVPacket  pkt;
        av_init_packet(&pkt);
        pkt.data = NULL;
        pkt.size = 0;

        if (thiz->seeking_state == 1)
        {
            LOG_I("seek to %d", thiz->seek_to_second);
            av_seek_frame(thiz->fmt_ctx, 0, 0, AVSEEK_FLAG_BACKWARD);
            thiz->frame_index = 0;
            while (thiz->seeking_state == 1)
            {
                if (av_read_frame(thiz->fmt_ctx, &pkt) < 0)
                {
                    thiz->seeking_state = 0;
                    break;
                }
                if (pkt.stream_index == thiz->video_stream_idx)
                {
                    thiz->frame_index++;
                    if (thiz->frame_index * thiz->period_float / 1000 >= thiz->seek_to_second)
                    {
                        thiz->seeking_state = 2;
                    }
                }
                AVPacket orig_pkt = pkt;
                av_packet_unref(&orig_pkt);
            }
            continue;
        }

#ifdef DEBUG_DOWNLOAD_SPEED
        if (!tick_start)
        {
            tick_start = rt_tick_get_millisecond();
        }
#endif
#ifdef DEBUG_IO_SPEED
        io_start = rt_tick_get_millisecond();
#endif /*DEBUG_IO_SPEED*/
        TRACE_MARK_START(TRACEID_AV_PACKET);
        if (av_read_frame(thiz->fmt_ctx, &pkt) < 0)
        {

            TRACE_MARK_STOP(TRACEID_AV_PACKET);
            if (thiz->cfg.notify && !thiz->is_closing)
                thiz->cfg.notify(thiz->user_data, e_ffmpeg_play_to_end, 0);

            if (thiz->cfg.is_loop)
            {
                pkt.data = NULL;
                pkt.size = 0;
                av_seek_frame(thiz->fmt_ctx, 0, 0, AVSEEK_FLAG_BACKWARD);
                thiz->frame_index = 0;
                thiz->last_seconds = -1;
                continue;
            }
            else
                break;

        }
        TRACE_MARK_STOP(TRACEID_AV_PACKET);

#ifdef DEBUG_DOWNLOAD_SPEED
        debug_download_size += pkt.size;
        tick_cur = rt_tick_get_millisecond();
        if (tick_cur - tick_start >= 2000)
        {
            LOG_I("down %d B/S", debug_download_size * 1000 / (tick_cur - tick_start));
            tick_start = tick_cur;
        }
#endif
#ifdef DEBUG_IO_SPEED
        io_read_size += pkt.size;
        io_ticks += rt_tick_get_millisecond() - io_start;
        if (io_ticks > 1000)
        {
            LOG_I("IO1 Speed %d kB/S", io_read_size / io_ticks);
            io_ticks = 0;
            io_read_size = 0;
        }
#endif /*DEBUG_IO_SPEED*/

        os_message_queue_t q;
        uint32_t   trace_id;
        if (thiz->is_network_file)
        {
            q = thiz->av_pkt_queue;
            trace_id = TRACEID_VIDEO_PACKET;
        }
        else
        {
            if (pkt.stream_index == thiz->video_stream_idx)
            {
                q = thiz->av_pkt_queue;
                trace_id = TRACEID_VIDEO_PACKET;

                if (thiz->cfg.notify && !thiz->is_closing)
                {
                    uint32_t seconds = thiz->frame_index * thiz->period_float / 1000;
                    if (seconds != thiz->last_seconds)
                    {
                        thiz->last_seconds = seconds;
                        thiz->cfg.notify(thiz->user_data, e_ffmpeg_progress, seconds);
                    }
                }
                thiz->frame_index++;
            }
            else if (pkt.stream_index == thiz->audio_stream_idx)
            {
                q = thiz->av_pkt_queue_audio;
                trace_id = TRACEID_AUDIO_PACKET;
            }
            else
                RT_ASSERT(0);
        }

        TRACE_MARK_START(trace_id);
        while (thiz->is_ok && 0 != os_message_put(q, &pkt, sizeof(pkt), 0))
        {
            if (thiz->is_network_file)
                os_delay(10);
            else
                os_delay(1);
#ifdef DEBUG_DOWNLOAD_SPEED
            tick_start = rt_tick_get_millisecond();
#endif
            if (thiz->is_paused || thiz->is_suspended)
            {
                LOG_I("put paused=%d suspend=%d", thiz->is_paused, thiz->is_suspended);
                os_event_flags_wait(thiz->evt_pause, 1, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, OS_WAIT_FORVER, NULL);
                LOG_I("put resumed");
                continue; //may exit after resume
            }
        }
        TRACE_MARK_STOP(trace_id);

    }

    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = NULL;
    pkt.size = 0;

    while (rt_thread_find("aud_dec"))
    {
        os_message_put(thiz->av_pkt_queue_audio, &pkt, sizeof(pkt), 0);
        if (thiz->evt_audio)   os_event_flags_set(thiz->evt_audio, 1);
        rt_thread_mdelay(10);
        LOG_I("wait audio dec thread exit");
    }

    while (rt_thread_find("vid_dec"))
    {
        os_message_put(thiz->av_pkt_queue, &pkt, sizeof(pkt), 0);
        if (thiz->evt_video)  os_event_flags_set(thiz->evt_video, 1);
        rt_thread_mdelay(10);
        LOG_I("wait video dec thread exit");
    }

    if (thiz->av_pkt_queue)
    {
        drop_all_avpacket(thiz->av_pkt_queue);
    }

    if (thiz->av_pkt_queue_audio)
    {
        drop_all_avpacket(thiz->av_pkt_queue_audio);
    }

    clean_up(thiz);

    LOG_I("media exit");
}

static void drop_all_avpacket(os_message_queue_t q)
{
    AVPacket                pkt;
    while (RT_EOK == os_message_get(q, &pkt, sizeof(pkt), 0))
        av_packet_unref(&pkt);

}

static void clean_up(ffmpeg_handle thiz)
{
    if (thiz->evt_video)
    {
        os_event_delete(thiz->evt_video);
        thiz->evt_video = NULL;
    }

    if (thiz->evt_video_decode)
    {
        os_event_delete(thiz->evt_video_decode);
        thiz->evt_video_decode = NULL;
    }

    if (thiz->evt_pause)
    {
        os_event_delete(thiz->evt_pause);
        thiz->evt_pause = NULL;
    }

    if (thiz->audio_handle)
    {
        audio_close(thiz->audio_handle);
        thiz->audio_handle = NULL;
    }
    if (thiz->evt_audio)
    {
        os_event_delete(thiz->evt_audio);
        thiz->evt_audio = NULL;
    }
    if (thiz->av_pkt_queue)
    {
        drop_all_avpacket(thiz->av_pkt_queue);
        os_message_delele_int(thiz->av_pkt_queue);
        thiz->av_pkt_queue = NULL;
    }

    if (thiz->av_pkt_queue_audio)
    {
        drop_all_avpacket(thiz->av_pkt_queue_audio);
        os_message_delele_int(thiz->av_pkt_queue_audio);
        thiz->av_pkt_queue_audio = NULL;
    }

    os_thread_delete(thiz->video_decode_thread);
    os_thread_delete(thiz->audio_decode_thread);

    if (thiz->audio_data && thiz->cfg.mem_free)
    {
        thiz->cfg.mem_free(thiz->audio_data);
        thiz->audio_data = NULL;
    }

    // Codec context is freed inside.
    avformat_close_input(&thiz->fmt_ctx);

    /* note: the internal buffer could have changed, and be != avio_ctx_buffer */
    if (thiz->avio_ctx)
    {
        av_freep(&thiz->avio_ctx->buffer);
        av_freep(&thiz->avio_ctx);
    }
    if (thiz->avio_ctx_buffer)
    {
        av_free(thiz->avio_ctx_buffer);
        thiz->avio_ctx_buffer = NULL;
    }

    if (thiz->network_queue)
    {
        media_queue_close(thiz->network_queue);
        thiz->network_queue = NULL;
    }

    if (thiz->audio_frame)
        av_frame_free(&thiz->audio_frame);

    media_cache_deinit(&thiz->video_cache, VIDEO_BUFFER_CAPACITY);

    rt_free(thiz);

    ffmpeg_memleak_check();

    g_player = NULL;
}

static int read_packet(void *opaque, uint8_t *buf, int buf_size)
{
    uint32_t len = (uint32_t)buf_size;
    media_packet_t *q = NULL;
    ffmpeg_handle thiz = (ffmpeg_handle)opaque;

    while (thiz->is_ok)
    {
        lock();
        q = rt_list_first_entry(&thiz->network_queue->root, media_packet_t, node);
        unlock();
        if (!q)
        {
            media_queue_wait(thiz->network_queue);
            continue;
        }
        if (q->data_len < (uint32_t)buf_size)
        {
            len = q->data_len;
        }
        memcpy(buf, &q->data[q->data_offset], len);
        q->data_len -= len;
        q->data_offset += len;
        media_queue_add_readed(thiz->network_queue, len);
        if (q->data_len == 0)
        {
            lock();
            rt_list_remove(&q->node);
            unlock();
            thiz->cfg.pack_free(q);
        }
        return (int)len;
    }
    LOG_I("avio read exit");
    return -1;
}

static int mediaplayer_start(ffmpeg_handle thiz, bool is_file)
{
    os_event_create(thiz->evt_audio);
    os_event_create(thiz->evt_video);
    os_event_create(thiz->evt_pause);
    os_event_create(thiz->evt_video_decode);

    av_register_all();

    media_cache_init(&thiz->video_cache, VIDEO_BUFFER_CAPACITY);

    if (is_file)
    {
        // open input file, and allocate format context
        if (avformat_open_input(&thiz->fmt_ctx, thiz->cfg.file_path, NULL, NULL) < 0)
        {
            LOG_E("avformat_open_input error");
            goto Exit;
        }
        thiz->fmt_ctx->flags |= AVFMT_FLAG_GENPTS;
        thiz->fmt_ctx->flags |= AVFMT_FLAG_NOBUFFER;
        thiz->fmt_ctx->debug = 0;//-1;
    }
    else
    {
        int ret;
        thiz->network_queue = media_queue_open(thiz->cfg.pack_free);
        if (!thiz->network_queue)
        {
            LOG_E("avio queue no mem");
            goto Exit;
        }
        if (!(thiz->fmt_ctx = avformat_alloc_context()))
        {
            LOG_E("avformat_open_input error");
            goto Exit;
        }

        thiz->avio_ctx_buffer_size = 8 * 1024;
        thiz->avio_ctx_buffer = av_malloc(thiz->avio_ctx_buffer_size);
        if (!thiz->avio_ctx_buffer)
        {
            LOG_E("avio no mem");
            goto Exit;
        }
        thiz->fmt_ctx->flags |= AVFMT_FLAG_GENPTS;
        thiz->fmt_ctx->flags |= AVFMT_FLAG_CUSTOM_IO;
        thiz->fmt_ctx->flags |= AVFMT_FLAG_NOBUFFER;
        thiz->fmt_ctx->debug = 0;//-1;

        thiz->avio_ctx = avio_alloc_context(thiz->avio_ctx_buffer, thiz->avio_ctx_buffer_size,
                                            0, thiz, &read_packet, NULL, NULL);
        if (!thiz->avio_ctx)
        {
            LOG_E("avio ctx error");
            goto Exit;
        }
        thiz->fmt_ctx->pb = thiz->avio_ctx; //make it use AVFMT_FLAG_CUSTOM_IO
        ret = avformat_open_input(&thiz->fmt_ctx, NULL, NULL, NULL);
        if (ret < 0)
        {
            LOG_E("avio err1");
            goto Exit;
        }
    }

    /// retrieve stream information
    if (avformat_find_stream_info(thiz->fmt_ctx, NULL) < 0)
    {
        LOG_E("find stream information\n");
        goto Exit;
    }

    //  Rewind to the start
    if (is_file && !thiz->is_network_file)
    {
        av_seek_frame(thiz->fmt_ctx, 0, 0, AVSEEK_FLAG_BACKWARD);
    }

    for (int i = 0; i < thiz->fmt_ctx->nb_streams; i++)
    {
        if (thiz->fmt_ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO
                || thiz->fmt_ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO)
            continue;
        LOG_I("unsupport codec=%d", thiz->fmt_ctx->streams[i]->codec->codec_type);
    }

    // Get video parameters
    if (open_codec_context(&thiz->video_stream_idx, thiz->fmt_ctx, AVMEDIA_TYPE_VIDEO) >= 0)
    {
        AVStream *video_stream = thiz->fmt_ctx->streams[thiz->video_stream_idx];
        thiz->video_dec_ctx = video_stream->codec;
        thiz->video_dec_ctx->skip_loop_filter = AVDISCARD_NONKEY;

        LOG_I("video codec: format=%d", thiz->video_dec_ctx->pix_fmt);

        if (AV_PIX_FMT_NONE == thiz->video_dec_ctx->pix_fmt)
        {
            if (thiz->cfg.fmt == IMG_DESC_FMT_RGB565)
                thiz->video_dec_ctx->pix_fmt = AV_PIX_FMT_RGB565LE;
            else if (thiz->cfg.fmt == IMG_DESC_FMT_RGB888)
                thiz->video_dec_ctx->pix_fmt = AV_PIX_FMT_RGB24;
            else if (thiz->cfg.fmt == IMG_DESC_FMT_ARGB8888)
                thiz->video_dec_ctx->pix_fmt = AV_PIX_FMT_ARGB;
            else
                thiz->video_dec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
        }
        thiz->width = thiz->video_dec_ctx->width;
        thiz->height = thiz->video_dec_ctx->height;

        int avg_frame_rate_num = video_stream->avg_frame_rate.num;
        if (avg_frame_rate_num > 0)
        {
            thiz->period_float = 1000.0f * (int64_t)video_stream->avg_frame_rate.den
                                 / avg_frame_rate_num;
            thiz->period = (uint32_t)thiz->period_float;
        }
        else
        {
            thiz->period = 40;
            thiz->period_float = 40.0f;
        }
        thiz->total_time_in_seconds = thiz->fmt_ctx->duration;
        char *name = avcodec_get_name(thiz->video_dec_ctx->codec_id);

        thiz->gpu_pic_fmt = e_sifli_fmt_yuv420p;
        if (!strcmp(name, "ezip") || !strcmp(name, "e888"))
        {
            thiz->gpu_pic_fmt = e_sifli_fmt_ezip;
        }
        else if (!memcmp(name, "mjpeg", 5))
        {
            thiz->gpu_pic_fmt = e_sifli_fmt_argb8888;
        }
        LOG_I("video codec: %s stream w=%d h=%d,period=%ld", name, thiz->width, thiz->height, thiz->period);
    }
    else
        LOG_E("cannot find video stream\n");

    // Get Audio parameters
    if (open_codec_context(&thiz->audio_stream_idx, thiz->fmt_ctx, AVMEDIA_TYPE_AUDIO) >= 0)
    {
        AVStream *audio_stream = thiz->fmt_ctx->streams[thiz->audio_stream_idx];
        thiz->audio_dec_ctx = audio_stream->codec;
        thiz->audio_channel = thiz->audio_dec_ctx->channels;
        thiz->audio_samplerate = thiz->audio_dec_ctx->sample_rate;
        LOG_I("audio codec: fmt=%d", thiz->audio_dec_ctx->sample_fmt);
    }
    else
        LOG_E("cannot find audio stream\n");

    thiz->audio_frame = av_frame_alloc();
    if (!thiz->audio_frame)
    {
        LOG_E("Could not allocate audio frame\n");
        goto Exit;
    }
    thiz->is_ok = 1;

    if (thiz->is_network_file)
    {
        // Initialize AV packet queue
        thiz->av_pkt_queue = os_message_queue_create_int("avpkt", NETWORK_BUFFER_CAPACITY, sizeof(AVPacket), NULL, 0);
        RT_ASSERT(thiz->av_pkt_queue != NULL);

        thiz->video_decode_thread = os_thread_create_int("vid_dec", video_audio_decode_thread, thiz,
                                    ffmpeg_thread_stack, sizeof(ffmpeg_thread_stack),
                                    network_decode_task_prio,
                                    RT_THREAD_TICK_DEFAULT);
        RT_ASSERT(thiz->video_decode_thread != NULL);
    }
    else
    {
        // Initialize AV packet queue
        thiz->av_pkt_queue = os_message_queue_create_int("avpkt", READ_BUFFER_CAPACITY, sizeof(AVPacket), NULL, 0);
        RT_ASSERT(thiz->av_pkt_queue != NULL);

        // Start decode thread;
        thiz->av_pkt_queue_audio = os_message_queue_create_int("aud_pkt", READ_BUFFER_CAPACITY, sizeof(AVPacket), NULL, 0);
        RT_ASSERT(thiz->av_pkt_queue_audio != NULL);

        if (thiz->cfg.audio_enable)
        {
            thiz->audio_decode_thread = os_thread_create_int("aud_dec", audio_decode_thread, thiz,
                                        ffmpeg_audio_dec_thread_stack, ffmpeg_audio_dec_thread_stack_size,
                                        audio_dec_task_prio,
                                        RT_THREAD_TICK_DEFAULT);
            RT_ASSERT(thiz->audio_decode_thread != NULL);
        }

        if (thiz->cfg.video_enable)
        {
            thiz->video_decode_thread = os_thread_create_int("vid_dec", video_decode_thread, thiz,
                                        ffmpeg_video_dec_thread_stack, ffmpeg_video_dec_thread_stack_size,
                                        video_dec_task_prio,
                                        RT_THREAD_TICK_DEFAULT);
            RT_ASSERT(thiz->video_decode_thread != NULL);
        }
    }

#if !FFMPEG_OPEN_AYNC
    rt_uint32_t stack_size = 4096;
    rt_uint8_t  priority = av_read_pkt_task_prio;
    if (thiz->is_network_file)
    {
        stack_size = NETWORK_READ_STACK_SIZE;
        priority = network_read_task_prio;
    }
    // Start file read thread;
    thiz->av_pkt_read_thread = rt_thread_create("ffmpeg_read", media_read_thread, thiz,
                               stack_size,
                               priority,
                               RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(thiz->av_pkt_read_thread != NULL);
    rt_thread_startup(thiz->av_pkt_read_thread);
#endif

    LOG_I("mediaplayer_start ok");
    return RT_EOK;
Exit:
    clean_up(thiz);
    LOG_I("mediaplayer_start failed");
    return RT_EIO;
}

static void mediaplayer_stop(ffmpeg_handle thiz)
{
    thiz->is_ok = 0;

    thiz->is_paused = 0;
    os_event_flags_set(thiz->evt_pause, 1); //must after thiz->is_ok = 0, wakeup read thread

    //make avio read exit if network stream
    if (thiz->network_queue)
    {
        media_queue_set(thiz->network_queue);
    }

    LOG_I("mediaplayer_stop async");
}

static void ezip_audio_decode_thread(void *p)
{
    ffmpeg_handle thiz = p;
    ezip_audio_packet_t pkt;
    LOG_I("audio decode task run\n");
    /*readed frame must decode it, or memory leak. so look thiz->is_ok first*/
    while (thiz->is_ok)
    {
        //can't call audio_close() if thiz->is_suspended. otherwie audio can't be resumed
        if (thiz->is_paused || !thiz->cfg.audio_enable)
        {
            //may has no pkt coming, close audio device to save power
            if (thiz->audio_handle)
            {
                audio_close(thiz->audio_handle);
                thiz->audio_handle = NULL;
            }
        }
        //LOG_I("audio get");
        os_message_get(thiz->av_pkt_queue_audio, &pkt, sizeof(pkt), OS_WAIT_FORVER);
        //LOG_I("audio get ok=%p", pkt.buf);
        ezip_audio_decode(thiz, audio_callback_func);
    }

    if (thiz->audio_handle)
    {
        audio_close(thiz->audio_handle);
        thiz->audio_handle = NULL;
    }
    LOG_I("%s exit", __FUNCTION__);
}

static void ezip_clean_up(ffmpeg_handle thiz)
{
    if (thiz->cfg.src == e_src_localfile)
    {
        if (thiz->is_nand)
        {
            thiz->ezip_fd = 0;
        }
        else
        {
#if RT_USING_DFS
            close(thiz->ezip_fd);
            thiz->ezip_fd = -1;
        }
#endif
    }

    if (thiz->evt_video)
    {
        os_event_delete(thiz->evt_video);
        thiz->evt_video = NULL;
    }

    if (thiz->evt_video_decode)
    {
        os_event_delete(thiz->evt_video_decode);
        thiz->evt_video_decode = NULL;
    }

    if (thiz->evt_pause)
    {
        os_event_delete(thiz->evt_pause);
        thiz->evt_pause = NULL;
    }

    if (thiz->audio_handle)
    {
        audio_close(thiz->audio_handle);
        thiz->audio_handle = NULL;
    }
    if (thiz->evt_audio)
    {
        os_event_delete(thiz->evt_audio);
        thiz->evt_audio = NULL;
    }

    if (thiz->av_pkt_queue_audio)
    {
        ezip_audio_packet_t pkt = {0};
        while (RT_EOK == os_message_get(thiz->av_pkt_queue_audio, &pkt, sizeof(pkt), 0));

        os_message_delele_int(thiz->av_pkt_queue_audio);
        thiz->av_pkt_queue_audio = NULL;
    }

    os_thread_delete(thiz->audio_decode_thread);

    if (thiz->audio_data && thiz->cfg.mem_free)
    {
        thiz->cfg.mem_free(thiz->audio_data);
        thiz->audio_data = NULL;
    }
    ezip_audio_cache_deinit(thiz);
    ezip_video_cache_deinit(thiz);

    rt_free(thiz);

    ffmpeg_memleak_check();

    g_player = NULL;

}
static void ezip_read_thread(void *p)
{
    long readed;
    sifli_ezip_packet_t packet;
#ifdef DEBUG_IO_SPEED
    rt_tick_t  io_start, io_ticks = 0;
    uint32_t io_read_size = 0;
#endif /*DEBUG_IO_SPEED*/

    ffmpeg_handle thiz = p;

    LOG_I("%s run", __FUNCTION__);

    while (thiz->is_ok)
    {
        if (thiz->seeking_state == 1)
        {
            if (thiz->is_nand)
            {
                ezip_flash_seek(thiz, sizeof(ezip_media_t), SEEK_SET);
            }
            else
            {
                lseek(thiz->ezip_fd, sizeof(ezip_media_t), SEEK_SET);
            }
            thiz->frame_index = 0;
            while (1)
            {
                int read_result = 0;
                if (thiz->is_nand)
                {
                    read_result = ezip_flash_read(thiz, &packet, sizeof(packet));
                }
                else
                {
#if RT_USING_DFS
                    read_result = read(thiz->ezip_fd, &packet, sizeof(packet));
#endif
                }

                if (read_result > 0)
                {
                    if (thiz->is_nand)
                    {
                        ezip_flash_seek(thiz, packet.data_len, SEEK_CUR);
                    }
                    else
                    {
#if RT_USING_DFS
                        lseek(thiz->ezip_fd, packet.data_len, SEEK_CUR);
#endif
                    }
                    if (!packet.is_audio)
                    {
                        uint32_t sec;
                        sec = thiz->frame_index * thiz->period_float / 1000;
                        thiz->frame_index++;
                        if (sec >= thiz->seek_to_second)
                        {
                            thiz->last_seconds = sec;
                            thiz->cfg.notify(thiz->user_data, e_ffmpeg_progress, sec);
                            break;
                        }
                    }
                }
                else
                {
                    thiz->last_seconds = thiz->total_time_in_seconds;
                    thiz->cfg.notify(thiz->user_data, e_ffmpeg_progress, thiz->total_time_in_seconds);
                    break;
                }
            }
            thiz->seeking_state = 0;
        }
        if (thiz->is_paused || thiz->is_suspended)
        {
            LOG_I("read paused=%d suspend=%d", thiz->is_paused, thiz->is_suspended);
            os_event_flags_wait(thiz->evt_pause, 1, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, OS_WAIT_FORVER, NULL);
            LOG_I("read resumed");
            continue; //may exit after resume
        }
#ifdef DEBUG_IO_SPEED
        io_start = rt_tick_get_millisecond();
#endif /*DEBUG_IO_SPEED*/
        TRACE_MARK_START(TRACEID_AV_PACKET);

        int read_result = 0;
        if (thiz->is_nand)
        {
            read_result = ezip_flash_read(thiz, &packet, sizeof(packet));
        }
        else
        {
#if RT_USING_DFS
            read_result = read(thiz->ezip_fd, &packet, sizeof(packet));
#endif
        }
        if (read_result <= 0)
        {
            LOG_I("read end");
            TRACE_MARK_STOP(TRACEID_AV_PACKET);
            if (thiz->cfg.notify && !thiz->is_closing)
                thiz->cfg.notify(thiz->user_data, e_ffmpeg_play_to_end, 0);

            if (thiz->cfg.is_loop)
            {
                if (thiz->is_nand)
                {
                    ezip_flash_seek(thiz, sizeof(ezip_media_t), SEEK_SET);
                }
                else
                {
#if RT_USING_DFS
                    lseek(thiz->ezip_fd, sizeof(ezip_media_t), SEEK_SET);
#endif
                }
                thiz->frame_index = 0;
                thiz->last_seconds = -1;
                continue;
            }
            else
                break;
        }
        TRACE_MARK_STOP(TRACEID_AV_PACKET);

        //LOG_I("is_audio=%d len=%d, pad=%d", packet.is_audio, packet.data_len, packet.padding_size);

        if (packet.is_audio)
        {
            ezip_audio_packet_t *audio = ezip_audio_read_packet(thiz, packet.data_len, packet.padding_size);
            RT_ASSERT(audio);
            os_message_queue_t q = thiz->av_pkt_queue_audio;
            uint32_t   trace_id  = TRACEID_AUDIO_PACKET;
            TRACE_MARK_START(trace_id);
            //LOG_I("audio put");
            while (thiz->is_ok && 0 != os_message_put(q, audio, sizeof(ezip_audio_packet_t), 0))
            {
                os_delay(1);
                //LOG_I("audio put again");
                if (thiz->is_paused || thiz->is_suspended)
                {
                    LOG_I("put paused=%d suspend=%d", thiz->is_paused, thiz->is_suspended);
                    os_event_flags_wait(thiz->evt_pause, 1, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, OS_WAIT_FORVER, NULL);
                    LOG_I("put resumed");
                    continue; //may exit after resume
                }
            }
            //LOG_I("audio put ok");
            TRACE_MARK_STOP(trace_id);
        }
        else
        {
            if (thiz->ezip_header.audio_codec[0] == 0 || !thiz->cfg.audio_enable)
            {
                while (thiz->is_ok)
                {
                    if (!ezip_video_need_decode(thiz))
                    {
                        os_delay(5);
                        continue;
                    }
                }
            }
            ezip_video_decode(thiz, packet.data_len, packet.padding_size);
            if (thiz->cfg.notify && !thiz->is_closing)
            {
                uint32_t seconds = thiz->frame_index * thiz->period_float / 1000;
                if (seconds != thiz->last_seconds)
                {
                    thiz->last_seconds = seconds;
                    thiz->cfg.notify(thiz->user_data, e_ffmpeg_progress, seconds);
                }
            }
            thiz->frame_index++;
        }

#ifdef DEBUG_IO_SPEED
        io_read_size += packet.data_len;
        io_ticks += rt_tick_get_millisecond() - io_start;
        if (io_ticks > 1000)
        {
            LOG_I("IO1 Speed %d kB/S", io_read_size / io_ticks);
            io_ticks = 0;
            io_read_size = 0;
        }
#endif /*DEBUG_IO_SPEED*/
    }

    while (rt_thread_find("ezipaud"))
    {
        ezip_audio_packet_t pkt = {0};
        os_message_put(thiz->av_pkt_queue_audio, &pkt, sizeof(pkt), 0);
        if (thiz->evt_audio)   os_event_flags_set(thiz->evt_audio, 1);
        rt_thread_mdelay(10);
        LOG_I("wait ezip audio thread exit");
    }

    ezip_clean_up(thiz);

    LOG_I("ezip read exit");
}

static bool demux_sifli_ezip_media(ffmpeg_handle thiz)
{
    if (thiz->is_network_file)
        return false;

    int fd;
    thiz->ezip_fd = -1;
    memset(&thiz->ezip_header, 0, sizeof(thiz->ezip_header));
    if (thiz->is_nand)
    {
        thiz->ezip_fd = 0;
        ezip_flash_read(thiz, &thiz->ezip_header, sizeof(thiz->ezip_header));
    }
    else
    {
#if RT_USING_DFS
        fd = open(thiz->cfg.file_path, O_RDONLY | O_BINARY);
        thiz->ezip_fd = fd;
        if (fd < 0)
            return false;
        read(fd, &thiz->ezip_header, sizeof(thiz->ezip_header));
#else
        return false;
#endif
    }

    if (memcmp(SIFLI_MEDIA_MAGIC, thiz->ezip_header.header, 8))
    {
        if (thiz->is_nand)
        {
            thiz->ezip_fd = 0;
        }
        else
        {
#if RT_USING_DFS
            if (thiz->cfg.src == e_src_localfile)
            {
                close(fd);
                thiz->ezip_fd = -1;
            }
#endif
        }
        return false;
    }

    os_event_create(thiz->evt_audio);
    os_event_create(thiz->evt_pause);
    ezip_audio_cache_init(thiz);
    ezip_video_cache_init(thiz);


    thiz->width = thiz->ezip_header.width;
    thiz->height = thiz->ezip_header.height;
    thiz->period = 1000 / thiz->ezip_header.fps;
    thiz->period_float = 1000.0f / thiz->ezip_header.fps;
    thiz->total_time_in_seconds = thiz->ezip_header.duration_seconds;

    thiz->is_sifli_ezip_memdia = 1;
    thiz->gpu_pic_fmt = e_sifli_fmt_ezip;
    thiz->is_ok = 1;
    thiz->ezip_header.audio_codec[3] = 0;
    LOG_I("ezip w=%d h=%d fps=%d smp=%d encode=%s ch=%d",
          thiz->width,
          thiz->height,
          thiz->ezip_header.fps,
          thiz->ezip_header.samplerate,
          thiz->ezip_header.audio_codec,
          thiz->ezip_header.ch);

    // Start file read thread;
    rt_uint32_t stack_size = 2048;
    rt_uint8_t  priority = av_read_pkt_task_prio;

    // Start decode thread;
    thiz->av_pkt_queue_audio = os_message_queue_create_int("aud_pkt", READ_BUFFER_CAPACITY, sizeof(ezip_audio_packet_t), NULL, 0);
    RT_ASSERT(thiz->av_pkt_queue_audio != NULL);

    if (thiz->cfg.audio_enable)
    {
        thiz->audio_decode_thread = os_thread_create_int("ezipaud", ezip_audio_decode_thread, thiz,
                                    ffmpeg_audio_dec_thread_stack, ffmpeg_audio_dec_thread_stack_size,
                                    audio_dec_task_prio,
                                    RT_THREAD_TICK_DEFAULT);
        RT_ASSERT(thiz->audio_decode_thread != NULL);
    }


    thiz->av_pkt_read_thread = rt_thread_create("ezipread", ezip_read_thread, thiz,
                               stack_size,
                               priority,
                               RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(thiz->av_pkt_read_thread != NULL);
    rt_thread_startup(thiz->av_pkt_read_thread);
    return true;
}

void ezip_media_stop(ffmpeg_handle thiz)
{
    thiz->is_ok = 0;
    thiz->is_paused = 0;
    os_event_flags_set(thiz->evt_pause, 1); //wakeup read thread
}

//only call in UI, thread safe
int ffmpeg_open(ffmpeg_handle *return_hanlde, ffmpeg_config_t *cfg, uint32_t user_data)
{
    int ret = 0;
    ffmpeg_handle thiz;
    if (!return_hanlde || !cfg || !cfg->mem_malloc || !cfg->mem_free)
        return -3;

    LOG_I("%s", __FUNCTION__);
    for (int i = 0; i < 20; i++)
    {
        if (!g_player)
        {
            break;
        }
        os_delay(10);
        LOG_I("wait media exit");
    }
    rt_enter_critical();
    do
    {
        if (g_player)
        {
            *return_hanlde = NULL;
            ret = -2;
            break;
        }
        else
        {
            g_player = (ffmpeg_handle) - 1; //make it busy temporary
            ret = 0;
        }
    }
    while (0);
    rt_exit_critical();

    if (ret < 0)
    {
        LOG_I("busy");
        return ret;
    }

    //make it not busy or busy
    g_player = (ffmpeg_handle)rt_calloc(1, sizeof(ffmpeg_decoder_t));
    thiz = g_player;
    *return_hanlde = thiz;

    if (!thiz)
    {
        return -1;
    }
    thiz->magic = FFMPEG_HANDLE_MAGIC;
    thiz->user_data = user_data;
    thiz->last_seconds = -1;
    memcpy(&thiz->cfg, cfg, sizeof(ffmpeg_config_t));

    if (!strncmp("http:/", cfg->file_path, 6) || !strncmp("https:/", cfg->file_path, 7))
    {
        thiz->is_network_file = 1;
    }
    if (!strncmp("nand://", cfg->file_path, 7))
    {
        uint32_t address;
        uint32_t len;
        thiz->is_nand = 1;
        sscanf(cfg->file_path, "nand://addr=0x%x&len=0x%x", &address, &len);
        thiz->src_in_nand_address = (uint8_t *)address;
        thiz->src_in_nand_len = len;
        LOG_I("nand: address=0x%x, len=0x%x", address, len);
    }
    if (demux_sifli_ezip_media(thiz))
    {
        return 0;
    }


#if FFMPEG_OPEN_AYNC
    os_event_create(thiz->evt_init);
    RT_ASSERT(thiz->evt_init);

    /* this API called in UI, can't call ffmpeg internal api, may dead lock:
       ffmpeg api-->av_malloc---->ffmpeg_alloc-->image_cache_alloc--->wait UI release ramfs.
       using a thread to start ffmpeg
    */

    // Start file read thread;
    rt_uint32_t stack_size = 8192;
    rt_uint8_t  priority = av_read_pkt_task_prio;
    if (thiz->is_network_file)
    {
        stack_size = NETWORK_READ_STACK_SIZE;
        priority = network_read_task_prio;
    }

    thiz->av_pkt_read_thread = rt_thread_create("ffmpeg_read", media_read_thread, thiz,
                               stack_size,
                               priority,
                               RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(thiz->av_pkt_read_thread != NULL);
    rt_thread_startup(thiz->av_pkt_read_thread);

    rt_uint32_t evt = 0;
    os_event_flags_wait(thiz->evt_init, EVT_INIT_OK | EVT_INIT_FAILED, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, 5000, &evt);

    ret = RT_ERROR;
    if (evt & EVT_INIT_OK)
    {
        ret = RT_EOK;
    }
    os_event_delete(thiz->evt_init);
    thiz->evt_init = NULL;

#else
    if (cfg->src == e_src_localfile)
    {
        LOG_E("mediaplayer_start %s fmt=%d", cfg->file_path, cfg->fmt);
        ffmeg_mem_init();
        ret = mediaplayer_start(thiz, 1);
    }
    else if (cfg->src == e_network_frames_stream)
    {
        ret = mediaplayer_start(thiz, 0);
    }
    else
    {
        ret = -1;
        LOG_E("under development");
        RT_ASSERT(0);
    }
#endif

    if (ret != RT_EOK)
    {
        if (g_player)
        {
            g_player->magic = ~FFMPEG_HANDLE_MAGIC;
            rt_free(g_player);
            g_player = NULL;
        }
    }

    return ret;
}

void ffmpeg_close(ffmpeg_handle thiz)
{
    if (!thiz || !g_player || thiz->magic != FFMPEG_HANDLE_MAGIC || thiz->is_closing)
        return;

    LOG_I("%s", __FUNCTION__);

    thiz->is_closing = 1;
    if (thiz->is_sifli_ezip_memdia)
    {
        ezip_media_stop(thiz);
        return;
    }
    mediaplayer_stop(thiz);

    return;

}

bool ffmpeg_is_video_available(ffmpeg_handle thiz)
{
    rt_slist_t *root;
    rt_slist_t *decoded;
    if (!thiz || thiz->magic != FFMPEG_HANDLE_MAGIC)
        return false;

    bool has_audio = false;
    root = &thiz->video_cache.decoded_frame_slist;
    if (thiz->is_sifli_ezip_memdia)
    {
        root = &thiz->ezip_video_cache.decoded_video_slist;
        if (thiz->ezip_header.audio_codec[0])
        {
            has_audio = true;
        }
    }
    else
    {
        if (thiz->audio_dec_ctx)
        {
            has_audio = true;
        }
    }
    if (!thiz->cfg.audio_enable)
    {
        has_audio = false;
    }
    rt_enter_critical();
    decoded = rt_slist_first(root);
    rt_exit_critical();
    if (decoded)
    {
        if (!has_audio)
        {
            rt_tick_t cur = rt_tick_get();
            rt_tick_t delta = cur - thiz->last_video_get_tick;
            if (delta < thiz->period && (thiz->period - delta) > 4)
            {
                return false;
            }
            thiz->last_video_get_tick = cur;
        }
    }
    return (decoded != NULL);
}

int ffmpeg_next_video_frame(ffmpeg_handle thiz, uint8_t *data)
{
    uint32_t current, last;

    if (!thiz || thiz->magic != FFMPEG_HANDLE_MAGIC || !data)
        return -RT_EINVAL;

    if (thiz->is_sifli_ezip_memdia)
    {
        uint32_t current, last;
        rt_slist_t *decoded;
        rt_slist_t *display;
        rt_slist_t *decoded_root;
        rt_slist_t *display_root;
        rt_slist_t *empty_root;
        decoded_root = &thiz->ezip_video_cache.decoded_video_slist;
        empty_root = &thiz->ezip_video_cache.empty_video_slist;
        display_root = &thiz->ezip_video_cache.display_video_slist;

        rt_enter_critical();
        decoded = rt_slist_first(decoded_root);
        if (decoded)
        {
            display = rt_slist_first(display_root);
            if (display) //move display to empty, only one in display list
            {
                rt_slist_remove(display_root, display);
                rt_slist_append(empty_root, display);
            }
            //move decoded to display
            rt_slist_remove(decoded_root, decoded);
            rt_slist_append(display_root, decoded);
        }

        rt_exit_critical();

        if (!decoded)
        {
            LOG_D("video get empty");
            return -RT_EEMPTY;;
        }
        ezip_video_packet_t *packet = rt_container_of(decoded, ezip_video_packet_t, snode);
        uint8_t **frame_data = (uint8_t **)data;
        frame_data[0] = packet->buffer;
        frame_data[1] = (uint8_t *)(packet->data_len);
        frame_data[2] = (uint8_t *)IMG_DESC_FMT_EZIP;
        return 0;
    }

    return media_video_get(&thiz->video_cache, thiz->cfg.fmt, data, e_sifli_fmt_ezip == thiz->gpu_pic_fmt);
}

void ffmpeg_eizp_release(uint8_t *ezip)
{
    av_free(ezip);
}

uint8_t *ffmpeg_get_first_ezip(const char *filename, uint32_t *w, uint32_t *h, uint32_t *psize)
{
    uint8_t *ezip = NULL;
    ezip_media_t ezip_header = {0};
    LOG_I("%s: filename %s", __func__, filename);
    int fd = open(filename, O_RDONLY | O_BINARY);
    if (fd < 0)
        return NULL;
    read(fd, &ezip_header, sizeof(ezip_header));
    if (memcmp(SIFLI_MEDIA_MAGIC, ezip_header.header, 8))
    {
        close(fd);
        return false;
    }

    if (w && h)
    {
        *w = ezip_header.width;
        *h = ezip_header.height;
    }

    sifli_ezip_packet_t packet;
    while (1)
    {
        if (read(fd, &packet, sizeof(packet)) <= 0)
        {
            LOG_I("%s: read packet fail", __func__);
            break;
        }

        if (packet.is_audio)
        {
            lseek(fd, packet.data_len, SEEK_CUR);
        }
        else
        {
            if (psize)
            {
                *psize = packet.data_len - packet.padding_size;
            }
            ezip = av_malloc(*psize);
            RT_ASSERT(ezip);
            if (ezip)
            {
                read(fd, ezip, *psize);
                LOG_I("%s: read ezip success", __func__);
                break;
            }
        }
    }
    close(fd);
    return ezip;

}
uint8_t *ffmpeg_get_first_ezip_in_nand(const char *nand_address, uint32_t nand_size, uint32_t *w, uint32_t *h, uint32_t *psize)
{
    uint32_t offset = 0;
    uint8_t *ezip = NULL;
    ezip_media_t ezip_header = {0};

    lv_img_decode_flash_read(nand_address + offset, &ezip_header, sizeof(ezip_header));
    offset += sizeof(ezip_header);

    if (memcmp(SIFLI_MEDIA_MAGIC, ezip_header.header, 8))
    {
        return false;
    }

    if (w && h)
    {
        *w = ezip_header.width;
        *h = ezip_header.height;
    }

    sifli_ezip_packet_t packet;
    while (1)
    {
        if (offset >= nand_size)
        {
            break;
        }
        lv_img_decode_flash_read(nand_address + offset, &packet, sizeof(packet));
        offset += sizeof(packet);

        if (packet.is_audio)
        {
            offset += packet.data_len;
        }
        else
        {
            if (psize)
            {
                *psize = packet.data_len - packet.padding_size;
            }
            ezip = av_malloc(*psize);
            RT_ASSERT(ezip);
            if (ezip)
            {
                lv_img_decode_flash_read(nand_address + offset, ezip, *psize);
                offset += *psize;
                LOG_I("%s: read ezip success", __func__);
                break;
            }
        }
    }
    return ezip;
}

int ffmpeg_get_video_info(ffmpeg_handle thiz, uint32_t *video_width, uint32_t *video_height, video_info_t *info)
{
    if (!thiz
            || thiz->magic != FFMPEG_HANDLE_MAGIC
            || !video_width
            || !video_height
            || !info)
        return -RT_EINVAL;
    if (thiz->width && thiz->height)
    {
        *video_width  = thiz->width;
        *video_height = thiz->height;
        info->gpu_pic_fmt = thiz->gpu_pic_fmt;
        info->period = thiz->period;
        info->total_time_in_seconds = thiz->total_time_in_seconds;
        return 0;
    }
    return -RT_EEMPTY;
}

void ffmpeg_pause(ffmpeg_handle thiz)
{
    if (thiz && thiz->magic == FFMPEG_HANDLE_MAGIC)
    {
        LOG_I("pause");
        thiz->is_paused = 1;
    }
}
void ffmpeg_seek(ffmpeg_handle thiz, uint32_t second)
{
    if (thiz && thiz->magic == FFMPEG_HANDLE_MAGIC)
    {
        thiz->seek_to_second = second;
        thiz->seeking_state = 1;
        while (1)
        {
            if (thiz->is_paused || thiz->is_suspended)
            {
                break;
            }
            if (!thiz->seeking_state)
                break;

            os_delay(100);
            LOG_I("ffmpeg seeking");
        }
    }
}
void ffmpeg_resume(ffmpeg_handle thiz)
{
    if (thiz && thiz->magic == FFMPEG_HANDLE_MAGIC)
    {
        LOG_I("resume");
        thiz->is_paused = 0;
        os_event_flags_set(thiz->evt_pause, 1);
    }
}
void ffmpeg_audio_mute(ffmpeg_handle thiz, bool is_mute)
{
    if (thiz && thiz->magic == FFMPEG_HANDLE_MAGIC)
    {
        thiz->cfg.audio_enable = !is_mute;
        LOG_I("mute=%d", is_mute);
    }
}
void ffmpeg_send_frame_to_decoder(ffmpeg_handle thiz, media_packet_t *p)
{
    if (thiz && thiz->magic == FFMPEG_HANDLE_MAGIC && thiz->network_queue && p)
    {
        media_queue_add_tail(thiz->network_queue, p);
    }
}

#if 0
static void ff_seek(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        ffmpeg_seek(g_player, atoi(argv[1]));
    }
}

MSH_CMD_EXPORT(ff_seek, ff_seek commnad);
#endif

