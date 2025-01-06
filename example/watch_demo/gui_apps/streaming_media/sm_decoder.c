#include "sm_api.h"

#define MEDIAPLAYER_USING_ACPU  0

static const int refcount = 1;

#define DEBUG_FFMPEG_FPS        1
#define USING_FAST_YUV2RGB      1


//share stack with mediaplayer, must not run at same time
extern uint32_t ffmpeg_thread_stack[4 * 1024];

RT_WEAK void app_watchdog_pet(void)
{
}

typedef struct streaming_tag
{
    media_queue_t              *queue_ptr;
    uint32_t                period;
    uint32_t                width;
    uint32_t                height;
    AVFormatContext        *fmt_ctx;
    AVPacket                pkt;
    AVCodecContext         *video_dec_ctx;
    AVCodecContext         *audio_dec_ctx;
    AVStream               *video_stream;
    AVStream               *audio_stream;
    AVCodec                *codec;
    AVFrame                *decode_frame;
    media_cache_t           cache;
    __IO int                is_audio_cache_empty_occured;
    streaming_callback_t  callbacks;
    int                     fmt;
    __IO int                is_ok;
    uint32_t                audio_samples;
    uint32_t                audio_samplerate;
    uint32_t                audio_channel;
    uint32_t                audio_frame_size;
    void                   *audio_frame_cache;
    __IO uint32_t           video_state;
    uint8_t                *mute_frame;
    uint32_t last_frame_tick;
} streaming_t;

#define USING_FAST_YUV2RGB  1


typedef streaming_t *streaming_handle;

static __IO uint8_t g_need_wait_i_frame = 0;
static __IO uint8_t g_need_clean_buffer = 0;
static uint8_t g_audio_trgger = 0;
static streaming_handle g_player = NULL;

#ifndef BSP_USING_PC_SIMULATOR
    static audio_client_t  audio_handle = NULL;
    static OS_EVENT_DECLAR(evt_audio);
#endif




static uint32_t g_size;

static uint32_t g_decode_v;
static uint32_t g_decode_a;
static uint32_t g_decode_last_v;
static uint32_t g_decode_last_a;

static uint8_t g_media_decode_exit = 0;

void sm_debug_get_decode_info(uint32_t *dv, uint32_t *da)
{
    *dv = g_decode_v - g_decode_last_v;
    *da = g_decode_a - g_decode_last_a;
    g_decode_last_v = g_decode_v;
    g_decode_last_a = g_decode_a;
}

#undef lock
#undef unlock
#define lock()      _lock(g_player->queue_ptr)
#define unlock()    _unlock(g_player->queue_ptr)


#define DROP_VIDEO_FRAME    1

#ifndef BSP_USING_PC_SIMULATOR
static int audio_callback_func(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (evt_audio)
            os_event_flags_set(evt_audio, 1);
    }
    return 0;
}
#endif

/*
  must finish decode to avoid memory leak in packets
*/
void *app_sram_alloc(size_t size);
void app_sram_free(void *ptr);

extern void ffmeg_mem_init();
void ffmpeg_memleak_check();


#define AUDIO_CACHE_SIZE 32000
static int decode_audio_frame(AVCodecContext *audio_dec_ctx,  int *got_frame, AVFrame *frame, AVPacket *pkt)
{
    int decoded = g_player->pkt.size;

    int ret = avcodec_decode_audio4(audio_dec_ctx, frame, got_frame, pkt);
    if (ret < 0)
    {
        LOG_E("Error decoding audio frame (%s)\n", av_err2str(ret));
        decoded = -1;
        goto Exit_decode;
    }
    /* Some audio decoders decode only part of the packet, and have to be
     * called again with the remainder of the packet data.
     * Sample: fate-suite/lossless-audio/luckynight-partial.shn
     * Also, some decoders might over-read the packet. */
    decoded = FFMIN(ret, g_player->pkt.size);

    if (*got_frame)
    {
        //LOG_D("audio pts=%f", g_player->pkt.pts * av_q2d(g_player->audio_dec_ctx->time_base));
        if (g_player->mute_frame == NULL)
        {
            g_player->audio_samples = frame->nb_samples * frame->channels;
            g_player->audio_channel = frame->channels;
            g_player->audio_samplerate = frame->sample_rate;

            LOG_E("av_get_bytes_per_sample=%d", av_get_bytes_per_sample(frame->format));
            g_player->audio_frame_size = g_player->audio_samples * 2; // av_get_bytes_per_sample(frame->format);

            g_player->mute_frame = g_player->callbacks.mem_malloc(g_player->audio_frame_size);

            if (!g_player->mute_frame)
            {
                LOG_E("audio frame no mem, size=%d", g_player->audio_frame_size);
                g_player->is_ok = 0;
                goto Exit_decode;
            }
            else
            {
                audio_parameter_t arg = {0};
                arg.write_bits_per_sample = 16;
#if AUDIO_USING_44100
                arg.write_samplerate = 44100;
#else
                arg.write_samplerate = g_player->audio_samplerate;
#endif
                arg.write_channnel_num = frame->channels < 2 ? 1 : 2;
                arg.write_cache_size = AUDIO_CACHE_SIZE;
                rt_kprintf("- now volume:%d\n", audio_server_get_private_volume(AUDIO_TYPE_LOCAL_MUSIC));
                audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 15);

                audio_handle = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &arg, audio_callback_func, NULL);
                RT_ASSERT(audio_handle);

                memset(g_player->mute_frame, 0, g_player->audio_frame_size);

                //fill audio buffer to full status
                for (int f = 0; f < AUDIO_CACHE_SIZE / g_player->audio_frame_size; f++)
                {
                    audio_write(audio_handle, (uint8_t *)g_player->mute_frame, g_player->audio_frame_size);
                }
            }

        }
        else if (g_player->audio_samples != (frame->nb_samples * frame->channels)
                 || g_player->audio_channel != frame->channels
                 || g_player->audio_samplerate != frame->sample_rate)
        {
            LOG_E("wrong audio format");
            goto Exit_decode;
        }

        media_audio_get(frame, (uint16_t *)g_player->mute_frame);
        while (0 == audio_write(audio_handle, (uint8_t *)g_player->mute_frame, g_player->audio_frame_size))
        {
            uint32_t    evt = 0;
            //rt_kprintf("== media_decode_thread_for_audio8\n");
            os_event_flags_wait(evt_audio, 1, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, 20, &evt);
            if (g_player->is_ok == 0)
            {
                LOG_I("media_decode_thread_for_audio exit @audio");
                goto Exit_decode;
            }
        }
    }
    return decoded;

Exit_decode:
    /* If we use frame reference counting, we own the data and need
     * to de-reference it when we don't use it anymore */
    if ((*got_frame) && refcount)
        av_frame_unref(frame);

    return decoded;
}

AVPacket packet_h264;
AVPacket packet_aac;



AVFrame *frame_aac = NULL;


extern uint8_t g_ffmpeg_debug;

static void media_decode_thread_entry(void *p)
{
    uint8_t   g_current_decode;
    int ret;
    int audio_frame_index = 0;
    int video_frame_index = 0;
    int got_frame = 0;

    g_decode_v = 0;
    g_decode_a = 0;
    g_decode_last_v = 0;
    g_decode_last_a = 0;

    g_player->video_state = 1;

    AVCodec *codec_h264 = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec_h264)
    {
        rt_kprintf("Codec not found\n");
        return;
    }
    AVCodecContext *codec_ctx_h264 = avcodec_alloc_context3(codec_h264);
    if (!codec_ctx_h264)
    {
        rt_kprintf("Could not allocate video codec context\n");
        return;
    }
    g_player->video_dec_ctx = codec_ctx_h264;



    AVCodecParserContext *parser_h264 = av_parser_init(AV_CODEC_ID_H264);
    if (!parser_h264)
    {
        rt_kprintf("Could not create H264 parser\n");
        return;
    }

    if (avcodec_open2(codec_ctx_h264, codec_h264, NULL) < 0)
    {
        rt_kprintf("Could not open codec\n");
        return;
    }

    codec_ctx_h264->skip_loop_filter |= AVDISCARD_NONINTRA;

    AVCodec *codec_aac = avcodec_find_decoder(AV_CODEC_ID_AAC);
    if (!codec_aac)
    {
        rt_kprintf("Codec not found\n");
        return;
    }

    AVCodecContext *codec_ctx_aac = avcodec_alloc_context3(codec_aac);
    if (!codec_ctx_aac)
    {
        rt_kprintf("Could not allocate video codec context\n");
        return;
    }
    g_player->audio_dec_ctx = codec_ctx_aac;
    codec_ctx_aac->request_sample_fmt = AV_SAMPLE_FMT_FLT;

    AVCodecParserContext *parser_aac = av_parser_init(AV_CODEC_ID_AAC);

    if (!parser_aac)
    {
        rt_kprintf("Could not create H264 parser\n");
        return;
    }

    if (avcodec_open2(codec_ctx_aac, codec_aac, NULL) < 0)
    {
        rt_kprintf("Could not open codec\n");
        return;
    }

    frame_aac = av_frame_alloc();
    if (!frame_aac)
    {
        rt_kprintf("Could not allocate audio frame\n");
        return;
    }


    parser_h264->flags |= PARSER_FLAG_COMPLETE_FRAMES;
    while (1)
    {
        uint32_t bytes_used;
        uint8_t *data = NULL;
        int size = 0;
        media_packet_t *q = NULL;
        uint8_t shouldwait = 0;
        app_watchdog_pet();

        if (g_media_decode_exit)
        {
            LOG_E("decode exit1");
            g_need_clean_buffer = 0;
            break;
        }
        if (g_need_clean_buffer)
        {
            media_queue_clean(g_player->queue_ptr);
            g_need_clean_buffer = 0;
            g_need_wait_i_frame = 1;
        }
        if (!g_audio_trgger)
        {
            if (media_bytes_in_queue(g_player->queue_ptr) < VIDEO_THRESHHOLD)
            {
                rt_thread_mdelay(100);
                continue;
            }
            g_audio_trgger = 1;
        }
        lock();
        if (rt_list_isempty(&g_player->queue_ptr->root))
        {
            shouldwait = 1;
        }
        else
        {
            shouldwait = 0;
            q = rt_list_first_entry(&g_player->queue_ptr->root, media_packet_t, node);
        }
        unlock();

        if (shouldwait)
        {
            media_queue_wait(g_player->queue_ptr);
            continue;
        }

        g_current_decode = q->data_type;
        if (q->data_type == 'a')
        {
            bytes_used = av_parser_parse2(parser_aac, codec_ctx_aac, &data, &size, &q->data[q->data_offset], q->data_len, 0, 0, AV_NOPTS_VALUE);
        }
        else
        {
            bytes_used = av_parser_parse2(parser_h264, codec_ctx_h264, &data, &size, &q->data[q->data_offset], q->data_len, 0, 0, AV_NOPTS_VALUE);
        }

        q->data_len -= bytes_used;
        q->data_offset += bytes_used;
        media_queue_add_readed(g_player->queue_ptr, bytes_used);

        if (size == 0)
        {
            if (q->data_len == 0)
            {
                if (q->data_type == 'a')
                    g_decode_a++;
                else
                    g_decode_v++;
                lock();
                rt_list_remove(&q->node);
                unlock();
                sm_packet_free(q);
            }
            continue;
        }

        // We have data of one packet, decode it; or decode whatever when ending
        if (g_current_decode == 'a')
        {
            av_init_packet(&packet_aac);
            packet_aac.data = data;
            packet_aac.size = size;
            ret = decode_audio_frame(codec_ctx_aac, &got_frame, frame_aac, &packet_aac);
        }
        else
        {
            do
            {
                if (g_need_wait_i_frame)
                {
                    if (AV_PICTURE_TYPE_I == parser_h264->pict_type)
                        g_need_wait_i_frame = 0;
                    else
                    {
                        LOG_D("drop p frame after clean");
                        break;
                    }
                }
                av_init_packet(&packet_h264);
                packet_h264.data = data;
                packet_h264.size = size;
                ret = media_decode_video(&g_player->cache,
                                         codec_ctx_h264,
                                         &got_frame,
                                         &packet_h264);
            }
            while (0);
        }

        if (q->data_len == 0)
        {
            if (q->data_type == 'a')
                g_decode_a++;
            else
                g_decode_v++;

            lock();
            rt_list_remove(&q->node);
            unlock();
            sm_packet_free(q);
        }
        if (ret < 0)
            rt_kprintf("Decode or write frame error\n");
    }

    av_init_packet(&packet_aac);
    packet_h264.data = NULL;
    packet_h264.size = 0;
    do
    {
        media_decode_video(&g_player->cache,
                           codec_ctx_h264,
                           &got_frame,
                           &packet_h264);
    }
    while (got_frame);
    media_cache_deinit(&g_player->cache, VIDEO_BUFFER_CAPACITY);

    av_init_packet(&packet_aac);
    packet_aac.data = NULL;
    packet_aac.size = 0;
    do
    {
        decode_audio_frame(codec_ctx_aac, &got_frame, frame_aac, &packet_aac);
    }
    while (got_frame);
    av_frame_free(&frame_aac);

    av_parser_close(parser_aac);
    avcodec_free_context(&codec_ctx_aac);
    av_parser_close(parser_h264);
    avcodec_free_context(&codec_ctx_h264);

    g_player->video_state = 0;
    g_media_decode_exit = 2;
}

int sm_decode_start(media_queue_t     *queue_ptr, int fmt, streaming_callback_t *callbacks)
{
    AVFormatContext *pFormatCtx;
    int             i, videoindex;
    AVCodecContext  *pCodecCtx;
    AVCodec         *pCodec;
    AVFrame         *pFrame, *pFrameYUV;
    uint8_t         *out_buffer;
    int             y_size;
    int             ret, got_picture;
    struct SwsContext *img_convert_ctx;
    int              get_cnt = 0;

    int frame_cnt;

    g_media_decode_exit = 0;
    g_need_clean_buffer = 0;
    g_need_wait_i_frame = 0;
    g_audio_trgger = 0;
    if (g_player)
    {
        LOG_E("mediaplayey in use");
        return RT_EBUSY;
    }
    if (!callbacks || !callbacks->mem_malloc || !callbacks->mem_free)
        return RT_EINVAL;
    g_player = (streaming_handle)rt_malloc(sizeof(streaming_t));
    if (!g_player)
    {
        return RT_ENOMEM;
    }
#ifndef BSP_USING_PC_SIMULATOR
    os_event_create(evt_audio);
#endif
    rt_memset(g_player, 0, sizeof(streaming_t));
    rt_memcpy(&g_player->callbacks, callbacks, sizeof(streaming_callback_t));

    g_player->fmt = fmt;
    g_player->queue_ptr = queue_ptr;

    media_cache_init(&g_player->cache, VIDEO_BUFFER_CAPACITY);


    av_register_all();

    g_player->is_ok = 1;

#if  0
    static struct rt_thread stid;
    rt_err_t err = rt_thread_init(&stid, "media_decode", media_decode_thread_entry, RT_NULL,
                                  ffmpeg_thread_stack, sizeof(ffmpeg_thread_stack),
                                  RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(err == RT_EOK);
    rt_thread_startup(&stid);
#else
    rt_thread_t tid;
    tid = rt_thread_create("media_decode",
                           media_decode_thread_entry,
                           NULL,
                           16 * 1024,
                           RT_THREAD_PRIORITY_MIDDLE,
                           RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(tid);
    rt_thread_startup(tid);
#endif

    return 0;
}

int sm_video_get(uint8_t *data)
{
    int r = RT_EOK;

    if (!g_player)
        r = -RT_EINVAL;
    else if (!g_player->is_ok)
    {
        LOG_E("error in mediaplayer");
        r = -RT_EIO;
    }
    else
        r = media_video_get(&g_player->cache, g_player->fmt, data);
    return r;
}

int sm_decode_dim(int *width, int *height)
{
    int r = RT_EOK;

    if (g_player && g_player->width && g_player->height)
    {
        *width = g_player->width;
        *height = g_player->height;
    }
    else
        r = -RT_ERROR;
    return r;
}

int sm_video_get_period(void)
{
    return 1000 / 15 / 2; //15fps
}

void sm_decode_clean(void)
{
    media_queue_set(g_player->queue_ptr);
    g_need_clean_buffer = 1;
    for (int i = 0; i < 200; i++)
    {
        rt_thread_mdelay(10);
        if (g_need_clean_buffer == 0)
        {
            LOG_D("wait decode ok");
            break;
        }
        LOG_D("wait decode clean");
    }
}

uint64_t sm_cache_unused(void)
{
    return media_bytes_in_queue(g_player->queue_ptr);
}

int sm_decode_stop(void)
{
    if (!g_player)
        return RT_EINVAL;

    g_player->is_ok = 0;
#ifndef BSP_USING_PC_SIMULATOR
    if (evt_audio)
        os_event_flags_set(evt_audio, 1);
#endif
    g_media_decode_exit = 1;
    while (g_player->video_state == 1)
    {
        media_queue_set(g_player->queue_ptr);
        rt_thread_mdelay(10);
        LOG_I("wait video thread exit");
    }

#ifndef BSP_USING_PC_SIMULATOR
    if (audio_handle)
    {
        audio_close(audio_handle);
        audio_handle = NULL;
    }
#endif

    if (g_player)
    {
        if (g_player->mute_frame)
            g_player->callbacks.mem_free(g_player->mute_frame);

        rt_free(g_player);
        g_player = NULL;
    }
    LOG_I("streaming_stop_decode ok");
    ffmpeg_memleak_check();

    return RT_EOK;
}


