#include <rtthread.h>
#include "media_dec.h"
#include "media_internal.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#define DBG_TAG           "media_dec"
//#define DBG_LVL           LOG_LVL_INFO
#define _MODULE_NAME_ "h264"
#include "log.h"

bool media_video_need_decode(media_cache_t *cache)
{
    bool ret = true;

    rt_enter_critical();

    if (rt_slist_isempty(&cache->empty_frame_slist))
    {
        ret = false;
    }
    rt_exit_critical();

    return ret;
}

int media_decode_video(ffmpeg_handle thiz,
                       int *got_frame,
                       AVPacket *p_AVPacket)
{
    int ret;
    int is_drop_occured = 0;
    int decoded = -1;
    AVFrame *frame;
    rt_slist_t *empty;
    media_cache_t *cache = &thiz->video_cache;
    AVCodecContext  *video_ctx = thiz->video_dec_ctx;
    //dropping frame must after decoded it
    //next frame may not be I frame, mostly P frame
    rt_enter_critical();

    if (rt_slist_isempty(&cache->empty_frame_slist))
    {
        //drop latest decoded frame, use it to decode again
        is_drop_occured = 1;
        empty = rt_slist_tail(&cache->decoded_frame_slist);
        if (empty)
        {
            //move to empty list, avoid be used to display
            rt_slist_remove(&cache->decoded_frame_slist, empty);
            rt_slist_append(&cache->empty_frame_slist, empty);
        }
    }
    empty = rt_slist_first(&cache->empty_frame_slist);

    rt_exit_critical();

    if (is_drop_occured)
    {
        LOG_I("video drop");
    }

    if (!empty)
    {
        LOG_I("cache error");
        RT_ASSERT(0);
        return -1;
    }

    ms_frame_data_t *one_cache = rt_container_of(empty, ms_frame_data_t, snode);
    frame = (AVFrame *)one_cache->frame;

    TRACE_MARK_START(TRACEID_VIDEO_DECODE);
    ret = avcodec_decode_video2(video_ctx, frame, got_frame, p_AVPacket);
    TRACE_MARK_STOP(TRACEID_VIDEO_DECODE);
    if (ret < 0)
    {
        LOG_E("Error decoding video frame (%s)\n", av_err2str(ret));
    }
    else if (*got_frame)
    {
        decoded = p_AVPacket->size;
        if (video_ctx->width != frame->width || video_ctx->height != frame->height)
        {
            LOG_E("frame size error=%dx%d", frame->width, frame->height);
        }
        else if ((frame->format != AV_PIX_FMT_YUV420P)
                 && (frame->format != AV_PIX_FMT_YUVJ420P)
                 && (frame->format != AV_PIX_FMT_RGB565LE)
                 && (frame->format != AV_PIX_FMT_ARGB)
                 && (frame->format != AV_PIX_FMT_BGR24))
        {
            LOG_E("format unsupport %d\n", frame->format);
        }
        else if (thiz->seeking_state)
        {
            if (frame->pict_type == AV_PICTURE_TYPE_I)
            {
                thiz->seeking_state = 0;
            }
        }
        else
        {
            rt_enter_critical();
            rt_slist_remove(&cache->empty_frame_slist, empty);
            rt_slist_append(&cache->decoded_frame_slist, empty);
            rt_exit_critical();
        }
    }

    return decoded;
}

int media_audio_len(AVFrame *frame)
{
    return (frame->nb_samples * frame->channels * sizeof(uint16_t));
}

int media_audio_get(AVFrame *frame, uint16_t *audio_data, uint32_t audio_data_size)
{
    int size = frame->nb_samples * frame->channels * sizeof(uint16_t);
    RT_ASSERT(size <= audio_data_size);

    if (audio_data)
    {
        /* Write the raw audio data samples of the first plane. This works
         * fine for packed formats (e.g. AV_SAMPLE_FMT_S16). However,
         * most audio decoders output planar audio, which uses a separate
         * plane of audio samples for each channel (e.g. AV_SAMPLE_FMT_S16P).
         * In other words, this code will write only the first audio channel
         * in these cases.
         * You should use libswresample or libavfilter to convert the frame
         * to packed data. */
        if (frame->format == AV_SAMPLE_FMT_S16P)
        {
            uint16_t *l = (uint16_t *)frame->extended_data[0];
            uint16_t *r = (uint16_t *)frame->extended_data[1];
            if (frame->channels == 2)
            {
                for (int j = 0; j < frame->nb_samples; j++)
                {
                    audio_data[2 * j] = (int16_t)(l[j]);
                    audio_data[2 * j + 1] = (int16_t)(r[j]);
                }
            }
            else
            {
                memcpy(audio_data, l, frame->nb_samples * 2);
            }
        }
        if (frame->format == AV_SAMPLE_FMT_S32P)
        {
            int32_t *l = (int32_t *)frame->extended_data[0];
            int32_t *r = (int32_t *)frame->extended_data[1];
            if (frame->channels  == 2)
            {
                for (int j = 0; j < frame->nb_samples; j++)
                {
                    audio_data[2 * j] = (int16_t)(l[j] >> 16);
                    audio_data[2 * j + 1] = (int16_t)(r[j] >> 16);
                }
            }
            else
            {
                memcpy(audio_data, l, frame->nb_samples * 2);
            }
        }
        else if (frame->format == AV_SAMPLE_FMT_FLTP)
        {
            float *fl = (float *)frame->extended_data[0];
            float *fr = (float *)frame->extended_data[1];
            float t;
            if (frame->channels  == 2)
            {
                for (int j = 0; j < frame->nb_samples; j++)
                {
                    t = fl[j];
                    if (t >= +0x999999f)
                    {
                        audio_data[2 * j] = 0x7FFF;
                    }
                    else if (t <= -0x999999f)
                    {
                        audio_data[2 * j] = 0x8000;
                    }
                    else
                    {
                        audio_data[2 * j] = (int16_t)(t * 32767.0f);
                    }

                    t = fr[j];

                    if (t >= +0x999999f)
                    {
                        audio_data[2 * j + 1] = 0x7FFF;
                    }
                    else if (t <= -0x999999f)
                    {
                        audio_data[2 * j + 1] = 0x8000;
                    }
                    else
                    {
                        audio_data[2 * j + 1] = (int16_t)(t * 32767.0f);
                    }
                }
            }
            else
            {
                for (int j = 0; j < frame->nb_samples; j++)
                {
                    t = fl[j];
                    if (t >= +0x999999f)
                    {
                        audio_data[j] = 0x7FFF;
                    }
                    else if (t <= -0x999999f)
                    {
                        audio_data[j] = 0x8000;
                    }
                    else
                    {
                        audio_data[j] = (int16_t)(t * 32767.0f);
                    }
                }
            }
        }
        else
        {
            LOG_E("unknow audio fmt\n");
        }
    }
    return size;
}

int ezip_video_cache_init(ffmpeg_handle thiz)
{
    memset(&thiz->ezip_video_cache, 0, sizeof(thiz->ezip_video_cache));
    rt_slist_init(&thiz->ezip_video_cache.decoded_video_slist);
    rt_slist_init(&thiz->ezip_video_cache.display_video_slist);
    rt_slist_init(&thiz->ezip_video_cache.empty_video_slist);

    for (int i = 0; i < VIDEO_BUFFER_CAPACITY; i++)
    {
        if (2 == thiz->is_sifli_ezip_memdia && thiz->ezip_header.max_frame_size > 0)
        {
            uint8_t *p;
            thiz->ezip_video_cache.cache[i].buffer_size = thiz->ezip_header.max_frame_size;
            p = thiz->cfg.mem_malloc(thiz->ezip_header.max_frame_size);
            thiz->ezip_video_cache.cache[i].buffer = p;
            RT_ASSERT(p);
        }
        rt_slist_append(&thiz->ezip_video_cache.empty_video_slist, &thiz->ezip_video_cache.cache[i].snode);
    }
    return 0;
}

void ezip_video_cache_deinit(ffmpeg_handle thiz)
{

    for (int i = 0; i < VIDEO_BUFFER_CAPACITY; i++)
    {
        if (thiz->ezip_video_cache.cache[i].buffer)
        {
            thiz->cfg.mem_free(thiz->ezip_video_cache.cache[i].buffer);
            thiz->ezip_video_cache.cache[i].buffer = NULL;
        }
    }
}

bool ezip_video_need_decode(ffmpeg_handle thiz)
{
    bool ret = true;

    rt_enter_critical();

    if (rt_slist_isempty(&thiz->ezip_video_cache.empty_video_slist))
    {
        ret = false;
    }
    rt_exit_critical();

    return ret;
}

int ezip_video_decode(ffmpeg_handle thiz,        uint32_t size, uint32_t paddings)
{
    rt_slist_t *decoded_root;
    rt_slist_t *empty_root;
    rt_slist_t *empty;
    bool is_drop = false;
    empty_root = &thiz->ezip_video_cache.empty_video_slist;
    decoded_root = &thiz->ezip_video_cache.decoded_video_slist;

    rt_enter_critical();

    empty = rt_slist_first(empty_root);
    if (empty)
    {
        rt_slist_remove(empty_root, empty);
    }
    else
    {
        is_drop = true;
        empty = rt_slist_tail(decoded_root);
        RT_ASSERT(empty);
        rt_slist_remove(decoded_root, empty);
    }

    rt_exit_critical();

    if (is_drop)
    {
        LOG_I("ezip video drop");
    }
    if (empty)
    {
        ezip_video_packet_t *pkt = rt_container_of(empty, ezip_video_packet_t, snode);
        if (pkt->buffer_size < size)
        {
            if (pkt->buffer)
                thiz->cfg.mem_free(pkt->buffer);
            pkt->buffer_size = size;
            pkt->buffer = thiz->cfg.mem_malloc(size);
            RT_ASSERT(pkt->buffer);
        }
        if (thiz->is_nand)
        {
            ezip_flash_read(thiz, pkt->buffer, size);
        }
        else
        {
            read(thiz->ezip_fd, pkt->buffer, size);
        }
        pkt->data_len = size - paddings;
        rt_enter_critical();
        rt_slist_append(decoded_root, empty);
        rt_exit_critical();
        return 0;
    }
    else
    {
        RT_ASSERT(0);
    }
    return -1;
}
int media_cache_init(media_cache_t *cache,      int cache_num)
{
    rt_slist_init(&cache->decoded_frame_slist);
    rt_slist_init(&cache->display_frame_slist);
    rt_slist_init(&cache->empty_frame_slist);
    RT_ASSERT(cache_num <= VIDEO_BUFFER_CAPACITY);
    for (int i = 0; i < cache_num; i++)
    {
        AVFrame *frame;
        frame = av_frame_alloc();
        if (frame == NULL)
        {
            LOG_E("video frame cache %d no mem", i);
            return -RT_ENOMEM;
        }
        cache->video_cache[i].frame = frame;
        rt_slist_append(&cache->empty_frame_slist, &cache->video_cache[i].snode);
    }
    return RT_EOK;
}

void media_cache_deinit(media_cache_t *cache, int cache_num)
{
    AVFrame *frame;
    RT_ASSERT(cache_num <= VIDEO_BUFFER_CAPACITY);
    for (int i = 0; i < cache_num; i++)
    {
        if (cache->video_cache[i].frame)
            av_frame_free((AVFrame **) & cache->video_cache[i].frame);
    }
}

int media_video_convert(uint8_t *buf, AVFrame *frame, int fmt)
{
    int r = RT_EOK;

    if (buf == NULL || frame == NULL)
    {
        r = - RT_EINVAL;
        return r;
    }
    if (fmt == IMG_DESC_FMT_EZIP)
    {
        uint8_t **frame_data = (uint8_t **)buf;
        int *p_size = (int *)frame->data[0];
        frame_data[0] = frame->data[0] + sizeof(int);
        frame_data[1] = (uint8_t *)(*p_size);
        frame_data[2] = (uint8_t *)IMG_DESC_FMT_EZIP;
        return RT_EOK;
    }

    if ((AV_PIX_FMT_YUV420P == frame->format)
            || (AV_PIX_FMT_YUVJ420P == frame->format))
    {
        if (frame->data[0] == NULL || frame->data[1] == NULL || frame->data[2] == NULL)
            r = -RT_EEMPTY;
        else if (fmt == IMG_DESC_FMT_RGB565)
        {
            yuv420_2_rgb565(buf,
                            (const uint8_t *)frame->data[0], //y
                            (const uint8_t *)frame->data[1], //u
                            (const uint8_t *)frame->data[2], //v
                            frame->width,
                            frame->height,
                            frame->linesize[0],
                            frame->linesize[0] >> 1,
                            frame->width << 1,
                            yuv2rgb565_table,
                            0);

        }
        else if (fmt == IMG_DESC_FMT_RGB888) //rgb888
        {
            yuv420_2_rgb888(buf,
                            (const uint8_t *)frame->data[0], //y
                            (const uint8_t *)frame->data[1], //u
                            (const uint8_t *)frame->data[2], //v
                            frame->width,
                            frame->height,
                            frame->linesize[0],
                            frame->linesize[0] >> 1,
                            (frame->width << 1) + frame->width,
                            yuv2rgb565_table,       // TODO: Check this
                            0);
        }
        else if (fmt == IMG_DESC_FMT_ARGB8888) //argb8888
        {
            yuv420_2_rgb8888(buf,
                             (const uint8_t *)frame->data[0], //y
                             (const uint8_t *)frame->data[1], //u
                             (const uint8_t *)frame->data[2], //v
                             frame->width,
                             frame->height,
                             frame->linesize[0],
                             frame->linesize[0] >> 1,
                             frame->width << 2,
                             yuv2rgb565_table,       // TODO: Check this
                             0);
        }
        else    //fmt==IMG_DESC_FMT_YUV420P
        {
            uint8_t **data = (uint8_t **)buf;
            data[0] = frame->data[0];
            data[1] = frame->data[1];
            data[2] = frame->data[2];
        }
    }
    else if (((AV_PIX_FMT_RGB565LE == frame->format) && (fmt == IMG_DESC_FMT_RGB565))
             || ((AV_PIX_FMT_ARGB == frame->format) && (fmt == IMG_DESC_FMT_ARGB8888)))
    {
        uint8_t **data = (uint8_t **)buf;
        data[0] = frame->data[0];
    }
    return r;
}


int media_video_get(media_cache_t *cache, int fmt, uint8_t *data, uint8_t is_ezip)
{
    uint32_t current, last;

    if (!cache || !data)
        return -RT_EINVAL;

    AVFrame *frame;
    rt_slist_t *decoded;
    rt_slist_t *display;

    rt_enter_critical();
    decoded = rt_slist_first(&cache->decoded_frame_slist);
    if (decoded)
    {
        display = rt_slist_first(&cache->display_frame_slist);
        if (display) //move display to empty, only one in display list
        {
            rt_slist_remove(&cache->display_frame_slist, display);
            rt_slist_append(&cache->empty_frame_slist, display);
        }
        //move decoded to display
        rt_slist_remove(&cache->decoded_frame_slist, decoded);
        rt_slist_append(&cache->display_frame_slist, decoded);
    }

    rt_exit_critical();

    if (!decoded)
    {
        LOG_D("video get empty");
        return -RT_EEMPTY;;
    }

    ms_frame_data_t *one_cache = rt_container_of(decoded, ms_frame_data_t, snode);
    frame = (AVFrame *)one_cache->frame;

    if (is_ezip)
        media_video_convert(data, frame, IMG_DESC_FMT_EZIP);
    else
        media_video_convert(data, frame, fmt);

    return 0;
}

