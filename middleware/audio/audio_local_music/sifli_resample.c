#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <audio_mem.h>
#include "sifli_resample.h"

sifli_resample_t *sifli_resample_open(uint8_t channels, uint32_t src_samplerate, uint32_t dst_samplerate)
{
    sifli_resample_t *p = (sifli_resample_t *)audio_mem_malloc(sizeof(sifli_resample_t));
    if (p)
    {
        memset(p, 0, sizeof(sifli_resample_t));
        p->src_samplerate = src_samplerate;
        p->channels = channels;
        p->dst_samplerate = dst_samplerate;
        p->ratio = (float)dst_samplerate / (float)src_samplerate;
        p->dst_size = p->ratio * 576 * 2 * sizeof(int16_t) * channels + 100;
        p->dst = (int16_t *)audio_mem_malloc(p->dst_size);
        if (!p->dst)
        {
            audio_mem_free(p);
            return NULL;
        }
    }
    return p;
}

int16_t *sifli_resample_get_output(sifli_resample_t *p)
{
    if (p)
        return p->dst;
    return NULL;
}
void sifli_resample_close(sifli_resample_t *p)
{
    if (p)
    {
        if (p->dst)
        {
            audio_mem_free(p->dst);
        }
        audio_mem_free(p);
    }
}
uint32_t sifli_resample_process(sifli_resample_t *p, int16_t *src, uint32_t src_bytes, uint8_t is_last_packet)
{
    uint32_t new_size =  src_bytes * p->ratio + 100;
    if (new_size > p->dst_size)
    {
        p->dst_size = new_size;
        audio_mem_free(p->dst);
        p->dst = (int16_t *)audio_mem_malloc(new_size);
        RT_ASSERT(p->dst);
    }

    if (p->src_samplerate == p->dst_samplerate)
    {
        memcpy(p->dst, src, src_bytes);
        p->dst_bytes = src_bytes;
        return src_bytes;
    }
    int16_t *dst = p->dst;
    uint32_t samples = src_bytes / sizeof(int16_t) / p->channels;
    uint32_t totla_dst_index = p->total_dst_index;
    int16_t data_left1, data_right1, data_left2, data_right2;
    uint32_t current = 0;
    int32_t  src_index;
    float src_index_float;
    float coef;
    float d1_ratio = 1.0f / p->ratio;
    while (1)
    {
        src_index_float = totla_dst_index * d1_ratio;
        src_index = (int32_t)src_index_float;
        coef = src_index_float - src_index;
        src_index = src_index - p->total_src_index;
        if (src_index + 1 >= samples)
        {
            if (p->channels == 1)
            {
                p->last_left = src[samples - 1];
                p->last_right = p->last_right;
            }
            else
            {
                p->last_left = src[2 * samples - 2];
                p->last_right = src[2 * samples - 1];
            }
            p-> total_src_index += samples;
            p->total_dst_index = totla_dst_index;
            break;
        }
        if (src_index < 0)
        {
            data_left1 = p->last_left;
            data_right1 = p->last_right;
            src_index = 0;
            if (p->channels == 1)
            {
                data_left2 = src[0];
                *dst++ = (1.0f - coef) * data_left1 + coef * data_left2;
            }
            else
            {
                data_left2 = src[0];
                data_right2 = src[1];
                *dst++ = (1.0f - coef) * data_left1 + coef * data_left2;
                *dst++ = (1.0f - coef) * data_right1 + coef * data_right2;
            }

            if (current * 2 >= p->dst_size)
            {
                RT_ASSERT(0);
            }
            current++;
            totla_dst_index++;

        }
        else
        {
            if (p->channels == 1)
            {
                data_left1 = src[src_index];
                data_left2 = src[src_index + 1];
                *dst++ = (1.0f - coef) * data_left1 + coef * data_left2;
            }
            else
            {
                data_left1 = src[src_index * 2];
                data_right1 = src[src_index * 2 + 1];
                data_left2 = src[src_index * 2 + 2];
                data_right2 = src[src_index * 2 + 3];
                *dst++ = (1.0f - coef) * data_left1 + coef * data_left2;
                *dst++ = (1.0f - coef) * data_right1 + coef * data_right2;
            }

            if (current * 2 >= p->dst_size)
            {
                RT_ASSERT(0);
            }
            current++;
            totla_dst_index++;
        }
    }
    if (is_last_packet)
    {
        if (p->channels == 1)
        {
            *dst++ = src[samples - 1];
        }
        else
        {
            *dst++ = src[2 * samples - 2];
            *dst++ = src[2 * samples - 1];
        }
        current++;
    }
    p->dst_bytes = current * sizeof(int16_t) * p->channels;
    return p->dst_bytes;
}
