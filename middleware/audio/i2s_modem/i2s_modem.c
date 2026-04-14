/*
 * SPDX-FileCopyrightText: 2022-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <rtdevice.h>
#include <drivers/audio.h>
#include <drv_i2s_audio.h>
#include <drv_audprc.h>

#include "audio_server.h"
#include "i2s_modem.h"

#define DBG_TAG  "mdm"
#include <rtdbg.h>


#define MODEM_I2S_DEBUG             0
#define MODEM_I2S_DEVICE_MODE       1 /* 1: modem i2s is master; 0: modem i2s is slave */
#define MODEM_I2S_DEVICE_CHANNELS   1
#define MODEM_I2S_DEVICE_NAME       "i2s2"

#define SWITCH_BUFFER_SIZE          (320 * 3)

#define MODEM_EVENT_I2S_RX          (1 << 0)
#define MODEM_EVENT_I2S_TX          (1 << 1)
#define MODEM_EVENT_AUDPRC_RX       (1 << 2)
#define MODEM_EVENT_AUDPRC_TX       (1 << 3)
#define MODEM_EVENT_EXIT            (1 << 4)

#define MODEM_EVENT_ALL ( \
                            MODEM_EVENT_I2S_RX | \
                            MODEM_EVENT_AUDPRC_RX | \
                            MODEM_EVENT_AUDPRC_RX | \
                            MODEM_EVENT_AUDPRC_TX | \
                            MODEM_EVENT_EXIT | \
                            0 \
                            )

static struct rt_event dal_modem_voice_event;



struct modem_server
{
    rt_device_t     i2s_dev;
    audio_client_t  client;
    int16_t         stereo[320];
    int16_t         mono[160];
    uint8_t         is_opened;
    uint8_t         exit;

#if MODEM_I2S_DEBUG
    rt_event_t event;
    rt_uint8_t audprc_2_i2s_cache[SWITCH_BUFFER_SIZE];
    rt_uint8_t i2s_2_audprc_cache[SWITCH_BUFFER_SIZE];
    struct rt_ringbuffer audprc2i2s_rb;
    struct rt_ringbuffer i2s2audprc_rb;
#endif
};

struct modem_server modem;


static int i2s_open(struct modem_server *thiz);
static int i2s_close(struct modem_server *thiz);

static inline void mono2stereo(int16_t *mono, uint32_t samples, int16_t *stereo)
{
    for (int i = 0; i < samples; i++)
    {
        *stereo++ = *mono;
        *stereo++ = *mono++;
    }
}

static inline void stereo2mono(int16_t *stereo, uint32_t samples, int16_t *mono)
{
    samples >>= 1;
    for (int i = 0; i < samples; i++)
    {
        *mono++ = *stereo++;
        stereo++;
    }
}

#if MODEM_I2S_DEBUG
static void modem_entry(void *parameter)
{
    struct modem_server *thiz = &modem;
    while (thiz->exit)
    {
        rt_uint32_t evt;
        if (rt_event_recv(thiz->event, MODEM_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &evt) == RT_EOK)
        {
            if (evt & MODEM_EVENT_EXIT)
            {
                break;
            }

            if (evt & MODEM_EVENT_I2S_RX)
            {
                int16_t debug[160];
                rt_ringbuffer_get(&thiz->i2s2audprc_rb, (uint8_t *)debug, sizeof(debug));
                LOG_HEX("i2s rx:\n", 16, debug, sizeof(debug));
            }
        }
    }
}
#endif

static void audprc_dma_rx(uint8_t channel_id, uint8_t *data, rt_size_t len)
{
    struct modem_server *thiz = &modem;

    RT_ASSERT(320 == len);
    if (channel_id > 0)
    {
        return;
    }

    if (thiz->is_opened && thiz->i2s_dev)
    {
#if MODEM_I2S_DEVICE_CHANNELS == 2
        mono2stereo((int16_t *)data, len / 2, thiz->stereo);
#if MODEM_I2S_DEBUG
        rt_ringbuffer_put(&thiz->audprc2i2s_rb, (uint8_t *)thiz->stereo, sizeof(thiz->stereo));
        rt_event_send(thiz->event, MODEM_EVENT_AUDPRC_RX);
#else
        bf0_i2s_device_write(thiz->i2s_dev, 0, (uint8_t *)thiz->stereo, sizeof(thiz->stereo));
#endif
#else
#if MODEM_I2S_DEBUG
        rt_ringbuffer_put(&thiz->audprc2i2s_rb, data, len);
        rt_event_send(thiz->event, MODEM_EVENT_AUDPRC_RX);
#else
        bf0_i2s_device_write(thiz->i2s_dev, 0, data, len);
#endif
#endif
    }
}

int i2s_modem_open(void)
{
    struct modem_server *thiz = &modem;

    //audio_server_set_private_volume(AUDIO_TYPE_MODEM_VOICE, 15);
    if (thiz->is_opened)
    {
        LOG_E("%s reopen err", __func__);
        return -2;
    }

#if MODEM_I2S_DEBUG
    thiz->exit = 0;
    thiz->event = rt_event_create("mdm_evt", RT_IPC_FLAG_FIFO);
    rt_ringbuffer_init(&thiz->audprc2i2s_rb, thiz->audprc_2_i2s_cache, sizeof(thiz->audprc_2_i2s_cache));
    rt_ringbuffer_init(&thiz->i2s2audprc_rb, thiz->i2s_2_audprc_cache, sizeof(thiz->i2s_2_audprc_cache));
    rt_thread_t tid;
    tid = rt_thread_create("mdmi2st", modem_entry, NULL, 4096, 16, 10);
    rt_thread_startup(tid);
#endif
    if (i2s_open(thiz))
    {
        LOG_E("%s i2s_open error", __func__);
        return -2;
    }

    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.read_cache_size = 0;
    pa.write_cache_size = 640;
    thiz->client = audio_open(AUDIO_TYPE_MODEM_VOICE, AUDIO_TXRX, &pa, NULL, NULL);
    RT_ASSERT(thiz->client);
    rt_device_set_audprc_dma_rx_callback(audprc_dma_rx);

    rt_base_t level = rt_hw_interrupt_disable();
    thiz->is_opened = 1;
    rt_hw_interrupt_enable(level);

    LOG_E("%s", __func__);

    return 0;
}


int i2s_modem_close(void)
{
    struct modem_server *thiz = &modem;

    rt_base_t level = rt_hw_interrupt_disable();
    thiz->is_opened = 0;
    rt_hw_interrupt_enable(level);

    i2s_close(thiz);
    audio_close(thiz->client);
    rt_device_set_audprc_dma_rx_callback(NULL);
    thiz->client = NULL;

#if MODEM_I2S_DEBUG
    thiz->exit = 1;
    rt_event_send(thiz->event, MODEM_EVENT_EXIT);
    while (rt_thread_find("mdmi2st"))
    {
        rt_thread_mdelay(10);
        LOG_I("wait modem exit");
    }
    rt_event_delete(thiz->event);
    thiz->event = NULL;
#endif

    LOG_E("%s", __func__);

    return 0;
}

static void i2s_dma_rx(char *name, uint8_t *data, rt_size_t len)
{
    struct modem_server *thiz = &modem;

    RT_ASSERT(MODEM_I2S_DEVICE_CHANNELS * 320 == len);

    if (thiz->is_opened && audio_get_audprc_dev())
    {
#if MODEM_I2S_DEVICE_CHANNELS == 2
        stereo2mono((int16_t *)data, len / 2, thiz->mono);
#if MODEM_I2S_DEBUG
        rt_ringbuffer_put(&thiz->i2s2audprc_rb, (uint8_t *)thiz->mono, sizeof(thiz->mono));
        rt_event_send(thiz->event, MODEM_EVENT_I2S_RX);
#else
        bf0_audprc_device_write(audio_get_audprc_dev(), 0, (uint8_t *)thiz->mono, sizeof(thiz->mono));
#endif
#else
#if MODEM_I2S_DEBUG
        rt_ringbuffer_put(&thiz->i2s2audprc_rb, data, len);
        rt_event_send(thiz->event, MODEM_EVENT_I2S_RX);
#else
        bf0_audprc_device_write(audio_get_audprc_dev(), 0, data, len);
#endif
#endif
    }
}

static void i2s_modem_start(void)
{
    struct modem_server *thiz = &modem;
    if (thiz->i2s_dev)
    {
        int stream = AUDIO_STREAM_REPLAY;
        rt_device_control(thiz->i2s_dev, AUDIO_CTL_START, &stream);
        stream = AUDIO_STREAM_RECORD;
        rt_device_control(thiz->i2s_dev, AUDIO_CTL_START, &stream);
    }
}

static int i2s_open(struct modem_server *thiz)
{
    int stream = 0;

    thiz->i2s_dev = rt_device_find(MODEM_I2S_DEVICE_NAME);
    if (NULL == thiz->i2s_dev)
    {
        LOG_E("%s not find %s", __func__, MODEM_I2S_DEVICE_NAME);
        return -1;
    }

    if (RT_EOK != rt_device_open(thiz->i2s_dev, RT_DEVICE_OFLAG_RDWR))
    {
        LOG_E("%s Fail to open i2s", __func__);
        return -2;
    }

    //config audio arg
    struct rt_audio_caps caps =
    {
        .main_type = AUDIO_TYPE_INPUT,
        .sub_type = AUDIO_DSP_PARAM,
        .udata.config.channels = MODEM_I2S_DEVICE_CHANNELS,
        .udata.config.samplefmt = 16,
        .udata.config.samplerate = 16000,
    };

    if (RT_EOK != rt_device_control(thiz->i2s_dev, AUDIO_CTL_CONFIGURE, &caps))
    {
        LOG_E("%s fmt err", __func__);
        return -3;
    }

    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = AUDIO_DSP_MODE;
    caps.udata.value = MODEM_I2S_DEVICE_MODE;
    rt_device_control(thiz->i2s_dev, AUDIO_CTL_CONFIGURE, &caps);

    int inter = 0; // 0:dma 1:audprc
    rt_device_control(thiz->i2s_dev, AUDIO_CTL_SETINPUT, (void *)inter);
    inter = 0;
    rt_device_control(thiz->i2s_dev, AUDIO_CTL_SETOUTPUT, (void *)inter);

    rt_device_set_i2s_dma_rx_callback(i2s_dma_rx);

    i2s_modem_start();

    return 0;
}

static int i2s_close(struct modem_server *thiz)
{
    rt_device_set_i2s_dma_rx_callback(NULL);
    if (thiz->i2s_dev)
    {
        int stream = AUDIO_STREAM_REPLAY;
        rt_device_control(thiz->i2s_dev, AUDIO_CTL_STOP, &stream);
        stream = AUDIO_STREAM_RECORD;
        rt_device_control(thiz->i2s_dev, AUDIO_CTL_STOP, &stream);

        rt_device_close(thiz->i2s_dev);
        thiz->i2s_dev = NULL;
        rt_device_set_i2s_dma_rx_callback(NULL);
    }
    return 0;
}

