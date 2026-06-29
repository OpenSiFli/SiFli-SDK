/*
 * Copyright (c) 2022-2024 Nordic Semiconductor ASA
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bap_broadcast_src_api.h"
#include <audio_server.h>
#include "sifli_resample.h"

#define MIC_SAMPLE_RATE         16000
#define BROADCAST_SAMPLE_RATE   48000
#define MIC_CHANNELS            1
#define MIC_BITS_PER_SAMPLE     16
#define MIC_READ_CACHE_SIZE     2048

static audio_client_t g_mic_client;
static sifli_resample_t *g_resample;

static int mic_callback(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    uint32_t out_bytes;

    if (cmd != as_callback_cmd_data_coming)
    {
        return 0;
    }

    if (!bap_broadcast_src_is_busy())
    {
        return 0;
    }

    //rt_kprintf("mic_callback into ---\n");

    audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
    out_bytes = sifli_resample_process(g_resample, (int16_t *)p->data, p->data_len, 0);
    if (out_bytes >= SPEAKER_10MS_DMA_SIZE)
    {
        ble_src_send((uint8_t *)sifli_resample_get_output(g_resample), SPEAKER_10MS_DMA_SIZE);
    }

    return 0;
}

static void mic_start(void)
{
    audio_parameter_t param = {0};

    if (g_mic_client)
    {
        return;
    }

    g_resample = sifli_resample_open(MIC_CHANNELS, MIC_SAMPLE_RATE, BROADCAST_SAMPLE_RATE);
    RT_ASSERT(g_resample);

    param.read_bits_per_sample = MIC_BITS_PER_SAMPLE;
    param.read_channnel_num = MIC_CHANNELS;
    param.read_samplerate = MIC_SAMPLE_RATE;
    param.read_cache_size = MIC_READ_CACHE_SIZE;

    g_mic_client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &param, mic_callback, NULL);
    RT_ASSERT(g_mic_client);
    printk("mic recording started\n");
}

static void mic_stop(void)
{
    if (g_mic_client)
    {
        audio_close(g_mic_client);
        g_mic_client = NULL;
        printk("mic recording stopped\n");
    }

    if (g_resample)
    {
        sifli_resample_close(g_resample);
        g_resample = NULL;
    }
}

int main(void)
{
    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    for (int i = 0; i < 5; i++)
    {
        rt_thread_mdelay(1000);
    }
    printk("Bluetooth initialized\n");
    bap_broadcast_src_start();
    printk("\r\ninput audio_src 0 to stop\n\n");
    mic_start();
    return 0;
}

__ROM_USED void audio_src(int argc, char **argv)
{
    if (argc < 2)
    {
        return;
    }

    if (argv[1][0] == '0')
    {
        mic_stop();
        bap_broadcast_src_stop();
        printk("\r\ninput audio_src 1 to start\n\n");
    }
    else
    {
        bap_broadcast_src_start();
        mic_start();
        printk("\r\ninput audio_src 0 to stop\n\n");
    }
}
MSH_CMD_EXPORT(audio_src, audio_src command)
