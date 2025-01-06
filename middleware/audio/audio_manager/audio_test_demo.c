/**
  ******************************************************************************
  * @file   audio_test_demo.c
  * @author Sifli software development team
  * @brief SIFLI audio test demo.
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
#include "audio_server.h"

#define DBG_TAG           "audio"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

#define MIC_RECORD_FILE "/mic16k.pcm"
//#define MIC_RECORD_FILE "/ramfs/mic16k.pcm"  //using ramfs if mounted ramfs

#if 0

static uint16_t *pcm;
static audio_client_t g_client;
static int cache_full;
static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        auido_gain_pcm((int16_t *)p->data, p->data_len, 4); //pcm data left shift 4 bits
        write(fd, p->data, p->data_len);
    }
    return 0;
}
static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (fd >= 0 && pcm && g_client)
        {
            read(fd, (void *)pcm, 2048);
            int writted = audio_write(g_client, (uint8_t *)pcm, 2048);
            if (writted == 0)
            {
                cache_full = 1;
            }
        }
    }
    return 0;
}
static void mic2file(uint8_t argc, char **argv)
{
    int fd;
    uint32_t record_seconds = 0;
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.read_cache_size = 2048;
    pa.write_cache_size = 2048;
    pcm = NULL;
    cache_full = 0;
    pcm = malloc(4096);
    RT_ASSERT(pcm);
    fd = open(MIC_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(fd >= 0);

    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, audio_callback_record, (void *)fd);
    RT_ASSERT(client);

    while (record_seconds < 5)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(client);
    close(fd);



    //play now
    pa.write_cache_size = 4096;
    fd = open(MIC_RECORD_FILE, O_RDONLY | O_BINARY);
    RT_ASSERT(fd >= 0);

    g_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &pa, audio_callback_play, (void *)fd);
    RT_ASSERT(g_client >= 0);
    read(fd, (void *)pcm, 4096);
    audio_write(g_client, (uint8_t *)pcm, 4096);
    record_seconds = 0;
    while (record_seconds < 5)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }

    audio_close(g_client);
    close(fd);
    unlink(MIC_RECORD_FILE);
    free(pcm);
}

MSH_CMD_EXPORT(mic2file, mic2file test);


static uint8_t g_rx_bitmap;
static uint8_t g_tx_bitmap;
static uint8_t *g_one_channel;
static uint8_t g_drop_cnt;
static int audio_loopback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    uint32_t tx_bitmap = (uint32_t)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        if (g_drop_cnt > 0)
        {
            g_drop_cnt--;
            return 0;
        }
        if (p->reserved == 0) //audprc data coming
        {
            //LOG_I("audprc data coming tx=%d", g_tx_bitmap);
            if (g_tx_bitmap == 1)
            {
                auido_gain_pcm((int16_t *)p->data, p->data_len, 2); //pcm data left shift 4 bits
                audio_write(g_client, (uint8_t *)p->data, p->data_len);
            }
        }
        else if ((g_rx_bitmap & 0x06) == 2)
        {
            //LOG_I("pdm left data coming tx=%d", g_tx_bitmap);
            if (g_tx_bitmap == 2)
            {
                //auido_gain_pcm((int16_t *)p->data, p->data_len, 1); //pcm data left shift 4 bits
                audio_write(g_client, (uint8_t *)p->data, p->data_len);
            }
        }
        else if ((g_rx_bitmap & 0x06) == 4)
        {
            //LOG_I("pdm right data coming tx=%d", g_tx_bitmap);
            if (g_tx_bitmap == 4)
            {
                //auido_gain_pcm((int16_t *)p->data, p->data_len, 1); //pcm data left shift 4 bits
                audio_write(g_client, (uint8_t *)p->data, p->data_len);
            }
        }
        else if ((g_rx_bitmap & 0x06) == 6)
        {
            //LOG_I("pdm left & right data coming tx=%d", g_tx_bitmap);
            if (g_tx_bitmap == 6)
            {
                //auido_gain_pcm((int16_t *)p->data, p->data_len, 1); //pcm data left shift 4 bits
                audio_write(g_client, (uint8_t *)p->data, p->data_len);
            }
            else if (g_tx_bitmap == 2 || g_tx_bitmap == 4)
            {
                //auido_gain_pcm((int16_t *)p->data, p->data_len, 1); //pcm data left shift 4 bits
                if (!g_one_channel)
                {
                    g_one_channel = malloc(p->data_len / 2);
                    RT_ASSERT(g_one_channel);
                }
                const int16_t *src = (const int16_t *)p->data;
                int16_t *dst = (int16_t *)g_one_channel;
                int i = 0;
                if (g_tx_bitmap == 2)
                {
                    i = 0;
                    //LOG_I("send left mic data");
                }
                else
                {
                    i = 1;
                    //LOG_I("send right mic data");
                }

                for (; i < p->data_len / 2; i += 2)
                {
                    *dst++ = src[i];
                }
                audio_write(g_client, (uint8_t *)g_one_channel, p->data_len / 2);
            }
        }
    }
    return 0;
}
/*
 mic2speaker <mic devices id bit map> <which id tx to speaker>
 mic devices id bit map
    1 -- audprc
    2 -- pdm left
    4 -- pdm left & right

example:
 mic2speaker 1  1   //rx audprc and tx to speaker
 mic2speaker 2  2   //rx pdm left and tx to speaker
 mic2speaker 6  6   //rx pdm left/right dual channel and tx to speaker
 mic2speaker 7  1   //rx pdm left/right dual channel and audprc tx to speaker
 mic2speaker 6  2   //rx pdm left/right dual channel and left tx to speaker
 mic2speaker 6  4   //rx pdm left/right dual channel and right tx to speaker
*/
static void mic2speaker(uint8_t argc, char **argv)
{
    g_drop_cnt = 100;
    g_one_channel = NULL;
    g_tx_bitmap = 1; //default tx audprc to speaker
    if (argc < 3)
    {
        return;
    }
    g_rx_bitmap = atoi(argv[1]);
    g_tx_bitmap = atoi(argv[2]);

    if (g_rx_bitmap == 4)
    {
        //i2s driver not support rx right only
        g_rx_bitmap = 6;
    }

    audio_parameter_t pa = {0};
    pa.codec = 0xFF; // 3 mic

    //hight 4 bits is rx bitmap
    pa.tsco  = g_rx_bitmap & 0x07;
    pa.write_bits_per_sample = 16;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.write_cache_size = 4096;

    if (g_tx_bitmap == 6)
    {

        pa.write_channnel_num = 2; //audprc tx using dual channel
    }
    else
    {
        pa.write_channnel_num = 1; //audprc tx using one channel
    }

    g_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TXRX, &pa, audio_loopback_play, (void *)NULL);
    RT_ASSERT(g_client);

    for (int i = 0; i < 10; i++)
    {
        rt_thread_mdelay(1 * 1000);
    }

    audio_close(g_client);

    if (g_one_channel)
        free(g_one_channel);
}
MSH_CMD_EXPORT(mic2speaker, mic2speaker);

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
