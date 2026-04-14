/*
 * SPDX-FileCopyrightText: 2022-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "audio_server.h"
#include "audio_mp3ctrl.h"
#include "drv_flash.h"

/* Common functions for RT-Thread based platform -----------------------------------------------*/

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif

#define FS_ROOT "root"

/**
 * @brief Mount fs.
 */
static int mnt_init(void)
{
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, FS_ROOT);
    if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)//Format file system
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}


#define MUSIC_FILE_PATH "/stereo.wav"

static mp3ctrl_handle g_mp3_handle = NULL;
static int g_is_end = 0;

/**
 * @brief callback function for mp3ctrl_open.
 */
static int play_callback_func(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    rt_kprintf("[LOCAL MUSIC]%s cmd %d\n", __func__, cmd);
    switch (cmd)
    {
    case as_callback_cmd_play_to_end:
        g_is_end = 1;
        break;

    default:
        break;
    }

    return 0;
}

#define MIC0_RECORD_FILE "/mic0_16k.pcm"
#define MIC1_RECORD_FILE "/mic1_16k.pcm"

#define RECORD_MEM_SIZE 64000
static int fd0;
static int fd1;
static int adc0_offset = 0;
static int adc1_offset = 0;
static uint8_t adc0_mem[RECORD_MEM_SIZE];
static uint8_t adc1_mem[RECORD_MEM_SIZE];

static int mic2speaker_callback(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    audio_client_t client = *((audio_client_t *)callback_userdata);
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        audio_write(client, (uint8_t *)p->data, p->data_len);
    }
    return 0;
}
static void mic2speaker(uint8_t argc, char **argv)
{
    uint32_t record_seconds = 0;
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    /* demo for signel mic and sigle i2s channel */
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.read_cache_size = 0;
    pa.read_which_mic = AUDIO_MIC0_ONLY; // AUDIO_MIC1_ONLY;
    pa.write_cache_size = 2048;

    /*
      client must set to null before audio_open(),
      mic2speaker_callback() may be called before audio_open() return,
      and in mic2speaker_callback() will call audio_write(client).
      audio_write(client) can using null client
     */

    audio_client_t client = NULL;

    client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TXRX, &pa, mic2speaker_callback, &client);
    RT_ASSERT(client);

    while (record_seconds < 10)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(client);
}

MSH_CMD_EXPORT(mic2speaker, mic2speaker test);

static int16_t *pcm1;
static int16_t *pcm2;
static int16_t *pcm_mix;

static audio_client_t g_client;
static int cache_full;

static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        rt_kprintf("%s adc=%d\n", __func__, p->reserved, p->data_len);
        if (p->reserved == 0)
        {
            //mic 0 or mic 1 at single mic mode, or mic 0 at dual mic mode
            if (adc0_offset + p->data_len < RECORD_MEM_SIZE)
            {
                memcpy(&adc0_mem[adc0_offset], p->data, p->data_len);
                adc0_offset += p->data_len;
            }
        }
        else
        {
            //mic 1 at dual mic mode
            if (adc0_offset + p->data_len < RECORD_MEM_SIZE)
            {
                memcpy(&adc1_mem[adc1_offset], p->data, p->data_len);
                adc1_offset += p->data_len;
            }
        }
    }
    return 0;
}

static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (fd0 >= 0 && fd1 >= 0 && pcm1 && pcm2 && g_client)
        {
            if (!cache_full)
            {
                read(fd0, (void *)pcm1, 1024);
                read(fd1, (void *)pcm2, 1024);
                for (int i = 0; i < 1024 / 2; i++)
                {
                    pcm_mix[i * 2] = pcm1[i];
                    pcm_mix[i * 2 + 1] = pcm2[i];
                }
            }
            int writted = audio_write(g_client, (uint8_t *)pcm_mix, 2048);
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
    uint32_t record_seconds = 0;
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;

    /* the demo only for dual mic channel */
    pa.write_channnel_num = 2;
    pa.write_samplerate = 16000;
    pa.write_cache_size = 0;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.read_cache_size = 0;
    pa.read_which_mic = AUDIO_MIC_ALL; // use dual mic
    pcm1 = NULL;
    pcm2 = NULL;
    cache_full = 0;
    pcm1 = malloc(4096);
    RT_ASSERT(pcm1);
    pcm2 = malloc(4096);
    RT_ASSERT(pcm2);
    pcm_mix = malloc(8192);
    RT_ASSERT(pcm_mix);

    adc0_offset = 0;
    adc1_offset = 0;

    fd0 = open(MIC0_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    fd1 = open(MIC1_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(fd0 >= 0 && fd1 >= 0);
    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, audio_callback_record, NULL);
    RT_ASSERT(client);

    while (record_seconds < 10)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(client);

    write(fd0, adc0_mem, RECORD_MEM_SIZE);
    write(fd1, adc1_mem, RECORD_MEM_SIZE);

    close(fd0);
    close(fd1);


    //play now
    pa.write_cache_size = 4096;
    pa.write_channnel_num = 2;

    fd0 = open(MIC0_RECORD_FILE, O_RDONLY | O_BINARY);
    RT_ASSERT(fd0 >= 0);
    fd1 = open(MIC1_RECORD_FILE, O_RDONLY | O_BINARY);
    RT_ASSERT(fd1 >= 0);

    g_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &pa, audio_callback_play, NULL);
    RT_ASSERT(g_client >= 0);
    record_seconds = 0;
    while (record_seconds < 5)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }

    audio_close(g_client);
    close(fd0);
    close(fd1);
    free(pcm1);
    free(pcm2);
    free(pcm_mix);
}

MSH_CMD_EXPORT(mic2file, mic2file test);



/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\n[LOCAL MUSIC]Local music Example.\n");

    mnt_init();

    /* ls files in root. */
    extern void ls(const char *name);
    ls("/");

    g_mp3_handle = mp3ctrl_open(AUDIO_TYPE_LOCAL_MUSIC, (void *)MUSIC_FILE_PATH, play_callback_func, NULL);
    RT_ASSERT(g_mp3_handle);
    mp3ctrl_play(g_mp3_handle);

    while (!g_is_end)
    {
        rt_thread_mdelay(100);
    }
    mp3ctrl_close(g_mp3_handle);

    return 0;
}

