/*
 * SPDX-FileCopyrightText: 2022-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "drivers/audio.h"
#include "ipc/ringbuffer.h"
#include "bf0_mbox_common.h"
#include "audio_mem.h"
#include "drv_io.h"
#include "audioproc.h"
#include "mem_section.h"
#include "audio_server.h"
#include "sdfilter.h"

#define DBG_TAG           "pdm_mic"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"


#define SAVE_ANYKA_OUTPUT_BY_DUMP   1

#define PDM_DELAY_SAMPLES       0
#define PDM_STEREO_DELAY_BYTES (PDM_DELAY_SAMPLES * 2 * 2)
#define PDM1_AFTER_PDM2         1 //delete this if PDM2 after PDM1


#define PDM_MOMO_FRAME_SIZE     320
#define PDM_STREO_FRAME_SIZE    (PDM_MOMO_FRAME_SIZE * 2)
#define PDM_4MIC_FRAME_SIZE     (PDM_STREO_FRAME_SIZE * 2)

#define PDM1_DEVICE_HNAME   "pdm1"
#define PDM2_DEVICE_HNAME   "pdm2"
#define PDM_THREAD_NAME     "pdm_4mic"
#define DELAY_SAMPLE    10

#define PDM_THREAD_STACK_SIZE    (4*1024)
#define PDM_THREAD_PRIORITY      10
#define AUD_TEST_FLEN       (0x80000)

#define MIC_RECORD_FILE     "/pdm_dump.wav"
#define ANYKA_OUTPUT_FILE   "/anyka_output.wav"

#define RX_EVT_PDM1     (1 << 0)
#define RX_EVT_PDM2     (1 << 1)
#define RX_EVT_EXIT     (1 << 2)


typedef struct
{
    uint8_t riff[4];
    uint32_t lenth;
    uint8_t wave[4];
    uint8_t fmt[4];
    uint32_t size1;
    uint16_t fmt_tag;
    uint16_t channel;
    uint32_t sampleRate;
    uint32_t bytePerSec;
    uint16_t blockAlign;
    uint16_t bitPerSample;
    uint8_t data[4];
    uint32_t size2;
} AUD_WAV_HDR_T;

static int pdm_status = 0;
static char pdm_interface[10];
static int total_channels = 0;
static int is_raw = 0;

audio_client_t client = NULL;
static long data_raw_len = 0;
static long anyka_out_len = 0;
static int fd_dump = 0;
static int fd_output = 0;
L2_RET_BSS_SECT_BEGIN(data2)
ALIGN(4) uint8_t data_raw[0x400000] L2_RET_BSS_SECT(data2);
ALIGN(4) uint8_t anyka_output[0x100000] L2_RET_BSS_SECT(data2);
L2_RET_BSS_SECT_END


/** Mount file system if using NAND, as BT NVDS is save in file*/
#if defined(RT_USING_DFS)
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#define NAND_MTD_NAME    "root"

#ifdef RT_USING_MTD_NOR
    #define ADDR_MASK 0xFF000000
    #define register_fs_device(flash_base, offset, size, name) register_nor_device(flash_base, offset, size, name)
#elif defined(RT_USING_MTD_NAND)
    #define ADDR_MASK 0xFC000000
    #define register_fs_device(flash_base, offset, size, name) register_nand_device(flash_base, offset, size, name)
#else
    #define ADDR_MASK 0xFC000000
    #define register_fs_device(flash_base, offset, size, name)
#endif

int mnt_init(void)
{
    //TODO: how to get base address
    register_fs_device(FS_REGION_START_ADDR & (ADDR_MASK), FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (ADDR_MASK)), FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif

static void wav_fill_header(void *data, uint32_t size, uint8_t channel)
{
    AUD_WAV_HDR_T hdr;

    hdr.riff[0] = 'R';
    hdr.riff[1] = 'I';
    hdr.riff[2] = 'F';
    hdr.riff[3] = 'F';
    hdr.lenth = size + 36;
    hdr.wave[0] = 'W';
    hdr.wave[1] = 'A';
    hdr.wave[2] = 'V';
    hdr.wave[3] = 'E';
    hdr.fmt[0] = 'f';
    hdr.fmt[1] = 'm';
    hdr.fmt[2] = 't';
    hdr.fmt[3] = ' ';
    hdr.size1 = 16;
    hdr.fmt_tag = 1;
    hdr.channel = channel;
    hdr.sampleRate = 16000;
    hdr.blockAlign = 2;
    hdr.bitPerSample = 16;
    hdr.bytePerSec = hdr.sampleRate * hdr.channel * hdr.bitPerSample / 8;
    hdr.data[0] = 'd';
    hdr.data[1] = 'a';
    hdr.data[2] = 't';
    hdr.data[3] = 'a';
    hdr.size2 = size;

    memcpy(data, &hdr, 44);
}

static void wav_save_data(uint8_t *dst, uint32_t dst_size, const uint8_t *src, uint32_t src_size, uint32_t *current_offset)
{
    uint32_t offset = *current_offset;
    if (offset + src_size > dst_size)
    {
        return;
    }
    memcpy(dst + offset, src, src_size);
    *current_offset = offset + src_size;
}

/*
    this is anyka lib callback for dump data debug
*/
void anyka_dump_data(T_ECHO_DUMP_ITEM_ID item_id, T_S32 size, T_pCVOID data, T_pCVOID meta, T_HANDLE cb_dump_param, T_U8 pathId)
{
    RT_ASSERT(is_raw == 0);
    if (item_id == ECHO_ITEM_ADC_STREAM)
    {
        wav_save_data(data_raw, sizeof(data_raw), (uint8_t *)data, size, &data_raw_len);
    }
#if SAVE_ANYKA_OUTPUT_BY_DUMP
    else if (item_id == ECHO_ITEM_NEAR_DENC)
    {

        wav_save_data(anyka_output, sizeof(anyka_output), (uint8_t *)data, size, &anyka_out_len);
    }
#endif
}


#include <finsh.h>
#include "audio_mp3ctrl.h"

static int mp3_callback_func(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (as_callback_cmd_play_to_end == cmd)
    {
        pdm_status = 0;
    }
}

void write_data_to_file(int fd, uint8_t *data, uint32_t data_len)
{
    uint32_t offset = 0;
    while (data_len > 512)
    {
        rt_kprintf("write to file\n");
        write(fd, data + offset, 512);
        offset += 512;
        data_len -= 512;
    }
    if (data_len > 0)
    {
        write(fd, data + offset, data_len);
    }
}
static int mic_callback(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    audio_client_t client = *((audio_client_t *)callback_userdata);
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        if (is_raw)
        {
            if (total_channels == 4)
            {
                RT_ASSERT(p->data_len == 640);
                //LOG_I("raw pdm %d stereo data comming len=%d", p->reserved, p->data_len);
                wav_save_data(data_raw, sizeof(data_raw), p->data, p->data_len, &data_raw_len);
            }
            else if (total_channels == 2)
            {
                RT_ASSERT(p->data_len == 640);
                //LOG_I("raw pdm %d stereo data comming len=%d", p->reserved, p->data_len);
                wav_save_data(data_raw, sizeof(data_raw), p->data, p->data_len, &data_raw_len);
            }
            else
            {
                RT_ASSERT(p->data_len == 320);
                //LOG_I("raw pdm %d mono data comming len=%d", p->reserved, p->data_len);
                wav_save_data(data_raw, sizeof(data_raw), p->data, p->data_len, &data_raw_len);
            }
        }
        else
        {
#if !SAVE_ANYKA_OUTPUT_BY_DUMP
            RT_ASSERT(p->data_len == 320);
            //LOG_I("anyka pdm %d data comming len=%d", p->reserved, p->data_len);
            wav_save_data(anyka_output, sizeof(anyka_output), p->data, p->data_len, &anyka_out_len);
#endif
        }
    }
    return 0;
}

static void pdm(uint8_t argc, char **argv)
{
    char *value = NULL;
    uint32_t samplerate = 16000;

    if (argc < 2)
    {
        rt_kprintf("no command: \r\n");
        return;
    }

    if (strcmp(argv[1], "open") == 0 && (pdm_status == 0) && argc >= 5)
    {
        pdm_status = 1;
        strncpy(pdm_interface, argv[2], sizeof(pdm_interface));
        total_channels = atoi(argv[3]);
        is_raw = 0;
        if (!strcmp(argv[4], "raw"))
        {
            is_raw = 1;
            wav_fill_header(data_raw, 0, total_channels);
        }
        else
        {
            wav_fill_header(data_raw, 0, total_channels);
            wav_fill_header(anyka_output, 0, 1);
        }

        data_raw_len = sizeof(AUD_WAV_HDR_T);
        anyka_out_len = data_raw_len;
        rt_kprintf("PDM opened at %d Hz\r\n", samplerate);

        audio_parameter_t pa = {0};

        pa.write_bits_per_sample = 16;
        pa.write_channnel_num = 1;
        pa.write_samplerate = 16000;
        pa.read_bits_per_sample = 16;
        pa.read_samplerate = 16000;
        if (!strcmp(pdm_interface, "pdm1"))
        {
            pa.read_which_mic = AUDIO_MIC0_ONLY;
        }
        else if (!strcmp(pdm_interface, "pdm2"))
        {
            pa.read_which_mic = AUDIO_MIC1_ONLY;
        }
        else
        {
            pa.read_which_mic = AUDIO_MIC_ALL;
        }
        if (total_channels == 4)
        {
            pa.read_which_mic = AUDIO_MIC_ALL;
        }
        if (!is_raw)
        {
            pa.is_need_3a = 1;
        }
        pa.read_channnel_num = 1;
        if (total_channels == 2 || total_channels == 4)
        {
            pa.read_channnel_num = 2; // each PDM is stereo
        }
        client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, mic_callback, &client);
        return;
    }

    if (strcmp(argv[1], "close") == 0 && (pdm_status == 1))
    {
        pdm_status = 0;

        if (is_raw)
        {
            wav_fill_header(data_raw, data_raw_len, total_channels);
        }
        else
        {
            wav_fill_header(data_raw, data_raw_len, total_channels);
            wav_fill_header(anyka_output, anyka_out_len, 1);
        }

        fd_dump = open(MIC_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
        RT_ASSERT(fd_dump >= 0);
        write_data_to_file(fd_dump, data_raw, data_raw_len);
        close(fd_dump);

        if (!is_raw)
        {
            fd_output = open(ANYKA_OUTPUT_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
            RT_ASSERT(fd_output >= 0);
            write_data_to_file(fd_output, anyka_output, anyka_out_len);
            close(fd_output);
        }
        rt_kprintf("PDM closed\r\n");
        audio_close(client);
    }

    if (strcmp(argv[1], "play") == 0 && (pdm_status == 0))
    {
        pdm_status = 2;
        audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 15);
        mp3ctrl_handle handle = mp3ctrl_open(AUDIO_TYPE_LOCAL_MUSIC, ANYKA_OUTPUT_FILE, mp3_callback_func, NULL);
        if (handle)
        {
            mp3ctrl_play(handle);
            for (int i = 0; i < 1000; i++)
            {
                if (pdm_status == 0)
                {
                    break;
                }
                rt_thread_mdelay(1000);
            }
            mp3ctrl_close(handle);
        }
        else
        {
            rt_kprintf("no anyka out file\n");
        }
        pdm_status = 0;
        return;
    }
}
MSH_CMD_EXPORT(pdm, PDM recording control);

int main(void)
{
    rt_kprintf("----4mic Record Example.\n");

    //56x
    HAL_PIN_Set(PAD_PA69, PDM1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA64, PDM1_DATA, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA73, PDM2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA71, PDM2_DATA, PIN_PULLDOWN, 1);

    while (1)
    {
        rt_thread_mdelay(10000);
    }
    return 0;
}
