/*
 * SPDX-FileCopyrightText: 2025-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include "opus.h"
#include "debug.h"
#include "opus_types.h"
#include "opus_private.h"
#include "opus_multistream.h"
#include "os_support.h"
#include "audio_server.h"
#include "os_adaptor.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "log.h"

#if defined(PKG_USING_LIBOGG)
    #include "ogg/config_types.h"
    #include "ogg/ogg.h"
    #include "crctable.h"
#endif


#define MIC_RECORD_FILE   "/mic16k.pcm"
#define OPUS_OGG_FILE     "/opus.ogg"
#define DBG_TAG           "opus"
#define DBG_LVL           LOG_LVL_ERROR

#define OPUS_GRANULE_PER_FRAME_10MS  480   /* 10ms @ 48kHz */

#if defined(BSP_USING_ACPU)
    #define OPUS_STACK_SIZE     16000
#else
    #define OPUS_STACK_SIZE     20000
#endif

static OpusEncoder *encoder;
static int pcm_file;
static struct rt_thread thread;
static uint8_t drop_noise_frame_cnt;
static uint32_t opus_stack[OPUS_STACK_SIZE / sizeof(uint32_t)];

static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        if (drop_noise_frame_cnt < 20)
        {
            drop_noise_frame_cnt++;
            return 0;
        }
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        write(fd, (uint8_t *)p->data, p->data_len);
    }
    return 0;
}

static void mic2file(void)
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
    pa.read_cache_size = 0;
    pa.write_cache_size = 2048;
    drop_noise_frame_cnt = 0;

    fd = open(MIC_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    if (fd < 0)
    {
        return;
    }

    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, audio_callback_record, (void *)fd);
    if (!client)
    {
        rt_kprintf("audio_open record failed\n");
        close(fd);
        return;
    }
    rt_kprintf("mic2file: recording 10s ...\n");

    while (record_seconds < 10)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
        rt_kprintf("record %u s\n", record_seconds);
    }
    audio_close(client);
    close(fd);
    rt_kprintf("mic2file: record end\n");
}

static void opus_test(void *p)
{
    int err;
    int stack;
    uint8_t buf[320];
    uint8_t encode_out[320];
    int read_len;
    uint32_t frame_count = 0;

    (void)p;
    rt_kprintf("opus_test start, stack var addr=0x%x\n", (unsigned int)(uintptr_t)&stack);

    /* 1. Opus encoder init */
    encoder = opus_encoder_create(16000, 1, OPUS_APPLICATION_VOIP, &err);
    if (!encoder)
    {
        rt_kprintf("opus_encoder_create failed\n");
        return;
    }

    opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
    opus_encoder_ctl(encoder, OPUS_SET_VBR(1));
    opus_encoder_ctl(encoder, OPUS_SET_VBR_CONSTRAINT(1));
    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(16000));
    opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(0));
    opus_encoder_ctl(encoder, OPUS_SET_LSB_DEPTH(16));
    opus_encoder_ctl(encoder, OPUS_SET_DTX(0));
    opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(0));
    opus_encoder_ctl(encoder, OPUS_SET_PACKET_LOSS_PERC(0));
    opus_encoder_ctl(encoder, OPUS_SET_PREDICTION_DISABLED(0));
    opus_encoder_ctl(encoder, OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND));
    opus_encoder_ctl(encoder, OPUS_SET_BANDWIDTH(OPUS_AUTO));
    opus_encoder_ctl(encoder, OPUS_SET_FORCE_MODE(MODE_SILK_ONLY));
    rt_kprintf("opus encoder ctrl done\n");

    if (pcm_file < 0)
    {
        rt_kprintf("pcm_file=%d invalid\n", pcm_file);
        opus_encoder_destroy(encoder);
        encoder = NULL;
        return;
    }

#if defined(PKG_USING_LIBOGG)
    int ogg_fd = open(OPUS_OGG_FILE, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY);
    if (ogg_fd < 0)
    {
        rt_kprintf("open %s failed (errno=%d)\n", OPUS_OGG_FILE, errno);
        close(pcm_file);
        pcm_file = -1;
        opus_encoder_destroy(encoder);
        encoder = NULL;
        return;
    }
    rt_kprintf("open %s ok, fd=%d\n", OPUS_OGG_FILE, ogg_fd);

    ogg_stream_state os;
    ogg_page og;
    ogg_packet op;
    int serialno = 0x12345678;
    if (ogg_stream_init(&os, serialno) != 0)
    {
        rt_kprintf("ogg_stream_init failed\n");
        close(ogg_fd);
        close(pcm_file);
        pcm_file = -1;
        opus_encoder_destroy(encoder);
        encoder = NULL;
        return;
    }
    rt_kprintf("ogg_stream_init ok, serialno=0x%x\n", serialno);

    /* OpusHead 包 (RFC 7845): 19 字节 for mono */
    unsigned char opus_head[19] = {
        'O', 'p', 'u', 's', 'H', 'e', 'a', 'd',    /* magic */
        1,                                         /* version */
        1,                                         /* channel count */
        0x00, 0x0f,                                /* pre_skip 3840 LE (80ms @48kHz) */
        0x80, 0xBB, 0x00, 0x00,                    /* input_sample_rate 48000 LE */
        0, 0,                                      /* output_gain 0 LE */
        0                                          /* channel_mapping_family 0 */
    };
    op.packet = opus_head;
    op.bytes = 19;
    op.b_o_s = 1;
    op.e_o_s = 0;
    op.granulepos = 0;
    op.packetno = 0;
    if (ogg_stream_packetin(&os, &op) != 0)
    {
        rt_kprintf("ogg_stream_packetin OpusHead failed\n");
        ogg_stream_clear(&os);
        close(ogg_fd);
        close(pcm_file);
        pcm_file = -1;
        opus_encoder_destroy(encoder);
        encoder = NULL;
        return;
    }
    rt_kprintf("OpusHead packetin ok\n");
    while (ogg_stream_pageout(&os, &og) == 1)
    {
        if (write(ogg_fd, og.header, (size_t)og.header_len) != (ssize_t)og.header_len ||
            write(ogg_fd, og.body, (size_t)og.body_len) != (ssize_t)og.body_len)
        {
            rt_kprintf("write ogg header page failed\n");
            break;
        }
        rt_kprintf("write ogg header page, header_len=%ld body_len=%ld\n", (long)og.header_len, (long)og.body_len);
    }

    unsigned char opus_tags[16] = {
        'O', 'p', 'u', 's', 'T', 'a', 'g', 's',    /* magic */
        0, 0, 0, 0,                                /* vendor string length = 0 LE */
        0, 0, 0, 0                                 /* user comment list length = 0 LE */
    };
    op.packet = opus_tags;
    op.bytes = 16;
    op.b_o_s = 0;
    op.e_o_s = 0;
    op.granulepos = 0;
    op.packetno = 1;
    if (ogg_stream_packetin(&os, &op) != 0)
    {
        rt_kprintf("ogg_stream_packetin OpusTags failed\n");
        ogg_stream_clear(&os);
        close(ogg_fd);
        close(pcm_file);
        pcm_file = -1;
        opus_encoder_destroy(encoder);
        encoder = NULL;
        return;
    }
    rt_kprintf("OpusTags packetin ok\n");
    while (ogg_stream_pageout(&os, &og) == 1)
    {
        if (write(ogg_fd, og.header, (size_t)og.header_len) != (ssize_t)og.header_len ||
            write(ogg_fd, og.body, (size_t)og.body_len) != (ssize_t)og.body_len)
        {
            rt_kprintf("write ogg OpusTags page failed\n");
            break;
        }
    }

    ogg_int64_t granulepos = 0;
    int is_last_frame = 0;

    read_len = read(pcm_file, buf, sizeof(buf));
    RT_ASSERT(read_len == (int)sizeof(buf));

    while (read_len == (int)sizeof(buf))
    {
        opus_int32 len = opus_encode(encoder, (const opus_int16 *)buf, 320 / 2, encode_out, 320);
        if (len < 0)
        {
            rt_kprintf("opus_encode err=%d frame=%u\n", (int)len, frame_count);
            break;
        }
        granulepos += OPUS_GRANULE_PER_FRAME_10MS;
        uint8_t next_buf[320];
        int next_buf_len = read(pcm_file, next_buf, sizeof(next_buf));
        if(next_buf_len != (int)sizeof(next_buf))
        {
            is_last_frame = 1;
        }
        else
        {
            memcpy(buf, next_buf, sizeof(next_buf));
        }
        op.packet = encode_out;
        op.bytes = (long)len;
        op.b_o_s = 0;
        op.e_o_s = is_last_frame ? 1 : 0;
        op.granulepos = granulepos;
        op.packetno = (ogg_int64_t)(frame_count + 2);  /* +2: OpusHead=0, OpusTags=1 */
        if (ogg_stream_packetin(&os, &op) != 0)
        {
            rt_kprintf("ogg_stream_packetin frame %u failed\n", frame_count);
            break;
        }
        while (ogg_stream_pageout(&os, &og) == 1)
        {
            if (write(ogg_fd, og.header, (size_t)og.header_len) != (ssize_t)og.header_len ||
                write(ogg_fd, og.body, (size_t)og.body_len) != (ssize_t)og.body_len)
            {
                rt_kprintf("write ogg page failed\n");
                break;
            }
        }
        frame_count++;
        if (is_last_frame)
            break;
    }
    while (ogg_stream_flush(&os, &og) == 1)
    {
        if (write(ogg_fd, og.header, (size_t)og.header_len) != (ssize_t)og.header_len ||
            write(ogg_fd, og.body, (size_t)og.body_len) != (ssize_t)og.body_len)
        {
            rt_kprintf("write ogg flush page failed\n");
            break;
        }
    }
    ogg_stream_clear(&os);
    close(ogg_fd);
    rt_kprintf("ogg write done, total frames=%u\n", frame_count);
#else
    read_len = read(pcm_file, buf, sizeof(buf));
    if (read_len != (int)sizeof(buf))
        break;
    opus_int32 len = opus_encode(encoder, (const opus_int16 *)buf, 320 / 2, encode_out, 320);
    if (len < 0)
        break;
#endif

    close(pcm_file);
    pcm_file = -1;
    opus_encoder_destroy(encoder);
    encoder = NULL;
    rt_kprintf("opus_test exit\n");
}

/*
cmd help:
 1. opus
    record a 16k pcm file to /mic16k.pcm, than play it
 2. opus /mic16k.pcm
   opus read file /mic16k.pcm, and encode and decode, than play
 3. opus xxxx
    if file xxxx not exist, opus record mic data, and decode it to speaker
 */
int opus(int argc, char *argv[])
{
    OpusEncoder *encoder = NULL;
    OpusDecoder *decoder = NULL;
    audio_client_t client = NULL;
    if (argc == 1)
    {
        mic2file();
        pcm_file = open(MIC_RECORD_FILE, O_RDONLY | O_BINARY);
        if (pcm_file < 0)
        {
            rt_kprintf("open file %s error\n", MIC_RECORD_FILE);
            return -1;
        }
    }
    else
    {
        pcm_file = open(argv[1], O_RDONLY | O_BINARY);
        if (pcm_file < 0)
        {
            rt_kprintf("open file %s error\n", argv[1]);
        }
    }

    rt_thread_init(&thread, "opus", opus_test, NULL, opus_stack, OPUS_STACK_SIZE, RT_THREAD_PRIORITY_HIGH, 10);
    rt_thread_startup(&thread);
    return 0;
}
MSH_CMD_EXPORT(opus, opus test);


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\n[Opus]Opus Example.\n");
    rt_thread_mdelay(1000);

    mic2file();
    pcm_file = open(MIC_RECORD_FILE, O_RDONLY | O_BINARY);

    rt_thread_init(&thread, "opus", opus_test, NULL, opus_stack, OPUS_STACK_SIZE, RT_THREAD_PRIORITY_HIGH, 10);
    rt_thread_startup(&thread);

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}


#if 1
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "drv_flash.h"

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
INIT_ENV_EXPORT(mnt_init);
#endif
