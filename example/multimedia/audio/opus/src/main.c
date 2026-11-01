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
#include "mem_section.h"
#if defined(BSP_USING_ACPU)
    #include "acpu_ctrl.h"
#endif
#define DBG_TAG           "opus"
#define DBG_LVL           LOG_LVL_ERROR
#include "log.h"

//#define MIC_RECORD_FILE "/ramfs/mic16k.pcm"
#define MIC_RECORD_FILE "/mic16k.pcm"

#define RECORD_USING_WEBRTC 0
#define RECORD_SECONDS        10
#define RECORD_FRAME_BYTES    320
#define RECORD_FRAMES         (RECORD_SECONDS * 100)
#define RECORD_BUF_SIZE       (RECORD_FRAMES * RECORD_FRAME_BYTES)
#define RECORD_TIMEOUT_MS     (RECORD_SECONDS * 1000 + 5000)  /* recording time plus margin */

static OpusEncoder *encoder;
static OpusDecoder *decoder;

static uint16_t *pcm;
static audio_client_t g_client;
static int cache_full;
static int pcm_file;
static struct rt_thread thread;
static int thread_running;
static int opus_ready;
static uint8_t drop_noise_frame_cnt;

static struct rt_semaphore loopback_sem;
static uint8_t loopback_pcm_buf[320];
static int loopback_pcm_len;

L2_RET_BSS_SECT_BEGIN(rec_pcm)
ALIGN(4) static uint8_t rec_pcm_buf[RECORD_BUF_SIZE] L2_RET_BSS_SECT(rec_pcm);
L2_RET_BSS_SECT_END

static volatile uint32_t rec_bytes;
static volatile uint8_t rec_full;
static struct rt_semaphore rec_done_sem;


#if RECORD_USING_WEBRTC
#include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"
#include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"
static NsxHandle               *pNS_inst;
static void                    *agcInst;
static uint8_t *frame0;
static uint8_t *frame1;
static uint8_t *in;
static uint8_t *out;
static void app_recorder_ans_proc(NsxHandle *h, int16_t spframe[160], int16_t outframe[160])
{
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
    if (h)
    {
        WebRtcNsx_Process(h, (const int16_t *const *)spframe_p, 1, outframe_p);
    }
}

static void app_recorder_agc_proc(void *h, int16_t spframe[160], int16_t outframe[160])
{
    int32_t micLevelIn = 0;
    int32_t micLevelOut = 0;
    uint8_t saturationWarning;
    uint16_t u16_frame_len = 160;
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
    if (h && 0 != WebRtcAgc_Process(h, (const int16_t *const *)spframe_p, 1, u16_frame_len, (int16_t *const *)outframe_p, micLevelIn, &micLevelOut, 0, &saturationWarning))
    {
        LOG_W("WebRtcAgc_Process error !\n");
    }
}

static void webrtc_process_frame(const uint8_t *p, uint32_t data_len)
{
    app_recorder_ans_proc(pNS_inst, (int16_t *)p, (int16_t *)frame0);
    app_recorder_agc_proc(agcInst, (int16_t *)frame0, (int16_t *)frame1);
}

static void webrtc_open()
{
    pNS_inst = WebRtcNsx_Create();
    RT_ASSERT(pNS_inst);
    if (0 != WebRtcNsx_Init(pNS_inst, 16000))
    {
        RT_ASSERT(0);
    }
    else if (0 != WebRtcNsx_set_policy(pNS_inst, 2))
    {
        RT_ASSERT(0);
    }
    WebRtcAgcConfig agcConfig;
    agcConfig.compressionGaindB = 19;
    agcConfig.limiterEnable = 1;
    agcConfig.targetLevelDbfs = 3;
    agcConfig.thrhold = 14;
    agcInst = WebRtcAgc_Create();
    RT_ASSERT(agcInst);
    if (0 != WebRtcAgc_Init(agcInst, 0, 255, 3, 16000)) // 3 --> kAgcModeFixedDigital
    {
        RT_ASSERT(0);
    }
    if (0 != WebRtcAgc_set_config(agcInst, agcConfig))
    {
        RT_ASSERT(0);
    }
    frame0 = malloc(320);
    RT_ASSERT(frame0);
    frame1 = malloc(320);
    RT_ASSERT(frame1);
}

static void webrtc_close()
{
    if (pNS_inst)
        WebRtcNsx_Free(pNS_inst);
    if (agcInst)
        WebRtcAgc_Free(agcInst);

    if (frame0)
        free(frame0);

    if (frame1)
        free(frame1);

    frame0   = NULL;
    frame1   = NULL;
    pNS_inst = NULL;
    agcInst  = NULL;
}

#endif

static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    (void)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        if (drop_noise_frame_cnt < 20)
        {
            drop_noise_frame_cnt++;
            return 0;
        }
        if (rec_full)
        {
            return 0;
        }
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        uint32_t len = p->data_len;
        if (len > RECORD_FRAME_BYTES)
        {
            len = RECORD_FRAME_BYTES;
        }
        if (rec_bytes + len > RECORD_BUF_SIZE)
        {
            len = RECORD_BUF_SIZE - rec_bytes;
        }
#if RECORD_USING_WEBRTC
        RT_ASSERT(p->data_len == RECORD_FRAME_BYTES);
        webrtc_process_frame(p->data, p->data_len);
        memcpy(&rec_pcm_buf[rec_bytes], frame1, len);
#else
        memcpy(&rec_pcm_buf[rec_bytes], p->data, len);
#endif
        rec_bytes += len;
        if (rec_bytes >= RECORD_BUF_SIZE)
        {
            rec_full = 1;
            rt_sem_release(&rec_done_sem);
        }
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

static int opus_loop_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_data_coming)
    {
        if (drop_noise_frame_cnt < 20)
        {
            drop_noise_frame_cnt++;
            return 0;
        }
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        loopback_pcm_len = p->data_len > sizeof(loopback_pcm_buf) ? sizeof(loopback_pcm_buf) : p->data_len;
        memcpy(loopback_pcm_buf, p->data, loopback_pcm_len);
        rt_sem_release(&loopback_sem);
    }
    return 0;
}
static void mic2file()
{
    int fd;
    int written;
    uint32_t play_seconds;
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
    rec_bytes = 0;
    rec_full = 0;
    cache_full = 0;
    rt_sem_init(&rec_done_sem, "rec_done", 0, RT_IPC_FLAG_FIFO);
#if RECORD_USING_WEBRTC
    webrtc_open();
#endif
    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, audio_callback_record, RT_NULL);
    RT_ASSERT(client);
    rt_kprintf("record start\n");

    if (rt_sem_take(&rec_done_sem, rt_tick_from_millisecond(RECORD_TIMEOUT_MS)) != RT_EOK)
    {
        rt_kprintf("record timeout: %u/%u bytes\n", rec_bytes, RECORD_BUF_SIZE);
    }

    audio_close(client);
    rt_sem_detach(&rec_done_sem);
#if RECORD_USING_WEBRTC
    webrtc_close();
#endif
    rt_kprintf("record stop: %u bytes (%u ms)\n", rec_bytes, rec_bytes / 32);

    /* record to file */
    fd = open(MIC_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    if (fd < 0)
    {
        LOG_E("open %s failed (errno=%d)", MIC_RECORD_FILE, errno);
        return;
    }
    written = write(fd, rec_pcm_buf, rec_bytes);
    if (written != (int)rec_bytes)
    {
        LOG_E("write %s failed: %d/%u", MIC_RECORD_FILE, written, rec_bytes);
    }
    close(fd);
    rt_kprintf("record end\n");
    //play now
    pcm = malloc(4096);
    RT_ASSERT(pcm);
    pa.write_cache_size = 4096;
    fd = open(MIC_RECORD_FILE, O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        LOG_E("open %s failed (errno=%d)", MIC_RECORD_FILE, errno);
        free(pcm);
        pcm = NULL;
        return;
    }

    g_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &pa, audio_callback_play, (void *)fd);
    RT_ASSERT(g_client >= 0);
    /* 16 kHz, 16-bit mono: 32 bytes per ms */
    play_seconds = (rec_bytes / 32 + 999) / 1000;
    rt_kprintf("playing start\n");
    while (play_seconds > 0)
    {
        rt_thread_mdelay(1000);
        play_seconds--;
    }

    audio_close(g_client);
    close(fd);
    free(pcm);
    pcm = NULL;
    rt_kprintf("playing end\n");
}
#if defined(BSP_USING_ACPU)
    #define OPUS_STACK_SIZE     16000
#else
    #define OPUS_STACK_SIZE     20000
#endif

static  uint32_t opus_stack[OPUS_STACK_SIZE / sizeof(uint32_t)];
static uint32_t debug_frame;
static void opus_test(void *p)
{

    int err;
    int stack;
    thread_running = 1;
    debug_frame = 0;
    rt_kprintf("opus runing stack var address =0x%x\r\n", &stack);

    encoder = opus_encoder_create(16000, 1, OPUS_APPLICATION_VOIP, &err);
    rt_kprintf("encoder create=%d\r\n", err);
    RT_ASSERT(encoder);
#if defined(USING_ACPU_CTRL_FWK)
    uint8_t error_code = 1;
    opus_encode_ctl_arg_t arg;
    arg.st = encoder;
    arg.id = 0;
    acpu_run_task(ACPU_TASK_opus_encoder_ctl, &arg, sizeof(arg), &error_code);
    RT_ASSERT(error_code == 0);
    rt_kprintf("encoder ctrl\r\n");
#else
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
#endif /* USING_ACPU_CTRL_FWK */
    decoder = opus_decoder_create(16000, 1, &err);
    rt_kprintf("decoder create=%d\r\n", err);
    RT_ASSERT(decoder);
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.read_cache_size = 0;
    pa.write_cache_size = 320 * 8;
    audio_client_t client;
    if (pcm_file >= 0)
    {
        client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_TXRX, &pa, NULL, NULL);
    }
    else
    {
        rt_sem_init(&loopback_sem, "opus_lb", 0, RT_IPC_FLAG_FIFO);
        drop_noise_frame_cnt = 0;
        client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_TXRX, &pa, opus_loop_record, NULL);
        // 10s at 10ms per frame = 1000 frames
        while (debug_frame < 1000)
        {
            rt_sem_take(&loopback_sem, RT_WAITING_FOREVER);

            short encode_out[160];
            short pcm_out[320];
            opus_int32 len = opus_encode(encoder, (const opus_int16 *)loopback_pcm_buf,
                                         loopback_pcm_len / 2, (uint8_t *)encode_out, sizeof(encode_out));
            LOG_I("opus encode len=%d frame=%d", len, debug_frame);
            if (len < 0 || len > sizeof(encode_out))
            {
                RT_ASSERT(0);
            }

            opus_int32 res = opus_decode(decoder, (uint8_t *)encode_out, len, pcm_out, 320, 0);
            LOG_I("opus decoce res=%d frame=%d", res, debug_frame);
            if (res != 320 / 2)
            {
                rt_kprintf("decode out samples=%d\n", res);
                RT_ASSERT(0);
            }

            while (1)
            {
                int writted = audio_write(client, (uint8_t *)pcm_out, 320);
                if (writted == 0)
                {
                    rt_thread_mdelay(5);
                }
                else
                {
                    break;
                }
            }

            debug_frame++;
        }
        rt_sem_detach(&loopback_sem);
    }
    uint8_t buf[320];
    short encode_out[320 / 2];
    short pcm[320];

    while (1)
    {

        int read_len = read(pcm_file, buf, sizeof(buf));
        if (read_len != sizeof(buf))
            break;

        opus_int32 len = opus_encode(encoder, (const opus_int16 *)buf, 320 / 2, (uint8_t *)&encode_out[0], 320);
        LOG_I("opus encode len=%d frame=%d", len, debug_frame);
        if (len < 0 || len > 320)
        {
            RT_ASSERT(0);
        }

        opus_int32 res = opus_decode(decoder, (uint8_t *)&encode_out[0], len, pcm, 320, 0);
        LOG_I("opus decoce res=%d frame=%d", res, debug_frame);
        if (res != 320 / 2)
        {
            rt_kprintf("decode out samples=%d\n", res);
            RT_ASSERT(0);
        }
        debug_frame++;
#if 1
        while (1)
        {
            int writted = audio_write(client, (uint8_t *)&pcm[0], 320);
            if (writted == 0)
            {
                rt_thread_mdelay(5);
            }
            else
            {
                break;
            }
        }
#endif
    }

    close(pcm_file);
    audio_close(client);

    opus_encoder_destroy(encoder);
    opus_decoder_destroy(decoder);

    //unlink(MIC_RECORD_FILE);

    thread_running = 0;
    rt_kprintf("---opus test exit---\r\n");
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

    if (!opus_ready)
    {
        rt_kprintf("system not ready yet, please wait...\n");
        return -1;
    }

    if (thread_running)
    {
        rt_kprintf("opus thread is already running, please wait for it to exit\n");
        return -1;
    }

    if (argc == 1)
    {
        mic2file();
        pcm_file = open(MIC_RECORD_FILE, O_RDONLY | O_BINARY);
        if (pcm_file < 0)
        {
            LOG_I("open file %s error", MIC_RECORD_FILE);
            return -1;
        }
    }
    else
    {
        pcm_file = open(argv[1], O_RDONLY | O_BINARY);
        if (pcm_file < 0)
        {
            LOG_I("open file %s error", argv[1]);
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
    opus_ready = 1;

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
