/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * FLAC encode / decode example
 *   - flac_test [seconds]  -- record mic -> encode FLAC -> decode & play
 *   - flac_enc             -- encode /mic_record.pcm -> /test.flac
 *   - flac_play            -- decode /test.flac & play to speaker
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "FLAC/stream_encoder.h"
#include "FLAC/stream_decoder.h"
#include "audio_server.h"
#include "os_adaptor.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "log.h"

#define DBG_TAG           "flac"
#define DBG_LVL           LOG_LVL_INFO

#define MIC_PCM_FILE            "/mic_record.pcm"
#define FLAC_FILE               "/test.flac"

#define DEFAULT_SAMPLE_RATE     16000
#define DEFAULT_CHANNELS        1
#define DEFAULT_BPS             16
#define PCM_READ_SAMPLES        1024
#define MIC_DEFAULT_SECONDS     10
#define FLAC_STACK_SIZE         (20 * 1024)

static struct rt_thread flac_thread;
static uint32_t flac_stack[FLAC_STACK_SIZE / sizeof(uint32_t)];
static uint32_t g_record_seconds;

static uint8_t drop_noise_cnt;

static struct rt_thread flac_play_thread;
static uint32_t flac_play_stack[FLAC_STACK_SIZE / sizeof(uint32_t)];
static char g_flac_play_path[128];

static int record_callback(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)(intptr_t)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        if (drop_noise_cnt < 20)
        {
            drop_noise_cnt++;
            return 0;
        }
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        write(fd, (uint8_t *)p->data, p->data_len);
    }
    return 0;
}

static int mic_record_to_file(uint32_t seconds)
{
    int fd;
    audio_parameter_t pa = {0};

    pa.write_bits_per_sample = 16;
    pa.write_channnel_num   = 1;
    pa.write_samplerate     = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num    = 1;
    pa.read_samplerate      = 16000;
    pa.read_cache_size      = 0;
    pa.write_cache_size     = 2048;
    drop_noise_cnt = 0;

    fd = open(MIC_PCM_FILE, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY);
    if (fd < 0)
    {
        rt_kprintf("flac: open %s failed\n", MIC_PCM_FILE);
        return -1;
    }

    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa,
                                       record_callback, (void *)(intptr_t)fd);
    if (!client)
    {
        rt_kprintf("flac: audio_open record failed\n");
        close(fd);
        return -1;
    }

    rt_kprintf("flac: recording %u seconds ...\n", seconds);
    for (uint32_t i = 0; i < seconds; i++)
    {
        rt_thread_mdelay(1000);
        rt_kprintf("  %u/%u s\n", i + 1, seconds);
    }

    audio_close(client);
    close(fd);
    rt_kprintf("flac: record done -> %s\n", MIC_PCM_FILE);
    return 0;
}

static int flac_encode_file(const char *pcm_path, const char *flac_path)
{
    FLAC__StreamEncoder *encoder = NULL;
    FLAC__StreamEncoderInitStatus init_status;
    int pcm_fd = -1;
    int ret = -1;
    uint32_t total_samples = 0;

    const uint32_t bytes_per_sample = DEFAULT_CHANNELS * (DEFAULT_BPS / 8);
    const uint32_t read_bytes = PCM_READ_SAMPLES * bytes_per_sample;
    uint8_t *raw_buf = NULL;
    FLAC__int32 *pcm_buf = NULL;

    raw_buf = (uint8_t *)rt_malloc(read_bytes);
    pcm_buf = (FLAC__int32 *)rt_malloc(PCM_READ_SAMPLES * DEFAULT_CHANNELS * sizeof(FLAC__int32));
    if (!raw_buf || !pcm_buf)
    {
        rt_kprintf("flac_enc: malloc failed\n");
        goto _enc_exit;
    }

    pcm_fd = open(pcm_path, O_RDONLY | O_BINARY);
    if (pcm_fd < 0)
    {
        rt_kprintf("flac_enc: open %s failed\n", pcm_path);
        goto _enc_exit;
    }

    encoder = FLAC__stream_encoder_new();
    if (!encoder)
    {
        rt_kprintf("flac_enc: encoder_new failed\n");
        goto _enc_exit;
    }

    FLAC__stream_encoder_set_channels(encoder, DEFAULT_CHANNELS);
    FLAC__stream_encoder_set_bits_per_sample(encoder, DEFAULT_BPS);
    FLAC__stream_encoder_set_sample_rate(encoder, DEFAULT_SAMPLE_RATE);
    FLAC__stream_encoder_set_compression_level(encoder, 5);
    FLAC__stream_encoder_set_verify(encoder, false);

    init_status = FLAC__stream_encoder_init_file(encoder, flac_path, NULL, NULL);
    if (init_status != FLAC__STREAM_ENCODER_INIT_STATUS_OK)
    {
        rt_kprintf("flac_enc: init_file failed, status=%d\n", (int)init_status);
        goto _enc_exit;
    }

    rt_kprintf("flac_enc: encoding %s -> %s\n", pcm_path, flac_path);

    while (1)
    {
        int rd = read(pcm_fd, raw_buf, read_bytes);
        if (rd <= 0)
            break;

        uint32_t samples_read = (uint32_t)rd / bytes_per_sample;
        int16_t *p16 = (int16_t *)raw_buf;
        for (uint32_t i = 0; i < samples_read * DEFAULT_CHANNELS; i++)
            pcm_buf[i] = (FLAC__int32)p16[i];

        if (!FLAC__stream_encoder_process_interleaved(encoder, pcm_buf, samples_read))
        {
            rt_kprintf("flac_enc: process error at sample %u\n", total_samples);
            break;
        }
        total_samples += samples_read;
    }

    FLAC__stream_encoder_finish(encoder);
    rt_kprintf("flac_enc: done, %u samples\n", total_samples);
    ret = 0;

_enc_exit:
    if (encoder)  FLAC__stream_encoder_delete(encoder);
    if (pcm_fd >= 0) close(pcm_fd);
    if (raw_buf)  rt_free(raw_buf);
    if (pcm_buf)  rt_free(pcm_buf);
    return ret;
}

typedef struct
{
    audio_client_t play_client;
    uint32_t total_samples;
    uint32_t sample_rate;
    uint32_t channels;
    uint32_t bps;
} flac_play_ctx_t;

static FLAC__StreamDecoderWriteStatus play_write_cb(
    const FLAC__StreamDecoder *decoder,
    const FLAC__Frame *frame,
    const FLAC__int32 *const buffer[],
    void *client_data)
{
    flac_play_ctx_t *ctx = (flac_play_ctx_t *)client_data;
    uint32_t nsamples = frame->header.blocksize;
    uint32_t ch = frame->header.channels;
    uint32_t bps = frame->header.bits_per_sample;
    (void)decoder;

    /* 转为 16-bit interleaved PCM 并通过 audio_write 播放 */
    int16_t *out = (int16_t *)rt_malloc(nsamples * ch * sizeof(int16_t));
    if (!out)
        return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;

    for (uint32_t s = 0; s < nsamples; s++)
    {
        for (uint32_t c = 0; c < ch; c++)
            out[s * ch + c] = (int16_t)buffer[c][s];
    }

    uint32_t total_bytes = nsamples * ch * sizeof(int16_t);
    uint32_t offset = 0;
    while (offset < total_bytes)
    {
        uint32_t chunk = total_bytes - offset;
        if (chunk > 320) chunk = 320;

        int wr;
        while (1)
        {
            wr = audio_write(ctx->play_client, (uint8_t *)out + offset, chunk);
            if (wr > 0)
                break;
            rt_thread_mdelay(1);
        }
        offset += chunk;
    }

    rt_free(out);
    ctx->total_samples += nsamples;
    return FLAC__STREAM_DECODER_WRITE_STATUS_CONTINUE;
}

static void play_metadata_cb(const FLAC__StreamDecoder *decoder,
                             const FLAC__StreamMetadata *metadata,
                             void *client_data)
{
    flac_play_ctx_t *ctx = (flac_play_ctx_t *)client_data;
    (void)decoder;

    if (metadata->type == FLAC__METADATA_TYPE_STREAMINFO)
    {
        ctx->sample_rate = metadata->data.stream_info.sample_rate;
        ctx->channels    = metadata->data.stream_info.channels;
        ctx->bps         = metadata->data.stream_info.bits_per_sample;
        rt_kprintf("flac_play: %uHz %uch %ubit, total=%llu samples\n",
                   ctx->sample_rate, ctx->channels, ctx->bps,
                   (unsigned long long)metadata->data.stream_info.total_samples);
    }
}

static void play_error_cb(const FLAC__StreamDecoder *decoder,
                          FLAC__StreamDecoderErrorStatus status,
                          void *client_data)
{
    (void)decoder;
    (void)client_data;
    rt_kprintf("flac_play: decode error, status=%d\n", (int)status);
}

static int flac_decode_and_play(const char *flac_path)
{
    FLAC__StreamDecoder *decoder = NULL;
    FLAC__StreamDecoderInitStatus init_status;
    flac_play_ctx_t ctx;
    int ret = -1;

    memset(&ctx, 0, sizeof(ctx));

    audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 15);

    decoder = FLAC__stream_decoder_new();
    if (!decoder)
    {
        rt_kprintf("flac_play: decoder_new failed\n");
        return -1;
    }

    init_status = FLAC__stream_decoder_init_file(decoder, flac_path,
                  play_write_cb, play_metadata_cb,
                  play_error_cb, &ctx);
    if (init_status != FLAC__STREAM_DECODER_INIT_STATUS_OK)
    {
        rt_kprintf("flac_play: init_file failed, status=%d\n", (int)init_status);
        FLAC__stream_decoder_delete(decoder);
        return -1;
    }

    if (!FLAC__stream_decoder_process_until_end_of_metadata(decoder))
    {
        rt_kprintf("flac_play: metadata process error, state=%s\n",
                   FLAC__stream_decoder_get_resolved_state_string(decoder));
        goto _play_exit;
    }

    if (ctx.sample_rate == 0 || ctx.channels == 0)
    {
        ctx.sample_rate = DEFAULT_SAMPLE_RATE;
        ctx.channels = DEFAULT_CHANNELS;
        ctx.bps = DEFAULT_BPS;
        rt_kprintf("flac_play: no streaminfo, fallback %uHz %uch %ubit\n",
                   ctx.sample_rate, ctx.channels, ctx.bps);
    }

    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num   = (uint8_t)ctx.channels;
    pa.write_samplerate     = ctx.sample_rate;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num    = (uint8_t)ctx.channels;
    pa.read_samplerate      = ctx.sample_rate;
    pa.read_cache_size      = 0;
    pa.write_cache_size     = 4096;

    ctx.play_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &pa, NULL, NULL);
    if (!ctx.play_client)
    {
        rt_kprintf("flac_play: audio_open play failed\n");
        goto _play_exit;
    }

    rt_kprintf("flac_play: playing %s ...\n", flac_path);

    if (!FLAC__stream_decoder_process_until_end_of_stream(decoder))
    {
        rt_kprintf("flac_play: process error, state=%s\n",
                   FLAC__stream_decoder_get_resolved_state_string(decoder));
    }

    rt_thread_mdelay(500);

    rt_kprintf("flac_play: done, played %u samples\n", ctx.total_samples);
    ret = 0;

_play_exit:
    FLAC__stream_decoder_finish(decoder);
    FLAC__stream_decoder_delete(decoder);
    if (ctx.play_client)
        audio_close(ctx.play_client);
    return ret;
}

static void flac_play_thread_entry(void *param)
{
    (void)param;
    flac_decode_and_play(g_flac_play_path);
    rt_kprintf("flac_play: thread exit\n");
}

static void flac_test_thread(void *param)
{
    uint32_t seconds = g_record_seconds;
    (void)param;

    if (mic_record_to_file(seconds) != 0)
        return;

    if (flac_encode_file(MIC_PCM_FILE, FLAC_FILE) != 0)
        return;

    flac_decode_and_play(FLAC_FILE);

    rt_kprintf("flac_test: all done!\n");
}

static int cmd_flac_test(int argc, char *argv[])
{
    g_record_seconds = (argc > 1) ? (uint32_t)atoi(argv[1]) : MIC_DEFAULT_SECONDS;
    rt_thread_init(&flac_thread, "flac", flac_test_thread, NULL,
                   flac_stack, FLAC_STACK_SIZE, RT_THREAD_PRIORITY_HIGH, 10);
    rt_thread_startup(&flac_thread);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_flac_test, flac_test, Record mic then FLAC encode and play);

static int cmd_flac_enc(int argc, char *argv[])
{
    const char *in  = (argc > 1) ? argv[1] : MIC_PCM_FILE;
    const char *out = (argc > 2) ? argv[2] : FLAC_FILE;
    return flac_encode_file(in, out);
}
MSH_CMD_EXPORT_ALIAS(cmd_flac_enc, flac_enc, Encode PCM to FLAC);

static int cmd_flac_play(int argc, char *argv[])
{
    const char *path = (argc > 1) ? argv[1] : FLAC_FILE;

    rt_strncpy(g_flac_play_path, path, sizeof(g_flac_play_path) - 1);
    g_flac_play_path[sizeof(g_flac_play_path) - 1] = '\0';

    rt_thread_init(&flac_play_thread, "flac_play", flac_play_thread_entry, NULL,
                   flac_play_stack, FLAC_STACK_SIZE, RT_THREAD_PRIORITY_HIGH, 10);
    rt_thread_startup(&flac_play_thread);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_flac_play, flac_play, Decode FLAC and play);


int main(void)
{
    rt_kprintf("\n[FLAC] FLAC Encode/Decode Example.\n");
    rt_kprintf("  Commands:\n");
    rt_kprintf("    flac_test [seconds]  - record -> encode -> play\n");
    rt_kprintf("    flac_enc [in] [out]  - encode PCM to FLAC\n");
    rt_kprintf("    flac_play [file]     - decode FLAC & play\n");

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

static int mnt_init(void)
{
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, FS_ROOT);
    if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)
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
