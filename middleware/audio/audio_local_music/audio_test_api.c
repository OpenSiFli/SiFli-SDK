#include <rtthread.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "os_adaptor.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "audio_server.h"
#include "audio_mp3ctrl.h"
#include "audio_test_api.h"

#define FILE_PATH_16000                 "/ring/long/16k.wav"     //"/ring/long/16k.mp3"
#define FILE_PATH_44100                 "/ring/long/44_1k.wav"  //"/ring/long/44_1.mp3"
#define FILE_PLAYING_WHEN_RECORD        "/dyn/16k.pcm"
#define FILE_RECORD                     "/dyn/record16k.pcm"

#define RECORD_TIME_SECOND  6

#define DBG_TAG           "audio"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

static audio_client_t      g_client;
static struct rt_event     event;
static struct rt_mutex     mutex;
static uint32_t g_samplerate = 16000;
static double x;
static uint32_t steps;
static uint8_t g_test_eq_enable;
static uint8_t g_test_3a_enable;
static uint8_t g_dump;
static uint8_t g_playing;

static mp3ctrl_handle g_mp3;
static int16_t *g_pcm;
static int g_fd;
static int g_bg_playing_fd;


#define TEST_E_1K       (1 << 0)
#define TEST_E_0        (1 << 1)
#define TEST_E_FILE     (1 << 2)
#define TEST_E_R_P      (1 << 3)
#define TEST_E_R        (1 << 4)
#define TEST_E_STOP     (1 << 5)

#define TEST_EVENT_ALL  (TEST_E_1K | TEST_E_0 | TEST_E_FILE | TEST_E_R_P | TEST_E_R | TEST_E_STOP)



static void record_play(void);
static void record_stop(void);
static void record_start(void);

uint8_t audio_test_api_eq_is_enable()
{
    return g_test_eq_enable;
}
uint8_t audio_test_api_3a_is_enable()
{
    return g_test_3a_enable;
}
void audio_test_play_1khz(uint32_t samplerate,  uint8_t eq_enble)
{
    LOG_I("call %s", __FUNCTION__);
    if (samplerate == 16000)
    {
        g_samplerate = 16000;
    }
    else
    {
        g_samplerate = 44100;
    }
    g_test_eq_enable = eq_enble;
    rt_event_send(&event, TEST_E_1K);
}

void audio_test_play_zero(uint32_t samplerate,  uint8_t eq_enble)
{
    LOG_I("call %s", __FUNCTION__);
    if (samplerate == 16000)
    {
        g_samplerate = 16000;
    }
    else
    {
        g_samplerate = 44100;
    }
    g_test_eq_enable = eq_enble;
    rt_event_send(&event, TEST_E_0);
}

void audio_test_play_file(uint32_t samplerate, uint8_t eq_enble)
{
    LOG_I("call %s", __FUNCTION__);
    if (samplerate == 16000)
    {
        g_samplerate = 16000;
    }
    else
    {
        g_samplerate = 44100;
    }
    g_test_eq_enable = eq_enble;
    rt_event_send(&event, TEST_E_FILE);
}

void audio_test_record_and_play_file(uint8_t eq_enble)
{
    LOG_I("call %s", __FUNCTION__);
    g_test_eq_enable = eq_enble;
    rt_event_send(&event, TEST_E_R_P);
}

void audio_test_record_16k_start(uint8_t playing, uint8_t dump)
{
    LOG_I("call %s", __FUNCTION__);
    g_dump = dump;
    g_playing = playing;
    rt_event_send(&event, TEST_E_R);
}
void audio_test_record_16k_end(void)
{
    rt_event_send(&event, TEST_E_STOP);
}

void audio_test_3a_enable(uint8_t is_enable, uint8_t eq_enable)
{
    g_test_eq_enable = eq_enable;
    g_test_3a_enable = is_enable;
}
void audio_test_stop(void)
{
    rt_event_send(&event, TEST_E_STOP);
}

int audio_test_cmd(int argc, char **argv)
{
    uint32_t sample = 44100;
    uint8_t     eq = 0;
    if (argc < 2)
        goto exit_;
    if (!strcmp(argv[1], "1k")
            || !strcmp(argv[1], "zero")
            || !strcmp(argv[1], "file"))
    {
        if (argc < 4)
            goto exit_;
        if (!strcmp(argv[2], "16000"))
        {
            sample = 16000;
        }
        if (argv[2][0] == '1')
        {
            eq = 1;
        }
        if (!strcmp(argv[1], "1k"))
            audio_test_play_1khz(sample, eq);
        else if (!strcmp(argv[1], "zero"))
            audio_test_play_zero(sample, eq);
        else if (!strcmp(argv[1], "file"))
            audio_test_play_file(sample, eq);
    }
    else if (!strcmp(argv[1], "recordplay"))
    {
        if (argc < 3)
            goto exit_;
        if (argv[2][0] == '1')
        {
            eq = 1;
        }
        audio_test_record_and_play_file(eq);
    }
    else if (!strcmp(argv[1], "record"))
    {
        uint8_t playing = 0, dump = 0;
        if (argc < 4)
            goto exit_;

        if (argv[2][0] == '1')
        {
            playing = 1;
        }
        if (argv[3][0] == '1')
        {
            dump = 1;
        }
        audio_test_record_16k_start(playing, dump);
    }
    else if (!strcmp(argv[1], "stop"))
    {
        audio_test_record_16k_end();
    }

    return 0;

exit_:
    LOG_I("usage:\r\n");
    LOG_I("audio_test_cmd 1k/zero/file 16000/44100 1/0(eq)\r\n");
    LOG_I("audio_test_cmd recordplay 1/0(eq)\r\n");
    LOG_I("audio_test_cmd record 1/0(playing) 1/0(dump)\r\n");
    LOG_I("audio_test_cmd stop\r\n");
    return 0;
}
MSH_CMD_EXPORT(audio_test_cmd, audio test cmd);
static void playing_file(void)
{
    if (g_samplerate == 16000)
        g_mp3 = mp3ctrl_open(AUDIO_TYPE_LOCAL_MUSIC, FILE_PATH_16000, NULL, NULL);
    else
        g_mp3 = mp3ctrl_open(AUDIO_TYPE_LOCAL_MUSIC, FILE_PATH_44100, NULL, NULL);
    if (!g_mp3)
    {
        return;
    }
    mp3ctrl_play(g_mp3);
    mp3ctrl_ioctl(g_mp3, 0, 0x7FFF);
}
static void playing_0(void)
{
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;

    if (g_samplerate == 16000)
    {
        pa.write_samplerate = 16000;
        pa.read_samplerate = 16000;
        pa.read_cache_size = 4;
        pa.write_cache_size = 2048;
    }
    else
    {
        pa.write_samplerate = 44100;
        pa.read_samplerate = 44100;
        pa.read_cache_size = 4;
        pa.write_cache_size = 2048;
    }
    steps = 0;
    g_client = audio_open(AUDIO_TYPE_BT_MUSIC, AUDIO_TX, &pa, NULL, NULL);
}


static void write_1k()
{
    rt_uint32_t evt = 0;
    int i = 0;
    double c = 2.0f * 3.14159265358979f * 1000.0f / (float)g_samplerate;
    double x;
    for (int i = 0; i < 160; i++)
    {
        x = 24000.0f * sin(c * steps);
        g_pcm[i] = (int16_t)x;
        steps++;
        steps = steps % g_samplerate;
        audio_dump_data(ADUMP_RAMP_IN_OUT, (uint8_t *)g_pcm, 320);
    }
    int ret = audio_write(g_client, (uint8_t *)g_pcm, 320);
}

static int callback_1k(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        write_1k();
    }
    return 0;
}
static void playing_1k(void)
{
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;

    if (g_samplerate == 16000)
    {
        pa.write_samplerate = 16000;
        pa.read_samplerate = 16000;
        pa.read_cache_size = 4;
        pa.write_cache_size = 4096;
    }
    else
    {
        pa.write_samplerate = 44100;
        pa.read_samplerate = 44100;
        pa.read_cache_size = 4;
        pa.write_cache_size = 4096 * 4;
    }

    steps = 0;
    g_client = audio_open(AUDIO_TYPE_BT_MUSIC, AUDIO_TX, &pa, callback_1k, NULL);
    write_1k();
    write_1k();
    write_1k();
    write_1k();
    write_1k();
    write_1k();
    write_1k();
    write_1k();
}


static void audio_thread_test(void *p)
{
    uint32_t last_event = 0xFFFFFFFF;

    while (1)
    {
        rt_uint32_t evt = 0;
        rt_event_recv(&event, TEST_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

        if (g_mp3)
        {
            mp3ctrl_close(g_mp3);
            g_mp3 = NULL;
        }
        if (g_client)
        {
            audio_close(g_client);
            g_client = NULL;
        }

        if (evt & TEST_E_1K)
        {
            if (g_client)
            {
                audio_close(g_client);
            }
            playing_1k();
        }
        if (evt & TEST_E_0)
        {
            if (g_client)
            {
                audio_close(g_client);
            }
            playing_0();
        }
        if (evt & TEST_E_FILE)
        {
            playing_file();
        }

        if (evt & TEST_E_R_P)
        {
            record_play();
        }
        if (evt & TEST_E_R)
        {
            record_start();
        }

        if (evt & TEST_E_STOP)
        {
            record_stop();
            g_test_eq_enable = 2;
        }
    }

}


static int audio_test_api_init(void)
{
    g_test_3a_enable = 1;
    g_fd = -1;
    g_pcm = malloc(4096);
    RT_ASSERT(g_pcm);
    rt_mutex_init(&mutex, "audio_evt", RT_IPC_FLAG_FIFO);
    rt_event_init(&event, "audtest", RT_IPC_FLAG_FIFO);
    rt_thread_t tid = rt_thread_create("audiotest", audio_thread_test, RT_NULL, 2048, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(tid);
    rt_thread_startup(tid);

    return 0;
}

INIT_ENV_EXPORT(audio_test_api_init);



static int audio_test_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        write(fd, p->data, p->data_len);
    }
    return 0;
}
static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (fd >= 0 && g_client)
        {
            read(fd, (void *)g_pcm, 2048);
            int writted = audio_write(g_client, (uint8_t *)g_pcm, 2048);
        }
    }
    return 0;
}
static void record_play(void)
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
    fd = open(FILE_RECORD, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(fd >= 0);
    g_fd = fd;
    g_client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, audio_test_callback_record, (void *)fd);
    RT_ASSERT(g_client);

    while (record_seconds < RECORD_TIME_SECOND)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(g_client);
    close(fd);
    g_fd = -1;


    //play now
    pa.write_cache_size = 4096;
    fd = open(FILE_RECORD, O_RDONLY | O_BINARY);
    RT_ASSERT(fd >= 0);

    g_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &pa, audio_callback_play, (void *)fd);
    RT_ASSERT(g_client >= 0);
    read(fd, (void *)g_pcm, 4096);
    audio_write(g_client, (uint8_t *)g_pcm, 4096);
    record_seconds = 0;
    while (record_seconds < RECORD_TIME_SECOND)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }

    audio_close(g_client);
    close(fd);
    unlink(FILE_RECORD);
}



static int audio_callback_record_start(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        if (fd >= 0)
        {
            write(fd, p->data, p->data_len);
        }
    }
    else if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (g_bg_playing_fd >= 0 && g_client)
        {
            int bytes = read(g_bg_playing_fd, (void *)g_pcm, 2048);
            if (bytes > 0)
            {
                audio_write(g_client, (uint8_t *)g_pcm, 2048);
            }
        }
    }
    return 0;
}
static void record_start(void)
{
    audio_rwflag_t rwflag = AUDIO_RX;
    int fd = -1;
    g_fd = -1;
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
    if (!g_dump)
    {
        fd = open(FILE_RECORD, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
        RT_ASSERT(fd >= 0);
        g_fd = fd;
    }
    else
    {
        audio_dump_enable(ADUMP_AUDPRC);
    }
    if (g_playing)
    {
        rwflag = AUDIO_TXRX;
        g_bg_playing_fd = open(FILE_PLAYING_WHEN_RECORD, O_RDONLY | O_BINARY);
    }
    g_client = audio_open(AUDIO_TYPE_LOCAL_RECORD, rwflag, &pa, audio_callback_record_start, (void *)fd);
    RT_ASSERT(g_client);
}

static void record_stop(void)
{
    if (g_fd >= 0)
    {
        close(g_fd);
        g_fd = -1;
    }
    if (g_dump)
    {
        g_dump = 0;
        audio_dump_clear();
        unlink(FILE_RECORD);
    }

    if (g_bg_playing_fd >= 0)
    {
        close(g_bg_playing_fd);
        g_bg_playing_fd = -1;
    }
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
