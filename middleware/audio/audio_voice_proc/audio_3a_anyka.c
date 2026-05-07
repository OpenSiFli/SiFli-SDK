/*
 * SPDX-FileCopyrightText: 2020-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#ifdef SOC_BF0_HCPU
#include <string.h>
#include <stdlib.h>
#include "audioproc.h"
#include "ipc/ringbuffer.h"
#include "bf0_mbox_common.h"
#include "ipc/dataqueue.h"
#include "drivers/audio.h"
#include "dfs_file.h"
#include "dfs_posix.h"

#include "audio_mem.h"
#define DBG_TAG           "a3a"
#define DBG_LVL           AUDIO_DBG_LVL //LOG_LVL_WARNING
#include "log.h"
#include "audio_server.h"

#include "audio_3a_anyka.h"

#if ANYKA_RUN_IN_ACPU
    #include "acpu_ctrl.h"
#endif

/*
    sometime audio_3a_uplink() const more than 10ms for dual mic, so use a thread to processl data only,
    processed data in jitter buffer cache, m_jitterBufLen should big enough to cache processed data

*/
#define INPUT_THREAD_NAME   "anyka00"

#define INPUT_EVT_DATA      (1 << 0)
#define INPUT_EVT_EXIT      (1 << 1)

typedef struct audio_3a_tag
{
    uint8_t     state;
    uint8_t     is_bt_voice;
    uint8_t     is_far_putted;
    uint8_t     input_frames;
    uint8_t     all_mic_channels;
    uint16_t    samplerate;
    uint8_t    *rbuf_dwlink_pool;
    uint8_t    *mic_far;
    rt_event_t  input_evt;
    struct rt_ringbuffer *anyka_input;
    struct rt_ringbuffer *anyka_output;

    struct rt_ringbuffer rbuf_dwlink;
    struct rt_ringbuffer *rbuf_out;

    T_pSD_PARAM_FACTORY factory_far;
    T_pSD_PARAM_FACTORY factory_near;
    void                *p_far;
    void                *p_near;
    T_AUDIO_FILTER_TS   ts_far;
    T_AUDIO_FILTER_TS   ts_dac_stream;
} audio_3a_t;

static audio_3a_t g_audio_3a_env =
{
    .state  = 0,
    .samplerate = 16000,
};

static uint8_t g_bypass;  /* for factory test */
static uint8_t g_mic_chhose; // 0---mic1_left; 1---mic1_right; 2---mic2_left; 3---mic2_right


/*
    How many samples need to be inserted at the very beginning of the reference signal
    Only then can the algorithm ensure that the MIC signal seen is delayed by DELAY_SAMPLE samples compared to the reference signal
*/
static uint16_t g_mic_delay_ref = 352;

static const char factory_far[] =
{
    "--eqLoad=0"
    " --eqmode=user"
    " --preGain=1dB"
    " --bands=\"4, hpf 100 0dB 0.8 enable, hpf 100 0dB 0.8 enable, pf1 600 -12dB 1 enable, pf1 3000 6dB 1 enable\""
    " --nrEna=1"
    " --noiseSuppressDb=-20"
    " --volLoad=1"
    " --limit=0.80FS"
    " --vol_dB=5dB"
};

static const char factory_near_1mic[] =
{
    "--eqLoad=1"
    " --eqmode=user"
    " --preGain=0dB"
    " --bands=\"4, hpf 800 0dB 0.85 enable, pf1 2500 -9dB 3 enable, pf1 3000 -9dB 3 enable, hpf 800 0dB 0.85 enable\""
    " --aecEna=1"
    " --tail=512"
    " --farDigiGain=1.0"
    " --nearDigiGain=1.0"
    " --farLimit=0.15FS"
    " --farBreakdownThresh=0"
    " --nrEna=1"
    " --noiseSuppressDb=-40"
    " --agcEna=1"
    " --agcLevel=0.25FS"
    " --maxGain=4"
    " --minGain=0.1"
    " --nearSensitivity=20"
    " --volLoad=1"
    " --limit=0.85FS"
    " --vol_dB=3dB"
};

static const char factory_near_2mic[] =
{
    "--eqLoad=1"
    " --eqmode=user"
    " --preGain=0dB"
    " --bands=\"4, hpf 800 0dB 0.85 enable, pf1 2500 -9dB 3 enable, pf1 3000 -9dB 3 enable, hpf 800 0dB 0.85 enable\""
    " --aecEna=1"
    " --tail=512"
    " --farDigiGain=1.0"
    " --nearDigiGain=1.0"
    " --farLimit=0.15FS"
    " --farBreakdownThresh=0"
    " --nrEna=1"
    " --noiseSuppressDb=-40"
    " --agcEna=1"
    " --agcLevel=0.25FS"
    " --maxGain=4"
    " --minGain=0.1"
    " --nearSensitivity=20"
    " --volLoad=1"
    " --limit=0.85FS"
    " --vol_dB=3dB"
    " --dencLoad=1"
    " --arrayGeometry=\"2, 0 0 0, 0.036 0 0\""
    " --targetSpherical=\"3 0 1\""
    " --interfSphericals=\"3, 1 0 2, 2 0 2, 0 0 2\""
};

static const char factory_near_4mic[] =
{
    "--eqLoad=1"
    " --eqmode=user"
    " --preGain=0dB"
    " --bands=\"4, hpf 800 0dB 0.85 enable, pf1 2500 -9dB 3 enable, pf1 3000 -9dB 3 enable, hpf 800 0dB 0.85 enable\""
    " --aecEna=1"
    " --tail=512"
    " --farDigiGain=1.0"
    " --nearDigiGain=1.0"
    " --farLimit=0.15FS"
    " --farBreakdownThresh=0"
    " --nrEna=1"
    " --noiseSuppressDb=-40"
    " --agcEna=1"
    " --agcLevel=0.25FS"
    " --maxGain=4"
    " --minGain=0.1"
    " --nearSensitivity=20"
    " --volLoad=1"
    " --limit=0.85FS"
    " --vol_dB=3dB"
    " --dencLoad=1"
    " --arrayGeometry=\"4, 0.035 0.018 0, 0.035 -0.018 0, -0.035 0.018 0, -0.035 -0.018 0\""
    " --targetSpherical=\"3 0 1\""
    " --interfSphericals=\"3, 1 0 2, 2 0 2, 0 0 2\""
};

static void input_thread_entry(void *parameter);

static void disable_parameter(char *src, uint8_t is_bt_voice, uint8_t disable_uplink_agc)
{
#if 0
    char *p;
    if (!is_bt_voice)
    {
        p = strstr(src, "--aecEna=1");
        if (p)
        {
            memcpy(p, "--aecEna=0", strlen("--aecEna=0"));
        }
    }
    if (disable_uplink_agc)
    {
        p = strstr(src, "--agcEna=1");
        if (p)
        {
            memcpy(p, "--agcEna=0", strlen("--agcEna=0"));
        }
    }
#endif
}

static void audio_3a_module_init(audio_3a_t *env, uint32_t samplerate)
{
    uint32_t size = ANYKA_FRAME_SIZE * 2;
    env->mic_far = audio_mem_calloc(1, size + g_mic_delay_ref * 2); /* 2 frame + delay*/
    RT_ASSERT(env->mic_far);
    env->rbuf_dwlink_pool = audio_mem_malloc(size);
    RT_ASSERT(env->rbuf_dwlink_pool);
    rt_ringbuffer_init(&env->rbuf_dwlink, env->rbuf_dwlink_pool, size);
    env->rbuf_out = rt_ringbuffer_create(ANYKA_FRAME_SIZE * 2);
    RT_ASSERT(env->rbuf_out);
    env->state = 1;
}

static void audio_3a_module_free(audio_3a_t *env)
{
    env->state = 0;
    rt_ringbuffer_reset(&env->rbuf_dwlink);
    audio_mem_free(env->rbuf_dwlink_pool);
    env->rbuf_dwlink_pool = NULL;
    rt_ringbuffer_destroy(env->rbuf_out);
    env->rbuf_out = NULL;
    audio_mem_free(env->mic_far);
    env->mic_far = NULL;
    rt_ringbuffer_destroy(env->anyka_input);
    env->anyka_input = NULL;
    rt_ringbuffer_destroy(env->anyka_output);
    env->anyka_output = NULL;
}

static void t4_flush_dcache_range(uint32_t start, uint32_t size)
{
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)start, size);
}

static T_VOID my_printf(T_pCSTR fmt, ...)
{
#if 1
    char str[128];
    rt_int32_t n;
    va_list args;
    va_start(args, fmt);
    n = rt_vsnprintf(str, sizeof(str) - 1, fmt, args);
    va_end(args);
    LOG_I("%s", str);
#endif
}
static void my_notify(T_ECHO_EVENT event, T_pVOID param, T_HANDLE cb_notify_param, T_U8 pathId)
{
    //rt_kprintf("~~~~notify: 0x%x\n", code);
}

RT_WEAK void anyka_dump_data(T_ECHO_DUMP_ITEM_ID item_id, T_S32 size, T_pCVOID data, T_pCVOID meta, T_HANDLE cb_dump_param, T_U8 pathId)
{

}
static void my_dump(T_ECHO_DUMP_ITEM_ID item_id, T_S32 size, T_pCVOID data, T_pCVOID meta, T_HANDLE cb_dump_param, T_U8 pathId)
{
    anyka_dump_data(item_id, size, data, meta, cb_dump_param, pathId);
}

static T_pVOID my_malloc(T_U32 size)
{
    return audio_mem_malloc(size);
}
static void my_free(T_pVOID p)
{
    audio_mem_free(p);
}


void audio_3a_set_bypass(uint8_t is_bypass, uint8_t mic, uint8_t down)
{
    g_bypass = is_bypass;
    g_mic_chhose = mic;
}

void audio_3a_open(uint32_t samplerate, uint8_t is_bt_voice, uint8_t disable_uplink_agc, uint8_t all_mic_channels)
{
    audio_3a_t *thiz = &g_audio_3a_env;

    if (thiz->state == 0)
    {
        int ret;
        const char *near = factory_near_1mic;
        thiz->all_mic_channels = all_mic_channels;
        if (all_mic_channels == 2)
        {
            near = factory_near_2mic;
        }
        else if (all_mic_channels == 4)
        {
            near = factory_near_4mic;
        }
        LOG_I("3a_w open");
        thiz->input_frames = 0;
        thiz->anyka_input = rt_ringbuffer_create(ANYKA_FRAME_SIZE * ANYKA_CACHED_FRMAES * all_mic_channels);
        RT_ASSERT(thiz->anyka_input);
        thiz->anyka_output = rt_ringbuffer_create(ANYKA_FRAME_SIZE * (ANYKA_CACHED_FRMAES + 1));
        RT_ASSERT(thiz->anyka_output);

        thiz->input_evt = rt_event_create("anyka_0", RT_IPC_FLAG_FIFO);
        RT_ASSERT(thiz->input_evt);
        rt_thread_t tid = rt_thread_create(INPUT_THREAD_NAME, input_thread_entry, thiz, 4096, RT_THREAD_PRIORITY_HIGH + RT_THREAD_PRIORITY_HIGHER, RT_THREAD_TICK_DEFAULT);
        RT_ASSERT(tid);
        rt_thread_startup(tid);

        if (samplerate == 8000)
        {
            thiz->samplerate = 8000;
        }
        else
        {
            thiz->samplerate = 16000;
        }

        audio_3a_module_init(thiz, thiz->samplerate);

#if ANYKA_RUN_IN_ACPU
        uint8_t error_code = 1;
        acpu_audio_3a_open_parameter_t arg;
        /* acpu can't access hcpu const var if not in psram */
        arg.const_far = audio_mem_malloc(sizeof(factory_far) + 1);
        RT_ASSERT(arg.const_far);
        strcpy(arg.const_far, factory_far);
        arg.const_near = audio_mem_malloc(strlen(factory_near_1mic) + 1);
        RT_ASSERT(arg.const_near);
        strcpy(arg.const_near, near);
        disable_parameter(arg.const_near, is_bt_voice, disable_uplink_agc);
        arg.samplerate = thiz->samplerate;
        arg.all_mic_channels = all_mic_channels;
        arg.is_bt_voice = is_bt_voice;

        SCB_CleanInvalidateDCache();
        acpu_run_task(ACPU_TASK_audio_3a_open, &arg, sizeof(arg), &error_code);
        RT_ASSERT(error_code == 0);
        audio_mem_free(arg.const_far);
        audio_mem_free(arg.const_near);
#else
        thiz->ts_far = 0;
        thiz->ts_dac_stream = 0;
        thiz->is_far_putted = 0;
        thiz->is_bt_voice = is_bt_voice;
        LOG_I("3a_w open samplerate=%ld", samplerate);

        T_SDLIB_PLATFORM_DEPENDENT_LIST *sd_cb;

        sd_cb = _SD_GetPlatformDependentList();

        sd_cb->Malloc = (MEDIALIB_CALLBACK_FUN_MALLOC)my_malloc;
        sd_cb->Free = (MEDIALIB_CALLBACK_FUN_FREE)my_free;
        sd_cb->printf = (MEDIALIB_CALLBACK_FUN_PRINTF)my_printf;
        sd_cb->flushDCache = t4_flush_dcache_range;

        // set debug level
        SD_ParamFactory_SetDebugZones(SD_DEFAULT_DEBUG_ZONES | SD_ZONE_ID_PARAM/*SD_ZONE_ID_VERBOSE*/);

        //_SD_Echo_PrintFarPathParamHelp();

        // 1. echo init
        T_ECHO_IN_INFO echo_in;
        memset(&echo_in, 0, sizeof(echo_in));
        echo_in.m_SampleRate = thiz->samplerate;
        echo_in.m_hwSampleRate = thiz->samplerate;

        echo_in.strVersion = AUDIO_FILTER_VERSION_STRING;
        echo_in.cb_fun.Malloc = (MEDIALIB_CALLBACK_FUN_MALLOC)my_malloc;
        echo_in.cb_fun.Free = (MEDIALIB_CALLBACK_FUN_FREE)my_free;
        echo_in.cb_fun.Printf = (MEDIALIB_CALLBACK_FUN_PRINTF)my_printf;
        echo_in.cb_fun.flushDCache = t4_flush_dcache_range;
        echo_in.cb_fun.notify = my_notify;
        echo_in.cb_fun.dump = my_dump;
        echo_in.m_Channels = 1;
        echo_in.m_BitsPerSample = 16;
        echo_in.m_jitterBufLen = (ANYKA_CACHED_FRMAES + 1) * ANYKA_FRAME_SIZE * 4; /*max 4 mic */

        echo_in.m_SampleRate = thiz->samplerate;
        echo_in.m_hwSampleRate = thiz->samplerate;

        // 2.  open far path
        if (is_bt_voice)
        {
            echo_in.m_pathId = 1;
            echo_in.m_mode = ECHO_MODE_AO_NORMAL;
            echo_in.cb_notify_param = (T_HANDLE)&thiz->p_far;
            thiz->p_far = _SD_Echo_Open(&echo_in);
            RT_ASSERT(thiz->p_far);

            thiz->factory_far = SD_ParamFactory_Create_ByCmdLine(factory_far, sizeof(factory_far));
            RT_ASSERT(thiz->factory_far);
            _SD_Echo_SetFarPathParam(thiz->p_far, thiz->factory_far);
        }

        // 3. open near path
        echo_in.m_pathId = 0;
        echo_in.m_syncedInputs = 0;
        echo_in.m_syncBufLen = 12000;
        echo_in.m_syncCompensate = DELAY_SAMPLE;
        if (all_mic_channels > 1)
            echo_in.m_mode = ECHO_MODE_AI_MIC_ARRAY_PROCESS;
        else
            echo_in.m_mode = ECHO_MODE_AI_NORMAL;

        echo_in.cb_notify_param = (T_HANDLE)&thiz->p_near;
        thiz->p_near = _SD_Echo_Open(&echo_in);
        RT_ASSERT(thiz->p_near);

        _SD_EQ_login(AK_NULL);
        _SD_NR_login(AK_NULL);
        _SD_AEC_login(AK_NULL);
        _SD_ASLC_login(AK_NULL);
        if (all_mic_channels > 1)
        {
            _SD_Denc_login(AK_NULL);
        }


        char *near_new = audio_mem_malloc(strlen(near) + 1);
        RT_ASSERT(near_new);
        strcpy(near_new, near);
        disable_parameter(near_new, is_bt_voice, disable_uplink_agc);
        thiz->factory_near = SD_ParamFactory_Create_ByCmdLine(near_new, strlen(near_new) + 1);
        RT_ASSERT(thiz->factory_near);
        audio_mem_free(near_new);
        ret = _SD_Echo_SetNearPathParam(thiz->p_near, thiz->factory_near);

#if 0
        char *strVal;
        /* can use user defined parameter, example my_paramter=3 */
        strVal = SD_ParamFactory_GetValue(thiz->factory_near, "my_paramter");
        if (strVal)
        {
            rt_kprintf("my_paramter=%d\n", atoi(strVal));
        }
#endif
        struct echo_param_aec aec_param;
        memset(&aec_param, 0, sizeof(aec_param));
        _SD_Echo_GetAecParam(thiz->p_near, &aec_param);
#endif
        if (is_bt_voice)
            bt_voice_open(samplerate);
    }
}

void audio_3a_close()
{
    audio_3a_t *thiz = &g_audio_3a_env;
    if (thiz->state == 1)
    {
        LOG_I("3a_w close");
        rt_event_send(thiz->input_evt, 2);
        while (rt_thread_find(INPUT_THREAD_NAME))
        {
            rt_thread_mdelay(10);
            LOG_I("kill anyka");
        }
        rt_event_delete(thiz->input_evt);

        audio_3a_module_free(thiz);
        if (thiz->is_bt_voice)
            bt_voice_close();

#if ANYKA_RUN_IN_ACPU
        uint8_t error_code = 1;
        SCB_CleanInvalidateDCache();
        acpu_run_task(ACPU_TASK_audio_3a_close, NULL, 0, &error_code);
        RT_ASSERT(error_code == 0);
#else
        if (thiz->p_far)
        {
            _SD_Echo_Reset(thiz->p_far);
            _SD_Echo_Close(thiz->p_far);
            thiz->p_far = NULL;
        }
        if (thiz->p_near)
        {
            _SD_Echo_Reset(thiz->p_near);
            _SD_Echo_Close(thiz->p_near);
            thiz->p_near = NULL;
        }
        if (thiz->factory_far)
        {
            SD_ParamFactory_Destroy(thiz->factory_far);
            thiz->factory_far = NULL;
        }
        if (thiz->factory_near)
        {
            SD_ParamFactory_Destroy(thiz->factory_near);
            thiz->factory_near = NULL;
        }
#endif
    }
}

uint8_t audio_3a_dnlink_buf_is_full(uint8_t size)
{
    audio_3a_t *env = &g_audio_3a_env;

    if (rt_ringbuffer_space_len(&env->rbuf_dwlink) <= size)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void audio_3a_downlink(uint8_t *fifo, uint8_t size)
{
    audio_3a_t *thiz = &g_audio_3a_env;
    uint16_t putsize, getsize;
    if (thiz->state == 0)
    {
        LOG_I("3a_w downlink error: closed");
        return;
    }

    uint8_t  data1[ANYKA_FRAME_SIZE];
    uint8_t  data2[ANYKA_FRAME_SIZE];

    uint8_t  *data_in, *data_out;
    if (rt_ringbuffer_space_len(&thiz->rbuf_dwlink) >= size)
    {
        putsize = rt_ringbuffer_put(&thiz->rbuf_dwlink, fifo, size);
        RT_ASSERT(putsize == size);
    }
    else
    {
        LOG_I("3a_w rbuf_dwlink full");
        putsize = rt_ringbuffer_put_force(&thiz->rbuf_dwlink, fifo, size);
    }

    while (rt_ringbuffer_data_len(&thiz->rbuf_dwlink) >= ANYKA_FRAME_SIZE)
    {
        data_in = data1;
        data_out = data2;
        getsize = rt_ringbuffer_get(&thiz->rbuf_dwlink, (uint8_t *)data_in, ANYKA_FRAME_SIZE);

        audio_dump_data(ADUMP_DOWNLINK, data_in, ANYKA_FRAME_SIZE);

        // todo: change to use HAL_HPAON_READ_GTIMER();
        rt_tick_t tick = rt_tick_get();
#if ANYKA_RUN_IN_ACPU
        uint8_t error_code = 1;
        acpu_audio_3a_downlink_parameter_t arg;
        arg.data_in = data_in;
        arg.data_out = data_out;
        arg.tick = tick;
        SCB_CleanInvalidateDCache();
        acpu_run_task(ACPU_TASK_audio_3a_downlink, &arg, sizeof(arg), &error_code);
        RT_ASSERT(error_code == 0);
#else
        getsize = _SD_Echo_FillFarStream(thiz->p_far, data_in, ANYKA_FRAME_SIZE, tick * 1000, 1);
        //RT_ASSERT(getsize == ANYKA_FRAME_SIZE);
        if (!getsize)
        {
            LOG_I("far full");
            continue;
        }
        getsize = _SD_Echo_GetDacStream(thiz->p_far, data_out, ANYKA_FRAME_SIZE, &thiz->ts_dac_stream, 1);

        if (!getsize)
        {
            LOG_I("dac empty");
            continue;
        }
#endif
        audio_dump_data(ADUMP_DOWNLINK_AGC, data_out, ANYKA_FRAME_SIZE);

        //put to speaker buffer for playing
        speaker_ring_put(data_out, ANYKA_FRAME_SIZE);
    }

}

void audio_3a_far_put(uint8_t *fifo, uint16_t fifo_size)
{
    audio_3a_t *env = &g_audio_3a_env;
    if (env->state == 0)
    {
        LOG_I("3a far put: closed");
        return;
    }
    RT_ASSERT(fifo_size == ANYKA_FRAME_SIZE);
#if DEBUG_FRAME_SYNC
    RT_ASSERT(env->is_far_using == 0);
#endif
    memcpy(env->mic_far + g_mic_delay_ref * 2, fifo, fifo_size);
#if DEBUG_FRAME_SYNC
    RT_ASSERT(env->is_far_using == 0);
#endif
    env->is_far_putted = 1;
}

static void dump_channel(uint8_t *dst_u8, uint8_t *src_u8, int step_int16, audio_dump_type_t type)
{
    int16_t *d = (int16_t *)dst_u8;
    int16_t *s = (int16_t *)src_u8;

    for (int i = 0; i < ANYKA_FRAME_SIZE / 2; i++)
    {
        *d++ = *s;
        s += step_int16;
    }
    audio_dump_data(type, dst_u8, ANYKA_FRAME_SIZE);
}

static inline void process_data(audio_3a_t *thiz)
{
    int ret;
    uint8_t result[ANYKA_FRAME_SIZE];
    uint8_t refframe[ANYKA_FRAME_SIZE];
    uint8_t fifo[ANYKA_FRAME_SIZE * 4];
    uint16_t fifo_size = thiz->all_mic_channels * ANYKA_FRAME_SIZE;

    if (0 == thiz->is_far_putted && thiz->is_bt_voice)
    {
        LOG_I("wait far put");
    }
#if DEBUG_FRAME_SYNC
    thiz->is_far_using = 1;
#endif
    memcpy(refframe, thiz->mic_far, ANYKA_FRAME_SIZE);
    memcpy(thiz->mic_far, thiz->mic_far + ANYKA_FRAME_SIZE, g_mic_delay_ref * 2);
#if DEBUG_FRAME_SYNC
    thiz->is_far_using = 0;
#endif
    rt_ringbuffer_get(thiz->anyka_input, fifo, fifo_size);

    audio_dump_data(ADUMP_AECM_INPUT1, (uint8_t *)refframe, ANYKA_FRAME_SIZE);
    if (thiz->all_mic_channels == 1)
    {
        audio_dump_data(ADUMP_AUDPRC, fifo, fifo_size);
        audio_dump_data(ADUMP_AECM_INPUT2, fifo, ANYKA_FRAME_SIZE);
    }
    else if (thiz->all_mic_channels == 2)
    {
        dump_channel(result, fifo + 0, 2, ADUMP_AUDPRC);
        dump_channel(result, fifo + 2, 2, ADUMP_AECM_INPUT2);
    }
    else if (thiz->all_mic_channels == 4)
    {
        dump_channel(result, fifo + 0, 4, ADUMP_AUDPRC);
        dump_channel(result, fifo + 2, 4, ADUMP_DC_OUT);
        dump_channel(result, fifo + 4, 4, ADUMP_AECM_INPUT2);
        dump_channel(result, fifo + 6, 4, ADUMP_RAMP_IN_OUT);
    }

    // todo: change to use HAL_HPAON_READ_GTIMER();
    uint64_t ts = rt_tick_get() * 1000;

#if ANYKA_RUN_IN_ACPU
    uint8_t error_code = 1;
    acpu_audio_3a_uplink_parameter_t arg;
    arg.refframe = refframe;
    arg.ts = ts;
    arg.fifo = fifo;
    arg.result = result;
    SCB_CleanInvalidateDCache();
    acpu_run_task(ACPU_TASK_audio_3a_uplink, &arg, sizeof(arg), &error_code);
    RT_ASSERT(error_code == 0);
#else
    ret = _SD_Echo_FillDacLoopback(thiz->p_near, (uint8_t *)refframe, ANYKA_FRAME_SIZE, ts, 1);

    //LOG_I("fill dac loopback=%d", ret);

    ts += DELAY_SAMPLE * 1000000ULL / thiz->samplerate;
    ret = _SD_Echo_FillAdcStream(thiz->p_near, fifo, fifo_size, ts, 1);

    //LOG_I("fill adc=%d", ret);
    T_AUDIO_FILTER_TS ts_result;
    memset(result, 0, ANYKA_FRAME_SIZE);
    ret = _SD_Echo_GetResult(thiz->p_near, result, sizeof(result), &ts_result, 1);
    //LOG_I("fill adc=%d", ret);
#endif
    rt_ringbuffer_put(thiz->anyka_output, result, ANYKA_FRAME_SIZE);
    audio_dump_data(ADUMP_RAMP_OUT_OUT, result, ANYKA_FRAME_SIZE);
    if (thiz->input_frames < ANYKA_CACHED_FRMAES)
        thiz->input_frames++;
}

static void input_thread_entry(void *parameter)
{
    audio_3a_t *thiz = (audio_3a_t *)parameter;
    while (1)
    {
        rt_uint32_t evt = 0;
        rt_event_recv(thiz->input_evt, 3, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        if (evt & INPUT_EVT_EXIT)
        {
            break;
        }
        if (evt & INPUT_EVT_DATA)
        {
            process_data(thiz);
            if (rt_ringbuffer_data_len(thiz->anyka_input) >= ANYKA_FRAME_SIZE * thiz->all_mic_channels)
            {
                rt_event_send(thiz->input_evt, INPUT_EVT_DATA);
            }
        }
    }
}

void audio_3a_uplink(uint8_t *fifo, uint16_t fifo_size, uint8_t is_mute, uint8_t is_bt_voice)
{
    int ret;
    uint8_t result[ANYKA_FRAME_SIZE];

    audio_3a_t *thiz = &g_audio_3a_env;
    RT_ASSERT(fifo_size == ANYKA_FRAME_SIZE * thiz->all_mic_channels);

    if (g_bypass)
    {
        int16_t *src = (int16_t *)fifo;
        int16_t *dst = (int16_t *)result;
        int step = thiz->all_mic_channels;

        src += g_mic_chhose;
        for (int i = 0; i < ANYKA_FRAME_SIZE / 2; i++)
        {
            dst[i] = *src;
            src += step;
        }
        if (is_mute)
        {
            memset(result, 0, sizeof(result));
        }
        rt_ringbuffer_put(thiz->rbuf_out, result, ANYKA_FRAME_SIZE);
    }
    else
    {
        rt_size_t w = rt_ringbuffer_put(thiz->anyka_input, fifo, fifo_size);
        if (w != fifo_size)
        {
            LOG_I("anya ring full");
        }
        rt_event_send(thiz->input_evt, INPUT_EVT_DATA);

        if (thiz->input_frames < ANYKA_CACHED_FRMAES)
        {
            return;
        }
        rt_size_t got = rt_ringbuffer_get(thiz->anyka_output, result, ANYKA_FRAME_SIZE);
        if (got != ANYKA_FRAME_SIZE)
        {
            LOG_E("anyka underrun");
        }
        if (is_mute)
        {
            memset(result, 0, sizeof(result));
        }
        rt_ringbuffer_put(thiz->rbuf_out, result, ANYKA_FRAME_SIZE);
        if (!is_bt_voice)
        {
            rt_ringbuffer_get(thiz->rbuf_out, fifo, ANYKA_FRAME_SIZE);
            return;
        }
    }

    if (thiz->samplerate == 8000)
    {
        while (rt_ringbuffer_data_len(thiz->rbuf_out) >= 120)
        {
            rt_ringbuffer_get(thiz->rbuf_out, result, 120);
#ifdef AUDIO_BT_AUDIO
            msbc_encode_process(result, 120);
#endif
        }
    }
    else
    {
        while (rt_ringbuffer_data_len(thiz->rbuf_out) >= 240)
        {
            rt_ringbuffer_get(thiz->rbuf_out, result, 240);
#ifdef AUDIO_BT_AUDIO
            msbc_encode_process(result, 240);
#endif
        }
    }
}



__WEAK void audio_tick_in(uint8_t type)
{
}
__WEAK void audio_tick_out(uint8_t type)
{
}
__WEAK void audio_time_print(void)
{
}
__WEAK void audio_uplink_time_print(void)
{
}
__WEAK void audio_dnlink_time_print(void)
{
}

#ifdef FFT_USING_ONCHIP
    #error "Anyka not using onchip fft"
#endif
uint32_t g_record_time = 10;
void audio_command_process(uint8_t *cmd_1)
{

}


#endif

