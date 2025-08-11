/*
 * SPDX-FileCopyrightText: 2020-2021 SiFli Technologies(Nanjing) Co., Ltd
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
#define DBG_TAG           "audio_3a"
#define DBG_LVL           AUDIO_DBG_LVL //LOG_LVL_WARNING
#include "log.h"
#include "audio_server.h"

#include "sdfilter.h"
#include "sdParamFactory.h"

#define ANYKA_FRAME_SIZE    320 //must same as CODEC_DATA_UNIT_LEN in audio_server.c

typedef struct _vad_instance
{
    T_U32 vadType;
    char *name;
    int32_t ts_active_start; // -1: not start, other: start ts
} t_vad_instance;

typedef struct audio_3a_tag
{
    uint8_t     state;
    uint8_t     is_bt_voice;
    uint8_t     is_far_putted;
    uint16_t    samplerate;
    uint16_t    frame_size;
    uint8_t    *rbuf_dwlink_pool;
    uint8_t    *mic_far;
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

static t_vad_instance vad1;
static struct echo_param_vad vad_param =
{

};

/*
    ×îºóËã·¨¿´µ½µÄmicÐÅºÅ±È²Î¿¼ÐÅºÅÍí¶àÉÙ¸ö²ÉÑù,
    µ÷Õûg_mic_delay_ref£¬±£Ö¤Êµ¼Ê²âÁ¿³öÀ´ºÍÕâ¸öÖµÒ»ÖÂ
*/
#define DELAY_SAMPLE    5

/*
   ÐèÒª°Ñ²Î¿¼ÐÅºÅ×îÇ°Ãæ²åÈë¶àÉÙ¸ö²ÉÑù£
   ²ÅÄÜ°Ñ±£Ö¤Ëã·¨¿´µ½µÄmicÐÅºÅ±È²Î¿¼ÐÅºÅÑÓ³ÙDELAY_SAMPLE
   ¸ö²ÉÑù
*/
static uint16_t g_mic_delay_ref = 434;

static const char factory_far[] =
{
    "--eqLoad=1"
    " --eqmode=user"
    " --preGain=1dB"
    " --bands=\"4, hpf 100 0dB 0.8 enable, hpf 100 0dB 0.8 enable, pf1 600 -12dB 1 enable, pf1 3000 6dB 1 enable\""
    " --nrEna=1"
    " --noiseSuppressDb=-20"
    " --volLoad=1"
    " --limit=0.15FS"
    " --vol_dB=2dB"
};

static const char factory_near[] =
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
    " --limit=1.0FS"
    " --vol_dB=0dB"
};

static void audio_3a_module_init(audio_3a_t *env, uint32_t samplerate)
{
    uint32_t size = ANYKA_FRAME_SIZE * 2;
    env->mic_far = audio_mem_malloc(size + g_mic_delay_ref * 2); /* 2 frame + delay*/
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
    audio_mem_free(env->rbuf_dwlink_pool);
    env->mic_far = NULL;
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

static void my_dump(T_ECHO_DUMP_ITEM_ID item_id, T_S32 size, T_pCVOID data, T_pCVOID meta, T_HANDLE cb_dump_param, T_U8 pathId)
{
    //printf("~~~~my_dump: %d %d\n", item_id, size);
    //if (item_id & ECHO_ITEM_PCM)
    //    dump_wav(item_id, size, data, (const T_ECHO_PCM_META *)meta);
}
static T_pVOID my_malloc(T_U32 size)
{
    return audio_mem_malloc(size);
}
static void my_free(T_pVOID p)
{
    audio_mem_free(p);
}

void audio_3a_open(uint32_t samplerate, uint8_t is_bt_voice, uint8_t disable_uplink_agc)
{
    audio_3a_t *thiz = &g_audio_3a_env;

    if (thiz->state == 0)
    {
        int ret;
        T_ECHO_IN_INFO echo_in;
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

        // 1. echo init
        memset(&echo_in, 0, sizeof(echo_in));
        echo_in.strVersion = AUDIO_FILTER_VERSION_STRING;
        echo_in.cb_fun.Malloc = (MEDIALIB_CALLBACK_FUN_MALLOC)my_malloc;
        echo_in.cb_fun.Free = (MEDIALIB_CALLBACK_FUN_FREE)my_free;
        echo_in.cb_fun.Printf = (MEDIALIB_CALLBACK_FUN_PRINTF)my_printf;
        echo_in.cb_fun.flushDCache = t4_flush_dcache_range;
        echo_in.cb_fun.notify = my_notify;
        echo_in.cb_fun.dump = my_dump;
        echo_in.m_Channels = 1;
        echo_in.m_BitsPerSample = 16;
        echo_in.m_jitterBufLen = 8192; //???

        if (samplerate == 8000)
        {
            echo_in.m_SampleRate = 8000;
            echo_in.m_hwSampleRate = 8000;
            thiz->samplerate = 8000;
            thiz->frame_size = ANYKA_FRAME_SIZE / 2;
            audio_3a_module_init(thiz, 8000);
        }
        else
        {
            echo_in.m_SampleRate = 16000;
            echo_in.m_hwSampleRate = 16000;
            thiz->samplerate = 16000;
            thiz->frame_size = ANYKA_FRAME_SIZE / 2;
            audio_3a_module_init(thiz, 16000);
        }

        // 2.  open far path
        echo_in.m_pathId = 1;
        echo_in.m_mode = ECHO_MODE_AO_NORMAL;
        echo_in.cb_notify_param = (T_HANDLE)&thiz->p_far;
        thiz->p_far = _SD_Echo_Open(&echo_in);
        RT_ASSERT(thiz->p_far);

        _SD_EQ_login(AK_NULL); //why no hanlde, how to avoid confilict bwtween AI and AO
        _SD_NR_login(AK_NULL);
        _SD_ASLC_login(AK_NULL);

        thiz->factory_far = SD_ParamFactory_Create_ByCmdLine(factory_far, sizeof(factory_far));

        // 3. open near path
        echo_in.m_pathId = 0;
        echo_in.m_syncedInputs = 0;
        echo_in.m_syncBufLen = 12000; //???
        echo_in.m_syncCompensate = DELAY_SAMPLE; //????
        echo_in.m_mode = ECHO_MODE_AI_NORMAL;
        echo_in.cb_notify_param = (T_HANDLE)&thiz->p_near;
        thiz->p_near = _SD_Echo_Open(&echo_in);
        RT_ASSERT(thiz->p_near);

        _SD_EQ_login(AK_NULL);
        _SD_NR_login(AK_NULL);
        _SD_AEC_login(AK_NULL);
        _SD_ASLC_login(AK_NULL);

        thiz->factory_near = SD_ParamFactory_Create_ByCmdLine(factory_near, sizeof(factory_near));

        ret = _SD_Echo_SetNearPathParam(thiz->p_near, thiz->factory_near);

        if (vad_param.vadType) // only set when type is valid
        {
            vad_param.userParam = &vad1;
            vad1.name = "instance_1";
            vad1.vadType = vad_param.vadType;
            vad1.ts_active_start = -1;
            ret = _SD_Echo_SetVadParam(thiz->p_near, 1, &vad_param);
            RT_ASSERT(ret);
        }
        char *strVal;
        // ²ÎÊýÁÐ±íÖÐÒ²¿ÉÒÔ°üº¬ÒôÆµ¿âÒÔÍâµÄ²ÎÊý£¬ÀýÈç ai_gain=3 ±íÊ¾ mic µÄÄ£ÄâÔöÒæÅäÖÃÎª3
        strVal = SD_ParamFactory_GetValue(thiz->factory_near, "ai_gain");
        if (strVal)
        {
            rt_kprintf("ai_gain=%d\n", atoi(strVal));
        }

        struct echo_param_aec aec_param;
        memset(&aec_param, 0, sizeof(aec_param));
        _SD_Echo_GetAecParam(thiz->p_near, &aec_param);

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
        audio_3a_module_free(thiz);
        if (thiz->is_bt_voice)
            bt_voice_close();

        _SD_Echo_Reset(thiz->p_far);
        _SD_Echo_Close(thiz->p_far);
        _SD_Echo_Reset(thiz->p_near);
        _SD_Echo_Close(thiz->p_near);
        SD_ParamFactory_Destroy(thiz->factory_far);
        SD_ParamFactory_Destroy(thiz->factory_near);
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

        getsize = _SD_Echo_FillFarStream(thiz->p_far, data_in, ANYKA_FRAME_SIZE, tick * 1000, 1);
        RT_ASSERT(getsize == ANYKA_FRAME_SIZE);
        getsize = _SD_Echo_GetDacStream(thiz->p_far, data_out, ANYKA_FRAME_SIZE, &thiz->ts_dac_stream, 1);
        RT_ASSERT(getsize == ANYKA_FRAME_SIZE);

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

void audio_3a_uplink(uint8_t *fifo, uint16_t fifo_size, uint8_t is_mute, uint8_t is_bt_voice)
{
    audio_3a_t *thiz = &g_audio_3a_env;
    int ret;

    int16_t refframe[ANYKA_FRAME_SIZE / 2];
    uint8_t result[ANYKA_FRAME_SIZE];

    RT_ASSERT(fifo_size == ANYKA_FRAME_SIZE);
    if (0 == thiz->is_far_putted)
    {
        LOG_I("wait far put");
        is_mute = 1;
        goto skip_3a_up;
    }
#if DEBUG_FRAME_SYNC
    thiz->is_far_using = 1;
#endif
    memcpy(refframe, thiz->mic_far, ANYKA_FRAME_SIZE);
    memcpy(thiz->mic_far, thiz->mic_far + ANYKA_FRAME_SIZE, g_mic_delay_ref * 2);
#if DEBUG_FRAME_SYNC
    thiz->is_far_using = 0;
#endif
    audio_dump_data(ADUMP_AECM_INPUT1, (uint8_t *)refframe, ANYKA_FRAME_SIZE);
    audio_dump_data(ADUMP_AECM_INPUT2, fifo, ANYKA_FRAME_SIZE);

    // todo: change to use HAL_HPAON_READ_GTIMER();
    uint64_t ts = rt_tick_get() * 1000;
    ret = _SD_Echo_FillDacLoopback(thiz->p_near, (uint8_t *)refframe, ANYKA_FRAME_SIZE, ts, 1);

    //LOG_I("fill dac loopback=%d", ret);

    ts += DELAY_SAMPLE * 1000000ULL / thiz->samplerate;
    ret = _SD_Echo_FillAdcStream(thiz->p_near, fifo, fifo_size, ts, 1);

    //LOG_I("fill adc=%d", ret);

    T_AUDIO_FILTER_TS ts_result;
    ret = _SD_Echo_GetResult(thiz->p_near, result, sizeof(result), &ts_result, 1);

    //LOG_I("fill adc=%d", ret);

skip_3a_up:

    if (is_mute)
    {
        memset(result, 0, ANYKA_FRAME_SIZE);
    }
    rt_ringbuffer_put(thiz->rbuf_out, result, ANYKA_FRAME_SIZE);

    if (!is_bt_voice)
    {
        rt_ringbuffer_get(thiz->rbuf_out, fifo, 320);
        return;
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

