/*
 * SPDX-FileCopyrightText: 2020-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include "acpu_ctrl.h"
#include "audio_3a_anyka.h"

static T_pSD_PARAM_FACTORY  g_factory_far;
static T_pSD_PARAM_FACTORY  g_factory_near;
static T_AUDIO_FILTER_TS    ts_far;
static T_AUDIO_FILTER_TS    ts_dac_stream;
static uint32_t             samplerate;
static void                 *p_far;
static void                 *p_near;

static T_VOID my_printf(T_pCSTR fmt, ...)
{
#if 0
    char str[128];
    rt_int32_t n;
    va_list args;
    va_start(args, fmt);
    n = vsnprintf(str, sizeof(str) - 1, fmt, args);
    va_end(args);
    print_buffer[128 - 1] = '\0';
    req_hcpu_run_task(HCPU_TASK_PRINTF, print_buffer, sizeof(print_buffer), NULL);
#endif
}

static void t4_flush_dcache_range(uint32_t start, uint32_t size)
{
    //SCB_CleanInvalidateDCache_by_Addr((uint32_t *)start, size);
}

static void my_notify(T_ECHO_EVENT event, T_pVOID param, T_HANDLE cb_notify_param, T_U8 pathId)
{
}

static void my_dump(T_ECHO_DUMP_ITEM_ID item_id, T_S32 size, T_pCVOID data, T_pCVOID meta, T_HANDLE cb_dump_param, T_U8 pathId)
{
}

int acpu_audio_3a_open(acpu_audio_3a_open_parameter_t *arg)
{
    int result;
    int ret = 0;
    T_SDLIB_PLATFORM_DEPENDENT_LIST *sd_cb;

    ts_far = 0;
    ts_dac_stream = 0;
    samplerate = arg->samplerate;

    sd_cb = _SD_GetPlatformDependentList();

    sd_cb->Malloc = (MEDIALIB_CALLBACK_FUN_MALLOC)malloc;
    sd_cb->Free = (MEDIALIB_CALLBACK_FUN_FREE)free;
    sd_cb->printf = (MEDIALIB_CALLBACK_FUN_PRINTF)my_printf;
    sd_cb->flushDCache = t4_flush_dcache_range;

    // set debug level
    SD_ParamFactory_SetDebugZones(SD_DEFAULT_DEBUG_ZONES | SD_ZONE_ID_PARAM/*SD_ZONE_ID_VERBOSE*/);

    //_SD_Echo_PrintFarPathParamHelp();

    // 1. echo init
    T_ECHO_IN_INFO echo_in;
    memset(&echo_in, 0, sizeof(echo_in));
    echo_in.strVersion = AUDIO_FILTER_VERSION_STRING;
    echo_in.cb_fun.Malloc = (MEDIALIB_CALLBACK_FUN_MALLOC)malloc;
    echo_in.cb_fun.Free = (MEDIALIB_CALLBACK_FUN_FREE)free;
    echo_in.cb_fun.Printf = (MEDIALIB_CALLBACK_FUN_PRINTF)my_printf;
    echo_in.cb_fun.flushDCache = t4_flush_dcache_range;
    echo_in.cb_fun.notify = my_notify;
    echo_in.cb_fun.dump = my_dump;
    echo_in.m_Channels = 1;
    echo_in.m_BitsPerSample = 16;
    echo_in.m_jitterBufLen = (ANYKA_CACHED_FRMAES + 1) * ANYKA_FRAME_SIZE * 4; /*max 4 mic */


    echo_in.m_SampleRate = samplerate;
    echo_in.m_hwSampleRate = samplerate;

    // 2.  open far path
    if (arg->is_bt_voice)
    {
        echo_in.m_pathId = 1;
        echo_in.m_mode = ECHO_MODE_AO_NORMAL;
        echo_in.cb_notify_param = (T_HANDLE)&p_far;
        p_far = _SD_Echo_Open(&echo_in);
        if (!p_far)
        {
            ret = -1;
            goto fail_exit;
        }

        g_factory_far = SD_ParamFactory_Create_ByCmdLine(arg->const_far, strlen(arg->const_far) + 1);

        if (!g_factory_far)
        {
            ret = -2;
            goto fail_exit;
        }
        _SD_Echo_SetFarPathParam(p_far, g_factory_far);
    }

    // 3. open near path
    echo_in.m_pathId = 0;
    echo_in.m_syncedInputs = 0;
    echo_in.m_syncBufLen = 12000;
    echo_in.m_syncCompensate = DELAY_SAMPLE;
    if (arg->all_mic_channels > 1)
        echo_in.m_mode = ECHO_MODE_AI_MIC_ARRAY_PROCESS;
    else
        echo_in.m_mode = ECHO_MODE_AI_NORMAL;

    echo_in.cb_notify_param = (T_HANDLE)&p_near;
    p_near = _SD_Echo_Open(&echo_in);
    if (!p_near)
    {
        ret = -3;
        goto fail_exit;
    }

    _SD_EQ_login(AK_NULL);
    _SD_NR_login(AK_NULL);
    _SD_AEC_login(AK_NULL);
    _SD_ASLC_login(AK_NULL);
    if (arg->all_mic_channels > 1)
    {
        _SD_Denc_login(AK_NULL);
    }

    g_factory_near = SD_ParamFactory_Create_ByCmdLine(arg->const_near, strlen(arg->const_near) + 1);
    if (!g_factory_near)
    {
        ret = -4;
        goto fail_exit;
    }
    result = _SD_Echo_SetNearPathParam(p_near, g_factory_near);
    if (result != AK_TRUE)
    {
        ret = -5;
        goto fail_exit;
    }

#if 0
    {
        char *strVal;
        strVal = SD_ParamFactory_GetValue(g_factory_near, "ai_gain");
        if (strVal)
        {
            acpu_printf("ai_gain=%d\n", atoi(strVal));
        }
    }
#endif
    struct echo_param_aec aec_param;
    memset(&aec_param, 0, sizeof(aec_param));

    _SD_Echo_GetAecParam(p_near, &aec_param);

    return 0;

fail_exit:
    acpu_audio_3a_close();
    return ret;
}

int acpu_audio_3a_close()
{
    if (p_far)
    {
        _SD_Echo_Reset(p_far);
        _SD_Echo_Close(p_far);
        p_far = NULL;
    }

    if (p_near)
    {
        _SD_Echo_Reset(p_near);
        _SD_Echo_Close(p_near);
        p_near = NULL;
    }
    if (g_factory_far)
    {
        SD_ParamFactory_Destroy(g_factory_far);
        g_factory_far = NULL;
    }
    if (g_factory_near)
    {
        SD_ParamFactory_Destroy(g_factory_near);
        g_factory_near = NULL;
    }
    return 0;
}

int acpu_audio_3a_downlink(acpu_audio_3a_downlink_parameter_t *arg)
{
    int32_t getsize;

    getsize = _SD_Echo_FillFarStream(p_far, arg->data_in, ANYKA_FRAME_SIZE, arg->tick * 1000, 1);

    if (!getsize)
    {
        //acpu_printf("dac empty\n");
        return 0;
    }
    getsize = _SD_Echo_GetDacStream(p_far, arg->data_out, ANYKA_FRAME_SIZE, &ts_dac_stream, 1);

    if (!getsize)
    {
        //acpu_printf("dac empty\n");
        return 0;
    }
    return getsize;
}

int acpu_audio_3a_uplink(acpu_audio_3a_uplink_parameter_t *arg)
{
    int ret;
    T_AUDIO_FILTER_TS ts_result;
    uint64_t ts = arg->ts;

    ret = _SD_Echo_FillDacLoopback(p_near, arg->refframe, ANYKA_FRAME_SIZE, ts, 1);

    //acpu_printf("fill dac loopback=%d", ret);

    ts += DELAY_SAMPLE * 1000000ULL / samplerate;
    ret = _SD_Echo_FillAdcStream(p_near, arg->fifo, ANYKA_FRAME_SIZE, ts, 1);

    //acpu_printf("fill adc=%d", ret);
    ret = _SD_Echo_GetResult(p_near, arg->result, ANYKA_FRAME_SIZE, &ts_result, 1);
    //acpu_printf("fill adc=%d", ret);

    return 0;
}

