/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if 0
#include <rtthread.h>
#ifdef SOC_BF0_HCPU
#include <string.h>
#include <stdlib.h>
#include "audioproc.h"
#include "ipc/ringbuffer.h"
#include "bf0_mbox_common.h"
#include "ipc/dataqueue.h"
#include "drivers/audio.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif

#include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"
#include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
#include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"
#include "webrtc/modules/audio_processing/dc_correction/dc_correction.h"
#include "webrtc/modules/audio_processing/ramp_in/ramp_in.h"
#include "webrtc/modules/audio_processing/ramp_out/ramp_out.h"

#ifdef SOLUTION
    #include "bt_connect.h"
#endif

#include "audio_mem.h"
#define DBG_TAG           "audio_3a"
#define DBG_LVL           AUDIO_DBG_LVL //LOG_LVL_WARNING
#include "log.h"
#include "audio_server.h"

#define CODEC_DATA_UNIT_LEN        (320)  //should same as audio_server.c
#define MIC_DELAY_REF_16K          378
#define MIC_DELAY_REF_8K           384

#define downlink_ans               0
enum AEC_MODE_TAG
{
    kQuietEarpieceOrHeadset = 0,
    kEarpiece,
    kLoudEarpiece,
    kSpeakerphone,
    kLoudSpeakerphone
};

typedef struct audio_3a_tag
{
    uint8_t     state;
    uint8_t     is_far_putted;
    uint8_t     is_far_using;
    uint16_t    frame_len; //byte
    uint16_t    samplerate;
    struct rt_ringbuffer *rbuf_out;
    uint8_t     *mic_far;
    struct rt_ringbuffer *rbuf_dwlink;
    NsxHandle   *pNS_inst;
    void        *aecmInst;
    void        *agcInst;
    void        *dwlink_agcInst;
    NsxHandle   *dwlink_ans;
    void        *dcInst;
    void        *rampInInst;
    void        *rampOutInst;
    uint8_t     *uplink_agc_out;
    uint8_t     *ans_out;
    uint8_t     *aecm_out;
} audio_3a_t;

static audio_3a_t g_audio_3a_env =
{
    .state  = 0,
    .frame_len = 0,
    .samplerate = 16000,
    .rbuf_out = NULL,
    .mic_far = NULL,
    .pNS_inst = NULL,
    .aecmInst = NULL,
    .agcInst  = NULL,
    .dwlink_agcInst = NULL,
    .dwlink_ans = NULL,
    .dcInst = NULL,
    .rampInInst = NULL,
    .rampOutInst = NULL,
};

//factory bypass using
static uint8_t g_bypass;
static uint8_t g_mic_choose;
static uint8_t g_down_choose;

static uint16_t g_mic_delay_ref = MIC_DELAY_REF_16K;

static int16_t g_cngMode = AecmFalse;
static int16_t g_echoMode = kSpeakerphone;
int16_t g_ul_agc_target_level_dbfs = 3;
int16_t g_ul_agc_compression_gain_db = 18;
uint8_t g_ul_agc_thrhold = 14;

int16_t g_dl_agc_target_level_dbfs = 6;
int16_t g_dl_agc_compression_gain_db = 10;
uint8_t g_dl_agc_thrhold = 14;
uint16_t g_aec_delay = 0;
#define DC_CORRECTION_BYPASS 0
#define RAMP_IN_BYPASS 0
#define RAMP_OUT_BYPASS 0
extern int g_rampin_interval;
extern int g_rampin_mute;
extern int g_rampout_mute;
extern int g_rampout_interval;
extern int g_rampout_gain_min;
extern uint16_t g_aecm_real_shift;
extern uint16_t g_aecm_imag_shift;
extern int16_t g_agc_decay;
extern uint32_t g_agc_min;


void audio_ramp_init(audio_3a_t *thiz)
{
#if (g_dc_enabled)
    thiz->dcInst = DcCorrection_Create();
    DcCorrection_Init(thiz->dcInst, DC_CORRECTION_BYPASS);
    RT_ASSERT(thiz->dcInst);
#endif
#if (g_ramin_enabled)
    thiz->rampInInst = RampIn_Create();
    RampIn_Init(thiz->rampInInst, RAMP_IN_BYPASS);
    RT_ASSERT(thiz->rampInInst);
#endif
    thiz->rampOutInst = RampOut_Create();
    RampOut_Init(thiz->rampOutInst, RAMP_OUT_BYPASS);

    RT_ASSERT(thiz->rampOutInst);
    LOG_I("rampin mute:%d,rampin inv:%d, rampout mute:%d, rampout interval=%d", g_rampin_mute, g_rampin_interval, g_rampout_mute, g_rampout_interval);
}


#ifdef FFT_USING_ONCHIP
//FFT_HandleTypeDef g_fft_handle;
#include "webrtc/common_audio/signal_processing/include/real_fft.h"
fft_env_t g_fft_env;
void FFT_Callback(struct __FFT_HandleTypeDef *fft)
{
    rt_event_send(g_fft_env.int_ev, 1);
}
void FFT1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_FFT_IRQHandler(&(g_fft_env.fft_handle));
    rt_interrupt_leave();

}
#endif

#ifdef AUDIO_3A_STATIC_TIME
uint32_t audio_get_curr_tick(void)
{
    return HAL_HPAON_READ_GTIMER();
}

uint32_t audio_get_delta_tick_in_10us(uint32_t last_tick)
{
    uint32_t curr_tick = HAL_HPAON_READ_GTIMER();
    return (curr_tick - last_tick) * (uint64_t)1000000 / HAL_LPTIM_GetFreq();
}
static uint32_t g_3a_test_cur;
typedef struct
{
    uint32_t times;
    uint32_t delt_sum;
    uint32_t delt;
} delta_tick_t;
delta_tick_t  g_delta[8];//ans_fix  agc   aecm  encode  decode  plc_good   plc_bad

static uint32_t g_get_curtick[AUDIO_TIME_MAX];
uint32_t g_audio_time[AUDIO_TIME_MAX];
uint32_t g_audio_time_max[AUDIO_TIME_MAX];
uint32_t g_audio_time_ave[AUDIO_TIME_MAX];
uint32_t g_audio_time_min[AUDIO_TIME_MAX];
uint32_t g_systick_cnt;
uint32_t g_uplink_max;
uint32_t g_dwlink_max;
void audio_tick_in(uint8_t type)
{
    g_get_curtick[type] = HAL_HPAON_READ_GTIMER();
    if (type == AUDIO_UPAGC_TIME)
    {
        g_systick_cnt++;
    }
}
void audio_tick_out(uint8_t type)
{
    uint32_t delta_tick;
    g_audio_time[type] = audio_get_delta_tick_in_10us(g_get_curtick[type]);
    g_get_curtick[type] = 0;
    if (type == AUDIO_UPLINK_TIME)
    {
        if (g_audio_time[type] > g_uplink_max)
        {
            g_uplink_max = g_audio_time[type] ;
        }
    }
    else if (type == AUDIO_DNLINK_TIME)
    {
        if (g_audio_time[type] > g_dwlink_max)
        {
            g_dwlink_max = g_audio_time[type] ;
        }
    }
}
uint8_t  g_audio_cnt = 0;
void audio_uplink_time_print(void)
{
    rt_kprintf("uplink_time: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", g_audio_time[0], g_audio_time[1], g_audio_time[2], g_audio_time[3], g_audio_time[5], g_audio_time[6], g_audio_time[7], g_audio_time[8], g_uplink_max);
}
void audio_dnlink_time_print(void)
{
    rt_kprintf("dnlink_time: %d, %d, %d, %d\n",  g_audio_time[9], g_audio_time[10], g_audio_time[11], g_dwlink_max);
}

#else
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
#endif

typedef enum  ANS_LEVEL_TAG
{
    ANS_LOW_LEVEL,
    ANS_MODERATE_LEVEL,
    ANS_HIGH_LEVEL,
    ANS_VERY_HIGH_LEVEL,
} ANS_LEVEL;

void audio_3a_set_bypass(uint8_t is_bypass, uint8_t mic, uint8_t down)
{
    g_bypass = is_bypass;
    if (is_bypass && mic < 3)
    {
        g_mic_choose = mic;
    }
    g_down_choose = down;
}

uint8_t audio_ans_init(audio_3a_t *thiz, uint32_t samplerate)
{
    uint8_t nMode = ANS_HIGH_LEVEL;

#ifdef AUDIO_3A_STATIC_TIME
    g_delta[0].delt_sum = 0;
    g_delta[0].times = 0;
#endif
    thiz->pNS_inst = WebRtcNsx_Create();
    RT_ASSERT(thiz->pNS_inst);

    if (thiz->pNS_inst != NULL)
    {
        if (0 != WebRtcNsx_Init(thiz->pNS_inst, samplerate))
        {
            LOG_W("fix WebRtcNs_Init error,rate:%d !\n", samplerate);
            return 2;
        }

        if (0 != WebRtcNsx_set_policy(thiz->pNS_inst, nMode))
        {
            LOG_W("fix WebRtcNs_set_policy error !\n");
            return 3;
        }
        LOG_I("fix WebRtc ANS Init finish !\n");
    }
    else
    {
        return 1;
    }
#if downlink_ans
    thiz->dwlink_ans = WebRtcNsx_Create();
    RT_ASSERT(thiz->dwlink_ans);
    if (thiz->dwlink_ans != NULL)
    {
        if (0 != WebRtcNsx_Init(thiz->dwlink_ans, samplerate))
        {
            LOG_W("downlink ans error,rate:%d", samplerate);
            return 2;
        }

        if (0 != WebRtcNsx_set_policy(thiz->dwlink_ans, nMode))
        {
            LOG_W("downlink ans policy error");
            return 3;
        }
        LOG_I("downlink ANS Init finish");
    }
    else
    {
        return -1;
    }
#endif
    return 0;
}

void audio_ans_proc(NsxHandle *p_ns_inst, int16_t spframe[160], int16_t outframe[160])
{
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
#ifdef AUDIO_3A_STATIC_TIME
    g_3a_test_cur = audio_get_curr_tick();
#endif
    WebRtcNsx_Process(p_ns_inst, (const int16_t *const *)spframe_p, 1, outframe_p);

#ifdef AUDIO_3A_STATIC_TIME
    g_delta[0].delt_sum += audio_get_delta_tick_in_10us(g_3a_test_cur);
    g_delta[0].times++;
#endif

}

uint8_t audio_aec_init(audio_3a_t *thiz, uint32_t samplerate)
{
#ifdef AUDIO_3A_STATIC_TIME
    g_delta[2].delt_sum = 0;
    g_delta[2].times = 0;
#endif

    thiz->aecmInst = WebRtcAecm_Create();

    if (thiz->aecmInst != NULL)
    {
        AecmConfig config;
        config.cngMode = g_cngMode;   //AecmTrue
        config.echoMode = g_echoMode; //kSpeakerphone;

        if (0 != WebRtcAecm_Init(thiz->aecmInst, samplerate))
        {
            LOG_W("WebRtcAecm_Init para error !\n");
            return 1;
        }

        if (0 != WebRtcAecm_set_config(thiz->aecmInst, config))
        {
            LOG_W("WebRtcAecm_set_config para error !\n");
            return 2;
        }
        LOG_I("WebRtc Aecm Init finish !\n");
    }
    else
    {
        return 3;
    }
    return 0;
}

typedef struct aec_input_para_tag
{
    int16_t *nearframe;
    int16_t *nearframe_clean;
    int16_t *outframe;
} aec_input_para_t;


int16_t audio_aec_proc(void *aecmInst, aec_input_para_t *pt_input_para, uint16_t samplerate)
{
    uint16_t u16_frame_len = 80;
    //uint16_t u16_delay;
#ifdef AUDIO_3A_STATIC_TIME
    g_3a_test_cur = audio_get_curr_tick();
#endif

    if (16000 == samplerate)
    {
        u16_frame_len = 160;
    }

    if (0 != WebRtcAecm_Process(aecmInst, pt_input_para->nearframe, pt_input_para->nearframe_clean, pt_input_para->outframe, u16_frame_len, g_aec_delay))
    {
        LOG_W("WebRtcAecm_Process  error !\n");
        return 2;
    }
#ifdef AUDIO_3A_STATIC_TIME
    g_delta[2].delt_sum += audio_get_delta_tick_in_10us(g_3a_test_cur);
    g_delta[2].times++;
#endif

    return 0;
}

int16_t audio_agc_init(audio_3a_t *thiz, uint16_t samplerate)
{
    int32_t minLevel = 0;
    int32_t maxLevel = 255;
    uint8_t ulagcMode = kAgcModeFixedDigital; //kAgcModeFixedDigital;// 0, 1, 2, 3     unsupport adapitve mode
    uint8_t dlagcMode = kAgcModeFixedDigital; //kAgcModeFixedDigital;// 0, 1, 2, 3     unsupport adapitve mode

    WebRtcAgcConfig agcConfig;
    agcConfig.compressionGaindB = g_ul_agc_compression_gain_db;
    agcConfig.limiterEnable = 1;
    agcConfig.targetLevelDbfs = g_ul_agc_target_level_dbfs;
    agcConfig.thrhold = g_ul_agc_thrhold;

#ifdef AUDIO_3A_STATIC_TIME
    g_delta[1].delt_sum = 0;
    g_delta[1].times = 0;
#endif

    thiz->agcInst = WebRtcAgc_Create();
    if (thiz->agcInst != NULL)
    {
        if (0 != WebRtcAgc_Init(thiz->agcInst, minLevel, maxLevel, ulagcMode, samplerate))
        {
            LOG_W("WebRtcAgc_Init para error !\n");
            return 1;
        }

        if (0 != WebRtcAgc_set_config(thiz->agcInst, agcConfig))
        {
            LOG_W("WebRtcAgc_set_config para error !\n");
            return 2;
        }
        LOG_I("WebRtc Agc Init finish !\n");
    }
    else
    {
        return 3;
    }

    agcConfig.compressionGaindB = g_dl_agc_compression_gain_db;
    agcConfig.limiterEnable = 1;
    agcConfig.targetLevelDbfs = g_dl_agc_target_level_dbfs;
    agcConfig.thrhold = g_dl_agc_thrhold;
    thiz->dwlink_agcInst = WebRtcAgc_Create();
    if (thiz->dwlink_agcInst != NULL)
    {
        if (0 != WebRtcAgc_Init(thiz->dwlink_agcInst, minLevel, maxLevel, dlagcMode, samplerate))
        {
            LOG_W("downlink WebRtcAgc_Init para error !\n");
            return 1;
        }

        if (0 != WebRtcAgc_set_config(thiz->dwlink_agcInst, agcConfig))
        {
            LOG_W("dowmlink WebRtcAgc_set_config para error !\n");
            return 2;
        }
        LOG_I("downlink WebRtc Agc Init finish !\n");
    }
    else
    {
        return 3;
    }

    return 0;
}

void audio_agc_proc(void *agcInst, int16_t spframe[160], int16_t outframe[160], uint16_t samplerate)
{
    int32_t micLevelIn = 0;
    int32_t micLevelOut = 0;
    uint8_t saturationWarning;
    uint16_t u16_frame_len = 80;
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};

#ifdef AUDIO_3A_STATIC_TIME
    g_3a_test_cur = audio_get_curr_tick();
#endif

    if (16000 == samplerate)
    {
        u16_frame_len = 160;
    }

    if (0 != WebRtcAgc_Process(agcInst, (const int16_t *const *)spframe_p, 1, u16_frame_len, (int16_t *const *)outframe_p, micLevelIn, &micLevelOut, 0, &saturationWarning))
    {
        LOG_W("WebRtcAgc_Process error !\n");
    }
#ifdef AUDIO_3A_STATIC_TIME
    g_delta[1].delt_sum += audio_get_delta_tick_in_10us(g_3a_test_cur);
    g_delta[1].times++;
#endif
}


void audio_3a_module_init(audio_3a_t *thiz, uint32_t samplerate)
{
    uint8_t ret = 0;
    if (samplerate == 16000)
    {
        g_mic_delay_ref = MIC_DELAY_REF_16K;
    }
    else
    {
        g_mic_delay_ref = MIC_DELAY_REF_8K;
    }

    audio_ramp_init(thiz);

    ret = audio_ans_init(thiz, samplerate);
    if (ret != 0)
    {
        RT_ASSERT(0);
    }

    ret = audio_aec_init(thiz, samplerate);
    if (ret != 0)
    {
        RT_ASSERT(0);
    }

    ret = audio_agc_init(thiz, samplerate);
    if (ret != 0)
    {
        RT_ASSERT(0);
    }

    thiz->rbuf_out = rt_ringbuffer_create(CODEC_DATA_UNIT_LEN * 2);
    thiz->mic_far = calloc(CODEC_DATA_UNIT_LEN + g_mic_delay_ref * 2, 1);
    thiz->rbuf_dwlink = rt_ringbuffer_create(CODEC_DATA_UNIT_LEN * 4);
    RT_ASSERT(thiz->rbuf_dwlink);
#ifdef FFT_USING_ONCHIP
    g_fft_env.fft_handle.Instance = hwp_fft1;
    g_fft_env.fft_handle.CpltCallback = FFT_Callback;
    g_fft_env.int_ev = rt_event_create("fft_int_ev", RT_IPC_FLAG_FIFO);
    HAL_RCC_EnableModule(RCC_MOD_FFT1);
    HAL_FFT_Init(&(g_fft_env.fft_handle));
    HAL_NVIC_SetPriority(FFT1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(FFT1_IRQn);
#endif

}

#define SWAP_IN_OUT()  do {temp = in; in = out; out = temp;} while(0)

void audio_3a_data_process(audio_3a_t *thiz, uint8_t *input, uint8_t index_8k)
{
    int16_t frame0[160];
    int16_t frame1[160];
    uint8_t result[320];

    int16_t  *in;
    int16_t  *out;
    int16_t  *temp;

    in = frame0;
    out = frame1;
    memcpy(in, input, thiz->frame_len);
    if (g_bypass)
    {
        goto bypass_3a;
    }
    //1. dc
#if (g_dc_enabled)
    DcCorrection_Process(thiz->dcInst, in, out, thiz->frame_len / 2);
    SWAP_IN_OUT();
#endif

    //2. ans
    audio_ans_proc(thiz->pNS_inst, in, out);
    SWAP_IN_OUT();

    //dump ans out
    if (is_audio_dump_enable_type(ADUMP_ANS_OUT))
    {
        if (8000 == thiz->samplerate) //cvsd
        {
            memcpy(thiz->ans_out + 160 * index_8k, in, 160);
            if (index_8k)
            {
                audio_dump_data(ADUMP_ANS_OUT, thiz->ans_out, 320);
            }
        }
        else
        {
            audio_dump_data(ADUMP_ANS_OUT, (uint8_t *)in, 320);
        }
    }

    //3. aecm

    aec_input_para_t input_para;
    input_para.nearframe = in;
    input_para.nearframe_clean = NULL;
    input_para.outframe = out;

    audio_aec_proc(thiz->aecmInst, &input_para, thiz->samplerate);
    SWAP_IN_OUT();

    //dump aecm out
    if (is_audio_dump_enable_type(ADUMP_AECM_OUT))
    {
        if (8000 == thiz->samplerate)
        {
            memcpy(thiz->aecm_out + 160 * index_8k, in, 160);
            if (index_8k)
            {
                audio_dump_data(ADUMP_AECM_OUT, thiz->aecm_out, 320);
            }
        }
        else
        {
            audio_dump_data(ADUMP_AECM_OUT, (uint8_t *)in, 320);
        }
    }

    //4. agc
    audio_agc_proc(thiz->agcInst, in, out, thiz->samplerate);
    SWAP_IN_OUT();

    if (is_audio_dump_enable_type(ADUMP_AGC_OUT))
    {
        if (8000 == thiz->samplerate)
        {
            memcpy(thiz->uplink_agc_out + 160 * index_8k, in, 160);
            if (index_8k)
            {
                audio_dump_data(ADUMP_AGC_OUT, thiz->uplink_agc_out, 320);
            }
        }
        else
        {
            audio_dump_data(ADUMP_AGC_OUT, (uint8_t *)in, 320);
        }
    }


    //5. ramp out
    int ret = RampOut_Process(thiz->rampOutInst, in, out, thiz->frame_len / 2);
    if (!ret)
    {
        SWAP_IN_OUT();
    }

    if (is_audio_dump_enable_type(ADUMP_RAMP_OUT_OUT))
    {
        if (8000 == thiz->samplerate)
        {
            memcpy(&result[160 * index_8k], in, 160);
            if (index_8k)
            {
                audio_dump_data(ADUMP_RAMP_OUT_OUT, result, 320);
            }
        }
        else
        {
            audio_dump_data(ADUMP_RAMP_OUT_OUT, (uint8_t *)in, 320);
        }
    }

bypass_3a:
    if (rt_ringbuffer_space_len(thiz->rbuf_out) >= thiz->frame_len)
    {
        rt_ringbuffer_put(thiz->rbuf_out, (uint8_t *)in, thiz->frame_len);
    }
    else
    {
        LOG_I("3a_w rbuf_out full\n");
    }
}

void audio_3a_module_free(audio_3a_t *thiz)
{
    RT_ASSERT(thiz);
    RT_ASSERT(thiz->pNS_inst);
    WebRtcNsx_Free(thiz->pNS_inst);
    thiz->pNS_inst = NULL;
#if downlink_ans
    RT_ASSERT(thiz->dwlink_ans);
    WebRtcNsx_Free(thiz->dwlink_ans);
    thiz->dwlink_ans = NULL;
#endif
    LOG_I("3a_w WebRtcNs_Free\n");
    WebRtcAecm_Free(thiz->aecmInst);
    LOG_I("3a_w WebRtcAecm_Free\n");
    thiz->aecmInst = NULL;
    WebRtcAgc_Free(thiz->agcInst);
    LOG_I("3a_w WebRtcAgc_Free\n");
    thiz->agcInst = NULL;

    WebRtcAgc_Free(thiz->dwlink_agcInst);
    LOG_I("3a_w WebRtcAgc_Free\n");
    thiz->dwlink_agcInst = NULL;


#if g_dc_enabled
    DcCorrection_Free(thiz->dcInst);
    thiz->dcInst = NULL;
#endif
#if (g_ramin_enabled)
    RampIn_Free(thiz->rampInInst);
    thiz->rampInInst = NULL;
#endif
    RampOut_Free(thiz->rampOutInst);
    thiz->rampOutInst = NULL;

    rt_ringbuffer_destroy(thiz->rbuf_out);
    free(thiz->mic_far);
    rt_ringbuffer_destroy(thiz->rbuf_dwlink);
#ifdef FFT_USING_ONCHIP
    HAL_FFT_DeInit(&(g_fft_env.fft_handle));
    rt_event_delete(g_fft_env.int_ev);
    HAL_RCC_DisableModule(RCC_MOD_FFT1);
#endif
    if (thiz->uplink_agc_out)
    {
        audio_mem_free(thiz->uplink_agc_out);
        thiz->uplink_agc_out = NULL;
    }
    if (thiz->ans_out)
    {
        audio_mem_free(thiz->ans_out);
        thiz->ans_out = NULL;
    }
    if (thiz->aecm_out)
    {
        audio_mem_free(thiz->aecm_out);
        thiz->aecm_out = NULL;
    }

}

void audio_3a_open(uint32_t samplerate)
{
    audio_3a_t *thiz = &g_audio_3a_env;
#ifdef SOLUTION
    bool talk_with_abox = false;

    if (BT_DEV_CLS_AUDIO_BOX ==  bt_connect_get_peer_type())
    {
        g_ul_agc_compression_gain_db = 0;
        g_dl_agc_compression_gain_db = 12;
        g_dl_agc_target_level_dbfs = 12;
        g_agc_decay = 1000;
        g_echoMode = kLoudSpeakerphone;

        talk_with_abox = true;
    }
    else
    {
        g_ul_agc_compression_gain_db = 18;
        g_dl_agc_compression_gain_db = 10;
        g_dl_agc_target_level_dbfs = 6;
        g_agc_decay = 100;
        g_echoMode = kSpeakerphone;
    }
    LOG_I("talk_with_abox=%d", talk_with_abox);
#endif

    LOG_I("g_ul_agc_compression_gain_db=%d, g_dl_agc_compression_gain_db=%d, g_agc_decay=%d, g_echoMode=%d",
          g_ul_agc_compression_gain_db, g_dl_agc_compression_gain_db, g_agc_decay, g_echoMode);
    if (thiz->state == 0)
    {
        LOG_I("3a_w open samplearate=%ld", samplerate);
        if (samplerate == 8000)
        {
            thiz->uplink_agc_out = audio_mem_malloc(CODEC_DATA_UNIT_LEN);
            thiz->ans_out = audio_mem_malloc(CODEC_DATA_UNIT_LEN);
            thiz->aecm_out = audio_mem_malloc(CODEC_DATA_UNIT_LEN);
            thiz->frame_len = 160;
            thiz->samplerate = 8000;
            audio_3a_module_init(thiz, 8000);
        }
        else
        {
            thiz->frame_len = 320;
            thiz->samplerate = 16000;
            audio_3a_module_init(thiz, 16000);
        }
        thiz->state = 1;
        thiz->is_far_putted = 0;
        thiz->is_far_using = 0;
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
        thiz->state = 0;
#ifdef AUDIO_BT_AUDIO
        bt_voice_close();
#endif
    }
}

void audio_3a_far_put(uint8_t *fifo, uint16_t fifo_size)
{
    audio_3a_t *thiz = &g_audio_3a_env;
    if (thiz->state == 0)
    {
        LOG_I("3a far put: closed");
        return;
    }
#if DEBUG_FRAME_SYNC
    RT_ASSERT(thiz->is_far_using == 0);
#endif
    memcpy(thiz->mic_far + g_mic_delay_ref * 2, fifo, fifo_size);
#if DEBUG_FRAME_SYNC
    RT_ASSERT(thiz->is_far_using == 0);
#endif
    thiz->is_far_putted = 1;
}

uint8_t audio_3a_dnlink_buf_is_full(uint8_t size)
{
    audio_3a_t *thiz = &g_audio_3a_env;

    if (rt_ringbuffer_space_len(thiz->rbuf_dwlink) <= size)
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
    int16_t  data1[160];
    int16_t  data2[160];

    int16_t  *in;
    int16_t  *out;
    int16_t  *temp;
    in = data1;
    out = data2;
    if (rt_ringbuffer_space_len(thiz->rbuf_dwlink) >= size)
    {
        putsize = rt_ringbuffer_put(thiz->rbuf_dwlink, fifo, size);
        RT_ASSERT(putsize == size);
    }
    else
    {
        LOG_I("3a_w rbuf_dwlink full");
    }

    while (rt_ringbuffer_data_len(thiz->rbuf_dwlink) >= thiz->frame_len)
    {
        getsize = rt_ringbuffer_get(thiz->rbuf_dwlink, (uint8_t *)in, thiz->frame_len);
        RT_ASSERT(getsize == thiz->frame_len);
        audio_dump_data(ADUMP_DOWNLINK, (uint8_t *)in, thiz->frame_len);
#if downlink_ans
        audio_ans_proc(thiz->dwlink_ans, in, out);
        SWAP_IN_OUT();
#endif
        audio_agc_proc(thiz->dwlink_agcInst, in, out, thiz->samplerate);

        audio_dump_data(ADUMP_DOWNLINK_AGC, (uint8_t *)out, thiz->frame_len);

        speaker_ring_put((uint8_t *)out, thiz->frame_len);
    }
}

void audio_3a_uplink(uint8_t *fifo, uint16_t fifo_size, uint8_t is_mute, uint8_t is_pdm)
{
    int16_t far[160];
    audio_3a_t *thiz = &g_audio_3a_env;
    uint16_t putsize, getsize;
    UNUSED(is_pdm);
    RT_ASSERT(fifo_size == 320);
    if (!thiz->is_far_putted)
    {
        LOG_I("wait far put");
    }

    audio_dump_data(ADUMP_AUDPRC, fifo, fifo_size);

#if DEBUG_FRAME_SYNC
    extern void save_mic_tick();
    save_mic_tick();
#endif
    thiz->is_far_using = 1;
    int32_t writted = WebRtcAecm_BufferFarend(thiz->aecmInst, (const int16_t *)thiz->mic_far, 160);
    memcpy(far, thiz->mic_far, CODEC_DATA_UNIT_LEN);
    memcpy(thiz->mic_far, thiz->mic_far + CODEC_DATA_UNIT_LEN, g_mic_delay_ref * 2);
    thiz->is_far_using = 0;

    if (writted != 160)
    {
        LOG_W("far put %d!\n", writted);
    }

    audio_dump_data(ADUMP_AECM_INPUT1, (uint8_t *)far, fifo_size);
    audio_dump_data(ADUMP_AECM_INPUT2, fifo, fifo_size);

    if (8000 == thiz->samplerate)
    {
        audio_3a_data_process(thiz, fifo, 0);
        audio_3a_data_process(thiz, fifo + 160, 1);
        while (rt_ringbuffer_data_len(thiz->rbuf_out) >= 120)
        {
            rt_ringbuffer_get(thiz->rbuf_out, (uint8_t *)&far[0], 120);
            if (is_mute)
            {
                memset(far, 0, 120);
            }
#ifdef AUDIO_BT_AUDIO
            msbc_encode_process((uint8_t *)&far[0], 120);
#endif
        }
    }
    else
    {
        audio_3a_data_process(thiz, fifo, 0);
        while (rt_ringbuffer_data_len(thiz->rbuf_out) >= 240)
        {
            rt_ringbuffer_get(thiz->rbuf_out, (uint8_t *)&far[0], 240);
            if (is_mute)
            {
                memset((uint8_t *)&far[0], 0, 240);
            }
#ifdef AUDIO_BT_AUDIO
            msbc_encode_process((uint8_t *)&far[0], 240);
#endif
        }

    }
}

#ifdef AUDIO_3A_STATIC_TIME
int read_3a_stat(int argc, char *argv[])
{
    rt_thread_t thread;

    if (argc != 1)
    {
        rt_kprintf("no para \n");
        return -1;
    }

    for (int i = 0; i < 8; i++)
    {
        g_delta[i].delt  = g_delta[i].delt_sum / g_delta[i].times;
        rt_kprintf("index:%d, times=%d, sum=%d, div=%d\n", i, g_delta[i].times, g_delta[i].delt_sum, g_delta[i].delt);
    }
    return 0;
}

MSH_CMD_EXPORT(read_3a_stat,    statistic running time);
#endif


/*usded by android test app*/
void webrtc_set_delay(uint16_t delay)
{
    g_aec_delay = delay;
    LOG_I("---u16_delay=%d", delay);
}
uint32_t g_record_time = 10;
void audio_command_process(uint8_t *cmd_1)
{
#if 0
    const char *cmd = (const char *)cmd_1;
    LOG_I("--dump cmd=[%s]", cmd);
    if (!memcmp(cmd, "cng=0", 5))
    {
        g_cngMode = AecmFalse;
    }
    else if (!memcmp(cmd, "cng=1", 5))
    {
        g_cngMode = AecmTrue;
    }
    else if (!memcmp(cmd, "echo=3", 6))
    {
        g_echoMode = kSpeakerphone;
    }
    else if (!memcmp(cmd, "echo=4", 6))
    {
        g_echoMode = kLoudSpeakerphone;
    }
    else if (!memcmp(cmd, "rshift=", 7))
    {
        g_aecm_real_shift = atoi(cmd + 7);
        LOG_I("set aec real shift=%d", g_aecm_real_shift);
    }
    else if (!memcmp(cmd, "ishift=", 7))
    {
        g_aecm_imag_shift = atoi(cmd + 7);
        LOG_I("set aec imag shift=%d", g_aecm_imag_shift);
    }
    else if (!memcmp(cmd, "time=", 5))
    {
        g_record_time = atoi(cmd + 5);
        LOG_I("---dump timeout=%d", g_record_time);
    }
    else if (!memcmp(cmd, "rampin_mute=", 12))
    {
        g_rampin_mute = atoi(cmd + 12);
        LOG_I("set rampin mute time=%d", g_rampin_mute);
    }
    else if (!memcmp(cmd, "rampin_inv=", 11))
    {
        g_rampin_interval = atoi(cmd + 11);
        LOG_I("set rampin interval =%d", g_rampin_interval);
    }
    else if (!memcmp(cmd, "rampout_mute=", 13))
    {
        g_rampout_mute = atoi(cmd + 13);
        LOG_I("set rampout mute time=%d", g_rampout_mute);
    }
    else if (!memcmp(cmd, "rampout_inv=", 12))
    {
        g_rampout_interval = atoi(cmd + 12);
        LOG_I("set rampout interval=%d", g_rampout_interval);
    }
    else if (!memcmp(cmd, "rampout_gainmin=", 16))
    {
        g_rampout_gain_min = atoi(cmd + 16);
        LOG_I("set rampout gain min=%d", g_rampout_gain_min);
    }
    else if (!memcmp(cmd, "dagcgain=", 9))
    {
        g_dl_agc_compression_gain_db = atoi(cmd + 9);
        LOG_I("set dl_agc_gain=%d", g_dl_agc_compression_gain_db);
    }
    else if (!memcmp(cmd, "uagcgain=", 9))
    {
        g_ul_agc_compression_gain_db = atoi(cmd + 9);
        LOG_I("set ul_agc_gain=%d", g_ul_agc_compression_gain_db);
    }
    else if (!memcmp(cmd, "dagctgt=", 8))
    {
        g_dl_agc_target_level_dbfs = atoi(cmd + 8);
        LOG_I("set dl_agc_target_level=%d", g_dl_agc_target_level_dbfs);
    }
    else if (!memcmp(cmd, "uagctgt=", 8))
    {
        g_ul_agc_target_level_dbfs = atoi(cmd + 8);
        LOG_I("set ul_agc_target_level=%d", g_ul_agc_target_level_dbfs);
    }
    else if (!memcmp(cmd, "aecdly=", 7))
    {
        g_aec_delay = atoi(cmd + 7);
        LOG_I("set aec delay=%d", g_aec_delay);
    }
    else if (!memcmp(cmd, "rshift=", 7))
    {
        g_aecm_real_shift = atoi(cmd + 7);
        LOG_I("set aec real shift=%d", g_aecm_real_shift);
    }
    else if (!memcmp(cmd, "ishift=", 7))
    {
        g_aecm_imag_shift = atoi(cmd + 7);
        LOG_I("set aec imag shift=%d", g_aecm_imag_shift);
    }
    else if (!memcmp(cmd, "uagcthr=", 8))
    {
        g_ul_agc_thrhold = atoi(cmd + 8);
        LOG_I("set uagcthr=%d", g_ul_agc_thrhold);
    }
    else if (!memcmp(cmd, "dagcthr=", 8))
    {
        g_dl_agc_thrhold = atoi(cmd + 8);
        LOG_I("set dagcthr=%d", g_dl_agc_thrhold);
    }
    else if (!memcmp(cmd, "uagcdecay=", 10))
    {
        g_agc_decay = atoi(cmd + 10);
        LOG_I("set g_agc_decay=%d", g_agc_decay);
    }
    else if (!memcmp(cmd, "uagcmin=", 8))
    {
        g_agc_min = atoi(cmd + 8);
        LOG_I("set g_agc_min=%d", g_agc_min);
    }
    else if (!memcmp(cmd, "clear", 5))
    {
    }
#endif
}

#endif

#endif