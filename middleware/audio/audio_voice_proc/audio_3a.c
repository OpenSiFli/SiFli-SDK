/**
  ******************************************************************************
  * @file   audio_3a.c
  * @author Sifli software development team
  * @brief SIFLI audio  3A algorithm module.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2020 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#ifdef WEBRTC_ANS_FIX
    #include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"
#endif
#ifdef WEBRTC_AECM
    #include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
#endif
#ifdef WEBRTC_AGC_FIX
    #include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"
#endif

#ifdef PKG_USING_WEBRTC
    #include "webrtc/modules/audio_processing/dc_correction/dc_correction.h"
    #include "webrtc/modules/audio_processing/ramp_in/ramp_in.h"
    #include "webrtc/modules/audio_processing/ramp_out/ramp_out.h"
#endif

#if defined(RT_USING_BT) && defined(SOLUTION_WATCH)
    #include "bt_connect.h"
#endif

#include "audio_mem.h"
#define DBG_TAG           "audio_3a"
#define DBG_LVL           AUDIO_DBG_LVL //LOG_LVL_WARNING
#include "log.h"
#include "audio_server.h"

#define DOWN_LINK_AGC_ENABLE  1

#define AUDIO_3A_RINGBUFFER_SIZE (320)

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
    uint8_t      state;
    volatile uint8_t is_far_putted;
    volatile uint8_t is_aecm_mic_putted;
    uint16_t     frame_len; //byte
    uint16_t     samplerate;
    struct rt_ringbuffer *rbuf_out;
    struct rt_ringbuffer *rbuf_far;
    struct rt_ringbuffer *rbuf_dwlink;
#ifdef WEBRTC_ANS_FIX
    NsxHandle *pNS_inst;
#endif
    void      *aecmInst;
    void      *agcInst;
    void      *dwlink_agcInst;
    void      *dcInst;
    void      *rampInInst;
    void      *rampOutInst;
} audio_3a_t;

static audio_3a_t g_audio_3a_env =
{
    .state  = 0,
    .frame_len = 0,
    .samplerate = 16000,
    .rbuf_out = NULL,
    .rbuf_far = NULL,
#ifdef WEBRTC_ANS_FIX
    .pNS_inst = NULL,
#endif
    .aecmInst = NULL,
    .agcInst  = NULL,
    .dwlink_agcInst = NULL,
    .dcInst = NULL,
    .rampInInst = NULL,
    .rampOutInst = NULL,
};

//factory bypass using
static uint8_t g_bypass;
static uint8_t g_mic_choose;
static uint8_t g_down_choose;

uint8_t      *g_3a_fifo;

volatile uint8_t g_uplink_agc = 1;
volatile uint16_t g_u16_test_aec = 1;
volatile uint16_t g_u16_test_agc = 1;
volatile bool g_ans1_disabled;
static int16_t g_cngMode = AecmFalse;
static int16_t g_echoMode = kSpeakerphone;
int16_t g_ul_agc_target_level_dbfs = 3;
int16_t g_ul_agc_compression_gain_db = 18;
uint8_t g_ul_agc_thrhold = 14;

int16_t g_dl_agc_target_level_dbfs = 6;
int16_t g_dl_agc_compression_gain_db = 10;
uint8_t g_dl_agc_thrhold = 14;
uint16_t g_aec_delay = 10;
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

void audio_ramp_init(audio_3a_t *p_3a_env)
{
#if (g_dc_enabled)
    p_3a_env->dcInst = DcCorrection_Create();
    DcCorrection_Init(p_3a_env->dcInst, DC_CORRECTION_BYPASS);
    RT_ASSERT(p_3a_env->dcInst);
#endif
#if (g_ramin_enabled)
    p_3a_env->rampInInst = RampIn_Create();
    RampIn_Init(p_3a_env->rampInInst, RAMP_IN_BYPASS);
    RT_ASSERT(p_3a_env->rampInInst);
#endif
    p_3a_env->rampOutInst = RampOut_Create();
    RampOut_Init(p_3a_env->rampOutInst, RAMP_OUT_BYPASS);

    RT_ASSERT(p_3a_env->rampOutInst);
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
#if 0
    if (g_audio_time[type] > g_audio_time_max[type])
    {
        g_audio_time_max[type] = g_audio_time[type];
    }
    g_audio_time_ave[type] += g_audio_time[type];
    if (g_audio_time[type] < g_audio_time_min[type])
    {
        g_audio_time_min[type] = g_audio_time[type];
    }
#endif
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
#if 0
void audio_time_print(void)
{
    //rt_hexdump("audio_time", 32, g_audio_time, AUDIO_TIME_MAX*4);
    int col = (AUDIO_TIME_MAX >> 2);

    if (g_audio_cnt < 128)
    {
        g_audio_cnt++;
        return;
    }
    g_audio_cnt = 0;
    if (AUDIO_TIME_MAX % 4)
    {
        col++;
    }
#if 0
    for (int i = 0; i < col; i++)
    {
        rt_kprintf("audio_time: %d, %d, %d, %d\n", g_audio_time[i * 4], g_audio_time[i * 4 + 1], g_audio_time[i * 4 + 2], g_audio_time[i * 4 + 3]);
    }

    for (int i = 0; i < col; i++)
    {
        rt_kprintf("max_audio_time: %d, %d, %d, %d\n", g_audio_time_max[i * 4], g_audio_time_max[i * 4 + 1], g_audio_time_max[i * 4 + 2], g_audio_time_max[i * 4 + 3]);
    }

    for (int i = 0; i < col; i++)
    {
        rt_kprintf("avg_audio_time: %d, %d, %d, %d\n", g_audio_time_ave[i * 4] >> 7, g_audio_time_ave[i * 4 + 1] >> 7, g_audio_time_ave[i * 4 + 2] >> 7, g_audio_time_ave[i * 4 + 3] >> 7);
    }
    for (int i = 0; i < col; i++)
    {
        rt_kprintf("min_audio_time: %d, %d, %d, %d\n", g_audio_time_min[i * 4], g_audio_time_min[i * 4 + 1], g_audio_time_min[i * 4 + 2], g_audio_time_min[i * 4 + 3]);
    }
#else
    rt_kprintf("ins_time: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", g_audio_time[0], g_audio_time[1], g_audio_time[2], g_audio_time[3], g_audio_time[5], g_audio_time[6], g_audio_time[7], g_audio_time[8], g_audio_time[9], g_audio_time[10], g_audio_time[11], g_audio_time[4]);
    rt_kprintf("max_time: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", g_audio_time_max[0], g_audio_time_max[1], g_audio_time_max[2], g_audio_time_max[3], g_audio_time_max[5], g_audio_time_max[6], g_audio_time_max[7], g_audio_time_max[8], g_audio_time_max[9], g_audio_time_max[10], g_audio_time_max[11], g_audio_time_max[4]);
    rt_kprintf("avg_time: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", g_audio_time_ave[0] >> 7, g_audio_time_ave[1] >> 7, g_audio_time_ave[2] >> 7, g_audio_time_ave[3] >> 7, g_audio_time_ave[5] >> 7, g_audio_time_ave[6] >> 7, g_audio_time_ave[7] >> 7, g_audio_time_ave[8] >> 7, g_audio_time_ave[9] >> 7, g_audio_time_ave[10] >> 7, g_audio_time_ave[11] >> 7, g_audio_time_ave[4] / g_systick_cnt);
    rt_kprintf("min_time: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", g_audio_time_min[0], g_audio_time_min[1], g_audio_time_min[2], g_audio_time_min[3], g_audio_time_min[5], g_audio_time_min[6], g_audio_time_min[7], g_audio_time_min[8], g_audio_time_min[9], g_audio_time_min[10], g_audio_time_min[11], g_audio_time_min[4], g_systick_cnt);

#endif
    for (int i = 0; i < AUDIO_TIME_MAX; i++)
    {
        g_audio_time_max[i] = 0;
        g_audio_time_ave[i] = 0;
        g_audio_time_min[i] = 0xFFFFFFFF;
        g_systick_cnt = 0;
    }
}
#endif
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

#ifdef WEBRTC_ANS_FIX

typedef enum  ANS_LEVEL_TAG
{
    ANS_LOW_LEVEL,
    ANS_MODERATE_LEVEL,
    ANS_HIGH_LEVEL,
    ANS_VERY_HIGH_LEVEL,
} ANS_LEVEL;

uint8_t audio_ans_init(audio_3a_t *p_3a_env, uint32_t samplerate)
{
    uint8_t nMode = ANS_HIGH_LEVEL;

#ifdef AUDIO_3A_STATIC_TIME
    g_delta[0].delt_sum = 0;
    g_delta[0].times = 0;
#endif
    p_3a_env->pNS_inst = WebRtcNsx_Create();
    RT_ASSERT(p_3a_env->pNS_inst);

    if (p_3a_env->pNS_inst != NULL)
    {
        if (0 != WebRtcNsx_Init(p_3a_env->pNS_inst, samplerate))
        {
            LOG_W("fix WebRtcNs_Init error,rate:%d !\n", samplerate);
            return 2;
        }

        if (0 != WebRtcNsx_set_policy(p_3a_env->pNS_inst, nMode))
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
#endif

#ifdef WEBRTC_AECM
uint8_t audio_aec_init(audio_3a_t *p_3a_env, uint32_t samplerate)
{
#ifdef AUDIO_3A_STATIC_TIME
    g_delta[2].delt_sum = 0;
    g_delta[2].times = 0;
#endif

    p_3a_env->aecmInst = WebRtcAecm_Create();

    if (p_3a_env->aecmInst != NULL)
    {
        AecmConfig config;
        config.cngMode = g_cngMode;   //AecmTrue
        config.echoMode = g_echoMode; //kSpeakerphone;

        if (0 != WebRtcAecm_Init(p_3a_env->aecmInst, samplerate))
        {
            LOG_W("WebRtcAecm_Init para error !\n");
            return 1;
        }

        if (0 != WebRtcAecm_set_config(p_3a_env->aecmInst, config))
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
#endif

#ifdef WEBRTC_AGC_FIX

int16_t audio_agc_init(audio_3a_t *p_3a_env, uint16_t samplerate)
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

    p_3a_env->agcInst = WebRtcAgc_Create();
    if (p_3a_env->agcInst != NULL)
    {
        if (0 != WebRtcAgc_Init(p_3a_env->agcInst, minLevel, maxLevel, ulagcMode, samplerate))
        {
            LOG_W("WebRtcAgc_Init para error !\n");
            return 1;
        }

        if (0 != WebRtcAgc_set_config(p_3a_env->agcInst, agcConfig))
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
#if DOWN_LINK_AGC_ENABLE
    agcConfig.compressionGaindB = g_dl_agc_compression_gain_db;
    agcConfig.limiterEnable = 1;
    agcConfig.targetLevelDbfs = g_dl_agc_target_level_dbfs;
    agcConfig.thrhold = g_dl_agc_thrhold;
    p_3a_env->dwlink_agcInst = WebRtcAgc_Create();
    if (p_3a_env->dwlink_agcInst != NULL)
    {
        if (0 != WebRtcAgc_Init(p_3a_env->dwlink_agcInst, minLevel, maxLevel, dlagcMode, samplerate))
        {
            LOG_W("downlink WebRtcAgc_Init para error !\n");
            return 1;
        }

        if (0 != WebRtcAgc_set_config(p_3a_env->dwlink_agcInst, agcConfig))
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
#endif
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
#endif

void audio_3a_module_init(audio_3a_t *p_3a_env, uint32_t samplerate)
{
    uint8_t ret = 0;

    audio_ramp_init(p_3a_env);

#ifdef WEBRTC_ANS_FIX
    ret = audio_ans_init(p_3a_env, samplerate);
    if (ret != 0)
    {
        RT_ASSERT(0);
    }
#endif
#ifdef WEBRTC_AECM
    ret = audio_aec_init(p_3a_env, samplerate);
    if (ret != 0)
    {
        RT_ASSERT(0);
    }
#endif
#ifdef WEBRTC_AGC_FIX
    ret = audio_agc_init(p_3a_env, samplerate);
    if (ret != 0)
    {
        RT_ASSERT(0);
    }
#endif
    g_audio_3a_env.rbuf_out = rt_ringbuffer_create(AUDIO_3A_RINGBUFFER_SIZE * 2);
    g_audio_3a_env.rbuf_far = rt_ringbuffer_create(AUDIO_3A_RINGBUFFER_SIZE * 2);
    g_audio_3a_env.rbuf_dwlink = rt_ringbuffer_create(AUDIO_3A_RINGBUFFER_SIZE * 4);
    RT_ASSERT(g_audio_3a_env.rbuf_dwlink);
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


#define  DC_CANCELLATION_ENABLE   0  // DC cancellation  used other algo
#if DC_CANCELLATION_ENABLE
void dc_cancellation_proc(int16_t spframe[160], uint16_t len)
{
    int16_t  dc_value = 0;
    int32_t  sum_value = 0;
    int16_t  i;

    for (i = 0; i < len; i++)
    {
        sum_value += spframe[i];
    }
    dc_value = sum_value / len;
    for (i = 0; i < len; i++)
    {
        spframe[i] -= dc_value;
    }
}
#endif

void audio_3a_data_process(audio_3a_t *p_3a_env, uint8_t *fifo, uint16_t fifo_size)
{
#ifdef AUDIO_MEM_ALLOC
    int16_t *spframe = audio_mem_malloc(160 * 2);
    int16_t *outframe = audio_mem_malloc(160 * 2);
    int16_t *outframe2 = audio_mem_malloc(160 * 2);
    RT_ASSERT(spframe && outframe && outframe2);
#else
    int16_t spframe[160];
    int16_t outframe[160];
    int16_t outframe2[160];
#endif
    uint16_t getsize, putsize;
    int16_t  *data_in;
    int16_t  *data_in2;
    int16_t  *data_out;
    int16_t  *temp;

    RT_ASSERT(fifo_size == p_3a_env->frame_len)

    memcpy(outframe2, fifo, fifo_size);

#if PKG_USING_AUDIO_TEST_API
    {
        extern uint8_t audio_test_api_3a_is_enable();
        g_uplink_agc = 0;
        g_ans1_disabled = 1;

        if (audio_test_api_3a_is_enable())
        {
            g_ans1_disabled = 0;
            g_uplink_agc = 1;
        }
    }
#endif

#ifdef PKG_USING_WEBRTC
    data_in = outframe2;
    data_out = spframe;
#if (g_dc_enabled)
    audio_tick_in(AUDIO_DC_TIME);
    DcCorrection_Process(p_3a_env->dcInst, data_in, data_out, p_3a_env->frame_len / 2);
    audio_tick_out(AUDIO_DC_TIME);
#else
    memcpy(data_out, data_in, p_3a_env->frame_len);
#endif
    audio_dump_data(ADUMP_DC_OUT, (uint8_t *)data_out, p_3a_env->frame_len);
#endif

    if (p_3a_env->is_far_putted)
    {
        data_in = data_out;  //spframe
        data_out = outframe;
        data_in2 = NULL;
#ifdef WEBRTC_ANS_FIX
        if (!g_ans1_disabled)
        {
            audio_tick_in(AUDIO_ANS1_TIME);
            audio_ans_proc(p_3a_env->pNS_inst, data_in, data_out);
            audio_tick_out(AUDIO_ANS1_TIME);
            audio_dump_data(ADUMP_ANS_OUT, (uint8_t *)data_out, p_3a_env->frame_len);
        }
        else
#endif
        {
            memcpy(data_out, data_in, p_3a_env->frame_len);
        }

        data_in2 = data_out; //outframe
#ifdef WEBRTC_AECM
        {
            aec_input_para_t input_para;
            data_out = outframe2;
            input_para.nearframe = outframe; //ans out
            input_para.nearframe_clean = NULL;
            input_para.outframe = data_out; //outframe2
            if (p_3a_env->is_aecm_mic_putted == 0)
            {
                LOG_I("---first aecm input2 dump");
                p_3a_env->is_aecm_mic_putted = 1;
            }
            audio_dump_data(ADUMP_AECM_INPUT2, (uint8_t *)data_in, p_3a_env->frame_len);
            audio_tick_in(AUDIO_AEC_TIME);
#if PKG_USING_AUDIO_TEST_API
            {
                extern uint8_t audio_test_api_3a_is_enable();
                if (audio_test_api_3a_is_enable())
                {
                    audio_aec_proc(p_3a_env->aecmInst, &input_para, p_3a_env->samplerate);
                }
                else
                {
                    data_out = data_in;
                }
            }
#else
            audio_aec_proc(p_3a_env->aecmInst, &input_para, p_3a_env->samplerate); //data_out = data_in;
#endif
            audio_tick_out(AUDIO_AEC_TIME);
            audio_dump_data(ADUMP_AECM_OUT, (uint8_t *)data_out, p_3a_env->frame_len);
        }
#endif


        data_in = data_out; //outframe2
        data_out = outframe;    //outframe
        memcpy(data_out, data_in, p_3a_env->frame_len);

        temp = data_in;     //outframe2
        data_in = data_out; //outframe
        data_out = temp;    //outframe2
#ifdef WEBRTC_AGC_FIX
        if (g_uplink_agc)
        {
            audio_tick_in(AUDIO_UPAGC_TIME);
            audio_agc_proc(p_3a_env->agcInst, data_in, data_out, p_3a_env->samplerate);
            audio_tick_out(AUDIO_UPAGC_TIME);
        }
        else
#endif
        {
            memcpy(data_out, data_in, p_3a_env->frame_len);
        }
        audio_dump_data(ADUMP_AGC_OUT, (uint8_t *)data_out, p_3a_env->frame_len);


#ifdef PKG_USING_WEBRTC
        temp = data_in;     //outframe
        data_in = data_out; //outframe2;
        data_out = temp;    //outframe
        audio_tick_in(AUDIO_RAMPOUT_TIME);
        if (RampOut_Process(p_3a_env->rampOutInst, data_in, data_out, p_3a_env->frame_len / 2) < 0)
        {
            data_out = data_in;
        }
        audio_tick_out(AUDIO_RAMPOUT_TIME);
        audio_dump_data(ADUMP_RAMP_OUT_OUT, (uint8_t *)data_out, p_3a_env->frame_len);
#endif
        if (rt_ringbuffer_space_len(p_3a_env->rbuf_out) >= p_3a_env->frame_len)
        {
            rt_ringbuffer_put(p_3a_env->rbuf_out, (uint8_t *)data_out, p_3a_env->frame_len);
        }
        else
        {
            LOG_I("3a_w rbuf_out full\n");
        }

#ifdef AUDIO_MEM_ALLOC
        audio_mem_free(spframe);
        audio_mem_free(outframe);
        audio_mem_free(outframe2);
#endif
    }
    else
    {
        LOG_I("3a_w rbuf_far empty");
    }
}

void audio_3a_module_free(audio_3a_t *p_3a_env)
{
#ifdef WEBRTC_ANS_FIX
    RT_ASSERT(p_3a_env);
    RT_ASSERT(p_3a_env->pNS_inst);
    WebRtcNsx_Free(p_3a_env->pNS_inst);
    p_3a_env->pNS_inst = NULL;
    LOG_I("3a_w WebRtcNs_Free\n");
#endif
#ifdef WEBRTC_AECM
    WebRtcAecm_Free(p_3a_env->aecmInst);
    LOG_I("3a_w WebRtcAecm_Free\n");
    p_3a_env->aecmInst = NULL;
#endif
#ifdef WEBRTC_AGC_FIX
    WebRtcAgc_Free(p_3a_env->agcInst);
    LOG_I("3a_w WebRtcAgc_Free\n");
    p_3a_env->agcInst = NULL;
#if DOWN_LINK_AGC_ENABLE
    WebRtcAgc_Free(p_3a_env->dwlink_agcInst);
    LOG_I("3a_w WebRtcAgc_Free\n");
    p_3a_env->dwlink_agcInst = NULL;
#endif
#endif

#if g_dc_enabled
    DcCorrection_Free(p_3a_env->dcInst);
    p_3a_env->dcInst = NULL;
#endif
#if (g_ramin_enabled)
    RampIn_Free(p_3a_env->rampInInst);
    p_3a_env->rampInInst = NULL;
#endif
    RampOut_Free(p_3a_env->rampOutInst);
    p_3a_env->rampOutInst = NULL;

    rt_ringbuffer_destroy(g_audio_3a_env.rbuf_out);
    rt_ringbuffer_destroy(g_audio_3a_env.rbuf_far);
    rt_ringbuffer_destroy(p_3a_env->rbuf_dwlink);
#ifdef FFT_USING_ONCHIP
    HAL_FFT_DeInit(&(g_fft_env.fft_handle));
    rt_event_delete(g_fft_env.int_ev);
    HAL_RCC_DisableModule(RCC_MOD_FFT1);
#endif

}

extern uint32_t bt_connect_get_peer_type(void);
void audio_3a_open(uint32_t samplerate)
{
    audio_3a_t *thiz = &g_audio_3a_env;
#if defined(SOLUTION_WATCH) && defined(RT_USING_BT)
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
    if (g_audio_3a_env.state == 0)
    {
        g_3a_fifo = audio_mem_malloc(240);
        RT_ASSERT(g_3a_fifo);
        LOG_I("3a_w open samplearate=%ld", samplerate);
        if (samplerate == 8000)
        {
            g_audio_3a_env.frame_len = 160;
            g_audio_3a_env.samplerate = 8000;
            audio_3a_module_init(&g_audio_3a_env, 8000);
        }
        else
        {
            g_audio_3a_env.frame_len = 320;
            g_audio_3a_env.samplerate = 16000;
            audio_3a_module_init(&g_audio_3a_env, 16000);
        }
        g_audio_3a_env.state = 1;
        g_audio_3a_env.is_far_putted = 0;
        g_audio_3a_env.is_aecm_mic_putted = 0;
#ifdef AUDIO_BT_AUDIO
        bt_voice_open(samplerate);
#endif
    }
}

void audio_3a_close()
{
    if (g_audio_3a_env.state == 1)
    {
        LOG_I("3a_w close");
        audio_3a_module_free(&g_audio_3a_env);
        g_audio_3a_env.state = 0;
#ifdef AUDIO_BT_AUDIO
        bt_voice_close();
#endif
        audio_mem_free(g_3a_fifo);
        g_3a_fifo = NULL;
    }
}

void audio_3a_far_put(uint8_t *fifo, uint16_t fifo_size)
{
    audio_3a_t *p_3a_env = &g_audio_3a_env;
    if (p_3a_env->state == 0)
    {
        LOG_I("3a far put: closed");
        return;
    }
    if (rt_ringbuffer_space_len(p_3a_env->rbuf_far) >= fifo_size)
    {
        rt_ringbuffer_put(p_3a_env->rbuf_far, fifo, fifo_size);
    }
    else
    {
        LOG_I("3a_w far full");
        rt_ringbuffer_put_force(p_3a_env->rbuf_far, fifo, fifo_size);
    }

    uint16_t farframe[160];


    while (1)
    {
        uint16_t get_size;
        if (rt_ringbuffer_data_len(p_3a_env->rbuf_far) < p_3a_env->frame_len)
        {
            break;
        }
        get_size = rt_ringbuffer_get(p_3a_env->rbuf_far, (uint8_t *)farframe, p_3a_env->frame_len);
        RT_ASSERT(get_size == p_3a_env->frame_len);
        audio_dump_data(ADUMP_AECM_INPUT1, (uint8_t *)farframe, p_3a_env->frame_len);
#ifdef WEBRTC_AECM
        WebRtcAecm_BufferFarend(p_3a_env->aecmInst, (const int16_t *)farframe,  p_3a_env->frame_len / 2);
#endif

        if (p_3a_env->is_far_putted == 0)
        {
            LOG_I("---first far dump");
        }

        p_3a_env->is_far_putted = 1;
    }
}

uint8_t audio_3a_dnlink_buf_is_full(uint8_t size)
{
    audio_3a_t *p_3a_env = &g_audio_3a_env;

    if (rt_ringbuffer_space_len(p_3a_env->rbuf_dwlink) <= size)
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
    audio_3a_t *p_3a_env = &g_audio_3a_env;
    uint16_t putsize, getsize;
    if (g_audio_3a_env.state == 0)
    {
        LOG_I("3a_w downlink error: closed");
        return;
    }
#ifdef AUDIO_MEM_ALLOC
    uint8_t  *data1 = audio_mem_malloc(320);
    uint8_t  *data2 = audio_mem_malloc(320);
#else
    uint8_t  data1[320];
    uint8_t  data2[320];
#endif
    uint8_t  *data_in, *data_out;
    if (rt_ringbuffer_space_len(p_3a_env->rbuf_dwlink) >= size)
    {
        putsize = rt_ringbuffer_put(p_3a_env->rbuf_dwlink, fifo, size);
        RT_ASSERT(putsize == size);
    }
    else
    {
        LOG_I("3a_w rbuf_dwlink full");
    }

    while (rt_ringbuffer_data_len(p_3a_env->rbuf_dwlink) >= p_3a_env->frame_len)
    {
        data_in = data1;
        data_out = data_in;
        getsize = rt_ringbuffer_get(p_3a_env->rbuf_dwlink, (uint8_t *)data_in, p_3a_env->frame_len);
        RT_ASSERT(getsize == p_3a_env->frame_len);
        audio_dump_data(ADUMP_DOWNLINK, data_in, p_3a_env->frame_len);

#ifdef WEBRTC_AGC_FIX
#if DOWN_LINK_AGC_ENABLE
#if PKG_USING_AUDIO_TEST_API
        {
            extern uint8_t audio_test_api_3a_is_enable();
            g_u16_test_agc = 0;
            if (audio_test_api_3a_is_enable())
            {
                g_u16_test_agc = 1;
            }
        }
#endif
        if (g_u16_test_agc == 1)
        {
            data_out = data2;
            audio_agc_proc(p_3a_env->dwlink_agcInst, (int16_t *)data_in, (int16_t *)data_out, p_3a_env->samplerate);
            audio_dump_data(ADUMP_DOWNLINK_AGC, data_out, p_3a_env->frame_len);
        }
#endif
#endif
        speaker_ring_put(data_out, p_3a_env->frame_len);
    }
#ifdef AUDIO_MEM_ALLOC
    audio_mem_free(data1);
    audio_mem_free(data2);
#endif
}

void audio_3a_uplink(uint8_t *fifo, uint16_t fifo_size, uint8_t is_mute, uint8_t is_pdm)
{
    audio_3a_t *p_3a_env = &g_audio_3a_env;
    uint16_t putsize, getsize;
    UNUSED(is_pdm);
    RT_ASSERT(fifo_size == 320);
#if (defined(WEBRTC_ANS_FIX) || (defined(WEBRTC_AECM)) || (defined(WEBRTC_AGC_FIX)))
    if (p_3a_env->frame_len == 160) // 8K
    {
        audio_3a_data_process(p_3a_env, fifo, 160);
        audio_3a_data_process(p_3a_env, fifo + 160, 160);
        while (rt_ringbuffer_data_len(p_3a_env->rbuf_out) >= 120)
        {
            rt_ringbuffer_get(p_3a_env->rbuf_out, g_3a_fifo, 120);
            if (is_mute)
            {
                memset(g_3a_fifo, 0, 120);
            }
#ifdef AUDIO_BT_AUDIO
            msbc_encode_process(g_3a_fifo, 120);
#endif
        }
    }
    else
    {
        audio_3a_data_process(p_3a_env, fifo, fifo_size);
        audio_tick_in(AUDIO_MSBC_ENCODE_TIME);
        while (rt_ringbuffer_data_len(p_3a_env->rbuf_out) >= 240)
        {
            rt_ringbuffer_get(p_3a_env->rbuf_out, g_3a_fifo, 240);
            if (is_mute)
            {
                memset(g_3a_fifo, 0, 240);
            }
#ifdef AUDIO_BT_AUDIO
            msbc_encode_process(g_3a_fifo, 240);
#endif
        }
        audio_tick_out(AUDIO_MSBC_ENCODE_TIME);
    }
#endif
}

#ifdef WEBRTC_AECM
int set_3a_aec_en(int argc, char *argv[])
{
    rt_thread_t thread;

    if (argc != 2)
    {
        rt_kprintf("arg para num error: aec_en, aecdelay\n");
        return -1;
    }
    g_aec_delay = strtol(argv[1], NULL, 10);
    rt_kprintf("g_u16_test_aec=%d,u16_delay=%d\n", g_u16_test_aec, g_aec_delay);

    return 0;
}

MSH_CMD_EXPORT(set_3a_aec_en,    aec enabel test);
#endif

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

#ifdef WEBRTC_AGC_FIX
int set_3a_agc_en(int argc, char *argv[])
{
    rt_thread_t thread;

    if (argc != 2)
    {
        rt_kprintf("arg para num error: agc_en\n");
        return -1;
    }
    g_u16_test_agc = strtol(argv[1], NULL, 16);
    rt_kprintf("g_u16_test_agc=%d\n", g_u16_test_agc);

    return 0;
}

MSH_CMD_EXPORT(set_3a_agc_en,    agc enabel test);
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
    if (!memcmp(cmd, "agc=1", 5))
    {
        g_u16_test_agc = 1;
    }
    else if (!memcmp(cmd, "agc=0", 5))
    {
        g_u16_test_agc = 0;
    }
    else if (!memcmp(cmd, "cng=0", 5))
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

