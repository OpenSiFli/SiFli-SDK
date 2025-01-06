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
#include "dfs_file.h"
#include "dfs_posix.h"

#include "Soundplus_adapter.h"



#include "audio_mem.h"
#define DBG_TAG           "audio_3a"
#define DBG_LVL           AUDIO_DBG_LVL //LOG_LVL_WARNING
#include "log.h"
#include "audio_server.h"

#define SOUNDPLUS_FRAME_SIZE            (480)   //15ms

typedef struct audio_3a_tag
{
    uint8_t     state;
    uint8_t     is_cvsd;
    uint8_t     cvsd_dump_count;
    uint8_t     is_far_putted;
    uint8_t     is_far_using;
    uint16_t    frame_size;    //unit is bytes
    uint16_t    samplerate;
    uint8_t    *rbuf_dwlink_pool;
    uint8_t    *mic_far;
    struct rt_ringbuffer rbuf_dwlink;
} audio_3a_t;


static audio_3a_t g_audio_3a_env =
{
    .state  = 0,
    .samplerate = 16000,
};
static uint16_t g_mic_delay_ref = 434 ;


static void audio_3a_module_init(audio_3a_t *env, uint32_t samplerate)
{
    uint32_t size = SOUNDPLUS_FRAME_SIZE * 2;

    env->mic_far = audio_mem_malloc(size + g_mic_delay_ref * 2);
    RT_ASSERT(env->mic_far);

    env->rbuf_dwlink_pool = audio_mem_malloc(size);
    RT_ASSERT(env->rbuf_dwlink_pool);
    rt_ringbuffer_init(&env->rbuf_dwlink, env->rbuf_dwlink_pool, size);
    env->state = 1;
}

static void audio_3a_module_free(audio_3a_t *env)
{
    env->state = 0;
    audio_mem_free(env->mic_far);
    rt_ringbuffer_reset(&env->rbuf_dwlink);
    audio_mem_free(env->rbuf_dwlink_pool);
    env->mic_far = NULL;
    env->rbuf_dwlink_pool = NULL;
}


void audio_3a_open(uint32_t samplerate)
{
    audio_3a_t *env = &g_audio_3a_env;
    if (env->state == 0)
    {
        env->is_far_putted = 0;

        LOG_I("3a_w open samplearate=%ld", samplerate);
        if (samplerate == 8000)
        {
            soundplus_init(2);
            soundplus_rx_init(2);
            env->samplerate = 8000;

            env->frame_size = SOUNDPLUS_FRAME_SIZE / 2;

            audio_3a_module_init(env, 8000);
        }
        else
        {
            soundplus_init(1);
            soundplus_rx_init(1);
            env->samplerate = 16000;
            env->frame_size = SOUNDPLUS_FRAME_SIZE;
            audio_3a_module_init(env, 16000);
        }
        bt_voice_open(samplerate);
    }
}

void audio_3a_close()
{
    if (g_audio_3a_env.state == 1)
    {
        LOG_I("3a_w close");
        audio_3a_module_free(&g_audio_3a_env);
        bt_voice_close();

        soundplus_deinit();
        soundplus_rx_deinit();

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
    audio_3a_t *env = &g_audio_3a_env;
    uint16_t putsize, getsize;
    if (g_audio_3a_env.state == 0)
    {
        LOG_I("3a_w downlink error: closed");
        return;
    }

    uint8_t  data1[SOUNDPLUS_FRAME_SIZE];
    uint8_t  data2[SOUNDPLUS_FRAME_SIZE];

    uint8_t  *data_in, *data_out;
    if (rt_ringbuffer_space_len(&env->rbuf_dwlink) >= size)
    {
        putsize = rt_ringbuffer_put(&env->rbuf_dwlink, fifo, size);
        RT_ASSERT(putsize == size);
    }
    else
    {
        LOG_I("3a_w rbuf_dwlink full");
        putsize = rt_ringbuffer_put_force(&env->rbuf_dwlink, fifo, size);
    }

    while (rt_ringbuffer_data_len(&env->rbuf_dwlink) >= env->frame_size)
    {
        data_in = data1;
        data_out = data2;
        getsize = rt_ringbuffer_get(&env->rbuf_dwlink, (uint8_t *)data_in, env->frame_size);

        audio_dump_data(ADUMP_DOWNLINK, data_in, env->frame_size);


        soundplus_deal_Rx((int16_t *)data_out, (int16_t *)data_in, env->frame_size / 2);

        audio_dump_data(ADUMP_DOWNLINK_AGC, data_out, env->frame_size);

        //put to speaker buffer for playing
        speaker_ring_put(data_out, env->frame_size);
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
#if DEBUG_FRAME_SYNC
    RT_ASSERT(env->is_far_using == 0);
#endif
    memcpy(env->mic_far + g_mic_delay_ref * 2, fifo, fifo_size);
#if DEBUG_FRAME_SYNC
    RT_ASSERT(env->is_far_using == 0);
#endif
    env->is_far_putted = 1;

}

void audio_3a_uplink(uint8_t *fifo, uint16_t fifo_size, uint8_t is_mute, uint8_t is_pdm)
{
    audio_3a_t *env = &g_audio_3a_env;
    uint16_t get_size;
    int ret, ref_index;

    int16_t refframe[SOUNDPLUS_FRAME_SIZE / 2];

    RT_ASSERT(fifo_size == SOUNDPLUS_FRAME_SIZE);
    if (0 == env->is_far_putted)
    {
        LOG_I("wait far put");
        is_mute = 1;
        goto skip_3a_up;
    }
#if DEBUG_FRAME_SYNC
    extern void save_mic_tick();
    save_mic_tick();
#endif
#if DEBUG_FRAME_SYNC
    env->is_far_using = 1;
#endif
    memcpy(refframe, env->mic_far, SOUNDPLUS_FRAME_SIZE);
    memcpy(env->mic_far, env->mic_far + SOUNDPLUS_FRAME_SIZE, g_mic_delay_ref * 2);
#if DEBUG_FRAME_SYNC
    env->is_far_using = 0;
#endif
    audio_dump_data(ADUMP_AECM_INPUT1, (uint8_t *)refframe, SOUNDPLUS_FRAME_SIZE);
    audio_dump_data(ADUMP_AECM_INPUT2, fifo, SOUNDPLUS_FRAME_SIZE);

    ref_index = 0;

do_8k_again:
    ret = soundplus_deal_Tx((int16_t *)fifo, &refframe[ref_index], env->frame_size / 2, env->frame_size / 2);
    if (ret)
    {
        LOG_I("soundplus deal tx error");
        return;
    }

skip_3a_up:

    if (is_mute)
    {
        memset(fifo, 0, fifo_size);
    }

    audio_dump_data(ADUMP_RAMP_OUT_OUT, fifo, SOUNDPLUS_FRAME_SIZE);

    if (env->samplerate == 8000)
    {
        //msbc encode 120 bytes for 8K
        msbc_encode_process(fifo, 120);
        msbc_encode_process(fifo + 120, 120);
        if (ref_index == 0)
        {
            ref_index = SOUNDPLUS_FRAME_SIZE / 4;
            fifo += SOUNDPLUS_FRAME_SIZE / 2;
            goto do_8k_again;
        }
        return;
    }
    else
    {
        //msbc encode one frame is 240 bytes for 16K samplerate
        msbc_encode_process(fifo, 240);
        msbc_encode_process(fifo + 240, 240);
        return;
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
uint32_t g_record_time = 10;
void audio_command_process(uint8_t *cmd_1)
{

}

#endif

