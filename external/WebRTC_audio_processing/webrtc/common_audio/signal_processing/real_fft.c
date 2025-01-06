/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webrtc/common_audio/signal_processing/include/real_fft.h"

#include <stdlib.h>

#include "webrtc/common_audio/signal_processing/include/signal_processing_library.h"



#ifdef FFT_USING_CMSIS_DSP
    #include "arm_math.h"
    #include "arm_const_structs.h"
#endif
#include <rtthread.h>
#include "webrtc_mem.h"

#define AUDIO_MEM_ALLOC 1
//#define MALLOC_EVERYTIME   1

struct RealFFT
{
    int order;
#ifdef AUDIO_MEM_ALLOC
    int16_t *complex_buf;
    uint8_t fft_busy;
#endif
};

struct RealFFT *WebRtcSpl_CreateRealFFT(int order)
{
    struct RealFFT *self = NULL;

    if (order > kMaxFFTOrder || order < 0)
    {
        return NULL;
    }

    self = malloc(sizeof(struct RealFFT));
    RT_ASSERT(self);
    if (self == NULL)
    {
        return NULL;
    }
    self->order = order;
#ifdef AUDIO_MEM_ALLOC
    self->fft_busy = 0;
    self->complex_buf = (int16_t *)malloc((2 << kMaxFFTOrder) * sizeof(int16_t));
    RT_ASSERT(self->complex_buf);
#endif
    return self;
}

void WebRtcSpl_FreeRealFFT(struct RealFFT *self)
{
    if (self != NULL)
    {
#ifdef AUDIO_MEM_ALLOC
        free(self->complex_buf);
#endif
        free(self);
    }
}

// The C version FFT functions (i.e. WebRtcSpl_RealForwardFFT and
// WebRtcSpl_RealInverseFFT) are real-valued FFT wrappers for complex-valued
// FFT implementation in SPL.
#ifdef FFT_USING_CMSIS_DSP
const arm_cfft_instance_q15 *const cmsis_cfft_instance_table_q15[7] =
{
    &arm_cfft_sR_q15_len16,
    &arm_cfft_sR_q15_len32,
    &arm_cfft_sR_q15_len64,
    &arm_cfft_sR_q15_len128,
    &arm_cfft_sR_q15_len256,
    &arm_cfft_sR_q15_len512,
    &arm_cfft_sR_q15_len1024,
    //&arm_cfft_sR_q15_len2048,
    //&arm_cfft_sR_q15_len4096,
};
int WebRtcSpl_RealForwardFFT(struct RealFFT *self,
                             const int16_t *real_data_in,
                             int16_t *complex_data_out)
{
    int i = 0;
    int j = 0;
    int result = 0;
    int n = 1 << self->order;
    // The complex-value FFT implementation needs a buffer to hold 2^order
    // 16-bit COMPLEX numbers, for both time and frequency data.

    rt_uint32_t evt;

#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        int16_t *complex_buffer = malloc((2 << kMaxFFTOrder) * 2);
        RT_ASSERT(complex_buffer);
    #else
        int16_t *complex_buffer = self->complex_buf;
        RT_ASSERT(!self->fft_busy);
        self->fft_busy = 1;
    #endif
#else
    int16_t complex_buffer[2 << kMaxFFTOrder];
#endif


    // Insert zeros to the imaginary parts for complex forward FFT input.
    for (i = 0, j = 0; i < n; i += 1, j += 2)
    {
        complex_buffer[j] = real_data_in[i];
        complex_buffer[j + 1] = 0;
    };

    //WebRtcSpl_ComplexBitReverse(complex_buffer, self->order);
    //result = WebRtcSpl_ComplexFFT(complex_buffer, self->order, 1);

    arm_cfft_q15(cmsis_cfft_instance_table_q15[self->order - 4], (q15_t *)complex_buffer, 0, 1);

    // For real FFT output, use only the first N + 2 elements;
    // complex forward FFT.
    memcpy(complex_data_out, complex_buffer, sizeof(int16_t) * (n + 2));

#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        free(complex_buffer);
    #else
        self->fft_busy = 0;
    #endif
#endif


    return result;
}
int scale_calculate(const int16_t *complex_data_in, int n)
{
    int scale;
    int tmp16;

    tmp16 = WebRtcSpl_MaxAbsValueW16(complex_data_in, 2 * n);

    if (tmp16 > 16383)      //2^14 - 1
        scale = 0;
    else if (tmp16 > 8191)      //2^13 - 1
        scale = 1;
    else if (tmp16 > 4095)      //2^12 - 1
        scale = 2;
    else if (tmp16 > 2047)      //2^11 - 1
        scale = 3;
    else if (tmp16 > 1023)      //2^10 - 1
        scale = 4;
    else if (tmp16 > 511)   //2^9 - 1
        scale = 5;
    else if (tmp16 > 255)   //2^8 - 1
        scale = 6;
    else if (tmp16 > 127)   //2^7 - 1
        scale = 7;
    else
        scale = 8;

    return scale;
}
int WebRtcSpl_RealInverseFFT(struct RealFFT *self,
                             const int16_t *complex_data_in,
                             int16_t *real_data_out)
{
    int i = 0;
    int j = 0;
    int result = 0;
    int n = 1 << self->order;
    // Create the buffer specific to complex-valued FFT implementation.

    rt_uint32_t evt;
    int scale = 0;

    scale = scale_calculate(complex_data_in, n);
#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        int16_t *complex_buffer = malloc((2 << kMaxFFTOrder) * 2);
        RT_ASSERT(complex_buffer);
    #else
        int16_t *complex_buffer = self->complex_buf;
        RT_ASSERT(!self->fft_busy);
        self->fft_busy = 1;
    #endif
#else
    int16_t complex_buffer[2 << kMaxFFTOrder];
#endif

    // For n-point FFT, first copy the first n + 2 elements into complex
    // FFT, then construct the remaining n - 2 elements by real FFT's
    // conjugate-symmetric properties.
    memcpy(complex_buffer, complex_data_in, sizeof(int16_t) * (n + 2));

    for (i = 0; i < n + 2; i++)
    {
        complex_buffer[i] <<= scale;
    }
    for (i = n + 2; i < 2 * n; i += 2)
    {
        complex_buffer[i] = complex_data_in[2 * n - i] << scale;
        complex_buffer[i + 1] = -complex_data_in[2 * n - i + 1] << scale;
    }

    //WebRtcSpl_ComplexBitReverse(complex_buffer, self->order);
    //result = WebRtcSpl_ComplexIFFT(complex_buffer, self->order, 1);
    arm_cfft_q15(cmsis_cfft_instance_table_q15[self->order - 4], (q15_t *)complex_buffer, 1, 1);
    result = self->order - scale;

    // Strip out the imaginary parts of the complex inverse FFT output.
    for (i = 0, j = 0; i < n; i += 1, j += 2)
    {
        real_data_out[i] = complex_buffer[j];
    }

#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        free(complex_buffer);
    #else
        self->fft_busy = 0;
    #endif
#endif

    return result;
}
#else
#ifndef FFT_USING_ONCHIP
int WebRtcSpl_RealForwardFFT(struct RealFFT *self,
                             const int16_t *real_data_in,
                             int16_t *complex_data_out)
{
    int i = 0;
    int j = 0;
    int result = 0;
    int n = 1 << self->order;
    // The complex-value FFT implementation needs a buffer to hold 2^order
    // 16-bit COMPLEX numbers, for both time and frequency data.
#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        int16_t *complex_buffer = malloc((2 << kMaxFFTOrder) * 2);
        RT_ASSERT(complex_buffer);
    #else
        int16_t *complex_buffer = self->complex_buf;
        RT_ASSERT(!self->fft_busy);
        self->fft_busy = 1;
    #endif
#else
    int16_t complex_buffer[2 << kMaxFFTOrder];
#endif
    // Insert zeros to the imaginary parts for complex forward FFT input.
    for (i = 0, j = 0; i < n; i += 1, j += 2)
    {
        complex_buffer[j] = real_data_in[i];
        complex_buffer[j + 1] = 0;
    };

    WebRtcSpl_ComplexBitReverse(complex_buffer, self->order);
    result = WebRtcSpl_ComplexFFT(complex_buffer, self->order, 1);

    // For real FFT output, use only the first N + 2 elements from
    // complex forward FFT.
    memcpy(complex_data_out, complex_buffer, sizeof(int16_t) * (n + 2));
#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        free(complex_buffer);
    #else
        self->fft_busy = 0;
    #endif
#endif
    return result;
}

int WebRtcSpl_RealInverseFFT(struct RealFFT *self,
                             const int16_t *complex_data_in,
                             int16_t *real_data_out)
{
    int i = 0;
    int j = 0;
    int result = 0;
    int n = 1 << self->order;
    // Create the buffer specific to complex-valued FFT implementation.
#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        int16_t *complex_buffer = malloc((2 << kMaxFFTOrder) * 2);
        RT_ASSERT(complex_buffer);
    #else
        int16_t *complex_buffer = self->complex_buf;
        RT_ASSERT(!self->fft_busy);
        self->fft_busy = 1;
    #endif
#else
    int16_t complex_buffer[2 << kMaxFFTOrder];
#endif
    // For n-point FFT, first copy the first n + 2 elements into complex
    // FFT, then construct the remaining n - 2 elements by real FFT's
    // conjugate-symmetric properties.
    memcpy(complex_buffer, complex_data_in, sizeof(int16_t) * (n + 2));
    for (i = n + 2; i < 2 * n; i += 2)
    {
        complex_buffer[i] = complex_data_in[2 * n - i];
        complex_buffer[i + 1] = -complex_data_in[2 * n - i + 1];
    }

    WebRtcSpl_ComplexBitReverse(complex_buffer, self->order);
    result = WebRtcSpl_ComplexIFFT(complex_buffer, self->order, 1);

    // Strip out the imaginary parts of the complex inverse FFT output.
    for (i = 0, j = 0; i < n; i += 1, j += 2)
    {
        real_data_out[i] = complex_buffer[j];
    }
#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        free(complex_buffer);
    #else
        self->fft_busy = 0;
    #endif
#endif
    return result;
}

#else
#include "bf0_hal_fft.h"
extern fft_env_t g_fft_env;
int WebRtcSpl_RealForwardFFT(struct RealFFT *self,
                             const int16_t *real_data_in,
                             int16_t *complex_data_out)
{
    int i = 0;
    int j = 0;
    int result = 0;
    int n = 1 << self->order;
    // The complex-value FFT implementation needs a buffer to hold 2^order
    // 16-bit COMPLEX numbers, for both time and frequency data.

    rt_uint32_t evt;

#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        int16_t *complex_buffer = malloc((2 << kMaxFFTOrder) * 2);
        RT_ASSERT(complex_buffer);
    #else
        int16_t *complex_buffer = self->complex_buf;
        RT_ASSERT(!self->fft_busy);
        self->fft_busy = 1;
    #endif
#else
    int16_t complex_buffer[2 << kMaxFFTOrder];
#endif
    // Insert zeros to the imaginary parts for complex forward FFT input.
    for (i = 0, j = 0; i < n; i += 1, j += 2)
    {
        complex_buffer[j] = real_data_in[i];
        complex_buffer[j + 1] = 0;
    };

    //WebRtcSpl_ComplexBitReverse(complex_buffer, self->order);
    //result = WebRtcSpl_ComplexFFT(complex_buffer, self->order, 1);
    FFT_ConfigTypeDef config;
    HAL_StatusTypeDef status;

    memset(&config, 0, sizeof(config));

    config.bitwidth = 1;//0:8bit 1:16bit 2:32bit
    config.fft_length = (self->order - 4);
    config.ifft_flag = 0;//0:fft  1:ifft
    config.rfft_flag = 0;
    config.input_data = complex_buffer;
    config.output_data = complex_buffer;
#if 0
    status = HAL_FFT_StartFFT_IT(&(g_fft_env.fft_handle), &config);
    //rt_kprintf("fft status:%d, config:0x%x\n", status, &config);
    RT_ASSERT(HAL_OK == status);

    rt_event_recv(g_fft_env.int_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
#else
    status = HAL_FFT_StartFFT(&(g_fft_env.fft_handle), &config);
#endif
    // For real FFT output, use only the first N + 2 elements fromrt_free(out_buffer);
    // complex forward FFT.
    memcpy(complex_data_out, complex_buffer, sizeof(int16_t) * (n + 2));

#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        free(complex_buffer);
    #else
        self->fft_busy = 0;
    #endif
#endif


    return result;
}
int scale_calculate(const int16_t *complex_data_in, int n)
{
    int scale;
    int tmp16;

    tmp16 = WebRtcSpl_MaxAbsValueW16(complex_data_in, 2 * n);

    if (tmp16 > 16383)      //2^14 - 1
        scale = 0;
    else if (tmp16 > 8191)      //2^13 - 1
        scale = 1;
    else if (tmp16 > 4095)      //2^12 - 1
        scale = 2;
    else if (tmp16 > 2047)      //2^11 - 1
        scale = 3;
    else if (tmp16 > 1023)      //2^10 - 1
        scale = 4;
    else if (tmp16 > 511)   //2^9 - 1
        scale = 5;
    else if (tmp16 > 255)   //2^8 - 1
        scale = 6;
    else if (tmp16 > 127)   //2^7 - 1
        scale = 7;
    else
        scale = 8;

    return scale;
}
int WebRtcSpl_RealInverseFFT(struct RealFFT *self,
                             const int16_t *complex_data_in,
                             int16_t *real_data_out)
{
    int i = 0;
    int j = 0;
    int result = 0;
    int n = 1 << self->order;
    // Create the buffer specific to complex-valued FFT implementation.
    //int16_t *out_buffer;
    rt_uint32_t evt;
    int scale = 0;

    scale = scale_calculate(complex_data_in, n);

#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        int16_t *complex_buffer = malloc((2 << kMaxFFTOrder) * 2);
        RT_ASSERT(complex_buffer);
    #else
        int16_t *complex_buffer = self->complex_buf;
        RT_ASSERT(!self->fft_busy);
        self->fft_busy = 1;
    #endif
#else
    int16_t complex_buffer[2 << kMaxFFTOrder];

#endif
    // For n-point FFT, first copy the first n + 2 elements into complex
    // FFT, then construct the remaining n - 2 elements by real FFT's
    // conjugate-symmetric properties.
    memcpy(complex_buffer, complex_data_in, sizeof(int16_t) * (n + 2));

    for (i = 0; i < n + 2; i++)
    {
        complex_buffer[i] <<= scale;
    }
    for (i = n + 2; i < 2 * n; i += 2)
    {
        complex_buffer[i] = complex_data_in[2 * n - i] << scale;
        complex_buffer[i + 1] = -complex_data_in[2 * n - i + 1] << scale;
    }

    //WebRtcSpl_ComplexBitReverse(complex_buffer, self->order);
    //result = WebRtcSpl_ComplexIFFT(complex_buffer, self->order, 1);
    FFT_ConfigTypeDef config;
    HAL_StatusTypeDef status;

    memset(&config, 0, sizeof(config));

    config.bitwidth = 1;//0:8bit 1:16bit 2:32bit
    config.fft_length = (self->order - 4);
    config.ifft_flag = 1;//0:fft  1:ifft
    config.rfft_flag = 0;
    config.input_data = complex_buffer;
    config.output_data = complex_buffer;

#if 0
    status = HAL_FFT_StartFFT_IT(&(g_fft_env.fft_handle), &config);
    RT_ASSERT(HAL_OK == status);
    result = self->order - scale;
    rt_event_recv(g_fft_env.int_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
#else
    status = HAL_FFT_StartFFT(&(g_fft_env.fft_handle), &config);
    result = self->order - scale;
#endif
    // Strip out the imaginary parts of the complex inverse FFT output.
    for (i = 0, j = 0; i < n; i += 1, j += 2)
    {
        real_data_out[i] = complex_buffer[j];
    }

#ifdef AUDIO_MEM_ALLOC
    #if MALLOC_EVERYTIME
        free(complex_buffer);
    #else
        self->fft_busy = 0;
    #endif
#endif

    return result;
}
#endif
#endif
#undef AUDIO_MEM_ALLOC
