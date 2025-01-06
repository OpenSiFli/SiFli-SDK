/*
* Copyright 2019-2020, Synopsys, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-3-Clause license found in
* the LICENSE file in the root directory of this source tree.
*
*/
#include <stdlib.h>
#include <math.h>
#include "audio_cvsd.h"

#include "assert.h"
#ifdef _ARC
    #include "arc/arc_intrinsics.h"
#endif
#ifdef __FXAPI__
    #include "fxarc.h"
#else
    #include "stdint.h"
#endif

#ifdef DEBUG_OUTPUT
    #include "stdio.h"
#endif

#define PRECISION  10
#define POS_ACCUM_MAX  (32767<<PRECISION)

//const int PRECISION = 10;
const int BETA_EXP = 10;                 // Exponential of beta = 1024 = 2^10
const int ETA_EXP = 5;                   // Exponential of eta = 32 = 2^5
const int BIT_MASK = 0xF;                // Bit mask for J = 4 and K = 4
const int MAX_DELTA = 1280 << PRECISION;
const int MIN_DELTA = 10 << PRECISION;
//const int POS_ACCUM_MAX = 32767 << PRECISION;
const int NEG_ACCUM_MAX = -POS_ACCUM_MAX;


uint32_t bswap32(uint32_t input_byte)
{
    uint32_t d0, d1, d2, d3, output_byte;
    d0 = input_byte & 0xff;
    d1 = (input_byte >> 8) & 0xff;
    d2 = (input_byte >> 16) & 0xff;
    d3 = (input_byte >> 24) & 0xff;
    output_byte = (d0 << 24) | (d1 << 16) | (d2 << 8) | d3;

    return output_byte;
}


short cvsdInit(cvsd_t *cvsd)
{
    cvsd->accumulator = 0;
    cvsd->step_size = MIN_DELTA;
    cvsd->output_byte = 0;
    return 0;
}

void encode_bit(const int16_t **in, int32_t *accum,
                int32_t *step_size, uint32_t *output_byte)
{
    if ((*(*in)++ << PRECISION) >= *accum)
    {
        *output_byte <<= 1;
        *accum += *step_size;
        *accum = min(POS_ACCUM_MAX, *accum);
    }
    else
    {
        *output_byte = (*output_byte << 1) | 1;
        *accum -= *step_size;
        *accum = max(NEG_ACCUM_MAX, *accum);
    }
    *accum -= (*accum >> ETA_EXP);
    uint32_t tmp = *output_byte & BIT_MASK;
    if ((tmp == BIT_MASK) || (tmp == 0))
    {
        *step_size += MIN_DELTA;
        *step_size = min(MAX_DELTA, *step_size);
    }
    else
    {
        *step_size -= (*step_size >> BETA_EXP);
        *step_size = max(MIN_DELTA, *step_size);
    }
}

void cvsdEncode(cvsd_t *cvsd,  const short *in,  unsigned int input_frame_len,  unsigned int *out)
{
    const int double_word_len = 32;
    const uint32_t double_word = (double_word_len - 1);
    const int32_t rest_samples = input_frame_len & double_word;
    const int32_t input_frame_len_multiplied_32 = input_frame_len / double_word_len;
    uint32_t output_byte = cvsd->output_byte;
    int32_t accum = cvsd->accumulator;
    int32_t step_size = cvsd->step_size;
    uint8_t *out_pos = (uint8_t *)out;
    //After interpolation minimum input_frame_len can not be less then 8 samples
    assert((input_frame_len % 8) == 0);

    for (int i = 0; i < input_frame_len_multiplied_32; i++)
    {
        for (int i = 0; i < double_word_len; i++)
        {
            encode_bit(&in, &accum, &step_size, &output_byte);
        }
        uint32_t *out_pos32 = (uint32_t *)out_pos;
        *out_pos32 = bswap32(output_byte);
        out_pos += sizeof(int32_t);
    }

    if (rest_samples)
    {
        for (int i = 0; i < rest_samples; i++)
        {
            encode_bit(&in, &accum, &step_size, &output_byte);
        }

        const uint32_t shifted_output_byte = output_byte << (double_word_len - rest_samples);
        uint32_t *out_pos32 = (uint32_t *)out_pos;
        *out_pos32 = bswap32(shifted_output_byte);
    }

    cvsd->output_byte = output_byte;
    cvsd->accumulator = accum;
    cvsd->step_size = step_size;
}

void cvsdDecode(cvsd_t *cvsd,  const unsigned char *in,  unsigned int input_frame_len,  short *out)
{
    uint32_t runner = cvsd->output_byte;
    int32_t accum = cvsd->accumulator;
    int32_t step_size = cvsd->step_size;
    for (int i = 0; i < input_frame_len; i++)
    {
#pragma clang loop unroll(full)
        for (int bit_counter = 128; bit_counter > 0; (bit_counter >>= 1))
        {
            *out++ = (accum >> PRECISION);
            if (in[i] & bit_counter)
            {
                runner = (runner << 1) | 1;
                accum -= step_size;
                accum = max(NEG_ACCUM_MAX, accum);
            }
            else
            {
                runner <<= 1;
                accum += step_size;
                accum = min(POS_ACCUM_MAX, accum);
            }
            accum -= (accum >> ETA_EXP);
            int32_t tmp = runner & BIT_MASK;
            if ((tmp == BIT_MASK) || (tmp == 0))
            {
                step_size += MIN_DELTA;
                step_size = min(MAX_DELTA, step_size);
            }
            else
            {
                step_size -= (step_size >> BETA_EXP);
                step_size = max(MIN_DELTA, step_size);
            }
        }
    }

    cvsd->accumulator = accum;
    cvsd->step_size = step_size;
    cvsd->output_byte = runner;
}
