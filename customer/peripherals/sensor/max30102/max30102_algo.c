/**
  ******************************************************************************
  * @file   max30102_algo.c
  * @author Sifli software development team
  * @brief  MAX30102 HR/SpO2 algorithm implementation (rewritten).
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include "max30102_algo.h"

/* Algorithm tuning constants. */
#define MAX30102_INVALID_VALUE    (-999)
#define MAX30102_SPO2_TABLE_SIZE  184
#define MAX30102_MAX_PEAKS        15
#define MAX30102_MAX_RATIOS       5

#define MAX30102_HR_MIN_BPM       40
#define MAX30102_HR_MAX_BPM       180

#define MAX30102_MIN_DC           1000
#define MAX30102_MAX_DC           200000
#define MAX30102_MIN_AMP_ABS      500
#define MAX30102_MIN_AMP_RATIO    50   /* mean/ratio => 2% */

#define MAX30102_MA_SIZE          4

/* SpO2 lookup table indexed by the ratio value. */
static const uint8_t max30102_spo2_table[MAX30102_SPO2_TABLE_SIZE] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99,
    99, 99, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 99, 99,
    99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 97, 97, 96, 96, 96, 96, 95, 95,
    95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 90, 90, 89, 89, 89, 88,
    88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 80, 80, 79, 78,
    78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 66, 66,
    65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
    49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31,
    30, 29, 28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10,
    9, 7, 6, 5, 3, 2, 1
};

/* Internal working buffers (not re-entrant). */
static int32_t max30102_ir[MAX30102_BUFFER_SIZE];
static int32_t max30102_min(int32_t a, int32_t b)
{
    return (a < b) ? a : b;
}

static void max30102_sort_ascend(int32_t *arr, int32_t size)
{
    int32_t i;
    for (i = 1; i < size; i++)
    {
        int32_t temp = arr[i];
        int32_t j;
        for (j = i; j > 0 && temp < arr[j - 1]; j--)
        {
            arr[j] = arr[j - 1];
        }
        arr[j] = temp;
    }
}

static int32_t max30102_median(int32_t *arr, int32_t size)
{
    if (size <= 0)
    {
        return MAX30102_INVALID_VALUE;
    }
    max30102_sort_ascend(arr, size);
    if (size & 1)
    {
        return arr[size / 2];
    }
    return (arr[size / 2 - 1] + arr[size / 2]) / 2;
}

static int32_t max30102_min_distance(void)
{
    int32_t d = (MAX30102_FS * 60) / MAX30102_HR_MAX_BPM;
    return max30102_min(d, MAX30102_BUFFER_SIZE - 1);
}

static int32_t max30102_max_distance(void)
{
    return (MAX30102_FS * 60) / MAX30102_HR_MIN_BPM;
}

static int32_t max30102_find_peaks(int32_t *x, int32_t size, int32_t min_height, int32_t min_dist, int32_t *locs)
{
    int32_t count = 0;
    int32_t last_peak = -min_dist;
    int32_t i;

    for (i = 1; i < size - 1; i++)
    {
        if (x[i] > min_height && x[i] > x[i - 1] && x[i] >= x[i + 1])
        {
            if (i - last_peak >= min_dist)
            {
                if (count < MAX30102_MAX_PEAKS)
                {
                    locs[count++] = i;
                    last_peak = i;
                }
            }
            else
            {
                if (count > 0 && x[i] > x[last_peak])
                {
                    locs[count - 1] = i;
                    last_peak = i;
                }
            }
        }
    }

    return count;
}

int max30102_calc_hr_spo2(const uint32_t *ir_buf,
                          int32_t ir_len,
                          const uint32_t *red_buf,
                          int32_t *spo2,
                          int8_t *spo2_valid,
                          int32_t *hr,
                          int8_t *hr_valid)
{
    int32_t i;
    int32_t k;
    int32_t buffer_len = MAX30102_BUFFER_SIZE;
    int32_t peaks[MAX30102_MAX_PEAKS];
    int32_t peak_count;
    int32_t intervals[MAX30102_MAX_PEAKS];
    int32_t interval_count = 0;
    int32_t ratios[MAX30102_MAX_RATIOS];
    int32_t ratio_count = 0;
    int32_t min_dist;
    int32_t max_dist;
    int32_t min_height;
    int32_t min_val;
    int32_t max_val;
    int32_t amp;
    uint32_t ir_mean = 0;
    uint32_t red_mean = 0;

    if (!ir_buf || !red_buf || !spo2 || !spo2_valid || !hr || !hr_valid)
    {
        return -1;
    }

    *spo2 = MAX30102_INVALID_VALUE;
    *spo2_valid = 0;
    *hr = MAX30102_INVALID_VALUE;
    *hr_valid = 0;

    if (ir_len < MAX30102_BUFFER_SIZE)
    {
        return -1;
    }

    for (k = 0; k < buffer_len; k++)
    {
        ir_mean += ir_buf[k];
        red_mean += red_buf[k];
    }
    ir_mean /= (uint32_t)buffer_len;
    red_mean /= (uint32_t)buffer_len;

    if (ir_mean < MAX30102_MIN_DC || ir_mean > MAX30102_MAX_DC ||
        red_mean < MAX30102_MIN_DC || red_mean > MAX30102_MAX_DC)
    {
        return 0;
    }

    min_val = 2147483647;
    max_val = -2147483647;
    for (k = 0; k < buffer_len; k++)
    {
        int32_t v = (int32_t)ir_buf[k] - (int32_t)ir_mean;
        max30102_ir[k] = v;
        if (v < min_val)
        {
            min_val = v;
        }
        if (v > max_val)
        {
            max_val = v;
        }
    }

    amp = max_val - min_val;
    if (amp < MAX30102_MIN_AMP_ABS || amp < (int32_t)(ir_mean / MAX30102_MIN_AMP_RATIO))
    {
        return 0;
    }

    for (k = 0; k < buffer_len - (MAX30102_MA_SIZE - 1); k++)
    {
        int32_t sum = 0;
        for (i = 0; i < MAX30102_MA_SIZE; i++)
        {
            sum += max30102_ir[k + i];
        }
        max30102_ir[k] = sum / MAX30102_MA_SIZE;
    }

    min_val = 2147483647;
    max_val = -2147483647;
    for (k = 0; k < buffer_len - (MAX30102_MA_SIZE - 1); k++)
    {
        if (max30102_ir[k] < min_val)
        {
            min_val = max30102_ir[k];
        }
        if (max30102_ir[k] > max_val)
        {
            max_val = max30102_ir[k];
        }
    }

    min_height = max_val / 2;
    if (min_height < max_val / 3)
    {
        min_height = max_val / 3;
    }
    if (min_height < 1)
    {
        min_height = 1;
    }

    min_dist = max30102_min_distance();
    max_dist = max30102_max_distance();

    peak_count = max30102_find_peaks(max30102_ir, buffer_len - (MAX30102_MA_SIZE - 1),
                                     min_height, min_dist, peaks);

    if (peak_count >= 2)
    {
        for (k = 1; k < peak_count; k++)
        {
            int32_t d = peaks[k] - peaks[k - 1];
            if (d >= min_dist && d <= max_dist)
            {
                intervals[interval_count++] = d;
            }
        }

        if (interval_count > 0)
        {
            int32_t interval = max30102_median(intervals, interval_count);
            if (interval > 0)
            {
                *hr = (int32_t)((60 * MAX30102_FS) / interval);
                if (*hr >= MAX30102_HR_MIN_BPM && *hr <= MAX30102_HR_MAX_BPM)
                {
                    *hr_valid = 1;
                }
            }
        }
    }

    if (!*hr_valid)
    {
        return 0;
    }

    for (k = 0; k < peak_count - 1 && ratio_count < MAX30102_MAX_RATIOS; k++)
    {
        int32_t p = peaks[k];
        int32_t p_next = peaks[k + 1];
        int32_t valley = p;
        int32_t ir_min = (int32_t)ir_buf[p];
        int32_t i_min = p;

        if (p_next <= p + 2)
        {
            continue;
        }

        for (i = p + 1; i < p_next; i++)
        {
            if ((int32_t)ir_buf[i] < ir_min)
            {
                ir_min = (int32_t)ir_buf[i];
                i_min = i;
            }
        }

        valley = i_min;

        if (valley <= 0 || valley >= buffer_len)
        {
            continue;
        }

        {
            int32_t ir_peak = (int32_t)ir_buf[p];
            int32_t red_peak = (int32_t)red_buf[p];
            int32_t ir_valley = (int32_t)ir_buf[valley];
            int32_t red_valley = (int32_t)red_buf[valley];
            int32_t ir_ac = ir_peak - ir_valley;
            int32_t red_ac = red_peak - red_valley;

            if (ir_ac <= 0 || red_ac <= 0 || ir_peak <= 0 || red_peak <= 0)
            {
                continue;
            }

            {
                int64_t nume = (int64_t)red_ac * (int64_t)ir_peak * 20;
                int64_t denom = (int64_t)ir_ac * (int64_t)red_peak;
                if (denom > 0)
                {
                    int32_t ratio = (int32_t)(nume / denom);
                    ratios[ratio_count++] = ratio;
                }
            }
        }
    }

    if (ratio_count > 0)
    {
        int32_t ratio_avg = max30102_median(ratios, ratio_count);
        if (ratio_avg > 2 && ratio_avg < MAX30102_SPO2_TABLE_SIZE)
        {
            *spo2 = max30102_spo2_table[ratio_avg];
            *spo2_valid = 1;
        }
    }

    return 0;
}
