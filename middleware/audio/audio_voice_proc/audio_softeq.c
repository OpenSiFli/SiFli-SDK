// freqshift.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SAMPLE_RATE 16000                   // Sample rate
#define FRAME_LENGTH ((16*15)>> 1)          // Frame length, 7.5ms data
#define SOFTEQ_PARAM_STAGE                  10
#define SOFTEQ_PARAM                        5
#define MAX_INT16                           32768

static float g_eq_param_f[SOFTEQ_PARAM_STAGE * SOFTEQ_PARAM];
static float d0_array[SOFTEQ_PARAM_STAGE], d1_array[SOFTEQ_PARAM_STAGE];
void soft_eq(int16_t *data_in, int16_t *data_out, int len, float *param, int stage)
{
    float d_in_f, d_out_f;
    for (int i = 0; i < stage; i++)
    {
        float d0, d1;
        d0 = d0_array[i], d1 = d1_array[i];
        for (int j = 0; j < len; j++)
        {
            int index = i * SOFTEQ_PARAM;
            float m0, m1, m2;
            d_in_f = data_in[j];
            d_in_f /= MAX_INT16;
            m0 = d_in_f * param[index];
            m1 = d_in_f * param[index + 1];
            m2 = d_in_f * param[index + 2];
            d_out_f = (m0 + d0) * 4;
            d0 = m1 + param[index + 3] * d_out_f + d1;
            d1 = m2 + param[index + 4] * d_out_f;
            if (d_out_f > 1)
                d_out_f = 1;
            if (d_out_f < -1)
                d_out_f = -1;
            data_out[j] = (int16_t)(d_out_f * MAX_INT16);
        }
        memcpy(data_in, data_out, len * sizeof(int16_t));
        d0_array[i] = d0;
        d1_array[i] = d1;
    }
}

void soft_eq_param(int32_t *param, float *param_f)
{
    int index, index2;
    for (int i = 0; i < SOFTEQ_PARAM_STAGE; i++)
        for (int j = 0; j < SOFTEQ_PARAM; j++)
        {
            index = i * SOFTEQ_PARAM + j;
            index2 = index;
            // Switch colume 2 and 3
            if (j == 2)
                index2++;
            else if (j == 3)
                index2--;
            if (param[index] < (1 << 23))
                param_f[index2] = (float)param[index];
            else
                param_f[index2] = (float)(param[index] - (1 << 24));
            param_f[index2] /= (1 << 23);
        }
}

// Test code in PC
#if 0
int16_t input_data[FRAME_LENGTH];
int16_t output_data[FRAME_LENGTH];
int32_t g_eq_param[] =
{
    2092371,   12612199,    4164961,    2072738,   14709203,
    2097628,   12605565,    4171651,    2074165,   14702575,
    2094955,   12629019,    4148197,    2053752,   14725661,
    2080652,   12761629,    4015587,    1941629,   14852087,
    2098833,   12897838,    3879378,    1805187,   14970349,
    2169530,   12992160,    3785056,    1668271,   15036566,
    2090754,   13443037,    3334179,    1402456,   15381158,
    2296306,   14830802,    1946414,     628332,   15949730,
    1948206,   15934672,     842544,     533697,   16392465,
    1419993,   16460420,    1225627,     256146,   16289398,
};

int main(int argc, char *argv[])
{
    FILE *fp = fopen(argv[1], "rb");
    FILE *fp_out = fopen(argv[2], "wb+");
    int len;
    int stage;

    stage = atoi(argv[3]);
    if (fp == NULL)
    {
        printf("Could not open %s!\n", argv[1]);
        exit(-1);
    }
    if (fp_out == NULL)
    {
        printf("Could not open %s for write!\n", argv[2]);
        exit(-2);
    }
    soft_eq_param(g_eq_param, g_eq_param_f);

    // For each stream , need to restart EQ
    memset(d0_array, 0, sizeof(d0_array));
    memset(d1_array, 0, sizeof(d1_array));
    len = fread(input_data, sizeof(int16_t), FRAME_LENGTH, fp);
    int first = 1;
    while (len == FRAME_LENGTH)
    {
        soft_eq(input_data, output_data, FRAME_LENGTH, g_eq_param_f, stage);
        fwrite(output_data, sizeof(int16_t), FRAME_LENGTH, fp_out);
        len = fread(input_data, sizeof(int16_t), FRAME_LENGTH, fp);
    }
    fclose(fp_out);
    fclose(fp);
    printf("Finished\n");
}
#endif