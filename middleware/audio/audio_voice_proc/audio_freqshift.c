﻿// freqshift.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define FREQ_SHIFT  10                      // Shifting 10Hz

#define HB_FO  16                           // filter order
#define HB_GRD  (HB_FO>>1)                  // fo/2,filter group delay
#define SAMPLE_RATE 16000                   // Sample rate
#define FRAME_LENGTH ((16*15)>> 1)          // Frame length, 7.5ms data
#define DELTA_PH (1024.0*FREQ_SHIFT/SAMPLE_RATE)    // Delta phase

static float hb_reg[HB_FO];
static float hb_buf[HB_GRD];
static float hb_in_f[FRAME_LENGTH];
static float hb_out_f[FRAME_LENGTH];
static float mix_in_r[FRAME_LENGTH];
#define mix_in_i hb_out_f
static float phase_acc;

static const float hb_coef[] =      // filter coef
{
    0.000061035156250,
    - 0.265533447265625,
    - 0.000061035156250,
    - 0.128784179687500,
    - 0.000183105468750,
    - 0.212615966796875,
    0.000274658203125,
    - 0.636840820312500,
    0,
    0.636840820312500,
    - 0.000274658203125,
    0.212615966796875,
    0.000183105468750,
    0.128784179687500,
    0.000061035156250,
    0.265533447265625,
    - 0.000061035156250
};

static const  float sin_table[] =  // sin([0:256] * pi / 512);
{
    0,
    0.006135884649154,
    0.012271538285720,
    0.018406729905805,
    0.024541228522912,
    0.030674803176637,
    0.036807222941359,
    0.042938256934941,
    0.049067674327418,
    0.055195244349690,
    0.061320736302209,
    0.067443919563664,
    0.073564563599667,
    0.079682437971430,
    0.085797312344440,
    0.091908956497133,
    0.098017140329561,
    0.104121633872055,
    0.110222207293883,
    0.116318630911905,
    0.122410675199216,
    0.128498110793793,
    0.134580708507126,
    0.140658239332849,
    0.146730474455362,
    0.152797185258443,
    0.158858143333861,
    0.164913120489970,
    0.170961888760301,
    0.177004220412149,
    0.183039887955141,
    0.189068664149806,
    0.195090322016128,
    0.201104634842092,
    0.207111376192219,
    0.213110319916091,
    0.219101240156870,
    0.225083911359793,
    0.231058108280671,
    0.237023605994367,
    0.242980179903264,
    0.248927605745720,
    0.254865659604515,
    0.260794117915276,
    0.266712757474898,
    0.272621355449949,
    0.278519689385053,
    0.284407537211272,
    0.290284677254462,
    0.296150888243624,
    0.302005949319228,
    0.307849640041535,
    0.313681740398892,
    0.319502030816016,
    0.325310292162263,
    0.331106305759876,
    0.336889853392220,
    0.342660717311994,
    0.348418680249435,
    0.354163525420490,
    0.359895036534988,
    0.365612997804774,
    0.371317193951837,
    0.377007410216418,
    0.382683432365090,
    0.388345046698826,
    0.393992040061048,
    0.399624199845647,
    0.405241314004990,
    0.410843171057904,
    0.416429560097637,
    0.422000270799800,
    0.427555093430282,
    0.433093818853152,
    0.438616238538528,
    0.444122144570429,
    0.449611329654607,
    0.455083587126344,
    0.460538710958240,
    0.465976495767966,
    0.471396736825998,
    0.476799230063322,
    0.482183772079123,
    0.487550160148436,
    0.492898192229784,
    0.498227666972782,
    0.503538383725718,
    0.508830142543107,
    0.514102744193222,
    0.519355990165590,
    0.524589682678469,
    0.529803624686295,
    0.534997619887097,
    0.540171472729893,
    0.545324988422046,
    0.550457972936605,
    0.555570233019602,
    0.560661576197336,
    0.565731810783613,
    0.570780745886967,
    0.575808191417845,
    0.580813958095765,
    0.585797857456439,
    0.590759701858874,
    0.595699304492433,
    0.600616479383869,
    0.605511041404326,
    0.610382806276309,
    0.615231590580627,
    0.620057211763289,
    0.624859488142386,
    0.629638238914927,
    0.634393284163645,
    0.639124444863776,
    0.643831542889791,
    0.648514401022112,
    0.653172842953777,
    0.657806693297079,
    0.662415777590172,
    0.666999922303637,
    0.671558954847018,
    0.676092703575316,
    0.680600997795453,
    0.685083667772700,
    0.689540544737067,
    0.693971460889654,
    0.698376249408973,
    0.702754744457225,
    0.707106781186547,
    0.711432195745216,
    0.715730825283819,
    0.720002507961382,
    0.724247082951467,
    0.728464390448225,
    0.732654271672413,
    0.736816568877370,
    0.740951125354959,
    0.745057785441466,
    0.749136394523459,
    0.753186799043613,
    0.757208846506485,
    0.761202385484262,
    0.765167265622459,
    0.769103337645580,
    0.773010453362737,
    0.776888465673232,
    0.780737228572094,
    0.784556597155575,
    0.788346427626606,
    0.792106577300212,
    0.795836904608884,
    0.799537269107905,
    0.803207531480645,
    0.806847553543799,
    0.810457198252595,
    0.814036329705948,
    0.817584813151584,
    0.821102514991105,
    0.824589302785025,
    0.828045045257756,
    0.831469612302545,
    0.834862874986380,
    0.838224705554838,
    0.841554977436898,
    0.844853565249707,
    0.848120344803297,
    0.851355193105265,
    0.854557988365401,
    0.857728610000272,
    0.860866938637767,
    0.863972856121587,
    0.867046245515693,
    0.870086991108711,
    0.873094978418290,
    0.876070094195407,
    0.879012226428633,
    0.881921264348355,
    0.884797098430938,
    0.887639620402854,
    0.890448723244758,
    0.893224301195515,
    0.895966249756185,
    0.898674465693954,
    0.901348847046022,
    0.903989293123443,
    0.906595704514915,
    0.909167983090522,
    0.911706032005430,
    0.914209755703531,
    0.916679059921043,
    0.919113851690058,
    0.921514039342042,
    0.923879532511287,
    0.926210242138311,
    0.928506080473216,
    0.930766961078984,
    0.932992798834739,
    0.935183509938947,
    0.937339011912575,
    0.939459223602190,
    0.941544065183021,
    0.943593458161960,
    0.945607325380521,
    0.947585591017741,
    0.949528180593037,
    0.951435020969008,
    0.953306040354194,
    0.955141168305771,
    0.956940335732209,
    0.958703474895872,
    0.960430519415566,
    0.962121404269042,
    0.963776065795440,
    0.965394441697689,
    0.966976471044852,
    0.968522094274417,
    0.970031253194544,
    0.971503890986252,
    0.972939952205560,
    0.974339382785576,
    0.975702130038529,
    0.977028142657754,
    0.978317370719628,
    0.979569765685441,
    0.980785280403230,
    0.981963869109555,
    0.983105487431216,
    0.984210092386929,
    0.985277642388941,
    0.986308097244599,
    0.987301418157858,
    0.988257567730749,
    0.989176509964781,
    0.990058210262297,
    0.990902635427780,
    0.991709753669100,
    0.992479534598710,
    0.993211949234795,
    0.993906970002356,
    0.994564570734255,
    0.995184726672197,
    0.995767414467660,
    0.996312612182778,
    0.996820299291166,
    0.997290456678690,
    0.997723066644192,
    0.998118112900149,
    0.998475580573295,
    0.998795456205172,
    0.999077727752645,
    0.999322384588350,
    0.999529417501093,
    0.999698818696204,
    0.999830581795823,
    0.999924701839145,
    0.999981175282601,
    1.000000000000000,
};

static void mixer_lut(float *cos_data, float *sin_data, int phase_index)
{
    if (phase_index <= 256)
        * cos_data = sin_table[256 - phase_index];
    else if (phase_index <= 512)
        * cos_data = (-1) * sin_table[phase_index - 256];
    else if (phase_index <= 768)
        * cos_data = (-1) * sin_table[768 - phase_index];
    else
        *cos_data = sin_table[phase_index - 768];

    if (phase_index <= 256)
        *sin_data = sin_table[phase_index];
    else if (phase_index <= 512)
        *sin_data = sin_table[512 - phase_index];
    else if (phase_index <= 768)
        *sin_data = (-1) * sin_table[phase_index - 512];
    else
        *sin_data = (-1) * sin_table[1024 - phase_index];
}

static void hb_filter(float *hb_in_f, float *hb_out_f, float *hb_reg, int len)
{
    float hs;

    for (int i = 0; i < len; i++)
    {
        hs = 0.0;
        for (int j = 0; j < HB_FO; j++)
        {
            if (j == 0)
                hs += hb_in_f[i] * hb_coef[j];
            else
                hs += hb_reg[j - 1] * hb_coef[j];
        }
        for (int j = HB_FO - 1; j > 0; j--)
        {
            hb_reg[j] = hb_reg[j - 1];
        }
        hb_reg[0] = hb_in_f[i];
        hb_out_f[i] = hs;
    }
}

static float mixer(float *in_r, float *in_i, int16_t *output_data, int len, float phase_acc)
{
    float sin_data, cos_data, out_f;
    int phase_index;
    phase_index = (int)floor(phase_acc + 0.5);
    phase_index %= 1024;
    mixer_lut(&cos_data, &sin_data, phase_index);
    for (int i = 0; i < len; i++)
    {
        out_f = in_r[i] * cos_data - in_i[i] * sin_data;
        output_data[i] = (int16_t)floor(out_f * 32768 + 0.5);
        phase_acc += (float)DELTA_PH;
        if (phase_acc >= 1024)
            phase_acc -= 1024;
        phase_index = (int)floor(phase_acc + 0.5);
        phase_index %= 1024;
        mixer_lut(&cos_data, &sin_data, phase_index);
    }
    return phase_acc;
}

float freq_shift(int16_t *data_in, int16_t *data_out, int len, float phase_acc, int first)
{
    for (int i = 0; i < len; i++)
    {
        hb_in_f[i] = (float)data_in[i];
        hb_in_f[i] /= 32768.0;
    }
    hb_filter(hb_in_f, hb_out_f, hb_reg, len);
    memcpy(mix_in_r, hb_buf, sizeof(hb_buf));
    memcpy(&mix_in_r[HB_GRD], hb_in_f, sizeof(hb_in_f) - sizeof(hb_buf));
    memcpy(hb_buf, &hb_in_f[len - HB_GRD], sizeof(hb_buf));
    if (first)
        memset(hb_out_f, 0, sizeof(hb_buf));
    phase_acc = mixer(mix_in_r, mix_in_i, data_out, len, phase_acc);
    return phase_acc;
}

// Test code in PC.
#if 0
int16_t input_data[FRAME_LENGTH];
int16_t output_data[FRAME_LENGTH];
int main(int argc, char *argv[])
{
    FILE *fp = fopen(argv[1], "rb");
    FILE *fp_out = fopen(argv[2], "wb+");
    FILE *fp_hb = fopen(argv[3], "wb+");
    int len;

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
    len = fread(input_data, sizeof(int16_t), FRAME_LENGTH, fp);
    int first = 1;
    while (len == FRAME_LENGTH)
    {
        phase_acc = freq_shift(input_data, output_data, len, phase_acc, first);
        first = 0;
        fwrite(output_data, sizeof(int16_t), FRAME_LENGTH, fp_out);
        len = fread(input_data, sizeof(int16_t), FRAME_LENGTH, fp);
    }
    fclose(fp_hb);
    fclose(fp_out);
    fclose(fp);
    printf("Finished\n");
}
#endif