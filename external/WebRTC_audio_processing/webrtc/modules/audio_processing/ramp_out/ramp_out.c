#include "webrtc/modules/audio_processing/ramp_out/ramp_out.h"

#include <assert.h>
#include <stdlib.h>
#include <rtthread.h>
#include "webrtc_mem.h"

#ifdef __cplusplus
extern "C" {
#endif

static const int16_t ramp_gain_table[37] = {519,    583,    654,    734,    823,    924,
                                            1036,   1163,   1305,   1464,   1642,   1843,
                                            2068,   2320,   2603,   2920,   3277,   3677,
                                            4125,   4629,   5193,   5827,   6538,   7336,
                                            8231,   9235,   10362,  11627,  13045,  14637,
                                            16423,  18427,  20675,  23198,  26029,  29205, 32767
                                           };

void *RampOut_Create()
{
    RampOutHandle *self = malloc(sizeof(RampOutHandle));
    RT_ASSERT(self);
    return self;
}

void RampOut_Free(void *RampOutInst)
{
    RampOutHandle *sst;
    sst = (RampOutHandle *)RampOutInst;
    free(sst);
}

int g_rampout_interval = RAMP_OUT_INTERVAL;
int g_rampout_mute = RAMP_OUT_START_MUTE;
int g_rampout_gain_min = RAMP_OUT_GAIN_MIN;
int RampOut_Init(void *RampOutInst, int bypass)
{
    RampOutHandle *sst;
    sst = (RampOutHandle *)RampOutInst;

    sst->bypass = bypass;
    sst->rampInterval = g_rampout_interval;
    sst->frameCnt = 0;
    sst->rampCnt = 0;
    sst->rampGain = g_rampout_gain_min;

    return 1;
}

int RampOut_Process(void *RampOutInst,
                     const int16_t *input,
                     int16_t *output,
                     int16_t nrOfSamples)
{
    int ret = 0;
    RampOutHandle *sst;
    sst = (RampOutHandle *)RampOutInst;
    if (!RampOutInst)
        return -1;
    int n;

    if (sst->frameCnt < g_rampout_mute)
    {
        for (n = 0; n < nrOfSamples; n++)
            output[n] = 0;

        sst->frameCnt = sst->frameCnt + 1;
    }
    else if (sst->rampGain < RAMP_OUT_GAIN_MAX)
    {
        //search ramp point
        output[0] = (int16_t)(((int32_t)((int32_t)input[0] * ramp_gain_table[sst->rampGain])) >> 15);
        for (n = 1; n < nrOfSamples; n++)
        {
            if ((sst->rampCnt > sst->rampInterval) && ((int32_t)input[n] * (int32_t)input[n - 1] <= 0))
            {
                sst->rampCnt = 0;
                sst->rampGain = sst->rampGain + 1;
            }
            output[n] = (int16_t)(((int32_t)((int32_t)input[n] * ramp_gain_table[sst->rampGain])) >> 15);
        }
        sst->rampCnt = sst->rampCnt + 1;
    }
    else
    {
        for (n = 0; n < nrOfSamples; n++)
            output[n] = input[n];
    }
    return ret;
}

#ifdef __cplusplus
}
#endif

