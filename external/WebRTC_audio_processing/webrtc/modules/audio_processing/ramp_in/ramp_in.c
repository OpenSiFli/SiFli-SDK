#include "webrtc/modules/audio_processing/ramp_in/ramp_in.h"

#include <assert.h>
#include <stdlib.h>
#include <rtthread.h>
#define DBG_TAG         "audio"
#include "log.h"
#include "webrtc_mem.h"

#ifdef __cplusplus
extern "C" {
#endif

void *RampIn_Create()
{
    RampInHandle *self = malloc(sizeof(RampInHandle));
    RT_ASSERT(self);
    return self;
}

void RampIn_Free(void *RampInInst)
{
    RampInHandle *sst;
    sst = (RampInHandle *)RampInInst;
    free(sst);
}

int g_rampin_interval = RAMP_IN_INTERVAL;
int g_rampin_mute = RAMP_IN_START_MUTE;
int RampIn_Init(void *RampInInst, int bypass)
{
    RampInHandle *sst;
    sst = (RampInHandle *)RampInInst;
    if (!RampInInst)
        return 1;
    sst->bypass = bypass;
    sst->rampInterval = g_rampin_interval;
    sst->frameCnt = 0;
    sst->rampCnt = 0;
    sst->rampGain = RAMP_IN_GAIN_MIN;
    return 1;
}


int RampIn_Process(void *RampInInst,
                    const int16_t *input,
                    int16_t *output,
                    int16_t nrOfSamples)
{
    int ret = 0;
    RampInHandle *sst;
    sst = (RampInHandle *)RampInInst;

    int n;
    if (!RampInInst)
        return -1;
    if (sst->frameCnt < g_rampin_mute)
    {
        for (n = 0; n < nrOfSamples; n++)
            output[n] = 0;
        sst->frameCnt = sst->frameCnt + 1;
    }
    else if (sst->rampGain > RAMP_IN_GAIN_MAX)
    {
        //search ramp point
        output[0] = ((int32_t)input[0]) >> sst->rampGain;
        for (n = 1; n < nrOfSamples; n++)
        {
            if ((sst->rampCnt > sst->rampInterval) && ((int32_t)input[n] * (int32_t)input[n - 1] <= 0))
            {
                sst->rampCnt = 0;
                sst->rampGain = sst->rampGain - 1;
            }
            output[n] = ((int32_t)input[n]) >> sst->rampGain;
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

