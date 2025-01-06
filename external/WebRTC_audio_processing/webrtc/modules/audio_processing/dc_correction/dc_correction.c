#include "webrtc/modules/audio_processing/dc_correction/dc_correction.h"

#include <assert.h>
#include <stdlib.h>
#include <rtthread.h>
#include "webrtc_mem.h"

#undef assert
#define assert RT_ASSERT

#ifdef __cplusplus
extern "C" {
#endif

/*
 * This function creates an instance of DC Correction.
 */
void *DcCorrection_Create()
{
    DcCorrectionHandle *self = malloc(sizeof(DcCorrectionHandle));
    assert(self);
    return self;
}

void DcCorrection_Free(void *dcInst)
{
    DcCorrectionHandle *sst;
    sst = (DcCorrectionHandle *)dcInst;
    free(sst);
}

int DcCorrection_Init(void *dcInst, int bypass)
{
    DcCorrectionHandle *sst;
    if (!dcInst)
    {
        return 1;
    }
    sst = (DcCorrectionHandle *)dcInst;
    sst->bypass = bypass;
    sst->frameCnt = 0;
    sst->dc_sum = 0;

    return 1;
}

void DcCorrection_Process(void *dcInst, const int16_t *input, int16_t *output, int16_t nrOfSamples)
{

    DcCorrectionHandle *sst;
    sst = (DcCorrectionHandle *)dcInst;

    int n;

    int16_t dc_val;

    if (!dcInst)
    {
        return;
    }
    if (sst->bypass == 1)
    {
        for (n = 0; n < nrOfSamples; n++)
        {
            output[n] = input[n];
        }
    }
    else
    {

        if (sst->frameCnt < 20)
        {
            for (n = 0; n < nrOfSamples; n++)
            {
                output[n] = 0;
                if (sst->frameCnt == 19)
                {
                    sst->DcBuffer[n] = input[n];
                    sst->dc_sum = sst->dc_sum + input[n];
                }
            }

            sst->frameCnt = sst->frameCnt + 1;
        }
        else
        {
            for (n = 0; n < nrOfSamples; n++)
            {

                dc_val = (int16_t)(sst->dc_sum / nrOfSamples);
                if (n < nrOfSamples / 2)
                {
                    output[n] = sst->DcBuffer[n + nrOfSamples / 2] - dc_val;
                }
                else
                {
                    output[n] = input[n - nrOfSamples / 2] - dc_val;
                }
                sst->dc_sum = sst->dc_sum + input[n] - sst->DcBuffer[n];
                sst->DcBuffer[n] = input[n];
            }
        }

    }

    return;
}


#ifdef __cplusplus
}
#endif
