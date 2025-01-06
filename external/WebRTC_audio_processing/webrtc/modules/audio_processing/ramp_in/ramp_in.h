#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_RAMP_IN_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_RAMP_IN_H_

#include "webrtc/typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RAMP_IN_START_MUTE 20 //200ms start mute
#define RAMP_IN_INTERVAL 2 //10ms
#define RAMP_IN_GAIN_MIN 6 //2^-6
#define RAMP_IN_GAIN_MAX 0 //2^0


typedef struct RampInHandle_
{
	int bypass;
	int rampInterval;
	int rampCnt;
	int frameCnt;
	int rampGain;
} RampInHandle;


/*
 * This function creates an instance of RAMP IN
 */
void* RampIn_Create();

/*
 * This function frees the dynamic memory of a specified RAMP IN
 * instance.
 *
 * Input:
 *      - RampInInst       : Pointer to RAMP IN instance that should be freed
 */
void RampIn_Free(void* RampInInst);

/*
 * This function initializes a RAMP IN instance
 *
 * Input:
 *      - RampInInst    : Instance that should be initialized
 *      - fs            : sampling frequency
 *
 * Output:
 *      - RampInInst       : Initialized instance
 *
 * Return value         :  0 - Ok
 *                        -1 - Error
 */
int RampIn_Init(void* RampInInst, int bypass);

/*
 * This functions does RAMP IN for the inserted speech frame. The
 * input and output signals should always be 10ms (80 or 160 samples).
 *
 * Input
 *      - RampInInst    : RAMP IN instance. Needs to be initiated before call.
 *      - input   : Pointer to speech frame buffer
 *      - nrOfSamples   : Number of samples to be processed
 *
 * Output:
 *      - RampInInst       : Updated RAMP IN  instance
 *      - output      : Pointer to output frame
 * Return:
 *        0 --- result in output
 *       -1 ----do nothing, result in input
 */

int RampIn_Process(void* RampInInst,
	const int16_t* input,
	int16_t* output,
	int16_t nrOfSamples);

#ifdef __cplusplus
}
#endif

#endif  // WEBRTC_MODULES_AUDIO_PROCESSING_RAMP_IN_H_
