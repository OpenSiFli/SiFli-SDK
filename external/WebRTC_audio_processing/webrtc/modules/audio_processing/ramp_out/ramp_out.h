#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_RAMP_OUT_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_RAMP_OUT_H_

#include "webrtc/typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RAMP_OUT_START_MUTE 0 //0ms start mute
#define RAMP_OUT_INTERVAL 0  //(n+1)*10ms=(0+1)*10ms=10ms
#define RAMP_OUT_GAIN_MIN 36 //-36+n=-36+36=0dB
#define RAMP_OUT_GAIN_MAX 36 //0dB

typedef struct RampOutHandle_
{
	int bypass;
	int rampInterval;
	int rampCnt;
	int frameCnt;
	int rampGain;
} RampOutHandle;


/*
 * This function creates an instance of RAMP OUT
 */
void* RampOut_Create();

/*
 * This function frees the dynamic memory of a specified RAMP OUT
 * instance.
 *
 * Input:
 *      - RampOutInst       : Pointer to RAMP OUT instance that should be freed
 */
void RampOut_Free(void* RampOutInst);

/*
 * This function initializes a RAMP OUT instance
 *
 * Input:
 *      - RampOutInst    : Instance that should be initialized
 *
 * Output:
 *      - RampOutInst       : Initialized instance
 *
 * Return value         :  0 - Ok
 *                        -1 - Error
 */
int RampOut_Init(void* RampOutInst, int bypass);

/*
 * This functions does RAMP OUT for the inserted speech frame. The
 * input and output signals should always be 10ms (80 or 160 samples).
 *
 * Input
 *      - RampOutInst    : RAMP OUT instance. Needs to be initiated before call.
 *      - input   : Pointer to speech frame buffer
 *      - nrOfSamples   : Number of samples to be processed
 *
 * Output:
 *      - RampOutInst       : Updated RAMP OUT  instance
 *      - output      : Pointer to output frame
 * Return:
 *        0 --- result in output
 *       -1 ----do nothing, result in input
 */

int RampOut_Process(void* RampOutInst,
	const int16_t* input,
	int16_t* output,
	int16_t nrOfSamples);

#ifdef __cplusplus
}
#endif

#endif  // WEBRTC_MODULES_AUDIO_PROCESSING_RAMP_OUT_H_
