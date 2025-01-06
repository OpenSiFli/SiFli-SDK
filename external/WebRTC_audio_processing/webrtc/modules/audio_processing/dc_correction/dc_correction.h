#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_DC_CORRECTION_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_DC_CORRECTION_H_

#include "webrtc/typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct DcCorrectionHandle_
{
	int bypass;
	short DcBuffer[160];
	int frameCnt;
	int32_t dc_sum;
} DcCorrectionHandle;


/*
 * This function creates an instance of DC Correction.
 */
void* DcCorrection_Create();

/*
 * This function frees the dynamic memory of a specified DC Correction
 * instance.
 *
 * Input:
 *      - DcInst       : Pointer to DC correction instance that should be freed
 */
void DcCorrection_Free(void* dcInst);

/*
 * This function initializes a NS instance
 *
 * Input:
 *      - nsxInst       : Instance that should be initialized
 *      - fs            : sampling frequency
 *
 * Output:
 *      - nsxInst       : Initialized instance
 *
 * Return value         :  0 - Ok
 *                        -1 - Error
 */
int DcCorrection_Init(void* dcInst, int bypass);

/*
 * This functions does dc correction for the inserted speech frame. The
 * input and output signals should always be 10ms (80 or 160 samples).
 *
 * Input
 *      - dcInst        : dc correction instance. Needs to be initiated before call.
 *      - speechFrame   : Pointer to speech frame buffer for each band
 *      - num_bands     : Number of bands
 *
 * Output:
 *      - nsxInst       : Updated NSx instance
 *      - outFrame      : Pointer to output frame for each band
 */

void DcCorrection_Process(void* dcInst,
	const int16_t* input,
	int16_t* output,	
	int16_t nrOfSamples);

#ifdef __cplusplus
}
#endif

#endif  // WEBRTC_MODULES_AUDIO_PROCESSING_DC_CORRECTION_H_
