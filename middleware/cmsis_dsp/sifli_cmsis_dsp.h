/**
  ******************************************************************************
  * @file   sifli_cmsis_dsp.h
  * @author Sifli software development team
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

#ifndef SIFLI_CMSIS_DSP_H
#define SIFLI_CMSIS_DSP_H

#include "arm_math.h"

#define FACC_MAX_BLOCK_SIZE 256

/**
 * @brief Convolution of Q7 sequences.
 * @param[in]  pSrcA    points to the first input sequence.
 * @param[in]  srcALen  length of the first input sequence.
 * @param[in]  pSrcB    points to the second input sequence.
 * @param[in]  srcBLen  length of the second input sequence.
 * @param[out] pDst     points to the block of output data  Length srcALen+srcBLen-1.
 */
void arm_conv_q7_facc(
    const q7_t *pSrcA,
    uint32_t srcALen,
    const q7_t *pSrcB,
    uint32_t srcBLen,
    q7_t *pDst);


/**
 * @brief Convolution of Q15 sequences.
 * @param[in]  pSrcA    points to the first input sequence.
 * @param[in]  srcALen  length of the first input sequence.
 * @param[in]  pSrcB    points to the second input sequence.
 * @param[in]  srcBLen  length of the second input sequence.
 * @param[out] pDst     points to the location where the output result is written.  Length srcALen+srcBLen-1.
 */
void arm_conv_q15_facc(
    const q15_t *pSrcA,
    uint32_t srcALen,
    const q15_t *pSrcB,
    uint32_t srcBLen,
    q15_t *pDst);



/**
* @brief  Initialization function for the Q7 FIR filter.
* @param[in,out] S          points to an instance of the Q7 FIR structure.
* @param[in]     numTaps    Number of filter coefficients in the filter.
* @param[in]     pCoeffs    points to the filter coefficients.
* @param[in]     pState     points to the state buffer.
* @param[in]     blockSize  number of samples that are processed.
*/
void arm_fir_init_q7_facc(
    arm_fir_instance_q7 *S,
    uint16_t numTaps,
    const q7_t *pCoeffs,
    q7_t *pState,
    uint32_t blockSize);


/**
* @brief Processing function for the Q7 FIR filter.
* @param[in]  S          points to an instance of the Q7 FIR filter structure.
* @param[in]  pSrc       points to the block of input data.
* @param[out] pDst       points to the block of output data.
* @param[in]  blockSize  number of samples to process. Must be 32-bit aligned if not the last block
*/
void arm_fir_q7_facc(
    const arm_fir_instance_q7 *S,
    const q7_t *pSrc,
    q7_t *pDst,
    uint32_t blockSize);


/**
* @brief  Initialization function for the Q15 FIR filter.
* @param[in,out] S          points to an instance of the Q15 FIR filter structure.
* @param[in]     numTaps    Number of filter coefficients in the filter. Must be even and greater than or equal to 4.
* @param[in]     pCoeffs    points to the filter coefficients.
* @param[in]     pState     points to the state buffer.
* @param[in]     blockSize  number of samples that are processed at a time. Must be 32-bit aligned if not the last block
* @return     The function returns either
* <code>ARM_MATH_SUCCESS</code> if initialization was successful or
* <code>ARM_MATH_ARGUMENT_ERROR</code> if <code>numTaps</code> is not a supported value.
*/
arm_status arm_fir_init_q15_facc(
    arm_fir_instance_q15 *S,
    uint16_t numTaps,
    const q15_t *pCoeffs,
    q15_t *pState,
    uint32_t blockSize);



/**
* @brief Processing function for the Q15 FIR filter.
* @param[in]  S          points to an instance of the Q15 FIR structure.
* @param[in]  pSrc       points to the block of input data.
* @param[out] pDst       points to the block of output data.
* @param[in]  blockSize  number of samples to process.
*/
void arm_fir_q15_facc(
    const arm_fir_instance_q15 *S,
    const q15_t *pSrc,
    q15_t *pDst,
    uint32_t blockSize);




/**
 * @brief Instance structure for the Q7 IIR filter.
 *     b[0] + b[1]z-1 + ... b[p-1]z-(p-1)
 *     ----------------------------------
 *     a[0] + a[1]z-1 + ... a[m-1]z-(m-1)
 */
typedef struct
{
    uint16_t m;              /**< IIR vector a step */
    uint16_t p;              /**< IIR vector b step */
    q7_t *pState;          /**< points to the state variable array. FACC requre (64+32)x32/8+4=388 bytes. */
    const q7_t *pCoeffsA;          /**< points to the coefficient array A.*/
    const q7_t *pCoeffsB;          /**< points to the coefficient array B*/
} arm_iir_instance_q7;

/**
 * @brief Instance structure for the Q15 IIR filter.
 *     b[0] + b[1]z-1 + ... b[p-1]z-(p-1)
 *     ----------------------------------
 *     1 + a[0]z-1 + ... a[m-1]z-(m-1)
 */
typedef struct
{
    uint16_t m;              /**< IIR vector a step, including first constant 1 */
    uint16_t p;              /**< IIR vector b step */
    q15_t *pState;           /**< points to the state variable array. FACC requre (64+32)x32/8+4=388 bytes. */
    const q15_t *pCoeffsA;          /**< points to the coefficient array A.*/
    const q15_t *pCoeffsB;          /**< points to the coefficient array B*/
} arm_iir_instance_q15;

/**
* @brief  Initialization function for the Q7 IIR filter.
* @param[in,out] S          points to an instance of the Q15 FIR filter structure.
* @param[in]     p          IIR vector b step .
* @param[in]     pCoeffsB   points to the filter coefficients B array
* @param[in]     m          IIR vector a step .
* @param[in]     pCoeffsA   points to the filter coefficients A array
* @param[in]     pState     points to the state buffer.
*/
void arm_iir_init_q7_facc(
    arm_iir_instance_q7 *S,
    uint16_t p,
    const q7_t *pCoeffsB,
    uint16_t m,
    const q7_t *pCoeffsA,
    q7_t *pState
);

/**
* @brief  Initialization function for the Q15 IIR filter.
* @param[in,out] S          points to an instance of the Q15 FIR filter structure.
* @param[in]     p          IIR vector b step .
* @param[in]     pCoeffsB   points to the filter coefficients B array
* @param[in]     m          IIR vector a step .
* @param[in]     pCoeffsA   points to the filter coefficients A array
* @param[in]     pState     points to the state buffer.
*/
void arm_iir_init_q15_facc(
    arm_iir_instance_q15 *S,
    uint16_t p,
    const q15_t *pCoeffsB,
    uint16_t m,
    const q15_t *pCoeffsA,
    q15_t *pState
);


/**
* @brief Processing function for the Q7 IIR filter.
* @param[in]  S          points to an instance of the Q15 IIR structure.
* @param[in]  pSrc       points to the block of input data.
* @param[out] pDst       points to the block of output data.
* @param[in]  blockSize  number of samples to process. Must be 32-bit aligned if not the last block
*/
void arm_iir_q7_facc(
    const arm_iir_instance_q7 *S,
    const q7_t *pSrc,
    q7_t *pDst,
    uint32_t blockSize);


/**
* @brief Processing function for the Q15 IIR filter.
* @param[in]  S          points to an instance of the Q15 IIR structure.
* @param[in]  pSrc       points to the block of input data.
* @param[out] pDst       points to the block of output data.
* @param[in]  blockSize  number of samples to process. Must be 32-bit aligned if not the last block
*/
void arm_iir_q15_facc(
    const arm_iir_instance_q15 *S,
    const q15_t *pSrc,
    q15_t *pDst,
    uint32_t blockSize);

/**
* @brief  Initialization Filter acceleration.
*/
void arm_dsp_facc_init(void);


#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
