/**
  ******************************************************************************
  * @file   sifli_cmsis_dsp.c
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

#include "rtthread.h"
#include "stdio.h"
#include "sifli_cmsis_dsp.h"
#include "bf0_hal.h"

#ifdef HAL_FACC_MODULE_ENABLED
static FACC_HandleTypeDef hfacc;

void arm_dsp_facc_init(void)
{
    memset(&hfacc, 0, sizeof(FACC_HandleTypeDef));
#ifdef BF0_LCPU
    hfacc.Instance = hwp_facc2;
    HAL_RCC_EnableModule(RCC_MOD_FACC2);
#else
    hfacc.Instance = hwp_facc1;
    HAL_RCC_EnableModule(RCC_MOD_FACC1);
#endif
    HAL_FACC_Init(&hfacc);
}


static void arm_conv_facc(
    uint8_t *pSrcA,
    uint32_t srcALen,
    uint8_t *pSrcB,
    uint32_t srcBLen,
    uint8_t *pDst,
    uint8_t is_8bit)
{
    FACC_ConfigTypeDef  config;
    uint8_t *temp_src;
    uint32_t temp_len;
    uint32_t max_size;

    if (srcALen == 0 || srcBLen == 0)
        return;

    memset(&config, 0, sizeof(FACC_ConfigTypeDef));
    config.conv_sel = 1;
    if (is_8bit)
    {
        config.fp_sel = 1;
        max_size = FACC_MAX_FIFO_SIZE;
        if (srcBLen > max_size && srcALen > max_size)
            return arm_conv_q7((const q7_t *)pSrcA, srcALen, (const q7_t *)pSrcB, srcBLen, (q7_t *)pDst);
    }
    else
    {
        max_size = (FACC_MAX_FIFO_SIZE >> 1);
        if (srcBLen > max_size && srcALen > max_size)
            return arm_conv_q15((const q15_t *)pSrcA, srcALen, (const q15_t *)pSrcB, srcBLen, (q15_t *)pDst);
    }

    if (HAL_FACC_Reset(&hfacc) != HAL_OK)
        goto __EXIT;

    if (srcBLen > max_size)
    {
        temp_len = srcALen;
        srcALen = srcBLen;
        srcBLen = temp_len;
        temp_src = pSrcA;
        pSrcA = pSrcB;
        pSrcB = temp_src;
    }

    if (is_8bit == 0)
    {
        srcALen <<= 1;
        srcBLen <<= 1;
    }
    HAL_FACC_Config(&hfacc, &config);
    if (HAL_FACC_SetConvKernel(&hfacc, (uint8_t *)pSrcB, srcBLen) != HAL_OK)
        goto __EXIT;


    while (srcALen > FACC_MAX_BLOCK_SIZE)
    {
        if (HAL_FACC_Config(&hfacc, &config) != HAL_OK ||
                HAL_FACC_Start(&hfacc, (uint8_t *)pSrcA, (uint8_t *)pDst, FACC_MAX_BLOCK_SIZE) != HAL_OK)
            goto __EXIT;
        pSrcA += FACC_MAX_BLOCK_SIZE;
        pDst += FACC_MAX_BLOCK_SIZE;
        srcALen -= FACC_MAX_BLOCK_SIZE;
    }
    if (srcALen > 0)
    {
        config.last_sel = 1;
        if (HAL_FACC_Config(&hfacc, &config) != HAL_OK ||
                HAL_FACC_Start(&hfacc, (uint8_t *)pSrcA, (uint8_t *)pDst, srcALen) != HAL_OK)
            goto __EXIT;
    }

__EXIT:
    return;
}

void arm_conv_q7_facc(
    const q7_t *pSrcA,
    uint32_t srcALen,
    const q7_t *pSrcB,
    uint32_t srcBLen,
    q7_t *pDst)
{
    arm_conv_facc((uint8_t *)pSrcA, srcALen, (uint8_t *)pSrcB, srcBLen, (uint8_t *)pDst, 1);

}

void arm_conv_q15_facc(
    const q15_t *pSrcA,
    uint32_t srcALen,
    const q15_t *pSrcB,
    uint32_t srcBLen,
    q15_t *pDst)
{
    arm_conv_facc((uint8_t *)pSrcA, srcALen, (uint8_t *)pSrcB, srcBLen, (uint8_t *)pDst, 0);
}


void arm_fir_init_q7_facc(
    arm_fir_instance_q7 *S,
    uint16_t numTaps,
    const q7_t *pCoeffs,
    q7_t *pState,
    uint32_t blockSize)
{
    arm_fir_init_q7(S, numTaps, pCoeffs, pState, blockSize);
    if (S->numTaps <= FACC_MAX_FIFO_SIZE)
    {
        FACC_ConfigTypeDef  config;
        int i;
        HAL_FACC_Reset(&hfacc);

        memset(&config, 0, sizeof(FACC_ConfigTypeDef));
        config.fp_sel = 1;
        HAL_FACC_Config(&hfacc, &config);
    }

__EXIT:
    return;

}

arm_status arm_fir_init_q15_facc(
    arm_fir_instance_q15 *S,
    uint16_t numTaps,
    const q15_t *pCoeffs,
    q15_t *pState,
    uint32_t blockSize)
{
    arm_status r;

    r = arm_fir_init_q15(S, numTaps, pCoeffs, pState, blockSize);
    if ((S->numTaps << 1) <= FACC_MAX_FIFO_SIZE)
    {
        FACC_ConfigTypeDef  config;
        HAL_FACC_Reset(&hfacc);

        memset(&config, 0, sizeof(FACC_ConfigTypeDef));
        HAL_FACC_Config(&hfacc, &config);
    }
    return r;
}

static void arm_fir_facc(
    const arm_fir_instance_q7 *S,           // Only support 0-states acceleration.
    uint8_t *pSrc,
    uint8_t *pDst,
    uint32_t blockSize,
    uint8_t is_8bit
)
{
    FACC_ConfigTypeDef  config;
    uint32_t max_size;

    if (blockSize == 0)
        return;

    memset(&config, 0, sizeof(FACC_ConfigTypeDef));

    if (is_8bit)
    {
        config.fp_sel = 1;
        if (S->numTaps > FACC_MAX_FIFO_SIZE)
            return arm_fir_q7(S, (const q7_t *)pSrc, (q7_t *)pDst, blockSize);
    }
    else
    {
        if (S->numTaps > (FACC_MAX_FIFO_SIZE >> 1))
            return arm_fir_q15((const arm_fir_instance_q15 *)S, (const q15_t *)pSrc, (q15_t *)pDst, blockSize);
        blockSize <<= 1;
    }
    HAL_FACC_Config(&hfacc, &config);
    HAL_FACC_Buffer_Enable(&hfacc, (uint8_t *)S->pState);
    HAL_FACC_Start(&hfacc, (uint8_t *)pSrc, (uint8_t *)pDst, blockSize);
}

void arm_fir_q7_facc(
    const arm_fir_instance_q7 *S,           // Only support 0-states acceleration.
    const q7_t *pSrc,
    q7_t *pDst,
    uint32_t blockSize)
{
    HAL_FACC_SetCoeffFirReverse(&hfacc, (uint8_t *)S->pCoeffs, S->numTaps);
    return arm_fir_facc(S, (uint8_t *)pSrc, (uint8_t *)pDst, blockSize, 1);
}

void arm_fir_q15_facc(
    const arm_fir_instance_q15 *S,
    const q15_t *pSrc,
    q15_t *pDst,
    uint32_t blockSize)
{
    HAL_FACC_SetCoeffFirReverse(&hfacc, (uint8_t *)S->pCoeffs, (S->numTaps << 1));
    return arm_fir_facc((const arm_fir_instance_q7 *)S, (uint8_t *)pSrc, (uint8_t *)pDst, blockSize, 0);
}



void arm_iir_facc(
    const arm_iir_instance_q7 *S,
    const q7_t *pSrc,
    q7_t *pDst,
    uint32_t blockSize,
    uint8_t is_8bit)
{
    FACC_ConfigTypeDef  config;
    uint32_t max_size;

    if (blockSize == 0)
        return;

    if (S->m + S->p > FACC_MAX_FIFO_SIZE)
        return;

    memset(&config, 0, sizeof(FACC_ConfigTypeDef));
    config.mod_sel = 1;
    if (is_8bit)
    {
        config.fp_sel = 1;
        if (S->m + S->p > FACC_MAX_FIFO_SIZE)
            return;
    }
    else
    {
        if (S->m + S->p > (FACC_MAX_FIFO_SIZE >> 1))
            return;
        blockSize <<= 1;
    }

    HAL_FACC_Config(&hfacc, &config);
    HAL_FACC_Buffer_Enable(&hfacc, (uint8_t *)S->pState);
    HAL_FACC_Start(&hfacc, (uint8_t *)pSrc, (uint8_t *)pDst, blockSize);
    //HAL_DBG_print_data((char *)S->pState, 0, FACC_IIR_STATE_SIZE);
    return;
}


void arm_iir_q7_facc(
    const arm_iir_instance_q7 *S,
    const q7_t *pSrc,
    q7_t *pDst,
    uint32_t blockSize)
{
    HAL_FACC_SetCoeff(&hfacc, (uint8_t *) S->pCoeffsB, S->p, (uint8_t *)S->pCoeffsA, S->m, 0);
    return arm_iir_facc(S, pSrc, pDst, blockSize, 1);
}

void arm_iir_q15_facc(
    const arm_iir_instance_q15 *S,
    const q15_t *pSrc,
    q15_t *pDst,
    uint32_t blockSize)
{
    HAL_FACC_SetCoeff(&hfacc, (uint8_t *)S->pCoeffsB, S->p << 1, (uint8_t *)S->pCoeffsA, S->m << 1, 0);
    return arm_iir_facc((const arm_iir_instance_q7 *)S, (const q7_t *)pSrc, (q7_t *)pDst, blockSize, 0);
}

void arm_iir_init_q7_facc(
    arm_iir_instance_q7 *S,
    uint16_t p,
    const q7_t *pCoeffsB,
    uint16_t m,
    const q7_t *pCoeffsA,
    q7_t *pState
)
{
    S->p = p;
    S->m = m;
    S->pCoeffsB = pCoeffsB;
    S->pCoeffsA = pCoeffsA;
    S->pState = pState;

    if (S->p + S->m <= FACC_MAX_FIFO_SIZE)
    {
        FACC_ConfigTypeDef  config;
        int i;
        HAL_FACC_Reset(&hfacc);

        memset(S->pState, 0, FACC_IIR_STATE_SIZE);
        memset(&config, 0, sizeof(FACC_ConfigTypeDef));
        config.fp_sel = 1;
        config.mod_sel = 1;
        HAL_FACC_Config(&hfacc, &config);
    }
}

void arm_iir_init_q15_facc(
    arm_iir_instance_q15 *S,
    uint16_t p,
    const q15_t *pCoeffsB,
    uint16_t m,
    const q15_t *pCoeffsA,
    q15_t *pState
)
{
    S->p = p;
    S->m = m;
    S->pCoeffsB = pCoeffsB;
    S->pCoeffsA = pCoeffsA;
    S->pState = pState;
    if (S->p + S->m <= (FACC_MAX_FIFO_SIZE >> 1))
    {
        FACC_ConfigTypeDef  config;
        int i;
        HAL_FACC_Reset(&hfacc);

        memset(S->pState, 0, FACC_IIR_STATE_SIZE);
        memset(&config, 0, sizeof(FACC_ConfigTypeDef));
        config.mod_sel = 1;
        HAL_FACC_Config(&hfacc, &config);
    }
}
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
