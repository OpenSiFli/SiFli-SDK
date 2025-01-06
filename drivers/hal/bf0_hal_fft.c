/**
  ******************************************************************************
  * @file   bf0_hal_fft.c
  * @author Sifli software development team
  * @brief   FFT HAL module driver.
  *          This file provides firmware functions to manage the following
  ******************************************************************************
*/
/**
 *
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

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup FFT FFT
  * @brief FFT HAL module driver
  * @{
  */

#if defined(HAL_FFT_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#ifdef hwp_fft2
    #define IS_VALID_INSTANCE(inst)  ((inst == hwp_fft1) || (inst == hwp_fft2))
#else
    #define IS_VALID_INSTANCE(inst)  (inst == hwp_fft1)
#endif

static uint32_t FFT_CalcDataSize(FFT_BitwidthTypeDef bitwidth, FFT_FFTLengthTypeDef fft_length,
                                 uint8_t rfft_flag, bool output)
{
    uint32_t size;

    size = (1UL << bitwidth);
    size *= (1UL << (fft_length + 4));
    if (!rfft_flag || output)
    {
        size *= 2;
    }

    return size;
}

#ifdef SF32LB56X
static uint32_t DCT_CalcDataSize(FFT_BitwidthTypeDef bitwidth, FFT_DCTLengthTypeDef dct_length,
                                 uint8_t rdct_flag)
{
    uint32_t size;

    size = (1UL << bitwidth);
    size *= (1UL << (dct_length + 3));
    if (!rdct_flag)
    {
        size *= 2;
    }

    return size;
}
#endif /* SF32LB56X */

static HAL_StatusTypeDef FFT_Config(FFT_TypeDef *fft, FFT_ConfigTypeDef *config)
{
    uint32_t cr;
    FFT_FFTLengthTypeDef max_fft_len;
    uint32_t data_size;

    if (!IS_VALID_INSTANCE(fft))
    {
        return HAL_ERROR;
    }
    if (hwp_fft1 == fft)
    {
        max_fft_len = FFT1_LEN_MAX;
    }
#ifdef hwp_fft2
    else if (hwp_fft2 == fft)
    {
        max_fft_len = FFT2_LEN_MAX;
    }
#endif
    else
    {
        return HAL_ERROR;
    }

    if (config->fft_length > max_fft_len)
    {
        return HAL_ERROR;
    }

    if (((uint32_t)config->input_data & 3) || ((uint32_t)config->output_data & 3))
    {
        return HAL_ERROR;
    }

    cr = MAKE_REG_VAL(config->bitwidth, FFT_CR_INPUT_BW_SEL_Msk, FFT_CR_INPUT_BW_SEL_Pos);
    cr |= MAKE_REG_VAL(config->bitwidth, FFT_CR_OUTPUT_BW_SEL_Msk, FFT_CR_OUTPUT_BW_SEL_Pos);
    cr |= MAKE_REG_VAL(config->fft_length, FFT_CR_FFT_LENGTH_Msk, FFT_CR_FFT_LENGTH_Pos);
    cr |= MAKE_REG_VAL(config->ifft_flag, FFT_CR_IFFT_MODE_Msk, FFT_CR_IFFT_MODE_Pos);
    cr |= MAKE_REG_VAL(config->rfft_flag, FFT_CR_RFFT_MODE_Msk, FFT_CR_RFFT_MODE_Pos);

#ifdef SF32LB56X
    if (config->win_enabled)
    {
        cr |= MAKE_REG_VAL(config->win_bitwidth, FFT_CR_FFT_WIN_BW_SEL_Msk, FFT_CR_FFT_WIN_BW_SEL_Pos);
        cr |= MAKE_REG_VAL(config->rwin_flag, FFT_CR_FFT_WIN_REAL_Msk, FFT_CR_FFT_WIN_REAL_Pos);
        cr |= FFT_CR_FFT_WIN_EN;
        fft->WAR = (uint32_t)config->win_data;
        if (IS_DCACHED_RAM((uint32_t)config->win_data))
        {
            data_size = FFT_CalcDataSize(config->win_bitwidth, config->fft_length, config->rwin_flag, false);
            mpu_dcache_clean(config->win_data, data_size);
        }
    }
#endif /* SF32LB56X */

    fft->CR = cr;
    fft->SAR = (uint32_t)config->input_data;
    fft->DAR = (uint32_t)config->output_data;

    if (IS_DCACHED_RAM((uint32_t)config->input_data))
    {
        data_size = FFT_CalcDataSize(config->bitwidth, config->fft_length, config->rfft_flag, false);
        mpu_dcache_clean(config->input_data, data_size);
    }

    if (IS_DCACHED_RAM((uint32_t)config->output_data))
    {
        data_size = FFT_CalcDataSize(config->bitwidth, config->fft_length, config->rfft_flag, true);
        mpu_dcache_invalidate(config->output_data, data_size);
    }

    return HAL_OK;
}

#ifdef SF32LB56X
static HAL_StatusTypeDef DCT_Config(FFT_TypeDef *fft, DCT_ConfigTypeDef *config)
{
    uint32_t cr;
    uint32_t data_size;

    if (!IS_VALID_INSTANCE(fft))
    {
        return HAL_ERROR;
    }

    if (config->dct_length >= DCT_LEN_TYPE_NUM)
    {
        return HAL_ERROR;
    }

    if (!config->rdct_flag)
    {
        if (config->idct_flag || (DCT_TYPE_4 != config->dct_type))
        {
            return HAL_ERROR;
        }
    }

    if (((uint32_t)config->input_data & 3) || ((uint32_t)config->output_data & 3))
    {
        return HAL_ERROR;
    }

    cr = MAKE_REG_VAL(config->bitwidth, FFT_CR_INPUT_BW_SEL_Msk, FFT_CR_INPUT_BW_SEL_Pos);
    cr |= MAKE_REG_VAL(config->bitwidth, FFT_CR_OUTPUT_BW_SEL_Msk, FFT_CR_OUTPUT_BW_SEL_Pos);
    cr |= MAKE_REG_VAL(config->dct_length, FFT_CR_FFT_LENGTH_Msk, FFT_CR_FFT_LENGTH_Pos);
    cr |= MAKE_REG_VAL(config->idct_flag, FFT_CR_DCT_MODE_Msk, FFT_CR_DCT_MODE_Pos);
    cr |= MAKE_REG_VAL(config->rdct_flag, FFT_CR_RFFT_MODE_Msk, FFT_CR_RFFT_MODE_Pos);
    cr |= MAKE_REG_VAL(config->dct_type, FFT_CR_DCT_TYPE_Msk, FFT_CR_DCT_TYPE_Pos);
    cr |= FFT_CR_FFT_DCT_MODE;

    fft->CR = cr;
    fft->SAR = (uint32_t)config->input_data;
    fft->DAR = (uint32_t)config->output_data;

    if (IS_DCACHED_RAM((uint32_t)config->input_data))
    {
        data_size = DCT_CalcDataSize(config->bitwidth, config->dct_length, config->rdct_flag);
        mpu_dcache_clean(config->input_data, data_size);
    }

    if (IS_DCACHED_RAM((uint32_t)config->output_data))
    {
        data_size = DCT_CalcDataSize(config->bitwidth, config->dct_length, config->rdct_flag);
        mpu_dcache_invalidate(config->output_data, data_size);
    }

    return HAL_OK;
}
#endif /* SF32LB56X */


__HAL_ROM_USED HAL_StatusTypeDef HAL_FFT_IRQHandler(FFT_HandleTypeDef *fft)
{
#ifdef SF32LB56X
    uint8_t step;
#endif

    if (!IS_VALID_INSTANCE(fft->Instance))
    {
        return HAL_ERROR;
    }

    fft->Instance->IRQ = FFT_IRQ_ICR | FFT_IRQ_IMR;

#ifdef SF32LB56X
    if (HAL_FFT_STATE_CDCT_REAL == fft->State)
    {
        /* start img part calculation */
        fft->State = HAL_FFT_STATE_BUSY;
        step = GET_REG_VAL(fft->Instance->CR, FFT_CR_INPUT_BW_SEL_Msk, FFT_CR_INPUT_BW_SEL_Pos) + 1;
        fft->Instance->SAR += step;
        fft->Instance->DAR += step;
        fft->Instance->IRQ &= ~FFT_IRQ_IMR;
        fft->Instance->CR |= FFT_CR_START;
        return HAL_OK;
    }
#endif /* SF32LB56X */
    fft->State = HAL_FFT_STATE_READY;

    if (fft->CpltCallback)
    {
        fft->CpltCallback(fft);
    }

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_FFT_Init(FFT_HandleTypeDef *fft)
{
    if (HAL_FFT_STATE_RESET != fft->State)
    {
        return HAL_ERROR;
    }
    if (!IS_VALID_INSTANCE(fft->Instance))
    {
        return HAL_ERROR;
    }

    fft->State = HAL_FFT_STATE_READY;
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FFT_DeInit(FFT_HandleTypeDef *fft)
{
    if (HAL_FFT_STATE_READY != fft->State)
    {
        return HAL_ERROR;
    }
    fft->State = HAL_FFT_STATE_RESET;
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FFT_StartFFT(FFT_HandleTypeDef *fft, FFT_ConfigTypeDef *config)
{
    HAL_StatusTypeDef status;

    if (HAL_FFT_STATE_READY != fft->State)
    {
        return HAL_BUSY;
    }

    fft->State = HAL_FFT_STATE_BUSY;


    status = FFT_Config(fft->Instance, config);

    if (HAL_OK != status)
    {
        goto __EXIT;
    }

    fft->Instance->IRQ |= FFT_IRQ_IMR;
    fft->Instance->CR |= FFT_CR_START;
    while (0 == (fft->Instance->IRQ & FFT_IRQ_IRSR))
    {
        __NOP();
        __NOP();
        __NOP();
    }
    fft->Instance->IRQ = FFT_IRQ_ICR;

__EXIT:
    fft->State = HAL_FFT_STATE_READY;

    return status;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FFT_StartFFT_IT(FFT_HandleTypeDef *fft, FFT_ConfigTypeDef *config)
{
    HAL_StatusTypeDef status;

    if (HAL_FFT_STATE_READY != fft->State)
    {
        return HAL_BUSY;
    }

    fft->State = HAL_FFT_STATE_BUSY;


    status = FFT_Config(fft->Instance, config);

    if (HAL_OK != status)
    {
        goto __EXIT;
    }

    fft->Instance->IRQ &= ~FFT_IRQ_IMR;
    fft->Instance->CR |= FFT_CR_START;

__EXIT:

    if (HAL_OK != status)
    {
        fft->State = HAL_FFT_STATE_READY;
    }

    return status;
}

#if defined(SF32LB56X) || defined(_SIFLI_DOXYGEN_)
__HAL_ROM_USED HAL_StatusTypeDef HAL_FFT_StartDCT(FFT_HandleTypeDef *fft, DCT_ConfigTypeDef *config)
{
    HAL_StatusTypeDef status;
    int32_t loop_cnt;
    uint32_t step;
    uint32_t i;

    if (HAL_FFT_STATE_READY != fft->State)
    {
        return HAL_BUSY;
    }

    fft->State = HAL_FFT_STATE_BUSY;

    if (config->rdct_flag)
    {
        loop_cnt = 1;
        step = 0;
    }
    else
    {
        loop_cnt = 2;
        if (FFT_BW_8BIT == config->bitwidth)
        {
            step = 1;
        }
        else if (FFT_BW_16BIT == config->bitwidth)
        {
            step = 2;
        }
        else
        {
            step = 4;
        }
    }

    for (i = 0; i < loop_cnt; i++)
    {

        status = DCT_Config(fft->Instance, config);

        if (HAL_OK != status)
        {
            goto __EXIT;
        }
        fft->Instance->SAR += i * step;
        fft->Instance->DAR += i * step;

        fft->Instance->IRQ |= FFT_IRQ_IMR;
        fft->Instance->CR |= FFT_CR_START;
        while (0 == (fft->Instance->IRQ & FFT_IRQ_IRSR))
        {
            __NOP();
            __NOP();
            __NOP();
        }
        fft->Instance->IRQ = FFT_IRQ_ICR;
    }

__EXIT:
    fft->State = HAL_FFT_STATE_READY;
    return status;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FFT_StartDCT_IT(FFT_HandleTypeDef *fft, DCT_ConfigTypeDef *config)
{
    HAL_StatusTypeDef status;

    if (HAL_FFT_STATE_READY != fft->State)
    {
        status = HAL_BUSY;
        goto __EXIT;
    }

    fft->State = HAL_FFT_STATE_BUSY;


    status = DCT_Config(fft->Instance, config);

    if (HAL_OK != status)
    {
        goto __EXIT;
    }

    if (0 == config->rdct_flag)
    {
        fft->State = HAL_FFT_STATE_CDCT_REAL;
    }

    fft->Instance->IRQ &= ~FFT_IRQ_IMR;
    fft->Instance->CR |= FFT_CR_START;

__EXIT:

    if (HAL_OK != status)
    {
        fft->State = HAL_FFT_STATE_READY;
    }

    return status;
}

#endif /* SF32LB56X */


#endif /* HAL_FFT_MODULE_ENABLED */


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
