/**
  ******************************************************************************
  * @file   bf0_hal_fft.h
  * @author Sifli software development team
  * @brief   Header file of FFT HAL module.
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_FFT_H
#define __BF0_HAL_FFT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup FFT
  * @ingroup BF0_HAL_Driver
  * @{
  */

/**
 * @brief  HAL FFT State structures definition
 */
typedef enum
{
    HAL_FFT_STATE_RESET             = 0x00U,    /*!< FFT not yet initialized or disabled       */
    HAL_FFT_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use  */
    HAL_FFT_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing            */
    HAL_FFT_STATE_ERROR             = 0x03U,    /*!< FFT state error                           */
    HAL_FFT_STATE_CDCT_REAL         = 0x04U,    /*!< CDCT real part calculation ongoing state  */
} HAL_FFT_StateTypeDef;

/** FFT handle structure definition */
typedef struct __FFT_HandleTypeDef
{
    FFT_TypeDef              *Instance;                                  /*!< FFT register base address.          */
    void (*CpltCallback)(struct __FFT_HandleTypeDef *fft);              /*!< FFT processing complete callback.   */
    __IO HAL_FFT_StateTypeDef State;                                     /*!< FFT state.                          */
} FFT_HandleTypeDef;

/** FFT Bitwidth Type Definition */
typedef enum
{
    FFT_BW_8BIT = 0,
    FFT_BW_16BIT = 1,
    FFT_BW_32BIT = 2,
    FFT_BW_MAX
} FFT_BitwidthTypeDef;

/** FFT Length Type Definition */
typedef enum
{
    FFT_LEN_16 = 0,
    FFT_LEN_32 = 1,
    FFT_LEN_64 = 2,
    FFT_LEN_128 = 3,
    FFT_LEN_256 = 4,
    FFT_LEN_512 = 5,
    FFT_LEN_1024 = 6,
    FFT_LEN_2048 = 7,
    FFT_LEN_4096 = 8,
    FFT_LEN_TYPE_NUM,
} FFT_FFTLengthTypeDef;

#ifdef SF32LB58X
/** FFT1 max length */
#define FFT1_LEN_MAX   (FFT_LEN_4096)
#else
/** FFT1 max length */
#define FFT1_LEN_MAX   (FFT_LEN_512)
#endif

#ifdef hwp_fft2
/** FFT2 max length */
#define FFT2_LEN_MAX   (FFT_LEN_1024)
#endif

/** FFT Config */
typedef struct
{
    FFT_BitwidthTypeDef bitwidth;
    /** FFT Length,
     *
     * Pro: FFT1 support 16~4096, FFT2 supports 16~1024
     * Lite: FFT1 support 16~512
     */
    FFT_FFTLengthTypeDef fft_length;
    /** real fft flag. 1: real fft, 0: complex fft */
    uint8_t rfft_flag;
    /** IFFT flag. 1: ifft, 0: fft */
    uint8_t ifft_flag;
    /** input data, address must be 4 bytes aligned */
    void *input_data;
    /** output data, address must be 4 bytes aligned */
    void *output_data;
#ifdef SF32LB56X
    /** window enabled, true: enabled, false: not enabled*/
    bool win_enabled;
    FFT_BitwidthTypeDef win_bitwidth;
    /** real window flag. 1: real window, 0: complex window */
    uint8_t rwin_flag;
    void *win_data;
#endif /* SF32LB56X */
} FFT_ConfigTypeDef;

#ifdef SF32LB56X

/** DCT Length Type Definition */
typedef enum
{
    DCT_LEN_8 = 0,
    DCT_LEN_16 = 1,
    DCT_LEN_32 = 2,
    DCT_LEN_64 = 3,
    DCT_LEN_128 = 4,
    DCT_LEN_256 = 5,
    DCT_LEN_TYPE_NUM,
} FFT_DCTLengthTypeDef;

typedef enum
{
    DCT_TYPE_INVALID = 0,
    DCT_TYPE_2 = 1,
    DCT_TYPE_3 = 2,
    DCT_TYPE_4 = 3,
} FFT_DCTTypeDef;


/** DCT Config */
typedef struct
{
    FFT_DCTTypeDef dct_type;
    FFT_BitwidthTypeDef bitwidth;
    /** DCT Length,
     *
     */
    FFT_DCTLengthTypeDef dct_length;
    /** real dct flag. 1: real dct, 0: complex dct */
    uint8_t rdct_flag;
    /** IDCT flag. 1: idct, 0: dct */
    uint8_t idct_flag;
    void *input_data;
    void *output_data;
} DCT_ConfigTypeDef;


#endif /* SF32LB56X */

/**
 * @brief  Init FFT Accelerator
 * @retval status
 *
 */
HAL_StatusTypeDef HAL_FFT_Init(FFT_HandleTypeDef *fft);

/**
 * @brief  DeInit FFT Accelerator
 * @retval status
 *
 */
HAL_StatusTypeDef HAL_FFT_DeInit(FFT_HandleTypeDef *fft);


/**
 * @brief  Start FFT in synchronous mode
 *
 * 8bit FFT: input format is Q1.7, output format as follows,
 * for FFT length greater than 128, output value needs to be upscaled accordingly further to match the FFT definition
 *
 *    FFT  Length     | Output Format    | Upscale value    |
 *    ----------------|------------------|------------------|
 *       16           |   5.3            | 0                |
 *       32           |   6.3            | 0                |
 *       64           |   7.1            | 0                |
 *      128           |   8.0            | 0                |
 *      256           |   8.0            | upscale by 2     |
 *      512           |   8.0            | upscale by 4     |
 *     1024           |   8.0            | upscale by 8     |
 *     2048           |   8.0            | upscale by 16    |
 *     4096           |   8.0            | upscale by 32    |
 *
 *
 *
 * 8bit IFFT: input format is Q1.7, output format is Q1.7
 *
 *
 * 16bit FFT: input format is Q1.15, output format as follows
 *
 *    FFT  Length     | Output Format    |
 *    ----------------|------------------|
 *       16           |   5.11           |
 *       32           |   6.10           |
 *       64           |   7.9            |
 *      128           |   8.8            |
 *      256           |   9.7            |
 *      512           |  10.6            |
 *     1024           |  11.5            |
 *     2048           |  12.4            |
 *     4096           |  13.3            |
 *
 *
 * 16bit IFFT: input format is Q1.15, output format is Q1.15
 *
 *
 *
 * 32bit FFT: input format is Q1.31, output format as follows
 *
 *    FFT  Length     | Output Format    |
 *    ----------------|------------------|
 *       16           |   5.27           |
 *       32           |   6.26           |
 *       64           |   7.25           |
 *      128           |   8.24           |
 *      256           |   9.23           |
 *      512           |  10.22           |
 *     1024           |  11.21           |
 *     2048           |  12.20           |
 *     4096           |  13.19           |
 *
 *
 * 32bit IFFT: input format is Q1.31, output format is Q1.31
 *
 *
 * @param[in] fft FFT handle
 * @param[in] config FFT config
 * @retval status
 */
HAL_StatusTypeDef HAL_FFT_StartFFT(FFT_HandleTypeDef *fft, FFT_ConfigTypeDef *config);

/**
 * @brief  Start FFT in asynchronous mode
 *
 *  CpltCallback is called after complete
 *
 *  Input and output data format is same as @ref HAL_FFT_StartFFT
 *
 * @param[in] fft FFT handle
 * @param[in] config FFT config
 * @retval status
 */

HAL_StatusTypeDef HAL_FFT_StartFFT_IT(FFT_HandleTypeDef *fft, FFT_ConfigTypeDef *config);

#if defined(SF32LB56X) || defined(_SIFLI_DOXYGEN_)


/**
 * @brief  Start DCT in synchronous mode
 *
 *
 * 8bit DCT: input format is Q1.7, output format as follows
 *
 *    DCT  Length     | Output Format    |
 *    ----------------|------------------|
 *        8           |   4.4            |
 *       16           |   5.3            |
 *       32           |   5.3            |
 *       64           |   6.2            |
 *      128           |   6.2            |
 *      256           |   7.1            |
 *
 *
 * 16bit DCT: input format is Q1.15, output format as follows
 *
 *    DCT  Length     | Output Format    |
 *    ----------------|------------------|
 *        8           |   4.12           |
 *       16           |   5.11           |
 *       32           |   5.11           |
 *       64           |   6.10           |
 *      128           |   6.10           |
 *      256           |   7.9            |
 *
 * 32bit DCT: input format is Q1.31, output format as follows
 *
 *    DCT  Length     | Output Format    |
 *    ----------------|------------------|
 *        8           |   4.28           |
 *       16           |   5.27           |
 *       32           |   5.27           |
 *       64           |   6.26           |
 *      128           |   6.26           |
 *      256           |   7.25           |
 *
 *
 * @param[in] fft FFT handle
 * @param[in] config DCT config
 * @retval status
 */
HAL_StatusTypeDef HAL_FFT_StartDCT(FFT_HandleTypeDef *fft, DCT_ConfigTypeDef *config);

/**
 * @brief  Start DCT in asynchronous mode
 *
 *  CpltCallback is called after complete
 *
 *  Input and output data format is same as @ref HAL_FFT_StartDCT
 *
 * @param[in] fft FFT handle
 * @param[in] config DCT config
 *
 * @retval status
 */

HAL_StatusTypeDef HAL_FFT_StartDCT_IT(FFT_HandleTypeDef *fft, DCT_ConfigTypeDef *config);
#endif /* SF32LB56X */

HAL_StatusTypeDef HAL_FFT_IRQHandler(FFT_HandleTypeDef *fft);


#ifdef __cplusplus
}
#endif

///@}   EFUSE

#endif /* __BF0_HAL_FFT_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
