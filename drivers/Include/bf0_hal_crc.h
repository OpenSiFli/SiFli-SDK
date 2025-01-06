/**
  ******************************************************************************
  * @file   bf0_hal_crc.h
  * @author Sifli software development team
  * @brief   Header file of CRC HAL module.
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

#ifndef __BF0_HAL_CRC_H
#define __BF0_HAL_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup CRC CRC
  * @brief CRC HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup CRC_Exported_Types CRC Exported Types
  * @{
  */

/** @defgroup CRC_Exported_Types_Group1 CRC State definition
  * @{
  */

typedef enum
{
    HAL_CRC_STATE_RESET     = 0x00U,    /*!< CRC not yet initialized or disabled */
    HAL_CRC_STATE_READY     = 0x01U,    /*!< CRC initialized and ready for use   */
    HAL_CRC_STATE_BUSY      = 0x02U,    /*!< CRC internal process is ongoing     */
    HAL_CRC_STATE_TIMEOUT   = 0x03U,    /*!< CRC timeout state                   */
    HAL_CRC_STATE_ERROR     = 0x04U     /*!< CRC error state                     */
} HAL_CRC_StateTypeDef;

/**
  * @}
  */

/** @defgroup CRC_Mode CRC Mode  definition
  * @{
  */

typedef enum
{
    CRC_7_MMC = 0,                      /* !< CRC-7/MMC */
    CRC_8,                              /* !< CRC-8 */
    CRC_8_ITU,                          /* !< CRC-8/ITU */
    CRC_8_ROHC,                         /* !< CRC-8/ROHC */
    CRC_8_MAXIM,                        /* !< CRC-8/MAXIM */
    CRC_16_IBM,                         /* !< CRC-16/IBM */
    CRC_16_MAXIM,                       /* !< CRC-16/MAXIM */
    CRC_16_USB,                         /* !< CRC-16/USB */
    CRC_16_MODBUS,                      /* !< CRC-16/MODBUS */
    CRC_16_CCITT,                       /* !< CRC-16/CCITT */
    CRC_16_CCITT_FALSE,                 /* !< CRC-16/CCITT-FALSE */
    CRC_16_X25,                         /* !< CRC-16/X25 */
    CRC_16_XMODEM,                      /* !< CRC-16/XMODEM */
    CRC_16_DNP,                         /* !< CRC-16/DNP */
    CRC_32,                             /* !< CRC-32 */
    CRC_32_MPEG_2,                      /* !< CRC-32/MPEG_2 */
    CRC_MODE_NUM,                       /* !< Total defined CRC_MODE */
} HAL_CRC_Mode;
/**
  * @}
  */



/** @defgroup CRC_Exported_Types_Group2 CRC Handle Structure definition
  * @{
  */
typedef struct
{
    CRC_TypeDef                 *Instance;  /*!< Register base address   */

    HAL_LockTypeDef             Lock;       /*!< CRC locking object      */

    __IO HAL_CRC_StateTypeDef   State;      /*!< CRC communication state */

    HAL_CRC_Mode                Mode;       /*!< CRC mode                */
} CRC_HandleTypeDef;
/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/** @defgroup CRC_Exported_Macros CRC Exported Macros
  * @{
  */

#define HAL_CRC_POLYSIZE_7      3
#define HAL_CRC_POLYSIZE_8      2
#define HAL_CRC_POLYSIZE_16     1
#define HAL_CRC_POLYSIZE_32     0

#define HAL_CRC_DATASIZE_8      0
#define HAL_CRC_DATASIZE_16     1
#define HAL_CRC_DATASIZE_24     2
#define HAL_CRC_DATASIZE_32     3

/** @brief Resets CRC handle state
  * @param  \__HANDLE__ CRC handle
  * @retval None
  */
#define __HAL_CRC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_CRC_STATE_RESET)

/**
  * @brief  Resets CRC Data Register.
  * @param  \__HANDLE__ CRC handle
  * @retval None
  */
#define __HAL_CRC_DR_RESET(__HANDLE__) ((__HANDLE__)->Instance->CR |= CRC_CR_RESET)

/**
  * @brief Stores a 8-bit data in the Independent Data(ID) register.
  * @param  \__HANDLE__ CRC handle
  * @param  \__VALUE__ 8-bit value to be stored in the ID register
  * @retval None
  */
#define __HAL_CRC_SET_IDR(__HANDLE__, __VALUE__) (WRITE_REG((__HANDLE__)->Instance->IDR, (__VALUE__)))

/**
  * @brief Returns the 8-bit data stored in the Independent Data(ID) register.
  * @param  \__HANDLE__ CRC handle
  * @retval 8-bit value of the ID register
  */
#define __HAL_CRC_GET_IDR(__HANDLE__) (((__HANDLE__)->Instance->IDR) & CRC_IDR_IDR)


/* TODO: Revisit with real define*/
#define IS_CRC_ALL_INSTANCE(__INSTANCE__) (1)


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup CRC_Exported_Functions CRC Exported Functions
  * @{
  */

/** @defgroup CRC_Exported_Functions_Group1 Initialization and de-initialization functions
@verbatim
  ==============================================================================
            ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the CRC according to the specified parameters
          in the CRC_InitTypeDef and create the associated handle
      (+) DeInitialize the CRC peripheral
      (+) Initialize the CRC MSP
      (+) DeInitialize CRC MSP

@endverbatim
* @{
*/

/**
  * @brief  Initializes the CRC according to the specified
  *         parameters in the CRC_InitTypeDef and creates the associated handle.
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);

/**
  * @brief  DeInitializes the CRC peripheral.
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);

/**
  * @brief  Initializes the CRC MSP.
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @retval None
  */
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);

/**
  * @brief  DeInitializes the CRC MSP.
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @retval None
  */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);
/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group2 Peripheral Control functions
    @verbatim
      ==============================================================================
                      ##### Peripheral Control functions #####
      ==============================================================================
        [..]  This section provides functions allowing to:
          (+) Compute the 32-bit CRC value of 32-bit data buffer,
              using combination of the previous CRC value and the new one.
          (+) Compute the 32-bit CRC value of 32-bit data buffer,
              independently of the previous CRC value.

    @endverbatim
* @{
*/

/**
  * @brief  Computes the 32-bit CRC of 32-bit data buffer using combination
  *         of the previous CRC value and the new one.
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @param  pBuffer pointer to the buffer containing the data to be computed
  * @param  BufferLength length of the buffer to be computed
  * @retval 32-bit CRC
  */
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint8_t *pBuffer, uint32_t BufferLength);


/**
  * @brief  Computes the 32-bit CRC of 32-bit data buffer independently
  *         of the previous CRC value.
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @param  pBuffer Pointer to the buffer containing the data to be computed
  * @param  BufferLength Length of the buffer to be computed
  * @retval 32-bit CRC
  */
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint8_t *pBuffer, uint32_t BufferLength);


/**
  * @brief  Set CRC mode for calculation
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @param  mode for CRC calculation, please ref to @ref CRC_Mode
  */
void HAL_CRC_Setmode(CRC_HandleTypeDef *hcrc, HAL_CRC_Mode mode);


/**
  * @brief  Set Cutomized CRC mode for calculation
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @param  init initial CRC value
  * @param  poly Programmable polynomial
  * @param  mode Mode for CRC calculation, but use customized initial value and poly
*/
void HAL_CRC_Setmode_Customized(CRC_HandleTypeDef *hcrc, uint32_t init, uint32_t poly, HAL_CRC_Mode mode);


/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group3 Peripheral State functions
  * @{
  */
/**
  * @brief  Get current state of CRC module
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @retval Current state of CRC module
*/
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_CRC_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
