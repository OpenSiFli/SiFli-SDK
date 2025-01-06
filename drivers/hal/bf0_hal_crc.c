/**
  ******************************************************************************
  * @file   bf0_hal_crc.c
  * @author Sifli software development team
  * @brief   CRC HAL module driver.
  *          This file provides firmware functions to manage the following
  ******************************************************************************
*/
/**
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
#include "bf0_hal_crc.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup CRC
  * @{
  */

#if defined(HAL_CRC_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
typedef struct
{
    uint32_t    poly;
    uint32_t    init;
    uint32_t    cr_setting;
    uint32_t    xor_res;
} crc_config;

static const crc_config g_crc_config[] =
{
    {0x09,      0x00, (0 << CRC_CR_REV_IN_Pos) | (0 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_7 << CRC_CR_POLYSIZE_Pos), 0x00},
    {0x07,      0x00, (0 << CRC_CR_REV_IN_Pos) | (0 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_8 << CRC_CR_POLYSIZE_Pos), 0x00},
    {0x07,      0x00, (0 << CRC_CR_REV_IN_Pos) | (0 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_8 << CRC_CR_POLYSIZE_Pos), 0x55},
    {0x07,      0xFF, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_8 << CRC_CR_POLYSIZE_Pos), 0x00},
    {0x31,      0x00, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_8 << CRC_CR_POLYSIZE_Pos), 0x00},
    {0x8005,    0x0000, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0x0000},
    {0x8005,    0x0000, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0Xffff},
    {0x8005,    0xFFFF, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0Xffff},
    {0x8005,    0xFFFF, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0x0000},
    {0x1021,    0x0000, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0x0000},
    {0x1021,    0xFFFF, (0 << CRC_CR_REV_IN_Pos) | (0 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0x0000},
    {0x1021,    0xFFFF, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0Xffff},
    {0x1021,    0x0000, (0 << CRC_CR_REV_IN_Pos) | (0 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0x0000},
    {0x3D65,    0x0000, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_16 << CRC_CR_POLYSIZE_Pos), 0Xffff},
    {0x04C11DB7, 0xFFFFFFFF, (1 << CRC_CR_REV_IN_Pos) | (1 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_32 << CRC_CR_POLYSIZE_Pos), 0Xffffffff},
    {0x04C11DB7, 0xFFFFFFFF, (0 << CRC_CR_REV_IN_Pos) | (0 << CRC_CR_REV_OUT_Pos) | (HAL_CRC_POLYSIZE_32 << CRC_CR_POLYSIZE_Pos), 0x00000000},
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup CRC_Exported_Functions
  * @{
  */

/** @addtogroup CRC_Exported_Functions_Group1
 *  @brief   Initialization and de-initialization functions
 *
  * @{
  */

__HAL_ROM_USED HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc)
{
    /* Check the CRC handle allocation */
    if (hcrc == NULL)
    {
        return HAL_ERROR;
    }

    if (hcrc->Instance == NULL)
        hcrc->Instance = hwp_crc;
    /* Check the parameters */
    HAL_ASSERT(IS_CRC_ALL_INSTANCE(hcrc->Instance));

    if (hcrc->State == HAL_CRC_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hcrc->Lock = HAL_UNLOCKED;
        /* Init the low level hardware */
        HAL_CRC_MspInit(hcrc);
    }

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_BUSY;

    /* calculate CRC-32 */
    hcrc->Instance->CR = (HAL_CRC_POLYSIZE_32 << CRC_CR_POLYSIZE_Pos) | (HAL_CRC_DATASIZE_32 << CRC_CR_DATASIZE_Pos) | CRC_CR_REV_OUT | (1 << CRC_CR_REV_IN_Pos);
    hcrc->Instance->INIT = 0xFFFFFFFF;
    hcrc->Instance->POL
        = 0X04C11DB7;

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc)
{
    /* Check the CRC handle allocation */
    if (hcrc == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_CRC_ALL_INSTANCE(hcrc->Instance));

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_BUSY;

    /* DeInit the low level hardware */
    HAL_CRC_MspDeInit(hcrc);

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hcrc);

    /* Return function status */
    return HAL_OK;
}

__weak void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcrc);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CRC_MspInit could be implemented in the user file
     */
}

__weak void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcrc);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CRC_MspDeInit could be implemented in the user file
     */
}

/**
  * @}
  */


__HAL_ROM_USED uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint8_t *pBuffer, uint32_t BufferLength)
{
    uint32_t index = 0U;
    uint32_t cr = hcrc->Instance->CR, data;
    int size = HAL_CRC_DATASIZE_8;

    cr &= ~CRC_CR_DATASIZE_Msk;

    /* Process Locked */
    __HAL_LOCK(hcrc);

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_BUSY;

    data = 0;
    if (((uint32_t)(pBuffer)) & 3)                                  // First few bytes if not word aligned
    {
        int cnt = 4 - (((uint32_t)(pBuffer)) & 3);
        for (index = 0; index < cnt; index++, size++)
        {
            uint32_t t = (uint32_t)pBuffer[index];
            t <<= (8 * index);
            data |= t;
        }
#if 0
        LOG_D("size=%d, data=0x%x\n", size, data);
#endif
        hcrc->Instance->CR = cr | ((size - 1) << CRC_CR_DATASIZE_Pos);
        hcrc->Instance->DR = data;
        while (0 == (hcrc->Instance->SR & CRC_SR_DONE));
    }

    hcrc->Instance->CR |= (HAL_CRC_DATASIZE_32 << CRC_CR_DATASIZE_Pos);
#if 0
    LOG_D("index=%d, BufferLength=0x%x\n", index, BufferLength);
#endif
    /* Enter Data to the CRC calculator */
    for (; index < BufferLength - 3; index += 4)
    {
        hcrc->Instance->DR = *(uint32_t *)(&(pBuffer[index]));
        while (0 == (hcrc->Instance->SR & CRC_SR_DONE));
    }

    if (index < BufferLength)                                       // last few bytes if not word aligned
    {
        int i;
        data = 0;
        for (i = 0; index < BufferLength; index++, size++, i++)
        {
            uint32_t t = (uint32_t)pBuffer[index];
            t <<= (8 * i);
            data |= t;
        }
        size = HAL_CRC_DATASIZE_8 + i - 1;
#if 0
        LOG_D("Remain size=%d, data=0x%x\n", size, data);
#endif
        hcrc->Instance->CR = cr | (size << CRC_CR_DATASIZE_Pos);
        hcrc->Instance->DR = data;
        while (0 == (hcrc->Instance->SR & CRC_SR_DONE));
    }
    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hcrc);

    /* Return the CRC computed value */
    return (hcrc->Instance->DR ^ g_crc_config[hcrc->Mode].xor_res);
}

__HAL_ROM_USED uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint8_t *pBuffer, uint32_t BufferLength)
{
    __HAL_CRC_DR_RESET(hcrc);
    return HAL_CRC_Accumulate(hcrc, pBuffer, BufferLength);
}

__HAL_ROM_USED void HAL_CRC_Setmode(CRC_HandleTypeDef *hcrc, HAL_CRC_Mode mode)
{
    /* Reset CRC Calculation Unit */
    //__HAL_CRC_DR_RESET(hcrc);
    hcrc->Instance->INIT = g_crc_config[mode].init;
    hcrc->Instance->POL = g_crc_config[mode].poly;
    hcrc->Instance->CR = g_crc_config[mode].cr_setting;
    hcrc->Mode = mode;
    __HAL_CRC_DR_RESET(hcrc);
}


__HAL_ROM_USED void HAL_CRC_Setmode_Customized(CRC_HandleTypeDef *hcrc, uint32_t init, uint32_t poly, HAL_CRC_Mode mode)
{
    /* Reset CRC Calculation Unit */
    //__HAL_CRC_DR_RESET(hcrc);
    hcrc->Instance->INIT = init;
    hcrc->Instance->POL = poly;
    hcrc->Instance->CR = g_crc_config[mode].cr_setting;
    hcrc->Mode = mode;
    __HAL_CRC_DR_RESET(hcrc);
}

/**
  * @}
  */

/** @addtogroup CRC_Exported_Functions_Group3
 *  @brief   Peripheral State functions
 *
@verbatim
  ==============================================================================
                      ##### Peripheral State functions #####
  ==============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the CRC state.
  * @param  hcrc pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC
  * @retval HAL state
  */
__HAL_ROM_USED HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc)
{
    return hcrc->State;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_CRC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
