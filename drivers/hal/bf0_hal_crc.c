/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
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

    /* Check the parameters */
    if (hcrc->Instance == NULL)
    {
        hcrc->Instance = hwp_crc;
    }
    else if (!IS_CRC_ALL_INSTANCE(hcrc->Instance))
    {
        return HAL_ERROR;
    }

    /* Allocate lock resource and initialize it */
    hcrc->Lock = HAL_UNLOCKED;

    if (hcrc->State == HAL_CRC_STATE_RESET)
    {
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
    hcrc->Mode = CRC_32;

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
        for (index = 0; index < (uint32_t)cnt; index++, size++)
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
