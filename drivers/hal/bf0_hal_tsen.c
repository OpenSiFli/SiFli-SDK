/**
  ******************************************************************************
  * @file   bf0_hal_tsen.c
  * @author Sifli software development team
  * @brief   Temperature sensor module driver.
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

/** @addtogroup TSEN
  * @{
  */

#if defined(HAL_TSEN_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

__HAL_ROM_USED HAL_StatusTypeDef HAL_TSEN_Init(TSEN_HandleTypeDef *htsen)
{
    /* Check the TSEN handle allocation */
    if (htsen == NULL)
    {
        return HAL_ERROR;
    }

    if (htsen->State == HAL_TSEN_STATE_RESET)
        /* Allocate lock resource and initialize it */
        htsen->Lock = HAL_UNLOCKED;

    if (htsen->Instance == NULL)
        htsen->Instance = hwp_tsen;


    HAL_RCC_EnableModule(RCC_MOD_TSEN);
#ifdef SF32LB52X
    hwp_hpsys_cfg->ANAU_CR |= HPSYS_CFG_ANAU_CR_EN_BG;
#else
    htsen->Instance->BGR |= TSEN_BGR_EN;
    htsen->Instance->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;

    HAL_Delay_us(50);
#endif
    /* Change TSEN peripheral state */
    htsen->State = HAL_TSEN_STATE_READY;

    /* Return function status */
    return HAL_OK;
}



__HAL_ROM_USED HAL_StatusTypeDef HAL_TSEN_DeInit(TSEN_HandleTypeDef *htsen)
{
    /* Check the TSEN handle allocation */
    if (htsen == NULL)
        return HAL_ERROR;

#ifdef SF32LB52X
    hwp_hpsys_cfg->ANAU_CR &= (~HPSYS_CFG_ANAU_CR_EN_BG);
#else
    htsen->Instance->ANAU_ANA_TP &= ~TSEN_ANAU_ANA_TP_ANAU_IARY_EN;
    htsen->Instance->BGR &= (~TSEN_BGR_EN);
#endif
    HAL_RCC_DisableModule(RCC_MOD_TSEN);

    /* Change TSEN peripheral state */
    htsen->State = HAL_TSEN_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(htsen);

    /* Return function status */
    return HAL_OK;
}


/**
  * @brief  Power on TSEN for preparing reading.
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  */
static void HAL_TSEN_Enable(TSEN_HandleTypeDef *htsen)
{
#ifndef SF32LB52X
    //TODO: for Micro, need use ADC_BG
    //htsen->Instance->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;
    //HAL_Delay(1);
#endif
    htsen->Instance->TSEN_CTRL_REG &= ~TSEN_TSEN_CTRL_REG_ANAU_TSEN_RSTB;
    htsen->Instance->TSEN_CTRL_REG |=  TSEN_TSEN_CTRL_REG_ANAU_TSEN_EN \
                                       | TSEN_TSEN_CTRL_REG_ANAU_TSEN_PU ;
    htsen->Instance->TSEN_CTRL_REG |= TSEN_TSEN_CTRL_REG_ANAU_TSEN_RSTB;
    HAL_Delay_us(20);
    htsen->Instance->TSEN_CTRL_REG |=  TSEN_TSEN_CTRL_REG_ANAU_TSEN_RUN ;
    htsen->State = HAL_TSEN_STATE_ENABLED;
}


/**
  * @brief  Power down TSEN after reading.
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  */
static void HAL_TSEN_Disable(TSEN_HandleTypeDef *htsen)
{
#ifndef SF32LB52X
    //TODO: for Micro, need use ADC_BG
    //htsen->Instance->ANAU_ANA_TP &= ~TSEN_ANAU_ANA_TP_ANAU_IARY_EN;
#endif
    htsen->Instance->TSEN_CTRL_REG &= ~(TSEN_TSEN_CTRL_REG_ANAU_TSEN_EN \
                                        | TSEN_TSEN_CTRL_REG_ANAU_TSEN_PU) ;
    htsen->State = HAL_TSEN_STATE_READY;
}


__HAL_ROM_USED void HAL_TSEN_IRQHandler(TSEN_HandleTypeDef *htsen)
{
    htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR;
    htsen->temperature = HAL_TSEN_Data(htsen);
    HAL_TSEN_Disable(htsen);
    NVIC_DisableIRQ(TSEN_IRQn);
}


__HAL_ROM_USED HAL_TSEN_StateTypeDef HAL_TSEN_Read_IT(TSEN_HandleTypeDef *htsen)
{
    if (htsen->State == HAL_TSEN_STATE_READY)
    {
        htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR;
        NVIC_ClearPendingIRQ(TSEN_IRQn);
        NVIC_EnableIRQ(TSEN_IRQn);
        HAL_TSEN_Enable(htsen);
        htsen->State = HAL_TSEN_STATE_BUSY;
        return htsen->State;
    }
    return -HAL_TSEN_STATE_ERROR;
}


__HAL_ROM_USED int HAL_TSEN_Read(TSEN_HandleTypeDef *htsen)
{
    int r = 0;

    if (htsen->State == HAL_TSEN_STATE_READY)
    {
        uint32_t count = 0;

        NVIC_DisableIRQ(TSEN_IRQn);
        HAL_TSEN_Enable(htsen);
        while ((htsen->Instance->TSEN_IRQ & TSEN_TSEN_IRQ_TSEN_IRSR) == 0)
        {
            HAL_Delay(1);
            count++;
            if (count > HAL_TSEN_MAX_DELAY)
            {
                r = -HAL_ERROR_TEMPRATURE;
                break;
            }
        }
        htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR;
        if (r >= 0)
        {
            r = HAL_TSEN_Data(htsen);
            htsen->temperature = r;
        }
        HAL_TSEN_Disable(htsen);
    }
    else
        r = -HAL_ERROR_TEMPRATURE;
    return r;
}

__HAL_ROM_USED HAL_TSEN_StateTypeDef HAL_TSEN_GetState(TSEN_HandleTypeDef *htsen)
{
    return htsen->State;
}



#endif /* HAL_TSEN_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
