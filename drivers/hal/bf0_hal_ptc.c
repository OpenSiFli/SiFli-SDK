/**
  ******************************************************************************
  * @file   bf0_hal_ptc.c
  * @author Sifli software development team
  * @brief Peripheral Task controller
  * @{
  ******************************************************************************
*/
/*
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


#if defined(HAL_PTC_ENABLED) ||defined(_SIFLI_DOXYGEN_)

/**
* @brief  Initialize PTC
* @param  hptc Handle of PTC
* @retval HAL_OK if successful, otherwise error
*/
__HAL_ROM_USED HAL_StatusTypeDef HAL_PTC_Init(PTC_HandleTypeDef *hptc)
{
    uint32_t address;

    HAL_ASSERT(hptc);
    HAL_ASSERT(hptc->Init.Channel < HAL_PTC_MAXCHN);

    address = (uint32_t)(&(hptc->Instance->TCR1));
    address += hptc->Init.Channel * sizeof(PTC_ChnTypeDef);
    hptc->Chn = (PTC_ChnTypeDef *)address;
    hptc->Chn->TAR = hptc->Init.Address;
    hptc->Chn->TDR = hptc->Init.data;
    hptc->State = HAL_PTC_STATE_READY;
    return HAL_OK;
}

/**
* @brief  Enable/disable PTC
* @param  hptc Handle of PTC
* @retval HAL_OK if successful, otherwise error
*/
__HAL_ROM_USED HAL_StatusTypeDef HAL_PTC_Enable(PTC_HandleTypeDef *hptc, int enable)
{
    if (enable)
    {
        uint32_t cr;
        hptc->Instance->ICR |= ((1UL << hptc->Init.Channel) | PTC_ICR_CTEIF);
        hptc->Instance->IER |= ((1UL << hptc->Init.Channel) | ((1UL << hptc->Init.Channel) << PTC_IER_TEIE_Pos));
        cr = hptc->Init.Sel;
        cr |= (((uint32_t)(hptc->Init.Operation)) << PTC_TCR1_OP_Pos);
#ifdef SF32LB58X
        cr |= (((uint32_t)(hptc->Init.Tripol)) << PTC_TCR1_TRIGPOL_Pos);
#endif // SOC_SF32LB58X
        hptc->Chn->TCR = cr;
#ifdef SF32LB58X
        if (hptc->Init.Channel < 4 && hptc->Init.Delay != 0)
        {
            hptc->Chn->RCR &= ~PTC_RCR1_DLY;
            hptc->Chn->RCR |= hptc->Init.Delay << PTC_RCR1_DLY_Pos;
        }
#endif // SOC_SF32LB58X
        hptc->State = HAL_PTC_STATE_RUNNING;
    }
    else
    {
        hptc->Instance->IER &= ~((1UL << hptc->Init.Channel) | ((1UL << hptc->Init.Channel) << PTC_IER_TEIE_Pos));
        hptc->State = HAL_PTC_STATE_READY;
    }
    return HAL_OK;
}

/**
* @brief  Handle PTC interrupt
* @param  hptc Handle of PTC
* @retval None
*/
__HAL_ROM_USED void HAL_PTC_IRQHandler(PTC_HandleTypeDef *hptc)
{
    hptc->Instance->ICR |= (1 << hptc->Init.Channel);        // Clear PTC interrupt.
}


#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
