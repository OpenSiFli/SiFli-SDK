/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
#ifndef SF32LB55X
        cr |= (((uint32_t)(hptc->Init.Tripol)) << PTC_TCR1_TRIGPOL_Pos);
        cr |= (((uint32_t)(hptc->Init.RepEn)) << PTC_TCR1_REPEN_Pos);
        cr |= (((uint32_t)(hptc->Init.RepTrig)) << PTC_TCR1_REPTRIG_Pos);
        cr |= (((uint32_t)(hptc->Init.RepIRQ)) << PTC_TCR1_REPIRQ_Pos);
#endif /* !SF32LB55X */

        hptc->Chn->TCR = cr;
#ifndef SF32LB55X
        if (hptc->Init.Channel < 4 && hptc->Init.Delay != 0)
        {
            hptc->Chn->RCR &= ~PTC_RCR1_DLY;
            hptc->Chn->RCR |= hptc->Init.Delay << PTC_RCR1_DLY_Pos;
        }

        if (hptc->Init.RepEn)
        {
            MODIFY_REG(hptc->Chn->RCR, PTC_RCR1_REP_Msk, MAKE_REG_VAL2(hptc->Init.Pen, PTC_RCR1_REP));
        }

#endif /* !SF32LB55X */

        if ((hptc->Init.Trigger_Pin < 32) && (hptc->Init.Sel == PTC_HCPU_PA31_0_A))
        {
            hptc->Instance->GPIO31_0 = (hptc->Init.Trigger_Pin - 0) << PTC_GPIO31_0_SELA_Pos;
        }
        else if ((hptc->Init.Trigger_Pin < 64) && (hptc->Init.Sel == PTC_HCPU_PA63_32_A))
        {
            hptc->Instance->GPIO63_32 = (hptc->Init.Trigger_Pin - 32) << PTC_GPIO63_32_SELA_Pos;
        }
#ifdef PTC_GPIO95_64_SELA_Pos
        else if ((hptc->Init.Trigger_Pin < 95) && (hptc->Init.Sel == PTC_HCPU_PA95_64_A))
        {
            hptc->Instance->GPIO95_64 = (hptc->Init.Trigger_Pin - 64) << PTC_GPIO95_64_SELA_Pos;
        }
#endif /* PTC_GPIO95_64_SELA_Pos */
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