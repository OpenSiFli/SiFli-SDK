/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
//TODO:
#if !defined(TSEN_BGR_EN)
    hwp_hpsys_cfg->ANAU_CR |= HPSYS_CFG_ANAU_CR_EN_BG;
#else
    htsen->Instance->BGR |= TSEN_BGR_EN;
    htsen->Instance->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;

    HAL_Delay_us(50);
#endif
    /* Change TSEN peripheral state */
    htsen->State = HAL_TSEN_STATE_READY;
    htsen->pclk = 0;

    /* Return function status */
    return HAL_OK;
}



__HAL_ROM_USED HAL_StatusTypeDef HAL_TSEN_DeInit(TSEN_HandleTypeDef *htsen)
{
    /* Check the TSEN handle allocation */
    if (htsen == NULL)
        return HAL_ERROR;

#if !defined(TSEN_BGR_EN)
//TODO: could anau_cr_en_bg be enabled always?
#ifdef SF32LB52X
    hwp_hpsys_cfg->ANAU_CR &= (~HPSYS_CFG_ANAU_CR_EN_BG);
#endif /* SF32LB52X */
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
#if !defined(SF32LB55X) && !defined(SF32LB56X) && !defined(SF32LB58X)
    uint32_t div;
    uint32_t pclk;
#endif /* !SF32LB55X && SF32LB56X && !SF32LB58X*/

#ifndef SF32LB52X
    //TODO: for Micro, need use ADC_BG
    //htsen->Instance->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;
    //HAL_Delay(1);
#endif

#if !defined(SF32LB55X) && !defined(SF32LB56X) && !defined(SF32LB58X)
    pclk = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
    if (pclk != htsen->pclk)
    {
        htsen->pclk = pclk;
        /* target freq: 0.5MHz */
        div = pclk / 500000;
        if (div > GET_REG_VAL2(TSEN_TSEN_CTRL_REG_ANAU_TSEN_CLK_DIV, TSEN_TSEN_CTRL_REG_ANAU_TSEN_CLK_DIV))
        {
            div = TSEN_TSEN_CTRL_REG_ANAU_TSEN_CLK_DIV;
        }
        else
        {
            div = MAKE_REG_VAL2(div, TSEN_TSEN_CTRL_REG_ANAU_TSEN_CLK_DIV);
        }
        MODIFY_REG(htsen->Instance->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_CLK_DIV_Msk, div);
    }
#endif /* !SF32LB55X && SF32LB56X && !SF32LB58X*/

    htsen->Instance->TSEN_CTRL_REG &= ~TSEN_TSEN_CTRL_REG_ANAU_TSEN_RSTB;
    htsen->Instance->TSEN_CTRL_REG |=  TSEN_TSEN_CTRL_REG_ANAU_TSEN_EN \
                                       | TSEN_TSEN_CTRL_REG_ANAU_TSEN_PU ;
    HAL_Delay_us(40);
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
    //TODO:
#ifdef TSEN_TSEN_IRQ_TSEN_ICR
    htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR;
#else
    htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR_0;
#endif
    htsen->temperature = HAL_TSEN_Data(htsen);
    HAL_TSEN_Disable(htsen);
    NVIC_DisableIRQ(TSEN_IRQn);
}


__HAL_ROM_USED HAL_TSEN_StateTypeDef HAL_TSEN_Read_IT(TSEN_HandleTypeDef *htsen)
{
    if (htsen->State == HAL_TSEN_STATE_READY)
    {
//TODO:
#ifdef TSEN_TSEN_IRQ_TSEN_ICR
        htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR;
#else
        htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR_0;
#endif
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
                /* sometimes interrupt is missing due to hw issue, still read the data from register */
                break;
            }
        }
#ifdef TSEN_TSEN_IRQ_TSEN_ICR
        htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR;
#else
        htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR_0;
#endif
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