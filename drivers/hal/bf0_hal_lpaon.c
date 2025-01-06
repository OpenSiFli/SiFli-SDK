/**
  ******************************************************************************
  * @file   bf0_hal_lpaon.c
  * @author Sifli software development team
  * @brief   AON HAL module driver.
  *          This file provides firmware functions to manage the following
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

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup AON AON
  * @brief AON HAL module driver
  * @{
  */

#ifdef HAL_AON_MODULE_ENABLED

#ifndef SF32LB55X
    #define LPSYS_AON_CR_PIN0_MODE_Pos LPSYS_AON_CR1_PIN0_MODE_Pos
    #define LPSYS_AON_CR_PIN0_MODE_Msk LPSYS_AON_CR1_PIN0_MODE_Msk
    #define LPSYS_AON_CR_PIN0_MODE     LPSYS_AON_CR1_PIN0_MODE

    #define LPSYS_AON_CR_PIN1_MODE_Pos LPSYS_AON_CR1_PIN1_MODE_Pos

#endif /* SF32LB55X */


#ifdef SF32LB55X
/** wakeup pin to gpio pin mapping table,
 *
 * it can be overrided because FPGA configuration is different
 *
 *   pin0: PB43
 *   pin1: PB44
 *   pin2: PB45
 *   pin3: PB46
 *   pin4: PB47
 *   pin5: PB48
 *
 */
__weak const AON_WakeupPinTypeDef HAL_LPAON_WakeupPinMapTbl[] =
{
    {(GPIO_TypeDef *)hwp_gpio2, 43},
    {(GPIO_TypeDef *)hwp_gpio2, 44},
    {(GPIO_TypeDef *)hwp_gpio2, 45},
    {(GPIO_TypeDef *)hwp_gpio2, 46},
    {(GPIO_TypeDef *)hwp_gpio2, 47},
    {(GPIO_TypeDef *)hwp_gpio2, 48},
};
#elif defined(SF32LB58X)
/** wakeup pin to gpio pin mapping table,
 *
 *
 *   pin0: PB54
 *   pin1: PB55
 *   pin2: PB56
 *   pin3: PB57
 *   pin4: PB58
 *   pin5: PB59
 *
 */
__weak const AON_WakeupPinTypeDef HAL_LPAON_WakeupPinMapTbl[] =
{
    {(GPIO_TypeDef *)hwp_gpio2, 54},
    {(GPIO_TypeDef *)hwp_gpio2, 55},
    {(GPIO_TypeDef *)hwp_gpio2, 56},
    {(GPIO_TypeDef *)hwp_gpio2, 57},
    {(GPIO_TypeDef *)hwp_gpio2, 58},
    {(GPIO_TypeDef *)hwp_gpio2, 59},
    {(GPIO_TypeDef *)hwp_gpio1, 64},
    {(GPIO_TypeDef *)hwp_gpio1, 65},
    {(GPIO_TypeDef *)hwp_gpio1, 66},
    {(GPIO_TypeDef *)hwp_gpio1, 67},
    {(GPIO_TypeDef *)hwp_gpio1, 68},
    {(GPIO_TypeDef *)hwp_gpio1, 69},
    {(GPIO_TypeDef *)hwp_pbr,    0}, /* PBR0 */
    {(GPIO_TypeDef *)hwp_pbr,    1}, /* PBR1 */
    {(GPIO_TypeDef *)hwp_pbr,    2}, /* PBR2 */
    {(GPIO_TypeDef *)hwp_pbr,    3}, /* PBR3 */
    {(GPIO_TypeDef *)hwp_pbr,    4}, /* PBR4 */
    {(GPIO_TypeDef *)hwp_pbr,    5}, /* PBR5 */
};
#elif defined(SF32LB56X)
/** wakeup pin to gpio pin mapping table,
 *
 *   pin0:  PB32
 *   pin1:  PB33
 *   pin2:  PB34
 *   pin3:  PB35
 *   pin4:  PB36
 *   pin5:  PA50
 *   pin6:  PA51
 *   pin7:  PA52
 *   pin8:  PA53
 *   pin9:  PA54
 *
 */
__weak const AON_WakeupPinTypeDef HAL_LPAON_WakeupPinMapTbl[] =
{
    {(GPIO_TypeDef *)hwp_gpio2, 32},
    {(GPIO_TypeDef *)hwp_gpio2, 33},
    {(GPIO_TypeDef *)hwp_gpio2, 34},
    {(GPIO_TypeDef *)hwp_gpio2, 35},
    {(GPIO_TypeDef *)hwp_gpio2, 36},
    {(GPIO_TypeDef *)hwp_gpio1, 50},
    {(GPIO_TypeDef *)hwp_gpio1, 51},
    {(GPIO_TypeDef *)hwp_gpio1, 52},
    {(GPIO_TypeDef *)hwp_gpio1, 53},
    {(GPIO_TypeDef *)hwp_gpio1, 54},
    {(GPIO_TypeDef *)hwp_pbr,    0}, /* PBR0 */
    {(GPIO_TypeDef *)hwp_pbr,    1}, /* PBR1 */
    {(GPIO_TypeDef *)hwp_pbr,    2}, /* PBR2 */
    {(GPIO_TypeDef *)hwp_pbr,    3}, /* PBR3 */
};

#endif


#ifndef SF32LB52X
    #define HAL_LPAON_WAKEUP_PIN_NUM  (sizeof(HAL_LPAON_WakeupPinMapTbl)/sizeof(HAL_LPAON_WakeupPinMapTbl[0]))
#else
    #define HAL_LPAON_WAKEUP_PIN_NUM  (LPSYS_AON_WSR_PIN_NUM)
#endif /* SF32LB52X */



/**
 * @brief  LPSYS wakeup the specified HPSYS or BLESYS
 * @param  core_id core id, CORE_ID_HCPU or CORE_ID_BCPU
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPAON_WakeCore(uint8_t core_id)
{
    HAL_StatusTypeDef ret = HAL_OK;
    if (core_id == CORE_ID_HCPU)
    {
        hwp_lpsys_aon->ISSR |= LPSYS_AON_ISSR_LP2HP_REQ;
        while (!(hwp_lpsys_aon->ISSR & LPSYS_AON_ISSR_HP_ACTIVE));
    }
    else
    {
        ret = HAL_ERROR;
    }

    return ret;
}

/**
 * @brief  Indicate LPSYS to enter light sleep mode
 * @param  lscr light sleep control register value
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPAON_EnterLightSleep(uint32_t lscr)
{
    //hwp_lpsys_aon->LSCR = lscr;
    HAL_LPAON_SET_POWER_MODE(AON_PMR_LIGHT_SLEEP);

    return HAL_OK;
}

/**
 * @brief  Indicate LPSYS to enter deep sleep mode
 * @param  dscr light sleep control register value
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPAON_EnterDeepSleep(uint32_t dscr)
{
    //hwp_lpsys_aon->DSCR = dscr;
    HAL_LPAON_SET_POWER_MODE(AON_PMR_DEEP_SLEEP);

    return HAL_OK;
}

/**
 * @brief  Indicate BLESYS to enter standby mode
 * @param  dscr light sleep control register value
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPAON_EnterStandby(uint32_t sbcr)
{
    //hwp_lpsys_aon->SBCR = sbcr;
    HAL_LPAON_SET_POWER_MODE(AON_PMR_STANDBY);

    return HAL_OK;
}


/**
 * @brief  Enable specified LPSYS wakeup source
 * @param  src wakeup source
 * @param  mode pin mode
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPAON_EnableWakeupSrc(LPAON_WakeupSrcTypeDef src, AON_PinModeTypeDef mode)

{
    uint32_t mask;
    uint32_t val;
    uint32_t pos;
    uint32_t wer_en;
    __IO uint32_t *cr;

    if ((src >= LPAON_WAKEUP_SRC_PIN0) && (src <= LPAON_WAKEUP_SRC_PIN_LAST))
    {
        wer_en = (LPSYS_AON_WER_PIN0 << (src - LPAON_WAKEUP_SRC_PIN0));

        /* workaround: clear pin status as it could be set before WER is set to 1 */
        HAL_LPAON_CLEAR_WSR(wer_en);

#ifdef hwp_pbr
        if ((src >= LPAON_WAKEUP_SRC_PBR_PIN_FIRST) && (src <= LPAON_WAKEUP_SRC_PIN_LAST))
        {
            HAL_PBR_ConfigMode((src - LPAON_WAKEUP_SRC_PBR_PIN_FIRST), false);
        }
#endif /* hwp_pbr */

#ifndef SF32LB55X
        if (src >= LPAON_WAKEUP_SRC_PIN8)
        {
            src -= 8;
            cr = &hwp_lpsys_aon->CR2;
        }
        else
        {
            cr = &hwp_lpsys_aon->CR1;
        }
#else
        cr = &hwp_lpsys_aon->CR;
#endif

        pos = LPSYS_AON_CR_PIN0_MODE_Pos + (src - LPAON_WAKEUP_SRC_PIN0) * (LPSYS_AON_CR_PIN1_MODE_Pos - LPSYS_AON_CR_PIN0_MODE_Pos);
        mask = (LPSYS_AON_CR_PIN0_MODE_Msk << (pos - LPSYS_AON_CR_PIN0_MODE_Pos));
        val = MAKE_REG_VAL(mode, mask, pos);
        MODIFY_REG(*cr, mask, val);

        hwp_lpsys_aon->WER |= wer_en;
    }
    else
    {

        hwp_lpsys_aon->WER |= (1UL << src);
    }

    return HAL_OK;
}

/**
 * @brief  Disable specified LPSYS wakeup source
 * @param  src wakeup source
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPAON_DisableWakeupSrc(LPAON_WakeupSrcTypeDef src)
{
    uint32_t wer_en;

    if ((src >= LPAON_WAKEUP_SRC_PIN0) && (src <= LPAON_WAKEUP_SRC_PIN_LAST))
    {
        wer_en = (LPSYS_AON_WER_PIN0 << (src - LPAON_WAKEUP_SRC_PIN0));
    }
    else
    {
        wer_en = (1UL << src);
    }

    hwp_lpsys_aon->WER &= ~wer_en;

    return HAL_OK;
}


#ifdef SF32LB52X
__HAL_ROM_USED int8_t HAL_LPAON_QueryWakeupPin(GPIO_TypeDef *gpio, uint16_t gpio_pin)
{
    return HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);
}


__HAL_ROM_USED GPIO_TypeDef *HAL_LPAON_QueryWakeupGpioPin(uint8_t wakeup_pin, uint16_t *gpio_pin)
{

    return HAL_HPAON_QueryWakeupGpioPin(wakeup_pin, gpio_pin);
}


#else

__HAL_ROM_USED int8_t HAL_LPAON_QueryWakeupPin(GPIO_TypeDef *gpio, uint16_t gpio_pin)
{
    uint32_t i;
    int8_t wakeup_pin = -1;

    for (i = 0; i < HAL_LPAON_WAKEUP_PIN_NUM; i++)
    {
        if ((gpio == HAL_LPAON_WakeupPinMapTbl[i].gpio)
                && (gpio_pin == HAL_LPAON_WakeupPinMapTbl[i].pin))
        {
            wakeup_pin = i;
            break;
        }
    }
    return wakeup_pin;
}

__HAL_ROM_USED GPIO_TypeDef *HAL_LPAON_QueryWakeupGpioPin(uint8_t wakeup_pin, uint16_t *gpio_pin)
{
    GPIO_TypeDef *gpio;

    if (!gpio_pin)
    {
        return NULL;
    }

    if (wakeup_pin >= HAL_LPAON_WAKEUP_PIN_NUM)
    {
        return NULL;
    }

    *gpio_pin = HAL_LPAON_WakeupPinMapTbl[wakeup_pin].pin;
    gpio = HAL_LPAON_WakeupPinMapTbl[wakeup_pin].gpio;

    return gpio;
}
#endif /* SF32LB52X */

__HAL_ROM_USED  HAL_StatusTypeDef HAL_LPAON_GetWakeupPinMode(uint8_t wakeup_pin, AON_PinModeTypeDef *mode)
{
    uint32_t mask;
    uint32_t pos;
    __IO uint32_t *cr;

    if (!mode)
    {
        return HAL_ERROR;
    }

    if (wakeup_pin >= HAL_LPAON_WAKEUP_PIN_NUM)
    {
        return HAL_ERROR;

    }

#ifdef SF32LB55X
    cr = &hwp_lpsys_aon->CR;
#else
    if (wakeup_pin > 7)
    {
        wakeup_pin -= 8;
        cr = &hwp_lpsys_aon->CR2;
    }
    else
    {
        cr = &hwp_lpsys_aon->CR1;
    }
#endif

    pos = LPSYS_AON_CR_PIN0_MODE_Pos + wakeup_pin * (LPSYS_AON_CR_PIN1_MODE_Pos - LPSYS_AON_CR_PIN0_MODE_Pos);
    mask = (LPSYS_AON_CR_PIN0_MODE_Msk << (pos - LPSYS_AON_CR_PIN0_MODE_Pos));
    *mode = GET_REG_VAL(*cr, mask, pos);

    return HAL_OK;
}


__HAL_ROM_USED void HAL_LPAON_ConfigStartAddr(uint32_t *start_addr)
{
    hwp_lpsys_aon->SPR = (*start_addr);
    start_addr++;
    hwp_lpsys_aon->PCR = (*start_addr);
}

__HAL_ROM_USED void HAL_LPAON_Deactivate(void)
{
    HAL_LPAON_CLEAR_LP_ACTIVE();
}

__HAL_ROM_USED void HAL_LPAON_EnableRC(void)
{
    hwp_lpsys_aon->ACR |= LPSYS_AON_ACR_HRC48_REQ;
    while (0 == (hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HRC48_RDY))
    {
    }
}

__HAL_ROM_USED void HAL_LPAON_DisableRC(void)
{
    hwp_lpsys_aon->ACR &= ~LPSYS_AON_ACR_HRC48_REQ;
}

__HAL_ROM_USED void HAL_LPAON_EnableXT48(void)
{
    hwp_lpsys_aon->ACR |= LPSYS_AON_ACR_HXT48_REQ;
    while (0 == (hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
    {
        /* wait until HXT48 ready */
    }
}

__HAL_ROM_USED void HAL_LPAON_DisableXT48(void)
{
    hwp_lpsys_aon->ACR &= ~LPSYS_AON_ACR_HXT48_REQ;
}

#endif /* HAL_AON_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
