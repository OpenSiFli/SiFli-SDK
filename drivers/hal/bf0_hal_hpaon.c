/**
  ******************************************************************************
  * @file   bf0_hal_hpaon.c
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
    #define HPSYS_AON_CR_PIN0_MODE_Pos HPSYS_AON_CR1_PIN0_MODE_Pos
    #define HPSYS_AON_CR_PIN0_MODE_Msk HPSYS_AON_CR1_PIN0_MODE_Msk
    #define HPSYS_AON_CR_PIN0_MODE     HPSYS_AON_CR1_PIN0_MODE

    #define HPSYS_AON_CR_PIN1_MODE_Pos HPSYS_AON_CR1_PIN1_MODE_Pos

#endif /* SF32LB55X */


#ifdef SF32LB55X
/** wakeup pin to gpio pin mapping table,
 *
 * it can be overrided because FPGA configuration is different
 *
 *   pin0: PA77
 *   pin1: PA78
 *   pin2: PA79
 *   pin3: PA80
 *
 */
__weak const AON_WakeupPinTypeDef HAL_HPAON_WakeupPinMapTbl[] =
{
    {(GPIO_TypeDef *)hwp_gpio1, 77},
    {(GPIO_TypeDef *)hwp_gpio1, 78},
    {(GPIO_TypeDef *)hwp_gpio1, 79},
    {(GPIO_TypeDef *)hwp_gpio1, 80},
};
#elif defined(SF32LB58X)
/** wakeup pin to gpio pin mapping table,
 *
 *   pin0:  PB54
 *   pin1:  PB55
 *   pin2:  PB56
 *   pin3:  PB57
 *   pin4:  PB58
 *   pin5:  PB59
 *   pin6:  PA64
 *   pin7:  PA65
 *   pin8:  PA66
 *   pin9:  PA67
 *   pin10: PA68
 *   pin11: PA69
 *   pin12: PBR0
 *   pin13: PBR1
 *   pin14: PBR2
 *   pin15: PBR3
 *   pin16: PBR4
 *   pin17: PBR5
 *
 */
__weak const AON_WakeupPinTypeDef HAL_HPAON_WakeupPinMapTbl[] =
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
 *   pin10: PBR0
 *   pin11: PBR1
 *   pin12: PBR2
 *   pin13: PBR3
 *
 */
__weak const AON_WakeupPinTypeDef HAL_HPAON_WakeupPinMapTbl[] =
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

#endif /* SF32LB55X */


#ifndef SF32LB52X
    #define HAL_HPAON_WAKEUP_PIN_NUM  (sizeof(HAL_HPAON_WakeupPinMapTbl)/sizeof(HAL_HPAON_WakeupPinMapTbl[0]))
#else
    #define HAL_HPAON_WAKEUP_PIN_NUM  (HPSYS_AON_WSR_PIN_NUM)
    #define HAL_HPAON_WAKEUP_PIN_FIRST   (24)
    #define HAL_HPAON_WAKEUP_PIN_LAST    (44)
#endif /* SF32LB52X */

#ifdef SF32LB52X
    HAL_RETM_BSS_SECT(g_hal_hpaon_lcpu_wakeup_ref_cnt, uint8_t g_hal_hpaon_lcpu_wakeup_ref_cnt);
#endif /* SF32LB52X */

/**
 * @brief  HPSYS wakeup the specified LPSYS or BLESYS
 * @param  core_id core id, CORE_ID_LCPU or CORE_ID_BCPU
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_WakeCore(uint8_t core_id)
{
    HAL_StatusTypeDef ret = HAL_OK;
    if (core_id == CORE_ID_LCPU)
    {
#ifdef SF32LB52X
        uint32_t mask;
#endif /* SF32LB52X */
        hwp_hpsys_aon->ISSR |= HPSYS_AON_ISSR_HP2LP_REQ;
        /* delay to ensure LCPU see the REQ, need one LP_CLK cycle, see: gitlab#1752,redmine#666, ext-redmine#663
         * double the value for enough margin
         */
        HAL_Delay_us(230);
        while (!(hwp_hpsys_aon->ISSR & HPSYS_AON_ISSR_LP_ACTIVE));
        HAL_Delay_us(30);
        /* delay to ensure HCPU see the updated LP_ACTIVE */
        while (!(hwp_hpsys_aon->ISSR & HPSYS_AON_ISSR_LP_ACTIVE));
#ifdef SF32LB52X
        mask = HAL_DisableInterrupt();
        HAL_ASSERT(g_hal_hpaon_lcpu_wakeup_ref_cnt < 20);
        g_hal_hpaon_lcpu_wakeup_ref_cnt++;
        HAL_EnableInterrupt(mask);
#endif /* SF32LB52X */
    }
    else
    {
        ret = HAL_ERROR;
    }

    return ret;
}

/**
 * @brief  Indicate HPSYS to enter light sleep mode
 * @param  lscr light sleep control register value
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_EnterLightSleep(uint32_t lscr)
{
    HAL_HPAON_SET_POWER_MODE(AON_PMR_LIGHT_SLEEP);
    //hwp_hpsys_aon->LSCR = lscr;

    return HAL_OK;
}

/**
 * @brief  Indicate HPSYS to enter deep sleep mode
 * @param  dscr deep sleep control register value
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_EnterDeepSleep(uint32_t dscr)
{
    HAL_HPAON_SET_POWER_MODE(AON_PMR_DEEP_SLEEP);
    //hwp_hpsys_aon->DSCR = dscr;

    return HAL_OK;
}

/**
 * @brief  Indicate HPSYS to enter standby mode
 * @param  sbcr standby sleep control register value
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_EnterStandby(uint32_t sbcr)
{
    HAL_HPAON_SET_POWER_MODE(AON_PMR_STANDBY);
    //hwp_hpsys_aon->SBCR = sbcr;

    return HAL_OK;
}

/**
 * @brief  Enable specified HPSYS wakeup source
 * @param  src wakeup source
 * @param  mode pin mode
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_EnableWakeupSrc(HPAON_WakeupSrcTypeDef src, AON_PinModeTypeDef mode)

{
    uint32_t mask;
    uint32_t val;
    uint32_t pos;
    uint32_t wer_en;
    __IO uint32_t *cr;

    if ((src >= HPAON_WAKEUP_SRC_PIN0) && (src <= HPAON_WAKEUP_SRC_PIN_LAST))
    {
        wer_en = (HPSYS_AON_WER_PIN0 << (src - HPAON_WAKEUP_SRC_PIN0));

        /* workaround: clear pin status as it could be set before WER is set to 1 */
        HAL_HPAON_CLEAR_WSR(wer_en);

#ifdef hwp_pbr
        if ((src >= HPAON_WAKEUP_SRC_PBR_PIN_FIRST) && (src <= HPAON_WAKEUP_SRC_PIN_LAST))
        {
            HAL_PBR_ConfigMode((src - HPAON_WAKEUP_SRC_PBR_PIN_FIRST), false);
        }
#endif /* hwp_pbr */

#ifndef SF32LB55X

        if (src >= HPAON_WAKEUP_SRC_PIN8)
        {
            src -= 8;
            cr = &hwp_hpsys_aon->CR2;
        }
        else
        {
            cr = &hwp_hpsys_aon->CR1;
        }
#else
        cr = &hwp_hpsys_aon->CR;

#endif /* SF32LB55X */

        pos = HPSYS_AON_CR_PIN0_MODE_Pos + (src - HPAON_WAKEUP_SRC_PIN0) * (HPSYS_AON_CR_PIN1_MODE_Pos - HPSYS_AON_CR_PIN0_MODE_Pos);
        mask = (HPSYS_AON_CR_PIN0_MODE_Msk << (pos - HPSYS_AON_CR_PIN0_MODE_Pos));
        val = MAKE_REG_VAL(mode, mask, pos);
        MODIFY_REG(*cr, mask, val);

        hwp_hpsys_aon->WER |= wer_en;
    }
    else
    {
        hwp_hpsys_aon->WER |= (1UL << src);
    }

    return HAL_OK;
}

/**
 * @brief  Disable specified HPSYS wakeup source
 * @param  src wakeup source
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_DisableWakeupSrc(HPAON_WakeupSrcTypeDef src)
{
    uint32_t wer_en;

    if ((src >= HPAON_WAKEUP_SRC_PIN0) && (src <= HPAON_WAKEUP_SRC_PIN_LAST))
    {
        wer_en = (HPSYS_AON_WER_PIN0 << (src - HPAON_WAKEUP_SRC_PIN0));
    }
    else
    {
        wer_en = (1UL << src);
    }

    hwp_hpsys_aon->WER &= ~wer_en;

    return HAL_OK;
}

#ifdef SF32LB52X
__HAL_ROM_USED int8_t HAL_HPAON_QueryWakeupPin(GPIO_TypeDef *gpio, uint16_t gpio_pin)
{
    int8_t wakeup_pin = -1;

    if ((gpio == hwp_gpio1) && (gpio_pin >= HAL_HPAON_WAKEUP_PIN_FIRST) && (gpio_pin <= HAL_HPAON_WAKEUP_PIN_LAST))
    {
        wakeup_pin = gpio_pin - HAL_HPAON_WAKEUP_PIN_FIRST;
    }

    return wakeup_pin;
}


__HAL_ROM_USED GPIO_TypeDef *HAL_HPAON_QueryWakeupGpioPin(uint8_t wakeup_pin, uint16_t *gpio_pin)
{
    GPIO_TypeDef *gpio;

    if (!gpio_pin)
    {
        return NULL;
    }

    if (wakeup_pin >= HAL_HPAON_WAKEUP_PIN_NUM)
    {
        return NULL;
    }

    *gpio_pin = wakeup_pin + HAL_HPAON_WAKEUP_PIN_FIRST;
    gpio = hwp_gpio1;

    return gpio;
}


#else
__HAL_ROM_USED int8_t HAL_HPAON_QueryWakeupPin(GPIO_TypeDef *gpio, uint16_t gpio_pin)
{
    uint32_t i;
    int8_t wakeup_pin = -1;

    for (i = 0; i < HAL_HPAON_WAKEUP_PIN_NUM; i++)
    {
        if ((gpio == HAL_HPAON_WakeupPinMapTbl[i].gpio)
                && (gpio_pin == HAL_HPAON_WakeupPinMapTbl[i].pin))
        {
            wakeup_pin = i;
            break;
        }
    }
    return wakeup_pin;
}

__HAL_ROM_USED GPIO_TypeDef *HAL_HPAON_QueryWakeupGpioPin(uint8_t wakeup_pin, uint16_t *gpio_pin)
{
    GPIO_TypeDef *gpio;

    if (!gpio_pin)
    {
        return NULL;
    }

    if (wakeup_pin >= HAL_HPAON_WAKEUP_PIN_NUM)
    {
        return NULL;
    }

    *gpio_pin = HAL_HPAON_WakeupPinMapTbl[wakeup_pin].pin;
    gpio = HAL_HPAON_WakeupPinMapTbl[wakeup_pin].gpio;

    return gpio;
}


#endif /* SF32LB52X */

__HAL_ROM_USED  HAL_StatusTypeDef HAL_HPAON_GetWakeupPinMode(uint8_t wakeup_pin, AON_PinModeTypeDef *mode)
{
    uint32_t mask;
    uint32_t pos;
    __IO uint32_t *cr;

    if (!mode)
    {
        return HAL_ERROR;
    }

    if (wakeup_pin >= HAL_HPAON_WAKEUP_PIN_NUM)
    {
        return HAL_ERROR;

    }

#ifdef SF32LB55X
    cr = &hwp_hpsys_aon->CR;
#else
    if (wakeup_pin > 7)
    {
        wakeup_pin -= 8;
        cr = &hwp_hpsys_aon->CR2;
    }
    else
    {
        cr = &hwp_hpsys_aon->CR1;
    }

#endif
    pos = HPSYS_AON_CR_PIN0_MODE_Pos + wakeup_pin * (HPSYS_AON_CR_PIN1_MODE_Pos - HPSYS_AON_CR_PIN0_MODE_Pos);
    mask = (HPSYS_AON_CR_PIN0_MODE_Msk << (pos - HPSYS_AON_CR_PIN0_MODE_Pos));
    *mode = GET_REG_VAL(*cr, mask, pos);

    //rt_kprintf("pos:%d,%d,%x\n", pos, mask,*cr);

    return HAL_OK;
}

#ifdef SOC_BF0_HCPU
    __USED
#endif
__HAL_ROM_USED void HAL_HPAON_Deactivate(void)
{
    HAL_HPAON_CLEAR_HP_ACTIVE();
    HAL_HPAON_DISABLE_PAD();
#ifndef SF32LB55X
    HAL_HPAON_DISABLE_VHP();
#endif // SF32LB55X
}

__HAL_ROM_USED void HAL_HPAON_EnableRC(void)
{
    hwp_hpsys_aon->ACR |= HPSYS_AON_ACR_HRC48_REQ;
    while (0 == (hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HRC48_RDY))
    {
    }
}

__HAL_ROM_USED void HAL_HPAON_DisableRC(void)
{
    hwp_hpsys_aon->ACR &= ~HPSYS_AON_ACR_HRC48_REQ;
}

__HAL_ROM_USED void HAL_HPAON_EnableXT48(void)
{
    hwp_hpsys_aon->ACR |= HPSYS_AON_ACR_HXT48_REQ;
    while (0 == (hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY))
    {
        /* wait until HXT48 ready */
    }
}

__HAL_ROM_USED void HAL_HPAON_DisableXT48(void)
{
    hwp_hpsys_aon->ACR &= ~HPSYS_AON_ACR_HXT48_REQ;
}

HAL_StatusTypeDef HAL_GTIMER_Check(uint32_t start, uint32_t timeout_count)
{
    uint32_t val;
    HAL_StatusTypeDef r = HAL_OK;

    val = HAL_GTIMER_READ();

    if (val < start)
        val = 0xFFFFFFFF - start + val;
    else
        val = val - start;
    if (val >= timeout_count)
        r = HAL_TIMEOUT;
    return r;
}



__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_StartGTimer(void)
{
    HAL_StatusTypeDef r = HAL_ERROR;

    if (hwp_hpsys_aon->ISSR & HPSYS_AON_ISSR_LP_ACTIVE)
    {
#ifndef SF32LB55X
        hwp_lpsys_aon->CR1 |= LPSYS_AON_CR1_GTIM_EN;
        hwp_hpsys_aon->CR1 |= HPSYS_AON_CR1_GTIM_EN;
#else
        /* enable gtimer in two subsys */
        hwp_lpsys_aon->CR |= LPSYS_AON_CR_GTIM_EN;
        hwp_hpsys_aon->CR |= HPSYS_AON_CR_GTIM_EN;
#endif
        /* sync two gtimer */
        hwp_hpsys_aon->GTIMR = 1;

        r = HAL_OK;
    }

    return r;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_StopGTimer(void)
{
    HAL_StatusTypeDef r = HAL_ERROR;

    if (hwp_hpsys_aon->ISSR & HPSYS_AON_ISSR_LP_ACTIVE)
    {
#ifndef SF32LB55X
        hwp_lpsys_aon->CR1 &= ~LPSYS_AON_CR1_GTIM_EN;
        hwp_hpsys_aon->CR1 &= ~HPSYS_AON_CR1_GTIM_EN;
#else
        /* enable gtimer in two subsys */
        hwp_lpsys_aon->CR &= ~LPSYS_AON_CR_GTIM_EN;
        hwp_hpsys_aon->CR &= ~HPSYS_AON_CR_GTIM_EN;
#endif
        r = HAL_OK;
    }

    return r;
}


#endif /* HAL_AON_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
