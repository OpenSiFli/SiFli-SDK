/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
__HAL_ROM_USED int8_t HAL_HPAON_QueryWakeupPin(GPIO_TypeDef *gpio, uint16_t gpio_pin)
{
    int8_t wakeup_pin = -1;
    if (gpio != hwp_gpio1)
    {
        return wakeup_pin;
    }

    if ((gpio_pin >= HPAON_WAKEUP_PIN_PART0_FIRST) && (gpio_pin <= HPAON_WAKEUP_PIN_PART0_LAST))
    {
        wakeup_pin = gpio_pin - HPAON_WAKEUP_PIN_PART0_FIRST;
    }
    else if ((gpio_pin >= HPAON_WAKEUP_PIN_PART1_FIRST) && (gpio_pin <= HPAON_WAKEUP_PIN_PART1_LAST))
    {
        wakeup_pin = gpio_pin - HPAON_WAKEUP_PIN_PART1_FIRST + HPAON_WAKEUP_PIN_PART0_SIZE;
    }
    else
    {
        /* do nothing */
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

    if (wakeup_pin >= HPAON_WAKEUP_PIN_NUM)
    {
        return NULL;
    }

    if (wakeup_pin < HPAON_WAKEUP_PIN_PART0_SIZE)
    {
        /* PA33 ~ PA42 */
        *gpio_pin = HPAON_WAKEUP_PIN_PART0_FIRST + wakeup_pin;
    }
    else
    {
        /* PA24 ~ PA27 */
        wakeup_pin -= HPAON_WAKEUP_PIN_PART0_SIZE;
        *gpio_pin = HPAON_WAKEUP_PIN_PART1_FIRST + wakeup_pin;
    }
    gpio = hwp_gpio1;

    return gpio;
}

HAL_StatusTypeDef HAL_HPAON_GetWakeupPinMode(uint8_t wakeup_pin, AON_PinModeTypeDef *mode)
{
    uint32_t mask;
    uint32_t pos;
    __IO uint32_t *wkup_mode;

    if (!mode)
    {
        return HAL_ERROR;
    }

    if (wakeup_pin >= HPAON_WAKEUP_PIN_NUM)
    {
        return HAL_ERROR;

    }

    wkup_mode = &hwp_pmuc->WKUP_MODE;
    pos = PMUC_WKUP_MODE_PA33_MODE_Pos + wakeup_pin * (PMUC_WKUP_MODE_PA34_MODE_Pos - PMUC_WKUP_MODE_PA33_MODE_Pos);
    mask = (PMUC_WKUP_MODE_PA33_MODE_Msk << (pos - PMUC_WKUP_MODE_PA33_MODE_Pos));
    *mode = GET_REG_VAL(*wkup_mode, mask, pos);

    //rt_kprintf("pos:%d,%d,%x\n", pos, mask,*cr);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_EnableWakeupSrc(HPAON_WakeupSrcTypeDef src, AON_PinModeTypeDef mode)
{
    uint32_t mask;
    uint32_t val;
    uint32_t pos;
    uint32_t wer_en;
    __IO uint32_t *cr;
    HAL_StatusTypeDef ret;

    if ((src >= HPAON_WAKEUP_SRC_PIN0) && (src <= HPAON_WAKEUP_SRC_PIN_LAST))
    {
        ret = HAL_PMU_EnablePinWakeup(src - HPAON_WAKEUP_SRC_PIN0, mode);
    }
    else
    {
        hwp_hpsys_aon->WER |= (1UL << src);
        ret = HAL_OK;
    }

    return ret;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_HPAON_DisableWakeupSrc(HPAON_WakeupSrcTypeDef src)
{
    HAL_StatusTypeDef ret;

    if ((src >= HPAON_WAKEUP_SRC_PIN0) && (src <= HPAON_WAKEUP_SRC_PIN_LAST))
    {
        ret = HAL_PMU_DisablePinWakeup(src - HPAON_WAKEUP_SRC_PIN0);
    }
    else
    {
        hwp_hpsys_aon->WER &= ~(1UL << src);
    }

    return ret;
}

#endif /* HAL_AON_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */