/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "string.h"
#include "bf0_hal.h"
#include "math.h"


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup PMU PMU
  * @brief PMU HAL module driver
  * @{
  */

#ifdef HAL_PMU_MODULE_ENABLED

#define PMU_WAKEUP_PIN_NUM  PMUC_WSR_PIN_NUM

//TODO:
HAL_StatusTypeDef HAL_PMU_EnablePinWakeup2(pin_pad pad, uint8_t mode)
{
    return HAL_ERROR;

}

HAL_StatusTypeDef HAL_PMU_DisablePinWakeup2(pin_pad pin)
{
    return HAL_ERROR;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_EnablePinWakeup(uint8_t pin, uint8_t mode)
{
    uint32_t mask;
    uint32_t pos;
    uint32_t val;

    if ((pin >= PMU_WAKEUP_PIN_NUM) || (mode > 4))
    {
        return HAL_ERROR;
    }

    /* workaround: clear pin status as it could be set before WER is set to 1 */
    HAL_PMU_CLEAR_WSR(1UL << (PMUC_WCR_PA33_Pos + pin));

    pos = PMUC_WKUP_MODE_PA33_MODE_Pos + pin * (PMUC_WKUP_MODE_PA34_MODE_Pos - PMUC_WKUP_MODE_PA33_MODE_Pos);
    mask = (PMUC_WKUP_MODE_PA33_MODE_Msk << (pos - PMUC_WKUP_MODE_PA33_MODE_Pos));
    val = MAKE_REG_VAL(mode, mask, pos);

    MODIFY_REG(hwp_pmuc->WKUP_MODE, mask, val);
    mask = PMUC_WER_PA33 << pin;
    hwp_pmuc->WER |= mask;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_DisablePinWakeup(uint8_t pin)
{
    uint32_t mask;

    if (pin >= PMU_WAKEUP_PIN_NUM)
    {
        return HAL_ERROR;
    }

    mask = PMUC_WER_PA33 << pin;
    hwp_pmuc->WER &= ~mask;

    return HAL_OK;
}


HAL_RAM_RET_CODE_SECT(HAL_PMU_ConfigPeriLdo, HAL_StatusTypeDef HAL_PMU_ConfigPeriLdo(PMU_PeriLdoTypeDef ldo, bool en, bool wait))
{
    uint32_t mask;
    uint32_t val;

    if ((PMU_PERI_LDO_1V8 != ldo)
            && (PMU_PERI_LDO2_3V3 != ldo)
            && (PMU_PERI_LDO3_3V3 != ldo))
    {
        return HAL_ERROR;
    }

    /* in assumption that they have same relative offset */
    mask = ((PMUC_PERI_LDO_EN_LDO18_Msk << (ldo - PMU_PERI_LDO_1V8))
            | (PMUC_PERI_LDO_LDO18_PD_VOUT_Msk << (ldo - PMU_PERI_LDO_1V8)));

    if (en)
    {
        val = PMUC_PERI_LDO_EN_LDO18_Msk;
    }
    else
    {
        val = PMUC_PERI_LDO_LDO18_PD_VOUT_Msk;
    }

    val = val << (ldo - PMU_PERI_LDO_1V8);

    if (PMU_PERI_LDO_1V8 == ldo)
    {
        /* TODO: raise LDO_1V8 voltage, need to use calibration data later */
        mask |= PMUC_PERI_LDO_LDO18_VREF_SEL_Msk;
        val |= MAKE_REG_VAL2(0xE, PMUC_PERI_LDO_LDO18_VREF_SEL);
    }

    MODIFY_REG(hwp_pmuc->PERI_LDO, mask, val);

    if (wait)
    {
        HAL_Delay_us(1000);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_LpCLockSelect(PMU_LpClockTypeDef lp_clock)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (PMU_LPCLK_RC10 == lp_clock)
    {
        hwp_pmuc->CR &= ~PMUC_CR_SEL_WDT;
        ret = HAL_OK;
    }
    else
    {
        // switch between RC32K and RC10K
        if (PMU_LPCLK_RC32 == lp_clock)
        {
            ret = HAL_PMU_RC32KReady();
            if (ret == HAL_ERROR)
            {
                HAL_ASSERT(0);
                return ret;
            }
        }
        hwp_pmuc->CR |= PMUC_CR_SEL_WDT;
        ret = HAL_OK;
    }
    return ret;
}

#endif /* HAL_PMU_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
