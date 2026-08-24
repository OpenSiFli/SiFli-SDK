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

#ifdef SOC_BF0_HCPU
typedef struct
{
    bool init;
    FACTORY_CFG_VBK_LDO_T data;
} PMU_CalDataTypeDef;

HAL_RETM_BSS_SECT(pmu_cal_data, static PMU_CalDataTypeDef pmu_cal_data);

#endif /* SOC_BF0_HCPU */


#ifdef SOC_BF0_HCPU
void HAL_PMU_LoadCalData(void)
{
    int ret;

    ret = BSP_CONFIG_get(FACTORY_CFG_ID_VBUCK, (uint8_t *)&pmu_cal_data.data, sizeof(pmu_cal_data.data));

    if (ret > 0)
    {
        pmu_cal_data.init = true;

        /* Buck */
        MODIFY_REG(hwp_pmuc->BUCK_CR1, PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Msk | PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Msk,
                   MAKE_REG_VAL(pmu_cal_data.data.buck_vos_polar, PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Msk, PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Pos)
                   | MAKE_REG_VAL(pmu_cal_data.data.buck_vos_trim, PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Msk, PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Pos));

        /* LPSYS LDO Config for D mode */
        MODIFY_REG(hwp_pmuc->LPSYS_VOUT, PMUC_LPSYS_VOUT_VOUT_Msk,
                   MAKE_REG_VAL(pmu_cal_data.data.lpsys_ldo_vout, PMUC_LPSYS_VOUT_VOUT_Msk, PMUC_LPSYS_VOUT_VOUT_Pos));

        /* VRET */
        MODIFY_REG(hwp_pmuc->VRET_CR, PMUC_VRET_CR_TRIM_Msk,
                   MAKE_REG_VAL(pmu_cal_data.data.vret_trim, PMUC_VRET_CR_TRIM_Msk, PMUC_VRET_CR_TRIM_Pos));

        /* PERI LDO */
        MODIFY_REG(hwp_pmuc->PERI_LDO, PMUC_PERI_LDO_LDO18_VREF_SEL_Msk | PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk | PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk,
                   MAKE_REG_VAL(pmu_cal_data.data.ldo18_vref_sel, PMUC_PERI_LDO_LDO18_VREF_SEL_Msk, PMUC_PERI_LDO_LDO18_VREF_SEL_Pos)
                   | MAKE_REG_VAL(pmu_cal_data.data.vdd33_ldo2_vout, PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk, PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Pos)
                   | MAKE_REG_VAL(pmu_cal_data.data.vdd33_ldo3_vout, PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk, PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Pos));

        /* AON BG */
        MODIFY_REG(hwp_pmuc->AON_BG, PMUC_AON_BG_BUF_VOS_POLAR_Msk | PMUC_AON_BG_BUF_VOS_TRIM_Msk,
                   MAKE_REG_VAL(pmu_cal_data.data.aon_vos_polar, PMUC_AON_BG_BUF_VOS_POLAR_Msk, PMUC_AON_BG_BUF_VOS_POLAR_Pos)
                   | MAKE_REG_VAL(pmu_cal_data.data.aon_vos_trim, PMUC_AON_BG_BUF_VOS_TRIM_Msk, PMUC_AON_BG_BUF_VOS_TRIM_Pos));
    }
}

HAL_RAM_RET_CODE_SECT(HAL_PMU_GetHpsysVoutRef, HAL_StatusTypeDef HAL_PMU_GetHpsysVoutRef(uint8_t *vout_ref))
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (pmu_cal_data.init && vout_ref)
    {
        *vout_ref = pmu_cal_data.data.hpsys_ldo_vout;
        ret = HAL_OK;
    }

    return ret;
}

HAL_RAM_RET_CODE_SECT(HAL_PMU_GetHpsysVoutRef2, HAL_StatusTypeDef HAL_PMU_GetHpsysVoutRef2(uint8_t *vout_ref))
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (pmu_cal_data.init && vout_ref)
    {
        *vout_ref = pmu_cal_data.data.hpsys_ldo_vout2;
        ret = HAL_OK;
    }

    return ret;
}

#endif /* SOC_BF0_HCPU */


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


#ifdef SOC_BF0_HCPU
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

    if ((PMU_PERI_LDO_1V8 == ldo) && !pmu_cal_data.init)
    {
        /* raise LDO_1V8 voltage in case no calibration data is available */
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
#endif /* SOC_BF0_HCPU */

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
