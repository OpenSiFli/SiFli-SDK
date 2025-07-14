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

#ifdef SF32LB52X

    #define PMU_CHG_CAL_TARGET_VOLT (4200)

#endif /* SF32LB52X */

#ifdef SF32LB55X
    #define PMU_WAKEUP_PIN_NUM  LPSYS_AON_WSR_PIN_NUM
#else
    #define PMU_WAKEUP_PIN_NUM  (2)
#endif /* SF32LB55X */

#ifdef SF32LB58X
    #define PMU_WAKEUP_PIN_SEL_NUM (16)
#else
    #define PMU_WAKEUP_PIN_SEL_NUM (LPSYS_AON_WSR_PIN_NUM)
#endif /* SF32LB58X */

#ifdef SF32LB52X
#define PMU_CHG_RESET()  \
    do   \
    {    \
        hwp_pmuc->CHG_CR3 |= PMUC_CHG_CR3_FORCE_RST;  \
        hwp_pmuc->CHG_CR3 &= ~PMUC_CHG_CR3_FORCE_RST; \
    }    \
    while (0)

#endif /* SF32LB52X */


#if (defined(SF32LB52X) || defined(SF32LB56X)) && defined(SOC_BF0_HCPU)
typedef struct
{
    bool init;
    FACTORY_CFG_VBK_LDO_T data;
} PMU_CalDataTypeDef;

HAL_RETM_BSS_SECT(pmu_cal_data, static PMU_CalDataTypeDef pmu_cal_data);
#endif /* (SF32LB52X || SF32LB56X) && SOC_BF0_HCPU*/

#ifdef PMUC_CR_PIN0_SEL
__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_SelectWakeupPin(uint8_t pin, uint8_t aon_wakeup_pin)
{
    uint32_t mask;
    uint32_t pos;
    uint32_t val;

    if ((pin >= PMU_WAKEUP_PIN_NUM) || (aon_wakeup_pin >= PMU_WAKEUP_PIN_SEL_NUM))
    {
        return HAL_ERROR;
    }

    pos = PMUC_CR_PIN0_SEL_Pos + pin * (PMUC_CR_PIN1_SEL_Pos - PMUC_CR_PIN0_SEL_Pos);
    mask = (PMUC_CR_PIN0_SEL_Msk << (pos - PMUC_CR_PIN0_SEL_Pos));
    val = MAKE_REG_VAL(aon_wakeup_pin, mask, pos);

    MODIFY_REG(hwp_pmuc->CR, mask, val);

#if defined(SF32LB52X) && defined(hwp_pbr)
    /* PBR0~PBR3 reuse PAD_PA24~PAD_PA27 */
    if (HAL_PBR_MAX >= aon_wakeup_pin)
    {
        HAL_PBR_ConfigMode(aon_wakeup_pin, false);
    }
#endif /* SF32LB52X && hwp_pbr */

    return HAL_OK;
}
#endif /* PMUC_CR_PIN0_SEL */


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
    HAL_PMU_CLEAR_WSR(1UL << (PMUC_WCR_PIN0_Pos + pin));

    pos = PMUC_CR_PIN0_MODE_Pos + pin * (PMUC_CR_PIN1_MODE_Pos - PMUC_CR_PIN0_MODE_Pos);
    mask = (PMUC_CR_PIN0_MODE_Msk << (pos - PMUC_CR_PIN0_MODE_Pos));
    val = MAKE_REG_VAL(mode, mask, pos);

    MODIFY_REG(hwp_pmuc->CR, mask, val);
    mask = PMUC_WER_PIN0 << pin;
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

    mask = PMUC_WER_PIN0 << pin;
    hwp_pmuc->WER &= ~mask;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_EnableRtcWakeup()
{

    hwp_pmuc->WER |= PMUC_WER_RTC;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_DisableRtcWakeup()
{
    hwp_pmuc->WER &= ~PMUC_WER_RTC;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_RC10Kconfig(void)
{
#ifdef SF32LB52X
    MODIFY_REG(hwp_pmuc->LRC10_CR, PMUC_LRC10_CR_CMPBM1_Msk | PMUC_LRC10_CR_CMPBM2_Msk | PMUC_LRC10_CR_CHGCAP_Msk | PMUC_LRC10_CR_REFRES_Msk,
               MAKE_REG_VAL(3, PMUC_LRC10_CR_CMPBM1_Msk, PMUC_LRC10_CR_CMPBM1_Pos) |
               MAKE_REG_VAL(1, PMUC_LRC10_CR_CMPBM2_Msk, PMUC_LRC10_CR_CMPBM2_Pos) |
               MAKE_REG_VAL(1, PMUC_LRC10_CR_CHGCAP_Msk, PMUC_LRC10_CR_CHGCAP_Pos) |
               MAKE_REG_VAL(1, PMUC_LRC10_CR_REFRES_Msk, PMUC_LRC10_CR_REFRES_Pos));

    //CLEAR_BIT(hwp_pmuc->LRC10_CR, PMUC_LRC10_CR_CMPBM1_Msk | PMUC_LRC10_CR_CMPBM2_Msk | PMUC_LRC10_CR_CHGCAP_Msk | PMUC_LRC10_CR_REFRES_Msk);
    //SET_BIT(hwp_pmuc->LRC10_CR, (3 << PMUC_LRC10_CR_CMPBM1_Pos) | (1 << PMUC_LRC10_CR_CMPBM2_Pos) | (1 << PMUC_LRC10_CR_CHGCAP_Pos) | (1 << PMUC_LRC10_CR_REFRES_Pos));
#elif defined(SF32LB56X)
    MODIFY_REG(hwp_pmuc->LRC_CR, PMUC_LRC_CR_CMPBM1_Msk | PMUC_LRC_CR_CMPBM2_Msk | PMUC_LRC_CR_CHGCAP_Msk | PMUC_LRC_CR_REFRES_Msk,
               MAKE_REG_VAL(3, PMUC_LRC_CR_CMPBM1_Msk, PMUC_LRC_CR_CMPBM1_Pos) |
               MAKE_REG_VAL(1, PMUC_LRC_CR_CMPBM2_Msk, PMUC_LRC_CR_CMPBM2_Pos) |
               MAKE_REG_VAL(1, PMUC_LRC_CR_CHGCAP_Msk, PMUC_LRC_CR_CHGCAP_Pos) |
               MAKE_REG_VAL(1, PMUC_LRC_CR_REFRES_Msk, PMUC_LRC_CR_REFRES_Pos));


    //CLEAR_BIT(hwp_pmuc->LRC_CR, PMUC_LRC_CR_CMPBM1_Msk | PMUC_LRC_CR_CMPBM2_Msk | PMUC_LRC_CR_CHGCAP_Msk | PMUC_LRC_CR_REFRES_Msk);
    //SET_BIT(hwp_pmuc->LRC_CR, (3 << PMUC_LRC_CR_CMPBM1_Pos) | (1 << PMUC_LRC_CR_CMPBM2_Pos) | (1 << PMUC_LRC_CR_CHGCAP_Pos) | (1 << PMUC_LRC_CR_REFRES_Pos));
#else
    return HAL_ERROR;
#endif
    // Modify REFRES need 1ms delay
    HAL_Delay(1);
    return HAL_OK;
}



#ifdef SF32LB52X
    HAL_RAM_RET_CODE_SECT(HAL_PMU_EnterHibernate,  __HAL_ROM_USED void HAL_PMU_EnterHibernate(void))
#else
    __HAL_ROM_USED void HAL_PMU_EnterHibernate(void)
#endif
{
    HAL_DisableInterrupt();

#ifdef SOC_BF0_HCPU
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_RCC_Reset_and_Halt_LCPU(1);
#endif

#ifdef SF32LB52X
    /* reduce VBAT_LDO output voltage to 3V to avoid leakage current if VCC is lower than 3.3V */
    MODIFY_REG(hwp_pmuc->AON_LDO, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk,
               MAKE_REG_VAL(0, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos));

    /* turn off LDO2 to ensure flash changing to 3byte address mode when boot up */
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, 0, 1);
    HAL_Delay_us_(8000);
    /* Change RC48 to default frequency */
    if (RCC_SYSCLK_HRC48 == HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS))
    {
        HAL_HPAON_EnableXT48();
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    }

    hwp_pmuc->HRC_CR = 0xA50C015;
    hwp_rtc->BKP0R = 0x5050;
#endif /* SF32LB52X */

    /* clear WSR as it's not cleared if triggered in sleep */
    HAL_PMU_CLEAR_WSR(hwp_pmuc->WSR);

    /* clear and delay 100ms to make sure the pulse can be detected */
    hwp_pmuc->CR &= ~PMUC_CR_HIBER_EN;
    HAL_Delay(100);
    hwp_pmuc->CR |= PMUC_CR_HIBER_EN;
    /* while loop until system is down */
    while (1) {};
}

#ifdef SF32LB52X
    HAL_RAM_RET_CODE_SECT(HAL_PMU_EnterShutdown, __HAL_ROM_USED void HAL_PMU_EnterShutdown(void))
#else
    __HAL_ROM_USED void HAL_PMU_EnterShutdown(void)
#endif
{
    HAL_DisableInterrupt();

#ifdef SOC_BF0_HCPU
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_RCC_Reset_and_Halt_LCPU(1);
#endif

#ifdef SF32LB52X
    /* reduce VBAT_LDO output voltage to 3V to avoid leakage current if VCC is lower than 3.3V */
    MODIFY_REG(hwp_pmuc->AON_LDO, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk,
               MAKE_REG_VAL(0, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos));

    /* turn off LDO2 to ensure flash changing to 3byte address mode when boot up */
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, 0, 1);
    HAL_Delay_us_(8000);
    /* Change RC48 to default frequency */
    if (RCC_SYSCLK_HRC48 == HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS))
    {
        HAL_HPAON_EnableXT48();
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    }

    hwp_pmuc->HRC_CR = 0xA50C015;
    hwp_rtc->BKP0R = 0x5050;
#endif /* SF32LB52X */

    /* clear WSR as it's not cleared if triggered in sleep */
    HAL_PMU_CLEAR_WSR(hwp_pmuc->WSR);

    HAL_PMU_LpCLockSelect(PMU_LPCLK_RC10);
    /* make sure clock has switched to RC10 */
    HAL_Delay_us(100);
    HAL_PMU_DisableXTAL32();
    /* disable RTC to save power */
    hwp_rtc->ISR |= RTC_ISR_INIT;

    /* clear and delay 100ms to make sure the pulse can be detected */
    hwp_pmuc->CR &= ~PMUC_CR_HIBER_EN;
    HAL_Delay(100);
    hwp_pmuc->CR |= PMUC_CR_HIBER_EN;
    /* while loop until system is down */
    while (1) {};
}

/**
 * @brief  Check PMU boot mode
 *
 * It should be called only once after boot. PMU status would be cleared afterwards.
 * @param[out] boot_mode boot mode
 * @param[out] wakeup_src wakeup source if boot mode is #PMU_HIBERNATE_BOOT or #PMU_SHUTDOWN_BOOT, see @ref PMU_WAKEUP_SRC
 *
 * @retval status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_CheckBootMode(PMU_BootModeTypeDef *boot_mode, uint32_t *wakeup_src)
{
    if (boot_mode && wakeup_src)
    {
        if (hwp_pmuc->CR & PMUC_CR_HIBER_EN)
        {
            if (hwp_pmuc->CR & PMUC_CR_SEL_LPCLK)
            {
                /* use LXT32, RTC can wakeup system on time */
                *boot_mode = PMU_HIBERNATE_BOOT;
            }
            else
            {
                /* use LRC10, RTC CANNOT wakeup system on time */
                *boot_mode = PMU_SHUTDOWN_BOOT;
                hwp_rtc->ISR &= ~RTC_ISR_INIT;
            }
            hwp_pmuc->CR &= ~PMUC_CR_HIBER_EN;
        }
        else
        {
            *boot_mode = PMU_COLD_BOOT;
            if (hwp_pmuc->CR & PMUC_CR_REBOOT)
            {
                *boot_mode = PMU_REBOOT_BOOT;
                hwp_pmuc->CR &= ~PMUC_CR_REBOOT;
            }
        }

        *wakeup_src = hwp_pmuc->WSR;
        if (*wakeup_src & PMUC_WSR_RTC)
        {
            /* clear RTC wakeup source */
            hwp_rtc->ISR &= ~RTC_ISR_ALRMF;
        }
        hwp_pmuc->WCR = *wakeup_src;
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }


}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_EnableXTAL32(void)
{
    uint32_t val = hwp_pmuc->LXT_CR;
    hwp_pmuc->LXT_CR &= ~(PMUC_LXT_CR_EN | PMUC_LXT_CR_RSN);     // Disable LXT

#ifdef BPS_V33_HAL
    //HAL_Delay_us(0);
    //HAL_Delay_us(10);

#ifdef SF32LB52X
#error "No need to enable V33 for 52x"
#endif /* SF32LB52X */

    // BUCK1 SETTING ?
#if defined(SF32LB56X)
    uint32_t buck1 = hwp_pmuc->BUCK1_CR1;
    buck1 &= ~(PMUC_BUCK1_CR1_BUCK1_CCH);
    buck1 |= (0xc << PMUC_BUCK1_CR1_BUCK1_CCH_Pos);
    hwp_pmuc->BUCK1_CR1 = buck1;
#else
    uint32_t buck1 = hwp_pmuc->BUCK1_CR;
    buck1 &= ~(PMUC_BUCK1_CR_BUCK1_CCH);
    buck1 |= (0xc << PMUC_BUCK1_CR_BUCK1_CCH_Pos);
    hwp_pmuc->BUCK1_CR = buck1;
#endif
    val &= ~(PMUC_LXT_CR_AMP_BM | PMUC_LXT_CR_BM | PMUC_LXT_CR_CAP_SEL);
    val |= ((3 << PMUC_LXT_CR_AMP_BM_Pos) | (3 << PMUC_LXT_CR_BM_Pos));
    val |= PMUC_LXT_CR_AMPCTRL_ENB ;
    hwp_pmuc->LXT_CR = val;
#else
    val &= ~(PMUC_LXT_CR_CAP_SEL);
    // TODO: Check ECO version to reduct BM from 0xF to 1?
    MODIFY_REG(val, PMUC_LXT_CR_BM_Msk, MAKE_REG_VAL(0x2, PMUC_LXT_CR_BM_Msk, PMUC_LXT_CR_BM_Pos));   // Increase current
    MODIFY_REG(val, PMUC_LXT_CR_AMP_BM_Msk, MAKE_REG_VAL(0x3, PMUC_LXT_CR_AMP_BM_Msk, PMUC_LXT_CR_AMP_BM_Pos));   // Increase current 10na
#endif

    HAL_PIN_SetXT32();

    val |= (PMUC_LXT_CR_EN | PMUC_LXT_CR_RSN);                // Enable again
    hwp_pmuc->LXT_CR = val;

    return HAL_OK;
}

#if defined(SF32LB58X) || defined(SF32LB56X) || defined(SF32LB52X)
__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_EnableAudio(int enable)
{
    if (enable)
        hwp_pmuc->HXT_CR1 |= PMUC_HXT_CR1_BUF_AUD_EN;
    else
        hwp_pmuc->HXT_CR1 &= (~PMUC_HXT_CR1_BUF_AUD_EN);
    return HAL_OK;
}
#endif

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_EnableDLL(int enable)
{
    if (enable)
        hwp_pmuc->HXT_CR1 |= PMUC_HXT_CR1_BUF_DLL_EN;
    else
        hwp_pmuc->HXT_CR1 &= (~PMUC_HXT_CR1_BUF_DLL_EN);
    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_DisableXTAL32(void)
{
    hwp_pmuc->LXT_CR &= ~(PMUC_LXT_CR_EN | PMUC_LXT_CR_RSN);

    return HAL_OK;
}

#define MAX_LXT_READY_DELAY_MS      1000
__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_LXTReady()
{
    HAL_StatusTypeDef ret = HAL_OK;

#ifdef FPGA
    //rdy signal from analog FPGA needn't check
    return ret;
#endif

    if (0 == (hwp_pmuc->LXT_CR & PMUC_LXT_CR_RDY))
    {
#if 0
        // Wait for at most around 1s (if current is RC10K,freq 10000)
        uint32_t start = HAL_GTIMER_READ();
        while (0 == (hwp_pmuc->LXT_CR & PMUC_LXT_CR_RDY) && HAL_GTIMER_Check(start, 20000) == HAL_OK);
        if (0 == (hwp_pmuc->LXT_CR & PMUC_LXT_CR_RDY))
            ret = HAL_ERROR;
#else
        uint32_t count = 0;
        HAL_Delay_us_(0);

        while (0 == (hwp_pmuc->LXT_CR & PMUC_LXT_CR_RDY))
        {
            count ++;
            HAL_Delay_us_(1000);
            if (count >= MAX_LXT_READY_DELAY_MS)
                break;
        }
        if (0 == (hwp_pmuc->LXT_CR & PMUC_LXT_CR_RDY))
            ret = HAL_ERROR;
#endif
    }
    return ret;
}

/**
 * @brief  Enable RC32K
 *
 * @param[in] enable 1: enable RC32K ,  0: disable RC32K
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_EnableRC32K(int enable)
{

#ifdef SF32LB52X
    if (enable)
    {
        hwp_pmuc->LRC32_CR |= PMUC_LRC32_CR_EN;
    }
    else
    {
        hwp_pmuc->LRC32_CR &= ~PMUC_LRC32_CR_EN;
    }
#endif

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_RC32KReady()
{
    HAL_StatusTypeDef ret = HAL_OK;

#if defined(SF32LB52X)
    if (0 == (hwp_pmuc->LRC32_CR & PMUC_LRC32_CR_RDY))
    {
        // Wait for at most around 1s (if current is RC10K,freq 10000)
        uint32_t start = HAL_GTIMER_READ();
        while (0 == (hwp_pmuc->LRC32_CR & PMUC_LRC32_CR_RDY) && HAL_GTIMER_Check(start, 10000) == HAL_OK);
        if (0 == (hwp_pmuc->LRC32_CR & PMUC_LRC32_CR_RDY))
            ret = HAL_ERROR;
    }
#endif

    return ret;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_LpCLockSelect(PMU_LpClockTypeDef lp_clock)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (PMU_LPCLK_RC10 == lp_clock)
    {
        hwp_pmuc->CR &= ~PMUC_CR_SEL_LPCLK;
        ret = HAL_OK;
    }
    else
    {
#ifdef SF32LB52X
        // 54x switch between RC32K and RC10K
        if (PMU_LPCLK_RC32 == lp_clock)
        {
            ret = HAL_PMU_RC32KReady();
            if (ret == HAL_ERROR)
            {
                HAL_ASSERT(0);
                return ret;
            }
        }
#else
        // 55/56/58 switch between LXT32K and RC10K
        if (PMU_LPCLK_XT32 == lp_clock)
        {
            ret = HAL_PMU_LXTReady();
            if (ret == HAL_ERROR)
            {
                HAL_ASSERT(0);
                return ret;
            }
            MODIFY_REG(hwp_pmuc->LRC_CR, PMUC_LRC_CR_CHGCRT_Msk, MAKE_REG_VAL(3, PMUC_LRC_CR_CHGCRT_Msk, PMUC_LRC_CR_CHGCRT_Pos));
        }
#endif
        hwp_pmuc->CR |= PMUC_CR_SEL_LPCLK;
        ret = HAL_OK;
    }
    return ret;
}

#if !defined(SF32LB56X)&&!defined(SF32LB52X)
__HAL_ROM_USED HAL_StatusTypeDef HAL_PMU_EnableBuck2(void)
{
    if (0 == (hwp_pmuc->BUCK2_CR & PMUC_BUCK2_CR_BUCK2_EN))
    {
        hwp_pmuc->BUCK2_CR |= PMUC_BUCK2_CR_BUCK2_EN;
        while (0 == (hwp_pmuc->BUCK2_CR & PMUC_BUCK2_CR_BUCK2_RDY))
        {
        }
    }

#ifdef SF32LB55X
    hwp_hpsys_aon->ACR |= HPSYS_AON_ACR_EXTPWR_REQ;
#endif /* SF32LB55X */

    return HAL_OK;
}
#endif /* SF32LB56X */

#ifdef SF32LB52X
    HAL_RAM_RET_CODE_SECT(HAL_PMU_Reboot,  __HAL_ROM_USED void HAL_PMU_Reboot(void))
#else
    __HAL_ROM_USED void HAL_PMU_Reboot(void)
#endif
{
    HAL_DisableInterrupt();

#ifdef SOC_BF0_HCPU
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_RCC_Reset_and_Halt_LCPU(1);
#endif

#ifdef SF32LB52X
    /* turn off LDO2 to ensure flash changing to 3byte address mode when boot up */
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, 0, 1);
    HAL_Delay_us_(8000);

    /* Change RC48 to default frequency */
    if (RCC_SYSCLK_HRC48 == HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS))
    {
        HAL_HPAON_EnableXT48();
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    }

    hwp_pmuc->HRC_CR = 0xA50C015;
    hwp_rtc->BKP0R = 0x5050;

#endif

    if (hwp_pmuc->CR & PMUC_CR_REBOOT)
    {
        /* clear and delay 100ms to make sure the pulse can be detected */
        hwp_pmuc->CR &= ~PMUC_CR_REBOOT;

        /* Switch to HAL_Delay_us as in bootloader, systick is no enabled yet. */
#ifdef TARMAC
        HAL_Delay_us(200);
#else
        HAL_Delay_us(100000);
#endif

    }

#ifdef SF32LB56X
    /* workaround as XT32K cannot be used in hibernate */
    HAL_PMU_LpCLockSelect(PMU_LPCLK_RC10);
    /* make sure clock has switched to RC10 */
    HAL_Delay_us(100);
    HAL_PMU_DisableXTAL32();
#endif /* SF32LB56X */
    hwp_pmuc->CR |= PMUC_CR_REBOOT;
    /* while loop until system reboot */
    while (1) {};
}

HAL_RAM_RET_CODE_SECT(HAL_PMU_SET_HXT_CBANK, __HAL_ROM_USED void HAL_PMU_SET_HXT_CBANK(uint32_t value))
{
    MODIFY_REG(hwp_pmuc->HXT_CR1, PMUC_HXT_CR1_CBANK_SEL_Msk,
               MAKE_REG_VAL(value, PMUC_HXT_CR1_CBANK_SEL_Msk, PMUC_HXT_CR1_CBANK_SEL_Pos));
}

__HAL_ROM_USED void HAL_PMU_SetWdt(uint32_t instance)
{
#ifdef SF32LB55X
    if (instance == IWDT_BASE)
        hwp_pmuc->WER |= PMUC_WER_IWDT;
#else
    if (instance == WDT1_BASE)
        hwp_pmuc->WER |= PMUC_WER_WDT1;
    else if (instance == WDT2_BASE)
        hwp_pmuc->WER |= PMUC_WER_WDT2;
#endif /* SF32LB58X */
}

#ifdef SF32LB56X
void HAL_PMU_ConfigDBL96(bool enabled)
{
    static uint32_t acr_hxt48_req;
    if (enabled && (0 == (hwp_pmuc->DBL96_CR & PMUC_DBL96_CR_EN)))
    {
        /* Enable DBL96 */
        /* Record old config */
        acr_hxt48_req = hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_REQ;
        /* Enable XT48 */
        HAL_LPAON_EnableXT48();
        /* enable dbl96 */
        hwp_pmuc->DBL96_CR |= PMUC_DBL96_CR_EN | PMUC_DBL96_CR_OUT_EN | PMUC_DBL96_CR_TODIG_EN;
        /* set lp_ldo to 1.1V */
        MODIFY_REG(hwp_pmuc->LPSYS_LDO, PMUC_LPSYS_LDO_VREF_Msk,
                   MAKE_REG_VAL(0xb, PMUC_LPSYS_LDO_VREF_Msk, PMUC_LPSYS_LDO_VREF_Pos));
        HAL_Delay_us(100);
    }
    else if ((!enabled) && (hwp_pmuc->DBL96_CR & PMUC_DBL96_CR_EN))
    {
        /* Disable DBL96 */
        /* set lp_ldo to 0.9V */
        MODIFY_REG(hwp_pmuc->LPSYS_LDO, PMUC_LPSYS_LDO_VREF_Msk,
                   MAKE_REG_VAL(0x6, PMUC_LPSYS_LDO_VREF_Msk, PMUC_LPSYS_LDO_VREF_Pos));
        /* Disable dbl96 */
        hwp_pmuc->DBL96_CR &= ~PMUC_DBL96_CR_EN;
        if (0 == acr_hxt48_req)
        {
            HAL_LPAON_DisableXT48();
        }
    }
}
#endif /* SF32LB56X */

#ifdef SF32LB52X

HAL_StatusTypeDef HAL_PMU_ChgInit(PMU_ChgHandleTypeDef *handle, PMU_ChgCalParamTypeDef *cal_param)
{
    if (!handle || !cal_param)
    {
        return HAL_ERROR;
    }
    handle->cv_vctrl = cal_param->cv_vctrl;
    if (cal_param->vbat_step)
    {
        handle->vbat_step = cal_param->vbat_step * 0.25f;
    }
    else
    {
        handle->vbat_step = 20.0f;
    }

    MODIFY_REG(hwp_pmuc->CHG_CR1, PMUC_CHG_CR1_CC_MN_Msk | PMUC_CHG_CR1_CC_MP_Msk | PMUC_CHG_CR1_CV_VCTRL_Msk,
               MAKE_REG_VAL(cal_param->cc_mn, PMUC_CHG_CR1_CC_MN_Msk, PMUC_CHG_CR1_CC_MN_Pos)
               | MAKE_REG_VAL(cal_param->cc_mp, PMUC_CHG_CR1_CC_MP_Msk, PMUC_CHG_CR1_CC_MP_Pos)
               | MAKE_REG_VAL(cal_param->cv_vctrl, PMUC_CHG_CR1_CV_VCTRL_Msk, PMUC_CHG_CR1_CV_VCTRL_Pos));

    MODIFY_REG(hwp_pmuc->CHG_CR2, PMUC_CHG_CR2_BG_PROG_V1P2_Msk | PMUC_CHG_CR2_REP_VCTRL_Msk,
               MAKE_REG_VAL(cal_param->rep_vctrl, PMUC_CHG_CR2_REP_VCTRL_Msk, PMUC_CHG_CR2_REP_VCTRL_Pos)
               | MAKE_REG_VAL(cal_param->bg, PMUC_CHG_CR2_BG_PROG_V1P2_Msk, PMUC_CHG_CR2_BG_PROG_V1P2_Pos));

    /* reset charger, so it would use the new target voltage to check EOC immediately */
    PMU_CHG_RESET();

    handle->state = HAL_PMU_CHG_STATE_READY;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PMU_ChgEnable(PMU_ChgHandleTypeDef *handle, bool en)
{
    if (!handle)
    {
        return HAL_ERROR;
    }

    if (en)
    {
        hwp_pmuc->CHG_CR3 &= ~PMUC_CHG_CR3_FORCE_RST;
        handle->state = HAL_PMU_CHG_STATE_READY;
    }
    else
    {

        hwp_pmuc->CHG_CR3 |= PMUC_CHG_CR3_FORCE_RST;
        handle->state = HAL_PMU_CHG_STATE_RESET;
    }

    return HAL_OK;
}

/*
    cc_ictrl    current(mA)
    0x0         5
    0x1         10
    0x2         15
    0x3         20
    0x4         25
    0x5         30
    0x6         35
    0x7         40
    0x8         45
    0x9         50
    0xA         55
    0xB         60
    0xC         65
    0xD         70
    0xE         75
    0xF         80
    0x10        90
    0x11        100
    0x12        110
    0x13        120
    0x14        130
    0x15        140
    0x16        150
    0x17        160
    0x18        170
    0x19        180
    0x1A        190
    0x1B        200
    0x1C        210
    0x1D        220
    0x1E        230
    0x1F        240
    0x20        260
    0x21        280
    0x22        300
    0x23        320
    0x24        340
    0x25        360
    0x26        380
    0x27        400
    0x28        420
    0x29        440
    0x2A        460
    0x2B        480
    0x2C        500
    0x2D        520
    0x2E        540
    0x2F        560
*/
uint32_t HAL_PMU_ChgConfigCcCurrent(PMU_ChgHandleTypeDef *handle, uint32_t current)
{
    uint16_t cc_ictrl;
    uint16_t step;
    uint16_t cc_ictrl_base;
    uint16_t current_base;

    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        current = 0;
        goto __EXIT;
    }

    if (0 == current)
    {
        goto __EXIT;
    }

    /* aligned to upper level */
    if (current <= 80)
    {
        step = 5;
        cc_ictrl_base = 0;
        current_base = 0;
    }
    else if (current <= 240)
    {
        step = 10;
        cc_ictrl_base = 0x10;
        current_base = 80;
    }
    else if (current <= 560)
    {
        step = 20;
        cc_ictrl_base = 0x20;
        current_base = 240;
    }
    else
    {
        current = 0;
        goto __EXIT;
    }

    /* up aligned */
    cc_ictrl = (current - current_base + step - 1 - step) / step + cc_ictrl_base;
    current = (cc_ictrl - cc_ictrl_base + 1) * step + current_base;

    MODIFY_REG(hwp_pmuc->CHG_CR1, PMUC_CHG_CR1_CC_ICTRL_Msk,
               MAKE_REG_VAL(cc_ictrl, PMUC_CHG_CR1_CC_ICTRL_Msk, PMUC_CHG_CR1_CC_ICTRL_Pos));

__EXIT:
    return current;
}

/*
    precc_ictrl     current(mA)
    0x0             0.5
    0x1             1.0
    0x2             1.5
    0x3             2.0
    0x4             2.5
    0x5             3.0
    0x6             3.5
    0x7             4.0
    0x8             4.5
    0x9             5.0
    0xA             5.5
    0xB             6.0
    0xC             6.5
    0xD             7.0
    0xE             7.5
    0xF             8.0
    0x10            9.0
    0x11            10
    0x12            11
    0x13            12
    0x14            13
    0x15            14
    0x16            15
    0x17            16
    0x18            17
    0x19            18
    0x1A            19
    0x1B            20
    0x1C            21
    0x1D            22
    0x1E            23
    0x1F            24
    0x20            26
    0x21            28
    0x22            30
    0x23            32
    0x24            34
    0x25            36
    0x26            38
    0x27            40
    0x28            42
    0x29            44
    0x2A            46
    0x2B            48
    0x2C            50
    0x2D            52
    0x2E            54
    0x2F            56
*/
float HAL_PMU_ChgConfigPreCcCurrent(PMU_ChgHandleTypeDef *handle, float current)
{
    uint16_t precc_ictrl;
    uint16_t step;
    uint16_t precc_ictrl_base;
    uint16_t current_base;
    uint16_t current_100x;

    current_100x = (uint16_t)(current * 100);

    if (0 == current_100x)
    {
        current = 0.0f;
        goto __EXIT;
    }

    /* aligned to upper level */
    if (current_100x <= 800)
    {
        step = 50;
        precc_ictrl_base = 0;
        current_base = 0;
    }
    else if (current_100x <= 2400)
    {
        step = 100;
        precc_ictrl_base = 0x10;
        current_base = 800;
    }
    else if (current_100x <= 5600)
    {
        step = 200;
        precc_ictrl_base = 0x20;
        current_base = 2400;
    }
    else
    {
        current = 0.0f;
        goto __EXIT;
    }

    /* up aligned */
    precc_ictrl = (current_100x - current_base + step - 1 - step) / step + precc_ictrl_base;
    current = ((precc_ictrl - precc_ictrl_base + 1) * step + current_base) / 100.0f;

    MODIFY_REG(hwp_pmuc->CHG_CR2, PMUC_CHG_CR2_PRECC_ICTRL_Msk,
               MAKE_REG_VAL(precc_ictrl, PMUC_CHG_CR2_PRECC_ICTRL_Msk, PMUC_CHG_CR2_PRECC_ICTRL_Pos));

__EXIT:
    return current;
}

uint32_t HAL_PMU_ChgConfigTargetVolt(PMU_ChgHandleTypeDef *handle, uint32_t volt_mv)
{
    uint16_t cv_vctrl;
    float step = handle->vbat_step;
    uint16_t delta;
    uint16_t max;

    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        volt_mv = 0;
        goto __EXIT;
    }

    if (volt_mv < PMU_CHG_CAL_TARGET_VOLT)
    {
        volt_mv = 0;
        goto __EXIT;
    }

    delta = (uint16_t)roundf(((float)volt_mv - (float)PMU_CHG_CAL_TARGET_VOLT) / step);
    cv_vctrl = handle->cv_vctrl + delta;
    max = GET_REG_VAL(PMUC_CHG_CR1_CV_VCTRL_Msk, PMUC_CHG_CR1_CV_VCTRL_Msk, PMUC_CHG_CR1_CV_VCTRL_Pos);
    if (cv_vctrl > max)
    {
        cv_vctrl = max;
    }

    MODIFY_REG(hwp_pmuc->CHG_CR1, PMUC_CHG_CR1_CV_VCTRL_Msk,
               MAKE_REG_VAL(cv_vctrl, PMUC_CHG_CR1_CV_VCTRL_Msk, PMUC_CHG_CR1_CV_VCTRL_Pos));
    volt_mv = (uint32_t)((float)PMU_CHG_CAL_TARGET_VOLT + (cv_vctrl - handle->cv_vctrl) * step);

    /* reset charger, so it would use the new target voltage to check EOC immediately */
    PMU_CHG_RESET();

__EXIT:
    return volt_mv;
}

uint32_t HAL_PMU_ChgConfigVbatHighVolt(PMU_ChgHandleTypeDef *handle, uint32_t volt_mv)
{
    uint16_t high_vctrl;
    float step = handle->vbat_step;
    uint16_t delta;
    uint16_t max;

    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        volt_mv = 0;
        goto __EXIT;
    }

    if (volt_mv < PMU_CHG_CAL_TARGET_VOLT)
    {
        volt_mv = 0;
        goto __EXIT;
    }

    delta = (uint16_t)roundf(((float)volt_mv - (float)PMU_CHG_CAL_TARGET_VOLT) / step);
    high_vctrl = handle->cv_vctrl + delta;
    max = GET_REG_VAL(PMUC_CHG_CR2_HIGH_VCTRL_Msk, PMUC_CHG_CR2_HIGH_VCTRL_Msk, PMUC_CHG_CR2_HIGH_VCTRL_Pos);
    if (high_vctrl > max)
    {
        high_vctrl = max;
    }

    MODIFY_REG(hwp_pmuc->CHG_CR2,  PMUC_CHG_CR2_HIGH_VCTRL_Msk,
               MAKE_REG_VAL(high_vctrl,  PMUC_CHG_CR2_HIGH_VCTRL_Msk,  PMUC_CHG_CR2_HIGH_VCTRL_Pos));
    volt_mv = (uint32_t)((float)PMU_CHG_CAL_TARGET_VOLT + (high_vctrl - handle->cv_vctrl) * step);

    /* reset charger, so it would use the new target voltage to check EOC immediately */
    PMU_CHG_RESET();
__EXIT:
    return volt_mv;
}


uint32_t HAL_PMU_ChgConfigRepVolt(PMU_ChgHandleTypeDef *handle, uint32_t volt_mv)
{
    uint16_t rep_vctrl;
    float step = handle->vbat_step;
    int16_t delta;
    uint16_t max;

    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        volt_mv = 0;
        goto __EXIT;
    }

    /* rep_vol = 4200mV + rep_vol_adj
     * rep_vol_adj = -40mV + delta * 20mV
     * rep_cvtrl = cv + delta
     */
    delta = (int16_t)roundf(((float)volt_mv - (float)PMU_CHG_CAL_TARGET_VOLT) / step);
    if (delta < -(int16_t)(handle->cv_vctrl))
    {
        delta = -handle->cv_vctrl;
    }
    rep_vctrl = handle->cv_vctrl + delta;
    max = GET_REG_VAL(PMUC_CHG_CR2_REP_VCTRL_Msk, PMUC_CHG_CR2_REP_VCTRL_Msk, PMUC_CHG_CR2_REP_VCTRL_Pos);
    if (rep_vctrl > max)
    {
        rep_vctrl = max;
        delta = (int16_t)rep_vctrl - (int16_t)handle->cv_vctrl;
    }
    MODIFY_REG(hwp_pmuc->CHG_CR2, PMUC_CHG_CR2_REP_VCTRL_Msk,
               MAKE_REG_VAL(rep_vctrl, PMUC_CHG_CR2_REP_VCTRL_Msk, PMUC_CHG_CR2_REP_VCTRL_Pos));
    volt_mv = (uint32_t)((float)PMU_CHG_CAL_TARGET_VOLT  + delta * step);
__EXIT:
    return volt_mv;
}

HAL_StatusTypeDef HAL_PMU_ChgConfigIRQ(PMU_ChgHandleTypeDef *handle, PMU_ChgIrqStatusTypeDef irq, PMU_ChgIrqTrigModeTypeDef trig_mode)
{
    uint32_t mask;
    uint32_t pos;
    uint32_t ie;

    if ((irq > PMU_CHG_IRQ_EOC) || (trig_mode > PMU_CHG_IRQ_TRIG_MODE_DISABLE))
    {
        return HAL_ERROR;
    }

    if (PMU_CHG_IRQ_EOC == irq)
    {
        ie = PMUC_CHG_CR4_IE_EOC_Msk;
    }
    else
    {
        ie = (1UL << irq);
    }
    if (PMU_CHG_IRQ_TRIG_MODE_DISABLE == trig_mode)
    {
        hwp_pmuc->CHG_CR4 &= ~ie;
    }
    else
    {
        hwp_pmuc->CHG_CR4 &= ~ie;
        pos = PMUC_CHG_CR4_IM_VBUS_RDY_Pos + (irq - PMU_CHG_IRQ_VBUS_RDY) * (PMUC_CHG_CR4_IM_VBAT_HIGH_Pos - PMUC_CHG_CR4_IM_VBUS_RDY_Pos);
        mask = PMUC_CHG_CR4_IM_VBUS_RDY_Msk << (pos - PMUC_CHG_CR4_IM_VBUS_RDY_Pos);
        MODIFY_REG(hwp_pmuc->CHG_CR4, mask, MAKE_REG_VAL(trig_mode, mask, pos));
        hwp_pmuc->CHG_CR4 |= ie;
    }

    return HAL_OK;
}

void HAL_PMU_ChgRegisterCallback(PMU_ChgHandleTypeDef *handle, PMU_ChgCallbackTypeDef callback)
{
    handle->callback = callback;
}

HAL_StatusTypeDef HAL_PMU_IRQHandler(PMU_ChgHandleTypeDef *handle)
{
    uint32_t status;

    status = (hwp_pmuc->CHG_CR5 >> 16);
    hwp_pmuc->CHG_CR5 = status;
    if (status && handle && handle->callback)
    {
        if (status & PMUC_CHG_CR5_IC_EOC)
        {
            status &= ~PMUC_CHG_CR5_IC_EOC;
            status |= PMUC_CHG_CR5_IC_EOC_MODE;
        }
        handle->callback(handle, status);
    }

    return HAL_OK;
}

int8_t HAL_PMU_ChgReadStatus(PMU_ChgHandleTypeDef *handle, PMU_ChgIrqStatusTypeDef irq)
{
    uint32_t mask;
    uint32_t pos;

    pos = PMUC_CHG_SR_VBUS_RDY_OUT_Pos + (irq - PMU_CHG_IRQ_VBUS_RDY) * (PMUC_CHG_SR_VBAT_HIGH_OUT_Pos - PMUC_CHG_SR_VBUS_RDY_OUT_Pos);
    mask = PMUC_CHG_SR_VBUS_RDY_OUT_Msk << (pos - PMUC_CHG_SR_VBUS_RDY_OUT_Pos);

    return GET_REG_VAL(hwp_pmuc->CHG_SR, mask, pos);
}

HAL_StatusTypeDef HAL_PMU_ChgStartForceCharging(PMU_ChgHandleTypeDef *handle)
{
    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        goto __EXIT;
    }

    hwp_pmuc->CHG_CR1 |= PMUC_CHG_CR1_EN | PMUC_CHG_CR1_LOOP_EN;
    hwp_pmuc->CHG_CR3 |= PMUC_CHG_CR3_FORCE_CTRL;

    return HAL_OK;

__EXIT:
    return HAL_ERROR;

}

HAL_StatusTypeDef HAL_PMU_ChgSuspendForceCharging(PMU_ChgHandleTypeDef *handle)
{
    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        return HAL_ERROR;
    }

    hwp_pmuc->CHG_CR1 &= ~PMUC_CHG_CR1_LOOP_EN;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PMU_ChgResumeForceCharging(PMU_ChgHandleTypeDef *handle)
{
    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        return HAL_ERROR;
    }

    hwp_pmuc->CHG_CR1 |= PMUC_CHG_CR1_LOOP_EN;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PMU_ChgStopForceCharging(PMU_ChgHandleTypeDef *handle)
{
    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        return HAL_ERROR;
    }

    hwp_pmuc->CHG_CR3 &= ~PMUC_CHG_CR3_FORCE_CTRL;
    hwp_pmuc->CHG_CR1 &= ~(PMUC_CHG_CR1_EN | PMUC_CHG_CR1_LOOP_EN);

    return HAL_OK;
}


HAL_PMU_ChgHwStateTypeDef HAL_PMU_ChgGetHwState(PMU_ChgHandleTypeDef *handle)
{
    return GET_REG_VAL(hwp_pmuc->CHG_SR, PMUC_CHG_SR_CHG_STATE_Msk, PMUC_CHG_SR_CHG_STATE_Pos);
}

uint8_t HAL_PMU_ChgConfigEocCc(PMU_ChgHandleTypeDef *handle, uint8_t percent)
{
    uint8_t step;
    uint8_t start;
    uint8_t range_eoc;
    uint8_t bm_eoc;

    if (!handle || (HAL_PMU_CHG_STATE_READY != handle->state))
    {
        percent = 0;
        goto __EXIT;
    }
    if ((percent > 36) || (percent < 4))
    {
        percent = 0;
        goto __EXIT;
    }

    /*
     * range_eoc, 0: 0.5x, 1: 1x
     * bm_eoc, range from 4%~18% (if range_eoc=0, step=2%)
     *         range from 8%~36% (if range_eoc=1, step=4s%)
     *
     */
    if (percent >= 20)
    {
        step = 4;
        start = 8;
        range_eoc = 1;
    }
    else
    {
        step = 2;
        start = 4;
        range_eoc = 0;
    }

    bm_eoc = (percent - start) / step;

    MODIFY_REG(hwp_pmuc->CHG_CR2, PMUC_CHG_CR2_BM_EOC_Msk | PMUC_CHG_CR2_RANGE_EOC_Msk,
               MAKE_REG_VAL(bm_eoc, PMUC_CHG_CR2_BM_EOC_Msk, PMUC_CHG_CR2_BM_EOC_Pos)
               | MAKE_REG_VAL(range_eoc, PMUC_CHG_CR2_RANGE_EOC_Msk, PMUC_CHG_CR2_RANGE_EOC_Pos));

    percent = bm_eoc * step + start;

__EXIT:
    return percent;
}


HAL_RAM_RET_CODE_SECT(HAL_PMU_ConfigPeriLdo, HAL_StatusTypeDef HAL_PMU_ConfigPeriLdo(PMU_PeriLdoTypeDef ldo, bool en, bool wait))
{
    uint32_t mask;
    uint32_t val;

    if ((PMU_PERI_LDO_1V8 != ldo) && SF32LB52X_LETTER_SERIES())
    {
        /* letter series doesn't support PERI_LDO V33 */
        return HAL_OK;
    }

#ifndef HAL_VDDSIP_LDO18_ENABLE
    if (SF32LB52X_LETTER_SERIES())
    {
        /* if VDDSIP is not supplied by interal LDO18, don't enable LDO18 */
        return HAL_OK;
    }
#endif /* !HAL_VDDSIP_LDO18_ENABLE */


    if ((PMU_PERI_LDO_1V8 != ldo)
            && (PMU_PERI_LDO2_3V3 != ldo)
            && (PMU_PERI_LDO3_3V3 != ldo))
    {
        return HAL_ERROR;
    }

    /* in assumption that they have same relative offset */
    mask = ((PMUC_PERI_LDO_EN_LDO18_Msk << (ldo - PMU_PERI_LDO_1V8))
            | (PMUC_PERI_LDO_LDO18_PD_Msk << (ldo - PMU_PERI_LDO_1V8)));

    if (en)
    {
        val = PMUC_PERI_LDO_EN_LDO18_Msk;
    }
    else
    {
        val = PMUC_PERI_LDO_LDO18_PD_Msk;
    }

    val = val << (ldo - PMU_PERI_LDO_1V8);

    MODIFY_REG(hwp_pmuc->PERI_LDO, mask, val);

    if (wait)
    {
        HAL_Delay_us(5000);
    }

    return HAL_OK;
}

#endif /* SF32LB52X */

__HAL_ROM_USED void HAL_PMU_Init(void)
{
#ifdef SF32LB56X
    MODIFY_REG(hwp_pmuc->BG1_CR, PMUC_BG1_CR_BG1_VREF_L_Msk | PMUC_BG1_CR_BG1_VREF_M_Msk,
               MAKE_REG_VAL(4, PMUC_BG1_CR_BG1_VREF_L_Msk, PMUC_BG1_CR_BG1_VREF_L_Pos)     /* vref_l=0.8V */
               | MAKE_REG_VAL(8, PMUC_BG1_CR_BG1_VREF_M_Msk, PMUC_BG1_CR_BG1_VREF_M_Pos)); /* vref_m=1.0V */

    hwp_pmuc->BUCK1_CR1 &= ~PMUC_BUCK1_CR1_BUCK1_ZCD_AON;
    hwp_pmuc->BUCK1_CR1 |= PMUC_BUCK1_CR1_BUCK1_H2M_EN | PMUC_BUCK1_CR1_BUCK1_M2L_EN
                           | PMUC_BUCK1_CR1_BUCK1_H2L_EN | PMUC_BUCK1_CR1_BUCK1_L2M_EN;

    /* to speedup deep sleep wakeup */
    MODIFY_REG(hwp_pmuc->BUCK1_CR2, PMUC_BUCK1_CR2_M2H_CNT_Msk | PMUC_BUCK1_CR2_L2M_CNT_Msk | PMUC_BUCK1_CR2_L2H_CNT_Msk,
               MAKE_REG_VAL(0, PMUC_BUCK1_CR2_M2H_CNT_Msk, PMUC_BUCK1_CR2_M2H_CNT_Pos)
               | MAKE_REG_VAL(0x0, PMUC_BUCK1_CR2_L2M_CNT_Msk, PMUC_BUCK1_CR2_L2M_CNT_Pos)
               | MAKE_REG_VAL(0x0, PMUC_BUCK1_CR2_L2H_CNT_Msk, PMUC_BUCK1_CR2_L2H_CNT_Pos));

    MODIFY_REG(hwp_pmuc->LPSYS_LDO, PMUC_LPSYS_LDO_VREF2_Msk,
               MAKE_REG_VAL(0, PMUC_LPSYS_LDO_VREF2_Msk, PMUC_LPSYS_LDO_VREF2_Pos));


    /* to speedup deep sleep wakeup */
    MODIFY_REG(hwp_pmuc->LPSYS_SWR, PMUC_LPSYS_SWR_DLY_Msk,
               MAKE_REG_VAL(1, PMUC_LPSYS_SWR_DLY_Msk, PMUC_LPSYS_SWR_DLY_Pos));

    /* XT48 ready flag is set after waiting for 2ms */
    if (HAL_PMU_LXT_ENABLED())
    {
        MODIFY_REG(hwp_pmuc->HXT_CR3, PMUC_HXT_CR3_DLY_Msk, MAKE_REG_VAL(0x3F, PMUC_HXT_CR3_DLY_Msk, PMUC_HXT_CR3_DLY_Pos));
    }
    else
    {
        MODIFY_REG(hwp_pmuc->HXT_CR3, PMUC_HXT_CR3_DLY_Msk, MAKE_REG_VAL(0x14, PMUC_HXT_CR3_DLY_Msk, PMUC_HXT_CR3_DLY_Pos));
    }

#elif defined(SF32LB52X)

    MODIFY_REG(hwp_pmuc->VRET_CR, PMUC_VRET_CR_VBIT_Msk,
               MAKE_REG_VAL(1, PMUC_VRET_CR_VBIT_Msk, PMUC_VRET_CR_VBIT_Pos));

    hwp_pmuc->BUCK_CR2 |= PMUC_BUCK_CR2_H2M_EN | PMUC_BUCK_CR2_M2L_EN
                          | PMUC_BUCK_CR2_H2L_EN | PMUC_BUCK_CR2_L2M_EN;

    /* speedup restore procedure */
    /* make sure buck is ready */
    MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_M2H_CNT_Msk | PMUC_BUCK_CR2_L2M_CNT_Msk | PMUC_BUCK_CR2_L2H_CNT_Msk,
               MAKE_REG_VAL(7, PMUC_BUCK_CR2_M2H_CNT_Msk, PMUC_BUCK_CR2_M2H_CNT_Pos)
               | MAKE_REG_VAL(7, PMUC_BUCK_CR2_L2M_CNT_Msk, PMUC_BUCK_CR2_L2M_CNT_Pos)
               | MAKE_REG_VAL(7, PMUC_BUCK_CR2_L2H_CNT_Msk, PMUC_BUCK_CR2_L2H_CNT_Pos));

    MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_L_Msk,
               MAKE_REG_VAL(6, PMUC_BUCK_CR2_SET_VOUT_L_Msk, PMUC_BUCK_CR2_SET_VOUT_L_Pos));

    /* set VBAT_LDO output voltage to default 3.3V */
    MODIFY_REG(hwp_pmuc->AON_LDO, PMUC_AON_LDO_VBAT_POR_TH_Msk | PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk,
               MAKE_REG_VAL(0, PMUC_AON_LDO_VBAT_POR_TH_Msk, PMUC_AON_LDO_VBAT_POR_TH_Pos)
               | MAKE_REG_VAL(6, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos));

    /* auto power down if VCC is low */
    hwp_pmuc->WER |= PMUC_WER_LOWBAT;

    /* speedup restore procedure */
    MODIFY_REG(hwp_pmuc->HPSYS_LDO, PMUC_HPSYS_LDO_DLY_Msk, MAKE_REG_VAL(1, PMUC_HPSYS_LDO_DLY_Msk, PMUC_HPSYS_LDO_DLY_Pos));
    MODIFY_REG(hwp_pmuc->LPSYS_LDO, PMUC_LPSYS_LDO_DLY_Msk, MAKE_REG_VAL(1, PMUC_LPSYS_LDO_DLY_Msk, PMUC_LPSYS_LDO_DLY_Pos));
    MODIFY_REG(hwp_pmuc->HPSYS_SWR, PMUC_HPSYS_SWR_DLY_Msk, MAKE_REG_VAL(3, PMUC_HPSYS_SWR_DLY_Msk, PMUC_HPSYS_SWR_DLY_Pos));
    MODIFY_REG(hwp_pmuc->LPSYS_SWR, PMUC_LPSYS_SWR_DLY_Msk, MAKE_REG_VAL(3, PMUC_LPSYS_SWR_DLY_Msk, PMUC_LPSYS_SWR_DLY_Pos));

    /* XT48 ready flag is set after waiting for 2ms, default 1ms is not enough, assume PMU always uses RC32K */
    MODIFY_REG(hwp_pmuc->HXT_CR3, PMUC_HXT_CR3_DLY_Msk, MAKE_REG_VAL(0x3F, PMUC_HXT_CR3_DLY_Msk, PMUC_HXT_CR3_DLY_Pos));

#else
    /* XT48 ready flag is set after waiting for 2ms*/
    if (HAL_PMU_LXT_ENABLED())
    {
        MODIFY_REG(hwp_pmuc->HXT_CR3, PMUC_HXT_CR3_DLY_Msk, MAKE_REG_VAL(0x3F, PMUC_HXT_CR3_DLY_Msk, PMUC_HXT_CR3_DLY_Pos));
    }
    else
    {
        MODIFY_REG(hwp_pmuc->HXT_CR3, PMUC_HXT_CR3_DLY_Msk, MAKE_REG_VAL(0x14, PMUC_HXT_CR3_DLY_Msk, PMUC_HXT_CR3_DLY_Pos));
    }
#endif /* SF32LB56X */

#ifdef PMUC_WKUP_CNT_PIN0_CNT_Pos
    hwp_pmuc->WKUP_CNT = 0x00010001;
#endif /* PMUC_WKUP_CNT_PIN0_CNT_Pos */

#ifdef SF32LB58X
    /* 58x also needs to enable buck2 when HCPU wakeup to avoid buck1 voltage drop */
    hwp_hpsys_aon->ACR |= HPSYS_AON_ACR_EXTPWR_REQ;
#endif /* SF32LB58X */
}

#if defined(SF32LB52X) && defined(SOC_BF0_HCPU)
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
        if (!SF32LB52X_LETTER_SERIES())
        {
            MODIFY_REG(hwp_pmuc->PERI_LDO, PMUC_PERI_LDO_LDO18_VREF_SEL_Msk | PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk | PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk,
                       MAKE_REG_VAL(pmu_cal_data.data.ldo18_vref_sel, PMUC_PERI_LDO_LDO18_VREF_SEL_Msk, PMUC_PERI_LDO_LDO18_VREF_SEL_Pos)
                       | MAKE_REG_VAL(pmu_cal_data.data.vdd33_ldo2_vout, PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk, PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Pos)
                       | MAKE_REG_VAL(pmu_cal_data.data.vdd33_ldo3_vout, PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk, PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Pos));
        }
#if !defined(PVDD_V18_ENABLE)
        else
        {
            MODIFY_REG(hwp_pmuc->PERI_LDO, PMUC_PERI_LDO_LDO18_VREF_SEL_Msk,
                       MAKE_REG_VAL(pmu_cal_data.data.ldo18_vref_sel, PMUC_PERI_LDO_LDO18_VREF_SEL_Msk, PMUC_PERI_LDO_LDO18_VREF_SEL_Pos));
        }
#endif /* PVDD_1V8_ENABLE */

        /* AON BG */
        if (!SF32LB52X_LETTER_SERIES())
        {
            MODIFY_REG(hwp_pmuc->AON_BG, PMUC_AON_BG_BUF_VOS_POLAR_Msk | PMUC_AON_BG_BUF_VOS_TRIM_Msk,
                       MAKE_REG_VAL(pmu_cal_data.data.aon_vos_polar, PMUC_AON_BG_BUF_VOS_POLAR_Msk, PMUC_AON_BG_BUF_VOS_POLAR_Pos)
                       | MAKE_REG_VAL(pmu_cal_data.data.aon_vos_trim, PMUC_AON_BG_BUF_VOS_TRIM_Msk, PMUC_AON_BG_BUF_VOS_TRIM_Pos));
        }
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


#endif /* SF32LB52X && SOC_BF0_HCPU */

#if defined(SF32LB56X) && defined(SOC_BF0_HCPU)
void HAL_PMU_SaveCalData(FACTORY_CFG_VBK_LDO_T *cfg)
{
    if (!cfg)
    {
        HAL_ASSERT(0);
        return;
    }

    pmu_cal_data.init = true;
    memcpy((void *)&pmu_cal_data.data, cfg, sizeof(pmu_cal_data.data));
}

HAL_RAM_RET_CODE_SECT(HAL_PMU_GetHpsysVoutRef, HAL_StatusTypeDef HAL_PMU_ConfigHpsysLdoVolt(PMU_HpsysLdoVoltTypeDef volt))
{
    uint8_t vref;
    uint8_t vref_max;

    if (pmu_cal_data.init)
    {
        vref = pmu_cal_data.data.hp_ldo;
    }
    else
    {
        vref = PMU_HPSYS_LDO_VREF_DEFAULT;
    }

    if (PMU_HPSYS_LDO_VOLT_HIGH == volt)
    {
        vref = vref + 2;
    }

    vref_max = (PMUC_HPSYS_LDO_VREF_Msk >> PMUC_HPSYS_LDO_VREF_Pos);
    if (vref > vref_max)
    {
        vref = vref_max;
    }
    MODIFY_REG(hwp_pmuc->HPSYS_LDO, PMUC_HPSYS_LDO_VREF_Msk, (vref << PMUC_HPSYS_LDO_VREF_Pos));

    return HAL_OK;
}

#endif /* SF32LB56X && SOC_BF0_HCPU */




#endif /* HAL_PMU_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */