/**
  ******************************************************************************
  * @file   bf0_hal_pmu.h
  * @author Sifli software development team
  * @brief   Header file of RCC HAL module.
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_PMU_H
#define __BF0_HAL_PMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup PMU PMU
  * @ingroup BF0_HAL_Driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PMU_Exported_Types PMU Exported Types
 * @{
 *
 * @brief PMU export types
 */

/** PMU boot mode */
typedef enum
{
    PMU_COLD_BOOT,  /**< code boot mode */
    PMU_HIBERNATE_BOOT,  /**< boot from hibernate mode, can be wakeup by RTC and PIN */
    PMU_SHUTDOWN_BOOT,   /**< boot from shutdown mode, can be wakeup by RTC and PIN, but RTC wakeup time is not precise  */
    PMU_REBOOT_BOOT     /**< boot from reboot */
} PMU_BootModeTypeDef;

typedef enum
{
    PMU_LPCLK_RC10,   /** RC10K as LP clock */
    PMU_LPCLK_XT32,   /** XTAL32K as LP clock */
    PMU_LPCLK_RC32,   /** RC32 as LP clock */
} PMU_LpClockTypeDef;

#ifdef SF32LB56X
/** HPSYS LDO voltage type */
typedef enum
{
    PMU_HPSYS_LDO_VOLT_NORMAL,
    PMU_HPSYS_LDO_VOLT_HIGH,
} PMU_HpsysLdoVoltTypeDef;
#endif /* SF32LB56X */


#ifdef SF32LB52X

/** Charger Calibration Parameters */
typedef struct
{
    uint8_t cc_mn;
    uint8_t cc_mp;
    uint8_t bg;
    uint8_t cv_vctrl;
    uint8_t rep_vctrl;
    /** 0.25mV one code */
    uint8_t vbat_step;
} PMU_ChgCalParamTypeDef;

/** Charger State Type */
typedef enum
{
    HAL_PMU_CHG_STATE_RESET             = 0x00U,
    HAL_PMU_CHG_STATE_READY             = 0x01U,
    HAL_PMU_CHG_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing             */
    HAL_PMU_CHG_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                              */
    HAL_PMU_CHG_STATE_ERROR             = 0x04U,    /*!< CHG state error                           */
    HAL_PMU_CHG_STATE_SUSPEND           = 0x05U     /*!< CHG process is suspended                  */
} HAL_PMU_ChgStateTypeDef;

/** Hardware state machine, must be consistent with the value of register CHG_SR.CHG_STATE */
typedef enum
{
    HAL_PMU_CHG_HW_ST_OFF    = 0x01,
    HAL_PMU_CHG_HW_ST_PWRUP  = 0x02,
    HAL_PMU_CHG_HW_ST_IDLE   = 0x04,
    HAL_PMU_CHG_HW_ST_PRECC  = 0x08,
    HAL_PMU_CHG_HW_ST_CC     = 0x10,
    HAL_PMU_CHG_HW_ST_CV     = 0x20,
    HAL_PMU_CHG_HW_ST_EOC    = 0x40,
} HAL_PMU_ChgHwStateTypeDef;

/** Charger Handle Type definition*/
typedef struct __PMU_ChgHandleTypeDef PMU_ChgHandleTypeDef;

/** Charger Callback Type Def */
typedef void (*PMU_ChgCallbackTypeDef)(PMU_ChgHandleTypeDef *handle, uint32_t status);

/** Charger Handle Structure Definition */
struct __PMU_ChgHandleTypeDef
{
    HAL_PMU_ChgStateTypeDef state;
    uint8_t cv_vctrl;
    float vbat_step;
    PMU_ChgCallbackTypeDef callback;
};

/** Charger IRQ Trigger Mode Type Definition */
typedef enum
{
    PMU_CHG_IRQ_TRIG_MODE_HIGH = 0,    /**< event status is high */
    PMU_CHG_IRQ_TRIG_MODE_LOW,         /**< event status is low */
    PMU_CHG_IRQ_TRIG_MODE_POS_EDGE,    /**< event status goes from low to high */
    PMU_CHG_IRQ_TRIG_MODE_NEG_EDGE,    /**< event status goes from high to low */
    PMU_CHG_IRQ_TRIG_MODE_DOUBLE_EDGE, /**< event status goes from high to low, or low to high */
    PMU_CHG_IRQ_TRIG_MODE_DISABLE,     /**< not trigger interrupt */
} PMU_ChgIrqTrigModeTypeDef;

/** Charger IRQ Status Type Definition */
typedef enum
{
    PMU_CHG_IRQ_VBUS_RDY   = 0,  /**< VBUS Ready */
    PMU_CHG_IRQ_VBAT_HIGH  = 1,  /**< VBAT High */
    PMU_CHG_IRQ_ABOVE_REP  = 2,  /**< Above Rep */
    PMU_CHG_IRQ_ABOVE_CC   = 3,  /**< Above CC */
    PMU_CHG_IRQ_CC         = 4,  /**< CC */
    PMU_CHG_IRQ_CV         = 5,  /**< CV */
    PMU_CHG_IRQ_EOC        = 6,  /**< End of Charging */
} PMU_ChgIrqStatusTypeDef;

/* PMU Peripheral LDO Type Definition */
typedef enum
{
    PMU_PERI_LDO_1V8      = PMUC_PERI_LDO_EN_LDO18_Pos,
    PMU_PERI_LDO2_3V3     = PMUC_PERI_LDO_EN_VDD33_LDO2_Pos,
    PMU_PERI_LDO3_3V3     = PMUC_PERI_LDO_EN_VDD33_LDO3_Pos,
} PMU_PeriLdoTypeDef;

#endif /* SF32LB52X */

///@} PMU_Exported_Types


#define PMU_HPSYS_LDO_VREF_DEFAULT   (0xB)

#define HAL_PMU_DISABLE_LPSYS_LDO()     do {hwp_pmuc->LDO_CR &= ~PMUC_LDO_CR_LPSYS_LDO_EN;} while (0)


#ifndef SF32LB55X
#define PMUC_WSR_PIN_ALL                (PMUC_WSR_PIN0 | PMUC_WSR_PIN1)
#else
#define PMUC_WSR_PIN_ALL                (PMUC_WSR_PIN0 | PMUC_WSR_PIN1 | PMUC_WSR_PIN2 \
                                             | PMUC_WSR_PIN3 | PMUC_WSR_PIN4 | PMUC_WSR_PIN5)
#endif /* SF32LB55X */

/** @defgroup PMU_LPSYS_PSW PMU_LPSYS_PSW
 * @{
 */

#define PMU_LPSYS_PSW_RET_LDO    (1UL << PMUC_LPSYS_SWR_PSW_Pos)
#define PMU_LPSYS_PSW_BUCK2      (2UL << PMUC_LPSYS_SWR_PSW_Pos)
#define PMU_LPSYS_PSW_LPSYS_LDO  (4UL << PMUC_LPSYS_SWR_PSW_Pos)

/**
 * @}
 */

/** @defgroup PMU_BLESYS_PSW PMU_BLESYS_PSW
 * @{
 */

#define PMU_BLESYS_PSW_RET_LDO    (1UL << PMUC_BLESYS_SWR_PSW_Pos)
#define PMU_BLESYS_PSW_BUCK2      (2UL << PMUC_BLESYS_SWR_PSW_Pos)
#define PMU_BLESYS_PSW_LPSYS_LDO  (4UL << PMUC_BLESYS_SWR_PSW_Pos)

/**
 * @}
 */


/** @defgroup PMU_WAKEUP_SRC PMU wakeup source
 * @{
 */

/** wakeup source RTC */
#define PMU_WAKEUP_SRC_RTC        PMUC_WSR_RTC
/** wakeup source PIN0 */
#define PMU_WAKEUP_SRC_PIN0       PMUC_WSR_PIN0
/** wakeup source PIN1 */
#define PMU_WAKEUP_SRC_PIN1       PMUC_WSR_PIN1
/** wakeup source PIN2 */
#define PMU_WAKEUP_SRC_PIN2       PMUC_WSR_PIN2
/** wakeup source PIN3 */
#define PMU_WAKEUP_SRC_PIN3       PMUC_WSR_PIN3
/** wakeup source PIN4 */
#define PMU_WAKEUP_SRC_PIN4       PMUC_WSR_PIN4
/** wakeup source PIN5 */
#define PMU_WAKEUP_SRC_PIN5       PMUC_WSR_PIN5

/**
 * @}
 */


/**
  * @brief  Select LPSYS Power Switch
  * @param[in] psw power switch flag, see @ref PMU_LPSYS_PSW
  * @retval None
  */
#define HAL_PMU_SELECT_LPSYS_PWR(psw)  \
            do \
            {  \
                MODIFY_REG(hwp_pmuc->LPSYS_SWR, PMUC_LPSYS_SWR_PSW_Msk, (psw));  \
            }  \
            while (0)

/**
* @brief  Select LPSYS Power Switch
* @param[in] psw power switch flag, see @ref PMU_LPSYS_PSW
* @retval None
*/
#define HAL_PMU_SELECT_BLESYS_PWR(psw)  \
            do \
            {  \
                MODIFY_REG(hwp_pmuc->BLESYS_SWR, PMUC_BLESYS_SWR_PSW_Msk, (psw));  \
            }  \
            while (0)

/**
 * @brief  Config bandgap1 VREF12
 * @param[in] val vandgap1 vref12 value
 * @retval None
 */
#define HAL_PMU_SET_BG1_VREF12(val) \
            do  \
            {   \
                MODIFY_REG(hwp_pmuc->BG1_CR, PMUC_BG1_CR_BG1_VREF12_Msk, \
                           MAKE_REG_VAL((val), PMUC_BG1_CR_BG1_VREF12_Msk, PMUC_BG1_CR_BG1_VREF12_Pos)); \
            } while (0)

/**
 * @brief  Config HPSYS LDO VREF
 * @param[in] val HPSYS LDO VREF value
 * @retval None
 */
#ifndef SF32LB56X
#define HAL_PMU_SET_HPSYS_LDO_VREF(val)  \
            do \
            {  \
                MODIFY_REG(hwp_pmuc->LDO_CR, PMUC_LDO_CR_HPSYS_LDO_VREF_Msk, \
                           MAKE_REG_VAL((val), PMUC_LDO_CR_HPSYS_LDO_VREF_Msk, PMUC_LDO_CR_HPSYS_LDO_VREF_Pos)); \
            }  \
            while (0)
#else
#define HAL_PMU_SET_HPSYS_LDO_VREF(val)  \
            do \
            {  \
                MODIFY_REG(hwp_pmuc->HPSYS_LDO, PMUC_HPSYS_LDO_VREF_Msk, \
                           MAKE_REG_VAL((val), PMUC_HPSYS_LDO_VREF_Msk, PMUC_HPSYS_LDO_VREF_Pos)); \
            }  \
            while (0)
#endif


/**
 * @brief  Config HPSYS LDO VREF2
 * @param[in] val HPSYS LDO VREF2 value
 * @retval None
 */
#if defined(SF32LB55X) || defined(SF32LB58X)
#define HAL_PMU_SET_HPSYS_LDO_VREF2(val)  \
            do \
            {  \
                MODIFY_REG(hwp_pmuc->LDO_CR, PMUC_LDO_CR_HPSYS_LDO_VREF2_Msk, \
                           MAKE_REG_VAL((val), PMUC_LDO_CR_HPSYS_LDO_VREF2_Msk, PMUC_LDO_CR_HPSYS_LDO_VREF2_Pos)); \
            }  \
            while (0)
#else
#define HAL_PMU_SET_HPSYS_LDO_VREF2(val)  \
            do \
            {  \
                MODIFY_REG(hwp_pmuc->HPSYS_LDO, PMUC_HPSYS_LDO_VREF2_Msk, \
                           MAKE_REG_VAL((val), PMUC_HPSYS_LDO_VREF2_Msk, PMUC_HPSYS_LDO_VREF2_Pos)); \
            }  \
            while (0)

#endif

/**
 * @brief  Config BUCK1 CCH
 * @param[in] val BUCK1 CCH value
 * @retval None
 */
#define HAL_PMU_SET_BUCK1_CCH(val)  \
            do \
            {  \
                MODIFY_REG(hwp_pmuc->BUCK1_CR, PMUC_BUCK1_CR_BUCK1_CCH_Msk, \
                           MAKE_REG_VAL((val), PMUC_BUCK1_CR_BUCK1_CCH_Msk, PMUC_BUCK1_CR_BUCK1_CCH_Pos)); \
            }  \
            while (0)

/**
 * @brief  Turn off HCPU power switch entirely during standby
 * @retval None
 */
#define HAL_PMU_HPSYS_POWER_OFF()  \
            do \
            {  \
                hwp_pmuc->HPSYS_SWR |=PMUC_HPSYS_SWR_NORET; \
            }  \
            while (0)

/**
 * @brief  Turn off HCPU power switch entirely during standby
 * @retval None
 */
#define HAL_PMU_HPSYS_POWER_ON()  \
            do \
            {  \
                hwp_pmuc->HPSYS_SWR &= ~PMUC_HPSYS_SWR_NORET_Msk; \
            }  \
            while (0)

/**
 * @brief  Vret switch to 0.7v when standby
 * @retval None
 */
#define HAL_PMU_SWITCH_VRET()  \
            do \
            {  \
                hwp_pmuc->VRET_CR =0x840e9c9d; \
            }  \
            while (0)


/**
 * @brief  Vret switch to 0.65v when standby
 * @retval None
 */
#define HAL_PMU_SWITCH_VRET_LOWER()  \
            do \
            {  \
                MODIFY_REG(hwp_pmuc->VRET_CR, PMUC_VRET_CR_VBIT_Msk, (1<<PMUC_VRET_CR_VBIT_Pos));   \
            }  \
            while (0)

/**
 * @brief Set the delay time of LDO ready
 * @retval None
 */
#define HAL_PMU_SET_LDO_RDY_DELAY(delay) \
            do \
            { \
                MODIFY_REG(hwp_pmuc->LDO_CR, PMUC_LDO_CR_LPSYS_LDO_DLY_Msk, ((delay)<<PMUC_LDO_CR_LPSYS_LDO_DLY_Pos));   \
            } \
            while (0)

/**
 * @brief  Set the delay time of xTal48M ready
 * @retval None
 */
#define HAL_PMU_SET_HXT3_RDY_DELAY(delay) \
            do \
            { \
                MODIFY_REG(hwp_pmuc->HXT_CR3, PMUC_HXT_CR3_DLY_Msk, ((delay)<<PMUC_HXT_CR3_DLY_Pos));   \
            } \
            while (0)


/**
 * @brief  Check weather LXT is enabled
 * @retval True if LXT enabled, False if using RC10k
 */
#define  HAL_PMU_LXT_ENABLED()   ((hwp_pmuc->CR&PMUC_CR_SEL_LPCLK)!=PMU_LPCLK_RC10)

/**
 * @brief  Check weather LXT is disabled
 * @retval True if using RC10k, False if using LXT
 */
#define  HAL_PMU_LXT_DISABLED()   ((hwp_pmuc->CR&PMUC_CR_SEL_LPCLK)!=PMU_LPCLK_XT32)

/**
 * @brief  Clear PMU wakeup source register
 * @param[in] wsr
 * @retval wsr wakeup source register value
 */
#define HAL_PMU_CLEAR_WSR(wsr)   (hwp_pmuc->WCR = (wsr))


/**
 * @brief  Get wakeup source
 * @retval wsr wakeup source register value
 */
#define HAL_PMU_GET_WSR()   (hwp_pmuc->WSR)

#ifdef SF32LB58X

/**
 * @brief  Configure Deep Sleep use BUCK2
 * @retval None
 */
#define HAL_PMU_LPSYS_DS_USE_BUCK2()                            \
            do                                                  \
            {                                                                                                      \
                MODIFY_REG(hwp_pmuc->LPSYS_SWR, PMUC_LPSYS_SWR_PSW_MEM_RET_Msk | PMUC_LPSYS_SWR_PSW_CORE_RET_Msk,  \
                           MAKE_REG_VAL(2, PMUC_LPSYS_SWR_PSW_MEM_RET_Msk, PMUC_LPSYS_SWR_PSW_MEM_RET_Pos)         \
                           | MAKE_REG_VAL(2, PMUC_LPSYS_SWR_PSW_CORE_RET_Msk, PMUC_LPSYS_SWR_PSW_CORE_RET_Pos));   \
            }                                                                                                      \
            while (0)

/**
 * @brief  Enable BUCK2 low power mode
 * @retval None
 */
#define HAL_PMU_ENABLE_BUCK2_LOW_PWR()                            \
            do                                                    \
            {                                                                                                   \
                MODIFY_REG(hwp_pmuc->BUCK2_CR, PMUC_BUCK2_CR_BUCK2_ZCD_AON_Msk,                                 \
                          MAKE_REG_VAL(0, PMUC_BUCK2_CR_BUCK2_ZCD_AON_Msk, PMUC_BUCK2_CR_BUCK2_ZCD_AON_Pos));   \
            }                                                                                                   \
            while (0)

/**
 * @brief  Configure BUCK2 to low voltage mode
 * @retval None
 */
#define HAL_PMU_SET_BUCK2_LOW_VOLTAGE()                           \
            do                                                    \
            {                                                                                                 \
                MODIFY_REG(hwp_pmuc->BG2_CR, PMUC_BG2_CR_BG2_VREF06_Msk | PMUC_BG2_CR_BG2_VREF12_Msk,         \
                           MAKE_REG_VAL(2, PMUC_BG2_CR_BG2_VREF06_Msk, PMUC_BG2_CR_BG2_VREF06_Pos)            \
                           | MAKE_REG_VAL(2, PMUC_BG2_CR_BG2_VREF12_Msk, PMUC_BG2_CR_BG2_VREF12_Pos));        \
            }                                                                                                 \
            while (0)

/**
 * @brief  Configure BUCK2 to high voltage mode
 * @retval None
 */
#define HAL_PMU_SET_BUCK2_HIGH_VOLTAGE()                           \
            do                                                     \
            {                                                                                                 \
                MODIFY_REG(hwp_pmuc->BG2_CR, PMUC_BG2_CR_BG2_VREF06_Msk | PMUC_BG2_CR_BG2_VREF12_Msk,         \
                           MAKE_REG_VAL(6, PMUC_BG2_CR_BG2_VREF06_Msk, PMUC_BG2_CR_BG2_VREF06_Pos)            \
                           | MAKE_REG_VAL(6, PMUC_BG2_CR_BG2_VREF12_Msk, PMUC_BG2_CR_BG2_VREF12_Pos));        \
            }                                                                                                 \
            while (0)


#else

#define HAL_PMU_SET_BUCK2_LOW_VOLTAGE()

#define HAL_PMU_SET_BUCK2_HIGH_VOLTAGE()


#endif /* SF32LB55X */

#define HAL_PMU_ENABLE_BOR()                                                \
            do                                                              \
            {                                                               \
                MODIFY_REG(hwp_pmuc->VRTC_CR, PMUC_VRTC_CR_BOR_VT_TRIM_Msk, \
                    MAKE_REG_VAL(0, PMUC_VRTC_CR_BOR_VT_TRIM_Msk, PMUC_VRTC_CR_BOR_VT_TRIM_Pos));   \
                hwp_pmuc->VRTC_CR |= PMUC_VRTC_CR_BOR_EN;                   \
            }                                                               \
            while (0)



#ifdef PMUC_CR_PIN0_SEL
/**
 * @brief  Select aon_wakeup_pin as wakeup pin for hibernate
 * @param  pin            pin number, range is 0~1
 * @param  aon_wakeup_pin aon wakeup pin number, its range differs from each chip,
                          sf32lb58x: 0~15, sf32lb56x: 0~13, sf32lb52x: 0~20
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_SelectWakeupPin(uint8_t pin, uint8_t aon_wakeup_pin);
#endif /* PMUC_CR_PIN0_SEL */


/**
 * @brief  Enable pin wakeup for hibernate
 * @param  pin pin number, its range differs from each chips,
               sf32lb55x: 0~5, others: 0~1
 * @param  mode pin wakeup mode, 0: high level, 1: low level, 2: positive edge, 3: negative edge
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_EnablePinWakeup(uint8_t pin, uint8_t mode);

/**
 * @brief  Disable pin wakeup for hibernate
 * @param  pin pin number, range: 0~5
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_DisablePinWakeup(uint8_t pin);

/**
 * @brief  Enable RTC wakeup for hibernate
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_EnableRtcWakeup(void);

/**
 * @brief  Disable RTC wakeup for hibernate
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_DisableRtcWakeup(void);

#ifdef PMUC_WER_CHG
__STATIC_INLINE HAL_StatusTypeDef HAL_PMU_EnableChgWakeup()
{

    hwp_pmuc->WER |= PMUC_WER_CHG;

    return HAL_OK;
}

__STATIC_INLINE HAL_StatusTypeDef HAL_PMU_DisableChgWakeup()
{
    hwp_pmuc->WER &= ~PMUC_WER_CHG;

    return HAL_OK;
}
#endif /* PMUC_WER_CHG */

/**
 * @brief  Make system enter hibernate mode
 *
 *  The function would wait for hibernation and not return. System can be woken up RTC and PIN precisely
 *
 * @retval void
 */
void HAL_PMU_EnterHibernate(void);


/**
 * @brief  Make system enter shutdown mode
 *
 *  The function would wait for shutdown and not return. System can be woken up RTC and PIN, but RTC wakeup time is not accurate.
 *
 * @retval void
 */
void HAL_PMU_EnterShutdown(void);




/**
 * @brief  Check PMU boot mode
 *
 * It should be called only once after boot. PMU status would be cleared afterwards.
 * @param[out] boot_mode boot mode
 * @param[out] wakeup_src wakeup source if boot mode is #PMU_HIBERNATE_BOOT or #PMU_SHUTDOWN_BOOT, see @ref PMU_WAKEUP_SRC
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_CheckBootMode(PMU_BootModeTypeDef *boot_mode, uint32_t *wakeup_src);

/**
 * @brief  Enable XTAL32K
 *
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_EnableXTAL32(void);

/**
 * @brief  Disable XTAL32K
 *
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_DisableXTAL32(void);


/**
 * @brief  Update RC10K configuration
 *
 * It should be called only once after boot. It will optimize RC10K performance for each chip.
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_RC10Kconfig(void);


/**
 * @brief  Check whether Low power crystal is ready
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_LXTReady(void);


/**
 * @brief  Enable RC32K
 *
 * @param[in] enable 1: enable RC32K ,  0: disable RC32K
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_EnableRC32K(int enable);

/**
 * @brief Check whether rc32k is ready
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_RC32KReady(void);

/**
 * @brief  Select low power clock
 *
 * @param[in] lp_clock low power clock source
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_LpCLockSelect(PMU_LpClockTypeDef lp_clock);

/**
 * @brief  Enable RC48M
 *
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_EnableRC48(void);


#ifndef SF32LB56X
/**
 * @brief  Enable BUCK2
 *
 * @retval status
 */

HAL_StatusTypeDef HAL_PMU_EnableBuck2(void);
#endif /* SF32LB56X */

/**
 * @brief  Enable DLL buf, need by all DLLs,
 *  need to turn off when go low power, but need to make sure all DLL is turned off
 * @param[in] enable 1: enable DLL ,  0: disable DLL
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_EnableDLL(int enable);

/**
 * @brief  Enable audio buf,
 *  need to turn off when go low power
 *
 * @param[in] enable 1: enable Audio ,  0: disable Audio
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_EnableAudio(int enable);

/**
 * @brief  Reboot system
 *
 *  Not support by Z0.
 *  The function would wait for reboot and not return.
 *  Need to call #HAL_PMU_CheckBootMode to clear reboot bit after reboot. Boot mode is cold boot for reboot.
 *
 * @retval void
 */
void HAL_PMU_Reboot(void);

/**
 * @brief  Set CABANK SEL after crystal calibration
 *
 * @param[in] value HXT CBANK register value
 *
 * @retval void
 */
void HAL_PMU_SET_HXT_CBANK(uint32_t value);

/**
 * @brief  Enable watchdog as reboot cause
 *
 * @param[in] instance Base address of watch dog
 *
 * @retval void
 */
void HAL_PMU_SetWdt(uint32_t instance);

#ifdef SF32LB56X
/**
 * @brief  Enable or disable DBL96
 *
 * the function should be put in RAM
 *
 * @param[in] enable true: enable dbl96, false: disable dbl96
 *
 * @retval void
 */
void HAL_PMU_ConfigDBL96(bool enabled);

/**
 * @brief Save PMU calibration data
 *
 *
 * @return void
 */
void HAL_PMU_SaveCalData(FACTORY_CFG_VBK_LDO_T *cfg);

/**
 * @brief Config HPSYS LDO Voltage
 *
 *
 * @return status
 */
HAL_StatusTypeDef HAL_PMU_ConfigHpsysLdoVolt(PMU_HpsysLdoVoltTypeDef volt);
#endif /* SF32LB56X */

#ifdef SF32LB52X

/**
 * @brief Init charger
 *
 * This function shoud be called before other APIs
 *
 * @param[in] handle charger handle
 * @param[in] cal_param calibration param
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_ChgInit(PMU_ChgHandleTypeDef *handle, PMU_ChgCalParamTypeDef *cal_param);

/**
 * @brief Enable or disable charger
 *
 *
 * @param[in] handle charger handle
 * @param[in] en     true: enable charger, false: disable charger
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_ChgEnable(PMU_ChgHandleTypeDef *handle, bool en);

/**
 * @brief Configure charger CC current
 *
 *
 * @param[in] handle charger handle
 * @param[in] current CC current in mA
 *
 * @retval actual CC current configured to charger
 */
uint32_t HAL_PMU_ChgConfigCcCurrent(PMU_ChgHandleTypeDef *handle, uint32_t current);

/**
 * @brief Configure PRECC current
 *
 *
 * @param[in] handle charger handle
 * @param[in] current PRECC current in mA
 *
 * @retval actual PRECC current configured to charger
 */
float HAL_PMU_ChgConfigPreCcCurrent(PMU_ChgHandleTypeDef *handle, float current);

/**
 * @brief Configure charging full target voltage
 *
 *
 * @param[in] handle charger handle
 * @param[in] volt_mv target voltage in mV
 *
 * @retval actual target voltage(mV) configured to charger
 */
uint32_t HAL_PMU_ChgConfigTargetVolt(PMU_ChgHandleTypeDef *handle, uint32_t volt_mv);

/**
 * @brief Configure charging REP voltage
 *
 *
 * @param[in] handle charger handle
 * @param[in] volt_mv rep voltage in mV
 *
 * @retval actual rep voltage(mV) configured to charger
 */
uint32_t HAL_PMU_ChgConfigRepVolt(PMU_ChgHandleTypeDef *handle, uint32_t volt_mv);


/**
 * @brief Configure charging vbat high voltage
 *
 *
 * @param[in] handle charger handle
 * @param[in] volt_mv vbat high voltage in mV
 *
 * @retval actual vbat high voltage(mV) configured to charger
 */

uint32_t HAL_PMU_ChgConfigVbatHighVolt(PMU_ChgHandleTypeDef *handle, uint32_t volt_mv);

/**
 * @brief Enable/disable IRQ in specified trig mode
 *
 *
 * @param[in] handle charger handle
 * @param[in] irq irq needs to be enabled or disabled
 * @param[in] trig_mode irq trig mode or disable
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_ChgConfigIRQ(PMU_ChgHandleTypeDef *handle, PMU_ChgIrqStatusTypeDef irq, PMU_ChgIrqTrigModeTypeDef trig_mode);

/**
 * @brief Register charger IRQ callback which is called when IRQ is triggered
 *
 *
 * @param[in] handle charger handle
 * @param[in] callback callback
 *
 * @retval actual rep voltage(mV) configured to charger
 */
void HAL_PMU_ChgRegisterCallback(PMU_ChgHandleTypeDef *handle, PMU_ChgCallbackTypeDef callback);

/**
 * @brief PMU IRQ Handler
 *
 * It should be callbed by IRQ handler registered in vector table
 *
 * @param[in] handle charger handle
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_IRQHandler(PMU_ChgHandleTypeDef *handle);

/**
 * @brief Read raw charger event status
 *
 * @param[in] handle charger handle
 * @param[in] irq event type
 *
 * @retval event status, 0 or 1
 */
int8_t HAL_PMU_ChgReadStatus(PMU_ChgHandleTypeDef *handle, PMU_ChgIrqStatusTypeDef irq);


/**
 * @brief Start force charging.
 *
 * HAL_PMU_ChgStopForceCharging should be called to stop force charging completely.
 *
 * @param[in] handle charger handle
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_ChgStartForceCharging(PMU_ChgHandleTypeDef *handle);

/**
 * @brief Suspend force charging
 *
 * Charging is stopped temporarily, but force control is not disbled.
 * HAL_PMU_ChgStopForceCharging should be called to disable force control completely.
 *
 * @param[in] handle charger handle
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_ChgSuspendForceCharging(PMU_ChgHandleTypeDef *handle);

/**
 * @brief Resume force charging
 *
 * @param[in] handle charger handle
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_ChgResumeForceCharging(PMU_ChgHandleTypeDef *handle);


/**
 * @brief Stop force charging
 *
 * @param[in] handle charger handle
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_PMU_ChgStopForceCharging(PMU_ChgHandleTypeDef *handle);

/**
 * @brief Read hardware state
 *
 * @param[in] handle charger handle
 *
 * @retval hardware state
 */
HAL_PMU_ChgHwStateTypeDef HAL_PMU_ChgGetHwState(PMU_ChgHandleTypeDef *handle);


/**
 * @brief Configure EOC Current
 *
 * @param[in] handle charger handle
 * @param[in] percent percentage of CC Curent, e.g. 10 is 10% of CC Current
 *                    range: [4, 36]
 *
 * @retval actual percentage, 0 for invalid input
 */
uint8_t HAL_PMU_ChgConfigEocCc(PMU_ChgHandleTypeDef *handle, uint8_t percent);

/**
 * @brief Open and close peripheral LDO
 *
 * @param[in] ldo which LDO needs to be opened or closed
 * @param[in] en true: open, false: close
 * @param[in] wait true: wait for stable, false: no wait for stable
 *
 * @retval hal status
 */
HAL_StatusTypeDef HAL_PMU_ConfigPeriLdo(PMU_PeriLdoTypeDef ldo, bool en, bool wait);

/**
 * @brief Load PMU calibration data to PMU register
 *
 *
 * @return void
 */
void HAL_PMU_LoadCalData(void);

/**
 * @brief Get HPSYS LDO (1.1V) calibration data
 *
 * @param[in,out] vout_ref pointer to data
 *
 * @retval hal status
 */
HAL_StatusTypeDef HAL_PMU_GetHpsysVoutRef(uint8_t *vout_ref);

/**
 * @brief Get HPSYS LDO (1.2V) calibration data
 *
 * @param[in,out] vout_ref pointer to data
 *
 * @retval hal status
 */
HAL_StatusTypeDef HAL_PMU_GetHpsysVoutRef2(uint8_t *vout_ref);


#endif /* SF32LB52X */





/**
 * @brief  Init PMU
 *
 *
 * @retval void
 */
void HAL_PMU_Init(void);


///@} PMU

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_PMU_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
