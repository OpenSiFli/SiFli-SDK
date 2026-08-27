/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_AON_H
#define __BF0_HAL_AON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
#include "bf0_pin_const.h"

/** @addtogroup AON AON
  * @ingroup BF0_HAL_Driver
  * @{
  */

/* Feature list, define supported feature macro in chip specific aon header file, such as bf0_hal_aon_sf32lb52x */
/* PMUC and HPSYS_AON share the same pin wakeup source register */
//#define AON_PMUC_WSR_PIN_COMBINED_SUPPORT
/* Reference count is used by HAL_HPAON_WakeCore and HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST */
//#define AON_LCPU_ACTIVE_REQUEST_REF_COUNT_SUPPORT


#ifdef SF32LB55X
#include "bf0_hal_aon_sf32lb55x.h"
#elif defined(SF32LB56X)
#include "bf0_hal_aon_sf32lb56x.h"
#elif defined(SF32LB58X)
#include "bf0_hal_aon_sf32lb58x.h"
#elif defined(SF32LB52X)
#include "bf0_hal_aon_sf32lb52x.h"
#elif defined(SF32LB57X)
#include "bf0_hal_aon_sf32lb57x.h"
#else
#error "Unsupported chip"
#endif /* SF32LB55X */


/* Exported types ------------------------------------------------------------*/
/** @defgroup AON_Exported_Types AON Exported Types
 * @{
 *
 * @brief AON export types
 */



/** AON Pin wakeup mode, should be consistent with PMU_PinModeTypeDef */
typedef enum
{
    AON_PIN_MODE_HIGH,   /**< high level to trigger pin wakeup */
    AON_PIN_MODE_LOW,    /**< low level to trigger pin wakeup */
    AON_PIN_MODE_POS_EDGE,  /**< postive edge to trigger pin wakeup */
    AON_PIN_MODE_NEG_EDGE,  /**< negative edge to trigger pin wakeup */
#if !defined(SF32LB57X)
    AON_PIN_MODE_DOUBLE_EDGE,  /**< positive or negative edge to trigger pin wakeup */
#endif
} AON_PinModeTypeDef;


/** AON wakeup pin type */
typedef struct
{
    GPIO_TypeDef *gpio;  /**< GPIO instance */
    uint16_t      pin;   /**< pin id, starting from 0*/
} AON_WakeupPinTypeDef;


#define AON_PMR_ACTIVE            0
#define AON_PMR_LIGHT_SLEEP       1
#define AON_PMR_DEEP_SLEEP        2
#define AON_PMR_STANDBY           3


///@} AON_Exported_Types



#ifdef AON_PMUC_WSR_PIN_COMBINED_SUPPORT

/**
 * @brief  Set hpsys wakeup enable register
 * @param  wer wakeup enable register value
 * @retval void
 */
#define HAL_HPAON_SET_WER(wer)     (hwp_hpsys_aon->WER = (wer))


/**
 * @brief  Get hpsys wakeup enable register
 * @retval wakeup enable register
 */
#define HAL_HPAON_GET_WER()     (hwp_hpsys_aon->WER | (HAL_PMU_GET_WER() & PMUC_WSR_PIN_ALL))

/**
 * @brief  Get hpsys wakeup source register
 * @retval wsr wakeup source register value
 */
#define HAL_HPAON_GET_WSR()        (hwp_hpsys_aon->WSR | (HAL_PMU_GET_WSR() & PMUC_WSR_PIN_ALL))

#define HAL_HPAON_GET_WSR_PIN()        HAL_PMU_GET_WSR_PIN()
#define HPSYS_AON_WSR_PIN_FIRST_POS    PMUC_WSR_PA33_Pos

/**
 * @brief  Clear hpsys wakeup source register
 * @param[in] wsr
 * @retval wsr wakeup source register value
 */
#define HAL_HPAON_CLEAR_WSR(wsr)                        \
   do                                                   \
   {                                                    \
      HAL_PMU_CLEAR_WSR(wsr & PMUC_WSR_PIN_ALL);        \
      hwp_hpsys_aon->WCR = ((wsr)|HPSYS_AON_WCR_AON);   \
   }                                                    \
   while(0)


#else

/**
 * @brief  Set hpsys wakeup enable register
 * @param  wer wakeup enable register value
 * @retval void
 */
#define HAL_HPAON_SET_WER(wer)     (hwp_hpsys_aon->WER = (wer))

/**
 * @brief  Get hpsys wakeup enable register
 * @retval wakeup enable register
 */
#define HAL_HPAON_GET_WER()     (hwp_hpsys_aon->WER)

/**
 * @brief  Get hpsys wakeup source register
 * @retval wsr wakeup source register value
 */
#define HAL_HPAON_GET_WSR()        (hwp_hpsys_aon->WSR)


#define HAL_HPAON_GET_WSR_PIN()        (hwp_hpsys_aon->WSR & HPSYS_AON_WSR_PIN_ALL)
#define HPSYS_AON_WSR_PIN_FIRST_POS    HPSYS_AON_WSR_PIN0_Pos

/**
 * @brief  Clear hpsys wakeup source register
 * @param[in] wsr
 * @retval wsr wakeup source register value
 */
#define HAL_HPAON_CLEAR_WSR(wsr)   (hwp_hpsys_aon->WCR = ((wsr)|HPSYS_AON_WCR_AON))
#endif /* AON_PMUC_WSR_PIN_COMBINED_SUPPORT */

/**
 * @brief  Get HPSYS power mode
 * @retval power mode
 */
#define HAL_HPAON_GET_POWER_MODE() ((hwp_hpsys_aon->PMR & HPSYS_AON_PMR_MODE_Msk)>>HPSYS_AON_PMR_MODE_Pos)

/**
 * @brief  Set HPSYS power mode
 * @param  mode power mode
 * @retval void
 */
#define HAL_HPAON_SET_POWER_MODE(mode) (hwp_hpsys_aon->PMR = (mode<<HPSYS_AON_PMR_MODE_Pos))

/**
 * @brief  Clear HPSYS power mode
 * @retval void
 */
#define HAL_HPAON_CLEAR_POWER_MODE() (hwp_hpsys_aon->PMR = 0)

/**
 * @brief  Set ISSR.hp_active bit
 * @retval void
 */
#define HAL_HPAON_SET_HP_ACTIVE()     (hwp_hpsys_aon->ISSR |= HPSYS_AON_ISSR_HP_ACTIVE)

/**
 * @brief  Clear ISSR.hp_active bit
 * @retval void
 */
#define HAL_HPAON_CLEAR_HP_ACTIVE()   (hwp_hpsys_aon->ISSR &= ~HPSYS_AON_ISSR_HP_ACTIVE)

/**
 * @brief  Cancel the LP active request
 * @retval void
 */
#ifdef AON_LCPU_ACTIVE_REQUEST_REF_COUNT_SUPPORT
extern uint8_t g_hal_hpaon_lcpu_wakeup_ref_cnt;
#define HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST()                     \
    do                                                           \
    {                                                            \
        uint32_t mask;                                           \
        mask = HAL_DisableInterrupt();                           \
        HAL_ASSERT(g_hal_hpaon_lcpu_wakeup_ref_cnt > 0);         \
        g_hal_hpaon_lcpu_wakeup_ref_cnt--;                       \
        if (0 == g_hal_hpaon_lcpu_wakeup_ref_cnt)                \
        {                                                        \
            hwp_hpsys_aon->ISSR &= ~HPSYS_AON_ISSR_HP2LP_REQ;    \
        }                                                        \
        HAL_EnableInterrupt(mask);                               \
    }                                                            \
    while (0)

#else
#define HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST()   (hwp_hpsys_aon->ISSR &= ~HPSYS_AON_ISSR_HP2LP_REQ)
#endif /* AON_LCPU_ACTIVE_REQUEST_REF_COUNT_SUPPORT */

/**
 * @brief  Check whether HP2LP_REQ is active
 * @retval true: active, false: inactive
 */
#define HAL_HPAON_IS_HP2LP_REQ_ACTIVE()        (hwp_hpsys_aon->ISSR & HPSYS_AON_ISSR_HP2LP_REQ)

/**
 * @brief  Check whether LP is active
 * @retval bool, true: active, false: not active
 */
#define HAL_HPAON_IS_LP_ACTIVE()               (hwp_hpsys_aon->ISSR & HPSYS_AON_ISSR_LP_ACTIVE)


#ifdef SF32LB55X

/**
 * @brief  Disable PADA in HPSYS
 * @retval void
 */
#define HAL_HPAON_DISABLE_PAD()       (hwp_hpsys_aon->ANACR |= HPSYS_AON_ANACR_PA_ISO)

/**
 * @brief  Enable PADA in HPSYS
 * @retval void
 */
#define HAL_HPAON_ENABLE_PAD()       (hwp_hpsys_aon->ANACR &= ~HPSYS_AON_ANACR_PA_ISO)
#else
/**
 * @brief  Disable PADA in HPSYS
 * @retval void
 */
#ifdef HPSYS_AON_ANACR_PA_ISO
#define HAL_HPAON_DISABLE_PAD()       (hwp_hpsys_aon->ANACR |= (HPSYS_AON_ANACR_PA_ISO))
#elif defined(PMUC_CR_PA_RET)
#define HAL_HPAON_DISABLE_PAD()       (hwp_pmuc->CR |= (PMUC_CR_PA_RET))
#else
#define HAL_HPAON_DISABLE_PAD()
#endif /* HPSYS_AON_ANACR_PA_ISO */

/**
 * @brief  Disable VHP in HPSYS
 * @retval void
 */
#define HAL_HPAON_DISABLE_VHP()       (hwp_hpsys_aon->ANACR |= (HPSYS_AON_ANACR_VHP_ISO))


/**
 * @brief  Enable PADA in HPSYS
 * @retval void
 */
#ifdef HPSYS_AON_ANACR_PA_ISO
#define HAL_HPAON_ENABLE_PAD()       (hwp_hpsys_aon->ANACR &= ~(HPSYS_AON_ANACR_PA_ISO))
#elif defined(PMUC_CR_PA_RET)
#define HAL_HPAON_ENABLE_PAD()       (hwp_pmuc->CR &= ~(PMUC_CR_PA_RET))
#else
#define HAL_HPAON_ENABLE_PAD()
#endif /* HPSYS_AON_ANACR_PA_ISO */

/**
 * @brief  Enable VHP in HPSYS
 * @retval void
 */
#define HAL_HPAON_ENABLE_VHP()       (hwp_hpsys_aon->ANACR &= ~(HPSYS_AON_ANACR_VHP_ISO))


#endif

/**
 * @brief  Set lpsys wakeup enable register
 * @param  wer wakeup enable register value
 * @retval void
 */
#define HAL_LPAON_SET_WER(wer)     hwp_lpsys_aon->WER = (wer)

/**
 * @brief  Get lpsys wakeup enable register
 * @retval wakeup enable register
 */
#define HAL_LPAON_GET_WER()     (hwp_lpsys_aon->WER)

/**
 * @brief  Get lpsys wakeup source register
 * @retval wsr wakeup source register value
 */
#define HAL_LPAON_GET_WSR()        (hwp_lpsys_aon->WSR)

#ifdef LPSYS_AON_WSR_PIN_ALL
#define HAL_LPAON_GET_WSR_PIN()        (hwp_lpsys_aon->WSR & LPSYS_AON_WSR_PIN_ALL)
#endif /* LPSYS_AON_WSR_PIN_ALL */


/**
 * @brief  Clear lpsys wakeup source register
 * @param[in] wsr
 * @retval wsr wakeup source register value
 */
#define HAL_LPAON_CLEAR_WSR(wsr)   (hwp_lpsys_aon->WCR = ((wsr)|LPSYS_AON_WCR_AON))


/**
 * @brief  Set LPSYS power mode
 * @param  mode power mode
 * @retval void
 */
#define HAL_LPAON_SET_POWER_MODE(mode) { \
                hwp_lpsys_aon->PMR &= ~LPSYS_AON_PMR_MODE_Msk; \
                hwp_lpsys_aon->PMR |= mode<<LPSYS_AON_PMR_MODE_Pos; \
            }

/**
 * @brief  Get LPSYS power mode
 * @retval power mode
 */
#define HAL_LPAON_GET_POWER_MODE() ((hwp_lpsys_aon->PMR & LPSYS_AON_PMR_MODE_Msk)>>LPSYS_AON_PMR_MODE_Pos)


/**
 * @brief  Clear LPSYS power mode
 * @retval void
 */
#define HAL_LPAON_CLEAR_POWER_MODE() (hwp_lpsys_aon->PMR &= ~LPSYS_AON_PMR_MODE_Msk)

/**
 * @brief  Set ISSR.lp_active bit
 * @retval void
 */
#define HAL_LPAON_SET_LP_ACTIVE()     (hwp_lpsys_aon->ISSR |= LPSYS_AON_ISSR_LP_ACTIVE)

/**
 * @brief  Clear ISSR.lp_active bit
 * @retval void
 */
#define HAL_LPAON_CLEAR_LP_ACTIVE()   (hwp_lpsys_aon->ISSR &= ~LPSYS_AON_ISSR_LP_ACTIVE)

/**
 * @brief  Cancel the HP active request
 * @retval void
 */
#define HAL_LPAON_CANCEL_HP_ACTIVE_REQUEST()   (hwp_lpsys_aon->ISSR &= ~LPSYS_AON_ISSR_LP2HP_REQ)

/**
 * @brief  Check whether HP is active
 * @retval bool, true: active, false: not active
 */
#define HAL_LPAON_IS_HP_ACTIVE()               (hwp_lpsys_aon->ISSR & LPSYS_AON_ISSR_HP_ACTIVE)


/**
 * @brief  Get LPSYS AON.SBCR
 * @retval power mode
 */
#define HAL_LPAON_GET_SBCR()          (hwp_lpsys_aon->SBCR)

/**
 * @brief  Set LPSYS AON.SBCR
 * @param  sbcr standby mode ctrl register value
 * @retval void
 */
#define HAL_LPAON_SET_SBCR(sbcr)      (hwp_lpsys_aon->SBCR = (sbcr))


#ifdef SF32LB55X
/**
 * @brief  Disable PAD in LPSYS
 * @retval void
 */
#define HAL_LPAON_DISABLE_PAD()      do                                                    \
                                     {                                                     \
                                        hwp_lpsys_aon->ANACR |= LPSYS_AON_ANACR_PB_ISO;    \
                                     }                                                     \
                                     while (0)


/**
 * @brief  Disable AON PAD in LPSYS
 * @retval void
 */
#define HAL_LPAON_DISABLE_AON_PAD()  do                                                    \
                                     {                                                     \
                                        hwp_lpsys_aon->ANACR |= LPSYS_AON_ANACR_PB_AON_ISO;\
                                     }                                                     \
                                     while (0)


/**
 * @brief  Enable both normal and AON PAD in LPSYS
 * @retval void
 */
#define HAL_LPAON_ENABLE_PAD()       do                                                    \
                                     {                                                     \
                                        hwp_lpsys_aon->ANACR &= ~LPSYS_AON_ANACR_PB_ISO;   \
                                        hwp_lpsys_aon->ANACR &= ~LPSYS_AON_ANACR_PB_AON_ISO;   \
                                     }                                                         \
                                     while (0)
#else
/**
 * @brief  Disable PADB in LPSYS
 * @retval void
 */
#ifdef  LPSYS_AON_ANACR_PB_ISO
#define HAL_LPAON_DISABLE_PAD()      do                                                    \
                                     {                                                     \
                                        hwp_lpsys_aon->ANACR |= LPSYS_AON_ANACR_PB_ISO;    \
                                     }                                                     \
                                     while (0)
#else
#define HAL_LPAON_DISABLE_PAD()
#endif

/**
 * @brief  Disable VLP in LPSYS
 * @retval void
*/
#define HAL_LPAON_DISABLE_VLP()      do                                                    \
                                     {                                                     \
                                        hwp_lpsys_aon->ANACR |= LPSYS_AON_ANACR_VLP_ISO;   \
                                     }                                                     \
                                     while (0)


//TODO
#define HAL_LPAON_DISABLE_AON_PAD()

/**
 * @brief  Enable PADB in LPSYS
 * @retval void
 */
#ifdef  LPSYS_AON_ANACR_PB_ISO
#define HAL_LPAON_ENABLE_PAD()       do                                                    \
                                     {                                                     \
                                        hwp_lpsys_aon->ANACR &= ~LPSYS_AON_ANACR_PB_ISO;   \
                                     }                                                     \
                                     while (0)
#else
#define HAL_LPAON_ENABLE_PAD()
#endif
/**
 * @brief  Enable VLP in LPSYS, it's related to analog
 * @retval void
*/
#define HAL_LPAON_ENABLE_VLP()    do                                                    \
                                  {                                                     \
                                     hwp_lpsys_aon->ANACR &= ~LPSYS_AON_ANACR_VLP_ISO;  \
                                  }                                                     \
                                  while (0)

/**
 * @brief  Enable Power Supply Request for Deep Sleep
 * @retval void
*/
#define HAL_LPAON_ENABLE_DS_PWR_REQ()      hwp_lpsys_aon->DSCR |= LPSYS_AON_DSCR_PWR_REQ

/**
 * @brief  Disable Power Supply Request for Deep Sleep
 * @retval void
*/
#define HAL_LPAON_DISABLE_DS_PWR_REQ()      hwp_lpsys_aon->DSCR &= ~LPSYS_AON_DSCR_PWR_REQ

#endif


/**
 * @brief  HPSYS wakeup the specified LPSYS or BLESYS
 * @param  core_id core id, CORE_ID_LCPU or CORE_ID_BCPU
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_WakeCore(uint8_t core_id);

/**
 * @brief  Indicate HPSYS to enter light sleep mode
 * @param  lscr light sleep control register value
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_EnterLightSleep(uint32_t lscr);

/**
 * @brief  Indicate HPSYS to enter deep sleep mode
 * @param  dscr deep sleep control register value
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_EnterDeepSleep(uint32_t dscr);

/**
 * @brief  Indicate HPSYS to enter standby mode
 * @param  sbcr standby sleep control register value
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_EnterStandby(uint32_t sbcr);


/**
 * @brief  Enable HPSYS wakeup source
 * @param  src wakeup source
 * @param  mode pin mode, needed when wakeup source is pin
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_EnableWakeupSrc(HPAON_WakeupSrcTypeDef src, AON_PinModeTypeDef mode);

/**
 * @brief  Disable specified HPSYS wakeup source
 * @param  src wakeup source
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_DisableWakeupSrc(HPAON_WakeupSrcTypeDef src);

//TODO:
/**
 * @brief  Enable HPSYS pin wakeup source
 * @param  pad pad
 * @param  mode pin mode
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_EnablePinWakeup(pin_pad pad, AON_PinModeTypeDef mode);

/**
 * @brief  Disable HPSYS pin wakeup source
 * @param  pad pad
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_DisablePinWakeup(pin_pad pad);


/**
 * @brief  Query wakeup pin bound with corresponding gpio pin
 * @param  gpio GPIO instance, e.g. hwp_gpio1
 * @param  gpio_pin gpio pin, start from 0, 0 is the first pin of the gpio instance
 * @return wakeup pin number, its range varies with different chips
 * @retval -1 invalid
 * @retval 0~3 for sf32lb55x, PA77~PA80
 * @retval 0~17 for sf32lb58x, PB54~PB59, PA64~PA69, PBR0~PBR5
 * @retval 0~13 for sf32lb56x, PB32~PB36, PA50~PA54, PBR0~PBR3
 * @retval 0~20 for sf32lb52x, PA24~PA44
 * @retval 0~13 for sf32lb57x, PA33~PA42, PA24~PA27
 *
 */
int8_t HAL_HPAON_QueryWakeupPin(GPIO_TypeDef *gpio, uint16_t gpio_pin);


/**
 * @brief  Query gpio pin bound with corresponding wakeup pin
 * @param[in] wakeup_pin wakeup pin, range: 0~5 (Z0), 0~3 (A0)
 * @param[in,out] gpio_pin  pointer to output gpio pin
 * @retval gpio instance, NULL: not found
 */
GPIO_TypeDef *HAL_HPAON_QueryWakeupGpioPin(uint8_t wakeup_pin, uint16_t *gpio_pin);


/**
 * @brief  Get wakeup pin mode
 * @param[in] wakeup_pin wakeup pin, range: 0~5 (Z0), 0~3 (A0)
 * @param[in,out] mode  pointer to output pin mode
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_GetWakeupPinMode(uint8_t wakeup_pin, AON_PinModeTypeDef *mode);

/**
 * @brief  Deactivate HPSYS
 * @retval void
 */
void HAL_HPAON_Deactivate(void);
/**
* @brief  Enable Oscillator from HPSYS
* @retval void
*/
void HAL_HPAON_EnableRC(void);

/**
 * @brief  Revoke Oscillator from HPSYS
 * @retval void
 */
void HAL_HPAON_DisableRC(void);

/**
* @brief  Enable XTAL48 from HPSYS
* @retval void
*/
void HAL_HPAON_EnableXT48(void);

/**
* @brief  Revoke XTAL48 from HPSYS
* @retval void
*/
void HAL_HPAON_DisableXT48(void);



/**
 * @brief  Start GTimer
 *
 * LPSYS must be awake, recommend to call this function at system intialization stage
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_StartGTimer(void);

/**
 * @brief  Stop GTimer
 *
 * LPSYS must be awake
 *
 * @retval status
 */
HAL_StatusTypeDef HAL_HPAON_StopGTimer(void);

/**
 * @brief  Read GTimer value by HPSYS
 * @retval gtimer value
 */
#define HAL_HPAON_READ_GTIMER()      (hwp_hpsys_aon->GTIMR)

/**
 * @brief  Read GTimer value by LPSYS
 * @retval gtimer value
 */
#define HAL_LPAON_READ_GTIMER()      (hwp_lpsys_aon->GTIMR)


#ifdef SOC_BF0_LCPU
#define HAL_GTIMER_READ() HAL_LPAON_READ_GTIMER()
#ifdef LPSYS_AON_WSR_PIN_NUM
#define HAL_AON_GetWakePinMode(pin, mode)  HAL_LPAON_GetWakeupPinMode((pin), (mode))
#define HAL_AON_QueryWakeupGpioPin(wakeup_pin, gpio_pin)  HAL_LPAON_QueryWakeupGpioPin((wakeup_pin), (gpio_pin))
#endif /* LPSYS_AON_WSR_PIN_NUM */
#ifdef SF32LB55X
#define HAL_GTIMER_ENABLE() (hwp_lpsys_aon->CR |= LPSYS_AON_CR_GTIM_EN)
#define HAL_GTIMER_IS_ENABLED() (hwp_lpsys_aon->CR&LPSYS_AON_CR_GTIM_EN)
#define HAL_GTIMER_DISABLE() (hwp_lpsys_aon->CR &= ~LPSYS_AON_CR_GTIM_EN)
#else
#define HAL_GTIMER_ENABLE() (hwp_lpsys_aon->CR1 |= LPSYS_AON_CR1_GTIM_EN)
#define HAL_GTIMER_IS_ENABLED() (hwp_lpsys_aon->CR1&LPSYS_AON_CR1_GTIM_EN)
#define HAL_GTIMER_DISABLE() (hwp_lpsys_aon->CR1 &= ~LPSYS_AON_CR1_GTIM_EN)
#endif /* SF32LB55X */
#else
#define HAL_GTIMER_READ() HAL_HPAON_READ_GTIMER()
#define HAL_AON_GetWakePinMode(pin, mode)  HAL_HPAON_GetWakeupPinMode((pin), (mode))
#define HAL_AON_QueryWakeupGpioPin(wakeup_pin, gpio_pin)  HAL_HPAON_QueryWakeupGpioPin((wakeup_pin), (gpio_pin))
#ifdef SF32LB55X
#define HAL_GTIMER_ENABLE() (hwp_hpsys_aon->CR |= HPSYS_AON_CR_GTIM_EN)
#define HAL_GTIMER_IS_ENABLED() (hwp_hpsys_aon->CR&HPSYS_AON_CR_GTIM_EN)
#define HAL_GTIMER_DISABLE() (hwp_hpsys_aon->CR &= ~HPSYS_AON_CR_GTIM_EN)
#else
#define HAL_GTIMER_ENABLE() (hwp_hpsys_aon->CR1 |= HPSYS_AON_CR1_GTIM_EN)
#define HAL_GTIMER_IS_ENABLED() (hwp_hpsys_aon->CR1&HPSYS_AON_CR1_GTIM_EN)
#define HAL_GTIMER_DISABLE() (hwp_hpsys_aon->CR1 &= ~HPSYS_AON_CR1_GTIM_EN)
#endif /* SF32LB55X */
#endif /* SOC_BF0_LCPU */





/**
 * @brief  LPSYS wakeup the specified HPSYS or BLESYS
 * @param  core_id core id, CORE_ID_HCPU or CORE_ID_BCPU
 * @retval status
 */
HAL_StatusTypeDef HAL_LPAON_WakeCore(uint8_t core_id);

/**
 * @brief  Indicate LPSYS to enter light sleep mode
 * @param  lscr light sleep control register value
 * @retval status
 */
HAL_StatusTypeDef HAL_LPAON_EnterLightSleep(uint32_t lscr);

/**
 * @brief  Indicate LSYS to enter deep sleep mode
 * @param  dscr deep sleep control register value
 * @retval status
 */
HAL_StatusTypeDef HAL_LPAON_EnterDeepSleep(uint32_t dscr);

/**
 * @brief  Indicate LPSYS to enter deep sleep mode
 * @param  sbcr deep sleep control register value
 * @retval status
 */
HAL_StatusTypeDef HAL_LPAON_EnterStandby(uint32_t sbcr);


/**
 * @brief  Enable LPSYS wakeup source
 * @param  src wakeup source
 * @param  mode pin mode, needed when wakeup source is pin
 * @retval status
 */
HAL_StatusTypeDef HAL_LPAON_EnableWakeupSrc(LPAON_WakeupSrcTypeDef src, AON_PinModeTypeDef mode);

/**
 * @brief  Disable specified LPSYS wakeup source
 * @param  src wakeup source
 * @retval status
 */
HAL_StatusTypeDef HAL_LPAON_DisableWakeupSrc(LPAON_WakeupSrcTypeDef src);

/**
 * @brief  Query wakeup pin bound with corresponding gpio pin
 * @param  gpio GPIO instance, e.g. hwp_gpio1
 * @param  gpio_pin gpio pin, start from 1
 * @retval wakeup pin, valid range 0~5, invalid: -1
 */
int8_t HAL_LPAON_QueryWakeupPin(GPIO_TypeDef *gpio, uint16_t gpio_pin);


/**
 * @brief  Query gpio pin bound with corresponding wakeup pin
 * @param[in] wakeup_pin wakeup pin, range: 0~5
 * @param[in,out] gpio_pin  pointer to output gpio pin
 * @retval gpio instance, NULL: not found
 */
GPIO_TypeDef *HAL_LPAON_QueryWakeupGpioPin(uint8_t wakeup_pin, uint16_t *gpio_pin);

/**
 * @brief  Get wakeup pin mode
 * @param[in] wakeup_pin wakeup pin, range: 0~5
 * @param[in,out] mode  pointer to output pin mode
 * @retval status
 */
HAL_StatusTypeDef HAL_LPAON_GetWakeupPinMode(uint8_t wakeup_pin, AON_PinModeTypeDef *mode);


/**
 * @brief  Configure LCPU start address
 * @retval void
 */
void HAL_LPAON_ConfigStartAddr(uint32_t *start_addr);

/**
 * @brief  Deactivate LPSYS
 * @retval void
 */
void HAL_LPAON_Deactivate(void);

/**
 * @brief  Enable Oscillator from LPSYS
 * @retval void
 */
void HAL_LPAON_EnableRC(void);

/**
 * @brief  Revoke Oscillator from LPSYS
 * @retval void
 */
void HAL_LPAON_DisableRC(void);

/**
* @brief  Enable XTAL48 from LPSYS
* @retval void
*/
void HAL_LPAON_EnableXT48(void);

/**
* @brief  Revoke XTAL48 from LPSYS
* @retval void
*/
void HAL_LPAON_DisableXT48(void);

/**
 * @brief The lcpu sleep function
 * @details Force the low-power core (LCPU) to enter sleep mode when it is not activated.
 */

void HAL_LPAON_Sleep(void);
/**
* @brief  Check GTimer timeout or not
* @param[in] ctx Context for GTimer, should be start GTimer read
* @param[in] timeout_count Count for timeout
* @retval HAL_OK: Not timeout  HAL_TIMEOUT:Timeout
*/
HAL_StatusTypeDef HAL_GTIMER_Check(uint32_t ctx, uint32_t timeout_count);

///@} AON

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_AON_H */