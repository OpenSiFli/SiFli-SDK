/**
  ******************************************************************************
  * @file   bf0_hal_aon.h
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

#ifndef __BF0_HAL_AON_H
#define __BF0_HAL_AON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup AON AON
  * @ingroup BF0_HAL_Driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup AON_Exported_Types AON Exported Types
 * @{
 *
 * @brief AON export types
 */

/** Pin wakeup mode */
typedef enum
{
    AON_PIN_MODE_HIGH,   /**< high level to trigger pin wakeup */
    AON_PIN_MODE_LOW,    /**< low level to trigger pin wakeup */
    AON_PIN_MODE_POS_EDGE,  /**< postive edge to trigger pin wakeup */
    AON_PIN_MODE_NEG_EDGE,  /**< negative edge to trigger pin wakeup */
    AON_PIN_MODE_DOUBLE_EDGE,  /**< positive or negative edge to trigger pin wakeup */
} AON_PinModeTypeDef;



/** @brief hpsys wakeup source */
typedef enum
{
    /* First part, keep enum value same as macro definition to simplify implementation */
    HPAON_WAKEUP_SRC_RTC       = HPSYS_AON_WER_RTC_Pos,   /**< RTC wakeup source */
    HPAON_WAKEUP_SRC_LPTIM1    = HPSYS_AON_WER_LPTIM1_Pos,    /**< LPTIM1 wakeup source */

    HPAON_WAKEUP_SRC_LP2HP_REQ = HPSYS_AON_WER_LP2HP_REQ_Pos,  /**< LP2HP manual wakeup source */
    HPAON_WAKEUP_SRC_LP2HP_IRQ = HPSYS_AON_WER_LP2HP_IRQ_Pos,  /**< LP2HP mailbox interrupt wakeup source */


#ifndef SF32LB55X
    HPAON_WAKEUP_SRC_GPIO1     = HPSYS_AON_WER_GPIO1_Pos,    /**< GPIO1 wakeup source */
#endif /* SF32LB55X */

#ifdef HPSYS_AON_WER_PMUC_Pos
    HPAON_WAKEUP_SRC_PMUC      = HPSYS_AON_WER_PMUC_Pos,     /**< PMUC wakeup source */
#endif /* HPSYS_AON_WER_PMUC_Pos */

    /* Second part, PIN wakeup source */
    /* NOTE:  HPAON_WAKEUP_SRC_PIN0 value must be greater than any non-pin wakeup source */
    HPAON_WAKEUP_SRC_PIN0 = 16,  /**< PIN0 wakeup source  */
    HPAON_WAKEUP_SRC_PIN1 = 17,  /**< PIN1 wakeup source  */
    HPAON_WAKEUP_SRC_PIN2,       /**< PIN2 wakeup source  */
    HPAON_WAKEUP_SRC_PIN3,       /**< PIN3 wakeup source  */
#ifndef SF32LB55X
    HPAON_WAKEUP_SRC_PIN4,
    HPAON_WAKEUP_SRC_PIN5,
    HPAON_WAKEUP_SRC_PIN6,
    HPAON_WAKEUP_SRC_PIN7,
    HPAON_WAKEUP_SRC_PIN8,
    HPAON_WAKEUP_SRC_PIN9,
    HPAON_WAKEUP_SRC_PIN10,
    HPAON_WAKEUP_SRC_PIN11,
    HPAON_WAKEUP_SRC_PIN12,
    HPAON_WAKEUP_SRC_PIN13,
#ifdef SF32LB58X
    HPAON_WAKEUP_SRC_PIN14,
    HPAON_WAKEUP_SRC_PIN15,
    HPAON_WAKEUP_SRC_PIN16,
    HPAON_WAKEUP_SRC_PIN17,
    HPAON_WAKEUP_SRC_PIN_LAST = HPAON_WAKEUP_SRC_PIN17,
    HPAON_WAKEUP_SRC_PBR_PIN_FIRST = HPAON_WAKEUP_SRC_PIN12,

#elif defined(SF32LB56X)
    HPAON_WAKEUP_SRC_PIN_LAST = HPAON_WAKEUP_SRC_PIN13,
    HPAON_WAKEUP_SRC_PBR_PIN_FIRST = HPAON_WAKEUP_SRC_PIN10,
#else
    HPAON_WAKEUP_SRC_PIN14,
    HPAON_WAKEUP_SRC_PIN15,
    HPAON_WAKEUP_SRC_PIN16,
    HPAON_WAKEUP_SRC_PIN17,
    HPAON_WAKEUP_SRC_PIN18,
    HPAON_WAKEUP_SRC_PIN19,
    HPAON_WAKEUP_SRC_PIN20,
    HPAON_WAKEUP_SRC_PIN_LAST = HPAON_WAKEUP_SRC_PIN20,
    HPAON_WAKEUP_SRC_PBR_PIN_FIRST = HPAON_WAKEUP_SRC_PIN17,
#endif /* SF32LB58X */
#else
    HPAON_WAKEUP_SRC_PIN_LAST = HPAON_WAKEUP_SRC_PIN3,
#endif /* SF32LB55X */
} HPAON_WakeupSrcTypeDef;


/** @brief lpsys wakeup source */
typedef enum
{
    /* First part, keep enum value same as macro definition to simplify implementation */
    LPAON_WAKEUP_SRC_RTC       = LPSYS_AON_WER_RTC_Pos,       /**< RTC wakeup source */

    LPAON_WAKEUP_SRC_HP2LP_REQ = LPSYS_AON_WER_HP2LP_REQ_Pos, /**< HP2LP manual wakeup source */
    LPAON_WAKEUP_SRC_HP2LP_IRQ = LPSYS_AON_WER_HP2LP_IRQ_Pos, /**< HP2LP mailbox interrupt wakeup source */

#ifndef SF32LB55X
    LPAON_WAKEUP_SRC_GPIO2     = LPSYS_AON_WER_GPIO2_Pos,    /**< GPIO2 wakeup source */
#endif /* SF32LB55X */

#ifdef SF32LB52X
    LPAON_WAKEUP_SRC_LPTIM3    = LPSYS_AON_WER_LPTIM3_Pos,   /**< LPTIM2 wakeup source */
#else
    LPAON_WAKEUP_SRC_LPTIM2    = LPSYS_AON_WER_LPTIM2_Pos,   /**< LPTIM2 wakeup source */
    LPAON_WAKEUP_SRC_LPCOMP1   = LPSYS_AON_WER_LPCOMP1_Pos,  /**< LPCOMP1 wakeup source */
    LPAON_WAKEUP_SRC_LPCOMP2   = LPSYS_AON_WER_LPCOMP2_Pos,  /**< LPCOMP2 wakeup source */
#endif /* SF32LB52X */

#ifndef SF32LB55X
    LPAON_WAKEUP_SRC_BT        = LPSYS_AON_WER_BT_Pos,       /**< BT wakeup source */
#else
    LPAON_WAKEUP_SRC_BLE       = LPSYS_AON_WER_BLE_Pos,      /**< BLE wakeup source */
#endif /* SF32LB55X */

    /* Second part, PIN wakeup source */
    /* NOTE:  HPAON_WAKEUP_SRC_PIN0 value must be greater than any non-pin wakeup source */
    LPAON_WAKEUP_SRC_PIN0 = 16,     /**< PIN0 wakeup source */
    LPAON_WAKEUP_SRC_PIN1,          /**< PIN1 wakeup source */
    LPAON_WAKEUP_SRC_PIN2,          /**< PIN2 wakeup source */
    LPAON_WAKEUP_SRC_PIN3,          /**< PIN3 wakeup source */
    LPAON_WAKEUP_SRC_PIN4,          /**< PIN4 wakeup source */
    LPAON_WAKEUP_SRC_PIN5,          /**< PIN5 wakeup source */
#ifndef SF32LB55X
    LPAON_WAKEUP_SRC_PIN6,
    LPAON_WAKEUP_SRC_PIN7,
    LPAON_WAKEUP_SRC_PIN8,
    LPAON_WAKEUP_SRC_PIN9,
    LPAON_WAKEUP_SRC_PIN10,
    LPAON_WAKEUP_SRC_PIN11,
    LPAON_WAKEUP_SRC_PIN12,
    LPAON_WAKEUP_SRC_PIN13,
#ifdef SF32LB58X
    LPAON_WAKEUP_SRC_PIN14,
    LPAON_WAKEUP_SRC_PIN15,
    LPAON_WAKEUP_SRC_PIN16,
    LPAON_WAKEUP_SRC_PIN17,
    LPAON_WAKEUP_SRC_PIN_LAST = LPAON_WAKEUP_SRC_PIN17,
    LPAON_WAKEUP_SRC_PBR_PIN_FIRST = LPAON_WAKEUP_SRC_PIN12,
#elif defined(SF32LB56X)
    LPAON_WAKEUP_SRC_PIN_LAST = LPAON_WAKEUP_SRC_PIN13,
    LPAON_WAKEUP_SRC_PBR_PIN_FIRST = LPAON_WAKEUP_SRC_PIN10,
#else
    LPAON_WAKEUP_SRC_PIN14,
    LPAON_WAKEUP_SRC_PIN15,
    LPAON_WAKEUP_SRC_PIN16,
    LPAON_WAKEUP_SRC_PIN17,
    LPAON_WAKEUP_SRC_PIN18,
    LPAON_WAKEUP_SRC_PIN19,
    LPAON_WAKEUP_SRC_PIN20,
    LPAON_WAKEUP_SRC_PIN_LAST = LPAON_WAKEUP_SRC_PIN20,
    LPAON_WAKEUP_SRC_PBR_PIN_FIRST = LPAON_WAKEUP_SRC_PIN17,
#endif /* SF32LB58X */
#else
    LPAON_WAKEUP_SRC_PIN_LAST = LPAON_WAKEUP_SRC_PIN5,
#endif /* SF32LB55X */

} LPAON_WakeupSrcTypeDef;


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

/**
 * @brief  Clear hpsys wakeup source register
 * @param[in] wsr
 * @retval wsr wakeup source register value
 */
#define HAL_HPAON_CLEAR_WSR(wsr)   (hwp_hpsys_aon->WCR = ((wsr)|HPSYS_AON_WCR_AON))

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
#ifdef SF32LB52X
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
#endif /* SF32LB52X */

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
#define HAL_HPAON_DISABLE_PAD()       (hwp_hpsys_aon->ANACR |= (HPSYS_AON_ANACR_PA_ISO))

/**
 * @brief  Disable VHP in HPSYS
 * @retval void
 */
#define HAL_HPAON_DISABLE_VHP()       (hwp_hpsys_aon->ANACR |= (HPSYS_AON_ANACR_VHP_ISO))


/**
 * @brief  Enable PADA in HPSYS
 * @retval void
 */
#define HAL_HPAON_ENABLE_PAD()       (hwp_hpsys_aon->ANACR &= ~(HPSYS_AON_ANACR_PA_ISO))

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
#define HAL_LPAON_DISABLE_PAD()      do                                                    \
                                     {                                                     \
                                        hwp_lpsys_aon->ANACR |= LPSYS_AON_ANACR_PB_ISO;    \
                                     }                                                     \
                                     while (0)


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
#define HAL_LPAON_ENABLE_PAD()       do                                                    \
                                     {                                                     \
                                        hwp_lpsys_aon->ANACR &= ~LPSYS_AON_ANACR_PB_ISO;   \
                                     }                                                     \
                                     while (0)

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

/**
 * @brief  Query wakeup pin bound with corresponding gpio pin
 * @param  gpio GPIO instance, e.g. hwp_gpio1
 * @param  gpio_pin gpio pin, start from 1
 * @retval wakeup pin, valid range: 0~5 (Z0), 0~3 (A0), invalid: -1
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
#define HAL_AON_GetWakePinMode(pin, mode)  HAL_LPAON_GetWakeupPinMode((pin), (mode))
#define HAL_AON_QueryWakeupGpioPin(wakeup_pin, gpio_pin)  HAL_LPAON_QueryWakeupGpioPin((wakeup_pin), (gpio_pin))
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

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
