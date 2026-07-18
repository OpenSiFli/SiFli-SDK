/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_AON_SF32LB52X_H
#define __BF0_HAL_AON_SF32LB52X_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __BF0_HAL_AON_H
#error "Not include this file directly, include bf0_hal_aon.h"
#endif /* __BF0_HAL_AON_H */


#define AON_LCPU_ACTIVE_REQUEST_REF_COUNT_SUPPORT

/** @brief hpsys wakeup source */
typedef enum
{
    /* First part, keep enum value same as macro definition to simplify implementation */
    HPAON_WAKEUP_SRC_RTC       = HPSYS_AON_WER_RTC_Pos,   /**< RTC wakeup source */
    HPAON_WAKEUP_SRC_LPTIM1    = HPSYS_AON_WER_LPTIM1_Pos,    /**< LPTIM1 wakeup source */
    HPAON_WAKEUP_SRC_LP2HP_REQ = HPSYS_AON_WER_LP2HP_REQ_Pos,  /**< LP2HP manual wakeup source */
    HPAON_WAKEUP_SRC_LP2HP_IRQ = HPSYS_AON_WER_LP2HP_IRQ_Pos,  /**< LP2HP mailbox interrupt wakeup source */
    HPAON_WAKEUP_SRC_GPIO1     = HPSYS_AON_WER_GPIO1_Pos,    /**< GPIO1 wakeup source */
    HPAON_WAKEUP_SRC_PMUC      = HPSYS_AON_WER_PMUC_Pos,     /**< PMUC wakeup source */

    /* Second part, PIN wakeup source */
    /* NOTE:  HPAON_WAKEUP_SRC_PIN0 value must be greater than any non-pin wakeup source */
    HPAON_WAKEUP_SRC_PIN0 = 16,  /**< PIN0 wakeup source  */
    HPAON_WAKEUP_SRC_PIN1 = 17,  /**< PIN1 wakeup source  */
    HPAON_WAKEUP_SRC_PIN2,       /**< PIN2 wakeup source  */
    HPAON_WAKEUP_SRC_PIN3,       /**< PIN3 wakeup source  */
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
    HPAON_WAKEUP_SRC_PIN14,
    HPAON_WAKEUP_SRC_PIN15,
    HPAON_WAKEUP_SRC_PIN16,
    HPAON_WAKEUP_SRC_PIN17,
    HPAON_WAKEUP_SRC_PIN18,
    HPAON_WAKEUP_SRC_PIN19,
    HPAON_WAKEUP_SRC_PIN20,
    HPAON_WAKEUP_SRC_PIN_LAST = HPAON_WAKEUP_SRC_PIN20,
    HPAON_WAKEUP_SRC_PBR_PIN_FIRST = HPAON_WAKEUP_SRC_PIN0,
    HPAON_WAKEUP_SRC_PBR_PIN_LAST = HPAON_WAKEUP_SRC_PIN3,
} HPAON_WakeupSrcTypeDef;


/** @brief lpsys wakeup source */
typedef enum
{
    /* First part, keep enum value same as macro definition to simplify implementation */
    LPAON_WAKEUP_SRC_RTC       = LPSYS_AON_WER_RTC_Pos,       /**< RTC wakeup source */
    LPAON_WAKEUP_SRC_HP2LP_REQ = LPSYS_AON_WER_HP2LP_REQ_Pos, /**< HP2LP manual wakeup source */
    LPAON_WAKEUP_SRC_HP2LP_IRQ = LPSYS_AON_WER_HP2LP_IRQ_Pos, /**< HP2LP mailbox interrupt wakeup source */
    LPAON_WAKEUP_SRC_GPIO2     = LPSYS_AON_WER_GPIO2_Pos,    /**< GPIO2 wakeup source */
    LPAON_WAKEUP_SRC_LPTIM3    = LPSYS_AON_WER_LPTIM3_Pos,   /**< LPTIM2 wakeup source */
    LPAON_WAKEUP_SRC_BT        = LPSYS_AON_WER_BT_Pos,       /**< BT wakeup source */

    /* Second part, PIN wakeup source */
    /* NOTE:  HPAON_WAKEUP_SRC_PIN0 value must be greater than any non-pin wakeup source */
    LPAON_WAKEUP_SRC_PIN0 = 16,     /**< PIN0 wakeup source */
    LPAON_WAKEUP_SRC_PIN1,          /**< PIN1 wakeup source */
    LPAON_WAKEUP_SRC_PIN2,          /**< PIN2 wakeup source */
    LPAON_WAKEUP_SRC_PIN3,          /**< PIN3 wakeup source */
    LPAON_WAKEUP_SRC_PIN4,          /**< PIN4 wakeup source */
    LPAON_WAKEUP_SRC_PIN5,          /**< PIN5 wakeup source */
    LPAON_WAKEUP_SRC_PIN6,
    LPAON_WAKEUP_SRC_PIN7,
    LPAON_WAKEUP_SRC_PIN8,
    LPAON_WAKEUP_SRC_PIN9,
    LPAON_WAKEUP_SRC_PIN10,
    LPAON_WAKEUP_SRC_PIN11,
    LPAON_WAKEUP_SRC_PIN12,
    LPAON_WAKEUP_SRC_PIN13,
    LPAON_WAKEUP_SRC_PIN14,
    LPAON_WAKEUP_SRC_PIN15,
    LPAON_WAKEUP_SRC_PIN16,
    LPAON_WAKEUP_SRC_PIN17,
    LPAON_WAKEUP_SRC_PIN18,
    LPAON_WAKEUP_SRC_PIN19,
    LPAON_WAKEUP_SRC_PIN20,
    LPAON_WAKEUP_SRC_PIN_LAST = LPAON_WAKEUP_SRC_PIN20,
    LPAON_WAKEUP_SRC_PBR_PIN_FIRST = LPAON_WAKEUP_SRC_PIN0,
    LPAON_WAKEUP_SRC_PBR_PIN_LAST = LPAON_WAKEUP_SRC_PIN3,
} LPAON_WakeupSrcTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_AON_SF32LB52X_H */