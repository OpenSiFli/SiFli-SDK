/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_AON_SF32LB57X_H
#define __BF0_HAL_AON_SF32LB57X_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __BF0_HAL_AON_H
#error "Not include this file directly, include bf0_hal_aon.h"
#endif /* __BF0_HAL_AON_H */

/* PMUC and HPSYS_AON share the same wakeup source pin register */
#define AON_PMUC_WSR_PIN_COMBINED_SUPPORT
/* Reference count is used by HAL_HPAON_WakeCore and HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST */
#define AON_LCPU_ACTIVE_REQUEST_REF_COUNT_SUPPORT

#define HPAON_WAKEUP_PIN_PART0_FIRST  (33)
#define HPAON_WAKEUP_PIN_PART0_LAST   (42)
#define HPAON_WAKEUP_PIN_PART0_SIZE   (HPAON_WAKEUP_PIN_PART0_LAST - HPAON_WAKEUP_PIN_PART0_FIRST + 1)

#define HPAON_WAKEUP_PIN_PART1_FIRST  (24)
#define HPAON_WAKEUP_PIN_PART1_LAST   (27)
#define HPAON_WAKEUP_PIN_PART1_SIZE   (HPAON_WAKEUP_PIN_PART1_LAST - HPAON_WAKEUP_PIN_PART1_FIRST + 1)

#define HPAON_WAKEUP_PIN_NUM  (HPAON_WAKEUP_PIN_PART0_SIZE + HPAON_WAKEUP_PIN_PART1_SIZE)


/** @brief hpsys wakeup source */
typedef enum
{
    /* First part, keep enum value same as macro definition to simplify implementation */
    HPAON_WAKEUP_SRC_RTC       = HPSYS_AON_WER_RTC_Pos,   /**< RTC wakeup source */
    HPAON_WAKEUP_SRC_LPTIM1    = HPSYS_AON_WER_LPTIM1_Pos,    /**< LPTIM1 wakeup source */
    HPAON_WAKEUP_SRC_LP2HP_REQ = HPSYS_AON_WER_LP2HP_REQ_Pos,  /**< LP2HP manual wakeup source */
    HPAON_WAKEUP_SRC_LP2HP_IRQ = HPSYS_AON_WER_LP2HP_IRQ_Pos,  /**< LP2HP mailbox interrupt wakeup source */
    HPAON_WAKEUP_SRC_GPIO1     = HPSYS_AON_WER_GPIO1_Pos,    /**< GPIO1 wakeup source */
    HPAON_WAKEUP_SRC_CHG       = HPSYS_AON_WER_CHG_Pos,     /**< PMUC wakeup source */

    HPAON_WAKEUP_SRC_PA33 = 0,   /**< PA33 wakeup source  */
    HPAON_WAKEUP_SRC_PA34 = 1,   /**< PA34 wakeup source  */
    HPAON_WAKEUP_SRC_PA35,       /**< PA35 wakeup source  */
    HPAON_WAKEUP_SRC_PA36,       /**< PA36 wakeup source  */
    HPAON_WAKEUP_SRC_PA37,
    HPAON_WAKEUP_SRC_PA38,
    HPAON_WAKEUP_SRC_PA39,
    HPAON_WAKEUP_SRC_PA40,
    HPAON_WAKEUP_SRC_PA41,
    HPAON_WAKEUP_SRC_PA42,
    HPAON_WAKEUP_SRC_PA24,
    HPAON_WAKEUP_SRC_PA25,
    HPAON_WAKEUP_SRC_PA26,
    HPAON_WAKEUP_SRC_PA27,
    HPAON_WAKEUP_SRC_PIN0 = 0,   /**< PA33 wakeup source  */
    HPAON_WAKEUP_SRC_PIN1 = 1,   /**< PA34 wakeup source  */
    HPAON_WAKEUP_SRC_PIN2,       /**< PA35 wakeup source  */
    HPAON_WAKEUP_SRC_PIN3,       /**< PA36 wakeup source  */
    HPAON_WAKEUP_SRC_PIN4,       /**< PA37 wakeup source  */
    HPAON_WAKEUP_SRC_PIN5,       /**< PA38 wakeup source  */
    HPAON_WAKEUP_SRC_PIN6,       /**< PA39 wakeup source  */
    HPAON_WAKEUP_SRC_PIN7,       /**< PA40 wakeup source  */
    HPAON_WAKEUP_SRC_PIN8,       /**< PA41 wakeup source  */
    HPAON_WAKEUP_SRC_PIN9,       /**< PA42 wakeup source  */
    HPAON_WAKEUP_SRC_PIN10,      /**< PA24 wakeup source  */
    HPAON_WAKEUP_SRC_PIN11,      /**< PA25 wakeup source  */
    HPAON_WAKEUP_SRC_PIN12,      /**< PA26 wakeup source  */
    HPAON_WAKEUP_SRC_PIN13,      /**< PA27 wakeup source  */
    HPAON_WAKEUP_SRC_PIN_FIRST = HPAON_WAKEUP_SRC_PA33,
    HPAON_WAKEUP_SRC_PIN_LAST = HPAON_WAKEUP_SRC_PA27,
} HPAON_WakeupSrcTypeDef;

/** @brief lpsys wakeup source */
typedef enum
{
    /* First part, keep enum value same as macro definition to simplify implementation */
    LPAON_WAKEUP_SRC_HP2LP_REQ = LPSYS_AON_WER_HP2LP_REQ_Pos, /**< HP2LP manual wakeup source */
    LPAON_WAKEUP_SRC_HP2LP_IRQ = LPSYS_AON_WER_HP2LP_IRQ_Pos, /**< HP2LP mailbox interrupt wakeup source */
    LPAON_WAKEUP_SRC_LPTIM3    = LPSYS_AON_WER_LPTIM3_Pos,   /**< LPTIM2 wakeup source */
    LPAON_WAKEUP_SRC_BT        = LPSYS_AON_WER_BT_Pos,       /**< BT wakeup source */
} LPAON_WakeupSrcTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_AON_SF32LB57X_H */