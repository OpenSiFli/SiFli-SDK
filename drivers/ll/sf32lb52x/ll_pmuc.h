/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_PMUC_H
#define __LL_PMUC_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_pmuc.h
 * @brief Header-only low-level PMUC APIs for SF32LB52x.
 */

/** @defgroup LL_PMUC_LPCLK LL PMUC Low-Power Clock Select */
/** @{ */
#define LL_PMUC_LPCLK_LRC10 0x00000000U
#define LL_PMUC_LPCLK_LRC32 PMUC_CR_SEL_LPCLK
/** @} */

/** @defgroup LL_PMUC_WKUP LL PMUC Wakeup Source Mask */
/** @{ */
#define LL_PMUC_WKUP_RTC    PMUC_WER_RTC
#define LL_PMUC_WKUP_WDT1   PMUC_WER_WDT1
#define LL_PMUC_WKUP_WDT2   PMUC_WER_WDT2
#define LL_PMUC_WKUP_PIN0   PMUC_WER_PIN0
#define LL_PMUC_WKUP_PIN1   PMUC_WER_PIN1
#define LL_PMUC_WKUP_LOWBAT PMUC_WER_LOWBAT
#define LL_PMUC_WKUP_CHG    PMUC_WER_CHG
/** @} */

/** @defgroup LL_PMUC_PIN_MODE LL PMUC Wakeup Pin Mode */
/** @{ */
#define LL_PMUC_PIN_MODE_HIGH_LEVEL           0x0U
#define LL_PMUC_PIN_MODE_LOW_LEVEL            0x1U
#define LL_PMUC_PIN_MODE_POS_EDGE             0x2U
#define LL_PMUC_PIN_MODE_NEG_EDGE             0x3U
#define LL_PMUC_PIN_MODE_BOTH_EDGE_HIGH_ACTIVE 0x4U
#define LL_PMUC_PIN_MODE_BOTH_EDGE_LOW_ACTIVE  0x5U
/** @} */

/** @defgroup LL_PMUC_PINSEL LL PMUC Wakeup Pin Select (PA24..PA44) */
/** @{ */
#define LL_PMUC_PINSEL_PA24 0U
#define LL_PMUC_PINSEL_PA25 1U
#define LL_PMUC_PINSEL_PA26 2U
#define LL_PMUC_PINSEL_PA27 3U
#define LL_PMUC_PINSEL_PA28 4U
#define LL_PMUC_PINSEL_PA29 5U
#define LL_PMUC_PINSEL_PA30 6U
#define LL_PMUC_PINSEL_PA31 7U
#define LL_PMUC_PINSEL_PA32 8U
#define LL_PMUC_PINSEL_PA33 9U
#define LL_PMUC_PINSEL_PA34 10U
#define LL_PMUC_PINSEL_PA35 11U
#define LL_PMUC_PINSEL_PA36 12U
#define LL_PMUC_PINSEL_PA37 13U
#define LL_PMUC_PINSEL_PA38 14U
#define LL_PMUC_PINSEL_PA39 15U
#define LL_PMUC_PINSEL_PA40 16U
#define LL_PMUC_PINSEL_PA41 17U
#define LL_PMUC_PINSEL_PA42 18U
#define LL_PMUC_PINSEL_PA43 19U
#define LL_PMUC_PINSEL_PA44 20U
/** @} */

/** @defgroup LL_PMUC_MASK LL PMUC Internal Masks */
/** @{ */
#define LL_PMUC_WER_MASK                                                       \
    (PMUC_WER_RTC | PMUC_WER_WDT1 | PMUC_WER_WDT2 | PMUC_WER_PIN0 |          \
     PMUC_WER_PIN1 | PMUC_WER_LOWBAT | PMUC_WER_CHG)
/** @} */

/**
 * @brief PMUC wakeup pin configuration.
 */
typedef struct
{
    uint32_t pin_sel; /**< Wakeup pin source select value for PA24..PA44. */
    uint32_t mode;    /**< Trigger mode, use @ref LL_PMUC_PIN_MODE_HIGH_LEVEL to @ref
                         LL_PMUC_PIN_MODE_BOTH_EDGE_LOW_ACTIVE. */
} ll_pmuc_wakeup_pin_config_t;

/**
 * @brief PMUC VRET configuration.
 */
typedef struct
{
    uint32_t dly;  /**< VRET startup delay for VRET_CR.DLY field. */
    uint32_t vbit; /**< VRET output setting for VRET_CR.VBIT field. */
    uint32_t trim; /**< VRET trim setting for VRET_CR.TRIM field. */
} ll_pmuc_vret_config_t;

/**
 * @brief Configure wakeup PIN0 select and trigger mode.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] cfg Pointer to wakeup pin configuration.
 */
static inline void
ll_pmuc_config_wakeup_pin0(PMUC_TypeDef *PMUCx,
                           const ll_pmuc_wakeup_pin_config_t *cfg)
{
    MODIFY_REG(PMUCx->CR, PMUC_CR_PIN0_SEL | PMUC_CR_PIN0_MODE,
               ((cfg->pin_sel << PMUC_CR_PIN0_SEL_Pos) & PMUC_CR_PIN0_SEL) |
                   ((cfg->mode << PMUC_CR_PIN0_MODE_Pos) & PMUC_CR_PIN0_MODE));
}

/**
 * @brief Configure wakeup PIN1 select and trigger mode.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] cfg Pointer to wakeup pin configuration.
 */
static inline void
ll_pmuc_config_wakeup_pin1(PMUC_TypeDef *PMUCx,
                           const ll_pmuc_wakeup_pin_config_t *cfg)
{
    MODIFY_REG(PMUCx->CR, PMUC_CR_PIN1_SEL | PMUC_CR_PIN1_MODE,
               ((cfg->pin_sel << PMUC_CR_PIN1_SEL_Pos) & PMUC_CR_PIN1_SEL) |
                   ((cfg->mode << PMUC_CR_PIN1_MODE_Pos) & PMUC_CR_PIN1_MODE));
}

/**
 * @brief Select PMUC low-power clock source.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] sel Clock select value, use @ref LL_PMUC_LPCLK_LRC10 or @ref
 * LL_PMUC_LPCLK_LRC32.
 */
static inline void ll_pmuc_select_lpclk(PMUC_TypeDef *PMUCx, uint32_t sel)
{
    MODIFY_REG(PMUCx->CR, PMUC_CR_SEL_LPCLK, (sel & PMUC_CR_SEL_LPCLK));
}

/**
 * @brief Configure VRET delay/voltage related fields.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] cfg Pointer to VRET configuration.
 */
static inline void ll_pmuc_config_vret(PMUC_TypeDef *PMUCx,
                                       const ll_pmuc_vret_config_t *cfg)
{
    MODIFY_REG(PMUCx->VRET_CR, PMUC_VRET_CR_DLY | PMUC_VRET_CR_VBIT | PMUC_VRET_CR_TRIM,
               ((cfg->dly << PMUC_VRET_CR_DLY_Pos) & PMUC_VRET_CR_DLY) |
                   ((cfg->vbit << PMUC_VRET_CR_VBIT_Pos) & PMUC_VRET_CR_VBIT) |
                   ((cfg->trim << PMUC_VRET_CR_TRIM_Pos) & PMUC_VRET_CR_TRIM));
}

/**
 * @brief Enable pin retention during hibernate.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void
ll_pmuc_enable_pin_retention_in_hibernate(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CR, PMUC_CR_PIN_RET);
}

/**
 * @brief Disable pin retention during hibernate.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void
ll_pmuc_disable_pin_retention_in_hibernate(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CR, PMUC_CR_PIN_RET);
}

/**
 * @brief Check whether pin retention in hibernate is enabled.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when PIN_RET is set.
 */
static inline uint32_t
ll_pmuc_is_pin_retention_in_hibernate_enabled(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->CR, PMUC_CR_PIN_RET);
}

/**
 * @brief Enable PMUC wakeup sources.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] src_mask Wakeup source mask, use @ref LL_PMUC_WKUP_RTC,
 * @ref LL_PMUC_WKUP_WDT1, @ref LL_PMUC_WKUP_WDT2, @ref LL_PMUC_WKUP_PIN0,
 * @ref LL_PMUC_WKUP_PIN1, @ref LL_PMUC_WKUP_LOWBAT, @ref LL_PMUC_WKUP_CHG.
 */
static inline void ll_pmuc_enable_wakeup_source(PMUC_TypeDef *PMUCx,
                                                 uint32_t src_mask)
{
    SET_BIT(PMUCx->WER, (src_mask & LL_PMUC_WER_MASK));
}

/**
 * @brief Disable PMUC wakeup sources.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] src_mask Wakeup source mask, use @ref LL_PMUC_WKUP_RTC,
 * @ref LL_PMUC_WKUP_WDT1, @ref LL_PMUC_WKUP_WDT2, @ref LL_PMUC_WKUP_PIN0,
 * @ref LL_PMUC_WKUP_PIN1, @ref LL_PMUC_WKUP_LOWBAT, @ref LL_PMUC_WKUP_CHG.
 */
static inline void ll_pmuc_disable_wakeup_source(PMUC_TypeDef *PMUCx,
                                                  uint32_t src_mask)
{
    CLEAR_BIT(PMUCx->WER, (src_mask & LL_PMUC_WER_MASK));
}

/**
 * @brief Get currently enabled PMUC wakeup source bits.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Enabled wakeup source mask from WER.
 */
static inline uint32_t ll_pmuc_get_enabled_wakeup_source(PMUC_TypeDef *PMUCx)
{
    return (READ_REG(PMUCx->WER) & LL_PMUC_WER_MASK);
}

/**
 * @brief Enter hibernate mode.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_enter_hibernate(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CR, PMUC_CR_HIBER_EN);
}

/**
 * @brief Clear hibernate flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_hibernate_flag(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CR, PMUC_CR_HIBER_EN);
}

/**
 * @brief Check hibernate flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when hibernate flag is set.
 */
static inline uint32_t ll_pmuc_is_hibernate_flag_set(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->CR, PMUC_CR_HIBER_EN);
}

/**
 * @brief Request software reboot.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_request_reboot(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CR, PMUC_CR_REBOOT);
}

/**
 * @brief Clear reboot flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_reboot_flag(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CR, PMUC_CR_REBOOT);
}

/**
 * @brief Check reboot flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when reboot flag is set.
 */
static inline uint32_t ll_pmuc_is_reboot_flag_set(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->CR, PMUC_CR_REBOOT);
}

/**
 * @brief Read PMUC wakeup status register.
 * @param[in] PMUCx PMUC instance pointer.
 * @return WSR register value.
 */
static inline uint32_t ll_pmuc_get_wakeup_status(PMUC_TypeDef *PMUCx)
{
    return READ_REG(PMUCx->WSR);
}

/**
 * @brief Check RTC wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.RTC is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_rtc(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_RTC);
}

/**
 * @brief Check PIN0 wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.PIN0 is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_pin0(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_PIN0);
}

/**
 * @brief Check PIN1 wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.PIN1 is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_pin1(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_PIN1);
}

/**
 * @brief Check LOWBAT wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.LOWBAT is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_lowbat(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_LOWBAT);
}

/**
 * @brief Check PWRKEY status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.PWRKEY is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_pwrkey(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_PWRKEY);
}

/**
 * @brief Check IWDT status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.IWDT is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_iwdt(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_IWDT);
}

/**
 * @brief Check WDT1 status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.WDT1 is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_wdt1(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_WDT1);
}

/**
 * @brief Check WDT2 status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.WDT2 is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_wdt2(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_WDT2);
}

/**
 * @brief Check CHG status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.CHG is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_chg(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_CHG);
}

/**
 * @brief Clear PIN0 wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_pin0(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_PIN0);
}

/**
 * @brief Clear PIN1 wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_pin1(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_PIN1);
}

/**
 * @brief Clear LOWBAT status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_lowbat(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_LOWBAT);
}

/**
 * @brief Clear PWRKEY status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_pwrkey(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_PWRKEY);
}

/**
 * @brief Clear WDT1 status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_wdt1(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_WDT1);
}

/**
 * @brief Clear WDT2 status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_wdt2(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_WDT2);
}

/**
 * @brief Clear AON wakeup IRQ status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_aon_irq(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_AON);
}

/**
 * @brief Enable LRC10 oscillator.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_enable_lrc10(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->LRC10_CR, PMUC_LRC10_CR_EN);
}

/**
 * @brief Disable LRC10 oscillator.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_disable_lrc10(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->LRC10_CR, PMUC_LRC10_CR_EN);
}

/**
 * @brief Check LRC10 ready status.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when LRC10 is ready.
 */
static inline uint32_t ll_pmuc_is_lrc10_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->LRC10_CR, PMUC_LRC10_CR_RDY);
}

/**
 * @brief Enable LRC32 oscillator.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_enable_lrc32(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->LRC32_CR, PMUC_LRC32_CR_EN);
}

/**
 * @brief Disable LRC32 oscillator.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_disable_lrc32(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->LRC32_CR, PMUC_LRC32_CR_EN);
}

/**
 * @brief Check LRC32 ready status.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when LRC32 is ready.
 */
static inline uint32_t ll_pmuc_is_lrc32_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->LRC32_CR, PMUC_LRC32_CR_RDY);
}

/**
 * @brief Enable LXT32 oscillator.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_enable_lxt32(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->LXT_CR, PMUC_LXT_CR_EN);
}

/**
 * @brief Disable LXT32 oscillator.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_disable_lxt32(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->LXT_CR, PMUC_LXT_CR_EN);
}

/**
 * @brief Check LXT32 ready status.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when LXT32 is ready.
 */
static inline uint32_t ll_pmuc_is_lxt32_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->LXT_CR, PMUC_LXT_CR_RDY);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_PMUC_H */
