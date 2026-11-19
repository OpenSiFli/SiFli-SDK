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
extern "C" {
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
#define LL_PMUC_PIN_MODE_HIGH_LEVEL            0x0U
#define LL_PMUC_PIN_MODE_LOW_LEVEL             0x1U
#define LL_PMUC_PIN_MODE_POS_EDGE              0x2U
#define LL_PMUC_PIN_MODE_NEG_EDGE              0x3U
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
#define LL_PMUC_WER_MASK                                                                           \
    (PMUC_WER_RTC | PMUC_WER_WDT1 | PMUC_WER_WDT2 | PMUC_WER_PIN0 | PMUC_WER_PIN1 |            \
     PMUC_WER_LOWBAT | PMUC_WER_CHG)
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
 * @brief Configure wakeup PIN1 select and trigger mode.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] cfg Pointer to wakeup pin configuration.
 */
static inline void ll_pmuc_config_wakeup_pin1(PMUC_TypeDef *PMUCx,
        const ll_pmuc_wakeup_pin_config_t *cfg)
{
    MODIFY_REG(PMUCx->CR, PMUC_CR_PIN1_SEL | PMUC_CR_PIN1_MODE,
               ((cfg->pin_sel << PMUC_CR_PIN1_SEL_Pos) & PMUC_CR_PIN1_SEL) |
               ((cfg->mode << PMUC_CR_PIN1_MODE_Pos) & PMUC_CR_PIN1_MODE));
}

/**
 * @brief Configure wakeup PIN0 select and trigger mode.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] cfg Pointer to wakeup pin configuration.
 */
static inline void ll_pmuc_config_wakeup_pin0(PMUC_TypeDef *PMUCx,
        const ll_pmuc_wakeup_pin_config_t *cfg)
{
    MODIFY_REG(PMUCx->CR, PMUC_CR_PIN0_SEL | PMUC_CR_PIN0_MODE,
               ((cfg->pin_sel << PMUC_CR_PIN0_SEL_Pos) & PMUC_CR_PIN0_SEL) |
               ((cfg->mode << PMUC_CR_PIN0_MODE_Pos) & PMUC_CR_PIN0_MODE));
}

/**
 * @brief Enable pin retention during hibernate.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_enable_pin_retention_in_hibernate(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CR, PMUC_CR_PIN_RET);
}

/**
 * @brief Disable pin retention during hibernate.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_disable_pin_retention_in_hibernate(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CR, PMUC_CR_PIN_RET);
}

/**
 * @brief Check whether pin retention in hibernate is enabled.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when PIN_RET is set.
 */
static inline uint32_t ll_pmuc_is_pin_retention_in_hibernate_enabled(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->CR, PMUC_CR_PIN_RET);
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
 * @brief Enable PMUC wakeup sources.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] src_mask Wakeup source mask, use @ref LL_PMUC_WKUP_RTC,
 * @ref LL_PMUC_WKUP_WDT1, @ref LL_PMUC_WKUP_WDT2, @ref LL_PMUC_WKUP_PIN0,
 * @ref LL_PMUC_WKUP_PIN1, @ref LL_PMUC_WKUP_LOWBAT, @ref LL_PMUC_WKUP_CHG.
 */
static inline void ll_pmuc_enable_wakeup_source(PMUC_TypeDef *PMUCx, uint32_t src_mask)
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
static inline void ll_pmuc_disable_wakeup_source(PMUC_TypeDef *PMUCx, uint32_t src_mask)
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
 * @brief Read PMUC wakeup status register.
 * @param[in] PMUCx PMUC instance pointer.
 * @return WSR register value.
 */
static inline uint32_t ll_pmuc_get_wakeup_status(PMUC_TypeDef *PMUCx)
{
    return READ_REG(PMUCx->WSR);
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
 * @brief Check PIN1 wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.PIN1 is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_pin1(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_PIN1);
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
 * @brief Check WDT2 status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.WDT2 is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_wdt2(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_WDT2);
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
 * @brief Check RTC wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when WSR.RTC is set.
 */
static inline uint32_t ll_pmuc_is_active_flag_wsr_rtc(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->WSR, PMUC_WSR_RTC);
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
 * @brief Clear PIN1 wakeup status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_pin1(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_PIN1);
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
 * @brief Clear WDT2 status flag.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_clear_flag_wcr_wdt2(PMUC_TypeDef *PMUCx)
{
    WRITE_REG(PMUCx->WCR, PMUC_WCR_WDT2);
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
 * @brief Configure VRET delay/voltage related fields.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] cfg Pointer to VRET configuration.
 */
static inline void ll_pmuc_config_vret(PMUC_TypeDef *PMUCx, const ll_pmuc_vret_config_t *cfg)
{
    MODIFY_REG(PMUCx->VRET_CR, PMUC_VRET_CR_DLY | PMUC_VRET_CR_VBIT | PMUC_VRET_CR_TRIM,
               ((cfg->dly << PMUC_VRET_CR_DLY_Pos) & PMUC_VRET_CR_DLY) |
               ((cfg->vbit << PMUC_VRET_CR_VBIT_Pos) & PMUC_VRET_CR_VBIT) |
               ((cfg->trim << PMUC_VRET_CR_TRIM_Pos) & PMUC_VRET_CR_TRIM));
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
 * @brief Check LRC32 ready status.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when LRC32 is ready.
 */
static inline uint32_t ll_pmuc_is_lrc32_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->LRC32_CR, PMUC_LRC32_CR_RDY);
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
 * @brief Check LXT32 ready status.
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when LXT32 is ready.
 */
static inline uint32_t ll_pmuc_is_lxt32_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->LXT_CR, PMUC_LXT_CR_RDY);
}

/**
 * @brief Configure and enable the LXT32 oscillator (LXT_CR) in one write.
 * @param[in] PMUCx   PMUC instance pointer.
 * @param[in] bm      Bias mode value (BM field).
 * @param[in] amp_bm  Amplifier bias value (AMP_BM field).
 * @param[in] cap_sel Capacitor select value (CAP_SEL field).
 * @param[in] rsn     Reset-while-standby enable (1 bit).
 */
static inline void ll_pmuc_config_lxt32(PMUC_TypeDef *PMUCx, uint32_t bm,
                                        uint32_t amp_bm, uint32_t cap_sel,
                                        uint32_t rsn)
{
    uint32_t value;

    value = MAKE_REG_VAL(bm, PMUC_LXT_CR_BM_Msk, PMUC_LXT_CR_BM_Pos) |
            MAKE_REG_VAL(amp_bm, PMUC_LXT_CR_AMP_BM_Msk, PMUC_LXT_CR_AMP_BM_Pos) |
            MAKE_REG_VAL(cap_sel, PMUC_LXT_CR_CAP_SEL_Msk, PMUC_LXT_CR_CAP_SEL_Pos) |
            (rsn ? PMUC_LXT_CR_RSN : 0U) | PMUC_LXT_CR_EN;
    MODIFY_REG(PMUCx->LXT_CR,
               PMUC_LXT_CR_EN | PMUC_LXT_CR_RSN | PMUC_LXT_CR_CAP_SEL |
               PMUC_LXT_CR_BM_Msk | PMUC_LXT_CR_AMP_BM_Msk,
               value);
}

/**
 * @brief Disable the LXT32 oscillator clearing both EN and RSN (LXT_CR).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_disable_lxt32_rsn(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->LXT_CR, PMUC_LXT_CR_EN | PMUC_LXT_CR_RSN);
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

static inline uint32_t ll_pmuc_buck_is_soft_start_done(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->BUCK_CR1, PMUC_BUCK_CR1_SS_DONE);
}

static inline void ll_pmuc_buck_set_pwm_mode(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->BUCK_CR1, PMUC_BUCK_CR1_CTRL);
}

static inline void ll_pmuc_buck_set_pfm_mode(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->BUCK_CR1, PMUC_BUCK_CR1_CTRL);
}

/*==============================================================================
 * BUCK Converter
 *============================================================================*/

static inline void ll_pmuc_buck_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->BUCK_CR1, PMUC_BUCK_CR1_EN);
}

static inline void ll_pmuc_buck_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->BUCK_CR1, PMUC_BUCK_CR1_EN);
}

static inline void ll_pmuc_buck_set_vout_low(PMUC_TypeDef *PMUCx, uint32_t vout)
{
    MODIFY_REG(PMUCx->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_L,
               MAKE_REG_VAL(vout, PMUC_BUCK_CR2_SET_VOUT_L_Msk, PMUC_BUCK_CR2_SET_VOUT_L_Pos));
}

static inline void ll_pmuc_buck_set_vout_mid(PMUC_TypeDef *PMUCx, uint32_t vout)
{
    MODIFY_REG(PMUCx->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M,
               MAKE_REG_VAL(vout, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
}

/*==============================================================================
 * LDO Control
 *============================================================================*/

/**
 * @brief Set HPSYS LDO output voltage (HPSYS_LDO.VREF[5:2]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vout 4-bit voltage code (must be <= 0xF).
 */
static inline void ll_pmuc_hpsys_ldo_set_vout(PMUC_TypeDef *PMUCx, uint32_t vout)
{
    MODIFY_REG(PMUCx->HPSYS_LDO, PMUC_HPSYS_LDO_VREF,
               MAKE_REG_VAL(vout, PMUC_HPSYS_LDO_VREF_Msk, PMUC_HPSYS_LDO_VREF_Pos));
}

/**
 * @brief Set LPSYS LDO output voltage (LPSYS_LDO.VREF[5:2]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vout 4-bit voltage code (must be <= 0xF).
 */
static inline void ll_pmuc_lpsys_ldo_set_vout(PMUC_TypeDef *PMUCx, uint32_t vout)
{
    MODIFY_REG(PMUCx->LPSYS_LDO, PMUC_LPSYS_LDO_VREF,
               MAKE_REG_VAL(vout, PMUC_LPSYS_LDO_VREF_Msk, PMUC_LPSYS_LDO_VREF_Pos));
}

/**
 * @brief Enable a PERI LDO (PERI_LDO): clear @p pd_mask and set @p en_mask.
 * @param[in] PMUCx   PMUC instance pointer.
 * @param[in] en_mask Enable bit mask of the target LDO.
 * @param[in] pd_mask Power-down bit mask of the target LDO.
 */
static inline void ll_pmuc_peri_ldo_enable(PMUC_TypeDef *PMUCx, uint32_t en_mask,
        uint32_t pd_mask)
{
    MODIFY_REG(PMUCx->PERI_LDO, pd_mask | en_mask, en_mask);
}

/**
 * @brief Disable a PERI LDO (PERI_LDO): clear @p en_mask and set @p pd_mask.
 * @param[in] PMUCx   PMUC instance pointer.
 * @param[in] en_mask Enable bit mask of the target LDO.
 * @param[in] pd_mask Power-down bit mask of the target LDO.
 */
static inline void ll_pmuc_peri_ldo_disable(PMUC_TypeDef *PMUCx, uint32_t en_mask,
        uint32_t pd_mask)
{
    MODIFY_REG(PMUCx->PERI_LDO, pd_mask | en_mask, pd_mask);
}

/**
 * @brief Set PERI VDD33 LDO3 output voltage (PERI_LDO.VDD33_LDO3_SET_VOUT[20:17]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vout 4-bit voltage code (must be <= 0xF).
 */
static inline void ll_pmuc_peri_ldo_set_vdd33_ldo3_vout(PMUC_TypeDef *PMUCx, uint32_t vout)
{
    MODIFY_REG(PMUCx->PERI_LDO, PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT,
               MAKE_REG_VAL(vout, PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk, PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Pos));
}

/**
 * @brief Set PERI VDD33 LDO2 output voltage (PERI_LDO.VDD33_LDO2_SET_VOUT[12:9]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vout 4-bit voltage code (must be <= 0xF).
 */
static inline void ll_pmuc_peri_ldo_set_vdd33_ldo2_vout(PMUC_TypeDef *PMUCx, uint32_t vout)
{
    MODIFY_REG(PMUCx->PERI_LDO, PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT,
               MAKE_REG_VAL(vout, PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk, PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Pos));
}

/**
 * @brief Set PERI LDO18 reference voltage (PERI_LDO.LDO18_VREF_SEL[4:1]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vref 4-bit voltage code (must be <= 0xF).
 */
static inline void ll_pmuc_peri_ldo_set_ldo18_vref(PMUC_TypeDef *PMUCx, uint32_t vref)
{
    MODIFY_REG(PMUCx->PERI_LDO, PMUC_PERI_LDO_LDO18_VREF_SEL,
               MAKE_REG_VAL(vref, PMUC_PERI_LDO_LDO18_VREF_SEL_Msk, PMUC_PERI_LDO_LDO18_VREF_SEL_Pos));
}

/**
 * @brief Set the HPSYS output voltage code (HPSYS_VOUT.VOUT[3:0]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vout 4-bit voltage code (must be <= 0xF).
 */
static inline void ll_pmuc_set_hpsys_vout(PMUC_TypeDef *PMUCx, uint32_t vout)
{
    MODIFY_REG(PMUCx->HPSYS_VOUT, PMUC_HPSYS_VOUT_VOUT,
               MAKE_REG_VAL(vout, PMUC_HPSYS_VOUT_VOUT_Msk, PMUC_HPSYS_VOUT_VOUT_Pos));
}

/*==============================================================================
 * Power Switches
 *============================================================================*/

/** @defgroup LL_PMUC_SWR Power Switch Source Select */
/** @{ */
/** PSW[1:0]: use the RET_LDO supply. */
#define LL_PMUC_SWR_SOURCE_RET_LDO    0U
/** HPSYS_SWR PSW[1:0]: use the HPSYS_LDO supply. */
#define LL_PMUC_SWR_SOURCE_HPSYS_LDO  1U
/** LPSYS_SWR PSW[1:0]: use the LPSYS_LDO supply. */
#define LL_PMUC_SWR_SOURCE_LPSYS_LDO  1U
static inline void ll_pmuc_vrtc_set_trim_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_VRTC_CR_BOR_VT_TRIM | PMUC_VRTC_CR_VRTC_TRIM;

    MODIFY_REG(PMUCx->VRTC_CR, mask, value & mask);
}

static inline void ll_pmuc_vrtc_enable_bor(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->VRTC_CR, PMUC_VRTC_CR_BOR_EN);
}

static inline void ll_pmuc_vrtc_disable_bor(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->VRTC_CR, PMUC_VRTC_CR_BOR_EN);
}

/*==============================================================================
 * VRTC
 *============================================================================*/

static inline void ll_pmuc_vrtc_set_vbit(PMUC_TypeDef *PMUCx, uint32_t vbit)
{
    MODIFY_REG(PMUCx->VRTC_CR, PMUC_VRTC_CR_VRTC_VBIT,
               MAKE_REG_VAL(vbit, PMUC_VRTC_CR_VRTC_VBIT_Msk, PMUC_VRTC_CR_VRTC_VBIT_Pos));
}

static inline uint32_t ll_pmuc_vret_is_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->VRET_CR, PMUC_VRET_CR_RDY);
}

static inline void ll_pmuc_vret_set_control(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_VRET_CR_BM | PMUC_VRET_CR_EN;

    MODIFY_REG(PMUCx->VRET_CR, mask, value & mask);
}

static inline void ll_pmuc_lrc10_set_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_LRC10_CR_REFRES | PMUC_LRC10_CR_CHGCAP |
                          PMUC_LRC10_CR_CHGCRT | PMUC_LRC10_CR_CMPBM2 |
                          PMUC_LRC10_CR_CMPBM1;

    MODIFY_REG(PMUCx->LRC10_CR, mask, value & mask);
}

static inline void ll_pmuc_lrc32_set_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_LRC32_CR_RSEL | PMUC_LRC32_CR_CHGCRT |
                          PMUC_LRC32_CR_CMPBM2 | PMUC_LRC32_CR_CMPBM1;

    MODIFY_REG(PMUCx->LRC32_CR, mask, value & mask);
}

static inline void ll_pmuc_lxt32_set_analog_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_LXT_CR_EXT_EN | PMUC_LXT_CR_BMSTART |
                          PMUC_LXT_CR_BMSEL | PMUC_LXT_CR_AMPCTRL_ENB;

    MODIFY_REG(PMUCx->LXT_CR, mask, value & mask);
}

/*==============================================================================
 * AON Bandgap / LDO
 *============================================================================*/

static inline void ll_pmuc_aon_bg_set_trim(PMUC_TypeDef *PMUCx, uint32_t trim, uint32_t step,
        uint32_t polar)
{
    uint32_t val = MAKE_REG_VAL(trim, PMUC_AON_BG_BUF_VOS_TRIM_Msk, PMUC_AON_BG_BUF_VOS_TRIM_Pos) |
                   MAKE_REG_VAL(step, PMUC_AON_BG_BUF_VOS_STEP_Msk, PMUC_AON_BG_BUF_VOS_STEP_Pos) |
                   MAKE_REG_VAL(polar, PMUC_AON_BG_BUF_VOS_POLAR_Msk, PMUC_AON_BG_BUF_VOS_POLAR_Pos);
    MODIFY_REG(PMUCx->AON_BG,
               PMUC_AON_BG_BUF_VOS_TRIM | PMUC_AON_BG_BUF_VOS_STEP | PMUC_AON_BG_BUF_VOS_POLAR,
               val);
}

static inline void ll_pmuc_aon_ldo_set_por_threshold(PMUC_TypeDef *PMUCx, uint32_t threshold)
{
    MODIFY_REG(PMUCx->AON_LDO, PMUC_AON_LDO_VBAT_POR_TH,
               MAKE_REG_VAL(threshold, PMUC_AON_LDO_VBAT_POR_TH_Msk,
                            PMUC_AON_LDO_VBAT_POR_TH_Pos));
}

static inline void ll_pmuc_aon_ldo_set_vout(PMUC_TypeDef *PMUCx, uint32_t vout)
{
    MODIFY_REG(PMUCx->AON_LDO, PMUC_AON_LDO_VBAT_LDO_SET_VOUT,
               MAKE_REG_VAL(vout, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk, PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos));
}

static inline void ll_pmuc_buck_set_analog_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_BUCK_CR1_BG_BUF_VOS_POLAR |
                          PMUC_BUCK_CR1_BG_BUF_VOS_STEP |
                          PMUC_BUCK_CR1_BG_BUF_VOS_TRIM |
                          PMUC_BUCK_CR1_UVLO_X_BIAS | PMUC_BUCK_CR1_ZCD_AON |
                          PMUC_BUCK_CR1_OCP_AON | PMUC_BUCK_CR1_SEL_LX22 |
                          PMUC_BUCK_CR1_SEL_IOCP_HI | PMUC_BUCK_CR1_IOCP_TUNE |
                          PMUC_BUCK_CR1_COMP_IDYN_TUNE |
                          PMUC_BUCK_CR1_COMP_IQ_TUNE | PMUC_BUCK_CR1_COMP_BM_AHI |
                          PMUC_BUCK_CR1_COT_CTUNE | PMUC_BUCK_CR1_MOT_CTUNE;

    MODIFY_REG(PMUCx->BUCK_CR1, mask, value & mask);
}

static inline void ll_pmuc_buck_set_transition_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_BUCK_CR2_TDIS | PMUC_BUCK_CR2_FORCE_RDY |
                          PMUC_BUCK_CR2_BYPASS_UVLO | PMUC_BUCK_CR2_BYPASS_OCP |
                          PMUC_BUCK_CR2_BYPASS_PG | PMUC_BUCK_CR2_L2M_CNT |
                          PMUC_BUCK_CR2_L2H_CNT | PMUC_BUCK_CR2_M2H_CNT |
                          PMUC_BUCK_CR2_L2M_EN | PMUC_BUCK_CR2_M2L_EN |
                          PMUC_BUCK_CR2_H2L_EN | PMUC_BUCK_CR2_H2M_EN;

    MODIFY_REG(PMUCx->BUCK_CR2, mask, value & mask);
}

/**
 * @brief Set the charger constant-voltage target (CHG_CR1.CV_VCTRL[31:26]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vctrl 6-bit voltage code (must be <= 0x3F).
 */
static inline void ll_pmuc_charger_set_cv_voltage(PMUC_TypeDef *PMUCx, uint32_t vctrl)
{
    MODIFY_REG(PMUCx->CHG_CR1, PMUC_CHG_CR1_CV_VCTRL,
               MAKE_REG_VAL(vctrl, PMUC_CHG_CR1_CV_VCTRL_Msk, PMUC_CHG_CR1_CV_VCTRL_Pos));
}

static inline void ll_pmuc_charger_set_cc_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_CHG_CR1_CC_RANGE | PMUC_CHG_CR1_CC_MN |
                          PMUC_CHG_CR1_CC_MP | PMUC_CHG_CR1_CC_VCTRL |
                          PMUC_CHG_CR1_LOOP_EN;

    MODIFY_REG(PMUCx->CHG_CR1, mask, value & mask);
}

/**
 * @brief Set the charger constant-current target (CHG_CR1.CC_ICTRL[7:2]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] ictrl 6-bit current code (must be <= 0x3F).
 */
static inline void ll_pmuc_charger_set_cc_current(PMUC_TypeDef *PMUCx, uint32_t ictrl)
{
    MODIFY_REG(PMUCx->CHG_CR1, PMUC_CHG_CR1_CC_ICTRL,
               MAKE_REG_VAL(ictrl, PMUC_CHG_CR1_CC_ICTRL_Msk, PMUC_CHG_CR1_CC_ICTRL_Pos));
}

/*==============================================================================
 * Charger Control
 *============================================================================*/

/**
 * @brief Enable the charger (CHG_CR1.EN).
 * @note CHG_CR1.EN is only effective when CHG_CR3.FORCE_CTRL is set
 *       (software-controlled mode), see reference manual page 47.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_charger_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CHG_CR1, PMUC_CHG_CR1_EN);
}

/**
 * @brief Disable the charger (CHG_CR1.EN).
 * @note See @ref ll_pmuc_charger_enable for the CHG_CR3.FORCE_CTRL requirement.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_charger_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CHG_CR1, PMUC_CHG_CR1_EN);
}

/**
 * @brief Set the charger VBAT range (CHG_CR2.VBAT_RANGE).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] range VBAT range (4 bits).
 */
static inline void ll_pmuc_charger_set_vbat_range(PMUC_TypeDef *PMUCx, uint32_t range)
{
    MODIFY_REG(PMUCx->CHG_CR2, PMUC_CHG_CR2_VBAT_RANGE,
               MAKE_REG_VAL(range, PMUC_CHG_CR2_VBAT_RANGE_Msk, PMUC_CHG_CR2_VBAT_RANGE_Pos));
}

static inline void ll_pmuc_charger_set_bg_voltage(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_CHG_CR2_RANGE_EOC | PMUC_CHG_CR2_BG_PROG_V1P2;

    MODIFY_REG(PMUCx->CHG_CR2, mask, value & mask);
}

/**
 * @brief Set the charger end-of-charge bias mode (CHG_CR2.BM_EOC).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] bm    EOC bias mode (3 bits).
 */
static inline void ll_pmuc_charger_set_bm_eoc(PMUC_TypeDef *PMUCx, uint32_t bm)
{
    MODIFY_REG(PMUCx->CHG_CR2, PMUC_CHG_CR2_BM_EOC,
               MAKE_REG_VAL(bm, PMUC_CHG_CR2_BM_EOC_Msk, PMUC_CHG_CR2_BM_EOC_Pos));
}

/**
 * @brief Set the charger high voltage (CHG_CR2.HIGH_VCTRL).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vctrl High voltage control (6 bits).
 */
static inline void ll_pmuc_charger_set_high_voltage(PMUC_TypeDef *PMUCx, uint32_t vctrl)
{
    MODIFY_REG(PMUCx->CHG_CR2, PMUC_CHG_CR2_HIGH_VCTRL,
               MAKE_REG_VAL(vctrl, PMUC_CHG_CR2_HIGH_VCTRL_Msk, PMUC_CHG_CR2_HIGH_VCTRL_Pos));
}

/**
 * @brief Set the charger reprog voltage (CHG_CR2.REP_VCTRL).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vctrl Reprogramming voltage control (6 bits).
 */
static inline void ll_pmuc_charger_set_reprog_voltage(PMUC_TypeDef *PMUCx, uint32_t vctrl)
{
    MODIFY_REG(PMUCx->CHG_CR2, PMUC_CHG_CR2_REP_VCTRL,
               MAKE_REG_VAL(vctrl, PMUC_CHG_CR2_REP_VCTRL_Msk, PMUC_CHG_CR2_REP_VCTRL_Pos));
}

/**
 * @brief Set the charger precharge current (CHG_CR2.PRECC_ICTRL).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] ictrl Precharge current control (6 bits).
 */
static inline void ll_pmuc_charger_set_precc_current(PMUC_TypeDef *PMUCx, uint32_t ictrl)
{
    MODIFY_REG(PMUCx->CHG_CR2, PMUC_CHG_CR2_PRECC_ICTRL,
               MAKE_REG_VAL(ictrl, PMUC_CHG_CR2_PRECC_ICTRL_Msk, PMUC_CHG_CR2_PRECC_ICTRL_Pos));
}

/**
 * @brief Set the charger precharge current range (CHG_CR2.PRECC_RANGE).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] range Precharge current range (2 bits).
 */
static inline void ll_pmuc_charger_set_precc_range(PMUC_TypeDef *PMUCx, uint32_t range)
{
    MODIFY_REG(PMUCx->CHG_CR2, PMUC_CHG_CR2_PRECC_RANGE,
               MAKE_REG_VAL(range, PMUC_CHG_CR2_PRECC_RANGE_Msk, PMUC_CHG_CR2_PRECC_RANGE_Pos));
}

/**
 * @brief Force charger control (CHG_CR3.FORCE_CTRL).
 * @note When FORCE_CTRL is set, the charger settings are controlled by the
 *       register fields instead of the automatic state machine.
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_charger_force_ctrl_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CHG_CR3, PMUC_CHG_CR3_FORCE_CTRL);
}

/**
 * @brief Release charger control to the state machine (CHG_CR3.FORCE_CTRL = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_charger_force_ctrl_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CHG_CR3, PMUC_CHG_CR3_FORCE_CTRL);
}

/**
 * @brief Force the charger reset (CHG_CR3.FORCE_RST).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_charger_force_reset_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CHG_CR3, PMUC_CHG_CR3_FORCE_RST);
}

/**
 * @brief Release the charger force reset (CHG_CR3.FORCE_RST = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_charger_force_reset_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CHG_CR3, PMUC_CHG_CR3_FORCE_RST);
}

/**
 * @brief Set the charger detection delay 2 (CHG_CR3.DLY2).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] dly   Delay value (5 bits).
 */
static inline void ll_pmuc_charger_set_dly2(PMUC_TypeDef *PMUCx, uint32_t dly)
{
    MODIFY_REG(PMUCx->CHG_CR3, PMUC_CHG_CR3_DLY2,
               MAKE_REG_VAL(dly, PMUC_CHG_CR3_DLY2_Msk, PMUC_CHG_CR3_DLY2_Pos));
}

/**
 * @brief Set the charger detection delay 1 (CHG_CR3.DLY1).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] dly   Delay value (6 bits).
 */
static inline void ll_pmuc_charger_set_dly1(PMUC_TypeDef *PMUCx, uint32_t dly)
{
    MODIFY_REG(PMUCx->CHG_CR3, PMUC_CHG_CR3_DLY1,
               MAKE_REG_VAL(dly, PMUC_CHG_CR3_DLY1_Msk, PMUC_CHG_CR3_DLY1_Pos));
}

/**
 * @brief Configure the charger interrupt mode bits (CHG_CR4.IM_*).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] mask  ORed PMUC_CHG_CR4_IM_* masks (0 = high level, 1 = low level,
 *                  2 = pos edge, 3 = neg edge).
 */
static inline void ll_pmuc_charger_config_irq_mode(PMUC_TypeDef *PMUCx, uint32_t mask)
{
    MODIFY_REG(PMUCx->CHG_CR4, PMUC_CHG_CR4_IM_VBUS_RDY | PMUC_CHG_CR4_IM_VBAT_HIGH |
               PMUC_CHG_CR4_IM_ABOVE_REP | PMUC_CHG_CR4_IM_ABOVE_CC |
               PMUC_CHG_CR4_IM_CC_MODE | PMUC_CHG_CR4_IM_CV_MODE |
               PMUC_CHG_CR4_IM_EOC_MODE,
               mask);
}

/**
 * @brief Configure the charger interrupt enable bits (CHG_CR4.IE_*).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] mask  ORed PMUC_CHG_CR4_IE_* masks.
 */
static inline void ll_pmuc_charger_config_irq_enable(PMUC_TypeDef *PMUCx, uint32_t mask)
{
    MODIFY_REG(PMUCx->CHG_CR4, PMUC_CHG_CR4_IE_VBUS_RDY | PMUC_CHG_CR4_IE_VBAT_HIGH |
               PMUC_CHG_CR4_IE_ABOVE_REP | PMUC_CHG_CR4_IE_ABOVE_CC |
               PMUC_CHG_CR4_IE_CC_MODE | PMUC_CHG_CR4_IE_CV_MODE |
               PMUC_CHG_CR4_IE_EOC_MODE | PMUC_CHG_CR4_IE_EOC,
               mask);
}

/**
 * @brief Get the charger interrupt status (CHG_CR5.IS_*).
 * @param[in] PMUCx PMUC instance pointer.
 * @return ORed PMUC_CHG_CR5_IS_* flags.
 */
static inline uint32_t ll_pmuc_charger_get_irq_status(PMUC_TypeDef *PMUCx)
{
    return READ_REG(PMUCx->CHG_CR5) & (PMUC_CHG_CR5_IS_VBUS_RDY | PMUC_CHG_CR5_IS_VBAT_HIGH |
                                       PMUC_CHG_CR5_IS_ABOVE_REP | PMUC_CHG_CR5_IS_ABOVE_CC |
                                       PMUC_CHG_CR5_IS_CC_MODE | PMUC_CHG_CR5_IS_CV_MODE |
                                       PMUC_CHG_CR5_IS_EOC_MODE | PMUC_CHG_CR5_IS_EOC);
}

/**
 * @brief Clear the charger interrupt flags (CHG_CR5.IC_*, write 1 to clear).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] mask  ORed PMUC_CHG_CR5_IC_* masks.
 */
static inline void ll_pmuc_charger_clear_irq(PMUC_TypeDef *PMUCx, uint32_t mask)
{
    const uint32_t valid_mask = PMUC_CHG_CR5_IC_EOC |
                                PMUC_CHG_CR5_IC_EOC_MODE |
                                PMUC_CHG_CR5_IC_CV_MODE |
                                PMUC_CHG_CR5_IC_CC_MODE |
                                PMUC_CHG_CR5_IC_ABOVE_CC |
                                PMUC_CHG_CR5_IC_ABOVE_REP |
                                PMUC_CHG_CR5_IC_VBAT_HIGH |
                                PMUC_CHG_CR5_IC_VBUS_RDY;

    WRITE_REG(PMUCx->CHG_CR5, mask & valid_mask);
}

/**
 * @brief Get the charger status (CHG_SR).
 * @param[in] PMUCx PMUC instance pointer.
 * @return ORed PMUC_CHG_SR_* flags (VBUS_RDY_OUT, VBAT_HIGH_OUT, ... CHG_STATE).
 */
static inline uint32_t ll_pmuc_charger_get_status(PMUC_TypeDef *PMUCx)
{
    const uint32_t status_mask = PMUC_CHG_SR_CHG_STATE |
                                 PMUC_CHG_SR_EOC_MODE | PMUC_CHG_SR_CV_MODE |
                                 PMUC_CHG_SR_CC_MODE |
                                 PMUC_CHG_SR_VBAT_ABOVE_CC_OUT |
                                 PMUC_CHG_SR_VBAT_ABOVE_REP_OUT |
                                 PMUC_CHG_SR_VBAT_HIGH_OUT |
                                 PMUC_CHG_SR_VBUS_RDY_OUT;

    return READ_REG(PMUCx->CHG_SR) & status_mask;
}

static inline uint32_t ll_pmuc_hpsys_ldo_is_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->HPSYS_LDO, PMUC_HPSYS_LDO_RDY);
}

static inline void ll_pmuc_hpsys_ldo_set_control(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_HPSYS_LDO_DLY | PMUC_HPSYS_LDO_VREF2 |
                          PMUC_HPSYS_LDO_BP | PMUC_HPSYS_LDO_EN;

    MODIFY_REG(PMUCx->HPSYS_LDO, mask, value & mask);
}

static inline uint32_t ll_pmuc_lpsys_ldo_is_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->LPSYS_LDO, PMUC_LPSYS_LDO_RDY);
}

static inline void ll_pmuc_lpsys_ldo_set_control(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_LPSYS_LDO_DLY | PMUC_LPSYS_LDO_VREF2 |
                          PMUC_LPSYS_LDO_BP | PMUC_LPSYS_LDO_EN;

    MODIFY_REG(PMUCx->LPSYS_LDO, mask, value & mask);
}

/**
 * @brief Check the HPSYS power switch ready status (HPSYS_SWR.RDY).
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when the switch is ready.
 */
static inline uint32_t ll_pmuc_swr_hpsys_is_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->HPSYS_SWR, PMUC_HPSYS_SWR_RDY);
}

static inline void ll_pmuc_hpsys_swr_set_control(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_HPSYS_SWR_NORET | PMUC_HPSYS_SWR_DLY |
                          PMUC_HPSYS_SWR_PSW_RET;

    MODIFY_REG(PMUCx->HPSYS_SWR, mask, value & mask);
}

/** @} */

/**
 * @brief Select the HPSYS power switch source (HPSYS_SWR.PSW[1:0]).
 * @note HPSYS_SWR has no enable bit; PSW selects the supply feeding the HPSYS
 *       power domain (see reference manual page 49).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] source Power source, see @ref LL_PMUC_SWR_SOURCE_*.
 */
static inline void ll_pmuc_swr_hpsys_set_power_source(PMUC_TypeDef *PMUCx, uint32_t source)
{
    MODIFY_REG(PMUCx->HPSYS_SWR, PMUC_HPSYS_SWR_PSW,
               MAKE_REG_VAL(source, PMUC_HPSYS_SWR_PSW_Msk, PMUC_HPSYS_SWR_PSW_Pos));
}

/**
 * @brief Check the LPSYS power switch ready status (LPSYS_SWR.RDY).
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when the switch is ready.
 */
static inline uint32_t ll_pmuc_swr_lpsys_is_ready(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->LPSYS_SWR, PMUC_LPSYS_SWR_RDY);
}

static inline void ll_pmuc_lpsys_swr_set_control(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_LPSYS_SWR_NORET | PMUC_LPSYS_SWR_DLY |
                          PMUC_LPSYS_SWR_PSW_RET;

    MODIFY_REG(PMUCx->LPSYS_SWR, mask, value & mask);
}

/**
 * @brief Select the LPSYS power switch source (LPSYS_SWR.PSW[1:0]).
 * @note LPSYS_SWR has no enable bit; PSW selects the supply feeding the LPSYS
 *       power domain (see reference manual page 49).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] source Power source, see @ref LL_PMUC_SWR_SOURCE_*.
 */
static inline void ll_pmuc_swr_lpsys_set_power_source(PMUC_TypeDef *PMUCx, uint32_t source)
{
    MODIFY_REG(PMUCx->LPSYS_SWR, PMUC_LPSYS_SWR_PSW,
               MAKE_REG_VAL(source, PMUC_LPSYS_SWR_PSW_Msk, PMUC_LPSYS_SWR_PSW_Pos));
}

static inline void ll_pmuc_peri_ldo_set_control(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_PERI_LDO_VDD33_LDO3_PD |
                          PMUC_PERI_LDO_EN_VDD33_LDO3 |
                          PMUC_PERI_LDO_VDD33_LDO2_PD |
                          PMUC_PERI_LDO_EN_VDD33_LDO2 | PMUC_PERI_LDO_LDO18_PD |
                          PMUC_PERI_LDO_EN_LDO18;

    MODIFY_REG(PMUCx->PERI_LDO, mask, value & mask);
}

/**
 * @brief Set the PMU DC test point and macro select (PMU_TR).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] tr    DC test point select (3 bits).
 * @param[in] mr    DC macro select (3 bits).
 */
static inline void ll_pmuc_set_dc_test(PMUC_TypeDef *PMUCx, uint32_t tr, uint32_t mr)
{
    MODIFY_REG(PMUCx->PMU_TR, PMUC_PMU_TR_PMU_DC_TR | PMUC_PMU_TR_PMU_DC_MR,
               MAKE_REG_VAL(tr, PMUC_PMU_TR_PMU_DC_TR_Msk, PMUC_PMU_TR_PMU_DC_TR_Pos) |
               MAKE_REG_VAL(mr, PMUC_PMU_TR_PMU_DC_MR_Msk, PMUC_PMU_TR_PMU_DC_MR_Pos));
}

static inline uint32_t ll_pmuc_get_pmu_reserved(PMUC_TypeDef *PMUCx)
{
    const uint32_t mask = PMUC_PMU_RSVD_RESERVE3 | PMUC_PMU_RSVD_RESERVE2 |
                          PMUC_PMU_RSVD_RESERVE1 | PMUC_PMU_RSVD_RESERVE0;

    return READ_REG(PMUCx->PMU_RSVD) & mask;
}

static inline void ll_pmuc_set_pmu_reserved(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_PMU_RSVD_RESERVE2 | PMUC_PMU_RSVD_RESERVE1 |
                          PMUC_PMU_RSVD_RESERVE0;

    MODIFY_REG(PMUCx->PMU_RSVD, mask, value & mask);
}

static inline void ll_pmuc_hxt48_set_analog_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_HXT_CR1_CBANK_SEL | PMUC_HXT_CR1_GM_EN |
                          PMUC_HXT_CR1_LDO_FLT_RSEL | PMUC_HXT_CR1_LDO_VREF |
                          PMUC_HXT_CR1_BUF_RF_STR | PMUC_HXT_CR1_BUF_DLL_STR |
                          PMUC_HXT_CR1_BUF_EN;

    MODIFY_REG(PMUCx->HXT_CR1, mask, value & mask);
}

/**
 * @brief Set the HXT48 audio buffer drive strength (HXT_CR1.BUF_AUD_STR[10:9]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] str 2-bit strength code (must be <= 0x3).
 */
static inline void ll_pmuc_hxt48_set_audio_buf_strength(PMUC_TypeDef *PMUCx, uint32_t str)
{
    MODIFY_REG(PMUCx->HXT_CR1, PMUC_HXT_CR1_BUF_AUD_STR,
               MAKE_REG_VAL(str, PMUC_HXT_CR1_BUF_AUD_STR_Msk, PMUC_HXT_CR1_BUF_AUD_STR_Pos));
}

/**
 * @brief Enable the HXT48 audio buffer (HXT_CR1.BUF_AUD_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_enable_audio_buf(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->HXT_CR1, PMUC_HXT_CR1_BUF_AUD_EN);
}

/**
 * @brief Disable the HXT48 audio buffer (HXT_CR1.BUF_AUD_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_disable_audio_buf(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->HXT_CR1, PMUC_HXT_CR1_BUF_AUD_EN);
}

/**
 * @brief Enable the HXT48 DLL buffer (HXT_CR1.BUF_DLL_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_enable_dll_buf(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->HXT_CR1, PMUC_HXT_CR1_BUF_DLL_EN);
}

/**
 * @brief Disable the HXT48 DLL buffer (HXT_CR1.BUF_DLL_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_disable_dll_buf(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->HXT_CR1, PMUC_HXT_CR1_BUF_DLL_EN);
}

/**
 * @brief Set the HXT48 digital buffer drive strength (HXT_CR1.BUF_DIG_STR[4:3]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] str 2-bit strength code (must be <= 0x3).
 */
static inline void ll_pmuc_hxt48_set_digital_buf_strength(PMUC_TypeDef *PMUCx, uint32_t str)
{
    MODIFY_REG(PMUCx->HXT_CR1, PMUC_HXT_CR1_BUF_DIG_STR,
               MAKE_REG_VAL(str, PMUC_HXT_CR1_BUF_DIG_STR_Msk, PMUC_HXT_CR1_BUF_DIG_STR_Pos));
}

/**
 * @brief Enable the HXT48 digital clock output buffer (HXT_CR1.BUF_DIG_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_enable_clock_out(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->HXT_CR1, PMUC_HXT_CR1_BUF_DIG_EN);
}

/**
 * @brief Disable the HXT48 digital clock output buffer (HXT_CR1.BUF_DIG_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_disable_clock_out(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->HXT_CR1, PMUC_HXT_CR1_BUF_DIG_EN);
}

/*==============================================================================
 * HXT48 Oscillator
 *============================================================================*/

/**
 * @brief Enable the HXT48 oscillator (HXT_CR1.EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->HXT_CR1, PMUC_HXT_CR1_EN);
}

/**
 * @brief Disable the HXT48 oscillator (HXT_CR1.EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->HXT_CR1, PMUC_HXT_CR1_EN);
}

/**
 * @brief Enable the HXT48 sleep mode (HXT_CR2.SLEEP_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_sleep_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->HXT_CR2, PMUC_HXT_CR2_SLEEP_EN);
}

/**
 * @brief Disable the HXT48 sleep mode (HXT_CR2.SLEEP_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_sleep_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->HXT_CR2, PMUC_HXT_CR2_SLEEP_EN);
}

static inline void ll_pmuc_hxt48_set_clock_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_HXT_CR2_SDADC_CLKDIV2_SEL |
                          PMUC_HXT_CR2_SDADC_CLKDIV1_SEL |
                          PMUC_HXT_CR2_SDADC_CLKIN_EN | PMUC_HXT_CR2_BUF_SEL3 |
                          PMUC_HXT_CR2_BUF_SEL2 | PMUC_HXT_CR2_ACBUF_RSEL |
                          PMUC_HXT_CR2_AGC_ISTART_SEL;

    MODIFY_REG(PMUCx->HXT_CR2, mask, value & mask);
}

/**
 * @brief Set the HXT48 IDAC value (HXT_CR2.IDAC).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] idac  IDAC value (10 bits).
 */
static inline void ll_pmuc_hxt48_set_idac(PMUC_TypeDef *PMUCx, uint32_t idac)
{
    MODIFY_REG(PMUCx->HXT_CR2, PMUC_HXT_CR2_IDAC,
               MAKE_REG_VAL(idac, PMUC_HXT_CR2_IDAC_Msk, PMUC_HXT_CR2_IDAC_Pos));
}

/**
 * @brief Enable the HXT48 IDAC (HXT_CR2.IDAC_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_idac_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->HXT_CR2, PMUC_HXT_CR2_IDAC_EN);
}

/**
 * @brief Disable the HXT48 IDAC (HXT_CR2.IDAC_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_idac_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->HXT_CR2, PMUC_HXT_CR2_IDAC_EN);
}

/**
 * @brief Set the HXT48 AC buffer select (HXT_CR2.ACBUF_SEL).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] sel   AC buffer select (2 bits).
 */
static inline void ll_pmuc_hxt48_set_acbuf_sel(PMUC_TypeDef *PMUCx, uint32_t sel)
{
    MODIFY_REG(PMUCx->HXT_CR2, PMUC_HXT_CR2_ACBUF_SEL,
               MAKE_REG_VAL(sel, PMUC_HXT_CR2_ACBUF_SEL_Msk, PMUC_HXT_CR2_ACBUF_SEL_Pos));
}

/**
 * @brief Set the HXT48 AGC VIN DC (HXT_CR2.AGC_VINDC).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vindc AGC VIN DC (2 bits).
 */
static inline void ll_pmuc_hxt48_set_agc_vindc(PMUC_TypeDef *PMUCx, uint32_t vindc)
{
    MODIFY_REG(PMUCx->HXT_CR2, PMUC_HXT_CR2_AGC_VINDC,
               MAKE_REG_VAL(vindc, PMUC_HXT_CR2_AGC_VINDC_Msk, PMUC_HXT_CR2_AGC_VINDC_Pos));
}

/**
 * @brief Set the HXT48 AGC threshold (HXT_CR2.AGC_VTH).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vth   AGC threshold (4 bits).
 */
static inline void ll_pmuc_hxt48_set_agc_vth(PMUC_TypeDef *PMUCx, uint32_t vth)
{
    MODIFY_REG(PMUCx->HXT_CR2, PMUC_HXT_CR2_AGC_VTH,
               MAKE_REG_VAL(vth, PMUC_HXT_CR2_AGC_VTH_Msk, PMUC_HXT_CR2_AGC_VTH_Pos));
}

/**
 * @brief Enable the HXT48 AGC (HXT_CR2.AGC_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_agc_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->HXT_CR2, PMUC_HXT_CR2_AGC_EN);
}

/**
 * @brief Disable the HXT48 AGC (HXT_CR2.AGC_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_hxt48_agc_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->HXT_CR2, PMUC_HXT_CR2_AGC_EN);
}

/**
 * @brief Set the HXT48 delay (HXT_CR3.DLY).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] dly   Delay value (6 bits).
 */
static inline void ll_pmuc_hxt48_set_dly(PMUC_TypeDef *PMUCx, uint32_t dly)
{
    MODIFY_REG(PMUCx->HXT_CR3, PMUC_HXT_CR3_DLY,
               MAKE_REG_VAL(dly, PMUC_HXT_CR3_DLY_Msk, PMUC_HXT_CR3_DLY_Pos));
}

/**
 * @brief Set the HXT48 buffer oscillator strength (HXT_CR3.BUF_OSLO_STR).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] str   Buffer strength (2 bits).
 */
static inline void ll_pmuc_hxt48_set_buf_oslo_str(PMUC_TypeDef *PMUCx, uint32_t str)
{
    MODIFY_REG(PMUCx->HXT_CR3, PMUC_HXT_CR3_BUF_OSLO_STR,
               MAKE_REG_VAL(str, PMUC_HXT_CR3_BUF_OSLO_STR_Msk, PMUC_HXT_CR3_BUF_OSLO_STR_Pos));
}

/**
 * @brief Set the HXT48 buffer DAC strength (HXT_CR3.BUF_DAC_STR).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] str   Buffer strength (2 bits).
 */
static inline void ll_pmuc_hxt48_set_buf_dac_str(PMUC_TypeDef *PMUCx, uint32_t str)
{
    MODIFY_REG(PMUCx->HXT_CR3, PMUC_HXT_CR3_BUF_DAC_STR,
               MAKE_REG_VAL(str, PMUC_HXT_CR3_BUF_DAC_STR_Msk, PMUC_HXT_CR3_BUF_DAC_STR_Pos));
}

static inline void ll_pmuc_hrc48_set_config(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_HRC_CR_DLY | PMUC_HRC_CR_CLKLP_STR |
                          PMUC_HRC_CR_CLKLP_SEL | PMUC_HRC_CR_CLKLP_EN |
                          PMUC_HRC_CR_CLKHP_STR | PMUC_HRC_CR_CLKHP_SEL |
                          PMUC_HRC_CR_CLKHP_EN | PMUC_HRC_CR_CLK96M_EN |
                          PMUC_HRC_CR_TEMP_TRIM | PMUC_HRC_CR_LDO_VREF |
                          PMUC_HRC_CR_EN;

    MODIFY_REG(PMUCx->HRC_CR, mask, value & mask);
}

/*==============================================================================
 * HRC48 Oscillator
 *============================================================================*/

/**
 * @brief Set the HRC48 frequency trim (HRC_CR.FREQ_TRIM[14:5], 10 bits).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] trim 10-bit frequency trim code (must be <= 0x3FF).
 */
static inline void ll_pmuc_hrc48_set_trim(PMUC_TypeDef *PMUCx, uint32_t trim)
{
    MODIFY_REG(PMUCx->HRC_CR, PMUC_HRC_CR_FREQ_TRIM,
               MAKE_REG_VAL(trim, PMUC_HRC_CR_FREQ_TRIM_Msk, PMUC_HRC_CR_FREQ_TRIM_Pos));
}

/**
 * @brief Set the DBL96 external delay select (DBL96_CR.DLY_SEL_EXT).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] sel   External delay select (11 bits).
 */
static inline void ll_pmuc_dbl96_set_dly_sel_ext(PMUC_TypeDef *PMUCx, uint32_t sel)
{
    MODIFY_REG(PMUCx->DBL96_CR, PMUC_DBL96_CR_DLY_SEL_EXT,
               MAKE_REG_VAL(sel, PMUC_DBL96_CR_DLY_SEL_EXT_Msk, PMUC_DBL96_CR_DLY_SEL_EXT_Pos));
}

static inline void ll_pmuc_dbl96_set_control(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_DBL96_CR_DLY_SEL_EXT_EN |
                          PMUC_DBL96_CR_DLY_EXT_EN | PMUC_DBL96_CR_TOOSLO_EN |
                          PMUC_DBL96_CR_TORF_EN | PMUC_DBL96_CR_TODIG_EN;

    MODIFY_REG(PMUCx->DBL96_CR, mask, value & mask);
}

/**
 * @brief Enable the DBL96 delay (DBL96_CR.DLY_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_dly_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->DBL96_CR, PMUC_DBL96_CR_DLY_EN);
}

/**
 * @brief Disable the DBL96 delay (DBL96_CR.DLY_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_dly_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->DBL96_CR, PMUC_DBL96_CR_DLY_EN);
}

/**
 * @brief Set the DBL96 phase enable mask (DBL96_CR.PH_EN).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] mask  Phase enable mask (4 bits).
 */
static inline void ll_pmuc_dbl96_set_ph_enable(PMUC_TypeDef *PMUCx, uint32_t mask)
{
    MODIFY_REG(PMUCx->DBL96_CR, PMUC_DBL96_CR_PH_EN,
               MAKE_REG_VAL(mask, PMUC_DBL96_CR_PH_EN_Msk, PMUC_DBL96_CR_PH_EN_Pos));
}

/**
 * @brief Enable the DBL96 loop reset (assert LOOP_RSTB low).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_loop_assert_reset(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->DBL96_CR, PMUC_DBL96_CR_LOOP_RSTB);
}

/**
 * @brief Release the DBL96 loop reset (LOOP_RSTB = 1).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_loop_release_reset(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->DBL96_CR, PMUC_DBL96_CR_LOOP_RSTB);
}

/**
 * @brief Set the DBL96 to-digital strength (DBL96_CR.TODIG_STR).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] str   Strength (2 bits).
 */
static inline void ll_pmuc_dbl96_set_todig_str(PMUC_TypeDef *PMUCx, uint32_t str)
{
    MODIFY_REG(PMUCx->DBL96_CR, PMUC_DBL96_CR_TODIG_STR,
               MAKE_REG_VAL(str, PMUC_DBL96_CR_TODIG_STR_Msk, PMUC_DBL96_CR_TODIG_STR_Pos));
}

/**
 * @brief Enable the DBL96 output (DBL96_CR.OUT_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_out_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->DBL96_CR, PMUC_DBL96_CR_OUT_EN);
}

/**
 * @brief Disable the DBL96 output (DBL96_CR.OUT_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_out_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->DBL96_CR, PMUC_DBL96_CR_OUT_EN);
}

/**
 * @brief Enable the DBL96 doubler (DBL96_CR.EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->DBL96_CR, PMUC_DBL96_CR_EN);
}

/**
 * @brief Disable the DBL96 doubler (DBL96_CR.EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->DBL96_CR, PMUC_DBL96_CR_EN);
}

/**
 * @brief Check whether the DBL96 calibration is locked (DBL96_CALR.CAL_LOCK).
 * @param[in] PMUCx PMUC instance pointer.
 * @return Non-zero when locked.
 */
static inline uint32_t ll_pmuc_dbl96_is_cal_locked(PMUC_TypeDef *PMUCx)
{
    return READ_BIT(PMUCx->DBL96_CALR, PMUC_DBL96_CALR_CAL_LOCK) ? 1UL : 0UL;
}

/**
 * @brief Get the DBL96 calibration result (DBL96_CALR.CAL_OP).
 * @param[in] PMUCx PMUC instance pointer.
 * @return Calibration operation result (11 bits).
 */
static inline uint32_t ll_pmuc_dbl96_get_cal_result(PMUC_TypeDef *PMUCx)
{
    return GET_REG_VAL2(PMUCx->DBL96_CALR, PMUC_DBL96_CALR_CAL_OP);
}

static inline void ll_pmuc_dbl96_set_cal_close_ext(PMUC_TypeDef *PMUCx, uint32_t en)
{
    MODIFY_REG(PMUCx->DBL96_CALR, PMUC_DBL96_CALR_CAL_CLOSE_EXT_EN,
               en ? PMUC_DBL96_CALR_CAL_CLOSE_EXT_EN : 0UL);
}

/**
 * @brief Enable the DBL96 calibration (DBL96_CALR.CAL_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_cal_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->DBL96_CALR, PMUC_DBL96_CALR_CAL_EN);
}

/**
 * @brief Disable the DBL96 calibration (DBL96_CALR.CAL_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_dbl96_cal_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->DBL96_CALR, PMUC_DBL96_CALR_CAL_EN);
}

/**
 * @brief Set the LPSYS bandgap 1.2V reference (CAU_BGR.LPBG_VREF12).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vref  Reference trim (4 bits).
 */
static inline void ll_pmuc_cau_set_lpbg_vref12(PMUC_TypeDef *PMUCx, uint32_t vref)
{
    MODIFY_REG(PMUCx->CAU_BGR, PMUC_CAU_BGR_LPBG_VREF12,
               MAKE_REG_VAL(vref, PMUC_CAU_BGR_LPBG_VREF12_Msk, PMUC_CAU_BGR_LPBG_VREF12_Pos));
}

/**
 * @brief Set the LPSYS bandgap 0.6V reference (CAU_BGR.LPBG_VREF06).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] vref  Reference trim (4 bits).
 */
static inline void ll_pmuc_cau_set_lpbg_vref06(PMUC_TypeDef *PMUCx, uint32_t vref)
{
    MODIFY_REG(PMUCx->CAU_BGR, PMUC_CAU_BGR_LPBG_VREF06,
               MAKE_REG_VAL(vref, PMUC_CAU_BGR_LPBG_VREF06_Msk, PMUC_CAU_BGR_LPBG_VREF06_Pos));
}

/**
 * @brief Enable the LPSYS bandgap (CAU_BGR.LPBG_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_cau_lpbg_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CAU_BGR, PMUC_CAU_BGR_LPBG_EN);
}

/**
 * @brief Disable the LPSYS bandgap (CAU_BGR.LPBG_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_cau_lpbg_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CAU_BGR, PMUC_CAU_BGR_LPBG_EN);
}

/**
 * @brief Enable the HPSYS bandgap (CAU_BGR.HPBG_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_cau_hpbg_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CAU_BGR, PMUC_CAU_BGR_HPBG_EN);
}

/**
 * @brief Disable the HPSYS bandgap (CAU_BGR.HPBG_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_cau_hpbg_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CAU_BGR, PMUC_CAU_BGR_HPBG_EN);
}

/**
 * @brief Enable the HPSYS bandgap VDDPSW (CAU_BGR.HPBG_VDDPSW_EN).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_cau_hpbg_vddpsw_enable(PMUC_TypeDef *PMUCx)
{
    SET_BIT(PMUCx->CAU_BGR, PMUC_CAU_BGR_HPBG_VDDPSW_EN);
}

/**
 * @brief Disable the HPSYS bandgap VDDPSW (CAU_BGR.HPBG_VDDPSW_EN = 0).
 * @param[in] PMUCx PMUC instance pointer.
 */
static inline void ll_pmuc_cau_hpbg_vddpsw_disable(PMUC_TypeDef *PMUCx)
{
    CLEAR_BIT(PMUCx->CAU_BGR, PMUC_CAU_BGR_HPBG_VDDPSW_EN);
}

/**
 * @brief Set the CAU DC test (CAU_TR).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] tr    DC test point select (3 bits).
 * @param[in] br    DC test block select (3 bits).
 * @param[in] mr    DC test macro select (3 bits).
 */
static inline void ll_pmuc_cau_set_dc_test(PMUC_TypeDef *PMUCx, uint32_t tr, uint32_t br,
        uint32_t mr)
{
    MODIFY_REG(PMUCx->CAU_TR, PMUC_CAU_TR_CAU_DC_TR | PMUC_CAU_TR_CAU_DC_BR |
               PMUC_CAU_TR_CAU_DC_MR,
               MAKE_REG_VAL(tr, PMUC_CAU_TR_CAU_DC_TR_Msk, PMUC_CAU_TR_CAU_DC_TR_Pos) |
               MAKE_REG_VAL(br, PMUC_CAU_TR_CAU_DC_BR_Msk, PMUC_CAU_TR_CAU_DC_BR_Pos) |
               MAKE_REG_VAL(mr, PMUC_CAU_TR_CAU_DC_MR_Msk, PMUC_CAU_TR_CAU_DC_MR_Pos));
}

static inline uint32_t ll_pmuc_get_cau_reserved(PMUC_TypeDef *PMUCx)
{
    const uint32_t mask = PMUC_CAU_RSVD_RESERVE2 | PMUC_CAU_RSVD_RESERVE1 |
                          PMUC_CAU_RSVD_RESERVE0;

    return READ_REG(PMUCx->CAU_RSVD) & mask;
}

static inline void ll_pmuc_set_cau_reserved(PMUC_TypeDef *PMUCx, uint32_t value)
{
    const uint32_t mask = PMUC_CAU_RSVD_RESERVE1 | PMUC_CAU_RSVD_RESERVE0;

    MODIFY_REG(PMUCx->CAU_RSVD, mask, value & mask);
}

/*==============================================================================
 * Counters
 *============================================================================*/

/**
 * @brief Set the wakeup pin debounce counts (WKUP_CNT).
 * @note The value is packed as PIN0_CNT[15:0] | (PIN1_CNT[15:0] << 16),
 *       i.e. cnt = (pin1_cnt << 16) | pin0_cnt.
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] cnt Packed 16-bit debounce counts for PIN0 and PIN1.
 */
static inline void ll_pmuc_set_wkup_count(PMUC_TypeDef *PMUCx, uint32_t cnt)
{
    WRITE_REG(PMUCx->WKUP_CNT,
              cnt & (PMUC_WKUP_CNT_PIN1_CNT | PMUC_WKUP_CNT_PIN0_CNT));
}

/**
 * @brief Set the power-key reset debounce count (PWRKEY_CNT.RST_CNT[19:4]).
 * @param[in] PMUCx PMUC instance pointer.
 * @param[in] cnt 16-bit debounce count (must be <= 0xFFFF).
 */
static inline void ll_pmuc_set_pwrkey_count(PMUC_TypeDef *PMUCx, uint32_t cnt)
{
    WRITE_REG(PMUCx->PWRKEY_CNT, MAKE_REG_VAL(cnt, PMUC_PWRKEY_CNT_RST_CNT_Msk, PMUC_PWRKEY_CNT_RST_CNT_Pos));
}

/*==============================================================================
 * Voltage Monitoring
 *============================================================================*/

/**
 * @brief Get the HPSYS LDO output voltage code (HPSYS_VOUT.VOUT[3:0]).
 * @param[in] PMUCx PMUC instance pointer.
 * @return 4-bit voltage code.
 */
static inline uint8_t ll_pmuc_get_hpsys_vout(PMUC_TypeDef *PMUCx)
{
    return (uint8_t)(READ_REG(PMUCx->HPSYS_VOUT) & PMUC_HPSYS_VOUT_VOUT_Msk);
}

/**
 * @brief Get the LPSYS LDO output voltage code (LPSYS_VOUT.VOUT[3:0]).
 * @param[in] PMUCx PMUC instance pointer.
 * @return 4-bit voltage code.
 */
static inline uint8_t ll_pmuc_get_lpsys_vout(PMUC_TypeDef *PMUCx)
{
    return (uint8_t)(READ_REG(PMUCx->LPSYS_VOUT) & PMUC_LPSYS_VOUT_VOUT_Msk);
}

/**
 * @brief Get the BUCK output voltage code (BUCK_VOUT.VOUT[3:0]).
 * @param[in] PMUCx PMUC instance pointer.
 * @return 4-bit voltage code.
 */
static inline uint8_t ll_pmuc_get_buck_vout(PMUC_TypeDef *PMUCx)
{
    return (uint8_t)(READ_REG(PMUCx->BUCK_VOUT) & PMUC_BUCK_VOUT_VOUT_Msk);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_PMUC_H */
