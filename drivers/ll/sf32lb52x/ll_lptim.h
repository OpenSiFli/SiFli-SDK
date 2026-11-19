/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_LPTIM_H
#define __LL_LPTIM_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_lptim.h
 * @brief Header-only low-level Low-Power Timer (LPTIM) APIs for SF32LB52x.
 *
 * LPTIM can operate in low-power modes. Supports compare match and auto-reload.
 */

/*==============================================================================
 * Clock Source (CFGR)
 *============================================================================*/

/** @defgroup LL_LPTIM_CLK_SRC Clock Source */
/** @{ */
#define LL_LPTIM_CLK_SRC_LPCLK  (0x0UL)
#define LL_LPTIM_CLK_SRC_PCLK2  (0x1UL)
#define LL_LPTIM_CLK_SRC_INPUT  (0x2UL)
#define LL_LPTIM_CLK_SRC_LPCOMP (0x3UL)
static inline uint32_t ll_lptim_get_compare_wakeup_flag(LPTIM_TypeDef *tim)
{
    return READ_BIT(tim->ISR, LPTIM_ISR_OCWKUP) ? 1UL : 0UL;
}

static inline uint32_t ll_lptim_get_auto_reload_wakeup_flag(LPTIM_TypeDef *tim)
{
    return READ_BIT(tim->ISR, LPTIM_ISR_OFWKUP) ? 1UL : 0UL;
}

static inline uint32_t ll_lptim_get_update_wakeup_flag(LPTIM_TypeDef *tim)
{
    return READ_BIT(tim->ISR, LPTIM_ISR_UEWKUP) ? 1UL : 0UL;
}

static inline uint32_t ll_lptim_get_trigger_flag(LPTIM_TypeDef *tim)
{
    return READ_BIT(tim->ISR, LPTIM_ISR_ET) ? 1UL : 0UL;
}

static inline uint32_t ll_lptim_get_compare_flag(LPTIM_TypeDef *tim)
{
    return READ_BIT(tim->ISR, LPTIM_ISR_OC) ? 1UL : 0UL;
}

static inline uint32_t ll_lptim_get_auto_reload_flag(LPTIM_TypeDef *tim)
{
    return READ_BIT(tim->ISR, LPTIM_ISR_OF) ? 1UL : 0UL;
}

static inline uint32_t ll_lptim_get_update_flag(LPTIM_TypeDef *tim)
{
    return READ_BIT(tim->ISR, LPTIM_ISR_UE) ? 1UL : 0UL;
}

/**
 * @brief Clear all wakeup status flags.
 * @note Wakeup flags must first be disabled in IER before clearing.
 */
static inline void ll_lptim_clear_wakeup_flags(LPTIM_TypeDef *tim)
{
    WRITE_REG(tim->ICR, LPTIM_ICR_WKUPCLR);
}

static inline void ll_lptim_clear_trigger_flag(LPTIM_TypeDef *tim)
{
    WRITE_REG(tim->ICR, LPTIM_ICR_ETCLR);
}

static inline void ll_lptim_clear_compare_flag(LPTIM_TypeDef *tim)
{
    WRITE_REG(tim->ICR, LPTIM_ICR_OCCLR);
}

static inline void ll_lptim_clear_auto_reload_flag(LPTIM_TypeDef *tim)
{
    WRITE_REG(tim->ICR, LPTIM_ICR_OFCLR);
}

static inline void ll_lptim_clear_update_flag(LPTIM_TypeDef *tim)
{
    WRITE_REG(tim->ICR, LPTIM_ICR_UECLR);
}

static inline void ll_lptim_enable_compare_wakeup(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->IER, LPTIM_IER_OCWE);
}

static inline void ll_lptim_disable_compare_wakeup(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->IER, LPTIM_IER_OCWE);
}

static inline void ll_lptim_enable_auto_reload_wakeup(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->IER, LPTIM_IER_OFWE);
}

static inline void ll_lptim_disable_auto_reload_wakeup(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->IER, LPTIM_IER_OFWE);
}

static inline void ll_lptim_enable_update_wakeup(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->IER, LPTIM_IER_UEWE);
}

static inline void ll_lptim_disable_update_wakeup(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->IER, LPTIM_IER_UEWE);
}

static inline void ll_lptim_enable_trigger_interrupt(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->IER, LPTIM_IER_ETIE);
}

static inline void ll_lptim_disable_trigger_interrupt(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->IER, LPTIM_IER_ETIE);
}

/*==============================================================================
 * Interrupt
 *============================================================================*/

static inline void ll_lptim_enable_compare_interrupt(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->IER, LPTIM_IER_OCIE);
}

static inline void ll_lptim_disable_compare_interrupt(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->IER, LPTIM_IER_OCIE);
}

static inline void ll_lptim_enable_auto_reload_interrupt(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->IER, LPTIM_IER_OFIE);
}

static inline void ll_lptim_disable_auto_reload_interrupt(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->IER, LPTIM_IER_OFIE);
}

static inline void ll_lptim_enable_update_interrupt(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->IER, LPTIM_IER_UEIE);
}

static inline void ll_lptim_disable_update_interrupt(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->IER, LPTIM_IER_UEIE);
}

/**
 * @brief Select the LPTIM counter clock mode (CFGR.COUNTMODE).
 * @param[in] tim  LPTIM instance pointer.
 * @param[in] mode Non-zero to count external input clock edges.
 */
static inline void ll_lptim_set_count_mode(LPTIM_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_COUNTMODE,
               mode ? LPTIM_CFGR_COUNTMODE : 0UL);
}

/**
 * @brief Set the output compare waveform polarity (CFGR.WAVPOL).
 * @param[in] tim      LPTIM instance pointer.
 * @param[in] polarity Non-zero to invert the compare waveform.
 */
static inline void ll_lptim_set_wave_polarity(LPTIM_TypeDef *tim, uint32_t polarity)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_WAVPOL,
               polarity ? LPTIM_CFGR_WAVPOL : 0UL);
}

/**
 * @brief Enable or disable set-once waveform mode (CFGR.WAVE).
 * @param[in] tim LPTIM instance pointer.
 * @param[in] en  Non-zero to enable set-once mode.
 */
static inline void ll_lptim_set_waveform(LPTIM_TypeDef *tim, uint32_t en)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_WAVE,
               en ? LPTIM_CFGR_WAVE : 0UL);
}

/**
 * @brief Enable timeout mode (reset counter on match)
 */
static inline void ll_lptim_enable_timeout(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->CFGR, LPTIM_CFGR_TIMOUT);
}

/**
 * @brief Disable timeout mode (reset counter on match)
 */
static inline void ll_lptim_disable_timeout(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CFGR, LPTIM_CFGR_TIMOUT);
}

/**
 * @brief Set the external trigger active edge (CFGR.TRIGEN).
 * @param[in] tim       LPTIM instance pointer.
 * @param[in] trigger_en Trigger edge selection, 0..3.
 */
static inline void ll_lptim_set_trigger_edge(LPTIM_TypeDef *tim, uint32_t trigger_en)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_TRIGEN,
               MAKE_REG_VAL(trigger_en, LPTIM_CFGR_TRIGEN_Msk, LPTIM_CFGR_TRIGEN_Pos));
}

/**
 * @brief Set the external trigger source (CFGR.TRIGSEL).
 * @param[in] tim    LPTIM instance pointer.
 * @param[in] source Trigger source selection, 0..7.
 */
static inline void ll_lptim_set_trigger_source(LPTIM_TypeDef *tim, uint32_t source)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_TRIGSEL,
               MAKE_REG_VAL(source, LPTIM_CFGR_TRIGSEL_Msk, LPTIM_CFGR_TRIGSEL_Pos));
}

/**
 * @brief Set clock prescaler divider (2^div).
 * @param[in] div Division exponent in the range 0 to 7 (divider 1 to 128).
 */
static inline void ll_lptim_set_prescaler(LPTIM_TypeDef *tim, uint32_t div)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_PRESC,
               MAKE_REG_VAL(div, LPTIM_CFGR_PRESC_Msk, LPTIM_CFGR_PRESC_Pos));
}

/*==============================================================================
 * Clock & Mode Configuration
 *============================================================================*/

/**
 * @brief Set clock source
 */
static inline void ll_lptim_set_clock_source(LPTIM_TypeDef *tim, uint32_t src)
{
    switch (src)
    {
    case LL_LPTIM_CLK_SRC_PCLK2:
        MODIFY_REG(tim->CFGR, LPTIM_CFGR_CKSEL | LPTIM_CFGR_INTCKSEL | LPTIM_CFGR_EXTCKSEL,
                   LPTIM_CFGR_INTCKSEL);
        break;
    case LL_LPTIM_CLK_SRC_INPUT:
        MODIFY_REG(tim->CFGR, LPTIM_CFGR_CKSEL | LPTIM_CFGR_INTCKSEL | LPTIM_CFGR_EXTCKSEL,
                   LPTIM_CFGR_CKSEL);
        break;
    case LL_LPTIM_CLK_SRC_LPCOMP:
        MODIFY_REG(tim->CFGR, LPTIM_CFGR_CKSEL | LPTIM_CFGR_INTCKSEL | LPTIM_CFGR_EXTCKSEL,
                   LPTIM_CFGR_CKSEL | LPTIM_CFGR_EXTCKSEL);
        break;
    default:
        MODIFY_REG(tim->CFGR, LPTIM_CFGR_CKSEL | LPTIM_CFGR_INTCKSEL | LPTIM_CFGR_EXTCKSEL,
                   0UL);
        break;
    }
}

/**
 * @brief Set the external trigger digital filter (CFGR.TRGFLT).
 * @param[in] tim    LPTIM instance pointer.
 * @param[in] filter Filter setting, 0..3.
 */
static inline void ll_lptim_set_trigger_filter(LPTIM_TypeDef *tim, uint32_t filter)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_TRGFLT,
               MAKE_REG_VAL(filter, LPTIM_CFGR_TRGFLT_Msk, LPTIM_CFGR_TRGFLT_Pos));
}

/**
 * @brief Set the external clock digital filter (CFGR.CKFLT).
 * @param[in] tim    LPTIM instance pointer.
 * @param[in] filter Filter setting, 0..3.
 */
static inline void ll_lptim_set_clock_filter(LPTIM_TypeDef *tim, uint32_t filter)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_CKFLT,
               MAKE_REG_VAL(filter, LPTIM_CFGR_CKFLT_Msk, LPTIM_CFGR_CKFLT_Pos));
}

/**
 * @brief Set the external clock active edge (CFGR.CKPOL).
 * @param[in] tim       LPTIM instance pointer.
 * @param[in] polarity  Clock polarity selection, 0..2.
 */
static inline void ll_lptim_set_clock_polarity(LPTIM_TypeDef *tim, uint32_t polarity)
{
    MODIFY_REG(tim->CFGR, LPTIM_CFGR_CKPOL,
               MAKE_REG_VAL(polarity, LPTIM_CFGR_CKPOL_Msk, LPTIM_CFGR_CKPOL_Pos));
}

/**
 * @brief Reset the counter.
 * @note COUNTRST is set by software and cleared by hardware. It must not be
 *       set again before hardware has cleared it, so it is only set when the
 *       bit is already cleared.
 */
static inline void ll_lptim_reset_counter(LPTIM_TypeDef *tim)
{
    if (!READ_BIT(tim->CR, LPTIM_CR_COUNTRST))
    {
        SET_BIT(tim->CR, LPTIM_CR_COUNTRST);
    }
}

/**
 * @brief Start counter in continuous mode.
 * @note CR_ENABLE must be set first.
 */
static inline void ll_lptim_start_continuous(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->CR, LPTIM_CR_CNTSTRT);
}

/**
 * @brief Start counter in single-shot mode.
 * @note CR_ENABLE must be set first. In single-shot mode the counter stops at
 *       the next update event; the repetition counter selects the pulse count.
 */
static inline void ll_lptim_start_single(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->CR, LPTIM_CR_SNGSTRT);
}

/** @} */

/*==============================================================================
 * Basic Control
 *============================================================================*/

static inline void ll_lptim_enable(LPTIM_TypeDef *tim)
{
    SET_BIT(tim->CR, LPTIM_CR_ENABLE);
}

static inline void ll_lptim_disable(LPTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR, LPTIM_CR_ENABLE);
}

static inline uint32_t ll_lptim_is_enabled(LPTIM_TypeDef *tim)
{
    return READ_BIT(tim->CR, LPTIM_CR_ENABLE) ? 1UL : 0UL;
}

/*==============================================================================
 * Compare & Auto-Reload
 *============================================================================*/

static inline void ll_lptim_set_compare(LPTIM_TypeDef *tim, uint32_t cmp)
{
    MODIFY_REG(tim->CMP, LPTIM_CMP_CMP,
               MAKE_REG_VAL(cmp, LPTIM_CMP_CMP_Msk, LPTIM_CMP_CMP_Pos));
}

static inline uint32_t ll_lptim_get_compare(LPTIM_TypeDef *tim)
{
    return GET_REG_VAL2(tim->CMP, LPTIM_CMP_CMP);
}

/**
 * @brief Set auto-reload value.
 * @note ARR must be strictly greater than CMP[15:0].
 */
static inline void ll_lptim_set_auto_reload(LPTIM_TypeDef *tim, uint32_t arr)
{
    MODIFY_REG(tim->ARR, LPTIM_ARR_ARR,
               MAKE_REG_VAL(arr, LPTIM_ARR_ARR_Msk, LPTIM_ARR_ARR_Pos));
}

static inline uint32_t ll_lptim_get_auto_reload(LPTIM_TypeDef *tim)
{
    return GET_REG_VAL2(tim->ARR, LPTIM_ARR_ARR);
}

/*==============================================================================
 * Counter
 *============================================================================*/

/**
 * @brief Get the current counter value.
 * @note With an asynchronous clock, read twice until the values match.
 */
static inline uint32_t ll_lptim_get_counter(LPTIM_TypeDef *tim)
{
    return GET_REG_VAL2(tim->CNT, LPTIM_CNT_CNT);
}

/*==============================================================================
 * Repetition (RCR)
 *============================================================================*/

/**
 * @brief Set repetition counter value.
 * @param[in] rep Repetition value; an update event is generated when it reaches
 *                zero.
 */
static inline void ll_lptim_set_repetition(LPTIM_TypeDef *tim, uint8_t rep)
{
    MODIFY_REG(tim->RCR, LPTIM_RCR_REP, rep);
}

/**
 * @brief Get the remaining repetition count.
 * @note With an asynchronous clock, read twice until the values match.
 */
static inline uint32_t ll_lptim_get_repetition(LPTIM_TypeDef *tim)
{
    return GET_REG_VAL2(tim->RCR, LPTIM_RCR_REP);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_LPTIM_H */
