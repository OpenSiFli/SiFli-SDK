/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_ATIM_H
#define __LL_ATIM_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_atim.h
 * @brief Header-only low-level Advanced Timer (ATIM) APIs for SF32LB52x.
 *
 * ATIM extends GPTIM with complementary outputs (CHxN), dead-time insertion,
 * break input, and 6 capture/compare channels.
 */

/*==============================================================================
 * Counter Mode (CR1.CMS)
 *============================================================================*/

/** @defgroup LL_ATIM_COUNTER_MODE Counter Alignment Mode */
/** @{ */
#define LL_ATIM_COUNTER_MODE_EDGE    (0x0UL)
#define LL_ATIM_COUNTER_MODE_CENTER1 (0x1UL)
#define LL_ATIM_COUNTER_MODE_CENTER2 (0x2UL)
#define LL_ATIM_COUNTER_MODE_CENTER3 (0x3UL)
/** @} */

/** @defgroup LL_ATIM_COUNTER_DIR Counter Direction */
/** @{ */
#define LL_ATIM_COUNTER_DIR_UP   (0x0UL)
#define LL_ATIM_COUNTER_DIR_DOWN (0x1UL)
/** @} */

/*==============================================================================
 * OC Mode (CCMRx.OCxM)
 *============================================================================*/

/** @defgroup LL_ATIM_OC_MODE Output Compare Mode */
/** @{ */
#define LL_ATIM_OC_MODE_FROZEN            (0x0UL)
#define LL_ATIM_OC_MODE_ACTIVE_ON_MATCH   (0x1UL)
#define LL_ATIM_OC_MODE_INACTIVE_ON_MATCH (0x2UL)
#define LL_ATIM_OC_MODE_TOGGLE            (0x3UL)
#define LL_ATIM_OC_MODE_FORCE_INACTIVE    (0x4UL)
#define LL_ATIM_OC_MODE_FORCE_ACTIVE      (0x5UL)
#define LL_ATIM_OC_MODE_PWM1              (0x6UL)
#define LL_ATIM_OC_MODE_PWM2              (0x7UL)
#define LL_ATIM_OC_MODE_RETRIG_OPM1       (0x8UL)
#define LL_ATIM_OC_MODE_RETRIG_OPM2       (0x9UL)
#define LL_ATIM_OC_MODE_COMBINED_PWM1     (0xCUL)
#define LL_ATIM_OC_MODE_COMBINED_PWM2     (0xDUL)
#define LL_ATIM_OC_MODE_ASYMMETRIC_PWM1   (0xEUL)
#define LL_ATIM_OC_MODE_ASYMMETRIC_PWM2   (0xFUL)
/** @} */

/** @defgroup LL_ATIM_CHANNEL_DIR Channel Direction */
/** @{ */
#define LL_ATIM_CHANNEL_OUTPUT    (0x0UL)
#define LL_ATIM_CHANNEL_INPUT_TIx (0x1UL)
#define LL_ATIM_CHANNEL_INPUT_TIy (0x2UL)
#define LL_ATIM_CHANNEL_INPUT_TRC (0x3UL)
/** @} */

/*==============================================================================
 * Break / Lock (BDTR)
 *============================================================================*/

/** @defgroup LL_ATIM_LOCK_LEVEL Lock Protection Level */
/** @{ */
#define LL_ATIM_LOCK_LEVEL_OFF (0x0UL)
#define LL_ATIM_LOCK_LEVEL_1   (0x1UL)
#define LL_ATIM_LOCK_LEVEL_2   (0x2UL)
#define LL_ATIM_LOCK_LEVEL_3   (0x3UL)
/** @} */

/** @defgroup LL_ATIM_BREAK_POLARITY Break Input Polarity */
/** @{ */
#define LL_ATIM_BREAK_POL_LOW  (0x0UL)
#define LL_ATIM_BREAK_POL_HIGH (0x1UL)
/** @} */

/** @defgroup LL_ATIM_RCR Repetition Counter Mask */
/** @{ */
#define LL_ATIM_RCR_REP_Msk ATIM_RCR_REP_Msk
static inline void ll_atim_enable_uif_remap(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, ATIM_CR1_UIFREMAP);
}

static inline void ll_atim_disable_uif_remap(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, ATIM_CR1_UIFREMAP);
}

static inline void ll_atim_enable_auto_reload_preload(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, ATIM_CR1_ARPE);
}

static inline void ll_atim_disable_auto_reload_preload(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, ATIM_CR1_ARPE);
}

/*==============================================================================
 * Counter Configuration
 *============================================================================*/

static inline void ll_atim_set_counter_mode(ATIM_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->CR1, ATIM_CR1_CMS,
               MAKE_REG_VAL(mode, ATIM_CR1_CMS_Msk, ATIM_CR1_CMS_Pos));
}

static inline void ll_atim_set_direction(ATIM_TypeDef *tim, uint32_t dir)
{
    MODIFY_REG(tim->CR1, ATIM_CR1_DIR,
               MAKE_REG_VAL(dir, ATIM_CR1_DIR_Msk, ATIM_CR1_DIR_Pos));
}

static inline void ll_atim_enable_one_pulse(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, ATIM_CR1_OPM);
}

static inline void ll_atim_disable_one_pulse(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, ATIM_CR1_OPM);
}

static inline void ll_atim_set_update_request_overflow_only(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, ATIM_CR1_URS);
}

static inline void ll_atim_set_update_request_any_event(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, ATIM_CR1_URS);
}

static inline void ll_atim_disable_update_event(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, ATIM_CR1_UDIS);
}

static inline void ll_atim_enable_update_event(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, ATIM_CR1_UDIS);
}

/** @} */

/*==============================================================================
 * Basic Control
 *============================================================================*/

static inline void ll_atim_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, ATIM_CR1_CEN);
}

static inline void ll_atim_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, ATIM_CR1_CEN);
}

static inline uint32_t ll_atim_is_enabled(ATIM_TypeDef *tim)
{
    return READ_BIT(tim->CR1, ATIM_CR1_CEN) ? 1UL : 0UL;
}

/**
 * @brief Set the master mode selection 2 (CR2.MMS2[23:20]).
 * @param[in] tim  ATIM instance pointer.
 * @param[in] mms2 Master mode selection 2 value.
 */
static inline void ll_atim_set_master_mode_selection2(ATIM_TypeDef *tim, uint32_t mms2)
{
    MODIFY_REG(tim->CR2, ATIM_CR2_MMS2,
               MAKE_REG_VAL(mms2, ATIM_CR2_MMS2_Msk, ATIM_CR2_MMS2_Pos));
}

/**
 * @brief Set the output idle state of an OC channel (CR2.OISx, channel 1..6).
 * @param[in] tim        ATIM instance pointer.
 * @param[in] ch         Channel, 1..6.
 * @param[in] idle_high  1 = OCx high on idle, 0 = low.
 */
static inline void ll_atim_set_output_idle_state(ATIM_TypeDef *tim, uint32_t ch,
        uint32_t idle_high)
{
    uint32_t bit;

    switch (ch)
    {
    case 1U:
        bit = ATIM_CR2_OIS1;
        break;
    case 2U:
        bit = ATIM_CR2_OIS2;
        break;
    case 3U:
        bit = ATIM_CR2_OIS3;
        break;
    case 4U:
        bit = ATIM_CR2_OIS4;
        break;
    case 5U:
        bit = ATIM_CR2_OIS5;
        break;
    case 6U:
        bit = ATIM_CR2_OIS6;
        break;
    default:
        return;
    }
    idle_high ? SET_BIT(tim->CR2, bit) : CLEAR_BIT(tim->CR2, bit);
}

/**
 * @brief Set the complementary output idle state (CR2.OISxN, channel 1..3).
 * @note CR2 only provides OIS1N/OIS2N/OIS3N; channel 4 has no complement.
 * @param[in] tim        ATIM instance pointer.
 * @param[in] ch         Channel, 1..3.
 * @param[in] idle_high  1 = OCxN high on idle, 0 = low.
 */
static inline void ll_atim_set_output_idle_state_n(ATIM_TypeDef *tim, uint32_t ch,
        uint32_t idle_high)
{
    uint32_t bit;

    switch (ch)
    {
    case 1U:
        bit = ATIM_CR2_OIS1N;
        break;
    case 2U:
        bit = ATIM_CR2_OIS2N;
        break;
    case 3U:
        bit = ATIM_CR2_OIS3N;
        break;
    default:
        return;
    }
    idle_high ? SET_BIT(tim->CR2, bit) : CLEAR_BIT(tim->CR2, bit);
}

/**
 * @brief Force TI1 to the CH1 input (CR2.TI1S).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_ti1_selection_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CR2, ATIM_CR2_TI1S);
}

/**
 * @brief Select TIMx_CH1 as TI1 (CR2.TI1S = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_ti1_selection_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR2, ATIM_CR2_TI1S);
}

/**
 * @brief Set the master mode selection (CR2.MMS[6:4]).
 * @param[in] tim ATIM instance pointer.
 * @param[in] mms Master mode selection value.
 */
static inline void ll_atim_set_master_mode_selection(ATIM_TypeDef *tim, uint32_t mms)
{
    MODIFY_REG(tim->CR2, ATIM_CR2_MMS,
               MAKE_REG_VAL(mms, ATIM_CR2_MMS_Msk, ATIM_CR2_MMS_Pos));
}

static inline void ll_atim_set_capture_compare_dma_selection(ATIM_TypeDef *tim,
        uint32_t selection)
{
    MODIFY_REG(tim->CR2, ATIM_CR2_CCDS,
               MAKE_REG_VAL(selection, ATIM_CR2_CCDS_Msk, ATIM_CR2_CCDS_Pos));
}

static inline void ll_atim_set_capture_compare_control_update_selection(ATIM_TypeDef *tim,
        uint32_t selection)
{
    MODIFY_REG(tim->CR2, ATIM_CR2_CCUS,
               MAKE_REG_VAL(selection, ATIM_CR2_CCUS_Msk, ATIM_CR2_CCUS_Pos));
}

/**
 * @brief Enable capture/compare preloaded control (CR2.CCPC).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_ccpc_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CR2, ATIM_CR2_CCPC);
}

/**
 * @brief Disable capture/compare preloaded control (CR2.CCPC = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_ccpc_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR2, ATIM_CR2_CCPC);
}

/**
 * @brief Set the slave mode selection (SMCR.SMS[19:16]).
 * @param[in] tim ATIM instance pointer.
 * @param[in] sms Slave mode selection value.
 */
static inline void ll_atim_set_slave_mode(ATIM_TypeDef *tim, uint32_t sms)
{
    MODIFY_REG(tim->SMCR, ATIM_SMCR_SMS,
               MAKE_REG_VAL(sms, ATIM_SMCR_SMS_Msk, ATIM_SMCR_SMS_Pos));
}

/**
 * @brief Set the external trigger polarity (SMCR.ETP).
 * @param[in] tim ATIM instance pointer.
 * @param[in] etp 1 = active low/inverted, 0 = non-inverted.
 */
static inline void ll_atim_set_external_trigger_polarity(ATIM_TypeDef *tim, uint32_t etp)
{
    MODIFY_REG(tim->SMCR, ATIM_SMCR_ETP,
               MAKE_REG_VAL(etp, ATIM_SMCR_ETP_Msk, ATIM_SMCR_ETP_Pos));
}

/**
 * @brief Enable the external clock mode (SMCR.ECE).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_external_clock_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->SMCR, ATIM_SMCR_ECE);
}

/**
 * @brief Disable the external clock mode (SMCR.ECE = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_external_clock_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->SMCR, ATIM_SMCR_ECE);
}

/**
 * @brief Set the external trigger prescaler (SMCR.ETPS[13:12]).
 * @param[in] tim  ATIM instance pointer.
 * @param[in] etps External trigger prescaler value.
 */
static inline void ll_atim_set_external_trigger_prescaler(ATIM_TypeDef *tim, uint32_t etps)
{
    MODIFY_REG(tim->SMCR, ATIM_SMCR_ETPS,
               MAKE_REG_VAL(etps, ATIM_SMCR_ETPS_Msk, ATIM_SMCR_ETPS_Pos));
}

/**
 * @brief Set the external trigger filter (SMCR.ETF[11:8]).
 * @param[in] tim ATIM instance pointer.
 * @param[in] etf External trigger filter value.
 */
static inline void ll_atim_set_external_trigger_filter(ATIM_TypeDef *tim, uint32_t etf)
{
    MODIFY_REG(tim->SMCR, ATIM_SMCR_ETF,
               MAKE_REG_VAL(etf, ATIM_SMCR_ETF_Msk, ATIM_SMCR_ETF_Pos));
}

/**
 * @brief Enable master/slave mode (SMCR.MSM).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_master_slave_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->SMCR, ATIM_SMCR_MSM);
}

/**
 * @brief Disable master/slave mode (SMCR.MSM = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_master_slave_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->SMCR, ATIM_SMCR_MSM);
}

/**
 * @brief Set the trigger selection (SMCR.TS[6:4]).
 * @param[in] tim ATIM instance pointer.
 * @param[in] ts  Trigger selection value.
 */
static inline void ll_atim_set_trigger_selection(ATIM_TypeDef *tim, uint32_t ts)
{
    MODIFY_REG(tim->SMCR, ATIM_SMCR_TS,
               MAKE_REG_VAL(ts, ATIM_SMCR_TS_Msk, ATIM_SMCR_TS_Pos));
}

static inline void ll_atim_enable_channel_interrupt(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        SET_BIT(tim->DIER, ATIM_DIER_CC1IE);
        break;
    case 2U:
        SET_BIT(tim->DIER, ATIM_DIER_CC2IE);
        break;
    case 3U:
        SET_BIT(tim->DIER, ATIM_DIER_CC3IE);
        break;
    case 4U:
        SET_BIT(tim->DIER, ATIM_DIER_CC4IE);
        break;
    case 5U:
        SET_BIT(tim->DIER, ATIM_DIER_CC5IE);
        break;
    case 6U:
        SET_BIT(tim->DIER, ATIM_DIER_CC6IE);
        break;
    default:
        break;
    }
}

static inline void ll_atim_disable_channel_interrupt(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC1IE);
        break;
    case 2U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC2IE);
        break;
    case 3U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC3IE);
        break;
    case 4U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC4IE);
        break;
    case 5U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC5IE);
        break;
    case 6U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC6IE);
        break;
    default:
        break;
    }
}

static inline void ll_atim_enable_trigger_dma(ATIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, ATIM_DIER_TDE);
}

static inline void ll_atim_disable_trigger_dma(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, ATIM_DIER_TDE);
}

static inline void ll_atim_enable_commutation_dma(ATIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, ATIM_DIER_COMDE);
}

static inline void ll_atim_disable_commutation_dma(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, ATIM_DIER_COMDE);
}

static inline void ll_atim_enable_channel_dma(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        SET_BIT(tim->DIER, ATIM_DIER_CC1DE);
        break;
    case 2U:
        SET_BIT(tim->DIER, ATIM_DIER_CC2DE);
        break;
    case 3U:
        SET_BIT(tim->DIER, ATIM_DIER_CC3DE);
        break;
    case 4U:
        SET_BIT(tim->DIER, ATIM_DIER_CC4DE);
        break;
    default:
        break;
    }
}

static inline void ll_atim_disable_channel_dma(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC1DE);
        break;
    case 2U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC2DE);
        break;
    case 3U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC3DE);
        break;
    case 4U:
        CLEAR_BIT(tim->DIER, ATIM_DIER_CC4DE);
        break;
    default:
        break;
    }
}

static inline void ll_atim_enable_update_dma(ATIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, ATIM_DIER_UDE);
}

static inline void ll_atim_disable_update_dma(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, ATIM_DIER_UDE);
}

static inline void ll_atim_enable_break_interrupt(ATIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, ATIM_DIER_BIE);
}

static inline void ll_atim_disable_break_interrupt(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, ATIM_DIER_BIE);
}

static inline void ll_atim_enable_trigger_interrupt(ATIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, ATIM_DIER_TIE);
}

static inline void ll_atim_disable_trigger_interrupt(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, ATIM_DIER_TIE);
}

static inline void ll_atim_enable_commutation_interrupt(ATIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, ATIM_DIER_COMIE);
}

static inline void ll_atim_disable_commutation_interrupt(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, ATIM_DIER_COMIE);
}

/*==============================================================================
 * Interrupt
 *============================================================================*/

static inline void ll_atim_enable_update_interrupt(ATIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, ATIM_DIER_UIE);
}

static inline void ll_atim_disable_update_interrupt(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, ATIM_DIER_UIE);
}

static inline uint32_t ll_atim_get_channel_flag(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        return READ_BIT(tim->SR, ATIM_SR_CC1IF) ? 1UL : 0UL;
    case 2U:
        return READ_BIT(tim->SR, ATIM_SR_CC2IF) ? 1UL : 0UL;
    case 3U:
        return READ_BIT(tim->SR, ATIM_SR_CC3IF) ? 1UL : 0UL;
    case 4U:
        return READ_BIT(tim->SR, ATIM_SR_CC4IF) ? 1UL : 0UL;
    case 5U:
        return READ_BIT(tim->SR, ATIM_SR_CC5IF) ? 1UL : 0UL;
    case 6U:
        return READ_BIT(tim->SR, ATIM_SR_CC6IF) ? 1UL : 0UL;
    default:
        return 0UL;
    }
}

static inline void ll_atim_clear_channel_flag(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC1IF);
        break;
    case 2U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC2IF);
        break;
    case 3U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC3IF);
        break;
    case 4U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC4IF);
        break;
    case 5U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC5IF);
        break;
    case 6U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC6IF);
        break;
    default:
        break;
    }
}

static inline uint32_t ll_atim_get_system_break_flag(ATIM_TypeDef *tim)
{
    return READ_BIT(tim->SR, ATIM_SR_SBIF) ? 1UL : 0UL;
}

static inline void ll_atim_clear_system_break_flag(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->SR, ~ATIM_SR_SBIF);
}

static inline uint32_t ll_atim_get_overcapture_flag(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        return READ_BIT(tim->SR, ATIM_SR_CC1OF) ? 1UL : 0UL;
    case 2U:
        return READ_BIT(tim->SR, ATIM_SR_CC2OF) ? 1UL : 0UL;
    case 3U:
        return READ_BIT(tim->SR, ATIM_SR_CC3OF) ? 1UL : 0UL;
    case 4U:
        return READ_BIT(tim->SR, ATIM_SR_CC4OF) ? 1UL : 0UL;
    default:
        return 0UL;
    }
}

static inline void ll_atim_clear_overcapture_flag(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC1OF);
        break;
    case 2U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC2OF);
        break;
    case 3U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC3OF);
        break;
    case 4U:
        WRITE_REG(tim->SR, ~ATIM_SR_CC4OF);
        break;
    default:
        break;
    }
}

static inline uint32_t ll_atim_get_break2_flag(ATIM_TypeDef *tim)
{
    return READ_BIT(tim->SR, ATIM_SR_B2IF) ? 1UL : 0UL;
}

static inline void ll_atim_clear_break2_flag(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->SR, ~ATIM_SR_B2IF);
}

static inline uint32_t ll_atim_get_break_flag(ATIM_TypeDef *tim)
{
    return READ_BIT(tim->SR, ATIM_SR_BIF) ? 1UL : 0UL;
}

static inline void ll_atim_clear_break_flag(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->SR, ~ATIM_SR_BIF);
}

static inline uint32_t ll_atim_get_trigger_flag(ATIM_TypeDef *tim)
{
    return READ_BIT(tim->SR, ATIM_SR_TIF) ? 1UL : 0UL;
}

static inline void ll_atim_clear_trigger_flag(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->SR, ~ATIM_SR_TIF);
}

static inline uint32_t ll_atim_get_commutation_flag(ATIM_TypeDef *tim)
{
    return READ_BIT(tim->SR, ATIM_SR_COMIF) ? 1UL : 0UL;
}

static inline void ll_atim_clear_commutation_flag(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->SR, ~ATIM_SR_COMIF);
}

static inline uint32_t ll_atim_get_update_flag(ATIM_TypeDef *tim)
{
    return READ_BIT(tim->SR, ATIM_SR_UIF) ? 1UL : 0UL;
}

static inline void ll_atim_clear_update_flag(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->SR, ~ATIM_SR_UIF);
}

static inline void ll_atim_generate_break2_event(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->EGR, ATIM_EGR_B2G);
}

static inline void ll_atim_generate_break_event(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->EGR, ATIM_EGR_BG);
}

static inline void ll_atim_generate_trigger_event(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->EGR, ATIM_EGR_TG);
}

static inline void ll_atim_generate_commutation_event(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->EGR, ATIM_EGR_COMG);
}

static inline void ll_atim_generate_channel_event(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1U:
        WRITE_REG(tim->EGR, ATIM_EGR_CC1G);
        break;
    case 2U:
        WRITE_REG(tim->EGR, ATIM_EGR_CC2G);
        break;
    case 3U:
        WRITE_REG(tim->EGR, ATIM_EGR_CC3G);
        break;
    case 4U:
        WRITE_REG(tim->EGR, ATIM_EGR_CC4G);
        break;
    default:
        break;
    }
}

static inline void ll_atim_generate_update(ATIM_TypeDef *tim)
{
    WRITE_REG(tim->EGR, ATIM_EGR_UG);
}

/**
 * @brief Set channel output compare mode (CCMRx.OCxM).
 * @param[in] tim  ATIM instance pointer.
 * @param[in] ch   Channel number in the range 1 to 4.
 * @param[in] mode Output compare mode, see @ref LL_ATIM_OC_MODE_*.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_atim_set_channel_oc_mode(ATIM_TypeDef *tim, uint32_t ch, uint32_t mode)
{
    switch (ch)
    {
    case 1:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_OC1M,
                   MAKE_REG_VAL(mode, ATIM_CCMR1_OC1M_Msk, ATIM_CCMR1_OC1M_Pos));
        break;
    case 2:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_OC2M,
                   MAKE_REG_VAL(mode, ATIM_CCMR1_OC2M_Msk, ATIM_CCMR1_OC2M_Pos));
        break;
    case 3:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_OC3M,
                   MAKE_REG_VAL(mode, ATIM_CCMR2_OC3M_Msk, ATIM_CCMR2_OC3M_Pos));
        break;
    case 4:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_OC4M,
                   MAKE_REG_VAL(mode, ATIM_CCMR2_OC4M_Msk, ATIM_CCMR2_OC4M_Pos));
        break;
    default:
        break;
    }
}

/**
 * @brief Enable or disable channel output compare preload (CCMRx.OCxPE).
 * @param[in] tim    ATIM instance pointer.
 * @param[in] ch     Channel number in the range 1 to 4.
 * @param[in] enable 1 to enable preload, 0 to disable.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_atim_set_channel_preload(ATIM_TypeDef *tim, uint32_t ch, uint32_t enable)
{
    switch (ch)
    {
    case 1:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_OC1PE, enable ? ATIM_CCMR1_OC1PE : 0UL);
        break;
    case 2:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_OC2PE, enable ? ATIM_CCMR1_OC2PE : 0UL);
        break;
    case 3:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_OC3PE, enable ? ATIM_CCMR2_OC3PE : 0UL);
        break;
    case 4:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_OC4PE, enable ? ATIM_CCMR2_OC4PE : 0UL);
        break;
    default:
        break;
    }
}

static inline void ll_atim_set_channel_clear(ATIM_TypeDef *tim, uint32_t ch, uint32_t enable)
{
    switch (ch)
    {
    case 1U:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_OC1CE, enable ? ATIM_CCMR1_OC1CE : 0UL);
        break;
    case 2U:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_OC2CE, enable ? ATIM_CCMR1_OC2CE : 0UL);
        break;
    case 3U:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_OC3CE, enable ? ATIM_CCMR2_OC3CE : 0UL);
        break;
    case 4U:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_OC4CE, enable ? ATIM_CCMR2_OC4CE : 0UL);
        break;
    default:
        break;
    }
}

static inline void ll_atim_set_input_capture_config(ATIM_TypeDef *tim, uint32_t ch,
        uint32_t prescaler, uint32_t filter)
{
    switch (ch)
    {
    case 1U:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_IC1PSC | ATIM_CCMR1_IC1F,
                   MAKE_REG_VAL(prescaler, ATIM_CCMR1_IC1PSC_Msk,
                                ATIM_CCMR1_IC1PSC_Pos) |
                   MAKE_REG_VAL(filter, ATIM_CCMR1_IC1F_Msk, ATIM_CCMR1_IC1F_Pos));
        break;
    case 2U:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_IC2PSC | ATIM_CCMR1_IC2F,
                   MAKE_REG_VAL(prescaler, ATIM_CCMR1_IC2PSC_Msk,
                                ATIM_CCMR1_IC2PSC_Pos) |
                   MAKE_REG_VAL(filter, ATIM_CCMR1_IC2F_Msk, ATIM_CCMR1_IC2F_Pos));
        break;
    case 3U:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_IC3PSC | ATIM_CCMR2_IC3F,
                   MAKE_REG_VAL(prescaler, ATIM_CCMR2_IC3PSC_Msk,
                                ATIM_CCMR2_IC3PSC_Pos) |
                   MAKE_REG_VAL(filter, ATIM_CCMR2_IC3F_Msk, ATIM_CCMR2_IC3F_Pos));
        break;
    case 4U:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_IC4PSC | ATIM_CCMR2_IC4F,
                   MAKE_REG_VAL(prescaler, ATIM_CCMR2_IC4PSC_Msk,
                                ATIM_CCMR2_IC4PSC_Pos) |
                   MAKE_REG_VAL(filter, ATIM_CCMR2_IC4F_Msk, ATIM_CCMR2_IC4F_Pos));
        break;
    default:
        break;
    }
}

static inline void ll_atim_set_channel_direction(ATIM_TypeDef *tim, uint32_t ch, uint32_t dir)
{
    switch (ch)
    {
    case 1U:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_CC1S,
                   MAKE_REG_VAL(dir, ATIM_CCMR1_CC1S_Msk, ATIM_CCMR1_CC1S_Pos));
        break;
    case 2U:
        MODIFY_REG(tim->CCMR1, ATIM_CCMR1_CC2S,
                   MAKE_REG_VAL(dir, ATIM_CCMR1_CC2S_Msk, ATIM_CCMR1_CC2S_Pos));
        break;
    case 3U:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_CC3S,
                   MAKE_REG_VAL(dir, ATIM_CCMR2_CC3S_Msk, ATIM_CCMR2_CC3S_Pos));
        break;
    case 4U:
        MODIFY_REG(tim->CCMR2, ATIM_CCMR2_CC4S,
                   MAKE_REG_VAL(dir, ATIM_CCMR2_CC4S_Msk, ATIM_CCMR2_CC4S_Pos));
        break;
    default:
        break;
    }
}

/**
 * @brief Set channel output polarity (CCER.CCxP)
 * @param[in] tim ATIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 6.
 * @param[in] pol 0 for active high, 1 for active low.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_atim_set_channel_polarity(ATIM_TypeDef *tim, uint32_t ch, uint32_t pol)
{
    switch (ch)
    {
    case 1:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC1P, pol ? ATIM_CCER_CC1P : 0UL);
        break;
    case 2:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC2P, pol ? ATIM_CCER_CC2P : 0UL);
        break;
    case 3:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC3P, pol ? ATIM_CCER_CC3P : 0UL);
        break;
    case 4:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC4P, pol ? ATIM_CCER_CC4P : 0UL);
        break;
    case 5:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC5P, pol ? ATIM_CCER_CC5P : 0UL);
        break;
    case 6:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC6P, pol ? ATIM_CCER_CC6P : 0UL);
        break;
    default:
        break;
    }
}

/**
 * @brief Enable channel output (CCER.CCxE)
 * @param[in] tim ATIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 6.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_atim_enable_channel(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        SET_BIT(tim->CCER, ATIM_CCER_CC1E);
        break;
    case 2:
        SET_BIT(tim->CCER, ATIM_CCER_CC2E);
        break;
    case 3:
        SET_BIT(tim->CCER, ATIM_CCER_CC3E);
        break;
    case 4:
        SET_BIT(tim->CCER, ATIM_CCER_CC4E);
        break;
    case 5:
        SET_BIT(tim->CCER, ATIM_CCER_CC5E);
        break;
    case 6:
        SET_BIT(tim->CCER, ATIM_CCER_CC6E);
        break;
    default:
        break;
    }
}

/**
 * @brief Disable channel output (CCER.CCxE).
 * @param[in] tim ATIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 6.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_atim_disable_channel(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC1E);
        break;
    case 2:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC2E);
        break;
    case 3:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC3E);
        break;
    case 4:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC4E);
        break;
    case 5:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC5E);
        break;
    case 6:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC6E);
        break;
    default:
        break;
    }
}

static inline void ll_atim_set_complementary_polarity(ATIM_TypeDef *tim, uint32_t ch,
        uint32_t pol)
{
    switch (ch)
    {
    case 1U:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC1NP, pol ? ATIM_CCER_CC1NP : 0UL);
        break;
    case 2U:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC2NP, pol ? ATIM_CCER_CC2NP : 0UL);
        break;
    case 3U:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC3NP, pol ? ATIM_CCER_CC3NP : 0UL);
        break;
    case 4U:
        MODIFY_REG(tim->CCER, ATIM_CCER_CC4NP, pol ? ATIM_CCER_CC4NP : 0UL);
        break;
    default:
        break;
    }
}

/**
 * @brief Enable complementary output (CCER.CCxNE)
 * @param[in] tim ATIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 3.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_atim_enable_complementary(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        SET_BIT(tim->CCER, ATIM_CCER_CC1NE);
        break;
    case 2:
        SET_BIT(tim->CCER, ATIM_CCER_CC2NE);
        break;
    case 3:
        SET_BIT(tim->CCER, ATIM_CCER_CC3NE);
        break;
    default:
        break;
    }
}

/**
 * @brief Disable complementary output (CCER.CCxNE).
 * @param[in] tim ATIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 3.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_atim_disable_complementary(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC1NE);
        break;
    case 2:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC2NE);
        break;
    case 3:
        CLEAR_BIT(tim->CCER, ATIM_CCER_CC3NE);
        break;
    default:
        break;
    }
}

/*==============================================================================
 * Counter Value
 *============================================================================*/

static inline void ll_atim_set_counter(ATIM_TypeDef *tim, uint32_t cnt)
{
    WRITE_REG(tim->CNT, cnt);
}

static inline uint32_t ll_atim_get_counter(ATIM_TypeDef *tim)
{
    return READ_REG(tim->CNT);
}

/*==============================================================================
 * Prescaler, Auto-Reload, Repetition
 *============================================================================*/

static inline void ll_atim_set_prescaler(ATIM_TypeDef *tim, uint16_t psc)
{
    WRITE_REG(tim->PSC, psc);
}

static inline uint16_t ll_atim_get_prescaler(ATIM_TypeDef *tim)
{
    return (uint16_t)READ_REG(tim->PSC);
}

static inline void ll_atim_set_auto_reload(ATIM_TypeDef *tim, uint32_t arr)
{
    WRITE_REG(tim->ARR, arr);
}

static inline uint32_t ll_atim_get_auto_reload(ATIM_TypeDef *tim)
{
    return READ_REG(tim->ARR);
}

static inline void ll_atim_set_repetition(ATIM_TypeDef *tim, uint16_t rep)
{
    MODIFY_REG(tim->RCR, LL_ATIM_RCR_REP_Msk,
               MAKE_REG_VAL(rep, LL_ATIM_RCR_REP_Msk, ATIM_RCR_REP_Pos));
}

/*==============================================================================
 * Capture/Compare Channels
 *============================================================================*/

/**
 * @brief Set compare/capture value for a channel.
 * @param[in] tim ATIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 6.
 * @param[in] val Compare/capture value.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_atim_set_compare(ATIM_TypeDef *tim, uint32_t ch, uint32_t val)
{
    switch (ch)
    {
    case 1:
        WRITE_REG(tim->CCR1, val);
        break;
    case 2:
        WRITE_REG(tim->CCR2, val);
        break;
    case 3:
        WRITE_REG(tim->CCR3, val);
        break;
    case 4:
        WRITE_REG(tim->CCR4, val);
        break;
    case 5:
        WRITE_REG(tim->CCR5, val);
        break;
    case 6:
        WRITE_REG(tim->CCR6, val);
        break;
    default:
        break;
    }
}

/**
 * @brief Read capture value for a channel.
 * @param[in] tim ATIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @return Capture value, or 0 for an invalid channel number.
 */
static inline uint32_t ll_atim_get_capture(ATIM_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        return READ_REG(tim->CCR1);
    case 2:
        return READ_REG(tim->CCR2);
    case 3:
        return READ_REG(tim->CCR3);
    case 4:
        return READ_REG(tim->CCR4);
    default:
        return 0U;
    }
}

static inline void ll_atim_set_off_state_run(ATIM_TypeDef *tim, uint32_t enable)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_OSSR, enable ? ATIM_BDTR_OSSR : 0UL);
}

static inline void ll_atim_set_off_state_idle(ATIM_TypeDef *tim, uint32_t enable)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_OSSI, enable ? ATIM_BDTR_OSSI : 0UL);
}

static inline void ll_atim_set_break2_bidirectional(ATIM_TypeDef *tim, uint32_t enable)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_BK2BID, enable ? ATIM_BDTR_BK2BID : 0UL);
}

static inline void ll_atim_set_break_bidirectional(ATIM_TypeDef *tim, uint32_t enable)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_BKBID, enable ? ATIM_BDTR_BKBID : 0UL);
}

static inline void ll_atim_set_break2_disarm(ATIM_TypeDef *tim, uint32_t disarm)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_BK2DSRM, disarm ? ATIM_BDTR_BK2DSRM : 0UL);
}

static inline void ll_atim_set_break_disarm(ATIM_TypeDef *tim, uint32_t disarm)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_BKDSRM, disarm ? ATIM_BDTR_BKDSRM : 0UL);
}

static inline void ll_atim_set_break2_polarity(ATIM_TypeDef *tim, uint32_t pol)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_BK2P, pol ? ATIM_BDTR_BK2P : 0UL);
}

static inline void ll_atim_enable_break2(ATIM_TypeDef *tim)
{
    SET_BIT(tim->BDTR, ATIM_BDTR_BK2E);
}

static inline void ll_atim_disable_break2(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->BDTR, ATIM_BDTR_BK2E);
}

static inline void ll_atim_set_break2_filter(ATIM_TypeDef *tim, uint32_t filter)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_BK2F,
               MAKE_REG_VAL(filter, ATIM_BDTR_BK2F_Msk, ATIM_BDTR_BK2F_Pos));
}

static inline void ll_atim_set_break_filter(ATIM_TypeDef *tim, uint32_t filter)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_BKF,
               MAKE_REG_VAL(filter, ATIM_BDTR_BKF_Msk, ATIM_BDTR_BKF_Pos));
}

static inline void ll_atim_enable_main_output(ATIM_TypeDef *tim)
{
    SET_BIT(tim->BDTR, ATIM_BDTR_MOE);
}

static inline void ll_atim_disable_main_output(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->BDTR, ATIM_BDTR_MOE);
}

static inline void ll_atim_enable_automatic_output(ATIM_TypeDef *tim)
{
    SET_BIT(tim->BDTR, ATIM_BDTR_AOE);
}

static inline void ll_atim_disable_automatic_output(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->BDTR, ATIM_BDTR_AOE);
}

static inline void ll_atim_set_break_polarity(ATIM_TypeDef *tim, uint32_t pol)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_BKP, pol ? ATIM_BDTR_BKP : 0UL);
}

/*==============================================================================
 * Break Control (BDTR)
 *============================================================================*/

static inline void ll_atim_enable_break(ATIM_TypeDef *tim)
{
    SET_BIT(tim->BDTR, ATIM_BDTR_BKE);
}

static inline void ll_atim_disable_break(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->BDTR, ATIM_BDTR_BKE);
}

static inline void ll_atim_enable_dead_time_prescaler(ATIM_TypeDef *tim)
{
    SET_BIT(tim->BDTR, ATIM_BDTR_DTPSC);
}

static inline void ll_atim_disable_dead_time_prescaler(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->BDTR, ATIM_BDTR_DTPSC);
}

/*==============================================================================
 * Dead-Time (BDTR)
 *============================================================================*/

/**
 * @brief Set the raw dead-time generator value.
 * @param[in] tim ATIM instance pointer.
 * @param[in] dtg Dead-time generator value in the range 0 to 1023.
 * @note A value of zero disables dead-time. For non-zero values, dead-time is
 *       (dtg + 1) timer clock cycles, or 16 times that value when DTPSC is set.
 */
static inline void ll_atim_set_dead_time(ATIM_TypeDef *tim, uint16_t dtg)
{
    MODIFY_REG(tim->BDTR, ATIM_BDTR_DTG,
               MAKE_REG_VAL(dtg, ATIM_BDTR_DTG_Msk, ATIM_BDTR_DTG_Pos));
}

/**
 * @brief Set the OC6 output compare mode (CCMR3.OC6M[31:28]).
 * @param[in] tim  ATIM instance pointer.
 * @param[in] mode Output compare mode (use LL_ATIM_OC_MODE_*).
 */
static inline void ll_atim_oc6_set_mode(ATIM_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->CCMR3, ATIM_CCMR3_OC6M,
               MAKE_REG_VAL(mode, ATIM_CCMR3_OC6M_Msk, ATIM_CCMR3_OC6M_Pos));
}

/**
 * @brief Enable OC6 preload (CCMR3.OC6PE).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_oc6_preload_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CCMR3, ATIM_CCMR3_OC6PE);
}

/**
 * @brief Disable OC6 preload (CCMR3.OC6PE = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_oc6_preload_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CCMR3, ATIM_CCMR3_OC6PE);
}

/**
 * @brief Enable OC6 clear (CCMR3.OC6CE).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_oc6_clear_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CCMR3, ATIM_CCMR3_OC6CE);
}

/**
 * @brief Disable OC6 clear (CCMR3.OC6CE = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_oc6_clear_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CCMR3, ATIM_CCMR3_OC6CE);
}

/**
 * @brief Set the OC5 output compare mode (CCMR3.OC5M[23:20]).
 * @param[in] tim  ATIM instance pointer.
 * @param[in] mode Output compare mode (use LL_ATIM_OC_MODE_*).
 */
static inline void ll_atim_oc5_set_mode(ATIM_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->CCMR3, ATIM_CCMR3_OC5M,
               MAKE_REG_VAL(mode, ATIM_CCMR3_OC5M_Msk, ATIM_CCMR3_OC5M_Pos));
}

/**
 * @brief Enable OC5 preload (CCMR3.OC5PE).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_oc5_preload_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CCMR3, ATIM_CCMR3_OC5PE);
}

/**
 * @brief Disable OC5 preload (CCMR3.OC5PE = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_oc5_preload_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CCMR3, ATIM_CCMR3_OC5PE);
}

/**
 * @brief Enable OC5 clear (CCMR3.OC5CE).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_oc5_clear_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->CCMR3, ATIM_CCMR3_OC5CE);
}

/**
 * @brief Disable OC5 clear (CCMR3.OC5CE = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_oc5_clear_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CCMR3, ATIM_CCMR3_OC5CE);
}

/**
 * @brief Set the group channel 5 selection (CCMR3.GC5C1..C3).
 * @param[in] tim ATIM instance pointer.
 * @param[in] grp Group selection value (3 bits: GC5C1/GC5C2/GC5C3).
 */
static inline void ll_atim_set_group_ch5(ATIM_TypeDef *tim, uint32_t grp)
{
    MODIFY_REG(tim->CCMR3, ATIM_CCMR3_GC5C1 | ATIM_CCMR3_GC5C2 | ATIM_CCMR3_GC5C3,
               MAKE_REG_VAL(grp, ATIM_CCMR3_GC5C1 | ATIM_CCMR3_GC5C2 | ATIM_CCMR3_GC5C3,
                            ATIM_CCMR3_GC5C1_Pos));
}

/**
 * @brief Set the ATIM lock protection level.
 * @param[in] tim ATIM instance pointer.
 * @param[in] level Lock level, one of @ref LL_ATIM_LOCK_LEVEL.
 * @note A non-zero lock level can only be written once after reset and cannot
 *       be cleared or reduced.
 */
static inline void ll_atim_set_lock_level(ATIM_TypeDef *tim, uint32_t level)
{
    MODIFY_REG(tim->AF1, ATIM_AF1_LOCK,
               MAKE_REG_VAL(level, ATIM_AF1_LOCK_Msk, ATIM_AF1_LOCK_Pos));
}

static inline void ll_atim_set_external_trigger_source(ATIM_TypeDef *tim, uint32_t source)
{
    MODIFY_REG(tim->AF1, ATIM_AF1_ETRSEL,
               MAKE_REG_VAL(source, ATIM_AF1_ETRSEL_Msk, ATIM_AF1_ETRSEL_Pos));
}

static inline void ll_atim_brk_cmp2_polarity_set(ATIM_TypeDef *tim, uint32_t pol)
{
    MODIFY_REG(tim->AF1, ATIM_AF1_BKCMP2P,
               MAKE_REG_VAL(pol, ATIM_AF1_BKCMP2P_Msk, ATIM_AF1_BKCMP2P_Pos));
}

static inline void ll_atim_brk_cmp1_polarity_set(ATIM_TypeDef *tim, uint32_t pol)
{
    MODIFY_REG(tim->AF1, ATIM_AF1_BKCMP1P,
               MAKE_REG_VAL(pol, ATIM_AF1_BKCMP1P_Msk, ATIM_AF1_BKCMP1P_Pos));
}

static inline void ll_atim_brk_input_polarity_set(ATIM_TypeDef *tim, uint32_t pol)
{
    MODIFY_REG(tim->AF1, ATIM_AF1_BKINP,
               MAKE_REG_VAL(pol, ATIM_AF1_BKINP_Msk, ATIM_AF1_BKINP_Pos));
}

static inline void ll_atim_brk_cmp2_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->AF1, ATIM_AF1_BKCMP2E);
}

static inline void ll_atim_brk_cmp2_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->AF1, ATIM_AF1_BKCMP2E);
}

static inline void ll_atim_brk_cmp1_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->AF1, ATIM_AF1_BKCMP1E);
}

static inline void ll_atim_brk_cmp1_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->AF1, ATIM_AF1_BKCMP1E);
}

static inline void ll_atim_brk_input_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->AF1, ATIM_AF1_BKINE);
}

static inline void ll_atim_brk_input_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->AF1, ATIM_AF1_BKINE);
}

/**
 * @brief Set the BRK2 COMP2 input polarity (AF2.BK2CMP2P).
 * @param[in] tim ATIM instance pointer.
 * @param[in] pol 1 = active high, 0 = active low.
 */
static inline void ll_atim_brk2_cmp2_polarity_set(ATIM_TypeDef *tim, uint32_t pol)
{
    MODIFY_REG(tim->AF2, ATIM_AF2_BK2CMP2P,
               MAKE_REG_VAL(pol, ATIM_AF2_BK2CMP2P_Msk, ATIM_AF2_BK2CMP2P_Pos));
}

/**
 * @brief Set the BRK2 COMP1 input polarity (AF2.BK2CMP1P).
 * @param[in] tim ATIM instance pointer.
 * @param[in] pol 1 = active high, 0 = active low.
 */
static inline void ll_atim_brk2_cmp1_polarity_set(ATIM_TypeDef *tim, uint32_t pol)
{
    MODIFY_REG(tim->AF2, ATIM_AF2_BK2CMP1P,
               MAKE_REG_VAL(pol, ATIM_AF2_BK2CMP1P_Msk, ATIM_AF2_BK2CMP1P_Pos));
}

/**
 * @brief Set the BRK2 BKIN2 input polarity (AF2.BK2INP).
 * @param[in] tim ATIM instance pointer.
 * @param[in] pol 1 = active high, 0 = active low.
 */
static inline void ll_atim_brk2_input_polarity_set(ATIM_TypeDef *tim, uint32_t pol)
{
    MODIFY_REG(tim->AF2, ATIM_AF2_BK2INP,
               MAKE_REG_VAL(pol, ATIM_AF2_BK2INP_Msk, ATIM_AF2_BK2INP_Pos));
}

/**
 * @brief Enable the BRK2 COMP2 input (AF2.BK2CMP2E).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_brk2_cmp2_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->AF2, ATIM_AF2_BK2CMP2E);
}

/**
 * @brief Disable the BRK2 COMP2 input (AF2.BK2CMP2E = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_brk2_cmp2_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->AF2, ATIM_AF2_BK2CMP2E);
}

/**
 * @brief Enable the BRK2 COMP1 input (AF2.BK2CMP1E).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_brk2_cmp1_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->AF2, ATIM_AF2_BK2CMP1E);
}

/**
 * @brief Disable the BRK2 COMP1 input (AF2.BK2CMP1E = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_brk2_cmp1_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->AF2, ATIM_AF2_BK2CMP1E);
}

/**
 * @brief Enable the BRK2 BKIN input (AF2.BK2INE).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_brk2_input_enable(ATIM_TypeDef *tim)
{
    SET_BIT(tim->AF2, ATIM_AF2_BK2INE);
}

/**
 * @brief Disable the BRK2 BKIN input (AF2.BK2INE = 0).
 * @param[in] tim ATIM instance pointer.
 */
static inline void ll_atim_brk2_input_disable(ATIM_TypeDef *tim)
{
    CLEAR_BIT(tim->AF2, ATIM_AF2_BK2INE);
}

static inline void ll_atim_set_debug_selection(ATIM_TypeDef *tim, uint32_t selection)
{
    MODIFY_REG(tim->DBGSEL, ATIM_DBGSEL_DBGSEL,
               MAKE_REG_VAL(selection, ATIM_DBGSEL_DBGSEL_Msk, ATIM_DBGSEL_DBGSEL_Pos));
}

/*==============================================================================
 * Frequency Calculation
 *============================================================================*/

static inline uint32_t ll_atim_calc_update_freq(uint32_t clk_freq, uint16_t psc, uint32_t arr)
{
    return (uint32_t)(clk_freq / (((uint64_t)psc + 1U) * ((uint64_t)arr + 1U)));
}

/**
 * @brief Calculate the compare value for a PWM duty cycle.
 * @param[in] arr Auto-reload value.
 * @param[in] permille Duty cycle in the range 0 to 1000.
 * @return Compare value corresponding to @p permille.
 */
static inline uint32_t ll_atim_calc_pwm_duty(uint32_t arr, uint32_t permille)
{
    return (uint32_t)((((uint64_t)arr + 1U) * permille) / 1000U);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_ATIM_H */
