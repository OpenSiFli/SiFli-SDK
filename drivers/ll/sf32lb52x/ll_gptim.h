/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_GPTIM_H
#define __LL_GPTIM_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_gptim.h
 * @brief Header-only low-level General-Purpose Timer (GPTIM) APIs for SF32LB52x.
 */

/*==============================================================================
 * Counter Mode (CR1.CMS)
 *============================================================================*/

/** @defgroup LL_GPTIM_COUNTER_MODE Counter Alignment Mode */
/** @{ */
#define LL_GPTIM_COUNTER_MODE_EDGE    (0x0UL) /**< Edge-aligned */
#define LL_GPTIM_COUNTER_MODE_CENTER1 (0x1UL) /**< Center-aligned mode 1 */
#define LL_GPTIM_COUNTER_MODE_CENTER2 (0x2UL) /**< Center-aligned mode 2 */
#define LL_GPTIM_COUNTER_MODE_CENTER3 (0x3UL) /**< Center-aligned mode 3 */
/** @} */

/** @defgroup LL_GPTIM_COUNTER_DIR Counter Direction (CR1.DIR) */
/** @{ */
#define LL_GPTIM_COUNTER_DIR_UP   (0x0UL)
#define LL_GPTIM_COUNTER_DIR_DOWN (0x1UL)
/** @} */

/*==============================================================================
 * Slave Mode (SMCR.SMS)
 *============================================================================*/

/** @defgroup LL_GPTIM_SLAVE_MODE Slave Mode Selection */
/** @{ */
#define LL_GPTIM_SLAVE_MODE_DISABLED (0x0UL)
#define LL_GPTIM_SLAVE_MODE_ENCODER1 (0x1UL)
#define LL_GPTIM_SLAVE_MODE_ENCODER2 (0x2UL)
#define LL_GPTIM_SLAVE_MODE_ENCODER3 (0x3UL)
#define LL_GPTIM_SLAVE_MODE_RESET    (0x4UL)
#define LL_GPTIM_SLAVE_MODE_GATED    (0x5UL)
#define LL_GPTIM_SLAVE_MODE_TRIGGER  (0x6UL)
#define LL_GPTIM_SLAVE_MODE_EXT_CLK1 (0x7UL)
#define LL_GPTIM_SLAVE_MODE_COMBINED (0x8UL)
/** @} */

/** @defgroup LL_GPTIM_MASTER_MODE Master Mode Selection */
/** @{ */
#define LL_GPTIM_MASTER_MODE_RESET          (0x0UL)
#define LL_GPTIM_MASTER_MODE_ENABLE         (0x1UL)
#define LL_GPTIM_MASTER_MODE_UPDATE         (0x2UL)
#define LL_GPTIM_MASTER_MODE_COMPARE_PULSE  (0x3UL)
#define LL_GPTIM_MASTER_MODE_COMPARE_OC1REF (0x4UL)
#define LL_GPTIM_MASTER_MODE_COMPARE_OC2REF (0x5UL)
#define LL_GPTIM_MASTER_MODE_COMPARE_OC3REF (0x6UL)
#define LL_GPTIM_MASTER_MODE_COMPARE_OC4REF (0x7UL)
/** @} */

/** @defgroup LL_GPTIM_TRIGGER_SRC Trigger Source (SMCR.TS) */
/** @{ */
#define LL_GPTIM_TRIGGER_SRC_ITR0    (0x0UL)
#define LL_GPTIM_TRIGGER_SRC_ITR1    (0x1UL)
#define LL_GPTIM_TRIGGER_SRC_ITR2    (0x2UL)
#define LL_GPTIM_TRIGGER_SRC_ITR3    (0x3UL)
#define LL_GPTIM_TRIGGER_SRC_TI1F_ED (0x4UL)
#define LL_GPTIM_TRIGGER_SRC_TI1FP1  (0x5UL)
#define LL_GPTIM_TRIGGER_SRC_TI2FP2  (0x6UL)
#define LL_GPTIM_TRIGGER_SRC_ETRF    (0x7UL)
/** @} */

/*==============================================================================
 * OC Mode (CCMRx.OCxM)
 *============================================================================*/

/** @defgroup LL_GPTIM_OC_MODE Output Compare Mode */
/** @{ */
#define LL_GPTIM_OC_MODE_FROZEN            (0x0UL)
#define LL_GPTIM_OC_MODE_ACTIVE_ON_MATCH   (0x1UL)
#define LL_GPTIM_OC_MODE_INACTIVE_ON_MATCH (0x2UL)
#define LL_GPTIM_OC_MODE_TOGGLE            (0x3UL)
#define LL_GPTIM_OC_MODE_FORCE_INACTIVE    (0x4UL)
#define LL_GPTIM_OC_MODE_FORCE_ACTIVE      (0x5UL)
#define LL_GPTIM_OC_MODE_PWM1              (0x6UL)
#define LL_GPTIM_OC_MODE_PWM2              (0x7UL)
#define LL_GPTIM_OC_MODE_RETRIG_OPM1       (0x8UL)
#define LL_GPTIM_OC_MODE_RETRIG_OPM2       (0x9UL)
#define LL_GPTIM_OC_MODE_COMBINED_PWM1     (0xCUL)
#define LL_GPTIM_OC_MODE_COMBINED_PWM2     (0xDUL)
#define LL_GPTIM_OC_MODE_ASYMMETRIC_PWM1   (0xEUL)
#define LL_GPTIM_OC_MODE_ASYMMETRIC_PWM2   (0xFUL)
/** @} */

/*==============================================================================
 * Channel Direction (CCMRx.CCxS)
 *============================================================================*/

/** @defgroup LL_GPTIM_CHANNEL_DIR Channel Direction */
/** @{ */
#define LL_GPTIM_CHANNEL_OUTPUT    (0x0UL)
#define LL_GPTIM_CHANNEL_INPUT_TIx (0x1UL)
#define LL_GPTIM_CHANNEL_INPUT_TIy (0x2UL)
#define LL_GPTIM_CHANNEL_INPUT_TRC (0x3UL)
/** @} */

/** @defgroup LL_GPTIM_RCR Repetition Counter Mask */
/** @{ */
#define LL_GPTIM_RCR_REP_Msk (0xFFUL)
static inline void ll_gptim_enable_uif_remap(GPT_TypeDef *tim)
{
    SET_BIT(tim->CR1, GPT_CR1_UIFREMAP);
}

static inline void ll_gptim_disable_uif_remap(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, GPT_CR1_UIFREMAP);
}

static inline void ll_gptim_enable_auto_reload_preload(GPT_TypeDef *tim)
{
    SET_BIT(tim->CR1, GPT_CR1_ARPE);
}

static inline void ll_gptim_disable_auto_reload_preload(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, GPT_CR1_ARPE);
}

/*==============================================================================
 * Counter Configuration
 *============================================================================*/

static inline void ll_gptim_set_counter_mode(GPT_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->CR1, GPT_CR1_CMS,
               MAKE_REG_VAL(mode, GPT_CR1_CMS_Msk, GPT_CR1_CMS_Pos));
}

static inline void ll_gptim_set_direction(GPT_TypeDef *tim, uint32_t dir)
{
    MODIFY_REG(tim->CR1, GPT_CR1_DIR,
               MAKE_REG_VAL(dir, GPT_CR1_DIR_Msk, GPT_CR1_DIR_Pos));
}

static inline void ll_gptim_enable_one_pulse(GPT_TypeDef *tim)
{
    SET_BIT(tim->CR1, GPT_CR1_OPM);
}

static inline void ll_gptim_disable_one_pulse(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, GPT_CR1_OPM);
}

/**
 * @brief Restrict update interrupt and DMA requests to counter overflow/underflow.
 * @note Software UG and slave reset can still load buffered registers without
 *       setting UIF or generating an update request.
 */
static inline void ll_gptim_set_update_request_overflow_only(GPT_TypeDef *tim)
{
    SET_BIT(tim->CR1, GPT_CR1_URS);
}

static inline void ll_gptim_set_update_request_any_event(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, GPT_CR1_URS);
}

/**
 * @brief Disable update event generation.
 * @note UG and slave reset still reinitialize the counter and prescaler, but
 *       buffered registers are not loaded and no update notification is sent.
 */
static inline void ll_gptim_disable_update_event(GPT_TypeDef *tim)
{
    SET_BIT(tim->CR1, GPT_CR1_UDIS);
}

static inline void ll_gptim_enable_update_event(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, GPT_CR1_UDIS);
}

/** @} */

/*==============================================================================
 * Basic Control
 *============================================================================*/

/**
 * @brief Enable counter
 */
static inline void ll_gptim_enable(GPT_TypeDef *tim)
{
    SET_BIT(tim->CR1, GPT_CR1_CEN);
}

/**
 * @brief Disable counter
 */
static inline void ll_gptim_disable(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, GPT_CR1_CEN);
}

/**
 * @brief Check if counter is enabled
 */
static inline uint32_t ll_gptim_is_enabled(GPT_TypeDef *tim)
{
    return READ_BIT(tim->CR1, GPT_CR1_CEN) ? 1UL : 0UL;
}

/**
 * @brief Select the XOR combination of TI1, TI2 and TI3 as TI1.
 * @param[in] tim GPTIM instance pointer.
 */
static inline void ll_gptim_ti1_selection_enable(GPT_TypeDef *tim)
{
    SET_BIT(tim->CR2, GPT_CR2_TI1S);
}

/**
 * @brief Select the TIMx_CH1 pin as TI1.
 * @param[in] tim GPTIM instance pointer.
 */
static inline void ll_gptim_ti1_selection_disable(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->CR2, GPT_CR2_TI1S);
}

static inline void ll_gptim_set_master_mode(GPT_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->CR2, GPT_CR2_MMS,
               MAKE_REG_VAL(mode, GPT_CR2_MMS_Msk, GPT_CR2_MMS_Pos));
}

/**
 * @brief Select the capture/compare DMA request source.
 * @param[in] tim GPTIM instance pointer.
 * @param[in] selection 0 for CC event, 1 for update event.
 */
static inline void ll_gptim_set_capture_compare_dma_selection(GPT_TypeDef *tim,
        uint32_t selection)
{
    MODIFY_REG(tim->CR2, GPT_CR2_CCDS,
               MAKE_REG_VAL(selection, GPT_CR2_CCDS_Msk, GPT_CR2_CCDS_Pos));
}

/*==============================================================================
 * Slave Mode
 *============================================================================*/

static inline void ll_gptim_set_slave_mode(GPT_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->SMCR, GPT_SMCR_SMS,
               MAKE_REG_VAL(mode, GPT_SMCR_SMS_Msk, GPT_SMCR_SMS_Pos));
}

static inline void ll_gptim_set_external_trigger_polarity(GPT_TypeDef *tim,
        uint32_t polarity)
{
    MODIFY_REG(tim->SMCR, GPT_SMCR_ETP,
               MAKE_REG_VAL(polarity, GPT_SMCR_ETP_Msk, GPT_SMCR_ETP_Pos));
}

static inline void ll_gptim_enable_external_clock(GPT_TypeDef *tim)
{
    SET_BIT(tim->SMCR, GPT_SMCR_ECE);
}

static inline void ll_gptim_disable_external_clock(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->SMCR, GPT_SMCR_ECE);
}

static inline void ll_gptim_set_external_trigger_prescaler(GPT_TypeDef *tim,
        uint32_t prescaler)
{
    MODIFY_REG(tim->SMCR, GPT_SMCR_ETPS,
               MAKE_REG_VAL(prescaler, GPT_SMCR_ETPS_Msk, GPT_SMCR_ETPS_Pos));
}

static inline void ll_gptim_set_external_trigger_filter(GPT_TypeDef *tim,
        uint32_t filter)
{
    MODIFY_REG(tim->SMCR, GPT_SMCR_ETF,
               MAKE_REG_VAL(filter, GPT_SMCR_ETF_Msk, GPT_SMCR_ETF_Pos));
}

static inline void ll_gptim_enable_master_slave_mode(GPT_TypeDef *tim)
{
    SET_BIT(tim->SMCR, GPT_SMCR_MSM);
}

static inline void ll_gptim_disable_master_slave_mode(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->SMCR, GPT_SMCR_MSM);
}

static inline void ll_gptim_set_trigger_source(GPT_TypeDef *tim, uint32_t src)
{
    MODIFY_REG(tim->SMCR, GPT_SMCR_TS,
               MAKE_REG_VAL(src, GPT_SMCR_TS_Msk, GPT_SMCR_TS_Pos));
}

static inline void ll_gptim_enable_trigger_dma(GPT_TypeDef *tim)
{
    SET_BIT(tim->DIER, GPT_DIER_TDE);
}

static inline void ll_gptim_disable_trigger_dma(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, GPT_DIER_TDE);
}

static inline void ll_gptim_enable_channel_dma(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        SET_BIT(tim->DIER, GPT_DIER_CC1DE);
        break;
    case 2:
        SET_BIT(tim->DIER, GPT_DIER_CC2DE);
        break;
    case 3:
        SET_BIT(tim->DIER, GPT_DIER_CC3DE);
        break;
    case 4:
        SET_BIT(tim->DIER, GPT_DIER_CC4DE);
        break;
    default:
        break;
    }
}

static inline void ll_gptim_disable_channel_dma(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        CLEAR_BIT(tim->DIER, GPT_DIER_CC1DE);
        break;
    case 2:
        CLEAR_BIT(tim->DIER, GPT_DIER_CC2DE);
        break;
    case 3:
        CLEAR_BIT(tim->DIER, GPT_DIER_CC3DE);
        break;
    case 4:
        CLEAR_BIT(tim->DIER, GPT_DIER_CC4DE);
        break;
    default:
        break;
    }
}

static inline void ll_gptim_enable_update_dma(GPT_TypeDef *tim)
{
    SET_BIT(tim->DIER, GPT_DIER_UDE);
}

static inline void ll_gptim_disable_update_dma(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, GPT_DIER_UDE);
}

static inline void ll_gptim_enable_trigger_interrupt(GPT_TypeDef *tim)
{
    SET_BIT(tim->DIER, GPT_DIER_TIE);
}

static inline void ll_gptim_disable_trigger_interrupt(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, GPT_DIER_TIE);
}

/**
 * @brief Enable channel interrupt / DMA
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_enable_channel_interrupt(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        SET_BIT(tim->DIER, GPT_DIER_CC1IE);
        break;
    case 2:
        SET_BIT(tim->DIER, GPT_DIER_CC2IE);
        break;
    case 3:
        SET_BIT(tim->DIER, GPT_DIER_CC3IE);
        break;
    case 4:
        SET_BIT(tim->DIER, GPT_DIER_CC4IE);
        break;
    default:
        break;
    }
}

/**
 * @brief Disable channel interrupt.
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_disable_channel_interrupt(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        CLEAR_BIT(tim->DIER, GPT_DIER_CC1IE);
        break;
    case 2:
        CLEAR_BIT(tim->DIER, GPT_DIER_CC2IE);
        break;
    case 3:
        CLEAR_BIT(tim->DIER, GPT_DIER_CC3IE);
        break;
    case 4:
        CLEAR_BIT(tim->DIER, GPT_DIER_CC4IE);
        break;
    default:
        break;
    }
}

/*==============================================================================
 * Interrupt / DMA
 *============================================================================*/

/**
 * @brief Enable update interrupt
 */
static inline void ll_gptim_enable_update_interrupt(GPT_TypeDef *tim)
{
    SET_BIT(tim->DIER, GPT_DIER_UIE);
}

static inline void ll_gptim_disable_update_interrupt(GPT_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, GPT_DIER_UIE);
}

static inline uint32_t ll_gptim_get_overcapture_flag(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        return READ_BIT(tim->SR, GPT_SR_CC1OF) ? 1UL : 0UL;
    case 2:
        return READ_BIT(tim->SR, GPT_SR_CC2OF) ? 1UL : 0UL;
    case 3:
        return READ_BIT(tim->SR, GPT_SR_CC3OF) ? 1UL : 0UL;
    case 4:
        return READ_BIT(tim->SR, GPT_SR_CC4OF) ? 1UL : 0UL;
    default:
        return 0UL;
    }
}

static inline void ll_gptim_clear_overcapture_flag(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        WRITE_REG(tim->SR, ~GPT_SR_CC1OF);
        break;
    case 2:
        WRITE_REG(tim->SR, ~GPT_SR_CC2OF);
        break;
    case 3:
        WRITE_REG(tim->SR, ~GPT_SR_CC3OF);
        break;
    case 4:
        WRITE_REG(tim->SR, ~GPT_SR_CC4OF);
        break;
    default:
        break;
    }
}

static inline uint32_t ll_gptim_get_trigger_flag(GPT_TypeDef *tim)
{
    return READ_BIT(tim->SR, GPT_SR_TIF) ? 1UL : 0UL;
}

static inline void ll_gptim_clear_trigger_flag(GPT_TypeDef *tim)
{
    WRITE_REG(tim->SR, ~GPT_SR_TIF);
}

/**
 * @brief Check channel capture/compare flag.
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @return Non-zero when the channel flag is set; 0 for an invalid channel number.
 */
static inline uint32_t ll_gptim_get_channel_flag(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        return READ_BIT(tim->SR, GPT_SR_CC1IF) ? 1UL : 0UL;
    case 2:
        return READ_BIT(tim->SR, GPT_SR_CC2IF) ? 1UL : 0UL;
    case 3:
        return READ_BIT(tim->SR, GPT_SR_CC3IF) ? 1UL : 0UL;
    case 4:
        return READ_BIT(tim->SR, GPT_SR_CC4IF) ? 1UL : 0UL;
    default:
        return 0UL;
    }
}

static inline void ll_gptim_clear_channel_flag(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        WRITE_REG(tim->SR, ~GPT_SR_CC1IF);
        break;
    case 2:
        WRITE_REG(tim->SR, ~GPT_SR_CC2IF);
        break;
    case 3:
        WRITE_REG(tim->SR, ~GPT_SR_CC3IF);
        break;
    case 4:
        WRITE_REG(tim->SR, ~GPT_SR_CC4IF);
        break;
    default:
        break;
    }
}

static inline uint32_t ll_gptim_get_update_flag(GPT_TypeDef *tim)
{
    return READ_BIT(tim->SR, GPT_SR_UIF) ? 1UL : 0UL;
}

static inline void ll_gptim_clear_update_flag(GPT_TypeDef *tim)
{
    WRITE_REG(tim->SR, ~GPT_SR_UIF);
}

static inline void ll_gptim_generate_trigger_event(GPT_TypeDef *tim)
{
    WRITE_REG(tim->EGR, GPT_EGR_TG);
}

static inline void ll_gptim_generate_channel_event(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        WRITE_REG(tim->EGR, GPT_EGR_CC1G);
        break;
    case 2:
        WRITE_REG(tim->EGR, GPT_EGR_CC2G);
        break;
    case 3:
        WRITE_REG(tim->EGR, GPT_EGR_CC3G);
        break;
    case 4:
        WRITE_REG(tim->EGR, GPT_EGR_CC4G);
        break;
    default:
        break;
    }
}

/**
 * @brief Generate update event (reinitialize counter & registers)
 */
static inline void ll_gptim_generate_update(GPT_TypeDef *tim)
{
    WRITE_REG(tim->EGR, GPT_EGR_UG);
}

/**
 * @brief Set output compare mode (CCMRx.OCxM).
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @param[in] mode Output compare mode, one of @ref LL_GPTIM_OC_MODE.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_set_output_compare_mode(GPT_TypeDef *tim, uint32_t ch, uint32_t mode)
{
    switch (ch)
    {
    case 1:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_OC1M,
                   MAKE_REG_VAL(mode, GPT_CCMR1_OC1M_Msk, GPT_CCMR1_OC1M_Pos));
        break;
    case 2:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_OC2M,
                   MAKE_REG_VAL(mode, GPT_CCMR1_OC2M_Msk, GPT_CCMR1_OC2M_Pos));
        break;
    case 3:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_OC3M,
                   MAKE_REG_VAL(mode, GPT_CCMR2_OC3M_Msk, GPT_CCMR2_OC3M_Pos));
        break;
    case 4:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_OC4M,
                   MAKE_REG_VAL(mode, GPT_CCMR2_OC4M_Msk, GPT_CCMR2_OC4M_Pos));
        break;
    default:
        break;
    }
}

/**
 * @brief Enable or disable output compare preload (CCMRx.OCxPE).
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @param[in] en Non-zero to enable preload, zero to disable it.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_set_output_compare_preload(GPT_TypeDef *tim, uint32_t ch, uint32_t en)
{
    switch (ch)
    {
    case 1:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_OC1PE, en ? GPT_CCMR1_OC1PE : 0UL);
        break;
    case 2:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_OC2PE, en ? GPT_CCMR1_OC2PE : 0UL);
        break;
    case 3:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_OC3PE, en ? GPT_CCMR2_OC3PE : 0UL);
        break;
    case 4:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_OC4PE, en ? GPT_CCMR2_OC4PE : 0UL);
        break;
    default:
        break;
    }
}

static inline void ll_gptim_set_channel_clear(GPT_TypeDef *tim, uint32_t ch, uint32_t en)
{
    switch (ch)
    {
    case 1U:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_OC1CE, en ? GPT_CCMR1_OC1CE : 0UL);
        break;
    case 2U:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_OC2CE, en ? GPT_CCMR1_OC2CE : 0UL);
        break;
    case 3U:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_OC3CE, en ? GPT_CCMR2_OC3CE : 0UL);
        break;
    case 4U:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_OC4CE, en ? GPT_CCMR2_OC4CE : 0UL);
        break;
    default:
        break;
    }
}

/**
 * @brief Configure input capture prescaler and filter (CCMRx.ICxPSC/ICxF).
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @param[in] prescaler Input capture prescaler (0 to 3).
 * @param[in] filter Input capture filter (0 to 15).
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_set_input_capture_config(GPT_TypeDef *tim, uint32_t ch,
        uint32_t prescaler, uint32_t filter)
{
    switch (ch)
    {
    case 1:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_IC1PSC | GPT_CCMR1_IC1F,
                   MAKE_REG_VAL(prescaler, GPT_CCMR1_IC1PSC_Msk, GPT_CCMR1_IC1PSC_Pos) |
                   MAKE_REG_VAL(filter, GPT_CCMR1_IC1F_Msk, GPT_CCMR1_IC1F_Pos));
        break;
    case 2:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_IC2PSC | GPT_CCMR1_IC2F,
                   MAKE_REG_VAL(prescaler, GPT_CCMR1_IC2PSC_Msk, GPT_CCMR1_IC2PSC_Pos) |
                   MAKE_REG_VAL(filter, GPT_CCMR1_IC2F_Msk, GPT_CCMR1_IC2F_Pos));
        break;
    case 3:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_IC3PSC | GPT_CCMR2_IC3F,
                   MAKE_REG_VAL(prescaler, GPT_CCMR2_IC3PSC_Msk, GPT_CCMR2_IC3PSC_Pos) |
                   MAKE_REG_VAL(filter, GPT_CCMR2_IC3F_Msk, GPT_CCMR2_IC3F_Pos));
        break;
    case 4:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_IC4PSC | GPT_CCMR2_IC4F,
                   MAKE_REG_VAL(prescaler, GPT_CCMR2_IC4PSC_Msk, GPT_CCMR2_IC4PSC_Pos) |
                   MAKE_REG_VAL(filter, GPT_CCMR2_IC4F_Msk, GPT_CCMR2_IC4F_Pos));
        break;
    default:
        break;
    }
}

/*==============================================================================
 * Channel Configuration (CCMRx / CCER)
 *============================================================================*/

/**
 * @brief Set channel direction (CCMRx.CCxS).
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @param[in] dir Direction, one of @ref LL_GPTIM_CHANNEL_DIR.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_set_channel_direction(GPT_TypeDef *tim, uint32_t ch, uint32_t dir)
{
    switch (ch)
    {
    case 1:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_CC1S,
                   MAKE_REG_VAL(dir, GPT_CCMR1_CC1S_Msk, GPT_CCMR1_CC1S_Pos));
        break;
    case 2:
        MODIFY_REG(tim->CCMR1, GPT_CCMR1_CC2S,
                   MAKE_REG_VAL(dir, GPT_CCMR1_CC2S_Msk, GPT_CCMR1_CC2S_Pos));
        break;
    case 3:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_CC3S,
                   MAKE_REG_VAL(dir, GPT_CCMR2_CC3S_Msk, GPT_CCMR2_CC3S_Pos));
        break;
    case 4:
        MODIFY_REG(tim->CCMR2, GPT_CCMR2_CC4S,
                   MAKE_REG_VAL(dir, GPT_CCMR2_CC4S_Msk, GPT_CCMR2_CC4S_Pos));
        break;
    default:
        break;
    }
}

static inline void ll_gptim_set_channel_complementary_polarity(GPT_TypeDef *tim,
        uint32_t ch, uint32_t pol)
{
    switch (ch)
    {
    case 1U:
        MODIFY_REG(tim->CCER, GPT_CCER_CC1NP, pol ? GPT_CCER_CC1NP : 0UL);
        break;
    case 2U:
        MODIFY_REG(tim->CCER, GPT_CCER_CC2NP, pol ? GPT_CCER_CC2NP : 0UL);
        break;
    case 3U:
        MODIFY_REG(tim->CCER, GPT_CCER_CC3NP, pol ? GPT_CCER_CC3NP : 0UL);
        break;
    case 4U:
        MODIFY_REG(tim->CCER, GPT_CCER_CC4NP, pol ? GPT_CCER_CC4NP : 0UL);
        break;
    default:
        break;
    }
}

/**
 * @brief Set channel output polarity (CCER.CCxP).
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @param[in] pol 0 for active high, 1 for active low.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_set_channel_polarity(GPT_TypeDef *tim, uint32_t ch, uint32_t pol)
{
    switch (ch)
    {
    case 1:
        MODIFY_REG(tim->CCER, GPT_CCER_CC1P, pol ? GPT_CCER_CC1P : 0UL);
        break;
    case 2:
        MODIFY_REG(tim->CCER, GPT_CCER_CC2P, pol ? GPT_CCER_CC2P : 0UL);
        break;
    case 3:
        MODIFY_REG(tim->CCER, GPT_CCER_CC3P, pol ? GPT_CCER_CC3P : 0UL);
        break;
    case 4:
        MODIFY_REG(tim->CCER, GPT_CCER_CC4P, pol ? GPT_CCER_CC4P : 0UL);
        break;
    default:
        break;
    }
}

/**
 * @brief Enable channel output (CCER.CCxE)
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_enable_channel(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        SET_BIT(tim->CCER, GPT_CCER_CC1E);
        break;
    case 2:
        SET_BIT(tim->CCER, GPT_CCER_CC2E);
        break;
    case 3:
        SET_BIT(tim->CCER, GPT_CCER_CC3E);
        break;
    case 4:
        SET_BIT(tim->CCER, GPT_CCER_CC4E);
        break;
    default:
        break;
    }
}

/**
 * @brief Disable channel output (CCER.CCxE).
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_disable_channel(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        CLEAR_BIT(tim->CCER, GPT_CCER_CC1E);
        break;
    case 2:
        CLEAR_BIT(tim->CCER, GPT_CCER_CC2E);
        break;
    case 3:
        CLEAR_BIT(tim->CCER, GPT_CCER_CC3E);
        break;
    case 4:
        CLEAR_BIT(tim->CCER, GPT_CCER_CC4E);
        break;
    default:
        break;
    }
}

static inline uint32_t ll_gptim_get_counter_uif_copy(GPT_TypeDef *tim)
{
    return READ_BIT(tim->CNT, GPT_CNT_UIFCPY) ? 1UL : 0UL;
}

/*==============================================================================
 * Counter Value
 *============================================================================*/

static inline void ll_gptim_set_counter(GPT_TypeDef *tim, uint16_t cnt)
{
    WRITE_REG(tim->CNT, ((uint32_t)cnt & GPT_CNT_CNT));
}

static inline uint16_t ll_gptim_get_counter(GPT_TypeDef *tim)
{
    return (uint16_t)GET_REG_VAL(READ_REG(tim->CNT), GPT_CNT_CNT_Msk, GPT_CNT_CNT_Pos);
}

/*==============================================================================
 * Prescaler & Auto-Reload
 *============================================================================*/

/**
 * @brief Set prescaler value. f_cnt = f_clk / (psc + 1)
 */
static inline void ll_gptim_set_prescaler(GPT_TypeDef *tim, uint16_t psc)
{
    WRITE_REG(tim->PSC, psc);
}

static inline uint16_t ll_gptim_get_prescaler(GPT_TypeDef *tim)
{
    return (uint16_t)READ_REG(tim->PSC);
}

/**
 * @brief Set auto-reload value
 */
static inline void ll_gptim_set_auto_reload(GPT_TypeDef *tim, uint16_t arr)
{
    WRITE_REG(tim->ARR, arr);
}

static inline uint16_t ll_gptim_get_auto_reload(GPT_TypeDef *tim)
{
    return (uint16_t)READ_REG(tim->ARR);
}

/**
 * @brief Set repetition counter value. PWM period count = REP + 1.
 */
static inline void ll_gptim_set_repetition(GPT_TypeDef *tim, uint8_t rep)
{
    MODIFY_REG(tim->RCR, LL_GPTIM_RCR_REP_Msk, rep);
}

/*==============================================================================
 * Capture/Compare Channels
 *============================================================================*/

/**
 * @brief Set compare/capture value for channel 1-4
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @param[in] val Compare/capture value.
 * @note Invalid channel numbers are ignored.
 */
static inline void ll_gptim_set_compare(GPT_TypeDef *tim, uint32_t ch, uint16_t val)
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
    default:
        break;
    }
}

/**
 * @brief Read capture value for a channel.
 * @param[in] tim GPTIM instance pointer.
 * @param[in] ch Channel number in the range 1 to 4.
 * @return Capture value, or 0 for an invalid channel number.
 */
static inline uint16_t ll_gptim_get_capture(GPT_TypeDef *tim, uint32_t ch)
{
    switch (ch)
    {
    case 1:
        return (uint16_t)READ_REG(tim->CCR1);
    case 2:
        return (uint16_t)READ_REG(tim->CCR2);
    case 3:
        return (uint16_t)READ_REG(tim->CCR3);
    case 4:
        return (uint16_t)READ_REG(tim->CCR4);
    default:
        return 0U;
    }
}

static inline void ll_gptim_set_debug_selection(GPT_TypeDef *tim, uint32_t selection)
{
    MODIFY_REG(tim->DBGSEL, GPT_DBGSEL_DBGSEL,
               MAKE_REG_VAL(selection, GPT_DBGSEL_DBGSEL_Msk, GPT_DBGSEL_DBGSEL_Pos));
}

static inline uint32_t ll_gptim_get_debug_selection(GPT_TypeDef *tim)
{
    return GET_REG_VAL(READ_REG(tim->DBGSEL), GPT_DBGSEL_DBGSEL_Msk,
                       GPT_DBGSEL_DBGSEL_Pos);
}

/*==============================================================================
 * Frequency Calculation
 *============================================================================*/

/**
 * @brief Calculate timer update frequency.
 * @param[in] clk_freq Timer input clock frequency in Hz.
 * @param[in] psc Prescaler value in the range 0 to 65535.
 * @param[in] arr Auto-reload value in the range 0 to 65535.
 * @return Update event frequency in Hz.
 */
static inline uint32_t ll_gptim_calc_update_freq(uint32_t clk_freq, uint16_t psc, uint16_t arr)
{
    return (uint32_t)(clk_freq / (((uint64_t)psc + 1U) * ((uint64_t)arr + 1U)));
}

/**
 * @brief Calculate the compare value for a PWM duty cycle.
 * @param[in] arr Auto-reload value.
 * @param[in] permille Duty cycle in 0.1% (1000 = 100.0%).
 * @return Compare value corresponding to @p permille.
 */
static inline uint16_t ll_gptim_calc_pwm_duty(uint16_t arr, uint32_t permille)
{
    uint32_t cmp;

    if (permille >= 1000U)
    {
        return arr;
    }

    cmp = ((uint32_t)(arr + 1U) * permille) / 1000U;
    return (uint16_t)cmp;
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_GPTIM_H */
