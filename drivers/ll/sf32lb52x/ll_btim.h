/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_BTIM_H
#define __LL_BTIM_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_btim.h
 * @brief Header-only low-level Basic Timer (BTIM) APIs for SF32LB52x.
 *
 * BTIM is a simple timer with basic counting and auto-reload, no I/O channels.
 */

/** @defgroup LL_BTIM_MASTER_MODE Master Mode Selection */
/** @{ */
#define LL_BTIM_MASTER_MODE_RESET  (0x0UL)
#define LL_BTIM_MASTER_MODE_ENABLE (0x1UL)
#define LL_BTIM_MASTER_MODE_UPDATE (0x2UL)
#define LL_BTIM_MASTER_MODE_GATING (0x3UL)
/** @} */

/** @defgroup LL_BTIM_SLAVE_MODE Slave Mode Selection */
/** @{ */
#define LL_BTIM_SLAVE_MODE_DISABLED (0x0UL)
#define LL_BTIM_SLAVE_MODE_RESET    (0x1UL)
#define LL_BTIM_SLAVE_MODE_TRIGGER  (0x2UL)
#define LL_BTIM_SLAVE_MODE_COMBINED (0x3UL)
#define LL_BTIM_SLAVE_MODE_EXT_CLK  (0x4UL)
/** @} */

/** @defgroup LL_BTIM_TRIGGER_SOURCE Trigger Source Selection */
/** @{ */
#define LL_BTIM_TRIGGER_SOURCE_ITR0 (0x0UL)
#define LL_BTIM_TRIGGER_SOURCE_ITR1 (0x1UL)
#define LL_BTIM_TRIGGER_SOURCE_ITR2 (0x2UL)
#define LL_BTIM_TRIGGER_SOURCE_ITR3 (0x3UL)
/** @} */

/** @defgroup LL_BTIM_GATING_POLARITY Gating Trigger Polarity */
/** @{ */
#define LL_BTIM_GATING_POLARITY_HIGH (0x0UL)
#define LL_BTIM_GATING_POLARITY_LOW  (0x1UL)
static inline void ll_btim_enable_auto_reload_preload(BTIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, BTIM_CR1_ARPE);
}

static inline void ll_btim_disable_auto_reload_preload(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, BTIM_CR1_ARPE);
}

static inline void ll_btim_enable_one_pulse(BTIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, BTIM_CR1_OPM);
}

static inline void ll_btim_disable_one_pulse(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, BTIM_CR1_OPM);
}

/**
 * @brief Restrict update interrupt and DMA requests to counter overflow.
 * @note Software UG and slave reset can still load buffered registers without
 *       setting UIF or generating an update request.
 */
static inline void ll_btim_set_update_request_overflow_only(BTIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, BTIM_CR1_URS);
}

static inline void ll_btim_set_update_request_any_event(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, BTIM_CR1_URS);
}

/**
 * @brief Disable update event generation.
 * @note UG and slave reset still reinitialize the counter and prescaler, but
 *       buffered registers are not loaded and no update notification is sent.
 */
static inline void ll_btim_disable_update_event(BTIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, BTIM_CR1_UDIS);
}

static inline void ll_btim_enable_update_event(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, BTIM_CR1_UDIS);
}

/** @} */

/**
 * @brief Enable counter
 */
static inline void ll_btim_enable(BTIM_TypeDef *tim)
{
    SET_BIT(tim->CR1, BTIM_CR1_CEN);
}

/**
 * @brief Disable counter
 */
static inline void ll_btim_disable(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->CR1, BTIM_CR1_CEN);
}

static inline uint32_t ll_btim_is_enabled(BTIM_TypeDef *tim)
{
    return READ_BIT(tim->CR1, BTIM_CR1_CEN) ? 1UL : 0UL;
}

/*==============================================================================
 * Master / Slave and Gating Control
 *============================================================================*/

static inline void ll_btim_set_master_mode(BTIM_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->CR2, BTIM_CR2_MMS, MAKE_REG_VAL(mode, BTIM_CR2_MMS, BTIM_CR2_MMS_Pos));
}

/**
 * @brief Enable gated mode.
 * @note The counter must also be enabled through CR1.CEN.
 */
static inline void ll_btim_enable_gated_mode(BTIM_TypeDef *tim)
{
    SET_BIT(tim->SMCR, BTIM_SMCR_GM);
}

static inline void ll_btim_disable_gated_mode(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->SMCR, BTIM_SMCR_GM);
}

static inline void ll_btim_set_gating_trigger_polarity(BTIM_TypeDef *tim, uint32_t polarity)
{
    MODIFY_REG(tim->SMCR, BTIM_SMCR_GTP, MAKE_REG_VAL(polarity, BTIM_SMCR_GTP, BTIM_SMCR_GTP_Pos));
}

static inline void ll_btim_set_gating_trigger_source(BTIM_TypeDef *tim, uint32_t source)
{
    MODIFY_REG(tim->SMCR, BTIM_SMCR_GTS, MAKE_REG_VAL(source, BTIM_SMCR_GTS, BTIM_SMCR_GTS_Pos));
}

static inline void ll_btim_set_slave_mode(BTIM_TypeDef *tim, uint32_t mode)
{
    MODIFY_REG(tim->SMCR, BTIM_SMCR_SMS, MAKE_REG_VAL(mode, BTIM_SMCR_SMS, BTIM_SMCR_SMS_Pos));
}

static inline void ll_btim_enable_master_slave_mode(BTIM_TypeDef *tim)
{
    SET_BIT(tim->SMCR, BTIM_SMCR_MSM);
}

static inline void ll_btim_disable_master_slave_mode(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->SMCR, BTIM_SMCR_MSM);
}

static inline void ll_btim_set_trigger_source(BTIM_TypeDef *tim, uint32_t source)
{
    MODIFY_REG(tim->SMCR, BTIM_SMCR_TS, MAKE_REG_VAL(source, BTIM_SMCR_TS, BTIM_SMCR_TS_Pos));
}

static inline void ll_btim_enable_update_dma(BTIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, BTIM_DIER_UDE);
}

static inline void ll_btim_disable_update_dma(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, BTIM_DIER_UDE);
}

static inline void ll_btim_enable_update_interrupt(BTIM_TypeDef *tim)
{
    SET_BIT(tim->DIER, BTIM_DIER_UIE);
}

static inline void ll_btim_disable_update_interrupt(BTIM_TypeDef *tim)
{
    CLEAR_BIT(tim->DIER, BTIM_DIER_UIE);
}

static inline void ll_btim_clear_update_flag(BTIM_TypeDef *tim)
{
    WRITE_REG(tim->SR, 0U);
}

static inline uint32_t ll_btim_get_update_flag(BTIM_TypeDef *tim)
{
    return READ_BIT(tim->SR, BTIM_SR_UIF) ? 1UL : 0UL;
}

static inline void ll_btim_generate_update(BTIM_TypeDef *tim)
{
    WRITE_REG(tim->EGR, BTIM_EGR_UG);
}

static inline void ll_btim_set_counter(BTIM_TypeDef *tim, uint32_t cnt)
{
    WRITE_REG(tim->CNT, cnt);
}

static inline uint32_t ll_btim_get_counter(BTIM_TypeDef *tim)
{
    return READ_REG(tim->CNT);
}

/**
 * @brief Set prescaler. f_cnt = f_clk / (psc + 1)
 */
static inline void ll_btim_set_prescaler(BTIM_TypeDef *tim, uint16_t psc)
{
    WRITE_REG(tim->PSC, psc);
}

static inline uint16_t ll_btim_get_prescaler(BTIM_TypeDef *tim)
{
    return (uint16_t)READ_REG(tim->PSC);
}

/**
 * @brief Set auto-reload value
 */
static inline void ll_btim_set_auto_reload(BTIM_TypeDef *tim, uint32_t arr)
{
    WRITE_REG(tim->ARR, arr);
}

static inline uint32_t ll_btim_get_auto_reload(BTIM_TypeDef *tim)
{
    return READ_REG(tim->ARR);
}

/**
 * @brief Calculate update frequency.
 * @param[in] clk_freq Timer input clock frequency in Hz.
 * @param[in] psc Prescaler value in the range 0 to 65535.
 * @param[in] arr Auto-reload value in the range 1 to 4294967295.
 * @return Update frequency in Hz, or 0 when ARR is zero.
 */
static inline uint32_t ll_btim_calc_update_freq(uint32_t clk_freq, uint16_t psc, uint32_t arr)
{
    if (arr == 0U)
    {
        return 0U;
    }

    return (uint32_t)(clk_freq / (((uint64_t)psc + 1U) * ((uint64_t)arr + 1U)));
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_BTIM_H */
