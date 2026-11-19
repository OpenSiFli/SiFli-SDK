/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_TSEN_H
#define __LL_TSEN_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set the TSEN clock divider (ANAU_TSEN_CLK_DIV[17:12]).
 * @note f_tsen = f_pclk / div; the TSEN clock should be 1MHz or 2MHz
 *       (see reference manual 8.2.3.1 / 8.2.3.3).
 * @param[in] tsen TSEN instance pointer.
 * @param[in] div  Divider value, 0..63.
 */
static inline void ll_tsen_set_clock_div(TSEN_TypeDef *tsen, uint32_t div)
{
    MODIFY_REG(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_CLK_DIV,
               MAKE_REG_VAL(div, TSEN_TSEN_CTRL_REG_ANAU_TSEN_CLK_DIV_Msk,
                            TSEN_TSEN_CTRL_REG_ANAU_TSEN_CLK_DIV_Pos));
}

/**
 * @file ll_tsen.h
 * @brief Header-only low-level Temperature Sensor (TSEN) APIs for SF32LB52x.
 *
 * TSEN provides an on-chip temperature sensor with 12-bit ADC readout.
 */

/*==============================================================================
 * Basic Control
 *
 * Recommended start sequence (see reference manual 8.2.3.3):
 *   1. ll_tsen_set_clock_div()  -> f_tsen = 1MHz or 2MHz
 *   2. ll_tsen_enable()         -> ANAU_TSEN_EN = 1
 *   3. ll_tsen_power_up()       -> ANAU_TSEN_PU = 1
 *   4. ll_tsen_assert_reset() then ll_tsen_release_reset() -> RSTB 0 -> 1
 *   5. ll_tsen_start()          -> ANAU_TSEN_RUN = 1, then poll/read data
 *============================================================================*/

/**
 * @brief Enable the TSEN digital module (ANAU_TSEN_EN).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_enable(TSEN_TypeDef *tsen)
{
    SET_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_EN);
}

/**
 * @brief Disable the TSEN digital module (ANAU_TSEN_EN).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_disable(TSEN_TypeDef *tsen)
{
    CLEAR_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_EN);
}

/**
 * @brief Check whether TSEN is ready (ANAU_TSEN_RDY).
 * @param[in] tsen TSEN instance pointer.
 * @return Non-zero when TSEN is ready.
 */
static inline uint32_t ll_tsen_is_ready(TSEN_TypeDef *tsen)
{
    return READ_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_RDY) ? 1UL : 0UL;
}

/**
 * @brief Select serial or parallel output (ANAU_TSEN_SER_PAR_SEL).
 * @param[in] tsen  TSEN instance pointer.
 * @param[in] mode  0=parallel, 1=serial.
 */
static inline void ll_tsen_set_sd_mode(TSEN_TypeDef *tsen, uint32_t mode)
{
    MODIFY_REG(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_SER_PAR_SEL,
               MAKE_REG_VAL(mode, TSEN_TSEN_CTRL_REG_ANAU_TSEN_SER_PAR_SEL_Msk,
                            TSEN_TSEN_CTRL_REG_ANAU_TSEN_SER_PAR_SEL_Pos));
}

/**
 * @brief Enable the sigma-delta signature mode (ANAU_TSEN_SGN_EN).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_sd_enable(TSEN_TypeDef *tsen)
{
    SET_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_SGN_EN);
}

/**
 * @brief Disable the sigma-delta signature mode (ANAU_TSEN_SGN_EN = 0).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_sd_disable(TSEN_TypeDef *tsen)
{
    CLEAR_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_SGN_EN);
}

/**
 * @brief Select the internal clock frequency (ANAU_TSEN_FCK_SEL[7:6]).
 * @param[in] tsen TSEN instance pointer.
 * @param[in] sel  Clock frequency select, 0..3.
 */
static inline void ll_tsen_set_fck_sel(TSEN_TypeDef *tsen, uint32_t sel)
{
    MODIFY_REG(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_FCK_SEL,
               MAKE_REG_VAL(sel, TSEN_TSEN_CTRL_REG_ANAU_TSEN_FCK_SEL_Msk,
                            TSEN_TSEN_CTRL_REG_ANAU_TSEN_FCK_SEL_Pos));
}

/*==============================================================================
 * Configuration
 *============================================================================*/

/**
 * @brief Set the bias current for VBE generation (ANAU_TSEN_IG_VBE[5:3]).
 * @param[in] tsen   TSEN instance pointer.
 * @param[in] ig_vbe Bias current code, 0..7.
 */
static inline void ll_tsen_set_bias_current(TSEN_TypeDef *tsen, uint32_t ig_vbe)
{
    MODIFY_REG(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_IG_VBE,
               MAKE_REG_VAL(ig_vbe, TSEN_TSEN_CTRL_REG_ANAU_TSEN_IG_VBE_Msk,
                            TSEN_TSEN_CTRL_REG_ANAU_TSEN_IG_VBE_Pos));
}

/*==============================================================================
 * Measurement Control
 *============================================================================*/

/**
 * @brief Start a measurement (ANAU_TSEN_RUN = 1).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_start(TSEN_TypeDef *tsen)
{
    SET_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_RUN);
}

/**
 * @brief Stop the measurement (ANAU_TSEN_RUN = 0).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_stop(TSEN_TypeDef *tsen)
{
    CLEAR_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_RUN);
}

/**
 * @brief De-assert the TSEN reset (ANAU_TSEN_RSTB = 1, active-low reset).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_release_reset(TSEN_TypeDef *tsen)
{
    SET_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_RSTB);
}

/**
 * @brief Assert the TSEN reset (ANAU_TSEN_RSTB = 0, active-low reset).
 * @note Hold the reset low for at least 20us before releasing
 *       (see reference manual 8.2.3.3).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_assert_reset(TSEN_TypeDef *tsen)
{
    CLEAR_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_RSTB);
}

/**
 * @brief Power up the TSEN analog (ANAU_TSEN_PU).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_power_up(TSEN_TypeDef *tsen)
{
    SET_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_PU);
}

/**
 * @brief Power down the TSEN analog (ANAU_TSEN_PU).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_power_down(TSEN_TypeDef *tsen)
{
    CLEAR_BIT(tsen->TSEN_CTRL_REG, TSEN_TSEN_CTRL_REG_ANAU_TSEN_PU);
}

/*==============================================================================
 * Read Data
 *============================================================================*/

/**
 * @brief Read the 12-bit temperature data (TSEN_RDATA).
 * @param[in] tsen TSEN instance pointer.
 * @return Raw ADC value (12 bits).
 */
static inline uint16_t ll_tsen_read_data(TSEN_TypeDef *tsen)
{
    return (uint16_t)(READ_REG(tsen->TSEN_RDATA) & TSEN_TSEN_RDATA_TSEN_RDATA);
}

/**
 * @brief Get the TSEN masked interrupt status (TSEN_ISR).
 * @param[in] tsen TSEN instance pointer.
 * @return Non-zero when the (unmasked) interrupt is pending.
 */
static inline uint32_t ll_tsen_get_interrupt_status(TSEN_TypeDef *tsen)
{
    return READ_BIT(tsen->TSEN_IRQ, TSEN_TSEN_IRQ_TSEN_ISR) ? 1UL : 0UL;
}

/**
 * @brief Get the TSEN raw interrupt flag (TSEN_IRSR).
 * @param[in] tsen TSEN instance pointer.
 * @return Non-zero when a conversion has completed.
 */
static inline uint32_t ll_tsen_get_interrupt_raw(TSEN_TypeDef *tsen)
{
    return READ_BIT(tsen->TSEN_IRQ, TSEN_TSEN_IRQ_TSEN_IRSR) ? 1UL : 0UL;
}

/*==============================================================================
 * Interrupt
 *============================================================================*/

/**
 * @brief Enable the TSEN interrupt (TSEN_IMR = 0).
 * @note IMR is an inverted mask: 0 enables the interrupt, 1 masks it
 *       (see reference manual 8.2.3.2).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_enable_interrupt(TSEN_TypeDef *tsen)
{
    CLEAR_BIT(tsen->TSEN_IRQ, TSEN_TSEN_IRQ_TSEN_IMR);
}

/**
 * @brief Disable the TSEN interrupt (TSEN_IMR = 1).
 * @note IMR is an inverted mask: 0 enables the interrupt, 1 masks it
 *       (see reference manual 8.2.3.2).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_disable_interrupt(TSEN_TypeDef *tsen)
{
    SET_BIT(tsen->TSEN_IRQ, TSEN_TSEN_IRQ_TSEN_IMR);
}

/**
 * @brief Clear the TSEN interrupt (TSEN_ICR, write 1 to clear).
 * @param[in] tsen TSEN instance pointer.
 */
static inline void ll_tsen_clear_interrupt(TSEN_TypeDef *tsen)
{
    /* ICR is rw1c: SET_BIT preserves the IMR mask bit in the same register. */
    SET_BIT(tsen->TSEN_IRQ, TSEN_TSEN_IRQ_TSEN_ICR);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_TSEN_H */
