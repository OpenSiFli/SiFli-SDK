/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_RNG_H
#define __LL_RNG_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Suspend random number generation (CTRL.gen_rand_num_suspend = 1).
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_suspend(TRNG_TypeDef *trng)
{
    SET_BIT(trng->CTRL, TRNG_CTRL_GEN_RAND_NUM_SUSPEND);
}

/**
 * @brief Resume random number generation (CTRL.gen_rand_num_suspend = 0).
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_resume(TRNG_TypeDef *trng)
{
    CLEAR_BIT(trng->CTRL, TRNG_CTRL_GEN_RAND_NUM_SUSPEND);
}

/**
 * @brief Stop and reset random number generation (CTRL.gen_rand_num_stop, rw level).
 * @note After stopping, call @ref ll_rng_start to restart the engine.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_stop(TRNG_TypeDef *trng)
{
    SET_BIT(trng->CTRL, TRNG_CTRL_GEN_RAND_NUM_STOP);
}

/**
 * @brief Stop and reset seed generation (CTRL.gen_seed_stop, rw level).
 * @note After stopping, call @ref ll_rng_start_seed to restart the engine.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_stop_seed(TRNG_TypeDef *trng)
{
    SET_BIT(trng->CTRL, TRNG_CTRL_GEN_SEED_STOP);
}

/*==============================================================================
 * Random Number Generation
 *============================================================================*/

/**
 * @brief Start continuous random number generation (CTRL.gen_rand_num_start,
 *        write-1-to-trigger).
 * @note If no seed is available, the engine generates a seed first. The single
 *       write also releases any pending gen_rand_num_stop/suspend.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_start(TRNG_TypeDef *trng)
{
    WRITE_REG(trng->CTRL, TRNG_CTRL_GEN_RAND_NUM_START);
}

/**
 * @file ll_rng.h
 * @brief Header-only low-level True Random Number Generator (TRNG) APIs for SF32LB52x.
 *
 * TRNG generates 256-bit true random seeds and can produce continuous random
 * number streams. The output consists of eight 32-bit words (RAND_NUM0..7).
 */

/*==============================================================================
 * Seed Generation
 *============================================================================*/

/**
 * @brief Start seed generation (CTRL.gen_seed_start, write-1-to-trigger).
 * @note Must be done before random number generation. The single write also
 *       releases any pending gen_seed_stop and triggers the seed engine.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_start_seed(TRNG_TypeDef *trng)
{
    WRITE_REG(trng->CTRL, TRNG_CTRL_GEN_SEED_START);
}

/**
 * @brief Check whether random data is ready (STAT.rand_num_valid).
 * @param[in] trng TRNG instance pointer.
 * @return Non-zero when new random data is available.
 */
static inline uint32_t ll_rng_is_data_ready(TRNG_TypeDef *trng)
{
    return READ_BIT(trng->STAT, TRNG_STAT_RAND_NUM_VALID) ? 1UL : 0UL;
}

/**
 * @brief Check whether the random number engine is busy (STAT.rand_num_gen_busy).
 * @param[in] trng TRNG instance pointer.
 * @return Non-zero when busy.
 */
static inline uint32_t ll_rng_is_busy(TRNG_TypeDef *trng)
{
    return READ_BIT(trng->STAT, TRNG_STAT_RAND_NUM_GEN_BUSY) ? 1UL : 0UL;
}

/**
 * @brief Check whether the generated seed is valid (STAT.seed_valid).
 * @param[in] trng TRNG instance pointer.
 * @return Non-zero when the seed is valid.
 */
static inline uint32_t ll_rng_is_seed_valid(TRNG_TypeDef *trng)
{
    return READ_BIT(trng->STAT, TRNG_STAT_SEED_VALID) ? 1UL : 0UL;
}

/**
 * @brief Check whether the seed generation engine is busy (STAT.seed_gen_busy).
 * @param[in] trng TRNG instance pointer.
 * @return Non-zero when busy.
 */
static inline uint32_t ll_rng_is_seed_busy(TRNG_TypeDef *trng)
{
    return READ_BIT(trng->STAT, TRNG_STAT_SEED_GEN_BUSY) ? 1UL : 0UL;
}

/**
 * @brief Set the random seed VN corrector check threshold (CFG.reject_threshold).
 * @note A higher threshold allows seed generation to pass more easily but
 *       lowers randomness (see reference manual 13.2.3.1).
 * @param[in] trng TRNG instance pointer.
 * @param[in] th   Threshold value (8 bits).
 */
static inline void ll_rng_set_reject_threshold(TRNG_TypeDef *trng, uint32_t th)
{
    MODIFY_REG(trng->CFG, TRNG_CFG_REJECT_THRESHOLD,
               MAKE_REG_VAL(th, TRNG_CFG_REJECT_THRESHOLD_Msk, TRNG_CFG_REJECT_THRESHOLD_Pos));
}

/**
 * @brief Select internal or external random seed (CFG.use_ext_seed).
 * @param[in] trng   TRNG instance pointer.
 * @param[in] use_ext 1 to use an external seed (written via
 *        @ref ll_rng_write_seed), 0 to use the internally generated seed.
 */
static inline void ll_rng_set_use_ext_seed(TRNG_TypeDef *trng, uint32_t use_ext)
{
    MODIFY_REG(trng->CFG, TRNG_CFG_USE_EXT_SEED,
               MAKE_REG_VAL(use_ext, TRNG_CFG_USE_EXT_SEED_Msk, TRNG_CFG_USE_EXT_SEED_Pos));
}

/*==============================================================================
 * Configuration
 *============================================================================*/

/**
 * @brief Enable/disable auto clock gating (CFG.auto_clock_enable).
 * @param[in] trng   TRNG instance pointer.
 * @param[in] enable 1 to enable auto clock gating, 0 to disable.
 */
static inline void ll_rng_set_auto_clock(TRNG_TypeDef *trng, uint32_t enable)
{
    MODIFY_REG(trng->CFG, TRNG_CFG_AUTO_CLOCK_ENABLE,
               MAKE_REG_VAL(enable, TRNG_CFG_AUTO_CLOCK_ENABLE_Msk, TRNG_CFG_AUTO_CLOCK_ENABLE_Pos));
}

/**
 * @brief Clear the random-number-available raw interrupt flag (IRQ.rand_num_avail,
 *        rw1c: write 1 to clear).
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_clear_interrupt_flag(TRNG_TypeDef *trng)
{
    /* Preserve the _MSK bits and only clear the rw1c flag (write 1 to clear). */
    WRITE_REG(trng->IRQ,
              (READ_REG(trng->IRQ) &
               (TRNG_IRQ_SEED_GEN_DONE_MSK | TRNG_IRQ_RAND_NUM_AVAIL_MSK |
                TRNG_IRQ_PRNG_LOCKUP_MSK)) |
              TRNG_IRQ_RAND_NUM_AVAIL);
}

/**
 * @brief Clear the seed-generation-done raw interrupt flag (rw1c).
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_clear_seed_done_flag(TRNG_TypeDef *trng)
{
    /* Preserve the _MSK bits and only clear the rw1c flag (write 1 to clear). */
    WRITE_REG(trng->IRQ,
              (READ_REG(trng->IRQ) &
               (TRNG_IRQ_SEED_GEN_DONE_MSK | TRNG_IRQ_RAND_NUM_AVAIL_MSK |
                TRNG_IRQ_PRNG_LOCKUP_MSK)) |
              TRNG_IRQ_SEED_GEN_DONE);
}

/**
 * @brief Enable the PRNG lockup interrupt (IRQ.prng_lockup_msk).
 * @note The _MSK bit is a mask: 1 = masked/disabled, 0 = enabled.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_enable_lockup_interrupt(TRNG_TypeDef *trng)
{
    CLEAR_BIT(trng->IRQ, TRNG_IRQ_PRNG_LOCKUP_MSK);
}

/**
 * @brief Disable the PRNG lockup interrupt (IRQ.prng_lockup_msk).
 * @note The _MSK bit is a mask: 1 = masked/disabled.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_disable_lockup_interrupt(TRNG_TypeDef *trng)
{
    SET_BIT(trng->IRQ, TRNG_IRQ_PRNG_LOCKUP_MSK);
}

/**
 * @brief Clear the PRNG lockup raw interrupt flag (rw1c).
 * @note On lockup the reference manual recommends regenerating a new seed.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_clear_lockup_flag(TRNG_TypeDef *trng)
{
    /* Preserve the _MSK bits and only clear the rw1c flag (write 1 to clear). */
    WRITE_REG(trng->IRQ,
              (READ_REG(trng->IRQ) &
               (TRNG_IRQ_SEED_GEN_DONE_MSK | TRNG_IRQ_RAND_NUM_AVAIL_MSK |
                TRNG_IRQ_PRNG_LOCKUP_MSK)) |
              TRNG_IRQ_PRNG_LOCKUP);
}

/*==============================================================================
 * Interrupt
 *============================================================================*/

/**
 * @brief Enable the random-number-available interrupt (IRQ.rand_num_avail_msk).
 * @note The _MSK bit is a mask: 1 = masked/disabled, 0 = enabled. Clearing it
 *       enables the interrupt (reference manual 13.2).
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_enable_interrupt(TRNG_TypeDef *trng)
{
    CLEAR_BIT(trng->IRQ, TRNG_IRQ_RAND_NUM_AVAIL_MSK);
}

/**
 * @brief Disable the random-number-available interrupt (IRQ.rand_num_avail_msk).
 * @note The _MSK bit is a mask: 1 = masked/disabled. Setting it masks the
 *       interrupt (reference manual 13.2).
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_disable_interrupt(TRNG_TypeDef *trng)
{
    SET_BIT(trng->IRQ, TRNG_IRQ_RAND_NUM_AVAIL_MSK);
}

/**
 * @brief Enable the seed-generation-done interrupt (IRQ.seed_gen_done_msk).
 * @note The _MSK bit is a mask: 1 = masked/disabled, 0 = enabled.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_enable_seed_done_interrupt(TRNG_TypeDef *trng)
{
    CLEAR_BIT(trng->IRQ, TRNG_IRQ_SEED_GEN_DONE_MSK);
}

/**
 * @brief Disable the seed-generation-done interrupt (IRQ.seed_gen_done_msk).
 * @note The _MSK bit is a mask: 1 = masked/disabled.
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_disable_seed_done_interrupt(TRNG_TypeDef *trng)
{
    SET_BIT(trng->IRQ, TRNG_IRQ_SEED_GEN_DONE_MSK);
}

/**
 * @brief Get the PRNG lockup raw interrupt flag (IRQ.prng_lockup).
 * @param[in] trng TRNG instance pointer.
 * @return Non-zero when the flag is set.
 */
static inline uint32_t ll_rng_get_lockup_flag(TRNG_TypeDef *trng)
{
    return READ_BIT(trng->IRQ, TRNG_IRQ_PRNG_LOCKUP) ? 1UL : 0UL;
}

/**
 * @brief Get the random-number-available raw interrupt flag (IRQ.rand_num_avail).
 * @param[in] trng TRNG instance pointer.
 * @return Non-zero when the flag is set.
 */
static inline uint32_t ll_rng_get_interrupt_flag(TRNG_TypeDef *trng)
{
    return READ_BIT(trng->IRQ, TRNG_IRQ_RAND_NUM_AVAIL) ? 1UL : 0UL;
}

/**
 * @brief Get the seed-generation-done raw interrupt flag (IRQ.seed_gen_done).
 * @param[in] trng TRNG instance pointer.
 * @return Non-zero when the flag is set.
 */
static inline uint32_t ll_rng_get_seed_done_flag(TRNG_TypeDef *trng)
{
    return READ_BIT(trng->IRQ, TRNG_IRQ_SEED_GEN_DONE) ? 1UL : 0UL;
}

/*==============================================================================
 * Seed Data
 *============================================================================*/

/**
 * @brief Read seed word by index (0..7).
 * @note When CFG.use_ext_seed is set, the seed registers are writable; use
 *       @ref ll_rng_write_seed to update the external seed in use.
 * @param[in] trng TRNG instance pointer.
 * @param[in] idx  Seed word index, 0..7.
 * @return Seed word value, or 0 if idx is out of range.
 */
static inline uint32_t ll_rng_read_seed(TRNG_TypeDef *trng, uint32_t idx)
{
    const volatile uint32_t *seed_regs = &trng->RAND_SEED0;
    return (idx < 8U) ? seed_regs[idx] : 0UL;
}

/**
 * @brief Write an external seed word by index (0..7).
 * @note Only meaningful when CFG.use_ext_seed is set; writing updates the
 *       random seed in use (see reference manual page 333).
 * @param[in] trng TRNG instance pointer.
 * @param[in] idx  Seed word index, 0..7.
 * @param[in] val  Seed word value.
 */
static inline void ll_rng_write_seed(TRNG_TypeDef *trng, uint32_t idx, uint32_t val)
{
    volatile uint32_t *seed_regs = &trng->RAND_SEED0;
    if (idx < 8U)
    {
        seed_regs[idx] = val;
    }
}

/*==============================================================================
 * Read Random Data
 *============================================================================*/

/**
 * @brief Read a 32-bit random word by index (0..7)
 */
static inline uint32_t ll_rng_read_word(TRNG_TypeDef *trng, uint32_t idx)
{
    const volatile uint32_t *rand_regs = &trng->RAND_NUM0;
    return (idx < 8U) ? rand_regs[idx] : 0UL;
}

/**
 * @brief Read all 8 random words (256 bits total)
 */
static inline void ll_rng_read_all(TRNG_TypeDef *trng, uint32_t *buf)
{
    buf[0] = trng->RAND_NUM0;
    buf[1] = trng->RAND_NUM1;
    buf[2] = trng->RAND_NUM2;
    buf[3] = trng->RAND_NUM3;
    buf[4] = trng->RAND_NUM4;
    buf[5] = trng->RAND_NUM5;
    buf[6] = trng->RAND_NUM6;
    buf[7] = trng->RAND_NUM7;
}

/*==============================================================================
 * Calibration
 *============================================================================*/

/**
 * @brief Set the inverter-chain oscillation period count (CAL_CFG.length).
 * @note The count is expressed in pclk cycles; measurement stops when the pclk
 *       counter reaches this threshold (see reference manual 13.2.3.4).
 * @param[in] trng TRNG instance pointer.
 * @param[in] len  Calibration length (16 bits, default 0xff).
 */
static inline void ll_rng_set_calibration_length(TRNG_TypeDef *trng, uint32_t len)
{
    MODIFY_REG(trng->CAL_CFG, TRNG_CAL_CFG_LENGTH,
               MAKE_REG_VAL(len, TRNG_CAL_CFG_LENGTH_Msk, TRNG_CAL_CFG_LENGTH_Pos));
}

/**
 * @brief Check whether calibration is done (CAL_CFG.done).
 * @param[in] trng TRNG instance pointer.
 * @return Non-zero when calibration is done.
 */
static inline uint32_t ll_rng_is_calibration_done(TRNG_TypeDef *trng)
{
    return READ_BIT(trng->CAL_CFG, TRNG_CAL_CFG_DONE) ? 1UL : 0UL;
}

/**
 * @brief Start calibration (CAL_CFG.enable).
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_start_calibration(TRNG_TypeDef *trng)
{
    SET_BIT(trng->CAL_CFG, TRNG_CAL_CFG_ENABLE);
}

/**
 * @brief Stop calibration (CAL_CFG.enable = 0).
 * @param[in] trng TRNG instance pointer.
 */
static inline void ll_rng_stop_calibration(TRNG_TypeDef *trng)
{
    CLEAR_BIT(trng->CAL_CFG, TRNG_CAL_CFG_ENABLE);
}

/**
 * @brief Select the inverter chain used for calibration (CAL_CFG.osc_clk_sel).
 * @param[in] trng TRNG instance pointer.
 * @param[in] sel  Inverter chain select (3 bits).
 */
static inline void ll_rng_set_calibration_osc_clk_sel(TRNG_TypeDef *trng, uint32_t sel)
{
    MODIFY_REG(trng->CAL_CFG, TRNG_CAL_CFG_OSC_CLK_SEL,
               MAKE_REG_VAL(sel, TRNG_CAL_CFG_OSC_CLK_SEL_Msk, TRNG_CAL_CFG_OSC_CLK_SEL_Pos));
}

/**
 * @brief Force the oscillator clock on during calibration (CAL_CFG.osc_clk_force_on).
 * @param[in] trng  TRNG instance pointer.
 * @param[in] force Non-zero to force the oscillator clock on.
 */
static inline void ll_rng_set_calibration_osc_clk_force_on(TRNG_TypeDef *trng,
        uint32_t force)
{
    MODIFY_REG(trng->CAL_CFG, TRNG_CAL_CFG_OSC_CLK_FORCE_ON,
               force ? TRNG_CAL_CFG_OSC_CLK_FORCE_ON : 0UL);
}

/**
 * @brief Get the calibration result (CAL_RESULT: pclk_cnt[15:0], osc_cnt[31:16]).
 * @param[in] trng TRNG instance pointer.
 * @return Raw CAL_RESULT value.
 */
static inline uint32_t ll_rng_get_calibration_result(TRNG_TypeDef *trng)
{
    return READ_REG(trng->CAL_RESULT) &
           (TRNG_CAL_RESULT_OSC_CNT | TRNG_CAL_RESULT_PCLK_CNT);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_RNG_H */
