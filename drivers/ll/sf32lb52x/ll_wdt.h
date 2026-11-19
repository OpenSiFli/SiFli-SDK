/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_WDT_H
#define __LL_WDT_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_wdt.h
 * @brief Header-only low-level Watchdog Timer (WDT) APIs for SF32LB52x.
 *
 * WDT features a 24-bit downcounter with configurable timeout and reset/interrupt
 * response modes.
 */

/** @defgroup LL_WDT_RESPONSE WDT Response Mode */
/** @{ */
/** Reset only: single-stage count, reset at timeout (response_mode = 0). */
#define LL_WDT_RESPONSE_RESET     (0x0UL)
/** Interrupt then reset: two-stage count, interrupt at 1st timeout, reset at 2nd (response_mode = 1). */
#define LL_WDT_RESPONSE_INTERRUPT (0x1UL)
/** @} */

/** @defgroup LL_WDT_RESET_LENGTH WDT Reset Pulse Length */
/** @{ */
/*
 * NOTE: the reference manual only states that reset_length[2:0] selects the
 * reset pulse length in WDT clock cycles, without an explicit 0..7 mapping.
 * The cycle values below are from the chip team's reference and should be
 * re-verified before production use.
 */
#define LL_WDT_RESET_LEN_128_CYCLES   (0x0UL)
#define LL_WDT_RESET_LEN_256_CYCLES   (0x1UL)
#define LL_WDT_RESET_LEN_512_CYCLES   (0x2UL)
#define LL_WDT_RESET_LEN_1024_CYCLES  (0x3UL)
#define LL_WDT_RESET_LEN_2048_CYCLES  (0x4UL)
#define LL_WDT_RESET_LEN_4096_CYCLES  (0x5UL)
#define LL_WDT_RESET_LEN_8192_CYCLES  (0x6UL)
#define LL_WDT_RESET_LEN_16384_CYCLES (0x7UL)
/** @} */

/** @defgroup LL_WDT_CCR_MAGIC WDT Counter Control Magic Values */
/** @{ */
/** Write 0x76 to WDT_CCR to (re)start / feed the watchdog counter. */
#define LL_WDT_CCR_FEED_VALUE (0x76UL)
/** Write 0x34 to WDT_CCR to stop the watchdog counter. */
#define LL_WDT_CCR_STOP_VALUE (0x34UL)
/** @} */

/** @defgroup LL_WDT_WP_MAGIC WDT Write-Protect Magic Values */
/** @{ */
/** Write to WDT_WP to enable write protection (wrpt_st becomes 1). */
#define LL_WDT_WP_LOCK_VALUE   (0x58AB99FCUL)
/** Write to WDT_WP to release write protection (see reference manual 9.5.2.2). */
#define LL_WDT_WP_UNLOCK_VALUE (0x51FF8621UL)
/*==============================================================================
 * Timeout Configuration
 *============================================================================*/

/**
 * @brief Set the 1st-stage timeout count value (WDT_CVR0.count_value_0, 24 bits).
 * @param[in] wdt WDT instance pointer.
 * @param[in] cnt Timeout count value (must be <= 0xFFFFFF).
 */
static inline void ll_wdt_set_timeout1(WDT_TypeDef *wdt, uint32_t cnt)
{
    MODIFY_REG(wdt->WDT_CVR0, WDT_WDT_CVR0_COUNT_VALUE_0,
               MAKE_REG_VAL(cnt, WDT_WDT_CVR0_COUNT_VALUE_0_Msk, WDT_WDT_CVR0_COUNT_VALUE_0_Pos));
}

/**
 * @brief Get the 1st-stage timeout count value (WDT_CVR0.count_value_0).
 * @note WDT_CVR0 holds the programmed timeout value; this chip has no live
 *       countdown readout register.
 * @param[in] wdt WDT instance pointer.
 * @return 1st-stage timeout count value.
 */
static inline uint32_t ll_wdt_get_timeout1(WDT_TypeDef *wdt)
{
    return READ_REG(wdt->WDT_CVR0) & WDT_WDT_CVR0_COUNT_VALUE_0;
}

/**
 * @brief Set the 2nd-stage timeout count value (WDT_CVR1.count_value_1, 24 bits).
 * @note Only used in two-stage mode (LL_WDT_RESPONSE_INTERRUPT).
 * @param[in] wdt WDT instance pointer.
 * @param[in] cnt Timeout count value (must be <= 0xFFFFFF).
 */
static inline void ll_wdt_set_timeout2(WDT_TypeDef *wdt, uint32_t cnt)
{
    MODIFY_REG(wdt->WDT_CVR1, WDT_WDT_CVR1_COUNT_VALUE_1,
               MAKE_REG_VAL(cnt, WDT_WDT_CVR1_COUNT_VALUE_1_Msk, WDT_WDT_CVR1_COUNT_VALUE_1_Pos));
}

/**
 * @brief Get the 2nd-stage timeout count value (WDT_CVR1.count_value_1).
 * @param[in] wdt WDT instance pointer.
 * @return 2nd-stage timeout count value.
 */
static inline uint32_t ll_wdt_get_timeout2(WDT_TypeDef *wdt)
{
    return READ_REG(wdt->WDT_CVR1) & WDT_WDT_CVR1_COUNT_VALUE_1;
}

/**
 * @brief Set the WDT response mode (WDT_CR.response_mode).
 * @param[in] wdt  WDT instance pointer.
 * @param[in] mode Use LL_WDT_RESPONSE_xxx.
 */
static inline void ll_wdt_set_response_mode(WDT_TypeDef *wdt, uint32_t mode)
{
    MODIFY_REG(wdt->WDT_CR, WDT_WDT_CR_RESPONSE_MODE,
               MAKE_REG_VAL(mode, WDT_WDT_CR_RESPONSE_MODE_Msk, WDT_WDT_CR_RESPONSE_MODE_Pos));
}

/**
 * @brief Set the reset pulse length (WDT_CR.reset_length[2:0]).
 * @param[in] wdt WDT instance pointer.
 * @param[in] len Reset pulse length, see @ref LL_WDT_RESET_LEN_*.
 */
static inline void ll_wdt_set_reset_length(WDT_TypeDef *wdt, uint32_t len)
{
    MODIFY_REG(wdt->WDT_CR, WDT_WDT_CR_RESET_LENGTH,
               MAKE_REG_VAL(len, WDT_WDT_CR_RESET_LENGTH_Msk, WDT_WDT_CR_RESET_LENGTH_Pos));
}

/** @} */

/*==============================================================================
 * Control
 *============================================================================*/

/**
 * @brief Start watchdog counting (write 0x76 to WDT_CCR).
 * @note The start command is synchronized from pclk to the WDT clock; check
 *       @ref ll_wdt_is_sync_done before relying on it (see manual 9.5.2.2).
 * @param[in] wdt WDT instance pointer.
 */
static inline void ll_wdt_enable(WDT_TypeDef *wdt)
{
    WRITE_REG(wdt->WDT_CCR, LL_WDT_CCR_FEED_VALUE);
}

/**
 * @brief Stop watchdog counting (write 0x34 to WDT_CCR).
 * @param[in] wdt WDT instance pointer.
 */
static inline void ll_wdt_disable(WDT_TypeDef *wdt)
{
    WRITE_REG(wdt->WDT_CCR, LL_WDT_CCR_STOP_VALUE);
}

/*==============================================================================
 * Counter Feed
 *============================================================================*/

/**
 * @brief Feed (restart) the watchdog counter (write 0x76 to WDT_CCR).
 * @note In two-stage mode (LL_WDT_RESPONSE_INTERRUPT), feeding during the 2nd
 *       stage returns the WDT to the 1st stage (see reference manual 9.5.2).
 * @param[in] wdt WDT instance pointer.
 */
static inline void ll_wdt_feed(WDT_TypeDef *wdt)
{
    WRITE_REG(wdt->WDT_CCR, LL_WDT_CCR_FEED_VALUE);
}

/*==============================================================================
 * Interrupt
 *============================================================================*/

/**
 * @brief Clear the WDT interrupt (WDT_ICR.int_clr, write 1 to clear).
 * @note In two-stage mode, clearing the interrupt returns the WDT to the 1st
 *       stage (see reference manual 9.5.2).
 * @param[in] wdt WDT instance pointer.
 */
static inline void ll_wdt_clear_interrupt(WDT_TypeDef *wdt)
{
    WRITE_REG(wdt->WDT_ICR, WDT_WDT_ICR_INT_CLR);
}

/**
 * @brief Check whether the WDT is running (WDT_SR.wdt_active).
 * @param[in] wdt WDT instance pointer.
 * @return Non-zero when the watchdog is running.
 */
static inline uint32_t ll_wdt_is_active(WDT_TypeDef *wdt)
{
    return READ_BIT(wdt->WDT_SR, WDT_WDT_SR_WDT_ACTIVE) ? 1UL : 0UL;
}

/**
 * @brief Get the WDT raw interrupt status (WDT_SR.int_assert).
 * @param[in] wdt WDT instance pointer.
 * @return Non-zero when the interrupt is asserted.
 */
static inline uint32_t ll_wdt_get_interrupt_status(WDT_TypeDef *wdt)
{
    return READ_BIT(wdt->WDT_SR, WDT_WDT_SR_INT_ASSERT) ? 1UL : 0UL;
}

/*==============================================================================
 * Write Protection
 *============================================================================*/

/**
 * @brief Enable write protection (write 0x58AB99FC to WDT_WP).
 * @note After this, WDT_CVR0/CVR1/CR/CCR cannot be modified; reads still work.
 *       Confirm via WDT_WP.wrpt_st.
 * @param[in] wdt WDT instance pointer.
 */
static inline void ll_wdt_lock(WDT_TypeDef *wdt)
{
    WRITE_REG(wdt->WDT_WP, LL_WDT_WP_LOCK_VALUE);
}

/**
 * @brief Disable write protection (write 0x51FF8621 to WDT_WP).
 * @param[in] wdt WDT instance pointer.
 */
static inline void ll_wdt_unlock(WDT_TypeDef *wdt)
{
    WRITE_REG(wdt->WDT_WP, LL_WDT_WP_UNLOCK_VALUE);
}

/**
 * @brief Check the write-protection state (WDT_WP.wrpt_st).
 * @param[in] wdt WDT instance pointer.
 * @return Non-zero when write protection is enabled.
 */
static inline uint32_t ll_wdt_is_write_protected(WDT_TypeDef *wdt)
{
    return READ_BIT(wdt->WDT_WP, WDT_WDT_WP_WRPT_ST) ? 1UL : 0UL;
}

/**
 * @brief Read the write-protection response value (WDT_WP.wrpt).
 * @param[in] wdt WDT instance pointer.
 * @return Write-protection response value.
 */
static inline uint32_t ll_wdt_get_write_protection_value(WDT_TypeDef *wdt)
{
    return GET_REG_VAL2(wdt->WDT_WP, WDT_WDT_WP_WRPT);
}

/**
 * @brief Check whether the last start/stop/clear operation has been synced from
 *        pclk to the WDT clock (WDT_FG.sync_fg).
 * @param[in] wdt WDT instance pointer.
 * @return Non-zero when the operation has taken effect.
 */
static inline uint32_t ll_wdt_is_sync_done(WDT_TypeDef *wdt)
{
    return READ_BIT(wdt->WDT_FG, WDT_WDT_FG_SYNC_FG) ? 1UL : 0UL;
}

/**
 * @brief Clear the WDT sync flag (WDT_FG.sync_fg_clr, write 1 to clear).
 * @param[in] wdt WDT instance pointer.
 */
static inline void ll_wdt_clear_sync_flag(WDT_TypeDef *wdt)
{
    WRITE_REG(wdt->WDT_FG, WDT_WDT_FG_SYNC_FG_CLR);
}

/*==============================================================================
 * Flags
 *============================================================================*/

/**
 * @brief Check whether this WDT has caused a system reset (WDT_FG.rst_fg).
 * @note rst_fg is only meaningful for the IWDT (see reference manual 9.5.2.2).
 * @param[in] wdt WDT instance pointer.
 * @return Non-zero when the WDT has reset the system.
 */
static inline uint32_t ll_wdt_is_reset_source(WDT_TypeDef *wdt)
{
    return READ_BIT(wdt->WDT_FG, WDT_WDT_FG_RST_FG) ? 1UL : 0UL;
}

/**
 * @brief Clear the WDT reset flag (WDT_FG.rst_fg_clr, write 1 to clear).
 * @param[in] wdt WDT instance pointer.
 */
static inline void ll_wdt_clear_reset_flag(WDT_TypeDef *wdt)
{
    WRITE_REG(wdt->WDT_FG, WDT_WDT_FG_RST_FG_CLR);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_WDT_H */
