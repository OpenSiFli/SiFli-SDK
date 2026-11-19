/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_RTC_H
#define __LL_RTC_H

#include <stdint.h>
#include "hpsys_cfg.h"
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the current time register value (raw TR: SS[9:0], SU, ST, MNU,
 *        MNT, HU, HT, PM).
 * @param[in] rtc RTC instance pointer.
 * @return Raw TR value.
 */
static inline uint32_t ll_rtc_get_time(RTC_TypeDef *rtc)
{
    return READ_REG(rtc->TR);
}

/**
 * @brief Get the current time from the HPSYS_CFG shadow register (HPSYS_CFG.RTC_TR).
 * @note Read after @ref ll_rtc_wait_sync unless CR.BYPSHAD is set.
 * @param[in] cfg HPSYS_CFG instance pointer.
 * @return Raw shadow TR value.
 */
static inline uint32_t ll_rtc_get_time_shadow(HPSYS_CFG_TypeDef *cfg)
{
    return READ_REG(cfg->RTC_TR);
}

/*==============================================================================
 * Time & Date
 *============================================================================*/

/**
 * @brief Set time (BCD format)
 * @param hh  Hours (0-23)
 * @param mm  Minutes (0-59)
 * @param ss  Seconds (0-59)
 */
static inline void ll_rtc_set_time(RTC_TypeDef *rtc, uint8_t hh, uint8_t mm, uint8_t ss)
{
    uint32_t tr =
        ((uint32_t)(hh / 10U) << RTC_TR_HT_Pos) | ((uint32_t)(hh % 10U) << RTC_TR_HU_Pos) |
        ((uint32_t)(mm / 10U) << RTC_TR_MNT_Pos) |
        ((uint32_t)(mm % 10U) << RTC_TR_MNU_Pos) | ((uint32_t)(ss / 10U) << RTC_TR_ST_Pos) |
        ((uint32_t)(ss % 10U) << RTC_TR_SU_Pos);
    WRITE_REG(rtc->TR, tr);
}

/**
 * @brief Get the sub-second counter value (TR.SS[9:0]).
 * @param[in] rtc RTC instance pointer.
 * @return Sub-second counter value.
 */
static inline uint32_t ll_rtc_get_subsecond(RTC_TypeDef *rtc)
{
    return GET_REG_VAL2(rtc->TR, RTC_TR_SS);
}

/**
 * @brief Set the sub-second counter value (TR.SS[9:0]).
 * @note Program in init mode.
 * @param[in] rtc    RTC instance pointer.
 * @param[in] subsec Sub-second counter value (must be <= 0x3FF).
 */
static inline void ll_rtc_set_subsecond(RTC_TypeDef *rtc, uint32_t subsec)
{
    MODIFY_REG(rtc->TR, RTC_TR_SS,
               MAKE_REG_VAL(subsec, RTC_TR_SS_Msk, RTC_TR_SS_Pos));
}

/**
 * @brief Get the current date register value (raw DR: DU, DT, MU, MT, WD, YU, YT).
 * @param[in] rtc RTC instance pointer.
 * @return Raw DR value.
 */
static inline uint32_t ll_rtc_get_date(RTC_TypeDef *rtc)
{
    return READ_REG(rtc->DR);
}

/**
 * @brief Get the current date from the HPSYS_CFG shadow register (HPSYS_CFG.RTC_DR).
 * @note Read after @ref ll_rtc_wait_sync unless CR.BYPSHAD is set.
 * @param[in] cfg HPSYS_CFG instance pointer.
 * @return Raw shadow DR value.
 */
static inline uint32_t ll_rtc_get_date_shadow(HPSYS_CFG_TypeDef *cfg)
{
    return READ_REG(cfg->RTC_DR);
}

/**
 * @brief Set the century bit (DR.CB) for years 0-69 of the 20th century.
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_century_set(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->DR, RTC_DR_CB);
}

/**
 * @brief Clear the century bit (DR.CB) for years 70-99 / 21st century.
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_century_clear(RTC_TypeDef *rtc)
{
    CLEAR_BIT(rtc->DR, RTC_DR_CB);
}

/**
 * @brief Set date (BCD format)
 * @param yy  Year (0-99)
 * @param mm  Month (1-12)
 * @param dd  Day (1-31)
 * @param wd  Weekday (1=Monday..7=Sunday)
 */
static inline void ll_rtc_set_date(RTC_TypeDef *rtc, uint8_t yy, uint8_t mm, uint8_t dd, uint8_t wd)
{
    uint32_t dr =
        ((uint32_t)(yy / 10U) << RTC_DR_YT_Pos) | ((uint32_t)(yy % 10U) << RTC_DR_YU_Pos) |
        ((uint32_t)(wd) << RTC_DR_WD_Pos) | ((uint32_t)(mm / 10U) << RTC_DR_MT_Pos) |
        ((uint32_t)(mm % 10U) << RTC_DR_MU_Pos) | ((uint32_t)(dd / 10U) << RTC_DR_DT_Pos) |
        ((uint32_t)(dd % 10U) << RTC_DR_DU_Pos);
    /* Update only the calendar fields; DR.CB (century) is managed separately
     * by ll_rtc_century_set/clear and must not be wiped by a full write. */
    MODIFY_REG(rtc->DR,
               RTC_DR_YT | RTC_DR_YU | RTC_DR_WD | RTC_DR_MT | RTC_DR_MU |
               RTC_DR_DT | RTC_DR_DU,
               dr);
}

/**
 * @brief Configure the remaining RTC control fields.
 * @param[in] rtc   RTC instance pointer.
 * @param[in] value Encoded COE/OSEL/POL/COSEL/BKP/SUB1H/ADD1H/TSIE/
 *                  REFCKON/TSEDGE/WUCKSEL fields.
 */
static inline void ll_rtc_set_control_config(RTC_TypeDef *rtc, uint32_t value)
{
    const uint32_t mask = RTC_CR_COE | RTC_CR_OSEL | RTC_CR_POL |
                          RTC_CR_COSEL | RTC_CR_BKP | RTC_CR_SUB1H |
                          RTC_CR_ADD1H | RTC_CR_TSIE | RTC_CR_REFCKON |
                          RTC_CR_TSEDGE | RTC_CR_WUCKSEL;

    MODIFY_REG(rtc->CR, mask, value & mask);
}

/**
 * @brief Enable the wakeup timer interrupt (CR.WUTIE).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_enable_wakeup_interrupt(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->CR, RTC_CR_WUTIE);
}

/**
 * @brief Disable the wakeup timer interrupt (CR.WUTIE).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_disable_wakeup_interrupt(RTC_TypeDef *rtc)
{
    CLEAR_BIT(rtc->CR, RTC_CR_WUTIE);
}

/**
 * @brief Enable the alarm interrupt (CR.ALRMIE).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_enable_alarm_interrupt(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->CR, RTC_CR_ALRMIE);
}

/**
 * @brief Disable the alarm interrupt (CR.ALRMIE).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_disable_alarm_interrupt(RTC_TypeDef *rtc)
{
    CLEAR_BIT(rtc->CR, RTC_CR_ALRMIE);
}

/**
 * @brief Enable the wakeup timer (CR.WUTE). The event sets ISR.WUTF.
 * @note To get an interrupt, also enable @ref ll_rtc_enable_wakeup_interrupt.
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_enable_wakeup(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->CR, RTC_CR_WUTE);
}

/**
 * @brief Disable the wakeup timer (CR.WUTE).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_disable_wakeup(RTC_TypeDef *rtc)
{
    CLEAR_BIT(rtc->CR, RTC_CR_WUTE);
}

/**
 * @brief Enable the alarm (CR.ALRME). The alarm event sets ISR.ALRMF.
 * @note To get an interrupt, also enable @ref ll_rtc_enable_alarm_interrupt.
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_enable_alarm(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->CR, RTC_CR_ALRME);
}

/**
 * @brief Disable the alarm (CR.ALRME).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_disable_alarm(RTC_TypeDef *rtc)
{
    CLEAR_BIT(rtc->CR, RTC_CR_ALRME);
}

/**
 * @brief Set time in 24h format
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_set_time_format_24h(RTC_TypeDef *rtc)
{
    CLEAR_BIT(rtc->CR, RTC_CR_FMT);
}

/**
 * @brief Set time in 12h format
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_set_time_format_12h(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->CR, RTC_CR_FMT);
}

/**
 * @brief Check whether the shadow-register bypass is enabled (CR.BYPSHAD).
 * @param[in] rtc RTC instance pointer.
 * @return Non-zero when shadow bypass is enabled, 0 otherwise.
 */
static inline uint32_t ll_rtc_is_bypass_shadow(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->CR, RTC_CR_BYPSHAD) != 0UL;
}

/**
 * @file ll_rtc.h
 * @brief Header-only low-level Real-Time Clock (RTC) APIs for SF32LB52x.
 *
 * Standard RTC with calendar, alarm, timestamp, tamper detection, and backup registers.
 */

/*==============================================================================
 * Initialization & Control
 *============================================================================*/

/**
 * @brief Enter RTC initialization mode (no write protection on SF32LB52x).
 * @note TR/DR/PSCLR/ALRMTR/ALRMDR must be programmed in initialization mode
 *       (enter init mode first, configure, then exit init mode).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_enter_init_mode(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->ISR, RTC_ISR_INIT);
    while (!READ_BIT(rtc->ISR, RTC_ISR_INITF))
    {
    }
}

/**
 * @brief Exit RTC initialization mode
 */
static inline void ll_rtc_exit_init_mode(RTC_TypeDef *rtc)
{
    CLEAR_BIT(rtc->ISR, RTC_ISR_INIT);
}

static inline uint32_t ll_rtc_is_initialized(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->ISR, RTC_ISR_INITS) ? 1UL : 0UL;
}

/*==============================================================================
 * Synchronization
 *============================================================================*/

/**
 * @brief Wait for RTC register synchronization (clear and wait for ISR.RSF).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_wait_sync(RTC_TypeDef *rtc)
{
    /* ISR RSF is rw0c: write 0 only to RSF to trigger re-synchronisation,
     * keep the other pending flags (write 1), preserve the rw INIT bit. */
    WRITE_REG(rtc->ISR, ~(RTC_ISR_RSF | RTC_ISR_INIT) |
              (READ_REG(rtc->ISR) & RTC_ISR_INIT));
    while (!READ_BIT(rtc->ISR, RTC_ISR_RSF))
    {
    }
}

static inline uint32_t ll_rtc_get_shift_pending_flag(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->ISR, RTC_ISR_SHPF) ? 1UL : 0UL;
}

/**
 * @brief Get the wakeup timer flag (ISR.WUTF).
 * @param[in] rtc RTC instance pointer.
 * @return Non-zero when the wakeup flag is set.
 */
static inline uint32_t ll_rtc_get_wakeup_flag(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->ISR, RTC_ISR_WUTF) ? 1UL : 0UL;
}

/**
 * @brief Clear the wakeup timer flag (ISR.WUTF, rw0c: write 0 to clear).
 * @note ISR pending flags are rw0c: writing 0 clears, writing 1 keeps. To
 *       avoid dropping a flag that becomes pending between the read and the
 *       write, write 0 only to WUTF and 1 to every other rw0c flag. The rw
 *       INIT bit is read back and preserved (mirrors the HAL clear macro).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_clear_wakeup_flag(RTC_TypeDef *rtc)
{
    WRITE_REG(rtc->ISR, ~(RTC_ISR_WUTF | RTC_ISR_INIT) |
              (READ_REG(rtc->ISR) & RTC_ISR_INIT));
}

static inline uint32_t ll_rtc_is_wakeup_write_allowed(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->ISR, RTC_ISR_WUTWF) ? 1UL : 0UL;
}

/**
 * @brief Get the alarm flag (ISR.ALRMF).
 * @param[in] rtc RTC instance pointer.
 * @return Non-zero when the alarm flag is set.
 */
static inline uint32_t ll_rtc_get_alarm_flag(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->ISR, RTC_ISR_ALRMF) ? 1UL : 0UL;
}

/**
 * @brief Clear the alarm flag (ISR.ALRMF, rw0c: write 0 to clear).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_clear_alarm_flag(RTC_TypeDef *rtc)
{
    /* rw0c: 0 clears the target, 1 keeps the other pending flags (see
     * ll_rtc_clear_wakeup_flag). Preserve the rw INIT bit. */
    WRITE_REG(rtc->ISR, ~(RTC_ISR_ALRMF | RTC_ISR_INIT) |
              (READ_REG(rtc->ISR) & RTC_ISR_INIT));
}

static inline uint32_t ll_rtc_is_alarm_write_allowed(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->ISR, RTC_ISR_ALRMWF) ? 1UL : 0UL;
}

/**
 * @brief Set RTC prescaler for 1Hz clock
 * @note ck_spre = ck_rtc / ((DIVA_INT+1)*(DIVB+1)); for a 32768Hz input use
 *       async_prediv=127 and sync_prediv=255. Program in init mode.
 * @param[in] rtc          RTC instance pointer.
 * @param[in] async_prediv Asynchronous prescaler (8 bits).
 * @param[in] sync_prediv  Synchronous prescaler (10 bits, must be <= 0x3FF).
 */
static inline void ll_rtc_set_prescaler(RTC_TypeDef *rtc, uint8_t async_prediv,
                                        uint16_t sync_prediv)
{
    MODIFY_REG(rtc->PSCLR, RTC_PSCLR_DIVA_INT,
               MAKE_REG_VAL(async_prediv, RTC_PSCLR_DIVA_INT_Msk, RTC_PSCLR_DIVA_INT_Pos));
    MODIFY_REG(rtc->PSCLR, RTC_PSCLR_DIVB,
               MAKE_REG_VAL(sync_prediv, RTC_PSCLR_DIVB_Msk, RTC_PSCLR_DIVB_Pos));
}

/**
 * @brief Set the asynchronous prescaler fraction (PSCLR.DIVA_FRAC[23:10]).
 * @note Used together with @ref ll_rtc_set_prescaler for an exact 1 Hz from
 *       non-32768Hz clocks (e.g. LRC10). Program in init mode.
 * @param[in] rtc  RTC instance pointer.
 * @param[in] frac Fractional divider value (must be <= 0x3FFF).
 */
static inline void ll_rtc_set_prescaler_frac(RTC_TypeDef *rtc, uint32_t frac)
{
    MODIFY_REG(rtc->PSCLR, RTC_PSCLR_DIVA_FRAC,
               MAKE_REG_VAL(frac, RTC_PSCLR_DIVA_FRAC_Msk, RTC_PSCLR_DIVA_FRAC_Pos));
}

/*==============================================================================
 * Wakeup Timer
 *============================================================================*/

/**
 * @brief Set periodic wakeup timer counter (WUTR.WUT, 18 bits).
 * @param[in] rtc RTC instance pointer.
 * @param[in] wut Wakeup timer counter value (must be <= 0x3FFFF).
 */
static inline void ll_rtc_set_wakeup_timer(RTC_TypeDef *rtc, uint32_t wut)
{
    MODIFY_REG(rtc->WUTR, RTC_WUTR_WUT,
               MAKE_REG_VAL(wut, RTC_WUTR_WUT_Msk, RTC_WUTR_WUT_Pos));
}

/**
 * @brief Get the wakeup timer counter value (WUTR.WUT).
 * @param[in] rtc RTC instance pointer.
 * @return Current WUT value.
 */
static inline uint32_t ll_rtc_get_wakeup_timer(RTC_TypeDef *rtc)
{
    return GET_REG_VAL2(rtc->WUTR, RTC_WUTR_WUT);
}

/**
 * @brief Get the alarm time register value (raw ALRMTR).
 * @param[in] rtc RTC instance pointer.
 * @return Raw ALRMTR value.
 */
static inline uint32_t ll_rtc_get_alarm(RTC_TypeDef *rtc)
{
    return READ_REG(rtc->ALRMTR);
}

/*==============================================================================
 * Alarm
 *============================================================================*/

/**
 * @brief Set alarm time (BCD format). The date fields are masked so the alarm
 *        matches the given time every day.
 * @note Program in init mode.
 * @param[in] rtc RTC instance pointer.
 * @param[in] hh  Hours (0-23)
 * @param[in] mm  Minutes (0-59)
 * @param[in] ss  Seconds (0-59)
 */
static inline void ll_rtc_set_alarm(RTC_TypeDef *rtc, uint8_t hh, uint8_t mm, uint8_t ss)
{
    uint32_t tr = ((uint32_t)(hh / 10U) << RTC_ALRMTR_HT_Pos) |
                  ((uint32_t)(hh % 10U) << RTC_ALRMTR_HU_Pos) |
                  ((uint32_t)(mm / 10U) << RTC_ALRMTR_MNT_Pos) |
                  ((uint32_t)(mm % 10U) << RTC_ALRMTR_MNU_Pos) |
                  ((uint32_t)(ss / 10U) << RTC_ALRMTR_ST_Pos) |
                  ((uint32_t)(ss % 10U) << RTC_ALRMTR_SU_Pos);
    WRITE_REG(rtc->ALRMTR, tr);
    /* Mask the date fields (MSK=1: field ignored) so the alarm is time-only */
    MODIFY_REG(rtc->ALRMDR,
               RTC_ALRMDR_MSKD | RTC_ALRMDR_MSKM | RTC_ALRMDR_MSKWD,
               RTC_ALRMDR_MSKD | RTC_ALRMDR_MSKM | RTC_ALRMDR_MSKWD);
}

/**
 * @brief Get the alarm date register value (raw ALRMDR).
 * @param[in] rtc RTC instance pointer.
 * @return Raw ALRMDR value.
 */
static inline uint32_t ll_rtc_get_alarm_date(RTC_TypeDef *rtc)
{
    return READ_REG(rtc->ALRMDR);
}

/**
 * @brief Set which alarm fields are masked (ALRMDR.MSK*, 1 = field ignored).
 * @param[in] rtc  RTC instance pointer.
 * @param[in] mask OR of RTC_ALRMDR_MSKSS/MSKS/MSKMN/MSKH/MSKD/MSKM/MSKWD.
 */
static inline void ll_rtc_set_alarm_mask(RTC_TypeDef *rtc, uint32_t mask)
{
    MODIFY_REG(rtc->ALRMDR,
               RTC_ALRMDR_MSKSS | RTC_ALRMDR_MSKS | RTC_ALRMDR_MSKMN | RTC_ALRMDR_MSKH |
               RTC_ALRMDR_MSKD | RTC_ALRMDR_MSKM | RTC_ALRMDR_MSKWD,
               mask);
}

/**
 * @brief Set the alarm date (ALRMDR, BCD format).
 * @note Mask bits are not touched; use @ref ll_rtc_set_alarm_mask to select
 *       which fields participate in the comparison.
 * @param[in] rtc RTC instance pointer.
 * @param[in] wd  Weekday (1=Monday..7=Sunday).
 * @param[in] mm  Month (1-12)
 * @param[in] dd  Day (1-31)
 */
static inline void ll_rtc_set_alarm_date(RTC_TypeDef *rtc, uint8_t wd, uint8_t mm, uint8_t dd)
{
    uint32_t dr = ((uint32_t)(wd) << RTC_ALRMDR_WD_Pos) |
                  ((uint32_t)(mm / 10U) << RTC_ALRMDR_MT_Pos) |
                  ((uint32_t)(mm % 10U) << RTC_ALRMDR_MU_Pos) |
                  ((uint32_t)(dd / 10U) << RTC_ALRMDR_DT_Pos) |
                  ((uint32_t)(dd % 10U) << RTC_ALRMDR_DU_Pos);
    MODIFY_REG(rtc->ALRMDR,
               RTC_ALRMDR_WD | RTC_ALRMDR_MT | RTC_ALRMDR_MU | RTC_ALRMDR_DT | RTC_ALRMDR_DU,
               dr);
}

/**
 * @brief Enable write protection (lock RTC registers)
 */
static inline void ll_rtc_enable_write_protection(RTC_TypeDef *rtc)
{
    /* LL gap: SF32LB52x RTC has no WPR register — write protection not available */
    (void)rtc;
}

static inline void ll_rtc_disable_write_protection(RTC_TypeDef *rtc)
{
    /* LL gap: SF32LB52x RTC has no WPR register — write protection not available */
    (void)rtc;
}

/*==============================================================================
 * Low-Power Clock Select
 *============================================================================*/

/** @defgroup LL_RTC_LPCLK RTC Low-Power Clock Source */
/** @{ */
#define LL_RTC_LPCLK_LRC10 0U /**< clk_rtc = clk_lrc10 (~10kHz) */
#define LL_RTC_LPCLK_LXT32 1U /**< clk_rtc = clk_lxt32 (32.768kHz) */
/** @} */

/*==============================================================================
 * Timestamp
 *============================================================================*/

/**
 * @brief Enable timestamp capture (CR.TSE).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_enable_timestamp(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->CR, RTC_CR_TSE);
}

/**
 * @brief Disable timestamp capture (CR.TSE).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_disable_timestamp(RTC_TypeDef *rtc)
{
    CLEAR_BIT(rtc->CR, RTC_CR_TSE);
}

/**
 * @brief Select the clk_rtc low-power clock source (CR.LPCKSEL).
 * @param[in] rtc RTC instance pointer.
 * @param[in] src Use LL_RTC_LPCLK_xxx.
 */
static inline void ll_rtc_set_lpclk_source(RTC_TypeDef *rtc, uint32_t src)
{
    MODIFY_REG(rtc->CR, RTC_CR_LPCKSEL,
               MAKE_REG_VAL(src, RTC_CR_LPCKSEL_Msk, RTC_CR_LPCKSEL_Pos));
}

/**
 * @brief Get the clk_rtc low-power clock source (CR.LPCKSEL).
 * @param[in] rtc RTC instance pointer.
 * @return Current LPCKSEL value.
 */
static inline uint32_t ll_rtc_get_lpclk_source(RTC_TypeDef *rtc)
{
    return GET_REG_VAL2(rtc->CR, RTC_CR_LPCKSEL);
}

/**
 * @brief Get the timestamp overflow flag (ISR.TSOVF).
 * @param[in] rtc RTC instance pointer.
 * @return Non-zero when a second timestamp event occurred before the first one
 *         was read (previous timestamp lost).
 */
static inline uint32_t ll_rtc_get_timestamp_overflow_flag(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->ISR, RTC_ISR_TSOVF) ? 1UL : 0UL;
}

/**
 * @brief Get the timestamp flag (ISR.TSF).
 * @param[in] rtc RTC instance pointer.
 * @return Non-zero when a timestamp event occurred.
 */
static inline uint32_t ll_rtc_get_timestamp_flag(RTC_TypeDef *rtc)
{
    return READ_BIT(rtc->ISR, RTC_ISR_TSF) ? 1UL : 0UL;
}

/**
 * @brief Clear the timestamp flag (ISR.TSF, rw0c: write 0 to clear).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_clear_timestamp_flag(RTC_TypeDef *rtc)
{
    /* rw0c: 0 clears the target, 1 keeps the other pending flags (see
     * ll_rtc_clear_wakeup_flag). Preserve the rw INIT bit. */
    WRITE_REG(rtc->ISR, ~(RTC_ISR_TSF | RTC_ISR_INIT) |
              (READ_REG(rtc->ISR) & RTC_ISR_INIT));
}

/**
 * @brief Add one second to the calendar (SHIFTR.ADD1S).
 * @param[in] rtc RTC instance pointer.
 */
static inline void ll_rtc_add_one_second(RTC_TypeDef *rtc)
{
    SET_BIT(rtc->SHIFTR, RTC_SHIFTR_ADD1S);
}

/**
 * @brief Shift the subsecond counter (SHIFTR.SUBFS[9:0]).
 * @param[in] rtc   RTC instance pointer.
 * @param[in] subfs Subsecond shift value (10 bits).
 */
static inline void ll_rtc_shift_subsecond(RTC_TypeDef *rtc, uint32_t subfs)
{
    MODIFY_REG(rtc->SHIFTR, RTC_SHIFTR_SUBFS,
               MAKE_REG_VAL(subfs, RTC_SHIFTR_SUBFS_Msk, RTC_SHIFTR_SUBFS_Pos));
}

/**
 * @brief Get the timestamp time register (raw TSTR).
 * @param[in] rtc RTC instance pointer.
 * @return Raw TSTR value.
 */
static inline uint32_t ll_rtc_get_timestamp_time(RTC_TypeDef *rtc)
{
    return READ_REG(rtc->TSTR);
}

/**
 * @brief Get the timestamp date register (raw TSDR).
 * @param[in] rtc RTC instance pointer.
 * @return Raw TSDR value.
 */
static inline uint32_t ll_rtc_get_timestamp_date(RTC_TypeDef *rtc)
{
    return READ_REG(rtc->TSDR);
}

/**
 * @brief Set the RTC output remap (OR.RTC_OUT_RMP).
 * @param[in] rtc RTC instance pointer.
 * @param[in] rmp Output remap selection.
 */
static inline void ll_rtc_set_output_remap(RTC_TypeDef *rtc, uint32_t rmp)
{
    MODIFY_REG(rtc->OR, RTC_OR_RTC_OUT_RMP,
               MAKE_REG_VAL(rmp, RTC_OR_RTC_OUT_RMP_Msk, RTC_OR_RTC_OUT_RMP_Pos));
}

/**
 * @brief Set the RTC alarm output type (OR.RTC_ALARM_TYPE).
 * @param[in] rtc  RTC instance pointer.
 * @param[in] type Alarm output type.
 */
static inline void ll_rtc_set_alarm_output_type(RTC_TypeDef *rtc, uint32_t type)
{
    MODIFY_REG(rtc->OR, RTC_OR_RTC_ALARM_TYPE,
               MAKE_REG_VAL(type, RTC_OR_RTC_ALARM_TYPE_Msk, RTC_OR_RTC_ALARM_TYPE_Pos));
}

/*==============================================================================
 * Backup Registers
 *============================================================================*/

/**
 * @brief Write a backup register (BKP0R..BKP9R).
 * @param[in] rtc RTC instance pointer.
 * @param[in] idx Backup register index, 0..9.
 * @param[in] val Value to write.
 */
static inline void ll_rtc_write_backup(RTC_TypeDef *rtc, uint32_t idx, uint32_t val)
{
    if (idx < 10U)
    {
        (&rtc->BKP0R)[idx] = val;
    }
}

/**
 * @brief Read a backup register (BKP0R..BKP9R).
 * @param[in] rtc RTC instance pointer.
 * @param[in] idx Backup register index, 0..9.
 * @return Backup register value, or 0 if idx is out of range.
 */
static inline uint32_t ll_rtc_read_backup(RTC_TypeDef *rtc, uint32_t idx)
{
    return (idx < 10U) ? (&rtc->BKP0R)[idx] : 0UL;
}

/**
 * @brief Configure the PBR pad control register (PBRCR).
 * @param[in] rtc     RTC instance pointer.
 * @param[in] rto     RTO bit.
 * @param[in] sns     SNS bit.
 * @param[in] dbg_sel Debug select (4 bits).
 */
static inline void ll_rtc_set_pbr_config(RTC_TypeDef *rtc, uint32_t rto, uint32_t sns,
        uint32_t dbg_sel)
{
    MODIFY_REG(rtc->PBRCR, RTC_PBRCR_RTO | RTC_PBRCR_SNS | RTC_PBRCR_DBG_SEL,
               MAKE_REG_VAL(rto, RTC_PBRCR_RTO_Msk, RTC_PBRCR_RTO_Pos) |
               MAKE_REG_VAL(sns, RTC_PBRCR_SNS_Msk, RTC_PBRCR_SNS_Pos) |
               MAKE_REG_VAL(dbg_sel, RTC_PBRCR_DBG_SEL_Msk, RTC_PBRCR_DBG_SEL_Pos));
}

/**
 * @brief Write a PBR pad control register by index (PBR0R..PBR3R).
 * @param[in] rtc RTC instance pointer.
 * @param[in] idx PBR index, 0..3.
 * @param[in] val Raw register value.
 */
static inline void ll_rtc_write_pbr(RTC_TypeDef *rtc, uint32_t idx, uint32_t val)
{
    if (idx < 4U)
    {
        (&rtc->PBR0R)[idx] = val;
    }
}

/**
 * @brief Read a PBR pad control register by index (PBR0R..PBR3R).
 * @param[in] rtc RTC instance pointer.
 * @param[in] idx PBR index, 0..3.
 * @return Raw register value, or 0 if idx is out of range.
 */
static inline uint32_t ll_rtc_read_pbr(RTC_TypeDef *rtc, uint32_t idx)
{
    return (idx < 4U) ? (&rtc->PBR0R)[idx] : 0UL;
}

/**
 * @brief Set the PA WKUP pull-down enable mask (PAWK1R.PE).
 * @param[in] rtc  RTC instance pointer.
 * @param[in] mask Pin mask (PA00..PA44), 1 = pull-down enabled by default.
 */
static inline void ll_rtc_set_wkup_pull_down(RTC_TypeDef *rtc, uint32_t mask)
{
    WRITE_REG(rtc->PAWK1R, mask);
}

/**
 * @brief Set the PA WKUP pull strength mask (PAWK2R.PS).
 * @param[in] rtc  RTC instance pointer.
 * @param[in] mask Pin mask.
 */
static inline void ll_rtc_set_wkup_pull_strength(RTC_TypeDef *rtc, uint32_t mask)
{
    WRITE_REG(rtc->PAWK2R, mask);
}

/**
 * @brief Set the PA WKUP input mask (PAWK3R.IS).
 * @param[in] rtc  RTC instance pointer.
 * @param[in] mask Pin mask.
 */
static inline void ll_rtc_set_wkup_input(RTC_TypeDef *rtc, uint32_t mask)
{
    WRITE_REG(rtc->PAWK3R, mask);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_RTC_H */
