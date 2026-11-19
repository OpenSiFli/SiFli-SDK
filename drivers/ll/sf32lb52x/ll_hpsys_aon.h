/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_HPSYS_AON_H
#define __LL_HPSYS_AON_H

#include <stddef.h>
#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_hpsys_aon.h
 * @brief Header-only low-level HPSYS_AON APIs for SF32LB52x.
 */

/*==============================================================================
 * Power Mode Control (PMR)
 *============================================================================*/

/** @defgroup LL_AON_PM LL AON Power Mode */
/** @{ */
#define LL_AON_PM_ACTIVE  (0U) /**< Active mode */
#define LL_AON_PM_LIGHT   (1U) /**< Light sleep mode */
#define LL_AON_PM_DEEP    (2U) /**< Deep sleep mode */
#define LL_AON_PM_STANDBY (3U) /**< Standby mode */

/**
 * @brief Force entering low power mode (PMR.FORCE_SLEEP, auto-cleared).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_pm_force_sleep_set(HPSYS_AON_TypeDef *AONx)
{
    SET_BIT(AONx->PMR, HPSYS_AON_PMR_FORCE_SLEEP);
}

/**
 * @brief Clear the force-sleep request (PMR.FORCE_SLEEP).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_pm_force_sleep_clear(HPSYS_AON_TypeDef *AONx)
{
    CLEAR_BIT(AONx->PMR, HPSYS_AON_PMR_FORCE_SLEEP);
}

/**
 * @brief Force the LCPU into sleep (PMR.FORCE_LCPU, debug only).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_pm_force_lcpu_set(HPSYS_AON_TypeDef *AONx)
{
    SET_BIT(AONx->PMR, HPSYS_AON_PMR_FORCE_LCPU);
}

/**
 * @brief Clear the force-LCPU request (PMR.FORCE_LCPU).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_pm_force_lcpu_clear(HPSYS_AON_TypeDef *AONx)
{
    CLEAR_BIT(AONx->PMR, HPSYS_AON_PMR_FORCE_LCPU);
}

/** @} */

/**
 * @brief Set the power mode (PMR.MODE).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_pm_set_mode(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    MODIFY_REG(AONx->PMR, HPSYS_AON_PMR_MODE,
               MAKE_REG_VAL(mode, HPSYS_AON_PMR_MODE_Msk, HPSYS_AON_PMR_MODE_Pos));
}

/**
 * @brief Get the power mode (PMR.MODE).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Current power mode, see @ref LL_AON_PM_*.
 */
static inline uint32_t ll_aon_pm_get_mode(HPSYS_AON_TypeDef *AONx)
{
    return GET_REG_VAL2(AONx->PMR, HPSYS_AON_PMR_MODE);
}

/*==============================================================================
 * Wakeup Pin Mode (CR1 / CR2 / CR3)
 *============================================================================*/

/** @defgroup LL_AON_PIN_MODE LL AON Wakeup Pin Mode */
/** @{ */
#define LL_AON_PIN_MODE_HIGH_LEVEL            0x0U /**< High level */
#define LL_AON_PIN_MODE_LOW_LEVEL             0x1U /**< Low level */
#define LL_AON_PIN_MODE_POS_EDGE              0x2U /**< Rising edge */
#define LL_AON_PIN_MODE_NEG_EDGE              0x3U /**< Falling edge */
#define LL_AON_PIN_MODE_BOTH_EDGE_HIGH_ACTIVE 0x4U /**< Either edge, high-active */
#define LL_AON_PIN_MODE_BOTH_EDGE_LOW_ACTIVE  0x5U /**< Either edge, low-active */
/** @} */

/**
 * @brief Get the PINx_MODE register for a wakeup pin index.
 * @param AONx HPSYS_AON instance pointer.
 * @param pin  Wakeup pin index (0..20, PA24..PA44).
 * @return Pointer to the PINx_MODE register, or NULL for an invalid pin.
 */
static inline volatile uint32_t *ll_aon_cr_reg(HPSYS_AON_TypeDef *AONx, uint32_t pin)
{
    if (pin <= 7U)
    {
        return &AONx->CR1;
    }
    else if (pin <= 15U)
    {
        return &AONx->CR2;
    }
    else if (pin <= 20U)
    {
        return &AONx->CR3;
    }

    return NULL;
}

/**
 * @brief Set the wakeup mode of a wakeup pin (CRx.PINx_MODE).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] pin  Wakeup pin index (0..20, PA24..PA44).
 * @param[in] mode Wakeup mode, see @ref LL_AON_PIN_MODE_*.
 */
static inline void ll_aon_cfg_set_pin_mode(HPSYS_AON_TypeDef *AONx, uint32_t pin, uint32_t mode)
{
    volatile uint32_t *reg = ll_aon_cr_reg(AONx, pin);
    uint32_t pos;

    if (reg == NULL)
    {
        return;
    }

    pos = (pin % 8U) * 3U;
    MODIFY_REG(*reg, 0x7UL << pos, (mode & 0x7UL) << pos);
}

/*==============================================================================
 * PINOUT Select & Global Timer Enable (CR1)
 *============================================================================*/

/** @defgroup LL_AON_PINOUT LL AON PINOUT Select */
/** @{ */
#define LL_AON_PINOUT_DISABLED (0x0U) /**< No output routed to PBR */
#define LL_AON_PINOUT_LPTIM1   (0x2U) /**< LPTIM1 PWM output (SEL1: inverted) */
#define LL_AON_PINOUT_LPTIM2   (0x3U) /**< LPTIM2 PWM output (SEL1: inverted) */

/**
 * @brief Enable the global timer (CR1.GTIM_EN).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_cfg_gtim_enable(HPSYS_AON_TypeDef *AONx)
{
    SET_BIT(AONx->CR1, HPSYS_AON_CR1_GTIM_EN);
}

/**
 * @brief Disable the global timer (CR1.GTIM_EN).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_cfg_gtim_disable(HPSYS_AON_TypeDef *AONx)
{
    CLEAR_BIT(AONx->CR1, HPSYS_AON_CR1_GTIM_EN);
}

/**
 * @brief Select the inverted output routed to PBR (CR1.PINOUT_SEL1).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] sel  Selection, see @ref LL_AON_PINOUT_*.
 */
static inline void ll_aon_cfg_set_pinout_sel1(HPSYS_AON_TypeDef *AONx, uint32_t sel)
{
    MODIFY_REG(AONx->CR1, HPSYS_AON_CR1_PINOUT_SEL1,
               MAKE_REG_VAL(sel, HPSYS_AON_CR1_PINOUT_SEL1_Msk, HPSYS_AON_CR1_PINOUT_SEL1_Pos));
}

/** @} */

/**
 * @brief Select the output routed to PBR (CR1.PINOUT_SEL0).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] sel  Selection, see @ref LL_AON_PINOUT_*.
 */
static inline void ll_aon_cfg_set_pinout_sel0(HPSYS_AON_TypeDef *AONx, uint32_t sel)
{
    MODIFY_REG(AONx->CR1, HPSYS_AON_CR1_PINOUT_SEL0,
               MAKE_REG_VAL(sel, HPSYS_AON_CR1_PINOUT_SEL0_Msk, HPSYS_AON_CR1_PINOUT_SEL0_Pos));
}

/*==============================================================================
 * Clock / Power Request Control (ACR / LSCR / DSCR / SBCR)
 *============================================================================*/

/**
 * @brief Get the request register for a power mode.
 * @note HXT48_REQ/HRC48_REQ/PWR_REQ/EXTPWR_REQ occupy the same bit positions in
 *       ACR/LSCR/DSCR/SBCR, so the ACR bit definitions are reused.
 * @param AONx HPSYS_AON instance pointer.
 * @param mode Power mode, see @ref LL_AON_PM_*.
 * @return Pointer to the request register.
 */
static inline volatile uint32_t *ll_aon_req_reg(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    switch (mode)
    {
    case LL_AON_PM_LIGHT:
        return &AONx->LSCR;
    case LL_AON_PM_DEEP:
        return &AONx->DSCR;
    case LL_AON_PM_STANDBY:
        return &AONx->SBCR;
    case LL_AON_PM_ACTIVE:
    default:
        return &AONx->ACR;
    }
}

/**
 * @brief Check whether HXT48 is ready (ACR.HXT48_RDY).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Non-zero when HXT48 is ready, 0 otherwise.
 */
static inline uint32_t ll_aon_hxt48_is_ready(HPSYS_AON_TypeDef *AONx)
{
    return READ_BIT(AONx->ACR, HPSYS_AON_ACR_HXT48_RDY) != 0UL;
}

/**
 * @brief Check whether HRC48 is ready (ACR.HRC48_RDY).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Non-zero when HRC48 is ready, 0 otherwise.
 */
static inline uint32_t ll_aon_hrc48_is_ready(HPSYS_AON_TypeDef *AONx)
{
    return READ_BIT(AONx->ACR, HPSYS_AON_ACR_HRC48_RDY) != 0UL;
}

/**
 * @brief Set the external power request in the given power mode (EXTPWR_REQ, debug only).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_extpwr_req_set(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    SET_BIT(*ll_aon_req_reg(AONx, mode), HPSYS_AON_ACR_EXTPWR_REQ);
}

/**
 * @brief Clear the external power request in the given power mode (EXTPWR_REQ, debug only).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_extpwr_req_clear(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    CLEAR_BIT(*ll_aon_req_reg(AONx, mode), HPSYS_AON_ACR_EXTPWR_REQ);
}

/**
 * @brief Request power in the given power mode (PWR_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_pwr_req_set(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    SET_BIT(*ll_aon_req_reg(AONx, mode), HPSYS_AON_ACR_PWR_REQ);
}

/**
 * @brief Release the power request in the given power mode (PWR_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_pwr_req_clear(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    CLEAR_BIT(*ll_aon_req_reg(AONx, mode), HPSYS_AON_ACR_PWR_REQ);
}

/**
 * @brief Request HXT48 in the given power mode (HXT48_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_hxt48_req_set(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    SET_BIT(*ll_aon_req_reg(AONx, mode), HPSYS_AON_ACR_HXT48_REQ);
}

/**
 * @brief Release the HXT48 request in the given power mode (HXT48_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_hxt48_req_clear(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    CLEAR_BIT(*ll_aon_req_reg(AONx, mode), HPSYS_AON_ACR_HXT48_REQ);
}

/**
 * @brief Request HRC48 in the given power mode (HRC48_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_hrc48_req_set(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    SET_BIT(*ll_aon_req_reg(AONx, mode), HPSYS_AON_ACR_HRC48_REQ);
}

/**
 * @brief Release the HRC48 request in the given power mode (HRC48_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mode Power mode, see @ref LL_AON_PM_*.
 */
static inline void ll_aon_hrc48_req_clear(HPSYS_AON_TypeDef *AONx, uint32_t mode)
{
    CLEAR_BIT(*ll_aon_req_reg(AONx, mode), HPSYS_AON_ACR_HRC48_REQ);
}

/*==============================================================================
 * RAM Power Down (SBCR)
 *============================================================================*/

/** @defgroup LL_AON_RAM_PD LL AON RAM Power-Down Select */
/** @{ */
#define LL_AON_RAM_PD_0 HPSYS_AON_SBCR_PD_RAM0 /**< Power down RAM0 */
#define LL_AON_RAM_PD_1 HPSYS_AON_SBCR_PD_RAM1 /**< Power down RAM1 */
#define LL_AON_RAM_PD_2 HPSYS_AON_SBCR_PD_RAM2

/**< Power down RAM2 */
/** @} */

/**
 * @brief Power down the selected RAMs in standby (SBCR.PD_RAMx, debug only).
 * @param[in] AONx    HPSYS_AON instance pointer.
 * @param[in] ram_mask Bitwise OR of @ref LL_AON_RAM_PD_* values.
 */
static inline void ll_aon_ram_pd_set(HPSYS_AON_TypeDef *AONx, uint32_t ram_mask)
{
    SET_BIT(AONx->SBCR, ram_mask);
}

/**
 * @brief Keep the selected RAMs powered in standby (SBCR.PD_RAMx, debug only).
 * @param[in] AONx    HPSYS_AON instance pointer.
 * @param[in] ram_mask Bitwise OR of @ref LL_AON_RAM_PD_* values.
 */
static inline void ll_aon_ram_pd_clear(HPSYS_AON_TypeDef *AONx, uint32_t ram_mask)
{
    CLEAR_BIT(AONx->SBCR, ram_mask);
}

/*==============================================================================
 * Wakeup Control (WER / WSR / WCR)
 *============================================================================*/

/** @defgroup LL_AON_WKUP LL AON Wakeup Source */
/** @{ */
#define LL_AON_WKUP_RTC       HPSYS_AON_WER_RTC       /**< RTC */
#define LL_AON_WKUP_GPIO1     HPSYS_AON_WER_GPIO1     /**< IO(PA) */
#define LL_AON_WKUP_LPTIM1    HPSYS_AON_WER_LPTIM1    /**< LPTIM1 */
#define LL_AON_WKUP_PMUC      HPSYS_AON_WER_PMUC      /**< PMUC */
#define LL_AON_WKUP_LP2HP_REQ HPSYS_AON_WER_LP2HP_REQ /**< LPSYS request */
#define LL_AON_WKUP_LP2HP_IRQ HPSYS_AON_WER_LP2HP_IRQ /**< MAILBOX2 */
#define LL_AON_WKUP_PIN0      HPSYS_AON_WER_PIN0
#define LL_AON_WKUP_PIN1      HPSYS_AON_WER_PIN1
#define LL_AON_WKUP_PIN2      HPSYS_AON_WER_PIN2
#define LL_AON_WKUP_PIN3      HPSYS_AON_WER_PIN3
#define LL_AON_WKUP_PIN4      HPSYS_AON_WER_PIN4
#define LL_AON_WKUP_PIN5      HPSYS_AON_WER_PIN5
#define LL_AON_WKUP_PIN6      HPSYS_AON_WER_PIN6
#define LL_AON_WKUP_PIN7      HPSYS_AON_WER_PIN7
#define LL_AON_WKUP_PIN8      HPSYS_AON_WER_PIN8
#define LL_AON_WKUP_PIN9      HPSYS_AON_WER_PIN9
#define LL_AON_WKUP_PIN10     HPSYS_AON_WER_PIN10
#define LL_AON_WKUP_PIN11     HPSYS_AON_WER_PIN11
#define LL_AON_WKUP_PIN12     HPSYS_AON_WER_PIN12
#define LL_AON_WKUP_PIN13     HPSYS_AON_WER_PIN13
#define LL_AON_WKUP_PIN14     HPSYS_AON_WER_PIN14
#define LL_AON_WKUP_PIN15     HPSYS_AON_WER_PIN15
#define LL_AON_WKUP_PIN16     HPSYS_AON_WER_PIN16
#define LL_AON_WKUP_PIN17     HPSYS_AON_WER_PIN17
#define LL_AON_WKUP_PIN18     HPSYS_AON_WER_PIN18
#define LL_AON_WKUP_PIN19     HPSYS_AON_WER_PIN19
#define LL_AON_WKUP_PIN20     HPSYS_AON_WER_PIN20
#define LL_AON_WKUP_PIN_ALL   HPSYS_AON_WSR_PIN_ALL  /**< All wakeup pins */
#define LL_AON_WKUP_AON       HPSYS_AON_WCR_AON

/**< AON wakeup IRQ (clear only) */
/** @} */

/**
 * @brief Enable the selected wakeup sources (WER).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mask Bitwise OR of @ref LL_AON_WKUP_* values.
 */
static inline void ll_aon_wakeup_enable(HPSYS_AON_TypeDef *AONx, uint32_t mask)
{
    SET_BIT(AONx->WER, mask);
}

/**
 * @brief Disable the selected wakeup sources (WER).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mask Bitwise OR of @ref LL_AON_WKUP_* values.
 */
static inline void ll_aon_wakeup_disable(HPSYS_AON_TypeDef *AONx, uint32_t mask)
{
    CLEAR_BIT(AONx->WER, mask);
}

/**
 * @brief Read the full wakeup status (WSR).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Wakeup status word (masked by WER).
 */
static inline uint32_t ll_aon_get_wakeup_status(HPSYS_AON_TypeDef *AONx)
{
    return READ_REG(AONx->WSR);
}

/**
 * @brief Check whether any of the selected wakeup sources is pending (WSR).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mask Bitwise OR of @ref LL_AON_WKUP_* values.
 * @return Non-zero when any selected source is pending, 0 otherwise.
 */
static inline uint32_t ll_aon_is_wakeup_pending(HPSYS_AON_TypeDef *AONx, uint32_t mask)
{
    return READ_BIT(AONx->WSR, mask) != 0UL;
}

/**
 * @brief Clear the selected wakeup flags (WCR, write-1-to-clear).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] mask Bitwise OR of @ref LL_AON_WKUP_* values or LL_AON_WKUP_AON.
 */
static inline void ll_aon_clear_wakeup_flag(HPSYS_AON_TypeDef *AONx, uint32_t mask)
{
    WRITE_REG(AONx->WCR, mask);
}

/**
 * @brief Check whether LPSYS is active (ISSR.LP_ACTIVE).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Non-zero when LPSYS is active, 0 otherwise.
 */
static inline uint32_t ll_aon_issr_is_lp_active(HPSYS_AON_TypeDef *AONx)
{
    return READ_BIT(AONx->ISSR, HPSYS_AON_ISSR_LP_ACTIVE) != 0UL;
}

/**
 * @brief Indicate that HPSYS is active (ISSR.HP_ACTIVE).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_issr_hp_active_set(HPSYS_AON_TypeDef *AONx)
{
    SET_BIT(AONx->ISSR, HPSYS_AON_ISSR_HP_ACTIVE);
}

/**
 * @brief Clear the HPSYS-active indication (ISSR.HP_ACTIVE).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_issr_hp_active_clear(HPSYS_AON_TypeDef *AONx)
{
    CLEAR_BIT(AONx->ISSR, HPSYS_AON_ISSR_HP_ACTIVE);
}

/**
 * @brief Check whether an LPSYS request exists (ISSR.LP2HP_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Non-zero when the request exists, 0 otherwise.
 */
static inline uint32_t ll_aon_issr_is_lp2hp_req(HPSYS_AON_TypeDef *AONx)
{
    return READ_BIT(AONx->ISSR, HPSYS_AON_ISSR_LP2HP_REQ) != 0UL;
}

/*==============================================================================
 * Inter System Status (ISSR)
 *============================================================================*/

/**
 * @brief Request LPSYS to stay in active mode (ISSR.HP2LP_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_issr_hp2lp_req_set(HPSYS_AON_TypeDef *AONx)
{
    SET_BIT(AONx->ISSR, HPSYS_AON_ISSR_HP2LP_REQ);
}

/**
 * @brief Release the HP2LP active request (ISSR.HP2LP_REQ).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_issr_hp2lp_req_clear(HPSYS_AON_TypeDef *AONx)
{
    CLEAR_BIT(AONx->ISSR, HPSYS_AON_ISSR_HP2LP_REQ);
}

/**
 * @brief Force off all HPSYS related analog modules (ANACR.VHP_ISO).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_anacr_vhp_iso_set(HPSYS_AON_TypeDef *AONx)
{
    SET_BIT(AONx->ANACR, HPSYS_AON_ANACR_VHP_ISO);
}

/**
 * @brief Clear the VHP isolation (ANACR.VHP_ISO).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_anacr_vhp_iso_clear(HPSYS_AON_TypeDef *AONx)
{
    CLEAR_BIT(AONx->ANACR, HPSYS_AON_ANACR_VHP_ISO);
}

/*==============================================================================
 * Analog Control (ANACR)
 *============================================================================*/

/**
 * @brief Force IO(PA) into retention mode (ANACR.PA_ISO).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_anacr_pa_iso_set(HPSYS_AON_TypeDef *AONx)
{
    SET_BIT(AONx->ANACR, HPSYS_AON_ANACR_PA_ISO);
}

/**
 * @brief Release IO(PA) retention (ANACR.PA_ISO).
 * @param[in] AONx HPSYS_AON instance pointer.
 */
static inline void ll_aon_anacr_pa_iso_clear(HPSYS_AON_TypeDef *AONx)
{
    CLEAR_BIT(AONx->ANACR, HPSYS_AON_ANACR_PA_ISO);
}

/*==============================================================================
 * Global Timer (GTIMR)
 *============================================================================*/

/**
 * @brief Read the global timer value (GTIMR.CNT).
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Current global timer count.
 */
static inline uint32_t ll_aon_gtim_get_count(HPSYS_AON_TypeDef *AONx)
{
    return READ_REG(AONx->GTIMR);
}

/**
 * @brief Set the AON reserve register 0.
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] value Reserve register value.
 */
static inline void ll_aon_set_reserve0(HPSYS_AON_TypeDef *AONx, uint32_t value)
{
    WRITE_REG(AONx->RESERVE0, value);
}

/**
 * @brief Read the AON reserve register 0.
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Reserve register value.
 */
static inline uint32_t ll_aon_get_reserve0(HPSYS_AON_TypeDef *AONx)
{
    return READ_REG(AONx->RESERVE0);
}

/**
 * @brief Set the AON reserve register 1.
 * @param[in] AONx HPSYS_AON instance pointer.
 * @param[in] value Reserve register value.
 */
static inline void ll_aon_set_reserve1(HPSYS_AON_TypeDef *AONx, uint32_t value)
{
    WRITE_REG(AONx->RESERVE1, value);
}

/**
 * @brief Read the AON reserve register 1.
 * @param[in] AONx HPSYS_AON instance pointer.
 * @return Reserve register value.
 */
static inline uint32_t ll_aon_get_reserve1(HPSYS_AON_TypeDef *AONx)
{
    return READ_REG(AONx->RESERVE1);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_HPSYS_AON_H */
