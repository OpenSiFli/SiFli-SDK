/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_RCC_H
#define __LL_RCC_H

#include <stddef.h>
#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_rcc.h
 * @brief Header-only low-level RCC (Reset & Clock Control) APIs for SF32LB52x.
 */

/*==============================================================================
 * Clock Source Selection Constants
 *============================================================================*/

/** @defgroup LL_RCC_SYS_CLKSRC System Clock Source (CSR.SEL_SYS) */
/** @{ */
#define LL_RCC_SYS_CLKSRC_HRC48 (0x0UL) /**< clk_hrc48  48MHz internal RC */
#define LL_RCC_SYS_CLKSRC_HXT48 (0x1UL) /**< clk_hxt48  48MHz external crystal */
#define LL_RCC_SYS_CLKSRC_DLL1  (0x3UL) /**< clk_dll1   DLL1 output */
/** @} */

/** @defgroup LL_RCC_PERI_CLKSRC Peripheral Clock Source (CSR.SEL_PERI) */
/** @{ */
#define LL_RCC_PERI_CLKSRC_HRC48 (0x0UL)
#define LL_RCC_PERI_CLKSRC_HXT48 (0x1UL)
/** @} */

/** @defgroup LL_RCC_TICK_CLKSRC Systick Clock Source (CSR.SEL_TICK) */
/** @{ */
#define LL_RCC_TICK_CLKSRC_RTC   (0x0UL)
#define LL_RCC_TICK_CLKSRC_HRC48 (0x2UL)
#define LL_RCC_TICK_CLKSRC_HXT48 (0x3UL)
/** @} */

/** @defgroup LL_RCC_USB_CLKSRC USB Clock Source (CSR.SEL_USBC) */
/** @{ */
#define LL_RCC_USB_CLKSRC_HPSYS (0x0UL)
#define LL_RCC_USB_CLKSRC_DLL2  (0x1UL)
/** @} */

/** @defgroup LL_RCC_MPI_CLKSRC MPI Clock Source (CSR.SEL_MPIx) */
/** @{ */
#define LL_RCC_MPI_CLKSRC_PERI (0x0UL)
#define LL_RCC_MPI_CLKSRC_DLL1 (0x1UL)
#define LL_RCC_MPI_CLKSRC_DLL2 (0x2UL)
/** @} */

/** @defgroup LL_RCC_SYS_LP_CLKSRC Low-Power System Clock (CSR.SEL_SYS_LP) */
/** @{ */
#define LL_RCC_SYS_LP_CLKSRC_DEFAULT (0x0UL) /**< Controlled by SEL_SYS */
#define LL_RCC_SYS_LP_CLKSRC_WDT     (0x1UL) /**< clk_wdt */
/** @} */

/*==============================================================================
 * DLL Frequency Stage Constants
 * f_dll_out = (STG+1) * 24MHz when OUT_DIV2_EN=0, or half of that when
 * OUT_DIV2_EN=1. NOTE: OUT_DIV2_EN resets to 1 (output divided by 2); use
 * ll_rcc_dll_set_out_div2 to configure it.
 *============================================================================*/

/** @defgroup LL_RCC_DLL_STG DLL Frequency Stage */
/** @{ */
#define LL_RCC_DLL_STG_48M  (1U)
#define LL_RCC_DLL_STG_72M  (2U)
#define LL_RCC_DLL_STG_96M  (3U)
#define LL_RCC_DLL_STG_120M (4U)
#define LL_RCC_DLL_STG_144M (5U)
#define LL_RCC_DLL_STG_168M (6U)
#define LL_RCC_DLL_STG_192M (7U)
#define LL_RCC_DLL_STG_216M (8U)
#define LL_RCC_DLL_STG_240M (9U)
#define LL_RCC_DLL_STG_264M (10U)
#define LL_RCC_DLL_STG_288M (11U)
#define LL_RCC_DLL_STG_312M (12U)
#define LL_RCC_DLL_STG_336M (13U)
#define LL_RCC_DLL_STG_360M (14U)
#define LL_RCC_DLL_STG_384M (15U)
/** @} */

/*==============================================================================
 * Module Reset Control
 *============================================================================*/

/** @defgroup LL_RCC_GROUP Reset / Clock-Enable Group Index */
/** @{ */
#define LL_RCC_GROUP_1 (0U) /**< RSTR1 / ENR1 / ESR1 / ECR1 */
#define LL_RCC_GROUP_2 (1U) /**< RSTR2 / ENR2 / ESR2 / ECR2 */
/** @} */

/**
 * @brief Get the RSTRx register address for a reset group index.
 * @note Newer SF32LB variants may expose more reset groups; extend the
 *       switch to add them (no driver changes needed).
 * @param rcc   RCC peripheral pointer
 * @param group Reset group index (LL_RCC_GROUP_x)
 * @return Pointer to the RSTRx register, or NULL for an invalid group.
 */
static inline volatile uint32_t *ll_rcc_rstr_reg(HPSYS_RCC_TypeDef *rcc, uint32_t group)
{
    switch (group)
    {
    case LL_RCC_GROUP_1:
        return &rcc->RSTR1;
    case LL_RCC_GROUP_2:
        return &rcc->RSTR2;
    default:
        return NULL;
    }
}

/**
 * @brief Assert reset for modules in a reset group.
 * @param rcc   RCC peripheral pointer
 * @param group Reset group index (LL_RCC_GROUP_x)
 * @param mask  Module mask (use HPSYS_RCC_RSTRx_xxx macros)
 */
static inline void ll_rcc_reset_module(HPSYS_RCC_TypeDef *rcc, uint32_t group, uint32_t mask)
{
    volatile uint32_t *rstr = ll_rcc_rstr_reg(rcc, group);

    if (rstr != NULL)
    {
        SET_BIT(*rstr, mask);
    }
}

/**
 * @brief Release reset for modules in a reset group.
 * @param rcc   RCC peripheral pointer
 * @param group Reset group index (LL_RCC_GROUP_x)
 * @param mask  Module mask (use HPSYS_RCC_RSTRx_xxx macros)
 */
static inline void ll_rcc_release_reset(HPSYS_RCC_TypeDef *rcc, uint32_t group, uint32_t mask)
{
    volatile uint32_t *rstr = ll_rcc_rstr_reg(rcc, group);

    if (rstr != NULL)
    {
        CLEAR_BIT(*rstr, mask);
    }
}

/**
 * @brief Check whether a module in a reset group is currently held in reset.
 * @param rcc   RCC peripheral pointer
 * @param group Reset group index (LL_RCC_GROUP_x)
 * @param mask  Module mask (use HPSYS_RCC_RSTRx_xxx macros)
 * @return Non-zero while the module is asserted in reset, 0 otherwise.
 */
static inline uint32_t ll_rcc_is_module_in_reset(HPSYS_RCC_TypeDef *rcc, uint32_t group,
        uint32_t mask)
{
    volatile uint32_t *rstr = ll_rcc_rstr_reg(rcc, group);

    return (rstr != NULL) ? (READ_BIT(*rstr, mask) != 0UL) : 0UL;
}

/*==============================================================================
 * Module Clock Enable Control
 *============================================================================*/

/**
 * @brief Get the ENRx register address for a clock-enable group index.
 * @note Newer SF32LB variants may expose more clock-enable groups; extend
 *       the switch to add them (no driver changes needed).
 * @param rcc   RCC peripheral pointer
 * @param group Clock-enable group index (LL_RCC_GROUP_x)
 * @return Pointer to the ENRx register, or NULL for an invalid group.
 */
static inline volatile uint32_t *ll_rcc_enr_reg(HPSYS_RCC_TypeDef *rcc, uint32_t group)
{
    switch (group)
    {
    case LL_RCC_GROUP_1:
        return &rcc->ENR1;
    case LL_RCC_GROUP_2:
        return &rcc->ENR2;
    default:
        return NULL;
    }
}

/**
 * @brief Enable module clock in a group (write ENRx).
 * @param rcc   RCC peripheral pointer
 * @param group Clock-enable group index (LL_RCC_GROUP_x)
 * @param mask  Module mask (use HPSYS_RCC_ENRx_xxx macros)
 */
static inline void ll_rcc_enable_clock(HPSYS_RCC_TypeDef *rcc, uint32_t group, uint32_t mask)
{
    volatile uint32_t *enr = ll_rcc_enr_reg(rcc, group);

    if (enr != NULL)
    {
        SET_BIT(*enr, mask);
    }
}

/**
 * @brief Disable module clock in a group (write ENRx).
 */
static inline void ll_rcc_disable_clock(HPSYS_RCC_TypeDef *rcc, uint32_t group, uint32_t mask)
{
    volatile uint32_t *enr = ll_rcc_enr_reg(rcc, group);

    if (enr != NULL)
    {
        CLEAR_BIT(*enr, mask);
    }
}

/**
 * @brief Check whether a module clock in a group is currently enabled (read ENRx).
 * @param rcc   RCC peripheral pointer.
 * @param group Clock-enable group index (LL_RCC_GROUP_x).
 * @param mask  Module mask (use HPSYS_RCC_ENRx_xxx macros).
 * @return Non-zero while the module clock is enabled, 0 otherwise.
 */
static inline uint32_t ll_rcc_is_clock_enabled(HPSYS_RCC_TypeDef *rcc, uint32_t group,
        uint32_t mask)
{
    volatile uint32_t *enr = ll_rcc_enr_reg(rcc, group);

    return (enr != NULL) ? (READ_BIT(*enr, mask) != 0UL) : 0UL;
}

/**
 * @brief Get the ESRx register address for a clock-enable group index.
 * @return Pointer to the ESRx register, or NULL for an invalid group.
 */
static inline volatile uint32_t *ll_rcc_esr_reg(HPSYS_RCC_TypeDef *rcc, uint32_t group)
{
    switch (group)
    {
    case LL_RCC_GROUP_1:
        return &rcc->ESR1;
    case LL_RCC_GROUP_2:
        return &rcc->ESR2;
    default:
        return NULL;
    }
}

/**
 * @brief Atomic set clock enable in a group (via ESRx, write 0 has no effect).
 */
static inline void ll_rcc_set_enable_clock(HPSYS_RCC_TypeDef *rcc, uint32_t group, uint32_t mask)
{
    volatile uint32_t *esr = ll_rcc_esr_reg(rcc, group);

    if (esr != NULL)
    {
        WRITE_REG(*esr, mask);
    }
}

/**
 * @brief Get the ECRx register address for a clock-enable group index.
 * @return Pointer to the ECRx register, or NULL for an invalid group.
 */
static inline volatile uint32_t *ll_rcc_ecr_reg(HPSYS_RCC_TypeDef *rcc, uint32_t group)
{
    switch (group)
    {
    case LL_RCC_GROUP_1:
        return &rcc->ECR1;
    case LL_RCC_GROUP_2:
        return &rcc->ECR2;
    default:
        return NULL;
    }
}

/**
 * @brief Atomic clear clock enable in a group (via ECRx, write 0 has no effect).
 */
static inline void ll_rcc_clear_enable_clock(HPSYS_RCC_TypeDef *rcc, uint32_t group, uint32_t mask)
{
    volatile uint32_t *ecr = ll_rcc_ecr_reg(rcc, group);

    if (ecr != NULL)
    {
        WRITE_REG(*ecr, mask);
    }
}

/**
 * @brief Set USB clock source
 * @param[in] rcc RCC peripheral pointer
 * @param[in] src Use LL_RCC_USB_CLKSRC_xxx
 */
static inline void ll_rcc_set_usb_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t src)
{
    MODIFY_REG(rcc->CSR, HPSYS_RCC_CSR_SEL_USBC,
               MAKE_REG_VAL(src, HPSYS_RCC_CSR_SEL_USBC_Msk, HPSYS_RCC_CSR_SEL_USBC_Pos));
}

/**
 * @brief Get USB clock source
 * @param[in] rcc RCC peripheral pointer
 * @return USB clock source selection value.
 */
static inline uint32_t ll_rcc_get_usb_clock_source(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CSR, HPSYS_RCC_CSR_SEL_USBC);
}

/**
 * @brief Set Systick reference clock source
 * @param[in] rcc RCC peripheral pointer
 * @param[in] src Use LL_RCC_TICK_CLKSRC_xxx
 */
static inline void ll_rcc_set_tick_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t src)
{
    MODIFY_REG(rcc->CSR, HPSYS_RCC_CSR_SEL_TICK,
               MAKE_REG_VAL(src, HPSYS_RCC_CSR_SEL_TICK_Msk, HPSYS_RCC_CSR_SEL_TICK_Pos));
}

/**
 * @brief Get Systick reference clock source
 * @param[in] rcc RCC peripheral pointer
 * @return Systick reference clock source selection value.
 */
static inline uint32_t ll_rcc_get_tick_clock_source(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CSR, HPSYS_RCC_CSR_SEL_TICK);
}

/**
 * @brief Set peripheral clock source clk_peri_hpsys (USART/SPI/I2C/GPTIM2/BTIM2)
 * @param[in] rcc RCC peripheral pointer
 * @param[in] src Use LL_RCC_PERI_CLKSRC_xxx
 */
static inline void ll_rcc_set_peri_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t src)
{
    MODIFY_REG(rcc->CSR, HPSYS_RCC_CSR_SEL_PERI,
               MAKE_REG_VAL(src, HPSYS_RCC_CSR_SEL_PERI_Msk, HPSYS_RCC_CSR_SEL_PERI_Pos));
}

/**
 * @brief Get peripheral clock source clk_peri_hpsys
 * @param[in] rcc RCC peripheral pointer
 * @return Peripheral clock source selection value.
 */
static inline uint32_t ll_rcc_get_peri_clock_source(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CSR, HPSYS_RCC_CSR_SEL_PERI);
}

/**
 * @brief Set MPI2 clock source
 * @param[in] rcc RCC peripheral pointer
 * @param[in] src Use LL_RCC_MPI_CLKSRC_xxx
 */
static inline void ll_rcc_set_mpi2_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t src)
{
    MODIFY_REG(rcc->CSR, HPSYS_RCC_CSR_SEL_MPI2,
               MAKE_REG_VAL(src, HPSYS_RCC_CSR_SEL_MPI2_Msk, HPSYS_RCC_CSR_SEL_MPI2_Pos));
}

/**
 * @brief Get MPI2 clock source
 * @param[in] rcc RCC peripheral pointer
 * @return MPI2 clock source selection value.
 */
static inline uint32_t ll_rcc_get_mpi2_clock_source(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CSR, HPSYS_RCC_CSR_SEL_MPI2);
}

/**
 * @brief Set MPI1 clock source
 * @param[in] rcc RCC peripheral pointer
 * @param[in] src Use LL_RCC_MPI_CLKSRC_xxx
 */
static inline void ll_rcc_set_mpi1_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t src)
{
    MODIFY_REG(rcc->CSR, HPSYS_RCC_CSR_SEL_MPI1,
               MAKE_REG_VAL(src, HPSYS_RCC_CSR_SEL_MPI1_Msk, HPSYS_RCC_CSR_SEL_MPI1_Pos));
}

/**
 * @brief Get MPI1 clock source
 * @param[in] rcc RCC peripheral pointer
 * @return MPI1 clock source selection value.
 */
static inline uint32_t ll_rcc_get_mpi1_clock_source(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CSR, HPSYS_RCC_CSR_SEL_MPI1);
}

/**
 * @brief Set low-power system clock source
 * @param[in] rcc RCC peripheral pointer
 * @param[in] src Use LL_RCC_SYS_LP_CLKSRC_xxx
 */
static inline void ll_rcc_set_sys_lp_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t src)
{
    MODIFY_REG(rcc->CSR, HPSYS_RCC_CSR_SEL_SYS_LP,
               MAKE_REG_VAL(src, HPSYS_RCC_CSR_SEL_SYS_LP_Msk, HPSYS_RCC_CSR_SEL_SYS_LP_Pos));
}

/**
 * @brief Get low-power system clock source
 * @param[in] rcc RCC peripheral pointer
 * @return Low-power system clock source selection value.
 */
static inline uint32_t ll_rcc_get_sys_lp_clock_source(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CSR, HPSYS_RCC_CSR_SEL_SYS_LP);
}

/*==============================================================================
 * Clock Source Selection
 *============================================================================*/

/**
 * @brief Set system clock source clk_hpsys
 * @param[in] rcc RCC peripheral pointer
 * @param[in] src Use LL_RCC_SYS_CLKSRC_xxx
 */
static inline void ll_rcc_set_sys_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t src)
{
    MODIFY_REG(rcc->CSR, HPSYS_RCC_CSR_SEL_SYS,
               MAKE_REG_VAL(src, HPSYS_RCC_CSR_SEL_SYS_Msk, HPSYS_RCC_CSR_SEL_SYS_Pos));
}

static inline uint32_t ll_rcc_get_sys_clock_source(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CSR, HPSYS_RCC_CSR_SEL_SYS);
}

/**
 * @brief Set Systick divider f_systick = f_SEL_TICK / div
 * @param[in] rcc RCC peripheral pointer
 * @param[in] div Divider value.
 */
static inline void ll_rcc_set_tick_div(HPSYS_RCC_TypeDef *rcc, uint32_t div)
{
    MODIFY_REG(rcc->CFGR, HPSYS_RCC_CFGR_TICKDIV,
               MAKE_REG_VAL(div, HPSYS_RCC_CFGR_TICKDIV_Msk, HPSYS_RCC_CFGR_TICKDIV_Pos));
}

/**
 * @brief Get Systick divider
 * @param[in] rcc RCC peripheral pointer
 * @return Current TICKDIV value.
 */
static inline uint32_t ll_rcc_get_tick_div(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CFGR, HPSYS_RCC_CFGR_TICKDIV);
}

/**
 * @brief Set APB2 clock divider pclk2_hpsys = hclk / (2^pdiv2)
 * @param[in] rcc    RCC peripheral pointer
 * @param[in] pdiv2  Exponent value (default 4 = divide by 16)
 */
static inline void ll_rcc_set_apb2_div(HPSYS_RCC_TypeDef *rcc, uint32_t pdiv2)
{
    MODIFY_REG(rcc->CFGR, HPSYS_RCC_CFGR_PDIV2,
               MAKE_REG_VAL(pdiv2, HPSYS_RCC_CFGR_PDIV2_Msk, HPSYS_RCC_CFGR_PDIV2_Pos));
}

/**
 * @brief Get APB2 clock divider exponent
 * @param[in] rcc RCC peripheral pointer
 * @return Current PDIV2 value.
 */
static inline uint32_t ll_rcc_get_apb2_div(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CFGR, HPSYS_RCC_CFGR_PDIV2);
}

/**
 * @brief Set APB1 clock divider pclk_hpsys = hclk / (2^pdiv1)
 * @param[in] rcc    RCC peripheral pointer
 * @param[in] pdiv1  Exponent value (default 1 = divide by 2)
 */
static inline void ll_rcc_set_apb1_div(HPSYS_RCC_TypeDef *rcc, uint32_t pdiv1)
{
    MODIFY_REG(rcc->CFGR, HPSYS_RCC_CFGR_PDIV1,
               MAKE_REG_VAL(pdiv1, HPSYS_RCC_CFGR_PDIV1_Msk, HPSYS_RCC_CFGR_PDIV1_Pos));
}

/**
 * @brief Get APB1 clock divider exponent
 * @param[in] rcc RCC peripheral pointer
 * @return Current PDIV1 value.
 */
static inline uint32_t ll_rcc_get_apb1_div(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CFGR, HPSYS_RCC_CFGR_PDIV1);
}

/*==============================================================================
 * Clock Divider Configuration
 *============================================================================*/

/**
 * @brief Set AHB clock divider hclk_hpsys = clk_hpsys / hdiv
 * @param[in] rcc   RCC peripheral pointer
 * @param[in] hdiv  Divider value (0 = no division)
 */
static inline void ll_rcc_set_ahb_div(HPSYS_RCC_TypeDef *rcc, uint32_t hdiv)
{
    MODIFY_REG(rcc->CFGR, HPSYS_RCC_CFGR_HDIV,
               MAKE_REG_VAL(hdiv, HPSYS_RCC_CFGR_HDIV_Msk, HPSYS_RCC_CFGR_HDIV_Pos));
}

/**
 * @brief Get AHB clock divider
 * @param[in] rcc RCC peripheral pointer
 * @return Current HDIV value.
 */
static inline uint32_t ll_rcc_get_ahb_div(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->CFGR, HPSYS_RCC_CFGR_HDIV);
}

/*==============================================================================
 * USB Clock
 *============================================================================*/

/**
 * @brief Set USB clock divider f_usb = f_src / div
 * @param[in] rcc RCC peripheral pointer
 * @param[in] div Divider value (result must be 60MHz)
 */
static inline void ll_rcc_set_usb_div(HPSYS_RCC_TypeDef *rcc, uint32_t div)
{
    MODIFY_REG(rcc->USBCR, HPSYS_RCC_USBCR_DIV,
               MAKE_REG_VAL(div, HPSYS_RCC_USBCR_DIV_Msk, HPSYS_RCC_USBCR_DIV_Pos));
}

/**
 * @brief Get USB clock divider
 * @param[in] rcc RCC peripheral pointer
 * @return Current USBCR.DIV value.
 */
static inline uint32_t ll_rcc_get_usb_div(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->USBCR, HPSYS_RCC_USBCR_DIV);
}

/*==============================================================================
 * DLL Control
 *============================================================================*/

/** @defgroup LL_RCC_DLL_INDEX DLL Instance Index */
/** @{ */
#define LL_RCC_DLL_INDEX_1 (0U) /**< DLL1 (DLL1CR) */
#define LL_RCC_DLL_INDEX_2 (1U) /**< DLL2 (DLL2CR) */
/** @} */

/**
 * @brief Get the DLLxCR register address for a DLL instance index.
 * @note DLL1CR/DLL2CR share an identical bit layout (EN, STG, IN/OUT_DIV2_EN,
 *       READY, ...), so all DLL helpers use the HPSYS_RCC_DLL1CR_* bit
 *       definitions regardless of the instance. Newer SF32LB variants may
 *       expose more DLL instances; extend the switch to add them (no driver
 *       changes needed).
 * @param rcc RCC peripheral pointer.
 * @param idx DLL instance index (LL_RCC_DLL_INDEX_x).
 * @return Pointer to the DLLxCR register, or NULL for an invalid index.
 */
static inline volatile uint32_t *ll_rcc_dll_cr_reg(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    switch (idx)
    {
    case LL_RCC_DLL_INDEX_1:
        return &rcc->DLL1CR;
    case LL_RCC_DLL_INDEX_2:
        return &rcc->DLL2CR;
    default:
        return NULL;
    }
}

/**
 * @brief Check a DLL instance ready status (DLLxCR.READY).
 * @return Non-zero when the DLL instance is ready.
 */
static inline uint32_t ll_rcc_dll_is_ready(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    volatile uint32_t *cr = ll_rcc_dll_cr_reg(rcc, idx);

    return (cr != NULL) ? (READ_BIT(*cr, HPSYS_RCC_DLL1CR_READY) != 0UL) : 0UL;
}

/**
 * @brief Select whether a DLL instance output is divided by 2 (DLLxCR.OUT_DIV2_EN).
 * @note OUT_DIV2_EN resets to 1 (output divided by 2), see reference manual
 *       page 35.
 * @param rcc  RCC peripheral pointer.
 * @param idx  DLL instance index (LL_RCC_DLL_INDEX_x).
 * @param div2 1 to divide the DLL output by 2, 0 for direct output.
 */
static inline void ll_rcc_dll_set_out_div2(HPSYS_RCC_TypeDef *rcc, uint32_t idx, uint32_t div2)
{
    volatile uint32_t *cr = ll_rcc_dll_cr_reg(rcc, idx);

    if (cr != NULL)
    {
        MODIFY_REG(*cr, HPSYS_RCC_DLL1CR_OUT_DIV2_EN,
                   MAKE_REG_VAL(div2, HPSYS_RCC_DLL1CR_OUT_DIV2_EN_Msk,
                                HPSYS_RCC_DLL1CR_OUT_DIV2_EN_Pos));
    }
}

/**
 * @brief Select whether a DLL instance input is divided by 2 (DLLxCR.IN_DIV2_EN).
 * @param rcc  RCC peripheral pointer.
 * @param idx  DLL instance index (LL_RCC_DLL_INDEX_x).
 * @param div2 1 to divide the DLL input by 2, 0 for direct input.
 */
static inline void ll_rcc_dll_set_in_div2(HPSYS_RCC_TypeDef *rcc, uint32_t idx, uint32_t div2)
{
    volatile uint32_t *cr = ll_rcc_dll_cr_reg(rcc, idx);

    if (cr != NULL)
    {
        MODIFY_REG(*cr, HPSYS_RCC_DLL1CR_IN_DIV2_EN,
                   MAKE_REG_VAL(div2, HPSYS_RCC_DLL1CR_IN_DIV2_EN_Msk,
                                HPSYS_RCC_DLL1CR_IN_DIV2_EN_Pos));
    }
}

/**
 * @brief Set a DLL instance frequency stage (DLLxCR.STG[5:2]).
 * @note f_dll_out = (STG+1)*24MHz when OUT_DIV2_EN=0, or half of that when
 *       OUT_DIV2_EN=1. OUT_DIV2_EN resets to 1, so use
 *       @ref ll_rcc_dll_set_out_div2 first if a non-divided output is wanted.
 * @param rcc RCC peripheral pointer.
 * @param idx DLL instance index (LL_RCC_DLL_INDEX_x).
 * @param stg Frequency stage, see @ref LL_RCC_DLL_STG_*.
 */
static inline void ll_rcc_dll_set_stg(HPSYS_RCC_TypeDef *rcc, uint32_t idx, uint32_t stg)
{
    volatile uint32_t *cr = ll_rcc_dll_cr_reg(rcc, idx);

    if (cr != NULL)
    {
        MODIFY_REG(*cr, HPSYS_RCC_DLL1CR_STG,
                   MAKE_REG_VAL(stg, HPSYS_RCC_DLL1CR_STG_Msk, HPSYS_RCC_DLL1CR_STG_Pos));
    }
}

/**
 * @brief Get a DLL instance frequency stage (DLLxCR.STG[5:2]).
 * @return Current frequency stage.
 */
static inline uint32_t ll_rcc_dll_get_stg(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    volatile uint32_t *cr = ll_rcc_dll_cr_reg(rcc, idx);

    return (cr != NULL) ? GET_REG_VAL2(*cr, HPSYS_RCC_DLL1CR_STG) : 0UL;
}

/**
 * @brief Enable a DLL instance (DLLxCR.EN).
 * @param rcc RCC peripheral pointer.
 * @param idx DLL instance index (LL_RCC_DLL_INDEX_x).
 */
static inline void ll_rcc_dll_enable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    volatile uint32_t *cr = ll_rcc_dll_cr_reg(rcc, idx);

    if (cr != NULL)
    {
        SET_BIT(*cr, HPSYS_RCC_DLL1CR_EN);
    }
}

/**
 * @brief Disable a DLL instance (DLLxCR.EN).
 */
static inline void ll_rcc_dll_disable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    volatile uint32_t *cr = ll_rcc_dll_cr_reg(rcc, idx);

    if (cr != NULL)
    {
        CLEAR_BIT(*cr, HPSYS_RCC_DLL1CR_EN);
    }
}

static inline uint32_t ll_rcc_hrc_is_calibration_done(HPSYS_RCC_TypeDef *rcc)
{
    return READ_BIT(rcc->HRCCAL1, HPSYS_RCC_HRCCAL1_CAL_DONE) ? 1UL : 0UL;
}

/*==============================================================================
 * HRC Calibration
 *============================================================================*/

/**
 * @brief Start HRC48 calibration
 * @note Before starting, the system clock must be switched away from
 *       clk_hrc48 (see reference manual 2.8.2 step 1). Wait for
 *       @ref ll_rcc_hrc_is_calibration_done before reading the result.
 * @param[in] rcc    RCC peripheral pointer.
 * @param[in] length Target clk_hxt48 cycles during calibration (default 0x8000).
 */
static inline void ll_rcc_hrc_start_calibration(HPSYS_RCC_TypeDef *rcc, uint16_t length)
{
    /* Clear enable to reset state */
    CLEAR_BIT(rcc->HRCCAL1, HPSYS_RCC_HRCCAL1_CAL_EN);
    /* Set calibration length (CAL_LENGTH is bit0..15, MAKE_REG_VAL keeps the
     * encoding correct even if the field position changes on other parts). */
    MODIFY_REG(rcc->HRCCAL1, HPSYS_RCC_HRCCAL1_CAL_LENGTH,
               MAKE_REG_VAL(length, HPSYS_RCC_HRCCAL1_CAL_LENGTH_Msk,
                            HPSYS_RCC_HRCCAL1_CAL_LENGTH_Pos));
    /* Start calibration */
    SET_BIT(rcc->HRCCAL1, HPSYS_RCC_HRCCAL1_CAL_EN);
}

/**
 * @brief Read HRC calibration results
 * @param[out] hxt_cnt clk_hxt48 count
 * @param[out] hrc_cnt clk_hrc48 count
 */
static inline void ll_rcc_hrc_get_calibration_result(HPSYS_RCC_TypeDef *rcc, uint16_t *hxt_cnt,
        uint16_t *hrc_cnt)
{
    uint32_t reg = READ_REG(rcc->HRCCAL2);
    *hrc_cnt = (uint16_t)GET_REG_VAL2(reg, HPSYS_RCC_HRCCAL2_HRC_CNT);
    *hxt_cnt = (uint16_t)GET_REG_VAL2(reg, HPSYS_RCC_HRCCAL2_HXT_CNT);
}
static inline void ll_rcc_deepwfi_set_dll_output(HPSYS_RCC_TypeDef *rcc,
        uint32_t dll1_out_en,
        uint32_t dll1_rstb,
        uint32_t dll2_out_en,
        uint32_t dll2_rstb)
{
    MODIFY_REG(rcc->DWCFGR,
               HPSYS_RCC_DWCFGR_DLL1_OUT_EN | HPSYS_RCC_DWCFGR_DLL1_OUT_RSTB |
               HPSYS_RCC_DWCFGR_DLL2_OUT_EN | HPSYS_RCC_DWCFGR_DLL2_OUT_RSTB,
               (dll1_out_en ? HPSYS_RCC_DWCFGR_DLL1_OUT_EN : 0UL) |
               (dll1_rstb ? HPSYS_RCC_DWCFGR_DLL1_OUT_RSTB : 0UL) |
               (dll2_out_en ? HPSYS_RCC_DWCFGR_DLL2_OUT_EN : 0UL) |
               (dll2_rstb ? HPSYS_RCC_DWCFGR_DLL2_OUT_RSTB : 0UL));
}

static inline void ll_rcc_deepwfi_set_sys_lp_clock_source(HPSYS_RCC_TypeDef *rcc,
        uint32_t src)
{
    MODIFY_REG(rcc->DWCFGR, HPSYS_RCC_DWCFGR_SEL_SYS_LP,
               MAKE_REG_VAL(src, HPSYS_RCC_DWCFGR_SEL_SYS_LP_Msk,
                            HPSYS_RCC_DWCFGR_SEL_SYS_LP_Pos));
}

/**
 * @brief Set system clock source during Deep WFI
 * @param[in] rcc RCC peripheral pointer.
 * @param[in] src Use LL_RCC_SYS_CLKSRC_xxx.
 */
static inline void ll_rcc_deepwfi_set_sys_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t src)
{
    MODIFY_REG(rcc->DWCFGR, HPSYS_RCC_DWCFGR_SEL_SYS,
               MAKE_REG_VAL(src, HPSYS_RCC_DWCFGR_SEL_SYS_Msk, HPSYS_RCC_DWCFGR_SEL_SYS_Pos));
}

/*==============================================================================
 * Deep WFI Clock Configuration
 *============================================================================*/

/**
 * @brief Enable divider reconfiguration during Deep WFI
 */
static inline void ll_rcc_deepwfi_enable_div_reconfig(HPSYS_RCC_TypeDef *rcc)
{
    SET_BIT(rcc->DWCFGR, HPSYS_RCC_DWCFGR_DIV_EN);
}

/**
 * @brief Disable divider reconfiguration during Deep WFI
 */
static inline void ll_rcc_deepwfi_disable_div_reconfig(HPSYS_RCC_TypeDef *rcc)
{
    CLEAR_BIT(rcc->DWCFGR, HPSYS_RCC_DWCFGR_DIV_EN);
}
static inline void ll_rcc_deepwfi_set_apb_div(HPSYS_RCC_TypeDef *rcc,
        uint32_t pdiv1, uint32_t pdiv2)
{
    MODIFY_REG(rcc->DWCFGR, HPSYS_RCC_DWCFGR_PDIV1 | HPSYS_RCC_DWCFGR_PDIV2,
               MAKE_REG_VAL(pdiv1, HPSYS_RCC_DWCFGR_PDIV1_Msk,
                            HPSYS_RCC_DWCFGR_PDIV1_Pos) |
               MAKE_REG_VAL(pdiv2, HPSYS_RCC_DWCFGR_PDIV2_Msk,
                            HPSYS_RCC_DWCFGR_PDIV2_Pos));
}

/**
 * @brief Set AHB divider during Deep WFI
 * @param[in] rcc  RCC peripheral pointer.
 * @param[in] hdiv Divider value (0 = no division).
 */
static inline void ll_rcc_deepwfi_set_ahb_div(HPSYS_RCC_TypeDef *rcc, uint32_t hdiv)
{
    MODIFY_REG(rcc->DWCFGR, HPSYS_RCC_DWCFGR_HDIV,
               MAKE_REG_VAL(hdiv, HPSYS_RCC_DWCFGR_HDIV_Msk, HPSYS_RCC_DWCFGR_HDIV_Pos));
}

/*==============================================================================
 * Clock Frequency Query
 *============================================================================*/

#define LL_RCC_HRC48_FREQ (48000000UL) /**< HRC48 frequency */
#define LL_RCC_HXT48_FREQ (48000000UL) /**
 * @brief Get current clk_hpsys system clock frequency
 * @note Only valid in normal active mode with CSR.SEL_SYS_LP=0 (system clock
 *       selected by CSR.SEL_SYS). It does not reflect Deep WFI (DWCFGR) or
 *       the clk_wdt low-power mode configuration.
 * @param[in] rcc RCC peripheral pointer.
 * @return clk_hpsys frequency in Hz.
 */
static inline uint32_t ll_rcc_get_sys_clock_freq(HPSYS_RCC_TypeDef *rcc)
{
    uint32_t src = ll_rcc_get_sys_clock_source(rcc);
    uint32_t freq;

    if (src == LL_RCC_SYS_CLKSRC_HXT48)
    {
        freq = LL_RCC_HXT48_FREQ;
    }
    else if (src == LL_RCC_SYS_CLKSRC_DLL1)
    {
        uint32_t stg = GET_REG_VAL2(rcc->DLL1CR, HPSYS_RCC_DLL1CR_STG);
        freq = (stg + 1U) * 24000000UL;
        if (READ_BIT(rcc->DLL1CR, HPSYS_RCC_DLL1CR_OUT_DIV2_EN))
        {
            freq /= 2U;
        }
    }
    else
    {
        freq = LL_RCC_HRC48_FREQ;
    }
    return freq;
}

/**
 * @brief Get current HCLK frequency
 */
static inline uint32_t ll_rcc_get_hclk_freq(HPSYS_RCC_TypeDef *rcc)
{
    uint32_t hdiv = ll_rcc_get_ahb_div(rcc);
    uint32_t clk_sys = ll_rcc_get_sys_clock_freq(rcc);
    return (hdiv == 0UL) ? clk_sys : (clk_sys / hdiv);
}

/**
 * @brief Get current PCLK2 frequency
 */
static inline uint32_t ll_rcc_get_pclk2_freq(HPSYS_RCC_TypeDef *rcc)
{
    uint32_t pdiv2 = GET_REG_VAL2(rcc->CFGR, HPSYS_RCC_CFGR_PDIV2);
    return ll_rcc_get_hclk_freq(rcc) / (1UL << pdiv2);
}

/**
 * @brief Get current PCLK frequency
 */
static inline uint32_t ll_rcc_get_pclk_freq(HPSYS_RCC_TypeDef *rcc)
{
    uint32_t pdiv1 = GET_REG_VAL2(rcc->CFGR, HPSYS_RCC_CFGR_PDIV1);
    return ll_rcc_get_hclk_freq(rcc) / (1UL << pdiv1);
}

/**< HXT48 frequency */

/*==============================================================================
 * Debug Configuration (DBGR / DBGCLKR / TESTR)
 *============================================================================*/

/* DBGR.FORCE_BUS / FORCE_GPIO bit definitions are missing from regs/hpsys_rcc.h;
 * defined here as LL-local until the regs header is updated upstream. */
#define HPSYS_RCC_DBGR_FORCE_BUS_Pos  (2U)
#define HPSYS_RCC_DBGR_FORCE_BUS_Msk  (0x1UL << HPSYS_RCC_DBGR_FORCE_BUS_Pos)
#define HPSYS_RCC_DBGR_FORCE_BUS      HPSYS_RCC_DBGR_FORCE_BUS_Msk
#define HPSYS_RCC_DBGR_FORCE_GPIO_Pos (3U)
#define HPSYS_RCC_DBGR_FORCE_GPIO_Msk (0x1UL << HPSYS_RCC_DBGR_FORCE_GPIO_Pos)
#define HPSYS_RCC_DBGR_FORCE_GPIO     HPSYS_RCC_DBGR_FORCE_GPIO_Msk

/**
 * @brief DBGCLKR DLL field mask for a DLL instance index.
 * @note DLL2 fields sit exactly 8 bits above the DLL1 fields in DBGCLKR,
 *       so shifting the DLL1 mask covers both instances.
 * @param[in] idx       DLL instance index (LL_RCC_DLL_INDEX_x).
 * @param[in] dll1_mask DLL1 field mask (e.g. HPSYS_RCC_DBGCLKR_DLL1_DBG).
 * @return Mask for the selected DLL instance.
 */
static inline uint32_t ll_rcc_dbgclkr_dll_field(uint32_t idx, uint32_t dll1_mask)
{
    return (idx == LL_RCC_DLL_INDEX_2) ? (dll1_mask << 8U) : dll1_mask;
}

/**
 * @brief Set the DLL output strength (DBGCLKR.DLLx_OUT_STR[1:0]).
 * @param[in] rcc RCC peripheral pointer.
 * @param[in] idx DLL instance index (LL_RCC_DLL_INDEX_x).
 * @param[in] str Output strength (2 bits).
 */
static inline void ll_rcc_set_dll_out_strength(HPSYS_RCC_TypeDef *rcc, uint32_t idx,
        uint32_t str)
{
    MODIFY_REG(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_OUT_STR),
               MAKE_REG_VAL(str, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_OUT_STR),
                            HPSYS_RCC_DBGCLKR_DLL1_OUT_STR_Pos +
                            ((idx == LL_RCC_DLL_INDEX_2) ? 8U : 0U)));
}

/**
 * @brief Enable the DLL clock gating (DBGCLKR.DLLx_CG_EN).
 */
static inline void ll_rcc_dll_clock_gate_enable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    SET_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_CG_EN));
}

/**
 * @brief Disable the DLL clock gating (DBGCLKR.DLLx_CG_EN = 0).
 */
static inline void ll_rcc_dll_clock_gate_disable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    CLEAR_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_CG_EN));
}

/**
 * @brief Assert the DLL output reset (DBGCLKR.DLLx_OUT_RSTB = 0).
 */
static inline void ll_rcc_dll_out_assert_reset(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    CLEAR_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_OUT_RSTB));
}

/**
 * @brief Release the DLL output reset (DBGCLKR.DLLx_OUT_RSTB = 1).
 */
static inline void ll_rcc_dll_out_release_reset(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    SET_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_OUT_RSTB));
}

/**
 * @brief Enable the DLL loopback (DBGCLKR.DLLx_LOOP_EN).
 */
static inline void ll_rcc_dll_loop_enable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    SET_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_LOOP_EN));
}

/**
 * @brief Disable the DLL loopback (DBGCLKR.DLLx_LOOP_EN = 0).
 */
static inline void ll_rcc_dll_loop_disable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    CLEAR_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_LOOP_EN));
}

/**
 * @brief Enable the DLL output (DBGCLKR.DLLx_OUT_EN).
 */
static inline void ll_rcc_dll_out_enable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    SET_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_OUT_EN));
}

/**
 * @brief Disable the DLL output (DBGCLKR.DLLx_OUT_EN = 0).
 */
static inline void ll_rcc_dll_out_disable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    CLEAR_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_OUT_EN));
}

/**
 * @brief Enable the DLL LDO (DBGCLKR.DLLx_LDO_EN).
 */
static inline void ll_rcc_dll_ldo_enable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    SET_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_LDO_EN));
}

/**
 * @brief Disable the DLL LDO (DBGCLKR.DLLx_LDO_EN = 0).
 */
static inline void ll_rcc_dll_ldo_disable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    CLEAR_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_LDO_EN));
}

/**
 * @brief Enable DLL debug access (DBGCLKR.DLLx_DBG).
 * @param[in] rcc RCC peripheral pointer.
 * @param[in] idx DLL instance index (LL_RCC_DLL_INDEX_x).
 */
static inline void ll_rcc_dll_dbg_enable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    SET_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_DBG));
}

/**
 * @brief Disable DLL debug access (DBGCLKR.DLLx_DBG = 0).
 */
static inline void ll_rcc_dll_dbg_disable(HPSYS_RCC_TypeDef *rcc, uint32_t idx)
{
    CLEAR_BIT(rcc->DBGCLKR, ll_rcc_dbgclkr_dll_field(idx, HPSYS_RCC_DBGCLKR_DLL1_DBG));
}

/**
 * @brief Enable the debug clock (DBGCLKR.CLK_EN).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_debug_clock_enable(HPSYS_RCC_TypeDef *rcc)
{
    SET_BIT(rcc->DBGCLKR, HPSYS_RCC_DBGCLKR_CLK_EN);
}

/**
 * @brief Disable the debug clock (DBGCLKR.CLK_EN = 0).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_debug_clock_disable(HPSYS_RCC_TypeDef *rcc)
{
    CLEAR_BIT(rcc->DBGCLKR, HPSYS_RCC_DBGCLKR_CLK_EN);
}

/**
 * @brief Set the debug clock source (DBGCLKR.CLK_SEL[1:0]).
 * @param[in] rcc RCC peripheral pointer.
 * @param[in] sel Debug clock source select.
 */
static inline void ll_rcc_set_debug_clock_source(HPSYS_RCC_TypeDef *rcc, uint32_t sel)
{
    MODIFY_REG(rcc->DBGCLKR, HPSYS_RCC_DBGCLKR_CLK_SEL,
               MAKE_REG_VAL(sel, HPSYS_RCC_DBGCLKR_CLK_SEL_Msk,
                            HPSYS_RCC_DBGCLKR_CLK_SEL_Pos));
}

/**
 * @brief Get the debug clock source (DBGCLKR.CLK_SEL[1:0]).
 * @param[in] rcc RCC peripheral pointer.
 * @return Debug clock source select.
 */
static inline uint32_t ll_rcc_get_debug_clock_source(HPSYS_RCC_TypeDef *rcc)
{
    return GET_REG_VAL2(rcc->DBGCLKR, HPSYS_RCC_DBGCLKR_CLK_SEL);
}

/**
 * @brief Force hclk_hp and pclk_hp on (DBGR.FORCE_HP).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_force_hp_enable(HPSYS_RCC_TypeDef *rcc)
{
    SET_BIT(rcc->DBGR, HPSYS_RCC_DBGR_FORCE_HP);
}

/**
 * @brief Allow hclk_hp and pclk_hp to gate (DBGR.FORCE_HP = 0).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_force_hp_disable(HPSYS_RCC_TypeDef *rcc)
{
    CLEAR_BIT(rcc->DBGR, HPSYS_RCC_DBGR_FORCE_HP);
}

/**
 * @brief Keep GPIO on HXT during LS/DS (DBGR.FORCE_GPIO).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_force_gpio_enable(HPSYS_RCC_TypeDef *rcc)
{
    SET_BIT(rcc->DBGR, HPSYS_RCC_DBGR_FORCE_GPIO);
}

/**
 * @brief Allow GPIO to switch to LPCLK during LS/DS (DBGR.FORCE_GPIO = 0).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_force_gpio_disable(HPSYS_RCC_TypeDef *rcc)
{
    CLEAR_BIT(rcc->DBGR, HPSYS_RCC_DBGR_FORCE_GPIO);
}

/**
 * @brief Force hclk_bus and hclk_hpbus on (DBGR.FORCE_BUS).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_force_bus_enable(HPSYS_RCC_TypeDef *rcc)
{
    SET_BIT(rcc->DBGR, HPSYS_RCC_DBGR_FORCE_BUS);
}

/**
 * @brief Allow hclk_bus and hclk_hpbus to gate (DBGR.FORCE_BUS = 0).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_force_bus_disable(HPSYS_RCC_TypeDef *rcc)
{
    CLEAR_BIT(rcc->DBGR, HPSYS_RCC_DBGR_FORCE_BUS);
}

/**
 * @brief Switch the system clock to CLK_LP during LS/DS (DBGR.SYSCLK_SWLP).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_sysclk_swlp_enable(HPSYS_RCC_TypeDef *rcc)
{
    SET_BIT(rcc->DBGR, HPSYS_RCC_DBGR_SYSCLK_SWLP);
}

/**
 * @brief Keep the system clock on HXT during LS/DS (DBGR.SYSCLK_SWLP = 0).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_sysclk_swlp_disable(HPSYS_RCC_TypeDef *rcc)
{
    CLEAR_BIT(rcc->DBGR, HPSYS_RCC_DBGR_SYSCLK_SWLP);
}

/**
 * @brief Force the system clock always on (DBGR.SYSCLK_AON).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_sysclk_aon_enable(HPSYS_RCC_TypeDef *rcc)
{
    SET_BIT(rcc->DBGR, HPSYS_RCC_DBGR_SYSCLK_AON);
}

/**
 * @brief Allow the system clock to gate (DBGR.SYSCLK_AON = 0).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_dbg_sysclk_aon_disable(HPSYS_RCC_TypeDef *rcc)
{
    CLEAR_BIT(rcc->DBGR, HPSYS_RCC_DBGR_SYSCLK_AON);
}

/**
 * @brief Enter scan test mode (TESTR.SCAN_MODE = 0xdeadbeef).
 * @param[in] rcc RCC peripheral pointer.
 */
static inline void ll_rcc_enter_scan_mode(HPSYS_RCC_TypeDef *rcc)
{
    WRITE_REG(rcc->TESTR, 0xdeadbeefUL);
}

/**
 * @brief Write the RCC test register (TESTR).
 * @param[in] rcc RCC peripheral pointer.
 * @param[in] val Raw value.
 */
static inline void ll_rcc_write_test_reg(HPSYS_RCC_TypeDef *rcc, uint32_t val)
{
    WRITE_REG(rcc->TESTR, val);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_RCC_H */
