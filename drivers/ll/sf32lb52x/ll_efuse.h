/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_EFUSE_H
#define __LL_EFUSE_H

#include <stddef.h>
#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_efuse.h
 * @brief Header-only low-level eFuse Controller (EFUSEC) APIs for SF32LB52x.
 *
 * EFUSEC manages one-time programmable eFuse memory organized in 4 banks
 * (each 8 x 32-bit words). Supports read mode and program mode.
 */

/** @defgroup LL_EFUSE_MODE eFuse Operation Mode */
/** @{ */
#define LL_EFUSE_MODE_READ    (0x0UL) /**< Read mode */
#define LL_EFUSE_MODE_PROGRAM (0x1UL) /**< Program (write) mode */
/** @} */

/** @defgroup LL_EFUSE_BANK eFuse Bank Selection */
/** @{ */
#define LL_EFUSE_BANK_0 (0x0UL)
#define LL_EFUSE_BANK_1 (0x1UL)
#define LL_EFUSE_BANK_2 (0x2UL)
#define LL_EFUSE_BANK_3 (0x3UL)
/** @} */

#define LL_EFUSE_BANK_COUNT      (4UL)
#define LL_EFUSE_BANK_WORD_COUNT (8UL)
static inline void ll_efuse_enable_interrupt(EFUSEC_TypeDef *efuse)
{
    SET_BIT(efuse->CR, EFUSEC_CR_IE);
}

static inline void ll_efuse_disable_interrupt(EFUSEC_TypeDef *efuse)
{
    CLEAR_BIT(efuse->CR, EFUSEC_CR_IE);
}

/**
 * @brief Select target bank
 */
static inline void ll_efuse_set_bank(EFUSEC_TypeDef *efuse, uint32_t bank)
{
    MODIFY_REG(efuse->CR, EFUSEC_CR_BANKSEL,
               MAKE_REG_VAL(bank, EFUSEC_CR_BANKSEL_Msk, EFUSEC_CR_BANKSEL_Pos));
}

/**
 * @brief Set operation mode
 */
static inline void ll_efuse_set_mode(EFUSEC_TypeDef *efuse, uint32_t mode)
{
    MODIFY_REG(efuse->CR, EFUSEC_CR_MODE,
               MAKE_REG_VAL(mode, EFUSEC_CR_MODE_Msk, EFUSEC_CR_MODE_Pos));
}

/*==============================================================================
 * Basic Control
 *============================================================================*/

/**
 * @brief Trigger the configured eFuse read or program operation.
 * @note CR.EN is a write-one-set, self-clearing command bit. Configure MODE,
 *       BANKSEL, PGM_DATA and TIMR, and clear the DONE flag before triggering.
 */
static inline void ll_efuse_start(EFUSEC_TypeDef *efuse)
{
    SET_BIT(efuse->CR, EFUSEC_CR_EN);
}

/*==============================================================================
 * Timing Configuration
 *============================================================================*/

/**
 * @brief Configure eFuse timing parameters
 * @param thrck  Read clock timing
 * @param thpck  Program clock timing
 * @param tckhp  Clock high pulse
 */
static inline void ll_efuse_set_timing(EFUSEC_TypeDef *efuse, uint32_t thrck, uint32_t thpck,
                                       uint32_t tckhp)
{
    WRITE_REG(efuse->TIMR,
              MAKE_REG_VAL(thrck, EFUSEC_TIMR_THRCK_Msk, EFUSEC_TIMR_THRCK_Pos) |
              MAKE_REG_VAL(thpck, EFUSEC_TIMR_THPCK_Msk, EFUSEC_TIMR_THPCK_Pos) |
              MAKE_REG_VAL(tckhp, EFUSEC_TIMR_TCKHP_Msk, EFUSEC_TIMR_TCKHP_Pos));
}

/*==============================================================================
 * Status & Interrupt
 *============================================================================*/

static inline uint32_t ll_efuse_is_done(EFUSEC_TypeDef *efuse)
{
    return READ_BIT(efuse->SR, EFUSEC_SR_DONE) ? 1UL : 0UL;
}

/**
 * @brief Clear the operation done flag.
 * @note SR.DONE is cleared by writing one.
 */
static inline void ll_efuse_clear_done(EFUSEC_TypeDef *efuse)
{
    WRITE_REG(efuse->SR, EFUSEC_SR_DONE);
}

/*==============================================================================
 * Program (Write) Data
 *============================================================================*/

/**
 * @brief Write program data into PGM_DATA registers
 * @param idx  Word index (0..7)
 * @param val  32-bit value to program
 */
static inline void ll_efuse_set_program_data(EFUSEC_TypeDef *efuse, uint32_t idx, uint32_t val)
{
    volatile uint32_t *pgm_regs = &efuse->PGM_DATA0;
    if (idx < 8U)
    {
        pgm_regs[idx] = val;
    }
}

/**
 * @brief Write all 8 program data words
 */
static inline void ll_efuse_set_program_data_all(EFUSEC_TypeDef *efuse, const uint32_t *data)
{
    volatile uint32_t *pgm_regs = &efuse->PGM_DATA0;
    uint32_t idx;

    if (data == NULL)
    {
        return;
    }

    for (idx = 0U; idx < LL_EFUSE_BANK_WORD_COUNT; idx++)
    {
        pgm_regs[idx] = data[idx];
    }
}

/*==============================================================================
 * Read Bank Data
 *============================================================================*/

/**
 * @brief Read one 32-bit word from a bank
 * @param[in] efuse EFUSEC instance pointer.
 * @param[in] bank Bank index, use @ref LL_EFUSE_BANK_0 to @ref LL_EFUSE_BANK_3.
 * @param[in] idx Word index within the bank, in the range 0 to 7.
 * @return Bank word value, or 0 when @p bank or @p idx is invalid.
 * @note BANKx_DATAy registers hold the result of the last read operation;
 *       trigger a read with @ref LL_EFUSE_MODE_READ before reading.
 */
static inline uint32_t ll_efuse_read_word(EFUSEC_TypeDef *efuse, uint32_t bank, uint32_t idx)
{
    volatile uint32_t *data_reg;

    if ((bank >= LL_EFUSE_BANK_COUNT) || (idx >= LL_EFUSE_BANK_WORD_COUNT))
    {
        return 0UL;
    }

    data_reg =
        (volatile uint32_t *)((uintptr_t)efuse + offsetof(EFUSEC_TypeDef, BANK0_DATA0) +
                              ((bank * LL_EFUSE_BANK_WORD_COUNT + idx) * sizeof(uint32_t)));
    return READ_REG(*data_reg);
}

/**
 * @brief Read entire bank (8 words = 32 bytes) into buffer
 * @param[in] efuse EFUSEC instance pointer.
 * @param[in] bank Bank index, use @ref LL_EFUSE_BANK_0 to @ref LL_EFUSE_BANK_3.
 * @param[out] buf Output buffer with at least @ref LL_EFUSE_BANK_WORD_COUNT words.
 * @return 1 when the bank is read; 0 when @p bank is invalid or @p buf is NULL.
 * @note BANKx_DATAy registers hold the result of the last read operation;
 *       trigger a read with @ref LL_EFUSE_MODE_READ before reading.
 */
static inline uint32_t ll_efuse_read_bank(EFUSEC_TypeDef *efuse, uint32_t bank, uint32_t *buf)
{
    uint32_t idx;

    if ((bank >= LL_EFUSE_BANK_COUNT) || (buf == NULL))
    {
        return 0UL;
    }

    for (idx = 0U; idx < LL_EFUSE_BANK_WORD_COUNT; idx++)
    {
        buf[idx] = ll_efuse_read_word(efuse, bank, idx);
    }

    return 1UL;
}

/**
 * @brief Read one 32-bit word at an absolute register offset from the eFuse base.
 * @param[in] efuse  EFUSEC instance pointer.
 * @param[in] offset Absolute byte offset of the BANKx_DATAy register from the
 *                   eFuse controller base (e.g. from a DT bank-offsets array).
 * @return Register word value.
 * @note BANKx_DATAy registers hold the result of the last read operation;
 *       trigger a read with @ref LL_EFUSE_MODE_READ before reading.
 */
static inline uint32_t ll_efuse_read_word_at(EFUSEC_TypeDef *efuse, uint32_t offset)
{
    volatile uint32_t *data_reg = (volatile uint32_t *)((uintptr_t)efuse + offset);

    return READ_REG(*data_reg);
}

/**
 * @brief Read the ANACR RESERVE1 field.
 * @param[in] efuse eFuse controller instance pointer.
 * @return RESERVE1 field value.
 */
static inline uint32_t ll_efuse_get_anacr_reserve1(EFUSEC_TypeDef *efuse)
{
    return (READ_REG(efuse->ANACR) & EFUSEC_ANACR_RESERVE1) >>
           EFUSEC_ANACR_RESERVE1_Pos;
}

/**
 * @brief Set the writable ANACR reserve field (ANACR.RESERVE0).
 * @param[in] efuse eFuse controller instance pointer.
 * @param[in] value Reserve field value (8 bits).
 */
static inline void ll_efuse_set_anacr_reserve0(EFUSEC_TypeDef *efuse, uint32_t value)
{
    MODIFY_REG(efuse->ANACR, EFUSEC_ANACR_RESERVE0,
               MAKE_REG_VAL(value, EFUSEC_ANACR_RESERVE0_Msk,
                            EFUSEC_ANACR_RESERVE0_Pos));
}

/**
 * @brief Read the ANACR RESERVE0 field.
 * @param[in] efuse eFuse controller instance pointer.
 * @return RESERVE0 field value.
 */
static inline uint32_t ll_efuse_get_anacr_reserve0(EFUSEC_TypeDef *efuse)
{
    return (READ_REG(efuse->ANACR) & EFUSEC_ANACR_RESERVE0) >>
           EFUSEC_ANACR_RESERVE0_Pos;
}

/**
 * @brief Set the eFuse analog LDO DC trimming (ANACR.LDO_DC_TR).
 * @param[in] efuse eFuse controller instance pointer.
 * @param[in] tr    DC trimming value (3 bits).
 */
static inline void ll_efuse_set_anacr_ldo_dc_trim(EFUSEC_TypeDef *efuse, uint32_t tr)
{
    MODIFY_REG(efuse->ANACR, EFUSEC_ANACR_LDO_DC_TR,
               MAKE_REG_VAL(tr, EFUSEC_ANACR_LDO_DC_TR_Msk, EFUSEC_ANACR_LDO_DC_TR_Pos));
}

/**
 * @brief Set the eFuse analog LDO mode (ANACR.LDO_MODE).
 * @param[in] efuse eFuse controller instance pointer.
 * @param[in] mode  LDO mode.
 */
static inline void ll_efuse_set_anacr_ldo_mode(EFUSEC_TypeDef *efuse, uint32_t mode)
{
    MODIFY_REG(efuse->ANACR, EFUSEC_ANACR_LDO_MODE,
               MAKE_REG_VAL(mode, EFUSEC_ANACR_LDO_MODE_Msk, EFUSEC_ANACR_LDO_MODE_Pos));
}

/**
 * @brief Set the eFuse analog LDO reference voltage select (ANACR.LDO_VREF_SEL).
 * @param[in] efuse eFuse controller instance pointer.
 * @param[in] sel   Reference voltage select (3 bits).
 */
static inline void ll_efuse_set_anacr_ldo_vref(EFUSEC_TypeDef *efuse, uint32_t sel)
{
    MODIFY_REG(efuse->ANACR, EFUSEC_ANACR_LDO_VREF_SEL,
               MAKE_REG_VAL(sel, EFUSEC_ANACR_LDO_VREF_SEL_Msk, EFUSEC_ANACR_LDO_VREF_SEL_Pos));
}

/**
 * @brief Set the eFuse analog LDO enable (ANACR.LDO_EN).
 * @param[in] efuse eFuse controller instance pointer.
 * @param[in] en    1 to enable, 0 to disable.
 */
static inline void ll_efuse_set_anacr_ldo_enable(EFUSEC_TypeDef *efuse, uint32_t en)
{
    en ? SET_BIT(efuse->ANACR, EFUSEC_ANACR_LDO_EN) : CLEAR_BIT(efuse->ANACR, EFUSEC_ANACR_LDO_EN);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_EFUSE_H */
