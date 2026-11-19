/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_MPI_H
#define __LL_MPI_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_mpi.h
 * @brief Header-only low-level Memory Parallel Interface (MPI) APIs for SF32LB52x.
 *
 * MPI is a multi-protocol external memory interface supporting QSPI NOR Flash,
 * OPI PSRAM, and HyperBus devices. Each instance has two chip-selects (CS1/CS2).
 */

/*==============================================================================
 * Device Type (DCR)
 *============================================================================*/

/** @defgroup LL_MPI_PROTO Protocol Mode */
/** @{ */
#define LL_MPI_PROTO_SPI   (0x0UL)
#define LL_MPI_PROTO_OPI   (0x1UL)
#define LL_MPI_PROTO_HYPER (0x2UL)
/** @} */

/** @defgroup LL_MPI_CS Chip Select */
/** @{ */
#define LL_MPI_CS_1 (1UL)
#define LL_MPI_CS_2 (2UL)
/** @} */

/** @defgroup LL_MPI_RB_SIZE Row Boundary Size (DCR.RBSIZE) */
/** @{ */
#define LL_MPI_RB_SIZE_NONE      (0x0UL) /**< No row boundary */
#define LL_MPI_RB_SIZE_16_BYTES  (0x1UL)
#define LL_MPI_RB_SIZE_32_BYTES  (0x2UL)
#define LL_MPI_RB_SIZE_64_BYTES  (0x3UL)
#define LL_MPI_RB_SIZE_128_BYTES (0x4UL)
#define LL_MPI_RB_SIZE_256_BYTES (0x5UL)
/** @} */

/** @defgroup LL_MPI_FIFOCR FIFO Flush */
/** @{ */
#define LL_MPI_FIFOCR_FLUSH (MPI_FIFOCR_RXCLR | MPI_FIFOCR_TXCLR)
static inline void ll_mpi_abort(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_ABORT);
}

static inline void ll_mpi_set_ahb_access_disable(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->CR, MPI_CR_AHBDIS, en ? MPI_CR_AHBDIS : 0UL);
}

/**
 * @brief Set dual-flash mode (CR.DFM).
 * @param[in] mpi MPI instance pointer.
 * @param[in] en Non-zero to enable dual-flash mode, zero to disable.
 */
static inline void ll_mpi_set_dual_flash_mode(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->CR, MPI_CR_DFM, en ? MPI_CR_DFM : 0UL);
}

static inline void ll_mpi_set_x16_mode(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->CR, MPI_CR_MX16, en ? MPI_CR_MX16 : 0UL);
}

static inline void ll_mpi_set_prefetch_enable(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->CR, MPI_CR_PREFE, en ? MPI_CR_PREFE : 0UL);
}

/**
 * @brief Set protocol mode (SPI / OPI / HyperBus)
 */
static inline void ll_mpi_set_protocol(MPI_TypeDef *mpi, uint32_t proto)
{
    if (proto == LL_MPI_PROTO_OPI || proto == LL_MPI_PROTO_HYPER)
    {
        /* HyperBus also uses the octal (x8) interface. */
        SET_BIT(mpi->CR, MPI_CR_OPIE);
    }
    else
    {
        CLEAR_BIT(mpi->CR, MPI_CR_OPIE);
    }
    if (proto == LL_MPI_PROTO_HYPER)
    {
        SET_BIT(mpi->DCR, MPI_DCR_HYPER);
    }
    else
    {
        CLEAR_BIT(mpi->DCR, MPI_DCR_HYPER);
    }
}

static inline void ll_mpi_set_hardware_interface_enable(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->CR, MPI_CR_HWIFE, en ? MPI_CR_HWIFE : 0UL);
}

/**
 * @brief Set status-match comparison mode (CR.SMM).
 * @param[in] mpi MPI instance pointer.
 * @param[in] mode 0 for AND mode, 1 for OR mode.
 */
static inline void ll_mpi_set_status_match_mode(MPI_TypeDef *mpi, uint32_t mode)
{
    MODIFY_REG(mpi->CR, MPI_CR_SMM, mode ? MPI_CR_SMM : 0UL);
}

/*==============================================================================
 * Chip Select & Protocol Configuration
 *============================================================================*/

/**
 * @brief Enable or disable status-match polling for a command sequence.
 * @param[in] mpi MPI instance pointer.
 * @param[in] cs Chip select, 1 or 2.
 * @param[in] en Non-zero to enable status-match polling, zero to disable it.
 * @note Maps to CR.SME1 (CS1/CMD1) or CR.SME2 (CS2/CMD2). Invalid chip
 *       selects are ignored.
 */
static inline void ll_mpi_set_status_match_enable(MPI_TypeDef *mpi, uint32_t cs, uint32_t en)
{
    if (cs == LL_MPI_CS_1)
    {
        MODIFY_REG(mpi->CR, MPI_CR_SME1, en ? MPI_CR_SME1 : 0UL);
    }
    else if (cs == LL_MPI_CS_2)
    {
        MODIFY_REG(mpi->CR, MPI_CR_SME2, en ? MPI_CR_SME2 : 0UL);
    }
}

/**
 * @brief Enable the CMD2 (chip select 2) command execution (CR.CMD2E).
 * @param[in] mpi MPI instance pointer.
 */
static inline void ll_mpi_enable_command2(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_CMD2E);
}

/**
 * @brief Disable the CMD2 (chip select 2) command execution (CR.CMD2E).
 * @param[in] mpi MPI instance pointer.
 */
static inline void ll_mpi_disable_command2(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_CMD2E);
}

static inline void ll_mpi_enable_row_boundary_interrupt(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_RBXIE);
}

static inline void ll_mpi_disable_row_boundary_interrupt(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_RBXIE);
}

static inline void ll_mpi_enable_cs_violation_interrupt(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_CSVIE);
}

static inline void ll_mpi_disable_cs_violation_interrupt(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_CSVIE);
}

static inline void ll_mpi_enable_status_match_interrupt(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_SMIE);
}

static inline void ll_mpi_disable_status_match_interrupt(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_SMIE);
}

static inline void ll_mpi_enable_fifo_overflow_interrupt(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_FOIE);
}

static inline void ll_mpi_disable_fifo_overflow_interrupt(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_FOIE);
}

static inline void ll_mpi_enable_fifo_underflow_interrupt(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_FUIE);
}

static inline void ll_mpi_disable_fifo_underflow_interrupt(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_FUIE);
}

static inline void ll_mpi_enable_transfer_complete_interrupt(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_TCIE);
}

static inline void ll_mpi_disable_transfer_complete_interrupt(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_TCIE);
}

static inline void ll_mpi_set_ctr_mode(MPI_TypeDef *mpi, uint32_t mode)
{
    MODIFY_REG(mpi->CR, MPI_CR_CTRM, mode ? MPI_CR_CTRM : 0UL);
}

static inline void ll_mpi_set_ctr_enable(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->CR, MPI_CR_CTRE, en ? MPI_CR_CTRE : 0UL);
}

/*==============================================================================
 * DMA
 *============================================================================*/

static inline void ll_mpi_enable_dma(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_DMAE);
}

static inline void ll_mpi_disable_dma(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_DMAE);
}

static inline void ll_mpi_set_hold_value(MPI_TypeDef *mpi, uint32_t value)
{
    MODIFY_REG(mpi->CR, MPI_CR_HOLD, value ? MPI_CR_HOLD : 0UL);
}

static inline void ll_mpi_set_hold_enable(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->CR, MPI_CR_HOLDE, en ? MPI_CR_HOLDE : 0UL);
}

/*==============================================================================
 * Write Protection
 *============================================================================*/

static inline void ll_mpi_enable_write(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_WP);
}

static inline void ll_mpi_disable_write(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_WP);
}

static inline void ll_mpi_set_write_protect_enable(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->CR, MPI_CR_WPE, en ? MPI_CR_WPE : 0UL);
}

/** @} */

/*==============================================================================
 * Basic Control
 *============================================================================*/

static inline void ll_mpi_enable(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CR, MPI_CR_EN);
}

static inline void ll_mpi_disable(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CR, MPI_CR_EN);
}

static inline uint32_t ll_mpi_is_enabled(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->CR, MPI_CR_EN) ? 1UL : 0UL;
}

/**
 * @brief Get data register for indirect read/write
 */
static inline uint32_t ll_mpi_read_data(MPI_TypeDef *mpi)
{
    return READ_REG(mpi->DR);
}

static inline void ll_mpi_write_data(MPI_TypeDef *mpi, uint32_t data)
{
    WRITE_REG(mpi->DR, data);
}

/**
 * @brief Set fixed-latency mode (DCR.FIXLAT).
 * @param[in] mpi MPI instance pointer.
 * @param[in] en Non-zero to enable fixed latency, zero otherwise.
 */
static inline void ll_mpi_set_fixed_latency(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->DCR, MPI_DCR_FIXLAT, en ? MPI_DCR_FIXLAT : 0UL);
}

/*==============================================================================
 * Timing Configuration
 *============================================================================*/

/**
 * @brief Configure CS timing parameters (in MPI clock cycles)
 * @param cslmax  CS low max
 * @param cslmin  CS low min
 * @param cshmin  CS high min
 * @param trcmin  TRC min
 */
static inline void ll_mpi_set_cs_timing(MPI_TypeDef *mpi, uint32_t cslmax, uint32_t cslmin,
                                        uint32_t cshmin, uint32_t trcmin)
{
    MODIFY_REG(mpi->DCR, MPI_DCR_CSLMAX,
               MAKE_REG_VAL(cslmax, MPI_DCR_CSLMAX_Msk, MPI_DCR_CSLMAX_Pos));
    MODIFY_REG(mpi->DCR, MPI_DCR_CSLMIN,
               MAKE_REG_VAL(cslmin, MPI_DCR_CSLMIN_Msk, MPI_DCR_CSLMIN_Pos));
    MODIFY_REG(mpi->DCR, MPI_DCR_CSHMIN,
               MAKE_REG_VAL(cshmin, MPI_DCR_CSHMIN_Msk, MPI_DCR_CSHMIN_Pos));
    MODIFY_REG(mpi->DCR, MPI_DCR_TRCMIN,
               MAKE_REG_VAL(trcmin, MPI_DCR_TRCMIN_Msk, MPI_DCR_TRCMIN_Pos));
}

/**
 * @brief Enable/disable the Xccela legacy protocol (DCR.XLEGACY).
 * @param[in] mpi MPI instance pointer.
 * @param[in] en  Non-zero to enable Xccela legacy mode, zero to disable it.
 */
static inline void ll_mpi_set_xlegacy(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->DCR, MPI_DCR_XLEGACY, en ? MPI_DCR_XLEGACY : 0UL);
}

/**
 * @brief Enable DQS (data strobe) for OPI PSRAM
 */
static inline void ll_mpi_enable_dqs(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->DCR, MPI_DCR_DQSE);
}

/**
 * @brief Enable/disable DQS (data strobe) sampling (DCR.DQSE).
 * @param[in] mpi MPI instance pointer.
 * @param[in] en  Non-zero to enable DQS sampling, zero to disable it.
 */
static inline void ll_mpi_set_dqs(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->DCR, MPI_DCR_DQSE, en ? MPI_DCR_DQSE : 0UL);
}

/**
 * @brief Set the row boundary size (DCR.RBSIZE).
 * @param[in] mpi MPI instance pointer.
 * @param[in] size Row boundary size, one of @ref LL_MPI_RB_SIZE.
 */
static inline void ll_mpi_set_row_boundary_size(MPI_TypeDef *mpi, uint32_t size)
{
    MODIFY_REG(mpi->DCR, MPI_DCR_RBSIZE,
               MAKE_REG_VAL(size, MPI_DCR_RBSIZE_Msk, MPI_DCR_RBSIZE_Pos));
}

/*==============================================================================
 * Clock Configuration
 *============================================================================*/

/**
 * @brief Set MPI clock prescaler divider.
 * @param[in] mpi MPI instance pointer.
 * @param[in] div Divider value; MCLK = FCLK / div (div 0 and 1 select /1).
 * @note FLASH clock = MCLK, PSRAM clock = MCLK / 2.
 */
static inline void ll_mpi_set_clock_div(MPI_TypeDef *mpi, uint8_t div)
{
    MODIFY_REG(mpi->PSCLR, MPI_PSCLR_DIV, div);
}

static inline uint32_t ll_mpi_is_busy(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->SR, MPI_SR_BUSY) ? 1UL : 0UL;
}

static inline uint32_t ll_mpi_get_row_boundary_flag(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->SR, MPI_SR_RBXF) ? 1UL : 0UL;
}

static inline uint32_t ll_mpi_get_cs_violation_flag(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->SR, MPI_SR_CSVF) ? 1UL : 0UL;
}

static inline uint32_t ll_mpi_get_status_match_flag(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->SR, MPI_SR_SMF) ? 1UL : 0UL;
}

static inline uint32_t ll_mpi_get_fifo_overflow_flag(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->SR, MPI_SR_FOF) ? 1UL : 0UL;
}

static inline uint32_t ll_mpi_get_fifo_underflow_flag(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->SR, MPI_SR_FUF) ? 1UL : 0UL;
}

/*==============================================================================
 * Status & Interrupt
 *============================================================================*/

static inline uint32_t ll_mpi_get_transfer_complete_flag(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->SR, MPI_SR_TCF) ? 1UL : 0UL;
}

static inline void ll_mpi_clear_row_boundary_flag(MPI_TypeDef *mpi)
{
    WRITE_REG(mpi->SCR, MPI_SCR_RBXFC);
}

static inline void ll_mpi_clear_cs_violation_flag(MPI_TypeDef *mpi)
{
    WRITE_REG(mpi->SCR, MPI_SCR_CSVFC);
}

static inline void ll_mpi_clear_status_match_flag(MPI_TypeDef *mpi)
{
    WRITE_REG(mpi->SCR, MPI_SCR_SMFC);
}

static inline void ll_mpi_clear_fifo_overflow_flag(MPI_TypeDef *mpi)
{
    WRITE_REG(mpi->SCR, MPI_SCR_FOFC);
}

static inline void ll_mpi_clear_fifo_underflow_flag(MPI_TypeDef *mpi)
{
    WRITE_REG(mpi->SCR, MPI_SCR_FUFC);
}

static inline void ll_mpi_clear_transfer_complete_flag(MPI_TypeDef *mpi)
{
    WRITE_REG(mpi->SCR, MPI_SCR_TCFC);
}

/*==============================================================================
 * Command Execution (CS1/CS2)
 *============================================================================*/

/**
 * @brief Execute indirect read/write command on specified CS
 * @param cs    Chip select (1 or 2)
 * @param cmd   Command byte
 * @param addr  Address
 * @param alt   Alternate bytes
 * @param dummy Dummy cycles
 * @param len   Data length in bytes (1..2^20)
 * @note DLR.DLEN stores len-1: the reference manual defines value n as
 *       transferring n+1 bytes (0 means one byte). Invalid chip selects and a
 *       zero length are ignored.
 */
static inline void ll_mpi_set_command(MPI_TypeDef *mpi, uint32_t cs, uint8_t cmd, uint32_t addr,
                                      uint8_t alt, uint8_t dummy, uint32_t len)
{
    if ((cs != LL_MPI_CS_1) && (cs != LL_MPI_CS_2))
    {
        return;
    }
    if (len == 0U)
    {
        return;
    }
    if (cs == LL_MPI_CS_1)
    {
        WRITE_REG(mpi->AR1, addr);
        WRITE_REG(mpi->ABR1, (uint32_t)alt);
        WRITE_REG(mpi->DLR1, MAKE_REG_VAL(len - 1U, MPI_DLR1_DLEN_Msk, MPI_DLR1_DLEN_Pos));
        MODIFY_REG(mpi->CCR1, MPI_CCR1_DCYC,
                   MAKE_REG_VAL(dummy, MPI_CCR1_DCYC_Msk, MPI_CCR1_DCYC_Pos));
        WRITE_REG(mpi->CMDR1, (uint32_t)cmd);
    }
    else
    {
        WRITE_REG(mpi->AR2, addr);
        WRITE_REG(mpi->ABR2, (uint32_t)alt);
        WRITE_REG(mpi->DLR2, MAKE_REG_VAL(len - 1U, MPI_DLR2_DLEN_Msk, MPI_DLR2_DLEN_Pos));
        MODIFY_REG(mpi->CCR2, MPI_CCR2_DCYC,
                   MAKE_REG_VAL(dummy, MPI_CCR2_DCYC_Msk, MPI_CCR2_DCYC_Pos));
        WRITE_REG(mpi->CMDR2, (uint32_t)cmd);
    }
}

/**
 * @brief Set the command byte for a chip select (CMDRx).
 * @param[in] mpi MPI instance pointer.
 * @param[in] cs  Chip select, 1 or 2.
 * @param[in] cmd Command byte.
 */
static inline void ll_mpi_set_command_byte(MPI_TypeDef *mpi, uint32_t cs, uint8_t cmd)
{
    if (cs == LL_MPI_CS_1)
    {
        WRITE_REG(mpi->CMDR1, (uint32_t)cmd);
    }
    else if (cs == LL_MPI_CS_2)
    {
        WRITE_REG(mpi->CMDR2, (uint32_t)cmd);
    }
}

/**
 * @brief Set the address for a chip select (ARx).
 * @param[in] mpi  MPI instance pointer.
 * @param[in] cs   Chip select, 1 or 2.
 * @param[in] addr Address value.
 */
static inline void ll_mpi_set_address(MPI_TypeDef *mpi, uint32_t cs, uint32_t addr)
{
    if (cs == LL_MPI_CS_1)
    {
        WRITE_REG(mpi->AR1, addr);
    }
    else if (cs == LL_MPI_CS_2)
    {
        WRITE_REG(mpi->AR2, addr);
    }
}

/**
 * @brief Set the alternate bytes for a chip select (ABRx).
 * @param[in] mpi MPI instance pointer.
 * @param[in] cs  Chip select, 1 or 2.
 * @param[in] alt Alternate byte value.
 */
static inline void ll_mpi_set_alt_bytes(MPI_TypeDef *mpi, uint32_t cs, uint32_t alt)
{
    if (cs == LL_MPI_CS_1)
    {
        WRITE_REG(mpi->ABR1, alt);
    }
    else if (cs == LL_MPI_CS_2)
    {
        WRITE_REG(mpi->ABR2, alt);
    }
}

/**
 * @brief Set the data length for a chip select (DLRx.DLEN, stores len-1).
 * @note DLR.DLEN stores len-1: the reference manual defines value n as
 *       transferring n+1 bytes (0 means one byte).
 * @param[in] mpi MPI instance pointer.
 * @param[in] cs  Chip select, 1 or 2.
 * @param[in] len Data length in bytes (must be > 0).
 */
static inline void ll_mpi_set_data_length(MPI_TypeDef *mpi, uint32_t cs, uint32_t len)
{
    if (cs == LL_MPI_CS_1)
    {
        MODIFY_REG(mpi->DLR1, MPI_DLR1_DLEN,
                   MAKE_REG_VAL(len - 1U, MPI_DLR1_DLEN_Msk, MPI_DLR1_DLEN_Pos));
    }
    else if (cs == LL_MPI_CS_2)
    {
        MODIFY_REG(mpi->DLR2, MPI_DLR2_DLEN,
                   MAKE_REG_VAL(len - 1U, MPI_DLR2_DLEN_Msk, MPI_DLR2_DLEN_Pos));
    }
}

/**
 * @brief Write the raw communication configuration register (CCR) for a chip select.
 * @param[in] mpi MPI instance pointer.
 * @param[in] cs  Chip select, 1 or 2.
 * @param[in] ccr Raw CCR value (pre-encoded protocol fields).
 */
static inline void ll_mpi_write_command_config(MPI_TypeDef *mpi, uint32_t cs, uint32_t ccr)
{
    if (cs == LL_MPI_CS_1)
    {
        WRITE_REG(mpi->CCR1, ccr);
    }
    else if (cs == LL_MPI_CS_2)
    {
        WRITE_REG(mpi->CCR2, ccr);
    }
}

/**
 * @brief Configure the communication configuration register (CCR) for a chip select.
 * @param[in] mpi MPI instance pointer.
 * @param[in] cs Chip select, 1 or 2.
 * @param[in] imode Instruction mode.
 * @param[in] admode Address mode.
 * @param[in] adsize Address size.
 * @param[in] abmode Alternate byte mode.
 * @param[in] absize Alternate byte size.
 * @param[in] dcyc Dummy cycles.
 * @param[in] dmode Data mode.
 * @param[in] fmode Function mode, 0 for read and 1 for write.
 */
static inline void ll_mpi_set_command_config(MPI_TypeDef *mpi, uint32_t cs, uint32_t imode,
        uint32_t admode, uint32_t adsize, uint32_t abmode,
        uint32_t absize, uint32_t dcyc, uint32_t dmode,
        uint32_t fmode)
{
    if ((cs != LL_MPI_CS_1) && (cs != LL_MPI_CS_2))
    {
        return;
    }
    if (cs == LL_MPI_CS_1)
    {
        WRITE_REG(mpi->CCR1,
                  MAKE_REG_VAL(imode, MPI_CCR1_IMODE_Msk, MPI_CCR1_IMODE_Pos) |
                  MAKE_REG_VAL(admode, MPI_CCR1_ADMODE_Msk, MPI_CCR1_ADMODE_Pos) |
                  MAKE_REG_VAL(adsize, MPI_CCR1_ADSIZE_Msk, MPI_CCR1_ADSIZE_Pos) |
                  MAKE_REG_VAL(abmode, MPI_CCR1_ABMODE_Msk, MPI_CCR1_ABMODE_Pos) |
                  MAKE_REG_VAL(absize, MPI_CCR1_ABSIZE_Msk, MPI_CCR1_ABSIZE_Pos) |
                  MAKE_REG_VAL(dcyc, MPI_CCR1_DCYC_Msk, MPI_CCR1_DCYC_Pos) |
                  MAKE_REG_VAL(dmode, MPI_CCR1_DMODE_Msk, MPI_CCR1_DMODE_Pos) |
                  MAKE_REG_VAL(fmode, MPI_CCR1_FMODE_Msk, MPI_CCR1_FMODE_Pos));
    }
    else
    {
        WRITE_REG(mpi->CCR2,
                  MAKE_REG_VAL(imode, MPI_CCR2_IMODE_Msk, MPI_CCR2_IMODE_Pos) |
                  MAKE_REG_VAL(admode, MPI_CCR2_ADMODE_Msk, MPI_CCR2_ADMODE_Pos) |
                  MAKE_REG_VAL(adsize, MPI_CCR2_ADSIZE_Msk, MPI_CCR2_ADSIZE_Pos) |
                  MAKE_REG_VAL(abmode, MPI_CCR2_ABMODE_Msk, MPI_CCR2_ABMODE_Pos) |
                  MAKE_REG_VAL(absize, MPI_CCR2_ABSIZE_Msk, MPI_CCR2_ABSIZE_Pos) |
                  MAKE_REG_VAL(dcyc, MPI_CCR2_DCYC_Msk, MPI_CCR2_DCYC_Pos) |
                  MAKE_REG_VAL(dmode, MPI_CCR2_DMODE_Msk, MPI_CCR2_DMODE_Pos) |
                  MAKE_REG_VAL(fmode, MPI_CCR2_FMODE_Msk, MPI_CCR2_FMODE_Pos));
    }
}

/**
 * @brief Set the AHB write command (HCMDR.WCMD).
 * @param[in] mpi MPI instance pointer.
 * @param[in] cmd Write command byte.
 */
static inline void ll_mpi_set_ahb_write_command(MPI_TypeDef *mpi, uint8_t cmd)
{
    MODIFY_REG(mpi->HCMDR, MPI_HCMDR_WCMD,
               MAKE_REG_VAL(cmd, MPI_HCMDR_WCMD_Msk, MPI_HCMDR_WCMD_Pos));
}

/**
 * @brief Set the AHB read command (HCMDR.RCMD).
 * @param[in] mpi MPI instance pointer.
 * @param[in] cmd Read command byte.
 */
static inline void ll_mpi_set_ahb_read_command(MPI_TypeDef *mpi, uint8_t cmd)
{
    MODIFY_REG(mpi->HCMDR, MPI_HCMDR_RCMD,
               MAKE_REG_VAL(cmd, MPI_HCMDR_RCMD_Msk, MPI_HCMDR_RCMD_Pos));
}

/**
 * @brief Set the AHB alternate bytes (HRABR).
 * @param[in] mpi MPI instance pointer.
 * @param[in] alt Alternate byte value.
 */
static inline void ll_mpi_set_ahb_alt_bytes(MPI_TypeDef *mpi, uint32_t alt)
{
    WRITE_REG(mpi->HRABR, alt);
}

/*==============================================================================
 * AHB Command Configuration
 *============================================================================*/

/**
 * @brief Set the AHB read command configuration (HRCCR).
 * @param[in] mpi MPI instance pointer.
 * @param[in] ccr Raw HRCCR value.
 */
static inline void ll_mpi_set_ahb_read_config(MPI_TypeDef *mpi, uint32_t ccr)
{
    const uint32_t valid_mask = MPI_HRCCR_DMODE | MPI_HRCCR_DCYC |
                                MPI_HRCCR_ABSIZE | MPI_HRCCR_ABMODE |
                                MPI_HRCCR_ADSIZE | MPI_HRCCR_ADMODE | MPI_HRCCR_IMODE;

    WRITE_REG(mpi->HRCCR, ccr & valid_mask);
}

/**
 * @brief Set the AHB write alternate byte register (HWABR.ABYTE).
 * @param[in] mpi   MPI instance pointer.
 * @param[in] abyte Alternate byte value.
 */
static inline void ll_mpi_set_ahb_write_alt_bytes(MPI_TypeDef *mpi, uint32_t abyte)
{
    WRITE_REG(mpi->HWABR, abyte);
}

/**
 * @brief Set the AHB write command configuration (HWCCR).
 * @param[in] mpi MPI instance pointer.
 * @param[in] ccr Raw HWCCR value.
 */
static inline void ll_mpi_set_ahb_write_config(MPI_TypeDef *mpi, uint32_t ccr)
{
    const uint32_t valid_mask = MPI_HWCCR_DMODE | MPI_HWCCR_DCYC |
                                MPI_HWCCR_ABSIZE | MPI_HWCCR_ABMODE |
                                MPI_HWCCR_ADSIZE | MPI_HWCCR_ADMODE | MPI_HWCCR_IMODE;

    WRITE_REG(mpi->HWCCR, ccr & valid_mask);
}

/*==============================================================================
 * FIFO
 *============================================================================*/

/**
 * @brief Flush FIFO
 */
static inline void ll_mpi_flush_fifo(MPI_TypeDef *mpi)
{
    WRITE_REG(mpi->FIFOCR, LL_MPI_FIFOCR_FLUSH);
}

/**
 * @brief Set the FIFO TX slots (FIFOCR.TXSLOTS).
 * @param[in] mpi   MPI instance pointer.
 * @param[in] slots TX slots value.
 */
static inline void ll_mpi_set_fifo_tx_slots(MPI_TypeDef *mpi, uint32_t slots)
{
    MODIFY_REG(mpi->FIFOCR, MPI_FIFOCR_TXSLOTS,
               MAKE_REG_VAL(slots, MPI_FIFOCR_TXSLOTS_Msk, MPI_FIFOCR_TXSLOTS_Pos));
}

static inline uint32_t ll_mpi_is_tx_fifo_full(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->FIFOCR, MPI_FIFOCR_TXF) ? 1UL : 0UL;
}

static inline uint32_t ll_mpi_is_rx_fifo_empty(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->FIFOCR, MPI_FIFOCR_RXE) ? 1UL : 0UL;
}

static inline void ll_mpi_set_debug_select(MPI_TypeDef *mpi, uint32_t sel)
{
    MODIFY_REG(mpi->MISCR, MPI_MISCR_DBGSEL,
               MAKE_REG_VAL(sel, MPI_MISCR_DBGSEL_Msk, MPI_MISCR_DBGSEL_Pos));
}

static inline void ll_mpi_set_dtr_presample(MPI_TypeDef *mpi, uint32_t en)
{
    MODIFY_REG(mpi->MISCR, MPI_MISCR_DTRPRE, en ? MPI_MISCR_DTRPRE : 0UL);
}

/**
 * @brief Set the SCK clock inversion (MISCR.SCKINV).
 * @param[in] mpi     MPI instance pointer.
 * @param[in] invert Non-zero to invert SCK, zero otherwise.
 */
static inline void ll_mpi_set_sck_invert(MPI_TypeDef *mpi, uint32_t invert)
{
    MODIFY_REG(mpi->MISCR, MPI_MISCR_SCKINV, invert ? MPI_MISCR_SCKINV : 0UL);
}

/*==============================================================================
 * Clock Signal Control (MISCR)
 *============================================================================*/

/**
 * @brief Set the receive clock inversion (MISCR.RXCLKINV).
 * @param[in] mpi     MPI instance pointer.
 * @param[in] invert Non-zero to invert the receive clock, zero otherwise.
 */
static inline void ll_mpi_set_rx_clk_invert(MPI_TypeDef *mpi, uint32_t invert)
{
    MODIFY_REG(mpi->MISCR, MPI_MISCR_RXCLKINV, invert ? MPI_MISCR_RXCLKINV : 0UL);
}

/**
 * @brief Set the DQS delay (MISCR.DQSDLY).
 * @param[in] mpi MPI instance pointer.
 * @param[in] dly DQS delay value.
 */
static inline void ll_mpi_set_dqs_delay(MPI_TypeDef *mpi, uint32_t dly)
{
    MODIFY_REG(mpi->MISCR, MPI_MISCR_DQSDLY,
               MAKE_REG_VAL(dly, MPI_MISCR_DQSDLY_Msk, MPI_MISCR_DQSDLY_Pos));
}

/**
 * @brief Set the SCK delay (MISCR.SCKDLY).
 * @param[in] mpi MPI instance pointer.
 * @param[in] dly SCK delay value.
 */
static inline void ll_mpi_set_sck_delay(MPI_TypeDef *mpi, uint32_t dly)
{
    MODIFY_REG(mpi->MISCR, MPI_MISCR_SCKDLY,
               MAKE_REG_VAL(dly, MPI_MISCR_SCKDLY_Msk, MPI_MISCR_SCKDLY_Pos));
}

static inline void ll_mpi_set_rx_clk_delay(MPI_TypeDef *mpi, uint32_t dly)
{
    MODIFY_REG(mpi->MISCR, MPI_MISCR_RXCLKDLY,
               MAKE_REG_VAL(dly, MPI_MISCR_RXCLKDLY_Msk, MPI_MISCR_RXCLKDLY_Pos));
}

/**
 * @brief Set the CTR starting address (CTRSAR.SA[31:10]).
 * @param[in] mpi  MPI instance pointer.
 * @param[in] addr Starting address (word-aligned; low 10 bits ignored).
 */
static inline void ll_mpi_set_ctr_start_address(MPI_TypeDef *mpi, uint32_t addr)
{
    WRITE_REG(mpi->CTRSAR, addr & MPI_CTRSAR_SA_Msk);
}

/**
 * @brief Set the CTR ending address (CTREAR.EA[31:10]).
 * @param[in] mpi  MPI instance pointer.
 * @param[in] addr Ending address (word-aligned; low 10 bits ignored).
 */
static inline void ll_mpi_set_ctr_end_address(MPI_TypeDef *mpi, uint32_t addr)
{
    WRITE_REG(mpi->CTREAR, addr & MPI_CTREAR_EA_Msk);
}

/**
 * @brief Set the nonce A (NONCEA).
 * @param[in] mpi   MPI instance pointer.
 * @param[in] nonce Nonce A value.
 */
static inline void ll_mpi_set_nonce_a(MPI_TypeDef *mpi, uint32_t nonce)
{
    WRITE_REG(mpi->NONCEA, nonce);
}

/**
 * @brief Set the nonce B (NONCEB).
 * @param[in] mpi   MPI instance pointer.
 * @param[in] nonce Nonce B value.
 */
static inline void ll_mpi_set_nonce_b(MPI_TypeDef *mpi, uint32_t nonce)
{
    WRITE_REG(mpi->NONCEB, nonce);
}

/**
 * @brief Set the address aliasing starting address (AASAR.SA[31:10]).
 * @param[in] mpi  MPI instance pointer.
 * @param[in] addr Starting address of the aliasing area.
 */
static inline void ll_mpi_set_aa_start_address(MPI_TypeDef *mpi, uint32_t addr)
{
    WRITE_REG(mpi->AASAR, addr & MPI_AASAR_SA_Msk);
}

/**
 * @brief Set the address aliasing ending address (AAEAR.EA[31:10]).
 * @param[in] mpi  MPI instance pointer.
 * @param[in] addr Ending address of the aliasing area.
 */
static inline void ll_mpi_set_aa_end_address(MPI_TypeDef *mpi, uint32_t addr)
{
    WRITE_REG(mpi->AAEAR, addr & MPI_AAEAR_EA_Msk);
}

/**
 * @brief Set the address aliasing offset address (AAOAR.OA[31:10]).
 * @param[in] mpi  MPI instance pointer.
 * @param[in] addr Offset applied to addresses inside the aliasing area.
 */
static inline void ll_mpi_set_aa_offset_address(MPI_TypeDef *mpi, uint32_t addr)
{
    WRITE_REG(mpi->AAOAR, addr & MPI_AAOAR_OA_Msk);
}

/**
 * @brief Set the command interval register (CIR).
 * @param[in] mpi MPI instance pointer.
 * @param[in] val Raw CIR value.
 */
static inline void ll_mpi_set_comm_interval(MPI_TypeDef *mpi, uint32_t val)
{
    WRITE_REG(mpi->CIR, val & (MPI_CIR_INTERVAL2 | MPI_CIR_INTERVAL1));
}

static inline void ll_mpi_set_status_match_value(MPI_TypeDef *mpi, uint32_t value)
{
    WRITE_REG(mpi->SMR, value);
}

static inline void ll_mpi_set_status_match_mask(MPI_TypeDef *mpi, uint32_t mask)
{
    WRITE_REG(mpi->SMKR, mask);
}

/**
 * @brief Set the timing register (TIMR).
 * @param[in] mpi MPI instance pointer.
 * @param[in] val Raw TIMR value.
 */
static inline void ll_mpi_set_timing(MPI_TypeDef *mpi, uint32_t val)
{
    WRITE_REG(mpi->TIMR, val);
}

static inline uint32_t ll_mpi_get_watchdog_timeout_flag(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->WDTR, MPI_WDTR_TOF) ? 1UL : 0UL;
}

/*==============================================================================
 * Watchdog
 *============================================================================*/

/**
 * @brief Set the MPI watchdog timer value (WDTR).
 * @param[in] mpi MPI instance pointer.
 * @param[in] val Watchdog timer value.
 */
static inline void ll_mpi_set_watchdog(MPI_TypeDef *mpi, uint32_t val)
{
    WRITE_REG(mpi->WDTR, val & (MPI_WDTR_EN | MPI_WDTR_TIMEOUT));
}

/**
 * @brief Set the prefetch starting address (PRSAR.SA[31:10]).
 * @param[in] mpi  MPI instance pointer.
 * @param[in] addr Starting address of the prefetch area.
 */
static inline void ll_mpi_set_prefetch_start_address(MPI_TypeDef *mpi, uint32_t addr)
{
    WRITE_REG(mpi->PRSAR, addr & MPI_PRSAR_SA_Msk);
}

/**
 * @brief Set the prefetch ending address (PREAR.EA[31:10]).
 * @param[in] mpi  MPI instance pointer.
 * @param[in] addr Ending address of the prefetch area.
 */
static inline void ll_mpi_set_prefetch_end_address(MPI_TypeDef *mpi, uint32_t addr)
{
    WRITE_REG(mpi->PREAR, addr & MPI_PREAR_EA_Msk);
}

/*==============================================================================
 * Calibration
 *============================================================================*/

/**
 * @brief Start MPI delay calibration (CALCR.EN).
 * @param[in] mpi MPI instance pointer.
 */
static inline void ll_mpi_calibration_enable(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CALCR, MPI_CALCR_EN);
}

/**
 * @brief Stop MPI delay calibration (CALCR.EN).
 * @param[in] mpi MPI instance pointer.
 */
static inline void ll_mpi_calibration_disable(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CALCR, MPI_CALCR_EN);
}

/**
 * @brief Check whether delay calibration is complete (CALCR.DONE).
 * @param[in] mpi MPI instance pointer.
 * @return Non-zero when calibration is done, 0 otherwise.
 */
static inline uint32_t ll_mpi_is_calibration_done(MPI_TypeDef *mpi)
{
    return READ_BIT(mpi->CALCR, MPI_CALCR_DONE) != 0UL;
}

/**
 * @brief Read the calibration delay value (CALCR.DELAY).
 * @param[in] mpi MPI instance pointer.
 * @return Calibration delay value.
 */
static inline uint32_t ll_mpi_get_calibration_delay(MPI_TypeDef *mpi)
{
    return GET_REG_VAL2(mpi->CALCR, MPI_CALCR_DELAY);
}

/**
 * @brief Enable the OPI calibration data output (CALDOR.EN).
 * @param[in] mpi MPI instance pointer.
 */
static inline void ll_mpi_calibration_output_enable(MPI_TypeDef *mpi)
{
    SET_BIT(mpi->CALDOR, MPI_CALDOR_EN);
}

/**
 * @brief Disable the OPI calibration data output (CALDOR.EN = 0).
 * @param[in] mpi MPI instance pointer.
 */
static inline void ll_mpi_calibration_output_disable(MPI_TypeDef *mpi)
{
    CLEAR_BIT(mpi->CALDOR, MPI_CALDOR_EN);
}

/**
 * @brief Get the OPI calibration feedback data (CALDOR.DATA).
 * @param[in] mpi MPI instance pointer.
 * @return Calibration data.
 */
static inline uint32_t ll_mpi_get_calibration_data(MPI_TypeDef *mpi)
{
    return GET_REG_VAL2(mpi->CALDOR, MPI_CALDOR_DATA);
}

/**
 * @brief Set the APM32 falling-edge TCPH (APM32CR.TCPHW).
 * @param[in] mpi   MPI instance pointer.
 * @param[in] tcphw TCPH falling value (4 bits).
 */
static inline void ll_mpi_set_apm32_tcphw(MPI_TypeDef *mpi, uint32_t tcphw)
{
    MODIFY_REG(mpi->APM32CR, MPI_APM32CR_TCPHW,
               MAKE_REG_VAL(tcphw, MPI_APM32CR_TCPHW_Msk, MPI_APM32CR_TCPHW_Pos));
}

/**
 * @brief Set the APM32 rising-edge TCPH (APM32CR.TCPHR).
 * @param[in] mpi   MPI instance pointer.
 * @param[in] tcphr TCPH rising value (4 bits).
 */
static inline void ll_mpi_set_apm32_tcphr(MPI_TypeDef *mpi, uint32_t tcphr)
{
    MODIFY_REG(mpi->APM32CR, MPI_APM32CR_TCPHR,
               MAKE_REG_VAL(tcphr, MPI_APM32CR_TCPHR_Msk, MPI_APM32CR_TCPHR_Pos));
}

static inline void ll_mpi_set_loop_command(MPI_TypeDef *mpi, uint32_t loop)
{
    MODIFY_REG(mpi->CR2, MPI_CR2_LOOP,
               MAKE_REG_VAL(loop, MPI_CR2_LOOP_Msk, MPI_CR2_LOOP_Pos));
}

/**
 * @brief Set the OPI output delay of a signal line (OPIDLY_OCR1.SO0..SO7).
 * @param[in] mpi   MPI instance pointer.
 * @param[in] pin   Signal line index, 0..7.
 * @param[in] delay Output delay (4 bits).
 */
static inline void ll_mpi_set_opi_output_delay(MPI_TypeDef *mpi, uint32_t pin, uint32_t delay)
{
    uint32_t msk, pos;

    switch (pin)
    {
    case 0U:
        msk = MPI_OPIDLY_OCR1_SO0_Msk;
        pos = MPI_OPIDLY_OCR1_SO0_Pos;
        break;
    case 1U:
        msk = MPI_OPIDLY_OCR1_SO1_Msk;
        pos = MPI_OPIDLY_OCR1_SO1_Pos;
        break;
    case 2U:
        msk = MPI_OPIDLY_OCR1_SO2_Msk;
        pos = MPI_OPIDLY_OCR1_SO2_Pos;
        break;
    case 3U:
        msk = MPI_OPIDLY_OCR1_SO3_Msk;
        pos = MPI_OPIDLY_OCR1_SO3_Pos;
        break;
    case 4U:
        msk = MPI_OPIDLY_OCR1_SO4_Msk;
        pos = MPI_OPIDLY_OCR1_SO4_Pos;
        break;
    case 5U:
        msk = MPI_OPIDLY_OCR1_SO5_Msk;
        pos = MPI_OPIDLY_OCR1_SO5_Pos;
        break;
    case 6U:
        msk = MPI_OPIDLY_OCR1_SO6_Msk;
        pos = MPI_OPIDLY_OCR1_SO6_Pos;
        break;
    case 7U:
        msk = MPI_OPIDLY_OCR1_SO7_Msk;
        pos = MPI_OPIDLY_OCR1_SO7_Pos;
        break;
    default:
        return;
    }
    MODIFY_REG(mpi->OPIDLY_OCR1, msk, MAKE_REG_VAL(delay, msk, pos));
}

/**
 * @brief Set the OPI input delay of a signal line (OPIDLY_ICR1.SI0..SI7).
 * @param[in] mpi   MPI instance pointer.
 * @param[in] pin   Signal line index, 0..7.
 * @param[in] delay Input delay (4 bits).
 */
static inline void ll_mpi_set_opi_input_delay(MPI_TypeDef *mpi, uint32_t pin, uint32_t delay)
{
    uint32_t msk, pos;

    switch (pin)
    {
    case 0U:
        msk = MPI_OPIDLY_ICR1_SI0_Msk;
        pos = MPI_OPIDLY_ICR1_SI0_Pos;
        break;
    case 1U:
        msk = MPI_OPIDLY_ICR1_SI1_Msk;
        pos = MPI_OPIDLY_ICR1_SI1_Pos;
        break;
    case 2U:
        msk = MPI_OPIDLY_ICR1_SI2_Msk;
        pos = MPI_OPIDLY_ICR1_SI2_Pos;
        break;
    case 3U:
        msk = MPI_OPIDLY_ICR1_SI3_Msk;
        pos = MPI_OPIDLY_ICR1_SI3_Pos;
        break;
    case 4U:
        msk = MPI_OPIDLY_ICR1_SI4_Msk;
        pos = MPI_OPIDLY_ICR1_SI4_Pos;
        break;
    case 5U:
        msk = MPI_OPIDLY_ICR1_SI5_Msk;
        pos = MPI_OPIDLY_ICR1_SI5_Pos;
        break;
    case 6U:
        msk = MPI_OPIDLY_ICR1_SI6_Msk;
        pos = MPI_OPIDLY_ICR1_SI6_Pos;
        break;
    case 7U:
        msk = MPI_OPIDLY_ICR1_SI7_Msk;
        pos = MPI_OPIDLY_ICR1_SI7_Pos;
        break;
    default:
        return;
    }
    MODIFY_REG(mpi->OPIDLY_ICR1, msk, MAKE_REG_VAL(delay, msk, pos));
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_MPI_H */
