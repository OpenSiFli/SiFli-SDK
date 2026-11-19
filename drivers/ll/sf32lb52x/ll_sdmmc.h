/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_SDMMC_H
#define __LL_SDMMC_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Disable the Command Complete Signal (CCR.CCS_DISABLE, CE-ATA).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_ccs_disable(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CCR, SD_CCR_CCS_DISABLE);
}

/**
 * @brief Enable the Command Complete Signal (CCR.CCS_DISABLE = 0, CE-ATA).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_ccs_enable(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CCR, SD_CCR_CCS_DISABLE);
}

/**
 * @brief Set the command index (CCR.CMD_INDEX).
 * @param[in] SDx   SD instance pointer.
 * @param[in] index Command index (6 bits).
 */
static inline void ll_sdmmc_set_cmd_index(SD_TypeDef *SDx, uint32_t index)
{
    MODIFY_REG(SDx->CCR, SD_CCR_CMD_INDEX,
               MAKE_REG_VAL(index, SD_CCR_CMD_INDEX_Msk, SD_CCR_CMD_INDEX_Pos));
}

/**
 * @brief Select whether the response is 136-bit long (CCR.CMD_LONG_RSP).
 * @param[in] SDx      SD instance pointer.
 * @param[in] long_rsp Non-zero for a long (136-bit) response.
 */
static inline void ll_sdmmc_set_cmd_long_rsp(SD_TypeDef *SDx, uint32_t long_rsp)
{
    MODIFY_REG(SDx->CCR, SD_CCR_CMD_LONG_RSP,
               long_rsp ? SD_CCR_CMD_LONG_RSP : 0U);
}

/**
 * @brief Select whether the command expects a response (CCR.CMD_HAS_RSP).
 * @param[in] SDx     SD instance pointer.
 * @param[in] has_rsp Non-zero for a response-expected command.
 */
static inline void ll_sdmmc_set_cmd_has_rsp(SD_TypeDef *SDx, uint32_t has_rsp)
{
    MODIFY_REG(SDx->CCR, SD_CCR_CMD_HAS_RSP,
               has_rsp ? SD_CCR_CMD_HAS_RSP : 0U);
}

/**
 * @brief Enable command pending for stop-command scheduling (CCR.CMD_PEND).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_enable_cmd_pend(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CCR, SD_CCR_CMD_PEND);
}

/**
 * @brief Disable command pending (CCR.CMD_PEND = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_disable_cmd_pend(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CCR, SD_CCR_CMD_PEND);
}

/**
 * @brief Enable the TX command output (CCR.CMD_TX_EN).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_enable_cmd_tx(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CCR, SD_CCR_CMD_TX_EN);
}

/**
 * @brief Disable the TX command output (CCR.CMD_TX_EN = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_disable_cmd_tx(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CCR, SD_CCR_CMD_TX_EN);
}

/**
 * @brief Start the command transmission (CCR.CMD_START, auto-cleared).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_start_cmd(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CCR, SD_CCR_CMD_START);
}

/**
 * @file ll_sdmmc.h
 * @brief Header-only low-level SD/MMC controller (SDMMC) APIs for SF32LB52x.
 *
 * The controller drives SD/MMC/eMMC cards over 1/4/8-wire buses, with a
 * command/response engine, block/stream data transfer, optional CE-ATA mode
 * and an AHB cache for read acceleration.
 */

/*==============================================================================
 * Command Path (CCR / CAR / RIR / RARx)
 *============================================================================*/

/**
 * @brief Write the command argument (CAR.CMD_ARG).
 * @param[in] SDx SD instance pointer.
 * @param[in] arg Command argument.
 */
static inline void ll_sdmmc_set_cmd_arg(SD_TypeDef *SDx, uint32_t arg)
{
    WRITE_REG(SDx->CAR, arg);
}

/**
 * @brief Get the response command index (RIR.RSP_INDEX).
 * @param[in] SDx SD instance pointer.
 * @return Response command index (6 bits).
 */
static inline uint32_t ll_sdmmc_get_rsp_index(SD_TypeDef *SDx)
{
    return GET_REG_VAL2(SDx->RIR, SD_RIR_RSP_INDEX);
}

/**
 * @brief Read a response word (RARx).
 * @param[in] SDx SD instance pointer.
 * @param[in] idx  Response word index in range 1..4.
 * @return Response word value.
 */
static inline uint32_t ll_sdmmc_get_rsp_word(SD_TypeDef *SDx, uint32_t idx)
{
    volatile uint32_t *rar = &SDx->RAR1;

    return (idx >= 1U && idx <= 4U) ? READ_REG(rar[idx - 1U]) : 0U;
}

/*==============================================================================
 * Data Path (DCR / DLR / BLR / FDLR / FIFO)
 *============================================================================*/

/**
 * @brief Set the block size (DCR.BLOCK_SIZE, value - 1).
 * @param[in] SDx SD instance pointer.
 * @param[in] size Block size in bytes minus 1 (max 2047).
 */
static inline void ll_sdmmc_set_block_size(SD_TypeDef *SDx, uint32_t size)
{
    MODIFY_REG(SDx->DCR, SD_DCR_BLOCK_SIZE,
               MAKE_REG_VAL(size, SD_DCR_BLOCK_SIZE_Msk, SD_DCR_BLOCK_SIZE_Pos));
}

/**
 * @brief Select the data bus width (DCR.WIRE_MODE).
 * @param[in] SDx   SD instance pointer.
 * @param[in] wires Wire count: 0 = 1-bit, 1 = 4-bit, 2/3 = 8-bit.
 */
static inline void ll_sdmmc_set_wire_mode(SD_TypeDef *SDx, uint32_t wires)
{
    MODIFY_REG(SDx->DCR, SD_DCR_WIRE_MODE,
               MAKE_REG_VAL(wires, SD_DCR_WIRE_MODE_Msk, SD_DCR_WIRE_MODE_Pos));
}

/**
 * @brief Select block or stream data mode (DCR.STREAM_MODE).
 * @param[in] SDx     SD instance pointer.
 * @param[in] stream  Non-zero selects stream mode, zero block mode.
 */
static inline void ll_sdmmc_set_stream_mode(SD_TypeDef *SDx, uint32_t stream)
{
    MODIFY_REG(SDx->DCR, SD_DCR_STREAM_MODE,
               stream ? SD_DCR_STREAM_MODE : 0U);
}

/**
 * @brief Select read or write direction (DCR.R_WN).
 * @param[in] SDx  SD instance pointer.
 * @param[in] read Non-zero for read, zero for write.
 */
static inline void ll_sdmmc_set_dir(SD_TypeDef *SDx, uint32_t read)
{
    MODIFY_REG(SDx->DCR, SD_DCR_R_WN, read ? SD_DCR_R_WN : 0U);
}

/**
 * @brief Enable the data transfer (DCR.TRAN_DATA_EN).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_enable_data_transfer(SD_TypeDef *SDx)
{
    SET_BIT(SDx->DCR, SD_DCR_TRAN_DATA_EN);
}

/**
 * @brief Disable the data transfer (DCR.TRAN_DATA_EN = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_disable_data_transfer(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->DCR, SD_DCR_TRAN_DATA_EN);
}

/**
 * @brief Start the data transfer (DCR.DATA_START, auto-cleared).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_start_data(SD_TypeDef *SDx)
{
    SET_BIT(SDx->DCR, SD_DCR_DATA_START);
}

/**
 * @brief Get the number of blocks transferred (DLR.BLOCK_TRAN_NUM).
 * @param[in] SDx SD instance pointer.
 * @return Transferred block count (16 bits).
 */
static inline uint32_t ll_sdmmc_get_block_tran_num(SD_TypeDef *SDx)
{
    return GET_REG_VAL2(SDx->DLR, SD_DLR_BLOCK_TRAN_NUM);
}

/**
 * @brief Set the data length (DLR.DATA_LEN, value - 1, multiple of block size).
 * @param[in] SDx  SD instance pointer.
 * @param[in] len  Data length in bytes minus 1 (16-bit field, 0x0..0xFFFF;
 *                 see UM52 DLR.DATA_LEN, max transfer 63.5 KB).
 */
static inline void ll_sdmmc_set_data_len(SD_TypeDef *SDx, uint32_t len)
{
    MODIFY_REG(SDx->DLR, SD_DLR_DATA_LEN,
               MAKE_REG_VAL(len, SD_DLR_DATA_LEN_Msk, SD_DLR_DATA_LEN_Pos));
}

/**
 * @brief Set the AHB burst length (BLR.BURST_LEN).
 * @param[in] SDx SD instance pointer.
 * @param[in] len Burst length (14 bits).
 */
static inline void ll_sdmmc_set_burst_len(SD_TypeDef *SDx, uint32_t len)
{
    MODIFY_REG(SDx->BLR, SD_BLR_BURST_LEN,
               MAKE_REG_VAL(len, SD_BLR_BURST_LEN_Msk, SD_BLR_BURST_LEN_Pos));
}

/**
 * @brief Set the FIFO read-valid configuration (FDLR).
 * @param[in] SDx   SD instance pointer.
 * @param[in] valid Valid data count to start reading the FIFO (10 bits).
 * @param[in] en    Non-zero uses the config value, zero uses half the FIFO length.
 */
static inline void ll_sdmmc_set_fifo_rd_valid(SD_TypeDef *SDx, uint32_t valid, uint32_t en)
{
    MODIFY_REG(SDx->FDLR, SD_FDLR_RD_VALID | SD_FDLR_RD_VALID_EN,
               MAKE_REG_VAL(valid, SD_FDLR_RD_VALID_Msk, SD_FDLR_RD_VALID_Pos) |
               (en ? SD_FDLR_RD_VALID_EN : 0U));
}

/**
 * @brief Write one word to the data FIFO (FIFO).
 * @param[in] SDx  SD instance pointer.
 * @param[in] data Word to write.
 */
static inline void ll_sdmmc_write_fifo(SD_TypeDef *SDx, uint32_t data)
{
    WRITE_REG(SDx->FIFO, data);
}

/**
 * @brief Read one word from the data FIFO (FIFO).
 * @param[in] SDx SD instance pointer.
 * @return Data word read from the FIFO.
 */
static inline uint32_t ll_sdmmc_read_fifo(SD_TypeDef *SDx)
{
    return READ_REG(SDx->FIFO);
}

/*==============================================================================
 * Status & Interrupts (SR / IER)
 *============================================================================*/

/* Missing combined clearable-bit mask (not in regs/sd.h) defined here as
 * LL-local; removed once the regs header is updated upstream. */
#define LL_SDMMC_SR_CLEARABLE                                                                    \
    (SD_SR_CMD_DONE | SD_SR_CMD_RSP_CRC | SD_SR_CMD_TIMEOUT | SD_SR_DATA_DONE |               \
     SD_SR_DATA_CRC | SD_SR_DATA_TIMEOUT | SD_SR_STARTBIT_ERROR | SD_SR_FIFO_UNDERRUN |       \
     SD_SR_FIFO_OVERRUN | SD_SR_CMD_SENT | SD_SR_CARD_EXIT_INT | SD_SR_CARD_REMOVE |          \
     SD_SR_SDIO | SD_SR_CACHE_ERR)

/**
 * @brief Get the raw status register (SR).
 * @param[in] SDx SD instance pointer.
 * @return SR register value.
 */
static inline uint32_t ll_sdmmc_get_status(SD_TypeDef *SDx)
{
    return READ_REG(SDx->SR);
}

/**
 * @brief Clear rw1c status flags (SR), preserving read-only bits.
 * @param[in] SDx    SD instance pointer.
 * @param[in] status Status flags to clear (write 1).
 */
static inline void ll_sdmmc_clear_status(SD_TypeDef *SDx, uint32_t status)
{
    WRITE_REG(SDx->SR, status & LL_SDMMC_SR_CLEARABLE);
}

/**
 * @brief Check the card-exist status (SR.CARD_EXIT).
 * @param[in] SDx SD instance pointer.
 * @return Non-zero when a card is present.
 */
static inline uint32_t ll_sdmmc_is_card_present(SD_TypeDef *SDx)
{
    return READ_BIT(SDx->SR, SD_SR_CARD_EXIT);
}

/**
 * @brief Check the data busy flag (SR.DATA_BUSY).
 * @param[in] SDx SD instance pointer.
 * @return Non-zero while a data transfer is in progress.
 */
static inline uint32_t ll_sdmmc_is_data_busy(SD_TypeDef *SDx)
{
    return READ_BIT(SDx->SR, SD_SR_DATA_BUSY);
}

/**
 * @brief Check the command busy flag (SR.CMD_BUSY).
 * @param[in] SDx SD instance pointer.
 * @return Non-zero while a command is in progress.
 */
static inline uint32_t ll_sdmmc_is_cmd_busy(SD_TypeDef *SDx)
{
    return READ_BIT(SDx->SR, SD_SR_CMD_BUSY);
}

/**
 * @brief Enable an interrupt mask (IER, 1 = enabled).
 * @param[in] SDx  SD instance pointer.
 * @param[in] mask Mask bits to enable.
 */
static inline void ll_sdmmc_enable_irq(SD_TypeDef *SDx, uint32_t mask)
{
    SET_BIT(SDx->IER, mask);
}

/**
 * @brief Disable an interrupt mask (IER, 0 = masked).
 * @param[in] SDx  SD instance pointer.
 * @param[in] mask Mask bits to disable.
 */
static inline void ll_sdmmc_disable_irq(SD_TypeDef *SDx, uint32_t mask)
{
    CLEAR_BIT(SDx->IER, mask);
}

/** @defgroup LL_SDMMC_IRQ_MASK SDMMC Interrupt Mask Bits (IER) */
/** @{ */
#define LL_SDMMC_IRQ_CMD_DONE         SD_IER_CMD_DONE_MASK
#define LL_SDMMC_IRQ_CMD_RSP_CRC      SD_IER_CMD_RSP_CRC_MASK
#define LL_SDMMC_IRQ_CMD_TIMEOUT      SD_IER_CMD_TIMEOUT_MASK
#define LL_SDMMC_IRQ_DATA_DONE        SD_IER_DATA_DONE_MASK
#define LL_SDMMC_IRQ_DATA_CRC         SD_IER_DATA_CRC_MASK
#define LL_SDMMC_IRQ_DATA_TIMEOUT     SD_IER_DATA_TIMEOUT_MASK
#define LL_SDMMC_IRQ_STARTBIT_ERROR   SD_IER_STARTBIT_ERROR_MASK
#define LL_SDMMC_IRQ_FIFO_UNDERRUN    SD_IER_FIFO_UNDERRUN_MASK
#define LL_SDMMC_IRQ_FIFO_OVERRUN     SD_IER_FIFO_OVERRUN_MASK
#define LL_SDMMC_IRQ_CMD_SENT         SD_IER_CMD_SENT_MASK
#define LL_SDMMC_IRQ_CARD_EXIT        SD_IER_CARD_EXIT_INT_MASK
#define LL_SDMMC_IRQ_CARD_REMOVE      SD_IER_CARD_REMOVE_INT_MASK
#define LL_SDMMC_IRQ_SDIO             SD_IER_SDIO_MASK
#define LL_SDMMC_IRQ_CACHE_ERR        SD_IER_CACHE_ERR_INT_MASK
/*==============================================================================
 * Debug & Misc (TOR / DBGR1 / DBGR2 / DSR)
 *============================================================================*/

/**
 * @brief Set the response/data timeout count (TOR.TIMEOUT_CNT).
 * @param[in] SDx    SD instance pointer.
 * @param[in] count  Timeout count (decremented under the card clock).
 */
static inline void ll_sdmmc_set_timeout(SD_TypeDef *SDx, uint32_t count)
{
    WRITE_REG(SDx->TOR, count);
}

/** @} */

/**
 * @brief Get the interrupt mask register (IER).
 * @param[in] SDx SD instance pointer.
 * @return IER register value.
 */
static inline uint32_t ll_sdmmc_get_irq_mask(SD_TypeDef *SDx)
{
    return READ_REG(SDx->IER);
}

/*==============================================================================
 * Clock & Power (CLKCR / PCR / CDCR)
 *============================================================================*/

/**
 * @brief Set the card clock divider (CLKCR.DIV).
 * @param[in] SDx SD instance pointer.
 * @param[in] div Clock divider (13 bits); card clock = clock / (div + 1).
 */
static inline void ll_sdmmc_set_clock_div(SD_TypeDef *SDx, uint32_t div)
{
    MODIFY_REG(SDx->CLKCR, SD_CLKCR_DIV,
               MAKE_REG_VAL(div, SD_CLKCR_DIV_Msk, SD_CLKCR_DIV_Pos));
}

/**
 * @brief Select the RX sample clock delay (CLKCR.CLK_TUNE_SEL).
 * @param[in] SDx  SD instance pointer.
 * @param[in] sel  Delay level (2 bits): 0 none, 1 ~1.5ns, 2 ~3ns, 3 ~5ns.
 */
static inline void ll_sdmmc_set_clock_tune(SD_TypeDef *SDx, uint32_t sel)
{
    MODIFY_REG(SDx->CLKCR, SD_CLKCR_CLK_TUNE_SEL,
               MAKE_REG_VAL(sel, SD_CLKCR_CLK_TUNE_SEL_Msk, SD_CLKCR_CLK_TUNE_SEL_Pos));
}

/**
 * @brief Enable void-FIFO-error protection (CLKCR.VOID_FIFO_ERROR).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_void_fifo_error_enable(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CLKCR, SD_CLKCR_VOID_FIFO_ERROR);
}

/**
 * @brief Disable void-FIFO-error protection (CLKCR.VOID_FIFO_ERROR = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_void_fifo_error_disable(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CLKCR, SD_CLKCR_VOID_FIFO_ERROR);
}

/**
 * @brief Stop the SD card clock (CLKCR.STOP_CLK).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_stop_clock(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CLKCR, SD_CLKCR_STOP_CLK);
}

/**
 * @brief Let the SD card clock run (CLKCR.STOP_CLK = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_start_clock(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CLKCR, SD_CLKCR_STOP_CLK);
}

/**
 * @brief Enable the ROD (receiver-on-die) control (PCR.ROD).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_enable_rod(SD_TypeDef *SDx)
{
    SET_BIT(SDx->PCR, SD_PCR_ROD);
}

/**
 * @brief Disable the ROD control (PCR.ROD = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_disable_rod(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->PCR, SD_PCR_ROD);
}

/**
 * @brief Select open-drain output (PCR.ODRAIN).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_set_open_drain(SD_TypeDef *SDx)
{
    SET_BIT(SDx->PCR, SD_PCR_ODRAIN);
}

/**
 * @brief Select push-pull output (PCR.ODRAIN = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_clear_open_drain(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->PCR, SD_PCR_ODRAIN);
}

/**
 * @brief Set the output voltage control (PCR.VOLTAGE).
 * @param[in] SDx  SD instance pointer.
 * @param[in] volt Voltage control value (8 bits).
 */
static inline void ll_sdmmc_set_voltage(SD_TypeDef *SDx, uint32_t volt)
{
    MODIFY_REG(SDx->PCR, SD_PCR_VOLTAGE,
               MAKE_REG_VAL(volt, SD_PCR_VOLTAGE_Msk, SD_PCR_VOLTAGE_Pos));
}

/**
 * @brief Power the card on (PCR.PWR).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_power_on(SD_TypeDef *SDx)
{
    MODIFY_REG(SDx->PCR, SD_PCR_PWR,
               MAKE_REG_VAL(1U, SD_PCR_PWR_Msk, SD_PCR_PWR_Pos));
}

/**
 * @brief Power the card off (PCR.PWR = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_power_off(SD_TypeDef *SDx)
{
    MODIFY_REG(SDx->PCR, SD_PCR_PWR, 0U);
}

/**
 * @brief Set the input/output sample timing (CDR.ITIMING/OTIMING).
 * @param[in] SDx      SD instance pointer.
 * @param[in] itiming  Input timing value (13 bits).
 * @param[in] otiming  Output timing value (13 bits).
 */
static inline void ll_sdmmc_set_timing(SD_TypeDef *SDx, uint32_t itiming, uint32_t otiming)
{
    MODIFY_REG(SDx->CDR, SD_CDR_ITIMING | SD_CDR_OTIMING,
               MAKE_REG_VAL(itiming, SD_CDR_ITIMING_Msk, SD_CDR_ITIMING_Pos) |
               MAKE_REG_VAL(otiming, SD_CDR_OTIMING_Msk, SD_CDR_OTIMING_Pos));
}

/**
 * @brief Set the CMD line drive mode (CDR.CMD_OD).
 * @param[in] SDx  SD instance pointer.
 * @param[in] od   Non-zero selects open-drain (eMMC), zero push-pull.
 */
static inline void ll_sdmmc_set_cmd_od(SD_TypeDef *SDx, uint32_t od)
{
    MODIFY_REG(SDx->CDR, SD_CDR_CMD_OD, od ? SD_CDR_CMD_OD : 0U);
}

/**
 * @brief Set the card-detect valid level (CDR.CD_HVALID).
 * @param[in] SDx   SD instance pointer.
 * @param[in] high  Non-zero means a high level indicates a card is present.
 */
static inline void ll_sdmmc_set_cd_valid_level(SD_TypeDef *SDx, uint32_t high)
{
    MODIFY_REG(SDx->CDR, SD_CDR_CD_HVALID, high ? SD_CDR_CD_HVALID : 0U);
}

/**
 * @brief Enable card detection (CDR.EN_CD).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_enable_card_detect(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CDR, SD_CDR_EN_CD);
}

/**
 * @brief Disable card detection (CDR.EN_CD = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_disable_card_detect(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CDR, SD_CDR_EN_CD);
}

/**
 * @brief Select the output timing source (CDR.OTIMING_SEL).
 * @param[in] SDx  SD instance pointer.
 * @param[in] sel  Non-zero uses the OTIMING register, zero negative edge.
 */
static inline void ll_sdmmc_set_otiming_sel(SD_TypeDef *SDx, uint32_t sel)
{
    MODIFY_REG(SDx->CDR, SD_CDR_OTIMING_SEL, sel ? SD_CDR_OTIMING_SEL : 0U);
}

/**
 * @brief Select the input sample timing source (CDR.ITIMING_SEL).
 * @param[in] SDx  SD instance pointer.
 * @param[in] sel  Non-zero uses the ITIMING register, zero positive edge.
 */
static inline void ll_sdmmc_set_itiming_sel(SD_TypeDef *SDx, uint32_t sel)
{
    MODIFY_REG(SDx->CDR, SD_CDR_ITIMING_SEL, sel ? SD_CDR_ITIMING_SEL : 0U);
}

/**
 * @brief Use SD_DATA[3] for card detection (CDR.SD_DATA3_CD).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_use_data3_cd(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CDR, SD_CDR_SD_DATA3_CD);
}

/**
 * @brief Use the dedicated SD_CDETECT pin for detection (CDR.SD_DATA3_CD = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_use_cd_pin(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CDR, SD_CDR_SD_DATA3_CD);
}

/**
 * @brief Get the data state machine status (DBGR1.DATA_ST).
 * @param[in] SDx SD instance pointer.
 * @return Data state (15 bits).
 */
static inline uint32_t ll_sdmmc_get_data_state(SD_TypeDef *SDx)
{
    return GET_REG_VAL2(SDx->DBGR1, SD_DBGR1_DATA_ST);
}

/**
 * @brief Get the command state machine status (DBGR1.CMD_ST).
 * @param[in] SDx SD instance pointer.
 * @return Command state (15 bits).
 */
static inline uint32_t ll_sdmmc_get_cmd_state(SD_TypeDef *SDx)
{
    return GET_REG_VAL2(SDx->DBGR1, SD_DBGR1_CMD_ST);
}

/**
 * @brief Set the debug select (DBGR2.DBG_SEL).
 * @param[in] SDx  SD instance pointer.
 * @param[in] sel  Debug select (2 bits).
 */
static inline void ll_sdmmc_set_dbg_sel(SD_TypeDef *SDx, uint32_t sel)
{
    MODIFY_REG(SDx->DBGR2, SD_DBGR2_DBG_SEL,
               MAKE_REG_VAL(sel, SD_DBGR2_DBG_SEL_Msk, SD_DBGR2_DBG_SEL_Pos));
}

/**
 * @brief Get the valid data counter (DBGR2.VALID_DATA_COU).
 * @param[in] SDx SD instance pointer.
 * @return Valid data counter (10 bits).
 */
static inline uint32_t ll_sdmmc_get_valid_data_count(SD_TypeDef *SDx)
{
    return GET_REG_VAL2(SDx->DBGR2, SD_DBGR2_VALID_DATA_COU);
}

/**
 * @brief Get the host word counter (DBGR2.HOST_WORD_COUNTER).
 * @param[in] SDx SD instance pointer.
 * @return Host word counter (14 bits).
 */
static inline uint32_t ll_sdmmc_get_host_word_count(SD_TypeDef *SDx)
{
    return GET_REG_VAL2(SDx->DBGR2, SD_DBGR2_HOST_WORD_COUNTER);
}

/**
 * @brief Enable 4-wire SDIO multi-block interrupts (CEATA.SDIO_4WIRES_MULTI_IRQ).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_sdio_4wires_multi_irq_enable(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CEATA, SD_CEATA_SDIO_4WIRES_MULTI_IRQ);
}

/**
 * @brief Disable 4-wire SDIO multi-block interrupts (CEATA.SDIO_4WIRES_MULTI_IRQ).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_sdio_4wires_multi_irq_disable(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CEATA, SD_CEATA_SDIO_4WIRES_MULTI_IRQ);
}

/**
 * @brief Enable 4-wire SDIO single-block interrupts (CEATA.SDIO_4WIRES_IRQ).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_sdio_4wires_irq_enable(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CEATA, SD_CEATA_SDIO_4WIRES_IRQ);
}

/**
 * @brief Disable 4-wire SDIO single-block interrupts (CEATA.SDIO_4WIRES_IRQ).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_sdio_4wires_irq_disable(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CEATA, SD_CEATA_SDIO_4WIRES_IRQ);
}

/**
 * @brief Enable SDIO card interrupts (CEATA.ENABLE_SDIO_IRQ).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_enable_sdio_irq(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CEATA, SD_CEATA_ENABLE_SDIO_IRQ);
}

/**
 * @brief Disable SDIO card interrupts (CEATA.ENABLE_SDIO_IRQ = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_disable_sdio_irq(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CEATA, SD_CEATA_ENABLE_SDIO_IRQ);
}

/*==============================================================================
 * CE-ATA / SDIO Mode (CEATA)
 *============================================================================*/

/**
 * @brief Select the card type (CEATA.ATA_MODE).
 * @param[in] SDx  SD instance pointer.
 * @param[in] ata  Non-zero selects CE-ATA device mode, zero SD card.
 */
static inline void ll_sdmmc_set_card_type(SD_TypeDef *SDx, uint32_t ata)
{
    MODIFY_REG(SDx->CEATA, SD_CEATA_ATA_MODE, ata ? SD_CEATA_ATA_MODE : 0U);
}

/**
 * @brief Get the SD data pad input status (DSR.SD_DATA_I_LL).
 * @param[in] SDx SD instance pointer.
 * @return Data pad input levels (8 bits).
 */
static inline uint32_t ll_sdmmc_get_data_pad_status(SD_TypeDef *SDx)
{
    return GET_REG_VAL2(SDx->DSR, SD_DSR_SD_DATA_I_LL);
}

/**
 * @brief Set the SD_CLK duty configuration (CDCR.CLK_CONFIG).
 * @param[in] SDx    SD instance pointer.
 * @param[in] half   Non-zero gives 50% duty cycle, zero a 1-pclk high level.
 */
static inline void ll_sdmmc_set_clk_config(SD_TypeDef *SDx, uint32_t half)
{
    MODIFY_REG(SDx->CDCR, SD_CDCR_CLK_CONFIG, half ? SD_CDCR_CLK_CONFIG : 0U);
}

/**
 * @brief Flush the AHB read cache (CASR.CACHE_FLUSH, rw1s).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_flush(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CASR, SD_CASR_CACHE_FLUSH);
}

/**
 * @brief Check the cache busy flag (CASR.CACHE_BUSY).
 * @param[in] SDx SD instance pointer.
 * @return Non-zero while the cache is working.
 */
static inline uint32_t ll_sdmmc_is_cache_busy(SD_TypeDef *SDx)
{
    return READ_BIT(SDx->CASR, SD_CASR_CACHE_BUSY);
}

/**
 * @brief Clear the SD busy flag (CASR.SD_BUSY, rw1c).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_clear_sd_busy(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CASR, SD_CASR_SD_BUSY);
}

/**
 * @brief Request a normal SD access (CASR.SD_REQ, rw1s).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_request_sd_access(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CASR, SD_CASR_SD_REQ);
}

/*==============================================================================
 * AHB Cache (CACR / CACNT / CAOFF / CASR)
 *============================================================================*/

/**
 * @brief Enable the AHB read cache (CACR.CACHE_EN).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_enable(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CACR, SD_CACR_CACHE_EN);
}

/**
 * @brief Disable the AHB read cache (CACR.CACHE_EN = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_disable(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CACR, SD_CACR_CACHE_EN);
}

/**
 * @brief Enable AHB read timeout recovery (CACR.CACHE_TO_EN).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_to_enable(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CACR, SD_CACR_CACHE_TO_EN);
}

/**
 * @brief Disable AHB read timeout recovery (CACR.CACHE_TO_EN = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_to_disable(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CACR, SD_CACR_CACHE_TO_EN);
}

/**
 * @brief Force the cache to finish reads before new fetches (CACR.CACHE_FORCE_READ).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_force_read(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CACR, SD_CACR_CACHE_FORCE_READ);
}

/**
 * @brief Disable forced cache read completion (CACR.CACHE_FORCE_READ = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_clear_force_read(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CACR, SD_CACR_CACHE_FORCE_READ);
}

/**
 * @brief Select the cache card size threshold (CACR.CACHE_SDSC).
 * @param[in] SDx  SD instance pointer.
 * @param[in] small Non-zero for cards <= 2 GB (byte addressing), zero for larger.
 */
static inline void ll_sdmmc_cache_set_sdsc(SD_TypeDef *SDx, uint32_t small)
{
    MODIFY_REG(SDx->CACR, SD_CACR_CACHE_SDSC, small ? SD_CACR_CACHE_SDSC : 0U);
}

/**
 * @brief Enable CRC checking of cached reads (CACR.CACHE_NOCRC = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_crc_enable(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CACR, SD_CACR_CACHE_NOCRC);
}

/**
 * @brief Disable CRC checking of cached reads (CACR.CACHE_NOCRC).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_crc_disable(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CACR, SD_CACR_CACHE_NOCRC);
}

/**
 * @brief Generate AHB error responses on cache errors (CACR.CACHE_HRESP).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_hresp_enable(SD_TypeDef *SDx)
{
    SET_BIT(SDx->CACR, SD_CACR_CACHE_HRESP);
}

/**
 * @brief Suppress AHB error responses on cache errors (CACR.CACHE_HRESP = 0).
 * @param[in] SDx SD instance pointer.
 */
static inline void ll_sdmmc_cache_hresp_disable(SD_TypeDef *SDx)
{
    CLEAR_BIT(SDx->CACR, SD_CACR_CACHE_HRESP);
}

/**
 * @brief Set the cache prefetch depth (CACR.CACHE_PREF_BLOCK).
 * @param[in] SDx   SD instance pointer.
 * @param[in] depth Prefetch depth in blocks (4 bits).
 */
static inline void ll_sdmmc_cache_set_pref_depth(SD_TypeDef *SDx, uint32_t depth)
{
    MODIFY_REG(SDx->CACR, SD_CACR_CACHE_PREF_BLOCK,
               MAKE_REG_VAL(depth, SD_CACR_CACHE_PREF_BLOCK_Msk,
                            SD_CACR_CACHE_PREF_BLOCK_Pos));
}

/**
 * @brief Set the cache depth (CACR.CACHE_BLOCK).
 * @param[in] SDx   SD instance pointer.
 * @param[in] depth Cache depth in blocks (3 bits).
 */
static inline void ll_sdmmc_cache_set_depth(SD_TypeDef *SDx, uint32_t depth)
{
    MODIFY_REG(SDx->CACR, SD_CACR_CACHE_BLOCK,
               MAKE_REG_VAL(depth, SD_CACR_CACHE_BLOCK_Msk, SD_CACR_CACHE_BLOCK_Pos));
}

/**
 * @brief Set the cache stop command (CACR.STOP_INDEX, default CMD12).
 * @param[in] SDx      SD instance pointer.
 * @param[in] index    Stop command index (6 bits).
 * @param[in] has_rsp  Non-zero if the command expects a response.
 * @param[in] long_rsp Non-zero if the response is 136-bit long.
 */
static inline void ll_sdmmc_cache_set_stop_cmd(SD_TypeDef *SDx, uint32_t index,
        uint32_t has_rsp, uint32_t long_rsp)
{
    MODIFY_REG(SDx->CACR,
               SD_CACR_STOP_INDEX | SD_CACR_STOP_HAS_RSP | SD_CACR_STOP_LONG_RSP,
               MAKE_REG_VAL(index, SD_CACR_STOP_INDEX_Msk, SD_CACR_STOP_INDEX_Pos) |
               (has_rsp ? SD_CACR_STOP_HAS_RSP : 0U) |
               (long_rsp ? SD_CACR_STOP_LONG_RSP : 0U));
}

/**
 * @brief Set the cache read command (CACR.READ_INDEX, default CMD18).
 * @param[in] SDx      SD instance pointer.
 * @param[in] index    Read command index (6 bits).
 * @param[in] has_rsp  Non-zero if the command expects a response.
 * @param[in] long_rsp Non-zero if the response is 136-bit long.
 */
static inline void ll_sdmmc_cache_set_read_cmd(SD_TypeDef *SDx, uint32_t index,
        uint32_t has_rsp, uint32_t long_rsp)
{
    MODIFY_REG(SDx->CACR,
               SD_CACR_READ_INDEX | SD_CACR_READ_HAS_RSP | SD_CACR_READ_LONG_RSP,
               MAKE_REG_VAL(index, SD_CACR_READ_INDEX_Msk, SD_CACR_READ_INDEX_Pos) |
               (has_rsp ? SD_CACR_READ_HAS_RSP : 0U) |
               (long_rsp ? SD_CACR_READ_LONG_RSP : 0U));
}

/**
 * @brief Set the cache timeout counter (CACNT.CACHE_TOR).
 * @param[in] SDx SD instance pointer.
 * @param[in] tor Timeout count for AHB reads (16 bits).
 */
static inline void ll_sdmmc_cache_set_tor(SD_TypeDef *SDx, uint32_t tor)
{
    MODIFY_REG(SDx->CACNT, SD_CACNT_CACHE_TOR,
               MAKE_REG_VAL(tor, SD_CACNT_CACHE_TOR_Msk, SD_CACNT_CACHE_TOR_Pos));
}

/**
 * @brief Set the cache data-command interval (CACNT.CACHE_NDC).
 * @param[in] SDx SD instance pointer.
 * @param[in] ndc Data-command interval in HCLK cycles (8 bits).
 */
static inline void ll_sdmmc_cache_set_ndc(SD_TypeDef *SDx, uint32_t ndc)
{
    MODIFY_REG(SDx->CACNT, SD_CACNT_CACHE_NDC,
               MAKE_REG_VAL(ndc, SD_CACNT_CACHE_NDC_Msk, SD_CACNT_CACHE_NDC_Pos));
}

/**
 * @brief Set the cache command-command interval (CACNT.CACHE_NCC).
 * @param[in] SDx SD instance pointer.
 * @param[in] ncc Command-command interval in HCLK cycles (8 bits).
 */
static inline void ll_sdmmc_cache_set_ncc(SD_TypeDef *SDx, uint32_t ncc)
{
    MODIFY_REG(SDx->CACNT, SD_CACNT_CACHE_NCC,
               MAKE_REG_VAL(ncc, SD_CACNT_CACHE_NCC_Msk, SD_CACNT_CACHE_NCC_Pos));
}

/**
 * @brief Set the cache AHB offset (CAOFF.CACHE_OFFSET).
 * @param[in] SDx     SD instance pointer.
 * @param[in] offset  Offset mapping AHB addresses to SD addresses.
 */
static inline void ll_sdmmc_cache_set_offset(SD_TypeDef *SDx, uint32_t offset)
{
    WRITE_REG(SDx->CAOFF, offset);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_SDMMC_H */
