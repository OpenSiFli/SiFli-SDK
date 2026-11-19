/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_I2C_H
#define __LL_I2C_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_i2c.h
 * @brief Header-only low-level I2C APIs for SF32LB52x.
 */

/** @defgroup LL_I2C_MODE LL I2C Bus Speed Mode */
/** @{ */
#define LL_I2C_MODE_STANDARD (0x0UL << I2C_CR_MODE_Pos)
#define LL_I2C_MODE_FAST (0x1UL << I2C_CR_MODE_Pos)
#define LL_I2C_MODE_HS_STD_FALLBK (0x2UL << I2C_CR_MODE_Pos)
#define LL_I2C_MODE_HS_FAST_FALLBK (0x3UL << I2C_CR_MODE_Pos)
/** @} */

/** @defgroup LL_I2C_ADDR_DIR LL I2C Address Direction */
/** @{ */
#define LL_I2C_ADDR_DIR_WRITE 0U
#define LL_I2C_ADDR_DIR_READ 1U
/** @} */

/**
 * @brief I2C timing configuration (MODE + DNF + LCR + WCR.CNT).
 */
typedef struct
{
    uint32_t mode; /**< Bus speed mode, use @ref LL_I2C_MODE_STANDARD to
                      @ref LL_I2C_MODE_HS_FAST_FALLBK. */
    uint32_t dnf;  /**< Digital filter value for CR.DNF[14:12], range 0..7. */
    uint32_t
    lcr_slv; /**< Standard-mode low/high timing count for LCR.SLV[8:0]. */
    uint32_t lcr_flv;  /**< Fast/Fast+-mode timing count for LCR.FLV[17:9]. */
    uint32_t lcr_hlvl; /**< High-speed low-period timing count for
                          LCR.HLVL[26:18]. */
    uint32_t lcr_hlvh; /**< High-speed high-period timing count for
                          LCR.HLVH[31:27]. */
    uint32_t wcr_cnt;  /**< Setup/hold timing count for WCR.CNT[7:0]. */
} ll_i2c_timing_config_t;

/**
 * @brief I2C master runtime configuration (SCL output mode + MSD behavior).
 */
typedef struct
{
    uint32_t scl_push_pull; /**< Non-zero to enable push-pull SCL output
                               (CR.SCLPP). */
    uint32_t msd_enable;    /**< Non-zero to enable master stop detected status
                               (CR.MSDE). */
} ll_i2c_master_runtime_config_t;

/**
 * @brief Assert the I2C unit reset (CR.UR).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_assert_unit_reset(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_UR);
}

/**
 * @brief Release the I2C unit reset (CR.UR).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_release_unit_reset(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->CR, I2C_CR_UR);
}

/**
 * @brief Check whether the I2C unit reset is asserted (CR.UR).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when unit reset is asserted.
 */
static inline uint32_t ll_i2c_is_unit_reset_asserted(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->CR, I2C_CR_UR);
}

/**
 * @brief Request a software reset (CR.RSTREQ).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_request_reset(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_RSTREQ);
}

/**
 * @brief Check whether a software reset is requested (CR.RSTREQ).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when RSTREQ is set.
 */
static inline uint32_t ll_i2c_is_reset_requested(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->CR, I2C_CR_RSTREQ);
}

/**
 * @brief Request a reset of the I2C bus state machine (CR.BRGRST).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_request_bus_reset(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_BRGRST);
}

/**
 * @brief Check whether an I2C bus state reset is in progress (CR.BRGRST).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when bus state reset is requested.
 */
static inline uint32_t ll_i2c_is_bus_reset_requested(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->CR, I2C_CR_BRGRST);
}

/**
 * @brief Configure I2C timing related registers.
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] cfg Pointer to timing configuration.
 */
static inline void ll_i2c_config_timing(I2C_TypeDef *I2Cx,
                                        const ll_i2c_timing_config_t *cfg)
{
    uint32_t lcr;

    MODIFY_REG(I2Cx->CR, I2C_CR_MODE | I2C_CR_DNF,
               (cfg->mode & I2C_CR_MODE) |
               ((cfg->dnf << I2C_CR_DNF_Pos) & I2C_CR_DNF));

    lcr = ((cfg->lcr_slv << I2C_LCR_SLV_Pos) & I2C_LCR_SLV) |
          ((cfg->lcr_flv << I2C_LCR_FLV_Pos) & I2C_LCR_FLV) |
          ((cfg->lcr_hlvl << I2C_LCR_HLVL_Pos) & I2C_LCR_HLVL) |
          ((cfg->lcr_hlvh << I2C_LCR_HLVH_Pos) & I2C_LCR_HLVH);
    WRITE_REG(I2Cx->LCR, lcr);

    MODIFY_REG(I2Cx->WCR, I2C_WCR_CNT,
               ((cfg->wcr_cnt << I2C_WCR_CNT_Pos) & I2C_WCR_CNT));
}

/**
 * @brief Enable slave address monitor.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_slave_monitor(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_SLVEN);
}

/**
 * @brief Disable slave address monitor.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_slave_monitor(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->CR, I2C_CR_SLVEN);
}

/**
 * @brief Enable SDA glitch cancellation (CR.SDAGCD = 0).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_sda_glitch_cancellation(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->CR, I2C_CR_SDAGCD);
}

/**
 * @brief Disable SDA glitch cancellation (CR.SDAGCD = 1).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_sda_glitch_cancellation(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_SDAGCD);
}

/**
 * @brief Check whether SDA glitch cancellation is enabled (CR.SDAGCD).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when glitch cancellation is enabled.
 */
static inline uint32_t ll_i2c_is_sda_glitch_cancellation_enabled(I2C_TypeDef *I2Cx)
{
    return (READ_BIT(I2Cx->CR, I2C_CR_SDAGCD) == 0U) ? 1UL : 0UL;
}

/**
 * @brief Configure master runtime behavior.
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] cfg Pointer to master runtime configuration.
 */
static inline void
ll_i2c_config_master_runtime(I2C_TypeDef *I2Cx,
                             const ll_i2c_master_runtime_config_t *cfg)
{
    uint32_t val;

    val = ((cfg->scl_push_pull != 0U) ? I2C_CR_SCLPP : 0U) |
          ((cfg->msd_enable != 0U) ? I2C_CR_MSDE : 0U);
    MODIFY_REG(I2Cx->CR, I2C_CR_SCLPP | I2C_CR_MSDE, val);
}

/**
 * @brief Enable master-slave data enable (CR.MSDE).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_msde(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_MSDE);
}

/**
 * @brief Disable master-slave data enable (CR.MSDE = 0).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_msde(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->CR, I2C_CR_MSDE);
}

/**
 * @brief Enable I2C general call response (CR.GCD = 0).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_general_call(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->CR, I2C_CR_GCD);
}

/**
 * @brief Disable I2C general call response (CR.GCD = 1).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_general_call(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_GCD);
}

/**
 * @brief Check whether I2C general call response is enabled (CR.GCD).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when general call response is enabled.
 */
static inline uint32_t ll_i2c_is_general_call_enabled(I2C_TypeDef *I2Cx)
{
    return (READ_BIT(I2Cx->CR, I2C_CR_GCD) == 0U) ? 1UL : 0UL;
}

/**
 * @brief Configure DMA transfer tail behavior.
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] last_stop Non-zero to send STOP after last DMA transfer.
 * @param[in] last_nack Non-zero to send NACK after last DMA read transfer.
 */
static inline void ll_i2c_config_dma_last(I2C_TypeDef *I2Cx, uint32_t last_stop,
        uint32_t last_nack)
{
    uint32_t val;

    val = ((last_stop != 0U) ? I2C_CR_LASTSTOP : 0U) |
          ((last_nack != 0U) ? I2C_CR_LASTNACK : 0U);
    MODIFY_REG(I2Cx->CR, I2C_CR_LASTSTOP | I2C_CR_LASTNACK, val);
}

/**
 * @brief Enable I2C DMA data transfer.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_dma(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_DMAEN);
}

/**
 * @brief Disable I2C DMA data transfer.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_dma(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->CR, I2C_CR_DMAEN);
}

/**
 * @brief Enable SCL output in master mode.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_scl(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_SCLE);
}

/**
 * @brief Disable SCL output in master mode.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_scl(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->CR, I2C_CR_SCLE);
}

/**
 * @brief Enable I2C unit.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->CR, I2C_CR_IUE);
}

/**
 * @brief Disable I2C unit.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->CR, I2C_CR_IUE);
}

/**
 * @brief Check whether I2C unit is enabled.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when enabled.
 */
static inline uint32_t ll_i2c_is_enabled(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->CR, I2C_CR_IUE);
}

/**
 * @brief Set the speed mode field only (CR.MODE), preserving other CR bits.
 * @param[in] I2Cx  I2C instance pointer.
 * @param[in] mode  Raw MODE field value.
 */
static inline void ll_i2c_set_speed_mode(I2C_TypeDef *I2Cx, uint32_t mode)
{
    MODIFY_REG(I2Cx->CR, I2C_CR_MODE,
               MAKE_REG_VAL(mode, I2C_CR_MODE_Msk, I2C_CR_MODE_Pos));
}

/**
 * @brief Get the speed mode field (CR.MODE).
 * @param[in] I2Cx I2C instance pointer.
 * @return Raw MODE field value.
 */
static inline uint32_t ll_i2c_get_speed_mode(I2C_TypeDef *I2Cx)
{
    return GET_REG_VAL2(I2Cx->CR, I2C_CR_MODE);
}

/**
 * @brief Write a combined START/TB/NACK/STOP command atomically (TCR).
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] tcr  Raw TCR command value.
 */
static inline void ll_i2c_write_tcr(I2C_TypeDef *I2Cx, uint32_t tcr)
{
    WRITE_REG(I2Cx->TCR, tcr);
}

/**
 * @brief Abort DMA transfer.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_abort_dma(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->TCR, I2C_TCR_ABORTDMA);
}

/**
 * @brief Request DMA RX operation.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_request_dma_rx(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->TCR, I2C_TCR_RXREQ);
}

/**
 * @brief Request DMA TX operation.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_request_dma_tx(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->TCR, I2C_TCR_TXREQ);
}

/**
 * @brief Abort current master transfer by issuing STOP immediately.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_master_abort(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->TCR, I2C_TCR_MA);
}

/**
 * @brief Configure next received byte to return ACK.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_set_ack_next(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->TCR, I2C_TCR_NACK);
}

/**
 * @brief Configure next received byte to return NACK.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_set_nack_next(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->TCR, I2C_TCR_NACK);
}

/**
 * @brief Request STOP condition in master mode.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_request_stop(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->TCR, I2C_TCR_STOP);
}

/**
 * @brief Request START condition in master mode.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_request_start(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->TCR, I2C_TCR_START);
}

/**
 * @brief Transmit one data byte (write DBR then request byte transfer).
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] data Byte to transmit.
 */
static inline void ll_i2c_transmit_byte(I2C_TypeDef *I2Cx, uint8_t data)
{
    WRITE_REG(I2Cx->DBR, ((uint32_t)data & I2C_DBR_DATA));
    SET_BIT(I2Cx->TCR, I2C_TCR_TB);
}

/**
 * @brief Request one-byte transfer (TX/RX phase).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_request_byte(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->TCR, I2C_TCR_TB);
}

/**
 * @brief Disable all interrupts by clearing the full IER register.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_all_irqs(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->IER, 0U);
}

/**
 * @brief Enable UF interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_uf(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_UFIE);
}

/**
 * @brief Disable UF interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_uf(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_UFIE);
}

/**
 * @brief Enable OF interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_of(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_OFIE);
}

/**
 * @brief Disable OF interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_of(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_OFIE);
}

/**
 * @brief Enable DMADONE interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_dmadone(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_DMADONEIE);
}

/**
 * @brief Disable DMADONE interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_dmadone(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_DMADONEIE);
}

/**
 * @brief Enable MSD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_msd(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_MSDIE);
}

/**
 * @brief Disable MSD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_msd(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_MSDIE);
}

/**
 * @brief Enable BED interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_bed(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_BEDIE);
}

/**
 * @brief Disable BED interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_bed(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_BEDIE);
}

/**
 * @brief Enable SAD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_sad(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_SADIE);
}

/**
 * @brief Disable SAD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_sad(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_SADIE);
}

/**
 * @brief Enable GCAD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_gcad(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_GCADIE);
}

/**
 * @brief Disable GCAD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_gcad(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_GCADIE);
}

/**
 * @brief Enable RF interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_rf(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_RFIE);
}

/**
 * @brief Disable RF interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_rf(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_RFIE);
}

/**
 * @brief Enable TE interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_te(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_TEIE);
}

/**
 * @brief Disable TE interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_te(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_TEIE);
}

/**
 * @brief Enable ALD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_ald(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_ALDIE);
}

/**
 * @brief Disable ALD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_ald(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_ALDIE);
}

/**
 * @brief Enable SSD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_enable_it_ssd(I2C_TypeDef *I2Cx)
{
    SET_BIT(I2Cx->IER, I2C_IER_SSDIE);
}

/**
 * @brief Disable SSD interrupt.
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_disable_it_ssd(I2C_TypeDef *I2Cx)
{
    CLEAR_BIT(I2Cx->IER, I2C_IER_SSDIE);
}

/**
 * @brief Read the aggregate status register (SR).
 * @param[in] I2Cx I2C instance pointer.
 * @return SR register value.
 */
static inline uint32_t ll_i2c_get_status(I2C_TypeDef *I2Cx)
{
    return READ_REG(I2Cx->SR);
}

/**
 * @brief Clear the latched status flags by writing SR.
 * @param[in] I2Cx  I2C instance pointer.
 * @param[in] flags Status flags to clear.
 */
static inline void ll_i2c_clear_status(I2C_TypeDef *I2Cx, uint32_t flags)
{
    WRITE_REG(I2Cx->SR, flags);
}

/**
 * @brief Check UF flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when UF flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_uf(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_UF);
}

/**
 * @brief Clear UF flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_uf(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_UF);
}

/**
 * @brief Check OF flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when OF flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_of(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_OF);
}

/**
 * @brief Clear OF flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_of(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_OF);
}

/**
 * @brief Check DMADONE flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when DMADONE flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_dmadone(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_DMADONE);
}

/**
 * @brief Clear DMADONE flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_dmadone(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_DMADONE);
}

/**
 * @brief Check MSD flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when MSD flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_msd(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_MSD);
}

/**
 * @brief Clear MSD flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_msd(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_MSD);
}

/**
 * @brief Check EBB flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when an early bus busy condition is detected.
 */
static inline uint32_t ll_i2c_is_active_flag_ebb(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_EBB);
}

/**
 * @brief Check BED flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when BED flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_bed(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_BED);
}

/**
 * @brief Clear BED flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_bed(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_BED);
}

/**
 * @brief Check SAD flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when SAD flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_sad(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_SAD);
}

/**
 * @brief Clear SAD flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_sad(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_SAD);
}

/**
 * @brief Check GCAD flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when a general call address is detected.
 */
static inline uint32_t ll_i2c_is_active_flag_gcad(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_GCAD);
}

/**
 * @brief Clear GCAD flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_gcad(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_GCAD);
}

/**
 * @brief Check RF flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when RF flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_rf(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_RF);
}

/**
 * @brief Clear RF flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_rf(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_RF);
}

/**
 * @brief Check TE flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when TE flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_te(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_TE);
}

/**
 * @brief Clear TE flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_te(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_TE);
}

/**
 * @brief Check ALD flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when ALD flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_ald(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_ALD);
}

/**
 * @brief Clear ALD flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_ald(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_ALD);
}

/**
 * @brief Check SSD flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when SSD flag is set.
 */
static inline uint32_t ll_i2c_is_active_flag_ssd(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_SSD);
}

/**
 * @brief Clear SSD flag (write-one-to-clear).
 * @param[in] I2Cx I2C instance pointer.
 */
static inline void ll_i2c_clear_flag_ssd(I2C_TypeDef *I2Cx)
{
    WRITE_REG(I2Cx->SR, I2C_SR_SSD);
}

/**
 * @brief Check IBB flag (bus busy by other masters).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when bus is busy.
 */
static inline uint32_t ll_i2c_is_active_flag_ibb(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_IBB);
}

/**
 * @brief Check UB flag (local unit busy).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when unit is busy.
 */
static inline uint32_t ll_i2c_is_active_flag_ub(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_UB);
}

/**
 * @brief Get ACK/NACK status of latest byte.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when NACK is observed.
 */
static inline uint32_t ll_i2c_get_ack_status(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_NACK);
}

/**
 * @brief Get current transfer direction from RWM flag.
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero for read direction, zero for write direction.
 */
static inline uint32_t ll_i2c_get_rw_mode(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->SR, I2C_SR_RWM);
}

/**
 * @brief Write 7-bit address with direction into DBR.
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] addr7 7-bit target address.
 * @param[in] dir Address direction bit, use @ref LL_I2C_ADDR_DIR_WRITE or @ref
 * LL_I2C_ADDR_DIR_READ.
 */
static inline void ll_i2c_transmit_address7(I2C_TypeDef *I2Cx, uint32_t addr7,
        uint32_t dir)
{
    WRITE_REG(I2Cx->DBR,
              (((addr7 << 1U) | (dir & LL_I2C_ADDR_DIR_READ)) & I2C_DBR_DATA));
}

/**
 * @brief Receive one data byte from DBR.
 * @param[in] I2Cx I2C instance pointer.
 * @return Received byte.
 */
static inline uint8_t ll_i2c_receive_byte(I2C_TypeDef *I2Cx)
{
    return (uint8_t)(READ_REG(I2Cx->DBR) & I2C_DBR_DATA);
}

/**
 * @brief Write one data byte to DBR without starting a transfer.
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] data Byte to write.
 */
static inline void ll_i2c_write_data(I2C_TypeDef *I2Cx, uint8_t data)
{
    WRITE_REG(I2Cx->DBR, ((uint32_t)data & I2C_DBR_DATA));
}

/**
 * @brief Set slave 7-bit address.
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] addr7 7-bit slave address in bits [6:0].
 */
static inline void ll_i2c_set_slave_address(I2C_TypeDef *I2Cx, uint32_t addr7)
{
    MODIFY_REG(I2Cx->SAR, I2C_SAR_ADDR,
               ((addr7 << I2C_SAR_ADDR_Pos) & I2C_SAR_ADDR));
}

/**
 * @brief Set the WCR divider field (WCR.DIV).
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] div Wait counter divider value, range 0..3.
 */
static inline void ll_i2c_set_wait_divider(I2C_TypeDef *I2Cx, uint32_t div)
{
    MODIFY_REG(I2Cx->WCR, I2C_WCR_DIV,
               MAKE_REG_VAL(div, I2C_WCR_DIV_Msk, I2C_WCR_DIV_Pos));
}

/**
 * @brief Get the WCR divider field (WCR.DIV).
 * @param[in] I2Cx I2C instance pointer.
 * @return Wait counter divider value.
 */
static inline uint32_t ll_i2c_get_wait_divider(I2C_TypeDef *I2Cx)
{
    return GET_REG_VAL2(I2Cx->WCR, I2C_WCR_DIV);
}

/**
 * @brief Set the high-speed hold count 2 (WCR.HSCNT2).
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] count High-speed hold count 2, range 0..31.
 */
static inline void ll_i2c_set_hs_hold_count2(I2C_TypeDef *I2Cx, uint32_t count)
{
    MODIFY_REG(I2Cx->WCR, I2C_WCR_HSCNT2,
               MAKE_REG_VAL(count, I2C_WCR_HSCNT2_Msk, I2C_WCR_HSCNT2_Pos));
}

/**
 * @brief Get the high-speed hold count 2 (WCR.HSCNT2).
 * @param[in] I2Cx I2C instance pointer.
 * @return High-speed hold count 2.
 */
static inline uint32_t ll_i2c_get_hs_hold_count2(I2C_TypeDef *I2Cx)
{
    return GET_REG_VAL2(I2Cx->WCR, I2C_WCR_HSCNT2);
}

/**
 * @brief Set the high-speed hold count 1 (WCR.HSCNT1).
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] count High-speed hold count 1, range 0..31.
 */
static inline void ll_i2c_set_hs_hold_count1(I2C_TypeDef *I2Cx, uint32_t count)
{
    MODIFY_REG(I2Cx->WCR, I2C_WCR_HSCNT1,
               MAKE_REG_VAL(count, I2C_WCR_HSCNT1_Msk, I2C_WCR_HSCNT1_Pos));
}

/**
 * @brief Get the high-speed hold count 1 (WCR.HSCNT1).
 * @param[in] I2Cx I2C instance pointer.
 * @return High-speed hold count 1.
 */
static inline uint32_t ll_i2c_get_hs_hold_count1(I2C_TypeDef *I2Cx)
{
    return GET_REG_VAL2(I2Cx->WCR, I2C_WCR_HSCNT1);
}

/**
 * @brief Set the bus reset SCL cycle count (RCCR.RSTCYC[3:0]).
 * @param[in] I2Cx   I2C instance pointer.
 * @param[in] cycles SCL cycle count during bus reset.
 */
static inline void ll_i2c_set_bus_reset_cycles(I2C_TypeDef *I2Cx, uint32_t cycles)
{
    MODIFY_REG(I2Cx->RCCR, I2C_RCCR_RSTCYC,
               MAKE_REG_VAL(cycles, I2C_RCCR_RSTCYC_Msk, I2C_RCCR_RSTCYC_Pos));
}

/**
 * @brief Read the SCL pin level (BMR.SCL).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when SCL is high.
 */
static inline uint32_t ll_i2c_get_scl_level(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->BMR, I2C_BMR_SCL) ? 1UL : 0UL;
}

/**
 * @brief Read the SDA pin level (BMR.SDA).
 * @param[in] I2Cx I2C instance pointer.
 * @return Non-zero when SDA is high.
 */
static inline uint32_t ll_i2c_get_sda_level(I2C_TypeDef *I2Cx)
{
    return READ_BIT(I2Cx->BMR, I2C_BMR_SDA) ? 1UL : 0UL;
}

/**
 * @brief Set DMA transfer byte count.
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] nbytes Transfer size in bytes, valid range 0..511.
 */
static inline void ll_i2c_set_dma_count(I2C_TypeDef *I2Cx, uint32_t nbytes)
{
    MODIFY_REG(I2Cx->DNR, I2C_DNR_NDT,
               ((nbytes << I2C_DNR_NDT_Pos) & I2C_DNR_NDT));
}

/**
 * @brief Get DMA remaining byte count.
 * @param[in] I2Cx I2C instance pointer.
 * @return Remaining DMA transfer bytes from DNR.NDT.
 */
static inline uint32_t ll_i2c_get_dma_count(I2C_TypeDef *I2Cx)
{
    return ((READ_REG(I2Cx->DNR) & I2C_DNR_NDT) >> I2C_DNR_NDT_Pos);
}

/**
 * @brief Push a byte into the TX FIFO (FIFO.DATA).
 * @param[in] I2Cx I2C instance pointer.
 * @param[in] data Byte to send.
 */
static inline void ll_i2c_write_tx_data(I2C_TypeDef *I2Cx, uint8_t data)
{
    WRITE_REG(I2Cx->FIFO, data);
}

/**
 * @brief Pop a byte from the RX FIFO (FIFO.DATA).
 * @param[in] I2Cx I2C instance pointer.
 * @return Received byte.
 */
static inline uint8_t ll_i2c_read_rx_data(I2C_TypeDef *I2Cx)
{
    return (uint8_t)READ_REG(I2Cx->FIFO);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_I2C_H */
