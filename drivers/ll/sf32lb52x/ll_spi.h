/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_SPI_H
#define __LL_SPI_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_spi.h
 * @brief Header-only low-level SPI APIs for SF32LB52x.
 */

/** @defgroup LL_SPI_PROTOCOL LL SPI Protocol Format */
/** @{ */
#define LL_SPI_PROTOCOL_SPI (0x0UL << SPI_TOP_CTRL_FRF_Pos)
#define LL_SPI_PROTOCOL_TI_SSP (0x1UL << SPI_TOP_CTRL_FRF_Pos)
#define LL_SPI_PROTOCOL_MICROWIRE (0x2UL << SPI_TOP_CTRL_FRF_Pos)
/** @} */

/** @defgroup LL_SPI_CPOL LL SPI Clock Polarity */
/** @{ */
#define LL_SPI_CPOL_LOW 0x00000000U
#define LL_SPI_CPOL_HIGH SPI_TOP_CTRL_SPO
/** @} */

/** @defgroup LL_SPI_CPHA LL SPI Clock Phase */
/** @{ */
#define LL_SPI_CPHA_1EDGE 0x00000000U
#define LL_SPI_CPHA_2EDGE SPI_TOP_CTRL_SPH
/** @} */

/** @defgroup LL_SPI_ROLE LL SPI Frame/Clock Role */
/** @{ */
#define LL_SPI_FRAME_MASTER 0x00000000U
#define LL_SPI_FRAME_SLAVE SPI_TOP_CTRL_SFRMDIR
#define LL_SPI_CLOCK_MASTER 0x00000000U
#define LL_SPI_CLOCK_SLAVE SPI_TOP_CTRL_SCLKDIR
/** @} */

/** @defgroup LL_SPI_DATAWIDTH LL SPI Data Width */
/** @{ */
#define LL_SPI_DATAWIDTH_8BIT (7UL << SPI_TOP_CTRL_DSS_Pos)
#define LL_SPI_DATAWIDTH_16BIT (15UL << SPI_TOP_CTRL_DSS_Pos)
#define LL_SPI_DATAWIDTH_24BIT (23UL << SPI_TOP_CTRL_DSS_Pos)
#define LL_SPI_DATAWIDTH_32BIT (31UL << SPI_TOP_CTRL_DSS_Pos)
/** @} */

/** @defgroup LL_SPI_CLOCKSRC LL SPI Clock Source */
/** @{ */
#define LL_SPI_CLOCKSRC_DIV 0x00000000U
#define LL_SPI_CLOCKSRC_SYS SPI_CLK_CTRL_CLK_SEL
/** @} */

/** @defgroup LL_SPI_ENDIAN LL SPI FIFO Endian */
/** @{ */
#define LL_SPI_TX_ENDIAN_MODE0 (0x0UL << SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Pos)
#define LL_SPI_TX_ENDIAN_MODE1 (0x1UL << SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Pos)
#define LL_SPI_TX_ENDIAN_MODE2 (0x2UL << SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Pos)
#define LL_SPI_TX_ENDIAN_MODE3 (0x3UL << SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Pos)

#define LL_SPI_RX_ENDIAN_MODE0 (0x0UL << SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Pos)
#define LL_SPI_RX_ENDIAN_MODE1 (0x1UL << SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Pos)
#define LL_SPI_RX_ENDIAN_MODE2 (0x2UL << SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Pos)
#define LL_SPI_RX_ENDIAN_MODE3 (0x3UL << SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Pos)
/** @} */

/**
 * @brief SPI protocol format configuration.
 */
typedef struct
{
    uint32_t protocol; /**< Protocol format, use @ref LL_SPI_PROTOCOL_SPI to
                          @ref LL_SPI_PROTOCOL_MICROWIRE. */
    uint32_t clock_polarity; /**< Clock polarity, use @ref LL_SPI_CPOL_LOW or
                                @ref LL_SPI_CPOL_HIGH. */
    uint32_t clock_phase;    /**< Clock phase, use @ref LL_SPI_CPHA_1EDGE or
                                @ref LL_SPI_CPHA_2EDGE. */
} ll_spi_protocol_config_t;

/**
 * @brief SPI role configuration.
 */
typedef struct
{
    uint32_t frame_dir; /**< Frame direction, use @ref LL_SPI_FRAME_MASTER or
                           @ref LL_SPI_FRAME_SLAVE. */
    uint32_t clock_dir; /**< Clock direction, use @ref LL_SPI_CLOCK_MASTER or
                           @ref LL_SPI_CLOCK_SLAVE. */
} ll_spi_role_config_t;

/**
 * @brief SPI frame behavior configuration.
 */
typedef struct
{
    uint32_t data_width; /**< Data width field value, use @ref
                            LL_SPI_DATAWIDTH_8BIT to @ref LL_SPI_DATAWIDTH_32BIT
                            or encoded DSS value. */
    uint32_t
        invert_frame;   /**< Frame inversion control, 0 or SPI_TOP_CTRL_IFS. */
    uint32_t trail_dma; /**< Trailing DMA control, 0 or SPI_TOP_CTRL_TRAIL. */
    uint32_t tte;       /**< TXD tristate control, 0 or SPI_TOP_CTRL_TTE. */
    uint32_t ttelp;     /**< TXD tristate timing, 0 or SPI_TOP_CTRL_TTELP. */
} ll_spi_frame_config_t;

/**
 * @brief SPI FIFO behavior configuration.
 */
typedef struct
{
    uint32_t tx_threshold; /**< TX threshold field value for TFT. */
    uint32_t rx_threshold; /**< RX threshold field value for RFT. */
    uint32_t
        tx_endian; /**< TX FIFO write endian, use @ref
                      LL_SPI_TX_ENDIAN_MODE0 to @ref LL_SPI_TX_ENDIAN_MODE3. */
    uint32_t
        rx_endian; /**< RX FIFO read endian, use @ref
                      LL_SPI_RX_ENDIAN_MODE0 to @ref LL_SPI_RX_ENDIAN_MODE3. */
    uint32_t
        packing_enable; /**< FIFO packing control, 0 or SPI_FIFO_CTRL_FPCKE. */
} ll_spi_fifo_config_t;

/**
 * @brief SPI clock path configuration.
 */
typedef struct
{
    uint32_t clk_div; /**< Clock divider field value for CLK_DIV. */
    uint32_t clk_sel; /**< Clock source, use @ref LL_SPI_CLOCKSRC_DIV or
                         @ref LL_SPI_CLOCKSRC_SYS. */
} ll_spi_clock_config_t;

/**
 * @brief Configure SPI protocol and clock mode fields.
 * @param[in] SPIx SPI instance pointer.
 * @param[in] cfg Pointer to protocol configuration.
 */
static inline void ll_spi_config_protocol(SPI_TypeDef *SPIx,
                                          const ll_spi_protocol_config_t *cfg)
{
    MODIFY_REG(SPIx->TOP_CTRL,
               SPI_TOP_CTRL_FRF | SPI_TOP_CTRL_SPO | SPI_TOP_CTRL_SPH,
               cfg->protocol | cfg->clock_polarity | cfg->clock_phase);
}

/**
 * @brief Configure SPI frame and clock role fields.
 * @param[in] SPIx SPI instance pointer.
 * @param[in] cfg Pointer to role configuration.
 */
static inline void ll_spi_config_role(SPI_TypeDef *SPIx,
                                      const ll_spi_role_config_t *cfg)
{
    MODIFY_REG(SPIx->TOP_CTRL, SPI_TOP_CTRL_SFRMDIR | SPI_TOP_CTRL_SCLKDIR,
               cfg->frame_dir | cfg->clock_dir);
}

/**
 * @brief Configure SPI frame width and frame behavior fields.
 * @param[in] SPIx SPI instance pointer.
 * @param[in] cfg Pointer to frame configuration.
 */
static inline void ll_spi_config_frame(SPI_TypeDef *SPIx,
                                       const ll_spi_frame_config_t *cfg)
{
    MODIFY_REG(SPIx->TOP_CTRL,
               SPI_TOP_CTRL_DSS | SPI_TOP_CTRL_IFS | SPI_TOP_CTRL_TRAIL |
                   SPI_TOP_CTRL_TTE | SPI_TOP_CTRL_TTELP,
               cfg->data_width | cfg->invert_frame | cfg->trail_dma | cfg->tte |
                   cfg->ttelp);
}

/**
 * @brief Configure SPI FIFO threshold, packing and endian fields.
 * @param[in] SPIx SPI instance pointer.
 * @param[in] cfg Pointer to FIFO configuration.
 */
static inline void ll_spi_config_fifo(SPI_TypeDef *SPIx,
                                      const ll_spi_fifo_config_t *cfg)
{
    MODIFY_REG(
        SPIx->FIFO_CTRL,
        SPI_FIFO_CTRL_TFT | SPI_FIFO_CTRL_RFT | SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN |
            SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN | SPI_FIFO_CTRL_FPCKE,
        ((cfg->tx_threshold << SPI_FIFO_CTRL_TFT_Pos) & SPI_FIFO_CTRL_TFT_Msk) |
            ((cfg->rx_threshold << SPI_FIFO_CTRL_RFT_Pos) &
             SPI_FIFO_CTRL_RFT_Msk) |
            cfg->tx_endian | cfg->rx_endian | cfg->packing_enable);
}

/**
 * @brief Configure SPI clock divider/source and enable SPI clock path.
 * @param[in] SPIx SPI instance pointer.
 * @param[in] cfg Pointer to clock configuration.
 */
static inline void ll_spi_config_clock(SPI_TypeDef *SPIx,
                                       const ll_spi_clock_config_t *cfg)
{
    MODIFY_REG(SPIx->CLK_CTRL,
               SPI_CLK_CTRL_CLK_DIV | SPI_CLK_CTRL_CLK_SEL |
                   SPI_CLK_CTRL_CLK_SSP_EN,
               ((cfg->clk_div << SPI_CLK_CTRL_CLK_DIV_Pos) &
                SPI_CLK_CTRL_CLK_DIV_Msk) |
                   cfg->clk_sel | SPI_CLK_CTRL_CLK_SSP_EN);
}

/**
 * @brief Configure receiver timeout threshold.
 * @param[in] SPIx SPI instance pointer.
 * @param[in] timeout Timeout field value.
 */
static inline void ll_spi_set_timeout(SPI_TypeDef *SPIx, uint32_t timeout)
{
    MODIFY_REG(SPIx->TO, SPI_TO_TIMEOUT,
               ((timeout << SPI_TO_TIMEOUT_Pos) & SPI_TO_TIMEOUT_Msk));
}

/**
 * @brief Configure receive-only cycle match value.
 * @param[in] SPIx SPI instance pointer.
 * @param[in] cycles Cycle match value.
 */
static inline void ll_spi_set_rwot_cycle_match(SPI_TypeDef *SPIx,
                                               uint32_t cycles)
{
    WRITE_REG(SPIx->RWOT_CCM, cycles);
}

/**
 * @brief Capture RWOT cycle counter sample.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_capture_rwot_counter(SPI_TypeDef *SPIx)
{
    WRITE_REG(SPIx->RWOT_CVWRN, 1U);
}

/**
 * @brief Read captured RWOT cycle counter value.
 * @param[in] SPIx SPI instance pointer.
 * @return Captured RWOT cycle count.
 */
static inline uint32_t ll_spi_get_rwot_counter_capture(SPI_TypeDef *SPIx)
{
    return READ_REG(SPIx->RWOT_CVWRN);
}

/**
 * @brief Enable SPI peripheral.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->TOP_CTRL, SPI_TOP_CTRL_SSE);
}

/**
 * @brief Disable SPI peripheral.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->TOP_CTRL, SPI_TOP_CTRL_SSE);
}

/**
 * @brief Check whether SPI peripheral is enabled.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when enabled.
 */
static inline uint32_t ll_spi_is_enabled(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->TOP_CTRL, SPI_TOP_CTRL_SSE);
}

/**
 * @brief Enable hold-frame-low mode.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_hold_frame_low(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->TOP_CTRL, SPI_TOP_CTRL_HOLD_FRAME_LOW);
}

/**
 * @brief Disable hold-frame-low mode.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_hold_frame_low(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->TOP_CTRL, SPI_TOP_CTRL_HOLD_FRAME_LOW);
}

/**
 * @brief Enable receive-only mode.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_receive_only(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->RWOT_CTRL, SPI_RWOT_CTRL_RWOT);
}

/**
 * @brief Disable receive-only mode.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_receive_only(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->RWOT_CTRL, SPI_RWOT_CTRL_RWOT);
}

/**
 * @brief Enable RWOT cycle counter.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_rwot_cycle_counter(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->RWOT_CTRL, SPI_RWOT_CTRL_CYCLE_RWOT_EN);
}

/**
 * @brief Disable RWOT cycle counter.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_rwot_cycle_counter(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->RWOT_CTRL, SPI_RWOT_CTRL_CYCLE_RWOT_EN);
}

/**
 * @brief Request setting RWOT cycle counter.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_set_rwot_cycle_counter(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->RWOT_CTRL, SPI_RWOT_CTRL_SET_RWOT_CYCLE);
}

/**
 * @brief Request clearing RWOT cycle counter.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_clear_rwot_cycle_counter(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->RWOT_CTRL, SPI_RWOT_CTRL_CLR_RWOT_CYCLE);
}

/**
 * @brief Enable three-wire half-duplex mode.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_three_wire(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->TRIWIRE_CTRL, SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN);
}

/**
 * @brief Disable three-wire half-duplex mode.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_three_wire(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->TRIWIRE_CTRL, SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN);
}

/**
 * @brief Set three-wire direction to TX (DIO output).
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_set_three_wire_tx(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->TRIWIRE_CTRL, SPI_TRIWIRE_CTRL_TXD_OEN);
}

/**
 * @brief Set three-wire direction to RX (DIO input).
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_set_three_wire_rx(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->TRIWIRE_CTRL, SPI_TRIWIRE_CTRL_TXD_OEN);
}

/**
 * @brief Select SPI_DI pin as data input source.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_select_data_input_from_spi_di(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->CLK_CTRL, SPI_CLK_CTRL_SPI_DI_SEL);
}

/**
 * @brief Select SPI_DIO pin as data input source.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_select_data_input_from_spi_dio(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->CLK_CTRL, SPI_CLK_CTRL_SPI_DI_SEL);
}

/**
 * @brief Write one 32-bit item to TX FIFO data port.
 * @param[in] SPIx SPI instance pointer.
 * @param[in] data 32-bit data payload.
 */
static inline void ll_spi_transmit_data32(SPI_TypeDef *SPIx, uint32_t data)
{
    WRITE_REG(SPIx->DATA, data);
}

/**
 * @brief Read one 32-bit item from RX FIFO data port.
 * @param[in] SPIx SPI instance pointer.
 * @return 32-bit data payload.
 */
static inline uint32_t ll_spi_receive_data32(SPI_TypeDef *SPIx)
{
    return READ_REG(SPIx->DATA);
}

/**
 * @brief Check BSY flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when BSY is set.
 */
static inline uint32_t ll_spi_is_active_flag_busy(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_BSY);
}

/**
 * @brief Check CSS flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when CSS is set.
 */
static inline uint32_t ll_spi_is_active_flag_css(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_CSS);
}

/**
 * @brief Check TFS flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when TFS is set.
 */
static inline uint32_t ll_spi_is_active_flag_tfs(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_TFS);
}

/**
 * @brief Check TNF flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when TNF is set.
 */
static inline uint32_t ll_spi_is_active_flag_tnf(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_TNF);
}

/**
 * @brief Check RFS flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when RFS is set.
 */
static inline uint32_t ll_spi_is_active_flag_rfs(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_RFS);
}

/**
 * @brief Check RNE flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when RNE is set.
 */
static inline uint32_t ll_spi_is_active_flag_rne(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_RNE);
}

/**
 * @brief Check TUR flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when TUR is set.
 */
static inline uint32_t ll_spi_is_active_flag_tur(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_TUR);
}

/**
 * @brief Check ROR flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when ROR is set.
 */
static inline uint32_t ll_spi_is_active_flag_ror(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_ROR);
}

/**
 * @brief Check TINT flag.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when TINT is set.
 */
static inline uint32_t ll_spi_is_active_flag_tint(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_TINT);
}

/**
 * @brief Get TX FIFO level.
 * @param[in] SPIx SPI instance pointer.
 * @return TX FIFO level field value.
 */
static inline uint32_t ll_spi_get_tx_fifo_level(SPI_TypeDef *SPIx)
{
    return ((READ_REG(SPIx->STATUS) & SPI_STATUS_TFL_Msk) >>
            SPI_STATUS_TFL_Pos);
}

/**
 * @brief Get RX FIFO level.
 * @param[in] SPIx SPI instance pointer.
 * @return RX FIFO level field value.
 */
static inline uint32_t ll_spi_get_rx_fifo_level(SPI_TypeDef *SPIx)
{
    return ((READ_REG(SPIx->STATUS) & SPI_STATUS_RFL_Msk) >>
            SPI_STATUS_RFL_Pos);
}

/**
 * @brief Get TX odd sample status.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when TX_OSS is set.
 */
static inline uint32_t ll_spi_get_tx_odd_sample_status(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_TX_OSS);
}

/**
 * @brief Get RX odd sample status.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when OSS is set.
 */
static inline uint32_t ll_spi_get_rx_odd_sample_status(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->STATUS, SPI_STATUS_OSS);
}

/**
 * @brief Clear TUR flag by rw1c write.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_clear_flag_tur(SPI_TypeDef *SPIx)
{
    WRITE_REG(SPIx->STATUS, SPI_STATUS_TUR);
}

/**
 * @brief Clear ROR flag by rw1c write.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_clear_flag_ror(SPI_TypeDef *SPIx)
{
    WRITE_REG(SPIx->STATUS, SPI_STATUS_ROR);
}

/**
 * @brief Clear TINT flag by rw1c write.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_clear_flag_tint(SPI_TypeDef *SPIx)
{
    WRITE_REG(SPIx->STATUS, SPI_STATUS_TINT);
}

/**
 * @brief Enable TX threshold interrupt.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_it_tx_threshold(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->INTE, SPI_INTE_TIE);
}

/**
 * @brief Disable TX threshold interrupt.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_it_tx_threshold(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->INTE, SPI_INTE_TIE);
}

/**
 * @brief Check TX threshold interrupt enable state.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when TX threshold interrupt is enabled.
 */
static inline uint32_t ll_spi_is_enabled_it_tx_threshold(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->INTE, SPI_INTE_TIE);
}

/**
 * @brief Enable RX threshold interrupt.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_it_rx_threshold(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->INTE, SPI_INTE_RIE);
}

/**
 * @brief Disable RX threshold interrupt.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_it_rx_threshold(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->INTE, SPI_INTE_RIE);
}

/**
 * @brief Check RX threshold interrupt enable state.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when RX threshold interrupt is enabled.
 */
static inline uint32_t ll_spi_is_enabled_it_rx_threshold(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->INTE, SPI_INTE_RIE);
}

/**
 * @brief Enable receiver timeout interrupt.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_it_timeout(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->INTE, SPI_INTE_TINTE);
}

/**
 * @brief Disable receiver timeout interrupt.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_it_timeout(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->INTE, SPI_INTE_TINTE);
}

/**
 * @brief Check receiver timeout interrupt enable state.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when timeout interrupt is enabled.
 */
static inline uint32_t ll_spi_is_enabled_it_timeout(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->INTE, SPI_INTE_TINTE);
}

/**
 * @brief Mask TX underrun interrupt source.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_mask_it_tx_underrun(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->INTE, SPI_INTE_TIM);
}

/**
 * @brief Unmask TX underrun interrupt source.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_unmask_it_tx_underrun(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->INTE, SPI_INTE_TIM);
}

/**
 * @brief Check TX underrun interrupt mask state.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when TX underrun interrupt is masked.
 */
static inline uint32_t ll_spi_is_masked_it_tx_underrun(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->INTE, SPI_INTE_TIM);
}

/**
 * @brief Mask RX overrun interrupt source.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_mask_it_rx_overrun(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->INTE, SPI_INTE_RIM);
}

/**
 * @brief Unmask RX overrun interrupt source.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_unmask_it_rx_overrun(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->INTE, SPI_INTE_RIM);
}

/**
 * @brief Check RX overrun interrupt mask state.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when RX overrun interrupt is masked.
 */
static inline uint32_t ll_spi_is_masked_it_rx_overrun(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->INTE, SPI_INTE_RIM);
}

/**
 * @brief Enable TX DMA request.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_dma_tx(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->FIFO_CTRL, SPI_FIFO_CTRL_TSRE);
}

/**
 * @brief Disable TX DMA request.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_dma_tx(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->FIFO_CTRL, SPI_FIFO_CTRL_TSRE);
}

/**
 * @brief Check TX DMA request enable state.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when TX DMA request is enabled.
 */
static inline uint32_t ll_spi_is_enabled_dma_tx(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->FIFO_CTRL, SPI_FIFO_CTRL_TSRE);
}

/**
 * @brief Enable RX DMA request.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_enable_dma_rx(SPI_TypeDef *SPIx)
{
    SET_BIT(SPIx->FIFO_CTRL, SPI_FIFO_CTRL_RSRE);
}

/**
 * @brief Disable RX DMA request.
 * @param[in] SPIx SPI instance pointer.
 */
static inline void ll_spi_disable_dma_rx(SPI_TypeDef *SPIx)
{
    CLEAR_BIT(SPIx->FIFO_CTRL, SPI_FIFO_CTRL_RSRE);
}

/**
 * @brief Check RX DMA request enable state.
 * @param[in] SPIx SPI instance pointer.
 * @return Non-zero when RX DMA request is enabled.
 */
static inline uint32_t ll_spi_is_enabled_dma_rx(SPI_TypeDef *SPIx)
{
    return READ_BIT(SPIx->FIFO_CTRL, SPI_FIFO_CTRL_RSRE);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_SPI_H */
