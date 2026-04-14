/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_USART_H
#define __LL_USART_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_usart.h
 * @brief Header-only low-level USART APIs for SF32LB52x.
 */

/** @defgroup LL_USART_DATAWIDTH LL USART Data Width */
/** @{ */
#define LL_USART_DATAWIDTH_6B 0x00000000U
#define LL_USART_DATAWIDTH_7B (0x1UL << USART_CR1_M_Pos)
#define LL_USART_DATAWIDTH_8B (0x2UL << USART_CR1_M_Pos)
#define LL_USART_DATAWIDTH_9B (0x3UL << USART_CR1_M_Pos)
/** @} */

/** @defgroup LL_USART_PARITY LL USART Parity */
/** @{ */
#define LL_USART_PARITY_NONE 0x00000000U
#define LL_USART_PARITY_EVEN USART_CR1_PCE
#define LL_USART_PARITY_ODD (USART_CR1_PCE | USART_CR1_PS)
/** @} */

/** @defgroup LL_USART_STOPBITS LL USART Stop Bits */
/** @{ */
#define LL_USART_STOPBITS_1 0x00000000U
#define LL_USART_STOPBITS_2 (0x2UL << USART_CR2_STOP_Pos)
/** @} */

/** @defgroup LL_USART_HWCONTROL LL USART Hardware Flow Control */
/** @{ */
#define LL_USART_HWCONTROL_NONE 0x00000000U
#define LL_USART_HWCONTROL_RTS USART_CR3_RTSE
#define LL_USART_HWCONTROL_CTS USART_CR3_CTSE
#define LL_USART_HWCONTROL_RTS_CTS (USART_CR3_RTSE | USART_CR3_CTSE)
/** @} */

/** @defgroup LL_USART_CLOCK LL USART Peripheral Clock */
/** @{ */
#define LL_USART_PERIPHCLK_HZ 48000000U
/** @} */

/**
 * @brief USART frame configuration.
 */
typedef struct
{
    uint32_t data_width; /**< Frame data width, use @ref LL_USART_DATAWIDTH_6B
                            to @ref LL_USART_DATAWIDTH_9B. */
    uint32_t parity;     /**< Parity mode, use @ref LL_USART_PARITY_NONE, @ref
                            LL_USART_PARITY_EVEN, or @ref LL_USART_PARITY_ODD. */
    uint32_t stop_bits;  /**< Stop bit setting, use @ref LL_USART_STOPBITS_1 or
                            @ref LL_USART_STOPBITS_2. */
} ll_usart_frame_config_t;

/**
 * @brief Configure frame format (data width + parity + stop bits).
 * @param[in] USARTx USART instance pointer.
 * @param[in] cfg Pointer to frame configuration.
 */
static inline void ll_usart_config_frame(USART_TypeDef *USARTx,
                                         const ll_usart_frame_config_t *cfg)
{
    MODIFY_REG(USARTx->CR1, USART_CR1_M | USART_CR1_PCE | USART_CR1_PS,
               cfg->data_width | cfg->parity);
    MODIFY_REG(USARTx->CR2, USART_CR2_STOP, cfg->stop_bits);
}

/**
 * @brief Configure USART baudrate and oversampling mode.
 * @param[in] USARTx USART instance pointer.
 * @param[in] baudrate Target baudrate in bps.
 */
static inline void ll_usart_config_baudrate(USART_TypeDef *USARTx,
                                            uint32_t baudrate)
{
    uint32_t div;

    div = LL_USART_PERIPHCLK_HZ / baudrate;
    if (div < 16U)
    {
        div = (LL_USART_PERIPHCLK_HZ * 2U) / baudrate;
        SET_BIT(USARTx->CR1, USART_CR1_OVER8);
        MODIFY_REG(
            USARTx->MISCR, USART_MISCR_SMPLINI_Msk,
            MAKE_REG_VAL(2U, USART_MISCR_SMPLINI_Msk, USART_MISCR_SMPLINI_Pos));
    }
    else
    {
        CLEAR_BIT(USARTx->CR1, USART_CR1_OVER8);
        MODIFY_REG(
            USARTx->MISCR, USART_MISCR_SMPLINI_Msk,
            MAKE_REG_VAL(6U, USART_MISCR_SMPLINI_Msk, USART_MISCR_SMPLINI_Pos));
    }

    WRITE_REG(USARTx->BRR, div);
}

/**
 * @brief Configure USART hardware flow control.
 * @param[in] USARTx USART instance pointer.
 * @param[in] hwcontrol Hardware flow control value from @ref
 * LL_USART_HWCONTROL.
 */
static inline void ll_usart_config_hwflow(USART_TypeDef *USARTx,
                                          uint32_t hwcontrol)
{
    MODIFY_REG(USARTx->CR3, USART_CR3_RTSE | USART_CR3_CTSE, hwcontrol);
}

/**
 * @brief Enable USART peripheral.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_UE);
}

/**
 * @brief Disable USART peripheral.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_UE);
}

/**
 * @brief Check whether USART peripheral is enabled.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when enabled, zero when disabled.
 */
static inline uint32_t ll_usart_is_enabled(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->CR1, USART_CR1_UE);
}

/**
 * @brief Enable TX direction.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_tx(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_TE);
}

/**
 * @brief Disable TX direction.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_tx(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_TE);
}

/**
 * @brief Enable RX direction.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_rx(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_RE);
}

/**
 * @brief Disable RX direction.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_rx(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_RE);
}

/**
 * @brief Write one 8-bit data frame to TDR.
 * @param[in] USARTx USART instance pointer.
 * @param[in] data 8-bit payload.
 */
static inline void ll_usart_transmit_data8(USART_TypeDef *USARTx, uint8_t data)
{
    WRITE_REG(USARTx->TDR, (uint32_t)data);
}

/**
 * @brief Read one 8-bit data frame from RDR.
 * @param[in] USARTx USART instance pointer.
 * @return 8-bit payload.
 */
static inline uint8_t ll_usart_receive_data8(USART_TypeDef *USARTx)
{
    return (uint8_t)(READ_REG(USARTx->RDR) & 0xFFU);
}

/**
 * @brief Write one 9-bit data frame to TDR.
 * @param[in] USARTx USART instance pointer.
 * @param[in] data 9-bit payload in bits [8:0].
 */
static inline void ll_usart_transmit_data9(USART_TypeDef *USARTx, uint16_t data)
{
    WRITE_REG(USARTx->TDR, ((uint32_t)data & 0x1FFU));
}

/**
 * @brief Read one 9-bit data frame from RDR.
 * @param[in] USARTx USART instance pointer.
 * @return 9-bit payload in bits [8:0].
 */
static inline uint16_t ll_usart_receive_data9(USART_TypeDef *USARTx)
{
    return (uint16_t)(READ_REG(USARTx->RDR) & 0x1FFU);
}

/**
 * @brief Check TXE flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when TXE is set.
 */
static inline uint32_t ll_usart_is_active_flag_txe(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_TXE);
}

/**
 * @brief Check RXNE flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when RXNE is set.
 */
static inline uint32_t ll_usart_is_active_flag_rxne(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_RXNE);
}

/**
 * @brief Check TC flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when TC is set.
 */
static inline uint32_t ll_usart_is_active_flag_tc(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_TC);
}

/**
 * @brief Check IDLE flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when IDLE is set.
 */
static inline uint32_t ll_usart_is_active_flag_idle(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_IDLE);
}

/**
 * @brief Check ORE flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when ORE is set.
 */
static inline uint32_t ll_usart_is_active_flag_ore(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_ORE);
}

/**
 * @brief Check FE flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when FE is set.
 */
static inline uint32_t ll_usart_is_active_flag_fe(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_FE);
}

/**
 * @brief Check NF flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when NF is set.
 */
static inline uint32_t ll_usart_is_active_flag_ne(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_NF);
}

/**
 * @brief Check PE flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when PE is set.
 */
static inline uint32_t ll_usart_is_active_flag_pe(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_PE);
}

/**
 * @brief Check CTS pin state flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when CTS is set.
 */
static inline uint32_t ll_usart_is_active_flag_cts(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_CTS);
}

/**
 * @brief Check CTS change interrupt flag.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when CTSIF is set.
 */
static inline uint32_t ll_usart_is_active_flag_ctsif(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->ISR, USART_ISR_CTSIF);
}

/**
 * @brief Clear PE flag.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_clear_flag_pe(USART_TypeDef *USARTx)
{
    WRITE_REG(USARTx->ICR, USART_ICR_PECF);
}

/**
 * @brief Clear FE flag.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_clear_flag_fe(USART_TypeDef *USARTx)
{
    WRITE_REG(USARTx->ICR, USART_ICR_FECF);
}

/**
 * @brief Clear NF flag.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_clear_flag_ne(USART_TypeDef *USARTx)
{
    WRITE_REG(USARTx->ICR, USART_ICR_NCF);
}

/**
 * @brief Clear ORE flag.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_clear_flag_ore(USART_TypeDef *USARTx)
{
    WRITE_REG(USARTx->ICR, USART_ICR_ORECF);
}

/**
 * @brief Clear IDLE flag.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_clear_flag_idle(USART_TypeDef *USARTx)
{
    WRITE_REG(USARTx->ICR, USART_ICR_IDLECF);
}

/**
 * @brief Clear TC flag.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_clear_flag_tc(USART_TypeDef *USARTx)
{
    WRITE_REG(USARTx->ICR, USART_ICR_TCCF);
}

/**
 * @brief Clear CTSIF flag.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_clear_flag_cts(USART_TypeDef *USARTx)
{
    WRITE_REG(USARTx->ICR, USART_ICR_CTSCF);
}

/**
 * @brief Enable RXNE interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_it_rxne(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_RXNEIE);
}

/**
 * @brief Disable RXNE interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_it_rxne(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_RXNEIE);
}

/**
 * @brief Check RXNE interrupt enable state.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when RXNE interrupt is enabled.
 */
static inline uint32_t ll_usart_is_enabled_it_rxne(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->CR1, USART_CR1_RXNEIE);
}

/**
 * @brief Enable TXE interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_it_txe(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_TXEIE);
}

/**
 * @brief Disable TXE interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_it_txe(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_TXEIE);
}

/**
 * @brief Check TXE interrupt enable state.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when TXE interrupt is enabled.
 */
static inline uint32_t ll_usart_is_enabled_it_txe(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->CR1, USART_CR1_TXEIE);
}

/**
 * @brief Enable TC interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_it_tc(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_TCIE);
}

/**
 * @brief Disable TC interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_it_tc(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_TCIE);
}

/**
 * @brief Check TC interrupt enable state.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when TC interrupt is enabled.
 */
static inline uint32_t ll_usart_is_enabled_it_tc(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->CR1, USART_CR1_TCIE);
}

/**
 * @brief Enable IDLE interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_it_idle(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_IDLEIE);
}

/**
 * @brief Disable IDLE interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_it_idle(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_IDLEIE);
}

/**
 * @brief Check IDLE interrupt enable state.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when IDLE interrupt is enabled.
 */
static inline uint32_t ll_usart_is_enabled_it_idle(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->CR1, USART_CR1_IDLEIE);
}

/**
 * @brief Enable parity error interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_it_pe(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_PEIE);
}

/**
 * @brief Disable parity error interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_it_pe(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_PEIE);
}

/**
 * @brief Check parity error interrupt enable state.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when parity error interrupt is enabled.
 */
static inline uint32_t ll_usart_is_enabled_it_pe(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->CR1, USART_CR1_PEIE);
}

/**
 * @brief Enable error interrupt (FE/ORE/NF).
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_it_error(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR3, USART_CR3_EIE);
}

/**
 * @brief Disable error interrupt (FE/ORE/NF).
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_it_error(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR3, USART_CR3_EIE);
}

/**
 * @brief Check error interrupt enable state.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when error interrupt is enabled.
 */
static inline uint32_t ll_usart_is_enabled_it_error(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->CR3, USART_CR3_EIE);
}

/**
 * @brief Enable CTS interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_it_cts(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR3, USART_CR3_CTSIE);
}

/**
 * @brief Disable CTS interrupt.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_it_cts(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR3, USART_CR3_CTSIE);
}

/**
 * @brief Check CTS interrupt enable state.
 * @param[in] USARTx USART instance pointer.
 * @return Non-zero when CTS interrupt is enabled.
 */
static inline uint32_t ll_usart_is_enabled_it_cts(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->CR3, USART_CR3_CTSIE);
}

/**
 * @brief Enable RX DMA request.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_dma_rx(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR3, USART_CR3_DMAR);
}

/**
 * @brief Disable RX DMA request.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_dma_rx(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR3, USART_CR3_DMAR);
}

/**
 * @brief Enable TX DMA request.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_dma_tx(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR3, USART_CR3_DMAT);
}

/**
 * @brief Disable TX DMA request.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_dma_tx(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR3, USART_CR3_DMAT);
}

/**
 * @brief Enable overrun detection.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_overrun_detect(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR3, USART_CR3_OVRDIS);
}

/**
 * @brief Disable overrun detection.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_overrun_detect(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR3, USART_CR3_OVRDIS);
}

/**
 * @brief Enable one-bit sampling mode.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_enable_onebit_sample(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR3, USART_CR3_ONEBIT);
}

/**
 * @brief Disable one-bit sampling mode.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_disable_onebit_sample(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR3, USART_CR3_ONEBIT);
}

/**
 * @brief Flush RX data by issuing RXFRQ request.
 * @param[in] USARTx USART instance pointer.
 */
static inline void ll_usart_request_rxdata_flush(USART_TypeDef *USARTx)
{
    WRITE_REG(USARTx->RQR, USART_RQR_RXFRQ);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_USART_H */
