/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_AUDPRC_H
#define __LL_AUDPRC_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_audprc.h
 * @brief Header-only low-level Audio Processor (AUDPRC) APIs for SF32LB52x.
 *
 * AUDPRC routes and processes digital audio between the codec and the AHB
 * bus: TX channels feed the DAC path, RX channels capture the ADC path, and
 * TX_OUT channels feed an external interface. A multi-stage SRC, an EQ stage
 * (50 coefficient registers) and per-channel volume ramping are provided.
 */

/**
 * @brief Get the TX channel configuration register.
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX channel index in range 0..3.
 * @return TX channel CFG register pointer.
 */
static inline __IO uint32_t *ll_audprc_tx_cfg_reg(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    return &((&AUDPRCx->TX_CH0_CFG)[ch * 2U]);
}

/**
 * @brief Get the RX channel configuration register.
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      RX channel index in range 0..1.
 * @return RX channel CFG register pointer.
 */
static inline __IO uint32_t *ll_audprc_rx_cfg_reg(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    return &((&AUDPRCx->RX_CH0_CFG)[ch * 2U]);
}

/**
 * @brief Get the TX_OUT channel configuration register.
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX_OUT channel index in range 0..1.
 * @return TX_OUT channel CFG register pointer.
 */
static inline __IO uint32_t *ll_audprc_tx_out_cfg_reg(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    return &((&AUDPRCx->TX_OUT_CH0_CFG)[ch * 2U]);
}

/*==============================================================================
 * Revision & Core Control (ID / CFG)
 *============================================================================*/

/**
 * @brief Get the AUDPRC revision ID (ID.REV).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @return Revision ID.
 */
static inline uint32_t ll_audprc_get_revision(AUDPRC_TypeDef *AUDPRCx)
{
    return READ_REG(AUDPRCx->ID);
}

/**
 * @brief Commit the AUDPRC clock divider (CFG.AUDCLK_DIV_UPDATE, write 1).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_clock_div_update(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_AUDCLK_DIV_UPDATE);
}

/**
 * @brief Set the AUDPRC clock divider (CFG.AUDCLK_DIV).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] div     Clock divider (4 bits); 0/1 both mean divide by 1.
 */
static inline void ll_audprc_set_clock_div(AUDPRC_TypeDef *AUDPRCx, uint32_t div)
{
    MODIFY_REG(AUDPRCx->CFG, AUDPRC_CFG_AUDCLK_DIV,
               MAKE_REG_VAL(div, AUDPRC_CFG_AUDCLK_DIV_Msk, AUDPRC_CFG_AUDCLK_DIV_Pos));
}

/**
 * @brief Select the audio strobe clock source (CFG.STB_CLK_SEL).
 * @param[in] AUDPRCx  AUDPRC instance pointer.
 * @param[in] use_pll  Non-zero selects PLL, zero selects XTAL.
 */
static inline void ll_audprc_set_stb_clk_sel(AUDPRC_TypeDef *AUDPRCx, uint32_t use_pll)
{
    MODIFY_REG(AUDPRCx->CFG, AUDPRC_CFG_STB_CLK_SEL,
               use_pll ? AUDPRC_CFG_STB_CLK_SEL : 0U);
}

/**
 * @brief Enable automatic clock gating (CFG.AUTO_GATE_EN).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_auto_gate_enable(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_AUTO_GATE_EN);
}

/**
 * @brief Disable automatic clock gating (CFG.AUTO_GATE_EN = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_auto_gate_disable(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->CFG, AUDPRC_CFG_AUTO_GATE_EN);
}

/**
 * @brief Enable the ADC path (CFG.ADC_PATH_EN).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_path_enable(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_ADC_PATH_EN);
}

/**
 * @brief Disable the ADC path (CFG.ADC_PATH_EN = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_path_disable(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->CFG, AUDPRC_CFG_ADC_PATH_EN);
}

/**
 * @brief Enable the DAC path (CFG.DAC_PATH_EN).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_dac_path_enable(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_DAC_PATH_EN);
}

/**
 * @brief Disable the DAC path (CFG.DAC_PATH_EN = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_dac_path_disable(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->CFG, AUDPRC_CFG_DAC_PATH_EN);
}

/**
 * @brief Assert the ADC path software reset (CFG.ADC_PATH_SRESET).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_path_reset(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_ADC_PATH_SRESET);
}

/**
 * @brief Release the ADC path software reset (CFG.ADC_PATH_SRESET = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_path_release_reset(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->CFG, AUDPRC_CFG_ADC_PATH_SRESET);
}

/**
 * @brief Assert the DAC path software reset (CFG.DAC_PATH_SRESET).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_dac_path_reset(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_DAC_PATH_SRESET);
}

/**
 * @brief Release the DAC path software reset (CFG.DAC_PATH_SRESET = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_dac_path_release_reset(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->CFG, AUDPRC_CFG_DAC_PATH_SRESET);
}

/**
 * @brief Flush the ADC path FIFOs (CFG.ADC_PATH_FLUSH).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_path_flush(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_ADC_PATH_FLUSH);
}

/**
 * @brief Flush the DAC path FIFOs (CFG.DAC_PATH_FLUSH).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_dac_path_flush(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_DAC_PATH_FLUSH);
}

/**
 * @brief Assert the AUDPRC software reset (CFG.SRESET).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_reset(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_SRESET);
}

/**
 * @brief Release the AUDPRC software reset (CFG.SRESET = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_release_reset(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->CFG, AUDPRC_CFG_SRESET);
}

/**
 * @brief Enable the AUDPRC core (CFG.ENABLE).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_enable(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->CFG, AUDPRC_CFG_ENABLE);
}

/**
 * @brief Disable the AUDPRC core (CFG.ENABLE = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_disable(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->CFG, AUDPRC_CFG_ENABLE);
}

/**
 * @brief Enable a TX channel (TX_CHx.CFG.ENABLE).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX channel index in range 0..3.
 */
static inline void ll_audprc_tx_channel_enable(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_tx_cfg_reg(AUDPRCx, ch);

    SET_BIT(*cfg, AUDPRC_TX_CH0_CFG_ENABLE);
}

/**
 * @brief Disable a TX channel (TX_CHx.CFG.ENABLE = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX channel index in range 0..3.
 */
static inline void ll_audprc_tx_channel_disable(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_tx_cfg_reg(AUDPRCx, ch);

    CLEAR_BIT(*cfg, AUDPRC_TX_CH0_CFG_ENABLE);
}

/**
 * @brief Enable an RX channel (RX_CHx.CFG.ENABLE).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      RX channel index in range 0..1.
 */
static inline void ll_audprc_rx_channel_enable(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_rx_cfg_reg(AUDPRCx, ch);

    SET_BIT(*cfg, AUDPRC_RX_CH0_CFG_ENABLE);
}

/**
 * @brief Disable an RX channel (RX_CHx.CFG.ENABLE = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      RX channel index in range 0..1.
 */
static inline void ll_audprc_rx_channel_disable(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_rx_cfg_reg(AUDPRCx, ch);

    CLEAR_BIT(*cfg, AUDPRC_RX_CH0_CFG_ENABLE);
}

/**
 * @brief Enable a TX_OUT channel (TX_OUT_CHx.CFG.ENABLE).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX_OUT channel index in range 0..1.
 */
static inline void ll_audprc_tx_out_channel_enable(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_tx_out_cfg_reg(AUDPRCx, ch);

    SET_BIT(*cfg, AUDPRC_TX_OUT_CH0_CFG_ENABLE);
}

/**
 * @brief Disable a TX_OUT channel (TX_OUT_CHx.CFG.ENABLE = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX_OUT channel index in range 0..1.
 */
static inline void ll_audprc_tx_out_channel_disable(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_tx_out_cfg_reg(AUDPRCx, ch);

    CLEAR_BIT(*cfg, AUDPRC_TX_OUT_CH0_CFG_ENABLE);
}

/**
 * @brief Set the ADC strobe divider (STB.ADC_DIV).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] div     ADC strobe divider (16 bits).
 */
static inline void ll_audprc_set_adc_stb_div(AUDPRC_TypeDef *AUDPRCx, uint32_t div)
{
    MODIFY_REG(AUDPRCx->STB, AUDPRC_STB_ADC_DIV,
               MAKE_REG_VAL(div, AUDPRC_STB_ADC_DIV_Msk, AUDPRC_STB_ADC_DIV_Pos));
}

/*==============================================================================
 * Strobe Divider (STB)
 *============================================================================*/

/**
 * @brief Set the DAC strobe divider (STB.DAC_DIV).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] div     DAC strobe divider (16 bits).
 */
static inline void ll_audprc_set_dac_stb_div(AUDPRC_TypeDef *AUDPRCx, uint32_t div)
{
    MODIFY_REG(AUDPRCx->STB, AUDPRC_STB_DAC_DIV,
               MAKE_REG_VAL(div, AUDPRC_STB_DAC_DIV_Msk, AUDPRC_STB_DAC_DIV_Pos));
}

/*==============================================================================
 * Audio Channels (TX_CHx / RX_CHx / TX_OUT_CHx)
 *============================================================================*/

/**
 * @brief Get the TX channel FIFO fill count (TX_CHx.CFG.FIFO_CNT).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX channel index in range 0..3.
 * @return FIFO fill count (4 bits).
 */
static inline uint32_t ll_audprc_get_tx_fifo_count(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_tx_cfg_reg(AUDPRCx, ch);

    return (*cfg & AUDPRC_TX_CH0_CFG_FIFO_CNT_Msk) >> AUDPRC_TX_CH0_CFG_FIFO_CNT_Pos;
}

/**
 * @brief Configure a TX channel (TX_CHx.CFG FORMAT/MODE/DMA_MSK).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX channel index in range 0..3.
 * @param[in] format  Data format: 0 = 16-bit, 1 = 24-bit.
 * @param[in] mode    Channel mode: 0 = mono, 1 = stereo (16-bit only).
 * @param[in] dma_msk Non-zero masks the DMA request.
 */
static inline void ll_audprc_tx_channel_config(AUDPRC_TypeDef *AUDPRCx, uint32_t ch,
        uint32_t format, uint32_t mode, uint32_t dma_msk)
{
    __IO uint32_t *cfg = ll_audprc_tx_cfg_reg(AUDPRCx, ch);

    MODIFY_REG(*cfg,
               AUDPRC_TX_CH0_CFG_FORMAT | AUDPRC_TX_CH0_CFG_MODE |
               AUDPRC_TX_CH0_CFG_DMA_MSK,
               MAKE_REG_VAL(format, AUDPRC_TX_CH0_CFG_FORMAT_Msk, AUDPRC_TX_CH0_CFG_FORMAT_Pos) |
               (mode ? AUDPRC_TX_CH0_CFG_MODE : 0U) |
               (dma_msk ? AUDPRC_TX_CH0_CFG_DMA_MSK : 0U));
}

/**
 * @brief Write a TX channel data entry (TX_CHx.ENTRY.DATA).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX channel index in range 0..3.
 * @param[in] data    Data word to write.
 */
static inline void ll_audprc_tx_write_entry(AUDPRC_TypeDef *AUDPRCx, uint32_t ch,
        uint32_t data)
{
    volatile uint32_t *entry = &((&AUDPRCx->TX_CH0_ENTRY)[ch * 2U]);

    WRITE_REG(*entry, data);
}

/**
 * @brief Get the RX channel FIFO fill count (RX_CHx.CFG.FIFO_CNT).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      RX channel index in range 0..1.
 * @return FIFO fill count (4 bits).
 */
static inline uint32_t ll_audprc_get_rx_fifo_count(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_rx_cfg_reg(AUDPRCx, ch);

    return (*cfg & AUDPRC_RX_CH0_CFG_FIFO_CNT_Msk) >> AUDPRC_RX_CH0_CFG_FIFO_CNT_Pos;
}

/**
 * @brief Configure an RX channel (RX_CHx.CFG FORMAT/MODE/DMA_MSK).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      RX channel index in range 0..1.
 * @param[in] format  Data format: 0 = 16-bit, 1 = 24-bit.
 * @param[in] mode    Channel mode: 0 = mono, 1 = stereo (16-bit only).
 * @param[in] dma_msk Non-zero masks the DMA request.
 */
static inline void ll_audprc_rx_channel_config(AUDPRC_TypeDef *AUDPRCx, uint32_t ch,
        uint32_t format, uint32_t mode, uint32_t dma_msk)
{
    __IO uint32_t *cfg = ll_audprc_rx_cfg_reg(AUDPRCx, ch);

    MODIFY_REG(*cfg,
               AUDPRC_RX_CH0_CFG_FORMAT | AUDPRC_RX_CH0_CFG_MODE |
               AUDPRC_RX_CH0_CFG_DMA_MSK,
               MAKE_REG_VAL(format, AUDPRC_RX_CH0_CFG_FORMAT_Msk, AUDPRC_RX_CH0_CFG_FORMAT_Pos) |
               (mode ? AUDPRC_RX_CH0_CFG_MODE : 0U) |
               (dma_msk ? AUDPRC_RX_CH0_CFG_DMA_MSK : 0U));
}

/**
 * @brief Read an RX channel data entry (RX_CHx.ENTRY.DATA).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      RX channel index in range 0..1.
 * @return Data word read from the channel.
 */
static inline uint32_t ll_audprc_rx_read_entry(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    volatile uint32_t *entry = &((&AUDPRCx->RX_CH0_ENTRY)[ch * 2U]);

    return READ_REG(*entry);
}

/**
 * @brief Get the TX_OUT channel FIFO fill count (TX_OUT_CHx.CFG.FIFO_CNT).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX_OUT channel index in range 0..1.
 * @return FIFO fill count (4 bits).
 */
static inline uint32_t ll_audprc_get_tx_out_fifo_count(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    __IO uint32_t *cfg = ll_audprc_tx_out_cfg_reg(AUDPRCx, ch);

    return (*cfg & AUDPRC_TX_OUT_CH0_CFG_FIFO_CNT_Msk) >>
           AUDPRC_TX_OUT_CH0_CFG_FIFO_CNT_Pos;
}

/**
 * @brief Configure a TX_OUT channel (TX_OUT_CHx.CFG FORMAT/MODE/DMA_MSK).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX_OUT channel index in range 0..1.
 * @param[in] format  Data format: 0 = 16-bit, 1 = 24-bit.
 * @param[in] mode    Channel mode: 0 = mono, 1 = stereo (16-bit only).
 * @param[in] dma_msk Non-zero masks the DMA request.
 */
static inline void ll_audprc_tx_out_channel_config(AUDPRC_TypeDef *AUDPRCx, uint32_t ch,
        uint32_t format, uint32_t mode,
        uint32_t dma_msk)
{
    __IO uint32_t *cfg = ll_audprc_tx_out_cfg_reg(AUDPRCx, ch);

    MODIFY_REG(*cfg,
               AUDPRC_TX_OUT_CH0_CFG_FORMAT | AUDPRC_TX_OUT_CH0_CFG_MODE |
               AUDPRC_TX_OUT_CH0_CFG_DMA_MSK,
               MAKE_REG_VAL(format, AUDPRC_TX_OUT_CH0_CFG_FORMAT_Msk,
                            AUDPRC_TX_OUT_CH0_CFG_FORMAT_Pos) |
               (mode ? AUDPRC_TX_OUT_CH0_CFG_MODE : 0U) |
               (dma_msk ? AUDPRC_TX_OUT_CH0_CFG_DMA_MSK : 0U));
}

/**
 * @brief Read a TX_OUT channel data entry (TX_OUT_CHx.ENTRY.DATA).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch      TX_OUT channel index in range 0..1.
 * @return Data word read from the channel.
 */
static inline uint32_t ll_audprc_tx_out_read_entry(AUDPRC_TypeDef *AUDPRCx, uint32_t ch)
{
    volatile uint32_t *entry = &((&AUDPRCx->TX_OUT_CH0_ENTRY)[ch * 2U]);

    return READ_REG(*entry);
}

/*==============================================================================
 * DAC Path (DAC_PATH_CFG0..CFG3)
 *============================================================================*/

/**
 * @brief Select the DAC path destination (DAC_PATH_CFG0.DST_SEL).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] sel     Destination select (2 bits): 0 = codec, 1 = external,
 *                    2 = APB interface.
 */
static inline void ll_audprc_dac_set_dst_sel(AUDPRC_TypeDef *AUDPRCx, uint32_t sel)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_DST_SEL,
               MAKE_REG_VAL(sel, AUDPRC_DAC_PATH_CFG0_DST_SEL_Msk,
                            AUDPRC_DAC_PATH_CFG0_DST_SEL_Pos));
}

/**
 * @brief Select a DAC mixer right input source (DAC_PATH_CFG0.MIXRSRC0/1).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] slot    Mixer slot (0 or 1).
 * @param[in] src     Source (3 bits): 0..3 = TX_CH0..3, 4 = mute.
 */
static inline void ll_audprc_dac_set_mix_r_src(AUDPRC_TypeDef *AUDPRCx, uint32_t slot,
        uint32_t src)
{
    if (slot == 0U)
    {
        MODIFY_REG(AUDPRCx->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_MIXRSRC0,
                   MAKE_REG_VAL(src, AUDPRC_DAC_PATH_CFG0_MIXRSRC0_Msk,
                                AUDPRC_DAC_PATH_CFG0_MIXRSRC0_Pos));
    }
    else
    {
        MODIFY_REG(AUDPRCx->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_MIXRSRC1,
                   MAKE_REG_VAL(src, AUDPRC_DAC_PATH_CFG0_MIXRSRC1_Msk,
                                AUDPRC_DAC_PATH_CFG0_MIXRSRC1_Pos));
    }
}

/**
 * @brief Select a DAC mixer left input source (DAC_PATH_CFG0.MIXLSRC0/1).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] slot    Mixer slot (0 or 1).
 * @param[in] src     Source (3 bits): 0..3 = TX_CH0..3, 4 = mute.
 */
static inline void ll_audprc_dac_set_mix_l_src(AUDPRC_TypeDef *AUDPRCx, uint32_t slot,
        uint32_t src)
{
    if (slot == 0U)
    {
        MODIFY_REG(AUDPRCx->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_MIXLSRC0,
                   MAKE_REG_VAL(src, AUDPRC_DAC_PATH_CFG0_MIXLSRC0_Msk,
                                AUDPRC_DAC_PATH_CFG0_MIXLSRC0_Pos));
    }
    else
    {
        MODIFY_REG(AUDPRCx->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_MIXLSRC1,
                   MAKE_REG_VAL(src, AUDPRC_DAC_PATH_CFG0_MIXLSRC1_Msk,
                                AUDPRC_DAC_PATH_CFG0_MIXLSRC1_Pos));
    }
}

/**
 * @brief Set the DAC mixer right volume (DAC_PATH_CFG0 ROUGH/FINE_VOL_R).
 * @param[in] AUDPRCx   AUDPRC instance pointer.
 * @param[in] rough_vol Rough volume (4 bits, 6 dB steps).
 * @param[in] fine_vol  Fine volume (4 bits, 0.5 dB steps).
 */
static inline void ll_audprc_dac_set_vol_r(AUDPRC_TypeDef *AUDPRCx, uint32_t rough_vol,
        uint32_t fine_vol)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG0,
               AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_R | AUDPRC_DAC_PATH_CFG0_FINE_VOL_R,
               MAKE_REG_VAL(rough_vol, AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_R_Msk,
                            AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_R_Pos) |
               MAKE_REG_VAL(fine_vol, AUDPRC_DAC_PATH_CFG0_FINE_VOL_R_Msk,
                            AUDPRC_DAC_PATH_CFG0_FINE_VOL_R_Pos));
}

/**
 * @brief Set the DAC mixer left volume (DAC_PATH_CFG0 ROUGH/FINE_VOL_L).
 * @param[in] AUDPRCx   AUDPRC instance pointer.
 * @param[in] rough_vol Rough volume (4 bits, 6 dB steps).
 * @param[in] fine_vol  Fine volume (4 bits, 0.5 dB steps).
 */
static inline void ll_audprc_dac_set_vol_l(AUDPRC_TypeDef *AUDPRCx, uint32_t rough_vol,
        uint32_t fine_vol)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG0,
               AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_L | AUDPRC_DAC_PATH_CFG0_FINE_VOL_L,
               MAKE_REG_VAL(rough_vol, AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_L_Msk,
                            AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_L_Pos) |
               MAKE_REG_VAL(fine_vol, AUDPRC_DAC_PATH_CFG0_FINE_VOL_L_Msk,
                            AUDPRC_DAC_PATH_CFG0_FINE_VOL_L_Pos));
}

/**
 * @brief Clear the SRC channel internal data (DAC_PATH_CFG1.SRC_CH_CLR).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_dac_src_ch_clear(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_SRC_CH_CLR);
}

/**
 * @brief Check whether the SRC channel clear is done (DAC_PATH_CFG1.SRC_CH_CLR_DONE).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @return Non-zero when the SRC channel clear is done.
 */
static inline uint32_t ll_audprc_dac_is_src_ch_clear_done(AUDPRC_TypeDef *AUDPRCx)
{
    return READ_BIT(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_SRC_CH_CLR_DONE);
}

/**
 * @brief Configure a SRC half-band filter stage (DAC_PATH_CFG1.SRC_HBFx_EN/MODE).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] stage   HBF stage in range 1..3.
 * @param[in] en      Non-zero enables the stage.
 * @param[in] mode    Mode: 0 = upsampling, 1 = downsampling.
 */
static inline void ll_audprc_dac_config_src_hbf(AUDPRC_TypeDef *AUDPRCx, uint32_t stage,
        uint32_t en, uint32_t mode)
{
    uint32_t en_msk;
    uint32_t mode_msk;
    uint32_t value = 0U;

    if (stage == 1U)
    {
        en_msk = AUDPRC_DAC_PATH_CFG1_SRC_HBF1_EN;
        mode_msk = AUDPRC_DAC_PATH_CFG1_SRC_HBF1_MODE;
        value = MAKE_REG_VAL(mode, AUDPRC_DAC_PATH_CFG1_SRC_HBF1_MODE_Msk,
                             AUDPRC_DAC_PATH_CFG1_SRC_HBF1_MODE_Pos) |
                (en ? AUDPRC_DAC_PATH_CFG1_SRC_HBF1_EN : 0U);
    }
    else if (stage == 2U)
    {
        en_msk = AUDPRC_DAC_PATH_CFG1_SRC_HBF2_EN;
        mode_msk = AUDPRC_DAC_PATH_CFG1_SRC_HBF2_MODE;
        value = MAKE_REG_VAL(mode, AUDPRC_DAC_PATH_CFG1_SRC_HBF2_MODE_Msk,
                             AUDPRC_DAC_PATH_CFG1_SRC_HBF2_MODE_Pos) |
                (en ? AUDPRC_DAC_PATH_CFG1_SRC_HBF2_EN : 0U);
    }
    else
    {
        en_msk = AUDPRC_DAC_PATH_CFG1_SRC_HBF3_EN;
        mode_msk = AUDPRC_DAC_PATH_CFG1_SRC_HBF3_MODE;
        value = MAKE_REG_VAL(mode, AUDPRC_DAC_PATH_CFG1_SRC_HBF3_MODE_Msk,
                             AUDPRC_DAC_PATH_CFG1_SRC_HBF3_MODE_Pos) |
                (en ? AUDPRC_DAC_PATH_CFG1_SRC_HBF3_EN : 0U);
    }
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG1, en_msk | mode_msk, value);
}

/**
 * @brief Enable the SRC channels (DAC_PATH_CFG1.SRC_CH_EN).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch_en   Channel enable (2 bits).
 */
static inline void ll_audprc_dac_set_src_ch_en(AUDPRC_TypeDef *AUDPRCx, uint32_t ch_en)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_SRC_CH_EN,
               MAKE_REG_VAL(ch_en, AUDPRC_DAC_PATH_CFG1_SRC_CH_EN_Msk,
                            AUDPRC_DAC_PATH_CFG1_SRC_CH_EN_Pos));
}

/**
 * @brief Clear the EQ internal state (DAC_PATH_CFG1.EQ_CLR).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_dac_eq_clear(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_CLR);
}

/**
 * @brief Check whether the EQ clear has completed (DAC_PATH_CFG1.EQ_CLR_DONE).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @return Non-zero when the EQ clear is done.
 */
static inline uint32_t ll_audprc_dac_is_eq_clear_done(AUDPRC_TypeDef *AUDPRCx)
{
    return READ_BIT(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_CLR_DONE);
}

/**
 * @brief Set the EQ stage count (DAC_PATH_CFG1.EQ_STAGE).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] stage   EQ stage count (4 bits, max 10).
 */
static inline void ll_audprc_dac_set_eq_stage(AUDPRC_TypeDef *AUDPRCx, uint32_t stage)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_STAGE,
               MAKE_REG_VAL(stage, AUDPRC_DAC_PATH_CFG1_EQ_STAGE_Msk,
                            AUDPRC_DAC_PATH_CFG1_EQ_STAGE_Pos));
}

/**
 * @brief Set the EQ channel enable (DAC_PATH_CFG1.EQ_CH_EN).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ch_en   Channel enable (2 bits): 0 = bypass, 1 = left,
 *                    2 = right, 3 = both.
 */
static inline void ll_audprc_dac_set_eq_ch_en(AUDPRC_TypeDef *AUDPRCx, uint32_t ch_en)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_CH_EN,
               MAKE_REG_VAL(ch_en, AUDPRC_DAC_PATH_CFG1_EQ_CH_EN_Msk,
                            AUDPRC_DAC_PATH_CFG1_EQ_CH_EN_Pos));
}

/**
 * @brief Select a DAC mux right input source (DAC_PATH_CFG1.MUXRSRC0/1).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] slot    Mux slot (0 or 1).
 * @param[in] src     Source (3 bits): 0..3 = TX_CH0/1/RX_CH0/1, 4 = mute.
 */
static inline void ll_audprc_dac_set_mux_r_src(AUDPRC_TypeDef *AUDPRCx, uint32_t slot,
        uint32_t src)
{
    if (slot == 0U)
    {
        MODIFY_REG(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_MUXRSRC0,
                   MAKE_REG_VAL(src, AUDPRC_DAC_PATH_CFG1_MUXRSRC0_Msk,
                                AUDPRC_DAC_PATH_CFG1_MUXRSRC0_Pos));
    }
    else
    {
        MODIFY_REG(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_MUXRSRC1,
                   MAKE_REG_VAL(src, AUDPRC_DAC_PATH_CFG1_MUXRSRC1_Msk,
                                AUDPRC_DAC_PATH_CFG1_MUXRSRC1_Pos));
    }
}

/**
 * @brief Select a DAC mux left input source (DAC_PATH_CFG1.MUXLSRC0/1).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] slot    Mux slot (0 or 1).
 * @param[in] src     Source (3 bits): 0..3 = TX_CH0/1/RX_CH0/1, 4 = mute.
 */
static inline void ll_audprc_dac_set_mux_l_src(AUDPRC_TypeDef *AUDPRCx, uint32_t slot,
        uint32_t src)
{
    if (slot == 0U)
    {
        MODIFY_REG(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_MUXLSRC0,
                   MAKE_REG_VAL(src, AUDPRC_DAC_PATH_CFG1_MUXLSRC0_Msk,
                                AUDPRC_DAC_PATH_CFG1_MUXLSRC0_Pos));
    }
    else
    {
        MODIFY_REG(AUDPRCx->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_MUXLSRC1,
                   MAKE_REG_VAL(src, AUDPRC_DAC_PATH_CFG1_MUXLSRC1_Msk,
                                AUDPRC_DAC_PATH_CFG1_MUXLSRC1_Pos));
    }
}

/**
 * @brief Configure the DAC SRC sinc filter (DAC_PATH_CFG2).
 * @param[in] AUDPRCx   AUDPRC instance pointer.
 * @param[in] ratio     Sinc ratio in S31.30 format (31 bits).
 * @param[in] sinc_en   Non-zero enables the sinc filter.
 */
static inline void ll_audprc_dac_set_src_sinc(AUDPRC_TypeDef *AUDPRCx, uint32_t ratio,
        uint32_t sinc_en)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG2,
               AUDPRC_DAC_PATH_CFG2_SINC_RATIO | AUDPRC_DAC_PATH_CFG2_SRC_SINC_EN,
               MAKE_REG_VAL(ratio, AUDPRC_DAC_PATH_CFG2_SINC_RATIO_Msk,
                            AUDPRC_DAC_PATH_CFG2_SINC_RATIO_Pos) |
               (sinc_en ? AUDPRC_DAC_PATH_CFG2_SRC_SINC_EN : 0U));
}

/**
 * @brief Get the DAC mixer right ramp status (DAC_PATH_CFG3.RAMP_STAT_R).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @return Right ramp status (2 bits).
 */
static inline uint32_t ll_audprc_dac_get_ramp_stat_r(AUDPRC_TypeDef *AUDPRCx)
{
    return GET_REG_VAL2(AUDPRCx->DAC_PATH_CFG3, AUDPRC_DAC_PATH_CFG3_RAMP_STAT_R);
}

/**
 * @brief Configure the DAC mixer right ramp (DAC_PATH_CFG3).
 * @param[in] AUDPRCx    AUDPRC instance pointer.
 * @param[in] en         Non-zero enables volume ramping.
 * @param[in] slow       Non-zero ramps slowly (0.5 dB steps), zero directly.
 * @param[in] zero_adj   Non-zero adjusts during zero-volume crossing.
 * @param[in] interval   Ramp interval (4 bits).
 */
static inline void ll_audprc_dac_set_ramp_r(AUDPRC_TypeDef *AUDPRCx, uint32_t en,
        uint32_t slow, uint32_t zero_adj,
        uint32_t interval)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG3,
               AUDPRC_DAC_PATH_CFG3_RAMP_EN_R | AUDPRC_DAC_PATH_CFG3_RAMP_MODE_R |
               AUDPRC_DAC_PATH_CFG3_ZERO_ADJUST_EN_R |
               AUDPRC_DAC_PATH_CFG3_RAMP_INTERVAL_R,
               (en ? AUDPRC_DAC_PATH_CFG3_RAMP_EN_R : 0U) |
               (slow ? AUDPRC_DAC_PATH_CFG3_RAMP_MODE_R : 0U) |
               (zero_adj ? AUDPRC_DAC_PATH_CFG3_ZERO_ADJUST_EN_R : 0U) |
               MAKE_REG_VAL(interval, AUDPRC_DAC_PATH_CFG3_RAMP_INTERVAL_R_Msk,
                            AUDPRC_DAC_PATH_CFG3_RAMP_INTERVAL_R_Pos));
}

/**
 * @brief Get the DAC mixer left ramp status (DAC_PATH_CFG3.RAMP_STAT_L).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @return Left ramp status (2 bits).
 */
static inline uint32_t ll_audprc_dac_get_ramp_stat_l(AUDPRC_TypeDef *AUDPRCx)
{
    return GET_REG_VAL2(AUDPRCx->DAC_PATH_CFG3, AUDPRC_DAC_PATH_CFG3_RAMP_STAT_L);
}

/**
 * @brief Configure the DAC mixer left ramp (DAC_PATH_CFG3).
 * @param[in] AUDPRCx    AUDPRC instance pointer.
 * @param[in] en         Non-zero enables volume ramping.
 * @param[in] slow       Non-zero ramps slowly (0.5 dB steps), zero directly.
 * @param[in] zero_adj   Non-zero adjusts during zero-volume crossing.
 * @param[in] interval   Ramp interval (4 bits).
 */
static inline void ll_audprc_dac_set_ramp_l(AUDPRC_TypeDef *AUDPRCx, uint32_t en,
        uint32_t slow, uint32_t zero_adj,
        uint32_t interval)
{
    MODIFY_REG(AUDPRCx->DAC_PATH_CFG3,
               AUDPRC_DAC_PATH_CFG3_RAMP_EN_L | AUDPRC_DAC_PATH_CFG3_RAMP_MODE_L |
               AUDPRC_DAC_PATH_CFG3_ZERO_ADJUST_EN_L |
               AUDPRC_DAC_PATH_CFG3_RAMP_INTERVAL_L,
               (en ? AUDPRC_DAC_PATH_CFG3_RAMP_EN_L : 0U) |
               (slow ? AUDPRC_DAC_PATH_CFG3_RAMP_MODE_L : 0U) |
               (zero_adj ? AUDPRC_DAC_PATH_CFG3_ZERO_ADJUST_EN_L : 0U) |
               MAKE_REG_VAL(interval, AUDPRC_DAC_PATH_CFG3_RAMP_INTERVAL_L_Msk,
                            AUDPRC_DAC_PATH_CFG3_RAMP_INTERVAL_L_Pos));
}

/**
 * @brief Enable RX-to-TX loopback (ADC_PATH_CFG0.RX2TX_LOOPBACK).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_rx2tx_loopback_enable(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_RX2TX_LOOPBACK);
}

/**
 * @brief Disable RX-to-TX loopback (ADC_PATH_CFG0.RX2TX_LOOPBACK = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_rx2tx_loopback_disable(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_RX2TX_LOOPBACK);
}

/**
 * @brief Swap the ADC path left/right data (ADC_PATH_CFG0.DATA_SWAP).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_swap_data(AUDPRC_TypeDef *AUDPRCx)
{
    SET_BIT(AUDPRCx->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_DATA_SWAP);
}

/**
 * @brief Disable the ADC path data swap (ADC_PATH_CFG0.DATA_SWAP = 0).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 */
static inline void ll_audprc_adc_disable_swap(AUDPRC_TypeDef *AUDPRCx)
{
    CLEAR_BIT(AUDPRCx->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_DATA_SWAP);
}

/*==============================================================================
 * ADC Path (ADC_PATH_CFG0)
 *============================================================================*/

/**
 * @brief Select the ADC path source (ADC_PATH_CFG0.SRC_SEL).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] ext     Non-zero selects the external interface, zero the codec.
 */
static inline void ll_audprc_adc_set_src_sel(AUDPRC_TypeDef *AUDPRCx, uint32_t ext)
{
    MODIFY_REG(AUDPRCx->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_SRC_SEL,
               ext ? AUDPRC_ADC_PATH_CFG0_SRC_SEL : 0U);
}

/**
 * @brief Set the ADC path right volume (ADC_PATH_CFG0 ROUGH/FINE_VOL_R).
 * @param[in] AUDPRCx   AUDPRC instance pointer.
 * @param[in] rough_vol Rough volume (4 bits, 6 dB steps).
 * @param[in] fine_vol  Fine volume (4 bits, 0.5 dB steps).
 */
static inline void ll_audprc_adc_set_vol_r(AUDPRC_TypeDef *AUDPRCx, uint32_t rough_vol,
        uint32_t fine_vol)
{
    MODIFY_REG(AUDPRCx->ADC_PATH_CFG0,
               AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_R | AUDPRC_ADC_PATH_CFG0_FINE_VOL_R,
               MAKE_REG_VAL(rough_vol, AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_R_Msk,
                            AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_R_Pos) |
               MAKE_REG_VAL(fine_vol, AUDPRC_ADC_PATH_CFG0_FINE_VOL_R_Msk,
                            AUDPRC_ADC_PATH_CFG0_FINE_VOL_R_Pos));
}

/**
 * @brief Set the ADC path left volume (ADC_PATH_CFG0 ROUGH/FINE_VOL_L).
 * @param[in] AUDPRCx   AUDPRC instance pointer.
 * @param[in] rough_vol Rough volume (4 bits, 6 dB steps).
 * @param[in] fine_vol  Fine volume (4 bits, 0.5 dB steps).
 */
static inline void ll_audprc_adc_set_vol_l(AUDPRC_TypeDef *AUDPRCx, uint32_t rough_vol,
        uint32_t fine_vol)
{
    MODIFY_REG(AUDPRCx->ADC_PATH_CFG0,
               AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_L | AUDPRC_ADC_PATH_CFG0_FINE_VOL_L,
               MAKE_REG_VAL(rough_vol, AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_L_Msk,
                            AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_L_Pos) |
               MAKE_REG_VAL(fine_vol, AUDPRC_ADC_PATH_CFG0_FINE_VOL_L_Msk,
                            AUDPRC_ADC_PATH_CFG0_FINE_VOL_L_Pos));
}

/*==============================================================================
 * DAC Equalizer Coefficients (DAC_EQ_CFG0..CFG49)
 *============================================================================*/

/**
 * @brief Write a DAC EQ coefficient register (DAC_EQ_CFGx.COEF, 24 bits).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] idx     Coefficient index in range 0..49.
 * @param[in] coef    Coefficient value (24 bits).
 */
static inline void ll_audprc_dac_set_eq_coef(AUDPRC_TypeDef *AUDPRCx, uint32_t idx,
        uint32_t coef)
{
    volatile uint32_t *eq = &AUDPRCx->DAC_EQ_CFG0;

    if (idx < 50U)
    {
        WRITE_REG(eq[idx], coef);
    }
}

/**
 * @brief Read a DAC EQ coefficient register (DAC_EQ_CFGx.COEF, 24 bits).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] idx     Coefficient index in range 0..49.
 * @return Coefficient value (24 bits).
 */
static inline uint32_t ll_audprc_dac_get_eq_coef(AUDPRC_TypeDef *AUDPRCx, uint32_t idx)
{
    volatile uint32_t *eq = &AUDPRCx->DAC_EQ_CFG0;

    return (idx < 50U) ? READ_REG(eq[idx]) : 0U;
}

/*==============================================================================
 * Interrupts (IRQ: status + mask)
 *============================================================================*/

/** @defgroup LL_AUDPRC_IRQ AUDPRC IRQ Status / Mask Bits */
/** @{ */
#define LL_AUDPRC_IRQ_TX0_FIFO_OF      AUDPRC_IRQ_TX0_FIFO_OF
#define LL_AUDPRC_IRQ_TX1_FIFO_OF      AUDPRC_IRQ_TX1_FIFO_OF
#define LL_AUDPRC_IRQ_TX2_FIFO_OF      AUDPRC_IRQ_TX2_FIFO_OF
#define LL_AUDPRC_IRQ_TX3_FIFO_OF      AUDPRC_IRQ_TX3_FIFO_OF
#define LL_AUDPRC_IRQ_RX0_FIFO_UF      AUDPRC_IRQ_RX0_FIFO_UF
#define LL_AUDPRC_IRQ_RX1_FIFO_UF      AUDPRC_IRQ_RX1_FIFO_UF
#define LL_AUDPRC_IRQ_TX_OUT_FIFO_UF   AUDPRC_IRQ_TX_OUT_FIFO_UF
#define LL_AUDPRC_IRQ_RX_IN_FIFO_OF    AUDPRC_IRQ_RX_IN_FIFO_OF
#define LL_AUDPRC_IRQ_TX_OUT0_FIFO_UF  AUDPRC_IRQ_TX_OUT0_FIFO_UF
#define LL_AUDPRC_IRQ_TX_OUT1_FIFO_UF  AUDPRC_IRQ_TX_OUT1_FIFO_UF

#define LL_AUDPRC_IRQ_TX0_FIFO_OF_MASK      AUDPRC_IRQ_TX0_FIFO_OF_MASK
#define LL_AUDPRC_IRQ_TX1_FIFO_OF_MASK      AUDPRC_IRQ_TX1_FIFO_OF_MASK
#define LL_AUDPRC_IRQ_TX2_FIFO_OF_MASK      AUDPRC_IRQ_TX2_FIFO_OF_MASK
#define LL_AUDPRC_IRQ_TX3_FIFO_OF_MASK      AUDPRC_IRQ_TX3_FIFO_OF_MASK
#define LL_AUDPRC_IRQ_RX0_FIFO_UF_MASK      AUDPRC_IRQ_RX0_FIFO_UF_MASK
#define LL_AUDPRC_IRQ_RX1_FIFO_UF_MASK      AUDPRC_IRQ_RX1_FIFO_UF_MASK
#define LL_AUDPRC_IRQ_TX_OUT_FIFO_UF_MASK   AUDPRC_IRQ_TX_OUT_FIFO_UF_MASK
#define LL_AUDPRC_IRQ_RX_IN_FIFO_OF_MASK    AUDPRC_IRQ_RX_IN_FIFO_OF_MASK
#define LL_AUDPRC_IRQ_TX_OUT0_FIFO_UF_MASK  AUDPRC_IRQ_TX_OUT0_FIFO_UF_MASK
#define LL_AUDPRC_IRQ_TX_OUT1_FIFO_UF_MASK  AUDPRC_IRQ_TX_OUT1_FIFO_UF_MASK
/** @} */

/* Missing combined bit masks (not in regs/audprc.h) defined here as LL-local;
 * removed once the regs header is updated upstream. */
#define LL_AUDPRC_IRQ_STAT_MASK 0x000003FFU /* bits 0..9: rw1c status flags */
#define LL_AUDPRC_IRQ_MSK_MASK  0x03FF0000U

/* bits 16..25: R/W masks (1=mask) */

/**
 * @brief Get the raw IRQ status register (IRQ).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @return IRQ register value.
 */
static inline uint32_t ll_audprc_get_irq_status(AUDPRC_TypeDef *AUDPRCx)
{
    return READ_REG(AUDPRCx->IRQ);
}

/**
 * @brief Clear IRQ status flags preserving the R/W mask bits.
 * @param[in] AUDPRCx  AUDPRC instance pointer.
 * @param[in] status   Status flags to clear (write 1, rw1c).
 */
static inline void ll_audprc_clear_irq_status(AUDPRC_TypeDef *AUDPRCx, uint32_t status)
{
    /* Preserve the high-bit mask field while clearing rw1c status bits. */
    WRITE_REG(AUDPRCx->IRQ,
              (READ_REG(AUDPRCx->IRQ) & LL_AUDPRC_IRQ_MSK_MASK) |
              (status & LL_AUDPRC_IRQ_STAT_MASK));
}

/**
 * @brief Get the IRQ mask field (IRQ, bits 16..25).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @return IRQ mask field value.
 */
static inline uint32_t ll_audprc_get_irq_mask(AUDPRC_TypeDef *AUDPRCx)
{
    return READ_REG(AUDPRCx->IRQ) & LL_AUDPRC_IRQ_MSK_MASK;
}

/**
 * @brief Set the IRQ mask field (IRQ bits 16..25).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @param[in] mask    Mask bits to set (1 = masked, 0 = enabled).
 */
static inline void ll_audprc_set_irq_mask(AUDPRC_TypeDef *AUDPRCx, uint32_t mask)
{
    /* IRQ low bits (0..9) are rw1c status: MODIFY_REG would write their read
     * value back and clear any pending flag. Write the whole register, keeping
     * only the R/W mask field and writing 0 to the rw1c status bits (no effect). */
    WRITE_REG(AUDPRCx->IRQ, mask & LL_AUDPRC_IRQ_MSK_MASK);
}

/*==============================================================================
 * Reserved control / status (RESERVED_IN / RESERVED_OUT)
 *============================================================================*/

/**
 * @brief Set a reserved control byte (RESERVED_IN.CTRL_x).
 * @param[in] AUDPRCx  AUDPRC instance pointer.
 * @param[in] byte_idx Control byte index in range 0..2 (CTRL_0/1/2).
 * @param[in] val      Control value (8 bits).
 */
static inline void ll_audprc_set_reserved_ctrl(AUDPRC_TypeDef *AUDPRCx,
        uint32_t byte_idx, uint32_t val)
{
    switch (byte_idx)
    {
    case 0U:
        MODIFY_REG(AUDPRCx->RESERVED_IN, AUDPRC_RESERVED_IN_CTRL_0,
                   MAKE_REG_VAL(val, AUDPRC_RESERVED_IN_CTRL_0_Msk,
                                AUDPRC_RESERVED_IN_CTRL_0_Pos));
        break;
    case 1U:
        MODIFY_REG(AUDPRCx->RESERVED_IN, AUDPRC_RESERVED_IN_CTRL_1,
                   MAKE_REG_VAL(val, AUDPRC_RESERVED_IN_CTRL_1_Msk,
                                AUDPRC_RESERVED_IN_CTRL_1_Pos));
        break;
    case 2U:
        MODIFY_REG(AUDPRCx->RESERVED_IN, AUDPRC_RESERVED_IN_CTRL_2,
                   MAKE_REG_VAL(val, AUDPRC_RESERVED_IN_CTRL_2_Msk,
                                AUDPRC_RESERVED_IN_CTRL_2_Pos));
        break;
    default:
        break;
    }
}

/**
 * @brief Get the reserved status byte (RESERVED_OUT.STAT).
 * @param[in] AUDPRCx AUDPRC instance pointer.
 * @return Status value (8 bits).
 */
static inline uint32_t ll_audprc_get_reserved_stat(AUDPRC_TypeDef *AUDPRCx)
{
    return GET_REG_VAL2(AUDPRCx->RESERVED_OUT, AUDPRC_RESERVED_OUT_STAT);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_AUDPRC_H */
