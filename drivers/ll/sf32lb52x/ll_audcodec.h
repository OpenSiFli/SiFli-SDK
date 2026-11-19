/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_AUDCODEC_H
#define __LL_AUDCODEC_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup LL_AUDCODEC_IRQ_FLAGS AUDCODEC Interrupt Flags */
/** @{ */
#define LL_AUDCODEC_IRQ_DAC_CH0_APB_OF  AUDCODEC_IRQ_DAC_CH0_APB_OF
#define LL_AUDCODEC_IRQ_DAC_CH0_OUT_UF  AUDCODEC_IRQ_DAC_CH0_OUT_UF
#define LL_AUDCODEC_IRQ_DAC_CH0_STB_OF  AUDCODEC_IRQ_DAC_CH0_STB_OF
#define LL_AUDCODEC_IRQ_DAC_CH1_APB_OF  AUDCODEC_IRQ_DAC_CH1_APB_OF
#define LL_AUDCODEC_IRQ_DAC_CH1_OUT_UF  AUDCODEC_IRQ_DAC_CH1_OUT_UF
#define LL_AUDCODEC_IRQ_DAC_CH1_STB_OF  AUDCODEC_IRQ_DAC_CH1_STB_OF
#define LL_AUDCODEC_IRQ_ADC_CH0_APB_OF  AUDCODEC_IRQ_ADC_CH0_APB_OF
#define LL_AUDCODEC_IRQ_ADC_CH0_APB_UF  AUDCODEC_IRQ_ADC_CH0_APB_UF
#define LL_AUDCODEC_IRQ_ADC_CH0_SAT     AUDCODEC_IRQ_ADC_CH0_SAT
#define LL_AUDCODEC_IRQ_ADC_CH1_APB_OF  AUDCODEC_IRQ_ADC_CH1_APB_OF
#define LL_AUDCODEC_IRQ_ADC_CH1_APB_UF  AUDCODEC_IRQ_ADC_CH1_APB_UF
#define LL_AUDCODEC_IRQ_ADC_CH1_SAT     AUDCODEC_IRQ_ADC_CH1_SAT

#define LL_AUDCODEC_IRQ_MASK_DAC_CH0_APB_OF  AUDCODEC_IRQ_MSK_DAC_CH0_APB_OF
#define LL_AUDCODEC_IRQ_MASK_DAC_CH0_OUT_UF  AUDCODEC_IRQ_MSK_DAC_CH0_OUT_UF
#define LL_AUDCODEC_IRQ_MASK_DAC_CH0_STB_OF  AUDCODEC_IRQ_MSK_DAC_CH0_STB_OF
#define LL_AUDCODEC_IRQ_MASK_DAC_CH1_APB_OF  AUDCODEC_IRQ_MSK_DAC_CH1_APB_OF
#define LL_AUDCODEC_IRQ_MASK_DAC_CH1_OUT_UF  AUDCODEC_IRQ_MSK_DAC_CH1_OUT_UF
#define LL_AUDCODEC_IRQ_MASK_DAC_CH1_STB_OF  AUDCODEC_IRQ_MSK_DAC_CH1_STB_OF
#define LL_AUDCODEC_IRQ_MASK_ADC_CH0_APB_OF  AUDCODEC_IRQ_MSK_ADC_CH0_APB_OF
#define LL_AUDCODEC_IRQ_MASK_ADC_CH0_APB_UF  AUDCODEC_IRQ_MSK_ADC_CH0_APB_UF
#define LL_AUDCODEC_IRQ_MASK_ADC_CH0_SAT     AUDCODEC_IRQ_MSK_ADC_CH0_SAT
#define LL_AUDCODEC_IRQ_MASK_ADC_CH1_APB_OF  AUDCODEC_IRQ_MSK_ADC_CH1_APB_OF
#define LL_AUDCODEC_IRQ_MASK_ADC_CH1_APB_UF  AUDCODEC_IRQ_MSK_ADC_CH1_APB_UF
#define LL_AUDCODEC_IRQ_MASK_ADC_CH1_SAT     AUDCODEC_IRQ_MSK_ADC_CH1_SAT
/** @} */

/**
 * @brief Get the function ID (ID.FUNC).
 * @param[in] codec Audio codec instance pointer.
 * @return Function ID.
 */
static inline uint32_t ll_audcodec_get_id(AUDCODEC_TypeDef *codec)
{
    return READ_REG(codec->ID);
}

/**
 * @brief Set the ADC enable delay count (CFG.ADC_EN_DLY_SEL).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] sel   Enable delay count.
 */
static inline void ll_audcodec_set_adc_en_dly_sel(AUDCODEC_TypeDef *codec, uint32_t sel)
{
    MODIFY_REG(codec->CFG, AUDCODEC_CFG_ADC_EN_DLY_SEL,
               MAKE_REG_VAL(sel, AUDCODEC_CFG_ADC_EN_DLY_SEL_Msk,
                            AUDCODEC_CFG_ADC_EN_DLY_SEL_Pos));
}

static inline void ll_audcodec_enable_dac_1k_mode(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->CFG, AUDCODEC_CFG_DAC_1K_MODE);
}

static inline void ll_audcodec_disable_dac_1k_mode(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->CFG, AUDCODEC_CFG_DAC_1K_MODE);
}

static inline void ll_audcodec_enable_dac(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->CFG, AUDCODEC_CFG_DAC_ENABLE);
}

static inline void ll_audcodec_disable_dac(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->CFG, AUDCODEC_CFG_DAC_ENABLE);
}

/**
 * @file ll_audcodec.h
 * @brief Header-only low-level Audio Codec (AUDCODEC) APIs for SF32LB52x.
 *
 * AUDCODEC provides stereo ADC (2 channels) and stereo DAC (2 channels)
 * with internal PLL and bandgap reference.
 */

/*==============================================================================
 * Basic Control
 *============================================================================*/

static inline void ll_audcodec_enable_adc(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->CFG, AUDCODEC_CFG_ADC_ENABLE);
}

static inline void ll_audcodec_disable_adc(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->CFG, AUDCODEC_CFG_ADC_ENABLE);
}

static inline uint32_t ll_audcodec_get_interrupt_status(AUDCODEC_TypeDef *codec)
{
    return READ_REG(codec->IRQ);
}

/**
 * @brief Clear interrupt status flags.
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] mask Interrupt status flags to clear.
 * @note AUDCODEC IRQ status flags are cleared by writing one.
 */
static inline void ll_audcodec_clear_interrupt_status(AUDCODEC_TypeDef *codec, uint32_t mask)
{
    WRITE_REG(codec->IRQ, mask);
}

/*==============================================================================
 * Interrupt
 *============================================================================*/

static inline void ll_audcodec_enable_interrupt(AUDCODEC_TypeDef *codec, uint32_t mask)
{
    SET_BIT(codec->IRQ_MSK, mask);
}

static inline void ll_audcodec_disable_interrupt(AUDCODEC_TypeDef *codec, uint32_t mask)
{
    CLEAR_BIT(codec->IRQ_MSK, mask);
}

/*==============================================================================
 * Path Configuration
 *============================================================================*/

/**
 * @brief Write the full DAC path configuration (DAC_CFG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw DAC_CFG value.
 */
static inline void ll_audcodec_dac_set_path_config(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_DAC_CFG_OSR_SEL | AUDCODEC_DAC_CFG_OP_MODE |
                    AUDCODEC_DAC_CFG_PATH_RESET | AUDCODEC_DAC_CFG_CLK_SRC_SEL |
                    AUDCODEC_DAC_CFG_CLK_DIV | AUDCODEC_DAC_CFG_MANUAL_OSR_MODE |
                    AUDCODEC_DAC_CFG_HBF1_BYPASS_M | AUDCODEC_DAC_CFG_HBF2_BYPASS_M |
                    AUDCODEC_DAC_CFG_HBF3_BYPASS_M | AUDCODEC_DAC_CFG_HBF4_BYPASS_M |
                    AUDCODEC_DAC_CFG_INTERP3_BYPASS_M | AUDCODEC_DAC_CFG_SINC_RATE_SEL_M |
                    AUDCODEC_DAC_CFG_SDM_OSR_SEL_M;
    MODIFY_REG(codec->DAC_CFG, mask, conf & mask);
}

/**
 * @brief Assert the DAC path reset (DAC_CFG.PATH_RESET = 1).
 * @param[in] codec AUDCODEC instance pointer.
 */
static inline void ll_audcodec_dac_path_set_reset(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->DAC_CFG, AUDCODEC_DAC_CFG_PATH_RESET);
}

/**
 * @brief Release the DAC path reset (DAC_CFG.PATH_RESET = 0).
 * @param[in] codec AUDCODEC instance pointer.
 */
static inline void ll_audcodec_dac_path_clear_reset(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->DAC_CFG, AUDCODEC_DAC_CFG_PATH_RESET);
}

/**
 * @brief Write the full ADC path configuration (ADC_CFG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw ADC_CFG value.
 */
static inline void ll_audcodec_adc_set_path_config(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_ADC_CFG_OSR_SEL | AUDCODEC_ADC_CFG_OP_MODE |
                    AUDCODEC_ADC_CFG_PATH_RESET | AUDCODEC_ADC_CFG_CLK_SRC_SEL |
                    AUDCODEC_ADC_CFG_CLK_DIV;
    MODIFY_REG(codec->ADC_CFG, mask, conf & mask);
}

/**
 * @brief Assert the ADC path reset (ADC_CFG.PATH_RESET = 1).
 * @param[in] codec AUDCODEC instance pointer.
 */
static inline void ll_audcodec_adc_path_set_reset(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->ADC_CFG, AUDCODEC_ADC_CFG_PATH_RESET);
}

/**
 * @brief Release the ADC path reset (ADC_CFG.PATH_RESET = 0).
 * @param[in] codec AUDCODEC instance pointer.
 */
static inline void ll_audcodec_adc_path_clear_reset(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->ADC_CFG, AUDCODEC_ADC_CFG_PATH_RESET);
}

/**
 * @brief Get APB FIFO fill-level status.
 */
static inline uint32_t ll_audcodec_get_status(AUDCODEC_TypeDef *codec)
{
    return READ_REG(codec->APB_STAT) &
           (AUDCODEC_APB_STAT_DAC_CH0_FIFO_CNT | AUDCODEC_APB_STAT_DAC_CH1_FIFO_CNT |
            AUDCODEC_APB_STAT_ADC_CH0_FIFO_CNT | AUDCODEC_APB_STAT_ADC_CH1_FIFO_CNT);
}

/**
 * @brief Write the full ADC channel configuration (ADC_CHx_CFG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @param[in] conf  Raw ADC_CHx_CFG value.
 * @note Invalid channel indices are ignored.
 */
static inline void ll_audcodec_adc_set_channel_config(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t conf)
{
    if (ch == 0U)
    {
        uint32_t mask = AUDCODEC_ADC_CH0_CFG_ENABLE |
                        AUDCODEC_ADC_CH0_CFG_HPF_BYPASS |
                        AUDCODEC_ADC_CH0_CFG_HPF_COEF |
                        AUDCODEC_ADC_CH0_CFG_STB_INV |
                        AUDCODEC_ADC_CH0_CFG_DMA_EN |
                        AUDCODEC_ADC_CH0_CFG_ROUGH_VOL |
                        AUDCODEC_ADC_CH0_CFG_FINE_VOL |
                        AUDCODEC_ADC_CH0_CFG_DATA_FORMAT |
                        AUDCODEC_ADC_CH0_CFG_SAT_DET_EN |
                        AUDCODEC_ADC_CH0_CFG_SAT_DET_LEN;
        MODIFY_REG(codec->ADC_CH0_CFG, mask, conf & mask);
    }
    else if (ch == 1U)
    {
        uint32_t mask = AUDCODEC_ADC_CH1_CFG_ENABLE |
                        AUDCODEC_ADC_CH1_CFG_HPF_BYPASS |
                        AUDCODEC_ADC_CH1_CFG_HPF_COEF |
                        AUDCODEC_ADC_CH1_CFG_STB_INV |
                        AUDCODEC_ADC_CH1_CFG_DMA_EN |
                        AUDCODEC_ADC_CH1_CFG_ROUGH_VOL |
                        AUDCODEC_ADC_CH1_CFG_FINE_VOL |
                        AUDCODEC_ADC_CH1_CFG_DATA_FORMAT |
                        AUDCODEC_ADC_CH1_CFG_SAT_DET_EN |
                        AUDCODEC_ADC_CH1_CFG_SAT_DET_LEN;
        MODIFY_REG(codec->ADC_CH1_CFG, mask, conf & mask);
    }
}

/**
 * @brief Enable/disable the ADC channel DMA (ADC_CHx_CFG.DMA_EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @param[in] en    Non-zero to enable DMA, zero to disable.
 */
static inline void ll_audcodec_adc_set_channel_dma_enable(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t en)
{
    if (ch == 0U)
    {
        MODIFY_REG(codec->ADC_CH0_CFG, AUDCODEC_ADC_CH0_CFG_DMA_EN,
                   en ? AUDCODEC_ADC_CH0_CFG_DMA_EN : 0UL);
    }
    else if (ch == 1U)
    {
        MODIFY_REG(codec->ADC_CH1_CFG, AUDCODEC_ADC_CH1_CFG_DMA_EN,
                   en ? AUDCODEC_ADC_CH1_CFG_DMA_EN : 0UL);
    }
}

/*==============================================================================
 * ADC Configuration
 *============================================================================*/

/**
 * @brief Enable or disable an ADC channel.
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch Channel index, 0 or 1.
 * @param[in] en Non-zero to enable the channel, zero to disable it.
 * @note Invalid channel indices are ignored.
 */
static inline void ll_audcodec_adc_set_channel_enable(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t en)
{
    if (ch == 0U)
    {
        if (en)
        {
            SET_BIT(codec->ADC_CH0_CFG, AUDCODEC_ADC_CH0_CFG_ENABLE);
        }
        else
        {
            CLEAR_BIT(codec->ADC_CH0_CFG, AUDCODEC_ADC_CH0_CFG_ENABLE);
        }
    }
    else if (ch == 1U)
    {
        if (en)
        {
            SET_BIT(codec->ADC_CH1_CFG, AUDCODEC_ADC_CH1_CFG_ENABLE);
        }
        else
        {
            CLEAR_BIT(codec->ADC_CH1_CFG, AUDCODEC_ADC_CH1_CFG_ENABLE);
        }
    }
}

/**
 * @brief Write the full DAC channel configuration (DAC_CHx_CFG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @param[in] conf  Raw DAC_CHx_CFG value.
 * @note Invalid channel indices are ignored.
 */
static inline void ll_audcodec_dac_set_channel_config(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t conf)
{
    if (ch == 0U)
    {
        uint32_t mask = AUDCODEC_DAC_CH0_CFG_ENABLE |
                        AUDCODEC_DAC_CH0_CFG_DOUT_MUTE |
                        AUDCODEC_DAC_CH0_CFG_DEM_MODE |
                        AUDCODEC_DAC_CH0_CFG_DMA_EN |
                        AUDCODEC_DAC_CH0_CFG_ROUGH_VOL |
                        AUDCODEC_DAC_CH0_CFG_FINE_VOL |
                        AUDCODEC_DAC_CH0_CFG_DATA_FORMAT |
                        AUDCODEC_DAC_CH0_CFG_SINC_GAIN |
                        AUDCODEC_DAC_CH0_CFG_DITHER_GAIN |
                        AUDCODEC_DAC_CH0_CFG_DITHER_EN |
                        AUDCODEC_DAC_CH0_CFG_CLK_ANA_POL;
        MODIFY_REG(codec->DAC_CH0_CFG, mask, conf & mask);
    }
    else if (ch == 1U)
    {
        uint32_t mask = AUDCODEC_DAC_CH1_CFG_ENABLE |
                        AUDCODEC_DAC_CH1_CFG_DOUT_MUTE |
                        AUDCODEC_DAC_CH1_CFG_DEM_MODE |
                        AUDCODEC_DAC_CH1_CFG_DMA_EN |
                        AUDCODEC_DAC_CH1_CFG_ROUGH_VOL |
                        AUDCODEC_DAC_CH1_CFG_FINE_VOL |
                        AUDCODEC_DAC_CH1_CFG_DATA_FORMAT |
                        AUDCODEC_DAC_CH1_CFG_SINC_GAIN |
                        AUDCODEC_DAC_CH1_CFG_DITHER_GAIN |
                        AUDCODEC_DAC_CH1_CFG_DITHER_EN |
                        AUDCODEC_DAC_CH1_CFG_CLK_ANA_POL;
        MODIFY_REG(codec->DAC_CH1_CFG, mask, conf & mask);
    }
}

/**
 * @brief Read the DAC channel configuration (DAC_CHx_CFG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @return Raw DAC_CHx_CFG value, or 0 for an invalid channel index.
 */
static inline uint32_t ll_audcodec_dac_get_channel_config(AUDCODEC_TypeDef *codec, uint32_t ch)
{
    if (ch == 0U)
    {
        return READ_REG(codec->DAC_CH0_CFG) &
               (AUDCODEC_DAC_CH0_CFG_ENABLE | AUDCODEC_DAC_CH0_CFG_DOUT_MUTE |
                AUDCODEC_DAC_CH0_CFG_DEM_MODE | AUDCODEC_DAC_CH0_CFG_STB_FIFO_CNT |
                AUDCODEC_DAC_CH0_CFG_DMA_EN | AUDCODEC_DAC_CH0_CFG_ROUGH_VOL |
                AUDCODEC_DAC_CH0_CFG_FINE_VOL | AUDCODEC_DAC_CH0_CFG_DATA_FORMAT |
                AUDCODEC_DAC_CH0_CFG_SINC_GAIN | AUDCODEC_DAC_CH0_CFG_DITHER_GAIN |
                AUDCODEC_DAC_CH0_CFG_DITHER_EN | AUDCODEC_DAC_CH0_CFG_CLK_ANA_POL);
    }
    if (ch == 1U)
    {
        return READ_REG(codec->DAC_CH1_CFG) &
               (AUDCODEC_DAC_CH1_CFG_ENABLE | AUDCODEC_DAC_CH1_CFG_DOUT_MUTE |
                AUDCODEC_DAC_CH1_CFG_DEM_MODE | AUDCODEC_DAC_CH1_CFG_STB_FIFO_CNT |
                AUDCODEC_DAC_CH1_CFG_DMA_EN | AUDCODEC_DAC_CH1_CFG_ROUGH_VOL |
                AUDCODEC_DAC_CH1_CFG_FINE_VOL | AUDCODEC_DAC_CH1_CFG_DATA_FORMAT |
                AUDCODEC_DAC_CH1_CFG_SINC_GAIN | AUDCODEC_DAC_CH1_CFG_DITHER_GAIN |
                AUDCODEC_DAC_CH1_CFG_DITHER_EN | AUDCODEC_DAC_CH1_CFG_CLK_ANA_POL);
    }
    return 0U;
}

/**
 * @brief Set the DAC channel volume (DAC_CHx_CFG.ROUGH_VOL/FINE_VOL).
 * @param[in] codec  AUDCODEC instance pointer.
 * @param[in] ch     Channel index, 0 or 1.
 * @param[in] rough  Rough volume code, 0..15.
 * @param[in] fine   Fine volume code, 0..12 (0xC = mute).
 */
static inline void ll_audcodec_dac_set_channel_volume(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t rough, uint32_t fine)
{
    if (ch == 0U)
    {
        MODIFY_REG(codec->DAC_CH0_CFG,
                   AUDCODEC_DAC_CH0_CFG_ROUGH_VOL | AUDCODEC_DAC_CH0_CFG_FINE_VOL,
                   MAKE_REG_VAL(rough, AUDCODEC_DAC_CH0_CFG_ROUGH_VOL_Msk,
                                AUDCODEC_DAC_CH0_CFG_ROUGH_VOL_Pos) |
                   MAKE_REG_VAL(fine, AUDCODEC_DAC_CH0_CFG_FINE_VOL_Msk,
                                AUDCODEC_DAC_CH0_CFG_FINE_VOL_Pos));
    }
    else if (ch == 1U)
    {
        MODIFY_REG(codec->DAC_CH1_CFG,
                   AUDCODEC_DAC_CH1_CFG_ROUGH_VOL | AUDCODEC_DAC_CH1_CFG_FINE_VOL,
                   MAKE_REG_VAL(rough, AUDCODEC_DAC_CH1_CFG_ROUGH_VOL_Msk,
                                AUDCODEC_DAC_CH1_CFG_ROUGH_VOL_Pos) |
                   MAKE_REG_VAL(fine, AUDCODEC_DAC_CH1_CFG_FINE_VOL_Msk,
                                AUDCODEC_DAC_CH1_CFG_FINE_VOL_Pos));
    }
}

/**
 * @brief Set only the DAC channel fine volume (DAC_CHx_CFG.FINE_VOL).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @param[in] fine  Fine volume code, 0..12 (0xC = mute).
 */
static inline void ll_audcodec_dac_set_channel_fine_volume(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t fine)
{
    if (ch == 0U)
    {
        MODIFY_REG(codec->DAC_CH0_CFG, AUDCODEC_DAC_CH0_CFG_FINE_VOL,
                   MAKE_REG_VAL(fine, AUDCODEC_DAC_CH0_CFG_FINE_VOL_Msk,
                                AUDCODEC_DAC_CH0_CFG_FINE_VOL_Pos));
    }
    else if (ch == 1U)
    {
        MODIFY_REG(codec->DAC_CH1_CFG, AUDCODEC_DAC_CH1_CFG_FINE_VOL,
                   MAKE_REG_VAL(fine, AUDCODEC_DAC_CH1_CFG_FINE_VOL_Msk,
                                AUDCODEC_DAC_CH1_CFG_FINE_VOL_Pos));
    }
}

/**
 * @brief Enable/disable the DAC channel DMA (DAC_CHx_CFG.DMA_EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @param[in] en    Non-zero to enable DMA, zero to disable.
 */
static inline void ll_audcodec_dac_set_channel_dma_enable(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t en)
{
    if (ch == 0U)
    {
        MODIFY_REG(codec->DAC_CH0_CFG, AUDCODEC_DAC_CH0_CFG_DMA_EN,
                   en ? AUDCODEC_DAC_CH0_CFG_DMA_EN : 0UL);
    }
    else if (ch == 1U)
    {
        MODIFY_REG(codec->DAC_CH1_CFG, AUDCODEC_DAC_CH1_CFG_DMA_EN,
                   en ? AUDCODEC_DAC_CH1_CFG_DMA_EN : 0UL);
    }
}

/**
 * @brief Mute/unmute a DAC channel (DAC_CHx_CFG.DOUT_MUTE).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @param[in] mute  Non-zero to mute, zero to unmute.
 */
static inline void ll_audcodec_dac_set_channel_mute(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t mute)
{
    if (ch == 0U)
    {
        MODIFY_REG(codec->DAC_CH0_CFG, AUDCODEC_DAC_CH0_CFG_DOUT_MUTE,
                   mute ? AUDCODEC_DAC_CH0_CFG_DOUT_MUTE : 0UL);
    }
    else if (ch == 1U)
    {
        MODIFY_REG(codec->DAC_CH1_CFG, AUDCODEC_DAC_CH1_CFG_DOUT_MUTE,
                   mute ? AUDCODEC_DAC_CH1_CFG_DOUT_MUTE : 0UL);
    }
}

/*==============================================================================
 * DAC Configuration
 *============================================================================*/

/**
 * @brief Enable or disable a DAC channel.
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch Channel index, 0 or 1.
 * @param[in] en Non-zero to enable the channel, zero to disable it.
 * @note Invalid channel indices are ignored.
 */
static inline void ll_audcodec_dac_set_channel_enable(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t en)
{
    if (ch == 0U)
    {
        if (en)
        {
            SET_BIT(codec->DAC_CH0_CFG, AUDCODEC_DAC_CH0_CFG_ENABLE);
        }
        else
        {
            CLEAR_BIT(codec->DAC_CH0_CFG, AUDCODEC_DAC_CH0_CFG_ENABLE);
        }
    }
    else if (ch == 1U)
    {
        if (en)
        {
            SET_BIT(codec->DAC_CH1_CFG, AUDCODEC_DAC_CH1_CFG_ENABLE);
        }
        else
        {
            CLEAR_BIT(codec->DAC_CH1_CFG, AUDCODEC_DAC_CH1_CFG_ENABLE);
        }
    }
}

static inline uint32_t ll_audcodec_dac_get_channel_config_ext(AUDCODEC_TypeDef *codec,
        uint32_t ch)
{
    if (ch == 0U)
    {
        return READ_REG(codec->DAC_CH0_CFG_EXT) &
               (AUDCODEC_DAC_CH0_CFG_EXT_RAMP_STAT |
                AUDCODEC_DAC_CH0_CFG_EXT_RAMP_INTERVAL |
                AUDCODEC_DAC_CH0_CFG_EXT_ZERO_ADJUST_EN |
                AUDCODEC_DAC_CH0_CFG_EXT_RAMP_MODE |
                AUDCODEC_DAC_CH0_CFG_EXT_RAMP_EN);
    }
    if (ch == 1U)
    {
        return READ_REG(codec->DAC_CH1_CFG_EXT) &
               (AUDCODEC_DAC_CH1_CFG_EXT_RAMP_STAT |
                AUDCODEC_DAC_CH1_CFG_EXT_RAMP_INTERVAL |
                AUDCODEC_DAC_CH1_CFG_EXT_ZERO_ADJUST_EN |
                AUDCODEC_DAC_CH1_CFG_EXT_RAMP_MODE |
                AUDCODEC_DAC_CH1_CFG_EXT_RAMP_EN);
    }
    return 0U;
}

/**
 * @brief Write the full DAC channel extended configuration (DAC_CHx_CFG_EXT).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @param[in] conf  Raw DAC_CHx_CFG_EXT value.
 * @note Invalid channel indices are ignored.
 */
static inline void ll_audcodec_dac_set_channel_config_ext(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t conf)
{
    if (ch == 0U)
    {
        uint32_t mask = AUDCODEC_DAC_CH0_CFG_EXT_RAMP_EN |
                        AUDCODEC_DAC_CH0_CFG_EXT_RAMP_MODE |
                        AUDCODEC_DAC_CH0_CFG_EXT_ZERO_ADJUST_EN |
                        AUDCODEC_DAC_CH0_CFG_EXT_RAMP_INTERVAL;
        MODIFY_REG(codec->DAC_CH0_CFG_EXT, mask, conf & mask);
    }
    else if (ch == 1U)
    {
        uint32_t mask = AUDCODEC_DAC_CH1_CFG_EXT_RAMP_EN |
                        AUDCODEC_DAC_CH1_CFG_EXT_RAMP_MODE |
                        AUDCODEC_DAC_CH1_CFG_EXT_ZERO_ADJUST_EN |
                        AUDCODEC_DAC_CH1_CFG_EXT_RAMP_INTERVAL;
        MODIFY_REG(codec->DAC_CH1_CFG_EXT, mask, conf & mask);
    }
}

/**
 * @brief Read an ADC channel APB data entry.
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch Channel index, 0 or 1.
 * @return ADC data, or 0 for an invalid channel index.
 * @note The ADC path must be configured in APB or raw-data APB mode.
 */
static inline uint32_t ll_audcodec_adc_read_entry(AUDCODEC_TypeDef *codec, uint32_t ch)
{
    if (ch == 0U)
    {
        return READ_REG(codec->ADC_CH0_ENTRY) & AUDCODEC_ADC_CH0_ENTRY_DATA;
    }
    if (ch == 1U)
    {
        return READ_REG(codec->ADC_CH1_ENTRY) & AUDCODEC_ADC_CH1_ENTRY_DATA;
    }
    return 0U;
}

/**
 * @brief Write a DAC channel APB data entry.
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch Channel index, 0 or 1.
 * @param[in] data DAC data to write.
 * @note The DAC path must be configured in APB mode. Invalid channel indices
 *       are ignored.
 */
static inline void ll_audcodec_dac_write_entry(AUDCODEC_TypeDef *codec, uint32_t ch, uint32_t data)
{
    if (ch == 0U)
    {
        WRITE_REG(codec->DAC_CH0_ENTRY, data & AUDCODEC_DAC_CH0_ENTRY_DATA);
    }
    else if (ch == 1U)
    {
        WRITE_REG(codec->DAC_CH1_ENTRY, data & AUDCODEC_DAC_CH1_ENTRY_DATA);
    }
}

/**
 * @brief Write the full DAC channel debug register (DAC_CHx_DEBUG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch    Channel index, 0 or 1.
 * @param[in] conf  Raw DAC_CHx_DEBUG value.
 * @note Invalid channel indices are ignored.
 */
static inline void ll_audcodec_dac_set_channel_debug(AUDCODEC_TypeDef *codec, uint32_t ch,
        uint32_t conf)
{
    if (ch == 0U)
    {
        MODIFY_REG(codec->DAC_CH0_DEBUG,
                   AUDCODEC_DAC_CH0_DEBUG_BYPASS | AUDCODEC_DAC_CH0_DEBUG_DATA_OUT,
                   conf & (AUDCODEC_DAC_CH0_DEBUG_BYPASS |
                           AUDCODEC_DAC_CH0_DEBUG_DATA_OUT));
    }
    else if (ch == 1U)
    {
        MODIFY_REG(codec->DAC_CH1_DEBUG,
                   AUDCODEC_DAC_CH1_DEBUG_BYPASS | AUDCODEC_DAC_CH1_DEBUG_DATA_OUT,
                   conf & (AUDCODEC_DAC_CH1_DEBUG_BYPASS |
                           AUDCODEC_DAC_CH1_DEBUG_DATA_OUT));
    }
}

/**
 * @brief Set a DAC channel DC offset.
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] ch Channel index, 0 or 1.
 * @param[in] dc 24-bit DC offset value.
 * @note Invalid channel indices are ignored.
 */
static inline void ll_audcodec_dac_set_dc_offset(AUDCODEC_TypeDef *codec, uint32_t ch, uint32_t dc)
{
    if (ch == 0U)
    {
        WRITE_REG(codec->DAC_CH0_DC, dc & AUDCODEC_DAC_CH0_DC_OFFSET_Msk);
    }
    else if (ch == 1U)
    {
        WRITE_REG(codec->DAC_CH1_DC, dc & AUDCODEC_DAC_CH1_DC_OFFSET_Msk);
    }
}

/**
 * @brief Configure the DC test (COMMON_CFG).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] tr    DC test point select (3 bits).
 * @param[in] br    DC test block select (3 bits).
 * @param[in] mr    DC test macro select (3 bits).
 */
static inline void ll_audcodec_set_dc_test_config(AUDCODEC_TypeDef *codec, uint32_t tr,
        uint32_t br, uint32_t mr)
{
    MODIFY_REG(codec->COMMON_CFG,
               AUDCODEC_COMMON_CFG_DC_TR | AUDCODEC_COMMON_CFG_DC_BR |
               AUDCODEC_COMMON_CFG_DC_MR,
               MAKE_REG_VAL(tr, AUDCODEC_COMMON_CFG_DC_TR_Msk, AUDCODEC_COMMON_CFG_DC_TR_Pos) |
               MAKE_REG_VAL(br, AUDCODEC_COMMON_CFG_DC_BR_Msk, AUDCODEC_COMMON_CFG_DC_BR_Pos) |
               MAKE_REG_VAL(mr, AUDCODEC_COMMON_CFG_DC_MR_Msk, AUDCODEC_COMMON_CFG_DC_MR_Pos));
}

/**
 * @brief Write the full bandgap configuration 0 (BG_CFG0).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw BG_CFG0 value.
 */
static inline void ll_audcodec_bg_set_cfg0(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_BG_CFG0_EN | AUDCODEC_BG_CFG0_LP_MODE |
                    AUDCODEC_BG_CFG0_VREF_SEL | AUDCODEC_BG_CFG0_EN_CHOP |
                    AUDCODEC_BG_CFG0_EN_SMPL | AUDCODEC_BG_CFG0_EN_RCFLT |
                    AUDCODEC_BG_CFG0_MIC_VREF_SEL | AUDCODEC_BG_CFG0_EN_AMP |
                    AUDCODEC_BG_CFG0_SET_VC;
    MODIFY_REG(codec->BG_CFG0, mask, conf & mask);
}

/**
 * @brief Enable/disable the bandgap sample switch (BG_CFG0.EN_SMPL).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_bg_set_smpl(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->BG_CFG0, AUDCODEC_BG_CFG0_EN_SMPL,
               en ? AUDCODEC_BG_CFG0_EN_SMPL : 0UL);
}

/*==============================================================================
 * Bandgap / Reference
 *============================================================================*/

/**
 * @brief Enable/disable the bandgap reference (BG_CFG0.EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_bg_set_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->BG_CFG0, AUDCODEC_BG_CFG0_EN,
               en ? AUDCODEC_BG_CFG0_EN : 0UL);
}

/**
 * @brief Write the full bandgap configuration 1 (BG_CFG1).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw BG_CFG1 value.
 */
static inline void ll_audcodec_bg_set_cfg1(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    WRITE_REG(codec->BG_CFG1, conf);
}

/**
 * @brief Write the full bandgap configuration 2 (BG_CFG2).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw BG_CFG2 value.
 */
static inline void ll_audcodec_bg_set_cfg2(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    WRITE_REG(codec->BG_CFG2, conf);
}

static inline void ll_audcodec_refgen_set_config(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_REFGEN_CFG_EN | AUDCODEC_REFGEN_CFG_EN_CHOP |
                    AUDCODEC_REFGEN_CFG_BM | AUDCODEC_REFGEN_CFG_LP_MODE |
                    AUDCODEC_REFGEN_CFG_LV_MODE | AUDCODEC_REFGEN_CFG_RZSEL |
                    AUDCODEC_REFGEN_CFG_DISCHG;
    MODIFY_REG(codec->REFGEN_CFG, mask, conf & mask);
}

/**
 * @brief Enable/disable the reference generator LV mode (REFGEN_CFG.LV_MODE).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_refgen_set_lv_mode(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->REFGEN_CFG, AUDCODEC_REFGEN_CFG_LV_MODE,
               en ? AUDCODEC_REFGEN_CFG_LV_MODE : 0UL);
}

/**
 * @brief Enable/disable the reference generator chopping (REFGEN_CFG.EN_CHOP).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_refgen_set_chop_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->REFGEN_CFG, AUDCODEC_REFGEN_CFG_EN_CHOP,
               en ? AUDCODEC_REFGEN_CFG_EN_CHOP : 0UL);
}

/**
 * @brief Enable/disable the reference generator (REFGEN_CFG.EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_refgen_set_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->REFGEN_CFG, AUDCODEC_REFGEN_CFG_EN,
               en ? AUDCODEC_REFGEN_CFG_EN : 0UL);
}

/*==============================================================================
 * PLL Control
 *============================================================================*/

static inline void ll_audcodec_pll_set_cfg0(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_PLL_CFG0_ICP_OS_SEL | AUDCODEC_PLL_CFG0_OPEN |
                    AUDCODEC_PLL_CFG0_ICP_SEL | AUDCODEC_PLL_CFG0_SEL_VREF_ANA |
                    AUDCODEC_PLL_CFG0_EN_ANA | AUDCODEC_PLL_CFG0_VCO_LP_MODE |
                    AUDCODEC_PLL_CFG0_FC_VCO | AUDCODEC_PLL_CFG0_EN_VCO_FLT |
                    AUDCODEC_PLL_CFG0_SEL_VREF_VCO | AUDCODEC_PLL_CFG0_EN_VCO |
                    AUDCODEC_PLL_CFG0_EN_IARY | AUDCODEC_PLL_CFG0_SEL_CKREF;
    MODIFY_REG(codec->PLL_CFG0, mask, conf & mask);
}

/**
 * @brief Enable/disable the PLL analog blocks (PLL_CFG0.EN_IARY/EN_VCO/EN_ANA).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_pll_set_analog_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG0,
               AUDCODEC_PLL_CFG0_EN_IARY | AUDCODEC_PLL_CFG0_EN_VCO |
               AUDCODEC_PLL_CFG0_EN_ANA,
               en ? (AUDCODEC_PLL_CFG0_EN_IARY | AUDCODEC_PLL_CFG0_EN_VCO |
                     AUDCODEC_PLL_CFG0_EN_ANA) :
               0UL);
}

/**
 * @brief Set the PLL VCO frequency code (PLL_CFG0.FC_VCO).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] fc    VCO frequency code, 0..31.
 */
static inline void ll_audcodec_pll_set_fc_vco(AUDCODEC_TypeDef *codec, uint32_t fc)
{
    MODIFY_REG(codec->PLL_CFG0, AUDCODEC_PLL_CFG0_FC_VCO,
               MAKE_REG_VAL(fc, AUDCODEC_PLL_CFG0_FC_VCO_Msk, AUDCODEC_PLL_CFG0_FC_VCO_Pos));
}

/**
 * @brief Set the PLL charge-pump current (PLL_CFG0.ICP_SEL).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] sel   ICP selection code.
 */
static inline void ll_audcodec_pll_set_icp_sel(AUDCODEC_TypeDef *codec, uint32_t sel)
{
    MODIFY_REG(codec->PLL_CFG0, AUDCODEC_PLL_CFG0_ICP_SEL,
               MAKE_REG_VAL(sel, AUDCODEC_PLL_CFG0_ICP_SEL_Msk,
                            AUDCODEC_PLL_CFG0_ICP_SEL_Pos));
}

/**
 * @brief Put the audio PLL into open-loop mode.
 * @note This controls PLL_CFG0.OPEN only; it does not power up or configure
 *       the audio PLL.
 */
static inline void ll_audcodec_pll_enable_open_loop(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->PLL_CFG0, AUDCODEC_PLL_CFG0_OPEN);
}

/**
 * @brief Return the audio PLL to closed-loop mode.
 */
static inline void ll_audcodec_pll_disable_open_loop(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->PLL_CFG0, AUDCODEC_PLL_CFG0_OPEN);
}

/**
 * @brief Read the PLL loop-filter configuration (PLL_CFG1).
 * @param[in] codec AUDCODEC instance pointer.
 * @return Raw PLL_CFG1 value.
 */
static inline uint32_t ll_audcodec_pll_get_cfg1(AUDCODEC_TypeDef *codec)
{
    return READ_REG(codec->PLL_CFG1);
}

/**
 * @brief Write the full PLL loop-filter configuration (PLL_CFG1).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw PLL_CFG1 value.
 */
static inline void ll_audcodec_pll_set_cfg1(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_PLL_CFG1_R3_SEL | AUDCODEC_PLL_CFG1_RZ_SEL |
                    AUDCODEC_PLL_CFG1_C2_SEL | AUDCODEC_PLL_CFG1_CZ_SEL |
                    AUDCODEC_PLL_CFG1_CSD_RST | AUDCODEC_PLL_CFG1_CSD_EN;
    MODIFY_REG(codec->PLL_CFG1, mask, conf & mask);
}

/**
 * @brief Enable/disable the PLL CSD (PLL_CFG1.CSD_EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_pll_set_csd_en(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG1, AUDCODEC_PLL_CFG1_CSD_EN,
               en ? AUDCODEC_PLL_CFG1_CSD_EN : 0UL);
}

/**
 * @brief Assert/de-assert the PLL CSD reset (PLL_CFG1.CSD_RST).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to assert reset, zero to release.
 */
static inline void ll_audcodec_pll_set_csd_rst(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG1, AUDCODEC_PLL_CFG1_CSD_RST,
               en ? AUDCODEC_PLL_CFG1_CSD_RST : 0UL);
}

/**
 * @brief Enable/disable the PLL loop-filter VCIN input (PLL_CFG2.EN_LF_VCIN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_pll_set_en_lf_vcin(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG2, AUDCODEC_PLL_CFG2_EN_LF_VCIN,
               en ? AUDCODEC_PLL_CFG2_EN_LF_VCIN : 0UL);
}

static inline void ll_audcodec_pll_set_cfg2(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_PLL_CFG2_MMD_STG | AUDCODEC_PLL_CFG2_TR_DTEST |
                    AUDCODEC_PLL_CFG2_TE_DTEST | AUDCODEC_PLL_CFG2_RSTB_SYNC_EN |
                    AUDCODEC_PLL_CFG2_RSTB | AUDCODEC_PLL_CFG2_SEL_VREF_DIG |
                    AUDCODEC_PLL_CFG2_EN_DIG | AUDCODEC_PLL_CFG2_EN_LF_TSTBUF |
                    AUDCODEC_PLL_CFG2_SEL_LF_VCIN | AUDCODEC_PLL_CFG2_EN_LF_VCIN;
    MODIFY_REG(codec->PLL_CFG2, mask, conf & mask);
}

/**
 * @brief Enable/disable the PLL digital block (PLL_CFG2.EN_DIG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_pll_set_en_dig(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG2, AUDCODEC_PLL_CFG2_EN_DIG,
               en ? AUDCODEC_PLL_CFG2_EN_DIG : 0UL);
}

/**
 * @brief Assert the PLL reset (PLL_CFG2.RSTB = 0, active-low reset).
 * @param[in] codec AUDCODEC instance pointer.
 */
static inline void ll_audcodec_pll_assert_reset(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->PLL_CFG2, AUDCODEC_PLL_CFG2_RSTB);
}

/**
 * @brief Release the PLL reset (PLL_CFG2.RSTB = 1).
 * @param[in] codec AUDCODEC instance pointer.
 */
static inline void ll_audcodec_pll_release_reset(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->PLL_CFG2, AUDCODEC_PLL_CFG2_RSTB);
}

/**
 * @brief Write the full PLL frequency configuration (PLL_CFG3).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw PLL_CFG3 value.
 */
static inline void ll_audcodec_pll_set_cfg3(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_PLL_CFG3_SDIN | AUDCODEC_PLL_CFG3_FCW |
                    AUDCODEC_PLL_CFG3_SDMIN_BYPASS | AUDCODEC_PLL_CFG3_SDM_MODE |
                    AUDCODEC_PLL_CFG3_EN_SDM_DITHER | AUDCODEC_PLL_CFG3_SDM_DITHER |
                    AUDCODEC_PLL_CFG3_EN_SDM | AUDCODEC_PLL_CFG3_SDMCLK_POL;
    MODIFY_REG(codec->PLL_CFG3, mask, conf & mask);
}

/**
 * @brief Enable/disable the PLL SDM (PLL_CFG3.EN_SDM).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_pll_set_en_sdm(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG3, AUDCODEC_PLL_CFG3_EN_SDM,
               en ? AUDCODEC_PLL_CFG3_EN_SDM : 0UL);
}

/**
 * @brief Enable/disable the PLL SDM input bypass (PLL_CFG3.SDMIN_BYPASS).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to bypass, zero to use SDM input.
 */
static inline void ll_audcodec_pll_set_sdmin_bypass(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG3, AUDCODEC_PLL_CFG3_SDMIN_BYPASS,
               en ? AUDCODEC_PLL_CFG3_SDMIN_BYPASS : 0UL);
}

/**
 * @brief Trigger the PLL SDM update (PLL_CFG3.SDM_UPDATE).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to pulse update, zero to clear.
 */
static inline void ll_audcodec_pll_set_sdm_update(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG3, AUDCODEC_PLL_CFG3_SDM_UPDATE,
               en ? AUDCODEC_PLL_CFG3_SDM_UPDATE : 0UL);
}

/**
 * @brief Enable/disable the PLL digital clock output (PLL_CFG4.EN_CLK_DIG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_pll_set_en_clk_dig(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CFG4, AUDCODEC_PLL_CFG4_EN_CLK_DIG,
               en ? AUDCODEC_PLL_CFG4_EN_CLK_DIG : 0UL);
}

/**
 * @brief Configure the PLL DAC clocks (PLL_CFG4 clock fields).
 * @param[in] codec            AUDCODEC instance pointer.
 * @param[in] en_chop_dac      Non-zero to enable chop DAC clock.
 * @param[in] en_clk_dac       Non-zero to enable DAC clock.
 * @param[in] sel_clk_dac_src  DAC clock source select.
 * @param[in] sel_clk_dac      DAC clock select.
 * @param[in] en_clk_dig       Non-zero to enable digital clock.
 */
static inline void ll_audcodec_pll_set_dac_clk_config(AUDCODEC_TypeDef *codec,
        uint32_t en_chop_dac, uint32_t en_clk_dac,
        uint32_t sel_clk_dac_src,
        uint32_t sel_clk_dac, uint32_t en_clk_dig)
{
    MODIFY_REG(codec->PLL_CFG4,
               AUDCODEC_PLL_CFG4_EN_CLK_CHOP_DAC | AUDCODEC_PLL_CFG4_EN_CLK_DAC |
               AUDCODEC_PLL_CFG4_SEL_CLK_DAC_SOURCE | AUDCODEC_PLL_CFG4_SEL_CLK_DAC |
               AUDCODEC_PLL_CFG4_EN_CLK_DIG,
               MAKE_REG_VAL(en_chop_dac, AUDCODEC_PLL_CFG4_EN_CLK_CHOP_DAC_Msk,
                            AUDCODEC_PLL_CFG4_EN_CLK_CHOP_DAC_Pos) |
               MAKE_REG_VAL(en_clk_dac, AUDCODEC_PLL_CFG4_EN_CLK_DAC_Msk,
                            AUDCODEC_PLL_CFG4_EN_CLK_DAC_Pos) |
               MAKE_REG_VAL(sel_clk_dac_src, AUDCODEC_PLL_CFG4_SEL_CLK_DAC_SOURCE_Msk,
                            AUDCODEC_PLL_CFG4_SEL_CLK_DAC_SOURCE_Pos) |
               MAKE_REG_VAL(sel_clk_dac, AUDCODEC_PLL_CFG4_SEL_CLK_DAC_Msk,
                            AUDCODEC_PLL_CFG4_SEL_CLK_DAC_Pos) |
               MAKE_REG_VAL(en_clk_dig, AUDCODEC_PLL_CFG4_EN_CLK_DIG_Msk,
                            AUDCODEC_PLL_CFG4_EN_CLK_DIG_Pos));
}

static inline void ll_audcodec_pll_set_cfg4(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_PLL_CFG4_DIVB_CLK_CHOP_DAC |
                    AUDCODEC_PLL_CFG4_DIVA_CLK_CHOP_DAC |
                    AUDCODEC_PLL_CFG4_EN_CLK_CHOP_DAC |
                    AUDCODEC_PLL_CFG4_DIVA_CLK_DAC | AUDCODEC_PLL_CFG4_EN_CLK_DAC |
                    AUDCODEC_PLL_CFG4_SEL_CLK_DAC | AUDCODEC_PLL_CFG4_SEL_CLK_DAC_SOURCE |
                    AUDCODEC_PLL_CFG4_SEL_CLK_DIG | AUDCODEC_PLL_CFG4_CLK_DIG_STR |
                    AUDCODEC_PLL_CFG4_DIVA_CLK_DIG | AUDCODEC_PLL_CFG4_EN_CLK_DIG;
    MODIFY_REG(codec->PLL_CFG4, mask, conf & mask);
}

static inline void ll_audcodec_pll_set_cfg5(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_PLL_CFG5_DIVB_CLK_CHOP_BG |
                    AUDCODEC_PLL_CFG5_DIVA_CLK_CHOP_BG | AUDCODEC_PLL_CFG5_EN_CLK_CHOP_BG |
                    AUDCODEC_PLL_CFG5_DIVB_CLK_CHOP_REFGEN |
                    AUDCODEC_PLL_CFG5_DIVA_CLK_CHOP_REFGEN |
                    AUDCODEC_PLL_CFG5_EN_CLK_CHOP_REFGEN |
                    AUDCODEC_PLL_CFG5_DIVB_CLK_CHOP_DAC2 |
                    AUDCODEC_PLL_CFG5_DIVA_CLK_CHOP_DAC2 |
                    AUDCODEC_PLL_CFG5_EN_CLK_CHOP_DAC2 |
                    AUDCODEC_PLL_CFG5_DIVA_CLK_DAC2 | AUDCODEC_PLL_CFG5_EN_CLK_DAC2 |
                    AUDCODEC_PLL_CFG5_SEL_CLK_DAC2;
    MODIFY_REG(codec->PLL_CFG5, mask, conf & mask);
}

/**
 * @brief Enable/disable the bandgap/refgen chopping clocks (PLL_CFG5).
 * @param[in] codec      AUDCODEC instance pointer.
 * @param[in] en_bg      Non-zero to enable the BG chop clock.
 * @param[in] en_refgen  Non-zero to enable the REFGEN chop clock.
 */
static inline void ll_audcodec_pll_set_chop_clocks(AUDCODEC_TypeDef *codec, uint32_t en_bg,
        uint32_t en_refgen)
{
    MODIFY_REG(codec->PLL_CFG5,
               AUDCODEC_PLL_CFG5_EN_CLK_CHOP_BG | AUDCODEC_PLL_CFG5_EN_CLK_CHOP_REFGEN,
               MAKE_REG_VAL(en_bg, AUDCODEC_PLL_CFG5_EN_CLK_CHOP_BG_Msk,
                            AUDCODEC_PLL_CFG5_EN_CLK_CHOP_BG_Pos) |
               MAKE_REG_VAL(en_refgen, AUDCODEC_PLL_CFG5_EN_CLK_CHOP_REFGEN_Msk,
                            AUDCODEC_PLL_CFG5_EN_CLK_CHOP_REFGEN_Pos));
}

/**
 * @brief Write the full ADC clock configuration (PLL_CFG6).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw PLL_CFG6 value.
 */
static inline void ll_audcodec_pll_set_cfg6(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_PLL_CFG6_SEL_TST_CLK | AUDCODEC_PLL_CFG6_EN_TST_CLK |
                    AUDCODEC_PLL_CFG6_EN_CLK_RCCAL | AUDCODEC_PLL_CFG6_SEL_CLK_CHOP_MICBIAS |
                    AUDCODEC_PLL_CFG6_EN_CLK_CHOP_MICBIAS | AUDCODEC_PLL_CFG6_SEL_CLK_ADC2 |
                    AUDCODEC_PLL_CFG6_DIVA_CLK_ADC2 | AUDCODEC_PLL_CFG6_EN_CLK_ADC2 |
                    AUDCODEC_PLL_CFG6_SEL_CLK_ADC1 | AUDCODEC_PLL_CFG6_DIVA_CLK_ADC1 |
                    AUDCODEC_PLL_CFG6_EN_CLK_ADC1 | AUDCODEC_PLL_CFG6_SEL_CLK_ADC0 |
                    AUDCODEC_PLL_CFG6_DIVA_CLK_ADC0 | AUDCODEC_PLL_CFG6_EN_CLK_ADC0 |
                    AUDCODEC_PLL_CFG6_SEL_CLK_ADC_SOURCE;
    MODIFY_REG(codec->PLL_CFG6, mask, conf & mask);
}

static inline uint32_t ll_audcodec_pll_get_status(AUDCODEC_TypeDef *codec)
{
    return READ_REG(codec->PLL_STAT) &
           (AUDCODEC_PLL_STAT_UNLOCK | AUDCODEC_PLL_STAT_SLIPPED_UP |
            AUDCODEC_PLL_STAT_SLIPPED_DN);
}

/**
 * @brief Check whether the audio PLL reports a locked state.
 * @return 1 when locked, otherwise 0.
 * @note The result is meaningful only after the PLL has been powered and
 *       configured.
 */
static inline uint32_t ll_audcodec_pll_is_locked(AUDCODEC_TypeDef *codec)
{
    return READ_BIT(codec->PLL_STAT, AUDCODEC_PLL_STAT_UNLOCK) ? 0UL : 1UL;
}

/**
 * @brief Write the full PLL calibration configuration (PLL_CAL_CFG).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] conf  Raw PLL_CAL_CFG value.
 */
static inline void ll_audcodec_pll_set_cal_config(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_PLL_CAL_CFG_LEN | AUDCODEC_PLL_CAL_CFG_EN;
    MODIFY_REG(codec->PLL_CAL_CFG, mask, conf & mask);
}

/**
 * @brief Check whether the PLL calibration is done (PLL_CAL_CFG.DONE).
 * @param[in] codec AUDCODEC instance pointer.
 * @return Non-zero when calibration has completed.
 */
static inline uint32_t ll_audcodec_pll_is_cal_done(AUDCODEC_TypeDef *codec)
{
    return READ_BIT(codec->PLL_CAL_CFG, AUDCODEC_PLL_CAL_CFG_DONE) ? 1UL : 0UL;
}

/**
 * @brief Enable/disable the PLL calibration (PLL_CAL_CFG.EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to start calibration, zero to stop.
 */
static inline void ll_audcodec_pll_set_cal_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->PLL_CAL_CFG, AUDCODEC_PLL_CAL_CFG_EN,
               en ? AUDCODEC_PLL_CAL_CFG_EN : 0UL);
}

/**
 * @brief Read the PLL calibration result (PLL_CAL_RESULT).
 * @param[in] codec AUDCODEC instance pointer.
 * @return Raw PLL_CAL_RESULT value (PLL_CNT/XTAL_CNT fields).
 */
static inline uint32_t ll_audcodec_pll_get_cal_result(AUDCODEC_TypeDef *codec)
{
    return READ_REG(codec->PLL_CAL_RESULT) &
           (AUDCODEC_PLL_CAL_RESULT_PLL_CNT | AUDCODEC_PLL_CAL_RESULT_XTAL_CNT);
}

static inline void ll_audcodec_adc_set_capcode(AUDCODEC_TypeDef *codec, uint32_t capcode)
{
    MODIFY_REG(codec->ADC_ANA_CFG, AUDCODEC_ADC_ANA_CFG_CAPCODE,
               MAKE_REG_VAL(capcode, AUDCODEC_ADC_ANA_CFG_CAPCODE_Msk,
                            AUDCODEC_ADC_ANA_CFG_CAPCODE_Pos));
}

/**
 * @brief Enable/disable the ADC microphone bias (ADC_ANA_CFG.MICBIAS_EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_adc_set_micbias_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC_ANA_CFG, AUDCODEC_ADC_ANA_CFG_MICBIAS_EN,
               en ? AUDCODEC_ADC_ANA_CFG_MICBIAS_EN : 0UL);
}

/**
 * @brief Enable/disable the micbias chopping (ADC_ANA_CFG.MICBIAS_CHOP_EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_adc_set_micbias_chop_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC_ANA_CFG, AUDCODEC_ADC_ANA_CFG_MICBIAS_CHOP_EN,
               en ? AUDCODEC_ADC_ANA_CFG_MICBIAS_CHOP_EN : 0UL);
}

/**
 * @brief Set the ADC1 FSP code (ADC1_CFG1.FSP).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] fsp   FSP code.
 */
static inline void ll_audcodec_adc1_set_fsp(AUDCODEC_TypeDef *codec, uint32_t fsp)
{
    MODIFY_REG(codec->ADC1_CFG1, AUDCODEC_ADC1_CFG1_FSP,
               MAKE_REG_VAL(fsp, AUDCODEC_ADC1_CFG1_FSP_Msk, AUDCODEC_ADC1_CFG1_FSP_Pos));
}

static inline void ll_audcodec_adc1_set_config(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_ADC1_CFG1_PERI_BM | AUDCODEC_ADC1_CFG1_CLKOUT_INV |
                    AUDCODEC_ADC1_CFG1_VCMST | AUDCODEC_ADC1_CFG1_FCHOP_SEL |
                    AUDCODEC_ADC1_CFG1_VREF_SEL | AUDCODEC_ADC1_CFG1_BM_INT2 |
                    AUDCODEC_ADC1_CFG1_BM_INT1 | AUDCODEC_ADC1_CFG1_VST_SEL |
                    AUDCODEC_ADC1_CFG1_GC | AUDCODEC_ADC1_CFG1_DACN_EN |
                    AUDCODEC_ADC1_CFG1_DIFF_EN | AUDCODEC_ADC1_CFG1_FSP;
    MODIFY_REG(codec->ADC1_CFG1, mask, conf & mask);
}

/*==============================================================================
 * ADC Analog Path (ADC1_CFG1 / ADC1_CFG2 / ADC2_CFG2)
 *============================================================================*/

/**
 * @brief Enable/disable the ADC1 differential input (ADC1_CFG1.DIFF_EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_adc1_set_diff_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC1_CFG1, AUDCODEC_ADC1_CFG1_DIFF_EN,
               en ? AUDCODEC_ADC1_CFG1_DIFF_EN : 0UL);
}

/**
 * @brief Enable/disable the ADC1 DACN input (ADC1_CFG1.DACN_EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_adc1_set_dacn_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC1_CFG1, AUDCODEC_ADC1_CFG1_DACN_EN,
               en ? AUDCODEC_ADC1_CFG1_DACN_EN : 0UL);
}

/**
 * @brief Set the ADC1 gain code (ADC1_CFG1.GC).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] gc    Gain code.
 */
static inline void ll_audcodec_adc1_set_gc(AUDCODEC_TypeDef *codec, uint32_t gc)
{
    MODIFY_REG(codec->ADC1_CFG1, AUDCODEC_ADC1_CFG1_GC,
               MAKE_REG_VAL(gc, AUDCODEC_ADC1_CFG1_GC_Msk, AUDCODEC_ADC1_CFG1_GC_Pos));
}

/**
 * @brief Set the ADC1 VREF selection (ADC1_CFG1.VREF_SEL).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] sel   VREF selection code.
 */
static inline void ll_audcodec_adc1_set_vref_sel(AUDCODEC_TypeDef *codec, uint32_t sel)
{
    MODIFY_REG(codec->ADC1_CFG1, AUDCODEC_ADC1_CFG1_VREF_SEL,
               MAKE_REG_VAL(sel, AUDCODEC_ADC1_CFG1_VREF_SEL_Msk,
                            AUDCODEC_ADC1_CFG1_VREF_SEL_Pos));
}

/**
 * @brief Assert/de-assert the ADC1 VCM startup pulse (ADC1_CFG1.VCMST).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to assert, zero to release.
 */
static inline void ll_audcodec_adc1_set_vcmst(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC1_CFG1, AUDCODEC_ADC1_CFG1_VCMST,
               en ? AUDCODEC_ADC1_CFG1_VCMST : 0UL);
}

/**
 * @brief Enable/disable the ADC1 (ADC1_CFG2.EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_adc1_set_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC1_CFG2, AUDCODEC_ADC1_CFG2_EN,
               en ? AUDCODEC_ADC1_CFG2_EN : 0UL);
}

/**
 * @brief Assert/release the ADC1 reset (ADC1_CFG2.RSTB, active-low).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to assert (pin low), zero to release.
 */
static inline void ll_audcodec_adc1_set_rstb(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC1_CFG2, AUDCODEC_ADC1_CFG2_RSTB,
               en ? 0UL : AUDCODEC_ADC1_CFG2_RSTB);
}

static inline void ll_audcodec_adc1_set_chop_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC1_CFG2, AUDCODEC_ADC1_CFG2_CHOP_EN,
               en ? AUDCODEC_ADC1_CFG2_CHOP_EN : 0UL);
}

/**
 * @brief Assert/de-assert the ADC1 clear pulse (ADC1_CFG2.CLEAR).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to assert, zero to release.
 */
static inline void ll_audcodec_adc1_set_clear(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC1_CFG2, AUDCODEC_ADC1_CFG2_CLEAR,
               en ? AUDCODEC_ADC1_CFG2_CLEAR : 0UL);
}

/**
 * @brief Set the ADC2 sampling frequency (ADC2_CFG1.FSP).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] fsp   Sampling frequency select (2 bits).
 */
static inline void ll_audcodec_set_adc2_fsp(AUDCODEC_TypeDef *codec, uint32_t fsp)
{
    MODIFY_REG(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_FSP,
               MAKE_REG_VAL(fsp, AUDCODEC_ADC2_CFG1_FSP_Msk, AUDCODEC_ADC2_CFG1_FSP_Pos));
}

/**
 * @brief Set the ADC2 gain code (ADC2_CFG1.GC).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] gc    Gain code (5 bits).
 */
static inline void ll_audcodec_set_adc2_gain(AUDCODEC_TypeDef *codec, uint32_t gc)
{
    MODIFY_REG(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_GC,
               MAKE_REG_VAL(gc, AUDCODEC_ADC2_CFG1_GC_Msk, AUDCODEC_ADC2_CFG1_GC_Pos));
}

/**
 * @brief Set the ADC2 start voltage (ADC2_CFG1.VST_SEL).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] sel   Start voltage select (3 bits).
 */
static inline void ll_audcodec_set_adc2_vst_sel(AUDCODEC_TypeDef *codec, uint32_t sel)
{
    MODIFY_REG(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_VST_SEL,
               MAKE_REG_VAL(sel, AUDCODEC_ADC2_CFG1_VST_SEL_Msk, AUDCODEC_ADC2_CFG1_VST_SEL_Pos));
}

/**
 * @brief Set the ADC2 first opamp bias mode (ADC2_CFG1.BM_INT1).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] bm    Bias mode (3 bits).
 */
static inline void ll_audcodec_set_adc2_bm_int1(AUDCODEC_TypeDef *codec, uint32_t bm)
{
    MODIFY_REG(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_BM_INT1,
               MAKE_REG_VAL(bm, AUDCODEC_ADC2_CFG1_BM_INT1_Msk, AUDCODEC_ADC2_CFG1_BM_INT1_Pos));
}

/**
 * @brief Set the ADC2 second/third opamp bias mode (ADC2_CFG1.BM_INT2).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] bm    Bias mode (3 bits).
 */
static inline void ll_audcodec_set_adc2_bm_int2(AUDCODEC_TypeDef *codec, uint32_t bm)
{
    MODIFY_REG(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_BM_INT2,
               MAKE_REG_VAL(bm, AUDCODEC_ADC2_CFG1_BM_INT2_Msk, AUDCODEC_ADC2_CFG1_BM_INT2_Pos));
}

/**
 * @brief Set the ADC2 VREF code (ADC2_CFG1.VREF_SEL).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] sel   VREF select (3 bits).
 */
static inline void ll_audcodec_set_adc2_vref_sel(AUDCODEC_TypeDef *codec, uint32_t sel)
{
    MODIFY_REG(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_VREF_SEL,
               MAKE_REG_VAL(sel, AUDCODEC_ADC2_CFG1_VREF_SEL_Msk, AUDCODEC_ADC2_CFG1_VREF_SEL_Pos));
}

/**
 * @brief Set the ADC2 chopping frequency (ADC2_CFG1.FCHOP_SEL).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] sel   Chopping frequency select (2 bits).
 */
static inline void ll_audcodec_set_adc2_fchop_sel(AUDCODEC_TypeDef *codec, uint32_t sel)
{
    MODIFY_REG(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_FCHOP_SEL,
               MAKE_REG_VAL(sel, AUDCODEC_ADC2_CFG1_FCHOP_SEL_Msk, AUDCODEC_ADC2_CFG1_FCHOP_SEL_Pos));
}

/**
 * @brief Enable ADC2 VCM quick settling (ADC2_CFG1.VCMST).
 * @param[in] codec Audio codec instance pointer.
 */
static inline void ll_audcodec_adc2_vcmst_enable(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_VCMST);
}

/**
 * @brief Disable ADC2 VCM quick settling (ADC2_CFG1.VCMST = 0).
 * @param[in] codec Audio codec instance pointer.
 */
static inline void ll_audcodec_adc2_vcmst_disable(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_VCMST);
}

/**
 * @brief Invert the ADC2 output clock (ADC2_CFG1.CLKOUT_INV).
 * @param[in] codec Audio codec instance pointer.
 */
static inline void ll_audcodec_adc2_clkout_inv_enable(AUDCODEC_TypeDef *codec)
{
    SET_BIT(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_CLKOUT_INV);
}

/**
 * @brief Use the non-inverted ADC2 output clock (ADC2_CFG1.CLKOUT_INV = 0).
 * @param[in] codec Audio codec instance pointer.
 */
static inline void ll_audcodec_adc2_clkout_inv_disable(AUDCODEC_TypeDef *codec)
{
    CLEAR_BIT(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_CLKOUT_INV);
}

/**
 * @brief Set the ADC2 peripheral circuits bias mode (ADC2_CFG1.PERI_BM).
 * @param[in] codec Audio codec instance pointer.
 * @param[in] bm    Bias mode (2 bits).
 */
static inline void ll_audcodec_set_adc2_peri_bm(AUDCODEC_TypeDef *codec, uint32_t bm)
{
    MODIFY_REG(codec->ADC2_CFG1, AUDCODEC_ADC2_CFG1_PERI_BM,
               MAKE_REG_VAL(bm, AUDCODEC_ADC2_CFG1_PERI_BM_Msk, AUDCODEC_ADC2_CFG1_PERI_BM_Pos));
}

/**
 * @brief Enable/disable the ADC2 (ADC2_CFG2.EN).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_adc2_set_enable(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC2_CFG2, AUDCODEC_ADC2_CFG2_EN,
               en ? AUDCODEC_ADC2_CFG2_EN : 0UL);
}

static inline void ll_audcodec_adc2_set_config(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_ADC2_CFG2_CLEAR | AUDCODEC_ADC2_CFG2_CHOP_EN |
                    AUDCODEC_ADC2_CFG2_RSTB | AUDCODEC_ADC2_CFG2_EN;
    MODIFY_REG(codec->ADC2_CFG2, mask, conf & mask);
}

/**
 * @brief Assert/release the ADC2 reset (ADC2_CFG2.RSTB, active-low).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to assert (pin low), zero to release.
 */
static inline void ll_audcodec_adc2_set_rstb(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->ADC2_CFG2, AUDCODEC_ADC2_CFG2_RSTB,
               en ? 0UL : AUDCODEC_ADC2_CFG2_RSTB);
}

static inline void ll_audcodec_dac1_set_config(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_DAC1_CFG_EN_OS_DAC | AUDCODEC_DAC1_CFG_OS_DAC |
                    AUDCODEC_DAC1_CFG_GAIN | AUDCODEC_DAC1_CFG_SR |
                    AUDCODEC_DAC1_CFG_POL_CLK | AUDCODEC_DAC1_CFG_LP_MODE |
                    AUDCODEC_DAC1_CFG_SEL_VCM | AUDCODEC_DAC1_CFG_BM |
                    AUDCODEC_DAC1_CFG_EN_CHOP | AUDCODEC_DAC1_CFG_EN_AMP |
                    AUDCODEC_DAC1_CFG_EN_VCM | AUDCODEC_DAC1_CFG_EN_DAC |
                    AUDCODEC_DAC1_CFG_SEL_VSTART;
    MODIFY_REG(codec->DAC1_CFG, mask, conf & mask);
}

/**
 * @brief Enable/disable the DAC1 (DAC1_CFG.EN_DAC).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac1_set_en_dac(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC1_CFG, AUDCODEC_DAC1_CFG_EN_DAC,
               en ? AUDCODEC_DAC1_CFG_EN_DAC : 0UL);
}

/**
 * @brief Enable/disable the DAC1 VCM (DAC1_CFG.EN_VCM).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac1_set_en_vcm(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC1_CFG, AUDCODEC_DAC1_CFG_EN_VCM,
               en ? AUDCODEC_DAC1_CFG_EN_VCM : 0UL);
}

/**
 * @brief Enable/disable the DAC1 amplifier (DAC1_CFG.EN_AMP).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac1_set_en_amp(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC1_CFG, AUDCODEC_DAC1_CFG_EN_AMP,
               en ? AUDCODEC_DAC1_CFG_EN_AMP : 0UL);
}

/*==============================================================================
 * DAC Analog Path (DAC1_CFG / DAC2_CFG)
 *============================================================================*/

/**
 * @brief Enable/disable the DAC1 low-power mode (DAC1_CFG.LP_MODE).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac1_set_lp_mode(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC1_CFG, AUDCODEC_DAC1_CFG_LP_MODE,
               en ? AUDCODEC_DAC1_CFG_LP_MODE : 0UL);
}

/**
 * @brief Enable/disable the DAC1 soft-reset (DAC1_CFG.SR).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to assert, zero to release.
 */
static inline void ll_audcodec_dac1_set_sr(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC1_CFG, AUDCODEC_DAC1_CFG_SR,
               en ? AUDCODEC_DAC1_CFG_SR : 0UL);
}

/**
 * @brief Enable/disable the DAC1 oversampling clock (DAC1_CFG.EN_OS_DAC).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac1_set_en_os_dac(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC1_CFG, AUDCODEC_DAC1_CFG_EN_OS_DAC,
               en ? AUDCODEC_DAC1_CFG_EN_OS_DAC : 0UL);
}

static inline void ll_audcodec_dac2_set_config(AUDCODEC_TypeDef *codec, uint32_t conf)
{
    uint32_t mask = AUDCODEC_DAC2_CFG_EN_OS_DAC | AUDCODEC_DAC2_CFG_OS_DAC |
                    AUDCODEC_DAC2_CFG_GAIN | AUDCODEC_DAC2_CFG_SR |
                    AUDCODEC_DAC2_CFG_POL_CLK | AUDCODEC_DAC2_CFG_LP_MODE |
                    AUDCODEC_DAC2_CFG_SEL_VCM | AUDCODEC_DAC2_CFG_BM |
                    AUDCODEC_DAC2_CFG_EN_CHOP | AUDCODEC_DAC2_CFG_EN_AMP |
                    AUDCODEC_DAC2_CFG_EN_VCM | AUDCODEC_DAC2_CFG_EN_DAC |
                    AUDCODEC_DAC2_CFG_SEL_VSTART;
    MODIFY_REG(codec->DAC2_CFG, mask, conf & mask);
}

/**
 * @brief Enable/disable the DAC2 (DAC2_CFG.EN_DAC).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac2_set_en_dac(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC2_CFG, AUDCODEC_DAC2_CFG_EN_DAC,
               en ? AUDCODEC_DAC2_CFG_EN_DAC : 0UL);
}

/**
 * @brief Enable/disable the DAC2 VCM (DAC2_CFG.EN_VCM).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac2_set_en_vcm(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC2_CFG, AUDCODEC_DAC2_CFG_EN_VCM,
               en ? AUDCODEC_DAC2_CFG_EN_VCM : 0UL);
}

/**
 * @brief Enable/disable the DAC2 amplifier (DAC2_CFG.EN_AMP).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac2_set_en_amp(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC2_CFG, AUDCODEC_DAC2_CFG_EN_AMP,
               en ? AUDCODEC_DAC2_CFG_EN_AMP : 0UL);
}

/**
 * @brief Enable/disable the DAC2 soft-reset (DAC2_CFG.SR).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to assert, zero to release.
 */
static inline void ll_audcodec_dac2_set_sr(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC2_CFG, AUDCODEC_DAC2_CFG_SR,
               en ? AUDCODEC_DAC2_CFG_SR : 0UL);
}

/**
 * @brief Enable/disable the DAC2 oversampling clock (DAC2_CFG.EN_OS_DAC).
 * @param[in] codec AUDCODEC instance pointer.
 * @param[in] en    Non-zero to enable, zero to disable.
 */
static inline void ll_audcodec_dac2_set_en_os_dac(AUDCODEC_TypeDef *codec, uint32_t en)
{
    MODIFY_REG(codec->DAC2_CFG, AUDCODEC_DAC2_CFG_EN_OS_DAC,
               en ? AUDCODEC_DAC2_CFG_EN_OS_DAC : 0UL);
}

static inline void ll_audcodec_set_reserved_in0(AUDCODEC_TypeDef *codec, uint32_t value)
{
    uint32_t mask = AUDCODEC_RESERVED_IN0_CTRL0 | AUDCODEC_RESERVED_IN0_CTRL1 |
                    AUDCODEC_RESERVED_IN0_CTRL2 | AUDCODEC_RESERVED_IN0_CTRL3;
    MODIFY_REG(codec->RESERVED_IN0, mask, value & mask);
}

static inline void ll_audcodec_set_reserved_in1(AUDCODEC_TypeDef *codec, uint32_t value)
{
    uint32_t mask = AUDCODEC_RESERVED_IN1_CTRL4 | AUDCODEC_RESERVED_IN1_CTRL5;
    MODIFY_REG(codec->RESERVED_IN1, mask, value & mask);
}

static inline uint32_t ll_audcodec_get_reserved_out(AUDCODEC_TypeDef *codec)
{
    return READ_REG(codec->RESERVED_OUT) &
           (AUDCODEC_RESERVED_OUT_STAT0 | AUDCODEC_RESERVED_OUT_STAT1);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_AUDCODEC_H */
