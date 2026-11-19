/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_PDM_H
#define __LL_PDM_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Swap the right and left channels (CFG0.SWAP_EN).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_swap_channels(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->CFG0, PDM_CFG0_SWAP_EN);
}

/**
 * @brief Disable channel swap (CFG0.SWAP_EN = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_disable_swap(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->CFG0, PDM_CFG0_SWAP_EN);
}

/**
 * @brief Enable stereo (dual channel) mode (CFG0.STEREO_EN).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_enable_stereo(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->CFG0, PDM_CFG0_STEREO_EN);
}

/**
 * @brief Disable stereo mode (CFG0.STEREO_EN = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_disable_stereo(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->CFG0, PDM_CFG0_STEREO_EN);
}

/**
 * @brief Enable the right PDM channel (CFG0.RIGHT_EN).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_enable_right(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->CFG0, PDM_CFG0_RIGHT_EN);
}

/**
 * @brief Disable the right PDM channel (CFG0.RIGHT_EN = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_disable_right(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->CFG0, PDM_CFG0_RIGHT_EN);
}

/**
 * @brief Enable the left PDM channel (CFG0.LEFT_EN).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_enable_left(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->CFG0, PDM_CFG0_LEFT_EN);
}

/**
 * @brief Disable the left PDM channel (CFG0.LEFT_EN = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_disable_left(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->CFG0, PDM_CFG0_LEFT_EN);
}

/**
 * @brief Set the PDM clock frequency division ratio (CFG0.CLK_DIV).
 * @param[in] PDMx PDM instance pointer.
 * @param[in] div  Clock division ratio (4 bits).
 */
static inline void ll_pdm_set_clock_div(PDM_TypeDef *PDMx, uint32_t div)
{
    MODIFY_REG(PDMx->CFG0, PDM_CFG0_CLK_DIV,
               MAKE_REG_VAL(div, PDM_CFG0_CLK_DIV_Msk, PDM_CFG0_CLK_DIV_Pos));
}

/**
 * @brief Select the PDM reference clock source (CFG0.CLK_SEL).
 * @param[in] PDMx    PDM instance pointer.
 * @param[in] use_dll Non-zero selects DLL 3.072 MHz, zero selects XTAL 9.6 MHz.
 */
static inline void ll_pdm_select_clock(PDM_TypeDef *PDMx, uint32_t use_dll)
{
    MODIFY_REG(PDMx->CFG0, PDM_CFG0_CLK_SEL,
               use_dll ? PDM_CFG0_CLK_SEL : 0U);
}

/**
 * @file ll_pdm.h
 * @brief Header-only low-level PDM-to-PCM (PDM) APIs for SF32LB52x.
 *
 * The PDM module decimates one/two PDM microphone bitstreams into PCM data
 * through a processing chain: SINC filter, high-pass filter (HPF), PGA gain
 * stage and low-pass filter (LPF), then feeds a FIFO readable via AHB or DMA.
 */

/*==============================================================================
 * Core Control (CFG0)
 *============================================================================*/

/**
 * @brief Enable the PDM-to-PCM module core (CFG0.PDMCOREEN).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_enable(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->CFG0, PDM_CFG0_PDMCOREEN);
}

/**
 * @brief Disable the PDM-to-PCM module core (CFG0.PDMCOREEN = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_disable(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->CFG0, PDM_CFG0_PDMCOREEN);
}

/**
 * @brief Set the sample delay of the right data stream (CFG1.SAMPLE_DLY_R).
 * @param[in] PDMx PDM instance pointer.
 * @param[in] dly  Delay count (3 bits).
 */
static inline void ll_pdm_set_right_sample_delay(PDM_TypeDef *PDMx, uint32_t dly)
{
    MODIFY_REG(PDMx->CFG1, PDM_CFG1_SAMPLE_DLY_R,
               MAKE_REG_VAL(dly, PDM_CFG1_SAMPLE_DLY_R_Msk, PDM_CFG1_SAMPLE_DLY_R_Pos));
}

/**
 * @brief Set the sample delay of the left data stream (CFG1.SAMPLE_DLY_L).
 * @param[in] PDMx PDM instance pointer.
 * @param[in] dly  Delay count (3 bits).
 */
static inline void ll_pdm_set_left_sample_delay(PDM_TypeDef *PDMx, uint32_t dly)
{
    MODIFY_REG(PDMx->CFG1, PDM_CFG1_SAMPLE_DLY_L,
               MAKE_REG_VAL(dly, PDM_CFG1_SAMPLE_DLY_L_Msk, PDM_CFG1_SAMPLE_DLY_L_Pos));
}

/*==============================================================================
 * Sample Alignment (CFG1)
 *============================================================================*/

/**
 * @brief Set the number of delay cells before the data stream is sampled (CFG1.BUF_DLY).
 * @param[in] PDMx PDM instance pointer.
 * @param[in] dly  Delay cell count (5 bits).
 */
static inline void ll_pdm_set_buf_delay(PDM_TypeDef *PDMx, uint32_t dly)
{
    MODIFY_REG(PDMx->CFG1, PDM_CFG1_BUF_DLY,
               MAKE_REG_VAL(dly, PDM_CFG1_BUF_DLY_Msk, PDM_CFG1_BUF_DLY_Pos));
}

/**
 * @brief Select the SINC filter order (SINC_CFG.SINC_ORDER_SEL).
 * @param[in] PDMx  PDM instance pointer.
 * @param[in] order SINC order select (1 bit).
 */
static inline void ll_pdm_set_sinc_order(PDM_TypeDef *PDMx, uint32_t order)
{
    MODIFY_REG(PDMx->SINC_CFG, PDM_SINC_CFG_SINC_ORDER_SEL,
               order ? PDM_SINC_CFG_SINC_ORDER_SEL : 0U);
}

/*==============================================================================
 * SINC Filter (SINC_CFG)
 *============================================================================*/

/**
 * @brief Set the SINC decimation rate (SINC_CFG.SINC_RATE).
 * @param[in] PDMx  PDM instance pointer.
 * @param[in] rate  SINC rate (8 bits).
 */
static inline void ll_pdm_set_sinc_rate(PDM_TypeDef *PDMx, uint32_t rate)
{
    MODIFY_REG(PDMx->SINC_CFG, PDM_SINC_CFG_SINC_RATE,
               MAKE_REG_VAL(rate, PDM_SINC_CFG_SINC_RATE_Msk, PDM_SINC_CFG_SINC_RATE_Pos));
}

/**
 * @brief Reset the high-pass filter (HPF_CFG.HPF_RST).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_hpf_reset(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->HPF_CFG, PDM_HPF_CFG_HPF_RST);
}

/**
 * @brief Release the high-pass filter reset (HPF_CFG.HPF_RST = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_hpf_release_reset(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->HPF_CFG, PDM_HPF_CFG_HPF_RST);
}

/**
 * @brief Bypass the high-pass filter (HPF_CFG.HPF_BYPASS).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_hpf_bypass(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->HPF_CFG, PDM_HPF_CFG_HPF_BYPASS);
}

/**
 * @brief Enable the high-pass filter (HPF_CFG.HPF_BYPASS = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_hpf_enable(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->HPF_CFG, PDM_HPF_CFG_HPF_BYPASS);
}

/*==============================================================================
 * High-Pass Filter (HPF_CFG)
 *============================================================================*/

/**
 * @brief Set the HPF coefficient (HPF_CFG.HPF_COEFF).
 * @param[in] PDMx PDM instance pointer.
 * @param[in] coeff HPF coefficient (4 bits).
 */
static inline void ll_pdm_set_hpf_coeff(PDM_TypeDef *PDMx, uint32_t coeff)
{
    MODIFY_REG(PDMx->HPF_CFG, PDM_HPF_CFG_HPF_COEFF,
               MAKE_REG_VAL(coeff, PDM_HPF_CFG_HPF_COEFF_Msk, PDM_HPF_CFG_HPF_COEFF_Pos));
}

/**
 * @brief Set the right-channel PGA gain (PGA_CFG.PGA_GAIN_R).
 * @param[in] PDMx PDM instance pointer.
 * @param[in] gain Right-channel gain (7 bits).
 */
static inline void ll_pdm_set_pga_gain_r(PDM_TypeDef *PDMx, uint32_t gain)
{
    MODIFY_REG(PDMx->PGA_CFG, PDM_PGA_CFG_PGA_GAIN_R,
               MAKE_REG_VAL(gain, PDM_PGA_CFG_PGA_GAIN_R_Msk, PDM_PGA_CFG_PGA_GAIN_R_Pos));
}

/*==============================================================================
 * PGA Gain (PGA_CFG)
 *============================================================================*/

/**
 * @brief Set the left-channel PGA gain (PGA_CFG.PGA_GAIN_L).
 * @param[in] PDMx PDM instance pointer.
 * @param[in] gain Left-channel gain (7 bits).
 */
static inline void ll_pdm_set_pga_gain_l(PDM_TypeDef *PDMx, uint32_t gain)
{
    MODIFY_REG(PDMx->PGA_CFG, PDM_PGA_CFG_PGA_GAIN_L,
               MAKE_REG_VAL(gain, PDM_PGA_CFG_PGA_GAIN_L_Msk, PDM_PGA_CFG_PGA_GAIN_L_Pos));
}

/*==============================================================================
 * Low-Pass Filter (LPF_CFG6)
 *============================================================================*/

/**
 * @brief Bypass the low-pass filter (LPF_CFG6.LPF_BYPASS).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_lpf_bypass(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->LPF_CFG6, PDM_LPF_CFG6_LPF_BYPASS);
}

/**
 * @brief Enable the low-pass filter (LPF_CFG6.LPF_BYPASS = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_lpf_enable(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->LPF_CFG6, PDM_LPF_CFG6_LPF_BYPASS);
}

/**
 * @brief Enable down-sampling in the LPF stage (LPF_CFG6.LPF_DS).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_lpf_ds_enable(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->LPF_CFG6, PDM_LPF_CFG6_LPF_DS);
}

/**
 * @brief Disable down-sampling in the LPF stage (LPF_CFG6.LPF_DS = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_lpf_ds_disable(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->LPF_CFG6, PDM_LPF_CFG6_LPF_DS);
}

/**
 * @brief Exchange the left/right FIFO data order (FIFO_CFG.LR_CHG).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_fifo_lr_exchange(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->FIFO_CFG, PDM_FIFO_CFG_LR_CHG);
}

/**
 * @brief Disable the left/right FIFO order exchange (FIFO_CFG.LR_CHG = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_fifo_lr_exchange_disable(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->FIFO_CFG, PDM_FIFO_CFG_LR_CHG);
}

/**
 * @brief Mask the left-channel RX DMA request (FIFO_CFG.RX_DMA_MSK_L).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_mask_dma_l(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->FIFO_CFG, PDM_FIFO_CFG_RX_DMA_MSK_L);
}

/**
 * @brief Unmask the left-channel RX DMA request (FIFO_CFG.RX_DMA_MSK_L = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_unmask_dma_l(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->FIFO_CFG, PDM_FIFO_CFG_RX_DMA_MSK_L);
}

/**
 * @brief Mask the right-channel RX DMA request (FIFO_CFG.RX_DMA_MSK_R).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_mask_dma_r(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->FIFO_CFG, PDM_FIFO_CFG_RX_DMA_MSK_R);
}

/**
 * @brief Unmask the right-channel RX DMA request (FIFO_CFG.RX_DMA_MSK_R = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_unmask_dma_r(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->FIFO_CFG, PDM_FIFO_CFG_RX_DMA_MSK_R);
}

/**
 * @brief Set the PCM sample shift (FIFO_CFG.PDM_SHIFT).
 * @param[in] PDMx   PDM instance pointer.
 * @param[in] shift  Sample shift (3 bits).
 */
static inline void ll_pdm_set_shift(PDM_TypeDef *PDMx, uint32_t shift)
{
    MODIFY_REG(PDMx->FIFO_CFG, PDM_FIFO_CFG_PDM_SHIFT,
               MAKE_REG_VAL(shift, PDM_FIFO_CFG_PDM_SHIFT_Msk, PDM_FIFO_CFG_PDM_SHIFT_Pos));
}

/**
 * @brief Select the output sample width (FIFO_CFG.BYTE_TRUNC).
 * @param[in] PDMx  PDM instance pointer.
 * @param[in] trunc Byte truncation mode (2 bits):
 *                  0 = 24 bits, 1 = 16 bits, 2 = 8 bits, 3 = 32 bits.
 */
static inline void ll_pdm_set_byte_trunc(PDM_TypeDef *PDMx, uint32_t trunc)
{
    MODIFY_REG(PDMx->FIFO_CFG, PDM_FIFO_CFG_BYTE_TRUNC,
               MAKE_REG_VAL(trunc, PDM_FIFO_CFG_BYTE_TRUNC_Msk, PDM_FIFO_CFG_BYTE_TRUNC_Pos));
}

/*==============================================================================
 * FIFO Configuration & Data (FIFO_CFG / DATA_L / DATA_R)
 *============================================================================*/

/**
 * @brief Combine left and right channels into one FIFO word (FIFO_CFG.BYTE_CON).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_fifo_combine_enable(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->FIFO_CFG, PDM_FIFO_CFG_BYTE_CON);
}

/**
 * @brief Keep left and right channels separate (FIFO_CFG.BYTE_CON = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_fifo_combine_disable(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->FIFO_CFG, PDM_FIFO_CFG_BYTE_CON);
}

/**
 * @brief Read the left-channel FIFO data entry (DATA_L).
 * @param[in] PDMx PDM instance pointer.
 * @return Left-channel PCM data word.
 */
static inline uint32_t ll_pdm_read_data_l(PDM_TypeDef *PDMx)
{
    return READ_REG(PDMx->DATA_L);
}

/**
 * @brief Read the right-channel FIFO data entry (DATA_R).
 * @param[in] PDMx PDM instance pointer.
 * @return Right-channel PCM data word.
 */
static inline uint32_t ll_pdm_read_data_r(PDM_TypeDef *PDMx)
{
    return READ_REG(PDMx->DATA_R);
}

/*==============================================================================
 * FIFO Status (FIFO_ST)
 *============================================================================*/

/**
 * @brief Check whether the left FIFO is full (FIFO_ST.FULL_L).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the left FIFO is full.
 */
static inline uint32_t ll_pdm_is_left_fifo_full(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->FIFO_ST, PDM_FIFO_ST_FULL_L);
}

/**
 * @brief Check whether the left FIFO is empty (FIFO_ST.EMPTY_L).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the left FIFO is empty.
 */
static inline uint32_t ll_pdm_is_left_fifo_empty(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->FIFO_ST, PDM_FIFO_ST_EMPTY_L);
}

/**
 * @brief Check whether the left FIFO is almost full (FIFO_ST.ALMOST_FULL_L).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the left FIFO is almost full.
 */
static inline uint32_t ll_pdm_is_left_fifo_almost_full(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->FIFO_ST, PDM_FIFO_ST_ALMOST_FULL_L);
}

/**
 * @brief Check whether the left FIFO is almost empty (FIFO_ST.ALMOST_EMPTY_L).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the left FIFO is almost empty.
 */
static inline uint32_t ll_pdm_is_left_fifo_almost_empty(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->FIFO_ST, PDM_FIFO_ST_ALMOST_EMPTY_L);
}

/**
 * @brief Check whether the right FIFO is full (FIFO_ST.FULL_R).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the right FIFO is full.
 */
static inline uint32_t ll_pdm_is_right_fifo_full(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->FIFO_ST, PDM_FIFO_ST_FULL_R);
}

/**
 * @brief Check whether the right FIFO is empty (FIFO_ST.EMPTY_R).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the right FIFO is empty.
 */
static inline uint32_t ll_pdm_is_right_fifo_empty(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->FIFO_ST, PDM_FIFO_ST_EMPTY_R);
}

/**
 * @brief Check whether the right FIFO is almost full (FIFO_ST.ALMOST_FULL_R).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the right FIFO is almost full.
 */
static inline uint32_t ll_pdm_is_right_fifo_almost_full(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->FIFO_ST, PDM_FIFO_ST_ALMOST_FULL_R);
}

/**
 * @brief Check whether the right FIFO is almost empty (FIFO_ST.ALMOST_EMPTY_R).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the right FIFO is almost empty.
 */
static inline uint32_t ll_pdm_is_right_fifo_almost_empty(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->FIFO_ST, PDM_FIFO_ST_ALMOST_EMPTY_R);
}

/*==============================================================================
 * Interrupts (INT_ST / INT_MSK / INT_CLR)
 *============================================================================*/

/**
 * @brief Check the left-channel overflow flag (INT_ST.OVERFLOW_L).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the left channel has overflowed.
 */
static inline uint32_t ll_pdm_is_active_flag_overflow_l(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->INT_ST, PDM_INT_ST_OVERFLOW_L);
}

/**
 * @brief Check the right-channel overflow flag (INT_ST.OVERFLOW_R).
 * @param[in] PDMx PDM instance pointer.
 * @return Non-zero when the right channel has overflowed.
 */
static inline uint32_t ll_pdm_is_active_flag_overflow_r(PDM_TypeDef *PDMx)
{
    return READ_BIT(PDMx->INT_ST, PDM_INT_ST_OVERFLOW_R);
}

/**
 * @brief Mask the left-channel overflow interrupt (INT_MSK.INT_MASK_L).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_mask_it_overflow_l(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->INT_MSK, PDM_INT_MSK_INT_MASK_L);
}

/**
 * @brief Unmask the left-channel overflow interrupt (INT_MSK.INT_MASK_L = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_unmask_it_overflow_l(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->INT_MSK, PDM_INT_MSK_INT_MASK_L);
}

/**
 * @brief Mask the right-channel overflow interrupt (INT_MSK.INT_MASK_R).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_mask_it_overflow_r(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->INT_MSK, PDM_INT_MSK_INT_MASK_R);
}

/**
 * @brief Unmask the right-channel overflow interrupt (INT_MSK.INT_MASK_R = 0).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_unmask_it_overflow_r(PDM_TypeDef *PDMx)
{
    CLEAR_BIT(PDMx->INT_MSK, PDM_INT_MSK_INT_MASK_R);
}

/**
 * @brief Clear the left-channel overflow flag (INT_CLR.INT_CLR_L, write 1).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_clear_flag_overflow_l(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->INT_CLR, PDM_INT_CLR_INT_CLR_L);
}

/**
 * @brief Clear the right-channel overflow flag (INT_CLR.INT_CLR_R, write 1).
 * @param[in] PDMx PDM instance pointer.
 */
static inline void ll_pdm_clear_flag_overflow_r(PDM_TypeDef *PDMx)
{
    SET_BIT(PDMx->INT_CLR, PDM_INT_CLR_INT_CLR_R);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_PDM_H */
