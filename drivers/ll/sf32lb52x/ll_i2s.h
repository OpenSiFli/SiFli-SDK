/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_I2S_H
#define __LL_I2S_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_i2s.h
 * @brief Header-only low-level I2S/PCM APIs for SF32LB52x.
 */

/** @defgroup LL_I2S_TIMING LL I2S Serial Timing Mode */
/** @{ */
#define LL_I2S_TIMING_I2S       (0x0UL << I2S_AUDIO_SERIAL_TIMING_TIMING_Pos)
#define LL_I2S_TIMING_LEFT_J    (0x1UL << I2S_AUDIO_SERIAL_TIMING_TIMING_Pos)
#define LL_I2S_TIMING_RIGHT_J   (0x2UL << I2S_AUDIO_SERIAL_TIMING_TIMING_Pos)
/** @} */

/** @defgroup LL_I2S_ROLE LL I2S Master/Slave */
/** @{ */
#define LL_I2S_MASTER 0x00000000U
#define LL_I2S_SLAVE  I2S_AUDIO_SERIAL_TIMING_SLAVE_EN
/** @} */

/** @defgroup LL_I2S_LRCK_POL LL I2S LRCK Polarity */
/** @{ */
#define LL_I2S_LRCK_POL_NORMAL 0x00000000U
#define LL_I2S_LRCK_POL_INVERT I2S_AUDIO_SERIAL_TIMING_LRCK_POL
/** @} */

/** @defgroup LL_I2S_INTERFACE LL I2S Interface Select */
/** @{ */
#define LL_I2S_INTF_I2S  0x00000000U
#define LL_I2S_INTF_PCM  I2S_AUDIO_TX_FUNC_EN_TX_INTF_SEL
/** @} */

/** @defgroup LL_I2S_SL_MERGE LL I2S Slave Timing Merge */
/** @{ */
#define LL_I2S_SL_MERGE_INDEPENDENT 0x00000000U
#define LL_I2S_SL_MERGE_SHARED     I2S_AUDIO_I2S_SL_MERGE_SLAVE_TIMING_MERGE
/** @} */

/** @defgroup LL_I2S_CH_SEL LL I2S Channel Source Select */
/** @{ */
#define LL_I2S_RIGHT_CH_SEL_RIGHT   0x0UL
#define LL_I2S_RIGHT_CH_SEL_LEFT    0x1UL
#define LL_I2S_RIGHT_CH_SEL_AVERAGE 0x2UL
#define LL_I2S_LEFT_CH_SEL_LEFT     0x0UL
#define LL_I2S_LEFT_CH_SEL_RIGHT    0x1UL
#define LL_I2S_LEFT_CH_SEL_AVERAGE  0x2UL
/** @} */

/** @defgroup LL_I2S_RECORD_DATA_SEL LL I2S Record Data Source */
/** @{ */
#define LL_I2S_RECORD_SRC_I2S_AUDIO 0x00000000U
#define LL_I2S_RECORD_SRC_BT         I2S_RECORD_DATA_SEL_RS_DATA_SEL
/** @} */

/** @defgroup LL_I2S_BT_PATH LL I2S BT Path Select */
/** @{ */
#define LL_I2S_BT_PATH_NORMAL 0x00000000U
#define LL_I2S_BT_PATH_LOOP   I2S_BT_PHONE_CTRL_BT_PATH_SEL
/** @} */

/** @defgroup LL_I2S_BT_PCM_BPS LL I2S BT PCM BPS Select */
/** @{ */
#define LL_I2S_BT_PCM_BPS_8K  0x00000000U
#define LL_I2S_BT_PCM_BPS_16K I2S_BT_PHONE_CTRL_BT_PCM_IF_BPS
/** @} */

/** @defgroup LL_I2S_BB_BPS LL I2S Baseband I2S BPS to Codec */
/** @{ */
#define LL_I2S_BB_BPS_DISABLE 0x00000000U
#define LL_I2S_BB_BPS_ENABLE  I2S_BT_PHONE_CTRL_BB_I2S_BPS_TO_CDC
/** @} */

/** @defgroup LL_I2S_BB_PCM_TIM_SEL LL I2S BB PCM Timing Source (BB_PCM_FORMAT.pcm_tim_sel[6:5]) */
/** @{ */
#define LL_I2S_BB_PCM_TIM_I2S   (0x0UL << I2S_BB_PCM_FORMAT_PCM_TIM_SEL_Pos) /**< 00: I2S timing */
#define LL_I2S_BB_PCM_TIM_LEFT  (0x1UL << I2S_BB_PCM_FORMAT_PCM_TIM_SEL_Pos) /**< 01: Left justified */
#define LL_I2S_BB_PCM_TIM_RIGHT (0x2UL << I2S_BB_PCM_FORMAT_PCM_TIM_SEL_Pos) /**< 10: Right justified */
#define LL_I2S_BB_PCM_TIM_PCM   (0x3UL << I2S_BB_PCM_FORMAT_PCM_TIM_SEL_Pos) /**< 11: PCM timing */
/** @} */

/** @defgroup LL_I2S_BB_PCM_CLK_POL LL I2S BB PCM Clock Polarity (BB_PCM_FORMAT.pcm_clk_pol[10]) */
/** @{ */
#define LL_I2S_BB_PCM_CLK_POL_NORMAL 0x00000000U
#define LL_I2S_BB_PCM_CLK_POL_RISE   I2S_BB_PCM_FORMAT_PCM_CLK_POL
/** @} */

/** @defgroup LL_I2S_BB_PCM_LRCK_POL LL I2S BB PCM LRCK Polarity (BB_PCM_FORMAT.i2s_lrck_pol[9]) */
/** @{ */
#define LL_I2S_BB_PCM_LRCK_POL_NORMAL 0x00000000U
#define LL_I2S_BB_PCM_LRCK_POL_INVERT I2S_BB_PCM_FORMAT_I2S_LRCK_POL
/** @} */

/** @defgroup LL_I2S_BB_PCM_LSB_FLAG LL I2S BB PCM LSB Flag (BB_PCM_FORMAT.pcm_lsb_flag[8]) */
/** @{ */
#define LL_I2S_BB_PCM_LSB_MSB_FIRST 0x00000000U
#define LL_I2S_BB_PCM_LSB_LSB_FIRST I2S_BB_PCM_FORMAT_PCM_LSB_FLAG
/** @} */

/** @defgroup LL_I2S_BB_PCM_SYNC_FLAG LL I2S BB PCM Sync Flag (BB_PCM_FORMAT.pcm_sync_flag[7]) */
/** @{ */
#define LL_I2S_BB_PCM_SYNC_SHORT 0x00000000U
#define LL_I2S_BB_PCM_SYNC_LONG  I2S_BB_PCM_FORMAT_PCM_SYNC_FLAG
/** @} */

/** @defgroup LL_I2S_BT_CLK_POL LL I2S BT PCM Clock Polarity */
/** @{ */
#define LL_I2S_BT_CLK_POL_NORMAL 0x00000000U
#define LL_I2S_BT_CLK_POL_INVERT I2S_BT_PCM_TIMING_CLK_POL
/** @} */

/** @defgroup LL_I2S_BT_SYNC_FLAG LL I2S BT PCM Sync Flag */
/** @{ */
#define LL_I2S_BT_SYNC_NORMAL 0x00000000U
#define LL_I2S_BT_SYNC_INVERT I2S_BT_PCM_TIMING_SYNC_FLAG
/** @} */

/** @defgroup LL_I2S_BT_LSB_FLAG LL I2S BT PCM LSB Flag */
/** @{ */
#define LL_I2S_BT_LSB_MSB_FIRST 0x00000000U
#define LL_I2S_BT_LSB_LSB_FIRST I2S_BT_PCM_TIMING_LSB_FLAG
/** @} */

/** @defgroup LL_I2S_IRQ LL I2S IRQ Mask/Status Bits */
/** @{ */
#define LL_I2S_IRQ_RX_FIFO_OVERFLOW  I2S_INT_STATUS_RX_FIFO_OVERFLOW
#define LL_I2S_IRQ_TX_FIFO_UNDERFLOW I2S_INT_STATUS_TX_FIFO_UNDERFLOW
#define LL_I2S_IRQ_RX_FIFO_MASK      I2S_INT_MASK_RX_FIFO_INT_MASK
#define LL_I2S_IRQ_TX_FIFO_MASK      I2S_INT_MASK_TX_FIFO_INT_MASK
/** @} */

/** @defgroup LL_I2S_DMA_MASK LL I2S DMA Mask Bits */
/** @{ */
#define LL_I2S_DMA_MASK_RX I2S_DMA_MASK_RX_DMA_MASK
#define LL_I2S_DMA_MASK_TX I2S_DMA_MASK_TX_DMA_MASK
/** @} */

/** @defgroup LL_I2S_LOOP LL I2S Debug Loop Mode */
/** @{ */
#define LL_I2S_LOOP_DA2AD I2S_DEBUG_LOOP_DA2AD_LOOP_BACK
#define LL_I2S_LOOP_AD2DA I2S_DEBUG_LOOP_AD2DA_LOOP_BACK
/** @} */

/** @defgroup LL_I2S_BALANCE LL I2S Balance Enable */
/** @{ */
#define LL_I2S_BALANCE_DISABLE (0x0UL << I2S_TX_LR_BAL_CTRL_EN_Pos)
#define LL_I2S_BALANCE_L_ONLY  (0x2UL << I2S_TX_LR_BAL_CTRL_EN_Pos)
#define LL_I2S_BALANCE_R_ONLY  (0x1UL << I2S_TX_LR_BAL_CTRL_EN_Pos)
/** @} */

/*==============================================================================
 * TX Path
 *============================================================================*/

/**
 * @brief TX serial timing configuration.
 */
typedef struct
{
    uint32_t timing;    /**< Timing mode, use @ref LL_I2S_TIMING_I2S, LL_I2S_TIMING_LEFT_J or LL_I2S_TIMING_RIGHT_J (TIMING=3 is reserved). */
    uint32_t role;      /**< Master/slave, use @ref LL_I2S_MASTER or @ref LL_I2S_SLAVE. */
    uint32_t lrck_pol;  /**< LRCK polarity, use @ref LL_I2S_LRCK_POL_NORMAL or @ref LL_I2S_LRCK_POL_INVERT. */
} ll_i2s_tx_timing_config_t;

/**
 * @brief TX PCM format configuration.
 */
typedef struct
{
    uint32_t data_width; /**< PCM data width in bits (for example, 16 for 16-bit PCM). */
    uint32_t track_flag; /**< Track flag enable, use 0 or I2S_TX_PCM_FORMAT_TRACK_FLAG. */
} ll_i2s_tx_pcm_format_config_t;

/**
 * @brief Configure TX PCM format (TX_PCM_FORMAT).
 * @param[in] I2Sx I2S instance pointer.
 * @param[in] cfg  Pointer to TX PCM format configuration.
 */
static inline void ll_i2s_config_tx_pcm_format(I2S_TypeDef *I2Sx,
        const ll_i2s_tx_pcm_format_config_t *cfg)
{
    MODIFY_REG(I2Sx->TX_PCM_FORMAT,
               I2S_TX_PCM_FORMAT_DW | I2S_TX_PCM_FORMAT_TRACK_FLAG,
               ((cfg->data_width << I2S_TX_PCM_FORMAT_DW_Pos) &
                I2S_TX_PCM_FORMAT_DW) |
               cfg->track_flag);
}

/**
 * @brief Set TX sample clock duty (TX_PCM_SAMPLE_CLK).
 * @param[in] I2Sx     I2S instance pointer.
 * @param[in] fs_duty  Sample clock duty value.
 */
static inline void ll_i2s_set_tx_sample_clk(I2S_TypeDef *I2Sx, uint32_t fs_duty)
{
    MODIFY_REG(I2Sx->TX_PCM_SAMPLE_CLK, I2S_TX_PCM_SAMPLE_CLK_FS_DUTY,
               ((fs_duty << I2S_TX_PCM_SAMPLE_CLK_FS_DUTY_Pos) &
                I2S_TX_PCM_SAMPLE_CLK_FS_DUTY));
}

/**
 * @brief Enable/disable TX re-sample smooth (TX_RS_SMOOTH).
 * @param[in] I2Sx    I2S instance pointer.
 * @param[in] enable  Non-zero to enable.
 */
static inline void ll_i2s_set_tx_rs_smooth(I2S_TypeDef *I2Sx, uint32_t enable)
{
    if (enable)
    {
        SET_BIT(I2Sx->TX_RS_SMOOTH, I2S_TX_RS_SMOOTH_EN);
    }
    else
    {
        CLEAR_BIT(I2Sx->TX_RS_SMOOTH, I2S_TX_RS_SMOOTH_EN);
    }
}

/**
 * @brief Set TX channel source select (TX_PCM_CH_SEL).
 * @param[in] I2Sx         I2S instance pointer.
 * @param[in] right_sel    Right output source, use @ref LL_I2S_RIGHT_CH_SEL_RIGHT,
 *                         @ref LL_I2S_RIGHT_CH_SEL_LEFT, or @ref LL_I2S_RIGHT_CH_SEL_AVERAGE.
 * @param[in] left_sel     Left output source, use @ref LL_I2S_LEFT_CH_SEL_LEFT,
 *                         @ref LL_I2S_LEFT_CH_SEL_RIGHT, or @ref LL_I2S_LEFT_CH_SEL_AVERAGE.
 */
static inline void ll_i2s_set_tx_ch_sel(I2S_TypeDef *I2Sx,
                                        uint32_t right_sel,
                                        uint32_t left_sel)
{
    MODIFY_REG(I2Sx->TX_PCM_CH_SEL,
               I2S_TX_PCM_CH_SEL_RIGHT_CHANNEL_SEL |
               I2S_TX_PCM_CH_SEL_LEFT_CHANNEL_SEL,
               ((right_sel << I2S_TX_PCM_CH_SEL_RIGHT_CHANNEL_SEL_Pos) &
                I2S_TX_PCM_CH_SEL_RIGHT_CHANNEL_SEL) |
               ((left_sel << I2S_TX_PCM_CH_SEL_LEFT_CHANNEL_SEL_Pos) &
                I2S_TX_PCM_CH_SEL_LEFT_CHANNEL_SEL));
}

/**
 * @brief Set TX volume (TX_VOL_CTRL).
 * @param[in] I2Sx  I2S instance pointer.
 * @param[in] vol   Volume level (0..15).
 */
static inline void ll_i2s_set_tx_volume(I2S_TypeDef *I2Sx, uint32_t vol)
{
    MODIFY_REG(I2Sx->TX_VOL_CTRL, I2S_TX_VOL_CTRL_VOL,
               ((vol << I2S_TX_VOL_CTRL_VOL_Pos) & I2S_TX_VOL_CTRL_VOL));
}

/**
 * @brief Set TX left/right balance (TX_LR_BAL_CTRL).
 * @param[in] I2Sx     I2S instance pointer.
 * @param[in] bal_vol  Balance volume (0..15).
 * @param[in] enable   Balance mode, use @ref LL_I2S_BALANCE_DISABLE to @ref LL_I2S_BALANCE_R_ONLY.
 */
static inline void ll_i2s_set_tx_balance(I2S_TypeDef *I2Sx,
        uint32_t bal_vol,
        uint32_t enable)
{
    MODIFY_REG(I2Sx->TX_LR_BAL_CTRL,
               I2S_TX_LR_BAL_CTRL_BAL_VOL | I2S_TX_LR_BAL_CTRL_EN,
               ((bal_vol << I2S_TX_LR_BAL_CTRL_BAL_VOL_Pos) &
                I2S_TX_LR_BAL_CTRL_BAL_VOL) |
               enable);
}

/**
 * @brief Set TX LRCK divider (AUDIO_TX_LRCK_DIV).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] duty_low   LRCK low duty.
 * @param[in] duty_high  LRCK high duty.
 */
static inline void ll_i2s_set_tx_lrck_div(I2S_TypeDef *I2Sx,
        uint32_t duty_low,
        uint32_t duty_high)
{
    WRITE_REG(I2Sx->AUDIO_TX_LRCK_DIV,
              ((duty_low << I2S_AUDIO_TX_LRCK_DIV_DUTY_LOW_Pos) &
               I2S_AUDIO_TX_LRCK_DIV_DUTY_LOW) |
              ((duty_high << I2S_AUDIO_TX_LRCK_DIV_DUTY_HIGH_Pos) &
               I2S_AUDIO_TX_LRCK_DIV_DUTY_HIGH));
}

/**
 * @brief Set TX BCLK divider (AUDIO_TX_BCLK_DIV).
 * @param[in] I2Sx  I2S instance pointer.
 * @param[in] duty  BCLK duty.
 */
static inline void ll_i2s_set_tx_bclk_div(I2S_TypeDef *I2Sx, uint32_t duty)
{
    MODIFY_REG(I2Sx->AUDIO_TX_BCLK_DIV, I2S_AUDIO_TX_BCLK_DIV_DUTY,
               ((duty << I2S_AUDIO_TX_BCLK_DIV_DUTY_Pos) &
                I2S_AUDIO_TX_BCLK_DIV_DUTY));
}

/**
 * @brief Set TX audio format data width (AUDIO_TX_FORMAT).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] data_width PCM data width.
 */
static inline void ll_i2s_set_tx_audio_data_width(I2S_TypeDef *I2Sx,
        uint32_t data_width)
{
    MODIFY_REG(I2Sx->AUDIO_TX_FORMAT, I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH,
               ((data_width << I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH_Pos) &
                I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH));
}

/**
 * @brief Configure TX serial timing (AUDIO_SERIAL_TIMING).
 * @param[in] I2Sx I2S instance pointer.
 * @param[in] cfg  Pointer to TX timing configuration.
 */
static inline void ll_i2s_config_tx_timing(I2S_TypeDef *I2Sx,
        const ll_i2s_tx_timing_config_t *cfg)
{
    MODIFY_REG(I2Sx->AUDIO_SERIAL_TIMING,
               I2S_AUDIO_SERIAL_TIMING_TIMING |
               I2S_AUDIO_SERIAL_TIMING_SLAVE_EN |
               I2S_AUDIO_SERIAL_TIMING_LRCK_POL,
               cfg->timing | cfg->role | cfg->lrck_pol);
}

/**
 * @brief Disable TX function.
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_disable_tx(I2S_TypeDef *I2Sx)
{
    CLEAR_REG(I2Sx->AUDIO_TX_FUNC_EN);
}

/**
 * @brief Enable TX function (AUDIO_TX_FUNC_EN).
 * @param[in] I2Sx     I2S instance pointer.
 * @param[in] intf_sel Interface select, use @ref LL_I2S_INTF_I2S or @ref LL_I2S_INTF_PCM.
 */
static inline void ll_i2s_enable_tx(I2S_TypeDef *I2Sx, uint32_t intf_sel)
{
    WRITE_REG(I2Sx->AUDIO_TX_FUNC_EN,
              I2S_AUDIO_TX_FUNC_EN_TX_EN | intf_sel);
}

/**
 * @brief Pause TX (AUDIO_TX_PAUSE).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_pause_tx(I2S_TypeDef *I2Sx)
{
    SET_BIT(I2Sx->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);
}

/**
 * @brief Resume TX (AUDIO_TX_PAUSE).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_resume_tx(I2S_TypeDef *I2Sx)
{
    CLEAR_BIT(I2Sx->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);
}

/**
 * @brief Set slave timing merge (AUDIO_I2S_SL_MERGE).
 * @param[in] I2Sx   I2S instance pointer.
 * @param[in] merge  Use @ref LL_I2S_SL_MERGE_INDEPENDENT or @ref LL_I2S_SL_MERGE_SHARED.
 */
static inline void ll_i2s_set_sl_merge(I2S_TypeDef *I2Sx, uint32_t merge)
{
    MODIFY_REG(I2Sx->AUDIO_I2S_SL_MERGE,
               I2S_AUDIO_I2S_SL_MERGE_SLAVE_TIMING_MERGE, merge);
}

/*==============================================================================
 * RX Path
 *============================================================================*/

/**
 * @brief RX serial timing configuration.
 */
typedef struct
{
    uint32_t timing;    /**< Timing mode, use @ref LL_I2S_TIMING_I2S, LL_I2S_TIMING_LEFT_J or LL_I2S_TIMING_RIGHT_J (TIMING=3 is reserved). */
    uint32_t role;      /**< Master/slave, use @ref LL_I2S_MASTER or @ref LL_I2S_SLAVE. */
    uint32_t lrck_pol;  /**< LRCK polarity, use @ref LL_I2S_LRCK_POL_NORMAL or @ref LL_I2S_LRCK_POL_INVERT. */
} ll_i2s_rx_timing_config_t;

/**
 * @brief Disable RX function.
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_disable_rx(I2S_TypeDef *I2Sx)
{
    CLEAR_REG(I2Sx->AUDIO_RX_FUNC_EN);
}

/**
 * @brief Enable RX function (AUDIO_RX_FUNC_EN).
 * @param[in] I2Sx     I2S instance pointer.
 * @param[in] intf_sel Interface select, use @ref LL_I2S_INTF_I2S or @ref LL_I2S_INTF_PCM.
 */
static inline void ll_i2s_enable_rx(I2S_TypeDef *I2Sx, uint32_t intf_sel)
{
    WRITE_REG(I2Sx->AUDIO_RX_FUNC_EN,
              I2S_AUDIO_RX_FUNC_EN_RX_EN |
              (intf_sel & I2S_AUDIO_RX_FUNC_EN_RX_INTF_SEL));
}

/**
 * @brief Pause RX (AUDIO_RX_PAUSE).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_pause_rx(I2S_TypeDef *I2Sx)
{
    SET_BIT(I2Sx->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
}

/**
 * @brief Resume RX (AUDIO_RX_PAUSE).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_resume_rx(I2S_TypeDef *I2Sx)
{
    CLEAR_BIT(I2Sx->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
}

/**
 * @brief Configure RX serial timing (AUDIO_RX_SERIAL_TIMING).
 * @param[in] I2Sx I2S instance pointer.
 * @param[in] cfg  Pointer to RX timing configuration.
 */
static inline void ll_i2s_config_rx_timing(I2S_TypeDef *I2Sx,
        const ll_i2s_rx_timing_config_t *cfg)
{
    MODIFY_REG(I2Sx->AUDIO_RX_SERIAL_TIMING,
               I2S_AUDIO_RX_SERIAL_TIMING_TIMING |
               I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN |
               I2S_AUDIO_RX_SERIAL_TIMING_LRCK_POL,
               cfg->timing | cfg->role | cfg->lrck_pol);
}

/**
 * @brief Set RX PCM data width (AUDIO_RX_PCM_DW).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] data_width PCM data width (0..31).
 */
static inline void ll_i2s_set_rx_pcm_data_width(I2S_TypeDef *I2Sx,
        uint32_t data_width)
{
    MODIFY_REG(I2Sx->AUDIO_RX_PCM_DW, I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH,
               ((data_width << I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH_Pos) &
                I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH));
}

/**
 * @brief Set RX LRCK divider (AUDIO_RX_LRCK_DIV).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] duty_low   LRCK low duty.
 * @param[in] duty_high  LRCK high duty.
 */
static inline void ll_i2s_set_rx_lrck_div(I2S_TypeDef *I2Sx,
        uint32_t duty_low,
        uint32_t duty_high)
{
    WRITE_REG(I2Sx->AUDIO_RX_LRCK_DIV,
              ((duty_low << I2S_AUDIO_RX_LRCK_DIV_DUTY_LOW_Pos) &
               I2S_AUDIO_RX_LRCK_DIV_DUTY_LOW) |
              ((duty_high << I2S_AUDIO_RX_LRCK_DIV_DUTY_HIGH_Pos) &
               I2S_AUDIO_RX_LRCK_DIV_DUTY_HIGH));
}

/**
 * @brief Set RX BCLK divider (AUDIO_RX_BCLK_DIV).
 * @param[in] I2Sx  I2S instance pointer.
 * @param[in] duty  BCLK duty.
 */
static inline void ll_i2s_set_rx_bclk_div(I2S_TypeDef *I2Sx, uint32_t duty)
{
    MODIFY_REG(I2Sx->AUDIO_RX_BCLK_DIV, I2S_AUDIO_RX_BCLK_DIV_DUTY,
               ((duty << I2S_AUDIO_RX_BCLK_DIV_DUTY_Pos) &
                I2S_AUDIO_RX_BCLK_DIV_DUTY));
}

/**
 * @brief Select record data source (RECORD_DATA_SEL).
 * @param[in] I2Sx  I2S instance pointer.
 * @param[in] src   Data source, use @ref LL_I2S_RECORD_SRC_I2S_AUDIO or
 *                   @ref LL_I2S_RECORD_SRC_BT.
 */
static inline void ll_i2s_select_record_src(I2S_TypeDef *I2Sx, uint32_t src)
{
    MODIFY_REG(I2Sx->RECORD_DATA_SEL, I2S_RECORD_DATA_SEL_RS_DATA_SEL, src);
}

/**
 * @brief Set RX re-sample clock divider (RX_RE_SAMPLE_CLK_DIV).
 * @param[in] I2Sx     I2S instance pointer.
 * @param[in] rs_duty  Re-sample duty.
 */
static inline void ll_i2s_set_rx_rs_clk_div(I2S_TypeDef *I2Sx, uint32_t rs_duty)
{
    MODIFY_REG(I2Sx->RX_RE_SAMPLE_CLK_DIV, I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY,
               ((rs_duty << I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY_Pos) &
                I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY));
}

/**
 * @brief Enable/disable RX re-sample smooth (RX_RE_SAMPLE).
 * @param[in] I2Sx    I2S instance pointer.
 * @param[in] enable  Non-zero to enable.
 */
static inline void ll_i2s_set_rx_rs_smooth(I2S_TypeDef *I2Sx, uint32_t enable)
{
    if (enable)
    {
        SET_BIT(I2Sx->RX_RE_SAMPLE, I2S_RX_RE_SAMPLE_SMOOTH_EN);
    }
    else
    {
        CLEAR_BIT(I2Sx->RX_RE_SAMPLE, I2S_RX_RE_SAMPLE_SMOOTH_EN);
    }
}

/**
 * @brief Set record format (RECORD_FORMAT).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] data_width Data width (0=16bit, 1=32bit).
 * @param[in] track      Track flag.
 */
static inline void ll_i2s_set_record_format(I2S_TypeDef *I2Sx,
        uint32_t data_width,
        uint32_t track)
{
    MODIFY_REG(I2Sx->RECORD_FORMAT,
               I2S_RECORD_FORMAT_DW | I2S_RECORD_FORMAT_TRACK,
               ((data_width << I2S_RECORD_FORMAT_DW_Pos) &
                I2S_RECORD_FORMAT_DW) |
               ((track << I2S_RECORD_FORMAT_TRACK_Pos) &
                I2S_RECORD_FORMAT_TRACK));
}

/**
 * @brief Set RX channel source select (RX_CH_SEL).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] right_sel  Right output source, use @ref LL_I2S_RIGHT_CH_SEL_RIGHT,
 *                       @ref LL_I2S_RIGHT_CH_SEL_LEFT, or @ref LL_I2S_RIGHT_CH_SEL_AVERAGE.
 * @param[in] left_sel   Left output source, use @ref LL_I2S_LEFT_CH_SEL_LEFT,
 *                       @ref LL_I2S_LEFT_CH_SEL_RIGHT, or @ref LL_I2S_LEFT_CH_SEL_AVERAGE.
 */
static inline void ll_i2s_set_rx_ch_sel(I2S_TypeDef *I2Sx,
                                        uint32_t right_sel,
                                        uint32_t left_sel)
{
    MODIFY_REG(I2Sx->RX_CH_SEL,
               I2S_RX_CH_SEL_RIGHT_CHANNEL_SEL |
               I2S_RX_CH_SEL_LEFT_CHANNEL_SEL,
               ((right_sel << I2S_RX_CH_SEL_RIGHT_CHANNEL_SEL_Pos) &
                I2S_RX_CH_SEL_RIGHT_CHANNEL_SEL) |
               ((left_sel << I2S_RX_CH_SEL_LEFT_CHANNEL_SEL_Pos) &
                I2S_RX_CH_SEL_LEFT_CHANNEL_SEL));
}

/*==============================================================================
 * BT Phone Path
 *============================================================================*/

/**
 * @brief BT phone control configuration.
 */
typedef struct
{
    uint32_t back_mix;       /**< Back mix enable, use 0 or I2S_BT_PHONE_CTRL_BT_BACK_MIX_EN. */
    uint32_t mix_smooth;     /**< Mix smooth filter, use 0 or I2S_BT_PHONE_CTRL_BT_MIX_SMOOTH_FILTER_EN. */
    uint32_t path;           /**< Path select, use @ref LL_I2S_BT_PATH_NORMAL or @ref LL_I2S_BT_PATH_LOOP. */
    uint32_t pcm_bps;        /**< PCM BPS, use @ref LL_I2S_BT_PCM_BPS_8K or @ref LL_I2S_BT_PCM_BPS_16K. */
    uint32_t bb_bps_to_cdc;  /**< BB I2S BPS to codec, use @ref LL_I2S_BB_BPS_DISABLE or @ref LL_I2S_BB_BPS_ENABLE. */
} ll_i2s_bt_phone_config_t;

/**
 * @brief Disable BT phone path.
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_disable_bt_phone(I2S_TypeDef *I2Sx)
{
    CLEAR_REG(I2Sx->BT_PHONE_CTRL);
}

/**
 * @brief Enable BT phone path (BT_PHONE_CTRL).
 * @param[in] I2Sx I2S instance pointer.
 * @param[in] cfg  Pointer to BT phone configuration.
 */
static inline void ll_i2s_enable_bt_phone(I2S_TypeDef *I2Sx,
        const ll_i2s_bt_phone_config_t *cfg)
{
    WRITE_REG(I2Sx->BT_PHONE_CTRL,
              I2S_BT_PHONE_CTRL_BT_PH_EN | cfg->back_mix | cfg->mix_smooth |
              cfg->path | cfg->pcm_bps | cfg->bb_bps_to_cdc);
}

/**
 * @brief Configure BB PCM format (BB_PCM_FORMAT).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] data_width PCM data width (0..31).
 * @param[in] tim_sel    Timing source, use @ref LL_I2S_BB_PCM_TIM_I2S to @ref LL_I2S_BB_PCM_TIM_PCM.
 * @param[in] sync_flag  Sync flag, use @ref LL_I2S_BB_PCM_SYNC_SHORT or @ref LL_I2S_BB_PCM_SYNC_LONG.
 * @param[in] lsb_flag   LSB flag, use @ref LL_I2S_BB_PCM_LSB_MSB_FIRST or @ref LL_I2S_BB_PCM_LSB_LSB_FIRST.
 * @param[in] lrck_pol   LRCK polarity, use @ref LL_I2S_BB_PCM_LRCK_POL_NORMAL or @ref LL_I2S_BB_PCM_LRCK_POL_INVERT.
 * @param[in] clk_pol    Clock polarity, use @ref LL_I2S_BB_PCM_CLK_POL_NORMAL or @ref LL_I2S_BB_PCM_CLK_POL_RISE.
 */
static inline void ll_i2s_config_bb_pcm_format(I2S_TypeDef *I2Sx,
        uint32_t data_width,
        uint32_t tim_sel,
        uint32_t sync_flag,
        uint32_t lsb_flag,
        uint32_t lrck_pol,
        uint32_t clk_pol)
{
    WRITE_REG(I2Sx->BB_PCM_FORMAT,
              ((data_width << I2S_BB_PCM_FORMAT_PCM_DW_Pos) &
               I2S_BB_PCM_FORMAT_PCM_DW) |
              tim_sel | sync_flag | lsb_flag | lrck_pol | clk_pol);
}

/**
 * @brief Set BT PCM data width (BT_PCM_DW).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] data_width PCM data width (0..31).
 */
static inline void ll_i2s_set_bt_pcm_data_width(I2S_TypeDef *I2Sx,
        uint32_t data_width)
{
    MODIFY_REG(I2Sx->BT_PCM_DW, I2S_BT_PCM_DW_DW,
               ((data_width << I2S_BT_PCM_DW_DW_Pos) & I2S_BT_PCM_DW_DW));
}

/**
 * @brief Configure BT PCM timing (BT_PCM_TIMING).
 * @param[in] I2Sx      I2S instance pointer.
 * @param[in] lsb_flag  LSB flag, use @ref LL_I2S_BT_LSB_MSB_FIRST or @ref LL_I2S_BT_LSB_LSB_FIRST.
 * @param[in] sync_flag Sync flag, use @ref LL_I2S_BT_SYNC_NORMAL or @ref LL_I2S_BT_SYNC_INVERT.
 * @param[in] clk_pol   Clock polarity, use @ref LL_I2S_BT_CLK_POL_NORMAL or @ref LL_I2S_BT_CLK_POL_INVERT.
 */
static inline void ll_i2s_config_bt_pcm_timing(I2S_TypeDef *I2Sx,
        uint32_t lsb_flag,
        uint32_t sync_flag,
        uint32_t clk_pol)
{
    MODIFY_REG(I2Sx->BT_PCM_TIMING,
               I2S_BT_PCM_TIMING_LSB_FLAG | I2S_BT_PCM_TIMING_SYNC_FLAG |
               I2S_BT_PCM_TIMING_CLK_POL,
               lsb_flag | sync_flag | clk_pol);
}

/**
 * @brief Set BT PCM clock duty (BT_PCM_CLK_DUTY).
 * @param[in] I2Sx      I2S instance pointer.
 * @param[in] clk_duty  Clock duty value.
 */
static inline void ll_i2s_set_bt_pcm_clk_duty(I2S_TypeDef *I2Sx,
        uint32_t clk_duty)
{
    MODIFY_REG(I2Sx->BT_PCM_CLK_DUTY, I2S_BT_PCM_CLK_DUTY_CLK_DUTY,
               ((clk_duty << I2S_BT_PCM_CLK_DUTY_CLK_DUTY_Pos) &
                I2S_BT_PCM_CLK_DUTY_CLK_DUTY));
}

/**
 * @brief Set BT PCM sync duty (BT_PCM_SYNC_DUTY).
 * @param[in] I2Sx       I2S instance pointer.
 * @param[in] sync_duty  Sync duty value.
 */
static inline void ll_i2s_set_bt_pcm_sync_duty(I2S_TypeDef *I2Sx,
        uint32_t sync_duty)
{
    MODIFY_REG(I2Sx->BT_PCM_SYNC_DUTY, I2S_BT_PCM_SYNC_DUTY_SYNC_DUTY,
               ((sync_duty << I2S_BT_PCM_SYNC_DUTY_SYNC_DUTY_Pos) &
                I2S_BT_PCM_SYNC_DUTY_SYNC_DUTY));
}

/**
 * @brief Set BT volume (BT_VOL_CTRL).
 * @param[in] I2Sx     I2S instance pointer.
 * @param[in] vol      Volume level (0..7).
 * @param[in] adj_en   Auto adjust enable, 0 or I2S_BT_VOL_CTRL_VOL_ADJ_EN.
 */
static inline void ll_i2s_set_bt_volume(I2S_TypeDef *I2Sx,
                                        uint32_t vol,
                                        uint32_t adj_en)
{
    MODIFY_REG(I2Sx->BT_VOL_CTRL,
               I2S_BT_VOL_CTRL_VOL | I2S_BT_VOL_CTRL_VOL_ADJ_EN,
               ((vol << I2S_BT_VOL_CTRL_VOL_Pos) & I2S_BT_VOL_CTRL_VOL) |
               adj_en);
}

/**
 * @brief Enable TX FIFO interrupt (INT_MASK.tx_fifo_int_mask).
 * @note The mask bit is active-high: 1 = masked/disabled, 0 = enabled. Clearing
 *       it enables the interrupt (reference manual 11.2).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_enable_it_tx_fifo(I2S_TypeDef *I2Sx)
{
    CLEAR_BIT(I2Sx->INT_MASK, I2S_INT_MASK_TX_FIFO_INT_MASK);
}

/**
 * @brief Disable TX FIFO interrupt (INT_MASK.tx_fifo_int_mask).
 * @note The mask bit is active-high: 1 = masked/disabled.
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_disable_it_tx_fifo(I2S_TypeDef *I2Sx)
{
    SET_BIT(I2Sx->INT_MASK, I2S_INT_MASK_TX_FIFO_INT_MASK);
}

/*==============================================================================
 * Interrupts
 *============================================================================*/

/**
 * @brief Enable RX FIFO interrupt (INT_MASK.rx_fifo_int_mask).
 * @note The mask bit is active-high: 1 = masked/disabled, 0 = enabled. Clearing
 *       it enables the interrupt (reference manual 11.2).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_enable_it_rx_fifo(I2S_TypeDef *I2Sx)
{
    CLEAR_BIT(I2Sx->INT_MASK, I2S_INT_MASK_RX_FIFO_INT_MASK);
}

/**
 * @brief Disable RX FIFO interrupt (INT_MASK.rx_fifo_int_mask).
 * @note The mask bit is active-high: 1 = masked/disabled.
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_disable_it_rx_fifo(I2S_TypeDef *I2Sx)
{
    SET_BIT(I2Sx->INT_MASK, I2S_INT_MASK_RX_FIFO_INT_MASK);
}

/**
 * @brief Get interrupt status (INT_STATUS).
 * @param[in] I2Sx I2S instance pointer.
 * @return INT_STATUS register value.
 */
static inline uint32_t ll_i2s_get_irq_status(I2S_TypeDef *I2Sx)
{
    return READ_REG(I2Sx->INT_STATUS);
}

/**
 * @brief Check TX FIFO underflow flag.
 * @param[in] I2Sx I2S instance pointer.
 * @return Non-zero if TX FIFO underflow occurred.
 */
static inline uint32_t ll_i2s_is_active_flag_tx_underflow(I2S_TypeDef *I2Sx)
{
    return READ_BIT(I2Sx->INT_STATUS, I2S_INT_STATUS_TX_FIFO_UNDERFLOW);
}

/**
 * @brief Check RX FIFO overflow flag.
 * @param[in] I2Sx I2S instance pointer.
 * @return Non-zero if RX FIFO overflow occurred.
 */
static inline uint32_t ll_i2s_is_active_flag_rx_overflow(I2S_TypeDef *I2Sx)
{
    return READ_BIT(I2Sx->INT_STATUS, I2S_INT_STATUS_RX_FIFO_OVERFLOW);
}

/**
 * @brief Write a PCM data sample to the TX FIFO data port (TX_DMA_ENTRY).
 * @note TX_DMA_ENTRY is a write-only data port, NOT a DMA address register.
 *       To let DMA feed it, configure the DMA destination with the port
 *       address returned by @ref ll_i2s_get_tx_dma_port_addr.
 * @param[in] I2Sx  I2S instance pointer.
 * @param[in] data  PCM data sample to write.
 */
static inline void ll_i2s_write_tx_data(I2S_TypeDef *I2Sx, uint32_t data)
{
    WRITE_REG(I2Sx->TX_DMA_ENTRY, data);
}

/**
 * @brief Read a PCM data sample from the RX FIFO data port (RX_DMA_ENTRY).
 * @note RX_DMA_ENTRY is a read-only data port, NOT a DMA address register.
 *       Reading it consumes one sample. To let DMA drain it, configure the
 *       DMA source with @ref ll_i2s_get_rx_dma_port_addr.
 * @param[in] I2Sx I2S instance pointer.
 * @return The PCM data sample read from the RX FIFO.
 */
static inline uint32_t ll_i2s_read_rx_data(I2S_TypeDef *I2Sx)
{
    return READ_REG(I2Sx->RX_DMA_ENTRY);
}

/*==============================================================================
 * DMA
 *============================================================================*/

/**
 * @brief Get the TX FIFO data-port address for DMA destination configuration.
 * @param[in] I2Sx I2S instance pointer.
 * @return Address of TX_DMA_ENTRY (pass to DMA as the destination).
 */
static inline uint32_t ll_i2s_get_tx_dma_port_addr(I2S_TypeDef *I2Sx)
{
    return (uint32_t)&I2Sx->TX_DMA_ENTRY;
}

/**
 * @brief Get the RX FIFO data-port address for DMA source configuration.
 * @param[in] I2Sx I2S instance pointer.
 * @return Address of RX_DMA_ENTRY (pass to DMA as the source).
 */
static inline uint32_t ll_i2s_get_rx_dma_port_addr(I2S_TypeDef *I2Sx)
{
    return (uint32_t)&I2Sx->RX_DMA_ENTRY;
}

/**
 * @brief Enable TX DMA (DMA_MASK.tx_dma_mask).
 * @note The mask bit is active-high: 1 = masked (DMA disabled), 0 = unmasked.
 *       Clearing it enables TX DMA (reference manual 11.2).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_enable_dma_tx(I2S_TypeDef *I2Sx)
{
    CLEAR_BIT(I2Sx->DMA_MASK, I2S_DMA_MASK_TX_DMA_MASK);
}

/**
 * @brief Disable TX DMA (DMA_MASK.tx_dma_mask).
 * @note The mask bit is active-high: 1 = masked (DMA disabled).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_disable_dma_tx(I2S_TypeDef *I2Sx)
{
    SET_BIT(I2Sx->DMA_MASK, I2S_DMA_MASK_TX_DMA_MASK);
}

/**
 * @brief Enable RX DMA (DMA_MASK.rx_dma_mask).
 * @note The mask bit is active-high: 1 = masked (DMA disabled), 0 = unmasked.
 *       Clearing it enables RX DMA (reference manual 11.2).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_enable_dma_rx(I2S_TypeDef *I2Sx)
{
    CLEAR_BIT(I2Sx->DMA_MASK, I2S_DMA_MASK_RX_DMA_MASK);
}

/**
 * @brief Disable RX DMA (DMA_MASK.rx_dma_mask).
 * @note The mask bit is active-high: 1 = masked (DMA disabled).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_disable_dma_rx(I2S_TypeDef *I2Sx)
{
    SET_BIT(I2Sx->DMA_MASK, I2S_DMA_MASK_RX_DMA_MASK);
}

/**
 * @brief Write debug loop control (DEBUG_LOOP).
 * @param[in] I2Sx    I2S instance pointer.
 * @param[in] loops   Loop mode bits, use @ref LL_I2S_LOOP_DA2AD and/or @ref LL_I2S_LOOP_AD2DA.
 * @param[in] clk_sel SP clock select.
 * @param[in] clk_div SP clock divider.
 */
static inline void ll_i2s_set_debug_loop(I2S_TypeDef *I2Sx,
        uint32_t loops,
        uint32_t clk_sel,
        uint32_t clk_div)
{
    WRITE_REG(I2Sx->DEBUG_LOOP,
              loops | ((clk_sel << I2S_DEBUG_LOOP_SP_CLK_SEL_Pos) &
                       I2S_DEBUG_LOOP_SP_CLK_SEL) |
              I2S_DEBUG_LOOP_SP_CLK_DIV_UPDATE |
              ((clk_div << I2S_DEBUG_LOOP_SP_CLK_DIV_Pos) &
               I2S_DEBUG_LOOP_SP_CLK_DIV));
}

/*==============================================================================
 * FIFO / Debug / Equalizer
 *============================================================================*/

/**
 * @brief Get FIFO output status (FIFO_STATUS).
 * @param[in] I2Sx I2S instance pointer.
 * @return FIFO_STATUS register value.
 */
static inline uint32_t ll_i2s_get_fifo_status(I2S_TypeDef *I2Sx)
{
    return READ_REG(I2Sx->FIFO_STATUS);
}

/**
 * @brief Enable TX equalizer (TX_EQUALIZER_EN).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_enable_tx_equalizer(I2S_TypeDef *I2Sx)
{
    SET_BIT(I2Sx->TX_EQUALIZER_EN, I2S_TX_EQUALIZER_EN_TX_EQUALIZER_EN);
}

/**
 * @brief Disable TX equalizer (TX_EQUALIZER_EN).
 * @param[in] I2Sx I2S instance pointer.
 */
static inline void ll_i2s_disable_tx_equalizer(I2S_TypeDef *I2Sx)
{
    CLEAR_BIT(I2Sx->TX_EQUALIZER_EN, I2S_TX_EQUALIZER_EN_TX_EQUALIZER_EN);
}

/**
 * @brief Set TX equalizer gains for bands 1-6 (TX_EQUALIZER_GAIN1).
 * @param[in] I2Sx I2S instance pointer.
 * @param[in] g1  Band 1 gain.
 * @param[in] g2  Band 2 gain.
 * @param[in] g3  Band 3 gain.
 * @param[in] g4  Band 4 gain.
 * @param[in] g5  Band 5 gain.
 * @param[in] g6  Band 6 gain.
 */
static inline void ll_i2s_set_tx_eq_gain1(I2S_TypeDef *I2Sx,
        uint32_t g1, uint32_t g2,
        uint32_t g3, uint32_t g4,
        uint32_t g5, uint32_t g6)
{
    WRITE_REG(I2Sx->TX_EQUALIZER_GAIN1,
              ((g1 << I2S_TX_EQUALIZER_GAIN1_BAND1_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN1_BAND1_GAIN) |
              ((g2 << I2S_TX_EQUALIZER_GAIN1_BAND2_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN1_BAND2_GAIN) |
              ((g3 << I2S_TX_EQUALIZER_GAIN1_BAND3_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN1_BAND3_GAIN) |
              ((g4 << I2S_TX_EQUALIZER_GAIN1_BAND4_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN1_BAND4_GAIN) |
              ((g5 << I2S_TX_EQUALIZER_GAIN1_BAND5_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN1_BAND5_GAIN) |
              ((g6 << I2S_TX_EQUALIZER_GAIN1_BAND6_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN1_BAND6_GAIN));
}

/**
 * @brief Set TX equalizer gains for bands 7-10 (TX_EQUALIZER_GAIN2).
 * @param[in] I2Sx I2S instance pointer.
 * @param[in] g7  Band 7 gain.
 * @param[in] g8  Band 8 gain.
 * @param[in] g9  Band 9 gain.
 * @param[in] g10 Band 10 gain.
 */
static inline void ll_i2s_set_tx_eq_gain2(I2S_TypeDef *I2Sx,
        uint32_t g7, uint32_t g8,
        uint32_t g9, uint32_t g10)
{
    WRITE_REG(I2Sx->TX_EQUALIZER_GAIN2,
              ((g7 << I2S_TX_EQUALIZER_GAIN2_BAND7_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN2_BAND7_GAIN) |
              ((g8 << I2S_TX_EQUALIZER_GAIN2_BAND8_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN2_BAND8_GAIN) |
              ((g9 << I2S_TX_EQUALIZER_GAIN2_BAND9_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN2_BAND9_GAIN) |
              ((g10 << I2S_TX_EQUALIZER_GAIN2_BAND10_GAIN_Pos) &
               I2S_TX_EQUALIZER_GAIN2_BAND10_GAIN));
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_I2S_H */