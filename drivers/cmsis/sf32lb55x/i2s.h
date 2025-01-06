/**
  ******************************************************************************
  * @file   i2s.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __I2S_H
#define __I2S_H

typedef struct
{
    __IO uint32_t RSVD41[4];
    __IO uint32_t TX_PCM_FORMAT;
    __IO uint32_t RSVD40[3];
    __IO uint32_t TX_PCM_SAMPLE_CLK;
    __IO uint32_t RSVD39[3];
    __IO uint32_t TX_RS_SMOOTH;
    __IO uint32_t RSVD38[3];
    __IO uint32_t TX_PCM_CH_SEL;
    __IO uint32_t RSVD37[3];
    __IO uint32_t TX_VOL_CTRL;
    __IO uint32_t RSVD36[3];
    __IO uint32_t TX_LR_BAL_CTRL;
    __IO uint32_t RSVD35[3];
    __IO uint32_t AUDIO_TX_LRCK_DIV;
    __IO uint32_t RSVD34[3];
    __IO uint32_t AUDIO_TX_BCLK_DIV;
    __IO uint32_t RSVD33[3];
    __IO uint32_t AUDIO_TX_FORMAT;
    __IO uint32_t RSVD32[3];
    __IO uint32_t AUDIO_SERIAL_TIMING;
    __IO uint32_t RSVD31[3];
    __IO uint32_t AUDIO_TX_FUNC_EN;
    __IO uint32_t RSVD30[3];
    __IO uint32_t AUDIO_TX_PAUSE;
    __IO uint32_t RSVD29;
    __IO uint32_t AUDIO_I2S_SL_MERGE;
    __IO uint32_t RSVD28[13];
    __IO uint32_t AUDIO_RX_FUNC_EN;
    __IO uint32_t RSVD27[3];
    __IO uint32_t AUDIO_RX_PAUSE;
    __IO uint32_t RSVD26[3];
    __IO uint32_t AUDIO_RX_SERIAL_TIMING;
    __IO uint32_t RSVD25[3];
    __IO uint32_t AUDIO_RX_PCM_DW;
    __IO uint32_t RSVD24[3];
    __IO uint32_t AUDIO_RX_LRCK_DIV;
    __IO uint32_t RSVD23[3];
    __IO uint32_t AUDIO_RX_BCLK_DIV;
    __IO uint32_t RSVD22[3];
    __IO uint32_t RECORD_DATA_SEL;
    __IO uint32_t RSVD21[3];
    __IO uint32_t RX_RE_SAMPLE_CLK_DIV;
    __IO uint32_t RSVD20[3];
    __IO uint32_t RX_RE_SAMPLE;
    __IO uint32_t RSVD19[3];
    __IO uint32_t RECORD_FORMAT;
    __IO uint32_t RSVD18[3];
    __IO uint32_t RX_CH_SEL;
    __IO uint32_t RSVD17[23];
    __IO uint32_t BT_PHONE_CTRL;
    __IO uint32_t RSVD16[3];
    __IO uint32_t BB_PCM_FORMAT;
    __IO uint32_t RSVD15[3];
    __IO uint32_t BT_PCM_DW;
    __IO uint32_t RSVD14[3];
    __IO uint32_t BT_PCM_TIMING;
    __IO uint32_t RSVD13[3];
    __IO uint32_t BT_PCM_CLK_DUTY;
    __IO uint32_t RSVD12[3];
    __IO uint32_t BT_PCM_SYNC_DUTY;
    __IO uint32_t RSVD11[3];
    __IO uint32_t BT_VOL_CTRL;
    __IO uint32_t RSVD10[39];
    __IO uint32_t INT_MASK;
    __IO uint32_t RSVD9[3];
    __IO uint32_t INT_STATUS;
    __IO uint32_t RSVD8[59];
    __IO uint32_t TX_DMA_ENTRY;
    __IO uint32_t RSVD7[15];
    __IO uint32_t RX_DMA_ENTRY;
    __IO uint32_t RSVD6[15];
    __IO uint32_t DMA_MASK;
    __IO uint32_t RSVD5[31];
    __IO uint32_t DEBUG_LOOP;
    __IO uint32_t RSVD4[63];
    __IO uint32_t FIFO_STATUS;
    __IO uint32_t RSVD3[63];
    __IO uint32_t TX_EQUALIZER_EN;
    __IO uint32_t RSVD2[3];
    __IO uint32_t TX_EQUALIZER_GAIN1;
    __IO uint32_t RSVD1[3];
    __IO uint32_t TX_EQUALIZER_GAIN2;
} I2S_TypeDef;


/*************** Bit definition for I2S_TX_PCM_FORMAT register ****************/
#define I2S_TX_PCM_FORMAT_DW_Pos        (0U)
#define I2S_TX_PCM_FORMAT_DW_Msk        (0x1FUL << I2S_TX_PCM_FORMAT_DW_Pos)
#define I2S_TX_PCM_FORMAT_DW            I2S_TX_PCM_FORMAT_DW_Msk
#define I2S_TX_PCM_FORMAT_TRACK_FLAG_Pos  (5U)
#define I2S_TX_PCM_FORMAT_TRACK_FLAG_Msk  (0x1UL << I2S_TX_PCM_FORMAT_TRACK_FLAG_Pos)
#define I2S_TX_PCM_FORMAT_TRACK_FLAG    I2S_TX_PCM_FORMAT_TRACK_FLAG_Msk

/************* Bit definition for I2S_TX_PCM_SAMPLE_CLK register **************/
#define I2S_TX_PCM_SAMPLE_CLK_FS_DUTY_Pos  (0U)
#define I2S_TX_PCM_SAMPLE_CLK_FS_DUTY_Msk  (0x1FFFUL << I2S_TX_PCM_SAMPLE_CLK_FS_DUTY_Pos)
#define I2S_TX_PCM_SAMPLE_CLK_FS_DUTY   I2S_TX_PCM_SAMPLE_CLK_FS_DUTY_Msk

/**************** Bit definition for I2S_TX_RS_SMOOTH register ****************/
#define I2S_TX_RS_SMOOTH_EN_Pos         (0U)
#define I2S_TX_RS_SMOOTH_EN_Msk         (0x1UL << I2S_TX_RS_SMOOTH_EN_Pos)
#define I2S_TX_RS_SMOOTH_EN             I2S_TX_RS_SMOOTH_EN_Msk

/*************** Bit definition for I2S_TX_PCM_CH_SEL register ****************/
#define I2S_TX_PCM_CH_SEL_RIGHT_CHANNEL_SEL_Pos  (0U)
#define I2S_TX_PCM_CH_SEL_RIGHT_CHANNEL_SEL_Msk  (0x3UL << I2S_TX_PCM_CH_SEL_RIGHT_CHANNEL_SEL_Pos)
#define I2S_TX_PCM_CH_SEL_RIGHT_CHANNEL_SEL  I2S_TX_PCM_CH_SEL_RIGHT_CHANNEL_SEL_Msk
#define I2S_TX_PCM_CH_SEL_LEFT_CHANNEL_SEL_Pos  (2U)
#define I2S_TX_PCM_CH_SEL_LEFT_CHANNEL_SEL_Msk  (0x3UL << I2S_TX_PCM_CH_SEL_LEFT_CHANNEL_SEL_Pos)
#define I2S_TX_PCM_CH_SEL_LEFT_CHANNEL_SEL  I2S_TX_PCM_CH_SEL_LEFT_CHANNEL_SEL_Msk

/**************** Bit definition for I2S_TX_VOL_CTRL register *****************/
#define I2S_TX_VOL_CTRL_VOL_Pos         (0U)
#define I2S_TX_VOL_CTRL_VOL_Msk         (0xFUL << I2S_TX_VOL_CTRL_VOL_Pos)
#define I2S_TX_VOL_CTRL_VOL             I2S_TX_VOL_CTRL_VOL_Msk

/*************** Bit definition for I2S_TX_LR_BAL_CTRL register ***************/
#define I2S_TX_LR_BAL_CTRL_BAL_VOL_Pos  (0U)
#define I2S_TX_LR_BAL_CTRL_BAL_VOL_Msk  (0xFUL << I2S_TX_LR_BAL_CTRL_BAL_VOL_Pos)
#define I2S_TX_LR_BAL_CTRL_BAL_VOL      I2S_TX_LR_BAL_CTRL_BAL_VOL_Msk
#define I2S_TX_LR_BAL_CTRL_EN_Pos       (4U)
#define I2S_TX_LR_BAL_CTRL_EN_Msk       (0x3UL << I2S_TX_LR_BAL_CTRL_EN_Pos)
#define I2S_TX_LR_BAL_CTRL_EN           I2S_TX_LR_BAL_CTRL_EN_Msk

/************* Bit definition for I2S_AUDIO_TX_LRCK_DIV register **************/
#define I2S_AUDIO_TX_LRCK_DIV_DUTY_LOW_Pos  (0U)
#define I2S_AUDIO_TX_LRCK_DIV_DUTY_LOW_Msk  (0xFFFUL << I2S_AUDIO_TX_LRCK_DIV_DUTY_LOW_Pos)
#define I2S_AUDIO_TX_LRCK_DIV_DUTY_LOW  I2S_AUDIO_TX_LRCK_DIV_DUTY_LOW_Msk
#define I2S_AUDIO_TX_LRCK_DIV_DUTY_HIGH_Pos  (16U)
#define I2S_AUDIO_TX_LRCK_DIV_DUTY_HIGH_Msk  (0xFFFUL << I2S_AUDIO_TX_LRCK_DIV_DUTY_HIGH_Pos)
#define I2S_AUDIO_TX_LRCK_DIV_DUTY_HIGH  I2S_AUDIO_TX_LRCK_DIV_DUTY_HIGH_Msk

/************* Bit definition for I2S_AUDIO_TX_BCLK_DIV register **************/
#define I2S_AUDIO_TX_BCLK_DIV_DUTY_Pos  (0U)
#define I2S_AUDIO_TX_BCLK_DIV_DUTY_Msk  (0x3FUL << I2S_AUDIO_TX_BCLK_DIV_DUTY_Pos)
#define I2S_AUDIO_TX_BCLK_DIV_DUTY      I2S_AUDIO_TX_BCLK_DIV_DUTY_Msk

/************** Bit definition for I2S_AUDIO_TX_FORMAT register ***************/
#define I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH_Pos  (0U)
#define I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH_Msk  (0x1FUL << I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH_Pos)
#define I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH  I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH_Msk

/************ Bit definition for I2S_AUDIO_SERIAL_TIMING register *************/
#define I2S_AUDIO_SERIAL_TIMING_TIMING_Pos  (0U)
#define I2S_AUDIO_SERIAL_TIMING_TIMING_Msk  (0x3UL << I2S_AUDIO_SERIAL_TIMING_TIMING_Pos)
#define I2S_AUDIO_SERIAL_TIMING_TIMING  I2S_AUDIO_SERIAL_TIMING_TIMING_Msk
#define I2S_AUDIO_SERIAL_TIMING_SLAVE_EN_Pos  (2U)
#define I2S_AUDIO_SERIAL_TIMING_SLAVE_EN_Msk  (0x1UL << I2S_AUDIO_SERIAL_TIMING_SLAVE_EN_Pos)
#define I2S_AUDIO_SERIAL_TIMING_SLAVE_EN  I2S_AUDIO_SERIAL_TIMING_SLAVE_EN_Msk
#define I2S_AUDIO_SERIAL_TIMING_LRCK_POL_Pos  (3U)
#define I2S_AUDIO_SERIAL_TIMING_LRCK_POL_Msk  (0x1UL << I2S_AUDIO_SERIAL_TIMING_LRCK_POL_Pos)
#define I2S_AUDIO_SERIAL_TIMING_LRCK_POL  I2S_AUDIO_SERIAL_TIMING_LRCK_POL_Msk

/************** Bit definition for I2S_AUDIO_TX_FUNC_EN register **************/
#define I2S_AUDIO_TX_FUNC_EN_TX_EN_Pos  (0U)
#define I2S_AUDIO_TX_FUNC_EN_TX_EN_Msk  (0x1UL << I2S_AUDIO_TX_FUNC_EN_TX_EN_Pos)
#define I2S_AUDIO_TX_FUNC_EN_TX_EN      I2S_AUDIO_TX_FUNC_EN_TX_EN_Msk

/*************** Bit definition for I2S_AUDIO_TX_PAUSE register ***************/
#define I2S_AUDIO_TX_PAUSE_TX_PAUSE_Pos  (0U)
#define I2S_AUDIO_TX_PAUSE_TX_PAUSE_Msk  (0x1UL << I2S_AUDIO_TX_PAUSE_TX_PAUSE_Pos)
#define I2S_AUDIO_TX_PAUSE_TX_PAUSE     I2S_AUDIO_TX_PAUSE_TX_PAUSE_Msk

/************* Bit definition for I2S_AUDIO_I2S_SL_MERGE register *************/
#define I2S_AUDIO_I2S_SL_MERGE_SLAVE_TIMING_MERGE_Pos  (0U)
#define I2S_AUDIO_I2S_SL_MERGE_SLAVE_TIMING_MERGE_Msk  (0x1UL << I2S_AUDIO_I2S_SL_MERGE_SLAVE_TIMING_MERGE_Pos)
#define I2S_AUDIO_I2S_SL_MERGE_SLAVE_TIMING_MERGE  I2S_AUDIO_I2S_SL_MERGE_SLAVE_TIMING_MERGE_Msk

/************** Bit definition for I2S_AUDIO_RX_FUNC_EN register **************/
#define I2S_AUDIO_RX_FUNC_EN_RX_EN_Pos  (0U)
#define I2S_AUDIO_RX_FUNC_EN_RX_EN_Msk  (0x1UL << I2S_AUDIO_RX_FUNC_EN_RX_EN_Pos)
#define I2S_AUDIO_RX_FUNC_EN_RX_EN      I2S_AUDIO_RX_FUNC_EN_RX_EN_Msk

/*************** Bit definition for I2S_AUDIO_RX_PAUSE register ***************/
#define I2S_AUDIO_RX_PAUSE_RX_PAUSE_Pos  (0U)
#define I2S_AUDIO_RX_PAUSE_RX_PAUSE_Msk  (0x1UL << I2S_AUDIO_RX_PAUSE_RX_PAUSE_Pos)
#define I2S_AUDIO_RX_PAUSE_RX_PAUSE     I2S_AUDIO_RX_PAUSE_RX_PAUSE_Msk

/*********** Bit definition for I2S_AUDIO_RX_SERIAL_TIMING register ***********/
#define I2S_AUDIO_RX_SERIAL_TIMING_TIMING_Pos  (0U)
#define I2S_AUDIO_RX_SERIAL_TIMING_TIMING_Msk  (0x3UL << I2S_AUDIO_RX_SERIAL_TIMING_TIMING_Pos)
#define I2S_AUDIO_RX_SERIAL_TIMING_TIMING  I2S_AUDIO_RX_SERIAL_TIMING_TIMING_Msk
#define I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN_Pos  (2U)
#define I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN_Msk  (0x1UL << I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN_Pos)
#define I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN  I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN_Msk
#define I2S_AUDIO_RX_SERIAL_TIMING_LRCK_POL_Pos  (3U)
#define I2S_AUDIO_RX_SERIAL_TIMING_LRCK_POL_Msk  (0x1UL << I2S_AUDIO_RX_SERIAL_TIMING_LRCK_POL_Pos)
#define I2S_AUDIO_RX_SERIAL_TIMING_LRCK_POL  I2S_AUDIO_RX_SERIAL_TIMING_LRCK_POL_Msk

/************** Bit definition for I2S_AUDIO_RX_PCM_DW register ***************/
#define I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH_Pos  (0U)
#define I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH_Msk  (0x1FUL << I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH_Pos)
#define I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH  I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH_Msk

/************* Bit definition for I2S_AUDIO_RX_LRCK_DIV register **************/
#define I2S_AUDIO_RX_LRCK_DIV_DUTY_LOW_Pos  (0U)
#define I2S_AUDIO_RX_LRCK_DIV_DUTY_LOW_Msk  (0xFFFUL << I2S_AUDIO_RX_LRCK_DIV_DUTY_LOW_Pos)
#define I2S_AUDIO_RX_LRCK_DIV_DUTY_LOW  I2S_AUDIO_RX_LRCK_DIV_DUTY_LOW_Msk
#define I2S_AUDIO_RX_LRCK_DIV_DUTY_HIGH_Pos  (16U)
#define I2S_AUDIO_RX_LRCK_DIV_DUTY_HIGH_Msk  (0xFFFUL << I2S_AUDIO_RX_LRCK_DIV_DUTY_HIGH_Pos)
#define I2S_AUDIO_RX_LRCK_DIV_DUTY_HIGH  I2S_AUDIO_RX_LRCK_DIV_DUTY_HIGH_Msk

/************* Bit definition for I2S_AUDIO_RX_BCLK_DIV register **************/
#define I2S_AUDIO_RX_BCLK_DIV_DUTY_Pos  (0U)
#define I2S_AUDIO_RX_BCLK_DIV_DUTY_Msk  (0x3FFUL << I2S_AUDIO_RX_BCLK_DIV_DUTY_Pos)
#define I2S_AUDIO_RX_BCLK_DIV_DUTY      I2S_AUDIO_RX_BCLK_DIV_DUTY_Msk

/************** Bit definition for I2S_RECORD_DATA_SEL register ***************/
#define I2S_RECORD_DATA_SEL_RS_DATA_SEL_Pos  (0U)
#define I2S_RECORD_DATA_SEL_RS_DATA_SEL_Msk  (0x1UL << I2S_RECORD_DATA_SEL_RS_DATA_SEL_Pos)
#define I2S_RECORD_DATA_SEL_RS_DATA_SEL  I2S_RECORD_DATA_SEL_RS_DATA_SEL_Msk

/************ Bit definition for I2S_RX_RE_SAMPLE_CLK_DIV register ************/
#define I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY_Pos  (0U)
#define I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY_Msk  (0x1FFFUL << I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY_Pos)
#define I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY  I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY_Msk

/**************** Bit definition for I2S_RX_RE_SAMPLE register ****************/
#define I2S_RX_RE_SAMPLE_SMOOTH_EN_Pos  (0U)
#define I2S_RX_RE_SAMPLE_SMOOTH_EN_Msk  (0x1UL << I2S_RX_RE_SAMPLE_SMOOTH_EN_Pos)
#define I2S_RX_RE_SAMPLE_SMOOTH_EN      I2S_RX_RE_SAMPLE_SMOOTH_EN_Msk

/*************** Bit definition for I2S_RECORD_FORMAT register ****************/
#define I2S_RECORD_FORMAT_DW_Pos        (0U)
#define I2S_RECORD_FORMAT_DW_Msk        (0x1UL << I2S_RECORD_FORMAT_DW_Pos)
#define I2S_RECORD_FORMAT_DW            I2S_RECORD_FORMAT_DW_Msk
#define I2S_RECORD_FORMAT_TRACK_Pos     (1U)
#define I2S_RECORD_FORMAT_TRACK_Msk     (0x1UL << I2S_RECORD_FORMAT_TRACK_Pos)
#define I2S_RECORD_FORMAT_TRACK         I2S_RECORD_FORMAT_TRACK_Msk

/***************** Bit definition for I2S_RX_CH_SEL register ******************/
#define I2S_RX_CH_SEL_RIGHT_CHANNEL_SEL_Pos  (0U)
#define I2S_RX_CH_SEL_RIGHT_CHANNEL_SEL_Msk  (0x3UL << I2S_RX_CH_SEL_RIGHT_CHANNEL_SEL_Pos)
#define I2S_RX_CH_SEL_RIGHT_CHANNEL_SEL  I2S_RX_CH_SEL_RIGHT_CHANNEL_SEL_Msk
#define I2S_RX_CH_SEL_LEFT_CHANNEL_SEL_Pos  (2U)
#define I2S_RX_CH_SEL_LEFT_CHANNEL_SEL_Msk  (0x3UL << I2S_RX_CH_SEL_LEFT_CHANNEL_SEL_Pos)
#define I2S_RX_CH_SEL_LEFT_CHANNEL_SEL  I2S_RX_CH_SEL_LEFT_CHANNEL_SEL_Msk

/*************** Bit definition for I2S_BT_PHONE_CTRL register ****************/
#define I2S_BT_PHONE_CTRL_BT_PH_EN_Pos  (0U)
#define I2S_BT_PHONE_CTRL_BT_PH_EN_Msk  (0x1UL << I2S_BT_PHONE_CTRL_BT_PH_EN_Pos)
#define I2S_BT_PHONE_CTRL_BT_PH_EN      I2S_BT_PHONE_CTRL_BT_PH_EN_Msk
#define I2S_BT_PHONE_CTRL_BT_BACK_MIX_EN_Pos  (1U)
#define I2S_BT_PHONE_CTRL_BT_BACK_MIX_EN_Msk  (0x1UL << I2S_BT_PHONE_CTRL_BT_BACK_MIX_EN_Pos)
#define I2S_BT_PHONE_CTRL_BT_BACK_MIX_EN  I2S_BT_PHONE_CTRL_BT_BACK_MIX_EN_Msk
#define I2S_BT_PHONE_CTRL_BT_MIX_SMOOTH_FILTER_EN_Pos  (2U)
#define I2S_BT_PHONE_CTRL_BT_MIX_SMOOTH_FILTER_EN_Msk  (0x1UL << I2S_BT_PHONE_CTRL_BT_MIX_SMOOTH_FILTER_EN_Pos)
#define I2S_BT_PHONE_CTRL_BT_MIX_SMOOTH_FILTER_EN  I2S_BT_PHONE_CTRL_BT_MIX_SMOOTH_FILTER_EN_Msk
#define I2S_BT_PHONE_CTRL_BT_PATH_SEL_Pos  (3U)
#define I2S_BT_PHONE_CTRL_BT_PATH_SEL_Msk  (0x1UL << I2S_BT_PHONE_CTRL_BT_PATH_SEL_Pos)
#define I2S_BT_PHONE_CTRL_BT_PATH_SEL   I2S_BT_PHONE_CTRL_BT_PATH_SEL_Msk
#define I2S_BT_PHONE_CTRL_BT_PCM_IF_BPS_Pos  (4U)
#define I2S_BT_PHONE_CTRL_BT_PCM_IF_BPS_Msk  (0x1UL << I2S_BT_PHONE_CTRL_BT_PCM_IF_BPS_Pos)
#define I2S_BT_PHONE_CTRL_BT_PCM_IF_BPS  I2S_BT_PHONE_CTRL_BT_PCM_IF_BPS_Msk
#define I2S_BT_PHONE_CTRL_BB_I2S_BPS_TO_CDC_Pos  (5U)
#define I2S_BT_PHONE_CTRL_BB_I2S_BPS_TO_CDC_Msk  (0x1UL << I2S_BT_PHONE_CTRL_BB_I2S_BPS_TO_CDC_Pos)
#define I2S_BT_PHONE_CTRL_BB_I2S_BPS_TO_CDC  I2S_BT_PHONE_CTRL_BB_I2S_BPS_TO_CDC_Msk

/*************** Bit definition for I2S_BB_PCM_FORMAT register ****************/
#define I2S_BB_PCM_FORMAT_PCM_DW_Pos    (0U)
#define I2S_BB_PCM_FORMAT_PCM_DW_Msk    (0x1FUL << I2S_BB_PCM_FORMAT_PCM_DW_Pos)
#define I2S_BB_PCM_FORMAT_PCM_DW        I2S_BB_PCM_FORMAT_PCM_DW_Msk
#define I2S_BB_PCM_FORMAT_PCM_TIM_SEL_Pos  (5U)
#define I2S_BB_PCM_FORMAT_PCM_TIM_SEL_Msk  (0x3UL << I2S_BB_PCM_FORMAT_PCM_TIM_SEL_Pos)
#define I2S_BB_PCM_FORMAT_PCM_TIM_SEL   I2S_BB_PCM_FORMAT_PCM_TIM_SEL_Msk
#define I2S_BB_PCM_FORMAT_PCM_SYNC_FLAG_Pos  (7U)
#define I2S_BB_PCM_FORMAT_PCM_SYNC_FLAG_Msk  (0x1UL << I2S_BB_PCM_FORMAT_PCM_SYNC_FLAG_Pos)
#define I2S_BB_PCM_FORMAT_PCM_SYNC_FLAG  I2S_BB_PCM_FORMAT_PCM_SYNC_FLAG_Msk
#define I2S_BB_PCM_FORMAT_PCM_LSB_FLAG_Pos  (8U)
#define I2S_BB_PCM_FORMAT_PCM_LSB_FLAG_Msk  (0x1UL << I2S_BB_PCM_FORMAT_PCM_LSB_FLAG_Pos)
#define I2S_BB_PCM_FORMAT_PCM_LSB_FLAG  I2S_BB_PCM_FORMAT_PCM_LSB_FLAG_Msk
#define I2S_BB_PCM_FORMAT_I2S_LRCK_POL_Pos  (9U)
#define I2S_BB_PCM_FORMAT_I2S_LRCK_POL_Msk  (0x1UL << I2S_BB_PCM_FORMAT_I2S_LRCK_POL_Pos)
#define I2S_BB_PCM_FORMAT_I2S_LRCK_POL  I2S_BB_PCM_FORMAT_I2S_LRCK_POL_Msk
#define I2S_BB_PCM_FORMAT_PCM_CLK_POL_Pos  (10U)
#define I2S_BB_PCM_FORMAT_PCM_CLK_POL_Msk  (0x1UL << I2S_BB_PCM_FORMAT_PCM_CLK_POL_Pos)
#define I2S_BB_PCM_FORMAT_PCM_CLK_POL   I2S_BB_PCM_FORMAT_PCM_CLK_POL_Msk

/***************** Bit definition for I2S_BT_PCM_DW register ******************/
#define I2S_BT_PCM_DW_DW_Pos            (0U)
#define I2S_BT_PCM_DW_DW_Msk            (0x1FUL << I2S_BT_PCM_DW_DW_Pos)
#define I2S_BT_PCM_DW_DW                I2S_BT_PCM_DW_DW_Msk

/*************** Bit definition for I2S_BT_PCM_TIMING register ****************/
#define I2S_BT_PCM_TIMING_LSB_FLAG_Pos  (0U)
#define I2S_BT_PCM_TIMING_LSB_FLAG_Msk  (0x1UL << I2S_BT_PCM_TIMING_LSB_FLAG_Pos)
#define I2S_BT_PCM_TIMING_LSB_FLAG      I2S_BT_PCM_TIMING_LSB_FLAG_Msk
#define I2S_BT_PCM_TIMING_SYNC_FLAG_Pos  (1U)
#define I2S_BT_PCM_TIMING_SYNC_FLAG_Msk  (0x1UL << I2S_BT_PCM_TIMING_SYNC_FLAG_Pos)
#define I2S_BT_PCM_TIMING_SYNC_FLAG     I2S_BT_PCM_TIMING_SYNC_FLAG_Msk
#define I2S_BT_PCM_TIMING_CLK_POL_Pos   (2U)
#define I2S_BT_PCM_TIMING_CLK_POL_Msk   (0x1UL << I2S_BT_PCM_TIMING_CLK_POL_Pos)
#define I2S_BT_PCM_TIMING_CLK_POL       I2S_BT_PCM_TIMING_CLK_POL_Msk

/************** Bit definition for I2S_BT_PCM_CLK_DUTY register ***************/
#define I2S_BT_PCM_CLK_DUTY_CLK_DUTY_Pos  (0U)
#define I2S_BT_PCM_CLK_DUTY_CLK_DUTY_Msk  (0x3FFUL << I2S_BT_PCM_CLK_DUTY_CLK_DUTY_Pos)
#define I2S_BT_PCM_CLK_DUTY_CLK_DUTY    I2S_BT_PCM_CLK_DUTY_CLK_DUTY_Msk

/************** Bit definition for I2S_BT_PCM_SYNC_DUTY register **************/
#define I2S_BT_PCM_SYNC_DUTY_SYNC_DUTY_Pos  (0U)
#define I2S_BT_PCM_SYNC_DUTY_SYNC_DUTY_Msk  (0x3FUL << I2S_BT_PCM_SYNC_DUTY_SYNC_DUTY_Pos)
#define I2S_BT_PCM_SYNC_DUTY_SYNC_DUTY  I2S_BT_PCM_SYNC_DUTY_SYNC_DUTY_Msk

/**************** Bit definition for I2S_BT_VOL_CTRL register *****************/
#define I2S_BT_VOL_CTRL_VOL_Pos         (0U)
#define I2S_BT_VOL_CTRL_VOL_Msk         (0x7UL << I2S_BT_VOL_CTRL_VOL_Pos)
#define I2S_BT_VOL_CTRL_VOL             I2S_BT_VOL_CTRL_VOL_Msk
#define I2S_BT_VOL_CTRL_VOL_ADJ_EN_Pos  (3U)
#define I2S_BT_VOL_CTRL_VOL_ADJ_EN_Msk  (0x1UL << I2S_BT_VOL_CTRL_VOL_ADJ_EN_Pos)
#define I2S_BT_VOL_CTRL_VOL_ADJ_EN      I2S_BT_VOL_CTRL_VOL_ADJ_EN_Msk

/****************** Bit definition for I2S_INT_MASK register ******************/
#define I2S_INT_MASK_RX_FIFO_INT_MASK_Pos  (0U)
#define I2S_INT_MASK_RX_FIFO_INT_MASK_Msk  (0x1UL << I2S_INT_MASK_RX_FIFO_INT_MASK_Pos)
#define I2S_INT_MASK_RX_FIFO_INT_MASK   I2S_INT_MASK_RX_FIFO_INT_MASK_Msk
#define I2S_INT_MASK_TX_FIFO_INT_MASK_Pos  (1U)
#define I2S_INT_MASK_TX_FIFO_INT_MASK_Msk  (0x1UL << I2S_INT_MASK_TX_FIFO_INT_MASK_Pos)
#define I2S_INT_MASK_TX_FIFO_INT_MASK   I2S_INT_MASK_TX_FIFO_INT_MASK_Msk

/***************** Bit definition for I2S_INT_STATUS register *****************/
#define I2S_INT_STATUS_RX_FIFO_OVERFLOW_Pos  (0U)
#define I2S_INT_STATUS_RX_FIFO_OVERFLOW_Msk  (0x1UL << I2S_INT_STATUS_RX_FIFO_OVERFLOW_Pos)
#define I2S_INT_STATUS_RX_FIFO_OVERFLOW  I2S_INT_STATUS_RX_FIFO_OVERFLOW_Msk
#define I2S_INT_STATUS_TX_FIFO_UNDERFLOW_Pos  (1U)
#define I2S_INT_STATUS_TX_FIFO_UNDERFLOW_Msk  (0x1UL << I2S_INT_STATUS_TX_FIFO_UNDERFLOW_Pos)
#define I2S_INT_STATUS_TX_FIFO_UNDERFLOW  I2S_INT_STATUS_TX_FIFO_UNDERFLOW_Msk

/**************** Bit definition for I2S_TX_DMA_ENTRY register ****************/
#define I2S_TX_DMA_ENTRY_TX_DMA_ENTRY_Pos  (0U)
#define I2S_TX_DMA_ENTRY_TX_DMA_ENTRY_Msk  (0xFFFFFFFFUL << I2S_TX_DMA_ENTRY_TX_DMA_ENTRY_Pos)
#define I2S_TX_DMA_ENTRY_TX_DMA_ENTRY   I2S_TX_DMA_ENTRY_TX_DMA_ENTRY_Msk

/**************** Bit definition for I2S_RX_DMA_ENTRY register ****************/
#define I2S_RX_DMA_ENTRY_RX_DMA_ENTRY_Pos  (0U)
#define I2S_RX_DMA_ENTRY_RX_DMA_ENTRY_Msk  (0xFFFFFFFFUL << I2S_RX_DMA_ENTRY_RX_DMA_ENTRY_Pos)
#define I2S_RX_DMA_ENTRY_RX_DMA_ENTRY   I2S_RX_DMA_ENTRY_RX_DMA_ENTRY_Msk

/****************** Bit definition for I2S_DMA_MASK register ******************/
#define I2S_DMA_MASK_RX_DMA_MASK_Pos    (0U)
#define I2S_DMA_MASK_RX_DMA_MASK_Msk    (0x1UL << I2S_DMA_MASK_RX_DMA_MASK_Pos)
#define I2S_DMA_MASK_RX_DMA_MASK        I2S_DMA_MASK_RX_DMA_MASK_Msk
#define I2S_DMA_MASK_TX_DMA_MASK_Pos    (1U)
#define I2S_DMA_MASK_TX_DMA_MASK_Msk    (0x1UL << I2S_DMA_MASK_TX_DMA_MASK_Pos)
#define I2S_DMA_MASK_TX_DMA_MASK        I2S_DMA_MASK_TX_DMA_MASK_Msk

/***************** Bit definition for I2S_DEBUG_LOOP register *****************/
#define I2S_DEBUG_LOOP_DA2AD_LOOP_BACK_Pos  (0U)
#define I2S_DEBUG_LOOP_DA2AD_LOOP_BACK_Msk  (0x1UL << I2S_DEBUG_LOOP_DA2AD_LOOP_BACK_Pos)
#define I2S_DEBUG_LOOP_DA2AD_LOOP_BACK  I2S_DEBUG_LOOP_DA2AD_LOOP_BACK_Msk
#define I2S_DEBUG_LOOP_AD2DA_LOOP_BACK_Pos  (1U)
#define I2S_DEBUG_LOOP_AD2DA_LOOP_BACK_Msk  (0x1UL << I2S_DEBUG_LOOP_AD2DA_LOOP_BACK_Pos)
#define I2S_DEBUG_LOOP_AD2DA_LOOP_BACK  I2S_DEBUG_LOOP_AD2DA_LOOP_BACK_Msk

/**************** Bit definition for I2S_FIFO_STATUS register *****************/
#define I2S_FIFO_STATUS_FIFO_STATUS_OUT_Pos  (0U)
#define I2S_FIFO_STATUS_FIFO_STATUS_OUT_Msk  (0xFFUL << I2S_FIFO_STATUS_FIFO_STATUS_OUT_Pos)
#define I2S_FIFO_STATUS_FIFO_STATUS_OUT  I2S_FIFO_STATUS_FIFO_STATUS_OUT_Msk

/************** Bit definition for I2S_TX_EQUALIZER_EN register ***************/
#define I2S_TX_EQUALIZER_EN_TX_EQUALIZER_EN_Pos  (0U)
#define I2S_TX_EQUALIZER_EN_TX_EQUALIZER_EN_Msk  (0x1UL << I2S_TX_EQUALIZER_EN_TX_EQUALIZER_EN_Pos)
#define I2S_TX_EQUALIZER_EN_TX_EQUALIZER_EN  I2S_TX_EQUALIZER_EN_TX_EQUALIZER_EN_Msk

/************* Bit definition for I2S_TX_EQUALIZER_GAIN1 register *************/
#define I2S_TX_EQUALIZER_GAIN1_BAND1_GAIN_Pos  (0U)
#define I2S_TX_EQUALIZER_GAIN1_BAND1_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN1_BAND1_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN1_BAND1_GAIN  I2S_TX_EQUALIZER_GAIN1_BAND1_GAIN_Msk
#define I2S_TX_EQUALIZER_GAIN1_BAND2_GAIN_Pos  (5U)
#define I2S_TX_EQUALIZER_GAIN1_BAND2_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN1_BAND2_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN1_BAND2_GAIN  I2S_TX_EQUALIZER_GAIN1_BAND2_GAIN_Msk
#define I2S_TX_EQUALIZER_GAIN1_BAND3_GAIN_Pos  (10U)
#define I2S_TX_EQUALIZER_GAIN1_BAND3_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN1_BAND3_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN1_BAND3_GAIN  I2S_TX_EQUALIZER_GAIN1_BAND3_GAIN_Msk
#define I2S_TX_EQUALIZER_GAIN1_BAND4_GAIN_Pos  (15U)
#define I2S_TX_EQUALIZER_GAIN1_BAND4_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN1_BAND4_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN1_BAND4_GAIN  I2S_TX_EQUALIZER_GAIN1_BAND4_GAIN_Msk
#define I2S_TX_EQUALIZER_GAIN1_BAND5_GAIN_Pos  (20U)
#define I2S_TX_EQUALIZER_GAIN1_BAND5_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN1_BAND5_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN1_BAND5_GAIN  I2S_TX_EQUALIZER_GAIN1_BAND5_GAIN_Msk
#define I2S_TX_EQUALIZER_GAIN1_BAND6_GAIN_Pos  (25U)
#define I2S_TX_EQUALIZER_GAIN1_BAND6_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN1_BAND6_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN1_BAND6_GAIN  I2S_TX_EQUALIZER_GAIN1_BAND6_GAIN_Msk

/************* Bit definition for I2S_TX_EQUALIZER_GAIN2 register *************/
#define I2S_TX_EQUALIZER_GAIN2_BAND7_GAIN_Pos  (0U)
#define I2S_TX_EQUALIZER_GAIN2_BAND7_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN2_BAND7_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN2_BAND7_GAIN  I2S_TX_EQUALIZER_GAIN2_BAND7_GAIN_Msk
#define I2S_TX_EQUALIZER_GAIN2_BAND8_GAIN_Pos  (5U)
#define I2S_TX_EQUALIZER_GAIN2_BAND8_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN2_BAND8_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN2_BAND8_GAIN  I2S_TX_EQUALIZER_GAIN2_BAND8_GAIN_Msk
#define I2S_TX_EQUALIZER_GAIN2_BAND9_GAIN_Pos  (10U)
#define I2S_TX_EQUALIZER_GAIN2_BAND9_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN2_BAND9_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN2_BAND9_GAIN  I2S_TX_EQUALIZER_GAIN2_BAND9_GAIN_Msk
#define I2S_TX_EQUALIZER_GAIN2_BAND10_GAIN_Pos  (15U)
#define I2S_TX_EQUALIZER_GAIN2_BAND10_GAIN_Msk  (0x1FUL << I2S_TX_EQUALIZER_GAIN2_BAND10_GAIN_Pos)
#define I2S_TX_EQUALIZER_GAIN2_BAND10_GAIN  I2S_TX_EQUALIZER_GAIN2_BAND10_GAIN_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
