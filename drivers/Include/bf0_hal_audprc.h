/**
  ******************************************************************************
  * @file   bf0_hal_audprc.h
  * @author Sifli software development team
  * @brief   Header file of AUDPRC HAL module.
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_AUDPRC_H
#define __BF0_HAL_AUDPRC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup AUDPRC
  * @brief AUDPRC HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup AUDPRC_Exported_Types AUDPRC Exported Types
  * @{
  */

/**
  * @brief AUDPRC TX/RX channel configure structure definition
  */
typedef struct
{
    uint8_t     dma_mask;      /*!<  dma mask */
    uint8_t     mode;           /*!<   tx/rx mode, 0 mono, 1 stereo , only for 16 bit format,
                                     *   for 24 bit format, channel can only set to mono mode */
    uint8_t     format;           /*!<   tx/rx format, 0 for 16 bit, 1 for 24 bit */
    uint8_t     en;           /*!<   enable this channel  */
} AUDPRC_ChnlCfgTypeDef;

/**
  * @brief AUDPRC DAC PATH configure structure definition
  */

typedef struct
{
    uint8_t     dst_sel;
    uint8_t     mixrsrc1;
    uint8_t     mixrsrc0;
    uint8_t     mixlsrc1;
    uint8_t     mixlsrc0;
    int8_t     vol_r;
    int8_t     vol_l;
    uint8_t     src_hbf3_mode;
    uint8_t     src_hbf3_en;
    uint8_t     src_hbf2_mode;
    uint8_t     src_hbf2_en;
    uint8_t     src_hbf1_mode;
    uint8_t     src_hbf1_en;
    uint8_t     src_ch_en;
    uint8_t     eq_clr;
    uint8_t     eq_stage;
    uint8_t     eq_ch_en;
    uint8_t     muxrsrc1;
    uint8_t     muxrsrc0;
    uint8_t     muxlsrc1;
    uint8_t     muxlsrc0;
    uint8_t     src_sinc_en;
    uint32_t    sinc_ratio;
} AUDPRC_DACCfgTypeDef;

/**
  * @brief AUDPRC ADC PATH configure structure definition
  */
typedef struct
{
    uint8_t     src_hbf3_mode;
    uint8_t     src_hbf3_en;
    uint8_t     src_hbf2_mode;
    uint8_t     src_hbf2_en;
    uint8_t     src_hbf1_mode;
    uint8_t     src_hbf1_en;
    uint8_t     src_ch_en;
    uint8_t     rx2tx_loopback;
    uint8_t     data_swap;
    uint8_t     src_sel;
    int8_t     vol_r;
    int8_t     vol_l;
    uint8_t     src_sinc_en;
    uint32_t    sinc_ratio;
} AUDPRC_ADCCfgTypeDef;

/**
  * @brief AUDPRC Init structure definition
  */
typedef struct
{
    uint32_t     clk_div;      /*!<  Source clk for AUDPRC module */
    uint16_t    adc_div;           /*!<   adc AUDPRC adc clock configure  */
    uint16_t    dac_div;           /*!<   dac AUDPRC dac clock configure  */
    uint8_t     clk_sel;            /*!< 0:xtal 48M 1:PLL 44.1M */
    AUDPRC_DACCfgTypeDef dac_cfg;    /*!<   dac AUDPRC DAC PATH configure  */
    AUDPRC_ADCCfgTypeDef adc_cfg;    /*!<   dac AUDPRC ADC PATH configure  */
} AUDPRC_InitTypeDef;

/**
  * @brief  audprc  clock config definition
  */
typedef struct
{
    uint32_t  samplerate;
    uint8_t   clk_src_sel; /*!< 0:xtal 48M 1:PLL 44.1M */
    uint16_t   clk_div;
} AUDPRC_CLK_CONFIG_TYPE;

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
    HAL_AUDPRC_STATE_RESET      = 0x00U,  /*!< AUDPRC not yet initialized or disabled                */
    HAL_AUDPRC_STATE_READY      = 0x01U,  /*!< AUDPRC initialized and ready for use                  */
    HAL_AUDPRC_STATE_BUSY       = 0x02U,  /*!< AUDPRC internal process is ongoing                    */
    HAL_AUDPRC_STATE_BUSY_TX    = 0x04U,  /*!< Data Transmission process is ongoing               */
    HAL_AUDPRC_STATE_BUSY_RX    = 0x08U,  /*!< Data Reception process is ongoing                  */
    HAL_AUDPRC_STATE_PAUSE      = 0x16U,  /*!< AUDPRC pause state: used in case of DMA               */
    HAL_AUDPRC_STATE_ERROR      = 0x32U   /*!< AUDPRC error state                                    */
} HAL_AUDPRC_StateTypeDef;

/**
  * @brief  AUDPRC TX/RX BUFFER/DMA ENUM definition
  */
typedef enum
{
    HAL_AUDPRC_TX_CH0         = 0x00U,  /*!< AUDPRC TX channel 0  */
    HAL_AUDPRC_TX_CH1         = 0x01U,  /*!< AUDPRC TX channel 1  */
    HAL_AUDPRC_TX_CH2         = 0x02U,  /*!< AUDPRC TX channel 2  */
    HAL_AUDPRC_TX_CH3         = 0x03U,  /*!< AUDPRC TX channel 3 */
    HAL_AUDPRC_RX_CH0         = 0x04U,  /*!< AUDPRC RX channel 0  */
    HAL_AUDPRC_RX_CH1         = 0x05U,  /*!< AUDPRC RX channel 1  */
    HAL_AUDPRC_TX_OUT_CH0     = 0x06U,   /*!< AUDPRC TX Output channel 0  */
    HAL_AUDPRC_TX_OUT_CH1     = 0x07U,   /*!< AUDPRC TX Output channel 1  */
    HAL_AUDPRC_INSTANC_CNT                /*!< Total dma/buffer counter  */
} HAL_AUDPRC_ChannelTypeDef;

/**
  * @brief AUDPRC handle Structure definition
  */
typedef struct
{
    AUDPRC_TypeDef                *Instance;    /*!< AUDPRC registers base address */

    AUDPRC_InitTypeDef            Init;         /*!< AUDPRC communication parameters */

    DMA_HandleTypeDef          *hdma[HAL_AUDPRC_INSTANC_CNT];   /*!< AUDPRC Tx/rx Channel DMA handle parameters */
    //DMA_HandleTypeDef          *hdmatx0;      /*!< AUDPRC Tx Channel0 DMA handle parameters */
    //DMA_HandleTypeDef          *hdmatx1;      /*!< AUDPRC Tx Channel1 DMA handle parameters */
    //DMA_HandleTypeDef          *hdmatx2;      /*!< AUDPRC Tx Channel2 DMA handle parameters */
    //DMA_HandleTypeDef          *hdmatx3;      /*!< AUDPRC Tx Channel3 DMA handle parameters */
    //DMA_HandleTypeDef          *hdmarx0;      /*!< AUDPRC Rx Channel0 DMA handle parameters */
    //DMA_HandleTypeDef          *hdmarx1;      /*!< AUDPRC Rx Channel1 DMA handle parameters */
    //DMA_HandleTypeDef          *hdmatoc0;      /*!< AUDPRC Tx Out Channel0 DMA handle parameters */
    //DMA_HandleTypeDef          *hdmatoc1;      /*!< AUDPRC Tx Out Channel1 DMA handle parameters */

    uint8_t                     *buf[HAL_AUDPRC_INSTANC_CNT];   /*!< AUDPRC  data buffer */
    //uint8_t                     *txbuf0;     /*!< AUDPRC Tx channel 0 data buffer */
    //uint8_t                     *txbuf1;    /*!< AUDPRC Tx channel 1 data buffer */
    //uint8_t                     *txbuf2;    /*!< AUDPRC Tx channel 2 data buffer */
    //uint8_t                     *txbuf3;    /*!< AUDPRC Tx channel 3 data buffer */
    //uint8_t                     *rxbuf0;    /*!< AUDPRC Rx channel 0 data buffer */
    //uint8_t                     *rxbuf1;     /*!< AUDPRC Rx channel 1 data buffer */
    //uint8_t                     *tobuf0;    /*!< AUDPRC Tx Out channel 0 data buffer */
    //uint8_t                     *tobuf1;    /*!< AUDPRC Tx Out channel 1 data buffer */
    uint32_t                    dataSize[HAL_AUDPRC_INSTANC_CNT];         /*!< AUDPRC Data size */
    uint16_t                    bufTxSize;         /*!< AUDPRC Data buffer size */
    uint16_t                    bufRxSize;         /*!< AUDPRC Data buffer size */

    __IO HAL_LockTypeDef       Lock;         /*!< AUDPRC locking object */

    __IO HAL_AUDPRC_StateTypeDef  State[HAL_AUDPRC_INSTANC_CNT];        /*!< AUDPRC communication state */

    __IO uint32_t              ErrorCode;    /*!< AUDPRC Error code
                                                This parameter can be a value of @ref AUDPRC_Error */
    uint16_t                   dest_sel;     /*!< AUDPRC destination path selet  */
    uint16_t                   channel_ref; /*!< HAL_AUDPRC_ChannelTypeDef bit map */
    AUDPRC_ChnlCfgTypeDef      cfg;
    AUDPRC_ChnlCfgTypeDef      cfg1;
} AUDPRC_HandleTypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AUDPRC_Exported_Constants AUDPRC Exported Constants
  * @{
  */
/** @defgroup AUDPRC_Error AUDPRC Error
  * @{
  */
#define HAL_AUDPRC_ERROR_NONE      (0x00000000U)  /*!< No error           */
#define HAL_AUDPRC_ERROR_TIMEOUT   (0x00000001U)  /*!< Timeout error      */
#define HAL_AUDPRC_ERROR_OVR       (0x00000002U)  /*!< OVR error          */
#define HAL_AUDPRC_ERROR_UDR       (0x00000004U)  /*!< UDR error          */
#define HAL_AUDPRC_ERROR_DMA       (0x00000008U)  /*!< DMA transfer error */
#define HAL_AUDPRC_ERROR_UNKNOW    (0x00000010U)  /*!< Unknow Error error */
/**
  * @}
  */


/** @defgroup AUDPRC_Flag_definition AUDPRC Flag definition
  * @{
  */

#define AUDPRC_TX_OUT1_FIFO_UNDERF              (1<<9)  /*!< AUDPRC STATUS TX OUT1 FIFO UNDER FLOW            */
#define AUDPRC_TX_OUT0_FIFO_UNDERF              (1<<8)  /*!< AUDPRC STATUS TX OUT0 FIFO UNDER FLOW          */
#define AUDPRC_RX_IN_FIFO_OVERF                 (1<<7)  /*!< AUDPRC STATUS Rx IN FIFO OVER FLOW            */
#define AUDPRC_TX_OUT_FIFO_UNDERF               (1<<6)  /*!< AUDPRC STATUS TX OUT FIFO UNDER FLOW           */
#define AUDPRC_RX1_FIFO_UNDERF                  (1<<5)  /*!< AUDPRC STATUS Rx0 FIF1 UNDER FLOW           */
#define AUDPRC_RX0_FIFO_UNDERF                  (1<<4)  /*!< AUDPRC STATUS Rx0 FIFO UNDER FLOW           */
#define AUDPRC_TX3_FIFO_OVERF                   (1<<3)  /*!< AUDPRC STATUS Tx3 FIFO OVER FLOW            */
#define AUDPRC_TX2_FIFO_OVERF                   (1<<2)  /*!< AUDPRC STATUS Tx2 FIFO OVER FLOW           */
#define AUDPRC_TX1_FIFO_OVERF                   (1<<1)  /*!< AUDPRC STATUS Tx1 FIFO OVER FLOW            */
#define AUDPRC_TX0_FIFO_OVERF                   (1<<0)  /*!< AUDPRC STATUS Tx0 FIFO OVER FLOW    */

/**
  * @}
  */


/** @defgroup AUDPRC_Source_definition AUDPRC Source definition
  * @{
  */

#define AUDPRC_RX_FROM_CODEC            (0)  /*!< AUDPRC Source from audio codec */
#define AUDPRC_RX_FROM_I2S              (1)  /*!< AUDPRC Srouce from external interface */

/**
  * @}
  */
/** @defgroup AUDPRC_Input_definition AUDPRC Input definition
  * @{
  */

#define AUDPRC_RX_TO_MEM              (0)  /*!< AUDPRC Output to RX buffer */
#define AUDPRC_RX_TO_MUX              (1)  /*!< AUDPRC Output to DAC MUX */

/**
  * @}
  */

/** @defgroup AUDPRC_Output_definition AUDPRC Output definition
  * @{
  */

#define AUDPRC_TX_TO_MEM              (2)  /*!< AUDPRC Output to TX OUT buffer */
#define AUDPRC_TX_TO_I2S              (1)  /*!< AUDPRC Output to I2S TX */
#define AUDPRC_TX_TO_CODEC            (0)  /*!< AUDPRC Output to audio codec  */

/**
  * @}
  */
#define AUDPRC_EQ_ENABLE              (1)

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup AUDPRC_Exported_macros AUDPRC Exported Macros
  * @{
  */

/** @brief  Reset AUDPRC handle state
  * @param  \__HANDLE__ AUDPRC handle.
  * @retval None
  */
#define __HAL_AUDPRC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_AUDPRC_STATE_RESET)

/** @brief  Enable audio prc .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG |= AUDPRC_CFG_ENABLE)

/** @brief  Disable the AUDPRC .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG &= (~AUDPRC_CFG_ENABLE))

/** @brief  Disable the AUDPRC dac path .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_DACPATH_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG &= (~AUDPRC_CFG_DAC_PATH_EN))

/** @brief  Enable audprc dac path .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_DACPATH_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG |= AUDPRC_CFG_DAC_PATH_EN)

/** @brief  Disable the AUDPRC adc path .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_ADCPATH_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG &= (~AUDPRC_CFG_ADC_PATH_EN))

/** @brief  Enable the audprc adc path .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_ADCPATH_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG |= AUDPRC_CFG_ADC_PATH_EN)

/** @brief AUDPRC soft reset start.
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_SRESET_START(__HANDLE__) (((__HANDLE__)->Instance->CFG) |= (AUDPRC_CFG_SRESET))

/** @brief AUDPRC soft reset stop.
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_SRESET_STOP(__HANDLE__) (((__HANDLE__)->Instance->CFG) &= (~AUDPRC_CFG_SRESET))

/** @brief AUDPRC DAC PATH soft reset.
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_DACPATH_SRESET(__HANDLE__) (((__HANDLE__)->Instance->CFG) |= (AUDPRC_CFG_DAC_PATH_SRESET))

/** @brief AUDPRC ADC PATH soft reset.
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_ADCPATH_SRESET(__HANDLE__) (((__HANDLE__)->Instance->CFG) |= (AUDPRC_CFG_ADC_PATH_SRESET))

/** @brief AUDPRC auto clock gate enable.
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_AUTO_GATE_EN(__HANDLE__) (((__HANDLE__)->Instance->CFG) |= (AUDPRC_CFG_AUTO_GATE_EN))

/** @brief Set the AUDPRC clock from crystal.
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_CLK_XTAL(__HANDLE__) (((__HANDLE__)->Instance->CFG) &= (~AUDPRC_CFG_STB_CLK_SEL))

/** @brief Set the AUDPRC clock from pll.
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_CLK_PLL(__HANDLE__) (((__HANDLE__)->Instance->CFG) |= (AUDPRC_CFG_STB_CLK_SEL))

/** @brief Set the AUDPRC DAC dest to audio codec.
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_DAC_DST_CODEC(__HANDLE__) (MODIFY_REG(((__HANDLE__)->Instance->DAC_PATH_CFG0), AUDPRC_DAC_PATH_CFG0_DST_SEL_Msk,\
               MAKE_REG_VAL(AUDPRC_TX_TO_CODEC, AUDPRC_DAC_PATH_CFG0_DST_SEL_Msk, AUDPRC_DAC_PATH_CFG0_DST_SEL_Pos)))

/** @brief Set the AUDPRC DAC dest to exteral interface(I2S).
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_DAC_DST_I2S(__HANDLE__) (MODIFY_REG(((__HANDLE__)->Instance->DAC_PATH_CFG0), AUDPRC_DAC_PATH_CFG0_DST_SEL_Msk,\
               MAKE_REG_VAL(AUDPRC_TX_TO_I2S, AUDPRC_DAC_PATH_CFG0_DST_SEL_Msk, AUDPRC_DAC_PATH_CFG0_DST_SEL_Pos)))

/** @brief Set the AUDPRC DAC dest to APB interface(memory) .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_DAC_DST_MEM(__HANDLE__) (MODIFY_REG(((__HANDLE__)->Instance->DAC_PATH_CFG0), AUDPRC_DAC_PATH_CFG0_DST_SEL_Msk,\
               MAKE_REG_VAL(AUDPRC_TX_TO_MEM, AUDPRC_DAC_PATH_CFG0_DST_SEL_Msk, AUDPRC_DAC_PATH_CFG0_DST_SEL_Pos)))

/** @brief Set the AUDPRC ADC source from audio codec .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_ADC_SRC_CODEC(__HANDLE__) (((__HANDLE__)->Instance->ADC_PATH_CFG0) &= (~AUDPRC_ADC_PATH_CFG0_SRC_SEL))

/** @brief Set the AUDPRC ADC source from external interface(I2S) .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @retval None
  */
#define __HAL_AUDPRC_ADC_SRC_I2S(__HANDLE__) (((__HANDLE__)->Instance->ADC_PATH_CFG0) |= (AUDPRC_ADC_PATH_CFG0_SRC_SEL))

/** @brief Set the AUDPRC STB div clk .
  * @param  \__HANDLE__ specifies the AUDPRC Handle.
  * @param  adc_div adc clock divider.
  * @param  dac_div dac clock divider.
  * @retval None
  */
#define __HAL_AUDPRC_STB_DIV_CLK(__HANDLE__, adc_div, dac_div) (((__HANDLE__)->Instance->STB) = ((uint32_t)(adc_div) << AUDPRC_STB_ADC_DIV_Pos) | ((uint32_t)(dac_div) << AUDPRC_STB_DAC_DIV_Pos))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup AUDPRC_Exported_Functions
  * @{
  */

/** @addtogroup AUDPRC_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/

/**
  * @brief Initializes the AUDPRC according to the specified parameters
  *         in the AUDPRC_InitTypeDef and create the associated handle.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_Init(AUDPRC_HandleTypeDef *haprc);

/**
  * @brief DeInitializes the AUDPRC peripheral
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_DeInit(AUDPRC_HandleTypeDef *haprc);

/**
  * @brief Initializes the AUDPRC pin setting.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @retval None
  */
void HAL_AUDPRC_MspInit(AUDPRC_HandleTypeDef *haprc);

/**
  * @brief deunitializes the AUDPRC pin setting.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @retval None
  */
void HAL_AUDPRC_MspDeInit(AUDPRC_HandleTypeDef *haprc);
/**
  * @}
  */

/** @addtogroup AUDPRC_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
/* Configure AUDPRC parameters  */

/**
  * @brief Set AUDPRC transmit channel configures.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param channel: configure channel
  * @param cfg: configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_Config_TChanel(AUDPRC_HandleTypeDef *haprc, int channel, AUDPRC_ChnlCfgTypeDef *cfg);

/**
  * @brief Set AUDPRC Receive configures.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param channel: configure channel
  * @param cfg: configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_Config_RChanel(AUDPRC_HandleTypeDef *haprc, int channel, AUDPRC_ChnlCfgTypeDef *cfg);

/**
  * @brief Set AUDPRC transmit output channel configures.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param channel: configure channel
  * @param cfg: configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_Config_OutChanel(AUDPRC_HandleTypeDef *haprc, int channel, AUDPRC_ChnlCfgTypeDef *cfg);

/**
 * @brief  all channel disable
 * @param haprc -pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
 * @return HAL status
 */
HAL_StatusTypeDef HAL_AUDPRC_Clear_All_Channel(AUDPRC_HandleTypeDef *haprc);
HAL_StatusTypeDef HAL_AUDPRC_Clear_Adc_Channel(AUDPRC_HandleTypeDef *haprc);
HAL_StatusTypeDef HAL_AUDPRC_Clear_Dac_Channel(AUDPRC_HandleTypeDef *haprc);

/**
  * @brief  Configure DAC path.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param cfg: configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_Config_DACPath(AUDPRC_HandleTypeDef *haprc, AUDPRC_DACCfgTypeDef *cfg);

/**
  * @brief  Configure ADC path.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param cfg: configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_Config_ADCPath(AUDPRC_HandleTypeDef *haprc, AUDPRC_ADCCfgTypeDef *cfg);


/**
  * @}
  */

/** @addtogroup AUDPRC_Exported_Functions_Group3
  * @{
  */
/* DMA operation functions  ***************************************************/

/**
  * @brief Set AUDPRC Transmit data with DMA mode.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param pData: data buffer
  * @param Size: data length need to be transmit
  * @param did:  DMA channel index
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_AUDPRC_Transmit_DMA(AUDPRC_HandleTypeDef *haprc, uint8_t *pData, uint32_t Size, uint32_t did);

/**
  * @brief Set AUDPRC Receive configures.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param pData: data buffer
  * @param Size: data length need to be transmit
  * @param did:  DMA channel index
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_Receive_DMA(AUDPRC_HandleTypeDef *haprc, uint8_t *pData, uint32_t Size, uint32_t did);

/**
  * @brief Stop AUD PRC DMA.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param did:  DMA channel index
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDPRC_DMAStop(AUDPRC_HandleTypeDef *haprc, uint32_t did);

/**
  * @}
  */

/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* __BF0_HAL_I2S_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
