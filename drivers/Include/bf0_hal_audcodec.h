/**
  ******************************************************************************
  * @file   bf0_hal_audcodec.h
  * @author Sifli software development team
  * @brief   Header file of AUDCODEC HAL module.
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

#ifndef __BF0_HAL_AUDCODEC_H
#define __BF0_HAL_AUDCODEC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup AUDCODEC
  * @brief AUDCODEC HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup AUDCODEC_Exported_Types AUDCODEC Exported Types
  * @{
  */

/**
  * @brief AUDCODEC TX/RX channel configure structure definition
  */
typedef struct
{
    uint8_t     clk_ana_pol;
    uint8_t     dither_en;
    uint8_t     dither_gain;
    uint8_t     sinc_gain;
    uint8_t     data_format;
    uint8_t     fine_vol;
    uint8_t     rough_vol;
    uint8_t     dma_en;
    uint8_t     dem_mode;
    uint8_t     dout_mute;
    uint8_t     enable;
    uint8_t     ramp_interval;
    uint8_t     zero_adjust_en;
    uint8_t     ramp_mode;
    uint8_t     ramp_en;
    uint32_t    dc_offset;
} AUDCODEC_DacChnlCfgTypeDef;

typedef struct
{
    uint8_t     sat_det_len;
    uint8_t     sat_det_en;
    uint8_t     data_format;
    uint8_t     fine_vol;
    uint8_t     rough_vol;
    uint8_t     dma_en;
    uint8_t     stb_inv;
    uint8_t     hpf_coef;
    uint8_t     hpf_bypass;
    uint8_t     enable;
} AUDCODEC_AdcChnlCfgTypeDef;

/**
  * @brief  AUDCODEC DAC clock config definition
  */
typedef struct
{
    uint32_t  samplerate;
    uint8_t   clk_src_sel; /*!< 0:xtal 48M 1:PLL 44.1M */
    uint8_t   clk_div;
    uint8_t   osr_sel;  /*!< 0:100 1:150 2:300 4:64 5:128 6:256 */  //hp  dac_cfg
    uint16_t  sinc_gain;  //hp  dac_ch0_cfg
    uint8_t   sel_clk_dac_source; /*!< 0:xtal 48M 1:PLL  */
    uint8_t   diva_clk_dac;
    uint8_t   diva_clk_chop_dac;
    uint8_t   divb_clk_chop_dac;  // lp pll_cfg4
    uint8_t   diva_clk_chop_bg;
    uint8_t   diva_clk_chop_refgen;  // lp  pll_cfg5
#ifdef SF32LB52X
    uint8_t   sel_clk_dac;
#endif
} AUDCODE_DAC_CLK_CONFIG_TYPE;

/**
  * @brief  AUDCODEC ADC clock config definition
  */
typedef struct
{
    uint32_t  samplerate;
    uint8_t   clk_src_sel; /*!< 0:xtal 48M 1:PLL 44.1M */
    uint8_t   clk_div;
    uint8_t   osr_sel;  /*!< 0:200 1:300 2:400 3:600 */  //lp  adc_cfg
    uint8_t   sel_clk_adc_source; /*!< 0:xtal 48M 1:PLL  */
    uint8_t   sel_clk_adc;
    uint8_t   diva_clk_adc; //lp pll_cfg6
    uint8_t   fsp; //ADC1_CFG1
} AUDCODE_ADC_CLK_CONFIG_TYPE;


/**
  * @brief AUDCODEC DAC PATH configure structure definition
  */

typedef struct
{
    uint8_t     opmode;  /*!<   0:audprc tx to audcode  1: mem tx to audcodec  */
    AUDCODE_DAC_CLK_CONFIG_TYPE *dac_clk;
} AUDCODEC_DACCfgTypeDef;

/**
  * @brief AUDCODEC ADC PATH configure structure definition
  */
typedef struct
{
    uint8_t     opmode;
    AUDCODE_ADC_CLK_CONFIG_TYPE *adc_clk;
} AUDCODEC_ADCCfgTypeDef;

/**
  * @brief AUDCODEC Init structure definition
  */
typedef struct
{
    //uint32_t     clk_div;      /*!<  Source clk for AUDCODEC module */
    //uint16_t    adc_div;           /*!<   adc AUDCODEC adc clock configure  */
    uint16_t    en_dly_sel;           /*!<   codec enable delay count  */
    uint8_t     samplerate_index;
    AUDCODEC_DACCfgTypeDef dac_cfg;    /*!<   dac AUDCODEC DAC PATH configure  */
    AUDCODEC_ADCCfgTypeDef adc_cfg;    /*!<   dac AUDCODEC ADC PATH configure  */
} AUDCODEC_InitTypeDef;

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
    HAL_AUDCODEC_STATE_RESET      = 0x00U,  /*!< AUDCODEC not yet initialized or disabled                */
    HAL_AUDCODEC_STATE_READY      = 0x01U,  /*!< AUDCODEC initialized and ready for use                  */
    HAL_AUDCODEC_STATE_BUSY       = 0x02U,  /*!< AUDCODEC internal process is ongoing                    */
    HAL_AUDCODEC_STATE_BUSY_TX    = 0x04U,  /*!< Data Transmission process is ongoing               */
    HAL_AUDCODEC_STATE_BUSY_RX    = 0x08U,  /*!< Data Reception process is ongoing                  */
    HAL_AUDCODEC_STATE_PAUSE      = 0x16U,  /*!< AUDCODEC pause state: used in case of DMA               */
    HAL_AUDCODEC_STATE_ERROR      = 0x32U   /*!< AUDCODEC error state                                    */
} HAL_AUDCODEC_StateTypeDef;

/**
  * @brief  AUDCODEC TX/RX BUFFER/DMA ENUM definition
  */
typedef enum
{
    HAL_AUDCODEC_DAC_CH0         = 0x00U,  /*!< AUDCODEC DAC channel 0  */
    HAL_AUDCODEC_DAC_CH1         = 0x01U,  /*!< AUDCODEC DAC channel 1  */
    HAL_AUDCODEC_ADC_CH0         = 0x02U,  /*!< AUDCODEC ADC channel 0  */
    HAL_AUDCODEC_ADC_CH1         = 0x03U,  /*!< AUDCODEC ADC channel 1  */
    HAL_AUDCODEC_INSTANC_CNT                /*!< Total dma/buffer counter  */
} HAL_AUDCODEC_ChannelTypeDef;


/**
  * @brief AUDCODEC handle Structure definition
  */
typedef struct
{
#ifdef SF32LB52X
    AUDCODEC_TypeDef                   *Instance;    /*!< AUDCODEC registers base address */
#else
    AUDCODEC_HP_TypeDef                *Instance_hp;    /*!< AUDCODEC HP registers base address */

    AUDCODEC_LP_TypeDef                *Instance_lp;    /*!< AUDCODEC LP registers base address */
#endif
    AUDCODEC_InitTypeDef            Init;         /*!< AUDCODEC communication parameters */

    AUDCODEC_DacChnlCfgTypeDef      DacChnl[2];

    AUDCODEC_AdcChnlCfgTypeDef      AdcChnl[2];

    DMA_HandleTypeDef          *hdma[HAL_AUDCODEC_INSTANC_CNT];   /*!< AUDCODEC Tx/rx Channel DMA handle parameters */


    uint8_t                     *buf[HAL_AUDCODEC_INSTANC_CNT];   /*!< AUDCODEC  data buffer */

    uint32_t                    dataSize[HAL_AUDCODEC_INSTANC_CNT];         /*!< AUDCODEC Data size */
    uint16_t                    bufSize;         /*!< AUDCODEC Data buffer size */
    uint16_t                    channel_ref; /*!< HAL_AUDCODEC_ChannelTypeDef bitmap */

    __IO HAL_LockTypeDef       Lock;         /*!< AUDCODEC locking object */

    __IO HAL_AUDCODEC_StateTypeDef  State[HAL_AUDCODEC_INSTANC_CNT];        /*!< AUDCODEC communication state */

    __IO uint32_t              ErrorCode;    /*!< AUDCODEC Error code
                                                This parameter can be a value of @ref AUDCODEC_Error */
} AUDCODEC_HandleTypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AUDCODEC_Exported_Constants AUDCODEC Exported Constants
  * @{
  */
/** @defgroup AUDCODEC_Error AUDCODEC Error
  * @{
  */
#define HAL_AUDCODEC_ERROR_NONE      (0x00000000U)  /*!< No error           */
#define HAL_AUDCODEC_ERROR_TIMEOUT   (0x00000001U)  /*!< Timeout error      */
#define HAL_AUDCODEC_ERROR_OVR       (0x00000002U)  /*!< OVR error          */
#define HAL_AUDCODEC_ERROR_UDR       (0x00000004U)  /*!< UDR error          */
#define HAL_AUDCODEC_ERROR_DMA       (0x00000008U)  /*!< DMA transfer error */
#define HAL_AUDCODEC_ERROR_UNKNOW    (0x00000010U)  /*!< Unknow Error error */
/**
  * @}
  */


/** @defgroup AUDCODEC_Flag_definition AUDCODEC Flag definition
  * @{
  */

#define AUDCODEC_TX_OUT1_FIFO_UNDERF              (1<<9)  /*!< AUDCODEC STATUS TX OUT1 FIFO UNDER FLOW            */
#define AUDCODEC_TX_OUT0_FIFO_UNDERF              (1<<8)  /*!< AUDCODEC STATUS TX OUT0 FIFO UNDER FLOW          */
#define AUDCODEC_RX_IN_FIFO_OVERF                 (1<<7)  /*!< AUDCODEC STATUS Rx IN FIFO OVER FLOW            */
#define AUDCODEC_TX_OUT_FIFO_UNDERF               (1<<6)  /*!< AUDCODEC STATUS TX OUT FIFO UNDER FLOW           */
#define AUDCODEC_RX1_FIFO_UNDERF                  (1<<5)  /*!< AUDCODEC STATUS Rx0 FIF1 UNDER FLOW           */
#define AUDCODEC_RX0_FIFO_UNDERF                  (1<<4)  /*!< AUDCODEC STATUS Rx0 FIFO UNDER FLOW           */
#define AUDCODEC_TX3_FIFO_OVERF                   (1<<3)  /*!< AUDCODEC STATUS Tx3 FIFO OVER FLOW            */
#define AUDCODEC_TX2_FIFO_OVERF                   (1<<2)  /*!< AUDCODEC STATUS Tx2 FIFO OVER FLOW           */
#define AUDCODEC_TX1_FIFO_OVERF                   (1<<1)  /*!< AUDCODEC STATUS Tx1 FIFO OVER FLOW            */
#define AUDCODEC_TX0_FIFO_OVERF                   (1<<0)  /*!< AUDCODEC STATUS Tx0 FIFO OVER FLOW    */


/**
  * @}
  */


/**
  * @} AUDCODEC_Exported_Constants
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup AUDCODEC_Exported_macros AUDCODEC Exported Macros
  * @{
  */

#ifdef SF32LB52X

/** @brief  Enable audio codec .
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_DAC_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG |= AUDCODEC_CFG_DAC_ENABLE)

/** @brief  Disable the AUDCODEC .
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_DAC_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG &= (~AUDCODEC_CFG_DAC_ENABLE))


/** @brief  Disable the AUDCODEC DAC CHannel0.
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_DAC_CH0_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->DAC_CH0_CFG &= (~AUDCODEC_DAC_CH0_CFG_ENABLE))

/** @brief  Disable the AUDCODEC DAC CHannel1.
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_DAC_CH1_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->DAC_CH1_CFG &= (~AUDCODEC_DAC_CH1_CFG_ENABLE))


/** @brief  Enable audio prc .
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_ADC_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG |= AUDCODEC_CFG_ADC_ENABLE)

/** @brief  Disable the AUDCODEC .
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_ADC_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CFG &= (~AUDCODEC_CFG_ADC_ENABLE))

/** @brief  Disable the AUDCODEC ADC CHannel0.
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_ADC_CH0_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->ADC_CH0_CFG &= (~AUDCODEC_ADC_CH0_CFG_ENABLE))

/** @brief  Disable the AUDCODEC ADC CHannel1.
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_ADC_CH1_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->ADC_CH1_CFG &= (~AUDCODEC_ADC_CH1_CFG_ENABLE))

#else

/** @brief  Enable audio codec .
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_HP_ENABLE(__HANDLE__) ((__HANDLE__)->Instance_hp->CFG |= AUDCODEC_HP_CFG_ENABLE)

/** @brief  Disable the AUDCODEC .
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_HP_DISABLE(__HANDLE__) ((__HANDLE__)->Instance_hp->CFG &= (~AUDCODEC_HP_CFG_ENABLE))


/** @brief  Disable the AUDCODEC DAC CHannel0.
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_HP_DAC_CH0_DISABLE(__HANDLE__) ((__HANDLE__)->Instance_hp->DAC_CH0_CFG &= (~AUDCODEC_HP_DAC_CH0_CFG_ENABLE))

/** @brief  Disable the AUDCODEC DAC CHannel1.
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_HP_DAC_CH1_DISABLE(__HANDLE__) ((__HANDLE__)->Instance_hp->DAC_CH1_CFG &= (~AUDCODEC_HP_DAC_CH1_CFG_ENABLE))




/** @brief  Enable audio prc .
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_LP_ENABLE(__HANDLE__) ((__HANDLE__)->Instance_lp->CFG |= AUDCODEC_LP_CFG_ENABLE)

/** @brief  Disable the AUDCODEC .
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_LP_DISABLE(__HANDLE__) ((__HANDLE__)->Instance_lp->CFG &= (~AUDCODEC_LP_CFG_ENABLE))

/** @brief  Disable the AUDCODEC ADC CHannel0.
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_HP_ADC_CH0_DISABLE(__HANDLE__) ((__HANDLE__)->Instance_lp->ADC_CH0_CFG &= (~AUDCODEC_LP_ADC_CH0_CFG_ENABLE))

/** @brief  Disable the AUDCODEC ADC CHannel1.
  * @param  \__HANDLE__ specifies the AUDCODEC Handle.
  * @retval None
  */
#define __HAL_AUDCODEC_HP_ADC_CH1_DISABLE(__HANDLE__) ((__HANDLE__)->Instance_lp->ADC_CH1_CFG &= (~AUDCODEC_LP_ADC_CH1_CFG_ENABLE))

#endif

/**
  * @} AUDCODEC_Exported_macros
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup AUDCODEC_Exported_Functions
  * @{
  */
HAL_StatusTypeDef HAL_AUDCODEC_Config_Analog_DACPath(AUDCODE_DAC_CLK_CONFIG_TYPE *cfg);
void HAL_TURN_OFF_PLL();

void HAL_AUDCODEC_Close_Analog_DACPath(void);

void HAL_AUCODEC_Refgen_Init(void);

void HAL_AUDCODEC_Config_Analog_ADCPath(AUDCODE_ADC_CLK_CONFIG_TYPE *cfg);

void HAL_AUDCODEC_Close_Analog_ADCPath(void);


/** @addtogroup AUDCODEC_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/

/**
  * @brief Initializes the AUDCODEC according to the specified parameters
  *         in the AUDCODEC_InitTypeDef and create the associated handle.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_Init(AUDCODEC_HandleTypeDef *hacodec);

/**
  * @brief DeInitializes the AUDCODEC peripheral
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_DeInit(AUDCODEC_HandleTypeDef *hacodec);

/**
  * @brief Initializes the AUDCODEC pin setting.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @retval None
  */
void HAL_AUDCODEC_MspInit(AUDCODEC_HandleTypeDef *hacodec);

/**
  * @brief deunitializes the AUDCODEC pin setting.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @retval None
  */
void HAL_AUDCODEC_MspDeInit(AUDCODEC_HandleTypeDef *hacodec);
/**
  * @} AUDCODEC_Exported_Functions_Group1
  */

/** @addtogroup AUDCODEC_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
/* Configure AUDCODEC parameters  */

/**
  * @brief Set AUDCODEC transmit channel configures.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param channel: configure channel number
  * @param cfg: configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_Config_TChanel(AUDCODEC_HandleTypeDef *hacodec, int channel, AUDCODEC_DACCfgTypeDef *cfg);

/**
  * @brief Set AUDCODEC Receive configures.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param channel: configure channel number
  * @param cfg: configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_Config_RChanel(AUDCODEC_HandleTypeDef *hacodec, int channel, AUDCODEC_ADCCfgTypeDef *cfg);


/**
 * @brief  all channel disable
 * @param hacodec -pointer to a AUDCODEC_HandleTypeDef structure that contains
 *         the configuration information for AUDCODEC module
 * @param ch_type_bit : bit0 is dac, bit1 is adc
 * @return HAL status
 */
HAL_StatusTypeDef HAL_AUDCODEC_Clear_All_Channel(AUDCODEC_HandleTypeDef *hacodec, uint8_t ch_type_bit);


/**
  * @brief  Configure DAC path volume.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param channel: channel 0 or 1
  * @param volume: configure volume -36dB~54dB, unit is 0.5db, use fix point Q15.1 format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_Config_DACPath_Volume(AUDCODEC_HandleTypeDef *hacodec, int channel, int volume);

/**
  * @brief  Bypass DAC path.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param  bypass 1:enable, 0: disable
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_Config_DACPath(AUDCODEC_HandleTypeDef *hacodec, uint16_t bypass);


/**
  * @brief  Configure DAC path mute.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param mute: is mute or not
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_Mute_DACPath(AUDCODEC_HandleTypeDef *hacodec, int mute);


/**
  * @brief  Configure ADC path.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param cfg: configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_Config_ADCPath(AUDCODEC_HandleTypeDef *hacodec, AUDCODEC_ADCCfgTypeDef *cfg);


/**
  * @} AUDCODEC_Exported_Functions_Group2
  */

/** @addtogroup AUDCODEC_Exported_Functions_Group3
  * @{
  */
/* DMA operation functions  ***************************************************/

/**
  * @brief Set AUDCODEC Transmit data with DMA mode.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param pData: data buffer
  * @param Size: data length need to be transmit
  * @param did:  DMA channel index
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_AUDCODEC_Transmit_DMA(AUDCODEC_HandleTypeDef *hacodec, uint8_t *pData, uint32_t Size, uint32_t did);

/**
  * @brief Set AUDCODEC Receive configures.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param pData: data buffer
  * @param Size: data length need to be transmit
  * @param did:  DMA channel index
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_Receive_DMA(AUDCODEC_HandleTypeDef *hacodec, uint8_t *pData, uint32_t Size, uint32_t did);

/**
  * @brief Stop AUD PRC DMA.
  * @param  hacodec: pointer to a AUDCODEC_HandleTypeDef structure that contains
  *         the configuration information for AUDCODEC module
  * @param did:  DMA channel index
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AUDCODEC_DMAStop(AUDCODEC_HandleTypeDef *hacodec, uint32_t did);

void set_pll_state(uint8_t state);
uint8_t get_pll_state();
void set_pll_freq_type(uint8_t type);
uint8_t get_pll_freq_type();
int updata_pll_freq(uint8_t type);
int bf0_enable_pll(uint32_t freq, uint8_t type);
void bf0_disable_pll();
void HAL_TURN_ON_PLL();

/**
  * @} AUDCODEC_Exported_Functions_Group3
  */

/**
  * @} AUDCODEC_Exported_Functions
*/

/**
  * @} AUDCODEC
*/

/**
  * @} BF0_HAL_Driver
*/

#ifdef __cplusplus
}
#endif


#endif /* __BF0_HAL_I2S_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
