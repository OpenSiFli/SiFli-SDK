/**
  ******************************************************************************
  * @file   bf0_hal_i2s.h
  * @author Sifli software development team
  * @brief   Header file of I2S HAL module.
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

#ifndef __BF0_HAL_I2S_H
#define __BF0_HAL_I2S_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup I2S
  * @brief I2S HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup I2S_Exported_Types I2S Exported Types
  * @{
  */

/**
  * @brief I2S bclk lrck divder table structure definition
  */
typedef struct
{
    uint32_t  samplerate;         /*!<  Sample rate   */
    uint16_t  lr_clk_duty_high;   /*!<  LRCK duty cycle high   */
    uint16_t  lr_clk_duty_low;    /*!<  RX LRCK duty cycle low   */
    uint16_t  blck_duty;          /*!<  bit clock duty cycle   */
} CLK_DIV_T;


/**
  * @brief I2S bus configure structure definition
  */
typedef struct
{
    uint8_t bus_dw;         /*!<  I2S bus data width 8, 16, 24 32 ...*/
    uint8_t data_dw;         /*!<  output data width, only support 8 or 16 */
    uint8_t slave_mode;      /*!<  0 master mode, 1 slave mode.*/
    uint8_t track;          /*!<  1 mono, 0s stereo */
    uint8_t chnl_sel;       /*!<  0 noral, 5 left/write switch , 0xa left right average */
    uint8_t lrck_invert;    /*!<  for standard I2S, set lrck_pol to 0,for Left/Right Justified, set lrck_pol to 1*/
    uint32_t sample_rate;   /*!<  Sample rate   */
    uint32_t bclk;          /*!<  I2S BCLK   */
    uint8_t vol;            /*!<  0 ~ 15, 0=+6db, 4=0db, 14=-15db, 15=mute */
    uint8_t balance_en;     /*!<  0 balance disable, 1 right channel adjust, 2 left channel adjust*/
    uint8_t balance_vol;    /*!<  when balance_en=1, 1 ~ 15, 1=-1.5db, 2=-3db ... 14=-21db, 15=mute */
    uint8_t extern_intf;    /*!<  0 use internal apb memory, 1 use external interface */
    uint8_t pcm_dw;          /*!<  For I2S rx/tx data width , related with peripheral */
    uint8_t clk_div_index;  /*!<  clock divder index(for txrx_clk_div), base on sample rate */
    CLK_DIV_T *clk_div;
} I2S_CFG_T;


/**
  * @brief I2S Init structure definition
  */
typedef struct
{
    uint32_t  src_clk_freq;      /*!<  Source clk for I2S module */
#ifndef SF32LB55X
    uint32_t mclk_div;            /*!<  I2S mclock for I2S module */
#endif
    I2S_CFG_T tx_cfg;           /*!<   Transmit i2s configure  */
    I2S_CFG_T rx_cfg;           /*!<   Receive i2s configure  */
} I2S_InitTypeDef;

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
    HAL_I2S_STATE_RESET      = 0x00U,  /*!< I2S not yet initialized or disabled                */
    HAL_I2S_STATE_READY      = 0x01U,  /*!< I2S initialized and ready for use                  */
    HAL_I2S_STATE_BUSY       = 0x02U,  /*!< I2S internal process is ongoing                    */
    HAL_I2S_STATE_BUSY_TX    = 0x04U,  /*!< Data Transmission process is ongoing               */
    HAL_I2S_STATE_BUSY_RX    = 0x08U,  /*!< Data Reception process is ongoing                  */
    HAL_I2S_STATE_PAUSE      = 0x16U,  /*!< I2S pause state: used in case of DMA               */
    HAL_I2S_STATE_ERROR      = 0x32U   /*!< I2S error state                                    */
} HAL_I2S_StateTypeDef;

/**
  * @brief I2S handle Structure definition
  */
typedef struct
{
    I2S_TypeDef                *Instance;    /*!< I2S registers base address */

    I2S_InitTypeDef            Init;         /*!< I2S communication parameters */

    uint32_t                   *pTxBuffPtr;  /*!< Pointer to I2S Tx transfer buffer */

    __IO uint32_t              TxXferSize;   /*!< I2S Tx transfer size */

    __IO uint32_t              TxXferCount;  /*!< I2S Tx transfer Counter */

    uint32_t                   *pRxBuffPtr;  /*!< Pointer to I2S Rx transfer buffer */

    __IO uint32_t              RxXferSize;   /*!< I2S Rx transfer size */

    __IO uint32_t              RxXferCount;  /*!< I2S Rx transfer counter
                                              (This field is initialized at the
                                               same value as transfer size at the
                                               beginning of the transfer and
                                               decremented when a sample is received.
                                               NbSamplesReceived = RxBufferSize-RxBufferCount) */

    DMA_HandleTypeDef          *hdmatx;      /*!< I2S Tx DMA handle parameters */

    DMA_HandleTypeDef          *hdmarx;      /*!< I2S Rx DMA handle parameters */

    __IO HAL_LockTypeDef       Lock;         /*!< I2S locking object */

    __IO HAL_I2S_StateTypeDef  State;        /*!< I2S communication state */

    __IO uint32_t              ErrorCode;    /*!< I2S Error code
                                                This parameter can be a value of @ref I2S_Error */

} I2S_HandleTypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2S_Exported_Constants I2S Exported Constants
  * @{
  */
/** @defgroup I2S_Error I2S Error
  * @{
  */
#define HAL_I2S_ERROR_NONE      (0x00000000U)  /*!< No error           */
#define HAL_I2S_ERROR_TIMEOUT   (0x00000001U)  /*!< Timeout error      */
#define HAL_I2S_ERROR_OVR       (0x00000002U)  /*!< OVR error          */
#define HAL_I2S_ERROR_UDR       (0x00000004U)  /*!< UDR error          */
#define HAL_I2S_ERROR_DMA       (0x00000008U)  /*!< DMA transfer error */
#define HAL_I2S_ERROR_UNKNOW    (0x00000010U)  /*!< Unknow Error error */
/**
  * @}
  */


/** @defgroup I2S_Interrupt_configuration_definition I2S Interrupt configuration definition
  * @{
  */
#define I2S_IT_TXE                      I2S_INT_MASK_TX_FIFO_INT_MASK   /*!< Tx empty interrupt           */
#define I2S_IT_RXNE                     I2S_INT_MASK_RX_FIFO_INT_MASK   /*!< Rx not empty interrupt          */
/**
  * @}
  */

/** @defgroup I2S_Flag_definition I2S Flag definition
  * @{
  */
#define I2S_FLAG_TXE                    I2S_INT_STATUS_TX_FIFO_UNDERFLOW    /*!< I2S STATUS TX EMPTY           */
#define I2S_FLAG_RXNE                   I2S_INT_STATUS_RX_FIFO_OVERFLOW /*!< I2S STATUS RX NOT EMPTY           */

#define I2S_FIFO_TX_FULL                (1<<7)  /*!< I2S STATUS Tx full           */
#define I2S_FIFO_TX_EMPTY               (1<<6)  /*!< I2S STATUS Tx empty           */
#define I2S_FIFO_TX_AMFULL              (1<<5)  /*!< I2S STATUS Tx almost full           */
#define I2S_FIFO_TX_AMEMPTY             (1<<4)  /*!< I2S STATUS Tx almost empty           */
#define I2S_FIFO_RX_FULL                (1<<3)  /*!< I2S STATUS Rx full           */
#define I2S_FIFO_RX_EMPTY               (1<<2)  /*!< I2S STATUS Rx empty           */
#define I2S_FIFO_RX_AMFULL              (1<<1)  /*!< I2S STATUS Rx almost full           */
#define I2S_FIFO_RX_AMEMPTY             (1<<0)  /*!< I2S STATUS Rx almost empty          */

/**
  * @}
  */

/** @defgroup I2S_sample_rate_definition I2S sample rate definition
  * @{
  */
#define I2S_SAMPLERATE_8000                     (8000)      /*!< sample rate 8K bps           */
#define I2S_SAMPLERATE_16000                    (16000)     /*!< sample rate 16K bps           */
#define I2S_SAMPLERATE_24000                    (24000)     /*!< sample rate 24K bps            */
#define I2S_SAMPLERATE_32000                    (32000)     /*!< sample rate 32K bps            */
#define I2S_SAMPLERATE_48000                    (48000)     /*!< sample rate 48K bps            */

#define I2S_SAMPLERATE_44100                    (44100)     /*!< Isample rate 44.1K bps            */
#define I2S_SAMPLERATE_22050                    (22050)     /*!< sample rate 22.05K bps            */
#define I2S_SAMPLERATE_11025                    (11025)     /*!< sample rate 11.025K bps            */

/**
  * @brief CHECK if valid sampel rate number
  * @param  SR Sample rate.
  * @retval true if success
  */
#define IS_I2S_SAMPLE_RATE(SR)      (((SR) == I2S_SAMPLERATE_8000) || \
                                     ((SR) == I2S_SAMPLERATE_16000) || \
                                     ((SR) == I2S_SAMPLERATE_24000) || \
                                     ((SR) == I2S_SAMPLERATE_32000) || \
                                     ((SR) == I2S_SAMPLERATE_48000) || \
                                     ((SR) == I2S_SAMPLERATE_44100) || \
                                     ((SR) == I2S_SAMPLERATE_22050) || \
                                     ((SR) == I2S_SAMPLERATE_11025) )

/**
  * @}
  */


/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup I2S_Exported_macros I2S Exported Macros
  * @{
  */

/** @brief  Reset I2S handle state
  * @param  \__HANDLE__ I2S handle.
  * @retval None
  */
#define __HAL_I2S_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_I2S_STATE_RESET)

/** @brief  Enable the specified I2S TX peripheral .
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_TX_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->AUDIO_TX_FUNC_EN |= I2S_AUDIO_TX_FUNC_EN_TX_EN)

/** @brief  Disable the specified I2S TX peripheral .
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_TX_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->AUDIO_TX_FUNC_EN &= (~I2S_AUDIO_TX_FUNC_EN_TX_EN))

/** @brief  Enable the specified I2S RX peripheral .
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_RX_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->AUDIO_RX_FUNC_EN |= I2S_AUDIO_RX_FUNC_EN_RX_EN)

/** @brief  Disable the specified I2S RX peripheral .
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_RX_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->AUDIO_RX_FUNC_EN &= (~I2S_AUDIO_RX_FUNC_EN_RX_EN))

#ifndef SF32LB55X
/** @brief  Use the external I2S TX interface.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_TX_INTF_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->AUDIO_TX_FUNC_EN |= I2S_AUDIO_TX_FUNC_EN_TX_INTF_SEL)

/** @brief  Use internal APB TX interface.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_TX_INTF_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->AUDIO_TX_FUNC_EN &= (~I2S_AUDIO_TX_FUNC_EN_TX_INTF_SEL))

/** @brief  Use the external I2S RX interface .
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_RX_INTF_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->AUDIO_RX_FUNC_EN |= I2S_AUDIO_RX_FUNC_EN_RX_INTF_SEL)

/** @brief  Use internal APB RX interface.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_RX_INTF_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->AUDIO_RX_FUNC_EN &= (~I2S_AUDIO_RX_FUNC_EN_RX_INTF_SEL))

#else
#define __HAL_I2S_TX_INTF_ENABLE(__HANDLE__)
#define __HAL_I2S_TX_INTF_DISABLE(__HANDLE__)
#define __HAL_I2S_RX_INTF_ENABLE(__HANDLE__)
#define __HAL_I2S_RX_INTF_DISABLE(__HANDLE__)

#endif

/** @brief  Enable the specified I2S interrupts.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @param  \__INTERRUPT__ specifies the interrupt source to enable or disable.
  *        This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  * @retval None
  */
#define __HAL_I2S_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->INT_MASK |= (__INTERRUPT__))

/** @brief  Disable the specified I2S interrupts.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @param  \__INTERRUPT__ specifies the interrupt source to enable or disable.
  *        This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  * @retval None
  */
#define __HAL_I2S_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->INT_MASK &= (~(__INTERRUPT__)))

/** @brief  Checks if the specified I2S interrupt source is enabled or disabled.
  * @param  \__HANDLE__ specifies the I2S Handle.
  *         This parameter can be I2S where x: 1, 2, or 3 to select the I2S peripheral.
  * @param  \__INTERRUPT__ specifies the I2S interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg I2S_IT_ERR: Error interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_I2S_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->INT_MASK & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Checks whether the specified I2S flag is set or not.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @param  \__FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg I2S_FLAG_RXNE: Receive buffer not empty flag
  *            @arg I2S_FLAG_TXE: Transmit buffer empty flag
  *            @arg I2S_FLAG_UDR: Underrun flag
  *            @arg I2S_FLAG_OVR: Overrun flag
  *            @arg I2S_FLAG_FRE: Frame error flag
  *            @arg I2S_FLAG_CHSIDE: Channel Side flag
  *            @arg I2S_FLAG_BSY: Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_I2S_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->FIFO_STATUS) & (__FLAG__)) == (__FLAG__))

/** @brief Clears the I2S OVR pending flag.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_CLEAR_OVRFLAG(__HANDLE__) (((__HANDLE__)->Instance->INT_STATUS) &= (~I2S_INT_STATUS_RX_FIFO_OVERFLOW))
/** @brief Clears the I2S UDR pending flag.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_CLEAR_UDRFLAG(__HANDLE__) (((__HANDLE__)->Instance->INT_STATUS) &= (~I2S_INT_STATUS_TX_FIFO_UNDERFLOW))

#ifndef SF32LB55X

/** @brief Set the I2S clock from crystal.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_CLK_XTAL(__HANDLE__) (((__HANDLE__)->Instance->DEBUG_LOOP) &= (~I2S_DEBUG_LOOP_SP_CLK_SEL))

/** @brief Set the I2S clock from pll.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __HAL_I2S_CLK_PLL(__HANDLE__) (((__HANDLE__)->Instance->DEBUG_LOOP) |= (I2S_DEBUG_LOOP_SP_CLK_SEL))

/** @brief Set the I2S SP clock divider.
  * @param  \__HANDLE__ specifies the I2S Handle.
  * @param  \__DIV__ clock divider.
  * @retval None
  */
#define __HAL_I2S_SET_SPCLK_DIV(__HANDLE__, __DIV__) \
    do {\
        ((__HANDLE__)->Instance->DEBUG_LOOP) &= (~I2S_DEBUG_LOOP_SP_CLK_DIV);    \
        ((__HANDLE__)->Instance->DEBUG_LOOP) |= ((__DIV__)<<I2S_DEBUG_LOOP_SP_CLK_DIV_Pos);    \
        ((__HANDLE__)->Instance->DEBUG_LOOP) |= (((__DIV__)<<I2S_DEBUG_LOOP_SP_CLK_DIV_Pos) |(I2S_DEBUG_LOOP_SP_CLK_DIV_UPDATE));    \
    }while(0)

#endif
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2S_Exported_Functions
  * @{
  */

/** @addtogroup I2S_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/

/**
  * @brief Initializes the I2S according to the specified parameters
  *         in the I2S_InitTypeDef and create the associated handle.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s);

/**
  * @brief DeInitializes the I2S peripheral
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_DeInit(I2S_HandleTypeDef *hi2s);

/**
  * @brief Initializes the I2S pin setting.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);

/**
  * @brief deunitializes the I2S pin setting.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);
/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
/* Configure I2S parameters  */

/**
  * @brief Set I2S transmit configures.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param cfg: channel, track, volume and others parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Config_Transmit(I2S_HandleTypeDef *hi2s, I2S_CFG_T *cfg);

/**
  * @brief Set I2S Receive configures.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param cfg: channel, track, volume and others parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Config_Receive(I2S_HandleTypeDef *hi2s, I2S_CFG_T *cfg);

/* Blocking mode: Polling */
/**
  * @brief Set I2S Transmit data with polling mode.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: data buffer
  * @param Size: data length need to be transmit
  * @param Timeout: time out threshold if not finish, in ms
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size, uint32_t Timeout);

/**
  * @brief Set I2S receive data with polling mode.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: data buffer
  * @param Size: data length need to be transmit
  * @param Timeout: time out threshold if not finish, in ms
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size, uint32_t Timeout);

/* Non-Blocking mode: Interrupt */

/**
  * @brief Transmit an amount of data in non-blocking mode with Interrupt
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: a 16-bit pointer to data buffer.
  * @param Size: number of data sample to be sent:
  * @note When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *       configuration phase, the Size parameter means the number of 16-bit data length
  *       in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *       the Size parameter means the number of 16-bit data length.
  * @note The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *       between Master and Slave(example: audio streaming).
  * @note This function can use an Audio Frequency up to 48KHz when I2S Clock Source is 32MHz
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size);

/**
  * @brief Receive an amount of data in non-blocking mode with Interrupt
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: a 16-bit pointer to the Receive data buffer.
  * @param Size: number of data sample to be sent:
  * @note When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *       configuration phase, the Size parameter means the number of 16-bit data length
  *       in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *       the Size parameter means the number of 16-bit data length.
  * @note The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *       between Master and Slave(example: audio streaming).
  * @note It is recommended to use DMA for the I2S receiver to avoid de-synchronisation
  * between Master and Slave otherwise the I2S interrupt should be optimized.
  * @note This function can use an Audio Frequency up to 48KHz when I2S Clock Source is 32MHz
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size);

/**
  * @brief  This function handles I2S interrupt request.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);

/* Non-Blocking mode: DMA */
/**
  * @brief Set I2S Transmit data with DMA mode.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: data buffer
  * @param Size: data length need to be transmit
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size);

/**
  * @brief Set I2S Transmit data with DMA mode.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: data buffer
  * @param Size: data length need to be transmit
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size);

/**
  * @brief Pauses the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);

/**
  * @brief Resumes the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);

/**
  * @brief Stops the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);

/**
  * @brief Pauses the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_TX_DMAPause(I2S_HandleTypeDef *hi2s);

/**
  * @brief Resumes the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_TX_DMAResume(I2S_HandleTypeDef *hi2s);

/**
  * @brief Stops the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_TX_DMAStop(I2S_HandleTypeDef *hi2s);

/**
  * @brief Pauses the audio stream recording.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_RX_DMAPause(I2S_HandleTypeDef *hi2s);

/**
  * @brief Resumes the audio stream recording.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_RX_DMAResume(I2S_HandleTypeDef *hi2s);

/**
  * @brief Stops the audio stream recording.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_RX_DMAStop(I2S_HandleTypeDef *hi2s);

/* Callbacks used in non blocking modes (Interrupt and DMA) *******************/

/**
  * @brief Tx Transfer Half completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);

/**
  * @brief Tx Transfer completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);

/**
  * @brief Rx Transfer half completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);

/**
  * @brief Rx Transfer completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);

/**
  * @brief I2S error callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);
/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control and State functions  ************************************/

/**
  * @brief  Return the I2S state
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL state
  */
HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s);

/**
  * @brief  Return the I2S error code
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval I2S Error Code
  */
uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s);


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
