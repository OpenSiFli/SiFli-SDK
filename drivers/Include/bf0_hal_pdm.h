/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BF0_PDM_H_
#define BF0_PDM_H_

#ifdef __cplusplus
extern "C" {
#endif


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup PDM PDM
 * @brief Pulse Density Modulation.
 * @{
 */

#if defined(SF32LB55X) || defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB52X)
#define PDM_PLL_CLK_FREQ  (3072000)
#else
#define PDM_PLL_CLK_FREQ  (49152000)
#endif /* SF32LB55X || SF32LB56X || SF32LB58X || SF32LB52X */

#define PDM_XTAL_CLK_FREQ  (9600000)

/**
 * @brief  HAL PDM State structures definition
 */
typedef enum
{
    PDM_STATE_RESET             = 0x00U,    /*!< PDM not yet initialized or disabled       */
    PDM_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
    PDM_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
    PDM_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
    PDM_STATE_ERROR             = 0x04U,    /*!< PDM state error                           */
    PDM_STATE_SUSPEND           = 0x05U,    /*!< PDM process is suspended                  */
    PDM_STATE_BUSY_RX           = 0X06U,
} PDM_StateTypeDef;



/** @defgroup PDM_Error_Code PDM Error Code
 * @{
 */
typedef enum
{

    PDM_ERROR_NONE                = (0x00000000U),     /*!< No error                               */
    PDM_ERROR_OVERFLOW_L          = (0x00000001U),     /*!< Left channerl receive overflow  error  */
    PDM_ERROR_OVERFLOW_R          = (0x00000002U),     /*!< Right channerl receive overflow  error */
    PDM_ERROR_DMA                 = (0x00000003U),     /*!< DMA tansfer  error */
} PDM_ErrorTypeDef;

/**
 * @}
 */



typedef enum
{
    PDM_MODE_ONCE,       /*!< PDM stopped when reached end of buffer       */
    PDM_MODE_LOOP,       /*!< PDM wirte wrap around               */
} PDM_ModeTypeDef;


typedef enum
{
    PDM_CHANNEL_LEFT_ONLY,   /*!<Only Left channel*/
    PDM_CHANNEL_RIGHT_ONLY,  /*!<Only Right channel*/
    PDM_CHANNEL_STEREO,      /*!<Both left&right channel*/
    PDM_CHANNEL_STEREO_SWAP  /*!< Stereo and swap left & right channel             */
} PDM_ChannelTypeDef;


typedef enum
{
    PDM_SAMPLE_8KHZ   = 8000,       /*!<PDM sample rate 8KHz*/
    PDM_SAMPLE_12KHZ  = 12000,      /*!<PDM sample rate 12KHz*/
    PDM_SAMPLE_16KHZ  = 16000,      /*!<PDM sample rate 16KHz*/
    PDM_SAMPLE_24KHZ  = 24000,      /*!<PDM sample rate 24KHz*/
    PDM_SAMPLE_32KHZ  = 32000,      /*!<PDM sample rate 32KHz*/
    PDM_SAMPLE_48KHZ  = 48000,      /*!<PDM sample rate 48KHz*/
    PDM_SAMPLE_96KHZ  = 96000,      /*!<PDM sample rate 96KHz*/
} PDM_SampleRateTypeDef;


typedef enum
{
    PDM_CHANNEL_DEPTH_8BIT  = 8,    /*!<PDM sample channel depth 8 bits*/
    PDM_CHANNEL_DEPTH_16BIT = 16,   /*!<PDM sample channel depth 16bits*/
    PDM_CHANNEL_DEPTH_24BIT = 24,   /*!<PDM sample channel depth 24bits*/
    PDM_CHANNEL_DEPTH_32BIT = 32    /*!<PDM sample channel depth 32bits*/

} PDM_ChannelDepthTypeDef;

typedef enum
{
    PDM_CFG_CHANNEL  = 1,       /*!< Configure PDM channel */
    PDM_CFG_SAMPLERATE = 2,       /*!< Configure PDM Sample Rate*/
    PDM_CFG_DEPTH = 4,           /*!< Configure PDM depth*/
} PDM_ConfigureTypeDef;


typedef struct
{
    uint32_t Mode;     /*!< Specifies the PDM wirte buffer mode.
                                     This parameter can be a value of @ref PDM_ModeTypeDef */

    uint32_t Channels; /*!< Specifies the channels PDM sample.
                                     This parameter can be a value of @ref PDM_ChannelTypeDef */

    uint32_t SampleRate; /*!< Specifies the channel sample rate.
                                     This parameter can be a value of @ref PDM_SampleRateTypeDef */

    uint32_t ChannelDepth; /*!< Specifies each channel sample bits.
                                     This parameter can be a value of @ref PDM_ChannelDepthTypeDef */

    int32_t LeftGain;   /*!< Left channel gain in 0.1db */
    int32_t RightGain;  /*!< Right channel gain in 0.1db */

    int32_t clkSrc;  /*!< clock source, from 3.072 or from  9.6 */
} PDM_InitTypeDef;


typedef struct
{
    PDM_TypeDef *Instance;

    PDM_InitTypeDef Init;                      /*!< PDM Initialize config                    */

    uint8_t                    *pRxBuffPtr;    /*!< Pointer to PDM Rx transfer Buffer        */

    uint16_t                   RxXferSize;     /*!< PDM Rx Transfer size                     */

    DMA_HandleTypeDef          *hdmarx;        /*!< PDM Rx DMA Handle parameters             */
    DMA_HandleTypeDef          *hdmarx_r;      /*!< PDM Rx right channel DMA Handle para     */

    HAL_LockTypeDef            Lock;           /*!< Locking object                           */

    __IO PDM_StateTypeDef State;               /*!< PDM State code                           */

    __IO uint32_t              ErrorCode;      /*!< PDM Error code                           */

} PDM_HandleTypeDef;




#define __HAL_PDM_ENABLE(__HANDLE__)   SET_BIT((__HANDLE__)->Instance->CFG0, PDM_CFG0_PDMCOREEN)
#define __HAL_PDM_DISABLE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->CFG0, PDM_CFG0_PDMCOREEN)

#define __HAL_PDM_ENABLE_IT(__HANDLE__, __INTERRUPT__)    CLEAR_BIT((__HANDLE__)->Instance->INT_MSK, (__INTERRUPT__))
#define __HAL_PDM_DISABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->INT_MSK, (__INTERRUPT__))


/**
 * @brief  Initialize the PDM according to the specified
 *         parameters in the PDM_InitTypeDef and initialize the associated handle.
 * @param  hpdm Pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PDM_Init(PDM_HandleTypeDef *hpdm);

/**
 * @brief  DeInitialize the PDM peripheral.
 * @param  hpdm Pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PDM_DeInit(PDM_HandleTypeDef *hpdm);

/**
 * @brief  Initialize the PDM MSP.
 * @param  hpdm Pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval None
 */
void HAL_PDM_MspInit(PDM_HandleTypeDef *hpdm);

/**
 * @brief  De-Initialize the PDM MSP.
 * @param  hpdm Pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval None
 */
void HAL_PDM_MspDeInit(PDM_HandleTypeDef *hpdm);


/**
 * @brief Set recieve gain
 * @param hpdm - Pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @param channel - PDM_CHANNEL_STEREO or PDM_CHANNEL_STEREO_SWAP will set both left and right channel
 * @param value - [0 ~ 255] gain value.  255 is maximum while 0 is minimum
 * @return
 */
HAL_StatusTypeDef HAL_PDM_Set_Gain(PDM_HandleTypeDef *hpdm, PDM_ChannelTypeDef channel, uint8_t value);

/**
 * @brief  Configure PDM parameters.
 * @param  hpdm Pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @param  type Configure mode
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PDM_Config(PDM_HandleTypeDef *hpdm, PDM_ConfigureTypeDef type);

/**
 * @brief Read PCM data from PDM module by DMA
 * @param hpdm - Pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @param pData - Buffer to save PCM data (Should be aligned to 4 bytes)
 * @param Size - Buffer size in bytes (Should be aligned to 4 bytes)
 * @return HAL status
 */
HAL_StatusTypeDef HAL_PDM_Receive_DMA(PDM_HandleTypeDef *hpdm, uint8_t *pData, uint32_t Size);


/**
 * @brief Stop the DMA Transfer.
 * @param  hpdm pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PDM_DMAStop(PDM_HandleTypeDef *hpdm);

/**
 * @brief General PDM interrupt handler.
 * @param  hpdm pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval None.
 */
void HAL_PDM_IRQHandler(PDM_HandleTypeDef *hpdm);

/**
 * @brief Rx Half Transfer completed callback.
 * @param  hpdm pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval None
 */
void HAL_PDM_RxHalfCpltCallback(PDM_HandleTypeDef *hpdm);

/**
 * @brief Rx Transfer completed callback.
 * @param  hpdm pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval None
 */
void HAL_PDM_RxCpltCallback(PDM_HandleTypeDef *hpdm);

/**
 * @brief PDM error callback.
 * @param  hpdm pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval None
 */
void HAL_PDM_ErrorCallback(PDM_HandleTypeDef *hpdm);

/**
 * @brief  PDM Abort Complete callback.
 * @param  hpdm pointer to a PDM_HandleTypeDef structure that contains
 *               the configuration information for the specified PDM module.
 * @retval None
 */
void HAL_PDM_AbortCpltCallback(PDM_HandleTypeDef *hpdm);


///@} PDM

///@} BF0_HAL_Driver


#ifdef __cplusplus
}
#endif

#endif // BF0_PDM_H_