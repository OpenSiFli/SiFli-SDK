/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_JPEGD_H
#define __BF0_HAL_JPEGD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup JPEGD JPEG Decoder hardware engine
 * @brief JPEG Decoder hardware engine
 * @{
 */

/**
 * @addtogroup  JPEGD_exported_constants
 * @{
 */
/**
 * @brief  HAL JPEGD State structures definition
 */
typedef enum
{
    HAL_JPEGD_STATE_RESET             = 0x00U,    /*!< JPEGD not yet initialized or disabled       */
    HAL_JPEGD_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use   */
    HAL_JPEGD_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing             */
    HAL_JPEGD_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                              */
    HAL_JPEGD_STATE_ERROR             = 0x04U,    /*!< JPEGD state error                           */
    HAL_JPEGD_STATE_SUSPEND           = 0x05U     /*!< JPEGD process is suspended                  */
} HAL_JPEGD_StateTypeDef;

/** HAL JPEGD Output Mode */
typedef enum
{
    HAL_JPEGD_OUTPUT_EPIC,           /*!< output to EPIC */
    HAL_JPEGD_OUTPUT_AHB,            /*!< output to AHB bus*/
} JPEGD_OutputModeTypeDef;

/**
 * @} JPEGD_exported_constants
 */

/* Exported types ------------------------------------------------------------*/

/** @defgroup JPEGD_Exported_Types JPG Decoder Exported Types
 * @{
 */

struct _JPEGD_HandleTypeDef;
typedef void (*JPEGD_CpltCallback)(struct _JPEGD_HandleTypeDef *hdl, void *param);

/** JPEG Decoding Configuration */
typedef struct _JPEGD_HandleTypeDef
{
    JPEGD_TypeDef               *Instance;                                 /*!< JPEGD register base address.          */
    JPEGD_CpltCallback          CpltCallback;                              /*!< JPEGD processing complete callback.   */
    __IO HAL_JPEGD_StateTypeDef State;                                     /*!< JPEGD state.                          */
    __IO uint32_t               ErrorCode;                                 /*!< JPEGD error code.                     */
    void                        *UserData;                                 /*!< User data                             */
    uint32_t                    RunTime;
    uint32_t                    StartTime;
    uint32_t                    RunNum;

    JPEGD_TypeDef               *RamInstance;                               /*!< JPEGD register on RAM. */
    JPEGD_TypeDef               *HwInstance;                                /*!< Backup of JPEGD register base address. */
    uint8_t                     RamInstance_used;
} JPEGD_HandleTypeDef;


/** JPEGD Decoding Configuration */
typedef struct
{
    /** input data buffer, must be 32bit aligned, format is JPEG*/
    uint8_t *input;
    /** output mode */
    JPEGD_OutputModeTypeDef output_mode;

    /*work buffer for JPEGD hardware.*/
    uint8_t *work_buffer;

    /** output data buffer, must be 32bit aligned
     *
     * used if #output_mode is #HAL_JPEGD_OUTPUT_AHB
     */
    uint8_t *output_y, *output_u, *output_v;
    /** topleft pixel x coordinate of the interested region
     *
     * starting from 0
     */
    uint16_t start_x;
    /** topleft pixel y coordinate of the interested region
     *
     * starting from 0
     */
    uint16_t start_y;
    /** interested region width
     *
     * set to -1 if all columns are needed
     */
    int16_t width;
    /** interested region height
     *
     * set to -1 if all rows are needed
     */
    int16_t height;
    /** JPEGD data size in bytes*/
    uint32_t input_data_size;
} JPEGD_DecodeConfigTypeDef;


/**
 * @}
 */


/**
 * @defgroup JPEGD_exported_functions JPEGD Exported functions
 * @ingroup JPEGD
 * @{
 *
 */

/**
 * @brief  Initialize JPEG Decoder
 * @param  hdl Handle for JPEG decoder
 * @param  cbk Callback functions when decode done
 * @param  user_data user data parameter for callback
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_JPEGD_Init(JPEGD_HandleTypeDef *hdl, JPEGD_CpltCallback cbk, void *user_data);

/**
 * @brief  JPEGD IRQ Handler
 * @param[in]  hdl JPEGD handle
 * @retval None
 */
HAL_StatusTypeDef HAL_JPEGD_IRQHandler(JPEGD_HandleTypeDef *hdl);

/**
 * @brief  Deinitialize JPEG Decoder
 * @param  hdl Handle for JPEG decoder
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_JPEGD_DeInit(JPEGD_HandleTypeDef *hdl);


/**
 * @brief  Start decoding in polling mode
 *
 * @param[in]  hdl JPEGD handle
 * @param[in]  config decoding configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_JPEGD_Decode(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config);

/**
 * @brief  Start decoding in interrupt mode
 *
 * @param[in]  hdl JPEGD handle
 * @param[in]  config decoding configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_JPEGD_Decode_IT(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config);


HAL_StatusTypeDef HAL_JPEGD_DecodeFast_Init(JPEGD_HandleTypeDef *hdl);
HAL_StatusTypeDef HAL_JPEGD_DecodeFast_IT(JPEGD_HandleTypeDef *hdl);

/**
 * @brief  Check if JPEGD is idle
 *
 * @param[in]  hdl JPEGD handle
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_JPEGD_CheckReady(JPEGD_HandleTypeDef *hdl);


/**
 * @brief  Get JPEGD work buffer size
 *
 * @param[in]  hdl JPEGD handle
 * @param[in]  config decoding configuration
 *
 * @retval HAL status
 */
int HAL_JPEGD_GetBufferSize(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config);

/**
 * @brief  Get JPEGD output buffer size
 *
 * @param[in]  hdl JPEGD handle
 * @param[in]  config decoding configuration
 * @param[out]  y_size output y size
 * @param[out]  u_size output u size
 * @param[out]  v_size output v size
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_JPEGD_GetOutputSize(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config, int *y_size, int *u_size, int *v_size);


/**
 * @brief  Get JPEGD dimension
 *
 * @param[in]  input JPEG buffer
 * @param[in]  size size of JPEG buffer
 * @param[out]  width width of JPEG
 * @param[out]  height height of JPEG
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_JPEGD_GetDim(uint8_t *input, uint32_t size, int *width, int *height);

/**
 *@} JPEGD_exported_functions
 */

/**
 *@} JPEGD
 */

/**
 *@} BF0_HAL_Driver
 */

#ifdef __cplusplus
}
#endif

#endif

/// @}  file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
