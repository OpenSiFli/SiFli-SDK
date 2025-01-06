/**
  ******************************************************************************
  * @file   bf0_hal_ezip.h
  * @author Sifli software development team
  * @brief   Header file of EZIP HAL module.
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

#ifndef BF0_HAL_EZIP_H
#define BF0_HAL_EZIP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "bf0_hal_def.h"


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup EZIP EZIP
 * @brief EZIP HAL module driver
 * @{
 */

//#define EZIP_DEBUG

#ifdef SF32LB56X
/** Support EZIP Non-Continuous Addressing Mode */
#define HAL_EZIP_NON_CONT_MODE_SUPPORTED
#endif /* SF32LB56X */



/**
 * @brief  HAL EZIP State structures definition
 */
typedef enum
{
    HAL_EZIP_STATE_RESET             = 0x00U,    /*!< EZIP not yet initialized or disabled       */
    HAL_EZIP_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use   */
    HAL_EZIP_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing             */
    HAL_EZIP_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                              */
    HAL_EZIP_STATE_ERROR             = 0x04U,    /*!< EZIP state error                           */
    HAL_EZIP_STATE_SUSPEND           = 0x05U     /*!< EZIP process is suspended                  */
} HAL_EZIP_StateTypeDef;

/** low 7bit indicates EZIP data type, EZIP, GZIP or LZ4 */
#define HAL_EZIP_MODE_DATA_TYPE_MASK (0x7F)
/** bit7 indicates EZIP data addressing type */
#define HAL_EZIP_MODE_ADDR_TYPE_MASK (0x80)
#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
/** Non-Continuous Addressing Mode */
#define HAL_EZIP_MODE_ADDR_NON_CONT_TYPE      (0x80)
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */
/** Continuous Data Addressing Mode */
#define HAL_EZIP_MODE_ADDR_CONT_TYPE          (0x00)


/** EZIP Work Mode */
typedef enum
{
    HAL_EZIP_MODE_EZIP,   /**< EZIP continuous mode */
    HAL_EZIP_MODE_GZIP,   /**< GZIP mode */
    HAL_EZIP_MODE_LZ4,     /**< LZ4 mode */
    HAL_EZIP_MODE_DATA_TYPE_NUM,
#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
    HAL_EZIP_MODE_EZIP_NON_CONT = HAL_EZIP_MODE_EZIP | HAL_EZIP_MODE_ADDR_NON_CONT_TYPE, /**< EZIP non-continuous mode */
    HAL_EZIP_MODE_GZIP_NON_CONT = HAL_EZIP_MODE_GZIP | HAL_EZIP_MODE_ADDR_NON_CONT_TYPE, /**< GZIP non-continuous mode */
    HAL_EZIP_MODE_LZ4_NON_CONT  = HAL_EZIP_MODE_LZ4  | HAL_EZIP_MODE_ADDR_NON_CONT_TYPE, /**< LZ4 non-continuous mode */
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */

} EZIP_WorkModeTypeDef;

/** EZIP Output Mode */
typedef enum
{
    HAL_EZIP_OUTPUT_EPIC,           /*!< output to EPIC */
    HAL_EZIP_OUTPUT_AHB,            /*!< output to AHB bus*/
} EZIP_OutputModeTypeDef;


#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
/** EZIP Block list type for non-continous mode. */
typedef struct
{
    uint32_t base_addr;             /*!< block base address */
    uint32_t block_size;            /*!< block size in byte */
    uint32_t block_num;             /*!< number of blocks   */
    uint32_t block_offset[0];       /*!< block offset list,
                                     * nor: base_addr+offset is absolute address
                                     * nand: base_addr+offset*block_size is absolute address
                                     */
} EZIP_BlockListTypeDef;
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */


/** EZIP Decoding Configuration */
typedef struct
{
    /** input data buffer, must be 32bit aligned, for non continuous work_mode, format is EZIP_BlockListTypeDef */
    uint8_t *input;
    /** output data buffer, must be 32bit aligned
     *
     * used if #output_mode is #HAL_EZIP_OUTPUT_AHB
     */
    uint8_t *output;
    /** work mode */
    EZIP_WorkModeTypeDef work_mode;
    /** output mode */
    EZIP_OutputModeTypeDef output_mode;
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
    /** ezip data size in bytes, only valid if data is saved in NAND */
    uint32_t input_data_size;
} EZIP_DecodeConfigTypeDef;


#ifdef EZIP_DEBUG
#define EZIP_OP_HIST_DEPTH  (4)
typedef struct
{
    uint8_t idx;
    EZIP_DecodeConfigTypeDef hist[EZIP_OP_HIST_DEPTH];
} EZIP_OpHistTypeDef;
#endif /* EZIP_DEBUG */


struct __FLASH_HandleTypeDef;

typedef struct __FLASH_HandleTypeDef *(*EZIP_FlashHandleQueryCbTypeDef)(uint32_t addr);


/** EZIP handle structure definition */
typedef struct __EZIP_HandleTypeDef
{
    EZIP_TypeDef              *Instance;                                  /*!< EZIP register base address.          */
    void (*CpltCallback)(struct __EZIP_HandleTypeDef *ezip);              /*!< EZIP processing complete callback.   */
    __IO HAL_EZIP_StateTypeDef State;                                     /*!< EZIP state.                          */
    __IO uint32_t              ErrorCode;                                 /*!< EZIP error code.                     */
    void                       *user_data;                                /*!< user_data                            */
    EZIP_FlashHandleQueryCbTypeDef flash_handle_query_cb;                 /*!< flash handle query callback          */

    EZIP_TypeDef               *RamInstance;                                     /*!< EZIP register on RAM. */
    EZIP_TypeDef               *HwInstance;                                      /*!< Backup of EZIP register base address. */
    uint8_t            RamInstance_used;

#ifdef EZIP_DEBUG
    EZIP_OpHistTypeDef         op_hist;
#endif
#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
    uint32_t                   non_cont_block_start_addr;                 /*!< base address of the blocks, internal used
                                                                           * valid if EZIP_NON_CONT_MODE is used
                                                                           */
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */

} EZIP_HandleTypeDef;

#define EZIP_PARA_OUT_EPIC         (0UL)
#define EZIP_PARA_OUT_AHB          (1UL)


#define EZIP_PARA_MOD_EZIP         (0)
#define EZIP_PARA_MOD_GZIP         (1)
#define EZIP_PARA_MOD_LZ4          (2)


#ifndef SF32LB55X

/** @defgroup EZIP_EZIPA_COLOR_TYPE_DEF  EZIPA Color type
 * @{
 */

/** grayscale */
#define EZIP_EZIPA_COLOR_GRAYSCALE        (0)
/** truecolor RGB888 */
#define EZIP_EZIPA_COLOR_RGB888           (2)
/** index-color */
#define EZIP_EZIPA_COLOR_INDEXCOLOR       (3)
/** grayscale with alpha */
#define EZIP_EZIPA_COLOR_GRAYSCALE_ALPHA  (4)
/** truecolor with alpha, ARGB8888 */
#define EZIP_EZIPA_COLOR_ARGB8888         (6)
/** RGB565  */
#define EZIP_EZIPA_COLOR_RGB565           (8)
/** ARGB8565 */
#define EZIP_EZIPA_COLOR_ARGB8565         (12)




/**
 * @}
 */



/** EZIPA header */
typedef struct
{
    /** number of frames */
    uint32_t frame_num;
    /** loop number, 0 for infinite loop */
    uint32_t play_num;
    /** image width in pixel, i.e. width of output buffer */
    uint32_t width;
    /** image height in pixel, i.e. height of output buffer */
    uint32_t height;
    uint8_t bit_depth;
    /** color type,  @ref EZIP_EZIPA_COLOR_TYPE_DEF
     */
    uint8_t color_type;
} EZIP_EZIPAHeaderTypeDef;

/** @defgroup EZIP_EZIPA_DISPOSE_OP_DEF  EZIPA Dispose Op type
 * @{
 */

/** The data in output buffer is left as is and used for next rendering */
#define EZIP_EZIPA_DISPOSE_OP_NONE        (0)
/** The data in output buffer is cleared to fully transparent black before rendering the next frame */
#define EZIP_EZIPA_DISPOSE_OP_BACKGROUND  (1)
/** The data in output buffer is reverted to the previous contents before rendering the next frame */
#define EZIP_EZIPA_DISPOSE_OP_PREVIOUS    (2)
/**
 * @}
 */


/** @defgroup EZIP_EZIPA_BLEND_OP_DEF  EZIPA Blend Op type
 * @{
 */

/** all color components of the frame, including alpha,
 * overwrite the current contents of the frame's output buffer region
 */
#define EZIP_EZIPA_BLEND_OP_SOURCE    (0)
/** Update data in output buffer by blending based on its alpha value, using simple OVER operation
 * as described in the "Alpha Channel Processing" section of the PNG specification [PNG-1.2]
 */
#define EZIP_EZIPA_BLEND_OP_OVER      (1)

/**
 * @}
 */


typedef struct
{
    /** frame sequence number, starting from 0 */
    uint32_t seq_num;
    /** frame width, unit: pixel */
    uint16_t width;
    /** frame height, unit: pixel */
    uint16_t height;
    /** x(column) position at which to render the current frame in output buffer, unit: pixel, starting from 0,
     */
    uint16_t x_offset;
    /** y(row) position at which to render the current frame in output buffer, unit: pixel, starting from 0,
     * relative to the topleft
     */
    uint16_t y_offset;
    /** Frame delay fraction numerator */
    uint16_t delay_num;
    /** Frame delay fraction denominator, 0 means 100
     * frame_delay=delay_num/delay_den, unit is second
     */
    uint16_t delay_den;
    /** disposal oparation type, @ref EZIP_EZIPA_DISPOSE_OP_DEF,
     * specify how the output buffer should be changed at the end of the delay (before rendering the next frame)
     */
    uint8_t dispose_op;
    /** blending operation type, @ref EZIP_EZIPA_BLEND_OP_DEF
     * specify whether the frame is to be alpha blended into the current output buffer content,
     * or whether it should completely replace its region in the output buffer.
     */
    uint8_t blend_op;
} EZIP_EZIPAFrameInfoTypeDef;

#endif /* SF32LB55X */

/**
 * @brief  EZIP Init
 *
 * @param[in]  ezip EZIP handle
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EZIP_Init(EZIP_HandleTypeDef *ezip);

/**
 * @brief  EZIP IRQ Handler
 *
 * @param  ezip EZIP handle
 *
 * @retval None
 */
HAL_StatusTypeDef HAL_EZIP_IRQHandler(EZIP_HandleTypeDef *ezip);

/**
 * @brief  Start decoding in polling mode
 *
 * @param[in]  ezip EZIP handle
 * @param[in]  config decoding configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EZIP_Decode(EZIP_HandleTypeDef *ezip, EZIP_DecodeConfigTypeDef *config);

/**
 * @brief  Start decoding in interrupt mode
 *
 * CpltCallback in handle is used to notify the completion
 *
 * @param[in]  ezip EZIP handle
 * @param[in]  config decoding configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EZIP_Decode_IT(EZIP_HandleTypeDef *ezip, EZIP_DecodeConfigTypeDef *config);

/**
 * @brief  EZIP Deinit
 *
 * @param[in]  ezip EZIP handle
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EZIP_DeInit(EZIP_HandleTypeDef *ezip);

/**
 * @brief  Check if EZIP is idle
 *
 * @param[in]  ezip EZIP handle
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EZIP_CheckReady(EZIP_HandleTypeDef *ezip);


#ifndef SF32LB55X

HAL_StatusTypeDef HAL_EZIP_OpenEZIPA(EZIP_HandleTypeDef *ezip,  uint8_t *data,
                                     EZIP_EZIPAHeaderTypeDef *header,
                                     EZIP_EZIPAFrameInfoTypeDef *frame);

HAL_StatusTypeDef HAL_EZIP_CloseEZIPA(EZIP_HandleTypeDef *ezip);

HAL_StatusTypeDef HAL_EZIP_GetNextFrameInfo(EZIP_HandleTypeDef *ezip,
        EZIP_EZIPAFrameInfoTypeDef *frame);

HAL_StatusTypeDef HAL_EZIP_SuspendEZIPA(EZIP_HandleTypeDef *ezip);

HAL_StatusTypeDef HAL_EZIP_ResumeEZIPA(EZIP_HandleTypeDef *ezip, const uint8_t *data, uint32_t seq_num, uint32_t play_loop_idx);

#endif /* SF32LB55X */

HAL_StatusTypeDef HAL_EZIP_DecodeFast_Init(EZIP_HandleTypeDef *hezip);
HAL_StatusTypeDef HAL_EZIP_DecodeFast_IT(EZIP_HandleTypeDef *hezip);

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* BF0_HAL_EZIP_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
