/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BF0_HAL_EPICTL_H
#define BF0_HAL_EPICTL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_epic.h"

#ifdef HAL_EPICTL_ENABLED

/**
 * @brief  HAL EPICTL State enumeration definition
 */
typedef enum
{
    HAL_EPICTL_STATE_RESET             = 0x00U,    /*!< EPICTL not yet initialized or disabled       */
    HAL_EPICTL_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
    HAL_EPICTL_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
} HAL_EPICTL_StateTypeDef;

/**
 * @brief  EPICTL handle structure definition
 */
typedef struct __EPICTL_HandleTypeDef
{
    EPIC_TypeDef               *Instance;                                        /*!< EPIC register base address. */

    void (* XferCpltCallback)(struct __EPICTL_HandleTypeDef *epictl);                /*!< EPICTL processing complete callback. */
    __IO HAL_EPICTL_StateTypeDef State;                                            /*!< EPICTL state.                        */
    __IO uint32_t              ErrorCode;                                          /*!< EPICTL error code.                   */



    uint32_t   start_tick;
    uint32_t   end_tick;

    void *user_data;                                                             /*!< user data */
    __IO uint32_t PerfCnt;                                                       /*!< EPICTL total running cycle counter */
    __IO uint32_t WaitCnt;
    __IO uint32_t HalCnt;
} EPICTL_HandleTypeDef;

typedef struct
{
    const uint8_t *src;
    uint32_t src_color_mode;      /**< color mode, refer to #EPIC_COLOR_RGB565 */
    uint32_t src_stride;          /**< data stride in BYTES of each line */

    const uint8_t *dst;
    uint32_t dst_color_mode;      /**< color mode, refer to #EPIC_COLOR_RGB565 */
    uint32_t dst_stride;          /**< data stride in BYTES of each line */
    uint32_t dst_compression_rate; /**< compression rate, 0 for uncompressed data */

    uint32_t transfer_width;     /**< transfer width in pixel */
    uint32_t transfer_height;    /**< transfer height in pixel */
} EPICTL_DataType;

//EPICTL init and deinit
void HAL_EPICTL_Init(EPICTL_HandleTypeDef *epictl);
void HAL_EPICTL_DeInit(EPICTL_HandleTypeDef *epictl);

//EPIC Transfer Layer IRQ Handler
void HAL_EPICTL_IRQHandler(EPICTL_HandleTypeDef *epictl);

/**
 * @brief  Initialize an epictl data struct to default vaules
 *
 * @param param  EPICTL data parameter
 * @retval None
 */
void HAL_EPICTL_DataInit(EPICTL_DataType *param);

/**
 * @brief  Start EPIC transfer in interrupt mode
 * This function is used to transfer data from src to dst
 * @param[in]  epic EPICTL handle
 * @param[in]  cfg  transfer configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPICTL_Start_IT(EPICTL_HandleTypeDef *epictl, EPICTL_DataType *cfg);

/**
 * @brief  Start EPIC transfer in polling mode
 * This function is used to transfer data from src to dst
 * @param[in]  epic EPICTL handle
 * @param[in]  cfg  transfer configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPICTL_Start(EPICTL_HandleTypeDef *epictl, EPICTL_DataType *cfg);


#include "sifli_cmpr.h"

/**
 * @brief Calculate the target size of an chunk_pixels with specified compression rate.
 * @retval Target size
 */
uint32_t HAL_EPICTL_CMPR_Target_size(uint32_t chunk_pixels, uint32_t color_mode, uint32_t cmpr_rate);

//Get the bytes of compressed line pixels with specified compression rate
#define HAL_EPICTL_GetCompressedBytes(line_pixels, pixel_bytes, cmpr_rate) CMPR_GetCompressedBytes(line_pixels, pixel_bytes, cmpr_rate)

#define HAL_EPICTL_CMPR_GetConfig(color_mode, p_cfg0, p_cfg1) CMPR_GetConfig(color_mode, p_cfg0, p_cfg1)
#endif /*HAL_EPICTL_ENABLED*/

#ifdef __cplusplus
}
#endif

#endif /* BF0_HAL_EPICTL_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/