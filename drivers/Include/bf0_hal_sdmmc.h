/**
  ******************************************************************************
  * @file   bf0_hal_sdmmc.h
  * @author Sifli software development team
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

#ifndef __BF0_HAL_SDMMC_H
#define __BF0_HAL_SDMMC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup SDMMC SDMMC Low layer(Z0)
  * @{
  */

/* Register address and description define ------------------------------------------------------------*/
/** @defgroup SDMMC_Register_Types SDMMC Register Types
  * @{
  */

#define HAL_SDMMC_DEFAULT_TIMEOUT       (400000)          /*!<   SDMMC default timeout config */

#define HAL_SDMMC_DATA_CARD2HOST        (SD_DCR_R_WN)    /*!<   data direction read */
#define HAL_SDMMC_DATA_HOST2CARD        (0)                 /*!<   data write */

#define HAL_SDMMC_DATA_STREAM_MODE      (SD_DCR_STREAM_MODE)     /*!<   transfer stream mode */
#define HAL_SDMMC_DATA_BLOCK_MODE       (0)                     /*!<   block mode */

#define HAL_SDMMC_WIRE_SINGLE           (0)                     /*!<   1 line mode */
#define HAL_SDMMC_WIRE_QLINE            (1<<SD_DCR_WIRE_MODE_Pos)    /*!<   4 line mode */
#define HAL_SDMMC_WIRE_OLINE            (2<<SD_DCR_WIRE_MODE_Pos)    /*!<   8 line mode */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SDMMC_Exported_Functions
  * @{
  */

/**
  * @brief SDMMC initial.
  * @param hsd sd handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_INIT(SD_TypeDef *hsd);

/**
  * @brief Get sd command response.
  * @param hsd sd handle.
  * @param resp response data buffer.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_GET_RESP(SD_TypeDef *hsd, uint32_t *resp);

/**
  * @brief Set sd timeout.
  * @param hsd sd handle.
  * @param timeout timeout value.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_TIMEOUT(SD_TypeDef *hsd, uint32_t timeout);

/**
  * @brief Set sd transfer data length.
  * @param hsd sd handle.
  * @param len data lenght.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_DATALEN(SD_TypeDef *hsd, uint32_t len);

/**
  * @brief Clear data lenght.
  * @param hsd sd handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_CLR_DATA_CTRL(SD_TypeDef *hsd);

/**
  * @brief Set tranmite configure.
  * @param hsd sd handle.
  * @param blk_size block size.
  * @param direct read or write.
  * @param wire wire mode.
  * @param mode block or stream.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_DATA_CTRL(SD_TypeDef *hsd, uint32_t blk_size,
        uint32_t direct, uint32_t wire, uint32_t mode);

/**
  * @brief Set sd line more.
  * @param hsd sd handle.
  * @param wire 1 line, 4 line or 8 line.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_WIRE(SD_TypeDef *hsd, uint32_t wire);

/**
  * @brief set data read/write mode.
  * @param hsd sd handle.
  * @param direct read or write.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_DIRECT(SD_TypeDef *hsd, uint32_t direct);

/**
  * @brief Set stream mode or block mode.
  * @param hsd sd handle.
  * @param stream stream mode or block mode.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_STREAM(SD_TypeDef *hsd, uint32_t stream);

/**
  * @brief Set transmit block size.
  * @param hsd sd handle.
  * @param blk_size block size, not larger than 2048.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_BLOCK_SIZE(SD_TypeDef *hsd, uint32_t blk_size);

/**
  * @brief Enable data transfer, it should be set before command setting.
  * @param hsd sd handle.
  * @param data_en data transmit enable.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_DATA_EN(SD_TypeDef *hsd, uint32_t data_en);

/**
  * @brief start data transfer.
  * @param hsd sd handle.
  * @param start start or stop.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_DATA_START(SD_TypeDef *hsd, uint32_t start);

/**
  * @brief Set SD IRQ mask.
  * @param hsd sd handle.
  * @param mask irq bit to be mask.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_IRQ_MASK(SD_TypeDef *hsd, uint32_t mask);

/**
  * @brief Get current IRQ setting.
  * @param hsd sd handle.
  * @retval IRQ MASK setting.
  */
uint32_t HAL_SDMMC_GET_IRQ_MASK(SD_TypeDef *hsd);

/**
  * @brief set sd command.
  * @param hsd sd handle.
  * @param cmd_idx command index.
  * @param resp response flag.
  * @param arg argument.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_CMD(SD_TypeDef *hsd, uint32_t cmd_idx,
                                    uint32_t resp, uint32_t arg);

/**
  * @brief Get SDMMC status.
  * @param hsd sd handle.
  * @retval status.
  */
uint32_t HAL_SDMMC_GET_STA(SD_TypeDef *hsd);

/**
  * @brief Clear irq bits.
  * @param hsd sd handle.
  * @param irq irq bits to be clear.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_CLR_INT(SD_TypeDef *hsd, uint32_t irq);

/**
  * @brief Set SD clock.
  * @param hsd sd handle.
  * @param div clock divider.
  * @param en enable clock or not
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_CLK_SET(SD_TypeDef *hsd, uint32_t div, uint8_t en);

/**
  * @brief Set SD power mode.
  * @param hsd sd handle.
  * @param pow power mode.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_POWER_SET(SD_TypeDef *hsd, uint32_t pow);

/**
  * @brief SD write data.
  * @param hsd sd handle.
  * @param buf data buffer.
  * @param size data size.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_WIRTE(SD_TypeDef *hsd, uint32_t *buf, uint32_t size);

/**
  * @brief SD read data.
  * @param hsd sd handle.
  * @param buf data buffer.
  * @param size data size.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_READ(SD_TypeDef *hsd, uint32_t *buf, uint32_t size);

/**
  * @brief get RCI.
  * @param hsd sd handle.
  * @retval rci value
  */
uint32_t HAL_SDMMC_GET_RCI(SD_TypeDef *hsd);

/**
  * @brief Switch sd to normal driver access.
  * @param hsd sd handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SWITCH_NORMAL(SD_TypeDef *hsd);

/**
  * @brief Switch sd to AHB access.
  * @param hsd sd handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SWITCH_AHB(SD_TypeDef *hsd);

/**
  * @brief Configure SD version (size larger than 2GB address will be block based).
  * @param hsd sd handle.
  * @param blk_mode block for read/write param (size larger than 2GB).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SELECT_VERSION(SD_TypeDef *hsd, uint8_t blk_mode);

/**
  * @brief Enable AHB read cache data, or AHB read return dummy.
  * @param hsd sd handle.
  * @param en cache enable or not.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_CACHE_EN(SD_TypeDef *hsd, uint8_t en);

/**
  * @brief Enable AHB read time out recover.
  * @param hsd sd handle.
  * @param en timeout enable or not.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_CACHE_TO_EN(SD_TypeDef *hsd, uint8_t en);

/**
  * @brief Configure MPI3/SD2 AHB space user.
  * @param hsd sd handle.
  * @param sd_map_en map for SD2 if enable, or for MPI3.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_ENABLE_AHB_MAP(SD_TypeDef *hsd, uint8_t sd_map_en);

/**
  * @brief Set cache OFFSET for AHB.
  * @param hsd sd handle.
  * @param offset AHB cache offset, it depend on card version.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_SET_CAOFFSET(SD_TypeDef *hsd, uint32_t offset);

/**
  * @brief Enable void fifo function.
  * @param hsd sd handle.
  * @param en enable void fifo function or not.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDMMC_VOID_FIFO(SD_TypeDef *hsd, uint8_t en);

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

#endif /* __BF0_HAL_SDMMC_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
