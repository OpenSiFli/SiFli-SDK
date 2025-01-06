/**
  ******************************************************************************
  * @file   drv_ext_dma.h
  * @author Sifli software development team
  * @brief EXT_DMA BSP driver
  * @{
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

#ifndef __DRV_EXT_DMA_H_
#define __DRV_EXT_DMA_H_

#include <rtthread.h>
#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_ext_dma EXT_DMA
  * @brief EXT_DMA BSP driver
  * @{
  */

typedef void (* pCallback)(void);

typedef enum
{
    EXT_DMA_XFER_CPLT_CB_ID          = 0x00U,    /*!< Full transfer     */
    EXT_DMA_XFER_ERROR_CB_ID         = 0x01U,    /*!< Error             */
} EXT_DMA_CallbackIDTypeDef;


typedef struct
{
    bool cmpr_en;
    uint8_t cmpr_rate;
    uint16_t col_num;
    uint16_t row_num;
    uint32_t src_format;
} EXT_DMA_CmprTypeDef;

/**
 * @brief Configure ext_dma with source, dest address auto increase mode.
 * @param[in] src_inc: Source address auto increase
 * @param[in] dst_inc: Dest address auto increase
 * @return configure result, 0 success
 */
rt_err_t EXT_DMA_Config(uint8_t src_inc, uint8_t dst_inc);
/**
 * @brief Configure ext_dma with source, dest address auto increase mode.
 * @param[in] src_inc Source address auto increase
 * @param[in] dst_inc Dest address auto increase
 * @param[in] cmpr compression configuration
 * @return configure result, 0 success
 */
rt_err_t EXT_DMA_ConfigCmpr(uint8_t src_inc, uint8_t dst_inc, const EXT_DMA_CmprTypeDef *cmpr);


rt_err_t EXT_DMA_START_ASYNC(uint32_t src, uint32_t dst, uint32_t len);
void EXT_DMA_Register_Callback(EXT_DMA_CallbackIDTypeDef cid, pCallback cb);

/**
 * @brief Get ext-dma error code
 * @return error code
 */
uint32_t EXT_DMA_GetError(void);

rt_err_t EXT_DMA_TRANS_SYNC(uint32_t src, uint32_t dst, uint32_t len, uint32_t timeout);

void EXT_DMA_Wait_ASYNC_Done(void);

/// @} drv_dma
/// @} bsp_driver

#ifdef __cplusplus
}
#endif

#endif /*__DRV_DMA_H_ */

/// @} file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
