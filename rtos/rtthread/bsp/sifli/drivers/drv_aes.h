/**
  ******************************************************************************
  * @file   drv_aes.h
  * @author Sifli software development team
  * @brief AES BSP driver
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

#ifndef __DRV_AES_H_
#define __DRV_AES_H_

#include <rtthread.h>
#include <board.h>
#include "bf0_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_aes AES
  * @brief AES BSP driver
  * @{
  */

typedef struct
{
    uint32_t *key;
    int key_size; //16:AES_KEY_128,  24: AES_KEY_192,  32:AES_KEY_LEN_256.
    uint32_t *iv;
    uint32_t mode; //0:ECB  1:CTR  2:CBC
} AES_KeyTypeDef;

typedef struct
{
    uint8_t *in_data;
    uint8_t *out_data;
    uint32_t size;
} AES_IOTypeDef;


/**
 * @brief encode/decode data in interrupt mode.
 * @param cfg - configuration of enc/dec key , inital vector, and mode.
 * @param data - input&output data
 * @param cb - finish callback
 * @return RT_EOK if successful
 */
rt_err_t drv_aes_enc_async(AES_KeyTypeDef *cfg, AES_IOTypeDef *data, pAESCallback cb);
rt_err_t drv_aes_dec_async(AES_KeyTypeDef *cfg, AES_IOTypeDef *data, pAESCallback cb);

/**
 * @brief encode/decode data in CPU polling mode.
 * @param cfg - configuration of enc/dec key , inital vector, and mode.
 * @param data - input&output data
 * @param cb - finish callback
 * @return RT_EOK if successful
 */
rt_err_t drv_aes_enc_sync(AES_KeyTypeDef *cfg, AES_IOTypeDef *data);
rt_err_t drv_aes_dec_sync(AES_KeyTypeDef *cfg, AES_IOTypeDef *data);

/**
 * @brief copy data like DMA in interrupt mode.
 * @param data - input&output data
 * @param cb - finish callback
 * @return RT_EOK if successful
 */
rt_err_t drv_aes_copy_async(AES_IOTypeDef *data, pAESCallback cb);

/// @} drv_aes
/// @} bsp_driver

#ifdef __cplusplus
}
#endif

#endif /*__DRV_AES_H_ */

/// @} file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
