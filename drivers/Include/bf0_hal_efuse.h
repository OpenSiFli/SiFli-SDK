/**
  ******************************************************************************
  * @file   bf0_hal_efuse.h
  * @author Sifli software development team
  * @brief   Header file of EFUSE HAL module.
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

#ifndef __BF0_HAL_EFUSE_H
#define __BF0_HAL_EFUSE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup EFUSE
  * @ingroup BF0_HAL_Driver
  * @{
  */

/** EFUSE bank size in bytes */
#define HAL_EFUSE_BANK_SIZE         32
/** EFUSE bank number */
#define HAL_EFUSE_BANK_NUM          (4)


/**
 * @brief  Init Efuse controller
 * @retval void
 */
HAL_StatusTypeDef HAL_EFUSE_Init(void);

/**
 * @brief  Configure bypass
 * @param enabled true: enable bypass, false: disable bypass
 * @retval void
 */
void HAL_EFUSE_ConfigBypass(bool enabled);

/**
 * @brief  Write data to efuse starting from bit_offset
 * @param bit_offset bit_offset in efuse, must be 32bits aligned, bank0: 0~255, bank1: 256~511
 * @param data point to the data to be written
 * @param size data size in byte, must be multiple of 4bytes and written data cannot cross bank boundary
 * @retval size successfully written
 */
int32_t HAL_EFUSE_Write(uint16_t bit_offset, uint8_t *data, int32_t size);


/**
 * @brief  Read data to efuse starting from bit_offset
 * @param bit_offset bit_offset in efuse, must be 32bits aligned, bank0: 0~255, bank1: 256~511
 * @param data point to buffer to save read data
 * @param size data size in byte, must be multiple of 4bytes and read data cannot cross bank boundary
 * @retval size successfully read
 */
int32_t HAL_EFUSE_Read(uint16_t bit_offset, uint8_t *data, int size);


#ifdef __cplusplus
}
#endif

///@}   EFUSE

#endif /* __BF0_HAL_EFUSE_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
