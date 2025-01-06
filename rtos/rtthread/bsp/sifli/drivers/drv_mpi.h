/**
  ******************************************************************************
  * @file   drv_mpi.h
  * @author Sifli software development team
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

#ifndef __DRV_MPI_H__
#define __DRV_MPI_H__

#include <rtconfig.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief  Set MPI ALIAS
* @param[in]  addr, mpi base address .
* @param[in]  start, alias start address .
* @param[in]  len, alias length .
* @param[in]  offset, alias offset .
* @retval none
*/
void rt_mpi_set_alias(uint32_t addr, uint32_t start, uint32_t len, uint32_t offset);

/**
* @brief  Set MPI NONCE
* @param[in]  addr, mpi base address .
* @param[in]  start, start address .
* @param[in]  end, end address.
* @param[in]  nonce, data.
* @retval none
*/
void rt_mpi_set_ctr_nonce(uint32_t addr, uint32_t start, uint32_t end, uint8_t *nonce);

/**
* @brief  Set MPI AES mode
* @param[in]  addr, mpi base address .
* @param[in]  aes256, aes mode.
* @retval none
*/
void rt_mpi_enable_aes(uint32_t addr, uint8_t aes256);

/**
* @brief  Disable MPI AES
* @param[in]  addr, mpi base address .
* @retval none
*/
void rt_mpi_disable_aes(uint32_t addr);

/**
* @brief  Enable MPI prefetch mode
* @param[in]  addr, prefetch phy address .
* @param[in]  len, prefetch data length.
* @retval 0 if success
*/
int rt_mpi_enable_prefetch(uint32_t addr, uint32_t length);

/**
* @brief  Disable MPI prefetch mode
* @param[in]  addr, prefetch phy address or MPI based address.
* @retval 0 if success
*/
int rt_mpi_disable_prefetch(uint32_t addr);


#ifdef __cplusplus
}
#endif

#endif  /* __DRV_MPI_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
