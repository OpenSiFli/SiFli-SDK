/**
  ******************************************************************************
  * @file   mem_pool_mng.h
  * @author Sifli software development team
  * @brief Memory Pool Manager
  * @{
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

#ifndef MEM_POOL_MNG_H
#define MEM_POOL_MNG_H
#include "rtthread.h"
#include <stdint.h>
#include <stdbool.h>
#include "section.h"

/**
 ****************************************************************************************
* @addtogroup mem_pool_mng Memory Pool Manager
* @ingroup middleware
* @brief Memory Pool Manager
* @{
****************************************************************************************
*/


#ifdef __cplusplus
extern "C" {
#endif

#ifndef MEM_POOL_L1_NON_RET_SIZE
#define MEM_POOL_L1_NON_RET_SIZE 0
#endif

#ifndef MEM_POOL_L2_NON_RET_SIZE
#define MEM_POOL_L2_NON_RET_SIZE 0
#endif
#ifndef MEM_POOL_L2_RET_SIZE
#define MEM_POOL_L2_RET_SIZE 0
#endif
#ifndef MEM_POOL_L2_CACHE_NON_RET_SIZE
#define MEM_POOL_L2_CACHE_NON_RET_SIZE 0
#endif
#ifndef MEM_POOL_L2_CACHE_RET_SIZE
#define MEM_POOL_L2_CACHE_RET_SIZE 0
#endif




/** memory pool id
 *
 * Fallback policy:
 *    L1_NON_RET -> L1_RET
 *    L2_CACHE_RET -> L2_RET -> L1_RET
 *    L2_CACHE_NON_RET -> L2_NON_RET -> L1_NON_RET
 *
 */
typedef enum
{
    MEM_POOL_L1_NON_RET = 0,
    MEM_POOL_L1_RET     = 1,
    MEM_POOL_CACHE_L1_NON_RET = 2,
    MEM_POOL_CACHE_L1_RET = 3,
    MEM_POOL_L2_NON_RET = 4,
    MEM_POOL_L2_RET     = 5,
    MEM_POOL_L2_CACHE_NON_RET = 6,
    MEM_POOL_L2_CACHE_RET     = 7,
    MEM_POOL_NUM
} mem_pool_id_t;


typedef struct
{
    void         *start_addr;                 /**< pool start address and size */
    uint32_t     pool_size;                  /**< pool size */
    uint32_t     available_size;             /**< available size */
    uint32_t     max_used_size;              /**< maximum allocated size */
} mem_pool_info_t;

void *mem_pool_alloc(mem_pool_id_t pool_id, size_t size);

void *mem_pool_realloc(void *p, size_t new_size);

void mem_pool_free(void *p);

void *mem_pool_calloc(mem_pool_id_t pool_id, size_t count, size_t size);

bool mem_pool_get_info(mem_pool_id_t pool_id, mem_pool_info_t *info);


/// @}  mem_pool_mng

#ifdef __cplusplus
}
#endif




/// @} file
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
