/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 ******************************************************************************
 * @file   app_bmem.h
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
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

#ifndef APP_BMEM_H
#define APP_BMEM_H

#include <stdbool.h>
#include <rtthread.h>
#include "stdint.h"

/**
 * @brief  The following interfaces provide block memory management functionality,
 *         which can reduce memory fragmentation and improve memory allocation speed.
 *         But the disadvantage is that the block size and number of blocks need to be pre-set,
 *         which reduces memory reusability. This function can be enabled through "USING_BLOCK_MEM" in menuconfig.
 */

#define USING_BMEM_MAGIC

#if 1//def RT_USING_MEMTRACE
    #define USING_BMEM_TRACE
#endif

/**
 * @brief  The memory item on the block memory
 */
struct struct_bmem_t
{
#ifdef USING_BMEM_MAGIC
    uint16_t                 magic;         /**< Magic, means freed por allocated   */
    uint16_t                 size;          /**< Allocated size                     */
#endif
#ifdef USING_BMEM_TRACE
    uint32_t                 ret_addr;      /**< Return addr                        */
#ifdef USING_BMEM_TICK
    uint32_t                 tick;          /**< Tick of allocated memory           */
#endif
#ifdef MEM_ASYN_FREE
    uint16_t                ref_count_magic;
    uint16_t                ref_count;
#endif
#endif
    struct struct_bmem_t    *next;          /**< Next bmem item which had freed     */
    void                    *node;
};

/**
 * @brief  The memory item on the block memory
 */
typedef struct struct_bmem_t bmem_item_t;

/**
 * @brief  Used to store information for each type of block memory.
 */
typedef struct bmem_node
{
    const char              *name;          /**< The name of block memory           */
    uint32_t                 size;          /**< The size of every block memory     */
    uint32_t                 num;           /**< The number of block memory         */
    uint8_t                 *header_ptr;    /**< The header pointer of block memory */
    uint8_t                 *tailer_ptr;    /**< The tailer pointer of block memory */
    bmem_item_t             *free_header;   /**< The free pointer header            */
    int32_t                  act_used_num;  /**< The current acture used number     */
    int32_t                  max_used_num;  /**< The maxium used number             */
}
bmem_node_t;

/**
 * @brief  The memory stucture of BMEM_REGISTER, used define each type of block memory.
 */
typedef struct
{
    const char              *name;          /**< The name of block memory           */
    uint32_t                 size;          /**< The size of every block memory     */
    uint32_t                 num;           /**< The number of block memory         */
    uint8_t                 *ptr;           /**< The pointer of block memory        */
}
bmem_desc_t;


#define BMEM_SECTION_NAME   bmem

#define ALIGN_SIZE_4(x)             ((x) / 4 * 4)
#define BMEM_SIZE_WITH_HEADER(size) (sizeof(bmem_item_t) + ALIGN_SIZE_4(size))
#define BMEM_NUM(total, size)       ALIGN_SIZE_4(total / BMEM_SIZE_WITH_HEADER(size))

#define BMEM_REGISTER_INT(bname, bsize, total_size) \
        static uint8_t CONCAT_2(CONCAT_2(BMEM_SECTION_NAME, _), bsize)[BMEM_NUM(total_size, bsize) * BMEM_SIZE_WITH_HEADER(bsize)] L2_CACHE_RET_BSS_SECT(psram_ret_cache);  \
        SECTION_ITEM_REGISTER(BMEM_SECTION_NAME, static const bmem_desc_t CONCAT_2(CONCAT_2(CONCAT_2(bname, _), bsize), _var)) =     \
        {                                                                                                                            \
            .name  = "bmem_" #bsize,                                                                                                 \
            .size   = ALIGN_SIZE_4(bsize),                                                                                           \
            .num  = BMEM_NUM(total_size, bsize),                                                                                     \
            .ptr = &CONCAT_2(CONCAT_2(bname, _),bsize)[0],                                                                           \
        }

#define  BMEM_REGISTER(bsize, num) BMEM_REGISTER_INT(BMEM_SECTION_NAME, bsize, num * (sizeof(bmem_item_t) +  bsize))

/**
 * @brief  Allocate block_mem from block_memheap, which is registered by BMEM_REGISTER.
           Allocation order: Find the block memory with the closest size, and if not:
           1) when BMEM_ALLOC_ALWAYS configed, find the block memory of the next level
           2) else,return NULL.
 * @param  size Size of the memory to allocate in bytes
 * @retval pointer Pointer of allocated memory.
 */
void    *bmem_alloc(uint32_t size);

/**
 * @brief  This function will release the previously allocated memory block by bmem_alloc.
 *         The released memory block is taken back to block_memheap.
 * @param  p the address of memory which will be released.
 */
int      bmem_free(void *p);

/**
 * @brief  Initialize block memory mutex and load block memheap from BMEM_REGISTER.
 */
int      bmem_init(void);

/**
 * @brief  Check if memory block is block memory.
 * @param  p the address of memory block.
 */
void    *mem_is_bmem(void *p);


#endif /*APP_BMEM_H*/
