/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 ******************************************************************************
 * @file   app_bmem.c
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

/*********************
 *      INCLUDES
 *********************/
#include <string.h>
#include "app_bmem.h"
#include "mem_section.h"

#define BMEM_ALLOC_ALWAYS

enum
{
    BMEM_NULL,
    BMEM_NORMAL,
};

typedef struct
{
    uint16_t            num;
    uint16_t            statue;
    bmem_node_t        *node_list;
    uint32_t            max_size;
} bmem_header_t;

#define MAGIC_FREE      0x1ea0
#define MAGIC_ALLOC     0x1ea1

#define BMEM_HEAD_SIZE              sizeof(bmem_item_t)
#define BMEM_VALID(p, bmem_node)    ((uint8_t *)p >= bmem_node->header_ptr && (uint8_t *)p < bmem_node->tailer_ptr)

static bmem_header_t    bmem_list_header;

void list_bmem(void)
{
#ifdef USING_BLOCK_MEM
    bmem_node_t *bmem_node = bmem_list_header.node_list;
    for (int i = 0; i < bmem_list_header.num; i++)
    {
        rt_kprintf("bmem: %s, num %d max %d actual %d\n", bmem_node->name, bmem_node->num, bmem_node->max_used_num, bmem_node->act_used_num);
        bmem_node++;
    }
#endif
}

#define FREE_LIST_NOT_EMPTY(bmem_node) (bmem_node->free_header)

static struct rt_mutex  bmem_mutex;

void *bmem_alloc(uint32_t size)
{
    if (BMEM_NULL == bmem_list_header.statue || size > bmem_list_header.max_size) return NULL;

    bmem_item_t *ptr = NULL;
    bmem_node_t *bmem_node = bmem_list_header.node_list;

    rt_mutex_take(&bmem_mutex, RT_WAITING_FOREVER);

    for (uint16_t i = 0; i < bmem_list_header.num; i++)
    {
        if (size < bmem_node->size)
        {
            //RT_ASSERT(bmem_node->name);
            if (FREE_LIST_NOT_EMPTY(bmem_node)) goto end;
#ifndef BMEM_ALLOC_ALWAYS
            //allocated fail, because the block memory with the closest size is full.
            break;
#endif
        }
        bmem_node++;
    }

    rt_mutex_release(&bmem_mutex);
    return NULL;

end:

    ptr = bmem_node->free_header;
#ifdef USING_BMEM_MAGIC
    RT_ASSERT(ptr->magic == MAGIC_FREE);
    ptr->magic = MAGIC_ALLOC;
    ptr->size = size;
#endif

    bmem_node->free_header = bmem_node->free_header->next;
    bmem_node->act_used_num++;
    if (bmem_node->max_used_num < bmem_node->act_used_num) bmem_node->max_used_num = bmem_node->act_used_num;

#ifdef USING_BMEM_TICK
    ptr->tick = (uint32_t) rt_system_get_time();
#else
    //reuse next for tick.
    ptr->next = (bmem_item_t *) rt_system_get_time();
#endif
#ifdef MEM_ASYN_FREE
    ptr->ref_count_magic = REF_COUNT_MAGIC;
    ptr->ref_count = 0;
#endif
    ptr->node = (void *) bmem_node;

    ptr = (bmem_item_t *)((uint8_t *)ptr + BMEM_HEAD_SIZE);

    rt_mutex_release(&bmem_mutex);
    return (void *)ptr;
}

int bmem_free(void *p)
{
    if (!p || BMEM_NULL == bmem_list_header.statue) return -1;

    bmem_node_t *bmem_node = mem_is_bmem(p);
    if (!bmem_node)
    {
        return -1;  /* not in bmem. */
    }

    bmem_item_t *bmem = (bmem_item_t *)((uint8_t *) p - BMEM_HEAD_SIZE);
#ifdef MEM_ASYN_FREE
    if (bmem->ref_count)
    {
        RT_ASSERT(REF_COUNT_MAGIC == bmem->ref_count_magic);
        extern void app_mem_insert_asyn_node(void *ptr, void (*)(void *));
        app_mem_insert_asyn_node(p, (void (*)(void *)) bmem_free);
        return 0;
    }
#endif

    rt_mutex_take(&bmem_mutex, RT_WAITING_FOREVER);
#ifdef USING_BMEM_MAGIC
    bmem_item_t *next = (bmem_item_t *)((uint8_t *) p + bmem_node->size);
    RT_ASSERT(MAGIC_ALLOC == bmem->magic &&
              bmem->size <= bmem_node->size);
    if (BMEM_VALID(next, bmem_node))
        RT_ASSERT(MAGIC_ALLOC == next->magic || MAGIC_FREE == next->magic);
    bmem->magic = MAGIC_FREE;
#endif
    bmem->next = bmem_node->free_header;
    bmem_node->free_header = bmem;
    bmem_node->act_used_num--;
    RT_ASSERT(bmem_node->act_used_num >= 0);
    rt_mutex_release(&bmem_mutex);

    return 0;
}

SECTION_DEF(BMEM_SECTION_NAME, bmem_desc_t);

/**
 * @brief  Load block memheap from BMEM_REGISTER.
 */
static void bmem_load(void)
{
    bmem_desc_t *block_desc;
    uint32_t    *end;
    uint32_t    *temp;
    uint32_t     num = 0;
    bmem_node_t *bmem_node;

    end  = (uint32_t *)SECTION_END_ADDR(BMEM_SECTION_NAME);
    temp = (uint32_t *)SECTION_START_ADDR(BMEM_SECTION_NAME);

    //rt_mutex_take(&bmem_mutex, RT_WAITING_FOREVER);
    bmem_list_header.node_list = NULL;
    bmem_list_header.max_size = 0;
    bmem_list_header.statue = BMEM_NORMAL;

    /* Get number of the block_memory from BMEM_REGISTER */
    while (temp < end)
    {
        block_desc = (bmem_desc_t *)temp;

        if (block_desc->size > 0 && block_desc->name && block_desc->num > 0)
        {
            temp += (sizeof(bmem_desc_t) >> 2);
            num++;
        }
        else
        {
            temp++;
        }
    }

    if (0 == num) return;

    temp = (uint32_t *)SECTION_START_ADDR(BMEM_SECTION_NAME);
    bmem_node = (bmem_node_t *) rt_calloc(1, sizeof(bmem_node_t) * num);
    RT_ASSERT(bmem_node);
    bmem_list_header.num = num;
    bmem_list_header.node_list = bmem_node;

    num = 0;
    while (temp < end)
    {
        block_desc = (bmem_desc_t *)temp;

        if (block_desc->size > 0 && block_desc->name && block_desc->num > 0)
        {
            uint32_t i;
            for (i = 0; i < num; i++)
            {
                if (block_desc->size < bmem_list_header.node_list[i].size) break;
            }

            bmem_node = &bmem_list_header.node_list[i];

            for (int k = num - 1; k >= (int) i; k--)
            {
                bmem_list_header.node_list[k + 1] = bmem_list_header.node_list[k];
            }

            RT_ASSERT(bmem_node);
            bmem_node->size = block_desc->size;
            bmem_node->name = block_desc->name;
            bmem_node->num = block_desc->num;
            bmem_node->header_ptr = block_desc->ptr;
            bmem_node->tailer_ptr = bmem_node->header_ptr + block_desc->num * (bmem_node->size + BMEM_HEAD_SIZE);
            bmem_node->act_used_num = 0;
            bmem_node->free_header = NULL;
            num++;

            for (i = block_desc->num; i > 0; i--)
            {
                bmem_item_t *bmem = (bmem_item_t *)(bmem_node->header_ptr + (i - 1) * (bmem_node->size + BMEM_HEAD_SIZE));
#ifdef USING_BMEM_MAGIC
                bmem->magic = MAGIC_FREE;
                bmem->size = bmem_node->size;
#endif
#ifdef USING_BMEM_TRACE
#ifdef USING_BMEM_TICK
                ((bmem_item_t *) bmem)->tick = 0;
#endif
                ((bmem_item_t *) bmem)->ret_addr = 0;
#endif
                bmem->next = bmem_node->free_header;
                bmem_node->free_header = (bmem_item_t *)bmem;
            }

            if (bmem_list_header.max_size < bmem_node->size)
            {
                bmem_list_header.max_size = bmem_node->size;
            }

            rt_kprintf("%s: %s, num %d\n", __func__, bmem_node->name, bmem_node->num);
            temp += (sizeof(bmem_desc_t) >> 2);
        }
        else
        {
            temp++;
        }

    }

    //rt_mutex_release(&bmem_mutex);
}

/**
 * @brief  Unload block memheap.
 */
static void bmem_unload(void)
{
    rt_mutex_take(&bmem_mutex, RT_WAITING_FOREVER);
    rt_free(bmem_list_header.node_list);
    memset(&bmem_list_header, 0x00, sizeof(bmem_list_header));
    rt_mutex_release(&bmem_mutex);
}

uint32_t bmem_backup(uint32_t (*func)(uint32_t addr, uint32_t size))
{
    uint32_t ret = 0;

    RT_ASSERT(func);

    rt_mutex_take(&bmem_mutex, RT_WAITING_FOREVER);
    bmem_node_t *bmem_node;
    for (uint16_t i = 0; i < bmem_list_header.num; i++)
    {
        bmem_node = &bmem_list_header.node_list[i];
        RT_ASSERT(bmem_node->name);
        uint32_t num = 0;
        if (0 == bmem_node->act_used_num)
        {
            continue;
        }

        uint32_t size = bmem_node->size + BMEM_HEAD_SIZE;
        uint32_t used_num = 0;

        for (uint32_t i = 0; i < bmem_node->num; i++)
        {
            bmem_item_t *bmem = (bmem_item_t *)(bmem_node->header_ptr + i * size);
#ifdef USING_BMEM_MAGIC
            if (MAGIC_ALLOC == bmem->magic)
#else
            if (1)//NEXT_ALLOC == bmem->next)
#endif
            {
                ret = func((uint32_t) bmem, size);
                if (ret) break;
            }

        }
    }
    rt_mutex_release(&bmem_mutex);
    return ret;
}

/**
 * @brief  Initialize block memory mutex and load block memheap from BMEM_REGISTER.
 */
int bmem_init(void)
{
    rt_kprintf("%s\n", __func__);
    rt_mutex_init(&bmem_mutex, "app_bmem", RT_IPC_FLAG_FIFO);
    bmem_load();
    return 0;
}

/**
 * @brief  Check if memory block is block memory.
 * @param  p the address of memory block.
 */
void *mem_is_bmem(void *p)
{
#ifdef USING_BMEM_MAGIC
    bmem_item_t *bmem = (bmem_item_t *) p - 1;
    if (MAGIC_ALLOC == bmem->magic
#ifdef MEM_ASYN_FREE
            && REF_COUNT_MAGIC == bmem->ref_count_magic
#endif
            && 'b' == (((bmem_node_t *) bmem->node)->name)[0]
            && 'm' == (((bmem_node_t *) bmem->node)->name)[1])
    {
        return bmem->node;
    }
#else
    bmem_node_t *bmem_node = bmem_list_header.node_list;
    for (uint16_t i = 0; i < bmem_list_header.num; i++)
    {
        //RT_ASSERT(bmem_node->name);
        if (BMEM_VALID(p, bmem_node)) return bmem_node;
        bmem_node++;
    }
#endif
    return NULL;
}

