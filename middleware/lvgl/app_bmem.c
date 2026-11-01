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
    uint16_t            statue;
    bmem_node_t         node;
    uint32_t            max_size;
} bmem_header_t;

#define MAGIC_FREE      0x1eb0
#define MAGIC_ALLOC     0x1eb1

#define BMEM_HEAD_SIZE              sizeof(bmem_item_t)
#define BMEM_FOOT_SIZE              sizeof(bmem_footer_t)
#define BMEM_VALID(p, bmem_node)    ((uint8_t *)p >= bmem_node->header_ptr && (uint8_t *)p < bmem_node->tailer_ptr)

static bmem_header_t    bmem_list_header;

static inline void bmem_check_run_footer(bmem_node_t *bmem_node, bmem_item_t *bmem, uint16_t units, uint16_t magic);

#define bmem_run_bin_index(units, bin_num) (units > bin_num ? (bin_num - 1) : (units - 1))

static inline uint16_t bmem_calc_run_bin_num(uint32_t unit_size)
{
    uint32_t max_units;
    uint32_t bin_num;

    RT_ASSERT(unit_size > 0);

    max_units = (BMEM_HEAD_SIZE + BMEM_MAX_ALLOC_SIZE + BMEM_FOOT_SIZE + unit_size - 1) / unit_size;
    bin_num = max_units + 1;
    if (bin_num > BMEM_RUN_BIN_NUM)
    {
        bin_num = BMEM_RUN_BIN_NUM;
    }

    return (uint16_t)bin_num;
}

void list_bmem(void)
{
#ifdef USING_BLOCK_MEM
    bmem_node_t *bmem_node = &bmem_list_header.node;

    if (BMEM_NULL == bmem_list_header.statue)
    {
        return;
    }

    rt_kprintf("bmem: %s, unit_size %d total_units %d actual_used_units %d max_used_units %d\n",
               bmem_node->name, bmem_node->size, bmem_node->num, bmem_node->act_used_num, bmem_node->max_used_num);
#endif
}

void list_bmem_detail(void)
{
#ifdef USING_BLOCK_MEM
    bmem_node_t *bmem_node = &bmem_list_header.node;

    if (BMEM_NULL == bmem_list_header.statue)
    {
        return;
    }

    list_bmem();
#ifdef USING_BMEM_MAGIC
    uint16_t bin_num = bmem_node->bin_num;
    uint32_t alloc_runs[BMEM_RUN_BIN_NUM] = {0};
    uint32_t alloc_units[BMEM_RUN_BIN_NUM] = {0};
    uint32_t free_runs[BMEM_RUN_BIN_NUM] = {0};
    uint32_t free_units[BMEM_RUN_BIN_NUM] = {0};

    for (uint32_t unit_index = 0; unit_index < bmem_node->num;)
    {
        bmem_item_t *bmem = (bmem_item_t *)(bmem_node->header_ptr + unit_index * bmem_node->size);
        uint16_t units;
        uint16_t bin;

        RT_ASSERT(MAGIC_ALLOC == bmem->magic || MAGIC_FREE == bmem->magic);
        units = bmem->units;
        RT_ASSERT(units > 0 && unit_index + units <= bmem_node->num);
        bmem_check_run_footer(bmem_node, bmem, units, bmem->magic);

        if (MAGIC_ALLOC == bmem->magic)
        {
            uint16_t alloc_units_by_size = (uint16_t)((bmem->size + bmem_node->size - 1) / bmem_node->size);

            RT_ASSERT(alloc_units_by_size > 0);
            bin = bmem_run_bin_index(alloc_units_by_size, bin_num);
            alloc_runs[bin]++;
            alloc_units[bin] += units;
        }
        else
        {
            bin = bmem_run_bin_index(units, bin_num);
            free_runs[bin]++;
            free_units[bin] += units;
        }

        unit_index += units;
    }

    rt_kprintf("%14s %10s %10s\n", "alloc_size", "count", "used_unit");
    for (uint16_t bin = 0; bin < bin_num; bin++)
    {
        if (alloc_runs[bin])
        {
            uint16_t units = bin + 1;

            if (bin_num - 1 == bin)
            {
                rt_kprintf("%13s%d %10d %10d\n", ">=", bmem_node->size * bin_num,
                           alloc_runs[bin], alloc_units[bin]);
            }
            else
            {
                rt_kprintf("%14d %10d %10d\n", units * bmem_node->size,
                           alloc_runs[bin], alloc_units[bin]);
            }
        }
    }

    rt_kprintf("%14s %10s %10s\n", "free_run_size", "count", "free_unit");
    for (uint16_t bin = 0; bin < bin_num; bin++)
    {
        if (free_runs[bin])
        {
            uint16_t units = bin + 1;

            if (bin_num - 1 == bin)
            {
                rt_kprintf("%13s%d %10d %10d\n", ">=", bmem_node->size * bin_num,
                           free_runs[bin], free_units[bin]);
            }
            else
            {
                rt_kprintf("%14d %10d %10d\n", units * bmem_node->size,
                           free_runs[bin], free_units[bin]);
            }
        }
    }
#endif
#endif
}

#ifdef RT_USING_FINSH
    #include <finsh.h>
    MSH_CMD_EXPORT_ALIAS(list_bmem_detail, list_bmem, list_bmem);
#endif

static struct rt_mutex  bmem_mutex;

static inline uint16_t bmem_run_units(bmem_node_t *bmem_node, uint32_t size)
{
    return (uint16_t)((BMEM_HEAD_SIZE + size + BMEM_FOOT_SIZE + bmem_node->size - 1) / bmem_node->size);
}

static inline uint8_t *bmem_run_end(bmem_node_t *bmem_node, bmem_item_t *bmem, uint16_t units)
{
    return (uint8_t *)bmem + units * bmem_node->size;
}

static inline bmem_footer_t *bmem_run_footer(bmem_node_t *bmem_node, bmem_item_t *bmem, uint16_t units)
{
    return (bmem_footer_t *)(bmem_run_end(bmem_node, bmem, units) - BMEM_FOOT_SIZE);
}

static inline void bmem_set_run_footer(bmem_node_t *bmem_node, bmem_item_t *bmem, uint16_t units, uint16_t magic)
{
    bmem_footer_t *footer = bmem_run_footer(bmem_node, bmem, units);

    footer->magic = magic;
    footer->units = units;
}

static inline void bmem_check_run_footer(bmem_node_t *bmem_node, bmem_item_t *bmem, uint16_t units, uint16_t magic)
{
    bmem_footer_t *footer = bmem_run_footer(bmem_node, bmem, units);

    RT_ASSERT(footer->magic == magic && footer->units == units);
}

static inline void bmem_insert_run(bmem_node_t *bmem_node, bmem_item_t *bmem, uint16_t units)
{
    uint16_t bin = bmem_run_bin_index(units, bmem_node->bin_num);

#ifdef USING_BMEM_MAGIC
    bmem->magic = MAGIC_FREE;
    bmem->size = 0;
    bmem->units = units;
    bmem_set_run_footer(bmem_node, bmem, units, MAGIC_FREE);
#endif
    bmem->node = (void *)bmem_node;
    bmem->next = bmem_node->free_bins[bin];
    bmem_node->free_bins[bin] = bmem;
    bmem_node->free_header = bmem;
    bmem_node->free_bin_map |= (1U << bin);
}

static inline void bmem_remove_run_from_bin(bmem_node_t *bmem_node, uint16_t bin, bmem_item_t *prev, bmem_item_t *bmem)
{
    if (prev)
    {
        prev->next = bmem->next;
    }
    else
    {
        bmem_node->free_bins[bin] = bmem->next;
    }

    if (!bmem_node->free_bins[bin])
    {
        bmem_node->free_bin_map &= ~(1U << bin);
    }

    bmem_node->free_header = NULL;
    for (uint16_t i = 0; i < bmem_node->bin_num; i++)
    {
        if (bmem_node->free_bins[i])
        {
            bmem_node->free_header = bmem_node->free_bins[i];
            break;
        }
    }
}

static bmem_item_t *bmem_find_run(bmem_node_t *bmem_node, uint16_t units, uint16_t *bin_out, bmem_item_t **prev_out)
{
    uint16_t start_bin = bmem_run_bin_index(units, bmem_node->bin_num);
    uint32_t map = bmem_node->free_bin_map & (~0U << start_bin);

    while (map)
    {
        uint16_t bin = 0;
        uint32_t bit = map & (~map + 1U);

        while ((bit >> bin) != 1U)
        {
            bin++;
        }

        bmem_item_t *prev = NULL;
        bmem_item_t *bmem = bmem_node->free_bins[bin];
        while (bmem)
        {
            if (bmem->units >= units)
            {
                *bin_out = bin;
                *prev_out = prev;
                return bmem;
            }

            prev = bmem;
            bmem = bmem->next;
        }

        map &= ~bit;
    }

    return NULL;
}

static void bmem_remove_run(bmem_node_t *bmem_node, bmem_item_t *bmem)
{
    uint16_t bin = bmem_run_bin_index(bmem->units, bmem_node->bin_num);
    bmem_item_t *prev = NULL;
    bmem_item_t *free_run = bmem_node->free_bins[bin];

    while (free_run)
    {
        if (free_run == bmem)
        {
            bmem_remove_run_from_bin(bmem_node, bin, prev, free_run);
            return;
        }

        prev = free_run;
        free_run = free_run->next;
    }

    RT_ASSERT(0);
}

static void bmem_merge_and_insert_run(bmem_node_t *bmem_node, bmem_item_t *bmem, uint16_t units)
{
    uint8_t *run_start = (uint8_t *)bmem;
    uint8_t *run_end = bmem_run_end(bmem_node, bmem, units);

    if (run_start > bmem_node->header_ptr)
    {
        bmem_footer_t *left_footer = (bmem_footer_t *)(run_start - BMEM_FOOT_SIZE);

        RT_ASSERT(MAGIC_FREE == left_footer->magic || MAGIC_ALLOC == left_footer->magic);
        if (MAGIC_FREE == left_footer->magic && left_footer->units > 0)
        {
            uint16_t left_units = left_footer->units;
            uint32_t left_size = left_units * bmem_node->size;

            if (left_size <= (uint32_t)(run_start - bmem_node->header_ptr))
            {
                uint8_t *left_start = run_start - left_size;
                bmem_item_t *left = (bmem_item_t *)left_start;

                if (bmem_run_end(bmem_node, left, left_units) == run_start &&
                        left->magic == MAGIC_FREE && left->units == left_units)
                {
                    bmem_check_run_footer(bmem_node, left, left_units, MAGIC_FREE);
                    bmem_remove_run(bmem_node, left);
                    bmem = left;
                    units += left_units;
                    run_start = (uint8_t *)bmem;
                }
            }
        }
    }

    run_end = bmem_run_end(bmem_node, bmem, units);
    if (run_end < bmem_node->tailer_ptr)
    {
        bmem_item_t *right = (bmem_item_t *)run_end;

        RT_ASSERT(MAGIC_FREE == right->magic || MAGIC_ALLOC == right->magic);
        if (MAGIC_FREE == right->magic && right->units > 0 && bmem_run_end(bmem_node, right, right->units) <= bmem_node->tailer_ptr)
        {
            uint16_t right_units = right->units;

            bmem_check_run_footer(bmem_node, right, right_units, MAGIC_FREE);
            bmem_remove_run(bmem_node, right);
            units += right_units;
        }
    }

    bmem_insert_run(bmem_node, bmem, units);
}

static inline bmem_item_t *bmem_find_alloc_run(bmem_node_t *bmem_node, uint32_t size, uint16_t *units, uint16_t *bin, bmem_item_t **prev)
{
    *units = bmem_run_units(bmem_node, size);
    return bmem_find_run(bmem_node, *units, bin, prev);
}

void *bmem_alloc(uint32_t size)
{
    if (size > BMEM_MAX_ALLOC_SIZE) return NULL;
#ifdef USING_BMEM_MAGIC
    if (size > 0xFFFF) return NULL;
#endif
    if (BMEM_NULL == bmem_list_header.statue || size > bmem_list_header.max_size) return NULL;

    bmem_node_t *bmem_node = &bmem_list_header.node;
    bmem_item_t *ptr = NULL;
    uint16_t units;
    uint16_t bin;
    uint16_t run_units;
    bmem_item_t *prev;

    rt_mutex_take(&bmem_mutex, RT_WAITING_FOREVER);

    ptr = bmem_find_alloc_run(bmem_node, size, &units, &bin, &prev);
    if (!ptr)
    {
        rt_mutex_release(&bmem_mutex);
        return NULL;
    }

    {
        run_units = ptr->units;

        bmem_remove_run_from_bin(bmem_node, bin, prev, ptr);
        if (run_units > units && (run_units - units) * bmem_node->size >= BMEM_HEAD_SIZE + BMEM_FOOT_SIZE)
        {
            bmem_item_t *remain = (bmem_item_t *)bmem_run_end(bmem_node, ptr, units);
            bmem_insert_run(bmem_node, remain, run_units - units);
        }
        else
        {
            units = run_units;
        }
    }

#ifdef USING_BMEM_MAGIC
    RT_ASSERT(ptr->magic == MAGIC_FREE);
    ptr->magic = MAGIC_ALLOC;
    ptr->size = size;
    ptr->units = units;
    bmem_set_run_footer(bmem_node, ptr, units, MAGIC_ALLOC);
#endif

    bmem_node->act_used_num += units;
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
    uint16_t units = bmem->units;
    RT_ASSERT(MAGIC_ALLOC == bmem->magic &&
              bmem->size <= bmem_node->size * bmem_node->num - BMEM_HEAD_SIZE - BMEM_FOOT_SIZE);
    bmem_check_run_footer(bmem_node, bmem, units, MAGIC_ALLOC);
    bmem->magic = MAGIC_FREE;
#else
    uint16_t units = bmem_run_units(bmem_node, 0);
#endif
    bmem_merge_and_insert_run(bmem_node, bmem, units);
    bmem_node->act_used_num -= units;
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
    uint32_t    *end = (uint32_t *)SECTION_END_ADDR(BMEM_SECTION_NAME);
    uint32_t    *temp = (uint32_t *)SECTION_START_ADDR(BMEM_SECTION_NAME);
    bmem_node_t *bmem_node = &bmem_list_header.node;

    memset(&bmem_list_header, 0x00, sizeof(bmem_list_header));

    while (temp < end)
    {
        block_desc = (bmem_desc_t *)temp;

        if (block_desc->size > 0 && block_desc->name && block_desc->num > 0)
        {
            RT_ASSERT(block_desc->num <= 0xFFFF);

            bmem_node->size = block_desc->size;
            bmem_node->name = block_desc->name;
            bmem_node->num = block_desc->num;
            bmem_node->bin_num = bmem_calc_run_bin_num(bmem_node->size);
            bmem_node->header_ptr = block_desc->ptr;
            bmem_node->tailer_ptr = bmem_node->header_ptr + block_desc->num * bmem_node->size;
            bmem_insert_run(bmem_node, (bmem_item_t *)bmem_node->header_ptr, (uint16_t)block_desc->num);

            bmem_list_header.max_size = bmem_node->size * bmem_node->num - BMEM_HEAD_SIZE - BMEM_FOOT_SIZE;
            bmem_list_header.statue = BMEM_NORMAL;

            rt_kprintf("%s: %s, unit_size %d total_units %d bin_num %d\n",
                       __func__, bmem_node->name, bmem_node->size, bmem_node->num, bmem_node->bin_num);
            return;
        }

        temp++;
    }
}

/**
 * @brief  Unload block memheap.
 */
static void bmem_unload(void)
{
    rt_mutex_take(&bmem_mutex, RT_WAITING_FOREVER);
    memset(&bmem_list_header, 0x00, sizeof(bmem_list_header));
    rt_mutex_release(&bmem_mutex);
}

uint32_t bmem_backup(uint32_t (*func)(uint32_t addr, uint32_t size))
{
    uint32_t ret = 0;

    RT_ASSERT(func);

    rt_mutex_take(&bmem_mutex, RT_WAITING_FOREVER);
    if (BMEM_NORMAL == bmem_list_header.statue && bmem_list_header.node.act_used_num > 0)
    {
        bmem_node_t *bmem_node = &bmem_list_header.node;

        RT_ASSERT(bmem_node->name);
        for (uint32_t unit_index = 0; unit_index < bmem_node->num;)
        {
            bmem_item_t *bmem = (bmem_item_t *)(bmem_node->header_ptr + unit_index * bmem_node->size);
#ifdef USING_BMEM_MAGIC
            uint16_t units;

            RT_ASSERT(MAGIC_ALLOC == bmem->magic || MAGIC_FREE == bmem->magic);
            units = bmem->units;
            RT_ASSERT(units > 0 && unit_index + units <= bmem_node->num);
            bmem_check_run_footer(bmem_node, bmem, units, bmem->magic);

            if (MAGIC_ALLOC == bmem->magic)
            {
                ret = func((uint32_t) bmem, units * bmem_node->size);
                if (ret) break;
            }
            else
            {
                ret = func((uint32_t) bmem, BMEM_HEAD_SIZE);
                if (ret) break;

                ret = func((uint32_t)bmem_run_footer(bmem_node, bmem, units), BMEM_FOOT_SIZE);
                if (ret) break;
            }
            unit_index += units;
#else
            ret = func((uint32_t) bmem, bmem_node->size);
            if (ret) break;
            unit_index++;
#endif
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
    bmem_node_t *node = &bmem_list_header.node;
#ifdef USING_BMEM_MAGIC
    bmem_item_t *bmem;

    if (!p || BMEM_NORMAL != bmem_list_header.statue || !BMEM_VALID(p, node)) return NULL;

    bmem = (bmem_item_t *)((uint8_t *)p - BMEM_HEAD_SIZE);
    return (MAGIC_ALLOC == bmem->magic && bmem->node == node) ? node : NULL;
#else
    return (p && BMEM_NORMAL == bmem_list_header.statue && BMEM_VALID(p, node)) ? node : NULL;
#endif
}
