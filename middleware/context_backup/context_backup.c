/**
  ******************************************************************************
  * @file   context_mng.c
  * @author Sifli software development team
  * @brief Sibles source of wrapper device for ipc queue
 *
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
#include "rtthread.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "board.h"
#include "context_backup.h"
#include "log.h"
#include "mem_section.h"
#ifdef CONTEXT_BACKUP_COMPRESSION_ENABLED
    #include "lz4.h"
#endif /* CONTEXT_BACKUP_COMPRESSION_ENABLED */

#ifdef SOC_BF0_HCPU

/** context data type  */
typedef enum
{
    CB_DATA_STACK,
    CB_DATA_HEAP,
    CB_DATA_STATIC
} cb_data_type_t;


typedef rt_err_t (*cb_restore_callback_t)(void *user_data, uint8_t *buf, rt_uint32_t size, rt_compressor_cb_t decompressor_cb);

typedef struct
{
    /** context data original address */
    //uint32_t data_addr;
    void *user_data;
    cb_restore_callback_t restore_callback;
    /** context data length,
     *
     * not including cb_block_hdr_t and padding byte for 4 bytes alignment
     */
    uint32_t data_len;
    /** data type, @see cb_data_type_t */
    uint8_t data_type;
    uint8_t compressed;
    uint8_t reserved[2];
} cb_block_hdr_t;


/*

typedef struct
{
    cb_block_hdr_t hdr;
    uint8_t data[0];
    uint8_t padding[]; //for 4 bytes aligned
} cb_block_t;

*/

typedef struct
{
    /** DB maximum len, excluding space of cb_context_db_t */
    uint32_t max_len;
    /** total block length
     *
     * size of block_list, including block header and data
     */
    uint32_t total_block_len;
    uint8_t block_num;
    /** indicate which data types need backup
     *
     * such as #CB_BACKUP_STACK_MASK, #CB_BACKUP_HEAP_MASK
     */
    uint8_t backup_mask;
    /** backup region number */
    uint8_t backup_region_num;
    uint8_t reserved;
    /** backup region list */
    cb_retained_region_t backup_region_list[CB_MAX_BACKUP_REGION_NUM];
    /* cb_block_t block_list[] */
} cb_context_db_t;


#define CB_CONTEXT_DB_HDR_SIZE    (sizeof(cb_context_db_t))
#define CB_BLOCK_HDR_SIZE         (sizeof(cb_block_hdr_t))
/** block size including header, data and padding byte */
#define CB_BLOCK_SIZE(data_len)   (RT_ALIGN(((data_len) + CB_BLOCK_HDR_SIZE), 4))
#define CB_AVAILABLE_SIZE(size)   (RT_ALIGN_DOWN((size), 4))
#define CB_NEXT_BLOCK(hdr)        ((cb_block_hdr_t *)((uint32_t)(hdr) + CB_BLOCK_SIZE((hdr)->data_len)))

#define CB_IS_IN_SRAM_RANGE(addr)    ((((addr) >= HPSYS_RAM0_BASE) && ((addr) < HPSYS_RAM_END)) ? true : false)

#ifdef HPSYS_ITCM_BASE
    #define CB_IS_IN_ITCM_RANGE(addr)    ((((addr) >= HPSYS_ITCM_BASE) && ((addr) < HPSYS_ITCM_END)) ? true : false)
#else
    #define CB_IS_IN_ITCM_RANGE(addr)    (false)
#endif /* HPSYS_ITCM_BASE */

#define CB_IS_IN_RETM_RANGE(addr)    ((((addr) >= HPSYS_RETM_BASE) && ((addr) < HPSYS_RETM_END)) ? true : false)


#if defined(SF32LB55X) || defined(SF32LB58X)
    #define CB_IS_IN_EZIP_ADDR_RANGE(addr)  (!CB_IS_IN_ITCM_RANGE((addr)) && !CB_IS_IN_RETM_RANGE((addr)))
#else
    #define CB_IS_IN_EZIP_ADDR_RANGE(addr)  (!CB_IS_IN_ITCM_RANGE((addr)))
#endif /* SF32LB55X || SF32LB58X  */


#define CB_MAX_BLOCK_HDR_LIST_LEN     (32)


RETM_BSS_SECT_BEGIN(cb_context_db)
static cb_context_db_t *cb_context_db RETM_BSS_SECT(cb_context_db);
RETM_BSS_SECT_END

RETM_BSS_SECT_BEGIN(cb_context_db_stats)
static uint32_t cb_max_used_size RETM_BSS_SECT(cb_context_db_stats);
static uint32_t cb_total_size RETM_BSS_SECT(cb_context_db_stats);
RETM_BSS_SECT_END


L1_NON_RET_BSS_SECT_BEGIN(last_context_db)
L1_NON_RET_BSS_SECT(last_context_db, RT_USED static cb_context_db_t last_context_db);
L1_NON_RET_BSS_SECT_END

L1_NON_RET_BSS_SECT_BEGIN(last_block_hdr_list)
L1_NON_RET_BSS_SECT(last_block_hdr_list, RT_USED static cb_block_hdr_t last_block_hdr_list[CB_MAX_BLOCK_HDR_LIST_LEN]);
L1_NON_RET_BSS_SECT_END

#ifdef CONTEXT_BACKUP_COMPRESSION_ENABLED
    L1_NON_RET_BSS_SECT_BEGIN(lz4_ctx)
    L1_NON_RET_BSS_SECT(lz4_ctx, static LZ4_stream_t lz4_ctx);
    L1_NON_RET_BSS_SECT_END
#endif /* CONTEXT_BACKUP_COMPRESSION_ENABLED */

static uint8_t *cb_get_block_data_buf(uint32_t *size)
{
    uint8_t *ptr;

    RT_ASSERT(cb_context_db);
    RT_ASSERT(size);

    ptr = (uint8_t *)(cb_context_db + 1) + cb_context_db->total_block_len;
    *size = cb_context_db->max_len - cb_context_db->total_block_len;
    /* truncate the size to multiple of 4 bytes */
    *size = CB_AVAILABLE_SIZE(*size);
    if (*size <= CB_BLOCK_HDR_SIZE)
    {
        ptr = NULL;
        *size = 0;
    }
    else
    {
        ptr += CB_BLOCK_HDR_SIZE;
        *size -= CB_BLOCK_HDR_SIZE;
    }

    return ptr;
}

static void cb_update_db(void *user_data, cb_restore_callback_t callback,
                         uint8_t *data_buf, uint32_t used_size, cb_data_type_t data_type,
                         bool compressed)
{
    cb_block_hdr_t *hdr;

    hdr = (cb_block_hdr_t *)(data_buf - CB_BLOCK_HDR_SIZE);

    hdr->user_data = user_data;
    hdr->restore_callback = callback;
    hdr->data_len = used_size;
    hdr->data_type = data_type;
    hdr->compressed = compressed ? 1 : 0;

    cb_context_db->block_num++;
    cb_context_db->total_block_len += CB_BLOCK_SIZE(hdr->data_len);
    RT_ASSERT(cb_context_db->total_block_len <= cb_context_db->max_len);

}

#if 0
static cb_block_hdr_t *cb_alloc_space(uint32_t addr, uint32_t len, cb_data_type_t data_type)
{
    uint16_t block_size;
    cb_block_hdr_t *hdr;

    RT_ASSERT(cb_context_db);

    if (CB_BLOCK_SIZE(len) > UINT16_MAX)
    {
        return NULL;
    }

    /* block length is 4 bytes aligned */
    block_size = (uint16_t)CB_BLOCK_SIZE(len);

    if ((cb_context_db->total_block_len + block_size) > cb_context_db->max_len)
    {
        return NULL;
    }

    hdr = (cb_block_hdr_t *)((uint32_t)(cb_context_db + 1) + cb_context_db->total_block_len);
    hdr->data_addr = addr;
    hdr->data_len = len;
    hdr->data_type = data_type;
    cb_context_db->total_block_len += block_size;
    cb_context_db->block_num++;

    return hdr;
}


static rt_err_t cb_save_data(uint32_t addr, uint32_t len, cb_data_type_t data_type)
{
    uint16_t block_size;
    cb_block_hdr_t *hdr;

    RT_ASSERT(cb_context_db);

    if (0 == len)
    {
        return RT_EOK;
    }

    hdr = cb_alloc_space(addr, len, data_type);
    if (!hdr)
    {
        return RT_EFULL;
    }

    memcpy((void *)(hdr + 1), (void *)addr, len);

    return RT_EOK;
}
#endif

static rt_err_t cb_save_all_stacks(void)
{
    struct rt_object_information *info;
    uint32_t max_size;
    rt_uint32_t used_size;
    struct rt_thread *thread;
    struct rt_list_node *node;
    struct rt_list_node *thread_list;
    uint32_t start_addr;
    struct rt_thread *curr_thread;
    rt_err_t err;
    uint8_t *data_buf;

    curr_thread = rt_thread_self();

    err = RT_EOK;
    info = rt_object_get_information(RT_Object_Class_Thread);
    thread_list = &info->object_list;
    for (node = thread_list->next; node != thread_list; node = node->next)
    {
        thread = rt_list_entry(node, struct rt_thread, list);
        if (thread == curr_thread)
        {
            /* don't save stack of current thread */
            continue;
        }

        if (!CB_IS_IN_SRAM_RANGE((uint32_t)thread->stack_addr) && !CB_IS_IN_ITCM_RANGE((uint32_t)thread->stack_addr))
        {
            /* only backup stack in SRAM */
            continue;
        }

        /* get next available data buffer */
        data_buf = cb_get_block_data_buf(&max_size);
        if (RT_NULL == data_buf)
        {
            err = RT_EFULL;
            break;
        }

        err = rt_thread_stack_backup(thread, data_buf, max_size, &used_size);
        if (RT_EOK != err)
        {
            break;
        }

        cb_update_db((void *)thread, rt_thread_stack_restore, data_buf, used_size, CB_DATA_STACK, false);

    }

    return err;
}

#ifdef CONTEXT_BACKUP_COMPRESSION_ENABLED
static rt_ubase_t cb_compress_data(void *dst, const void *src, rt_ubase_t src_size, rt_ubase_t dst_size)
{
    rt_ubase_t cmpr_len;
    uint32_t size_field_len = sizeof(uint32_t) * 2;
    cmpr_len = LZ4_compress_fast_extState(&lz4_ctx, (const char *)src, (void *)((uint32_t)dst + size_field_len),
                                          src_size, dst_size - size_field_len, 1);
    if (cmpr_len > 0)
    {
        /* save orignal size */
        *(uint32_t *)dst = src_size;
        /* save compressed size which is needed by ezip for decompression */
        *((uint32_t *)dst + 1) = cmpr_len;
        return (cmpr_len + size_field_len);
    }
    else
    {
        return 0;
    }
}

static rt_ubase_t cb_decompress_data(void *dst, const void *src, rt_ubase_t src_size, rt_ubase_t dst_size)
{
    EZIP_DecodeConfigTypeDef config;
    HAL_StatusTypeDef res;
    uint32_t size_field_len = sizeof(uint32_t) * 2;
    EZIP_HandleTypeDef ezip_handle = {0};
    uint32_t org_size;
    uint32_t cmpr_size;
    uint32_t output_size;

    org_size = *(uint32_t *)src;
    cmpr_size = *((uint32_t *)src + 1);
    if (org_size > dst_size)
    {
        return 0;
    }

    if (CB_IS_IN_EZIP_ADDR_RANGE((uint32_t)dst) && CB_IS_IN_EZIP_ADDR_RANGE((uint32_t)src))
    {
        /* skip orignal size field */
        config.input = (uint8_t *)((uint32_t)src + sizeof(uint32_t));
        config.output = dst;
        config.start_x = 0;
        config.start_y = 0;
        config.width = 0;
        config.height = 0;
        config.work_mode = HAL_EZIP_MODE_LZ4;
        config.output_mode = HAL_EZIP_OUTPUT_AHB;
        ezip_handle.Instance = hwp_ezip;
        HAL_EZIP_Init(&ezip_handle);
        res = HAL_EZIP_Decode(&ezip_handle, &config);
        RT_ASSERT(HAL_OK == res);
    }
    else
    {
        /* Use software decompression instead as EZIP cannot access src or dst buffer */
        /* skip two size fields */
        src = (void *)((uint32_t *)src + 2);
        output_size = LZ4_decompress_safe(src, (char *)dst, cmpr_size, org_size);
        RT_ASSERT(output_size == org_size);
    }

    return org_size;
}

#endif

static rt_err_t cb_save_all_heaps(void)
{
    uint32_t start_addr;
    uint32_t size;
    rt_uint32_t used_size;
    rt_err_t err;
    uint8_t *data_buf;

#ifdef RT_USING_MEMHEAP
    struct rt_object_information *info;
    struct rt_list_node *heap_list;
    struct rt_memheap *mh;
    struct rt_list_node *node;
#endif


#ifdef RT_USING_SMALL_MEM
#ifndef RT_USING_MEMHEAP_AS_HEAP

    do
    {
        rt_compressor_cb_t compressor_cb;
        bool compressed;
#ifdef CONTEXT_BACKUP_COMPRESSION_ENABLED
        compressor_cb = cb_compress_data;
        compressed = true;
#else
        compressor_cb = RT_NULL;
        compressed = false;
#endif
        start_addr = rt_mem_base();
        if (!CB_IS_IN_SRAM_RANGE(start_addr) && !CB_IS_IN_ITCM_RANGE(start_addr))
        {
            break;
        }

        /* get next available data buffer */
        data_buf = cb_get_block_data_buf(&size);
        if (RT_NULL == data_buf)
        {
            err = RT_EFULL;
            goto __EXIT;
        }

        err = rt_mem_backup(data_buf, size, &used_size, compressor_cb);
        if (RT_EOK != err)
        {
            goto __EXIT;
        }

        cb_update_db(NULL, rt_mem_restore, data_buf, used_size, CB_DATA_HEAP, compressed);
    }
    while (0);
#endif
#endif

#ifdef RT_USING_MEMHEAP
    info = rt_object_get_information(RT_Object_Class_MemHeap);
    heap_list = &info->object_list;
    for (node = heap_list->next; node != heap_list; node = node->next)
    {
        mh = (struct rt_memheap *)rt_list_entry(node, struct rt_object, list);

        if (!CB_IS_IN_SRAM_RANGE((uint32_t)mh->start_addr) && !CB_IS_IN_ITCM_RANGE((uint32_t)mh->start_addr))
        {
            /* only backup heap in SRAM */
            continue;
        }

        /* get next available data buffer */
        data_buf = cb_get_block_data_buf(&size);
        if (RT_NULL == data_buf)
        {
            LOG_W("memheap[%p] bakcup fail", mh);
            err = RT_EFULL;
            goto __EXIT;
        }

        err = rt_memheap_backup(mh, data_buf, size, &used_size);
        if (RT_EOK != err)
        {
            LOG_W("memheap[%p] bakcup fail", mh);
            goto __EXIT;
        }

        cb_update_db((void *)mh, rt_memheap_restore, data_buf, used_size, CB_DATA_HEAP, false);
    }
#endif

    return RT_EOK;

__EXIT:

    return err;
}

static rt_err_t cb_restore_static_data(void *target, uint8_t *buf, rt_uint32_t size, rt_compressor_cb_t decompressor_cb)
{
    cb_retained_region_t *backup_region;
    rt_ubase_t actual_size;

    RT_ASSERT(target && buf);

    backup_region = (cb_retained_region_t *)target;
    if (decompressor_cb)
    {
        actual_size = decompressor_cb((void *)backup_region->start_addr, buf, size, backup_region->len);
        RT_ASSERT(actual_size == backup_region->len);
    }
    else
    {
        RT_ASSERT(size == backup_region->len);
        memcpy((void *)backup_region->start_addr, buf, size);
    }

    return RT_EOK;
}



rt_err_t cb_save_static_data(void)
{
    uint32_t i;
    uint32_t max_size;
    uint8_t *data_buf;
    rt_err_t err;
    cb_retained_region_t *backup_region;
    uint32_t wr_size;
    bool compressed;

    RT_ASSERT(cb_context_db);

    backup_region = &cb_context_db->backup_region_list[0];
    for (i = 0; i < cb_context_db->backup_region_num; i++)
    {
        if (backup_region->len > 0)
        {
            /* get next available data buffer */
            data_buf = cb_get_block_data_buf(&max_size);
            if (RT_NULL == data_buf)
            {
                err = RT_EFULL;
                goto __EXIT;
            }

#ifdef CONTEXT_BACKUP_COMPRESSION_ENABLED
            wr_size = cb_compress_data((void *)data_buf, (void *)backup_region->start_addr, backup_region->len, max_size);
            compressed = true;
#else
            if (backup_region->len <= max_size)
            {
                memcpy((void *)data_buf, (void *)backup_region->start_addr, backup_region->len);
                wr_size = backup_region->len;
            }
            else
            {
                /* no sufficient space */
                wr_size = 0;
            }
            compressed = false;
#endif
            if (0 == wr_size)
            {
                err = RT_EFULL;
                goto __EXIT;
            }
            cb_update_db((void *)backup_region, cb_restore_static_data, data_buf, wr_size, CB_DATA_STATIC, compressed);
        }

        backup_region++;
    }
    return RT_EOK;

__EXIT:

    return err;

}


rt_err_t cb_init(cb_backup_param_t *param)
{
    uint32_t i;
    cb_retained_region_t *backup_region;
    rt_err_t err;
    uint32_t max_size;
    uint8_t *data_buf;

    RT_ASSERT(NULL == cb_context_db);

    if (!param || (param->ret_mem_size < CB_CONTEXT_DB_HDR_SIZE))
    {
        return RT_ERROR;
    }

    if (param->backup_region_num > CB_MAX_BACKUP_REGION_NUM)
    {
        return RT_EFULL;
    }

    cb_context_db = (cb_context_db_t *)param->ret_mem_start_addr;
    cb_context_db->max_len = param->ret_mem_size - CB_CONTEXT_DB_HDR_SIZE;
    cb_context_db->backup_mask = param->backup_mask;

    cb_context_db->block_num = 0;
    cb_context_db->total_block_len = 0;


    memcpy(&cb_context_db->backup_region_list[0], &param->backup_region_list[0], sizeof(cb_context_db->backup_region_list));
    cb_context_db->backup_region_num = param->backup_region_num;


    return RT_EOK;

__EXIT:
    return RT_ERROR;
}

rt_err_t cb_deinit(void)
{
    cb_context_db = NULL;

    return RT_EOK;
}


rt_err_t cb_save_context(void)
{
    rt_err_t err;

    if (!cb_context_db)
    {
        return RT_ERROR;
    }

    /* static data is restored first as heap restore needs static variable */
    if (cb_context_db->backup_mask & CB_BACKUP_STATIC_DATA_MASK)
    {
        err = cb_save_static_data();
        if (RT_EOK != err)
        {
            goto __EXIT;
        }
    }

    if (cb_context_db->backup_mask & CB_BACKUP_HEAP_MASK)
    {
        err = cb_save_all_heaps();
        if (RT_EOK != err)
        {
            goto __EXIT;
        }
    }

    /* stack is restored after static data and heap, as thread control block is needed by stack restore  */
    if (cb_context_db->backup_mask & CB_BACKUP_STACK_MASK)
    {
        err = cb_save_all_stacks();
        if (RT_EOK != err)
        {
            goto __EXIT;
        }
    }

    if (cb_context_db->total_block_len > cb_max_used_size)
    {
        cb_max_used_size = cb_context_db->total_block_len;
        cb_total_size = cb_context_db->max_len;
    }

    return RT_EOK;

__EXIT:

    return err;
}


rt_err_t cb_restore_context(void)
{
    uint32_t i;
    cb_block_hdr_t *hdr;
    rt_err_t err;

    if (!cb_context_db)
    {
        return RT_ERROR;
    }

    memcpy(&last_context_db, cb_context_db, sizeof(last_context_db));
    hdr = (cb_block_hdr_t *)(cb_context_db + 1);
    for (i = 0; i < cb_context_db->block_num; i++)
    {
        RT_ASSERT(hdr->restore_callback);

#ifdef CONTEXT_BACKUP_COMPRESSION_ENABLED
        if (hdr->compressed)
        {
            err = hdr->restore_callback(hdr->user_data, (uint8_t *)(hdr + 1), hdr->data_len, cb_decompress_data);
        }
        else
#endif /* CONTEXT_BACKUP_COMPRESSION_ENABLED */
        {
            err = hdr->restore_callback(hdr->user_data, (uint8_t *)(hdr + 1), hdr->data_len, RT_NULL);
        }
        RT_ASSERT(RT_EOK == err);
        if (i < CB_MAX_BLOCK_HDR_LIST_LEN)
        {
            memcpy(&last_block_hdr_list[i], hdr, sizeof(last_block_hdr_list[0]));
        }

        hdr = CB_NEXT_BLOCK(hdr);

        RT_ASSERT(((uint32_t)hdr - (uint32_t)(cb_context_db + 1)) <= cb_context_db->total_block_len);
    }

    /* deallocate blocks */
    cb_context_db->block_num = 0;
    cb_context_db->total_block_len = 0;

    return RT_EOK;
}


void cb_get_stats(uint32_t *total, uint32_t *min_free)
{
    if (total)
    {
        *total = cb_total_size;
    }
    if (min_free)
    {
        *min_free = cb_total_size - cb_max_used_size;
    }
}
#endif // SOC_BF0_HCPU

