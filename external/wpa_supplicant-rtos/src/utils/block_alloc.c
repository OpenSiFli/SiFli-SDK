/** @file block_alloc.c
 *
 *  @brief  This file provides Block Allocator APIs.
 *
 *  Copyright 2008-2020, 2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <fsl_os_abstraction.h>
#include "rtthread.h"
#include "os.h"

#include <block_alloc.h>

#define ba_e(...) PRINTF("[ba]Error: " _fmt_ "\n\r", ##__VA_ARGS__)
#define ba_w(...) PRINTF("[ba]Warn: ", _fmt_ "\n\r", ##__VA_ARGS__)

#ifdef CONFIG_BLOCK_ALLOCATOR_DEBUG
#define ba_d(...) PRINTF("[ba]" _fmt_ "\n\r", ##__VA_ARGS__)
#else
#define ba_d(...)
#endif /* ! BLOCK_ALLOCATOR_DEBUG */

/* alignment value should be a power of 2 */
#define __ALIGN__(num, num_type, align) MASK(num, (num_type)align - 1)
#define MASK(num, mask)                 ((num + mask) & ~(mask))

#define ffs __builtin_ffs

#define mul_32(num)     (num << 5)
#define div_32(num)     (num >> 5)
#define div_8(num)      (num >> 3)
#define modulo_32(num)  (num & 31)
#define bit_sizeof(num) (num << 3)

#define get_word(bitmap, block_num) (bitmap + div_32(block_num))
#define get_bit(block_num)          ((uint32_t)0x01 << modulo_32(block_num))

static int ffs2(int mask)
{
    int bit;

    if (mask == 0)
    {
        return(0);
    }
    for (bit = 1; !(mask & 1); bit++)
    {
        mask >>= 1;
    }
    return(bit);
}

static void ba_bitmap_set_bit(struct pool_info *b, int block_num)
{
    /* Set the specified bit to 1 indicating that now it is allocated */
    *(get_word(b->bitmap, block_num)) |= get_bit(block_num);
}

static void ba_bitmap_reset_bit(struct pool_info *b, int block_num)
{
    /* Set the specified bit to 0 indicating that now it is free */
    *(get_word(b->bitmap, block_num)) &= ~(get_bit(block_num));
}

static int ba_bitmap_get_bit(struct pool_info *b, int block_num)
{
    /* return bit value in bitmap at location block_num */
    return *(get_word(b->bitmap, block_num)) & (get_bit(block_num));
}

/* This function returns the free block number if successful.
 * It returns -1 on error */
static int ba_get_free_block(const struct pool_info *b)
{
    int i, block_num = 0;
    for (i = 0; i < b->max_block_count; i += bit_sizeof(sizeof(*b->bitmap)))
    {
        block_num = ffs2(~(*get_word(b->bitmap, i)));
        if (block_num != 0)
        {
            /* ffs returns bit position in a word. Hence,
             *  adding 'i' to return position of bit in Bitmap
             */
            return block_num - 1 + i;
        }
    }
    return -1;
}

static bool is_bitmap_empty(const struct pool_info *b)
{
    uint32_t i, *bitmap = b->bitmap;
    for (i = 0; i < b->max_block_count; i += bit_sizeof(sizeof(*b->bitmap)))
    {
        if (*bitmap != 0)
            return false;
        bitmap++;
    }
    return true;
}

/* This function creates the pool for a particular block size */
struct pool_info *ba_pool_create(unsigned long block_size, void *buf, unsigned long pool_size)
{
    int alloc_bytes;
    osa_status_t status;
    struct pool_info *b;
    if (buf == NULL)
    {
        ba_d("Input pointer is NULL.");
        return NULL;
    }

    /* Ensure that the buffer given by user is 4 byte aligned */
    if ((unsigned)buf & 0x3)
    {
        ba_d("Input pointer needs to be word aligned.");
        return NULL;
    }

    if (pool_size % block_size)
    {
        ba_d("Pool size(%d) should be a multiple of block_size(%d)", pool_size, block_size);
        return NULL;
    }

    b = os_zalloc(sizeof(struct pool_info));
    if (!b)
    {
        ba_d("Memory allocation for pool info failed");
        return NULL;
    }

    b->head = (char *)buf;
    ba_d("Address of pool info is: %p.", b->head);

    status = OSA_MutexCreate((osa_mutex_handle_t)b->mutex);
    if (status != KOSA_StatusSuccess)
    {
        ba_d("Mutex creation failed");
        goto err;
    }

    b->block_size      = block_size;
    b->max_block_count = (pool_size / block_size);
    if (b->max_block_count == 0)
    {
        ba_d("Block count is 0");
        goto err;
    }
    alloc_bytes = __ALIGN__(b->max_block_count, unsigned long, bit_sizeof(sizeof(*b->bitmap)));
    b->bitmap   = os_zalloc(div_8(alloc_bytes));
    if (!b->bitmap)
    {
        ba_d("Memory allocation for bitmap failed");
        goto err;
    }
    return b;
err:
    os_free(b);
    return NULL;
}

/* This function destroys the pool (if created) */
int ba_pool_delete(struct pool_info *b)
{
    osa_status_t status;
    if (b == NULL)
    {
        ba_d("Input pointer is NULL.");
        return -1;
    }

    /* Delete the pool only if all its blocks are unallocated or free */
    if (is_bitmap_empty(b))
    {
        status = OSA_MutexLock((osa_mutex_handle_t)b->mutex, osaWaitForever_c);
        if (status != KOSA_StatusSuccess)
        {
            ba_d("Failed to get the lock for block allocator.");
            return -1;
        }
        status = OSA_MutexDestroy((osa_mutex_handle_t)b->mutex);
        if (status != KOSA_StatusSuccess)
        {
            ba_d("Deleting the mutex for block allocator.");
        }
        os_free(b->bitmap);
        b->bitmap = NULL;
        os_free(b);
        return 0;
    }
    else
    {
        ba_d("Trying to free a pool which has blocks allocated.");
        ba_d("Free all the blocks and then try to delete the pool.");
        return -1;
    }
}

/* This function is used to allocate a free block */
void *ba_block_allocate(struct pool_info *b)
{
    int block_num;
    osa_status_t status;
    void *ptr;
    if (b == NULL)
    {
        ba_d("Input pointer is NULL or not initialized.");
        return NULL;
    }

    status = OSA_MutexLock((osa_mutex_handle_t)b->mutex, osaWaitForever_c);
    if (status != KOSA_StatusSuccess)
    {
        return NULL;
    }

    /* Get the position of the first free block */
    block_num = ba_get_free_block(b);
    if (block_num == -1 || block_num >= b->max_block_count)
    {
        ba_d("No free block. Returning.");
        OSA_MutexUnlock((osa_mutex_handle_t)b->mutex);
        return NULL;
    }
    ba_d("Block number to be allocated is: %d.", block_num);
    ptr = ((uint8_t *)b->head + (b->block_size * block_num));

    /* Update the bitmap by setting the corresponding bit to 1 (indicating
     * allocated) */
    ba_bitmap_set_bit(b, block_num);
    OSA_MutexUnlock((osa_mutex_handle_t)b->mutex);
    return ptr;
}

/* This function is used to free an allocated block */
int ba_block_release(struct pool_info *b, void *alloc)
{
    osa_status_t status;
    unsigned int diff;

    if (b == NULL || alloc == NULL || alloc < b->head)
    {
        ba_d("Input pointer is NULL or not initialized.");
        return -1;
    }

    diff = (unsigned int)((uint8_t *)alloc - (uint8_t *)b->head);

    if (diff % b->block_size)
    {
        ba_d("Invalid address. Cannot free the memory.");
        return -1;
    }
    else
    {
        diff /= b->block_size;
    }

    status = OSA_MutexLock((osa_mutex_handle_t)b->mutex, osaWaitForever_c);
    if (status != KOSA_StatusSuccess)
    {
        ba_d("Failed to get the lock for block allocator.");
        return -1;
    }

    /* Check if the specified bit is allocated (1) else return error */
    if (!ba_bitmap_get_bit(b, diff))
    {
        ba_d("Trying to free a location that is already free.");
        OSA_MutexUnlock((osa_mutex_handle_t)b->mutex);
        return -1;
    }

    ba_bitmap_reset_bit(b, diff);
    OSA_MutexUnlock((osa_mutex_handle_t)b->mutex);

    ba_d("Freed the requested block successfully.");
    return 0;
}
