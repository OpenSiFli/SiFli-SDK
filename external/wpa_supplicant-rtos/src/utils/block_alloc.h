/** @file block_alloc.h
 *
 *  @brief  This file provides Block Allocator APIs.
 *
 *  Copyright 2008-2020 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 *
 * There are some packages which perform a large number of fixed-size
 * allocations. But, the maximum number of allocations that are active at any
 * given point in time is very less.
 * It would be desirable to have a slab allocator which can reduce the
 * fragmentation that may potentially be caused by the use of
 * such large number of allocations.
 * The slab/block allocator aims at allocating fixed size blocks as per the
 * requirement of the user, subject to the total size specified by the user
 * when initializing the pool of that specific block size.
 *
 * Example: If a user wishes to create a pool with restricted access and
 * having 10 blocks of size say 20 bytes, then he can first initialize the pool.
 *
 * For that he can use:
 *
 * * struct pool_info *pool_ptr;
 *
 * * char buffer[20 * 10];
 *
 * * char *block_ptr;
 *
 * Steps:
 *
 * * Initialize Pool:
 * pool_ptr = ba_pool_create(20, buffer, sizeof(buffer));
 *
 * Further, he may allocate or deallocate blocks using the pool pointer.
 *
 * * Allocation
 * block_ptr = ba_block_allocate(pool_ptr);
 *
 * * Deallocation:
 * ba_block_release(pool_ptr, block_ptr);
 *
 * Further, if a user wishes to delete the memory pool, he may do so by:
 *
 * * Delete pool
 * ba_pool_delete(pool_ptr);
 *
 * \note A pool can be deleted only if all the blocks in it are free.
 */

#ifndef BLOCK_ALLOC_H_
#define BLOCK_ALLOC_H_

struct pool_info
{
    /** head points to the start of the block for that particular size. */
    void *head;
    /** bitmap is used to know the status of each block starting from
     *  block 0. (corresponding to LSB) */
    uint32_t *bitmap;
    /** block_count indicates the maximum number of blocks in memory pool */
    unsigned long max_block_count;
    /** block_size indicates the fixed size of each block in the pool. */
    unsigned long block_size;
    /** mutex to restrict access to critical section. */
    OSA_MUTEX_HANDLE_DEFINE(mutex);
};

/** Create a memory pool of specified block size.
 *
 *  This function creates and initializes a pool with blocks of specified size.
 *  \param block_size The required block size for each block.
 *  \param buf A pointer to the caller allocated buffer. This buffer needs
 *  to be 4 byte aligned.
 *   All allocations are satisfied from this pool.
 *  \param pool_size Total size of the buffer.
 *
 *  \return pointer to the created pool if successful.
 *  \return NULL if unsuccessful.
 */
struct pool_info *ba_pool_create(unsigned long block_size, void *buf, unsigned long pool_size);

/** Delete already initialized memory pool.
 *
 *  This function deletes an initialized memory pool.
 *  \param b A pointer of structure pool_info to be deleted.
 *
 *  \return WM_SUCCESS if successful.
 *  \return -WM_FAIL if unsuccessful.
 */
int ba_pool_delete(struct pool_info *b);

/** Allocate a block from the memory pool.
 *
 *  This function allocates a block of the specified size.
 *  \param b An already initialized pointer of structure pool_info corresponding
 *  to the required size.
 *
 *  \return the pointer if successful.
 *  \return NULL if unsuccessful.
 */
void *ba_block_allocate(struct pool_info *b);

/** Free a block from the memory pool.
 *
 *  This function frees a block of the specified size.
 *  \param b An already initialized pointer of structure pool_info whose block is to be freed.
 *  \param alloc The actual pointer to be deallocated.
 *
 *  \return WM_SUCCESS if successful.
 *  \return -WM_FAIL if unsuccessful.
 */
int ba_block_release(struct pool_info *b, void *alloc);
#endif
