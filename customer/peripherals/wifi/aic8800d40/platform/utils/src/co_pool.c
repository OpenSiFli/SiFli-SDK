/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file co_pool.c
 *
 * Copyright (C) RivieraWaves 2011-2019
 *
 * @brief Implementation of the co_pool API.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CO_POOL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
// for NULL
#include <stddef.h>
#include "co_pool.h"
// for assertions
#include "dbg_assert.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void co_pool_init(struct co_pool *pool,
                  struct co_pool_hdr *pool_hdr,
                  void *elements,
                  uint32_t elem_size,
                  uint32_t elem_cnt)
{
    uint32_t i;

    // sanity check: check that elements are aligned on word
    AIC_ASSERT_ERR((((intptr_t)elements) & 3) == 0);
    AIC_ASSERT_ERR((elem_size & 3) == 0);


    // build the linked list of elementary descriptors and make them point to the
    // associated pool elements.
    for (i = 0; i < elem_cnt; i++)
    {
        pool_hdr[i].next = &(pool_hdr[i + 1]);

        // initialize the pointer to the element
        pool_hdr[i].element = (void*)((long)elements + (i * elem_size));
    }

    // the last element in the list points to NULL
    pool_hdr[elem_cnt - 1].next = NULL;

    // initialize the pool
    pool->first_ptr = pool_hdr;
    pool->freecnt = elem_cnt;
}



struct co_pool_hdr *co_pool_alloc(struct co_pool *pool,
                                  uint32_t nbelem)
{
    uint32_t i;
    struct co_pool_hdr *desclist_ptr, *currentdesc_ptr, *nextdesc_ptr;

    // sanity check
    AIC_ASSERT_ERR(nbelem != 0);

    // if there aren't enough free elements in the pool, return NULL
    if (pool->freecnt < nbelem)
    {
        return NULL;
    }
    // in the other cases, the function returns a pointer to the list of descriptors that
    // have been allocated.
    desclist_ptr = pool->first_ptr;
    nextdesc_ptr = pool->first_ptr;
    currentdesc_ptr = pool->first_ptr;

    // jump over nbelem elements of the pool
    for (i = 0; i < nbelem; i++)
    {
        currentdesc_ptr = nextdesc_ptr;
        nextdesc_ptr = currentdesc_ptr->next ;
    }

    // Update the free list pointer value as well as the remaining number of element in
    // the pool
    pool->first_ptr = currentdesc_ptr->next;
    pool->freecnt -= nbelem;

    // The last element in the returned list points to NULL
    currentdesc_ptr->next = NULL;

    return desclist_ptr;
}


void co_pool_free(struct co_pool *pool,
                  struct co_pool_hdr *elements,
                  uint32_t nbelem)
{
    uint32_t i;
    struct co_pool_hdr *descfreelist_ptr, *currentdesc_ptr, *lastdesc_ptr;

    // sanity check: passed parameters are OK
    AIC_ASSERT_ERR(nbelem != 0);
    AIC_ASSERT_ERR(elements);

    descfreelist_ptr = pool->first_ptr;
    lastdesc_ptr = elements;

    pool->first_ptr = elements;
    pool->freecnt += nbelem;

    // move to the last element of the freed list
    for (i = 0; i < (nbelem - 1); i++)
    {
        currentdesc_ptr = lastdesc_ptr;
        lastdesc_ptr = currentdesc_ptr->next ;
    }

    // Hook the list of released descriptor onto the free list of descriptor.
    lastdesc_ptr->next = descfreelist_ptr;
}

/// @}
