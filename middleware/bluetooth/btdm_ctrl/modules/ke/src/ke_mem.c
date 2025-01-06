/**
 ****************************************************************************************
 *
 * @file ke_mem.c
 *
 * @brief Implementation of the heap management module.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MEM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>             // standard definition
#include <stdbool.h>            // standard boolean
#include "arch.h"               // architecture

#include "co_math.h"            // computation utilities
#include "co_utils.h"           // computation utilities
#include "ke_int.h"             // kernel environment
#include "ke_mem.h"             // kernel memory

#include "dbg.h"

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
/// Pattern used to check if memory block is not corrupted
#define KE_LIST_PATTERN                         (0xA5)
#define KE_ALLOCATED_PATTERN                    (0x83)
#define KE_FREE_PATTERN                         (0xF0)

/*
 * LOCAL TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Free memory block delimiter structure (size must be word multiple)
/// Heap can be represented as a doubled linked list.
struct mblock_free
{
    /// Used to check if memory block has been corrupted or not
    uint8_t  corrupt_check;
    /// Heap identifier
    uint8_t  heap_id;
    /// Size of the current free block (including delimiter)
    uint16_t free_size;
    /// Next free block pointer
    struct mblock_free *next;
    /// Previous free block pointer
    struct mblock_free *previous;
};

/// Used memory block delimiter structure (size must be word multiple)
struct mblock_used
{
    /// Used to check if memory block has been corrupted or not
    uint8_t  corrupt_check;
    /// Heap identifier
    uint8_t  heap_id;
    /// Size of the current used block (including delimiter)
    uint16_t size;
    /// for mem leak detect
    uint32_t ret_addr;
};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/// Internal function used for memory allocation
#if !(PLF_MEM_LEAK_DETECT)
    __STATIC void *ke_malloc_internal(uint32_t size, uint8_t type)
#else
    #define ke_malloc_internal(size, type) ke_malloc_internal_with_mem_leak_detection(size, type, filename, line)
    __STATIC void *ke_malloc_internal_with_mem_leak_detection(uint32_t size, uint8_t type, const char *filename, uint16_t line)
#endif // !(PLF_MEM_LEAK_DETECT)

{
#if !(PLF_MEM_LEAK_DETECT)
    DBG_FUNC_ENTER(ke_malloc_internal);
#endif // !(PLF_MEM_LEAK_DETECT)
    struct mblock_free *node = NULL, *found = NULL;
    uint8_t cursor = 0;
    struct mblock_used *alloc = NULL;
    uint32_t totalsize;
    uint8_t heap_id = type;
#if (TRACER_PRESENT && TRC_MEM)
    uint8_t trc_heap_id = 0;
#endif /*(TRACER_PRESENT && TRC_MEM)*/
    DBG_SWDIAG(KE, MALLOC, 1);

    // compute overall block size (including requested size PLUS descriptor size)
    totalsize = CO_ALIGN4_HI(size) + sizeof(struct mblock_used);
    if (totalsize < sizeof(struct mblock_free))
    {
        totalsize = sizeof(struct mblock_free);
    }

    // protect accesses to descriptors
    GLOBAL_INT_DISABLE();

    while (cursor < KE_MEM_BLOCK_MAX)
    {
        // Select Heap to use, first try to use current heap.
        node = ke_env.heap[heap_id];
        DBG_MEM_GRANT_CTRL(ke_env.heap[heap_id], true);
        ASSERT_ERR_KE(node != NULL);

        // go through free memory blocks list
        while (node != NULL)
        {
            ASSERT_ERR_KE(node->corrupt_check == KE_LIST_PATTERN);
            // check if there is enough room in this free block
            if (node->free_size >= totalsize)
            {
                // for first block of the chain, default mblock_free structure must be kept
                if ((node->previous != NULL) || (node->free_size >= (totalsize + sizeof(struct mblock_free))))
                {
                    // if a match was already found, check if this one is smaller
                    if ((found == NULL) || (found->free_size > node->free_size))
                    {
                        found = node;
                    }
                }
            }

            // move to next block
            node = node->next;
        }

        // Update size to use complete list if possible.
        if (found != NULL)
        {
            if (found->free_size < (totalsize + sizeof(struct mblock_free)))
            {
                totalsize = found->free_size;

                ASSERT_ERR_KE(found->previous != NULL);

                // update double linked list
                found->previous->next = found->next;
                if (found->next != NULL)
                {
                    found->next->previous = found->previous;
                }

                // compute the pointer to the beginning of the free space
#if CPU_WORD_SIZE == 4
                alloc = (struct mblock_used *)((uint32_t)found);
#elif CPU_WORD_SIZE == 2
                alloc = (struct mblock_used *)((uint16_t)found);
#endif
            }
            else
            {
                // found a free block that matches, subtract the allocation size from the
                // free block size. If equal, the free block will be kept with 0 size... but
                // moving it out of the linked list is too much work.
                found->free_size -= totalsize;

                // compute the pointer to the beginning of the free space
#if CPU_WORD_SIZE == 4
                alloc = (struct mblock_used *)((uint32_t)found + found->free_size);
#elif CPU_WORD_SIZE == 2
                alloc = (struct mblock_used *)((uint16_t)found + found->free_size);
#endif
            }

#if (TRACER_PRESENT && TRC_MEM)
            trc_heap_id = heap_id;
            TRC_REQ_MEM_ALLOC(trc_heap_id, alloc, size);
#endif /*(TRACER_PRESENT && TRC_MEM)*/

            // save the size of the allocated block
            alloc->size          = totalsize;
            alloc->corrupt_check = KE_ALLOCATED_PATTERN;
            alloc->heap_id       = heap_id;

            // move to the user memory space
            alloc++;

            DBG_DATA_ALLOC(alloc, filename, line);
            DBG_MEM_PERM_SET(alloc, size, true, true, true);

#if (SYSTEM_MEM_SAFETY_MARGIN_SIZE || KE_PROFILING)
            ke_env.total_heap_used    += totalsize;
#endif // (SYSTEM_MEM_SAFETY_MARGIN_SIZE || KE_PROFILING)

#if KE_PROFILING
            ke_env.heap_used[heap_id] += totalsize;
            if (ke_env.max_heap_used < ke_env.total_heap_used)
            {
                ke_env.max_heap_used = ke_env.total_heap_used;
            }
#endif //KE_PROFILING

            DBG_MEM_GRANT_CTRL(ke_env.heap[heap_id], false);
            break; // Nothing more to do, exit
        }

        // increment cursor and heap id
        cursor ++;
        CO_VAL_INC(heap_id, KE_MEM_BLOCK_MAX);
        DBG_MEM_GRANT_CTRL(ke_env.heap[heap_id], false);
    }

    // end of protection (as early as possible)
    GLOBAL_INT_RESTORE();

//    debug_mem_set((uint32_t*)alloc,type);
    DBG_SWDIAG(KE, MALLOC, 0);

#if !(PLF_MEM_LEAK_DETECT)
    DBG_FUNC_EXIT(ke_malloc_internal);
#endif // !(PLF_MEM_LEAK_DETECT)
    return (void *)alloc;
}
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void ke_mem_init(uint8_t type, uint8_t *heap, uint16_t heap_size)
{
    // align first free descriptor to word boundary
#if CPU_WORD_SIZE == 4
    ke_env.heap[type] = (struct mblock_free *)CO_ALIGN4_HI((uint32_t)heap);
#elif CPU_WORD_SIZE == 2
    ke_env.heap[type] = (struct mblock_free *)CO_ALIGN2_HI((uint16_t)heap);
#else
#error No word size defined
#endif // CPU_WORD_SIZE

    // protect accesses to descriptors
    GLOBAL_INT_DISABLE();
    DBG_MEM_GRANT_CTRL(heap, true);

    // initialize the first block
    //  + compute the size from the last aligned word before heap_end
    ke_env.heap[type]->free_size = ((uint32_t)&heap[heap_size] & (~3)) - (uint32_t)(ke_env.heap[type]);
    ke_env.heap[type]->corrupt_check = KE_LIST_PATTERN;
    ke_env.heap[type]->heap_id = type;
    ke_env.heap[type]->next = NULL;
    ke_env.heap[type]->previous = NULL;

    ke_env.heap_size[type] = heap_size;

#if (SYSTEM_MEM_SAFETY_MARGIN_SIZE)
    // add heap size to limit
    ke_env.user_mem_size_limit += heap_size;
#endif // (SYSTEM_MEM_SAFETY_MARGIN_SIZE)

//    dbg_mem.cpt = 0;

    // end of protection
    DBG_MEM_PERM_SET(heap, heap_size, false, false, true);
    DBG_MEM_GRANT_CTRL(heap, false);
    GLOBAL_INT_RESTORE();
}


bool ke_mem_is_memory_heap_pool_used(uint8_t type)
{
    bool is_memory_heap_pool_used = true;
    struct mblock_free *first;

    // select which memory block to check.
    uint8_t *block = (uint8_t *)ke_env.heap[type];
    // size of the current free block (including delimiter)
    uint32_t size = ke_env.heap_size[type];

    // align first free descriptor to word boundary
    first =  ke_env.heap[type];

    // protect accesses to descriptors
    GLOBAL_INT_DISABLE();
    DBG_MEM_GRANT_CTRL(first, true);

    // calculate size if whole memory block is free.
    size = ((uint32_t)&block[size] & (~3)) - (uint32_t)first;

    // compare it with available memory size of first block.
    if (first->free_size == size)
    {
        is_memory_heap_pool_used = false;
    }

    DBG_MEM_GRANT_CTRL(first, false);
    // end of protection
    GLOBAL_INT_RESTORE();

    // return result.
    return is_memory_heap_pool_used;
}

#if !(PLF_MEM_LEAK_DETECT)
    void *ke_malloc_system(uint32_t size, uint8_t type)
#else
    void *ke_malloc_system_with_mem_leak_detection(uint32_t size, uint8_t type, const char *filename, uint16_t line)
#endif // !(PLF_MEM_LEAK_DETECT)
{
    void *p_alloc = ke_malloc_internal(size, type);

    // Re-boot platform if no more empty space
    ASSERT_WARN(p_alloc != NULL, size, type);
    if (p_alloc == NULL)
    {
        platform_reset(RESET_MEM_ALLOC_FAIL);
    }
    else
    {
        struct mblock_used *alloc = ((struct mblock_used *)p_alloc) - 1;
#ifdef _MSC_VER
        alloc->ret_addr = (uint32_t) _ReturnAddress();
#else
        alloc->ret_addr = (uint32_t)__builtin_return_address(0);
#endif
    }

    return p_alloc;
}

#if !(PLF_MEM_LEAK_DETECT)
    void *ke_malloc_user(uint32_t size, uint8_t type)
#else
    void *ke_malloc_user_with_mem_leak_detection(uint32_t size, uint8_t type, const char *filename, uint16_t line)
#endif // !(PLF_MEM_LEAK_DETECT)
{
    void *p_alloc = NULL;

#if(SYSTEM_MEM_SAFETY_MARGIN_SIZE)
    ASSERT_ERR(ke_env.user_mem_size_limit > 0);
    // worst case, could be exceed by ~sizeof(mblock_free) bytes during allocation
    if ((ke_env.total_heap_used + (int32_t)size) <= ke_env.user_mem_size_limit)
#endif // (SYSTEM_MEM_SAFETY_MARGIN_SIZE)
    {
        p_alloc = ke_malloc_internal(size, type);
    }

    ASSERT_WARN(p_alloc != NULL, size, type);
    if (p_alloc != NULL)
    {
        struct mblock_used *alloc = ((struct mblock_used *)p_alloc) - 1;
#ifdef _MSC_VER
        alloc->ret_addr = (uint32_t) _ReturnAddress();
#else
        alloc->ret_addr = (uint32_t)__builtin_return_address(0);
#endif

    }

    return p_alloc;
}


void ke_free(void *mem_ptr)
{
    DBG_FUNC_ENTER(ke_free);
    struct mblock_free *freed;
    struct mblock_used *bfreed;
    struct mblock_free *node, *next_node, *prev_node;
    uint32_t size;
    uint8_t heap_id;
#if (TRACER_PRESENT && TRC_MEM)
    uint8_t trc_heap_id = 0;
#endif /*(TRACER_PRESENT && TRC_MEM)*/

    DBG_DATA_FREE(mem_ptr);

    DBG_SWDIAG(KE, FREE, 1);
    // sanity checks
    ASSERT_INFO_KE(mem_ptr != NULL, mem_ptr, 0);

//    debug_mem_reset((uint32_t*)mem_ptr);
    // point to the block descriptor (before user memory so decrement)
    bfreed = ((struct mblock_used *)mem_ptr) - 1;

    DBG_MEM_PERM_SET(bfreed, sizeof(struct mblock_used), true, true, false);

    // check if memory block has been corrupted or not
    ASSERT_INFO_KE(bfreed->corrupt_check == KE_ALLOCATED_PATTERN, bfreed->corrupt_check, mem_ptr);
    heap_id = bfreed->heap_id;

    // change corruption token in order to know if buffer has been already freed.
    bfreed->corrupt_check = KE_FREE_PATTERN;

    // point to the first node of the free elements linked list
    size = bfreed->size;
    node = NULL;

    freed = ((struct mblock_free *)bfreed);
    node = ke_env.heap[heap_id];

    DBG_MEM_PERM_SET(bfreed, sizeof(struct mblock_used), false, false, false);

    // protect accesses to descriptors
    GLOBAL_INT_DISABLE();
    DBG_MEM_GRANT_CTRL(mem_ptr, true);


    // sanity checks
    ASSERT_ERR_KE(bfreed->heap_id < KE_MEM_BLOCK_MAX);
    ASSERT_ERR_KE(((uint32_t)mem_ptr > (uint32_t)node));

    TRC_REQ_MEM_FREE(trc_heap_id, freed, size);
    DBG_MEM_PERM_SET(freed, size, false, false, false);

    prev_node = NULL;

    while (node != NULL)
    {
        ASSERT_ERR_KE(node->corrupt_check == KE_LIST_PATTERN);
        // check if the freed block is right after the current block
        if ((uint32_t)freed == ((uint32_t)node + node->free_size))
        {
            // append the freed block to the current one
            node->free_size += size;

            // check if this merge made the link between free blocks
            if (((uint32_t) node->next) == (((uint32_t)node) + node->free_size))
            {
                next_node = node->next;
                // add the size of the next node to the current node
                node->free_size += next_node->free_size;
                // update the next of the current node
                ASSERT_ERR_KE(next_node != NULL);
                node->next = next_node->next;
                // update linked list.
                if (next_node->next != NULL)
                {
                    next_node->next->previous = node;
                }
            }
            goto free_end;
        }
        else if ((uint32_t)freed < (uint32_t)node)
        {
            // sanity check: can not happen before first node
            ASSERT_ERR_KE(prev_node != NULL);

            // update the next pointer of the previous node
            prev_node->next = freed;
            freed->previous = prev_node;

            freed->corrupt_check = KE_LIST_PATTERN;

            // check if the released node is right before the free block
            if (((uint32_t)freed + size) == (uint32_t)node)
            {
                // merge the two nodes
                freed->next = node->next;
                if (node->next != NULL)
                {
                    node->next->previous = freed;
                }
                freed->free_size = node->free_size + size;
            }
            else
            {
                // insert the new node
                freed->next = node;
                node->previous = freed;
                freed->free_size = size;
            }
            goto free_end;
        }

        // move to the next free block node
        prev_node = node;
        node = node->next;

    }


    // if reached here, freed block is after last free block and not contiguous
    prev_node->next = (struct mblock_free *)freed;
    freed->next = NULL;
    freed->previous = prev_node;
    freed->free_size = size;
    freed->corrupt_check = KE_LIST_PATTERN;


free_end:
#if KE_PROFILING
    ke_env.heap_used[heap_id] -= size;
#endif //KE_PROFILING

#if (SYSTEM_MEM_SAFETY_MARGIN_SIZE || KE_PROFILING)
    ke_env.total_heap_used    -= size;
#endif // (SYSTEM_MEM_SAFETY_MARGIN_SIZE || KE_PROFILING)

    // end of protection
    DBG_MEM_GRANT_CTRL(mem_ptr, false);
    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(KE, FREE, 0);

    DBG_FUNC_EXIT(ke_free);
}


bool ke_is_free(void *mem_ptr)
{
    bool result;
    struct mblock_used *p_mem_info = ((struct mblock_used *)mem_ptr) - 1;

    DBG_MEM_PERM_SET(p_mem_info, sizeof(struct mblock_used), true, true, false);
    // use corrupt check info in order to know if pointer already free.
    result = (p_mem_info->corrupt_check != KE_ALLOCATED_PATTERN);
    DBG_MEM_PERM_SET(p_mem_info, sizeof(struct mblock_used), false, false, false);

    return (result);
}

bool ke_is_lower_than_target(uint8_t type, uint32_t target_size)
{
    // heap should init in the beginning.
    ASSERT_ERR_KE(ke_env.heap[type]);
    return (ke_env.heap[type]->free_size < target_size);
}

bool ke_is_lower_than_target_all(uint32_t target_size)
{
    uint32_t i;
    uint32_t left_size = 0;
    bool ret = 0;
    for (i = KE_MEM_ENV; i < KE_MEM_BLOCK_MAX; i++)
    {
        if (ke_env.heap[i])
            left_size += ke_env.heap[i]->free_size;
    }
    return ((left_size > 0) ? (left_size < target_size) : 0);
}


#if (KE_PROFILING)
uint16_t ke_get_mem_usage(uint8_t type)
{
    ASSERT_ERR(type < KE_MEM_BLOCK_MAX);

    return ke_env.heap_used[type];
}

uint32_t ke_get_max_mem_usage(void)
{
    uint32_t ret = ke_env.max_heap_used;

    // Cleared max measured value.
    ke_env.max_heap_used = 0;

    return ret;
}
#endif // (KE_PROFILING)

bool ke_check_malloc(uint32_t size, uint8_t type)
{
    void *p_buf = NULL;

    p_buf = ke_malloc_user(size, type);

    if (NULL == p_buf)
    {
        return false;
    }
    else
    {
        ke_free(p_buf);
        return true;
    }
}

///@} KE_MEM
