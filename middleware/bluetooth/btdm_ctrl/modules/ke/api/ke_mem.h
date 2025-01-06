/**
 ****************************************************************************************
 *
 * @file ke_mem.h
 *
 * @brief API for the heap management module.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _KE_MEM_H_
#define _KE_MEM_H_

#include "rwip_config.h"     // IP configuration
#include <stdint.h>          // standard integer
#include <stdbool.h>         // standard includes

/**
 ****************************************************************************************
 * @defgroup MEM Memory
 * @ingroup KERNEL
 * @brief Heap management module.
 *
 * This module implements heap management functions that allow initializing heap,
 * allocating and freeing memory.
 *
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Heap initialization.
 *
 * This function performs the following operations:
 * - sanity checks
 * - check memory allocated is at least large enough to hold two block descriptors to hold
 * start and end
 * - initialize the first and last descriptors
 * - save heap into kernel environment variable.
 *
 * @param[in]     type      Memory type.
 * @param[in|out] heap      Heap pointer
 * @param[in]     heap_size Size of the heap
 *
 *
 ****************************************************************************************
 */
void ke_mem_init(uint8_t type, uint8_t *heap, uint16_t heap_size);

#if !(PLF_MEM_LEAK_DETECT)
    /**
    ****************************************************************************************
    * @brief Allocation of a block of memory.
    *
    * Allocates a memory block whose size is size; if no memory is available return NULL
    *
    * @param[in] size Size of the memory area that need to be allocated.
    * @param[in] type Type of memory block
    *
    * @return A pointer to the allocated memory area.
    *
    ****************************************************************************************
    */
    void *ke_malloc_system(uint32_t size, uint8_t type);
#else
    /// Allocation of a block of memory with memory leak detection
    void *ke_malloc_system_with_mem_leak_detection(uint32_t size, uint8_t type, const char *filename, uint16_t line);
    /// Update use of ke_malloc function
    #define ke_malloc_system(size, type)  ke_malloc_system_with_mem_leak_detection((size), (type), __FILE__, __LINE__)
#endif // !(PLF_MEM_LEAK_DETECT)

#if !(PLF_MEM_LEAK_DETECT)
    /**
    ****************************************************************************************
    * @brief Allocation of a block of memory with a runtime limit
    * If runtime limit is reached, no more memory is allocated
    *
    * A pointer allocated with runtime limit shall be free with #ke_free_with_runtime_limit
    *
    * If no memory is available or runtime limit reach, return NULL
    *
    * @param[in] size Size of the memory area that need to be allocated.
    * @param[in] type Type of memory block
    *
    * @return A pointer to the allocated memory area.
    ****************************************************************************************
    */
    void *ke_malloc_user(uint32_t size, uint8_t type);
#else
    /// Allocation of a block of memory with memory leak detection
    void *ke_malloc_user_with_mem_leak_detection(uint32_t size, uint8_t type, const char *filename, uint16_t line);
    /// Update use of ke_malloc function
    #define ke_malloc_user(size, type)  ke_malloc_user_with_mem_leak_detection((size), (type), __FILE__, __LINE__)
#endif // (!PLF_MEM_LEAK_DETECT)

#define ke_malloc(size, type)   ke_malloc_system(size, type)

/**
 ****************************************************************************************
 * @brief Freeing of a block of memory.
 *
 * Free the memory area pointed by mem_ptr : mark the block as free and insert it in
 * the pool of free block.
 *
 * @param[in] mem_ptr Pointer to the memory area that need to be freed.
 *
 ****************************************************************************************
 */
void ke_free(void *mem_ptr);

/**
 ****************************************************************************************
 * @brief Check if current heap memory pool is used (if some memory buffer allocated)
 *
 * @note This function can be used in sleep mode to decide if a memory block can be turned off.
 *  *
 * @param[in] type Type of memory heap block (see enum #KE_MEM_HEAP)
 *
 * @return True if heap not used, false else.
 ****************************************************************************************
 */
bool ke_mem_is_memory_heap_pool_used(uint8_t type);



// To check if dedicated type is enough.
bool ke_is_lower_than_target(uint8_t type, uint32_t target_size);

bool ke_is_lower_than_target_all(uint32_t target_size);

#if (KE_PROFILING)

    /**
    ****************************************************************************************
    * @brief Retrieve memory usage of selected heap.
    *
    * @param[in] type Type of memory heap block
    *
    * @return current memory usage of current heap.
    ****************************************************************************************
    */
    uint16_t ke_get_mem_usage(uint8_t type);


    /**
    ****************************************************************************************
    * @brief Retrieve max memory usage of all heap.
    * This command also resets max measured value.
    *
    * @return max memory usage of all heap.
    ****************************************************************************************
    */
    uint32_t ke_get_max_mem_usage(void);

#endif // (KE_PROFILING)

bool ke_check_malloc(uint32_t size, uint8_t type);

///@} MEM

#endif // _KE_MEM_H_

