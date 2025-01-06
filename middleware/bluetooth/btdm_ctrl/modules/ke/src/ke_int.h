/**
 ****************************************************************************************
 *
 * @file ke_int.h
 *
 * @brief This file contains the definition of the kernel. - Internals
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _KE_ENV_H_
#define _KE_ENV_H_

/// @addtogroup KERNEL
/// @{

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"          // stack configuration
#include "ke_event.h"             // kernel event
#include "co_list.h"              // kernel queue definition

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

// forward declaration
struct mblock_free;

/// Kernel environment definition
struct ke_env_tag
{
    /// Queue of sent messages but not yet delivered to receiver
    struct co_list queue_sent;
    /// Queue of messages delivered but not consumed by receiver
    struct co_list queue_saved;
    /// Queue of timers
    struct co_list queue_timer;
    /// Root pointer = pointer to first element of heap linked lists
    struct mblock_free *heap[KE_MEM_BLOCK_MAX];
    /// Size of heaps
    uint16_t heap_size[KE_MEM_BLOCK_MAX];

#if (KE_PROFILING)
    /// Size of heap used
    uint16_t heap_used[KE_MEM_BLOCK_MAX];
    /// Maximum heap memory used
    int32_t  max_heap_used;
#endif //KE_PROFILING

#if (SYSTEM_MEM_SAFETY_MARGIN_SIZE || KE_PROFILING)
    /// Total heap used
    int32_t  total_heap_used;
#endif // (SYSTEM_MEM_SAFETY_MARGIN_SIZE || KE_PROFILING)
#if (SYSTEM_MEM_SAFETY_MARGIN_SIZE)
    /// Maximum memory size for user space allocation
    int32_t  user_mem_size_limit;
#endif // (SYSTEM_MEM_SAFETY_MARGIN_SIZE)
};

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Kernel environment
extern struct ke_env_tag ke_env;


/*
 * FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if current pointer is free or not
 *
 * @param[in] mem_ptr pointer to a memory block
 *
 * @return true if already free, false else.
 ****************************************************************************************
 */
bool ke_is_free(void *mem_ptr);

/// @} KERNAL

#endif // _KE_ENV_H_
