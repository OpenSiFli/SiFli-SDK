/**
 ****************************************************************************************
 *
 * @file ke.c
 *
 * @brief This file contains the kernel definition.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup KE
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"        // stack configuration

#include "arch.h"               // architecture
#include "ke.h"                 // kernel
#include "ke_int.h"             // kernel environment
#include "ke_mem.h"             // kernel memory
#include "ke_msg.h"             // kernel message
#include "ke_event.h"           // kernel event
#include "ke_queue.h"           // kernel queue
#include "ke_task.h"            // kernel task
#include "ke_timer.h"           // kernel timer
#include "ke_lp.h"              // kernel low power
#include <string.h>             // used for memset.


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Kernel environment
struct ke_env_tag ke_env;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void ke_init(void)
{
    // ke_mem_init MUST be called first to be able to allocate memory right from start
    memset(ke_env.heap, 0, sizeof(struct mblock_free *) * KE_MEM_BLOCK_MAX);
    memset(ke_env.heap_size, 0, sizeof(uint16_t) * KE_MEM_BLOCK_MAX);
#if (KE_PROFILING)
    ke_env.max_heap_used = 0;
    memset(ke_env.heap_used, 0, sizeof(uint16_t) * KE_MEM_BLOCK_MAX);
#endif //KE_PROFILING

#if (SYSTEM_MEM_SAFETY_MARGIN_SIZE || KE_PROFILING)
    ke_env.total_heap_used = 0;
#endif // (SYSTEM_MEM_SAFETY_MARGIN_SIZE || KE_PROFILING)

#if (SYSTEM_MEM_SAFETY_MARGIN_SIZE)
    ke_env.user_mem_size_limit = 0 - (SYSTEM_MEM_SAFETY_MARGIN_SIZE); // start with a negative value
#endif // (SYSTEM_MEM_SAFETY_MARGIN_SIZE)

    // Initialize event module
    ke_event_init();

    // initialize the kernel message queue, mandatory before any message can be transmitted
    ke_env.queue_saved.first = NULL;
    ke_env.queue_saved.last = NULL;
    ke_env.queue_sent.first = NULL;
    ke_env.queue_sent.last = NULL;
    ke_env.queue_timer.first = NULL;
    ke_env.queue_timer.last = NULL;

    // Job queue to execute
    ke_task_init();

#if defined(BF0_LCPU)
    // Initialize hook for sleep trigger
    ke_sleep_os_hook();
#endif // RT_USING_PM
}

void ke_flush(void)
{
    // free all pending message(s)
    while (1)
    {
        struct ke_msg *msg = (struct ke_msg *) ke_queue_pop(&ke_env.queue_sent);
        if (msg == NULL)
            break;
        ke_msg_free(msg);
    }
    // free all saved message(s)
    while (1)
    {
        struct ke_msg *msg = (struct ke_msg *) ke_queue_pop(&ke_env.queue_saved);
        if (msg == NULL)
            break;
        ke_msg_free(msg);
    }
}

bool ke_sleep_check(void)
{
    return (ke_event_get_all() == 0);
}

#if (KE_PROFILING)
enum KE_STATUS ke_stats_get(uint32_t *max_msg_sent, uint32_t *max_msg_saved, uint32_t *max_timer_used,
                            uint32_t *max_heap_used)
{
    *max_msg_sent   = ke_env.queue_sent.maxcnt;
    *max_msg_saved  = ke_env.queue_saved.maxcnt;
    *max_timer_used = ke_env.queue_timer.maxcnt;
    *max_heap_used  = ke_env.max_heap_used;

    return KE_SUCCESS;
}
#endif //KE_PROFILING

///@} KE
