/**
 ****************************************************************************************
 *
 * @file ke_queue.c
 *
 * @brief This file contains all the functions that handle the different queues
 * (timer queue, save queue, user queue)
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup QUEUE
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>        // standard definitions
#include <stdint.h>        // standard integer
#include <stdbool.h>       // standard boolean
#include "arch.h"          // architectural definitions
#include "ke_queue.h"      // kernel queues
#include "ke_mem.h"        // kernel memory definitions

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
struct co_list_hdr *ke_queue_extract(struct co_list *const queue,
                                     bool (*func)(struct co_list_hdr const *elmt, uint32_t arg), uint32_t arg)
{
    struct co_list_hdr *prev = NULL;
    struct co_list_hdr *element  = queue->first;

    while (element)
    {
        if (func(element, arg))
        {
            if (prev)
            {
                // second or more
                prev->next = element->next;
            }
            else
            {
                // first message
                queue->first = element->next;
            }

            if (element->next)
            {
                // not the last
                element->next = NULL;
            }
            else
            {
                // last message
                queue->last = prev;
            }

            break;
        }

        prev = element;
        element = element->next;
    }

#if (KE_PROFILING)
    if (element != NULL)
    {
        queue->cnt--;
        if (queue->mincnt > queue->cnt)
        {
            queue->mincnt = queue->cnt;
        }
    }
#endif //KE_PROFILING

    return element;
}

void ke_queue_insert(struct co_list *const queue, struct co_list_hdr *const element,
                     bool (*cmp)(struct co_list_hdr const *elementA, struct co_list_hdr const *elementB))
{
    struct co_list_hdr *prev = NULL;
    struct co_list_hdr *scan = queue->first;

    for (;;)
    {
        // scan the list until the end or cmp() returns true
        if (scan)
        {
            if (cmp(element, scan))
            {
                // insert now
                break;
            }
            prev = scan;
            scan = scan->next;
        }
        else
        {
            // end of list
            queue->last = element;
            break;
        }
    }

    element->next = scan;

    if (prev)
    {
        // second or more
        prev->next = element;
    }
    else
    {
        // first message
        queue->first = element;
    }

#if (KE_PROFILING)
    queue->cnt++;
    if (queue->maxcnt < queue->cnt)
    {
        queue->maxcnt = queue->cnt;
    }
#endif //KE_PROFILING
}

/// @} QUEUE
