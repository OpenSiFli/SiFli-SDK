/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file co_dlist.c
 *
 * Copyright (C) RivieraWaves 2011-2019
 *
 * @brief Double linked list management functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CO_DLIST
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "co_dlist.h"

// for assertions
#include "dbg_assert.h"


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void co_dlist_init(struct co_dlist *list)
{
    list->first = NULL;
    list->last = NULL;
    list->cnt = 0;

    #if NX_DLIST_DEBUG
    list->maxcnt = 0;
    list->mincnt = 0xFFFFFFFF;
    #endif // NX_DLIST_DEBUG
}

void co_dlist_push_back(struct co_dlist *list, struct co_dlist_hdr *list_hdr)
{

    // check if the list is empty
    if (co_dlist_is_empty(list))
    {
        // list is empty => update the first because the added one is also the first
        list->first = list_hdr;
    }
    else
    {
        // list not empty => just update the last
        list->last->next = list_hdr;
    }

    // add the element at the back of the list
    list_hdr->prev = list->last;
    list->last = list_hdr;
    list_hdr->next = NULL;
    list->cnt++;

    #if NX_DLIST_DEBUG
    if(list->maxcnt < list->cnt)
    {
        list->maxcnt = list->cnt;
    }
    #endif // NX_DLIST_DEBUG
}

void co_dlist_push_front(struct co_dlist *list,struct co_dlist_hdr *list_hdr)
{
    // check if list is empty
    if (co_dlist_is_empty(list))
    {
        // list is empty => update the last because the added one is also the last
        list->last = list_hdr;
    }
    else
    {
        // list is empty => just update the first
        list->first->prev = list_hdr;
    }

    // add the element at the front of the list
    list_hdr->next = list->first;
    list->first = list_hdr;
    list_hdr->prev = NULL;
    list->cnt++;

    #if NX_DLIST_DEBUG
    if(list->maxcnt < list->cnt)
    {
        list->maxcnt = list->cnt;
    }
    #endif // NX_DLIST_DEBUG
}

struct co_dlist_hdr *co_dlist_pop_front(struct co_dlist *list)
{
    struct co_dlist_hdr *element;

    element = list->first;
    if ( element != NULL )
    {
        // The list isn't empty : extract the first element
        list->cnt--;
        list->first = list->first->next;

        // check if list is empty after popping
        if (!co_dlist_is_empty(list))
        {
            list->first->prev = NULL;
        }
        else
        {
            // since list is empty, clear the last
            list->last = NULL;
        }

        #if NX_DLIST_DEBUG
        if(list->mincnt > list->cnt)
        {
            list->mincnt = list->cnt;
        }
        #endif // NX_DLIST_DEBUG
    }
    return element;
}

void co_dlist_extract(struct co_dlist *list, struct co_dlist_hdr const *list_hdr)
{
    struct co_dlist_hdr *first;

    // retrieve the first pointer
    first = list->first;

    // sanity check: should never call extract on empty list
    AIC_ASSERT_ERR(first != NULL);

    // check if first is the searched element
    if (first == list_hdr)
    {
        // Extract first element
        list->first = first->next;

        // check if list is empty after extracting
        if (!co_dlist_is_empty(list))
        {
            list->first->prev = NULL;
        }
        else
        {
            // since list is empty, clear the last
            list->last = NULL;
        }
    }
    else
    {
        #if NX_DLIST_DEBUG
        struct co_dlist_hdr *scan_list;
        // verify that the element is in the list (for debugging only)
        // Look for the element in the list
        scan_list = list->first;
        while (scan_list->next != list_hdr)    // Element not found
        {
            scan_list = scan_list->next;
            AIC_ASSERT_ERR(scan_list != NULL);  // element is not in the list !
        }
        // end of : verify that the element is in the list (for debugging only)
        #endif // NX_DLIST_DEBUG

        // re move the element by linking its next and previous elements
        list_hdr->prev->next = list_hdr->next;

        // check if the searched element is the last
        if(list->last == list_hdr)
        {
            // the searched is the last element => update the last element with the previous
            list->last = list_hdr->prev;
        }
        else
        {
            // the searched is not the last element => update the next's previous
            list_hdr->next->prev = list_hdr->prev;
        }
    }

    list->cnt--;

    #if NX_DLIST_DEBUG
    if(list->mincnt > list->cnt)
    {
        list->mincnt = list->cnt;
    }
    #endif // NX_DLIST_DEBUG
}

/// @}
