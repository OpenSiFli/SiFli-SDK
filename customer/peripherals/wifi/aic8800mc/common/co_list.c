/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 ****************************************************************************************
 * @addtogroup CO_LIST
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
// for mem* functions
#include <string.h>

#include "co_list.h"

// for assertions
#include "osal_debug.h"


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void co_list_init(struct co_list *list)
{
    list->first = NULL;
    list->last = NULL;
}

void co_list_pool_init(struct co_list *list,
                       void *pool,
                       size_t elmt_size,
                       uint32_t elmt_cnt,
                       void *default_value)
{
    uint32_t i;

    // initialize the free list relative to the pool
    co_list_init(list);

    // Add each element of the pool to this list, and init them one by one
    for (i = 0; i < elmt_cnt; i++)
    {
        if (default_value)
        {
            memcpy(pool, default_value, elmt_size);
        }
        co_list_push_back(list, (struct co_list_hdr *) pool);

        // move to the next pool element
        pool = (void *)((uint8_t *)pool + (uint32_t)elmt_size);
    }
}

void co_list_push_back(struct co_list *list,
                       struct co_list_hdr *list_hdr)
{
    // Sanity check
    ASSERT_ERR(list_hdr != NULL);

    // check if list is empty
    if (co_list_is_empty(list))
    {
        // list empty => pushed element is also head
        list->first = list_hdr;
    }
    else
    {
        // list not empty => update next of last
        list->last->next = list_hdr;
    }

    // add element at the end of the list
    list->last = list_hdr;
    list_hdr->next = NULL;
}

void co_list_push_front(struct co_list *list,
                        struct co_list_hdr *list_hdr)
{
    // Sanity check
    ASSERT_ERR(list_hdr != NULL);

    // check if list is empty
    if (co_list_is_empty(list))
    {
        // list empty => pushed element is also head
        list->last = list_hdr;
    }

    // add element at the beginning of the list
    list_hdr->next = list->first;
    list->first = list_hdr;
}

struct co_list_hdr *co_list_pop_front(struct co_list *list)
{
    struct co_list_hdr *element;

    // check if list is empty
    element = list->first;
    if (element != NULL)
    {
        // The list isn't empty : extract the first element
        list->first = list->first->next;
        if (list->last == element)
            list->last = NULL;
    }

    return element;
}

void co_list_extract(struct co_list *list,
                     struct co_list_hdr *list_hdr)
{
    struct co_list_hdr *scan_list;

    // sanity check
    ASSERT_ERR(list != NULL);

    scan_list = list->first;

    // Check if list is empty or not
    if (scan_list == NULL)
        return;

    // check if searched element is first
    if (scan_list == list_hdr)
    {
        // Extract first element
        list->first = scan_list->next;
        if (list->last == scan_list)
            list->last = NULL;
    }
    else
    {
        // Look for the element in the list
        while ((scan_list->next != NULL) && (scan_list->next != list_hdr))
        {
            scan_list = scan_list->next;
        }

        // Check if element was found in the list
        if (scan_list->next != NULL)
        {
            // check if the removed element is the last in the list
            if (list->last == list_hdr)
            {
                // Last element will be extracted
                list->last = scan_list;
            }
            // Extract the element from the list
            scan_list->next = list_hdr->next;
        }
    }
}

bool co_list_find(struct co_list *list,
                  struct co_list_hdr *list_hdr)
{
    struct co_list_hdr *tmp_list_hdr;

    // Go through the list to find the element
    tmp_list_hdr = list->first;
    while ((tmp_list_hdr != list_hdr) && (tmp_list_hdr != NULL))
    {
        tmp_list_hdr = tmp_list_hdr->next;
    }

    return (tmp_list_hdr == list_hdr);
}

uint32_t co_list_cnt(const struct co_list *const list)
{
    uint32_t cnt = 0;
    struct co_list_hdr *elt = co_list_pick(list);

    // Go through the list to count the number of elements
    while (elt != NULL)
    {
        cnt++;
        elt = co_list_next(elt);
    }

    return (cnt);
}

/**
 ****************************************************************************************
 * @brief Insert an element in a sorted list.
 *
 * This primitive use a comparison function from the parameter list to select where the
 * element must be inserted.
 *
 * @param[in]  list     Pointer to the list.
 * @param[in]  element  Pointer to the element to insert.
 * @param[in]  cmp      Comparison function (return true if first element has to be inserted
 *                      before the second one).
 *
 * @return              Pointer to the element found and removed (NULL otherwise).
 ****************************************************************************************
 */
void co_list_insert(struct co_list *const list,
                    struct co_list_hdr *const element,
                    bool (*cmp)(struct co_list_hdr const *elementA,
                                struct co_list_hdr const *elementB))
{
    struct co_list_hdr *prev = NULL;
    struct co_list_hdr *scan = list->first;

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
            list->last = element;
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
        list->first = element;
    }
}

void co_list_insert_after(struct co_list *const list,
                          struct co_list_hdr *const prev_element,
                          struct co_list_hdr *const element)
{
    struct co_list_hdr *scan = list->first;

    if (prev_element == NULL)
    {
        // Insert the element in front on the list
        co_list_push_front(list, element);
    }
    else
    {
        // Look for prev_element in the list
        while (scan)
        {
            if (scan == prev_element)
            {
                break;
            }

            // Get next element
            scan = scan->next;
        }

        // If prev_element has been found, insert element
        if (scan)
        {
            element->next = prev_element->next;
            prev_element->next = element;

            if (list->last == prev_element)
            {
                list->last = element;
            }
        }
    }
}

void co_list_insert_before(struct co_list *const list,
                           struct co_list_hdr *const next_element,
                           struct co_list_hdr *const element)
{
    if (next_element == NULL)
    {
        // Insert the element at the end of the list
        co_list_push_back(list, element);
    }
    else if (next_element == list->first)
    {
        // Insert the element in front of the list
        co_list_push_front(list, element);
    }
    else
    {
        struct co_list_hdr *scan = list->first;

        // Look for next_element in the list
        while (scan)
        {
            if (scan->next == next_element)
            {
                break;
            }

            // Get next element
            scan = scan->next;
        }

        // Insert element after scan
        if (scan)
        {
            element->next = next_element;
            scan->next = element;
        }
    }
}

void co_list_concat(struct co_list *list1, struct co_list *list2)
{
    // If list2 is empty, don't do anything
    if (list2->first != NULL)
    {
        // Check if list1 is empty
        if (list1->first == NULL)
        {
            // If list1 is empty, list1 becomes list2
            *list1 = *list2;
        }
        else
        {
            // Otherwise, append list2 to list1
            list1->last->next = list2->first;
            list1->last = list2->last;
        }
        // Clear list2
        list2->first = NULL;
    }
}


void co_list_remove(struct co_list *list,
                    struct co_list_hdr *prev_element,
                    struct co_list_hdr *element)
{
    // sanity check
    ASSERT_ERR(list != NULL);
    ASSERT_ERR((prev_element == NULL) || (prev_element->next == element));
    ASSERT_ERR(element != NULL);


    if (prev_element == NULL)
    {
        list->first = element->next;
    }
    else
    {
        prev_element->next = element->next;
    }
    if (list->last == element)
        list->last = prev_element;

    element->next = NULL;
}

uint16_t co_list_size(struct co_list *list)
{
    uint16_t count = 0;
    struct co_list_hdr *tmp_list_hdr = list->first;

    // browse list to count number of elements
    while (tmp_list_hdr != NULL)
    {
        tmp_list_hdr = tmp_list_hdr->next;
        count++;
    }

    return count;
}

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/// Doubly-linked list

void co_dl_list_init(list_node_t *list)
{
    list->prev = list->next = list;
}

void co_dl_list_insert_tail(list_node_t *list, list_node_t *node)
{
    node->prev = list;
    node->next = list->next;
    list->next->prev = node;
    list->next = node;
}

void co_dl_list_insert_head(list_node_t *list, list_node_t *node)
{
    node->prev = list->prev;
    node->next = list;
    list->prev->next = node;
    list->prev = node;
}

void co_dl_list_remove_node(list_node_t *node)
{
    list_node_t *temp_node;

    temp_node = node->next;
    node->prev->next = temp_node;
    temp_node->prev = node->prev;

    node->next = node->prev = NULL;
}

list_node_t *co_dl_list_remove_node_and_get_next(list_node_t *node)
{
    list_node_t *second;

    second = node->next;
    node->prev->next = second;
    second->prev = node->prev;

    node->next = node->prev = NULL;
    return (second);
}

list_node_t *co_dl_list_remove_list_head(list_node_t *list)
{
    list_node_t *first;

    first = list->prev;
    first->prev->next = list;
    list->prev = first->prev;

    first->next = first->prev = NULL;
    return (first);
}

bool co_dl_list_is_empty(list_node_t *list)
{
    return ((list->prev) == list);
}

/// @}
