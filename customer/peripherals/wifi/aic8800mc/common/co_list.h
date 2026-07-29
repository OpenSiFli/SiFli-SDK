/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
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

#ifndef _CO_LIST_H_
#define _CO_LIST_H_

/**
 ****************************************************************************************
 * @defgroup CO_LIST CO_LIST
 * @ingroup COMMON
 * @brief  List management.
 *
 * This module contains the list structures and handling functions.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "co_int.h"
#include "co_bool.h"
// for NULL and size_t

// for __STATIC_INLINE
#include "compiler.h"

#define co_list_init            aicwf_co_list_init
#define co_list_pool_init       aicwf_co_list_pool_init
#define co_list_push_back       aicwf_co_list_push_back
#define co_list_push_front      aicwf_co_list_push_front
#define co_list_pop_front       aicwf_co_list_pop_front
#define co_list_extract         aicwf_co_list_extract
#define co_list_find            aicwf_co_list_find
#define co_list_insert_after    aicwf_co_list_insert_after
#define co_list_insert_before   aicwf_co_list_insert_before
#define co_list_size            aicwf_co_list_size

/*
 * STRUCTURE DECLARATIONS
 ****************************************************************************************
 */
/// structure of a list element header
struct co_list_hdr
{
    /// Pointer to the next element in the list
    struct co_list_hdr *next;
};

/// structure of a list
struct co_list
{
    /// pointer to first element of the list
    struct co_list_hdr *first;
    /// pointer to the last element
    struct co_list_hdr *last;
};

///co Doubly-linked list node

typedef struct list_node_t
{
    struct list_node_t *prev;
    struct list_node_t *next;
    void *data;
} list_node_t;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialize a list to defaults values.
 * @param[in] list           Pointer to the list structure.
 ****************************************************************************************
 */
void co_list_init(struct co_list *list);

/**
 ****************************************************************************************
 * @brief Initialize a pool to default values, and initialize the relative free list.
 *
 * @param[in] list           Pointer to the list structure
 * @param[in] pool           Pointer to the pool to be initialized
 * @param[in] elmt_size      Size of one element of the pool
 * @param[in] elmt_cnt       Nb of elements available in the pool
 * @param[in] default_value  Pointer to the default value of each element (may be NULL)
 ****************************************************************************************
 */
void co_list_pool_init(struct co_list *list,
                       void *pool,
                       size_t elmt_size,
                       uint32_t elmt_cnt,
                       void *default_value);

/**
 ****************************************************************************************
 * @brief Add an element as last on the list.
 *
 * @param[in] list           Pointer to the list structure
 * @param[in] list_hdr       Pointer to the header to add at the end of the list
 ****************************************************************************************
 */
void co_list_push_back(struct co_list *list,
                       struct co_list_hdr *list_hdr);

/**
 ****************************************************************************************
 * @brief Add an element as first on the list.
 *
 * @param[in] list           Pointer to the list structure
 * @param[in] list_hdr       Pointer to the header to add at the beginning of the list
 ****************************************************************************************
 */
void co_list_push_front(struct co_list *list,
                        struct co_list_hdr *list_hdr);
/**
 ****************************************************************************************
 * @brief Extract the first element of the list.
 *
 * @param[in] list           Pointer to the list structure
 *
 * @return The pointer to the element extracted, and NULL if the list is empty.
 ****************************************************************************************
 */
struct co_list_hdr *co_list_pop_front(struct co_list *list);

/**
 ****************************************************************************************
 * @brief Search for a given element in the list, and extract it if found.
 *
 * @param[in] list           Pointer to the list structure
 * @param[in] list_hdr       Pointer to the searched element
 *
 * @return CO_EMPTY if the list is empty, CO_FAIL if the element not found in the list,
 * CO_OK else.
 ****************************************************************************************
 */
void co_list_extract(struct co_list *list,
                     struct co_list_hdr *list_hdr);

/**
 ****************************************************************************************
 * @brief Searched a given element in the list.
 *
 * @param[in] list           Pointer to the list structure
 * @param[in] list_hdr       Pointer to the searched element
 *
 * @return true if the element is found in the list, false otherwise
 ****************************************************************************************
 */
bool co_list_find(struct co_list *list,
                  struct co_list_hdr *list_hdr);

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
                                struct co_list_hdr const *elementB));

/**
 ****************************************************************************************
 * @brief Insert an element in a sorted list after the provided element.
 *
 * This primitive use a comparison function from the parameter list to select where the
 * element must be inserted.
 *
 * @param[in]  list           Pointer to the list.
 * @param[in]  prev_element   Pointer to the element to find in the list
 * @param[in]  element        Pointer to the element to insert.
 *
 * If prev_element is not found, the provided element is not inserted
 ****************************************************************************************
 */
void co_list_insert_after(struct co_list *const list,
                          struct co_list_hdr *const prev_element,
                          struct co_list_hdr *const element);

/**
 ****************************************************************************************
 * @brief Insert an element in a sorted list before the provided element.
 *
 * This primitive use a comparison function from the parameter list to select where the
 * element must be inserted.
 *
 * @param[in]  list           Pointer to the list.
 * @param[in]  next_element   Pointer to the element to find in the list
 * @param[in]  element        Pointer to the element to insert.
 *
 * If next_element is not found, the provided element is not inserted
 ****************************************************************************************
 */
void co_list_insert_before(struct co_list *const list,
                           struct co_list_hdr *const next_element,
                           struct co_list_hdr *const element);

/**
 ****************************************************************************************
 * @brief Concatenate two lists.
 * The resulting list is the list passed as the first parameter. The second list is
 * emptied.
 *
 * @param[in]  list1          First list (will get the result of the concatenation)
 * @param[in]  list2          Second list (will be emptied after the concatenation)
 ****************************************************************************************
 */
void co_list_concat(struct co_list *list1, struct co_list *list2);

/**
 ****************************************************************************************
 * @brief Remove the element in the list after the provided element.
 *
 * This primitive removes an element in the list. It is assume that element is part of
 * the list.
 *
 * @param[in] list          Pointer to the list.
 * @param[in] prev_element  Pointer to the previous element.
 *                          NULL if @p element is the first element in the list
 * @param[in] element       Pointer to the element to remove.
 *
 ****************************************************************************************
 */
void co_list_remove(struct co_list *list,
                    struct co_list_hdr *prev_element,
                    struct co_list_hdr *element);

/**
 ****************************************************************************************
 * @brief Count number of elements present in the list
 *
 * @param list           Pointer to the list structure
 *
 * @return Number of elements present in the list
 ****************************************************************************************
 */
uint16_t co_list_size(struct co_list *list);

/**
 ****************************************************************************************
 * @brief Test if the list is empty.
 *
 * @param[in] list           Pointer to the list structure.
 *
 * @return true if the list is empty, false else otherwise.
 ****************************************************************************************
 */
__STATIC_INLINE bool co_list_is_empty(const struct co_list *const list)
{
    bool listempty;
    listempty = (list->first == NULL);
    return (listempty);
}

/**
 ****************************************************************************************
 * @brief Return the number of element of the list.
 *
 * @param[in] list           Pointer to the list structure.
 *
 * @return The number of elements in the list.
 ****************************************************************************************
 */
uint32_t co_list_cnt(const struct co_list *const list);

/**
 ****************************************************************************************
 * @brief Pick the first element from the list without removing it.
 *
 * @param[in] list           Pointer to the list structure.
 *
 * @return First element address. Returns NULL pointer if the list is empty.
 ****************************************************************************************
 */
__STATIC_INLINE struct co_list_hdr *co_list_pick(const struct co_list *const list)
{
    return (list->first);
}

/**
 ****************************************************************************************
 * @brief Pick the last element from the list without removing it.
 *
 * @param[in] list           Pointer to the list structure.
 *
 * @return Last element address. Returns invalid value if the list is empty.
 ****************************************************************************************
 */
__STATIC_INLINE struct co_list_hdr *co_list_pick_last(const struct co_list *const list)
{
    return (list->last);
}

/**
 ****************************************************************************************
 * @brief Return following element of a list element.
 *
 * @param[in] list_hdr     Pointer to the list element.
 *
 * @return The pointer to the next element.
 ****************************************************************************************
 */
__STATIC_INLINE struct co_list_hdr *co_list_next(const struct co_list_hdr *const list_hdr)
{
    return (list_hdr->next);
}


/// @} end of group COLIST

/**
 ****************************************************************************************
 * @brief * Doubly-linked list initialize .
 *
 * @param Doubly-linked list Entry.
 *
 ****************************************************************************************
 */
void co_dl_list_init(list_node_t *list);
/**
 ****************************************************************************************
 * @brief * Doubly-linked list insert node in Tail .
 *
 * @param  tail: Doubly-linked list.   entry: node which want to be inserted in tail of Doubly-linked list
 *
 ****************************************************************************************
 */
void co_dl_list_insert_tail(list_node_t *list, list_node_t *node);
/**
 ****************************************************************************************
 * @brief * Doubly-linked list insert node in Head .
 *
 * @param  head: Doubly-linked list.   entry: node which want to be inserted in head of Doubly-linked list
 *
 ****************************************************************************************
 */
void co_dl_list_insert_head(list_node_t *list, list_node_t *node);
/**
 ****************************************************************************************
 * @brief * remove current Node form a Doubly-linked list .
 *
 * @param   Node.
 *
 ****************************************************************************************
 */
void co_dl_list_remove_node(list_node_t *node);
/**
 ****************************************************************************************
 * @brief * remove current Node form a Doubly-linked list ,and return next Node's ptr .
 *
 * @param
 *
 * @return  Node which is next after removed Node.
 ****************************************************************************************
 */
list_node_t *co_dl_list_remove_node_and_get_next(list_node_t *node);
/**
 ****************************************************************************************
 * @brief * Get and remove Node form the Head of a Doubly-linked list .
 *
 * @param Doubly-linked list.
 *
 * @return Head of a Doubly-linked list
 ****************************************************************************************
 */
list_node_t *co_dl_list_remove_list_head(list_node_t *list);
/**
 ****************************************************************************************
 * @brief * Doubly-linked list is Empty .
 *
 * @param Doubly-linked list.
 *
 * @return true: empty  false: not empty
 ****************************************************************************************
 */
bool co_dl_list_is_empty(list_node_t *list);

#endif // _CO_LIST_H_
