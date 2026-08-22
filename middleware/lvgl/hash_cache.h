/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HASH_CACHE_H
#define HASH_CACHE_H

#include <stdint.h>
#include "rtdef.h"

#ifndef LV_IMG_CF_JPG
#define LV_IMG_CF_JPG LV_IMG_CF_RAW
#endif

typedef void *(*hash_alloc_fn)(rt_size_t size);
typedef void (*hash_free_fn)(void *ptr);
typedef int (*hash_match_fn)(void *key1, void *key2);
typedef void (*hash_save_fn)(void *dst, void *src);
typedef uint32_t (*hash_map_fn)(void *key);

typedef struct hash_node
{
    struct hash_node *next;
    uint32_t data_size;
} hash_node_t;

typedef struct
{
    hash_alloc_fn alloc;
    hash_free_fn free_fn;
    hash_match_fn match;
    hash_save_fn save;
    hash_map_fn map;
    hash_node_t *head;
} hash_cache_t;

void hash_cache_reg(hash_cache_t *cache,
                    hash_alloc_fn alloc_fn,
                    hash_free_fn free_fn,
                    hash_match_fn match_fn,
                    hash_save_fn save_fn,
                    hash_map_fn map_fn,
                    void *unused0,
                    void *unused1,
                    uint32_t table_num,
                    uint32_t max_cache_size);
hash_node_t *hash_cache_get(hash_cache_t *cache, void *key);
hash_node_t *hash_cache_alloc(hash_cache_t *cache, void *key, uint32_t data_size);
void hash_cache_set(hash_cache_t *cache, hash_node_t *node, uint32_t flags);
void hash_cache_delete_all(hash_cache_t *cache);

#endif
