/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hash_cache.h"

void hash_cache_reg(hash_cache_t *cache,
                    hash_alloc_fn alloc_fn,
                    hash_free_fn free_fn,
                    hash_match_fn match_fn,
                    hash_save_fn save_fn,
                    hash_map_fn map_fn,
                    void *unused0,
                    void *unused1,
                    uint32_t table_num,
                    uint32_t max_cache_size)
{
    (void)unused0;
    (void)unused1;
    (void)table_num;
    (void)max_cache_size;

    cache->alloc = alloc_fn;
    cache->free_fn = free_fn;
    cache->match = match_fn;
    cache->save = save_fn;
    cache->map = map_fn;
    cache->head = NULL;
}

hash_node_t *hash_cache_get(hash_cache_t *cache, void *key)
{
    hash_node_t *node;

    for (node = cache->head; node; node = node->next)
    {
        if (cache->match && cache->match(node + 1, key))
        {
            return node;
        }
    }

    return NULL;
}

hash_node_t *hash_cache_alloc(hash_cache_t *cache, void *key, uint32_t data_size)
{
    hash_node_t *node;

    if (!cache || !cache->alloc) return NULL;
    node = (hash_node_t *)cache->alloc(sizeof(hash_node_t) + data_size);
    if (!node) return NULL;

    node->next = NULL;
    node->data_size = data_size;
    if (cache->save) cache->save(node + 1, key);

    return node;
}

void hash_cache_set(hash_cache_t *cache, hash_node_t *node, uint32_t flags)
{
    (void)flags;

    if (!cache || !node) return;
    node->next = cache->head;
    cache->head = node;
}

void hash_cache_delete_all(hash_cache_t *cache)
{
    hash_node_t *node;
    hash_node_t *next;

    if (!cache || !cache->free_fn) return;
    node = cache->head;
    while (node)
    {
        next = node->next;
        cache->free_fn(node);
        node = next;
    }
    cache->head = NULL;
}
