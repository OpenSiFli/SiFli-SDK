/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_EPIC_SHADOW_CACHE_H__
#define __DRV_EPIC_SHADOW_CACHE_H__

#include "rtthread.h"

/** Geometry parameters used to identify a generated shadow mask. */
typedef struct
{
    uint16_t width;
    uint16_t radius;
    uint16_t core_width;
    uint16_t core_height;
    uint32_t size;
} drv_epic_shadow_cache_key_t;

/** Generates an A8 shadow mask into the supplied buffer. */
typedef rt_err_t (*drv_epic_shadow_cache_generate_cb)(uint8_t *mask, void *user_data);

/**
 * Acquire a cached shadow mask, or generate a temporary one when necessary.
 */
rt_err_t drv_epic_shadow_cache_acquire(const drv_epic_shadow_cache_key_t *key,
                                       size_t bytes,
                                       drv_epic_shadow_cache_generate_cb generate,
                                       void *user_data,
                                       uint8_t **mask);

/** Release a mask acquired with drv_epic_shadow_cache_acquire(). */
void drv_epic_shadow_cache_release(uint8_t *mask);

#endif /* __DRV_EPIC_SHADOW_CACHE_H__ */
