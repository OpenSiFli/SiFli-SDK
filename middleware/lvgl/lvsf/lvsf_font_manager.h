/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_FONT_MANAGER_H
#define LVSF_FONT_MANAGER_H

#include <stdint.h>

#include "lvgl.h"

#ifdef LV_USING_FREETYPE_ENGINE
#include "lv_freetype.h"
typedef struct lv_font_freetype_lib_dsc lv_font_freetype_lib_dsc_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lvsf_font_entry lvsf_font_entry_t;

typedef struct
{
    const char *name;
    const char *path;
    uint8_t external;
    uint8_t enabled;
    uint8_t priority;
} lvsf_font_config_t;

#ifdef LV_USING_FREETYPE_ENGINE

int lvsf_font_manager_init(uint32_t cache_size);
void lvsf_font_manager_deinit(void);

int lvsf_font_register(const lvsf_font_config_t *config);
int lvsf_font_register_batch(const lvsf_font_config_t *configs, uint32_t count);
int lvsf_font_register_lib(const lv_font_freetype_lib_dsc_t *lib, const char *name, uint8_t priority);

lvsf_font_entry_t *lvsf_font_find(const char *font_name);
lvsf_font_entry_t *lvsf_font_find_path(const char *font_path);
lv_font_t *lvsf_font_get(const char *font_name, uint16_t size);
lv_font_t *lvsf_font_get_from_entry(lvsf_font_entry_t *entry, uint16_t size);
lvsf_font_entry_t *lvsf_font_traverse(lvsf_font_entry_t *prev);
uint32_t lvsf_font_get_count(void);
lvsf_font_entry_t *lvsf_font_get_default(void);
int lvsf_font_set_default(const char *font_name);
int lvsf_font_set_priority(const char *font_name, uint8_t priority);
void lvsf_font_clear_cache(const char *font_name);
void lvsf_font_clear_all_cache(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
