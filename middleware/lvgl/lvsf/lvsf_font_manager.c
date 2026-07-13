/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "lvsf_font.h"
#include "lvsf_font_manager.h"

#ifdef LV_USING_FREETYPE_ENGINE

#include <rtthread.h>
#include <string.h>

#include "lv_freetype.h"
#include "lvsf_ft_reg.h"

#ifndef LVSF_FONT_MIN_FREE_HEAP
    #ifdef CONFIG_LVSF_FONT_MIN_FREE_HEAP
        #define LVSF_FONT_MIN_FREE_HEAP CONFIG_LVSF_FONT_MIN_FREE_HEAP
    #else
        #define LVSF_FONT_MIN_FREE_HEAP 0
    #endif
#endif

#if !defined(LVSF_FONT_UNLOAD_REF_CHECK) && defined(CONFIG_LVSF_FONT_UNLOAD_REF_CHECK)
    #define LVSF_FONT_UNLOAD_REF_CHECK 1
#endif

SECTION_DEF(FONT_SECTION_NAME, font_desc_t);

typedef struct lvsf_font_cache
{
    uint16_t size;
    lv_font_t *font;
    struct lvsf_font_cache *next;
} lvsf_font_cache_t;

struct lvsf_font_entry
{
    rt_list_t node;
    char *name;
    char *path;
    uint8_t external;
    uint8_t enabled;
    uint8_t priority;
    const lv_font_freetype_lib_dsc_t *lib;
    lvsf_font_cache_t *cache;
};

extern uint32_t ft_get_cache_size(void);

static rt_list_t g_font_list;
static uint8_t g_font_list_inited;
static uint8_t g_freetype_inited;
static uint32_t g_freetype_cache_size;
static lvsf_font_entry_t *g_builtin_font;
static lvsf_font_entry_t *g_default_font;
/* Set by lvsf_font_unload() when a font object could not be freed. */
static uint8_t g_font_in_use;

static void lvsf_font_list_init(void)
{
    if (!g_font_list_inited)
    {
        rt_list_init(&g_font_list);
        g_font_list_inited = 1;
    }
}

static char *lvsf_font_strdup(const char *str)
{
    size_t len;
    char *copy;

    if (!str) return RT_NULL;

    len = strlen(str) + 1;
    copy = rt_malloc(len);
    RT_ASSERT(copy);
    memcpy(copy, str, len);
    return copy;
}

static char *lvsf_font_path_to_name(const char *path)
{
    const char *base;
    const char *dot;
    size_t len;
    char *name;

    if (!path) return RT_NULL;

    base = strrchr(path, '/');
    base = base ? base + 1 : path;
    dot = strrchr(base, '.');
    len = dot && dot > base ? (size_t)(dot - base) : strlen(base);

    name = rt_malloc(len + 1);
    RT_ASSERT(name);
    memcpy(name, base, len);
    name[len] = '\0';
    return name;
}

static void lvsf_font_apply_priority(lvsf_font_entry_t *entry)
{
    rt_list_t *pos;

    if (!entry) return;

    rt_list_remove(&entry->node);
    rt_list_for_each(pos, &g_font_list)
    {
        lvsf_font_entry_t *iter = rt_list_entry(pos, lvsf_font_entry_t, node);
        if (entry->priority < iter->priority)
        {
            rt_list_insert_before(&iter->node, &entry->node);
            return;
        }
    }

    rt_list_insert_before(&g_font_list, &entry->node);
}

#ifdef LVSF_FONT_UNLOAD_REF_CHECK
static bool lvsf_style_refs_font(const lv_style_t *style, const lv_font_t *font)
{
    lv_style_value_t value;

    if (!style) return false;
    if (lv_style_get_prop(style, LV_STYLE_TEXT_FONT, &value) != LV_STYLE_RES_FOUND) return false;
    return value.ptr == (const void *)font;
}

static bool lvsf_obj_refs_font(const lv_obj_t *obj, const lv_font_t *font)
{
    uint32_t i;
    uint32_t child_cnt;

    if (!obj) return false;

    for (i = 0; i < obj->style_cnt; i++)
    {
        if (lvsf_style_refs_font(obj->styles[i].style, font)) return true;
    }

    child_cnt = lv_obj_get_child_cnt(obj);
    for (i = 0; i < child_cnt; i++)
    {
        if (lvsf_obj_refs_font(lv_obj_get_child((lv_obj_t *)obj, i), font)) return true;
    }

    return false;
}

/*
 * A safety net, not a proof: the scan sees the styles attached to an object,
 * which is where fonts normally live, but not a font kept anywhere else - an
 * unattached style, a span's own style, a draw descriptor of the application.
 * Re-pointing every reference before unloading remains the caller's job.
 *
 * Reads LVGL objects, so it runs in the LVGL thread like any other LVGL call.
 */
/* A theme bakes its fonts into styles of its own, attached to an object only
 * while a widget of that kind is on screen and out of reach of the object
 * walk otherwise. The theme holds the same pointers, so ask it directly. */
static bool lvsf_theme_refs_font(const lv_theme_t *theme, const lv_font_t *font)
{
    for (; theme; theme = theme->parent)
    {
        if (theme->font_small == font || theme->font_normal == font ||
                theme->font_large == font || theme->font_subtitle == font ||
                theme->font_title == font || theme->font_bigl == font)
        {
            return true;
        }
    }

    return false;
}

static bool lvsf_font_is_displayed(const lv_font_t *font)
{
    lv_disp_t *disp;

    for (disp = lv_disp_get_next(RT_NULL); disp; disp = lv_disp_get_next(disp))
    {
        uint32_t i;

        if (lvsf_theme_refs_font(lv_disp_get_theme(disp), font)) return true;

        /* The two layers are screens as well (lv_disp_drv_register). */
        for (i = 0; i < disp->screen_cnt; i++)
        {
            if (lvsf_obj_refs_font(disp->screens[i], font)) return true;
        }
    }

    return false;
}
#endif /* LVSF_FONT_UNLOAD_REF_CHECK */

/* Fonts of the manager may use each other as fallback; drop those links before
 * the target is freed. Fallbacks the application set on its own fonts are out
 * of reach - re-point them before unloading. */
static void lvsf_font_detach_fallback(const lv_font_t *font)
{
    rt_list_t *pos;

    rt_list_for_each(pos, &g_font_list)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        lvsf_font_cache_t *cache;

        for (cache = entry->cache; cache; cache = cache->next)
        {
            if (cache->font && cache->font->fallback == font)
            {
                cache->font->fallback = RT_NULL;
            }
        }
    }
}

static bool lvsf_font_free_runtime(lv_font_t *font)
{
    if (!font) return true;

#ifdef LVSF_FONT_UNLOAD_REF_CHECK
    if (lvsf_font_is_displayed(font))
    {
        rt_kprintf("[lvsf_font] font %s size=%u still referenced by a live object, kept\n",
                   font->font_name ? font->font_name : "unknown",
                   (unsigned int)lvsf_get_size_from_font(font));
        return false;
    }
#endif

    lvsf_font_detach_fallback(font);
    lv_freetype_font_deinit(font);
    rt_free(font);
    return true;
}

/* Returns true when every font object of the entry could be freed. */
static bool lvsf_font_entry_clear_cache(lvsf_font_entry_t *entry)
{
    lvsf_font_cache_t **pp;
    bool all_freed = true;

    if (!entry) return true;

    pp = &entry->cache;
    while (*pp)
    {
        lvsf_font_cache_t *cache = *pp;

        if (lvsf_font_free_runtime(cache->font))
        {
            *pp = cache->next;
            rt_free(cache);
        }
        else
        {
            all_freed = false;
            pp = &cache->next;
        }
    }

    return all_freed;
}

/* The caller must have unlinked the entry and confirmed that its cache is
 * empty (lvsf_font_entry_clear_cache() returned true). */
static void lvsf_font_free_entry(lvsf_font_entry_t *entry)
{
    if (!entry) return;

    if (entry->name) rt_free(entry->name);
    if (entry->path) rt_free(entry->path);
    rt_free(entry);
}

/* Detach an entry that is about to be freed from the manager's own pointers. */
static void lvsf_font_forget_entry(lvsf_font_entry_t *entry)
{
    if (g_default_font == entry) g_default_font = g_builtin_font != entry ? g_builtin_font : RT_NULL;
    if (g_builtin_font == entry) g_builtin_font = RT_NULL;
    rt_list_remove(&entry->node);
}

/*
 * An opened font keeps its index tables resident, and a full CJK font can eat
 * most of a small heap. Without a floor the font loads fine and the system dies
 * later on an unrelated allocation. Report the failure here instead, so the
 * application can fall back to another font.
 *
 * Only fonts loaded from the file system are gated: refusing a builtin font
 * would leave the application with no font at all, and its data lives in flash
 * anyway.
 */
static bool lvsf_font_heap_headroom_ok(const lvsf_font_entry_t *entry)
{
#if defined(LVSF_FONT_MIN_FREE_HEAP) && (LVSF_FONT_MIN_FREE_HEAP > 0) && \
    !defined(FREETYPE_CACHE_IN_SRAM_STANDALONE) && !defined(FREETYPE_CACHE_IN_PSRAM)
    rt_uint32_t total = 0;
    rt_uint32_t used = 0;
    rt_uint32_t max_used = 0;

    if (!entry->external) return true;

    rt_memory_info(&total, &used, &max_used);
    if (total <= used || (total - used) < (rt_uint32_t)LVSF_FONT_MIN_FREE_HEAP)
    {
        return false;
    }
#else
    (void)entry;
#endif
    return true;
}

static int lvsf_font_ensure_freetype(uint32_t cache_size)
{
    if (!cache_size)
    {
        cache_size = g_freetype_cache_size ? g_freetype_cache_size : ft_get_cache_size();
    }

    g_freetype_cache_size = cache_size;
    if (g_freetype_inited) return 0;

#ifdef PKG_SCHRIFT
    rt_kprintf("[lvsf_font] freetype init cache=%u faces=%u sizes=schrift\n",
               cache_size, (uint32_t)LV_FREETYPE_CACHE_FT_FACES);
    if (lv_freetype_init(LV_FREETYPE_CACHE_FT_FACES, cache_size) != 0)
#else
    rt_kprintf("[lvsf_font] freetype init cache=%u faces=%u sizes=%u\n",
               cache_size, (uint32_t)LV_FREETYPE_CACHE_FT_FACES,
               (uint32_t)LV_FREETYPE_CACHE_FT_SIZES);
    if (lv_freetype_init(LV_FREETYPE_CACHE_FT_FACES, LV_FREETYPE_CACHE_FT_SIZES, cache_size) != 0)
#endif
    {
        return -1;
    }

    g_freetype_inited = 1;
    return 0;
}

static lvsf_font_entry_t *lvsf_font_find_by_name(const char *font_name)
{
    rt_list_t *pos;

    if (!font_name) return RT_NULL;

    lvsf_font_list_init();
    rt_list_for_each(pos, &g_font_list)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        if (entry->name && strcmp(entry->name, font_name) == 0)
        {
            return entry;
        }
    }

    return RT_NULL;
}

static lvsf_font_entry_t *lvsf_font_find_by_path(const char *font_path)
{
    rt_list_t *pos;

    if (!font_path) return RT_NULL;

    lvsf_font_list_init();
    rt_list_for_each(pos, &g_font_list)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        if (entry->path && strcmp(entry->path, font_path) == 0)
        {
            return entry;
        }
    }

    return RT_NULL;
}

static lvsf_font_entry_t *lvsf_font_find_by_lib(const lv_font_freetype_lib_dsc_t *lib)
{
    rt_list_t *pos;

    if (!lib) return RT_NULL;

    lvsf_font_list_init();
    rt_list_for_each(pos, &g_font_list)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        if (entry->lib == lib)
        {
            return entry;
        }
    }

    return RT_NULL;
}

static lvsf_font_entry_t *lvsf_font_create_builtin(const font_desc_t *desc)
{
    lvsf_font_entry_t *entry;

    if (!desc || !desc->font_lib || !desc->font_name) return RT_NULL;

    entry = lvsf_font_find_by_lib(desc->font_lib);
    if (entry) return entry;

    entry = rt_calloc(1, sizeof(*entry));
    RT_ASSERT(entry);
    rt_list_init(&entry->node);
    entry->name = lvsf_font_strdup(desc->font_name);
    entry->path = desc->font_lib->font_lib_data ? lvsf_font_strdup(desc->font_lib->font_lib_data) : RT_NULL;
    entry->lib = desc->font_lib;
    entry->enabled = 1;
    entry->priority = 0;
    rt_list_insert_before(&g_font_list, &entry->node);

    if (!g_builtin_font)
    {
        g_builtin_font = entry;
        g_default_font = entry;
    }

    return entry;
}

static void lvsf_font_register_builtin(void)
{
    uint32_t i;
    uint32_t count;

    if (g_builtin_font) return;

    lvsf_font_list_init();

    count = SECTION_ITEM_COUNT(FONT_SECTION_NAME, font_desc_t);
    for (i = 0; i < count; i++)
    {
        const font_desc_t *desc = SECTION_ITEM_GET(FONT_SECTION_NAME, font_desc_t, i);
        lvsf_font_create_builtin(desc);
    }
}

static lvsf_font_entry_t *lvsf_font_create_external(const char *font_path, const char *font_name)
{
    lvsf_font_entry_t *entry;
    char *name;

    if (!font_path) return RT_NULL;

    entry = lvsf_font_find_by_path(font_path);
    if (entry) return entry;

    name = font_name ? lvsf_font_strdup(font_name) : lvsf_font_path_to_name(font_path);

    /* The name is the lookup key, so a second entry under the same name
     * (Roboto.ttf and Roboto.otf both map to "Roboto") would be unreachable
     * by every by-name API. */
    if (lvsf_font_find_by_name(name))
    {
        rt_kprintf("[lvsf_font] name collision: %s (from %s)\n", name, font_path);
        rt_free(name);
        return RT_NULL;
    }

    entry = rt_calloc(1, sizeof(*entry));
    RT_ASSERT(entry);
    rt_list_init(&entry->node);
    entry->name = name;
    entry->path = lvsf_font_strdup(font_path);
    entry->external = 1;
    entry->enabled = 1;
    rt_list_insert_before(&g_font_list, &entry->node);
    return entry;
}

static lv_font_t *lvsf_font_get_or_create(lvsf_font_entry_t *entry, uint16_t size)
{
    lvsf_font_cache_t *cache;
    lv_font_t *font;

    if (!entry) return RT_NULL;
    if (size == 0) size = FONT_NORMAL;

    for (cache = entry->cache; cache; cache = cache->next)
    {
        if (cache->size == size)
        {
            return cache->font;
        }
    }

    if (lvsf_font_ensure_freetype(0) != 0)
    {
        return RT_NULL;
    }

    rt_kprintf("[lvsf_font] create start name=%s size=%u external=%u src=%s\n",
               entry->name ? entry->name : "unknown",
               size,
               entry->external,
               entry->external ? (entry->path ? entry->path : "null") : "<mem>");

    font = rt_calloc(1, sizeof(*font));
    RT_ASSERT(font);

    if (lv_freetype_font_init(font,
                              entry->external ? entry->path : entry->lib->font_lib_data,
                              entry->external ? 0 : entry->lib->font_lib_size,
                              size,
                              entry->name) != 0)
    {
        /* The glyph cache may be holding the heap the index tables need, and
         * it is rebuilt on demand: drop it and give the font one more try. */
        lv_freetype_clean_cache(FT_CACHE_WHOLE_CLEAN);

        if (lv_freetype_font_init(font,
                                  entry->external ? entry->path : entry->lib->font_lib_data,
                                  entry->external ? 0 : entry->lib->font_lib_size,
                                  size,
                                  entry->name) != 0)
        {
            rt_kprintf("[lvsf_font] create failed name=%s size=%u\n",
                       entry->name ? entry->name : "unknown", size);
            rt_free(font);
            return RT_NULL;
        }
    }

    /* The font is open now: its tables are resident and their cost is visible
     * in the heap. Give it back if it does not leave the system enough room. */
    if (!lvsf_font_heap_headroom_ok(entry))
    {
        rt_kprintf("[lvsf_font] %s size=%u leaves less than %d bytes of heap, refused\n",
                   entry->name ? entry->name : "unknown", size, LVSF_FONT_MIN_FREE_HEAP);
        lv_freetype_font_deinit(font);
        rt_free(font);
        return RT_NULL;
    }

    cache = rt_calloc(1, sizeof(*cache));
    RT_ASSERT(cache);
    cache->size = size;
    cache->font = font;
    cache->next = entry->cache;
    entry->cache = cache;

    rt_kprintf("[lvsf_font] create ok name=%s size=%u font=0x%x dsc=0x%x\n",
               entry->name ? entry->name : "unknown",
               size,
               (uint32_t)font,
               (uint32_t)font->user_data);
    return font;
}

static int lvsf_font_path_matches(lvsf_font_entry_t *entry, const char *font_path)
{
    size_t prefix_len;

    if (!entry || !entry->external || !entry->path) return 0;
    if (!font_path) return 1;
    if (strcmp(entry->path, font_path) == 0) return 1;
    if (strcmp(entry->name, font_path) == 0) return 1;

    prefix_len = strlen(font_path);
    if (prefix_len > 0 && strncmp(entry->path, font_path, prefix_len) == 0)
    {
        if (entry->path[prefix_len] == '\0' || entry->path[prefix_len] == '/')
        {
            return 1;
        }
    }

    return 0;
}

lv_font_t *lvsf_get_font_from_size(uint16_t size)
{
    rt_list_t *pos;

    lvsf_font_register_builtin();

    rt_list_for_each(pos, &g_font_list)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        if (!entry->enabled) continue;

        lv_font_t *font = lvsf_font_get_or_create(entry, size);
        if (font)
        {
            return font;
        }
    }

    /* The loop above already tried every enabled entry; retry the default
     * font only if it was skipped for being disabled. */
    {
        lvsf_font_entry_t *fallback = g_default_font ? g_default_font : g_builtin_font;
        if (fallback && !fallback->enabled)
        {
            return lvsf_font_get_or_create(fallback, size);
        }
    }
    return RT_NULL;
}

lv_font_t *lvsf_get_font_by_name(char *font_name, int size)
{
    lvsf_font_entry_t *entry;

    lvsf_font_register_builtin();

    if (!font_name)
    {
        return lvsf_get_font_from_size(size < 0 ? FONT_NORMAL : (uint16_t)size);
    }

    if (strcmp(font_name, "default") == 0)
    {
        entry = g_default_font ? g_default_font : g_builtin_font;
    }
    else
    {
        entry = lvsf_font_find_by_name(font_name);
    }

    if (!entry) return RT_NULL;
    if (size < 0) size = FONT_NORMAL;
    return lvsf_font_get_or_create(entry, (uint16_t)size);
}

lv_font_t *lvsf_get_font_by_lib(const lv_font_freetype_lib_dsc_t *font_lib, uint16_t size)
{
    lvsf_font_register_builtin();

    return lvsf_font_get_or_create(lvsf_font_find_by_lib(font_lib), size);
}

void lvsf_font_load(uint32_t cache_size)
{
    lvsf_font_register_builtin();
    lvsf_font_ensure_freetype(cache_size);
}

int lvsf_font_manager_init(uint32_t cache_size)
{
    lvsf_font_register_builtin();
    return lvsf_font_ensure_freetype(cache_size);
}

void lvsf_font_manager_deinit(void)
{
    lvsf_font_deinit();
}

void lvsf_font_inital(uint32_t cache_size, bool init)
{
    static const uint16_t default_sizes[] = {
        FONT_SMALL,
        FONT_NORMAL,
        FONT_SUBTITLE,
        FONT_TITLE,
        FONT_BIGL,
        FONT_HUGE,
    };
    uint32_t i;

    (void)init;

    lvsf_font_load(cache_size);
    for (i = 0; i < sizeof(default_sizes) / sizeof(default_sizes[0]); i++)
    {
        /* A builtin font may point at a file this product does not ship; one
         * failed open is enough to know, so do not retry it per size. */
        if (!lvsf_font_get_or_create(g_builtin_font, default_sizes[i]))
        {
            rt_kprintf("[lvsf_font] builtin font unavailable, skip prewarm\n");
            break;
        }
    }
}

void lvsf_font_unload(void)
{
    rt_list_t *pos;
    rt_list_t *next;

    lvsf_font_register_builtin();

    for (pos = g_font_list.next; pos != &g_font_list; pos = next)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        next = pos->next;
        /* Keep an entry whose font objects are still on screen (its cache
         * holds them); the caller has to re-point its styles first. */
        if (!lvsf_font_entry_clear_cache(entry))
        {
            /* The entry stays alive with the fonts that are still displayed,
             * but must not be picked implicitly any more: this is a teardown. */
            g_font_in_use = 1;
            if (entry != g_builtin_font) entry->enabled = 0;
            continue;
        }
        if (entry == g_builtin_font) continue;

        lvsf_font_forget_entry(entry);
        lvsf_font_free_entry(entry);
    }

    if (g_builtin_font)
    {
        g_builtin_font->enabled = 1;
    }
    g_default_font = g_builtin_font;
}

/* 0 when every font object is gone, -1 when some are still displayed - the
 * FreeType library must not be torn down under them. */
int lvsf_font_deinit(void)
{
    g_font_in_use = 0;
    lvsf_font_unload();
    if (g_font_in_use)
    {
        rt_kprintf("[lvsf_font] fonts still in use, keeping the engine alive\n");
        return -1;
    }

    g_freetype_inited = 0;
    return 0;
}

int lvsf_font_load_ex(char *font_path, uint16_t *size)
{
    lvsf_font_entry_t *entry;
    uint8_t created;
    uint8_t got_font = 0;
    char *name;
    int i;

    if (!font_path) return -1;

    lvsf_font_register_builtin();

    created = (lvsf_font_find_by_path(font_path) == RT_NULL);
    name = lvsf_font_path_to_name(font_path);
    entry = lvsf_font_create_external(font_path, name);
    rt_free(name);
    if (!entry) return -1;

    if (size)
    {
        for (i = 0; size[i]; i++)
        {
            if (lvsf_font_get_or_create(entry, size[i])) got_font = 1;
        }
    }
    else if (lvsf_font_get_or_create(entry, FONT_NORMAL))
    {
        got_font = 1;
    }

    if (!got_font)
    {
        /* Roll back only an entry this call registered: a pre-existing
         * entry may own font objects still referenced by live styles. */
        if (created && lvsf_font_entry_clear_cache(entry))
        {
            lvsf_font_forget_entry(entry);
            lvsf_font_free_entry(entry);
        }
        return -1;
    }

    return 1;
}

int lvsf_font_register(const lvsf_font_config_t *config)
{
    lvsf_font_entry_t *entry;

    if (!config || !config->name) return -1;

    lvsf_font_register_builtin();
    if (config->external)
    {
        entry = lvsf_font_create_external(config->path, config->name);
        if (!entry) return -1;
    }
    else if (strcmp(config->name, "default") == 0)
    {
        entry = g_default_font ? g_default_font : g_builtin_font;
    }
    else
    {
        entry = lvsf_font_find_by_name(config->name);
        if (!entry) return -1;
    }

    if (!entry)
    {
        return -1;
    }

    entry->enabled = config->enabled ? 1 : 0;
    entry->priority = config->priority;
    lvsf_font_apply_priority(entry);
    if (!g_default_font || config->priority == 0)
    {
        g_default_font = entry;
    }
    return 0;
}

int lvsf_font_register_batch(const lvsf_font_config_t *configs, uint32_t count)
{
    uint32_t i;

    if (!configs || !count) return -1;

    for (i = 0; i < count; i++)
    {
        int ret = lvsf_font_register(&configs[i]);
        if (ret != 0) return ret;
    }

    return 0;
}

int lvsf_font_register_lib(const lv_font_freetype_lib_dsc_t *lib, const char *name, uint8_t priority)
{
    lvsf_font_entry_t *entry;

    if (!lib || !name) return -1;

    lvsf_font_register_builtin();
    entry = lvsf_font_find_by_lib(lib);
    if (!entry)
    {
        entry = lvsf_font_find_by_name(name);
    }
    if (!entry)
    {
        entry = rt_calloc(1, sizeof(*entry));
        RT_ASSERT(entry);
        rt_list_init(&entry->node);
        entry->name = lvsf_font_strdup(name);
        entry->path = lib->font_lib_data ? lvsf_font_strdup(lib->font_lib_data) : RT_NULL;
        entry->lib = lib;
        entry->enabled = 1;
        rt_list_insert_before(&g_font_list, &entry->node);
    }
    else
    {
        /* Cached fonts hold raw pointers into entry->name/path, so leak the
         * old strings rather than free them under a live user. */
        if (!entry->cache)
        {
            if (entry->name) rt_free(entry->name);
            if (entry->path) rt_free(entry->path);
        }
        entry->name = lvsf_font_strdup(name);
        entry->path = lib->font_lib_data ? lvsf_font_strdup(lib->font_lib_data) : RT_NULL;
        entry->lib = lib;
    }

    entry->enabled = 1;
    entry->priority = priority;
    lvsf_font_apply_priority(entry);
    if (!g_default_font || priority == 0)
    {
        g_default_font = entry;
    }
    return 0;
}

int lvsf_font_unload_ex(char *font_path)
{
    rt_list_t *pos;
    rt_list_t *next;
    int ret = 0;

    lvsf_font_register_builtin();

    for (pos = g_font_list.next; pos != &g_font_list; pos = next)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        next = pos->next;
        if (!entry->external) continue;
        if (!lvsf_font_path_matches(entry, font_path)) continue;

        /* Font objects still selected by a live style must outlive this call;
         * keep the whole entry so the application can still reach them. */
        if (!lvsf_font_entry_clear_cache(entry))
        {
            rt_kprintf("[lvsf_font] %s still in use, not unloaded\n",
                       entry->name ? entry->name : "unknown");
            ret = -1;
            continue;
        }

        lvsf_font_forget_entry(entry);
        lvsf_font_free_entry(entry);
    }

    return ret;
}

int lvsf_font_set_enable(char *font_name, int enable)
{
    lvsf_font_entry_t *entry;

    lvsf_font_register_builtin();
    entry = lvsf_font_find_by_name(font_name);
    if (!entry) return -1;

    entry->enabled = enable ? 1 : 0;
    return 0;
}

char *lvsf_font_trav_ex(rt_list_t **list, int ex)
{
    rt_list_t *pos;

    lvsf_font_register_builtin();

    pos = (list && *list) ? *list : g_font_list.next;
    while (pos != &g_font_list)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        pos = pos->next;
        if (list)
        {
            *list = (pos == &g_font_list) ? RT_NULL : pos;
        }

        if ((ex == 1 && !entry->external) || (ex == 2 && entry->external))
        {
            continue;
        }
        return entry->name;
    }

    if (list) *list = RT_NULL;
    return RT_NULL;
}

void lvsf_font_set_order(char **font_name, uint16_t font_num)
{
    int i;

    lvsf_font_register_builtin();

    for (i = (int)font_num - 1; i >= 0; i--)
    {
        lvsf_font_entry_t *entry = lvsf_font_find_by_name(font_name[i]);
        if (!entry) continue;
        rt_list_remove(&entry->node);
        rt_list_insert_after(&g_font_list, &entry->node);
    }
}

void lvsf_font_set_order_reverse(char **font_name, uint16_t font_num)
{
    uint16_t i;

    lvsf_font_register_builtin();

    for (i = 0; i < font_num; i++)
    {
        lvsf_font_entry_t *entry = lvsf_font_find_by_name(font_name[i]);
        if (!entry) continue;
        rt_list_remove(&entry->node);
        rt_list_insert_before(&g_font_list, &entry->node);
    }
}

void lvsf_font_reset_order(void)
{
    rt_list_t *pos;

    lvsf_font_register_builtin();
    rt_list_for_each(pos, &g_font_list)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        entry->enabled = entry->external ? 0 : 1;
    }

    if (g_builtin_font)
    {
        rt_list_remove(&g_builtin_font->node);
        rt_list_insert_after(&g_font_list, &g_builtin_font->node);
    }
    g_default_font = g_builtin_font;
}

void lvsf_font_indicated_inital(const char *indicated_font)
{
    lvsf_font_entry_t *entry;

    entry = lvsf_font_find_by_name(indicated_font);
    if (entry)
    {
        entry->enabled = 1;
        lvsf_font_get_or_create(entry, FONT_NORMAL);
    }
}

void lvsf_set_font_size_by_name(char *font_name, uint16_t *size)
{
    lvsf_font_entry_t *entry;
    int i;

    if (!size) return;

    if (!font_name)
    {
        rt_list_t *pos;
        lvsf_font_register_builtin();
        rt_list_for_each(pos, &g_font_list)
        {
            lvsf_font_entry_t *iter = rt_list_entry(pos, lvsf_font_entry_t, node);
            for (i = 0; size[i]; i++)
            {
                lvsf_font_get_or_create(iter, size[i]);
            }
        }
        return;
    }

    entry = lvsf_font_find_by_name(font_name);
    if (!entry) return;

    for (i = 0; size[i]; i++)
    {
        lvsf_font_get_or_create(entry, size[i]);
    }
}

void lvsf_reset_font_size_by_name(char *font_name)
{
    lvsf_font_entry_t *entry;

    if (!font_name) return;
    entry = lvsf_font_find_by_name(font_name);
    if (!entry) return;

    lvsf_font_entry_clear_cache(entry);
}

lvsf_font_entry_t *lvsf_font_find(const char *font_name)
{
    lvsf_font_register_builtin();
    if (!font_name || strcmp(font_name, "default") == 0)
    {
        return g_builtin_font;
    }

    return lvsf_font_find_by_name(font_name);
}

lvsf_font_entry_t *lvsf_font_find_path(const char *font_path)
{
    lvsf_font_register_builtin();
    return lvsf_font_find_by_path(font_path);
}

lv_font_t *lvsf_font_get(const char *font_name, uint16_t size)
{
    return lvsf_get_font_by_name((char *)font_name, size);
}

lv_font_t *lvsf_font_get_from_entry(lvsf_font_entry_t *entry, uint16_t size)
{
    return lvsf_font_get_or_create(entry, size);
}

lvsf_font_entry_t *lvsf_font_traverse(lvsf_font_entry_t *prev)
{
    rt_list_t *pos;

    lvsf_font_register_builtin();
    pos = prev ? prev->node.next : g_font_list.next;
    if (pos == &g_font_list) return RT_NULL;
    return rt_list_entry(pos, lvsf_font_entry_t, node);
}

uint32_t lvsf_font_get_count(void)
{
    uint32_t count = 0;
    rt_list_t *pos;

    lvsf_font_register_builtin();
    rt_list_for_each(pos, &g_font_list)
    {
        count++;
    }

    return count;
}

lvsf_font_entry_t *lvsf_font_get_default(void)
{
    lvsf_font_register_builtin();
    return g_default_font ? g_default_font : g_builtin_font;
}

int lvsf_font_set_default(const char *font_name)
{
    lvsf_font_entry_t *entry = lvsf_font_find(font_name);

    if (!entry) return -1;
    g_default_font = entry;
    return 0;
}

int lvsf_font_set_priority(const char *font_name, uint8_t priority)
{
    lvsf_font_entry_t *entry = lvsf_font_find(font_name);

    if (!entry) return -1;
    entry->priority = priority;
    lvsf_font_apply_priority(entry);
    return 0;
}

void lvsf_font_clear_cache(const char *font_name)
{
    lvsf_font_entry_t *entry = lvsf_font_find(font_name);

    if (entry) lvsf_font_entry_clear_cache(entry);
}

void lvsf_font_clear_all_cache(void)
{
    rt_list_t *pos;

    lvsf_font_register_builtin();
    rt_list_for_each(pos, &g_font_list)
    {
        lvsf_font_entry_t *entry = rt_list_entry(pos, lvsf_font_entry_t, node);
        lvsf_font_entry_clear_cache(entry);
    }
}

#endif
