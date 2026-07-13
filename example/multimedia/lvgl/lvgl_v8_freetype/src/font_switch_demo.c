/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Load TTF fonts from /font/ and switch between them at runtime: pick a font
 * from the list, change its size, and watch the previous one being released.
 * See README.md for the rules that come with the font manager API.
 */

#include "rtthread.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef RT_USING_DFS
#include <dfs_posix.h>
#endif

#ifdef LV_USING_FREETYPE_ENGINE
#include "lvsf/lvsf_font.h"
#include "lvsf/lvsf_font_manager.h"
#include "lvsf/lvsf_ft_reg.h"
#endif

#define FONT_DEMO_PATH "/font/"
#define FONT_DEMO_DEFAULT_FONT "/font/DroidSansFallback_Simplified.ttf"
#define FONT_PREVIEW_TEXT "中文 English Mix\nHello LVGL 123\n标点: . , ! ? - _ ( )"

#ifdef LV_USING_FREETYPE_ENGINE

#define FONT_DEMO_SIZE_MIN 12
#define FONT_DEMO_SIZE_MAX 60
#define FONT_DEMO_SIZE_STEP 2
#define FONT_DEMO_TITLE_SIZE 28
#define FONT_DEMO_UI_SIZE 20
#define FONT_DEMO_BUTTON_SIZE 18

typedef struct
{
    lv_obj_t *main_cont;
    lv_obj_t *title_label;
    lv_obj_t *preview_label;
    lv_obj_t *font_info_label;
    lv_obj_t *font_size_label;
    lv_obj_t *bottom_bar;
    lv_obj_t *font_list_btn;
    lv_obj_t *size_inc_btn;
    lv_obj_t *size_dec_btn;
    lv_obj_t *font_list_label;
    lv_obj_t *size_inc_label;
    lv_obj_t *size_dec_label;
    lv_obj_t *font_list_page;
    lv_obj_t *font_list_scroll;
    int current_font_size;
    int current_font_ready;
    int current_font_enabled;
    char current_font_path[256];
    char current_font_name[128];
} font_switch_demo_t;

static font_switch_demo_t g_demo;

/*
 * MSH commands run in the finsh shell thread while lv_task_handler runs in
 * the main thread; LVGL v8 APIs are not thread-safe and this port provides
 * no global LVGL lock. Shell commands therefore only post a request here
 * and a demo-owned lv_timer executes it in the LVGL thread.
 */
typedef enum
{
    DEMO_CMD_NONE = 0,
    DEMO_CMD_VERIFY_FONT,
    DEMO_CMD_ENABLE_FONT,
    DEMO_CMD_RESTART,
} demo_cmd_t;

static volatile demo_cmd_t g_demo_pending_cmd;
/* Kept outside g_demo: the demo re-init memsets g_demo. */
static lv_timer_t *g_demo_cmd_timer;

static void demo_log_mem(const char *tag)
{
    rt_uint32_t total = 0;
    rt_uint32_t used = 0;
    rt_uint32_t max_used = 0;

    rt_memory_info(&total, &used, &max_used);
    rt_kprintf("[font_demo] mem %s total=%u used=%u free=%u max=%u\n",
               tag ? tag : "-",
               total,
               used,
               total > used ? total - used : 0,
               max_used);
}

static void create_font_list(lv_obj_t *parent);
void lv_example_font_switch_demo(void);

static lv_font_t *demo_get_selected_font(void)
{
    if (!g_demo.current_font_name[0] || strcmp(g_demo.current_font_name, "Built-in") == 0)
    {
        return RT_NULL;
    }

    return lvsf_get_font_by_name(g_demo.current_font_name, g_demo.current_font_size);
}

static int demo_enable_selected_font(void)
{
    int ret;
    char *font_order_name;

    if (!g_demo.current_font_name[0] || strcmp(g_demo.current_font_name, "Built-in") == 0)
    {
        rt_kprintf("[font_demo] enable: no external font selected\n");
        return -1;
    }

    rt_kprintf("[font_demo] enable start: %s\n", g_demo.current_font_name);
    ret = lvsf_font_set_enable(g_demo.current_font_name, 1);
    if (ret != 0)
    {
        rt_kprintf("[font_demo] enable failed: %s ret=%d\n", g_demo.current_font_name, ret);
        return -1;
    }

    font_order_name = g_demo.current_font_name;
    lvsf_font_set_order(&font_order_name, 1);
    g_demo.current_font_enabled = 1;

    rt_kprintf("[font_demo] enable/order ok: %s\n", g_demo.current_font_name);
    return 0;
}

static const lv_font_t *demo_get_ui_font(void)
{
    return LV_FONT_DEFAULT;
}

static void demo_attach_font_fallback(lv_font_t *font, uint16_t size)
{
    if (font && font->fallback == RT_NULL)
    {
        font->fallback = demo_get_ui_font();
        rt_kprintf("[font_demo] attach fallback: name=%s size=%d font=0x%x fallback=0x%x\n",
                   font->font_name ? font->font_name : "unknown",
                   size,
                   (uint32_t)font,
                   (uint32_t)font->fallback);
    }
}

static lv_font_t *demo_get_external_font(uint16_t size)
{
    lv_font_t *font;

    if (!g_demo.current_font_enabled ||
            !g_demo.current_font_name[0] ||
            strcmp(g_demo.current_font_name, "Built-in") == 0)
    {
        return RT_NULL;
    }

    font = lvsf_get_font_by_name(g_demo.current_font_name, size);
    if (font)
    {
        demo_attach_font_fallback(font, size);
        g_demo.current_font_ready = 1;
    }

    return font;
}

static const lv_font_t *demo_get_display_font(uint16_t size)
{
    lv_font_t *font = demo_get_external_font(size);

    return font ? font : demo_get_ui_font();
}

static void demo_set_label_font(lv_obj_t *label, const lv_font_t *font)
{
    if (label && lv_obj_is_valid(label))
    {
        lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    }
}

static void demo_apply_ui_fonts(void)
{
    const lv_font_t *title_font = demo_get_display_font(FONT_DEMO_TITLE_SIZE);
    const lv_font_t *ui_font = demo_get_display_font(FONT_DEMO_UI_SIZE);
    const lv_font_t *button_font = demo_get_display_font(FONT_DEMO_BUTTON_SIZE);

    rt_kprintf("[font_demo] ui fonts: name=%s enabled=%d title=0x%x/%d ui=0x%x/%d button=0x%x/%d\n",
               g_demo.current_font_name[0] ? g_demo.current_font_name : "Built-in",
               g_demo.current_font_enabled,
               (uint32_t)title_font, FONT_DEMO_TITLE_SIZE,
               (uint32_t)ui_font, FONT_DEMO_UI_SIZE,
               (uint32_t)button_font, FONT_DEMO_BUTTON_SIZE);

    demo_set_label_font(g_demo.title_label, title_font);
    demo_set_label_font(g_demo.font_info_label, ui_font);
    demo_set_label_font(g_demo.font_size_label, ui_font);
    demo_set_label_font(g_demo.size_dec_label, button_font);
    demo_set_label_font(g_demo.size_inc_label, button_font);
    demo_set_label_font(g_demo.font_list_label, button_font);
}

static int file_exists(const char *path)
{
#ifdef RT_USING_DFS
    return access(path, 0) == 0 ? 1 : 0;
#else
    return 0;
#endif
}

/* Match *.ttf / *.otf by case-insensitive suffix (FreeType handles both);
 * a plain strstr(".ttf") would also match names like "old.ttf.bak". */
static int demo_is_font_file(const char *name)
{
    size_t len = strlen(name);
    const char *ext;

    if (len < 5) return 0;
    ext = name + len - 4;
    if (ext[0] != '.') return 0;
    if ((ext[1] | 0x20) == 't' && (ext[2] | 0x20) == 't' && (ext[3] | 0x20) == 'f') return 1;
    if ((ext[1] | 0x20) == 'o' && (ext[2] | 0x20) == 't' && (ext[3] | 0x20) == 'f') return 1;
    return 0;
}

/* Must match lvsf_font_path_to_name(): strip only the last extension, so
 * "My.Font.ttf" maps to "My.Font" on both sides. */
static void extract_font_name(const char *path, char *name, int max_len)
{
    const char *base;
    const char *dot;
    int len;

    if (!path || !name || max_len <= 0) return;

    base = strrchr(path, '/');
    base = base ? base + 1 : path;

    dot = strrchr(base, '.');
    len = (dot && dot > base) ? (int)(dot - base) : (int)strlen(base);
    if (len > max_len - 1) len = max_len - 1;
    memcpy(name, base, len);
    name[len] = '\0';
}

static int load_ttf_font(const char *font_path)
{
    int ret;
    char font_name[128];

    if (!font_path)
    {
        rt_kprintf("[font_demo] Error: font_path is NULL\n");
        return -1;
    }

    if (!file_exists(font_path))
    {
        rt_kprintf("[font_demo] Error: font file not found: %s\n", font_path);
        return -1;
    }

    extract_font_name(font_path, font_name, sizeof(font_name));

    rt_kprintf("[font_demo] load: path=%s name=%s\n", font_path, font_name);

    /* NULL sizes: lvsf_font_load_ex() validates the file by creating one font
     * object, so a broken TTF fails here and not at the next UI update. */
    ret = lvsf_font_load_ex((char *)font_path, NULL);
    if (ret == -1)
    {
        rt_kprintf("[font_demo] Failed to load font: %s\n", font_name);
        return -1;
    }

    /* Fails when the font does not fit this board: the manager refuses one
     * that would leave less than LVSF_FONT_MIN_FREE_HEAP of heap. */
    if (!lvsf_get_font_by_name(font_name, g_demo.current_font_size))
    {
        rt_kprintf("[font_demo] Failed to create font %s at size %d\n",
                   font_name, g_demo.current_font_size);
        lvsf_font_set_enable(font_name, 0);
        /* Nothing displays the new font yet, so it can go right back. A
         * re-selected current font is still in use and must stay. */
        if (strcmp(font_path, g_demo.current_font_path) != 0)
        {
            lvsf_font_unload_ex((char *)font_path);
        }
        return -1;
    }

    /* Disable the previously selected external font: only the newly
     * selected one should stay enabled in the lvsf priority list. */
    if (g_demo.current_font_name[0] &&
            strcmp(g_demo.current_font_name, "Built-in") != 0 &&
            strcmp(g_demo.current_font_name, font_name) != 0)
    {
        lvsf_font_set_enable(g_demo.current_font_name, 0);
    }

    g_demo.current_font_ready = 1;
    g_demo.current_font_enabled = 1;

    strncpy(g_demo.current_font_path, font_path, sizeof(g_demo.current_font_path) - 1);
    g_demo.current_font_path[sizeof(g_demo.current_font_path) - 1] = '\0';
    strncpy(g_demo.current_font_name, font_name, sizeof(g_demo.current_font_name) - 1);
    g_demo.current_font_name[sizeof(g_demo.current_font_name) - 1] = '\0';

    return 0;
}

static void update_font_status(void)
{
    lv_font_t *font;
    lv_font_t *preview_font = RT_NULL;
    const char *font_name;
    const char *font_state;

    font_name = g_demo.current_font_name[0] ? g_demo.current_font_name : "Built-in";
    if (strcmp(font_name, "Built-in") == 0)
    {
        font_state = "built-in";
    }
    else
    {
        font = RT_NULL;
        if (g_demo.current_font_enabled)
        {
            font = demo_get_external_font(g_demo.current_font_size);
            g_demo.current_font_ready = font ? 1 : 0;
        }

        if (font)
        {
            preview_font = font;
            font_state = "ready/enabled/preview";
            rt_kprintf("[font_demo] preview external: name=%s size=%d font=0x%x\n",
                       font_name, g_demo.current_font_size, (uint32_t)font);
        }
        else if (g_demo.current_font_ready)
        {
            font_state = g_demo.current_font_enabled ? "ready/enabled" : "ready";
        }
        else
        {
            font_state = g_demo.current_font_enabled ? "enabled" : "registered/disabled";
        }
    }

    demo_apply_ui_fonts();

    lv_obj_set_style_text_font(g_demo.preview_label,
                               preview_font ? preview_font : demo_get_ui_font(),
                               LV_PART_MAIN);
    lv_label_set_text_fmt(g_demo.preview_label,
                          "%s\n\nSelected font: %s\nSize: %d\nFont object: %s",
                          FONT_PREVIEW_TEXT,
                          font_name,
                          g_demo.current_font_size,
                          font_state);

    lv_label_set_text_fmt(g_demo.font_info_label, "Font: %s", font_name);

    lv_label_set_text_fmt(g_demo.font_size_label, "Size: %d", g_demo.current_font_size);

}

static void demo_verify_font_impl(void)
{
    lv_font_t *font;

    if (!g_demo.current_font_name[0] || strcmp(g_demo.current_font_name, "Built-in") == 0)
    {
        rt_kprintf("[font_demo] verify: no external font selected\n");
        return;
    }

    rt_kprintf("[font_demo] verify: create start name=%s size=%d\n",
               g_demo.current_font_name, g_demo.current_font_size);
    font = demo_get_selected_font();
    rt_kprintf("[font_demo] verify: create result font=0x%x\n", (uint32_t)font);
    g_demo.current_font_ready = font ? 1 : 0;

    if (g_demo.preview_label && lv_obj_is_valid(g_demo.preview_label))
    {
        update_font_status();
    }
}

static void demo_enable_font_impl(void)
{
    if (demo_enable_selected_font() == 0 &&
            g_demo.preview_label && lv_obj_is_valid(g_demo.preview_label))
    {
        update_font_status();
    }
}

static void demo_cmd_timer_cb(lv_timer_t *timer)
{
    demo_cmd_t cmd = g_demo_pending_cmd;

    (void)timer;

    if (cmd == DEMO_CMD_NONE) return;
    g_demo_pending_cmd = DEMO_CMD_NONE;

    switch (cmd)
    {
    case DEMO_CMD_VERIFY_FONT:
        demo_verify_font_impl();
        break;
    case DEMO_CMD_ENABLE_FONT:
        demo_enable_font_impl();
        break;
    case DEMO_CMD_RESTART:
        lv_example_font_switch_demo();
        break;
    default:
        break;
    }
}

static void demo_post_cmd(demo_cmd_t cmd)
{
    if (!g_demo_cmd_timer)
    {
        rt_kprintf("[font_demo] demo not started; start it from the application (main) first\n");
        return;
    }
    if (g_demo_pending_cmd != DEMO_CMD_NONE && g_demo_pending_cmd != cmd)
    {
        rt_kprintf("[font_demo] pending command %d replaced by %d\n",
                   (int)g_demo_pending_cmd, (int)cmd);
    }
    g_demo_pending_cmd = cmd;
}

void font_demo_verify_font(void)
{
    demo_post_cmd(DEMO_CMD_VERIFY_FONT);
}

void font_demo_enable_font(void)
{
    demo_post_cmd(DEMO_CMD_ENABLE_FONT);
}

static void btn_size_inc_cb(lv_event_t *e)
{
    (void)e;

    if (g_demo.current_font_size < FONT_DEMO_SIZE_MAX)
    {
        g_demo.current_font_size += FONT_DEMO_SIZE_STEP;
        update_font_status();
    }
    else
    {
        rt_kprintf("[font_demo] size max reached: %d\n", g_demo.current_font_size);
    }
}

static void btn_size_dec_cb(lv_event_t *e)
{
    (void)e;

    if (g_demo.current_font_size > FONT_DEMO_SIZE_MIN)
    {
        g_demo.current_font_size -= FONT_DEMO_SIZE_STEP;
        update_font_status();
    }
    else
    {
        rt_kprintf("[font_demo] size min reached: %d\n", g_demo.current_font_size);
    }
}

/* A displayed font cannot be unloaded, so give the UI back to the built-in
 * font first - that is the contract of lvsf_font_unload_ex(). */
static void demo_release_current_font(void)
{
    char path[sizeof(g_demo.current_font_path)];

    if (!g_demo.current_font_name[0] ||
            strcmp(g_demo.current_font_name, "Built-in") == 0)
    {
        return;
    }

    strncpy(path, g_demo.current_font_path, sizeof(path) - 1);
    path[sizeof(path) - 1] = '\0';

    lvsf_font_set_enable(g_demo.current_font_name, 0);
    strncpy(g_demo.current_font_name, "Built-in", sizeof(g_demo.current_font_name) - 1);
    g_demo.current_font_name[sizeof(g_demo.current_font_name) - 1] = '\0';
    g_demo.current_font_path[0] = '\0';
    g_demo.current_font_enabled = 0;
    g_demo.current_font_ready = 0;

    update_font_status();   /* every label now selects the built-in font */
    lvsf_font_unload_ex(path);
    demo_log_mem("after release current font");
}

static void btn_font_select_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_current_target(e);
    char *font_path = (char *)lv_obj_get_user_data(btn);

    if (!font_path)
    {
        rt_kprintf("[font_demo] Error: selected font path is NULL\n");
        return;
    }

    rt_kprintf("[font_demo] list select: %s\n", font_path);

    char prev_path[sizeof(g_demo.current_font_path)];
    char prev_name[sizeof(g_demo.current_font_name)];
    strncpy(prev_path, g_demo.current_font_path, sizeof(prev_path) - 1);
    prev_path[sizeof(prev_path) - 1] = '\0';
    strncpy(prev_name, g_demo.current_font_name, sizeof(prev_name) - 1);
    prev_name[sizeof(prev_name) - 1] = '\0';

    int has_prev = prev_name[0] && strcmp(prev_name, "Built-in") != 0 &&
                   strcmp(prev_path, font_path) != 0;

    int switched = (load_ttf_font(font_path) == 0);

    if (!switched && has_prev)
    {
        /* Both fonts are resident during a switch and the manager refuses one
         * that would leave the heap below its floor, so free the old font
         * and try once more. */
        rt_kprintf("[font_demo] retry after releasing %s\n", prev_name);
        demo_release_current_font();
        has_prev = 0;
        switched = (load_ttf_font(font_path) == 0);
    }

    if (switched)
    {
        demo_enable_selected_font();
        update_font_status();
    }
    else
    {
        rt_kprintf("[font_demo] list select load failed: %s\n", font_path);
        update_font_status();
    }

    /* Close the font list page: hide it right away (a hidden object is never
     * drawn) and delete it asynchronously, since this callback runs on one of
     * its own children. */
    if (g_demo.font_list_page)
    {
        lv_obj_add_flag(g_demo.font_list_page, LV_OBJ_FLAG_HIDDEN);
        lv_obj_del_async(g_demo.font_list_page);
        g_demo.font_list_page = NULL;
        g_demo.font_list_scroll = NULL;
    }

    /* Single resident external font: every label now selects the new font, so
     * release the previous one entirely - its face, its accumulated per-size
     * font objects and its cached glyphs. */
    if (switched && has_prev)
    {
        lvsf_font_unload_ex(prev_path);
        demo_log_mem("after unload previous font");
    }
}

static void btn_close_list_cb(lv_event_t *e)
{
    (void)e;

    if (g_demo.font_list_page)
    {
        lv_obj_add_flag(g_demo.font_list_page, LV_OBJ_FLAG_HIDDEN);
        rt_kprintf("[font_demo] list close page=0x%x\n", (uint32_t)g_demo.font_list_page);
    }
}

static void btn_show_font_list_cb(lv_event_t *e)
{
    (void)e;

    /* Rebuild the list on every open so TTF files added to the filesystem
     * at runtime show up without a reboot. */
    if (g_demo.font_list_page)
    {
        lv_obj_del(g_demo.font_list_page);
        g_demo.font_list_page = NULL;
        g_demo.font_list_scroll = NULL;
    }

    create_font_list(g_demo.main_cont);

    if (g_demo.font_list_page)
    {
        lv_obj_clear_flag(g_demo.font_list_page, LV_OBJ_FLAG_HIDDEN);
    }
}

static void btn_font_path_delete_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_current_target(e);
    char *font_path = (char *)lv_obj_get_user_data(btn);

    if (font_path)
    {
        rt_free(font_path);
        lv_obj_set_user_data(btn, NULL);
    }
}

static void create_font_list(lv_obj_t *parent)
{
    lv_obj_t *page;
    lv_obj_t *title;
    lv_obj_t *close_btn;
    lv_obj_t *close_label;
    lv_obj_t *scroll;
    lv_obj_t *font_btn;
    lv_obj_t *btn_label;
    char full_path[256];
    lv_coord_t y = 0;
    uint16_t font_count = 0;

#ifdef RT_USING_DFS
    DIR *dir;
    struct dirent *entry;
#endif

    rt_kprintf("[font_demo] create_font_list start parent=0x%x\n", (uint32_t)parent);
    demo_log_mem("create_font_list start");

    page = lv_obj_create(parent);
    lv_obj_set_size(page, LV_HOR_RES_MAX * 85 / 100, LV_VER_RES_MAX * 75 / 100);
    lv_obj_center(page);
    lv_obj_set_style_bg_opa(page, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(page, lv_color_hex(0xf5f5f5), LV_PART_MAIN);
    lv_obj_set_style_border_width(page, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(page, lv_color_hex(0x4CAF50), LV_PART_MAIN);
    lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);

    g_demo.font_list_page = page;

    title = lv_label_create(page);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);
    lv_obj_set_style_text_font(title, demo_get_ui_font(), LV_PART_MAIN);
    lv_label_set_text(title, "Available Fonts");

    close_btn = lv_btn_create(page);
    lv_obj_set_size(close_btn, 50, 40);
    lv_obj_align(close_btn, LV_ALIGN_TOP_RIGHT, -15, 10);
    lv_obj_set_style_bg_color(close_btn, lv_color_hex(0xff4444), LV_PART_MAIN);
    close_label = lv_label_create(close_btn);
    lv_obj_set_style_text_font(close_label, demo_get_ui_font(), LV_PART_MAIN);
    lv_label_set_text(close_label, "X");
    lv_obj_center(close_label);
    lv_obj_add_event_cb(close_btn, btn_close_list_cb, LV_EVENT_CLICKED, NULL);

    scroll = lv_obj_create(page);
    lv_obj_set_size(scroll, LV_PCT(90), LV_PCT(80));
    lv_obj_align(scroll, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_bg_opa(scroll, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scroll, 10, LV_PART_MAIN);

    g_demo.font_list_scroll = scroll;

#ifdef RT_USING_DFS
    dir = opendir(FONT_DEMO_PATH);
    rt_kprintf("[font_demo] opendir %s dir=0x%x\n", FONT_DEMO_PATH, (uint32_t)dir);
    if (dir)
    {
        while ((entry = readdir(dir)) != NULL)
        {
            if (demo_is_font_file(entry->d_name))
            {
                snprintf(full_path, sizeof(full_path), "%s%s", FONT_DEMO_PATH, entry->d_name);
                rt_kprintf("[font_demo] list font[%d]: %s\n", font_count, full_path);

                font_btn = lv_btn_create(scroll);
                lv_obj_set_size(font_btn, LV_PCT(100), 50);
                lv_obj_align(font_btn, LV_ALIGN_TOP_MID, 0, y);
                lv_obj_set_style_bg_color(font_btn, lv_color_hex(0xffffff), LV_PART_MAIN);
                lv_obj_set_style_bg_color(font_btn, lv_color_hex(0xe8f5e9), LV_STATE_PRESSED);
                lv_obj_set_style_border_width(font_btn, 1, LV_PART_MAIN);
                lv_obj_set_style_border_color(font_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);
                lv_obj_set_style_radius(font_btn, 8, LV_PART_MAIN);

                btn_label = lv_label_create(font_btn);
                lv_obj_set_width(btn_label, LV_PCT(95));
                lv_obj_set_style_text_font(btn_label, demo_get_ui_font(), LV_PART_MAIN);
                lv_obj_set_style_text_color(btn_label, lv_color_hex(0x333333), LV_PART_MAIN);
                lv_obj_set_style_text_align(btn_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
                lv_label_set_long_mode(btn_label, LV_LABEL_LONG_DOT);
                lv_label_set_text(btn_label, entry->d_name);
                lv_obj_center(btn_label);

                char *path_dup = rt_strdup(full_path);
                if (path_dup)
                {
                    lv_obj_set_user_data(font_btn, path_dup);
                    lv_obj_add_event_cb(font_btn, btn_font_select_cb, LV_EVENT_CLICKED, NULL);
                    lv_obj_add_event_cb(font_btn, btn_font_path_delete_cb, LV_EVENT_DELETE, NULL);
                }
                else
                {
                    rt_kprintf("[font_demo] path strdup failed: %s\n", full_path);
                }

                y += 55;
                font_count++;

            }
        }
        closedir(dir);
    }
#else
    rt_kprintf("[font_demo] Warning: DFS not enabled\n");
#endif

    if (y == 0)
    {
        lv_obj_t *msg = lv_label_create(scroll);
        lv_obj_set_style_text_color(msg, lv_color_hex(0x999999), LV_PART_MAIN);
        lv_obj_set_style_text_font(msg, demo_get_ui_font(), LV_PART_MAIN);
        lv_label_set_text(msg, "No TTF fonts found in /font/\n\nPlease place TTF files in /font/ directory");
        lv_obj_center(msg);
    }

    rt_kprintf("[font_demo] create_font_list done page=0x%x scroll=0x%x count=%d y=%d\n",
               (uint32_t)page, (uint32_t)scroll, font_count, y);
    demo_log_mem("create_font_list done");
}

static void create_ui(void)
{
    lv_obj_t *scr;

    scr = lv_scr_act();

    g_demo.main_cont = lv_obj_create(scr);
    lv_obj_set_size(g_demo.main_cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(g_demo.main_cont, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_demo.main_cont, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_clear_flag(g_demo.main_cont, LV_OBJ_FLAG_SCROLLABLE);

    g_demo.title_label = lv_label_create(g_demo.main_cont);
    lv_obj_set_width(g_demo.title_label, LV_HOR_RES_MAX * 92 / 100);
    lv_obj_align(g_demo.title_label, LV_ALIGN_TOP_MID, 0, 15);
    lv_obj_set_style_text_font(g_demo.title_label,
                               demo_get_ui_font(),
                               LV_PART_MAIN);
    lv_obj_set_style_text_color(g_demo.title_label, lv_color_hex(0x2196F3), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_demo.title_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_long_mode(g_demo.title_label, LV_LABEL_LONG_DOT);
    lv_label_set_text(g_demo.title_label, "Font Switch Demo");

    /* The bordered frame scrolls vertically, so a preview that outgrows it
     * can be scrolled instead of being clipped. */
    lv_obj_t *preview_cont = lv_obj_create(g_demo.main_cont);
    lv_obj_set_size(preview_cont, LV_HOR_RES_MAX * 9 / 10, LV_VER_RES_MAX * 46 / 100);
    lv_obj_align(preview_cont, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(preview_cont, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_border_width(preview_cont, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(preview_cont, lv_color_hex(0xe0e0e0), LV_PART_MAIN);
    lv_obj_set_style_pad_all(preview_cont, 15, LV_PART_MAIN);
    lv_obj_set_scroll_dir(preview_cont, LV_DIR_VER);

    g_demo.preview_label = lv_label_create(preview_cont);
    lv_obj_set_width(g_demo.preview_label, LV_PCT(100));
    lv_obj_set_style_text_color(g_demo.preview_label, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_label_set_long_mode(g_demo.preview_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_demo.preview_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    g_demo.font_info_label = lv_label_create(g_demo.main_cont);
    lv_obj_set_width(g_demo.font_info_label, LV_HOR_RES_MAX * 9 / 10);
    lv_obj_align(g_demo.font_info_label, LV_ALIGN_BOTTOM_MID, 0, -82);
    lv_obj_set_style_text_color(g_demo.font_info_label, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_demo.font_info_label, demo_get_ui_font(), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_demo.font_info_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_long_mode(g_demo.font_info_label, LV_LABEL_LONG_DOT);
    lv_label_set_text(g_demo.font_info_label, "Font: Built-in");

    g_demo.bottom_bar = lv_obj_create(g_demo.main_cont);
    lv_obj_set_size(g_demo.bottom_bar, LV_HOR_RES_MAX * 92 / 100, 56);
    lv_obj_align(g_demo.bottom_bar, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_set_style_bg_opa(g_demo.bottom_bar, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_demo.bottom_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_demo.bottom_bar, 0, LV_PART_MAIN);
    /* Round panel: the usable chord is narrower down here, so pull the
     * evenly spaced children inward, away from the corner. */
    lv_obj_set_style_pad_left(g_demo.bottom_bar, 40, LV_PART_MAIN);
    lv_obj_set_style_pad_right(g_demo.bottom_bar, 40, LV_PART_MAIN);
    lv_obj_clear_flag(g_demo.bottom_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(g_demo.bottom_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(g_demo.bottom_bar,
                          LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    g_demo.font_size_label = lv_label_create(g_demo.bottom_bar);
    lv_obj_set_size(g_demo.font_size_label, 86, 44);
    lv_obj_set_style_text_color(g_demo.font_size_label, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_demo.font_size_label, demo_get_ui_font(), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_demo.font_size_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text_fmt(g_demo.font_size_label, "Size: %d", g_demo.current_font_size);
    lv_obj_set_style_pad_top(g_demo.font_size_label, 10, LV_PART_MAIN);

    g_demo.size_dec_btn = lv_btn_create(g_demo.bottom_bar);
    lv_obj_set_size(g_demo.size_dec_btn, 52, 44);
    lv_obj_set_style_bg_color(g_demo.size_dec_btn, lv_color_hex(0xFF9800), LV_PART_MAIN);
    lv_obj_set_style_radius(g_demo.size_dec_btn, 8, LV_PART_MAIN);
    g_demo.size_dec_label = lv_label_create(g_demo.size_dec_btn);
    lv_label_set_text(g_demo.size_dec_label, "A-");
    lv_obj_set_style_text_font(g_demo.size_dec_label, demo_get_ui_font(), LV_PART_MAIN);
    lv_obj_center(g_demo.size_dec_label);
    lv_obj_add_event_cb(g_demo.size_dec_btn, btn_size_dec_cb, LV_EVENT_CLICKED, NULL);

    g_demo.size_inc_btn = lv_btn_create(g_demo.bottom_bar);
    lv_obj_set_size(g_demo.size_inc_btn, 52, 44);
    lv_obj_set_style_bg_color(g_demo.size_inc_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);
    lv_obj_set_style_radius(g_demo.size_inc_btn, 8, LV_PART_MAIN);
    g_demo.size_inc_label = lv_label_create(g_demo.size_inc_btn);
    lv_label_set_text(g_demo.size_inc_label, "A+");
    lv_obj_set_style_text_font(g_demo.size_inc_label, demo_get_ui_font(), LV_PART_MAIN);
    lv_obj_center(g_demo.size_inc_label);
    lv_obj_add_event_cb(g_demo.size_inc_btn, btn_size_inc_cb, LV_EVENT_CLICKED, NULL);

    g_demo.font_list_btn = lv_btn_create(g_demo.bottom_bar);
    lv_obj_set_size(g_demo.font_list_btn, 118, 44);
    lv_obj_set_style_bg_color(g_demo.font_list_btn, lv_color_hex(0x2196F3), LV_PART_MAIN);
    lv_obj_set_style_radius(g_demo.font_list_btn, 8, LV_PART_MAIN);
    g_demo.font_list_label = lv_label_create(g_demo.font_list_btn);
    lv_obj_set_style_text_font(g_demo.font_list_label, demo_get_ui_font(), LV_PART_MAIN);
    lv_label_set_text(g_demo.font_list_label, "Fonts");
    lv_obj_center(g_demo.font_list_label);
    lv_obj_add_event_cb(g_demo.font_list_btn, btn_show_font_list_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_move_to_index(g_demo.size_dec_btn, 0);

}

#endif /* LV_USING_FREETYPE_ENGINE */

/**
 * @brief Start the dynamic font switch demo
 *
 * This demo shows how to:
 * 1. Initialize the lvsf font manager
 * 2. Load TTF font files from filesystem
 * 3. Switch fonts dynamically at runtime
 * 4. Adjust font size
 * 5. Validate selected font objects by size
 */
void lv_example_font_switch_demo(void)
{
#ifdef LV_USING_FREETYPE_ENGINE
    rt_kprintf("[font_demo] start\n");
    demo_log_mem("demo start");

    if (g_demo.main_cont && lv_obj_is_valid(g_demo.main_cont))
    {
        lv_obj_del(g_demo.main_cont);
        g_demo.main_cont = NULL;
        g_demo.font_list_page = NULL;
    }

    /* Manager state survives a re-init, so the font of the previous run has to
     * go now - nothing displays it any more (the UI above is gone), and after
     * the memset below the demo would not even know its name. */
    if (g_demo.current_font_name[0] &&
            strcmp(g_demo.current_font_name, "Built-in") != 0)
    {
        lvsf_font_set_enable(g_demo.current_font_name, 0);
        lvsf_font_unload_ex(g_demo.current_font_path);
    }

    memset(&g_demo, 0, sizeof(g_demo));
    g_demo.current_font_size = 24;
    strncpy(g_demo.current_font_name, "Built-in", sizeof(g_demo.current_font_name) - 1);

    /* Use the project configured FreeType cache size. */
    lvsf_font_manager_init(0);

    /* Executes shell-posted commands in the LVGL thread. */
    if (!g_demo_cmd_timer)
    {
        g_demo_cmd_timer = lv_timer_create(demo_cmd_timer_cb, 100, NULL);
    }

    /* Load and enable the default external font; fall back to built-in
     * fonts when it is missing or broken. */
    if (file_exists(FONT_DEMO_DEFAULT_FONT))
    {
        if (load_ttf_font(FONT_DEMO_DEFAULT_FONT) == 0)
        {
            demo_enable_selected_font();
        }
        else
        {
            rt_kprintf("[font_demo] Using built-in fonts\n");
        }
    }
    else
    {
        rt_kprintf("[font_demo] Default font not found: %s\n", FONT_DEMO_DEFAULT_FONT);
    }

    create_ui();
    update_font_status();
    demo_log_mem("demo ready");
    rt_kprintf("[font_demo] ready\n");
#else
    rt_kprintf("\n========================================\n");
    rt_kprintf("  LVGL v8 Dynamic Font Switch Demo\n");
    rt_kprintf("========================================\n");
    rt_kprintf("[font_demo] ERROR: FreeType engine not enabled!\n");
    rt_kprintf("[font_demo] Please enable LV_USING_FREETYPE_ENGINE in Kconfig\n");
    rt_kprintf("[font_demo] ========================================\n");
#endif
}

#ifdef RT_USING_FINSH
#include <finsh.h>
#ifdef LV_USING_FREETYPE_ENGINE
/* All three commands only post a request; the demo's lv_timer runs the
 * actual LVGL/font-manager work in the LVGL thread (see demo_cmd_timer_cb). */
static void font_demo_restart(void)
{
    demo_post_cmd(DEMO_CMD_RESTART);
}
MSH_CMD_EXPORT_ALIAS(font_demo_restart, lv_example_font_switch_demo, Start dynamic font switch demo);
MSH_CMD_EXPORT(font_demo_enable_font, Enable selected demo font and set lvsf order);
MSH_CMD_EXPORT(font_demo_verify_font, Create selected demo font object for diagnosis);
#else
MSH_CMD_EXPORT(lv_example_font_switch_demo, Start dynamic font switch demo);
#endif
#endif
