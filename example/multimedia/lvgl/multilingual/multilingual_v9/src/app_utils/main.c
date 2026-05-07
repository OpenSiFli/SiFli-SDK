/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "rtthread.h"
#include "rtdevice.h"
#include "bf0_hal.h"
#include "board.h"
#include "drv_io.h"
#include "littlevgl2rtt.h"
#include "app_comm.h"
#include "app_lang.h"
#include "app_module.h"
#include "app_nvm_lang_compat.h"
#include "lv_ext_resource_manager.h"
#include "lvsf/lv_ex_data.h"
#include "lv_tiny_ttf.h"

#ifdef RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
    #ifndef BSP_USING_PC_SIMULATOR
        #include "drv_flash.h"
    #endif
#endif

#define DEMO_EXTERNAL_LOCALE_ZH_TW  "zh_tw"
#define DEMO_STATUS_BUF_LEN         192
#define DEMO_INFO_BUF_LEN           256
#define DEMO_LOCALE_LIST_BUF_LEN    384

#if LV_HOR_RES_MAX > 350
enum
{
    DEMO_FONT_SMALL = 16,
    DEMO_FONT_NORMAL = 20,
    DEMO_FONT_TITLE = 28,
};
#else
enum
{
    DEMO_FONT_SMALL = 12,
    DEMO_FONT_NORMAL = 16,
    DEMO_FONT_TITLE = 24,
};
#endif

typedef enum
{
    DEMO_ACTION_SWITCH_EN = 0,
    DEMO_ACTION_SWITCH_ZH,
    DEMO_ACTION_SWITCH_FS_ZH_TW,
    DEMO_ACTION_RESCAN_FS,
    DEMO_ACTION_COUNT,
} demo_action_t;

typedef struct
{
    lv_obj_t *btn;
    lv_obj_t *label;
    demo_action_t action;
} demo_button_t;

typedef struct
{
    lv_obj_t *page;
    lv_obj_t *title;
    lv_obj_t *desc;
    lv_obj_t *locale_info;
    lv_obj_t *nvm_info;
    lv_obj_t *pack_info;
    lv_obj_t *locale_list;
    lv_obj_t *installer_info;
    lv_obj_t *installer_list;
    lv_obj_t *preview_title;
    lv_obj_t *preview_labels[4];
    lv_obj_t *builtin_title;
    lv_obj_t *external_title;
    lv_obj_t *status;
    demo_button_t buttons[DEMO_ACTION_COUNT];
    bool has_fs_zh_tw;
    char locale_info_buf[DEMO_INFO_BUF_LEN];
    char nvm_info_buf[DEMO_INFO_BUF_LEN];
    char pack_info_buf[DEMO_INFO_BUF_LEN];
    char installer_info_buf[DEMO_INFO_BUF_LEN];
    char locale_list_buf[DEMO_LOCALE_LIST_BUF_LEN];
    char installer_list_buf[DEMO_LOCALE_LIST_BUF_LEN];
    char status_buf[DEMO_STATUS_BUF_LEN];
} demo_ui_t;

static demo_ui_t s_demo_ui;
extern const unsigned char DroidSansFallback[];
extern const int DroidSansFallback_size;

typedef struct
{
    uint16_t size;
    lv_font_t *font;
} demo_font_cache_t;

static demo_font_cache_t s_font_cache[] =
{
    {DEMO_FONT_SMALL, NULL},
    {DEMO_FONT_NORMAL, NULL},
    {DEMO_FONT_TITLE, NULL},
};

#if defined(RT_USING_DFS) && defined(FS_REGION_START_ADDR) && defined(FS_REGION_SIZE)
#define DEMO_FS_ROOT_DEVICE "langfs"

static int demo_mnt_init(void)
{
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, DEMO_FS_ROOT_DEVICE);
    if (dfs_mount(DEMO_FS_ROOT_DEVICE, "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", DEMO_FS_ROOT_DEVICE) == 0)
        {
            rt_kprintf("make elm fs on flash success, mount again\n");
            if (dfs_mount(DEMO_FS_ROOT_DEVICE, "/", "elm", 0, 0) == 0)
            {
                rt_kprintf("mount fs on flash success\n");
            }
            else
            {
                rt_kprintf("mount fs on flash fail after mkfs\n");
            }
        }
        else
        {
            rt_kprintf("dfs_mkfs elm flash fail\n");
        }
    }

    return RT_EOK;
}
INIT_ENV_EXPORT(demo_mnt_init);
#endif

static void demo_set_label_text(lv_obj_t *obj, const char *text, uint16_t size, lv_color_t color)
{
    const lv_font_t *font = LV_FONT_DEFAULT;

    for (uint32_t i = 0; i < sizeof(s_font_cache) / sizeof(s_font_cache[0]); i++)
    {
        if (s_font_cache[i].size != size)
        {
            continue;
        }

        if (NULL == s_font_cache[i].font)
        {
            s_font_cache[i].font = lv_tiny_ttf_create_data(DroidSansFallback, DroidSansFallback_size, size);
        }

        if (s_font_cache[i].font)
        {
            font = s_font_cache[i].font;
        }
        break;
    }

    lv_label_set_text(obj, text);
    lv_obj_set_style_text_font(obj, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(obj, color, LV_PART_MAIN);
}

static void demo_append_list_item(char *buf, size_t buf_size, bool *first, const char *item)
{
    size_t used;
    size_t remain;

    if ((NULL == buf) || (0 == buf_size) || (NULL == first) || (NULL == item))
    {
        return;
    }

    used = strlen(buf);
    if (used >= buf_size)
    {
        return;
    }

    remain = buf_size - used;
    if (remain <= 1)
    {
        return;
    }

    snprintf(buf + used,
             remain,
             "%s%s",
             *first ? "" : ", ",
             item);
    *first = false;
}

static void demo_set_status(const char *text)
{
    rt_strncpy(s_demo_ui.status_buf, text, sizeof(s_demo_ui.status_buf) - 1);
    s_demo_ui.status_buf[sizeof(s_demo_ui.status_buf) - 1] = '\0';
    demo_set_label_text(s_demo_ui.status, s_demo_ui.status_buf, DEMO_FONT_SMALL, lv_color_hex(0xFFD166));
}

static void demo_collect_registered_locale_info(uint32_t *pack_count, uint32_t *locale_count, char *buf, size_t buf_size)
{
    bool first = true;
    lv_lang_pack_node_t *iter;

    *pack_count = 0;
    *locale_count = 0;
    buf[0] = '\0';

    LV_EXT_LANG_PACK_LIST_ITER(NULL, iter)
    {
        const lv_i18n_lang_pack_t *lang_pack_iter;

        (*pack_count)++;
        LV_EXT_LANG_PACK_ITER(iter, lang_pack_iter)
        {
            const char *locale = LV_EXT_LANG_PACK_ITER_GET_NAME(lang_pack_iter);

            demo_append_list_item(buf, buf_size, &first, locale);
            (*locale_count)++;
        }
    }

    if (buf[0] == '\0')
    {
        rt_strncpy(buf, "(none)", buf_size - 1);
        buf[buf_size - 1] = '\0';
    }
}

static void demo_collect_installer_locale_info(uint32_t *locale_count, char *buf, size_t buf_size)
{
    bool first = true;

    *locale_count = 0;
    s_demo_ui.has_fs_zh_tw = false;
    buf[0] = '\0';

#if !defined(BSP_USING_PC_SIMULATOR) && defined(RT_USING_DFS)
    {
        uint32_t i = 0;
        const char *locale;

        app_lang_load_pack_list(LANG_INSTALLER_PATH);
        while ((locale = app_lang_pack_iterator(&i)) != NULL)
        {
            demo_append_list_item(buf, buf_size, &first, locale);
            (*locale_count)++;

            if (0 == strcmp(locale, DEMO_EXTERNAL_LOCALE_ZH_TW))
            {
                s_demo_ui.has_fs_zh_tw = true;
            }
        }
    }
#endif

    if (buf[0] == '\0')
    {
#if !defined(BSP_USING_PC_SIMULATOR) && defined(RT_USING_DFS)
        rt_strncpy(buf, "(none)", buf_size - 1);
#else
        rt_strncpy(buf, "(dfs disabled)", buf_size - 1);
#endif
        buf[buf_size - 1] = '\0';
    }
}

static bool demo_is_external_locale(const char *locale)
{
    if (NULL == locale)
    {
        return false;
    }

    return (0 == strcmp(locale, DEMO_EXTERNAL_LOCALE_ZH_TW));
}

static bool demo_has_external_locale(const char *locale)
{
    char tmp_buf[DEMO_LOCALE_LIST_BUF_LEN];
    uint32_t locale_count = 0;

    demo_collect_installer_locale_info(&locale_count, tmp_buf, sizeof(tmp_buf));
    (void)locale_count;

    if (0 == strcmp(locale, DEMO_EXTERNAL_LOCALE_ZH_TW))
    {
        return s_demo_ui.has_fs_zh_tw;
    }

    return false;
}

static void demo_update_button_state(void)
{
    uint8_t i;

    for (i = 0; i < DEMO_ACTION_COUNT; i++)
    {
        lv_obj_clear_state(s_demo_ui.buttons[i].btn, LV_STATE_DISABLED);
    }

#if !defined(BSP_USING_PC_SIMULATOR) && defined(RT_USING_DFS)
    if (!s_demo_ui.has_fs_zh_tw)
    {
        lv_obj_add_state(s_demo_ui.buttons[DEMO_ACTION_SWITCH_FS_ZH_TW].btn, LV_STATE_DISABLED);
    }
#else
    lv_obj_add_state(s_demo_ui.buttons[DEMO_ACTION_SWITCH_FS_ZH_TW].btn, LV_STATE_DISABLED);
    lv_obj_add_state(s_demo_ui.buttons[DEMO_ACTION_RESCAN_FS].btn, LV_STATE_DISABLED);
#endif
}

static void demo_refresh_ui(void)
{
    uint32_t pack_count;
    uint32_t locale_count;
    uint32_t installer_locale_count;
    lv_coord_t screen_width = lv_disp_get_hor_res(NULL);
    lv_coord_t button_width = (screen_width - 36) / 2;
    lv_coord_t y = 12;
    uint8_t i;

    demo_collect_registered_locale_info(&pack_count,
                                        &locale_count,
                                        s_demo_ui.locale_list_buf,
                                        sizeof(s_demo_ui.locale_list_buf));
    demo_collect_installer_locale_info(&installer_locale_count,
                                       s_demo_ui.installer_list_buf,
                                       sizeof(s_demo_ui.installer_list_buf));

    demo_set_label_text(s_demo_ui.title,
                        LV_EXT_STR_GET_BY_KEY(demo_title, "LVGL Multilingual Demo"),
                        DEMO_FONT_TITLE,
                        lv_color_hex(0xFFFFFF));
    demo_set_label_text(s_demo_ui.desc,
                        LV_EXT_STR_GET_BY_KEY(demo_desc, "This page validates builtin packs, filesystem external packs, locale switching, installer rescan, and NVM persistence using the current lv_ext_resouce/app_lang/app_comm flow."),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0xD9E2EC));

    snprintf(s_demo_ui.locale_info_buf,
             sizeof(s_demo_ui.locale_info_buf),
             LV_EXT_STR_GET_BY_KEY(demo_locale_fmt, "Current locale: %s"),
             lv_ext_get_locale() ? lv_ext_get_locale() : "(null)");
    demo_set_label_text(s_demo_ui.locale_info,
                        s_demo_ui.locale_info_buf,
                        DEMO_FONT_SMALL,
                        lv_color_hex(0x8ECAE6));

    snprintf(s_demo_ui.nvm_info_buf,
             sizeof(s_demo_ui.nvm_info_buf),
             LV_EXT_STR_GET_BY_KEY(demo_nvm_fmt, "Stored locale (NVM/share_prefs): %s"),
             nvm_sys_get(locale_lang) ? nvm_sys_get(locale_lang) : "(null)");
    demo_set_label_text(s_demo_ui.nvm_info,
                        s_demo_ui.nvm_info_buf,
                        DEMO_FONT_SMALL,
                        lv_color_hex(0x8ECAE6));

    snprintf(s_demo_ui.pack_info_buf,
             sizeof(s_demo_ui.pack_info_buf),
             LV_EXT_STR_GET_BY_KEY(demo_pack_count_fmt, "Registered pack nodes: %d, locales: %d"),
             (int)pack_count,
             (int)locale_count);
    demo_set_label_text(s_demo_ui.pack_info,
                        s_demo_ui.pack_info_buf,
                        DEMO_FONT_SMALL,
                        lv_color_hex(0x8ECAE6));

    snprintf(s_demo_ui.locale_info_buf,
             sizeof(s_demo_ui.locale_info_buf),
             LV_EXT_STR_GET_BY_KEY(demo_locales_fmt, "Registered locales: %s"),
             s_demo_ui.locale_list_buf);
    demo_set_label_text(s_demo_ui.locale_list,
                        s_demo_ui.locale_info_buf,
                        DEMO_FONT_SMALL,
                        lv_color_hex(0x8ECAE6));

    snprintf(s_demo_ui.installer_info_buf,
             sizeof(s_demo_ui.installer_info_buf),
             LV_EXT_STR_GET_BY_KEY(demo_installer_count_fmt, "Installer locales on filesystem: %d"),
             (int)installer_locale_count);
    demo_set_label_text(s_demo_ui.installer_info,
                        s_demo_ui.installer_info_buf,
                        DEMO_FONT_SMALL,
                        lv_color_hex(0x90E0EF));

    snprintf(s_demo_ui.installer_info_buf,
             sizeof(s_demo_ui.installer_info_buf),
             LV_EXT_STR_GET_BY_KEY(demo_installer_locales_fmt, "Installer locale list: %s"),
             s_demo_ui.installer_list_buf);
    demo_set_label_text(s_demo_ui.installer_list,
                        s_demo_ui.installer_info_buf,
                        DEMO_FONT_SMALL,
                        lv_color_hex(0x90E0EF));

    demo_set_label_text(s_demo_ui.preview_title,
                        LV_EXT_STR_GET_BY_KEY(demo_preview_title, "What this page proves"),
                        DEMO_FONT_NORMAL,
                        lv_color_hex(0xFFFFFF));
    demo_set_label_text(s_demo_ui.preview_labels[0],
                        LV_EXT_STR_GET_BY_KEY(demo_preview_line_0, "Builtin locales come from strings/*.json compiled into lang_pack.c."),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0xFFFFFF));
    demo_set_label_text(s_demo_ui.preview_labels[1],
                        LV_EXT_STR_GET_BY_KEY(demo_preview_line_1, "External locales come from /ex/resource/lang/installer/*.bin and are loaded by app_lang_install_pack()."),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0xD9E2EC));
    demo_set_label_text(s_demo_ui.preview_labels[2],
                        LV_EXT_STR_GET_BY_KEY(demo_preview_line_2, "Tap Rescan after replacing installer .bin to simulate OTA or Bluetooth update of external language packs."),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0x7FDBFF));
    demo_set_label_text(s_demo_ui.preview_labels[3],
                        LV_EXT_STR_GET_BY_KEY(demo_preview_line_3, "Switch a locale, reboot, then compare Stored locale and startup language to verify NVM persistence."),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0xB5F2B4));

    demo_set_label_text(s_demo_ui.builtin_title,
                        LV_EXT_STR_GET_BY_KEY(demo_builtin_title, "Builtin locale switch"),
                        DEMO_FONT_NORMAL,
                        lv_color_hex(0xFFFFFF));
    demo_set_label_text(s_demo_ui.external_title,
                        LV_EXT_STR_GET_BY_KEY(demo_external_title, "External pack switch"),
                        DEMO_FONT_NORMAL,
                        lv_color_hex(0xFFFFFF));

    demo_set_label_text(s_demo_ui.buttons[DEMO_ACTION_SWITCH_EN].label,
                        LV_EXT_STR_GET_BY_KEY(demo_btn_switch_en, "Switch en_us"),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0xFFFFFF));
    demo_set_label_text(s_demo_ui.buttons[DEMO_ACTION_SWITCH_ZH].label,
                        LV_EXT_STR_GET_BY_KEY(demo_btn_switch_zh, "Switch zh_cn"),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0xFFFFFF));
    demo_set_label_text(s_demo_ui.buttons[DEMO_ACTION_SWITCH_FS_ZH_TW].label,
                        LV_EXT_STR_GET_BY_KEY(demo_btn_switch_fs_zh_tw, "Switch zh_tw"),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0xFFFFFF));
    demo_set_label_text(s_demo_ui.buttons[DEMO_ACTION_RESCAN_FS].label,
                        LV_EXT_STR_GET_BY_KEY(demo_btn_rescan, "Rescan installer"),
                        DEMO_FONT_SMALL,
                        lv_color_hex(0xFFFFFF));

    demo_update_button_state();

    lv_obj_set_width(s_demo_ui.title, screen_width - 24);
    lv_obj_align(s_demo_ui.title, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.title);
    y += lv_obj_get_height(s_demo_ui.title) + 6;

    lv_obj_set_width(s_demo_ui.desc, screen_width - 24);
    lv_obj_align(s_demo_ui.desc, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.desc);
    y += lv_obj_get_height(s_demo_ui.desc) + 8;

    lv_obj_set_width(s_demo_ui.locale_info, screen_width - 24);
    lv_obj_align(s_demo_ui.locale_info, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.locale_info);
    y += lv_obj_get_height(s_demo_ui.locale_info) + 4;

    lv_obj_set_width(s_demo_ui.nvm_info, screen_width - 24);
    lv_obj_align(s_demo_ui.nvm_info, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.nvm_info);
    y += lv_obj_get_height(s_demo_ui.nvm_info) + 4;

    lv_obj_set_width(s_demo_ui.pack_info, screen_width - 24);
    lv_obj_align(s_demo_ui.pack_info, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.pack_info);
    y += lv_obj_get_height(s_demo_ui.pack_info) + 4;

    lv_obj_set_width(s_demo_ui.locale_list, screen_width - 24);
    lv_obj_align(s_demo_ui.locale_list, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.locale_list);
    y += lv_obj_get_height(s_demo_ui.locale_list) + 4;

    lv_obj_set_width(s_demo_ui.installer_info, screen_width - 24);
    lv_obj_align(s_demo_ui.installer_info, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.installer_info);
    y += lv_obj_get_height(s_demo_ui.installer_info) + 4;

    lv_obj_set_width(s_demo_ui.installer_list, screen_width - 24);
    lv_obj_align(s_demo_ui.installer_list, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.installer_list);
    y += lv_obj_get_height(s_demo_ui.installer_list) + 10;

    lv_obj_set_width(s_demo_ui.preview_title, screen_width - 24);
    lv_obj_align(s_demo_ui.preview_title, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.preview_title);
    y += lv_obj_get_height(s_demo_ui.preview_title) + 4;

    for (i = 0; i < sizeof(s_demo_ui.preview_labels) / sizeof(s_demo_ui.preview_labels[0]); i++)
    {
        lv_obj_set_width(s_demo_ui.preview_labels[i], screen_width - 24);
        lv_obj_align(s_demo_ui.preview_labels[i], LV_ALIGN_TOP_LEFT, 12, y);
        lv_obj_update_layout(s_demo_ui.preview_labels[i]);
        y += lv_obj_get_height(s_demo_ui.preview_labels[i]) + 2;
    }
    y += 8;

    lv_obj_set_width(s_demo_ui.builtin_title, screen_width - 24);
    lv_obj_align(s_demo_ui.builtin_title, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.builtin_title);
    y += lv_obj_get_height(s_demo_ui.builtin_title) + 6;

    for (i = 0; i < 2; i++)
    {
        lv_obj_set_size(s_demo_ui.buttons[i].btn, button_width, 44);
        lv_obj_align(s_demo_ui.buttons[i].btn,
                     LV_ALIGN_TOP_LEFT,
                     12 + (i % 2) * (button_width + 12),
                     y);
    }
    y += 52;

    lv_obj_set_width(s_demo_ui.external_title, screen_width - 24);
    lv_obj_align(s_demo_ui.external_title, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.external_title);
    y += lv_obj_get_height(s_demo_ui.external_title) + 6;

    for (i = 2; i < DEMO_ACTION_COUNT; i++)
    {
        lv_obj_set_size(s_demo_ui.buttons[i].btn, button_width, 44);
        lv_obj_align(s_demo_ui.buttons[i].btn,
                     LV_ALIGN_TOP_LEFT,
                     12 + (i - 2) * (button_width + 12),
                     y);
    }
    y += 52;

    lv_obj_set_width(s_demo_ui.status, screen_width - 24);
    lv_obj_align(s_demo_ui.status, LV_ALIGN_TOP_LEFT, 12, y);
    lv_obj_update_layout(s_demo_ui.status);
}

static void demo_apply_locale(const char *locale)
{
    if (demo_is_external_locale(locale) && !demo_has_external_locale(locale))
    {
        snprintf(s_demo_ui.status_buf,
                 sizeof(s_demo_ui.status_buf),
                 LV_EXT_STR_GET_BY_KEY(demo_status_missing_pack_fmt, "Installer package missing: %s"),
                 locale);
        demo_set_status(s_demo_ui.status_buf);
        demo_refresh_ui();
        return;
    }

    app_locale_lang_update(locale);

    if ((lv_ext_get_locale() != NULL) && (0 == strcmp(lv_ext_get_locale(), locale)))
    {
        snprintf(s_demo_ui.status_buf,
                 sizeof(s_demo_ui.status_buf),
                 LV_EXT_STR_GET_BY_KEY(demo_status_switched_fmt, "Switched locale: %s"),
                 locale);
    }
    else
    {
        snprintf(s_demo_ui.status_buf,
                 sizeof(s_demo_ui.status_buf),
                 LV_EXT_STR_GET_BY_KEY(demo_status_switch_failed_fmt, "Switch failed: %s"),
                 locale);
    }

    demo_set_status(s_demo_ui.status_buf);
    demo_refresh_ui();
}

static void demo_rescan_installer(void)
{
#if !defined(BSP_USING_PC_SIMULATOR) && defined(RT_USING_DFS)
    uint32_t installer_locale_count = 0;
    const char *current_locale = lv_ext_get_locale();

    demo_collect_installer_locale_info(&installer_locale_count,
                                       s_demo_ui.installer_list_buf,
                                       sizeof(s_demo_ui.installer_list_buf));

    if (demo_is_external_locale(current_locale) && demo_has_external_locale(current_locale))
    {
        app_locale_lang_update(current_locale);
        snprintf(s_demo_ui.status_buf,
                 sizeof(s_demo_ui.status_buf),
                 LV_EXT_STR_GET_BY_KEY(demo_status_rescan_reload_fmt, "Installer rescanned and external locale reloaded: %s"),
                 current_locale);
    }
    else
    {
        snprintf(s_demo_ui.status_buf,
                 sizeof(s_demo_ui.status_buf),
                 LV_EXT_STR_GET_BY_KEY(demo_status_rescan_ok_fmt, "Installer rescanned: %d locale(s)"),
                 (int)installer_locale_count);
    }
#else
    rt_strncpy(s_demo_ui.status_buf,
               LV_EXT_STR_GET_BY_KEY(demo_status_dfs_unavailable, "Filesystem external language-pack support is unavailable in this build."),
               sizeof(s_demo_ui.status_buf) - 1);
    s_demo_ui.status_buf[sizeof(s_demo_ui.status_buf) - 1] = '\0';
#endif

    demo_set_status(s_demo_ui.status_buf);
    demo_refresh_ui();
}

static void demo_btn_event_cb(lv_event_t *e)
{
    demo_action_t action = (demo_action_t)(uintptr_t)lv_event_get_user_data(e);

    switch (action)
    {
    case DEMO_ACTION_SWITCH_EN:
        demo_apply_locale("en_us");
        break;

    case DEMO_ACTION_SWITCH_ZH:
        demo_apply_locale("zh_cn");
        break;

    case DEMO_ACTION_SWITCH_FS_ZH_TW:
        demo_apply_locale(DEMO_EXTERNAL_LOCALE_ZH_TW);
        break;

    case DEMO_ACTION_RESCAN_FS:
        demo_rescan_installer();
        break;

    default:
        break;
    }
}

static lv_obj_t *demo_create_button(lv_obj_t *parent, demo_action_t action)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_t *label = lv_label_create(btn);

    s_demo_ui.buttons[action].btn = btn;
    s_demo_ui.buttons[action].label = label;
    s_demo_ui.buttons[action].action = action;

    lv_obj_add_event_cb(btn, demo_btn_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)action);
    lv_obj_center(label);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x243B53), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x486581), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x334E68), LV_PART_MAIN | LV_STATE_DISABLED);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 10, LV_PART_MAIN);

    return btn;
}

static void demo_create_ui(void)
{
    uint8_t i;

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x10131A), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, LV_PART_MAIN);

    s_demo_ui.page = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_demo_ui.page, lv_pct(100), lv_pct(100));
    lv_obj_center(s_demo_ui.page);
    lv_obj_set_style_bg_color(s_demo_ui.page, lv_color_hex(0x17212B), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_demo_ui.page, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_demo_ui.page, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_demo_ui.page, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(s_demo_ui.page, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_demo_ui.page, LV_SCROLLBAR_MODE_AUTO);

    s_demo_ui.title = lv_label_create(s_demo_ui.page);
    s_demo_ui.desc = lv_label_create(s_demo_ui.page);
    s_demo_ui.locale_info = lv_label_create(s_demo_ui.page);
    s_demo_ui.nvm_info = lv_label_create(s_demo_ui.page);
    s_demo_ui.pack_info = lv_label_create(s_demo_ui.page);
    s_demo_ui.locale_list = lv_label_create(s_demo_ui.page);
    s_demo_ui.installer_info = lv_label_create(s_demo_ui.page);
    s_demo_ui.installer_list = lv_label_create(s_demo_ui.page);
    s_demo_ui.preview_title = lv_label_create(s_demo_ui.page);
    s_demo_ui.builtin_title = lv_label_create(s_demo_ui.page);
    s_demo_ui.external_title = lv_label_create(s_demo_ui.page);
    s_demo_ui.status = lv_label_create(s_demo_ui.page);

    for (i = 0; i < sizeof(s_demo_ui.preview_labels) / sizeof(s_demo_ui.preview_labels[0]); i++)
    {
        s_demo_ui.preview_labels[i] = lv_label_create(s_demo_ui.page);
    }

    for (i = 0; i < DEMO_ACTION_COUNT; i++)
    {
        demo_create_button(s_demo_ui.page, (demo_action_t)i);
    }

    demo_set_status(LV_EXT_STR_GET_BY_KEY(demo_status_ready, "Ready: builtin locale switch, external installer scan, and NVM display are available."));
    demo_refresh_ui();
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }

    lv_ex_data_pool_init();
    resource_init();
    app_locale_lang_init();

    demo_create_ui();

    while (1)
    {
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }

    return RT_EOK;
}
