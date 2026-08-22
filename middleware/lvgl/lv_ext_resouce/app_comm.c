/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 ******************************************************************************
 * @file   app_comm.c
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "app_comm.h"

#include <sys/stat.h>

#include "app_lang.h"
#include "lvsf_resource.h"
#include "app_nvm_lang_compat.h"
#include "log.h"

#ifdef RT_USING_DFS
#include <dfs_posix.h>
#endif

#ifdef APP_TOOL_SUPPORT
#include "app_tool_comm.h"
#endif

/**
 * @brief  Initalize local language.
 *         1) Get language list(ex_lang) from flash's language packet if exist.
 *         2) Set default language.
 */
void app_locale_lang_init(void)
{
#if !defined(BSP_USING_PC_SIMULATOR) && defined(RT_USING_DFS)
    app_lang_load_pack_list(LANG_INSTALLER_PATH);
    app_lang_install_pack(LANG_INSTALLER_PATH, nvm_sys_get(locale_lang));
#endif

    lv_ext_set_locale(NULL, nvm_sys_get(locale_lang));
#ifdef APP_TOOL_SUPPORT
    sfat_set_lang_type(nvm_sys_get(locale_lang));
#endif
}

/**
 * @brief  Updated and set local language.
 * @param  locale Locale language name.
 */
void app_locale_lang_update(const char *locale)
{
    LOG_I("%s: %s", __func__, locale);

    if (NULL == locale)
    {
        locale = lv_i18n_lang_pack[0]->locale;
    }

#if !defined(BSP_USING_PC_SIMULATOR) && defined(RT_USING_DFS)
    app_lang_install_pack(LANG_INSTALLER_PATH, locale);
#endif
    lv_ext_set_locale(NULL, locale);
    nvm_sys_copy_update(locale_lang, (void *)locale, MAX_LANG_NAME_LEN, 0);
#ifdef APP_SETTING_USED
    extern void setting_lang_update(void);
    setting_lang_update();
#endif
#ifdef APP_DLMODULE_APP_USED
    extern void dynamic_app_lang_update(void);
    dynamic_app_lang_update();
#endif
#ifdef APP_WF_USED
    wf_lang_update();
#ifdef APP_DLMODULE_WF_USED
    extern void dynamic_wf_lang_update(void);
    dynamic_wf_lang_update();
#endif
#endif
#ifdef MENU_FRAMEWORK
    app_menu_lang_update();
#endif

#ifdef APP_TOOL_SUPPORT
    sfat_set_lang_type(locale);
#endif
}

char *app_strdup(const char *src)
{
    size_t len;
    char *dst;

    if (!src) return NULL;
    len = strlen(src) + 1;
    dst = rt_malloc(len);
    if (!dst) return NULL;
    memcpy(dst, src, len);
    return dst;
}

int app_get_o_directory(void)
{
#ifdef RT_USING_DFS
    return O_DIRECTORY;
#else
    return 0;
#endif
}

int app_mkdir(const char *path)
{
#ifdef RT_USING_DFS
    char tmp[256];
    char *p;

    if (!path || !path[0]) return -RT_ERROR;

    rt_snprintf(tmp, sizeof(tmp), "%s", path);
    for (p = tmp + 1; *p; p++)
    {
        if (*p != '/') continue;

        *p = '\0';
        if (tmp[0] != '\0') mkdir(tmp, 0);
        *p = '/';
    }

    if (mkdir(tmp, 0) == 0 || access(tmp, 0) == 0)
        return RT_EOK;
#else
    LV_UNUSED(path);
#endif

    return -RT_ERROR;
}

size_t app_get_file_size(const char *file)
{
#ifdef RT_USING_DFS
    struct stat st;

    if (!file || stat(file, &st) != 0)
        return 0;

    return st.st_size > 0 ? (size_t)st.st_size : 0;
#else
    LV_UNUSED(file);
    return 0;
#endif
}

bool app_file_check_valid(const char *path)
{
#ifdef RT_USING_DFS
    return (path && access(path, 0) == 0);
#else
    LV_UNUSED(path);
    return false;
#endif
}

bool app_path_check_valid(const char *path)
{
    return app_file_check_valid(path);
}

lv_res_t lv_async_call_ext(lv_async_cb_t cb, void *user_data, uint32_t delay_ms)
{
    LV_UNUSED(delay_ms);
    return lv_async_call(cb, user_data);
}
