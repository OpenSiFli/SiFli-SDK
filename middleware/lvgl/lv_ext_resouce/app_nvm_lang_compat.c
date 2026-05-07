/*
 ******************************************************************************
 * @file   app_nvm_lang_compat.c
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

#include <stdbool.h>
#include <string.h>
#include "app_nvm_lang_compat.h"
#include "lv_ext_resource_manager.h"
#include "share_prefs.h"

#define APP_LOCALE_PREFS_NAME    "app_locale"
#define APP_LOCALE_PREFS_KEY     "locale_lang"

static char s_locale_lang[MAX_LANG_NAME_LEN];
static char s_default_locale[MAX_LANG_NAME_LEN];
static bool s_is_initialized;

static void app_nvm_lang_cache_copy(char *dst, const char *src, size_t len)
{
    size_t copy_len = 0;

    memset(dst, 0, MAX_LANG_NAME_LEN);

    if (NULL == src)
    {
        return;
    }

    while ((copy_len < (MAX_LANG_NAME_LEN - 1))
            && (copy_len < len)
            && (src[copy_len] != '\0'))
    {
        dst[copy_len] = src[copy_len];
        copy_len++;
    }
}

static const char *app_nvm_lang_default_locale_get(void)
{
    const lv_i18n_lang_pack_t *lang_pack = lv_ext_get_lang_pack(NULL);

    if (lang_pack && *lang_pack && (*lang_pack)->locale)
    {
        return (*lang_pack)->locale;
    }

    return lv_ext_get_locale();
}

static void app_nvm_lang_compat_ensure_init(void)
{
    if (s_is_initialized)
    {
        return;
    }

    app_nvm_lang_compat_init(app_nvm_lang_default_locale_get());
}

void app_nvm_lang_compat_init(const char *default_locale)
{
    if ((NULL == default_locale) || (0 == default_locale[0]))
    {
        default_locale = app_nvm_lang_default_locale_get();
    }

    app_nvm_lang_cache_copy(s_default_locale,
                            default_locale,
                            default_locale ? MAX_LANG_NAME_LEN : 0);
    app_nvm_lang_cache_copy(s_locale_lang, s_default_locale, MAX_LANG_NAME_LEN);

#ifdef BSP_SHARE_PREFS
    share_prefs_t *prefs = share_prefs_open(APP_LOCALE_PREFS_NAME, SHAREPREFS_MODE_PRIVATE);

    if (prefs)
    {
        int32_t len = share_prefs_get_string(prefs,
                                             APP_LOCALE_PREFS_KEY,
                                             s_locale_lang,
                                             sizeof(s_locale_lang) - 1);
        share_prefs_close(prefs);

        if (len > 0)
        {
            if (len >= (int32_t)sizeof(s_locale_lang))
            {
                len = sizeof(s_locale_lang) - 1;
            }
            s_locale_lang[len] = 0;
        }
        else
        {
            app_nvm_lang_cache_copy(s_locale_lang, s_default_locale, MAX_LANG_NAME_LEN);
        }
    }
#endif

    s_is_initialized = true;
}

const char *app_nvm_lang_get_locale_lang(void)
{
    app_nvm_lang_compat_ensure_init();

    if (s_locale_lang[0])
    {
        return s_locale_lang;
    }

    return s_default_locale;
}

void app_nvm_lang_copy_update_locale_lang(const void *buf, size_t len, uint8_t immediately)
{
    char locale[MAX_LANG_NAME_LEN];

    app_nvm_lang_compat_ensure_init();

    (void)immediately;

    app_nvm_lang_cache_copy(locale, (const char *)buf, len);
    if (0 == memcmp(s_locale_lang, locale, sizeof(s_locale_lang)))
    {
        return;
    }

    app_nvm_lang_cache_copy(s_locale_lang, locale, sizeof(locale));

#ifdef BSP_SHARE_PREFS
    share_prefs_t *prefs = share_prefs_open(APP_LOCALE_PREFS_NAME, SHAREPREFS_MODE_PRIVATE);

    if (prefs)
    {
        share_prefs_set_string(prefs, APP_LOCALE_PREFS_KEY, s_locale_lang);
        share_prefs_close(prefs);
    }
#endif
}
