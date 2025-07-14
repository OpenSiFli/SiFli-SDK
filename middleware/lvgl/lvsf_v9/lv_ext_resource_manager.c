/**
  ******************************************************************************
  * @file   lv_ext_resource_manager.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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
#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"
#include <string.h>
#ifdef LV_USING_EXT_RESOURCE_MANAGER

#include "lv_ext_resource_manager_builtin.h"


#ifdef LV_EXT_RES_STANDALONE
    #include <dfs_posix.h>
    #include "mod_installer.h"
#endif /* LV_EXT_RES_STANDALONE */


#if 0
static lv_ext_res_mng_t resource_manager;



lv_img_dsc_t *lv_ext_img_get(lv_img_dsc_t **img, const char *key)
{
    if (NULL == *img)
    {
        if (resource_manager && resource_manager->get_img)
        {
            *img = resource_manager->get_img(resource_manager, key);
        }
    }

    return *img;
}


lv_font_t *lv_ext_font_get(uint8_t font_size)
{
    const lv_ex_font_config_t *config;
    lv_font_t *font = NULL;

    if (resource_manager && resource_manager->get_font_config)
    {
        config = resource_manager->get_font_config(resource_manager);
        if (config)
        {
            while (config->size >= 0)
            {
                if (font_size == config->size)
                {
                    font = config->font;
                    break;
                }
                config++;
            }
        }
    }

    if (NULL == font)
    {
        font = LV_FONT_THEME_DEFAULT;
    }

    return font;
}


lv_res_t lv_ext_register_res_manager(lv_ext_res_mng_t res_mng)
{
    lv_res_t result = LV_RES_INV;

    resource_manager = res_mng;

    if (resource_manager && resource_manager->load)
    {
        result = resource_manager->load(resource_manager);
        if (LV_RES_OK == result)
        {
            if (resource_manager->get_lang_pack)
            {
                lang_pack = resource_manager->get_lang_pack(resource_manager);
            }
            if (lang_pack)
            {
                curr_lang = lang_pack[0];
            }
        }
    }

    return result;
}

/* get resource manager */
lv_ext_res_mng_t lv_ext_get_res_manager(void)
{
    return resource_manager;
}

lv_res_t lv_ext_rm_add_resource(lv_ext_resource_type type, void *resource)
{
    return LV_RES_OK;

}
#endif /* 0 */



extern const lv_i18n_lang_t *const lv_i18n_lang_pack[];

static struct lv_ext_res_mng_tag kernel_res_mng;


lv_res_t resource_init(void)
{
#if 0
    lv_res_t lv_res;

#ifdef LV_EXT_RES_NON_STANDALONE
    lv_res = lv_ext_res_mng_builtin_init(&kernel_res_mng, lv_i18n_lang_pack);

#else
    lv_ext_res_mng_dl_impl_data_t user_data;
    int fd;
    rt_err_t r;

    fd = open(LANG_INSTALL_PATH "/" LANG_PACK_MOD_NAME, O_RDONLY);
    if (fd >= 0)
    {
        close(fd);
        r = RT_EOK;
    }
    else
    {
        /* install language pack first */
        r = mod_install(LANG_PACK_MOD_NAME, LANG_DEFAULT_PACKAGE_PATH "/" LANG_PACK_MOD_NAME ".so",
                        LANG_INSTALL_PATH);
    }
    if (RT_EOK == r)
    {
        user_data.str_module_name = LANG_PACK_MOD_NAME;
        lv_res = lv_ext_res_mng_dl_impl_init(&kernel_res_mng, &user_data);
    }
    else
    {
        lv_res = LV_RES_INV;
    }

#endif
#else

    lv_ext_res_mng_builtin_init(&kernel_res_mng, lv_i18n_lang_pack);

#endif
    return LV_RES_OK;
}


lv_res_t resource_deinit(void)
{
    lv_res_t r;
#if 0
#ifdef LV_EXT_RES_NON_STANDALONE
    r = LV_RES_OK;

#else
    struct lv_ext_res_mng_tag old_res_mng;

    rt_enter_critical();
    memcpy(&old_res_mng, &kernel_res_mng, sizeof(old_res_mng));
    memset(&kernel_res_mng, 0, sizeof(kernel_res_mng));
    rt_exit_critical();

    r = lv_ext_res_mng_dl_impl_destroy(&old_res_mng);

#endif
#else

    r = lv_ext_res_mng_builtin_destroy(&kernel_res_mng);

#endif
    return r;
}



const char *lv_ext_str_get(lv_ext_res_mng_t res_mng, uint32_t offset, const char *key)
{
    const char *s = NULL;
    lv_i18n_phrase_t *phrase;


    if (!res_mng)
    {
        res_mng = &kernel_res_mng;
    }

    if ((res_mng->curr_lang) && res_mng->curr_lang->translation)
    {
        phrase = (lv_i18n_phrase_t *)((uint32_t)res_mng->curr_lang->translation + offset);

        if (phrase->singular)
        {
            s = phrase->singular;
        }
        else
        {
            /* not found */
            s = key;
        }
    }
    else
    {
        s = key;
    }

    return s;
}


lv_res_t lv_ext_set_locale(lv_ext_res_mng_t res_mng, const char *locale)
{
    const lv_i18n_lang_pack_t *new_lang;
    lv_res_t res = LV_RES_INV;
    lv_lang_pack_node_t *node;

    if (!res_mng)
    {
        res_mng = &kernel_res_mng;
    }

#if 0
    if (res_mng->lang_pack)
    {
        new_lang = &res_mng->lang_pack[0];
        while (*new_lang)
        {
            if (0 == strncmp((*new_lang)->locale, locale, strlen((*new_lang)->locale)))
            {
                rt_kprintf("match lang\n");
                res = LV_RES_OK;
                res_mng->curr_lang = *new_lang;
                break;
            }
            new_lang++;
        }
    }
#else
    _LV_LL_READ(&(res_mng->lang_pack_list), node)
    {
        if (!node->disabled)
        {
            new_lang = &node->lang_pack[0];
            while (*new_lang)
            {
                if (0 == strncmp((*new_lang)->locale, locale, strlen((*new_lang)->locale)))
                {
                    rt_kprintf("match lang\n");
                    res = LV_RES_OK;
                    res_mng->curr_lang = *new_lang;
                    res_mng->curr_lang_node = node;
                    break;
                }
                new_lang++;
            }
        }
    }

#endif


    return res;
}

const char *lv_ext_get_locale(void)
{
    const char *locale = NULL;

    if (kernel_res_mng.curr_lang)
    {
        locale = kernel_res_mng.curr_lang->locale;
    }

    return locale;
}

const lv_ll_t *lv_ext_get_lang_pack_list(lv_ext_res_mng_t res_mng)
{
    if (!res_mng)
    {
        res_mng = &kernel_res_mng;
    }

    return &res_mng->lang_pack_list;
}

const lv_lang_pack_node_t *lv_ext_add_lang_pack(lv_ext_res_mng_t res_mng,
        const lv_i18n_lang_pack_t *lang_pack,
        const char *lang_pack_name)
{
    lv_lang_pack_node_t *node;
    lv_lang_pack_node_t *iter;
    const lv_i18n_lang_pack_t *new_lang_pack;
    bool duplicate_lang;

    if (!res_mng)
    {
        res_mng = &kernel_res_mng;
    }
    if (!lang_pack || (NULL == *lang_pack) || !lang_pack_name)
    {
        return NULL;
    }

    duplicate_lang = false;
    LV_EXT_LANG_PACK_LIST_ITER(res_mng, iter)
    {
        if ((iter->lang_pack == lang_pack)
                || (0 == strcmp(iter->lang_pack_name, lang_pack_name)))
        {
            duplicate_lang = true;
            break;
        }
        if (iter->disabled)
        {
            continue;
        }
        LV_EXT_LANG_PACK_ITER(iter, old_lang_pack)
        {
            new_lang_pack = lang_pack;
            for (; (NULL != new_lang_pack) && (*new_lang_pack); new_lang_pack++)
            {
                if (0 == strcmp((*new_lang_pack)->locale, (*old_lang_pack)->locale))
                {
                    duplicate_lang = true;
                    break;
                }
            }
            if (duplicate_lang)
            {
                break;
            }
        }
        if (duplicate_lang)
        {
            break;
        }
    }

    if (duplicate_lang)
    {
        return NULL;
    }

    node = _lv_ll_ins_tail(&res_mng->lang_pack_list);
    node->disabled = false;
    node->lang_pack = lang_pack;
    node->lang_pack_name = lang_pack_name;

    return node;
}

lv_res_t lv_ext_del_lang_pack(lv_ext_res_mng_t res_mng, const char *lang_pack_name)
{
    lv_lang_pack_node_t *iter;
    bool found;

    if (!res_mng)
    {
        res_mng = &kernel_res_mng;
    }

    found = false;
    LV_EXT_LANG_PACK_LIST_ITER(res_mng, iter)
    {
        if ((iter->lang_pack_name == lang_pack_name)
                || (0 == strcmp(iter->lang_pack_name, lang_pack_name)))
        {
            found = true;
            break;
        }
    }

    if (!found)
    {
        return LV_RES_INV;
    }

    if (iter == _lv_ll_get_head(&res_mng->lang_pack_list))
    {
        /* cannot delete embedded lang_pack */
        return LV_RES_INV;
    }

    if (res_mng->curr_lang_node == iter)
    {
        res_mng->curr_lang_node = _lv_ll_get_head(&res_mng->lang_pack_list);
        res_mng->curr_lang = res_mng->curr_lang_node->lang_pack[0];
    }
    _lv_ll_remove(&res_mng->lang_pack_list, iter);
    lv_mem_free(iter);

    return LV_RES_OK;
}

lv_res_t lv_ext_disable_lang_pack(lv_ext_res_mng_t res_mng,
                                  lv_lang_pack_node_t *node)
{
    if (!res_mng)
    {
        res_mng = &kernel_res_mng;
    }

    if (!node)
    {
        return LV_RES_INV;
    }

    node->disabled = true;

    return LV_RES_OK;
}

lv_res_t lv_ext_enable_lang_pack(lv_ext_res_mng_t res_mng,
                                 lv_lang_pack_node_t *node)
{
    if (!res_mng)
    {
        res_mng = &kernel_res_mng;
    }

    if (!node)
    {
        return LV_RES_INV;
    }

    node->disabled = false;

    return LV_RES_OK;
}


lv_res_t lv_ext_clear_lang_pack(lv_ext_res_mng_t res_mng)
{
    if (!res_mng)
    {
        res_mng = &kernel_res_mng;
    }

    _lv_ll_clear(&res_mng->lang_pack_list);

    return LV_RES_OK;
}


#endif /*LV_USING_EXT_RESOURCE_MANAGER*/




/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
