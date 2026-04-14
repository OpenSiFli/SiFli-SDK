/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lv_ext_resource_manager.h"

#if 0//defined(LV_EXT_RES_NON_STANDALONE)

static struct lv_ext_res_mng_tag res_mng_builtin_impl;

static lv_res_t load(lv_ext_res_mng_t res_mng)
{

    return LV_RES_OK;
}

static lv_img_dsc_t *get_img(lv_ext_res_mng_t res_mng, const char *key)
{
    return NULL;
}

static const char *get_string(lv_ext_res_mng_t res_mng, uint32_t offset, const char *key)
{

    return NULL;
}

static const lv_i18n_lang_pack_t *get_lang_pack(lv_ext_res_mng_t res_mng)
{
    return lv_i18n_lang_pack;
}

static const lv_ex_font_config_t *get_font_config(lv_ext_res_mng_t res_mng)
{
    return lv_ex_font_config;
}

static lv_res_t set_locale(const char *locale)
{
    return LV_RES_OK;

}

lv_res_t lv_ext_res_mng_init(void *user_data)
{
    res_mng_builtin_impl.get_img = get_img;
    res_mng_builtin_impl.get_string = get_string;
    res_mng_builtin_impl.load = load;
    res_mng_builtin_impl.get_lang_pack = get_lang_pack;
    res_mng_builtin_impl.get_font_config = get_font_config;
    res_mng_builtin_impl.user_data = user_data;

    return lv_ext_register_res_manager(&res_mng_builtin_impl);
}

#endif /*LV_EXT_RESOURCE_MANAGER_BUILTIN*/

#ifdef LV_USING_EXT_RESOURCE_MANAGER

lv_res_t lv_ext_res_mng_builtin_init(lv_ext_res_mng_t res_mng, const lv_i18n_lang_pack_t *lang_pack)
{
    const lv_lang_pack_node_t *node;

    RT_ASSERT(res_mng);

    _lv_ll_init(&res_mng->lang_pack_list, sizeof(lv_lang_pack_node_t));

    node = lv_ext_add_lang_pack(res_mng, lang_pack, "default");
    RT_ASSERT(node);
    res_mng->curr_lang = lang_pack[0];
    res_mng->curr_lang_node = node;

    return LV_RES_OK;
}

lv_res_t lv_ext_res_mng_builtin_destroy(lv_ext_res_mng_t res_mng)
{
    lv_ext_clear_lang_pack(res_mng);
    res_mng->curr_lang = NULL;
    res_mng->curr_lang_node = NULL;

    return LV_RES_OK;
}
#endif /* LV_USING_EXT_RESOURCE_MANAGER */

