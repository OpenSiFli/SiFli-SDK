/**
  ******************************************************************************
  * @file   lv_ext_resource_manager_builtin.c
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



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
