/**
  ******************************************************************************
  * @file   lv_ext_resource_manager.h
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

#ifndef LV_EXT_RESOURCE_MANAGER_H_
#define LV_EXT_RESOURCE_MANAGER_H_

#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"

#ifdef LV_USING_EXT_RESOURCE_MANAGER
//#include "app_mem.h"

#define LV_RES_MAKE_VAR(type,key) _v_##type##_##key
#define LV_RES_MAKE_MACRO(type,key) m_res_##type##_##key
#define LV_OFFSET_IN_TYPE(type,key) ((uint32_t)(&(((type *)0)->key)))

typedef enum
{
    LV_EXT_RES_IMG,
    LV_EXT_RES_STRING,
} lv_ext_resource_type;



typedef enum
{
    LV_I18N_PLURAL_TYPE_ZERO,
    LV_I18N_PLURAL_TYPE_ONE,
    LV_I18N_PLURAL_TYPE_TWO,
    LV_I18N_PLURAL_TYPE_FEW,
    LV_I18N_PLURAL_TYPE_MANY,
    LV_I18N_PLURAL_TYPE_OTHER,
    _LV_I18N_PLURAL_TYPE_NUM,
} lv_i18n_plural_type_t;

typedef struct
{
    const char *key;
    const char *singular;
    const char *plurals[_LV_I18N_PLURAL_TYPE_NUM];
} lv_i18n_phrase_t;

/*  generated translation table

typedef struct
{
    lv_i18n_phrase_t key1;
    lv_i18n_phrase_t key2;
    lv_i18n_phrase_t key3;
    ...
} lang_translation_t

*/



typedef struct
{
    const char *locale;
    /* placeholder of lang_translation_t field*/
    const void *translation;
} lv_i18n_lang_t;


typedef const lv_i18n_lang_t *lv_i18n_lang_pack_t;

typedef struct lv_ext_res_mng_tag *lv_ext_res_mng_t;


typedef struct
{
    /** font size, 0: default, 1: size 1, 2: size 2, etc. < 0: invalid*/
    const int8_t size;
    lv_font_t *font;
} lv_ex_font_config_t;



typedef struct
{
    const char *lang_pack_name;
    const lv_i18n_lang_pack_t *lang_pack;
    bool disabled;
} lv_lang_pack_node_t;

struct lv_ext_res_mng_tag
{
    const lv_i18n_lang_t *curr_lang;
    const lv_lang_pack_node_t *curr_lang_node;
    lv_ll_t lang_pack_list;

    lv_res_t (*load)(lv_ext_res_mng_t res_mng);
    lv_img_dsc_t *(*get_img)(lv_ext_res_mng_t res_mng, const char *key);
    const char *(*get_string)(lv_ext_res_mng_t res_mng, uint32_t offset, const char *key);
    void *user_data;
    const lv_i18n_lang_pack_t *(*get_lang_pack)(lv_ext_res_mng_t res_mng);
    const lv_ex_font_config_t *(*get_font_config)(lv_ext_res_mng_t res_mng);
};


/* register resource manager */
lv_res_t lv_ext_register_res_manager(lv_ext_res_mng_t res_manager);
/* get resource manager */
lv_ext_res_mng_t lv_ext_get_res_manager(void);

lv_res_t lv_ext_res_mng_init(void *user_data);

lv_res_t lv_ext_rm_add_resource(lv_ext_resource_type type, void *resource);


/***********************************************************
 * string
 **********************************************************/
/*
  in resource file, values of foo_string for different languages
  are defined as below

char *foo_string_zh_cn = "测试"
char *foo_string_en_us = "test"

user can get foo_string in terms of current locale setting,
for chinese string "测试" would be returned, while english environment would get string "test"

e.g.
lv_label_set_text(label, LV_EXT_STR_GET(foo_string))
 */

#define LV_EXT_STR_ID(key) LV_OFFSET_IN_TYPE(lang_translation_t, key)
#define LV_EXT_STR_ID_2(key, desc) LV_OFFSET_IN_TYPE(lang_translation_t, key)

const char *lv_ext_str_get(lv_ext_res_mng_t res_mng, uint32_t offset, const char *key);

/* get string according to locale setting*/
#ifdef BUILD_DLMODULE

    #define LV_EXT_STR_GET(id) lv_ext_str_get((lv_ext_res_mng_t)dlmodule_get_res_mng(MODULE_GET()), id, "Unknown string id")
#else
    #define LV_EXT_STR_GET(id) lv_ext_str_get(NULL, id, "Unknown string id")
#endif

#ifdef BUILD_DLMODULE
    #define LV_EXT_STR_GET_BY_KEY(key, desc) lv_ext_str_get((lv_ext_res_mng_t)dlmodule_get_res_mng(MODULE_GET()), LV_EXT_STR_ID_2(key, desc), "Unknown string id")
#else
    #define LV_EXT_STR_GET_BY_KEY(key, desc) lv_ext_str_get(NULL, LV_EXT_STR_ID_2(key, desc), "Unknown string id")
#endif

#define LV_EXT_STR_GET_BY_ID(id)     LV_EXT_STR_GET(id)

typedef uint32_t lv_ext_str_id_t;

typedef enum
{
    LV_LOCALE_INVALID,
    LV_LOCALE_EN_US,
    LV_LOCALE_ZH_CN,
} lv_locale_t;

/* set locale */
lv_res_t lv_ext_set_locale(lv_ext_res_mng_t res_mng, const char *locale);
/* get locale */
const char *lv_ext_get_locale(void);

const lv_i18n_lang_pack_t *lv_ext_get_lang_pack(lv_ext_res_mng_t res_mng);

const lv_ll_t *lv_ext_get_lang_pack_list(lv_ext_res_mng_t res_mng);

const lv_lang_pack_node_t *lv_ext_add_lang_pack(lv_ext_res_mng_t res_mng,
        const lv_i18n_lang_pack_t *lang_pack, const char *lang_pack_name);
lv_res_t lv_ext_del_lang_pack(lv_ext_res_mng_t res_mng, const char *lang_pack_name);

lv_res_t lv_ext_disable_lang_pack(lv_ext_res_mng_t res_mng,
                                  lv_lang_pack_node_t *node);
lv_res_t lv_ext_enable_lang_pack(lv_ext_res_mng_t res_mng,
                                 lv_lang_pack_node_t *node);
lv_res_t lv_ext_clear_lang_pack(lv_ext_res_mng_t res_mng);


#define LV_EXT_LANG_PACK_LIST_ITER_DEF(iter)    lv_lang_pack_node_t *iter

#define LV_EXT_LANG_PACK_LIST_ITER(res_mng,iter)  \
        _LV_LL_READ((lv_ext_get_lang_pack_list(res_mng)), iter)

#define LV_EXT_LANG_PACK_ITER(iter, lang_pack_iter)  \
        for (const lv_i18n_lang_pack_t *lang_pack_iter = iter->lang_pack; (NULL != lang_pack_iter) && (NULL != *lang_pack_iter); lang_pack_iter++)

#define LV_EXT_LANG_PACK_ITER_GET_NAME(lang_pack_iter)  (*lang_pack_iter)->locale



lv_res_t resource_init(void);
lv_res_t resource_deinit(void);

#include "lv_ext_resource_manager_builtin.h"


#include "lang_pack.h"

/***********************************************************
 * Style
 **********************************************************/
//#define LV_EXT_STYLE_GET(style) lv_ext_style_get(#style)

#else
#define lv_ext_set_locale(res_mng, local) LV_RES_INV

#define lv_ext_get_locale()  ""

#define LV_EXT_STR_ID(key)  #key
#define LV_EXT_STR_ID_2(key, desc)  desc
#define LV_EXT_STR_GET(id) id
#define LV_EXT_STR_GET_BY_KEY(key, desc) desc
#define LV_EXT_STR_GET_BY_ID(id)  LV_EXT_STR_GET(id)


#define LV_EXT_LANG_PACK_LIST_ITER_DEF(iter)

#define LV_EXT_LANG_PACK_LIST_ITER(res_mng,iter)  for (; false; )

#define LV_EXT_LANG_PACK_ITER(iter, lang_pack_iter)  for (; false; )

#define LV_EXT_LANG_PACK_ITER_GET_NAME(iter)  NULL


typedef const char *lv_ext_str_id_t;

#define resource_init()

#endif  /* LV_USING_EXT_RESOURCE_MANAGER */


#include "sf_type.h"

#ifndef SOLUTION_WATCH
    #define RES_PATH    "/ex/resource/"
    #define RES_SUFFIX  ".bin"

    #ifdef LV_USING_FILE_RESOURCE
        #define LV_EXT_IMG_GET(key) (RES_PATH STRINGIFY(key) RES_SUFFIX)
    #else
        #define LV_EXT_IMG_GET(key) (const void *)&key
    #endif

    #define LV_EXT_FILE_IMG_GET(key) (RES_PATH STRINGIFY(key) RES_SUFFIX)
#endif

#define LINE_HEIGHT_DEFAULT 0XFFFF



//lv_ext_set_local_font(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_color_t color)
#define lv_ext_set_local_text_color lv_obj_set_style_text_color

//lv_obj_set_style_local_bg_opa(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_opa_t opa)
#define lv_ext_set_local_bg_opa  lv_obj_set_style_bg_opa

//lv_ext_set_local_bg_color(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_color_t color)
#define lv_ext_set_local_bg_color  lv_obj_set_style_bg_color

//lv_ext_set_local_bg_grad_color(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_color_t color)
#define lv_ext_set_local_bg_grad_color  lv_obj_set_style_bg_grad_color

//lv_ext_set_local_font(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_font_t * font)
#define lv_ext_set_local_text_font  lv_obj_set_style_text_font

//lv_ext_set_local_img_opa(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_opa_t opa)
#define lv_ext_set_local_img_opa  lv_obj_set_style_image_opa

//lv_ext_set_local_text_sel_color(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_color_t color)
#define lv_ext_set_local_text_sel_color  lv_obj_set_style_text_color

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_text_letter_space  lv_obj_set_style_text_letter_space

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_transition_prop_6  lv_obj_set_style_transitio

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_border_color  lv_obj_set_style_border_color

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_border_opa  lv_obj_set_style_border_opa

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_border_width  lv_obj_set_style_border_width

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_radius(obj, part, prop, value) \
        lv_obj_set_style_radius(obj, (lv_style_int_t) (value),part)

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_outline_color  lv_style_set_outline_color

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_outline_opa  lv_style_set_outline_opa

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_outline_width  lv_style_set_outline_width

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_transition_time  lv_style_set_transition

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_transform_height  lv_ext_set_local_transform_height

//lv_ext_set_local_text_letter_space(lv_obj_t * obj, uint8_t type, lv_style_property_t prop, lv_style_int_t int)
#define lv_ext_set_local_transform_width  lv_ext_set_local_transform_width

static inline  void lv_ext_set_local_bg(lv_obj_t *obj, lv_color_t bgcolor, lv_opa_t opa)
{
    lv_ext_set_local_bg_opa(obj, opa, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_ext_set_local_bg_color(obj, bgcolor, LV_PART_MAIN | LV_STATE_DEFAULT);
}



#endif /* LV_EXT_RESOURCE_MANAGER_H_ */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
