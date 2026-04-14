/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LV_EXT_RESOURCE_MANAGER_DL_IMPL_H_
#define LV_EXT_RESOURCE_MANAGER_DL_IMPL_H_

#include "lv_ext_resource_manager.h"

#ifdef LV_USING_EXT_RESOURCE_MANAGER

typedef struct
{
    const char *img_module_name;
    const char *str_module_name;
    void *img_module;
    void *str_module;
    void *font_module;
} lv_ext_res_mng_dl_impl_data_t;

#define LANG_PACK_MOD_NAME  "lang_pack"
#define LANG_INSTALL_PATH   "watchface"
#define LANG_DEFAULT_PACKAGE_PATH   "ex/installer"

//lv_res_t lv_ext_res_mng_dl_impl_init(lv_ext_res_mng_t res_mng, lv_ext_res_mng_dl_impl_data_t *user_data);

lv_res_t lv_ext_res_mng_dl_impl_destroy(lv_ext_res_mng_t res_mng);

#endif /* LV_USING_EXT_RESOURCE_MANAGER */

#endif /* LV_EXT_RESOURCE_MANAGER_DL_IMPL_H_ */
