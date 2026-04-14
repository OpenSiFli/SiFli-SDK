/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LV_EXT_RESOURCE_MANAGER_BUILTIN_H_
#define LV_EXT_RESOURCE_MANAGER_BUILTIN_H_

#include "lv_ext_resource_manager.h"

#ifdef LV_USING_EXT_RESOURCE_MANAGER
    lv_res_t lv_ext_res_mng_builtin_init(lv_ext_res_mng_t res_mng, const lv_i18n_lang_pack_t *lang_pack);
    lv_res_t lv_ext_res_mng_builtin_destroy(lv_ext_res_mng_t res_mng);
#endif /* LV_USING_EXT_RESOURCE_MANAGER */

#endif /* LV_EXT_RESOURCE_MANAGER_DL_IMPL_H_ */

