/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "osal_debug.h"
#include "osal_porting.h"
#include <string.h>


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
/// Debug module environment definition. (moved here for host)
struct debug_env_tag dbg_env =
{
    .filter_module = DBG_MOD_ALL,
    .filter_severity = DBG_SEV_IDX_INF,//DBG_SEV_IDX_VRB,//
};

int dbg_test_module_severity(unsigned int mod_idx, unsigned int sev_idx)
{
    int ret = 0;
    if ((mod_idx < DBG_MOD_IDX_MAX) && (dbg_env.filter_module & CO_BIT(mod_idx)) &&
            (dbg_env.filter_severity >= sev_idx))
    {
        ret = 1;
    }
    return ret;
}

