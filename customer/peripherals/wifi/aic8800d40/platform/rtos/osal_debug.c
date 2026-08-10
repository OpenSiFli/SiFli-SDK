/**
  ******************************************************************************
  * @file   osal_debug.c
  * @author AIC software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
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
#include "co_math.h"

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
/// Debug module environment definition.
struct debug_env_tag dbg_env = {
    .filter_module = DBG_MOD_ALL,// & ~(CO_BIT(DBG_MOD_IDX_MACIF) | CO_BIT(DBG_MOD_IDX_SDIO)),
#ifdef AIC_DEV_AIC8800_DEBUG_LOG
    .filter_severity = DBG_SEV_IDX_INF,
#else
    .filter_severity = DBG_SEV_IDX_WRN,
#endif
};

void dbg_set_module(unsigned int module)
{
    dbg_env.filter_module = module;
}

void dbg_set_severity(unsigned int severity)
{
    dbg_env.filter_severity = severity;
}

unsigned int dbg_get_module(void)
{
    return dbg_env.filter_module;
}

unsigned int dbg_get_severity(void)
{
    return dbg_env.filter_severity;
}

int dbg_test_severity(unsigned int sev_idx)
{
    int ret = 0;
    if (dbg_env.filter_severity >= sev_idx) {
        ret = 1;
    }
    return ret;
}

int dbg_test_module_severity(unsigned int mod_idx, unsigned int sev_idx)
{
    int ret = 0;
    if ((mod_idx < DBG_MOD_IDX_MAX) && (dbg_env.filter_module & CO_BIT(mod_idx)) &&
        (dbg_env.filter_severity >= sev_idx)) {
        ret = 1;
    }
    return ret;
}

