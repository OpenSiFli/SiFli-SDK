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

#include "rwnx_main.h"
#include "rwnx_platform.h"
//#include "reg_access.h"
#include "rwnx_defs.h"
#include "rwnx_msg_tx.h"
#include "aicwf_sdio.h"
#include "osal_debug.h"


int rwnx_rdata_init(struct rwnx_plat *rwnx_plat, void **platform_data)
{
    int ret = 0;
    struct rwnx_hw *rwnx_hw;

    // alloc structs
    rwnx_hw = rtos_malloc(sizeof(struct rwnx_hw));
    if (rwnx_hw == NULL)
    {
        DBG_RDATA_ERR("rwnx_hw alloc failed\r\n");
        return -1;
    }
    rwnx_hw->plat = rwnx_plat;
    rwnx_hw->bus_if = rwnx_plat->sdiodev->bus_if;
    rwnx_hw->sdiodev = rwnx_plat->sdiodev;
    rwnx_plat->sdiodev->rwnx_hw = rwnx_hw;

    ret = rwnx_platform_on(rwnx_hw);
    if (ret)
    {
        goto err_platon;
    }

    *platform_data = rwnx_hw;

#if 0
    // Test dbg msg
    const uint32_t mem_addr = 0x40500000;
    struct dbg_mem_read_cfm rd_mem_addr_cfm;
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, mem_addr, &rd_mem_addr_cfm);
    if (ret)
    {
        DBG_RDATA_ERR("%x rd fail: %d\n", mem_addr, ret);
        goto err_test;
    }
#endif

    return 0;

err_test:
    rwnx_platform_off(rwnx_hw);

err_platon:
    rtos_free(rwnx_hw);
    return ret;
}

void rwnx_rdata_deinit(struct rwnx_hw *rwnx_hw)
{
    RWNX_DBG(RWNX_FN_ENTRY_STR);
    rwnx_platform_off(rwnx_hw);
}
