/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "rtthread.h"
#include <stdint.h>

#include "ll_crc.h"

#ifndef SF32LB52X
    #error "This example currently supports SF32LB52x only."
#endif

#define LL_CRC_EXPECTED_CRC32_MPEG2_123456789 0x0376E6E7U

static inline void ll_crc_wait_done(CRC_TypeDef *CRCx)
{
    while (ll_crc_is_active_flag_done(CRCx) == 0U)
    {
    }
}

/**
 * @brief Application entry for LL CRC example.
 * @return This function does not return.
 */
int main(void)
{
    ll_crc_ctrl_config_t ctrl_cfg = {
        .data_size = LL_CRC_DATASIZE_32B,
        .poly_size = LL_CRC_POLYSIZE_32B,
        .rev_in = LL_CRC_REV_IN_NONE,
        .rev_out = LL_CRC_REV_OUT_DISABLE,
    };
    uint32_t result;

    HAL_RCC_EnableModule(RCC_MOD_CRC1);

    ll_crc_set_init(hwp_crc, 0xFFFFFFFFU);
    ll_crc_set_poly(hwp_crc, 0x04C11DB7U);
    ll_crc_config_ctrl(hwp_crc, &ctrl_cfg);
    ll_crc_reset(hwp_crc);

    /* Test vector "123456789" with 32-bit main path + 8-bit tail. */
    ll_crc_push_data32(hwp_crc, 0x34333231U);
    ll_crc_wait_done(hwp_crc);

    ll_crc_push_data32(hwp_crc, 0x38373635U);
    ll_crc_wait_done(hwp_crc);

    ll_crc_push_data8(hwp_crc, 0x39U);
    ll_crc_wait_done(hwp_crc);

    result = ll_crc_read_result(hwp_crc);

    rt_kprintf("LL CRC example started.\n");
    rt_kprintf("CRC-32/MPEG-2('123456789') result=0x%08lX, expect=0x%08lX\n",
               (unsigned long)result,
               (unsigned long)LL_CRC_EXPECTED_CRC32_MPEG2_123456789);

    if (ll_crc_is_active_flag_overflow(hwp_crc) != 0U)
    {
        rt_kprintf("LL CRC overflow flag is set.\n");
    }

    if (result == LL_CRC_EXPECTED_CRC32_MPEG2_123456789)
    {
        rt_kprintf("LL CRC check PASS.\n");
    }
    else
    {
        rt_kprintf("LL CRC check FAIL.\n");
    }

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
