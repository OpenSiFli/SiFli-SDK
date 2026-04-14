/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "nor_cfg.h"


NOR_CFG_REGISTER(en25qx128a) =
{
    .id_type = {
        .manufacture_id = 0x1C,
        .memory_type = 0x71,
        .memory_density = 0x18,
        .ext_flags = 0x0,
        .mem_size = 0x1000000
    },
    .cmd_tbl_type = NOR_TYPE0,
    .ext_cfg = {
        .otp_base = 0xFFC000
    }
};
