/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "nand_cfg.h"


NAND_CFG_REGISTER(hyf4gq4utacae) =
{
    .id_type = {
        .manufacture_id = 0x01,
        .memory_type = 0x35,
        .memory_density = 0x01,
        .ext_flags = 0x70,
        .mem_size = 0x20000000
    },
    .cmd_tbl_type = NAND_TYPE3,
    .ext_cfg = {
        .ecc_err_mask = 0x8   // ecc status value 3 means uncorrectable error
    }
};
