/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _NAND_CFG_H
#define _NAND_CFG_H
#include <stdint.h>
#include "rtconfig.h"
#include "register.h"
#include "flash_table.h"
#include "section.h"

#define NOR_CFG_SECTION_NAME   nor_cfg_db

#define NOR_CFG_REGISTER(name)      \
    SECTION_ITEM_REGISTER(NOR_CFG_SECTION_NAME, static const nor_cfg_t CONCAT_2(nor_, name))


typedef struct nor_cfg_tag
{
    FLASH_RDID_TYPE_T id_type;
    FLASH_CMD_TABLE_ID_T cmd_tbl_type;
    nor_ext_cfg_t ext_cfg;
} nor_cfg_t;


#endif /* _NOR_CFG_H */
