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

#define NAND_CFG_SECTION_NAME   nand_cfg_db

#define NAND_CFG_REGISTER(name)      \
    SECTION_ITEM_REGISTER(NAND_CFG_SECTION_NAME, static const nand_cfg_t CONCAT_2(nand_, name))


typedef struct nand_cfg_tag
{
    FLASH_RDID_TYPE_T id_type;
    NAND_CMD_TABLE_ID_T cmd_tbl_type;
    nand_ext_cfg_t ext_cfg;
} nand_cfg_t;


#endif /* _NAND_CFG_H */
