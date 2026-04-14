/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "nand_cfg.h"

SECTION_DEF(NAND_CFG_SECTION_NAME, nand_cfg_t);

const nand_ext_cfg_t *spi_nand_get_ext_cfg_by_id(uint8_t fid, uint8_t did, uint8_t mtype)
{
    const nand_cfg_t *nand_cfg;
    uint32_t *end;
    uint32_t *temp;

    end = (uint32_t *)SECTION_END_ADDR(NAND_CFG_SECTION_NAME);
    temp = (uint32_t *)SECTION_START_ADDR(NAND_CFG_SECTION_NAME);
    while (temp < end)
    {
        nand_cfg = (const nand_cfg_t *)temp;
        if (nand_cfg->id_type.mem_size)
        {
            if (fid == nand_cfg->id_type.manufacture_id
                    && mtype == nand_cfg->id_type.memory_type
                    && did == nand_cfg->id_type.memory_density)
            {
                return &nand_cfg->ext_cfg;
            }
            temp += (sizeof(nand_cfg_t) / sizeof(uint32_t));
        }
        else
        {
            temp++;
        }
    }

    return NULL;
}

const FLASH_RDID_TYPE_T *spi_nand_get_user_flash_cfg(uint8_t fid, uint8_t did, uint8_t mtype, NAND_CMD_TABLE_ID_T *cmd_tbl_type)
{
    const nand_cfg_t *nand_cfg;
    uint32_t *end;
    uint32_t *temp;

    end = (uint32_t *)SECTION_END_ADDR(NAND_CFG_SECTION_NAME);
    temp = (uint32_t *)SECTION_START_ADDR(NAND_CFG_SECTION_NAME);
    while (temp < end)
    {
        nand_cfg = (const nand_cfg_t *)temp;
        if (nand_cfg->id_type.mem_size)
        {
            if (fid == nand_cfg->id_type.manufacture_id
                    && mtype == nand_cfg->id_type.memory_type
                    && did == nand_cfg->id_type.memory_density)
            {
                if (cmd_tbl_type)
                {
                    *cmd_tbl_type = nand_cfg->cmd_tbl_type;
                }
                return &nand_cfg->id_type;
            }
            temp += (sizeof(nand_cfg_t) / sizeof(uint32_t));
        }
        else
        {
            temp++;
        }
    }

    return NULL;

}