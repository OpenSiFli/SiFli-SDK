/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "nor_cfg.h"

SECTION_DEF(NOR_CFG_SECTION_NAME, nor_cfg_t);

HAL_RAM_RET_CODE_SECT(spi_nor_get_ext_cfg_by_id, const nor_ext_cfg_t *spi_nor_get_ext_cfg_by_id(uint8_t fid, uint8_t did, uint8_t mtype))
{
    const nor_cfg_t *nor_cfg;
    uint32_t *end;
    uint32_t *temp;

    end = (uint32_t *)SECTION_END_ADDR(NOR_CFG_SECTION_NAME);
    temp = (uint32_t *)SECTION_START_ADDR(NOR_CFG_SECTION_NAME);
    while (temp < end)
    {
        nor_cfg = (const nor_cfg_t *)temp;
        if (nor_cfg->id_type.mem_size)
        {
            if (fid == nor_cfg->id_type.manufacture_id
                    && mtype == nor_cfg->id_type.memory_type
                    && did == nor_cfg->id_type.memory_density)
            {
                return &nor_cfg->ext_cfg;
            }
            temp += (sizeof(nor_cfg_t) / sizeof(uint32_t));
        }
        else
        {
            temp++;
        }
    }

    return NULL;
}

HAL_RAM_RET_CODE_SECT(spi_nor_get_user_flash_cfg, const FLASH_RDID_TYPE_T *spi_nor_get_user_flash_cfg(uint8_t fid, uint8_t did, uint8_t mtype, FLASH_CMD_TABLE_ID_T *cmd_tbl_type))
{
    const nor_cfg_t *nor_cfg;
    uint32_t *end;
    uint32_t *temp;

    end = (uint32_t *)SECTION_END_ADDR(NOR_CFG_SECTION_NAME);
    temp = (uint32_t *)SECTION_START_ADDR(NOR_CFG_SECTION_NAME);
    while (temp < end)
    {
        nor_cfg = (const nor_cfg_t *)temp;
        if (nor_cfg->id_type.mem_size)
        {
            if (fid == nor_cfg->id_type.manufacture_id
                    && mtype == nor_cfg->id_type.memory_type
                    && did == nor_cfg->id_type.memory_density)
            {
                if (cmd_tbl_type)
                {
                    *cmd_tbl_type = nor_cfg->cmd_tbl_type;
                }
                return &nor_cfg->id_type;
            }
            temp += (sizeof(nor_cfg_t) / sizeof(uint32_t));
        }
        else
        {
            temp++;
        }
    }

    return NULL;

}