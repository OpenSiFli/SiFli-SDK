/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CUSTOM_MEM_MAP__
#define __CUSTOM_MEM_MAP__

#ifdef USING_PARTITION_TABLE
    #include "ptab.h"
#endif /* USING_PARTITION_TABLE */

#define FAL_PART_TABLE \
{ \
    {FAL_PART_MAGIC_WORD,       "dfu",      NOR_FLASH4_DEV_NAME,    KVDB_DFU_REGION_OFFSET,   KVDB_DFU_REGION_SIZE, 0}, \
    {FAL_PART_MAGIC_WORD,       "ble",      NOR_FLASH4_DEV_NAME,    KVDB_BLE_REGION_OFFSET,   KVDB_BLE_REGION_SIZE, 0}, \
}

#endif  /* __MEM_MAP__ */

