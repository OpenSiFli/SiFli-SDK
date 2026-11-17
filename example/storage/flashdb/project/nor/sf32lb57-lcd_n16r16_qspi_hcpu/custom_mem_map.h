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

/* The flashdb example keeps its key-value and time-series data in FAL
 * partitions, not in files, so this board needs two extra 16 KB regions
 * besides the dfu/ble key-value ones.  The offsets follow KVDB_BLE
 * (0xCE4000 + 0x4000) and stay inside the 16 MB NOR of this board. */
#define FAL_PART_TABLE \
{ \
    {FAL_PART_MAGIC_WORD,       "kvdb_tst",      NOR_FLASH2_DEV_NAME,    KVDB_TST_REGION_OFFSET,   KVDB_TST_REGION_SIZE, 0}, \
    {FAL_PART_MAGIC_WORD,       "tsdb_tst",      NOR_FLASH2_DEV_NAME,    TSDB_TST_REGION_OFFSET,   TSDB_TST_REGION_SIZE, 0}, \
    {FAL_PART_MAGIC_WORD,       "dfu",      NOR_FLASH2_DEV_NAME,    KVDB_DFU_REGION_OFFSET,   KVDB_DFU_REGION_SIZE, 0}, \
    {FAL_PART_MAGIC_WORD,       "ble",      NOR_FLASH2_DEV_NAME,    KVDB_BLE_REGION_OFFSET,   KVDB_BLE_REGION_SIZE, 0}, \
}

#endif  /* __CUSTOM_MEM_MAP__ */
