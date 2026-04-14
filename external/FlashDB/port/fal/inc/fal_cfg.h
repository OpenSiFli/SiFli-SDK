/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtconfig.h>
#include <board.h>

#ifdef PKG_USING_FLASHDB
#include "fdb_def.h"
#endif

#define FAL_PART_HAS_TABLE_CFG

#ifndef NOR_FLASH1_DEV_NAME
    #define NOR_FLASH1_DEV_NAME             "flash1"
#endif /* NOR_FLASH1_DEV_NAME */
#ifndef NOR_FLASH2_DEV_NAME

    #define NOR_FLASH2_DEV_NAME             "flash2"
#endif /* NOR_FLASH2_DEV_NAME */

#ifndef NOR_FLASH3_DEV_NAME
    #define NOR_FLASH3_DEV_NAME             "flash3"
#endif /* NOR_FLASH3_DEV_NAME */

#ifndef NOR_FLASH4_DEV_NAME
    #define NOR_FLASH4_DEV_NAME             "flash4"
#endif /* NOR_FLASH4_DEV_NAME */

#ifndef SDMMC1_DEV_NAME
    #define SDMMC1_DEV_NAME                 "sd0"
#endif /* SDMMC1_DEV_NAME */

#ifndef SDMMC2_DEV_NAME
    #define SDMMC2_DEV_NAME                 "sd1"
#endif /* SDMMC2_DEV_NAME */

#if defined (SOLUTION)
#define FAL_PART_DEF(flash_part_id)      \
    {FAL_PART_MAGIC_WORD,                \
     FLASH_PART_NAME(flash_part_id),     \
     FLASH_PART_DEVICE(flash_part_id),   \
     FLASH_PART_RESET(flash_part_id),    \
     FLASH_PART_PATH(flash_part_id),     \
     FLASH_PART_OFFSET(flash_part_id),   \
     FLASH_PART_SIZE(flash_part_id), 0}
#define FAL_FS_PART_DEF(flash_part_id)   \
    {FAL_PART_MAGIC_WORD,                \
     FLASH_PART_NAME(flash_part_id),     \
     FLASH_PART_DEVICE(flash_part_id),   \
     FLASH_PART_RESET(flash_part_id),    \
     FLASH_PART_PATH(flash_part_id),     \
     FLASH_PART_OFFSET(flash_part_id),   \
     FLASH_PART_SIZE(flash_part_id), FAL_FS_PART_FLAG}
#else
#define FAL_PART_DEF(flash_part_id)      \
    {FAL_PART_MAGIC_WORD,                \
     FLASH_PART_NAME(flash_part_id),     \
     FLASH_PART_DEVICE(flash_part_id),   \
     FLASH_PART_OFFSET(flash_part_id),   \
     FLASH_PART_SIZE(flash_part_id), 0}
#define FAL_FS_PART_DEF(flash_part_id)   \
    {FAL_PART_MAGIC_WORD,                \
     FLASH_PART_NAME(flash_part_id),     \
     FLASH_PART_DEVICE(flash_part_id),   \
     FLASH_PART_OFFSET(flash_part_id),   \
     FLASH_PART_SIZE(flash_part_id), FAL_FS_PART_FLAG}
#endif

#define FAL_FS_PART_FLAG                 (1)

/* partition magic word */
#define FAL_PART_MAGIC_WORD         0x45503130
#define FAL_PART_MAGIC_WORD_H       0x4550L
#define FAL_PART_MAGIC_WORD_L       0x3130L
#define FAL_PART_MAGIC_WROD         0x45503130

extern const struct fal_flash_dev nor_flash1;
extern const struct fal_flash_dev nor_flash2;
extern const struct fal_flash_dev nor_flash3;
extern const struct fal_flash_dev nor_flash4;
extern const struct fal_flash_dev fal_sdmmc1;
extern const struct fal_flash_dev fal_sdmmc2;

#ifdef BSP_USING_PC_SIMULATOR
/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &nor_flash1,                                                     \
    &nor_flash2,                                                     \
    &nor_flash3,                                                     \
    &nor_flash4,                                                     \
    &fal_sdmmc1,                                                     \
    &fal_sdmmc2,                                                     \
}
#endif

/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
#ifndef SOLUTION
/* customized FAL_PART_TABLE can be defined in board.h */
#ifndef FAL_PART_TABLE
/* partition table */
#define FAL_PART_TABLE                                                               \
{                                                                                    \
    {FAL_PART_MAGIC_WORD,       "dfu",      NOR_FLASH1_DEV_NAME,    0x18000,    16*1024, 0}, \
    {FAL_PART_MAGIC_WORD,       "ble",      NOR_FLASH1_DEV_NAME,    0x1c000,    16*1024, 0}, \
    {FAL_PART_MAGIC_WORD,       "prefdb",   NOR_FLASH1_DEV_NAME,    0x1f0000,   64*1024, 0}, \
}
#endif /* !FAL_PART_TABLE */
#else
#include "flash_map.h"
#endif /* !SOLUTION */
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
