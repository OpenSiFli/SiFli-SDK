/*
 * File      : fal_cfg.h
 * This file is part of FAL (Flash Abstraction Layer) package
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-17     armink       the first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtconfig.h>
#include <board.h>

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


#define FAL_PART_DEF(flash_part_id)      \
    {FAL_PART_MAGIC_WORD,                \
     FLASH_PART_NAME(flash_part_id),     \
     FLASH_PART_DEVICE(flash_part_id),   \
     FLASH_PART_OFFSET(flash_part_id),   \
     FLASH_PART_SIZE(flash_part_id), 0}


#define FAL_FS_PART_FLAG                 (1)

#define FAL_FS_PART_DEF(flash_part_id)   \
    {FAL_PART_MAGIC_WORD,                \
     FLASH_PART_NAME(flash_part_id),     \
     FLASH_PART_DEVICE(flash_part_id),   \
     FLASH_PART_OFFSET(flash_part_id),   \
     FLASH_PART_SIZE(flash_part_id), FAL_FS_PART_FLAG}

/* partition magic word */
#define FAL_PART_MAGIC_WORD         0x45503130
#define FAL_PART_MAGIC_WORD_H       0x4550L
#define FAL_PART_MAGIC_WORD_L       0x3130L
#define FAL_PART_MAGIC_WROD         0x45503130

extern const struct fal_flash_dev nor_flash1;
extern const struct fal_flash_dev nor_flash2;
extern const struct fal_flash_dev nor_flash3;
extern const struct fal_flash_dev nor_flash4;

#ifdef BSP_USING_PC_SIMULATOR
/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &nor_flash1,                                                     \
    &nor_flash2,                                                     \
    &nor_flash3,                                                     \
    &nor_flash4,                                                     \
}
#endif

/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
#ifndef SOLUTION_WATCH
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
#endif /* !SOLUTION_WATCH */
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
