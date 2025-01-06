/*
 * Copyright (c) 2020, Armink, <armink.ztl@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief configuration file
 */

#ifndef _FDB_CFG_H_
#define _FDB_CFG_H_

#include "rtthread.h"

/* using KVDB feature */
#define FDB_USING_KVDB

#ifdef FDB_USING_KVDB
/* Auto update KV to latest default when current KVDB version number is changed. @see fdb_kvdb.ver_num */
/* #define FDB_KV_AUTO_UPDATE */
#endif

/* using TSDB (Time series database) feature */
#define FDB_USING_TSDB

#ifdef PKG_FDB_USING_AUTO_MODE
#ifdef RT_USING_MTD_NOR
#define FDB_USING_FAL_MODE                                  /*Has NOR MTD, use FAL mode*/
#elif defined(RT_USING_MTD_NAND) && defined(RT_USING_DFS)   /*Has NAND MTD, use file mode*/
#ifdef RT_USING_POSIX
#define PKG_FDB_USING_FILE_POSIX_MODE
#else
#define PKG_FDB_USING_FILE_LIBC_MODE
#endif
#else
#error Could not select KVDB mode.
#endif
#endif

/* Using FAL storage mode */
#define FDB_USING_FAL_MODE
#if defined(PKG_FDB_USING_FILE_LIBC_MODE) || defined(PKG_FDB_USING_FILE_POSIX_MODE)
#undef FDB_USING_FAL_MODE
#endif /* PKG_FDB_USING_FAL_MODE */

#ifdef FDB_USING_FAL_MODE
/* the flash write granularity, unit: bit
 * only support 1(nor flash)/ 8(stm32f2/f4)/ 32(stm32f1) */
#define FDB_WRITE_GRAN      1          /* @note you must define it for a value */
#endif

/* Using file storage mode by LIBC file API, like fopen/fread/fwrte/fclose */

#ifdef PKG_FDB_USING_FILE_LIBC_MODE
#define FDB_USING_FILE_LIBC_MODE
#endif /* PKG_FDB_USING_FILE_LIBC_MODE */

/* Using file storage mode by POSIX file API, like open/read/write/close */
#ifdef PKG_FDB_USING_FILE_POSIX_MODE
#define FDB_USING_FILE_POSIX_MODE
#endif /* PKG_FDB_USING_FILE_POSIX_MODE */

/* MCU Endian Configuration, default is Little Endian Order. */
/* #define FDB_BIG_ENDIAN */ 

/* log print macro. default EF_PRINT macro is printf() */
/* #define FDB_PRINT(...)              my_printf(__VA_ARGS__) */

/* print debug information */
#define FDB_DEBUG_ENABLE

#endif /* _FDB_CFG_H_ */
