/*
 * YAFFS: Yet another Flash File System . A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2018 Aleph One Ltd.
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1 as
 * published by the Free Software Foundation.
 *
 * Note: Only YAFFS headers are LGPL, YAFFS C code is covered by GPL.
 */


#ifndef __YAFFS_UTILSENV_H__
#define __YAFFS_UTILSENV_H__

#define YCHAR char
#define YUCHAR unsigned char

#ifndef u8
#define u8 unsigned char
#endif

#ifndef u16
#define u16 unsigned short
#endif

#ifndef u32
#define u32 unsigned int
#endif

#ifndef s32
#define s32 int
#endif


#include <sys/types.h>
#include <string.h>
#include "yaffs_list.h"
#include "yaffs_hweight.h"

#define hweight8(x) yaffs_hweight8(x)
#define hweight32(x) yaffs_hweight32(x)

#define yaffs_trace(...) do {} while (0)

#define LOFF_T_32_BIT


#ifndef S_IFREG
#define S_IFREG		0100000
#endif

#ifndef S_ISSOCK
#define S_ISSOCK(m)	(((m) & S_IFMT) == S_IFSOCK)
#endif
#ifndef S_ISLNK
#define S_ISLNK(m)	(((m) & S_IFMT) == S_IFLNK)
#endif
#ifndef S_ISDIR
#define S_ISDIR(m)	(((m) & S_IFMT) == S_IFDIR)
#endif
#ifndef S_ISREG
#define S_ISREG(m)	(((m) & S_IFMT) == S_IFREG)
#endif
#ifndef S_ISBLK
#define S_ISBLK(m)	(((m) & S_IFMT) == S_IFBLK)
#endif
#ifndef S_ISCHR
#define S_ISCHR(m)	(((m) & S_IFMT) == S_IFCHR)
#endif
#ifndef S_ISFIFO
#define S_ISFIFO(m)	(((m) & S_IFMT) == S_IFIFO)
#endif

#endif
