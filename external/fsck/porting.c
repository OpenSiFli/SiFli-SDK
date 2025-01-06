/*  $NetBSD: fsutil.c,v 1.15 2006/06/05 16:52:05 christos Exp $ */

/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 1990, 1993
 *  The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


#include <sys/stat.h>

#include <errno.h>
#ifndef __ANDROID__
#endif
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "fsutil.h"
#include <fcntl.h>
#ifdef _WIN32
    #include <io.h>
#endif
#include "dosfs.h"

#ifdef _WIN32
int port_read(int fd, void *buf, size_t len)
{
    int r = _read(fd, buf, len);
    return r;
}

int port_write(int fd, const void *buf, size_t len)
{
    return _write(fd, buf, len);
}
int port_open(const char *file, int flags, ...)
{
    return _open(file, flags | _O_BINARY);
}
int port_close(int d)
{
    return _close(d);
}

int port_lseek(int fd, int offset, int whence)
{
    int r = _lseek(fd, offset, whence);
    return r;
}
#else
#undef off_t
#include "rtthread.h"

static rt_device_t fat_dev;
static int fat_offset;
static int sector_size;
static int total_sz_in_bytes;

#ifndef SEEK_SET
typedef enum
{
    SEEK_SET = 0x00,      /**< Set the position from absolutely (from the start of file)*/
    SEEK_CUR = 0x01,      /**< Set the position from the current position*/
    SEEK_END = 0x02,      /**< Set the position from the end of the file*/
} ;
#endif

int port_read(int fd, void *buf, size_t len)
{
    int r;
    if (0 == sector_size)
    {
        return 0;
    }
    if (fat_offset % sector_size)
    {
        return 0;
    }
    if (len % sector_size)
    {
        return 0;
    }
    // printf("read:%d,%d,%d\n", fat_offset, sector_size, len);
    r = rt_device_read(fat_dev, (fat_offset / sector_size), buf, (len / sector_size));
    r *= sector_size;
    return r;
}

int port_write(int fd, const void *buf, size_t len)
{
    int r;
    if (0 == sector_size)
    {
        return 0;
    }
    if (fat_offset % sector_size)
    {
        return 0;
    }
    if (len % sector_size)
    {
        return 0;
    }
    // printf("write:%d,%d,%d\n", fat_offset, sector_size, len);
    r = rt_device_write(fat_dev, (fat_offset / sector_size), buf, (len / sector_size));
    r *= sector_size;
    return r;
}
int port_open(const char *file, int flags, ...)
{
    struct rt_device_blk_geometry geometry;
    fat_offset = 0;
    if (fat_dev)
    {
        rt_set_errno(-EBUSY);
        return -2;
    }
    /* open specific device */
    if (file == NULL)
    {
        /* which is a non-device filesystem mount */
        fat_dev = NULL;
    }
    else if ((fat_dev = rt_device_find(file)) == NULL)
    {
        /* no this device */
        rt_set_errno(-ENODEV);
        return -1;
    }
    if (fat_dev)
    {
        rt_device_open(fat_dev, RT_DEVICE_OFLAG_RDWR);
        if (RT_EOK == rt_device_control(fat_dev, RT_DEVICE_CTRL_BLK_GETGEOME, (void *)&geometry))
        {
            sector_size = geometry.bytes_per_sector;
            total_sz_in_bytes = geometry.bytes_per_sector * geometry.sector_count;
        }
        else
        {
            sector_size = 0;
            total_sz_in_bytes = 0;
        }
    }
    return 3;
}

int port_close(int d)
{
    fat_offset = 0;
    if (fat_dev)
        rt_device_close(fat_dev);

    fat_dev = NULL;
    return 0;
}

int port_lseek(int fd, int offset, int whence)
{
    if (whence == SEEK_SET)
        fat_offset = 0;
    else if (whence == SEEK_END)
        fat_offset = total_sz_in_bytes;

    fat_offset += offset;
    return fat_offset;
}
#endif
