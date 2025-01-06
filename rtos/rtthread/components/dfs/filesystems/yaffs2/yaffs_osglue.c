/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2011 Aleph One Ltd.
 *   for Toby Churchill Ltd and Brightstar Engineering
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include "yaffs/yaffs_guts.h"
#include "yaffs/yaffs_trace.h"

#include "rtthread.h"

unsigned int yaffs_trace_mask = 0;

static int yaffsfs_lastError;
static rt_mutex_t mutex = RT_NULL;

void yaffsfs_SetError(int err)
{
    //Do whatever to set error
    yaffsfs_lastError = err;
}

int yaffsfs_GetLastError(void)
{
    return yaffsfs_lastError;
}

void yaffsfs_Lock(void)
{
    rt_mutex_take(mutex, RT_WAITING_FOREVER);
}

void yaffsfs_Unlock(void)
{
    rt_mutex_release(mutex);
}

void yaffsfs_LockInit(void)
{
    if (mutex == RT_NULL)
        mutex = rt_mutex_create("ymutex", RT_IPC_FLAG_PRIO);
}

u32 yaffsfs_CurrentTime(void)
{
    return 0;
}

RT_WEAK void *yaffsfs_malloc(size_t size)
{
    return rt_malloc(size);
}

RT_WEAK void yaffsfs_free(void *ptr)
{
    rt_free(ptr);
}

void yaffsfs_OSInitialisation(void)
{
    yaffsfs_LockInit();
}

int yaffsfs_CheckMemRegion(const void *addr, size_t size, int write_request)
{
    return 0;
}

void yaffs_bug_fn(const char *file_name, int line_no)
{
    rt_kprintf("%s: %s\n", file_name, line_no);
    RT_ASSERT(0);
}

size_t strnlen(const char *s, size_t m)
{
    size_t n;

    for (n = 0; n < m; n ++)
    {
        if (s[n] == '\0')
        {
            n++;
            break;
        }
    }

    return n;
}

