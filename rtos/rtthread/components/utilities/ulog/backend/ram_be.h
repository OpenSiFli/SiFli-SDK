/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-25     armink       the first version
 */

#ifndef _RAM_BE_H_
#define _RAM_BE_H_

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#if 0
rt_uint32_t ulog_ram_be_buf_size_get(void);

typedef void (*ulog_ram_be_buf_export_cb_t)(void *start, rt_uint32_t size);

void ulog_ram_be_buf_get(ulog_ram_be_buf_export_cb_t export);
#endif

typedef struct
{
    rt_bool_t full;
    rt_uint32_t wr_offset;
    rt_uint8_t buf[ULOG_RAM_BE_BUF_SIZE];
} ulog_ram_be_buf_t;

void *ulog_ram_be_buf_get(rt_uint32_t *size);

#ifdef __cplusplus
}
#endif

#endif /* _RAM_BE_H_ */
