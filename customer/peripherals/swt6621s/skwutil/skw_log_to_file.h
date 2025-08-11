/*****************************************************************
 *Copyright (C) 2021 Seekwave Tech Inc.
 *Filename : skw_sdio.h
 *Authors:seekwave platform
 *
 * This software is licensed under the terms of the the GNU
 * General Public License version 2, as published by the Free
 * Software Foundation, and may be copied, distributed, and
 * modified under those terms.
 *
 * This program is distributed in the hope that it will be usefull,
 * but without any warranty;without even the implied warranty of
 * merchantability or fitness for a partcular purpose. See the
 * GUN General Public License for more details.
 * **************************************************************/
#ifndef __SKW_LOG_H__
#define __SKW_LOG_H__

#include "drv_config.h"
#include "drv_io.h"
#include <rtthread.h>


/****************************************************************
 *Description:the skwsdio log define and the skwsdio data debug,
 *Func: skwsdio_log, skwsdio_err, skwsdio_data_pr;
 *Calls:
 *Call By:
 *Input: skwsdio log debug informations
 *Output:
 *Return：
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2022-07-18
 * **************************************************************/
#define skwlog_log(fmt, args...) \
    rt_kprintf("[SKWLOG]:" fmt, ## args)

#define skwlog_err(fmt, args...) \
    rt_kprintf("[SKWLOG_ERR]:" fmt, ## args)

#define skwlog_warn(fmt, args...) \
    rt_kprintf("[SKWLOG_WARN]:" fmt, ## args)

int skw_modem_log_init(struct sv6160_platform_data *p_data, struct file *fp, void *ucom);
void skw_modem_log_set_assert_status(uint32_t cp_assert);
void skw_modem_dumpmodem_start_rec(void);
void skw_modem_dumpmodem_stop_rec(void);
void skw_modem_log_start_rec(void);
void skw_modem_log_stop_rec(void);
void skw_modem_log_exit(void);
#endif
