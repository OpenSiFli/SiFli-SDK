/** @file wifi_ping.h
 *
 *  @brief  This file provides the support for network utility ping
 */
/*
 *  Copyright 2008-2020, 2024 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef _WIFI_PING_H_
#define _WIFI_PING_H_

#include <wmlog.h>

#define ping_e(...) wmlog_e("ping", ##__VA_ARGS__)
#define ping_w(...) wmlog_w("ping", ##__VA_ARGS__)

#define PING_ID                  0xAFAFU
#define PING_INTERVAL            1000
#define PING_DEFAULT_TIMEOUT_SEC 2
#define PING_DEFAULT_COUNT       10
#define PING_DEFAULT_SIZE        56
#define PING_MAX_SIZE            65507U
#define PING_MAX_COUNT           65535U

/** Register Network Utility CLI commands.
 *
 *  Register the Network Utility CLI commands. Currently, only ping command is
 *  supported.
 *
 *  \note This function can only be called by the application after \ref
 *  wlan_init() called.
 *
 *  \return WM_SUCCESS if the CLI commands are registered
 *  \return -WM_FAIL otherwise (for example if this function
 *          was called while the CLI commands were already registered)
 */

int ping_cli_init(void);

#if defined(SDK_OS_FREE_RTOS)
    int ping(uint16_t count, int interval, unsigned short size, unsigned int r_timeout, ip_addr_t *addr);
#elif __ZEPHYR__
    int ping(uint16_t count, int interval, unsigned short size, unsigned int r_timeout, const char *addr);
#endif
void ping_stats(int *total, int *recvd);

/** Unregister Network Utility CLI commands.
 *
 *  Unregister the Network Utility CLI commands.
 *
 *  \return WM_SUCCESS if the CLI commands are unregistered
 *  \return -WM_FAIL otherwise
 */

int ping_cli_deinit(void);
#endif /*_WIFI_PING_H_ */
