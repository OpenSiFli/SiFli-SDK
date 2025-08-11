/*!\file iperf.h
 *\brief This file provides support for iperf console commands.
 */
/*
 *  Copyright 2008-2020, 2024 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef _IPERF_H_
#define _IPERF_H_

#include <wmlog.h>

#define iperf_e(...) wmlog_e("iperf", ##__VA_ARGS__)
#define iperf_w(...) wmlog_w("iperf", ##__VA_ARGS__)

/** Register the Network Utility CLI command iperf.
 *
 *  \note This function can only be called by the application after \ref
 *  wlan_init() called.
 *
 *  \return WM_SUCCESS if the CLI commands are registered
 *  \return -WM_FAIL otherwise (for example if this function
 *          was called while the CLI commands were already registered)
 */

int iperf_cli_init();

/** Unregister Network Utility CLI command iperf.
 *
 *  \return WM_SUCCESS if the CLI commands are unregistered
 *  \return -WM_FAIL otherwise
 */

int iperf_cli_deinit();
#endif /*_IPERF_H_ */
