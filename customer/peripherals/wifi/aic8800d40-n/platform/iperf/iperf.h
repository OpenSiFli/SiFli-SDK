/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * iperf2-style network performance tool (TCP/UDP, server/client).
 * Ported from the t3x platform tree, see the iperf.c file header for details.
 *
 * Entry called from the iwpriv iperf start/stop command in wifi_port_cmd.c:
 *   handle = iperf_parse_argv(argc, argv)   0=success -1=bad args -2=handled in place
 *   iperf_start(nif, handle)                start traffic/report tasks
 *   iperf_stop("a")                         stop the running instance
 */

#ifndef _IPERF_H_
#define _IPERF_H_

#if 1 //PRJCONF_NET_EN

#include "lwip/netif.h"
#include "lwip/inet.h"
#include "rtos_port.h"

#ifdef __cplusplus
extern "C" {
#endif

int iperf_start(struct netif *nif, int handle);
int iperf_stop(char *arg);
int iperf_parse_argv(int argc, char *argv[]);
int iperf_handle_free(int handle);
int iperf_handle_start(struct netif *nif, int handle);

#ifdef __cplusplus
}
#endif

#endif /* PRJCONF_NET_EN */
#endif /* _IPERF_H_ */
