/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (C) 2018-2020 AICSemi Ltd.
 *
 * All Rights Reserved
 */

#ifndef NET_AL_H_
#define NET_AL_H_

//-------------------------------------------------------------------
// Driver Header Files
//-------------------------------------------------------------------
#include "netif_def.h"
#include "aic_plat_net.h"
#include "aic_plat_sock.h"
#include "aic_plat_types.h"

#if defined(RT_USING_SAL) && defined(SAL_USING_POSIX)
#if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 1))
#if defined(SAL_USING_AF_UNIX)
#define WIFI_USING_LOOPBACK_NETDEV
#else
#error "loopback netdev required"
#endif
#else // old version RTT
/// Force using loopback netdev, make sure SAL has merged sendmsg/recvmsg api & AF_UNIX feature
#define WIFI_USING_LOOPBACK_NETDEV
#endif
#endif

#define AICWF_WIFI_HDR          WIFI_HDR
#define AICWF_WIFI_HDR_LEN      WIFI_HDR_LEN

#ifndef CONFIG_LWIP
typedef int             err_t;
#endif

extern struct aic_netif_dev netif_dev;

//-------------------------------------------------------------------
// Driver platform functions
//-------------------------------------------------------------------
uint8_t *platform_net_buf_rx_alloc(uint32_t length);
void *platform_net_buf_rx_desc(uint8_t *buffer);
void platform_net_buf_rx_free(void *ref);

//-------------------------------------------------------------------
// Driver netif base API define
//-------------------------------------------------------------------
int net_if_add(net_if_t *net_if, const uint32_t *ipaddr,
                  const uint32_t *netmask, const uint32_t *gw, void *state_vif);
int net_if_get_name(net_if_t *net_if, char *buf, int len);
int net_if_get_wifi_idx(net_if_t *net_if);
void net_if_up(net_if_t *net_if);
void net_if_down(net_if_t *net_if);
void net_if_set_default(net_if_t *net_if);
void net_if_set_ip(net_if_t *net_if, uint32_t ip, uint32_t mask, uint32_t gw);
int net_if_get_ip(net_if_t *net_if, uint32_t *ip, uint32_t *mask, uint32_t *gw);

err_t net_if_init(struct netif *net_if);
int net_init(void);
void net_deinit(void);
void net_predeinit(void);

net_buf_tx_t *net_buf_tx_alloc(const uint8_t *payload, uint32_t length);
void net_buf_tx_free(net_buf_tx_t *buf);
uint8_t *net_buf_rx_alloc(uint32_t length);
void net_buf_rx_free(void *msg);
bool net_buf_tx_free_list_is_full(void);
int tx_eth_data_process(uint8_t *data, uint32_t len,void*msg);
#ifdef CONFIG_RX_NOCOPY
int rx_eth_data_process(unsigned char *pdata, unsigned short len, net_if_t *netif,
                              void *lwip_msg, uint8_t is_reord);
#else
int rx_eth_data_process(unsigned char *pdata, unsigned short len, struct netif *netif);
#endif /* CONFIG_RX_NOCOPY */
int tx_mon_data_process(uint8_t *pkt, uint32_t len);

//-------------------------------------------------------------------
// Driver other API define
//-------------------------------------------------------------------
int net_dhcp_start(int net_id);

#ifdef WIFI_USING_LOOPBACK_NETDEV
void net_loopback_netdev_register(void);
#endif

#endif // NET_AL_H_