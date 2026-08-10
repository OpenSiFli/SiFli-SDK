/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _NETIF_CO_H_
#define _NETIF_CO_H_

#include "netif_def.h"
#include "netif_port.h"
#include "aic_plat_types.h"

#ifndef ETH_ALEN
#define ETH_ALEN                (6)
#endif
#define NET_AL_MAX_IFNAME       (4)

net_if_t *net_if_find_from_name(const char *name);
const uint8_t *net_if_get_mac_addr(net_if_t *net_if);
struct fhost_vif_tag *net_if_vif_info(net_if_t *net_if);
net_if_t *net_if_find_from_wifi_idx(unsigned int idx);
net_if_t *net_if_from_fvif_idx(unsigned int idx);

//-------------------------------------------------------------------
// Driver EAP pkt API define
//-------------------------------------------------------------------
err_t net_eth_receive(unsigned char *pdata, unsigned short len, struct netif *netif);

//-------------------------------------------------------------------
// Driver netif l2 API define
//-------------------------------------------------------------------
int net_l2_send(net_if_t *net_if, const uint8_t *data, int data_len,
                uint16_t ethertype, const uint8_t *dst_addr, bool *ack);
int net_l2_socket_create(net_if_t *net_if, uint16_t ethertype);
int net_l2_socket_delete(int sock);

//-------------------------------------------------------------------
// Driver other API define
//-------------------------------------------------------------------
char* aic_inet_ntoa(struct in_addr addr);

#endif /* _NETIF_CO_H_ */