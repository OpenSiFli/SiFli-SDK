/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LWIP_PORT_NETIF_STATE_H
#define LWIP_PORT_NETIF_STATE_H
#include <lwip/opt.h>
#ifdef __cplusplus
extern "C" {
#endif
void lwip_netif_state_on_timeout_init(void);
void lwip_netif_state_on_timeout_uninit(void);
void lwip_netif_state_on_link_up(void);
void lwip_netif_state_on_link_down(void);
int lwip_netif_state_timeout_is_init(void);
#ifdef __cplusplus
}
#endif
#endif /* LWIP_PORT_NETIF_STATE_H */