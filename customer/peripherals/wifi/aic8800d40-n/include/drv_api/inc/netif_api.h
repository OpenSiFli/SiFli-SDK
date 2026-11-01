/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NETIF_API_H_
#define _NETIF_API_H_

#include "aic_porting_types.h"
#include "netif_def.h"

typedef void (*aic_tx_cb)(uint32_t frame_id, bool acknowledged, void *arg);

uint32_t aic_offsetof(void);

uint8_t *aic_get_mac_address(void);
void aic_set_mac_address(uint8_t *addr);
void *aic_get_vif_mac_addr(void);
net_if_t aic_get_vif_net_if(void);
void aic_tx_start(net_if_t net_if, net_buf_tx_t *net_buf, aic_tx_cb cfm_cb, void *cfm_cb_arg);
#ifdef CONFIG_FILTER_TCP_ACK
    int aic_filter_send_tcp_ack(uint8_t *buf, uint32_t plen, void *msg);
#endif

/* Opaque access to fhost VIF for platform init */
void *aic_get_fhost_default_vif(void);
void  aic_fhost_vif_set_mac(void *vif, const uint8_t *mac);
const uint8_t *aic_fhost_vif_get_mac(void *vif);
int   aic_fhost_vif_index(void *vif);

/* Network interface lookup (from netif_co) */
net_if_t net_if_find_from_wifi_idx(unsigned int idx);

/* Per-vif readiness: netif 绑定 vif 已加入固件且 (STA) 已关联。
 * 并发模式下按接口精确门控, 替代全局模式判断 */
int aic_net_if_vif_ready(net_if_t net_if);

/* ---- Per-role netif query (STA / AP / P2P) ---- */

#define AIC_NETIF_NAME_LEN  16

typedef enum aic_if_role
{
    AIC_IF_ROLE_STA = 0,   /* STA 接口 (非 P2P) */
    AIC_IF_ROLE_AP,        /* SoftAP 接口 (非 P2P, 含 APSTA 并发中的 AP 面) */
    AIC_IF_ROLE_P2P,       /* P2P 接口 (client 或 GO) */
} aic_if_role_t;

struct aic_netif_info
{
    aic_if_role_t role;    /* 查询角色 (回填) */
    int vif_idx;           /* fhost vif index, -1 = 未找到 */
    net_if_t net_if;       /* netif handle, NULL = 未找到 */
    char name[AIC_NETIF_NAME_LEN]; /* 网卡名 (platform aicwf_net_if_get_name), '\0' = 未找到 */
};

/* 按角色查询当前活跃 netif (返回首个匹配 VIF)。
 * role 接口未创建/未加入固件时返回 -1 且 info 填充未找到语义。 */
int aic_netif_get_info(aic_if_role_t role, struct aic_netif_info *info);

#endif /* _NETIF_API_H_ */

