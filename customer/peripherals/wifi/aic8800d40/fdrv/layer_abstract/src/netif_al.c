/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "tx_swdesc.h"
#include "fhost.h"
#include "fhost_tx.h"
#include "fhost_config.h"
#include "netif_al.h"
#include "rwnx_platform.h"

uint32_t aic_offsetof(void)
{
    uint32_t offset = offsetof(struct hostdesc, flags) + MEMBER_SIZE(struct hostdesc, flags);
    return offset;
}

uint8_t* aic_get_mac_address(void)
{
    return get_mac_address();
}

void aic_set_mac_address(uint8_t *addr)
{
    set_mac_address(addr);
}

void *aic_get_vif_mac_addr(void)
{
    void *mac_addr = NULL;
    struct fhost_vif_tag *vif;
    vif = &fhost_env.vif[0];
    mac_addr = (void *)&vif->mac_addr;
    return mac_addr;
}

net_if_t *aic_get_vif_net_if(void)
{
    net_if_t *net_if = NULL;
    struct fhost_vif_tag *vif;
    vif = &fhost_env.vif[0];
    net_if = vif->net_if;
    return net_if;
}

void aic_tx_start(net_if_t *net_if, net_buf_tx_t *net_buf, aic_tx_cb cfm_cb, void *cfm_cb_arg)
{
    fhost_tx_start(net_if, net_buf, (cb_fhost_tx)cfm_cb, cfm_cb_arg);
}

int aic_monitor_tx(net_if_t *net_if, uint8_t *pkt, uint32_t len)
{
    int ret = 0;
    do {
        struct fhost_vif_tag *vif;
        int fvif_idx;
        if (pkt == NULL) {
            DBG_NETAL_ERR("null pkt\n");
            ret = -1;
            break;
        }
        if (!len) {
            DBG_NETAL_ERR("zero len\n");
            ret = -2;
            break;
        }
        // Get the private information about the VIF
        vif = net_if_vif_info(net_if);
        fvif_idx = fhost_vif_idx_from_vif_tag(vif);
        if ((fvif_idx < 0) || (fvif_idx >= NX_VIRT_DEV_MAX)) {
            DBG_NETAL_ERR("invalid fvif=%p, idx=%d\n", vif, fvif_idx);
            ret = -2;
            break;
        }
        fhost_send_80211_frame(fvif_idx, pkt, len, NULL, NULL);
    } while (0);
    return ret;
}