// SPDX-License-Identifier: GPL-2.0

/******************************************************************************
 *
 * Copyright (C) 2020 SeekWave Technology Co.,Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 ******************************************************************************/

#include "skw_sdio.h"
#include "skw_iface.h"
#include "skw_sdio_core.h"
#include "skw_type_msg.h"
void mutex_init(rt_mutex_t mutex);
#define  atomic_set(x, y) x = y;
//void atomic_set(int x, int y)
struct skw_peer *skw_peer_alloc(void)
{
    int len;

    len = sizeof(struct skw_peer);
    len += sizeof(struct skw_ctx_entry);

    return rt_malloc(len);
}
void skw_peer_init(struct skw_peer *peer, const u8 *addr, int idx)
{
    int i;
    struct skw_ctx_entry *entry;

    if (idx >= SKW_MAX_PEER_SUPPORT)
        peer->flags |= SKW_PEER_FLAG_BAD_ID;

    if (!addr)
        peer->flags |= SKW_PEER_FLAG_BAD_ADDR;

    peer->rx_filter = 0;
    mutex_init(&peer->ptk_conf.lock);
    mutex_init(&peer->gtk_conf.lock);

    for (i = 0; i < SKW_NR_TID; i++)
    {
        peer->reorder[i].ref_cnt = 0;
        // skw_compat_setup_timer(&peer->reorder[i].timer, skw_reorder_timeout);
        // INIT_LIST_HEAD(&peer->reorder[i].todo.list);
        // spin_lock_init(&peer->reorder[i].todo.lock);
        // spin_lock_init(&peer->reorder[i].lock);
    }

    peer->idx = idx;
    peer->iface = NULL;
    peer->sm.addr = peer->addr;
    peer->sm.state = SKW_STATE_NONE;

    entry = skw_ctx_entry(peer);
    entry->peer = peer;
    entry->idx = idx;

    if (addr)
    {
        skw_ether_copy(entry->addr, addr);
        skw_ether_copy(peer->addr, addr);
    }
}
struct skw_peer_ctx *skw_get_ctx(struct skw_core *skw, u8 lmac_id, u8 idx)
{
    if (idx >= SKW_MAX_PEER_SUPPORT)
        return NULL;

    return &skw->hw.lmac[lmac_id].peer_ctx[idx];
}
struct skw_ctx_entry *skw_get_ctx_entry(struct skw_core *skw, const u8 *addr)
{
    int i, j;
    struct skw_ctx_entry *entry;

    for (i = 0; i < skw->hw.nr_lmac; i++)
    {
        for (j = 0; j < SKW_MAX_PEER_SUPPORT; j++)
        {
            entry = skw->hw.lmac[i].peer_ctx[j].entry;
            if (memcmp(entry->addr, addr, 6) == 0)
                return entry;
        }
    }

    return NULL;
}
void skw_purge_key_conf(struct skw_key_conf *conf)
{
    int idx;
    struct skw_key *key;

    if (!conf)
        return;

    mutex_lock(&conf->lock);

    for (idx = 0; idx < SKW_NUM_MAX_KEY; idx++)
    {
        key = conf->key[idx];
        if (!key)
            continue;

        // skw_del_key(key);
        conf->key[idx] = NULL;
    }

    conf->flags = 0;
    conf->installed_bitmap = 0;
    conf->skw_cipher = 0;

    mutex_unlock(&conf->lock);
}
void skw_peer_free(struct skw_peer *peer)
{
    int i;
    struct skw_ctx_entry *entry;

    if (!peer)
        return;

    // for (i = 0; i < SKW_NR_TID; i++)
    //  skw_del_tid_rx(peer, i);

    skw_purge_key_conf(&peer->ptk_conf);
    skw_purge_key_conf(&peer->gtk_conf);

    entry = skw_ctx_entry(peer);

}
int __skw_peer_ctx_bind(struct skw_iface *iface, struct skw_peer_ctx *ctx,
                        struct skw_peer *peer)
{
    iface->peer_map &= ~BIT(ctx->idx);
    skw_peer_free(ctx->peer);
    ctx->peer = NULL;

    if (peer)  //if
    {
        peer->iface = iface;
        peer->sm.inst = iface->id;
        peer->sm.addr = peer->addr;
        //peer->sm.iface_iftype = iface->wdev.iftype;
        ctx->peer = peer;

        iface->peer_map |= BIT(ctx->idx);

    }

    return 0;
}
int skw_peer_ctx_bind(struct skw_iface *iface, struct skw_peer_ctx *ctx,
                      struct skw_peer *peer)
{
    int ret;

    if (!iface || !ctx)
        return -1;

    rt_kprintf("ctx: %d, %s\n", ctx->idx, peer ? "bind" : "unbind");

    //mutex_lock(&ctx->lock);
    ret = __skw_peer_ctx_bind(iface, ctx, peer);
    //mutex_unlock(&ctx->lock);

    return ret;
}

int skw_lmac_bind_iface(struct skw_core *skw, struct skw_iface *iface, int lmac_id)
{
    struct skw_lmac *lmac;

    if (lmac_id >= skw->hw.nr_lmac)
    {
        rt_kprintf("invalid lmac id: %d\n", skw->hw.nr_lmac);
        return -1;
    }

    iface->lmac_id = lmac_id;
    lmac = &skw->hw.lmac[lmac_id];


    if (0) //(skw->hw.bus == SKW_BUS_PCIE) {
    {
        // TODO:
        // register edma channel
        // skw_edma_enable_channel(&lmac->edma_tx_chn, isr);
    }

    SKW_SET(lmac->iface_bitmap, BIT(iface->id));
    SKW_SET(lmac->flags, SKW_LMAC_FLAG_ACTIVED);

    return 0;
}
void skw_join_resp_handler(struct skw_core *skw,
                           struct skw_iface *iface,
                           struct skw_join_resp *resp)
{
    skw_lmac_bind_iface(iface->skw, iface, resp->lmac_id);
    iface->default_multicast = resp->multicast_idx;
}
