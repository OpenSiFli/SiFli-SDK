/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "rwnx_defs.h"
#include "rwnx_rx.h"
//#include "rwnx_tx.h"
//#include "rwnx_main.h"
#include "rwnx_platform.h"
#include "aicwf_txrxif.h"
#include "aicwf_config.h"
#include "mac.h"
#include "mac_frame.h"
#ifdef CONFIG_VNET_MODE
    #include "netal_service.h"
    #include <rtthread.h>
    #include <wlan_dev.h>
    #include "aic8800mc_wlan_port.h"
#endif

#ifdef CONFIG_DATA_TXRX

static int rwnx_rxbuf_eth_ind(struct rx_buf_tag *rx_buf)
{
#if defined(CONFIG_VNET_MODE)
    int ret;
    struct hw_rxhdr *hw_rxhdr = &rx_buf->hwrxhdr;
    uint8_t *frame = (uint8_t *)rx_buf->payload;
    struct mac_hdr *machdr_ptr = (struct mac_hdr *)frame;
    uint8_t mac_hdr_len = sizeof(struct mac_hdr);
    struct mac_addr *da, *sa;
    struct mac_addr mac_da;
    struct mac_eth_hdr *eth_hdr;
    uint32_t ethhdr_ofst;
    int eth_totollen;
    if (machdr_ptr->fctl & MAC_FCTRL_TODS)  // Get DA
    {
        da = (void *)&machdr_ptr->addr3;
        sa = (void *)&machdr_ptr->addr2;
    }
    else
    {
        da = (void *)&machdr_ptr->addr1;
        sa = (void *)&machdr_ptr->addr3;
    }
    MAC_ADDR_CPY(&mac_da, da);
    if ((machdr_ptr->fctl & MAC_FCTRL_TYPESUBTYPE_MASK) == MAC_FCTRL_QOS_DATA)
    {
        mac_hdr_len += 2;
    }
    if (machdr_ptr->fctl & MAC_FCTRL_ORDER)
    {
        mac_hdr_len += MAC_HTCTRL_LEN;
    }
    switch (hw_rxhdr->hwvect.decr_status)
    {
    case RWNX_RX_HD_DECR_CCMP128:
    case RWNX_RX_HD_DECR_TKIP:
        mac_hdr_len += MAC_IV_LEN + MAC_EIV_LEN;
        break;
    case RWNX_RX_HD_DECR_WEP:
        mac_hdr_len += MAC_IV_LEN;
        break;
    default:
        break;
    }
    ethhdr_ofst = mac_hdr_len + sizeof(struct llc_snap) - sizeof(struct mac_eth_hdr);
    eth_hdr = (struct mac_eth_hdr *)(frame + ethhdr_ofst);
    eth_totollen = (int)hw_rxhdr->hwvect.len - ethhdr_ofst;
    // fill the ethernet header, order: type, sa, da
    MAC_ADDR_CPY((void *)&eth_hdr->sa, sa);
    MAC_ADDR_CPY((void *)&eth_hdr->da, &mac_da);
#ifdef RT_USING_WIFI
    aic8800mc_wlan_rx_data(eth_hdr, eth_totollen);
    ret = 0;
#else
    ret = netal_eth_protocol_recv(eth_hdr, eth_totollen);
#endif

#elif defined(CONFIG_RAWDATA_MODE)
#if 0
    int ret = 0;
    ret = nlaic_rx_rawdata(skb->data, skb->len);
    if (ret)
    {
        printk("rx rawdata err=%d\n", ret);
    }
    dev_kfree_skb(skb);
#endif
#endif
    return ret;
}

#ifdef AICWF_RX_REORDER
void reord_rxframe_free(spinlock_t *lock, struct list_head *q, struct list_head *list)
{
    spin_lock_bh(lock);
    list_add(list, q);
    spin_unlock_bh(lock);
}

struct recv_msdu *reord_rxframe_alloc(spinlock_t *lock, struct list_head *q)
{
    struct recv_msdu *rxframe;

    spin_lock_bh(lock);
    if (list_empty(q))
    {
        spin_unlock_bh(lock);
        return NULL;
    }
    rxframe = list_entry(q->next, struct recv_msdu, rxframe_list);
    list_del_init(q->next);
    spin_unlock_bh(lock);
    return rxframe;
}

struct reord_ctrl_info *reord_init_sta(const uint8_t *mac_addr)
{
    return NULL;
}

int reord_flush_tid(uint8_t *mac, uint8_t tid)
{
    return 0;
}

void reord_deinit_sta(struct reord_ctrl_info *reord_info)
{
    int idx;
    uint8_t *mac_addr = &reord_info->mac_addr[0];
    WRN_REORD("reord_deinit_sta %02x:%02x:%02x\n", mac_addr[0], mac_addr[1], mac_addr[2]);
    for (idx = 0; idx < 8; idx++)
    {
        struct reord_ctrl *preorder_ctrl = &reord_info->preorder_ctrl[idx];
        struct recv_msdu *prframe;
        preorder_ctrl->enable = false;
        rtos_mutex_lock(preorder_ctrl->reord_list_lock, -1);
        do
        {
            prframe = (struct recv_msdu *)co_list_pop_front(&preorder_ctrl->reord_list);
            if (prframe)
            {
                reord_rxframe_free(prframe);
            }
        }
        while (prframe);
        rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
        rtos_mutex_delete(preorder_ctrl->reord_list_lock);
        if (preorder_ctrl->reord_timer)
        {
#if (AICWF_RWNX_TIMER_EN)
            rwnx_timer_delete(preorder_ctrl->reord_timer);
#else
            rtos_timer_delete(preorder_ctrl->reord_timer, 0);
#endif
            preorder_ctrl->reord_timer = NULL;
        }
    }
    rtos_free(reord_info);
}

int reord_single_frame_ind(struct aicwf_rx_priv *rx_priv, struct recv_msdu *prframe)
{
    struct list_head *rxframes_freequeue = NULL;
    struct sk_buff *skb = NULL;

    rxframes_freequeue = &rx_priv->rxframes_freequeue;
    skb = prframe->pkt;
    if (skb == NULL)
    {
        txrx_err("skb is NULL\n");
        return -1;
    }

#ifdef CONFIG_VNET_MODE
    if (vnet_dev == NULL)
    {
        txrx_err("vnet_dev is NULL\n");
        return -1;
    }
    //printk("reord_single_frame_ind sn=%d, len=%d\n", prframe->seq_num, skb->len);

    skb->data = prframe->rx_data;
    skb_set_tail_pointer(skb, prframe->len);
    skb->len = prframe->len;
    skb->dev = vnet_dev;
    skb->protocol = eth_type_trans(skb, vnet_dev);

    memset(skb->cb, 0, sizeof(skb->cb));
    if (in_interrupt())
    {
        netif_rx(skb);
    }
    else
    {
        /*
        * If the receive is not processed inside an ISR, the softirqd must be woken explicitly to service the NET_RX_SOFTIRQ.
        * * In 2.6 kernels, this is handledby netif_rx_ni(), but in earlier kernels, we need to do it manually.
        */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
        netif_rx_ni(skb);
#else
        ulong flags;
        netif_rx(skb);
        local_irq_save(flags);
        RAISE_RX_SOFTIRQ();
        local_irq_restore(flags);
#endif
    }

#elif defined(CONFIG_RAWDATA_MODE)
    //TBD:
#endif

    prframe->pkt = NULL;
    reord_rxframe_free(&rx_priv->freeq_lock, rxframes_freequeue, &prframe->rxframe_list);

    return 0;
}

bool reord_rxframes_process(struct aicwf_rx_priv *rx_priv, struct reord_ctrl *preorder_ctrl, int bforced)
{
    struct list_head *phead, *plist;
    struct recv_msdu *prframe;
    bool bPktInBuf = false;

    if (bforced == true)
    {
        phead = &preorder_ctrl->reord_list;
        if (list_empty(phead))
        {
            return false;
        }

        plist = phead->next;
        prframe = list_entry(plist, struct recv_msdu, reord_pending_list);
        preorder_ctrl->ind_sn = prframe->seq_num;
    }

    phead = &preorder_ctrl->reord_list;
    if (list_empty(phead))
    {
        return bPktInBuf;
    }

    list_for_each_entry(prframe, phead, reord_pending_list)
    {
        if (!SN_LESS(preorder_ctrl->ind_sn, prframe->seq_num))
        {
            if (SN_EQUAL(preorder_ctrl->ind_sn, prframe->seq_num))
            {
                preorder_ctrl->ind_sn = (preorder_ctrl->ind_sn + 1) & 0xFFF;
            }
        }
        else
        {
            bPktInBuf = true;
            break;
        }
    }

    return bPktInBuf;
}

void reord_rxframes_ind(struct aicwf_rx_priv *rx_priv,
                        struct reord_ctrl *preorder_ctrl)
{
    struct list_head *phead, *plist;
    struct recv_msdu *prframe;

    phead = &preorder_ctrl->reord_list;
    while (1)
    {
        //spin_lock_bh(&preorder_ctrl->reord_list_lock);
        if (list_empty(phead))
        {
            //spin_unlock_bh(&preorder_ctrl->reord_list_lock);
            break;
        }

        plist = phead->next;
        prframe = list_entry(plist, struct recv_msdu, reord_pending_list);

        if (!SN_LESS(preorder_ctrl->ind_sn, prframe->seq_num))
        {
            list_del_init(&(prframe->reord_pending_list));
            //spin_unlock_bh(&preorder_ctrl->reord_list_lock);
            reord_single_frame_ind(rx_priv, prframe);
        }
        else
        {
            //spin_unlock_bh(&preorder_ctrl->reord_list_lock);
            break;
        }
    }
}

void reord_timeout_handler(struct timer_list *t)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    struct reord_ctrl *preorder_ctrl = (struct reord_ctrl *)data;
#else
    struct reord_ctrl *preorder_ctrl = from_timer(preorder_ctrl, t, reord_timer);
#endif
    struct aicwf_rx_priv *rx_priv = preorder_ctrl->rx_priv;

    if (reord_rxframes_process(rx_priv, preorder_ctrl, true) == true)
    {
        mod_timer(&preorder_ctrl->reord_timer, jiffies + msecs_to_jiffies(REORDER_UPDATE_TIME));
    }

    if (!work_pending(&preorder_ctrl->reord_timer_work))
        schedule_work(&preorder_ctrl->reord_timer_work);
}

void reord_timeout_worker(struct work_struct *work)
{
    struct reord_ctrl *preorder_ctrl = container_of(work, struct reord_ctrl, reord_timer_work);
    struct aicwf_rx_priv *rx_priv = preorder_ctrl->rx_priv;

    spin_lock_bh(&preorder_ctrl->reord_list_lock);
    reord_rxframes_ind(rx_priv, preorder_ctrl);
    spin_unlock_bh(&preorder_ctrl->reord_list_lock);
    return ;
}

int reord_process_unit(struct aicwf_rx_priv *rx_priv, struct rx_buf_tag *rx_buf, u16 seq_num, u8 *mac_addr, u8 tid)
{
    return 0;
}

int reorder_list_flush_tid(struct aicwf_rx_priv *rx_priv, struct sk_buff *skb, u8 tid)
{
    struct reord_ctrl_info *reord_info;
    struct reord_ctrl *preorder_ctrl;
    struct ethhdr *eh = (struct ethhdr *)(skb->data);
    u8 *mac;
    unsigned long flags;
    u8 found = 0;
    struct list_head *phead, *plist;
    struct recv_msdu *prframe;
    int ret;
    //printk("flush:tid=%d", tid);

    mac = eh->h_dest;

    spin_lock_bh(&rx_priv->stas_reord_lock);
    list_for_each_entry(reord_info, &rx_priv->stas_reord_list, list)
    {
        if (!memcmp(mac, reord_info->mac_addr, ETH_ALEN))
        {
            found = 1;
            preorder_ctrl = &reord_info->preorder_ctrl[tid];
            break;
        }
    }
    if (!found)
    {
        spin_unlock_bh(&rx_priv->stas_reord_lock);
        return 0;
    }
    spin_unlock_bh(&rx_priv->stas_reord_lock);

    if (preorder_ctrl->enable == false)
        return 0;
    spin_lock_irqsave(&preorder_ctrl->reord_list_lock, flags);
    phead = &preorder_ctrl->reord_list;
    while (1)
    {
        if (list_empty(phead))
        {
            break;
        }
        plist = phead->next;
        prframe = list_entry(plist, struct recv_msdu, reord_pending_list);
        reord_single_frame_ind(rx_priv, prframe);
        list_del_init(&(prframe->reord_pending_list));
    }

    preorder_ctrl->enable = false;
    spin_unlock_irqrestore(&preorder_ctrl->reord_list_lock, flags);
    if (timer_pending(&preorder_ctrl->reord_timer))
        ret = del_timer_sync(&preorder_ctrl->reord_timer);
    cancel_work_sync(&preorder_ctrl->reord_timer_work);

    return 0;
}

int reord_need_check(struct reord_ctrl *preorder_ctrl, u16 seq_num)
{
    u8 wsize = preorder_ctrl->wsize_b;
    u16 wend = (preorder_ctrl->ind_sn + wsize - 1) & 0xFFF;

    if (preorder_ctrl->ind_sn == 0xFFFF)
    {
        preorder_ctrl->ind_sn = seq_num;
    }

    if (SN_LESS(seq_num, preorder_ctrl->ind_sn))
    {
        return -1;
    }

    if (SN_EQUAL(seq_num, preorder_ctrl->ind_sn))
    {
        preorder_ctrl->ind_sn = (preorder_ctrl->ind_sn + 1) & 0xFFF;
    }
    else if (SN_LESS(wend, seq_num))
    {
        if (seq_num >= (wsize - 1))
            preorder_ctrl->ind_sn = seq_num - (wsize - 1);
        else
            preorder_ctrl->ind_sn = 0xFFF - (wsize - (seq_num + 1)) + 1;
    }

    return 0;
}

int reord_rxframe_enqueue(struct reord_ctrl *preorder_ctrl, struct recv_msdu *prframe)
{
    struct list_head *preord_list = &preorder_ctrl->reord_list;
    struct list_head *phead, *plist;
    struct recv_msdu *pnextrframe;

    phead = preord_list;
    plist = phead->next;

    while (phead != plist)
    {
        pnextrframe = list_entry(plist, struct recv_msdu, reord_pending_list);
        if (SN_LESS(pnextrframe->seq_num, prframe->seq_num))
        {
            plist = plist->next;
            continue;
        }
        else if (SN_EQUAL(pnextrframe->seq_num, prframe->seq_num))
        {
            return -1;
        }
        else
        {
            break;
        }
    }
    list_add_tail(&(prframe->reord_pending_list), plist);

    return 0;
}

int rwnx_rxdataind_aicwf(struct rwnx_hw *rwnx_hw, void *hostid, void *rx_priv)
{
#ifdef CONFIG_VNET_MODE
    struct rx_buf_tag *rx_buf = (struct rx_buf_tag *)hostid
                                struct hw_rxhdr * hw_rxhdr = &rx_buf->hwrxhdr;
    uint8_t *frame = (uint8_t *)rx_buf->payload;
    struct mac_hdr *machdr_ptr = (struct mac_hdr *)frame;
    uint8_t is_qos = 0;
    uint16_t *qos;
    struct mac_addr *da, *sa;
    uint8_t macaddr[MAC_ADDR_LEN] = {0,};
    uint8_t tid = 0;

    if ((machdr_ptr->fctl & MAC_FCTRL_TYPE_MASK) == MAC_FCTRL_DATA_T)
    {
        if ((machdr_ptr->fctl & MAC_FCTRL_TYPESUBTYPE_MASK) == MAC_FCTRL_QOS_DATA)
        {
            struct mac_hdr_qos *machdr_qos_ptr = (struct mac_hdr_qos *)frame;
            is_qos = 1;
            qos = &machdr_qos_ptr->qos;
            tid = (*qos & MAC_QOSCTRL_UP_MSK);
        }
        if (machdr_ptr->fctl & MAC_FCTRL_TODS)  // Get DA
        {
            da = &machdr_ptr->addr3;
            sa = &machdr_ptr->addr2;
            memcpy(macaddr, sa, MAC_ADDR_LEN);
        }
        else
        {
            da = &machdr_ptr->addr1;
            sa = &machdr_ptr->addr3;
            memcpy(macaddr, da, MAC_ADDR_LEN);
        }
        if (is_qos && hw_rxhdr->flags_need_reord)
        {
            uint16_t seq_num = machdr_ptr->seq >> MAC_SEQCTRL_NUM_OFT;
            reord_process_unit(rx_priv, rx_buf, seq_num, macaddr, tid);
        }
        else if (is_qos  && !hw_rxhdr->flags_need_reord)
        {
            reord_flush_tid(macaddr, tid);
            rwnx_rxbuf_eth_ind(rx_buf);
        }
        else
        {
            rwnx_rxbuf_eth_ind(rx_buf);
        }
    }
    else
    {
        DBG_MACIF_ERR("Not data frame\r\n");
    }

#else
#endif

    return 0;
}

#else
int rwnx_rxdata_forward(struct rwnx_hw *rwnx_hw, void *hostid, void *rx_priv)
{
    struct rx_buf_tag *rx_buf = (struct rx_buf_tag *)hostid;
    struct hw_rxhdr *hw_rxhdr = &rx_buf->hwrxhdr;
    uint8_t *frame = (uint8_t *)rx_buf->payload;
    struct mac_hdr *machdr_ptr = (struct mac_hdr *)frame;
    if ((machdr_ptr->fctl & MAC_FCTRL_TYPE_MASK) == MAC_FCTRL_DATA_T)
    {
        rwnx_rxbuf_eth_ind(rx_buf);
    }
    else
    {
        DBG_MACIF_ERR("Not data frame\r\n");
    }

    return 0;
}
#endif

#endif
