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

#include "aicwf_txrxif.h"
#include "rwnx_platform.h"
#include "rwnx_defs.h"
//#include "rwnx_rx.h"
#ifdef AICWF_SDIO_SUPPORT
    #include "sdio_host.h"
#endif
#include "aicwf_sdio.h"

#define CFG_BUS_BUF_ALWAYS_IN_HEAP  0

#define BUS_RX_BUF_COUNT    16
#define BUS_RX_BUF_SIZE     2048
#define BUS_TX_BUF_COUNT    8
#define BUS_TX_BUF_SIZE     1600
#define SYS_ALIGN_SIZE      4U
#define BUS_BUF_ALIGNED(addr)   (((uint32_t)(addr) + (SYS_ALIGN_SIZE - 1)) & (~(SYS_ALIGN_SIZE - 1)))

static struct aicwf_buf_list_s bus_rx_buf_list;
static struct aicwf_buf_node_s bus_rx_buf_node[BUS_RX_BUF_COUNT];
#if !CFG_BUS_BUF_ALWAYS_IN_HEAP
    static uint8_t bus_rx_buf_pool[BUS_RX_BUF_COUNT][BUS_RX_BUF_SIZE];
#endif

#ifdef CONFIG_DATA_TXRX
    static struct aicwf_buf_list_s bus_tx_buf_list;
    static struct aicwf_buf_node_s bus_tx_buf_node[BUS_TX_BUF_COUNT];
    #if !CFG_BUS_BUF_ALWAYS_IN_HEAP
        static uint8_t bus_tx_buf_pool[BUS_TX_BUF_COUNT][BUS_TX_BUF_SIZE];
    #endif
#endif

#if 0
    #include "bf0_hal_def.h"

    HAL_RETM_BSS_SECT(aic_bus_rxbuf, static struct aicwf_buf_node_s bus_rx_buf_node[BUS_RX_BUF_COUNT]);
    HAL_RETM_BSS_SECT(aic_bus_rxbuf, static uint8_t bus_rx_buf_pool[BUS_RX_BUF_COUNT][BUS_RX_BUF_SIZE]);
    HAL_RETM_BSS_SECT(aic_bus_rxbuf, static struct aicwf_buf_list_s bus_rx_buf_list);

    #ifdef CONFIG_DATA_TXRX
        HAL_RETM_BSS_SECT(aic_bus_txbuf, static struct aicwf_buf_node_s bus_tx_buf_node[BUS_TX_BUF_COUNT]);
        HAL_RETM_BSS_SECT(aic_bus_txbuf, static uint8_t bus_tx_buf_pool[BUS_TX_BUF_COUNT][BUS_TX_BUF_SIZE]);
        HAL_RETM_BSS_SECT(aic_bus_txbuf, static struct aicwf_buf_list_s bus_tx_buf_list);
    #endif

#endif

void aicwf_busrx_buf_init(void)
{
    int idx, ret;
    ret = rtos_mutex_create(&bus_rx_buf_list.mutex, "bus_rx_buf_list.mutex");
    if (ret)
    {
        DBG_XHCI_ERR("bus rx buf mutex create fail: %d\n", ret);
        return;
    }
    co_list_init(&bus_rx_buf_list.list);
    if (rtos_semaphore_create(&bus_rx_buf_list.ava_node_sema, "bus_rx_buf_list.ava_node_sema", BUS_RX_BUF_COUNT, 0))
    {
        DBG_XHCI_ERR("bus ava_node_sema create fail\n");
        return;
    }
    for (idx = 0; idx < BUS_RX_BUF_COUNT; idx++)
    {
        struct aicwf_buf_node_s *node = &bus_rx_buf_node[idx];
#if CFG_BUS_BUF_ALWAYS_IN_HEAP
        node->buf_raw = rtos_malloc(BUS_RX_BUF_SIZE + SYS_ALIGN_SIZE);
        if (node->buf_raw == NULL)
        {
            DBG_XHCI_ERR("busrx buf init fail(len=%d)!!!\n", BUS_RX_BUF_SIZE);
            break;
        }
#else
        node->buf_raw = &bus_rx_buf_pool[idx][0];
#endif
        node->buf = NULL;
        node->buf_len = 0;
        node->pad_len = 0;
        co_list_push_back(&bus_rx_buf_list.list, &node->hdr);
        rtos_semaphore_signal(bus_rx_buf_list.ava_node_sema, 0);
    }
    DBG_XHCI_INF("bus ava_node_sema initial count:%d\n", rtos_semaphore_get_count(bus_rx_buf_list.ava_node_sema));
}

struct aicwf_buf_node_s *aicwf_busrx_buf_alloc(uint16_t buf_len)
{
    struct aicwf_buf_node_s *node = NULL;

    int ret = rtos_semaphore_wait(bus_rx_buf_list.ava_node_sema, 100);

    if (ret == 0)
    {
        rtos_mutex_lock(bus_rx_buf_list.mutex, -1);
        node = (struct aicwf_buf_node_s *)co_list_pop_front(&bus_rx_buf_list.list);
        if (buf_len > BUS_RX_BUF_SIZE)
        {
            uint8_t *buf_raw = rtos_malloc(buf_len + SYS_ALIGN_SIZE);
            if (buf_raw == NULL)
            {
                DBG_XHCI_ERR("bus buf alloc fail(len=%d)!!!\n", buf_len);
                node->buf = NULL;
                node->buf_len = 0;
                node->pad_len = 0;
            }
            else
            {
                node->buf = (uint8_t *)BUS_BUF_ALIGNED(buf_raw);
                node->buf_len = buf_len;
                node->pad_len = node->buf - buf_raw;
            }
        }
        else
        {
#if CFG_BUS_BUF_ALWAYS_IN_HEAP
            node->buf = (uint8_t *)BUS_BUF_ALIGNED(node->buf_raw);
#else
            node->buf = node->buf_raw;
#endif
            node->buf_len = buf_len;
            node->pad_len = 0;
        }
        rtos_mutex_unlock(bus_rx_buf_list.mutex);
    }
    return node;
}

void aicwf_busrx_buf_free(struct aicwf_buf_node_s *node)
{
    rtos_mutex_lock(bus_rx_buf_list.mutex, -1);
    if (node->buf_len == 0)
    {
        DBG_XHCI_ERR("null bus buf free, buf=%p\n", node->buf);
    }
    else if (node->buf_len > BUS_RX_BUF_SIZE)
    {
        uint8_t *buf_raw = node->buf - node->pad_len;
        rtos_free(buf_raw);
    }
    node->buf = NULL;
    node->buf_len = 0;
    node->pad_len = 0;
    co_list_push_back(&bus_rx_buf_list.list, &node->hdr);
    rtos_mutex_unlock(bus_rx_buf_list.mutex);
    rtos_semaphore_signal(bus_rx_buf_list.ava_node_sema, 0);
}

void aicwf_busrx_buf_deinit(void)
{
    rtos_mutex_delete(bus_rx_buf_list.mutex);
    rtos_semaphore_delete(bus_rx_buf_list.ava_node_sema);
}

#ifdef CONFIG_DATA_TXRX
void aicwf_bustx_buf_init(void)
{
    int idx, ret;
    ret = rtos_mutex_create(&bus_tx_buf_list.mutex, "bus_tx_buf_list.mutex");
    if (ret)
    {
        DBG_XHCI_ERR("bus tx buf mutex create fail: %d\n", ret);
        return;
    }
    co_list_init(&bus_tx_buf_list.list);
    if (rtos_semaphore_create(&bus_tx_buf_list.ava_node_sema, "bus_tx_buf_list.ava_node_sema", BUS_TX_BUF_COUNT, 0))
    {
        DBG_XHCI_ERR("bus ava_node_sema create fail\n");
        return;
    }
    for (idx = 0; idx < BUS_TX_BUF_COUNT; idx++)
    {
        struct aicwf_buf_node_s *node = &bus_tx_buf_node[idx];
#if CFG_BUS_BUF_ALWAYS_IN_HEAP
        node->buf_raw = rtos_malloc(BUS_TX_BUF_SIZE + SYS_ALIGN_SIZE);
        if (node->buf_raw == NULL)
        {
            DBG_XHCI_ERR("bustx buf init fail(len=%d)!!!\n", BUS_TX_BUF_SIZE);
            break;
        }
#else
        node->buf_raw = &bus_tx_buf_pool[idx][0];
#endif
        node->buf = NULL;
        node->buf_len = 0;
        node->pad_len = 0;
        co_list_push_back(&bus_tx_buf_list.list, &node->hdr);
        rtos_semaphore_signal(bus_tx_buf_list.ava_node_sema, 0);
    }
    DBG_XHCI_INF("bus ava_node_sema initial count:%d\n", rtos_semaphore_get_count(bus_tx_buf_list.ava_node_sema));
}

struct aicwf_buf_node_s *aicwf_bustx_buf_alloc(uint16_t buf_len)
{
    struct aicwf_buf_node_s *node = NULL;

    int ret = rtos_semaphore_wait(bus_tx_buf_list.ava_node_sema, 100);

    if (ret == 0)
    {
        rtos_mutex_lock(bus_tx_buf_list.mutex, -1);
        node = (struct aicwf_buf_node_s *)co_list_pop_front(&bus_tx_buf_list.list);
        if (buf_len > BUS_TX_BUF_SIZE)
        {
            uint8_t *buf_raw = rtos_malloc(buf_len + SYS_ALIGN_SIZE);
            if (buf_raw == NULL)
            {
                DBG_XHCI_ERR("bus buf alloc fail(len=%d)!!!\n", buf_len);
                node->buf = NULL;
                node->buf_len = 0;
                node->pad_len = 0;
            }
            else
            {
                node->buf = (uint8_t *)BUS_BUF_ALIGNED(buf_raw);
                node->buf_len = buf_len;
                node->pad_len = node->buf - buf_raw;
            }
        }
        else
        {
#if CFG_BUS_BUF_ALWAYS_IN_HEAP
            node->buf = (uint8_t *)BUS_BUF_ALIGNED(node->buf_raw);
#else
            node->buf = node->buf_raw;
#endif
            node->buf_len = buf_len;
            node->pad_len = 0;
        }
        rtos_mutex_unlock(bus_tx_buf_list.mutex);
    }
    return node;
}

void aicwf_bustx_buf_free(struct aicwf_buf_node_s *node)
{
    rtos_mutex_lock(bus_tx_buf_list.mutex, -1);
    if (node->buf_len == 0)
    {
        DBG_XHCI_ERR("null bus buf free, buf=%p\n", node->buf);
    }
    else if (node->buf_len > BUS_TX_BUF_SIZE)
    {
        uint8_t *buf_raw = node->buf - node->pad_len;
        rtos_free(buf_raw);
    }
    node->buf = NULL;
    node->buf_len = 0;
    node->pad_len = 0;
    co_list_push_back(&bus_tx_buf_list.list, &node->hdr);
    rtos_mutex_unlock(bus_tx_buf_list.mutex);
    rtos_semaphore_signal(bus_tx_buf_list.ava_node_sema, 0);
}

void aicwf_bustx_buf_deinit(void)
{
    rtos_mutex_delete(bus_tx_buf_list.mutex);
    rtos_semaphore_delete(bus_tx_buf_list.ava_node_sema);
}
#endif

void aicwf_busrx_thread(void *bus_data)
{
    struct aicwf_bus *bus_if = (struct aicwf_bus *)bus_data;
    bus_if->ops->rxtask(bus_data);
    bus_if->busrx_thread = NULL;
}

void aicwf_bustx_thread(void *bus_data)
{
    struct aicwf_bus *bus_if = (struct aicwf_bus *)bus_data;
    bus_if->ops->txtask(bus_data);
    bus_if->bustx_thread = NULL;
}

int aicwf_bus_init(struct aicwf_bus *bus_if)
{
    int ret = 0;

    aicwf_busrx_buf_init();

    bus_if->cmd_buf = rtos_calloc(1, CMD_BUF_MAX);
    if (!bus_if->cmd_buf)
    {
        ret = -1;//-ENOMEM;
        DBG_XHCI_ERR("proto_attach failed\n");
        goto fail;
    }

    ret = rtos_semaphore_create(&bus_if->busrx_trgg, "busrx_trgg", 1, 0);
    if (ret)
    {
        DBG_XHCI_ERR("busrx_trgg create fail: %d\n", ret);
        goto fail;
    }

    ret = rtos_task_create(aicwf_busrx_thread, "busrx_task", XHCI_BUSRX_TASK,
                           xhci_busrx_stack_size, (void *)bus_if, xhci_busrx_priority,
                           &bus_if->busrx_thread);
    if (ret)
    {
        DBG_XHCI_ERR("busrx task create fail,%d\n", ret);
        goto fail;
    }

#ifdef CONFIG_DATA_TXRX
    aicwf_bustx_buf_init();
#endif

    ret = rtos_semaphore_create(&bus_if->bustx_trgg, "bustx_trgg", 1, 0);
    if (ret)
    {
        DBG_XHCI_ERR("bustx_trgg create fail: %d\n", ret);
        goto fail;
    }

    ret = rtos_task_create(aicwf_bustx_thread, "bustx_task", XHCI_BUSTX_TASK,
                           xhci_bustx_stack_size, (void *)bus_if, xhci_bustx_priority,
                           &bus_if->bustx_thread);
    if (ret)
    {
        DBG_XHCI_ERR("busrx task create fail,%d\n", ret);
        goto fail;
    }

#if 0
    init_completion(&bus_if->bustx_trgg);
    init_completion(&bus_if->busrx_trgg);
#ifdef AICWF_SDIO_SUPPORT
    bus_if->bustx_thread = kthread_run(sdio_bustx_thread, (void *)bus_if, "aicwf_bustx_thread");
    bus_if->busrx_thread = kthread_run(sdio_busrx_thread, (void *)bus_if->bus_priv.sdio->rx_priv, "aicwf_busrx_thread");
#endif
#ifdef AICWF_USB_SUPPORT
    bus_if->bustx_thread = kthread_run(usb_bustx_thread, (void *)bus_if, "aicwf_bustx_thread");
    bus_if->busrx_thread = kthread_run(usb_busrx_thread, (void *)bus_if->bus_priv.usb->rx_priv, "aicwf_busrx_thread");
#endif

    if (IS_ERR(bus_if->bustx_thread))
    {
        bus_if->bustx_thread  = NULL;
        DBG_XHCI_ERR("aicwf_bustx_thread run fail\n");
        goto fail;
    }

    if (IS_ERR(bus_if->busrx_thread))
    {
        bus_if->busrx_thread  = NULL;
        DBG_XHCI_ERR("aicwf_bustx_thread run fail\n");
        goto fail;
    }
#endif

    return ret;
fail:
    aicwf_bus_deinit(bus_if);

    return ret;
}

void aicwf_bus_deinit(struct aicwf_bus *bus_if)
{

    DBG_XHCI_INF("%s\n", __func__);

    aicwf_bus_stop(bus_if);

    if (bus_if->cmd_buf)
    {
        rtos_free(bus_if->cmd_buf);
        bus_if->cmd_buf = NULL;
    }

    if (bus_if->busrx_thread)
    {
        rtos_task_delete(bus_if->busrx_thread);
        bus_if->busrx_thread = NULL;
    }

    if (bus_if->busrx_trgg)
    {
        rtos_semaphore_delete(bus_if->busrx_trgg);
        bus_if->busrx_trgg = NULL;
    }

    if (bus_if->bustx_thread)
    {
        rtos_task_delete(bus_if->bustx_thread);
        bus_if->bustx_thread = NULL;
    }

    if (bus_if->bustx_trgg)
    {
        rtos_semaphore_delete(bus_if->bustx_trgg);
        bus_if->bustx_trgg = NULL;
    }

    aicwf_busrx_buf_deinit();
    aicwf_bustx_buf_deinit();

    DBG_XHCI_INF("exit %s\n", __func__);
}

#ifdef CONFIG_DATA_TXRX
int aicwf_frame_tx(struct aicwf_bus *bus_if, uint8_t *pkt, uint32_t len)
{
    int ret;
    if (bus_if->state == BUS_DOWN_ST)
    {
        DBG_XHCI_ERR("bus down\r\n");
        return -1;
    }
    ret = aicwf_bus_txdata(bus_if, pkt, len);
    return ret;
}
#endif

#if 0
void aicwf_msg_tx(void *dev,  u8 *msg, uint len)
{
#ifdef AICWF_SDIO_SUPPORT
    struct aic_sdio_dev *sdiodev = (struct aic_sdio_dev *)dev;
    aicwf_bus_txmsg(sdiodev->bus_if, msg, len);
#else
    struct aic_usb_dev *usbdev = (struct aic_usb_dev *)dev;

    if (!usbdev->state)
    {
        DBG_XHCI_ERR("down\n");
        return;
    }
    aicwf_bus_txmsg(usbdev->bus_if, msg, len);
#endif
}
#endif


int aicwf_bus_msg_tx(struct aicwf_bus *bus_if, struct lmac_msg *msg, uint16_t len)
{
    uint8_t ret;
    uint16_t index = 0;
    uint8_t *buffer;

    //DBG_XHCI_INF("%s, enter\n", __func__);

    buffer = bus_if->cmd_buf;

    rtos_memset(buffer, 0, CMD_BUF_MAX);
    buffer[0] = (len + 4) & 0x00ff;
    buffer[1] = ((len + 4) >> 8) & 0x0f;
    buffer[2] = 0x11;
    //if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8801 || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
    //    g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
    buffer[3] = 0x0;
    //else if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
    //    buffer[3] = crc8_ponl_107(&buffer[0], 3); // crc8
    index += 4;
    //there is a dummy word
    index += 4;

    //make sure little endian
    put_u16(&buffer[index], msg->id);
    index += 2;
    put_u16(&buffer[index], msg->dest_id);
    index += 2;
    put_u16(&buffer[index], msg->src_id);
    index += 2;
    put_u16(&buffer[index], msg->param_len);
    index += 2;
    rtos_memcpy(&buffer[index], (u8 *)msg->param, msg->param_len);

    ret = aicwf_bus_txmsg(bus_if, buffer, len + 8);
    //DBG_XHCI_INF("%s, ret %d\n", __func__, ret);

    return ret;
}

struct aicwf_tx_priv *aicwf_tx_init(void *arg)
{
    int ret;
    struct aicwf_tx_priv *tx_priv;
    DBG_XHCI_INF("%s \r\n",  __func__);

    tx_priv = rtos_calloc(1, sizeof(struct aicwf_tx_priv));
    if (!tx_priv)
        return NULL;

#if 0
#ifdef AICWF_SDIO_SUPPORT
    tx_priv->sdiodev = (struct aic_sdio_dev *)arg;
#else
    tx_priv->usbdev = (struct aic_usb_dev *)arg;
#endif
#endif

    tx_priv->cmd_txstate = false;
    tx_priv->cmd_tx_succ = false;
    tx_priv->cmd_buf = NULL;
    tx_priv->cmd_len = 0;
    ret = rtos_mutex_create(&tx_priv->cmd_txsema, "bustx cmd mutex");
    if (ret)
    {
        DBG_XHCI_ERR("bustx cmd mutex create fail: %d\r\n", ret);
        rtos_free(tx_priv);
        return NULL;
    }

#ifdef CONFIG_DATA_TXRX
    tx_priv->fw_avail_bufcnt = 0;
    rtos_atomic_set(&tx_priv->tx_pktcnt, 0);
    tx_priv->aggr_count = 0;
    tx_priv->len = 0;
    tx_priv->aggr_buf = rtos_calloc(1, MAX_AGGR_TXPKT_LEN);
    if (!tx_priv->aggr_buf)
    {
        DBG_XHCI_ERR("Alloc bus->txdata_buf failed!\r\n");
        rtos_mutex_delete(tx_priv->cmd_txsema);
        rtos_free(tx_priv);
        return NULL;
    }
    tx_priv->head = tx_priv->aggr_buf;
    tx_priv->tail = tx_priv->aggr_buf;
#endif

    return tx_priv;
}

void aicwf_tx_deinit(struct aicwf_tx_priv *tx_priv)
{
    if (tx_priv)
    {
        if (tx_priv->cmd_txsema)
        {
            rtos_mutex_delete(tx_priv->cmd_txsema);
            tx_priv->cmd_txsema = NULL;
        }
#ifdef CONFIG_DATA_TXRX
        if (tx_priv->aggr_buf)
        {
            rtos_free(tx_priv->aggr_buf);
            tx_priv->aggr_buf = NULL;
        }
#endif
        rtos_free(tx_priv);
    }
}

#if 0
#ifdef AICWF_SDIO_SUPPORT
static bool aicwf_next_ptk(struct sk_buff *skb)
{
    u8 *data;
    u16 aggr_len = 0;

    if (skb->data == NULL || skb->len == 0)
    {
        return false;
    }
    data = skb->data;
    aggr_len = (*skb->data | (*(skb->data + 1) << 8));
    if (aggr_len == 0)
    {
        return false;
    }

    return true;
}
#endif


int aicwf_process_rxframes(struct aicwf_rx_priv *rx_priv)
{
#ifdef AICWF_SDIO_SUPPORT
    int ret = 0;
    unsigned long flags = 0;
    struct sk_buff *skb = NULL;
    u16 pkt_len = 0;
    //struct sk_buff *skb_inblock = NULL;
    u16 aggr_len = 0, adjust_len = 0, total_len = 0;
    u8 *data = NULL;
    u8_l *msg = NULL;
    struct sk_buff  *rx_skb = NULL;

    while (1)
    {
        spin_lock_irqsave(&rx_priv->rxqlock, flags);
        if (aicwf_is_framequeue_empty(&rx_priv->rxq))
        {
            spin_unlock_irqrestore(&rx_priv->rxqlock, flags);
            break;
        }
        skb = aicwf_frame_dequeue(&rx_priv->rxq);
        spin_unlock_irqrestore(&rx_priv->rxqlock, flags);
        if (skb == NULL)
        {
            DBG_XHCI_ERR("skb_error\r\n");
            break;
        }
        total_len = skb->len;
        while (aicwf_next_ptk(skb))
        {
            data = skb->data;
            pkt_len = (*skb->data | (*(skb->data + 1) << 8));

            if (total_len < pkt_len)
            {
                break;
            }

            if ((skb->data[2] & SDIO_TYPE_CFG) != SDIO_TYPE_CFG)  // type : data
            {
#if 0
                struct sk_buff  *rx_skb = dev_alloc_skb(skb->len + 2);
                skb_reserve(rx_skb, 2); /* align IP on 16B boundary */
                memcpy(skb_put(rx_skb, skb->len), (skb->data + 4), (skb->len - 4));

                /* Write metadata, and then pass to the receive level */
                rx_skb->dev = vnet_dev;
                rx_skb->protocol = eth_type_trans(rx_skb, vnet_dev);

                netif_rx(rx_skb);
#else
                aggr_len = (pkt_len + 4);

                if (aggr_len & (RX_ALIGNMENT - 1))
                    adjust_len = roundup(aggr_len, RX_ALIGNMENT);
                else
                    adjust_len = aggr_len;

                rx_skb = dev_alloc_skb(aggr_len);
                if (NULL ==  rx_skb)
                {
                    DBG_XHCI_ERR("no more space! skip\n");
                    continue;
                }
                memcpy(skb_put(rx_skb, aggr_len),  skb->data, aggr_len);
                skb_pull(rx_skb, 4);

                // 802.11 packet
                rwnx_rx_handle_data(rx_priv, rx_skb);
                skb_pull(skb, adjust_len); // used by aggr
                total_len -= adjust_len;

#endif
            }
            else   //  type : config
            {
                aggr_len = pkt_len;

                if (aggr_len & (RX_ALIGNMENT - 1))
                    adjust_len = roundup(aggr_len, RX_ALIGNMENT);
                else
                    adjust_len = aggr_len;

                msg = kmalloc(aggr_len + 4, GFP_KERNEL);
                if (msg == NULL)
                {
                    DBG_XHCI_ERR("no more space for msg!\n");
                    aicwf_dev_skb_free(skb);
                    return -EBADE;
                }

#ifdef CONFIG_VNET_MODE
                memcpy(msg, data, aggr_len + 4);
                if ((*(msg + 2) & 0x7f) == SDIO_TYPE_CFG_CMD_RSP)
                    rwnx_rx_handle_msg(rx_priv->sdiodev->rwnx_hw, (struct ipc_e2a_msg *)(msg + 4));
#endif

                skb_pull(skb, adjust_len + 4);
                kfree(msg);
            }
        }

        dev_kfree_skb(skb);
        atomic_dec(&rx_priv->rx_cnt);
    }

#if defined(CONFIG_SDIO_PWRCTRL)
    aicwf_sdio_pwr_stctl(rx_priv->sdiodev, SDIO_ACTIVE_ST);
#endif

    return ret;
#else //AICWF_USB_SUPPORT
    int ret = 0;
    unsigned long flags = 0;
    struct sk_buff *skb = NULL; /* Packet for event or data frames */
    u16 pkt_len = 0;
    u16 aggr_len = 0, adjust_len = 0;
    u8 *data = NULL;
    u8_l *msg = NULL;

    while (1)
    {
        spin_lock_irqsave(&rx_priv->rxqlock, flags);
        if (aicwf_is_framequeue_empty(&rx_priv->rxq))
        {
            usb_info("no more rxdata\n");
            spin_unlock_irqrestore(&rx_priv->rxqlock, flags);
            break;
        }
        skb = aicwf_frame_dequeue(&rx_priv->rxq);
        spin_unlock_irqrestore(&rx_priv->rxqlock, flags);
        if (skb == NULL)
        {
            DBG_XHCI_ERR("skb_error\r\n");
            break;
        }
        data = skb->data;
        pkt_len = (*skb->data | (*(skb->data + 1) << 8));
        //printk("p:%d, s:%d , %x\n", pkt_len, skb->len, data[2]);
        if (pkt_len > 1700)
        {
            dev_kfree_skb(skb);
            atomic_dec(&rx_priv->rx_cnt);
            continue;
        }

        if ((skb->data[2] & USB_TYPE_CFG) != USB_TYPE_CFG)  // type : data
        {
#if 0
            struct sk_buff  *rx_skb = dev_alloc_skb(skb->len + 2);
            skb_reserve(rx_skb, 2); /* align IP on 16B boundary */
            memcpy(skb_put(rx_skb, skb->len), (skb->data + 4), (skb->len - 4));

            /* Write metadata, and then pass to the receive level */
            rx_skb->dev = vnet_dev;
            rx_skb->protocol = eth_type_trans(rx_skb, vnet_dev);

            netif_rx(rx_skb);
#else
            struct sk_buff  *rx_skb = dev_alloc_skb(skb->len);
            if (NULL ==  rx_skb)
            {
                DBG_XHCI_ERR("no more space! skip\n");
                continue;
            }
            memcpy(skb_put(rx_skb, skb->len),  skb->data, skb->len);
            skb_pull(rx_skb, 4);

            // 802.11 packet
            rwnx_rx_handle_data(rx_priv, rx_skb);
#endif
        }
        else   //  type : config
        {
            aggr_len = pkt_len;
            if (aggr_len & (RX_ALIGNMENT - 1))
                adjust_len = roundup(aggr_len, RX_ALIGNMENT);
            else
                adjust_len = aggr_len;

            msg = kmalloc(aggr_len + 4, GFP_KERNEL);
            if (msg == NULL)
            {
                DBG_XHCI_ERR("no more space for msg!\n");
                aicwf_dev_skb_free(skb);
                return -EBADE;
            }

#ifdef CONFIG_VNET_MODE
            memcpy(msg, data, aggr_len + 4);
            if ((*(msg + 2) & 0x7f) == USB_TYPE_CFG_CMD_RSP)
                rwnx_rx_handle_msg(rx_priv->usbdev, (struct ipc_e2a_msg *)(msg + 4));
#endif

            skb_pull(skb, adjust_len + 4);
            kfree(msg);
        }

        dev_kfree_skb(skb);
        atomic_dec(&rx_priv->rx_cnt);
    }

    return ret;
#endif //AICWF_SDIO_SUPPORT
    return 0;

}
#endif

#ifdef AICWF_RX_REORDER
static struct recv_msdu *aicwf_rxframe_queue_init(struct list_head *q, int qsize)
{
    int i;
    struct recv_msdu *req, *reqs;

    reqs = vmalloc(qsize * sizeof(struct recv_msdu));
    if (reqs == NULL)
        return NULL;

    req = reqs;
    for (i = 0; i < qsize; i++)
    {
        INIT_LIST_HEAD(&req->rxframe_list);
        list_add(&req->rxframe_list, q);
        req->len = 0;
        req++;
    }

    return reqs;
}
static void aicwf_recvframe_queue_deinit(struct list_head *q)
{
    struct recv_msdu *req, *next;

    list_for_each_entry_safe(req, next, q, rxframe_list)
    {
        list_del_init(&req->rxframe_list);
    }
}
#endif

struct aicwf_rx_priv *aicwf_rx_init(void *arg)
{
    struct aicwf_rx_priv *rx_priv;
    rx_priv = rtos_calloc(1, sizeof(struct aicwf_rx_priv));
    if (!rx_priv)
        return NULL;

    aicwf_frame_queue_init(&rx_priv->rxq);
    //spin_lock_init(&rx_priv->rxqlock);
    //atomic_set(&rx_priv->rx_cnt, 0);

#ifdef AICWF_RX_REORDER
    INIT_LIST_HEAD(&rx_priv->rxframes_freequeue);
    spin_lock_init(&rx_priv->freeq_lock);
    rx_priv->recv_frames = aicwf_rxframe_queue_init(&rx_priv->rxframes_freequeue, MAX_REORD_RXFRAME);
    if (!rx_priv->recv_frames)
    {
        DBG_XHCI_ERR("no enough buffer for free recv frame queue!\n");
        rtos_free(rx_priv);
        return NULL;
    }
    spin_lock_init(&rx_priv->stas_reord_lock);
    INIT_LIST_HEAD(&rx_priv->stas_reord_list);
#endif

    return rx_priv;
}

void aicwf_rx_deinit(struct aicwf_rx_priv *rx_priv)
{
#if 0//def AICWF_RX_REORDER
    struct reord_ctrl_info *reord_info, *tmp;

    DBG_XHCI_INF("%s\n", __func__);

    spin_lock_bh(&rx_priv->stas_reord_lock);
    list_for_each_entry_safe(reord_info, tmp,
                             &rx_priv->stas_reord_list, list)
    {
        reord_deinit_sta(rx_priv, reord_info);
    }
    spin_unlock_bh(&rx_priv->stas_reord_lock);
#endif

#ifdef AICWF_SDIO_SUPPORT
    if (rx_priv->sdiodev->bus_if->busrx_thread)
    {
        complete(&rx_priv->sdiodev->bus_if->busrx_trgg);
        kthread_stop(rx_priv->sdiodev->bus_if->busrx_thread);
        rx_priv->sdiodev->bus_if->busrx_thread = NULL;
    }
#endif
#ifdef AICWF_USB_SUPPORT
    if (rx_priv->usbdev->bus_if->busrx_thread)
    {
        complete(&rx_priv->usbdev->bus_if->busrx_trgg);
        kthread_stop(rx_priv->usbdev->bus_if->busrx_thread);
        rx_priv->usbdev->bus_if->busrx_thread = NULL;
    }
#endif

    aicwf_frame_queue_flush(&rx_priv->rxq);
    aicwf_frame_queue_deinit(&rx_priv->rxq);
#ifdef AICWF_RX_REORDER
    aicwf_recvframe_queue_deinit(&rx_priv->rxframes_freequeue);
    if (rx_priv->recv_frames)
        vfree(rx_priv->recv_frames);
#endif

    rtos_free(rx_priv);
    DBG_XHCI_INF("exit %s\n", __func__);
}

#if 0
bool aicwf_frame_enqueue(struct device *dev, struct frame_queue *q, struct sk_buff *pkt)
{
    return aicwf_frame_enq(dev, q, pkt, 0);
}

void aicwf_dev_skb_free(struct sk_buff *skb)
{
    if (!skb)
        return;

    dev_kfree_skb_any(skb);
}

static struct sk_buff *aicwf_frame_queue_penq(struct frame_queue *pq, int prio, struct sk_buff *p)
{
    struct sk_buff_head *q;

    if (pq->queuelist[prio].qlen >= pq->qmax)
        return NULL;

    q = &pq->queuelist[prio];
    __skb_queue_tail(q, p);
    pq->qcnt++;
    if (pq->hi_prio < prio)
        pq->hi_prio = (u16)prio;

    return p;
}
#endif

void aicwf_frame_queue_init(struct aicwf_buf_list_s *buf_list)
{
    int ret;
    ret = rtos_mutex_create(&buf_list->mutex, "buf_list.mutex");
    if (ret)
    {
        DBG_XHCI_ERR("buf_list mutex create fail: %d\n", ret);
        return;
    }
    co_list_init(&buf_list->list);
}

void aicwf_frame_queue_deinit(struct aicwf_buf_list_s *buf_list)
{
    rtos_mutex_delete(buf_list->mutex);
}

void aicwf_frame_enqueue(struct aicwf_buf_list_s *buf_list, struct aicwf_buf_node_s *node)
{
    rtos_mutex_lock(buf_list->mutex, -1);
    co_list_push_back(&buf_list->list, &node->hdr);
    rtos_mutex_unlock(buf_list->mutex);
}

struct aicwf_buf_node_s *aicwf_frame_dequeue(struct aicwf_buf_list_s *buf_list)
{
    struct aicwf_buf_node_s *node;
    rtos_mutex_lock(buf_list->mutex, -1);
    node = (struct aicwf_buf_node_s *)co_list_pop_front(&buf_list->list);
    rtos_mutex_unlock(buf_list->mutex);
    return node;
}

void aicwf_frame_queue_flush(struct aicwf_buf_list_s *buf_list)
{
    struct aicwf_buf_node_s *node = aicwf_frame_dequeue(buf_list);

    while (node)
    {
        aicwf_busrx_buf_free(node);
        node = aicwf_frame_dequeue(buf_list);
    }
}

bool aicwf_is_framequeue_empty(struct aicwf_buf_list_s *buf_list)
{
    int ret = false;
    if (co_list_is_empty(&buf_list->list))
    {
        ret = true;
    }
    return ret;
}

#if CONFIG_NOT_DEFINED
struct sk_buff *aicwf_frame_queue_peek_tail(struct frame_queue *pq, int *prio_out)
{
    int prio;

    if (pq->qcnt == 0)
        return NULL;

    for (prio = 0; prio < pq->hi_prio; prio++)
        if (!skb_queue_empty(&pq->queuelist[prio]))
            break;

    if (prio_out)
        *prio_out = prio;

    return skb_peek_tail(&pq->queuelist[prio]);
}

struct sk_buff *aicwf_frame_dequeue(struct frame_queue *pq)
{
    struct sk_buff_head *q;
    struct sk_buff *p;
    int prio;

    if (pq->qcnt == 0)
        return NULL;

    while ((prio = pq->hi_prio) > 0 && skb_queue_empty(&pq->queuelist[prio]))
        pq->hi_prio--;

    q = &pq->queuelist[prio];
    p = __skb_dequeue(q);
    if (p == NULL)
        return NULL;

    pq->qcnt--;

    return p;
}

static struct sk_buff *aicwf_skb_dequeue_tail(struct frame_queue *pq, int prio)
{
    struct sk_buff_head *q = &pq->queuelist[prio];
    struct sk_buff *p = skb_dequeue_tail(q);

    if (!p)
        return NULL;

    pq->qcnt--;
    return p;
}

bool aicwf_frame_enq(struct device *dev, struct frame_queue *q, struct sk_buff *pkt, int prio)
{
#if 1
    struct sk_buff *p = NULL;
    int prio_modified = -1;

    if (q->queuelist[prio].qlen < q->qmax && q->qcnt < q->qmax)
    {
        aicwf_frame_queue_penq(q, prio, pkt);
        return true;
    }
    if (q->queuelist[prio].qlen >= q->qmax)
    {
        prio_modified = prio;
    }
    else if (q->qcnt >= q->qmax)
    {
        p = aicwf_frame_queue_peek_tail(q, &prio_modified);
        if (prio_modified > prio)
            return false;
    }

    if (prio_modified >= 0)
    {
        if (prio_modified == prio)
            return false;

        p = aicwf_skb_dequeue_tail(q, prio_modified);
        aicwf_dev_skb_free(p);

        p = aicwf_frame_queue_penq(q, prio_modified, pkt);
        if (p == NULL)
            DBG_XHCI_ERR("failed\n");
    }

    return p != NULL;
#else
    if (q->queuelist[prio].qlen < q->qmax && q->qcnt < q->qmax)
    {
        aicwf_frame_queue_penq(q, prio, pkt);
        return true;
    }
    else
        return false;
#endif
}

#endif
