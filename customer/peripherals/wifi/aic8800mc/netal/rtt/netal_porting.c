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

#include "netal_porting.h"
#include "osal_debug.h"
#include <string.h>

#include "rtconfig.h"
#include "rtthread.h"
#include <netif/ethernetif.h>
#include "netdev.h"
#include <lwip/netif.h>
#include <lwip/dhcp.h>
#ifdef RT_USING_WIFI
    #include "aic8800mc_wlan_port.h"
#endif

#define MAX_ADDR_LEN    6

struct aicwf_net_device
{
    /* inherit from ethernet device */
    struct eth_device parent;

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN]; /* hw address   */

    struct rt_mutex lock;
};

#define aicwf_net_lock(dev)      rt_mutex_take(&((struct aicwf_net_device*)dev)->lock, RT_WAITING_FOREVER);
#define aicwf_net_unlock(dev)    rt_mutex_release(&((struct aicwf_net_device*)dev)->lock);

static struct aicwf_net_device aicwf_net_dev;

/* initialize the interface */
static rt_err_t aicwf_net_init(rt_device_t dev)
{
    struct aicwf_net_device *aicwf_net = (struct aicwf_net_device *)dev;

    aicwf_net_lock(dev);

    aicwf_net_unlock(dev);
    return RT_EOK;
}

/* control the interface */
static rt_err_t aicwf_net_control(rt_device_t dev, int cmd, void *args)
{
    struct aicwf_net_device *aicwf_net = (struct aicwf_net_device *)dev;
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args) rt_memcpy(args, aicwf_net->dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

/* Open the ethernet interface */
static rt_err_t aicwf_net_open(rt_device_t dev, uint16_t oflag)
{
    return RT_EOK;
}

/* Close the interface */
static rt_err_t aicwf_net_close(rt_device_t dev)
{
    return RT_EOK;
}

/* Read */
static rt_size_t aicwf_net_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return RT_EOK;
}

/* Write */
static rt_size_t aicwf_net_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

/* ethernet device interface */
/* Transmit packet. */
static rt_err_t aicwf_net_tx(rt_device_t dev, struct pbuf *p)
{
    rt_err_t ret = RT_EOK;

    netal_eth_env_t *net_env = ((struct eth_device *)dev)->parent.user_data;
    //struct aicwf_net_device *aicwf_net = (struct aicwf_net_device *)dev;
    //netal_eth_env_t *net_env = (netal_eth_env_t *)dev->user_data;

    if (net_env == RT_NULL)
    {
        return -RT_ERROR;
    }

    aicwf_net_lock(dev);

#if 0//def RT_WLAN_PROT_LWIP_PBUF_FORCE
    {
        rt_wlan_prot_transfer_dev(wlan, p, p->tot_len);
        return RT_EOK;
    }
#else
    do
    {
        rt_uint8_t *frame;

        /* sending data directly */
        if (p->len == p->tot_len)
        {
            frame = (rt_uint8_t *)p->payload;
            net_env->ops.eth_send(frame, p->tot_len);
            DBG_NETAL_VRB("F:%s L:%d run len:%d", __FUNCTION__, __LINE__, p->tot_len);
            break;
        }
        frame = rtos_malloc(p->tot_len);
        if (frame == RT_NULL)
        {
            DBG_NETAL_ERR("F:%s L:%d malloc out_buf fail\n", __FUNCTION__, __LINE__);
            ret = -RT_ENOMEM;
            break;
        }
        /*copy pbuf -> data dat*/
        pbuf_copy_partial(p, frame, p->tot_len, 0);
        /* send data */
        net_env->ops.eth_send(frame, p->tot_len);
        DBG_NETAL_VRB("F:%s L:%d run len:%d", __FUNCTION__, __LINE__, p->tot_len);
        rtos_free(frame);
    }
    while (0);
#endif

    aicwf_net_unlock(dev);

    return ret;
}

#if 0
/* recv packet. */
static struct pbuf *aicwf_net_rx(rt_device_t dev)
{
    struct aicwf_net_device *aicwf_net = (struct aicwf_net_device *)dev;
    struct pbuf *p = RT_NULL;

    uint8_t eir, eir_clr;
    uint32_t pk_counter;
    rt_uint32_t level;
    rt_uint32_t len;
    rt_uint16_t rxstat;

    aicwf_net_lock(dev);

    aicwf_net_unlock(dev);

    return p;
}
#endif

static int netif_lwip_recv(netal_eth_env_t *net_env, void *buff, int len)
{
#ifdef RT_USING_WIFI
    aic8800mc_wlan_rx_data(buff, len);
    return RT_EOK;
#else
    struct aicwf_net_device *p_wf_net = net_env->eth_dev;
    struct eth_device *eth_dev = &p_wf_net->parent;
    struct pbuf *p = RT_NULL;

    DBG_NETAL_VRB("F:%s L:%d run\n", __FUNCTION__, __LINE__);

    if (eth_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    {
        int count = 0;
        err_t ret;

        while (p == RT_NULL)
        {
            p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
            if (p != RT_NULL)
                break;

            p = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
            if (p != RT_NULL)
                break;

            DBG_NETAL_VRB("F:%s L:%d wait for pbuf_alloc!\n", __FUNCTION__, __LINE__);
            rt_thread_delay(1);
            count++;

            if (count >= 10)
            {
                DBG_NETAL_ERR("F:%s L:%d pbuf allocate fail!!!\n", __FUNCTION__, __LINE__);
                return -RT_ENOMEM;
            }
        }
        pbuf_take(p, buff, len);
        ret = eth_dev->netif->input(p, eth_dev->netif);
        if (ret != ERR_OK)
        {
            DBG_NETAL_ERR("F:%s L:%d IP input error, ret=%d\n", __FUNCTION__, __LINE__, ret);
            pbuf_free(p);
            p = RT_NULL;
        }
        DBG_NETAL_VRB("F:%s L:%d netif iput success! len:%d\n", __FUNCTION__, __LINE__, len);
        return RT_EOK;
    }
#endif
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops aicwf_net_ops =
{
    aicwf_net_init,
    aicwf_net_open,
    aicwf_net_close,
    aicwf_net_read,
    aicwf_net_write,
    aicwf_net_control
};
#endif

void netif_sync_hwaddr(uint8_t *mac_addr)
{
#ifdef RT_USING_NETDEV
#ifdef RT_USING_WIFI
    const char *netdev_name = "w0";
#else
    const char *netdev_name = "e0";
#endif
    struct netdev *netdev = netdev_get_by_name(netdev_name);
    if (netdev)
    {
        rt_memcpy(netdev->hwaddr, mac_addr, MAX_ADDR_LEN);
        DBG_NETAL_INF("sync netdev hwaddr\r\n");
    }
#endif

#ifdef RT_USING_WIFI
    const char *netif_name = "w00";
#else
    const char *netif_name = "e00";
#endif
    struct netif *netif = netif_find(netif_name);
    if (netif)
    {
        rt_memcpy(netif->hwaddr, mac_addr, MAX_ADDR_LEN);
        DBG_NETAL_INF("sync netif hwaddr\r\n");
    }
}

extern void lwip_sys_init(void);

static rt_bool_t init_ok = RT_FALSE;

void netif_lwip_reset(void)
{
    init_ok = RT_FALSE;
}

static int netif_lwip_init(netal_eth_env_t *net_env)
{
    if (init_ok)
    {
        rt_kprintf("netif_lwip_init already init.\n");
        rt_memcpy(&aicwf_net_dev.dev_addr[0], &net_env->mac_addr[0], MAX_ADDR_LEN);
        netif_sync_hwaddr(&net_env->mac_addr[0]);
        return 0;
    }

    if (net_env == NULL)
    {
        DBG_NETAL_ERR("invalid net_env\r\n");
        return -1;
    }

    rt_memcpy(&aicwf_net_dev.dev_addr[0], &net_env->mac_addr[0], MAX_ADDR_LEN);

#ifndef RT_USING_WIFI
    memset(&aicwf_net_dev, 0, sizeof(aicwf_net_dev));

    rt_memcpy(&aicwf_net_dev.dev_addr[0], &net_env->mac_addr[0], MAX_ADDR_LEN);

    aicwf_net_dev.parent.parent.type    = RT_Device_Class_NetIf;
#ifdef RT_USING_DEVICE_OPS
    aicwf_net_dev.parent.parent.ops     = &aicwf_net_ops;
#else
    aicwf_net_dev.parent.parent.init    = aicwf_net_init;
    aicwf_net_dev.parent.parent.open    = aicwf_net_open;
    aicwf_net_dev.parent.parent.close   = aicwf_net_close;
    aicwf_net_dev.parent.parent.read    = aicwf_net_read;
    aicwf_net_dev.parent.parent.write   = aicwf_net_write;
    aicwf_net_dev.parent.parent.control = aicwf_net_control;
#endif
    aicwf_net_dev.parent.parent.user_data = net_env;

    aicwf_net_dev.parent.eth_rx  = NULL;
    aicwf_net_dev.parent.eth_tx  = aicwf_net_tx;

    rt_mutex_init(&aicwf_net_dev.lock, "aicwf_net", RT_IPC_FLAG_FIFO);

    lwip_sys_init();

    eth_device_init(&(aicwf_net_dev.parent), "e0");

    net_env->eth_dev = (void *)&aicwf_net_dev;
#else
    net_env->eth_dev = (void *)&aicwf_net_dev;
    netif_sync_hwaddr(&net_env->mac_addr[0]);
#endif

    init_ok = RT_TRUE;

    return 0;
}

static int netif_lwip_linkchange(netal_eth_env_t *net_env, int link_up)
{
    int ret = 0;
#ifdef RT_USING_WIFI
    if (link_up)
    {
        aic8800mc_wlan_report_connect();
    }
    else
    {
        aic8800mc_wlan_report_disconnect();
    }
#else
    struct aicwf_net_device *p_wf_net = net_env->eth_dev;
    struct eth_device *eth_dev = &p_wf_net->parent;
    rt_bool_t up = (link_up) ? RT_TRUE : RT_FALSE;
    rt_err_t err = eth_device_linkchange(eth_dev, up);
    if (err)
    {
        DBG_NETAL_ERR("lwip linkchange err: %d\r\n", err);
        ret = -1;
    }
#endif
    return ret;
}

static int netif_lwip_addrinfoset(netal_eth_env_t *net_env, uint32_t *ip, uint32_t *nm, uint32_t *gw)
{
    int ret = 0;
#ifdef RT_USING_WIFI
#else
    struct aicwf_net_device *p_wf_net = net_env->eth_dev;
    struct eth_device *eth_dev = &p_wf_net->parent;
    struct netif *net_if = eth_dev->netif;
    ip4_addr_t addr_ip, addr_nm, addr_gw;
    if (ip && nm && gw)
    {
        ip4_addr_set_u32(&addr_ip, *ip);
        ip4_addr_set_u32(&addr_nm, *nm);
        ip4_addr_set_u32(&addr_gw, *gw);
        netif_set_addr(net_if, &addr_ip, &addr_nm, &addr_gw);
    }
    else
    {
        if (ip)
        {
            ip4_addr_set_u32(&addr_ip, *ip);
            netif_set_ipaddr(net_if, &addr_ip);
        }
        if (nm)
        {
            ip4_addr_set_u32(&addr_nm, *nm);
            netif_set_netmask(net_if, &addr_nm);
        }
        if (gw)
        {
            ip4_addr_set_u32(&addr_gw, *gw);
            netif_set_gw(net_if, &addr_gw);
        }
    }
#endif
    return ret;
}

const netal_porting_ops_t netal_porting_ops =
{
    .eth_init = netif_lwip_init,
    .eth_recv = netif_lwip_recv,
    .eth_linkchange = netif_lwip_linkchange,
    .eth_addrinfoset = netif_lwip_addrinfoset,
};

