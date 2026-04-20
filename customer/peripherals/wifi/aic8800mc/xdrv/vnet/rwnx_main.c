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

// #include <rtthread.h>
#include "rwnx_main.h"
#include "rwnx_platform.h"
//#include "reg_access.h"
#include "rwnx_defs.h"
#include "rwnx_msg_tx.h"
#include "aicwf_sdio.h"
#include "osal_debug.h"
#include "osal_service.h"
#include "netal_service.h"
// #include <wlan_mgnt.h>
// #include <wlan_prot.h>
// #include "aic8800mc_wlan_port.h"
#ifdef RT_USING_LWIP
    #include "lwip/netif.h"
#endif
#ifdef RT_USING_NETDEV
    #include "netdev.h"
#endif

#define VNET_RWNX_HW     g_rwnx_plat->sdiodev->rwnx_hw

aicwf_custom_msg_vnet_t g_custom_msg_vnet;

int vnet_sync_mac_addr(void)
{
    int ret = 0;
    do
    {
        struct fhcustmsg_mac_addr_cfm cfm;
        ret = rwnx_send_fhcustmsg_get_mac_addr_req(VNET_RWNX_HW, &cfm);
        if (ret)
        {
            DBG_VNET_ERR("fhcustmsg_get_mac_addr_req fail: %d\n", ret);
            break;
        }
        DBG_VNET_INF("sync mac: %02x:%02x:%02x:%02x:%02x:%02x\n",
                     cfm.mac_addr[0], cfm.mac_addr[1], cfm.mac_addr[2], cfm.mac_addr[3], cfm.mac_addr[4], cfm.mac_addr[5]);
        rtos_memcpy(&g_custom_msg_vnet.mac_addr[0], &cfm.mac_addr[0], MAC_ADDR_LEN);
    }
    while (0);
    return ret;
}

int vnet_sync_wlan_status(void)
{
    int ret = 0;
    do
    {
        struct fhcustmsg_wlan_status_cfm cfm;
        ret = rwnx_send_fhcustmsg_wlan_status_req(VNET_RWNX_HW, &cfm);
        if (ret)
        {
            DBG_VNET_ERR("fhcustmsg_wlan_status_get fail: %d\n", ret);
            break;
        }
        if (cfm.status == WLAN_DISCONNECT)
        {
            g_custom_msg_vnet.wlan_status = WLAN_DISCONNECT;
            DBG_VNET_INF("wlan_status: disconnect\n");
#if defined(CONFIG_VNET_MODE) && !defined(RT_USING_WIFI)
            netal_eth_protocol_linkchange(0);
#endif
        }
        else if (cfm.status == WLAN_CONNECTED)
        {
            g_custom_msg_vnet.wlan_status = WLAN_CONNECTED;
            g_custom_msg_vnet.ip = cfm.ip;
            g_custom_msg_vnet.mk = cfm.mk;
            g_custom_msg_vnet.gw = cfm.gw;
            DBG_VNET_INF("wlan_status: connected\n");
            DBG_VNET_INF("ssid: \'%s\' (%3d dBm) akm=%d\n", cfm.ussid, cfm.rssi, cfm.akm);
            DBG_VNET_INF("ip: %d.%d.%d.%d,  gw: %d.%d.%d.%d,  mk: %d.%d.%d.%d\n",
                         (cfm.ip >> 0) & 0xff, (cfm.ip >> 8) & 0xff, (cfm.ip >> 16) & 0xff, (cfm.ip >> 24) & 0xff,
                         (cfm.gw >> 0) & 0xff, (cfm.gw >> 8) & 0xff, (cfm.gw >> 16) & 0xff, (cfm.gw >> 24) & 0xff,
                         (cfm.mk >> 0) & 0xff, (cfm.mk >> 8) & 0xff, (cfm.mk >> 16) & 0xff, (cfm.mk >> 24) & 0xff);
            DBG_MACIF_INF("dns1: %d.%d.%d.%d,  dns2: %d.%d.%d.%d\n",
                          (cfm.dns1 >> 0) & 0xff, (cfm.dns1 >> 8) & 0xff, (cfm.dns1 >> 16) & 0xff, (cfm.dns1 >> 24) & 0xff,
                          (cfm.dns2 >> 0) & 0xff, (cfm.dns2 >> 8) & 0xff, (cfm.dns2 >> 16) & 0xff, (cfm.dns2 >> 24) & 0xff);

#if defined(CONFIG_VNET_MODE)
#ifdef RT_USING_NETDEV
#ifdef RT_USING_WIFI
            const char *netdev_name = "w0";
#else
            const char *netdev_name = "e0";
#endif
            struct netdev *netdev = netdev_get_by_name(netdev_name);
            if (netdev && !(netdev->flags & NETDEV_FLAG_LINK_UP))
            {
#ifndef RT_USING_WIFI
                ret = netal_eth_protocol_linkchange(1);
                if (ret)
                {
                    DBG_MACIF_ERR("eth linkchange err: %d\n", ret);
                    return ret;
                }
                ret = netal_eth_protocol_addrinfoset(&cfm.ip, &cfm.mk, &cfm.gw);
                if (ret)
                {
                    DBG_MACIF_ERR("eth addrinfoset err: %d\n", ret);
                    return ret;
                }
#endif

                netdev_set_dns_server(netdev, 0, (ip_addr_t *)&cfm.dns1);
                netdev_set_dns_server(netdev, 1, (ip_addr_t *)&cfm.dns2);
            }
#endif
#endif
        }
        else if (cfm.status == WLAN_SCANNING)
        {
            g_custom_msg_vnet.wlan_status = WLAN_SCANNING;
            DBG_VNET_INF("wlan_status: scanning\n");
        }
        else if (cfm.status == WLAN_CONNECTING)
        {
            g_custom_msg_vnet.wlan_status = WLAN_CONNECTING;
            DBG_VNET_INF("ssid: \'%s\' (%3d dBm) akm=%d\n", cfm.ussid, cfm.rssi, cfm.akm);
            DBG_VNET_INF("wlan_status: connecting\n");
        }
        else
        {
            DBG_VNET_ERR("wlan_status: invalid status\n");
        }
    }
    while (0);
    return ret;
}

// run once task
void vnet_init_task(void *param)
{
    DBG_VNET_INF("vnet init task enter\n");
#if 1
    do
    {
        int ret;
        struct rwnx_hw *rwnx_hw;
        if (g_rwnx_plat == NULL)
        {
            DBG_VNET_ERR("g_rwnx_plat is NULL\n");
            break;
        }
        rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
        // Test dbg msg
        const uint32_t mem_addr = 0x40500000;
        struct dbg_mem_read_cfm rd_mem_addr_cfm;
        ret = rwnx_send_dbg_mem_read_req(rwnx_hw, mem_addr, &rd_mem_addr_cfm);
        if (ret)
        {
            DBG_VNET_ERR("%x rd fail: %d\n", mem_addr, ret);
            break;
        }
        DBG_VNET_INF("rd [%08x]=0x%x\n", mem_addr, rd_mem_addr_cfm.memdata);
    }
    while (0);
#endif

    vnet_sync_mac_addr();

    netal_eth_protocol_init(g_custom_msg_vnet.mac_addr);

    // aic8800mc_wlan_init();
    // rt_wlan_set_mode("wifi_aic8800mc", RT_WLAN_STATION);
    // rt_wlan_prot_attach("wifi_aic8800mc", RT_WLAN_PROT_LWIP);

    vnet_sync_wlan_status();

    DBG_VNET_INF("vnet init task exit\n");
    //rtos_task_delete(NULL); // some rtos need it
}

int rwnx_vnet_init(struct rwnx_plat *rwnx_plat, void **platform_data)
{
    int ret = 0;
    struct rwnx_hw *rwnx_hw;
    rtos_task_handle task_handle;
    const uint8_t default_mac[] = {0x88, 0x00, 0xA6, 0x9C, 0x42, 0x20};

    // alloc structs
    rwnx_hw = rtos_malloc(sizeof(struct rwnx_hw));
    if (rwnx_hw == NULL)
    {
        DBG_VNET_ERR("rwnx_hw alloc failed\r\n");
        return -1;
    }
    rwnx_hw->plat = rwnx_plat;
    rwnx_hw->bus_if = rwnx_plat->sdiodev->bus_if;
    rwnx_hw->sdiodev = rwnx_plat->sdiodev;
    rwnx_plat->sdiodev->rwnx_hw = rwnx_hw;

    ret = rwnx_platform_on(rwnx_hw);
    if (ret)
    {
        goto err_platon;
    }

    *platform_data = rwnx_hw;

    // init
    g_custom_msg_vnet.aic_mode_status       = AIC_MODE_IDLE;
    g_custom_msg_vnet.wlan_status           = WLAN_DISCONNECT;
    rtos_memcpy(&g_custom_msg_vnet.mac_addr[0], &default_mac[0], MAC_ADDR_LEN);

    ret = rtos_task_create(vnet_init_task, "vnet_init_task", XDRV_VNET_INIT_TASK,
                           xdrv_vnet_init_stack_size, NULL, xdrv_vnet_init_priority,
                           &task_handle);
    if (ret)
    {
        DBG_VNET_ERR("vnet init task create fail,%d\n", ret);
    }
#if 0
    // Test dbg msg
    const uint32_t mem_addr = 0x40500000;
    struct dbg_mem_read_cfm rd_mem_addr_cfm;
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, mem_addr, &rd_mem_addr_cfm);
    if (ret)
    {
        DBG_VNET_ERR("%x rd fail: %d\n", mem_addr, ret);
        goto err_test;
    }
#endif

    return 0;

err_test:
    rwnx_platform_off(rwnx_hw);

err_platon:
    rtos_free(rwnx_hw);
    return ret;
}

void rwnx_vnet_deinit(struct rwnx_hw *rwnx_hw)
{
    RWNX_DBG(RWNX_FN_ENTRY_STR);
    rwnx_platform_off(rwnx_hw);
    rtos_free(rwnx_hw);
    VNET_RWNX_HW = NULL;
#if defined(CONFIG_VNET_MODE) && !defined(RT_USING_WIFI)
    netal_eth_protocol_linkchange(0);
#endif
}
