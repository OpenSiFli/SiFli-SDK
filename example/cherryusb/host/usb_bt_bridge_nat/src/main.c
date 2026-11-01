/*
 * SPDX-FileCopyrightText: 2024-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "usbh_core.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#ifdef RT_USING_NETDEV
    #include <netdev.h>
#endif
#ifdef LWIP_USING_NAT
    #include "ipv4_nat.h"
#endif
#include "bf0_sibles.h"
#ifdef BT_FINSH
    #include "bts2_msg.h"
    #include "gap_api.h"
    #include "sc_api.h"
    #include "bts2_app_interface.h"
    #ifdef BT_FINSH_PAN
        #include "bts2_app_pan.h"
        #include "pan_api.h"
    #endif
#endif
#include <stdint.h>
#include <string.h>

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
    #include "dfs_file.h"
    #include "dfs_posix.h"
    #include "drv_flash.h"
#endif

#define RNDIS_NETIF_NAME "u2"
#define PAN_NETIF_NAME "b0"
#define DHCP_TIMEOUT_MS 30000U
#define DHCP_POLL_INTERVAL_MS 500U
#define DHCP_REPORT_INTERVAL_MS 5000U
#define RNDIS_LINK_SETTLE_MS 3000U
#define BT_APP_READY 1U
#define PAN_CONNECT_TIMEOUT_MS 30000U
#define PAN_INQUIRY_TIMEOUT_S 15U
#define PAN_INQUIRY_MAX_RESPONSES 10U

struct rndis_status
{
    uint8_t present;
    uint8_t link_up;
    uint8_t dhcp_supplied;
    char ip[IP4ADDR_STRLEN_MAX];
    char netmask[IP4ADDR_STRLEN_MAX];
    char gateway[IP4ADDR_STRLEN_MAX];
};

#ifdef USB_BT_BRIDGE_BT_NAME
    #define USB_BT_BRIDGE_LOCAL_NAME USB_BT_BRIDGE_BT_NAME
#else
    #define USB_BT_BRIDGE_LOCAL_NAME "SiFli_USB_Bridge"
#endif

static rt_mailbox_t s_bt_ready_mb;
static volatile uint8_t s_bt_stack_ready;
#ifdef LWIP_USING_NAT
    static void update_nat(void);
    static uint8_t s_nat_initialized;
    static uint8_t s_nat_enabled;
    static ip_nat_entry_t s_nat_entry;
    /* set by the BT event callback, applied by the monitor thread */
    static volatile uint8_t s_nat_dirty;
#endif
#if defined(BT_FINSH) && defined(BT_FINSH_PAN)
    static rt_timer_t s_pan_connect_timer;
    static volatile uint8_t s_pan_connect_pending;
    static volatile uint8_t s_pan_connected;
    static BTS2S_BD_ADDR s_pan_target;
#endif

static const char *format_bt_addr(const uint8_t addr[6], char *buf, size_t size)
{
    rt_snprintf(buf, size, "%02X:%02X:%02X:%02X:%02X:%02X",
                addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    return buf;
}

static void pan_connect_timeout(void *parameter)
{
    (void)parameter;
    if (s_pan_connect_pending)
    {
        s_pan_connect_pending = 0U;
        rt_kprintf("[BT] PAN connect timeout: %04X:%02X:%06lX\n",
                   s_pan_target.nap, s_pan_target.uap, s_pan_target.lap);
    }
}

static int usb_bt_bridge_bt_event_handler(uint16_t type,
        uint16_t event_id,
        uint8_t *data,
        uint16_t data_len)
{
    if (type == BT_NOTIFY_COMMON &&
            event_id == BT_NOTIFY_COMMON_BT_STACK_READY)
    {
        s_bt_stack_ready = 1U;
        if (s_bt_ready_mb != RT_NULL)
        {
            if (rt_mb_send(s_bt_ready_mb, BT_APP_READY) != RT_EOK)
            {
                rt_kprintf("[BT] Ready event mailbox full\n");
            }
        }
    }
#if defined(BT_FINSH) && defined(BT_FINSH_PAN)
    else if (type == BT_NOTIFY_COMMON &&
             event_id == BT_NOTIFY_COMMON_DISCOVER_IND)
    {
        const bt_notify_remote_device_info_t *device;
        size_t name_length;
        char mac_str[18];

        if (data == RT_NULL || data_len < sizeof(bt_notify_remote_device_info_t))
        {
            rt_kprintf("[BT] Invalid inquiry result len=%u\n", data_len);
            return 0;
        }

        device = (const bt_notify_remote_device_info_t *)data;
        name_length = rt_strnlen(device->bt_name, sizeof(device->bt_name));
        rt_kprintf("[BT] Found %s name=%.*s rssi=%d class=0x%06lX\n",
                   format_bt_addr(device->mac.addr, mac_str, sizeof(mac_str)),
                   (int)name_length,
                   name_length > 0U ? device->bt_name : "<unknown>",
                   device->rssi, device->dev_cls);
    }
    else if (type == BT_NOTIFY_COMMON &&
             event_id == BT_NOTIFY_COMMON_INQUIRY_CMP)
    {
        rt_kprintf("[BT] Inquiry completed\n");
    }
#endif
    else if (type == BT_NOTIFY_PAN)
    {
        const bt_notify_profile_state_info_t *profile;
        char mac_str[18];

        if (data == RT_NULL || data_len < sizeof(bt_notify_profile_state_info_t))
        {
            rt_kprintf("[BT] Invalid PAN event=%u len=%u\n", event_id, data_len);
            return 0;
        }

        profile = (const bt_notify_profile_state_info_t *)data;
        if (event_id == BT_NOTIFY_PAN_PROFILE_CONNECTED)
        {
#if defined(BT_FINSH) && defined(BT_FINSH_PAN)
            s_pan_connect_pending = 0U;
            s_pan_connected = 1U;
#ifdef LWIP_USING_NAT
            /* do not take the lwIP core lock from this callback, let the
             * monitor thread apply the change */
            s_nat_dirty = 1U;
#endif
            if (s_pan_connect_timer != RT_NULL)
            {
                rt_timer_stop(s_pan_connect_timer);
            }
#endif
            rt_kprintf("[BT] PAN connected %s role=0x%02X channel=%u, b0 should be available\n",
                       format_bt_addr(profile->mac.addr, mac_str, sizeof(mac_str)),
                       profile->profile_role, profile->profile_channel);
        }
        else if (event_id == BT_NOTIFY_PAN_PROFILE_DISCONNECTED)
        {
            const char *reason;

#if defined(BT_FINSH) && defined(BT_FINSH_PAN)
            uint8_t was_pending = s_pan_connect_pending;

            s_pan_connect_pending = 0U;
            s_pan_connected = 0U;
#ifdef LWIP_USING_NAT
            s_nat_dirty = 1U;
#endif
            if (s_pan_connect_timer != RT_NULL)
            {
                rt_timer_stop(s_pan_connect_timer);
            }
            reason = was_pending ? "connect failed" : "disconnected";
#else
            reason = "disconnected";
#endif
            rt_kprintf("[BT] PAN %s %s reason=0x%02X\n",
                       reason,
                       format_bt_addr(profile->mac.addr, mac_str, sizeof(mac_str)),
                       profile->res);
        }
    }

    return 0;
}

static void bt_start_thread(void *parameter)
{
    rt_uint32_t event;

    (void)parameter;
    sifli_ble_enable();
    rt_kprintf("[BT] Bluetooth Classic PAN startup requested\n");

    if (rt_mb_recv(s_bt_ready_mb, &event, rt_tick_from_millisecond(10000)) == RT_EOK &&
            event == BT_APP_READY)
    {
        bt_interface_set_local_name(strlen(USB_BT_BRIDGE_LOCAL_NAME),
                                    (void *)USB_BT_BRIDGE_LOCAL_NAME);
        rt_kprintf("[BT] Classic Bluetooth stack ready, PAN NAP enabled\n");
    }
    else
    {
        rt_kprintf("[BT] Bluetooth stack ready timeout\n");
    }
}

static struct netif *find_netif_locked(const char *name)
{
    struct netif *netif;

    for (netif = netif_list; netif != RT_NULL; netif = netif->next)
    {
        if (netif->name[0] == name[0] && netif->name[1] == name[1])
        {
            return netif;
        }
    }

    return RT_NULL;
}

static struct netif *find_rndis_netif_locked(void)
{
    return find_netif_locked(RNDIS_NETIF_NAME);
}

#ifdef LWIP_USING_NAT
static void update_nat_locked(void)
{
    struct netif *rndis_netif;
    struct netif *pan_netif;
    struct dhcp *dhcp;
    uint8_t ready;
    err_t ret;

    rndis_netif = find_rndis_netif_locked();
    pan_netif = find_netif_locked(PAN_NETIF_NAME);
    dhcp = rndis_netif != RT_NULL ? netif_dhcp_data(rndis_netif) : RT_NULL;
    ready = rndis_netif != RT_NULL && pan_netif != RT_NULL &&
            dhcp != RT_NULL && dhcp_supplied_address(rndis_netif) &&
            netif_is_up(rndis_netif) && netif_is_link_up(rndis_netif) &&
            !ip4_addr_isany_val(*netif_ip4_addr(rndis_netif)) &&
            netif_is_up(pan_netif) && netif_is_link_up(pan_netif) &&
            !ip4_addr_isany_val(*netif_ip4_addr(pan_netif));

    if (s_nat_enabled &&
            (!ready || s_nat_entry.out_if != rndis_netif ||
             s_nat_entry.in_if != pan_netif))
    {
        ip_nat_remove(&s_nat_entry);
        memset(&s_nat_entry, 0, sizeof(s_nat_entry));
        s_nat_enabled = 0U;
        rt_kprintf("[NAT] Disabled\n");
    }

    if (!ready || s_nat_enabled)
    {
        return;
    }

    if (!s_nat_initialized)
    {
        ip_nat_init();
        s_nat_initialized = 1U;
    }

    IP_ADDR4(&s_nat_entry.source_net, 192, 168, 43, 0);
    IP_ADDR4(&s_nat_entry.source_netmask, 255, 255, 255, 0);
    IP_ADDR4(&s_nat_entry.dest_net, 0, 0, 0, 0);
    IP_ADDR4(&s_nat_entry.dest_netmask, 0, 0, 0, 0);
    s_nat_entry.out_if = rndis_netif;
    s_nat_entry.in_if = pan_netif;

    ret = ip_nat_add(&s_nat_entry);
    if (ret == ERR_OK)
    {
        s_nat_enabled = 1U;
        rt_kprintf("[NAT] Enabled 192.168.43.0/24 b0 -> u2\n");
    }
    else
    {
        memset(&s_nat_entry, 0, sizeof(s_nat_entry));
        rt_kprintf("[NAT] Enable failed ret=%d\n", ret);
    }
}

static void update_nat(void)
{
    LOCK_TCPIP_CORE();
    update_nat_locked();
    UNLOCK_TCPIP_CORE();
}
#endif

static void get_rndis_status(struct rndis_status *status)
{
    struct netif *netif;
    struct dhcp *dhcp;

    memset(status, 0, sizeof(*status));
    LOCK_TCPIP_CORE();
    netif = find_rndis_netif_locked();
    if (netif != RT_NULL)
    {
        status->present = 1U;
        status->link_up = netif_is_link_up(netif) ? 1U : 0U;
        dhcp = netif_dhcp_data(netif);
        status->dhcp_supplied = (dhcp != RT_NULL &&
                                 dhcp_supplied_address(netif)) ? 1U : 0U;
        ipaddr_ntoa_r(&netif->ip_addr, status->ip, sizeof(status->ip));
        ipaddr_ntoa_r(&netif->netmask, status->netmask,
                      sizeof(status->netmask));
        ipaddr_ntoa_r(&netif->gw, status->gateway,
                      sizeof(status->gateway));
    }
    UNLOCK_TCPIP_CORE();
}

static int wait_for_rndis_link(void)
{
    struct rndis_status status;

    while (1)
    {
        get_rndis_status(&status);
        if (status.present && status.link_up)
        {
            return 1;
        }

        rt_thread_mdelay(DHCP_POLL_INTERVAL_MS);
    }
}

static int wait_for_dhcp(void)
{
    struct rndis_status status;
    uint32_t elapsed = 0U;
    uint32_t report_elapsed = 0U;
    uint8_t reported = 0U;

    while (elapsed < DHCP_TIMEOUT_MS)
    {
        get_rndis_status(&status);
        if (!status.present || !status.link_up)
        {
            rt_kprintf("[USB] DHCP stopped: RNDIS link lost\n");
            return 0;
        }

        if (status.dhcp_supplied)
        {
            rt_kprintf("[USB] DHCP bound IP=%s mask=%s gw=%s\n",
                       status.ip, status.netmask, status.gateway);
            return 1;
        }

        if (reported == 0U || (elapsed - report_elapsed) >= DHCP_REPORT_INTERVAL_MS)
        {
            rt_kprintf("[USB] DHCP pending ip=%s\n", status.ip);
            reported = 1U;
            report_elapsed = elapsed;
        }

        rt_thread_mdelay(DHCP_POLL_INTERVAL_MS);
        elapsed += DHCP_POLL_INTERVAL_MS;
    }

    rt_kprintf("[USB] DHCP timeout\n");
    return 0;
}

#ifdef RT_USING_NETDEV
static void set_rndis_default_netdev(void)
{
    struct netdev *rndis_netdev;

    rndis_netdev = netdev_get_by_name(RNDIS_NETIF_NAME);
    if (rndis_netdev == RT_NULL)
    {
        return;
    }

    LOCK_TCPIP_CORE();
    netdev_set_default(rndis_netdev);
    UNLOCK_TCPIP_CORE();
}
#endif /* RT_USING_NETDEV */

static void rndis_monitor_thread(void *parameter)
{
    struct rndis_status status;

    (void)parameter;
    rt_kprintf("[USB] Waiting for RNDIS netif '%s'\n", RNDIS_NETIF_NAME);

    while (1)
    {
        wait_for_rndis_link();
        rt_thread_mdelay(RNDIS_LINK_SETTLE_MS);
        if (!wait_for_dhcp())
        {
            rt_thread_mdelay(DHCP_POLL_INTERVAL_MS);
            continue;
        }

#ifdef RT_USING_NETDEV
        set_rndis_default_netdev();
#endif

#ifdef LWIP_USING_NAT
        update_nat();
#endif

        while (1)
        {
            rt_thread_mdelay(DHCP_POLL_INTERVAL_MS);
#ifdef LWIP_USING_NAT
            if (s_nat_dirty)
            {
                s_nat_dirty = 0U;
                update_nat();
            }
#endif
            get_rndis_status(&status);
            if (!status.present || !status.link_up || !status.dhcp_supplied)
            {
#ifdef LWIP_USING_NAT
                update_nat();
#endif
                rt_kprintf("[USB] RNDIS network lost\n");
                break;
            }
        }
    }
}

#if defined(BT_FINSH) && defined(BT_FINSH_PAN)
static void pan_net_help(void)
{
    rt_kprintf("Usage:\n");
    rt_kprintf("  pan_net inquiry start\n");
    rt_kprintf("  pan_net inquiry stop\n");
    rt_kprintf("  pan_net conn <XX:XX:XX:XX:XX:XX|XXXXXXXXXXXX>\n");
}

__ROM_USED void pan_net(int argc, char **argv)
{
    bd_addr_t mac;
    BTS2S_BD_ADDR address;
    int ret;

    if (argc == 3 && strcmp(argv[1], "inquiry") == 0)
    {
        if (!s_bt_stack_ready)
        {
            rt_kprintf("[BT] Bluetooth stack is not ready\n");
            return;
        }

        if (strcmp(argv[2], "start") == 0)
        {
            bt_start_inquiry_ex_t inquiry = {0};

            inquiry.dev_cls_mask = 0U;
            inquiry.max_timeout = PAN_INQUIRY_TIMEOUT_S;
            inquiry.max_rsp = PAN_INQUIRY_MAX_RESPONSES;
            ret = bt_interface_start_inquiry_ex(&inquiry);
            if (ret == 0)
            {
                rt_kprintf("[BT] Inquiry request accepted timeout=%us max_rsp=%u\n",
                           inquiry.max_timeout, inquiry.max_rsp);
            }
            else
            {
                rt_kprintf("[BT] Inquiry request rejected ret=%d\n", ret);
            }
            return;
        }
        if (strcmp(argv[2], "stop") == 0)
        {
            bt_interface_stop_inquiry();
            rt_kprintf("[BT] Inquiry stop requested\n");
            return;
        }
    }
    else if (argc == 3 && strcmp(argv[1], "conn") == 0)
    {
        if (!s_bt_stack_ready)
        {
            rt_kprintf("[BT] Bluetooth stack is not ready\n");
            return;
        }

        if (s_pan_connect_pending)
        {
            rt_kprintf("[BT] PAN connection already pending\n");
            return;
        }
        if (s_pan_connected)
        {
            rt_kprintf("[BT] PAN is already connected\n");
            return;
        }

        if (bt_addr_convert_from_string_to_general(argv[2], &mac) != BD_ADDR_LEN)
        {
            rt_kprintf("[BT] Invalid Bluetooth address: %s\n", argv[2]);
            pan_net_help();
            return;
        }

        bt_addr_convert_to_bts(&mac, &address);
        if (address.nap == 0U && address.uap == 0U && address.lap == 0U)
        {
            rt_kprintf("[BT] Invalid Bluetooth address: %s\n", argv[2]);
            pan_net_help();
            return;
        }

        if (s_pan_connect_timer == RT_NULL)
        {
            s_pan_connect_timer = rt_timer_create("pan_conn",
                                                  pan_connect_timeout,
                                                  RT_NULL,
                                                  rt_tick_from_millisecond(PAN_CONNECT_TIMEOUT_MS),
                                                  RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
            if (s_pan_connect_timer == RT_NULL)
            {
                rt_kprintf("[BT] PAN connect timer create failed\n");
                return;
            }
        }

        s_pan_target = address;
        s_pan_connect_pending = 1U;
        rt_timer_stop(s_pan_connect_timer);
        if (rt_timer_start(s_pan_connect_timer) != RT_EOK)
        {
            s_pan_connect_pending = 0U;
            rt_kprintf("[BT] PAN connect timer start failed\n");
            return;
        }

        bt_interface_stop_inquiry();
        ret = bt_pan_connect_request_ext(&address, PAN_PANU_ROLE);
        if (ret != 0)
        {
            rt_timer_stop(s_pan_connect_timer);
            s_pan_connect_pending = 0U;
            rt_kprintf("[BT] PAN connect request rejected ret=%d\n", ret);
            return;
        }

        rt_kprintf("[BT] PAN connect request accepted: %04X:%02X:%06lX (local NAP, remote PANU)\n",
                   address.nap, address.uap, address.lap);
        return;
    }

    pan_net_help();
}
MSH_CMD_EXPORT(pan_net, scan and connect a Bluetooth PANU device)
#endif

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
#define NAND_MTD_NAME "root"
static int fs_mount_init(void)
{
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000),
                         FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)),
                         FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("[FS] mount fs on flash to root success\n");
    }
    else
    {
        rt_kprintf("[FS] mount fs on flash to root fail, mkfs first\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("[FS] make elm fs on flash success, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
            {
                rt_kprintf("[FS] mount fs on flash success\n");
            }
            else
            {
                rt_kprintf("[FS] mount to fs on flash fail\n");
            }
        }
        else
        {
            rt_kprintf("[FS] dfs_mkfs elm flash fail\n");
        }
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(fs_mount_init);
#endif

int main(void)
{
    rt_thread_t bt_thread;
    rt_thread_t thread;
    int ret;

    s_bt_ready_mb = rt_mb_create("bt_ready", 1, RT_IPC_FLAG_FIFO);
    if (s_bt_ready_mb == RT_NULL)
    {
        rt_kprintf("[BT] Ready mailbox create failed\n");
        return -RT_ENOMEM;
    }

    ret = (int)bt_interface_register_bt_event_notify_callback(
              usb_bt_bridge_bt_event_handler);
    if (ret != BT_INTERFACE_STATUS_OK)
    {
        rt_kprintf("[BT] Event callback registration failed ret=%d\n", ret);
        return ret;
    }

    bt_thread = rt_thread_create("bt_start", bt_start_thread, RT_NULL,
                                 2048, 18, 10);
    if (bt_thread == RT_NULL)
    {
        rt_kprintf("[BT] Startup thread create failed\n");
        return -RT_ENOMEM;
    }
    rt_thread_startup(bt_thread);

    ret = usbh_initialize(0, (uintptr_t)USBC_BASE, RT_NULL);
    if (ret < 0)
    {
        rt_kprintf("[USB] Host init failed ret=%d\n", ret);
        return ret;
    }

    thread = rt_thread_create("rndis_mon", rndis_monitor_thread, RT_NULL,
                              2048, 20, 10);
    if (thread == RT_NULL)
    {
        rt_kprintf("[USB] RNDIS monitor create failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(thread);
    return 0;
}
