/** @file net.c
 *
 *  @brief  This file provides network porting code
 *
 *  Copyright 2008-2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <inttypes.h>
#include <wifi.h>

#include <os_adaptor.h>
#include <osa_freertos.h>

#include <wm_net.h>
#include <wlan_wifi.h>
#include <lwip/opt.h>
#include <lwip/netifapi.h>
#include <lwip/tcpip.h>
#include <lwip/dns.h>
#include <lwip/dhcp.h>
#include <lwip/prot/dhcp.h>
#include <lwip/ip_addr.h>
#include <lwip/prot/autoip.h>
#include <lwip/stats.h>
#include <wmlog.h>
#include <wifi.h>

#ifdef CONFIG_IPV6
    #define IPV6_ADDR_STATE_TENTATIVE  "Tentative"
    #define IPV6_ADDR_STATE_PREFERRED  "Preferred"
    #define IPV6_ADDR_STATE_INVALID    "Invalid"
    #define IPV6_ADDR_STATE_VALID      "Valid"
    #define IPV6_ADDR_STATE_DEPRECATED "Deprecated"
    #define IPV6_ADDR_TYPE_LINKLOCAL   "Link-Local"
    #define IPV6_ADDR_TYPE_GLOBAL      "Global"
    #define IPV6_ADDR_TYPE_UNIQUELOCAL "Unique-Local"
    #define IPV6_ADDR_TYPE_SITELOCAL   "Site-Local"
    #define IPV6_ADDR_UNKNOWN          "Unknown"
#endif

#define DNS_PORT   0x35
#define DHCPD_PORT 0x43
#define DHCPC_PORT 0x44
#ifdef CONFIG_IPV6
    #define DHCP_TIMEOUT (60 * 1000)
#else
    #define DHCP_TIMEOUT (120 * 1000)
#endif

#define net_e(...) wmlog_e("net", ##__VA_ARGS__)

#ifdef CONFIG_NET_DEBUG
    #define net_d(...) wmlog("net", ##__VA_ARGS__)
#else
    #define net_d(...)
#endif /* ! CONFIG_NET_DEBUG */

struct interface
{
    struct netif netif;
    ip_addr_t ipaddr;
    ip_addr_t nmask;
    ip_addr_t gw;
};
typedef struct interface interface_t;

static interface_t g_mlan;
static interface_t g_uap;

static int net_wlan_init_done;
static os_timer_t dhcp_timer;

static void dhcp_timer_cb(osa_timer_arg_t arg);

err_t lwip_netif_init(struct netif *netif);
err_t lwip_netif_uap_init(struct netif *netif);
void handle_data_packet(const t_u8 interface, const t_u8 *rcvdata, const t_u16 datalen);
void handle_amsdu_data_packet(t_u8 interface, t_u8 *rcvdata, t_u16 datalen);
void handle_deliver_packet_above(t_void *rxpd, t_u8 interface, t_void *lwip_pbuf);
bool wrapper_net_is_ip_or_ipv6(const t_u8 *buffer);

NETIF_DECLARE_EXT_CALLBACK(netif_ext_callback)

#ifdef CONFIG_IPV6
    static void wm_netif_ipv6_status_callback(struct netif *n);
#endif

static void (*wm_netif_status_callback_ptr)(struct netif *n);
static void wm_netif_status_callback(struct netif *n);

static void netif_ext_status_callback(struct netif *netif,
                                      netif_nsc_reason_t reason,
                                      const netif_ext_callback_args_t *args)
{
    interface_t *if_handle = (interface_t *)net_get_mlan_handle();

    if (&if_handle->netif == netif)
    {
#ifdef CONFIG_IPV6
        if ((reason & (LWIP_NSC_IPV6_ADDR_STATE_CHANGED | LWIP_NSC_IPV6_SET)) != LWIP_NSC_NONE)
        {
            wm_netif_ipv6_status_callback(netif);
        }
        else
#endif
        {
            if ((reason & (LWIP_NSC_IPV4_SETTINGS_CHANGED | LWIP_NSC_IPV4_ADDRESS_CHANGED | LWIP_NSC_IPV4_ADDR_VALID |
                           LWIP_NSC_IPV4_GATEWAY_CHANGED | LWIP_NSC_IPV4_NETMASK_CHANGED)) != LWIP_NSC_NONE)
            {
                if (wm_netif_status_callback_ptr != NULL)
                {
                    wm_netif_status_callback_ptr(netif);
                }
            }
        }
    }
}

#ifdef CONFIG_IPV6
char *ipv6_addr_state_to_desc(unsigned char addr_state)
{
    if (ip6_addr_istentative((addr_state)) != 0U)
    {
        return IPV6_ADDR_STATE_TENTATIVE;
    }
    else if (ip6_addr_ispreferred((addr_state)))
    {
        return IPV6_ADDR_STATE_PREFERRED;
    }
    else if (ip6_addr_isdeprecated((addr_state)))
    {
        return IPV6_ADDR_STATE_DEPRECATED;
    }
    else if (ip6_addr_isinvalid((addr_state)))
    {
        return IPV6_ADDR_STATE_INVALID;
    }
    else
    {
        return IPV6_ADDR_UNKNOWN;
    }
}

char *ipv6_addr_addr_to_desc(struct net_ipv6_config *ipv6_conf)
{
    ip6_addr_t ip6_addr;

    (void)memcpy((void *)ip6_addr.addr, (const void *)ipv6_conf->address, sizeof(ip6_addr.addr));

    return inet6_ntoa(ip6_addr);
}

char *ipv6_addr_type_to_desc(struct net_ipv6_config *ipv6_conf)
{
    ip6_addr_t ip6_addr;

    (void)memcpy((void *)ip6_addr.addr, (const void *)ipv6_conf->address, sizeof(ip6_addr.addr));

    if (ip6_addr_islinklocal(&ip6_addr))
    {
        return IPV6_ADDR_TYPE_LINKLOCAL;
    }
    else if (ip6_addr_isglobal(&ip6_addr))
    {
        return IPV6_ADDR_TYPE_GLOBAL;
    }
    else if (ip6_addr_isuniquelocal(&ip6_addr))
    {
        return IPV6_ADDR_TYPE_UNIQUELOCAL;
    }
    else if (ip6_addr_issitelocal(&ip6_addr))
    {
        return IPV6_ADDR_TYPE_SITELOCAL;
    }
    else
    {
        return IPV6_ADDR_UNKNOWN;
    }
}
#endif /* CONFIG_IPV6 */

int net_dhcp_hostname_set(char *hostname)
{
    netif_set_hostname(&g_mlan.netif, hostname);
    return WM_SUCCESS;
}

#ifndef CONFIG_NO_WIFI_TCPIP_INIT
static void tcpip_init_done_cb(void *arg)
{
    sys_sem_t *init_sem;

    LWIP_ASSERT("arg != NULL", arg != NULL);

    init_sem = (sys_sem_t *)arg;

    sys_sem_signal(init_sem);
}

void net_ipv4stack_init(void)
{
    static bool tcpip_init_done;
    err_t err;
    sys_sem_t init_sem;

    if (tcpip_init_done)
    {
        return;
    }

    err = sys_sem_new(&init_sem, 0);
    LWIP_ASSERT("failed to create init_sem", err == ERR_OK);
    LWIP_UNUSED_ARG(err);

    tcpip_init(tcpip_init_done_cb, &init_sem);

    sys_sem_wait(&init_sem);
    sys_sem_free(&init_sem);

    tcpip_init_done = true;

    net_d("Initialized TCP/IP v4 stack");
}
#endif

#ifdef CONFIG_IPV6
void net_ipv6stack_init(struct netif *netif)
{
    LOCK_TCPIP_CORE();
    netif_create_ip6_linklocal_address(netif, 1);
    UNLOCK_TCPIP_CORE();

    net_d("Initialized TCP/IP v6 stack for interface : %c%c%d", netif->name[0], netif->name[1], netif->num);
}

static void wm_netif_ipv6_status_callback(struct netif *n)
{
    /*  TODO: Implement appropriate functionality here*/
    net_d("Received callback on IPv6 address state change");

    (void)wlan_wlcmgr_send_msg(WIFI_EVENT_NET_IPV6_CONFIG, WIFI_EVENT_REASON_SUCCESS, NULL);
}
#endif /* CONFIG_IPV6 */

void net_wlan_set_mac_address(unsigned char *stamac, unsigned char *uapmac)
{
    (void)memcpy(&g_mlan.netif.hwaddr[0], &stamac[0], MLAN_MAC_ADDR_LENGTH);
    (void)memcpy(&g_uap.netif.hwaddr[0], &uapmac[0], MLAN_MAC_ADDR_LENGTH);
}
//#include "skw_sdio_core.h"
int net_wlan_init(void)
{
    int ret;

    if (!net_wlan_init_done)
    {
#ifndef CONFIG_NO_WIFI_TCPIP_INIT
        net_ipv4stack_init();
#endif

        (void)wifi_register_data_input_callback(&handle_data_packet);
        (void)wifi_register_amsdu_data_input_callback(&handle_amsdu_data_packet);
        (void)wifi_register_deliver_packet_above_callback(&handle_deliver_packet_above);
        (void)wifi_register_wrapper_net_is_ip_or_ipv6_callback(&wrapper_net_is_ip_or_ipv6);
        ip_2_ip4(&g_mlan.ipaddr)->addr = INADDR_ANY;

        ret = netifapi_netif_add(&g_mlan.netif, ip_2_ip4(&g_mlan.ipaddr), ip_2_ip4(&g_mlan.ipaddr),
                                 ip_2_ip4(&g_mlan.ipaddr), NULL, lwip_netif_init, tcpip_input);//注册为ml设备的网卡

        if (ret != WM_SUCCESS)
        {
            net_e("MLAN interface add failed");

            return -WM_FAIL;
        }
#ifdef CONFIG_IPV6
        net_ipv6stack_init(&g_mlan.netif);

#endif /* CONFIG_IPV6 */

        // ret = netifapi_netif_add(&g_uap.netif, ip_2_ip4(&g_uap.ipaddr), ip_2_ip4(&g_uap.ipaddr),
        //                          ip_2_ip4(&g_uap.ipaddr), NULL, lwip_netif_uap_init, tcpip_input);
        // if (ret != WM_SUCCESS)
        // {
        //     net_e("UAP interface add failed");
        //     return -WM_FAIL;
        // }
#ifdef CONFIG_IPV6
        net_ipv6stack_init(&g_uap.netif);
#endif /* CONFIG_IPV6 */


        ret = os_timer_create_t(&dhcp_timer, "dhcp-timer", rt_tick_from_millisecond(DHCP_TIMEOUT), &dhcp_timer_cb, NULL,
                                OS_TIMER_ONE_SHOT, OS_TIMER_NO_ACTIVATE);
        if (ret != WM_SUCCESS)
        {
            net_e("Unable to start dhcp timer");
            return ret;
        }

        LOCK_TCPIP_CORE();
        netif_add_ext_callback(&netif_ext_callback, netif_ext_status_callback);
        UNLOCK_TCPIP_CORE();

        net_wlan_init_done = 1;

        net_d("Initialized TCP/IP networking stack");
    }

    (void)wlan_wlcmgr_send_msg(WIFI_EVENT_NET_INTERFACE_CONFIG, WIFI_EVENT_REASON_SUCCESS, NULL);
    return WM_SUCCESS;
}

struct netif *net_get_sta_interface(void)
{
    return &g_mlan.netif;
}

struct netif *net_get_uap_interface(void)
{
    return &g_uap.netif;
}

int net_get_if_name_netif(char *pif_name, struct netif *iface)
{
    char if_name[NETIF_NAMESIZE];
    int ret;

    ret = netifapi_netif_index_to_name(iface->num + 1, if_name);

    if (ret != WM_SUCCESS)
    {
        net_e("get interface name failed");
        return -WM_FAIL;
    }

    (void)strncpy(pif_name, if_name, NETIF_NAMESIZE);

    return WM_SUCCESS;
}

static int net_netif_deinit(struct netif *netif)
{
    int ret;
#ifdef CONFIG_IPV6
    if (netif->mld_mac_filter != NULL)
    {
        ip6_addr_t ip6_allnodes_ll;
        ip6_addr_set_allnodes_linklocal(&ip6_allnodes_ll);
        (void)netif->mld_mac_filter(netif, &ip6_allnodes_ll, NETIF_DEL_MAC_FILTER);
    }
#endif
    ret = netifapi_netif_remove(netif);

    if (ret != WM_SUCCESS)
    {
        net_e("Interface remove failed");
        return -WM_FAIL;
    }

    if (netif->state != NULL)
    {
        mem_free(netif->state);
        netif->state = NULL;
    }

    return WM_SUCCESS;
}

int net_wlan_deinit(void)
{
    int ret;

    if (net_wlan_init_done != 1)
    {
        return -WM_FAIL;
    }

    ret = net_netif_deinit(&g_mlan.netif);
    if (ret != WM_SUCCESS)
    {
        net_e("MLAN interface deinit failed");
        return -WM_FAIL;
    }

    ret = net_netif_deinit(&g_uap.netif);
    if (ret != WM_SUCCESS)
    {
        net_e("UAP interface deinit failed");
        return -WM_FAIL;
    }

    ret = os_timer_delete(&dhcp_timer);
    if (ret != WM_SUCCESS)
    {
        net_e("DHCP timer deletion failed");
        return -WM_FAIL;
    }

    LOCK_TCPIP_CORE();
    netif_remove_ext_callback(&netif_ext_callback);
    UNLOCK_TCPIP_CORE();

    wm_netif_status_callback_ptr = NULL;
    net_wlan_init_done           = 0;

    net_d("DeInitialized TCP/IP networking stack");

    return WM_SUCCESS;
}

static void wm_netif_status_callback(struct netif *n)
{
    /* Variables to hold whether dhcp address has been supplied */
    /* is_dhcp_address: true when an dhcp address is available */
    bool is_dhcp_address = ((n->flags & NETIF_FLAG_UP) && (dhcp_supplied_address(n)));
    /* is_default_dhcp_address: true when the ip address of the passed
     *                          interface is the same as INADDR_ANY
     */
    bool is_default_dhcp_address = (ip_2_ip4(&(n->ip_addr))->addr == INADDR_ANY);
    /* is_dhcp_off: true if dhcp is switched off*/
    bool is_dhcp_off = (netif_dhcp_data(n)->state == DHCP_STATE_OFF);
    /* State variables to be assigned to the event flag
     * a value of -1 represents failed state while the value of
     * 1 represents a successful connection event
     * The value of zero is default
     */
    enum connection_states
    {
        DHCP_FAILED  = -1,
        DHCP_IGNORE  = 0,
        DHCP_SUCCESS = 1
    } event_flag_dhcp_connection;
    /* A flag to keep the status of dhcp connection
     * This flag is in state 1, if the connection has succeeded
     * This flag is in state -1, if the connection has failed
     * The flag value is zero, if we are still in process of connection
     * establishment or dhcp is off
     */
    enum wifi_event_reason wifi_event_reason;
    event_flag_dhcp_connection = DHCP_IGNORE;

    if (is_dhcp_off)
    {
        /* Do Nothing */
    }
    else if (is_dhcp_address && !(is_default_dhcp_address))
    {
        /* If a valid non-default dhcp address is provided */
        event_flag_dhcp_connection = DHCP_SUCCESS;
    }
    else if (is_dhcp_address)
    {
        /* If the supplied dhcp address is the default address */
        event_flag_dhcp_connection = DHCP_FAILED;
    }
    else if (!dhcp_supplied_address(n))
    {
        /* If no ip address is supplied */
        event_flag_dhcp_connection = DHCP_FAILED;
    }
    else
    {
        /* Do Nothing */
    }

    /* Based on the value of status flag send corresponding message
     * to wlcmgr
     */
    switch (event_flag_dhcp_connection)
    {
    case DHCP_SUCCESS:
        wifi_event_reason = WIFI_EVENT_REASON_SUCCESS;
        break;
    case DHCP_FAILED:
        wifi_event_reason = WIFI_EVENT_REASON_FAILURE;
        break;
    default:
        net_d("Unexpected DHCP event");
        break;
    }
    if (event_flag_dhcp_connection != DHCP_IGNORE)
    {
        (void)wlan_wlcmgr_send_msg(WIFI_EVENT_NET_DHCP_CONFIG, wifi_event_reason, NULL);
    }
}

void net_stop_dhcp_timer(void)
{
    (void)os_timer_deactivate((os_timer_t *)&dhcp_timer);
}

static void stop_cb(void *ctx)
{
    interface_t *if_handle = (interface_t *)net_get_mlan_handle();

    dhcp_release_and_stop(&if_handle->netif);
    netif_set_down(&if_handle->netif);
    wm_netif_status_callback_ptr = NULL;
}

static void dhcp_timer_cb(osa_timer_arg_t arg)
{
    (void)tcpip_try_callback(stop_cb, NULL);
    net_e("DHCP timeout, failed to get IPv4 address");
    (void)wlan_wlcmgr_send_msg(WIFI_EVENT_NET_DHCP_CONFIG, WIFI_EVENT_REASON_FAILURE, NULL);
}

static int check_iface_mask(void *handle, uint32_t ipaddr)
{
    uint32_t interface_ip, interface_mask;
    (void)net_get_if_ip_addr(&interface_ip, handle);
    (void)net_get_if_ip_mask(&interface_mask, handle);
    if (interface_ip > 0U)
    {
        if ((interface_ip & interface_mask) == (ipaddr & interface_mask))
        {
            return WM_SUCCESS;
        }
    }
    return -WM_FAIL;
}

static void *net_ip_to_interface(uint32_t ipaddr)
{
    int ret;
    void *handle;
    /* Check mlan handle */
    handle = net_get_mlan_handle();
    ret    = check_iface_mask(handle, ipaddr);
    if (ret == WM_SUCCESS)
    {
        return handle;
    }

    /* Check uap handle */
    handle = net_get_uap_handle();
    ret    = check_iface_mask(handle, ipaddr);
    if (ret == WM_SUCCESS)
    {
        return handle;
    }

    /* If more interfaces are added then above check needs to done for
     * those newly added interfaces
     */
    return NULL;
}

void *net_sock_to_interface(int sock)
{
    struct sockaddr_in peer;
    socklen_t peerlen = sizeof(peer);
    void *req_iface   = NULL;
    int ret;

    (void)memset(&peer, 0, sizeof(struct sockaddr_in));
    ret = getpeername(sock, (struct sockaddr *)(void *)&peer, &peerlen);
    if (ret < 0)
    {
        net_e("Failed to get peer name");
    }
    req_iface = net_ip_to_interface(peer.sin_addr.s_addr);
    return req_iface;
}

void *net_get_sta_handle(void)
{
    return &g_mlan;
}

void *net_get_uap_handle(void)
{
    return &g_uap;
}


int net_alloc_client_data_id()
{
    int idx = -1;

#if LWIP_NUM_NETIF_CLIENT_DATA > 0
    LOCK_TCPIP_CORE();
    idx = (int)netif_alloc_client_data_id();
    UNLOCK_TCPIP_CORE();
#endif

    return idx;
}

void net_interface_up(void *intrfc_handle)
{
    interface_t *if_handle = (interface_t *)intrfc_handle;
    (void)netifapi_netif_set_up(&if_handle->netif);
}

void net_interface_down(void *intrfc_handle)
{
    interface_t *if_handle = (interface_t *)intrfc_handle;
    (void)netifapi_netif_set_down(&if_handle->netif);
}

void net_interface_dhcp_stop(void *intrfc_handle)
{
    interface_t *if_handle = (interface_t *)intrfc_handle;
    (void)netifapi_dhcp_release_and_stop(&if_handle->netif);
    wm_netif_status_callback_ptr = NULL;
}

void net_interface_dhcp_cleanup(void *intrfc_handle)
{
    interface_t *if_handle = (interface_t *)intrfc_handle;
    (void)dhcp_cleanup(&if_handle->netif);
}

int net_configure_address(struct net_ip_config *addr, void *intrfc_handle)
{
#ifdef CONFIG_IPV6
    t_u8 i;
#endif

    if (addr == NULL)
    {
        return -WM_E_INVAL;
    }
    if (intrfc_handle == NULL)
    {
        return -WM_E_INVAL;
    }

    interface_t *if_handle = (interface_t *)intrfc_handle;

    net_d("configuring interface %s (with %s)", (if_handle == &g_mlan) ? "mlan" : "uap",
          (addr->ipv4.addr_type == NET_ADDR_TYPE_DHCP) ? "DHCP client" : "Static IP");

    (void)netifapi_netif_set_down(&if_handle->netif);
    wm_netif_status_callback_ptr = NULL;

#ifdef CONFIG_IPV6
    if (if_handle == &g_mlan || if_handle == &g_uap)
    {
        LOCK_TCPIP_CORE();

        for (i = 0; i < CONFIG_MAX_IPV6_ADDRESSES; i++)
        {
            if (if_handle->netif.ip6_addr_state[i] != IP6_ADDR_INVALID)
            {
                netif_ip6_addr_set_state(&if_handle->netif, i, IP6_ADDR_INVALID);
                netif_ip6_addr_set(&if_handle->netif, i, IP6_ADDR_ANY6);
            }
        }

        netif_create_ip6_linklocal_address(&if_handle->netif, 1);

        UNLOCK_TCPIP_CORE();

        /* Explicitly call this function so that the linklocal address
         * gets updated even if the interface does not get any IPv6
         * address in its lifetime */
        if (if_handle == &g_mlan)
        {
            wm_netif_ipv6_status_callback(&if_handle->netif);
        }
    }
#endif
    if (if_handle == &g_mlan)
    {
        (void)netifapi_netif_set_default(&if_handle->netif);
    }
    switch (addr->ipv4.addr_type)
    {
    case NET_ADDR_TYPE_STATIC:
        ip_2_ip4(&(if_handle->ipaddr))->addr = addr->ipv4.address;
        ip_2_ip4(&(if_handle->nmask))->addr  = addr->ipv4.netmask;
        ip_2_ip4(&(if_handle->gw))->addr     = addr->ipv4.gw;
        (void)netifapi_netif_set_addr(&if_handle->netif, ip_2_ip4(&if_handle->ipaddr), ip_2_ip4(&if_handle->nmask),
                                      ip_2_ip4(&if_handle->gw));
        (void)netifapi_netif_set_up(&if_handle->netif);
        break;

    case NET_ADDR_TYPE_DHCP:
        /* Reset the address since we might be
           transitioning from static to DHCP */
        (void)memset(&if_handle->ipaddr, 0, sizeof(ip_addr_t));
        (void)memset(&if_handle->nmask, 0, sizeof(ip_addr_t));
        (void)memset(&if_handle->gw, 0, sizeof(ip_addr_t));
        (void)netifapi_netif_set_addr(&if_handle->netif, ip_2_ip4(&if_handle->ipaddr), ip_2_ip4(&if_handle->nmask),
                                      ip_2_ip4(&if_handle->gw));
        (void)netifapi_netif_set_up(&if_handle->netif);
        (void)os_timer_activate(&dhcp_timer);
        wm_netif_status_callback_ptr = wm_netif_status_callback;
        (void)netifapi_dhcp_start(&if_handle->netif);
        break;
    case NET_ADDR_TYPE_LLA:
        /* For dhcp, instead of netifapi_netif_set_up, a
           netifapi_dhcp_start() call will be used */
        net_e("Not supported as of now...");
        break;
    default:
        net_d("Unexpected addr type");
        break;
    }
    /* Finally this should send the following event. */
    if (if_handle == &g_mlan)
    {
        (void)wlan_wlcmgr_send_msg(WIFI_EVENT_NET_STA_ADDR_CONFIG, WIFI_EVENT_REASON_SUCCESS, NULL);

        /* XXX For DHCP, the above event will only indicate that the
         * DHCP address obtaining process has started. Once the DHCP
         * address has been obtained, another event,
         * WD_EVENT_NET_DHCP_CONFIG, should be sent to the wlcmgr.
         */
    }
    else if (if_handle == &g_uap)
    {
        (void)wlan_wlcmgr_send_msg(WIFI_EVENT_UAP_NET_ADDR_CONFIG, WIFI_EVENT_REASON_SUCCESS, NULL);
    }
    else
    {
        /* Do Nothing */
    }

    return WM_SUCCESS;
}

int net_get_if_addr(struct net_ip_config *addr, void *intrfc_handle)
{
    const ip_addr_t *tmp;
    interface_t *if_handle = (interface_t *)intrfc_handle;

    addr->ipv4.address = ip_2_ip4(&(if_handle->netif.ip_addr))->addr;
    addr->ipv4.netmask = ip_2_ip4(&(if_handle->netif.netmask))->addr;
    addr->ipv4.gw      = ip_2_ip4(&(if_handle->netif.gw))->addr;

    tmp             = dns_getserver(0);
    addr->ipv4.dns1 = ip_2_ip4(tmp)->addr;
    tmp             = dns_getserver(1);
    addr->ipv4.dns2 = ip_2_ip4(tmp)->addr;

    return WM_SUCCESS;
}

#ifdef CONFIG_IPV6
int net_get_if_ipv6_addr(struct net_ip_config *addr, void *intrfc_handle)
{
    interface_t *if_handle = (interface_t *)intrfc_handle;
    int i;

    for (i = 0; i < CONFIG_MAX_IPV6_ADDRESSES; i++)
    {
        (void)memcpy(addr->ipv6[i].address, ip_2_ip6(&(if_handle->netif.ip6_addr[i]))->addr, 16);
        addr->ipv6[i].addr_state = if_handle->netif.ip6_addr_state[i];
    }
    /* TODO carry out more processing based on IPv6 fields in netif */
    return WM_SUCCESS;
}

int net_get_if_ipv6_pref_addr(struct net_ip_config *addr, void *intrfc_handle)
{
    int i, ret = 0;
    interface_t *if_handle = (interface_t *)intrfc_handle;

    for (i = 0; i < CONFIG_MAX_IPV6_ADDRESSES; i++)
    {
        if (if_handle->netif.ip6_addr_state[i] == IP6_ADDR_PREFERRED)
        {
            (void)memcpy(addr->ipv6[ret++].address, ip_2_ip6(&(if_handle->netif.ip6_addr[i]))->addr, 16);
        }
    }
    return ret;
}
#endif /* CONFIG_IPV6 */

int net_get_if_name(char *pif_name, void *intrfc_handle)
{
    interface_t *if_handle       = (interface_t *)intrfc_handle;
    char if_name[NETIF_NAMESIZE] = {0};
    int ret;

    ret = netifapi_netif_index_to_name(if_handle->netif.num + 1, if_name);

    if (ret != WM_SUCCESS)
    {
        net_e("get interface name failed");
        return -WM_FAIL;
    }

    (void)strncpy(pif_name, if_name, NETIF_NAMESIZE);

    return WM_SUCCESS;
}

int net_get_if_ip_addr(uint32_t *ip, void *intrfc_handle)
{
    interface_t *if_handle = (interface_t *)intrfc_handle;

    *ip = ip_2_ip4(&(if_handle->netif.ip_addr))->addr;
    return WM_SUCCESS;
}

int net_get_if_ip_mask(uint32_t *nm, void *intrfc_handle)
{
    interface_t *if_handle = (interface_t *)intrfc_handle;

    *nm = ip_2_ip4(&(if_handle->netif.netmask))->addr;
    return WM_SUCCESS;
}

void net_configure_dns(struct net_ip_config *ip, unsigned int role)
{
    ip4_addr_t tmp;

    if (ip->ipv4.addr_type == NET_ADDR_TYPE_STATIC)
    {
        if (role != WLAN_BSS_ROLE_UAP)
        {
            if (ip->ipv4.dns1 == 0U)
            {
                ip->ipv4.dns1 = ip->ipv4.gw;
            }
            if (ip->ipv4.dns2 == 0U)
            {
                ip->ipv4.dns2 = ip->ipv4.dns1;
            }
        }
        tmp.addr = ip->ipv4.dns1;
        dns_setserver(0, (ip_addr_t *)(void *)&tmp);
        tmp.addr = ip->ipv4.dns2;
        dns_setserver(1, (ip_addr_t *)(void *)&tmp);
    }

    /* DNS MAX Retries should be configured in lwip/dns.c to 3/4 */
    /* DNS Cache size of about 4 is sufficient */
}

void net_stat(void)
{
    stats_display();
}
