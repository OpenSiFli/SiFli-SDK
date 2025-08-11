/** @file wlan_tests.c
 *
 *  @brief  This file provides WLAN Test API
 *
 *  Copyright 2008-2024 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <string.h>
#include <osa.h>
#include <wm_net.h> /* for net_inet_aton */
#include <wlan_wifi.h>
#include <wifi.h>
#include <wlan_tests.h>
#include <wlan_11d.h>
#if CONFIG_WPS2
    #include <wifi_nxp_wps.h>
#endif
#if CONFIG_WPA_SUPP_DPP
    #include "dpp.h"
#endif

#if CONFIG_MEM_POOLS
    #include <mem_pool_config.h>
#endif

#include <cli_utils.h>
#ifdef __ZEPHYR__
    #include "wifi_shell.h"
#else
    #include <cli.h>
#endif

/*
 * NXP Test Framework (MTF) functions
 */

#if (CONFIG_CSI) || (CONFIG_NET_MONITOR)
static uint8_t broadcast_mac[MLAN_MAC_ADDR_LENGTH] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
#endif

#if CONFIG_CSI
    wlan_csi_config_params_t g_csi_params;
#endif

#if CONFIG_NET_MONITOR
    wlan_net_monitor_t g_net_monitor_param;
#endif

#if CONFIG_HOST_SLEEP
    extern uint64_t rtc_timeout;
#endif
#ifdef __ZEPHYR__
    extern char *net_sprint_addr(sa_family_t af, const void *addr);
#endif

static void print_address(struct wlan_ip_config *addr, enum wlan_bss_role role)
{
// #if SDK_DEBUGCONSOLE != DEBUGCONSOLE_DISABLE
#if defined(SDK_OS_FREE_RTOS)
    struct ip4_addr ip, gw, nm, dns1, dns2;
#elif __ZEPHYR__
    struct in_addr ip, gw, nm;
#endif
    char addr_type[10] = {0};

    /* If the current network role is STA and ipv4 is not connected then do
     * not print the addresses */
    if (role == WLAN_BSS_ROLE_STA && !is_sta_ipv4_connected())
    {
        goto out;
    }
#if defined(SDK_OS_FREE_RTOS)
    ip.addr   = addr->ipv4.address;
    gw.addr   = addr->ipv4.gw;
    nm.addr   = addr->ipv4.netmask;
    dns1.addr = addr->ipv4.dns1;
    dns2.addr = addr->ipv4.dns2;
#elif __ZEPHYR__
    ip.s_addr = addr->ipv4.address;
    gw.s_addr = addr->ipv4.gw;
    nm.s_addr = addr->ipv4.netmask;
#endif
    if (addr->ipv4.addr_type == ADDR_TYPE_STATIC)
    {
        (void)strcpy(addr_type, "STATIC");
    }
    else if (addr->ipv4.addr_type == ADDR_TYPE_STATIC)
    {
        (void)strcpy(addr_type, "AUTO IP");
    }
    else
    {
        (void)strcpy(addr_type, "DHCP");
    }

    (void)PRINTF("\r\n\tIPv4 Address\r\n");
    (void)PRINTF("\taddress: %s", addr_type);
#if defined(SDK_OS_FREE_RTOS)
    (void)PRINTF("\r\n\t\tIP:\t\t%s", inet_ntoa(ip));
    (void)PRINTF("\r\n\t\tgateway:\t%s", inet_ntoa(gw));
    (void)PRINTF("\r\n\t\tnetmask:\t%s", inet_ntoa(nm));
    (void)PRINTF("\r\n\t\tdns1:\t\t%s", inet_ntoa(dns1));
    (void)PRINTF("\r\n\t\tdns2:\t\t%s", inet_ntoa(dns2));
#elif defined(FSL_RTOS_THREADX)
    (void)PRINTF("\r\n\t\tIP:\t\t%d.%d.%d.%d", (addr->ipv4.address >> 24), (addr->ipv4.address >> 16 & 0xFF),
                 (addr->ipv4.address >> 8 & 0xFF), (addr->ipv4.address & 0xFF));
    (void)PRINTF("\r\n\t\tgateway:\t%d.%d.%d.%d", (addr->ipv4.gw >> 24), (addr->ipv4.gw >> 16 & 0xFF),
                 (addr->ipv4.gw >> 8 & 0xFF), (addr->ipv4.gw & 0xFF));
    (void)PRINTF("\r\n\t\tnetmask:\t%d.%d.%d.%d", (addr->ipv4.netmask >> 24), (addr->ipv4.netmask >> 16 & 0xFF),
                 (addr->ipv4.netmask >> 8 & 0xFF), (addr->ipv4.netmask & 0xFF));
//    (void)PRINTF("\r\n\t\tdns1:\t\t%s", inet_ntoa(dns1));
//    (void)PRINTF("\r\n\t\tdns2:\t\t%s", inet_ntoa(dns2));
#elif __ZEPHYR__
    (void)PRINTF("\r\n\t\tIP:\t\t%s", net_sprint_addr(AF_INET, &ip));
    (void)PRINTF("\r\n\t\tgateway:\t%s", net_sprint_addr(AF_INET, &gw));
    (void)PRINTF("\r\n\t\tnetmask:\t%s", net_sprint_addr(AF_INET, &nm));
#endif
    (void)PRINTF("\r\n");
out:

#if CONFIG_IPV6
    if (role == WLAN_BSS_ROLE_STA || role == WLAN_BSS_ROLE_UAP)
    {
        int i;
        (void)PRINTF("\r\n\tIPv6 Addresses\r\n");
#ifndef __ZEPHYR__
        for (i = 0; i < CONFIG_MAX_IPV6_ADDRESSES; i++)
        {
            if (addr->ipv6[i].addr_state != (unsigned char)IP6_ADDR_INVALID)
#else
        for (i = 0; i < CONFIG_MAX_IPV6_ADDRESSES && i < addr->ipv6_count; i++)
        {
            if ((addr->ipv6[i].addr_state == (unsigned char)NET_ADDR_TENTATIVE) ||
                    (addr->ipv6[i].addr_state == (unsigned char)NET_ADDR_PREFERRED))
#endif
            {
                (void)PRINTF("\t%-13s:\t%s (%s)\r\n", ipv6_addr_type_to_desc((struct net_ipv6_config *)&addr->ipv6[i]),
                             ipv6_addr_addr_to_desc((struct net_ipv6_config *)&addr->ipv6[i]),
                             ipv6_addr_state_to_desc(addr->ipv6[i].addr_state));
            }
        }
        (void)PRINTF("\r\n");
    }
#endif
    return;
}

static const char *print_role(enum wlan_bss_role role)
{
    if (role == WLAN_BSS_ROLE_STA)
    {
        return "Infra";
    }
    else if (role == WLAN_BSS_ROLE_UAP)
    {
        return "uAP";
    }
    else if (role == WLAN_BSS_ROLE_ANY)
    {
        return "any";
    }
    else
    {
        return "unknown";
    }
}

inline static const char *sec_tag(struct wlan_network *network)
{
    if (network->security_specific == 0U)
    {
        return "\tsecurity [Wildcard]";
    }
    else
    {
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
        if (is_ep_valid_security(network->security.type))
        {
            if (network->security.wpa3_sb_192)
            {
                return "\tsecurity: WPA3 SuiteB(192)";
            }
            else if (network->security.wpa3_sb)
            {
                return "\tsecurity: WPA3 SuiteB";
            }

            return "\tsecurity: WPA2";
        }
#endif
        return "\tsecurity";
    }
}
#if CONFIG_WIFI_CAPA
static int get_capa(char *arg, uint8_t *wlan_capa)
{
    if (!arg)
        return 1;
#if CONFIG_11AX
    if (string_equal(arg, "11ax") != 0)
    {
        *wlan_capa = (WIFI_SUPPORT_11AX | WIFI_SUPPORT_11AC | WIFI_SUPPORT_11N | WIFI_SUPPORT_LEGACY);
        return 0;
    }
    else
#endif
#if CONFIG_11AC
        if (string_equal(arg, "11ac") != 0)
        {
            *wlan_capa = (WIFI_SUPPORT_11AC | WIFI_SUPPORT_11N | WIFI_SUPPORT_LEGACY);
            return 0;
        }
        else
#endif
            if (string_equal(arg, "11n") != 0)
            {
                *wlan_capa = (WIFI_SUPPORT_11N | WIFI_SUPPORT_LEGACY);
                return 0;
            }
            else if (string_equal(arg, "legacy") != 0)
            {
                *wlan_capa = WIFI_SUPPORT_LEGACY;
                return 0;
            }
            else
                return 1;
}
#endif

static void print_network(struct wlan_network *network)
{
    // #if SDK_DEBUGCONSOLE != DEBUGCONSOLE_DISABLE
    (void)PRINTF("\"%s\"\r\n\tSSID: %s\r\n\tBSSID: ", network->name,
                 network->ssid[0] != '\0' ? network->ssid : "(hidden)");
    print_mac(network->bssid);

    if ((network->dot11n != 0U)
#if CONFIG_11AC
            || (network->dot11ac != 0U)
#endif
#if CONFIG_11AX
            || (network->dot11ax != 0U)
#endif
       )
    {
        (void)PRINTF("\r\n\tmode: ");
#if CONFIG_11AC
#if CONFIG_11AX
        if (network->dot11ax != 0U)
        {
            (void)PRINTF("802.11AX ");
        }
        else
#endif
            if (network->dot11ac != 0U)
            {
                (void)PRINTF("802.11AC ");
            }
            else
#endif
                if (network->dot11n != 0U)
                {
                    (void)PRINTF("802.11N ");
                }
                else
                {
                    (void)PRINTF("802.11BG ");
                }
    }

    if (network->channel != 0U)
    {
        (void)PRINTF("\r\n\tchannel: %d", network->channel);
    }
    else
    {
        (void)PRINTF("\r\n\tchannel: %s", "(Auto)");
    }
    (void)PRINTF("\r\n\trole: %s\r\n", print_role(network->role));

    if (network->role == WLAN_BSS_ROLE_STA)
    {
        (void)PRINTF("\r\n\tRSSI: %ddBm\r\n", network->rssi);
    }

    switch (network->security.type)
    {
    case WLAN_SECURITY_NONE:
        (void)PRINTF("%s: none\r\n", sec_tag(network));
        break;
    case WLAN_SECURITY_WEP_OPEN:
        (void)PRINTF("%s: WEP (open)\r\n", sec_tag(network));
        break;
    case WLAN_SECURITY_WEP_SHARED:
        (void)PRINTF("%s: WEP (shared)\r\n", sec_tag(network));
        break;
    case WLAN_SECURITY_WPA:
        (void)PRINTF("%s: WPA\r\n", sec_tag(network));
        break;
    case WLAN_SECURITY_WPA2:
#if (CONFIG_11R)
        if (network->ft_psk == 1U)
        {
            (void)PRINTF("%s: WPA2-FT-PSK", sec_tag(network));
        }
        else
#endif
        {
            (void)PRINTF("%s: WPA2", sec_tag(network));
        }
        (void)PRINTF("\r\n");
        break;
#if CONFIG_11R
    case WLAN_SECURITY_WPA2_FT:
        (void)PRINTF("%s: WPA2-FT-PSK\r\n", sec_tag(network));
        break;
#endif
    case WLAN_SECURITY_WPA_WPA2_MIXED:
        (void)PRINTF("%s: WPA/WPA2 Mixed\r\n", sec_tag(network));
        break;
#if (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE) || (CONFIG_WPA2_ENTP)
#if CONFIG_EAP_TLS
    case WLAN_SECURITY_EAP_TLS:
        (void)PRINTF("%s Enterprise EAP-TLS\r\n", sec_tag(network));
        break;
#endif
#endif
#if CONFIG_PEAP_MSCHAPV2
    case WLAN_SECURITY_EAP_PEAP_MSCHAPV2:
        (void)PRINTF("%s: WPA2 Enterprise PEAP-MSCHAPV2\r\n", sec_tag(network));
        break;
#endif
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
    case WLAN_SECURITY_EAP_TLS_SHA256:
        (void)PRINTF("%s Enterprise EAP-TLS-SHA256\r\n", sec_tag(network));
        break;
#if CONFIG_11R
    case WLAN_SECURITY_EAP_TLS_FT:
        (void)PRINTF("%s Enterprise EAP-TLS-FT\r\n", sec_tag(network));
        break;
    case WLAN_SECURITY_EAP_TLS_FT_SHA384:
        (void)PRINTF("%s Enterprise EAP-TLS-FT-SHA384\r\n", sec_tag(network));
        break;
#endif
#endif
#if CONFIG_EAP_TTLS
    case WLAN_SECURITY_EAP_TTLS:
        (void)PRINTF("%s Enterprise EAP-TTLSv%d\r\n", sec_tag(network), network->security.eap_ver);
        break;
#if CONFIG_EAP_MSCHAPV2
    case WLAN_SECURITY_EAP_TTLS_MSCHAPV2:
        (void)PRINTF("%s Enterprise EAP-TTLSv%d-MSCHAPV2\r\n", sec_tag(network), network->security.eap_ver);
        break;
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
    case WLAN_SECURITY_EAP_PEAP_MSCHAPV2:
        (void)PRINTF("%s Enterprise EAP-PEAPv%d-MSCHAPV2\r\n", sec_tag(network), network->security.eap_ver);
        break;
#endif
#if CONFIG_EAP_TLS
    case WLAN_SECURITY_EAP_PEAP_TLS:
        (void)PRINTF("%s Enterprise EAP-PEAPv%d-TLS\r\n", sec_tag(network), network->security.eap_ver);
        break;
#endif
#if CONFIG_EAP_GTC
    case WLAN_SECURITY_EAP_PEAP_GTC:
        (void)PRINTF("%s Enterprise EAP-PEAPv%d-GTC\r\n", sec_tag(network), network->security.eap_ver);
        break;
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
    case WLAN_SECURITY_EAP_FAST_MSCHAPV2:
        (void)PRINTF("%s Enterprise EAP-FAST-MSCHAPV2\r\n", sec_tag(network));
        break;
#endif
#if CONFIG_EAP_GTC
    case WLAN_SECURITY_EAP_FAST_GTC:
        (void)PRINTF("%s Enterprise EAP-FAST-GTC\r\n", sec_tag(network));
        break;
#endif
#endif
#if CONFIG_EAP_SIM
    case WLAN_SECURITY_EAP_SIM:
        (void)PRINTF("%s Enterprise EAP-SIM\r\n", sec_tag(network));
        break;
#endif
#if CONFIG_EAP_AKA
    case WLAN_SECURITY_EAP_AKA:
        (void)PRINTF("%s Enterprise EAP-AKA\r\n", sec_tag(network));
        break;
#endif
#if CONFIG_EAP_AKA_PRIME
    case WLAN_SECURITY_EAP_AKA_PRIME:
        (void)PRINTF("%s Enterprise EAP-AKA-PRIME\r\n", sec_tag(network));
        break;
#endif
#endif
#if CONFIG_DRIVER_OWE
    case WLAN_SECURITY_OWE_ONLY:
        (void)PRINTF("%s: OWE Only\r\n", sec_tag(network));
        break;
#endif
    case WLAN_SECURITY_WPA3_SAE:
#if (CONFIG_11R)
        if (network->ft_sae == 1U)
        {
            (void)PRINTF("%s: WPA3-FT-SAE", sec_tag(network));
        }
        else
#endif
        {
            (void)PRINTF("%s: WPA3-SAE", sec_tag(network));
        }
        (void)PRINTF("\r\n");
        break;
    case WLAN_SECURITY_WPA3_SAE_EXT_KEY:
        (void)PRINTF("%s: WPA3-SAE-EXT-KEY\r\n", sec_tag(network));
        break;
#if CONFIG_WPA_SUPP
#if CONFIG_11R
    case WLAN_SECURITY_WPA3_FT_SAE:
        (void)PRINTF("%s: WPA3-FT-SAE\r\n", sec_tag(network));
        break;
#endif
#endif
    case WLAN_SECURITY_WPA2_WPA3_SAE_MIXED:
        (void)PRINTF("%s: WPA2/WPA3 SAE Mixed\r\n", sec_tag(network));
        break;
    default:
        (void)PRINTF("\r\nUnexpected WLAN SECURITY\r\n");
        break;
    }
#if CONFIG_WIFI_CAPA
    if (network->role == WLAN_BSS_ROLE_UAP)
    {
#if CONFIG_11AX
        uint8_t enable_11ax = false;
#endif
#if CONFIG_11AC
        uint8_t enable_11ac = false;
#endif
        uint8_t enable_11n = false;

#if CONFIG_11AX
        enable_11ax = wlan_check_11ax_capa(network->channel);
#endif
#if CONFIG_11AC
        enable_11ac = wlan_check_11ac_capa(network->channel);
#endif
        enable_11n = wlan_check_11n_capa(network->channel);
#if CONFIG_11AX
        if (network->wlan_capa & WIFI_SUPPORT_11AX)
        {
            if (!enable_11ax)
            {
#if CONFIG_11AC
                if (enable_11ac)
                {
                    (void)PRINTF("\twifi capability: 11ac\r\n");
                }
                else
#endif
                {
                    (void)PRINTF("\twifi capability: 11n\r\n");
                }
            }
            else
            {
                (void)PRINTF("\twifi capability: 11ax\r\n");
            }

            (void)PRINTF("\tuser configure: 11ax\r\n");
        }
        else
#endif
#if CONFIG_11AC
            if (network->wlan_capa & WIFI_SUPPORT_11AC)
            {
                if (!enable_11ac)
                {
                    (void)PRINTF("\twifi capability: 11n\r\n");
                }
                else
                {
                    (void)PRINTF("\twifi capability: 11ac\r\n");
                }

                (void)PRINTF("\tuser configure: 11ac\r\n");
            }
            else
#endif
                if (network->wlan_capa & WIFI_SUPPORT_11N)
                {
                    if (!enable_11n)
                    {
                        (void)PRINTF("\twifi capability: legacy\r\n");
                    }
                    else
                    {
                        (void)PRINTF("\twifi capability: 11n\r\n");
                    }

                    (void)PRINTF("\tuser configure: 11n\r\n");
                }
                else
                {
                    (void)PRINTF("\twifi capability: legacy\r\n");
                    (void)PRINTF("\tuser configure: legacy\r\n");
                }
    }
#endif
#if CONFIG_WPA_SUPP
    if (network->role == WLAN_BSS_ROLE_STA)
    {
        (void)PRINTF("\tProactive Key Caching: %s\r\n", network->security.pkc == 1 ? "Enabled" : "Disabled");
    }
#endif
    print_address(&network->ip, network->role);
#if CONFIG_SCAN_WITH_RSSIFILTER
    (void)PRINTF("\r\n\trssi threshold: %d \r\n", network->rssi_threshold);
#endif
}

/* Parse the 'arg' string as "ip:ipaddr,gwaddr,netmask,[dns1,dns2]" into
 * a wlan_ip_config data structure */
static int get_address(char *arg, struct wlan_ip_config *ip)
{
    char *ipaddr = NULL, *gwaddr = NULL, *netmask = NULL;
    char *dns1 = NULL, *dns2 = NULL;

    ipaddr = strstr(arg, "ip:");
    if (ipaddr == NULL)
    {
        return -1;
    }
    ipaddr += 3;

    gwaddr = strstr(ipaddr, ",");
    if (gwaddr == NULL)
    {
        return -1;
    }
    *gwaddr++ = (char)0;

    netmask = strstr(gwaddr, ",");
    if (netmask == NULL)
    {
        return -1;
    }
    *netmask++ = (char)0;

    dns1 = strstr(netmask, ",");
    if (dns1 != NULL)
    {
        *dns1++ = (char)0;
        dns2    = strstr(dns1, ",");
    }
    ip->ipv4.address = net_inet_aton(ipaddr);
    ip->ipv4.gw      = net_inet_aton(gwaddr);
    ip->ipv4.netmask = net_inet_aton(netmask);

    if (dns1 != NULL)
    {
        ip->ipv4.dns1 = net_inet_aton(dns1);
    }

    if (dns2 != NULL)
    {
        ip->ipv4.dns2 = net_inet_aton(dns2);
    }

    return 0;
}

static int get_security(int argc, char **argv, struct wlan_network_security *sec)
{
    int ret = WM_SUCCESS;

    if (argc < 1)
    {
        return -WM_FAIL;
    }

    /* copy the PSK phrase */
    sec->psk_len = (uint8_t)strlen(argv[0]);
    if (sec->psk_len < WLAN_PSK_MIN_LENGTH)
    {
        return -WM_FAIL;
    }

    if (sec->psk_len < sizeof(sec->psk))
    {
        (void)strcpy(sec->psk, argv[0]);
    }
    else
    {
        return -WM_FAIL;
    }

    return ret;
}

static bool get_role(char *arg, enum wlan_bss_role *role)
{
    if (arg == NULL)
    {
        return true;
    }

    if (string_equal(arg, "sta") != false)
    {
        *role = WLAN_BSS_ROLE_STA;
        return false;
    }
    else if (string_equal(arg, "uap") != false)
    {
        *role = WLAN_BSS_ROLE_UAP;
        return false;
    }
    else
    {
        return true;
    }
}

/*
 * MTF Shell Commands
 */
static void dump_wlan_add_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("For Station interface\r\n");
    (void)PRINTF("  For DHCP IP Address assignment:\r\n");
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> [wpa2 <psk/psk-sha256"
#if CONFIG_11R
        "/ft-psk"
#endif
        "> <secret>] [mfpc <1> mfpr <0>] "
        "\r\n");
    (void)PRINTF("      If using WPA2 security, set the PMF configuration as mentioned above.\r\n");
#if CONFIG_WPA_SUPP
    (void)PRINTF(
        "If using proactive key caching set pkc as 1, to disable set to 0(default), if okc is set this is not "
        "used.\r\n");
    (void)PRINTF("If using specific ciphers, set the group, pairwise and group mgmt using gc, pc and gmc options.\r\n");
    (void)PRINTF(
        "supported ciphers: ccmp=0x10"
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
        ", gcmp=0x40, gcmp_256=0x100, ccmp_256=0x200"
#endif
        "\r\n");
    (void)PRINTF(
        "supported group mgmt ciphers: aes_128_cmac=0x20"
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
        ", bip_gmac_128=0x800, bip_gmac_256=0x1000, "
        "bip_cmac_256=0x2000"
#endif
        "\r\n");
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> [wpa3-sb/wpa3-sb-192] ["
#if CONFIG_EAP_TLS
        "eap-tls/eap-tls-sha256"
#if CONFIG_11R
        "/eap-tls-ft/eap-tls-ft-sha384"
#endif
#endif
#if CONFIG_EAP_TLS
        " [tls_cipher <ECC_P384/RSA_3K>]"
#endif
        " id <identity> "
        "[key_passwd "
        "<client_key_passwd>][hash <hash>][domain_match <domain_match_string>][domain_suffix_match "
        "<domain_suffix_match_string>]] "
        "[mfpc <1> mfpr <0/1>] [mc 0x10 uc 0x10 gc 0x20]"
        "\r\n");
#if CONFIG_EAP_TTLS
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> [wpa3-sb/wpa3-sb-192] [eap-ttls aid <anonymous identity> [key2_passwd "
        "<client_key2_passwd>]] [mfpc <1> mfpr <0/1>]"
        "\r\n");
#if CONFIG_EAP_MSCHAPV2
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> [wpa3-sb/wpa3-sb-192] [eap-ttls-mschapv2 aid <anonymous identity> id "
        "<identity> pass "
        "<password> [key_passwd <client_key_passwd>]] [mfpc <1> mfpr <0/1>]"
        "\r\n");
#endif
#endif
#if CONFIG_EAP_PEAP
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> [wpa3-sb/wpa3-sb-192] ["
#if CONFIG_EAP_MSCHAPV2
        "eap-peap-mschapv2"
#endif
#if CONFIG_EAP_TLS
        "/eap-peap-tls"
#endif
#if CONFIG_EAP_GTC
        "/eap-peap-gtc"
#endif
        " [ver 0/1] id <identity> pass "
        "<password> [key_passwd <client_key_passwd>]] [mfpc <1> mfpr <0/1>]"
        "\r\n");
#endif
#if CONFIG_EAP_FAST
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> [wpa3-sb/wpa3-sb-192] ["
#if CONFIG_EAP_MSCHAPV2
        "eap-fast-mschapv2"
#endif
#if CONFIG_EAP_GTC
        "/eap-fast-gtc"
#endif
        " aid <anonymous "
        "identity> id <identity> pass "
        "<password> [key_passwd <client_key_passwd>]] [mfpc <1> mfpr <0/1>]"
        "\r\n");
#endif
#if (CONFIG_EAP_SIM) || (CONFIG_EAP_AKA) || (CONFIG_EAP_AKA_PRIME)
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> ["
#if CONFIG_EAP_SIM
        "eap-sim"
#endif
#if CONFIG_EAP_AKA
        "/eap-aka"
#endif
#if CONFIG_EAP_AKA_PRIME
        "/eap-aka-prime"
#endif
        " id <identity> pass <password>]"
        "\r\n");
#endif
    (void)PRINTF("      If using WPA2/WPA3 Enterprise security, set the PMF configuration as required.\r\n");
#endif
#endif
#if CONFIG_DRIVER_OWE
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> <owe_only> "
#if CONFIG_WPA_SUPP
        "[og <\"19 20 21\">] "
#endif
        "mfpc 1 mfpr 1"
        "\r\n");
    (void)PRINTF("      If using OWE only security, always set the PMF configuration.\r\n");
#endif
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> [wpa3 sae"
#if CONFIG_WPA_SUPP
#if CONFIG_11R
        "/ft-sae"
#endif
#endif
        " <secret> "
#if CONFIG_WPA_SUPP
        "[sg <\"19 20 21\">] "
#endif
        "[pwe <0/1/2>] mfpc <1> mfpr <0/1>]"
        "\r\n");
    (void)PRINTF("      If using WPA3 SAE security, always set the PMF configuration.\r\n");
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid> [wpa2 psk psk-sha256 <secret> wpa3 sae <secret>] [mfpc <1> mfpr <0>] "
        "\r\n");
    (void)PRINTF("      If using WPA2/WPA3 Mixed security, set the PMF configuration as mentioned above.\r\n");
    (void)PRINTF("  For static IP address assignment:\r\n");
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid>\r\n"
        "    ip:<ip_addr>,<gateway_ip>,<netmask>\r\n");
    (void)PRINTF(
        "    [bssid <bssid>] [channel <channel number>]\r\n"
        "    [wpa2 <psk/psk-sha256"
#if CONFIG_11R
        "/ft-psk"
#endif
        "> <secret>]"
#if (CONFIG_WPA2_ENTP) || (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE)
        " [wpa3-sb/wpa3-sb-192] "
#if CONFIG_EAP_TLS
        "[eap-tls/eap-tls-sha256"
#if CONFIG_11R
        "/eap-tls-ft/eap-tls-ft-sha384"
#endif
#endif
        "]"
#endif
#if CONFIG_DRIVER_OWE
        " [owe_only]"
#endif
        " [wpa3 sae"
#if CONFIG_WPA_SUPP
#if CONFIG_11R
        "/ft-sae"
#endif
#endif
        " <secret>]"
        " [mfpc <0/1> mfpr <0/1>]"
        "\r\n");

    (void)PRINTF("For Micro-AP interface\r\n");
    (void)PRINTF(
        "    wlan-add <profile_name> ssid <ssid>\r\n"
        "    ip:<ip_addr>,<gateway_ip>,<netmask>\r\n");
    (void)PRINTF(
        "    role uap [bssid <bssid>]\r\n"
        "    [channel <channelnumber>]\r\n");
    (void)PRINTF(
        "    [wpa2 <psk/psk-sha256> <secret>] [wpa3 sae <secret> "
#if CONFIG_WPA_SUPP
        "[sg <\"19 20 21\">] "
#endif
        "[pwe <0/1/2>] [tr <0/1"
#if CONFIG_WPA_SUPP
        "/2/4/8"
#endif
        ">]]\r\n");
#if CONFIG_WPA_SUPP
#if CONFIG_HOSTAPD
#if CONFIG_11R
    (void)PRINTF("    [ft-psk <secret>] [wpa3 ft-sae <secret>]\r\n");
#endif
#if (CONFIG_WPA2_ENTP) || (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE)
    (void)PRINTF(
        "    [wpa3-sb/wpa3-sb-192] "
        "["
#if CONFIG_EAP_TLS
        "eap-tls/eap-tls-sha256"
#endif
#if CONFIG_EAP_TTLS
        "/eap-ttls"
#if CONFIG_EAP_MSCHAPV2
        "/eap-ttls-mschapv2"
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
        "/eap-peap-mschapv2"
#endif
#if CONFIG_EAP_TLS
        "/eap-peap-tls"
#endif
#if CONFIG_EAP_GTC
        "/eap-peap-gtc"
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
        "/eap-fast-mschapv2"
#endif
#if CONFIG_EAP_GTC
        "/eap-fast-gtc"
#endif
#endif
#if CONFIG_EAP_SIM
        "/eap-sim"
#endif
#if CONFIG_EAP_AKA
        "/eap-aka"
#endif
#if CONFIG_EAP_AKA_PRIME
        "/eap-aka-prime"
#endif
        "]\r\n");
#if CONFIG_11R
    (void)PRINTF("    [eap-tls-ft/eap-tls-ft-sha384]\r\n");
#endif
#endif
#endif
#endif
#if CONFIG_DRIVER_OWE
    (void)PRINTF(
        "    [owe_only "
#if CONFIG_WPA_SUPP
        "[og <\"19 20 21\">]"
#endif
        "]\r\n");
#endif
    (void)PRINTF("    [mfpc <0/1>] [mfpr <0/1>]\r\n");
#if (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE)
#if (CONFIG_EAP_SIM) || (CONFIG_EAP_AKA) || (CONFIG_EAP_AKA_PRIME)
    (void)PRINTF(
        "    If using eap-sim/eap-aka/eap-aka-prime use read_gsm_triplets to add GSM authentication triplets and "
        "read_milenage to add Milenage keys and hlr_cli to start hlr_auc_gw\r\n");
#endif
#endif
#if CONFIG_WIFI_DTIM_PERIOD
    (void)PRINTF("If setting dtim\r\n");
    (void)PRINTF("The value of dtim is an integer. The default value is 10.\r\n");
#endif
#if CONFIG_11AC
    (void)PRINTF(
        "Note: Setting the channel value greater than or equal to 36 is mandatory,\r\n"
        "      if UAP bandwidth is set to 80MHz.\r\n");
#endif
#if CONFIG_WIFI_CAPA
    (void)PRINTF("\r\n");
#if CONFIG_11AX
    (void)PRINTF("    [capa <11ax/11ac/11n/legacy>]\r\n");
#elif CONFIG_11AC
    (void)PRINTF("    [capa <11ac/11n/legacy>]\r\n");
#else
    (void)PRINTF("    [capa <11n/legacy>]\r\n");
#endif
#endif
    (void)PRINTF("If Set channel to 0, set acs_band to 0 1.\r\n");
    (void)PRINTF("0: 2.4GHz channel   1: 5GHz channel  Not support to select dual band automatically.\r\n");
}

static void test_wlan_add(int argc, char **argv)
{
    struct wlan_network network;
    int ret    = 0;
    int arg    = 1;
    size_t len = 0U;
    struct
    {
        unsigned ssid : 1;
        unsigned bssid : 1;
        unsigned channel : 1;
        unsigned address : 2;
        unsigned security : 1;
        unsigned security2 : 1;
        unsigned security3 : 1;
        unsigned role : 1;
        unsigned mfpc : 1;
        unsigned mfpr : 1;
#if CONFIG_WPA_SUPP
        unsigned pkc : 1;
        unsigned gcipher : 1;
        unsigned pcipher : 1;
        unsigned gmcipher : 1;
#endif
#if CONFIG_WIFI_DTIM_PERIOD
        unsigned dtim : 1;
#endif
#if CONFIG_WIFI_CAPA
        unsigned wlan_capa : 1;
#endif
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
        unsigned wpa3_sb : 1;
#endif
        unsigned acs_band : 1;
    } info;

    (void)memset(&info, 0, sizeof(info));
    (void)memset(&network, 0, sizeof(struct wlan_network));

    if (argc < 4)
    {
        dump_wlan_add_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    len = strlen(argv[arg]);
    if (len >= WLAN_NETWORK_NAME_MAX_LENGTH)
    {
        (void)PRINTF("Error: network name too long\r\n");
        return;
    }

    (void)memcpy(network.name, argv[arg], len);
    arg++;
    info.address = (uint8_t)ADDR_TYPE_DHCP;

    /* To set default values for mfpc and mfpr following settings are  necessary.
     * Later on in wlan_add_network these values will be compared to set default values (mfpc 1 and mfpr 0).
     * If mfpc and mfpr values are mentioned via command line, those will be considered for non sae conditions */
    network.security.mfpc = -1;
    network.security.mfpr = -1;

    do
    {
        if ((info.ssid == 0U) && string_equal("ssid", argv[arg]))
        {
            len = strlen(argv[arg + 1]);
            if (len > IEEEtypes_SSID_SIZE)
            {
                (void)PRINTF("Error: SSID is too long\r\n");
                return;
            }
            (void)memcpy(network.ssid, argv[arg + 1], len);
            arg += 2;
            info.ssid = 1;
        }
        else if ((info.bssid == 0U) && string_equal("bssid", argv[arg]))
        {
            if (get_mac(argv[arg + 1], network.bssid, ':') != false)
            {
                (void)PRINTF(
                    "Error: invalid BSSID argument"
                    "\r\n");
                return;
            }
            arg += 2;
            info.bssid = 1;
        }
        else if ((info.channel == 0U) && string_equal("channel", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &network.channel, strlen(argv[arg + 1])))
            {
                (void)PRINTF(
                    "Error: invalid channel"
                    " argument\n");
                return;
            }
            arg += 2;
            info.channel = 1;
        }
        else if (strncmp(argv[arg], "ip:", 3) == 0)
        {
            if (get_address(argv[arg], &network.ip) != 0)
            {
                (void)PRINTF(
                    "Error: invalid address"
                    " argument\n");
                return;
            }
            arg++;
            info.address = (uint8_t)ADDR_TYPE_STATIC;
        }
        else if ((info.security == 0U) && string_equal("wpa", argv[arg]))
        {
            arg += 1;

            if (string_equal(argv[arg], "wpa2") != false)
            {
                network.security.type = WLAN_SECURITY_WPA_WPA2_MIXED;
                arg += 1;

                if (string_equal(argv[arg], "psk") != false)
                {
                    network.security.key_mgmt |= WLAN_KEY_MGMT_PSK;
                    arg += 1;
                }
                else
                {
                    (void)PRINTF(
                        "Error: invalid WPA WPA2 security"
                        " argument\r\n");
                    return;
                }
            }
            else
            {
                network.security.type = WLAN_SECURITY_WPA;

                if (string_equal(argv[arg], "psk") != false)
                {
                    network.security.key_mgmt |= WLAN_KEY_MGMT_PSK;
                    arg += 1;
                }
                else
                {
                    (void)PRINTF(
                        "Error: invalid WPA security"
                        " argument\r\n");
                    return;
                }
            }

            if (get_security(argc - arg, argv + arg, &network.security) != 0)
            {
                (void)PRINTF(
                    "Error: invalid WPA security"
                    " argument\r\n");
                return;
            }
            arg += 1;
            info.security++;
        }
        else if ((info.security2 == 0U) && (string_equal("wpa2", argv[arg])))
        {
            arg += 1;

            if (string_equal(argv[arg], "wpa") != false)
            {
                network.security.type = WLAN_SECURITY_WPA_WPA2_MIXED;

                arg += 1;

                if (string_equal(argv[arg], "psk") != false)
                {
                    network.security.key_mgmt |= WLAN_KEY_MGMT_PSK;
                    arg += 1;
                }
                else
                {
                    (void)PRINTF(
                        "Error: invalid WPA2 WPA security"
                        " argument\r\n");
                    return;
                }
            }
            else
            {
                network.security.type = WLAN_SECURITY_WPA2;

                if (string_equal(argv[arg], "psk") != false)
                {
                    network.security.key_mgmt |= WLAN_KEY_MGMT_PSK;
                    arg += 1;

#if CONFIG_11R
                    if (string_equal(argv[arg], "ft-psk") != false)
                    {
                        network.security.key_mgmt |= WLAN_KEY_MGMT_FT_PSK;
                        arg += 1;
                    }
#endif

                    if (string_equal(argv[arg], "psk-sha256") != false)
                    {
                        network.security.key_mgmt |= WLAN_KEY_MGMT_PSK_SHA256;
                        arg += 1;
                    }
                }
                else if (string_equal(argv[arg], "psk-sha256") != false)
                {
                    network.security.key_mgmt = WLAN_KEY_MGMT_PSK_SHA256;
                    arg += 1;
                }
#if CONFIG_11R
                else if (string_equal(argv[arg], "ft-psk") != false)
                {
                    network.security.type     = WLAN_SECURITY_WPA2_FT;
                    network.security.key_mgmt = WLAN_KEY_MGMT_FT_PSK;

                    arg += 1;
                }
#endif
                else
                {
                    (void)PRINTF(
                        "Error: invalid WPA2 security"
                        " argument\r\n");
                    return;
                }
            }

            if (get_security(argc - arg, argv + arg, &network.security) != 0)
            {
                (void)PRINTF(
                    "Error: invalid WPA2 security"
                    " argument\r\n");
                return;
            }
            arg += 1;
            info.security2++;
        }
#if CONFIG_DRIVER_OWE
        else if (!info.security && string_equal("owe_only", argv[arg]))
        {
            network.security.type = WLAN_SECURITY_OWE_ONLY;

            network.security.key_mgmt = WLAN_KEY_MGMT_OWE;

            arg += 1;
            info.security++;

#if CONFIG_WPA_SUPP
            if (string_equal(argv[arg], "og") != false)
            {
                network.security.owe_groups = string_dup(argv[arg + 1]);
                arg += 2;
            }
#endif
        }
#endif
        else if ((info.security3 == 0U) && string_equal("wpa3", argv[arg]))
        {
            if ((string_equal(argv[arg + 1], "sae") != false) || (string_equal(argv[arg + 1], "ft-sae") != false) ||
                    (string_equal(argv[arg + 1], "sae-ext-key") != false))
            {
                arg += 1;

#if !CONFIG_WPA_SUPP
                if (string_equal(argv[arg], "ft-sae") != false)
                {
                    (void)PRINTF("Error: WPA3 FT-SAE not supported\r\n");
                    return;
                }
#endif

                if (string_equal(argv[arg], "sae") != false)
                {
                    network.security.type = WLAN_SECURITY_WPA3_SAE;
                    network.security.key_mgmt |= WLAN_KEY_MGMT_SAE;
                    arg += 1;

                    if (string_equal(argv[arg], "sae-ext-key") != false)
                    {
                        network.security.key_mgmt |= WLAN_KEY_MGMT_SAE_EXT_KEY;
                        arg += 1;
                    }
#if CONFIG_WPA_SUPP
#if CONFIG_11R
                    else if (string_equal(argv[arg], "ft-sae") != false)
                    {
                        network.security.key_mgmt |= WLAN_KEY_MGMT_FT_SAE;
                        arg += 1;
                    }
#endif
#endif
                }
#if CONFIG_WPA_SUPP
#if CONFIG_11R
                else if (string_equal(argv[arg], "ft-sae") != false)
                {
                    network.security.type = WLAN_SECURITY_WPA3_FT_SAE;
                    network.security.key_mgmt |= WLAN_KEY_MGMT_FT_SAE;
                    arg += 1;

                    if (string_equal(argv[arg], "sae") != false)
                    {
                        network.security.key_mgmt |= WLAN_KEY_MGMT_SAE;
                        arg += 1;
                    }
                }
#endif
#endif
                else if (string_equal(argv[arg], "sae-ext-key") != false)
                {
                    network.security.type = WLAN_SECURITY_WPA3_SAE_EXT_KEY;
                    network.security.key_mgmt |= WLAN_KEY_MGMT_SAE_EXT_KEY;
                    arg += 1;

                    if (string_equal(argv[arg], "sae") != false)
                    {
                        network.security.key_mgmt |= WLAN_KEY_MGMT_SAE;
                        arg += 1;
                    }
                }
                /* copy the SAE password */
                network.security.password_len = strlen(argv[arg]);
                if (network.security.password_len == 0U)
                {
                    (void)PRINTF(
                        "Error: invalid WPA3 security"
                        " argument\r\n");
                    return;
                }
                if (network.security.password_len < sizeof(network.security.password))
                {
                    (void)strcpy(network.security.password, argv[arg]);
                }
                else
                {
                    (void)PRINTF(
                        "Error: invalid WPA3 security"
                        " argument\r\n");
                    return;
                }
                arg += 1;

#if CONFIG_WPA_SUPP
                if (string_equal(argv[arg], "sg") != false)
                {
                    network.security.sae_groups = string_dup(argv[arg + 1]);
                    arg += 2;
                }
#endif
                if (string_equal(argv[arg], "pwe") != false)
                {
                    errno                           = 0;
                    network.security.pwe_derivation = strtol(argv[arg + 1], NULL, 10);
                    if (errno != 0)
                    {
                        (void)PRINTF("Error during strtol:pwe errno:%d\r\n", errno);
                        return;
                    }
                    if (arg + 1 >= argc ||
                            (network.security.pwe_derivation != 0 && network.security.pwe_derivation != 1 &&
                             network.security.pwe_derivation != 2))
                    {
                        (void)PRINTF(
                            "Error: invalid wireless"
                            " network pwe derivation\r\n");
                        return;
                    }
                    arg += 2;

                    if (string_equal(argv[arg], "tr") != false)
                    {
                        errno                               = 0;
                        network.security.transition_disable = strtol(argv[arg + 1], NULL, 10);
                        if (errno != 0)
                        {
                            (void)PRINTF("Error during strtol:pwe errno:%d\r\n", errno);
                            return;
                        }
                        if (arg + 1 >= argc ||
                                (network.security.transition_disable != 0 && network.security.transition_disable != 1
#if CONFIG_WPA_SUPP
                                 && network.security.transition_disable != 2 && network.security.transition_disable != 4 &&
                                 network.security.transition_disable != 8
#endif
                                ))
                        {
                            (void)PRINTF(
                                "Error: invalid wireless"
                                " network transition state\r\n");
                            return;
                        }
                        arg += 2;
                    }
                }
                else
                {
                    if (network.role == WLAN_BSS_ROLE_STA)
                    {
                        network.security.pwe_derivation = 2;
                    }
                }
            }
            else
            {
                (void)PRINTF(
                    "Error: invalid WPA3 security"
                    " argument\r\n");
                return;
            }
            info.security3++;
        }
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
        else if ((info.wpa3_sb == 0U) && string_equal("wpa3-sb", argv[arg]))
        {
            network.security.wpa3_sb = 1;
            arg += 1;
            info.wpa3_sb = 1;
        }
        else if ((info.wpa3_sb == 0U) && string_equal("wpa3-sb-192", argv[arg]))
        {
            network.security.wpa3_sb_192 = 1;
            arg += 1;
            info.wpa3_sb = 1;
        }
        else if ((info.security2 == 0U) &&
                 (
#if CONFIG_EAP_TLS
                     string_equal("eap-tls", argv[arg]) || string_equal("eap-tls-sha256", argv[arg]) ||
#if CONFIG_11R
                     string_equal("eap-tls-ft", argv[arg]) || string_equal("eap-tls-ft-sha384", argv[arg]) ||
#endif
#endif
#if CONFIG_EAP_TTLS
                     string_equal("eap-ttls", argv[arg]) ||
#if CONFIG_EAP_MSCHAPV2
                     string_equal("eap-ttls-mschapv2", argv[arg]) ||
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
                     string_equal("eap-peap-mschapv2", argv[arg]) ||
#endif
#if CONFIG_EAP_TLS
                     string_equal("eap-peap-tls", argv[arg]) ||
#endif
#if CONFIG_EAP_GTC
                     string_equal("eap-peap-gtc", argv[arg]) ||
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
                     string_equal("eap-fast-mschapv2", argv[arg]) ||
#endif
#if CONFIG_EAP_GTC
                     string_equal("eap-fast-gtc", argv[arg]) ||
#endif
#endif
#if CONFIG_EAP_SIM
                     string_equal("eap-sim", argv[arg]) ||
#endif
#if CONFIG_EAP_AKA
                     string_equal("eap-aka", argv[arg]) ||
#endif
#if CONFIG_EAP_AKA_PRIME
                     string_equal("eap-aka-prime", argv[arg]) ||
#endif
                     false))
        {
#if CONFIG_EAP_TLS
            if (string_equal("eap-tls", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_TLS;
            }
            if (string_equal("eap-tls-sha256", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_TLS_SHA256;
            }
#if CONFIG_11R
            if (string_equal("eap-tls-ft", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_TLS_FT;
            }
            if (string_equal("eap-tls-ft-sha384", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_TLS_FT_SHA384;
            }
#endif
#endif
#if CONFIG_EAP_TTLS
            if (string_equal("eap-ttls", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_TTLS;
            }
#if CONFIG_EAP_MSCHAPV2
            if (string_equal("eap-ttls-mschapv2", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_TTLS_MSCHAPV2;
            }
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
            if (string_equal("eap-peap-mschapv2", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_PEAP_MSCHAPV2;
            }
#endif
#if CONFIG_EAP_TLS
            if (string_equal("eap-peap-tls", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_PEAP_TLS;
            }
#endif
#if CONFIG_EAP_GTC
            if (string_equal("eap-peap-gtc", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_PEAP_GTC;
            }
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
            if (string_equal("eap-fast-mschapv2", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_FAST_MSCHAPV2;
            }
#endif
#if CONFIG_EAP_GTC
            if (string_equal("eap-fast-gtc", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_FAST_GTC;
            }
#endif
#endif
#if CONFIG_EAP_SIM
            if (string_equal("eap-sim", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_SIM;
            }
#endif
#if CONFIG_EAP_AKA
            if (string_equal("eap-aka", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_AKA;
            }
#endif
#if CONFIG_EAP_AKA_PRIME
            if (string_equal("eap-aka-prime", argv[arg]))
            {
                network.security.type = WLAN_SECURITY_EAP_AKA_PRIME;
            }
#endif

#if CONFIG_EAP_PEAP
            network.security.eap_ver = 1;
            if (string_equal(argv[arg + 1], "ver") != false)
            {
                errno                    = 0;
                network.security.eap_ver = (bool)strtol(argv[arg + 2], NULL, 10);
                if (errno != 0)
                {
                    (void)PRINTF("Error during strtol:eap_ver errno:%d\r\n", errno);
                    return;
                }
                if (arg + 1 >= argc || (network.security.eap_ver != 0U && network.security.eap_ver != 1U))
                {
                    (void)PRINTF(
                        "Error: invalid wireless"
                        " network peap version\r\n");
                    return;
                }
                arg += 2;
            }

            if (string_equal(argv[arg + 1], "label") != false)
            {
                errno                       = 0;
                network.security.peap_label = (bool)strtol(argv[arg + 2], NULL, 10);
                if (errno != 0)
                {
                    (void)PRINTF("Error during strtol:peap_label errno:%d\r\n", errno);
                    return;
                }
                if (arg + 1 >= argc || (network.security.peap_label != 0U && network.security.peap_label != 1U))
                {
                    (void)PRINTF(
                        "Error: invalid wireless"
                        " network peap label\r\n");
                    return;
                }
                arg += 2;
            }

            if (string_equal(argv[arg + 1], "eap_crypto_binding") != false)
            {
                errno                               = 0;
                network.security.eap_crypto_binding = strtol(argv[arg + 2], NULL, 10);
                if (errno != 0)
                {
                    (void)PRINTF("Error during strtol:eap_crypto_binding errno:%d\r\n", errno);
                    return;
                }
                if (arg + 1 >= argc ||
                        (network.security.eap_crypto_binding != 0U && network.security.eap_crypto_binding != 1U &&
                         network.security.eap_crypto_binding != 2U))
                {
                    (void)PRINTF(
                        "Error: invalid wireless"
                        " network eap_crypto_binding\r\n");
                    return;
                }
                arg += 2;
            }
#endif
#if (CONFIG_EAP_SIM) || (CONFIG_EAP_AKA) || (CONFIG_EAP_AKA_PRIME)
            network.security.eap_result_ind = 1;
            if (string_equal(argv[arg + 1], "result_ind") != false)
            {
                errno                           = 0;
                network.security.eap_result_ind = (bool)strtol(argv[arg + 2], NULL, 10);
                if (errno != 0)
                {
                    (void)PRINTF("Error during strtol:eap_result_ind errno:%d\r\n", errno);
                    return;
                }
                if (arg + 1 >= argc || (network.security.eap_result_ind != 0U && network.security.eap_result_ind != 1U))
                {
                    (void)PRINTF(
                        "Error: invalid wireless"
                        " network result indication\r\n");
                    return;
                }
                arg += 2;
            }
#endif
#if CONFIG_EAP_TLS
            if ((string_equal(argv[arg + 1], "tls_cipher") != false) && network.security.wpa3_sb_192 &&
                    (network.security.type == WLAN_SECURITY_EAP_TLS))
            {
                if (string_equal(argv[arg + 2], "ECC_P384") != false)
                {
                    network.security.tls_cipher = EAP_TLS_ECC_P384;
                }
                else if (string_equal(argv[arg + 2], "RSA_3K") != false)
                {
                    network.security.tls_cipher = EAP_TLS_RSA_3K;
                }
                arg += 2;
            }
#endif
            if (string_equal(argv[arg + 1], "aid") != false)
            {
                /* Set Client Anonymous Identity */
                if (strlen(argv[arg + 2]) > IDENTITY_MAX_LENGTH)
                {
                    (void)PRINTF("Error: Aid name exceeded the max length %d\r\n", IDENTITY_MAX_LENGTH);
                    return;
                }
                strcpy(network.security.anonymous_identity, argv[arg + 2]);
                arg += 2;
            }

            while (string_equal(argv[arg + 1], "id") != false)
            {
                /* For Station Set Client Identity */
                if (strlen(argv[arg + 2]) > IDENTITY_MAX_LENGTH)
                {
                    (void)PRINTF("Error: id exceeded the max length %d\r\n", IDENTITY_MAX_LENGTH);
                    return;
                }

                strcpy(network.security.identity, argv[arg + 2]);
#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE
                /* For uAP Set External Client Identity */
                strcpy(network.security.identities[network.security.nusers], argv[arg + 2]);
#endif

                arg += 2;

                if (string_equal(argv[arg + 1], "pass") != false)
                {
                    /* Set Client Password */
                    if (strlen(argv[arg + 2]) > PASSWORD_MAX_LENGTH)
                    {
                        (void)PRINTF("Error: passwd exceeded the max length %d\r\n", PASSWORD_MAX_LENGTH);
                        return;
                    }

                    strcpy(network.security.eap_password, argv[arg + 2]);
#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE
                    /* For uAP Set External Client Identity */
                    strcpy(network.security.passwords[network.security.nusers], argv[arg + 2]);
#endif
                    arg += 2;
                }
#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE
                network.security.nusers += 1;
#endif
            }

            if (string_equal(argv[arg + 1], "key_passwd") != false)
            {
                if (strlen(argv[arg + 2]) > PASSWORD_MAX_LENGTH)
                {
                    (void)PRINTF("Error: key_passwd exceeded the maximum length %d\n", PASSWORD_MAX_LENGTH);
                    return;
                }
                /* Set Client/Server Key password */
                strcpy(network.security.client_key_passwd, argv[arg + 2]);
#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE
                strcpy(network.security.server_key_passwd, argv[arg + 2]);
#endif
                arg += 2;
            }

#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE

#if CONFIG_EAP_FAST
            if (string_equal(argv[arg + 1], "pac_opa_enc_key") != false)
            {
                /* Encryption key for EAP-FAST PAC-Opaque values. */
                if (strlen(argv[arg + 2]) > PAC_OPAQUE_ENCR_KEY_MAX_LENGTH)
                {
                    (void)PRINTF("Error: pac_opa_enc_key exceeded the maximum length %d.\r\n",
                                 PAC_OPAQUE_ENCR_KEY_MAX_LENGTH);
                    return;
                }
                strcpy(network.security.pac_opaque_encr_key, argv[arg + 2]);
                arg += 2;
            }

            if (string_equal(argv[arg + 1], "a_id") != false)
            {
                /* EAP-FAST authority identity (A-ID) */
                if (strlen(argv[arg + 2]) > A_ID_MAX_LENGTH)
                {
                    (void)PRINTF("Error: a_id exceeded the maximum length %d\r\n", A_ID_MAX_LENGTH);
                    return;
                }
                strcpy(network.security.a_id, argv[arg + 2]);
                arg += 2;
            }

            if (string_equal(argv[arg + 1], "fast_prov") != false)
            {
                /* EAP-FAST provisioning modes */
                errno                      = 0;
                network.security.fast_prov = strtol(argv[arg + 2], NULL, 10);
                if (errno != 0)
                {
                    (void)PRINTF("Error during strtol:fast_prov errno:%d\r\n", errno);
                    return;
                }
                if (arg + 1 >= argc || (network.security.fast_prov != 0U && network.security.fast_prov != 1U &&
                                        network.security.fast_prov != 2U && network.security.fast_prov != 3U))
                {
                    (void)PRINTF(
                        "Error: invalid wireless"
                        " network fast_prov\r\n");
                    return;
                }
                arg += 2;
            }
#endif
#endif
            if (string_equal(argv[arg + 1], "hash") != false)
            {
                /* CA Cert hash */
                if (strlen(argv[arg + 2]) > HASH_MAX_LENGTH)
                {
                    (void)PRINTF("Error: hash exceeded the maximum length %d\r\n", HASH_MAX_LENGTH);
                    return;
                }

                strcpy(network.security.ca_cert_hash, argv[arg + 2]);
                arg += 2;
            }

            if (string_equal(argv[arg + 1], "domain_match") != false)
            {
                /* Domain match */
                if (strlen(argv[arg + 2]) > DOMAIN_MATCH_MAX_LENGTH)
                {
                    (void)PRINTF("Error: domain_match exceeded the maximum length %d\r\n", DOMAIN_MATCH_MAX_LENGTH);
                    return;
                }

                strcpy(network.security.domain_match, argv[arg + 2]);
                arg += 2;
            }

            if (string_equal(argv[arg + 1], "domain_suffix_match") != false)
            {
                /* Domain match */
                if (strlen(argv[arg + 2]) > DOMAIN_MATCH_MAX_LENGTH)
                {
                    (void)PRINTF("Error: domain_suffix_match exceeded the maximum length %d\r\n",
                                 DOMAIN_MATCH_MAX_LENGTH);
                    return;
                }
                strcpy(network.security.domain_suffix_match, argv[arg + 2]);
                arg += 2;
            }

            info.security2++;
            arg += 1;
        }
#else  /* CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE */
#if CONFIG_WPA2_ENTP
        else if (!info.security2 && string_equal("eap-tls", argv[arg]))
        {
            t_u8 *data   = NULL;
            int data_len = 0;

            network.security.type = WLAN_SECURITY_EAP_TLS;
            /* Set Client Identity */
            strcpy(network.identity, (const char *)"client1");
            /** Set SSID specific network search */
            network.ssid_specific = 1;
            /* Specify CA certificate */
            data_len = wlan_get_entp_cert_files(FILE_TYPE_ENTP_CA_CERT, &data);
            if (data_len == 0)
            {
                wlan_free_entp_cert_files();
                (void)PRINTF("Error: invalid ca cert file\r\n");
                return;
            }
            network.security.tls_cert.ca_chain = wm_mbedtls_parse_cert((const unsigned char *)data, (size_t)data_len);
            /* Specify Client certificate */
            data_len = wlan_get_entp_cert_files(FILE_TYPE_ENTP_CLIENT_CERT, &data);
            if (data_len == 0)
            {
                wlan_free_entp_cert_files();
                (void)PRINTF("Error: invalid client cert file\r\n");
                return;
            }
            network.security.tls_cert.own_cert = wm_mbedtls_parse_cert((const unsigned char *)data, (size_t)data_len);
            /* Specify Client key */
            data_len = wlan_get_entp_cert_files(FILE_TYPE_ENTP_CLIENT_KEY, &data);
            if (data_len == 0)
            {
                wlan_free_entp_cert_files();
                (void)PRINTF("Error: invalid client key file\r\n");
                return;
            }
            network.security.tls_cert.own_key =
                wm_mbedtls_parse_key((const unsigned char *)data, (size_t)data_len, NULL, 0);
            /* Specify address type as DHCP */
            arg += 1;
            wlan_free_entp_cert_files();
        }
#endif /* CONFIG_WPA2_ENTP */
#if CONFIG_PEAP_MSCHAPV2
        else if (!info.security2 && string_equal("peap", argv[arg]))
        {
            /* Specify address type as DHCP */
            network.security.type = WLAN_SECURITY_EAP_PEAP_MSCHAPV2;
            /** Set USERNAME */
            if (strlen(argv[arg + 1]) > IDENTITY_MAX_LENGTH)
            {
                (void)PRINTF("Error: USERNAME is too long\r\n");
                return;
            }
            (void)memcpy(network.identity, argv[arg + 1], strlen(argv[arg + 1]));
            /* Set PASSWORD */
            if (strlen(argv[arg + 2]) > PASSWORD_MAX_LENGTH)
            {
                (void)PRINTF("Error: PASSWORD is too long\r\n");
                return;
            }
            (void)memcpy(network.password, argv[arg + 2], strlen(argv[arg + 2]));
            /* Set Client Identity(username) */
            //        strcpy(network.anonymous_identity, (const char *)"MSCHAPV2");
            strcpy(network.anonymous_identity, (const char *)"anon");
            /** Set SSID specific network search */
            network.ssid_specific = 1;
            /* Set network mode infra */
            network.type = WLAN_BSS_TYPE_STA;
            /* Set network mode infra */
            network.role = WLAN_BSS_ROLE_STA;
            /* Specify address type as DHCP */
            network.ip.ipv4.addr_type = ADDR_TYPE_DHCP;
            arg += 3;
        }
#endif /* CONFIG_PEAP_MSCHAPV2 */
#endif /* CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE */
        else if ((info.role == 0U) && string_equal("role", argv[arg]))
        {
            if (arg + 1 >= argc || get_role(argv[arg + 1], &network.role))
            {
                (void)PRINTF(
                    "Error: invalid wireless"
                    " network role\r\n");
                return;
            }
            arg += 2;
            info.role++;
        }
        else if ((info.mfpc == 0U) && string_equal("mfpc", argv[arg]))
        {
            errno                 = 0;
            network.security.mfpc = (bool)strtol(argv[arg + 1], NULL, 10);
            if (errno != 0)
            {
                (void)PRINTF("Error during strtol:mfpc errno:%d\r\n", errno);
                return;
            }
            if (arg + 1 >= argc || (network.security.mfpc != false && network.security.mfpc != true))
            {
                (void)PRINTF(
                    "Error: invalid wireless"
                    " network mfpc\r\n");
                return;
            }
            arg += 2;
            info.mfpc++;
        }
        else if ((info.mfpr == 0U) && string_equal("mfpr", argv[arg]))
        {
            errno                 = 0;
            network.security.mfpr = (bool)strtol(argv[arg + 1], NULL, 10);
            if (errno != 0)
            {
                (void)PRINTF("Error during strtol:mfpr errno:%d\r\n", errno);
                return;
            }
            if (arg + 1 >= argc || (network.security.mfpr != false && network.security.mfpr != true))
            {
                (void)PRINTF(
                    "Error: invalid wireless"
                    " network mfpr\r\n");
                return;
            }
            arg += 2;
            info.mfpr++;
        }
        else if (strncmp(argv[arg], "autoip", 6) == 0)
        {
            info.address = (uint8_t)ADDR_TYPE_LLA;
            arg++;
        }
#if CONFIG_WPA_SUPP
        else if (!info.pkc && string_equal("pkc", argv[arg]))
        {
            unsigned short pkc;
            if (arg + 1 >= argc)
            {
                (void)PRINTF(
                    "Error: invalid proactive key caching"
                    " argument \r\n");
                return;
            }
            pkc = a2hex_or_atoi(argv[arg + 1]);

            network.security.pkc = pkc;
            arg += 2;
            info.pkc = 1;
        }
        else if (!info.gcipher && string_equal("gc", argv[arg]))
        {
            unsigned short gcipher;
            if (arg + 1 >= argc)
            {
                (void)PRINTF(
                    "Error: invalid Group cipher"
                    " argument \r\n");
                return;
            }
            gcipher = a2hex_or_atoi(argv[arg + 1]);

            network.security.group_cipher = gcipher;
            arg += 2;
            info.gcipher = 1;
        }
        else if (!info.pcipher && string_equal("pc", argv[arg]))
        {
            unsigned short pcipher;
            if (arg + 1 >= argc)
            {
                (void)PRINTF(
                    "Error: invalid Pairwise cipher"
                    " argument \r\n");
                return;
            }
            pcipher = a2hex_or_atoi(argv[arg + 1]);

            network.security.pairwise_cipher = pcipher;
            arg += 2;
            info.pcipher = 1;
        }
        else if (!info.gmcipher && string_equal("gmc", argv[arg]))
        {
            unsigned short gmcipher;
            if (arg + 1 >= argc)
            {
                (void)PRINTF(
                    "Error: invalid Group Mgmt cipher"
                    " argument \r\n");
                return;
            }
            gmcipher = a2hex_or_atoi(argv[arg + 1]);

            network.security.group_mgmt_cipher = gmcipher;
            arg += 2;
            info.gmcipher = 1;
        }
#endif
#if CONFIG_WIFI_DTIM_PERIOD
        else if (!info.dtim && string_equal("dtim", argv[arg]))
        {
            unsigned int dtim_period;
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &dtim_period, strlen(argv[arg + 1])))
            {
                (void)PRINTF(
                    "Error: invalid dtim"
                    " argument \r\n");
                return;
            }
            network.dtim_period = (uint8_t)(dtim_period & 0XFF);
            arg += 2;
            info.dtim = 1;
        }
#endif
#if CONFIG_WIFI_CAPA
        else if (!info.wlan_capa && network.role == WLAN_BSS_ROLE_UAP && string_equal("capa", argv[arg]))
        {
            if (arg + 1 >= argc || get_capa(argv[arg + 1], &network.wlan_capa))
            {
                (void)PRINTF(
                    "Error: invalid wireless"
                    " capability\r\n");
                return;
            }
            arg += 2;
            info.wlan_capa++;
        }
#endif
        else if (!info.acs_band && string_equal("acs_band", argv[arg]))
        {
            unsigned int ACS_band = 0;
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &ACS_band, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid acs_band\r\n");
                return;
            }
            if (ACS_band != 0 && ACS_band != 1)
            {
                (void)PRINTF("Pls Set acs_band to 0 or 1.\r\n");
                (void)PRINTF(
                    "0: 2.4GHz channel   1: 5GHz channel\r\n"
                    "Not support to select dual band automatically.\r\n");
                return;
            }
            network.acs_band = (uint16_t)ACS_band;
            arg += 2;
            info.acs_band = 1;
        }
        else
        {
            dump_wlan_add_usage();
            (void)PRINTF("Error: argument %d is invalid\r\n", arg);
            return;
        }
    }
    while (arg < argc);

    if ((info.ssid == 0U) && (info.bssid == 0U))
    {
        dump_wlan_add_usage();
        (void)PRINTF("Error: specify at least the SSID or BSSID\r\n");
        return;
    }

    if ((info.security && info.security2 && info.security3) ||
            ((network.security.type == WLAN_SECURITY_WPA) && info.security && !info.security2))
    {
        dump_wlan_add_usage();
        (void)PRINTF("Error: not support WPA or WPA/WPA2/WPA3 Mixed\r\n");
        return;
    }

    if ((network.security.type == WLAN_SECURITY_WPA2) || (network.security.type == WLAN_SECURITY_WPA3_SAE))
    {
        if ((network.security.psk_len != 0U) && (network.security.password_len != 0U))
        {
            network.security.type = WLAN_SECURITY_WPA2_WPA3_SAE_MIXED;
        }
    }

    network.ip.ipv4.addr_type         = (enum address_types)(info.address);
    network.ssid[IEEEtypes_SSID_SIZE] = '\0';
    ret                               = wlan_add_network(&network);
    switch (ret)
    {
    case WM_SUCCESS:
        (void)PRINTF("Added \"%s\"\r\n", network.name);
        break;
    case -WM_E_INVAL:
        (void)PRINTF("Error: network already exists or invalid arguments\r\n");
        break;
    case -WM_E_NOMEM:
        (void)PRINTF("Error: network list is full\r\n");
        break;
    case WLAN_ERROR_STATE:
        (void)PRINTF("Error: can't add networks in this state\r\n");
        break;
    default:
        (void)PRINTF(
            "Error: unable to add network for unknown"
            " reason\r\n");
        break;
    }
}

#if CONFIG_WPA_SUPP_WPS
/** Enum that indicates type of WPS session
 *   either a push button or a PIN based session is
 *   determined by value fo this enum
 */
enum wps_session_types
{
    /** WPS session is not active */
    WPS_SESSION_INACTIVE = 0xffff,
    /** WPS Push Button session active */
    WPS_SESSION_PBC = 0x0004,
    /** WPS PIN session active */
    WPS_SESSION_PIN = 0x0000,
};
#endif

static int __scan_cb(unsigned int count)
{
    struct wlan_scan_result res;
    unsigned int i;
    int err;

    if (count == 0U)
    {
        (void)PRINTF("no networks found\r\n");
        return 0;
    }

    (void)PRINTF("%d network%s found:\r\n", count, count == 1U ? "" : "s");

    for (i = 0; i < count; i++)
    {
        err = wlan_get_scan_result(i, &res);
        if (err != 0)
        {
            (void)PRINTF("Error: can't get scan res %d\r\n", i);
            continue;
        }

        print_mac(res.bssid);

        if (res.ssid[0] != '\0')
        {
            (void)PRINTF(" \"%s\" %s\r\n", res.ssid, print_role(res.role));
        }
        else
        {
            (void)PRINTF(" (hidden) %s\r\n", print_role(res.role));
        }
        (void)PRINTF("\tmode: ");
#if CONFIG_11AC
#if CONFIG_11AX
        if (res.dot11ax != 0U)
        {
            (void)PRINTF("802.11AX ");
        }
        else
#endif
            if (res.dot11ac != 0U)
            {
                (void)PRINTF("802.11AC ");
            }
            else
#endif
                if (res.dot11n != 0U)
                {
                    (void)PRINTF("802.11N ");
                }
                else
                {
                    (void)PRINTF("802.11BG ");
                }
        (void)PRINTF("\r\n");

        (void)PRINTF("\tchannel: %d\r\n", res.channel);
        (void)PRINTF("\trssi: -%d dBm\r\n", res.rssi);
        (void)PRINTF("\tsecurity: ");
        if (res.wep != 0U)
        {
            (void)PRINTF("WEP ");
        }
        if ((res.wpa != 0U) && (res.wpa2 != 0U))
        {
            (void)PRINTF("WPA/WPA2 Mixed ");
        }
        else if ((res.wpa2 != 0U) && (res.wpa3_sae != 0U))
        {
            (void)PRINTF("WPA2/WPA3 SAE Mixed ");
        }
        else
        {
            if (res.wpa != 0U)
            {
                (void)PRINTF("WPA ");
            }
            if (res.wpa2 != 0U)
            {
                (void)PRINTF("WPA2 ");
            }
            if (res.wpa2_sha256 != 0U)
            {
                (void)PRINTF("WPA2-SHA256");
            }
            if (res.wpa3_sae != 0U)
            {
                (void)PRINTF("WPA3-SAE ");
            }
#if CONFIG_DRIVER_OWE
            if (res.owe != 0U)
            {
                (void)PRINTF("OWE Only");
            }
#endif
            if (res.wpa2_entp != 0U)
            {
                (void)PRINTF("WPA2 Enterprise ");
            }
            if (res.wpa2_entp_sha256 != 0U)
            {
                (void)PRINTF("WPA2-SHA256 Enterprise ");
            }
            if (res.wpa3_1x_sha256 != 0U)
            {
                (void)PRINTF("WPA3-SHA256 Enterprise ");
            }
            if (res.wpa3_1x_sha384 != 0U)
            {
                (void)PRINTF("WPA3-SHA384 Enterprise ");
            }
        }
#if (CONFIG_11R)
        if (res.ft_1x != 0U)
        {
            (void)PRINTF("with FT_802.1x");
        }
        if (res.ft_psk != 0U)
        {
            (void)PRINTF("with FT_PSK");
        }
        if (res.ft_sae != 0U)
        {
            (void)PRINTF("with FT_SAE");
        }
        if (res.ft_1x_sha384 != 0U)
        {
            (void)PRINTF("with FT_802.1x SHA384");
        }
#endif
        if (!((res.wep != 0U) || (res.wpa != 0U) || (res.wpa2 != 0U) || (res.wpa3_sae != 0U) || (res.wpa2_entp != 0U) ||
                (res.wpa2_sha256 != 0U) ||
#if CONFIG_DRIVER_OWE
                (res.owe != 0U) ||
#endif
                (res.wpa2_entp_sha256 != 0U) || (res.wpa3_1x_sha256 != 0U) || (res.wpa3_1x_sha384 != 0U)))
        {
            (void)PRINTF("OPEN ");
        }
        (void)PRINTF("\r\n");

        (void)PRINTF("\tWMM: %s\r\n", (res.wmm != 0U) ? "YES" : "NO");

#if CONFIG_11K
        if (res.neighbor_report_supported == true)
        {
            (void)PRINTF("\t802.11K: YES\r\n");
        }
#endif
#if CONFIG_11V
        if (res.bss_transition_supported == true)
        {
            (void)PRINTF("\t802.11V: YES\r\n");
        }
#endif
        if ((res.ap_mfpc == true) && (res.ap_mfpr == true))
        {
            (void)PRINTF("\t802.11W: Capable, Required\r\n");
        }
        if ((res.ap_mfpc == true) && (res.ap_mfpr == false))
        {
            (void)PRINTF("\t802.11W: Capable\r\n");
        }
        if ((res.ap_mfpc == false) && (res.ap_mfpr == false))
        {
            (void)PRINTF("\t802.11W: NA\r\n");
        }
#if CONFIG_WPA_SUPP_WPS
        if (res.wps)
        {
            if (res.wps_session == WPS_SESSION_PBC)
                (void)PRINTF("\tWPS: %s, Session: %s\r\n", "YES", "Push Button");
            else if (res.wps_session == WPS_SESSION_PIN)
                (void)PRINTF("\tWPS: %s, Session: %s\r\n", "YES", "PIN");
            else
                (void)PRINTF("\tWPS: %s, Session: %s\r\n", "YES", "Not active");
        }
        else
            (void)PRINTF("\tWPS: %s \r\n", "NO");
#endif
#if CONFIG_DRIVER_OWE
        if (res.trans_ssid_len != 0U)
        {
            (void)PRINTF("\tOWE BSSID: ");
            print_mac(res.trans_bssid);
            (void)PRINTF("\r\n\tOWE SSID:");
            if (res.trans_ssid_len != 0U)
            {
                (void)PRINTF(" \"%s\"\r\n", res.trans_ssid);
            }
        }
#endif
    }
#if CONFIG_WIFI_SMOKE_TESTS
    PRINTF("SCAN COMPLETED !\r\n");
#endif
    return 0;
}

static void test_wlan_thread_info(int argc, char **argv)
{
    /* TODO: implement for Zephyr */
#ifndef __ZEPHYR__
    OSA_DumpThreadInfo(NULL);
#endif
}


static void test_wlan_net_stats(int argc, char **argv)
{
    net_stat();
}

static void test_wlan_scan(int argc, char **argv)
{
    if (wlan_scan(__scan_cb) != 0)
    {
        (void)PRINTF("Error: scan request failed\r\n");
    }
    else
    {
        (void)PRINTF("Scan scheduled...\r\n");
    }
}
MSH_CMD_EXPORT(test_wlan_scan, test_wlan_scan);
static void dump_wlan_scan_opt_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "    wlan-scan-opt ssid <ssid> bssid <bssid> "
#if CONFIG_SCAN_WITH_RSSIFILTER
        "channel <channel> probes <probes> rssi_threshold <rssi_threshold>"
#else
        "channel <channel> probes <probes>"
#endif
        "\r\n");
}

static void test_wlan_scan_opt(int argc, char **argv)
{
    wlan_scan_params_v2_t wlan_scan_param;
    int arg = 1;
#if CONFIG_COMBO_SCAN
    int num_ssid = 0;
#endif
    struct
    {
        unsigned ssid : 1;
        unsigned bssid : 1;
        unsigned channel : 1;
        unsigned probes : 1;
#if CONFIG_SCAN_WITH_RSSIFILTER
        unsigned rssi_threshold : 1;
#endif
    } info;

    (void)memset(&info, 0, sizeof(info));
    (void)memset(&wlan_scan_param, 0, sizeof(wlan_scan_params_v2_t));

    if (argc < 3)
    {
        dump_wlan_scan_opt_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    do
    {
#if CONFIG_COMBO_SCAN
        if (string_equal("ssid", argv[arg]))
        {
            if (num_ssid >= MAX_NUM_SSID)
            {
                (void)PRINTF("Error: the number of SSID is more than 2\r\n");
                return;
            }
#else
        if ((info.ssid == 0U) && string_equal("ssid", argv[arg]))
        {
#endif
            if (strlen(argv[arg + 1]) > IEEEtypes_SSID_SIZE)
            {
                (void)PRINTF("Error: SSID is too long\r\n");
                return;
            }
#if CONFIG_COMBO_SCAN
            (void)memcpy(wlan_scan_param.ssid[num_ssid], argv[arg + 1], strlen(argv[arg + 1]));
            num_ssid++;
#else
            (void)memcpy(wlan_scan_param.ssid, argv[arg + 1], strlen(argv[arg + 1]));
#endif
            arg += 2;
            info.ssid = 1;
        }
        else if ((info.bssid == 0U) && string_equal("bssid", argv[arg]))
        {
            if (get_mac(argv[arg + 1], (char *)wlan_scan_param.bssid, ':') != false)
            {
                (void)PRINTF(
                    "Error: invalid BSSID argument"
                    "\r\n");
                return;
            }
            arg += 2;
            info.bssid = 1;
        }
        else if ((info.channel == 0U) && string_equal("channel", argv[arg]))
        {
            if (arg + 1 >= argc ||
                    get_uint(argv[arg + 1], (unsigned int *)(void *)&wlan_scan_param.chan_list[0].chan_number,
                             strlen(argv[arg + 1])))
            {
                (void)PRINTF(
                    "Error: invalid channel"
                    " argument\n");
                return;
            }
            wlan_scan_param.num_channels           = 1;
            wlan_scan_param.chan_list[0].scan_type = MLAN_SCAN_TYPE_ACTIVE;
            wlan_scan_param.chan_list[0].scan_time = 120;
            arg += 2;
            info.channel = 1;
        }
        else if ((info.probes == 0U) && string_equal("probes", argv[arg]))
        {
            if (arg + 1 >= argc ||
                    get_uint(argv[arg + 1], (unsigned int *)(void *)&wlan_scan_param.num_probes, strlen(argv[arg + 1])))
            {
                (void)PRINTF(
                    "Error: invalid probes"
                    " argument\n");
                return;
            }
            if (wlan_scan_param.num_probes > 4U)
            {
                (void)PRINTF(
                    "Error: invalid number of probes"
                    "\r\n");
                return;
            }
            arg += 2;
            info.probes = 1;
        }
#if CONFIG_SCAN_WITH_RSSIFILTER
        else if (!info.rssi_threshold && string_equal("rssi_threshold", argv[arg]))
        {
            if (arg + 1 >= argc)
            {
                (void)PRINTF(
                    "Error: invalid rssi threshold"
                    " argument\n");
                return;
            }
            wlan_scan_param.rssi_threshold = atoi(argv[arg + 1]);
            if (wlan_scan_param.rssi_threshold < -101)
            {
                (void)PRINTF(
                    "Error: invalid value of rssi threshold"
                    "\r\n");
                return;
            }
            arg += 2;
            info.rssi_threshold = 1;
        }
#endif
        else
        {
            dump_wlan_scan_opt_usage();
            (void)PRINTF("Error: argument %d is invalid\r\n", arg);
            return;
        }
    }
    while (arg < argc);

    if ((info.ssid == 0U) && (info.bssid == 0U))
    {
        dump_wlan_scan_opt_usage();
        (void)PRINTF("Error: specify at least the SSID or BSSID\r\n");
        return;
    }

    wlan_scan_param.cb = __scan_cb;

    if (wlan_scan_with_opt(wlan_scan_param) != 0)
    {
        (void)PRINTF("Error: scan request failed\r\n");
    }
    else
    {
        (void)PRINTF("Scan for ");
        if (info.ssid != 0U)
        {
#if CONFIG_COMBO_SCAN
            (void)PRINTF("ssid \"%s\" ", wlan_scan_param.ssid[0]);
#else
            (void)PRINTF("ssid \"%s\" ", wlan_scan_param.ssid);
#endif
        }
        if (info.bssid != 0U)
        {
            (void)PRINTF("bssid ");
            print_mac((const char *)wlan_scan_param.bssid);
        }
        if (info.probes != 0U)
        {
            (void)PRINTF("with %d probes ", wlan_scan_param.num_probes);
#if CONFIG_SCAN_WITH_RSSIFILTER
            wlan_set_rssi_threshold(wlan_scan_param.rssi_threshold);
            if (info.rssi_threshold != 0U)
                (void)PRINTF("with %d rssi_threshold ", wlan_scan_param.rssi_threshold);
#endif
        }
        (void)PRINTF("scheduled...\r\n");
    }
}

static void test_wlan_remove(int argc, char **argv)
{
    int ret;

    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <profile_name>\r\n", argv[0]);
        (void)PRINTF("Error: specify network to remove\r\n");
        return;
    }

    ret = wlan_remove_network(argv[1]);
    switch (ret)
    {
    case WM_SUCCESS:
        (void)PRINTF("Removed \"%s\"\r\n", argv[1]);
        break;
    case -WM_E_INVAL:
        (void)PRINTF("Error: network not found\r\n");
        break;
    case WLAN_ERROR_STATE:
        (void)PRINTF("Error: can't remove network in this state\r\n");
        break;
    default:
        (void)PRINTF("Error: unable to remove network\r\n");
        break;
    }
}

static void test_wlan_connect(int argc, char **argv)
{
    (void)PRINTF(
        "Connecting to network...\r\nUse 'wlan-stat' for "
        "current connection status.\r\n");

    int ret = wlan_connect(argc >= 2 ? argv[1] : NULL);

    if (ret == WLAN_ERROR_STATE)
    {
        (void)PRINTF("Error: connect manager not running\r\n");
        return;
    }

    if (ret == -WM_E_INVAL)
    {
        (void)PRINTF("Usage: %s <profile_name>\r\n", argv[0]);
        (void)PRINTF("Error: specify a network to connect\r\n");
        return;
    }
}

static void dump_wlan_connect_opt_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-connect-opt <profile_name> skipDFS\r\n");
    (void)PRINTF("    Need to specify profile_name at least.\r\n");
    (void)PRINTF("    Other parameters are optional.\r\n");
    (void)PRINTF("  To skip DFS channels when scanning:\r\n");
    (void)PRINTF("    wlan-connect-opt <profile_name> skipDFS\r\n");
}

static void test_wlan_connect_opt(int argc, char **argv)
{
    bool skip_dfs = false;
    int arg       = 2;
    int ret       = 0;

    if (argc < 2)
    {
        dump_wlan_connect_opt_usage();
        return;
    }
    else if (argc > 2)
    {
        if (string_equal("skipDFS", argv[arg]))
        {
            skip_dfs = true;
            arg++;
        }
        else
        {
            dump_wlan_connect_opt_usage();
            (void)PRINTF("Error! Argument %d is invalid\r\n", arg);
            return;
        }
    }
    ret = wlan_connect_opt(argv[1], skip_dfs);
    if (ret == WLAN_ERROR_STATE)
    {
        (void)PRINTF("Error: connect manager not running\r\n");
        return;
    }
}

static void test_wlan_reassociate(int argc, char **argv)
{
    (void)PRINTF(
        "Reassociating to network...\r\nUse 'wlan-stat' for "
        "current connection status.\r\n");

    int ret = wlan_reassociate();

    if (ret == WLAN_ERROR_STATE)
    {
        (void)PRINTF("Error: connect manager not running or not connected\r\n");
        return;
    }

    if (ret == -WM_E_INVAL)
    {
        (void)PRINTF("Usage: %s \r\n", argv[0]);
        return;
    }
}

static void test_wlan_start_network(int argc, char **argv)
{
    int ret;

    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <profile_name>\r\n", argv[0]);
        (void)PRINTF("Error: specify a network to start\r\n");
        return;
    }

    ret = wlan_start_network(argv[1]);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Error: unable to start network\r\n");
    }
}

static void test_wlan_stop_network(int argc, char **argv)
{
    int ret;
    struct wlan_network network;

    (void)memset(&network, 0x00, sizeof(struct wlan_network));
    (void)wlan_get_current_uap_network(&network);
    ret = wlan_stop_network(network.name);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Error: unable to stop network\r\n");
    }
}

static void test_wlan_disconnect(int argc, char **argv)
{
    if (wlan_disconnect() != WM_SUCCESS)
    {
        (void)PRINTF("Error: unable to disconnect\r\n");
    }
}

static void test_wlan_uap_disconnect_sta(int argc, char **argv)
{
    int ret, i;
    uint8_t raw_mac[MLAN_MAC_ADDR_LENGTH];
    wifi_sta_list_t *sl = NULL;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s C0:95:DA:XX:XX:XX\r\n", argv[0]);
        return;
    }

    ret = get_mac(argv[1], (char *)raw_mac, ':');
    if (ret != 0)
    {
        (void)PRINTF("Error: invalid MAC argument\r\n");
        return;
    }

    (void)wifi_uap_bss_sta_list(&sl);

    if (sl == NULL)
    {
        (void)PRINTF("Failed to get sta list\n\r");
        return;
    }

    wifi_sta_info_t *si = (wifi_sta_info_t *)(void *)(((t_u8 *)&sl->count) + sizeof(int));

    if (!sl->count)
    {
        (void)PRINTF("No station conneted. Pls connect to internal uap firstly\r\n");
        return;
    }

    for (i = 0; i < sl->count; i++)
    {
        if (memcmp(raw_mac, si[i].mac, MLAN_MAC_ADDR_LENGTH) == 0)
        {
            break;
        }
    }

    if (i == sl->count)
    {
        (void)PRINTF("Error: There is no connected station whose mac address is %s\r\n", argv[1]);
        (void)PRINTF("Pls enter correct mac address!\r\n");
        return;
    }

    wlan_uap_disconnect_sta(raw_mac);

#if !CONFIG_MEM_POOLS
    OSA_MemoryFree(sl);
#else
    OSA_MemoryPoolFree(buf_256_MemoryPool, sl);
#endif

    return;
}

static void test_wlan_stat(int argc, char **argv)
{
    enum wlan_connection_state state;
    enum wlan_ps_mode ps_mode;
    char ps_mode_str[30];

    if (wlan_get_ps_mode(&ps_mode) != 0)
    {
        (void)PRINTF(
            "Error: unable to get power save"
            " mode\r\n");
        return;
    }

    switch (ps_mode)
    {
    case WLAN_IEEE:
        (void)strcpy(ps_mode_str, "IEEE ps");
        break;
    case WLAN_DEEP_SLEEP:
        (void)strcpy(ps_mode_str, "Deep sleep");
        break;
    case WLAN_IEEE_DEEP_SLEEP:
        (void)strcpy(ps_mode_str, "IEEE ps and Deep sleep");
        break;
#if CONFIG_WNM_PS
    case WLAN_WNM:
        (void)strcpy(ps_mode_str, "WNM ps");
        break;
    case WLAN_WNM_DEEP_SLEEP:
        (void)strcpy(ps_mode_str, "WNM ps and Deep sleep");
        break;
#endif
    case WLAN_ACTIVE_W:
    default:
        (void)strcpy(ps_mode_str, "Active");
        break;
    }

    if (wlan_get_connection_state(&state) != 0)
    {
        (void)PRINTF(
            "Error: unable to get STA connection"
            " state\r\n");
    }
    else
    {
        switch (state)
        {
        case WLAN_DISCONNECTED:
            (void)PRINTF("Station disconnected (%s)\r\n", ps_mode_str);
            break;
        case WLAN_SCANNING:
            (void)PRINTF("Station scanning (%s)\r\n", ps_mode_str);
            break;
        case WLAN_ASSOCIATING:
            (void)PRINTF("Station associating (%s)\r\n", ps_mode_str);
            break;
        case WLAN_ASSOCIATED:
            (void)PRINTF("Station associated (%s)\r\n", ps_mode_str);
            break;
        case WLAN_CONNECTING:
            (void)PRINTF("Station connecting (%s)\r\n", ps_mode_str);
            break;
        case WLAN_CONNECTED:
            (void)PRINTF("Station connected (%s)\r\n", ps_mode_str);
            break;
        default:
            (void)PRINTF(
                "Error: invalid STA state"
                " %d\r\n",
                state);
            break;
        }
    }
    if (wlan_get_uap_connection_state(&state) != 0)
    {
        (void)PRINTF(
            "Error: unable to get uAP connection"
            " state\r\n");
    }
    else
    {
        switch (state)
        {
        case WLAN_UAP_STARTED:
            (void)strcpy(ps_mode_str, "Active");
            (void)PRINTF("uAP started (%s)\r\n", ps_mode_str);
            break;
        case WLAN_UAP_STOPPED:
            (void)PRINTF("uAP stopped\r\n");
            break;
        default:
            (void)PRINTF(
                "Error: invalid uAP state"
                " %d\r\n",
                state);
            break;
        }
    }
}

static void test_wlan_list(int argc, char **argv)
{
    struct wlan_network network;
    unsigned int count;
    unsigned int i;

    if (wlan_get_network_count(&count) != 0)
    {
        (void)PRINTF("Error: unable to get number of networks\r\n");
        return;
    }

    (void)PRINTF("%d network%s%s\r\n", count, count == 1U ? "" : "s", count > 0U ? ":" : "");
    for (i = 0; i < WLAN_MAX_KNOWN_NETWORKS; i++)
    {
        if (wlan_get_network(i, &network) == WM_SUCCESS)
        {
            print_network(&network);
        }
    }
}

static void test_wlan_info(int argc, char **argv)
{
    enum wlan_connection_state state;
#if !CONFIG_MEM_POOLS
    struct wlan_network *network = (struct wlan_network *)OSA_MemoryAllocate(sizeof(struct wlan_network));
#else
    struct wlan_network *network = OSA_MemoryPoolAllocate(buf_2048_MemoryPool);
#endif
    if (!network)
    {
        (void)PRINTF("Error: unable to malloc memory\r\n");
        return;
    }

    if (wlan_get_connection_state(&state) != 0)
    {
        (void)PRINTF(
            "Error: unable to get STA connection"
            " state\r\n");
    }
    else
    {
        switch (state)
        {
        case WLAN_CONNECTED:
            if (!wlan_get_current_network(network))
            {
                (void)PRINTF("Station connected to:\r\n");
                print_network(network);
            }
            else
                (void)PRINTF("Station not connected\r\n");
            break;
        default:
            (void)PRINTF("Station not connected\r\n");
            break;
        }
    }
    if (wlan_get_current_uap_network(network) != 0)
        (void)PRINTF("uAP not started\r\n");
    else
    {
        if (network->role == WLAN_BSS_ROLE_UAP)
            (void)PRINTF("uAP started as:\r\n");

        print_network(network);
    }
    if (network)
    {
#if !CONFIG_MEM_POOLS
        OSA_MemoryFree(network);
#else
        OSA_MemoryPoolFree(buf_2048_MemoryPool, network);
#endif
    }
}

static void test_wlan_address(int argc, char **argv)
{
    struct wlan_network network;

    if (wlan_get_current_network(&network) != 0)
    {
        (void)PRINTF("not connected\r\n");
        return;
    }
    print_address(&network.ip, network.role);
}

static void test_wlan_get_uap_channel(int argc, char **argv)
{
    int channel;
    int rv = wlan_get_uap_channel(&channel);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get channel: %d\r\n", rv);
    }
    else
    {
        (void)PRINTF("uAP channel: %d\r\n", channel);
    }
}

static void test_wlan_get_uap_sta_list(int argc, char **argv)
{
    // #if SDK_DEBUGCONSOLE != DEBUGCONSOLE_DISABLE
    int i;
    wifi_sta_list_t *sl = NULL;

    (void)wifi_uap_bss_sta_list(&sl);

    if (sl == NULL)
    {
        (void)PRINTF("Failed to get sta list\n\r");
        return;
    }

    wifi_sta_info_t *si = (wifi_sta_info_t *)(void *)(&sl->count + 1);

    (void)PRINTF("Number of STA = %d \r\n\r\n", sl->count);
    for (i = 0; i < sl->count; i++)
    {
        (void)PRINTF("STA %d information:\r\n", i + 1);
        (void)PRINTF("=====================\r\n");
        (void)PRINTF("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", si[i].mac[0], si[i].mac[1], si[i].mac[2],
                     si[i].mac[3], si[i].mac[4], si[i].mac[5]);
        (void)PRINTF("Power mfg status: %s\r\n", (si[i].power_mgmt_status == 0U) ? "active" : "power save");
        (void)PRINTF("Rssi : %d dBm\r\n\r\n", (signed char)si[i].rssi);
    }
#if !CONFIG_MEM_POOLS
    OSA_MemoryFree(sl);
#else
    OSA_MemoryPoolFree(buf_256_MemoryPool, sl);
#endif
}

static void test_wlan_ieee_ps(int argc, char **argv)
{
    int choice             = -1;
    int ret                = -WM_FAIL;
    unsigned int condition = 0;

    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <0/1>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 0 to Disable or 1 to Enable\r\n");
        return;
    }

    errno  = 0;
    choice = strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:mfpc errno:%d\r\n", errno);
        return;
    }

    if (choice == 0)
    {
        ret = wlan_ieeeps_off();
        if (ret == WM_SUCCESS)
        {
            (void)PRINTF("Turned off IEEE Power Save mode\r\n");
        }
    }
    else if (choice == 1)
    {
        ret = wlan_ieeeps_on(condition);
        if (ret == WM_SUCCESS)
        {
            (void)PRINTF("Turned on IEEE Power Save mode\r\n");
        }
        else
        {
            (void)PRINTF("Failed to turn on IEEE Power Save mode or WNM Power Save mode is enabled\r\n");
        }
    }
    else
    {
        (void)PRINTF("Error: Specify 0 to Disable or 1 to Enable\r\n");
    }
}

static void test_wlan_set_ps_cfg(int argc, char **argv)
{
    struct wlan_ieeeps_config pscfg;
    int ret = -WM_FAIL;

    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <null_pkt_interval>\r\n", argv[0]);
        (void)PRINTF("null_pkt_interval (0: Unchanged, -1: Disable, n: Interval in seconds)\r\n");
        return;
    }

    (void)memset(&pscfg, 0, sizeof(pscfg));
    pscfg.ps_null_interval = atoi(argv[1]);

    ret = wlan_set_ieeeps_cfg(&pscfg);
    if (ret == WM_SUCCESS)
        (void)PRINTF("Set power save cfg successfully");
    else
        (void)PRINTF("Failed to set power save cfg, error: %d", ret);
}

#if !CONFIG_WPA_SUPP
#if (CONFIG_WNM_PS)
static void test_wlan_wnm_ps(int argc, char **argv)
{
    int choice                = -1;
    int ret                   = -WM_FAIL;
    unsigned int condition    = 0;
    unsigned int wnm_interval = 0;

    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <0/1>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 0 to Disable or 1 to Enable\r\n");
        (void)PRINTF("If enable, please specify sleep_interval\r\n");
        (void)PRINTF("Example:\r\n");
        (void)PRINTF("	  wlan-wnm-ps 1 5\r\n");
        return;
    }

    choice = atoi(argv[1]);

    if (choice == 0)
    {
        ret = wlan_wnmps_off();
        if (ret == WM_SUCCESS)
            (void)PRINTF("Turned off WNM Power Save mode\r\n");
        else
            (void)PRINTF("Failed to turn off WNM Power Save mode\r\n");
    }
    else if (choice == 1)
    {
        if (get_uint(argv[2], &wnm_interval, strlen(argv[2])) == 0)
        {
            ret = wlan_wnmps_on(condition, (t_u16)wnm_interval);
        }
        else
        {
            (void)PRINTF("Error: please specify sleep_interval\r\n");
            return;
        }

        if (ret == WM_SUCCESS)
            (void)PRINTF("Turned on WNM Power Save mode\r\n");
        else
            (void)PRINTF("Failed to turn on WNM Power Save mode or IEEE Power Save mode is enabled\r\n");
    }
    else
    {
        (void)PRINTF("Error: Specify 0 to Disable or 1 to Enable\r\n");
    }
}
#endif
#endif

static void test_wlan_deep_sleep_ps(int argc, char **argv)
{
    int choice = -1;
    int ret    = -WM_FAIL;

    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <0/1>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 0 to Disable or 1 to Enable\r\n");
        return;
    }

    errno  = 0;
    choice = strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:deep_sleep_ps errno:%d\r\n", errno);
        return;
    }

    if (choice == 0)
    {
        ret = wlan_deepsleepps_off();
        if (ret == WM_SUCCESS)
        {
            (void)PRINTF("Turned off Deep Sleep Power Save mode\r\n");
        }
        else
        {
            (void)PRINTF("Failed to turn off Deep Sleep Power Save mode\r\n");
        }
    }
    else if (choice == 1)
    {
        ret = wlan_deepsleepps_on();
        if (ret == WM_SUCCESS)
        {
            (void)PRINTF("Turned on Deep Sleep Power Save mode\r\n");
        }
        else
        {
            (void)PRINTF("Failed to turn on Deep Sleep Power Save mode\r\n");
        }
    }
    else
    {
        (void)PRINTF("Error: Specify 0 to Disable or 1 to Enable\r\n");
    }
}

static void test_wlan_get_beacon_interval(int argc, char **argv)
{
    int beacon_interval = wlan_get_beacon_period();

    (void)PRINTF("Beacon interval: %d\r\n", beacon_interval);
}

#if CONFIG_WIFI_TX_PER_TRACK
static void dump_wlan_tx_pert_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "    wlan-tx-pert <0/1> <STA/AP> <p:tx_pert_check_period> "
        "<r:tx_pert_check_ratio> <n:tx_pert_check_num>"
        "\r\n");
    (void)PRINTF("Options:\r\n");
    (void)PRINTF("    <0/1>: Disable/enable Tx Pert tracking.\r\n");
    (void)PRINTF("    <STA/UAP>: User needs to indicate which interface this tracking for.\r\n");
    (void)PRINTF("    <p>: Tx Pert check period. Unit is second.\r\n");
    (void)PRINTF(
        "    <r>: Tx Pert ratio threshold (unit 10 percent). (Fail TX packet)/(Total TX packets). The default value is "
        "5.\r\n");
    (void)PRINTF(
        "    <n>: A watermark of check number (default 5). Fw will start tracking Tx Pert after sending n "
        "packets.\r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("    wlan-tx-pert 1 UAP 5 3 5\r\n");
    (void)PRINTF("Note:\r\n");
    (void)PRINTF("    Please verify by iperf or ping\r\n");
    (void)PRINTF("    When the traffic quality is good enough, it will not be triggered\r\n");
}

static void test_wlan_tx_pert(int argc, char **argv)
{
    struct wlan_tx_pert_info tx_pert;
    mlan_bss_type bss_type = MLAN_BSS_TYPE_STA;

    if (argc < 2)
    {
        dump_wlan_tx_pert_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    (void)memset(&tx_pert, 0, sizeof(tx_pert));
    tx_pert.tx_pert_check = atoi(argv[1]);
    if (tx_pert.tx_pert_check == 1 && argc < 6)
    {
        (void)PRINTF("Error: invalid number of arguments.\r\n");
        (void)PRINTF(
            "Need specify bss_type tx_pert_chk_prd, tx_perf_chk_ratio and tx_pert_chk_num"
            "\r\n");
        return;
    }
    if (string_equal("STA", argv[2]))
        bss_type = MLAN_BSS_TYPE_STA;
    else if (string_equal("UAP", argv[2]))
        bss_type = MLAN_BSS_TYPE_UAP;
    if (tx_pert.tx_pert_check == 1)
    {
        tx_pert.tx_pert_check_peroid = (t_u8)atoi(argv[3]);
        tx_pert.tx_pert_check_ratio  = (t_u8)atoi(argv[4]);
        tx_pert.tx_pert_check_num    = atoi(argv[5]);
    }
    wlan_set_tx_pert(&tx_pert, bss_type);
}
#endif

#if CONFIG_TX_RX_HISTOGRAM
static void dump_wlan_txrx_histogram_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan_txrx_histogram <action> <enable>\r\n");
    (void)PRINTF("        <enable> : 0 - disable TX/RX statistics\r\n");
    (void)PRINTF("                   1 - enable TX/RX statistics\r\n");
    (void)PRINTF("                   2 - get TX/RX statistics\r\n");
    (void)PRINTF("        <action> : 1 - enable/disable/get TX statistics\r\n");
    (void)PRINTF("                   2 - enable/disable/get RX statistics\r\n");
    (void)PRINTF("                   3 - enable/disable/get TX and RX statistics\r\n");
    (void)PRINTF("Note:\r\n");
    (void)PRINTF("    When enable is 0 or 1, the action parameter should not be entered\r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("    wlan_txrx_histogram 2 3\r\n");
}

static void test_wlan_txrx_histogram(int argc, char **argv)
{
    struct wlan_txrx_histogram_info txrx_histogram;
    t_u8 *buf = NULL;

    tx_pkt_ht_rate_info *tx_ht_info;
    tx_pkt_vht_rate_info *tx_vht_info;
    tx_pkt_he_rate_info *tx_he_info;
    tx_pkt_rate_info *tx_info;
    rx_pkt_ht_rate_info *rx_ht_info;
    rx_pkt_vht_rate_info *rx_vht_info;
    rx_pkt_he_rate_info *rx_he_info;
    rx_pkt_rate_info *rx_info;

    t_u8 *pos             = NULL;
    t_u16 resp_value_size = 0;
    int i                 = 0;
    t_u16 buf_size        = 0;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_txrx_histogram_usage();
        return;
    }

    (void)memset(&txrx_histogram, 0, sizeof(txrx_histogram));
    txrx_histogram.enable = atoi(argv[1]);
    if (argc == 2)
    {
        txrx_histogram.action = 0;
    }
    else
    {
        txrx_histogram.action = atoi(argv[2]);
    }

    if ((txrx_histogram.enable > 2) || (txrx_histogram.action > 3))
    {
        (void)PRINTF("Error: invalid arguments.\r\n");
        dump_wlan_txrx_histogram_usage();
        return;
    }
    if ((txrx_histogram.enable == 0 || txrx_histogram.enable == 1) && (txrx_histogram.action != 0))
    {
        (void)PRINTF("Error: invalid arguments.\r\n");
        dump_wlan_txrx_histogram_usage();
        return;
    }

    if (txrx_histogram.enable & GET_TX_RX_HISTOGRAM)
    {
        if (txrx_histogram.action == FLAG_TX_HISTOGRAM)
        {
            buf_size = sizeof(resp_value_size) + sizeof(tx_pkt_ht_rate_info) + sizeof(tx_pkt_vht_rate_info) +
                       sizeof(tx_pkt_he_rate_info) + sizeof(tx_pkt_rate_info);
        }
        else if (txrx_histogram.action == FLAG_RX_HISTOGRAM)
        {
            buf_size = sizeof(resp_value_size) + sizeof(rx_pkt_ht_rate_info) + sizeof(rx_pkt_vht_rate_info) +
                       sizeof(rx_pkt_he_rate_info) + sizeof(rx_pkt_rate_info);
        }
        else if ((txrx_histogram.action & FLAG_TX_HISTOGRAM) && (txrx_histogram.action & FLAG_RX_HISTOGRAM))
        {
            buf_size = sizeof(resp_value_size) + sizeof(tx_pkt_ht_rate_info) + sizeof(tx_pkt_vht_rate_info) +
                       sizeof(tx_pkt_he_rate_info) + sizeof(tx_pkt_rate_info) + sizeof(rx_pkt_ht_rate_info) +
                       sizeof(rx_pkt_vht_rate_info) + sizeof(rx_pkt_he_rate_info) + sizeof(rx_pkt_rate_info);
        }
    }
    if (buf_size > 0)
    {
        buf = OSA_MemoryAllocate(buf_size);
        if (!buf)
        {
            PRINTF("test_wlan_txrx_histogram buf allocate memory failed\r\n");
            return;
        }
        (void)memset(buf, 0, sizeof(buf_size));
        (void)memcpy(buf, &buf_size, sizeof(buf_size));
    }

    wlan_set_txrx_histogram(&txrx_histogram, buf);

    if (buf == NULL)
    {
        return;
    }

    /*Make the pos pointer points to the size*/
    pos = (t_u8 *)buf;
    memcpy(&resp_value_size, pos, sizeof(resp_value_size));
    /*Make the pos pointer points to the data replied by fw*/
    pos += sizeof(resp_value_size);

    if (txrx_histogram.enable & GET_TX_RX_HISTOGRAM)
    {
        if (txrx_histogram.action & FLAG_TX_HISTOGRAM)
        {
            PRINTF("The TX histogram statistic:\n");
            PRINTF("============================================\n");
            tx_ht_info = (tx_pkt_ht_rate_info *)pos;
            for (i = 0; i < 16; i++)
            {
                PRINTF("htmcs_txcnt[%d]       = %u\n", i, tx_ht_info->htmcs_txcnt[i]);
                PRINTF("htsgi_txcnt[%d]       = %u\n", i, tx_ht_info->htsgi_txcnt[i]);
                PRINTF("htstbcrate_txcnt[%d]  = %u\n", i, tx_ht_info->htstbcrate_txcnt[i]);
            }
            pos += sizeof(tx_pkt_ht_rate_info);
            tx_vht_info = (tx_pkt_vht_rate_info *)pos;
            for (i = 0; i < 10; i++)
            {
                PRINTF("vhtmcs_txcnt[%d]      = %u\n", i, tx_vht_info->vhtmcs_txcnt[i]);
                PRINTF("vhtsgi_txcnt[%d]      = %u\n", i, tx_vht_info->vhtsgi_txcnt[i]);
                PRINTF("vhtstbcrate_txcnt[%d] = %u\n", i, tx_vht_info->vhtstbcrate_txcnt[i]);
            }
            pos += sizeof(tx_pkt_vht_rate_info);
            if (resp_value_size == (sizeof(tx_pkt_ht_rate_info) + sizeof(tx_pkt_vht_rate_info) +
                                    sizeof(tx_pkt_he_rate_info) + sizeof(tx_pkt_rate_info)) ||
                    resp_value_size ==
                    (sizeof(tx_pkt_ht_rate_info) + sizeof(tx_pkt_vht_rate_info) + sizeof(tx_pkt_he_rate_info) +
                     sizeof(tx_pkt_rate_info) + sizeof(rx_pkt_ht_rate_info) + sizeof(rx_pkt_vht_rate_info) +
                     sizeof(rx_pkt_he_rate_info) + sizeof(rx_pkt_rate_info)))
            {
                tx_he_info = (tx_pkt_he_rate_info *)pos;
                for (i = 0; i < 12; i++)
                {
                    PRINTF("hemcs_txcnt[%d]      = %u\n", i, tx_he_info->hemcs_txcnt[i]);
                    PRINTF("hestbcrate_txcnt[%d] = %u\n", i, tx_he_info->hestbcrate_txcnt[i]);
                }
                pos += sizeof(tx_pkt_he_rate_info);
            }
            tx_info = (tx_pkt_rate_info *)pos;
            for (i = 0; i < 2; i++)
                PRINTF("nss_txcnt[%d]         = %u\n", i, tx_info->nss_txcnt[i]);
            for (i = 0; i < 3; i++)
                PRINTF("bandwidth_txcnt[%d]   = %u\n", i, tx_info->bandwidth_txcnt[i]);
            for (i = 0; i < 4; i++)
                PRINTF("preamble_txcnt[%d]    = %u\n", i, tx_info->preamble_txcnt[i]);
            PRINTF("ldpc_txcnt           = %u\n", tx_info->ldpc_txcnt);
            PRINTF("rts_txcnt            = %u\n", tx_info->rts_txcnt);
            PRINTF("ack_RSSI             = %d\n\n", tx_info->ack_RSSI);
            pos += sizeof(tx_pkt_rate_info);
        }
        if (txrx_histogram.action & FLAG_RX_HISTOGRAM)
        {
            PRINTF("The RX histogram statistic:\n");
            PRINTF("============================================\n");
            rx_ht_info = (rx_pkt_ht_rate_info *)pos;
            for (i = 0; i < 16; i++)
            {
                PRINTF("htmcs_rxcnt[%d]       = %u\n", i, rx_ht_info->htmcs_rxcnt[i]);
                PRINTF("htsgi_rxcnt[%d]       = %u\n", i, rx_ht_info->htsgi_rxcnt[i]);
                PRINTF("htstbcrate_rxcnt[%d]  = %u\n", i, rx_ht_info->htstbcrate_rxcnt[i]);
            }
            pos += sizeof(rx_pkt_ht_rate_info);
            rx_vht_info = (rx_pkt_vht_rate_info *)pos;
            for (i = 0; i < 10; i++)
            {
                PRINTF("vhtmcs_rxcnt[%d]      = %u\n", i, rx_vht_info->vhtmcs_rxcnt[i]);
                PRINTF("vhtsgi_rxcnt[%d]      = %u\n", i, rx_vht_info->vhtsgi_rxcnt[i]);
                PRINTF("vhtstbcrate_rxcnt[%d] = %u\n", i, rx_vht_info->vhtstbcrate_rxcnt[i]);
            }
            pos += sizeof(rx_pkt_vht_rate_info);
            if (resp_value_size == (sizeof(rx_pkt_ht_rate_info) + sizeof(rx_pkt_vht_rate_info) +
                                    sizeof(rx_pkt_he_rate_info) + sizeof(rx_pkt_rate_info)) ||
                    resp_value_size ==
                    (sizeof(tx_pkt_ht_rate_info) + sizeof(tx_pkt_vht_rate_info) + sizeof(tx_pkt_he_rate_info) +
                     sizeof(tx_pkt_rate_info) + sizeof(rx_pkt_ht_rate_info) + sizeof(rx_pkt_vht_rate_info) +
                     sizeof(rx_pkt_he_rate_info) + sizeof(rx_pkt_rate_info)))
            {
                rx_he_info = (rx_pkt_he_rate_info *)pos;
                for (i = 0; i < 12; i++)
                {
                    PRINTF("hemcs_rxcnt[%d]      = %u\n", i, rx_he_info->hemcs_rxcnt[i]);
                    PRINTF("hestbcrate_rxcnt[%d] = %u\n", i, rx_he_info->hestbcrate_rxcnt[i]);
                }
                pos += sizeof(rx_pkt_he_rate_info);
            }
            rx_info = (rx_pkt_rate_info *)pos;
            for (i = 0; i < 2; i++)
                PRINTF("nss_rxcnt[%d]         = %u\n", i, rx_info->nss_rxcnt[i]);
            PRINTF("nsts_rxcnt           = %u\n", rx_info->nsts_rxcnt);
            for (i = 0; i < 3; i++)
                PRINTF("bandwidth_rxcnt[%d]   = %u\n", i, rx_info->bandwidth_rxcnt[i]);
            for (i = 0; i < 6; i++)
                PRINTF("preamble_rxcnt[%d]    = %u\n", i, rx_info->preamble_rxcnt[i]);
            for (i = 0; i < 2; i++)
                PRINTF("ldpc_txbfcnt[%d]      = %u\n", i, rx_info->ldpc_txbfcnt[i]);
            for (i = 0; i < 2; i++)
                PRINTF("rssi_value[%d]        = %d\n", i, rx_info->rssi_value[i]);
            for (i = 0; i < 4; i++)
                PRINTF("rssi_chain0[%d]       = %d\n", i, rx_info->rssi_chain0[i]);
            for (i = 0; i < 4; i++)
                PRINTF("rssi_chain1[%d]       = %d\n", i, rx_info->rssi_chain1[i]);
            PRINTF("\n");
        }
    }
    else if (txrx_histogram.enable & ENABLE_TX_RX_HISTOGRAM)
        PRINTF("Enable the TX and RX histogram statistic\n");
    else
    {
        PRINTF("Disable the TX and RX histogram statistic\n");
    }
    if (buf)
    {
        OSA_MemoryFree(buf);
    }
}
#endif

#if CONFIG_ROAMING
static void dump_wlan_roaming_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "    wlan-roaming <0/1> <rssi_threshold>"
        "\r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("    wlan-roaming 1 40\r\n");
}

static void test_wlan_roaming(int argc, char **argv)
{
    int enable                 = 0;
    uint8_t rssi_low_threshold = 0;

    if ((argc != 2) && (argc != 3))
    {
        dump_wlan_roaming_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    errno  = 0;
    enable = (int)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan roaming errno:%d\r\n", errno);
        return;
    }

    if (argc == 3)
    {
        errno              = 0;
        rssi_low_threshold = (uint8_t)strtol(argv[2], NULL, 10);
        if (errno != 0)
        {
            (void)PRINTF("Error during strtol:rssi_threshold errno:%d\r\n", errno);
            return;
        }
    }

    wlan_set_roaming(enable, rssi_low_threshold);
    return;
}
#endif

#if CONFIG_WIFI_MAX_CLIENTS_CNT
static void test_wlan_set_max_clients_count(int argc, char **argv)
{
    int max_clients_count;
    int ret;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s  max_clients_count\r\n", argv[0]);
        return;
    }

    max_clients_count = atoi(argv[1]);

    ret = wlan_set_uap_max_clients(max_clients_count);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set max clients count\r\n");
    }
}
#endif

static void test_wlan_set_rts(int argc, char **argv)
{
    int rthr;
    int ret;
    mlan_bss_type bss_type = MLAN_BSS_TYPE_STA;

    if (argc != 3)
    {
        (void)PRINTF("Usage: %s <sta/uap> <rts threshold>\r\n", argv[0]);
        return;
    }
    if (string_equal("sta", argv[1]))
        bss_type = MLAN_BSS_TYPE_STA;
    else if (string_equal("uap", argv[1]))
        bss_type = MLAN_BSS_TYPE_UAP;
    else
    {
        (void)PRINTF("Usage: %s <sta/uap> <rts threshold>\r\n", argv[0]);
        return;
    }

    rthr = atoi(argv[2]);

    if (bss_type == MLAN_BSS_TYPE_STA)
        ret = wlan_set_rts(rthr);
    else
        ret = wlan_set_uap_rts(rthr);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set rts threshold\r\n");
    }
}

#if CONFIG_WIFI_FRAG_THRESHOLD
static void test_wlan_set_frag(int argc, char **argv)
{
    int frag;
    int ret;
    mlan_bss_type bss_type = MLAN_BSS_TYPE_STA;

    if (argc != 3)
    {
        (void)PRINTF("Usage: %s <sta/uap> <fragment threshold>\r\n", argv[0]);
        return;
    }

    if (string_equal("sta", argv[1]))
        bss_type = MLAN_BSS_TYPE_STA;
    else if (string_equal("uap", argv[1]))
        bss_type = MLAN_BSS_TYPE_UAP;
    else
    {
        (void)PRINTF("Usage: %s <sta/uap> <fragment threshold>\r\n", argv[0]);
        return;
    }

    frag = atoi(argv[2]);

    if (bss_type == MLAN_BSS_TYPE_STA)
        ret = wlan_set_frag(frag);
    else
        ret = wlan_set_uap_frag(frag);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set fragment threshold\r\n");
    }
}
#endif


#if CONFIG_11K
static void test_wlan_host_11k_cfg(int argc, char **argv)
{
    int enable_11k;
    int ret;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s <0/1> < 0--disable host 11k; 1---enable host 11k>\r\n", argv[0]);
        return;
    }

    errno      = 0;
    enable_11k = (int)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan_host_11k errno:%d\r\n", errno);
        return;
    }

    ret = wlan_host_11k_cfg(enable_11k);

    if (ret == -WM_E_PERM)
    {
        (void)PRINTF("Please disable fw base 11k.(wlan-host-11k-enable 0)\r\n");
    }
    else if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set 11k config\r\n");
    }
    else
    {
        /* Do nothing */
    }
}

static void test_wlan_host_11k_neighbor_request(int argc, char **argv)
{
    int ret;
    int len                            = 0;
    char ssid[IEEEtypes_SSID_SIZE + 1] = {0};

    if ((argc != 1 && argc != 3) || (argc == 3 && !string_equal("ssid", argv[1])))
    {
        (void)PRINTF("Usage: %s\r\n", argv[0]);
        (void)PRINTF("or     %s ssid <ssid>\r\n", argv[0]);
        return;
    }

    if (argc == 3)
    {
        if (strlen(argv[2]) > IEEEtypes_SSID_SIZE)
        {
            (void)PRINTF("Error: ssid too long\r\n");
            return;
        }
        else
        {
            (void)memcpy((void *)ssid, (const void *)argv[2], (size_t)strlen(argv[2]));
            len       = (int)strlen(argv[2]);
            ssid[len] = '\0';
        }
    }

    ret = wlan_host_11k_neighbor_req(ssid);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Error: send neighbor report request fail\r\n");
        return;
    }
}
#endif

#if CONFIG_11V
static void test_wlan_host_11v_bss_trans_query(int argc, char **argv)
{
    int ret;
    int query_reason;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s <query_reason[0..16]>\r\n", argv[0]);
        return;
    }

    errno        = 0;
    query_reason = (int)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan_host_11v_bss_trans_query errno:%d\r\n", errno);
        return;
    }

    if (query_reason < 0 || query_reason > 16)
    {
        (void)PRINTF("Usage: %s <query_reason[0..16]>\r\n", argv[0]);
        return;
    }

    ret = wlan_host_11v_bss_trans_query((t_u8)query_reason);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Error: send bss transition query failed\r\n");
        return;
    }
}
#endif

#if !CONFIG_WPA_SUPP
#if CONFIG_DRIVER_MBO
static void test_wlan_mbo_cfg(int argc, char **argv)
{
    int enable_mbo;
    int ret;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s <0/1> < 0--disable MBO; 1---enable MBO>\r\n", argv[0]);
        return;
    }

    errno      = 0;
    enable_mbo = (int)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan mbo cfg:%d\r\n", errno);
        return;
    }

    ret = wlan_host_mbo_cfg(enable_mbo);

    if (ret == -WM_E_PERM)
    {
        (void)PRINTF("Please disable MBO.(wlan-mbo-enable 0)\r\n");
    }
    else if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to config MBO\r\n");
    }
    else
    {
        /* Do nothing */
    }
}

static void test_wlan_mbo_non_prefer_chs(int argc, char **argv)
{
    int ret;
    uint8_t ch0, ch1, preference0, preference1;

    if (argc != 5)
    {
        (void)PRINTF(
            "Usage: %s <ch0> <Preference0: 0/1/255> <ch1> <Preference1: 0/1/255> < 0--non-operable; 1--prefers not to "
            "operate; 255--prefers to operate>\r\n",
            argv[0]);
        return;
    }

    errno = 0;
    ch0   = (uint8_t)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan mbo non prefer chs:%d\r\n", errno);
        return;
    }

    errno       = 0;
    preference0 = (uint8_t)strtol(argv[2], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan mbo non prefer chs:%d\r\n", errno);
        return;
    }

    errno = 0;
    ch1   = (uint8_t)strtol(argv[3], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan mbo non prefer chs:%d\r\n", errno);
        return;
    }

    errno       = 0;
    preference1 = (uint8_t)strtol(argv[4], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan mbo non prefer chs:%d\r\n", errno);
        return;
    }

    ret = wlan_mbo_peferch_cfg(ch0, preference0, ch1, preference1);

    if (ret == -WM_E_PERM)
    {
        (void)PRINTF("Please add pefer or non-pefer channels.\r\n");
    }
    else if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to add pefer or non-pefer channels.\r\n");
    }
    else
    {
        /* Do nothing */
    }
}
#endif
#endif

#if (CONFIG_11MC) || (CONFIG_11AZ)

static void dump_wlan_unassoc_ftm_cfg_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-unassoc-ftm-cfg <action> config <1/0>\r\n");
    (void)PRINTF("action: 1 -> Set\r\n");
    (void)PRINTF("config: 1 -> Enable; 0 -> Disable \r\n");
}

static void test_wlan_unassoc_ftm_cfg(int argc, char **argv)
{
    unsigned int action, config;
    int arg = 2;

    if (argc < 2 || argc > 4)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_unassoc_ftm_cfg_usage();
        return;
    }

    action = a2hex_or_atoi(argv[1]);
    if (action != 1)
    {
        dump_wlan_unassoc_ftm_cfg_usage();
        return;
    }

    while (arg < argc)
    {
        if (string_equal("config", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &config, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid config argument\r\n");
                dump_wlan_unassoc_ftm_cfg_usage();
                return;
            }
        }
        else
        {
            (void)PRINTF("Error: invalid [%s] argument\r\n", argv[arg]);
            dump_wlan_unassoc_ftm_cfg_usage();
            return;
        }
        arg += 2;
    }
    wlan_unassoc_ftm_cfg(action, config);
}

static void dump_wlan_ftm_ctrl_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-ftm-ctrl <action> loop_cnt <count> channel <channel> mac <peer_mac>\r\n");
    (void)PRINTF("action:\r\n");
    (void)PRINTF(" 1: Start non-secure 11mc/11az FTM with associated Peer AP\r\n");
    (void)PRINTF(" 2: Stop FTM session\r\n");
    (void)PRINTF(" 3: Start secure 11az FTM with associated Peer AP\r\n");
    (void)PRINTF(" 4: Start non-secure 11az/11mc FTM with unassoc Peer\r\n");
    (void)PRINTF(" 5: Start secure 11az FTM with unassociated & pre-authenticated Peer\r\n");
    (void)PRINTF("loop_cnt: number of ftm sessions to run repeatedly (default:1, 0:non-stop, n:times>)\r\n");
    (void)PRINTF("channel: Channel on which FTM must be started\r\n");
    (void)PRINTF("mac: Mac address of the peer with whom FTM session is required\r\n\r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("Run non-secure FTM session:\r\n");
    (void)PRINTF("wlan-ftm-ctrl 1 loop_cnt 1 channel 36 mac 00:50:43:20:bc:44\r\n");
    (void)PRINTF("Runs secure 11az FTM session:\r\n");
    (void)PRINTF("wlan-ftm-ctrl 3 loop_cnt 1 channel 36 mac 00:50:43:20:bc:44\r\n");
    (void)PRINTF("Runs non-secure FTM session with unassoc peer until user terminate:\r\n");
    (void)PRINTF("wlan-ftm-ctrl 4 loop_cnt 1 channel 36 mac 00:50:43:20:bc:44\r\n");
    (void)PRINTF("Runs Secure FTM session with unassociated Peer AP:\r\n");
    (void)PRINTF("wlan-ftm-ctrl 5 loop_cnt 1 channel 36 mac 00:50:43:20:bc:44\r\n");
    (void)PRINTF("Stop the FTM session:\r\n");
    (void)PRINTF("wlan-ftm-ctrl 2\r\n");
}
static void test_wlan_ftm_ctrl(int argc, char **argv)
{
    unsigned action, loop_cnt, channel;
    int arg = 2;
    t_u8 peer_mac[MLAN_MAC_ADDR_LENGTH];
    struct
    {
        unsigned loop_cnt : 1;
        unsigned channel : 1;
        unsigned mac : 1;
    } info;

    (void)memset(&info, 0, sizeof(info));

    if (argc < 2 || argc > 8)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_ftm_ctrl_usage();
        return;
    }

    action = a2hex_or_atoi(argv[1]);
    if (action < 1 && action > 5)
    {
        dump_wlan_ftm_ctrl_usage();
        return;
    }

    if (action == 2)
        goto done;

    do
    {
        if (!info.loop_cnt && string_equal("loop_cnt", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &loop_cnt, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid loop_cnt argument\r\n");
                dump_wlan_ftm_ctrl_usage();
                return;
            }
            arg += 2;
            info.loop_cnt = 1;
        }
        else if (!info.channel && string_equal("channel", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &channel, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid channel argument\r\n");
                dump_wlan_ftm_ctrl_usage();
                return;
            }
            arg += 2;
            info.channel = 1;
        }
        else if (!info.mac && string_equal("mac", argv[arg]))
        {
            if (get_mac(argv[arg + 1], (char *)peer_mac, ':'))
            {
                (void)PRINTF("Error: invalid MAC argument\r\n");
                dump_wlan_ftm_ctrl_usage();
                return;
            }
            arg += 2;
            info.mac = 1;
        }
        else
        {
            (void)PRINTF("Error: invalid argument\r\n");
            dump_wlan_ftm_ctrl_usage();
            return;
        }
    }
    while (arg < argc);
done:
    wlan_ftm_start_stop(action, loop_cnt, peer_mac, channel);
}

static void dump_wlan_11mc_nego_cfg()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "wlan-11mc-nego-cfg burst_dur <burst_dur> min_delta <min_delta> asap <asap>"
        " ftm_per_burst <ftm_per_burst> BW <bw> burst_period <burst_period>\r\n");

    (void)PRINTF("burst_dur: 2 or 11\r\n");
    (void)PRINTF("min_delta: 1 to 63\r\n");
    (void)PRINTF("asap: 0 or 1\r\n");
    (void)PRINTF("ftm_per_burst: 2 to 10\r\n");
    (void)PRINTF("BW: 9 to 13\r\n");
    (void)PRINTF("burst_period: 1 to 10\r\n");
    (void)PRINTF(
        "Example:\r\n"
        "wlan-11mc-nego-cfg burst_dur 11 min_delta 60 asap 1 ftm_per_burst 5 BW 13 burst_period 10");
}

static void dump_wlan_civ_cfg()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "wlan-civ-cfg civ_req <civ_req> loc_type <loc_type> country_code <country_code>"
        " addr_type <addr_type>\r\n");

    (void)PRINTF("civ_req: 0 or 1\r\n");
    (void)PRINTF("loc_type: 1\r\n");
    (void)PRINTF("country_code: 0 for USA\r\n");
    (void)PRINTF("addr_type: 22\r\n");
    (void)PRINTF(
        "Example:\r\n"
        "wlan-civ-cfg civ_req 1 loc_type 1 country_code 0 addr_type 22");
}

static void dump_wlan_loc_cfg()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "wlan-loc-cfg lci_req <lci request> latit <latitude> longi <longitude> altit <altitude>"
        " lat_uncert <latitude uncertainity> lon_uncert <longitude uncertainity> alt_uncert <altitude "
        "uncertainity>\r\n");

    (void)PRINTF("lci_req: 0 or 1\r\n");
    (void)PRINTF("latitude: -180.0 to 180.0\r\n");
    (void)PRINTF("longitude: -180.0 to 180.0\r\n");
    (void)PRINTF("altitude: -180.0 to 180.0\r\n");
    (void)PRINTF("latitude uncertainity: 0 to 255\r\n");
    (void)PRINTF("longitude uncertainity: 0 to 255\r\n");
    (void)PRINTF("altitude uncertainity: 0 to 255\r\n");
    (void)PRINTF(
        "Example:\r\n"
        "wlan-loc-cfg lci_req 1 latit -111.111 longi 222.222 altit 33.333 lat_uncert 1 lon_uncert 2 alt_uncert 3");
}

static void dump_wlan_11az_rang_cfg()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "wlan-11az-rang-cfg <protocol> format_bw <format_bw> num_measurements <num_measurements>"
        " measurement_freq <measurement_freq> i2r_sts <i2r_sts> r2i_sts <r2i_sts> i2r_lmr <i2r_lmr>\r\n");

    (void)PRINTF("protocol: 0 or 1\r\n");
    (void)PRINTF("format_bw: 0 to 2\r\n");
    (void)PRINTF("num_measurements: 1 to 10\r\n");
    (void)PRINTF("measurement_freq: 1 to 10\r\n");
    (void)PRINTF("i2r_sts: 0/1 - Num of antennas: 0=>1 antenna and 1=>2 antennas\r\n");
    (void)PRINTF("r2i_sts: 0/1 - Num of antennas: 0=>1 antenna and 1=>2 antennas\r\n");
    (void)PRINTF("i2r_lmr: 0 never, 1 always, 2 up to RSTA\r\n");
    (void)PRINTF(
        "Example:\r\n"
        "wlan-11az-rang-cfg 1 format_bw 2 num_measurements 5 measurement_freq 4 i2r_sts 0 r2i_sts 0 i2r_lmr 0\r\n");
}

static void test_wlan_loc_cfg(int argc, char **argv)
{
    unsigned lci_req, lati_uncertanity, longi_uncertanity, alti_uncertanity;
    double latitude, longitude, altitude;
    location_cfg_info_t lci;
    int arg = 1;
    if (argc != 15)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_loc_cfg();
        return;
    }

    if (argc >= 2)
    {
        lci_req           = 0;
        latitude          = LCI_LATITIUDE;
        longitude         = LCI_LONGITUDE;
        altitude          = LCI_ALTITUDE;
        lati_uncertanity  = LCI_LATITUDE_UNCERTAINITY;
        longi_uncertanity = LCI_LONGITUDE_UNCERTAINITY;
        alti_uncertanity  = LCI_ALTITUDE_UNCERTAINITY;
    }

    while (arg < argc)
    {
        if (string_equal("lci_req", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &lci_req, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid number of LCI require argument\r\n");
                dump_wlan_loc_cfg();
                return;
            }
        }
        else if (string_equal("latit", argv[arg]))
        {
            latitude = strtod(argv[arg + 1], NULL);
            if (!latitude)
            {
                (void)PRINTF("Error: invalid latitude argument\r\n");
                dump_wlan_loc_cfg();
                return;
            }
        }
        else if (string_equal("longi", argv[arg]))
        {
            longitude = strtod(argv[arg + 1], NULL);
            if (!longitude)
            {
                (void)PRINTF("Error: invalid longitude argument\r\n");
                dump_wlan_loc_cfg();
                return;
            }
        }
        else if (string_equal("altit", argv[arg]))
        {
            altitude = strtod(argv[arg + 1], NULL);
            if (!latitude)
            {
                (void)PRINTF("Error: invalid latitude argument\r\n");
                dump_wlan_loc_cfg();
                return;
            }
        }
        else if (string_equal("lat_uncert", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &lati_uncertanity, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid latitude uncertanity argument\r\n");
                dump_wlan_loc_cfg();
                return;
            }
        }
        else if (string_equal("lon_uncert", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &longi_uncertanity, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid longitude uncertanity argument\r\n");
                dump_wlan_loc_cfg();
                return;
            }
        }
        else if (string_equal("alt_uncert", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &alti_uncertanity, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid altitude uncertanity argument\r\n");
                dump_wlan_loc_cfg();
                return;
            }
        }
        else
        {
            (void)PRINTF("Error: invalid [%s] argument\r\n", argv[arg]);
            dump_wlan_loc_cfg();
            return;
        }
        arg += 2;
    }

    lci.lci_req   = lci_req;
    lci.longitude = longitude;
    lci.latitude  = latitude;
    lci.altitude  = altitude;
    lci.lat_unc   = lati_uncertanity;
    lci.long_unc  = longi_uncertanity;
    lci.alt_unc   = alti_uncertanity;

    wlan_ftm_location_cfg(&lci);
    return;
}

static void test_wlan_civ_cfg(int argc, char **argv)
{
    unsigned civ_req, loc_type, addr_type, addr_len, country_code;
    location_civic_rep_t lcr;
    int arg        = 1;
    char *civ_addr = CIVIC_ADDRESS;
    if (argc != 9)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_civ_cfg();
        return;
    }

    civ_req      = 0;
    loc_type     = CIVIC_LOCATION_TYPE;
    addr_type    = CIVIC_ADDRESS_TYPE;
    country_code = CIVIC_COUNTRY_CODE;

    while (arg < argc)
    {
        if (string_equal("civ_req", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &civ_req, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid number of civic require argument\r\n");
                dump_wlan_civ_cfg();
                return;
            }
        }
        else if (string_equal("loc_type", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &loc_type, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid location type argument\r\n");
                dump_wlan_civ_cfg();
                return;
            }
        }
        else if (string_equal("addr_type", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &addr_type, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid address type argument\r\n");
                dump_wlan_civ_cfg();
                return;
            }
        }
        else if (string_equal("country_code", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &country_code, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid address length argument\r\n");
                dump_wlan_civ_cfg();
                return;
            }
        }
        else
        {
            (void)PRINTF("Error: invalid [%s] argument\r\n", argv[arg]);
            dump_wlan_civ_cfg();
            return;
        }
        arg += 2;
    }

    lcr.civic_req           = civ_req;
    lcr.civic_location_type = loc_type;
    lcr.civic_address_type  = addr_type;
    lcr.country_code        = country_code;

    wlan_ftm_civic_cfg(&lcr);
    return;
}

static void test_wlan_11mc_nego_cfg(int argc, char **argv)
{
    unsigned burst_exp, burst_dur, min_delta, asap, ftm_per_burst, bw, burst_period;
    ftm_11mc_nego_cfg_t dot11mc_cfg;
    int arg        = 1;
    char *civ_addr = CIVIC_ADDRESS;
    if (argc != 13)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_11mc_nego_cfg();
        return;
    }

    burst_exp     = 0;
    burst_dur     = BURST_DURATION;
    min_delta     = MIN_DELTA;
    asap          = IS_ASAP;
    ftm_per_burst = FTM_PER_BURST;
    bw            = BW;
    burst_period  = BURST_PERIOD;

    while (arg < argc)
    {
        if (string_equal("burst_dur", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &burst_dur, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid value of burst duration argument\r\n");
                dump_wlan_11mc_nego_cfg();
                return;
            }
        }
        else if (string_equal("min_delta", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &min_delta, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid minimum delta argument\r\n");
                dump_wlan_11mc_nego_cfg();
                return;
            }
        }
        else if (string_equal("asap", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &asap, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid ASAP argument\r\n");
                dump_wlan_11mc_nego_cfg();
                return;
            }
        }
        else if (string_equal("ftm_per_burst", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &ftm_per_burst, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid frames per burst argument\r\n");
                dump_wlan_11mc_nego_cfg();
                return;
            }
        }
        else if (string_equal("BW", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &bw, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid bandwidth argument\r\n");
                dump_wlan_11mc_nego_cfg();
                return;
            }
        }
        else if (string_equal("burst_period", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &burst_period, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid burst period argument\r\n");
                dump_wlan_11mc_nego_cfg();
                return;
            }
        }
        else
        {
            (void)PRINTF("Error: invalid [%s] argument\r\n", argv[arg]);
            dump_wlan_11mc_nego_cfg();
            return;
        }
        arg += 2;
    }

    dot11mc_cfg.burst_exponent  = burst_exp;
    dot11mc_cfg.burst_duration  = burst_dur;
    dot11mc_cfg.min_delta_FTM   = min_delta;
    dot11mc_cfg.is_ASAP         = asap;
    dot11mc_cfg.per_burst_FTM   = ftm_per_burst;
    dot11mc_cfg.channel_spacing = bw;
    dot11mc_cfg.burst_period    = burst_period;

    wlan_ftm_11mc_cfg(&dot11mc_cfg);
    return;
}

static void test_wlan_11az_rang_cfg(int argc, char **argv)
{
    unsigned protocol, format_bw, num_measurements, measurement_freq, i2r_sts, r2i_sts, i2r_lmr, civic_req, lci_req;
    ranging_11az_cfg_t cfg_11az;
    int arg = 2;

    if (argc != 14)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_11az_rang_cfg();
        return;
    }

    if (get_uint(argv[1], &protocol, strlen(argv[1])))
    {
        (void)PRINTF("Error: invalid protocol argument\r\n");
        dump_wlan_11az_rang_cfg();
        return;
    }

    if (protocol != 0 && protocol != 1)
    {
        (void)PRINTF("Error: invalid protocol argument\r\n");
        dump_wlan_11az_rang_cfg();
        return;
    }

    format_bw        = FORMAT_BW;
    num_measurements = AZ_NUMBER_OF_MEASUREMENTS;
    measurement_freq = AZ_MEASUREMENT_FREQ;
    i2r_sts          = MAX_I2R_STS_UPTO80;
    r2i_sts          = MAX_R2I_STS_UPTO80;
    i2r_lmr          = I2R_LMR_FEEDBACK;
    civic_req        = CIVIC_REQUEST;
    lci_req          = LCI_REQUEST;

    while (arg < argc)
    {
        if (string_equal("num_measurements", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &num_measurements, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid number of measurements argument\r\n");
                dump_wlan_11az_rang_cfg();
                return;
            }
        }
        else if (string_equal("measurement_freq", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &measurement_freq, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid measurement frequency argument\r\n");
                dump_wlan_11az_rang_cfg();
                return;
            }
        }
        else if (string_equal("i2r_sts", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &i2r_sts, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid I2R STS argument\r\n");
                dump_wlan_11az_rang_cfg();
                return;
            }
        }
        else if (string_equal("r2i_sts", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &r2i_sts, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid R2I STS argument\r\n");
                dump_wlan_11az_rang_cfg();
                return;
            }
        }
        else if (string_equal("i2r_lmr", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &i2r_lmr, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid R2I STS argument\r\n");
                dump_wlan_11az_rang_cfg();
                return;
            }
        }
        else if (string_equal("format_bw", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &format_bw, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid format bandwidth argument\r\n");
                dump_wlan_11az_rang_cfg();
                return;
            }
        }
        else
        {
            (void)PRINTF("Error: invalid [%s] argument\r\n", argv[arg]);
            dump_wlan_11az_rang_cfg();
            return;
        }
        arg += 2;
    }

    cfg_11az.format_bw                 = format_bw;
    cfg_11az.max_i2r_sts_upto80        = i2r_sts;
    cfg_11az.max_r2i_sts_upto80        = r2i_sts;
    cfg_11az.az_measurement_freq       = measurement_freq;
    cfg_11az.az_number_of_measurements = num_measurements;
    cfg_11az.i2r_lmr_feedback          = i2r_lmr;

    cfg_11az.civic_req = civic_req;
    cfg_11az.lci_req   = lci_req;

    wlan_ftm_cfg(protocol, &cfg_11az);
    return;
}
#endif

#if CONFIG_WPA_SUPP
#if (CONFIG_11AX && defined(CONFIG_MBO))
static void test_wlan_mbo_non_prefer_chs(int argc, char **argv)
{
    int ret;

    if (argc != 2)
    {
        (void)PRINTF(
            "Usage: %s \"<oper_class>:<chan>:<preference>:<reason> <oper_class>:<chan>:<preference>:<reason>\"\r\n",
            argv[0]);
        return;
    }

    ret = wlan_mbo_peferch_cfg(argv[1]);

    if (ret == -WM_E_PERM)
    {
        (void)PRINTF("Please add pefer or non-pefer channels.\r\n");
    }
    else if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to add pefer or non-pefer channels.\r\n");
    }
    else
    {
        (void)PRINTF("Successfully set MBO non-preferred channels\r\n");
    }
}

static void test_wlan_mbo_set_cell_capa(int argc, char **argv)
{
    int ret;
    uint8_t cell_capa;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s <cell capa: 1/2/3(default)>\r\n", argv[0]);
        (void)PRINTF(
            "\tMBO Cellular Data Capabilities\r\n"
            "\t# 1 = Cellular data connection available\r\n"
            "\t# 2 = Cellular data connection not available\r\n"
            "\t# 3 = Not cellular capable (default)\r\n");
        return;
    }

    errno     = 0;
    cell_capa = (uint8_t)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan mbo cell_capa:%d\r\n", errno);
        return;
    }

    ret = wlan_mbo_set_cell_capa(cell_capa);

    if (ret == -WM_E_PERM)
    {
        (void)PRINTF("Please set correct mbo cell capa.\r\n");
    }
    else if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set mbo cell capa.\r\n");
    }
    else
    {
        /* Do nothing */
    }
}

static void test_wlan_mbo_set_oce(int argc, char **argv)
{
    int ret;
    uint8_t oce;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s <oce: 1(default)/2>\r\n", argv[0]);
        (void)PRINTF(
            "\tOptimized Connectivity Experience (OCE)\r\n"
            "\t# oce: Enable OCE features\r\n"
            "\t# 1 = Enable OCE in non-AP STA mode (default;\r\n"
            "\tdisabled if the driver does not indicate support for OCE in STA mode)\r\n"
            "\t# 2 = Enable OCE in STA-CFON mode\r\n");
        return;
    }

    errno = 0;
    oce   = (uint8_t)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan mbo oce:%d\r\n", errno);
        return;
    }

    ret = wlan_mbo_set_oce(oce);

    if (ret == -WM_E_PERM)
    {
        (void)PRINTF("Please set correct mbo oce.\r\n");
    }
    else if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set mbo oce.\r\n");
    }
    else
    {
        /* Do nothing */
    }
}
#endif

static void test_wlan_set_okc(int argc, char **argv)
{
    int ret;
    uint8_t okc;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s <okc: 0(default)/1>\r\n", argv[0]);
        (void)PRINTF(
            "\tOpportunistic Key Caching (also known as Proactive Key Caching) default\r\n"
            "\tThis parameter can be used to set the default behavior for the\r\n"
            "\tBy default, OKC is disabled unless enabled with the global okc=1 parameter \r\n"
            "\tor with the per-network proactive_key_caching=1 parameter.\r\n"
            "With okc=1, OKC is enabled by default, but can be disabled with per-network proactive_key_caching=0 "
            "parameter.\r\n"
            "\t# 0 = Disable OKC (default)\r\n"
            "\t# 1 = Enable OKC\r\n");
        return;
    }

    errno = 0;
    okc   = (uint8_t)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan okc:%d\r\n", errno);
        return;
    }

    ret = wlan_set_okc(okc);

    if (ret == -WM_E_PERM)
    {
        (void)PRINTF("Please set correct okc.\r\n");
    }
    else if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set okc.\r\n");
    }
    else
    {
        /* Do nothing */
    }
}

static void test_wlan_pmksa_list(int argc, char **argv)
{
    int ret;
    char *buf     = NULL;
    size_t buflen = 512;

    if (argc != 1)
    {
        (void)PRINTF("Usage: %s \r\n", argv[0]);
        return;
    }

    buf = (char *)OSA_MemoryAllocate(buflen);

    if (buf == NULL)
    {
        (void)PRINTF("Failed to alloc buffer\r\n");
        return;
    }

    ret = wlan_pmksa_list(buf, buflen);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("PMKSA list is empty\r\n");
    }
    else
    {
        (void)PRINTF("PMKSA list\r\n");
        (void)PRINTF("%s\r\n", buf);
    }
}

static void test_wlan_pmksa_flush(int argc, char **argv)
{
    int ret;

    if (argc != 1)
    {
        (void)PRINTF("Usage: %s \r\n", argv[0]);
        return;
    }

    ret = wlan_pmksa_flush();

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("failed to flush pmksa\r\n");
    }
    else
    {
        (void)PRINTF("Flushed PMKSA cache\r\n");
    }
}

static void test_wlan_set_scan_interval(int argc, char **argv)
{
    int ret, scan_int;

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s <scan_int: in seconds>\r\n", argv[0]);
        return;
    }

    errno    = 0;
    scan_int = (uint8_t)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:wlan scan int:%d\r\n", errno);
        return;
    }

    ret = wlan_set_scan_interval(scan_int);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set scan interval.\r\n");
    }
    else
    {
        (void)PRINTF("Scan interval set successfully.\r\n");
    }
}

#endif

#if CONFIG_UAP_STA_MAC_ADDR_FILTER
/**
 *  @brief Show usage information for the sta_filter_table command
 *
 *  $return         N/A
 */
static void print_sta_filter_table_usage(void)
{
    (void)PRINTF("\r\nUsage : sta_filter_table <FILTERMODE> <MACADDRESS_LIST>\r\n");
    (void)PRINTF("\r\nOptions: FILTERMODE : 0 - Disable filter table");
    (void)PRINTF("\r\n                      1 - allow MAC addresses specified in the allowed list");
    (void)PRINTF("\r\n                      2 - block MAC addresses specified in the banned list");
    (void)PRINTF("\r\n         MACADDRESS_LIST is the list of MAC addresses to be acted upon. Each");
    (void)PRINTF("\r\n                      MAC address must be separated with a space. Maximum of");
    (void)PRINTF("\r\n                      16 MAC addresses are supported.\r\n");
    return;
}

static void test_wlan_set_sta_filter(int argc, char **argv)
{
    int i           = 0;
    int ret         = WM_SUCCESS;
    int filter_mode = 0;
    int mac_count   = 0;
    unsigned char mac_addr[WLAN_MAX_STA_FILTER_NUM * WLAN_MAC_ADDR_LENGTH];

    if (argc < 2 || argc > (WLAN_MAX_STA_FILTER_NUM + 2))
    {
        (void)PRINTF("ERR:Too many or too few farguments.\r\n");
        print_sta_filter_table_usage();
        return;
    }

    argc--;
    argv++;

    if (((atoi(argv[0]) < 0) || (atoi(argv[0]) > 2)))
    {
        (void)PRINTF("ERR:Illegal FILTERMODE parameter %s. Must be either '0', '1', or '2'.\r\n", argv[1]);
        print_sta_filter_table_usage();
        return;
    }

    filter_mode = atoi(argv[0]);

    mac_count = argc - 1;

    if (mac_count)
    {
        for (i = 0; i < mac_count; i++)
        {
            ret = get_mac(argv[i + 1], (char *)&mac_addr[i * WLAN_MAC_ADDR_LENGTH], ':');
            if (ret != 0)
            {
                (void)PRINTF("Error: invalid MAC argument\r\n");
                return;
            }
        }
    }
    else
    {
        memset(mac_addr, 0, 16 * WLAN_MAC_ADDR_LENGTH);
    }

    wlan_set_sta_mac_filter(filter_mode, mac_count, mac_addr);

    return;
}
#endif

#if CONFIG_WIFI_GET_LOG
static void test_wlan_get_log(int argc, char **argv)
{
    wlan_pkt_stats_t stats;
    int ret, i;

    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <sta/uap> <ext>\r\n", argv[0]);
        return;
    }

    if (string_equal("sta", argv[1]))
        ret = wlan_get_log(&stats);
    else if (string_equal("uap", argv[1]))
        ret = wlan_uap_get_log(&stats);
    else
    {
        (void)PRINTF("Usage: %s <sta/uap> <ext>\r\n", argv[0]);
        return;
    }

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to get log\r\n");
    }
    else
    {
        (void)PRINTF(
            "dot11GroupTransmittedFrameCount    %u\r\n"
            "dot11FailedCount                   %u\r\n"
            "dot11RetryCount                    %u\r\n"
            "dot11MultipleRetryCount            %u\r\n"
            "dot11FrameDuplicateCount           %u\r\n"
            "dot11RTSSuccessCount               %u\r\n"
            "dot11RTSFailureCount               %u\r\n"
            "dot11ACKFailureCount               %u\r\n"
            "dot11ReceivedFragmentCount         %u\r\n"
            "dot11GroupReceivedFrameCount       %u\r\n"
            "dot11FCSErrorCount                 %u\r\n"
            "dot11TransmittedFrameCount         %u\r\n"
            "wepicverrcnt-1                     %u\r\n"
            "wepicverrcnt-2                     %u\r\n"
            "wepicverrcnt-3                     %u\r\n"
            "wepicverrcnt-4                     %u\r\n"
            "beaconReceivedCount                %u\r\n"
            "beaconMissedCount                  %u\r\n"
            "rxUnicastCount                     %u\r\n"
            "txOverrunCount                     %u\r\n"
            "rxOverrunCount                     %u\r\n",
            stats.mcast_tx_frame, stats.failed, stats.retry, stats.multi_retry, stats.frame_dup, stats.rts_success,
            stats.rts_failure, stats.ack_failure, stats.rx_frag, stats.mcast_rx_frame, stats.fcs_error, stats.tx_frame,
            stats.wep_icv_error[0], stats.wep_icv_error[1], stats.wep_icv_error[2], stats.wep_icv_error[3],
            stats.bcn_rcv_cnt, stats.bcn_miss_cnt, stats.rx_unicast_cnt, stats.tx_overrun_cnt, stats.rx_overrun_cnt);

        if (argc == 3 && !(strcmp(argv[2], "ext")))
        {
            (void)PRINTF(
                "rxStuckIssueCount-1                %u\r\n"
                "rxStuckIssueCount-2                %u\r\n"
                "rxStuckRecoveryCount               %u\r\n"
                "rxStuckTsf-1                       %llu\r\n"
                "rxStuckTsf-2                       %llu\r\n"
                "txWatchdogRecoveryCount            %u\r\n"
                "txWatchdogTsf-1                    %llu\r\n"
                "txWatchdogTsf-2                    %llu\r\n"
                "channelSwitchAnnouncementSent      %u\r\n"
                "channelSwitchState                 %u\r\n"
                "registerClass                      %u\r\n"
                "channelNumber                      %u\r\n"
                "channelSwitchMode                  %u\r\n"
                "RxResetRecoveryCount               %u\r\n"
                "RxIsr2NotDoneCnt                   %u\r\n"
                "gdmaAbortCnt                       %u\r\n"
                "gResetRxMacCnt                     %u\r\n"
                "gOwnrshpCtlErrCnt                  %u\r\n"
                "gOwnrshpBcnErrCnt                  %u\r\n"
                "gOwnrshpMgtErrCnt                  %u\r\n"
                "gOwnrshpDatErrCnt                  %u\r\n"
                "bigtk_mmeGoodCnt                   %u\r\n"
                "bigtk_replayErrCnt                 %u\r\n"
                "bigtk_micErrCnt                    %u\r\n"
                "bigtk_mmeNotFoundCnt               %u\r\n",
                stats.rx_stuck_issue_cnt[0], stats.rx_stuck_issue_cnt[1], stats.rx_stuck_recovery_cnt,
                stats.rx_stuck_tsf[0], stats.rx_stuck_tsf[1], stats.tx_watchdog_recovery_cnt, stats.tx_watchdog_tsf[0],
                stats.tx_watchdog_tsf[1], stats.channel_switch_ann_sent, stats.channel_switch_state, stats.reg_class,
                stats.channel_number, stats.channel_switch_mode, stats.rx_reset_mac_recovery_cnt,
                stats.rx_Isr2_NotDone_Cnt, stats.gdma_abort_cnt, stats.g_reset_rx_mac_cnt, stats.dwCtlErrCnt,
                stats.dwBcnErrCnt, stats.dwMgtErrCnt, stats.dwDatErrCnt, stats.bigtk_mmeGoodCnt,
                stats.bigtk_replayErrCnt, stats.bigtk_micErrCnt, stats.bigtk_mmeNotFoundCnt);
        }

        (void)PRINTF("dot11TransmittedFragmentCount      %u\r\n", stats.tx_frag_cnt);
        (void)PRINTF("dot11QosTransmittedFragmentCount   ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_tx_frag_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosFailedCount                ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_failed_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosRetryCount                 ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_retry_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosMultipleRetryCount         ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_multi_retry_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosFrameDuplicateCount        ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_frm_dup_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosRTSSuccessCount            ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_rts_suc_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosRTSFailureCount            ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_rts_failure_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosACKFailureCount            ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_ack_failure_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosReceivedFragmentCount      ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_rx_frag_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosTransmittedFrameCount      ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_tx_frm_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosDiscardedFrameCount        ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_discarded_frm_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosMPDUsReceivedCount         ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_mpdus_rx_cnt[i]);
        }
        (void)PRINTF("\r\ndot11QosRetriesReceivedCount       ");
        for (i = 0; i < 8; i++)
        {
            (void)PRINTF("%u ", stats.qos_retries_rx_cnt[i]);
        }
        (void)PRINTF(
            "\r\ndot11RSNAStatsCMACICVErrors          %u\r\n"
            "dot11RSNAStatsCMACReplays            %u\r\n"
            "dot11RSNAStatsRobustMgmtCCMPReplays  %u\r\n"
            "dot11RSNAStatsTKIPICVErrors          %u\r\n"
            "dot11RSNAStatsTKIPReplays            %u\r\n"
            "dot11RSNAStatsCCMPDecryptErrors      %u\r\n"
            "dot11RSNAstatsCCMPReplays            %u\r\n"
            "dot11TransmittedAMSDUCount           %u\r\n"
            "dot11FailedAMSDUCount                %u\r\n"
            "dot11RetryAMSDUCount                 %u\r\n"
            "dot11MultipleRetryAMSDUCount         %u\r\n"
            "dot11TransmittedOctetsInAMSDUCount   %llu\r\n"
            "dot11AMSDUAckFailureCount            %u\r\n"
            "dot11ReceivedAMSDUCount              %u\r\n"
            "dot11ReceivedOctetsInAMSDUCount      %llu\r\n"
            "dot11TransmittedAMPDUCount           %u\r\n"
            "dot11TransmittedMPDUsInAMPDUCount    %u\r\n"
            "dot11TransmittedOctetsInAMPDUCount   %llu\r\n"
            "dot11AMPDUReceivedCount              %u\r\n"
            "dot11MPDUInReceivedAMPDUCount        %u\r\n"
            "dot11ReceivedOctetsInAMPDUCount      %llu\r\n"
            "dot11AMPDUDelimiterCRCErrorCount     %u\r\n",
            stats.cmacicv_errors, stats.cmac_replays, stats.mgmt_ccmp_replays, stats.tkipicv_errors, stats.tkip_replays,
            stats.ccmp_decrypt_errors, stats.ccmp_replays, stats.tx_amsdu_cnt, stats.failed_amsdu_cnt,
            stats.retry_amsdu_cnt, stats.multi_retry_amsdu_cnt, stats.tx_octets_in_amsdu_cnt,
            stats.amsdu_ack_failure_cnt, stats.rx_amsdu_cnt, stats.rx_octets_in_amsdu_cnt, stats.tx_ampdu_cnt,
            stats.tx_mpdus_in_ampdu_cnt, stats.tx_octets_in_ampdu_cnt, stats.ampdu_rx_cnt, stats.mpdu_in_rx_ampdu_cnt,
            stats.rx_octets_in_ampdu_cnt, stats.ampdu_delimiter_crc_error_cnt);
    }
}
#endif

static void dump_multiple_mef_config_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-multi-mef <ping/arp/multicast/ns/del> [<action>]\r\n");
    (void)PRINTF("        ping/arp/multicast/ns\r\n");
    (void)PRINTF("                 -- MEF entry type, will add one mef entry at a time\r\n");
    (void)PRINTF("        del      -- Delete all previous MEF entries\r\n");
    (void)PRINTF("        action   -- 0--discard and not wake host\r\n");
    (void)PRINTF("                    1--discard and wake host\r\n");
    (void)PRINTF("                    3--allow and wake host\r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("    wlan-multi-mef ping 3\r\n");
    (void)PRINTF("    wlan-multi-mef del\r\n");
}

static void test_wlan_set_multiple_mef_config(int argc, char **argv)
{
    int type        = MEF_TYPE_END;
    t_u8 mef_action = 0;
    if (argc < 2)
    {
        dump_multiple_mef_config_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    /* Delete previous MEF configure */
    if (argc == 2)
    {
        if (string_equal("del", argv[1]))
            type = MEF_TYPE_DELETE;
        else
        {
            dump_multiple_mef_config_usage();
            (void)PRINTF("Error: invalid mef type\r\n");
            return;
        }
    }
    /* Add MEF entry */
    else if (argc >= 3)
    {
        if (string_equal("ping", argv[1]))
        {
            type       = MEF_TYPE_PING;
            mef_action = (t_u8)atoi(argv[2]);
        }
        else if (string_equal("arp", argv[1]))
        {
            type       = MEF_TYPE_ARP;
            mef_action = (t_u8)atoi(argv[2]);
        }
        else if (string_equal("multicast", argv[1]))
        {
            type       = MEF_TYPE_MULTICAST;
            mef_action = (t_u8)atoi(argv[2]);
        }
        else if (string_equal("ns", argv[1]))
        {
            type       = MEF_TYPE_IPV6_NS;
            mef_action = (t_u8)atoi(argv[2]);
        }
        else
        {
            (void)PRINTF("Error: invalid mef type\r\n");
            return;
        }
    }
    wlan_config_mef(type, mef_action);
}

#if CONFIG_HOST_SLEEP
#ifdef RW610
static void test_wlan_wakeup_condition(int argc, char **argv)
{
    uint8_t is_mef = MFALSE;
    uint32_t wake_up_conds = 0;

    if (argc < 2 || argc > 3)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("    wlan-wakeup-condition <mef/wowlan wake_up_conds>\r\n");
        (void)PRINTF("    wowlan -- default host wakeup\r\n");
        (void)PRINTF("    [wake_up_conds] -- value for wowlan host wakeup conditions only\r\n");
        (void)PRINTF("	       bit 0: WAKE_ON_ALL_BROADCAST\r\n");
        (void)PRINTF("	       bit 1: WAKE_ON_UNICAST\r\n");
        (void)PRINTF("	       bit 2: WAKE_ON_MAC_EVENT\r\n");
        (void)PRINTF("	       bit 3: WAKE_ON_MULTICAST\r\n");
        (void)PRINTF("	       bit 4: WAKE_ON_ARP_BROADCAST\r\n");
        (void)PRINTF("	       bit 6: WAKE_ON_MGMT_FRAME\r\n");
        (void)PRINTF("	       All bit 0 discard and not wakeup host\r\n");
        (void)PRINTF("    mef     -- MEF host wakeup\r\n");
        (void)PRINTF("Example:\r\n");
        (void)PRINTF("    wlan-wakeup-condition mef\r\n");
        (void)PRINTF("    wlan-wakeup-condition wowlan 0x1e\r\n");
        return;
    }
    if (string_equal("mef", argv[1]))
    {
        is_mef = MTRUE;
    }
    else if (string_equal("wowlan", argv[1]))
    {
        if (argc < 3)
        {
            (void)PRINTF("wake_up_conds need be specified\r\n");
            return;
        }

        errno         = 0;
        wake_up_conds = (int)strtol(argv[2], NULL, 16);
        if (errno != 0)
        {
            (void)PRINTF("Error during strtol:wake_up_conds errno:%d\r\n", errno);
            return;
        }
    }
    else
    {
        (void)PRINTF("wowlan/mef need be specified\r\n");
        return;
    }
    wlan_wowlan_config(is_mef, wake_up_conds);
    return;
}
#endif /*RW610*/

extern wlan_flt_cfg_t g_flt_cfg;

#ifndef RW610
static void test_wlan_host_sleep(int argc, char **argv)
{
    int choice = -1, wowlan = 0;
    int ret = -WM_FAIL;

    if ((argc < 2) || (argc > 4))
    {
        goto done;
    }

    errno  = 0;
    choice = (int)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:host_sleep errno:%d\r\n", errno);
        goto done;
    }
    if ((choice != 0) && (choice != 1))
    {
        goto done;
    }

    if (choice == 0)
    {
        ret = wlan_send_host_sleep(HOST_SLEEP_CFG_CANCEL);
        if (ret == WM_SUCCESS)
        {
            (void)PRINTF("Cancel Previous configured Host sleep configuration");
        }
        else
        {
            (void)PRINTF("Failed to Cancel Previous configured Host sleep configuration, error: %d", ret);
        }
    }
    else if (choice == 1)
    {
        if (argc < 3)
        {
            goto done;
        }

        if (string_equal(argv[2], "wowlan"))
        {
            errno  = 0;
            wowlan = (int)strtol(argv[3], NULL, 16);
            if (errno != 0)
            {
                (void)PRINTF("Error during strtol:wowlan errno:%d\r\n", errno);
                return;
            }

            ret = wlan_send_host_sleep(wowlan);
            if (ret == WM_SUCCESS)
            {
                (void)PRINTF("Host sleep configuration req sent");
            }
            else
            {
                (void)PRINTF("Failed to host sleep configuration, error: %d", ret);
            }
        }
        else if (string_equal(argv[2], "mef"))
        {
            ret = wlan_send_host_sleep(HOST_SLEEP_NO_COND);
            if (ret == WM_SUCCESS)
            {
                (void)PRINTF("Host sleep configuration successs with MEF");
            }
            else
            {
                (void)PRINTF("Failed to host sleep configuration, error: %d", ret);
            }
        }
        else
        {
            goto done;
        }
    }
    else
    {
done:
        (void)PRINTF("Error: invalid number of arguments\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("    wlan-host-sleep <1/0> <wowlan [wake_up_conds]/mef>\r\n");
        (void)PRINTF("    [wake_up_conds] -- value for host wakeup conditions\r\n");
        (void)PRINTF("	       bit 0: WAKE_ON_ALL_BROADCAST\r\n");
        (void)PRINTF("	       bit 1: WAKE_ON_UNICAST\r\n");
        (void)PRINTF("	       bit 2: WAKE_ON_MAC_EVENT\r\n");
        (void)PRINTF("	       bit 3: WAKE_ON_MULTICAST\r\n");
        (void)PRINTF("	       bit 4: WAKE_ON_ARP_BROADCAST\r\n");
        (void)PRINTF("	       bit 6: WAKE_ON_MGMT_FRAME\r\n");
        (void)PRINTF("	       All bit 0 discard and not wakeup host\r\n");
        (void)PRINTF("	       All bit 1 cancel host sleep configuration\r\n");
        (void)PRINTF("    mef     -- MEF host wakeup\r\n");
        (void)PRINTF("Example:\r\n");
        (void)PRINTF("    wlan-host-sleep <1/0> mef\r\n");
        (void)PRINTF("    wlan-host-sleep <1/0> wowlan 0x1e\r\n");
        (void)PRINTF("    wlan-host-sleep <1/0> wowlan 0x1e\r\n");
        return;
    }
}
#endif

#ifdef RW610
#if !(CONFIG_WIFI_BLE_COEX_APP)
#ifdef __ZEPHYR__
static void test_wlan_auto_host_sleep(int argc, char **argv)
{
    bool is_manual   = MFALSE;
    t_u8 is_periodic = 0;
    t_u8 enable      = 0;

    if (argc > 3 || argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("    wlan-auto-host-sleep <enable> <periodic>\r\n");
        (void)PRINTF("    enable      -- enable/disable host sleep\r\n");
        (void)PRINTF("                   0 - disable host sleep\r\n");
        (void)PRINTF("                   1 - enable host sleep\r\n");
        (void)PRINTF("    periodic    -- Host enter low power periodically or oneshot\r\n");
        (void)PRINTF(
            "                            0 - Oneshot. Host will enter low power only once and keep full power after "
            "waking "
            "up.\r\n");
        (void)PRINTF("                   1 - Periodic. Host will enter low power periodically.\r\n");
        (void)PRINTF("Examples:\r\n");
        (void)PRINTF("    wlan-auto-host-sleep 1 1\r\n");
        (void)PRINTF("    wlan-auto-host-sleep 0\r\n");
        return;
    }
    enable = (t_u8)atoi(argv[1]);
    if (enable != 0 && enable != 1)
    {
        (void)PRINTF("Error! Invalid input of parameter <enable>\r\n");
        return;
    }
    /* Disable auto host sleep */
    if (enable == 0)
    {
        wlan_cancel_host_sleep();
        wlan_clear_host_sleep_config();
        (void)PRINTF("Auto Host Sleep disabled\r\n");
        return;
    }
    is_periodic = (t_u8)atoi(argv[2]);
    wlan_config_host_sleep(is_manual, is_periodic);
}
#else
static void test_wlan_auto_host_sleep(int argc, char **argv)
{
    bool is_manual    = MFALSE;
    int rtc_timeout_s = 0;
    t_u8 is_periodic  = 0;
    t_u8 enable       = 0;

    if (argc > 5 || argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("    wlan-auto-host-sleep <enable> <mode> <rtc_timeout> <periodic>\r\n");
        (void)PRINTF("    enable      -- enable/disable host sleep\r\n");
        (void)PRINTF("                   0 - disable host sleep\r\n");
        (void)PRINTF("                   1 - enable host sleep\r\n");
        (void)PRINTF("    mode        -- Mode of how host enter low power.\r\n");
        (void)PRINTF("                   manual - Manual mode. Need to use suspend command to enter low power.\r\n");
        (void)PRINTF("                   pm     - Power Manager.");
        (void)PRINTF("    rtc_timeout -- RTC timer value. Unit is second.\r\n");
        (void)PRINTF("    periodic    -- Host enter low power periodically or oneshot\r\n");
        (void)PRINTF(
            "                   0 - Oneshot. Host will enter low power only once and keep full power after waking "
            "up.\r\n");
        (void)PRINTF("                   1 - Periodic. Host will enter low power periodically.\r\n");
        (void)PRINTF("    Parameters <rtc_timer> and <periodic> are for Power Manager ONLY!\r\n");
        (void)PRINTF("Examples:\r\n");
        (void)PRINTF("    wlan-auto-host-sleep 1 pm 60 1\r\n");
        (void)PRINTF("    wlan-auto-host-sleep 1 pm 5 0\r\n");
        (void)PRINTF("    wlan-auto-host-sleep 1 manual\r\n");
        (void)PRINTF("    wlan-auto-host-sleep 0\r\n");
        return;
    }

    enable = (t_u8)atoi(argv[1]);
    if (enable != 0 && enable != 1)
    {
        (void)PRINTF("Error! Invalid input of parameter <enable>\r\n");
        return;
    }
    /* Disable auto host sleep */
    if (enable == 0)
    {
        wlan_cancel_host_sleep();
        wlan_clear_host_sleep_config();
        (void)PRINTF("Auto Host Sleep disabled\r\n");
        return;
    }
    if (string_equal("manual", argv[2]))
    {
        is_manual = MTRUE;
    }
    else if (string_equal("pm", argv[2]))
    {
        if (argc != 5)
        {
            (void)PRINTF("Error!Invalid number of inputs! Need to specify both <rtc_timeout> and <periodic>\r\n");
            return;
        }
        rtc_timeout_s = atoi(argv[3]);
        if (rtc_timeout_s == 0)
        {
            (void)PRINTF("Error!Invalid value of <rtc_timeout>!\r\n");
            return;
        }
        rtc_timeout = rtc_timeout_s * 1000000;
        is_periodic = (t_u8)atoi(argv[4]);
    }
    else
    {
        (void)PRINTF("Invalid input!\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("    wlan-auto-host-sleep <enable> <mode> <rtc_timer> <periodic>\r\n");
        return;
    }
    (void)PRINTF("%s is selected for host sleep\r\n", is_manual ? "Manual mode" : "Power Manager");
    if (!is_manual)
        (void)PRINTF("Host will enter low power %s\r\n", is_periodic ? "periodically" : "only once");
    wlan_config_host_sleep(is_manual, is_periodic);
}
#endif
#else
static void test_wlan_ns_offload(int argc, char **argv)
{
    int ret = -WM_FAIL;
    ret     = wlan_set_ipv6_ns_offload();
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Enabled  wlan IPv6 NS offload feature");
    }
    else
    {
        (void)PRINTF("Failed to enabled wlan auto arp offload, error: %d", ret);
    }
}

static void test_wlan_auto_arp(int argc, char **argv)
{
    int ret = -WM_FAIL;
    ret     = wlan_set_auto_arp();
    if (ret == WM_SUCCESS)
        (void)PRINTF("Enabled  wlan auto arp offload feature\r\n");
    else
        (void)PRINTF("Failed to enabled wlan auto arp offload, error: %d\r\n", ret);
}

static void dump_wlan_add_packet_filter()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("For wowlan Add packet filter\r\n");
    (void)PRINTF("wowlan magic filter:\r\n");
    (void)PRINTF("wlan_add_packet_filter 1:\r\n");
    (void)PRINTF("wowlan User defined pattren packet filter:\r\n");
    (void)PRINTF("wlan_add_packet_filter 0 <number of patterns> <ptn_len> <pkt_offset> <ptn> ........:\r\n");
    (void)PRINTF(
        "For 2 number of patterns Usage \r\nwlan_add_packet_filter 0 2 6 0 0xff 0xff 0xff 0xff 0xff 0xff 4 20 192 168 "
        "10 1\r\n");
    (void)PRINTF("wowlan User defined pattren and magic packet filter:\r\n");
    (void)PRINTF("wlan_add_packet_filter 1 <number of patterns> <ptn_len> <pkt_offset> <ptn> ........:\r\n");
    (void)PRINTF(
        "For 2 number of patterns Usage \r\nwlan_add_packet_filter 1 2 6 0 0xff 0xff 0xff 0xff 0xff 0xff 4 20 192 168 "
        "10 1\r\n");
}

static void test_wlan_add_packet_filter(int argc, char **argv)
{
    int ret = -WM_FAIL;
    t_u8 i = 0, j = 0, k = 0;
    wlan_wowlan_ptn_cfg_t wowlan_ptn_cfg;
    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <0/1>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 1 to magic filter\r\n");
        dump_wlan_add_packet_filter();
        return;
    }
    if (argc > 3 && atoi(argv[2]) != argc - 3)
    {
        (void)PRINTF("Usage: %s 0/1 <patterns number> <ptn_len> <pkt_offset> <ptn> ...........\r\n", argv[0]);
        dump_wlan_add_packet_filter();
        return;
    }
    (void)memset(&wowlan_ptn_cfg, 0, sizeof(wlan_wowlan_ptn_cfg_t));
    wowlan_ptn_cfg.enable = atoi(argv[1]);
    if (argc > 2)
    {
        wowlan_ptn_cfg.n_patterns = atoi(argv[2]);
        for (i = 0, k = 0; (i + 3 < argc) && k < MAX_NUM_FILTERS; k++)
        {
            wowlan_ptn_cfg.patterns[k].pattern_len = atoi(argv[i + 3]);
            i++;
            wowlan_ptn_cfg.patterns[k].pkt_offset = atoi(argv[i + 3]);
            i++;
            for (j = 0; j < wowlan_ptn_cfg.patterns[k].pattern_len; j++)
                wowlan_ptn_cfg.patterns[k].pattern[j] = atoi(argv[j + i + 3]);
            i = +j;
            (void)memset(wowlan_ptn_cfg.patterns[k].mask, 0x3f, 6);
        }
    }
    ret = wlan_wowlan_cfg_ptn_match(&wowlan_ptn_cfg);
    if (ret == WM_SUCCESS)
        (void)PRINTF("Enabled pkt filter offload feature");
    else
        (void)PRINTF("Failed to enabled magic pkt filter offload, error: %d", ret);
}
#endif /*RW610*/
#else
static void test_wlan_ns_offload(int argc, char **argv)
{
    int ret = -WM_FAIL;
    ret     = wlan_set_ipv6_ns_offload();
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Enabled  wlan IPv6 NS offload feature");
    }
    else
    {
        (void)PRINTF("Failed to enabled wlan auto arp offload, error: %d", ret);
    }
}

static void test_wlan_auto_arp(int argc, char **argv)
{
    int ret = -WM_FAIL;
    ret     = wlan_set_auto_arp();
    if (ret == WM_SUCCESS)
        (void)PRINTF("Enabled  wlan auto arp offload feature\r\n");
    else
        (void)PRINTF("Failed to enabled wlan auto arp offload, error: %d\r\n", ret);
}

static void dump_wlan_add_packet_filter()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("For wowlan Add packet filter\r\n");
    (void)PRINTF("wowlan magic filter:\r\n");
    (void)PRINTF("wlan_add_packet_filter 1:\r\n");
    (void)PRINTF("wowlan User defined pattren packet filter:\r\n");
    (void)PRINTF("wlan_add_packet_filter 0 <number of patterns> <ptn_len> <pkt_offset> <ptn> ........:\r\n");
    (void)PRINTF(
        "For 2 number of patterns Usage \r\nwlan_add_packet_filter 0 2 6 0 0xff 0xff 0xff 0xff 0xff 0xff 4 20 192 168 "
        "10 1\r\n");
    (void)PRINTF("wowlan User defined pattren and magic packet filter:\r\n");
    (void)PRINTF("wlan_add_packet_filter 1 <number of patterns> <ptn_len> <pkt_offset> <ptn> ........:\r\n");
    (void)PRINTF(
        "For 2 number of patterns Usage \r\nwlan_add_packet_filter 1 2 6 0 0xff 0xff 0xff 0xff 0xff 0xff 4 20 192 168 "
        "10 1\r\n");
}

static void test_wlan_add_packet_filter(int argc, char **argv)
{
    int ret = -WM_FAIL;
    t_u8 i = 0, j = 0, k = 0;
    wlan_wowlan_ptn_cfg_t wowlan_ptn_cfg;
    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <0/1>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 1 to magic filter\r\n");
        dump_wlan_add_packet_filter();
        return;
    }
    if (argc > 3 && atoi(argv[2]) != argc - 3)
    {
        (void)PRINTF("Usage: %s 0/1 <patterns number> <ptn_len> <pkt_offset> <ptn> ...........\r\n", argv[0]);
        dump_wlan_add_packet_filter();
        return;
    }
    (void)memset(&wowlan_ptn_cfg, 0, sizeof(wlan_wowlan_ptn_cfg_t));
    wowlan_ptn_cfg.enable = atoi(argv[1]);
    if (argc > 2)
    {
        wowlan_ptn_cfg.n_patterns = atoi(argv[2]);
        for (i = 0, k = 0; (i + 3 < argc) && k < MAX_NUM_FILTERS; k++)
        {
            wowlan_ptn_cfg.patterns[k].pattern_len = atoi(argv[i + 3]);
            i++;
            wowlan_ptn_cfg.patterns[k].pkt_offset = atoi(argv[i + 3]);
            i++;
            for (j = 0; j < wowlan_ptn_cfg.patterns[k].pattern_len; j++)
                wowlan_ptn_cfg.patterns[k].pattern[j] = atoi(argv[j + i + 3]);
            i = +j;
            (void)memset(wowlan_ptn_cfg.patterns[k].mask, 0x3f, 6);
        }
    }
    ret = wlan_wowlan_cfg_ptn_match(&wowlan_ptn_cfg);
    if (ret == WM_SUCCESS)
        (void)PRINTF("Enabled pkt filter offload feature");
    else
        (void)PRINTF("Failed to enabled magic pkt filter offload, error: %d", ret);
}
#endif /*RW610*/
#endif /* CONFIG_HOST_SLEEP */

#if CONFIG_SEND_HOSTCMD
#define HOSTCMD_RESP_BUFF_SIZE 1024
static uint8_t host_cmd_resp_buf[HOSTCMD_RESP_BUFF_SIZE] = {0};
/* Command taken from robust_btc.conf*/
static uint8_t host_cmd_buf[] = {0xe0, 0, 0x12, 0, 0x3c, 0, 0, 0, 0x01, 0, 0, 0, 0x38, 0x02, 0x02, 0, 0x07, 0x01};

static void test_wlan_send_hostcmd(int argc, char **argv)
{
    int ret           = -WM_FAIL;
    uint32_t reqd_len = 0;
    uint32_t len;

    ret = wlan_send_hostcmd(host_cmd_buf, sizeof(host_cmd_buf) / sizeof(uint8_t), host_cmd_resp_buf,
                            HOSTCMD_RESP_BUFF_SIZE, &reqd_len);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Hostcmd success, response is");
        for (len = 0; len < reqd_len; len++)
        {
            (void)PRINTF("%x\t", host_cmd_resp_buf[len]);
        }
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}
#endif

#if defined(RW610) || defined(SD9177)
static void test_wlan_ext_coex_uwb_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-ext-coex-uwb \r\n");
    (void)PRINTF("        - Enable UWB Coex\r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("    wlan-ext-coex-uwb \r\n");
    (void)PRINTF("    - Enable UWB Coex \r\n");
}

static void test_wlan_ext_coex_uwb(int argc, char **argv)
{
    int ret           = -WM_FAIL;
    uint32_t reqd_len = 0;

    t_u8 cmd_buf[]    = {0xe0, 0x00, 0x11, 0x00, 0x4a, 0x00, 0x00, 0x00, 0x01 /* Get/Set */,
                         0x00, 0x00, 0x00, 0x38, 0x02, 0x01, 0x00, 0x03
                        };
    t_u8 resp_buf[64] = {0};

    /**
     * Command taken from robust_btc.conf
     *    external_coex_uwb_config={
     *        CmdCode=0x00e0
     *        Action:2=1                          # 0x0 get, 0x1 set
     *        RSVD:2=0
     *        RobustCoexTlvType:2=0x0238          # TLV ID
     *        RobustCoexTlvLength:2={
     *            Enabled:1=0x03                  # 0x03 to configure UWB
     *        }
     *}
     */

    if (argc > 1)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        test_wlan_ext_coex_uwb_usage();
        return;
    }

    ret = wlan_send_hostcmd(cmd_buf, sizeof(cmd_buf) / sizeof(t_u8), resp_buf, sizeof(resp_buf), &reqd_len);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Hostcmd success, response is\r\n");
        for (ret = 0; ret < reqd_len; ret++)
            (void)PRINTF("%x\t", resp_buf[ret]);
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}
#endif

#ifdef SD8801
static void test_wlan_8801_enable_ext_coex(int argc, char **argv)
{
    int ret = -WM_FAIL;
    wlan_ext_coex_config_t ext_coex_config;

    ext_coex_config.Enabled                        = 1;
    ext_coex_config.IgnorePriority                 = 0;
    ext_coex_config.DefaultPriority                = 0;
    ext_coex_config.EXT_RADIO_REQ_ip_gpio_num      = 3;
    ext_coex_config.EXT_RADIO_REQ_ip_gpio_polarity = 1;
    ext_coex_config.EXT_RADIO_PRI_ip_gpio_num      = 2;
    ext_coex_config.EXT_RADIO_PRI_ip_gpio_polarity = 1;
    ext_coex_config.WLAN_GRANT_op_gpio_num         = 1;
    ext_coex_config.WLAN_GRANT_op_gpio_polarity    = 0;
    ext_coex_config.reserved_1                     = 0x28;
    ext_coex_config.reserved_2                     = 0x3c;

    ret = wlan_set_ext_coex_config(ext_coex_config);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("8801 External Coex Config set successfully");
    }
    else
    {
        (void)PRINTF("8801 External Coex Config error: %d", ret);
    }
}

static void test_wlan_8801_ext_coex_stats(int argc, char **argv)
{
    int ret = -WM_FAIL;
    wlan_ext_coex_stats_t ext_coex_stats;

    ret = wlan_get_ext_coex_stats(&ext_coex_stats);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get external Coex statistics\r\n");
    }
    else
    {
        (void)PRINTF("BLE_EIP: %d, BLE_PRI: %d, WLAN_EIP: %d\r\n", ext_coex_stats.ext_radio_req_count,
                     ext_coex_stats.ext_radio_pri_count, ext_coex_stats.wlan_grant_count);
    }
}
#endif

#if !defined(SD8801) && !defined(RW610)
static void test_wlan_set_uap_bandwidth(int argc, char **argv)
{
    uint8_t bandwidth;
    int ret = -WM_FAIL;

    if (argc < 2)
    {
#if CONFIG_11AC
        (void)PRINTF("Usage: %s <1/2/3>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 1 to set bandwidth 20MHz or 2 for 40MHz or 3 for 80MHz\r\n");
#else
        (void)PRINTF("Usage: %s <1/2>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 1 to set bandwidth 20MHz or 2 for 40MHz\r\n");
#endif
        return;
    }

    errno     = 0;
    bandwidth = (uint8_t)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:uap_bandwidth errno:%d\r\n", errno);
        return;
    }

    ret = wlan_uap_set_bandwidth(bandwidth);

    if (ret != WM_SUCCESS)
    {
#if CONFIG_11AC
        (void)PRINTF("Usage: %s <1/2/3>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 1 to set bandwidth 20MHz or 2 for 40MHz or 3 for 80MHz\r\n");
#else
        (void)PRINTF("Usage: %s <1/2>\r\n", argv[0]);
        (void)PRINTF("Error: Specify 1 to set bandwidth 20MHz or 2 for 40MHz\r\n");
#endif
    }
    else
    {
        (void)PRINTF("bandwidth set successfully\r\n");
    }
}
#endif

static void dump_hidden_ssid_usage()
{
    (void)PRINTF("Usage: wlan-set-uap-hidden-ssid <0/1/2>\r\n");
    (void)PRINTF(
        "Error: 0: broadcast SSID in beacons.\r\n"
        "1: send empty SSID (length=0) in beacons.\r\n"
        "2: clear SSID (ACSII 0), but keep the original length\r\n");
}

static void test_wlan_set_uap_hidden_ssid(int argc, char **argv)
{
    uint8_t hidden_ssid;
    int ret = -WM_FAIL;

    if (argc < 2)
    {
        dump_hidden_ssid_usage();
        return;
    }

    errno       = 0;
    hidden_ssid = (uint8_t)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:uap_bandwidth errno:%d\r\n", errno);
        return;
    }

    ret = wlan_uap_set_hidden_ssid(hidden_ssid);

    if (ret != WM_SUCCESS)
    {
        dump_hidden_ssid_usage();
    }
    else
    {
        (void)PRINTF("SSID broadcast control set successfully\r\n");
    }
}

#if CONFIG_WIFI_MEM_ACCESS
static void dump_wlan_mem_access_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Get value of memory:\r\n");
    (void)PRINTF("    wlan-mem-access <memeory_address>\r\n");
    (void)PRINTF("Set value of memory:\r\n");
    (void)PRINTF("    wlan-mem-access <memeory_address> <value>\r\n");
    (void)PRINTF("The format of memory address and value:\r\n");
    (void)PRINTF(
        "    Hexadecimal value. For example:\r\n"
        "        0x00001200\r\n"
        "        0X00001200\r\n"
        "        0x1200\r\n"
        "        0X1200\r\n");
}

static void test_wlan_mem_access(int argc, char **argv)
{
    int ret;
    uint16_t action  = 0;
    uint32_t address = 0;
    uint32_t value   = 0;
    if (argc < 2 || argc > 3)
    {
        dump_wlan_mem_access_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    else if (argc == 2)
        action = ACTION_GET;
    else
    {
        action = ACTION_SET;
        if (argv[2][0] == '0' && (argv[2][1] == 'x' || argv[2][1] == 'X'))
            value = a2hex_or_atoi(argv[2]);
        else
        {
            dump_wlan_mem_access_usage();
            (void)PRINTF("Error: invalid value argument\r\n");
            return;
        }
    }
    if (argv[1][0] == '0' && (argv[1][1] == 'x' || argv[1][1] == 'X'))
        address = a2hex_or_atoi(argv[1]);
    else
    {
        dump_wlan_mem_access_usage();
        (void)PRINTF("Error: invalid address argument\r\n");
        return;
    }

    ret = wlan_mem_access(action, address, &value);

    if (ret == WM_SUCCESS)
    {
        if (action == ACTION_GET)
            (void)PRINTF("At Memory 0x%x: 0x%x\r\n", address, value);
        else
            (void)PRINTF("Set the Memory successfully\r\n");
    }
    else
        wlcm_e("Read/write Mem failed");
}
#endif

#if CONFIG_WIFI_BOOT_SLEEP
static void dump_wlan_boot_sleep_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Get boot sleep status:\r\n");
    (void)PRINTF("    wlan-boot-sleep \r\n");
    (void)PRINTF("Set boot sleep:\r\n");
    (void)PRINTF("    wlan-boot-sleep <0/1>\r\n");
}

static void test_wlan_boot_sleep(int argc, char **argv)
{
    int ret;
    uint16_t action = 0;
    uint16_t enable = 0;
    if (argc < 1 || argc > 2)
    {
        dump_wlan_boot_sleep_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    else if (argc == 1)
        action = ACTION_GET;
    else
    {
        action = ACTION_SET;
        enable = a2hex_or_atoi(argv[1]);
    }

    ret = wlan_boot_sleep(action, &enable);

    if (ret == WM_SUCCESS)
    {
        if (action == ACTION_GET)
            (void)PRINTF("Boot sleep status: %d\r\n", enable);
        else
            (void)PRINTF("Boot sleep status is: %s\r\n", enable == 1 ? "Enabled" : "Disabled");
    }
    else
    {
        dump_wlan_boot_sleep_usage();
        wlcm_e("Wlan boot sleep failed");
    }
}
#endif

#if (CONFIG_11R)
static void dump_wlan_ft_roam_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Roam to new AP using FT:\r\n");
    (void)PRINTF("    wlan-ft-roam <bssid> <channel>\r\n");
}

static void test_wlan_ft_roam(int argc, char **argv)
{
    int ret;
    t_u8 bssid[IEEEtypes_ADDRESS_SIZE] = {0};
    t_u8 channel                       = 0;
    if (argc != 3)
    {
        dump_wlan_ft_roam_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    if (get_mac(argv[1], (char *)bssid, ':') != false)
    {
        (void)PRINTF(
            "Error: invalid BSSID argument"
            "\r\n");
        dump_wlan_ft_roam_usage();
        return;
    }

    errno   = 0;
    channel = (t_u8)strtol(argv[2], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:channel errno:%d\r\n", errno);
        dump_wlan_ft_roam_usage();
        return;
    }

    ret = wlan_ft_roam(bssid, channel);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to start FT roaming\r\n");
    }
}
#endif

#if CONFIG_HEAP_STAT
static void test_heap_stat(int argc, char **argv)
{
    OSA_DumpMemStats();
}
#endif

#if CONFIG_WIFI_EU_CRYPTO
static void dump_wlan_eu_crypto_rc4(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Algorithm RC4 encryption and decryption verification\r\n");
    (void)PRINTF("wlan-eu-crypto-rc4 <EncDec>\r\n");
    (void)PRINTF("EncDec: 0-Decrypt, 1-Encrypt\r\n");
}
static void test_wlan_eu_crypto_rc4(int argc, char **argv)
{
    unsigned int EncDec = 0U;
    t_u8 DATA[80]       = {0};
    t_u16 Length;
    int ret;
    t_u16 Dec_DataLength;
    t_u16 Enc_DataLength;
    t_u16 KeyLength;
    t_u16 KeyIVLength;
    if (argc != 2)
    {
        dump_wlan_eu_crypto_rc4();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    (void)get_uint(argv[1], &EncDec, 1);
    if (EncDec != 0U && EncDec != 1U)
    {
        dump_wlan_eu_crypto_rc4();
        (void)PRINTF("Error: invalid EncDec\r\n");
        return;
    }
    /*Algorithm: RC4*/
    t_u8 Key[16]     = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
    KeyLength        = 16;
    t_u8 EncData[16] = {0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12};
    Enc_DataLength   = 16;
    t_u8 DecData[16] = {0xd9, 0x90, 0x42, 0xad, 0x51, 0xab, 0x11, 0x3f, 0x24, 0x46, 0x69, 0xe6, 0xf1, 0xac, 0x49, 0xf5};
    Dec_DataLength   = 16;
    t_u8 KeyIV[8]    = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    KeyIVLength      = 8;

    if (EncDec == 0U)
    {
        (void)memcpy(DATA, DecData, Dec_DataLength);
        Length = Dec_DataLength;
        ret    = wlan_set_crypto_RC4_decrypt(Key, KeyLength, KeyIV, KeyIVLength, DATA, &Length);
    }
    else
    {
        (void)memcpy(DATA, EncData, Enc_DataLength);
        Length = Enc_DataLength;
        ret    = wlan_set_crypto_RC4_encrypt(Key, KeyLength, KeyIV, KeyIVLength, DATA, &Length);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Raw Data:\r\n");
        if (EncDec == 0U)
        {
            dump_hex((t_u8 *)DecData, Dec_DataLength);
            (void)PRINTF("Decrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
        else
        {
            dump_hex((t_u8 *)EncData, Enc_DataLength);
            (void)PRINTF("Encrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}

static void dump_wlan_eu_crypto_aes_ecb(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Algorithm AES-ECB encryption and decryption verification\r\n");
    (void)PRINTF("wlan-eu-crypto-aes-ecb <EncDec>\r\n");
    (void)PRINTF("EncDec: 0-Decrypt, 1-Encrypt\r\n");
}
static void test_wlan_eu_crypto_aes_ecb(int argc, char **argv)
{
    unsigned int EncDec = 0U;
    t_u8 DATA[80]       = {0};
    t_u16 Length;
    int ret;
    t_u16 Dec_DataLength;
    t_u16 Enc_DataLength;
    t_u16 KeyLength;
    t_u16 KeyIVLength;
    if (argc != 2)
    {
        dump_wlan_eu_crypto_aes_ecb();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    (void)get_uint(argv[1], &EncDec, 1);
    if (EncDec != 0U && EncDec != 1U)
    {
        dump_wlan_eu_crypto_aes_ecb();
        (void)PRINTF("Error: invalid EncDec\r\n");
        return;
    }
    /*Algorithm: AES_ECB*/
    t_u8 Key[16]     = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
    KeyLength        = 16;
    t_u8 EncData[16] = {0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12};
    Enc_DataLength   = 16;
    t_u8 DecData[16] = {0xc6, 0x93, 0x9d, 0xaa, 0xd1, 0xd0, 0x68, 0x28, 0xfe, 0x88, 0x52, 0x75, 0xa9, 0x43, 0xf9, 0xc0};
    Dec_DataLength   = 16;
    t_u8 KeyIV[8]    = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    KeyIVLength      = 8;

    if (EncDec == 0U)
    {
        (void)memcpy(DATA, DecData, Dec_DataLength);
        Length = Dec_DataLength;
        ret    = wlan_set_crypto_AES_ECB_decrypt(Key, KeyLength, KeyIV, KeyIVLength, DATA, &Length);
    }
    else
    {
        (void)memcpy(DATA, EncData, Enc_DataLength);
        Length = Enc_DataLength;
        ret    = wlan_set_crypto_AES_ECB_encrypt(Key, KeyLength, KeyIV, KeyIVLength, DATA, &Length);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Raw Data:\r\n");
        if (EncDec == 0U)
        {
            dump_hex((t_u8 *)DecData, Dec_DataLength);
            (void)PRINTF("Decrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
        else
        {
            dump_hex((t_u8 *)EncData, Enc_DataLength);
            (void)PRINTF("Encrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}

static void dump_wlan_eu_crypto_aes_wrap(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Algorithm AES-WRAP encryption and decryption verification\r\n");
    (void)PRINTF("wlan-eu-crypto-aes-wrap <EncDec>\r\n");
    (void)PRINTF("EncDec: 0-Decrypt, 1-Encrypt\r\n");
}
static void test_wlan_eu_crypto_aes_wrap(int argc, char **argv)
{
    unsigned int EncDec = 0U;
    t_u8 DATA[80]       = {0};
    t_u16 Length;
    int ret;
    t_u16 Dec_DataLength;
    t_u16 Enc_DataLength;
    t_u16 KeyLength;
    t_u16 KeyIVLength;
    if (argc != 2)
    {
        dump_wlan_eu_crypto_aes_wrap();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    (void)get_uint(argv[1], &EncDec, 1);
    if (EncDec != 0U && EncDec != 1U)
    {
        dump_wlan_eu_crypto_aes_wrap();
        (void)PRINTF("Error: invalid EncDec\r\n");
        return;
    }
    /*Algorithm: AES_WRAP*/
    t_u8 Key[16]     = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
    KeyLength        = 16;
    t_u8 EncData[16] = {0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12};
    Enc_DataLength   = 16;
    t_u8 DecData[24] = {0xfa, 0xda, 0x96, 0x53, 0x30, 0x97, 0x4b, 0x61, 0x77, 0xc6, 0xd4, 0x3c,
                        0xd2, 0x0e, 0x1f, 0x6d, 0x43, 0x8a, 0x0a, 0x1c, 0x4f, 0x6a, 0x1a, 0xd7
                       };
    Dec_DataLength   = 24;
    t_u8 KeyIV[8]    = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    KeyIVLength      = 8;

    if (EncDec == 0U)
    {
        (void)memcpy(DATA, DecData, Dec_DataLength);
        Length = Dec_DataLength;
        ret    = wlan_set_crypto_AES_WRAP_decrypt(Key, KeyLength, KeyIV, KeyIVLength, DATA, &Length);
    }
    else
    {
        (void)memcpy(DATA, EncData, Enc_DataLength);
        Length = Enc_DataLength;
        ret    = wlan_set_crypto_AES_WRAP_encrypt(Key, KeyLength, KeyIV, KeyIVLength, DATA, &Length);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Raw Data:\r\n");
        if (EncDec == 0U)
        {
            dump_hex((t_u8 *)DecData, Dec_DataLength);
            (void)PRINTF("Decrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
        else
        {
            dump_hex((t_u8 *)EncData, Enc_DataLength);
            (void)PRINTF("Encrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}

static void dump_wlan_eu_crypto_ccmp_128(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Algorithm AES-CCMP-128 encryption and decryption verification\r\n");
    (void)PRINTF("wlan-eu-crypto-ccmp-128 <EncDec>\r\n");
    (void)PRINTF("EncDec: 0-Decrypt, 1-Encrypt\r\n");
}
static void test_wlan_eu_crypto_ccmp_128(int argc, char **argv)
{
    unsigned int EncDec = 0U;
    t_u8 DATA[80]       = {0};
    t_u16 Length;
    int ret;
    t_u16 Dec_DataLength;
    t_u16 Enc_DataLength;
    t_u16 KeyLength;
    t_u16 NonceLength;
    t_u16 AADLength;
    if (argc != 2)
    {
        dump_wlan_eu_crypto_ccmp_128();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    (void)get_uint(argv[1], &EncDec, 1);
    if (EncDec != 0U && EncDec != 1U)
    {
        dump_wlan_eu_crypto_ccmp_128();
        (void)PRINTF("Error: invalid EncDec\r\n");
        return;
    }
    /*Algorithm: AES_CCMP_128*/
    t_u8 Key[16]     = {0xc9, 0x7c, 0x1f, 0x67, 0xce, 0x37, 0x11, 0x85, 0x51, 0x4a, 0x8a, 0x19, 0xf2, 0xbd, 0xd5, 0x2f};
    KeyLength        = 16;
    t_u8 EncData[20] = {0xf8, 0xba, 0x1a, 0x55, 0xd0, 0x2f, 0x85, 0xae, 0x96, 0x7b,
                        0xb6, 0x2f, 0xb6, 0xcd, 0xa8, 0xeb, 0x7e, 0x78, 0xa0, 0x50
                       };
    Enc_DataLength   = 20;
    t_u8 DecData[28] = {0xf3, 0xd0, 0xa2, 0xfe, 0x9a, 0x3d, 0xbf, 0x23, 0x42, 0xa6, 0x43, 0xe4, 0x32, 0x46,
                        0xe8, 0x0c, 0x3c, 0x04, 0xd0, 0x19, 0x78, 0x45, 0xce, 0x0b, 0x16, 0xf9, 0x76, 0x23
                       };
    Dec_DataLength   = 28;
    t_u8 Nonce[13]   = {0x00, 0x50, 0x30, 0xf1, 0x84, 0x44, 0x08, 0xb5, 0x03, 0x97, 0x76, 0xe7, 0x0c};
    NonceLength      = 13;
    t_u8 AAD[22]     = {0x08, 0x40, 0x0f, 0xd2, 0xe1, 0x28, 0xa5, 0x7c, 0x50, 0x30, 0xf1,
                        0x84, 0x44, 0x08, 0xab, 0xae, 0xa5, 0xb8, 0xfc, 0xba, 0x00, 0x00
                       };
    AADLength        = 22;

    if (EncDec == 0U)
    {
        (void)memcpy(DATA, DecData, Dec_DataLength);
        Length = Dec_DataLength;
        ret    = wlan_set_crypto_AES_CCMP_decrypt(Key, KeyLength, AAD, AADLength, Nonce, NonceLength, DATA, &Length);
    }
    else
    {
        (void)memcpy(DATA, EncData, Enc_DataLength);
        Length = Enc_DataLength;
        ret    = wlan_set_crypto_AES_CCMP_encrypt(Key, KeyLength, AAD, AADLength, Nonce, NonceLength, DATA, &Length);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Raw Data:\r\n");
        if (EncDec == 0U)
        {
            dump_hex((t_u8 *)DecData, Dec_DataLength);
            (void)PRINTF("Decrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
        else
        {
            dump_hex((t_u8 *)EncData, Enc_DataLength);
            (void)PRINTF("Encrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}
#if !defined(SD8978)
static void dump_wlan_eu_crypto_ccmp_256(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Algorithm AES-CCMP-256 encryption and decryption verification\r\n");
    (void)PRINTF("wlan-eu-crypto-ccmp-256 <EncDec>\r\n");
    (void)PRINTF("EncDec: 0-Decrypt, 1-Encrypt\r\n");
}
static void test_wlan_eu_crypto_ccmp_256(int argc, char **argv)
{
    unsigned int EncDec = 0U;
    t_u8 DATA[80]       = {0};
    t_u16 Length;
    int ret;
    t_u16 Dec_DataLength;
    t_u16 Enc_DataLength;
    t_u16 KeyLength;
    t_u16 NonceLength;
    t_u16 AADLength;
    if (argc != 2)
    {
        dump_wlan_eu_crypto_ccmp_256();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    (void)get_uint(argv[1], &EncDec, 1);
    if (EncDec != 0U && EncDec != 1U)
    {
        dump_wlan_eu_crypto_ccmp_256();
        (void)PRINTF("Error: invalid EncDec\r\n");
        return;
    }
    /*Algorithm: AES_WRAP*/
    t_u8 Key[32]     = {0xc9, 0x7c, 0x1f, 0x67, 0xce, 0x37, 0x11, 0x85, 0x51, 0x4a, 0x8a, 0x19, 0xf2, 0xbd, 0xd5, 0x2f,
                        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
                       };
    KeyLength        = 32;
    t_u8 EncData[20] = {0xf8, 0xba, 0x1a, 0x55, 0xd0, 0x2f, 0x85, 0xae, 0x96, 0x7b,
                        0xb6, 0x2f, 0xb6, 0xcd, 0xa8, 0xeb, 0x7e, 0x78, 0xa0, 0x50
                       };
    Enc_DataLength   = 20;
    t_u8 DecData[36] = {0x6d, 0x15, 0x5d, 0x88, 0x32, 0x66, 0x82, 0x56, 0xd6, 0xa9, 0x2b, 0x78,
                        0xe1, 0x1d, 0x8e, 0x54, 0x49, 0x5d, 0xd1, 0x74, 0x80, 0xaa, 0x56, 0xc9,
                        0x49, 0x2e, 0x88, 0x2b, 0x97, 0x64, 0x2f, 0x80, 0xd5, 0x0f, 0xe9, 0x7b
                       };
    Dec_DataLength   = 36;
    t_u8 Nonce[13]   = {0x00, 0x50, 0x30, 0xf1, 0x84, 0x44, 0x08, 0xb5, 0x03, 0x97, 0x76, 0xe7, 0x0c};
    NonceLength      = 13;
    t_u8 AAD[22]     = {0x08, 0x40, 0x0f, 0xd2, 0xe1, 0x28, 0xa5, 0x7c, 0x50, 0x30, 0xf1,
                        0x84, 0x44, 0x08, 0xab, 0xae, 0xa5, 0xb8, 0xfc, 0xba, 0x00, 0x00
                       };
    AADLength        = 22;

    if (EncDec == 0U)
    {
        (void)memcpy(DATA, DecData, Dec_DataLength);
        Length = Dec_DataLength;
        ret    = wlan_set_crypto_AES_CCMP_decrypt(Key, KeyLength, AAD, AADLength, Nonce, NonceLength, DATA, &Length);
    }
    else
    {
        (void)memcpy(DATA, EncData, Enc_DataLength);
        Length = Enc_DataLength;
        ret    = wlan_set_crypto_AES_CCMP_encrypt(Key, KeyLength, AAD, AADLength, Nonce, NonceLength, DATA, &Length);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Raw Data:\r\n");
        if (EncDec == 0U)
        {
            dump_hex((t_u8 *)DecData, Dec_DataLength);
            (void)PRINTF("Decrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
        else
        {
            dump_hex((t_u8 *)EncData, Enc_DataLength);
            (void)PRINTF("Encrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}
static void dump_wlan_eu_crypto_gcmp_128(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Algorithm AES-GCMP-128 encryption and decryption verification\r\n");
    (void)PRINTF("wlan-eu-crypto-gcmp-128 <EncDec>\r\n");
    (void)PRINTF("EncDec: 0-Decrypt, 1-Encrypt\r\n");
}
static void test_wlan_eu_crypto_gcmp_128(int argc, char **argv)
{
    unsigned int EncDec = 0U;
    t_u8 DATA[80]       = {0};
    t_u16 Length;
    int ret;
    t_u16 Dec_DataLength;
    t_u16 Dec_DataOnlyLength;
    t_u16 Dec_TagLength;
    t_u16 Enc_DataLength;
    t_u16 KeyLength;
    t_u16 NonceLength;
    t_u16 AADLength;
    if (argc != 2)
    {
        dump_wlan_eu_crypto_gcmp_128();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    (void)get_uint(argv[1], &EncDec, 1);
    if (EncDec != 0U && EncDec != 1U)
    {
        dump_wlan_eu_crypto_gcmp_128();
        (void)PRINTF("Error: invalid EncDec\r\n");
        return;
    }
    /*Algorithm: AES_WRAP*/
    t_u8 Key[16]     = {0xc9, 0x7c, 0x1f, 0x67, 0xce, 0x37, 0x11, 0x85, 0x51, 0x4a, 0x8a, 0x19, 0xf2, 0xbd, 0xd5, 0x2f};
    KeyLength        = 16;
    t_u8 EncData[40] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
                        0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b,
                        0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
                       };
    Enc_DataLength   = 40;
    t_u8 DecDataOnly[40] = {0x60, 0xe9, 0x70, 0x0c, 0xc4, 0xd4, 0x0a, 0xc6, 0xd2, 0x88, 0xb2, 0x01, 0xc3, 0x8f,
                            0x5b, 0xf0, 0x8b, 0x80, 0x74, 0x42, 0x64, 0x0a, 0x15, 0x96, 0xe5, 0xdb, 0xda, 0xd4,
                            0x1d, 0x1f, 0x36, 0x23, 0xf4, 0x5d, 0x7a, 0x12, 0xdb, 0x7a, 0xfb, 0x23
                           };
    Dec_DataOnlyLength   = 40;
#if defined(SD9177) || defined(RW610)
    t_u8 DecTag[16] = {0xde, 0xf6, 0x19, 0xc2, 0xa3, 0x74, 0xb6, 0xdf, 0x66, 0xff, 0xa5, 0x3b, 0x6c, 0x69, 0xd7, 0x9e};
#else
    t_u8 DecTag[16] = {0xe9, 0x04, 0x97, 0xa1, 0xec, 0x9c, 0x5e, 0x8b, 0x85, 0x5b, 0x9d, 0xc3, 0xa8, 0x16, 0x91, 0xa3};
#endif
    Dec_TagLength = 16;

    t_u8 DecData[56] = {0}; /*Dec-data-only + Tag*/
    memcpy(DecData, DecDataOnly, Dec_DataOnlyLength);
    memcpy(DecData + Dec_DataOnlyLength, DecTag, Dec_TagLength);
    Dec_DataLength = Dec_DataOnlyLength + Dec_TagLength;

    t_u8 Nonce[12] = {0x50, 0x30, 0xf1, 0x84, 0x44, 0x08, 0x00, 0x89, 0x5f, 0x5f, 0x2b, 0x08};
    NonceLength    = 12;
    t_u8 AAD[24]   = {0x88, 0x48, 0x0f, 0xd2, 0xe1, 0x28, 0xa5, 0x7c, 0x50, 0x30, 0xf1, 0x84,
                      0x44, 0x08, 0x50, 0x30, 0xf1, 0x84, 0x44, 0x08, 0x80, 0x33, 0x03, 0x00
                     };
    AADLength      = 24;

    if (EncDec == 0U)
    {
        (void)memcpy(DATA, DecData, Dec_DataLength);
        Length = Dec_DataLength;
        ret    = wlan_set_crypto_AES_GCMP_decrypt(Key, KeyLength, AAD, AADLength, Nonce, NonceLength, DATA, &Length);
    }
    else
    {
        (void)memcpy(DATA, EncData, Enc_DataLength);
        Length = Enc_DataLength;
        ret    = wlan_set_crypto_AES_GCMP_encrypt(Key, KeyLength, AAD, AADLength, Nonce, NonceLength, DATA, &Length);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Raw Data:\r\n");
        if (EncDec == 0U)
        {
            dump_hex((t_u8 *)DecData, Dec_DataLength);
            (void)PRINTF("Decrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
        else
        {
            dump_hex((t_u8 *)EncData, Enc_DataLength);
            (void)PRINTF("Encrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}
static void dump_wlan_eu_crypto_gcmp_256(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Algorithm AES-GCMP-256 encryption and decryption verification\r\n");
    (void)PRINTF("wlan-eu-crypto-gcmp-256 <EncDec>\r\n");
    (void)PRINTF("EncDec: 0-Decrypt, 1-Encrypt\r\n");
}
static void test_wlan_eu_crypto_gcmp_256(int argc, char **argv)
{
    unsigned int EncDec = 0U;
    t_u8 DATA[80]       = {0};
    t_u16 Length;
    int ret;
    t_u16 Dec_DataLength;
    t_u16 Dec_DataOnlyLength;
    t_u16 Dec_TagLength;
    t_u16 Enc_DataLength;
    t_u16 KeyLength;
    t_u16 NonceLength;
    t_u16 AADLength;
    if (argc != 2)
    {
        dump_wlan_eu_crypto_gcmp_256();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }
    (void)get_uint(argv[1], &EncDec, 1);
    if (EncDec != 0U && EncDec != 1U)
    {
        dump_wlan_eu_crypto_gcmp_256();
        (void)PRINTF("Error: invalid EncDec\r\n");
        return;
    }
    /*Algorithm: AES_WRAP*/
    t_u8 Key[32]     = {0xc9, 0x7c, 0x1f, 0x67, 0xce, 0x37, 0x11, 0x85, 0x51, 0x4a, 0x8a, 0x19, 0xf2, 0xbd, 0xd5, 0x2f,
                        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
                       };
    KeyLength        = 32;
    t_u8 EncData[40] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
                        0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b,
                        0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
                       };
    Enc_DataLength   = 40;

    t_u8 DecDataOnly[40] = {0x65, 0x83, 0x43, 0xc8, 0xb1, 0x44, 0x47, 0xd9, 0x21, 0x1d, 0xef, 0xd4, 0x6a, 0xd8,
                            0x9c, 0x71, 0x0c, 0x6f, 0xc3, 0x33, 0x33, 0x23, 0x6e, 0x39, 0x97, 0xb9, 0x17, 0x6a,
                            0x5a, 0x8b, 0xe7, 0x79, 0xb2, 0x12, 0x66, 0x55, 0x5e, 0x70, 0xad, 0x79
                           };
    Dec_DataOnlyLength   = 40;

#if defined(SD9177) || defined(RW610)
    t_u8 DecTag[16] = {0x11, 0x43, 0x16, 0x85, 0x90, 0x95, 0x47, 0x3d, 0x5b, 0x1b, 0xd5, 0x96, 0xb3, 0xde, 0xa3, 0xbf};
#else
    t_u8 DecTag[16] = {0x11, 0x53, 0x9a, 0x0e, 0x22, 0xc1, 0x26, 0x0c, 0xbb, 0xe8, 0x35, 0x93, 0x35, 0xf3, 0x37, 0x65};
#endif
    Dec_TagLength = 16;

    t_u8 DecData[56] = {0}; /*Dec-data-only + Tag*/
    memcpy(DecData, DecDataOnly, Dec_DataOnlyLength);
    memcpy(DecData + Dec_DataOnlyLength, DecTag, Dec_TagLength);
    Dec_DataLength = Dec_DataOnlyLength + Dec_TagLength;

    t_u8 Nonce[12] = {0x50, 0x30, 0xf1, 0x84, 0x44, 0x08, 0x00, 0x89, 0x5f, 0x5f, 0x2b, 0x08};
    NonceLength    = 12;
    t_u8 AAD[24]   = {0x88, 0x48, 0x0f, 0xd2, 0xe1, 0x28, 0xa5, 0x7c, 0x50, 0x30, 0xf1, 0x84,
                      0x44, 0x08, 0x50, 0x30, 0xf1, 0x84, 0x44, 0x08, 0x80, 0x33, 0x03, 0x00
                     };
    AADLength      = 24;

    if (EncDec == 0U)
    {
        (void)memcpy(DATA, DecData, Dec_DataLength);
        Length = Dec_DataLength;
        ret    = wlan_set_crypto_AES_GCMP_decrypt(Key, KeyLength, AAD, AADLength, Nonce, NonceLength, DATA, &Length);
    }
    else
    {
        (void)memcpy(DATA, EncData, Enc_DataLength);
        Length = Enc_DataLength;
        ret    = wlan_set_crypto_AES_GCMP_encrypt(Key, KeyLength, AAD, AADLength, Nonce, NonceLength, DATA, &Length);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Raw Data:\r\n");
        if (EncDec == 0U)
        {
            dump_hex((t_u8 *)DecData, Dec_DataLength);
            (void)PRINTF("Decrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
        else
        {
            dump_hex((t_u8 *)EncData, Enc_DataLength);
            (void)PRINTF("Encrypted Data:\r\n");
            dump_hex((t_u8 *)DATA, Length);
        }
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}
#endif
#endif
#if CONFIG_HEAP_DEBUG
int os_mem_alloc_cnt = 0;
int os_mem_free_cnt  = 0;

static void test_wlan_os_mem_stat(int argc, char **argv)
{
    (void)PRINTF("os_mem_alloc_cnt: %d \r\n", os_mem_alloc_cnt);
    (void)PRINTF("os_mem_free_cnt : %d \r\n", os_mem_free_cnt);
    (void)PRINTF("FreeHeapSize    : %d \r\n\r\n", xPortGetFreeHeapSize());
    wlan_show_os_mem_stat();
}
#endif

#if CONFIG_RX_ABORT_CFG
static void dump_wlan_rx_abort_cfg_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Get rx abort config:\r\n");
    (void)PRINTF("    wlan-rx-abort-cfg\r\n");
    (void)PRINTF("Set rx abort config:\r\n");
    (void)PRINTF("    wlan-rx-abort-cfg <enable> <rssi_threshold>\r\n");
    (void)PRINTF("Options: \r\n");
    (void)PRINTF("    <enable>        : 1--Enable 0--Disable\r\n");
    (void)PRINTF("    <rssi_threshold>: weak RSSI pkt threshold in dBm (absolute value)\r\n");
    (void)PRINTF("                      (default = 70)\r\n");
    (void)PRINTF("For example:\r\n");
    (void)PRINTF("    wlan-rx-abort-cfg      : Get current rx abort config\r\n");
    (void)PRINTF("    wlan-rx-abort-cfg 1 40 : Enable rx abort and set weak RSSI Threshold to -40 dBm\r\n");
    (void)PRINTF("    wlan-rx-abort-cfg 0    : Disable rx abort\r\n");
}

static void test_wlan_rx_abort_cfg(int argc, char **argv)
{
    struct wlan_rx_abort_cfg cfg;

    if (argc > 3)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_rx_abort_cfg_usage();
        return;
    }
    (void)memset(&cfg, 0, sizeof(cfg));
    /* GET */
    if (argc == 1)
    {
        dump_wlan_rx_abort_cfg_usage();
        wlan_set_get_rx_abort_cfg(&cfg, ACTION_GET);
        (void)PRINTF("Static Rx Abort %s\r\n", cfg.enable == 1 ? "enabled" : "disabled");
        if (cfg.enable == 1)
        {
            (void)PRINTF("RSSI threshold : %ddBm\r\n", cfg.rssi_threshold);
        }
    }
    /* SET */
    else
    {
        cfg.enable = (t_u8)atoi(argv[1]);
        if (cfg.enable)
        {
            if (argc == 2)
            {
                cfg.rssi_threshold = 70;
                (void)PRINTF("No RSSI threshold set by user.\r\n");
                (void)PRINTF("Use default value 70 instead.\r\n");
            }
            else
            {
                cfg.rssi_threshold = (int)atoi(argv[2]);
                if (cfg.rssi_threshold > 0x7f)
                {
                    (void)PRINTF("Invalid threshold value\r\n");
                    (void)PRINTF("RSSI threshold should less than 0x7f\r\n");
                    return;
                }
            }
        }
        wlan_set_get_rx_abort_cfg(&cfg, ACTION_SET);
    }
    return;
}
#endif

#if CONFIG_RX_ABORT_CFG_EXT
static void dump_wlan_rx_abort_cfg_ext_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Get dynamic rx abort cfg:\r\n");
    (void)PRINTF("  wlan-get-rx-abort-cfg-ext\r\n");
    (void)PRINTF("Set dynamic rx abort cfg:\r\n");
    (void)PRINTF(
        "  wlan-set-rx-abort-cfg-ext enable <enable/disable> margin <margin> ceil <ceil_rssi_thresh> "
        "floor <floor_rssi_thresh>\r\n");
    (void)PRINTF("Options: \r\n");
    (void)PRINTF("    enable <enable>\r\n");
    (void)PRINTF("              0 -- Disable Rx abort\r\n");
    (void)PRINTF("              1 -- Enable Rx abort of pkt having weak RSSI\r\n");
    (void)PRINTF("    margin <margin>\r\n");
    (void)PRINTF("              rssi margin in dBm (absolute val)\r\n");
    (void)PRINTF("              (default = 10)\r\n");
    (void)PRINTF("    ceil <ceil_rssi_thresh>\r\n");
    (void)PRINTF("              rceiling weak RSSI pkt threshold in dBm (absolute val)\r\n");
    (void)PRINTF("              (default = 62)\r\n");
    (void)PRINTF("    floor <floor_rssi_thresh>\r\n");
    (void)PRINTF("              floor weak RSSI pkt threshold in dBm (absolute val)\r\n");
    (void)PRINTF("              (default = 82)\r\n");
    (void)PRINTF("For example:\r\n");
    (void)PRINTF("    wlan-get-rx-abort-cfg-ext: Display current rx abort configuration\r\n");
    (void)PRINTF("    wlan-set-rx-abort-cfg-ext enable 1 margin 5 ceil 40 floor 70 :\r\n");
    (void)PRINTF(
        "         Enable dynamic rx abort,set margin to -5 dBm, set ceil RSSI Threshold to -40 dBm and set floor RSSI "
        "threshold to -70 dbm\r\n");
    (void)PRINTF("    wlan-set-rx-abort-cfg-ext enable 1\r\n");
    (void)PRINTF("    Don't set RSSI margin, drive will set defult RSSI margin threshold value.\r\n");
    (void)PRINTF("    Don't set ceil RSSI threshold, driver will set default ceil RSSI threshold value.\r\n");
    (void)PRINTF("    Don't set floor RSSI threshold, driver will set default floor RSSI threshold value.\r\n");
    (void)PRINTF("\r\n");
    (void)PRINTF("    wlan-set-rx-abort-cfg-ext enable 1 ceil 255 \r\n");
    (void)PRINTF("    Don't set RSSI margin, drive will set defult RSSI margin threshold value.\r\n");
    (void)PRINTF(
        "    Input ceil RSSI threshold to 0xff, set ceil value to default based on EDMAC enabled or disabled "
        "status.\r\n");
    (void)PRINTF("    In this case, don't set floor RSSI threshold.\r\n");
    (void)PRINTF("    wlan-set-rx-abort-cfg-ext enable 0    : Disable dynamic rx abort\r\n");
}

static void test_wlan_get_rx_abort_cfg_ext(int argc, char **argv)
{
#if !CONFIG_MEM_POOLS
    struct wlan_rx_abort_cfg_ext *cfg =
        (struct wlan_rx_abort_cfg_ext *)OSA_MemoryAllocate(sizeof(struct wlan_rx_abort_cfg_ext));
#else
    struct wlan_rx_abort_cfg_ext *cfg =
        (struct wlan_rx_abort_cfg_ext *)OSA_MemoryPoolAllocate(buf_32_MemoryPool);
#endif
    if (cfg == NULL)
    {
        (void)PRINTF("test_wlan_get_rx_abort_cfg_ext malloc fail! \r\n");
        return;
    }
    (void)memset(cfg, 0, sizeof(*cfg));

    wlan_get_rx_abort_cfg_ext(cfg);

    (void)PRINTF("Dynamic Rx Abort %s\r\n", cfg->enable == 1 ? "enabled" : "disabled");
    if (cfg->enable == 1)
    {
        int rssi;
        rssi = cfg->rssi_margin;
        if (rssi > 0x7f)
            rssi = -(256 - rssi);
        (void)PRINTF("Margin RSSI: %s%d dbm\r\n", ((rssi > 0) ? "-" : ""), rssi);

        rssi = cfg->ceil_rssi_threshold;
        if (rssi > 0x7f)
            rssi = -(256 - rssi);
        (void)PRINTF("Ceil RSSI threshold: %s%d dbm\r\n", ((rssi > 0) ? "-" : ""), rssi);

        rssi = cfg->floor_rssi_threshold;
        if (rssi > 0x7f)
            rssi = -(256 - rssi);
        (void)PRINTF("Floor rssi threshold: %s%d dbm\r\n", ((rssi > 0) ? "-" : ""), rssi);

        rssi = cfg->current_dynamic_rssi_threshold;
        if (rssi > 0x7f)
            rssi = -(256 - rssi);
        (void)PRINTF("Dynamic RSSI Threshold : %d dbm (%s)\r\n", rssi,
                     cfg->rssi_default_config ? (cfg->edmac_enable ? "EDMAC based" : "Default") : "Config based");
    }
    (void)PRINTF("\r\n");
    OSA_MemoryFree(cfg);
    return;
}

static void test_wlan_set_rx_abort_cfg_ext(int argc, char **argv)
{
    int arg = 0;
    unsigned int value;
    struct wlan_rx_abort_cfg_ext cfg;

    struct
    {
        uint8_t enable : 1;
        uint8_t margin : 1;
        uint8_t ceil_thresh : 1;
        uint8_t floor_thresh : 1;
    } info;

    (void)memset(&info, 0, sizeof(info));

    (void)memset(&cfg, 0, sizeof(cfg));

    if (argc < 3 && argc > 9)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_rx_abort_cfg_ext_usage();
        return;
    }
    arg++;
    do
    {
        if (!info.enable && string_equal("enable", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid enable argument\r\n");
                dump_wlan_rx_abort_cfg_ext_usage();
                return;
            }
            if (value != 0 && value != 1)
            {
                (void)PRINTF("Error: invalid action argument\r\n");
                dump_wlan_rx_abort_cfg_ext_usage();
                return;
            }
            if (value == 0) /*Disable dynamic rx bort config*/
            {
                cfg.enable = 0;
                break;
            }
            else /* Enable dynamic rx abort config*/
            {
                cfg.enable = 1;
            }
            arg += 2;
            info.enable = 1;
        }
        else if (!info.margin && string_equal("margin", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid margin argument\r\n");
                dump_wlan_rx_abort_cfg_ext_usage();
                return;
            }
            if (value > 0x7f)
            {
                (void)PRINTF("Error: Invalid Margin value\r\n");
                dump_wlan_rx_abort_cfg_ext_usage();
                return;
            }
            cfg.rssi_margin = value;
            arg += 2;
            info.margin = 1;
        }
        else if (!info.ceil_thresh && string_equal("ceil", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid ceil argument\r\n");
                dump_wlan_rx_abort_cfg_ext_usage();
                return;
            }
            if (value > 0x7f && value != 0xff)
            {
                (void)PRINTF("Error: Invalid ceil value\r\n");
                dump_wlan_rx_abort_cfg_ext_usage();
                return;
            }
            cfg.ceil_rssi_threshold = value;
            arg += 2;
            info.ceil_thresh = 1;
        }
        else if (!info.floor_thresh && string_equal("floor", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid floor argument\r\n");
                dump_wlan_rx_abort_cfg_ext_usage();
                return;
            }
            if (value > 0x7f)
            {
                (void)PRINTF("Error: Invalid floor value\r\n");
                dump_wlan_rx_abort_cfg_ext_usage();
                return;
            }
            cfg.floor_rssi_threshold = value;
            arg += 2;
            info.floor_thresh = 1;
        }
        else
        {
            (void)PRINTF("Error: argument %d is invalid\r\n", arg);
            dump_wlan_rx_abort_cfg_ext_usage();
            return;
        }
    }
    while (arg < argc);

    if (cfg.enable == 0)
    {
        (void)PRINTF("Disable dynamic rx abort config\r\n");
        wlan_set_rx_abort_cfg_ext((const struct wlan_rx_abort_cfg_ext *)&cfg);
        return;
    }
    if (cfg.rssi_margin == 0)
    {
        (void)PRINTF("No Margin RSSI is set by user.\r\n");
        (void)PRINTF("  Use default value instead.\r\n");
        cfg.rssi_margin = 10;
    }

    if (cfg.ceil_rssi_threshold == 0)
    {
        (void)PRINTF("No Ceil RSSI threshold is set by user.\r\n");
        (void)PRINTF("  Use default value instead.\r\n");
        cfg.ceil_rssi_threshold = 62;
    }

    if (cfg.floor_rssi_threshold == 0)
    {
        if (cfg.ceil_rssi_threshold == 0xff)
        {
            (void)PRINTF("No Floor rssi threshold is set by user.\r\n");
            (void)PRINTF("  Driver set floor rssi threshold to 0xff.\r\n");
            cfg.floor_rssi_threshold = 0xff;
        }
        else
        {
            (void)PRINTF("No Floor rssi threshold is set by user.\r\n");
            (void)PRINTF("  Use default value instead.\r\n");
            cfg.floor_rssi_threshold = 82;
        }
    }

    wlan_set_rx_abort_cfg_ext((const struct wlan_rx_abort_cfg_ext *)&cfg);

    return;
}
#endif

#if CONFIG_CCK_DESENSE_CFG
static void dump_wlan_cck_desense_cfg_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Get current cck desense config:\r\n");
    (void)PRINTF("    wlan-cck-desense-cfg\r\n");
    (void)PRINTF("Set cck desense config:\r\n");
    (void)PRINTF("    wlan-cck-desense-cfg <mode> <margin ceil_thresh> <num_on_intervals num_off_intervals>\r\n");
    (void)PRINTF("Options: \r\n");
    (void)PRINTF("    <enable>            :0 - Disable cck desense\r\n");
    (void)PRINTF("                         1 - Enable dynamic cck desense mode\r\n");
    (void)PRINTF("                         2 - Enable dynamic enhanced cck desense mode\r\n");
    (void)PRINTF("    <margin>            :rssi margin in dBm (absolute val)\r\n");
    (void)PRINTF("                         (default = 10)\r\n");
    (void)PRINTF("    <ceil_thresh>       :ceiling weak RSSI pkt threshold in dBm (absolute val)\r\n");
    (void)PRINTF("                         (default = 70)\r\n");
    (void)PRINTF("    <num_on_intervals>  :number of rateadapt intervals to keep cck desense \"on\"\r\n");
    (void)PRINTF("                         [for mode 2 only] (default = 20)\r\n");
    (void)PRINTF("    <num_off_intervals> :number of rateadapt intervals to keep cck desense \"off\"\r\n");
    (void)PRINTF("                         [for mode 2 only] (default = 3)\r\n");
    (void)PRINTF("For example:\r\n");
    (void)PRINTF("    wlan-cck-desense-cfg              : Get current cck desense config\r\n");
    (void)PRINTF("    wlan-cck-desense-cfg 0            : Disable cck desense\r\n");
    (void)PRINTF(
        "    wlan-cck-desense-cfg 1 10 70      : Set dynamic mode, margin to -10 dBm and ceil RSSI Threshold to -70 "
        "dBm\r\n");
    (void)PRINTF(
        "    wlan-cck-desense-cfg 2 10 60 30 5 : Set dynamic enhanced mode, margin to -10 dBm, ceil RSSI Threshold to "
        "-60 dBm,\r\n");
    (void)PRINTF("                                        num on intervals to 30 and num off intervals to 5\r\n");
    (void)PRINTF(
        "    wlan-cck-desense-cfg 2 5 60       : Set dynamic enhanced mode, set margin to -5 dBm, set ceil RSSI "
        "Threshold to -60 dBm,\r\n");
    (void)PRINTF("                                        and retain previous num on/off intervals setting.\r\n");
}

static void test_wlan_cck_desense_cfg(int argc, char **argv)
{
    struct wlan_cck_desense_cfg cfg;
    int num_on_intervals  = 0;
    int num_off_intervals = 0;

    if (argc > 6)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_cck_desense_cfg_usage();
        return;
    }
    memset(&cfg, 0x0, sizeof(cfg));
    /* GET */
    if (argc == 1)
    {
        dump_wlan_cck_desense_cfg_usage();
        wlan_set_get_cck_desense_cfg(&cfg, ACTION_GET);
        (void)PRINTF("CCK Desense %s\r\n", (cfg.mode) ? "enabled" : "disabled");
        if (cfg.mode != CCK_DESENSE_MODE_DISABLED)
        {
            (void)PRINTF("Mode: %s\r\n", (cfg.mode == CCK_DESENSE_MODE_DYNAMIC) ? "Dynamic" : "Dynamic Enhanced");
            (void)PRINTF("Margin : %ddBm\r\n", cfg.margin);
            (void)PRINTF("Ceil RSSI Threshold : %ddBm\r\n", cfg.ceil_thresh);
        }
        if (cfg.mode == CCK_DESENSE_MODE_DYN_ENH)
        {
            (void)PRINTF("Num ON intervals  : %d\r\n", cfg.num_on_intervals);
            (void)PRINTF("Num OFF intervals : %d\r\n", cfg.num_off_intervals);
        }
    }
    /* SET */
    else
    {
        cfg.mode = (t_u16)atoi(argv[1]);
        if (cfg.mode > CCK_DESENSE_MODE_DYN_ENH)
        {
            (void)PRINTF("Invalid cck desense mode\r\n");
            dump_wlan_cck_desense_cfg_usage();
            return;
        }
        if ((cfg.mode == CCK_DESENSE_MODE_DISABLED && argc > 2) ||
                (cfg.mode == CCK_DESENSE_MODE_DYNAMIC && argc != 4) ||
                (cfg.mode == CCK_DESENSE_MODE_DYN_ENH && (argc < 4 || argc == 5)))
        {
            (void)PRINTF("Invalid number of args for requested mode\r\n");
            dump_wlan_cck_desense_cfg_usage();
            return;
        }
        if (argc > 2)
        {
            cfg.margin      = (int)atoi(argv[2]);
            cfg.ceil_thresh = (int)atoi(argv[3]);
            if (cfg.margin > 0x7f)
            {
                (void)PRINTF("Invalid margin value\r\n");
                (void)PRINTF("The margin should less than 0x7f\r\n");
                return;
            }
            if (cfg.ceil_thresh > 0x7f)
            {
                (void)PRINTF("Invalid ceil threshold value\r\n");
                (void)PRINTF("The ceil threshold should less than 0x7f\r\n");
                return;
            }
        }
        if (argc > 4)
        {
            num_on_intervals  = atoi(argv[4]);
            num_off_intervals = atoi(argv[5]);
            if (num_on_intervals > 0xff || num_off_intervals > 0xff)
            {
                (void)PRINTF("Invalid ON/OFF intervals value\r\n");
                (void)PRINTF("The ON/OFF interval should less than 0xff\r\n");
                return;
            }
            cfg.num_on_intervals  = (t_u8)num_on_intervals;
            cfg.num_off_intervals = (t_u8)num_off_intervals;
        }
        wlan_set_get_cck_desense_cfg(&cfg, ACTION_SET);
    }
    return;
}
#endif

#if CONFIG_MULTI_CHAN
static void test_wlan_set_multi_chan_status(int argc, char **argv)
{
    int ret;
    int enable;

    if (argc != 2)
    {
        (void)PRINTF("Invalid arguments\r\n");
        return;
    }

    errno  = 0;
    enable = strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:enable multi chan status errno:%d\r\n", errno);
        return;
    }

    ret = wlan_set_multi_chan_status(enable);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Set multi_chan_status fail, please set before uap start/sta connect\r\n");
    }
}

static void test_wlan_get_multi_chan_status(int argc, char **argv)
{
    int ret;
    int enable;

    ret = wlan_get_multi_chan_status(&enable);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Get multi_chan_policy fail\r\n");
        return;
    }

    (void)PRINTF("Get multi_chan_policy %d\r\n", enable);
}

static void dump_drcs_cfg(void)
{
    (void)PRINTF("wlan-set-drcs usage:\r\n");
    (void)PRINTF("arguments group <channel_time> <switch_time> <undoze_time> <mode>\r\n");
    (void)PRINTF("input one group, same settings for both channel 0 and channel 1\r\n");
    (void)PRINTF("input two groups, different settings for channel 0 first and then channel 1\r\n");
    (void)PRINTF("channel_time: Channel time stayed (in TU 1024us) for chan_idx\r\n");
    (void)PRINTF(
        "switch_time: Channel switch time (in TU 1024us) for chan_idx, including doze for old channel and undoze for "
        "new channel\r\n");
    (void)PRINTF("undoze_time: Undoze time during switch time (in TU 1024us) for chan_idx\r\n");
    (void)PRINTF("mode: Channel switch scheme 0-PM1, 1-Null2Self\r\n");
    (void)PRINTF("Example for same settings for channel 0 and 1:\r\n");
    (void)PRINTF("wlan-set-drcs 15 10 5 0:\r\n");
    (void)PRINTF("Example for different settings for channel 0 and 1:\r\n");
    (void)PRINTF("wlan-set-drcs 15 10 5 0 16 8 4 1:\r\n");
}

static void get_drcs_cfg(char **data, wlan_drcs_cfg_t *drcs_cfg)
{
    errno              = 0;
    drcs_cfg->chantime = (t_u8)strtol(data[0], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:drcs_cfg chantime errno:%d\r\n", errno);
        return;
    }

    errno                = 0;
    drcs_cfg->switchtime = (t_u8)strtol(data[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:drcs_cfg switchtime errno:%d\r\n", errno);
        return;
    }

    errno                = 0;
    drcs_cfg->undozetime = (t_u8)strtol(data[2], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:drcs_cfg undozetime errno:%d\r\n", errno);
        return;
    }

    errno          = 0;
    drcs_cfg->mode = (t_u8)strtol(data[3], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:drcs_cfg mode errno:%d\r\n", errno);
        return;
    }
}

static void test_wlan_set_drcs_cfg(int argc, char **argv)
{
    wlan_drcs_cfg_t drcs_cfg[2] = {0};

    if (argc != 5 && argc != 9)
    {
        dump_drcs_cfg();
        return;
    }

    if (argc == 5)
    {
        get_drcs_cfg(&argv[1], &drcs_cfg[0]);
        drcs_cfg[0].chan_idx = 0x03;
    }
    else
    {
        get_drcs_cfg(&argv[1], &drcs_cfg[0]);
        get_drcs_cfg(&argv[5], &drcs_cfg[1]);
        drcs_cfg[0].chan_idx = 0x01;
        drcs_cfg[1].chan_idx = 0x02;
    }

    (void)wlan_set_drcs_cfg(&drcs_cfg[0], 2);
}

static void test_wlan_get_drcs_cfg(int argc, char **argv)
{
    int ret;
    wlan_drcs_cfg_t drcs_cfg[2] = {0};

    ret = wlan_get_drcs_cfg(&drcs_cfg[0], 2);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("get drcs cfg fail\r\n");
        return;
    }

    (void)PRINTF("chan_idx: 0x%02x\r\n", drcs_cfg[0].chan_idx);
    (void)PRINTF("chan_time: %d\r\n", drcs_cfg[0].chantime);
    (void)PRINTF("switch_time: %d\r\n", drcs_cfg[0].switchtime);
    (void)PRINTF("undoze_time: %d\r\n", drcs_cfg[0].undozetime);
    (void)PRINTF("mode: %d\r\n", drcs_cfg[0].mode);
    if (drcs_cfg[0].chan_idx != (t_u16)0x03U)
    {
        (void)PRINTF("chan_idx: 0x%02x\r\n", drcs_cfg[1].chan_idx);
        (void)PRINTF("chan_time: %d\r\n", drcs_cfg[1].chantime);
        (void)PRINTF("switch_time: %d\r\n", drcs_cfg[1].switchtime);
        (void)PRINTF("undoze_time: %d\r\n", drcs_cfg[1].undozetime);
        (void)PRINTF("mode: %d\r\n", drcs_cfg[1].mode);
    }
}
#endif

static void dump_wlan_set_antcfg_usage(void)
{
    (void)PRINTF("Usage:\r\n");
#ifndef RW610
    (void)PRINTF("wlan-set-antcfg <ant mode> [evaluate_time] \r\n");
#else
    (void)PRINTF("wlan-set-antcfg <ant_mode> <evaluate_time> <evaluate_mode>\r\n");
#endif
    (void)PRINTF("\r\n");
    (void)PRINTF("\t<ant_mode>: \r\n");
    (void)PRINTF("\t           1   -- Tx/Rx antenna 1\r\n");
    (void)PRINTF("\t           2   -- Tx/Rx antenna 2\r\n");
    (void)PRINTF("\t           0xFFFF  -- Tx/Rx antenna diversity\r\n");
    (void)PRINTF("\t[evaluate_time]: \r\n");
    (void)PRINTF("\t           If ant mode = 0xFFFF, use this to configure\r\n");
    (void)PRINTF("\t           SAD evaluate time interval in milli seconds unit.\r\n");
    (void)PRINTF("\t           MAX evaluate time is 65535ms.\r\n");
    (void)PRINTF("\t           If not specified, default value is 6000 milli seconds.\r\n");
#ifdef RW610
    (void)PRINTF("\t<evaluate_mode>: \r\n");
    (void)PRINTF("\t           0: Ant1 + Ant2\r\n");
    (void)PRINTF("\t           1: Ant2 + Ant3\r\n");
    (void)PRINTF("\t           2: Ant1 + Ant3\r\n");
    (void)PRINTF("\t           255: invalid evaluate mode\r\n");
    (void)PRINTF("\t           If not used, just keep this field empty.\r\n");
#endif
    (void)PRINTF("Examples:\r\n");
    (void)PRINTF("wlan-set-antcfg 1\r\n");
    (void)PRINTF("wlan-set-antcfg 0xffff\r\n");
    (void)PRINTF("wlan-set-antcfg 0xffff 5000\r\n");
#ifdef RW610
    (void)PRINTF("wlan-set-antcfg 0xffff 6000 0\r\n");
#endif
}

#ifndef RW610
static void wlan_antcfg_set(int argc, char *argv[])
{
    int ret;
    uint32_t ant_mode;
    uint16_t evaluate_time = 0;

    if (!(argc >= 2 && argc <= 3))
    {
        dump_wlan_set_antcfg_usage();
        return;
    }

    errno    = 0;
    ant_mode = (uint32_t)strtol(argv[1], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol errno:%d", errno);
        return;
    }

    if ((argc == 3) && (ant_mode != 0xFFFFU))
    {
        dump_wlan_set_antcfg_usage();
        return;
    }

    errno = 0;
    if (argc == 3)
    {
        evaluate_time = (uint16_t)strtol(argv[2], NULL, 10);
    }
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol errno:%d", errno);
        return;
    }

    ret = wlan_set_antcfg(ant_mode, evaluate_time);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Antenna configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("Antenna configuration failed\r\n");
        dump_wlan_set_antcfg_usage();
    }
}
#else
static void wlan_antcfg_set(int argc, char *argv[])
{
    int ret;
    uint32_t ant_mode;
    uint16_t evaluate_time = 0;
    uint8_t evaluate_mode  = 0xFF;

    if (argc < 2 || argc > 4)
    {
        dump_wlan_set_antcfg_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    errno    = 0;
    ant_mode = (uint32_t)strtol(argv[1], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol errno:%d", errno);
        return;
    }

    if ((argc == 3 || argc == 4) && (ant_mode != 0xFFFFU))
    {
        dump_wlan_set_antcfg_usage();
        (void)PRINTF("Error: invalid ant_mode\r\n");
        return;
    }

    errno = 0;
    if (argc == 3 || argc == 4)
    {
        evaluate_time = (uint16_t)strtol(argv[2], NULL, 10);

        if (errno != 0)
        {
            (void)PRINTF("Error during strtol errno:%d", errno);
            return;
        }
    }

    errno = 0;
    if (argc == 4)
    {
        evaluate_mode = (uint8_t)strtol(argv[3], NULL, 10);

        if (errno != 0)
        {
            (void)PRINTF("Error during strtol errno:%d", errno);
            return;
        }

        if ((evaluate_mode != 0) && (evaluate_mode != 1) && (evaluate_mode != 2) && (evaluate_mode != 255))
        {
            dump_wlan_set_antcfg_usage();
            (void)PRINTF("Error: invalid evaluate_mode\r\n");
            return;
        }
    }

    ret = wlan_set_antcfg(ant_mode, evaluate_time, evaluate_mode);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Antenna configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("Antenna configuration failed\r\n");
        dump_wlan_set_antcfg_usage();
    }
}
#endif /*RW610*/

static void dump_wlan_get_antcfg_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-antcfg \r\n");
}

static void wlan_antcfg_get(int argc, char *argv[])
{
    int ret                = -WM_FAIL;
    uint32_t ant_mode      = 0;
    uint16_t evaluate_time = 0;
#ifdef RW610
    uint8_t evaluate_mode = 0;
#endif
    uint16_t current_antenna = 0;

    if (argc != 1)
    {
        dump_wlan_get_antcfg_usage();
        return;
    }

#ifndef RW610
    ret = wlan_get_antcfg(&ant_mode, &evaluate_time, &current_antenna);
#else
    ret               = wlan_get_antcfg(&ant_mode, &evaluate_time, &evaluate_mode, &current_antenna);
#endif
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Mode of Tx/Rx path is : %x\r\n", ant_mode);
        if (ant_mode == 0XFFFFU)
        {
            (void)PRINTF("Evaluate time : %d\r\n", evaluate_time);
#ifdef RW610
            if (evaluate_mode == 0)
                (void)PRINTF("Evaluate mode : Ant1 + Ant2\r\n");
            if (evaluate_mode == 1)
                (void)PRINTF("Evaluate mode : Ant2 + Ant3\r\n");
            if (evaluate_mode == 2)
                (void)PRINTF("Evaluate mode : Ant1 + Ant3\r\n");
            if (evaluate_mode == 0xFF)
                (void)PRINTF("Default diversity mode.\r\n");
#endif
        }
        if (current_antenna > 0)
        {
            (void)PRINTF("Current antenna is Ant%d\n", current_antenna);
        }
    }
    else
    {
        (void)PRINTF("antcfg configuration read failed\r\n");
        dump_wlan_get_antcfg_usage();
    }
}

#if CONFIG_SCAN_CHANNEL_GAP
static void test_wlan_set_scan_channel_gap(int argc, char **argv)
{
    unsigned scan_chan_gap;
    if (argc != 2)
    {
        (void)PRINTF("Invalid arguments\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("wlan-scan-channel-gap <scan_gap_value>\r\n");
        (void)PRINTF("scan_gap_value: [2,500]\r\n");
        return;
    }
    scan_chan_gap = a2hex_or_atoi(argv[1]);
    if (scan_chan_gap < 2 || scan_chan_gap > 500)
    {
        (void)PRINTF("Invaild scan_gap value!\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("wlan-scan-channel-gap <scan_gap_value>\r\n");
        (void)PRINTF("scan_gap_value: [2,500]\r\n");
        return;
    }
    wlan_set_scan_channel_gap(scan_chan_gap);
}
#endif

#if CONFIG_WMM
static void test_wlan_wmm_tx_stats(int argc, char **argv)
{
    int bss_type = atoi(argv[1]);

    wlan_wmm_tx_stats_dump(bss_type);
}
#endif

#if defined(RW610)
static void dump_wlan_set_regioncode_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-regioncode <region-code>\r\n");
    (void)PRINTF("where, region code =\r\n");
    (void)PRINTF("0x00 : World Wide Safe Mode\r\n");
    (void)PRINTF("0x10 : US FCC, Singapore\r\n");
    (void)PRINTF("0x20 : IC Canada\r\n");
    (void)PRINTF("0x30 : ETSI, Australia, Republic of Korea\r\n");
    (void)PRINTF("0x32 : France\r\n");
    (void)PRINTF("0x50 : China\r\n");
    (void)PRINTF("0xFF : Japan\r\n");
}
#endif

static void test_wlan_set_regioncode(int argc, char **argv)
{
#if defined(RW610)
    if (argc != 2)
    {
        dump_wlan_set_regioncode_usage();
        return;
    }

    if (is_uap_started())
    {
        (void)PRINTF("Error: region code can not be set after uAP start!\r\n");
        return;
    }

    errno             = 0;
    t_u32 region_code = (t_u32)strtol(argv[1], NULL, 0);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol errno:%d", errno);
        return;
    }
    int rv = wlan_set_region_code(region_code);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set region code: 0x%x\r\n", region_code);
    }
    else
    {
        (void)PRINTF("Region code: 0x%x set\r\n", region_code);
    }
#else
    (void)PRINTF("Set Region code Feature not supported.\r\n");
    return;
#endif
}

static void test_wlan_get_regioncode(int argc, char **argv)
{
    t_u32 region_code = 0;
    int rv            = wlan_get_region_code(&region_code);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get region code: 0x%x\r\n", region_code);
    }
    else
    {
        (void)PRINTF("Region code: 0x%x\r\n", region_code);
    }
}
static void test_wlan_set_mac_address(int argc, char **argv)
{
    int ret;
    uint8_t raw_mac[MLAN_MAC_ADDR_LENGTH];

    if (argc != 2)
    {
        (void)PRINTF("Usage: %s MAC_Address\r\n", argv[0]);
        return;
    }

    ret = get_mac(argv[1], (char *)raw_mac, ':');
    if (ret != 0)
    {
        (void)PRINTF("Error: invalid MAC argument\r\n");
        return;
    }

    wlan_set_mac_addr(raw_mac);
}

#if defined(RW610) && (CONFIG_WIFI_RESET)
static void test_wlan_reset(int argc, char **argv)
{
    int option;

    option = atoi(argv[1]);
    if (argc != 2 || (option != 0 && option != 1 && option != 2))
    {
        (void)PRINTF("Usage: %s <options>\r\n", argv[0]);
        (void)PRINTF("0 to Disable WiFi\r\n");
        (void)PRINTF("1 to Enable WiFi\r\n");
        (void)PRINTF("2 to Reset WiFi\r\n");
        return;
    }
#if CONFIG_CSI
    if (option == 2)
    {
        (void)memset((void *)&g_csi_params, 0, sizeof(g_csi_params));
    }
#endif
#if CONFIG_NET_MONITOR
    if (option == 2)
    {
        (void)memset((void *)&g_net_monitor_param, 0, sizeof(g_net_monitor_param));
    }
#endif

    wlan_reset((cli_reset_option)option);
}
#endif

#if CONFIG_ECSA
static void test_wlan_uap_set_ecsa_cfg(int argc, char **argv)
{
    int ret;
    t_u8 block_tx     = 0;
    t_u8 oper_class   = 0;
    t_u8 new_channel  = 0;
    t_u8 switch_count = 0;
    t_u8 band_width   = 0;

    if ((5 == argc) || (6 == argc))
    {
        block_tx     = (t_u8)atoi(argv[1]);
        oper_class   = (t_u8)atoi(argv[2]);
        new_channel  = (t_u8)atoi(argv[3]);
        switch_count = (t_u8)atoi(argv[4]);

        if (6 == argc)
        {
            band_width = (t_u8)atoi(argv[5]);
        }
    }
    else
    {
        (void)PRINTF("Error        : invalid number of arguments \r\n");
        (void)PRINTF("Usage        : %s <block_tx> <oper_class> <new_channel> <switch_count> <bandwidth>\r\n", argv[0]);
        (void)PRINTF("block_tx     : 0 -- no need to block traffic, 1 -- need block traffic \r\n");
        (void)PRINTF(
            "oper_class   : Operating class according to IEEE std802.11 spec. when 0 is used, only CSA IE will be "
            "used\r\n");
        (void)PRINTF("new_channel  : The channel will switch to \r\n");
        (void)PRINTF("switch count : Channel switch time to send ECSA ie \r\n");
        (void)PRINTF("bandwidth    : Channel width switch to(optional),RW610 only support 20M channels \r\n");

        (void)PRINTF("\r\nUsage example : wlan-set-ecsa-cfg 1 0 36 10 1 \r\n");

        return;
    }

    /* Disable action Temporary */
    if (0 == switch_count)
    {
        (void)PRINTF("Error : invalid arguments \r\n");
        (void)PRINTF("argv[4] switch_count cannot be 0\r\n");
        return;
    }

    ret = wlan_uap_set_ecsa_cfg(block_tx, oper_class, new_channel, switch_count, band_width);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set ecsa cfg \r\n");
    }
}
#endif /* CONFIG_ECSA */

#if CONFIG_11AX
#if CONFIG_MMSF
static void dump_wlan_set_mmsf_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("set mmsf:\r\n");
    (void)PRINTF("wlan-set-mmsf <enable> <Density> <MMSF>\r\n");
    (void)PRINTF("enable:\r\n");
    (void)PRINTF("      0: disable\r\n");
    (void)PRINTF("      1: enable\r\n");
    (void)PRINTF("Density:\r\n");
    (void)PRINTF("      Range: [0x0,0xFF]. Default value is 0x30.\r\n");
    (void)PRINTF("      Pls enter value like this: 0x20 or 0X20\r\n");
    (void)PRINTF("MMSF:\r\n");
    (void)PRINTF("      Range: [0x0,0xFF]. Default value is 0x6.\r\n");
    (void)PRINTF("      Pls enter value like this: 0x20 or 0X20\r\n");
}

static void test_wlan_set_mmsf(int argc, char **argv)
{
    t_u32 value;
    int ret;
    t_u8 enable, Density, MMSF;

    if (argc < 2 || argc > 4)
    {
        dump_wlan_set_mmsf_usage();
        return;
    }

    if (argc >= 2)
    {
        if (get_uint(argv[1], &value, strlen(argv[1])) || (value != 0 && value != 1))
        {
            dump_wlan_set_mmsf_usage();
            return;
        }
        enable  = value & 0xFF;
        Density = WLAN_AMPDU_DENSITY;
        MMSF    = WLAN_AMPDU_MMSF;
    }

    if (argc >= 3)
    {
        if (argv[2][0] == '0' && (argv[2][1] == 'x' || argv[2][1] == 'X'))
            value = a2hex_or_atoi(argv[2]);
        else
        {
            dump_wlan_set_mmsf_usage();
            return;
        }
        Density = value & 0xFF;
    }

    if (argc == 4)
    {
        if (argv[3][0] == '0' && (argv[3][1] == 'x' || argv[3][1] == 'X'))
            value = a2hex_or_atoi(argv[3]);
        else
        {
            dump_wlan_set_mmsf_usage();
            return;
        }
        MMSF = value & 0xFF;
    }

    ret = wlan_set_mmsf(enable, Density, MMSF);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set MMSF config.\r\n");
    }
    else
    {
        (void)PRINTF("Success to set MMSF config.\r\n");
    }

    return;
}

static void test_wlan_get_mmsf(int argc, char **argv)
{
    int ret;
    t_u8 enable, Density, MMSF;
    (void)memset(&enable, 0x0, sizeof(t_u8));
    (void)memset(&Density, 0x0, sizeof(t_u8));
    (void)memset(&MMSF, 0x0, sizeof(t_u8));

    ret = wlan_get_mmsf(&enable, &Density, &MMSF);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to get MMSF configure.\r\n");
    }
    else
    {
        (void)PRINTF("MMSF configure:\r\n");
        (void)PRINTF("Enable MMSF: %s\r\n", enable == 0 ? "Disable" : "Enable");
        (void)PRINTF("Density: 0x%02x\r\n", Density);
        (void)PRINTF("MMSF: 0x%02x\r\n", MMSF);
    }

    return;
}
#endif
#endif /* CONFIG_11AX */

#if CONFIG_WIFI_RECOVERY
static void test_wlan_recovery_test(int argc, char **argv)
{
    int ret;
    ret = wlan_recovery_test();

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("timeout happends.\r\n");
    }
    return;
}
#endif

#if CONFIG_SUBSCRIBE_EVENT_SUPPORT
/**
 *  @brief This function print the get subscribe event from firmware for user test.
 */
static void print_get_sub_event(wlan_ds_subscribe_evt *sub_evt)
{
    t_u16 evt_bitmap = sub_evt->evt_bitmap;
    PRINTF("evt_bitmap = %u\r\n", evt_bitmap);
    if (evt_bitmap & SUBSCRIBE_EVT_RSSI_LOW)
    {
        PRINTF("rssi low is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->low_rssi, sub_evt->low_rssi_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_RSSI_HIGH)
    {
        PRINTF("rssi high is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->high_rssi, sub_evt->high_rssi_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_SNR_LOW)
    {
        PRINTF("snr low is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->low_snr, sub_evt->low_snr_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_SNR_HIGH)
    {
        PRINTF("snr high is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->high_snr, sub_evt->high_snr_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_MAX_FAIL)
    {
        PRINTF("max fail is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->failure_count, sub_evt->failure_count_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_BEACON_MISSED)
    {
        PRINTF("beacon miss is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->beacon_miss, sub_evt->beacon_miss_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_DATA_RSSI_LOW)
    {
        PRINTF("data rssi low is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->data_low_rssi, sub_evt->data_low_rssi_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_DATA_RSSI_HIGH)
    {
        PRINTF("data rssi high is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->data_high_rssi, sub_evt->data_high_rssi_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_DATA_SNR_LOW)
    {
        PRINTF("data snr low is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->data_low_snr, sub_evt->data_low_snr_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_DATA_SNR_HIGH)
    {
        PRINTF("data snr high is enabled! ");
        PRINTF("value = %u, freq = %u\r\n", sub_evt->data_high_snr, sub_evt->data_high_snr_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_LINK_QUALITY)
    {
        PRINTF("link quality is enabled! ");
        PRINTF(
            "link_snr = %u, link_snr_freq = %u, "
            "link_rate = %u, link_rate_freq = %u, "
            "link_tx_latency = %u, link_tx_lantency_freq = %u\r\n",
            sub_evt->link_snr, sub_evt->link_snr_freq, sub_evt->link_rate, sub_evt->link_rate_freq,
            sub_evt->link_tx_latency, sub_evt->link_tx_lantency_freq);
    }
    if (evt_bitmap & SUBSCRIBE_EVT_PRE_BEACON_LOST)
    {
        PRINTF("pre beacon lost is enabled! ");
        PRINTF("value = %u\r\n", sub_evt->pre_beacon_miss);
    }
}

/**
 *  @brief This function dump the usage of wlan-subscribe-event cmd for user test.
 */
static void dump_wlan_subscribe_event_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Subscribe event to firmware:\r\n");
    (void)PRINTF("    wlan-subscribe-event <action> <type> <value> <freq>\r\n");
    (void)PRINTF("Options: \r\n");
    (void)PRINTF("    <action>  : 1:set, 2:get, 3:clear\r\n");
    (void)PRINTF(
        "    <type>: 0:rssi_low, 1:rssi_high 2:snr_low, 3:snr_high, 4:max_fail, 5:beacon_missed, 6:data_rssi_low, "
        "7:data_rssi_high, 8:data_snr_low, 9:data_snr_high, 10:link_quality, 11:pre_beacon_lost\r\n");
    (void)PRINTF("    <value>  : when action is set, specific int type value\r\n");
    (void)PRINTF(
        "    <freq>  : when action is set, specific unsigned int type freq, when the freq = 0, "
        "the event will trigger one time, and the freq = 1, the event will continually trigger.\r\n");
    (void)PRINTF("For example:\r\n");
    (void)PRINTF(
        "    wlan-subscribe-event set 0 50 0 : Subscribe the rssi low event, threshold is 50, freq is 0\r\n"
        "    wlan-subscribe-event set 2 50 0 : Subscribe the snr low event, threshold is 50, freq is 0\r\n"
        "    wlan-subscribe-event set 4 50 0 : Subscribe the max_fail event, threshold is 50, freq is 0\r\n"
        "    wlan-subscribe-event set 5 50 0 : Subscribe the beacon_missed event, threshold is 50, freq is 0\r\n"
        "    wlan-subscribe-event set 6 50 0 : Subscribe the data rssi low event, threshold is 50, freq is 0\r\n"
        "    wlan-subscribe-event set 8 50 0 : Subscribe the data snr low event, threshold is 50, freq is 0\r\n"
        "    wlan-subscribe-event set 11 50 0 : Subscribe the pre_beacon_lost event, threshold is 50, freq is 0\r\n");
    (void)PRINTF(
        "    wlan-subscribe-event set 10 5 0 5 0 5 0  : Subscribe the link quanlity event"
        "    link_snr threshold is 5, link_snr freq is 0"
        "    link_rate threshold is 5, link_rate freq is 0"
        "    link_tx_latency threshold is 5, link_tx_latency freq is 0\r\n");
    (void)PRINTF("    wlan-subscribe-event get      : Get the all subscribe event parameter\r\n");
    (void)PRINTF(
        "    wlan-subscribe-event clear 0  : Disable the rssi_low event\r\n"
        "    wlan-subscribe-event clear 2  : Disable the snr_low event\r\n"
        "    wlan-subscribe-event clear 4  : Disable the max_fail event\r\n"
        "    wlan-subscribe-event clear 5  : Disable the beacon_missed event\r\n"
        "    wlan-subscribe-event clear 6  : Disable the data_rssi_low event\r\n"
        "    wlan-subscribe-event clear 8  : Disable the data_snr_low event\r\n"
        "    wlan-subscribe-event clear 10 : Disable the link_quality event\r\n"
        "    wlan-subscribe-event clear 11 : Disable the pre_beacon_lost event\r\n");
}

/**
 *  @brief This function subscribe event to firmware for user test.
 */
static void test_wlan_subscribe_event(int argc, char **argv)
{
    int ret                   = 0;
    unsigned int thresh_value = 0, freq = 0;

    /*analyse action type*/
    switch (argc)
    {
    case 2:
    {
        if (strncmp(argv[1], "get", strlen(argv[1])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        wlan_ds_subscribe_evt sub_evt;
        ret = wlan_get_subscribe_event(&sub_evt);
        if (ret == WM_SUCCESS)
            print_get_sub_event(&sub_evt);
        break;
    }
    case 3:
    {
        if (strncmp(argv[1], "clear", strlen(argv[1])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        unsigned int event_id = MAX_EVENT_ID;
        if (get_uint(argv[2], &event_id, strlen(argv[2])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (event_id >= MAX_EVENT_ID)
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        ret = wlan_clear_subscribe_event(event_id);
        break;
    }
    case 5:
    {
        if (strncmp(argv[1], "set", strlen(argv[1])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (get_uint(argv[3], &thresh_value, strlen(argv[3])) || get_uint(argv[4], &freq, strlen(argv[4])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        unsigned int event_id = MAX_EVENT_ID;
        if (get_uint(argv[2], &event_id, strlen(argv[2])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (event_id >= MAX_EVENT_ID)
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        ret = wlan_set_subscribe_event(event_id, thresh_value, freq);
        break;
    }
    case 9:
    {
        unsigned int link_snr = 0, link_snr_freq = 0, link_rate = 0;
        unsigned int link_rate_freq = 0, link_tx_latency = 0, link_tx_lantency_freq = 0;
        if (strncmp(argv[1], "set", strlen(argv[1])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (get_uint(argv[3], &link_snr, strlen(argv[3])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (get_uint(argv[4], &link_snr_freq, strlen(argv[4])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (get_uint(argv[5], &link_rate, strlen(argv[5])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (get_uint(argv[6], &link_rate_freq, strlen(argv[6])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (get_uint(argv[7], &link_tx_latency, strlen(argv[7])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (get_uint(argv[8], &link_tx_lantency_freq, strlen(argv[8])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        unsigned int event_id = MAX_EVENT_ID;
        if (get_uint(argv[2], &event_id, strlen(argv[2])))
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        if (event_id >= MAX_EVENT_ID)
        {
            dump_wlan_subscribe_event_usage();
            return;
        }
        ret = wlan_set_threshold_link_quality(event_id, link_snr, link_snr_freq, link_rate, link_rate_freq,
                                              link_tx_latency, link_tx_lantency_freq);
    }
    break;
    default:
        dump_wlan_subscribe_event_usage();
        return;
    }
    if (ret == WM_E_INVAL)
        dump_wlan_subscribe_event_usage();
    else if (ret != WM_SUCCESS)
        (void)PRINTF("wlan-subscribe-event unkown fail\r\n");
    return;
}
#endif

#if CONFIG_WIFI_REG_ACCESS
static void dump_wlan_reg_access_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Read the register:\r\n");
    (void)PRINTF("    wlan-reg-access <type> <offset>\r\n");
    (void)PRINTF("Write the register:\r\n");
    (void)PRINTF("    wlan-reg-access <type> <offset> <value>\r\n");
    (void)PRINTF("Options: \r\n");
    (void)PRINTF("    <type>  : 1:MAC, 2:BBP, 3:RF, 4:CAU\r\n");
    (void)PRINTF("    <offset>: offset of register\r\n");
    (void)PRINTF("For example:\r\n");
    (void)PRINTF("    wlan-reg-access 1 0x9b8             : Read the MAC register\r\n");
    (void)PRINTF("    wlan-reg-access 1 0x9b8 0x80000000 : Write 0x80000000 to MAC register\r\n");
}

static void test_wlan_reg_access(int argc, char **argv)
{
    t_u32 type, offset;
    t_u32 value  = 0;
    t_u16 action = ACTION_GET;
    int ret;

    if (argc < 3 || argc > 4)
    {
        dump_wlan_reg_access_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    if ((a2hex_or_atoi(argv[1]) != 1 && a2hex_or_atoi(argv[1]) != 2 && a2hex_or_atoi(argv[1]) != 3 &&
            a2hex_or_atoi(argv[1]) != 4))
    {
        dump_wlan_reg_access_usage();
        (void)PRINTF("Error: Illegal register type %s. Must be either '1','2','3' or '4'.\r\n", argv[1]);
        return;
    }
    type   = a2hex_or_atoi(argv[1]);
    offset = a2hex_or_atoi(argv[2]);
    if (argc == 4)
    {
        action = ACTION_SET;
        value  = a2hex_or_atoi(argv[3]);
    }

    ret = wlan_reg_access((wifi_reg_t)type, action, offset, (uint32_t *)&value);

    if (ret == WM_SUCCESS)
    {
        if (action == ACTION_GET)
            (void)PRINTF("Value = 0x%x\r\n", value);
        else
            (void)PRINTF("Set the register successfully\r\n");
    }
    else
        (void)PRINTF("Read/write register failed");
}
#endif

#if CONFIG_WMM_UAPSD
static void test_wlan_wmm_uapsd_qosinfo(int argc, char **argv)
{
    unsigned int qos_info = 0xf;
    if (argc == 1)
    {
        wlan_wmm_uapsd_qosinfo((t_u8 *)&qos_info, 0);
        (void)PRINTF("qos_info = %d\r\n", qos_info);
    }
    else if (argc == 2 && !get_uint(argv[1], &qos_info, strlen(argv[1])))
    {
        if (qos_info == 0)
            (void)PRINTF("qos_info can't be zero, input %s\r\n", argv[1]);
        else
            wlan_wmm_uapsd_qosinfo((t_u8 *)&qos_info, 1);
    }
    else
    {
        (void)PRINTF("Usage: %s <null|qos_info>\r\n", argv[0]);
        (void)PRINTF("set qos_info value to UAPSD QOS_INFO\r\n");
        (void)PRINTF("bit0:VO; bit1:VI; bit2:BK; bit3:BE\r\n");
        return;
    }
}
static void test_wlan_set_wmm_uapsd(int argc, char **argv)
{
    t_u8 enable;

    enable = atoi(argv[1]);
    if (argc != 2 || (enable != 0 && enable != 1))
    {
        (void)PRINTF("Usage: %s <enable>\r\n", argv[0]);
        (void)PRINTF("0 to Disable UAPSD\r\n");
        (void)PRINTF("1 to Enable UAPSD\r\n");
        return;
    }

    (void)wlan_set_wmm_uapsd(enable);
}

static void test_wlan_sleep_period(int argc, char **argv)
{
    unsigned int period = 0;
    if (argc == 1)
    {
        wlan_sleep_period(&period, 0);
        (void)PRINTF("period = %d\r\n", period);
    }
    else if (argc == 2 && !get_uint(argv[1], &period, strlen(argv[1])))
        wlan_sleep_period(&period, 1);
    else
    {
        (void)PRINTF("Usage: %s <period(ms)>\r\n", argv[0]);
    }
}
#endif


#if CONFIG_TX_AMPDU_PROT_MODE
static void dump_wlan_tx_ampdu_prot_mode_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-tx-ampdu-prot-mode <mode>\r\n");
    (void)PRINTF("    <mode>: 0 - Set RTS/CTS mode \r\n");
    (void)PRINTF("            1 - Set CTS2SELF mode \r\n");
    (void)PRINTF("            2 - Disable Protection mode \r\n");
    (void)PRINTF("            3 - Set Dynamic RTS/CTS mode \r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("    wlan-tx-ampdu-prot-mode\r\n");
    (void)PRINTF("    - Get currently set protection mode for TX AMPDU.\r\n");
    (void)PRINTF("    wlan-tx-ampdu-prot-mode 1\r\n");
    (void)PRINTF("    - Set protection mode for TX AMPDU to CTS2SELF.\r\n");
}

static void test_wlan_tx_ampdu_prot_mode(int argc, char **argv)
{
    tx_ampdu_prot_mode_para data;

    if (argc > 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_tx_ampdu_prot_mode_usage();
        return;
    }

    /* GET */
    if (argc == 1)
    {
        dump_wlan_tx_ampdu_prot_mode_usage();
        wlan_tx_ampdu_prot_mode(&data, ACTION_GET);
        (void)PRINTF("\r\nTx AMPDU protection mode: ");
        switch (data.mode)
        {
        case TX_AMPDU_RTS_CTS:
            (void)PRINTF("RTS/CTS\r\n");
            break;
        case TX_AMPDU_CTS_2_SELF:
            (void)PRINTF("CTS-2-SELF\r\n");
            break;
        case TX_AMPDU_DISABLE_PROTECTION:
            (void)PRINTF("Disabled\r\n");
            break;
        case TX_AMPDU_DYNAMIC_RTS_CTS:
            (void)PRINTF("DYNAMIC RTS/CTS\r\n");
            break;
        default:
            (void)PRINTF("Invalid protection mode\r\n");
            break;
        }
    }
    else /* SET */
    {
        data.mode = atoi(argv[1]);
        if (data.mode < 0 || data.mode > 3)
        {
            (void)PRINTF("Error: invalid protection mode\r\n");
            dump_wlan_tx_ampdu_prot_mode_usage();
            return;
        }
        wlan_tx_ampdu_prot_mode(&data, ACTION_SET);
    }
}
#endif

#if CONFIG_CSI
static void dump_wlan_csi_filter_usage()
{
    (void)PRINTF("Error: invalid number of arguments\r\n");
    (void)PRINTF("Usage : wlan-set-csi-filter <opt> <macaddr> <pkt_type> <type> <flag>\r\n");
    (void)PRINTF("opt   : add/delete/clear/dump \r\n");
    (void)PRINTF("add   : All options need to be filled in \r\n");
    (void)PRINTF("delete: Delete recent filter information \r\n");
    (void)PRINTF("clear : Clear all filter information \r\n");
    (void)PRINTF("dump  : Dump csi cfg information \r\n");

    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-set-csi-filter add 00:18:E7:ED:2D:C1 255 255 0 \r\n");
    (void)PRINTF("wlan-set-csi-filter delete \r\n");
    (void)PRINTF("wlan-set-csi-filter clear \r\n");
    (void)PRINTF("wlan-set-csi-filter dump \r\n");
}

void dump_csi_param_header()
{
    (void)PRINTF("\r\nThe current csi_param is: \r\n");
    if (g_csi_params.bss_type == 0)
        (void)PRINTF("bss_type      : sta\r\n");
    else
        (void)PRINTF("bss_type      : uap\r\n");
    (void)PRINTF("csi_enable    : %d \r\n", g_csi_params.csi_enable);
    (void)PRINTF("head_id       : %d \r\n", g_csi_params.head_id);
    (void)PRINTF("tail_id       : %d \r\n", g_csi_params.tail_id);
    (void)PRINTF("csi_filter_cnt: %d \r\n", g_csi_params.csi_filter_cnt);
    (void)PRINTF("chip_id       : %d \r\n", g_csi_params.chip_id);
    (void)PRINTF("band_config   : %d \r\n", g_csi_params.band_config);
    (void)PRINTF("channel       : %d \r\n", g_csi_params.channel);
    (void)PRINTF("csi_monitor_enable : %d \r\n", g_csi_params.csi_monitor_enable);
    (void)PRINTF("ra4us         : %d \r\n", g_csi_params.ra4us);

    (void)PRINTF("\r\n");
}

void set_csi_param_header(t_u8 bss_type,
                          t_u16 csi_enable,
                          t_u32 head_id,
                          t_u32 tail_id,
                          t_u8 chip_id,
                          t_u8 band_config,
                          t_u8 channel,
                          t_u8 csi_monitor_enable,
                          t_u8 ra4us)
{
    g_csi_params.bss_type           = bss_type;
    g_csi_params.csi_enable         = csi_enable;
    g_csi_params.head_id            = head_id;
    g_csi_params.tail_id            = tail_id;
    g_csi_params.chip_id            = chip_id;
    g_csi_params.band_config        = band_config;
    g_csi_params.channel            = channel;
    g_csi_params.csi_monitor_enable = csi_monitor_enable;
    g_csi_params.ra4us              = ra4us;

    dump_csi_param_header();
}

void set_csi_filter(t_u8 pkt_type, t_u8 subtype, t_u8 flags, int op_index, t_u8 *mac)
{
    t_u8 temp_filter_cnt = g_csi_params.csi_filter_cnt;
    int i                = 0;

    switch (op_index)
    {
    case CSI_FILTER_OPT_ADD:
        if (temp_filter_cnt < CSI_FILTER_MAX)
        {
            (void)memcpy(&g_csi_params.csi_filter[temp_filter_cnt].mac_addr[0], mac, MLAN_MAC_ADDR_LENGTH);
            g_csi_params.csi_filter[temp_filter_cnt].pkt_type = pkt_type;
            g_csi_params.csi_filter[temp_filter_cnt].subtype  = subtype;
            g_csi_params.csi_filter[temp_filter_cnt].flags    = flags;
            g_csi_params.csi_filter_cnt++;
        }
        else
        {
            (void)PRINTF("max csi filter cnt is 16 \r\n");
            return;
        }
        break;

    case CSI_FILTER_OPT_DELETE:
        if (temp_filter_cnt > 0)
        {
            memset(&g_csi_params.csi_filter[temp_filter_cnt], 0, sizeof(wifi_csi_filter_t));
            g_csi_params.csi_filter_cnt--;
        }
        else
        {
            (void)PRINTF("csi filter cnt is 0 \r\n");
            return;
        }
        break;

    case CSI_FILTER_OPT_CLEAR:
        for (i = 0; i < temp_filter_cnt; i++)
        {
            memset(&g_csi_params.csi_filter[i], 0, sizeof(wifi_csi_filter_t));
        }
        g_csi_params.csi_filter_cnt = 0;
        break;

    case CSI_FILTER_OPT_DUMP:
        dump_csi_param_header();

        for (i = 0; i < temp_filter_cnt; i++)
        {
            (void)PRINTF("mac_addr      : %02X:%02X:%02X:%02X:%02X:%02X \r\n",
                         g_csi_params.csi_filter[i].mac_addr[0], g_csi_params.csi_filter[i].mac_addr[1],
                         g_csi_params.csi_filter[i].mac_addr[2], g_csi_params.csi_filter[i].mac_addr[3],
                         g_csi_params.csi_filter[i].mac_addr[4], g_csi_params.csi_filter[i].mac_addr[5]);

            (void)PRINTF("pkt_type      : %d \r\n", g_csi_params.csi_filter[i].pkt_type);
            (void)PRINTF("subtype       : %d \r\n", g_csi_params.csi_filter[i].subtype);
            (void)PRINTF("flags         : %d \r\n", g_csi_params.csi_filter[i].flags);
            (void)PRINTF("\r\n");
        }
        break;

    default:
        (void)PRINTF("unknown argument!\r\n");
        break;
    }
}

int csi_data_recv_user(void *buffer, size_t data_len)
{
    PRINTF("CSI user callback: Event CSI data\r\n");
    dump_hex(buffer, data_len);
    return WM_SUCCESS;
}

static void test_wlan_set_csi_param_header(int argc, char **argv)
{
    t_u8 bss_type           = 0;
    t_u16 csi_enable        = 0;
    t_u32 head_id           = 0;
    t_u32 tail_id           = 0;
    t_u8 chip_id            = 0;
    t_u8 band_config        = 0;
    t_u8 channel            = 0;
    t_u8 csi_monitor_enable = 0;
    t_u8 ra4us              = 0;
    int ret                 = -1;

    if (argc != 10)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        (void)PRINTF(
            "Usage: %s <sta/uap> <csi_enable> <head_id> <tail_id> <chip_id> <band_config> <channel> "
            "<csi_monitor_enable> "
            "<ra4us>\r\n\r\n",
            argv[0]);

        (void)PRINTF("[csi_enable] :1/2 to Enable/Disable CSI\r\n");
        (void)PRINTF("[head_id, head_id, chip_id] are used to seperate CSI event records received from FW\r\n");
        (void)PRINTF(
            "[Bandcfg] defined as below: \r\n"
            "    Band Info - (00)=2.4GHz, (01)=5GHz \r\n"
            "    t_u8  chanBand    : 2;\r\n"
            "    Channel Width - (00)=20MHz, (10)=40MHz, (11)=80MHz\r\n"
            "    t_u8  chanWidth   : 2;\r\n"
            "    Secondary Channel Offset - (00)=None, (01)=Above, (11)=Below\r\n"
            "    t_u8  chan2Offset : 2;\r\n"
            "    Channel Selection Mode - (00)=manual, (01)=ACS, (02)=Adoption mode\r\n"
            "    t_u8  scanMode    : 2;\r\n");
        (void)PRINTF("[channel] : monitor channel number\r\n");
        (void)PRINTF("[csi_monitor_enable] : 1-csi_monitor enable, 0-MAC filter enable\r\n");
        (void)PRINTF(
            "[ra4us] : 1/0 to Enable/DisEnable CSI data received in cfg channel with mac addr filter, not only RA is "
            "us or other\r\n");

        (void)PRINTF("\r\nUsage example : \r\n");
        (void)PRINTF("wlan-set-csi-param-header sta 1 66051 66051 170 0 11 1 1\r\n");

        dump_csi_param_header();

        return;
    }

    /*
     * csi param header headid, tailid, chipid are used to seperate CSI event records received from FW.
     * FW adds user configured headid, chipid and tailid for each CSI event record.
     * User could configure these fields and used these fields to parse CSI event buffer and do verification.
     * All the CSI filters share the same CSI param header.
     */
    if (string_equal("sta", argv[1]))
        bss_type = 0;
    else if (string_equal("uap", argv[1]))
        bss_type = 1;
    else
    {
        PRINTF("Please put sta or uap\r\n");
        return;
    }
    csi_enable         = (t_u16)atoi(argv[2]);
    head_id            = (t_u32)atoi(argv[3]);
    tail_id            = (t_u32)atoi(argv[4]);
    chip_id            = (t_u8)atoi(argv[5]);
    band_config        = (t_u8)atoi(argv[6]);
    channel            = (t_u8)atoi(argv[7]);
    csi_monitor_enable = (t_u8)atoi(argv[8]);
    ra4us              = (t_u8)atoi(argv[9]);

    if (csi_enable == 1)
    {
        ret = wlan_register_csi_user_callback(csi_data_recv_user);
        if (ret != WM_SUCCESS)
        {
            PRINTF("Error during register csi user callback\r\n");
        }
    }

    memcpy((void *)&g_csi_params, (void *)wlan_get_csi_cfg_param_default(), sizeof(wlan_csi_config_params_t));
    set_csi_param_header(bss_type, csi_enable, head_id, tail_id, chip_id, band_config, channel, csi_monitor_enable,
                         ra4us);
    wlan_set_csi_cfg_param_default(&g_csi_params);
}

static void test_wlan_set_csi_filter(int argc, char **argv)
{
    int ret = 0;
    t_u8 raw_mac[MLAN_MAC_ADDR_LENGTH];
    t_u8 pkt_type = 0;
    t_u8 subtype  = 0;
    t_u8 flags    = 0;
    int op_index  = 0;

    if (argc < 2)
    {
        dump_wlan_csi_filter_usage();
        return;
    }

    if (string_equal("add", argv[1]))
    {
        if (6 == argc)
        {
            ret = get_mac(argv[2], (char *)raw_mac, ':');
            if (ret != 0)
            {
                (void)PRINTF("Error: invalid MAC argument\r\n");
                return;
            }
            if ((memcmp(&raw_mac[0], broadcast_mac, MLAN_MAC_ADDR_LENGTH) == 0) || (raw_mac[0] & 0x01))
            {
                (void)PRINTF("Error: only support unicast mac\r\n");
                return;
            }

            /*
             * pkt_type and subtype are the 802.11 framecontrol pkttype and subtype
             * flags:
             * bit0 reserved, must be 0
             * bit1 set to 1: wait for trigger
             * bit2 set to 1: send csi error event when timeout
             */
            pkt_type = (t_u8)atoi(argv[3]);
            subtype  = (t_u8)atoi(argv[4]);
            flags    = (t_u8)atoi(argv[5]);

            op_index = CSI_FILTER_OPT_ADD;
        }
        else
        {
            dump_wlan_csi_filter_usage();
            return;
        }
    }
    else if (string_equal("delete", argv[1]))
        op_index = CSI_FILTER_OPT_DELETE;
    else if (string_equal("clear", argv[1]))
        op_index = CSI_FILTER_OPT_CLEAR;
    else if (string_equal("dump", argv[1]))
        op_index = CSI_FILTER_OPT_DUMP;
    else
    {
        (void)PRINTF("Unknown argument!\r\n");
        return;
    }

    memcpy((void *)&g_csi_params, (void *)wlan_get_csi_cfg_param_default(), sizeof(wlan_csi_config_params_t));
    set_csi_filter(pkt_type, subtype, flags, op_index, raw_mac);
    wlan_set_csi_cfg_param_default(&g_csi_params);
}

static void test_wlan_csi_cfg(int argc, char **argv)
{
    int ret;

    ret = wlan_csi_cfg(&g_csi_params);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to send csi cfg\r\n");
    }
}
#endif

#if (CONFIG_11K) || (CONFIG_11V) || (CONFIG_11R) || (CONFIG_ROAMING)
static void test_wlan_rssi_low_threshold(int argc, char **argv)
{
    uint8_t rssi_threshold;

    if (argc < 2)
    {
        (void)PRINTF("Usage: %s <rssi threshold value>\r\n", argv[0]);
        (void)PRINTF("Error: Default value is 70. Specify the value you want to set as threshold.\r\n");
        return;
    }

    errno          = 0;
    rssi_threshold = (uint8_t)strtol(argv[1], NULL, 10);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtol:rssi_threshold errno:%d\r\n", errno);
        return;
    }

    wlan_set_rssi_low_threshold(rssi_threshold);

    (void)PRINTF("rssi threshold set successfully.\r\n");
}
#endif

#if CONFIG_NET_MONITOR
static void dump_wlan_set_monitor_filter_usage()
{
    (void)PRINTF("Error : invalid arguments\r\n");
    (void)PRINTF("Usage : wlan-set-monitor-filter <opt> <macaddr>\r\n");
    (void)PRINTF("opt   : add/delete/clear/dump \r\n");
    (void)PRINTF("add   : All options need to be filled in \r\n");
    (void)PRINTF("delete: Delete recent mac addr \r\n");
    (void)PRINTF("clear : Clear all mac addr \r\n");
    (void)PRINTF("dump  : Dump monitor cfg information \r\n");

    (void)PRINTF("\r\nUsage example ：\r\n");
    (void)PRINTF("wlan-set-monitor-filter add 64:64:4A:D6:FA:7B \r\n");
    (void)PRINTF("wlan-set-monitor-filter delete \r\n");
    (void)PRINTF("wlan-set-monitor-filter clear  \r\n");
    (void)PRINTF("wlan-set-monitor-filter dump   \r\n");
}

static void dump_monitor_param()
{
    int i = 0;

    (void)PRINTF("\r\n");
    (void)PRINTF("current parameters: \r\n");
    (void)PRINTF("action            : %d \r\n", g_net_monitor_param.action);
    (void)PRINTF("monitor_activity  : %d \r\n", g_net_monitor_param.monitor_activity);
    (void)PRINTF("filter_flags      : %d \r\n", g_net_monitor_param.filter_flags);
    (void)PRINTF("radio_type        : %d \r\n", g_net_monitor_param.radio_type);
    (void)PRINTF("chan_number       : %d \r\n", g_net_monitor_param.chan_number);
    (void)PRINTF("filter_num        : %d \r\n", g_net_monitor_param.filter_num);
    (void)PRINTF("\r\n");

    for (i = 0; i < g_net_monitor_param.filter_num; i++)
    {
        (void)PRINTF("mac_addr      : %02X:%02X:%02X:%02X:%02X:%02X \r\n", g_net_monitor_param.mac_addr[i][0],
                     g_net_monitor_param.mac_addr[i][1], g_net_monitor_param.mac_addr[i][2],
                     g_net_monitor_param.mac_addr[i][3], g_net_monitor_param.mac_addr[i][4],
                     g_net_monitor_param.mac_addr[i][5]);
    }
}

static void test_wlan_set_monitor_param(int argc, char **argv)
{
    if (argc != 6)
    {
        (void)PRINTF("Error             : invalid number of arguments\r\n");
        (void)PRINTF("Usage             : %s <action> <monitor_activity> <filter_flags> <radio_type> <chan_number>\r\n",
                     argv[0]);
        (void)PRINTF("action            : 0/1 to Action Get/Set \r\n");
        (void)PRINTF("monitor_activity  : 1 to enable and other parameters to disable monitor activity \r\n");
        (void)PRINTF("filter_flags      : network monitor fitler flag \r\n");
        (void)PRINTF("chan_number       : channel to monitor \r\n");

        (void)PRINTF("\r\nUsage example ：\r\n");
        (void)PRINTF("wlan-set-monitor-param 1 1 7 0 1 \r\n");

        dump_monitor_param();
        return;
    }

    g_net_monitor_param.action           = (t_u16)atoi(argv[1]);
    g_net_monitor_param.monitor_activity = (t_u16)atoi(argv[2]);

    /*
     * filter_flags:
     * bit 0: (1/0) enable/disable management frame
     * bit 1: (1/0) enable/disable control frame
     * bit 2: (1/0) enable/disable data frame
     */
    g_net_monitor_param.filter_flags = (t_u16)atoi(argv[3]);

    /*
     * radio_type:
     * Band Info - (00)=2.4GHz, (01)=5GHz
     * t_u8  chanBand    : 2;
     * Channel Width - (00)=20MHz, (10)=40MHz, (11)=80MHz
     * t_u8  chanWidth   : 2;
     * Secondary Channel Offset - (00)=None, (01)=Above, (11)=Below
     * t_u8  chan2Offset : 2;
     * Channel Selection Mode - (00)=manual, (01)=ACS, (02)=Adoption mode
     * t_u8  scanMode    : 2;
     */
    g_net_monitor_param.radio_type  = (t_u8)atoi(argv[4]);
    g_net_monitor_param.chan_number = (t_u8)atoi(argv[5]);

    dump_monitor_param();
}

void set_monitor_filter(int op_index, t_u8 *mac)
{
    t_u8 temp_filter_num = g_net_monitor_param.filter_num;

    switch (op_index)
    {
    case MONITOR_FILTER_OPT_ADD_MAC:
        if (temp_filter_num < MAX_MONIT_MAC_FILTER_NUM)
        {
            (void)memcpy(&g_net_monitor_param.mac_addr[temp_filter_num], mac, MLAN_MAC_ADDR_LENGTH);
            g_net_monitor_param.filter_num++;
        }
        else
        {
            (void)PRINTF("Max filter num is 3 \r\n");
            return;
        }
        break;

    case MONITOR_FILTER_OPT_DELETE_MAC:
        if (temp_filter_num > 0)
        {
            memset(&g_net_monitor_param.mac_addr[temp_filter_num], 0, MLAN_MAC_ADDR_LENGTH);
            g_net_monitor_param.filter_num--;
        }
        else
        {
            (void)PRINTF("Monitor filter num is 0 \r\n");
            return;
        }
        break;

    case MONITOR_FILTER_OPT_CLEAR_MAC:
        memset(&g_net_monitor_param.mac_addr[0], 0, MAX_MONIT_MAC_FILTER_NUM * MLAN_MAC_ADDR_LENGTH);
        g_net_monitor_param.filter_num = 0;
        break;

    case MONITOR_FILTER_OPT_DUMP:
        dump_monitor_param();
        break;

    default:
        (void)PRINTF("unknown argument!\r\n");
        break;
    }
}

static void test_wlan_set_monitor_filter(int argc, char **argv)
{
    int ret = 0;
    t_u8 raw_mac[MLAN_MAC_ADDR_LENGTH];
    int op_index = 0;

    if (3 == argc)
    {
        if (string_equal("add", argv[1]))
        {
            ret = get_mac(argv[2], (char *)raw_mac, ':');
            if (ret != 0)
            {
                (void)PRINTF("Error: invalid MAC argument\r\n");
                return;
            }
            if ((memcmp(&raw_mac[0], broadcast_mac, MLAN_MAC_ADDR_LENGTH) == 0) || (raw_mac[0] & 0x01))
            {
                (void)PRINTF("Error: only support unicast mac\r\n");
                return;
            }
            op_index = MONITOR_FILTER_OPT_ADD_MAC;
        }
        else
        {
            dump_wlan_set_monitor_filter_usage();
            return;
        }
    }
    else if (2 == argc)
    {
        if (string_equal("delete", argv[1]))
            op_index = MONITOR_FILTER_OPT_DELETE_MAC;
        else if (string_equal("clear", argv[1]))
            op_index = MONITOR_FILTER_OPT_CLEAR_MAC;
        else if (string_equal("dump", argv[1]))
            op_index = MONITOR_FILTER_OPT_DUMP;
        else
        {
            (void)PRINTF("Unknown argument!\r\n\r\n");
            dump_wlan_set_monitor_filter_usage();
            return;
        }
    }
    else
    {
        dump_wlan_set_monitor_filter_usage();
        return;
    }

    set_monitor_filter(op_index, raw_mac);
}

/* Due to hardware issues, 9177 needs to scan the specified channel
 * that will be monitored before run wlan-net-monitor-cfg
 */
static void test_wlan_net_monitor_cfg(int argc, char **argv)
{
    int ret;

    ret = wlan_net_monitor_cfg(&g_net_monitor_param);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to send monitor cfg\r\n");
    }
}
#endif


#if CONFIG_CPU_LOADING
static void dump_wlan_cpu_loading_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-cpu-loading start <start> sample_loops <number> sample_period <period>\r\n");
    (void)PRINTF("    <start>: \r\n");
    (void)PRINTF("	        0: stop ongoing collecting cpu loading info.\r\n");
    (void)PRINTF("	        1: start collecting cpu loading info.\r\n");
    (void)PRINTF("    <number>: \r\n");
    (void)PRINTF("	        The cycle numbers to collect cpu loading info.\r\n");
    (void)PRINTF(
        "	        If no value is set, will keep collect cpu loading info until a stop command is received.\r\n");
    (void)PRINTF("    <period>: \r\n");
    (void)PRINTF("	        The period to collect cpu loading info.\r\n");
    (void)PRINTF("	        If no value is set, period is asigned as 2s.\r\n");
    (void)PRINTF("	        Note: The smaller the period, the greater the impact on performance.\r\n");
    (void)PRINTF("	   For example:\r\n");
    (void)PRINTF("	        wlan-cpu-loading start 0\r\n");
    (void)PRINTF("	            stop ongoing collecting cpu loading info.\r\n");
    (void)PRINTF("	        wlan-cpu-loading start 1 sample_loops 10\r\n");
    (void)PRINTF(
        "	            The cycle numbers of collecting cpu loading is 10 and collect period is default value 2s.\r\n");
    (void)PRINTF("	        wlan-cpu-loading start 1 sample_loops 10 sample_period 5\r\n");
    (void)PRINTF("	            The cycle numbers of collecting cpu loading is 10 and collect period is 5s.\r\n");
    (void)PRINTF("	        wlan-cpu-loading start 1\r\n");
    (void)PRINTF("	            Ongoing colloecting cpu loading info until execute 'wlan-cpu-loading 0'.\r\n");
    (void)PRINTF("	        wlan-cpu-loading start 1 sample_period 4\r\n");
    (void)PRINTF(
        "	            Ongoing colloecting cpu loading info until execute 'wlan-cpu-loading 0'. And the collect period "
        "is 4s.\r\n");
}

static void test_wlan_cpu_loading(int argc, char **argv)
{
    int arg = 0;
    unsigned int value;
    uint8_t start, period = 0;
    uint32_t number = 0;

    struct
    {
        uint8_t start : 1;
        uint8_t number : 1;
        uint8_t period : 1;
    } info;

    (void)memset(&info, 0, sizeof(info));

    if (argc < 2 && argc > 4)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_cpu_loading_usage();
        return;
    }

    arg++;
    do
    {
        if (!info.start && string_equal("start", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid <start> argument\r\n");
                dump_wlan_cpu_loading_usage();
                return;
            }
            if (value != 0 && value != 1)
            {
                (void)PRINTF("Error: invalid <start> argument\r\n");
                dump_wlan_cpu_loading_usage();
                return;
            }

            start = value & 0xFF;
            arg += 2;
            info.start = 1;
        }
        else if (!info.number && string_equal("sample_loops", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid <number> argument\r\n");
                dump_wlan_cpu_loading_usage();
                return;
            }

            number = value;
            arg += 2;
            info.number = 1;
        }
        else if (!info.period && string_equal("sample_period", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid <period> argument\r\n");
                dump_wlan_cpu_loading_usage();
                return;
            }

            period = value & 0xFF;
            arg += 2;
            info.period = 1;
        }
        else
        {
            (void)PRINTF("Error: argument %d is invalid\r\n", arg);
            dump_wlan_cpu_loading_usage();
            return;
        }
    }
    while (arg < argc);

    if (start == 1)
    {
        (void)PRINTF("Start cpu loading test:\r\n");
        if (number == 0)
            (void)PRINTF("%s\r\n", "Keeping CPU loading test");
        else
            (void)PRINTF("Cycle numbers of CPU loading test: %d\r\n", number);

        if (period == 0)
            (void)PRINTF("Period of CPU loading test: 2s\r\n");
        else
            (void)PRINTF("Period of CPU loading test: %d s\r\n", period);
    }

    wlan_cpu_loading(start, number, period);
}
#endif

#if CONFIG_TSP
static void dump_wlan_tsp_cfg_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "    wlan-set-tsp-cfg en <0/1> bo <0-20> high <0-300> low <0-300> "
        "dcstep <1-100> dcmin <0-100> hightemp <-100-150> lowtemp <-100-150>\r\n");
    (void)PRINTF("	  <en>: 0 -- disable   1 -- enable\r\n");
    (void)PRINTF("	  <bo>: power backoff [0...20]\r\n");
    (void)PRINTF("	  <high>: High power Threshold [0...300]\r\n");
    (void)PRINTF("	  <low>: Low power Threshold [0...300]\r\n");
    (void)PRINTF("	  <dcstep>: Duty Cycle setp [1...100]\r\n");
    (void)PRINTF("	  <dcmin>: Duty Cycle min [0...100]\r\n");
    (void)PRINTF("	  <hightemp>: High Throttle Threshold temperature [-100...150]\r\n");
    (void)PRINTF("	  <lowtemp>: Low Throttle Threshold temperature [-100...150]\r\n");
    (void)PRINTF("	  High Threshold must be greater than Low Threshold\r\n");
    (void)PRINTF("	  High Throttle Threshold temperature must be greater than Low Throttle Threshold temperature.\r\n");
    (void)PRINTF("	  If you want to get tsp cfg, you can just use wlan-get-tsp-cfg.\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF(
        "wlan-set-tsp-cfg wlan-set-tsp-cfg en 1 bo 0 high 93 low 83 dcstep 5 dcmin 10 hightemp 120 lowtemp 110 \r\n");
    (void)PRINTF("wlan-set-tsp-cfg en 0 \r\n");
}
static void test_wlan_set_tsp_cfg(int argc, char **argv)
{
    int arg = 0;
    unsigned int value;
    int tempvalue;
    t_u16 enable        = 0;
    t_u32 back_off      = 0;
    t_u32 highThreshold = 0;
    t_u32 lowThreshold  = 0;
    t_u32 dutycycstep   = 0;
    t_u32 dutycycmin    = 0;
    int highthrtemp     = 0;
    int lowthrtemp      = 0;
    int ret             = WM_SUCCESS;

    struct
    {
        unsigned enable : 1;
        unsigned backoff : 1;
        unsigned high : 1;
        unsigned low : 1;
        unsigned dutycycstep : 1;
        unsigned dutycycmin : 1;
        unsigned highthrtemp : 1;
        unsigned lowthrtemp : 1;
    } info;

    (void)memset(&info, 0, sizeof(info));

    if (argc < 3 || argc > 17)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_tsp_cfg_usage();
        return;
    }

    arg++;
    do
    {
        if (!info.enable && string_equal("en", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])) || (value != 0 && value != 1))
            {
                (void)PRINTF("Error: invalid enable argument\r\n");
                dump_wlan_tsp_cfg_usage();
                return;
            }
            arg += 2;
            info.enable = 1;
            enable      = value & 0xFF;
        }
        else if (!info.backoff && string_equal("bo", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])) || value > 20)
            {
                (void)PRINTF("Error: invalid backoff argument\r\n");
                dump_wlan_tsp_cfg_usage();
                return;
            }
            arg += 2;
            info.backoff = 1;
            back_off     = value;
        }
        else if (!info.high && string_equal("high", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])) || value > 300)
            {
                (void)PRINTF("Error: invalid high threshold argument\r\n");
                dump_wlan_tsp_cfg_usage();
                return;
            }
            arg += 2;
            info.high     = 1;
            highThreshold = value;
        }
        else if (!info.low && string_equal("low", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])) || value > 300)
            {
                (void)PRINTF("Error: invalid low threshold argument\r\n");
                dump_wlan_tsp_cfg_usage();
                return;
            }
            arg += 2;
            info.low     = 1;
            lowThreshold = value;
        }
        else if (!info.dutycycstep && string_equal("dcstep", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])) || value > 100)
            {
                (void)PRINTF("Error: invalid dutycycstep argument\r\n");
                dump_wlan_tsp_cfg_usage();
                return;
            }
            arg += 2;
            info.dutycycstep = 1;
            dutycycstep      = value;
        }
        else if (!info.dutycycmin && string_equal("dcmin", argv[arg]))
        {
            if (get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])) || value > 100)
            {
                (void)PRINTF("Error: invalid dutycycmin argument\r\n");
                dump_wlan_tsp_cfg_usage();
                return;
            }
            arg += 2;
            info.dutycycmin = 1;
            dutycycmin      = value;
        }
        else if (!info.highthrtemp && string_equal("hightemp", argv[arg]))
        {
            tempvalue = (int)atoi(argv[arg + 1]);
            if (tempvalue < -100 || tempvalue > 150)
            {
                (void)PRINTF("Error: invalid high throttle temperature threshold argument\r\n");
                dump_wlan_tsp_cfg_usage();
                return;
            }
            arg += 2;
            info.highthrtemp = 1;
            highthrtemp      = tempvalue;
        }
        else if (!info.lowthrtemp && string_equal("lowtemp", argv[arg]))
        {
            tempvalue = (int)atoi(argv[arg + 1]);
            if (tempvalue < -100 || tempvalue > 150)
            {
                (void)PRINTF("Error: invalid low throttle temperature threshold argument\r\n");
                dump_wlan_tsp_cfg_usage();
                return;
            }
            arg += 2;
            info.lowthrtemp = 1;
            lowthrtemp      = tempvalue;
        }
        else
        {
            (void)PRINTF("Error: invalid [%d] argument\r\n", arg + 1);
            dump_wlan_tsp_cfg_usage();
            return;
        }
    }
    while (arg < argc);

    if (enable && ((highThreshold <= lowThreshold) || (highthrtemp <= lowthrtemp)))
    {
        (void)PRINTF("Error: High Threshold must be greater than Low Threshold\r\n");
        dump_wlan_tsp_cfg_usage();
        return;
    }

    ret = wlan_set_tsp_cfg(enable, back_off, highThreshold, lowThreshold, dutycycstep, dutycycmin, highthrtemp,
                           lowthrtemp);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set TSP config\r\n");
        return;
    }
}

static void test_wlan_get_tsp_cfg(int argc, char **argv)
{
    t_u16 enable        = 0;
    t_u32 back_off      = 0;
    t_u32 highThreshold = 0;
    t_u32 lowThreshold  = 0;
    t_u32 dutycycstep   = 0;
    t_u32 dutycycmin    = 0;
    int highthrtemp     = 0;
    int lowthrtemp      = 0;
    int currCAUTemp     = 0;
    int currRFUTemp     = 0;

    int ret = WM_SUCCESS;

    if (argc != 1)
    {
        dump_wlan_tsp_cfg_usage();
        return;
    }

    ret = wlan_get_tsp_cfg(&enable, &back_off, &highThreshold, &lowThreshold, &dutycycstep, &dutycycmin, &highthrtemp,
                           &lowthrtemp, &currCAUTemp, &currRFUTemp);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get TSP config\r\n");
        return;
    }

    (void)PRINTF("TSP Configuration:\r\n");
    (void)PRINTF("	Enable TSP Algorithm: %d\r\n", enable);
    (void)PRINTF("		0: disable 1: enable\r\n");
    (void)PRINTF("	Power Management Backoff: %d dB\r\n", back_off);
    (void)PRINTF("	Low Power BOT  Threshold(celcius): %d\r\n", lowThreshold);
    (void)PRINTF("	High Power BOT Threshold(celcius): %d\r\n", highThreshold);
    (void)PRINTF("	Duty Cycle setp(percentage): %d\r\n", dutycycstep);
    (void)PRINTF("	Duty Cycle min (percentage): %d\r\n", dutycycmin);
    (void)PRINTF("	High Throttle Threshold temperature(celcius): %d\r\n", highthrtemp);
    (void)PRINTF("	Low Throttle  Threshold temperature(celcius): %d\r\n", lowthrtemp);
    (void)PRINTF("	CAU TSEN Temperature(celcius): %d\r\n", currCAUTemp);
    (void)PRINTF("	RFU      Temperature(celcius): %d\r\n", currRFUTemp);
}
#endif

#if CONFIG_CLOUD_KEEP_ALIVE

#include "lwip/priv/tcp_priv.h"
#include "lwip/stats.h"
#include "lwip/inet.h"
#include "lwip/inet_chksum.h"

#define DATA_LEN 1460

static void dump_wlan_tcp_client_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-tcp-client dst_ip <dst_ip> src_port <src_port> dst_port <dst_port>\r\n");
    (void)PRINTF("        <dst_ip> Destination IP\r\n");
    (void)PRINTF("        <src_port> Source port\r\n");
    (void)PRINTF("        <dst_port> Destination port\r\n");
    (void)PRINTF("        Please specify dst_ip src_port and dst_port\r\n");
    (void)PRINTF("    wlan-tcp-client dst_ip 192.168.1.50 src_port 54236 dst_port 9526\r\n");
}

static err_t wlan_tcp_client_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
    char clientString[] = "hello\r\n";
    t_u8 count          = 5;
    char *clientData    = NULL;
#if !CONFIG_MEM_POOLS
    clientData = OSA_MemoryAllocate(DATA_LEN);
#else
    clientData = OSA_MemoryPoolAllocate(buf_1536_MemoryPool);
#endif
    if (clientData == NULL)
    {
        (void)PRINTF("Error: failed to alloc memory!\r\n");
        return ERR_MEM;
    }
    (void)memset((t_u8 *)clientData, 0, DATA_LEN);
    (void)memcpy(clientData, clientString, sizeof(clientString));

    while (count--)
    {
        tcp_write(pcb, clientData, DATA_LEN, TCP_WRITE_FLAG_COPY);
        tcp_output(pcb);
    }
#if !CONFIG_MEM_POOLS
    OSA_MemoryFree(clientData);
#else
    OSA_MemoryPoolFree(buf_1536_MemoryPool, clientData);
#endif
    return ERR_OK;
}

static void test_wlan_tcp_client(int argc, char **argv)
{
    struct tcp_pcb *tcp_client_pcb;
    ip_addr_t ipaddr;
    int arg           = 0;
    int dst_ip_set    = 0;
    int src_port_set  = 0;
    int dst_port_set  = 0;
    t_u32 dst_ip      = 0;
    t_u16 src_port    = 0;
    t_u16 dst_port    = 0;
    unsigned int port = 0;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_tcp_client_usage();
        return;
    }

    if (!is_sta_connected())
    {
        (void)PRINTF("Can not start wlan_tcp_client in disconnected state\r\n");
        return;
    }

    arg += 1;
    do
    {
        if (string_equal("dst_ip", argv[arg]))
        {
            dst_ip     = net_inet_aton(argv[arg + 1]);
            dst_ip_set = 1;
            arg += 2;
        }
        else if (string_equal("src_port", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], (unsigned int *)&port, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid src_port argument\r\n");
                return;
            }
            src_port     = (uint16_t)(port & 0XFFFF);
            src_port_set = 1;
            arg += 2;
        }
        else if (string_equal("dst_port", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], (unsigned int *)&port, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Error: invalid dst_port argument\r\n");
                return;
            }
            dst_port     = (uint16_t)(port & 0XFFFF);
            dst_port_set = 1;
            arg += 2;
        }
        else
        {
            (void)PRINTF("Error: argument %d is invalid\r\n", arg);
            dump_wlan_tcp_client_usage();
            return;
        }
    }
    while (arg < argc);

    if (!dst_ip_set || !src_port_set || !dst_port_set)
    {
        dump_wlan_tcp_client_usage();
        (void)PRINTF("Error: please specify dst_ip src_port and dst_port\r\n");
        return;
    }

    if (!wlan_cloud_keep_alive_enabled(dst_ip, dst_port))
    {
        (void)PRINTF("Cloud keep alive not started for given destination ip and port\r\n");
        return;
    }

    (void)memcpy(&ipaddr.u_addr.ip4, &dst_ip, sizeof(ip4_addr_t));
#if CONFIG_IPV6
    ipaddr.type = IPADDR_TYPE_V4;
#endif

    tcp_client_pcb = tcp_new();
    tcp_bind(tcp_client_pcb, IP_ADDR_ANY, src_port);

    if (tcp_client_pcb != NULL)
    {
        tcp_connect(tcp_client_pcb, &ipaddr, dst_port, wlan_tcp_client_connected);
    }
}

static void dump_wlan_cloud_keep_alive_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-cloud-keep-alive start id <id> dst_mac <dst_mac> dst_ip <dst_ip> dst_port <dst_port>\r\n");
    (void)PRINTF("        <id> Keep alive id from 0 to 3\r\n");
    (void)PRINTF("        <dst_mac> Destination MAC address\r\n");
    (void)PRINTF("        <dst_ip> Destination IP\r\n");
    (void)PRINTF("        <dst_port> Destination port\r\n");
    (void)PRINTF("        Please specify dst_mac, dst_ip and dst_port\r\n");
    (void)PRINTF("    wlan-cloud-keep-alive stop\r\n");
    (void)PRINTF("    wlan-cloud-keep-alive reset\r\n");
}

static void test_wlan_cloud_keep_alive(int argc, char **argv)
{
    int ret          = -WM_FAIL;
    int arg          = 1;
    int id_set       = 0;
    int dst_mac_set  = 0;
    int dst_ip_set   = 0;
    int dst_port_set = 0;

    wlan_cloud_keep_alive_t cloud_keep_alive;

    /* Period to send keep alive packet, set the default value to 55s(The unit is milliseconds) */
    t_u32 send_interval_default = 55000;

    /* Period to send retry packet, set the default value to 20s(The unit is milliseconds) */
    t_u16 retry_interval_default = 20000;

    /* Count to send retry packet, set the default value to 3 */
    t_u16 retry_count_default = 3;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_cloud_keep_alive_usage();
        return;
    }

    memset(&cloud_keep_alive, 0x00, sizeof(wlan_cloud_keep_alive_t));

    if (string_equal("start", argv[1]))
    {
        /* Period to send keep alive packet, set the default value to 55s(The unit is milliseconds) */
        cloud_keep_alive.send_interval = send_interval_default;
        /* Period to send retry packet, set the default value to 20s(The unit is milliseconds) */
        cloud_keep_alive.retry_interval = retry_interval_default;
        /* Count to send retry packet, set the default value to 3 */
        cloud_keep_alive.retry_count = retry_count_default;

        arg += 1;
        do
        {
            if (string_equal("id", argv[arg]))
            {
                errno                           = 0;
                cloud_keep_alive.mkeep_alive_id = strtol(argv[arg + 1], NULL, 10);
                if (errno != 0)
                {
                    (void)PRINTF("Error during strtol:id errno:%d\r\n", errno);
                    return;
                }

                id_set = 1;
                arg += 2;
            }
            else if (string_equal("dst_mac", argv[arg]))
            {
                ret = get_mac(argv[arg + 1], (char *)&cloud_keep_alive.dst_mac, ':');
                if (ret != 0)
                {
                    (void)PRINTF("Error: invalid dst_mac argument\r\n");
                    return;
                }
                dst_mac_set = 1;
                arg += 2;
            }
            else if (string_equal("dst_ip", argv[arg]))
            {
                cloud_keep_alive.dst_ip = net_inet_aton(argv[arg + 1]);
                dst_ip_set              = 1;
                arg += 2;
            }
            else if (string_equal("dst_port", argv[arg]))
            {
                unsigned int dst_port;

                if (arg + 1 >= argc || get_uint(argv[arg + 1], (unsigned int *)&dst_port, strlen(argv[arg + 1])))
                {
                    (void)PRINTF("Error: invalid dst_port argument\r\n");
                    return;
                }
                cloud_keep_alive.dst_port = (uint16_t)(dst_port & 0XFFFF);
                dst_port_set              = 1;
                arg += 2;
            }
            else
            {
                (void)PRINTF("Error: argument %d is invalid\r\n", arg);
                dump_wlan_cloud_keep_alive_usage();
                return;
            }
        }
        while (arg < argc);
        if (!id_set || !dst_mac_set || !dst_ip_set || !dst_port_set)
        {
            dump_wlan_cloud_keep_alive_usage();
            (void)PRINTF("Error: please specify dst_mac, dst_ip and dst_port\r\n");
            return;
        }

        cloud_keep_alive.enable = MTRUE;
    }
    else if (string_equal("stop", argv[1]))
    {
        cloud_keep_alive.enable = MFALSE;
        cloud_keep_alive.reset  = MFALSE;
    }
    else if (string_equal("reset", argv[1]))
    {
        cloud_keep_alive.enable = MFALSE;
        cloud_keep_alive.reset  = MTRUE;
    }
    else
    {
        (void)PRINTF("Error: invalid [%d] argument, give start/stop/reset\r\n", arg + 1);
        dump_wlan_cloud_keep_alive_usage();
        return;
    }
    if (cloud_keep_alive.enable == MTRUE)
    {
        ret = wlan_save_cloud_keep_alive_params(&cloud_keep_alive, 0, 0, 0, 0, MTRUE);
    }
    else
    {
        ret = wlan_stop_cloud_keep_alive(&cloud_keep_alive);
    }
}
#endif

static void test_wlan_get_signal(int argc, char **argv)
{
    wlan_rssi_info_t signal;
    int ret = WM_SUCCESS;

    if (!is_sta_connected())
    {
        (void)PRINTF("Can not get RSSI information in disconnected state\r\n");
        return;
    }

    (void)memset(&signal, 0, sizeof(wlan_rssi_info_t));

    ret = wlan_get_signal_info(&signal);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get RSSI information\r\n");
        return;
    }
    (void)PRINTF("\tBeaconLast\tBeacon Average\tData Last\tData Average\r\n");
    (void)PRINTF("RSSI\t%-10d \t%-10d \t%-10d \t%-10d\r\n", (int)signal.bcn_rssi_last, (int)signal.bcn_rssi_avg,
                 (int)signal.data_rssi_last, (int)signal.data_rssi_avg);
    (void)PRINTF("SNR \t%-10d \t%-10d \t%-10d \t%-10d\r\n", (int)signal.bcn_snr_last, (int)signal.bcn_snr_avg,
                 (int)signal.data_snr_last, (int)signal.data_snr_avg);
    (void)PRINTF("NF  \t%-10d \t%-10d \t%-10d \t%-10d\r\n", (int)signal.bcn_nf_last, (int)signal.bcn_nf_avg,
                 (int)signal.data_nf_last, (int)signal.data_nf_avg);
}

static void dump_wlan_bandcfg_bit_usage(void)
{
    (void)PRINTF("        Bits in Band:\r\n");
    (void)PRINTF("        bit 0: B (Not support set)\r\n");
    (void)PRINTF("        bit 1: G (Not support set)\r\n");
    (void)PRINTF("        bit 2: A (Not support set)\r\n");
    (void)PRINTF("        bit 3: GN (Not support set)\r\n");
    (void)PRINTF("        bit 4: AN (Not support set)\r\n");
#if CONFIG_11AC
    (void)PRINTF("        bit 5: AC 2.4G (Not support set)\r\n");
    (void)PRINTF("        bit 6: AC 5G (Not support set)\r\n");
#endif
#if CONFIG_11AX
    (void)PRINTF("        bit 8: AX 2.4G\r\n");
    (void)PRINTF("        bit 9: AX 5G\r\n");
#endif
}

static void test_wlan_get_bandcfg(int argc, char **argv)
{
    wlan_bandcfg_t bandcfg;
    int ret = WM_SUCCESS;

    (void)memset(&bandcfg, 0, sizeof(bandcfg));

    ret = wlan_get_bandcfg(&bandcfg);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get bandcfg\r\n");
        return;
    }
    (void)PRINTF("\tconfig band: 0x%x\r\n", bandcfg.config_bands);
    (void)PRINTF("\tfw band: 0x%x\r\n", bandcfg.fw_bands);
    dump_wlan_bandcfg_bit_usage();
}

#if CONFIG_11AX
static void dump_wlan_set_bandcfg(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-set-bandcfg <value>\r\n");
    dump_wlan_bandcfg_bit_usage();
}
#endif

static void test_wlan_set_bandcfg(int argc, char **argv)
{
#if CONFIG_11AX
    wlan_bandcfg_t bandcfg;
    uint32_t bandcfg_11ax_2G = 0;
    uint32_t bandcfg_11ax_5G = 0;
    uint32_t val             = 0;
    int ret                  = WM_SUCCESS;
#endif

#if !CONFIG_11AX
    (void)PRINTF("Block set bandcfg when 11AX is not supported.\r\n");
    return;
#else

    if (argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_bandcfg();
        return;
    }

    val = a2hex_or_atoi(argv[1]);

    bandcfg_11ax_2G = (val & MBIT(8));
    bandcfg_11ax_5G = (val & MBIT(9));

    if ((bandcfg_11ax_2G && !bandcfg_11ax_5G) || (!bandcfg_11ax_2G && bandcfg_11ax_5G))
    {
        (void)PRINTF("Please set 11ax 2G/5G bit both 0 or both 1.\r\n");
        dump_wlan_set_bandcfg();
        return;
    }

    (void)memset(&bandcfg, 0, sizeof(bandcfg));
    ret = wlan_get_bandcfg(&bandcfg);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get bandcfg\r\n");
        return;
    }

    if (bandcfg_11ax_2G)
    {
        bandcfg.config_bands |= (MBIT(8));
    }
    else
    {
        bandcfg.config_bands &= ~(MBIT(8));
    }

    if (bandcfg_11ax_5G)
    {
        bandcfg.config_bands |= (MBIT(9));
    }
    else
    {
        bandcfg.config_bands &= ~(MBIT(9));
    }

    ret = wlan_set_bandcfg(&bandcfg);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set bandcfg\r\n");
        return;
    }
#endif
}

static void dump_wlan_set_multiple_dtim_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    This command is to set Next Wakeup RX Beacon Time\r\n");
    (void)PRINTF("    Will take effect after enter power save mode by command wlan-ieee-ps 1\r\n");
    (void)PRINTF("    Next Wakeup RX Beacon Time = DTIM * BeaconPeriod * multiple_dtim\r\n");

    (void)PRINTF("    wlan-set-multiple-dtim <value>\r\n");
    (void)PRINTF("        <value> Value of multiple dtim, range[1,20]\r\n");
}

static void test_wlan_set_multiple_dtim(int argc, char **argv)
{
    uint8_t multiple_dtim = 0;

    if (argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_multiple_dtim_usage();
        return;
    }

    multiple_dtim = (t_u8)atoi(argv[1]);

    if (multiple_dtim < 1 || multiple_dtim > 20)
    {
        (void)PRINTF("Error: value of multiple dtim is out of range\r\n");
        dump_wlan_set_multiple_dtim_usage();
        return;
    }

    wlan_set_ps_cfg(multiple_dtim, 5, 0, 0, PS_MODE_AUTO, DELAY_TO_PS_DEFAULT);
    (void)PRINTF("Set multiple dtim to %d\r\n", multiple_dtim);
}

static void dump_wlan_set_su_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-set-su <0/1>\r\n");
    (void)PRINTF("    <start/stop>: 1 -- stop su\r\n");
    (void)PRINTF("                  0 -- start su\r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("    wlan-set-su\r\n");
    (void)PRINTF("    - Get current su state.\r\n");
    (void)PRINTF("    wlan-set-su 1\r\n");
    (void)PRINTF("    - stop su\r\n");
}

static void test_wlan_set_su(int argc, char **argv)
{
    int ret           = -WM_FAIL;
    uint32_t reqd_len = 0;
    uint8_t state;
    uint8_t debug_resp_buf[64] = {0};

    (void)memset(debug_resp_buf, 0, sizeof(debug_resp_buf));
    /**
     * Command taken from debug.conf
     *  start_su={
     *        CmdCode=0x008b
     *        Action:2=1
     *        SUBID:2=0x101
     *        Value:4=1           # 1 -- stop_su;
     *                            # 0 -- start_su;
     */
    uint8_t debug_cmd_buf[] = {0x8b, 0, 0x10, 0, 0, 0, 0, 0, 0x01, 0, 0x01, 0x01, 0x01, 0, 0, 0};

    if (argc > 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_su_usage();
        return;
    }

    /* SET */
    if (argc == 2)
    {
        state             = atoi(argv[1]);
        debug_cmd_buf[12] = state;
    }
    else /* GET */
    {
        debug_cmd_buf[8] = 0;
    }

    ret = wlan_send_hostcmd(debug_cmd_buf, sizeof(debug_cmd_buf) / sizeof(uint8_t), debug_resp_buf,
                            sizeof(debug_resp_buf), &reqd_len);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Hostcmd success, response is\r\n");
        for (ret = 0; ret < reqd_len; ret++)
            (void)PRINTF("%x\t", debug_resp_buf[ret]);
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}

#if CONFIG_WIFI_FORCE_RTS
#define HOSTCMD_RESP_BUFF_SIZE 1024
uint8_t debug_resp_buf[HOSTCMD_RESP_BUFF_SIZE] = {0};

static void dump_wlan_set_forceRTS_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-set-forceRTS <0/1>\r\n");
    (void)PRINTF("    <start/stop>: 1 -- start forceRTS\r\n");
    (void)PRINTF("                  0 -- stop forceRTS\r\n");
    (void)PRINTF("Example:\r\n");
    (void)PRINTF("    wlan-set-forceRTS\r\n");
    (void)PRINTF("    - Get current forceRTS state.\r\n");
    (void)PRINTF("    wlan-set-forceRTS 1\r\n");
    (void)PRINTF("    - Set start forceRTS\r\n");
}

/* Bypass wmmTurboMode TxopLimit setting if for certificate is true, for BE traffic only. (Case: HE 5.71.1) */
static void test_wlan_set_forceRTS(int argc, char **argv)
{
    int ret           = -WM_FAIL;
    uint32_t reqd_len = 0;
    uint8_t state;
    /**
     * Command taken from debug.conf
     * start_forceRTS={
     *      CmdCode=0x008b
     *      Action:2=1
     *      SUBID:2=0x104
     *      Value:1=1           # 1 -- start forceRTS;
     *                          # 0 -- stop forceRTS;
     */
    uint8_t debug_cmd_buf[] = {0x8b, 0, 0x0d, 0, 0, 0, 0, 0, 0x01, 0, 0x04, 0x01, 0x01};

    if (argc > 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_forceRTS_usage();
        return;
    }

    /* SET */
    if (argc == 2)
    {
        state             = atoi(argv[1]);
        debug_cmd_buf[12] = state;
    }
    else /* GET */
    {
        dump_wlan_set_forceRTS_usage();
        debug_cmd_buf[8] = 0;
    }

    ret = wlan_send_hostcmd(debug_cmd_buf, sizeof(debug_cmd_buf) / sizeof(uint8_t), debug_resp_buf,
                            HOSTCMD_RESP_BUFF_SIZE, &reqd_len);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Hostcmd success, response is\r\n");
        for (ret = 0; ret < reqd_len; ret++)
            (void)PRINTF("%x\t", debug_resp_buf[ret]);
    }
    else
    {
        (void)PRINTF("Hostcmd failed error: %d", ret);
    }
}
#endif

#if (CONFIG_IPS)
static void dump_wlan_set_ips_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-ips option\r\n");
    (void)PRINTF("option:\r\n");
    (void)PRINTF("0: disable ips enhance\r\n");
    (void)PRINTF("1: enable ips enhance\r\n");
}
static void test_wlan_set_ips(int argc, char **argv)
{
    unsigned int option;

    if (argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_ips_usage();
        return;
    }

    if (get_uint(argv[1], &option, strlen(argv[1])))
    {
        (void)PRINTF("Error: invalid option argument\r\n");
        dump_wlan_set_ips_usage();
        return;
    }

    if (option != 0 && option != 1)
    {
        (void)PRINTF("Error: invalid option argument\r\n");
        dump_wlan_set_ips_usage();
        return;
    }

    wlan_set_ips(option);
}
#endif

#if (CONFIG_WPA_SUPP_WPS) || (CONFIG_WPS2)
static void test_wlan_start_wps_pbc(int argc, char **argv)
{
    int ret;

    if (argc != 1)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    ret = wlan_start_wps_pbc();

    if (ret == -WM_FAIL)
    {
        PRINTF("Start WPS PBC failed\r\n");
    }
    else if (ret == -2)
    {
        PRINTF("FAIL-PBC-OVERLAP\r\n");
    }
    else
    {
        PRINTF("Started WPS PBC session\r\n");
    }
}

static void test_wlan_start_wps_pin(int argc, char **argv)
{
    int ret = -WM_FAIL;

    if (argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

#if (CONFIG_WPA_SUPP_WPS)
    ret = wlan_start_wps_pin(argv[1]);
#else
    ret = wlan_start_wps_pin((uint32_t)atoi(argv[1]));
#endif

    if (ret != WM_SUCCESS)
    {
        PRINTF("Invalid PIN entered\r\n");
    }
    else
    {
        PRINTF("Started WPS PIN session with pin as: %s\r\n", argv[1]);
    }
}

static void test_wlan_wps_generate_pin(int argc, char **argv)
{
    uint32_t pin = 0;

    if (argc != 1)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    wlan_wps_generate_pin(&pin);
    PRINTF("WPS PIN is: %08d\r\n", pin);
}
#endif
#if CONFIG_WPA_SUPP_WPS
static void test_wlan_wps_cancel(int argc, char **argv)
{
    int ret;

    if (argc != 1)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    ret = wlan_wps_cancel();

    if (ret != WM_SUCCESS)
    {
        PRINTF("Cancel WPS failed\r\n");
    }
}
#if CONFIG_WPA_SUPP_AP
static void test_wlan_start_ap_wps_pbc(int argc, char **argv)
{
    int ret;

    if (argc != 1)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    ret = wlan_start_ap_wps_pbc();

    if (ret != WM_SUCCESS)
    {
        PRINTF("Start AP WPS PBC failed\r\n");
    }
}

static void test_wlan_start_ap_wps_pin(int argc, char **argv)
{
    int ret = -WM_FAIL;

    if (argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    ret = wlan_start_ap_wps_pin(argv[1]);

    if (ret != WM_SUCCESS)
    {
        PRINTF("Start AP WPS PIN failed\r\n");
    }
    else
    {
        PRINTF("Started AP WPS PIN session with pin as: %s\r\n", argv[1]);
    }
}

static void test_wlan_wps_ap_cancel(int argc, char **argv)
{
    int ret;

    if (argc != 1)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    ret = wlan_wps_ap_cancel();

    if (ret != WM_SUCCESS)
    {
        PRINTF("Cancel WPS failed\r\n");
    }
}
#endif
#endif

#if CONFIG_TURBO_MODE
static void test_wlan_get_turbo_mode(int argc, char **argv)
{
    int ret = -WM_FAIL;
    uint8_t mode;
    int bss_type = MLAN_BSS_TYPE_ANY;
    if (argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        (void)PRINTF("Usage: wlan-get-turbo-mode STA/UAP\r\n");
        return;
    }

    if (string_equal("STA", argv[1]))
    {
        bss_type = MLAN_BSS_TYPE_STA;
        ret      = wlan_get_turbo_mode(&mode);
    }
    else if (string_equal("UAP", argv[1]))
    {
        bss_type = MLAN_BSS_TYPE_UAP;
        ret      = wlan_get_uap_turbo_mode(&mode);
    }
    else
    {
        (void)PRINTF("Error: invalid BSS type\r\n");
        (void)PRINTF("Usage: wlan-get-turbo-mode STA/UAP\r\n");
        return;
    }

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("%s Turbo mode: %d\r\n", bss_type == MLAN_BSS_TYPE_STA ? "STA" : "UAP", mode);
    }
    else
    {
        (void)PRINTF("Failed to get %s turbo mode\r\n", bss_type == MLAN_BSS_TYPE_STA ? "STA" : "UAP");
    }

    return;
}

static void dump_wlan_set_turbo_mode_usage()
{
    (void)PRINTF("Usage: wlan-set-turbo-mode <STA/UAP> <mode>\r\n");
    (void)PRINTF("          <STA/UAP>  'STA'  or 'UAP' \r\n");
    (void)PRINTF("          <mode> can be 0,1,2,3\r\n");
}

static void test_wlan_set_turbo_mode(int argc, char **argv)
{
    int ret = -WM_FAIL;
    unsigned int value;
    uint8_t mode;
    int bss_type;
    if (argc != 3)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_turbo_mode_usage();
        return;
    }

    if (string_equal("STA", argv[1]))
        bss_type = MLAN_BSS_TYPE_STA;
    else if (string_equal("UAP", argv[1]))
        bss_type = MLAN_BSS_TYPE_UAP;
    else
    {
        (void)PRINTF("Error: invalid BSS type\r\n");
        dump_wlan_set_turbo_mode_usage();
        return;
    }

    if (get_uint(argv[2], &value, strlen(argv[2])) && value > 3)
    {
        (void)PRINTF("Invalid mode argument\r\n");
        dump_wlan_set_turbo_mode_usage();
        return;
    }
    mode = value & 0xFF;

    if (bss_type == MLAN_BSS_TYPE_STA)
    {
        ret = wlan_set_turbo_mode(mode);
    }
    else
    {
        ret = wlan_set_uap_turbo_mode(mode);
    }

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Set %s turbo mode to %d\r\n", bss_type == MLAN_BSS_TYPE_STA ? "STA" : "UAP", mode);
    }
    else
    {
        (void)PRINTF("Failed to set %s turbo mode\r\n", bss_type == MLAN_BSS_TYPE_STA ? "STA" : "UAP");
    }

    return;
}
#endif

#if CONFIG_11AX

static void dump_wlan_set_debug_htc_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-set-debug-htc \r\n");
    (void)PRINTF("    <count>:1\r\n");
    (void)PRINTF("    <vht:1>\r\n");
    (void)PRINTF("    <he:1>\r\n");
    (void)PRINTF("    <rxNss:1>\r\n");
    (void)PRINTF("    <channelWidth:1>\r\n");
    (void)PRINTF("    <ulMuDisable:1>\r\n");
    (void)PRINTF("    <txNSTS:1>\r\n");
    (void)PRINTF("    <erSuDisable:1>\r\n");
    (void)PRINTF("    <dlResoundRecomm:1>\r\n");
    (void)PRINTF("    <ulMuDataDisable:1>\r\n");
}

static void test_wlan_set_debug_htc(int argc, char **argv)
{
    int ret = -WM_FAIL;
    uint8_t count, vht, he, rxNss, channelWidth, ulMuDisable, txNSTS, erSuDisable, dlResoundRecomm, ulMuDataDisable;
    /**
     * Command taken from debug.conf
     *  send_om_set={
     *  CmdCode=0x008b          # do NOT change this line
     *  Action:2=1              # 1 - HE-TB-PPDU with dummy UPH
     *  SUBID:2=0x111           # Send NULL
     *  count:1=0x40            # Count of packets with OM in HE-TB-PPDU format
     *  vht:1=1                 # HT Control Field: For HT Variant-0, VHT variant-1, HE Variant-1
     *  he:1=1                  # HT Control Field: For VHT Variant-0, HE variant-1
     *  rxNss:1=0
     *  channelWidth:1=0
     *  ulMuDisable:1=0
     *  txNSTS:1=0
     *  erSuDisable:1=0
     *  dlResoundRecomm:1=0
     *  ulMuDataDisable:1=0
     *     }
     *
     */

    if (argc != 11)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_debug_htc_usage();
        return;
    }

    count           = atoi(argv[1]);
    vht             = atoi(argv[2]);
    he              = atoi(argv[3]);
    rxNss           = atoi(argv[4]);
    channelWidth    = atoi(argv[5]);
    ulMuDisable     = atoi(argv[6]);
    txNSTS          = atoi(argv[7]);
    erSuDisable     = atoi(argv[8]);
    dlResoundRecomm = atoi(argv[9]);
    ulMuDataDisable = atoi(argv[10]);

    ret = wlan_send_debug_htc(count, vht, he, rxNss, channelWidth, ulMuDisable, txNSTS, erSuDisable, dlResoundRecomm,
                              ulMuDataDisable);

    if (ret == WM_SUCCESS)
        (void)PRINTF("HTC parameter set successfully\r\n");
    else
        (void)PRINTF("Failed to set HTC parameter\r\n");
}

static void dump_wlan_enable_disable_htc_usage()
{
    (void)PRINTF("Usage: wlan-enable-disable-htc <option>\r\n");
    (void)PRINTF("          <option>  0 => disable \r\n");
    (void)PRINTF("                    1 => enable \r\n");
}

static void test_wlan_enable_disable_htc(int argc, char **argv)
{
    int ret = -WM_FAIL;
    unsigned int option;

    if (argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_enable_disable_htc_usage();
        return;
    }
    if (get_uint(argv[1], &option, strlen(argv[1])) && option > 1)
    {
        (void)PRINTF("Invalid option argument\r\n");
        dump_wlan_enable_disable_htc_usage();
        return;
    }

    ret = wlan_enable_disable_htc((uint8_t)option);

    if (ret == WM_SUCCESS)
    {
        if (option)
            (void)PRINTF("HTC enabled\r\n");
        else
            (void)PRINTF("HTC disabled\r\n");
    }
    else
    {
        (void)PRINTF("Failed to enable/disable HTC\r\n");
    }
}
#endif

static void dump_wlan_11d_enable_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-11d-enable <sta/uap> <0/1>, 0: disable, 1: enable\r\n");
    (void)PRINTF("This command is only used to enable/disable 11D\r\n");
    (void)PRINTF("Please use wlan-set-regioncode command to set region\r\n");
}

static void test_wlan_11d_enable(int argc, char **argv)
{
    int state;

    if (argc != 3)
    {
        dump_wlan_11d_enable_usage();
        return;
    }

    state = atoi(argv[2]);
    if (state != 0 && state != 1)
    {
        dump_wlan_11d_enable_usage();
        return;
    }

    if (string_equal("sta", argv[1]))
        wlan_set_11d_state(WLAN_BSS_TYPE_STA, state);
    else if (string_equal("uap", argv[1]))
    {
        if (is_uap_started())
        {
            (void)PRINTF("Error: 11d status can not be changed after uAP start!\r\n");
            return;
        }
        wlan_set_11d_state(WLAN_BSS_TYPE_UAP, state);
    }
    else
        dump_wlan_11d_enable_usage();
}

#if defined(RW610)
static void dump_wlan_country_code(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("Set country code:\r\n");
    (void)PRINTF("    wlan-set-country <country_code_str 3 bytes>\r\n");
    (void)PRINTF("First two octets are used as the\r\n");
    (void)PRINTF("first two octets of the Country String\r\n");
    (void)PRINTF("For example:\r\n");
    (void)PRINTF("    wlan-set-country WW\r\n");
#if 0
    (void)PRINTF("    wlan-set-country EU\r\n");
#endif
    (void)PRINTF("Country Code Options: \r\n");
    (void)PRINTF("    WW  (World Wide Safe)\r\n");
#if 0
    (void)PRINTF("    US  (US FCC)\r\n");
    (void)PRINTF("    CA  (IC Canada)\r\n");
    (void)PRINTF("    SG  (Singapore)\r\n");
    (void)PRINTF("    EU  (ETSI)\r\n");
    (void)PRINTF("    AU  (Australia)\r\n");
    (void)PRINTF("    KR  (Republic Of Korea)\r\n");
    (void)PRINTF("    FR  (France)\r\n");
    (void)PRINTF("    JP  (Japan)\r\n");
    (void)PRINTF("    CN  (China)\r\n");
#endif
    (void)PRINTF("The third octet of the Country String as below\r\n");
    (void)PRINTF("All environments of the current frequency band and country (default)\r\n");
    (void)PRINTF("country3=0x20\r\n");
    (void)PRINTF("Outdoor environment only\r\n");
    (void)PRINTF("country3=0x4f\r\n");
    (void)PRINTF("Indoor environment only\r\n");
    (void)PRINTF("country3=0x49\r\n");
    (void)PRINTF("Noncountry entity (country_code=XX)\r\n");
    (void)PRINTF("country3=0x58\r\n");
    (void)PRINTF("IEEE 802.11 standard Annex E table indication: 0x01 .. 0x1f\r\n");
    (void)PRINTF("Annex E, Table E-4 (Global operating classes)\r\n");
    (void)PRINTF("country3=0x04\r\n");
}
#endif

static void test_wlan_set_country_code(int argc, char **argv)
{
#if defined(RW610)
    int ret;
    char country_code[3] = {0};

    if (argc != 2)
    {
        (void)PRINTF("Invalid arguments\r\n");
        dump_wlan_country_code();
        return;
    }

    country_code[0] = argv[1][0];
    country_code[1] = argv[1][1];

    ret = wlan_set_country_code(country_code);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Set country code %s is Invalid\r\n", country_code);
    }
    else
    {
        (void)PRINTF("Set country code %s is successful\r\n", country_code);
    }
#else
    (void)PRINTF("Set country code Feature not supported.\r\n");
    return;
#endif
}

static void dump_wlan_set_country_ie_ignore_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-set-country-ignore-ie <0/1>\r\n");
    (void)PRINTF("    <enable/disable>: 1 -- Ignore country ie\r\n");
    (void)PRINTF("                      0 -- Use country ie(default)\r\n");
}

static void test_wlan_set_country_ie_ignore(int argc, char **argv)
{
    int ret        = -WM_FAIL;
    uint8_t ignore = 0;

    if (argc > 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_country_ie_ignore_usage();
        return;
    }

    /* SET */
    if (argc == 2)
    {
        ignore = atoi(argv[1]);
    }

    ret = wlan_set_country_ie_ignore(&ignore);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Set country ie ignore is failed\r\n");
    }
    else
    {
        (void)PRINTF("Country ie \"%s\" is set\r\n", ignore == 0 ? "follow" : "ignore");
    }
}

#if CONFIG_COEX_DUTY_CYCLE
static void dump_wlan_single_ant_duty_cycle_usage()
{
    (void)PRINTF("Usage: wlan-single-ant-duty-cycle <enable/disable> [<Ieee154Duration> <TotalDuration>]\r\n");
    (void)PRINTF("    <enable/disable> Enable - 1, Disable - 0\r\n");
    (void)PRINTF("    <Ieee154Duration> Enter value in Units (1Unit = 1ms), no more than TotalDuration\r\n");
    (void)PRINTF("    <TotalDuration> Enter value in Units (1Unit = 1ms), total duty cycle time\r\n");
    (void)PRINTF("    Ieee154Duration should not equal to TotalDuration-Ieee154Duration\r\n");
}

static void test_wlan_single_ant_duty_cycle(int argc, char **argv)
{
    int ret = -WM_FAIL;
    unsigned int enable, nbTime, wlanTime;
    if (argc != 4 && argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_single_ant_duty_cycle_usage();
        return;
    }

    if (get_uint(argv[1], &enable, strlen(argv[1])))
    {
        (void)PRINTF("Invalid arguments\r\n");
        dump_wlan_single_ant_duty_cycle_usage();
        return;
    }

    if (enable == 1 && argc == 4)
    {
        if (get_uint(argv[2], &nbTime, strlen(argv[2])) || get_uint(argv[3], &wlanTime, strlen(argv[3])))
        {
            (void)PRINTF("Invalid arguments\r\n");
            dump_wlan_single_ant_duty_cycle_usage();
            return;
        }

        if ((nbTime > wlanTime) || (nbTime == (wlanTime - nbTime)))
        {
            (void)PRINTF("Invalid arguments\r\n");
            dump_wlan_single_ant_duty_cycle_usage();
            return;
        }
    }
    else
    {
        nbTime   = 0;
        wlanTime = 0;
    }

    ret = wlan_single_ant_duty_cycle((t_u16)enable, (t_u16)nbTime, (t_u16)wlanTime);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Set single ant duty cycle successfully\r\n");
    }
    else
    {
        (void)PRINTF("Failed to set single ant duty cycle\r\n");
    }

    return;
}

static void dump_wlan_dual_ant_duty_cycle_usage()
{
    (void)PRINTF(
        "Usage: wlan-dual-ant-duty-cycle <enable/disable> [<Ieee154Duration> <TotalDuration> "
        "<Ieee154FarRangeDuration>]\r\n");
    (void)PRINTF("    <enable/disable> Enable - 1, Disable - 0\r\n");
    (void)PRINTF("    <Ieee154Duration> Enter value in Units (1Unit = 1ms), no more than TotalDuration\r\n");
    (void)PRINTF("    <TotalDuration> Enter value in Units (1Unit = 1ms)\r\n");
    (void)PRINTF("    <Ieee154FarRangeDuration> Enter value in Units (1Unit = 1ms)\r\n");
    (void)PRINTF("    Ieee154Duration, TotalDuration and Ieee154FarRangeDuration should not equal to each other\r\n");
}

static void test_wlan_dual_ant_duty_cycle(int argc, char **argv)
{
    int ret = -WM_FAIL;
    unsigned int enable, nbTime, wlanTime, wlanBlockTime;
    if (argc != 5 && argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_dual_ant_duty_cycle_usage();
        return;
    }

    if (get_uint(argv[1], &enable, strlen(argv[1])))
    {
        (void)PRINTF("Invalid arguments\r\n");
        dump_wlan_dual_ant_duty_cycle_usage();
        return;
    }

    if (enable == 1 && argc == 5)
    {
        if (get_uint(argv[2], &nbTime, strlen(argv[2])) || get_uint(argv[3], &wlanTime, strlen(argv[3])) ||
                get_uint(argv[4], &wlanBlockTime, strlen(argv[4])))
        {
            (void)PRINTF("Invalid arguments\r\n");
            dump_wlan_dual_ant_duty_cycle_usage();
            return;
        }

        if ((nbTime > wlanTime) || (nbTime == (wlanTime - nbTime)) || (nbTime == wlanBlockTime) ||
                (wlanBlockTime == (wlanTime - nbTime)))
        {
            (void)PRINTF("Invalid arguments\r\n");
            dump_wlan_dual_ant_duty_cycle_usage();
            return;
        }
    }
    else
    {
        nbTime        = 0;
        wlanTime      = 0;
        wlanBlockTime = 0;
    }

    ret = wlan_dual_ant_duty_cycle((t_u16)enable, (t_u16)nbTime, (t_u16)wlanTime, (t_u16)wlanBlockTime);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Set dual ant duty cycle successfully\r\n");
    }
    else
    {
        (void)PRINTF("Failed to set dual ant duty cycle\r\n");
    }

    return;
}
#endif

#if CONFIG_EXTERNAL_COEX_PTA

static void dump_wlan_external_coex_pta_usage()
{
    (void)PRINTF(
        "Usage: wlan-external-coex-pta enable <PTA/WCI-2/WCI-2 GPIO> ExtWifiBtArb <enable/disable> PolGrantPin "
        "<high/low> "
        "PriPtaInt <enable/disable>  StateFromPta <state pin/ priority pin/ state input disable> SampTiming <Sample "
        "timing> "
        "InfoSampTiming <Sample timing> TrafficPrio <enable/disable> CoexHwIntWic <enable/disable>\r\n");
    (void)PRINTF("    enable <enable/disable>\r\n");
    (void)PRINTF("          Select PTA interface: 5, Select WCI-2 interface: 6, Select WCI-2 GPIO interface: 7.\r\n");
    (void)PRINTF("    ExtWifiBtArb <enable/disable>\r\n");
    (void)PRINTF("          Enable Ext-WifiBtArb: 1, Disbale Ext-WifiBtArb: 0.\r\n");
    (void)PRINTF("    PolGrantPin <high/low> \r\n");
    (void)PRINTF("          Active High: 0, Active Low: 1.\r\n");
    (void)PRINTF("    PriPtaInt <enable/disable> \r\n");
    (void)PRINTF("          Enable PriPta-Init: 1, Disable PriPta-Init: 0.\r\n");
    (void)PRINTF("    StateFromPta <state pin/ priority pin/ state input disable> \r\n");
    (void)PRINTF("          State input disbale : 0.\r\n");
    (void)PRINTF("          State info is from state pin : 1.\r\n");
    (void)PRINTF("          State info is sampled on priority pin: 2.\r\n");
    (void)PRINTF("    SampTiming <Sample timing> \r\n");
    (void)PRINTF("          Timing to sample Priority bit.\r\n");
    (void)PRINTF("          Sample timing range [20, 200].\r\n");
    (void)PRINTF("          Defalut value: 100.\r\n");
    (void)PRINTF("    InfoSampTiming <Sample timing> \r\n");
    (void)PRINTF("          Timing to sample Tx/Rx info.\r\n");
    (void)PRINTF("          Sample timing range [20, 200].\r\n");
    (void)PRINTF("          Defalut value: 100.\r\n");
    (void)PRINTF("    TrafficPrio <enable/disable> \r\n");
    (void)PRINTF("          Enable external traffic Tx/Rx Priority: 1.\r\n");
    (void)PRINTF("          Disable external traffic Tx/Rx Priority: 0.\r\n");
    (void)PRINTF("    CoexHwIntWic <enable/disable> \r\n");
    (void)PRINTF("          Enable WCI-2 interface: 1.\r\n");
    (void)PRINTF("          Disable WCI-2 interface: 0.\r\n");
}

static void test_wlan_external_coex_pta(int argc, char **argv)
{
    int ret = 0, arg = 0;
    unsigned int value;
    ext_coex_pta_cfg coex_pta_config;

    if (argc < 2 || argc > 19)
    {
        (void)PRINTF("Error: invalid number of arguments.\r\n");
        dump_wlan_external_coex_pta_usage();
        return;
    }

    struct
    {
        unsigned enable : 1;
        unsigned ExtWifiBtArb : 1;
        unsigned PolGrantPin : 1;
        unsigned PriPtaInt : 2;
        unsigned StateFromPta : 1;
        unsigned SampTiming : 1;
        unsigned InfoSampTiming : 1;
        unsigned TrafficPrio : 1;
        unsigned CoexHwIntWic : 1;
    } info;

    arg++;
    (void)memset(&coex_pta_config, 0x0, sizeof(ext_coex_pta_cfg));
    (void)memset(&info, 0x0, sizeof(info));

    do
    {
        if (!info.enable && string_equal("enable", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid enable argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value != EXT_COEX_PTA_INTERFACE && value != EXT_COEX_WCI2_INTERFACE &&
                    value != EXT_COEX_WCI2_GPIO_INTERFACE)
            {
                (void)PRINTF("Invalid enable argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.enabled = value & 0xFF;
            info.enable             = 1;
            arg += 2;
        }
        else if (!info.ExtWifiBtArb && string_equal("ExtWifiBtArb", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid ExtWifiBtArb argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value != COEX_PTA_FEATURE_ENABLE && value != COEX_PTA_FEATURE_DISABLE)
            {
                (void)PRINTF("Invalid ExtWifiBtArb argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.ext_WifiBtArb = value & 0xFF;
            info.ExtWifiBtArb             = 1;
            arg += 2;
        }
        else if (!info.PolGrantPin && string_equal("PolGrantPin", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid PolGrantPin argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value != POL_GRANT_PIN_HIGH && value != POL_GRANT_PIN_LOW)
            {
                (void)PRINTF("Invalid PolGrantPin argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.polGrantPin = value & 0x0FF;
            info.PolGrantPin            = 1;
            arg += 2;
        }
        else if (!info.PriPtaInt && string_equal("PriPtaInt", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid PolGrantPin argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value != COEX_PTA_FEATURE_ENABLE && value != COEX_PTA_FEATURE_DISABLE)
            {
                (void)PRINTF("Invalid PolGrantPin argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.enable_PriPtaInt = value & 0xFF;
            info.PriPtaInt                   = 1;
            arg += 2;
        }
        else if (!info.StateFromPta && string_equal("StateFromPta", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid StateFromPta argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value != STATE_INPUT_DISABLE && value != STATE_PTA_PIN && value != STATE_PRIORITY_PIN)
            {
                (void)PRINTF("Invalid StateFromPta argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.enable_StatusFromPta = value & 0xFF;
            info.StateFromPta                    = 1;
            arg += 2;
        }
        else if (!info.SampTiming && string_equal("SampTiming", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid SampTiming argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value < MIN_SAMP_TIMING || value > MAX_SAMP_TIMING)
            {
                (void)PRINTF("Invalid SampTiming argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.setPriSampTiming = value & 0xFFFF;
            info.SampTiming                  = 1;
            arg += 2;
        }
        else if (!info.InfoSampTiming && string_equal("InfoSampTiming", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid InfoSampTiming argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value < MIN_SAMP_TIMING || value > MAX_SAMP_TIMING)
            {
                (void)PRINTF("Invalid InfoSampTiming argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.setStateInfoSampTiming = value & 0xFFFF;
            info.InfoSampTiming                    = 1;
            arg += 2;
        }
        else if (!info.TrafficPrio && string_equal("TrafficPrio", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid TrafficPrio argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value != COEX_PTA_FEATURE_ENABLE && value != COEX_PTA_FEATURE_DISABLE)
            {
                (void)PRINTF("Invalid TrafficPrio argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.extRadioTrafficPrio = value & 0xFF;
            info.TrafficPrio                    = 1;
            arg += 2;
        }
        else if (!info.CoexHwIntWic && string_equal("CoexHwIntWic", argv[arg]))
        {
            if (arg + 1 >= argc || get_uint(argv[arg + 1], &value, strlen(argv[arg + 1])))
            {
                (void)PRINTF("Invalid CoexHwIntWic argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }

            if (value != COEX_PTA_FEATURE_ENABLE && value != COEX_PTA_FEATURE_DISABLE)
            {
                (void)PRINTF("Invalid CoexHwIntWic argument.\r\n");
                dump_wlan_external_coex_pta_usage();
                return;
            }
            coex_pta_config.extCoexHwIntWci2 = value & 0xFF;
            info.CoexHwIntWic                = 1;
            arg += 2;
        }
        else
        {
            (void)PRINTF("Invalid %d argument %s\r\n", arg, argv[arg]);
            dump_wlan_external_coex_pta_usage();
            return;
        }
    }
    while (arg < argc);

    if (info.enable != 1)
    {
        (void)PRINTF("Error: Missing <enable> argument.\r\n");
        ret++;
    }
    if (info.ExtWifiBtArb != 1)
    {
        (void)PRINTF("Error: Missing <ExtWifiBtArb> argument.\r\n");
        ret++;
    }
    if (info.PolGrantPin != 1)
    {
        (void)PRINTF("Error: Missing <PolGrantPin> argument.\r\n");
        ret++;
    }
    if (info.PriPtaInt != 1)
    {
        (void)PRINTF("Error: Missing <PriPtaInt> argument.\r\n");
        ret++;
    }
    if (info.StateFromPta != 1)
    {
        (void)PRINTF("Error: Missing <StateFromPta> argument.\r\n");
        ret++;
    }
    if (info.SampTiming != 1)
    {
        coex_pta_config.setPriSampTiming = SAMPLE_TIMING_VALUE;
        (void)PRINTF("Info: Missing <SampTiming> argument. Use default value 100.\r\n");
    }
    if (info.InfoSampTiming != 1)
    {
        coex_pta_config.setStateInfoSampTiming = SAMPLE_TIMING_VALUE;
        (void)PRINTF("Info: Missing <InfoSampTiming> argument. Use default value 100.\r\n");
    }
    if (info.TrafficPrio != 1)
    {
        (void)PRINTF("Error: Missing <TrafficPrio> argument.\r\n");
        ret++;
    }
    if (info.CoexHwIntWic != 1)
    {
        (void)PRINTF("Error: Missing <CoexHwIntWic> argument.\r\n");
        ret++;
    }

    if (ret != 0)
    {
        dump_wlan_external_coex_pta_usage();
        return;
    }

    ret = wlan_external_coex_pta_cfg(coex_pta_config);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Success to set external coex pta config:\r\n");
        (void)PRINTF("Enable ");
        if (coex_pta_config.enabled == EXT_COEX_PTA_INTERFACE)
            (void)PRINTF("PTA interface.\r\n");
        else if (coex_pta_config.enabled == EXT_COEX_WCI2_INTERFACE)
            (void)PRINTF("WCI-2 interface.\r\n");
        else if (coex_pta_config.enabled == EXT_COEX_WCI2_GPIO_INTERFACE)
            (void)PRINTF("WCI-2 GPIO interface.\r\n");
        else
            (void)PRINTF("Unknow interface.\r\n");

        (void)PRINTF("%s WifiBtArb.\r\n",
                     coex_pta_config.ext_WifiBtArb == COEX_PTA_FEATURE_ENABLE ? "Enable" : "Disbale");
        (void)PRINTF("PolGrantPin active %s.\r\n", coex_pta_config.polGrantPin == POL_GRANT_PIN_HIGH ? "High" : "Low");
        (void)PRINTF("%s PriPtaInt.\r\n",
                     coex_pta_config.enable_PriPtaInt == COEX_PTA_FEATURE_ENABLE ? "Enable" : "Disbale");
        if (coex_pta_config.enable_StatusFromPta == STATE_INPUT_DISABLE)
            (void)PRINTF("State input disable.\r\n");
        else if (coex_pta_config.enable_StatusFromPta == STATE_PTA_PIN)
            (void)PRINTF("State info is from state pin.\r\n");
        else if (coex_pta_config.enable_StatusFromPta == STATE_PRIORITY_PIN)
            (void)PRINTF("State info is sampled on priority pin.\r\n");
        else
            (void)PRINTF("Unknow state pin info\r\n");

        (void)PRINTF("Timing to sample Priority bit: %d.\r\n", coex_pta_config.setPriSampTiming);
        (void)PRINTF("Timing to sample Tx/Rx info: %d.\r\n", coex_pta_config.setStateInfoSampTiming);
        (void)PRINTF("%s external traffic Tx/Rx Priority.\r\n",
                     coex_pta_config.extRadioTrafficPrio == COEX_PTA_FEATURE_ENABLE ? "Enable" : "Disbale");
        (void)PRINTF("%s WCI-2 interface.\r\n",
                     coex_pta_config.extCoexHwIntWci2 == COEX_PTA_FEATURE_ENABLE ? "Enable" : "Disbale");
    }
    else
        (void)PRINTF("Failed to set external coex pta parameters.\r\n");

    return;
}
#endif

#if CONFIG_WPA_SUPP_DPP
static void test_wlan_dpp_configurator_add(int argc, char **argv)
{
    int conf_id, is_ap = 0;
    char empty_cmd[1] = {0};

    if (is_uap_started())
    {
        is_ap = 1;
    }
    if (argc >= 1)
    {
        conf_id = wlan_dpp_configurator_add(is_ap, argv[1]);
    }
    else
    {
        conf_id = wlan_dpp_configurator_add(is_ap, empty_cmd);
    }
    if (conf_id == -WM_FAIL)
    {
        (void)PRINTF("\r\nDPP add configurator failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\nconf_id = %d\r\n", conf_id);
    }
}

static void dump_dpp_configurator_params_usage(void)
{
    (void)PRINTF("set DPP configurator params\r\n");
    (void)PRINTF("Usage: wlan-dpp-configurator-params \" conf=....\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-configurator-params \" conf=sta-dpp ssid=4450505f54455354 configurator=1\"\r\n");
    (void)PRINTF("#space character exists between \" & conf word.\r\n");
}

static void test_wlan_dpp_configurator_params(int argc, char **argv)
{
    int is_ap = 0;

    if (argc < 1)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_configurator_params_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }
    wlan_dpp_configurator_params(is_ap, argv[1]);
}

static void dump_dpp_mud_url_usage(void)
{
    (void)PRINTF("MUD URL for Enrollee's DPP Configuration Request\r\n");
    (void)PRINTF("Usage: wlan-dpp-mud-url \"https://....\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-mud-url \"https://example.com/mud\"\r\n");
}

static void test_wlan_dpp_mud_url(int argc, char **argv)
{
    int is_ap = 0;

    if (argc < 1)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_mud_url_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }
    wlan_dpp_mud_url(is_ap, argv[1]);
}

static void dump_dpp_bootstrap_gen_usage(void)
{
    (void)PRINTF("Generate QR code\r\n");
    (void)PRINTF("Usage: wlan-dpp-bootstrap-gen \" type=....\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-bootstrap-gen \"type=qrcode chan=115/36 mac=00:50:43:02:11:22\"\r\n");
}

static void test_wlan_dpp_bootstrap_gen(int argc, char **argv)
{
    int is_ap = 0, id;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_bootstrap_gen_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }
    id = wlan_dpp_bootstrap_gen(is_ap, argv[1]);
    if (id == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP bootstrap generate failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n bootstrap generate id = %d\r\n", id);
    }
}

static void dump_dpp_bootstrap_get_uri_usage(void)
{
    (void)PRINTF("Get QR code string by <bootstrap-id>\r\n");
    (void)PRINTF("Usage: wlan-dpp-bootstrap-get-uri <bootstrap-id>\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-bootstrap-get-uri 1\r\n");
}

static void test_wlan_dpp_bootstrap_get_uri(int argc, char **argv)
{
    int is_ap       = 0, id;
    const char *uri = NULL;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_bootstrap_get_uri_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }
    id = (int)atoi(argv[1]);

    uri = wlan_dpp_bootstrap_get_uri(is_ap, id);
    if (uri)
    {
        OSA_TimeDelay(1000);
        (void)PRINTF("\r\nBootstrapping QR Code URI:\r\n");
        (void)PRINTF("\r\n%s\r\n\r\n", uri);
    }
    else
    {
        (void)PRINTF("Error: generate bootstrapping QR Code URI failed!!\r\n");
    }
}

static void dump_dpp_qr_code_usage(void)
{
    (void)PRINTF("Enter the QR code\r\n");
    (void)PRINTF("Usage: wlan-dpp-qr-code <URI-from-QR-Code-read-from-enrollee>\r\n");
}

static void test_wlan_dpp_qr_code(int argc, char **argv)
{
    int id;
    int is_ap = 0;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_qr_code_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }
    id = wlan_dpp_qr_code(is_ap, argv[1]);
    if (id == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP bootstrap generate failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n DPP qr code id = %d\r\n", id);
    }
}

static void dump_dpp_auth_init_usage(void)
{
    (void)PRINTF("Send provisioning Auth request to responder\r\n");
    (void)PRINTF("Usage: wlan-dpp-auth-init \"peer=...\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-auth-init  \"peer=1 role=enrollee\"\r\n");
}

static void test_wlan_dpp_auth_init(int argc, char **argv)
{
    int ret;
    int is_ap = 0;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_auth_init_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }
    (void)wlan_ieeeps_off();
    (void)wlan_deepsleepps_off();
    ret = wlan_dpp_auth_init(is_ap, argv[1]);
    if (ret == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP Auth Init failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n DPP Auth Init OK!\r\n");
    }
}

static void dump_dpp_listen_usage(void)
{
    (void)PRINTF("Make device listen to DPP request.\r\n");
    (void)PRINTF("Usage: wlan-dpp-listen \"<frequency>...\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-listen 5180\r\n");
}

static void test_wlan_dpp_listen(int argc, char **argv)
{
    int ret;
    int is_ap = 0;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_listen_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }
    (void)wlan_ieeeps_off();
    (void)wlan_deepsleepps_off();
    ret = wlan_dpp_listen(is_ap, argv[1]);
    if (ret == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP Listen failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n DPP Listen OK!\r\n");
    }
}

static void test_wlan_dpp_stop_listen(int argc, char **argv)
{
    int ret;
    int is_ap = 0;

    if (is_uap_started())
    {
        is_ap = 1;
    }
    ret = wlan_dpp_stop_listen(is_ap);
    if (ret == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP Listen STOP failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n DPP Listen STOP OK!\r\n");
    }
}

static void dump_dpp_pkex_add_usage(void)
{
    (void)PRINTF("Set DPP bootstrapping through PKEX(Public Key Exchange)\r\n");
    (void)PRINTF("Usage: wlan-dpp-pkex-add \"own=<bootstrap_id> identifier=<string> code=<string>...\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-pkex-add \"own=1 identifier=test code=DPP_Device_PKEX\"\r\n");
}

static void test_wlan_dpp_pkex_add(int argc, char **argv)
{
    int ret;
    int is_ap = 0;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_pkex_add_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }

    (void)wlan_ieeeps_off();
    (void)wlan_deepsleepps_off();
    ret = wlan_dpp_pkex_add(is_ap, argv[1]);
    if (ret == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP add PKEX failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n DPP add PKEX OK!\r\n");
    }
}

static void dump_dpp_chirp_usage(void)
{
    (void)PRINTF("sends DPP presence announcement.\r\n");
    (void)PRINTF("Usage: wlan-dpp-chirp \"own=<bootstrap id> listen=<freq>...\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-chirp \"own=1 listen=2412\"\r\n");
}

static void test_wlan_dpp_chirp(int argc, char **argv)
{
    int ret;
    int is_ap = 0;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_chirp_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }

    (void)wlan_ieeeps_off();
    (void)wlan_deepsleepps_off();
    ret = wlan_dpp_chirp(is_ap, argv[1]);
    if (ret == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP chirping failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n DPP chirping OK!\r\n");
    }
}

static void dump_dpp_reconfig_usage(void)
{
    (void)PRINTF("Make STA device do DPP reconfig.\r\n");
    (void)PRINTF("Usage: wlan-dpp-reconfig \"<network_id> ...\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-reconfig 1\r\n");
}

static void test_wlan_dpp_reconfig(int argc, char **argv)
{
    int ret;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_reconfig_usage();
        return;
    }
    if (is_uap_started())
    {
        (void)PRINTF("\r\n Only support STA to do DPP reconfig\r\n");
    }
    (void)wlan_ieeeps_off();
    (void)wlan_deepsleepps_off();
    ret = wlan_dpp_reconfig(argv[1]);
    if (ret == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP reconfig failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n DPP reconfig OK!\r\n");
    }
}

static void dump_dpp_configurator_sign_usage(void)
{
    (void)PRINTF("Configurator configures itself as an Enrollee AP/STA\r\n");
    (void)PRINTF("Usage: wlan-dpp-configurator-sign \" conf=....\"\r\n");
    (void)PRINTF("\r\nUsage example : \r\n");
    (void)PRINTF("wlan-dpp-configurator-sign \" conf=sta-dpp ssid=4450505f54455354 configurator=1\"\r\n");
    (void)PRINTF("#space character exists between \" & conf word.\r\n");
}

static void test_wlan_dpp_configurator_sign(int argc, char **argv)
{
    int ret;
    int is_ap = 0;

    if (argc < 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_dpp_configurator_sign_usage();
        return;
    }
    if (is_uap_started())
    {
        is_ap = 1;
    }

    ret = wlan_dpp_configurator_sign(is_ap, argv[1]);
    if (ret == -WM_FAIL)
    {
        (void)PRINTF("\r\n DPP chirping failed!!\r\n");
    }
    else
    {
        (void)PRINTF("\r\n DPP chirping OK!\r\n");
    }
}
#endif

#if CONFIG_IMD3_CFG

static void dump_wlan_imd3_cfg_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-imd3-cfg <enable / disable> <isolation index>\r\n");
    (void)PRINTF("      <enable> 1: enable / 0: disable\r\n");
    (void)PRINTF("      <isolation index> range: 1 ~ 5 or 15\r\n");
    (void)PRINTF("      If set index to 15, use default value.");
    (void)PRINTF("Fox example:\r\n");
    (void)PRINTF("    wlan-imd3-cfg 0  \r\n");
    (void)PRINTF("    wlan-imd3-cfg 1 3\r\n");
}

static void test_wlan_imd3_cfg(int argc, char **argv)
{
    int ret = 0;
    uint8_t enable, index, imd3_cfg = 0;
    unsigned int value;

    if (argc != 2 && argc != 3)
    {
        (void)PRINTF("Error: invalid number of arguments.\r\n");
        dump_wlan_imd3_cfg_usage();
        return;
    }

    if (get_uint(argv[1], &value, strlen(argv[1])) || (value != 1 && value != 0))
    {
        (void)PRINTF("Invalid <enable> argument \r\n");
        dump_wlan_imd3_cfg_usage();
        return;
    }

    enable = value & 0xFF;

    if (enable == 1 && argc == 2)
    {
        (void)PRINTF("Missing <isolation index> argument \r\n");
        dump_wlan_imd3_cfg_usage();
        return;
    }

    if (enable == 1 && argc == 3)
    {
        if (get_uint(argv[2], &value, strlen(argv[2])) || ((value < 1 || value > 5) && value != 15))
        {
            (void)PRINTF("Invalid <index> argument \r\n");
            dump_wlan_imd3_cfg_usage();
            return;
        }

        index = value & 0xFF;
        /* imd3_cfg --> low 4 bits: enable / high 4 bits: index*/
        imd3_cfg |= enable;
        index <<= 4;
        imd3_cfg |= index;
    }

    ret = wlan_imd3_cfg(imd3_cfg);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Success to set IMD cfg.\r\n");
    }
    else
    {
        (void)PRINTF("Failed to set IMD cfg.\r\n");
    }
}
#endif

#if CONFIG_AUTO_RECONNECT
#define AUTO_RECON_CNT_DEF      255U
#define AUTO_RECON_TIME_INT_DEF 10U
#define AUTO_RECON_FLAG_DEF     0U

static void dump_wlan_auto_reconnect_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-auto-reconnect <0/1/2> [<reconnect counter> <reconnect interval> <flags>]\r\n");
    (void)PRINTF("        <0/1/2> : 0 - Disable auto reconnect\r\n");
    (void)PRINTF("                  1 - Enable auto reconnect\r\n");
    (void)PRINTF("                  2 - Get auto reconnect configuration\r\n");
    (void)PRINTF("        <reconnect counter>  : 1-255 Auto reconnect attempts (Defult:255 - retry forever)\r\n");
    (void)PRINTF("        <reconnect interval> : 0-255 Auto reconnect time period in seconds(Default:10 sec)\r\n");
    (void)PRINTF(
        "        <flags> : 0-15, 0: Default, Don't report link loss, 1: Report link loss to host, 2-15: Reserved\r\n");
    (void)PRINTF("Examples:\r\n");
    (void)PRINTF("    wlan-auto-reconnect 0\r\n");
    (void)PRINTF("    wlan-auto-reconnect 1 10 10 0\r\n");
    (void)PRINTF("    wlan-auto-reconnect 2\r\n");
    return;
}

static void test_wlan_auto_reconnect(int argc, char **argv)
{
    int ret      = -WM_FAIL;
    char *endptr = NULL;
    int enable   = -1;
    wlan_auto_reconnect_config_t recon_config;

    uint8_t recon_counter  = AUTO_RECON_CNT_DEF;
    uint8_t recon_interval = AUTO_RECON_TIME_INT_DEF;
    uint16_t flags         = AUTO_RECON_FLAG_DEF;

    if (argc < 2 || argc > 5)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        goto done;
    }

    errno  = 0;
    enable = (int)strtol(argv[1], &endptr, 10);
    if (errno != 0 || *endptr != '\0')
    {
        (void)PRINTF("Error during strtol:enable\r\n");
        goto done;
    }

    if (enable == 0)
    {
        ret = wlan_auto_reconnect_disable();
        if (ret == WM_SUCCESS)
        {
            (void)PRINTF("Disabled auto reconnect\r\n");
        }
        else
        {
            (void)PRINTF("Failed to disable auto reconnect, error: %d\r\n", ret);
        }
    }
    else if (enable == 1)
    {
        if (argc > 2)
        {
            errno         = 0;
            recon_counter = (uint8_t)strtol(argv[2], &endptr, 10);
            if (errno != 0 || *endptr != '\0')
            {
                (void)PRINTF("Error during strtol:reconnect counter\r\n");
                goto done;
            }
        }

        if (recon_counter == 0)
        {
            (void)PRINTF("Auto reconnect counter can not be 0\r\n");
            goto done;
        }

        if (argc > 3)
        {
            errno          = 0;
            recon_interval = (uint8_t)strtol(argv[3], &endptr, 10);
            if (errno != 0 || *endptr != '\0')
            {
                (void)PRINTF("Error during strtol:reconnect interval\r\n");
                goto done;
            }
        }

        if (argc > 4)
        {
            errno = 0;
            flags = (uint16_t)strtol(argv[4], &endptr, 10);
            if (errno != 0 || *endptr != '\0')
            {
                (void)PRINTF("Error during strtol:flags\r\n");
                goto done;
            }
        }

        recon_config.reconnect_counter  = recon_counter;
        recon_config.reconnect_interval = recon_interval;
        recon_config.flags              = flags;

        ret = wlan_auto_reconnect_enable(recon_config);
        if (ret == WM_SUCCESS)
        {
            (void)PRINTF("Enabled auto reconnect\r\n");
        }
        else
        {
            (void)PRINTF("Failed to enable auto reconnect, error: %d\r\n", ret);
        }
    }
    else if (enable == 2)
    {
        ret = wlan_get_auto_reconnect_config(&recon_config);
        if (ret == WM_SUCCESS)
        {
            (void)PRINTF("Auto Reconnect Counter = %d\r\n", recon_config.reconnect_counter);
            (void)PRINTF("Auto Reconnect Interval = %d\r\n", recon_config.reconnect_interval);
            (void)PRINTF("Auto Reconnect Flags = %d\r\n", recon_config.flags);
        }
        else
        {
            (void)PRINTF("Failed to get auto reconnect configuration, error: %d\r\n", ret);
        }
    }
    else
    {
        (void)PRINTF("Error: Specify 0/1/2 to Disable/Enable/Get auto reconnect configuration\r\n");
        goto done;
    }

    return;

done:
    dump_wlan_auto_reconnect_usage();
    return;
}
#endif

#if (CONFIG_WIFI_IND_RESET) && (CONFIG_WIFI_IND_DNLD)
static void dump_wlan_set_ind_rst_cfg_usage(void)
{
    (void)PRINTF("Usage :                                                                \r\n");
    (void)PRINTF("         wlan-set-indrstcfg <ir_mode> [gpio_pin]                       \r\n");
    (void)PRINTF("         ir_mode   : 0 -- Disable                                      \r\n");
    (void)PRINTF("                     1 -- Enable out band reset, disable in band       \r\n");
    (void)PRINTF("                     2 -- Enable in band, disable out band             \r\n");
    (void)PRINTF("         gpio_pin  : 255 -- Default pin for reset                      \r\n");
    (void)PRINTF("                     any other number for changing the gpio for reset. \r\n");
    (void)PRINTF("Example :                                                              \r\n");
    (void)PRINTF("         wlan-set-indrstcfg 1 255   : Set default pin as reset pin     \r\n");
    (void)PRINTF("         wlan-set-indrstcfg 0       : Disable the independent reset    \r\n");
    (void)PRINTF("         wlan-set-indrstcfg 2       : Enable in band reset mode        \r\n");
}

static void test_set_indrst_cfg(int argc, char **argv)
{
    wifi_indrst_cfg_t indrst_cfg;

    if (argc < 2 || argc > 3)
    {
        dump_wlan_set_ind_rst_cfg_usage();
        return;
    }

    memset(&indrst_cfg, 0, sizeof(wifi_indrst_cfg_t));

    if ((argc == 2) || (argc == 3))
    {
        errno              = 0;
        indrst_cfg.ir_mode = (uint8_t)strtol(argv[1], NULL, 0);

        if (errno != 0)
        {
            (void)PRINTF("Error during strtoul errno:%d", errno);
        }

        /* ir_mode */
        if (indrst_cfg.ir_mode < 0 || indrst_cfg.ir_mode > 2)
        {
            (void)PRINTF("Invalid ir mode parameter (0/1/2)!\n\r");
            return;
        }

        /* gpio_pin */
        if (argc == 3)
        {
            errno               = 0;
            indrst_cfg.gpio_pin = (uint8_t)strtol(argv[2], NULL, 0);

            if (errno != 0)
            {
                (void)PRINTF("Error during strtoul errno:%d", errno);
            }

            if ((indrst_cfg.gpio_pin != 0xFF) && (indrst_cfg.gpio_pin < 0))
            {
                (void)PRINTF("Invalid gpio pin no !\n\r");
                return;
            }
        }
    }

    int rv = wlan_set_indrst_cfg(&indrst_cfg);

    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set independent reset config\r\n");
    }
    else
    {
        (void)PRINTF("Independent Reset Mode set as: %s\n\r",
                     (indrst_cfg.ir_mode == 0) ? "disabled" : ((indrst_cfg.ir_mode == 1) ? "Out Band" : "In Band"));
    }
}

static void test_get_indrst_cfg(int argc, char **argv)
{
    wifi_indrst_cfg_t indrst_cfg;

    memset(&indrst_cfg, 0, sizeof(wifi_indrst_cfg_t));
    int rv = wlan_get_indrst_cfg(&indrst_cfg);

    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get independent reset config\r\n");
    }
    else
    {
        if ((indrst_cfg.ir_mode < 0) || (indrst_cfg.ir_mode > 2))
        {
            (void)PRINTF("FW error Mode must be 0, 1 or 2\n");
            return;
        }
        PRINTF("Independent Reset Mode = %s\r\n",
               (indrst_cfg.ir_mode == 0) ? "disabled" : ((indrst_cfg.ir_mode == 1) ? "Out Band" : "In Band"));
        if (indrst_cfg.ir_mode == 1)
            (void)PRINTF("GPIO Pin = %d\n\n", indrst_cfg.gpio_pin);
    }
}

static void dump_wlan_independent_reset_usage(void)
{
    (void)PRINTF("Usage :                                     \r\n");
    (void)PRINTF("         wlan-independent-reset             \r\n");
}

static void test_wlan_independent_reset(int argc, char **argv)
{
    int ret = -WM_FAIL;

    if (argc != 1)
    {
        dump_wlan_independent_reset_usage();
        return;
    }

    ret = wlan_independent_reset();

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Independent reset success\r\n");
    }
    else
    {
        (void)PRINTF("Independent reset failed\r\n");
    }
}
#endif

#if CONFIG_INACTIVITY_TIMEOUT_EXT
static void dump_wlan_sta_inactivityto_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-sta-inactivityto <n> <m> <l> [k] [j]\r\n");
    (void)PRINTF("where the parameter are:\r\n");
    (void)PRINTF("      <n>: timeout unit in microseconds.\r\n");
    (void)PRINTF("      <m>: Inactivity timeout for unicast data.\r\n");
    (void)PRINTF("      <l>: Inactivity timeout for multicast data.\r\n");
    (void)PRINTF("      [k]: Inactivity timeout for new Rx traffic after PS notification to AP.\r\n");
    (void)PRINTF("      [j]: Inactivity timeout for cmd.\r\n");
    (void)PRINTF("Fox example:\r\n");
    (void)PRINTF("    wlan-sta-inactivityto          : Get the timeout value\r\n");
    (void)PRINTF("    wlan-sta-inactivityto 1000 2 3 : Set timeout unit to 1000 us (1 ms),\r\n");
    (void)PRINTF("                                     inactivity timeout for unicast data is 2 ms,\r\n");
    (void)PRINTF("                                     inactivity timeout for multicast data is 3 ms\r\n");
}

static void test_wlan_sta_inactivityto(int argc, char **argv)
{
    int ret      = 0;
    t_u16 action = 0;
    wlan_inactivity_to_t inac_to;

    (void)memset(&inac_to, 0, sizeof(wlan_inactivity_to_t));

    /* Check if arguments are valid */
    if ((argc != 1) && (argc != 4) && (argc != 5) && (argc != 6))
    {
        (void)PRINTF("Error: invalid number of arguments.\r\n");
        dump_wlan_sta_inactivityto_usage();
        return;
    }

    /* GET operation */
    if (argc == 1)
    {
        action = ACTION_GET;
    }
    else /* SET operation */
    {
        action                  = ACTION_SET;
        inac_to.timeout_unit    = strtol(argv[1], NULL, 0);
        inac_to.unicast_timeout = strtol(argv[2], NULL, 0);
        inac_to.mcast_timeout   = strtol(argv[3], NULL, 0);
        if (argc >= 5)
            inac_to.ps_entry_timeout = strtol(argv[4], NULL, 0);
        if (argc == 6)
            inac_to.ps_cmd_timeout = strtol(argv[5], NULL, 0);
    }

    ret = wlan_sta_inactivityto(&inac_to, action);
    if (ret == WM_SUCCESS)
    {
        if (action == ACTION_GET)
        {
            (void)PRINTF("Timeout unit is %d us\r\n", inac_to.timeout_unit);
            (void)PRINTF("Inactivity timeout for unicast data is %d ms\r\n", inac_to.unicast_timeout);
            (void)PRINTF("Inactivity timeout for multicast data is %d ms\r\n", inac_to.mcast_timeout);
            (void)PRINTF("Inactivity timeout for new Rx traffic is %d ms\r\n", inac_to.ps_entry_timeout);
            (void)PRINTF("Inactivity timeout for cmd is %d ms\r\n", inac_to.ps_cmd_timeout);
        }
        else
        {
            (void)PRINTF("Success to set STA inactivity timeout.\r\n");
        }
    }
    else
    {
        (void)PRINTF("Failed to set STA inactivity timeout.\r\n");
    }
}
#endif

#ifdef RW610
static void test_wlan_get_temperature(int argc, char **argv)
{
    int32_t board_temperature = 0;

    board_temperature = wlan_get_temperature();

    (void)PRINTF("Board temperature :%d \r\n", board_temperature);
}
#endif

#if CONFIG_AUTO_NULL_TX
static void dump_wlan_auto_null_tx_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("    wlan-auto-null-tx <sta/uap> start interval <interval> dst_mac <dst_mac>\r\n");
    (void)PRINTF("        <interval> bit15:14 unit: 00-s 01-us 10-ms 11-one_shot  bit13-0: interval\r\n");
    (void)PRINTF("                   Please set interval Hexadecimal value. For example: 0x8064\r\n");
    (void)PRINTF("        <dst_mac> Destination MAC address\r\n");
    (void)PRINTF(
        "                  Please specify dst_mac if bss_type is uAP, and dst_mac should be of STA which connected to "
        "uAP\r\n");
    (void)PRINTF("                  If bss_type is not uAP, no need to input dst_mac\r\n");
    (void)PRINTF("    wlan-auto-null-tx sta stop\r\n");
}

static void test_wlan_auto_null_tx(int argc, char **argv)
{
    int ret                = -WM_FAIL;
    int arg                = 2;
    mlan_bss_type bss_type = (mlan_bss_type)0;

    wlan_auto_null_tx_t auto_null_tx;

    if (argc < 3)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_auto_null_tx_usage();
        return;
    }

    memset(&auto_null_tx, 0x00, sizeof(wlan_auto_null_tx_t));

    if (string_equal("sta", argv[1]))
        bss_type = MLAN_BSS_TYPE_STA;
    else if (string_equal("uap", argv[1]))
        bss_type = MLAN_BSS_TYPE_UAP;
    else
    {
        (void)PRINTF("Invalid bss type selection\r\n");
        return;
    }

    if (string_equal("start", argv[2]))
    {
        auto_null_tx.start = MTRUE;
        arg += 1;
        if (string_equal("interval", argv[arg]))
        {
            if (argv[arg + 1][0] == '0' && (argv[arg + 1][1] == 'x' || argv[arg + 1][1] == 'X'))
            {
                auto_null_tx.interval = a2hex_or_atoi(argv[arg + 1]);
            }
            else
            {
                (void)PRINTF("Error: invalid interval argument\r\n");
                dump_wlan_auto_null_tx_usage();
                return;
            }
            arg += 2;
        }
        else
        {
            (void)PRINTF("Error: argument %d is invalid\r\n", arg);
            dump_wlan_auto_null_tx_usage();
            return;
        }

        if (bss_type == MLAN_BSS_TYPE_STA)
        {
            if (is_sta_connected())
            {
                ret = wlan_get_current_network_bssid((char *)&auto_null_tx.dst_mac);
                if (ret != 0)
                {
                    (void)PRINTF("Error: could not get current network bssid\r\n");
                    return;
                }
            }
            else
            {
                (void)PRINTF("Error: not conneted AP\r\n");
                return;
            }
        }
        if (bss_type == MLAN_BSS_TYPE_UAP)
        {
            wifi_sta_list_t *sl = NULL;
            (void)wifi_uap_bss_sta_list(&sl);
            if (!sl)
            {
                (void)PRINTF("Failed to get sta list\n\r");
                return;
            }
            if (is_uap_started() && (sl->count > 0))
            {
                wifi_sta_info_t *si = (wifi_sta_info_t *)(void *)(&sl->count + 1);
                if (string_equal("dst_mac", argv[arg]))
                {
                    unsigned int mac_matched = 0;
                    ret                      = get_mac(argv[arg + 1], (char *)&auto_null_tx.dst_mac, ':');
                    if (ret != 0)
                    {
                        dump_wlan_auto_null_tx_usage();
                        (void)PRINTF("Error: invalid dst_mac argument\r\n");
                        return;
                    }
                    for (int i = 0; i < sl->count; i++)
                    {
                        if (!memcmp(si[i].mac, auto_null_tx.dst_mac, 6))
                        {
                            mac_matched = 1;
                            break;
                        }
                    }
                    if (mac_matched == 0)
                    {
                        (void)PRINTF("Error: wrong dst_mac, please put one connected STA mac address\r\n");
                        return;
                    }
                    arg += 2;
                }
                else
                {
                    (void)PRINTF("Error: argument %d is invalid\r\n", arg);
                    dump_wlan_auto_null_tx_usage();
                    return;
                }
            }
            else
            {
                if (!is_uap_started())
                    (void)PRINTF("uap isn't up\r\n");
                else
                    (void)PRINTF("There is no STA connected to uAP\r\n");
            }
        }
    }
    else if (string_equal("stop", argv[2]))
    {
        auto_null_tx.start = MFALSE;
    }
    else
    {
        (void)PRINTF("Error: invalid [%d] argument, give start/stop\r\n", arg + 1);
        dump_wlan_auto_null_tx_usage();
        return;
    }

    /* bit7-4: bandwidth. bit3-0: priority, ignored if non-WMM */
    auto_null_tx.priority = 0x07;

    ret = wlan_auto_null_tx(&auto_null_tx, bss_type);
}
#endif

#if defined(RW610) && (CONFIG_ANT_DETECT)
static void swap_scan_entry(scan_result_entry_t *pEntry1, scan_result_entry_t *pEntry2)
{
    scan_result_entry_t temp;

    (void)memcpy((void *)&temp, (const void *)pEntry1, sizeof(scan_result_entry_t));
    (void)memcpy((void *)pEntry1, (const void *)pEntry2, sizeof(scan_result_entry_t));
    (void)memcpy((void *)pEntry2, (const void *)&temp, sizeof(scan_result_entry_t));
}

static void copy_scan_result(scan_result_entry_t *pDst, struct wifi_scan_result2 *pSrc)
{
    (void)memcpy((void *)&pDst->bssid[0], (const void *)&pSrc->bssid[0], sizeof(pSrc->bssid));
    (void)memcpy((void *)&pDst->ssid[0], (const void *)((char *)&pSrc->ssid[0]), (size_t)pSrc->ssid_len);
    pDst->ssid[pSrc->ssid_len] = (char)0;
    pDst->ssid_len             = (size_t)pSrc->ssid_len;
    pDst->channel              = pSrc->Channel;
    pDst->rssi                 = pSrc->RSSI;
}

static void wlan_sort_scan_entry(scan_result_entry_t *pScan_entry)
{
    unsigned int i;
    unsigned int j;
    unsigned int minIdx;

    /* selection sort */
    for (i = 0; i < (ANT_DETECT_MAX_SCAN_ENTRY - 1); i++)
    {
        minIdx = i;
        for (j = i + 1; j < ANT_DETECT_MAX_SCAN_ENTRY; j++)
        {
            if (pScan_entry[j].rssi < pScan_entry[minIdx].rssi)
            {
                minIdx = j;
            }
        }

        if (minIdx != i)
        {
            swap_scan_entry(&pScan_entry[minIdx], &pScan_entry[i]);
        }
    }
}

static unsigned char wlan_calculate_avg_rssi(scan_result_entry_t *pScan_entry)
{
    unsigned int i;
    unsigned char avg_rssi = 0;
    uint16_t sum_rssi      = 0;
    uint8_t valid_entry    = 0;

    for (i = 0; i < ANT_DETECT_MAX_SCAN_ENTRY; i++)
    {
        if (pScan_entry[i].rssi == 0xff)
        {
            break;
        }
        else if (i > 0)
        {
            if (pScan_entry[i].rssi - pScan_entry[0].rssi >= 30)
            {
                break;
            }
            else
            {
                valid_entry++;
            }
        }
        else
        {
            valid_entry++;
        }
    }

    if (valid_entry > 0 && valid_entry <= ANT_DETECT_MAX_SCAN_ENTRY)
    {
        (void)PRINTF("%d valid scan entry found\r\n", valid_entry);
        for (i = 0; i < valid_entry; i++)
        {
            sum_rssi += pScan_entry[i].rssi;
        }
        avg_rssi = sum_rssi / valid_entry;
    }

    return avg_rssi;
}

/** Find top 5 best scan info (with high RSSI) and save to scan entry
 *
 * If scan_networks_count is no more than 5, just copy scan info and sort;
 * Otherwise, use below logic:
 * 1.Get the first 5 scan results in scan table to scan entries;
 * 2.sort scan info in scan entries from high rssi to low rssi;
 * 3.then when get a new scan results in scan table, if it has
 *   high rssi than the lowest one in scan entries, replace the
 *   lowest entry with the new scan info and re-sort scan entries.
 *
 */
static void wlan_get_best_scan_info(wlan_ant_detect_data_t *pData, unsigned int scan_networks_count)
{
    int ret;
    unsigned int i;
    unsigned int net_count      = scan_networks_count;
    unsigned int entry_count    = ANT_DETECT_MAX_SCAN_ENTRY;
    wlan_ant_scan_info_t *pInfo = &pData->scan_info[pData->current_ant - 1];
    struct wifi_scan_result2 *res;

    if (net_count <= entry_count)
    {
        for (i = 0; i < net_count; i++)
        {
            ret = wifi_get_scan_result(i, &res);
            if (ret == WM_SUCCESS)
            {
                copy_scan_result(&pInfo->scan_entry[i], res);
            }
            else
            {
                PRINTF("Error: can't get scan res %d\r\n", i);
            }
        }
        wlan_sort_scan_entry(&pInfo->scan_entry[0]);
    }
    else
    {
        unsigned int lowest_rssi_index = entry_count - 1;
        for (i = 0; i < net_count; i++)
        {
            ret = wifi_get_scan_result(i, &res);
            if (ret == WM_SUCCESS)
            {
                if (i < entry_count)
                {
                    copy_scan_result(&pInfo->scan_entry[i], res);

                    if (i == entry_count - 1)
                    {
                        wlan_sort_scan_entry(&pInfo->scan_entry[0]);
                    }
                }
                else
                {
                    if (res->RSSI < pInfo->scan_entry[lowest_rssi_index].rssi)
                    {
                        copy_scan_result(&pInfo->scan_entry[lowest_rssi_index], res);
                        wlan_sort_scan_entry(&pInfo->scan_entry[0]);
                    }
                }
            }
            else
            {
                PRINTF("Error: can't get scan res %d\r\n", i);
            }
        }
    }

    PRINTF("List top %d best scanned AP's info:\r\n", entry_count);
    for (i = 0; i < entry_count; i++)
    {
        print_mac(pInfo->scan_entry[i].bssid);
        PRINTF(" \"%s\"\r\n", pInfo->scan_entry[i].ssid);
        PRINTF("\tchannel: %d\r\n", pInfo->scan_entry[i].channel);
        PRINTF("\trssi: -%d dBm\r\n", pInfo->scan_entry[i].rssi);
    }

    if (pData->detect_mode != PCB_DETECT_MODE)
    {
        pInfo->avg_rssi = wlan_calculate_avg_rssi(&pInfo->scan_entry[0]);
        PRINTF("avg_rssi: -%d dBm\r\n", pInfo->avg_rssi);
    }
}

static void wlan_get_specific_scan_info(wlan_ant_scan_info_t *pInfo, unsigned int scan_networks_count)
{
    int ret;
    unsigned int i;
    unsigned int count = scan_networks_count;
    struct wifi_scan_result2 *res;

    for (i = 0; i < count; i++)
    {
        ret = wifi_get_scan_result(i, &res);
        if (ret == WM_SUCCESS)
        {
            copy_scan_result(&pInfo->scan_entry[pInfo->entry_idx], res);
        }
        else
        {
            PRINTF("Error: can't get scan res %d\r\n", i);
        }
    }

    print_mac(pInfo->scan_entry[pInfo->entry_idx].bssid);
    PRINTF(" \"%s\"\r\n", pInfo->scan_entry[pInfo->entry_idx].ssid);
    PRINTF("\tchannel: %d\r\n", pInfo->scan_entry[pInfo->entry_idx].channel);
    PRINTF("\trssi: -%d dBm\r\n", pInfo->scan_entry[pInfo->entry_idx].rssi);
}

static void wlan_get_best_two_ants(wlan_ant_detect_data_t *pData)
{
    uint16_t i;
    uint16_t minIdx1;
    uint16_t minIdx2;
    wlan_ant_scan_info_t *pScan_info = &pData->scan_info[0];

    if (pScan_info[0].avg_rssi < pScan_info[1].avg_rssi)
    {
        minIdx1 = 0;
        minIdx2 = 1;
    }
    else
    {
        minIdx1 = 1;
        minIdx2 = 0;
    }

    for (i = 2; i < pData->ant_port_count; i++)
    {
        if (pScan_info[i].avg_rssi < pScan_info[minIdx1].avg_rssi)
        {
            minIdx2 = minIdx1;
            minIdx1 = i;
        }
        else if (pScan_info[i].avg_rssi < pScan_info[minIdx2].avg_rssi)
        {
            minIdx2 = i;
        }
    }

    pData->best_ant      = minIdx1 + 1;
    pData->next_best_ant = minIdx2 + 1;
    pData->detect_done   = 1;
}

static void wlan_evaluate_ant_by_avg_rssi(wlan_ant_detect_data_t *pData)
{
    uint16_t i;
    uint8_t valid_res                = pData->ant_port_count;
    wlan_ant_scan_info_t *pScan_info = &pData->scan_info[0];

    (void)PRINTF("\nEvaluate result:\r\n");
    (void)PRINTF("\t        avg_rssi\r\n");
    for (i = 0; i < pData->ant_port_count; i++)
    {
        (void)PRINTF("Antenna %d", i + 1);
        (void)PRINTF("\t-%d dBm\r\n", pScan_info[i].avg_rssi);
        if (pScan_info[i].avg_rssi == 0xff)
        {
            valid_res--;
        }
    }

    (void)PRINTF("There are %d antenna with valid result%s.\r\n", valid_res, valid_res <= 1U ? "" : "s");
    if (valid_res <= 1)
    {
        pData->detect_done = -1;
        return;
    }

    wlan_get_best_two_ants(pData);
}

/** Evaluate all antennas by one common device
 *
 * This function is to find one common BSSID in all antennas' scan entries,
 * and to find 2 best antennas based on the common BSSID's rssi value.
 *
 * The logic is:
 * 1.Find the ant who has the best scan RSSI of all antennas' scan entries;
 * 2.Take this ant and its best scan BSSID as a reference;
 * 3.Loop the reference ant's scan entried (rssi from high to low) and check
 *   whether the same BSSID is in other antennas;
 * 4.If find the same BSSID in one antenna, record its entry idx to the
 *   corresponding location in array com_idx_per_ant[];
 * 5.As com_idx_per_ant[] is initialized as 0xff, check its value and we can
 *   we can know whether find a common BSSID or not.
 * 6.Rank and find 2 best antennas based on the common BSSID's rssi value.
 *
 * \return 1 if find one common BSSID.
 * \return -1 if no common BSSID.
 *
 */
static int wlan_evaluate_ant_by_common_device(wlan_ant_detect_data_t *pData)
{
    uint8_t i;
    uint8_t j;
    uint8_t com_idx_per_ant[MAX_ANTENNA_PORT_NUM];
    uint8_t current_best_ant;
    int detect_done                  = 0;
    t_u8 zero_bssid[]                = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    wlan_ant_scan_info_t *pScan_info = &pData->scan_info[0];
    wlan_ant_scan_info_t *pRef_info  = NULL;

    current_best_ant = 0;
    for (i = 1; i < pData->ant_port_count; i++)
    {
        if (pData->scan_info[i].scan_entry[0].rssi < pData->scan_info[current_best_ant].scan_entry[0].rssi)
        {
            current_best_ant = i;
        }
    }

    pRef_info = &pData->scan_info[current_best_ant];

    memset((void *)&com_idx_per_ant[0], 0xff, pData->ant_port_count);
    for (i = 0; i < ANT_DETECT_MAX_SCAN_ENTRY; i++)
    {
        uint8_t temp = 0;
        if (memcmp(pRef_info->scan_entry[i].bssid, zero_bssid, 6) == 0)
        {
            break;
        }

        for (uint8_t ant_idx = 0; ant_idx < pData->ant_port_count; ant_idx++)
        {
            if (ant_idx == current_best_ant)
            {
                continue;
            }

            for (j = 0; j < ANT_DETECT_MAX_SCAN_ENTRY; j++)
            {
                if (memcmp(pScan_info[ant_idx].scan_entry[j].bssid, zero_bssid, 6) == 0)
                {
                    break;
                }

                if (memcmp(pRef_info->scan_entry[i].bssid, pScan_info[ant_idx].scan_entry[j].bssid, 6) == 0)
                {
                    com_idx_per_ant[ant_idx] = j;
                    break;
                }
            }
        }

        for (uint8_t k = 0; k < pData->ant_port_count; k++)
        {
            if (com_idx_per_ant[k] == 0xff)
            {
                temp++;
            }
        }

        /*
         * if other antennas have the same BSSID as reference ant,
         * then the corresponding value in com_idx_per_ant[] array
         * is not 0xff and temp should be 1
         */
        if (temp == 1)
        {
            com_idx_per_ant[current_best_ant] = i;
            detect_done                       = 1;
            break;
        }
        else // if (temp > 1)
        {
            if (i == ANT_DETECT_MAX_SCAN_ENTRY - 1)
            {
                detect_done = -1;
                break;
            }
            else
            {
                memset((void *)&com_idx_per_ant[0], 0xff, pData->ant_port_count);
            }
        }
    }

    if (detect_done == 1)
    {
        (void)PRINTF("\nFind one common device\r\n");
        (void)PRINTF("List the info on every antenna for this common device\r\n");
        for (i = 0; i < pData->ant_port_count; i++)
        {
            PRINTF("Antenna %d:\r\n", i + 1);
            print_mac(pScan_info[i].scan_entry[com_idx_per_ant[i]].bssid);
            PRINTF(" \"%s\"\r\n", pScan_info[i].scan_entry[com_idx_per_ant[i]].ssid);
            PRINTF("\trssi[%d]: -%d dBm\r\n", i, pScan_info[i].scan_entry[com_idx_per_ant[i]].rssi);
            pScan_info[i].avg_rssi = pScan_info[i].scan_entry[com_idx_per_ant[i]].rssi;
        }
        wlan_get_best_two_ants(pData);
    }
    else
    {
        detect_done = -1;
    }

    return detect_done;
}

uint8_t device_count_to_check = PCB_DETECT_MODE_CHECK_DEVICE_COUNT;
static void wlan_evaluate_ant_by_specific_device(wlan_ant_detect_data_t *pData)
{
    unsigned int i;
    unsigned int j;
    uint16_t sum_rssi                = 0;
    wlan_ant_scan_info_t *pScan_info = &pData->scan_info[0];

    (void)PRINTF("\nEvaluate result:\r\n");
    (void)PRINTF("\t       avg_rssi\r\n");
    for (i = 0; i < pData->ant_port_count; i++)
    {
        sum_rssi = 0;
        (void)PRINTF("Antenna %d", i + 1);
        for (j = 0; j < device_count_to_check; j++)
        {
            sum_rssi += pScan_info[i].scan_entry[j].rssi;
        }
        pScan_info[i].avg_rssi = sum_rssi / device_count_to_check;
        PRINTF("\t-%d dBm\r\n", pScan_info[i].avg_rssi);
    }

    wlan_get_best_two_ants(pData);
}

wlan_ant_detect_data_t wlan_ant_detect_data;
static int __ant_detect_scan_cb(unsigned int count)
{
    wlan_ant_detect_data_t *pData = (wlan_ant_detect_data_t *)&wlan_ant_detect_data;
    wlan_ant_scan_info_t *pInfo   = (wlan_ant_scan_info_t *)&pData->scan_info[pData->current_ant - 1];

    if (count == 0U)
    {
        pInfo->avg_rssi = 0xff;
        (void)PRINTF("no networks found\r\n");
        goto end;
    }

    (void)PRINTF("%d network%s found\r\n", count, (count == 1U ? "" : "s"));
    if (pData->detect_mode == PCB_DETECT_MODE)
    {
        if (pData->current_ant == 1)
        {
            if (count == 1)
            {
                device_count_to_check = 1;
            }
            wlan_get_best_scan_info(pData, count);
        }
        else
        {
            wlan_get_specific_scan_info(pInfo, count);
        }
    }
    else
    {
        wlan_get_best_scan_info(pData, count);
    }

end:
    pInfo->scan_done = MTRUE;
    if (pData->detect_mode == PCB_DETECT_MODE)
    {
        if (count == 0 && pData->current_ant == 1)
        {
            pData->detect_done = -1;
        }
        else if (pData->current_ant == pData->ant_port_count && pInfo->entry_idx == device_count_to_check - 1)
        {
            (void)PRINTF("evaluate ant by specific device\r\n");
            wlan_evaluate_ant_by_specific_device(pData);
        }
    }
    else if (pData->current_ant == pData->ant_port_count)
    {
        if (wlan_evaluate_ant_by_common_device(pData) == 1)
        {
            pData->detect_done = 1;
        }
        else
        {
            wlan_evaluate_ant_by_avg_rssi(pData);
        }
    }

    return 0;
}

static int wlan_detect_ant_start_scan(wlan_scan_params_v2_t wlan_scan_param)
{
    char zero_bssid[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int ret;
    ret = wlan_scan_with_opt(wlan_scan_param);

    if (ret != 0)
    {
        (void)PRINTF("Error: scan request failed\r\n");
    }
    else
    {
        (void)PRINTF("Scan on ");
        if (wlan_scan_param.num_channels > 1)
        {
            (void)PRINTF("%d channels ", wlan_scan_param.num_channels);
        }
        else if (wlan_scan_param.num_channels == 1)
        {
            (void)PRINTF("channel %d ", wlan_scan_param.chan_list[0].chan_number);
        }
        else
        {
            (void)PRINTF("full channel ");
        }

        if (memcmp(&wlan_scan_param.bssid[0], &zero_bssid[0], 6))
        {
            (void)PRINTF("and for bssid ");
            print_mac((const char *)wlan_scan_param.bssid);
        }
        (void)PRINTF("scheduled...\r\n");
    }

    return ret;
}

static void wlan_detect_ant_set_mode(uint16_t best_ant, uint16_t next_best_ant)
{
    int ret;
    uint32_t ant_mode;
    uint16_t evaluate_time                   = 0x0;
    uint8_t evaluate_mode                    = 0xff;
    uint8_t evaluate_mode_lookup_table[3][3] =
    {
        /*   next best ant  */
        /* Ant1  Ant2  Ant3 */
        {0xff, 0, 2}, /* Ant1 */
        {0, 0xff, 1},
        /* Ant2 */    /* best ant */
        {2, 1, 0xff}, /* Ant3 */
    };

    evaluate_mode = evaluate_mode_lookup_table[best_ant - 1][next_best_ant - 1];
    ant_mode      = 0xffff;
    ret           = wlan_set_antcfg(ant_mode, evaluate_time, evaluate_mode);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Enable Antenna diversity with evaluate mode %d successful\r\n", evaluate_mode);
    }
    else
    {
        (void)PRINTF("Enable Antenna diversity with evaluate mode %d failed\r\n", evaluate_mode);
    }
}

static void wlan_start_detect_ant(void)
{
    int ret = -1;
    uint32_t ant_mode;
    uint16_t evaluate_time = 0x0;
    uint8_t evaluate_mode  = 0xff;
    uint16_t current_antenna;
    unsigned int i                       = 0;
    unsigned int chIdx                   = 0;
    wlan_ant_detect_data_t *pDetect_data = (wlan_ant_detect_data_t *)&wlan_ant_detect_data;
    wlan_scan_params_v2_t wlan_scan_param;

    (void)memset(&wlan_scan_param, 0, sizeof(wlan_scan_params_v2_t));
    if (pDetect_data->detect_mode == QUICK_DETECT_MODE)
    {
        wlan_scan_param.num_channels = 1;
    }
    else if (pDetect_data->detect_mode == NORMAL_DETECT_MODE)
    {
        wlan_scan_param.num_channels = pDetect_data->channel_list->num_channels;
        for (i = 0; i < pDetect_data->channel_list->num_channels; i++)
        {
            wlan_scan_param.chan_list[i].chan_number = pDetect_data->channel_list->chan_number[i];
        }
    }
    wlan_scan_param.cb = __ant_detect_scan_cb;

    do
    {
        pDetect_data->detect_done = 0;

        if (pDetect_data->detect_mode == QUICK_DETECT_MODE)
        {
            wlan_scan_param.chan_list[0].chan_number = pDetect_data->channel_list->chan_number[chIdx];
        }

        for (current_antenna = 1; current_antenna <= pDetect_data->ant_port_count; current_antenna++)
        {
            ant_mode = (1 << (current_antenna - 1));

            ret = wlan_set_antcfg(ant_mode, evaluate_time, evaluate_mode);
            if (ret == WM_SUCCESS)
            {
                (void)PRINTF("\nStart to evaluate antenna %d\r\n", current_antenna);
                pDetect_data->current_ant = current_antenna;

                if (pDetect_data->detect_mode == PCB_DETECT_MODE && current_antenna > 1)
                {
                    for (i = 0; i < device_count_to_check; i++)
                    {
                        scan_result_entry_t *pSpecInfo = &pDetect_data->scan_info[0].scan_entry[i];
                        (void)memcpy((void *)&wlan_scan_param.bssid[0], (const void *)&pSpecInfo->bssid[0],
                                     sizeof(wlan_scan_param.bssid));
                        wlan_scan_param.num_channels             = 1;
                        wlan_scan_param.chan_list[0].chan_number = pSpecInfo->channel;

                        pDetect_data->scan_info[current_antenna - 1].scan_done = MFALSE;
                        pDetect_data->scan_info[current_antenna - 1].entry_idx = i;
                        ret = wlan_detect_ant_start_scan(wlan_scan_param);
                        if (ret != 0)
                        {
                            break;
                        }
                        else
                        {
                            while (pDetect_data->scan_info[current_antenna - 1].scan_done != MTRUE)
                            {
                                OSA_TimeDelay(3);
                            }
                        }
                    }
                }
                else
                {
                    ret = wlan_detect_ant_start_scan(wlan_scan_param);
                    if (ret != 0)
                    {
                        break;
                    }
                    else
                    {
                        while (pDetect_data->scan_info[current_antenna - 1].scan_done != MTRUE)
                        {
                            OSA_TimeDelay(3);
                        }
                    }
                }
            }
            else
            {
                (void)PRINTF("\nError: Failed to set Antenna %d\r\n", current_antenna);
                break;
            }
        }

        if (ret != 0)
        {
            break;
        }

        while (pDetect_data->detect_done == 0)
        {
            // wait untill evaluation complete
            OSA_TimeDelay(2);
        }

        if (pDetect_data->detect_mode == QUICK_DETECT_MODE)
        {
            if (pDetect_data->detect_done == 1)
            {
                break;
            }
            else if (pDetect_data->detect_done == -1)
            {
                chIdx++;
                if (chIdx >= pDetect_data->channel_list->num_channels)
                {
                    break;
                }
            }
        }
        else
        {
            break;
        }
    }
    while (pDetect_data->detect_done != 1);

    if (pDetect_data->detect_done == 1)
    {
        (void)PRINTF("\nCurrently, best antenna is %d, next best antenna is %d\r\n", pDetect_data->best_ant,
                     pDetect_data->next_best_ant);
        wlan_detect_ant_set_mode(pDetect_data->best_ant, pDetect_data->next_best_ant);
    }
    else
    {
        ant_mode = 1;
        ret      = wlan_set_antcfg(ant_mode, evaluate_time, evaluate_mode);
        (void)PRINTF("\nFailed to detect antenna\r\n");
    }
}

static void dump_wlan_detect_ant_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-detect-ant <detect_mode> <ant_port_count> channel <channel> ... \r\n");
    (void)PRINTF("\r\n");
    (void)PRINTF("\t<detect_mode>: \r\n");
    (void)PRINTF("\t             0 -- normal detect mode: scan on all cfg channel list antenna by antenna.\r\n");
    (void)PRINTF("\t             1 -- quick detect mode: scan channel by channel on all antennas,\r\n");
    (void)PRINTF("\t                  and stop detect once detect done on one of channel in channel list.\r\n");
    (void)PRINTF("\t             2 -- PCB detect mode: scan on full channel list with PCB antenna firstly,\r\n");
    (void)PRINTF(
        "\t                  and select best 2 ex-APs, for the other antennas, just scan the specific channel\r\n");
    (void)PRINTF(
        "\t                  and the specific BSSID of 2 ex-APs; then compare scan RSSI and find best 2 ANTs.\r\n");
    (void)PRINTF("\t<ant_port_count>: \r\n");
    (void)PRINTF("\t                total count of antenna port, max 4\r\n");
    (void)PRINTF("\tchannel <channel> ...: \r\n");
    (void)PRINTF("\t                     You can specify one or more channels to scan.\r\n");
    (void)PRINTF("\t                     If configure more than one channel, please seperate with ','.\r\n");
    (void)PRINTF("\t                     If not specified, will scan on full channel list.\r\n");
    (void)PRINTF("Examples:\r\n");
    (void)PRINTF("wlan-detect-ant 1 3 channel 1\r\n");
    (void)PRINTF("wlan-detect-ant 1 3 channel 1,6,11,36,40,44\r\n");
    (void)PRINTF("wlan-detect-ant 1 3\r\n");
    (void)PRINTF("wlan-detect-ant 0 3\r\n");
    (void)PRINTF("wlan-detect-ant 2 3\r\n");
}

static void test_wlan_detect_ant(int argc, char **argv)
{
    unsigned start_msec;
    unsigned end_msec;
    int arg = 1;
    uint8_t i;
    uint8_t j;
    uint8_t detect_mode                       = 0;
    uint8_t antenna_port_count                = 0;
    wlan_ant_detect_data_t *pDetect_data      = NULL;
    cfg_scan_channel_list_t *cfg_channel_list = NULL;

    if (argc < 3 || argc == 4)
    {
        dump_wlan_detect_ant_usage();
        (void)PRINTF("Error: invalid number of arguments\r\n");
        return;
    }

    detect_mode = (uint8_t)atoi(argv[arg]);
    if (detect_mode != NORMAL_DETECT_MODE && detect_mode != QUICK_DETECT_MODE && detect_mode != PCB_DETECT_MODE)
    {
        dump_wlan_detect_ant_usage();
        (void)PRINTF("Error: invalid detect_mode\r\n");
        return;
    }

    arg                = 2;
    antenna_port_count = (uint8_t)atoi(argv[arg]);
    if (antenna_port_count > MAX_ANTENNA_PORT_NUM || antenna_port_count < 3)
    {
        dump_wlan_detect_ant_usage();
        (void)PRINTF("Error: invalid antenna_port_count\r\n");
        return;
    }

    arg = 3;
    (void)memset(&wlan_ant_detect_data, 0x00, sizeof(wlan_ant_detect_data_t));
    pDetect_data = (wlan_ant_detect_data_t *)&wlan_ant_detect_data;
    for (i = 0; i < antenna_port_count; i++)
    {
        wlan_ant_scan_info_t *pInfo = (wlan_ant_scan_info_t *)&pDetect_data->scan_info[i];
        for (j = 0; j < ANT_DETECT_MAX_SCAN_ENTRY; j++)
        {
            pInfo->scan_entry[j].rssi = 0xff;
        }
    }

    pDetect_data->detect_mode    = detect_mode;
    pDetect_data->ant_port_count = antenna_port_count;
    if (pDetect_data->detect_mode == PCB_DETECT_MODE)
    {
        goto start_detect;
    }

    cfg_channel_list = (cfg_scan_channel_list_t *)OSA_MemoryAllocate(sizeof(cfg_scan_channel_list_t));
    if (cfg_channel_list == NULL)
    {
        (void)PRINTF("Failed to alloc buffer for channel list\r\n");
        return;
    }
    (void)memset(cfg_channel_list, 0, sizeof(cfg_scan_channel_list_t));

    if (argc > 4 && string_equal("channel", argv[arg]))
    {
        if (get_channel_list(argv[arg + 1], (uint8_t *)&cfg_channel_list->num_channels,
                             (uint8_t *)&cfg_channel_list->chan_number[0], ',') != false)
        {
            (void)PRINTF(
                "Error: invalid channel"
                " argument\n");
            OSA_MemoryFree(cfg_channel_list);
            return;
        }
    }
    else
    {
        // get full channel list
        wlan_chanlist_t chanlist;
        (void)memset(&chanlist, 0x00, sizeof(wlan_chanlist_t));
        int rv = wlan_get_chanlist(&chanlist);
        if (rv != WM_SUCCESS)
        {
            (void)PRINTF("Unable to get channel list configuration\r\n");
            OSA_MemoryFree(cfg_channel_list);
            return;
        }
        else
        {
            uint8_t i                      = 0;
            cfg_channel_list->num_channels = chanlist.num_chans;
            for (i = 0; i < chanlist.num_chans; i++)
            {
                cfg_channel_list->chan_number[i] = chanlist.chan_info[i].chan_num;
            }
        }
    }

    pDetect_data->channel_list = cfg_channel_list;

start_detect:
    start_msec = OSA_TicksToMsec(OSA_TicksGet());
    (void)PRINTF("%d: Start to detect ant\r\n", start_msec);
    wlan_start_detect_ant();
    if (cfg_channel_list)
    {
        OSA_MemoryFree(cfg_channel_list);
    }
    end_msec = OSA_TicksToMsec(OSA_TicksGet());
    (void)PRINTF("%d: End of detect ant\r\n", end_msec);
    (void)PRINTF("It cost %dms to detect ant\r\n", end_msec - start_msec);
}
#endif

static struct cli_command tests[] =
{
    {"wlan-thread-info", NULL, test_wlan_thread_info},
    {"wlan-net-stats", NULL, test_wlan_net_stats},
    {"wlan-set-mac", "<MAC_Address>", test_wlan_set_mac_address},
    {"wlan-scan", NULL, test_wlan_scan},
    {"wlan-scan-opt", "ssid <ssid> bssid ...", test_wlan_scan_opt},
    {"wlan-add", "<profile_name> ssid <ssid> bssid...", test_wlan_add},
    {"wlan-remove", "<profile_name>", test_wlan_remove},
    {"wlan-list", NULL, test_wlan_list},
    {"wlan-connect", "<profile_name>", test_wlan_connect},
    {"wlan-connect-opt", "<profile_name> ...", test_wlan_connect_opt},
    {"wlan-reassociate", NULL, test_wlan_reassociate},
    {"wlan-start-network", "<profile_name>", test_wlan_start_network},
    {"wlan-stop-network", NULL, test_wlan_stop_network},
    {"wlan-disconnect", NULL, test_wlan_disconnect},
    {"wlan-uap-disconnect-sta", "<mac address>", test_wlan_uap_disconnect_sta},
    {"wlan-stat", NULL, test_wlan_stat},
    {"wlan-info", NULL, test_wlan_info},
    {"wlan-address", NULL, test_wlan_address},
    {"wlan-get-uap-channel", NULL, test_wlan_get_uap_channel},
    {"wlan-get-uap-sta-list", NULL, test_wlan_get_uap_sta_list},
    {"wlan-ieee-ps", "<0/1>", test_wlan_ieee_ps},
    {"wlan-set-ps-cfg", "<null_pkt_interval>", test_wlan_set_ps_cfg},
    {"wlan-deep-sleep-ps", "<0/1>", test_wlan_deep_sleep_ps},
    {"wlan-get-beacon-interval", NULL, test_wlan_get_beacon_interval},
#if !CONFIG_WPA_SUPP
#if (CONFIG_WNM_PS)
    {"wlan-wnm-ps", "<0/1> <sleep_interval>", test_wlan_wnm_ps},
#endif
#endif
#if CONFIG_WIFI_MAX_CLIENTS_CNT
    {"wlan-set-max-clients-count", "<max clients count>", test_wlan_set_max_clients_count},
#endif
    {"wlan-rts", "<sta/uap> <rts threshold>", test_wlan_set_rts},
#if CONFIG_WIFI_FRAG_THRESHOLD
    {"wlan-frag", "<sta/uap> <fragment threshold>", test_wlan_set_frag},
#endif
#if CONFIG_11K
    {"wlan-host-11k-enable", "<0/1>", test_wlan_host_11k_cfg},
    {"wlan-host-11k-neighbor-req", "[ssid <ssid>]", test_wlan_host_11k_neighbor_request},
#endif
#if CONFIG_11V
    {"wlan-host-11v-bss-trans-query", "<0..16>", test_wlan_host_11v_bss_trans_query},
#endif
#if CONFIG_DRIVER_MBO
    {"wlan-mbo-enable", "<0/1>", test_wlan_mbo_cfg},
    {
        "wlan-mbo-nonprefer-ch", "<ch0> <Preference0: 0/1/255> <ch1> <Preference1: 0/1/255>",
        test_wlan_mbo_non_prefer_chs
    },
#endif
#if CONFIG_WPA_SUPP
#if (CONFIG_11AX && defined(CONFIG_MBO))
    {
        "wlan-mbo-nonprefer-ch", "\"<oper_class>:<chan>:<preference>:<reason> <oper_class>:<chan>:<preference>:<reason>\"",
        test_wlan_mbo_non_prefer_chs
    },
    {"wlan-mbo-set-cell-capa", "<cell capa: 1/2/3(default)>", test_wlan_mbo_set_cell_capa},
    {"wlan-mbo-set-oce", "<oce: 1(default)/2>", test_wlan_mbo_set_oce},
#endif
    {"wlan-set-okc", "<okc: 0(default)/1>", test_wlan_set_okc},
    {"wlan-pmksa-list", NULL, test_wlan_pmksa_list},
    {"wlan-pmksa-flush", NULL, test_wlan_pmksa_flush},
    {"wlan-set-scan-interval", "<scan_int: in seconds>", test_wlan_set_scan_interval},
#endif
#if (CONFIG_11MC) || (CONFIG_11AZ)
    {"wlan-unassoc-ftm-cfg", "<set_get> <on_off>", test_wlan_unassoc_ftm_cfg},
    {"wlan-ftm-ctrl", "<action> <loop_cnt> <peer_mac> <channel>", test_wlan_ftm_ctrl},
    {
        "wlan-11mc-nego-cfg", "<burst_inst> <burst_dur> <min_delta> <asap> <ftm_per_burst> <bw> <burst_period>",
        test_wlan_11mc_nego_cfg
    },
    {"wlan-loc-cfg", "<lci_req> <latit> <longi> <altit> <lat_uncert> <lon_uncert> <alt_uncert>", test_wlan_loc_cfg},
    {"wlan-civ-cfg", "<civ_loc> <civ_loc_type> <country_code> <civ_add_type>", test_wlan_civ_cfg},
    {
        "wlan-11az-rang-cfg", "<protocol> <format_bw> <num_measurements> <measurement_freq> <i2r_sts> <r2i_sts> <i2r_lmr>",
        test_wlan_11az_rang_cfg
    },
#endif
#if CONFIG_UAP_STA_MAC_ADDR_FILTER
    {"wlan-sta-filter", " <filter mode> [<mac address list>]", test_wlan_set_sta_filter},
#endif
#if CONFIG_WIFI_GET_LOG
    {"wlan-get-log", "<sta/uap> <ext>", test_wlan_get_log},
#endif
#if CONFIG_WIFI_TX_PER_TRACK
    {"wlan-tx-pert", "<0/1> <STA/UAP> <p> <r> <n>", test_wlan_tx_pert},
#endif
#if CONFIG_ROAMING
    {"wlan-roaming", "<0/1> <rssi_threshold>", test_wlan_roaming},
#endif
    {"wlan-multi-mef", "<ping/arp/multicast/del> [<action>]", test_wlan_set_multiple_mef_config},
#if CONFIG_HOST_SLEEP
#ifdef RW610
    {"wlan-wakeup-condition", "<mef/wowlan wake_up_conds>", test_wlan_wakeup_condition},
#if !defined(CONFIG_WIFI_BLE_COEX_APP)
#ifdef __ZEPHYR__
    {"wlan-auto-host-sleep", "<enable> <periodic>", test_wlan_auto_host_sleep},
#else
    {"wlan-auto-host-sleep", "<enable> <mode> <rtc_timer> <periodic>", test_wlan_auto_host_sleep},
#endif
#endif
#else
    {"enable-ns-offload", NULL, test_wlan_ns_offload},
    {"wlan-auto-arp", NULL, test_wlan_auto_arp},
    {
        "wlan-add-packet-filter", "0/1 <patterns number> <ptn_len> <pkt_offset> <ptn> ...........",
        test_wlan_add_packet_filter
    },
    {"wlan-host-sleep", "<0/1> mef/wowlan <wake_up_conds>", test_wlan_host_sleep},
#endif /*RW610*/
#endif /*CONFIG_HOST_SLEEP*/
#if CONFIG_SEND_HOSTCMD
    {"wlan-send-hostcmd", NULL, test_wlan_send_hostcmd},
#endif
#if defined(RW610) || defined(SD9177)
    {"wlan-ext-coex-uwb", NULL, test_wlan_ext_coex_uwb},
#endif
#if !defined(SD8801) && !defined(RW610)
#if CONFIG_11AC
    {"wlan-set-uap-bandwidth", "<1/2/3> 1:20 MHz 2:40MHz 3:80MHz", test_wlan_set_uap_bandwidth},
#else
    {"wlan-set-uap-bandwidth", "<1/2> 1:20 MHz 2:40MHz", test_wlan_set_uap_bandwidth},
#endif
#endif
    {"wlan-set-uap-hidden-ssid", "<0/1/2>", test_wlan_set_uap_hidden_ssid},
#ifdef SD8801
    {"wlan-8801-enable-ext-coex", NULL, test_wlan_8801_enable_ext_coex},
    {"wlan-8801-get-ext-coex-stats", NULL, test_wlan_8801_ext_coex_stats},
#endif
#if CONFIG_WIFI_EU_CRYPTO
    {"wlan-eu-crypto-rc4", "<EncDec>", test_wlan_eu_crypto_rc4},
    {"wlan-eu-crypto-aes-wrap", "<EncDec>", test_wlan_eu_crypto_aes_wrap},
    {"wlan-eu-crypto-aes-ecb", "<EncDec>", test_wlan_eu_crypto_aes_ecb},
    {"wlan-eu-crypto-ccmp-128", "<EncDec>", test_wlan_eu_crypto_ccmp_128},
#if !defined(SD8978)
    {"wlan-eu-crypto-ccmp-256", "<EncDec>", test_wlan_eu_crypto_ccmp_256},
    {"wlan-eu-crypto-gcmp-128", "<EncDec>", test_wlan_eu_crypto_gcmp_128},
    {"wlan-eu-crypto-gcmp-256", "<EncDec>", test_wlan_eu_crypto_gcmp_256},
#endif
#endif
#if CONFIG_WIFI_MEM_ACCESS
    {"wlan-mem-access", "<memory_address> [<value>]", test_wlan_mem_access},
#endif
#if CONFIG_WIFI_BOOT_SLEEP
    {"wlan-boot-sleep", "<0/1> 0:Disable 1:Enable", test_wlan_boot_sleep},
#endif
#if CONFIG_HEAP_STAT
    {"heap-stat", NULL, test_heap_stat},
#endif
#if CONFIG_HEAP_DEBUG
    {"wlan-os-mem-stat", NULL, test_wlan_os_mem_stat},
#endif
#if CONFIG_MULTI_CHAN
    {"wlan-set-mc-policy", "<0/1>(disable/enable)", test_wlan_set_multi_chan_status},
    {"wlan-get-mc-policy", NULL, test_wlan_get_multi_chan_status},
    {
        "wlan-set-drcs",
        "<channel_time> <switch_time> <undoze_time> <mode> [<channel_time> <switch_time> <undoze_time> <mode>]",
        test_wlan_set_drcs_cfg
    },
    {"wlan-get-drcs", NULL, test_wlan_get_drcs_cfg},
#endif
#if (CONFIG_11R)
    {"wlan-ft-roam", "<bssid> <channel>", test_wlan_ft_roam},
#endif
#ifndef RW610
    {"wlan-set-antcfg", "<ant mode> [evaluate_time]", wlan_antcfg_set},
#else
    {"wlan-set-antcfg", "<ant_mode> <evaluate_time> <evaluate_mode>", wlan_antcfg_set},
#endif
    {"wlan-get-antcfg", NULL, wlan_antcfg_get},
#if CONFIG_SCAN_CHANNEL_GAP
    {"wlan-scan-channel-gap", "<channel_gap_value>", test_wlan_set_scan_channel_gap},
#endif
#if CONFIG_WMM
    {"wlan-wmm-stat", "<bss_type>", test_wlan_wmm_tx_stats},
#endif
#if defined(RW610) && (CONFIG_WIFI_RESET)
    {"wlan-reset", NULL, test_wlan_reset},
#endif
    {"wlan-set-regioncode", "<region-code>", test_wlan_set_regioncode},
    {"wlan-get-regioncode", NULL, test_wlan_get_regioncode},
    {"wlan-11d-enable", "<sta/uap> <0/1>", test_wlan_11d_enable},
#if CONFIG_ECSA
    {
        "wlan-uap-set-ecsa-cfg", "<block_tx> <oper_class> <new_channel> <switch_count> <bandwidth>",
        test_wlan_uap_set_ecsa_cfg
    },
#endif
#if CONFIG_CSI
    {"wlan-csi-cfg", NULL, test_wlan_csi_cfg},
    {
        "wlan-set-csi-param-header",
        " <sta/uap> <csi_enable> <head_id> <tail_id> <chip_id> <band_config> <channel> <csi_monitor_enable> <ra4us>",
        test_wlan_set_csi_param_header
    },
    {"wlan-set-csi-filter", "<opt> <macaddr> <pkt_type> <type> <flag>", test_wlan_set_csi_filter},
#endif
#if CONFIG_TX_RX_HISTOGRAM
    {"wlan-txrx-histogram", "<action> <enable>", test_wlan_txrx_histogram},
#endif
#if CONFIG_SUBSCRIBE_EVENT_SUPPORT
    {"wlan-subscribe-event", "<action> <type> <value> <freq>", test_wlan_subscribe_event},
#endif
#if CONFIG_WIFI_REG_ACCESS
    {"wlan-reg-access", "<type> <offset> [value]", test_wlan_reg_access},
#endif
#if CONFIG_WMM_UAPSD
    {"wlan-uapsd-enable", "<uapsd_enable>", test_wlan_set_wmm_uapsd},
    {"wlan-uapsd-qosinfo", "<qos_info>", test_wlan_wmm_uapsd_qosinfo},
    {"wlan-uapsd-sleep-period", "<sleep_period>", test_wlan_sleep_period},
#endif
#if CONFIG_TX_AMPDU_PROT_MODE
    {"wlan-tx-ampdu-prot-mode", "<mode>", test_wlan_tx_ampdu_prot_mode},
#endif
#if (CONFIG_11K) || (CONFIG_11V) || (CONFIG_11R) || (CONFIG_ROAMING)
    {"wlan-rssi-low-threshold", "<threshold_value>", test_wlan_rssi_low_threshold},
#endif
#if CONFIG_RX_ABORT_CFG
    {"wlan-rx-abort-cfg", NULL, test_wlan_rx_abort_cfg},
#endif
#if CONFIG_RX_ABORT_CFG_EXT
    {
        "wlan-set-rx-abort-cfg-ext", "enable <enable> margin <margin> ceil <ceil_thresh> floor <floor_thresh>",
        test_wlan_set_rx_abort_cfg_ext
    },
    {"wlan-get-rx-abort-cfg-ext", NULL, test_wlan_get_rx_abort_cfg_ext},
#endif
#if CONFIG_CCK_DESENSE_CFG
    {"wlan-cck-desense-cfg", NULL, test_wlan_cck_desense_cfg},
#endif
#if (CONFIG_WPS2) || (CONFIG_WPA_SUPP_WPS)
    {"wlan-generate-wps-pin", NULL, test_wlan_wps_generate_pin},
    {"wlan-start-wps-pbc", NULL, test_wlan_start_wps_pbc},
    {"wlan-start-wps-pin", "<8 digit pin>", test_wlan_start_wps_pin},
#endif
#if CONFIG_WPA_SUPP_WPS
    {"wlan-wps-cancel", NULL, test_wlan_wps_cancel},
#if CONFIG_WPA_SUPP_AP
    {"wlan-start-ap-wps-pbc", NULL, test_wlan_start_ap_wps_pbc},
    {"wlan-start-ap-wps-pin", "<8 digit pin>", test_wlan_start_ap_wps_pin},
    {"wlan-wps-ap-cancel", NULL, test_wlan_wps_ap_cancel},
#endif
#endif
#if CONFIG_WPA_SUPP_DPP
    {"wlan-dpp-configurator-add", NULL, test_wlan_dpp_configurator_add},
    {
        "wlan-dpp-configurator-params", " conf=<sta-dpp/ap-dpp> ssid=<ascii> configurator=<id>",
        test_wlan_dpp_configurator_params
    },
    {"wlan-dpp-mud-url", "https://...", test_wlan_dpp_mud_url},
    {"wlan-dpp-bootstrap-gen", "type=<qrcode> chan=<op>/<ch> mac=<addr>", test_wlan_dpp_bootstrap_gen},
    {"wlan-dpp-bootstrap-get-uri", "<bootstrap_gen id>", test_wlan_dpp_bootstrap_get_uri},
    {"wlan-dpp-qr-code", "<DPP:...>", test_wlan_dpp_qr_code},
    {"wlan-dpp-auth-init", " peer=<id> role=<enrollee/configurator>", test_wlan_dpp_auth_init},
    {"wlan-dpp-listen", "<frequency>...", test_wlan_dpp_listen},
    {"wlan-dpp-stop-listen", NULL, test_wlan_dpp_stop_listen},
    {"wlan-dpp-pkex-add", " own=<bootstrap_id> identifier=<string> code=<string>", test_wlan_dpp_pkex_add},
    {"wlan-dpp-chirp", " own=<bootstrap id> listen=<freq>...", test_wlan_dpp_chirp},
    {"wlan-dpp-reconfig", "<network id> ...", test_wlan_dpp_reconfig},
    {
        "wlan-dpp-configurator-sign", " conf=<sta-dpp/ap-dpp> ssid=<ascii> configurator=<id>",
        test_wlan_dpp_configurator_sign
    },
#endif
#if CONFIG_NET_MONITOR
    {"wlan-net-monitor-cfg", NULL, test_wlan_net_monitor_cfg},
    {"wlan-set-monitor-filter", "<opt> <macaddr>", test_wlan_set_monitor_filter},
    {
        "wlan-set-monitor-param", "<action> <monitor_activity> <filter_flags> <radio_type> <chan_number>",
        test_wlan_set_monitor_param
    },
#endif
#if CONFIG_TSP
    {
        "wlan-set-tsp-cfg",
        "<enable> <backoff> <highThreshold> <lowThreshold> <dutycycstep> <dutycycmin> <highthrtemp> <lowthrtemp>",
        test_wlan_set_tsp_cfg
    },
    {"wlan-get-tsp-cfg", NULL, test_wlan_get_tsp_cfg},
#endif
#if CONFIG_CPU_LOADING
    {"wlan-cpu-loading", "start <start> sample_loops <number> sample_period <period>", test_wlan_cpu_loading},
#endif
    {"wlan-get-signal", NULL, test_wlan_get_signal},
    {"wlan-set-bandcfg", NULL, test_wlan_set_bandcfg},
    {"wlan-get-bandcfg", NULL, test_wlan_get_bandcfg},
#if (CONFIG_IPS)
    {"wlan-set-ips", "<option>", test_wlan_set_ips},
#endif
#if CONFIG_11AX
    {
        "wlan-set-debug-htc",
        "<count> <vht> <he> <rxNss> <channelWidth> <ulMuDisable> <txNSTS> <erSuDisable> <erSuDisable> <erSuDisable>",
        test_wlan_set_debug_htc
    },
    {"wlan-enable-disable-htc", "<option>", test_wlan_enable_disable_htc},
#endif
    {"wlan-set-su", "<0/1>", test_wlan_set_su},
#if CONFIG_WIFI_FORCE_RTS
    {"wlan-set-forceRTS", "<0/1>", test_wlan_set_forceRTS},
#endif
#if CONFIG_11AX
#if CONFIG_MMSF
    {"wlan-set-mmsf", "<enable> <Density> <MMSF>", test_wlan_set_mmsf},
    {"wlan-get-mmsf", NULL, test_wlan_get_mmsf},
#endif
#endif
#if CONFIG_TURBO_MODE
    {"wlan-get-turbo-mode", "<STA/UAP>", test_wlan_get_turbo_mode},
    {"wlan-set-turbo-mode", "<STA/UAP> <mode>", test_wlan_set_turbo_mode},
#endif
    {"wlan-set-multiple-dtim", "<value>", test_wlan_set_multiple_dtim},
#if CONFIG_CLOUD_KEEP_ALIVE
    {"wlan-cloud-keep-alive", "<start/stop/reset>", test_wlan_cloud_keep_alive},
    {"wlan_tcp_client", "dst_ip <dst_ip> src_port <src_port> dst_port <dst_port>", test_wlan_tcp_client},
#endif
    {"wlan-set-country", "<country_code_str>", test_wlan_set_country_code},
    {"wlan-set-country-ie-ignore", "<0/1>", test_wlan_set_country_ie_ignore},
#if CONFIG_COEX_DUTY_CYCLE
    {
        "wlan-single-ant-duty-cycle", "<enable/disable> [<Ieee154Duration> <TotalDuration>]",
        test_wlan_single_ant_duty_cycle
    },
    {
        "wlan-dual-ant-duty-cycle", "<enable/disable> [<Ieee154Duration> <TotalDuration> <Ieee154FarRangeDuration>]",
        test_wlan_dual_ant_duty_cycle
    },
#endif
#if CONFIG_EXTERNAL_COEX_PTA
    {
        "wlan-external-coex-pta",
        "enable <PTA/WCI-2/WCI-2 GPIO> ExtWifiBtArb <enable/disable> PolGrantPin <high/low> PriPtaInt <enable/disable> "
        "StateFromPta <state pin/ priority pin/ state input disable> SampTiming <Sample timing> InfoSampTiming <Sample "
        "timing> "
        "TrafficPrio <enable/disable> CoexHwIntWic <enable/disable>",
        test_wlan_external_coex_pta
    },
#endif
#if CONFIG_IMD3_CFG
    {"wlan-imd3-cfg", "<enable>", test_wlan_imd3_cfg},
#endif
#if CONFIG_AUTO_RECONNECT
    {"wlan-auto-reconnect", "<0/1/2> [<reconnect counter> <reconnect interval> <flags>]", test_wlan_auto_reconnect},
#endif
#if (CONFIG_WIFI_IND_RESET) && (CONFIG_WIFI_IND_DNLD)
    {"wlan-set-indrstcfg", "<mode> <gpio_pin>", test_set_indrst_cfg},
    {"wlan-get-indrstcfg", NULL, test_get_indrst_cfg},
    {"wlan-independent-reset", "<mode>", test_wlan_independent_reset},
#endif
#if CONFIG_INACTIVITY_TIMEOUT_EXT
    {"wlan-sta-inactivityto", "<n> <m> <l> [k] [j]", test_wlan_sta_inactivityto},
#endif
#ifdef RW610
    {"wlan-get-temperature", NULL, test_wlan_get_temperature},
#endif
#if CONFIG_AUTO_NULL_TX
    {"wlan-auto-null-tx", "<sta/uap> <start/stop>", test_wlan_auto_null_tx},
#endif
#if defined(RW610) && (CONFIG_ANT_DETECT)
    {"wlan-detect-ant", "<detect_mode> <ant_port_count> channel <channel> ...", test_wlan_detect_ant},
#endif
#if CONFIG_WIFI_RECOVERY
    {"wlan-recovery-test", NULL, test_wlan_recovery_test},
#endif
};

/* Register our commands with the MTF. */
int wlan_cli_init(void)
{
    int i;

    i = wlan_basic_cli_init();
    if (i != WLAN_ERROR_NONE)
    {
        return i;
    }

    if (cli_register_commands(tests, (int)(sizeof(tests) / sizeof(struct cli_command))) != 0)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

/* Unregister our commands with the MTF. */
int wlan_cli_deinit(void)
{
    int i;

    if (cli_unregister_commands(tests, (int)(sizeof(tests) / sizeof(struct cli_command))) != 0)
    {
        return -WM_FAIL;
    }

    i = wlan_basic_cli_deinit();
    if (i != WLAN_ERROR_NONE)
    {
        return i;
    }

    return WM_SUCCESS;
}
