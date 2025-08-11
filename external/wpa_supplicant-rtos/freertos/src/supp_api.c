/** @file supp_api.c
 *
 *  @brief  This file provides wpa supplicant APIs.
 *
 *  Copyright 2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "rtthread.h"
#include "fsl_os_abstraction.h"

#include "includes.h"
#include "utils/common.h"
#include "common/defs.h"
#include "common/ptksa_cache.h"
#include "wpa_supplicant/config.h"
#include "wpa_supplicant_i.h"
#include "driver_i.h"
#include "ap.h"
#include "crc32.h"

#include "supp_main.h"
#include "supp_api.h"
#include "wpa_supplicant/bss.h"
#include "wpa_supplicant/wps_supplicant.h"
#include "rsn_supp/wpa.h"
#include "wpa_supplicant/wnm_sta.h"
#include "eap_defs.h"
#include "eap.h"
#include "crypto/tls.h"
#include "wpa_supplicant/scan.h"
#include "wpa_supplicant/sme.h"
#include "rsn_supp/pmksa_cache.h"
#include "stdlib.h"
#include "ap/ap_config.h"
#include "ap/hostapd.h"
#include "ap/sta_info.h"

#if CONFIG_HOSTAPD
#include "ap/hostapd.h"
#include "ap/ap_config.h"
#include "ap/wps_hostapd.h"
#include "ap/sta_info.h"
#include "ap/ap_drv_ops.h"
#endif

#ifdef CONFIG_DPP
#include "ap/dpp_hostapd.h"
#include "wpa_supplicant/dpp_supplicant.h"
#endif

#if CONFIG_11R
#include "wpa_i.h"
#endif

#define EAP_TTLS_AUTH_PAP      1
#define EAP_TTLS_AUTH_CHAP     2
#define EAP_TTLS_AUTH_MSCHAP   4
#define EAP_TTLS_AUTH_MSCHAPV2 8

#define UAP_DTIM_PERIOD 1

enum supp_thread_state
{
    SUPP_THREAD_STOPPED = 0,
    SUPP_THREAD_RUNNING,
};

unsigned char bandwidth = 2;
unsigned char h_hidden_ssid = 0;
unsigned short h_beacon_int = 100;
int h_max_num_sta = 8;

int cli_main(int, const char **);

OSA_SEMAPHORE_HANDLE_DEFINE(wpaSuppReadySemaphoreHandle);

#if CONFIG_HOSTAPD
OSA_SEMAPHORE_HANDLE_DEFINE(hostapdReadySemaphoreHandle);
#endif
#ifndef __ZEPHYR__
extern OSA_EVENT_HANDLE_DEFINE(supplicant_event_Handle);
#endif
extern struct wpa_global *global;

OSA_MUTEX_HANDLE_DEFINE(wpa_supplicant_mutex);

struct wpa_supp_api_ctrl
{
    const struct netif *dev;
    enum requested_ops requested_op;
    enum supp_thread_state supp_thread_state;
};

static struct wpa_supp_api_ctrl wpa_supp_api_ctrl;
static void wpa_supp_scan_res_fail_handler(struct wpa_supplicant *wpa_s);

static inline struct wpa_supplicant *get_wpa_s_handle(const struct netif *dev)
{
    struct wpa_supplicant *wpa_s = NULL;
    int ret = OSA_SemaphoreWait((osa_semaphore_handle_t)wpaSuppReadySemaphoreHandle, osaWaitForever_c);
    char ifname[NETIF_NAMESIZE];

    if (ret)
    {
        wpa_printf(MSG_DEBUG, "%s: WPA supplicant not ready: %d", __func__, ret);
        return NULL;
    }

    OSA_SemaphorePost((osa_semaphore_handle_t)wpaSuppReadySemaphoreHandle);

#ifdef __ZEPHYR__
    const struct device *dev_temp = NULL;
    dev_temp = net_if_get_device((struct net_if *)dev);
    strncpy(ifname, dev_temp->name, NETIF_NAMESIZE - 1);
    ifname[NETIF_NAMESIZE - 1] = '\0';
#else
    (void)netifapi_netif_index_to_name(dev->num + 1, ifname);
#endif
    rt_kprintf("%s %d ifname: %s,dev->num=%d\n",__func__,__LINE__, ifname,dev->num + 1);
    wpa_s = wpa_supplicant_get_iface(global, ifname);
    if (!wpa_s)
    {
        rt_kprintf("%s: Unable to get wpa_s handle for %s,wpa_s=0x%p,dev=%p\n", __func__, ifname,wpa_s,dev);
        RT_ASSERT(0);
        return NULL;
    }

    return wpa_s;
}

#if CONFIG_HOSTAPD
static inline struct hostapd_iface *get_hostapd_handle(const struct netif *dev)
{
    struct hostapd_iface *hapd_s = NULL;
    int ret = OSA_SemaphoreWait((osa_semaphore_handle_t)hostapdReadySemaphoreHandle, osaWaitForever_c);
    char ifname[NETIF_NAMESIZE];

    if (ret)
    {
        wpa_printf(MSG_DEBUG, "%s: HOSTAPD not ready: %d", __func__, ret);
        return NULL;
    }

    OSA_SemaphorePost((osa_semaphore_handle_t)hostapdReadySemaphoreHandle);

#ifdef __ZEPHYR__
    const struct device *dev_temp = NULL;
    dev_temp = net_if_get_device((struct net_if *)dev);
    strncpy(ifname, dev_temp->name, NETIF_NAMESIZE - 1);
    ifname[NETIF_NAMESIZE - 1] = '\0';
#else
    (void)netifapi_netif_index_to_name(dev->num + 1, ifname);
#endif

    hapd_s = hostapd_get_interface(ifname);
    if (!hapd_s)
    {
        wpa_printf(MSG_DEBUG, "%s: Unable to get hapd_s handle for %s", __func__, ifname);
        return NULL;
    }

    return hapd_s;
}
#endif

int wpa_supp_req_status(enum requested_ops ops)
{
    int status = -1;
    struct wpa_supplicant *wpa_s;
    struct wpa_supp_api_ctrl *ctrl = &wpa_supp_api_ctrl;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (ops == START)
    {
        struct hostapd_iface *hapd_s;

        hapd_s = get_hostapd_handle(ctrl->dev);
        if (!hapd_s)
        {
            status = -1;
            goto out;
        }

        if (hapd_s->state == HAPD_IFACE_ENABLED)
            status = 0;
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(ctrl->dev);
        if (!wpa_s)
        {
            goto out;
        }

        if ((ctrl->requested_op == CONNECT) || (ctrl->requested_op == START) || (ctrl->requested_op == RECONNECT))
        {
            if (wpa_s->wpa_state == WPA_COMPLETED)
                status = 0;
        }
#if CONFIG_HOSTAPD
    }
#endif

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
    return status;
}

static int *freq_range_to_channel_list(struct wpa_supplicant *wpa_s, char *val)
{
    struct wpa_freq_range_list ranges;
    int *freqs = NULL;
    struct hostapd_hw_modes *mode;
    u16 i;

    if (wpa_s->hw.modes == NULL)
        return NULL;

    os_memset(&ranges, 0, sizeof(ranges));
    if (freq_range_list_parse(&ranges, val) < 0)
        return NULL;

    for (i = 0; i < wpa_s->hw.num_modes; i++)
    {
        int j;

        // Skip HOSTAPD_MODE_IEEE80211B
        if (i == HOSTAPD_MODE_IEEE80211B)
        {
            continue;
        }

        mode = &wpa_s->hw.modes[i];
        for (j = 0; j < mode->num_channels; j++)
        {
            unsigned int freq;

            if (mode->channels[j].flag & HOSTAPD_CHAN_DISABLED)
                continue;

            freq = mode->channels[j].freq;
            if (!freq_range_list_includes(&ranges, freq))
                continue;

            int_array_add_unique(&freqs, freq);
        }
    }

    os_free(ranges.range);
    return freqs;
}

int wpa_supp_scan(const struct netif *dev, wlan_scan_params_v2_t *params)
{
    unsigned int manual_scan_passive  = 0;
    unsigned int manual_scan_use_id   = 0;
    unsigned int manual_scan_only_new = 1;
    unsigned int scan_only            = 0;
    unsigned int scan_id_count        = 0;
    int scan_id[MAX_SCAN_ID]          = {0};
    void (*scan_res_handler)(struct wpa_supplicant * wpa_s, struct wpa_scan_results * scan_res);
    int *manual_scan_freqs      = NULL;
    struct wpa_ssid_value *ssid = NULL;
    unsigned int ssid_count     = 0;
    char frequency[256]         = {0};
    char *pos                   = frequency;
    struct wpa_supplicant *wpa_s;
    int ret = 0, i, freq;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    if (wpa_s->wpa_state == WPA_INTERFACE_DISABLED)
    {
        ret = -1;
        goto out;
    }

    if (radio_work_pending(wpa_s, "scan"))
    {
        wpa_printf(MSG_DEBUG, "Pending scan scheduled - reject new request");
        ret = -1;
        goto out;
    }

    if (params)
    {
        if (params->scan_only)
            scan_only = 1;

        if (params->num_channels)
        {
            for (i = 0; i < params->num_channels; i++)
            {
                freq = ieee80211_chan_to_freq(NULL, 81, params->chan_list[i].chan_number);
                if (freq <= 0)
                {
                    freq = ieee80211_chan_to_freq(NULL, 128, params->chan_list[i].chan_number);
                }
                if (freq <= 0)
                {
                    wpa_printf(MSG_DEBUG, "Invalid channel %d", params->chan_list[i].chan_number);
                    ret = -1;
                    goto out;
                }
                if (i == (params->num_channels - 1))
                {
                    (void)sprintf(pos, "%d", freq);
                }
                else
                {
                    ret = sprintf(pos, "%d,", freq);
                    if (ret < 0)
                    {
                        ret = -1;
                        goto out;
                    }
                    pos += ret;
                }
            }

            manual_scan_freqs = freq_range_to_channel_list(wpa_s, frequency);
            if (manual_scan_freqs == NULL)
            {
                ret = -1;
                goto out;
            }
        }

        if (params->is_bssid)
        {
            os_memcpy(wpa_s->next_scan_bssid, params->bssid, ETH_ALEN);
            wpa_s->next_scan_bssid_wildcard_ssid = 0;
        }

        if (params->is_ssid)
        {
            ssid = os_zalloc(sizeof(*ssid));
            if (ssid == NULL)
            {
                ret = -1;
                goto out;
            }
            ssid->ssid_len = os_strlen((const char *)&params->ssid[0]);
            os_memcpy(ssid->ssid, &params->ssid[0], ssid->ssid_len);
            wpa_hexdump_ascii(MSG_DEBUG, "scan SSID", ssid->ssid, ssid->ssid_len);
            ssid_count++;
        }
    }

    wpa_s->num_ssids_from_scan_req = ssid_count;
    os_free(wpa_s->ssids_from_scan_req);
    if (ssid_count)
    {
        wpa_s->ssids_from_scan_req = ssid;
    }
    else
    {
        wpa_s->ssids_from_scan_req = NULL;
    }

    if (scan_only)
        scan_res_handler = scan_only_handler;
    else if (wpa_s->scan_res_handler == scan_only_handler)
        scan_res_handler = NULL;
    else
        scan_res_handler = wpa_s->scan_res_handler;

    if (!wpa_s->sched_scanning && !wpa_s->scanning &&
        ((wpa_s->wpa_state <= WPA_SCANNING) || (wpa_s->wpa_state == WPA_COMPLETED)))
    {
        wpa_s->manual_scan_passive  = manual_scan_passive;
        wpa_s->manual_scan_use_id   = manual_scan_use_id;
        wpa_s->manual_scan_only_new = manual_scan_only_new;
        wpa_s->scan_id_count        = scan_id_count;
        os_memcpy(wpa_s->scan_id, scan_id, scan_id_count * sizeof(int));
        wpa_s->scan_res_handler = scan_res_handler;
        os_free(wpa_s->manual_scan_freqs);
        wpa_s->manual_scan_freqs = manual_scan_freqs;
        manual_scan_freqs        = NULL;

        wpa_s->normal_scans   = 0;
        wpa_s->scan_req       = MANUAL_SCAN_REQ;
        wpa_s->after_wps      = 0;
        wpa_s->known_wps_freq = 0;
        wpa_supplicant_req_scan(wpa_s, 0, 0);
        if (wpa_s->manual_scan_use_id)
        {
            wpa_s->manual_scan_id++;
            wpa_dbg(wpa_s, MSG_DEBUG, "Assigned scan id %u", wpa_s->manual_scan_id);
        }
    }
    else if (wpa_s->sched_scanning)
    {
        wpa_s->manual_scan_passive  = manual_scan_passive;
        wpa_s->manual_scan_use_id   = manual_scan_use_id;
        wpa_s->manual_scan_only_new = manual_scan_only_new;
        wpa_s->scan_id_count        = scan_id_count;
        os_memcpy(wpa_s->scan_id, scan_id, scan_id_count * sizeof(int));
        wpa_s->scan_res_handler = scan_res_handler;
        os_free(wpa_s->manual_scan_freqs);
        wpa_s->manual_scan_freqs = manual_scan_freqs;
        manual_scan_freqs        = NULL;

        wpa_printf(MSG_DEBUG, "Stop ongoing sched_scan to allow requested full scan to proceed");
        wpa_supplicant_cancel_sched_scan(wpa_s);
        wpa_s->scan_req = MANUAL_SCAN_REQ;
        wpa_supplicant_req_scan(wpa_s, 0, 0);
        if (wpa_s->manual_scan_use_id)
        {
            wpa_s->manual_scan_id++;
            wpa_dbg(wpa_s, MSG_DEBUG, "Assigned scan id %u", wpa_s->manual_scan_id);
        }
    }
    else
    {
        wpa_printf(MSG_DEBUG, "Ongoing scan action - reject new request");
        ret = -1;
        goto out;
    }

    send_wpa_supplicant_dummy_event();
    ret = 0;

out:
    if (ret != 0)
    {
        os_free(manual_scan_freqs);
    }

    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_abort_scan(const struct netif *dev)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    wpa_s->scan_res_fail_handler = wpa_supp_scan_res_fail_handler;

    ret = wpas_abort_ongoing_scan(wpa_s);
    if (ret != 0)
    {
        goto out;
    }
    send_wpa_supplicant_dummy_event();

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_cancel_scan(const struct netif *dev)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }
    if (wpa_s->wpa_state == WPA_SCANNING || wpa_s->wpa_state == WPA_DISCONNECTED)
        wpa_supplicant_cancel_scan(wpa_s);

    send_wpa_supplicant_dummy_event();

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE

static int wpa_config_process_blob(struct wpa_config *config, char *name, u8 *data, size_t data_len)
{
    struct wpa_config_blob *blob;

    blob = os_zalloc(sizeof(*blob));
    if (blob == NULL)
    {
        return -1;
    }

    blob->data = os_zalloc(data_len);
    if (blob->data == NULL)
    {
        os_free(blob);
        return -1;
    }

    blob->name = os_strdup(name);

    if (blob->name == NULL)
    {
        wpa_config_free_blob(blob);
        return -1;
    }

    os_memcpy(blob->data, data, data_len);
    blob->len = data_len;

    wpa_config_set_blob(config, blob);

    return 0;
}
#endif
static int wpa_parse_intlist(int **int_list, char *val)
{
    int *list;
    int count;
    char *pos, *end;

    os_free(*int_list);
    *int_list = NULL;

    pos   = val;
    count = 0;
    while (*pos != '\0')
    {
        if (*pos == ' ')
            count++;
        pos++;
    }

    list = os_malloc(sizeof(int) * (count + 2));
    if (list == NULL)
        return -1;
    pos   = val;
    count = 0;
    while (*pos != '\0')
    {
        end = os_strchr(pos, ' ');

        list[count++] = atoi(pos);
        if (!end)
            break;
        pos = end + 1;
    }
    list[count] = -1;

    *int_list = list;
    return 0;
}
#if CONFIG_HOSTAPD
#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE

static struct hostapd_eap_user *hostapd_config_read_default_eap_user(struct wlan_network *network,
                                                                     struct hostapd_eap_user **pnew_user)
{
    struct hostapd_eap_user *user = NULL, *tail = NULL, *new_user = NULL;
#if 0
    size_t i;
    unsigned char *id[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8"};
#endif

    user = os_zalloc(sizeof(*user));
    if (user == NULL)
    {
        wpa_printf(MSG_DEBUG, "EAP user allocation failed");
        goto failed;
    }
    user->force_version = -1;

#if CONFIG_EAP_PEAP
    if (
#if CONFIG_EAP_MSCHAPV2
        (network->security.type == WLAN_SECURITY_EAP_PEAP_MSCHAPV2) ||
#endif
#if CONFIG_EAP_TLS
        (network->security.type == WLAN_SECURITY_EAP_PEAP_TLS) ||
#endif
#if CONFIG_EAP_GTC
        (network->security.type == WLAN_SECURITY_EAP_PEAP_GTC) ||
#endif
        false)
    {
        user->methods[0].method = EAP_TYPE_PEAP;
        user->methods[0].vendor = EAP_VENDOR_IETF;
    }
#endif

#if CONFIG_EAP_TTLS
    if (
#if CONFIG_EAP_MSCHAPV2
        (network->security.type == WLAN_SECURITY_EAP_TTLS_MSCHAPV2) ||
#endif
#if CONFIG_EAP_TLS
        (network->security.type == WLAN_SECURITY_EAP_TTLS) ||
#endif
        false)
    {
        user->methods[0].method = EAP_TYPE_TTLS;
        user->methods[0].vendor = EAP_VENDOR_IETF;
    }
#endif
#if CONFIG_EAP_FAST
    if (
#if CONFIG_EAP_MSCHAPV2
        (network->security.type == WLAN_SECURITY_EAP_FAST_MSCHAPV2) ||
#endif
#if CONFIG_EAP_GTC
        (network->security.type == WLAN_SECURITY_EAP_FAST_GTC) ||
#endif
        false)
    {
        user->methods[0].method = EAP_TYPE_FAST;
        user->methods[0].vendor = EAP_VENDOR_IETF;
    }
#endif

    if (tail == NULL)
    {
        tail = new_user = user;
    }
    else
    {
        tail->next = user;
        tail       = user;
    }

    *pnew_user = new_user;

    return tail;

failed:
    if (new_user)
        hostapd_config_free_eap_user(new_user);

    return NULL;
}

static int hostapd_config_read_default2_eap_user(struct wlan_network *network,
                                                 struct hostapd_bss_config *conf)
{
    struct hostapd_eap_user *user = NULL, *tail = NULL, *new_user = NULL;
    u8 i, idx = 0, idx_inc = 0, idx_max = 9;
    char *id[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8"};
    u32 method;

#if CONFIG_EAP_SIM
    if (network->security.type == WLAN_SECURITY_EAP_SIM)
    {
        idx = 1;
        idx_inc = 2;
        idx_max = 6;
        method = EAP_TYPE_SIM;
    }
#endif
#if CONFIG_EAP_AKA
    if (network->security.type == WLAN_SECURITY_EAP_AKA)
    {
        idx = 0;
        idx_inc = 2;
        idx_max = 5;
        method = EAP_TYPE_AKA;
    }
#endif
#if CONFIG_EAP_AKA_PRIME
    if (network->security.type == WLAN_SECURITY_EAP_AKA_PRIME)
    {
        idx = 6;
        idx_inc = 1;
        idx_max = 9;
        method = EAP_TYPE_AKA_PRIME;
    }
#endif

    for (i = idx; i < idx_max; i += idx_inc)
    {
        user = os_zalloc(sizeof(*user));
        if (user == NULL)
        {
            wpa_printf(MSG_DEBUG, "EAP user allocation failed");
            goto failed;
        }
        user->force_version = -1;

        user->identity = os_memdup(id[i], os_strlen(id[i]));
        if (user->identity == NULL)
        {
            wpa_printf(MSG_DEBUG,
                       "Failed to allocate "
                       "memory for EAP identity");
            goto failed;
        }

        user->identity_len    = os_strlen(id[i]);
        user->wildcard_prefix = 1;

        user->methods[0].method = method;
        user->methods[0].vendor = EAP_VENDOR_IETF;

        if (tail == NULL)
        {
            tail = new_user = user;
        }
        else
        {
            tail->next = user;
            tail       = user;
        }
    }

    hostapd_config_free_eap_users(conf->eap_user);
    conf->eap_user = new_user;

    return 0;

failed:
    if (new_user)
        hostapd_config_free_eap_user(new_user);

    return -1;
}

#if CONFIG_EAP_TLS
static int hostapd_config_read_eap_user(const size_t nusers,
                                        const char (*identities)[IDENTITY_MAX_LENGTH],
                                        struct hostapd_bss_config *conf)
{
    struct hostapd_eap_user *user = NULL, *tail = NULL, *new_user = NULL;
    size_t i;
    int ret = 0;

    if (!nusers)
    {
        wpa_printf(MSG_DEBUG, "EAP users not found.");
        return -1;
    }

    for (i = 0; i < nusers; i++)
    {
        user = os_zalloc(sizeof(*user));
        if (user == NULL)
        {
            wpa_printf(MSG_DEBUG, "EAP user allocation failed");
            goto failed;
        }
        user->force_version = -1;

        user->identity = os_memdup(identities[i], os_strlen(identities[i]));
        if (user->identity == NULL)
        {
            wpa_printf(MSG_DEBUG,
                       "Failed to allocate "
                       "memory for EAP identity");
            goto failed;
        }

        user->identity_len = os_strlen(identities[i]);

        user->methods[0].method = EAP_TYPE_TLS;
        user->methods[0].vendor = EAP_VENDOR_IETF;

        if (tail == NULL)
        {
            tail = new_user = user;
        }
        else
        {
            tail->next = user;
            tail       = user;
        }
        continue;

    failed:
        if (user)
            hostapd_config_free_eap_user(user);
        ret = -1;
        break;
    }

    if (ret == 0)
    {
        hostapd_config_free_eap_users(conf->eap_user);
        conf->eap_user = new_user;
    }
    else
    {
        hostapd_config_free_eap_users(new_user);
    }

    return ret;
}
#endif

static int hostapd_config_read_eap_phase2_user(struct wlan_network *network, struct hostapd_bss_config *conf)
{
    struct hostapd_eap_user *user = NULL, *tail = NULL, *new_user = NULL;
    size_t i, nusers = network->security.nusers;
    int ret = 0;
    const char *identity, *password;

    if (!nusers)
    {
        wpa_printf(MSG_DEBUG, "EAP users not found.");
        return -1;
    }

    tail = hostapd_config_read_default_eap_user(network, &new_user);

    for (i = 0; i < nusers; i++)
    {
        identity = (const char *)&network->security.identities[i];
        password = (const char *)&network->security.passwords[i];

        if (
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
            (network->security.type == WLAN_SECURITY_EAP_PEAP_MSCHAPV2) ||
#endif
#if CONFIG_EAP_GTC
            (network->security.type == WLAN_SECURITY_EAP_PEAP_GTC) ||
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
            (network->security.type == WLAN_SECURITY_EAP_FAST_MSCHAPV2) ||
#endif
#if CONFIG_EAP_GTC
            (network->security.type == WLAN_SECURITY_EAP_FAST_GTC) ||
#endif
#endif
            false)
        {
            user = os_zalloc(sizeof(*user));
            if (user == NULL)
            {
                wpa_printf(MSG_DEBUG, "EAP user allocation failed");
                goto failed;
            }
            user->force_version = -1;

            user->identity = os_memdup(identity, os_strlen(identity));
            if (user->identity == NULL)
            {
                wpa_printf(MSG_DEBUG,
                           "Failed to allocate "
                           "memory for EAP identity");
                goto failed;
            }

            user->identity_len = os_strlen(identity);

#if CONFIG_EAP_MSCHAPV2
            user->methods[0].method = EAP_TYPE_MSCHAPV2;
            user->methods[0].vendor = EAP_VENDOR_IETF;
#endif
#if CONFIG_EAP_GTC
            user->methods[1].method = EAP_TYPE_GTC;
            user->methods[1].vendor = EAP_VENDOR_IETF;
#endif
            user->password = os_memdup(password, os_strlen(password));
            if (user->password == NULL)
            {
                wpa_printf(MSG_DEBUG,
                           "Failed to allocate "
                           "memory for EAP password");
                goto failed;
            }
            user->password_len = os_strlen(password);

            user->phase2 = 1;

            if (tail == NULL)
            {
                tail = new_user = user;
            }
            else
            {
                tail->next = user;
                tail       = user;
            }
        }
#if CONFIG_EAP_TLS
        if (
#if CONFIG_EAP_PEAP
            (network->security.type == WLAN_SECURITY_EAP_PEAP_TLS) ||
#endif
#if CONFIG_EAP_TTLS
            (network->security.type == WLAN_SECURITY_EAP_TTLS) ||
#endif
            false)
        {
            user = os_zalloc(sizeof(*user));
            if (user == NULL)
            {
                wpa_printf(MSG_DEBUG, "EAP user allocation failed");
                goto failed;
            }
            user->force_version = -1;

            user->identity = os_memdup(identity, os_strlen(identity));
            if (user->identity == NULL)
            {
                wpa_printf(MSG_DEBUG,
                           "Failed to allocate "
                           "memory for EAP identity");
                goto failed;
            }

            user->identity_len = os_strlen(identity);

            user->methods[0].method = EAP_TYPE_TLS;
            user->methods[0].vendor = EAP_VENDOR_IETF;

            user->password = os_memdup(password, os_strlen(password));
            if (user->password == NULL)
            {
                wpa_printf(MSG_DEBUG,
                           "Failed to allocate "
                           "memory for EAP password");
                goto failed;
            }
            user->password_len = os_strlen(password);

            user->phase2 = 1;

            if (tail == NULL)
            {
                tail = new_user = user;
            }
            else
            {
                tail->next = user;
                tail       = user;
            }
        }
#endif
#if CONFIG_EAP_TTLS
#if CONFIG_EAP_MSCHAPV2
        if (network->security.type == WLAN_SECURITY_EAP_TTLS_MSCHAPV2)
        {
            user = os_zalloc(sizeof(*user));
            if (user == NULL)
            {
                wpa_printf(MSG_DEBUG, "EAP user allocation failed");
                goto failed;
            }
            user->force_version = -1;

            user->identity = os_memdup(identity, os_strlen(identity));
            if (user->identity == NULL)
            {
                wpa_printf(MSG_DEBUG,
                           "Failed to allocate "
                           "memory for EAP identity");
                goto failed;
            }

            user->identity_len = os_strlen(identity);

            user->methods[0].method = EAP_TYPE_NONE;
            user->methods[0].vendor = EAP_VENDOR_IETF;

            user->ttls_auth |= EAP_TTLS_AUTH_PAP;
            user->ttls_auth |= EAP_TTLS_AUTH_CHAP;
            user->ttls_auth |= EAP_TTLS_AUTH_MSCHAP;
            user->ttls_auth |= EAP_TTLS_AUTH_MSCHAPV2;

            user->password = os_memdup(password, os_strlen(password));
            if (user->password == NULL)
            {
                wpa_printf(MSG_DEBUG,
                           "Failed to allocate "
                           "memory for EAP password");
                goto failed;
            }
            user->password_len = os_strlen(password);

            user->phase2 = 1;

            if (tail == NULL)
            {
                tail = new_user = user;
            }
            else
            {
                tail->next = user;
                tail       = user;
            }
        }
#endif
#endif
        continue;

    failed:
        if (user)
            hostapd_config_free_eap_user(user);
        ret = -1;
        break;
    }

    if (ret == 0)
    {
        hostapd_config_free_eap_users(conf->eap_user);
        conf->eap_user = new_user;
    }
    else
    {
        hostapd_config_free_eap_users(new_user);
    }

    return ret;
}
#endif

#if CONFIG_WPA_SUPP_WPA3
static int parse_sae_password(struct hostapd_bss_config *bss, const char *sae_password)
{
    struct sae_password_entry *pw;

    pw = os_zalloc(sizeof(*pw));
    if (!pw)
        return -1;
    os_memset(pw->peer_addr, 0xff, ETH_ALEN); /* default to wildcard */

#if CONFIG_SAE_PK
    pos2 = os_strstr(pos, "|pk=");
    if (pos2)
    {
        const char *epos;
        char *tmp;

        if (!end)
            end = pos2;
        pos2 += 4;
        epos = os_strchr(pos2, '|');
        if (epos)
        {
            tmp = os_malloc(epos - pos2 + 1);
            if (!tmp)
                goto fail;
            os_memcpy(tmp, pos2, epos - pos2);
            tmp[epos - pos2] = '\0';
        }
        else
        {
            tmp = os_strdup(pos2);
            if (!tmp)
                goto fail;
        }

        pw->pk = sae_parse_pk(tmp);
        str_clear_free(tmp);
        if (!pw->pk)
            goto fail;
    }
#endif /* CONFIG_SAE_PK */

    pw->password = os_strdup(sae_password);
    if (!pw->password)
        goto fail;

#if CONFIG_SAE_PK
    if (pw->pk &&
#if CONFIG_TESTING_OPTIONS
        !bss->sae_pk_password_check_skip &&
#endif /* CONFIG_TESTING_OPTIONS */
        !sae_pk_valid_password(pw->password))
    {
        wpa_printf(MSG_INFO, "Invalid SAE password for a SAE-PK sae_password entry");
        goto fail;
    }
#endif /* CONFIG_SAE_PK */

    pw->next           = bss->sae_passwords;
    bss->sae_passwords = pw;

    return 0;
fail:
    // str_clear_free(pw->password);
    // os_free(pw->identifier);
#if CONFIG_SAE_PK
    sae_deinit_pk(pw->pk);
#endif /* CONFIG_SAE_PK */
    os_free(pw);
    return -1;
}
#endif



static int hostapd_update_bss(struct hostapd_iface *hapd_s, struct wlan_network *network)
{
    struct hostapd_config *conf;
    struct hostapd_bss_config *bss;
    struct hostapd_ssid *ssid;
    struct hostapd_data *hapd;
#if (CONFIG_WPA_SUPP_WPA3) || (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE)
    int ret = -1;
#endif

    hapd = hapd_s->bss[0];
    conf = hapd_s->conf;
    bss  = conf->last_bss;

    hapd_s->freq = 0;

    bss->max_num_sta = 10;

    os_memcpy(bss->bssid, network->bssid, ETH_ALEN);
    os_memcpy(hapd->own_addr, network->bssid, ETH_ALEN);

    conf->acs_exclude_dfs = 1;
    if (network->channel > 14 || (network->channel == 0 && network->acs_band == 1))
    {
        conf->hw_mode = HOSTAPD_MODE_IEEE80211A;
    }
    else
    {
        conf->hw_mode = HOSTAPD_MODE_IEEE80211G;
    }

    conf->channel = network->channel;
    conf->acs     = conf->channel == 0;

    if(conf->ieee80211d)
    {
        conf->spectrum_mgmt_required = 1;
        conf->local_pwr_constraint = 3;
    }
    else
    {
        conf->spectrum_mgmt_required = 0;
        conf->local_pwr_constraint = -1;
    }

    conf->obss_interval = 10;

    if (network->dot11n)
    {
        conf->ieee80211n              = 1;
        conf->ht_capab                = network->ht_capab;
        conf->ht40_plus_minus_allowed = 1;
        conf->secondary_channel = 0;

        bss->wmm_enabled = 1;
        bss->wmm_uapsd = 0;

        if (network->sec_channel_offset == 1)
        {
            conf->secondary_channel = 1;
        }
        else if (network->sec_channel_offset == 3)
        {
            conf->secondary_channel = -1;
        }
    }
    else
    {
        conf->ieee80211n = 0;
        conf->ht_capab = 0;
        conf->secondary_channel = 0;
        bss->wmm_enabled = 0;
    }

#if CONFIG_11AC
    if (network->dot11ac)
    {
        conf->ieee80211ac = 1;

        if (conf->acs == 0)
        {
            if (network->channel > 14)
            {
                conf->vht_oper_chwidth             = network->vht_oper_chwidth;
            }
            else
            {
                conf->vht_oper_chwidth             = 0;
                conf->vht_oper_centr_freq_seg0_idx = 0;
            }
        }
        conf->vht_capab = network->vht_capab;
    }
    else
    {
        conf->ieee80211ac = 0;
    }
#endif

#if CONFIG_11AX
    if (network->dot11ax)
    {
        conf->ieee80211ax = 1;

        if (conf->acs == 0)
        {
            if (network->channel > 14)
            {
                conf->he_oper_chwidth             = network->he_oper_chwidth;
            }
            else
            {
                conf->he_oper_chwidth             = 0;
                conf->he_oper_centr_freq_seg0_idx = 0;
            }
        }
        conf->he_phy_capab.he_su_beamformer = 0;
        conf->he_phy_capab.he_su_beamformee = 1;
        conf->he_phy_capab.he_mu_beamformer = 0;
        conf->he_op.he_bss_color            = 1;
        conf->he_op.he_default_pe_duration  = 0;
        /* Set default basic MCS/NSS set to single stream MCS 0-7 */
        conf->he_op.he_basic_mcs_nss_set    = 0xfffc;
    }
    else
    {
        conf->ieee80211ax = 0;
    }
#endif

    conf->no_pri_sec_switch = 1;
    conf->ht_op_mode_fixed  = 1;

#if CONFIG_WIFI_DTIM_PERIOD
    bss->dtim_period = network->dtim_period == 0 ? UAP_DTIM_PERIOD : network->dtim_period;
#endif

    ssid           = &bss->ssid;
    ssid->ssid_len = os_strlen(network->ssid);
    if (ssid->ssid_len > SSID_MAX_LEN || ssid->ssid_len < 1)
    {
        wpa_printf(MSG_DEBUG, "invalid SSID '%s'", ssid->ssid);
        return -1;
    }
    os_memcpy(ssid->ssid, network->ssid, ssid->ssid_len);
    ssid->ssid_set   = 1;
    ssid->short_ssid = crc32(ssid->ssid, ssid->ssid_len);

    bss->wpa_key_mgmt = network->security.key_mgmt;

    if (network->security.type != WLAN_SECURITY_NONE)
    {
        bss->wpa = WPA_PROTO_RSN;

        bss->wpa_group         = network->security.group_cipher;
        bss->rsn_pairwise      = network->security.pairwise_cipher;
        bss->wpa_pairwise      = network->security.pairwise_cipher;
        bss->group_mgmt_cipher = network->security.group_mgmt_cipher;
    }

    switch (network->security.type)
    {
#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE
#if CONFIG_EAP_TLS
        case WLAN_SECURITY_EAP_TLS:
        case WLAN_SECURITY_EAP_TLS_SHA256:
#if CONFIG_11R
        case WLAN_SECURITY_EAP_TLS_FT:
        case WLAN_SECURITY_EAP_TLS_FT_SHA384:
#endif
#endif
#if CONFIG_EAP_TTLS
        case WLAN_SECURITY_EAP_TTLS:
#if CONFIG_EAP_MSCHAPV2
        case WLAN_SECURITY_EAP_TTLS_MSCHAPV2:
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
        case WLAN_SECURITY_EAP_PEAP_MSCHAPV2:
#endif
#if CONFIG_EAP_TLS
        case WLAN_SECURITY_EAP_PEAP_TLS:
#endif
#if CONFIG_EAP_GTC
        case WLAN_SECURITY_EAP_PEAP_GTC:
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
        case WLAN_SECURITY_EAP_FAST_MSCHAPV2:
#endif
#if CONFIG_EAP_GTC
        case WLAN_SECURITY_EAP_FAST_GTC:
#endif
#endif
#if CONFIG_EAP_SIM
        case WLAN_SECURITY_EAP_SIM:
#endif
#if CONFIG_EAP_AKA
        case WLAN_SECURITY_EAP_AKA:
#endif
#if CONFIG_EAP_AKA_PRIME
        case WLAN_SECURITY_EAP_AKA_PRIME:
#endif
            if (network->security.type == WLAN_SECURITY_EAP_TLS_SHA256)
            {
                bss->wpa_key_mgmt = WPA_KEY_MGMT_IEEE8021X_SHA256;
            }
#if CONFIG_11R
            else if (network->security.type == WLAN_SECURITY_EAP_TLS_FT)
            {
                bss->wpa_key_mgmt          = WPA_KEY_MGMT_IEEE8021X | WPA_KEY_MGMT_FT_IEEE8021X;
                bss->mobility_domain[0]    = 'a';
                bss->mobility_domain[1]    = '1';
                bss->pmk_r1_push           = 1;
                bss->ft_over_ds            = 0;
                bss->ft_psk_generate_local = 1;
            }
            else if (network->security.type == WLAN_SECURITY_EAP_TLS_FT_SHA384)
            {
                bss->wpa_key_mgmt          = WPA_KEY_MGMT_IEEE8021X | WPA_KEY_MGMT_FT_IEEE8021X_SHA384;
                bss->mobility_domain[0]    = 'a';
                bss->mobility_domain[1]    = '2';
                bss->pmk_r1_push           = 1;
                bss->ft_over_ds            = 0;
                bss->ft_psk_generate_local = 1;
            }
#endif
            else
            {
                if (network->security.wpa3_sb_192)
                {
                    bss->wpa_key_mgmt      = WPA_KEY_MGMT_IEEE8021X_SUITE_B_192;
                    bss->group_mgmt_cipher = WPA_CIPHER_BIP_GMAC_256;
                    bss->tls_flags |= TLS_CONN_SUITEB;
                }
                else if (network->security.wpa3_sb)
                {
                    bss->wpa_key_mgmt = WPA_KEY_MGMT_IEEE8021X_SUITE_B;
                    bss->tls_flags |= TLS_CONN_SUITEB;
                }
                else
                {
                    bss->wpa_key_mgmt = WPA_KEY_MGMT_IEEE8021X;
                }
            }

            bss->ieee802_1x    = 1; /* use IEEE 802.1X */
            bss->eapol_version = 2;
            bss->eap_server    = 1;

#if CONFIG_EAP_FAST
            if (
#if CONFIG_EAP_MSCHAPV2
                    (network->security.type == WLAN_SECURITY_EAP_FAST_MSCHAPV2) ||
#endif
#if CONFIG_EAP_GTC
                    (network->security.type == WLAN_SECURITY_EAP_FAST_GTC)
#endif
                    || false)
            {
                size_t idlen = os_strlen(network->security.pac_opaque_encr_key);
                if (idlen != 32)
                {
                    wpa_printf(MSG_ERROR, "Invalid pac_opaque_encr_key");
                    return -1;
                }

                os_free(bss->pac_opaque_encr_key);
                bss->pac_opaque_encr_key = os_malloc(16);

                if (bss->pac_opaque_encr_key == NULL)
                {
                    wpa_printf(MSG_ERROR, "No memory for pac_opaque_encr_key");
                    return -1;
                }
                else if (hexstr2bin(network->security.pac_opaque_encr_key, bss->pac_opaque_encr_key, 16))
                {
                    wpa_printf(MSG_ERROR, "Invalid pac_opaque_encr_key");
                    return -1;
                }

                idlen = os_strlen(network->security.a_id);
                if (idlen & 1)
                {
                    wpa_printf(MSG_ERROR, "Invalid eap_fast_a_id");
                    return -1;
                }

                os_free(bss->eap_fast_a_id);
                bss->eap_fast_a_id = os_malloc(idlen / 2);
                if (bss->eap_fast_a_id == NULL || hexstr2bin(network->security.a_id, bss->eap_fast_a_id, idlen / 2))
                {
                    wpa_printf(MSG_ERROR, "Line %d: Failed to parse eap_fast_a_id");
                    os_free(bss->eap_fast_a_id);
                    bss->eap_fast_a_id = NULL;
                    return -1;
                }
                else
                {
                    bss->eap_fast_a_id_len = idlen / 2;
                }

                bss->eap_fast_a_id_info =
                    dup_binstr("hostapd FAST/MSCHAPv2 GTC", os_strlen("hostapd FAST/MSCHAPv2 GTC"));

                bss->eap_fast_prov = network->security.fast_prov;
            }
#endif
            bss->ca_cert_blob_len     = network->security.ca_cert_len;
            bss->ca_cert_blob         = network->security.ca_cert_data;
            bss->server_cert_blob_len = network->security.server_cert_len;
            bss->server_cert_blob     = network->security.server_cert_data;
            bss->private_key_blob_len = network->security.server_key_len;
            bss->private_key_blob     = network->security.server_key_data;
            bss->dh_blob_len          = network->security.dh_len;
            bss->dh_blob              = network->security.dh_data;

            str_clear_free(bss->private_key_passwd);
            bss->private_key_passwd =
                dup_binstr(network->security.server_key_passwd, os_strlen(network->security.server_key_passwd));

#if CONFIG_EAP_TLS
            if ((network->security.type == WLAN_SECURITY_EAP_TLS) ||
                (network->security.type == WLAN_SECURITY_EAP_TLS_SHA256)
#if CONFIG_11R
                || (network->security.type == WLAN_SECURITY_EAP_TLS_FT) ||
                (network->security.type == WLAN_SECURITY_EAP_TLS_FT_SHA384)
#endif
                )
            {
                ret = hostapd_config_read_eap_user(network->security.nusers, network->security.identities, bss);
                if (ret != 0)
                {
                    wpa_printf(MSG_DEBUG, "%s:Failed to read eap users", __func__);
                    return -1;
                }
            }
#endif

            if (
#if CONFIG_EAP_SIM
                (network->security.type == WLAN_SECURITY_EAP_SIM) ||
#endif
#if CONFIG_EAP_AKA
                (network->security.type == WLAN_SECURITY_EAP_AKA) ||
#endif
#if CONFIG_EAP_AKA_PRIME
                (network->security.type == WLAN_SECURITY_EAP_AKA_PRIME) ||
#endif
                false)
            {
                ret = hostapd_config_read_default2_eap_user(network, bss);
                if (ret != 0)
                {
                    wpa_printf(MSG_DEBUG, "%s:Failed to read eap users", __func__);
                    return -1;
                }
            }

            if (
#if CONFIG_EAP_TTLS
                (network->security.type == WLAN_SECURITY_EAP_TTLS) ||
#if CONFIG_EAP_MSCHAPV2
                (network->security.type == WLAN_SECURITY_EAP_TTLS_MSCHAPV2) ||
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
                (network->security.type == WLAN_SECURITY_EAP_PEAP_MSCHAPV2) ||
#endif
#if CONFIG_EAP_TTLS
                (network->security.type == WLAN_SECURITY_EAP_PEAP_TLS) ||
#endif
#if CONFIG_EAP_GTC
                (network->security.type == WLAN_SECURITY_EAP_PEAP_GTC) ||
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
                (network->security.type == WLAN_SECURITY_EAP_FAST_MSCHAPV2) ||
#endif
#if CONFIG_EAP_GTC
                (network->security.type == WLAN_SECURITY_EAP_FAST_GTC) ||
#endif
#endif
                false)
            {
                ret = hostapd_config_read_eap_phase2_user(network, bss);
                if (ret != 0)
                {
                    wpa_printf(MSG_DEBUG, "%s:Failed to read eap phase2 users", __func__);
                    return -1;
                }
            }

            break;
#endif
#if CONFIG_WPA_SUPP_WPA3
        case WLAN_SECURITY_WPA3_SAE:
        case WLAN_SECURITY_WPA2_WPA3_SAE_MIXED:
#if CONFIG_11R
        case WLAN_SECURITY_WPA3_FT_SAE:
            if (network->security.type == WLAN_SECURITY_WPA3_FT_SAE)
            {
                bss->mobility_domain[0]    = 'a';
                bss->mobility_domain[1]    = '3';
                bss->pmk_r1_push           = 1;
                bss->ft_over_ds            = 0;
                bss->ft_psk_generate_local = 1;
            }
            else
#endif
                if (network->security.type == WLAN_SECURITY_WPA2_WPA3_SAE_MIXED)
            {
                if (network->security.pmk_valid && bss->ssid.wpa_psk)
                {
                    os_memcpy(bss->ssid.wpa_psk->psk, network->security.pmk, PMK_LEN);
                    bss->ssid.wpa_psk->group = 1;
                    bss->ssid.wpa_psk_set    = 1;
                }
                else
                {
                    bss->ssid.wpa_passphrase = os_strdup(network->security.psk);
                    if (bss->ssid.wpa_passphrase)
                    {
                        hostapd_config_clear_wpa_psk(&bss->ssid.wpa_psk);
                        bss->ssid.wpa_passphrase_set = 1;
                    }
                    else
                    {
                        wpa_printf(MSG_DEBUG, "%s:Failed to copy passphrase", __func__);
                        return -1;
                    }
                }
            }

            if (wpa_parse_intlist(&bss->sae_groups, network->security.sae_groups))
            {
                wpa_printf(MSG_ERROR, "Invalid sae_groups value");
                return -1;
            }

            bss->sae_pwe            = network->security.pwe_derivation;
            bss->transition_disable = network->security.transition_disable;

            ret = parse_sae_password(bss, network->security.password);
            if (ret != 0)
            {
                wpa_printf(MSG_DEBUG, "%s:Failed to set sae_password", __func__);
                return -1;
            }

            break;
#endif
#ifdef CONFIG_OWE
        case WLAN_SECURITY_OWE_ONLY:

            if (wpa_parse_intlist(&bss->owe_groups, network->security.owe_groups))
            {
                wpa_printf(MSG_ERROR, "Invalid owe_groups value");
                return -1;
            }
            break;
#endif
        case WLAN_SECURITY_WPA2:
#if CONFIG_11R
        case WLAN_SECURITY_WPA2_FT:
#endif
        case WLAN_SECURITY_WPA_WPA2_MIXED:

#if CONFIG_11R
            if (network->security.type == WLAN_SECURITY_WPA2_FT)
            {
                bss->mobility_domain[0]    = 'a';
                bss->mobility_domain[1]    = '4';
                bss->pmk_r1_push           = 1;
                bss->ft_over_ds            = 0;
                bss->ft_psk_generate_local = 1;
            }
            else
#endif
            {
                if (network->security.type == WLAN_SECURITY_WPA_WPA2_MIXED)
                {
                    bss->wpa = WPA_PROTO_WPA | WPA_PROTO_RSN;
                }
            }

            if (network->security.pmk_valid && bss->ssid.wpa_psk)
            {
                os_memcpy(bss->ssid.wpa_psk->psk, network->security.pmk, PMK_LEN);
                bss->ssid.wpa_psk->group = 1;
                bss->ssid.wpa_psk_set    = 1;
            }
            else
            {
                // str_clear_free(bss->ssid.wpa_passphrase);
                bss->ssid.wpa_passphrase = os_strdup(network->security.psk);
                if (bss->ssid.wpa_passphrase)
                {
                    hostapd_config_clear_wpa_psk(&bss->ssid.wpa_psk);
                    bss->ssid.wpa_passphrase_set = 1;
                }
                else
                {
                    wpa_printf(MSG_DEBUG, "%s:Failed to copy passphrase", __func__);
                    return -1;
                }
            }
            break;
#ifdef CONFIG_DPP
        case WLAN_SECURITY_DPP:
            bss->wpa_key_mgmt = WPA_KEY_MGMT_DPP;
            break;
#endif
        default:
            break;
    }

#ifdef CONFIG_DPP
    if ((network->security.key_mgmt & WLAN_KEY_MGMT_DPP) || (network->security.key_mgmt & WLAN_KEY_MGMT_PSK) ||
        (network->security.key_mgmt & WLAN_KEY_MGMT_SAE) || (network->security.key_mgmt & WLAN_KEY_MGMT_IEEE8021X))
    {
        if (network->security.dpp_connector)
        {
            os_free(bss->dpp_connector);
            bss->dpp_connector = os_strdup((const char *)network->security.dpp_connector);
        }
        if (network->security.dpp_c_sign_key)
        {
            wpabuf_free(bss->dpp_csign);
            bss->dpp_csign = wpabuf_parse_bin((const char *)network->security.dpp_c_sign_key);
            if (!bss->dpp_csign)
            {
                wpa_printf(MSG_ERROR, "%s: Invalid dpp_csign '%s'", __func__, network->security.dpp_c_sign_key);
                return -1;
            }
        }
        if (network->security.dpp_net_access_key)
        {
            wpabuf_free(bss->dpp_netaccesskey);
            bss->dpp_netaccesskey = wpabuf_parse_bin((const char *)network->security.dpp_net_access_key);
            if (!bss->dpp_netaccesskey)
            {
                wpa_printf(MSG_ERROR, "%s: Invalid dpp_netaccesskey '%s'", __func__, network->security.dpp_net_access_key);
                return -1;
            }
        }
#ifdef CONFIG_DPP2
        /* Add channels from scan results for APs that advertise Configurator
         * Connectivity element */
        bss->dpp_configurator_connectivity = 1;
#endif /* CONFIG_DPP2 */
    }
#endif
    if ((network->security.mfpc) && (network->security.mfpr))
        bss->ieee80211w = MGMT_FRAME_PROTECTION_REQUIRED;
    else if (network->security.mfpc)
        bss->ieee80211w = MGMT_FRAME_PROTECTION_OPTIONAL;

    return 0;
}

static void hostapd_reset_bss(struct hostapd_bss_config *bss)
{
    bss->wpa_key_mgmt = 0;

    bss->wpa = 0;

    bss->wpa_group         = 0;
    bss->rsn_pairwise      = 0;
    bss->wpa_pairwise      = 0;
    bss->group_mgmt_cipher = 0;

#if CONFIG_11R
    bss->pmk_r1_push           = 0;
    bss->ft_over_ds            = 0;
    bss->ft_psk_generate_local = 0;
#endif

    bss->tls_flags     = 0;
    bss->ieee802_1x    = 0;
    bss->eap_server    = 1;

    bss->sae_pwe = 0;
    bss->transition_disable = 0;

    bss->ssid.wpa_passphrase_set = 0;

    bss->ieee80211w = NO_MGMT_FRAME_PROTECTION;
}

#if CONFIG_WPA_SUPP_WPA3
void hostapd_config_free_sae_passwords(struct hostapd_bss_config *conf)
{
    struct sae_password_entry *pw, *tmp;

    pw                  = conf->sae_passwords;
    conf->sae_passwords = NULL;
    while (pw)
    {
        tmp = pw;
        pw  = pw->next;
        str_clear_free(tmp->password);
        os_free(tmp->identifier);
#if CONFIG_SAE
        sae_deinit_pt(tmp->pt);
#endif /* CONFIG_SAE */
#if CONFIG_SAE_PK
        sae_deinit_pk(tmp->pk);
#endif /* CONFIG_SAE_PK */
        os_free(tmp);
    }
}
#endif

static int hostapd_free_bss(struct hostapd_iface *hapd_s, struct wlan_network *network)
{
    struct hostapd_config *conf;
    struct hostapd_bss_config *bss;

    conf = hapd_s->conf;
    bss  = conf->last_bss;

    switch (network->security.type)
    {
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
        case WLAN_SECURITY_EAP_TLS:
        case WLAN_SECURITY_EAP_TLS_SHA256:
#if CONFIG_11R
        case WLAN_SECURITY_EAP_TLS_FT:
        case WLAN_SECURITY_EAP_TLS_FT_SHA384:
#endif
#endif
#if CONFIG_EAP_TTLS
        case WLAN_SECURITY_EAP_TTLS:
#if CONFIG_EAP_MSCHAPV2
        case WLAN_SECURITY_EAP_TTLS_MSCHAPV2:
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
        case WLAN_SECURITY_EAP_PEAP_MSCHAPV2:
#endif
#if CONFIG_EAP_TLS
        case WLAN_SECURITY_EAP_PEAP_TLS:
#endif
#if CONFIG_EAP_GTC
        case WLAN_SECURITY_EAP_PEAP_GTC:
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
        case WLAN_SECURITY_EAP_FAST_MSCHAPV2:
#endif
#if CONFIG_EAP_GTC
        case WLAN_SECURITY_EAP_FAST_GTC:
#endif
            if (bss->pac_opaque_encr_key)
            {
                os_free(bss->pac_opaque_encr_key);
                bss->pac_opaque_encr_key = NULL;
            }
            if (bss->eap_fast_a_id)
            {
                os_free(bss->eap_fast_a_id);
                bss->eap_fast_a_id = NULL;
            }
            str_clear_free(bss->eap_fast_a_id_info);
            bss->eap_fast_a_id_info = NULL;
#endif
            hostapd_config_free_eap_users(bss->eap_user);
            bss->eap_user = NULL;
            break;
#endif
#if CONFIG_WPA_SUPP_WPA3
        case WLAN_SECURITY_WPA2_WPA3_SAE_MIXED:
            str_clear_free(bss->ssid.wpa_passphrase);
            bss->ssid.wpa_passphrase = NULL;
            hostapd_config_clear_wpa_psk(&bss->ssid.wpa_psk);
        case WLAN_SECURITY_WPA3_SAE:
#if CONFIG_11R
        case WLAN_SECURITY_WPA3_FT_SAE:
#endif
            hostapd_config_free_sae_passwords(bss);
            break;
#endif
#ifdef CONFIG_OWE
        case WLAN_SECURITY_OWE_ONLY:
            break;
#endif
        case WLAN_SECURITY_WPA2:
#if CONFIG_11R
        case WLAN_SECURITY_WPA2_FT:
#endif
        case WLAN_SECURITY_WPA_WPA2_MIXED:

            str_clear_free(bss->ssid.wpa_passphrase);
            bss->ssid.wpa_passphrase = NULL;
            hostapd_config_clear_wpa_psk(&bss->ssid.wpa_psk);
            break;
        default:
            break;
    }

    hostapd_reset_bss(bss);
    if (bss->accept_mac)
    {
        os_free(bss->accept_mac);
        bss->accept_mac =0;
    }
    if (bss->deny_mac)
    {
        os_free(bss->deny_mac);
        bss->deny_mac = 0;
    }
    return 0;
}
#endif

#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
static void str2hex(const char *str, char *strH)
{
    unsigned int i, j;

    for (i = 0, j = 0; i < strlen(str); i++, j += 2)
    {
        sprintf((char*)strH + j, "%02X", str[i]);
    }

    strH[j] = '\0';
}
#endif

int wpa_supp_add_network(const struct netif *dev, struct wlan_network *network)
{
    int ret               = 0;
    struct wpa_ssid *ssid = NULL;
    bool pmf              = true;
    struct wpa_supplicant *wpa_s;
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
    char phase1[256] = {0};
    char *openssl_ciphers;
    char HashH[65] = {0};
    char hashstr[128] = {0};
#endif
#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;
#endif

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (network->role == WLAN_BSS_ROLE_UAP)
    {
        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

        ssid       = wpa_supplicant_add_network(wpa_s);
        ssid->ssid = os_zalloc(sizeof(u8) * MAX_SSID_LEN);

        ssid->ssid_len = os_strlen(network->ssid);
        memcpy(ssid->ssid, network->ssid, ssid->ssid_len);
        if (network->bssid_specific)
        {
            memcpy(ssid->bssid, network->bssid, IEEEtypes_ADDRESS_SIZE);
            ssid->bssid_set = 1;
        }

        ssid->disabled = 1;
        ssid->key_mgmt = network->security.key_mgmt;
        ssid->scan_ssid = 1;
        //ssid->proactive_key_caching = network->security.pkc;

        if (network->security.type == WLAN_SECURITY_WILDCARD)
        {
            if (network->security.psk_len)
            {
                network->security.type = WLAN_SECURITY_WPA2;
            }
            else
            {
                network->security.type = WLAN_SECURITY_NONE;
            }
        }

        if (network->security.type != WLAN_SECURITY_NONE)
        {
            ssid->proto             = WPA_PROTO_RSN;
            ssid->group_cipher      = network->security.group_cipher;
            ssid->pairwise_cipher   = network->security.pairwise_cipher;
            ssid->group_mgmt_cipher = network->security.group_mgmt_cipher;
        }

        if (network->channel != 0U)
        {
            /* We use global channel list here and also use the widest
             * op_class for 5GHz channels as there is no user input
             * for these.
             */
            int freq = ieee80211_chan_to_freq(NULL, 81, network->channel);

            if (freq <= 0)
            {
                freq = ieee80211_chan_to_freq(NULL, 128, network->channel);
            }

            if (freq <= 0)
            {
                wpa_printf(MSG_DEBUG, "Invalid channel %d", network->channel);
                ret = -1;
                goto out;
            }

            if (network->role == WLAN_BSS_ROLE_UAP)
            {
                ssid->frequency = freq;
            }
            else
            {
                ssid->scan_freq = os_zalloc(2 * sizeof(int));
                if (!ssid->scan_freq)
                {
                    ret = -1;
                    goto out;
                }
                ssid->scan_freq[0] = freq;
                ssid->scan_freq[1] = 0;

                ssid->freq_list = os_zalloc(2 * sizeof(int));
                if (!ssid->freq_list)
                {
                    os_free(ssid->scan_freq);
                    ret = -1;
                    goto out;
                }
                ssid->freq_list[0] = freq;
                ssid->freq_list[1] = 0;
            }
        }

        wpa_s->conf->filter_ssids = 1;
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
        openssl_ciphers           = "DEFAULT:!EXP:!LOW";
#endif

        if (network->role == WLAN_BSS_ROLE_UAP)
        {
            wpa_s->conf->ap_scan = 2;
            ssid->mode           = WPAS_MODE_AP;
        }
        else
        {
            wpa_s->conf->ap_scan = 1;
            ssid->mode           = WPAS_MODE_INFRA;
        }

        switch (network->security.type)
        {
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
            case WLAN_SECURITY_EAP_TLS:
            case WLAN_SECURITY_EAP_TLS_SHA256:
#if CONFIG_11R
            case WLAN_SECURITY_EAP_TLS_FT:
            case WLAN_SECURITY_EAP_TLS_FT_SHA384:
#endif
#endif
#if CONFIG_EAP_TTLS
            case WLAN_SECURITY_EAP_TTLS:
#if CONFIG_EAP_MSCHAPV2
            case WLAN_SECURITY_EAP_TTLS_MSCHAPV2:
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
            case WLAN_SECURITY_EAP_PEAP_MSCHAPV2:
#endif
#if CONFIG_EAP_TLS
            case WLAN_SECURITY_EAP_PEAP_TLS:
#endif
#if CONFIG_EAP_GTC
            case WLAN_SECURITY_EAP_PEAP_GTC:
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
            case WLAN_SECURITY_EAP_FAST_MSCHAPV2:
#endif
#if CONFIG_EAP_GTC
            case WLAN_SECURITY_EAP_FAST_GTC:
#endif
#endif
#if CONFIG_EAP_SIM
            case WLAN_SECURITY_EAP_SIM:
#endif
#if CONFIG_EAP_AKA
            case WLAN_SECURITY_EAP_AKA:
#endif
#if CONFIG_EAP_AKA_PRIME
            case WLAN_SECURITY_EAP_AKA_PRIME:
#endif
                if (network->role == WLAN_BSS_ROLE_UAP)
                {
                    ret = -1;
                    goto out;
                }
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
                if (network->security.type == WLAN_SECURITY_EAP_TLS_SHA256)
                    ssid->key_mgmt = WPA_KEY_MGMT_IEEE8021X_SHA256;
#if CONFIG_11R
                else if (network->security.type == WLAN_SECURITY_EAP_TLS_FT)
                    ssid->key_mgmt = WPA_KEY_MGMT_FT_IEEE8021X;
                else if (network->security.type == WLAN_SECURITY_EAP_TLS_FT_SHA384)
                {
                    ssid->key_mgmt = WPA_KEY_MGMT_FT_IEEE8021X_SHA384;
                    openssl_ciphers = "SUITEB192";
                }

#endif
                else
#endif
#endif
                {
                    if (network->security.wpa3_sb_192)
                    {
                        ssid->key_mgmt  = WPA_KEY_MGMT_IEEE8021X_SUITE_B_192;
                        openssl_ciphers = "SUITEB192";
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
                        if (network->security.tls_cipher == EAP_TLS_ECC_P384)
                        {
                            /* Per WiFi Cert WPA3 test plan 19.5.2 step 1, STAUT should only send
                             * TLS_ECDHE_ECDSA_AES_256_GCM_SHA384 as the TLS cipher in the Client
                             * Hello frame during EAP exchange */
                            str_clear_free(ssid->eap.openssl_ciphers);
                            ssid->eap.openssl_ciphers = dup_binstr(openssl_ciphers, os_strlen(openssl_ciphers));
                        }
                        else if (network->security.tls_cipher == EAP_TLS_RSA_3K)
                        {
                            /* Per WiFi Cert WPA3 test plan 19.5.2 step 4, STAUT should only send
                             * TLS_ECDHE_RSA_AES_256_GCM_SHA384 and TLS_DHE_RSA_AES_256_GCM_SHA384
                             * as the TLS ciphers in the Client Hello frame during EAP exchange */
                            os_snprintf(phase1, sizeof(phase1), "tls_suiteb=1");
                            str_clear_free(ssid->eap.phase1);
                            ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));
                        }
#endif
#endif
                    }
                    else if (network->security.wpa3_sb)
                    {
                        ssid->key_mgmt  = WPA_KEY_MGMT_IEEE8021X_SUITE_B;
                        openssl_ciphers = "SUITEB128";
                    }
                    else
                    {
                        ssid->key_mgmt = WPA_KEY_MGMT_IEEE8021X;
                    }
                }

                str_clear_free((char *)wpa_s->conf->openssl_ciphers);
                wpa_s->conf->openssl_ciphers = dup_binstr(openssl_ciphers, os_strlen(openssl_ciphers));

                ssid->eap.eap_methods = os_zalloc(sizeof(struct eap_method_type));
                if (!ssid->eap.eap_methods)
                {
                    ret = -1;
                    goto out;
                }

                ssid->eap.eap_methods->vendor = EAP_VENDOR_IETF;
                ssid->eap.eap_methods->method = EAP_TYPE_TLS;

                ssid->eap.anonymous_identity_len = os_strlen(network->security.anonymous_identity);
                if (ssid->eap.anonymous_identity_len)
                {
                    str_clear_free((char *)ssid->eap.anonymous_identity);
                    ssid->eap.anonymous_identity =
                        (u8 *)dup_binstr(network->security.anonymous_identity, ssid->eap.anonymous_identity_len);
                }

                ssid->eap.identity_len = os_strlen(network->security.identity);
                if (ssid->eap.identity_len)
                {
                    str_clear_free((char *)ssid->eap.identity);
                    ssid->eap.identity = (u8 *)dup_binstr(network->security.identity, ssid->eap.identity_len);
                }

                ssid->eap.password_len = os_strlen(network->security.eap_password);
                if (ssid->eap.password_len)
                {
                    str_clear_free((char *)ssid->eap.password);
                    ssid->eap.password = (u8 *)dup_binstr(network->security.eap_password, ssid->eap.password_len);
                }

                if (os_strlen(network->security.domain_match))
                {
                    str_clear_free(ssid->eap.cert.domain_match);
                    ssid->eap.cert.domain_match = dup_binstr(network->security.domain_match, os_strlen(network->security.domain_match));
                }

                if (os_strlen(network->security.domain_suffix_match))
                {
                    str_clear_free(ssid->eap.cert.domain_suffix_match);
                    ssid->eap.cert.domain_suffix_match = dup_binstr(network->security.domain_suffix_match, os_strlen(network->security.domain_suffix_match));
                }

                if (os_strlen(network->security.ca_cert_hash))
                {
                    str2hex(network->security.ca_cert_hash, HashH);
                    os_snprintf(hashstr, sizeof(hashstr), "hash://server/sha256/%s", HashH);
                    str_clear_free(ssid->eap.cert.ca_cert);
                    ssid->eap.cert.ca_cert = dup_binstr(hashstr, os_strlen(hashstr));
                }
                else if (network->security.ca_cert_len)
                {
                    wpa_config_process_blob(wpa_s->conf, "ca_cert", network->security.ca_cert_data,
                                            network->security.ca_cert_len);

                    str_clear_free(ssid->eap.cert.ca_cert);
                    ssid->eap.cert.ca_cert = dup_binstr("blob://ca_cert", os_strlen("blob://ca_cert"));
                }

                if (network->security.client_cert_len)
                {
                    wpa_config_process_blob(wpa_s->conf, "client_cert", network->security.client_cert_data,
                                            network->security.client_cert_len);

                    str_clear_free(ssid->eap.cert.client_cert);
                    ssid->eap.cert.client_cert = dup_binstr("blob://client_cert", os_strlen("blob://client_cert"));
                }

                if (network->security.client_key_len)
                {
                    wpa_config_process_blob(wpa_s->conf, "private_key", network->security.client_key_data,
                                            network->security.client_key_len);

                    str_clear_free(ssid->eap.cert.private_key);
                    ssid->eap.cert.private_key = dup_binstr("blob://private_key", os_strlen("blob://private_key"));

                    str_clear_free(ssid->eap.cert.private_key_passwd);
                    ssid->eap.cert.private_key_passwd =
                        dup_binstr(network->security.client_key_passwd, os_strlen(network->security.client_key_passwd));
                }

                if (network->security.ca_cert2_len)
                {
                    wpa_config_process_blob(wpa_s->conf, "ca_cert2", network->security.ca_cert2_data,
                                            network->security.ca_cert2_len);

                    str_clear_free(ssid->eap.phase2_cert.ca_cert);
                    ssid->eap.phase2_cert.ca_cert = dup_binstr("blob://ca_cert2", os_strlen("blob://ca_cert2"));
                }
                else
                {
                    str_clear_free(ssid->eap.phase2_cert.ca_cert);
                    ssid->eap.phase2_cert.ca_cert = dup_binstr("blob://ca_cert", os_strlen("blob://ca_cert"));
                }

                if (network->security.client_cert2_len)
                {
                    wpa_config_process_blob(wpa_s->conf, "client_cert2", network->security.client_cert2_data,
                                            network->security.client_cert2_len);

                    str_clear_free(ssid->eap.phase2_cert.client_cert);
                    ssid->eap.phase2_cert.client_cert =
                        dup_binstr("blob://client_cert2", os_strlen("blob://client_cert2"));
                }
                else
                {
                    str_clear_free(ssid->eap.phase2_cert.client_cert);
                    ssid->eap.phase2_cert.client_cert =
                        dup_binstr("blob://client_cert", os_strlen("blob://client_cert"));
                }

                if (network->security.client_key2_len)
                {
                    wpa_config_process_blob(wpa_s->conf, "private_key2", network->security.client_key2_data,
                                            network->security.client_key2_len);

                    str_clear_free(ssid->eap.phase2_cert.private_key);
                    ssid->eap.phase2_cert.private_key =
                        dup_binstr("blob://private_key2", os_strlen("blob://private_key2"));

                    str_clear_free(ssid->eap.phase2_cert.private_key_passwd);
                    ssid->eap.phase2_cert.private_key_passwd = dup_binstr(
                        network->security.client_key2_passwd, os_strlen(network->security.client_key2_passwd));
                }
                else
                {
                    ssid->eap.phase2_cert.private_key =
                        dup_binstr("blob://private_key", os_strlen("blob://private_key"));
                    str_clear_free(ssid->eap.phase2_cert.private_key_passwd);
                    ssid->eap.phase2_cert.private_key_passwd =
                        dup_binstr(network->security.client_key_passwd, os_strlen(network->security.client_key_passwd));
                }

                str_clear_free(ssid->eap.pac_file);
                ssid->eap.pac_file = dup_binstr("blob://eap-fast-pac", os_strlen("blob://eap-fast-pac"));

#if CONFIG_EAP_TTLS
                if (network->security.type == WLAN_SECURITY_EAP_TTLS)
                {
                    ssid->eap.eap_methods->method = EAP_TYPE_TTLS;

                    const char *phase2 = "autheap=TLS";

                    str_clear_free(ssid->eap.phase2);
                    ssid->eap.phase2 = dup_binstr(phase2, os_strlen(phase2));
                }
#if CONFIG_EAP_MSCHAPV2
                if (network->security.type == WLAN_SECURITY_EAP_TTLS_MSCHAPV2)
                {
                    ssid->eap.eap_methods->method = EAP_TYPE_TTLS;

                    const char *phase2 = "auth=MSCHAPV2";

                    str_clear_free(ssid->eap.phase2);
                    ssid->eap.phase2 = dup_binstr(phase2, os_strlen(phase2));
                }
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
                if (network->security.type == WLAN_SECURITY_EAP_PEAP_MSCHAPV2)
                {
                    ssid->eap.eap_methods->method = EAP_TYPE_PEAP;

                    const char *phase2 = "auth=MSCHAPV2";

                    os_snprintf(phase1, sizeof(phase1), "peapver=%d peaplabel=%d crypto_binding=%d", network->security.eap_ver,
                                network->security.peap_label, network->security.eap_crypto_binding);

                    str_clear_free(ssid->eap.phase1);
                    ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));

                    str_clear_free(ssid->eap.phase2);
                    ssid->eap.phase2 = dup_binstr(phase2, os_strlen(phase2));
                }
#endif
#if CONFIG_EAP_TLS
                if (network->security.type == WLAN_SECURITY_EAP_PEAP_TLS)
                {
                    ssid->eap.eap_methods->method = EAP_TYPE_PEAP;

                    const char *phase2 = "auth=TLS";

                    os_snprintf(phase1, sizeof(phase1), "peapver=%d peaplabel=%d crypto_binding=%d", network->security.eap_ver,
                                network->security.peap_label, network->security.eap_crypto_binding);

                    str_clear_free(ssid->eap.phase1);
                    ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));

                    str_clear_free(ssid->eap.phase2);
                    ssid->eap.phase2 = dup_binstr(phase2, os_strlen(phase2));
                }
#endif
#if CONFIG_EAP_GTC
                if (network->security.type == WLAN_SECURITY_EAP_PEAP_GTC)
                {
                    ssid->eap.eap_methods->method = EAP_TYPE_PEAP;

                    const char *phase2 = "auth=GTC";

                    os_snprintf(phase1, sizeof(phase1), "peapver=%d peaplabel=%d crypto_binding=%d", network->security.eap_ver,
                                network->security.peap_label, network->security.eap_crypto_binding);

                    str_clear_free(ssid->eap.phase1);
                    ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));

                    str_clear_free(ssid->eap.phase2);
                    ssid->eap.phase2 = dup_binstr(phase2, os_strlen(phase2));
                }
#endif
#endif
#if CONFIG_EAP_SIM
                if (network->security.type == WLAN_SECURITY_EAP_SIM)
                {
                    os_snprintf(phase1, sizeof(phase1), "result_ind=%d", network->security.eap_result_ind);
                    str_clear_free(ssid->eap.phase1);
                    ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));

                    ssid->eap.eap_methods->method = EAP_TYPE_SIM;
                }
#endif
#if CONFIG_EAP_AKA
                if (network->security.type == WLAN_SECURITY_EAP_AKA)
                {
                    os_snprintf(phase1, sizeof(phase1), "result_ind=%d", network->security.eap_result_ind);
                    str_clear_free(ssid->eap.phase1);
                    ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));

                    ssid->eap.eap_methods->method = EAP_TYPE_AKA;
                }
#endif
#if CONFIG_EAP_AKA_PRIME
                if (network->security.type == WLAN_SECURITY_EAP_AKA_PRIME)
                {
                    os_snprintf(phase1, sizeof(phase1), "result_ind=%d", network->security.eap_result_ind);
                    str_clear_free(ssid->eap.phase1);
                    ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));

                    ssid->eap.eap_methods->method = EAP_TYPE_AKA_PRIME;
                }
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
                if (network->security.type == WLAN_SECURITY_EAP_FAST_MSCHAPV2)
                {
                    ssid->eap.eap_methods->method = EAP_TYPE_FAST;

                    int prov           = 3;
                    const char *phase2 = "auth=MSCHAPV2";

                    os_snprintf(phase1, sizeof(phase1), "fast_provisioning=%d fast_pac_format=binary", prov);

                    str_clear_free(ssid->eap.phase1);
                    ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));

                    str_clear_free(ssid->eap.phase2);
                    ssid->eap.phase2 = dup_binstr(phase2, os_strlen(phase2));
                }
#endif
#if CONFIG_EAP_GTC
                if (network->security.type == WLAN_SECURITY_EAP_FAST_GTC)
                {
                    ssid->eap.eap_methods->method = EAP_TYPE_FAST;

                    int prov           = 3;
                    const char *phase2 = "auth=GTC";

                    os_snprintf(phase1, sizeof(phase1), "fast_provisioning=%d fast_pac_format=binary", prov);

                    str_clear_free(ssid->eap.phase1);
                    ssid->eap.phase1 = dup_binstr(phase1, os_strlen(phase1));

                    str_clear_free(ssid->eap.phase2);
                    ssid->eap.phase2 = dup_binstr(phase2, os_strlen(phase2));
                }
#endif
#endif
                break;
#endif
#if CONFIG_WPA_SUPP_WPA3
            case WLAN_SECURITY_WPA3_SAE:
            case WLAN_SECURITY_WPA2_WPA3_SAE_MIXED:
            case WLAN_SECURITY_WPA3_SAE_EXT_KEY:
#if CONFIG_11R
            case WLAN_SECURITY_WPA3_FT_SAE:
                if (network->security.type == WLAN_SECURITY_WPA3_FT_SAE)
                {
                    if (network->role == WLAN_BSS_ROLE_UAP)
                    {
                        ret = -1;
                        goto out;
                    }
                }
                else
#endif
                    if (network->security.type == WLAN_SECURITY_WPA2_WPA3_SAE_MIXED)
                    {
                        str_clear_free(ssid->passphrase);
                        ssid->passphrase = dup_binstr(network->security.psk, network->security.psk_len);

                        if (ssid->passphrase == NULL)
                        {
                            wpa_printf(MSG_DEBUG, "%s:Failed to copy passphrase\n", __func__);
                            ret = -1;
                            goto out;
                        }
                        wpa_config_update_psk(ssid);

                    }

                if (wpa_parse_intlist(&wpa_s->conf->sae_groups, network->security.sae_groups))
                {
                    wpa_printf(MSG_ERROR, "Invalid sae_groups value");
                    return -1;
                }

                wpa_s->conf->sae_pwe     = network->security.pwe_derivation;
                ssid->transition_disable = network->security.transition_disable;
                str_clear_free(ssid->sae_password);
                ssid->sae_password = dup_binstr(network->security.password, network->security.password_len);

                if (ssid->sae_password == NULL)
                {
                    wpa_printf(MSG_DEBUG, "%s:Failed to copy sae_password\n", __func__);
                    ret = -1;
                    goto out;
                }
                break;
#endif
#ifdef CONFIG_OWE
            case WLAN_SECURITY_OWE_ONLY:
                ssid->owe_only = 1;
                break;
#endif
            case WLAN_SECURITY_WPA:
            case WLAN_SECURITY_WPA2:
            case WLAN_SECURITY_WPA_WPA2_MIXED:
#if CONFIG_11R
            case WLAN_SECURITY_WPA2_FT:
                if (network->security.type == WLAN_SECURITY_WPA2_FT)
                {
                    if (network->role == WLAN_BSS_ROLE_UAP)
                    {
                        ret = -1;
                        goto out;
                    }
                }
#endif
                ssid->group_cipher    = WPA_CIPHER_CCMP | WPA_CIPHER_TKIP;
                ssid->proto           = WPA_PROTO_RSN;
                ssid->pairwise_cipher = WPA_CIPHER_CCMP;

                if ((network->security.type == WLAN_SECURITY_WPA) || (network->security.type == WLAN_SECURITY_WPA_WPA2_MIXED))
                {
                    ssid->proto           |= WPA_PROTO_WPA;
                    ssid->group_cipher    |= WPA_CIPHER_TKIP;
                    ssid->pairwise_cipher |= WPA_CIPHER_TKIP;
                }

                if (network->security.type == WLAN_SECURITY_WPA2)
                {
                    ssid->group_cipher    |= WPA_CIPHER_TKIP;
                }

                if (network->security.pmk_valid)
                {
                    os_memcpy(ssid->psk, network->security.pmk, PMK_LEN);
                    ssid->psk_set = 1;
                }
                else
                {
                    str_clear_free(ssid->passphrase);
                    ssid->passphrase = dup_binstr(network->security.psk, network->security.psk_len);

                    if (ssid->passphrase == NULL)
                    {
                        wpa_printf(MSG_DEBUG, "%s:Failed to copy passphrase\n", __func__);
                        ret = -1;
                        goto out;
                    }
                    wpa_config_update_psk(ssid);
                }
                break;
            default:
                break;
        }

        if (pmf)
        {
            if ((network->security.mfpc) && (network->security.mfpr))
                ssid->ieee80211w = MGMT_FRAME_PROTECTION_REQUIRED;
            else if (network->security.mfpc)
                ssid->ieee80211w = MGMT_FRAME_PROTECTION_OPTIONAL;
        }

        wpa_sm_pmksa_cache_flush(wpa_s->wpa, ssid);

        wpa_config_update_prio_list(wpa_s->conf);

        network->id = ssid->id;

        wpa_supp_api_ctrl.dev          = dev;
        wpa_supp_api_ctrl.requested_op = ADD;

#if CONFIG_HOSTAPD
    }
#endif

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

static void wpa_supp_scan_res_fail_handler(struct wpa_supplicant *wpa_s)
{
    if (wpa_s->scan_work)
    {
        struct wpa_radio_work *work = wpa_s->scan_work;
        wpa_s->scan_work            = NULL;
        radio_work_done(work);
    }

    wpa_msg(wpa_s, MSG_INFO, WPA_EVENT_SCAN_FAILED "ret=-1");
}

int wpa_supp_connect(const struct netif *dev, struct wlan_network *network)
{
    struct wpa_ssid *ssid = NULL, *ssid_prev = NULL;
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);
rt_kprintf("%s %d\n", __func__, __LINE__);
    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    ssid = ssid_prev = wpa_s->conf->ssid;
rt_kprintf("%s %d\n", __func__, __LINE__);
    while (ssid)
    {
        if (network->id == ssid->id)
        {
            break;
        }
        ssid_prev = ssid;
        ssid = ssid->next;
    }
    if (ssid == NULL)
    {
        ret = -1;
        goto out;
    }
rt_kprintf("%s %d\n", __func__, __LINE__);
    /* the specific connect ssid isn't in the foremost */
    if (ssid != ssid_prev)
    {
       /* remove the specific connect ssid to the foremost in the conf->ssid list */
      ssid_prev->next = ssid->next;
      ssid->next = wpa_s->conf->ssid;
      wpa_s->conf->ssid = ssid;
    }
rt_kprintf("%s %d\n", __func__, __LINE__);
    if (ssid && ssid == wpa_s->current_ssid && wpa_s->current_ssid && wpa_s->wpa_state >= WPA_AUTHENTICATING)
    {
        /* We are already associated with the selected network */
        wpa_printf(MSG_DEBUG,
                   "Already associated with the "
                   "selected network - do Deauthentication");
        wpa_supplicant_deauthenticate(wpa_s, WLAN_REASON_DEAUTH_LEAVING);//删除认证
        goto out;
    }
rt_kprintf("%s %d\n", __func__, __LINE__);
    wpa_s->scan_min_time.sec  = 0;
    wpa_s->scan_min_time.usec = 0;

    //wpa_supplicant_enable_network(wpa_s, ssid);
rt_kprintf("%s %d\n", __func__, __LINE__);
    wpa_supplicant_select_network(wpa_s, ssid);
rt_kprintf("%s %d\n", __func__, __LINE__);
    send_wpa_supplicant_dummy_event();
rt_kprintf("%s %d\n", __func__, __LINE__);
    wpa_supp_api_ctrl.dev          = dev;
    wpa_supp_api_ctrl.requested_op = CONNECT;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_disable(const struct netif *dev, struct wlan_network *network)
{
    struct wpa_ssid *ssid = NULL;
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    ssid = wpa_s->conf->ssid;

    while (ssid)
    {
        if (network->id == ssid->id)
        {
            break;
        }
        ssid = ssid->next;
    }

    if (ssid == NULL)
    {
        ret = -1;
        goto out;
    }

    wpa_supplicant_disable_network(wpa_s, ssid);

    send_wpa_supplicant_dummy_event();

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_disconnect(const struct netif *dev)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }
#if 0
    sme_clear_on_disassoc(wpa_s);

    wpabuf_free(wpa_s->pending_eapol_rx);
    wpa_s->pending_eapol_rx = NULL;

#if CONFIG_SAE
    os_free(wpa_s->sme.sae_rejected_groups);
    wpa_s->sme.sae_rejected_groups = NULL;
#endif /* CONFIG_SAE */

    os_free(wpa_s->next_scan_freqs);
    wpa_s->next_scan_freqs = NULL;

    os_free(wpa_s->manual_scan_freqs);
    wpa_s->manual_scan_freqs = NULL;

    os_free(wpa_s->select_network_scan_freqs);
    wpa_s->select_network_scan_freqs = NULL;

    os_free(wpa_s->manual_sched_scan_freqs);
    wpa_s->manual_sched_scan_freqs = NULL;

    os_free(wpa_s->last_scan_res);
    wpa_s->last_scan_res = NULL;
#endif

    wpa_s->scan_res_fail_handler = NULL;

    wpa_config_remove_blob(wpa_s->conf, "eap-fast-pac");

    eapol_sm_invalidate_cached_session(wpa_s->eapol);
    wpas_request_disconnection(wpa_s);

    send_wpa_supplicant_dummy_event();

    wpa_supp_api_ctrl.dev          = dev;
    wpa_supp_api_ctrl.requested_op = DISCONNECT;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_remove_network(const struct netif *dev, struct wlan_network *network)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;
#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;
#endif

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (network->role == WLAN_BSS_ROLE_UAP)
    {
        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s || network->id == -1)
        {
            ret = -1;
            goto out;
        }

        switch (network->security.type)
        {
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
            case WLAN_SECURITY_EAP_TLS:
            case WLAN_SECURITY_EAP_TLS_SHA256:
#if CONFIG_11R
            case WLAN_SECURITY_EAP_TLS_FT:
            case WLAN_SECURITY_EAP_TLS_FT_SHA384:
#endif
#endif
#if CONFIG_EAP_TTLS
            case WLAN_SECURITY_EAP_TTLS:
#if CONFIG_EAP_MSCHAPV2
            case WLAN_SECURITY_EAP_TTLS_MSCHAPV2:
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
            case WLAN_SECURITY_EAP_PEAP_MSCHAPV2:
#endif
#if CONFIG_EAP_TLS
            case WLAN_SECURITY_EAP_PEAP_TLS:
#endif
#if CONFIG_EAP_GTC
            case WLAN_SECURITY_EAP_PEAP_GTC:
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
            case WLAN_SECURITY_EAP_FAST_MSCHAPV2:
#endif
#if CONFIG_EAP_GTC
            case WLAN_SECURITY_EAP_FAST_GTC:
#endif
#endif
#if CONFIG_EAP_SIM
            case WLAN_SECURITY_EAP_SIM:
#endif
#if CONFIG_EAP_AKA
            case WLAN_SECURITY_EAP_AKA:
#endif
#if CONFIG_EAP_AKA_PRIME
            case WLAN_SECURITY_EAP_AKA_PRIME:
#endif
                wpa_config_remove_blob(wpa_s->conf, "ca_cert");
                wpa_config_remove_blob(wpa_s->conf, "client_cert");
                wpa_config_remove_blob(wpa_s->conf, "private_key");
                wpa_config_remove_blob(wpa_s->conf, "ca_cert2");
                wpa_config_remove_blob(wpa_s->conf, "cloent_cert2");
                wpa_config_remove_blob(wpa_s->conf, "private_key2");
                wpa_config_remove_blob(wpa_s->conf, "eap-fast-pac");
                break;
#endif
            default:
                break;
        }

        ret = wpa_supplicant_remove_network(wpa_s, network->id);
        if (ret == -1)
        {
            wpa_printf(MSG_DEBUG,
                    "Could not find network "
                    "id=%d",
                    network->id);
            ret = -1;
            goto out;
        }
        if (ret == -2)
        {
            wpa_printf(MSG_DEBUG,
                    "Not able to remove the "
                    "network id=%d",
                    network->id);
            ret = -1;
            goto out;
        }

        ret = 0;

        wpa_supp_api_ctrl.dev          = dev;
        wpa_supp_api_ctrl.requested_op = REMOVE;

#if CONFIG_HOSTAPD
    }
#endif

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}


int wpa_supp_pmksa_list(const struct netif *dev, char *buf, size_t buflen)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;
    size_t reply_len = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    reply_len = wpa_sm_pmksa_cache_list(wpa_s->wpa, buf, buflen);
#if CONFIG_AP
    reply_len += wpas_ap_pmksa_cache_list(wpa_s, &buf[reply_len], buflen - reply_len);
#endif /* CONFIG_AP */
    
    if (reply_len == 0)
    {
        ret = -1;
    }

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_pmksa_flush(const struct netif *dev)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    pmksa_cache_clear_current(wpa_s->wpa);
    ptksa_cache_flush(wpa_s->ptksa, NULL, WPA_CIPHER_NONE);
    wpa_sm_pmksa_cache_flush(wpa_s->wpa, NULL);
#if CONFIG_AP
    wpas_ap_pmksa_cache_flush(wpa_s);
#endif /* CONFIG_AP */

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}
/*
触发 WPA Supplicant 进行一次重新关联（reassociate）操作，即断开当前连接并尝试重新连接到配置的 Wi-Fi 网络
*/
int wpa_supp_reassociate(const struct netif *dev)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }
    if (wpa_s->driver->deauthenticate) {
        wpa_s->driver->deauthenticate(wpa_s->drv_priv, wpa_s->bssid, WLAN_REASON_PREV_AUTH_NOT_VALID);
    }
    rt_kprintf("%s %d\n", __func__, __LINE__);
    wpas_request_connection(wpa_s);
rt_kprintf("%s %d\n", __func__, __LINE__);
    send_wpa_supplicant_dummy_event();
rt_kprintf("%s %d\n", __func__, __LINE__);
    wpa_supp_api_ctrl.dev          = dev;
    wpa_supp_api_ctrl.requested_op = RECONNECT;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_set_autoconnect(const struct netif *dev, int enable)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    wpa_s->auto_reconnect_disabled = !enable;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_set_scan_interval(const struct netif *dev, int scan_int)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    ret = wpa_supplicant_set_scan_interval(wpa_s, scan_int);

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

void wpa_supp_set_ap_max_num_sta(const struct netif *dev, unsigned int max_num_sta)
{
    h_max_num_sta = (int)max_num_sta;
}

void wpa_supp_set_ap_beacon_int(const struct netif *dev, unsigned short beacon_int)
{
    h_beacon_int = beacon_int;
}

void wpa_supp_set_ap_bw(const struct netif *dev, unsigned char bw)
{
    bandwidth = bw;
}

void wpa_supp_set_ap_hidden_ssid(const struct netif *dev, rt_uint8_t hidden_ssid)
{
    h_hidden_ssid = hidden_ssid;
}

int wpa_supp_set_ap_country(const struct netif *dev, const char *country, const unsigned char country3)
{
    int ret = 0;

#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;
    struct hostapd_config *conf;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        ret = -1;
        goto out;
    }

    conf = hapd_s->conf;

    conf->country[0] = country[0];
    conf->country[1] = country[1];
    conf->country[2] = country3;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

#else
    struct wpa_ssid *ssid = NULL;
    struct wpa_supplicant *wpa_s;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    wpa_s->conf->country[0] = country[0];
    wpa_s->conf->country[1] = country[1];
    wpa_s->conf->country[2] = country3;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
#endif

    return ret;
}

void wpa_supp_set_ap_11d_state(const struct netif *dev, int state)
{
#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;
    struct hostapd_config *conf;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        goto out;
    }

    conf = hapd_s->conf;
    conf->ieee80211d = state ? 1 : 0;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
#endif
}

int wpa_supp_start_ap(const struct netif *dev, struct wlan_network *network, int reload)
{
    int ret = 0;

#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;
    struct hostapd_config *conf;
    struct hostapd_bss_config *bss;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        ret = -1;
        goto out;
    }

    ret = hostapd_update_bss(hapd_s, network);
    if (ret != 0)
    {
        ret = -1;
        goto out;
    }

    conf = hapd_s->conf;
    bss  = conf->last_bss;

    bss->ignore_broadcast_ssid = h_hidden_ssid;
    conf->beacon_int = h_beacon_int;
    bss->max_num_sta = h_max_num_sta;
    if(!bss->ignore_broadcast_ssid)
    {
        bss->wps_state = WPS_STATE_CONFIGURED;
    }

    if (bandwidth == 1)
    {
        conf->secondary_channel = 0;
        conf->ht_capab &= ~HT_CAP_INFO_SUPP_CHANNEL_WIDTH_SET;
        conf->ht_capab &= ~HT_CAP_INFO_SHORT_GI40MHZ;

#if CONFIG_11AC
        conf->vht_oper_chwidth             = CHANWIDTH_USE_HT;
        conf->vht_oper_centr_freq_seg0_idx = network->channel;
#endif
#if CONFIG_11AX
        conf->he_oper_chwidth             = CHANWIDTH_USE_HT;
        conf->he_oper_centr_freq_seg0_idx = network->channel;
#endif
    }
    else if (bandwidth == 2)
    {
        conf->ht_capab |= HT_CAP_INFO_SUPP_CHANNEL_WIDTH_SET;
        conf->ht_capab |= HT_CAP_INFO_SHORT_GI40MHZ;

#if CONFIG_11AC
        conf->vht_oper_chwidth             = CHANWIDTH_USE_HT;
        if (network->channel >= 36 && network->channel <= 40)
        {
            conf->vht_oper_centr_freq_seg0_idx = 38;
        }
        else if (network->channel >= 44 && network->channel <= 48)
        {
            conf->vht_oper_centr_freq_seg0_idx = 46;
        }
        else if (network->channel >= 149 && network->channel <= 153)
        {
            conf->vht_oper_centr_freq_seg0_idx = 151;
        }
        else if (network->channel >= 157 && network->channel <= 161)
        {
            conf->vht_oper_centr_freq_seg0_idx = 159;
        }
        else if (network->channel >= 165 && network->channel <= 169)
        {
            conf->vht_oper_centr_freq_seg0_idx = 167;
        }
        else if (network->channel >= 173 && network->channel <= 177)
        {
            conf->vht_oper_centr_freq_seg0_idx = 175;
        }
#endif
#if CONFIG_11AX
        conf->he_oper_chwidth             = CHANWIDTH_USE_HT;
        if (network->channel >= 36 && network->channel <= 40)
        {
            conf->he_oper_centr_freq_seg0_idx = 38;
        }
        else if (network->channel >= 44 && network->channel <= 48)
        {
            conf->he_oper_centr_freq_seg0_idx = 46;
        }
        else if (network->channel >= 149 && network->channel <= 153)
        {
            conf->he_oper_centr_freq_seg0_idx = 151;
        }
        else if (network->channel >= 157 && network->channel <= 161)
        {
            conf->he_oper_centr_freq_seg0_idx = 159;
        }
        else if (network->channel >= 165 && network->channel <= 169)
        {
            conf->he_oper_centr_freq_seg0_idx = 167;
        }
        else if (network->channel >= 173 && network->channel <= 177)
        {
            conf->he_oper_centr_freq_seg0_idx = 175;
        }
#endif
    }
    else if (bandwidth == 3)
    {
        conf->ht_capab |= HT_CAP_INFO_SUPP_CHANNEL_WIDTH_SET;
        conf->ht_capab |= HT_CAP_INFO_SHORT_GI40MHZ;

#if CONFIG_11AC
        conf->vht_oper_chwidth             = CHANWIDTH_80MHZ;

        if (network->channel >= 36 && network->channel <= 48)
        {
            conf->vht_oper_centr_freq_seg0_idx = 42;
        }
        else if (network->channel >= 149 && network->channel <= 161)
        {
            conf->vht_oper_centr_freq_seg0_idx = 155;
        }
        else if (network->channel >= 165 && network->channel <= 177)
        {
            conf->vht_oper_centr_freq_seg0_idx = 171;
        }
#endif
#if CONFIG_11AX
        conf->he_oper_chwidth             = CHANWIDTH_80MHZ;
        if (network->channel >= 36 && network->channel <= 48)
        {
            conf->he_oper_centr_freq_seg0_idx = 42;
        }
        else if (network->channel >= 149 && network->channel <= 161)
        {
            conf->he_oper_centr_freq_seg0_idx = 155;
        }
        else if (network->channel >= 165 && network->channel <= 177)
        {
            conf->he_oper_centr_freq_seg0_idx = 171;
        }
#endif
    }

    if (reload)
    {
        ret                            = hostapd_reload_iface(hapd_s);
    }
    else
    {
        ret                            = hostapd_enable_iface(hapd_s);
    }
    wpa_supp_api_ctrl.dev          = dev;
    wpa_supp_api_ctrl.requested_op = START;
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

#else
    struct wpa_ssid *ssid = NULL;
    struct wpa_supplicant *wpa_s;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    ssid = wpa_s->conf->ssid;

    while (ssid)
    {
        if (network->id == ssid->id)
        {
            break;
        }
        ssid = ssid->next;
    }

    if (ssid == NULL)
    {
        ret = -1;
        goto out;
    }

    wpa_s->connect_without_scan = ssid;

    wpa_supplicant_enable_network(wpa_s, ssid);

    wpa_supplicant_select_network(wpa_s, ssid);

    send_wpa_supplicant_dummy_event();

    wpa_supp_api_ctrl.dev = dev;
    wpa_supp_api_ctrl.requested_op = START;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
#endif

    return ret;
}

int wpa_supp_stop_ap(const struct netif *dev, struct wlan_network *network)

{
    int ret = 0;

#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        ret = -1;
        goto out;
    }

    ret = hostapd_disable_iface(hapd_s);

    bandwidth = 2;
    h_hidden_ssid = 0;
    h_beacon_int = 100;
    h_max_num_sta = 8;

    ret = hostapd_free_bss(hapd_s, network);
    if (ret != 0)
    {
        ret = -1;
        goto out;
    }

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

#else
    struct wpa_supplicant *wpa_s;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }
#if CONFIG_AP
    ret = wpas_ap_stop_ap(wpa_s);
#endif
    // wpa_supplicant_ap_deinit(wpa_s);

    wpa_supp_api_ctrl.dev = dev;
    wpa_supp_api_ctrl.requested_op = STOP;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
#endif

    return ret;
}

void wpa_supp_set_bgscan(const struct netif *dev, const int short_interval, const int signal_threshold, const int long_interval)
{
    struct wpa_supplicant *wpa_s;
    char bgscan_str[128] = {0};

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        goto out;
    }

    snprintf(bgscan_str, 128, "simple:%d:%d:%d", short_interval, signal_threshold, long_interval);

    os_free(wpa_s->conf->bgscan);

    wpa_s->conf->bgscan = os_strdup(bgscan_str);

    //wpa_supplicant_reset_bgscan(wpa_s);

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
}

int wpa_supp_set_okc(const struct netif *dev, unsigned char okc)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    wpa_s->conf->okc = okc;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_roam(const struct netif *dev, unsigned char *bssid)
{
    struct wpa_supplicant *wpa_s;
    // struct wpa_bss *bss;
    // struct wpa_ssid *ssid;
    struct wpa_radio_work *already_connecting;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

#if 0
    ssid = wpa_s->current_ssid;

    wpa_printf(MSG_DEBUG, "ROAM " MACSTR, MAC2STR(bssid));

    if (!ssid)
    {
        wpa_printf(MSG_DEBUG,
                   "ROAM: No network "
                   "configuration known for the target AP");
        ret = -1;
        goto out;
    }

    bss = wpa_bss_get(wpa_s, bssid, ssid->ssid, ssid->ssid_len);
    if (!bss)
    {
        wpa_printf(MSG_DEBUG,
                   "ROAM: Target AP not found "
                   "from BSS table");
        ret = -1;
        goto out;
    }

    /*
     * TODO: Find best network configuration block from configuration to
     * allow roaming to other networks
     */
#endif
    already_connecting       = radio_work_pending(wpa_s, "sme-connect");
    wpa_s->own_reconnect_req = 1;
    // wpa_supplicant_connect(wpa_s, bss, ssid);

    send_wpa_supplicant_dummy_event();

    /*
     * Indicate that an explicitly requested roam is in progress so scan
     * results that come in before the 'sme-connect' radio work gets
     * executed do not override the original connection attempt.
     */
    if (!already_connecting && radio_work_pending(wpa_s, "sme-connect"))
        wpa_s->roam_in_progress = true;

    wpa_supp_api_ctrl.dev          = dev;
    wpa_supp_api_ctrl.requested_op = ROAM;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

static void wpas_neighbor_rep_cb(void *ctx, struct wpabuf *neighbor_rep)
{
    struct wpa_supplicant *wpa_s = ctx;
    size_t len;
    const u8 *data;

    /*
     * Neighbor Report element (IEEE P802.11-REVmc/D5.0)
     * BSSID[6]
     * BSSID Information[4]
     * Operating Class[1]
     * Channel Number[1]
     * PHY Type[1]
     * Optional Subelements[variable]
     */
#define NR_IE_MIN_LEN (ETH_ALEN + 4 + 1 + 1 + 1)

    if (!neighbor_rep || wpabuf_len(neighbor_rep) == 0)
    {
        wpa_msg_ctrl(wpa_s, MSG_INFO, RRM_EVENT_NEIGHBOR_REP_FAILED);
        goto out;
    }

    data = wpabuf_head_u8(neighbor_rep);
    len  = wpabuf_len(neighbor_rep);

    while (len >= 2 + NR_IE_MIN_LEN)
    {
        const u8 *nr;
        char lci[256 * 2 + 1];
        char civic[256 * 2 + 1];
        u8 nr_len     = data[1];
        const u8 *pos = data, *end;

        if (pos[0] != WLAN_EID_NEIGHBOR_REPORT || nr_len < NR_IE_MIN_LEN)
        {
            wpa_dbg(wpa_s, MSG_DEBUG, "CTRL: Invalid Neighbor Report element: id=%u len=%u", data[0], nr_len);
            goto out;
        }

        if (2U + nr_len > len)
        {
            wpa_dbg(wpa_s, MSG_DEBUG, "CTRL: Invalid Neighbor Report element: id=%u len=%zu nr_len=%u", data[0], len,
                    nr_len);
            goto out;
        }
        pos += 2;
        end = pos + nr_len;

        nr = pos;
        pos += NR_IE_MIN_LEN;

        lci[0]   = '\0';
        civic[0] = '\0';
        while (end - pos > 2)
        {
            u8 s_id, s_len;

            s_id  = *pos++;
            s_len = *pos++;
            if (s_len > end - pos)
                goto out;
            if (s_id == WLAN_EID_MEASURE_REPORT && s_len > 3)
            {
                /* Measurement Token[1] */
                /* Measurement Report Mode[1] */
                /* Measurement Type[1] */
                /* Measurement Report[variable] */
                switch (pos[2])
                {
                    case MEASURE_TYPE_LCI:
                        if (lci[0])
                            break;
                        wpa_snprintf_hex(lci, sizeof(lci), pos, s_len);
                        break;
                    case MEASURE_TYPE_LOCATION_CIVIC:
                        if (civic[0])
                            break;
                        wpa_snprintf_hex(civic, sizeof(civic), pos, s_len);
                        break;
                }
            }

            pos += s_len;
        }

        wpa_msg_ctrl(wpa_s, MSG_INFO,
                     RRM_EVENT_NEIGHBOR_REP_RXED "bssid=" MACSTR " info=0x%x op_class=%u chan=%u phy_type=%u%s%s%s%s",
                     MAC2STR(nr), WPA_GET_LE32(nr + ETH_ALEN), nr[ETH_ALEN + 4], nr[ETH_ALEN + 5], nr[ETH_ALEN + 6],
                     lci[0] ? " lci=" : "", lci, civic[0] ? " civic=" : "", civic);

        data = end;
        len -= 2 + nr_len;
    }
#if CONFIG_11K
    wpa_msg_ctrl(wpa_s, MSG_INFO, RRM_EVENT_NEIGHBOR_REP_COMPLETED);
#endif
out:
    wpabuf_free(neighbor_rep);
}

int wpa_supp_send_neighbor_rep(const struct netif *dev, char *pssid, int lci, int civic)
{
    struct wpa_supplicant *wpa_s;
    struct wpa_ssid_value ssid, *ssid_p = NULL;
    int ret;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (wpa_s == NULL)
    {
        ret = -1;
        goto out;
    }

    if (pssid != NULL)
    {
        os_memset(&ssid, 0x00, sizeof(struct wpa_ssid_value));
        ssid.ssid_len = os_strlen(pssid);
        os_memcpy(ssid.ssid, pssid, ssid.ssid_len);
        ssid_p = &ssid;
    }

    ret = wpas_rrm_send_neighbor_rep_request(wpa_s, ssid_p, lci, civic, wpas_neighbor_rep_cb, wpa_s);

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_send_btm_query(const struct netif *dev, int query_reason)
{
    struct wpa_supplicant *wpa_s;
    int ret;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    ret = wnm_send_bss_transition_mgmt_query(wpa_s, query_reason, NULL, 0);

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

#ifdef CONFIG_MBO
int wpa_supp_mbo_update_non_pref_chan(const struct netif *dev, const char *non_pref_chan)
{
    struct wpa_supplicant *wpa_s;
    int ret;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    ret = wpas_mbo_update_non_pref_chan(wpa_s, non_pref_chan);

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_mbo_set_cell_capa(const struct netif *dev, unsigned char mbo_cell_capa)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    wpas_mbo_update_cell_capa(wpa_s, mbo_cell_capa);

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_mbo_set_oce(const struct netif *dev, unsigned char oce)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    wpa_s->conf->oce = oce;
    if (wpa_s->conf->oce)
    {
        if ((wpa_s->conf->oce & OCE_STA) && (wpa_s->drv_flags & WPA_DRIVER_FLAGS_OCE_STA))
            wpa_s->enable_oce = OCE_STA;

        if ((wpa_s->conf->oce & OCE_STA_CFON) && (wpa_s->drv_flags & WPA_DRIVER_FLAGS_OCE_STA_CFON))
        {
            /* TODO: Need to add STA-CFON support */
            wpa_printf(MSG_DEBUG, "OCE STA-CFON feature is not yet supported");
            ret = -1;
        }
    }
    else
    {
        wpa_s->enable_oce = 0;
    }
    wpa_supplicant_set_default_scan_ies(wpa_s);

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}
#endif

#if CONFIG_11R
int wpa_supp_ft_ds(const struct netif *dev, unsigned char *bssid)
{
    struct wpa_supplicant *wpa_s;
    struct wpa_bss *bss;
    const u8 *mdie;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }

    bss = wpa_bss_get_bssid(wpa_s, bssid);
    if (bss)
        mdie = wpa_bss_get_ie(bss, WLAN_EID_MOBILITY_DOMAIN);
    else
        mdie = NULL;

    ret = wpa_ft_start_over_ds(wpa_s->wpa, bssid, mdie);

    wpa_supp_api_ctrl.dev          = dev;
    wpa_supp_api_ctrl.requested_op = FT_DS;

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}
#endif /* CONFIG_11R */

int wpa_supp_notify_assoc(const struct netif *dev)
{
    struct wpa_supplicant *wpa_s;
    struct wpa_bss *bss;
    const u8 *mdie;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }
#if CONFIG_11R
    /* If FT is configured and if ft re-assoc is not completed then
     * no need to notify assoc.
     */
    if (wpa_key_mgmt_ft(wpa_s->wpa->key_mgmt) && !(wpa_s->wpa->ft_reassoc_completed))
        goto out;
#endif
    wpa_sm_notify_assoc(wpa_s->wpa, wpa_s->current_bss->bssid);

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

void wpa_supp_notify_acs(const struct netif *dev)
{
#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;
    struct hostapd_config *conf;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        goto out;
    }

    conf = hapd_s->conf;
    conf->ht_capab &= ~HT_CAP_INFO_SHORT_GI40MHZ;
    conf->vht_capab &= ~VHT_CAP_SHORT_GI_80;
#if CONFIG_11AX
    conf->he_oper_chwidth = -1;
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
#endif
}

#if CONFIG_HOSTAPD
int wpa_supp_get_sta_info(const struct netif *dev, unsigned char *sta_addr, unsigned char *is_11n_enabled)
{
    // struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    struct hostapd_iface *hapd_s;
    struct hostapd_data *hapd;
    struct sta_info *sta;

    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        ret = -1;
        goto out;
    }
    hapd = hapd_s->bss[0];

    sta = ap_get_sta(hapd, sta_addr);
    if (!sta)
    {
        ret = -1;
        goto out;
    }

    if (sta->flags & WLAN_STA_HT)
    {
        *is_11n_enabled = 1;
    }
    else
    {
        *is_11n_enabled = 0;
    }

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}
#endif

#if CONFIG_WPA_SUPP_WPS
int wpa_supp_start_wps_pbc(const struct netif *dev, int is_ap)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];

        ret = hostapd_wps_button_pushed(hapd, NULL);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = wpa_supplicant_ap_wps_pbc(wpa_s, NULL, NULL);
            goto out;
        }
#endif /* CONFIG_AP */

        ret = wpas_wps_start_pbc(wpa_s, NULL, 0, 0);

        wpa_supp_api_ctrl.dev          = dev;
        wpa_supp_api_ctrl.requested_op = WPS_PBC;
#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_start_wps_pin(const struct netif *dev, const char *pin, int is_ap)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];

        ret = hostapd_wps_add_pin(hapd, NULL, "any", pin, 0);
        if (ret != 0)
        {
            ret = -1;
            goto out;
        }
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = wpa_supplicant_ap_wps_pin(wpa_s, NULL, pin, NULL, 0, 0);
            goto out;
        }
#endif /* CONFIG_AP */

        if (pin)
        {
            ret = wpas_wps_start_pin(wpa_s, NULL, pin, 0, DEV_PW_DEFAULT);
            goto out;
        }

        ret = wpas_wps_start_pin(wpa_s, NULL, NULL, 0, DEV_PW_DEFAULT);

        wpa_supp_api_ctrl.dev          = dev;
        wpa_supp_api_ctrl.requested_op = WPS_PIN;
#if CONFIG_HOSTAPD
    }
#endif

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_wps_pin_valid(const struct netif *dev, const unsigned char *tpin)
{
    (void)dev;

    char pin[9];
    size_t len;
    char *pos;

    wpa_hexdump_ascii_key(MSG_DEBUG, "WPS_CHECK_PIN", (u8 *)tpin, os_strlen((const char *)tpin));
    for (pos = (char *)tpin, len = 0; *pos != '\0'; pos++)
    {
        if (*pos < '0' || *pos > '9')
            continue;
        pin[len++] = *pos;
        if (len == 9)
        {
            wpa_printf(MSG_DEBUG, "WPS: Too long PIN");
            return -1;
        }
    }
    if (len != 4 && len != 8)
    {
        wpa_printf(MSG_DEBUG, "WPS: Invalid PIN length %d", (int)len);
        return -1;
    }
    pin[len] = '\0';

    if (len == 8)
    {
        unsigned int pin_val;
        pin_val = atoi(pin);
        if (!wps_pin_valid(pin_val))
        {
            wpa_printf(MSG_DEBUG, "WPS: Invalid checksum digit");
            return -1;
        }
    }

    return 0;
}

int wpa_supp_wps_generate_pin(const struct netif *dev, unsigned int *pin)
{
    (void)dev;

    return wps_generate_pin(pin);
}

int wpa_supp_cancel_wps(const struct netif *dev, int is_ap)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];

        ret = hostapd_wps_cancel(hapd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

        ret = wpas_wps_cancel(wpa_s);

        wpa_supp_api_ctrl.dev          = dev;
        wpa_supp_api_ctrl.requested_op = WPS_CANCEL;
#if CONFIG_HOSTAPD
    }
#endif

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}
#endif

#ifdef CONFIG_DPP
int wpa_supp_dpp_bootstrap_gen(const struct netif *dev, int is_ap, const char *buf)
{
    struct wpa_supplicant *wpa_s;
    int id = -1;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            goto out;
        }
        hapd = hapd_s->bss[0];
        id = dpp_bootstrap_gen(hapd->iface->interfaces->dpp, buf);
        if (id < 0)
        {
            wpa_printf(MSG_DEBUG, "%s:Failed to generate bootstrap\n", __func__);
        }
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            goto out;
        }
#endif /* CONFIG_AP */

        id = dpp_bootstrap_gen(wpa_s->dpp, buf);
        if (id < 0)
        {
            wpa_printf(MSG_DEBUG, "%s:Failed to generate bootstrap\n", __func__);
        }

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return id;
}

const char *wpa_supp_dpp_bootstrap_get_uri(const struct netif *dev, int is_ap, unsigned int id)
{
    struct wpa_supplicant *wpa_s;
    const char *uri = NULL;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            goto out;
        }
        hapd = hapd_s->bss[0];
        uri = dpp_bootstrap_get_uri(hapd->iface->interfaces->dpp, id);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            goto out;
        }
#endif /* CONFIG_AP */

        uri = dpp_bootstrap_get_uri(wpa_s->dpp, id);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return uri;
}

int wpa_supp_dpp_listen(const struct netif *dev, int is_ap, const char *cmd)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        ret = hostapd_dpp_listen(hapd, cmd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */

        wpa_s->conf->dpp_config_processing = 2;
        ret = wpas_dpp_listen(wpa_s, cmd);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_dpp_stop_listen(const struct netif *dev, int is_ap)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        hostapd_dpp_listen_stop(hapd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */

        wpas_dpp_listen_stop(wpa_s);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_dpp_configurator_add(const struct netif *dev, int is_ap, const char *cmd)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        ret = dpp_configurator_add(hapd->iface->interfaces->dpp, cmd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */

        ret = dpp_configurator_add(wpa_s->dpp, cmd);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

void wpa_supp_dpp_configurator_params(const struct netif *dev, int is_ap, const char *cmd)
{
    struct wpa_supplicant *wpa_s;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            goto out;
        }
        hapd = hapd_s->bss[0];
        os_free(hapd->dpp_configurator_params);
        hapd->dpp_configurator_params = os_strdup(cmd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            goto out;
        }
#endif /* CONFIG_AP */

        os_free(wpa_s->dpp_configurator_params);
        wpa_s->dpp_configurator_params = os_strdup(cmd);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
}

void wpa_supp_dpp_mud_url(const struct netif *dev, int is_ap, const char *cmd)
{
    struct wpa_supplicant *wpa_s;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s || !hapd_s->conf)
        {
            goto out;
        }
        hapd = hapd_s->bss[0];
        os_free(hapd->conf->dpp_mud_url);
        hapd->conf->dpp_mud_url = os_strdup(cmd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s || !wpa_s->conf)
        {
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            goto out;
        }
#endif /* CONFIG_AP */

        os_free(wpa_s->conf->dpp_mud_url);
        wpa_s->conf->dpp_mud_url = os_strdup(cmd);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
}

int wpa_supp_dpp_configurator_get_key(const struct netif *dev, int is_ap, unsigned int id, char *buf, size_t buflen)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        ret = dpp_configurator_get_key_id(hapd->iface->interfaces->dpp, id, buf, buflen);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */

        ret = dpp_configurator_get_key_id(wpa_s->dpp, id, buf, buflen);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_dpp_qr_code(const struct netif *dev, int is_ap, char *cmd)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        ret = hostapd_dpp_qr_code(hapd, cmd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */

        ret = wpas_dpp_qr_code(wpa_s, cmd);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_dpp_auth_init(const struct netif *dev, int is_ap, const char *cmd)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        hapd->dpp_resp_wait_time = 5000;
        ret = hostapd_dpp_auth_init(hapd, cmd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */
        wpa_s->dpp_resp_wait_time = 5000;
        wpa_s->conf->dpp_config_processing = 2;
        ret = wpas_dpp_auth_init(wpa_s, cmd);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_dpp_pkex_add(const struct netif *dev, int is_ap, const char *cmd)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        hapd->dpp_resp_wait_time = 5000;
        ret = hostapd_dpp_pkex_add(hapd, cmd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */

        wpa_s->dpp_resp_wait_time = 5000;
        wpa_s->conf->dpp_config_processing = 2;
        ret = wpas_dpp_pkex_add(wpa_s, cmd);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_dpp_chirp(const struct netif *dev, int is_ap, const char *cmd)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        hapd->dpp_resp_wait_time = 5000;
#ifdef CONFIG_DPP2
        ret = hostapd_dpp_chirp(hapd, cmd);
#endif
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */

        wpa_s->dpp_resp_wait_time = 5000;
        wpa_s->conf->dpp_config_processing = 2;
#ifdef CONFIG_DPP2
        ret = wpas_dpp_chirp(wpa_s, cmd);
#endif

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_dpp_reconfig(const struct netif *dev, const char *cmd)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        ret = -1;
        goto out;
    }
#ifdef CONFIG_DPP2
    ret = wpas_dpp_reconfig(wpa_s, cmd);
#endif

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}

int wpa_supp_dpp_configurator_sign(const struct netif *dev, int is_ap, const char *cmd)
{
    struct wpa_supplicant *wpa_s;
    int ret = 0;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (is_ap)
    {
        struct hostapd_iface *hapd_s;
        struct hostapd_data *hapd;

        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
        hapd = hapd_s->bss[0];
        ret = hostapd_dpp_configurator_sign(hapd, cmd);
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            ret = -1;
            goto out;
        }

#if CONFIG_AP
        if (wpa_s->ap_iface)
        {
            ret = -1;
            goto out;
        }
#endif /* CONFIG_AP */

        ret = wpas_dpp_configurator_sign(wpa_s, cmd);

#if CONFIG_HOSTAPD
    }
#endif
out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);

    return ret;
}
#endif /* CONFIG_DPP */

static inline enum wlan_security_type wpas_key_mgmt_to_wpa(int key_mgmt)
{
    switch (key_mgmt)
    {
        case WPA_KEY_MGMT_NONE:
            return WLAN_SECURITY_NONE;
        case WPA_KEY_MGMT_PSK:
        case WPA_KEY_MGMT_PSK_SHA256:
            return WLAN_SECURITY_WPA2;
#ifdef CONFIG_OWE
        case WPA_KEY_MGMT_OWE:
            return WLAN_SECURITY_OWE_ONLY;
#endif
        case WPA_KEY_MGMT_SAE:
            return WLAN_SECURITY_WPA3_SAE;
        case WPA_KEY_MGMT_SAE_EXT_KEY:
            return WLAN_SECURITY_WPA3_SAE_EXT_KEY;
        default:
            return WLAN_SECURITY_WILDCARD;
    }
}

int wpa_supp_status(const struct netif *dev)
{
    int status = 0;
    struct wpa_supplicant *wpa_s;
    struct wpa_supp_api_ctrl *ctrl = &wpa_supp_api_ctrl;

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);
    rt_kprintf("%s %d\n", __func__, __LINE__);
    wpa_s = get_wpa_s_handle(dev);
    if (!wpa_s)
    {
        status = -1;
        goto out;
    }
    rt_kprintf("%s %d wpa_state=%d\n", __func__, __LINE__,wpa_s->wpa_state);
    ctrl->supp_thread_state = SUPP_THREAD_RUNNING;
    status = wpa_s->wpa_state;
#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;

    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        status = -1;
        goto out;
    }
#endif

out:
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
    return status;
}

int wpa_supp_network_status(const struct netif *dev, struct wlan_network *network)
{
    struct wpa_supplicant *wpa_s;
    int ret                    = -1;
    struct wpa_signal_info *si = NULL;
#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;
#endif

    OSA_MutexLock((osa_mutex_handle_t)wpa_supplicant_mutex, osaWaitForever_c);

#if CONFIG_HOSTAPD
    if (network->role == WLAN_BSS_ROLE_UAP)
    {
        hapd_s = get_hostapd_handle(dev);
        if (!hapd_s)
        {
            ret = -1;
            goto out;
        }
    }
    else
    {
#endif
        wpa_s = get_wpa_s_handle(dev);
        if (!wpa_s)
        {
            goto out;
        }

        si = os_zalloc(sizeof(struct wpa_signal_info));
        if (!si)
        {
            goto out;
        }

        if (wpa_s->wpa_state >= WPA_ASSOCIATED)
        {
            struct wpa_ssid *ssid = wpa_s->current_ssid;
            u8 channel;

            os_memcpy(network->bssid, wpa_s->bssid, ETH_ALEN);

            if (ssid->ieee80211w == MGMT_FRAME_PROTECTION_REQUIRED)
            {
                network->security.mfpc = 1;
                network->security.mfpr = 1;
            }
            else if (ssid->ieee80211w == MGMT_FRAME_PROTECTION_OPTIONAL)
            {
                network->security.mfpc = 1;
            }

            ieee80211_freq_to_chan(wpa_s->assoc_freq, &channel);
            network->channel = channel;

            if (ssid)
            {
                u8 *_ssid                 = ssid->ssid;
                size_t ssid_len           = ssid->ssid_len;
                u8 ssid_buf[SSID_MAX_LEN] = {0};

                if (ssid_len == 0)
                {
                    int _res = wpa_drv_get_ssid(wpa_s, ssid_buf);

                    if (_res < 0)
                        ssid_len = 0;
                    else
                        ssid_len = _res;
                    _ssid = ssid_buf;
                }
                os_memcpy(network->ssid, _ssid, ssid_len);

                if (os_strstr(network->name, "wps_network"))
                {
                    os_memset(network->name, 0x00, sizeof(network->name));
                    os_memcpy(network->name, _ssid, ssid_len);
                    network->id            = ssid->id;
                    network->wps_network   = true;
                    network->security.type = wpas_key_mgmt_to_wpa(ssid->key_mgmt);

                    if (ssid->ieee80211w == MGMT_FRAME_PROTECTION_DEFAULT)
                    {
                        network->security.mfpc = 0;
                        network->security.mfpr = 0;
                    }

                    if (ssid->export_keys)
                    {
                        if (ssid->psk_set)
                        {
                            network->security.pmk_valid = true;
                            os_memcpy(network->security.pmk, ssid->psk, PMK_LEN);
                        }
                        else if (network->security.type == WLAN_SECURITY_WPA3_SAE ||
                                 network->security.type == WLAN_SECURITY_WPA3_SAE_EXT_KEY)
                        {
                            network->security.password_len = os_strlen(ssid->passphrase);
                            os_memcpy(network->security.password, ssid->passphrase, network->security.password_len);
                        }
                        else
                        {
                            network->security.psk_len = os_strlen(ssid->passphrase);
                            os_memcpy(network->security.psk, ssid->passphrase, network->security.psk_len);
                        }
                    }
                }
            }

            network->beacon_period = wpa_s->current_bss->beacon_int;

            ret = wpa_drv_signal_poll(wpa_s, si);
            if (!ret)
            {
                network->rssi = si->current_signal;
            }
            else
            {
                network->rssi = -WPA_INVALID_NOISE;
            }

            network->dot11n = wpa_s->connection_ht;
#if CONFIG_11AC
            network->dot11ac = wpa_s->connection_vht;
#endif
#if CONFIG_11AX
            network->dot11ax = wpa_s->connection_he;
#endif

#if CONFIG_11K
            network->neighbor_report_supported = wpa_s->rrm.rrm_used == 1 ? true : false;
#endif
#if CONFIG_11V
            if (wpa_bss_ext_capab(wpa_s->current_bss, WLAN_EXT_CAPAB_BSS_TRANSITION))
            {
                network->bss_transition_supported = true;
            }
            else
            {
                network->bss_transition_supported = false;
            }
#endif
        }
        else
        {
            ret = 0;
        }
#if CONFIG_HOSTAPD
    }
#endif
out:
    os_free(si);
    OSA_MutexUnlock((osa_mutex_handle_t)wpa_supplicant_mutex);
    return ret;
}

static int wpa_supp_add_acl_maclist(struct mac_acl_entry **acl, int *num, int vlan_id, const u8 *addr)
{
    struct mac_acl_entry *newacl;

    newacl = os_realloc_array(*acl, *num + 1, sizeof(**acl));
    if (!newacl)
    {
        wpa_printf(MSG_ERROR, "MAC list reallocation failed");
        return -1;
    }

    *acl = newacl;
    os_memcpy((*acl)[*num].addr, addr, ETH_ALEN);
    os_memset(&(*acl)[*num].vlan_id, 0, sizeof((*acl)[*num].vlan_id));
    (*acl)[*num].vlan_id.untagged = vlan_id;
    (*acl)[*num].vlan_id.notempty = !!vlan_id;
    (*num)++;
    return 0;
}

int wpa_supp_acl_comp(const void *a, const void *b)
{
	const struct mac_acl_entry *aa = a;
	const struct mac_acl_entry *bb = b;
	return os_memcmp(aa->addr, bb->addr, sizeof(macaddr));
}

int wpa_supp_set_mac_acl(const struct netif *dev, int filter_mode, char mac_count, unsigned char *mac_addr)
{
    int num = 0;
    struct hostapd_iface *hapd_s;
    struct hostapd_data *hapd;
    struct hostapd_bss_config *bss;
#if CONFIG_HOSTAPD
    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        return -1;
    }
#endif
    hapd = hapd_s->bss[0];
    bss = hapd->conf;

    switch(filter_mode)
    {
        case 0:
            bss->macaddr_acl = 0;
            if (bss->accept_mac)
                os_free(bss->accept_mac);
            bss->accept_mac	 = NULL;
            bss->num_accept_mac = 0;

            if (bss->deny_mac)
                os_free(bss->deny_mac);
            bss->deny_mac	   = NULL;
            bss->num_deny_mac = 0;
            break;
        case 1:
            for (num = 0; num < mac_count;)
            {
                if (wpa_supp_add_acl_maclist(&bss->accept_mac, &num, 0, &mac_addr[num*ETH_ALEN]) < 0)
                {
                    wpa_printf(MSG_ERROR, "Line %d: add accept_mac fail", __LINE__);
                    return -1;
                }
            }
            bss->num_accept_mac = mac_count;
            bss->macaddr_acl = 1;
            qsort(bss->accept_mac, bss->num_accept_mac, sizeof(struct mac_acl_entry), wpa_supp_acl_comp);
            break;
        case 2:
            for (num = 0; num < mac_count;)
            {
                if (wpa_supp_add_acl_maclist(&bss->deny_mac, &num, 0, &mac_addr[num*ETH_ALEN]) < 0)
                {
                    wpa_printf(MSG_ERROR, "Line %d: add deny_mac fail", __LINE__);
                    return -1;
                }
            }
            bss->num_deny_mac = mac_count;
            bss->macaddr_acl = 0;
            qsort(bss->deny_mac, bss->num_deny_mac, sizeof(struct mac_acl_entry), wpa_supp_acl_comp);
            break;
        default:
          return -1;
    }
    return 0;
}

static void (*msg_cb_ptr)(const char *txt, size_t len);

static void wpa_supplicant_msg_cb(void *ctx, int level, enum wpa_msg_type type, const char *txt, size_t len)
{
    msg_cb_ptr(txt, len);
}

int wpa_supp_init(void (*msg_cb)(const char *txt, size_t len))
{
    osa_status_t status;

    status = OSA_MutexCreate((osa_mutex_handle_t)wpa_supplicant_mutex);
    if (status != KOSA_StatusSuccess)
    {
        wpa_printf(MSG_DEBUG, "Failed to initialize wpa supp mutex");
        return -1;
    }

    status = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)wpaSuppReadySemaphoreHandle);
    if (status != KOSA_StatusSuccess)
    {
        wpa_printf(MSG_DEBUG, "Failed to initialize wpa supp ready semaphore");
        return -1;
    }

    msg_cb_ptr = msg_cb;

    wpa_msg_register_cb(wpa_supplicant_msg_cb);

#if CONFIG_HOSTAPD
    status = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)hostapdReadySemaphoreHandle);
    if (status != KOSA_StatusSuccess)
    {
        wpa_printf(MSG_DEBUG, "Failed to initialize wpa hostapd ready semaphore");
        return -1;
    }
#endif

    bandwidth = 2;
    h_hidden_ssid = 0;
    h_beacon_int = 100;
    h_max_num_sta = 8;

    return 0;
}

int wpa_supp_deinit(void)
{
    osa_status_t status;

#ifndef __ZEPHYR__
    status = OSA_EventDestroy((osa_event_handle_t)supplicant_event_Handle);
    if (status != KOSA_StatusSuccess)
    {
        wpa_printf(MSG_DEBUG, "Failed to destroy supp event handle");
        return -1;
    }
#endif
    status = OSA_MutexDestroy((osa_mutex_handle_t)wpa_supplicant_mutex);
    if (status != KOSA_StatusSuccess)
    {
        wpa_printf(MSG_DEBUG, "Failed to destroy wpa supp mutex");
        return -1;
    }

    status = OSA_SemaphoreDestroy((osa_semaphore_handle_t)wpaSuppReadySemaphoreHandle);
    if (status != KOSA_StatusSuccess)
    {
        wpa_printf(MSG_DEBUG, "Failed to destroy wpa supp ready semaphore");
        return -1;
    }

#if CONFIG_HOSTAPD
    status = OSA_SemaphoreDestroy((osa_semaphore_handle_t)hostapdReadySemaphoreHandle);
    if (status != KOSA_StatusSuccess)
    {
        wpa_printf(MSG_DEBUG, "Failed to destroy wpa hostapd ready semaphore");
        return -1;
    }
#endif

    return 0;
}
#if 0
void hostapd_connected_sta_list(wifi_sta_info_t *si, wifi_sta_list_t *sl)
{
    struct hostapd_iface *hapd_s;
    struct netif *netif       = net_get_uap_interface();
#if CONFIG_HOSTAPD
    hapd_s                    = get_hostapd_handle(netif);
#endif
    struct hostapd_data *hapd = hapd_s->bss[0];
    struct sta_info *sta;
    int count = 0, i = 0;

    for (sta = hapd->sta_list; sta; sta = sta->next)
        count++;

    PRINTF("\r\nNumber of STA = %d\r\n", count);

    for (sta = hapd->sta_list; sta; sta = sta->next)
    {
        PRINTF("\r\nSTA %d information:\n\r", sta->aid);
        PRINTF("=====================\r\n");
        PRINTF("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", sta->addr[0], sta->addr[1], sta->addr[2], sta->addr[3],
               sta->addr[4], sta->addr[5]);
        for (i = 0; i < sl->count; i++)
        {
          if (memcmp(si[i].mac, sta->addr, MLAN_MAC_ADDR_LENGTH) == 0)
          {
                PRINTF("Power mfg status: %s\r\n", (si[i].power_mgmt_status == 0U) ? "active" : "power save");
                PRINTF("Rssi : %d dBm\r\n\r\n", (signed char)si[i].rssi);
                break;
          }
        }
    }
}
#endif
int wpa_supp_deauth_sta(struct netif *dev, const u8* sta_addr)
{
    int ret = 0;
    int reason = WLAN_REASON_PREV_AUTH_NOT_VALID;
    struct sta_info *sta;
    struct hostapd_data *hapd;

#if CONFIG_HOSTAPD
    struct hostapd_iface *hapd_s;
    hapd_s = get_hostapd_handle(dev);
    if (!hapd_s)
    {
        ret = -1;
        goto out;
    }

    hapd = hapd_s->bss[0];
#elif CONFIG_AP
    struct wpa_supplicant *wpa_s;
    wpa_s = get_wpa_s_handle(dev);
    if (wpa_s->ap_iface == NULL)
    {
        ret = -1;
        goto out;
    }

    hapd = wpa_s->ap_iface->bss[0];
#else
    return -1;
#endif

    //hostapd_drv_sta_deauth(hapd, sta_addr, reason);

    // sta = ap_get_sta(hapd, sta_addr);
    // if (sta)
    //     ap_sta_deauthenticate(hapd, sta, reason);
    // else if (sta_addr[0] == 0xff)
    //     hostapd_free_stas(hapd);

out:
    return ret;

}

