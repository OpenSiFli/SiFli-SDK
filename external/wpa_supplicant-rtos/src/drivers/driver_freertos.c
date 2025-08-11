/*
 * Driver interaction with FreeRTOS WLAN device drivers.
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "includes.h"
#include "utils/common.h"
#include "eloop.h"
#include "driver_freertos.h"
#include "supp_main.h"
#include "rtthread.h"
//#include "wm_net.h"
#include "fsl_os_abstraction.h"
#include "wm_net.h"
#include "ap/hostapd.h"

#define SCAN_TIMEOUT   30
#define SURVEY_TIMEOUT 30

static void wpa_drv_freertos_event_proc_mgmt_rx(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event);

static void wpa_drv_freertos_event_proc_eapol_rx(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event);

static int wpa_drv_freertos_set_key(void *priv, struct wpa_driver_set_key_params *params);

static void wpa_drv_freertos_event_ecsa_complete(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event);

static int wpa_drv_freertos_cancel_remain_on_channel(void *priv);

static void wpa_drv_freertos_event_dfs_cac_started(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event);

static void wpa_drv_freertos_event_dfs_cac_finished(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event);

void wpa_supplicant_event_wrapper(void *ctx, enum wpa_event_type event, union wpa_event_data *data)
{
    struct wpa_supplicant_event_msg *msg = NULL;

    msg = os_zalloc(sizeof(*msg));
    if (!msg)
    {
        wpa_printf(MSG_ERROR, "Failed to allocate msg for event: %d", event);
        return;
    }

    msg->ctx   = ctx;
    msg->event = event;
    if (data)
    {
        msg->data = os_zalloc(sizeof(*data));
        if (!msg->data)
        {
            wpa_printf(MSG_ERROR, "Failed to allocate data for event: %d", event);
            os_free(msg);
            return;
        }
        os_memcpy(msg->data, data, sizeof(*data));

        if (wpa_supplicant_event_wrapper_deep_copy(msg, event, msg->data) != 0)
        {
            wpa_printf(MSG_ERROR, "Deep copy failed for event: %d", event);
            os_free(msg->data);
            os_free(msg);
            return;
        }
    }
    send_wpa_supplicant_event(msg);
}
void hostapd_event_wrapper(void *ctx, enum wpa_event_type event, union wpa_event_data *data)
{
    struct wpa_supplicant_event_msg *msg = NULL;

    msg = os_zalloc(sizeof(*msg));
    if (!msg)
    {
        wpa_printf(MSG_ERROR, "Failed to allocate msg for event: %d", event);
        return;
    }

    msg->hostapd = 1;
    msg->ctx   = ctx;
    msg->event = event;
    if (data)
    {
        msg->data = os_zalloc(sizeof(*data));
        if (!msg->data)
        {
            wpa_printf(MSG_ERROR, "Failed to allocate data for event: %d", event);
            os_free(msg);
            return;
        }
        os_memcpy(msg->data, data, sizeof(*data));

        if (wpa_supplicant_event_wrapper_deep_copy(msg, event, msg->data) != 0)
        {
            wpa_printf(MSG_ERROR, "Deep copy failed for event: %d", event);
            os_free(msg->data);
            os_free(msg);
            return;
        }
    }
    send_wpa_supplicant_event(msg);
}

void wpa_drv_freertos_event_mac_changed(struct freertos_drv_if_ctx *if_ctx)
{
#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_INTERFACE_MAC_CHANGED, NULL);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx,
                EVENT_INTERFACE_MAC_CHANGED,
                NULL);
}

void wpa_drv_freertos_event_chan_list_changed(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_CHANNEL_LIST_CHANGED, event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx,
                EVENT_CHANNEL_LIST_CHANGED,
                event);
}

static int wpa_drv_freertos_abort_scan(void *priv, u64 scan_cookie)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->scan_abort)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for scan_abort", __func__);
        goto out;
    }

    ret = dev_ops->scan_abort(if_ctx->dev_priv);
out:
    return ret;
}

/**
 * wpa_drv_freertos_scan_timeout - Scan timeout to report scan completion
 * @eloop_ctx: Driver private data
 * @timeout_ctx: ctx argument given to wpa_drv_freertos_init()
 *
 * This function can be used as registered timeout when starting a scan to
 * generate a scan completed event if the driver does not report this.
 */
void wpa_drv_freertos_scan_timeout(void *eloop_ctx, void *timeout_ctx)
{
    struct freertos_drv_if_ctx *if_ctx = NULL;

    if_ctx = eloop_ctx;

    wpa_printf(MSG_ERROR, "%s: Scan timeout - try to abort it", __func__);

    if (wpa_drv_freertos_abort_scan(if_ctx, 0) == 0)
    {
        return;
    }
}

void wpa_drv_freertos_event_proc_scan_start(struct freertos_drv_if_ctx *if_ctx)
{
    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_SCAN_STARTED, NULL);
}
//用于处理扫描被中止/取消的情况（如用户主动取消、驱动异常等）
void wpa_drv_freertos_event_proc_scan_abort(struct freertos_drv_if_ctx *if_ctx)
{
    union wpa_event_data event;
    struct scan_info *info = NULL;

    wpa_printf(MSG_DEBUG, "%s: Scan aborted by driver", __func__);

    if_ctx->scan_res2_get_in_prog = false;

    memset(&event, 0, sizeof(event));

    info                = &event.scan_info;
    info->aborted       = 1;
    info->external_scan = 0;
    info->nl_scan_event = 1;

    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_SCAN_RESULTS, &event);
}
//用于处理扫描正常完成的情况（即扫描流程顺利结束，获得了扫描结果）。
void wpa_drv_freertos_event_proc_scan_done(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
    eloop_cancel_timeout(wpa_drv_freertos_scan_timeout, if_ctx, if_ctx->supp_if_ctx);

    if_ctx->scan_res2_get_in_prog = false;

#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_SCAN_RESULTS, event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_SCAN_RESULTS, event);
}

void wpa_drv_freertos_event_proc_auth_resp(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_AUTH, event);
}

static void wpa_drv_freertos_free_pairwise_key_params(struct wpa_driver_set_key_params *params)
{
    if (params->ifname)
    {
        os_free((void *)params->ifname);
    }

    if (params->addr)
    {
        os_free((void *)params->addr);
    }

    if (params->seq)
    {
        os_free((void *)params->seq);
    }

    if (params->key)
    {
        os_free((void *)params->key);
    }

    os_free(params);
}

void wpa_drv_freertos_event_proc_assoc_resp(struct freertos_drv_if_ctx *if_ctx,
                                            union wpa_event_data *event,
                                            unsigned int status)
{
    if (status != WLAN_STATUS_SUCCESS)
    {
        if (if_ctx->key_params)
        {
            if_ctx->ft_roaming = false;

            wpa_drv_freertos_free_pairwise_key_params(if_ctx->key_params);
            if_ctx->key_params = NULL;
        }

        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_ASSOC_REJECT, event);
    }
    else
    {
        if_ctx->associated = true;

        os_memcpy(if_ctx->bssid, event->assoc_info.addr, ETH_ALEN); 
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_ASSOC, event);
    }
}

void wpa_drv_freertos_event_proc_deauth(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event, const struct ieee80211_mgmt *mgmt)
{
    const u8 *bssid = NULL;

    bssid = mgmt->bssid;

    if ((if_ctx->capa.flags & WPA_DRIVER_FLAGS_SME) &&
            !if_ctx->associated &&
            os_memcmp(bssid, if_ctx->auth_bssid, ETH_ALEN) != 0 &&
            os_memcmp(bssid, if_ctx->auth_attempt_bssid, ETH_ALEN) != 0 &&
            os_memcmp(bssid, if_ctx->prev_bssid, ETH_ALEN) == 0)
    {
        /*
         * Avoid issues with some roaming cases where
         * disconnection event for the old AP may show up after
         * we have started connection with the new AP.
         * In case of locally generated event clear
         * ignore_next_local_deauth as well, to avoid next local
         * deauth event be wrongly ignored.
         */
        wpa_printf(MSG_DEBUG,
                "nl80211: Ignore deauth/disassoc event from old AP " MACSTR " when already authenticating with " MACSTR,
                MAC2STR(bssid),
                MAC2STR(if_ctx->auth_attempt_bssid));
        return;
    }

    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_DEAUTH, event);
}

void wpa_drv_freertos_event_proc_disassoc(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_DISASSOC, event);
}

static void wpa_drv_freertos_event_acs_channel_selected(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_ACS_CHANNEL_SELECTED, event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_ACS_CHANNEL_SELECTED, event);
}

static void wpa_drv_freertos_event_mgmt_tx_status(struct freertos_drv_if_ctx *if_ctx,
                                                  const u8 *frame,
                                                  size_t len,
                                                  bool ack)
{
    union wpa_event_data event;
    const struct ieee80211_hdr *hdr;
    u16 fc;

    wpa_printf(MSG_DEBUG, "Frame TX status event");

    hdr = (const struct ieee80211_hdr *)frame;
    fc  = le_to_host16(hdr->frame_control);

    os_memset(&event, 0, sizeof(event));
    event.tx_status.type     = WLAN_FC_GET_TYPE(fc);
    event.tx_status.stype    = WLAN_FC_GET_STYPE(fc);
    event.tx_status.dst      = hdr->addr1;
    event.tx_status.data     = frame;
    event.tx_status.data_len = len;
    event.tx_status.ack      = ack;

#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_TX_STATUS, &event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_TX_STATUS, &event);
}

static void wpa_drv_freertos_event_proc_unprot_deauth(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_UNPROT_DEAUTH, event);
}

static void wpa_drv_freertos_event_proc_unprot_disassoc(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_UNPROT_DISASSOC, event);
}

void wpa_drv_freertos_event_proc_remain_on_channel(struct freertos_drv_if_ctx *if_ctx,
                                                   int cancel_event,
                                                   union wpa_event_data *event)
{
    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx,
                                 cancel_event ? EVENT_CANCEL_REMAIN_ON_CHANNEL : EVENT_REMAIN_ON_CHANNEL, event);
}

void wpa_drv_freertos_event_proc_survey_res(struct freertos_drv_if_ctx *if_ctx,
                                            struct freq_survey *survey,
                                            bool more_res)
{
    struct survey_results *survey_results;

    if (survey == NULL)
    {
        if (!more_res)
        {
            if_ctx->survey_res_get_in_prog = false;
        }
        return;
    }

    survey_results = &if_ctx->data->survey_results;

    if (survey_results == NULL)
    {
        if_ctx->survey_res_get_in_prog = false;
        os_free(survey);
        return;
    }

    if (survey_results->freq_filter && survey_results->freq_filter != survey->freq)
    {
        wpa_printf(MSG_EXCESSIVE, "Ignoring survey data for freq %d MHz", survey->freq);
        os_free(survey);
        return;
    }

    wpa_printf(MSG_EXCESSIVE,
               "Freq survey dump event (freq=%d MHz noise=%d channel_time=%ld busy_time=%ld tx_time=%ld "
               "rx_time=%ld filled=%04x)",
               survey->freq, survey->nf, (unsigned long int)survey->channel_time,
               (unsigned long int)survey->channel_time_busy, (unsigned long int)survey->channel_time_tx,
               (unsigned long int)survey->channel_time_rx, survey->filled);

    dl_list_add_tail(&survey_results->survey_list, &survey->list);

    if_ctx->survey_res_get_in_prog = more_res;
}

static void wpa_drv_freertos_event_proc_signal_change(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
    wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_SIGNAL_CHANGE, event);
}

static void *wpa_drv_freertos_global_init(void *ctx)
{
    struct freertos_drv_ctx *drv_ctx = NULL;

    drv_ctx = os_zalloc(sizeof(*drv_ctx));

    if (!drv_ctx)
    {
        return NULL;
    }

    drv_ctx->supp_ctx = ctx;

    return drv_ctx;
}

static void wpa_drv_freertos_global_deinit(void *priv)
{
    struct freertos_drv_ctx *drv_ctx = priv;

    if (!drv_ctx)
    {
        return;
    }

    os_free(drv_ctx);
}

/**
 * wpa_driver_freertos_init - Initialize freertos driver interface
 * @ctx: Context to be used when calling wpa_supplicant functions,
 *       e.g., wpa_supplicant_event_wrapper()
 * @ifname: Interface name, e.g., wlan0
 * @global_priv: private driver global data from global_init()
 *
 * Returns: Pointer to private data, %NULL on failure
 */
static void *wpa_drv_freertos_init(void *ctx, const char *ifname, void *global_priv)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    const struct netif *device                      = NULL;
    struct freertos_wpa_supp_dev_callbk_fns callbk_fns;
    u8 ext_capab[10]           = {0x00, 0x00, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    u8 ext_capab_mask[10]      = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned int ext_capab_len = 10;

#ifdef __ZEPHYR__
    device = net_if_get_binding(ifname);
#else
    LOCK_TCPIP_CORE();
    device = netif_find(ifname);
    UNLOCK_TCPIP_CORE();
#endif

    if (!device)
    {
        wpa_printf(MSG_ERROR, "%s: Interface %s not found", __func__, ifname);
        goto out;
    }

    if_ctx = os_zalloc(sizeof(*if_ctx));

    if (if_ctx == NULL)
    {
        goto out;
    }

    if_ctx->supp_if_ctx = ctx;

    if_ctx->dev_ctx = device;
    if_ctx->drv_ctx = global_priv;

#ifdef __ZEPHYR__
    if_ctx->dev_ops = (struct freertos_wpa_supp_dev_ops *)net_if_get_dev_config((struct netif *)device);
#else
#if LWIP_NUM_NETIF_CLIENT_DATA > 0
    if_ctx->dev_ops = netif_get_client_data(device, LWIP_NETIF_CLIENT_DATA_INDEX_MAX);
#endif
#endif

    if_ctx->extended_capa = os_malloc(ext_capab_len);
    if (if_ctx->extended_capa)
    {
        os_memcpy(if_ctx->extended_capa, ext_capab, ext_capab_len);
    }

    if_ctx->extended_capa_mask = os_malloc(ext_capab_len);
    if (if_ctx->extended_capa_mask)
    {
        os_memcpy(if_ctx->extended_capa_mask, ext_capab_mask, ext_capab_len);
    }

    if_ctx->extended_capa_len = ext_capab_len;

	/*
     * There is no driver capability flag for this, so assume it is
     * supported and disable this on first attempt to use if the driver
     * rejects the command due to missing support.
     */
    if_ctx->set_rekey_offload = 1;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->init)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for init\n", __func__);

        os_free(if_ctx);
        if_ctx = NULL;
        goto out;
    }

    os_memset(&callbk_fns, 0, sizeof(callbk_fns));

    callbk_fns.scan_start        = wpa_drv_freertos_event_proc_scan_start;
    callbk_fns.scan_abort        = wpa_drv_freertos_event_proc_scan_abort;
    callbk_fns.scan_done         = wpa_drv_freertos_event_proc_scan_done;
    callbk_fns.survey_res        = wpa_drv_freertos_event_proc_survey_res;
    callbk_fns.auth_resp         = wpa_drv_freertos_event_proc_auth_resp;
    callbk_fns.assoc_resp        = wpa_drv_freertos_event_proc_assoc_resp;
    callbk_fns.deauth            = wpa_drv_freertos_event_proc_deauth;
    callbk_fns.disassoc          = wpa_drv_freertos_event_proc_disassoc;
    callbk_fns.acs_channel_sel   = wpa_drv_freertos_event_acs_channel_selected;
    callbk_fns.mgmt_tx_status    = wpa_drv_freertos_event_mgmt_tx_status;
    callbk_fns.unprot_deauth     = wpa_drv_freertos_event_proc_unprot_deauth;
    callbk_fns.unprot_disassoc   = wpa_drv_freertos_event_proc_unprot_disassoc;
    callbk_fns.remain_on_channel = wpa_drv_freertos_event_proc_remain_on_channel;
    callbk_fns.mgmt_rx           = wpa_drv_freertos_event_proc_mgmt_rx;
    callbk_fns.eapol_rx          = wpa_drv_freertos_event_proc_eapol_rx;
    callbk_fns.signal_change     = wpa_drv_freertos_event_proc_signal_change;
    callbk_fns.mac_changed = wpa_drv_freertos_event_mac_changed;
    callbk_fns.chan_list_changed = wpa_drv_freertos_event_chan_list_changed;
    callbk_fns.ecsa_complete     = wpa_drv_freertos_event_ecsa_complete;
    callbk_fns.dfs_cac_started   = wpa_drv_freertos_event_dfs_cac_started;
    callbk_fns.dfs_cac_finished  = wpa_drv_freertos_event_dfs_cac_finished;

    if_ctx->dev_priv = dev_ops->init(if_ctx, ifname, &callbk_fns);

    if (!if_ctx->dev_priv)
    {
        wpa_printf(MSG_ERROR, "%s: Failed to initialize the interface\n", __func__);
        os_free(if_ctx);
        if_ctx = NULL;

        goto out;
    }

out:
    return if_ctx;
}

static void wpa_drv_freertos_deinit(void *priv)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->deinit)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for deinit", __func__);
        return;
    }

    dev_ops->deinit(if_ctx->dev_priv);

    os_free(if_ctx->extended_capa_mask);
    os_free(if_ctx->extended_capa);
    os_free(if_ctx);
}

static int wpa_drv_freertos_scan2(void *priv, struct wpa_driver_scan_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int timeout                                     = 0;
    int ret                                         = -1;

    if (!priv || !params)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    if (if_ctx->scan_res2_get_in_prog)
    {
        wpa_printf(MSG_ERROR, "%s: Scan is already in progress", __func__);
        goto out;
    }

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->scan2)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for scan2", __func__);
        goto out;
    }

    ret = dev_ops->scan2(if_ctx->dev_priv, params);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: scan2 op failed", __func__);
        goto out;
    }

    /* The driver delivers events to notify when scan is
     * complete, so use longer timeout to avoid race conditions
     * with scanning and following association request.
     */
    timeout = SCAN_TIMEOUT;

    wpa_printf(MSG_DEBUG, "%s: Scan requested - scan timeout %d seconds\n", __func__, timeout);

    eloop_cancel_timeout(wpa_drv_freertos_scan_timeout, if_ctx, if_ctx->supp_if_ctx);

    eloop_register_timeout(timeout, 0, wpa_drv_freertos_scan_timeout, if_ctx, if_ctx->supp_if_ctx);

    ret = 0;

out:

    return ret;
}

static int wpa_drv_freertos_set_default_scan_ies(void *priv, const u8 *ies, size_t ies_len)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv || !ies)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->set_default_scan_ies)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for set_default_scan_ies", __func__);
        goto out;
    }

    ret = dev_ops->set_default_scan_ies(if_ctx->dev_priv, ies, ies_len);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: set_default_scan_ies op failed", __func__);
        goto out;
    }

    ret = 0;

out:

    return ret;
}

int wpa_drv_freertos_sched_scan(void *priv, struct wpa_driver_scan_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv || !params)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->sched_scan)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for sched scan", __func__);
        goto out;
    }

    ret = dev_ops->sched_scan(if_ctx->dev_priv, params);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: sched scan op failed", __func__);
        goto out;
    }

    ret = 0;

out:

    return ret;
}

int wpa_drv_freertos_stop_sched_scan(void *priv)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->stop_sched_scan)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for stop sched scan", __func__);
        goto out;
    }

    ret = dev_ops->stop_sched_scan(if_ctx->dev_priv);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: stop sched scan op failed", __func__);
        goto out;
    }

    ret = 0;

out:

    return ret;
}

/**
 * wpa_drv_freertos_get_scan_results2 - Fetch the latest scan results
 * @priv: Pointer to private data from wpa_drv_freertos_init()
 * Returns: Scan results on success, -1 on failure
 */
struct wpa_scan_results *wpa_drv_freertos_get_scan_results2(void *priv)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    // unsigned int i                                  = 0;
    int ret = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->get_scan_results2)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for get2_scan_results2", __func__);
        goto out;
    }

    if_ctx->scan_res2 = os_zalloc(sizeof(*if_ctx->scan_res2));

    if (!if_ctx->scan_res2)
    {
        wpa_printf(MSG_ERROR, "%s: Failed to alloc memory for scan results", __func__);
        goto out;
    }

    if_ctx->scan_res2_get_in_prog = true;

    ret = dev_ops->get_scan_results2(if_ctx->dev_priv, if_ctx->scan_res2);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: get2_scan_results2 op failed", __func__);
        goto out;
    }

    ret = 0;

out:
    if (ret == -1)
    {
        if (if_ctx->scan_res2)
        {
            wpa_scan_results_free(if_ctx->scan_res2);
            if_ctx->scan_res2 = NULL;
        }
    }

    if_ctx->scan_res2_get_in_prog = false;

    return if_ctx->scan_res2;
}

static int wpa_drv_freertos_deauthenticate(void *priv, const unsigned char *addr, u16 reason_code)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if ((!priv) || (!addr))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params\n", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->deauthenticate(if_ctx->dev_priv, (const char *)addr, reason_code);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: deauthenticate op failed\n", __func__);
        goto out;
    }

    ret = 0;

out:
    return ret;
}
/*
wpa_drv_freertos_authenticate 是 FreeRTOS 平台下 wpa_supplicant 驱动适配层的认证接口实现。
它的作用是：根据上层传入的认证参数，调用底层驱动的认证流程，完成与目标 AP 的 802.11 认证过程
*/
static int wpa_drv_freertos_authenticate(void *priv, struct wpa_driver_auth_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    struct wpa_bss *curr_bss;
    int ret = -1;
    //RT_ASSERT(0);
    if ((!priv) || (!params))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params\n", __func__);
        goto out;
    }

    if_ctx = priv;

    if (params->local_state_change)
    {
        wpa_printf(MSG_DEBUG, "Local state change only");
        ret = 0;
        goto out;
    }

    if_ctx->ft_roaming = false;

    if (params->auth_alg == WPA_AUTH_ALG_FT)
    {
        if_ctx->ft_roaming = true;
    }

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    os_memcpy(if_ctx->ssid, params->ssid, params->ssid_len);

    if_ctx->ssid_len = params->ssid_len;

    curr_bss = wpa_bss_get(if_ctx->supp_if_ctx, params->bssid, params->ssid, params->ssid_len);

    if (!curr_bss)
    {
        wpa_printf(MSG_ERROR, "%s: Failed to get BSS", __func__);
        ret = -1;
        goto out;
    }
    if (params->bssid)
        os_memcpy(if_ctx->auth_attempt_bssid, params->bssid, ETH_ALEN);

    if (if_ctx->associated)
        os_memcpy(if_ctx->prev_bssid, if_ctx->bssid, ETH_ALEN);

    os_memset(if_ctx->auth_bssid, 0, ETH_ALEN);

    if_ctx->associated = false;

    ret = dev_ops->authenticate(if_ctx->dev_priv, params, curr_bss);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: authenticate op failed\n", __func__);
        goto out;
    }

    ret = 0;

out:
    return ret;
}

static int wpa_drv_freertos_associate(void *priv, struct wpa_driver_associate_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if ((!priv) || (!params))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params\n", __func__);
        goto out;
    }

    if_ctx = priv;

    if (params->mode == IEEE80211_MODE_AP)
    {
        if_ctx->is_ap = 1;
        return 0;
    }

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->associate(if_ctx->dev_priv, params);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: associate op failed\n", __func__);
        goto out;
    }

    ret = 0;

out:
    return ret;
}

static int _wpa_drv_freertos_set_key(void *priv,
                                     const unsigned char *ifname,
                                     enum wpa_alg alg,
                                     const u8 *addr,
                                     int key_idx,
                                     int set_tx,
                                     const u8 *seq,
                                     size_t seq_len,
                                     const u8 *key,
                                     size_t key_len,
                                     enum key_flag key_flag)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }
    if ((alg != WPA_ALG_NONE) && (!key))
    {
        wpa_printf(MSG_ERROR, "%s: Missing mandatory params", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    wpa_printf(MSG_EXCESSIVE,
               "%s: priv:%p alg %d addr %p key_idx %d set_tx %d seq %p "
               "seq_len %d key %p key_len %d key_flag %d",
               __func__, if_ctx->dev_priv, alg, addr, key_idx, set_tx, seq, seq_len, key, key_len, key_flag);

    ret = dev_ops->set_key(if_ctx->dev_priv, ifname, alg, addr, key_idx, set_tx, seq, seq_len, key, key_len, key_flag);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: set_key op failed", __func__);
        goto out;
    }
    ret = 0;
out:
    return ret;
}

static int wpa_drv_freertos_save_pairwise_key_params(void *priv, struct wpa_driver_set_key_params *params)
{
    struct freertos_drv_if_ctx *if_ctx = NULL;
    int ret = -1;

    if_ctx = priv;

    if (if_ctx->key_params)
    {
        wpa_drv_freertos_free_pairwise_key_params(if_ctx->key_params);
        if_ctx->key_params = NULL;
    }

    if_ctx->key_params = (struct wpa_driver_set_key_params *)os_zalloc(sizeof(struct wpa_driver_set_key_params));

    if (!if_ctx->key_params)
    {
        wpa_printf(MSG_DEBUG, "%s: failed to alloc", __func__);
        return -1;
    }

    if (params->ifname)
    {
        if_ctx->key_params->ifname = os_strdup(params->ifname);

        if (!if_ctx->key_params->ifname)
        {
            wpa_printf(MSG_DEBUG, "%s: failed to alloc ifname", __func__);
            goto out;
        }
    }

    if (params->addr)
    {
        if_ctx->key_params->addr = os_memdup(params->addr, ETH_ALEN);

        if (!if_ctx->key_params->addr)
        {
            wpa_printf(MSG_DEBUG, "%s: failed to alloc addr", __func__);
            goto out;
        }
    }

    if (params->seq)
    {
        if_ctx->key_params->seq = os_memdup(params->seq, params->seq_len);

        if (!if_ctx->key_params->seq)
        {
            wpa_printf(MSG_DEBUG, "%s: failed to alloc seq", __func__);
            goto out;
        }
        if_ctx->key_params->seq_len = params->seq_len;
    }

    if (params->key)
    {
        if_ctx->key_params->key = os_memdup(params->key, params->key_len);

        if (!if_ctx->key_params->key)
        {
            wpa_printf(MSG_DEBUG, "%s: failed to alloc key", __func__);
            goto out;
        }
        if_ctx->key_params->key_len = params->key_len;
    }

    if_ctx->key_params->alg = params->alg;

    if_ctx->key_params->key_idx = params->key_idx;

    if_ctx->key_params->set_tx = params->set_tx;

    if_ctx->key_params->key_flag = params->key_flag;

    ret = 0;

out:
    if (ret)
    {
        wpa_drv_freertos_free_pairwise_key_params(if_ctx->key_params);
        if_ctx->key_params = NULL;
    }

    return ret;
}

static void wpa_drv_freertos_set_rekey_info(void *priv, const u8 *kek, size_t kek_len, const u8 *kck, size_t kck_len, const u8 *replay_ctr)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }

    if (!kek || !kck || !replay_ctr)
    {
        wpa_printf(MSG_ERROR, "%s: Missing mandatory params", __func__);
        goto out;
    }

    if_ctx = priv;

    if (!if_ctx->set_rekey_offload)
        return;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->set_rekey_info(if_ctx->dev_priv, kek, kek_len, kck, kck_len, replay_ctr);

    if (ret)
    {
        if_ctx->set_rekey_offload = 0;
        wpa_printf(MSG_ERROR, "%s: set_rekey_info op failed", __func__);
        goto out;
    }

out:
    return;
}

static int wpa_drv_freertos_del_key(void *priv, const u8 *addr, int key_idx)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->del_key(if_ctx->dev_priv, addr, key_idx);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: set_key op failed", __func__);
        goto out;
    }

    ret = 0;

out:
    return ret;
}

static int wpa_drv_freertos_set_key(void *priv, struct wpa_driver_set_key_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    enum key_flag key_flag = params->key_flag;
    enum wpa_alg alg = params->alg;
    const u8 *addr   = params->addr;
    int key_idx      = params->key_idx;

    if (check_key_flag(key_flag))
    {
        wpa_printf(MSG_DEBUG, "%s: invalid key_flag", __func__);
        return -1;
    }

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        return -1;
    }

    if (alg == WPA_ALG_NONE)
    {
        return wpa_drv_freertos_del_key(priv, addr, key_idx);
    }

    if_ctx = priv;

    if (if_ctx->ft_roaming && (key_flag & KEY_FLAG_PAIRWISE))
    {
        return wpa_drv_freertos_save_pairwise_key_params(priv, params);
    }

    if (if_ctx->ft_roaming && (key_flag & KEY_FLAG_GROUP))
    {
        if (if_ctx->key_params && (if_ctx->key_params->key_flag & KEY_FLAG_PAIRWISE))
        {
            if_ctx->ft_roaming = false;

            _wpa_drv_freertos_set_key(priv, (const unsigned char *)if_ctx->key_params->ifname, if_ctx->key_params->alg, if_ctx->key_params->addr,
                    if_ctx->key_params->key_idx, if_ctx->key_params->set_tx, if_ctx->key_params->seq, if_ctx->key_params->seq_len, if_ctx->key_params->key,
                    if_ctx->key_params->key_len, if_ctx->key_params->key_flag);

            wpa_drv_freertos_free_pairwise_key_params(if_ctx->key_params);

            if_ctx->key_params = NULL;
        }
    }

    return _wpa_drv_freertos_set_key(priv, (const unsigned char *)params->ifname, params->alg, params->addr,
                                     params->key_idx, params->set_tx, params->seq, params->seq_len, params->key,
                                     params->key_len, params->key_flag);
}

static int wpa_drv_freertos_get_capa(void *priv, struct wpa_driver_capa *capa)
{
    struct freertos_drv_if_ctx *if_ctx = NULL;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        return -1;
    }

    if_ctx = (struct freertos_drv_if_ctx *)priv;

    os_memset(capa, 0, sizeof(*capa));

    capa->enc |= WPA_DRIVER_CAPA_ENC_TKIP;
    capa->enc |= WPA_DRIVER_CAPA_ENC_CCMP;
    capa->enc |= WPA_DRIVER_CAPA_ENC_GCMP;
    capa->enc |= WPA_DRIVER_CAPA_ENC_GCMP_256;
    capa->enc |= WPA_DRIVER_CAPA_ENC_CCMP_256;
    capa->enc |= WPA_DRIVER_CAPA_ENC_BIP;
    capa->enc |= WPA_DRIVER_CAPA_ENC_BIP_GMAC_128;
    capa->enc |= WPA_DRIVER_CAPA_ENC_BIP_GMAC_256;
    capa->enc |= WPA_DRIVER_CAPA_ENC_BIP_CMAC_256;

    /* Use SME */
    capa->flags = 0;
    capa->flags |= WPA_DRIVER_FLAGS_SME;
    capa->flags |= WPA_DRIVER_FLAGS_OBSS_SCAN;
    capa->flags |= WPA_DRIVER_FLAGS_SAE;
    capa->flags |= WPA_DRIVER_FLAGS_AP;
    capa->flags |= WPA_DRIVER_FLAGS_ACS_OFFLOAD;
    capa->flags |= WPA_DRIVER_FLAGS_DFS_OFFLOAD;
    //capa->flags |= WPA_DRIVER_FLAGS_AP_UAPSD;
    capa->flags |= WPA_DRIVER_FLAGS_INACTIVITY_TIMER;
    capa->flags |= WPA_DRIVER_FLAGS_AP_MLME;
    capa->flags |= WPA_DRIVER_FLAGS_AP_TEARDOWN_SUPPORT;
    capa->flags |= WPA_DRIVER_FLAGS_AP_CSA;
#if !defined(RW610) && !defined(SD8801)
    capa->flags |= WPA_DRIVER_FLAGS_HT_2040_COEX;
#endif
    capa->flags |= WPA_DRIVER_FLAGS_HE_CAPABILITIES;

    capa->flags |= WPA_DRIVER_FLAGS_OFFCHANNEL_TX;

    capa->flags2 |= WPA_DRIVER_FLAGS2_AP_SME;

    capa->rrm_flags |= WPA_DRIVER_FLAGS_SUPPORT_RRM;
    //capa->rrm_flags |= WPA_DRIVER_FLAGS_SUPPORT_BEACON_REPORT;
    capa->rrm_flags |= WPA_DRIVER_FLAGS_TX_POWER_INSERTION;
    capa->rrm_flags |= WPA_DRIVER_FLAGS_SUPPORT_SET_SCAN_DWELL;

    capa->max_scan_ssids       = 10;
    capa->max_sched_scan_ssids = 10;
    capa->sched_scan_supported = 0;
    capa->max_remain_on_chan   = 5000;
    capa->max_stations         = 32;
    capa->max_acl_mac_addrs    = 32;

    if (if_ctx->extended_capa)
    {
        capa->extended_capa = if_ctx->extended_capa;
    }

    if (if_ctx->extended_capa_mask)
    {
        capa->extended_capa_mask = if_ctx->extended_capa_mask;
    }

    capa->extended_capa_len = if_ctx->extended_capa_len;

    if_ctx->capa = *capa;

    return 0;
}

static int wpa_drv_freertos_get_ext_capab(
    void *priv, enum wpa_driver_if_type type, const u8 **ext_capa, const u8 **ext_capa_mask, unsigned int *ext_capa_len)
{
    struct freertos_drv_if_ctx *if_ctx = NULL;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        return -1;
    }

    if_ctx = (struct freertos_drv_if_ctx *)priv;

    if (!ext_capa || !ext_capa_mask || !ext_capa_len)
        return -1;

    *ext_capa      = if_ctx->extended_capa;
    *ext_capa_mask = if_ctx->extended_capa_mask;
    *ext_capa_len  = if_ctx->extended_capa_len;

    return 0;
}

static int wpa_drv_freertos_get_bssid(void *priv, u8 *bssid)
{
    struct freertos_drv_if_ctx *if_ctx = NULL;

    if_ctx = priv;

    if (!if_ctx->associated)
        return -1;

    os_memcpy(bssid, if_ctx->bssid, ETH_ALEN);

    return 0;
}

static int wpa_drv_freertos_get_ssid(void *priv, u8 *ssid)
{
    struct freertos_drv_if_ctx *if_ctx = NULL;

    if_ctx = priv;

    if (!if_ctx->associated)
        return -1;

    wpa_printf(MSG_DEBUG, "%s: SSID size: %d\n", __func__, if_ctx->ssid_len);

    os_memcpy(ssid, if_ctx->ssid, if_ctx->ssid_len);

    return if_ctx->ssid_len;
}

static int wpa_drv_freertos_set_mac_addr(void *priv, const u8 *addr)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
#ifdef __ZEPHYR__
    const struct device *dev                        = NULL;
#endif
    int ret                                         = -1;

    if ((!priv) || (!addr))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

#ifdef __ZEPHYR__
    dev = net_if_get_device((struct net_if *)if_ctx->dev_ctx);
    /* TODO: net_if has no num (number of this interface) */
    wpa_printf(MSG_EXCESSIVE, "set_mac_addr for %c%c to " MACSTR, dev->name[0], dev->name[1], MAC2STR(addr));
#else
    wpa_printf(MSG_EXCESSIVE, "set_mac_addr for %c%c%d to " MACSTR, if_ctx->dev_ctx->name[0], if_ctx->dev_ctx->name[1],
               if_ctx->dev_ctx->num, MAC2STR(addr));
#endif

    os_memcpy(if_ctx->addr, addr, ETH_ALEN);

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->set_mac_addr(if_ctx->dev_priv, addr);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: set mac addr op failed\n", __func__);
        goto out;
    }

    ret = 0;

out:
    return ret;
}

static int wpa_drv_freertos_set_supp_port(void *priv, int authorized)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;

    int ret = -1;

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->set_supp_port(if_ctx->dev_priv, authorized, (char *)if_ctx->bssid);

    return ret;
}

static int wpa_drv_freertos_set_country(void *priv, const char *alpha2_arg)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    char alpha2[3];
    int ret = -1;

    if_ctx = priv;

    alpha2[0] = alpha2_arg[0];
    alpha2[1] = alpha2_arg[1];
    alpha2[2] = '\0';

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->set_country(if_ctx->dev_priv, alpha2);

    return ret;
}

static int wpa_drv_freertos_get_country(void *priv, char *alpha2)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if_ctx = priv;

    alpha2[0] = '\0';

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->get_country(if_ctx->dev_priv, alpha2);

    return ret;
}

static int wpa_drv_freertos_signal_poll(void *priv, struct wpa_signal_info *si)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }

    if (!si)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    os_memset(si, 0, sizeof(*si));

    if (dev_ops && dev_ops->signal_poll)
    {
        ret = dev_ops->signal_poll(if_ctx->dev_priv, si, if_ctx->bssid);
        if (ret)
        {
            wpa_printf(MSG_ERROR, "%s: Signal polling failed: %d", __func__, ret);
            goto out;
        }
    }
    else
    {
        wpa_printf(MSG_ERROR, "%s: Signal polling not supported", __func__);
        goto out;
    }

out:
    return ret;
}

static int wpa_drv_freertos_signal_monitor(void *priv, int threshold, int hysteresis)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (dev_ops && dev_ops->signal_monitor)
    {
        ret = dev_ops->signal_monitor(if_ctx->dev_priv, threshold, hysteresis);
        if (ret)
        {
            wpa_printf(MSG_ERROR, "%s: Signal monitor failed: %d", __func__, ret);
            goto out;
        }
    }
    else
    {
        wpa_printf(MSG_ERROR, "%s: Signal monitor not supported", __func__);
        goto out;
    }

out:
    return ret;
}

static int wpa_drv_freertos_send_action(void *priv,
                                        unsigned int freq,
                                        unsigned int wait_time,
                                        const u8 *dst,
                                        const u8 *src,
                                        const u8 *bssid,
                                        const u8 *data,
                                        size_t data_len,
                                        int no_cck)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;
    u8 *buf;
    struct ieee80211_hdr *hdr;

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    wpa_printf(MSG_EXCESSIVE,
               "Send Action frame ("
               "freq=%u MHz wait=%d ms no_cck=%d)",
               freq, wait_time, no_cck);

    buf = os_zalloc(24 + data_len);
    if (buf == NULL)
        return ret;
    os_memcpy(buf + 24, data, data_len);
    hdr                = (struct ieee80211_hdr *)buf;
    hdr->frame_control = IEEE80211_FC(WLAN_FC_TYPE_MGMT, WLAN_FC_STYPE_ACTION);
    os_memcpy(hdr->addr1, dst, ETH_ALEN);
    os_memcpy(hdr->addr2, src, ETH_ALEN);
    os_memcpy(hdr->addr3, bssid, ETH_ALEN);

    ret = dev_ops->send_mlme(if_ctx->dev_priv, buf, 24 + data_len, 0, freq, no_cck, 1, wait_time, 0);

    os_free(buf);

    return ret;
}

static void wpa_drv_freertos_send_action_cancel_wait(void *priv)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    struct wpa_supplicant *wpa_s;
    struct hostapd_data *hapd;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        return;
    }

    if_ctx = priv;
    if (if_ctx->is_ap)
    {
        hapd = if_ctx->hapd;
        if (!hapd)
        {
            wpa_printf(MSG_ERROR, "%s: Invalid hostapd handle", __func__);
            return;
        }
#ifdef CONFIG_DPP
        if (hapd->dpp_auth && !hapd->dpp_auth_ok_on_ack && hapd->dpp_auth->neg_freq > 0 &&
            hapd->dpp_auth->curr_freq != hapd->dpp_auth->neg_freq)
        {
            wpa_drv_freertos_cancel_remain_on_channel(priv);
        }
#endif
    }
    else
    {
        wpa_s = if_ctx->supp_if_ctx;
        if (!wpa_s)
        {
            wpa_printf(MSG_ERROR, "%s: Invalid wpa_supplicant handle", __func__);
            return;
        }
#ifdef CONFIG_DPP
        if (wpa_s->dpp_auth && !wpa_s->dpp_auth_ok_on_ack && wpa_s->dpp_auth->neg_freq > 0 &&
            wpa_s->dpp_auth->curr_freq != wpa_s->dpp_auth->neg_freq)
        {
            wpa_drv_freertos_cancel_remain_on_channel(priv);
        }
#endif
    }
}

static int wpa_drv_freertos_remain_on_channel(void *priv, unsigned int freq, unsigned int duration)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (dev_ops && dev_ops->remain_on_channel)
    {
        ret = dev_ops->remain_on_channel(if_ctx->dev_priv, freq, duration);
        if (ret)
        {
            wpa_printf(MSG_ERROR, "%s: Remain on channel failed: %d", __func__, ret);
            goto out;
        }
    }
    else
    {
        wpa_printf(MSG_ERROR, "%s: Remain on channel not supported", __func__);
        goto out;
    }

out:
    return ret;
}

static int wpa_drv_freertos_cancel_remain_on_channel(void *priv)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (dev_ops && dev_ops->cancel_remain_on_channel)
    {
        ret = dev_ops->cancel_remain_on_channel(if_ctx->dev_priv);
        if (ret)
        {
            wpa_printf(MSG_ERROR, "%s: Cancel Remain on channel failed: %d", __func__, ret);
            goto out;
        }
    }
    else
    {
        wpa_printf(MSG_ERROR, "%s: Cancel Remain on channel not supported", __func__);
        goto out;
    }

out:
    return ret;
}

static void wpa_drv_freertos_event_proc_mgmt_rx(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_RX_MGMT, event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_RX_MGMT, event);
}

static void wpa_drv_freertos_event_proc_eapol_rx(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_EAPOL_RX, event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_EAPOL_RX, event);
}

static void wpa_drv_freertos_event_ecsa_complete(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_CH_SWITCH, event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_CH_SWITCH, event);
}

static void wpa_drv_freertos_event_dfs_cac_started(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_DFS_CAC_STARTED, event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_DFS_CAC_STARTED, event);
}

static void wpa_drv_freertos_event_dfs_cac_finished(struct freertos_drv_if_ctx *if_ctx, union wpa_event_data *event)
{
#ifdef CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event_wrapper(if_ctx->hapd, EVENT_DFS_CAC_FINISHED, event);
    else
#endif
        wpa_supplicant_event_wrapper(if_ctx->supp_if_ctx, EVENT_DFS_CAC_FINISHED, event);
}

static void *wpa_drv_freertos_hapd_init(struct hostapd_data *hapd, struct wpa_init_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    #if 0
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    const struct netif *device                      = NULL;
#ifdef __ZEPHYR__
    const struct net_linkaddr *link_addr            = NULL;
#endif
    struct freertos_hostapd_dev_callbk_fns callbk_fns;

#ifdef __ZEPHYR__
    device = net_if_get_binding(params->ifname);
#else
    LOCK_TCPIP_CORE();
    device = netif_find(params->ifname);
    UNLOCK_TCPIP_CORE();
#endif

    if (!device)
    {
        wpa_printf(MSG_ERROR, "%s: Interface %s not found", __func__, params->ifname);
        goto out;
    }

    if_ctx = os_zalloc(sizeof(*if_ctx));

    if (if_ctx == NULL)
    {
        goto out;
    }

    if_ctx->hapd  = hapd;
    if_ctx->is_ap = 1;

    if_ctx->dev_ctx = device;
    if_ctx->drv_ctx = params->global_priv;

#ifdef __ZEPHYR__
    if_ctx->dev_ops = (struct freertos_wpa_supp_dev_ops *)net_if_get_dev_config((struct netif *)device);
    link_addr = net_if_get_link_addr((struct net_if *)device);
    os_memcpy(params->own_addr, link_addr->addr, link_addr->len);
#else
#if LWIP_NUM_NETIF_CLIENT_DATA > 0
    if_ctx->dev_ops = netif_get_client_data(device, LWIP_NETIF_CLIENT_DATA_INDEX_MAX);
#endif

    os_memcpy(params->own_addr, device->hwaddr, device->hwaddr_len);
#endif

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->hapd_init)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for hapd init", __func__);
        if_ctx = NULL;
        goto out;
    }

    os_memset(&callbk_fns, 0, sizeof(callbk_fns));

    callbk_fns.scan_done       = wpa_drv_freertos_event_proc_scan_done;
    callbk_fns.survey_res      = wpa_drv_freertos_event_proc_survey_res;
    callbk_fns.acs_channel_sel = wpa_drv_freertos_event_acs_channel_selected;
    callbk_fns.mgmt_rx         = wpa_drv_freertos_event_proc_mgmt_rx;
    callbk_fns.eapol_rx        = wpa_drv_freertos_event_proc_eapol_rx;
    callbk_fns.mgmt_tx_status  = wpa_drv_freertos_event_mgmt_tx_status;
    callbk_fns.mac_changed = wpa_drv_freertos_event_mac_changed;
    callbk_fns.chan_list_changed = wpa_drv_freertos_event_chan_list_changed;
    callbk_fns.ecsa_complete   = wpa_drv_freertos_event_ecsa_complete;
    callbk_fns.dfs_cac_started  = wpa_drv_freertos_event_dfs_cac_started;
    callbk_fns.dfs_cac_finished = wpa_drv_freertos_event_dfs_cac_finished;

    if_ctx->dev_priv = dev_ops->hapd_init(if_ctx, params->ifname, &callbk_fns);

    if (!if_ctx->dev_priv)
    {
        wpa_printf(MSG_ERROR, "%s: Failed to initialize the interface", __func__);
        os_free(if_ctx);
        if_ctx = NULL;
        goto out;
    }
out:
#endif
    return if_ctx;
}

static void wpa_drv_freertos_hapd_deinit(void *priv)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    if (!dev_ops->hapd_deinit)
    {
        wpa_printf(MSG_ERROR, "%s: No op registered for hapd deinit", __func__);
        return;
    }

    dev_ops->hapd_deinit(if_ctx->dev_priv);

    os_free(if_ctx);
}

static void clean_survey_results(struct survey_results *survey_results)
{
    struct freq_survey *survey, *tmp;

    if (dl_list_empty(&survey_results->survey_list))
        return;

    dl_list_for_each_safe(survey, tmp, &survey_results->survey_list, struct freq_survey, list)
    {
        dl_list_del(&survey->list);
        os_free(survey);
    }
}

static int wpa_drv_freertos_get_survey(void *priv, unsigned int freq)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;
    unsigned int i                                  = 0;
    union wpa_event_data *data;
    struct survey_results *survey_results = NULL;
    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    if_ctx->data = data = os_zalloc(sizeof(*data));

    if (!data)
    {
        wpa_printf(MSG_ERROR, "Failed to allocate data");
        goto out;
    }

    survey_results = &data->survey_results;

    dl_list_init(&survey_results->survey_list);

    if (freq)
        survey_results->freq_filter = freq;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->get_survey_results(if_ctx->dev_priv);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: get_survey_results op failed", __func__);
        goto out;
    }

    if_ctx->survey_res_get_in_prog = true;

    /* Wait for the device to populate the scan results */
    while ((if_ctx->survey_res_get_in_prog) && (i < SURVEY_TIMEOUT))
    {
        //OSA_TaskYield();
        os_sleep(1, 0);
        i++;
    }

    if (i == SURVEY_TIMEOUT)
    {
        wpa_printf(MSG_ERROR, "%s: Timed out waiting for survey results", __func__);
        ret = -1;
        goto out;
    }

#if CONFIG_HOSTAPD
    if (if_ctx->hapd)
        hostapd_event(if_ctx->hapd, EVENT_SURVEY, data);
    else
#endif
        wpa_supplicant_event(if_ctx->supp_if_ctx, EVENT_SURVEY, data);

    ret = 0;

out:
    if (if_ctx->data != NULL)
    {
        clean_survey_results(survey_results);
        os_free(if_ctx->data);
        if_ctx->data = NULL;
    }

    return ret;
}

int wpa_drv_freertos_do_acs(void *priv, struct drv_acs_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if ((!priv) || (!params))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->do_acs(if_ctx->dev_priv, params);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: do_acs op failed", __func__);
        goto out;
    }

    ret = 0;

out:
    return ret;
}

int wpa_drv_freertos_set_ap(void *priv, struct wpa_driver_ap_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if ((!priv) || (!params))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    if_ctx->beacon_set = params->reenable ? 0 : if_ctx->beacon_set;

    wpa_printf(MSG_EXCESSIVE, "beacon set : %d", if_ctx->beacon_set);
    wpa_hexdump(MSG_EXCESSIVE, "Beacon head", params->head, params->head_len);
    wpa_hexdump(MSG_EXCESSIVE, "Beacon tail", params->tail, params->tail_len);
    wpa_printf(MSG_EXCESSIVE, "beacon_int=%d", params->beacon_int);
    wpa_printf(MSG_EXCESSIVE, "beacon_rate=%u", params->beacon_rate);
    wpa_printf(MSG_EXCESSIVE, "rate_type=%d", params->rate_type);
    wpa_printf(MSG_EXCESSIVE, "dtim_period=%d", params->dtim_period);
    wpa_printf(MSG_EXCESSIVE, "ssid=%s", wpa_ssid_txt(params->ssid, params->ssid_len));
    wpa_printf(MSG_EXCESSIVE, "ht_opmode=%d", params->ht_opmode);
    if (params->freq)
    {
        wpa_printf(MSG_EXCESSIVE, "params->freq->freq: %d", params->freq->freq);
        wpa_printf(MSG_EXCESSIVE, "params->freq->channel: %d", params->freq->channel);
        wpa_printf(MSG_EXCESSIVE, "params->freq->ht_enabled: %d", params->freq->ht_enabled);
        wpa_printf(MSG_EXCESSIVE, "params->freq->sec_channel_offset: %d", params->freq->sec_channel_offset);
        wpa_printf(MSG_EXCESSIVE, "params->freq->vht_enabled: %d", params->freq->vht_enabled);
        wpa_printf(MSG_EXCESSIVE, "params->freq->he_enabled: %d", params->freq->he_enabled);
        wpa_printf(MSG_EXCESSIVE, "params->freq->bandwidth: %d", params->freq->bandwidth);
    }
    if (params->proberesp && params->proberesp_len)
    {
        wpa_hexdump(MSG_EXCESSIVE, "proberesp (offload)", params->proberesp, params->proberesp_len);
    }
    switch (params->hide_ssid)
    {
        case NO_SSID_HIDING:
            wpa_printf(MSG_EXCESSIVE, "hidden SSID not in use");
            break;
        case HIDDEN_SSID_ZERO_LEN:
            wpa_printf(MSG_EXCESSIVE, "hidden SSID zero len");
            break;
        case HIDDEN_SSID_ZERO_CONTENTS:
            wpa_printf(MSG_EXCESSIVE, "hidden SSID zero contents");
            break;
    }
    wpa_printf(MSG_EXCESSIVE, "privacy=%d", params->privacy);
    wpa_printf(MSG_EXCESSIVE, "auth_algs=0x%x", params->auth_algs);
    wpa_printf(MSG_EXCESSIVE, "wpa_version=0x%x", params->wpa_version);
    wpa_printf(MSG_EXCESSIVE, "key_mgmt_suites=0x%x", params->key_mgmt_suites);
    // num_suites = wpa_key_mgmt_to_suites(params->key_mgmt_suites, suites, ARRAY_SIZE(suites));
    wpa_printf(MSG_EXCESSIVE, "pairwise_ciphers=0x%x", params->pairwise_ciphers);
    // num_suites = wpa_cipher_to_cipher_suites(params->pairwise_ciphers, suites, ARRAY_SIZE(suites));
    wpa_printf(MSG_EXCESSIVE, "group_cipher=0x%x", params->group_cipher);
    // suite = wpa_cipher_to_cipher_suite(params->group_cipher);

    if (params->beacon_ies)
    {
        wpa_hexdump_buf(MSG_EXCESSIVE, "beacon_ies", params->beacon_ies);
    }
    if (params->proberesp_ies)
    {
        wpa_hexdump_buf(MSG_EXCESSIVE, "proberesp_ies", params->proberesp_ies);
    }
    if (params->assocresp_ies)
    {
        wpa_hexdump_buf(MSG_EXCESSIVE, "assocresp_ies", params->assocresp_ies);
    }

#ifdef CONFIG_IEEE80211AX
    if (params->he_spr_ctrl)
    {
        wpa_printf(MSG_EXCESSIVE, "he_spr_ctrl=0x%x", params->he_spr_ctrl);
    }
#endif

    if (params->twt_responder)
    {
        wpa_printf(MSG_EXCESSIVE, "twt_responder=%d", params->twt_responder);
    }

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->set_ap(if_ctx->dev_priv, if_ctx->beacon_set, params);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: set_ap op failed", __func__);
        goto out;
    }

    if_ctx->beacon_set = 1;

    ret = 0;

out:
    return ret;
}

static int wpa_drv_freertos_send_mlme(void *priv,
                                      const u8 *data,
                                      size_t data_len,
                                      int noack,
                                      unsigned int freq,
                                      const u16 *csa_offs,
                                      size_t csa_offs_len,
                                      int no_encrypt,
                                      unsigned int wait)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        return ret;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    wpa_printf(MSG_EXCESSIVE,
               "Send MLME ("
               "freq=%d MHz wait=%d ms no_encrypt=%d)",
               freq, wait, no_encrypt);

    return dev_ops->send_mlme(if_ctx->dev_priv, data, data_len, 0, freq, no_encrypt, 1, wait, 0);
}

static int wpa_drv_freertos_sta_add(void *priv, struct hostapd_sta_add_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if ((!priv) || (!params))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    if ((params->flags & WPA_STA_TDLS_PEER))
        return -1;

    wpa_printf(MSG_EXCESSIVE, "%s STA " MACSTR, params->set ? "Set" : "Add", MAC2STR(params->addr));
    /*msg = nl80211_bss_msg(bss, 0, params->set ? NL80211_CMD_SET_STATION : NL80211_CMD_NEW_STATION);
    if (!msg || nla_put(msg, NL80211_ATTR_MAC, ETH_ALEN, params->addr))
        goto fail;*/

    /*
     * Set the below properties only in one of the following cases:
     * 1. New station is added, already associated.
     * 2. Set WPA_STA_TDLS_PEER station.
     * 3. Set an already added unassociated station, if driver supports
     * full AP client state. (Set these properties after station became
     * associated will be rejected by the driver).
     */
    if (!params->set || (params->flags & WPA_STA_TDLS_PEER) || (params->set && (params->flags & WPA_STA_ASSOCIATED)))
    {
        wpa_hexdump(MSG_EXCESSIVE, "  * supported rates", params->supp_rates, params->supp_rates_len);
        wpa_printf(MSG_EXCESSIVE, "  * capability=0x%x", params->capability);
        /*if (nla_put(msg, NL80211_ATTR_STA_SUPPORTED_RATES, params->supp_rates_len, params->supp_rates) ||
            nla_put_u16(msg, NL80211_ATTR_STA_CAPABILITY, params->capability))
            goto fail;*/

        if (params->ht_capabilities)
        {
            wpa_hexdump(MSG_EXCESSIVE, "  * ht_capabilities", (u8 *)params->ht_capabilities,
                        sizeof(*params->ht_capabilities));
            /*   if (nla_put(msg, NL80211_ATTR_HT_CAPABILITY, sizeof(*params->ht_capabilities),
               params->ht_capabilities)) goto fail;*/
        }

        if (params->vht_capabilities)
        {
            wpa_hexdump(MSG_EXCESSIVE, "  * vht_capabilities", (u8 *)params->vht_capabilities,
                        sizeof(*params->vht_capabilities));
            /* if (nla_put(msg, NL80211_ATTR_VHT_CAPABILITY, sizeof(*params->vht_capabilities),
               params->vht_capabilities)) goto fail;*/
        }

        if (params->he_capab)
        {
            wpa_hexdump(MSG_EXCESSIVE, "  * he_capab", params->he_capab, params->he_capab_len);
            /*  if (nla_put(msg, NL80211_ATTR_HE_CAPABILITY, params->he_capab_len, params->he_capab))
                  goto fail;*/
        }

        if (params->he_6ghz_capab)
        {
            wpa_hexdump(MSG_EXCESSIVE, "  * he_6ghz_capab", params->he_6ghz_capab, sizeof(*params->he_6ghz_capab));
            /* if (nla_put(msg, NL80211_ATTR_HE_6GHZ_CAPABILITY, sizeof(*params->he_6ghz_capab), params->he_6ghz_capab))
                 goto fail;*/
        }

        if (params->ext_capab)
        {
            wpa_hexdump(MSG_EXCESSIVE, "  * ext_capab", params->ext_capab, params->ext_capab_len);
            /*  if (nla_put(msg, NL80211_ATTR_STA_EXT_CAPABILITY, params->ext_capab_len, params->ext_capab))
                  goto fail;*/
        }

        /* if (is_ap_interface(drv->nlmode) &&
             nla_put_u8(msg, NL80211_ATTR_STA_SUPPORT_P2P_PS,
                        params->support_p2p_ps ? NL80211_P2P_PS_SUPPORTED : NL80211_P2P_PS_UNSUPPORTED))
             goto fail;*/
    }
    if (!params->set)
    {
        if (params->aid)
        {
            wpa_printf(MSG_EXCESSIVE, "  * aid=%u", params->aid);
            /* if (nla_put_u16(msg, NL80211_ATTR_STA_AID, params->aid))
                 goto fail;*/
        }
        else
        {
            /*
             * cfg80211 validates that AID is non-zero, so we have
             * to make this a non-zero value for the TDLS case where
             * a stub STA entry is used for now and for a station
             * that is still not associated.
             */
            wpa_printf(MSG_EXCESSIVE, "  * aid=1 (%s workaround)",
                       (params->flags & WPA_STA_TDLS_PEER) ? "TDLS" : "UNASSOC_STA");
            /*if (nla_put_u16(msg, NL80211_ATTR_STA_AID, 1))
                goto fail;*/
        }
        wpa_printf(MSG_EXCESSIVE, "  * listen_interval=%u", params->listen_interval);
        /*if (nla_put_u16(msg, NL80211_ATTR_STA_LISTEN_INTERVAL, params->listen_interval))
            goto fail;*/
    }
    else if (params->aid && (params->flags & WPA_STA_TDLS_PEER))
    {
        wpa_printf(MSG_EXCESSIVE, "  * peer_aid=%u", params->aid);
        /* if (nla_put_u16(msg, NL80211_ATTR_PEER_AID, params->aid))
             goto fail;*/
    }
    else if ((params->flags & WPA_STA_ASSOCIATED))
    {
        wpa_printf(MSG_EXCESSIVE, "  * aid=%u", params->aid);
        wpa_printf(MSG_EXCESSIVE, "  * listen_interval=%u", params->listen_interval);
        /* if (nla_put_u16(msg, NL80211_ATTR_STA_AID, params->aid) ||
             nla_put_u16(msg, NL80211_ATTR_STA_LISTEN_INTERVAL, params->listen_interval))
             goto fail;*/
    }

    if (params->vht_opmode_enabled)
    {
        wpa_printf(MSG_EXCESSIVE, "  * opmode=%u", params->vht_opmode);
        /* if (nla_put_u8(msg, NL80211_ATTR_OPMODE_NOTIF, params->vht_opmode))
             goto fail;*/
    }

    if (params->supp_channels)
    {
        wpa_hexdump(MSG_EXCESSIVE, "  * supported channels", params->supp_channels, params->supp_channels_len);
        /* if (nla_put(msg, NL80211_ATTR_STA_SUPPORTED_CHANNELS, params->supp_channels_len, params->supp_channels))
             goto fail;*/
    }

    if (params->supp_oper_classes)
    {
        wpa_hexdump(MSG_EXCESSIVE, "  * supported operating classes", params->supp_oper_classes,
                    params->supp_oper_classes_len);
        /* if (nla_put(msg, NL80211_ATTR_STA_SUPPORTED_OPER_CLASSES, params->supp_oper_classes_len,
                     params->supp_oper_classes))
             goto fail;*/
    }

    /* os_memset(&upd, 0, sizeof(upd));
     upd.set  = sta_flags_nl80211(params->flags);
     upd.mask = upd.set | sta_flags_nl80211(params->flags_mask);*/

    /*
     * If the driver doesn't support full AP client state, ignore ASSOC/AUTH
     * flags, as nl80211 driver moves a new station, by default, into
     * associated state.
     *
     * On the other hand, if the driver supports that feature and the
     * station is added in unauthenticated state, set the
     * authenticated/associated bits in the mask to prevent moving this
     * station to associated state before it is actually associated.
     *
     * This is irrelevant for mesh mode where the station is added to the
     * driver as authenticated already, and ASSOCIATED isn't part of the
     * nl80211 API.
     */
    /*
    if (!is_mesh_interface(drv->nlmode))
    {
        if (!FULL_AP_CLIENT_STATE_SUPP(drv->capa.flags))
        {
            wpa_printf(MSG_EXCESSIVE, "Ignore ASSOC/AUTH flags since driver doesn't support full AP client state");
            upd.mask &= ~(BIT(NL80211_STA_FLAG_ASSOCIATED) | BIT(NL80211_STA_FLAG_AUTHENTICATED));
        }
        else if (!params->set && !(params->flags & WPA_STA_TDLS_PEER))
        {
            if (!(params->flags & WPA_STA_AUTHENTICATED))
                upd.mask |= BIT(NL80211_STA_FLAG_AUTHENTICATED);
            if (!(params->flags & WPA_STA_ASSOCIATED))
                upd.mask |= BIT(NL80211_STA_FLAG_ASSOCIATED);
        }
#ifdef CONFIG_MESH
    }
    else
    {
        if (params->plink_state == PLINK_ESTAB && params->peer_aid)
        {
            ret = nla_put_u16(msg, NL80211_ATTR_MESH_PEER_AID, params->peer_aid);
            if (ret)
                goto fail;
        }
#endif*/ /* CONFIG_MESH */
    // }

    // wpa_printf(MSG_EXCESSIVE, "  * flags set=0x%x mask=0x%x", upd.set, upd.mask);
    /*if (nla_put(msg, NL80211_ATTR_STA_FLAGS2, sizeof(upd), &upd))
        goto fail;*/

    /*
     if ((!params->set || (params->flags & WPA_STA_TDLS_PEER) || FULL_AP_CLIENT_STATE_SUPP(drv->capa.flags)) &&
         (params->flags & WPA_STA_WMM))
     {
         struct nlattr *wme = nla_nest_start(msg, NL80211_ATTR_STA_WME);

         wpa_printf(MSG_EXCESSIVE, "  * qosinfo=0x%x", params->qosinfo);
         if (!wme || nla_put_u8(msg, NL80211_STA_WME_UAPSD_QUEUES, params->qosinfo & WMM_QOSINFO_STA_AC_MASK) ||
             nla_put_u8(msg, NL80211_STA_WME_MAX_SP,
                        (params->qosinfo >> WMM_QOSINFO_STA_SP_SHIFT) & WMM_QOSINFO_STA_SP_MASK))
             goto fail;
         nla_nest_end(msg, wme);
     }*/

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->sta_add(if_ctx->dev_priv, params);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: sta_add op failed %d (%s)", __func__, ret, params->set ? "SET" : "NEW");
        goto out;
    }

    ret = 0;

out:
    return ret;
}

static int wpa_drv_freertos_sta_remove(void *priv, const u8 *addr)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int ret                                         = -1;

    if ((!priv) || (!addr))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    ret = dev_ops->sta_remove(if_ctx->dev_priv, addr);

    if (ret)
    {
        wpa_printf(MSG_ERROR, "%s: sta_remove op failed %d", __func__, ret);
        goto out;
    }

    ret = 0;

out:
    return ret;
}

static int wpa_drv_freertos_send_eapol(
    void *priv, const u8 *addr, const u8 *data, size_t data_len, int encrypt, const u8 *own_addr, u32 flags)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    unsigned char *bp;
    struct l2_ethhdr *eth;
    size_t len;
    int status = -1;
    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }

    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    len = data_len + sizeof(struct l2_ethhdr);
    bp  = os_malloc(len);
    if (bp == NULL)
    {
        wpa_printf(MSG_INFO, "EAPOL frame discarded, cannot malloc temp buffer of size %d!", len);
        goto out;
    }

    eth = (struct l2_ethhdr *)bp;
    os_memcpy(eth->h_dest, addr, ETH_ALEN);
    os_memcpy(eth->h_source, own_addr, ETH_ALEN);
    eth->h_proto = host_to_be16(ETH_P_EAPOL);
    os_memcpy(eth + 1, data, data_len);
    wpa_hexdump(MSG_MSGDUMP, "TX EAPOL", bp, len);

    status = dev_ops->hapd_send_eapol(if_ctx->dev_priv, bp, len);

    if (status)
    {
        wpa_printf(MSG_ERROR, "%s: hapd_send_eapol failed", __func__);
    }

    if (bp != NULL)
        os_free(bp);

out:
    return status;
}

static int wpa_drv_freertos_set_freq(void *priv, struct hostapd_freq_params *freq)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int status                                      = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }
    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    status = dev_ops->set_freq(if_ctx->dev_priv, freq);

    if (status)
    {
        wpa_printf(MSG_ERROR, "%s: set_freq failed", __func__);
    }

out:
    return status;
}

static int wpa_drv_freertos_set_rts(void *priv, int rts_threshold)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int status                                      = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }
    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    status = dev_ops->set_rts(if_ctx->dev_priv, rts_threshold);

    if (status)
    {
        wpa_printf(MSG_ERROR, "%s: set rts threshold failed", __func__);
    }

out:
    return status;
}

static int wpa_drv_freertos_set_frag(void *priv, int frag_threshold)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int status                                      = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }
    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    status = dev_ops->set_frag(if_ctx->dev_priv, frag_threshold);

    if (status)
    {
        wpa_printf(MSG_ERROR, "%s: set frag threshold failed", __func__);
    }

out:
    return status;
}

int wpa_drv_freertos_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr, u16 reason)
{
    struct ieee80211_mgmt mgmt;

    if ((!priv) || (!own_addr) || (!addr))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        return -1;
    }

    memset(&mgmt, 0, sizeof(mgmt));
    mgmt.frame_control = IEEE80211_FC(WLAN_FC_TYPE_MGMT, WLAN_FC_STYPE_DEAUTH);
    memcpy(mgmt.da, addr, ETH_ALEN);
    memcpy(mgmt.sa, own_addr, ETH_ALEN);
    memcpy(mgmt.bssid, own_addr, ETH_ALEN);
    mgmt.u.deauth.reason_code = host_to_le16(reason);

    return wpa_drv_freertos_send_mlme(priv, (u8 *)&mgmt, IEEE80211_HDRLEN + sizeof(mgmt.u.deauth), 0, 0, NULL, 0, 0, 0);
}

static int wpa_drv_freertos_sta_disassoc(void *priv, const u8 *own_addr, const u8 *addr, u16 reason)
{
    struct ieee80211_mgmt mgmt;

    if ((!priv) || (!own_addr) || (!addr))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        return -1;
    }

    memset(&mgmt, 0, sizeof(mgmt));
    mgmt.frame_control = IEEE80211_FC(WLAN_FC_TYPE_MGMT, WLAN_FC_STYPE_DISASSOC);
    memcpy(mgmt.da, addr, ETH_ALEN);
    memcpy(mgmt.sa, own_addr, ETH_ALEN);
    memcpy(mgmt.bssid, own_addr, ETH_ALEN);
    mgmt.u.disassoc.reason_code = host_to_le16(reason);

    return wpa_drv_freertos_send_mlme(priv, (u8 *)&mgmt, IEEE80211_HDRLEN + sizeof(mgmt.u.disassoc), 0, 0, NULL, 0, 0,
                                      0);
}

static int wpa_drv_freertos_stop_ap(void *if_priv)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int status                                      = -1;

    if (if_priv == NULL)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }
    if_ctx = if_priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    status = dev_ops->stop_ap(if_ctx->dev_priv);

    if (status)
    {
        wpa_printf(MSG_ERROR, "%s: stop ap failed", __func__);
        goto out;
    }

    if_ctx->beacon_set = 0;

out:
    return status;
}

static int wpa_drv_freertos_deinit_ap(void *if_priv)
{
    return wpa_drv_freertos_stop_ap(if_priv);
}

#if 0
static int wpa_drv_freertos_set_acl(void *if_priv, struct hostapd_acl_params *params)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int status                                      = -1;

    if ((!if_priv) || (!params))
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        goto out;
    }

    wpa_printf(MSG_EXCESSIVE, "Set %s ACL (num_mac_acl=%d)", params->acl_policy ? "Accept" : "Deny",
               params->num_mac_acl);

    if_ctx = if_priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    status = dev_ops->set_acl(if_ctx->dev_priv, params);

    if (status)
    {
        wpa_printf(MSG_ERROR, "%s: set acl failed", __func__);
    }
out:
    return status;
}
#endif

static void freertos_set_ht40_mode(struct hostapd_hw_modes *mode, int start, int end)
{
    int c;

    for (c = 0; c < mode->num_channels; c++)
    {
        struct hostapd_channel_data *chan = &mode->channels[c];
        if (chan->freq - 10 >= start && chan->freq + 10 <= end)
            chan->flag |= HOSTAPD_CHAN_HT40;
    }
}

static void freertos_set_ht40_mode_sec(struct hostapd_hw_modes *mode, int start, int end)
{
    int c;

    for (c = 0; c < mode->num_channels; c++)
    {
        struct hostapd_channel_data *chan = &mode->channels[c];
        if (!(chan->flag & HOSTAPD_CHAN_HT40))
            continue;
        if (chan->freq - 30 >= start && chan->freq - 10 <= end)
            chan->flag |= HOSTAPD_CHAN_HT40MINUS;
        if (chan->freq + 10 >= start && chan->freq + 30 <= end)
            chan->flag |= HOSTAPD_CHAN_HT40PLUS;
    }
}

#ifdef CONFIG_5GHz_SUPPORT
static void freertos_set_vht_mode(struct hostapd_hw_modes *mode, int start, int end, int max_bw)
{
    int c;

    for (c = 0; c < mode->num_channels; c++)
    {
        struct hostapd_channel_data *chan = &mode->channels[c];
        if (chan->freq - 10 >= start && chan->freq + 70 <= end)
            chan->flag |= HOSTAPD_CHAN_VHT_10_70;

        if (chan->freq - 30 >= start && chan->freq + 50 <= end)
            chan->flag |= HOSTAPD_CHAN_VHT_30_50;

        if (chan->freq - 50 >= start && chan->freq + 30 <= end)
            chan->flag |= HOSTAPD_CHAN_VHT_50_30;

        if (chan->freq - 70 >= start && chan->freq + 10 <= end)
            chan->flag |= HOSTAPD_CHAN_VHT_70_10;

        if (max_bw >= 160)
        {
            if (chan->freq - 10 >= start && chan->freq + 150 <= end)
                chan->flag |= HOSTAPD_CHAN_VHT_10_150;

            if (chan->freq - 30 >= start && chan->freq + 130 <= end)
                chan->flag |= HOSTAPD_CHAN_VHT_30_130;

            if (chan->freq - 50 >= start && chan->freq + 110 <= end)
                chan->flag |= HOSTAPD_CHAN_VHT_50_110;

            if (chan->freq - 70 >= start && chan->freq + 90 <= end)
                chan->flag |= HOSTAPD_CHAN_VHT_70_90;

            if (chan->freq - 90 >= start && chan->freq + 70 <= end)
                chan->flag |= HOSTAPD_CHAN_VHT_90_70;

            if (chan->freq - 110 >= start && chan->freq + 50 <= end)
                chan->flag |= HOSTAPD_CHAN_VHT_110_50;

            if (chan->freq - 130 >= start && chan->freq + 30 <= end)
                chan->flag |= HOSTAPD_CHAN_VHT_130_30;

            if (chan->freq - 150 >= start && chan->freq + 10 <= end)
                chan->flag |= HOSTAPD_CHAN_VHT_150_10;
        }
    }
}
#endif

static void freertos_reg_rule_max_eirp(struct hostapd_hw_modes *mode, int start, int end, int max_eirp)
{
    int c;

    for (c = 0; c < mode->num_channels; c++)
    {
        struct hostapd_channel_data *chan = &mode->channels[c];
        if ((u32)chan->freq >= start && (u32)chan->freq <= end)
            chan->max_tx_power = max_eirp;
    }
}

#define MAX_NUM_CHANNEL    (14)
#if CONFIG_5GHz_SUPPORT
#define MAX_NUM_CHANNEL_5G (28)
#endif

static struct hostapd_hw_modes *wpa_drv_freertos_get_hw_feature_data(void *if_priv, u16 *num_modes, u16 *flags, u8 *dfs)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int status                                      = -1;
    struct hostapd_hw_modes *modes;
    size_t i;
    int start, end;
#if CONFIG_5GHz_SUPPORT
    int k;
    bool support_5G = 0;
#endif

    if (!if_priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid params", __func__);
        return NULL;
    }

    if_ctx = if_priv;
    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    *num_modes = 2;
#if CONFIG_5GHz_SUPPORT
    support_5G = dev_ops->get_modes(if_ctx->dev_priv);
    if (support_5G)
    {
        *num_modes += 1;
    }
#endif

    *flags     = 0;
    *dfs       = 0;

    modes = os_zalloc(*num_modes * sizeof(struct hostapd_hw_modes));
    if (modes == NULL)
        return NULL;

    //.1
    modes[0].mode         = HOSTAPD_MODE_IEEE80211B;
    modes[0].num_channels = MAX_NUM_CHANNEL;
    modes[0].num_rates    = 4;
    modes[0].channels     = os_zalloc(MAX_NUM_CHANNEL * sizeof(struct hostapd_channel_data));
    modes[0].rates        = os_zalloc(modes[0].num_rates * sizeof(int));
    if (modes[0].channels == NULL || modes[0].rates == NULL)
        goto fail;

    start = 2412;
    end   = 2484;
    for (i = 0; i < MAX_NUM_CHANNEL; i++)
    {
        modes[0].channels[i].chan = i + 1;

        if (i == 13) // channel=14
            modes[0].channels[i].freq = 2484;
        else
            modes[0].channels[i].freq = 2412 + 5 * i;

        modes[0].channels[i].flag = 0;

        modes[0].channels[i].allowed_bw =
            HOSTAPD_CHAN_WIDTH_20;
    }

    freertos_reg_rule_max_eirp(&modes[0], start, end, 20);

    modes[0].rates[0]  = 10;
    modes[0].rates[1]  = 20;
    modes[0].rates[2]  = 55;
    modes[0].rates[3]  = 110;

    //.2
    modes[1].mode         = HOSTAPD_MODE_IEEE80211G;
    modes[1].num_channels = MAX_NUM_CHANNEL;
    modes[1].num_rates    = 12;
    modes[1].channels     = os_zalloc(MAX_NUM_CHANNEL * sizeof(struct hostapd_channel_data));
    modes[1].rates        = os_zalloc(modes[1].num_rates * sizeof(int));
    if (modes[1].channels == NULL || modes[1].rates == NULL)
        goto fail;

    start = 2412;
    end   = 2484;
    for (i = 0; i < MAX_NUM_CHANNEL; i++)
    {
        modes[1].channels[i].chan = i + 1;

        if (i == 13) // channel=14
            modes[1].channels[i].freq = 2484;
        else
            modes[1].channels[i].freq = 2412 + 5 * i;

        modes[1].channels[i].flag = 0;

        modes[1].channels[i].allowed_bw =
            HOSTAPD_CHAN_WIDTH_20 | HOSTAPD_CHAN_WIDTH_40P | HOSTAPD_CHAN_WIDTH_40M | HOSTAPD_CHAN_WIDTH_80;
    }

    freertos_set_ht40_mode(&modes[1], start, end);
    freertos_set_ht40_mode_sec(&modes[1], start, end);
    freertos_reg_rule_max_eirp(&modes[1], start, end, 20);

    modes[1].rates[0]  = 10;
    modes[1].rates[1]  = 20;
    modes[1].rates[2]  = 55;
    modes[1].rates[3]  = 110;
    modes[1].rates[4]  = 60;
    modes[1].rates[5]  = 90;
    modes[1].rates[6]  = 120;
    modes[1].rates[7]  = 180;
    modes[1].rates[8]  = 240;
    modes[1].rates[9]  = 360;
    modes[1].rates[10] = 480;
    modes[1].rates[11] = 540;

#if CONFIG_5GHz_SUPPORT
    //.3
    if (support_5G)
    {
        modes[2].mode         = HOSTAPD_MODE_IEEE80211A;
        modes[2].num_channels = MAX_NUM_CHANNEL_5G;
        modes[2].num_rates    = 8;
        modes[2].channels     = os_zalloc(modes[2].num_channels * sizeof(struct hostapd_channel_data));
        modes[2].rates        = os_zalloc(modes[2].num_rates * sizeof(int));
        if (modes[2].channels == NULL || modes[2].rates == NULL)
            goto fail;

        start = 5180;
        end   = 5885;

        k = 0;
        // 5G band1 Channel: 36, 40, 44, 48
        for (i = 0; i < 4; i++)
        {
            modes[2].channels[k].chan       = 36 + (i * 4);
            modes[2].channels[k].freq       = 5180 + (i * 20);
            modes[2].channels[k].flag       = 0;
            modes[2].channels[k].allowed_bw = HOSTAPD_CHAN_WIDTH_20 | HOSTAPD_CHAN_WIDTH_40P | HOSTAPD_CHAN_WIDTH_40M |
                                              HOSTAPD_CHAN_WIDTH_80 | HOSTAPD_CHAN_WIDTH_160;
            k++;
        }

        // 5G band2 Channel: 52, 56, 60, 64
        for (i = 0; i < 4; i++)
        {
            modes[2].channels[k].chan       = 52 + (i * 4);
            modes[2].channels[k].freq       = 5260 + (i * 20);
            modes[2].channels[k].flag       = 0;
            modes[2].channels[k].allowed_bw = HOSTAPD_CHAN_WIDTH_20 | HOSTAPD_CHAN_WIDTH_40P | HOSTAPD_CHAN_WIDTH_40M |
                                              HOSTAPD_CHAN_WIDTH_80 | HOSTAPD_CHAN_WIDTH_160;
            k++;
        }

        // 5G band3 Channel: 100, 104, 108. 112, 116, 120, 124, 128, 132, 136, 140
        for (i = 0; i < 12; i++)
        {
            modes[2].channels[k].chan       = 100 + (i * 4);
            modes[2].channels[k].freq       = 5500 + (i * 20);
            modes[2].channels[k].flag       = 0;
            modes[2].channels[k].allowed_bw = HOSTAPD_CHAN_WIDTH_20 | HOSTAPD_CHAN_WIDTH_40P | HOSTAPD_CHAN_WIDTH_40M |
                                              HOSTAPD_CHAN_WIDTH_80 | HOSTAPD_CHAN_WIDTH_160;
            k++;
        }

        // 5G band4 Channel: 149, 153, 157, 161, 165
        for (i = 0; i < 8; i++)
        {
            modes[2].channels[k].chan       = 149 + (i * 4);
            modes[2].channels[k].freq       = 5745 + (i * 20);
            modes[2].channels[k].flag       = 0;
            modes[2].channels[k].allowed_bw = HOSTAPD_CHAN_WIDTH_20 | HOSTAPD_CHAN_WIDTH_40P | HOSTAPD_CHAN_WIDTH_40M |
                                              HOSTAPD_CHAN_WIDTH_80 | HOSTAPD_CHAN_WIDTH_160;
            k++;
        }

        freertos_set_ht40_mode(&modes[2], start, end);
        freertos_set_ht40_mode_sec(&modes[2], start, end);
        freertos_reg_rule_max_eirp(&modes[2], start, end, 20);
        freertos_set_vht_mode(&modes[2], start, end, 80);

        modes[2].rates[0] = 60;
        modes[2].rates[1] = 90;
        modes[2].rates[2] = 120;
        modes[2].rates[3] = 180;
        modes[2].rates[4] = 240;
        modes[2].rates[5] = 360;
        modes[2].rates[6] = 480;
        modes[2].rates[7] = 540;
    }
#endif

    status = dev_ops->set_modes(if_ctx->dev_priv, modes);
    if (status)
    {
        wpa_printf(MSG_ERROR, "%s: set modes failed", __func__);
        goto fail;
    }

    return modes;

fail:
    if (modes)
    {
        for (i = 0; i < *num_modes; i++)
        {
            os_free(modes[i].channels);
            os_free(modes[i].rates);
        }
        os_free(modes);
    }

    return NULL;
}

static int wpa_drv_freertos_dpp_listen(void *priv, bool enable)
{
    struct freertos_drv_if_ctx *if_ctx              = NULL;
    const struct freertos_wpa_supp_dev_ops *dev_ops = NULL;
    int status                                      = -1;

    if (!priv)
    {
        wpa_printf(MSG_ERROR, "%s: Invalid handle", __func__);
        goto out;
    }
    if_ctx = priv;

    dev_ops = (struct freertos_wpa_supp_dev_ops *)if_ctx->dev_ops;

    status = dev_ops->dpp_listen(if_ctx->dev_priv, enable);

    if (status)
    {
        wpa_printf(MSG_ERROR, "%s: set frag threshold failed", __func__);
    }

out:
    return status;
}

const struct wpa_driver_ops wpa_driver_freertos_ops = {
    .name                     = "FreeRTOS",
    .desc                     = "FreeRTOS wpa_supplicant driver",
    .global_init              = wpa_drv_freertos_global_init,
    .global_deinit            = wpa_drv_freertos_global_deinit,
    .init2                    = wpa_drv_freertos_init,
    .deinit                   = wpa_drv_freertos_deinit,
    .scan2                    = wpa_drv_freertos_scan2,
    .set_default_scan_ies     = wpa_drv_freertos_set_default_scan_ies,
    .sched_scan               = wpa_drv_freertos_sched_scan,
    .stop_sched_scan          = wpa_drv_freertos_stop_sched_scan,
    .abort_scan               = wpa_drv_freertos_abort_scan,
    .get_scan_results2        = wpa_drv_freertos_get_scan_results2,
    .authenticate             = wpa_drv_freertos_authenticate,
    .associate                = wpa_drv_freertos_associate,
    .get_capa                 = wpa_drv_freertos_get_capa,
    .get_ext_capab            = wpa_drv_freertos_get_ext_capab,
    .get_bssid                = wpa_drv_freertos_get_bssid,
    .get_ssid                 = wpa_drv_freertos_get_ssid,
    .set_mac_addr             = wpa_drv_freertos_set_mac_addr,
    .set_supp_port            = wpa_drv_freertos_set_supp_port,
    .set_country              = wpa_drv_freertos_set_country,
    .get_country              = wpa_drv_freertos_get_country,
    .deauthenticate           = wpa_drv_freertos_deauthenticate,
    .set_key                  = wpa_drv_freertos_set_key,
    .set_rekey_info           = wpa_drv_freertos_set_rekey_info,
    .signal_poll              = wpa_drv_freertos_signal_poll,
    .signal_monitor           = wpa_drv_freertos_signal_monitor,
    .send_action              = wpa_drv_freertos_send_action,
    .send_action_cancel_wait  = wpa_drv_freertos_send_action_cancel_wait,
    .remain_on_channel        = wpa_drv_freertos_remain_on_channel,
    .cancel_remain_on_channel = wpa_drv_freertos_cancel_remain_on_channel,
    .hapd_init                = wpa_drv_freertos_hapd_init,
    .hapd_deinit              = wpa_drv_freertos_hapd_deinit,
    .get_survey               = wpa_drv_freertos_get_survey,
    .do_acs                   = wpa_drv_freertos_do_acs,
    .set_ap                   = wpa_drv_freertos_set_ap,
    .send_mlme                = wpa_drv_freertos_send_mlme,
    .sta_add                  = wpa_drv_freertos_sta_add,
    .sta_remove               = wpa_drv_freertos_sta_remove,
    .hapd_send_eapol = wpa_drv_freertos_send_eapol,
    .set_freq        = wpa_drv_freertos_set_freq,
    .set_rts         = wpa_drv_freertos_set_rts,
    .set_frag        = wpa_drv_freertos_set_frag,
    .sta_deauth      = wpa_drv_freertos_sta_deauth,
    .sta_disassoc    = wpa_drv_freertos_sta_disassoc,
    .stop_ap         = wpa_drv_freertos_stop_ap,
    .deinit_ap       = wpa_drv_freertos_deinit_ap,
    // .set_acl                  = wpa_drv_freertos_set_acl,
    .get_hw_feature_data = wpa_drv_freertos_get_hw_feature_data,
    .dpp_listen          = wpa_drv_freertos_dpp_listen,
};
