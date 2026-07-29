/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <wlan_dev.h>
#include <wlan_mgnt.h>
#include <wlan_prot.h>
#include <wlan_cfg.h>
#include <rtdbg.h>
#include "wifi_if.h"
#include "rwnx_defs.h"
#include "rwnx_platform.h"
#include "rwnx_main.h"
#include "rwnx_msg_tx.h"
#include "aicwf_sdio.h"
#include "osal_debug.h"
#include "netal_service.h"

#define AIC8800MC_DEV_NAME  "wifi_aic8800mc"
#define WLAN_DEV_NAME "wlan0"
extern struct rwnx_plat *g_rwnx_plat;

static struct rt_wlan_device aic8800mc_wlan_dev;
static rt_wlan_mode_t aic8800mc_wlan_mode = RT_WLAN_NONE;

static struct rt_device wifi_device;

static rt_err_t aic8800mc_wlan_dev_init(struct rt_wlan_device *wlan)
{
    LOG_E("aic8800mc wlan init\n");
    wifi_open();
    return RT_EOK;
}
static rt_err_t aic8800mc_wlan_dev_mode(struct rt_wlan_device *wlan, rt_wlan_mode_t mode)
{
    LOG_E("aic8800mc wlan set mode: %d\n", mode);
    aic8800mc_wlan_mode = mode;
    return RT_EOK;
}

static rt_err_t aic8800mc_wlan_dev_scan(struct rt_wlan_device *wlan, struct rt_scan_info *scan_info)
{
    struct rwnx_hw *rwnx_hw;
    int ret;
    rt_kprintf("1111111111111\n");
    if (g_rwnx_plat == RT_NULL || g_rwnx_plat->sdiodev == RT_NULL)
    {
        LOG_E("aic8800mc not ready for scan\n");
        return -RT_ERROR;
    }

    rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
    if (rwnx_hw == RT_NULL)
    {
        LOG_E("rwnx_hw is NULL\n");
        return -RT_ERROR;
    }

    ret = rwnx_send_fhcustmsg_scan_req(rwnx_hw);
    if (ret)
    {
        LOG_E("scan req failed: %d\n", ret);
        return -RT_ERROR;
    }

    LOG_E("scan started\n");
    return RT_EOK;
}

static rt_err_t aic8800mc_wlan_dev_join(struct rt_wlan_device *wlan, struct rt_sta_info *sta_info)
{
    struct rwnx_hw *rwnx_hw;
    int ret;
    char ssid[RT_WLAN_SSID_MAX_LENGTH + 1] = {0};
    char password[RT_WLAN_PASSWORD_MAX_LENGTH + 1] = {0};

    if (g_rwnx_plat == RT_NULL || g_rwnx_plat->sdiodev == RT_NULL)
    {
        LOG_E("aic8800mc not ready for join\n");
        return -RT_ERROR;
    }

    rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
    if (rwnx_hw == RT_NULL)
    {
        LOG_E("rwnx_hw is NULL\n");
        return -RT_ERROR;
    }

    rt_memcpy(ssid, sta_info->ssid.val, sta_info->ssid.len);
    ssid[sta_info->ssid.len] = '\0';

    if (sta_info->key.len > 0)
    {
        rt_memcpy(password, sta_info->key.val, sta_info->key.len);
        password[sta_info->key.len] = '\0';
    }

    ret = rwnx_send_fhcustmsg_connect_req(rwnx_hw, ssid, password);
    if (ret)
    {
        LOG_E("connect req failed: %d\n", ret);
        return -RT_ERROR;
    }

    LOG_E("joining SSID: %s\n", ssid);
    return RT_EOK;
}

static rt_err_t aic8800mc_wlan_dev_softap(struct rt_wlan_device *wlan, struct rt_ap_info *ap_info)
{
    LOG_E("aic8800mc softap not supported\n");
    return -RT_ENOSYS;
}

static rt_err_t aic8800mc_wlan_dev_disconnect(struct rt_wlan_device *wlan)
{
    struct rwnx_hw *rwnx_hw;
    int ret;

    if (g_rwnx_plat == RT_NULL || g_rwnx_plat->sdiodev == RT_NULL)
    {
        LOG_E("aic8800mc not ready for disconnect\n");
        return -RT_ERROR;
    }

    rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
    if (rwnx_hw == RT_NULL)
    {
        LOG_E("rwnx_hw is NULL\n");
        return -RT_ERROR;
    }

    ret = rwnx_send_fhcustmsg_disconnect_req(rwnx_hw);
    if (ret)
    {
        LOG_E("disconnect req failed: %d\n", ret);
        return -RT_ERROR;
    }

    LOG_E("disconnected\n");
    return RT_EOK;
}

static rt_err_t aic8800mc_wlan_dev_ap_stop(struct rt_wlan_device *wlan)
{
    return -RT_ENOSYS;
}

static rt_err_t aic8800mc_wlan_dev_ap_deauth(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    return -RT_ENOSYS;
}

static rt_err_t aic8800mc_wlan_dev_scan_stop(struct rt_wlan_device *wlan)
{
    return -RT_ENOSYS;
}

static int aic8800mc_wlan_dev_get_rssi(struct rt_wlan_device *wlan)
{
    struct rwnx_hw *rwnx_hw;
    struct fhcustmsg_wlan_status_cfm cfm;
    int ret;

    if (g_rwnx_plat == RT_NULL || g_rwnx_plat->sdiodev == RT_NULL)
        return -1;

    rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
    if (rwnx_hw == RT_NULL)
        return -1;

    ret = rwnx_send_fhcustmsg_wlan_status_req(rwnx_hw, &cfm);
    if (ret)
        return -1;

    return cfm.rssi;
}

static rt_err_t aic8800mc_wlan_dev_set_powersave(struct rt_wlan_device *wlan, int level)
{
    return -RT_ENOSYS;
}

static int aic8800mc_wlan_dev_get_powersave(struct rt_wlan_device *wlan)
{
    return 0;
}

static rt_err_t aic8800mc_wlan_dev_cfg_promisc(struct rt_wlan_device *wlan, rt_bool_t start)
{
    return -RT_ENOSYS;
}

static rt_err_t aic8800mc_wlan_dev_cfg_filter(struct rt_wlan_device *wlan, struct rt_wlan_filter *filter)
{
    return -RT_ENOSYS;
}

static rt_err_t aic8800mc_wlan_dev_set_channel(struct rt_wlan_device *wlan, int channel)
{
    return -RT_ENOSYS;
}

static int aic8800mc_wlan_dev_get_channel(struct rt_wlan_device *wlan)
{
    return -1;
}

static rt_err_t aic8800mc_wlan_dev_set_country(struct rt_wlan_device *wlan, rt_country_code_t country_code)
{
    return -RT_ENOSYS;
}

static rt_country_code_t aic8800mc_wlan_dev_get_country(struct rt_wlan_device *wlan)
{
    return RT_COUNTRY_CHINA;
}

static rt_err_t aic8800mc_wlan_dev_set_mac(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    struct rwnx_hw *rwnx_hw;
    int ret;

    if (g_rwnx_plat == RT_NULL || g_rwnx_plat->sdiodev == RT_NULL)
        return -RT_ERROR;

    rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
    if (rwnx_hw == RT_NULL)
        return -RT_ERROR;

    ret = rwnx_send_fhcustmsg_set_mac_addr_req(rwnx_hw, mac);
    if (ret)
    {
        LOG_E("set mac failed: %d\n", ret);
        return -RT_ERROR;
    }

    rt_memcpy(g_custom_msg_vnet.mac_addr, mac, 6);
    return RT_EOK;
}

static rt_err_t aic8800mc_wlan_dev_get_mac(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    if (g_custom_msg_vnet.mac_addr[0] || g_custom_msg_vnet.mac_addr[1] ||
            g_custom_msg_vnet.mac_addr[2] || g_custom_msg_vnet.mac_addr[3] ||
            g_custom_msg_vnet.mac_addr[4] || g_custom_msg_vnet.mac_addr[5])
    {
        rt_memcpy(mac, g_custom_msg_vnet.mac_addr, 6);
        return RT_EOK;
    }

    struct rwnx_hw *rwnx_hw;
    struct fhcustmsg_mac_addr_cfm cfm;
    int ret;

    if (g_rwnx_plat == RT_NULL || g_rwnx_plat->sdiodev == RT_NULL)
        return -RT_ERROR;

    rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
    if (rwnx_hw == RT_NULL)
        return -RT_ERROR;

    ret = rwnx_send_fhcustmsg_get_mac_addr_req(rwnx_hw, &cfm);
    if (ret)
    {
        LOG_E("get mac failed: %d\n", ret);
        return -RT_ERROR;
    }

    rt_memcpy(mac, cfm.mac_addr, 6);
    return RT_EOK;
}

static int aic8800mc_wlan_dev_recv(struct rt_wlan_device *wlan, void *buff, int len)
{
    return rt_wlan_dev_report_data(wlan, buff, len);
}

void aic8800mc_wlan_rx_data(void *buff, int len)
{
    aic8800mc_wlan_dev_recv(&aic8800mc_wlan_dev, buff, len);
}

static int aic8800mc_wlan_dev_send(struct rt_wlan_device *wlan, void *buff, int len)
{
    if (p_netal_eth_env == RT_NULL || p_netal_eth_env->ops.eth_send == RT_NULL)
    {
        LOG_E("eth_send not ready\n");
        return -1;
    }

    p_netal_eth_env->ops.eth_send(buff, len);
    return len;
}

static const struct rt_wlan_dev_ops aic8800mc_wlan_ops =
{
    aic8800mc_wlan_dev_init,
    aic8800mc_wlan_dev_mode,
    aic8800mc_wlan_dev_scan,
    aic8800mc_wlan_dev_join,
    aic8800mc_wlan_dev_softap,
    aic8800mc_wlan_dev_disconnect,
    aic8800mc_wlan_dev_ap_stop,
    aic8800mc_wlan_dev_ap_deauth,
    aic8800mc_wlan_dev_scan_stop,
    aic8800mc_wlan_dev_get_rssi,
    aic8800mc_wlan_dev_set_powersave,
    aic8800mc_wlan_dev_get_powersave,
    aic8800mc_wlan_dev_cfg_promisc,
    aic8800mc_wlan_dev_cfg_filter,
    aic8800mc_wlan_dev_set_channel,
    aic8800mc_wlan_dev_get_channel,
    aic8800mc_wlan_dev_set_country,
    aic8800mc_wlan_dev_get_country,
    aic8800mc_wlan_dev_set_mac,
    aic8800mc_wlan_dev_get_mac,
    aic8800mc_wlan_dev_recv,
    aic8800mc_wlan_dev_send,
};

void aic8800mc_wlan_report_event(rt_wlan_dev_event_t event, void *buff, int len)
{
    struct rt_wlan_buff wlan_buff;
    wlan_buff.data = buff;
    wlan_buff.len = len;
    rt_wlan_dev_indicate_event_handle(&aic8800mc_wlan_dev, event, &wlan_buff);
}

void aic8800mc_wlan_report_connect(void)
{
    aic8800mc_wlan_report_event(RT_WLAN_DEV_EVT_CONNECT, RT_NULL, 0);
}

void aic8800mc_wlan_report_disconnect(void)
{
    aic8800mc_wlan_report_event(RT_WLAN_DEV_EVT_DISCONNECT, RT_NULL, 0);
}

void aic8800mc_wlan_report_connect_fail(void)
{
    aic8800mc_wlan_report_event(RT_WLAN_DEV_EVT_CONNECT_FAIL, RT_NULL, 0);
}

void aic8800mc_wlan_report_scan_done(void)
{
    aic8800mc_wlan_report_event(RT_WLAN_DEV_EVT_SCAN_DONE, RT_NULL, 0);
}

void aic8800mc_wlan_report_scan_result(struct rt_wlan_info *info)
{
    struct rt_wlan_buff wlan_buff;
    wlan_buff.data = info;
    wlan_buff.len = sizeof(struct rt_wlan_info);
    rt_wlan_dev_indicate_event_handle(&aic8800mc_wlan_dev, RT_WLAN_DEV_EVT_SCAN_REPORT, &wlan_buff);
}

int aic8800mc_wlan_init(void)
{
    rt_err_t ret;
    rt_memset(&aic8800mc_wlan_dev, 0, sizeof(aic8800mc_wlan_dev));

    ret = rt_wlan_dev_register(&aic8800mc_wlan_dev,
                               WLAN_DEV_NAME,
                               &aic8800mc_wlan_ops,
                               RT_WLAN_FLAG_STA_ONLY,
                               RT_NULL);
    if (ret != RT_EOK)
    {
        LOG_E("wlan dev register failed: %d\n", ret);
        return -1;
    }

    LOG_E("aic8800mc wlan device registered: %s\n", WLAN_DEV_NAME);
    return 0;
}
INIT_PRE_APP_EXPORT(aic8800mc_wlan_init);

// int aic_wifi_device(void)
// {
//     rt_err_t rt_err = RT_EOK;

//     rt_err = rt_device_register();
// }
// INIT_PRE_APP_EXPORT(aic_wifi_device);

int aic8800mc_wifi_init(void)
{
    rt_err_t ret;

    ret = rt_wlan_set_mode(WLAN_DEV_NAME, RT_WLAN_STATION);
    rt_kprintf("set_mode ret=%d\n", ret);
    return ret;
}
INIT_APP_EXPORT(aic8800mc_wifi_init);
