/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi WPS platform functions.
 */

#include "wifi_port_priv.h"
#include "wifi_api.h"
#include "aic_log.h"

extern void aic_set_wps_status_cb(aic_wps_status_notify cb);

#define WPSNOTIFYLISTNUM    5

static WSPNOTIFYFUNC WpsNotifyList[5];
static int WpsNotifyCnt = 0;

struct plat_wps_dev wps_dev = {0,};

/**
 * @brief Set the WPS provisioning method
 * @param method WPS method (0=PIN, 1=PBC button)
 */
static void aicwf_app_wlan_set_wps_method(int method)
{
    wps_dev.wpsmethod = method;
}

/**
 * @brief Get the current WPS provisioning method
 * @return WPS method (0=PIN, 1=PBC)
 */
static int aicwf_app_wlan_get_wps_method(void)
{
    return wps_dev.wpsmethod;
}

/**
 * @brief Register a WPS state notification callback
 * @param NotifyFunc Notification callback function pointer
 * @return 0=success, -1=notification list is full
 */
int app_wlan_wps_notify_reg(WSPNOTIFYFUNC NotifyFunc)
{
    if (WpsNotifyCnt < WPSNOTIFYLISTNUM)
    {
        WpsNotifyList[WpsNotifyCnt++] = NotifyFunc;
        return 0;
    }
    DBG_APP_INF("Wps notify list is full");
    return -1;
}

/**
 * @brief WPS internal state notification dispatch
 *
 * Iterates over the registered notification callbacks and calls each of them
 * to report the WPS state change.
 *
 * @param state WPS notification state code
 * @return 0
 */
static int aicwf_app_wlan_wps_notify(unsigned char state)
{
    int i = 0;
    DBG_APP_INF("Wps notify %d", state);
    for (i = 0; i < WpsNotifyCnt; i++)
    {
        (WpsNotifyList[i])(state);
    }
    return 0;
}

/**
 * @brief Get the WPS registration state
 * @return WPS registration result state code
 */
int app_wlan_wps_get_reg_status(void)
{
    DBG_APP_INF("[WPS] app_wlan_wps_get_reg_status %d\r\n", wps_dev.wps_ret);
    return wps_dev.wps_ret;
}

/**
 * @brief WPS state callback handler
 *
 * Receives low-level WPS events and converts them into platform WPS states.
 * Triggers the corresponding notification callback according to the event type.
 *
 * @param event WPS event type
 */
static void aicwf_app_wlan_wps_status_callback(int event)
{
    struct plat_wps_dev *wpsdev = &wps_dev;
    DBG_APP_INF("%s:ev:%d\n", __func__, event);
    switch (event)
    {
    case WPS_EV_PBC_OVERLAP:
        wpsdev->wps_ret = WPS_REG_OVERLAP;
        break;
    case WPS_EV_PBC_TIMEOUT:
        wpsdev->wps_ret = WPS_REG_FAIL;
        aicwf_app_wlan_wps_notify(3);
        break;
    case WPS_EV_PBC_ACTIVE:
        wpsdev->wps_ret = WPS_REG_ONGOING;
        aicwf_app_wlan_wps_notify(1);
        break;
    case WPS_EV_PBC_DISABLE:
        break;
    case WPS_EV_PWD_AUTH_FAIL:
    case WPS_EV_AP_PIN_TIMEOUT:
        wpsdev->wps_ret = WPS_REG_FAIL;
        aicwf_app_wlan_wps_notify(3);
        break;
    case WPS_EV_AP_PIN_SUCCESS:
        wpsdev->wps_ret = WPS_REG_SUCC;
        aicwf_app_wlan_wps_notify(2);
        break;
    case WPS_EV_SUCCESS:
        wpsdev->wps_ret = WPS_REG_SUCC;
        aicwf_app_wlan_wps_notify(2);
        break;
    case WPS_EV_SELECTED_TIMEOUT:
        wpsdev->wps_ret = WPS_REG_FAIL;
        aicwf_app_wlan_wps_notify(3);
        break;
    default:
        break;
    }
}

/**
 * @brief Start WPS provisioning via the PBC button
 */
void app_wlan_wps_use_pbc(void)
{
    struct plat_wps_dev *wpsdev = &wps_dev;
    /* [BUGFIX] (common: APSTA concurrency) SoftAP runs on vif2 (non-zero), so use the actual AP vif idx */
    int ap_vif = aic_wifi_get_ap_vif_idx();
    if (ap_vif < 0)
        ap_vif = 0;

    wpsdev->wps_ret = WPS_REG_INVALIDE;

    if (app_wlan_status() != 0)
        return;

    wpsdev->wps_ret = WPS_REG_ONGOING;
    DBG_APP_INF("[AIC] WPS Use PBC...\r\n");
    aicwf_app_wlan_set_wps_method(1);

    aic_set_wps_status_cb(aicwf_app_wlan_wps_status_callback);
    aic_ap_wps_pbc(ap_vif);
    return;
}

/**
 * @brief Start the WPS provisioning flow
 */
void app_wlan_wps_launch(void)
{
    struct plat_wps_dev *wpsdev = &wps_dev;

    wpsdev->wps_ret = WPS_REG_INVALIDE;

    if (app_wlan_status() != 0)
        return;

    DBG_APP_INF("[AIC] WPS Launch...\r\n");
    wpsdev->wps_ret = WPS_REG_SUCC;
    return;
}

/**
 * @brief Disable the WPS function
 */
void app_wlan_wps_disable(void)
{
    int ret = -1;
    struct plat_wps_dev *wpsdev = &wps_dev;
    /* [BUGFIX] (common: APSTA concurrency) SoftAP runs on vif2 (non-zero), so use the actual AP vif idx */
    int ap_vif = aic_wifi_get_ap_vif_idx();
    if (ap_vif < 0)
        ap_vif = 0;

    wpsdev->wps_ret = WPS_REG_INVALIDE;

    if (app_wlan_status() != 0)
        return;
    DBG_APP_INF("[AIC] WPS Disable...\r\n");

    ret = aic_ap_wps_disable(ap_vif);

    if (ret == 0)
        wpsdev->wps_ret = WPS_REG_SUCC;
    else
        wpsdev->wps_ret = WPS_REG_FAIL;

    ret = aic_ap_wps_cacel(ap_vif);
    if (ret == 0)
        wpsdev->wps_ret = WPS_REG_SUCC;
    else
    {
        wpsdev->wps_ret = WPS_REG_FAIL;
    }
    return;
}

/**
 * @brief Enable the WPS function
 */
void app_wlan_wps_enable(void)
{
    struct plat_wps_dev *wpsdev = &wps_dev;

    DBG_APP_INF("[AIC] WPS Enable...\r\n");
    wpsdev->wps_ret = WPS_REG_INVALIDE;

    if (app_wlan_status() != 0)
        return;

    wpsdev->wps_ret = WPS_REG_SUCC;
    return;
}

/**
 * @brief Cancel the current WPS operation
 */
void app_wlan_wps_use_cancel(void)
{
    int ret = -1;
    struct plat_wps_dev *wpsdev = &wps_dev;
    /* [BUGFIX] (common: APSTA concurrency) SoftAP runs on vif2 (non-zero), so use the actual AP vif idx */
    int ap_vif = aic_wifi_get_ap_vif_idx();
    if (ap_vif < 0)
        ap_vif = 0;

    wpsdev->wps_ret = WPS_REG_ONGOING;

    DBG_APP_INF("[AIC] WPS Cancel...\r\n");

    wpsdev->wps_ret = WPS_REG_INVALIDE;

    if (app_wlan_status() != 0)
        return;

    ret = aic_ap_wps_cacel(ap_vif);

    if (ret == 0)
        wpsdev->wps_ret = WPS_REG_SUCC;
    else
        wpsdev->wps_ret = WPS_REG_FAIL;
    return;
}

/**
 * @brief Start WPS provisioning via PIN code
 * @param pin PIN code string (8 digits)
 */
void app_wlan_wps_use_pin(char *pin)
{
    int ret = -1;
    struct plat_wps_dev *wpsdev = &wps_dev;
    /* [BUGFIX] (common: APSTA concurrency) SoftAP runs on vif2 (non-zero), so use the actual AP vif idx */
    int ap_vif = aic_wifi_get_ap_vif_idx();
    if (ap_vif < 0)
        ap_vif = 0;

    DBG_APP_INF("[AIC] WPS Use PIN %s ...\r\n", pin);
    wpsdev->wps_ret = WPS_REG_INVALIDE;

    if (app_wlan_status() != 0)
        return;

    if (pin == NULL)
        return;

    aicwf_app_wlan_set_wps_method(0);
    aic_set_wps_status_cb(aicwf_app_wlan_wps_status_callback);
    ret = aic_ap_wps_pin(ap_vif, pin);

    if (ret != 0)
    {
        wpsdev->wps_ret = WPS_REG_FAIL;
        return;
    }

    wpsdev->wps_ret = WPS_REG_ONGOING;
    return;
}

/**
 * @brief Validate the WPS PIN checksum
 *
 * Validates the PIN code using the standard WPS PIN checksum algorithm.
 *
 * @param PIN 8-digit PIN code (unsigned long format)
 * @return 1=checksum valid, 0=checksum invalid
 */
int app_wlan_validate_checksum(unsigned long int PIN)
{
    unsigned int accum = 0;
    accum += 3 * ((PIN / 10000000) % 10);
    accum += 1 * ((PIN / 1000000) % 10);
    accum += 3 * ((PIN / 100000) % 10);
    accum += 1 * ((PIN / 10000) % 10);
    accum += 3 * ((PIN / 1000) % 10);
    accum += 1 * ((PIN / 100) % 10);
    accum += 3 * ((PIN / 10) % 10);
    accum += 1 * ((PIN / 1) % 10);
    return (0 == (accum % 10));
}
