/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/pin.h>
#include <aic_log.h>

int wifi_pwrkey_pin = 0;
int wifi_rstkey_pin = 0;
int bt_rstkey_pin = 0;
#ifndef AIC_WIRELESS_PWR_GPIO
#define AIC_WIRELESS_PWR_GPIO (26)
#endif

static void platform_pwr_wifi_pin_set(int on)
{
    if (wifi_pwrkey_pin > 0)
    {
        rt_pin_write(wifi_pwrkey_pin, on);
    }
}

void platform_pwr_wifi_pin_init(void)
{
    wifi_pwrkey_pin = AIC_WIRELESS_PWR_GPIO;
    if (wifi_pwrkey_pin > 0)
    {
        rt_pin_mode(wifi_pwrkey_pin, PIN_MODE_OUTPUT);
    }
}

void platform_pwr_wifi_pin_enable(void)
{
    platform_pwr_wifi_pin_set(1);
}

void platform_pwr_wifi_pin_disable(void)
{
    platform_pwr_wifi_pin_set(0);
}

static void platform_rst_wifi_pin_set(int on)
{
    if (wifi_rstkey_pin > 0)
    {
        rt_pin_write(wifi_rstkey_pin, on);
    }
}

void platform_rst_wifi_pin_init(void)
{
    wifi_rstkey_pin = AIC_DEV_AIC8800_WLAN0_RST_GPIO;
    if (wifi_rstkey_pin > 0)
    {
        rt_pin_mode(wifi_rstkey_pin, PIN_MODE_OUTPUT);
    }
}

void platform_rst_wifi_pin_enable(void)
{
    platform_rst_wifi_pin_set(0);
}

void platform_rst_wifi_pin_disable(void)
{
    platform_rst_wifi_pin_set(1);
}

static void platform_rst_bt_pin_set(int on)
{
    if (bt_rstkey_pin > 0)
    {
        rt_pin_write(bt_rstkey_pin, on);
    }
}

void platform_rst_bt_pin_init(void)
{
    bt_rstkey_pin = AIC_DEV_AIC8800_BT_RST_GPIO;
    if (bt_rstkey_pin > 0)
    {
        rt_pin_mode(bt_rstkey_pin, PIN_MODE_OUTPUT);
    }
}

void platform_rst_bt_pin_enable(void)
{
    platform_rst_bt_pin_set(1);
}

void platform_rst_bt_pin_disable(void)
{
    platform_rst_bt_pin_set(0);
}

