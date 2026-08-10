/*
 * Copyright (c) 2023, ArtInChip Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/pin.h>
#include <drivers/sdio.h>
#include <drivers/mmcsd_card.h>
#include <rtdevice.h>
#include <rtthread.h>

#include "sdio_func.h"
#include "sdio_port.h"
#include "aic_plat_log.h"
#include "rtos_port.h"
#include "plat_port.h"
#include "wifi_port.h"

struct rt_mmcsd_card *g_wifi_if_sdio = NULL;
struct sdio_func g_wifi_if_sdio_funcs[SDIOM_MAX_FUNCS];

int aic8800_reset(void)
{
    return 0;
}

int aic8800_power_on(void)
{
    return 0;
}

int aic8800_power_off(void)
{
    return 0;
}

static rt_int32_t wifi_if_sdio_probe(struct rt_mmcsd_card *card)
{
    int idx;
    AIC_LOG_PRINTF("%s: card_type=%d, host_flags=0x%x\n", __func__, card->card_type, card->host->flags);
    for (idx = 0; idx < SDIOM_MAX_FUNCS; idx++) {
        g_wifi_if_sdio_funcs[idx].num    = card->sdio_function[idx]->num;
        g_wifi_if_sdio_funcs[idx].vendor = card->sdio_function[idx]->manufacturer;
        g_wifi_if_sdio_funcs[idx].device = card->sdio_function[idx]->product;
        g_wifi_if_sdio_funcs[idx].drv_priv = card;
    }

    aicwf_sdio_probe(&g_wifi_if_sdio_funcs[0]);

    /*
     * Announce the card only after the whole init chain (firmware download,
     * fhost init, wlan device registration) has finished. The auto-probe
     * fallback in wifi_if_sdio_init() polls g_wifi_if_sdio; setting it at
     * probe start let the WiFi thread proceed while the probe chain was still
     * running (in the mmcsd thread), racing the chip communication and ending
     * in a cmd_mgr timeout during scan.
     */
    g_wifi_if_sdio = card;
}

static rt_int32_t wifi_if_sdio_remove(struct rt_mmcsd_card *card)
{
    AIC_LOG_PRINTF("%s: card_type=%d, host_flags=0x%x\n", __func__, card->card_type, card->host->flags);
    aicwf_sdio_remove(&g_wifi_if_sdio_funcs[0]);
    return 0;
}

#if defined(CONFIG_AIC8800D80_SUPPORT)
struct rt_sdio_device_id wifi_if_sdio_ids_d80[]= {
    { 0, 0xc8a1, 0x0182},
};
#endif

#if defined(CONFIG_AIC8800DC_SUPPORT) || defined(CONFIG_AIC8800DW_SUPPORT)
struct rt_sdio_device_id wifi_if_sdio_ids_dcdw[]= {
    { 0, 0xc8a1, 0xc08d},
};
#endif

struct rt_sdio_driver wifi_if_sdio_drv = {
    "aicwfsdio",
    wifi_if_sdio_probe,
    wifi_if_sdio_remove,
    NULL,
};

#if 1
int wifi_if_sdio_init(void)
{
    int chip_id = CONFIG_CHIPID_SELECT;
    if (0) {}
    #if defined(CONFIG_AIC8800D80_SUPPORT)
    else if (chip_id == PRODUCT_ID_AIC8800D80) {
        wifi_if_sdio_drv.id = wifi_if_sdio_ids_d80;
    }
    #endif
    #if defined(CONFIG_AIC8800DC_SUPPORT) || defined(CONFIG_AIC8800DW_SUPPORT)
    else if ((chip_id == PRODUCT_ID_AIC8800DC) || (chip_id == PRODUCT_ID_AIC8800DW)) {
        wifi_if_sdio_drv.id = wifi_if_sdio_ids_dcdw;
    }
    #endif
    else {
        AIC_LOG_PRINTF("invalid chip_id = %d\n", chip_id);
        return -1;
    }
    AIC_LOG_PRINTF("sdio id = 0x%x\n", wifi_if_sdio_drv.id->product);
    int ret = sdio_register_driver(&wifi_if_sdio_drv);
    if (ret < 0) {
        /*
         * SDMMC2 card detection runs asynchronously in the mmcsd thread and
         * may not have finished yet, so sdio_register_driver() cannot match
         * the card and returns -RT_EEMPTY (-4). The driver is already queued
         * in the sdio_drivers list, so once the card is detected
         * sdio_register_card() will auto-probe it. Wait for that auto-probe
         * to complete instead of failing immediately.
         */
        AIC_LOG_PRINTF("rt sdio register driver failed:%d, wait for card auto-probe...\n", ret);
        int cnt = 0;
        while ((g_wifi_if_sdio == NULL) && (cnt++ < 200))   /* up to ~20s */
        {
            rtos_task_suspend(100);
        }
        if (g_wifi_if_sdio == NULL) {
            AIC_LOG_PRINTF("wifi sdio card not detected, give up\n");
            return -1;
        }
        ret = 0;
    }

    return ret;
}

#else
int wifi_if_sdio_init_my(void)
{
    platform_pwr_wifi_pin_init();
    platform_pwr_wifi_pin_disable();
    rtos_task_suspend(10);
    platform_pwr_wifi_pin_enable();

    AIC_LOG_PRINTF("sdio id = 0x%x\n", wifi_if_sdio_drv.id->product);
    sdio_register_driver(&wifi_if_sdio_drv);

    return 0;
}
INIT_COMPONENT_EXPORT(wifi_if_sdio_init_my);
#endif

int wifi_if_sdio_deinit(void)
{
    int ret = sdio_unregister_driver(&wifi_if_sdio_drv);
    AIC_LOG_PRINTF("deinit sdio id = 0x%x\n", wifi_if_sdio_drv.id->product);
    if (ret < 0) {
        AIC_LOG_PRINTF("rt sdio unregister driver failed:%d\n", ret);
        return ret;
    }
    return 0;
}