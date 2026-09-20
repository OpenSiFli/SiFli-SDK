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
#include <rtthread.h>
#include <rtdevice.h>
#include "bf0_hal.h"
#include "drv_gpio.h"
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif
#include "sdio_func.h"
#include "aicwf_sdio.h"
#include "osal_debug.h"
#include "wifi_cli.h"
#include "wifi_if.h"
#include "netal_porting.h"
struct rt_mmcsd_card *g_wifi_if_sdio = NULL;
struct sdio_func g_wifi_if_sdio_func1;
#define WIFI_POWER_PIN  85   // B18
#define WIFI_WAKEUP_PIN 69         // A2
//#define WIFI_USE_WIFI_WAKEUP_HCPU_PIN 1
//#define WIFI_WAKEUP_HCPU_PIN 51
static uint8_t wifi_power_flag = 0;
static uint8_t wifi_sleep_flag = 0; // enters sleep mode at boot
#ifdef WIFI_USE_WIFI_WAKEUP_HCPU_PIN
//static struct rt_work net_work;
//
//static void wifi_wakeup_host_work(struct rt_work *work, void *work_data)
//{
//    wifi_if_sdio_resume();
//}
static void wifi_wakeup_hcpu_irq_pin_handle(void)
{
    rt_kprintf("%s\n", __func__);
//    rt_work_init(&net_work, wifi_wakeup_host_work, (void *)0);
//    rt_work_submit(&(net_work.work), 5);
}
static void wifi_wakeup_hcpu_irq_pin_init(void)
{
#if defined(BSP_USING_PM) && defined(WIFI_WAKEUP_HCPU_PIN)
    rt_pin_mode(WIFI_WAKEUP_HCPU_PIN, PIN_MODE_INPUT);
    GPIO_TypeDef *gpio = GET_GPIO_INSTANCE(WIFI_WAKEUP_HCPU_PIN);
    uint16_t gpio_pin = GET_GPIOx_PIN(WIFI_WAKEUP_HCPU_PIN);
    int8_t wakeup_pin;
    if (WIFI_WAKEUP_HCPU_PIN > 96)
        wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    else
        wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);
    RT_ASSERT(wakeup_pin >= 0);
    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_NEG_EDGE);
#endif/* BSP_USING_PM */
#ifdef WIFI_WAKEUP_HCPU_PIN

    rt_pin_mode(WIFI_WAKEUP_HCPU_PIN, PIN_MODE_INPUT);
    // enable LSM int
    rt_pin_attach_irq(WIFI_WAKEUP_HCPU_PIN, PIN_IRQ_MODE_FALLING, (void *) wifi_wakeup_hcpu_irq_pin_handle,
                      (void *)(rt_uint32_t) WIFI_WAKEUP_HCPU_PIN);
    rt_pin_irq_enable(WIFI_WAKEUP_HCPU_PIN, 1);
#endif
}
static void wifi_wakeup_hcpu_irq_deinit(void)
{
#if defined(BSP_USING_PM) && defined(WIFI_WAKEUP_HCPU_PIN)
    GPIO_TypeDef *gpio = GET_GPIO_INSTANCE(WIFI_WAKEUP_HCPU_PIN);
    int8_t gpio_pin = GET_GPIOx_PIN(WIFI_WAKEUP_HCPU_PIN);
    int8_t wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    RT_ASSERT(wakeup_pin >= 0);
    pm_disable_pin_wakeup(wakeup_pin);
#endif /* BSP_USING_PM */
#ifdef WIFI_WAKEUP_HCPU_PIN
    rt_pin_irq_enable(WIFI_WAKEUP_HCPU_PIN, 0);
    rt_pin_detach_irq(WIFI_WAKEUP_HCPU_PIN);
    rt_pin_mode(WIFI_WAKEUP_HCPU_PIN, PIN_MODE_INPUT);
#endif
}
#endif
void wifi_if_power_on(void)
{
#ifdef WIFI_USE_WIFI_WAKEUP_HCPU_PIN
    wifi_wakeup_hcpu_irq_pin_init();
#endif
#ifdef WIFI_POWER_PIN
    rt_pin_mode(WIFI_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(WIFI_POWER_PIN, PIN_LOW);
    rt_thread_mdelay(20);
    rt_pin_write(WIFI_POWER_PIN, PIN_HIGH);
    rt_thread_mdelay(20);
#endif
}
void wifi_if_power_off(void)
{
#ifdef WIFI_USE_WIFI_WAKEUP_HCPU_PIN
    wifi_wakeup_hcpu_irq_deinit();
#endif
#ifdef WIFI_POWER_PIN
    rt_pin_mode(WIFI_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(WIFI_POWER_PIN, PIN_LOW);
#endif
}
void wifi_if_wakeup(void)
{
#ifdef WIFI_WAKEUP_PIN
    rt_pin_mode(WIFI_WAKEUP_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(WIFI_WAKEUP_PIN, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(WIFI_WAKEUP_PIN, PIN_HIGH);
    rt_thread_mdelay(10);
    rt_pin_write(WIFI_WAKEUP_PIN, PIN_LOW);
#endif
}
static rt_int32_t wifi_if_sdio_probe(struct rt_mmcsd_card *card)
{
    rt_int32_t err;
    rt_bool_t was_ready = (RT_NULL != g_wifi_if_sdio);

    DBG_APP_INF("%s\n", __func__);
    g_wifi_if_sdio = card;
    g_wifi_if_sdio_func1.num = g_wifi_if_sdio->sdio_function[1]->num;
    err = aicwf_sdio_probe(&g_wifi_if_sdio_func1);
    if (0 != err)
    {
        /* aicwf_sdio_probe() also reports an error when the driver is already
           probed, which happens when the driver is registered again while the
           card is still enumerated. Only roll back a fresh failure, otherwise
           the working device would be reported as missing. */
        if (was_ready)
        {
            return 0;
        }
        DBG_APP_ERR("%s: aicwf_sdio_probe failed: %d\n", __func__, err);
        g_wifi_if_sdio = RT_NULL;
        return err;
    }
    return 0;
}
static rt_int32_t wifi_if_sdio_remove(struct rt_mmcsd_card *card)
{
    aicwf_sdio_remove(&g_wifi_if_sdio_func1);
    g_wifi_if_sdio = NULL;
    return 0;
}
static struct rt_sdio_device_id wifi_if_sdio_id = {0, 0xc8a1, 0};
static struct rt_sdio_driver wifi_if_sdio =
{
    "aicwfsdio",
    wifi_if_sdio_probe,
    wifi_if_sdio_remove,
    &wifi_if_sdio_id,
};
#ifdef BSP_USING_SD_LINE
__weak int rt_hw_sdio_deinit(void)
{
    return 0;
}
#endif
#ifdef BSP_USING_SDHCI
__weak int rt_hw_sdmmc_deinit(uint8_t id)
{
    return 0;
}
#endif
void sdio_init(void)
{
#ifdef BSP_USING_SDHCI1
    int rt_hw_sdmmc_num_init(uint8_t sdhci_num);
    rt_hw_sdmmc_num_init(0);
#elif BSP_USING_SDHCI2
    int rt_hw_sdmmc_num_init(uint8_t sdhci_num);
    rt_hw_sdmmc_num_init(1);
#elif BSP_USING_SD_LINE
    extern int rt_hw_sdio_init(void);
    rt_hw_sdio_init();
#endif
}

void sdio_deinit(void)
{
#ifdef BSP_USING_SDHCI1
    rt_hw_sdmmc_deinit(0);
#elif BSP_USING_SDHCI2
    rt_hw_sdmmc_deinit(1);
#elif BSP_USING_SD_LINE
    rt_hw_sdio_deinit();
#endif
}
/* Re-post a single SDIO card detect request. mmcsd_detect() only probes one
   round per mailbox message and never retries on failure, so this can restart
   detection when a probe fails.
   It only re-posts the detect message without re-initializing the controller,
   so kernel objects such as "sdhci" are not initialized twice. */
#if defined(BSP_USING_SDHCI1) || defined(BSP_USING_SDHCI2)
extern struct rt_mmcsd_host *sdhci_get_emmchost(void);

static int sdio_detect_again(void)
{
    extern void mmcsd_change(struct rt_mmcsd_host * host);
    struct rt_mmcsd_host *host = sdhci_get_emmchost();

    if (RT_NULL == host)
    {
        return -1;  /* no usable host yet, retrying is pointless */
    }
    mmcsd_change(host);
    return 0;
}
#else
/* SD_LINE has no host re-post interface; keep the callers buildable. */
static int sdio_detect_again(void)
{
    return -1;
}
#endif

/* SDIO is driven by the SDHCI controller selected in the board config:
   BSP_USING_SDHCI1 maps to sdhci_ctx[0], BSP_USING_SDHCI2 to sdhci_ctx[1]. */
#if defined(BSP_USING_SDHCI1) || defined(BSP_USING_SDHCI2)
    #if defined(BSP_USING_SDHCI1)
        #define WIFI_SDHCI_HOST_IDX    (0)
    #else
        #define WIFI_SDHCI_HOST_IDX    (1)
    #endif
    extern int sdhci_reinit_host(uint8_t host_index);
#endif

/* Number of re-enumerations allowed while the card stays attached but the
   driver probe keeps failing; bounded so the timer thread does not re-init the
   controller on every poll. */
static int wifi_detect_reinit_cnt = 0;
#define WIFI_DETECT_REINIT_MAX  3

/* Standalone card re-detection interface:
   On cold boot (card not found yet) it just re-posts one detect request.
   After OTA the old card is still attached to host->card (mmcsd_detect does
   not handle SDIO card removal), so the wifi driver (rwnx/netdev) must be
   released first and the same host re-enumerated. A full sdio_deinit() +
   sdio_init() must be avoided (it re-initializes event/mutex and asserts). */
int wifi_if_sdio_detect(void)
{
#if !defined(BSP_USING_SDHCI1) && !defined(BSP_USING_SDHCI2)
    return -1;
#else
    if (RT_NULL == g_wifi_if_sdio)
    {
        struct rt_mmcsd_host *host = sdhci_get_emmchost();

        /* A card object still attached to the host means enumeration succeeded
           but the driver probe failed; mmcsd_detect() skips such a host, so
           re-enumerate instead of re-posting the detect request. */
        if (RT_NULL != host && RT_NULL != host->card)
        {
            if (++wifi_detect_reinit_cnt > WIFI_DETECT_REINIT_MAX)
            {
                return 0;   /* leave it to the polling timeout */
            }
            return sdhci_reinit_host(WIFI_SDHCI_HOST_IDX);
        }
        return sdio_detect_again();
    }

    wifi_if_sdio_remove(g_wifi_if_sdio);           /* release the driver, keep the driver registered */
    return sdhci_reinit_host(WIFI_SDHCI_HOST_IDX); /* detach old card + reset + re-enumerate */
#endif
}

/* Timer-based card re-detection polling after OTA:
   The module reports nothing after the last OTA message, so the host can only
   observe the new card when it re-enumerates. The module reboots ~2s after the
   last message (HOST_OTA_REBOOT's reboot_timer), then its bootloader has to
   erase, LZMA-decompress and CRC-check the image before the new firmware starts;
   that total time is not bounded by any protocol, so the polling window has to
   be generous: 3s settle + 30 x 1s poll (~33s). */
#define WIFI_DETECT_SETTLE_MS   3000
#define WIFI_DETECT_POLL_MS     1000
#define WIFI_DETECT_POLL_MAX    30

static rt_timer_t wifi_detect_timer = RT_NULL;
static rt_tick_t  wifi_detect_start_tick = 0;
static int        wifi_detect_settled = 0;
static int        wifi_detect_poll_cnt = 0;
static int        wifi_detect_fail_cnt = 0;
static int        wifi_detect_active = 0;
static int        wifi_detect_resume_pending = 0;

static void wifi_detect_timer_cb(void *param)
{
    (void)param;

    if (!wifi_detect_settled)
    {
        /* Phase 0: wait for the module to reboot and the old card to disappear. The old card is still present during this window, so re-enumerating immediately would wrongly detach the old card. */
        if (rt_tick_get() - wifi_detect_start_tick < rt_tick_from_millisecond(WIFI_DETECT_SETTLE_MS))
            return;

        if (wifi_if_sdio_detect() < 0)
        {
            DBG_APP_ERR("wifi re-detect request failed\n");
            /* Stay in phase 0 so the next tick retries; give up only after a
               few consecutive failures (e.g. the host controller is gone). */
            if (++wifi_detect_fail_cnt >= 3)
            {
                DBG_APP_ERR("wifi re-detect abort\n");
                rt_timer_stop(wifi_detect_timer);
                wifi_detect_active = 0;
                wifi_detect_fail_cnt = 0;
            }
            return;
        }
        wifi_detect_fail_cnt = 0;
        wifi_detect_settled = 1;
        wifi_detect_poll_cnt = 0;
        return;
    }

    /* Phase 1: periodic polling. Detection is asynchronous (detect thread), so judge the previous round one beat later. */
    if (RT_NULL != g_wifi_if_sdio)
    {
        DBG_APP_INF("wifi re-detect ok, poll=%d\n", wifi_detect_poll_cnt);
        rt_timer_stop(wifi_detect_timer);
        wifi_detect_active = 0;
        wifi_detect_settled = 0;
        wifi_detect_poll_cnt = 0;
        return;
    }

    if (++wifi_detect_poll_cnt > WIFI_DETECT_POLL_MAX)
    {
        DBG_APP_ERR("wifi re-detect timeout after %d poll\n", wifi_detect_poll_cnt);
        rt_timer_stop(wifi_detect_timer);
        wifi_detect_active = 0;
        wifi_detect_settled = 0;
        wifi_detect_poll_cnt = 0;
        return;
    }

    wifi_if_sdio_detect();
}

void wifi_if_sdio_detect_poll_start(void)
{
    if (RT_NULL == wifi_detect_timer)
    {
        wifi_detect_timer = rt_timer_create("wifi_det",
                                            wifi_detect_timer_cb, RT_NULL,
                                            rt_tick_from_millisecond(WIFI_DETECT_POLL_MS),
                                            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    }

    wifi_detect_start_tick = rt_tick_get();
    wifi_detect_settled = 0;
    wifi_detect_poll_cnt = 0;
    wifi_detect_fail_cnt = 0;
    wifi_detect_reinit_cnt = 0;
    wifi_detect_active = 1;
    if (wifi_detect_timer)
    {
        rt_timer_start(wifi_detect_timer);
    }
}

/* Stop the post-OTA polling before the SDIO host is torn down or reopened,
   otherwise the timer callback would keep using a host that sdio_deinit()
   has already freed. */
static void wifi_if_sdio_detect_poll_stop(void)
{
    if (wifi_detect_timer)
    {
        rt_timer_stop(wifi_detect_timer);
    }
    wifi_detect_active = 0;
    wifi_detect_settled = 0;
    wifi_detect_poll_cnt = 0;
    wifi_detect_fail_cnt = 0;
}

__weak void tcpip_suspend(void)
{

}
__weak void tcpip_resume(void)
{

}
int wifi_if_sdio_suspend(void)
{
    DBG_APP_INF("%s\n", __func__);
    wifi_detect_resume_pending = wifi_detect_active;
    wifi_if_sdio_detect_poll_stop();
    sdio_unregister_driver(&wifi_if_sdio);
    sdio_deinit();
    rt_thread_mdelay(10);
    tcpip_suspend();
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif
    return 0;
}
int wifi_if_sdio_resume(void)
{
    DBG_APP_INF("%s\n", __func__);
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif
    wifi_if_wakeup();
    rt_thread_mdelay(300);
    sdio_register_driver(&wifi_if_sdio);
    sdio_init();
    tcpip_resume();
    if (wifi_detect_resume_pending && RT_NULL == g_wifi_if_sdio)
    {
        /* Resume may run while the module is still rebooting after OTA;
           keep the re-detection polling alive. */
        wifi_if_sdio_detect_poll_start();
    }
    wifi_detect_resume_pending = 0;
    return 0;
}
void wifi_open(void)
{
    DBG_APP_INF("%s\n", __func__);
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif
    wifi_if_power_on();
    rt_thread_mdelay(800);
    sdio_register_driver(&wifi_if_sdio);
    sdio_init();
    /* The module-side SDIO device usually becomes ready later than the host's
       first detection (especially on first boot after OTA: the bootloader must
       erase, LZMA-decompress, CRC-verify and rewrite the image before starting
       the app). mmcsd_detect() only probes one round per mailbox message and
       never retries, so a miss would fail permanently and require a host reset.
       Re-post the detect request on failure to turn "one shot" into "wait
       until found". */
    {
        const int max_retry = 10;
        int retry, wait;

        for (retry = 0; (RT_NULL == g_wifi_if_sdio) && (retry < max_retry); retry++)
        {
            if (sdio_detect_again())
            {
                break;  /* no usable mmcsd host, retrying is pointless */
            }
            /* wait for this round of detection to complete (incl. SDIO driver probe), each round is a few tens of ms */
            for (wait = 0; (RT_NULL == g_wifi_if_sdio) && (wait < 6); wait++)
            {
                rt_thread_mdelay(50);
            }
        }
        if (RT_NULL == g_wifi_if_sdio)
        {
            DBG_APP_ERR("%s: sdio detect failed after %d retry\n", __func__, retry);
        }
        else
        {
            DBG_APP_INF("%s: sdio detect ok, retry = %d\n", __func__, retry);
        }
    }
    DBG_APP_INF("%s %d\n", __func__, __LINE__);
}
void wifi_close(void)
{
    DBG_APP_INF("%s\n", __func__);
    wifi_if_sdio_detect_poll_stop();
    sdio_unregister_driver(&wifi_if_sdio);
    sdio_deinit();
    rt_thread_mdelay(10);
    wifi_if_power_off();
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif
    netif_lwip_reset();
}
int wifi_if_sdio_init(void)
{
    DBG_APP_INF("%s\n", __func__);
    int wifi_power_flag = 1;
    if (wifi_power_flag)
    {
        wifi_open();
    }
    else
    {
        wifi_if_power_off();
    }
    return 0;
}
//INIT_ENV_EXPORT(wifi_if_sdio_init);
void wifi_if_sdio_exit(void)
{
    DBG_APP_INF("%s\n", __func__);
    wifi_if_sdio_detect_poll_stop();
    sdio_unregister_driver(&wifi_if_sdio);
}
int wifi_if_cli_start(void)
{
#ifdef BSP_USING_SDHCI1
    HAL_RCC_EnableModule(RCC_MOD_SDMMC1);
#elif BSP_USING_SDHCI2
    HAL_RCC_EnableModule(RCC_MOD_SDMMC2);
#endif
    return wifi_cli_init();
}
INIT_APP_EXPORT(wifi_if_cli_start);
#ifdef FINSH_USING_MSH
#include <finsh.h>
rt_err_t wifi_test(int argc, char **argv)
{
    int ret;
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif
    rt_pin_mode(18, PIN_MODE_OUTPUT);
    rt_pin_write(18, PIN_HIGH);

    ret = wifi_cli_handler(argc, argv);
    rt_pin_write(18, PIN_LOW);
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif
    return ret;
}
MSH_CMD_EXPORT(wifi_test, wifi_test);
//void stats_display(void);
rt_err_t lwip_stat(int argc, char **argv)
{
    //stats_display();
    return 0;
}
MSH_CMD_EXPORT(lwip_stat, lwip stats);
uint8_t test_wifi = 0;
rt_err_t wifi_stat(int argc, char **argv)
{
    test_wifi = 1;
    wifi_if_sdio_init();
    return 0;
}
MSH_CMD_EXPORT(wifi_stat, wifi stats);
rt_err_t wifi_pin(int argc, char **argv)
{
#ifdef WIFI_POWER_PIN
    rt_pin_mode(WIFI_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(WIFI_POWER_PIN, PIN_LOW);
    rt_thread_mdelay(2000);
    rt_pin_write(WIFI_POWER_PIN, PIN_HIGH);
    rt_thread_mdelay(20);
#endif
    return 0;
}
MSH_CMD_EXPORT(wifi_pin, wifi pin);
#endif
