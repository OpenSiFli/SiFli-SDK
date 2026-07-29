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
static uint8_t wifi_sleep_flag = 0; //开机会进入休眠
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
    rt_uint32_t result = 0;
    uint8_t in_data;
    DBG_APP_INF("%s\n", __func__);
    g_wifi_if_sdio = card;
    g_wifi_if_sdio_func1.num = g_wifi_if_sdio->sdio_function[1]->num;
    aicwf_sdio_probe(&g_wifi_if_sdio_func1);
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
__weak void tcpip_suspend(void)
{

}
__weak void tcpip_resume(void)
{

}
int wifi_if_sdio_suspend(void)
{
    DBG_APP_INF("%s\n", __func__);
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
    DBG_APP_INF("%s %d\n", __func__, __LINE__);
}
void wifi_close(void)
{
    DBG_APP_INF("%s\n", __func__);
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
