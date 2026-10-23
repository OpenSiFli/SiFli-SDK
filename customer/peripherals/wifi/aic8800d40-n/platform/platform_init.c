/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "bf0_hal.h"
#include "plat_port.h"
#include "rtos_port.h"

/* Power-on timing parameters, kept consistent with platform/hal/plat_port.c (ref aic8800d40 aic8800_ele) */
#define AIC_WIFI_PWR_OFF_MS     (10)
#define AIC_WIFI_PWR_ON_MS      (200)

/**
 * @brief WiFi chip power-on (board level, timing per the aic8800d40 driver's aic8800_ele)
 *
 * Drive the power pin low -> delay -> drive it high -> delay for the chip ROM boot.
 *
 * @note Init level depends on the power control method:
 *   - Direct host GPIO: registered at INIT_BOARD_EXPORT (level 1).
 *   - I2C IO expander (AIC_WIRELESS_PWR_VIA_IOEXP): registered at
 *     INIT_PREV_EXPORT (level 2): later than I2C bus registration (level 1),
 *     earlier than SDIO host registration (level 3, rt_hw_sdio_init in
 *     drv_sdio.c), and the scheduler is already running at that point
 *     (aw9523_init contains rt_thread_mdelay).
 *
 * @note Do not use INIT_ENV_EXPORT: in this SDK's rtdef.h the levels are
 *       BOARD=1, PREV=2, DEVICE=3, COMPONENT=4, ENV=5, PRE_APP=6, APP=7,
 *       so ENV comes after DEVICE. Using ENV makes power-on happen after SDIO
 *       enumeration (the log shows "wifi pwr on IO expander" printed after
 *       "detect SDIO begin", and enumeration times out with no response).
 *
 * @note When the pin number is not configured (negative) or the expander init
 *       fails, this function does nothing.
 *
 * @note On the direct path the scheduler is not running yet, so blocking delays
 *       such as rt_thread_mdelay cannot be used; use HAL_Delay busy-wait
 *       uniformly (same as aic8800_ele in aic8800d40).
 */
static int aicwf_wifi_power_on(void)
{
    aicwf_platform_pwr_wifi_pin_init();
    aicwf_platform_pwr_wifi_pin_disable();
    HAL_Delay(AIC_WIFI_PWR_OFF_MS);
    aicwf_platform_pwr_wifi_pin_enable();
    HAL_Delay(AIC_WIFI_PWR_ON_MS);

    return 0;
}
#ifdef AIC_WIRELESS_PWR_VIA_IOEXP
    /* PWRON sits on the I2C IO expander, so use INIT_PREV_EXPORT (level 2):
    *  - later than I2C bus registration (rt_hw_i2c_init in drv_i2c.c is INIT_BOARD_EXPORT, level 1);
    *  - aw9523_init() contains rt_thread_mdelay(5) and needs the scheduler running,
    *    i.e. it can only execute in the rt_components_init() stage (level >= 2);
    *  - earlier than SDIO host registration (rt_hw_sdio_init in drv_sdio.c is
    *    INIT_DEVICE_EXPORT, level 3), ensuring the module is powered before card enum.
    * Note: do not use INIT_ENV_EXPORT, which is level 5 in this SDK and comes after DEVICE(3). */
    INIT_PREV_EXPORT(aicwf_wifi_power_on);
#else
    INIT_BOARD_EXPORT(aicwf_wifi_power_on);
#endif
