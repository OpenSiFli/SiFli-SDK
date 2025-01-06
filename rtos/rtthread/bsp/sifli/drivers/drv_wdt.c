/**
  ******************************************************************************
  * @file   drv_wdt.c
  * @author Sifli software development team
  * @brief Watchdog BSP driver
  * @{
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <board.h>
#include <string.h>
#include "drv_wdt.h"
#include "module_record.h"
/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_wdt Watchdog
  * @brief Watchdog BSP driver
  * @{
  */

//#define DRV_DEBUG
#define LOG_TAG             "drv.wdt"
#include <drv_log.h>

#if defined(SOC_SF32LB52X)
    #define WDT_CLOCK_FREQ wdt_get_backup_clk_freq()
#else
    #if defined(LXT_DISABLE)||defined(SOC_SF32LB55X)
        #define WDT_CLOCK_FREQ HAL_LPTIM_GetFreq()
    #elif defined(FPGA)
        #define WDT_CLOCK_FREQ 12000  // For FPGA, RC10K is actually 12K
    #else
        #define WDT_CLOCK_FREQ 10000  // For ASIC, RC10K might vary
    #endif
#endif

static WDT_HandleTypeDef hwdt, hiwdt;
static struct rt_watchdog_ops ops;
static rt_watchdog_t watchdog;
static rt_err_t wdt_set_timeout(WDT_HandleTypeDef *wdt, rt_uint32_t reload_timeout);

RT_WEAK void wdt_store_exception_information(void)
{
    return;
}

void wdt_reconfig(void)
{
    hwdt.Instance = hwp_wdt2;
    HAL_WDT_Refresh(&hwdt);
    __HAL_WDT_STOP(&hwdt);
    hwdt.Instance = hwp_wdt1;
    HAL_WDT_Refresh(&hwdt);
    __HAL_WDT_STOP(&hwdt);
#ifdef SOC_BF0_HCPU
    hiwdt.Instance = hwp_iwdt;
    HAL_WDT_Refresh(&hiwdt);
    wdt_set_timeout(&hiwdt, WDT_TIMEOUT);
#endif
}

#ifdef SOC_SF32LB55X
void WDT_IRQHandler(void)
{
    static int printed;
    if (printed == 0)
    {
        printed++;
        sifli_record_crash_status(1);
        sifli_record_wdt_irq_status(1);
        sifli_record_crash_save_process(RECORD_CRASH_SAVE_WDT_START);
        wdt_reconfig();
        wdt_store_exception_information();
    }
}
#else
void WDT_IRQHandler(void)
{
    static int printed;
    if (printed == 0)
    {
        printed++;

#ifdef SOC_BF0_LCPU
        // Avoid recurisive
        if (__HAL_SYSCFG_Get_Trigger_Assert_Flag() == 0)
        {
            wdt_store_exception_information();
            HAL_LPAON_WakeCore(CORE_ID_HCPU);
            __HAL_SYSCFG_Trigger_Assert();
            hwdt.Instance = hwp_wdt2;
            __HAL_WDT_STOP(&hwdt);
            __HAL_SYSCFG_Enable_WDT_REBOOT(1);              // When timeout, reboot whole system instead of subsys.
        }
#else // HCPU
        sifli_record_crash_status(1);
        sifli_record_wdt_irq_status(1);
        sifli_record_crash_save_process(RECORD_CRASH_SAVE_WDT_START);
        if (__HAL_SYSCFG_Get_Trigger_Assert_Flag() == 0)
        {
            HAL_HPAON_WakeCore(CORE_ID_LCPU);
            __HAL_SYSCFG_Trigger_Assert();
        }
        wdt_reconfig();
        wdt_store_exception_information();
#endif
    }
}
#endif



#ifdef SOC_SF32LB52X
#define HAL_Get_backup(idx) wdt_get_backup_status()
#define wdt_set_iwdt_timeout(iwdt,timeout)

#ifdef SOC_BF0_HCPU
#define HAL_Set_backup(type, status) wdt_set_backup_status(status)

uint32_t watchdog_status = 1;
static void wdt_set_backup_status(uint32_t status)
{
    watchdog_status = status;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_STATUS, &status, 4);
}

static uint32_t wdt_get_backup_status(void)
{
    return watchdog_status;
}

static uint16_t wdt_get_backup_clk_freq(void)
{
    return LXT_FREQ;
}

#else

#undef WDT_TIMEOUT
#undef BSP_WDT_TIMEOUT
#define WDT_TIMEOUT wdt_get_backup_time()
#define BSP_WDT_TIMEOUT wdt_get_backup_time()
#define HAL_Set_backup(type, status)

static uint32_t wdt_get_backup_status(void)
{
    uint32_t status;
    uint16_t len = 4;
    HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_WDT_STATUS, (uint8_t *)&status, &len);
    return status;
}

static uint16_t wdt_get_backup_clk_freq(void)
{
    uint16_t freq;
    uint16_t len = 2;
    HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_WDT_CLK_FEQ, (uint8_t *)&freq, &len);
    return freq;
}

#endif

static uint32_t wdt_get_backup_time(void)
{
    uint32_t time;
    uint16_t len = 4;

    HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_WDT_TIME, (uint8_t *)&time, &len);
    // Not set 0 as wdt time
    if (time == 0)
        time = 30;
    return time;

}

#else

#ifdef SOC_BF0_HCPU
#define wdt_set_iwdt_timeout(iwdt,timeout) \
{\
    iwdt.Instance = hwp_iwdt;\
    wdt_set_timeout(&iwdt,timeout); \
}
#else
#define wdt_set_iwdt_timeout(iwdt,timeout)
#endif

#endif //SOC_SF32LB52X


static rt_err_t wdt_init(rt_watchdog_t *wdt)
{
#ifdef SOC_BF0_HCPU
    hwdt.Instance = hwp_wdt1;
#else
    hwdt.Instance = hwp_wdt2;
#endif
    if (BSP_WDT_TIMEOUT  >= 2) // Minimal timeout is 2 seconds
        hwdt.Init.Reload = hwdt.Init.Reload * (BSP_WDT_TIMEOUT  - 1);
    else
        hwdt.Init.Reload = WDT_CLOCK_FREQ;

    hwdt.Init.Reload2 = WDT_CLOCK_FREQ * WDT_REBOOT_TIMEOUT;
    __HAL_WDT_INT(&hwdt, 1);


#ifdef SOC_BF0_HCPU
    hiwdt.Instance = hwp_iwdt;
    hiwdt.Init.Reload = WDT_CLOCK_FREQ * WDT_TIMEOUT + IWDT_RELOAD_DIFFTIME;
    hiwdt.Init.Reload2 = WDT_CLOCK_FREQ * IWDT_RELOAD_DIFFTIME;
    __HAL_WDT_INT(&hiwdt, 1);
#endif

    return RT_EOK;
}


static rt_err_t wdt_set_timeout(WDT_HandleTypeDef *wdt, rt_uint32_t reload_timeout)
{
    wdt->Init.Reload = WDT_CLOCK_FREQ * reload_timeout;

    if (hwp_iwdt == wdt->Instance)
        wdt->Init.Reload2 = WDT_CLOCK_FREQ * IWDT_RELOAD_DIFFTIME;
    else
        wdt->Init.Reload2 = WDT_CLOCK_FREQ * WDT_REBOOT_TIMEOUT;

    //Disable by status
    if (HAL_Get_backup(RTC_BAKCUP_WDT_STATUS) != 0x01)
        return -RT_ERROR;
    if (HAL_WDT_Init(wdt) != HAL_OK)//wdt init
    {
        LOG_E("wdg set wdt timeout failed.");
        return -RT_ERROR;
    }
#if !defined(SOC_SF32LB52X)&&!defined(SOC_SF32LB55X)    // 52x should not acces PMU for PMU in HCPU
    HAL_PMU_SetWdt((uint32_t)wdt->Instance);            // Add reboot cause for watchdog
#endif
    __HAL_SYSCFG_Enable_WDT_REBOOT(1);                  // When timeout, reboot whole system instead of subsys.
    return RT_EOK;
}


static rt_err_t wdt_control(rt_watchdog_t *wdt, int cmd, void *arg)
{
    switch (cmd)
    {
    /* feed the watchdog */
    case RT_DEVICE_CTRL_WDT_KEEPALIVE:
    {
        if (HAL_Get_backup(RTC_BAKCUP_WDT_STATUS) != 0x01)
            break;
        HAL_WDT_Refresh(&hwdt);
#ifdef SOC_BF0_HCPU
        HAL_WDT_Refresh(&hiwdt);
#endif
    }
    break;
    /* set watchdog timeout */
    case RT_DEVICE_CTRL_WDT_SET_TIMEOUT:
        wdt_set_timeout(&hwdt, *((rt_uint32_t *)arg));
#ifdef SOC_BF0_HCPU
        wdt_set_timeout(&hiwdt, (*((rt_uint32_t *)arg)) + IWDT_RELOAD_DIFFTIME);
#endif
        break;
    case RT_DEVICE_CTRL_WDT_START:
        __HAL_WDT_START(&hwdt);
#ifdef SOC_BF0_HCPU
        __HAL_WDT_START(&hiwdt);
#endif

        break;

#if defined(RT_USING_PM)
    case RT_DEVICE_CTRL_SUSPEND:
    {
        uint32_t status = HAL_Get_backup(RTC_BAKCUP_WDT_STATUS);
        wdt_set_iwdt_timeout(hiwdt, IWDT_SLEEP_TIMEOUT);
    }
    break;

    case RT_DEVICE_CTRL_RESUME:
    {
        uint32_t status = HAL_Get_backup(RTC_BAKCUP_WDT_STATUS);
        if (0x01 == status)
        {
            __HAL_WDT_STOP(&hwdt);
            hwdt.Instance = hwp_wdt1;
            wdt_set_timeout(&hwdt, WDT_TIMEOUT);

            wdt_set_iwdt_timeout(hiwdt, WDT_TIMEOUT + IWDT_RELOAD_DIFFTIME);
        }
    }
    break;
#endif

    case RT_DEVICE_CTRL_WDT_STOP:
    {
#if defined(SOC_SF32LB52X)
        HAL_Set_backup(RTC_BAKCUP_WDT_STATUS, 0XFF);
#else
        HAL_Set_backup(RTC_BAKCUP_WDT_STATUS, 0);
#endif
        __HAL_WDT_STOP(&hwdt);
#ifdef SOC_BF0_HCPU
        __HAL_WDT_STOP(&hiwdt);
#endif
    }
    break;

    case RT_DEVICE_CTRL_WDT_SET_STATUS:
    {
        uint32_t status = *((uint32_t *)arg);

#if defined(BSP_USING_WDT2_SWITCH)
        WDT_HandleTypeDef hwdt2 = {0};
#endif

        if (!status)
        {
#ifdef SOC_BF0_HCPU
            __HAL_WDT_STOP(&hiwdt);
#endif
            __HAL_WDT_STOP(&hwdt);
#if defined(BSP_USING_WDT2_SWITCH)
            status = 0xFF;
            hwdt2 = hwdt;
            hwdt2.Instance = hwp_wdt2;
            HAL_HPAON_WakeCore(CORE_ID_LCPU);
            __HAL_WDT_STOP(&hwdt2);
            HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
        }
        else
        {
            __HAL_WDT_STOP(&hwdt);
            HAL_WDT_Init(&hwdt);
#if defined(SOC_BF0_HCPU)
#if defined(BSP_USING_WDT2_SWITCH)
            hwdt2 = hwdt;
            hwdt2.Instance = hwp_wdt2;
            HAL_HPAON_WakeCore(CORE_ID_LCPU);
            __HAL_WDT_STOP(&hwdt2);
            wdt_set_timeout(&hwdt2, WDT_TIMEOUT);
            HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
            hiwdt.Instance = hwp_iwdt;
            wdt_set_timeout(&hiwdt, WDT_TIMEOUT + IWDT_RELOAD_DIFFTIME);
#endif
            __HAL_SYSCFG_Enable_WDT_REBOOT(1);
        }
        HAL_Set_backup(RTC_BAKCUP_WDT_STATUS, status);
    }
    break;

    case RT_DEVICE_CTRL_WDT_GET_STATUS:
    {
        uint32_t *status = (uint32_t *)arg;
        *status = HAL_Get_backup(RTC_BAKCUP_WDT_STATUS);
#if defined(SOC_SF32LB52X) && defined(SOC_BF0_HCPU)
        if (0xFF == *status)
        {
            *status = 0;
        }
#endif

    }
    break;
    default:
        return -RT_ERROR;
    }
    return RT_EOK;
}

__ROM_USED void rt_wdt_restore(void)
{
#ifdef SOC_BF0_LCPU
    //WDT_HandleTypeDef wdt_handle;
    uint32_t wdt_status = HAL_Get_backup(RTC_BAKCUP_WDT_STATUS);
    if (0x01 == wdt_status)
    {
        HAL_WDT_Init(&hwdt);
        __HAL_SYSCFG_Enable_WDT_REBOOT(1);              // When timeout, reboot whole system instead of subsys.
    }
#endif  /* SOC_BF0_LCPU */
}

__ROM_USED int rt_wdt_init(void)
{
    ops.init = &wdt_init;
    ops.control = &wdt_control;
    watchdog.ops = &ops;

    /* register watchdog device */
    if (rt_hw_watchdog_register(&watchdog, "wdt", RT_DEVICE_FLAG_DEACTIVATE, RT_NULL) != RT_EOK)
    {
        LOG_E("wdt device register failed.");
        return -RT_ERROR;
    }
    LOG_D("wdt device register success.");
    return RT_EOK;
}

__ROM_USED int cmd_wdt(int argc, char *argv[])
{
    if (strcmp(argv[1], "stop") == 0)
    {
        __HAL_WDT_STOP(&hwdt);
#ifdef SOC_BF0_HCPU
        __HAL_WDT_STOP(&hiwdt);
#endif
    }
    else if (strcmp(argv[1], "start") == 0)
    {
        wdt_init(&watchdog);
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_wdt, __cmd_wdt, Watch dog command);

/// @} drv_wdt
/// @} bsp_driver
/// @} file

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
