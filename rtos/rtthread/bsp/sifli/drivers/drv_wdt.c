/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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

// Shall remove in future
#if 0
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
#endif

static WDT_HandleTypeDef hwdt, hiwdt;
static struct rt_watchdog_ops ops;
static rt_watchdog_t watchdog;
static rt_err_t wdt_set_timeout(WDT_HandleTypeDef *wdt, rt_uint32_t reload_timeout);
static uint32_t wdt_get_backup_time(void);

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
    wdt_set_timeout(&hiwdt, wdt_get_backup_time());
#endif
}

#ifdef SOC_SF32LB55X
void WDT_IRQHandler(void)
{
    static int printed;
    if (printed == 0)
    {
        printed++;
#ifdef SOC_BF0_HCPU
        sifli_record_crash_status(1);
        sifli_record_wdt_irq_status(1);
        sifli_record_crash_save_process(RECORD_CRASH_SAVE_WDT_START);
        wdt_reconfig();
        wdt_store_exception_information();
#endif
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

#if defined(SOC_BF0_HCPU) && defined(SOC_SF32LB52X)
    uint32_t watchdog_status = 1;
#endif // defined(SOC_BF0_HCPU) && defined(SOC_SF32LB52X)

static void wdt_set_backup_status(uint32_t status)
{
#ifdef SOC_BF0_HCPU

#ifdef SOC_SF32LB52X
    watchdog_status = status;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_STATUS, &status, 4);
#else // !SOC_SF32LB52X
    HAL_Set_backup(RTC_BAKCUP_WDT_STATUS, status);
#endif // SOC_SF32LB52X

#else // !SOC_BF0_HCPU

#ifdef SOC_SF32LB52X
    // Do nothing
#else // !SOC_SF32LB52X
    HAL_Set_backup(RTC_BAKCUP_WDT_STATUS, status);
#endif // SOC_SF32LB52X

#endif //SOC_BF0_HCPU
}

static uint32_t wdt_get_backup_status(void)
{
    uint32_t status = 0;
#ifdef SOC_BF0_HCPU

#ifdef SOC_SF32LB52X
    status = watchdog_status;
#else // !SOC_SF32LB52X
    status = HAL_Get_backup(RTC_BAKCUP_WDT_STATUS);
#endif // SOC_SF32LB52X

#else // !SOC_BF0_HCPU

#ifdef SOC_SF32LB52X
    uint16_t len = 4;
    HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_WDT_STATUS, (uint8_t *)&status, &len);
#else // !SOC_SF32LB52X
    status = HAL_Get_backup(RTC_BAKCUP_WDT_STATUS);
#endif // SOC_SF32LB52X

#endif // SOC_BF0_HCPU
    return status;
}

static uint16_t wdt_get_backup_clk_freq(void)
{
    uint16_t freq = LXT_FREQ;

#ifdef SOC_SF32LB52X

#ifdef SOC_BF0_HCPU
    // Do nothing
#else // !SOC_BF0_HCPU
    uint16_t len = 2;
    HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_WDT_CLK_FEQ, (uint8_t *)&freq, &len);
#endif // SOC_BF0_HCPU

#else // !SOC_SF32LB52X

#if defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)
    // 56x or 58x always use RC10K as WDT clock source
    uint32_t is_xtal_enb = 0;
#else // !defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)
    uint32_t is_xtal_enb = HAL_LXT_ENABLED();
#endif // defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)
    if (!is_xtal_enb)
    {
        uint32_t cycle = HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
        if (cycle == 0)
            freq = 10000; // FPGA also tread 10K to adapt with ROM only versions
        else
            freq = (uint16_t)(48000000UL / (float)cycle * HAL_RC_CAL_GetLPCycle());
    }
#endif // SOC_SF32LB52X

    return freq;
}

static uint32_t wdt_get_backup_time(void)
{
    uint32_t time = WDT_TIMEOUT;
#if defined(LCPU_CONFIG_V2) || defined(SOC_SF32LB52X)
    uint16_t len = 4;
    HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_WDT_TIME, (uint8_t *)&time, &len);
    // Not set 0 as wdt time
    if (time == 0)
        time = 30;
#endif // defined(LCPU_CONFIG_V2) || defined(SOC_SF32LB52X)
    return time;
}

void wdt_set_iwdt_timeout(WDT_HandleTypeDef *iwdt, uint32_t timeout)
{
#if defined(SOC_BF0_HCPU) && !defined(SOC_SF32LB52X)
    iwdt->Instance = hwp_iwdt;
    wdt_set_timeout(iwdt, timeout);
#endif // defined(SOC_BF0_HCPU) && !defined(SOC_SF32LB52X)
}

// Shall remove in future
#if 0
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
#endif

static rt_err_t wdt_init(rt_watchdog_t *wdt)
{
#ifdef SOC_BF0_HCPU
    hwdt.Instance = hwp_wdt1;
#else
    hwdt.Instance = hwp_wdt2;
#endif
    if (wdt_get_backup_time()  >= 2) // Minimal timeout is 2 seconds
        hwdt.Init.Reload = hwdt.Init.Reload * (wdt_get_backup_time()  - 1);
    else
        hwdt.Init.Reload = wdt_get_backup_clk_freq();

    hwdt.Init.Reload2 = wdt_get_backup_clk_freq() * WDT_REBOOT_TIMEOUT;
    __HAL_WDT_INT(&hwdt, 1);

#ifdef SOC_BF0_HCPU
    hiwdt.Instance = hwp_iwdt;
    hiwdt.Init.Reload = wdt_get_backup_clk_freq() * wdt_get_backup_time() + IWDT_RELOAD_DIFFTIME;
    hiwdt.Init.Reload2 = wdt_get_backup_clk_freq() * IWDT_RELOAD_DIFFTIME;
    __HAL_WDT_INT(&hiwdt, 1);
#endif

    return RT_EOK;
}

static rt_err_t wdt_set_timeout(WDT_HandleTypeDef *wdt, rt_uint32_t reload_timeout)
{
    wdt->Init.Reload = wdt_get_backup_clk_freq() * reload_timeout;

    if (hwp_iwdt == wdt->Instance)
        wdt->Init.Reload2 = wdt_get_backup_clk_freq() * IWDT_RELOAD_DIFFTIME;
    else
        wdt->Init.Reload2 = wdt_get_backup_clk_freq() * WDT_REBOOT_TIMEOUT;

    //Disable by status
    if (wdt_get_backup_status() != 0x01)
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
        if (wdt_get_backup_status() != 0x01)
            break;
        HAL_WDT_Refresh(&hwdt);
#ifdef SOC_BF0_HCPU
        HAL_WDT_Refresh(&hiwdt);
#endif
    }
    break;
    /* set watchdog timeout */
    case RT_DEVICE_CTRL_WDT_SET_TIMEOUT:
#if defined(ROM_CONFIG_V2) && defined(SOC_BF0_LCPU)
        wdt_set_timeout(&hwdt, wdt_get_backup_time());
#else // !ROM_CONFIG_V2
        wdt_set_timeout(&hwdt, *((rt_uint32_t *)arg));
#endif // ROM_CONFIG_V2
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
#ifdef SOC_BF0_HCPU
        uint32_t status = wdt_get_backup_status();
        wdt_set_iwdt_timeout(&hiwdt, IWDT_SLEEP_TIMEOUT);
#endif
    }
    break;

    case RT_DEVICE_CTRL_RESUME:
    {
#ifdef SOC_BF0_HCPU
        uint32_t status = wdt_get_backup_status();
        if (0x01 == status)
        {
            __HAL_WDT_STOP(&hwdt);
            hwdt.Instance = hwp_wdt1;
            wdt_set_timeout(&hwdt, wdt_get_backup_time());
            wdt_set_iwdt_timeout(&hiwdt, wdt_get_backup_time() + IWDT_RELOAD_DIFFTIME);
        }
#endif
    }
    break;
#endif

    case RT_DEVICE_CTRL_WDT_STOP:
    {
#if defined(SOC_SF32LB52X)
        wdt_set_backup_status(0XFF);
#else
        wdt_set_backup_status(0);
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
            wdt_set_timeout(&hwdt2, wdt_get_backup_time());
            HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
            hiwdt.Instance = hwp_iwdt;
            wdt_set_timeout(&hiwdt, wdt_get_backup_time() + IWDT_RELOAD_DIFFTIME);
#endif
            __HAL_SYSCFG_Enable_WDT_REBOOT(1);
        }
        wdt_set_backup_status(status);
    }
    break;

    case RT_DEVICE_CTRL_WDT_GET_STATUS:
    {
        uint32_t *status = (uint32_t *)arg;
        *status = wdt_get_backup_status();
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
    uint32_t wdt_status = wdt_get_backup_status();
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

