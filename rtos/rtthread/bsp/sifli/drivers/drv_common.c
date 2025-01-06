/**
  ******************************************************************************
  * @file   drv_common.c
  * @author Sifli software development team
  * @brief Common functions for BSP driver
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

#include "rtthread.h"
#include "string.h"
#include "drv_common.h"
#include "semihosting.h"
#include "drv_log.h"
#include "drv_gpio.h"
#include "bf0_hal.h"

#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif /* BSP_USING_PM */

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_common Common functions
  * @brief Common functions for BSP driver
  * @{
  */

#ifdef RT_USING_SERIAL
    #include "drv_usart.h"
#endif

#ifdef RWBT_ENABLE
    #include "dbg_trc.h"
#endif /* RWBT_ENABLE */

#ifdef BSP_USING_HWMAILBOX
    #include "drv_mailbox.h"
#endif




#if defined(BSP_USING_PSRAM) && defined(RT_USING_MEMHEAP_AS_HEAP)
    static struct rt_memheap _psram_heap;
#endif

#ifdef SF32LB52X
    /* SYSTICK support high precision fixed clock source, such as RC48/XT48*/
    #define SYSTICK_HIGH_PRICISION_FIXED_CLK_SUPPORT
#endif /* SF32LB52X */

#if defined(BSP_PM_FREQ_SCALING) && !defined(SYSTICK_HIGH_PRICISION_FIXED_CLK_SUPPORT)
    #ifdef SOC_BF0_HCPU
        #define HAL_DELAY_US_TIMER  (hwp_btim1)
        #define HAL_DELAY_US_TIMER_NAME "btim1"
    #else
        #define HAL_DELAY_US_TIMER  (hwp_btim3)
        #define HAL_DELAY_US_TIMER_NAME "btim3"
    #endif /* SOC_BF0_HCPU */
    static rt_device_t hal_delay_us_timer;
#endif /* BSP_PM_FREQ_SCALING && !SYSTICK_HIGH_PRICISION_FIXED_CLK_SUPPORT */


#ifdef RT_USING_BT
    #include "drv_bt.h"
#endif

#ifdef SF32LB55X

    #ifndef BLE_TX_POWER_VAL
        #if defined(BLE_POWER_CLASS3)
            #define BLE_TX_POWER_VAL 0
        #elif defined(BLE_POWER_CLASS2)
            #define BLE_TX_POWER_VAL 4
        #elif defined(BLE_POWER_CLASS15)
            #define BLE_TX_POWER_VAL 10
        #else
            #define BLE_TX_POWER_VAL 0
        #endif // defined(BLE_POWER_CLASS3)
    #endif // BLE_TX_POWER_VAL

#else // !SF32LB55X

    #ifndef BT_TX_POWER_VAL_MAX
        #define BT_TX_POWER_VAL_MAX 10
    #endif
    #ifndef BT_TX_POWER_VAL_MIN
        #define BT_TX_POWER_VAL_MIN 0
    #endif
    #ifndef BT_TX_POWER_VAL_INIT
        #define BT_TX_POWER_VAL_INIT 0
    #endif

#endif // SF32LB55X


#ifdef RT_HAL_DEBUG
void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[RT_CONSOLEBUF_SIZE];
    extern void rt_kputs(const char *str);

    va_start(args, fmt);
    rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    rt_kputs(rt_log_buf);
    va_end(args);
}
#endif

#if defined(RT_USING_FINSH) || defined(_SIFLI_DOXYGEN_)
#include <finsh.h>
/**
* @brief  reboot commands.
* This function provide 'reboot' command to shell(FINSH) . It will reboot current core.
*/
__ROM_USED void reboot(uint8_t argc, char **argv)
{
    drv_reboot();
}
FINSH_FUNCTION_EXPORT_ALIAS(reboot, __cmd_reboot, Reboot System);
#endif /* RT_USING_FINSH */

/**
* @brief  hardware systick initialization.
*/
__ROM_USED void rt_hw_systick_init(void)
{

#if defined(SYSTICK_HIGH_PRICISION_FIXED_CLK_SUPPORT) && defined(SOC_BF0_HCPU)

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_TICK, RCC_CLK_TICK_HRC48);
    /* workaround: add delay to avoid systick config failure in some chips due to known reason */
    HAL_Delay_us(200);
    MODIFY_REG(hwp_hpsys_rcc->CFGR, HPSYS_RCC_CFGR_TICKDIV_Msk,
               MAKE_REG_VAL(60, HPSYS_RCC_CFGR_TICKDIV_Msk, HPSYS_RCC_CFGR_TICKDIV_Pos));
    HAL_SYSTICK_Config(800000 / RT_TICK_PER_SECOND);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

#else

#if defined(BSP_PM_FREQ_SCALING)
    /* TODO: half of LP clock */
    if (HAL_LXT_DISABLED())
    {
        HAL_SYSTICK_Config(HAL_LPTIM_GetFreq() / 2 / RT_TICK_PER_SECOND);
    }
    else
    {
        HAL_SYSTICK_Config(32768 / 2 / RT_TICK_PER_SECOND);
    }
    //TODO: config clock source for 52x
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
#else
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq(CORE_ID_DEFAULT) / RT_TICK_PER_SECOND);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
#endif /* BSP_PM_FREQ_SCALING */

#endif /* SF32LB52X */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

__ROM_USED int rt_in_system_heap(void *ptr)
{
    if (ptr >= HEAP_BEGIN && (uint32_t)ptr < HEAP_END)
        return 1;
    else
        return 0;
}

/**
 * @brief This is the systick timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    uint32_t status;
    uint32_t pin_wsr;
#ifdef BSP_USING_PM
    rt_tick_t old_tick;
    rt_tick_t new_tick;
#endif /* BSP_USING_PM */

    /* enter interrupt */
    rt_interrupt_enter();

    /* Trigger GPIO callback manually as GPIO edge detection interrupt may get lost
       and WSR.PIN status is not cleared */
#ifdef SOC_BF0_HCPU
    status = HAL_HPAON_GET_WSR() & HPSYS_AON_WSR_PIN_ALL;
    if (status)
    {
        pin_wsr = status >> HPSYS_AON_WSR_PIN0_Pos;
#ifdef RT_USING_PIN
        drv_pin_irq_from_wsr(pin_wsr);
#endif /* RT_USING_PIN */
    }
#else
    status = HAL_LPAON_GET_WSR() & LPSYS_AON_WSR_PIN_ALL;
    if (status)
    {
        pin_wsr = status >> LPSYS_AON_WSR_PIN0_Pos;
#ifdef RT_USING_PIN
        drv_pin_irq_from_wsr(pin_wsr);
#endif /* RT_USING_PIN */
    }
#endif



#ifdef RT_USING_PIN
    drv_pin_check();
#endif /* RT_USING_PIN */

    /* hal tick is not used, don't compensate */
    HAL_IncTick();
#if defined(BSP_USING_PM)
    old_tick = rt_tick_get();

#ifdef SOC_BF0_HCPU
    if (HAL_HPAON_IS_LP_ACTIVE() && HAL_HPAON_IS_HP2LP_REQ_ACTIVE())
#else
#ifdef SF32LB52X
//TODO: LCPU cannot access PMU when HCPU is in sleep
    if (HAL_LPAON_IS_HP_ACTIVE())
#else
    if (true)
#endif /* SF32LB52X */
#endif /* SOC_BF0_HCPU */
    {
        new_tick = pm_latch_tick(old_tick + 1, HAL_GTIMER_READ(), HAL_LPTIM_GetFreq(), (void *)1);
    }
    else
    {
        new_tick = old_tick + 1;
    }

    if (new_tick != old_tick)
    {
        if (new_tick == (old_tick + 1))
        {
            rt_tick_increase();
        }
        else
        {
            /* avoid using rt_tick defined by ROM, if step is more than 2, just increase by 2,
             * it can happen only when system is halted and continue runnnig.
             */
            rt_tick_increase();
            rt_tick_increase();
            /* update last_latch_tick as rt_tick is not updated as expected */
            pm_set_last_latch_tick(old_tick + 2);
            //rt_tick_increase_by_step(new_tick - old_tick);
        }
    }
#else
    rt_tick_increase();
#endif /* BSP_USING_PM */

    /* leave interrupt */
    rt_interrupt_leave();
}

time_t drv_get_timestamp(void)
{
#ifdef HAL_RTC_MODULE_ENABLED
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    RTC_HandleTypeDef RTC_Handler;
    struct tm tm_new;

    memset(&RTC_Handler, 0, sizeof(RTC_Handler));
    RTC_Handler.Instance = hwp_rtc;
    HAL_RTC_GetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
    while (HAL_RTC_GetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN) == HAL_ERROR)
    {
        HAL_RTC_GetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
    };

    tm_new.tm_sec  = RTC_TimeStruct.Seconds + ((RTC_TimeStruct.SubSeconds > (RC10K_SUB_SEC_DIVB >> 1)) ? 1 : 0);
    tm_new.tm_min  = RTC_TimeStruct.Minutes;
    tm_new.tm_hour = RTC_TimeStruct.Hours;
    tm_new.tm_mday = RTC_DateStruct.Date;
    tm_new.tm_mon  = RTC_DateStruct.Month - 1;
    tm_new.tm_wday  = RTC_DateStruct.WeekDay == RTC_WEEKDAY_SUNDAY ? 0 : RTC_DateStruct.WeekDay;
    if (RTC_DateStruct.Year & RTC_CENTURY_BIT)
        tm_new.tm_year = RTC_DateStruct.Year & (~RTC_CENTURY_BIT);
    else
        tm_new.tm_year = RTC_DateStruct.Year + 100;

    return mktime(&tm_new);
#else
    // Hardware should support RTC for this function.
    RT_ASSERT(0);
    return 0;
#endif
}

/**
 * @brief Get current systick timer count.
 * @retval Current ticks in milliseconds
 */
static uint8_t  use_RC10k = 0xFF;
static uint32_t soft_rc10_backup = 0;
static uint16_t rc10_freq_khz;

void drv_set_soft_rc10_backup(uint32_t backup)
{
    soft_rc10_backup = backup;
    /* RC10K freq_khz = 48000 * LXT_LP_CYCLE / v; */
    rc10_freq_khz = (uint16_t)(48000UL * LXT_LP_CYCLE / backup);
}

uint32_t HAL_GetTick(void)
{
    uint32_t t, t_in_ms;

    if (0xFF == use_RC10k)
    {
        //Read pmu register will cost about 100us
        use_RC10k = !HAL_LXT_ENABLED();
    }

#ifdef SOC_BF0_HCPU
    t = HAL_HPAON_READ_GTIMER();
#else
    t = HAL_LPAON_READ_GTIMER();
#endif

    if (use_RC10k) //Use RC10k
    {
        if (soft_rc10_backup == 0)
        {
            //Use 10k directly, as read RTC register will cost about 100us too.
            t_in_ms = t / 10;
        }
        else
        {
            /*  v  = HAL_Get_backup(RTC_BACKUP_LPCYCLE);
                RC10K freq_khz = 48000 * LXT_LP_CYCLE / v;

                t_in_ms  = t * freq_khz
             */
            t_in_ms = t / rc10_freq_khz;
        }
    }
    else //Use xtal32k
    {
        /*
           t_in_ms = t * 1000 / 32768
                   = CONVERT(t, 1000, 32768)
        */
        t_in_ms = ((t >> 15) * 1000) + (((t & 0x7FFF) * 1000) >> 15);
    }

    return t_in_ms;
}


void HAL_SuspendTick(void)
{
}

void HAL_ResumeTick(void)
{
}

void HAL_Delay(__IO uint32_t Delay)
{
    while (Delay > 0)
    {
        HAL_Delay_us(1000);
        Delay--;
    }
}


/* re-implement tick interface for BF0 HAL */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    /* Return function status */
    return HAL_OK;
}


#ifndef LXT_FREQ
    #define LXT_FREQ 32768
#endif

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif

__HAL_ROM_USED float HAL_LPTIM_GetFreq()
{
    if (HAL_LXT_DISABLED())
    {
        uint32_t cycle = HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
        if (cycle == 0)
            return 9700;
        else
            return (48000000UL / (float)cycle * HAL_RC_CAL_GetLPCycle());
    }
    else
    {
        return LXT_FREQ;
    }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param[in] s: error message
  * @param[in] num: error number
  * @retval None
  */
__ROM_USED void _Error_Handler(char *s, int num)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler */
}

__ROM_USED void rt_hw_console_output(const char *str)
{
}

#ifndef SF32LB55X
int8_t bt_rf_get_max_tx_pwr(void)
{
    return BT_TX_POWER_VAL_MAX;
}

int8_t bt_rf_get_min_tx_pwr(void)
{
    return BT_TX_POWER_VAL_MIN;
}
#endif // !SF32LB55X

int8_t bt_rf_get_init_tx_pwr(void)
{
    int8_t pwr;
#ifdef SF32LB55X
    pwr = BLE_TX_POWER_VAL;
#else
    pwr = BT_TX_POWER_VAL_INIT;
#endif
    return pwr;
}




/**
 * @brief This function will delay for some us.
 *
 * @param us the delay time of us
 */
__ROM_USED void rt_hw_us_delay(rt_uint32_t us)
{
    rt_uint32_t told, tnow, tcnt = 0;

#if defined(BSP_PM_FREQ_SCALING) && !defined(SYSTICK_HIGH_PRICISION_FIXED_CLK_SUPPORT)
    uint32_t max_cnt = HAL_DELAY_US_TIMER->ARR;

    told = HAL_DELAY_US_TIMER->CNT;
    while (1)
    {
        tnow = HAL_DELAY_US_TIMER->CNT;
        if (tnow >= told)
        {
            tcnt = tnow - told;
        }
        else
        {
            tcnt = HAL_DELAY_US_TIMER->ARR - told + tnow;
        }
        if (tcnt >= us)
        {
            break;
        }
    }

#else
    rt_uint32_t ticks;
    rt_uint32_t reload = SysTick->LOAD;

    ticks = us * reload / (1000000 / RT_TICK_PER_SECOND);
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
#endif /* BSP_PM_FREQ_SCALING && !SYSTICK_HIGH_PRICISION_FIXED_CLK_SUPPORT */
}

__ROM_USED void drv_get_hpsys_clk(hpsys_clk_setting_t *clk_setting)
{
    RT_ASSERT(clk_setting);

    clk_setting->sysclk = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
    clk_setting->hclk = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);
    clk_setting->pclk1 = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
    clk_setting->pclk2 = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 0);


}
__ROM_USED void drv_get_lpsys_clk(lpsys_clk_setting_t *clk_setting)
{
    RT_ASSERT(clk_setting);

#ifdef SOC_BF0_HCPU
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif

    clk_setting->sysclk = HAL_RCC_GetSysCLKFreq(CORE_ID_LCPU);
    clk_setting->hclk = HAL_RCC_GetHCLKFreq(CORE_ID_LCPU);
    clk_setting->pclk1 = HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 1);
    clk_setting->pclk2 = HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 0);


#ifdef SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif /* SF32LB52X */
}

__ROM_USED void drv_get_blesys_clk(blesys_clk_setting_t *clk_setting)
{
    RT_ASSERT(clk_setting);
#ifdef SOC_BF_Z0

#ifdef SOC_BF0_HCPU
    HAL_HPAON_WakeCore(CORE_ID_BCPU);
#endif

    clk_setting->sysclk = HAL_RCC_GetSysCLKFreq(CORE_ID_BCPU);
    clk_setting->hclk = HAL_RCC_GetHCLKFreq(CORE_ID_BCPU);
#else
    clk_setting->sysclk = 0;//HAL_RCC_GetSysCLKFreq(CORE_ID_BCPU);
    clk_setting->hclk = 0;//HAL_RCC_GetHCLKFreq(CORE_ID_BCPU);
#endif /* BF_SOC_Z0 */
    //clk_setting->macclk = HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 1);
    //clk_setting->macfreq = HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 0);
}

#ifdef  BSP_USING_FULL_ASSERT
void HAL_AssertFailed(char *file, uint32_t line)
{
    volatile uint32_t dummy = 0;

    rt_kprintf("HAL assertion failed in file:%s, line number:%d \n", file, line);
    RT_ASSERT(0);
    while (0 == dummy);
}
#endif

#ifdef SOC_BF0_LCPU
static void lcpu_thread_init_hook(rt_thread_t thread)
{
#ifdef ARCH_CPU_STACK_GROWS_UPWARD
    thread->sp = (void *)rt_hw_stack_init(thread->entry, thread->parameter,
                                          (void *)((char *)thread->stack_addr),
                                          (void *)rt_thread_exit);
    rt_hw_set_stack_limit(thread->sp, (void *)((rt_uint8_t *)thread->stack_addr + thread->stack_size - 4));

#else
    thread->sp = (void *)rt_hw_stack_init(thread->entry, thread->parameter,
                                          (void *)((char *)thread->stack_addr + thread->stack_size - 4),
                                          (void *)rt_thread_exit);
    rt_hw_set_stack_limit(thread->sp, thread->stack_addr);
#endif

}
#endif /* SOC_BF0_LCPU */

RT_WEAK void rt_hw_preboard_init(void)
{

}

/**
 * This function will initial Sifli01 FPGA board.
 */
RT_WEAK void rt_hw_board_init()
{
    rt_hw_preboard_init();

#ifdef SCB_EnableICache
    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();
#endif

#ifdef SCB_EnableDCache
    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();
#endif

    /* HAL_Init() function is called at the beginning of the program */
    HAL_Init();

#ifdef SOC_BF0_LCPU
    /* use hook to overwrite the init result performed by rt_hw_stack_init in ROM
     *  and set stack limit for each thread
     */
    rt_thread_inited_sethook(lcpu_thread_init_hook);
#endif /* SOC_BF0_LCPU */

    /* System clock initialization */
    SystemClock_Config();
    rt_hw_systick_init();

#ifdef RT_USING_WDT
    /* Start Watchdog at earliest possible */
    {
        extern void rt_hw_watchdog_init(void);
        rt_hw_watchdog_init();
#if defined(SOLUTION_USING_DEVELOPER_MODE)
        extern void service_set_watchdog_enable(uint8_t enable);
        service_set_watchdog_enable(0);
#endif
    }
#endif


    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif


    /* Pin driver initialization is open by default */
#ifdef RT_USING_PIN
    rt_hw_pin_init();
#endif

    /* USART driver initialization is open by default */
#ifdef RT_USING_SERIAL
    rt_hw_usart_init();
#endif

#ifdef PKG_USING_SEGGER_RTT
    {
        extern void SEGGER_RTT_Init(void);
        SEGGER_RTT_Init();
    }
#endif

#ifdef PKG_SEGGER_RTT_AS_SERIAL_DEVICE
    {
        extern int hw_segger_init(void);
        hw_segger_init();
    }
#endif

#ifdef BSP_USING_HWMAILBOX
    rt_hw_mailbox_init();
#endif

    /* Set the shell console output device */
#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif


#ifdef SOC_BF0_HCPU
    rt_kprintf("Serial:%x,Chip:%x,Package:%x,Rev:%x  Reason:%08x\r\n",
               __HAL_SYSCFG_GET_SID(), __HAL_SYSCFG_GET_CID(), __HAL_SYSCFG_GET_PID(), __HAL_SYSCFG_GET_REVID(), HAL_PMU_GET_WSR());

#ifdef RT_USING_PM
    rt_kprintf("Serial PowerOnMOde:%d rtc_record:%08x\n", SystemPowerOnModeGet(), HAL_Get_backup(RTC_BACKUP_MODULE_RECORD));
#endif
#endif

    /* Initial flash before components init to make sure otp prepared */
//#ifdef BSP_USING_SPI_FLASH
//    extern int rt_sys_spi_flash_init(void);
    //rt_sys_spi_flash_init();
//#endif


    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

#ifdef ULOG_OUTPUT_TIME
rt_tick_t ulog_get_tick(void)
{
#ifdef SOC_BF0_HCPU
    return (rt_tick_t)(HAL_HPAON_READ_GTIMER());
#else
    return (rt_tick_t)(HAL_LPAON_READ_GTIMER());
#endif
}

#endif

#ifndef SOC_BF_Z0
rt_tick_t rt_system_get_time(void)
{
#ifdef SOC_BF0_HCPU
    return (rt_tick_t)(HAL_HPAON_READ_GTIMER());
#else
    return (rt_tick_t)(HAL_LPAON_READ_GTIMER());
#endif
}
#endif /* !SOC_BF_Z0 */

void drv_reboot(void)
{
    rt_hw_interrupt_disable();
    HAL_PMU_Reboot();
}

#if defined(BSP_PM_FREQ_SCALING) && !defined(SYSTICK_HIGH_PRICISION_FIXED_CLK_SUPPORT)
void HAL_Delay_us(uint32_t us)
{
#define MAX_US_DELAY    10000
    uint32_t ticks;

    if ((0 == us)
            && !__HAL_GPT_IS_ENABLED(HAL_DELAY_US_TIMER))
    {
        /* sysclk_m needs to be initialized as it's a random value in keil driver */
        HAL_Delay_us_(0);
    }

    while (us > 0)
    {
        if (us > MAX_US_DELAY)
        {
            ticks = MAX_US_DELAY;
            us -= MAX_US_DELAY;
        }
        else
        {
            ticks = us;
            us = 0;
        }
        if (!__HAL_GPT_IS_ENABLED(HAL_DELAY_US_TIMER)) // btim1 not enabled yet, use loop
            HAL_Delay_us_(ticks);
        else
            rt_hw_us_delay(ticks);
    }
}

static int hal_delay_us_timer_init(void)
{
    uint32_t freq = 1000000;
    uint32_t mode = HWTIMER_MODE_PERIOD;
    rt_hwtimerval_t t;
    rt_err_t err;

    hal_delay_us_timer = rt_device_find(HAL_DELAY_US_TIMER_NAME);
    RT_ASSERT(hal_delay_us_timer);

    err = rt_device_open(hal_delay_us_timer, RT_DEVICE_FLAG_RDWR);
    RT_ASSERT(RT_EOK == err);
    err = rt_device_control(hal_delay_us_timer, HWTIMER_CTRL_FREQ_SET, (void *)&freq);
    RT_ASSERT(RT_EOK == err);
    err = rt_device_control(hal_delay_us_timer, HWTIMER_CTRL_MODE_SET, (void *)&mode);
    RT_ASSERT(RT_EOK == err);

    t.sec = 0;
    t.usec = 60000;
    freq = rt_device_write(hal_delay_us_timer, 0, &t, sizeof(t));
    RT_ASSERT(freq == sizeof(t));

    return 0;
}
INIT_COMPONENT_EXPORT(hal_delay_us_timer_init);
#endif /* BSP_PM_FREQ_SCALING && !SYSTICK_HIGH_PRICISION_FIXED_CLK_SUPPORT */

#ifndef BSP_NOT_DISABLE_UNUSED_MODULE

#ifdef SF32LB55X
void HAL_RCC_MspInit(void)
{
#ifdef SOC_BF0_HCPU

#ifndef BSP_USING_UART1
    HAL_RCC_DisableModule(RCC_MOD_USART1);
#endif /* !BSP_USING_UART1 */
#ifndef BSP_USING_UART2
    HAL_RCC_DisableModule(RCC_MOD_USART2);
#endif /* BSP_USING_UART2 */

#ifndef BSP_USING_SPI1
    HAL_RCC_DisableModule(RCC_MOD_SPI1);
#endif /* !BSP_USING_SPI1 */
#ifndef BSP_USING_SPI2
    HAL_RCC_DisableModule(RCC_MOD_SPI2);
#endif /* !BSP_USING_SPI2 */

#ifndef BSP_USING_I2C1
    HAL_RCC_DisableModule(RCC_MOD_I2C1);
#endif /* !BSP_USING_I2C1 */
#ifndef BSP_USING_I2C2
    HAL_RCC_DisableModule(RCC_MOD_I2C2);
#endif /* !BSP_USING_I2C2 */
#ifndef BSP_USING_I2C3
    HAL_RCC_DisableModule(RCC_MOD_I2C3);
#endif /* !BSP_USING_I2C3 */

#if !defined(BSP_USING_GPTIM1) && !defined(BSP_USING_PWM2)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM1);
#endif /* !BSP_USING_GPTIM1 */
#if !defined(BSP_USING_GPTIM2) && !defined(BSP_USING_PWM3)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM2);
#endif /* !BSP_USING_GPTIM2 */

#ifndef BSP_USING_BTIM1
    HAL_RCC_DisableModule(RCC_MOD_BTIM1);
#endif /* !BSP_USING_BTIM1 */
#ifndef BSP_USING_BTIM2
    HAL_RCC_DisableModule(RCC_MOD_BTIM2);
#endif /* !BSP_USING_BTIM2 */

#ifndef BSP_ENABLE_QSPI1
    HAL_RCC_DisableModule(RCC_MOD_QSPI1);
#endif /* !BSP_ENABLE_QSPI1 */
#ifndef BSP_ENABLE_QSPI2
    HAL_RCC_DisableModule(RCC_MOD_QSPI2);
#endif /* !BSP_ENABLE_QSPI2 */
#ifndef BSP_ENABLE_QSPI3
    HAL_RCC_DisableModule(RCC_MOD_QSPI3);
#endif /* !BSP_ENABLE_QSPI3 */

#ifndef BSP_USING_PSRAM0
    HAL_RCC_DisableModule(RCC_MOD_PSRAMC);
#endif /* BSP_USING_PSRAM0 */

#if !defined(BSP_USING_USBD) && !defined(BSP_USING_USBH)
    HAL_RCC_DisableModule(RCC_MOD_USBC);
#endif /* BSP_USING_USBD */

#ifndef BSP_USING_NN_ACC
    HAL_RCC_DisableModule(RCC_MOD_NNACC);
#endif /* !BSP_USING_NN_ACC */

#ifndef BSP_ENABLE_I2S_MIC
    HAL_RCC_DisableModule(RCC_MOD_I2S1);
#endif /* !BSP_ENABLE_I2S_MIC */
#ifndef BSP_ENABLE_I2S_CODEC
    HAL_RCC_DisableModule(RCC_MOD_I2S2);
#endif /* !BSP_ENABLE_I2S_CODEC */

#ifndef BSP_USING_SDHCI1
    HAL_RCC_DisableModule(RCC_MOD_SDMMC1);
#endif /* !BSP_USING_SDHCI1 */
#ifndef BSP_USING_SDHCI2
    HAL_RCC_DisableModule(RCC_MOD_SDMMC2);
#endif /* !BSP_USING_SDHCI2 */

#ifndef BSP_USING_PDM1
    HAL_RCC_DisableModule(RCC_MOD_PDM1);
#endif /* !BSP_USING_PDM1 */
#ifndef BSP_USING_PDM2
    HAL_RCC_DisableModule(RCC_MOD_PDM2);
#endif /* !BSP_USING_PDM2 */

    HAL_RCC_DisableModule(RCC_MOD_DSIHOST);
    HAL_RCC_DisableModule(RCC_MOD_DSIPHY);

#ifndef BSP_USING_BUSMON
    HAL_RCC_DisableModule(RCC_MOD_BUSMON1);
#endif /* BSP_USING_BUSMON */

#else

#ifndef BSP_USING_UART3
    HAL_RCC_DisableModule(RCC_MOD_USART3);
#endif /* !BSP_USING_UART3 */
#ifndef BSP_USING_UART4
    HAL_RCC_DisableModule(RCC_MOD_USART4);
#endif /* BSP_USING_UART4 */
#ifndef BSP_USING_UART5
    HAL_RCC_DisableModule(RCC_MOD_USART5);
#endif /* BSP_USING_UART5 */

#ifndef BSP_USING_SPI3
    HAL_RCC_DisableModule(RCC_MOD_SPI3);
#endif /* !BSP_USING_SPI3 */
#ifndef BSP_USING_SPI4
    HAL_RCC_DisableModule(RCC_MOD_SPI4);
#endif /* !BSP_USING_SPI4 */

#ifndef BSP_USING_I2C4
    HAL_RCC_DisableModule(RCC_MOD_I2C4);
#endif /* !BSP_USING_I2C4 */
#ifndef BSP_USING_I2C5
    HAL_RCC_DisableModule(RCC_MOD_I2C5);
#endif /* !BSP_USING_I2C5 */
#ifndef BSP_USING_I2C6
    HAL_RCC_DisableModule(RCC_MOD_I2C6);
#endif /* !BSP_USING_I2C6 */

#if !defined(BSP_USING_GPTIM3) && !defined(BSP_USING_PWM4)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM3);
#endif /* !BSP_USING_GPTIM3 */
#if !defined(BSP_USING_GPTIM4) && !defined(BSP_USING_PWM5)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM4);
#endif /* !BSP_USING_GPTIM4 */
#if !defined(BSP_USING_GPTIM5) && !defined(BSP_USING_PWM6)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM5);
#endif /* !BSP_USING_GPTIM5 */

#ifndef BSP_USING_BTIM3
    HAL_RCC_DisableModule(RCC_MOD_BTIM3);
#endif /* !BSP_USING_BTIM3 */
#ifndef BSP_USING_BTIM4
    HAL_RCC_DisableModule(RCC_MOD_BTIM4);
#endif /* !BSP_USING_BTIM4 */

#ifndef BSP_ENABLE_QSPI4
    HAL_RCC_DisableModule(RCC_MOD_QSPI4);
#endif /* !BSP_ENABLE_QSPI4 */

#ifndef BSP_USING_BUSMON
    HAL_RCC_DisableModule(RCC_MOD_BUSMON2);
#endif /* BSP_USING_BUSMON */

#endif /* SOC_BF0_HCPU */
}
#endif /* SF32LB55X */

#ifdef SF32LB58X
void HAL_RCC_MspInit(void)
{
#ifdef SOC_BF0_HCPU

#ifndef BSP_USING_UART1
    HAL_RCC_DisableModule(RCC_MOD_USART1);
#endif /* !BSP_USING_UART1 */
#ifndef BSP_USING_UART2
    HAL_RCC_DisableModule(RCC_MOD_USART2);
#endif /* BSP_USING_UART2 */
#ifndef BSP_USING_UART3
    HAL_RCC_DisableModule(RCC_MOD_USART3);
#endif /* BSP_USING_UART3 */


#ifndef BSP_USING_SPI1
    HAL_RCC_DisableModule(RCC_MOD_SPI1);
#endif /* !BSP_USING_SPI1 */
#ifndef BSP_USING_SPI2
    HAL_RCC_DisableModule(RCC_MOD_SPI2);
#endif /* !BSP_USING_SPI2 */

#ifndef BSP_USING_I2C1
    HAL_RCC_DisableModule(RCC_MOD_I2C1);
#endif /* !BSP_USING_I2C1 */
#ifndef BSP_USING_I2C2
    HAL_RCC_DisableModule(RCC_MOD_I2C2);
#endif /* !BSP_USING_I2C2 */
#ifndef BSP_USING_I2C3
    HAL_RCC_DisableModule(RCC_MOD_I2C3);
#endif /* !BSP_USING_I2C3 */
#ifndef BSP_USING_I2C4
    HAL_RCC_DisableModule(RCC_MOD_I2C4);
#endif /* !BSP_USING_I2C4 */

#if !defined(BSP_USING_GPTIM1) && !defined(BSP_USING_PWM2)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM1);
#endif /* !BSP_USING_GPTIM1 */
#if !defined(BSP_USING_GPTIM2) && !defined(BSP_USING_PWM3)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM2);
#endif /* !BSP_USING_GPTIM2 */

#ifndef BSP_USING_BTIM1
    HAL_RCC_DisableModule(RCC_MOD_BTIM1);
#endif /* !BSP_USING_BTIM1 */
#ifndef BSP_USING_BTIM2
    HAL_RCC_DisableModule(RCC_MOD_BTIM2);
#endif /* !BSP_USING_BTIM2 */

#if !defined(BSP_USING_ATIM1) && !defined(BSP_USING_PWMA1)
    HAL_RCC_DisableModule(RCC_MOD_ATIM1);
#endif /* !BSP_USING_ATIM1 */
#if !defined(BSP_USING_ATIM2) && !defined(BSP_USING_PWMA2)
    HAL_RCC_DisableModule(RCC_MOD_ATIM2);
#endif /* !BSP_USING_ATIM2 */

#ifndef BSP_ENABLE_MPI1
    HAL_RCC_DisableModule(RCC_MOD_MPI1);
#endif /* !BSP_ENABLE_MPI1 */
#ifndef BSP_ENABLE_MPI2
    HAL_RCC_DisableModule(RCC_MOD_MPI2);
#endif /* !BSP_ENABLE_MPI2 */
#ifndef BSP_ENABLE_MPI3
    HAL_RCC_DisableModule(RCC_MOD_MPI3);
#endif /* !BSP_ENABLE_MPI3 */
#ifndef BSP_ENABLE_MPI4
    HAL_RCC_DisableModule(RCC_MOD_MPI4);
#endif /* !BSP_ENABLE_MPI4 */

#if !defined(BSP_USING_USBD) && !defined(BSP_USING_USBH)
    HAL_RCC_DisableModule(RCC_MOD_USBC);
#endif /* BSP_USING_USBD */

#ifndef BSP_USING_NN_ACC
    HAL_RCC_DisableModule(RCC_MOD_NNACC1);
#endif /* !BSP_USING_NN_ACC */

#ifndef BSP_ENABLE_I2S_MIC
    HAL_RCC_DisableModule(RCC_MOD_I2S1);
#endif /* !BSP_ENABLE_I2S_MIC */
#ifndef BSP_ENABLE_I2S_CODEC
    HAL_RCC_DisableModule(RCC_MOD_I2S2);
#endif /* !BSP_ENABLE_I2S_CODEC */

#ifndef BSP_USING_SDHCI1
    HAL_RCC_DisableModule(RCC_MOD_SDMMC1);
#endif /* !BSP_USING_SDHCI1 */
#ifndef BSP_USING_SDHCI2
    HAL_RCC_DisableModule(RCC_MOD_SDMMC2);
#endif /* !BSP_USING_SDHCI2 */

#ifndef BSP_USING_PDM1
    HAL_RCC_DisableModule(RCC_MOD_PDM1);
#endif /* !BSP_USING_PDM1 */
#ifndef BSP_USING_PDM2
    HAL_RCC_DisableModule(RCC_MOD_PDM2);
#endif /* !BSP_USING_PDM2 */

    HAL_RCC_DisableModule(RCC_MOD_DSIHOST);
    HAL_RCC_DisableModule(RCC_MOD_DSIPHY);

#ifndef BSP_ENABLE_AUD_PRC
    HAL_RCC_DisableModule(RCC_MOD_AUDPRC);
#endif /* !BSP_ENABLE_AUD_PRC  */

#ifndef BSP_ENABLE_AUD_CODEC
    HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_HP);
#endif /* !BSP_ENABLE_AUD_CODEC */

#ifndef BSP_USING_BUSMON
    HAL_RCC_DisableModule(RCC_MOD_BUSMON1);
    HAL_RCC_DisableModule(RCC_MOD_BUSMON2);
#endif /* BSP_USING_BUSMON */

    HAL_RCC_DisableModule(RCC_MOD_CAN1);
    HAL_RCC_DisableModule(RCC_MOD_CAN2);

#else

#ifndef BSP_USING_UART4
    HAL_RCC_DisableModule(RCC_MOD_USART4);
#endif /* BSP_USING_UART4 */
#ifndef BSP_USING_UART5
    HAL_RCC_DisableModule(RCC_MOD_USART5);
#endif /* BSP_USING_UART5 */
#ifndef BSP_USING_UART6
    HAL_RCC_DisableModule(RCC_MOD_USART6);
#endif /* BSP_USING_UART6 */

#ifndef BSP_USING_SPI3
    HAL_RCC_DisableModule(RCC_MOD_SPI3);
#endif /* !BSP_USING_SPI3 */
#ifndef BSP_USING_SPI4
    HAL_RCC_DisableModule(RCC_MOD_SPI4);
#endif /* !BSP_USING_SPI4 */

#ifndef BSP_USING_I2C5
    HAL_RCC_DisableModule(RCC_MOD_I2C5);
#endif /* !BSP_USING_I2C5 */
#ifndef BSP_USING_I2C6
    HAL_RCC_DisableModule(RCC_MOD_I2C6);
#endif /* !BSP_USING_I2C6 */
#ifndef BSP_USING_I2C7
    HAL_RCC_DisableModule(RCC_MOD_I2C7);
#endif /* !BSP_USING_I2C6 */

#if !defined(BSP_USING_GPTIM3) && !defined(BSP_USING_PWM4)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM3);
#endif /* !BSP_USING_GPTIM3 */
#if !defined(BSP_USING_GPTIM4) && !defined(BSP_USING_PWM5)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM4);
#endif /* !BSP_USING_GPTIM4 */
#if !defined(BSP_USING_GPTIM5) && !defined(BSP_USING_PWM6)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM5);
#endif /* !BSP_USING_GPTIM5 */

#ifndef BSP_USING_BTIM3
    HAL_RCC_DisableModule(RCC_MOD_BTIM3);
#endif /* !BSP_USING_BTIM3 */
#ifndef BSP_USING_BTIM4
    HAL_RCC_DisableModule(RCC_MOD_BTIM4);
#endif /* !BSP_USING_BTIM4 */

#ifndef BSP_ENABLE_MPI5
    //HAL_RCC_DisableModule(RCC_MOD_MPI5);
#endif /* !BSP_ENABLE_MPI5 */

#ifndef BSP_USING_NN_ACC
    HAL_RCC_DisableModule(RCC_MOD_NNACC2);
#endif /* !BSP_USING_NN_ACC */

#ifndef BSP_ENABLE_I2S3
    HAL_RCC_DisableModule(RCC_MOD_I2S3);
#endif /* !BSP_ENABLE_I2S3 */

// only used by HCPU
//#ifndef BSP_ENABLE_AUD_CODEC
//    HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_LP);
//#endif /* !BSP_ENABLE_AUD_CODEC */

#ifndef BSP_USING_BUSMON
    HAL_RCC_DisableModule(RCC_MOD_BUSMON3);
#endif /* BSP_USING_BUSMON */

#endif /* SOC_BF0_HCPU */
}
#endif /* SF32LB58X */


#ifdef SF32LB56X
void HAL_RCC_MspInit(void)
{
#ifdef SOC_BF0_HCPU

#ifndef BSP_USING_UART1
    HAL_RCC_DisableModule(RCC_MOD_USART1);
#endif /* !BSP_USING_UART1 */
#ifndef BSP_USING_UART2
    HAL_RCC_DisableModule(RCC_MOD_USART2);
#endif /* BSP_USING_UART2 */
#ifndef BSP_USING_UART3
    HAL_RCC_DisableModule(RCC_MOD_USART3);
#endif /* BSP_USING_UART3 */


#ifndef BSP_USING_SPI1
    HAL_RCC_DisableModule(RCC_MOD_SPI1);
#endif /* !BSP_USING_SPI1 */
#ifndef BSP_USING_SPI2
    HAL_RCC_DisableModule(RCC_MOD_SPI2);
#endif /* !BSP_USING_SPI2 */

#ifndef BSP_USING_I2C1
    HAL_RCC_DisableModule(RCC_MOD_I2C1);
#endif /* !BSP_USING_I2C1 */
#ifndef BSP_USING_I2C2
    HAL_RCC_DisableModule(RCC_MOD_I2C2);
#endif /* !BSP_USING_I2C2 */
#ifndef BSP_USING_I2C3
    HAL_RCC_DisableModule(RCC_MOD_I2C3);
#endif /* !BSP_USING_I2C3 */
#ifndef BSP_USING_I2C4
    HAL_RCC_DisableModule(RCC_MOD_I2C4);
#endif /* !BSP_USING_I2C4 */

#if !defined(BSP_USING_GPTIM1) && !defined(BSP_USING_PWM2)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM1);
#endif /* !BSP_USING_GPTIM1 */
#if !defined(BSP_USING_GPTIM2) && !defined(BSP_USING_PWM3)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM2);
#endif /* !BSP_USING_GPTIM2 */

#ifndef BSP_USING_BTIM1
    HAL_RCC_DisableModule(RCC_MOD_BTIM1);
#endif /* !BSP_USING_BTIM1 */
#ifndef BSP_USING_BTIM2
    HAL_RCC_DisableModule(RCC_MOD_BTIM2);
#endif /* !BSP_USING_BTIM2 */

#if !defined(BSP_USING_ATIM1) && !defined(BSP_USING_PWMA1)
    HAL_RCC_DisableModule(RCC_MOD_ATIM1);
#endif /* !BSP_USING_ATIM1 */

#ifndef BSP_ENABLE_MPI1
    HAL_RCC_DisableModule(RCC_MOD_MPI1);
#endif /* !BSP_ENABLE_MPI1 */
#ifndef BSP_ENABLE_MPI2
    HAL_RCC_DisableModule(RCC_MOD_MPI2);
#endif /* !BSP_ENABLE_MPI2 */
#ifndef BSP_ENABLE_MPI3
    HAL_RCC_DisableModule(RCC_MOD_MPI3);
#endif /* !BSP_ENABLE_MPI3 */

#if !defined(BSP_USING_USBD) && !defined(BSP_USING_USBH)
    HAL_RCC_DisableModule(RCC_MOD_USBC);
#endif /* BSP_USING_USBD */

#ifndef BSP_USING_NN_ACC
    HAL_RCC_DisableModule(RCC_MOD_NNACC1);
#endif /* !BSP_USING_NN_ACC */

#ifndef BSP_ENABLE_I2S_CODEC
    HAL_RCC_DisableModule(RCC_MOD_I2S1);
#endif /* !BSP_ENABLE_I2S_CODEC */

#ifndef BSP_USING_SDHCI1
    HAL_RCC_DisableModule(RCC_MOD_SDMMC1);
#endif /* !BSP_USING_SDHCI1 */
#ifndef BSP_USING_SDHCI2
    HAL_RCC_DisableModule(RCC_MOD_SDMMC2);
#endif /* !BSP_USING_SDHCI2 */

#ifndef BSP_USING_PDM1
    HAL_RCC_DisableModule(RCC_MOD_PDM1);
#endif /* !BSP_USING_PDM1 */
#ifndef BSP_USING_PDM2
    HAL_RCC_DisableModule(RCC_MOD_PDM2);
#endif /* !BSP_USING_PDM2 */

#ifndef BSP_ENABLE_AUD_PRC
    HAL_RCC_DisableModule(RCC_MOD_AUDPRC);
#endif /* !BSP_ENABLE_AUD_PRC  */

#ifndef BSP_ENABLE_AUD_CODEC
    HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_HP);
#endif /* !BSP_ENABLE_AUD_CODEC */

#ifndef BSP_USING_BUSMON
    HAL_RCC_DisableModule(RCC_MOD_BUSMON1);
#endif /* BSP_USING_BUSMON */

    HAL_RCC_DisableModule(RCC_MOD_CAN1);

#else

#ifndef BSP_USING_UART4
    HAL_RCC_DisableModule(RCC_MOD_USART4);
#endif /* BSP_USING_UART4 */
#ifndef BSP_USING_UART5
    HAL_RCC_DisableModule(RCC_MOD_USART5);
#endif /* BSP_USING_UART5 */
#ifndef BSP_USING_UART6
    HAL_RCC_DisableModule(RCC_MOD_USART6);
#endif /* BSP_USING_UART6 */

#ifndef BSP_USING_SPI3
    HAL_RCC_DisableModule(RCC_MOD_SPI3);
#endif /* !BSP_USING_SPI3 */
#ifndef BSP_USING_SPI4
    HAL_RCC_DisableModule(RCC_MOD_SPI4);
#endif /* !BSP_USING_SPI4 */

#ifndef BSP_USING_I2C5
    HAL_RCC_DisableModule(RCC_MOD_I2C5);
#endif /* !BSP_USING_I2C5 */
#ifndef BSP_USING_I2C6
    HAL_RCC_DisableModule(RCC_MOD_I2C6);
#endif /* !BSP_USING_I2C6 */
#ifndef BSP_USING_I2C7
    HAL_RCC_DisableModule(RCC_MOD_I2C7);
#endif /* !BSP_USING_I2C6 */

#if !defined(BSP_USING_GPTIM3) && !defined(BSP_USING_PWM4)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM3);
#endif /* !BSP_USING_GPTIM3 */
#if !defined(BSP_USING_GPTIM4) && !defined(BSP_USING_PWM5)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM4);
#endif /* !BSP_USING_GPTIM4 */
#if !defined(BSP_USING_GPTIM5) && !defined(BSP_USING_PWM6)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM5);
#endif /* !BSP_USING_GPTIM5 */

#ifndef BSP_USING_BTIM3
    HAL_RCC_DisableModule(RCC_MOD_BTIM3);
#endif /* !BSP_USING_BTIM3 */
#ifndef BSP_USING_BTIM4
    HAL_RCC_DisableModule(RCC_MOD_BTIM4);
#endif /* !BSP_USING_BTIM4 */

#ifndef BSP_ENABLE_MPI5
    //HAL_RCC_DisableModule(RCC_MOD_MPI5);
#endif /* !BSP_ENABLE_MPI5 */

// only used by HCPU
//#ifndef BSP_ENABLE_AUD_CODEC
//    HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_LP);
//#endif /* !BSP_ENABLE_AUD_CODEC */

#ifndef BSP_USING_BUSMON
    HAL_RCC_DisableModule(RCC_MOD_BUSMON2);
#endif /* BSP_USING_BUSMON */

#endif /* SOC_BF0_HCPU */
}
#endif /* SF32LB56X */


#ifdef SF32LB52X
void HAL_RCC_MspInit(void)
{
#ifdef SOC_BF0_HCPU

#ifndef BSP_USING_UART1
    HAL_RCC_DisableModule(RCC_MOD_USART1);
#endif /* !BSP_USING_UART1 */
#ifndef BSP_USING_UART2
    HAL_RCC_DisableModule(RCC_MOD_USART2);
#endif /* BSP_USING_UART2 */
#ifndef BSP_USING_UART3
    HAL_RCC_DisableModule(RCC_MOD_USART3);
#endif /* BSP_USING_UART3 */


#ifndef BSP_USING_SPI1
    HAL_RCC_DisableModule(RCC_MOD_SPI1);
#endif /* !BSP_USING_SPI1 */
#ifndef BSP_USING_SPI2
    HAL_RCC_DisableModule(RCC_MOD_SPI2);
#endif /* !BSP_USING_SPI2 */

#ifndef BSP_USING_I2C1
    HAL_RCC_DisableModule(RCC_MOD_I2C1);
#endif /* !BSP_USING_I2C1 */
#ifndef BSP_USING_I2C2
    HAL_RCC_DisableModule(RCC_MOD_I2C2);
#endif /* !BSP_USING_I2C2 */
#ifndef BSP_USING_I2C3
    HAL_RCC_DisableModule(RCC_MOD_I2C3);
#endif /* !BSP_USING_I2C3 */
#ifndef BSP_USING_I2C4
    HAL_RCC_DisableModule(RCC_MOD_I2C4);
#endif /* !BSP_USING_I2C4 */

#if !defined(BSP_USING_GPTIM1) && !defined(BSP_USING_PWM2)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM1);
#endif /* !BSP_USING_GPTIM1 */
#if !defined(BSP_USING_GPTIM2) && !defined(BSP_USING_PWM3)
    HAL_RCC_DisableModule(RCC_MOD_GPTIM2);
#endif /* !BSP_USING_GPTIM2 */

#ifndef BSP_USING_BTIM1
    HAL_RCC_DisableModule(RCC_MOD_BTIM1);
#endif /* !BSP_USING_BTIM1 */
#ifndef BSP_USING_BTIM2
    HAL_RCC_DisableModule(RCC_MOD_BTIM2);
#endif /* !BSP_USING_BTIM2 */

#if !defined(BSP_USING_ATIM1) && !defined(BSP_USING_PWMA1)
    HAL_RCC_DisableModule(RCC_MOD_ATIM1);
#endif /* !BSP_USING_ATIM1 */

#ifndef BSP_ENABLE_MPI1
    HAL_RCC_DisableModule(RCC_MOD_MPI1);
#endif /* !BSP_ENABLE_MPI1 */
#ifndef BSP_ENABLE_MPI2
    HAL_RCC_DisableModule(RCC_MOD_MPI2);
#endif /* !BSP_ENABLE_MPI2 */

#if !defined(BSP_USING_USBD) && !defined(BSP_USING_USBH)
    HAL_RCC_DisableModule(RCC_MOD_USBC);
#endif /* BSP_USING_USBD */

#ifndef BSP_ENABLE_I2S_CODEC
    HAL_RCC_DisableModule(RCC_MOD_I2S1);
#endif /* !BSP_ENABLE_I2S_CODEC */

#ifndef BSP_USING_SD_LINE
    HAL_RCC_DisableModule(RCC_MOD_SDMMC1);
#endif /* !BSP_USING_SDHCI1 */

#ifndef BSP_USING_PDM1
    HAL_RCC_DisableModule(RCC_MOD_PDM1);
#endif /* !BSP_USING_PDM1 */

#ifndef BSP_ENABLE_AUD_PRC
    HAL_RCC_DisableModule(RCC_MOD_AUDPRC);
#endif /* !BSP_ENABLE_AUD_PRC  */

#ifndef BSP_ENABLE_AUD_CODEC
    HAL_RCC_DisableModule(RCC_MOD_AUDCODEC);
#endif /* !BSP_ENABLE_AUD_CODEC */

#endif /* SOC_BF0_HCPU */
}
#endif /* SF32LB52X */

#endif /* BSP_NOT_DISABLE_UNUSED_MODULE */

/// @} drv_common
/// @} bsp_driver
/// @} file

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
