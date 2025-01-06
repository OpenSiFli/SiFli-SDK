/**
  ******************************************************************************
  * @file   drv_rtc.c
  * @author Sifli software development team
  * @brief Real Timer Clock BSP driver
  This driver is validated by using MSH command 'date'.
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

#include <string.h>
#include "board.h"
#include "bf0_pm.h"
#include "sys/time.h"

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif

#ifdef FPGA
    #define DEFAULT_CYCLE 960000    // Assume 10K, Total cycles are 48000 * 200 / 10.
#else
    #define DEFAULT_CYCLE 1200000   // Assume 8K, Total cycles are 48000 * 200 / 8.
#endif

#ifndef BSP_RTC_PPM
    #define BSP_RTC_PPM 0
#endif

extern struct tm *gmtime_r(const time_t *timep, struct tm *r);

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_rtc RTC
  * @brief Real Timer Clock BSP driver
  This driver is validated by using MSH command 'date'.
  * @{
  */

#if defined(BSP_USING_ONCHIP_RTC) || defined(_SIFLI_DOXYGEN_)

//#define DRV_DEBUG
#define LOG_TAG             "drv.rtc"
#include <drv_log.h>

#define BKUP_REG_DATA 0xA5A5

static struct rt_device rtc;

__ROM_USED RTC_HandleTypeDef RTC_Handler;

static struct rt_mutex rtc_mutex;

static inline void rtc_enter_critical(void)
{
    rt_err_t err;

    err = rt_mutex_take(&rtc_mutex, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
}

static inline void rtc_exit_critical(void)
{
    rt_err_t err;

    err = rt_mutex_release(&rtc_mutex);
    RT_ASSERT(RT_EOK == err);
}

/**
* @brief  Get current date and time.
* @retval current data/time in struct tm format.
*/
static time_t get_rtc_timestamp(void)
{
    time_t r = drv_get_timestamp();

    LOG_D("get rtc time.");
    return r;
}

/**
* @brief  Set current date and time.
* @param[in] time_stamp current data/time in timestamp.
* @retval RT_EOK if success,  -RT_ERROR if failed.
*/
static rt_err_t set_rtc_time_stamp(time_t time_stamp)
{
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    struct tm *p_tm;

    p_tm = localtime(&time_stamp);
    RTC_TimeStruct.Seconds = p_tm->tm_sec ;
    RTC_TimeStruct.Minutes = p_tm->tm_min ;
    RTC_TimeStruct.Hours   = p_tm->tm_hour;
    RTC_DateStruct.Date    = p_tm->tm_mday;
    RTC_DateStruct.Month   = p_tm->tm_mon + 1 ;
    RTC_DateStruct.Year    = p_tm->tm_year;
    RTC_DateStruct.WeekDay = p_tm->tm_wday == 0 ? RTC_WEEKDAY_SUNDAY : p_tm->tm_wday;

    if (HAL_RTC_SetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN) != HAL_OK)
    {
        return -RT_ERROR;
    }
    if (HAL_RTC_SetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN) != HAL_OK)
    {
        return -RT_ERROR;
    }
    if (HAL_LXT_DISABLED())
    {
        drv_rtc_calculate_delta(1);  //Reset RTC update inteval.
    }
    LOG_D("set rtc time. %d:%d:%d", p_tm->tm_hour, p_tm->tm_min,  p_tm->tm_sec);
    return RT_EOK;
}

/**
* @brief  Get current date and time.
  @param  subseconds return subseconds, in unit of 1/RC10K_SUB_SEC_DIVB second
* @retval current senconds
*/
static uint32_t get_rtc_timestamp2(uint32_t *subseconds)
{
    float t;
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    struct tm tm_new;

    HAL_RTC_GetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
    while (HAL_RTC_GetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN) == HAL_ERROR)
    {
        HAL_RTC_GetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
    };

    tm_new.tm_sec  = RTC_TimeStruct.Seconds;
    tm_new.tm_min  = RTC_TimeStruct.Minutes;
    tm_new.tm_hour = RTC_TimeStruct.Hours;
    tm_new.tm_mday = RTC_DateStruct.Date;
    tm_new.tm_mon  = RTC_DateStruct.Month - 1;
    tm_new.tm_wday  = RTC_DateStruct.WeekDay == RTC_WEEKDAY_SUNDAY ? 0 : RTC_DateStruct.WeekDay;

    if (RTC_DateStruct.Year & RTC_CENTURY_BIT)
        tm_new.tm_year = RTC_DateStruct.Year & (~RTC_CENTURY_BIT);
    else
        tm_new.tm_year = RTC_DateStruct.Year + 100;

    *subseconds = RTC_TimeStruct.SubSeconds;
    LOG_D("get rtc time2. %d:%d:%d", tm_new.tm_hour,  tm_new.tm_min, tm_new.tm_sec);
    return mktime(&tm_new);
}

/**
* @brief  RTC initialize
*
*/
static void rt_rtc_init(void)
{
#if 0
    __HAL_RCC_PWR_CLK_ENABLE();

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
#endif

}

void drv_rtc_callback(int reason)
{
#ifdef RT_USING_ALARM
    if (RTC_CBK_ALARM == reason)
    {
        rt_alarm_update(&rtc, 1);
    }
    else if (RTC_CBK_WAKEUP == reason)
    {
        HAL_RTC_DeactivateWakeUpTimer(&RTC_Handler);
    }


#endif
}

uint32_t rtc_get_lpcycle()
{
    uint32_t value;
#ifdef  BF0_HCPU
    value = HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
    if (value == 0)
        value = DEFAULT_CYCLE;
    value += BSP_RTC_PPM;   // Calibrate in initial with 8 cycle
    LOG_I("RC10K Freq=%dkHz, cycle=%d, rtc ppm=%d\n", 48000 * LXT_LP_CYCLE / (uint32_t)value, (uint32_t)value, BSP_RTC_PPM);
    HAL_Set_backup(RTC_BACKUP_LPCYCLE, (uint32_t)value);
#else
    value = HAL_Get_backup(RTC_BACKUP_LPCYCLE);
#endif
    return value;
}

void rtc_rc10_calculate_div(RTC_HandleTypeDef *hdl, uint32_t value)
{
    hdl->Init.DivB = RC10K_SUB_SEC_DIVB;

    // 1 seconds has total 1/(x/(48*8))/256=1.5M/x cycles, times 2^14 for DIVA
    uint32_t divider = RTC_Handler.Init.DivB * value;
    value = ((uint64_t)48000000 * LXT_LP_CYCLE * (1 << 14) + (divider >> 1)) / divider;
    hdl->Init.DivAInt = (uint32_t)(value >> 14);
    hdl->Init.DivAFrac = (uint32_t)(value & ((1 << 14) - 1));
    LOG_I("DIVA=%d, DIVA_FRA=%d, DIVB=%d\n",
          hdl->Init.DivAInt, hdl->Init.DivAFrac, hdl->Init.DivB);
}


#if defined(LXT_DISABLE) && defined(BF0_HCPU)
bool rtc_reconfig()
{
    uint32_t cur_ave;

    cur_ave = rtc_get_lpcycle();
    rtc_rc10_calculate_div(&RTC_Handler, cur_ave);

    LOG_I("rtc change cycle=%d ppm=%d", HAL_Get_backup(RTC_BACKUP_LPCYCLE),  BSP_RTC_PPM);
    if (HAL_RTC_Init(&RTC_Handler, RTC_INIT_REINIT) != HAL_OK)
    {
        LOG_I("rtc reconfig error");
        return false;
    }
    HAL_RTC_RegCallback(&RTC_Handler, drv_rtc_callback);
    HAL_Set_backup(RTC_BACKUP_LPCYCLE,  cur_ave);
    drv_set_soft_rc10_backup(cur_ave);
    LOG_I("rtc reconfig ok ave=%d", cur_ave);
    return true;
}
#endif

/**
* @brief  RTC configuration
* @param[in] dev rtc device object.
*/
static rt_err_t rt_rtc_config(struct rt_device *dev)
{
    uint32_t wakesrc;

    RTC_Handler.Instance = (RTC_TypeDef *) RTC_BASE;

    uint32_t psclr = RTC_Handler.Instance->PSCLR;
    LOG_I("PSCLR=0x%x DivAI=%d DivAF=%d B=%d", psclr,
          (psclr  & RTC_PSCLR_DIVA_INT) >> RTC_PSCLR_DIVA_INT_Pos,
          (psclr  & RTC_PSCLR_DIVA_FRAC) >> RTC_PSCLR_DIVA_FRAC_Pos,
          (psclr  & RTC_PSCLR_DIVB) >> RTC_PSCLR_DIVB_Pos);

#ifndef LXT_DISABLE
    // Wait for LXT Ready.
#ifdef SF32LB52X
    if (HAL_PMU_LXTReady() != HAL_OK)
#else
    if (HAL_RTC_LXT_ENABLED() && HAL_PMU_LXTReady() != HAL_OK)
#endif
    {
        LOG_I("RTC use LXT, but LXT is not ready\n");
        RT_ASSERT(0);
    }
#endif

    // Set default DIVA/B
    RTC_Handler.Init.DivAInt = LXT_FREQ / RC10K_SUB_SEC_DIVB;
    RTC_Handler.Init.DivAFrac = (LXT_FREQ % RC10K_SUB_SEC_DIVB) * ((1 << 14) / RC10K_SUB_SEC_DIVB);
    RTC_Handler.Init.DivB = RC10K_SUB_SEC_DIVB;

    if (HAL_RTC_LXT_ENABLED())
        LOG_I("RTC use LXT RTC_CR=%08X\n", hwp_rtc->CR);
    else
    {
        uint64_t value = 0;
        value = rtc_get_lpcycle();
        drv_set_soft_rc10_backup((uint32_t)value);
        if (value)
            rtc_rc10_calculate_div(&RTC_Handler, value);
    }


    if (0 == SystemPowerOnModeGet() &&                      // Cold boot
            HAL_Get_backup(RTC_BACKUP_INITIALIZED) == 0)    // And not software reboot, which RTC already initialized.
    {
        wakesrc = RTC_INIT_NORMAL;
    }
    else
    {
        wakesrc = RTC_INIT_SKIP;
    }

    LOG_I("Init RTC, wake = %d\n", wakesrc);
    if (HAL_RTC_Init(&RTC_Handler, wakesrc) != HAL_OK)
    {
        return -RT_ERROR;
    }

    HAL_RTC_RegCallback(&RTC_Handler, drv_rtc_callback);
    HAL_Set_backup(RTC_BACKUP_INITIALIZED, 1);
    return RT_EOK;
}

#define RC10K_SW_PPM  0

#define RTC_DELTA_THRESHOLD     1000            // Threashhold to update RTC
#define MAX_DELTA_BETWEEN_RTC_AVE               100
void drv_rtc_calculate_delta(int reset)
{
#if defined(LXT_DISABLE) && defined(BF0_HCPU)
    static uint32_t rtc_changed = 0;
    static uint32_t rtc_cycle_count_init = 0;
    static double rtc_a = 0.0;
    static double delta_total = 0.0;

    if (rtc_changed == 0 && reset == 0)
    {
        if (rtc_reconfig())
        {
            rtc_changed = 1;
            rtc_cycle_count_init = HAL_Get_backup(RTC_BACKUP_LPCYCLE);
        }
    }
    if (rtc_cycle_count_init == 0 || reset)
    {
        rtc_cycle_count_init = HAL_Get_backup(RTC_BACKUP_LPCYCLE);  // Get initial lpcycle, RTC is running based on it.
        delta_total = 0.0;
        uint32_t sub = 0;
        time_t t = get_rtc_timestamp2(&sub);
        rtc_a = 1.0 * t + ((float)(1.0 * sub)) / RC10K_SUB_SEC_DIVB;
        LOG_I("Get initial cycles: %d rtc=%.4f\r\n", rtc_cycle_count_init, rtc_a);
    }
    else
    {
        uint32_t sub2 = 0;
        double rtc_cal = 0.0;
        double delta = 0.0;
        time_t t2 = get_rtc_timestamp2(&sub2);
        uint32_t cur_ave = HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE) ;
        uint32_t ref_cycle = cur_ave + RC10K_SW_PPM;

        double rtc_b = 1.0 * t2 + ((double)(1.0 * sub2)) / RC10K_SUB_SEC_DIVB;

        delta = rtc_b - rtc_a;                              // Delta time between rtc_a to rtc_b, in seconds.
        rtc_cal = delta * ref_cycle / rtc_cycle_count_init + rtc_a; // Calculate accurate rtc_b
        delta = rtc_cal - rtc_b;                            // Detla time of accurrate rtc_b and current rtc_b

        delta_total += delta;                               // Accumulate error;

        if (delta_total > 1.0 || delta_total < -1.0)
        {
            rtc_cal = delta_total + rtc_b;                  // Accurate time

            set_rtc_time_stamp((uint32_t)rtc_cal);          // Apply integal part difference.

            rt_kprintf("rtc change from %d to %d (a=%.4f b=%.4f c=%.4f delta_sum=%.4f)\n", (uint32_t)rtc_b, (uint32_t)rtc_cal, rtc_a, rtc_b, rtc_cal, delta_total);

            delta_total = rtc_cal - (uint32_t)rtc_cal;      // Continue with subseconds
            rtc_a = (uint32_t)rtc_cal;                      // Next inteval start time
            if ((cur_ave > rtc_cycle_count_init && (cur_ave - rtc_cycle_count_init) > MAX_DELTA_BETWEEN_RTC_AVE)
                    || (cur_ave < rtc_cycle_count_init && (rtc_cycle_count_init - cur_ave) > MAX_DELTA_BETWEEN_RTC_AVE))
            {
                if (rtc_reconfig())
                {
                    rtc_cycle_count_init = HAL_Get_backup(RTC_BACKUP_LPCYCLE);
                }
            }
        }
        else
        {
            rtc_a = rtc_b;                                  // Next inteval start time
            rt_kprintf("|deta| <= 1.0\n");
        }
        rt_kprintf("origin: f=%.4fKHz,cycle=%d avr: f=%.4fKHz cycle_ave=%d delta=%.4lf, delta_sum=%.4lf\n",
                   (float)48000 * LXT_LP_CYCLE / rtc_cycle_count_init, rtc_cycle_count_init, (float)48000 * LXT_LP_CYCLE / ref_cycle,  ref_cycle, delta, delta_total);
    }
#endif
}


/** @defgroup rtc_device Real time clock device functions registered to OS
  * @ingroup drv_rtc
  * @{
 */

/**
* @brief  RTC controls.
* @param[in]  dev: rtc device handle.
* @param[in]  cmd: control commands.
* @param[in]  args: control command arguments.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t rt_rtc_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
#ifdef RT_USING_ALARM
    static RTC_AlarmTypeDef sAlarm;
#endif

    rtc_enter_critical();

    RT_ASSERT(dev != RT_NULL);
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *(rt_uint32_t *)args = get_rtc_timestamp();
        LOG_D("RTC: get rtc_time %x\n", *(rt_uint32_t *)args);
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        if (set_rtc_time_stamp(*(rt_uint32_t *)args))
        {
            result = -RT_ERROR;
        }
        LOG_D("RTC: set rtc_time %x\n", *(rt_uint32_t *)args);
        break;
    case RT_DEVICE_CTRL_RTC_INC_1S:
    {
        int32_t inc = (int32_t) args;
        HAL_RTC_IncOneSecond(&RTC_Handler, inc);
        break;
    }

    case RT_DEVICE_CTRL_RTC_GET_TIMEVAL:
    {
        struct timeval *tv = (struct timeval *)args;
        tv->tv_sec = get_rtc_timestamp2((uint32_t *)&tv->tv_usec);
        tv->tv_usec = tv->tv_usec * (1000000 / RC10K_SUB_SEC_DIVB);
        break;
    }
#ifdef RT_USING_ALARM
    case RT_DEVICE_CTRL_RTC_SET_ALARM:
    {
        struct rt_rtc_wkalarm *wkalarm = (struct rt_rtc_wkalarm *)args;
        if (wkalarm->enable)
        {
            memset(&(sAlarm), 0, sizeof(sAlarm));
            sAlarm.AlarmTime.Hours = wkalarm->tm_hour;
            sAlarm.AlarmTime.Minutes = wkalarm->tm_min;
            sAlarm.AlarmTime.Seconds = wkalarm->tm_sec;
            /*Ignore Month,day and Weekday, and use high precison on subsecond match to avoid multiple interrupt in one alarm*/
            sAlarm.AlarmMask = RTC_ALRMDR_MSKD | RTC_ALRMDR_MSKM | RTC_ALRMDR_MSKWD | (10 << RTC_ALRMDR_MSKSS_Pos) ;
            if (HAL_OK != HAL_RTC_SetAlarm(&RTC_Handler, &sAlarm, RTC_FORMAT_BIN))
            {
                result = -RT_ERROR;
            }
        }
        else
        {
            if (HAL_OK != HAL_RTC_DeactivateAlarm(&RTC_Handler))
            {
                result = -RT_ERROR;
            }
        }
    }
    break;

    case RT_DEVICE_CTRL_RTC_GET_ALARM:
    {
        struct rt_rtc_wkalarm *wkalarm = (struct rt_rtc_wkalarm *)args;
        memset(&(sAlarm), 0, sizeof(sAlarm));
        if (HAL_OK == HAL_RTC_GetAlarm(&RTC_Handler, &sAlarm, RTC_FORMAT_BIN))
        {
            wkalarm->tm_hour = sAlarm.AlarmTime.Hours;
            wkalarm->tm_min = sAlarm.AlarmTime.Minutes;
            wkalarm->tm_sec = sAlarm.AlarmTime.Seconds;
        }
        else
        {
            result = -RT_ERROR;
        }
    }
    break;
    case RT_DEVICE_CTRL_RTC_SET_WAKE_TIMER:
    {
        uint32_t sleep_tick = (uint32_t) args;
        if (HAL_OK != HAL_RTC_SetWakeUpTimer(&RTC_Handler, sleep_tick / 1000 * 256, RTC_WAKEUP_SUBSEC))
        {
            result = -RT_ERROR;
        }
        break;
    }
#endif
    }

    rtc_exit_critical();
    return result;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rtc_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_rtc_control
};
#endif

/// @} rtc_device

/**
* @brief  Register RTC device.
* @param[in]  device: rtc device handle.
* @param[in]  name: device name.
* @param[in]  flag: device flags..
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t rt_hw_rtc_register(rt_device_t device, const char *name, rt_uint32_t flag)
{
    RT_ASSERT(device != RT_NULL);

    rt_rtc_init();
    if (rt_rtc_config(device) != RT_EOK)
    {
        return -RT_ERROR;
    }
#ifdef RT_USING_DEVICE_OPS
    device->ops         = &rtc_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_rtc_control;
#endif
    device->type        = RT_Device_Class_RTC;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->user_data   = RT_NULL;

    /* register a character device */
    return rt_device_register(device, name, flag);
}

/**
* @brief  RTC device driver initilization.
* This is entry function of RTC device driver.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
__HAL_ROM_USED int rt_hw_rtc_init(void)
{
    rt_err_t result;

    result = rt_mutex_init(&rtc_mutex, "rtc", 0);
    RT_ASSERT(RT_EOK == result);

    result = rt_hw_rtc_register(&rtc, "rtc", RT_DEVICE_FLAG_RDWR);
    if (result != RT_EOK)
    {
        LOG_E("rtc register err code: %d", result);
        return result;
    }
    LOG_D("rtc init success");
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_rtc_init);


/**
  * @brief  This function handles Alarm interrupt request.
  * @retval None
  */
void RTC_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_RTC_IRQHandler(&RTC_Handler);
    /* leave interrupt */
    rt_interrupt_leave();
}


#endif /* BSP_USING_ONCHIP_RTC */

/// @} drv_rtc
/// @} bsp_driver

/** @addtogroup bsp_sample BSP driver sample commands.
  * @{
  */

/** @defgroup bsp_sample_rtc RTC sample commands
  * @brief RTC sample commands
  *
  * RTC driver could be tested with MSH command 'date',
  *
  * @{
  */
/// @} bsp_sample_rtc
/// @} bsp_sample


#ifdef APP_BSP_TEST
#include <finsh.h>
#include <rtdevice.h>

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH) &&defined(RT_USING_ALARM)

static rt_alarm_t g_alarm, g_alarm2, g_alarm3;
static rt_device_t device;


static void alarm_callback(rt_alarm_t alarm, time_t timestamp)
{
    LOG_I("Alarm triggered, %d\n", timestamp);
}

static struct rt_alarm_setup g_alarm_setup;
static time_t now;
static void alarm(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        /* set time and date */
        uint16_t year;
        uint8_t month, day, hour, min, sec;

        if (strcmp(argv[1], "enable") == 0)
        {
            sec = atoi(argv[2]);
            memset(&g_alarm_setup, 0, sizeof(g_alarm_setup));
            static time_t now;
            static struct tm *tm, tm_tmp;

            now = time(NULL);
            gmtime_r(&now, &g_alarm_setup.wktime);
            g_alarm_setup.flag = RT_ALARM_DAILY;
            g_alarm_setup.wktime.tm_sec += sec;
            if (g_alarm_setup.wktime.tm_sec > 59)
            {
                g_alarm_setup.wktime.tm_sec -= 59;
                g_alarm_setup.wktime.tm_min += 1;
            }
            g_alarm = rt_alarm_create(alarm_callback, &g_alarm_setup);
            g_alarm_setup.flag = RT_ALARM_ONESHOT;
            g_alarm2 = rt_alarm_create(alarm_callback, &g_alarm_setup);
            g_alarm_setup.flag = RT_ALARM_DAILY;
            g_alarm_setup.wktime.tm_sec += 1;
            if (g_alarm_setup.wktime.tm_sec > 59)
            {
                g_alarm_setup.wktime.tm_sec -= 59;
                g_alarm_setup.wktime.tm_min += 1;
            }
            g_alarm3 = rt_alarm_create(alarm_callback, &g_alarm_setup);
            LOG_I("Alarm started\n");
            rt_alarm_start(g_alarm);
            rt_alarm_start(g_alarm2);
            rt_alarm_start(g_alarm3);
        }
        else
        {
            rt_alarm_stop(g_alarm);
            rt_alarm_delete(g_alarm);
            rt_alarm_stop(g_alarm2);
            rt_alarm_delete(g_alarm2);
            rt_alarm_stop(g_alarm3);
            rt_alarm_delete(g_alarm3);
        }
    }
    else
    {
        LOG_E("please input: alarm [enable/disable] [hour min sec] \n");
        LOG_E("e.g: alarm enable 14 01 32  or alarm disable\n");
    }
}
MSH_CMD_EXPORT(alarm, set alarm [hour min sec]);
#endif /* defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)*/
#endif /* BSP_TEST*/



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
