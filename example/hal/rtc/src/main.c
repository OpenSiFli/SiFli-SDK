#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include "rtthread.h"


/* RTC functions and declarations. */
RTC_HandleTypeDef RTC_Handler;

/**
  * @brief  This function handles Alarm interrupt request.
  * @retval None
  */
void RTC_IRQHandler(void)
{
    rt_interrupt_enter();
    /* enter interrupt */
    HAL_RTC_IRQHandler(&RTC_Handler);
    /* leave interrupt */
    rt_interrupt_leave();
}

/**
  * @brief  This function will called by HAL_RTC_IRQHandler.
  * @retval None
  */
void drv_rtc_callback(int reason)
{
    if (RTC_CBK_ALARM == reason)
    {
        rt_kprintf("Alarm triggered.\n");
        // To handle Alarm expired.
    }
    else if (RTC_CBK_WAKEUP == reason)
    {
        HAL_RTC_DeactivateWakeUpTimer(&RTC_Handler);
    }
}

/**
  * @brief  Initialization and configuration RTC drvier instance.
  */
void rtc_config(void)
{
    /* RTC instance. */
    RTC_Handler.Instance = (RTC_TypeDef *) RTC_BASE;

    /* RTC use LXT (32K). Check whether LXT is enabled and ready. */
#ifndef LXT_DISABLE
#ifdef SF32LB52X
    if (HAL_PMU_LXTReady() != HAL_OK)
#else
    if (HAL_RTC_LXT_ENABLED() && HAL_PMU_LXTReady() != HAL_OK)
#endif
    {
        rt_kprintf("RTC use LXT, but LXT is not ready\n");
        HAL_ASSERT(0);
    }

    /* Set default DIVA/B. */
    RTC_Handler.Init.DivAInt = 0x80;
    RTC_Handler.Init.DivAFrac = 0x0;
    RTC_Handler.Init.DivB = 0x100;
    rt_kprintf("RTC use LXT RTC_CR=%08X\n", hwp_rtc->CR);
#else
#error "LXT IS NEEDED."
#endif
    /* hal rtc initialization. */
    /* wakesrc see RTC_INIT_xxx (RTC_INIT_NORMAL/RTC_INIT_SKIP/RTC_INIT_REINIT) in bf0_hal_rtc.h. */
    uint32_t wakesrc = RTC_INIT_NORMAL;
    if (HAL_RTC_Init(&RTC_Handler, wakesrc) != HAL_OK)
    {
        rt_kprintf("RTC Init failed.\n");
        HAL_ASSERT(0);
    }
    rt_kprintf("RTC Init success.\n");
    // HAL_RTC_RegCallback(&RTC_Handler, drv_rtc_callback);
    // HAL_Set_backup(RTC_BACKUP_INITIALIZED, 1);
}

/**
  * @brief  This function sets default date and time.
  */
void set_date_time(void)
{
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    struct tm p_tm;

    /* Set Time to 8:30:00, Set Date to 2025.01.01 . */
    p_tm.tm_sec  = 0;
    p_tm.tm_min  = 30;
    p_tm.tm_hour = 8;
    p_tm.tm_mday = 1;
    p_tm.tm_mon  = 0;  /* month since january, 0 ~ 11 */
    p_tm.tm_year = (2025 - 1900);  /* year since 1900. */
    p_tm.tm_wday  = 3;
    RTC_TimeStruct.Seconds = p_tm.tm_sec ;
    RTC_TimeStruct.Minutes = p_tm.tm_min ;
    RTC_TimeStruct.Hours   = p_tm.tm_hour;
    RTC_DateStruct.Date    = p_tm.tm_mday;
    RTC_DateStruct.Month   = p_tm.tm_mon + 1 ;
    RTC_DateStruct.Year    = p_tm.tm_year;
    RTC_DateStruct.WeekDay = p_tm.tm_wday == 0 ? RTC_WEEKDAY_SUNDAY : p_tm.tm_wday;

    /* Set time. */
    if (HAL_RTC_SetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN) != HAL_OK)
    {
        rt_kprintf("SET TIME ERR!\n");
        HAL_ASSERT(0);
    }

    /* Set date. */
    if (HAL_RTC_SetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN) != HAL_OK)
    {
        rt_kprintf("SET DATE ERR!\n");
        HAL_ASSERT(0);
    }

    rt_kprintf("SET RTC TIME : %s", asctime(&p_tm));
}

/**
  * @brief  This function gets current date and time by RTC.
  * @retval none
  */
void get_date_time(void)
{
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    struct tm tm_new;

    /* Get time. */
    HAL_RTC_GetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
    /* Get date. */
    while (HAL_RTC_GetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN) == HAL_ERROR)
    {
        /* Retry if error. */
        HAL_RTC_GetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
    };

    /* Convert to local time. */
    tm_new.tm_sec  = RTC_TimeStruct.Seconds + ((RTC_TimeStruct.SubSeconds > 128) ? 1 : 0);
    tm_new.tm_min  = RTC_TimeStruct.Minutes;
    tm_new.tm_hour = RTC_TimeStruct.Hours;
    tm_new.tm_mday = RTC_DateStruct.Date;
    tm_new.tm_mon  = RTC_DateStruct.Month - 1;
    tm_new.tm_wday  = RTC_DateStruct.WeekDay == RTC_WEEKDAY_SUNDAY ? 0 : RTC_DateStruct.WeekDay;
    if (RTC_DateStruct.Year & RTC_CENTURY_BIT)
        tm_new.tm_year = RTC_DateStruct.Year & (~RTC_CENTURY_BIT);
    else
        tm_new.tm_year = RTC_DateStruct.Year + 100;

    rt_kprintf("GET RTC TIME : %s", asctime(&tm_new));
}

/**
  * @brief  Set alarm.
  * @retval none
  */
void set_alarm(void)
{
    static RTC_AlarmTypeDef sAlarm;

    /* Set one alarm which will expired at 8:31:00. */
    memset(&(sAlarm), 0, sizeof(sAlarm));
    sAlarm.AlarmTime.Hours = 8;
    sAlarm.AlarmTime.Minutes = 31;
    sAlarm.AlarmTime.Seconds = 0;
    rt_kprintf("SET ALARM : [%d %d %d]\n", sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes, sAlarm.AlarmTime.Seconds);
    /* Ignore Month,day and Weekday, and use high precison on subsecond match to avoid multiple interrupt in one alarm */
    sAlarm.AlarmMask = RTC_ALRMDR_MSKD | RTC_ALRMDR_MSKM | RTC_ALRMDR_MSKWD | (10 << RTC_ALRMDR_MSKSS_Pos) ;
    /* Set alarm. */
    HAL_RTC_SetAlarm(&RTC_Handler, &sAlarm, RTC_FORMAT_BIN);
    /* Register rtc callback function. */
    HAL_RTC_RegCallback(&RTC_Handler, drv_rtc_callback);

    /* Deactivate alarm */
    // HAL_RTC_DeactivateAlarm(&RTC_Handler);
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{

    /* RTC initialization. */
    rtc_config();
    /* Set system time. */
    set_date_time();
    /* Get system time. */
    get_date_time();
    /* Setup alarm. */
    set_alarm();

    /* Infinite loop */
    while (1)
    {
        /* Dleay 1s. */
        HAL_Delay(1000);
        /* Get system time. */
        get_date_time();
    }
    return 0;
}

