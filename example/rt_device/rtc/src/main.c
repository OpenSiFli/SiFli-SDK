#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>

/* struct date_time */
typedef struct date_time
{
    int year;     /* eg: 2024 */
    int month;    /* eg:  1 (1-12) */
    int day;      /* eg: 31 (1~xx) */
    int hour;     /* eg:  8 (0~23) */
    int minute;   /* eg: 30 (0-59) */
    int second;   /* eg:  0 (0~59) */
} date_time_t;

void print_time(const char *tag, date_time_t dt)
{
    printf("%s %04d %02d %02d %02d:%02d:%02d\n", tag, dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
}

/**
 * @brief Convert local time to {date_time_t}
 */
#define LOCAL_TIME_2_DATE_TIME_T(DT, TM)      \
    (DT)->year = (TM)->tm_year + 1900;        \
    (DT)->month = (TM)->tm_mon + 1;           \
    (DT)->day = (TM)->tm_mday;                \
    (DT)->hour = (TM)->tm_hour;               \
    (DT)->minute = (TM)->tm_min;              \
    (DT)->second = (TM)->tm_sec;              \


/**
  * @brief  Set system date and time.
  * @param  dt date and time, eg: 2024.1.1 08:30:00
  *
  * @retval If success, return RT_EOK.
  */
rt_err_t set_date_time(date_time_t dt)
{
    time_t now;
    rt_device_t device;
    struct tm *p_tm;
    struct tm tm_new = {0};

    /* Get current time (calendar time). */
    now = time(RT_NULL);

    /* Lock scheduler */
    rt_enter_critical();
    /* Convert calendar time to local time. */
    p_tm = localtime(&now);
    /* Copy time */
    memcpy(&tm_new, p_tm, sizeof(struct tm));
    /* Unlock scheduler */
    rt_exit_critical();

    /* Update system time. */
    tm_new.tm_year = dt.year  - 1900;
    tm_new.tm_mon  = dt.month - 1;
    tm_new.tm_mday = dt.day;
    tm_new.tm_hour = dt.hour;
    tm_new.tm_min = dt.minute;
    tm_new.tm_sec = dt.second;
    /* Convert local time to calendar time. */
    now = mktime(&tm_new);

    /* Find RTC device. */
    device = rt_device_find("rtc");
    if (device == RT_NULL)
    {
        rt_kprintf("app_set_system_time not find device\n");
        return -RT_ERROR;
    }
    /* Update time to RTC device */
    rt_device_control(device, RT_DEVICE_CTRL_RTC_SET_TIME, &now);

    return RT_EOK;
}

/**
  * @brief  Get system date and time.
  * @param dt pointer to variable used to storing current system time.
  * @retval date_time_t current system time.
  */
date_time_t *get_date_time(date_time_t *dt)
{
    /* Get current time(calendar time) */
    time_t ts = time(NULL);
    /* Convert calendar time to local time */
    struct tm *p_tm = localtime(&ts);

    /* Convert local time to date_time_t  */
    LOCAL_TIME_2_DATE_TIME_T(dt, p_tm);

    return dt;
}

#if defined(RT_USING_RTC) && defined(RT_USING_ALARM)
/**
  * @brief  This function handles alarm.
  * @retval none
  */
static void alarm_callback(rt_alarm_t alarm, time_t timestamp)
{
    /* Convert calendar time to local time */
    struct tm *tm_expired = localtime(&timestamp);
    /* Convert local time to date_time_t  */
    date_time_t dt_expired = {0};

    /* Convert local time to date_time_t  */
    LOCAL_TIME_2_DATE_TIME_T(&dt_expired, tm_expired);
    print_time("Alarm triggered at ", dt_expired);
}

/**
  * @brief  Set alarm.
  * @retval none
  */
void set_alarm(int hour, int min, int sec)
{
    rt_alarm_t g_alarm;
    static struct rt_alarm_setup g_alarm_setup;
    static struct tm tm_now;
    time_t now, ts_new;

    memset(&g_alarm_setup, 0, sizeof(g_alarm_setup));

    /* Get current system time (calendar time) */
    now = time(NULL);
    /* Convert calendar time to UTC time */
    gmtime_r(&now, &g_alarm_setup.wktime);
    g_alarm_setup.wktime.tm_hour = hour;
    g_alarm_setup.wktime.tm_min = min;
    g_alarm_setup.wktime.tm_sec = sec;
    /* Alarm is oneshot alarm.
     * Alarm flags see RT_ALARM_XXX(RT_ALARM_DAILY/RT_ALARM_WEELY...) in alarm.h.
     */
    g_alarm_setup.flag = RT_ALARM_ONESHOT;
    /* Create alarm */
    g_alarm = rt_alarm_create(alarm_callback, &g_alarm_setup);
    /* Start alarm */
    rt_alarm_start(g_alarm);

    rt_kprintf("SET ONESHOT ALARM : [%02d:%02d:%02d] \n", g_alarm_setup.wktime.tm_hour, g_alarm_setup.wktime.tm_min, g_alarm_setup.wktime.tm_sec);
}
#endif

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Set system time to 2024/1/1 8:30:0 */
    date_time_t dt_set = {2024, 1, 1, 8, 30, 0};
    rt_err_t ret = set_date_time(dt_set);
    RT_ASSERT(ret == RT_EOK);
    print_time("set system time (by RT DEVICE): ", dt_set);
    /* Get system time. */
    date_time_t dt_now = {0};
    get_date_time(&dt_now);
    print_time("current system time: ", dt_now);

    /* Use RTT API to set system time to 2024/2/1 8:30:0 */
    dt_set.month = 2;
    set_date(dt_set.year, dt_set.month, dt_set.day);
    set_time(dt_set.hour, dt_set.minute, dt_set.second);
    print_time("set system time (by RTT API): ", dt_set);
    /* Get system time. */
    get_date_time(&dt_now);
    print_time("current system time: ", dt_now);

    /* Set alarm */
#if defined(RT_USING_RTC) && defined(RT_USING_ALARM)
    /* Create a alarm which will expire at 8:32:0 (2 minutes later)*/
    set_alarm(8, 32, 0);
#endif

    /* Infinite loop */
    while (1)
    {
        /* Get system time per second. */
        rt_thread_mdelay(1000);
        get_date_time(&dt_now);
        print_time("current system time: ", dt_now);
    }
    return 0;
}

