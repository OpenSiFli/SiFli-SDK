/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-26     zylx         first version
 */
#include "rtthread.h"
#include "rthw.h"
#include "string.h"
#if defined(__CC_ARM) || defined(__CLANG_ARM) || defined (__IAR_SYSTEMS_ICC__)
#include <sys/time.h>

/* seconds per day */
#define SPD 24*60*60

/* days per month -- nonleap! */
const short __spm[13] =
{
    0,
    (31),
    (31 + 28),
    (31 + 28 + 31),
    (31 + 28 + 31 + 30),
    (31 + 28 + 31 + 30 + 31),
    (31 + 28 + 31 + 30 + 31 + 30),
    (31 + 28 + 31 + 30 + 31 + 30 + 31),
    (31 + 28 + 31 + 30 + 31 + 30 + 31 + 31),
    (31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30),
    (31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31),
    (31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30),
    (31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30 + 31),
};

int __isleap(int year)
{
    /* every fourth year is a leap year except for century years that are
     * not divisible by 400. */
    /*  return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)); */
    return (!(year % 4) && ((year % 100) || !(year % 400)));
}

/**
 * This function will convert Time (Restartable)
 *
 * @param timep the timestamp
 * @param the structure to stores information
 *
 * @return the structure to stores information
 *
 */
struct tm *gmtime_r(const time_t *timep, struct tm *r)
{
    time_t i;
    register time_t work = *timep % (SPD);
    r->tm_sec = work % 60;
    work /= 60;
    r->tm_min = work % 60;
    r->tm_hour = work / 60;
    work = *timep / (SPD);
    r->tm_wday = (4 + work) % 7;
    for (i = 1970;; ++i)
    {
        register time_t k = __isleap(i) ? 366 : 365;
        if (work >= k)
            work -= k;
        else
            break;
    }
    r->tm_year = i - 1900;
    r->tm_yday = work;

    r->tm_mday = 1;
    if (__isleap(i) && (work > 58))
    {
        if (work == 59)
            r->tm_mday = 2; /* 29.2. */
        work -= 1;
    }

    for (i = 11; i && (__spm[i] > work); --i)
        ;
    r->tm_mon = i;
    r->tm_mday += work - __spm[i];
    return r;
}
#endif /* end of __CC_ARM or __CLANG_ARM or __IAR_SYSTEMS_ICC__ */

#if defined(__CC_ARM) || defined(__CLANG_ARM) || defined (__IAR_SYSTEMS_ICC__) || defined(__GNUC__)
static struct timezone _current_timezone;

void tz_set(struct timezone  *tz)
{
    rt_base_t level;
    level = rt_hw_interrupt_disable();
    memcpy(&_current_timezone, tz, sizeof(struct timezone));
    rt_hw_interrupt_enable(level);
}

struct timezone   *tz_get(void)
{
    return &_current_timezone;
}

#ifdef RT_USING_DEVICE
int gettimeofday(struct timeval *tp, struct timezone  *tz)
{
    time_t time;
    rt_device_t device;

    device = rt_device_find("rtc");
    RT_ASSERT(device != RT_NULL);

    if (tp)
        rt_device_control(device, RT_DEVICE_CTRL_RTC_GET_TIMEVAL, tp);
    if (tz)
        memcpy(tz, &_current_timezone, sizeof(struct timezone));
    return time;
}
#endif
#endif

