/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#ifndef _SYS_TIME_H_
#define _SYS_TIME_H_

#include <time.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MILLISECOND_PER_SECOND  1000UL
#define MICROSECOND_PER_SECOND  1000000UL
#define NANOSECOND_PER_SECOND   1000000000UL

#define MILLISECOND_PER_TICK    (MILLISECOND_PER_SECOND / RT_TICK_PER_SECOND)
#define MICROSECOND_PER_TICK    (MICROSECOND_PER_SECOND / RT_TICK_PER_SECOND)
#define NANOSECOND_PER_TICK     (NANOSECOND_PER_SECOND  / RT_TICK_PER_SECOND)

#ifndef _TIMEVAL_DEFINED
#define _TIMEVAL_DEFINED
/*
 * Structure returned by gettimeofday(2) system call,
 * and used in other calls.
 */
struct timeval
{
    long    tv_sec;     /* seconds */
    long    tv_usec;    /* and microseconds */
};
#endif /* _TIMEVAL_DEFINED */

#ifndef _TIMESPEC_DEFINED
#define _TIMESPEC_DEFINED

#if defined(__ARMCC_VERSION) || defined(_WIN32) || (defined(__ICCARM__) && (__VER__ < 8010001))
/*
 * Structure defined by POSIX.1b to be like a timeval.
 */
struct timespec
{
    time_t  tv_sec;     /* seconds */
    long    tv_nsec;    /* and nanoseconds */
};
#endif /* defined(__ARMCC_VERSION) || defined(_WIN32) || (defined(__ICCARM__) && (__VER__ < 8010001)) */
#endif /* _TIMESPEC_DEFINED */

struct timezone
{
    int tz_minuteswest;   /* minutes west of Greenwich */
    int tz_dsttime;       /* type of dst correction */
};

int gettimeofday(struct timeval *tp, struct timezone *tz);
struct tm *gmtime_r(const time_t *timep, struct tm *r);
int rt_timespec_to_tick(const struct timespec *time);
void tz_set(struct timezone  *tz);
struct timezone   *tz_get(void);

#ifdef __cplusplus
}
#endif

#endif /* _SYS_TIME_H_ */
