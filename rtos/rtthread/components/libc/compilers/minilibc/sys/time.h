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

#include <sys/types.h>

#ifndef _TIME_T_DECLARED
    #define _TIME_T_DECLARED
    typedef long time_t;
#endif
/*
 * Structure returned by gettimeofday(2) system call,
 * and used in other calls.
 */
struct timeval
{
    long    tv_sec;     /* seconds */
    long    tv_usec;    /* and microseconds */
};

/*
 * Structure defined by POSIX.1b to be like a timeval.
 */
#if !defined(_TIME_SPEC_DECLARED)
#define _TIME_SPEC_DECLARED
struct timespec
{
    time_t  tv_sec;     /* seconds */
    long    tv_nsec;    /* and nanoseconds */
};
#endif

struct timezone
{
    int tz_minuteswest;   /* minutes west of Greenwich */
    int tz_dsttime;   /* type of dst correction */
};

struct tm
{
    int tm_sec;           /* Seconds. [0-60] (1 leap second) */
    int tm_min;           /* Minutes. [0-59] */
    int tm_hour;          /* Hours.   [0-23] */
    int tm_mday;          /* Day.     [1-31] */
    int tm_mon;           /* Month.   [0-11] */
    int tm_year;          /* Year - 1900. */
    int tm_wday;          /* Day of week. [0-6] */
    int tm_yday;          /* Days in year.[0-365] */
    int tm_isdst;         /* DST.     [-1/0/1]*/

    long int tm_gmtoff;       /* Seconds east of UTC.  */
    const char *tm_zone;      /* Timezone abbreviation.  */
};

int gettimeofday(struct timeval *tp, struct timezone *tz);
void tz_set(struct timezone  *tz);
struct timezone   *tz_get(void);


#endif

