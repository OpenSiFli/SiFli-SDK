/** @file supp_crypto.c
 *
 *  @brief  This file provides mbedtls port layer API for supplicant
 *
 *  Copyright 2024 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "supp_crypto.h"

#ifdef RW610
/* As mbedtls didn't support time check for cert files, we add support for it internally, and only use it temporary.
 * Delete this impl after mbedtls support it.
 */
#include "mbedtls/platform_time.h"
#include "fsl_rtc.h"

#ifdef MBEDTLS_PLATFORM_TIME_ALT
mbedtls_time_t time_func_alt(mbedtls_time_t *timer)
{
    (void)(timer);
    return 0;
}
#endif

void supp_set_mbedtls_set_time()
{
#ifdef MBEDTLS_PLATFORM_TIME_ALT
    mbedtls_platform_set_time(time_func_alt);
#endif
}

#ifdef MBEDTLS_PLATFORM_GMTIME_R_ALT
struct tm *mbedtls_platform_gmtime_r(const mbedtls_time_t *tt,
                                     struct tm *tm_buf)
{
    (void)(tt);
    rtc_datetime_t date;

    /* Get date time */
    RTC_GetDatetime(RTC, &date);

    tm_buf->tm_sec = date.second;
    tm_buf->tm_min = date.minute;
    tm_buf->tm_hour = date.hour;
    tm_buf->tm_mday = date.day;
    tm_buf->tm_mon = date.month - 1;
    tm_buf->tm_year = date.year - 1900;

    return tm_buf;
}
#endif

#if defined ( __GNUC__ )
#if defined(MBEDTLS_HAVE_TIME) || defined(MBEDTLS_HAVE_TIME_DATE)
int _gettimeofday(struct timeval *tv, void *tzvp)
{
    (void)tzvp;

    rtc_datetime_t date;
    struct tm time_tm = {0};
    time_t time;

    /* Get date time */
    RTC_GetDatetime(RTC, &date);
    time_tm.tm_sec = date.second;
    time_tm.tm_min = date.minute;
    time_tm.tm_hour = date.hour;
    time_tm.tm_mday = date.day;
    time_tm.tm_mon = date.month - 1;
    time_tm.tm_year = date.year - 1900;

    time = mktime(&time_tm);

    tv->tv_sec = time;
    tv->tv_usec = 0;

    return 0;
}
#endif
#endif
#endif