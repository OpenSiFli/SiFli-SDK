/*
 *  Copyright 2008-2020, 2024 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

/** @file wmtime.h
 *
 *  @brief This file provides the time management APIs.
 *
 * Time Management Subsystem, Internally wireless
 * microcontroller manages time in terms of seconds since epoch (1 Jan 1970
 * 00:00). This representation is known as "posix time". These routines can
 * be used to do conversions between human readable time and posix time as well
 * as to set the system time to desired value.
 *
 */

#ifndef _WMTIME_H_
#define _WMTIME_H_

#include <wmtypes.h>
#include <time.h>
#include <wm_net.h>

/** Structure is used to hold the date and time */
struct datetime_t
{
    uint16_t year;  /*!< Range from 1970 to 2099.*/
    uint8_t month;  /*!< Range from 1 to 12.*/
    uint8_t day;    /*!< Range from 1 to 31 (depending on month).*/
    uint8_t hour;   /*!< Range from 0 to 23.*/
    uint8_t minute; /*!< Range from 0 to 59.*/
    uint8_t second; /*!< Range from 0 to 59.*/
};

typedef long suseconds_t;

struct rtc_cb_t
{
    uint32_t base;
    int initialized;
    int (*set_datetime)(void *base, struct datetime_t *datetime);
    void (*get_datetime)(void *base, struct datetime_t *datetime);
};

extern void wmtime_register_rtc_cb(struct rtc_cb_t *rtc_cb);

/** Convert HTTP date format to POSIX time format
 *
 * \param[in] date HTTP date format
 *
 * \return success or failure as:
 *     -WM_FAIL: Conversion failed. Invalid format/data
 *     else valid time_t value
 */
time_t http_date_to_time(const unsigned char *date);

/** Set the date and time
 *
 * \param[in] tm The rtc value is updated with the values in tm structure
 * \return success or failure as:
 *     0: Success
 *     -1: Failed validation of tm structure
 */
extern int wmtime_time_set(const struct tm *tm);

/**
 * Get date and time
 *
 * \param[out] tm tm structure is updated to get the current value in rtc
 * \return success or failure as:
 *     0: Success
 *     non-zero: Internal error
 */
extern int wmtime_time_get(struct tm *tm);

/** Set the date and time using posix time
 *
 *  \param[in] time The rtc value is updated with the value present in time
 *  \return success or failure as:
 *      0: Success
 *      non-zero: Internal error
 */
extern int wmtime_time_set_posix(time_t time);

/**
 * Get date and time in posix format
 *
 *  \return time_t value from RTC
 */
extern time_t wmtime_time_get_posix(void);

/**
 * Convert to tm structure from POSIX/Unix time (Seconds since epoch)
 *
 * \param[in] time This is POSIX time that is to be converted into \ref tm
 * \param[out] result This should point to pre-allocated \ref tm instance
 * \return pointer to struct tm; NULL in case of error
 */
struct tm *gmtime_r(const time_t *time, struct tm *result);

/**
 * Converts to POSIX/Unix time from tm structure
 *
 * \param[in] tm This is \ref tm instance that is to be converted into
 * time_t format
 * \return time_t POSIX/Unix time equivalent
 */
extern time_t mktime(struct tm *tm);

/**
 * Converts the broken-down time value tm into a null-terminated string.
 *
 * \param[in] tm This is \ref tm instance that is to be converted string.
 *
 * @return Pointer to a statically allocated string which contains the date
 * and time format as follows "Tue Mar 24 09:20:14 2015". The statically
 * allocated string might be overwritten by subsequent calls to any of the
 * date and time functions.
 */
char *asctime(const struct tm *tm);

/**
 * Initialize time subsystem including RTC. Sets system time to 1/1/1970 00:00
 * (i.e. epoch 0)
 *
 * \return WM_SUCCESS on success, zero otherwise
 */
extern int wmtime_init(void);

/** Get current wmtime initialization status
 *
 *\return true if initialized, false if not.
 */
bool is_wmtime_init_done();

int gettimeofday(struct timeval *tv, void *tz);
#endif
