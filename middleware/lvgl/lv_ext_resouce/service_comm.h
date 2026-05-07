/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 ******************************************************************************
 * @file   service_comm.h
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
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

#ifndef _SERVICE_COMM_H_
#define _SERVICE_COMM_H_

#include <stdbool.h>
#include "rtthread.h"
#include "data_service.h"
#include "string.h"

#if !defined(BSP_USING_PC_SIMULATOR)
    #include <sys/time.h>
#else
    #include <time.h>
#endif

#ifdef BSP_USING_PC_SIMULATOR
    #define __ARRAY_EMPTY_          1
#else
    #define __ARRAY_EMPTY_
#endif

/** @defgroup SIFLI_DEFAULT_DATE SiFli default date
  * @{
  */
#define SIFLI_DEFAULT_YEAR          (2019)
#define SIFLI_DEFAULT_MON           (1)
#define SIFLI_DEFAULT_DAY           (10)
#define SIFLI_DEFAULT_HOUR          (8)
#define SIFLI_DEFAULT_MIN           (0)
#define SIFLI_DEFAULT_SECOND        (0)
#define SIFLI_DEFAULT_TIMEZONE      (32)  /// 15 minutes increments between local time and UTC. Valid range is -48-56 and -128 means unknown.

/**
  * @}
  */

/**
 * @brief  The begin ID of the communication interface:
           between data_services,
           between processers,
           between HCPU and LCPU
 */
typedef enum
{
    LCPU_SERVICE_CUSTOM_ID_BEGIN            = MSG_SERVICE_CUSTOM_ID_BEGIN + 0x1000, /**< used between LCPU and HCPU     */
    BAT_SERVICE_CUSTOM_ID_BEGIN             = MSG_SERVICE_CUSTOM_ID_BEGIN + 0x2000, /**< used between BAT and APP       */
    SENSOR_SERVICE_CUSTOM_ID_BEGIN          = MSG_SERVICE_CUSTOM_ID_BEGIN + 0x3000, /**< used between SENSOR and APP    */
    PMIC_SERVICE_CUSTOM_ID_BEGIN            = MSG_SERVICE_CUSTOM_ID_BEGIN + 0x7000, /**< used between PMIC and APP      */
    BLE_SERVICE_CUSTOM_ID_BEGIN             = MSG_SERVICE_CUSTOM_ID_BEGIN + 0x8000, /**< used between BLE/BT and APP    */
    OTA_SERVICE_CUSTOM_ID_BEGIN             = MSG_SERVICE_CUSTOM_ID_BEGIN + 0x9000, /**< used between OTA and APP       */
    APP_POPUP_CUSTOM_ID_BEGIN               = MSG_SERVICE_CUSTOM_ID_BEGIN + 0xa000, /**< used between POPUP and APP     */
    USB_SERVICE_CUSTOM_ID_BEGIN             = MSG_SERVICE_CUSTOM_ID_BEGIN + 0xb000, /**< used between USB and APP     */
    TF_SERVICE_CUSTOM_ID_BEGIN              = MSG_SERVICE_CUSTOM_ID_BEGIN + 0xc000, /**< used between TF and APP     */
    SYS_CFG_SERVICE_CUSTOM_ID_BEGIN         = MSG_SERVICE_CUSTOM_ID_BEGIN + 0xc100, /**< used between lcpu sys cfg and APP */
} ds_type_t;

/**
 * @brief  The primitive of the communication interface: between processers
 */
typedef struct
{
    void            *cb;                    /**< primitive callback when received in receive process    */
    uint32_t         user_data;             /**< user data of primitive                                 */
    uint32_t         data_len       : 30;   /**< data length of body which will send to another process */
    uint32_t         reserved       : 1;    /**< reserved for future.                                   */
    uint32_t         discardable    : 1;    /**< if queue full, this message can be discarded           */
    void            *data;                  /**< body which will send to another processer              */
} comm_msg_t;

/**
 * @brief  The function prototype of primitive callback.
 */
typedef int (* comm_msg_cb_t)(comm_msg_t *msg);

/**
 * @brief  The time stucture, used for @ref service_current_time_get
 */
typedef struct
{
    uint16_t        year;
    uint8_t         month;
    uint8_t         day;
    uint8_t         hour;
    uint8_t         min;
    uint8_t         second;
    uint16_t        ms;
    uint8_t         wday;
} time_data_t;

/**
 * @brief  The time stucture, used for @ref service_sys_time_set
*/
typedef struct
{
    uint16_t        year;
    uint8_t         month;
    uint8_t         day;
    uint8_t         hour;
    uint8_t         min;
    uint8_t         second;
    uint8_t         hour_mode;
    int8_t          zone;       // 15 minutes increments between local time and UTC. Valid range is -48-56 and -128 means unknown.
    uint8_t         dst_offset; // DaySaving Time. 0: Standard time. 2: +0.5h. 4: +1h. 8: +2h. 255: unknown. Others are reserved.
} sys_time_t;

#if defined(BSP_USING_PC_SIMULATOR)
struct timezone
{
    int tz_minuteswest;   /* minutes west of Greenwich */
    int tz_dsttime;       /* type of dst correction */
};
#endif

#define service_get_elapse_ms(cur, pre) ((UINT32_MAX - pre + 1 + cur) & (UINT32_MAX))

/**
 * @brief  Generate a Message used to inter-processer communication.
 * @param  payload Message content which use to communicaiton
 * @param  data_len Message length
 * @param  user_data Message's user_data
 * @retval Pointer Message pointer with return type of comm_msg_t
 */
void            *service_msg_fill(void *payload, uint32_t data_len, void *cb, uint32_t user_data);

/**
 * @brief  Release a msg which applied by service_msg_fill.
 * @param  msg  Message which applied by service_msg_fill
 */
void             service_msg_rel(comm_msg_t *msg);

/**
 * @brief  Get current timezone.
 * @retval Pointer Timezone pointer
 */
struct timezone *service_timezone_get(void);

/**
 * @brief  Set current timezone.
 * @param  tz Current timezone
 */
void             service_timezone_set(struct timezone *tz);

/**
 * @brief  Get timestamp from timezone.
 * @param  uint32_t timestamp
 */
uint32_t         service_timezone_get_timestamp(int tz_minutes);
// get local timestamp
uint32_t         service_timestamp_get(void);

/**
 * @brief  Set system time.
 * @param  cur_time Current time
 */
void             service_sys_time_set(sys_time_t *cur_time);

/**
 * @brief  Set system timestamp.
 * @param  timestamp Current timestamp
 */
void             service_sys_timestamp_set(time_t timestamp);

/**
 * @brief  Get tick, unit is millisecond.
 * @retval Pointer Message pointer with return type of comm_msg_t
 */
uint32_t         service_current_ms_get(void);

/**
 * @brief  Get current time.
 * @param  ms_need Is need millisecond
 * @retval Pointer Current time with time_data_t's type
 */
time_data_t     *service_current_time_get(bool ms_need);

/**
 * @brief  Intial time when power_on /awake from sleep /reset_system_time.
 */
int              service_time_init(void);

/**
 * @brief  Enable watchdog or not.
 * @param  enable Enable or not
 * @retval Pointer Current time with time_data_t's type
 */
void            service_watchdog_enable_set(uint8_t enable);

/**
 * @brief  Pet watchdog.
 */
void            service_watchdog_pet(void);

#if defined(BSP_USING_PC_SIMULATOR)
    extern time_t       time(time_t *raw_time);
    extern time_t       mktime(struct tm *const tm);
    extern struct tm   *localtime(const time_t *t);
#endif

/**
 * @brief  Calculate the CRC32 value of a memory buffer.
 * @param  Crc accumulated CRC32 value, must be 0 on first call
 * @param  Buf buffer to calculate CRC32 value for
 * @param  Size bytes in buffer
 * @retval crc_value Calculated CRC32 value
 */
uint32_t calc_crc32(uint32_t crc, const void *buf, uint32_t size);

#endif //_SERVICE_COMM_H_

