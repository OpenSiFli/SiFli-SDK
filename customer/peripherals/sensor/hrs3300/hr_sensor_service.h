/**
  ******************************************************************************
  * @file   hr_sensor_service.h
  * @author Sifli software development team
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

#ifndef _HR_SENSOR_SERVICE_H_
#define _HR_SENSOR_SERVICE_H_

#include "board.h"
#include "sensor.h"

#if defined (HR_ALGO_USING_GSENSOR_DATA)
    #include "gsensor_service.h"
#endif

#define HR_PERIOD_TIMER     40
#define HR_ALGO_PEROID      1000

typedef enum
{
    HEALTH_ALG_NOT_OPEN = 0x01,
    HEALTH_NO_TOUCH,
    HEALTH_PPG_LEN_TOO_SHORT,
    HEALTH_HR_READY,
    HEALTH_ALG_TIMEOUT,
    HEALTH_SETTLE
} hr_msg_code_t;

typedef enum
{
    MSG_ALG_NOT_OPEN = 0x01,
    MSG_NO_TOUCH = 0x02,
    MSG_PPG_LEN_TOO_SHORT = 0x03,
    MSG_HR_READY = 0x04,
    MSG_ALG_TIMEOUT = 0x05,
    MSG_SETTLE = 0x06
} hrs3300_msg_code_t;

typedef enum
{
    MSG_BP_ALG_NOT_OPEN = 0x01,
    MSG_BP_NO_TOUCH = 0x02,
    MSG_BP_PPG_LEN_TOO_SHORT = 0x03,
    MSG_BP_READY = 0x04,
    MSG_BP_ALG_TIMEOUT = 0x05,
    MSG_BP_SETTLE = 0x06
} hrs3300_bp_msg_code_t;

typedef struct
{
    hrs3300_msg_code_t alg_status;
    uint32_t           data_cnt;
    uint8_t            hr_result;
    uint8_t            hr_result_qual; // ericy add20170111
    bool               object_flg;
    uint32_t           timestamp;
} hr_algo_result_t;

typedef struct
{
    hrs3300_bp_msg_code_t bp_alg_status;
    uint8_t            sbp;
    uint8_t            dbp;
    uint32_t           data_cnt;
    uint8_t            hr_result; //20170614 ericy
    bool               object_flg;
    uint32_t           timestamp;
} bp_algo_result_t;

typedef struct
{
    uint8_t hr_id;
    uint8_t type;
} hr_sensor_info_t;

/* hr device structure */
struct hr_sensor_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
};

typedef struct
{
    uint16_t hrm_raw;
    uint16_t alg_raw;
} hr_raw_data_t;


#endif  // SENSOR_GOODIX_GH3011_H__
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
