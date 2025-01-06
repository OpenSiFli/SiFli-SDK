/**
  ******************************************************************************
  * @file   hr_service.h
  * @author Sifli software development team
  * @brief Sifli heart rate service interface
  * @{
  ******************************************************************************
*/
/*
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

#ifndef HEART_RATE_SERVICE_H
#define HEART_RATE_SERVICE_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "data_service_subscriber.h"

enum {
    MSG_SERVICE_HR_DAY_TABLE_REQ        = MSG_SERVICE_CUSTOM_ID_BEGIN,
    MSG_SERVICE_HR_DAY_TABLE_RSP        = MSG_SERVICE_HR_DAY_TABLE_REQ | RSP_MSG_TYPE,
    MSG_SERVICE_HR_MON_TABLE_REQ        = MSG_SERVICE_HR_DAY_TABLE_REQ + 1, 
    MSG_SERVICE_HR_MON_TABLE_RSP        = MSG_SERVICE_HR_MON_TABLE_REQ | RSP_MSG_TYPE,
    MSG_SERVICE_HR_REGION_REQ           = MSG_SERVICE_HR_DAY_TABLE_REQ + 2,
    MSG_SERVICE_HR_REGION_RSP           = MSG_SERVICE_HR_REGION_REQ | RSP_MSG_TYPE,
    MSG_SERVICE_HR_MAX_MIN_REQ          = MSG_SERVICE_HR_DAY_TABLE_REQ + 3,
    MSG_SERVICE_HR_MAX_MIN_RSP          = MSG_SERVICE_HR_MAX_MIN_REQ | RSP_MSG_TYPE,
    MSG_SERVICE_RHR_VALUE_REQ           = MSG_SERVICE_HR_DAY_TABLE_REQ + 4,
    MSG_SERVICE_RHR_VALUE_RSP           = MSG_SERVICE_RHR_VALUE_REQ | RSP_MSG_TYPE,

};

#define datas_push_day_table_to_client(svc,len,data)        datas_push_msg_to_client(svc,MSG_SERVICE_HR_DAY_TABLE_RSP,len,data)
#define datas_push_mon_table_to_client(svc,len,data)        datas_push_msg_to_client(svc,MSG_SERVICE_HR_MON_TABLE_RSP,len,data)
#define datas_push_region_to_client(svc,len,data)           datas_push_msg_to_client(svc,MSG_SERVICE_HR_REGION_RSP,len,data)
#define datas_push_maxmin_to_client(svc,len,data)           datas_push_msg_to_client(svc,MSG_SERVICE_HR_MAX_MIN_RSP,len,data)
#define datas_push_rhr_value_to_client(svc,len,data)        datas_push_msg_to_client(svc,MSG_SERVICE_RHR_VALUE_RSP,len,data)

#define HRS_DAY_TABLE_LEN           (24)
#define HRS_MON_TABLE_LEN           (30)
#define HRS_REGION_LEN              (5)
#define HRS_MAX_MIN_LEN             (3)
#define HRS_RHR_HIST_LEN            (3)


typedef struct 
{
    uint16_t today[24];
    uint16_t mon[30];
    uint8_t region[5];
    uint8_t max;
    uint8_t min;
    uint8_t rhr;
    uint8_t max_rhr;
    uint8_t min_rhr;
    uint8_t ave_rhr;
} custom_hr_data_table_t;

/// @} file

#endif  /*HEART_RATE_SERVICE_H*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
