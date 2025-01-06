/**
  ******************************************************************************
  * @file   bf0_sibles_tips.c
  * @author Sifli software development team
  * @brief  Sibles BLE time profile server implmentation.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
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

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles.h"
#include "bf0_ble_tips.h"

#define LOG_TAG "ble_tips"
#include "log.h"

#ifdef BSP_BLE_TIMES

//current time octets is 10
#define CURRENT_TIME_OCTETS_SIZE 10

/// time Service Attributes Indexes
enum
{
    TIP_IDX_SVC,
    TIP_IDX_CUR_TIME_CHAR,
    TIP_IDX_CUR_TIME_VAL,
    TIP_IDX_CUR_TIME_NTF_CFG,
    TIP_IDX_LOC_TIME_INF_CHAR,
    TIP_IDX_LOC_TIME_INF_VAL,
    TIP_IDX_REF_TIME_INF_CHAR,
    TIP_IDX_REF_TIME_INF_VAL,
    TIP_IDX_NB,
};

typedef enum
{
    TIPS_STATE_IDLE,
    TIPS_STATE_READY,
    TIPS_STATE_BUSY,
} ble_tips_state_t;


/// Full Time server Database Description - Used to add attributes into the database
const struct attm_desc tip_att_db[TIP_IDX_NB] =
{
    // Time Service Declaration
    [TIP_IDX_SVC]                  =   {ATT_DECL_PRIMARY_SERVICE,  PERM(RD, ENABLE), 0, 0},

    // Current time Characteristic Declaration
    [TIP_IDX_CUR_TIME_CHAR]        =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    // Current time Characteristic Value
    [TIP_IDX_CUR_TIME_VAL]         =   {ATT_CHAR_CT_TIME,    PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(NTF, ENABLE), PERM(RI, ENABLE), CURRENT_TIME_OCTETS_SIZE},
    // Current time Characteristic - Client Characteristic Configuration Descriptor
    [TIP_IDX_CUR_TIME_NTF_CFG]     =   {ATT_DESC_CLIENT_CHAR_CFG,  PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), 2},

    // Local time information Characteristic Declaration
    [TIP_IDX_LOC_TIME_INF_CHAR]        =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    // Local time information Characteristic Value
    [TIP_IDX_LOC_TIME_INF_VAL]         =   {ATT_CHAR_LOCAL_TIME_INFO,    PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), sizeof(ble_tips_local_time_info_t)},

    // Reference time information Characteristic Declaration
    [TIP_IDX_REF_TIME_INF_CHAR]        =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    // Reference time information Characteristic Value
    [TIP_IDX_REF_TIME_INF_VAL]         =   {ATT_CHAR_REFERENCE_TIME_INFO,    PERM(RD, ENABLE),  PERM(RI, ENABLE), sizeof(ble_tips_ref_time_info_t)},
};


typedef struct
{
    ble_tips_callback callback;
    sibles_hdl handle;
    uint8_t state;
    uint16_t cccd_enable;
    ble_tips_time_env_t time_env;
} ble_tips_env_t;

static uint8_t char_stream[CURRENT_TIME_OCTETS_SIZE];

static ble_tips_env_t g_tips_env_t;

static ble_tips_env_t *ble_tips_get_env(void)
{
    return &g_tips_env_t;
}

static uint16_t co_read16p(void const *ptr16)
{
    uint16_t value = ((uint8_t const volatile *)ptr16)[0] | ((uint8_t const volatile *)ptr16)[1] << 8;
    return value;
}

static void co_write16p(void const *ptr16, uint16_t value)
{
    uint8_t volatile *ptr = (uint8_t *)ptr16;

    *ptr++ = value & 0xff;
    *ptr = (value & 0xff00) >> 8;
}

static uint8_t *cur_time_struct2char_stream(ble_tips_cur_time_t cur_time)
{
    uint8_t cursor = 0;
    co_write16p(char_stream, cur_time.date_time.year);
    cursor += 2;
    char_stream[cursor++] = cur_time.date_time.month;
    char_stream[cursor++] = cur_time.date_time.day;
    char_stream[cursor++] = cur_time.date_time.hour;
    char_stream[cursor++] = cur_time.date_time.min;
    char_stream[cursor++] = cur_time.date_time.sec;
    char_stream[cursor++] = cur_time.day_of_week;
    char_stream[cursor++] = cur_time.fraction_256;
    char_stream[cursor++] = cur_time.adjust_reason;
    return char_stream;
}

static void char_stream2cur_time_struct(uint8_t *value, ble_tips_cur_time_t *cur_time)
{
    uint8_t cursor = 0;
    cur_time->date_time.year = co_read16p(value);
    cursor += 2;
    cur_time->date_time.month = value[cursor++];
    cur_time->date_time.day = value[cursor++];
    cur_time->date_time.hour = value[cursor++];
    cur_time->date_time.min = value[cursor++];
    cur_time->date_time.sec = value[cursor++];
    cur_time->day_of_week = value[cursor++];
    cur_time->fraction_256 = value[cursor++];
    cur_time->adjust_reason = value[cursor++];
}

static uint8_t *ble_tips_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    ble_tips_env_t *env = ble_tips_get_env();
    switch (idx)
    {
    case TIP_IDX_CUR_TIME_VAL:
    {
        *len = CURRENT_TIME_OCTETS_SIZE;
        if (env->callback)
            env->time_env.cur_time = *(ble_tips_cur_time_t *)env->callback(conn_idx, BLE_TIPS_GET_CURRENT_TIME, NULL);
        return cur_time_struct2char_stream(env->time_env.cur_time);
        break;
    }
    case TIP_IDX_CUR_TIME_NTF_CFG:
    {
        *len = 2;
        return (uint8_t *)&env->cccd_enable;
        break;
    }
    case TIP_IDX_LOC_TIME_INF_VAL:
    {
        *len = sizeof(ble_tips_local_time_info_t);
        if (env->callback)
            env->time_env.local_time_inf = *(ble_tips_local_time_info_t *)env->callback(conn_idx, BLE_TIPS_GET_LOCAL_TIME_INFO, NULL);
        return (uint8_t *)&env->time_env.local_time_inf;
        break;
    }
    case TIP_IDX_REF_TIME_INF_VAL:
    {
        *len = sizeof(ble_tips_ref_time_info_t);
        if (env->callback)
            env->time_env.ref_time_inf = *(ble_tips_ref_time_info_t *)env->callback(conn_idx, BLE_TIPS_GET_REF_TIME_INFO, NULL);
        return (uint8_t *)&env->time_env.ref_time_inf;
        break;
    }
    default:
        break;
    }
    *len = 0;
    return NULL;
}

static uint8_t ble_tips_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    ble_tips_env_t *env = ble_tips_get_env();
    switch (para->idx)
    {
    case TIP_IDX_CUR_TIME_VAL:
    {
        char_stream2cur_time_struct(para->value, &env->time_env.cur_time);
        if (env->callback)
            env->callback(conn_idx, BLE_TIPS_SET_CURRENT_TIME, (uint8_t *)&env->time_env.cur_time);
        break;
    }
    case TIP_IDX_CUR_TIME_NTF_CFG:
    {
        env->cccd_enable = co_read16p(para->value);
        break;
    }
    case TIP_IDX_LOC_TIME_INF_VAL:
    {
        env->time_env.local_time_inf = *(ble_tips_local_time_info_t *)(para->value);
        if (env->callback)
            env->callback(conn_idx, BLE_TIPS_SET_LOCAL_TIME_INFO, (uint8_t *)&env->time_env.local_time_inf);
        break;
    }
    default:
        break;
    }
    return 0;
}

static uint8_t cur_time_cmp(ble_tips_cur_time_t cur_time1, ble_tips_cur_time_t cur_time2)
{
    uint8_t res = 1;
    do
    {
        if (cur_time1.date_time.year != cur_time2.date_time.year) break;
        if (cur_time1.date_time.month != cur_time2.date_time.month) break;
        if (cur_time1.date_time.day != cur_time2.date_time.day) break;
        if (cur_time1.date_time.hour != cur_time2.date_time.hour) break;
        if (cur_time1.date_time.min != cur_time2.date_time.min) break;
        if (cur_time1.date_time.sec != cur_time2.date_time.sec) break;
        if (cur_time1.day_of_week != cur_time2.day_of_week) break;
        if (cur_time1.fraction_256 != cur_time2.fraction_256) break;
        if (cur_time1.adjust_reason != cur_time2.adjust_reason) break;
        res = 0;
    }
    while (0);
    return res;
}


int8_t ble_tips_notify_current_time(uint8_t conn_idx, ble_tips_cur_time_t cur_time)
{
    ble_tips_env_t *env = ble_tips_get_env();
    uint8_t ret = -1;
    if (env->state == TIPS_STATE_READY && env->cccd_enable == 1)
    {
        ret = -2;
        if (cur_time_cmp(env->time_env.cur_time, cur_time) == 1)
        {
            env->time_env.cur_time = cur_time;
            sibles_value_t value;
            value.hdl = env->handle;
            value.idx = TIP_IDX_CUR_TIME_VAL;
            value.len = CURRENT_TIME_OCTETS_SIZE;
            value.value = cur_time_struct2char_stream(env->time_env.cur_time);
            int ret = sibles_write_value(conn_idx, &value);
            ret = 0;
        }
    }
    return ret;
}

void ble_tips_init(ble_tips_callback callback, ble_tips_time_env_t time_env)
{
    ble_tips_env_t *env = ble_tips_get_env();

    if (env->state == TIPS_STATE_IDLE)
    {
        sibles_register_svc_t svc;

        svc.att_db = (struct attm_desc *)&tip_att_db;
        svc.num_entry = TIP_IDX_NB;
        svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH);
        svc.uuid = ATT_SVC_CURRENT_TIME;
        env->handle = sibles_register_svc(&svc);
        if (env->handle)
            sibles_register_cbk(env->handle, ble_tips_get_cbk, ble_tips_set_cbk);
        env->state = TIPS_STATE_READY;
    }
    env->callback = callback;
    env->time_env = time_env;
}


#endif //BSP_BLE_TIMES


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
