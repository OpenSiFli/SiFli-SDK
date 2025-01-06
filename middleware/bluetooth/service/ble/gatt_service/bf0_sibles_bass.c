/**
  ******************************************************************************
  * @file   bf0_sibles_bass.c
  * @author Sifli software development team
  * @brief Header file - Sibles BLE BAS service implmentation.
 *
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

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles.h"
#include "bf0_ble_bass.h"

#define LOG_TAG "ble_bass"
#include "log.h"


/// Battery Service Attributes Indexes
enum
{
    BAS_IDX_SVC,

    BAS_IDX_BATT_LVL_CHAR,
    BAS_IDX_BATT_LVL_VAL,
    BAS_IDX_BATT_LVL_NTF_CFG,

    BAS_IDX_NB,
};

typedef enum
{
    BASS_STATE_IDLE,
    BASS_STATE_READY,
    BASS_STATE_BUSY,
} ble_bass_state_t;



/// Full BAS Database Description - Used to add attributes into the database
const struct attm_desc bas_att_db[BAS_IDX_NB] =
{
    // Battery Service Declaration
    [BAS_IDX_SVC]                  =   {ATT_DECL_PRIMARY_SERVICE,  PERM(RD, ENABLE), 0, 0},

    // Battery Level Characteristic Declaration
    [BAS_IDX_BATT_LVL_CHAR]        =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    // Battery Level Characteristic Value
    [BAS_IDX_BATT_LVL_VAL]         =   {ATT_CHAR_BATTERY_LEVEL,    PERM(RD, ENABLE) | PERM(NTF, ENABLE), PERM(RI, ENABLE), 0},
    // Battery Level Characteristic - Client Characteristic Configuration Descriptor
    [BAS_IDX_BATT_LVL_NTF_CFG]     =   {ATT_DESC_CLIENT_CHAR_CFG,  PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},
};


typedef struct
{
    ble_bass_callback callback;
    sibles_hdl handle;
    uint8_t state;
    uint8_t cccd_enable;
    uint8_t bas_lvl;
} ble_bass_env_t;

static ble_bass_env_t g_bass_env_t;

static ble_bass_env_t *ble_bass_get_env(void)
{
    return &g_bass_env_t;
}

static uint8_t *ble_bass_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    ble_bass_env_t *env = ble_bass_get_env();
    switch (idx)
    {
    case BAS_IDX_BATT_LVL_VAL:
    {
        *len = sizeof(uint8_t);
        if (env->callback)
            env->bas_lvl = env->callback(conn_idx, BLE_BASS_GET_BATTERY_LVL);
        LOG_I("battery lvl %d", env->bas_lvl);
        return &env->bas_lvl;
        break;
    }
    default:
        break;
    }
    *len = 0;
    return NULL;
}

static uint8_t ble_bass_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    ble_bass_env_t *env = ble_bass_get_env();
    switch (para->idx)
    {
    case BAS_IDX_BATT_LVL_NTF_CFG:
    {
        LOG_I("bas enable %d", *(para->value));
        env->cccd_enable = *(para->value);
        break;
    }
    default:
        break;
    }
    return 0;
}

int8_t ble_bass_notify_battery_lvl(uint8_t conn_idx, uint8_t lvl)
{
    ble_bass_env_t *env = ble_bass_get_env();
    uint8_t ret = -1;
    if (env->state == BASS_STATE_READY)
    {
        if (env->bas_lvl != lvl)
        {
            env->bas_lvl = lvl;
            sibles_value_t value;
            value.hdl = env->handle;
            value.idx = BAS_IDX_BATT_LVL_VAL;
            value.len = sizeof(uint8_t);
            value.value = &env->bas_lvl;
            int ret = sibles_write_value(conn_idx, &value);
            ret = 0;
        }
        ret = -2;
    }
    return ret;
}



void ble_bass_init(ble_bass_callback callback, uint8_t battery_lvl)
{
    ble_bass_env_t *env = ble_bass_get_env();

    if (env->state == BASS_STATE_IDLE)
    {
        sibles_register_svc_t svc;

        svc.att_db = (struct attm_desc *)&bas_att_db;
        svc.num_entry = BAS_IDX_NB;
        svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH);
        svc.uuid = ATT_SVC_BATTERY_SERVICE;
        env->handle = sibles_register_svc(&svc);
        if (env->handle)
            sibles_register_cbk(env->handle, ble_bass_get_cbk, ble_bass_set_cbk);
        env->state = BASS_STATE_READY;
    }
    env->bas_lvl = battery_lvl;
    env->callback = callback;
}



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
