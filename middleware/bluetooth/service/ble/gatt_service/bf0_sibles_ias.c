/**
  ******************************************************************************
  * @file   bf0_sibles_ias.c
  * @author Sifli software development team
  * @brief Header file - Sibles BLE IAS implmentation.
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
#include "bf0_sibles_ias.h"

#define LOG_TAG "ias_srv"
#include "log.h"


/// IAS Handles offsets
enum
{
    IAS_IDX_SVC,

    IAS_IDX_ALERT_LVL_CHAR,
    IAS_IDX_ALERT_LVL_VAL,

    IAS_IDX_NB,
};

typedef enum
{
    IAS_STATE_IDLE,
    IAS_STATE_READY
} ble_ias_state_t;


/// Full IAS Database Description - Used to add attributes into the database
const struct attm_desc ias_att_db[IAS_IDX_NB] =
{
    // Immediate Alert Service Declaration
    [IAS_IDX_SVC]                   =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},
    // Alert Level Characteristic Declaration
    [IAS_IDX_ALERT_LVL_CHAR]        =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Alert Level Characteristic Value
    [IAS_IDX_ALERT_LVL_VAL]         =   {ATT_CHAR_ALERT_LEVEL, PERM(WRITE_COMMAND, ENABLE), PERM(RI, ENABLE), sizeof(uint8_t)},
};


#define MAX_IAS_USER 3
typedef struct
{
    ble_ias_state_t state;
    ble_ias_callback callback[MAX_IAS_USER];
    sibles_hdl handle;
} ble_ias_env_t;


static ble_ias_env_t g_ias_env;


static ble_ias_env_t *ble_ias_get_env(void)
{
    return &g_ias_env;
}

static uint8_t ble_ias_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    ble_ias_env_t *env = ble_ias_get_env();
    switch (para->idx)
    {
    case IAS_IDX_ALERT_LVL_VAL:
    {
        uint32_t i;
        for (i = 0; i < MAX_IAS_USER; i++)
        {
            LOG_I("callback %x\r\n", env->callback[i]);
            if (env->callback[i])
                env->callback[i](conn_idx, *(para->value));
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

static int8_t ble_ias_register_callback(ble_ias_callback callback)
{
    int8_t ret = 0;
    uint32_t i;
    ble_ias_env_t *env = ble_ias_get_env();
    for (i = 0; i < MAX_IAS_USER; i++)
    {
        if (env->callback[i] == NULL)
        {
            env->callback[i] = callback;
            break;
        }
    }
    if (i == MAX_IAS_USER)
        ret = -1;
    return ret;
}

int8_t ble_ias_init(ble_ias_callback callback)
{
    ble_ias_env_t *env = ble_ias_get_env();

    if (env->state == IAS_STATE_IDLE)
    {
        sibles_register_svc_t svc;

        svc.att_db = (struct attm_desc *)&ias_att_db;
        svc.num_entry = IAS_IDX_NB;
        svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH);
        svc.uuid = ATT_SVC_IMMEDIATE_ALERT;
        env->handle = sibles_register_svc(&svc);
        if (env->handle)
            sibles_register_cbk(env->handle, NULL, ble_ias_set_cbk);
        env->state = IAS_STATE_READY;
    }
    return ble_ias_register_callback(callback);
}



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
