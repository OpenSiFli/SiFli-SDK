/**
  ******************************************************************************
  * @file   bf0_sibles_diss.c
  * @author Sifli software development team
  * @brief  Sibles device information service server.
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
#include "bf0_sibles_diss.h"
#ifdef BSP_BLE_DISS

#define LOG_TAG "ble_dis"
#include "log.h"

/// Device Information Service Attributes Characteristics
enum
{
    /// Device Information
    DISS_SVC,
    /// Manufacturer Name
    DISS_MANUFACTURER_NAME_CHAR,
    DISS_MANUFACTURER_NAME_VAL,
    /// Model Number
    DISS_MODEL_NB_STR_CHAR,
    DISS_MODEL_NB_STR_VAL,
    /// Serial Number
    DISS_SERIAL_NB_STR_CHAR,
    DISS_SERIAL_NB_STR_VAL,
    /// HW Revision Number
    DISS_HARD_REV_STR_CHAR,
    DISS_HARD_REV_STR_VAL,
    /// FW Revision Number
    DISS_FIRM_REV_STR_CHAR,
    DISS_FIRM_REV_STR_VAL,
    /// SW Revision Number
    DISS_SW_REV_STR_CHAR,
    DISS_SW_REV_STR_VAL,
    /// System Identifier Name
    DISS_SYSTEM_ID_CHAR,
    DISS_SYSTEM_ID_VAL,
    /// IEEE Certificate
    DISS_IEEE_CHAR,
    DISS_IEEE_VAL,
    /// Plug and Play Identifier
    DISS_PNP_ID_CHAR,
    DISS_PNP_ID_VAL,

    DISS_NB,
};

/// Full DIS Database Description - Used to add attributes into the database
const struct attm_desc dis_att_db[DISS_NB] =
{
    // Device Information Service Declaration
    [DISS_SVC]                           =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},

    // Manufacturer Name Characteristic Declaration
    [DISS_MANUFACTURER_NAME_CHAR]        =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Manufacturer Name Characteristic Value
    [DISS_MANUFACTURER_NAME_VAL]         =   {ATT_CHAR_MANUF_NAME, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_VAL_MAX_LEN},

    // Model Number String Characteristic Declaration
    [DISS_MODEL_NB_STR_CHAR]             =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Model Number String Characteristic Value
    [DISS_MODEL_NB_STR_VAL]              =   {ATT_CHAR_MODEL_NB, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_VAL_MAX_LEN},

    // Serial Number String Characteristic Declaration
    [DISS_SERIAL_NB_STR_CHAR]            =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Serial Number String Characteristic Value
    [DISS_SERIAL_NB_STR_VAL]             =   {ATT_CHAR_SERIAL_NB, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_VAL_MAX_LEN},

    // Hardware Revision String Characteristic Declaration
    [DISS_HARD_REV_STR_CHAR]             =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Hardware Revision String Characteristic Value
    [DISS_HARD_REV_STR_VAL]              =   {ATT_CHAR_HW_REV, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_VAL_MAX_LEN},

    // Firmware Revision String Characteristic Declaration
    [DISS_FIRM_REV_STR_CHAR]             =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Firmware Revision String Characteristic Value
    [DISS_FIRM_REV_STR_VAL]              =   {ATT_CHAR_FW_REV, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_VAL_MAX_LEN},

    // Software Revision String Characteristic Declaration
    [DISS_SW_REV_STR_CHAR]               =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // Software Revision String Characteristic Value
    [DISS_SW_REV_STR_VAL]                =   {ATT_CHAR_SW_REV, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_VAL_MAX_LEN},

    // System ID Characteristic Declaration
    [DISS_SYSTEM_ID_CHAR]                =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // System ID Characteristic Value
    [DISS_SYSTEM_ID_VAL]                 =   {ATT_CHAR_SYS_ID, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_SYSTEM_ID_LEN},

    // IEEE 11073-20601 Regulatory Certification Data List Characteristic Declaration
    [DISS_IEEE_CHAR]                     =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // IEEE 11073-20601 Regulatory Certification Data List Characteristic Value
    [DISS_IEEE_VAL]                      =   {ATT_CHAR_IEEE_CERTIF, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_SYSTEM_ID_LEN},

    // PnP ID Characteristic Declaration
    [DISS_PNP_ID_CHAR]                   =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // PnP ID Characteristic Value
    [DISS_PNP_ID_VAL]                    =   {ATT_CHAR_PNP_ID, PERM(RD, ENABLE), PERM(RI, ENABLE), SIBLES_DIS_PNP_ID_LEN},
};

typedef struct
{
    ble_dis_callback callback;
    sibles_hdl handle;
} ble_dis_env_t;


static ble_dis_env_t ble_dis_env;

static ble_dis_env_t *ble_dis_get_env(void)
{
    return &ble_dis_env;
};

static uint8_t *ble_dis_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    ble_dis_env_t *env = ble_dis_get_env();
    uint8_t *data = NULL;

    switch (idx)
    {
    case DISS_MANUFACTURER_NAME_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_MANU_NAME, len);
        }
        else
        {
            *len = SIBLES_DIS_MANUFACTURER_NAME_LEN;
            data = (uint8_t *)SIBLES_DIS_MANUFACTURER_NAME;
        }
        break;
    }
    case DISS_MODEL_NB_STR_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_MODEL_NB, len);
        }
        else
        {
            *len = SIBLES_DIS_MODEL_NB_STR_LEN;
            data = (uint8_t *)SIBLES_DIS_MODEL_NB_STR;
        }
        break;
    }
    case DISS_SERIAL_NB_STR_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_SERI_NB, len);
        }
        else
        {
            *len = SIBLES_DIS_SERIAL_NB_STR_LEN;
            data = (uint8_t *)SIBLES_DIS_SERIAL_NB_STR;
        }
        break;
    }
    case DISS_HARD_REV_STR_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_HW_REV, len);
        }
        else
        {
            *len = SIBLES_DIS_HARD_REV_STR_LEN;
            data = (uint8_t *)SIBLES_DIS_HARD_REV_STR;
        }
        break;
    }
    case DISS_FIRM_REV_STR_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_FW_REV, len);
        }
        else
        {
            *len = SIBLES_DIS_FIRM_REV_STR_LEN;
            data = (uint8_t *)SIBLES_DIS_FIRM_REV_STR;
        }
        break;
    }
    case DISS_SW_REV_STR_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_SW_REV, len);
        }
        else
        {
            *len = SIBLES_DIS_SW_REV_STR_LEN;
            data = (uint8_t *)SIBLES_DIS_SW_REV_STR;
        }
        break;
    }
    case DISS_SYSTEM_ID_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_SYS_ID, len);
        }
        else
        {
            *len = SIBLES_DIS_SYSTEM_ID_LEN;
            data = (uint8_t *)SIBLES_DIS_SYSTEM_ID;
        }
        break;
    }
    case DISS_IEEE_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_IEEE_DATA, len);
        }
        else
        {
            *len = SIBLES_DIS_IEEE_LEN;
            data = (uint8_t *)SIBLES_DIS_IEEE;
        }
        break;
    }
    case DISS_PNP_ID_VAL:
    {
        if (env->callback)
        {
            data = env->callback(conn_idx, BLE_DIS_GET_PNP_ID, len);
        }
        else
        {
            *len = SIBLES_DIS_PNP_ID_LEN;
            data = (uint8_t *)SIBLES_DIS_PNP_ID;
        }
    }
    default:
        *len = 0;
        data = NULL;
        break;
    }

    return data;
}

static uint8_t ble_dis_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    switch (para->idx)
    {
    default:
        break;
    }
    return 0;
}

void sibles_ble_diss_init(ble_dis_callback callback)
{
    ble_dis_env_t *env = ble_dis_get_env();

    sibles_register_svc_t svc;
    svc.att_db = (struct attm_desc *)&dis_att_db;
    svc.num_entry = DISS_NB;
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH);
    svc.uuid = ATT_SVC_DEVICE_INFO;
    env->handle = sibles_register_svc(&svc);
    if (env->handle)
        sibles_register_cbk(env->handle, ble_dis_get_cbk, ble_dis_set_cbk);

    env->callback = callback;
}

#endif