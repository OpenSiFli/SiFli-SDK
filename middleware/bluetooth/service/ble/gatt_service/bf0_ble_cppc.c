/**
  ******************************************************************************
  * @file   bf0_ble_cppc.c
  * @author Sifli software development team
  * @brief  Sibles Cycling Power Profile Collector.
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
#include "att.h"
#include "bf0_ble_gap.h"
#include "os_adaptor.h"
#include "bf0_ble_cppc.h"

#ifdef BSP_BLE_CPPC

#define LOG_TAG "cppc"
#include "log.h"


static uint8_t conn_idx = 0;
typedef struct
{
    uint16_t hdl_start;
    uint16_t hdl_end;
} ble_cppc_svc_t;

typedef struct
{
    uint16_t attr_hdl;
    uint16_t value_hdl;
    uint16_t cccd_hdl;
    uint16_t cepd_hdl;
    uint8_t prop;
    uint8_t enabled;
} ble_cppc_char_t;

enum ble_cppc_state_t
{
    BLE_CPPC_STATE_IDLE,
    BLE_CPPC_STATE_READY,
    BLE_CPPC_STATE_BUSY
};


typedef struct
{
    uint8_t state;
    uint8_t is_enable;
    uint8_t conn_idx;
    uint16_t remote_handle;
    ble_cppc_svc_t svc;
    ble_cppc_char_t cpf_char;
    ble_cppc_char_t cpm_char;
    ble_cppc_char_t sl_char;
    ble_cppc_char_t cpcp_char;
    ble_cppc_char_t cpv_char;
} ble_cppc_env_t;

static ble_cppc_env_t g_ble_cppc_env;
static ble_cppc_env_t *ble_cppc_get_env(void)
{
    return &g_ble_cppc_env;
};

static uint16_t co_read16p(void const *ptr16)
{
    uint16_t value = ((uint8_t const volatile *)ptr16)[0] | ((uint8_t const volatile *)ptr16)[1] << 8;
    return value;
}
static uint32_t co_read32p(void const *ptr32)
{
    uint16_t addr_l, addr_h;
    addr_l = co_read16p(ptr32);
    addr_h = co_read16p((uint8_t *)ptr32 + 2);
    return ((uint32_t)addr_l | (uint32_t)addr_h << 16);

}

static uint32_t co_read24p(void const *ptr24)
{
    uint16_t addr_l, addr_h;
    addr_l = co_read16p(ptr24);
    addr_h = *((uint8_t const volatile *)ptr24 + 2) & 0x00FF;
    return ((uint32_t)addr_l | (uint32_t)addr_h << 16);
}

static void co_write32p(void const *ptr32, uint32_t value)
{
    uint8_t volatile *ptr = (uint8_t *)ptr32;

    *ptr++ = (uint8_t)(value & 0xff);
    *ptr++ = (uint8_t)((value & 0xff00) >> 8);
    *ptr++ = (uint8_t)((value & 0xff0000) >> 16);
    *ptr = (uint8_t)((value & 0xff000000) >> 24);
}

static void co_write16p(void const *ptr16, uint16_t value)
{
    uint8_t volatile *ptr = (uint8_t *)ptr16;

    *ptr++ = value & 0xff;
    *ptr = (value & 0xff00) >> 8;
}

uint8_t prf_unpack_date_time(uint8_t *packed_date, struct prf_date_time *date_time)
{
    date_time->year = co_read16p(&(packed_date[0]));
    date_time->month = packed_date[2];
    date_time->day   = packed_date[3];
    date_time->hour  = packed_date[4];
    date_time->min   = packed_date[5];
    date_time->sec   = packed_date[6];

    return 7;
}

static uint8_t ble_cppc_unpack_cp_measure(uint8_t *packed_date, ble_cppc_cpm *date)
{
    date->flags = co_read16p(&packed_date[0]);
    date->inst_power = co_read16p(&packed_date[2]);
    uint8_t offset = CPP_CP_MEAS_NTF_MIN_LEN;
    if (date->flags & CPP_MEAS_PEDAL_POWER_BALANCE_PRESENT)
    {
        //Unpack Pedal Power Balance info
        date->pedal_power_balance = packed_date[offset];
        offset++;
    }
    if (date->flags & CPP_MEAS_ACCUM_TORQUE_PRESENT)
    {
        //Unpack Accumulated Torque info
        date->accum_torque = co_read16p(&packed_date[offset]);
        offset += 2;
    }

    if (date->flags & CPP_MEAS_WHEEL_REV_DATA_PRESENT)
    {
        //Unpack Wheel Revolution Data (Cumulative Wheel & Last Wheel Event Time)
        date->cumul_wheel_rev = co_read32p(&packed_date[offset]);
        offset += 4;
        date->last_wheel_evt_time = co_read16p(&packed_date[offset]);
        offset += 2;
    }

    if (date->flags & CPP_MEAS_CRANK_REV_DATA_PRESENT)
    {
        //Unpack Crank Revolution Data (Cumulative Crank & Last Crank Event Time)
        date->cumul_crank_rev = co_read16p(&packed_date[offset]);
        offset += 2;
        date->last_crank_evt_time = co_read16p(&packed_date[offset]);
        offset += 2;
    }

    if (date->flags & CPP_MEAS_EXTREME_FORCE_MAGNITUDES_PRESENT)
    {
        //Unpack Extreme Force Magnitudes (Maximum Force Magnitude & Minimum Force Magnitude)
        date->max_force_magnitude = co_read16p(&packed_date[offset]);
        offset += 2;
        date->min_force_magnitude = co_read16p(&packed_date[offset]);
        offset += 2;
    }

    else if (date->flags & CPP_MEAS_EXTREME_TORQUE_MAGNITUDES_PRESENT)
    {
        //Unpack Extreme Force Magnitudes (Maximum Force Magnitude & Minimum Force Magnitude)
        date->max_torque_magnitude = co_read16p(&packed_date[offset]);
        offset += 2;
        date->min_torque_magnitude = co_read16p(&packed_date[offset]);
        offset += 2;
    }

    if (date->flags & CPP_MEAS_EXTREME_ANGLES_PRESENT)
    {
        //Unpack Extreme Angles (Maximum Angle & Minimum Angle)
        uint32_t angle = co_read24p(&packed_date[offset]);
        offset += 3;

        //Force to 12 bits
        date->max_angle = (angle & (0x0FFF));
        date->min_angle = ((angle >> 12) & 0x0FFF);
    }

    if (date->flags & CPP_MEAS_TOP_DEAD_SPOT_ANGLE_PRESENT)
    {
        //Unpack Top Dead Spot Angle
        date->top_dead_spot_angle = co_read16p(&packed_date[offset]);
        offset += 2;
    }

    if (date->flags & CPP_MEAS_BOTTOM_DEAD_SPOT_ANGLE_PRESENT)
    {
        //Unpack Bottom Dead Spot Angle
        date->bot_dead_spot_angle = co_read16p(&packed_date[offset]);
        offset += 2;
    }

    if (date->flags & CPP_MEAS_ACCUM_ENERGY_PRESENT)
    {
        //Unpack Accumulated Energy
        date->accum_energy = co_read16p(&packed_date[32]);
        offset += 2;
    }

    return offset;

}
static uint8_t ble_cppc_unpack_cp_vector(uint8_t *packed_date, ble_cppc_cpv *date, uint16_t length)
{
    // Offset
    uint8_t offset = CPP_CP_VECTOR_MIN_LEN;
    // Flags

    date->flags = packed_date[0];

    if (date->flags & CPP_VECTOR_CRANK_REV_DATA_PRESENT)
    {
        // Unpack Crank Revolution Data

        date->cumul_crank_rev = co_read16p(&packed_date[offset]);
        offset += 2;
        // Unpack Last Crank Evt time

        date->last_crank_evt_time = co_read16p(&packed_date[offset]);
        offset += 2;
    }
    if (date->flags & CPP_VECTOR_FIRST_CRANK_MEAS_ANGLE_PRESENT)
    {
        // Unpack First Crank Measurement Angle

        date->first_crank_meas_angle = co_read16p(&packed_date[offset]);
        offset += 2;
    }
    if (!(date->flags & CPP_VECTOR_INST_FORCE_MAGNITUDE_ARRAY_PRESENT) !=
            !(date->flags & CPP_VECTOR_INST_TORQUE_MAGNITUDE_ARRAY_PRESENT))
    {
        // Unpack Force or Torque magnitude (mutually excluded)

        date->nb = (length - offset) / 2;

        if (date->nb)
        {
            for (int i = 0; i < date->nb; i++)
            {
                // Handle the array buffer to extract parameters

                date->force_torque_magnitude[i] = co_read16p(&packed_date[offset]);
                offset += 2;
            }
        }
    }
    return offset;

}


static void ble_cppc_cp_measure_hanlder(uint16_t event, uint8_t *data)
{
    ble_cppc_cpm cpm;
    ble_cppc_unpack_cp_measure(data, &cpm);
    ble_event_publish(event, &cpm, sizeof(ble_cppc_cpm));
}

static void ble_cppc_cp_vector_hanlder(uint16_t event, uint8_t *data, uint16_t length)
{
    ble_cppc_cpv cpv;
    ble_cppc_unpack_cp_vector(data, &cpv, length);
    ble_event_publish(event, &cpv, sizeof(ble_cppc_cpv));
}

int8_t ble_cppc_enable(uint8_t conn_idx)
{
    ble_cppc_env_t *env = ble_cppc_get_env();
    uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_CYCLING_POWER);
    int8_t ret = sibles_search_service(conn_idx, ATT_UUID_16_LEN, (uint8_t *)&svc_uuid);
    if (ret == 0)
        env->state = BLE_CPPC_STATE_BUSY;
    return ret;
}

int8_t ble_cppc_read_cpf(uint8_t conn_idx)
{
    int8_t ret;
    ble_cppc_env_t *env = ble_cppc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->cpf_char.value_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}


int8_t ble_cppc_read_sen_loca(uint8_t conn_idx)
{
    int8_t ret;
    ble_cppc_env_t *env = ble_cppc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;

    value.handle = env->sl_char.value_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}
int8_t ble_cppc_read_cpcp_cep(uint8_t conn_idx)
{
    int8_t ret;
    ble_cppc_env_t *env = ble_cppc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->cpcp_char.cepd_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;

}
static void ble_cppc_cpf_handler(uint8_t *data)
{
    ble_cppc_cpf cpf;
    cpf.sensor_feat = co_read32p(data);
    ble_event_publish(BLE_CPPC_READ_CPF_RSP, &cpf, sizeof(ble_cppc_cpf));
}

static void ble_cppc_sl_handler(uint8_t *data)
{
    ble_cppc_sl sl;
    sl.sensor_loc = *data;
    ble_event_publish(BLE_CPPC_READ_SL_RSP, &sl, sizeof(ble_cppc_sl));
}

static void ble_cppc_cep_handler(uint8_t *data)
{
    ble_cppc_cep cep;
    cep.cep_val = co_read16p(data);
    ble_event_publish(BLE_CPPC_READ_CPCP_CEP_RSP, &cep, sizeof(ble_cppc_cep));
}

static uint8_t ble_cppc_cppc_unpack(uint8_t *packed_date, ble_cpcp_notyf_rsp *date, uint16_t length)
{
    uint8_t offset = CPP_CP_CNTL_PT_RSP_MIN_LEN;

    // Requested operation code
    date->req_op_code = packed_date[1];

    // Response value

    date->resp_value = packed_date[2];

    if ((date->resp_value == CPP_CTNL_PT_RESP_SUCCESS) && (length >= 3))
    {
        switch (date->req_op_code)
        {
        case (CPP_CTNL_PT_REQ_SUPP_SENSOR_LOC):
        {
            // Get the number of supported locations that have been received
            uint8_t nb_supp_loc = (length - 3);
            // Location
            uint8_t loc;

            for (uint8_t counter = 0; counter < nb_supp_loc; counter++)
            {
                loc = packed_date[counter + CPP_CP_CNTL_PT_RSP_MIN_LEN];
                // Check if valid
                if (loc < CPP_LOC_MAX)
                {
                    date->value.supp_loc |= (1 << loc);

                }
                offset++;
            }
        }
        break;

        case (CPP_CTNL_PT_REQ_CRANK_LENGTH):
        {
            date->value.crank_length = co_read16p(&packed_date[offset]);
            offset += 2;
        }
        break;

        case (CPP_CTNL_PT_REQ_CHAIN_LENGTH):
        {


            date->value.chain_length = co_read16p(&packed_date[offset]);
            offset += 2;
        }
        break;

        case (CPP_CTNL_PT_REQ_CHAIN_WEIGHT):
        {

            date->value.chain_weight = co_read16p(&packed_date[offset]);
            offset += 2;
        }
        break;

        case (CPP_CTNL_PT_REQ_SPAN_LENGTH):
        {


            date->value.span_length = co_read16p(&packed_date[offset]);
            offset += 2;
        }
        break;

        case (CPP_CTNL_PT_START_OFFSET_COMP):
        {


            date->value.offset_comp = co_read16p(&packed_date[offset]);
            offset += 2;
        }
        break;

        case (CPP_CTNL_REQ_SAMPLING_RATE):
        {


            date->value.sampling_rate = packed_date[offset];
            offset++;
        }
        break;

        case (CPP_CTNL_REQ_FACTORY_CALIBRATION_DATE):
        {
            offset += prf_unpack_date_time(
                          (uint8_t *) & (packed_date[offset]), &(date->value.factory_calibration));
        }
        break;

        case (CPP_CTNL_PT_SET_CUMUL_VAL):
        case (CPP_CTNL_PT_UPD_SENSOR_LOC):
        case (CPP_CTNL_PT_SET_CRANK_LENGTH):
        case (CPP_CTNL_PT_SET_CHAIN_LENGTH):
        case (CPP_CTNL_PT_SET_CHAIN_WEIGHT):
        case (CPP_CTNL_PT_SET_SPAN_LENGTH):
        case (CPP_CTNL_MASK_CP_MEAS_CH_CONTENT):
        {
            // No parameters
        } break;

        default:
        {

        } break;
        }
    }
    return offset;
}

static void ble_cppc_cp_control_hanlder(uint16_t event, uint8_t *data, uint16_t length)
{
    ble_cpcp_notyf_rsp cpcp;

    ble_cppc_cppc_unpack(data, &cpcp, length);
    ble_event_publish(event, &cpcp, sizeof(ble_cpcp_notyf_rsp));
}

uint8_t ble_cppc_pack_ctnl_pt_req(ble_cppc_cpcp *param, uint8_t *req)
{
    // Request Length
    uint8_t req_len = CPP_CP_CNTL_PT_REQ_MIN_LEN;

    // Set the operation code
    req[0] = param->op_code;

    // Fulfill the message according to the operation code
    switch (param->op_code)
    {
    case (CPP_CTNL_PT_SET_CUMUL_VAL):
    {
        // Set the cumulative value
        co_write32p(&req[req_len], param->value.cumul_val);
        // Update length
        req_len += 4;
    }
    break;

    case (CPP_CTNL_PT_UPD_SENSOR_LOC):
    {
        // Set the sensor location
        req[req_len] = param->value.sensor_loc;
        // Update length
        req_len++;
    }
    break;

    case (CPP_CTNL_PT_SET_CRANK_LENGTH):
    {
        // Set the crank length
        co_write16p(&req[req_len], param->value.crank_length);
        // Update length
        req_len += 2;
    }
    break;

    case (CPP_CTNL_PT_SET_CHAIN_LENGTH):
    {
        // Set the chain length
        co_write16p(&req[req_len], param->value.chain_length);
        // Update length
        req_len += 2;
    }
    break;

    case (CPP_CTNL_PT_SET_CHAIN_WEIGHT):
    {
        // Set the chain weight
        co_write16p(&req[req_len], param->value.chain_weight);
        // Update length
        req_len += 2;
    }
    break;

    case (CPP_CTNL_PT_SET_SPAN_LENGTH):
    {
        // Set the span length
        co_write16p(&req[req_len], param->value.span_length);
        // Update length
        req_len += 2;
    }
    break;

    case (CPP_CTNL_MASK_CP_MEAS_CH_CONTENT):
    {
        // Set the Content Mask
        co_write16p(&req[req_len], param->value.mask_content);
        // Update length
        req_len += 2;
    }
    break;

    case (CPP_CTNL_PT_REQ_SUPP_SENSOR_LOC):
    case (CPP_CTNL_PT_REQ_CRANK_LENGTH):
    case (CPP_CTNL_PT_REQ_CHAIN_LENGTH):
    case (CPP_CTNL_PT_REQ_CHAIN_WEIGHT):
    case (CPP_CTNL_PT_REQ_SPAN_LENGTH):
    case (CPP_CTNL_PT_START_OFFSET_COMP):
    case (CPP_CTNL_REQ_SAMPLING_RATE):
    case (CPP_CTNL_REQ_FACTORY_CALIBRATION_DATE):
    {
        // Nothing more to do
    } break;

    default:
    {

    }
    break;
    }

    return req_len;
}

void ble_cppc_write_cpcp_val_handler(ble_cppc_cpcp *param)
{
    int8_t res;
    ble_cppc_env_t *env = ble_cppc_get_env();
    sibles_write_remote_value_t value;
    value.handle = env->cpcp_char.value_hdl;
    value.write_type = SIBLES_WRITE;
    value.value = malloc(8);
    value.len = ble_cppc_pack_ctnl_pt_req(param, value.value);
    res = sibles_write_remote_value(env->remote_handle, conn_idx, &value);
    free(value.value);
    OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
}

void ble_cppc_config(int argc, char *argv[])
{
    ble_cppc_env_t *env = ble_cppc_get_env();
    if (strcmp(argv[1], "scv") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_SET_CUMUL_VAL;
        temp.value.cumul_val = atoi(argv[2]);
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "usl") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_UPD_SENSOR_LOC;
        temp.value.sensor_loc = atoi(argv[2]);
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "scrl") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_SET_CRANK_LENGTH;
        temp.value.crank_length = atoi(argv[2]);
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "schl") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_SET_CHAIN_LENGTH;
        temp.value.chain_length = atoi(argv[2]);
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "scw") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_SET_CHAIN_WEIGHT;
        temp.value.chain_length = atoi(argv[2]);
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "spl") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_SET_SPAN_LENGTH;
        temp.value.chain_length = atoi(argv[2]);
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "mask") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_MASK_CP_MEAS_CH_CONTENT;
        temp.value.chain_length = atoi(argv[2]);
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "rssl") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_REQ_SUPP_SENSOR_LOC;
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "rcrl") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_REQ_CRANK_LENGTH;
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "rchl") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_REQ_CHAIN_LENGTH;
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "rcw") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_REQ_CHAIN_WEIGHT;
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "rsl") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_REQ_SPAN_LENGTH;
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "soc") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_PT_START_OFFSET_COMP;
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "rsr") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_REQ_SAMPLING_RATE;
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "rfcd") == 0)
    {
        ble_cppc_cpcp temp;
        temp.op_code = CPP_CTNL_REQ_FACTORY_CALIBRATION_DATE;
        ble_cppc_write_cpcp_val_handler(&temp);
    }
    else if (strcmp(argv[1], "read_cpf") == 0)
    {
        ble_cppc_read_cpf(env->conn_idx);
    }
    else if (strcmp(argv[1], "read_sen_loca") == 0)
    {
        ble_cppc_read_sen_loca(env->conn_idx);
    }
    else if (strcmp(argv[1], "read_cpcp_cep_char") == 0)
    {
        ble_cppc_read_cpcp_cep(env->conn_idx);
    }
}

#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(ble_cppc_config, "BLE CPPC Configure");
#endif


int ble_cppc_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    ble_cppc_env_t *env = ble_cppc_get_env();

    LOG_I("cpc gattc event handler %d\r\n", event_id);
    int8_t res;

    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        LOG_I("cp register ret %d\r\n", rsp->status);
        sibles_write_remote_value_t value;
        uint16_t enable = 1;
        uint16_t info = 2;
        value.handle = env->cpm_char.cccd_hdl;
        value.write_type = SIBLES_WRITE;
        value.len = 2;
        value.value = (uint8_t *)&enable;
        conn_idx = rsp->conn_idx;
        ble_cppc_read_cpf(env->conn_idx);
        ble_cppc_read_sen_loca(env->conn_idx);
        ble_cppc_read_cpcp_cep(env->conn_idx);
        res = sibles_write_remote_value(env->remote_handle, rsp->conn_idx, &value);
        value.handle = env->cpv_char.cccd_hdl;
        res = sibles_write_remote_value(env->remote_handle, rsp->conn_idx, &value);
        value.handle = env->cpcp_char.cccd_hdl;
        value.value = (uint8_t *)&info;
        res = sibles_write_remote_value(env->remote_handle, rsp->conn_idx, &value);
        OS_ASSERT(res == SIBLES_WRITE_NO_ERR);

        env->state = BLE_CPPC_STATE_READY;
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        LOG_I("cpc handle:%d", ind->handle);

        if (ind->handle == env->cpm_char.value_hdl)
        {
            ble_cppc_cp_measure_hanlder(BLE_CPPC_CPM_NOTIFY, ind->value);
        }
        if (ind->handle == env->cpv_char.value_hdl)
        {
            ble_cppc_cp_vector_hanlder(BLE_CPPC_CPV_NOTIFY, ind->value, ind->length);
        }
        if (ind->handle == env->cpcp_char.value_hdl)
        {
            ble_cppc_cp_control_hanlder(BLE_CPPC_CPPC_NOTIFICATION_IND, ind->value, ind->length);
        }
        // Notify upper layer
        break;
    }
    case SIBLES_READ_REMOTE_VALUE_RSP:
    {
        sibles_read_remote_value_rsp_t *rsp = (sibles_read_remote_value_rsp_t *)data;
        if (rsp->handle == env->cpf_char.value_hdl)
        {
            ble_cppc_cpf_handler(rsp->value);
        }
        else if (rsp->handle == env->sl_char.value_hdl)
        {
            ble_cppc_sl_handler(rsp->value);
        }
        else if (rsp->handle == env->cpcp_char.cepd_hdl)
        {
            ble_cppc_cep_handler(rsp->value);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

int ble_cppc_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_cppc_env_t *env = ble_cppc_get_env();
    switch (event_id)
    {
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;
        if (rsp->result != HL_ERR_NO_ERROR)
            break; // Do nothing
        uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_CYCLING_POWER);
        uint16_t cpm_char_uuid = ATT_UUID_16(ATT_CHAR_CP_MEAS);
        uint16_t cpf_char_uuid = ATT_UUID_16(ATT_CHAR_CP_FEAT);
        uint16_t sl_char_uuid = ATT_UUID_16(ATT_CHAR_SENSOR_LOC);
        uint16_t cpv_char_uuid = ATT_UUID_16(ATT_CHAR_CP_VECTOR);
        uint16_t cpcp_char_uuid = ATT_UUID_16(ATT_CHAR_CP_CNTL_PT);

#ifdef SOC_SF32LB55X
        if (!(rsp->svc->uuid_len == ATT_UUID_16_LEN && !memcmp(rsp->svc->uuid, &svc_uuid, rsp->svc->uuid_len)))
        {
            // Not expected uuid
            break;
        }
#else
        // rsp->svc may null
        if (memcmp(rsp->search_uuid, &svc_uuid, rsp->search_svc_len) != 0)
            break;
#endif

        env->svc.hdl_start = rsp->svc->hdl_start;
        env->svc.hdl_end = rsp->svc->hdl_end;
        uint32_t i;
        uint16_t offset = 0;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            if (!memcmp(chara->uuid, &cpm_char_uuid, chara->uuid_len))
            {
                LOG_I("cpm received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);

                RT_ASSERT(chara->desc_count == 1);

                env->cpm_char.attr_hdl = chara->attr_hdl;
                env->cpm_char.value_hdl = chara->pointer_hdl;
                env->cpm_char.prop = chara->prop;
                env->cpm_char.cccd_hdl = chara->desc[0].attr_hdl;
                env->cpm_char.enabled = 1;


            }
            else if (!memcmp(chara->uuid, &cpf_char_uuid, chara->uuid_len))
            {
                LOG_I("cpf received, att handle(%x)", chara->attr_hdl);

                env->cpf_char.attr_hdl = chara->attr_hdl;
                env->cpf_char.value_hdl = chara->pointer_hdl;
                env->cpf_char.prop = chara->prop;
            }
            else if (!memcmp(chara->uuid, &sl_char_uuid, chara->uuid_len))
            {
                LOG_I("sl received, att handle(%x)", chara->attr_hdl);

                env->sl_char.attr_hdl = chara->attr_hdl;
                env->sl_char.value_hdl = chara->pointer_hdl;
                env->sl_char.prop = chara->prop;
            }
            else if (!memcmp(chara->uuid, &cpv_char_uuid, chara->uuid_len))
            {
                LOG_I("cpv received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                RT_ASSERT(chara->desc_count == 1);

                env->cpv_char.attr_hdl = chara->attr_hdl;
                env->cpv_char.value_hdl = chara->pointer_hdl;
                env->cpv_char.prop = chara->prop;
                env->cpv_char.cccd_hdl = chara->desc[0].attr_hdl;
                env->cpv_char.enabled = 1;

            }
            else if (!memcmp(chara->uuid, &cpcp_char_uuid, chara->uuid_len))
            {
                LOG_I("cpcp received,att handle(%x),des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                RT_ASSERT(chara->desc_count == 2);

                env->cpcp_char.attr_hdl = chara->attr_hdl;
                env->cpcp_char.value_hdl = chara->pointer_hdl;
                env->cpcp_char.prop = chara->prop;
                env->cpcp_char.cccd_hdl = chara->desc[0].attr_hdl;
                env->cpcp_char.cepd_hdl = chara->desc[1].attr_hdl;
                env->cpcp_char.enabled = 1;
            }

            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        //register first
        env->remote_handle = sibles_register_remote_svc(rsp->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_cppc_gattc_event_handler);
        // subscribe data src. then subscribe notfi src.
        break;
    }
    case SIBLES_REMOTE_CONNECTED_IND:
    {
        sibles_remote_connected_ind_t *ind = (sibles_remote_connected_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        break;
    }
    case BLE_GAP_BOND_IND:
    {
        ble_gap_bond_ind_t *evt = (ble_gap_bond_ind_t *)data;
        if (evt->info == GAPC_PAIRING_SUCCEED && env->is_enable)
            ble_cppc_enable(evt->conn_idx);
        break;
    }
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        if (env->is_enable)
            ble_cppc_enable(ind->conn_idx);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if (env->conn_idx == ind->conn_idx)
            sibles_unregister_remote_svc(env->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_cppc_gattc_event_handler);
        break;
    }
    default:
        break;
    }
    return 0;
}

void ble_cppc_init(uint8_t enable)
{
    ble_cppc_env_t *env = ble_cppc_get_env();
    env->is_enable = enable;
    LOG_I("ble_cppc_init: cppc enable %d\n", enable);
}

BLE_EVENT_REGISTER(ble_cppc_ble_event_handler, (uint32_t)NULL);

#endif






















