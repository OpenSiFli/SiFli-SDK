/**
  ******************************************************************************
  * @file   bf0_ble_cscpc.c
  * @author Sifli software development team
  * @brief  Sibles Cycling Speed and Cadence Profile Collector.
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

#include "rtconfig.h"
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles.h"
#include "att.h"
#include "bf0_ble_cscpc.h"
#include "bf0_ble_gap.h"
#include "os_adaptor.h"
#include "bf0_sibles_internal.h"
#include "bf0_ble_gap_internal.h"

#ifdef BSP_BLE_CSCPC
#ifndef BSP_USING_PC_SIMULATOR
    #include "rtc.h"
#endif
#define LOG_TAG "cscp_srv"
#include "log.h"

#define CSCP_SC_CNTL_PT_REQ_MIN_LEN         (1)

static uint16_t co_read16p(void const *ptr16)
{
    uint16_t value = ((uint8_t *)ptr16)[0] | ((uint8_t *)ptr16)[1] << 8;
    return value;
}

static uint32_t co_read32p(void const *ptr32)
{
    uint16_t addr_l, addr_h;
    addr_l = co_read16p(ptr32);
    addr_h = co_read16p((uint8_t *)ptr32 + 2);
    return ((uint32_t)addr_l | (uint32_t)addr_h << 16);
}
static void co_write32p(void const *ptr32, uint32_t value)
{
    uint8_t *ptr = (uint8_t *)ptr32;

    *ptr++ = (uint8_t)(value & 0xff);
    *ptr++ = (uint8_t)((value & 0xff00) >> 8);
    *ptr++ = (uint8_t)((value & 0xff0000) >> 16);
    *ptr = (uint8_t)((value & 0xff000000) >> 24);
}


typedef struct
{
    uint16_t hdl_start;
    uint16_t hdl_end;
} ble_cscpc_svc_t;

typedef struct
{
    uint16_t attr_hdl;
    uint16_t value_hdl;
    uint16_t cccd_hdl;
    uint16_t cepd_hdl;
    uint8_t prop;
    uint8_t enabled;
} ble_cscpc_char_t;

enum ble_cscpc_state_t
{
    BLE_CSCPC_STATE_IDLE,
    BLE_CSCPC_STATE_READY,
    BLE_CSCPC_STATE_BUSY
};

typedef struct
{
    uint8_t state;
    uint8_t is_enable;
    uint8_t conn_idx;
    uint16_t remote_handle;
    ble_cscpc_svc_t svc;
    ble_cscpc_char_t csc_meas_char;
    ble_cscpc_char_t csc_feat_info;
    ble_cscpc_char_t sensor_loc_char;
    ble_cscpc_char_t sc_ctnl_pt_info;
} ble_cscpc_env_t;

static ble_cscpc_env_t g_ble_cscpc_env;

static ble_cscpc_env_t *ble_cscpc_get_env(void)
{
    return &g_ble_cscpc_env;
};

static void ble_cscpc_csc_measurement_handler(uint16_t event, uint8_t *data)
{
    uint8_t offset = 1;
    ble_csc_meas_value_ind ind;
    ind.csc_meas.flags = data[0];
    if (ind.csc_meas.flags & BLE_CSCP_MEAS_WHEEL_REV_DATA_PRESENT)
    {
        // Cumulative Wheel Revolutions

        ind.csc_meas.cumul_wheel_rev = co_read32p(&data[offset]);
        offset += 4;

        // Last Wheel Event Time
        ind.csc_meas.last_wheel_evt_time = co_read16p(&data[offset]);
        offset += 2;
    }

    // Cumulative Crank Revolutions
    // Last Crank Event Time
    if (ind.csc_meas.flags & BLE_CSCP_MEAS_CRANK_REV_DATA_PRESENT)
    {
        // Cumulative Crank Revolutions
        ind.csc_meas.cumul_crank_rev = co_read16p(&data[offset]);
        offset += 2;

        // Last Crank Event Time
        ind.csc_meas.last_crank_evt_time = co_read16p(&data[offset]);
    }

    ble_event_publish(event, &ind, sizeof(ble_csc_meas_value_ind));
}

static void ble_cscpc_csc_feature_handler(uint16_t event, uint8_t *data)
{
    ble_csc_feat_value_ind ind;
    struct gattc_event_ind const *rsp = (struct gattc_event_ind const *)data;
    ind.sensor_feat = co_read16p(rsp);
    ble_event_publish(event, &ind, sizeof(ble_csc_feat_value_ind));
}

static void ble_cscpc_sensor_location_handler(uint16_t event, uint8_t *data)
{
    ble_sensor_loc_value_ind ind;
    //struct gattc_event_ind const *rsp=(struct gattc_event_ind const *)data;
    ind.sensor_loc = *data;
    ble_event_publish(event, &ind, sizeof(ble_sensor_loc_value_ind));
}


uint8_t ble_cscpc_pack_ctnl_pt_req(struct cscp_sc_ctnl_pt_req *param, uint8_t *req)
{
    // Request Length
    uint8_t req_len = CSCP_SC_CNTL_PT_REQ_MIN_LEN;
    // Set the operation code
    req[0] = param->op_code;
    // Fulfill the message according to the operation code
    switch (param->op_code)
    {
    case (CSCP_CTNL_PT_OP_SET_CUMUL_VAL):
    {
        // Set the cumulative value
        co_write32p(&req[req_len], param->value.cumul_val);
        // Update length
        req_len += 4;
    }
    break;

    case (CSCP_CTNL_PT_OP_UPD_LOC):
    {
        // Set the sensor location
        req[req_len] = param->value.sensor_loc;
        // Update length
        req_len++;
    }
    break;

    case (CSCP_CTNL_PT_OP_RESERVED):
    case (CSCP_CTNL_PT_OP_START_CALIB):
    case (CSCP_CTNL_PT_OP_REQ_SUPP_LOC):
    {
        // Nothing more to do
    } break;

    default:
    {
        //status = PRF_ERR_INVALID_PARAM;
    }
    break;
    }
    return req_len;
}
static int8_t ble_cscpc_sc_ctnl_pt_write_handler(struct cscp_sc_ctnl_pt_req *data)
{
    int8_t ret;
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    sibles_write_remote_value_t value;
    value.value = malloc(8);
    value.write_type = SIBLES_WRITE;
    value.handle = env->sensor_loc_char.value_hdl;
    value.len = ble_cscpc_pack_ctnl_pt_req(data, value.value);
    ret = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    free(value.value);
    OS_ASSERT(ret == SIBLES_WRITE_NO_ERR);
    return ret;
}

static void ble_cscpc_sc_ctnl_pt_ind_handler(uint16_t event, sibles_remote_event_ind_t *data)
{
    sibles_remote_event_ind_t const *rsp = (sibles_remote_event_ind_t const *)data;
    // confirm that indication has been correctly received

    struct cscpc_ctnl_pt_rsp ind;
    // Requested operation code
    ind.ctnl_pt_rsp.req_op_code = rsp->value[1];
    // Response value
    ind.ctnl_pt_rsp.resp_value = rsp->value[2];
    // Get the list of supported sensor locations if needed
    if ((ind.ctnl_pt_rsp.req_op_code == CSCP_CTNL_PT_OP_REQ_SUPP_LOC) && (ind.ctnl_pt_rsp.resp_value == CSCP_CTNL_PT_RESP_SUCCESS) && (rsp->length > 3))
    {
        // Get the number of supported locations that have been received
        uint8_t nb_supp_loc = (rsp->length - 3);
        // Counter
        uint8_t counter;
        // Location
        uint8_t loc;

        for (counter = 0; counter < nb_supp_loc; counter++)
        {
            loc = rsp->value[counter + 3];

            // Check if valid
            if (loc < CSCP_LOC_MAX)
            {
                ind.ctnl_pt_rsp.supp_loc |= (1 << loc);
            }

        }
    }
    ble_event_publish(event, &ind, sizeof(struct cscpc_ctnl_pt_rsp));
}



int8_t ble_cscpc_enable(uint8_t conn_idx)
{
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_CYCLING_SPEED_CADENCE);
    int8_t ret = sibles_search_service(conn_idx, ATT_UUID_16_LEN, (uint8_t *)&svc_uuid);
    if (ret == 0)
        env->state = BLE_CSCPC_STATE_BUSY;
    return ret;
}

int8_t ble_cscpc_read_csc_feature(uint8_t conn_idx)
{
    int8_t ret;
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->csc_feat_info.value_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}

int8_t ble_cscpc_read_sensor_location(uint8_t conn_idx)
{
    int8_t ret;
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->sensor_loc_char.value_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}

int8_t ble_cscpc_read_sc_ct_pt_cepd(uint8_t conn_idx)
{
    int8_t ret;
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->sc_ctnl_pt_info.cepd_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}

void ble_csc_meas_ntf_enable(uint16_t flag)
{
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    int8_t res;
    sibles_write_remote_value_t value;
    uint16_t enable = flag;
    value.handle = env->csc_meas_char.cccd_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = 2;
    value.value = (uint8_t *)&enable;
    res = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
    env->state = BLE_CSCPC_STATE_READY;
}

void ble_sc_ct_pt_ind_enable(uint16_t flag)
{
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    int8_t res;
    sibles_write_remote_value_t value;
    uint16_t enable = flag;
    value.handle = env->sc_ctnl_pt_info.cccd_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = 2;
    value.value = (uint8_t *)&enable;
    res = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
    env->state = BLE_CSCPC_STATE_READY;
}


int ble_cscpc_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    LOG_I("cscpc gattc event handler %d\r\n", event_id);
    int8_t res;

    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        LOG_I("csc register ret %d\r\n", rsp->status);
        sibles_write_remote_value_t value1;
        sibles_write_remote_value_t value2;
        uint16_t enable1 = 1;          //Notify
        uint16_t enable2 = 2;        //Indicate
        value1.handle = env->csc_meas_char.cccd_hdl;
        value1.write_type = SIBLES_WRITE;
        value1.len = 2;
        value1.value = (uint8_t *)&enable1;
        res = sibles_write_remote_value(env->remote_handle, rsp->conn_idx, &value1);
        OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
        value2.handle = env->sc_ctnl_pt_info.cccd_hdl;
        value2.write_type = SIBLES_WRITE;
        value2.len = 2;
        value2.value = (uint8_t *)&enable2;
        res = sibles_write_remote_value(env->remote_handle, rsp->conn_idx, &value2);
        OS_ASSERT(res == SIBLES_WRITE_NO_ERR);

        env->state = BLE_CSCPC_STATE_READY;
        ble_cscpc_read_csc_feature(env->conn_idx);
        ble_cscpc_read_sensor_location(env->conn_idx);
        ble_cscpc_read_sc_ct_pt_cepd(env->conn_idx);

        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        LOG_I("cscpc handle:%d", ind->handle);
        if (ind->handle == env->csc_meas_char.value_hdl)
        {
            ble_cscpc_csc_measurement_handler(BLE_CSCPC_CSC_MEASUREMENT_NOTIFY, ind->value);
        }
        else if (ind->handle == env->sc_ctnl_pt_info.value_hdl)
        {
            ble_cscpc_sc_ctnl_pt_ind_handler(BLE_CSCPC_CONTROL_POINT_INDICATE, ind);
        }

        break;
    }
    case SIBLES_READ_REMOTE_VALUE_RSP:
    {
        sibles_read_remote_value_rsp_t *rsp = (sibles_read_remote_value_rsp_t *)data;
        if (rsp->handle == env->csc_feat_info.value_hdl)
        {
            ble_cscpc_csc_feature_handler(BLE_CSCPC_READ_CSC_FEATURE_RSP, rsp->value);
        }
        else if (rsp->handle == env->sensor_loc_char.value_hdl)
        {

            ble_cscpc_sensor_location_handler(BLE_CSCPC_READ_SENSOR_LOCATION_RSP, rsp->value);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

int ble_cscpc_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    switch (event_id)
    {
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;
        if (rsp->result != HL_ERR_NO_ERROR)
            break; // Do nothing
        uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_CYCLING_SPEED_CADENCE);
        uint16_t csc_meas_char_uuid = ATT_UUID_16(ATT_CHAR_CSC_MEAS);
        uint16_t csc_feat_info_uuid = ATT_UUID_16(ATT_CHAR_CSC_FEAT);
        uint16_t sensor_loc_char_uuid = ATT_UUID_16(ATT_CHAR_SENSOR_LOC);
        uint16_t sc_ctnl_pt_info_uuid = ATT_UUID_16(ATT_CHAR_SC_CNTL_PT);
        // rsp->svc may null
        if (memcmp(rsp->search_uuid, &svc_uuid, rsp->search_svc_len) != 0)
            break;

        env->svc.hdl_start = rsp->svc->hdl_start;
        env->svc.hdl_end = rsp->svc->hdl_end;
        uint32_t i;
        uint16_t offset = 0;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            if (!memcmp(chara->uuid, &csc_meas_char_uuid, chara->uuid_len))
            {
                LOG_I("csc measurement received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                RT_ASSERT(chara->desc_count == 1);
                env->csc_meas_char.attr_hdl = chara->attr_hdl;
                env->csc_meas_char.value_hdl = chara->pointer_hdl;
                env->csc_meas_char.prop = chara->prop;
                env->csc_meas_char.cccd_hdl = chara->desc[0].attr_hdl;
                env->csc_meas_char.enabled = 1;
            }
            else if (!memcmp(chara->uuid, &csc_feat_info_uuid, chara->uuid_len))
            {
                LOG_I("csc feature received, att handle(%x)", chara->attr_hdl);
                env->csc_feat_info.attr_hdl = chara->attr_hdl;
                env->csc_feat_info.value_hdl = chara->pointer_hdl;
                env->csc_feat_info.prop = chara->prop;
                env->csc_feat_info.enabled = 1;
            }
            else if (!memcmp(chara->uuid, &sensor_loc_char_uuid, chara->uuid_len))
            {
                LOG_I("sensor location received, att handle(%x)", chara->attr_hdl);
                env->sensor_loc_char.attr_hdl = chara->attr_hdl;
                env->sensor_loc_char.value_hdl = chara->pointer_hdl;
                env->sensor_loc_char.prop = chara->prop;
                env->sensor_loc_char.enabled = 1;
            }
            else if (!memcmp(chara->uuid, &sc_ctnl_pt_info_uuid, chara->uuid_len))
            {
                LOG_I("sensor control point received, att handle(%x)", chara->attr_hdl);
                LOG_I("sc_pt desc_count is %x,", chara->desc_count);
                //RT_ASSERT(chara->desc_count == 2);
                env->sc_ctnl_pt_info.attr_hdl = chara->attr_hdl;
                env->sc_ctnl_pt_info.value_hdl = chara->pointer_hdl;
                env->sc_ctnl_pt_info.prop = chara->prop;
                env->sc_ctnl_pt_info.cccd_hdl = chara->desc[1].attr_hdl;     //desc_count==3,0 is cepd;1 is cccd;2 is cepd
                env->sc_ctnl_pt_info.cepd_hdl = chara->desc[0].attr_hdl;
                env->sc_ctnl_pt_info.enabled = 1;
            }

            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        //register first
        env->remote_handle = sibles_register_remote_svc(rsp->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_cscpc_gattc_event_handler);
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
            ble_cscpc_enable(evt->conn_idx);
        break;
    }
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        if (env->is_enable)
            ble_cscpc_enable(ind->conn_idx);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if (env->conn_idx == ind->conn_idx)
            sibles_unregister_remote_svc(env->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_cscpc_gattc_event_handler);
        break;
    }
    default:
        break;
    }
    return 0;
}

void ble_cscpc_init(uint8_t enable)
{
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    env->is_enable = enable;
    LOG_D("ble_cscpc_init: cscpic enable %d\n", enable);
}

BLE_EVENT_REGISTER(ble_cscpc_ble_event_handler, (uint32_t)NULL);

void csc_test(int argc, char **argv)
{
    ble_cscpc_env_t *env = ble_cscpc_get_env();
    if (argc > 1)
    {
        if (strcmp("csc_feat_read", argv[1]) == 0)
        {
            int8_t ret = ble_cscpc_read_csc_feature(env->conn_idx);
        }
        else if (strcmp("sensor_loc_read", argv[1]) == 0)
        {
            int8_t ret = ble_cscpc_read_sensor_location(env->conn_idx);
        }
        else if (strcmp("csc_meas_enable", argv[1]) == 0)
        {
            uint16_t flag = atoi(argv[2]);
            ble_csc_meas_ntf_enable(flag);
        }
        else if (strcmp("sc_pt_enable", argv[1]) == 0)
        {
            uint16_t flag = atoi(argv[2]);
            ble_sc_ct_pt_ind_enable(flag);
        }
        else if (strcmp("write_sc_ct_pt", argv[1]) == 0)
        {
            if (strcmp("cumul", argv[2]) == 0)
            {
                struct cscp_sc_ctnl_pt_req data;
                data.op_code = CSCP_CTNL_PT_OP_SET_CUMUL_VAL;
                data.value.cumul_val = atoi(argv[3]);
                int8_t ret = ble_cscpc_sc_ctnl_pt_write_handler(&data);
            }
            else if (strcmp("sensor", argv[2]) == 0)
            {
                struct cscp_sc_ctnl_pt_req data;
                data.op_code = CSCP_CTNL_PT_OP_UPD_LOC;
                data.value.sensor_loc = atoi(argv[3]);
                int8_t ret = ble_cscpc_sc_ctnl_pt_write_handler(&data);
            }
        }
    }
}
#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(csc_test, "csc_test");
#endif

#endif //BSP_BLE_CSCPC






