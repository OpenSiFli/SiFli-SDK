/**
  ******************************************************************************
  * @file   bf0_ble_cscpc.h
  * @author Sifli software development team
  * @brief Header file - Sibles Cycling Speed and Cadence Profile Collector definition.
 *
  ******************************************************************************
*/
/*
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

#ifndef __BF0_BLE_CSCPC_H
#define __BF0_BLE_CSCPC_H

#include "bf0_ble_common.h"

enum cscp_meas_flags
{
    /// Wheel Revolution Data Present
    BLE_CSCP_MEAS_WHEEL_REV_DATA_PRESENT    = 0x01,
    /// Crank Revolution Data Present
    BLE_CSCP_MEAS_CRANK_REV_DATA_PRESENT    = 0x02,

    /// All present
    BLE_CSCP_MEAS_ALL_PRESENT               = 0x03,
};

enum ble_cscpc_event
{
    //CSC Measurement Notify RSP see @ble_csc_meas_value_ind
    BLE_CSCPC_CSC_MEASUREMENT_NOTIFY = BLE_CSCPC_TYPE,
    //CSC Feature Read RSP see @ble_csc_feat_value_ind
    BLE_CSCPC_READ_CSC_FEATURE_RSP,
    //Sensor Loaction Read RSP see @ble_sensor_loc_value_ind
    BLE_CSCPC_READ_SENSOR_LOCATION_RSP,
    //SC Control Point Write RSP
    BLE_CSCPC_WEITE_CONTROL_POINT_RSP,
    //SC Control Point Indicate RSP @cscpc_ctnl_pt_rsp
    BLE_CSCPC_CONTROL_POINT_INDICATE,
};

enum cscp_sc_ctnl_pt_op_code
{
    /// Reserved value
    CSCP_CTNL_PT_OP_RESERVED        = 0,

    /// Set Cumulative Value
    CSCP_CTNL_PT_OP_SET_CUMUL_VAL,
    /// Start Sensor Calibration
    CSCP_CTNL_PT_OP_START_CALIB,
    /// Update Sensor Location
    CSCP_CTNL_PT_OP_UPD_LOC,
    /// Request Supported Sensor Locations
    CSCP_CTNL_PT_OP_REQ_SUPP_LOC,

    /// Response Code
    CSCP_CTNL_PT_RSP_CODE           = 16,
};

enum cscp_ctnl_pt_resp_val
{
    /// Reserved value
    CSCP_CTNL_PT_RESP_RESERVED      = 0,

    /// Success
    CSCP_CTNL_PT_RESP_SUCCESS,
    /// Operation Code Not Supported
    CSCP_CTNL_PT_RESP_NOT_SUPP,
    /// Invalid Parameter
    CSCP_CTNL_PT_RESP_INV_PARAM,
    /// Operation Failed
    CSCP_CTNL_PT_RESP_FAILED,
};

enum cscp_sensor_loc
{
    /// Other (0)
    CSCP_LOC_OTHER          = 0,
    /// Front Wheel (4)
    CSCP_LOC_FRONT_WHEEL    = 4,
    /// Left Crank (5)
    CSCP_LOC_LEFT_CRANK,
    /// Right Crank (6)
    CSCP_LOC_RIGHT_CRANK,
    /// Left Pedal (7)
    CSCP_LOC_LEFT_PEDAL,
    /// Right Pedal (8)
    CSCP_LOC_RIGHT_PEDAL,
    /// Rear Dropout (9)
    CSCP_LOC_REAR_DROPOUT,
    /// Chainstay (10)
    CSCP_LOC_CHAINSTAY,
    /// Front Hub (11)
    CSCP_LOC_FRONT_HUB,
    /// Rear Wheel (12)
    CSCP_LOC_REAR_WHEEL,
    /// Rear Hub (13)
    CSCP_LOC_REAR_HUB,

    CSCP_LOC_MAX,
};

typedef struct
{
    /// Operation code
    uint8_t operation;
    /// Status
    uint8_t status;
} ble_cscpc_cmp_evt;

struct cscp_csc_meas
{
    /// Flags
    uint8_t flags;
    /// Cumulative Crank Revolution
    uint16_t cumul_crank_rev;
    /// Last Crank Event Time
    uint16_t last_crank_evt_time;
    /// Last Wheel Event Time
    uint16_t last_wheel_evt_time;
    /// Cumulative Wheel Revolution
    uint32_t cumul_wheel_rev;
};

typedef struct
{
    uint8_t att_code;
    struct cscp_csc_meas csc_meas;
} ble_csc_meas_value_ind;

typedef struct
{
    uint8_t att_code;
    uint16_t sensor_feat;
} ble_csc_feat_value_ind;

typedef struct
{
    uint8_t att_code;
    uint8_t sensor_loc;
} ble_sensor_loc_value_ind;

typedef struct
{
    uint8_t att_code;
    uint16_t ntf_cfg;
} ble_sc_ctnl_pt_value_ind;

struct cscp_sc_ctnl_pt_req
{
    /// Operation Code
    uint8_t op_code;
    /// Value
    union cscp_sc_ctnl_pt_req_val
    {
        /// Sensor Location
        uint8_t sensor_loc;
        /// Cumulative value
        uint32_t cumul_val;
    } value;
};

struct cscpc_ctnl_pt_cfg_req
{
    /// Operation Code
    uint8_t operation;
    /// SC Control Point Request
    struct cscp_sc_ctnl_pt_req sc_ctnl_pt;
};


struct cscp_sc_ctnl_pt_rsp
{
    /// Requested Operation Code
    uint8_t req_op_code;
    /// Response Value
    uint8_t resp_value;
    /// List of supported locations
    uint16_t supp_loc;
};


struct cscpc_ctnl_pt_rsp
{
    /// SC Control Point Response
    struct cscp_sc_ctnl_pt_rsp ctnl_pt_rsp;
};

void ble_cscpc_init(uint8_t enable);
//csc feature read req
int8_t ble_cscpc_read_csc_feature(uint8_t conn_idx);
//sensor location read req
int8_t ble_cscpc_read_sensor_location(uint8_t conn_idx);

int8_t ble_cscpc_enable(uint8_t conn_idx);
//sc control point characteristic extended properties descriptors read req
int8_t ble_cscpc_read_sc_ct_pt_cepd(uint8_t conn_idx);

/**
  ******************************************************************************
  csc measurement notify on-off
  flag==1    notify on
  flag==0    notify off
  ******************************************************************************
*/
void ble_csc_meas_ntf_enable(uint16_t flag);

/**
  ******************************************************************************
  sc control point indicate on-off
  flag==2    indicate on
  flag==0    indicate off
  ******************************************************************************
*/
void ble_sc_ct_pt_ind_enable(uint16_t flag);
#endif //__BF0_BLE_CSCPC_H