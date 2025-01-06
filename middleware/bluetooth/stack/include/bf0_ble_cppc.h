/**
  ******************************************************************************
  * @file   bf0_ble_cppc.h
  * @author Sifli software development team
  * @brief Header file - Sibles  Cycling Power Profile Collector definition.
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

#ifndef __BF0_BLE_CPPC_H
#define __BF0_BLE_CPPC_H

#include "bf0_ble_common.h"


#ifndef __ARRAY_EMPTY
    #define __ARRAY_EMPTY
#endif



#define CPP_ERROR_PROC_IN_PROGRESS        (0xFE)

/// CP Measurement Notification Value Max Length
#define CPP_CP_MEAS_NTF_MAX_LEN           (35)
/// CP Measurement Value Min Length
#define CPP_CP_MEAS_NTF_MIN_LEN           (4)

/// ADV Header size
#define CPP_CP_ADV_HEADER_LEN             (3)
/// ADV Length size
#define CPP_CP_ADV_LENGTH_LEN             (1)

/// CP Measurement Advertisement Value Max Length
#define CPP_CP_MEAS_ADV_MAX_LEN           (CPP_CP_MEAS_NTF_MAX_LEN + CPP_CP_ADV_HEADER_LEN)
/// CP Measurement Value Min Length
#define CPP_CP_MEAS_ADV_MIN_LEN           (CPP_CP_MEAS_NTF_MIN_LEN + CPP_CP_ADV_HEADER_LEN)

/// CP Vector Value Max Length
#define CPP_CP_VECTOR_MAX_LEN             (19)
/// CP Vector Value Min Length
#define CPP_CP_VECTOR_MIN_LEN             (1)

/// CP Control Point Value Max Length
#define CPP_CP_CNTL_PT_REQ_MAX_LEN        (9)
/// CP Control Point Value Min Length
#define CPP_CP_CNTL_PT_REQ_MIN_LEN        (1)

/// CP Control Point Value Max Length
#define CPP_CP_CNTL_PT_RSP_MAX_LEN        (20)
/// CP Control Point Value Min Length
#define CPP_CP_CNTL_PT_RSP_MIN_LEN        (3)
enum cpp_meas_flags
{
    /// Pedal Power Balance Present
    CPP_MEAS_PEDAL_POWER_BALANCE_PRESENT         = 0x0001,
    /// Pedal Power Balance Reference
    CPP_MEAS_PEDAL_POWER_BALANCE_REFERENCE       = 0x0002,
    /// Accumulated Torque Present
    CPP_MEAS_ACCUM_TORQUE_PRESENT                = 0x0004,
    /// Accumulated Torque Source
    CPP_MEAS_ACCUM_TORQUE_SOURCE                 = 0x0008,
    /// Wheel Revolution Data Present
    CPP_MEAS_WHEEL_REV_DATA_PRESENT              = 0x0010,
    /// Crank Revolution Data Present
    CPP_MEAS_CRANK_REV_DATA_PRESENT              = 0x0020,
    /// Extreme Force Magnitudes Present
    CPP_MEAS_EXTREME_FORCE_MAGNITUDES_PRESENT    = 0x0040,
    /// Extreme Torque Magnitudes Present
    CPP_MEAS_EXTREME_TORQUE_MAGNITUDES_PRESENT   = 0x0080,
    /// Extreme Angles Present
    CPP_MEAS_EXTREME_ANGLES_PRESENT              = 0x0100,
    /// Top Dead Spot Angle Present
    CPP_MEAS_TOP_DEAD_SPOT_ANGLE_PRESENT         = 0x0200,
    /// Bottom Dead Spot Angle Present
    CPP_MEAS_BOTTOM_DEAD_SPOT_ANGLE_PRESENT      = 0x0400,
    /// Accumulated Energy Present
    CPP_MEAS_ACCUM_ENERGY_PRESENT                = 0x0800,
    /// Offset Compensation Indicator
    CPP_MEAS_OFFSET_COMPENSATION_INDICATOR       = 0x1000,

    /// All Supported
    CPP_MEAS_ALL_SUPP                            = 0x1FFF,
};

enum cpp_vector_flags
{
    /// Crank Revolution Data Present
    CPP_VECTOR_CRANK_REV_DATA_PRESENT                = 0x01,
    /// First Crank Measurement Angle Present
    CPP_VECTOR_FIRST_CRANK_MEAS_ANGLE_PRESENT        = 0x02,
    /// Instantaneous Force Magnitude Array Present
    CPP_VECTOR_INST_FORCE_MAGNITUDE_ARRAY_PRESENT    = 0x04,
    /// Instantaneous Torque Magnitude Array Present
    CPP_VECTOR_INST_TORQUE_MAGNITUDE_ARRAY_PRESENT   = 0x08,
    /// Instantaneous Measurement Direction LSB
    CPP_VECTOR_INST_MEAS_DIRECTION_LSB                 = 0x10,
    /// Instantaneous Measurement Direction MSB
    CPP_VECTOR_INST_MEAS_DIRECTION_MSB                 = 0x20,

    ///All suported
    CPP_VECTOR_ALL_SUPP                                 = 0x3F,
};

enum cpp_ctnl_pt_code
{
    /// Reserved value
    CPP_CTNL_PT_RESERVED        = 0,

    /// Set Cumulative Value
    CPP_CTNL_PT_SET_CUMUL_VAL,
    /// Update Sensor Location
    CPP_CTNL_PT_UPD_SENSOR_LOC,
    /// Request Supported Sensor Locations
    CPP_CTNL_PT_REQ_SUPP_SENSOR_LOC,
    /// Set Crank Length
    CPP_CTNL_PT_SET_CRANK_LENGTH,
    /// Request Crank Length
    CPP_CTNL_PT_REQ_CRANK_LENGTH,
    /// Set Chain Length
    CPP_CTNL_PT_SET_CHAIN_LENGTH,
    /// Request Chain Length
    CPP_CTNL_PT_REQ_CHAIN_LENGTH,
    /// Set Chain Weight
    CPP_CTNL_PT_SET_CHAIN_WEIGHT,
    /// Request Chain Weight
    CPP_CTNL_PT_REQ_CHAIN_WEIGHT,
    /// Set Span Length
    CPP_CTNL_PT_SET_SPAN_LENGTH,
    /// Request Span Length
    CPP_CTNL_PT_REQ_SPAN_LENGTH,
    /// Start Offset Compensation
    CPP_CTNL_PT_START_OFFSET_COMP,
    /// Mask CP Measurement Characteristic Content
    CPP_CTNL_MASK_CP_MEAS_CH_CONTENT,
    /// Request Sampling Rate
    CPP_CTNL_REQ_SAMPLING_RATE,
    /// Request Factory Calibration Date
    CPP_CTNL_REQ_FACTORY_CALIBRATION_DATE,

    /// Response Code
    CPP_CTNL_PT_RSP_CODE             = 32,
};

enum cpp_ctnl_pt_resp_val
{
    /// Reserved value
    CPP_CTNL_PT_RESP_RESERVED      = 0,

    /// Success
    CPP_CTNL_PT_RESP_SUCCESS,
    /// Operation Code Not Supported
    CPP_CTNL_PT_RESP_NOT_SUPP,
    /// Invalid Parameter
    CPP_CTNL_PT_RESP_INV_PARAM,
    /// Operation Failed
    CPP_CTNL_PT_RESP_FAILED,
};

enum ble_cppc_event
{
    // Cycling power measurement notify event, see @ble_cppc_cpm
    BLE_CPPC_CPM_NOTIFY = BLE_CPPC_TYPE,
    // Cycling power Feature read response event, see @ble_cppc_cpf
    BLE_CPPC_READ_CPF_RSP,
    // Cycling power sensor location read response event see @ble_cppc_sl
    BLE_CPPC_READ_SL_RSP,
    // Cycling power control point read extended properties descriptor response event
    // see @ble_cppc_cep
    BLE_CPPC_READ_CPCP_CEP_RSP,
    // Cycling power vector notify event see @ble_cppc_cpv
    BLE_CPPC_CPV_NOTIFY,
    // Cycling power point indicate event, see @ble_cpcp_notyf_rsp
    BLE_CPPC_CPPC_NOTIFICATION_IND,
};

enum cpp_sensor_loc
{
    /// Other (0)
    CPP_LOC_OTHER          = 0,
    /// Top of shoe (1)
    CPP_LOC_TOP_SHOE,
    /// In shoe (2)
    CPP_LOC_IN_SHOE,
    /// Hip (3)
    CPP_LOC_HIP,
    /// Front Wheel (4)
    CPP_LOC_FRONT_WHEEL,
    /// Left Crank (5)
    CPP_LOC_LEFT_CRANK,
    /// Right Crank (6)
    CPP_LOC_RIGHT_CRANK,
    /// Left Pedal (7)
    CPP_LOC_LEFT_PEDAL,
    /// Right Pedal (8)
    CPP_LOC_RIGHT_PEDAL,
    /// Front Hub (9)
    CPP_LOC_FRONT_HUB,
    /// Rear Dropout (10)
    CPP_LOC_REAR_DROPOUT,
    /// Chainstay (11)
    CPP_LOC_CHAINSTAY,
    /// Rear Wheel (12)
    CPP_LOC_REAR_WHEEL,
    /// Rear Hub (13)
    CPP_LOC_REAR_HUB,
    /// Chest (14)
    CPP_LOC_CHEST,

    CPP_LOC_MAX,
};

struct gattc_event_cfm
{
    /// Attribute handle
    uint16_t handle;
};


typedef struct
{
    /// Flags
    uint16_t flags;
    /// Instantaneous Power
    int16_t inst_power;
    /// Pedal Power Balance
    uint8_t pedal_power_balance;
    /// Accumulated torque
    uint16_t accum_torque;
    /// Cumulative Wheel Revolutions
    uint32_t cumul_wheel_rev;
    /// Last Wheel Event Time
    uint16_t last_wheel_evt_time;
    /// Cumulative Crank Revolution
    uint16_t cumul_crank_rev;
    /// Last Crank Event Time
    uint16_t last_crank_evt_time;
    /// Maximum Force Magnitude
    int16_t max_force_magnitude;
    /// Minimum Force Magnitude
    int16_t min_force_magnitude;
    /// Maximum Torque Magnitude
    int16_t max_torque_magnitude;
    /// Minimum Torque Magnitude
    int16_t min_torque_magnitude;
    /// Maximum Angle (12 bits)
    uint16_t max_angle;
    /// Minimum Angle (12bits)
    uint16_t min_angle;
    /// Top Dead Spot Angle
    uint16_t top_dead_spot_angle;
    /// Bottom Dead Spot Angle
    uint16_t bot_dead_spot_angle;
    ///Accumulated energy
    uint16_t accum_energy;

} ble_cppc_cpm;

typedef struct
{
    /// Flags
    uint8_t flags;
    /// Force-Torque Magnitude Array Length
    uint8_t nb;
    /// Cumulative Crank Revolutions
    uint16_t cumul_crank_rev;
    /// Last Crank Event Time
    uint16_t last_crank_evt_time;
    /// First Crank Measurement Angle
    uint16_t first_crank_meas_angle;
    ///Mutually excluded Force and Torque Magnitude Arrays
    int16_t  force_torque_magnitude[__ARRAY_EMPTY];
} ble_cppc_cpv;

typedef struct
{
    /// CP Feature
    uint32_t sensor_feat;
} ble_cppc_cpf;


typedef struct
{
    /// Sensor Location
    uint8_t sensor_loc;
} ble_cppc_sl;

typedef struct
{
    uint16_t cep_val;
} ble_cppc_cep;

typedef struct
{
    /// Operation Code
    uint8_t op_code;
    /// Value
    union ble_cppc_cpcp_req_val
    {
        /// Cumulative Value
        uint32_t cumul_val;
        /// Sensor Location
        uint8_t sensor_loc;
        /// Crank Length
        uint16_t crank_length;
        /// Chain Length
        uint16_t chain_length;
        /// Chain Weight
        uint16_t chain_weight;
        /// Span Length
        uint16_t span_length;
        /// Mask Content
        uint16_t mask_content;
    } value;
} ble_cppc_cpcp;

struct prf_date_time
{
    /// year time element
    uint16_t year;
    /// month time element
    uint8_t month;
    /// day time element
    uint8_t day;
    /// hour time element
    uint8_t hour;
    /// minute time element
    uint8_t min;
    /// second time element
    uint8_t sec;
};

typedef struct
{
    uint8_t req_op_code;
    /// Response Value
    uint8_t resp_value;

    ///Value
    union ble_cpcp_notyf_rsp_val
    {
        /// List of supported locations
        uint32_t supp_loc;
        /// Crank Length
        uint16_t crank_length;
        /// Chain Length
        uint16_t chain_length;
        /// Chain Weight
        uint16_t chain_weight;
        /// Span Length
        uint16_t span_length;
        /// Start Offset Compensation
        int16_t offset_comp;
        /// Sampling Rate Procedure
        uint8_t sampling_rate;
        /// Calibration date
        struct prf_date_time factory_calibration;
    } value;
} ble_cpcp_notyf_rsp;

// cycling power service init
void ble_cppc_init(uint8_t enable);

// cycling power service start search
int8_t ble_cppc_enable(uint8_t conn_idx);

// cycling power feature read
int8_t ble_cppc_read_cpf(uint8_t conn_idx);

// sensor location read
int8_t ble_cppc_read_sen_loca(uint8_t conn_idx);

// cycling power cnotrol wirte
void ble_cpc_write_cpcp_val_handler(ble_cppc_cpcp *param);


#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

