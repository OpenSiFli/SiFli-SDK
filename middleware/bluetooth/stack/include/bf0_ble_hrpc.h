/**
  ******************************************************************************
  * @file   bf0_ble_hrpc.h
  * @author Sifli software development team
  * @brief Header file - Sibles heart rate service collector definition.
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


#ifndef __BF0_BLE_HRPC_H
#define __BF0_BLE_HRPC_H

#include "bf0_ble_common.h"

/// maximum number of RR-Interval supported
#define HRS_MAX_RR_INTERVAL  (4)

enum
{
    HR_OTHER = 0X00,
    HR_CHEST,
    HR_WRIST,
    HR_FINGER,
    HR_HAND,
    HR_EARLOBE,
    HR_FOOT,
};




enum ble_hrpc_event
{
    // Remote heart rate measurement Notification see @ble_hrpc_heart_rate_t
    BLE_HRPC_HREAT_RATE_NOTIFY = BLE_HRP_TYPE,
    // Read remote body sensor location response see @ble_hrpc_body_sensor_location_t
    BLE_HRPC_READ_BODY_SENSOR_LOCATION_RSP,
};
/// Heart Rate Measurement Flags field bit values
enum
{
    /// Heart Rate Value Format bit
    /// Heart Rate Value Format is set to UINT8. Units: beats per minute (bpm)
    HRS_FLAG_HR_8BITS_VALUE                = 0x00,
    /// Heart Rate Value Format is set to UINT16. Units: beats per minute (bpm)
    HRS_FLAG_HR_16BITS_VALUE               = 0x01,

    /// Sensor Contact Status bits
    /// Sensor Contact feature is not supported in the current connection
    HRS_FLAG_SENSOR_CCT_FET_NOT_SUPPORTED  = 0x00,
    /// Sensor Contact feature supported in the current connection
    HRS_FLAG_SENSOR_CCT_FET_SUPPORTED      = 0x04,
    /// Contact is not detected
    HRS_FLAG_SENSOR_CCT_NOT_DETECTED       = 0x00,
    /// Contact is detected
    HRS_FLAG_SENSOR_CCT_DETECTED           = 0x02,

    /// Energy Expended Status bit
    /// Energy Expended field is not present
    HRS_FLAG_ENERGY_EXPENDED_NOT_PRESENT   = 0x00,
    /// Energy Expended field is present. Units: kilo Joules
    HRS_FLAG_ENERGY_EXPENDED_PRESENT       = 0x08,

    /// RR-Interval bit
    /// RR-Interval values are not present.
    HRS_FLAG_RR_INTERVAL_NOT_PRESENT       = 0x00,
    /// One or more RR-Interval values are present. Units: 1/1024 seconds
    HRS_FLAG_RR_INTERVAL_PRESENT           = 0x10,
};

/// Heart Rate Feature Flags field bit values
enum
{
    /// Body Sensor Location support bit
    /// Body Sensor Location feature Not Supported
    HRS_F_BODY_SENSOR_LOCATION_NOT_SUPPORTED     = 0x00,
    /// Body Sensor Location feature Supported
    HRS_F_BODY_SENSOR_LOCATION_SUPPORTED         = 0x01,

    /// Energy Expended support bit
    /// Energy Expended feature Not Supported
    HRS_F_ENERGY_EXPENDED_NOT_SUPPORTED          = 0x00,
    /// Energy Expended feature Supported
    HRS_F_ENERGY_EXPENDED_SUPPORTED              = 0x02,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Heart Rate measurement structure
typedef struct
{
    /// Flag
    uint8_t flags;
    /// Heart Rate Measurement Value
    uint16_t heart_rate;
    /// Energy Expended
    uint16_t energy_expended;
    /// RR-Interval numbers (max 4)
    uint8_t nb_rr_interval;
    /// RR-Intervals
    uint16_t rr_intervals[HRS_MAX_RR_INTERVAL];
} ble_hrpc_heart_rate_t;


/**
 * @brief The structure of BLE_HRPC_READ_BODY_SENSOR_LOCATION_RSP.
 */
/// body sensor location structure
typedef struct
{
    /// body sensor loction
    uint8_t location;

} ble_hrpc_body_sensor_location_t;


/**
 * @brief initiate heart rate profile collector
   @param[in] is_enable whether enabled.
 */
void ble_hrpc_init(uint8_t enable);


/**
 * @brief Enable CCCD notify of heart rate measurement.
   @param[in] conn_idx connection index.
   @param[in] is_enable whether enabled.
   @retval result 0 is successful.
 */
int8_t ble_hrpc_enable_read_heart_rate(uint8_t conn_idx, uint8_t enable);


/**
 * @brief read remote body sensor location.
   @param[in] conn_idx connection index.
   @retval result 0 is successful.
 */
int8_t ble_hrpc_read_body_sensor_location(uint8_t conn_idx);


/**
 * @brief reset remote energy expended.
   @param[in] conn_idx connection index.
   @retval result SIBLES_WRITE_NO_ERR is successful.
 */
int8_t ble_hrpc_write_heart_rate_cntl_point(uint8_t conn_idx);


/**
 * @brief search remote heart rate profile .
   @param[in] conn_idx connection index.
   @retval result 0 is successful.
 */
int8_t ble_hrpc_enable(uint8_t conn_idx);


#endif //__BF0_BLE_HRPC_H


