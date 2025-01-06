/**
  ******************************************************************************
  * @file   sensor_inven_icm20948.h
  * @author Sifli software development team
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

#ifndef SENSOR_INVEN_ICM20948_H__
#define SENSOR_INVEN_ICM20948_H__

#include "ICM20948.h"

/* define ICM-20948 Device I2C address*/
#define I2C_ADD_ICM20948_HIGH               0x69
#define I2C_ADD_ICM20948_LOW                0x68

#define ICM20948_ADDR_DEFAULT  I2C_ADD_ICM20948_LOW


/* Accelerometer full scale range */
enum icm20948_accel_range
{
    ICM20948_ACCEL_RANGE_2G  = 0, // ±2G
    ICM20948_ACCEL_RANGE_4G  = 1, // ±4G
    ICM20948_ACCEL_RANGE_8G  = 2, // ±8G
    ICM20948_ACCEL_RANGE_16G = 3  // ±16G
};

/* Gyroscope full scale range */
enum icm20948_gyro_range
{
    ICM20948_GYRO_RANGE_250DPS  = 0, // ±250°/s
    ICM20948_GYRO_RANGE_500DPS  = 1, // ±500°/s
    ICM20948_GYRO_RANGE_1000DPS = 2, // ±1000°/s
    ICM20948_GYRO_RANGE_2000DPS = 3  // ±2000°/s
};

/* Digital Low Pass Filter parameters */
enum icm20948_dlpf
{
    ICM20948_DLPF_DISABLE = 0, //256HZ
    ICM20948_DLPF_188HZ = 1,
    ICM20948_DLPF_98HZ  = 2,
    ICM20948_DLPF_42HZ  = 3,
    ICM20948_DLPF_20HZ  = 4,
    ICM20948_DLPF_10HZ  = 5,
    ICM20948_DLPF_5HZ   = 6
};

/* sleep mode parameters */
enum icm20948_sleep
{
    ICM20948_SLEEP_DISABLE = 0,
    ICM20948_SLEEP_ENABLE  = 1
};

/* Supported configuration items */
enum icm20948_cmd
{
    ICM20948_GYRO_RANGE,  /* Gyroscope full scale range */
    ICM20948_ACCEL_RANGE, /* Accelerometer full scale range */
    ICM20948_DLPF_CONFIG, /* Digital Low Pass Filter */
    ICM20948_SAMPLE_RATE, /* Sample Rate —— 16-bit unsigned value.
                            Sample Rate = [1000 -  4]HZ when dlpf is enable
                            Sample Rate = [8000 - 32]HZ when dlpf is disable */
    ICM20948_SLEEP        /* Sleep mode */
};

/* 3-axis data structure */
struct icm20948_3axes
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};

/* icm20948 config structure */
struct icm20948_config
{
    rt_uint16_t accel_range;
    rt_uint16_t gyro_range;
};

/* icm20948 device structure */
struct icm20948_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    struct icm20948_config config;
};


int rt_hw_icm20948_init(const char *name, struct rt_sensor_config *cfg);

#endif  // SENSOR_INVEN_ICM20948_H__/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
