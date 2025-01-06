/**
  ******************************************************************************
  * @file   gsensor_service.h
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

#ifndef _GSENSOR_SERVICE_H_
#define _GSENSOR_SERVICE_H_

#include "board.h"
#include "sensor.h"

#include "lsm6dsl.h"
#include "lsm6dsl_reg.h"


#define GSENSOR_FIFO_SIZE                   (52 / 4)
#define SENSORS_ALGO_BUF_ARRAY              10      // odr 50hz, run 200ms interval


/* define LSM6DSL Device I2C address*/


/* Accelerometer full scale range */
enum lsm6dsl_accel_range
{
    LSM6DSL_ACCEL_RANGE_2G  = 0, // ±2G
    LSM6DSL_ACCEL_RANGE_4G  = 1, // ±4G
    LSM6DSL_ACCEL_RANGE_8G  = 2, // ±8G
    LSM6DSL_ACCEL_RANGE_16G = 3  // ±16G
};

/* Gyroscope full scale range */
enum lsm6dsl_gyro_range
{
    LSM6DSL_GYRO_RANGE_250DPS  = 0, // ±250°/s
    LSM6DSL_GYRO_RANGE_500DPS  = 1, // ±500°/s
    LSM6DSL_GYRO_RANGE_1000DPS = 2, // ±1000°/s
    LSM6DSL_GYRO_RANGE_2000DPS = 3  // ±2000°/s
};

/* Digital Low Pass Filter parameters */
enum lsm6dsl_dlpf
{
    LSM6DSL_DLPF_DISABLE = 0, //256HZ
    LSM6DSL_DLPF_188HZ = 1,
    LSM6DSL_DLPF_98HZ  = 2,
    LSM6DSL_DLPF_42HZ  = 3,
    LSM6DSL_DLPF_20HZ  = 4,
    LSM6DSL_DLPF_10HZ  = 5,
    LSM6DSL_DLPF_5HZ   = 6
};

/* sleep mode parameters */
enum lsm6dsl_sleep
{
    LSM6DSL_SLEEP_DISABLE = 0,
    LSM6DSL_SLEEP_ENABLE  = 1
};

/* Supported configuration items */
enum lsm6dsl_cmd
{
    LSM6DSL_GYRO_RANGE,  /* Gyroscope full scale range */
    LSM6DSL_ACCEL_RANGE, /* Accelerometer full scale range */
    LSM6DSL_DLPF_CONFIG, /* Digital Low Pass Filter */
    LSM6DSL_SAMPLE_RATE, /* Sample Rate —— 16-bit unsigned value.
                            Sample Rate = [1000 -  4]HZ when dlpf is enable
                            Sample Rate = [8000 - 32]HZ when dlpf is disable */
    LSM6DSL_SLEEP        /* Sleep mode */
};

/* 3-axis data structure */
struct lsm6dsl_3axes
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};

/* icm20948 config structure */
struct lsm6dsl_config
{
    rt_uint16_t accel_range;
#ifdef USING_GYRO_SENSOR
    rt_uint16_t gyro_range;
#endif
};

/* icm20948 device structure */
struct lsm6dsl_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    struct lsm6dsl_config config;
};

int rt_hw_lsm6dsl_register(const char *name, struct rt_sensor_config *cfg);
int rt_hw_lsm6dsl_init(void);
int rt_hw_lsm6dsl_deinit(void);


#endif  // SENSOR_ST_LSM6DSL_H__/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
