/**
  ******************************************************************************
  * @file   lsm6dsl.h
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

#ifndef __LSM6DSL_SENSOR_HDR_FILE__
#define __LSM6DSL_SENSOR_HDR_FILE__
#include "lsm6dsl_reg.h"

typedef enum
{
    LSM6DSL_FIFO_ODR_DISABLE   =  0,
    LSM6DSL_FIFO_ODR_12Hz5     =  1,
    LSM6DSL_FIFO_ODR_26Hz      =  2,
    LSM6DSL_FIFO_ODR_52Hz      =  3,
    LSM6DSL_FIFO_ODR_104Hz     =  4,
    LSM6DSL_FIFO_ODR_208Hz     =  5,
    LSM6DSL_FIFO_ODR_416Hz     =  6,
    LSM6DSL_FIFO_ODR_833Hz     =  7,
    LSM6DSL_FIFO_ODR_1k66Hz    =  8,
    LSM6DSL_FIFO_ODR_3k33Hz    =  9,
    LSM6DSL_FIFO_ODR_6k66Hz    = 10,
    LSM6DSL_FIFO_ODR_RATE_ND   = 11,    /* ERROR CODE */
} lsm6dsl_fifo_odr_t;

typedef enum
{
    LSM6DSL_FIFO_GYRO       = 0,
    LSM6DSL_FIFO_XL         = 1,
    LSM6DSL_FIFO_TEMP       = 2,
    LSM6DSL_FIFO_STEP       = 3,
    LSM_6DSL_FIFO_CNT
} lsm6dsl_fifo_func_t;

typedef enum
{
#ifdef USING_GYRO_SENSOR
    LSM6DSL_FIFO_PATTERN_GX1  = 0,
    LSM6DSL_FIFO_PATTERN_GY2  = 1,
    LSM6DSL_FIFO_PATTERN_GZ3  = 2,
    LSM6DSL_FIFO_PATTERN_XLX1 = 3,
    LSM6DSL_FIFO_PATTERN_XLY2 = 4,
    LSM6DSL_FIFO_PATTERN_XLZ3 = 5,
#else
    LSM6DSL_FIFO_PATTERN_XLX1 = 0,
    LSM6DSL_FIFO_PATTERN_XLY2 = 1,
    LSM6DSL_FIFO_PATTERN_XLZ3 = 2,
#endif
} lsm6dsl_fifo_pattern_id_t;


int lsm6dsl_init(void);
uint32_t lsm6dsl_get_bus_handle(void);
uint8_t lsm6dsl_get_dev_addr(void);
uint8_t lsm6dsl_get_dev_id(void);
int lsm6dsl_open(void);
int lsm6dsl_close(void);

int lsm6dsl_fifo_enable(lsm6dsl_fifo_func_t func, lsm6dsl_fifo_odr_t rate);
int lsm6dsl_fifo_disable(lsm6dsl_fifo_func_t func);
int lsm6dsl_get_fifo_count(void);
int lsm6dsl_read_fifo(uint8_t *buf, int len);
int lsm6dsl_set_fifo_threshold(int thd);
int lsm6dsl_get_waterm_status(void);
int lsm6dsl_get_overrun_status(void);
int lsm6dsl_get_fifo_full_status(void);
int lsm6dsl_get_fifo_empty_status(void);
int lsm6dsl_set_fifo_mode(lsm6dsl_fifo_mode_t val);
int lsm6dsl_get_fifo_pattern(void);


int lsm6dsl_get_fifo_data_arr(void);

int lsm6dsl_awt_enable(int en);
int lsm6dsl_pedo_enable(int en);
int lsm6dsl_pedo_fifo2step(uint8_t *buf, int len);


int lsm6dsl_gyro_read(int *psX, int *psY, int *psZ);
int lsm6dsl_accel_read(int *psX, int *psY, int *psZ);
int lsm6dsl_tempra_read(float *tempra);
int lsm6dsl_step_read(int32_t *step);


void lsm6dsl_accel_set_range(uint8_t range);
void lsm6dsl_gyro_set_range(uint8_t range);

int lsm6dsl_self_check(void);

#endif /* __LSM6DSL_SENSOR_HDR_FILE__*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
