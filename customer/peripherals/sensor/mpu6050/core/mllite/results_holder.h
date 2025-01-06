/**
  ******************************************************************************
  * @file   results_holder.h
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

#include "mltypes.h"

#ifndef INV_RESULTS_HOLDER_H__
#define INV_RESULTS_HOLDER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define INV_MOTION                       0x0001
#define INV_NO_MOTION                    0x0002

/**************************************************************************/
/*  The value of inv_get_gyro_sum_of_sqr is scaled such the (1 dps)^2 =   */
/*  2^GYRO_MAG_SQR_SHIFT. This number must be >=0 and even.               */
/*  The value of inv_accel_sum_of_sqr is scaled such that (1g)^2 =        */
/*  2^ACC_MAG_SQR_SHIFT                                                   */
/**************************************************************************/
#define ACC_MAG_SQR_SHIFT 16

void inv_store_gaming_quaternion(const long *quat, inv_time_t timestamp);

// States
#define SF_NORMAL 0
#define SF_UNCALIBRATED 1
#define SF_STARTUP_SETTLE 2
#define SF_FAST_SETTLE 3
#define SF_DISTURBANCE 4
#define SF_SLOW_SETTLE 5

int inv_get_acc_state(void);
void inv_set_acc_state(int state);
int inv_get_motion_state(unsigned int *cntr);
void inv_set_motion_state(unsigned char state);
inv_error_t inv_get_gravity(long *data);
inv_error_t inv_get_6axis_quaternion(long *data);
inv_error_t inv_get_quaternion(long *data);
inv_error_t inv_get_quaternion_float(float *data);
void inv_get_quaternion_set(long *data, int *accuracy, inv_time_t *timestamp);

inv_error_t inv_enable_results_holder(void);
inv_error_t inv_init_results_holder(void);

/* Magnetic Field Parameters*/
void inv_set_local_field(const long *data);
void inv_get_local_field(long *data);
void inv_set_mag_scale(const long *data);
void inv_get_mag_scale(long *data);
void inv_set_compass_correction(const long *data, inv_time_t timestamp);
void inv_get_compass_correction(long *data, inv_time_t *timestamp);
int inv_got_compass_bias(void);
void inv_set_compass_bias_found(int state);
int inv_get_large_mag_field(void);
void inv_set_large_mag_field(int state);
void inv_set_compass_state(int state);
int inv_get_compass_state(void);
void inv_set_compass_bias_error(const long *bias_error);
void inv_get_compass_bias_error(long *bias_error);
inv_error_t inv_get_linear_accel(long *data);
inv_error_t inv_get_accel(long *data);
inv_error_t inv_get_accel_float(float *data);
inv_error_t inv_get_gyro_float(float *data);
inv_error_t inv_get_linear_accel_float(float *data);
void inv_set_heading_confidence_interval(float ci);
float inv_get_heading_confidence_interval(void);

int inv_got_accel_bias(void);
void inv_set_accel_bias_found(int state);


#ifdef __cplusplus
}
#endif

#endif // INV_RESULTS_HOLDER_H__/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
