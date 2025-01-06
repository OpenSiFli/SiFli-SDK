/**
  ******************************************************************************
  * @file   hal_outputs.h
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

#ifndef INV_HAL_OUTPUTS_H__
#define INV_HAL_OUTPUTS_H__

#ifdef __cplusplus
extern "C" {
#endif

int inv_get_sensor_type_orientation(float *values, int8_t *accuracy,
                                    inv_time_t *timestamp);
int inv_get_sensor_type_accelerometer(float *values, int8_t *accuracy,
                                      inv_time_t *timestamp);
int inv_get_sensor_type_gyroscope(float *values, int8_t *accuracy,
                                  inv_time_t *timestamp);
int inv_get_sensor_type_gyroscope_raw(float *values, int8_t *accuracy,
                                      inv_time_t *timestamp);
int inv_get_sensor_type_magnetic_field(float *values, int8_t *accuracy,
                                       inv_time_t *timestamp);
int inv_get_sensor_type_rotation_vector(float *values, int8_t *accuracy,
                                        inv_time_t *timestamp);

int inv_get_sensor_type_linear_acceleration(float *values,
        int8_t *accuracy,
        inv_time_t *timestamp);
int inv_get_sensor_type_gravity(float *values, int8_t *accuracy,
                                inv_time_t *timestamp);

inv_error_t inv_enable_hal_outputs(void);
inv_error_t inv_disable_hal_outputs(void);
inv_error_t inv_init_hal_outputs(void);
inv_error_t inv_start_hal_outputs(void);
inv_error_t inv_stop_hal_outputs(void);

#ifdef __cplusplus
}
#endif

#endif                          // INV_HAL_OUTPUTS_H__/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
