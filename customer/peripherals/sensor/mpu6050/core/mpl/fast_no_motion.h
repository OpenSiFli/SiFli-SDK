/**
  ******************************************************************************
  * @file   fast_no_motion.h
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

#ifndef MLDMP_FAST_NO_MOTION_H__
#define MLDMP_FAST_NO_MOTION_H__

#include "mltypes.h"

#ifdef __cplusplus
extern "C" {
#endif

inv_error_t inv_enable_fast_nomot(void);
inv_error_t inv_disable_fast_nomot(void);
inv_error_t inv_start_fast_nomot(void);
inv_error_t inv_stop_fast_nomot(void);
inv_error_t inv_init_fast_nomot(void);
void inv_set_default_number_of_samples(int count);
inv_error_t inv_fast_nomot_is_enabled(unsigned char *is_enabled);
inv_error_t inv_update_fast_nomot(long *gyro);

void inv_get_fast_nomot_accel_param(long *cntr, long long *param);
void inv_get_fast_nomot_compass_param(long *cntr, long long *param);
void inv_set_fast_nomot_accel_threshold(long long thresh);
void inv_set_fast_nomot_compass_threshold(long long thresh);
void int_set_fast_nomot_gyro_threshold(long long thresh);

void inv_fnm_debug_print(void);

#ifdef __cplusplus
}
#endif


#endif // MLDMP_FAST_NO_MOTION_H__

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
