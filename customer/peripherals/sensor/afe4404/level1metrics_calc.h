/**
  ******************************************************************************
  * @file   level1metrics_calc.h
  * @author Sifli software development team
  * @brief Level 1 metrics API is used to calculated the
 * 1Hz metrics: Calories, VO2, est Blood Latate and EPOC,
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

#ifdef __cplusplus
extern "C" {
#endif


#ifndef LEVEL1_METRICS_H_
#define LEVEL1_METRICS_H_

#include <stdbool.h>
#include <stdint.h>
#include "hqerror.h"
#include "pp_config.h"

#ifndef __lifeq_deprecated__
#if !defined(EMBEDDED) && !defined(__MINGW_ATTRIB_DEPRECATED)
#define __lifeq_deprecated__      __attribute__ ((deprecated))

#else
#define __lifeq_deprecated__
#endif
#endif


/**
 * Gender type enum.
 */
typedef enum
{
    PP_GENDER_UNDEFINED     = 0,    /// This will return an error state on PP_InitLifeQ
    PP_GENDER_MALE          = 1,
    PP_GENDER_FEMALE        = 2,
    PP_GENDER_OTHER         = 3,    /// This will default to female calculations.
} gender_t;


/**
 * LifeQ user profile enum.
 */
typedef enum
{
    LQ_AGE,                 /// AGE [0, 110] years
    LQ_WEIGHT,              /// WEIGHT [100, 50 000] in kg*100
    LQ_HEIGHT,              /// HEIGHT [2000, 25 000] in cm*100
    LQ_HEART_RATE_REST,     /// HR REST [15, 240] in bpm, (HR REST < HR MAX)
    LQ_HEART_RATE_MAX,      /// HR MAX [15, 240] in bpm, (HR REST < HR MAX)
    LQ_VO2_MAX,             /// VO2MAX [0, 1500] in (L/min)*100
    LQ_BODY_FAT,            /// BF [0, 50] in %
    LQ_GENDER,              /// (@see gender_t)
} lq_profile_t;

/**
 * LifeQ user profile struct.
 */
typedef struct LQ_User
{
    uint16_t age_years;
    uint16_t height_centimeter;
    uint16_t hr_rest_bpm;
    uint16_t hr_max_bpm;
    uint16_t vo2min_liter_pm;
    uint16_t vo2max_liter_pm;
    float bf_percentage;
    float weight_kg;
    gender_t gender;
} lq_user_t;

/**
 * HRV metrics struct.
 */
typedef struct hrv_metrics
{
    uint32_t very_low_fr;     ///< Very-low frequency components. unit=Rad/sec.
    uint32_t low_fr;          ///< Low frequency components.  unit=Rad/sec.
    uint32_t high_fr;         ///< High frequency components. unit=Rad/sec.
    uint32_t sdnn;            ///< standard deviation of RR intervals. unit=time in sec (scaled)
    uint32_t sdsd;            ///< Standard deviation of successive differences. unit=time in sec (scaled)
    uint32_t rmssd;           ///< Root mean square of successful differences. unit=time in sec (scaled)
    uint32_t pnn50;           ///< Proportion of successful differences > 50. (scaled)
    uint32_t hrv_score;       ///< Derived from RMSSD. (scaled)
} hrv_metrics_t;


/**
 * HRV calculation state enum.
 */
typedef enum
{
    PP_HRV_BUSY  = 0,       /// Deperecated use HRV_BUSY
    PP_HRV_READY = 1,       /// Deperecated use HRV_READY
    PP_HRV_FAILED = 2,      /// Deperecated use HRV_FAILED

    HRV_BUSY  = 0,
    HRV_READY = 1,
    HRV_FAILED = 2,
} hrv_state_t;



#endif /* defined(PHYS_PARAMS_CALC_H_) */

#ifdef __cplusplus
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
