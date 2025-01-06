/**
  ******************************************************************************
  * @file   phys_calc.h
  * @author Sifli software development team
  * @brief Physiological Parameter Calculation API
 * ----------------------------------------------
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


#ifndef PHYS_PARAMS_CALC_H_
#define PHYS_PARAMS_CALC_H_


#include "pp_config.h"



//If a function is deprecated it means it might be removed in the future.
// ____lifeq_deprecated__ functions have been replaced with __attribute__ ((deprecated)).
#ifndef ____lifeq_deprecated__
#define ____lifeq_deprecated__      __attribute__ ((deprecated))
#endif


//If a function has the not implemented tag it should not be called and is reserved for future implementation.
#ifndef __not_implemented__
#define __not_implemented__
#endif


//If a function has the experimental tag it is not considered ready for commercial use.
#ifndef __experimental__
#define __experimental__
#endif


/**
 * Algorithm type enum.
 */
typedef enum
{
    PP_HEART_RATE  = 0,   // code for heart rate algorithm.
    PP_HRV         = 1,   // code for Heart rate variability algorithm.
    PP_SPO2        = 2,   // code for SPO2 algorithm.
    PP_CALIBRATION = 3,   // code for calibration irrespective of the algorithm in use (while initializing, to not feed unwanted data to algorithms).
    PP_HEART_RATE_AND_HRV = 4, // code for running heart rate variability and heart rate algorithms in parallel.
    PP_SLEEP       = 5,   // code for sleep algorithm, includes heart rate variability and heart rate algorithms in parallel.
    PP_STRESS = 6,        // __not_implemented__
    PP_RR         = 7,    // only runs RR algorithm on inputs.
    PP_SKIN_DETECT = 50,  // all algorithms are switched off and input samples are evaluated for on-skin.

    PP_CALC_ALL    = 99,  // run all available algorithms. Under development.
} alg_input_selection_t;



typedef enum
{
    LQ_LED_NONE     = 100,
    LQ_LED_GREEN    = 0,
    LQ_LED_RED      = 1,
    LQ_LED_INFRARED = 2,
    LQ_LED_AMBIENT  = 3,
} lq_led_t;        /// Wave length of LED.



typedef struct input_sample
{
    uint32_t  sample;                 ///< Sample value.
    uint32_t  rf_kohm;                ///< Channel gain as kohm value.
    int16_t   isub;                   ///< only TI4404 & TI4405 otherwise 0;
    uint16_t  led_curr_mAx10;         ///< Input current in mA x 10.
    uint8_t   count;                  ///< Number of samples accumulated to get sample.
    uint8_t   num_amp;                ///< Number of trans impedance amplifiers being used.
    uint8_t   led;                    ///< Sample wavelength description.
} led_sample_t;


/**
 * pp results struct.
 */
typedef struct pp_results
{

    // -------------------------------------------------------------------------
    // alg_input_selection_t.PP_HEART_RATE
    // -------------------------------------------------------------------------
    uint8_t  lq_hr;         ///< Heart rate (bpm).
    uint8_t  lq_cadence;    ///< Cadence  (rpm).
    int8_t   hr_confidence; ///< @see PP_GetHRConfidence().

    // -------------------------------------------------------------------------
    // lifeq level 1 metrics
    // -------------------------------------------------------------------------
    uint8_t  lq_score ; // __not_implemented__;
    uint16_t lq_vo2;       ///< Oxyen consumption (l/min*100).
    uint16_t lq_epoc;      ///< (ml/kg*10) @PP_GetLifeQ_EPOC().
    uint16_t lq_lactate;   ///< Blood lactate level (mmol/l*100).
    uint16_t lq_calories;  ///< Calories burnt (kCal).
    uint16_t lq_MaxHR;     ///< max hr (bpm).
    // -------------------------------------------------------------------------
} pp_results_t;


/**
 * pp samples struct.
 */
typedef struct pp_samples
{
    int32_t accel1;
    int32_t accel2;
    int32_t accel3;
    led_sample_t ch_green;
    led_sample_t ch_ambient;
    led_sample_t ch_red;
    led_sample_t ch_infrared;
} pp_samples_t;


#endif /* defined(PHYS_PARAMS_CALC_H_) */

#ifdef __cplusplus
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
