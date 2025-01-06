/**
  ******************************************************************************
  * @file   pp_config.h
  * @author Sifli software development team
  * @brief Heart rate algorithm configuration API
 * ---------------------------------------------
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

#ifndef __lifeqinside__lifeq_config__
#define __lifeqinside__lifeq_config__

//#include <stdbool.h>
#include <stdint.h>
#include "hqerror.h"


#if (defined(EMBEDDED_ARM) && ~defined(EMBEDDED))
    #define EMBEDDED
#endif


//If a function is marked for removal or not supported in future releases.
#ifdef __MINGW_ATTRIB_DEPRECATED
    #define __lifeq_deprecated__      __MINGW_ATTRIB_DEPRECATED
#endif

#ifndef __lifeq_deprecated__
    #if !defined(EMBEDDED) && !defined(__MINGW_ATTRIB_DEPRECATED)
        #define __lifeq_deprecated__      __attribute__ ((deprecated))
    #else
        #define __lifeq_deprecated__
    #endif
#endif



//If a function has the not implemented tag it should not be called and is reserved for future implementation.
#ifndef __not_implemented__
    #define __not_implemented__
#endif


//If a function has the experimental tag it is not considered ready for commercial use.
#ifndef __experimental__
    #define __experimental__
#endif

#define CONFIG_OFF          0   /// see config_t
#define CONFIG_ON           1   //  see config_t
#define CONFIG_DEFAULT      2   //  see config_t

typedef enum
{
    OFF     = CONFIG_OFF,       /// Property is disabled.
    ON      = CONFIG_ON,        /// Property is activated.
    DEFAULT = CONFIG_DEFAULT    /// Property default value is set.
} config_t;         /// Configuration options applied properties.


/**
 * Available window durations.
 */
typedef enum
{
    PP_CONFIG_WINDOW_UNSET =  0,
    PP_CONFIG_WINDOW_1SEC  =  1000,   /// Select a 1 second window.
    PP_CONFIG_WINDOW_2SEC  =  2000,   /// Select a 2 second window.
    PP_CONFIG_WINDOW_3SEC  =  3000,   /// Select a 3 second window.
    PP_CONFIG_WINDOW_5SEC  =  5000,   /// Select a 5 second window.
    PP_CONFIG_WINDOW_7SEC  =  7000,   /// Select a 5 second window.
    PP_CONFIG_WINDOW_10SEC = 10000,   /// Select a 10 second window.
    PP_CONFIG_WINDOW_20SEC = 20000,   /// Select a 20 second window.
    PP_CONFIG_WINDOW_30SEC = 30000,   /// Select a 30 second window.
    PP_CONFIG_WINDOW_40SEC = 40000,   /// Select a 40 second window.
    PP_CONFIG_WINDOW_50SEC = 50000    /// Select a 50 second window.
} config_sec_t;                       /// Collection of default timing options.


/**
 * LifeQ algorithm operation modes.
 */
typedef enum
{
    PP_CONFIG_LIFE_STYLE  = 0x00,    /// Non-exercise to aid calorie and vo2 calculations.
    PP_CONFIG_EXERCISE    = 0xB0,    /// ACTIVITY_EXERCISE_OTHER, unspecified activity.
    PP_CONFIG_WALKING     = 0x50,    /// Parameters optimised for walking-type exercise.
    PP_CONFIG_RUNNING     = 0x60,    /// Parameters optimised for running-type exercise.
    PP_CONFIG_ROWING      = 0xB1,    /// Parameters optimised for rowing-type exercise.
    PP_CONFIG_CYCLING     = 0xB3,    /// Parameters optimised for cycling-type exercise.
    PP_CONFIG_GYM_WEIGHT  = 0xA1,    /// Parameters optimised for weight-type exercise.

    PP_CONFIG_SLEEP       = 0x20,    /// Parameters optimised for sleep.
    //    PP_CONFIG_SWIMMING  =  13,      __not_implemented__

    PP_CONFIG_AUTO        = 0xFF,    /// Auto detect user activity

} config_mode_t;


/**
 * Available input data speed rates. @see PP_Init and PP_Set_Fs
 */
typedef enum
{
    DEVICE_DATA_INPUT_FS_25HZ  =  25,
    DEVICE_DATA_INPUT_FS_50HZ  =  50,
    DEVICE_DATA_INPUT_FS_128HZ = 128,
} config_input_fs_t ;


/**
 * Available accelerometer scaling ranges. @see PP_Set_Gs
 */
typedef enum
{
    DEVICE_CONFIG_ACC_2G  =  2,
    DEVICE_CONFIG_ACC_4G  =  4,
    DEVICE_CONFIG_ACC_6G  =  6,
    DEVICE_CONFIG_ACC_8G  =  8,
} config_g_t;


typedef enum
{
    AFE_TI4404  = 0,
    AFE_TI4405  = 1,
    AFE_A = 10,
} config_afe_t;


/**
 * Available LED wavelengths. @see PP_Set_SkinDetectWavelength
 */
typedef enum
{
    LED_WL_GREEN,
    LED_WL_RED,
    LED_WL_IRED
} led_wl_t;


typedef enum
{
    CONFIG_WEAR_DEVICE_DISABLED  = 0,     ///< Default value, uses skin detection on leds.
    CONFIG_WEAR_DEVICE_OFF  = 1,          ///< Overrides the sensor on skin detection as OFF.
    CONFIG_WEAR_DEVICE_ON  = 2,           ///< Overrides the sensor on skin detection as ON.
} config_wear_detection_t;



#endif /* defined(__lifeqinside__lifeq_config__) */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
