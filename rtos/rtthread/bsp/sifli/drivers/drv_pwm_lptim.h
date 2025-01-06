/**
  ******************************************************************************
  * @file   drv_pwm_lptim.h
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
#ifndef __DRV_PWM_LPTIM_H__
#define __DRV_PWM_LPTIM_H__
#include "drv_config.h"
#include "bf0_hal_lptim.h"
#if defined(BSP_USING_PWM_LPTIM1) ||defined(BSP_USING_PWM_LPTIM2)||defined(BSP_USING_PWM_LPTIM3)|| defined(_SIFLI_DOXYGEN_)

#define MAX_PERIOD 65535
#define MIN_PERIOD 3
#define MIN_PULSE 2

struct bf0_pwm_lp
{
    struct rt_device_pwm    pwm_device;    /*!<PWM device object handle*/
    LPTIM_HandleTypeDef     tim_handle;    /*!<LPTimer device object handle used in PWM*/
    struct rt_pwm_configuration config;     /*!<PWM configuration*/
    char *name;                         /*!<Device name*/
};

typedef enum
{
    LPTIME_PWM_CLK_SOURCE_USING_LPCLK       = 0,
    LPTIME_PWM_CLK_SOURCE_USING_APBCLK      = 1,
    LPTIME_PWM_CLK_SOURCE_USING_EXTER_CLK   = 2,
    LPTIME_PWM_CLK_SOURCE_CNT
} Lptime_pwm_ClockSource_Enum_T;

#endif


#endif
