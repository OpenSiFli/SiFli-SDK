/**
  ******************************************************************************
  * @file   mlmath.h
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

#ifndef _ML_MATH_H_
#define _ML_MATH_H_

#ifndef MLMATH
// This define makes Microsoft pickup things like M_PI
#define _USE_MATH_DEFINES
#include <math.h>

#ifdef WIN32
    // Microsoft doesn't follow standards
    #define  round(x)(((double)((long long)((x)>0?(x)+.5:(x)-.5))))
    #define roundf(x)(((float )((long long)((x)>0?(x)+.5f:(x)-.5f))))
#endif

#else  // MLMATH

#ifdef __cplusplus
extern "C" {
#endif
/* MPL needs below functions */
double  ml_asin(double);
double  ml_atan(double);
double  ml_atan2(double, double);
double  ml_log(double);
double  ml_sqrt(double);
double  ml_ceil(double);
double  ml_floor(double);
double  ml_cos(double);
double  ml_sin(double);
double  ml_acos(double);
#ifdef __cplusplus
} // extern "C"
#endif

/*
 * We rename functions here to provide the hook for other
 * customized math functions.
 */
#define sqrt(x)      ml_sqrt(x)
#define log(x)       ml_log(x)
#define asin(x)      ml_asin(x)
#define atan(x)      ml_atan(x)
#define atan2(x,y)   ml_atan2(x,y)
#define ceil(x)      ml_ceil(x)
#define floor(x)     ml_floor(x)
#define fabs(x)      (((x)<0)?-(x):(x))
#define round(x)     (((double)((long long)((x)>0?(x)+.5:(x)-.5))))
#define roundf(x)    (((float )((long long)((x)>0?(x)+.5f:(x)-.5f))))
#define cos(x)       ml_cos(x)
#define sin(x)       ml_sin(x)
#define acos(x)      ml_acos(x)

#define pow(x,y)     ml_pow(x,y)

#ifdef LINUX
    /* stubs for float version of math functions */
    #define cosf(x)      ml_cos(x)
    #define sinf(x)      ml_sin(x)
    #define atan2f(x,y)  ml_atan2(x,y)
    #define sqrtf(x)     ml_sqrt(x)
#endif



#endif // MLMATH

#ifndef M_PI
    #define M_PI 3.14159265358979
#endif

#ifndef ABS
    #define ABS(x) (((x)>=0)?(x):-(x))
#endif

#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
    #define MAX(x,y) (((x)>(y))?(x):(y))
#endif

/*---------------------------*/
#endif /* !_ML_MATH_H_ *//************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
