/**
  ******************************************************************************
  * @file   bf0_hal_math.h
  * @author Sifli software development team
  * @brief   Header file of FFT HAL module.
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_MATH_H
#define __BF0_HAL_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SF32LB52X
#if defined(__CC_ARM) || defined(__CLANG_ARM) || (defined(__GNUC__) && (__GNUC__ > 9))
#define HAL_MATH_CDE_SUPPORT
#endif /* defined(__CC_ARM) || defined(__CLANG_ARM) || (defined(__GNUC__) && (__GNUC__ > 9)) */
#endif /* SF32LB52X */

/* Includes ------------------------------------------------------------------*/
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#include <arm_acle.h>
#ifdef HAL_MATH_CDE_SUPPORT
#include <arm_cde.h>
#endif /* HAL_MATH_CDE_SUPPORT */
#elif defined(__ICCARM__)
#include <intrinsics.h>
#elif defined(__GNUC__)
#include <arm_acle.h>
#ifdef HAL_MATH_CDE_SUPPORT
#include <arm_cde.h>
#endif /* HAL_MATH_CDE_SUPPORT */
#endif /* defined(__CC_ARM) */

#include "bf0_hal_def.h"

uint32_t __arm_cx2(int, uint32_t, uint32_t);
uint32_t __arm_cx2a(int, uint32_t, uint32_t, uint32_t);
uint64_t __arm_cx2d(int, uint32_t, uint32_t);
uint64_t __arm_cx2da(int, uint64_t, uint32_t, uint32_t);

/** @addtogroup MATH
  * @ingroup BF0_HAL_Driver
  * @{
  */

__STATIC_INLINE uint32_t HAL_MATH_DisableInterrupt(void)
{
    uint32_t mask;

    mask = __get_PRIMASK();
    __set_PRIMASK(1);
    return mask;
}

__STATIC_INLINE void HAL_MATH_EnableInterrupt(uint32_t mask)
{
    __set_PRIMASK(mask);
}


/** normal coprocessor for math */
#define CP_MATH_NORMAL   (2)
/** coprocessor used by CDE */
#define CP_MATH_CDE      (1)

#define CP_MATH_NORM_OP_SET_XY         (0)
#define CP_MATH_NORM_OP_SET_X          (0)
#define CP_MATH_NORM_OP_SET_Y          (1)
#define CP_MATH_NORM_OP_SET_ANGLE      (2)
#define CP_MATH_NORM_OP_SET_ITER_NUM   (3)
#define CP_MATH_NORM_OP_SET_SCALE      (4)
#define CP_MATH_NORM_OP_SET_MODE       (5)

#define CP_MATH_NORM_MODE_COS          (1)
#define CP_MATH_NORM_MODE_SIN          (2)
#define CP_MATH_NORM_MODE_ATAN         (3)
#define CP_MATH_NORM_MODE_MOD          (4)
#define CP_MATH_NORM_MODE_ANG          (5)
#define CP_MATH_NORM_MODE_ROT          (6)
#define CP_MATH_NORM_MODE_MUL          (7)
#define CP_MATH_NORM_MODE_DIV          (8)
#define CP_MATH_NORM_MODE_COSH         (9)
#define CP_MATH_NORM_MODE_SINH         (10)
#define CP_MATH_NORM_MODE_ATANH        (11)
#define CP_MATH_NORM_MODE_MODH         (12)
#define CP_MATH_NORM_MODE_ANGH         (13)
#define CP_MATH_NORM_MODE_SQRT         (14)
#define CP_MATH_NORM_MODE_EXP          (15)
#define CP_MATH_NORM_MODE_LN           (16)

#define CP_MATH_CDE_MODE_COS      (0)
#define CP_MATH_CDE_MODE_SIN      (0)
#define CP_MATH_CDE_MODE_ATAN     (0)
#define CP_MATH_CDE_MODE_MOD      (1)
#define CP_MATH_CDE_MODE_ANG      (2)
#define CP_MATH_CDE_MODE_ROT      (3)
#define CP_MATH_CDE_MODE_MUL      (4)
#define CP_MATH_CDE_MODE_DIV      (5)
#define CP_MATH_CDE_MODE_COSH     (6)
#define CP_MATH_CDE_MODE_SINH     (7)
#define CP_MATH_CDE_MODE_ATANH    (8)
#define CP_MATH_CDE_MODE_MODH     (9)
#define CP_MATH_CDE_MODE_ANGH     (10)
#define CP_MATH_CDE_MODE_SQRT     (11)
#define CP_MATH_CDE_MODE_EXP      (12)
#define CP_MATH_CDE_MODE_LN       (13)


/**
 * @brief  Move x and y to 64bit register, y is put in high 32bit and x is put in low 32bit
 *
 * @param[in] x x value
 * @param[in] y y value
 *
 * @return void
 */
#define CP_MATH_SET_XY(x,y)       __arm_mcrr(CP_MATH_NORMAL, CP_MATH_NORM_OP_SET_XY, (((uint64_t)((uint32_t)(y))) << 32) | ((uint32_t)(x)), 0)
#define CP_MATH_SET_X(x)          __arm_mcr(CP_MATH_NORMAL, CP_MATH_NORM_OP_SET_X, (x), 0, 0, 0)
#define CP_MATH_SET_Y(y)          __arm_mcr(CP_MATH_NORMAL, CP_MATH_NORM_OP_SET_Y, (y), 0, 0, 0)
#define CP_MATH_SET_ANGLE(angle)  __arm_mcr(CP_MATH_NORMAL, CP_MATH_NORM_OP_SET_ANGLE, (angle), 0, 1, 2)
#define CP_MATH_SET_ITER_NUM(num) __arm_mcr(CP_MATH_NORMAL, CP_MATH_NORM_OP_SET_ITER_NUM, (num), 0, 1, 3)
#define CP_MATH_SET_SCALE(scale)  __arm_mcr(CP_MATH_NORMAL, CP_MATH_NORM_OP_SET_SCALE, (scale), 0, 0, 0)
#define CP_MATH_SET_MODE(mode)    __arm_mcr(CP_MATH_NORMAL, CP_MATH_NORM_OP_SET_MODE, (mode), 0, 1, 4)

#define CP_MATH_GET_RESULT_UINT64() __arm_mrrc(CP_MATH_NORMAL, 0, 0)
#define CP_MATH_GET_SCALE_AND_VAL(arg_result, arg_scale, arg_val)   \
    do                                                  \
    {                                                   \
        if (arg_scale)                                  \
        {                                               \
            *arg_scale = (int8_t)((arg_result >> 32) & UINT8_MAX);  \
        }                                                           \
        if (arg_val)                                                \
        {                                                           \
            *arg_val = (int32_t)(arg_result & UINT32_MAX);          \
        }                                                           \
    }                                                               \
    while (0)

#define CP_MATH_CDE_MAKE_CTRL_CODE(mode, iter_num)   ((((uint8_t)mode) & 0xF) | (((uint8_t)iter_num & 0x1F) << 4))


/**
 * @brief  Calculate cosine and sine of an angle
 *
 * @param[in] angle angle in radians, devided by pi, Q1.31 format, range: [-1, 1], it's mapped to [-pi, pi]
 * @param[out] cos_val cosine of the angle, Q1.31 format
 * @param[out] sin_val sine of the angle, Q1.31 format
 *
 * @retval cosine value
 */
__STATIC_INLINE void HAL_CP_CosSin(int32_t angle, int32_t *cos_val, int32_t *sin_val)
{
    uint64_t cos_sin;

    CP_MATH_SET_ANGLE(angle);
    CP_MATH_SET_ITER_NUM(11);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_COS);

    cos_sin = CP_MATH_GET_RESULT_UINT64();
    if (cos_val)
    {
        *cos_val = (int32_t)(cos_sin & UINT32_MAX);
    }
    if (sin_val)
    {
        *sin_val = (int32_t)((cos_sin >> 32) & UINT32_MAX);
    }

    return;
}

/**
 * @brief  Calculate sine of an angle
 *
 * @param[in] angle angle in radians, devided by pi, Q1.31 format, range: [-1, 1], it's mapped to [-pi, pi]
 *
 * @retval sine value in Q1.31 format
 */
__STATIC_INLINE int32_t HAL_CP_Sin(int32_t angle)
{
    int32_t result;

    CP_MATH_SET_ANGLE(angle);
    CP_MATH_SET_ITER_NUM(11);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_SIN);

    result = (int32_t)(CP_MATH_GET_RESULT_UINT64() & UINT32_MAX);

    return result;
}

/**
 * @brief  Calculate arctangent
 *         formula: atan*pi=atan(data*(2^scale))
 *
 * @param[in] scale scaling factor, range is [0,12]
 * @param[in] data data in Q1.31 format
 *
 * @retval arctangent in radians, divided by pi, Q1.31 format, range is [-1, 1] which is mapped to [-pi, pi]
 */
__STATIC_INLINE int32_t HAL_CP_Atan(int8_t scale, int32_t data)
{
    int32_t result;

    CP_MATH_SET_Y(data);
    CP_MATH_SET_ITER_NUM(11);
    CP_MATH_SET_SCALE(scale);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_ATAN);

    result = (int32_t)(CP_MATH_GET_RESULT_UINT64() & UINT32_MAX);
    return result;
}

/**
 * @brief  Calculate magnitude(i.e. modulus) of the vector v=(x,y),
 *         formula: magnitude=mod*(2^scale)=sqrt(x^2 + y^2)
 *
 * @param[in] x x coordinate, Q1.31 format
 * @param[in] y y coordinate, Q1.31 format
 * @param[out] scale points to output scaling value, range is [-30, 1]
 * @param[out] mod points to output modulus, Q1.31 format
 *
 * @retval void
 */
__STATIC_INLINE void HAL_CP_Mod(int32_t x, int32_t y, int8_t *scale, int32_t *mod)
{
    uint64_t result ;

    CP_MATH_SET_XY(x, y);
    CP_MATH_SET_ITER_NUM(11);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_MOD);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, scale, mod);

    return;
}

/**
 * @brief  Calculate phase angle of the vector v=(x,y),
 *         formula: angle*pi=arctangent(y/x)
 *
 * @param[in] x x coordinate, Q1.31 format
 * @param[in] y y coordinate, Q1.31 format
 *
 * @retval angle in radians devided by pi, range is [-1,1] which is mapped to [-pi, pi]
 */
__STATIC_INLINE int32_t HAL_CP_Ang(int32_t x, int32_t y)
{
    uint64_t result ;

    CP_MATH_SET_XY(x, y);
    CP_MATH_SET_ITER_NUM(11);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_ANG);

    result = CP_MATH_GET_RESULT_UINT64();
    return (result & UINT32_MAX);
}

/**
 * @brief  Calculate the rotated vector
 *
 * @param[in] x x coordinate, Q1.31 format, range is (-0.707, 0.707)
 * @param[in] y y coordinate, Q1.31 format, range is (-0.707, 0.707)
 * @param[in] angle angle in radians, devided by pi, Q1.31 format, range: [-1, 1], which is mapped to [-pi, pi]
 * @param[out] x2 points to x coordinate of the rotated vector
 * @param[out] y2 points to y coordinate of the rotated vector

 * @return void
 */
__STATIC_INLINE void HAL_CP_Rot(int32_t x, int32_t y, int32_t angle, int32_t *x2, int32_t *y2)
{
    uint64_t result;

    CP_MATH_SET_XY(x, y);
    CP_MATH_SET_ANGLE(angle);
    CP_MATH_SET_ITER_NUM(11);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_ROT);

    result = CP_MATH_GET_RESULT_UINT64();
    if (x2)
    {
        *x2 = (int32_t)(result & UINT32_MAX);
    }
    if (y2)
    {
        *y2 = (int32_t)(result >> 32);
    }

    return;
}

/**
 * @brief  Multiple data0 by data1
 *         formula: val*(2^scale)=data0*data1
 *
 * @param[in] data0 data0 in Q1.31 format
 * @param[in] data1 data1 in Q1.31 format
 * @param[out] scale points to scaling factor, Q8.0 format, range is [-60,0]
 * @param[out] val points to output value, Q1.31 format

 * @return void
 */
__STATIC_INLINE void HAL_CP_Mul(int32_t data0, int32_t data1, int8_t *scale, int32_t *val)
{
    uint64_t result;

    CP_MATH_SET_XY(data0, data1);
    CP_MATH_SET_ITER_NUM(11);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_MUL);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, scale, val);

    return;
}


/**
 * @brief  Divide numeritor by denominator
 *         formula: num/den=val*(2^scale)
 *
 * @param[in] num numerator in Q1.31 format
 * @param[in] den denominator in Q1.31 format
 * @param[out] scale points to scaling factor, Q8.0 format, range is [-30,30]
 * @param[out] val points to output vallue, Q1.31 format

 * @return void
 */
__STATIC_INLINE void HAL_CP_Div(int32_t num, int32_t den, int8_t *scale, int32_t *val)
{
    uint64_t result;

    CP_MATH_SET_XY(den, num);
    CP_MATH_SET_ITER_NUM(11);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_DIV);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, scale, val);

    return;
}

/**
 * @brief  Calculate cosineh of an angle
 *         formula: cosh*(2^output_scale)=cosh(angle*(2^scale)),
 *                  angle*(2^scale) range is (-1.118, 1.118)
 *
 * @param[in] scale scaling factor, Q8.0 format, range is [0, 1]
 * @param[in] angle angle in Q1.31 format, range: [-1, 1)
 * @param[out] output_scale points to output scaling factor, Q8.0 format, range is [0, 1]
 * @param[out] cosh points to output cosh value
 *
 * @return void
 */
__STATIC_INLINE void HAL_CP_Cosh(int8_t scale, int32_t angle, int8_t *output_scale, int32_t *cosh)
{
    uint64_t result;

    CP_MATH_SET_ANGLE(angle);
    CP_MATH_SET_SCALE(scale);
    CP_MATH_SET_ITER_NUM(12);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_COSH);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, cosh);

    return;
}

/**
 * @brief  Calculate sineh of an angle
 *         formula: sinh*(2^output_scale)=sinh(angle*(2^scale)),
 *                  angle*(2^scale) range is (-1.118, 1.118)
 *
 * @param[in] scale scaling factor, Q8.0 format, range is [0,1]
 * @param[in] angle angle in Q1.31 format, range: [-1, 1)
 * @param[out] output_scale points to output scaling factor, Q8.0 format, range is [0, 1]
 * @param[out] sinh points to output sinh value
 *
 * @return void
 */
__STATIC_INLINE void HAL_CP_Sinh(int8_t scale, int32_t angle, int8_t *output_scale, int32_t *sinh)
{
    uint64_t result;

    CP_MATH_SET_ANGLE(angle);
    CP_MATH_SET_SCALE(scale);
    CP_MATH_SET_ITER_NUM(12);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_SINH);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, sinh);

    return;
}

/**
 * @brief  Calculate atanh of data, data range is (-0.806, 0.806)
 *         formula: atanh*(2^scale)=atanh(data)
 *
 * @param[in] data data in Q1.31 format
 * @param[out] scale points to output scaling factor, Q8.0 format, range is [0, 1]
 * @param[out] atanh points to output atanh value
 *
 * @return void
 */
__STATIC_INLINE void HAL_CP_Atanh(int32_t data, int8_t *scale, int32_t *atanh)
{
    uint64_t result;

    CP_MATH_SET_Y(data);
    CP_MATH_SET_ITER_NUM(12);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_ATANH);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, scale, atanh);

    return;
}

/**
 * @brief  Calculate sqrt(x^2-y^2)
 *         modh*(2^scale)=sqrt(x^2-y^2),
 *         x range is (-1, 1), y range is (-0.806, 0.806), and abs(x)>=abs(y)
 *
 * @param[in] x x coordinate, Q1.31 format
 * @param[in] y y coordinate, Q1.31 format
 * @param[out] scale points to output scaling factor in Q8.0 format
 * @param[out] modh points to output modulush, Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CP_Modh(int32_t x, int32_t y, int8_t *scale, int32_t *modh)
{
    uint64_t result;

    CP_MATH_SET_XY(x, y);
    CP_MATH_SET_ITER_NUM(12);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_MODH);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, scale, modh);

    return;
}

/**
 * @brief  Calculate arctanh(y/x)
 *         angh*(2^scale)=arctanh(y/x),
 *         x range is (-1, 1), y range is (-0.806, 0.806), and abs(x)>=abs(y)
 *
 * @param[in] x x coordinate, Q1.31 format
 * @param[in] y y coordinate, Q1.31 format
 * @param[out] scale points to output scaling factor in Q8.0 format
 * @param[out] angh points to output angh, Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CP_Angh(int32_t x, int32_t y, int8_t *scale, int32_t *angh)
{
    uint64_t result;

    CP_MATH_SET_XY(x, y);
    CP_MATH_SET_ITER_NUM(12);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_ANGH);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, scale, angh);

    return;
}

/**
 * @brief  Square root
 *         formula: output_val*(2^output_scale)=sqrt(input_val*(2^input_scale)),
 *         input_val*(2^input_scale) range is (0.027, 2.34)
 *
 * @param[in] input_scale input scaling factor in Q8.0 format, range is [0, 2]
 * @param[in] input_val input value in Q1.31 format
 * @param[out] output_scale ponits to output scaling factor in Q8.0 format, range is [0, 2]
 * @param[out] output_val points to output value in Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CP_Sqrt(int8_t input_scale, int32_t input_val, int8_t *output_scale, int32_t *output_val)
{
    uint64_t result;

    CP_MATH_SET_X(input_val);
    CP_MATH_SET_SCALE(input_scale);
    CP_MATH_SET_ITER_NUM(12);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_SQRT);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, output_val);
    return;
}

/**
 * @brief  Exp of the given value
 *         formula: output_val*(2^output_scale)=exp(input_val*(2^input_scale)),
 *                  input_val*(2^input_scale) range is (-1.118, 1.118)
 *
 * @param[in] input_scale input scaling factor in Q8.0 format, range is [0, 1]
 * @param[in] input_val input value in Q1.31 format
 * @param[out] output_scale ponits to output scaling factor in Q8.0 format, range is [0, 2]
 * @param[out] output_val points to output value in Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CP_Exp(int8_t input_scale, int32_t input_val, int8_t *output_scale, int32_t *output_val)
{
    uint64_t result;

    CP_MATH_SET_ANGLE(input_val);
    CP_MATH_SET_SCALE(input_scale);
    CP_MATH_SET_ITER_NUM(12);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_EXP);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, output_val);

    return;
}

/**
 * @brief  Ln of the given value
 *         formula: output_val*(2^output_scale)=ln(input_val*(2^input_scale)),
                    input_val*(2^input_scale) range is (0.107, 9.35)
 *
 * @param[in] input_scale input scaling factor in Q8.0 format, range is [0, 4]
 * @param[in] input_val input value in Q1.31 format
 * @param[out] output_scale ponits to output scaling factor in Q8.0 format, range is [1, 2]
 * @param[out] output_val points to output value in Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CP_Ln(int8_t input_scale, int32_t input_val, int8_t *output_scale, int32_t *output_val)
{
    uint64_t result;

    CP_MATH_SET_X(input_val);
    CP_MATH_SET_SCALE(input_scale);
    CP_MATH_SET_ITER_NUM(12);
    CP_MATH_SET_MODE(CP_MATH_NORM_MODE_LN);

    result = CP_MATH_GET_RESULT_UINT64();
    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, output_val);

    return;
}


#ifdef HAL_MATH_CDE_SUPPORT
/**
 * @brief  Calculate cosine and sine of an angle
 *
 * @param[in] angle angle in radians, devided by pi, Q1.31 format, range: [-1, 1], it's mapped to [-pi, pi]
 * @param[out] cos_val cosine of the angle, Q1.31 format
 * @param[out] sin_val sine of the angle, Q1.31 format
 *
 * @retval cosine value
 */
__STATIC_INLINE void HAL_CDE_CosSin(int32_t angle, int32_t *cos_val, int32_t *sin_val)
{
    volatile uint64_t cos_sin;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    cos_sin = __arm_cx2d(CP_MATH_CDE, angle,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_COS, 11));
    HAL_MATH_EnableInterrupt(mask);

    if (cos_val)
    {
        *cos_val = (int32_t)(cos_sin & UINT32_MAX);
    }
    if (sin_val)
    {
        *sin_val = (int32_t)((cos_sin >> 32) & UINT32_MAX);
    }

    return;
}

/**
 * @brief  Calculate sine of an angle
 *
 * @param[in] angle angle in radians, devided by pi, Q1.31 format, range: [-1, 1], it's mapped to [-pi, pi]
 *
 * @retval sine value in Q1.31 format
 */
__STATIC_INLINE int32_t HAL_CDE_Sin(int32_t angle)
{
    volatile int32_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = (int32_t)__arm_cx2(CP_MATH_CDE, angle,
                                CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_SIN, 11));
    HAL_MATH_EnableInterrupt(mask);

    return result;
}

/**
 * @brief  Calculate arctangent
 *         formula: atan*pi=atan(data*(2^scale))
 *
 * @param[in] scale scaling factor, range is [0,12]
 * @param[in] data data in Q1.31 format
 *
 * @retval arctangent in radians, divided by pi, Q1.31 format, range is [-1, 1] which is mapped to [-pi, pi]
 */
__STATIC_INLINE int32_t HAL_CDE_Atan(int8_t scale, int32_t data)
{
    volatile int32_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = (int32_t)(__arm_cx2da(CP_MATH_CDE, scale, data,
                                   CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_ATAN, 11)) & UINT32_MAX);
    HAL_MATH_EnableInterrupt(mask);

    return result;
}

/**
 * @brief  Calculate magnitude(i.e. modulus) of the vector v=(x,y),
 *         formula: magnitude=mod*(2^scale)=sqrt(x^2 + y^2)
 *
 * @param[in] x x coordinate, Q1.31 format
 * @param[in] y y coordinate, Q1.31 format
 * @param[out] scale points to output scaling value, range is [-30, 1]
 * @param[out] mod points to output modulus, Q1.31 format
 *
 * @retval void
 */
__STATIC_INLINE void HAL_CDE_Mod(int32_t x, int32_t y, int8_t *scale, int32_t *mod)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, y, x,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_MOD, 11));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, scale, mod);

    return;
}

/**
 * @brief  Calculate phase angle of the vector v=(x,y),
 *         formula: angle*pi=arctangent(y/x)
 *
 * @param[in] x x coordinate, Q1.31 format
 * @param[in] y y coordinate, Q1.31 format
 *
 * @retval angle in radians devided by pi, range is [-1,1] which is mapped to [-pi, pi]
 */
__STATIC_INLINE int32_t HAL_CDE_Ang(int32_t x, int32_t y)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, y, x,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_ANG, 11));
    HAL_MATH_EnableInterrupt(mask);

    return (result & UINT32_MAX);
}

/**
 * @brief  Calculate the rotated vector
 *
 * @param[in] x x coordinate, Q1.31 format, range is (-0.707, 0.707)
 * @param[in] y y coordinate, Q1.31 format, range is (-0.707, 0.707)
 * @param[in] angle angle in radians, devided by pi, Q1.31 format, range: [-1, 1], which is mapped to [-pi, pi]
 * @param[out] x2 points to x coordinate of the rotated vector
 * @param[out] y2 points to y coordinate of the rotated vector

 * @return void
 */
__STATIC_INLINE void HAL_CDE_Rot(int32_t x, int32_t y, int32_t angle, int32_t *x2, int32_t *y2)
{
    volatile uint64_t result;
    uint32_t mask;

    result = (uint64_t)((uint32_t)angle);
    result = (result << 32) | (uint32_t)y;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, result, x,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_ROT, 11));
    HAL_MATH_EnableInterrupt(mask);

    if (x2)
    {
        *x2 = (int32_t)(result & UINT32_MAX);
    }
    if (y2)
    {
        *y2 = (int32_t)(result >> 32);
    }

    return;
}

/**
 * @brief  Multiple data0 by data1
 *         formula: val*(2^scale)=data0*data1
 *
 * @param[in] data0 data0 in Q1.31 format
 * @param[in] data1 data1 in Q1.31 format
 * @param[out] scale points to scaling factor, Q8.0 format, range is [-60,0]
 * @param[out] val points to output value, Q1.31 format

 * @return void
 */
__STATIC_INLINE void HAL_CDE_Mul(int32_t data0, int32_t data1, int8_t *scale, int32_t *val)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, data1, data0,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_MUL, 11));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, scale, val);
    return;
}


/**
 * @brief  Divide numeritor by denominator
 *         formula: num/den=val*(2^scale)
 *
 * @param[in] num numerator in Q1.31 format
 * @param[in] den denominator in Q1.31 format
 * @param[out] scale points to scaling factor, Q8.0 format, range is [-30,30]
 * @param[out] val points to output vallue, Q1.31 format

 * @return void
 */
__STATIC_INLINE void HAL_CDE_Div(int32_t num, int32_t den, int8_t *scale, int32_t *val)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, num, den,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_DIV, 11));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, scale, val);

    return;
}

/**
 * @brief  Calculate cosineh of an angle
 *         formula: cosh*(2^output_scale)=cosh(angle*(2^scale)),
 *                  angle*(2^scale) range is (-1.118, 1.118)
 *
 * @param[in] scale scaling factor, Q8.0 format, range is [0, 1]
 * @param[in] angle angle in Q1.31 format, range: [-1, 1)
 * @param[out] output_scale points to output scaling factor, Q8.0 format, range is [0, 1]
 * @param[out] cosh points to output cosh value
 *
 * @return void
 */
__STATIC_INLINE void HAL_CDE_Cosh(int8_t scale, int32_t angle, int8_t *output_scale, int32_t *cosh)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, scale, angle,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_COSH, 12));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, cosh);

    return;
}

/**
 * @brief  Calculate sineh of an angle
 *         formula: sinh*(2^output_scale)=sinh(angle*(2^scale)),
 *                  angle*(2^scale) range is (-1.118, 1.118)
 *
 * @param[in] scale scaling factor, Q8.0 format, range is [0,1]
 * @param[in] angle angle in Q1.31 format, range: [-1, 1)
 * @param[out] output_scale points to output scaling factor, Q8.0 format, range is [0, 1]
 * @param[out] sinh points to output sinh value
 *
 * @return void
 */
__STATIC_INLINE void HAL_CDE_Sinh(int8_t scale, int32_t angle, int8_t *output_scale, int32_t *sinh)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, scale, angle,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_SINH, 12));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, sinh);

    return;
}

/**
 * @brief  Calculate atanh of data, data range is (-0.806, 0.806)
 *         formula: atanh*(2^scale)=atanh(data)
 *
 * @param[in] data data in Q1.31 format
 * @param[out] scale points to output scaling factor, Q8.0 format, range is [0, 1]
 * @param[out] atanh points to output atanh value
 *
 * @return void
 */
__STATIC_INLINE void HAL_CDE_Atanh(int32_t data, int8_t *scale, int32_t *atanh)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, 0, data,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_ATANH, 12));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, scale, atanh);

    return;
}

/**
 * @brief  Calculate sqrt(x^2-y^2)
 *         modh*(2^scale)=sqrt(x^2-y^2),
 *         x range is (-1, 1), y range is (-0.806, 0.806), and abs(x)>=abs(y)
 *
 * @param[in] x x coordinate, Q1.31 format
 * @param[in] y y coordinate, Q1.31 format
 * @param[out] scale points to output scaling factor in Q8.0 format
 * @param[out] modh points to output modulush, Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CDE_Modh(int32_t x, int32_t y, int8_t *scale, int32_t *modh)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, y, x,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_MODH, 12));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, scale, modh);

    return;
}

/**
 * @brief  Calculate arctanh(y/x)
 *         angh*(2^scale)=arctanh(y/x),
 *         x range is (-1, 1), y range is (-0.806, 0.806), and abs(x)>=abs(y)
 *
 * @param[in] x x coordinate, Q1.31 format
 * @param[in] y y coordinate, Q1.31 format
 * @param[out] scale points to output scaling factor in Q8.0 format
 * @param[out] angh points to output angh, Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CDE_Angh(int32_t x, int32_t y, int8_t *scale, int32_t *angh)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, y, x,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_ANGH, 12));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, scale, angh);

    return;
}

/**
 * @brief  Square root
 *         formula: output_val*(2^output_scale)=sqrt(input_val*(2^input_scale)),
 *         input_val*(2^input_scale) range is (0.027, 2.34)
 *
 * @param[in] input_scale input scaling factor in Q8.0 format, range is [0, 2]
 * @param[in] input_val input value in Q1.31 format
 * @param[out] output_scale ponits to output scaling factor in Q8.0 format, range is [0, 2]
 * @param[out] output_val points to output value in Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CDE_Sqrt(int8_t input_scale, int32_t input_val, int8_t *output_scale, int32_t *output_val)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, input_scale, input_val,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_SQRT, 12));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, output_val);

    return;
}

/**
 * @brief  Exp of the given value
 *         formula: output_val*(2^output_scale)=exp(input_val*(2^input_scale)),
 *                  input_val*(2^input_scale) range is (-1.118, 1.118)
 *
 * @param[in] input_scale input scaling factor in Q8.0 format, range is [0, 1]
 * @param[in] input_val input value in Q1.31 format
 * @param[out] output_scale ponits to output scaling factor in Q8.0 format, range is [0, 2]
 * @param[out] output_val points to output value in Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CDE_Exp(int8_t input_scale, int32_t input_val, int8_t *output_scale, int32_t *output_val)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, input_scale, input_val,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_EXP, 12));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, output_val);

    return;
}

/**
 * @brief  Ln of the given value
 *         formula: output_val*(2^output_scale)=ln(input_val*(2^input_scale)),
                    input_val*(2^input_scale) range is (0.107, 9.35)
 *
 * @param[in] input_scale input scaling factor in Q8.0 format, range is [0, 4]
 * @param[in] input_val input value in Q1.31 format
 * @param[out] output_scale ponits to output scaling factor in Q8.0 format, range is [1, 2]
 * @param[out] output_val points to output value in Q1.31 format
 *
 * @return void
 */
__STATIC_INLINE void HAL_CDE_Ln(int8_t input_scale, int32_t input_val, int8_t *output_scale, int32_t *output_val)
{
    volatile uint64_t result;
    uint32_t mask;

    mask = HAL_MATH_DisableInterrupt();
    result = __arm_cx2da(CP_MATH_CDE, input_scale, input_val,
                         CP_MATH_CDE_MAKE_CTRL_CODE(CP_MATH_CDE_MODE_LN, 12));
    HAL_MATH_EnableInterrupt(mask);

    CP_MATH_GET_SCALE_AND_VAL(result, output_scale, output_val);

    return;
}

#endif /* HAL_MATH_CDE_SUPPORT */

#ifdef __cplusplus
}
#endif

///@}   MATH

#endif /* __BF0_HAL_MATH_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
