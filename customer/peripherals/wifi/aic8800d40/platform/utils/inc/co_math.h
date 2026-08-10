/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file co_math.h
 *
 * @brief Common optimized math functions
 *
 * Copyright (C) RivieraWaves 2011-2019
 *
 ****************************************************************************************
 */

#ifndef _CO_MATH_H_
#define _CO_MATH_H_

/**
 *****************************************************************************************
 * @defgroup CO_MATH CO_MATH
 * @ingroup COMMON
 * @brief  Optimized math functions and other computations.
 * @{
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "aic_plat_types.h"        // standard integer definitions
#include <stdlib.h>        // standard library
#include "compiler.h"      // for __STATIC_INLINE

/*
 * MACROS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Return value with one bit set.
 *
 * @param[in] pos Position of the bit to set.
 *
 * @return Value with one bit set.  There is no return type since this is a macro and this
 * will be resolved by the compiler upon assignment to an l-value.
 ****************************************************************************************
 */
#define CO_BIT(pos) (1UL<<(pos))

/**
 ****************************************************************************************
 * @brief Align val on the multiple of 4 equal or nearest higher.
 * @param[in] val Value to align.
 * @return Value aligned.
 ****************************************************************************************
 */
#define CO_ALIGN4_HI(val) (((val)+3)&~3)

/**
 ****************************************************************************************
 * @brief Align val on the multiple of a given number equal or nearest higher.
 *
 * x value should be a power of 2.
 *
 * @param[in] val  Value to align.
 * @param[in] x    Multiple value.
 * @return Value   aligned.
 ****************************************************************************************
 */
#define CO_ALIGNx_HI(val, x) (((val)+((x)-1))&~((x)-1))

/**
 ****************************************************************************************
 * @brief Align val on the multiple of 4 equal or nearest lower.
 * @param[in] val Value to align.
 * @return Value aligned.
 ****************************************************************************************
 */
#define CO_ALIGN4_LO(val) ((val)&~3)

/**
 ****************************************************************************************
 * @brief Align val on the multiple of 2 equal or nearest higher.
 * @param[in] val Value to align.
 * @return Value aligned.
 ****************************************************************************************
 */
#define CO_ALIGN2_HI(val) (((val)+1)&~1)


/**
 ****************************************************************************************
 * @brief Align val on the multiple of 2 equal or nearest lower.
 * @param[in] val Value to align.
 * @return Value aligned.
 ****************************************************************************************
 */
#define CO_ALIGN2_LO(val) ((val)&~1)


/*
 * FUNCTION DEFINTIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Count leading zeros.
 * @param[in] val Value to count the number of leading zeros on.
 * @return Number of leading zeros when value is written as 32 bits.
 ****************************************************************************************
 */
uint32_t co_clz(uint32_t val);

/**
 ****************************************************************************************
 * @brief Function to initialize the random seed.
 * @param[in] seed The seed number to use to generate the random sequence.
 ****************************************************************************************
 */
__STATIC_INLINE void co_random_init(uint32_t seed)
{
    srand(seed);
}

/**
 ****************************************************************************************
 * @brief Function to get an 8 bit random number.
 * @return Random byte value.
 ****************************************************************************************
 */
__STATIC_INLINE uint8_t co_rand_byte(void)
{
    return (uint8_t)(rand() & 0xFF);
}

/**
 ****************************************************************************************
 * @brief Function to get an 16 bit random number.
 * @return Random half word value.
 ****************************************************************************************
 */
__STATIC_INLINE uint16_t co_rand_hword(void)
{
    return (uint16_t)(rand() & 0xFFFF);
}

/**
 ****************************************************************************************
 * @brief Function to get an 32 bit random number.
 * @return Random word value.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_rand_word(void)
{
    return (uint32_t)rand();
}

/**
 ****************************************************************************************
 * @brief Function to return the smallest of 2 unsigned 32 bits words.
 * @param[in] a First value
 * @param[in] b Second value
 * @return The smallest value between a and b
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_min(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

/**
 ****************************************************************************************
 * @brief Function to return the greatest of 2 unsigned 32 bits words.
 * @param[in] a First value
 * @param[in] b Second value
 * @return The greatest value between a and b
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_max(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}

/**
 ****************************************************************************************
 * @brief Function to return the absolute value of a signed integer.
 * @param[in] val Value
 * @return The absolute value of val.
 ****************************************************************************************
 */
__STATIC_INLINE int co_abs(int val)
{
    return val < 0 ? val*(-1) : val;
}

/**
 ****************************************************************************************
 * @brief Compute a CRC32 on the buffer passed as parameter. The initial value of the
 * computation is taken from crc parameter, allowing for incremental computation.
 *
 * @param[in] addr   Pointer to the buffer on which the CRC has to be computed
 * @param[in] len    Length of the buffer
 * @param[in] crc    The initial value of the CRC computation
 *
 * @return The CRC computed on the buffer.
 ****************************************************************************************
 */
uint32_t co_crc32(uint32_t addr, uint32_t len, uint32_t crc);

uint8_t co_crc8(uint8_t *pdata, uint32_t len, uint8_t crc);

/// @} CO_MATH

#endif // _CO_MATH_H_
