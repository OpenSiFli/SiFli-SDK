/**
  ******************************************************************************
  * @file   sf_type.h
  * @author Sifli software development team
  * @brief Sifli middle common interface
  * @{
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

#ifndef SF_TYPE_H
#define SF_TYPE_H

#include "rtconfig.h"
#ifdef BSP_USING_RTTHREAD
    #include <rtthread.h>
#endif /* BSP_USING_RTTHREAD */
#include <stdbool.h>
#include <stdint.h>

/**
 ****************************************************************************************
* @addtogroup common Common
* @ingroup middleware
* @brief Sifli middle common interface
* @{
****************************************************************************************
*/

#define SF_EOK                          0               /**< There is no error */
#define SF_ERR                          1               /**< A generic error happens */
#define SF_ETIMEOUT                     2               /**< Timed out */
#define SF_EFULL                        3               /**< The resource is full */
#define SF_EEMPTY                       4               /**< The resource is empty */
#define SF_ENOMEM                       5               /**< No memory */
#define SF_ENOSYS                       6               /**< No system */
#define SF_EBUSY                        7               /**< Busy */
#define SF_EIO                          8               /**< IO error */
#define SF_EINTR                        9               /**< Interrupted system call */
#define SF_EINVAL                       10              /**< Invalid argument */

typedef int sf_err_t;

#ifdef BSP_USING_RTTHREAD
    #define SF_ASSERT   RT_ASSERT
#else
    #define SF_ASSERT(expr)  if ((expr)==0) while (1)
#endif

#define CONCAT_2_(p1, p2)     p1##p2

/**@brief Concatenates two parameters.
 *
 * It realizes two level expansion to make it sure that all the parameters
 * are actually expanded before gluing them together.
 *
 * @param p1 First parameter to concatenating
 * @param p2 Second parameter to concatenating
 *
 * @return Two parameters glued together.
 *         They have to create correct C mnemonic in other case
 *         preprocessor error would be generated.
 *
 */
#define CONCAT_2(p1, p2)      CONCAT_2_(p1, p2)

#define STRINGIFY_(val)       #val

/** Converts a macro argument into a character constant.
 */
#define STRINGIFY(val)       STRINGIFY_(val)


/**
 * @brief Extracting data from the brackets
 *
 * This macro get rid of brackets around the argument.
 * It can be used to pass multiple arguments in logical one argument to a macro.
 * Call it with arguments inside brackets:
 * @code
 * #define ARGUMENTS (a, b, c)
 * BRACKET_EXTRACT(ARGUMENTS)
 * @endcode
 * It would produce:
 * @code
 * a, b, c
 * @endcode
 *
 * @param a Argument with anything inside brackets
 * @return Anything that appears inside the brackets of the argument
 *
 * @note
 * The argument of the macro have to be inside brackets.
 * In other case the compilation would fail.
 */
#define BRACKET_EXTRACT(a)  BRACKET_EXTRACT_(a)
#define BRACKET_EXTRACT_(a) BRACKET_EXTRACT__ a
#define BRACKET_EXTRACT__(...) __VA_ARGS__


/**
 * @brief Get the first argument
 *
 * @param ... Arguments to select
 *
 * @return First argument or empty if no arguments are provided
 */
#define GET_VA_ARG_1(...) GET_VA_ARG_1_(__VA_ARGS__, ) // Make sure that also for 1 argument it works
#define GET_VA_ARG_1_(a1, ...) a1

/**
 * @brief Get all the arguments but the first one
 *
 * @param ... Arguments to select
 *
 * @return All arguments after the first one or empty if less than 2 arguments are provided
 */
#define GET_ARGS_AFTER_1(...) GET_ARGS_AFTER_1_(__VA_ARGS__, ) // Make sure that also for 1 argument it works
#define GET_ARGS_AFTER_1_(a1, ...) __VA_ARGS__


/**
 * @brief Mapping macro with current index
 *
 *
 * @param ... Macro name to be used for argument processing followed by arguments to process.
 *            Macro should have following form: MACRO(argument, index)
 * @return All arguments processed by given macro
 */
#define MACRO_MAP_FOR_N_LIST 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, \
                            19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32




/**
 * @brief Repeating macro.
 *
 * @param count Count of repeats.
 * @param macro Macro must have the following form: MACRO(arguments).
 * @param ...   Arguments passed to the macro.
 *
 * @return All arguments processed by the given macro.
 */
#define MACRO_REPEAT(count, macro, ...)     MACRO_REPEAT_(count, macro, __VA_ARGS__)
#define MACRO_REPEAT_(count, macro, ...)    CONCAT_2(MACRO_REPEAT_, count)(macro, __VA_ARGS__)

#define MACRO_REPEAT_0(macro, ...)
#define MACRO_REPEAT_1(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_0(macro, __VA_ARGS__)
#define MACRO_REPEAT_2(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_1(macro, __VA_ARGS__)
#define MACRO_REPEAT_3(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_2(macro, __VA_ARGS__)
#define MACRO_REPEAT_4(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_3(macro, __VA_ARGS__)
#define MACRO_REPEAT_5(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_4(macro, __VA_ARGS__)
#define MACRO_REPEAT_6(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_5(macro, __VA_ARGS__)
#define MACRO_REPEAT_7(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_6(macro, __VA_ARGS__)
#define MACRO_REPEAT_8(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_7(macro, __VA_ARGS__)
#define MACRO_REPEAT_9(macro, ...)  macro(__VA_ARGS__) MACRO_REPEAT_8(macro, __VA_ARGS__)
#define MACRO_REPEAT_10(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_9(macro, __VA_ARGS__)
#define MACRO_REPEAT_11(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_10(macro, __VA_ARGS__)
#define MACRO_REPEAT_12(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_11(macro, __VA_ARGS__)
#define MACRO_REPEAT_13(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_12(macro, __VA_ARGS__)
#define MACRO_REPEAT_14(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_13(macro, __VA_ARGS__)
#define MACRO_REPEAT_15(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_14(macro, __VA_ARGS__)
#define MACRO_REPEAT_16(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_15(macro, __VA_ARGS__)
#define MACRO_REPEAT_17(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_16(macro, __VA_ARGS__)
#define MACRO_REPEAT_18(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_17(macro, __VA_ARGS__)
#define MACRO_REPEAT_19(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_18(macro, __VA_ARGS__)
#define MACRO_REPEAT_20(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_19(macro, __VA_ARGS__)
#define MACRO_REPEAT_21(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_20(macro, __VA_ARGS__)
#define MACRO_REPEAT_22(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_21(macro, __VA_ARGS__)
#define MACRO_REPEAT_23(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_22(macro, __VA_ARGS__)
#define MACRO_REPEAT_24(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_23(macro, __VA_ARGS__)
#define MACRO_REPEAT_25(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_24(macro, __VA_ARGS__)
#define MACRO_REPEAT_26(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_25(macro, __VA_ARGS__)
#define MACRO_REPEAT_27(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_26(macro, __VA_ARGS__)
#define MACRO_REPEAT_28(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_27(macro, __VA_ARGS__)
#define MACRO_REPEAT_29(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_28(macro, __VA_ARGS__)
#define MACRO_REPEAT_30(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_29(macro, __VA_ARGS__)
#define MACRO_REPEAT_31(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_30(macro, __VA_ARGS__)
#define MACRO_REPEAT_32(macro, ...) macro(__VA_ARGS__) MACRO_REPEAT_31(macro, __VA_ARGS__)


/**
 * @brief Repeating macro with current index.
 *
 * Macro similar to @ref MACRO_REPEAT but the processing function gets the arguments
 * and the current argument index (beginning from 0).

 * @param count Count of repeats.
 * @param macro Macro must have the following form: MACRO(index, arguments).
 * @param ...   Arguments passed to the macro.
 *
 * @return All arguments processed by the given macro.
 */
#define MACRO_REPEAT_FOR(count, macro, ...)     MACRO_REPEAT_FOR_(count, macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_(count, macro, ...)    CONCAT_2(MACRO_REPEAT_FOR_, count)((MACRO_MAP_FOR_N_LIST), macro, __VA_ARGS__)

#define MACRO_REPEAT_FOR_0(n_list, macro, ...)
#define MACRO_REPEAT_FOR_1(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_0((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_2(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_1((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_3(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_2((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_4(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_3((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_5(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_4((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_6(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_5((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_7(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_6((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_8(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_7((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_9(n_list, macro, ...)  macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_8((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_10(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_9((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_11(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_10((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_12(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_11((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_13(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_12((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_14(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_13((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_15(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_14((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_16(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_15((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_17(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_16((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_18(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_17((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_19(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_18((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_20(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_19((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_21(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_20((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_22(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_21((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_23(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_22((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_24(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_23((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_25(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_24((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_26(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_25((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_27(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_26((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_28(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_27((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_29(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_28((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_30(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_29((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_31(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_30((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)
#define MACRO_REPEAT_FOR_32(n_list, macro, ...) macro(GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), __VA_ARGS__) MACRO_REPEAT_FOR_31((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__)




/// @} common
/// @} file


#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
