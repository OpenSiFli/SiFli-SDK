/**
  ******************************************************************************
  * @file   section.h
  * @author Sifli software development team
  * @brief Sifli Power management API
  *
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


#ifndef SECTION_H__
#define SECTION_H__

#include "sf_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup section_vars Section variables
 * @ingroup middleware
 * @{
 *
 * @brief Section variables.
 */

#if defined(__ICCARM__)
// Enable IAR language extensions
#pragma language=extended
#endif

#if defined(__CC_ARM) || defined(__CLANG_ARM)
/**@brief   Macro for obtaining the address of the beginning of a section.
 *
 * param[in]    section_name    Name of the section.
 * @hideinitializer
 */
#define SECTION_START_ADDR(section_name)            &CONCAT_2(section_name, $$Base)
#define LOAD_REGION_START_ADDR(region_name)         &CONCAT_2(CONCAT_2(Load$$LR$$,region_name), $$Base)
#define EXEC_REGION_LOAD_START_ADDR(region_name)    &CONCAT_2(CONCAT_2(Load$$,region_name), $$Base)
#define EXEC_REGION_START_ADDR(region_name)         &CONCAT_2(CONCAT_2(Image$$,region_name), $$Base)

#elif defined(__GNUC__)
#define SECTION_START_ADDR(section_name)       &CONCAT_2(CONCAT_2(__, section_name), _start__)
#define EXEC_REGION_START_ADDR(region_name)    &CONCAT_2(CONCAT_2(__, region_name), _start__)
#define EXEC_REGION_LOAD_START_ADDR(region_name)    SECTION_START_ADDR(CONCAT_2(region_name, _load))

#elif defined(__ICCARM__)
#define SECTION_START_ADDR(section_name)       __section_begin(STRINGIFY(section_name))
#define LOAD_REGION_START_ADDR(region_name)    &CONCAT_2(region_name, $$Base)
#define EXEC_REGION_START_ADDR(region_name)    &CONCAT_2(region_name, $$Base)
#define EXEC_REGION_LOAD_START_ADDR(region_name)     EXEC_REGION_START_ADDR(CONCAT_2(region_name, _init))


#elif defined(_MSC_VER)
#define SECTION_START_ADDR(section_name)       &CONCAT_2(section_name, _start)
#define LOAD_REGION_START_ADDR(region_name)    (0)
#endif


#if defined(__CC_ARM) || defined(__CLANG_ARM)
/**@brief    Macro for obtaining the address of the end of a section.
 *
 * @param[in]   section_name    Name of the section.
 * @hideinitializer
 */
#define SECTION_END_ADDR(section_name)            &CONCAT_2(section_name, $$Limit)
#define LOAD_REGION_END_ADDR(region_name)         &CONCAT_2(CONCAT_2(Load$$LR$$,region_name), $$Limit)
#define EXEC_REGION_LOAD_END_ADDR(region_name)    &CONCAT_2(CONCAT_2(Load$$,region_name), $$Limit)
#define EXEC_REGION_END_ADDR(region_name)         &CONCAT_2(CONCAT_2(Image$$,region_name), $$Limit)

#elif defined(__GNUC__)
#define SECTION_END_ADDR(section_name)         &CONCAT_2(CONCAT_2(__, section_name), _end__)
#define EXEC_REGION_END_ADDR(region_name)      &CONCAT_2(CONCAT_2(__, region_name), _end__)
#define EXEC_REGION_LOAD_END_ADDR(region_name)    SECTION_END_ADDR(CONCAT_2(region_name, _load))


#elif defined(__ICCARM__)
#define SECTION_END_ADDR(section_name)         __section_end(STRINGIFY(section_name))
#define LOAD_REGION_END_ADDR(region_name)      &CONCAT_2(region_name, $$Limit)
#define EXEC_REGION_END_ADDR(region_name)      &CONCAT_2(region_name, $$Limit)
#define EXEC_REGION_LOAD_END_ADDR(region_name) EXEC_REGION_END_ADDR(CONCAT_2(region_name, _init))

#elif defined(_MSC_VER)
#define SECTION_END_ADDR(section_name)         &CONCAT_2(section_name, _end)
#define LOAD_REGION_END_ADDR(region_name)      (0)
#endif


/**@brief   Macro for retrieving the length of a given section, in bytes.
 *
 * @param[in]   section_name    Name of the section.
 * @hideinitializer
 */
#define SECTION_LENGTH(section_name)                        \
    ((size_t)SECTION_END_ADDR(section_name) -               \
     (size_t)SECTION_START_ADDR(section_name))


#define EXEC_REGION_SIZE(region_name) ((size_t)(EXEC_REGION_END_ADDR(region_name)) - (size_t)(EXEC_REGION_START_ADDR(region_name)))


#if defined(__CC_ARM) || defined(__CLANG_ARM)
/**@brief   Macro for creating a section.
 *
 * @param[in]   section_name    Name of the section.
 * @param[in]   data_type       Data type of the variables to be registered in the section.
 *
 * @warning Data type must be word aligned to prevent padding.
 * @hideinitializer
 */
#define SECTION_DEF(section_name, data_type)                \
    extern data_type * CONCAT_2(section_name, $$Base);      \
    extern void      * CONCAT_2(section_name, $$Limit)

#define LOAD_REGION_DEF(region_name)                                           \
    extern uint8_t * CONCAT_2(CONCAT_2(Load$$LR$$, region_name), $$Base);      \
    extern void      * CONCAT_2(CONCAT_2(Load$$LR$$, region_name), $$Limit)

#define EXEC_REGION_LOAD_SYM_DEF(region_name)                                 \
    extern uint8_t * CONCAT_2(CONCAT_2(Load$$, region_name), $$Base);         \
    extern void      * CONCAT_2(CONCAT_2(Load$$, region_name), $$Limit)

#define EXEC_REGION_DEF(region_name)                                           \
    extern uint8_t * CONCAT_2(CONCAT_2(Image$$, region_name), $$Base);         \
    extern void      * CONCAT_2(CONCAT_2(Image$$, region_name), $$Limit)


#elif defined(__GNUC__)
#define SECTION_DEF(section_name, data_type)                \
    extern data_type * CONCAT_2(CONCAT_2(__, section_name), _start__);         \
    extern void      * CONCAT_2(CONCAT_2(__, section_name), _end__)

#define EXEC_REGION_DEF(region_name)                                           \
    extern uint8_t * CONCAT_2(CONCAT_2(__, region_name), _start__);            \
    extern void    * CONCAT_2(CONCAT_2(__, region_name), _end__)


#define LOAD_REGION_DEF(region_name)                                           \
    EXEC_REGION_DEF(CONCAT_2(region_name, _load))

#define EXEC_REGION_LOAD_SYM_DEF(region_name)     LOAD_REGION_DEF(region_name)


#elif defined(__ICCARM__)
#define SECTION_DEF(section_name, data_type)                \
    _Pragma(STRINGIFY(section = STRINGIFY(section_name)))


#define EXEC_REGION_DEF(region_name)                                           \
    extern uint8_t * CONCAT_2(region_name, $$Base);                            \
    extern void    * CONCAT_2(region_name, $$Limit)

#define LOAD_REGION_DEF(region_name)                                           \
    EXEC_REGION_DEF(region_name)

#define EXEC_REGION_LOAD_SYM_DEF(region_name)     LOAD_REGION_DEF(region_name)



#elif defined(_MSC_VER)
#define SECTION_DEF(section_name, data_type)                \
    SECTION(STRINGIFY(CONCAT_2(section_name,$0)))           \
    data_type CONCAT_2(section_name, _start);               \
    SECTION(STRINGIFY(CONCAT_2(section_name,$2)))           \
    data_type CONCAT_2(section_name, _end)
#endif


#if defined(__CC_ARM) || defined(__CLANG_ARM)
/**@brief   Macro for declaring a variable and registering it in a section.
 *
 * @details Declares a variable and registers it in a named section. This macro ensures that the
 *          variable is not stripped away when using optimizations.
 *
 * @note The order in which variables are placed in a section is dependent on the order in
 *       which the linker script encounters the variables during linking.
 *
 * @param[in]   section_name    Name of the section.
 * @param[in]   section_var     Variable to register in the given section.
 * @hideinitializer
 */
#define SECTION_ITEM_REGISTER(section_name, section_var) \
    section_var __attribute__ ((section(STRINGIFY(section_name)))) __attribute__((used))

#elif defined(__GNUC__)
#define SECTION_ITEM_REGISTER(section_name, section_var) \
    section_var __attribute__ ((section("." STRINGIFY(section_name)))) __attribute__((used))

#elif defined(__ICCARM__)
#define SECTION_ITEM_REGISTER(section_name, section_var) \
    __root section_var @ STRINGIFY(section_name)
#elif defined(_MSC_VER)
#define SECTION_ITEM_REGISTER(section_name, section_var) \
    SECTION(STRINGIFY(CONCAT_2(section_name,$1)))        \
    section_var
#endif


/**@brief   Macro for retrieving a variable from a section.
 *
 * @warning     The stored symbol can only be resolved using this macro if the
 *              type of the data is word aligned. The operation of acquiring
 *              the stored symbol relies on the size of the stored type. No
 *              padding can exist in the named section in between individual
 *              stored items or this macro will fail.
 *
 * @param[in]   section_name    Name of the section.
 * @param[in]   data_type       Data type of the variable.
 * @param[in]   i               Index of the variable in section.
 * @hideinitializer
 */
#define SECTION_ITEM_GET(section_name, data_type, i) \
    ((data_type*)SECTION_START_ADDR(section_name) + (i))


/**@brief   Macro for getting the number of variables in a section.
 *
 * @param[in]   section_name    Name of the section.
 * @param[in]   data_type       Data type of the variables in the section.
 * @hideinitializer
 */
#define SECTION_ITEM_COUNT(section_name, data_type) \
    SECTION_LENGTH(section_name) / sizeof(data_type)


#if defined(__CC_ARM)
/**@brief   Macro for defining the begin of ZI DATA section,
 *
 * all variables inside #SECTION_ZIDATA_BEGIN and #SECTION_ZIDATA_END are put in the section
 *
 * @param[in]   section_name    Name of the section.
 * @hideinitializer
 */
#define SECTION_ZIDATA_BEGIN(section_name)    _Pragma(STRINGIFY(arm section zidata=STRINGIFY(section_name)))
#elif defined(__CLANG_ARM)
#define SECTION_ZIDATA_BEGIN(section_name)    _Pragma(STRINGIFY(clang section bss=STRINGIFY(section_name)))
#elif defined(__GNUC__)
#define SECTION_ZIDATA_BEGIN(section_name)    _Pragma(STRINGIFY(clang section bss=STRINGIFY(section_name)))
#else
#define SECTION_ZIDATA_BEGIN(section_name)
#endif

#if defined(__CC_ARM)
/**@brief   Macro for defining the end of ZI DATA section,
 *
 * all variables inside #SECTION_ZIDATA_BEGIN and #SECTION_ZIDATA_END are put in the section
 *
 * @hideinitializer
 */
#define SECTION_ZIDATA_END                    _Pragma(STRINGIFY(arm section zidata))
#elif defined(__CLANG_ARM)
#define SECTION_ZIDATA_END                    _Pragma(STRINGIFY(clang section bss=""))
#elif defined(__GNUC__)
#define SECTION_ZIDATA_END                    _Pragma(STRINGIFY(clang section bss=""))
#else
#define SECTION_ZIDATA_END
#endif


/** @} */

#ifdef __cplusplus
}
#endif

#endif // SECTION_H__
