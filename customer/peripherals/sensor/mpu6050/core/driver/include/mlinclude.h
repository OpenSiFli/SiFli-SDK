/**
  ******************************************************************************
  * @file   mlinclude.h
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

#ifndef INV_INCLUDE_H__
#define INV_INCLUDE_H__

#define INVENSENSE_FUNC_START  typedef int invensensePutFunctionCallsHere

#ifdef COVERAGE
    #include "utestCommon.h"
#endif
#ifdef PROFILE
    #include "profile.h"
#endif

#ifdef WIN32
#ifdef COVERAGE

extern int functionEnterLog(const char *file, const char *func);
extern int functionExitLog(const char *file, const char *func);

#undef INVENSENSE_FUNC_START
#define INVENSENSE_FUNC_START  __pragma(message(__FILE__ "|"__FUNCTION__ )) \
    int dslkQjDsd = functionEnterLog(__FILE__, __FUNCTION__)
#endif // COVERAGE
#endif // WIN32

#ifdef PROFILE
    #undef INVENSENSE_FUNC_START
    #define INVENSENSE_FUNC_START int dslkQjDsd = profileEnter(__FILE__, __FUNCTION__)
    #define return if ( profileExit(__FILE__, __FUNCTION__) ) return
#endif // PROFILE

// #define return if ( functionExitLog(__FILE__, __FUNCTION__) ) return

#endif //INV_INCLUDE_H__/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
