/**
  ******************************************************************************
  * @file   stdint_invensense.h
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

#ifndef STDINT_INVENSENSE_H
#define STDINT_INVENSENSE_H

#ifndef WIN32

    #if defined __KERNEL__
        #include <linux/types.h>
    #elif defined EMPL
        #include <stdint.h>
        #include <string.h>
    #else
        #include <stdint.h>
    #endif

#else

    #include <windows.h>

    typedef signed char int8_t;
    typedef short int16_t;
    typedef long int32_t;
    typedef long long int64_t;

    typedef unsigned char uint8_t;
    typedef unsigned short uint16_t;
    typedef unsigned long uint32_t;
    typedef unsigned long long uint64_t;

    typedef int int_fast8_t;
    typedef int int_fast16_t;
    typedef long int_fast32_t;

    typedef unsigned int uint_fast8_t;
    typedef unsigned int uint_fast16_t;
    typedef unsigned long uint_fast32_t;

#endif

#endif // STDINT_INVENSENSE_H/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
