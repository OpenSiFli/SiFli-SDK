/**
  ******************************************************************************
  * @file   bts2_type.h
  * @author Sifli software development team
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

#ifndef _BTS2_TYPE_H_
#define _BTS2_TYPE_H_

#ifdef __cplusplus
extern "C" {
#endif

#undef  FALSE
#define FALSE   (0)

#undef  TRUE
#define TRUE    (1)

#ifndef NULL
#define NULL    (0)
#endif

#define EPHDL                   0xFFFF /* Error handle */

#define CFG_ARM


#if defined(CFG_MS) || defined(CFG_WINCE) || defined(CFG_ARM)

#define CP_ACP                    0           // default to ANSI code page
#define CP_OEMCP                  1           // default to OEM  code page
#define CP_MACCP                  2           // default to MAC  code page
#define CP_THREAD_ACP             3           // current thread's ANSI code page
#define CP_SYMBOL                 42          // SYMBOL translations

#define CP_UTF7                   65000       // UTF-7 translation
#define CP_UTF8                   65001       // UTF-8 translation

typedef unsigned char           U8;
typedef unsigned short int      U16;
typedef unsigned long int       U24;
typedef unsigned long int       U32;

typedef signed char             S8;
typedef signed short int        S16;
typedef signed long int         S24;
typedef signed long int         S32;
typedef unsigned short          WORD;
typedef void                   *LPVOID;
//typedef wchar_t               WCHAR;

#ifndef BOOL
typedef int BOOL;
#endif
#endif

#if defined(CFG_GNU) || defined(CFG_ANDROID)

#define CP_ACP                    0           // default to ANSI code page
#define CP_OEMCP                  1           // default to OEM  code page
#define CP_MACCP                  2           // default to MAC  code page
#define CP_THREAD_ACP             3           // current thread's ANSI code page
#define CP_SYMBOL                 42          // SYMBOL translations

#define CP_UTF7                   65000       // UTF-7 translation
#define CP_UTF8                   65001       // UTF-8 translation


typedef unsigned char           U8;
typedef unsigned short int      U16;
typedef unsigned long int       U24;
typedef unsigned long int       U32;

typedef signed char             S8;
typedef signed short int        S16;
typedef signed long int         S24;
typedef signed long int         S32;
typedef unsigned short          WORD;
typedef void                   *LPVOID;
typedef unsigned short          WCHAR;

#ifndef BOOL
typedef int BOOL;
#endif
#endif


/* depend on BTS2_GET_TIME */
#define MILLISECOND             ((U32)(1))
#define SECOND                  ((U32)(1000 * MILLISECOND))
#define MINUTE                  ((U32)(60 * SECOND))
typedef unsigned long int       TID;

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
