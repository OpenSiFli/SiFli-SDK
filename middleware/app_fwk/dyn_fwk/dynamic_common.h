/*
 ******************************************************************************
 * @file   dynamic_common.h
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
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

#ifndef _DYNAMIC_COMMON_H_
#define _DYNAMIC_COMMON_H_

#include <stdint.h>

#if defined (RT_USING_DFS) && defined (RT_USING_MODULE)
    #include "apm.h"
#endif

#include "gui_app_fwk.h"
extern uint8_t dynamic_log;

#define DYN_LOG(fmt, ...) \
    if (dynamic_log) \
        rt_kprintf(fmt, ##__VA_ARGS__)

#define DYN_COPY_STR(src, dst)                              \
        if(src)                                             \
            app_free((void *)src);                          \
        char * p = app_malloc(strlen((char *)dst) + 1);     \
        RT_ASSERT(p);                                       \
        strcpy(p, (const char *)dst);                       \
        src = (uint32_t)p;


#if !defined(BUILD_DLMODULE)
    #define DLMOLDULE_GET_VERSION(id) 0
#else
    #define DLMOLDULE_GET_VERSION(id) CONCAT_2(id, _version)
#endif

char *dynamic_copy_str(char *dest, const char *src, uint32_t len);


#endif
