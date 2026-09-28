/*
 * SPDX-FileCopyrightText: 2022-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WEBRTC_MEM_H
#define __WEBRTC_MEM_H

#include <rtthread.h>

#if BF0_ACPU
    // should 58x use malloc/free for big heap?
    #undef malloc
    #undef free
    #undef calloc
    #undef realloc
    extern void *acpu_call_hcpu_malloc(uint32_t size);
    extern void acpu_call_hcpu_free(void *p);
    extern void *acpu_call_hcpu_calloc(uint32_t count, uint32_t size);
    extern void *acpu_call_hcpu_realloc(void *address, uint32_t newsize);

    #define malloc(size)    acpu_call_hcpu_malloc(size)
    #define free(ptr)       acpu_call_hcpu_free(ptr)
    #define calloc(c,s)     acpu_call_hcpu_calloc(c, s)
    #define realloc(m, n)   acpu_call_hcpu_realloc(m, n)

#elif AUDIO
    #include "audio_mem.h"
    #undef malloc
    #undef free
    #undef calloc
    #undef realloc

    #define malloc(size)    audio_mem_malloc(size)
    #define free(ptr)       audio_mem_free(ptr)
    #define calloc(c,s)     audio_mem_calloc(c,s)
    #define realloc(m, n)   audio_mem_realloc(m, n)

#endif

#endif // __WEBRTC_MEM_H

