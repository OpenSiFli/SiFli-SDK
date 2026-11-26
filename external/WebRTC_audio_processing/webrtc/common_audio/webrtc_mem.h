/*
 * SPDX-FileCopyrightText: 2022-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WEBRTC_MEM_H
#define __WEBRTC_MEM_H

#include <rtthread.h>
#include <string.h>

#if BF0_ACPU
    // should 58x use malloc/free for big heap?
    #undef malloc
    #undef free
    #undef calloc
    #undef realloc
    extern void *acpu_app_malloc(uint32_t size);
    extern void acpu_app_free(void *p);
    extern void *acpu_app_calloc(uint32_t count, uint32_t size);
    extern void *acpu_app_realloc(void *address, uint32_t newsize);

    extern void acpu_printf(const char *fmt, ...);
    static inline void *webrtc_acpu_malloc(uint32_t size)
    {
        void *p = acpu_app_malloc(size);
        return p;
    }
    static inline void *webrtc_acpu_calloc(uint32_t c, uint32_t size)
    {
        void *p = webrtc_acpu_malloc(c * size);
        if (p)
        {
            memset(p, 0, c * size);
        }
        return p;
    }
    static inline void *webrtc_acpu_realloc(void *m, uint32_t size)
    {
        void *p = acpu_app_realloc(m, size);
        return p;
    }
    static inline void webrtc_acpu_free(void *p)
    {
        if (p)
        {
            acpu_app_free(p);
        }
    }
    #define malloc(size)    webrtc_acpu_malloc(size)
    #define free(ptr)       webrtc_acpu_free(ptr)
    #define calloc(c,s)     webrtc_acpu_calloc(c, s)
    #define realloc(m, s)   webrtc_acpu_realloc(m, s)

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

