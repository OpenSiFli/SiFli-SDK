/*
 * SPDX-FileCopyrightText: 2022-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WEBRTC_MEM_H
#define __WEBRTC_MEM_H

#include <rtthread.h>

#ifdef AUDIO
#include "audio_mem.h"
#endif

#if defined (SYS_HEAP_IN_PSRAM) && defined (AUDIO_MEM_ALLOC)
    #undef malloc
    #undef free
    #undef realloc
    #define malloc(size)    sram_malloc(size)
    #define free(ptr)       sram_free(ptr)
    #define calloc(c,s)     sram_calloc(c,s)
    #define realloc(m, n)   sram_realloc(m, n)
#endif

#endif // __AUDIO_MEM_H

