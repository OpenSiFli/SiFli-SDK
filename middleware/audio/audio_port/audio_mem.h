/**
  ******************************************************************************
  * @file   audio_mem.h
  * @author Sifli software development team
  * @brief Header files for SIFLI Audio memory porting APIs.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2022,  Sifli Technology
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


#ifndef __AUDIO_MEM_H
#define __AUDIO_MEM_H

#include <stdint.h>
#include <rtthread.h>
#include "board.h"

#define     AUDIO_MEMORY_LEAK_CHECK         0

#if AUDIO_MEMORY_LEAK_CHECK
    void *audio_mem_malloc_do(uint32_t size, const char *file, int line);
    void  audio_mem_free_do(void *ptr);
    void *audio_mem_calloc_do(uint32_t count, uint32_t size, const char *file, int line);
    void *audio_mem_realloc_do(void *mem_address, unsigned int newsizeconst, const char *file, int line);

    #define audio_mem_malloc(size)  audio_mem_malloc_do(size, __FILE__, __LINE__)
    #define audio_mem_free(ptr)     audio_mem_free_do(ptr)
    #define audio_mem_calloc(c,s)   audio_mem_calloc_do(c,s,__FILE__, __LINE__)
    #define audio_mem_realloc(m, n) audio_mem_realloc_do(m, n, __FILE__, __LINE__)
    #define webrtc_mem_malloc(size)  audio_mem_malloc_do(size, __FILE__, __LINE__)
    #define webrtc_mem_free(ptr)     audio_mem_free_do(ptr)
    #define webrtc_mem_calloc(c,s)   audio_mem_calloc_do(c,s,__FILE__, __LINE__)
    #define webrtc_mem_realloc(m, n) audio_mem_realloc_do(m, n, __FILE__, __LINE__)
#else
    void *audio_mem_malloc(uint32_t size);
    void  audio_mem_free(void *ptr);
    void *audio_mem_calloc(uint32_t count, uint32_t size);
    void *audio_mem_realloc(void *mem_address, unsigned int newsize);

    #if !defined (SYS_HEAP_IN_PSRAM)
        #define sram_malloc(size)   audio_mem_malloc(size)
        #define sram_free(ptr)      audio_mem_free(ptr)
        #define sram_calloc(c,s)    audio_mem_calloc(c,s)
        #define sram_realloc(m, n)  audio_mem_realloc(m, n)
    #else
        extern void *app_sram_alloc(rt_size_t size);
        extern void *app_sram_realloc(void *ptr, rt_size_t newsize);
        extern void *app_sram_calloc(rt_size_t count, rt_size_t size);
        extern void app_sram_free(void *ptr);
        #define sram_malloc(size)   app_sram_alloc(size)
        #define sram_free(ptr)      app_sram_free(ptr)
        #define sram_calloc(c,s)    app_sram_calloc(c,s)
        #define sram_realloc(m, n)  app_sram_realloc(m, n)
    #endif
#endif


#endif // __AUDIO_MEM_H


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
