/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AUDIO_MEM_H
#define __AUDIO_MEM_H


#include <stdint.h>
#include <rtthread.h>
#include "board.h"


#ifdef __cplusplus
extern "C" {
#endif

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


#ifdef __cplusplus
}
#endif
#endif // __AUDIO_MEM_H

