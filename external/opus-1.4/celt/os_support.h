/* Copyright (C) 2007 Jean-Marc Valin

   File: os_support.h
   This is the (tiny) OS abstraction layer. Aside from math.h, this is the
   only place where system headers are allowed.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

   1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef OS_SUPPORT_H
#define OS_SUPPORT_H

#ifdef CUSTOM_SUPPORT
#  include "custom_support.h"
#endif

#include "opus_types.h"
#include "opus_defines.h"

#include <string.h>
#include <stdlib.h>

#include "rtconfig.h"
#ifdef SOLUTION_WATCH
#include "audio_mem.h"
#endif

#ifndef BF0_ACPU
#include "rtthread.h"
#endif

/** Opus wrapper for malloc(). To do your own dynamic allocation, all you need to do is replace this function and opus_free */
#ifndef OVERRIDE_OPUS_ALLOC
#define opus_alloc(a)  do_opus_alloc(a,__FILE__, __LINE__)
static OPUS_INLINE void *do_opus_alloc (size_t size, const char *file, int line)
{
    void *p;
#if defined(BF0_ACPU)
    p  = malloc(size);
    if (!p)
    {
        extern void* acpu_call_hcpu_malloc(uint32_t size);
        p = acpu_call_hcpu_malloc(size);
    }
    return p;
#endif

#ifdef SOLUTION_WATCH
    p = audio_mem_malloc(size);
    rt_kprintf("opus audio alloc=%d 0x%p\r\n", size, p);
#else
    p = malloc(size);
#ifndef BF0_ACPU
    rt_kprintf("opus alloc=%d 0x%p\r\n", size, p);
#endif
#endif
   return p;
}
#endif

/** Same as celt_alloc(), except that the area is only needed inside a CELT call (might cause problem with wideband though) */
#ifndef OVERRIDE_OPUS_ALLOC_SCRATCH
static OPUS_INLINE void *opus_alloc_scratch (size_t size)
{
   /* Scratch space doesn't need to be cleared */
   return opus_alloc(size);
}
#endif

/** Opus wrapper for free(). To do your own dynamic allocation, all you need to do is replace this function and opus_alloc */
#ifndef OVERRIDE_OPUS_FREE
static OPUS_INLINE void opus_free (void *ptr)
{
#if defined(BF0_ACPU)
    extern void* acpu_call_hcpu_free(void *p);
    acpu_call_hcpu_free(ptr);
    return;
#endif
#ifdef SOLUTION_WATCH
    return audio_mem_free(ptr);
#else
   free(ptr);
#endif
}
#endif

/** Copy n elements from src to dst. The 0* term provides compile-time type checking  */
#ifndef OVERRIDE_OPUS_COPY
#define OPUS_COPY(dst, src, n) (memcpy((dst), (src), (n)*sizeof(*(dst)) + 0*((dst)-(src)) ))
#endif

/** Copy n elements from src to dst, allowing overlapping regions. The 0* term
    provides compile-time type checking */
#ifndef OVERRIDE_OPUS_MOVE
#define OPUS_MOVE(dst, src, n) (memmove((dst), (src), (n)*sizeof(*(dst)) + 0*((dst)-(src)) ))
#endif

/** Set n elements of dst to zero */
#ifndef OVERRIDE_OPUS_CLEAR
#define OPUS_CLEAR(dst, n) (memset((dst), 0, (n)*sizeof(*(dst))))
#endif

/*#ifdef __GNUC__
#pragma GCC poison printf sprintf
#pragma GCC poison malloc free realloc calloc
#endif*/

#endif /* OS_SUPPORT_H */

