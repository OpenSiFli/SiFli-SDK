/**
  ******************************************************************************
  * @file   mlos.h
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

#ifndef _MLOS_H
#define _MLOS_H

#ifndef __KERNEL__
    #include <stdio.h>
#endif

#include "mltypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(LINUX) || defined(__KERNEL__)
typedef unsigned int HANDLE;
#endif

/* ------------ */
/* - Defines. - */
/* ------------ */

/* - MLOSCreateFile defines. - */

#define MLOS_GENERIC_READ         ((unsigned int)0x80000000)
#define MLOS_GENERIC_WRITE        ((unsigned int)0x40000000)
#define MLOS_FILE_SHARE_READ      ((unsigned int)0x00000001)
#define MLOS_FILE_SHARE_WRITE     ((unsigned int)0x00000002)
#define MLOS_OPEN_EXISTING        ((unsigned int)0x00000003)

/* ---------- */
/* - Enums. - */
/* ---------- */

/* --------------- */
/* - Structures. - */
/* --------------- */

/* --------------------- */
/* - Function p-types. - */
/* --------------------- */

#ifndef __KERNEL__
#include <string.h>
void *inv_malloc(unsigned int numBytes);
inv_error_t inv_free(void *ptr);
inv_error_t inv_create_mutex(HANDLE *mutex);
inv_error_t inv_lock_mutex(HANDLE mutex);
inv_error_t inv_unlock_mutex(HANDLE mutex);
FILE *inv_fopen(char *filename);
void inv_fclose(FILE *fp);

inv_error_t inv_destroy_mutex(HANDLE handle);

void inv_sleep(int mSecs);
#ifdef EMPL
inv_error_t inv_get_tick_count(inv_time_t *timestamp);
#else
inv_time_t inv_get_tick_count(void);
#endif


/* Kernel implmentations */
#define GFP_KERNEL (0x70)
static inline void *kmalloc(size_t size,
                            unsigned int gfp_flags)
{
    (void)gfp_flags;
    return inv_malloc((unsigned int)size);
}
static inline void *kzalloc(size_t size, unsigned int gfp_flags)
{
    void *tmp = inv_malloc((unsigned int)size);
    (void)gfp_flags;
    if (tmp)
        memset(tmp, 0, size);
    return tmp;
}
static inline void kfree(void *ptr)
{
    inv_free(ptr);
}
static inline void msleep(long msecs)
{
    inv_sleep(msecs);
}
static inline void udelay(unsigned long usecs)
{
    inv_sleep((usecs + 999) / 1000);
}
#else
#include <linux/delay.h>
#endif

#ifdef __cplusplus
}
#endif
#endif              /* _MLOS_H */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
