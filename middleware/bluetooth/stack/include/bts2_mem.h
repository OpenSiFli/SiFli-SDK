/**
  ******************************************************************************
  * @file   bts2_mem.h
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

#ifndef _BTS2_MEM_H__
#define _BTS2_MEM_H__
#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bmemmove function copies count bytes of characters from src to dest.
 *      If some regions of the source area and the destination overlap, bmemmove
 *      ensures that the original source bytes in the overlapping region are
 *      copied before being overwritten.
 *
 * INPUT:
 *      void *dest: Destination object.
 *      const void *src: Source object.
 *      int count: Number of characters to copy.
 *
 * OUTPUT:
 *      bmemcpy returns the value of dest.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void *bmemmove(void *dest, const void *src, U32 count);

void *bt_mem_alloc(rt_size_t size);
void *bt_mem_calloc(rt_size_t count, rt_size_t nbytes);
void bt_mem_free(void *ptr);


//void *bmalloc(U32 nbytes);
#define bmalloc(size) bt_mem_alloc(size)
#define bcalloc(count, nbytes) bt_mem_calloc(count, nbytes)
#define bfree(ptr) bt_mem_free(ptr)
//void *bcalloc(rt_size_t count, rt_size_t nbytes);
//void bfree(void *ptr);
#ifndef malloc
#define malloc bmalloc
#endif

#ifndef free
#define free bfree
#endif

#ifndef calloc
#define calloc(unit,size) bcalloc(unit, size)
#endif
#ifdef __cplusplus
}
#endif
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
