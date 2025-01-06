/**
  ******************************************************************************
  * @file   lvsf_emoji.c
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

#include "rtthread.h"
#include "lv_ext_resource_manager.h"
#ifdef RT_USING_DFS
    #include <dfs.h>
    #include <dfs_file.h>
#endif

#if defined (SOLUTION_WATCH) && defined (EMOJI_SUPPORT)
    #include "images_emoji_header.h"
#endif

#if 0 //ifdef RT_USING_DFS
#define EMOJI_RES_PATH    "/ex/resource/"
#define EMOJI_RES_SUFFIX  ".bin"

//emoji filename format:  emoji_xxx. where xxx is letter's unicode.
char emoji_file_name[60];

static inline void *lv_get_emoji_by_unicode(uint32_t u_letter)
{
    RT_ASSERT(strlen(EMOJI_RES_PATH EMOJI_RES_SUFFIX "emoji_") + 8 < 60);
    sprintf(emoji_file_name, "%semoji_%x%s", EMOJI_RES_PATH, u_letter, EMOJI_RES_SUFFIX);
    //sprintf(emoji_file_name, "%s%s%s", "/ex/resource/", "clock_bg", ".bin");

    struct stat buf;
    if (RT_EOK == dfs_file_stat(emoji_file_name, &buf))
    {
        return &emoji_file_name;
    }

    return NULL;
}
#else
void *lv_get_emoji_by_unicode(uint32_t u_letter)
{
#define GET_EMOJI_INFO(_id) \
    case 0x##_id: \
    { \
    return (void *)&emoji_##_id;\
    }

    switch (u_letter)
    {
#ifndef SOLUTION_WATCH
        /*
        GET_EMOJI_INFO(1f302)
        GET_EMOJI_INFO(1f392)
        GET_EMOJI_INFO(1f393)
        GET_EMOJI_INFO(1f39a)
        GET_EMOJI_INFO(2639)
        GET_EMOJI_INFO(263a)
        GET_EMOJI_INFO(26d1)
        */
#else /*for SOLUTION_WATCH*/
#ifdef EMOJI_SUPPORT
#include "emoji_info.h"
#endif
#endif
    default:
        return NULL;
    }
}
#endif


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
