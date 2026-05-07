/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 ******************************************************************************
 * @file   app_fat.h
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
#ifndef _APP_LANG_H_
#define _APP_LANG_H_

#include <rtthread.h>

typedef struct
{
    const char      *pgm_name;  /**< Language packet name. which is english_name                     */
    const char      *locale;    /**< Name using this language.                                       */
} ex_lang_node_t;

typedef struct
{
    const char      *pgm_name;  /**< Current language name.                                           */
    void            *lang_pack; /**< Current language packet. Including current multi_language entries*/
    uint16_t         list_num;  /**< Number of multi_language                                         */
    ex_lang_node_t  *node_list; /**< Multi_language list                                              */
} ex_lang_t;

/**
 * @brief  Load multi-language list from flash, which dir is "path".
           This is only used in file_system.
 * @param  path The directory of multi-language package bin.
 * @retval lang_list Pointer of multi-language list.
 */
ex_lang_t  *app_lang_load_pack_list(const char *path);

/**
 * @brief  Traverse through all multilingual lists one by one,
           and return the names of each multi-language
 * @param  i Index, increase by 1 each time.
 * @retval name The name of each multi-language.
 */
const char *app_lang_pack_iterator(uint32_t *i);

/**
 * @brief  Install a language to memory, which file_name is "path/locale".
           This funciton only used to set current language.
 * @param  path The directory of multi-language package bin.
 * @param  The names of multi-language, which will be installed.
 * @retval install_result RT_EOK: normal; RT_ERR: abnormal.
 */
rt_err_t    app_lang_install_pack(const char *path, const char *locale);


/**
 * @brief  Delete a language from flash, which file_name is "path/locale" .
 * @param  path The directory of multi-language package bin.
 * @param  lang_pack_name The directory of multi-language package bin.
 * @retval delete_result RT_EOK: normal; RT_ERR: abnormal.
 */
rt_err_t    app_lang_del_one_pack(const char *path, const char *lang_pack_name);

#if 0
/*
 * SDK local extension kept for later use. It is not part of the original
 * solution-side multilingual public header.
 */
void app_fat_set_para(bool nor_lang_using_mem);
#endif

#endif /* _APP_LANG_H_ */
