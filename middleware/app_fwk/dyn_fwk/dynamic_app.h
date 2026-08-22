/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 ******************************************************************************
 * @file   dynamic_app.h
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

#ifndef _DYNAMIC_APP_H_
#define _DYNAMIC_APP_H_
#include "dynamic_common.h"
#include "gui_app_fwk.h"

#ifndef app_is_built_in
#define app_is_built_in(path) (false)
#endif

enum
{
    APP_MOD_NULL    = 0X00,         /**< dlmodule no use                */
    APP_MOD_APP     = 0X01,         /**< dlmodule used for appication   */
    APP_MOD_SETTING = 0X02,         /**< dlmodule used for setting      */
    APP_MOD_POPUP   = 0X04,         /**< dlmodule used for popup        */
};
typedef uint16_t app_mod_use_t;

typedef struct
{
    builtin_app_desc_t desc;        /**< dynamic app description        */
    app_mod_use_t mod_use;          /**< dlmodule use status            */
    const char  *ext_icon;          /**< dynamic app icon style         */
    void *mod;                      /**< dlmodule instance              */
    void *user_data;                /**< user data                      */
    void *bg_data;                  /**< bg data for background         */
    uint8_t      mod_invalid;       /**< dlmodule invalid               */
    rt_list_t list;                 /**< list link                      */
} dyn_app_node_t;

typedef bool (*dyn_list_iterator_t)(dyn_app_node_t *node);

/**
 * @brief  Delete application in db
 * @param  app_name application name
 * @retval 0: success, otherwise: fail
 */
int32_t dynamic_app_del_app(const char *app_name);

/**
 * @brief  Check path is built_in
 * @param  path to check
 * @retval true: is built_in, false: non built_in
 */
//bool dynamic_app_is_built_in(const char *path);
#define dynamic_app_is_built_in(path)       app_is_built_in(path)

/**
 * @brief  Load application from db, used after add dsc
 * @retval RT_EOK success, otherwise fail
 */
int dynamic_app_load(void);

/**
 * @brief  Auto add appicaton in db in both built_in dir and dynamic dir
 * @retval 0: success, otherwise: fail
 */
int dynamic_app_add_dsc(void);

/**
 * @brief  clear use flag and try to close watchface.
 * @param  app_node node of appication
 * @retval none
 */
void dynamic_app_close(dyn_app_node_t *app_node, app_mod_use_t mod_use);

/**
 * @brief  deinit 'dyn_app_list' and close all modules
 * @param  app_node node of appication
 * @retval none
 */
void dynamic_app_list_deinit(void);

/**
 * @brief  get node from 'dyn_app_list' by id
 * @param  id dynamic app id
 * @retval app node, NULL: not find
 */
dyn_app_node_t *dynamic_app_list_get_node(const char *id);

/**
 * @brief  get 'dyn_app_list' count
 * @retval node count
 */
uint32_t dynamic_app_get_cnt(void);

/**
 * @brief  delete app from 'dyn_app_list' and close module
 * @retval RT_EOK: delete suceess from list, RT_ERROR: delete failed
 */
rt_err_t dynamic_app_list_del_app(const char *id);

/**
 * @brief  Register a new application to 'dyn_app_list', for dynamic application only.
 * @param  id application's name.
 * @param  str_id application's title string for multi-language.
 * @param  entry entry function
 * @param  thumbnail application's thumbnail, must be 'APP_GET_IMG(tn)'
 * @retval none
 */
void dynamic_app_register(const char *id, const char *title, gui_app_entry_func_ptr_t entry, const void *thumbnail);

/**
 * @brief  open application, load code in RAM.
 * @param  app_node application node of 'dyn_app_list'
 * @retval none
 */
void dynamic_app_run(dyn_app_node_t *app_node, app_mod_use_t mod_use);

/**
 * @brief  get pointer of 'dyn_app_list'
 * @retval pointer of 'dyn_app_list'
 */
rt_list_t *dynamic_app_get_list(void);

/**
 * @brief  get pointer of 'app_db'
 * @retval pointer of 'app_db'
 */
void *dynamic_app_get_db(void);

/**
 * @brief  Traverse the dynamic application list and output its nodes one by one.
 * @param  node The previous node. Input NULL for the first time.
 * @retval next node. If it is empty, it means the traversal is ended.
 */
dyn_app_node_t *dynamic_app_list_get_app_next(dyn_app_node_t *node);

/**
 * @brief  Add application by dsc file
 * @param  dsc_filename dsc file full path
 * @param  app_name app name that has been added
 *         If app is deleted or db is closed, app_name cannot be used anymore,
 *         caller doesn't need to free the memory of app_name
 * @retval 0: success, otherwise: fail
 */
int32_t dynamic_app_add_by_dsc_file(const char *dsc_filename, const char **app_name);

/**
 * @brief  Update dynamic app title when multi_lang changed
 * @retval None
 */
void dynamic_app_lang_update(void);

/**
 * @brief  Get dynamic app specified style icon
 * @param[in]  app_name App name
 * @param[in]  style icon style, 1 for default icon(installer/xxx_tn.bin), others for additional icon
 * @retval icon path, NULL for get failed
 */
const char *dynamic_app_get_icon(const char *app_name, uint8_t style);

/**
 * @brief  get current mod
 * @retval NULL: no dynamic app opend
 */
void *dynamic_app_get_current_mod(void);


/**
 * @brief Delete all external dynamic apps except the built-in
 * @retval None
 */
void dynamic_app_del_ext_all(void);
#endif
