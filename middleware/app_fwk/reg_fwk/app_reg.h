/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 ******************************************************************************
 * @file   app_reg.h
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

#ifndef _APP_REG_H_
#define _APP_REG_H_

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include "section.h"
#include "lv_ext_resource_manager.h"
#include "gui_app_fwk.h"

#ifndef GUI_APP_FWK_HAS_PAGE_MEM
    #define GUI_APP_FWK_HAS_PAGE_MEM 0
#endif

#ifdef GUI_APP_BUILTIN
    #define APP_REG_MULTI_STYLE_BUILTIN 1
#else
    #define APP_REG_MULTI_STYLE_BUILTIN 0
#endif

#if GUI_APP_FWK_HAS_PAGE_MEM
    #define APP_REG_GUI_APP_REGIST_MSG_HANDLER_EXT(app_id, handler, usr_data, mem_size) \
        gui_app_regist_msg_handler_ext(app_id, handler, usr_data, mem_size)
    #define APP_REG_GUI_APP_CREATE_PAGE_FOR_APP_EXT(app_id, page_id, handler, user_data, mem_size) \
        gui_app_create_page_for_app_ext(app_id, page_id, handler, user_data, mem_size)
#else
    #define APP_REG_GUI_APP_REGIST_MSG_HANDLER_EXT(app_id, handler, usr_data, mem_size) \
        gui_app_regist_msg_handler_ext(app_id, handler, usr_data)
    #define APP_REG_GUI_APP_CREATE_PAGE_FOR_APP_EXT(app_id, page_id, handler, user_data, mem_size) \
        gui_app_create_page_for_app_ext(app_id, page_id, handler, user_data)
#endif

int gui_app_create_subpage_ext(const char *app_id, const char *sub_id, void *user_data);

typedef struct
{
    const char *app_id;
    const char *page_id;
    gui_page_msg_cb_t handler;
    uint32_t mem_size;
    const uint32_t  reserved;
} app_subpage_desc_t;

typedef struct
{
    char *page_id;
    lv_img_dsc_t *img_src;
    lv_ext_str_id_t txt;
} mainmenu_ext_icons_t;

#if defined(APP_MENU_EXT_USED)
    extern const mainmenu_ext_icons_t *mainmenu_ext_icons_table;
#endif

#if APP_REG_MULTI_STYLE_BUILTIN
    #define BUILTIN_APP_LIST_OPEN(style, built_app)                                                                             \
            extern void gui_get_builtin_app_table(uint16_t n);                                                                  \
            gui_get_builtin_app_table(style);                                                                                   \
            built_app = gui_builtin_app_list_open();
#else
    #define BUILTIN_APP_LIST_OPEN(style, built_app)                                                                             \
            do                                                                                                                  \
            {                                                                                                                   \
                (void)(style);                                                                                                  \
                built_app = gui_builtin_app_list_open();                                                                        \
            } while (0)
#endif

#define BUILTIN_APP_LIST_NEXT(built_app)                                                                                        \
        built_app = gui_builtin_app_list_get_next(built_app);

#define BUILTIN_APP_LIST_CLOSE(built_app)                                                                                       \
        gui_builtin_app_list_close(built_app);

typedef void(*builtin_app_read_app_cb)(builtin_app_desc_t *app, mainmenu_ext_icons_t *table_elem, void *user_data);
typedef struct
{
    const char *id;
} app_list_sort_t;

#define APP_MSG_HANDLER(on_start, on_resume, on_pause, on_stop)                                                                 \
static void msg_handler(gui_app_msg_type_t msg, void *param)                                                                    \
{                                                                                                                               \
    switch (msg)                                                                                                                \
    {                                                                                                                           \
    case GUI_APP_MSG_ONSTART:                                                                                                   \
        on_start();                                                                                                             \
        break;                                                                                                                  \
    case GUI_APP_MSG_ONRESUME:                                                                                                  \
        on_resume();                                                                                                            \
        break;                                                                                                                  \
    case GUI_APP_MSG_ONPAUSE:                                                                                                   \
        on_pause();                                                                                                             \
        break;                                                                                                                  \
    case GUI_APP_MSG_ONSTOP:                                                                                                    \
        on_stop();                                                                                                              \
        break;                                                                                                                  \
    default:                                                                                                                    \
        break;                                                                                                                  \
    }                                                                                                                           \
}

#define APPLICATION_MAIN(app_id, ptr_size)                                                                                      \
    static int app_main(intent_t i)                                                                                             \
    {                                                                                                                           \
        char *subpage = (char *)intent_get_string(i, app_id);                                                                   \
        if (!subpage)                                                                                                           \
        {                                                                                                                       \
            int err = APP_REG_GUI_APP_REGIST_MSG_HANDLER_EXT(app_id, msg_handler, NULL, ptr_size);                             \
            RT_ASSERT(RT_EOK == err);                                                                                           \
        }                                                                                                                       \
        intent_reinit(i, app_id);                                                                                               \
        return 0;                                                                                                               \
    }


#define APP_SUBPAGE_SECTION_NAME   app_subpage_db

#define APP_PAGE_REGISTER(app, subpage, ptr_size)                                                                               \
            APP_MSG_HANDLER(on_start, on_resume, on_pause, on_stop);                                                            \
            SECTION_ITEM_REGISTER(APP_SUBPAGE_SECTION_NAME, static const app_subpage_desc_t app_subpage) =                      \
            {                                                                                                                   \
                .app_id      = app,                                                                                             \
                .page_id     = subpage,                                                                                         \
                .handler     = msg_handler,                                                                                     \
                .mem_size    = ptr_size                                                                                         \
            }

#if APP_REG_MULTI_STYLE_BUILTIN
    #define APPLICATION_REGISTER(key_str, img, app_name, ptr_size)                                                              \
        APP_PAGE_REGISTER(app_name, "root", ptr_size);                                                                          \
        APPLICATION_MAIN(app_name, ptr_size);                                                                                   \
        BUILTIN_APP_EXPORT(key_str, APP_GET_IMG(img), app_name, app_main, 1);                                                  \
        BUILTIN_APP_EXPORT(key_str, APP_GET_IMG(CONCAT_2(img, 2)), app_name, app_main, 2);

    #define APPLICATION_REGISTER_HIDDEN(key_str, app_name, ptr_size)                                                            \
        APP_PAGE_REGISTER(app_name, "root", ptr_size);                                                                          \
        APPLICATION_MAIN(app_name, ptr_size)                                                                                    \
        BUILTIN_APP_EXPORT(key_str, NULL, app_name, app_main, 1);                                                               \
        BUILTIN_APP_EXPORT(key_str, NULL, app_name, app_main, 2);

    #define APPLICATION_REGISTER_PATH(key_str, img, app_name, ptr_size)                                                         \
        APP_PAGE_REGISTER(app_name, "root", ptr_size);                                                                          \
        APPLICATION_MAIN(app_name, ptr_size);                                                                                   \
        BUILTIN_APP_EXPORT(key_str, img, app_name, app_main, 1);                                                                \
        BUILTIN_APP_EXPORT(key_str, img, app_name, app_main, 2);
#else
    #define APPLICATION_REGISTER(key_str, img, app_name, ptr_size)                                                              \
        APP_PAGE_REGISTER(app_name, "root", ptr_size);                                                                          \
        APPLICATION_MAIN(app_name, ptr_size);                                                                                   \
        BUILTIN_APP_EXPORT(key_str, APP_GET_IMG(img), app_name, app_main)

    #define APPLICATION_REGISTER_HIDDEN(key_str, app_name, ptr_size)                                                            \
        APP_PAGE_REGISTER(app_name, "root", ptr_size);                                                                          \
        APPLICATION_MAIN(app_name, ptr_size)                                                                                    \
        BUILTIN_APP_EXPORT(key_str, NULL, app_name, app_main)

    #define APPLICATION_REGISTER_PATH(key_str, img, app_name, ptr_size)                                                         \
        APP_PAGE_REGISTER(app_name, "root", ptr_size);                                                                          \
        APPLICATION_MAIN(app_name, ptr_size);                                                                                   \
        BUILTIN_APP_EXPORT(key_str, img, app_name, app_main)
#endif

/** for compile only*/
#define DLMODULE_INIT_DEF(init_func)

/** for compile only*/
#define DLMODULE_DEINIT_DEF(cleanup_func)

#if defined(DYN_APP)
#undef APPLICATION_REGISTER
#if defined (BSP_USING_PC_SIMULATOR)
#define APPLICATION_REGISTER(key_str, img, app_name, ptr_size)  \
        APP_MSG_HANDLER(on_start, on_resume, on_pause, on_stop);   \
        APPLICATION_MAIN(app_name, ptr_size); \
        BUILTIN_APP_EXPORT(key_str, APP_GET_THUM(tn), app_name, app_main)
#elif defined(BUILD_DLMODULE) && defined(APP_DLMODULE_APP_USED)
#undef DLMODULE_INIT_DEF
#undef DLMODULE_DEINIT_DEF
/** load init func only once even if module opend multiple times*/
#define DLMODULE_INIT_DEF(init_func)                                    \
        void _module_init(void)                                         \
        {                                                               \
            module_init_func_t func = init_func;                        \
            if (func) func();                                           \
        }


/** load deinit func only when dlmodule delete*/
#define DLMODULE_DEINIT_DEF(cleanup_func)                               \
        void _module_deinit (void)                                      \
        {                                                               \
            module_cleanup_func_t func = cleanup_func;                  \
            if (func) func();                                           \
        }

#define APPLICATION_REGISTER(key_str, img, app_name, ptr_size)  \
        APP_MSG_HANDLER(on_start, on_resume, on_pause, on_stop);   \
        APPLICATION_MAIN(app_name, ptr_size);   \
        static void dyn_app_register(void)  \
        {   \
            dynamic_app_register(app_name, app_get_str_from_id(key_str), app_main, APP_GET_THUM(tn));\
        }   \
        MODULE_INIT_DEF(dyn_app_register)   \
        MODULE_CLEANUP_DEF(NULL)
#else
#error "DYN_APP defined, app should used as dynamic app and enable APP_DLMODULE_APP_USED!!!"
#endif
#endif

#define APP_GET_PAGE_MEM_PTR    gui_app_this_page_memory()
#define APP_GET_PAGE_USERDATA_PTR    gui_app_this_page_userdata()

#if defined(APP_MENU_EXT_USED)
    #define GET_EXT_APPLICATION_TABLE(icon_table) icon_table = mainmenu_ext_icons_table;
#endif

/**
 * Run subpage for application
 * @param app_id application id
 * @param sub_id subpage id
 * @return error number
 */
int gui_app_run_subpage(const char *app_id, const char *sub_id, void *user_data);

void builtin_app_get_table(builtin_app_desc_t **beg, builtin_app_desc_t **end, uint16_t style);

builtin_app_desc_t *builtin_app_get_next(builtin_app_desc_t *cur, const builtin_app_desc_t *end);

int16_t menu_app_num_get(uint16_t style);

void builtin_app_read_all(builtin_app_read_app_cb cb, app_list_sort_t *sort_tab, uint16_t size, uint16_t style, void *user_data);

#if defined(APP_MENU_EXT_USED)
    uint16_t menu_ext_icons_table_size_get(void);
#endif

const char *gui_app_get_reg_main(void);
int gui_app_set_reg_main(const char *main_name);
int gui_app_get_builtin_app(void);
int gui_app_get_ext_app(void);
int gui_app_get_dyn_app(void);

#endif
