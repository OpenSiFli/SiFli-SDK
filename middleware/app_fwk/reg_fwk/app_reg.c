/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 ******************************************************************************
 * @file   app_reg.c
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


#include <rtthread.h>
#include "app_reg.h"
#ifdef APP_DLMODULE_APP_USED
    #include "dynamic_app.h"
#endif
#ifdef APP_TOOL_SUPPORT
    #include "app_tool_comm.h"
#endif

static const char *g_reg_main = "Main";

#if APP_REG_MULTI_STYLE_BUILTIN
#if defined (_MSC_VER)
#ifndef STR
    #define STR(str) #str
#endif
#define BUILT_IN_APP_TAB_START(n)                                                                                           \
__declspec(allocate(STR(BuiltinApp##n##Tab$0))) RT_USED static const builtin_app_desc_t __builtin_app##n##_table_start =    \
{                                                                                                                           \
    MSC_APP_STRUCT_MAGIC_HEAD,                                                                                              \
    1,                                                                                                                      \
    NULL,                                                                                                                   \
    "",                                                                                                                     \
    NULL                                                                                                                    \
};
#define BUILT_IN_APP_TAB_END(n)                                                                                             \
__declspec(allocate(STR(BuiltinApp##n##Tab$1.end))) RT_USED static const builtin_app_desc_t __builtin_app##n##_table_end = \
{                                                                                                                           \
    MSC_APP_STRUCT_MAGIC_HEAD,                                                                                              \
    2,                                                                                                                      \
    NULL,                                                                                                                   \
    "",                                                                                                                     \
    NULL                                                                                                                    \
};

#endif

#if defined(__CC_ARM) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
#define BUILT_IN_APP_TAB(beg, end, n)                                               \
    extern const int BuiltinApp##n##Tab$$Base;                                      \
    extern const int BuiltinApp##n##Tab$$Limit;                                     \
    beg = (builtin_app_desc_t *) &BuiltinApp##n##Tab$$Base;                         \
    end = (builtin_app_desc_t *) &BuiltinApp##n##Tab$$Limit;
#elif defined (__ICCARM__) || defined(__ICCRX__)
#error "tobe contribute"
#elif defined (__GNUC__)
#define BUILT_IN_APP_TAB(beg, end, n)                                               \
    extern const int BuiltinApp##n##Tab_start;                                      \
    extern const int BuiltinApp##n##Tab_end;                                        \
    beg = (builtin_app_desc_t *)&BuiltinApp##n##Tab_start;                          \
    end = (builtin_app_desc_t *)&BuiltinApp##n##Tab_end;
#elif defined (_MSC_VER)
#define BUILT_IN_APP_TAB(beg, end, n)                                               \
    uint32_t *ptr_begin, *ptr_end;                                                  \
    ptr_begin = (uint32_t *)&__builtin_app##n##_table_start;                        \
    ptr_end = (uint32_t *)&__builtin_app##n##_table_end;                            \
    ptr_begin += (sizeof(builtin_app_desc_t) / sizeof(uint32_t));                   \
    while (*((uint32_t *)ptr_begin) == 0) ptr_begin ++;                             \
    ptr_end --;                                                                     \
    while (*((uint32_t *)ptr_end) == 0) ptr_end --;                                 \
    ptr_end++;                                                                      \
    beg = (builtin_app_desc_t *)ptr_begin;                                          \
    end = (builtin_app_desc_t *)ptr_end;
#endif


#if defined (_MSC_VER)
    #pragma section("BuiltinApp1Tab$0", read)
    BUILT_IN_APP_TAB_START(1);
    #pragma section("BuiltinApp1Tab$1.end", read)
    BUILT_IN_APP_TAB_END(1);

    #pragma section("BuiltinApp2Tab$0", read)
    BUILT_IN_APP_TAB_START(2);
    #pragma section("BuiltinApp2Tab$1.end", read)
    BUILT_IN_APP_TAB_END(2);
#endif
#endif

#if defined(APP_MENU_EXT_USED)
const mainmenu_ext_icons_t *mainmenu_ext_icons_table;
#endif

void builtin_app_get_table(builtin_app_desc_t **beg, builtin_app_desc_t **end, uint16_t style)
{
#if APP_REG_MULTI_STYLE_BUILTIN
#define APP_GET_TAB_N(n)                                                            \
        case n:                                                                     \
        {                                                                           \
            BUILT_IN_APP_TAB((*beg), (*end), n);                                    \
            break;                                                                  \
        }
    switch (style)
    {
    default:
        ;
        APP_GET_TAB_N(1);
        APP_GET_TAB_N(2);
    }
#else
    (void)style;
    if (beg) *beg = (builtin_app_desc_t *)gui_builtin_app_list_open();
    if (end) *end = NULL;
#endif
}

builtin_app_desc_t *builtin_app_get_next(builtin_app_desc_t *cur, const builtin_app_desc_t *end)
{
#if APP_REG_MULTI_STYLE_BUILTIN
    builtin_app_desc_t *ptr_app = cur + 1;

#if defined(BSP_USING_PC_SIMULATOR)
    for (uint32_t tmp = (uint32_t)ptr_app; tmp < (uint32_t)end; tmp++)
    {
        if (MSC_APP_STRUCT_MAGIC_HEAD == ((builtin_app_desc_t *)tmp)->magic_flag)
        {
            ptr_app = (builtin_app_desc_t *)tmp;
            break;
        }
    }
#endif

    if ((unsigned int *)ptr_app >= (unsigned int *)end)
        return NULL;
    else
        return (builtin_app_desc_t *)ptr_app;
#else
    (void)end;
    return (builtin_app_desc_t *)gui_builtin_app_list_get_next(cur);
#endif
}

int16_t menu_app_num_get(uint16_t style)
{
    builtin_app_desc_t *app;
    int16_t num = 0;
    builtin_app_desc_t *end = NULL;

    builtin_app_get_table(&app, &end, style);
    while (app)
    {
        if (app->icon != NULL && strcmp(app->id, "Main") != 0)
        {
            num++;
        }

#if APP_REG_MULTI_STYLE_BUILTIN
        app = builtin_app_get_next(app, end);
        if (end != NULL && app != NULL && app > end) app = NULL;
#else
        app = builtin_app_get_next(app, end);
#endif
    }

    return num;
}

void builtin_app_read_all(builtin_app_read_app_cb cb, app_list_sort_t *sort_tab, uint16_t size, uint16_t style, void *user_data)
{
    builtin_app_desc_t *start;
    builtin_app_desc_t *end = NULL;
    builtin_app_desc_t *temp_app;

    if (cb == NULL) return;

    builtin_app_get_table(&start, &end, style);

    if (sort_tab && size)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            temp_app = start;
            while (temp_app)
            {
                if (temp_app->icon != NULL && 0 == strcmp(sort_tab[i].id, temp_app->id))
                {
                    cb(temp_app, NULL, user_data);
                    break;
                }

#if APP_REG_MULTI_STYLE_BUILTIN
                temp_app = builtin_app_get_next(temp_app, end);
                if (end != NULL && temp_app != NULL && temp_app > end) temp_app = NULL;
#else
                temp_app = builtin_app_get_next(temp_app, end);
#endif
            }
        }
    }

    temp_app = start;
    while (temp_app)
    {
        uint32_t i = 0;

        for (i = 0; i < size; i++)
        {
            if (temp_app->icon != NULL && 0 == strcmp(sort_tab[i].id, temp_app->id))
            {
                break;
            }
        }

        if (i == size && temp_app->icon != NULL)
        {
            cb(temp_app, NULL, user_data);
        }

#if APP_REG_MULTI_STYLE_BUILTIN
        temp_app = builtin_app_get_next(temp_app, end);
        if (end != NULL && temp_app != NULL && temp_app > end) temp_app = NULL;
#else
        temp_app = builtin_app_get_next(temp_app, end);
#endif
    }

#if defined(APP_TOOL_SUPPORT)
    temp_app = NULL;
    while (1)
    {
        temp_app = (builtin_app_desc_t *)sfat_manager_get_app_next((void *)temp_app, style);
        if (temp_app == NULL) break;
        cb(temp_app, NULL, user_data);
    }
#endif

#if defined(APP_MENU_EXT_USED)
    if (mainmenu_ext_icons_table != NULL)
    {
        cb(NULL, (mainmenu_ext_icons_t *)mainmenu_ext_icons_table, user_data);
    }
#endif

#if APP_REG_MULTI_STYLE_BUILTIN
    for (uint8_t i = 0; i < SCRIPT_TYPE_NUM; i++)
    {
        temp_app = NULL;
        while (1)
        {
            temp_app = (builtin_app_desc_t *)gui_script_app_list_get_next(temp_app, i);
            if (temp_app == NULL)
            {
                gui_script_app_list_get_next((const builtin_app_desc_t *)-1, i);
                break;
            }
            cb(temp_app, NULL, user_data);
        }
    }
#else
    temp_app = NULL;
    while (1)
    {
        temp_app = (builtin_app_desc_t *)gui_script_app_list_get_next(temp_app);
        if (temp_app == NULL)
        {
#if defined(PKG_USING_MICROPYTHON) || defined(PKG_USING_QUICKJS)
            gui_script_app_list_get_next((const builtin_app_desc_t *)-1);
#endif
            break;
        }
        cb(temp_app, NULL, user_data);
    }
#endif
}

SECTION_DEF(APP_SUBPAGE_SECTION_NAME, app_subpage_desc_t);
app_subpage_desc_t *gui_app_get_subpage_desc(const char *app, const char *subpage)
{
    app_subpage_desc_t *subpage_desc;
    uint32_t *end;
    uint32_t *temp;

    if (!app || !subpage)
        return NULL;

    end = (uint32_t *)SECTION_END_ADDR(APP_SUBPAGE_SECTION_NAME);
    temp = (uint32_t *)SECTION_START_ADDR(APP_SUBPAGE_SECTION_NAME);
    while (temp < end)
    {
        subpage_desc = (app_subpage_desc_t *)temp;
        if (subpage_desc->app_id
                && subpage_desc->page_id
                && subpage_desc->handler)
        {
            if (0 == strcmp(subpage_desc->app_id, app) && 0 == strcmp(subpage_desc->page_id, subpage))
                return subpage_desc;
            temp += (sizeof(app_subpage_desc_t) >> 2);
        }
        else
        {
            temp++;
        }
    }

    return NULL;
}

int gui_app_create_subpage_ext(const char *app_id, const char *sub_id, void *user_data)
{
    int err = RT_EOK;

    app_subpage_desc_t *subpage_desc = gui_app_get_subpage_desc(app_id, sub_id);
    if (!subpage_desc)
    {
        rt_kprintf("%s:app_id %s sub_id %s is not fund! \n", __func__, app_id, sub_id);
        return RT_EINVAL;
    }

    err = APP_REG_GUI_APP_CREATE_PAGE_FOR_APP_EXT(app_id, sub_id, subpage_desc->handler, user_data, subpage_desc->mem_size);
    RT_ASSERT(RT_EOK == err);

    return err;
}

int gui_app_run_subpage(const char *app_id, const char *sub_id, void *user_data)
{
    if (!app_id || !sub_id)
    {
        return RT_ERROR;
    }

    if (gui_page_is_actived(app_id, sub_id))
    {
        rt_kprintf("%s:[%s][%s] is running, will do nothing! \n", __func__, app_id, sub_id);
        return RT_ERROR;
    }

    rt_kprintf("%s:app_name %s, subpage %s \n", __func__, app_id, sub_id);

    return gui_app_create_subpage_ext(app_id, sub_id, user_data);
}

const char *gui_app_get_reg_main(void)
{
    return g_reg_main;
}

int gui_app_set_reg_main(const char *main_name)
{
    if (main_name == NULL)
    {
        return RT_EINVAL;
    }

    g_reg_main = main_name;
    return RT_EOK;
}

int gui_app_get_builtin_app(void)
{
    return menu_app_num_get(0);
}

int gui_app_get_ext_app(void)
{
    return 0;
}

int gui_app_get_dyn_app(void)
{
    return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
