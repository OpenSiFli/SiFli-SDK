/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
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
#include <log.h>
#include "app_reg.h"
#ifdef APP_DLMODULE_APP_USED
    #include "dynamic_app.h"
#endif
#ifdef APP_TOOL_SUPPORT
    #include "app_tool_comm.h"
#endif

#include "lvsf_resource.h"

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

#if defined(__CC_ARM) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))     /* ARM C Compiler */
#define BUILT_IN_APP_TAB(beg, end, n)                                               \
    extern const int BuiltinApp##n##Tab$$Base;                                      \
    extern const int BuiltinApp##n##Tab$$Limit;                                     \
    beg = (builtin_app_desc_t *) &BuiltinApp##n##Tab$$Base;                         \
    end = (builtin_app_desc_t *)   &BuiltinApp##n##Tab$$Limit;
#elif defined (__ICCARM__) || defined(__ICCRX__)      /* for IAR Compiler */
#error "tobe contribute"
#elif defined (__GNUC__)                              /* for GCC Compiler */
#define BUILT_IN_APP_TAB(beg, end, n)                                               \
    extern const int BuiltinApp##n##Tab_start;                                      \
    extern const int BuiltinApp##n##Tab_end;                                        \
    beg = (builtin_app_desc_t *)&BuiltinApp##n##Tab_start;                          \
    end = (builtin_app_desc_t *) &BuiltinApp##n##Tab_end;
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
#endif /* defined(__CC_ARM) */


#if defined (_MSC_VER)
    //Register section of the first built_in_app table
    #pragma section("BuiltinApp1Tab$0", read)
    BUILT_IN_APP_TAB_START(1);
    #pragma section("BuiltinApp1Tab$1.end", read)
    BUILT_IN_APP_TAB_END(1);

    //Register section of the second built_in_app table
    #pragma section("BuiltinApp2Tab$0", read)
    BUILT_IN_APP_TAB_START(2);
    #pragma section("BuiltinApp2Tab$1.end", read)
    BUILT_IN_APP_TAB_END(2);
#endif

#if defined(APP_MENU_EXT_USED)
const mainmenu_ext_icons_t *mainmenu_ext_icons_table;

#if defined(APP_TLV_USED)
#define MAINMENU_EXT_ICONS_TABLE(n) \
        const mainmenu_ext_icons_t mainmenu_ext_icons##n##_table[] =                                                                                      \
        {                                                                                                                                                 \
            {"lang",       (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_language, n),           app_get_strid(key_language, "Language")},                       \
            {"sounds",     (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_setting_sound, n),      app_get_strid(key_sounds_vibrate, "Sound and Vibrate")},        \
            {"display",    (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_setting_brightness, n), app_get_strid(key_display, "Display")},   \
            {"lockscreen", (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_lock_screen, n),        app_get_strid(key_lock_screen, "Lock screen")},                 \
            {"menuview",   (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_view, n),             app_get_strid(key_menu_view, "Menu view")},                     \
            {"battery",    (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_power_down, n),         app_get_strid(key_battery, "Battery")},                         \
            {"tlv_anim",   (lv_img_dsc_t *)APP_GET_IMG_N(img_3d, n),                      app_get_strid(key_tlv_anim, "Tlv anim")},                       \
            {"develop",    (lv_img_dsc_t *)APP_GET_IMG_N(img_ecg, n),                     app_get_strid(key_developer_mode, "Developer mode")},           \
            {"about",      (lv_img_dsc_t *)APP_GET_IMG_N(img_about, n),                   app_get_strid(key_about, "About")},                             \
            {"system",     (lv_img_dsc_t *)APP_GET_IMG_N(img_powerOff, n),                app_get_strid(key_system, "System")},                           \
        };
#else
#define MAINMENU_EXT_ICONS_TABLE(n) \
        const mainmenu_ext_icons_t mainmenu_ext_icons##n##_table[] =                                                                                      \
        {                                                                                                                                                 \
            {"lang",       (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_language, n),           app_get_strid(key_language, "Language")},                       \
            {"sounds",     (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_setting_sound, n),      app_get_strid(key_sounds_vibrate, "Sound and Vibrate")},        \
            {"display",    (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_setting_brightness, n), app_get_strid(key_display, "Display")},   \
            {"lockscreen", (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_lock_screen, n),        app_get_strid(key_lock_screen, "Lock screen")},                 \
            {"menuview",   (lv_img_dsc_t *)APP_GET_IMG_N(img_menu_view, n),             app_get_strid(key_menu_view, "Menu view")},                     \
            {"develop",    (lv_img_dsc_t *)APP_GET_IMG_N(img_ecg, n),                     app_get_strid(key_developer_mode, "Developer mode")},           \
            {"about",      (lv_img_dsc_t *)APP_GET_IMG_N(img_about, n),                   app_get_strid(key_about, "About")},                             \
            {"system",     (lv_img_dsc_t *)APP_GET_IMG_N(img_powerOff, n),                app_get_strid(key_system, "System")},                           \
        };

#endif
#define APP_GET_IMG_N(img, n) APP_GET_IMG_FROM_APP(mainmenu, img)
//Register the first extended icon table, whose image file is special and does not contain numbers
MAINMENU_EXT_ICONS_TABLE(1);
#undef APP_GET_IMG_N
#define APP_GET_IMG_N(img, n) APP_GET_IMG_FROM_APP(mainmenu##n, img##n)

//Register the second extended icon table, whose image file contain numbers
MAINMENU_EXT_ICONS_TABLE(2);

/*
//Register the third extended icon table, whose image file contain numbers
MAINMENU_EXT_ICONS_TABLE(3);

//Register the subsequent extended icon table, whose image file contain numbers
MAINMENU_EXT_ICONS_TABLE(4);
*/
#endif

void builtin_app_get_table(builtin_app_desc_t **beg, builtin_app_desc_t **end, uint16_t n)
{
#if defined(APP_MENU_EXT_USED)
#define APP_GET_TAB_N(n)                                                            \
        case n:                                                                     \
        {                                                                           \
            BUILT_IN_APP_TAB((*beg), (*end), n);                                    \
            extern const mainmenu_ext_icons_t mainmenu_ext_icons##n##_table[];      \
            mainmenu_ext_icons_table = &mainmenu_ext_icons##n##_table[0];           \
            break;                                                                  \
        }
#else
#define APP_GET_TAB_N(n)                                                            \
        case n:                                                                     \
        {                                                                           \
            BUILT_IN_APP_TAB((*beg), (*end), n);                                    \
            break;                                                                  \
        }

#endif
    switch (n)
    {
    default:
        ;
        APP_GET_TAB_N(1);
        APP_GET_TAB_N(2);
        /*
        APP_GET_TAB_N(3);
        APP_GET_TAB_N(4);
        */
    }
}

#if defined(APP_MENU_EXT_USED)
uint16_t menu_ext_icons_table_size_get(void)
{
    return (uint16_t)(sizeof(mainmenu_ext_icons1_table) / sizeof(mainmenu_ext_icons_t));
}
#endif

builtin_app_desc_t *builtin_app_get_next(builtin_app_desc_t *cur, const builtin_app_desc_t *end)
{
    builtin_app_desc_t *ptr_app = cur + 1;

    //handle the align between setion and structure.
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
}

int16_t menu_app_num_get(uint16_t style)
{
    builtin_app_desc_t *p_builtin_app = NULL;
    uint16_t num = 0;
    builtin_app_desc_t *p_start_app;
    builtin_app_desc_t *p_end_app;
    builtin_app_get_table(&p_start_app, &p_end_app, style);

    int tlv_num = 0;
#ifdef APP_TLV_USED
    tlv_num = tlv_count_get();
#endif

    p_builtin_app = p_start_app;
    if (p_builtin_app)
    {
        do
        {
            if (strcmp(p_builtin_app->id, "aod_main")  &&                     /*Exclude the AOD */
                    strcmp(p_builtin_app->id, "OTA")  &&                      /*Exclude the OTA */
                    strcmp(p_builtin_app->id, "Main") &&                      /*Exclude the Main */
                    (strcmp(p_builtin_app->id, "Tileview") || 0 < tlv_num))   /*Exclude Tlv_Fwk enable but no tlv application */
            {
                num++;
            }

            p_builtin_app = builtin_app_get_next(p_builtin_app, p_end_app);
        }
        while (p_builtin_app && p_builtin_app <= p_end_app);
        p_builtin_app = NULL;
    }
    LOG_I("%s: num %d", __func__, num);
    return num;
}

void builtin_app_read_all(builtin_app_read_app_cb cb, app_list_sort_t *sort_tab, uint16_t size, uint16_t style, void *user_data)
{
    builtin_app_desc_t *p_start_app;
    builtin_app_desc_t *p_end_app;
    uint16_t ret_cnt = 0;

    builtin_app_get_table(&p_start_app, &p_end_app, style);

    builtin_app_desc_t *temp_app = NULL;
    if (sort_tab && size)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            temp_app = p_start_app;
            while (temp_app && temp_app <= p_end_app)
            {
                if (NULL != temp_app->icon && 0 == strcmp(sort_tab[i].id, temp_app->id))
                {
                    cb(temp_app, NULL, user_data);
                    break;
                }
                temp_app = builtin_app_get_next(temp_app, p_end_app);
            }
        }
    }
    temp_app = p_start_app;
    while (temp_app && temp_app <= p_end_app)
    {
        uint32_t i = 0;
        for (i = 0; i < size; i++)
        {
            if (NULL != temp_app->icon && 0 == strcmp(sort_tab[i].id, temp_app->id))
                break;
        }
        if (i == size && temp_app->icon)
            cb(temp_app, NULL, user_data);
        temp_app = builtin_app_get_next(temp_app, p_end_app);
    }


#ifdef APP_TOOL_SUPPORT
    temp_app = NULL;
    while (1)
    {
        temp_app = (builtin_app_desc_t *)sfat_manager_get_app_next((void *)temp_app, style);
        if (temp_app == NULL) break;
        cb(temp_app, NULL, user_data);
    }
#endif

#if defined(APP_DLMODULE_APP_USED)
    dyn_app_node_t *node = NULL;
    while ((node = dynamic_app_list_get_app_next(node)))
    {
        temp_app = &node->desc;
        node->ext_icon = dynamic_app_get_icon(node->desc.id, style);
        LOG_I("%s:get dynamic app %s, icon %s", __func__, node->desc.id, node->ext_icon);
        cb(temp_app, NULL, user_data);
    }
#endif

#if defined(APP_MENU_EXT_USED)
    const mainmenu_ext_icons_t *ext_icon_table;
    GET_EXT_APPLICATION_TABLE(ext_icon_table)
    uint16_t k;

    uint16_t tabSize = menu_ext_icons_table_size_get();
    LOG_I("%s: load ext icon %d", __func__, tabSize);
    for (k = 0; k < tabSize; k++)
    {
        cb(NULL, (mainmenu_ext_icons_t *)&ext_icon_table[k], user_data);
    }
#endif

    for (uint8_t i = 0; i < SCRIPT_TYPE_NUM; i++)
    {
        temp_app = NULL;
        while (1)
        {
            temp_app = (builtin_app_desc_t *)gui_script_app_list_get_next(temp_app, i);
            if (temp_app == NULL)
            {
                gui_script_app_list_get_next((const builtin_app_desc_t *) -1, i);
                break;
            }
            cb(temp_app, NULL, user_data);
        }
    }
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
        if (subpage_desc->app_id \
                && subpage_desc->page_id \
                && subpage_desc->handler)
        {
            //rt_kprintf("%s:app_id %s  page_id %s \n", __func__, subpage_desc->app_id, subpage_desc->page_id);
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

    err = gui_app_create_page_for_app_ext(app_id, sub_id, subpage_desc->handler, user_data, subpage_desc->mem_size);
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

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

