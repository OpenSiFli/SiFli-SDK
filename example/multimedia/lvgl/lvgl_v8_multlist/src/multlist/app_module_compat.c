/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app_module.h"

const char *app_get_str_from_id(uint32_t id)
{
    switch(id)
    {
    case key_list_hor:
        return "horizontal list";
    case key_list_ver:
        return "vertical list";
    case key_page_hor:
        return "horizontal page";
    case key_page_ver:
        return "vertical page";
    case key_anim_hor:
        return "horizontal aniamtion";
    case key_anim_ver:
        return "vertical aniamtion";
    case key_list_intercom:
        return "Intercom list";
    case key_multlist_img:
        return "multlist";
    default:
        return "";
    }
}
