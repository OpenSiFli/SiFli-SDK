/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MULTLIST_CONFIG_H
#define MULTLIST_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* 原 app_module.h 内容 */
#include <stddef.h>
#include <stdint.h>

#include "app_reg.h"

typedef struct
{
    char *data;
} comm_msg_t;

const char *app_get_str_from_id(uint32_t id);

#define app_get_strid(key, fallback) ((uint32_t)(key))

static inline void send_msg_to_gui_thread(char *str,
                                          size_t len,
                                          void (*cb)(comm_msg_t *),
                                          uint32_t arg0,
                                          uint32_t arg1)
{
    comm_msg_t msg;

    LV_UNUSED(len);
    LV_UNUSED(arg0);
    LV_UNUSED(arg1);

    if(cb == NULL) return;

    msg.data = str;
    cb(&msg);
}

#define NEED_WAKEUP_UI 0
#define APP_GET_IMG(name) LV_EXT_IMG_GET(name)

enum
{
    key_list_hor = 0,
    key_list_ver,
    key_page_hor,
    key_page_ver,
    key_anim_hor,
    key_anim_ver,
    key_list_intercom,
    key_multlist_img,
};

/* 原 global.h 内容（合并进来） */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rtthread.h"
#include "lvgl.h"
#include "lv_gesture.h"
#include "app_mem.h"
#include "lv_ext_resource_manager.h"
#include "lvsf_font.h"

LV_IMG_DECLARE(demo_list_0);
LV_IMG_DECLARE(demo_list_1);
LV_IMG_DECLARE(demo_list_2);
LV_IMG_DECLARE(demo_list_3);
LV_IMG_DECLARE(menu_multlist);

lv_obj_t *demo_multlist_create_back_button(lv_obj_t *parent);

#ifdef __cplusplus
}
#endif

#endif /* MULTLIST_CONFIG_H */
