/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVGL_V8_MULTLIST_GLOBAL_H
#define LVGL_V8_MULTLIST_GLOBAL_H

#include <stdbool.h>
#include <stdint.h>
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

#endif
