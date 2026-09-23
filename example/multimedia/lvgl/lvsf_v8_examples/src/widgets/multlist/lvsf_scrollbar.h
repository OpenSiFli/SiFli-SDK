/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVGL_V8_MULTLIST_SCROLLBAR_H
#define LVGL_V8_MULTLIST_SCROLLBAR_H

#include "lvgl.h"

#ifndef LV_SCROLLBAR_SQUARE_TYPE
#define LV_SCROLLBAR_SQUARE_TYPE 0
#endif

static inline void lv_scrollbar_create(lv_obj_t *obj, int type)
{
    LV_UNUSED(type);
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_AUTO);
}

#endif
