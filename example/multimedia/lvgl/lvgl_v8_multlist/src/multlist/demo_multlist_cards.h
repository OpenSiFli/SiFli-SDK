/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVGL_V8_MULTLIST_DEMO_MULTLIST_CARDS_H
#define LVGL_V8_MULTLIST_DEMO_MULTLIST_CARDS_H

#include "global.h"

typedef enum
{
    DEMO_MULTLIST_CARD_STYLE_LIST = 0,
    DEMO_MULTLIST_CARD_STYLE_ANIM,
    DEMO_MULTLIST_CARD_STYLE_COUNT
} demo_multlist_card_style_t;

void demo_multlist_card_cache_init(void);
void demo_multlist_card_cache_deinit(void);

uint32_t demo_multlist_get_image_count(void);
const void *demo_multlist_get_image_src(uint32_t index);

const lv_img_dsc_t *demo_multlist_get_card_snapshot(demo_multlist_card_style_t style,
                                                    uint32_t index);
void demo_multlist_get_card_size(demo_multlist_card_style_t style,
                                 lv_coord_t *w,
                                 lv_coord_t *h);

#endif
