/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_GIF_H_
#define _LVSF_GIF_H_
#include "lvgl.h"
#include "gif_dec.h"

lv_obj_t *lv_sfgif_create(lv_obj_t *parent);

void lv_sfgif_open(lv_obj_t *gif, const char *gif_data, const char *src_img,
                   lv_coord_t x, lv_coord_t y, uint32_t anim_time, uint16_t period);

void lv_sfgif_resume(lv_obj_t *gif);

void lv_sfgif_pause(lv_obj_t *gif);

void lv_sfgif_close(lv_obj_t *gif);

void lv_sfgif_set_zoom(lv_obj_t *gif, uint16_t zoom);

#endif
