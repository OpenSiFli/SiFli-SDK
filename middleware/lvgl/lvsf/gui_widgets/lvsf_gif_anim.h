/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_GIF_ANIM_H
#define _LVSF_GIF_ANIM_H

#include <rtthread.h>
#include "lvgl.h"
#include "app_mem.h"
#include "service_comm.h"

#define GIF_ANIM_DELAY   100

enum
{
    LVSF_GIF_LAYER_DEFAULT,
    LVSF_GIF_LAYER_FOREGROUND,
    LVSF_GIF_LAYER_BACKGROUND,
};

typedef uint16_t lvsf_gif_layer_t;


typedef struct
{
    lv_obj_t *parent;
    lv_obj_t *gif;
    lv_obj_t *gif_img;
    const void *gif_data;
    const void *img_data;

    uint32_t duration;
    uint32_t delay;
    int width_zoom;
    int height_zoom;
    lvsf_gif_layer_t layer;
    lv_color_t bg_color;
    bool en_bg_color;

} lvsf_gif_anim_t;

typedef struct
{
    int16_t color_depth;
    bool gif_release_on_pause;

} lvsf_gif_anim_cfg_t;

void lvsf_gif_anim_set_zoom(lvsf_gif_anim_t *gif_anim, int width_zoom, int height_zoom);
void lvsf_gif_anim_set_layer(lvsf_gif_anim_t *gif_anim, lvsf_gif_layer_t layer);
void lvsf_gif_anim_enable_bg_color(lvsf_gif_anim_t *gif_anim, bool enable);

lvsf_gif_anim_t *lvsf_gif_anim_init(lv_obj_t *parent, const void *gif_data, const void *src_img,
                                    lv_coord_t x, lv_coord_t y, uint32_t duration, uint32_t delay);
void lvsf_gif_anim_resume(lvsf_gif_anim_t *gif_anim);
void lvsf_gif_anim_pause(lvsf_gif_anim_t *gif_anim);
void lvsf_gif_anim_deinit(lvsf_gif_anim_t *gif_anim);


#endif /* _LVSF_GIF_ANIM_H */


