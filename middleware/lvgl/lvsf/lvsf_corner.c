/**
  ******************************************************************************
  * @file   lvsf_corner.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include "rtconfig.h"

#include "lvgl.h"
#include "lvsf.h"

#if LVSF_USE_CORNER!=0

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_lvsfcorner_class


#define LVSF_CORNER_CURVE_TEXT_LEN  14
#define LVSF_CORNER_CURVE_TEXT_SIZE 1
#define LVSF_CORNER_TEXT_LEN        3
#define LVSF_CORNER_TEXT_SIZE       2


#if (IMAGE_CACHE_IN_PSRAM_SIZE > 0 || IMAGE_CACHE_IN_SRAM_SIZE > 0)
    #include "app_mem.h"
    #define corner_buf_alloc app_canvas_mem_alloc
    #define corner_buf_free app_canvas_mem_free
#else
    #define corner_buf_alloc lv_mem_alloc
    #define corner_buf_free lv_mem_free
#endif

/**********************
 *      TYPEDEFS
 **********************/
/// Sifli LVGL Corner data structure.
typedef struct
{
    lv_canvas_t obj_ext;
    uint16_t x;             //<! Central X
    uint16_t y;             //<! Central y
    uint16_t radius;        //<! Radius of corner
    uint16_t zone;          //<! Quad zone, 1:right top, 2 left top, 3:left bottom, 4 right bottom
    lv_color_t bg_color;    //<! Back ground color
    lv_obj_t *arc;          //<! Arc ring in the quater
    lv_obj_t *img;          //<! image in the quater
    lv_obj_t *text;         //<! text in the quater
    uint8_t *buf;           //<! Buffer for the quarter canvas
} lv_lvsfcorner_ext_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_lvsfcorner_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfcorner_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);


/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_lvsfcorner_class =
{
    .constructor_cb = lv_lvsfcorner_constructor,
    .destructor_cb = lv_lvsfcorner_destructor,
    .instance_size = sizeof(lv_lvsfcorner_ext_t),
    .base_class = &lv_canvas_class
};

/**********************
 *      MACROS
 **********************/


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_lvsfcorner_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    lv_lvsfcorner_ext_t *ext = (lv_lvsfcorner_ext_t *)obj;
    LV_UNUSED(class_p);

    ext->buf = (uint8_t *)corner_buf_alloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(GRA_CORNER_SIZE, GRA_CORNER_SIZE));
    if (!ext->buf)
    {
        LV_LOG("lvsf_corner_create: size %d\n", LV_CANVAS_BUF_SIZE_TRUE_COLOR(GRA_CORNER_SIZE, GRA_CORNER_SIZE));
        LV_ASSERT(ext->buf);
    }
    lv_canvas_set_buffer(obj, ext->buf, GRA_CORNER_SIZE, GRA_CORNER_SIZE, LV_IMG_CF_TRUE_COLOR);
    memset(ext->buf, 0, LV_CANVAS_BUF_SIZE_TRUE_COLOR(GRA_CORNER_SIZE, GRA_CORNER_SIZE));
    lv_obj_move_background(obj);
}

static void lv_lvsfcorner_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

    lv_lvsfcorner_ext_t *ext = (lv_lvsfcorner_ext_t *)obj;

    if (ext && ext->buf)
        corner_buf_free(ext->buf);
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
lv_obj_t *lv_lvsfcorner_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(obj, GRA_CORNER_SIZE, GRA_CORNER_SIZE);
    lv_obj_update_layout(obj);
    return obj;
}


void lv_lvsfcorner_zone(lv_obj_t *corner, uint16_t zone, uint16_t r, uint16_t x, uint16_t y)
{
    lv_lvsfcorner_ext_t *ext = (lv_lvsfcorner_ext_t *)corner;
    int corner_startx, corner_starty; //top-left coordinate of canvas

    ext->radius = r;
    ext->zone = zone;
    ext->x = x;
    ext->y = y;
    switch (ext->zone)
    {
    case 1:
        corner_startx = 2 * ext->radius - GRA_CORNER_SIZE;
        corner_starty = 0;
        break;
    case 2:
        corner_startx = 0;
        corner_starty = 0;
        break;
    case 3:
        corner_startx = 0;
        corner_starty = 2 * ext->radius - GRA_CORNER_SIZE;
        break;
    case 4:
        corner_startx = 2 * ext->radius - GRA_CORNER_SIZE;
        corner_starty = 2 * ext->radius - GRA_CORNER_SIZE;
        break;
    }
    lv_obj_set_pos(corner, ext->x + corner_startx - ext->radius, ext->y + corner_starty - ext->radius);
    lv_obj_update_layout(corner);
}


lv_obj_t *lv_lvsfcorner_curve_text(lv_obj_t *corner, char *text)
{
    lv_lvsfcorner_ext_t *ext = (lv_lvsfcorner_ext_t *)corner;
    int start;

    memset(ext->buf, 0, LV_CANVAS_BUF_SIZE_TRUE_COLOR(GRA_CORNER_SIZE, GRA_CORNER_SIZE));
    start = 90 * (ext->zone - 1) + 45;

    if (ext->zone > 2)
        start -= (TEXT_ANGLE * (LVSF_CORNER_CURVE_TEXT_LEN) / 2);
    else
        start += (TEXT_ANGLE * (LVSF_CORNER_CURVE_TEXT_LEN) / 2);
    lv_color_t color = lv_obj_get_style_text_color(corner, LV_PART_MAIN);

    int corner_startx = lv_obj_get_x(corner) + ext->radius - ext->x;
    int corner_starty = lv_obj_get_y(corner) + ext->radius - ext->y;
    lvsf_curve_draw_text(corner, text, corner_startx, corner_starty, start, ext->radius, color, LVSF_CORNER_CURVE_TEXT_SIZE);
    return corner;
}

lv_obj_t *lv_lvsfcorner_arc(lv_obj_t *corner, int start, int end, lv_color_t color)
{
    lv_lvsfcorner_ext_t *ext = (lv_lvsfcorner_ext_t *)corner;
    if (ext->arc)
        lv_obj_del(ext->arc);
    ext->arc = lv_arc_create(corner);
    lv_obj_set_style_arc_color(ext->arc, color, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ext->arc, GRA_CORNER_LINE_WIDTH, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ext->arc, LV_OPA_TRANSP, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ext->arc, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(ext->arc, LV_OBJ_FLAG_CLICKABLE);
    lv_arc_set_angles(ext->arc, start, end);
    lv_arc_set_bg_angles(ext->arc, start, end);
    lv_obj_set_size(ext->arc, ext->radius * 2, ext->radius * 2);
    lv_obj_set_pos(ext->arc, ext->x - ext->radius - lv_obj_get_x(corner), ext->y - ext->radius - lv_obj_get_y(corner));
    lv_obj_update_layout(ext->arc);
    return corner;
}

void lv_lvsfcorner_arc_scale(lv_obj_t *corner, int percent)
{
    lv_lvsfcorner_ext_t *ext = (lv_lvsfcorner_ext_t *)corner;
    lv_arc_set_value(ext->arc, percent);
}

lv_obj_t *lv_lvsfcorner_img(lv_obj_t *corner, const char *src)
{
    lv_lvsfcorner_ext_t *ext = (lv_lvsfcorner_ext_t *)corner;
    int x, y;
    int radius = ext->radius + ext->radius / 10;

    if (ext->img)
        lv_obj_del(ext->img);
    ext->img = lv_img_create(corner);
    lv_img_set_src(ext->img, src);


    lv_img_t *img_ext = (lv_img_t *)(ext->img);

    switch (ext->zone)
    {
    case 1:
        x = ext->x - lv_obj_get_x(corner) + ((lv_trigo_sin(45) * radius) >> LV_TRIGO_SHIFT);
        y = ext->y - lv_obj_get_y(corner) - ((lv_trigo_sin(45) * radius) >> LV_TRIGO_SHIFT) - img_ext->h;
        break;
    case 2:
        y = ext->y - lv_obj_get_y(corner) - ((lv_trigo_sin(45) * radius) >> LV_TRIGO_SHIFT) - img_ext->h;
        x = ext->x - lv_obj_get_x(corner) - ((lv_trigo_sin(45) * radius) >> LV_TRIGO_SHIFT) - img_ext->w;
        break;
    case 3:
        x = ext->x - lv_obj_get_x(corner) - ((lv_trigo_sin(45) * radius) >> LV_TRIGO_SHIFT) - img_ext->w;
        y = ext->y - lv_obj_get_y(corner) + ((lv_trigo_sin(45) * radius) >> LV_TRIGO_SHIFT);
        break;
    case 4:
        x = ext->x - lv_obj_get_x(corner) + ((lv_trigo_sin(45) * radius) >> LV_TRIGO_SHIFT);
        y = ext->y - lv_obj_get_y(corner) + ((lv_trigo_sin(45) * radius) >> LV_TRIGO_SHIFT);
        break;
    }

    lv_obj_clear_flag(ext->img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_pos(ext->img, x, y);
    lv_obj_update_layout(ext->img);
    return corner;
}

lv_obj_t *lv_lvsfcorner_text(lv_obj_t *corner,  char *text)
{
    lv_lvsfcorner_ext_t *ext = (lv_lvsfcorner_ext_t *)corner;
    int x, y, angle;
    int icon_width, icon_height, radius_icon;
    lv_color_t color;
    int size = LVSF_CORNER_TEXT_SIZE;

    icon_width = LVSF_CORNER_TEXT_LEN * lvsf_font_width(size, text[0]);
    icon_height = lvsf_font_height(size);
    radius_icon = ext->radius + icon_height * LVSF_CORNER_TEXT_LEN / 2;
    color = lv_obj_get_style_text_color(corner, LV_PART_MAIN);

    switch (ext->zone)
    {
    case 1:
        angle = 45;
        x = ext->x - lv_obj_get_x(corner) + ((lv_trigo_sin(45) * (2 * radius_icon - icon_width)) >> (LV_TRIGO_SHIFT + 1));
        y = ext->y - lv_obj_get_y(corner) - ((lv_trigo_sin(45) * (2 * radius_icon + icon_width)) >> (LV_TRIGO_SHIFT + 1));
        break;
    case 2:
        angle = -45;
        x = ext->x - lv_obj_get_x(corner) - ((lv_trigo_sin(45) * (2 * radius_icon + icon_width)) >> (LV_TRIGO_SHIFT + 1));
        y = ext->y - lv_obj_get_y(corner) - ((lv_trigo_sin(45) * (2 * radius_icon - icon_width)) >> (LV_TRIGO_SHIFT + 1));
        break;
    case 3:
        angle = 45;
        radius_icon -= icon_height;
        x = ext->x - lv_obj_get_x(corner) - ((lv_trigo_sin(45) * (2 * radius_icon + icon_width)) >> (LV_TRIGO_SHIFT + 1));
        y = ext->y - lv_obj_get_y(corner) + ((lv_trigo_sin(45) * (2 * radius_icon - icon_width)) >> (LV_TRIGO_SHIFT + 1));
        break;
    case 4:
        angle = -45;
        radius_icon -= icon_height;
        x = ext->x - lv_obj_get_x(corner) + ((lv_trigo_sin(45) * (2 * radius_icon - icon_width)) >> (LV_TRIGO_SHIFT + 1));
        y = ext->y - lv_obj_get_y(corner) + ((lv_trigo_sin(45) * (2 * radius_icon + icon_width)) >> (LV_TRIGO_SHIFT + 1));
        break;
    }
    lvsf_text_create2(corner, text, x, y, angle, color, size);


    return corner;
}

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
