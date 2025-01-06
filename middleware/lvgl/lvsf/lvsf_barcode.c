/**
  ******************************************************************************
  * @file   lvsf_comp.c
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

#if LVSF_USE_BARCODE!=0
#include "barcode128.h"

#define BARCODE_MARGIN 4

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_lvsfbarcode_class

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
    lv_img_t img;
    lv_point_t *points;
} lv_lvsfbarcode_ext_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_lvsfbarcode_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfbarcode_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);


/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_lvsfbarcode_class =
{
    .constructor_cb = lv_lvsfbarcode_constructor,
    .destructor_cb = lv_lvsfbarcode_destructor,
    .instance_size = sizeof(lv_lvsfbarcode_ext_t),
    .base_class = &lv_img_class
};

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_lvsfbarcode_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{

    LV_UNUSED(class_p);
}

static void lv_lvsfbarcode_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    lv_lvsfbarcode_ext_t *ext = (lv_lvsfbarcode_ext_t *)obj;

    if (ext->points)
    {
        lv_mem_free(ext->points);
        ext->points = NULL;
    }
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_obj_t *lv_lvsfbarcode_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    return obj;
}

void lv_lvsfbarcode_set_text(lv_obj_t *barcode, char *text)
{
    char *coded_string;
    int coded_len;
    int scale, i, width, height;
    int x;
    lv_lvsfbarcode_ext_t *ext = (lv_lvsfbarcode_ext_t *)barcode;
    lv_point_t *point_a;

    coded_len = (strlen(text) + 2) * 6 + 1;
    coded_string = (char *)lv_mem_alloc(coded_len * 2);
    barcode128_parse(text, strlen(text), coded_string, &coded_len, 'A');

    for (i = 0, width = 0; i < (int)strlen(coded_string); i++)
    {
        width += (coded_string[i] - '0');
    }
    scale = (LV_HOR_RES - BARCODE_MARGIN * 2) / width;
    if (scale == 0)
        scale = 1;
    height = width * scale / 3;

    if (ext->points)
        lv_mem_free(ext->points);
    ext->points = lv_mem_alloc((strlen(coded_string) + 1) * sizeof(lv_point_t));
    point_a = ext->points;

    lv_obj_set_size(barcode, width * scale, height);
    for (i = 0, x = 0; i < (int)strlen(coded_string) && x < LV_HOR_RES; i++)
    {
        lv_obj_t *line;
        width = (coded_string[i] - '0');
        if ((i % 2) == 1)
        {
            line = lv_line_create(barcode);
            lv_obj_set_style_line_color(line, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_line_width(line, width * scale, LV_PART_MAIN | LV_STATE_DEFAULT);
            point_a[0].x = x + width * scale / 2;
            point_a[0].y = 0;
            point_a[1].x = x + width * scale / 2;
            point_a[1].y = height;
#ifdef DISABLE_LVGL_V8
            lv_line_set_points(line, (lv_point_precise_t *)point_a, 2);
#else
            lv_line_set_points(line, point_a, 2);
#endif
            point_a += 2;
        }
        x += width * scale;
    }
    lv_mem_free(coded_string);
}

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

