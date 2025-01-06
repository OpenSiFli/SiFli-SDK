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

#if LVSF_USE_COMPOSITE!=0

#include "stdio.h"

const lv_point_t g_comp_info[] =
{
    {CIRCULAR_SIZE,         CIRCULAR_SIZE},
    {MODULAR_SMALL_SIZE,    MODULAR_SMALL_SIZE},
    {MODULAR_LARGE_SIZE,    MODULAR_LARGE_SIZE},
    {EXTRA_LARGE_SIZE,      EXTRA_LARGE_SIZE},
    {UTILITY_SIZE,          UTILITY_SIZE},
    {GRA_CIRCULAR_SIZE,     GRA_CIRCULAR_SIZE},
};


/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_lvsfcomp_class

/**********************
 *      TYPEDEFS
 **********************/
/// Sifli LVGL Corner data structure.
typedef struct
{
    lv_obj_t obj;
    lvsf_comp_type type;    //<! type of composite
    lv_obj_t *arc;          //<! arc in the composite
    lv_obj_t *img;          //<! image in the quater
    lv_obj_t *ring[3];      //<! rings in the composite(max 3 rings)
    lv_obj_t *text;         //<! text in the composite
} lv_lvsfcomp_ext_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_lvsfcomp_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfcomp_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);


/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_lvsfcomp_class =
{
    .constructor_cb = lv_lvsfcomp_constructor,
    .destructor_cb = lv_lvsfcomp_destructor,
    .instance_size = sizeof(lv_lvsfcomp_ext_t),
    .base_class = &lv_obj_class
};

/**********************
 *      MACROS
 **********************/


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_lvsfcomp_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

}

static void lv_lvsfcomp_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

    lv_lvsfcomp_ext_t *ext = (lv_lvsfcomp_ext_t *)obj;
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/*
 * Create UI composite
 * @param parent parent of the composite object
 * @param type type of the composite
 * @retval return the composite object created.
*/

lv_obj_t *lv_lvsfcomp_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    return obj;
}

void lv_lvsfcomp_set_type(lv_obj_t *comp, int type)
{
    lv_lvsfcomp_ext_t *ext = (lv_lvsfcomp_ext_t *)comp;
    ext->type = (lvsf_comp_type)type;
    lv_obj_set_size(comp, g_comp_info[type].x, g_comp_info[type].y);
    lv_obj_update_layout(comp);
}

/**
 * Create text in UI composite
 * @param comp the composite object
 * @param text text content
 * @note color are in the text(use recolor), font is set in themes.
 * @retval return the label object created.
*/
lv_obj_t *lv_lvsfcomp_text(lv_obj_t *comp, const char *text)
{
    lv_obj_t *label = lv_label_create(comp);
    lv_lvsfcomp_ext_t *ext = (lv_lvsfcomp_ext_t *)comp;

    lv_label_set_recolor(label, true);
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
    if (ext->text)
        lv_obj_del(ext->text);
    ext->text = label;
    return label;
}

/**
 * Create Ring in UI composite
 * @param comp the composite object
 * @param index The ring index, 0  means the outter most.
 * @param scale the ring in percentage,
 * @param color the color of the ring
 * @param bg_color the background color of the ring
 * @note radius is half width of comp, ring width and rounded is in theme,
    user could use lv_lvsfcomp_set_ring to update the ring scale.
 * @retval return the ring object created.
*/
lv_obj_t *lv_lvsfcomp_ring(lv_obj_t *comp, int index, int scale, lv_color_t color, lv_color_t bg_color)
{
    lv_lvsfcomp_ext_t *ext = (lv_lvsfcomp_ext_t *)comp;
    int width = lv_obj_get_width(comp);
    int height = lv_obj_get_height(comp);
    int line_width;

    LV_DEBUG_ASSERT(index < 3, "Index error", index);
    lv_obj_t *ring = lv_arc_create(comp);

    line_width = lv_obj_get_style_line_width(comp, LV_PART_MAIN);

    width -= (line_width + RING_GAP) * 2 * index;
    if (line_width)
    {
        lv_obj_set_style_arc_width(ring, line_width, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_arc_width(ring, line_width, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    }
    lv_obj_set_style_arc_color(ring, bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_color(ring, color, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ring, LV_OPA_TRANSP, LV_PART_KNOB | LV_STATE_DEFAULT);

    lv_arc_set_bg_angles(ring, 0, 360);
    lv_arc_set_value(ring, scale);
    lv_arc_set_rotation(ring, 90);          // Start from bottom.
    lv_obj_set_size(ring, width, width);
    lv_obj_align(ring, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE);

    if (ext->ring[index])
        lv_obj_del(ext->ring[index]);
    ext->ring[index] = ring;
    return ring;
}

/**
 * Create ARC in UI composite
 * @param comp the composite object
 * @param scale the arc in percentage,
 * @param color the color of the ring
 * @param bg_color the background color of the ring
 * @param index The ring index, 0  means the outter most.
 * @note radius is half width of comp, ring width and rounded is in theme,
    user could use lv_lvsfcomp_set_arc to update the ring scale.
 * @retval return the arc object created.
*/
lv_obj_t *lv_lvsfcomp_arc(lv_obj_t *comp, int scale, lv_color_t color, lv_color_t bg_color)
{
    lv_lvsfcomp_ext_t *ext = (lv_lvsfcomp_ext_t *)comp;
    int width = lv_obj_get_width(comp);
    int height = lv_obj_get_height(comp);
    int line_width;

    lv_obj_t *arc = lv_arc_create(comp);

    line_width = lv_obj_get_style_line_width(comp, LV_PART_MAIN);

    if (line_width)
    {
        lv_obj_set_style_arc_width(arc, line_width, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_arc_width(arc, line_width, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    }
    lv_obj_set_style_arc_color(arc, bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(arc, LV_RADIUS_CIRCLE, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_arc_set_bg_angles(arc, 140, 40);
    lv_arc_set_value(arc, scale);

    lv_obj_set_size(arc, width, width);
    lv_obj_set_style_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_update_layout(arc);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);

    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    if (ext->arc)
        lv_obj_del(ext->arc);
    ext->arc = arc;

    return arc;
}


/**
 * Update Ring scale in UI composite
 * @param comp the composite object
 * @param index The ring index, 0  means the outter most.
 * @param scale the ring in percentage,
*/
void lv_lvsfcomp_set_ring(lv_obj_t *comp, int index, int scale)
{
    lv_lvsfcomp_ext_t *ext = (lv_lvsfcomp_ext_t *)comp;

    LV_DEBUG_ASSERT(index < 3, "Index error", index);
    LV_DEBUG_ASSERT(ext->ring[index], "Ring is NULL error", ext->ring[index]);

    lv_arc_set_value(ext->ring[index], scale);
}

/**
 * Update Ring scale in UI composite
 * @param ring the ring part of composite object
 * @param scale the ring scale in percentage,
*/
void lv_lvsfcomp_set_arc(lv_obj_t *comp, int scale)
{
    lv_lvsfcomp_ext_t *ext = (lv_lvsfcomp_ext_t *)comp;

    LV_DEBUG_ASSERT(ext->arc, "ARC is NULL error", ext->arc);
    lv_arc_set_value(ext->arc, scale);
}

/**
 * Create image in UI composite
 * @param comp the composite object
 * @param img_src the image source
 * @retval return the image object created.
*/
lv_obj_t *lv_lvsfcomp_img(lv_obj_t *comp, const char *img_src)
{
    lv_obj_t *img_var = lv_img_create(comp);

    lv_img_set_src(img_var, img_src);
    lv_obj_align(img_var, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(img_var, LV_OBJ_FLAG_CLICKABLE);
    return img_var;
}

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
