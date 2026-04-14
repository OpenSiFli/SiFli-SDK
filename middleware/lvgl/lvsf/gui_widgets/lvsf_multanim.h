/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 ******************************************************************************
 * @file   LVSF_MULTANIM.h
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
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

#ifndef _LVSF_MULTANIM_H
#define _LVSF_MULTANIM_H

#include "lvsf_conf_internal.h"

#if LVSF_USE_MULTANIM != 0

#ifdef __cplusplus
extern "C" {
#endif

/*********************
*      INCLUDES
*********************/
#include "lvgl.h"

/*********************
*      DEFINES
*********************/

/**********************
*      MACROS
**********************/


/**********************
* GLOBAL PROTOTYPES
**********************/
enum
{
    LV_MULTANIM_LEFT    = (1U << 0),
    LV_MULTANIM_RIGHT   = (1U << 1),
    LV_MULTANIM_TOP     = (1U << 3),
    LV_MULTANIM_BOTTOM  = (1U << 4),
};
typedef uint8_t lv_multanim_dir;
#define LV_MULTANIM_HOR (LV_MULTANIM_LEFT | LV_MULTANIM_RIGHT)
#define LV_MULTANIM_VER (LV_MULTANIM_TOP | LV_MULTANIM_BOTTOM)
#define LV_MULTANIM_NEG (LV_MULTANIM_LEFT | LV_MULTANIM_TOP)
#define LV_MULTANIM_POS (LV_MULTANIM_RIGHT | LV_MULTANIM_BOTTOM)

enum
{
    LV_MULTANIM_NONE,
    LV_MULTANIM_ZOOM,
    LV_MULTANIM_3D,
    LV_MULTANIM_SWITCH,
    LV_MULTANIM_TURN,
    LV_MULTANIM_SCALE,
    LV_MULTANIM_FADE,
    LV_MULTANIM_OPEN,
    LV_MULTANIM_ROLL,
    LV_MULTANIM_DRAG,
    LV_MULTANIM_BOOK,
    LV_MULTANIM_SHUTTLE,
    LV_MULTANIM_SHUTTER,
    LV_MULTANIM_MAX,
};
typedef uint16_t lv_multanim_type;

typedef void(*lv_multanim_ready_cb)(lv_obj_t *);

typedef struct
{
    lv_obj_t                obj;
    lv_obj_t                *major_img;
    lv_obj_t                *minor_img;
    const lv_img_dsc_t      *mask_l;
    const lv_img_dsc_t      *mask_r;
    lv_coord_t              *offset_arr;
    uint8_t                 *third_buf;
    lv_img_dsc_t            dsc;
    int32_t                 process;
    int32_t                 range;
    lv_point_t              view_start;
    lv_point_t              view_end;
    lv_coord_t              zoom_start;
    lv_coord_t              zoom_end;
    lv_multanim_type        type;
    lv_multanim_dir         dir;
} lv_multanim_ext_t;

extern const lv_obj_class_t lv_multanim_class;

/**
* Create an multanim instance
* @param parent pointer to an object, it will be the parent of the new page
* @return pointer to the created list
*/
lv_obj_t *lv_multanim_create(lv_obj_t *parent);

/**
* set the type of animation,must be set before start animation
* @param multanim the instance of multanim
* @param type the type of multanim
* @return the previous animation type
*/
lv_multanim_type lv_multanim_set_type(lv_obj_t *multanim, lv_multanim_type type);

/**
* set the dir of animation
* @param multanim the instance of multanim
* @param dir the dir of multanim
* @return void
*/
void lv_multanim_set_dir(lv_obj_t *multanim, lv_multanim_dir dir);

/**
* set the major of image for multanim, must be set.
* @param multanim the instance of multanim
* @param major_img the major image
* @return void
*/
void lv_multanim_set_major_img(lv_obj_t *multanim, lv_obj_t *major_img);

/**
* set the minor of image for multanim.
* @param multanim the instance of multanim
* @param minor_img the minor image
* @return void
*/
void lv_multanim_set_minor_img(lv_obj_t *multanim, lv_obj_t *minor_img);

/**
* set the range.
* @param multanim the instance of multanim
* @param range the range of animation
* @return void
*/
void lv_multanim_set_range(lv_obj_t *multanim, int32_t range);

/**
* set the viewpoint.
* @param multanim the instance of multanim
* @param start_v the start point of viewpoint when start animation
* @param end_v the end point of viewpoint when start animation
* @return void
*/
void lv_multanim_set_viewpoint(lv_obj_t *multanim, lv_point_t *start_v, lv_point_t *end_v);

/**
* set the zoom.
* @param multanim the instance of multanim
* @param start_zoom the zoom of image when start animation
* @param end_zoom the zoom of image when start animation
* @return void
*/
void lv_multanim_set_zoom(lv_obj_t *multanim, lv_coord_t start_zoom, lv_coord_t zoom_end);

/**
* set the process,the main animation interface
* @param multanim the instance of multanim
* @param process Progress value of animation[0,1024]
* @return void
*/
void lv_multanim_set_process(lv_obj_t *multanim, int32_t process);

/**
* set mask for some mask animation
* @param multanim the instance of multanim
* @param mask_l the mask gradually disappearing on the left
* @param mask_r the mask gradually disappearing on the right
* @return void
*/
void lv_multanim_set_mask(lv_obj_t *multanim, const lv_img_dsc_t *mask_l, const lv_img_dsc_t *mask_r);

/**
* get the progress of multanim
* @param multanim the instance of multanim
* @return the progress
*/
int32_t lv_multanim_get_proc(lv_obj_t *multanim);

/**
* get the major image
* @param multanim the instance of multanim
* @return the minor image
*/
lv_obj_t *lv_multanim_get_major_img(lv_obj_t *multanim);

/**
* get the minor image
* @param multanim the instance of multanim
* @return the minor image
*/
lv_obj_t *lv_multanim_get_minor_img(lv_obj_t *multanim);

/**
* Read the dsc of the mask according to the specified path
* @param src the path of mask
* @return the dsc of mask
*/
lv_img_dsc_t *lv_multanim_create_mask(const void *src);

/**
* free the dsc of mask
* @param dsc the dsc of mask created by lv_multanim_create_mask
* @return void
*/
void lv_multanim_free_mask(lv_img_dsc_t *dsc);

/**
* get the type of multanim
* @param multanim the instance of multanim
* @return the type of multanim
*/
lv_multanim_type lv_multanim_get_type(lv_obj_t *multanim);

/**
* start animation
* @param multanim the instance of multanim
* @param period the duration of animation in milliseconds
* @param start_pro the start progress value of animation[0,1024]
* @param end_pro the end progress value of animation[0,1024]
* @param ready_cb callback function when animation is ready
* @return void
*/
void lv_multanim_start_anim(lv_obj_t *multanim, uint32_t period, int32_t start_pro, int32_t end_pro, lv_anim_ready_cb_t ready_cb);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif// LVSF_USE_MULTANIM

#endif
