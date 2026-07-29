/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lv_bar.h
 *
 */

#ifndef LVSF_MULTSLIDER_H
#define LVSF_MULTSLIDER_H

#if LVSF_USE_MULTSLIDER != 0

#ifdef __cplusplus
extern "C" {
#endif

/*********************
*      INCLUDES
*********************/

/*********************
*      DEFINES
*********************/

/**********************
*      TYPEDEFS
**********************/

typedef struct
{
    lv_obj_t base;
    char *txt;
    int32_t cur_value;          /**< Current value of the multslider*/
    int32_t min_value;          /**< Minimum value of the multslider*/
    int32_t max_value;          /**< Maximum value of the multslider*/
    uint32_t txt_en : 1;
} lv_multslider_t;

extern const lv_obj_class_t lv_multslider_class;


/**********************
* GLOBAL PROTOTYPES
**********************/

/**
* Create a multslider object
*/
lv_obj_t *lv_multslider_create(lv_obj_t *parent);

void lv_multslider_set_txt(lv_obj_t *multslider, const char *txt);

void lv_multslider_set_value(lv_obj_t *multslider, int32_t value, lv_anim_enable_t anim);

void lv_multslider_set_range(lv_obj_t *v, int32_t min, int32_t max);

int32_t lv_multslider_get_max_value(lv_obj_t *multslider);

int32_t lv_multslider_get_min_value(lv_obj_t *multslider);

int32_t lv_multslider_get_value(lv_obj_t *multslider);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif

#endif
