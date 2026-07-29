/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_MULTROLLER_H_
#define _LVSF_MULTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "lvsf_multlist.h"

#if 1//LVSF_USING_MULTROLLER != 0
/*********************
 *      DEFINES
 *********************/

/**********************
*      TYPEDEFS
**********************/
typedef struct
{
    lv_multlist_ext_t       list;
    uint32_t                show_cnt;
    char                   *options;
    lv_color_t              color;
    uint16_t                focus_w;
    uint16_t                focus_h;
    uint8_t                 focus_en;
} lv_multroller_ext_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a roller object
 * @param parent    pointer to an object, it will be the parent of the new roller.
 * @return          pointer to the created roller
 */
lv_obj_t *lv_multroller_create(lv_obj_t *parent);

/*=====================
 * Setter functions
 *====================*/

/**
 * Set the number of displayed sub-elements.
   The size of the child elements is determined by dividing each child element equally
   based on the size of the parent control.Default num 3.
 * @param multroller       pointer to a roller object
 * @param cnt   number of desired visible cnt
 */
void lv_multroller_set_show_cnt(lv_obj_t *multroller, uint8_t cnt);

/**
 * Set the options on a roller
 * @param multroller pointer to roller object
 * @param options   a string with '\n' separated options. E.g. "One\nTwo\nThree"
 * @param mode      `LV_ROLLER_MODE_NORMAL` or `LV_ROLLER_MODE_INFINITE`
 */
void lv_multroller_set_options(lv_obj_t *multroller, const char *options);

/**
 * Set the selected option
 * @param multroller       pointer to a roller object
 * @param sel_opt   index of the selected option (0 ... number of option - 1);
 * @param anim_time 0: set immediately. other set the animation timer;
 */
void lv_multroller_set_selected(lv_obj_t *multroller, uint16_t sel_opt, uint32_t anim_time);

/**
 * Set the focus area size and another color for the text
 * @param multroller pointer to a roller object
 * @param color the color of txt in focus area;
 * @param w the width of the the focus area;
 * @param h the height of the focus area;
 */
void lv_multroller_set_focus_param(lv_obj_t *multroller, lv_color_t color, uint16_t w, uint16_t h);

/*=====================
 * Getter functions
 *====================*/

/**
 * Get the index of the selected option
 * @param multroller   pointer to a roller object
 * @return             index of the selected option (0 ... number of option - 1);
 */
uint16_t lv_multroller_get_selected(lv_obj_t *multroller);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
#endif