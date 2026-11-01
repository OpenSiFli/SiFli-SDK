/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_ARCTEXT_H_
#define _LVSF_ARCTEXT_H_

#include "lvgl.h"
#include "lvsf_conf_internal.h"

#if LVSF_USING_ARCTEXT != 0

#ifdef __cplusplus
extern "C" {
#endif
/*********************
*      DEFINES
*********************/

/**********************
*      TYPEDEFS
**********************/
enum
{
    LV_ARCTEXT_ALIGN_CENTER = 0, /**< Align text to center */
    LV_ARCTEXT_ALIGN_LEFT, /**< Align text to left */
    LV_ARCTEXT_ALIGN_RIGHT, /**< Align text to right */
};
typedef uint8_t lv_arctext_align_t;
typedef struct
{
    lv_obj_t *img;
    lv_img_dsc_t dsc;
    int32_t angle;
    lv_coord_t pos_x;
    lv_coord_t offset_x;
    lv_coord_t offset_y;
    lv_coord_t pivot_x;
    lv_coord_t pivot_y;
    uint8_t     w;
} lv_arctext_item_t;

/*Data of the arc text object.*/
typedef struct
{
    lv_obj_t                base;
    lv_ll_t                 items;
    int16_t                 angle;
    uint16_t                w_equal;
    lv_arctext_align_t      align;
    int16_t                 txt_cnt;
    lv_coord_t              txt_w;
    uint8_t                 invalid_en;
    uint8_t                 snapshot_en;
    uint8_t                 mirror_en;
} lv_arctext_ext_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create an arc text object
 * @param parent    pointer to an object, it will be the parent of the new arctext.
 * @return          pointer to the created arctext
 */
lv_obj_t *lv_arctext_create(lv_obj_t *parent);


/*=====================
 * Setter functions
 *====================*/

/**
 * The specific display content of this arc-shaped text.
 * @param arctext   pointer to an arc-shaped text object
 * @param text      The string text content
*/
void lv_arctext_text(lv_obj_t *arctext, const char *text);

/**
 * Set the angle based alignment method for text alignment
 * @param arctext    pointer to an arc-shaped text object
 * @param align      Alignment type
 * @param angle      Alignment angle in degrees; 0 is at the top.
*/
void lv_arctext_set_align(lv_obj_t *arctext, lv_arctext_align_t align, int16_t angle);

/**
 * Set each text of the curved text to be displayed with equal spacing.
 * @param arctext    pointer to an arc-shaped text object
 * @param width      The width of each text
*/
void lv_arctext_set_equal_width(lv_obj_t *arctext, lv_coord_t width);

/**
 * Select snapshot rendering or direct glyph rendering.
 * @param arctext   Pointer to an arc text object.
 * @param en        Nonzero enables snapshots; zero draws glyph bitmaps directly.
 * @note Set the text again after changing this mode.
*/
void lv_arctext_set_snapshot(lv_obj_t *arctext, uint8_t en);

/**
 * Arrange the text in a counterclockwise or clockwise direction
 * @param arctext   pointer to an arc-shaped text object
 * @param en      1:counterclockwise, 0:clockwise direction
*/
void lv_arctext_set_mirror(lv_obj_t *arctext, uint8_t en);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LVSF_USING_ARCTEXT */
#endif /* _LVSF_ARCTEXT_H_ */
