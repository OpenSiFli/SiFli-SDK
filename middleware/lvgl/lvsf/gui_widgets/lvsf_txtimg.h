/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lvsf_txtimg.h
 *
 */

#ifndef LVSF_TXTIMG_H
#define LVSF_TXTIMG_H

#include "lvsf_conf_internal.h"

#if LVSF_USE_TXTIMG != 0

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
 *      TYPEDEFS
 **********************/

enum
{
    LV_TXTIMG_ANIM_BUF = (1 << 0),
    LV_TXTIMG_HOR_ANIM = (1 << 1),
    LV_TXTIMG_RESIDENCY = (1 << 2),
};

typedef uint32_t lv_txtimg_flg_t;

typedef struct
{
    lv_img_dsc_t dsc;
    uint8_t *img_data;
    uint32_t img_size;
    char *txt;
    lv_coord_t w;
    lv_coord_t h;
    lv_coord_t x;
    lv_coord_t y;
} lv_txtimg_line_t;

#define TXTIMG_LINE_LEN 1024

typedef struct
{
    lv_obj_t base;
    lv_ll_t lines;
    lv_timer_t *timer;
    lv_coord_t txt_w;
    lv_coord_t line_w;
    lv_coord_t zoom;
    lv_coord_t w;
    lv_coord_t offset_x;
    lv_txtimg_flg_t flg;
} lv_txtimg_t;

extern const lv_obj_class_t lv_txtimg_class;


/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a txtimg object
 */
lv_obj_t *lv_txtimg_create(lv_obj_t *parent);

/**
 * Keep the previous text and insert the latest str.
 * @param txtimg  The instance for txtimg.
 * @param text the string to insert.
 * @return
 */
void lv_txtimg_set_txt(lv_obj_t *txtimg, const char *text);

/**
 * Replace the original string text,Clear the previous str and keep only the current str
 * @param txtimg  The instance for txtimg.
 * @param text the string to set.
 * @return
 */
void lv_txtimg_ins_txt(lv_obj_t *txtimg, const char *text);

/**
 * add one line text,
 * @param txtimg  The instance for txtimg.
 * @param text the string to set.
 * @return the line len
 */
int32_t lv_txtimg_set_txt_line(lv_obj_t *txtimg, const char *text);

/**
 * For some languages that do not support converting bitmap to A8 format, a camera interface needs to be used to take a picture of the text and position it as an image
 * @param txtimg  The instance for txtimg.
 * @param text the string to set.
 * @return the line len
 */
int32_t lv_txtimg_snapshot_txt_line(lv_obj_t *txtimg, const char *text);
/**
 * set the zoom of txtimg
 * @param txtimg  The instance for txtimg.
 * @param zoom   set txtimg zoom.
 * @return
 */
void lv_txtimg_set_zoom(lv_obj_t *txtimg, lv_coord_t zoom);

/**
 * refresh set the txtimg of size to txt area.
 * @param txtimg  The instance for txtimg.
 * @return
 */
void lv_txtimg_refr_size(lv_obj_t *txtimg);

/**
 * refresh set the txtimg of size to txt area.
 * @param txtimg  The instance for txtimg.
 * @return
 */
void lv_txtimg_set_flg(lv_obj_t *txtimg, uint32_t flg);

uint32_t lv_txtimg_get_flg(lv_obj_t *txtimg);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LVSF_USE_TXTIMG */

#endif /* LVSF_TXTIMG_H */
