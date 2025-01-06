/**
 * @file lv_gif.h
 *
 */

#ifndef LV_GIF_H
#define LV_GIF_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "lvgl.h"
#if LV_USE_GIF

#include "gifdec.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
extern const lv_obj_class_t lv_gif_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

lv_obj_t * lv_gif_create(lv_obj_t * parent);
void lv_gif_set_src(lv_obj_t * obj, const void * src);
void lv_gif_restart(lv_obj_t * obj);
void lv_gif_pause(lv_obj_t * obj);
void lv_gif_resume(lv_obj_t * obj);



/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_GIF*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_GIF_H*/
