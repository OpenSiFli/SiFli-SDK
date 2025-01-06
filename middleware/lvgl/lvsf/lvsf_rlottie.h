/**
 * @file lvsf_rlottie.h
 *
 */

#ifndef LVSF_RLOTTIE_H
#define LVSF_RLOTTIE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/
extern const lv_obj_class_t lv_rlottie_class;

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

lv_obj_t *lv_rlottie_create(lv_obj_t *parent);

int lv_rlottie_file(lv_obj_t *lottie, const char *path);

int lv_rlottie_raw(lv_obj_t *lottie, const char *rlottie_desc);

int lv_rlottie_play(lv_obj_t *lottie, int enable);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_RLOTTIE_H*/


