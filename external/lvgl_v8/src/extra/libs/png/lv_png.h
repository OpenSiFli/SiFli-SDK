/**
 * @file lv_png.h
 *
 */

#ifndef LV_PNG_H
#define LV_PNG_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../../../lv_conf_internal.h"
#if LV_USE_PNG

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Register the PNG decoder functions in LVGL
 */
void lv_png_init(void);
lv_res_t lvsf_png_decoder_info(const void *src, lv_img_header_t *header);
lv_res_t lvsf_png_decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc);
void lvsf_png_decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_PNG*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_PNG_H*/
