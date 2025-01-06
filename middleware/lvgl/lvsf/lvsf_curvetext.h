#ifndef _LVSF_CURVETEXT_H
#define _LVSF_CURVETEXT_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if LVSF_USE_CURVE != 0

/*Testing of dependencies*/
#if LV_USE_CANVAS == 0
#error "lv_lvsfcurve: lv_canvas is required. Enable it in lv_conf.h (LV_USE_CANVAS  1) "
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

lv_obj_t *lv_lvsfcurve_create(lv_obj_t *parent);

void lv_lvsfcurve_set_buf(lv_obj_t *curve, uint16_t txt_width, uint16_t txt_height);

void lv_lvsfcurve_set_pivot(lv_obj_t *curve, lv_coord_t x, lv_coord_t y);

void lv_lvsfcurve_draw_arc(lv_obj_t *curve, lv_coord_t r, int32_t start_angle, int32_t end_angle, lv_color_t color, lv_coord_t width);

void lv_lvsfcurve_text(lv_obj_t *curve, char *text, int angle, int r, lv_color_t color, int size);

#endif /*LVSF_USE_CURVE*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*_LVSF_CURVETEXT_H*/
