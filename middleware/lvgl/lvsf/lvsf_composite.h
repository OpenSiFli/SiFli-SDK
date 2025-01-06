/**
  ******************************************************************************
  * @file   lvsf_composite.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
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

#ifndef LVSF_COMPOSITE_H
#define LVSF_COMPOSITE_H

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup lvsf_composite Sifli LVGL watch composite
  * @ingroup middleware
  * @{
  */

/*********************
 *      DEFINES
 *********************/
/*********************
 *      DEFINES
 *********************/
#define CIRCULAR_SIZE               (LV_HOR_RES_MAX / 6)
#define CIRCULAR_LINE_WIDTH         (CIRCULAR_SIZE / 8)

#define MODULAR_SMALL_SIZE          (LV_HOR_RES_MAX / 5)
#define MODULAR_SMALL_LINE_WIDTH    (MODULAR_SMALL_SIZE / 8)

#define MODULAR_LARGE_SIZE          (LV_HOR_RES_MAX * 5 / 2)
#define MODULAR_LARGE_LINE_WIDTH    -1         // Do not have ring

#define EXTRA_LARGE_SIZE            (LV_HOR_RES_MAX / 2)
#define EXTRA_LARGE_LINE_WIDTH      (EXTRA_LARGE_SIZE / 8)

#define UTILITY_SIZE                LV_HOR_RES_MAX/8
#define UTILITY_LINE_WIDTH          LV_HOR_RES_MAX/80


#define GRA_CORNER_SIZE             LV_HOR_RES_MAX/3

#define GRA_CORNER_LINE_WIDTH       LV_HOR_RES_MAX/60
#define GRA_CORNER_SMALL_SIZE       LV_HOR_RES_MAX/12

#define GRA_CIRCULAR_SIZE           LV_HOR_RES_MAX/3
#define GRA_CIRCULAR_LINE_WIDTH     LV_HOR_RES_MAX/30
#define GRA_CIRCULAR_SMALL_SIZE     LV_HOR_RES_MAX/8

#define TEXT_ANGLE  4
#define RING_GAP    2

/**********************
 *      TYPEDEFS
 *********************/
/**
    @brief See Apple watch Complications for example of lvsf composite
*/
typedef enum
{
    LVSF_COMP_CIRCLE = 0,
    LVSF_COMP_MOD_SMALL,
    LVSF_COMP_MOD_LARGE,
    LVSF_COMP_EXTRA_LARGE,
    LVSF_COMP_UTILITY_SMALL,
    //LVSF_COMP_UTILITY_LARGE,
    LVSF_COMP_CIRCULAR,
    LVSF_COMP_CORNER,
    //LVSF_COMP_BEZEL,
    //LVSF_COMP_RECTANGULAR,
    LVSF_COMP_MAX
} lvsf_comp_type;


/**
 * Make the lvsfcomp's class publicly available.
 */
extern const lv_obj_class_t lv_lvsfcomp_class;
extern const lv_obj_class_t lv_lvsfcorner_class;


/**
 * Create UI composite
 * @param parent parent of the composite object
 * @retval return the composite object created.
*/
lv_obj_t *lv_lvsfcomp_create(lv_obj_t *parent);

/**
 * Set UI composite type
 * @param comp the composite object
 * @param type the type of composite object, see \ref lvsf_comp_type
 * @retval return the composite object created.
*/
void lv_lvsfcomp_set_type(lv_obj_t *comp, int type);

/**
 * Create text in UI composite
 * @param comp the composite object
 * @param text text content
 * @note if color are in the text(use recolor), font is set in themes.
 * @retval return the label object created.
*/
lv_obj_t *lv_lvsfcomp_text(lv_obj_t *comp, const char *text);


/**
 * Create Ring in UI composite
 * @param comp the composite object
 * @param index The ring index, 0  means the outter most.
 * @param scale the ring in percentage,
 * @param color the color of the ring
 * @param bg_color the background color of the ring
*/
lv_obj_t *lv_lvsfcomp_ring(lv_obj_t *comp, int index, int scale, lv_color_t color, lv_color_t bg_color);

/**
 * Create ARC in UI composite
 * @param comp the composite object
 * @param scale the arc in percentage,
 * @param color the color of the ring
 * @param bg_color the background color of the ring
*/
lv_obj_t *lv_lvsfcomp_arc(lv_obj_t *comp, int scale, lv_color_t color, lv_color_t bg_color);

/**
 * Update Ring scale in UI composite
 * @param comp the composite object
 * @param index The ring index, 0  means the outter most.
 * @param scale the ring in percentage,
*/
void lv_lvsfcomp_set_ring(lv_obj_t *comp, int index, int scale);


/**
 * Update ARC scale in UI composite
 * @param comp the composite object
 * @param scale the arc in percentage,
*/
void lv_lvsfcomp_set_arc(lv_obj_t *comp, int scale);


/**
 * Create image in UI composite
 * @param comp the composite object
 * @param img_src the image source
 * @retval return the image object created.
*/
lv_obj_t *lv_lvsfcomp_img(lv_obj_t *comp, const char *img_src);


/** @addtogroup lvsf_corner  Watch corner composite
  * @ingroup lvsf_composite
  * @{
  */

/**
 * Create watch corner composite
 * @param parent the parent object
 * @retval corner object.
*/
lv_obj_t *lv_lvsfcorner_create(lv_obj_t *parent);

/**
 * Set corner zone
 * @param corner the corner object
 * @param zone zone of quarter.
 * @param r radius of circle that corner belong to.
 * @param x center x of circle that corner belong to.
 * @param y center y of circle that corner belong to.
 * @retval corner object.
*/
void lv_lvsfcorner_zone(lv_obj_t *corner, uint16_t zone, uint16_t r, uint16_t x, uint16_t y);


/**
 * Set curve text for the corner.
 * @param[in] corner the corner object
 * @param[in] text content of curved test.
 * @retval corner object.
*/
lv_obj_t *lv_lvsfcorner_curve_text(lv_obj_t *corner, char *text);

/**
 * Set image for the corner.
 * @param[in] corner the corner object
 * @param[in] src image source.
 * @retval corner object.
*/
lv_obj_t *lv_lvsfcorner_img(lv_obj_t *corner, const char *src);

/**
 * Set text for the corner.
 * @param[in] corner the corner object
 * @param[in] text The text in the corner.
 * @retval corner object.
*/
lv_obj_t *lv_lvsfcorner_text(lv_obj_t *corner,  char *text);

/**
 * Set arc ring for the corner.
 * @param[in] corner the corner object
 * @param[in] start start angle of the ring.
 * @param[in] end end angle of the ring.
 * @param[in] color color of the ring.
 * @retval corner object.
*/
lv_obj_t *lv_lvsfcorner_arc(lv_obj_t *corner, int start, int end, lv_color_t color);

/**
 * Set arc scale value.
 * @param[in] corner the corner object
 * @param[in] percent percentage of arc value..
 * @retval none.
*/
void lv_lvsfcorner_arc_scale(lv_obj_t *corner, int percent);



#ifdef __cplusplus
} /* extern "C" */
#endif

/// @}  lvsf_corner
/// @}  lvsf_composite

#endif /*LVSF_COMPOSITE_H*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
