/**
  ******************************************************************************
  * @file   lvsf_comp.h
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

#ifndef LVSF_COMP_H
#define LVSF_COMP_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup lvsf_comp  Sifli LVGL extension
  * @ingroup lvsf_composite
  * @{
  */

/**
 * Create Text - Advance version
 * @param parent parent of the text object
 * @param text the content of the text
 * @param x x-coordinate of the text starting point
 * @param y y-coordinate of the text starting point
 * @param angle the turning angle of the text
 * @param color the color of the text
 * @param size the size of the text, from 0(size 8) to 3 (size 22)
 * @retval return the parent object, this function ONLY draw text on parent canvas,
            and do NOT create new object for text.
*/
lv_obj_t *lvsf_text_create2(lv_obj_t *parent, char *text, int x, int y, int angle, lv_color_t color, int size);

/**
 * Create curved text
 * @param parent parent of the text object
 * @param text the content of the text
 * @param x x-coordinate of the curve central point
 * @param y y-coordinate of the curve central point
 * @param angle the starting angle of the text
 * @param r radius of the the curve
 * @param color the color of the text
 * @param size the size of the text, from 0(size 8) to 3 (size 22)
 * @retval return the text object created.
*/
lv_obj_t *lvsf_curve_text(lv_obj_t *parent, char *text, int x, int y, int angle, int r, lv_color_t color, int size);

/**
 * Draw curved text, help function of lvsf_curve_text
 * @param parent parent of the text object
 * @param text the content of the text
 * @param corner_startx x-coordinate of the parent x start
 * @param corner_starty y-coordinate of the parent y start
 * @param angle the starting angle of the text
 * @param r radius of the the curve
 * @param color the color of the text
 * @param size the size of the text, from 0(size 8) to 3 (size 22)
*/
void lvsf_curve_draw_text(lv_obj_t *parent, char *text, int corner_startx, int corner_starty, int angle, int r, lv_color_t color, int size);

/**
 * Get font width
 * @param font_size the size of font
 * @param c the character of a font.
 * @retval return the width of the character.
*/
int lvsf_font_width(int font_size, char c);

/**
 * Get font height
 * @param font_size the size of font
 * @retval return the height of the character.
*/
int lvsf_font_height(int font_size);

/**
 * Rotate the canvas
 * @param[in] canvas The vanvas to rotate
 * @param[in] img image to be rotated
 * @param[in] angle angle to rotate
 * @param[in] zoom  the scale for the image to zoom out.
 * @param[in] offset_x x offset of the image after rotation.
 * @param[in] offset_y y offset of the image after rotation.
 * @param[in] pivot_x x pivot of the image after rotation.
 * @param[in] pivot_y x pivot  of the image after rotation.
*/
void lvsf_canvas_rotate(lv_obj_t *canvas, lv_img_dsc_t *img, int16_t angle, int32_t zoom, lv_coord_t offset_x, lv_coord_t offset_y,
                        int32_t pivot_x, int32_t pivot_y);


/// @}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_IMGBTN_H*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
