/**
  ******************************************************************************
  * @file   bf0_lib.h
  * @author Sifli software development team
  * @brief   This file contains all the functions prototypes for the Sifli library
  * @{
  ******************************************************************************
*/
/*
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

#ifndef __BF0_LIB_H
#define __BF0_LIB_H
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

int get_icon_transform_param(float x, float y, float icon_r, float *p_float_x, float *p_float_y, float *p_float_icon_r, float *p_float_pivot_r, float scr_width, float scr_height);


/*Gauss interface*/
typedef struct
{
    uint8_t *data;            /**< data, color format is specified by #color_mode,
                                   point to the pixel data corresponding to pixel(#x_offset,#y_offset) */
    uint32_t color_mode;      /**< color mode, refer to #EPIC_COLOR_RGB565 */
    uint16_t width;           /**< layer width to be processed */
    uint16_t height;          /**< layer heigth to be processed  */
} BlurDataType;

typedef void (*gauss_cbk)(void);

/**
* @brief  Initialize Gauss module
* @param  in Input data, image to be blured.
* @param  out Output data, image blured
* @param  radius blur kernal size, typical 60.
* @retval  RT_EOK if success, otherwise failed.
*/
void *gauss_init(BlurDataType *in, BlurDataType *out, uint16_t radius);

/**
 * @brief  De-Initialize Gauss module
 * @retval  RT_EOK if success, otherwise failed.
*/
int gauss_deinit(void *gauss);

/**
 * @brief  Trigger Gauss calculation
 * @param   cbk, callback function called when finish.
 * @retval  RT_EOK if success, otherwise failed.
*/
int gauss_start_IT(void *gauss, gauss_cbk cbk);



#ifdef __cplusplus
}
#endif

#endif /* __BF0_CP_HAL_H */



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
