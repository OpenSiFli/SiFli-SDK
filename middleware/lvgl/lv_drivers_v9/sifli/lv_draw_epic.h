/**
  ******************************************************************************
  * @file   lv_draw_epic.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
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


#ifndef LV_DRAW_EPIC_H
#define LV_DRAW_EPIC_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "../../lv_conf_internal.h"

#if LV_USE_DRAW_EPIC
#include "lv_draw_sw.h"

#include "drv_epic.h"
/*********************
 *      DEFINES
 *********************/
#define GPU_BLEND_EXP_MS     100
#define LV_EPIC_LOG     LV_LOG_INFO
/**********************
 *      TYPEDEFS
 **********************/

typedef lv_layer_t lv_epic_layer_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lv_draw_buf_epic_init_handlers(void);

void lv_draw_epic_init(void);

void lv_draw_epic_deinit(void);

void lv_draw_epic_fill(lv_draw_unit_t *draw_unit, const lv_draw_fill_dsc_t *dsc,
                       const lv_area_t *coords);

void lv_draw_epic_img(lv_draw_unit_t *draw_unit, const lv_draw_image_dsc_t *dsc,
                      const lv_area_t *coords);

void lv_draw_epic_layer(lv_draw_unit_t *draw_unit, const lv_draw_image_dsc_t *draw_dsc,
                        const lv_area_t *coords);

void lv_draw_epic_label(lv_draw_unit_t *draw_unit, const lv_draw_label_dsc_t *dsc,
                        const lv_area_t *coords);

void lv_draw_epic_border(lv_draw_unit_t *draw_unit, const lv_draw_border_dsc_t *dsc,
                         const lv_area_t *coords);
void lv_draw_epic_arc(lv_draw_unit_t *draw_unit, const lv_draw_arc_dsc_t *dsc, const lv_area_t *coords);
/**********************
 *      MACROS
 **********************/
#endif /*LV_USE_DRAW_EPIC*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRAW_EPIC_H*/
