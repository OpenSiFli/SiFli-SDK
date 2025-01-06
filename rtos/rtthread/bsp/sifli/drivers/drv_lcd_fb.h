/**
  ******************************************************************************
  * @file   drv_lcd_fb.h
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

#ifndef __DRV_LCD_FB_H__
#define __DRV_LCD_FB_H__

#include "drv_lcd.h"


typedef struct
{
    LCD_AreaDef area; /*fb area, origin is LCD's TL*/
    uint8_t    *p_data;
    uint16_t   format; /*Like RTGRAPHIC_PIXEL_FORMAT_RGB565, RTGRAPHIC_PIXEL_FORMAT_RGB888*/
    uint8_t    cmpr_rate;
    uint32_t   line_bytes; /*Bytes of one line*/
} lcd_fb_desc_t;

typedef void (*write_fb_cbk)(lcd_fb_desc_t *fb_desc);

uint32_t drv_lcd_fb_init(const char *lcd_dev_name);
uint32_t drv_lcd_fb_deinit(void);
uint32_t drv_lcd_fb_set(lcd_fb_desc_t *fb_desc);
uint32_t drv_lcd_fb_is_busy(void);


//Copy src to fb and send
rt_err_t drv_lcd_fb_wait_write_done(int32_t wait_ms);
rt_err_t drv_lcd_fb_write_send(LCD_AreaDef *write_area, LCD_AreaDef *src_area, const uint8_t *src, write_fb_cbk cb, uint8_t send);

//Wrote to fb directly
rt_err_t drv_lcd_fb_get_write_area(LCD_AreaDef *write_area, int32_t wait_ms);
rt_err_t drv_lcd_fb_send(write_fb_cbk cb);

#endif





/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
