/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
void drv_lcd_fb_write_send_parallel(uint8_t enable);

//Copy src to fb and send
rt_err_t drv_lcd_fb_wait_write_done(int32_t wait_ms);
rt_err_t drv_lcd_fb_write_send(LCD_AreaDef *write_area, LCD_AreaDef *src_area, const uint8_t *src, write_fb_cbk cb, uint8_t send);

//Wrote to fb directly
rt_err_t drv_lcd_fb_get_write_area(LCD_AreaDef *write_area, int32_t wait_ms);
rt_err_t drv_lcd_fb_wait_all_done(uint8_t *p_data, int32_t wait_ms);
rt_err_t drv_lcd_fb_send(write_fb_cbk cb);

#endif

