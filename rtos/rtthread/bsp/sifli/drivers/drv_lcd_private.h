/**
  ******************************************************************************
  * @file   drv_lcd_private.h
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

#ifndef __DRV_LCD_PRIVATE_H__
#define __DRV_LCD_PRIVATE_H__

#include "drv_lcd.h"

#define LCD_DRV_AYSNC_WRITE  0x00000001
#define LCD_DRV_ACTIVED      0x00000002


#define MAX_LCD_DRAW_TIME  (500)  /*ms*/
#define LCD_ENTER_LP_INTERVAL  (1)  /*ms*/

#define MIN_BRIGHTNESS_LEVEL 0
#define MAX_BRIGHTNESS_LEVEL 100

#define MAX_TIMEOUT_RETRY 3  /*Maximum timeout retry times,  -1 for infinity, */

#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
    #define MAX(x,y) (((x)>(y))?(x):(y))
#endif

typedef struct
{
    struct rt_device parent;
    LCDC_HandleTypeDef hlcdc;
    lcd_drv_desc_t *p_drv_ops;
    uint16_t buf_format;
    HAL_LCDC_LayerDef select_layer;

    struct rt_semaphore  sem; /*API lock semaphore*/
    struct rt_semaphore  draw_sem; /*Async draw semaphore*/
    struct rt_semaphore  sync_msg_sem; /*Sync msg semaphore*/
    LCD_DrvStatusTypeDef status;
    uint8_t brightness;

    uint32_t start_tick;       /*Last aysnc send start tick*/
    uint32_t end_tick;         /*Last aysnc send end tick*/
    uint8_t draw_lock;      /*Draw lock*/
    uint8_t draw_error;   /*Draw core failed flag*/

    //Configuration union start//
    uint32_t auto_lowpower : 1;  /*Auto enter low power mode*/
    uint32_t force_lcd_missing : 1;      /*Force LCD missing*/
    /*draw timeout :
        0 - Reset LCD and try to restart it
        1 - Raise an assertion
        2 - Delete LCD driver and nothing else.
    */
    uint32_t assert_timeout : 2;
    uint32_t send_time_log: 1;
    uint32_t reserved: 27;
    //Configuration union end//

    int8_t timeout_retry_cnt; /*Current lefted retry times,  -1 for retry infinitely, */

    uint32_t last_esd_check_tick;

    LCD_DrvRotateTypeDef rotate; /*Lcd rotate*/

    struct rt_thread task;
    rt_mq_t  mq;

    uint32_t debug_cnt1; /*'draw_rect_async' counts*/
    uint32_t debug_cnt2; /* HAL 'XferCpltCallback' cbk counts*/
    uint32_t debug_cnt3; /*'draw_rect_async' cbk counts*/
} LCD_DrvTypeDef;


typedef enum
{
    LCD_MSG_INVALID,
    /*Asynchronized executed MSG*/
    __LCD_ASYNCHRONIZED_MSG_START,

    LCD_MSG_OPEN,
    LCD_MSG_POWER_ON,
    LCD_MSG_DRAW_RECT_ASYNC,
    LCD_MSG_DRAW_COMP_RECT_ASYNC,
    LCD_MSG_SET_NEXT_TE,
    LCD_MSG_GET_BRIGHTNESS_ASYNC,
    LCD_MSG_FLUSH_RECT_ASYNC,
    __LCD_ASYNCHRONIZED_MSG_END,


    /*Synchronized executed MSG*/
    __LCD_SYNCHRONIZED_MSG_START,
    LCD_MSG_CLOSE,
    LCD_MSG_POWER_OFF,
    LCD_MSG_SET_MODE,
    LCD_MSG_DRAW_RECT,
    LCD_MSG_SET_WINDOW,
    LCD_MSG_SET_PIXEL,
    LCD_MSG_GET_PIXEL,
    LCD_MSG_SET_BRIGHTNESS,
    LCD_MSG_CTRL_SET_LCD_PRESENT,
    LCD_MSG_CTRL_ASSERT_IF_DRAWTIMEOUT,
    __LCD_SYNCHRONIZED_MSG_END,
} LCD_MsgIdDef;
#define IS_SYNC_MSG_ID(msg_id) (((msg_id)>__LCD_SYNCHRONIZED_MSG_START)&&((msg_id) < __LCD_SYNCHRONIZED_MSG_END))


typedef struct
{
    const uint8_t *pixels;
    LCD_AreaDef area;
} LCD_DrawCtxDef;

typedef struct
{
    const uint8_t *data;
    int16_t x;
    int16_t y;
} LCD_PixelDef;


typedef struct
{
    LCD_DrvTypeDef *driver;
    LCD_MsgIdDef id;
    uint32_t tick;

    union
    {
        uint8_t brightness;
        uint8_t TE_on;
        LCD_DrawCtxDef draw_ctx;
        LCD_AreaDef    window;
        LCD_PixelDef pixel;
        void *compress_buf;
        uint8_t *p_brightness_ret;
        lcd_flush_info_t flush;
        uint8_t idle_mode_on;
        uint8_t is_lcd_present;
        uint8_t assert_timeout;
    } content;


} LCD_MsgTypeDef;


HAL_LCDC_PixelFormat rt_lcd_format_to_hal_lcd_format(uint16_t rt_color_format);
rt_err_t rt_lcd_layer_control(LCD_DrvTypeDef *p_drv_lcd, int cmd, void *args);

#endif /* __DRV_LCD_PRIVATE_H__ */

