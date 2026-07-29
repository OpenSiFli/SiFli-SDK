/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_LCD_PRIVATE_H__
#define __DRV_LCD_PRIVATE_H__

#include "drv_lcd.h"

#define LCD_DRV_AYSNC_WRITE  0x00000001
#define LCD_DRV_ACTIVED      0x00000002

#if defined(BSP_LCDC_USING_DIRECT_EPD)
    #define MAX_LCD_DRAW_TIME  (5000)  /*ms*/
#else
    #define MAX_LCD_DRAW_TIME  (500)  /*ms*/
#endif
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
    uint32_t start_tick;
    uint32_t draw_core_max;
    uint32_t draw_core_min;
    uint32_t draw_core_cnt;

    uint32_t draw_core_err_cnt;
} LCD_DRVStatistTypeDef;

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
    uint32_t statistics_log: 1; //Show statistical logs
    uint32_t skip_draw_core: 1;
    uint32_t reserved: 25;
    //Configuration union end//

    int8_t timeout_retry_cnt; /*Current lefted retry times,  -1 for retry infinitely, */

    uint32_t last_esd_check_tick;

    LCD_DrvRotateTypeDef rotate; /*Lcd rotate*/

    struct rt_thread task;
    rt_mq_t  mq;

    uint32_t debug_cnt1; /*'draw_rect_async' counts*/
    uint32_t debug_cnt2; /* HAL 'XferCpltCallback' cbk counts*/
    uint32_t debug_cnt3; /*'draw_rect_async' cbk counts*/


    LCD_DRVStatistTypeDef statistics;
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
    LCD_MSG_CONTROL,
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
    rt_device_t dev;
    int cmd;
    void *args;
} LCD_CtrlDef;

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
        LCD_CtrlDef ctrl_ctx;
    } content;


} LCD_MsgTypeDef;


HAL_LCDC_PixelFormat rt_lcd_format_to_hal_lcd_format(uint16_t rt_color_format);
rt_err_t rt_lcd_layer_control(LCD_DrvTypeDef *p_drv_lcd, int cmd, void *args);

#endif /* __DRV_LCD_PRIVATE_H__ */

