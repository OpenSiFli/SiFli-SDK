/**
  ******************************************************************************
  * @file   drv_ram_lcd.c
  * @author Sifli software development team
  * @brief  Enables the Display.
  *
* *****************************************************************************
**/
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

#include "drv_lcd.h"
#include "drv_io.h"
#include "string.h"

#include "drv_lcd_private.h"
#ifdef APP_BSP_TEST

#undef  LOG_TAG
#define LOG_TAG                "drv.ramlcd"

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   rt_kprintf(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif


#define RAM_LCD_WIDTH  120
#define RAM_LCD_HEIGHT 120
#define RAM_LCD_FROMAT LCDC_PIXEL_FORMAT_RGB565

#define RAM_LCD_BUFFER_SIZE(bits_per_pixel) ((RAM_LCD_WIDTH * RAM_LCD_HEIGHT * (bits_per_pixel)) >> 3)
//////////////////////////////////////////////////////////////////////////////////


static LCD_DrvTypeDef drv_lcd;
static void lcd_draw_rect(const char *pixels, int x0, int y0, int x1, int y1);

struct rt_device_graphic_info ram_lcd_cfg =
{
    .pixel_format = 0,
    .bits_per_pixel = 0,
    .draw_align     = 1,
    .width = RAM_LCD_WIDTH,
    .height = RAM_LCD_HEIGHT,
    .framebuffer = NULL,
    .bandwidth = 48000000, //Unuse.
};

static LCDC_InitTypeDef lcdc_int_cfg =
{
    .freq = 240 * 1000 * 1000, //Non-zero frequency, avoid assertion only.
    .lcd_itf = LCDC_INTF_AHB,
    .color_mode = RAM_LCD_FROMAT,
};


static rt_err_t sram_lcd_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t sram_lcd_cfg_reinit(uint16_t cf)
{
    if ((ram_lcd_cfg.pixel_format == cf)
            && (ram_lcd_cfg.framebuffer != NULL))return RT_EOK;


    switch (cf)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        ram_lcd_cfg.pixel_format = cf;
        ram_lcd_cfg.bits_per_pixel = 16;
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        ram_lcd_cfg.pixel_format = cf;
        ram_lcd_cfg.bits_per_pixel = 24;
        break;

    default:
        DEBUG_PRINTF("sram_lcd_cfg_reinit INVALID color format \n");
        return RT_EINVAL;
    }

    if (NULL != ram_lcd_cfg.framebuffer)
    {
        rt_free(ram_lcd_cfg.framebuffer);
        ram_lcd_cfg.framebuffer = NULL;
    }

    ram_lcd_cfg.framebuffer = rt_malloc(RAM_LCD_BUFFER_SIZE(ram_lcd_cfg.bits_per_pixel));
    if (NULL != ram_lcd_cfg.framebuffer)
    {
        memset(ram_lcd_cfg.framebuffer, 0, RAM_LCD_BUFFER_SIZE(ram_lcd_cfg.bits_per_pixel));
        DEBUG_PRINTF("ram_lcd framebuffer = %x\n", ram_lcd_cfg.framebuffer);


        lcdc_int_cfg.cfg.ahb.lcd_mem = (uint32_t) ram_lcd_cfg.framebuffer;
        lcdc_int_cfg.color_mode = rt_lcd_format_to_hal_lcd_format(ram_lcd_cfg.pixel_format);
        memcpy(&drv_lcd.hlcdc.Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
        HAL_LCDC_Init(&drv_lcd.hlcdc);

        return RT_EOK;
    }
    else
    {
        DEBUG_PRINTF("ram alloc frambuffer failed! \n");
        return RT_ENOMEM;
    }

}

static rt_err_t sram_lcd_open(rt_device_t dev, rt_uint16_t oflag)
{

    rt_err_t err;

    err = sram_lcd_cfg_reinit(RTGRAPHIC_PIXEL_FORMAT_RGB565);
    if (RT_EOK == err)
    {

        DEBUG_PRINTF("ram lcd_open \n");


        rt_sem_init(&drv_lcd.sem, "ram_lcd", 1, RT_IPC_FLAG_FIFO);


#if SOC_BF0_HCPU
        HAL_NVIC_SetPriority(LCDC1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(LCDC1_IRQn);
#else
        HAL_NVIC_SetPriority(LCDC2_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(LCDC2_IRQn);
#endif

        /*Set LCDC background*/
        HAL_LCDC_SetBgColor(&drv_lcd.hlcdc, 120, 112, 176);

        /*Set default layer configuration*/
        HAL_LCDC_LayerReset(&drv_lcd.hlcdc, HAL_LCDC_LAYER_DEFAULT);

        drv_lcd.status |= LCD_DRV_ACTIVED;
    }

    return err;
}

static rt_err_t sram_lcd_close(rt_device_t dev)
{
    if (NULL != ram_lcd_cfg.framebuffer)
    {
        rt_free(ram_lcd_cfg.framebuffer);
        DEBUG_PRINTF("ram lcd free buffer %x\n", ram_lcd_cfg.framebuffer);
        ram_lcd_cfg.framebuffer = NULL;
    }
    rt_sem_detach(&drv_lcd.sem);

    drv_lcd.status &= ~LCD_DRV_ACTIVED;

    return RT_EOK;
}

static rt_err_t sram_lcd_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case RTGRAPHIC_CTRL_GET_INFO:
    {
        struct rt_device_graphic_info *info;

        info = (struct rt_device_graphic_info *) args;
        RT_ASSERT(info != RT_NULL);

        memcpy(info, &ram_lcd_cfg, sizeof(struct rt_device_graphic_info));
    }
    break;

    case RTGRAPHIC_CTRL_RECT_UPDATE:
        /* nothong to be done */
        break;

    case RTGRAPHIC_CTRL_POWERON:
    case RTGRAPHIC_CTRL_SET_BRIGHTNESS:
        break;

    case RTGRAPHIC_CTRL_POWEROFF:
        break;


    case RT_DEVICE_CTRL_RESUME:
        break;

    case RT_DEVICE_CTRL_SUSPEND:
        break;

    case SF_GRAPHIC_CTRL_LCDC_OUT_FORMAT:
        if (args)
        {

            rt_err_t err;

            err = sram_lcd_cfg_reinit(*((uint16_t *)args));
            if (RT_EOK == err)
            {

            }
            else
            {
                DEBUG_PRINTF("set sram lcd out buf format err=%d \n", err);
            }

        }
        break;

    default:
        rt_lcd_layer_control(&drv_lcd, cmd, args);
        break;
    }

    return RT_EOK;
}


static void lcd_set_window(int x0, int y0, int x1, int y1)
{
    DEBUG_PRINTF("ram lcd_set_window [%d,%d,%d,%d] \n", x0, y0, x1, y1);

    lcdc_int_cfg.cfg.ahb.lcd_mem = (uint32_t) ram_lcd_cfg.framebuffer + ((x0 + y0 * ram_lcd_cfg.width) * (ram_lcd_cfg.bits_per_pixel >> 3));
    lcdc_int_cfg.cfg.ahb.lcd_o_width = (ram_lcd_cfg.width - (x1 - x0 + 1)) * (ram_lcd_cfg.bits_per_pixel >> 3);


    memcpy(&drv_lcd.hlcdc.Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(&drv_lcd.hlcdc);

    HAL_LCDC_SetROIArea(&drv_lcd.hlcdc, x0, y0, x1, y1);
}

static void lcd_set_pixel(const char *pixel, int x, int y)
{
    lcd_draw_rect(pixel, x, y, x, y);
}

static void lcd_get_pixel(char *pixel, int x, int y)
{
    rt_uint8_t *p_buf;

    if (NULL == pixel)
    {
        return;
    }

    p_buf = ram_lcd_cfg.framebuffer + ((x + y * ram_lcd_cfg.width) * (ram_lcd_cfg.bits_per_pixel >> 3));


    memcpy(pixel, p_buf, (ram_lcd_cfg.bits_per_pixel >> 3));


    DEBUG_PRINTF("ram lcd pixel[%d,%d]: addr %x = %x\n", x, y, p_buf,
                 (RTGRAPHIC_PIXEL_FORMAT_RGB565 ==  ram_lcd_cfg.pixel_format) ?  * ((uint16_t *) p_buf) : * ((uint32_t *) p_buf)
                );
}

static void lcd_draw_hline(const char *pixel, int x1, int x2, int y)
{

}

static void lcd_draw_vline(const char *pixel, int x, int y1, int y2)
{
}

static void lcd_blit_line(const char *pixels, int x, int y, rt_size_t size)
{
    lcd_draw_rect(pixels, x, y, x + size - 1, y);
}


LCDC_HandleTypeDef *get_ram_lcd_handler(void)
{
    if (drv_lcd.status & LCD_DRV_ACTIVED)
        return &drv_lcd.hlcdc;
    else
        return NULL;
}

static void SendLayerDataCpltCbk(LCDC_HandleTypeDef *lcdc)
{
    LCD_DrvTypeDef *p_lcd = rt_container_of(lcdc, LCD_DrvTypeDef, hlcdc);
    rt_device_t p_lcd_device = &p_lcd->parent;

    lcdc->XferCpltCallback = NULL;

    {
        rt_err_t err;
        err = rt_sem_release(&(p_lcd->sem));
        RT_ASSERT(RT_EOK == err);
    }


    if (p_lcd_device->tx_complete)
    {
        p_lcd_device->tx_complete(p_lcd_device, lcdc->Layer[p_lcd->select_layer].data);
    }

}


static void lcd_draw_rect(const char *pixels, int x0, int y0, int x1, int y1)
{
    rt_err_t err;

    /*
        int y;
        uint32_t copy_bytes_per_row, hor_line_bytes;
        rt_uint8_t *p_buf;

        p_buf = ram_lcd_cfg.framebuffer + (x0 * (ram_lcd_cfg.bits_per_pixel>>3));
        copy_bytes_per_row = (x1 - x0 + 1) * (ram_lcd_cfg.bits_per_pixel>>3);
        hor_line_bytes = ram_lcd_cfg.width * (ram_lcd_cfg.bits_per_pixel>>3);

        if(copy_bytes_per_row)
        {
            for(y= y0; y <= y1; y++)
            {
                memcpy(p_buf,pixels,copy_bytes_per_row);
            }
        }
    */

    err = rt_sem_take(&(drv_lcd.sem), rt_tick_from_millisecond(MAX_LCD_DRAW_TIME));
    RT_ASSERT(RT_EOK == err);

    drv_lcd.hlcdc.XferCpltCallback = SendLayerDataCpltCbk;
    HAL_LCDC_LayerSetData(&drv_lcd.hlcdc, drv_lcd.select_layer, (uint8_t *)pixels, x0, y0, x1, y1);
    HAL_LCDC_SendLayerData_IT(&drv_lcd.hlcdc);


    err = rt_sem_take(&(drv_lcd.sem), rt_tick_from_millisecond(MAX_LCD_DRAW_TIME));
    RT_ASSERT(RT_EOK == err);

    err = rt_sem_release(&(drv_lcd.sem));
    RT_ASSERT(RT_EOK == err);

}

static void lcd_draw_rect_async(const char *pixels, int x0, int y0, int x1, int y1)
{
    rt_err_t err;

    DEBUG_PRINTF("ram lcd_draw_rect area[%d,%d,%d,%d] \n", x0, y0, x1, y1);

    err = rt_sem_take(&(drv_lcd.sem), rt_tick_from_millisecond(MAX_LCD_DRAW_TIME));
    RT_ASSERT(RT_EOK == err);

    drv_lcd.hlcdc.XferCpltCallback = SendLayerDataCpltCbk;
    HAL_LCDC_LayerSetData(&drv_lcd.hlcdc, drv_lcd.select_layer, (uint8_t *)pixels, x0, y0, x1, y1);
    HAL_LCDC_SendLayerData_IT(&drv_lcd.hlcdc);
}



static int rt_hw_ram_lcd_init(void)
{
    static const struct rt_device_graphic_ops ops =
    {
        lcd_set_pixel,
        lcd_get_pixel,
        NULL,
        NULL,
        lcd_blit_line,
        lcd_draw_rect,
        lcd_draw_rect_async,
        lcd_set_window,
    };



#if SOC_BF0_HCPU
    drv_lcd.hlcdc.Instance = LCDC1;
#else
    drv_lcd.hlcdc.Instance = LCDC2;
#endif

    drv_lcd.select_layer = HAL_LCDC_LAYER_DEFAULT;

    {
        rt_device_t p_lcd_device;
        p_lcd_device = &drv_lcd.parent;

        /* register lcd device */
        p_lcd_device->type = RT_Device_Class_Graphic;
        p_lcd_device->init = sram_lcd_init;
        p_lcd_device->open = sram_lcd_open;
        p_lcd_device->close = sram_lcd_close;
        p_lcd_device->control = sram_lcd_control;
        p_lcd_device->read = RT_NULL;
        p_lcd_device->write = RT_NULL;

        p_lcd_device->user_data = (void *)&ops;

        /* register graphic device driver */
        rt_device_register(p_lcd_device, "ram_lcd",
                           RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
    }
    return 0;
}
INIT_BOARD_EXPORT(rt_hw_ram_lcd_init);
#endif /* APP_BSP_TEST */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

