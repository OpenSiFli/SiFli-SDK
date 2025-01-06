/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#ifdef BSP_USING_TOUCHD
    #include "drv_touch.h"
#endif
#include "mem_section.h"



#ifdef BSP_USING_TOUCHD
static struct rt_semaphore tp_sema;

static rt_err_t tp_rx_indicate(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&tp_sema);
    return RT_EOK;
}

static void touch_read_task(void *parameter)
{
    rt_device_t touch_device = parameter;
    if (touch_device)
    {
        /*Open touch device*/
        if (RT_EOK == rt_device_open(touch_device, RT_DEVICE_FLAG_RDONLY))
        {
            /*Setup rx indicate callback and semaphore*/
            rt_sem_init(&tp_sema, "tpsem", 0, RT_IPC_FLAG_FIFO);
            rt_device_set_rx_indicate(touch_device, tp_rx_indicate);

            while (1)
            {
                struct touch_message touch_data;

                /*Wait touch indication*/
                rt_sem_take(&tp_sema, RT_WAITING_FOREVER);

                /*Read touch point data*/
                rt_device_read(touch_device, 0, &touch_data, 1);

                if (TOUCH_EVENT_DOWN == touch_data.event)
                    rt_kprintf("Touch down [%d,%d]\r\n", touch_data.x, touch_data.y);
                else
                    rt_kprintf("Touch up   [%d,%d]\r\n", touch_data.x, touch_data.y);
            }
        }
        else
        {
            rt_kprintf("Touch open error!\n");
        }
    }
}

#endif



static uint8_t get_pixel_size(uint16_t color_format)
{
    uint8_t pixel_size;

    if (RTGRAPHIC_PIXEL_FORMAT_RGB565 == color_format)
    {
        pixel_size = 2;
    }
    else if (RTGRAPHIC_PIXEL_FORMAT_RGB888 == color_format)
    {
        pixel_size = 3;
    }
    else
    {
        pixel_size = 0;
    }

    return pixel_size;
}

static uint32_t make_color(uint16_t cf, uint32_t rgb888)
{
    uint8_t r, g, b;

    r = (rgb888 >> 16) & 0xFF;
    g = (rgb888 >> 8) & 0xFF;
    b = (rgb888 >> 0) & 0xFF;

    switch (cf)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        return ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF8) >> 3));
    case RTGRAPHIC_PIXEL_FORMAT_RGB666:
        return ((((r) & 0xFC) << 10) | (((g) & 0xFC) << 4) | (((b) & 0xFC) >> 2));
    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        return ((((r) & 0xFF) << 16) | (((g) & 0xFF) << 8) | ((b) & 0xFF));

    default:
        return 0;
    }
}

static void fill_color(uint8_t *buf, uint32_t width, uint32_t height,
                       uint16_t cf, uint32_t ARGB8888)
{
    uint8_t pixel_size;

    if (RTGRAPHIC_PIXEL_FORMAT_RGB565 == cf)
    {
        pixel_size = 2;
    }
    else if (RTGRAPHIC_PIXEL_FORMAT_RGB888 == cf)
    {
        pixel_size = 3;
    }
    else
    {
        RT_ASSERT(0);
    }

    uint32_t i, j, k, c;
    c = make_color(cf, ARGB8888);
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            for (k = 0; k < pixel_size; k++)
            {
                *buf++ = (c >> (k << 3)) & 0xFF;
            }
        }
    }
}

/*
        |<-------------  buffer width --------------------->|
---      0x00000001   0x00000002  0x00000003  ....  0x000000FF
^       0x00000001   0x00000002  0x00000003  ....  0x000000FF
|       0x00000001   0x00000002  0x00000003  ....  0x000000FF
buffer   0x00000001   0x00000002  0x00000003  ....  0x000000FF
height   0x00000001   0x00000002  0x00000003  ....  0x000000FF
 |      0x00000001   0x00000002  0x00000003  ....  0x000000FF
 |      0x00000001   0x00000002  0x00000003  ....  0x000000FF
 v      0x00000001   0x00000002  0x00000003  ....  0x000000FF
----     0x00000001   0x00000002  0x00000003  ....  0x000000FF

*/
static void fill_hor_gradient_color(uint8_t *buf, uint32_t width, uint32_t height,
                                    uint16_t cf, uint32_t color)
{
    uint8_t pixel_size;
    uint8_t *buf_head = buf;

    pixel_size = get_pixel_size(cf);

    uint32_t i, j, k;
    for (i = 0; i < height; i++)
    {
        if (0 == i) //fill first line
        {
            for (j = 0; j < width; j++)
            {
                uint8_t r, g, b;
                uint32_t grad_color;

                r = ((color >> 16) & 0xFF) * j / width;
                g = ((color >> 8) & 0xFF) * j / width;
                b = ((color >> 0) & 0xFF) * j / width;

                grad_color = make_color(cf, (r << 16 | g << 8 | b));
                switch (cf)
                {
                case RTGRAPHIC_PIXEL_FORMAT_RGB565:
                    grad_color = grad_color & 0xFFDF; //Convert to RGB555 to make gradient more smooth.
                    *(buf ++) = (uint8_t)((grad_color >> 0) & 0xFF);
                    *(buf ++) = (uint8_t)((grad_color >> 8) & 0xFF);
                    break;

                case RTGRAPHIC_PIXEL_FORMAT_RGB888:
                    *(buf ++) = (uint8_t)((grad_color >> 0) & 0xFF);
                    *(buf ++) = (uint8_t)((grad_color >> 8) & 0xFF);
                    *(buf ++) = (uint8_t)((grad_color >> 16) & 0xFF);
                    break;
                default:
                    RT_ASSERT(0);
                    break;
                }
            }
        }
        else//copy to left lines
        {
            memcpy(buf, buf - width * pixel_size, width * pixel_size);
            buf += width * pixel_size;
        }
    }
}


/*
        |<-------------  buffer width --------------------->|
---        0x00000000      0x00000000      0x00000000       0x00000000
^          0x00010101      0x00010101      0x00010101       0x00010101
|          0x00020202      0x00020202      0x00020202       0x00020202
buffer     0x00030303      0x00030303      0x00030303       0x00030303
height           ....         ....         ....         ....
 |         0x00FFFFFF      0x00FFFFFF      0x00FFFFFF      0x00FFFFFF
 |         0x00000001      0x00000001      0x00000001      0x00000001
 |         0x00020202      0x00020202      0x00020202      0x00020202
 |         0x00030303      0x00030303      0x00030303      0x00030303
 |             ....      ....           ....              ....
 |         0x00FFFFFF      0x00FFFFFF      0x00FFFFFF      0x00FFFFFF
 v              ...          ...           ...                ...
----

*/
static void fill_ver_gradient_color(uint8_t *buf, uint32_t width, uint32_t height,
                                    uint16_t cf)
{
    uint8_t pixel_size;
    uint8_t *buf_head = buf;

    pixel_size = get_pixel_size(cf);

    uint32_t i, j;
    for (i = 0; i < height; i++)
    {
        uint8_t r, g, b;
        uint32_t grad_color;

        r = (uint8_t) i & 0xFF;
        g = r;
        b = r;

        grad_color = make_color(cf, (r << 16 | g << 8 | b));

        for (j = 0; j < width; j++)
        {
            switch (cf)
            {
            case RTGRAPHIC_PIXEL_FORMAT_RGB565:
                *(buf ++) = (uint8_t)((grad_color >> 0) & 0xFF);
                *(buf ++) = (uint8_t)((grad_color >> 8) & 0xFF);
                break;

            case RTGRAPHIC_PIXEL_FORMAT_RGB888:
                *(buf ++) = (uint8_t)((grad_color >> 0) & 0xFF);
                *(buf ++) = (uint8_t)((grad_color >> 8) & 0xFF);
                *(buf ++) = (uint8_t)((grad_color >> 16) & 0xFF);
                break;
            default:
                RT_ASSERT(0);
                break;
            }
        }
    }
}


static void fill_color_bar(uint8_t *buf, uint32_t width, uint32_t height, uint16_t cf)
{
    uint32_t i = 0;
    uint32_t line_height = height / 8;
    uint32_t offset = get_pixel_size(cf) * width * line_height;

    fill_hor_gradient_color(buf + offset * i, width, line_height, cf, 0xFFFFFF);
    i++;
    fill_hor_gradient_color(buf + offset * i, width, line_height, cf, 0x777777);
    i++;
    fill_hor_gradient_color(buf + offset * i, width, line_height, cf, 0xFF0000);
    i++;
    fill_hor_gradient_color(buf + offset * i, width, line_height, cf, 0x00FF00);
    i++;
    fill_hor_gradient_color(buf + offset * i, width, line_height, cf, 0x0000FF);
    i++;
    fill_hor_gradient_color(buf + offset * i, width, line_height, cf, 0x00FFFF);
    i++;
    fill_hor_gradient_color(buf + offset * i, width, line_height, cf, 0xFF00FF);
    i++;
    fill_hor_gradient_color(buf + offset * i, width, line_height, cf, 0xFFFF00);
    i++;
}


#ifdef BSP_USING_LCD
#define FB_WIDTH  LCD_HOR_RES_MAX
#define FB_HEIGHT LCD_VER_RES_MAX
#define FB_COLOR_FORMAT RTGRAPHIC_PIXEL_FORMAT_RGB888
#define FB_PIXEL_BYTES  3
#define FB_TOTAL_BYTES (FB_WIDTH * FB_HEIGHT *FB_PIXEL_BYTES)

/*
    Define framebuffer for flushing LCD, using double framebuffer if LCD is ramless.
*/
L2_NON_RET_BSS_SECT_BEGIN(frambuf)
L2_NON_RET_BSS_SECT(frambuf, ALIGN(64) static uint8_t framebuffer1[FB_TOTAL_BYTES]);
#ifdef BSP_USING_RAMLESS_LCD
    L2_NON_RET_BSS_SECT(frambuf, ALIGN(64) static uint8_t framebuffer2[FB_TOTAL_BYTES]);
#endif /* BSP_USING_RAMLESS_LCD */
L2_NON_RET_BSS_SECT_END

static struct rt_semaphore lcd_sema;
static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{
    rt_sem_release(&lcd_sema);
    return RT_EOK;
}


static void lcd_refresh_task(void *parameter)
{
    /*
        Check macro values
    */
    RT_ASSERT(get_pixel_size(FB_COLOR_FORMAT) == FB_PIXEL_BYTES);




    /*
        Open LCD Device and get LCD infomation
    */
    rt_device_t lcd_device = (rt_device_t) parameter;
    if (rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
    {
        struct rt_device_graphic_info info;
        if (rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &info) == RT_EOK)
        {
            rt_kprintf("Lcd info w:%d, h%d, bits_per_pixel %d, draw_align:%d\r\n",
                       info.width, info.height, info.bits_per_pixel, info.draw_align);
        }
    }
    else
    {
        rt_kprintf("Lcd open error!\n");
        return;
    }



    /*
        Start Loop
    */
    uint8_t loop = 0;
    uint8_t *p_framebuffer = (uint8_t *)&framebuffer1[0];

    rt_sem_init(&lcd_sema, "lcdsem", 0, RT_IPC_FLAG_FIFO);


    while (1)
    {
        uint16_t framebuffer_color_format = FB_COLOR_FORMAT;

        /*Fill framebuffer*/
        rt_kprintf("Fill framebuffer addr=0x%x, w=%d, h=%d, size=%d(Bytes)\n", p_framebuffer, FB_WIDTH, FB_HEIGHT, FB_TOTAL_BYTES);
        if (0 == loop) fill_color_bar((uint8_t *)p_framebuffer, FB_WIDTH, FB_HEIGHT, framebuffer_color_format);  //Fill 8 gradient color bar
        if (1 == loop) fill_ver_gradient_color((uint8_t *)p_framebuffer, FB_WIDTH, FB_HEIGHT, framebuffer_color_format);  //Fill vertical grey color bar
        if (2 == loop) fill_color((uint8_t *)p_framebuffer, FB_WIDTH, FB_HEIGHT, framebuffer_color_format, 0xFF0000); //Fill RED color
        if (3 == loop) fill_color((uint8_t *)p_framebuffer, FB_WIDTH, FB_HEIGHT, framebuffer_color_format, 0x00FF00); //Fill GREEN color
        if (4 == loop) fill_color((uint8_t *)p_framebuffer, FB_WIDTH, FB_HEIGHT, framebuffer_color_format, 0x0000FF); //Fill BLUE color
        if (5 == loop) fill_color((uint8_t *)p_framebuffer, FB_WIDTH, FB_HEIGHT, framebuffer_color_format, 0xFFFFFF); //Fill WHITE color
        if (6 == loop) fill_color((uint8_t *)p_framebuffer, FB_WIDTH, FB_HEIGHT, framebuffer_color_format, 0x000000); //Fill BLACK color
        loop = (6 == loop) ? 0 : (loop + 1);


        /*Flush framebuffer to LCD*/
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &framebuffer_color_format);
        rt_graphix_ops(lcd_device)->set_window(0, 0, FB_WIDTH - 1, FB_HEIGHT - 1);
        rt_device_set_tx_complete(lcd_device, lcd_flush_done);
        rt_graphix_ops(lcd_device)->draw_rect_async((const char *)p_framebuffer, 0, 0, FB_WIDTH - 1, FB_HEIGHT - 1);


        /*Waitting for Flushing LCD done*/
        rt_sem_take(&lcd_sema, RT_WAITING_FOREVER);


        /* Set LCD backlight brightness level */
        uint8_t brightness = 100;
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &brightness);


        /*Delay*/
        rt_thread_delay(rt_tick_from_millisecond(3000));

#ifdef BSP_USING_RAMLESS_LCD
        /*
            ramless LCD is reading current framebuffer always,

            so fill another framebuffer.
        */
        if (p_framebuffer == (uint8_t *)&framebuffer1[0])
            p_framebuffer = (uint8_t *)&framebuffer2[0];
        else
            p_framebuffer = (uint8_t *)&framebuffer1[0];
#endif /* BSP_USING_RAMLESS_LCD */
    }

}
#endif


int main(void)
{
    rt_device_t p_device;

    rt_thread_t tid;

#ifdef BSP_USING_LCD
    /*
        Create LCD refreshing task if found 'lcd' device.
    */
    p_device = rt_device_find("lcd");
    if (p_device)
    {
        tid = rt_thread_create("lcd_refr", lcd_refresh_task, p_device, 1024, 30, 10);
        rt_thread_startup(tid);
    }
#endif


#ifdef BSP_USING_TOUCHD
    /*
        Create touch reading task if found 'touch' device.
    */
    p_device = rt_device_find("touch");
    if (p_device)
    {
        tid = rt_thread_create("touch_read", touch_read_task, p_device, 1024, 16, 10);
        rt_thread_startup(tid);
    }
#endif



    while (1)
    {
        rt_thread_mdelay(5000);
        rt_kprintf("__main loop__\r\n");
    }

    return RT_EOK;
}




/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

