/**
  ******************************************************************************
  * @file   sdl_fb.c
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

#include <rtthread.h>

#include <stdio.h>

#include <sdl.h>
#include <rtdevice.h>
#include "sdl_drv.h"
extern void rt_hw_exit(void);

struct sdlfb_device
{
    struct rt_device parent;

    SDL_Renderer *renderer;         /* window renderer */
    SDL_Surface  *surface;          /* screen surface  */

    rt_uint16_t width;
    rt_uint16_t height;
};
struct sdlfb_device _device;

/* common device interface */
static rt_err_t  sdlfb_init(rt_device_t dev)
{
    return RT_EOK;
}
static rt_err_t  sdlfb_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}
static rt_err_t  sdlfb_close(rt_device_t dev)
{
    SDL_Quit();
    return RT_EOK;
}

int sdlfb_info(int *format, int *bpp)
{
    int bit_pp;                         /* texture bits per pixel */
    Uint32 Rmask, Gmask, Bmask, Amask;  /* masks for pixel format passed into OpenGL */

    /* Grab info about format that will be passed into OpenGL */
    SDL_PixelFormatEnumToMasks(SDL_SCREEN_FORMAT, &bit_pp, &Rmask, &Gmask,
                               &Bmask, &Amask);

    *bpp = bit_pp;
    switch (SDL_SCREEN_FORMAT)
    {
    case SDL_PIXELFORMAT_RGB565:
        *format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
        break;
    case SDL_PIXELFORMAT_RGB888:
        *format = RTGRAPHIC_PIXEL_FORMAT_RGB888;
        break;
    case SDL_PIXELFORMAT_ARGB8888:
        *format = RTGRAPHIC_PIXEL_FORMAT_ARGB888;
        break;
    case SDL_PIXELFORMAT_ABGR8888:
        *format = RTGRAPHIC_PIXEL_FORMAT_ABGR888;
    }

    return 0;
}

static rt_mutex_t sdllock;
static rt_err_t  sdlfb_control(rt_device_t dev, int cmd, void *args)
{
    struct sdlfb_device *device;

    rt_mutex_take(sdllock, RT_WAITING_FOREVER);
    device = (struct sdlfb_device *)dev;
    RT_ASSERT(device != RT_NULL);

    switch (cmd)
    {
    case RTGRAPHIC_CTRL_GET_INFO:
    {
        int format, bpp;
        struct rt_device_graphic_info *info;

        sdlfb_info(&format, &bpp);
        info = (struct rt_device_graphic_info *) args;
        info->bits_per_pixel = bpp;
        info->pixel_format = format;
        info->framebuffer = (rt_uint8_t *)device->surface->pixels;
        info->width = device->width;
        info->height = device->height;
    }
    break;

    case RTGRAPHIC_CTRL_RECT_UPDATE:
    {
        SDL_Texture *texture;
        struct rt_device_rect_info *rect;
        SDL_Rect _rect = { 0, 0, SDL_SCREEN_WIDTH, SDL_SCREEN_HEIGHT };

        rect = (struct rt_device_rect_info *)args;

        SDL_RenderClear(_device.renderer);

        texture = SDL_CreateTextureFromSurface(_device.renderer, _device.surface);
        SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);
        SDL_RenderCopy(_device.renderer, texture, NULL, &_rect);

        SDL_RenderPresent(_device.renderer);

        SDL_DestroyTexture(texture);
    }
    break;
    case RTGRAPHIC_CTRL_SET_MODE:
    {
        break;
    }
    break;
    }
    rt_mutex_release(sdllock);
    return RT_EOK;
}

Uint16 pixels[16 * 16] =    // ...or with raw pixel data:
{
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0aab, 0x0789, 0x0bcc, 0x0eee, 0x09aa, 0x099a, 0x0ddd,
    0x0fff, 0x0eee, 0x0899, 0x0fff, 0x0fff, 0x1fff, 0x0dde, 0x0dee,
    0x0fff, 0xabbc, 0xf779, 0x8cdd, 0x3fff, 0x9bbc, 0xaaab, 0x6fff,
    0x0fff, 0x3fff, 0xbaab, 0x0fff, 0x0fff, 0x6689, 0x6fff, 0x0dee,
    0xe678, 0xf134, 0x8abb, 0xf235, 0xf678, 0xf013, 0xf568, 0xf001,
    0xd889, 0x7abc, 0xf001, 0x0fff, 0x0fff, 0x0bcc, 0x9124, 0x5fff,
    0xf124, 0xf356, 0x3eee, 0x0fff, 0x7bbc, 0xf124, 0x0789, 0x2fff,
    0xf002, 0xd789, 0xf024, 0x0fff, 0x0fff, 0x0002, 0x0134, 0xd79a,
    0x1fff, 0xf023, 0xf000, 0xf124, 0xc99a, 0xf024, 0x0567, 0x0fff,
    0xf002, 0xe678, 0xf013, 0x0fff, 0x0ddd, 0x0fff, 0x0fff, 0xb689,
    0x8abb, 0x0fff, 0x0fff, 0xf001, 0xf235, 0xf013, 0x0fff, 0xd789,
    0xf002, 0x9899, 0xf001, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0xe789,
    0xf023, 0xf000, 0xf001, 0xe456, 0x8bcc, 0xf013, 0xf002, 0xf012,
    0x1767, 0x5aaa, 0xf013, 0xf001, 0xf000, 0x0fff, 0x7fff, 0xf124,
    0x0fff, 0x089a, 0x0578, 0x0fff, 0x089a, 0x0013, 0x0245, 0x0eff,
    0x0223, 0x0dde, 0x0135, 0x0789, 0x0ddd, 0xbbbc, 0xf346, 0x0467,
    0x0fff, 0x4eee, 0x3ddd, 0x0edd, 0x0dee, 0x0fff, 0x0fff, 0x0dee,
    0x0def, 0x08ab, 0x0fff, 0x7fff, 0xfabc, 0xf356, 0x0457, 0x0467,
    0x0fff, 0x0bcd, 0x4bde, 0x9bcc, 0x8dee, 0x8eff, 0x8fff, 0x9fff,
    0xadee, 0xeccd, 0xf689, 0xc357, 0x2356, 0x0356, 0x0467, 0x0467,
    0x0fff, 0x0ccd, 0x0bdd, 0x0cdd, 0x0aaa, 0x2234, 0x4135, 0x4346,
    0x5356, 0x2246, 0x0346, 0x0356, 0x0467, 0x0356, 0x0467, 0x0467,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
    0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff
};

void sdlfb_hw_init(void)
{
    SDL_Window *win;

    if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
    {
        fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
        exit(1);
    }

    _device.parent.init = sdlfb_init;
    _device.parent.open = sdlfb_open;
    _device.parent.close = sdlfb_close;
    _device.parent.read = RT_NULL;
    _device.parent.write = RT_NULL;
    _device.parent.control = sdlfb_control;

    _device.width  = SDL_SCREEN_WIDTH;
    _device.height = SDL_SCREEN_HEIGHT;

    {
        int bpp;                            /* texture bits per pixel */
        Uint32 Rmask, Gmask, Bmask, Amask;  /* masks for pixel format passed into OpenGL */

        /* Grab info about format that will be passed into OpenGL */
        SDL_PixelFormatEnumToMasks(SDL_SCREEN_FORMAT, &bpp, &Rmask, &Gmask,
                                   &Bmask, &Amask);

        _device.surface = SDL_CreateRGBSurface(0, SDL_SCREEN_WIDTH, SDL_SCREEN_HEIGHT,
                                               bpp, Rmask, Gmask, Bmask, Amask);
    }

    win = SDL_CreateWindow("RT-Thread/Demo",
                           SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                           SDL_SCREEN_WIDTH, SDL_SCREEN_HEIGHT, SDL_WINDOW_OPENGL);
    if (win == NULL)
    {
        exit(1);
    }

    /* set the icon of Window */
    {
        SDL_Surface *surface;
        surface = SDL_CreateRGBSurfaceFrom(pixels, 16, 16, 16, 16 * 2, 0x0f00, 0x00f0, 0x000f, 0xf000);
        SDL_SetWindowIcon(win, surface);
    }

    _device.renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    rt_device_register(RT_DEVICE(&_device), "lcd", RT_DEVICE_FLAG_RDWR);

    sdllock = rt_mutex_create("fb", RT_IPC_FLAG_FIFO);
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
