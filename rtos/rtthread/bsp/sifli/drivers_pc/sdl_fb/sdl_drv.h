/**
  ******************************************************************************
  * @file   sdl_drv.h
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

#ifndef SDL_DRV_H
#define SDL_DRV_H

#include "rtconfig.h"
#define RTGUI_MOUSE_BUTTON_DOWN     0x01
#define RTGUI_MOUSE_BUTTON_LEFT     0x02
#define RTGUI_MOUSE_BUTTON_RIGHT    0x04
#define RTGUI_MOUSE_BUTTON_MIDDLE   0x08
#define RTGUI_MOUSE_BUTTON_UP       0x10

struct rtgui_event_mouse
{
    int x, y;
    rt_uint32_t button;
};

struct rtgui_event_kbd
{
    int mod;
    int key;
};
#ifdef LV_HOR_RES_MAX
    #define SDL_SCREEN_WIDTH    LV_HOR_RES_MAX
#else
    #define SDL_SCREEN_WIDTH    454
#endif
#ifdef LV_VER_RES_MAX
    #define SDL_SCREEN_HEIGHT   LV_VER_RES_MAX
#else
    #define SDL_SCREEN_HEIGHT   454
#endif

#define SDL_SCREEN_FORMAT   SDL_PIXELFORMAT_RGB565

void sdlfb_hw_init(void);

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
