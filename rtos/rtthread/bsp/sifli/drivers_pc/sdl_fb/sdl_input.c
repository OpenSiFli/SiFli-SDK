/**
  ******************************************************************************
  * @file   sdl_input.c
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

#include  <windows.h>
#include  <mmsystem.h>
#include  <stdio.h>
#include  <sdl.h>
#include  <rtthread.h>
#include  "sdl_drv.h"


static HANDLE  sdl_ok_event = NULL;

void rtgui_server_post_event(void *event, int len)
{
}

static int sdl_loop(LPVOID lpParam)
{
    int quit = 0;
    SDL_Event event;
    int state = 0;
    int timeout = 1000000;

    int last_x = -1, last_y = -1;

    int motion_x, motion_y;
    int motion_button = 0;
    int motion_tick = 50;

    int mouse_id = 1;

    SetEvent(sdl_ok_event);

    /* handle SDL event */
    while (!quit)
    {
        if (motion_button & RTGUI_MOUSE_BUTTON_DOWN)
        {
            timeout = motion_tick - SDL_GetTicks();
        }
        else
        {
            timeout = 1000;
        }

        int tick = SDL_GetTicks();
        SDL_WaitEventTimeout(&event, timeout);
        //rt_kprintf("Got event %x\n", event.type);
        if (SDL_GetTicks() >= (uint32_t) motion_tick && (motion_button & RTGUI_MOUSE_BUTTON_DOWN)) /* whether timeout */
        {
            /* sendout motion event */
            struct rtgui_event_mouse emouse;

            if ((motion_x > 0 && motion_x < SDL_SCREEN_WIDTH) &&
                    (motion_y > 0 && motion_y < SDL_SCREEN_HEIGHT))
            {
                emouse.x = motion_x;
                emouse.y = motion_y;

                /* init mouse button */
                emouse.button = motion_button;

                /* send event to server */
                rtgui_server_post_event(&emouse, sizeof(struct rtgui_event_mouse));
            }

            /* reset motion tick */
            motion_tick = 50 + SDL_GetTicks();
        }

        switch (event.type)
        {
        case SDL_MOUSEMOTION:
        {
            /* save to (x,y) in the motion */
            motion_x = ((SDL_MouseMotionEvent *)&event)->x;
            motion_y = ((SDL_MouseMotionEvent *)&event)->y;
        }
        break;

        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
        {
            int x, y;
            struct rtgui_event_mouse emouse;
            SDL_MouseButtonEvent *mb;


            if (event.type == SDL_MOUSEBUTTONDOWN && state == 0)
                mouse_id = rt_tick_get();

            mb = (SDL_MouseButtonEvent *)&event;

            x = mb->x;
            y = mb->y;
            if ((x > 0 && x < SDL_SCREEN_WIDTH) &&
                    (y > 0 && y < SDL_SCREEN_HEIGHT))
            {
                emouse.x = x;
                emouse.y = y;

                /* init mouse button */
                emouse.button = 0;

                /* set emouse button */
                if (mb->button & (1 << (SDL_BUTTON_LEFT - 1)))
                {
                    emouse.button |= RTGUI_MOUSE_BUTTON_LEFT;
                }
                else if (mb->button & (1 << (SDL_BUTTON_RIGHT - 1)))
                {
                    emouse.button |= RTGUI_MOUSE_BUTTON_RIGHT;
                }
                else if (mb->button & (1 << (SDL_BUTTON_MIDDLE - 1)))
                {
                    emouse.button |= RTGUI_MOUSE_BUTTON_MIDDLE;
                }

                if (mb->type == SDL_MOUSEBUTTONDOWN)
                {
                    emouse.button |= RTGUI_MOUSE_BUTTON_DOWN;
                    motion_button = emouse.button;

                    /* set motion timeout tick */
                    motion_tick = 50 + SDL_GetTicks();

                    if (state == 0)
                    {
                        /* send event to server */
                        rtgui_server_post_event(&emouse, sizeof(struct rtgui_event_mouse));
                        last_x = -1;
                        last_y = -1;

                        state = 1;
                    }
                }
                else
                {
                    emouse.button |= RTGUI_MOUSE_BUTTON_UP;
                    motion_button = 0;

                    if (state == 1)
                    {
                        /* send event to server */
                        rtgui_server_post_event(&emouse, sizeof(struct rtgui_event_mouse));
                        last_x = -1;
                        last_y = -1;

                        state = 0;
                    }
                }
            }
        }
        break;

        case SDL_KEYUP:
        {
            struct rtgui_event_kbd ekbd;
            ekbd.mod = event.key.keysym.mod;
            ekbd.key = event.key.keysym.sym;
            rtgui_server_post_event(&ekbd, sizeof(struct rtgui_event_mouse));

        }
        break;

        case SDL_KEYDOWN:
        {
            struct rtgui_event_kbd ekbd;
            ekbd.mod = event.key.keysym.mod;
            ekbd.key = event.key.keysym.sym;
            rtgui_server_post_event(&ekbd, sizeof(struct rtgui_event_mouse));
        }
        break;

        case SDL_QUIT:
            SDL_Quit();
            quit = 1;
            break;

        default:
            break;
        }

        if (quit)
        {
            exit(1);
            break;
        }

    }
    {
        extern void rt_hw_exit(void);
        rt_hw_exit();
    }
    return 0;
}

/* start sdl thread */
void rt_hw_sdl_start(void)
{
    HANDLE thread;
    sdl_ok_event = CreateEvent(NULL,
                               FALSE,
                               FALSE,
                               NULL);
    if (sdl_ok_event == NULL)
    {
        printf("error, create SDL event failed\n");
        exit(-1);
    }

    sdlfb_hw_init();

    /* create thread that loop sdl event */
    thread = SDL_CreateThread(sdl_loop,
                              "sdl_loop",
                              0);
    if (thread == NULL)
    {
        //Display Error Message

        return;
    }
    ResumeThread(thread);

    /* wait until SDL LCD device is registered and seted */
    WaitForSingleObject(sdl_ok_event, INFINITE);
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
