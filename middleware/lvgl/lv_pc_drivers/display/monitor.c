/**
  ******************************************************************************
  * @file   monitor.c
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

#include "monitor.h"
#if USE_MONITOR

#ifndef MONITOR_SDL_INCLUDE_PATH
    #define MONITOR_SDL_INCLUDE_PATH <SDL2/SDL.h>
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include MONITOR_SDL_INCLUDE_PATH
#include "../indev/mouse.h"
#include "../indev/keyboard.h"
#include "../indev/mousewheel.h"

/*********************
 *      DEFINES
 *********************/
#define SDL_REFR_PERIOD     10  /*ms*/

#ifndef MONITOR_ZOOM
    #define MONITOR_ZOOM        1
#endif

#if defined(__APPLE__) && defined(TARGET_OS_MAC)
    #if __APPLE__ && TARGET_OS_MAC
        #define MONITOR_APPLE
    #endif
#endif

#if defined(__EMSCRIPTEN__)
    #define MONITOR_EMSCRIPTEN
#endif

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    volatile bool sdl_refr_qry;
#if MONITOR_DOUBLE_BUFFERED
    uint32_t *tft_fb_act;
#else
    uint32_t tft_fb[LV_HOR_RES_MAX * LV_VER_RES_MAX];
#endif
} monitor_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static int monitor_sdl_refr_thread(void *param);
static void window_create(monitor_t *m);
static void window_update(monitor_t *m);

/***********************
 *   GLOBAL PROTOTYPES
 ***********************/

/**********************
 *  STATIC VARIABLES
 **********************/
monitor_t monitor;

struct rt_semaphore ut_wait_sema;

#if MONITOR_DUAL
    monitor_t monitor2;
#endif

static volatile bool sdl_inited = false;
static volatile bool sdl_quit_qry = false;

static char *title_hint;
static Sint32 mouse_x;           /**< X coordinate, relative to window */
static Sint32 mouse_y;           /**< Y coordinate, relative to window */

int quit_filter(void *userdata, SDL_Event *event);
static void monitor_sdl_clean_up(void);
static void monitor_sdl_init(void);
#ifdef MONITOR_EMSCRIPTEN
    void monitor_sdl_refr_core(void); /* called from Emscripten loop */
#else
    static void monitor_sdl_refr_core(void);
#endif

static void monitor_update_window_title(void);

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ut_wait(size_t line, const char *hint)
{
    monitor_update_title_hint(hint);
    rt_sem_take(&ut_wait_sema, RT_WAITING_FOREVER);
}

int ut_continue(void)
{
    rt_sem_release(&ut_wait_sema);
    return 0;
}

void monitor_update_title_hint(const char *s)
{
    if (title_hint)
    {
        rt_free(title_hint);
    }
    title_hint = rt_malloc(strlen(s) + 1);
    RT_ASSERT(title_hint);
    title_hint[0] = 0;
    strcat(title_hint, s);

    monitor_update_window_title();

}

/**
 * Initialize the monitor
 */
void monitor_init(void)
{

    rt_sem_init(&ut_wait_sema, "ut", 0, RT_IPC_FLAG_FIFO);

    /*OSX needs to initialize SDL here*/
#if defined(MONITOR_APPLE) || defined(MONITOR_EMSCRIPTEN)
    monitor_sdl_init();
#endif

#ifndef MONITOR_EMSCRIPTEN
    SDL_CreateThread(monitor_sdl_refr_thread, "sdl_refr", NULL);
    while (sdl_inited == false); /*Wait until 'sdl_refr' initializes the SDL*/
#endif
}

/**
 * Flush a buffer to the marked area
 * @param drv pointer to driver where this function belongs
 * @param area an area where to copy `color_p`
 * @param color_p an array of pixel to copy to the `area` part of the screen
 */
void monitor_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    lv_coord_t hres = disp_drv->rotated == 0 ? disp_drv->hor_res : disp_drv->ver_res;
    lv_coord_t vres = disp_drv->rotated == 0 ? disp_drv->ver_res : disp_drv->hor_res;

//    printf("x1:%d,y1:%d,x2:%d,y2:%d\n", area->x1, area->y1, area->x2, area->y2);

    /*Return if the area is out the screen*/
    if (area->x2 < 0 || area->y2 < 0 || area->x1 > hres - 1 || area->y1 > vres - 1)
    {

        lv_disp_flush_ready(disp_drv);
        return;
    }

#if MONITOR_DOUBLE_BUFFERED
    monitor.tft_fb_act = (uint32_t *)color_p;

    monitor.sdl_refr_qry = true;

    /*IMPORTANT! It must be called to tell the system the flush is ready*/
    lv_disp_flush_ready(disp_drv);
#else

    int32_t y;
    lv_coord_t vdb_width = disp_drv->buffer->area.x2 - disp_drv->buffer->area.x1 + 1;
#if LV_COLOR_DEPTH != 32    /*32 is valid but support 24 for backward compatibility too*/
    int32_t x;
    for (y = area->y1; y <= area->y2; y++)
    {
        for (x = area->x1; x <= area->x2; x++)
        {
            monitor.tft_fb[y * MONITOR_HOR_RES + x] = lv_color_to32(color_p[(y - area->y1) * vdb_width + (x - area->x1)]);
        }

    }
#else
    uint32_t w = lv_area_get_width(area);
    color_p += area->y1 * vdb_width;
    for (y = area->y1; y <= area->y2 && y < MONITOR_VER_RES; y++)
    {
        memcpy(&monitor.tft_fb[y * MONITOR_HOR_RES + area->x1], &color_p[0], w * sizeof(lv_color_t));
        color_p += vdb_width;
    }
#endif

    monitor.sdl_refr_qry = true;

    /*IMPORTANT! It must be called to tell the system the flush is ready*/
    lv_disp_flush_ready(disp_drv);
#endif
}


#if MONITOR_DUAL

/**
 * Flush a buffer to the marked area
 * @param drv pointer to driver where this function belongs
 * @param area an area where to copy `color_p`
 * @param color_p an array of pixel to copy to the `area` part of the screen
 */
void monitor_flush2(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    lv_coord_t hres = disp_drv->rotated == 0 ? disp_drv->hor_res : disp_drv->ver_res;
    lv_coord_t vres = disp_drv->rotated == 0 ? disp_drv->ver_res : disp_drv->hor_res;

    /*Return if the area is out the screen*/
    if (area->x2 < 0 || area->y2 < 0 || area->x1 > hres - 1 || area->y1 > vres - 1)
    {
        lv_disp_flush_ready(disp_drv);
        return;
    }

#if MONITOR_DOUBLE_BUFFERED
    monitor2.tft_fb_act = (uint32_t *)color_p;

    monitor2.sdl_refr_qry = true;

    /*IMPORTANT! It must be called to tell the system the flush is ready*/
    lv_disp_flush_ready(disp_drv);
#else

    int32_t y;
#if LV_COLOR_DEPTH != 32    /*32 is valid but support 24 for backward compatibility too*/
    int32_t x;
    for (y = area->y1; y <= area->y2; y++)
    {
        for (x = area->x1; x <= area->x2; x++)
        {
            monitor2.tft_fb[y * MONITOR_HOR_RES + x] = lv_color_to32(*color_p);
            color_p++;
        }

    }
#else
    uint32_t w = lv_area_get_width(area);
    for (y = area->y1; y <= area->y2 && y < MONITOR_VER_RES; y++)
    {
        memcpy(&monitor2.tft_fb[y * MONITOR_HOR_RES + area->x1], color_p, w * sizeof(lv_color_t));
        color_p += w;
    }
#endif

    monitor2.sdl_refr_qry = true;

    /*IMPORTANT! It must be called to tell the system the flush is ready*/
    lv_disp_flush_ready(disp_drv);
#endif
}
#endif

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * SDL main thread. All SDL related task have to be handled here!
 * It initializes SDL, handles drawing and the mouse.
 */

static int monitor_sdl_refr_thread(void *param)
{
    (void)param;

    /*If not OSX initialize SDL in the Thread*/
#ifndef MONITOR_APPLE
    monitor_sdl_init();
#endif
    /*Run until quit event not arrives*/
    while (sdl_quit_qry == false)
    {
        /*Refresh handling*/
        monitor_sdl_refr_core();
    }

    monitor_sdl_clean_up();
    exit(0);

    return 0;
}

int quit_filter(void *userdata, SDL_Event *event)
{
    (void)userdata;

    if (event->type == SDL_WINDOWEVENT)
    {
        if (event->window.event == SDL_WINDOWEVENT_CLOSE)
        {
            sdl_quit_qry = true;
        }
    }
    else if (event->type == SDL_QUIT)
    {
        sdl_quit_qry = true;
    }



    return 1;
}

static void monitor_sdl_clean_up(void)
{
    SDL_DestroyTexture(monitor.texture);
    SDL_DestroyRenderer(monitor.renderer);
    SDL_DestroyWindow(monitor.window);

#if MONITOR_DUAL
    SDL_DestroyTexture(monitor2.texture);
    SDL_DestroyRenderer(monitor2.renderer);
    SDL_DestroyWindow(monitor2.window);

#endif

    SDL_Quit();
}

static void monitor_update_window_title(void)
{
    char title_buff[200];

    memset(title_buff, 0, sizeof(title_buff));
    if (title_hint)
    {
        rt_snprintf(title_buff, sizeof(title_buff), "%d,%d: %s", mouse_x, mouse_y, title_hint);
    }
    else
    {
        rt_snprintf(title_buff, sizeof(title_buff), "%d,%d", mouse_x, mouse_y);
    }

    SDL_SetWindowTitle(monitor.window, title_buff);
}

static void monitor_sdl_init(void)
{
    /*Initialize the SDL*/
    SDL_Init(SDL_INIT_VIDEO);

    SDL_SetEventFilter(quit_filter, NULL);

    window_create(&monitor);
#if MONITOR_DUAL
    window_create(&monitor2);
    int x, y;
    SDL_GetWindowPosition(monitor2.window, &x, &y);
    SDL_SetWindowPosition(monitor.window, x + MONITOR_HOR_RES / 2 + 10, y);
    SDL_SetWindowPosition(monitor2.window, x - MONITOR_HOR_RES / 2 - 10, y);
#endif

    sdl_inited = true;
}

#ifdef MONITOR_EMSCRIPTEN
    void monitor_sdl_refr_core(void)
#else
    static void monitor_sdl_refr_core(void)
#endif
{

    if (monitor.sdl_refr_qry != false)
    {
        monitor.sdl_refr_qry = false;
        window_update(&monitor);
    }

#if MONITOR_DUAL
    if (monitor2.sdl_refr_qry != false)
    {
        monitor2.sdl_refr_qry = false;
        window_update(&monitor2);
    }
#endif

#if !defined(MONITOR_APPLE) && !defined(MONITOR_EMSCRIPTEN)
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
#if USE_MOUSE != 0
        mouse_handler(&event);
#endif

#if USE_MOUSEWHEEL != 0
        mousewheel_handler(&event);
#endif

#if USE_KEYBOARD
        keyboard_handler(&event);
#endif
        if ((&event)->type == SDL_WINDOWEVENT)
        {
            switch ((&event)->window.event)
            {
#if SDL_VERSION_ATLEAST(2, 0, 5)
            case SDL_WINDOWEVENT_TAKE_FOCUS:
#endif
            case SDL_WINDOWEVENT_EXPOSED:
                window_update(&monitor);
#if MONITOR_DUAL
                window_update(&monitor2);
#endif
                break;
            default:
                break;
            }
        }
        else if ((&event)->type == SDL_MOUSEMOTION)
        {
            mouse_x = event.motion.x;
            mouse_y = event.motion.y;
            monitor_update_window_title();
        }
    }
#endif /*MONITOR_APPLE*/

    /*Sleep some time*/
    SDL_Delay(SDL_REFR_PERIOD);

}

static void window_create(monitor_t *m)
{
    m->window = SDL_CreateWindow("TFT Simulator",
                                 SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                 MONITOR_HOR_RES * MONITOR_ZOOM, MONITOR_VER_RES * MONITOR_ZOOM, 0);       /*last param. SDL_WINDOW_BORDERLESS to hide borders*/

#if MONITOR_VIRTUAL_MACHINE || defined(MONITOR_EMSCRIPTEN)
    m->renderer = SDL_CreateRenderer(m->window, -1, SDL_RENDERER_SOFTWARE);
#else
    m->renderer = SDL_CreateRenderer(m->window, -1, 0);
#endif
    m->texture = SDL_CreateTexture(m->renderer,
                                   SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, MONITOR_HOR_RES, MONITOR_VER_RES);
    SDL_SetTextureBlendMode(m->texture, SDL_BLENDMODE_BLEND);

    /*Initialize the frame buffer to gray (77 is an empirical value) */
#if MONITOR_DOUBLE_BUFFERED
    SDL_UpdateTexture(m->texture, NULL, m->tft_fb_act, MONITOR_HOR_RES * sizeof(uint32_t));
#else
    memset(m->tft_fb, 0x44, MONITOR_HOR_RES * MONITOR_VER_RES * sizeof(uint32_t));
#endif

    m->sdl_refr_qry = true;

}

static void window_update(monitor_t *m)
{
#if MONITOR_DOUBLE_BUFFERED == 0
    SDL_UpdateTexture(m->texture, NULL, m->tft_fb, MONITOR_HOR_RES * sizeof(uint32_t));
#else
    if (m->tft_fb_act == NULL) return;
    SDL_UpdateTexture(m->texture, NULL, m->tft_fb_act, MONITOR_HOR_RES * sizeof(uint32_t));
#endif
    SDL_RenderClear(m->renderer);
    /*Test: Draw a background to test transparent screens (LV_COLOR_SCREEN_TRANSP)*/
    //        SDL_SetRenderDrawColor(renderer, 0xff, 0, 0, 0xff);
    //        SDL_Rect r;
    //        r.x = 0; r.y = 0; r.w = MONITOR_HOR_RES; r.w = MONITOR_VER_RES;
    //        SDL_RenderDrawRect(renderer, &r);

    /*Update the renderer with the texture containing the rendered image*/
    SDL_RenderCopy(m->renderer, m->texture, NULL, NULL);
    SDL_RenderPresent(m->renderer);
}

#endif /*USE_MONITOR*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
