/**
  ******************************************************************************
  * @file   keyboard.c
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

#include "keyboard.h"
#include "lvsf_input.h"
#if USE_KEYBOARD

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static uint32_t keycode_to_ascii(uint32_t sdl_key);

/**********************
 *  STATIC VARIABLES
 **********************/
static uint32_t last_key;
static lv_indev_state_t state;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize the keyboard
 */
void keyboard_init(void)
{
    /*Nothing to init*/
}

/**
 * Get the last pressed or released character from the PC's keyboard
 * @param indev_drv pointer to the related input device driver
 * @param data store the read data here
 * @return false: because the points are not buffered, so no more data to be read
 */
bool keyboard_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    (void) indev_drv;      /*Unused*/
    data->state = state;
    data->key = keycode_to_ascii(last_key);
#ifdef BSP_USING_LVGL_INPUT_AGENT
    {
        extern int lv_indev_agent_filter(lv_indev_drv_t *drv, lv_indev_data_t *data);
        lv_indev_agent_filter(indev_drv, data);
    }
#endif

    keypad_do_event(data->key, data->state);

    return false;
}

/**
 * It is called periodically from the SDL thread to check a key is pressed/released
 * @param event describes the event
 */
void keyboard_handler(SDL_Event *event)
{
    /* We only care about SDL_KEYDOWN and SDL_KEYUP events */
    switch (event->type)
    {
    case SDL_KEYDOWN:                       /*Button press*/
        last_key = event->key.keysym.sym;   /*Save the pressed key*/
        state = LV_INDEV_STATE_PR;          /*Save the key is pressed now*/
        break;
    case SDL_KEYUP:                         /*Button release*/
        state = LV_INDEV_STATE_REL;         /*Save the key is released but keep the last key*/
        break;
    default:
        break;

    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Convert the key code LV_KEY_... "codes" or leave them if they are not control characters
 * @param sdl_key the key code
 * @return
 */
static uint32_t keycode_to_ascii(uint32_t sdl_key)
{
    /*Remap some key to LV_KEY_... to manage groups*/
    switch (sdl_key)
    {
    case SDLK_RIGHT:
    case SDLK_KP_PLUS:
        return LV_KEY_RIGHT;

    case SDLK_LEFT:
    case SDLK_KP_MINUS:
        return LV_KEY_LEFT;

    case SDLK_UP:
        return LV_KEY_UP;

    case SDLK_DOWN:
        return LV_KEY_DOWN;

    case SDLK_ESCAPE:
        return LV_KEY_ESC;

    case SDLK_HOME:
        return LV_KEY_HOME;

#ifdef  LV_KEY_BACKSPACE        /*For backward compatibility*/
    case SDLK_BACKSPACE:
        return LV_KEY_BACKSPACE;
#endif

#ifdef  LV_KEY_DEL        /*For backward compatibility*/
    case SDLK_DELETE:
        return LV_KEY_DEL;
#endif
    case SDLK_KP_ENTER:
    case '\r':
        return LV_KEY_ENTER;
    case SDLK_PAGEDOWN:
        return LV_KEY_NEXT;
    default:
        return sdl_key;
    }
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
