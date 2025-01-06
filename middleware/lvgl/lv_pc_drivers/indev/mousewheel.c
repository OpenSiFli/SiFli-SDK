/**
  ******************************************************************************
  * @file   mousewheel.c
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

#include "mousewheel.h"
#include "lvsf_input.h"

#if USE_MOUSEWHEEL

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static int16_t enc_diff = 0;
static lv_indev_state_t state = LV_INDEV_STATE_REL;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize the mousewheel
 */
void mousewheel_init(void)
{
    /*Nothing to init*/
}

/**
 * Get encoder (i.e. mouse wheel) ticks difference and pressed state
 * @param indev_drv pointer to the related input device driver
 * @param data store the read data here
 * @return false: all ticks and button state are handled
 */
bool mousewheel_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    (void) indev_drv;      /*Unused*/

    if ((data->state != state) || (data->enc_diff != enc_diff))
    {
        data->state = state;
        data->enc_diff = enc_diff;
        enc_diff = 0;

        wheel_do_event(data->enc_diff, data->state);
    }
    return false;       /*No more data to read so return false*/
}

/**
 * It is called periodically from the SDL thread to check mouse wheel state
 * @param event describes the event
 */
void mousewheel_handler(SDL_Event *event)
{
    switch (event->type)
    {
    case SDL_MOUSEWHEEL:
        // Scroll down (y = -1) means positive encoder turn,
        // so invert it
#ifdef __EMSCRIPTEN__
        /*Escripten scales it wrong*/
        if (event->wheel.y < 0) enc_diff++;
        if (event->wheel.y > 0) enc_diff--;
#else
        enc_diff = -event->wheel.y;
#endif
        break;
    case SDL_MOUSEBUTTONDOWN:
        if (event->button.button == SDL_BUTTON_MIDDLE)
        {
            state = LV_INDEV_STATE_PR;
        }
        break;
    case SDL_MOUSEBUTTONUP:
        if (event->button.button == SDL_BUTTON_MIDDLE)
        {
            state = LV_INDEV_STATE_REL;
        }
        break;
    default:
        break;
    }
}



/**********************
 *   GLOBAL FUNCTIONS
 **********************/


/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
