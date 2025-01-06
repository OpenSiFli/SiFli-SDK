/**
  ******************************************************************************
  * @file   lv_keypad.c
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

#include "rtconfig.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "board.h"


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void keypad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static uint32_t keycode_to_ascii(uint32_t button);

#ifdef BSP_USING_BUTTON
    extern bool button_read(uint32_t *key, uint8_t *state);
#endif
/**********************
 *  STATIC VARIABLES
 **********************/

static lv_indev_drv_t kb_drv;
static lv_indev_t *kb_indev;
static rt_device_t g_t_button;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void button_key_read(uint32_t *last_key, lv_indev_state_t *state);


/**
 * Initialize the keyboard
 */
void keypad_init(void)
{
#if 0
    g_t_button = rt_device_find("keypad");
    if (NULL == g_t_button)
    {
        kb_indev = NULL;
        LV_LOG_WARN("keypad_init can't find keypad");
    }
    else
    {
        lv_indev_drv_t kb_drv;
        rt_device_init(g_t_button);
        rt_device_open(g_t_button, RT_DEVICE_OFLAG_RDONLY);

        lv_indev_drv_init(&kb_drv);
        kb_drv.type = LV_INDEV_TYPE_KEYPAD;
        kb_drv.read_cb = keypad_read;
        kb_indev = lv_indev_drv_register(&kb_drv);

#ifdef BSP_USING_LVGL_INPUT_AGENT
        {
            extern void lv_indev_agent_init(lv_indev_drv_t *drv);
            lv_indev_agent_init(&kb_indev->driver);
        }
#endif
    }
#else


    lv_indev_drv_init(&kb_drv);
    kb_drv.type = LV_INDEV_TYPE_KEYPAD;
    kb_drv.read_cb = keypad_read;
    kb_indev = lv_indev_drv_register(&kb_drv);

#ifdef BSP_USING_LVGL_INPUT_AGENT
    {
        extern void lv_indev_agent_init(lv_indev_drv_t *drv);
        lv_indev_agent_init(&kb_drv);
    }
#endif


#endif

}


lv_indev_t *keypad_get_indev_handler(void)
{
    return kb_indev;
}

/**
 * Convert the key code LV_KEY_... "codes" or leave them if they are not control characters
 * @param button the key code
 * @return
 */
static uint32_t keycode_to_ascii(uint32_t button)
{
    uint32_t ret_value;

    switch (button)
    {
    case 1:
        ret_value = LV_KEY_ENTER;
        break;

    case 2:
        ret_value = LV_KEY_HOME;
        break;

    case 3:
        ret_value =  LV_KEY_NEXT;
        break;

    default:
        ret_value = button;
        break;
    }

    return ret_value;
}

RT_WEAK void button_key_read(uint32_t *last_key, lv_indev_state_t *state)
{
}

/**
 * Get the last pressed or released character from keypad
 * @param indev_drv pointer to the related input device driver
 * @param data store the read data here
 * @return false: because the points are not buffered, so no more data to be read
 */
static void keypad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    bool more = false;
#if 0
    uint32_t buffer[2];
#endif
    uint32_t last_key = 0;
    lv_indev_state_t state = LV_INDEV_STATE_REL;

    (void) indev_drv;      /*Unused*/

#if 0
    if (g_t_button)
    {
        rt_device_read(g_t_button, 0, buffer, sizeof(buffer));
    }
    else
    {
        while (1);
    }

    last_key = buffer[0];
    state = buffer[1];
#else
    button_key_read(&last_key, &state);
#endif


    data->state = state;
    data->key = keycode_to_ascii(last_key);

#ifdef BSP_USING_LVGL_INPUT_AGENT
    {
        extern int lv_indev_agent_filter(lv_indev_drv_t *drv, lv_indev_data_t *data);
        lv_indev_agent_filter(indev_drv, data);
    }
#endif

    keypad_do_event(data->key, data->state);

}


/**********************
 *   GLOBAL FUNCTIONS
 **********************/


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
