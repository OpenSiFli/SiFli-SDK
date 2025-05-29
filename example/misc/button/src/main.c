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
#include <string.h>
#include "button.h"
#include <board.h>

/*
    action:
    BUTTON_PRESSED  = 0,    Indicates that a button is pressed
    BUTTON_RELEASED = 1,    Indicates that a button is released
    BUTTON_LONG_PRESSED = 2, Indicates that a button is long released
    BUTTON_CLICKED  = 3,     Indicates that a button is clicked
*/
static void button_event_handler(int32_t pin, button_action_t action)
{
    rt_kprintf("button:%d,%d\n", pin, action);
}

int main(void)
{
#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLDOWN, 1);
#elif defined (BSP_USING_BOARD_SF32LB58_LCD_N16R64N4)
    HAL_PIN_Set(PAD_PB56, GPIO_B56, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PB54, GPIO_B54, PIN_PULLDOWN, 1);
#endif
    /*
    587 board PB pin needs to add 96
    54+96->150
    56+96->152
    */

    button_cfg_t cfg;

#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    cfg.pin = 34;
#elif defined (BSP_USING_BOARD_SF32LB58_LCD_N16R64N4)
    cfg.pin = 152;
#endif
    cfg.active_state = BUTTON_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));


#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    cfg.pin = 11;
#elif defined (BSP_USING_BOARD_SF32LB58_LCD_N16R64N4)
    cfg.pin = 150;
#endif
    cfg.active_state = BUTTON_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));

    while (1)
    {
        rt_thread_mdelay(1000000);
    }

    return RT_EOK;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

