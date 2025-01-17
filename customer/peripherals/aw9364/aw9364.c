/**
  ******************************************************************************
  * @file   aw9364.c
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
#include "board.h"
#include "string.h"

/* Define -------------------------------------------------------------------*/

#define DRV_DEBUG
#define LOG_TAG              "drv.bl"
#include <drv_log.h>

/* macro ------------------------------------------------------------------*/
//larger than 20 us, set to 50
#define AW9364_READYTIME_TON               (50)

// larger than 0.5us ,set to 50
#define AW9364_PULSE_THI                   (50)

// larger than 0.5us and less than 500us, set to 50
#define AW9364_PULSE_TLO                   (50)

// larger than 2.5ms
#define AW9364_SHUTDONW_TSHDN              (3000)


#define AW9364_LIGHT_MAX_LEVEL             (16)


/* function and value-----------------------------------------------------------*/

static struct rt_device aw9364_device;

static uint8_t aw9364_bl;       // save local bl, check previous level
//Compatible with old .config&rt_config.h, remove me if all .config&rt_config.h updated.
#ifndef LCD_BACKLIGHT_CONTROL_PIN
    #define LCD_BACKLIGHT_CONTROL_PIN AW9364_LIN_IO
#endif

#define aw9364_wait_us(us)          HAL_Delay_us(us)
#define aw9364_switch_level(level)   rt_pin_write(LCD_BACKLIGHT_CONTROL_PIN, level)



static int aw9364_init(void)
{
    rt_err_t ret = RT_EOK;

    rt_pin_mode(LCD_BACKLIGHT_CONTROL_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LCD_BACKLIGHT_CONTROL_PIN, 0);

    // Init to 0xff, diff normal bl set and boot up
    aw9364_bl = 0xff;

    return 0;
}


static void aw9364_set_readytime()
{
    // SET TO HIGH FIRST to avoid from shut donw status
    aw9364_switch_level(1);

    // start a timer with ready duration
    aw9364_wait_us(AW9364_READYTIME_TON);

#if 0
    // set to low
    ret = aw9364_switch_level(0);
    if (ret != 0)
        LOG_E("set leve fail\n");

    // start a timer with pulse low duration
    ret = aw9364_wait_us(AW9364_PULSE_TLO);
    if (ret != 0)
        LOG_E("TIME OUT\n");

    // SET TO HIGH , always keep high if opened
    ret = aw9364_switch_level(1);
    if (ret != 0)
        LOG_E("set leve fail\n");

#endif
}

// output high
static void aw9364_set_shutdonw()
{
    aw9364_switch_level(0);
    // start a timer with ready duration
    aw9364_wait_us(AW9364_SHUTDONW_TSHDN);
}

// input high, output high
static void aw9364_set_pulse_normal()
{
    // set to low, default input should be high
    aw9364_switch_level(0);

    // start a timer with normal pulse duration
    aw9364_wait_us(AW9364_PULSE_TLO);
    // SET TO HIGH , always keep high if opened
    aw9364_switch_level(1);

    // wait thi
    aw9364_wait_us(AW9364_PULSE_THI);
}

/* output interface -----------------------------------------------------------*/
static int sif_aw9364_set_backlight(uint8_t backlight)
{
    int ret = 0;

    if (aw9364_bl != 0xff)
    {
        aw9364_set_shutdonw();
    }

    if (backlight > 0)
    {
        if (backlight <= AW9364_LIGHT_MAX_LEVEL)
        {
            uint8_t bl = AW9364_LIGHT_MAX_LEVEL - backlight;

            // On time, need only one puse
            aw9364_set_readytime();

            if (bl > 0)
            {
                int i;
                for (i = 0; i < bl; i++)
                    aw9364_set_pulse_normal();
            }
        }
        else
        {
            aw9364_set_readytime();
            backlight = AW9364_LIGHT_MAX_LEVEL;
        }
    }
    else
    {
        aw9364_set_shutdonw();
    }

    aw9364_bl = backlight;

    return ret;
}

static rt_size_t backligt_get(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    if (buffer != NULL)
    {
        *((uint8_t *) buffer)  =  aw9364_bl * 100 / AW9364_LIGHT_MAX_LEVEL;

        return 1;
    }

    return 0;
}

static rt_size_t backligt_set(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    if (buffer != NULL)
    {
        uint8_t percent = *((uint8_t *) buffer);

        if (percent > 100) percent = 100;

        sif_aw9364_set_backlight(((uint16_t)percent) * AW9364_LIGHT_MAX_LEVEL / 100);

        return 1;
    }

    return 0;

}

static int sif_aw9364_init(void)
{

    memset(&aw9364_device, 0, sizeof(aw9364_device));

    aw9364_device.type = RT_Device_Class_Char;
    aw9364_device.read = backligt_get;
    aw9364_device.write = backligt_set;



    /* register graphic device driver */
    rt_device_register(&aw9364_device, "lcdlight",
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return aw9364_init();
}

INIT_COMPONENT_EXPORT(sif_aw9364_init);

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
