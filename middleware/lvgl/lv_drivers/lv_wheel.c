/**
  ******************************************************************************
  * @file   lv_wheel.c
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
#include "rtthread.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "board.h"

static lv_indev_drv_t indev_drv;
static rt_device_t wheel_device = NULL;

static void input_read(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data)   //(lv_indev_data_t *data)//(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data
{

    if (wheel_device)
    {
        int16_t wheel_data;

        if (2 == rt_device_read(wheel_device, 0, &wheel_data, 2))
        {
            data->enc_diff = wheel_data;

            wheel_do_event(data->enc_diff, data->state);
        }
    }

#if 0//def BSP_USING_LVGL_INPUT_AGENT
    {
        extern int lv_indev_agent_filter(lv_indev_drv_t *drv, lv_indev_data_t *data);
        lv_indev_agent_filter(indev_drv, data);
    }
#endif

}

static lv_indev_t *wheel_indev;

void wheel_init(void)
{
    /*Open wheel device*/
    RT_ASSERT(NULL == wheel_device);
    wheel_device = rt_device_find("wheel");
    if (wheel_device)
    {
        if (RT_EOK == rt_device_open(wheel_device, RT_DEVICE_FLAG_RDONLY))
        {
            //slv_indev_t *dev;
            lv_indev_drv_init(&indev_drv);

            indev_drv.type = LV_INDEV_TYPE_ENCODER;
            indev_drv.read_cb = input_read;
            wheel_indev = lv_indev_drv_register(&indev_drv);
        }
        else
        {
            wheel_device = NULL;
            wheel_indev = NULL;
        }
    }


#if 0//def BSP_USING_LVGL_INPUT_AGENT
    {
        extern void lv_indev_agent_init(lv_indev_drv_t *drv);
        lv_indev_agent_init(&wheel_indev->driver);
    }
#endif

}

lv_indev_t *wheel_get_indev_handler(void)
{
    return wheel_indev;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
