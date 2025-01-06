/**
  ******************************************************************************
  * @file   drv_touch.h
  * @author Sifli software development team
  * @brief   This file contains all the functions prototypes for the LCD driver.
  * @attention
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

#ifndef __DRV_TOUCH_H__
#define __DRV_TOUCH_H__
#include "rtthread.h"
#include "rtdevice.h"
#include "drv_io.h"

#ifdef __cplusplus
extern "C" {
#endif


#define IIC_RETRY_NUM 2

#define TOUCH_EVENT_UP      (0x01)
#define TOUCH_EVENT_DOWN    (0x02)
#define TOUCH_EVENT_MOVE    (0x03)
#define TOUCH_EVENT_NONE    (0x80)

struct touch_message
{
    rt_uint16_t x;
    rt_uint16_t y;
    rt_uint8_t event;
};
typedef struct touch_message *touch_msg_t;

struct touch_ops
{
    rt_err_t (* read_point)(touch_msg_t); //Retrun RT_EOK if more points is ready to read
    rt_err_t (* init)(void);
    rt_err_t (* deinit)(void);
};
typedef struct touch_ops *touch_ops_t;

struct touch_drivers
{
    rt_list_t       list;
    rt_bool_t (*probe)(void);
    rt_sem_t        isr_sem;
    touch_ops_t     ops;
    void           *user_data;
};
typedef struct touch_drivers *touch_drv_t;

extern void rt_touch_drivers_register(touch_drv_t drv);
extern rt_err_t rt_touch_irq_pin_attach(rt_uint32_t mode, void (*hdr)(void *args), void  *args);
extern rt_err_t rt_touch_irq_pin_detach(void);
extern rt_err_t rt_touch_irq_pin_enable(rt_uint32_t enabled);


#ifdef __cplusplus
}
#endif
#endif
