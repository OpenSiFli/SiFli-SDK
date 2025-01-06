/**
  ******************************************************************************
  * @file   drv_gpio.h
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

#ifndef __DRV_GPIO_H__
#define __DRV_GPIO_H__

#include <drv_common.h>

#define __GPIO_INSTANCE(GPIOx)  (GPIO##GPIOx##_BASE)

/** get driver pin id by instance id and its pin id
 *
 * e.g. GET_PIN(1, 0) for GPIO1 pin0 -->  driver pin id is 0
 *      GET_PIN(2, 0) for GPIO2 pin0 -->  driver pin id is 96
 *
 * @param  GPIOx GPIO instance id, starting from 0, 0 is for PBR, 1 is for GPIO1, 2 is for GPIO2, etc.
 * @param  PIN GPIO instance pin id, starting from 0
 * @return driver pin id, counting from 0
 */
#define GET_PIN(GPIOx,PIN)  ((GPIOx != 0) ? ((__GPIO_INSTANCE(GPIOx) == GPIO2_BASE) ? (GPIO1_PIN_NUM + (PIN)) : (PIN)) : (GPIO1_PIN_NUM + GPIO2_PIN_NUM + (PIN)))

#ifdef hwp_pbr
    /** get driver pin id by instance id and its pin id
    *
    * e.g. GET_PIN_2(hwp_gpio1, 0) for GPIO1 pin0 -->  driver pin id is 0
    *      GET_PIN_2(hwp_gpio2, 0) for GPIO2 pin0 -->  driver pin id is 96
    *
    * @param  GPIOx GPIO instance, hwp_pbr for PBR, hwp_gpio1 for GPIO1, hwp_gpio2 for GPIO2
    * @param  PIN GPIO instance pin id, starting from 0
    * @return driver pin id, counting from 0
    */
    #define GET_PIN_2(GPIOx,PIN)  ((GPIOx != hwp_pbr) ? (((GPIOx) == hwp_gpio2) ? (GPIO1_PIN_NUM + (PIN)) : (PIN)) : (GPIO1_PIN_NUM + GPIO2_PIN_NUM + (PIN)))

    /** get GPIO instance according to driver pin  */
    #define GET_GPIO_INSTANCE(PIN)  ((PIN) >= GPIO1_PIN_NUM ? (((PIN) < (GPIO1_PIN_NUM + GPIO2_PIN_NUM)) ? hwp_gpio2 : hwp_pbr) : hwp_gpio1)

    /** get GPIO instance pin id according to driver pin id */
    #define GET_GPIOx_PIN(PIN) ((PIN) >= GPIO1_PIN_NUM ? (((PIN) < (GPIO1_PIN_NUM + GPIO2_PIN_NUM)) ? ((PIN) - GPIO1_PIN_NUM) : ((PIN) - GPIO1_PIN_NUM - GPIO2_PIN_NUM)) : (PIN))

#else
    #define GET_PIN_2(GPIOx,PIN)  (((GPIOx) == hwp_gpio2) ? (GPIO1_PIN_NUM + (PIN)) : (PIN))

    #define GET_GPIO_INSTANCE(PIN)  ((PIN) >= GPIO1_PIN_NUM ? hwp_gpio2 : hwp_gpio1)

    #define GET_GPIOx_PIN(PIN) ((PIN) >= GPIO1_PIN_NUM ? (PIN) - GPIO1_PIN_NUM : (PIN))
#endif /* hwp_pbr */


int rt_hw_pin_init(void);

/**
 * @brief Trigger pin state change check
 *
 * @return void
 */
void drv_pin_check(void);

void drv_pin_irq_from_wsr(uint32_t wsr_pins);


#endif /* __DRV_GPIO_H__ */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
