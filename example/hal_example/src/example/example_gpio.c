/**
  ******************************************************************************
  * @file   example_gpio.c
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

#include <string.h>
#include <stdlib.h>
#include "utest.h"
#include "bf0_hal.h"
#include "tc_utils.h"


/* Example Description:
 *
#ifdef BSP_USING_BOARD_MPW_EVB
 * 0. Connect GPIO_A37 with GPIO_A38, GPIO_A37 used as input pin and GPIO_A38 used as output pin
#elif defined(BSP_USING_BOARD_EC_LB551XXX)
 * 0. Connect GPIO_A01(TPRST) with GPIO_A03(TPEN), GPIO_A01 used as input pin and GPIO_A03 used as output pin
#endif
 * 1. Set output pin to low level, read input pin state, print read value 0.
 *    Delay 500ms.
 *    Set output pin to high level, read input pin state, print read value 1
 * 2. Delay 1000ms
 *    Config input pin rising edge detection, read input pin state in interrupt, print read value 1
 * 3. Delay 1000ms
 *    Config input pin falling edge detection, read input pin state in interrupt, print read value 0
 * 4. Delay 1000ms
 *    Config input pin double edge detection, read input pin state in interrupt, print read value 0
 * 5. Delay 1000ms
 *    Config input pin high level detection, read input pin state in interrupt,
 *    print read value twice, first is 1, second is 0 as interrupt is not cleared when input pin state has changed to 0
 * 6. Delay 1000ms
 *    Config input pin low level detection, read input pin state in interrupt,
 *    print read value twice, first is 0, second is 1 as interrupt is not cleared when input pin state has changed to 1
 */

#ifdef HAL_GPIO_MODULE_ENABLED

#define PAD_PA_00 PAD_PA00

static volatile uint8_t intFlag = 0;
static volatile uint32_t setPin = 0;
static volatile uint32_t setValue = 0;

#ifdef BSP_USING_BOARD_EH_LB563XXX_V2
    #define BSP_USING_BOARD_EH_LB563XXX
#endif
#ifdef BSP_USING_BOARD_EH_LB561XXX_V2
    #define BSP_USING_BOARD_EH_LB561XXX
#endif
#ifdef BSP_USING_BOARD_EC_LB567XXX_V2
    #define BSP_USING_BOARD_EC_LB567XXX
#endif /* BSP_USING_BOARD_EC_LB567XXX_V2 */
#ifdef BSP_USING_BOARD_EH_LB520XXX_V2
    #define BSP_USING_BOARD_EH_LB520XXX
#endif
#ifdef BSP_USING_BOARD_EH_LB523XXX_V2
    #define BSP_USING_BOARD_EH_LB523XXX
#endif
#ifdef BSP_USING_BOARD_EH_LB525XXX_V2
    #define BSP_USING_BOARD_EH_LB525XXX
#endif
#ifdef BSP_USING_BOARD_EH_LB551
    #define BSP_USING_BOARD_EH_SS6600XXX
#endif
#ifdef BSP_USING_BOARD_EC_LB583XXX_V2
    #define BSP_USING_BOARD_EC_LB583XXX
#endif
#ifdef BSP_USING_BOARD_EC_LB587XXX_V2
    #define BSP_USING_BOARD_EC_LB587XXX
#endif

#ifdef BSP_USING_BOARD_EH_SS6500XXX_V2
    #define BSP_USING_BOARD_EH_SS6500XXX
#endif


#if defined(BSP_USING_BOARD_EC_LB551XXX)
    static uint32_t inPin = 1;
    static uint32_t outPin = 3;
#elif defined(BSP_USING_BOARD_EH_SS6600XXX)
    static uint32_t inPin = 10;
    static uint32_t outPin = 80;
#elif defined(BSP_USING_BOARD_EC_LB555XXX)||defined(BSP_USING_BOARD_EC_LB553XXX)||defined(BSP_USING_BOARD_EC_LB557XXX)
    static uint32_t inPin = 1;
    static uint32_t outPin = 3;
#elif defined(BSP_USING_BOARD_EH_LB555)
    static uint32_t inPin = 8;
    static uint32_t outPin = 9;
#elif defined(BSP_USING_BOARD_EC_LB583XXX)||defined(BSP_USING_BOARD_EC_LB587XXX) ||defined(BSP_USING_BOARD_EC_LB585XXX)
    // TODO: review board for pins
    static uint32_t inPin = 20;
    static uint32_t outPin = 21;
#elif defined(BSP_USING_BOARD_EC_LB563XXX)||defined(BSP_USING_BOARD_EC_SS6700XXX)||defined(BSP_USING_BOARD_EC_LB567XXX)\
    ||defined(BSP_USING_BOARD_EH_SS6700XXX) || defined(BSP_USING_BOARD_EH_LB563XXX) || defined(BSP_USING_BOARD_EH_LB561XXX) \
    ||defined(BSP_USING_BOARD_EH_LB564XXX)||defined(BSP_USING_BOARD_EM_LB566XXX)
    // TODO: review board for pins
    static uint32_t inPin = 40;
    static uint32_t outPin = 41;
#elif defined(BSP_USING_BOARD_EH_LB523XXX) || defined(BSP_USING_BOARD_EH_SS6500XXX) || defined(BSP_USING_BOARD_EH_LB525XXX) \
    || defined(BSP_USING_BOARD_EH_LB520XXX)
    static uint32_t inPin = 24;
    static uint32_t outPin = 25;
#else
    static uint32_t inPin = BSP_KEY1_PIN;
    static uint32_t outPin = BSP_LED1_PIN;
#endif


static uint8_t step;


void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    GPIO_PinState value;
    //LOG_I("gpio_int_callback pin: %d", GPIO_Pin);

    if (inPin != GPIO_Pin)
    {
        return;
    }

    value = HAL_GPIO_ReadPin(hwp_gpio1, GPIO_Pin);
    LOG_I("step %d: read value %d", step + 1, value);

    /* for level trigger mode, change output pin state to avoid triggering interrupt again */
    if (0 != setPin)
    {
        HAL_GPIO_WritePin(hgpio, setPin, setValue);
        setPin = 0;
    }

    intFlag = 1;
}

void GPIO1_IRQHandler(void)
{
    ENTER_INTERRUPT();

    HAL_GPIO_IRQHandler(hwp_gpio1);

    LEAVE_INTERRUPT();
}


static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}


static void wait_4_int()
{
#if 1
    while (0 == intFlag);
#else
    rt_thread_mdelay(2000);
#endif
}

static void testcase(int argc, char **argv)
{
    uint32_t value;
    GPIO_InitTypeDef GPIO_InitStruct;

    uint32_t inMode[6] = {GPIO_MODE_INPUT, GPIO_MODE_IT_RISING, GPIO_MODE_IT_FALLING,
                          GPIO_MODE_IT_RISING_FALLING, GPIO_MODE_IT_HIGH_LEVEL, GPIO_MODE_IT_LOW_LEVEL
                         };

    HAL_PIN_Set(PAD_PA_00 + outPin, GPIO_A0 + outPin, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA_00 + inPin, GPIO_A0 + inPin, PIN_NOPULL, 1);
    HAL_PIN_SetMode(PAD_PA00 + inPin, 1, PIN_DIGITAL_IO_NORMAL);

    GPIO_InitStruct.Pin = outPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

    LOG_I("input pin: GPIO_A%d, output pin: GPIO_A%d", inPin, outPin);
    for (step = 0; step < 6; step++)
    {
        rt_thread_mdelay(1000);
        if (GPIO_MODE_INPUT == inMode[step])  /* input mode, no interrupt */
        {
            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* set output pin state to 0, i.e. low level */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 0);
            /* can read same value from input pin */
            value = HAL_GPIO_ReadPin(hwp_gpio1, inPin);
            LOG_I("step %d: read value %d", step + 1, value);
            uassert_int_equal(value, 0);

            rt_thread_mdelay(500);

            /* set output pin state to 1, i.e. high level */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 1);
            value = HAL_GPIO_ReadPin(hwp_gpio1, inPin);
            LOG_I("step %d: read value %d", step + 1, value);
            uassert_int_equal(value, 1);
        }
        else if (GPIO_MODE_IT_RISING == inMode[step]) /* input mode, rising edge detection */
        {
            /* initialize pin state to 0 */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 0);

            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable RISING edge detection */
            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = inMode[step];
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable interrupt */
            HAL_NVIC_SetPriority(GPIO1_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(GPIO1_IRQn);

            intFlag = 0;
            /* change output pin state to trigger interrupt */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 1);

            /* wait for interrupt */
            wait_4_int();

            HAL_GPIO_DeInit(hwp_gpio1, inPin);
            HAL_NVIC_DisableIRQ(GPIO1_IRQn);
        }
        else if (GPIO_MODE_IT_FALLING == inMode[step]) /* input mode, falling edge detection */
        {
            /* initialize pin state to 1 */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 1);

            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable FALLING edge detection */
            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = inMode[step];
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable interrupt */
            HAL_NVIC_SetPriority(GPIO1_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(GPIO1_IRQn);

            intFlag = 0;
            /* change output pin state to trigger interrupt */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 0);
            /* wait for interrupt */
            wait_4_int();

            HAL_GPIO_DeInit(hwp_gpio1, inPin);
            HAL_NVIC_DisableIRQ(GPIO1_IRQn);
        }
        else if (GPIO_MODE_IT_RISING_FALLING == inMode[step]) /* input mode, double edge detection */
        {
            /* initialize pin state to 1 */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 1);

            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable double edge detection */
            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = inMode[step];
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable interrupt */
            HAL_NVIC_SetPriority(GPIO1_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(GPIO1_IRQn);

            intFlag = 0;
            /* change output pin state to trigger interrupt */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 0);

            /* wait for interrupt */
            wait_4_int();
            HAL_GPIO_DeInit(hwp_gpio1, inPin);
            HAL_NVIC_DisableIRQ(GPIO1_IRQn);
        }
        else if (GPIO_MODE_IT_HIGH_LEVEL == inMode[step])  /* input mode, high level detection */
        {
            /* initialize pin state to 0 */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 0);

            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable high level detection */
            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = inMode[step];
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable interrupt */
            HAL_NVIC_SetPriority(GPIO1_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(GPIO1_IRQn);

            intFlag = 0;
            setPin = outPin;
            /* change output pin state to 0 after receiving interrupt */
            setValue = 0;
            /* change output pin state to trigger interrupt */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 1);

            /* wait for interrupt */
            wait_4_int();
            HAL_GPIO_DeInit(hwp_gpio1, inPin);
            HAL_NVIC_DisableIRQ(GPIO1_IRQn);
        }
        else if (GPIO_MODE_IT_LOW_LEVEL == inMode[step]) /* input mode, low level detection */
        {
            /* initialize pin state to 1 */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 1);

            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable low level detection */
            GPIO_InitStruct.Pin = inPin;
            GPIO_InitStruct.Mode = inMode[step];
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

            /* enable interrupt */
            HAL_NVIC_SetPriority(GPIO1_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(GPIO1_IRQn);

            intFlag = 0;
            setPin = outPin;
            /* change output pin state to 1 after receiving interrupt */
            setValue = 1;
            /* change output pin state to trigger interrupt */
            HAL_GPIO_WritePin(hwp_gpio1, outPin, 0);

            /* wait for interrupt */
            wait_4_int();
            HAL_GPIO_DeInit(hwp_gpio1, inPin);
            HAL_NVIC_DisableIRQ(GPIO1_IRQn);
        }
    }
}

#ifndef HAL_USING_HTOL
    UTEST_TC_EXPORT(testcase, "example_gpio", utest_tc_init, utest_tc_cleanup, 10);
#endif


#endif /*HAL_GPIO_MODULE_ENABLED*/
