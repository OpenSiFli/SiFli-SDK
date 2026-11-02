/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"

#ifdef SF32LB52X
    #define Pin_Out 41
    #define Pin_In 42
    #define GPIO_IRQn GPIO1_IRQn
    #define hwp_gpio hwp_gpio1
    #define RCC_MOD_GPIO RCC_MOD_GPIO1
#elif defined(SF32LB58X)
    #define Pin_Out 28
    #define Pin_In 29
    #define GPIO_IRQn GPIO2_IRQn
    #define hwp_gpio hwp_gpio2
    #define RCC_MOD_GPIO RCC_MOD_GPIO2
#elif defined(SF32LB56X)
    #define Pin_Out 20
    #define Pin_In 12
    #define GPIO_IRQn GPIO1_IRQn
    #define hwp_gpio hwp_gpio1
    #define RCC_MOD_GPIO RCC_MOD_GPIO1
#elif defined(SF32LB57X)
    #define Pin_Out 41
    #define Pin_In 42
    #define GPIO_IRQn GPIO1_IRQn
    #define hwp_gpio hwp_gpio1
    #define RCC_MOD_GPIO RCC_MOD_GPIO1
#endif

void delayms(unsigned short int ms)
{
    HAL_Delay(ms);
}

/// @brief Initialization work before power on EEPROM
/// @param
void gpio_init(void)
{
    //1. pin mux
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA00 + Pin_Out, GPIO_A0 + Pin_Out, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA00 + Pin_In, GPIO_A0 + Pin_In, PIN_PULLDOWN, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PB00 + Pin_Out, GPIO_B0 + Pin_Out, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB00 + Pin_In, GPIO_B0 + Pin_In, PIN_PULLDOWN, 0);
#elif defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA00 + Pin_Out, GPIO_A0 + Pin_Out, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA00 + Pin_In, GPIO_A0 + Pin_In, PIN_PULLDOWN, 1);
#elif defined(SF32LB57X)
    HAL_PIN_Set(PAD_PA00 + Pin_Out, GPIO_A0 + Pin_Out, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA00 + Pin_In, GPIO_A0 + Pin_In, PIN_PULLDOWN, 1);
#endif

    // 2. gpio init
    HAL_RCC_EnableModule(RCC_MOD_GPIO); // GPIO clock enable

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = Pin_Out;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Pin_In;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; //Set interrupt to trigger on raising and falling edge
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(GPIO_IRQn, 2, 1); // Configure NVIC priority

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
    HAL_NVIC_EnableIRQ(GPIO_IRQn); //Enable GPIO interrupt in the NVIC interrupt controller.
}

void GPIO1_IRQHandler(void) // Define the interrupt siervice routine (ISR) according to the interrupt vector table
{
    rt_interrupt_enter();
    HAL_GPIO_IRQHandler(hwp_gpio);
    rt_interrupt_leave();
}

void GPIO2_IRQHandler(void) // Define the interrupt siervice routine (ISR) according to the interrupt vector table
{
    rt_interrupt_enter();
    HAL_GPIO_IRQHandler(hwp_gpio);
    rt_interrupt_leave();
}

void gpio_test(void)
{
    HAL_GPIO_TogglePin(hwp_gpio, Pin_Out); //Toggle Pin_out to triger an interrupt on Pin_In
    rt_thread_mdelay(1000);
}

void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin) // override the weak Callback to add user defined action, it's called by HAL_GPIO_EXTI_IRQHandler
{
    if (GPIO_Pin == Pin_In)
    {
        // 你的中断处理逻辑
        rt_kprintf("Interrupt occurred!\n");
        rt_kprintf("Pin_Out %d has been toggle, value = %d\n", Pin_Out, HAL_GPIO_ReadPin(hwp_gpio, Pin_Out));
        rt_kprintf("Pin_In %d, value = %d\n", Pin_In, HAL_GPIO_ReadPin(hwp_gpio, Pin_In));
        rt_kprintf(" \n");
    }
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("Start gpio demo!\n"); // Output a start message on console using rt_kprintf function
    gpio_init();
    while (1)
    {
        gpio_test();
        // rt_thread_mdelay(5000);
    }
    return 0;
}

