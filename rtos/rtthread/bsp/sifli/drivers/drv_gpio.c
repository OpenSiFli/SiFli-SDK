/**
  ******************************************************************************
  * @file   drv_gpio.c
  * @author Sifli software development team
  * @brief GPIO BSP driver
  * @{
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

#include <board.h>
#include <stdlib.h>
#include "drv_gpio.h"
#include "string.h"
#include "mem_section.h"
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif /* BSP_USING_PM */

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_gpio GPIO
  * @brief GPIO BSP driver
  * @{
  */

#ifdef RT_USING_PIN

#define MAX_PIN_INPUT_CNT           (32)
#define PIN1_MAX_HANDLE             (GPIO1_PIN_NUM)
#define PBR_PIN_OFFSET              (GPIO1_PIN_NUM + GPIO2_PIN_NUM)

#ifdef hwp_pbr
    #define PIN_TOTAL_NUM               (GPIO1_PIN_NUM + GPIO2_PIN_NUM + HAL_PBR_MAX + 1)
#else
    #define PIN_TOTAL_NUM               (GPIO1_PIN_NUM + GPIO2_PIN_NUM)
#endif /* hwp_pbr */

#define GPIO_IRQ_PRIORITY           (5)

#ifdef SOC_BF0_HCPU
#define GPIO_ENABLE_GPIO1_IRQ()   \
    do                            \
    {                             \
        HAL_NVIC_SetPriority(GPIO1_IRQn, GPIO_IRQ_PRIORITY, 0);  \
        HAL_NVIC_EnableIRQ(GPIO1_IRQn);                          \
    }                                                            \
    while (0)
#endif /* SOC_BF0_HCPU */

#define GPIO_ENABLE_GPIO2_IRQ()   \
    do                            \
    {                             \
        HAL_NVIC_SetPriority(GPIO2_IRQn, GPIO_IRQ_PRIORITY, 0);  \
        HAL_NVIC_EnableIRQ(GPIO2_IRQn);                          \
    }                                                            \
    while (0)



static uint16_t pin_irq_hdr_num;
#ifdef hwp_pbr
    static uint16_t pbr_pin_irq_hdr_num;
    static uint8_t pbr_pin_state[HAL_PBR_MAX + 1];
    static uint8_t pbr_pin_irq_en[HAL_PBR_MAX + 1];
    static uint32_t pbr_pin_irq_pending_bitmap;
#endif /* hwp_pbr */
__ROM_USED struct rt_pin_irq_hdr pin_irq_hdr_tab[MAX_PIN_INPUT_CNT] =
{
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
    {-1, 0, 0, 0, RT_NULL, RT_NULL},
};
//static uint32_t pin_irq_enable_mask = 0;

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

/*
    Record recently gpio irq tick
*/
#ifdef SOC_BF0_HCPU
    #define DEBUG_RECENT_GPIO_IRQ_RECORDS  10
#else
    #define DEBUG_RECENT_GPIO_IRQ_RECORDS  2
#endif /* SOC_BF0_HCPU */

static volatile uint32_t gpio1_irq_count = 0;
static volatile uint32_t gpio2_irq_count = 0;

#if (DEBUG_RECENT_GPIO_IRQ_RECORDS > 0)
typedef struct _gpio_irq_record_t
{
    uint32_t tick;
    uint16_t core;
    uint16_t pin;
} gpio_irq_record_t;

static volatile gpio_irq_record_t gpio_irq_records[DEBUG_RECENT_GPIO_IRQ_RECORDS];
static volatile uint32_t gpio_irq_records_iter = 0;
static void record_pin_irq(GPIO_TypeDef *hgpio, uint16_t pin_num)
{
    gpio_irq_records[gpio_irq_records_iter].core = (hgpio == (GPIO_TypeDef *)hwp_gpio1);
    gpio_irq_records[gpio_irq_records_iter].pin  = pin_num;
    gpio_irq_records[gpio_irq_records_iter].tick = HAL_GetTick();

    if (gpio_irq_records_iter + 1 < DEBUG_RECENT_GPIO_IRQ_RECORDS)
        gpio_irq_records_iter++;
    else
        gpio_irq_records_iter = 0;
}


#endif /*(DEBUG_RECENT_GPIO_IRQ_RECORDS > 0)*/

/* called in systick_Handler */
void drv_pin_check(void)
{
#ifdef hwp_pbr
    uint32_t i;
    int8_t val;
    bool irq_triggered = false;
    struct rt_pin_irq_hdr *hdr_item;
    rt_base_t level;
    uint8_t trigger_pin_num;
    uint8_t pbr_pin;
    int8_t wakeup_pin;

    if (0 == pbr_pin_irq_hdr_num)
    {
        return;
    }

    hdr_item = &pin_irq_hdr_tab[0];
    trigger_pin_num = 0;
    level = rt_hw_interrupt_disable();
    for (i = 0; i < pin_irq_hdr_num; i++, hdr_item++)
    {
        irq_triggered = false;
        if (hdr_item->hdr && (hwp_pbr == GET_GPIO_INSTANCE(hdr_item->pin)) && (hdr_item->en))
        {
            pbr_pin = GET_GPIOx_PIN(hdr_item->pin);
            RT_ASSERT(pbr_pin <= HAL_PBR_MAX);
            val = HAL_PBR_ReadPin(pbr_pin);
            RT_ASSERT(val >= 0);
            switch (hdr_item->mode)
            {
            case PIN_IRQ_MODE_RISING:
                if ((0 == pbr_pin_state[pbr_pin]) && (1 == val))
                {
                    irq_triggered = true;
                }
                break;
            case PIN_IRQ_MODE_FALLING:
                if ((1 == pbr_pin_state[pbr_pin]) && (0 == val))
                {
                    irq_triggered = true;
                }
                break;
            case PIN_IRQ_MODE_RISING_FALLING:
                if (pbr_pin_state[pbr_pin] != val)
                {
                    irq_triggered = true;
                }
                break;
            case PIN_IRQ_MODE_HIGH_LEVEL:
                if (1 == val)
                {
                    irq_triggered = true;
                }
                break;
            case PIN_IRQ_MODE_LOW_LEVEL:
                if (0 == val)
                {
                    irq_triggered = true;
                }
                break;
            }
            hdr_item->state = val;
            pbr_pin_state[pbr_pin] = val;
            if (irq_triggered)
            {
                /* clear wsr to avoid trigger agagin by wsr check*/
#ifdef SOC_BF0_HCPU
                wakeup_pin = HAL_HPAON_QueryWakeupPin(hwp_pbr, pbr_pin);
                if (wakeup_pin >= 0)
                {
                    HAL_HPAON_CLEAR_WSR(1UL << (HPSYS_AON_WCR_PIN0_Pos + wakeup_pin));
                }
#else
                wakeup_pin = HAL_LPAON_QueryWakeupPin(hwp_pbr, pbr_pin);
                if (wakeup_pin >= 0)
                {
                    HAL_LPAON_CLEAR_WSR(1UL << (LPSYS_AON_WCR_PIN0_Pos + wakeup_pin));
                }
#endif /* SOC_BF0_HCPU */

                RT_ASSERT(pbr_pin < 32);
                pbr_pin_irq_pending_bitmap |= (1 << pbr_pin);
                RT_ASSERT(trigger_pin_num < MAX_PIN_INPUT_CNT);
                trigger_pin_num++;

            }
        }
    }
    rt_hw_interrupt_enable(level);

    if (trigger_pin_num > 0)
    {
#ifdef SOC_BF0_HCPU
        HAL_NVIC_SetPendingIRQ(GPIO1_IRQn);
#else
        HAL_NVIC_SetPendingIRQ(GPIO2_IRQn);
#endif /* SOC_BF0_HCPU */
    }



#endif /* hwp_pbr */
}


#ifdef BSP_USING_PM
/*
    Save current core's gpio configuration to buffer.
*/

static void sifli_pin_suspend(rt_device_t dev, rt_uint32_t flag)
{
}

static void sifli_pin_resume(rt_device_t dev, rt_uint32_t flag)
{
    uint32_t i;
    struct rt_pin_irq_hdr *item;
    rt_base_t level = rt_hw_interrupt_disable();


    item = &pin_irq_hdr_tab[0];
    for (i = 0; i < pin_irq_hdr_num; i++)
    {
        if (item->en)
        {
            if (item->pin < PIN1_MAX_HANDLE)
            {
#ifdef SOC_BF0_HCPU
                GPIO_ENABLE_GPIO1_IRQ();
#endif /* SOC_BF0_HCPU */
            }
            else if (item->pin < PBR_PIN_OFFSET)
            {
                GPIO_ENABLE_GPIO2_IRQ();
            }
            else
            {
#ifdef SOC_BF0_HCPU
                GPIO_ENABLE_GPIO1_IRQ();
#else
                GPIO_ENABLE_GPIO2_IRQ();
#endif /* SOC_BF0_HCPU */
            }
        }
        item++;
    }
    rt_hw_interrupt_enable(level);
}

#endif /* RT_USING_PM */

static void sifli_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    if ((dev == NULL) || (dev->user_data == NULL))
        return;

    if (pin < PIN1_MAX_HANDLE)
        HAL_GPIO_WritePin((GPIO_TypeDef *)hwp_gpio1, pin, (GPIO_PinState)value);
    else if (pin < (PIN1_MAX_HANDLE + GPIO2_PIN_NUM))
        HAL_GPIO_WritePin((GPIO_TypeDef *)hwp_gpio2, pin - PIN1_MAX_HANDLE, (GPIO_PinState)value);
    else
#ifdef hwp_pbr
        HAL_PBR_WritePin(pin - (PIN1_MAX_HANDLE + GPIO2_PIN_NUM), (uint8_t)value);
#else
        RT_ASSERT(0);
#endif /* hwp_pbr */
}

static int sifli_pin_read(rt_device_t dev, rt_base_t pin)
{
    int value;
    if ((dev == NULL) || (dev->user_data == NULL))
        return 0;

    if (pin < PIN1_MAX_HANDLE)
        value = HAL_GPIO_ReadPin((GPIO_TypeDef *)hwp_gpio1, pin);
    else if (pin < (PIN1_MAX_HANDLE + GPIO2_PIN_NUM))
        value = HAL_GPIO_ReadPin((GPIO_TypeDef *)hwp_gpio2, pin - PIN1_MAX_HANDLE);
    else
    {
#ifdef hwp_pbr
        value = HAL_PBR_ReadPin(pin - (PIN1_MAX_HANDLE + GPIO2_PIN_NUM));
        RT_ASSERT(value >= 0);
#else
        RT_ASSERT(0);
#endif /* hwp_pbr */
    }

    return value;
}

static void sifli_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_TypeDef *gphandle = NULL;
    bool output_en = false;

    if ((dev == NULL) || (dev->user_data == NULL))
        return;

    /* Configure GPIO_InitStructure */
    if (pin < PIN1_MAX_HANDLE)
    {
        GPIO_InitStruct.Pin = pin;
        gphandle = (GPIO_TypeDef *)hwp_gpio1;
    }
    else
    {
        GPIO_InitStruct.Pin = pin - PIN1_MAX_HANDLE;
        gphandle = (GPIO_TypeDef *)hwp_gpio2;
    }
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    if (mode == PIN_MODE_OUTPUT)
    {
        /* output setting */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        output_en = true;
    }
    else if (mode == PIN_MODE_INPUT)
    {
        /* input setting: not pull. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        /* input setting: pull up. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        /* input setting: pull down. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        /* output setting: od. */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        output_en = true;
    }

    if (pin < (PIN1_MAX_HANDLE + GPIO2_PIN_NUM))
    {
        HAL_GPIO_Init(gphandle, &GPIO_InitStruct);
    }
    else
    {
#ifdef hwp_pbr
        HAL_StatusTypeDef status;
        /* PBR pin doesn't support open-drain output */
        RT_ASSERT(mode != PIN_MODE_OUTPUT_OD);
        status = HAL_PBR_ConfigMode(pin - PBR_PIN_OFFSET, output_en);
        RT_ASSERT(HAL_OK == status);
#else
        RT_ASSERT(0);
#endif /* hwp_pbr */
    }
}

static rt_err_t sifli_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                     rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    rt_base_t level;
    int i;
    struct rt_pin_irq_hdr *item;

    level = rt_hw_interrupt_disable();

    for (i = 0; i < pin_irq_hdr_num; i++)
        if (pin_irq_hdr_tab[i].pin == pin)
            break;
    if (i < pin_irq_hdr_num)
    {
        if (pin_irq_hdr_tab[i].pin == pin &&
                pin_irq_hdr_tab[i].hdr == hdr &&
                pin_irq_hdr_tab[i].mode == mode &&
                pin_irq_hdr_tab[i].args == args)    // attached before
        {
            rt_hw_interrupt_enable(level);
            return RT_EOK;
        }
        else // found but not correct parameters, quit ? or recover ?
        {
            rt_hw_interrupt_enable(level);
            return RT_EBUSY;
        }
    }
    else // not found
    {
        RT_ASSERT(pin_irq_hdr_num < MAX_PIN_INPUT_CNT);
        item = &pin_irq_hdr_tab[pin_irq_hdr_num];
        RT_ASSERT(item->pin == -1);
        item->pin = pin;
        item->en = 0;
        item->hdr = hdr;
        item->mode = mode;
        item->args = args;
        pin_irq_hdr_num++;
#ifdef hwp_pbr
        if (pin >= PBR_PIN_OFFSET)
        {
            pbr_pin_irq_hdr_num++;
        }
#endif /* hwp_pbr */

        // else //full
        //    assert(0);

        rt_hw_interrupt_enable(level);
    }

    return RT_EOK;
}

static rt_err_t sifli_pin_dettach_irq(struct rt_device *device, rt_int32_t pin)
{
    rt_base_t level;
    int i;

    level = rt_hw_interrupt_disable();

    for (i = 0; i < pin_irq_hdr_num; i++)
        if (pin_irq_hdr_tab[i].pin == pin)
            break;
    if (i < pin_irq_hdr_num)
    {
        if (i != (pin_irq_hdr_num - 1))
        {
            memcpy(&pin_irq_hdr_tab[i], &pin_irq_hdr_tab[pin_irq_hdr_num - 1], sizeof(pin_irq_hdr_tab[0]));
        }

        pin_irq_hdr_tab[pin_irq_hdr_num - 1].pin = -1;
        pin_irq_hdr_tab[pin_irq_hdr_num - 1].en = 0;
        pin_irq_hdr_tab[pin_irq_hdr_num - 1].hdr = RT_NULL;
        pin_irq_hdr_tab[pin_irq_hdr_num - 1].mode = 0;
        pin_irq_hdr_tab[pin_irq_hdr_num - 1].args = RT_NULL;

#ifdef hwp_pbr
        if (hwp_pbr == GET_GPIO_INSTANCE(pin))
        {
            RT_ASSERT(pbr_pin_irq_hdr_num > 0);
            pbr_pin_irq_hdr_num--;
        }
#endif /* hwp_pbr */

        pin_irq_hdr_num--;
    }

    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t sifli_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                     rt_uint32_t enabled)
{
    rt_base_t level;
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_TypeDef *gphandle = NULL;
    int i;
#ifdef hwp_pbr
    uint8_t pbr_pin = 0;
#endif /* hwp_pbr */

    if ((device == NULL) || (device->user_data == NULL))
        return RT_ERROR;

    /* Configure GPIO_InitStructure */
    if (pin < PIN1_MAX_HANDLE)
    {
        GPIO_InitStruct.Pin = pin;
        gphandle = (GPIO_TypeDef *)hwp_gpio1;
    }
    else if (pin < (PIN1_MAX_HANDLE + GPIO2_PIN_NUM))
    {
        GPIO_InitStruct.Pin = pin - PIN1_MAX_HANDLE;
        gphandle = (GPIO_TypeDef *)hwp_gpio2;
    }
#ifdef hwp_pbr
    else
    {
        pbr_pin = pin - PBR_PIN_OFFSET;
        RT_ASSERT(pbr_pin <= HAL_PBR_MAX);
    }
#endif  /* hwp_pbr */

    if (enabled == PIN_IRQ_ENABLE)
    {
        level = rt_hw_interrupt_disable();

        for (i = 0; i < pin_irq_hdr_num; i++)
            if (pin_irq_hdr_tab[i].pin == pin)
                break;

        if (i >= pin_irq_hdr_num) // not attached
        {
            rt_hw_interrupt_enable(level);
            return RT_ENOSYS;
        }

        /* Configure GPIO_InitStructure */
        //GPIO_InitStruct.Pin = pin;
        switch (pin_irq_hdr_tab[i].mode)
        {
        case PIN_IRQ_MODE_RISING:
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            break;
        case PIN_IRQ_MODE_FALLING:
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            break;
        case PIN_IRQ_MODE_RISING_FALLING:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
            break;
        case PIN_IRQ_MODE_HIGH_LEVEL:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Mode = GPIO_MODE_IT_HIGH_LEVEL;
            break;
        case PIN_IRQ_MODE_LOW_LEVEL:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Mode = GPIO_MODE_IT_LOW_LEVEL;
            break;
        }

        if (gphandle)
        {
            HAL_GPIO_Init(gphandle, &GPIO_InitStruct);

            if (pin < PIN1_MAX_HANDLE)   // PA
            {
#ifdef SOC_BF0_HCPU     // gpio1 only work on hcpu
                GPIO_ENABLE_GPIO1_IRQ();
#endif // SOC_BF0_HCPU
            }
            else // PB
            {
                GPIO_ENABLE_GPIO2_IRQ();
            }
            pin_irq_hdr_tab[i].en = 1;
        }
        else
        {
#ifdef hwp_pbr
            int8_t pin_state;
            pin_state = HAL_PBR_ReadPin(pbr_pin);
            RT_ASSERT(pin_state >= 0);
            pin_irq_hdr_tab[i].state = pin_state & 1;
            pbr_pin_state[pbr_pin] = pin_state & 1;
            pin_irq_hdr_tab[i].en = 1;
            pbr_pin_irq_en[pbr_pin] = 1;

#ifdef SOC_BF0_HCPU
            GPIO_ENABLE_GPIO1_IRQ();
#else
            GPIO_ENABLE_GPIO2_IRQ();
#endif  /* SOC_BF0_HCPU */
#else
            RT_ASSERT(0);
#endif /* hwp_pbr */
        }
        //pin_irq_enable_mask |= (1 << pin);

        rt_hw_interrupt_enable(level);
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        level = rt_hw_interrupt_disable();

        for (i = 0; i < pin_irq_hdr_num; i++)
            if (pin_irq_hdr_tab[i].pin == pin)
                break;
        if (i < pin_irq_hdr_num)
        {
            pin_irq_hdr_tab[i].en = 0;
        }
        if (gphandle)
        {
            HAL_GPIO_DeInit(gphandle, GPIO_InitStruct.Pin);
        }
        else
        {
#ifdef hwp_pbr
            pbr_pin_irq_en[pbr_pin] = 0;
            pbr_pin_irq_pending_bitmap &= ~(1 << pbr_pin);
#else
            RT_ASSERT(0);
#endif /* hwp_pbr */
        }
        //pin_irq_enable_mask &= ~(1 << pin);
        //HAL_NVIC_DisableIRQ(GPIO_IRQn);
        rt_hw_interrupt_enable(level);
    }
    else
    {
        return -RT_ENOSYS;
    }

    return RT_EOK;
}
const static struct rt_pin_ops _sifli_pin_ops =
{
    sifli_pin_mode,
    sifli_pin_write,
    sifli_pin_read,
    sifli_pin_attach_irq,
    sifli_pin_dettach_irq,
    sifli_pin_irq_enable,
#ifdef BSP_USING_PM
    sifli_pin_suspend,
    sifli_pin_resume,
#else
    NULL,
    NULL,
#endif /* RT_USING_PM */

};

//rt_inline void pin_irq_hdr(int irqno)
void pin_irq_hdr(GPIO_TypeDef *hgpio, int irqno)
{
    int i, pin;

    if (hgpio == (GPIO_TypeDef *)hwp_gpio1)
        pin = irqno;
    else if (hgpio == (GPIO_TypeDef *)hwp_gpio2)
        pin = irqno + PIN1_MAX_HANDLE;
#ifdef hwp_pbr
    else if (hgpio == (GPIO_TypeDef *)hwp_pbr)
        pin = irqno + PIN1_MAX_HANDLE + GPIO2_PIN_NUM;
#endif /* hwp_pbr */
    else
        RT_ASSERT(0);

    for (i = 0; i < pin_irq_hdr_num; i++)
        if (pin_irq_hdr_tab[i].pin == pin)
            break;

    if (i < pin_irq_hdr_num)
    {
#ifdef hwp_pbr
        if (hgpio == (GPIO_TypeDef *)hwp_pbr)
        {
            int8_t val = HAL_PBR_ReadPin(irqno);
            RT_ASSERT(val >= 0);
            pin_irq_hdr_tab[i].state = val; // not used anymore
        }
#endif /* hwp_pbr */
        pin_irq_hdr_tab[i].hdr(pin_irq_hdr_tab[i].args);
    }
}

void drv_pin_irq_from_wsr(uint32_t wsr_pins)
{
    rt_base_t level = rt_hw_interrupt_disable();

    if (wsr_pins)
    {
        // rt_kprintf("Force trigger GPIO handler %x (%d) \r\n", wsr_pins, rt_interrupt_get_nest());
#ifdef SOC_BF0_HCPU
        HAL_NVIC_SetPendingIRQ(GPIO1_IRQn);
#else
        HAL_NVIC_SetPendingIRQ(GPIO2_IRQn);
#endif
    }
    rt_hw_interrupt_enable(level);
}

void check_wsr_pin(void)
{
    uint32_t status;
    GPIO_TypeDef *gpio;
    uint16_t pin;
    uint32_t pin_wsr;
    uint32_t i;
    AON_PinModeTypeDef pin_mode;
    HAL_StatusTypeDef hal_status;
    uint16_t wake_pin_num;
    rt_base_t level;
#ifdef HPSYS_AON_WSR_PBR_PIN_FIRST
    uint32_t pbr_pin_wsr;
#endif /* HPSYS_AON_WSR_PBR_PIN_FIRST */

    level = rt_hw_interrupt_disable();

#ifdef SOC_BF0_HCPU
    status = HAL_HPAON_GET_WSR() & HPSYS_AON_WSR_PIN_ALL;
    pin_wsr = status >> HPSYS_AON_WSR_PIN0_Pos;
    HAL_HPAON_CLEAR_WSR(status);
    wake_pin_num = HPSYS_AON_WSR_PIN_NUM;
#else
    status = HAL_LPAON_GET_WSR() & LPSYS_AON_WSR_PIN_ALL;
    pin_wsr = status >> LPSYS_AON_WSR_PIN0_Pos;
    HAL_LPAON_CLEAR_WSR(status);
    wake_pin_num = LPSYS_AON_WSR_PIN_NUM;
#endif

#ifdef HPSYS_AON_WSR_PBR_PIN_FIRST
    pbr_pin_wsr = (status & LPSYS_AON_WSR_PBR_PIN_ALL) >> LPSYS_AON_WSR_PBR_PIN_FIRST;
    /* update pbr_pin_state  and pending_bitmap to avoid trigger callback twice */
    for (i = 0; (i <= HAL_PBR_MAX) && pbr_pin_wsr; i++)
    {
        if (pbr_pin_wsr & 1)
        {
            if (pbr_pin_irq_en[i])
            {
                pbr_pin_state[i] = HAL_PBR_ReadPin(i);
                pbr_pin_irq_pending_bitmap &= ~(1 << i);
            }
        }
        pbr_pin_wsr >>= 1;
    }
#endif /* HPSYS_AON_WSR_PBR_PIN_FIRST */

    rt_hw_interrupt_enable(level);

    for (i = 0; (i < wake_pin_num) && pin_wsr; i++)
    {
        if (pin_wsr & 1)
        {
            hal_status = HAL_AON_GetWakePinMode(i, &pin_mode);
            if ((HAL_OK == hal_status) && (pin_mode != AON_PIN_MODE_HIGH)
                    && (pin_mode != AON_PIN_MODE_LOW))
            {
                gpio = HAL_AON_QueryWakeupGpioPin(i, &pin);
                RT_ASSERT(gpio);
                //rt_kprintf("Execute force trigger GPIO handler %d \r\n", pin);

#ifdef hwp_pbr
                if (gpio == hwp_pbr)
                {
                    /* do nothing */
                }
                else
#endif /* hwp_pbr */
                {
                    HAL_GPIO_ClearPinInterrupt(gpio, pin); //Clear GPIO pin pending IRQ.
                }
                HAL_GPIO_EXTI_Callback(gpio, pin);
            }
        }
        pin_wsr >>= 1;
    }

#ifdef hwp_pbr
    /* handle pending pbr pin callback detected by drv_pin_check */
    level = rt_hw_interrupt_disable();
    status = pbr_pin_irq_pending_bitmap;
    pbr_pin_irq_pending_bitmap = 0;
    rt_hw_interrupt_enable(level);

    for (i = 0; (i <= HAL_PBR_MAX) && status; i++)
    {
        if (status & 1)
        {
            HAL_GPIO_EXTI_Callback(hwp_pbr, i);
        }
        status >>= 1;
    }
#endif /* hwp_pbr */

}

void GPIO1_IRQHandler(void)
{
    rt_interrupt_enter();

    gpio1_irq_count++;

#ifdef SOC_BF0_HCPU
    check_wsr_pin();
#endif /* SOC_BF0_HCPU */
    HAL_GPIO_IRQHandler((GPIO_TypeDef *)hwp_gpio1);
    rt_interrupt_leave();
}

void GPIO2_IRQHandler(void)
{
    uint32_t i;
    uint32_t pin;

    rt_interrupt_enter();

    gpio2_irq_count++;

#ifdef SOC_BF0_LCPU
    check_wsr_pin();
#endif /* SOC_BF0_LCPU */

#ifdef SOC_BF0_HCPU
    for (i = 0; i < pin_irq_hdr_num; i++)
    {
        /* LPSYS should be active when HPSYS is active,
           so HCPU only needs to handle interrupt source registered by its own */
        if ((pin_irq_hdr_tab[i].pin >= PIN1_MAX_HANDLE) && (pin_irq_hdr_tab[i].pin < PBR_PIN_OFFSET))
        {
            pin = pin_irq_hdr_tab[i].pin - PIN1_MAX_HANDLE;
            HAL_GPIO_EXTI_IRQHandler((GPIO_TypeDef *)hwp_gpio2, pin);
        }
    }
#else
    if (HAL_LPAON_IS_HP_ACTIVE())
    {
        /* If HPSYS is active, only handle interrupt source registered by LCPU,
           others should be handled by HCPU */
        for (i = 0; i < pin_irq_hdr_num; i++)
        {
            if ((pin_irq_hdr_tab[i].pin >= PIN1_MAX_HANDLE) && (pin_irq_hdr_tab[i].pin < PBR_PIN_OFFSET))
            {
                pin = pin_irq_hdr_tab[i].pin - PIN1_MAX_HANDLE;
                HAL_GPIO_EXTI_IRQHandler((GPIO_TypeDef *)hwp_gpio2, pin);
            }
        }
    }
    else
    {
        /* If HPSYS is inactive, clear all interrupt source even if it's not registered by LCPU side */
        HAL_GPIO_IRQHandler((GPIO_TypeDef *)hwp_gpio2);
    }


#endif /* SOC_BF0_HCPU */


    rt_interrupt_leave();
}

__ROM_USED int rt_hw_pin_init(void)
{
    return rt_device_pin_register("pin", &_sifli_pin_ops, hwp_gpio1);
}

#if defined(SF32LB52X)
    __ROM_USED
#endif /* SF32LB52X */
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    rt_base_t level = rt_hw_interrupt_disable();//Prevent re-entry from systick_IRQ/GPIO_IRQ/AON_IRQ

#if (DEBUG_RECENT_GPIO_IRQ_RECORDS > 0)
    record_pin_irq(hgpio, GPIO_Pin);
#endif

    pin_irq_hdr(hgpio, GPIO_Pin);

    rt_hw_interrupt_enable(level);
}

#ifdef RT_USING_FINSH
//#define DRV_TEST
#if 1//def DRV_TEST

#define DBG_TAG "TEST.GPIO"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


static void gpio_int_test(void *args)
{
    int value = (int)args;
    LOG_I("gpio_int_test %d\n", value);
}

#define PAD_PA_00 PAD_PA00
#define PAD_PB_00 PAD_PB00

static void print_pin_state(int pin)
{
    int pad, hcpu;

    GPIO_TypeDef *gphandle = NULL;
    uint16_t gpio_pin;

    pin_function func;
    PIN_ModeTypeDef pinmode;
    int func_idx;
    uint32_t mode;
    uint32_t pin_val;

    int pin_irq_hdr_tab_idx;

    const char *str_pull, *str_gpio;

#ifdef hwp_pbr
    bool output_en;
    HAL_StatusTypeDef status;
    uint32_t i;
#endif /* hwp_pbr */


    if (pin < PIN1_MAX_HANDLE)
    {
        pad = (int)pin + PAD_PA_00;
        gpio_pin = pin;
        hcpu = 1;
        gphandle = (GPIO_TypeDef *)hwp_gpio1;
    }
    else
    {
        hcpu = 0;
        if (pin < PBR_PIN_OFFSET)
        {
            pad = (int)pin - PIN1_MAX_HANDLE + PAD_PB_00;
            gpio_pin = pin - PIN1_MAX_HANDLE;
            gphandle = (GPIO_TypeDef *)hwp_gpio2;
        }
        else
        {
#ifdef hwp_pbr
            pad = (int)pin - PBR_PIN_OFFSET + PAD_PBR0;
            gpio_pin = pin - PBR_PIN_OFFSET;
            gphandle = hwp_pbr;
#else
            LOG_I("invalid pin: %d", pin);
            return;
#endif /* hwp_pbr */
        }
    }

    //Pin mux & Pull state
    func_idx = HAL_PIN_Get(pad, &func, &pinmode, hcpu);

    switch (pinmode)
    {
    case PIN_ANALOG_INPUT:
        str_pull = "ANA_IN";
        break;
    case PIN_DIGITAL_IO_NORMAL:
        str_pull = "DIG_IO";
        break;
    case PIN_DIGITAL_IO_PULLUP:
        str_pull = "DIG_IO_PU";
        break;
    case PIN_DIGITAL_IO_PULLDOWN:
        str_pull = "DIG_IO_PD";
        break;
    case PIN_DIGITAL_O_NORMAL:
        str_pull = "DIG_OUT";
        break;
    case PIN_DIGITAL_O_PULLUP:
        str_pull = "DIG_OUT_PU";
        break;
    default:
        str_pull = "UNKNOW";
        break;
    }

    //GPIO mode
#ifdef hwp_pbr
    if (gphandle == hwp_pbr)
    {
        status = HAL_PBR_GetMode(gpio_pin, &output_en);
        if (HAL_OK != status)
        {
            LOG_I("Fail to get PBR[%d] mode", gpio_pin);
            return;
        }
        if (output_en)
        {
            mode = GPIO_MODE_OUTPUT;
        }
        else
        {
            mode = GPIO_MODE_INPUT;
            for (i = 0; i < pin_irq_hdr_num; i++)
            {
                if (pin_irq_hdr_tab[i].pin == pin)
                {
                    break;
                }
            }
            if (i < pin_irq_hdr_num)
            {
                switch (pin_irq_hdr_tab[i].mode)
                {
                case PIN_IRQ_MODE_RISING:
                    mode = GPIO_MODE_IT_RISING;
                    break;
                case PIN_IRQ_MODE_FALLING:
                    mode = GPIO_MODE_IT_FALLING;
                    break;
                case PIN_IRQ_MODE_RISING_FALLING:
                    mode = GPIO_MODE_IT_RISING_FALLING;
                    break;
                case PIN_IRQ_MODE_HIGH_LEVEL:
                    mode = GPIO_MODE_IT_HIGH_LEVEL;
                    break;
                case PIN_IRQ_MODE_LOW_LEVEL:
                    mode = GPIO_MODE_IT_LOW_LEVEL;
                    break;
                default:
                {
                    RT_ASSERT(0);
                }
                }
            }
        }
        pin_val = HAL_PBR_ReadPin(gpio_pin);
    }
    else
#endif /* hwp_pbr */
    {

        mode = HAL_GPIO_GetMode(gphandle, gpio_pin);
        pin_val = HAL_GPIO_ReadPin(gphandle, gpio_pin);
    }

#define VALUE_TO_NAME_CASE(e) case e: str_gpio = #e; break

    switch (mode)
    {
        VALUE_TO_NAME_CASE(GPIO_MODE_INPUT);
        VALUE_TO_NAME_CASE(GPIO_MODE_OUTPUT);
        VALUE_TO_NAME_CASE(GPIO_MODE_OUTPUT_OD);
        VALUE_TO_NAME_CASE(GPIO_MODE_IT_RISING);
        VALUE_TO_NAME_CASE(GPIO_MODE_IT_FALLING);
        VALUE_TO_NAME_CASE(GPIO_MODE_IT_RISING_FALLING);
        VALUE_TO_NAME_CASE(GPIO_MODE_IT_HIGH_LEVEL);
        VALUE_TO_NAME_CASE(GPIO_MODE_IT_LOW_LEVEL);

    default:
        str_gpio = "GPIO_MODE_UNKNOW";
        break;
    }

    //Drv_gpio pin_irq_hdr_tab
    for (pin_irq_hdr_tab_idx = 0; pin_irq_hdr_tab_idx < pin_irq_hdr_num; pin_irq_hdr_tab_idx++)
    {
        if (pin_irq_hdr_tab[pin_irq_hdr_tab_idx].pin == pin)
        {
            break;
        }
    }

    if (pin_irq_hdr_tab_idx != pin_irq_hdr_num)
    {
        /*
                const char *str_drv_irq;

        #undef VALUE_TO_NAME_CASE
        #define VALUE_TO_NAME_CASE(e) case e: str_drv_irq = #e; break
                switch(pin_irq_hdr_tab[pin_irq_hdr_tab_idx].mode)
                {
                    VALUE_TO_NAME_CASE(PIN_IRQ_MODE_FALLING);
                    VALUE_TO_NAME_CASE(PIN_IRQ_MODE_RISING);
                    VALUE_TO_NAME_CASE(PIN_IRQ_MODE_RISING_FALLING);
                    VALUE_TO_NAME_CASE(PIN_IRQ_MODE_HIGH_LEVEL);
                    VALUE_TO_NAME_CASE(PIN_IRQ_MODE_LOW_LEVEL);

                    default:   str_drv_irq = "PIN_IRQ_MODE_UNKNOW";  break;
                }
        */

        LOG_I("PIN %d, FUNC=%d, VAL=%d, %s, %s, irqhdr=%x, arg=%x", pin, func_idx, pin_val, str_pull, str_gpio,
              pin_irq_hdr_tab[pin_irq_hdr_tab_idx].hdr,
              pin_irq_hdr_tab[pin_irq_hdr_tab_idx].args);
    }
    else
    {
        LOG_I("PIN %d, FUNC=%d, VAL=%d, %s, %s, irqhdr=/, arg=/", pin, func_idx, pin_val, str_pull, str_gpio);
    }
}


__ROM_USED int cmd_pin(int argc, char **argv)
{
    rt_device_t device ;
    if (argc <= 2)
    {
        LOG_I("usage: pin <mode|read|write|mux|status> pin# <value>");
        LOG_I("        example:                               ");
        LOG_I("             pin mode  #pin <PIN MODE VALUES>  ");
        LOG_I("             pin write #pin 1         ");
        LOG_I("             pin read  #pin           ");
        LOG_I("             pin mux   #pin 0         ");
        LOG_I("             pin status #pin          ");
        LOG_I("             pin mux   #pin ?         ");
        LOG_I("             pin status all           ");
        LOG_I("\n    PIN MODE VALUES:                ");
        LOG_I("        PIN_MODE_OUTPUT         0x00  ");
        LOG_I("        PIN_MODE_INPUT          0x01  ");
        LOG_I("        PIN_MODE_INPUT_PULLUP   0x02  ");
        LOG_I("        PIN_MODE_INPUT_PULLDOWN 0x03  ");
        LOG_I("        PIN_MODE_OUTPUT_OD      0x04  ");

        return (-RT_EINVAL);
    }

    device = rt_device_find("pin");
    if (!device)
    {
        LOG_I("Find device pin fail\n");
        return (-RT_EIO);
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (argc > 3)
    {
        if (strcmp(argv[1], "mode") == 0)
        {
            struct rt_device_pin_mode m;
            m.pin = atoi(argv[2]);
            m.mode = atoi(argv[3]);
            rt_device_control(device, 0, &m);
            if (PIN_MODE_INPUT == m.mode)   // for input mode, set interrupt
            {
                // connect 2 pin, 1 as output and 2 as input
                rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING_FALLING, gpio_int_test, (void *)(rt_uint32_t)m.pin);
                rt_pin_irq_enable(m.pin, 1);
            }
        }
        else if (strcmp(argv[1], "write") == 0)
        {
            struct rt_device_pin_status st;
            st.pin = atoi(argv[2]);
            st.status = atoi(argv[3]);
            rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
        }
        else if (strcmp(argv[1], "mux") == 0)
        {
            int32_t pin = atoi(argv[2]);

            int pad, hcpu;

            if (pin < PIN1_MAX_HANDLE)
            {
                pad = (int)pin + PAD_PA_00;
                hcpu = 1;
            }
            else
            {
                pad = (int)pin - PIN1_MAX_HANDLE + PAD_PB_00;
                hcpu = 0;
            }


            if (strcmp(argv[3], "?") == 0)//Get pin mux
            {
                print_pin_state(pin);
            }
            else //Set pin mux
            {
                int func = atoi(argv[3]);
                HAL_PIN_Select(pad, func, hcpu);
            }


        }
        else
        {
            LOG_I("Unknow cmd!");
        }
    }
    else // argc==3, only for read
    {
        if (strcmp(argv[1], "read") == 0)
        {
            struct rt_device_pin_status st;
            st.pin = atoi(argv[2]);
            rt_device_read(device, 0, &st, sizeof(struct rt_device_pin_status));
            LOG_I("value=%d\n", st.status);
        }
        else if (strcmp(argv[1], "status") == 0)
        {
            if (strcmp(argv[2], "all") == 0)
            {
                for (int pin = 0; pin < PIN_TOTAL_NUM; pin++)
                    print_pin_state(pin);
            }
            else
            {
                print_pin_state(atoi(argv[2]));
            }

        }
        else
        {
            LOG_I("Invalid parameter\n");
        }

    }
    rt_device_close(device);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pin, __cmd_pin, pin gpio functions);
#endif

#endif /* finsh */

#endif /* RT_USING_PIN */

/// @} drv_gpio
/// @} bsp_driver
/// @} file

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
