/**
  ******************************************************************************
  * @file   drv_pwm.c
  * @author Sifli software development team
  * @brief PWM BSP driver
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
#include "bf0_hal_rcc.h"


/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_rgbled RGBLED driver
  * @ingroup bsp_driver
  * @brief RGBLED BSP driver
  * @{
  */

#if defined(BSP_USING_RGBLED) || defined(_SIFLI_DOXYGEN_)

#include "drv_config.h"
#include "mem_section.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.rgb"
#include <drv_log.h>

#define MAX_PERIOD_GPT (0xFFFF)
#define MAX_PERIOD_ATM (0xFFFFFFFF)
#define MIN_PERIOD 3
#define MIN_PULSE 2

enum
{
#ifdef BSP_USING_RGBLED_WITCH_PWM2
    PWM2_INDEX,
#endif
#ifdef BSP_USING_RGBLED_WITCH_PWM3
    PWM3_INDEX,
#endif

};

#define RGB_COLOR_LEN   24
#define RGB_REST_LEN   300
#define RGB_STOP_LEN   1

#define pwm_peroid  1600        // reg value =24
#define pulse_peroid  800

#define reg_high 18
#define reg_low  7
#define reg_end  50



L1_NON_RET_BSS_SECT_BEGIN(rgb_led_buf)
L1_NON_RET_BSS_SECT(rgb_led_buf, ALIGN(16) static uint16_t rgb_buff[RGB_REST_LEN + RGB_COLOR_LEN + RGB_STOP_LEN]);
L1_NON_RET_BSS_SECT_END


struct bf0_rgbled
{
    struct rt_device_pwm pwm_device;    /*!<PWM device object handle*/
    GPT_HandleTypeDef    tim_handle;    /*!<General Purpose Timer(GPT) device object handle used in PWM*/
    rt_uint8_t channel;                 /*!<GPT channel used*/
    char *name;                         /*!<Device name*/
    DMA_HandleTypeDef   dma_handle;      /*!< DMA device Handle used by this driver */
    IRQn_Type           dma_irq;
};

static struct bf0_rgbled bf0_rgbled_obj[] =
{

#ifdef BSP_USING_RGBLED_WITCH_PWM3
    RGBLED_CONFIG,
#endif

};
#if !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)

static void PWMx_DMA_IRQHandler(uint32_t index)
{
    GPT_HandleTypeDef *handle;

    /* enter interrupt */
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(handle->hdma[GPT_DMA_ID_CC1]);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_RGBLED_WITCH_PWM3) || defined(_SIFLI_DOXYGEN_)
/**
  * @brief  HW GTIM2 interrupt
  */
void GPTIM2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_rgbled_obj[PWM3_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}

#if defined(BSP_USING_RGBLED_WITCH_PWM3_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void PWM3_CC1_DMA_IRQHandler(void)
{
    PWMx_DMA_IRQHandler(PWM3_INDEX);
}
#endif

#if defined(BSP_USING_RGBLED_WITCH_PWM3_DMA)
uint32_t bf0_gtim_get_channel(HAL_GPT_ActiveChannel       Channel)
{
    uint32_t i = 0, channel = 0;
    for (i = 0; i < 4; i++)
    {
        if ((0x01 << (i)) & (Channel))
        {
            channel = (i + 1);
            break;
        }
    }
    return channel;
}
void HAL_GPT_PWM_PulseFinishedCallback(GPT_HandleTypeDef *htim)
{
    uint32_t channel = bf0_gtim_get_channel(htim->Channel);
    rt_uint32_t tim_channel = 0x04 * (channel - 1);
    HAL_GPT_PWM_Stop_DMA(htim, tim_channel);
    HAL_PIN_Set(PAD_PA32, GPIO_A32, PIN_NOPULL, 1);   // RGB LED PIN
    rt_pin_mode(RGBLED_CONTROL_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RGBLED_CONTROL_PIN, PIN_LOW);
}
#endif

#endif


static rt_err_t drv_rgbled_control(struct rt_device_pwm *device, int cmd, void *arg);
static struct rt_pwm_ops drv_ops =
{
    drv_rgbled_control
};

static void creater_color_array(uint32_t color)
{
    uint16_t i;
    rt_memset(rgb_buff, 0, sizeof(uint16_t) * RGB_REST_LEN + RGB_COLOR_LEN + 1);
    uint8_t red = (uint8_t)((color & 0x00ff0000) >> 16);
    uint8_t green = (uint8_t)((color & 0x0000ff00) >> 8);
    uint8_t blue = (uint8_t)((color & 0x000000ff) >> 0);

    for (i = 0; i < 8; i++)
    {
        if ((green << i) & 0x80)
        {
            rgb_buff[i + RGB_REST_LEN] = reg_high;
        }
        else
        {
            rgb_buff[i + RGB_REST_LEN] = reg_low;
        }
    }
    for (i = 0; i < 8; i++)
    {
        if ((red << i) & 0x80)
        {
            rgb_buff[i + 8 + RGB_REST_LEN] = reg_high;
        }
        else
        {
            rgb_buff[i + 8 + RGB_REST_LEN] = reg_low;
        }
    }

    for (i = 0; i < 8; i++)
    {
        if ((blue << i) & 0x80)
        {
            rgb_buff[i + 16 + RGB_REST_LEN] = reg_high;
        }
        else
        {
            rgb_buff[i + 16 + RGB_REST_LEN] = reg_low;
        }
    }
    rgb_buff[RGB_REST_LEN + RGB_COLOR_LEN] = reg_end;

#ifdef DRV_DEBUG
    for (i = RGB_REST_LEN; i < RGB_REST_LEN + RGB_COLOR_LEN + RGB_STOP_LEN; i++)
    {
        rt_kprintf("%d,", rgb_buff[i]);
    }
#endif
    return;
}


/**
* @brief  rgbled update color.
* @param[in]  htim: GPT device object handle.
* @param[in]  configuration: GPT configuration, mainly GPT channel number, color.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_rgbled_update_color(struct bf0_rgbled *pwm_obj, struct rt_rgbled_configuration *configuration)
{
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t period, pulse;
    rt_uint32_t GPT_clock, psc;

    rt_uint32_t channel = 0x04 * (pwm_obj->channel - 1);
    GPT_HandleTypeDef *htim  = &(pwm_obj->tim_handle);
    htim->Channel = 0x01 * channel;
    DMA_HandleTypeDef *pwm_dma = &(pwm_obj->dma_handle);
    IRQn_Type hdma_pwm_irq   = pwm_obj->dma_irq;
    rt_uint32_t max_period = MAX_PERIOD_GPT;

#ifdef HAL_ATIM_MODULE_ENABLED
    if (IS_GPT_ADVANCED_INSTANCE(htim->Instance) != RESET)
        max_period = MAX_PERIOD_ATM;
#endif

#ifdef SF32LB52X
    if (htim->Instance == hwp_gptim2)
        GPT_clock = 24000000;
    else
#endif
        GPT_clock = HAL_RCC_GetPCLKFreq(htim->core, 1);

    //GPT_clock = SystemCoreClock;

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    GPT_clock /= 1000000UL;
    period = (unsigned long long)pwm_peroid * GPT_clock / 1000ULL ;
    psc = period / max_period + 1;
    period = period / psc;
    __HAL_GPT_SET_PRESCALER(htim, psc - 1);
#ifdef DRV_DEBUG
    LOG_I("psc %d, Period %d,", psc, period);
#endif
    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    __HAL_GPT_SET_AUTORELOAD(htim, period - 1);

    pulse = (unsigned long long)pulse_peroid * GPT_clock / psc / 1000ULL;

    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }
#ifdef DRV_DEBUG
    LOG_I("Pulse %d", pulse);
#endif
    __HAL_GPT_SET_COMPARE(htim, channel, pulse - 1);
    //__HAL_GPT_SET_COUNTER(htim, 0);

    /* Update frequency value */
    HAL_GPT_GenerateEvent(htim, GPT_EVENTSOURCE_UPDATE);
    GPT_OC_InitTypeDef oc_config = {0};
    oc_config.OCMode = GPT_OCMODE_PWM1;
    oc_config.Pulse = __HAL_GPT_GET_COMPARE(htim, channel);
    oc_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    oc_config.OCFastMode = GPT_OCFAST_DISABLE;
    if (HAL_GPT_PWM_ConfigChannel(htim, &oc_config, channel) != HAL_OK)
    {
        LOG_E("%x channel %d config failed", htim, configuration->channel);

        return RT_ERROR;
    }
#ifdef DRV_DEBUG
    LOG_I("pwm config: dir=%d, periphinc=%d, meminc =%d, perialign =%d, memalign=%d, mode =%d, pri= %d, request =%d;\n",
          pwm_dma->Init.Direction,
          pwm_dma->Init.PeriphInc,
          pwm_dma->Init.MemInc,
          pwm_dma->Init.PeriphDataAlignment,
          pwm_dma->Init.MemDataAlignment,
          pwm_dma->Init.Mode,
          pwm_dma->Init.Priority,
          pwm_dma->Init.Request);
#endif
    creater_color_array(configuration->color_rgb);

    HAL_DMA_Init(pwm_dma);
    __HAL_LINKDMA(htim, hdma[GPT_DMA_ID_CC1], pwm_obj->dma_handle);
    HAL_NVIC_SetPriority(hdma_pwm_irq, 0, 0);
    HAL_NVIC_EnableIRQ(hdma_pwm_irq);

    uint32_t *dma_data  = (uint32_t *)rgb_buff;
    uint16_t len = RGB_REST_LEN + RGB_REST_LEN + RGB_STOP_LEN;
    HAL_GPT_PWM_Start_DMA(htim, channel, dma_data, len);
#ifdef HAL_ATIM_MODULE_ENABLED
    if (configuration->is_comp)
        HAL_TIMEx_PWMN_Start(htim, channel);
#endif

    return RT_EOK;
}
/**
* @brief  rgbled controls.
* @param[in]  device: pwm device handle.
* @param[in]  cmd: control commands.
* @param[in]  arg: control command arguments.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_rgbled_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    struct rt_rgbled_configuration *configuration = (struct rt_rgbled_configuration *)arg;
    struct bf0_rgbled *pwm_obj  = (struct bf0_rgbled *) device->parent.user_data;
    GPT_HandleTypeDef *htim = &(pwm_obj->tim_handle);

    if ((RT_DEVICE_CTRL_RESUME != cmd) && (RT_DEVICE_CTRL_SUSPEND != cmd))
    {
        /* arg is not configuration for RESUME and SUSPEND command */
        RT_ASSERT(configuration->channel > 0); //Channel id must > 0
    }

    switch (cmd)
    {
    case PWM_CMD_SET_COLOR:
        drv_rgbled_update_color(pwm_obj, configuration);
        break;
    default:
        break;
    }
    return RT_EOK;
}

/**
* @} pwm_device
*/

/**
* @brief PWM device hardware initialization.
* @param[in]  device: pwm device handle.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t bf0_hw_pwm_init(struct bf0_rgbled *device)
{
    rt_err_t result = RT_EOK;
    GPT_HandleTypeDef *tim = RT_NULL;
    GPT_ClockConfigTypeDef clock_config = {0};

    RT_ASSERT(device != RT_NULL);

    tim = (GPT_HandleTypeDef *)&device->tim_handle;

    /* configure the timer to pwm mode */
    tim->Init.Prescaler = 0;
    tim->Init.CounterMode = GPT_COUNTERMODE_UP;
    tim->Init.Period = 0;

    if (HAL_GPT_Base_Init(tim) != HAL_OK)
    {
        LOG_E("%s time base init failed", device->name);
        result = -RT_ERROR;
        goto __exit;
    }

    clock_config.ClockSource = GPT_CLOCKSOURCE_INTERNAL;
    if (HAL_GPT_ConfigClockSource(tim, &clock_config) != HAL_OK)
    {
        LOG_E("%s clock init failed", device->name);
        result = -RT_ERROR;
        goto __exit;
    }

    if (HAL_GPT_PWM_Init(tim) != HAL_OK)
    {
        LOG_E("%s pwm init failed", device->name);
        result = -RT_ERROR;
        goto __exit;
    }

#ifdef BSP_PWM3_USING_DMA

#endif
    /* pwm pin configuration */
    //HAL_GPT_MspPostInit(tim);

    /* enable update request source */
    __HAL_GPT_URS_ENABLE(tim);
__exit:
    return result;
}

static void bf0_hw_pwm_config_dma(struct bf0_rgbled *device)
{
    device->dma_handle.Init.Direction          = DMA_MEMORY_TO_PERIPH;
    device->dma_handle.Init.PeriphInc          = DMA_PINC_DISABLE;
    device->dma_handle.Init.MemInc             = DMA_MINC_ENABLE;
    device->dma_handle.Init.PeriphDataAlignment    = DMA_PDATAALIGN_HALFWORD;
    device->dma_handle.Init.MemDataAlignment   = DMA_MDATAALIGN_HALFWORD;
    device->dma_handle.Init.Mode               = DMA_NORMAL;     /*DMA use circular mode*/
    device->dma_handle.Init.Priority           = DMA_PRIORITY_LOW;
}

/**
* @brief RGBLED device driver initialization.
* This is entry function of RGBLED device driver.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static int bf0_rgbled_init(void)
{
    int i = 0;
    int result = RT_EOK;

    for (i = 0; i < sizeof(bf0_rgbled_obj) / sizeof(bf0_rgbled_obj[0]); i++)
    {
        /* pwm init */
        if (bf0_hw_pwm_init(&bf0_rgbled_obj[i]) != RT_EOK)
        {
            LOG_E("%s init failed", bf0_rgbled_obj[i].name);
            result = -RT_ERROR;
            goto __exit;
        }
        else
        {
            LOG_D("%s init success", bf0_rgbled_obj[i].name);
            bf0_hw_pwm_config_dma(&bf0_rgbled_obj[i]);
            /* register pwm device */
            if (rt_device_pwm_register(rt_calloc(1, sizeof(struct rt_device_pwm)), bf0_rgbled_obj[i].name, &drv_ops, &bf0_rgbled_obj[i]) == RT_EOK)
            {

                LOG_D("%s register success", bf0_rgbled_obj[i].name);
            }
            else
            {
                LOG_E("%s register failed", bf0_rgbled_obj[i].name);
                result = -RT_ERROR;
            }
        }
    }

__exit:
    return result;
}
INIT_DEVICE_EXPORT(bf0_rgbled_init);

/// @} drv_pwm
/// @} bsp_driver


#define DRV_TEST
#ifdef DRV_TEST

#include <finsh.h>

/** @addtogroup bsp_sample BSP driver sample commands.
  * @{
  */

/** @defgroup bsp_sample_pwm PWM sample commands
  * @ingroup bsp_sample
  * @brief PWM sample commands
  *
  * These sample commands demonstrate the usage of pwm driver.
  * User need to connect a buzzer to PWM output for testing.
  * @{
  */

/**
* @brief rgbled color display device.
* Usage: pwm_set [device] [channel] [period] [pulse]
*
* example: pwm_set pwm3 1 1000000 500
*/


static int rgb_color(int argc, char **argv)
{
    int result = 0;
    struct rt_device_pwm *rgbled_device;
    /*rgbled poweron*/
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, true, true);

    if (argc != 2)
    {
        LOG_I("Usage: rgb_color <rgb>");
        result = -RT_ERROR;
        goto _exit;
    }
    uint32_t color = atoh(argv[2]);

    rgbled_device = (struct rt_device_pwm *)rt_device_find("rgbled");
    if (!rgbled_device)
    {
        result = -RT_EIO;
        goto _exit;
    }
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1);   // RGB LED
    struct rt_rgbled_configuration configuration;
    configuration.color_rgb = color;
    rt_device_control(&rgbled_device->parent, PWM_CMD_SET_COLOR, &configuration);
_exit:
    return result;
}
MSH_CMD_EXPORT(rgb_color, rgb_color 0xffffff);

#endif /* DRV_TEST */

#endif /* RT_USING_PWM */

/// @} bsp_sample_pwm

/// @} bsp_sample


/// @} file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
