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
#include <string.h>
#include "bf0_hal_lptim.h"
/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_pwm PWM driver
  * @ingroup bsp_driver
  * @brief PWM BSP driver
  * @{
  */

#if defined(BSP_USING_PWM_LPTIM1) ||defined(BSP_USING_PWM_LPTIM2)||defined(BSP_USING_PWM_LPTIM3)|| defined(_SIFLI_DOXYGEN_)

#include "drv_config.h"
#include "drv_pwm_lptim.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.pwm"
#include <drv_log.h>





enum
{
#ifdef BSP_USING_PWM_LPTIM1
    PWM_LPTIM1_INDEX,
#endif
#ifdef BSP_USING_PWM_LPTIM2
    PWM_LPTIM2_INDEX,
#endif
#ifdef BSP_USING_PWM_LPTIM3
    PWM_LPTIM3_INDEX,
#endif
};


static struct bf0_pwm_lp bf0_pwm_obj[] =
{
#ifdef BSP_USING_PWM_LPTIM1
    PWM_LPTIM1_CONFIG,
#endif

#ifdef BSP_USING_PWM_LPTIM2
    PWM_LPTIM2_CONFIG,
#endif

#ifdef BSP_USING_PWM_LPTIM3
    PWM_LPTIM3_CONFIG,
#endif
};

/** @defgroup pwm_device PWM device functions registered to OS
 * @ingroup drv_pwm
 * @{
 */

static rt_err_t drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg);
static struct rt_pwm_ops drv_ops =
{
    drv_pwm_control
};

/**
* @brief  Enable/disable a PWM device.
* @param[in]  hpwm: pwm device object handle.
* @param[in]  configuration: GPT configuration, mainly GPT channel number.
* @param[in]  enable: 1 enable, 0 disable.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t lp_timer_freq_div_calc(struct bf0_pwm_lp *hpwm, rt_uint32_t *period_ns, rt_uint32_t *pulse_ns, rt_uint32_t *psc_reg)
{
    rt_uint32_t period, pulse, lptimer_freq, psc_temp, fpclk2;
    LPTIM_HandleTypeDef *tim = &(hpwm->tim_handle);

    uint32_t psc[8] = {LPTIM_PRESCALER_DIV1, LPTIM_PRESCALER_DIV2, LPTIM_PRESCALER_DIV4, LPTIM_PRESCALER_DIV8, LPTIM_PRESCALER_DIV16,
                       LPTIM_PRESCALER_DIV32, LPTIM_PRESCALER_DIV64, LPTIM_PRESCALER_DIV128
                      };

    uint32_t div[8] = {1, 2, 4, 8, 16, 32, 64, 128};

    period = *period_ns;
    pulse = *pulse_ns;

    LOG_I("1 Psc %x, Period %d, Pulse %d\n", *psc_reg, *period_ns, *pulse_ns);
    if ((tim->Init.Clock.Source == LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC) && (tim->Init.Clock.IntSource == LPTIM_INTCLOCKSOURCE_LPCLOCK))
    {
        lptimer_freq = HAL_LPTIM_GetFreq();
    }
    else if ((tim->Init.Clock.Source == LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC) && (tim->Init.Clock.IntSource == LPTIM_INTLOCKSOURCE_APBCLOCK))
    {
#ifdef SOC_BF0_HCPU
        lptimer_freq = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 0);/*get pclk2*/
#else
        lptimer_freq = HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 0);/*get pclk2*/
#endif

    }

    period = (rt_uint32_t)((float)(period / 1000) * lptimer_freq / (float)1000000);
    pulse = (rt_uint32_t)((float)(pulse / 1000) * lptimer_freq / (float)1000000);

    psc_temp = period / MAX_PERIOD + 1; /* used to  caculate prescale*/

    if (psc_temp > 128)
    {
        LOG_E("period over range %d !\n", *period_ns);
        return -RT_ERROR;
    }

    for (int m = 0; m < 8; m++)
    {
        if (psc_temp <= div[m])
        {
            *psc_reg = psc[m];
            *period_ns = period / div[m];
            if (*period_ns < MIN_PERIOD)
            {
                *period_ns = MIN_PERIOD;
            }

            *pulse_ns =  pulse / div[m];
            if (*pulse_ns < MIN_PULSE)
            {
                *pulse_ns = MIN_PULSE;
            }
            else if (*pulse_ns > *period_ns)
            {
                *pulse_ns = *period_ns;
            }

            LOG_I("2 Psc %x, Period %d, Pulse %d\n", *psc_reg, *period_ns, *pulse_ns);

            break;
        }
    }
    return RT_EOK;
}



static rt_err_t drv_pwm_enable(struct bf0_pwm_lp *hpwm, struct rt_pwm_configuration *configuration, rt_bool_t enable)
{
    rt_err_t ret = RT_EOK;

    if (!enable)
    {
        HAL_LPTIM_PWM_Stop(&(hpwm->tim_handle));
    }
    else
    {
        rt_uint32_t period, pulse, psc_reg;
        if (configuration->period > 0)
            hpwm->config.period = configuration->period;
        if (configuration->pulse > 0)
            hpwm->config.pulse = configuration->pulse;

        period = hpwm->config.period;
        pulse = hpwm->config.pulse;

        LOG_I("0 Psc %x, Period %d, Pulse %d\n", psc_reg, period, pulse);

        ret = lp_timer_freq_div_calc(hpwm, &period, &pulse, &psc_reg);
        if (RT_EOK != ret)
        {
            return ret;
        }

        LOG_I("3 Psc %x, Period %d, Pulse %d\n", psc_reg, period, pulse);

        HAL_LPTIM_PWM_Start(&(hpwm->tim_handle), period, pulse, psc_reg);
    }

    return RT_EOK;
}

/**
* @brief  Get a PWM device configuration.
* @param[in]  hpwm: pwm device object handle.
* @param[out] configuration: pwm configuration, return period and pulse.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_get(struct bf0_pwm_lp *hpwm, struct rt_pwm_configuration *configuration)
{
    memcpy(configuration, &(hpwm->config), sizeof(struct rt_pwm_configuration));
    return RT_EOK;
}

/**
* @brief  Set a PWM device configuration.
* @param[in]  hpwm: pwm device object handle.
* @param[in]  configuration: pwm configuration, input mainly period and pulse.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_set(struct bf0_pwm_lp *hpwm, struct rt_pwm_configuration *configuration)
{
    memcpy(&(hpwm->config), configuration, sizeof(struct rt_pwm_configuration));
    return RT_EOK;
}

/**
* @brief  Set a PWM device period.
* @param[in]  hpwm: pwm device object handle.
* @param[in]  configuration: PWM configuration, input mainly period and pulse.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_set_period(struct bf0_pwm_lp *hpwm, struct rt_pwm_configuration *configuration)
{
    rt_uint32_t period, pulse, psc_reg;
    rt_err_t ret = RT_EOK;
    if (configuration->period > 0)
        hpwm->config.period = configuration->period;
    if (configuration->pulse > 0)
        hpwm->config.pulse = configuration->pulse;

    period = hpwm->config.period;
    pulse = hpwm->config.pulse;

    ret = lp_timer_freq_div_calc(hpwm, &period, &pulse, &psc_reg);
    if (RT_EOK != ret)
    {
        return ret;
    }
    HAL_LPTIM_PWM_Set_Period(&(hpwm->tim_handle), period, pulse, psc_reg);
    return RT_EOK;
}

/**
* @brief  Set a PWM device clocksource.
* @param[in]  hpwm: pwm device object handle.
* @param[in]  configuration: PWM configuration, clock source user  reserved.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/

static rt_err_t drv_pwm_set_clocksource(struct bf0_pwm_lp *hpwm, struct rt_pwm_configuration *configuration)
{
    uint8_t clk_source = 0;
    rt_err_t result = RT_EOK;
    LPTIM_HandleTypeDef *tim = &(hpwm->tim_handle);

    if (!hpwm)
    {
        LOG_I("device is'nt exist!");
        result = -RT_ERROR;
    }
    if (!configuration)
    {
        LOG_I("config is null!");
        result = -RT_ERROR;
    }
    clk_source = configuration->reserved;
    switch (clk_source)
    {
    case LPTIME_PWM_CLK_SOURCE_USING_LPCLK:
    {
        tim->Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;  /*use inter clk source*/
        tim->Init.Clock.IntSource = LPTIM_INTCLOCKSOURCE_LPCLOCK;   /*use pclk2 as inter clk source*/
    }
    break;
    case LPTIME_PWM_CLK_SOURCE_USING_APBCLK:
    {
        tim->Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;  /*use inter clk source*/
        tim->Init.Clock.IntSource = LPTIM_INTLOCKSOURCE_APBCLOCK;   /*use pclk2 as inter clk source*/

    }
    break;
    case LPTIME_PWM_CLK_SOURCE_USING_EXTER_CLK:
    {
        /*need to add*/
    }
    break;
    default:
        break;
    }

    if (HAL_LPTIM_Init(tim) != HAL_OK)
    {
        LOG_E("%s set clock source failed", hpwm->name);
        result = -RT_ERROR;
        goto __exit;
    }
__exit:
    return result;
}
/**
* @brief  pwm controls.
* @param[in]  device: pwm device handle.
* @param[in]  cmd: control commands.
* @param[in]  arg: control command arguments.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;
    struct bf0_pwm_lp *hpwm = (struct bf0_pwm_lp *)device->parent.user_data;

    //LOG_I("period %d pulse %d", configuration->period, configuration->pulse);

    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        return drv_pwm_enable(hpwm, configuration, RT_TRUE);
    case PWM_CMD_DISABLE:
        return drv_pwm_enable(hpwm, configuration, RT_FALSE);
    case PWM_CMD_SET:
        return drv_pwm_set(hpwm, configuration);
    case PWM_CMD_GET:
        return drv_pwm_get(hpwm, configuration);
    case PWM_CMD_SET_PERIOD:
        return drv_pwm_set_period(hpwm, configuration);
    case PWM_CMD_SET_CLK_SOURECE:
        return drv_pwm_set_clocksource(hpwm, configuration);
    default:
        return RT_EINVAL;
    }
}

/**
* @} pwm_device
*/

/**
* @brief PWM device hardware initialization.
* @param[in]  device: pwm device handle.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t bf0_hw_pwm_init(struct bf0_pwm_lp *device)
{
    rt_err_t result = RT_EOK;
    LPTIM_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(device != RT_NULL);

    tim = (LPTIM_HandleTypeDef *)&device->tim_handle;

    memset(&(tim->Init), 0, sizeof(tim->Init));

    HAL_LPTIM_InitDefault(tim);
    if (HAL_LPTIM_Init(tim) != HAL_OK)
    {
        LOG_E("%s time base init failed", device->name);
        result = -RT_ERROR;
        goto __exit;
    }

    /* pwm pin configuration */
    //HAL_LPTim_MspPostInit(tim);

__exit:
    return result;
}



/**
* @brief PWM device driver initialization.
* This is entry function of PWM device driver.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static int bf0_pwm_init_lp(void)
{
    int i = 0;
    int result = RT_EOK;

    for (i = 0; i < sizeof(bf0_pwm_obj) / sizeof(bf0_pwm_obj[0]); i++)
    {
        /* pwm init */
        if (bf0_hw_pwm_init(&bf0_pwm_obj[i]) != RT_EOK)
        {
            LOG_E("%s init failed", bf0_pwm_obj[i].name);
            result = -RT_ERROR;
            goto __exit;
        }
        else
        {
            LOG_D("%s init success", bf0_pwm_obj[i].name);

            /* register pwm device */
            if (rt_device_pwm_register(rt_calloc(1, sizeof(struct rt_device_pwm)), bf0_pwm_obj[i].name, &drv_ops, &bf0_pwm_obj[i]) == RT_EOK)
            {

                LOG_D("%s register success", bf0_pwm_obj[i].name);
            }
            else
            {
                LOG_E("%s register failed", bf0_pwm_obj[i].name);
                result = -RT_ERROR;
            }
        }
    }

__exit:
    return result;
}
INIT_DEVICE_EXPORT(bf0_pwm_init_lp);

/// @} drv_pwm
/// @} bsp_driver

#endif /* RT_USING_PWM */

/// @} bsp_sample_pwm

/// @} bsp_sample


/// @} file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
