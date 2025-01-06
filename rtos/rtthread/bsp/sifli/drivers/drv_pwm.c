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

/** @defgroup drv_pwm PWM driver
  * @ingroup bsp_driver
  * @brief PWM BSP driver
  * @{
  */

#if defined(BSP_USING_PWM) || defined(_SIFLI_DOXYGEN_)

#include "drv_config.h"


//#define DRV_DEBUG
#define LOG_TAG             "drv.pwm"
#include <drv_log.h>

#define MAX_PERIOD_GPT (0xFFFF)
#define MAX_PERIOD_ATM (0xFFFFFFFF)
#define MIN_PERIOD 3
#define MIN_PULSE 2

//extern void HAL_GPT_MspPostInit(GPT_HandleTypeDef *htim);

enum
{
#ifdef BSP_USING_PWM1
    PWM1_INDEX,
#endif
#ifdef BSP_USING_PWM2
    PWM2_INDEX,
#endif
#ifdef BSP_USING_PWM3
    PWM3_INDEX,
#endif
#ifdef BSP_USING_PWM4
    PWM4_INDEX,
#endif
#ifdef BSP_USING_PWM5
    PWM5_INDEX,
#endif
#ifdef BSP_USING_PWM6
    PWM6_INDEX,
#endif
#ifdef BSP_USING_PWMA1
    PWMA1_INDEX,
#endif
#ifdef BSP_USING_PWMA2
    PWMA2_INDEX,
#endif

};

struct bf0_pwm
{
    struct rt_device_pwm pwm_device;    /*!<PWM device object handle*/
    GPT_HandleTypeDef    tim_handle;    /*!<General Purpose Timer(GPT) device object handle used in PWM*/
    rt_uint8_t channel;                 /*!<GPT channel used*/
    char *name;                         /*!<Device name*/
};

static struct bf0_pwm bf0_pwm_obj[] =
{
#ifdef BSP_USING_PWM1
    PWM1_CONFIG,
#endif

#ifdef BSP_USING_PWM2
    PWM2_CONFIG,
#endif

#ifdef BSP_USING_PWM3
    PWM3_CONFIG,
#endif

#ifdef BSP_USING_PWM4
    PWM4_CONFIG,
#endif

#ifdef BSP_USING_PWM5
    PWM5_CONFIG,
#endif

#ifdef BSP_USING_PWM6
    PWM6_CONFIG,
#endif
#ifdef BSP_USING_PWMA1
    PWMA1_CONFIG,
#endif
#ifdef BSP_USING_PWMA2
    PWMA2_CONFIG,
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
* @param[in]  htim: GPT device object handle.
* @param[in]  configuration: GPT configuration, mainly GPT channel number.
* @param[in]  enable: 1 enable, 0 disable.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_enable(GPT_HandleTypeDef *htim, struct rt_pwm_configuration *configuration, rt_bool_t enable)
{
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = 0x04 * (configuration->channel - 1);

    if (!enable)
    {
        HAL_GPT_PWM_Stop(htim, channel);
#ifdef HAL_ATIM_MODULE_ENABLED
        if (configuration->is_comp)
            HAL_TIMEx_PWMN_Stop(htim, channel);
#endif
    }
    else
    {
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

        HAL_GPT_PWM_Start(htim, channel);
#ifdef HAL_ATIM_MODULE_ENABLED
        if (configuration->is_comp)
            HAL_TIMEx_PWMN_Start(htim, channel);
#endif
    }

    return RT_EOK;
}

/**
* @brief  Get a PWM device configuration.
* @param[in]  htim: GPT device object handle.
* @param[in,out]  configuration: GPT configuration, input mainly GPT channel number. return period and pulse.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_get(GPT_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
{
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = 0x04 * (configuration->channel - 1);
    rt_uint64_t GPT_clock;

#ifdef SF32LB52X
    if (htim->Instance == hwp_gptim2)
        GPT_clock = 24000000;
    else
#endif
        GPT_clock = HAL_RCC_GetPCLKFreq(htim->core, 1);


    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    GPT_clock /= 1000000UL;
    configuration->period = (__HAL_GPT_GET_AUTORELOAD(htim) + 1) * (htim->Instance->PSC + 1) * 1000UL / GPT_clock;
    configuration->pulse = (__HAL_GPT_GET_COMPARE(htim, channel) + 1) * (htim->Instance->PSC + 1) * 1000UL / GPT_clock;

    return RT_EOK;
}

/**
* @brief  Set a PWM device configuration.
* @param[in]  htim: GPT device object handle.
* @param[in]  configuration: GPT configuration, input mainly GPT channel number, period and pulse.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_set(GPT_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
{
    rt_uint32_t period, pulse;
    rt_uint32_t GPT_clock, psc;
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = 0x04 * (configuration->channel - 1);
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
    period = (unsigned long long)configuration->period * GPT_clock / 1000ULL ;
    psc = period / max_period + 1;
    period = period / psc;
    __HAL_GPT_SET_PRESCALER(htim, psc - 1);

    LOG_I("psc %d, Period %d,", psc, period);

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    __HAL_GPT_SET_AUTORELOAD(htim, period - 1);

    pulse = (unsigned long long)configuration->pulse * GPT_clock / psc / 1000ULL;
    LOG_I("Pulse %d", pulse);
    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }
    __HAL_GPT_SET_COMPARE(htim, channel, pulse - 1);
    //__HAL_GPT_SET_COUNTER(htim, 0);

    /* Update frequency value */
    HAL_GPT_GenerateEvent(htim, GPT_EVENTSOURCE_UPDATE);

    return RT_EOK;
}

/**
* @brief  Set a PWM device configuration.
* @param[in]  htim: GPT device object handle.
* @param[in]  configuration: GPT configuration, input mainly GPT channel number and period.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_set_period(GPT_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
{
    rt_uint32_t period;
    rt_uint32_t GPT_clock, psc;
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

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    GPT_clock /= 1000000UL;
    period = (unsigned long long)configuration->period * GPT_clock / 1000ULL ;
    psc = period / max_period + 1;
    period = period / psc;

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    __HAL_GPT_SET_AUTORELOAD(htim, period - 1);

    /* Update frequency value */
    HAL_GPT_GenerateEvent(htim, GPT_EVENTSOURCE_UPDATE);

    return RT_EOK;
}

static rt_err_t drv_pwm_set_break_dead(GPT_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
{
#ifdef HAL_ATIM_MODULE_ENABLED
    TIMEx_BreakDeadTimeConfigTypeDef bdt = {0};
    RT_ASSERT((configuration != NULL) && (htim != NULL))
    struct rt_pwm_break_dead *bkd = (struct rt_pwm_break_dead *)&configuration->break_dead;
    rt_uint32_t GPT_clock = 0, dead_time = 0;

#ifdef SF32LB52X
    if (htim->Instance == hwp_gptim2)
        GPT_clock = 24000000;
    else
#endif
        GPT_clock = HAL_RCC_GetPCLKFreq(htim->core, 1);

    GPT_clock /= 1000000UL;

    if (bkd->dptsc) //dead-time is tCLK*(DTG+1)*16, step=(1/pclk)*16
    {
        dead_time = (unsigned long long)configuration->dead_time * GPT_clock / 16000ULL - 1;
        if (dead_time > 1023)
        {
            LOG_E("%s err dead_time beyond the range (max:%dns) that can be set", __FUNCTION__, (16 * 1024 * 1000 / GPT_clock));
            return RT_ERROR;
        }
    }
    else //dead-time is tCLK*(DTG+1), step=(1/pclk)
    {
        dead_time = (unsigned long long)configuration->dead_time * GPT_clock / 1000ULL - 1;
        if (dead_time > 1023)
        {
            LOG_E("%s err dead_time beyond the range (max:%dns) that can be set", __FUNCTION__, (1024 * 1000 / GPT_clock));
        }
    }
    if (dead_time)
        bdt.DeadTime = dead_time;
    else
        bdt.DeadTime = bkd->dtg;
    bdt.AutomaticOutput = bkd->aoe;
    bdt.BreakFilter = bkd->bkf;
    bdt.BreakPolarity = bkd->bkp;
    bdt.BreakState = bkd->bke;
    bdt.Break2Filter = bkd->bk2f;
    bdt.Break2Polarity = bkd->bk2p;
    bdt.Break2State = bkd->bk2e;
    bdt.DeadTimePsc = bkd->dptsc;
    bdt.LockLevel = 0;
    bdt.OffStateIDLEMode = bkd->ossi;
    bdt.OffStateRunMode = bkd->ossr;
    HAL_TIMEx_ConfigBreakDeadTime(htim, &bdt);
#endif

    return RT_EOK;
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
    GPT_HandleTypeDef *htim = (GPT_HandleTypeDef *)device->parent.user_data;

    if ((RT_DEVICE_CTRL_RESUME != cmd) && (RT_DEVICE_CTRL_SUSPEND != cmd))
    {
        /* arg is not configuration for RESUME and SUSPEND command */
        RT_ASSERT(configuration->channel > 0); //Channel id must > 0
    }

    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        return drv_pwm_enable(htim, configuration, RT_TRUE);
    case PWM_CMD_DISABLE:
        return drv_pwm_enable(htim, configuration, RT_FALSE);
    case PWM_CMD_SET:
        return drv_pwm_set(htim, configuration);
    case PWM_CMD_GET:
        return drv_pwm_get(htim, configuration);
    case PWM_CMD_SET_PERIOD:
        return drv_pwm_set_period(htim, configuration);
    case PWM_CMD_BREAK_DEAD:
        return drv_pwm_set_break_dead(htim, configuration);
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
static rt_err_t bf0_hw_pwm_init(struct bf0_pwm *device)
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

    /* pwm pin configuration */
    //HAL_GPT_MspPostInit(tim);

    /* enable update request source */
    __HAL_GPT_URS_ENABLE(tim);

__exit:
    return result;
}


/**
* @brief PWM device driver initialization.
* This is entry function of PWM device driver.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static int bf0_pwm_init(void)
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
            if (rt_device_pwm_register(rt_calloc(1, sizeof(struct rt_device_pwm)), bf0_pwm_obj[i].name, &drv_ops, &bf0_pwm_obj[i].tim_handle) == RT_EOK)
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
INIT_DEVICE_EXPORT(bf0_pwm_init);

/// @} drv_pwm
/// @} bsp_driver


//#define DRV_TEST
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

FINSH_FUNCTION_EXPORT_ALIAS(rt_pwm_enable2, pwm_enable, enable pwm by channel.);
FINSH_FUNCTION_EXPORT_ALIAS(rt_pwm_set, pwm_set, set pwm.);

/**
* @brief enable pwm device.
* Usage: pwm_enable [device] [channel]
*
* example: pwm_enable pwm3 1
*/
static int pwm_enable(int argc, char **argv)
{
    int result = 0;
    struct rt_device_pwm *device = RT_NULL;

    if (argc != 3)
    {
        LOG_I("Usage: pwm_enable <pwm device> <channel>");
        result = -RT_ERROR;
        goto _exit;
    }

    device = (struct rt_device_pwm *)rt_device_find(argv[1]);
    if (!device)
    {
        result = -RT_EIO;
        LOG_I("open %s failed 1", argv[1]);
        goto _exit;
    }

    result = rt_pwm_enable(device, atoi(argv[2]));

_exit:
    return result;
}
MSH_CMD_EXPORT(pwm_enable, pwm_enable pwm3 1);

/**
* @brief disable pwm device.
* Usage: pwm_disable [device] [channel]
*
* example: pwm_disable pwm3 1
*/
static int pwm_disable(int argc, char **argv)
{
    int result = 0;
    struct rt_device_pwm *device = RT_NULL;

    if (argc != 3)
    {
        LOG_I("Usage: pwm_disable <pwm device> <channel>");
        result = -RT_ERROR;
        goto _exit;
    }

    device = (struct rt_device_pwm *)rt_device_find(argv[1]);
    if (!device)
    {
        result = -RT_EIO;
        goto _exit;
    }

    result = rt_pwm_disable(device, atoi(argv[2]));

_exit:
    return result;
}
MSH_CMD_EXPORT(pwm_disable, pwm_disable pwm3 1);


/**
* @brief Setup pwm device.
* Usage: pwm_set [device] [channel] [period] [pulse]
*
* example: pwm_set pwm3 1 1000000 500
*/
static int pwm_set(int argc, char **argv)
{
    int result = 0;
    struct rt_device_pwm *device = RT_NULL;

    if (argc != 5)
    {
        LOG_I("Usage: pwm_set <pwm device> <channel> <period> <pulse>");
        result = -RT_ERROR;
        goto _exit;
    }

    device = (struct rt_device_pwm *)rt_device_find(argv[1]);
    if (!device)
    {
        result = -RT_EIO;
        goto _exit;
    }
    result = rt_pwm_set(device, atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));

_exit:
    return result;
}
MSH_CMD_EXPORT(pwm_set, pwm_set pwm3 1 1000000 500);

#define VCO 1000000000
#define L1  VCO/262
#define L1S VCO/277
#define L2  VCO/294
#define L2S VCO/311
#define L3  VCO/330
#define L4  VCO/349
#define L4S VCO/370
#define L5  VCO/392
#define L5S VCO/415
#define L6  VCO/440
#define L6S VCO/466
#define L7  VCO/494
#define M1  VCO/523
#define M1S VCO/554
#define M2  VCO/587
#define M2S VCO/622
#define M3  VCO/659
#define M4  VCO/698
#define M4S VCO/740
#define M5  VCO/784
#define M5S VCO/831
#define M6  VCO/880
#define M6S VCO/932
#define M7  VCO/988
#define H1  VCO/1046
#define H1S VCO/1109
#define H2  VCO/1175
#define H2S VCO/1245
#define H3  VCO/1318
#define H4  VCO/1397
#define H4S VCO/1480
#define H5  VCO/1568
#define H5S VCO/1661
#define H6  VCO/1760
#define H6S VCO/1865
#define H7  VCO/1976
#define HH1 VCO/2092
#define HH1S VCO/2218
#define M0  100
uint32_t notes[] =
{
    L1 /*00*/, L1S/*01*/, L2 /*02*/, L2S/*03*/, L3 /*04*/, L4/*05*/,
    L4S/*06*/, L5 /*07*/, L5S/*08*/, L6 /*09*/, L6S/*0A*/, L7/*0B*/,
    M1 /*0C*/, M1S/*0D*/, M2 /*0E*/, M2S/*0F*/, M3 /*10*/, M4/*11*/,
    M4S/*12*/, M5 /*13*/, M5S/*14*/, M6 /*15*/, M6S/*16*/, M7/*17*/,
    H1 /*18*/, H1S/*19*/, H2 /*1A*/, H2S/*1B*/, H3 /*1C*/, H4/*1D*/,
    H4S/*1E*/, H5 /*1F*/, H5S/*20*/, H6 /*21*/, H6S/*22*/, H7/*23*/,
    M0 /*24*/,
};

static rt_err_t rt_pwm_set_period(struct rt_device_pwm *device, int channel, rt_uint32_t period, rt_uint32_t pulse)
{
    rt_err_t result = RT_EOK;
    struct rt_pwm_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }

    configuration.channel = channel;
    configuration.period = period;
    configuration.pulse = pulse;
    result = rt_device_control(&device->parent, PWM_CMD_SET_PERIOD, &configuration);

    return result;
}

/**
* @brief Play notes on a pwm device.
* Usage: pwm_set [device] [channel] [delay] [note number] [notes]
*
* example: pwm_play     pwm3 1 1000 07 00010203040506
*/
static int pwm_play(int argc, char **argv)
{
    int result = 0;
    int len, i;
    struct rt_device_pwm *device = RT_NULL;

    if (argc != 6)
    {
        LOG_I("Usage: pwm_play <pwm device> <channel> <delay> <note number> <notes>");
        result = -RT_ERROR;
        goto _exit;
    }

    device = (struct rt_device_pwm *)rt_device_find(argv[1]);
    if (!device)
    {
        result = -RT_EIO;
        goto _exit;
    }

    len = atoi(argv[4]);
    rt_pwm_enable(device, atoi(argv[2]));
    for (i = 0; i < atoi(argv[4]); i++)
    {
        int index = (uint8_t)atoh(&(argv[5][i * 2]));
        LOG_I("Play %d, %d", index, notes[i]);
        result = rt_pwm_set(device, atoi(argv[2]), notes[index], 1000);
        rt_thread_mdelay(atoi(argv[3]));
    }
    rt_pwm_disable(device, atoi(argv[2]));

_exit:
    return result;
}
MSH_CMD_EXPORT(pwm_play, pwm_play     pwm3 1 1000 07 00010203040506);

#endif /* DRV_TEST */

#endif /* RT_USING_PWM */

/// @} bsp_sample_pwm

/// @} bsp_sample


/// @} file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
