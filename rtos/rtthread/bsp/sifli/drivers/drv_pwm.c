/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <board.h>
#include "bf0_hal_rcc.h"

#if defined(BSP_USING_PWM1) || defined(_SIFLI_DOXYGEN_)

#include "drv_config.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.pwm"
#include <drv_log.h>

#define MAX_PERIOD_GPT (0xFFFFFFFF)
#define MIN_PERIOD 3
#define MIN_PULSE 1
#define PWM_CLK_FREQ (48000000UL) /* 48MHz */

//extern void HAL_PWM_MspPostInit(PWM_HandleTypeDef *hpwm);

enum
{
#ifdef BSP_USING_PWM1
    PWM1_INDEX,
#endif
};
struct bf0_pwm_dma
{
    DMA_HandleTypeDef   dma_handle;      /*!< DMA device Handle used by this driver */
    IRQn_Type           dma_irq;
    uint16_t            dma_handle_index;
    uint8_t             flag_dma_eanbled;
};

struct bf0_pwm
{
    struct rt_device_pwm pwm_device;    /*!<PWM device object handle*/
    PWM_HandleTypeDef    pwm_handle;    /*!<General Purpose Timer(GPT) device object handle used in PWM*/
    rt_uint8_t channel;                 /*!<GPT channel used*/
    char *name;                         /*!<Device name*/
    struct bf0_pwm_dma *pwm_cc_dma[4];
    struct bf0_pwm_dma *pwm_update_dma;
};

static struct bf0_pwm bf0_pwm_obj[] =
{
#ifdef BSP_USING_PWM1
    PWM1_CONFIG,
#endif
};
static void pwm_get_dma_info(void)
{
    /*PWM1 CC1 DMA*/
#ifdef BSP_PWM1_CC1_USING_DMA
    {
        static struct bf0_pwm_dma pwm1_cc1_dma = PWM1_CC1_DMA_CONFIG;
        bf0_pwm_obj[PWM1_INDEX].pwm_cc_dma[0] = &pwm1_cc1_dma;
    }
#endif

    /*PWM1 CC2 DMA*/
#ifdef BSP_PWM1_CC2_USING_DMA
    {
        static struct bf0_pwm_dma pwm1_cc2_dma = PWM1_CC2_DMA_CONFIG;
        bf0_pwm_obj[PWM1_INDEX].pwm_cc_dma[1] = &pwm1_cc2_dma;
    }
#endif

    /*PWM1 CC3 DMA*/
#ifdef BSP_PWM1_CC3_USING_DMA
    {
        static struct bf0_pwm_dma pwm1_cc3_dma = PWM1_CC3_DMA_CONFIG;
        bf0_pwm_obj[PWM1_INDEX].pwm_cc_dma[2] = &pwm1_cc3_dma;
    }
#endif

    /*PWM1 CC4 DMA*/
#ifdef BSP_PWM1_CC4_USING_DMA
    {
        static struct bf0_pwm_dma pwm1_cc4_dma = PWM1_CC4_DMA_CONFIG;
        bf0_pwm_obj[PWM1_INDEX].pwm_cc_dma[3] = &pwm1_cc4_dma;
    }
#endif

}

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
* @param[in]  hpwm: GPT device object handle.
* @param[in]  configuration: GPT configuration, mainly GPT channel number.
* @param[in]  enable: 1 enable, 0 disable.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_enable(struct bf0_pwm *pwm, struct rt_pwm_configuration *configuration, rt_bool_t enable)
{
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = configuration->channel - 1;
    PWM_HandleTypeDef *hpwm = &(pwm->pwm_handle);
    struct bf0_pwm_dma *pwm_dma;

    if (configuration->dma_type)
        pwm_dma = pwm->pwm_cc_dma[configuration->channel - 1];
    else
        pwm_dma = pwm->pwm_update_dma;

    if (!enable)
    {
        if (pwm_dma)
        {
//TODO:fixme DMA stop function
            RT_ASSERT(0);
#if 0
            pwm_dma->flag_dma_eanbled = 0;

            if (pwm_dma->dma_handle_index == PWM_DMA_ID_UPDATE) /*Stop TIM Update DMA*/
                HAL_PWM_Update_Stop_DMA(hpwm, channel);
            else                                                /*Stop TIM CCX DMA*/
                HAL_PWM_Stop_DMA(hpwm, channel);
            HAL_NVIC_DisableIRQ(pwm_dma->dma_irq);
            HAL_DMA_DeInit(&(pwm_dma->dma_handle));
#endif
        }
        else
        {
            HAL_PWM_Stop(hpwm, channel);
        }
    }
    else
    {
        PWM_OC_InitTypeDef oc_config = {0};
        oc_config.OCMode = PWM_OCMODE_PWM1;
        oc_config.Pulse = __HAL_PWM_GET_COMPARE(hpwm, channel);
        oc_config.OCPolarity = PWM_OCPOLARITY_HIGH;
        if (HAL_PWM_ConfigChannel(hpwm, &oc_config, channel) != HAL_OK)
        {
            LOG_E("%x channel %d config failed", hpwm, configuration->channel);

            return RT_ERROR;
        }
        if (pwm_dma)
        {
//TODO:fixme DMA start function
            RT_ASSERT(0);
#if 0
            if (!pwm_dma->flag_dma_eanbled)
            {
                HAL_DMA_Init(&(pwm_dma->dma_handle));
                __HAL_LINKDMA(hpwm, hdma[pwm_dma->dma_handle_index], pwm_dma->dma_handle);
                HAL_NVIC_SetPriority(pwm_dma->dma_irq, pwm_dma->dma_handle.Init.Priority, 0);
                HAL_NVIC_EnableIRQ(pwm_dma->dma_irq);
                pwm_dma->flag_dma_eanbled = 1;
            }
            if (configuration->dma_data && configuration->data_len)
            {
                if (pwm_dma->dma_handle_index == PWM_DMA_ID_UPDATE) /*Start TIM Update DMA*/
                    HAL_PWM_Update_Start_DMA(hpwm, channel, (uint32_t *)configuration->dma_data, configuration->data_len);
                else                                                /*Start TIM CCX DMA*/
                    HAL_PWM_art_DMA(hpwm, channel, (uint32_t *)configuration->dma_data, configuration->data_len);
            }
#endif
        }
        else
        {
            HAL_PWM_Start(hpwm, channel);
        }
    }

    return RT_EOK;
}

/**
* @brief  Get a PWM device configuration.
* @param[in]  hpwm: GPT device object handle.
* @param[in,out]  configuration: GPT configuration, input mainly GPT channel number. return period and pulse.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_get(struct bf0_pwm *pwm, struct rt_pwm_configuration *configuration)
{
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = (configuration->channel - 1);
    rt_uint64_t PWM_clock;
    rt_uint32_t psc;
    PWM_HandleTypeDef *hpwm = &(pwm->pwm_handle);
    /* fixed 48MHz */
    PWM_clock = PWM_CLK_FREQ;

    RT_ASSERT(channel < 4);

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    PWM_clock /= 1000000UL;
    psc = __HAL_PWM_GET_PRESCALER(hpwm, channel);
    configuration->period = (__HAL_PWM_GET_AUTORELOAD(hpwm, channel) + 1) * (psc + 1) * 1000UL / PWM_clock;
    configuration->pulse = (__HAL_PWM_GET_COMPARE(hpwm, channel) + 1) * (psc + 1) * 1000UL / PWM_clock;

    return RT_EOK;
}

/**
* @brief  Set a PWM device configuration.
* @param[in]  hpwm: GPT device object handle.
* @param[in]  configuration: GPT configuration, input mainly GPT channel number, period and pulse.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
// Define a global variable to store the calculated pulse value
//TODO:
// unsigned long long global_pulse_values[10] = {0};
// size_t global_array_length = 0;

static rt_err_t drv_pwm_set(struct bf0_pwm *pwm, struct rt_pwm_configuration *configuration)
{
    rt_uint32_t period, pulse;
    rt_uint32_t PWM_clock, psc;
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = (configuration->channel - 1);
    rt_uint32_t max_period = MAX_PERIOD_GPT;
    PWM_HandleTypeDef *hpwm = &(pwm->pwm_handle);

    RT_ASSERT(channel < 4);

    /* fixed 48MHz */
    PWM_clock = PWM_CLK_FREQ;

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    PWM_clock /= 1000000UL;//In Mhz units
    // LOG_I("PWM_clock:%d\n",PWM_clock);
    period = (unsigned long long)configuration->period * PWM_clock / 1000ULL ;
    psc = period / max_period + 1;
    period = period / psc;
    __HAL_PWM_SET_PRESCALER(hpwm, channel, psc - 1);

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    // LOG_I("period:%d\n",period);
    __HAL_PWM_SET_AUTORELOAD(hpwm, channel, period - 1);
    pulse = (unsigned long long)configuration->pulse * PWM_clock / psc / 1000ULL;

    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse >= period)       /*if pulse reach to 100%, need set pulse = period + 1, because pulse = period, the real percentage = 99.9983%  */
    {
        pulse = period + 1;
    }

    __HAL_PWM_SET_COMPARE(hpwm, channel, pulse - 1);

    //pulse compute conversion
    if (configuration->use_percentage)//If you need to perform ratio calculation on pulse
    {
        RT_ASSERT(0);
//TODO:
#if 0
        for (size_t i = 0; i < configuration->data_len; i++)
        {
            unsigned long long pulse_a = ((unsigned long long)configuration->pulse_dma_data[i] * PWM_clock / psc / 1000ULL) - 1;
            if (pulse_a < MIN_PULSE)
            {
                pulse_a = MIN_PULSE;
            }
            else if (pulse_a > period)
            {
                pulse_a = period;
            }

            global_pulse_values[i] = pulse_a;
        }
        global_array_length = configuration->data_len;
#endif
    }
    //__HAL_PWM_SET_COUNTER(hpwm, 0);

    /* Update frequency value */
    HAL_PWM_GenerateEvent(hpwm, PWM_EVENTSOURCE_UPDATE1 << channel);

    return RT_EOK;
}

/**
* @brief  Set a PWM device configuration.
* @param[in]  hpwm: GPT device object handle.
* @param[in]  configuration: GPT configuration, input mainly GPT channel number and period.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t drv_pwm_set_period(struct bf0_pwm *pwm, struct rt_pwm_configuration *configuration)
{
    rt_uint32_t period;
    rt_uint32_t PWM_clock, psc;
    rt_uint32_t max_period = MAX_PERIOD_GPT;
    PWM_HandleTypeDef *hpwm = &(pwm->pwm_handle);
    rt_uint32_t channel = (configuration->channel - 1);

    RT_ASSERT(channel < 4);

    PWM_clock = PWM_CLK_FREQ;

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    PWM_clock /= 1000000UL;
    period = (unsigned long long)configuration->period * PWM_clock / 1000ULL ;
    psc = period / max_period + 1;
    period = period / psc;

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    __HAL_PWM_SET_AUTORELOAD(hpwm, channel, period - 1);

    /* Update frequency value */
    HAL_PWM_GenerateEvent(hpwm, PWM_EVENTSOURCE_UPDATE1 << channel);

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
    //PWM_HandleTypeDef *hpwm = (PWM_HandleTypeDef *)device->parent.user_data;
    struct bf0_pwm *pwm = (struct bf0_pwm *)device->parent.user_data;

    if ((RT_DEVICE_CTRL_RESUME != cmd) && (RT_DEVICE_CTRL_SUSPEND != cmd))
    {
        /* arg is not configuration for RESUME and SUSPEND command */
        RT_ASSERT(configuration->channel > 0); //Channel id must > 0
    }
    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        return drv_pwm_enable(pwm, configuration, RT_TRUE);
    case PWM_CMD_DISABLE:
        return drv_pwm_enable(pwm, configuration, RT_FALSE);
    case PWM_CMD_SET:
        return drv_pwm_set(pwm, configuration);
    case PWM_CMD_GET:
        return drv_pwm_get(pwm, configuration);
    case PWM_CMD_SET_PERIOD:
        return drv_pwm_set_period(pwm, configuration);
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
    PWM_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(device != RT_NULL);

    tim = (PWM_HandleTypeDef *)&device->pwm_handle;

    if (HAL_PWM_Init(tim) != HAL_OK)
    {
        LOG_E("%s init failed", device->name);
        result = -RT_ERROR;
    }

    return result;
}
static void bf0_hw_pwm_config_dma(struct bf0_pwm *device)
{
    uint16_t i;
    for (i = 0; i < 4; i++)
    {
        if (device->pwm_cc_dma[i])
        {
            device->pwm_cc_dma[i]->dma_handle.Init.Direction          = DMA_MEMORY_TO_PERIPH;
            device->pwm_cc_dma[i]->dma_handle.Init.PeriphInc          = DMA_PINC_DISABLE;
            device->pwm_cc_dma[i]->dma_handle.Init.MemInc             = DMA_MINC_ENABLE;
            device->pwm_cc_dma[i]->dma_handle.Init.Mode               = DMA_NORMAL;//DMA_CIRCULAR;     /*DMA use circular mode*/
            device->pwm_cc_dma[i]->dma_handle.Init.Priority           = DMA_PRIORITY_LOW;
        }
    }
    if (device->pwm_update_dma)
    {
        device->pwm_update_dma->dma_handle.Init.Direction          = DMA_MEMORY_TO_PERIPH;
        device->pwm_update_dma->dma_handle.Init.PeriphInc          = DMA_PINC_DISABLE;
        device->pwm_update_dma->dma_handle.Init.MemInc             = DMA_MINC_ENABLE;
        device->pwm_update_dma->dma_handle.Init.Mode               = DMA_NORMAL;//DMA_CIRCULAR;     /*DMA use circular mode*/
        device->pwm_update_dma->dma_handle.Init.Priority           = DMA_PRIORITY_LOW;
    }
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

    pwm_get_dma_info();

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
            bf0_hw_pwm_config_dma(&bf0_pwm_obj[i]);

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
INIT_DEVICE_EXPORT(bf0_pwm_init);

#endif /* RT_USING_PWM */

