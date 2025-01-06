/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-07     aozima       the first version
 * 2018-11-16     Ernest Chen  add finsh command and update adc function
 */

#ifndef __ADC_H__
#define __ADC_H__
#include <rtthread.h>

struct rt_adc_device;
struct rt_adc_ops
{
    rt_err_t (*enabled)(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled);
    rt_err_t (*convert)(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value);
    rt_err_t (*init)(struct rt_adc_device *device);
    rt_err_t (*control)(struct rt_adc_device *device, rt_uint32_t cmd, void *arg);
};

/** Invalid ADC channel */
#define ADC_INVALID_CHANNEL (0xFF)

struct rt_adc_device
{
    struct rt_device parent;
    const struct rt_adc_ops *ops;
    /** used ADC channel, #ADC_INVALID_CHANNEL: invalid channel */
    rt_uint32_t channel;
};
typedef struct rt_adc_device *rt_adc_device_t;

typedef enum
{
    RT_ADC_CMD_ENABLE = 30 + 1,
    RT_ADC_CMD_DISABLE,
    RT_ADC_CMD_READ,
} rt_adc_cmd_t;


typedef struct
{
    /** ADC channel, starting from 0 */
    rt_uint32_t channel;
    /** ADC value in 0.1mV, i.e. 1 equals to 0.1mV, 10 equals to 1mV */
    rt_uint32_t value;
} rt_adc_cmd_read_arg_t;

rt_err_t rt_hw_adc_register(rt_adc_device_t adc, const char *name, const struct rt_adc_ops *ops, const void *user_data);

rt_uint32_t rt_adc_read(rt_adc_device_t dev, rt_uint32_t channel);
rt_err_t rt_adc_enable(rt_adc_device_t dev, rt_uint32_t channel);
rt_err_t rt_adc_disable(rt_adc_device_t dev, rt_uint32_t channel);
rt_err_t rt_adc_init(rt_adc_device_t dev);
#endif /* __ADC_H__ */
