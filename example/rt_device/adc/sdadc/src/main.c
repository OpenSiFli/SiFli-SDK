/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "board.h"

#define DBG_TAG "sdadc"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define SDADC_DEV_NAME        "sdadc"

/* SDADC channel <-> pin (driver channel numbering 1..4):
 *   - SF32LB55X: CH1=PB_23, CH2=PB_24, CH3=PB_25, CH4=PB_26
 *   - SF32LB58X: CH1=PB_40, CH2=PB_41, CH3=PB_42, CH4=PB_43 */
#if defined(SF32LB58X)
    #define SDADC_CH1_PAD       PAD_PB40
    #define SDADC_CH1_GPIO      GPIO_B40
    #define SDADC_CH2_PAD       PAD_PB41
    #define SDADC_CH2_GPIO      GPIO_B41
    #define SDADC_CH3_PAD       PAD_PB42
    #define SDADC_CH3_GPIO      GPIO_B42
    #define SDADC_CH4_PAD       PAD_PB43
    #define SDADC_CH4_GPIO      GPIO_B43
#elif defined(SF32LB55X)
    #define SDADC_CH1_PAD       PAD_PB23
    #define SDADC_CH1_GPIO      GPIO_B23
    #define SDADC_CH2_PAD       PAD_PB24
    #define SDADC_CH2_GPIO      GPIO_B24
    #define SDADC_CH3_PAD       PAD_PB25
    #define SDADC_CH3_GPIO      GPIO_B25
    #define SDADC_CH4_PAD       PAD_PB26
    #define SDADC_CH4_GPIO      GPIO_B26
#else
    #error "SDADC is only supported on SF32LB55X and SF32LB58X"
#endif

static const rt_uint32_t g_sdadc_channels[] = {1, 2, 3, 4};
#define SDADC_CH_COUNT   (sizeof(g_sdadc_channels) / sizeof(g_sdadc_channels[0]))

static rt_device_t s_sd_adc_dev;

/* Released from the SDADC conversion-complete interrupt (via rx_indicate). */
static struct rt_semaphore g_sdadc_rx_sem;

static rt_err_t sdadc_pin_config(rt_uint32_t channel)
{
    uint32_t pad, gpio;

    switch (channel)
    {
    case 1:
        pad = SDADC_CH1_PAD;
        gpio = SDADC_CH1_GPIO;
        break;
    case 2:
        pad = SDADC_CH2_PAD;
        gpio = SDADC_CH2_GPIO;
        break;
    case 3:
        pad = SDADC_CH3_PAD;
        gpio = SDADC_CH3_GPIO;
        break;
    case 4:
        pad = SDADC_CH4_PAD;
        gpio = SDADC_CH4_GPIO;
        break;
    default:
        LOG_E("unsupported SDADC channel %d", channel);
        return -RT_ERROR;
    }

    HAL_PIN_Set(pad, gpio, PIN_NOPULL, 0);
    HAL_PIN_Select(pad, 10, 0);
    return RT_EOK;
}

static rt_err_t sdadc_rx_indicate(rt_device_t dev, rt_size_t size)
{
    (void)dev;
    (void)size;
    rt_sem_release(&g_sdadc_rx_sem);
    return RT_EOK;
}

static rt_err_t adc_example_init(void)
{
    rt_err_t r;
    rt_uint32_t i;

    /* Find SDADC RT device */
    s_sd_adc_dev = rt_device_find(SDADC_DEV_NAME);
    if (s_sd_adc_dev == RT_NULL)
    {
        LOG_E("find %s failed", SDADC_DEV_NAME);
        return -RT_ERROR;
    }

    r = rt_adc_init((rt_adc_device_t)s_sd_adc_dev);
    if (r != RT_EOK)
    {
        LOG_E("rt_adc_init %s failed: %d", SDADC_DEV_NAME, r);
        return r;
    }

    /* Configure pinmux and enable each channel (multi-channel: enables the matching slot). */
    for (i = 0; i < SDADC_CH_COUNT; i++)
    {
        rt_uint32_t ch = g_sdadc_channels[i];

        r = sdadc_pin_config(ch);
        if (r != RT_EOK)
            return r;

        r = rt_adc_enable((rt_adc_device_t)s_sd_adc_dev, ch);
        if (r != RT_EOK)
        {
            LOG_E("rt_adc_enable ch%d failed: %d", ch, r);
            return r;
        }
        LOG_I("SDADC ch%d enabled", ch);
    }

    rt_sem_init(&g_sdadc_rx_sem, "sdadc_rx", 0, RT_IPC_FLAG_FIFO);
    rt_device_set_rx_indicate(s_sd_adc_dev, sdadc_rx_indicate);

    LOG_I("SDADC multi-channel + timer trigger + interrupt, %d channels", SDADC_CH_COUNT);
    return RT_EOK;
}

int main(void)
{
    rt_uint32_t i;

    rt_kprintf("\n======== SDADC Multi-Channel (IRQ-driven) Example ========\n");

    if (adc_example_init() != RT_EOK)
        return -RT_ERROR;

    while (1)
    {
        /* Wait for the timer-triggered conversion-complete interrupt instead of polling. */
        if (rt_sem_take(&g_sdadc_rx_sem, rt_tick_from_millisecond(2000)) != RT_EOK)
        {
            LOG_W("SDADC conversion-complete notification timeout");
            continue;
        }

        for (i = 0; i < SDADC_CH_COUNT; i++)
        {
            rt_uint32_t ch = g_sdadc_channels[i];
            /* Non-blocking: the latest per-slot value is kept fresh by the background scan. */
            rt_uint32_t value = rt_adc_read((rt_adc_device_t)s_sd_adc_dev, ch);
            LOG_I("ch%d: %d (%.1f mV)", ch, value, value / 10.0f);
        }
        rt_kprintf("----\n");
    }

    return 0;
}
