

#include "rtthread.h"
#include "rtdevice.h"
#include "charge.h"
//#include "drv_gpio.h"
#include "board.h"
#include "drv_gpio.h"
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif

static rt_charge_device_t charge_device;
rt_err_t sgm40561_control(rt_charge_device_t *charge, int cmd, void *args)
{
    rt_charge_err_t ret = RT_CHARGE_EOK;

    switch (cmd)
    {
    case RT_CHARGE_GET_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        //RT_ASSERT(BSP_CHARGING_PIN >= 0);
        if (BSP_CHARGING_PIN >= 0)
        {
#ifdef BSP_CHARGING_PIN_ACTIVE_HIGH
            *status = rt_pin_read(BSP_CHARGING_PIN);
#else
            *status = !rt_pin_read(BSP_CHARGING_PIN);
#endif
        }
        else
            ret = RT_CHARGE_ERROR_UNSUPPORTED;
    }
    break;

    case RT_CHARGE_GET_DETECT_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        //RT_ASSERT(BSP_CHARGER_INT_PIN >= 0);
        if (BSP_CHARGER_INT_PIN >= 0)
        {
#ifdef BSP_CHARGING_PIN_ACTIVE_HIGH
            *status = rt_pin_read(BSP_CHARGER_INT_PIN);
#else
            *status = !rt_pin_read(BSP_CHARGER_INT_PIN);
#endif
        }
        else
            ret = RT_CHARGE_ERROR_UNSUPPORTED;
    }
    break;

    case RT_CHARGE_GET_FULL_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        //RT_ASSERT(BSP_CHARGE_FULL_PIN >= 0);
        if (BSP_CHARGE_FULL_PIN >= 0)
        {
#ifdef BSP_CHARGE_FULL_PIN_ACTIVE_HIGH
            *status = rt_pin_read(BSP_CHARGE_FULL_PIN);
#else
            *status = !rt_pin_read(BSP_CHARGE_FULL_PIN);
#endif
        }
        else
            ret = RT_CHARGE_ERROR_UNSUPPORTED;

    }
    break;
#if 0
    case RT_CHARGE_ENABLE:
    {
        uint8_t *enable = (uint8_t *)args;
        RT_ASSERT(BSP_CHARGER_EN_PIN >= 0);
        rt_pin_write(BSP_CHARGER_EN_PIN, *enable);
    }
    break;
#endif
    default:
        ret = RT_CHARGE_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

static const struct rt_charge_ops sgm40561_ops =
{
    .control = sgm40561_control
};


void sgm40561_input_handle(void *args)
{
    static uint32_t last_time;

    uint32_t cur_time = rt_tick_get();
    uint32_t tick = (cur_time - last_time + UINT32_MAX + 1) & UINT32_MAX;

    if (tick > 150)
    {
        rt_charge_event_notify(RT_CHARGE_EVENT_DETECT);
    }

    last_time = cur_time;
    return;

}
void sgm40561_pin_init(void)
{
#if(BSP_CHARGING_PIN != -1) && defined(BSP_CHARGING_PIN)
    rt_pin_mode(BSP_CHARGING_PIN, PIN_MODE_INPUT);
#endif

#if(BSP_CHARGE_FULL_PIN != -1) && defined(BSP_CHARGE_FULL_PIN)
    rt_pin_mode(BSP_CHARGE_FULL_PIN, PIN_MODE_INPUT);
#endif

#if(BSP_CHARGER_EN_PIN != -1) && defined(BSP_CHARGER_EN_PIN)
    rt_pin_mode(BSP_CHARGER_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BSP_CHARGER_EN_PIN, 1);    //disbale  charge
#endif

#if defined(BSP_USING_CHARGER_DETECT)

#ifdef BSP_USING_PM
    GPIO_TypeDef *gpio = GET_GPIO_INSTANCE(BSP_CHARGER_INT_PIN);
    uint16_t gpio_pin = GET_GPIOx_PIN(BSP_CHARGER_INT_PIN);
    int8_t wakeup_pin;
    if (BSP_CHARGER_INT_PIN > 96)
        wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    else
        wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);

    RT_ASSERT(wakeup_pin >= 0);
    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_DOUBLE_EDGE);
#endif/* BSP_USING_PM */

    rt_pin_mode(BSP_CHARGER_INT_PIN, PIN_MODE_INPUT);

#if !defined (DFU_OTA_MANAGER)  //avoid ota triggers charging interruption to clear WSR before entering app
    // enable LSM int
    rt_pin_attach_irq(BSP_CHARGER_INT_PIN, PIN_IRQ_MODE_RISING_FALLING, (void *) sgm40561_input_handle,
                      (void *)(rt_uint32_t) BSP_CHARGER_INT_PIN);
    rt_pin_irq_enable(BSP_CHARGER_INT_PIN, 1);
#endif

#endif

}

int sgm40561_init(void)
{
    sgm40561_pin_init();
    rt_charge_register(&charge_device, &sgm40561_ops, RT_NULL);
    return RT_EOK;
}

INIT_PREV_EXPORT(sgm40561_init);


