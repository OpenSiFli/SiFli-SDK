

#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
#include "bt_fr508x.h"
#include "bt_fr508x_urc.h"
#include "bt_fr508x_control.h"
#if defined(PMIC_CONTROL_SERVICE)
    #include "pmic_service.h"
#endif


#define DBG_TAG               "fr508x"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

static const struct rt_bt_ops fr508x_bt_ops =
{
    .control = fr508x_control
};

#if (-1 != BT_WAKEUP_MCU_PIN)
#include "bf0_pm.h"
#include "drv_gpio.h"


void fr508x_wakeup_event_handler(void)
{
}

static void fr508x_wakeup_mcu_pin_init(void)
{
#ifdef BSP_USING_PM
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;

    gpio = GET_GPIO_INSTANCE(BT_WAKEUP_MCU_PIN);
    gpio_pin = GET_GPIOx_PIN(BT_WAKEUP_MCU_PIN);
#if (BT_WAKEUP_MCU_PIN < GPIO1_PIN_NUM)
    wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);
#else
    wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
#endif
    RT_ASSERT(wakeup_pin >= 0);

    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_LOW);
#endif /* BSP_USING_PM */
    rt_pin_mode(BT_WAKEUP_MCU_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(BT_WAKEUP_MCU_PIN, PIN_IRQ_MODE_LOW_LEVEL, (void *) fr508x_wakeup_event_handler,
                      (void *)(rt_uint32_t) BT_WAKEUP_MCU_PIN);
    rt_pin_irq_enable(BT_WAKEUP_MCU_PIN, 1);
    return;
}


static void fr508x_wakeup_mcu_pin_deinit(void)
{
#ifdef BSP_USING_PM
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;

    gpio = GET_GPIO_INSTANCE(BT_WAKEUP_MCU_PIN);
    gpio_pin = GET_GPIOx_PIN(BT_WAKEUP_MCU_PIN);
#if (BT_WAKEUP_MCU_PIN < GPIO1_PIN_NUM)
    wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);
#else
    wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
#endif
    RT_ASSERT(wakeup_pin >= 0);

    pm_disable_pin_wakeup(wakeup_pin);
#endif /* BSP_USING_PM */
    rt_pin_irq_enable(BT_WAKEUP_MCU_PIN, 0);
    rt_pin_detach_irq(BT_WAKEUP_MCU_PIN);
    return;
}

#endif

void fr508x_pin_init(void)
{
#if (-1 != BT_POWER_PIN)
#ifdef PMIC_CONTROL_SERVICE
    pmic_service_control(PMIC_CONTROL_BT, 1);
#else
    rt_pin_mode(BT_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BT_POWER_PIN, PIN_HIGH);
#endif
#endif

#if (-1 != MCU_WAKEUP_BT_PIN)
    rt_pin_mode(MCU_WAKEUP_BT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_WAKEUP_BT_PIN, PIN_HIGH);
#endif

#if (-1 != BT_WAKEUP_MCU_PIN)
    fr508x_wakeup_mcu_pin_init();
#endif
    return;
}


void fr508x_pin_deinit(void)
{
#if (-1 != BT_WAKEUP_MCU_PIN)
    fr508x_wakeup_mcu_pin_deinit();
#endif
    return;
}


int fr508x_at_client_init()
{
    rt_err_t ret = RT_EOK;
    ret = at_client_init(BT_UART_NAME, BT_RECV_BUFF_LEN);
    LOG_I("%s:at_init ret:%d", __func__, ret);
    at_client_t at_client = at_client_get(BT_UART_NAME);
    RT_ASSERT(at_client != RT_NULL);
    fr508x_set_cmd_table(at_client);
    /* register URC data setupution function  */
    fr508x_set_urc(at_client);
    return ret;
}

int fr508x_init(void)
{
    /* initialize AT client */
    rt_err_t ret = RT_EOK;
    rt_hw_bt_init(&fr508x_bt_ops, 0);
    fr508x_at_client_init();
    fr508x_pin_init();
    return BT_EOK;
}

INIT_COMPONENT_EXPORT(fr508x_init);


