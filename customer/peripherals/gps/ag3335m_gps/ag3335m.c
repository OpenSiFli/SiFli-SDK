

#include <stdio.h>
#include <string.h>
#include "rtdevice.h"
#include "drv_gpio.h"
#include "ag3335m.h"
#include "ag3335m_control.h"
#include "button.h"
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif

#if defined(PMIC_CONTROL_SERVICE)
    #include "pmic_service.h"
#endif


#define DBG_TAG               "ag3335m"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>
#if 0
    #define GPS_WAKEUP_MCU_PIN (77)
    #define GPS_RTC_EINT_PIN (27+96)
    #define MCU_WAKEUP_GPS_PIN (23+96) //PB23
    #define GPS_RESET_PIN (15+96)
#endif

static const struct rt_gps_ops ag3335m_gps_ops =
{
    .control = ag3335m_control
};

static void ag3335m_wakeup_event_handler(void *args)
{

}


static void ag3335m_wakeup_mcu_pin_init(void)
{
#ifdef BSP_USING_PM
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;

    gpio = GET_GPIO_INSTANCE(GPS_WAKEUP_MCU_PIN);
    gpio_pin = GET_GPIOx_PIN(GPS_WAKEUP_MCU_PIN);
#if (GPS_WAKEUP_MCU_PIN < GPIO1_PIN_NUM)
    wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);
#else
    wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
#endif
    RT_ASSERT(wakeup_pin >= 0);

    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_LOW);
#endif /* BSP_USING_PM */
    rt_pin_mode(GPS_WAKEUP_MCU_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(GPS_WAKEUP_MCU_PIN, PIN_IRQ_MODE_LOW_LEVEL, (void *) ag3335m_wakeup_event_handler,
                      (void *)(rt_uint32_t) GPS_WAKEUP_MCU_PIN);
    rt_pin_irq_enable(GPS_WAKEUP_MCU_PIN, 1);
    return;
}


void ag3335m_pin_init()
{
#ifdef PMIC_CONTROL_SERVICE
    pmic_service_control(PMIC_CONTROL_GPS, 1);
#endif

#if (-1 != GPS_POWER_PIN)
    rt_pin_mode(GPS_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(GPS_POWER_PIN, PIN_HIGH);
#endif

#if (-1 != MCU_RESET_GPS_PIN)
    rt_pin_mode(MCU_RESET_GPS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_RESET_GPS_PIN, PIN_HIGH);
#endif

    //rt_pin_mode(GPS_RTC_EINT_PIN, PIN_MODE_OUTPUT);
    //rt_pin_write(GPS_RTC_EINT_PIN, PIN_HIGH);

#if (-1 != MCU_WAKEUP_GPS_PIN)
    rt_pin_mode(MCU_WAKEUP_GPS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_WAKEUP_GPS_PIN, PIN_HIGH);
#endif

#if (-1 != GPS_WAKEUP_MCU_PIN)
    ag3335m_wakeup_mcu_pin_init();
#endif
    return;
}

int ag3335m_at_client_init()
{
    rt_err_t ret = RT_EOK;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    ret = at_client_init(GPS_UART_NAME, GPS_RECV_BUFF_LEN);
    LOG_I("%s:at_init ret:%d", __func__, ret);
    at_client_t at_client = at_client_get(GPS_UART_NAME);
    RT_ASSERT(at_client != RT_NULL);
    config.baud_rate = GPS_UART_BAUD;
    rt_device_control(at_client->device, RT_DEVICE_CTRL_CONFIG, &config);
    ag3335m_set_cmd_table(at_client);
    /* register URC data setupution function  */
    ag3335m_set_urc(at_client);
    return ret;
}

int ag3335m_init(void)
{
    /* initialize AT client */
    rt_err_t ret = RT_EOK;
    ag3335m_pin_init();
    rt_hw_gps_init(&ag3335m_gps_ops);
    ag3335m_at_client_init();
    return GPS_EOK;
}
INIT_COMPONENT_EXPORT(ag3335m_init);

