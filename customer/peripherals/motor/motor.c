

#include <stdio.h>
#include <string.h>
#include <rtdevice.h>
#include "drv_motor.h"
#include "motor.h"
#if defined(PMIC_CONTROL_SERVICE)
    #include "pmic_service.h"
#endif


#define DBG_TAG               "motor_comm"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

motor_err_t motor_open(void)
{
    rt_err_t ret = RT_EOK;
#ifdef PMIC_CONTROL_SERVICE
    pmic_service_control(PMIC_CONTROL_MOTOR, 1);
#else
#if (MOTOR_POWER_IO >= 0)
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) return ret;
    m.pin = MOTOR_POWER_IO;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = MOTOR_POWER_IO;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);

    if (ret != MOTOR_EOK)
        ret = MOTOR_ERROR;
#endif
#endif
    return ret;
}

motor_err_t motor_close(void)
{
    rt_err_t ret = RT_EOK;
#ifdef PMIC_CONTROL_SERVICE
    pmic_service_control(PMIC_CONTROL_MOTOR, 0);
#else
#if (MOTOR_POWER_IO >= 0)
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) return ret;
    m.pin = MOTOR_POWER_IO;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = MOTOR_POWER_IO;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);

    if (ret != MOTOR_EOK)
        ret = MOTOR_ERROR;
#endif
#endif
    return ret;
}

motor_err_t motor_ctrl(uint8_t status)
{
    rt_err_t ret = RT_EOK;
#ifdef MOTOR_SW_CONTRL
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) return ret;
    m.pin = MOTOR_CTRL_IO;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);
    if (status == 1)
    {
        st.pin = MOTOR_CTRL_IO;
        st.status = 0;
        rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    }
    else
    {
        st.pin = MOTOR_CTRL_IO;
        st.status = 1;
        rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    }
    ret = rt_device_close(device);

    if (ret != MOTOR_EOK)
        ret = MOTOR_ERROR;
#endif
    return ret;
}

motor_err_t motor_pwm_open(void *args)
{
    rt_err_t ret = RT_EOK;
#if defined (BSP_USING_PWM) && defined (MOTOR_USE_PWM)
    struct rt_device_pwm *motor_device = (struct rt_device_pwm *)rt_device_find(MOTOR_INTERFACE_NAME);
    if (!motor_device)
    {
        RT_ASSERT(0);
    }
    else
    {
        rt_uint32_t period = MOTOR_PERIOD * 1000 * 1000;
        //rt_kprintf("motor_pwm_open cyc %d %d %d\n",*(uint32_t *)args, period, (period / 100) * (*(uint32_t *)args));
        rt_pwm_set(motor_device, MOTOR_CHANEL_NUM, period, (period / 100) * (*(uint32_t *)args));
        ret = rt_pwm_enable(motor_device, MOTOR_CHANEL_NUM);
    }
#endif
    return ret;
}

motor_err_t motor_pwm_close(void *args)
{
    rt_err_t ret = RT_EOK;
#if defined (BSP_USING_PWM) && defined (MOTOR_USE_PWM)
    struct rt_device_pwm *motor_device = (struct rt_device_pwm *)rt_device_find(MOTOR_INTERFACE_NAME);
    if (!motor_device)
    {
        RT_ASSERT(0);
    }
    else
    {
        ret = rt_pwm_disable(motor_device, MOTOR_CHANEL_NUM);
        //rt_kprintf("motor_pwm_close ret %d\n",ret);
    }
#endif
    return ret;
}

motor_err_t motor_control(struct rt_motor_device *bt_handle, int cmd, void *args)
{
    motor_err_t ret = MOTOR_EOK;

    switch (cmd)
    {
    case MOTOR_CONTROL_CLOSE_DEVICE:
    {
        ret = motor_close();
    }
    break;

    case MOTOR_CONTROL_OPEN_DEVICE:
    {
        ret = motor_open();
    }
    break;

    case MOTOR_CONTROL_CTRL_IO:
    {
        //rt_kprintf("motor_control ctrl %d\n",*(uint8_t *)args);
        ret = motor_ctrl(*(uint8_t *)args);

    }
    break;

    case MOTOR_CONTROL_PWM_OPEN:
    {
        ret = motor_pwm_open(args);
    }
    break;

    case MOTOR_CONTROL_PWM_CLOSE:
    {
        ret = motor_pwm_close(args);
    }
    break;

    default:
        break;
    }
    return ret;
}

void motor_pin_init()
{

    return;
}

static const struct rt_motor_ops motor_comm_ops =
{
    .control = motor_control
};

int motor_init(void)
{
    rt_err_t ret = RT_EOK;
    rt_hw_motor_init(&motor_comm_ops);

    //motor_pin_init();
    return MOTOR_EOK;
}

INIT_COMPONENT_EXPORT(motor_init);


