#ifndef __MOTOR_DEVICE_H__
#define __MOTOR_DEVICE_H__
#include <rtthread.h>

#define MOTOR_DEVICE_FLAG_OPEN (0x80)

typedef enum
{
    MOTOR_CONTROL_CLOSE_DEVICE = 0x20 + 1,     /**< close motor device */
    MOTOR_CONTROL_OPEN_DEVICE,                 /**< open motor device */
    MOTOR_CONTROL_CTRL_IO,
    MOTOR_CONTROL_PWM_OPEN,
    MOTOR_CONTROL_PWM_CLOSE,

    MOTOR_CONTROL_CMD_MAX
} motor_control_cmd_t;

typedef enum
{
    MOTOR_STATE_POWER_OFF = 0,
    MOTOR_STATE_POWER_ON,
} motor_state_t;

typedef enum
{
    MOTOR_EOK = 0,
    MOTOR_ERROR,
} motor_err_t;

typedef struct
{
    motor_state_t last_state;
    motor_state_t cur_state;
} motor_status_t;

typedef struct rt_motor_device
{
    struct rt_device   parent;
    //rt_mutex_t handle_lock;
    motor_status_t status;
    uint8_t ctrl_status;
    const struct rt_motor_ops *ops;
} rt_motor_t;

typedef motor_err_t (*motor_control_cb)(struct rt_motor_device *dev_handle, int cmd, void *arg);

struct rt_motor_ops
{
    motor_control_cb control;
};

rt_err_t rt_motor_register(struct rt_motor_device *dev_handle, const char *name);
#endif
