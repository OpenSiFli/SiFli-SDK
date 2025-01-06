

#include <stdio.h>
#include <string.h>
#include "drv_motor.h"
#define DBG_TAG    "drv_motor"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>


#ifdef RT_USING_MOTOR
struct rt_motor_device mt_obj = {0};


int rt_hw_motor_init(const struct rt_motor_ops *ops)
{
    rt_err_t result = 0;
    mt_obj.ops = ops;


    mt_obj.status.cur_state = MOTOR_STATE_POWER_OFF;
    mt_obj.status.last_state = MOTOR_STATE_POWER_OFF;
    result = rt_motor_register(&mt_obj, MOTOR_DEVICE_NAME);
    RT_ASSERT(result == RT_EOK);
    return result;
}

#endif /* RT_USING_BT */

