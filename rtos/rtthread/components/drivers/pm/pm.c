/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-06-02     Bernard      the first version
 * 2018-08-02     Tanek        split run and sleep modes, support custom mode
 * 2019-04-28     Zero-Free    improve PM mode and device ops interface
 */

#include <rthw.h>
#include <rtthread.h>
#include <drivers/pm.h>

#ifdef RT_USING_PM

#ifdef PM_REQUEST_DEBUG
typedef struct
{
    const char *file;
    uint32_t line_num;
} pm_request_hist_item;

#define PM_REQUEST_HIST_LEN  (32)

typedef struct
{
    uint32_t idx;
    pm_request_hist_item items[PM_REQUEST_HIST_LEN];
} pm_request_hist_t;

pm_request_hist_t _pm_request_hist;

#endif /* PM_REQUEST_DEBUG */

const static pm_policy_t default_pm_policy[] =
{
#ifdef PM_STANDBY_ENABLE
    {10000, PM_SLEEP_MODE_STANDBY},
#else
    {15, PM_SLEEP_MODE_LIGHT},
//    {25, PM_SLEEP_MODE_DEEP},
#endif /* PM_STANDBY_ENABLE */
};

__ROM_USED struct rt_pm _pm;
__ROM_USED uint8_t _pm_default_sleep = RT_PM_DEFAULT_SLEEP_MODE;
__ROM_USED struct rt_pm_notify _pm_notify;
__ROM_USED uint8_t _pm_init_flag = 0;

#define RT_PM_TICKLESS_THRESH (2)

RT_WEAK uint32_t rt_pm_enter_critical(uint8_t sleep_mode)
{
    return rt_hw_interrupt_disable();
}

RT_WEAK void rt_pm_exit_critical(uint32_t ctx, uint8_t sleep_mode)
{
    rt_hw_interrupt_enable(ctx);
}

/**
 * This function will suspend all registered devices
 */
static int _pm_device_suspend(uint8_t mode)
{
    int index, ret = RT_EOK;

    for (index = 0; index < _pm.device_pm_number; index++)
    {
        if (_pm.device_pm[index].ops->suspend != RT_NULL)
        {
            ret = _pm.device_pm[index].ops->suspend(_pm.device_pm[index].device, mode);
            if (ret != RT_EOK)
                break;
        }
    }

    return ret;
}

/**
 * This function will resume all registered devices
 */
static void _pm_device_resume(uint8_t mode)
{
    int index;

    for (index = 0; index < _pm.device_pm_number; index++)
    {
        if (_pm.device_pm[index].ops->resume != RT_NULL)
        {
            _pm.device_pm[index].ops->resume(_pm.device_pm[index].device, mode);
        }
    }
}

/**
 * This function will update the frequency of all registered devices
 */
static void _pm_device_frequency_change(uint8_t mode)
{
    rt_uint32_t index;

    /* make the frequency change */
    for (index = 0; index < _pm.device_pm_number; index ++)
    {
        if (_pm.device_pm[index].ops->frequency_change != RT_NULL)
            _pm.device_pm[index].ops->frequency_change(_pm.device_pm[index].device, mode);
    }
}

/**
 * This function will update the system clock frequency when idle
 */
static void _pm_frequency_scaling(struct rt_pm *pm)
{
    rt_base_t level;

    if (pm->flags & RT_PM_FREQUENCY_PENDING)
    {
        level = rt_hw_interrupt_disable();
        /* change system runing mode */
        pm->ops->run(pm, pm->run_mode);
        /* changer device frequency */
        _pm_device_frequency_change(pm->run_mode);
        pm->flags &= ~RT_PM_FREQUENCY_PENDING;
        rt_hw_interrupt_enable(level);
    }
}

/**
 * This function selects the sleep mode according to the rt_pm_request/rt_pm_release count.
 */
#if 0
static uint8_t _pm_select_sleep_mode(struct rt_pm *pm)
{
    int index;
    uint8_t mode;

    mode = _pm_default_sleep;
    for (index = PM_SLEEP_MODE_NONE; index < PM_SLEEP_MODE_MAX; index ++)
    {
        if (pm->modes[index])
        {
            mode = index;
            break;
        }
    }
    pm->sleep_mode = mode;

    return mode;
}
#else

__ROM_USED uint8_t _pm_select_sleep_mode(struct rt_pm *pm, rt_tick_t tick)
{
    uint8_t mode;
    int32_t i;
#ifdef RT_PM_USING_PROGRESSIVE_POLICY
    bool same_policy_for_infinite_sleep_time = false;
#else
    bool same_policy_for_infinite_sleep_time = true;
#endif /*RT_PM_USING_PROGRESSIVE_POLICY */


    RT_ASSERT(pm);

    if (pm->pm_select)
    {
        mode = pm->pm_select(tick);
    }
    else
    {
        mode = PM_SLEEP_MODE_IDLE;
        if ((tick < RT_TICK_MAX) || same_policy_for_infinite_sleep_time)
        {
#ifdef RT_PM_USING_PROGRESSIVE_POLICY
            /* reset auto sleep mode */
            pm->prog_policy_curr_idx = 0;
#endif /*RT_PM_USING_PROGRESSIVE_POLICY */
            /* select sleep mode according to timeout_tick */

            for (i = pm->policy_num - 1; i >= 0; i--)
            {
                if (tick >= rt_tick_from_millisecond(pm->policy[i].thresh))
                {
                    mode = pm->policy[i].mode;
                    break;
                }
            }
        }
#ifdef RT_PM_USING_PROGRESSIVE_POLICY
        else // sleep for infinite time
        {
            rt_tick_t sleep_tick;

            RT_ASSERT(pm->prog_policy_curr_idx <= pm->policy_num);

            /* sleep time is chosen by curr mode */
            if (pm->prog_policy_curr_idx < pm->policy_num)
            {
                sleep_tick = rt_tick_from_millisecond(pm->policy[pm->prog_policy_curr_idx].thresh);
                RT_ASSERT(RT_EOK == rt_timer_control(&pm->prog_policy_timer, RT_TIMER_CTRL_SET_TIME, sleep_tick));
                RT_ASSERT(RT_EOK == rt_timer_start(&pm->prog_policy_timer));
                mode = pm->policy[pm->prog_policy_curr_idx];
            }
            else
            {
                mode = pm->policy[pm->policy_num - 1].mode;
            }
        }
#endif /* RT_PM_USING_PROGRESSIVE_POLICY */
    }
    pm->sleep_mode = mode;
    return mode;
}
#endif

#if 0
/**
 * This function changes the power sleep mode base on the result of selection
 */
static void _pm_change_sleep_mode(struct rt_pm *pm, uint8_t mode)
{
    rt_tick_t timeout_tick, delta_tick;
    rt_base_t level;
    int ret = RT_EOK;

    if (mode == PM_SLEEP_MODE_NONE)
    {
        pm->sleep_mode = mode;
        pm->ops->sleep(pm, PM_SLEEP_MODE_NONE);
    }
    else
    {
        level = rt_pm_enter_critical(mode);

        /* Notify app will enter sleep mode */
        if (_pm_notify.notify)
            _pm_notify.notify(RT_PM_ENTER_SLEEP, mode, _pm_notify.data);

        /* Suspend all peripheral device */
        ret = _pm_device_suspend(mode);
        if (ret != RT_EOK)
        {
            _pm_device_resume(mode);
            if (_pm_notify.notify)
                _pm_notify.notify(RT_PM_EXIT_SLEEP, mode, _pm_notify.data);
            rt_pm_exit_critical(level, mode);

            return;
        }

        /* Tickless*/
        if (pm->timer_mask & (0x01 << mode))
        {
            timeout_tick = rt_timer_next_timeout_tick();
            if (timeout_tick == RT_TICK_MAX)
            {
                if (pm->ops->timer_start)
                {
                    pm->ops->timer_start(pm, RT_TICK_MAX);
                }
            }
            else
            {
                timeout_tick = timeout_tick - rt_tick_get();
                if (timeout_tick < RT_PM_TICKLESS_THRESH)
                {
                    mode = PM_SLEEP_MODE_IDLE;
                }
                else
                {
                    pm->ops->timer_start(pm, timeout_tick);
                }
            }
        }

        /* enter lower power state */
        pm->ops->sleep(pm, mode);

        /* wake up from lower power state*/
        if (pm->timer_mask & (0x01 << mode))
        {
            delta_tick = pm->ops->timer_get_tick(pm);
            pm->ops->timer_stop(pm);
            if (delta_tick)
            {
                rt_tick_set(rt_tick_get() + delta_tick);
                rt_timer_check();
            }
        }

        /* resume all device */
        _pm_device_resume(pm->sleep_mode);

        if (_pm_notify.notify)
            _pm_notify.notify(RT_PM_EXIT_SLEEP, mode, _pm_notify.data);

        rt_pm_exit_critical(level, mode);
    }
}
#endif

/**
 *
 */
static void pm_exit(struct rt_pm *pm, uint8_t mode, rt_tick_t sleep_tick)
{
    if (!pm->pm_select)
    {
#ifdef RT_PM_USING_PROGRESSIVE_POLICY
        uint8_t curr_idx;

        curr_idx = pm->prog_policy_curr_idx;

        if (curr_idx < pm->policy_num)
        {
            RT_ASSERT(RT_EOK == rt_timer_stop(pm->prog_policy_timer));
        }
        if ((curr_idx < pm->policy_num) && (sleep_tick >= rt_tick_from_millisecond(pm->policy[curr_idx].thresh)))
        {
            /* prepare for next sleep mode */
            if (pm->prog_policy_curr_idx < pm->policy_num)
            {
                pm->prog_policy_curr_idx++;
            }
        }
        else
        {
            /* abort, reset */
            pm->prog_policy_curr_idx = 0;
        }
#endif /* RT_PM_USING_PROGRESSIVE_POLICY */
    }
}


/**
 *
 */
static uint8_t _pm_enter_sleep(struct rt_pm *pm)
{
    rt_tick_t timeout_tick, delta_tick;
    rt_base_t level;
    int ret = RT_EOK;
    uint8_t mode;

    level = rt_hw_interrupt_disable();

    if ((pm->modes[PM_SLEEP_MODE_IDLE] > 0)
            || (pm->modes[PM_SLEEP_MODE_NONE] > 0))
    {
        mode = PM_SLEEP_MODE_IDLE;
        rt_hw_interrupt_enable(level);
        return mode;
    }

    timeout_tick = rt_timer_next_timeout_tick();
    if (RT_TICK_MAX != timeout_tick)
    {
        timeout_tick = timeout_tick - rt_tick_get();
    }
    mode = _pm_select_sleep_mode(pm, timeout_tick);
    pm->sleep_mode = mode;

    if (mode <= PM_SLEEP_MODE_IDLE)
    {
        rt_hw_interrupt_enable(level);
        return mode;
    }

    /* Notify app will enter sleep mode */
    if (_pm_notify.notify)
        _pm_notify.notify(RT_PM_ENTER_SLEEP, mode, _pm_notify.data);

    /* Suspend all peripheral device */
    ret = _pm_device_suspend(mode);
    if (ret != RT_EOK)
    {
        _pm_device_resume(mode);
        if (_pm_notify.notify)
            _pm_notify.notify(RT_PM_EXIT_SLEEP, mode, _pm_notify.data);
        mode = PM_SLEEP_MODE_IDLE;
        rt_hw_interrupt_enable(level);

        return mode;
    }

    /* Tickless*/
    if (pm->timer_mask & (0x01 << mode))
    {
        timeout_tick = rt_timer_next_timeout_tick();
        if (timeout_tick == RT_TICK_MAX)
        {
            if (pm->ops->timer_start)
            {
                pm->ops->timer_start(pm, RT_TICK_MAX);
            }
        }
        else
        {
            timeout_tick = timeout_tick - rt_tick_get();
            if (timeout_tick < RT_PM_TICKLESS_THRESH)
            {
                mode = PM_SLEEP_MODE_IDLE;
            }
            else
            {
                pm->ops->timer_start(pm, timeout_tick);
            }
        }
    }

    /* enter lower power state */
    pm->ops->sleep(pm, mode);

    /* wake up from lower power state*/
    if (pm->timer_mask & (0x01 << mode))
    {
        delta_tick = pm->ops->timer_get_tick(pm);
        pm->ops->timer_stop(pm);
        if (delta_tick)
        {
            rt_tick_set(rt_tick_get() + delta_tick);
            rt_timer_check();
        }
    }
    else
    {
        delta_tick = 0;
    }

    pm_exit(pm, mode, delta_tick);

    /* resume all device */
    _pm_device_resume(mode);

    if (_pm_notify.notify)
        _pm_notify.notify(RT_PM_EXIT_SLEEP, mode, _pm_notify.data);

    rt_hw_interrupt_enable(level);

    return mode;
}

/**
 * This function will enter corresponding power mode.
 */
__ROM_USED void rt_system_power_manager(void)
{
    if (_pm_init_flag == 0)
        return;

    /* CPU frequency scaling according to the runing mode settings */
    _pm_frequency_scaling(&_pm);

    /* Low Power Mode Processing */
    if (_pm_enter_sleep(&_pm) <= PM_SLEEP_MODE_IDLE)
    {
        rt_base_t level;
        level = rt_hw_interrupt_disable();
        if (_pm.ops->enter_idle)
        {
            _pm.ops->enter_idle(&_pm);
        }
        rt_hw_interrupt_enable(level);
        rt_hw_cpu_idle();
    }
}

/**
 * Upper application or device driver requests the system
 * stall in corresponding power mode.
 *
 * @param parameter the parameter of run mode or sleep mode
 */
#ifndef PM_REQUEST_DEBUG
    __ROM_USED void rt_pm_request(uint8_t mode)
#else
    __ROM_USED void rt_pm_request_debug(uint8_t mode, const char *file, uint32_t line)
#endif
{
    rt_base_t level;
    struct rt_pm *pm;
#ifdef PM_REQUEST_DEBUG
    uint32_t i;
    uint8_t duplicate_num;
#endif /* PM_REQUEST_DEBUG */

    if (_pm_init_flag == 0)
        return;

    if (mode > (PM_SLEEP_MODE_MAX - 1))
        return;

    level = rt_hw_interrupt_disable();
    pm = &_pm;
    if (pm->modes[mode] < 255)
        pm->modes[mode] ++;
    else
        RT_ASSERT(0);

#ifdef PM_REQUEST_DEBUG
    if ((mode == PM_SLEEP_MODE_IDLE) && (_pm_request_hist.idx < PM_REQUEST_HIST_LEN))
    {
        duplicate_num = 0;
        for (i = 0; i < _pm_request_hist.idx; i++)
        {
            if ((_pm_request_hist.items[i].file == file) && (_pm_request_hist.items[i].line_num == line))
            {
                duplicate_num++;
            }
        }
        RT_ASSERT(duplicate_num < 3);
        _pm_request_hist.items[_pm_request_hist.idx].file = file;
        _pm_request_hist.items[_pm_request_hist.idx].line_num = line;
        _pm_request_hist.idx++;
    }
#endif /* PM_REQUEST_DEBUG */
    rt_hw_interrupt_enable(level);
}

/**
 * Upper application or device driver releases the stall
 * of corresponding power mode.
 *
 * @param parameter the parameter of run mode or sleep mode
 *
 */
#ifndef PM_REQUEST_DEBUG
    __ROM_USED void rt_pm_release(uint8_t mode)
#else
    __ROM_USED void rt_pm_release_debug(uint8_t mode, const char *file, uint32_t line)
#endif /* PM_REQUEST_DEBUG */
{
    rt_ubase_t level;
    struct rt_pm *pm;
#ifdef PM_REQUEST_DEBUG
    uint32_t i;
#endif /* PM_REQUEST_DEBUG */

    if (_pm_init_flag == 0)
        return;

    if (mode > (PM_SLEEP_MODE_MAX - 1))
        return;

    level = rt_hw_interrupt_disable();
    pm = &_pm;

#ifdef PM_REQUEST_DEBUG
    if (mode == PM_SLEEP_MODE_IDLE)
    {
        RT_ASSERT(_pm_request_hist.idx <= PM_REQUEST_HIST_LEN);
        for (i = 0; i < _pm_request_hist.idx; i++)
        {
            if (_pm_request_hist.items[i].file == file)
            {
                break;
            }
        }
        if (i < _pm_request_hist.idx)
        {
            /* remove the matched request and replace by last one */
            _pm_request_hist.items[i].file = _pm_request_hist.items[_pm_request_hist.idx - 1].file;
            _pm_request_hist.items[i].line_num = _pm_request_hist.items[_pm_request_hist.idx - 1].line_num;
            _pm_request_hist.idx--;
        }
    }
#endif /* PM_REQUEST_DEBUG */

    if (pm->modes[mode] > 0)
        pm->modes[mode] --;
#ifdef PM_REQUEST_DEBUG
    else
        RT_ASSERT(0);
#endif /* PM_REQUEST_DEBUG */

#ifdef PM_REQUEST_DEBUG
    if ((mode == PM_SLEEP_MODE_IDLE) && (0 == pm->modes[PM_SLEEP_MODE_IDLE]))
    {
        /* if file name not match, request may not be cleared, clear forcely */
        _pm_request_hist.idx = 0;
    }
#endif /* PM_REQUEST_DEBUG */

    rt_hw_interrupt_enable(level);
}

__ROM_USED rt_uint8_t rt_pm_sleep_mode_state_get(uint8_t mode)
{
    if (_pm_init_flag == 0)
        return 0 ;

    if (mode > (PM_SLEEP_MODE_MAX - 1))
        return 0;

    return _pm.modes[mode];
}

/**
 * Register a device with PM feature
 *
 * @param device the device with PM feature
 * @param ops the PM ops for device
 */
__ROM_USED void rt_pm_device_register(struct rt_device *device, const struct rt_device_pm_ops *ops)
{
    rt_base_t level;
    struct rt_device_pm *device_pm;

    RT_DEBUG_NOT_IN_INTERRUPT;

    level = rt_hw_interrupt_disable();

    device_pm = (struct rt_device_pm *)RT_KERNEL_REALLOC(_pm.device_pm,
                (_pm.device_pm_number + 1) * sizeof(struct rt_device_pm));
    if (device_pm != RT_NULL)
    {
        _pm.device_pm = device_pm;
        _pm.device_pm[_pm.device_pm_number].device = device;
        _pm.device_pm[_pm.device_pm_number].ops    = ops;
        _pm.device_pm_number += 1;
    }

    rt_hw_interrupt_enable(level);
}

/**
 * Unregister device from PM manager.
 *
 * @param device the device with PM feature
 */
__ROM_USED void rt_pm_device_unregister(struct rt_device *device)
{
    rt_ubase_t level;
    rt_uint32_t index;
    RT_DEBUG_NOT_IN_INTERRUPT;

    level = rt_hw_interrupt_disable();

    for (index = 0; index < _pm.device_pm_number; index ++)
    {
        if (_pm.device_pm[index].device == device)
        {
            /* remove current entry */
            for (; index < _pm.device_pm_number - 1; index ++)
            {
                _pm.device_pm[index] = _pm.device_pm[index + 1];
            }

            _pm.device_pm[_pm.device_pm_number - 1].device = RT_NULL;
            _pm.device_pm[_pm.device_pm_number - 1].ops = RT_NULL;

            _pm.device_pm_number -= 1;
            /* break out and not touch memory */
            break;
        }
    }

    rt_hw_interrupt_enable(level);
}

/**
 * This function set notification callback for application
 */
__ROM_USED void rt_pm_notify_set(void (*notify)(uint8_t event, uint8_t mode, void *data), void *data)
{
    _pm_notify.notify = notify;
    _pm_notify.data = data;
}

/**
 * This function set default sleep mode when no pm_request
 */
__ROM_USED void rt_pm_default_set(uint8_t sleep_mode)
{
    _pm_default_sleep = sleep_mode;
}

/**
 * RT-Thread device interface for PM device
 */
static rt_size_t _rt_pm_device_read(rt_device_t dev,
                                    rt_off_t    pos,
                                    void       *buffer,
                                    rt_size_t   size)
{
    struct rt_pm *pm;
    rt_size_t length;

    length = 0;
    pm = (struct rt_pm *)dev;
    RT_ASSERT(pm != RT_NULL);

    if (pos < PM_SLEEP_MODE_MAX)
    {
        int mode;

        mode = pm->modes[pos];
        length = rt_snprintf(buffer, size, "%d", mode);
    }

    return length;
}

static rt_size_t _rt_pm_device_write(rt_device_t dev,
                                     rt_off_t    pos,
                                     const void *buffer,
                                     rt_size_t   size)
{
    unsigned char request;

    if (size)
    {
        /* get request */
        request = *(unsigned char *)buffer;
        if (request ==  0x01)
        {
            rt_pm_request(pos);
        }
        else if (request == 0x00)
        {
            rt_pm_release(pos);
        }
    }

    return 1;
}

static rt_err_t _rt_pm_device_control(rt_device_t dev,
                                      int         cmd,
                                      void       *args)
{
    rt_uint32_t mode;

    switch (cmd)
    {
    case RT_PM_DEVICE_CTRL_REQUEST:
        mode = (rt_uint32_t)args;
        rt_pm_request(mode);
        break;

    case RT_PM_DEVICE_CTRL_RELEASE:
        mode = (rt_uint32_t)args;
        rt_pm_release(mode);
        break;
    }

    return RT_EOK;
}

#ifdef RT_PM_USING_PROGRESSIVE_POLICY
static void sleep_timeout(void *parameter)
{
}
#endif /* RT_PM_USING_PROGRESSIVE_POLICY */

__ROM_USED int rt_pm_run_enter(uint8_t mode)
{
    rt_base_t level;
    struct rt_pm *pm;

    if (_pm_init_flag == 0)
        return -RT_EIO;

    if (mode >= PM_RUN_MODE_MAX)
        return -RT_EINVAL;

    level = rt_hw_interrupt_disable();
    pm = &_pm;
    if (mode < pm->run_mode)
    {
        pm->flags &= ~RT_PM_FREQUENCY_PENDING;
        /* change system runing mode */
        pm->ops->run(pm, mode);
        /* changer device frequency */
        _pm_device_frequency_change(mode);
    }
    else if (mode > pm->run_mode)
    {
        pm->flags |= RT_PM_FREQUENCY_PENDING;
    }
    pm->run_mode = mode;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

/**
 * This function will initialize power management.
 *
 * @param ops the PM operations.
 * @param timer_mask indicates which mode has timer feature.
 * @param user_data user data
 */
__ROM_USED void rt_system_pm_init(const struct rt_pm_ops *ops,
                                  rt_uint8_t              timer_mask,
                                  void                   *user_data)
{
    struct rt_device *device;
    struct rt_pm *pm;

    pm = &_pm;
    device = &(_pm.parent);

    device->type        = RT_Device_Class_PM;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = _rt_pm_device_read;
    device->write       = _rt_pm_device_write;
    device->control     = _rt_pm_device_control;
    device->user_data   = user_data;

    /* register PM device to the system */
    rt_device_register(device, "pm", RT_DEVICE_FLAG_RDWR);

    rt_memset(pm->modes, 0, sizeof(pm->modes));
    pm->sleep_mode = _pm_default_sleep;
    pm->run_mode   = RT_PM_DEFAULT_RUN_MODE;
    pm->timer_mask = timer_mask;

    pm->ops = ops;

    pm->device_pm = RT_NULL;
    pm->device_pm_number = 0;
    pm->pm_select = RT_NULL;
    pm->policy_num = sizeof(default_pm_policy) / sizeof(default_pm_policy[0]);
    pm->policy = &default_pm_policy[0];

#ifdef RT_PM_USING_PROGRESSIVE_POLICY
    pm->prog_policy_curr_idx = 0;
    rt_timer_init(pm->prog_policy_timer, "pm", sleep_timeout, RT_TIMER_FLAG_ONE_SHOT);
#endif /* RT_PM_USING_PROGRESSIVE_POLICY */

    _pm_init_flag = 1;
}

__ROM_USED void rt_pm_policy_register(rt_uint8_t num, const pm_policy_t *policy)
{
    _pm.policy_num = num;
    _pm.policy = policy;
}

__ROM_USED void rt_pm_override_mode_select(pm_select_method_t pm_select)
{
    _pm.pm_select = pm_select;
}

__ROM_USED void rt_pm_hw_device_start(void)
{
    rt_base_t mask;

    mask = rt_hw_interrupt_disable();
    _pm.act_hw_device_cnt++;
    rt_hw_interrupt_enable(mask);
}

__ROM_USED void rt_pm_hw_device_stop(void)
{
    rt_base_t mask;

    mask = rt_hw_interrupt_disable();
    RT_ASSERT(_pm.act_hw_device_cnt);
    _pm.act_hw_device_cnt--;
    rt_hw_interrupt_enable(mask);

}

__ROM_USED bool rt_pm_is_hw_device_running(void)
{
    return (_pm.act_hw_device_cnt > 0);
}

__ROM_USED void rt_pm_exit_idle(void)
{
    if (_pm_init_flag && _pm.ops && _pm.ops->exit_idle)
    {
        _pm.ops->exit_idle(&_pm);
    }
}

#if defined(RT_USING_FINSH)&&!defined(LCPU_MEM_OPTIMIZE)
#include <finsh.h>

static const char *_pm_sleep_str[] = PM_SLEEP_MODE_NAMES;
static const char *_pm_run_str[] = PM_RUN_MODE_NAMES;

static void rt_pm_release_mode(int argc, char **argv)
{
    int mode = 0;
    if (argc >= 2)
    {
        mode = atoi(argv[1]);
    }

    rt_pm_release(mode);
}
MSH_CMD_EXPORT_ALIAS(rt_pm_release_mode, pm_release, release power management mode);

static void rt_pm_request_mode(int argc, char **argv)
{
    int mode = 0;
    if (argc >= 2)
    {
        mode = atoi(argv[1]);
    }

    rt_pm_request(mode);
}
MSH_CMD_EXPORT_ALIAS(rt_pm_request_mode, pm_request, request power management mode);

static void rt_pm_run_mode_switch(int argc, char **argv)
{
    int mode = 0;
    if (argc >= 2)
    {
        mode = atoi(argv[1]);
    }

    rt_pm_run_enter(mode);
}
MSH_CMD_EXPORT_ALIAS(rt_pm_run_mode_switch, pm_run, switch power management run mode);

static void rt_pm_dump_status(void)
{
    rt_uint32_t index;
    struct rt_pm *pm;

    pm = &_pm;

    rt_kprintf("| Power Management Mode | Counter | Timer |\n");
    rt_kprintf("+-----------------------+---------+-------+\n");
    for (index = 0; index < PM_SLEEP_MODE_MAX; index ++)
    {
        int has_timer = 0;
        if (pm->timer_mask & (1 << index))
            has_timer = 1;

        rt_kprintf("| %021s | %7d | %5d |\n", _pm_sleep_str[index], pm->modes[index], has_timer);
    }
    rt_kprintf("+-----------------------+---------+-------+\n");

    rt_kprintf("pm current sleep mode: %s\n", _pm_sleep_str[pm->sleep_mode]);
    rt_kprintf("pm current run mode:   %s\n", _pm_run_str[pm->run_mode]);
}
FINSH_FUNCTION_EXPORT_ALIAS(rt_pm_dump_status, pm_dump, dump power management status);
MSH_CMD_EXPORT_ALIAS(rt_pm_dump_status, pm_dump, dump power management status);
#endif

#endif /* RT_USING_PM */
