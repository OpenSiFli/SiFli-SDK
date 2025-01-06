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

#ifndef __PM_H__
#define __PM_H__

#include <rtthread.h>
#include "stdbool.h"

#ifndef PM_HAS_CUSTOM_CONFIG

/* All modes used for rt_pm_request() and rt_pm_release() */
enum
{
    /* sleep modes */
    PM_SLEEP_MODE_NONE = 0,
    PM_SLEEP_MODE_IDLE,
    PM_SLEEP_MODE_LIGHT,
    PM_SLEEP_MODE_DEEP,
    PM_SLEEP_MODE_STANDBY,
    PM_SLEEP_MODE_SHUTDOWN,
    PM_SLEEP_MODE_MAX,
};

enum
{
    /* run modes*/
    PM_RUN_MODE_HIGH_SPEED = 0,
    PM_RUN_MODE_NORMAL_SPEED,
    PM_RUN_MODE_MEDIUM_SPEED,
    PM_RUN_MODE_LOW_SPEED,
    PM_RUN_MODE_MAX,
};

enum
{
    RT_PM_FREQUENCY_PENDING = 0x01,
};

#define RT_PM_DEFAULT_SLEEP_MODE PM_SLEEP_MODE_IDLE
#define RT_PM_DEFAULT_RUN_MODE   PM_RUN_MODE_HIGH_SPEED

/* The name of all modes used in the msh command "pm_dump" */
#define PM_SLEEP_MODE_NAMES     \
{                               \
    "None Mode",                \
    "Idle Mode",                \
    "LightSleep Mode",          \
    "DeepSleep Mode",           \
    "Standby Mode",             \
    "Shutdown Mode",            \
}

#define PM_RUN_MODE_NAMES       \
{                               \
    "High Speed",               \
    "Normal Speed",             \
    "Medium Speed",             \
    "Low Mode",                 \
}

#else /* PM_HAS_CUSTOM_CONFIG */

#include <pm_cfg.h>

#endif /* PM_HAS_CUSTOM_CONFIG */

/**
 * device control flag to request or release power
 */
#define RT_PM_DEVICE_CTRL_REQUEST   (0x20+0x01)
#define RT_PM_DEVICE_CTRL_RELEASE   (0x20+0x00)

struct rt_pm;

/**
 * low power mode operations
 */
struct rt_pm_ops
{
    void (*sleep)(struct rt_pm *pm, uint8_t mode);
    void (*run)(struct rt_pm *pm, uint8_t mode);
    void (*timer_start)(struct rt_pm *pm, rt_uint32_t timeout);
    void (*timer_stop)(struct rt_pm *pm);
    void (*enter_idle)(struct rt_pm *pm);
    void (*exit_idle)(struct rt_pm *pm);
    rt_tick_t (*timer_get_tick)(struct rt_pm *pm);
};

struct rt_device_pm_ops
{
    int (*suspend)(const struct rt_device *device, uint8_t mode);
    void (*resume)(const struct rt_device *device, uint8_t mode);
    int (*frequency_change)(const struct rt_device *device, uint8_t mode);
};

struct rt_device_pm
{
    const struct rt_device *device;
    const struct rt_device_pm_ops *ops;
};

/** pm policy item structure */
typedef struct
{
    uint32_t    thresh;      /**< sleep time threshold in millisecond */
    uint8_t     mode;        /**< power mode is chosen if sleep time threshold is satisfied */
} pm_policy_t;

/** power mode select method type */
typedef uint8_t (*pm_select_method_t)(rt_tick_t tick);

/**
 * power management
 */
struct rt_pm
{
    struct rt_device parent;

    /* modes */
    rt_uint8_t modes[PM_SLEEP_MODE_MAX];
    rt_uint8_t sleep_mode;    /* current sleep mode */
    rt_uint8_t run_mode;      /* current running mode */

    /* the list of device, which has PM feature */
    rt_uint8_t device_pm_number;
    struct rt_device_pm *device_pm;

    /* if the mode has timer, the corresponding bit is 1*/
    rt_uint8_t timer_mask;
    rt_uint8_t flags;
    pm_select_method_t pm_select;  /**< overrided pm select method  */
#ifdef RT_PM_USING_PROGRESSIVE_POLICY
    rt_uint8_t prog_policy_curr_idx;  /**< current policy index for progressive sleep */
    struct rt_timer prog_policy_timer;  /**< timer for progressive sleep  */
#endif /* RT_PM_USING_BUILTIN_POLICY */
    rt_uint8_t policy_num;     /**< indicate the number of policy */
    const pm_policy_t *policy;

    const struct rt_pm_ops *ops;
    /** Running hw device couner */
    uint32_t act_hw_device_cnt;
};

enum
{
    RT_PM_ENTER_SLEEP = 0,
    RT_PM_EXIT_SLEEP,
};

struct rt_pm_notify
{
    void (*notify)(uint8_t event, uint8_t mode, void *data);
    void *data;
};

#ifndef PM_REQUEST_DEBUG
    void rt_pm_request(uint8_t sleep_mode);
    void rt_pm_release(uint8_t sleep_mode);
#else
    void rt_pm_request_debug(uint8_t sleep_mode, const char *file, uint32_t line);
    void rt_pm_release_debug(uint8_t sleep_mode, const char *file, uint32_t line);
    #define rt_pm_request(sleep_mode)  rt_pm_request_debug((sleep_mode), __FILE__, __LINE__)
    #define rt_pm_release(sleep_mode)  rt_pm_release_debug((sleep_mode), __FILE__, __LINE__)
#endif

int rt_pm_run_enter(uint8_t run_mode);
rt_uint8_t rt_pm_sleep_mode_state_get(uint8_t sleep_mode);

void rt_pm_device_register(struct rt_device *device, const struct rt_device_pm_ops *ops);
void rt_pm_device_unregister(struct rt_device *device);

void rt_pm_notify_set(void (*notify)(uint8_t event, uint8_t mode, void *data), void *data);
void rt_pm_default_set(uint8_t sleep_mode);

void rt_system_pm_init(const struct rt_pm_ops *ops,
                       uint8_t              timer_mask,
                       void                 *user_data);

/* policy list is defined in ascending order,
   which means deeper power mode has greater index */
void rt_pm_policy_register(rt_uint8_t num, const pm_policy_t *policy);
void rt_pm_override_mode_select(pm_select_method_t pm_select);

void rt_pm_hw_device_start(void);
void rt_pm_hw_device_stop(void);
bool rt_pm_is_hw_device_running(void);

void rt_pm_exit_idle(void);

#endif /* __PM_H__ */
