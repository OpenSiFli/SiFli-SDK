#ifndef __CHARGE_H__
#define __CHARGE_H__
#include <rtthread.h>

#define CHARGE_DEFAULT_FULL_VOLTAGE (4200)     /*mv*/
#define CHARGE_DEFAULT_REP_VOLTAGE  (CHARGE_DEFAULT_FULL_VOLTAGE - 135 )     /*mv*/
#define CHARGE_DEFAULT_VBAT_HIGH_VOLTAGE (4800)     /*mv*/
#define CHARGE_CC_CURRENT_LEVEL0   (65)
#define CHARGE_CC_CURRENT_LEVEL1   (300)


typedef enum
{
    RT_CHARGE_ENABLE = 0x100 + 1,
    RT_CHARGE_GET_STATUS,
    RT_CHARGE_GET_FULL_STATUS,
    RT_CHARGE_GET_DETECT_STATUS,
#if 0 //  defined(CHARGE_MONITOR_TEMP_CONFIG)    
    RT_CHARGE_GET_CORE_TEMPERATURE,
#endif
    RT_CHARGE_FORCE_ENABLE_CHARGING,
    RT_CHARGE_FORCE_SUSPEND_CHARGING,
    RT_CHARGE_FORCE_RESUME_CHARGING,
    RT_CHARGE_GET_HW_STATE,
    RT_CHARGE_SET_PRECC_CURRENT,
    RT_CHARGE_SET_CC_CURRENT,
    RT_CHARGE_SET_TARGET_VOLT,
    RT_CHARGE_SET_REPVOLT,
    RT_CHARGE_SET_OVER_VOLT,
} rt_charge_cmd_t;

typedef enum
{
    RT_CHARGE_HW_ST_OFF      = 0x01,
    RT_CHARGE_HW_ST_PWRUP    = 0x02,
    RT_CHARGE_HW_ST_IDLE     = 0x04,
    RT_CHARGE_HW_PRECC       = 0x08,
    RT_CHARGE_HW_ST_CC       = 0x10,
    RT_CHARGE_HW_ST_CV       = 0x20,
    RT_CHARGE_HW_ST_EOC      = 0x40,
} rt_charge_hw_state_t;



typedef enum
{
    RT_CHARGE_EOK = 0,
    RT_CHARGE_ERROR_UNSUPPORTED        = 0x11000000,
} rt_charge_err_t;


typedef enum
{
    RT_CHARGE_EVENT_DETECT = 0,          /*charging and plugging detection*/
    RT_CHARGE_EVENT_END,                 /*end of charge*/
} rt_charge_event_t;

struct rt_charge_device;
struct rt_charge_ops
{
    rt_err_t (*control)(struct rt_charge_device *device, int cmd, void *arg);
};

typedef struct rt_charge_device
{
    struct rt_device parent;
    const struct rt_charge_ops *ops;
} rt_charge_device_t;



/**
* @brief enable charge
* @param[in] enable 0:disable 1:enable
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_enable(uint8_t enable);

/**
* @brief get charge status
* @param[out] state 0:uncharged 1:in charge
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_get_status(uint8_t *state);

/**
* @brief get charging cable inserted status
* @param[out] state 0:uninserted 1:inserted
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_get_detect_status(uint8_t *state);

/**
* @brief get charge full state
* @param[out] state 0:Not full 1:full
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_get_full_status(uint8_t *state);

/**
* @brief get core temperature
* @param[out] temp
* @return function execution result @see rt_charge_err_t
*/
#if 0 // defined(CHARGE_MONITOR_TEMP_CONFIG)
    rt_charge_err_t rt_charge_get_core_temp(int *temp);
#endif

/**
* @brief force enable charge
* @param[in] enable 0:disable 1:enable
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_force_enable(uint8_t enable);

/**
* @brief force suspend charging
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_force_suspend_charging(void);

/**
* @brief force resume charging
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_force_resume_charging(void);

/**
* @brief get charge hw status
* @param[out] @see rt_charge_hw_state_t
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_get_hw_status(rt_charge_hw_state_t *state);



/**
* @brief set charge rx ind
* @param[in] rx_ind
* @return function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_set_rx_ind(rt_err_t (*rx_ind)(rt_device_t dev, rt_size_t size));


/**
 * @brief Configure PRECC current
 *
 *
 * @param[in] current PRECC current in mA
 *
 * @retval function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_set_precc_current(float current);


/**
 * @brief Configure CC current
 *
 *
 * @param[in] current CC current in mA
 *
 * @retval function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_set_cc_current(uint32_t current);


/**
 * @brief Configure charging full target voltage
 *
 *
 * @param[in] volt target voltage in mV
 *
 * @retval function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_set_target_volt(uint32_t volt);


/**
 * @brief Configure charging REP voltage
 *
 *
 * @param[in] volt rep voltage in mV
 *
 * @retval function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_set_repvolt(uint32_t volt);


/**
 * @brief Configure charging over voltage protection
 *
 *
 * @param[in] volt over voltage protection in mV
 *
 * @retval function execution result @see rt_charge_err_t
*/
rt_charge_err_t rt_charge_set_over_volt(uint32_t volt);


void rt_charge_event_notify(rt_charge_event_t event);


rt_err_t rt_charge_register(rt_charge_device_t *charge, const struct rt_charge_ops *ops, const void *user_data);

#endif /* __CHARGE_H__ */
