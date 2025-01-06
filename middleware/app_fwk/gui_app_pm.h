#ifndef __GUI_APP_PM_H__
#define __GUI_APP_PM_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "button.h"

/** GUI PM event */
typedef enum
{
    GUI_PM_EVT_INVALID,
    GUI_PM_EVT_SUSPEND, /**< Supend GUI */
    GUI_PM_EVT_RESUME,  /**< Resume GUI */
    GUI_PM_EVT_SHUTDOWN, /**< Shutdown */
} gui_pm_event_type_t;

/** GUI PM action sent to FSM */
typedef enum
{
    GUI_PM_ACTION_INVALID = 0,
    GUI_PM_ACTION_BUTTON_PRESSED,    /**< Indicates that a button is pressed */
    GUI_PM_ACTION_BUTTON_RELEASED,   /**< Indicates that a button is released */
    GUI_PM_ACTION_BUTTON_LONG_PRESSED,/**< Indicates that a button is long released */
    GUI_PM_ACTION_BUTTON_CLICKED,     /**< Indicates that a button is clicked */
    GUI_PM_ACTION_WAKEUP,             /**< wakeup GUI if it's suspened */
    GUI_PM_ACTION_SLEEP,              /**< suspend GUI if it's awake */
} gui_pm_action_t;

typedef void (*gui_pm_event_handler_t)(gui_pm_event_type_t event);

/** SYS Power Manager FSM event */
typedef enum
{
    SYS_PWRON_EVT_INVALID = 0,
    SYS_PWRON_EVT_BUTTON_RELEASED,   /**< Indicates that a button is released */
    SYS_PWRON_EVT_BUTTON_LONG_PRESSED,/**< Indicates that a button is long released */
    SYS_PWRON_EVT_STANDBY_WAKEUP,     /**< Indicates that a button is clicked */
    SYS_PWRON_EVT_INIT,
} sys_pwron_evt_t;


/** Power on reason */
typedef enum
{
    GUI_POWER_ON_REASON_UNKNOWN, /**< Power on reason unknown */
    GUI_POWER_ON_REASON_NORMAL,  /**< Normal power on, such as cold boot or power on by long pressed */
    GUI_POWER_ON_REASON_ALARM,   /**< Power on triggered by alarm, run minimal system */
    GUI_POWER_ON_REASON_CHARGER  /**< Power on triggered by charger plugged in, run minimal system */
} sys_power_on_reason_t;


/** Init GUI CTX sema and lock
 *
 * @param[in]  void
 *
 * @return void
 */
void gui_ctx_init(void);

/** Init GUI PM module
 *
 * @param[in]  lcd LCD device
 * @param[in]  handler PM event handler
 *
 * @return void
 */
void gui_pm_init(rt_device_t lcd, gui_pm_event_handler_t handler);

/** GUI PM Finite State Machine
 *
 *  ACTIVE:
 *     GUI_PM_ACTION_BUTTON_CLICKED       -> INACTIVE_PENDING
 *     GUI_PM_ACTION_SLEEP                -> INACTIVE_PENDING
 *     GUI_PM_ACTION_BUTTON_LONG_PRESSED  -> SHUTDOWN
 *  INACTIVE_PENDING:
 *     GUI_PM_ACTION_BUTTON_CLICKED -> ACTIVE
 *     GUI_PM_ACTION_WAKEUP         -> ACTIVE
 *  INACTIVE:
 *     GUI_PM_ACTION_BUTTON_CLICKED -> ACTIVE
 *     GUI_PM_ACTION_WAKEUP         -> ACTIVE
 *
 * @param[in] action action
 *
 * @return void
 */
void gui_pm_fsm(gui_pm_action_t action);

/** Suspend GUI if fsm in INACTIVE_PENDING state and change to INACTIVE
 *
 * Current thread would be suspended and not continue if succeed to suspend
 * the thread is resumed if fsm change to ACTIVE
 *
 *
 * @return void
 */
void gui_suspend(void);

/** Check whether GUI should be suspened
 *
 * @retval true: GUI is to be suspended, false: GUI is not to be suspended
 */
bool gui_is_force_close(void);

/** Check whether GUI is active
 *
 * @retval true: GUI is active, false: GUI is not active
 */
bool gui_is_active(void);


/** Get Power On Reasons
 *
 */
sys_power_on_reason_t sys_get_power_on_reason(void);

/** set system is_powered_on status
 *
 */

void sys_set_is_power_on(bool is_poweron);

/** Check whether system is powered on by known reason
 *
 *
 * @retval true: has been powered on, power on reason is already known
 *         false: not powered on yet, power on reason is unknown yet
 */
bool sys_get_is_power_on(void);

/** Set power on reason and make system power on
 *
 * This function should be called when sys_get_is_power_on returns false.
 * After calling this function, #sys_get_is_power_on would return true,
 * and #sys_get_power_on_reason return the given reason
 *
 * @param[in] reason power on reason
 *
 * @return void
 */
void sys_power_on(sys_power_on_reason_t reason);


/** Init sys power on manager
 *
 * It would get blocked if power on reason cannot be decided,
 * e.g. system in wakeup by pin, system will not be powered on until LONG_PRESSED event is seen.
 * So this function will get blocked until LONG_PRESSED event happens,
 * it will return also if timeout or RELEASED event is seen.
 *
 */
void sys_poweron_mng_init(void);


/** Sys power on manager fsm
 *
 *
 */
void sys_poweron_fsm(sys_pwron_evt_t evt);



#endif  /* __GUI_APP_PM_H__ */
