#include <rtthread.h>
#include "board.h"
#ifdef BSP_USING_PM
#include "drv_gpio.h"
#include "bf0_pm.h"
#include "gui_app_pm.h"
#include "drv_io.h"
#ifdef GUI_PM_METRICS_ENABLED
    #include "metrics_collector.h"
    #include "metrics_id_middleware.h"
#endif /* GUI_PM_METRICS_ENABLED */

#define LOG_TAG       "APP.FWK.PM"
#include "log.h"


#define GUI_APP_PIN_WAKEUP (HPSYS_AON_WSR_PIN0 | HPSYS_AON_WSR_PIN1 | HPSYS_AON_WSR_PIN2 \
                            | HPSYS_AON_WSR_PIN3)



typedef struct
{
    /** true: has powered on, false: not powered on yet */
    bool is_poweron;
    struct rt_semaphore sema;
    sys_power_on_reason_t power_on_reason;
} sys_poweron_mng_t;


/** GUI state */
typedef enum
{
    GUI_STATE_INACTIVE = 0,  /**< GUI is inactive  */
    GUI_STATE_ACTIVE,        /**< GUI is active  */
    GUI_STATE_POWER_DOWN,    /**< system is power down */
    GUI_STATE_INACTIVE_PENDING, /**< GUI is going to INACTIVE state  */
} gui_state_t;


#ifdef GUI_PM_METRICS_ENABLED
typedef struct
{
    uint32_t lcd_on_start_time;
    uint32_t lcd_idle_start_time;
    uint8_t lcd_brightness;
    float lcd_on_time;
    float lcd_idle_time;
    float lcd_off_time;
} gui_stat_t;
#endif /* GUI_PM_METRICS_ENABLED */

typedef struct
{
    rt_device_t lcd;
    rt_device_t touch;
    rt_device_t wheel;
    gui_state_t state;
    struct rt_semaphore sema;
    struct rt_mutex lock;
    gui_pm_event_handler_t handler;
    bool lcd_opened;
#ifdef GUI_PM_METRICS_ENABLED
    gui_stat_t stat;
#endif /* GUI_PM_METRICS_ENABLED */
} gui_ctx_t;

#ifdef GUI_PM_METRICS_ENABLED
typedef struct
{
    uint8_t lcd_brightness;
    uint8_t reserved[3];
    float lcd_on_time;
    float lcd_idle_time;
} gui_pm_stat_metrics_t;
#endif /* GUI_PM_METRICS_ENABLED */

static gui_ctx_t s_gui_ctx;
static sys_poweron_mng_t s_sys_poweron_mng;

#ifdef GUI_PM_METRICS_ENABLED
    mc_collector_t gui_pm_metrics_collector;
#endif /* GUI_PM_METRICS_ENABLED */

static void gui_pm_enter_critical(void);
static void gui_pm_exit_critical(void);



#if defined (APP_SCREEN_ALWAYS_ON_DEFINED) && defined (SCREEN_ALWAYS_ON_DISPLAY)
    extern bool screen_always_on_is_valid(void);
#endif /* APP_SCREEN_ALWAYS_ON_DEFINED */

#define ENUM_TO_NAME_CASE(e) case e: return #e
static char *gui_state_to_name(gui_state_t s)
{
    switch (s)
    {
        ENUM_TO_NAME_CASE(GUI_STATE_INACTIVE);
        ENUM_TO_NAME_CASE(GUI_STATE_ACTIVE);
        ENUM_TO_NAME_CASE(GUI_STATE_POWER_DOWN);
        ENUM_TO_NAME_CASE(GUI_STATE_INACTIVE_PENDING);

    default:
        return "UNKNOW";
    }
}

static char *gui_pm_action_to_name(gui_pm_action_t a)
{
    switch (a)
    {
        ENUM_TO_NAME_CASE(GUI_PM_ACTION_INVALID);
        ENUM_TO_NAME_CASE(GUI_PM_ACTION_BUTTON_PRESSED);
        ENUM_TO_NAME_CASE(GUI_PM_ACTION_BUTTON_RELEASED);
        ENUM_TO_NAME_CASE(GUI_PM_ACTION_BUTTON_LONG_PRESSED);

        ENUM_TO_NAME_CASE(GUI_PM_ACTION_BUTTON_CLICKED);
        ENUM_TO_NAME_CASE(GUI_PM_ACTION_WAKEUP);
        ENUM_TO_NAME_CASE(GUI_PM_ACTION_SLEEP);

    default:
        return "UNKNOW";
    }
}

void close_display(void)
{
    gui_pm_enter_critical();
    if (s_gui_ctx.lcd && s_gui_ctx.lcd_opened)
    {
        //touch_suspend();

#if defined (APP_SCREEN_ALWAYS_ON_DEFINED) && defined (SCREEN_ALWAYS_ON_DISPLAY)
        if (screen_always_on_is_valid())
        {
            const uint8_t idle_mode_on = 1;
            rt_device_control(s_gui_ctx.lcd, RTGRAPHIC_CTRL_SET_MODE, (void *)&idle_mode_on);
#ifdef GUI_PM_METRICS_ENABLED
            s_gui_ctx.stat.lcd_idle_start_time = HAL_GTIMER_READ();
#endif /* GUI_PM_METRICS_ENABLED */
        }
        else
#endif /*APP_SCREEN_ALWAYS_ON_DEFINED*/
        {
            rt_device_control(s_gui_ctx.lcd, RTGRAPHIC_CTRL_POWEROFF, NULL);
        }

        if (s_gui_ctx.touch) rt_device_control(s_gui_ctx.touch, RTGRAPHIC_CTRL_POWEROFF, NULL);
        //if(s_gui_ctx.touch)  rt_device_close(s_gui_ctx.touch);
        //if(s_gui_ctx.wheel)  rt_device_close(s_gui_ctx.wheel);

        s_gui_ctx.lcd_opened = false;
#ifdef GUI_PM_METRICS_ENABLED
        s_gui_ctx.stat.lcd_on_time += (float)(HAL_GTIMER_READ() - s_gui_ctx.stat.lcd_on_start_time) / HAL_LPTIM_GetFreq();
#endif /* GUI_PM_METRICS_ENABLED */
    }
    gui_pm_exit_critical();

}

void open_display(void)
{
    uint16_t cf;

    gui_pm_enter_critical();
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */
    if (s_gui_ctx.lcd && !s_gui_ctx.lcd_opened)
    {
#if defined (APP_SCREEN_ALWAYS_ON_DEFINED) && defined (SCREEN_ALWAYS_ON_DISPLAY)
        if (screen_always_on_is_valid())
        {
            const uint8_t idle_mode_on = 0;
            rt_device_control(s_gui_ctx.lcd, RTGRAPHIC_CTRL_SET_MODE, (void *)&idle_mode_on);
#ifdef GUI_PM_METRICS_ENABLED
            s_gui_ctx.stat.lcd_idle_time += (float)(HAL_GTIMER_READ() - s_gui_ctx.stat.lcd_idle_start_time) / HAL_LPTIM_GetFreq();
#endif /* GUI_PM_METRICS_ENABLED */
        }
        else
#endif /*APP_SCREEN_ALWAYS_ON_DEFINED*/
        {
            rt_device_control(s_gui_ctx.lcd, RTGRAPHIC_CTRL_POWERON, NULL);
        }

        if (s_gui_ctx.touch)  rt_device_control(s_gui_ctx.touch, RTGRAPHIC_CTRL_POWERON, NULL);
        //if(s_gui_ctx.touch)  rt_device_open(s_gui_ctx.touch, RT_DEVICE_FLAG_RDONLY);
        //if(s_gui_ctx.wheel)  rt_device_open(s_gui_ctx.wheel, RT_DEVICE_FLAG_RDONLY);

        //touch_resume();

        s_gui_ctx.lcd_opened = true;
#ifdef GUI_PM_METRICS_ENABLED
        s_gui_ctx.stat.lcd_on_start_time = HAL_GTIMER_READ();
        s_gui_ctx.stat.lcd_brightness = 0;//Default value
        rt_device_control(s_gui_ctx.lcd, RTGRAPHIC_CTRL_GET_BRIGHTNESS_ASYNC, (void *)&s_gui_ctx.stat.lcd_brightness);
#endif /* GUI_PM_METRICS_ENABLED */
    }
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */
    gui_pm_exit_critical();
}

static void change_gui_state(gui_state_t new_state)
{
    gui_state_t tgt_state = s_gui_ctx.state;

    if (GUI_STATE_ACTIVE == new_state)
    {
        tgt_state = new_state;
    }
    else if (GUI_STATE_POWER_DOWN != s_gui_ctx.state)
    {
        tgt_state = new_state;
    }

    //State changed
    if (tgt_state != s_gui_ctx.state)
    {
        LOG_I("change_gui_state:[%s]->[%s]", gui_state_to_name(s_gui_ctx.state), gui_state_to_name(tgt_state));

        s_gui_ctx.state = tgt_state;
    }
}


static void gui_sleep(void)
{
    rt_sem_take(&s_gui_ctx.sema, RT_WAITING_FOREVER);
}

static void gui_wakeup(void)
{
    rt_sem_release(&s_gui_ctx.sema);
}

static void emulate_pin_irq(uint32_t wsr)
{
    uint16_t pin;
    HAL_StatusTypeDef status;
    GPIO_TypeDef *gpio;

    wsr >>= HPSYS_AON_WSR_PIN0_Pos;
    wsr &= 0x3F;
    for (uint32_t i = 0; (i < 6) && wsr; i++)
    {
        if (wsr & 1)
        {
            gpio = HAL_HPAON_QueryWakeupGpioPin(i, &pin);
            RT_ASSERT(gpio);
            button_irq_trigger(GET_PIN_2(gpio, pin));
        }
        wsr >>= 1;
    }
}

static void gui_pm_enter_critical(void)
{
    rt_err_t err;

    err = rt_mutex_take(&s_gui_ctx.lock, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
}


static void gui_pm_exit_critical(void)
{
    rt_err_t err;

    err = rt_mutex_release(&s_gui_ctx.lock);
    RT_ASSERT(RT_EOK == err);
}

void aon_irq_handler_hook(uint32_t wsr)
{
#ifdef BSP_PM_STANDBY_SHUTDOWN
    if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
    {
        sys_poweron_fsm(SYS_PWRON_EVT_STANDBY_WAKEUP);
    }
#endif /* BSP_PM_STANDBY_SHUTDOWN */
}

bool gui_is_force_close(void)
{
    if (GUI_STATE_INACTIVE_PENDING == s_gui_ctx.state)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool gui_is_active(void)
{
    if (GUI_STATE_ACTIVE == s_gui_ctx.state)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void gui_resume(void)
{
    pm_scenario_start(PM_SCENARIO_UI);
    LOG_I("gui_resume");
    gui_wakeup();
}

void gui_suspend(void)
{
    gui_pm_enter_critical();
    if (GUI_STATE_INACTIVE_PENDING == s_gui_ctx.state)
    {
        change_gui_state(GUI_STATE_INACTIVE);
        gui_pm_exit_critical();

        LOG_I("gui_suspend");

        close_display();
        pm_scenario_stop(PM_SCENARIO_UI);
        gui_sleep();
        LOG_I("gui_suspend_resume");
        open_display();
    }
    else
    {
        gui_pm_exit_critical();
        LOG_I("gui suspend canel");
    }
}

void gui_pm_fsm(gui_pm_action_t action)
{
    gui_pm_event_type_t event;

    LOG_I("fsm:Cur[%s],Action[%s]", gui_state_to_name(s_gui_ctx.state),
          gui_pm_action_to_name(action));

    gui_pm_enter_critical();

    event = GUI_PM_EVT_INVALID;

    switch (s_gui_ctx.state)
    {
    case GUI_STATE_ACTIVE:
    {
        if ((GUI_PM_ACTION_BUTTON_CLICKED == action)
                || (GUI_PM_ACTION_SLEEP == action))
        {
            change_gui_state(GUI_STATE_INACTIVE_PENDING);
            event = GUI_PM_EVT_SUSPEND;
        }
        else if (GUI_PM_ACTION_BUTTON_LONG_PRESSED == action)
        {
            event = GUI_PM_EVT_SHUTDOWN;
        }
        break;
    }
    case GUI_STATE_INACTIVE_PENDING:
    {
        if ((GUI_PM_ACTION_BUTTON_CLICKED == action)
                || (GUI_PM_ACTION_WAKEUP == action))
        {
            change_gui_state(GUI_STATE_ACTIVE);
            event = GUI_PM_EVT_RESUME;
        }
        break;
    }
    case GUI_STATE_INACTIVE: //未激活
    {
        if ((GUI_PM_ACTION_BUTTON_CLICKED == action)
                || (GUI_PM_ACTION_WAKEUP == action))
        {
            event = GUI_PM_EVT_RESUME;
            change_gui_state(GUI_STATE_ACTIVE);
            gui_resume();
        }
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    if (GUI_PM_EVT_INVALID != event)
    {
        RT_ASSERT(s_gui_ctx.handler);
        s_gui_ctx.handler(event);
    }

    gui_pm_exit_critical();
}

void gui_ctx_init(void)
{
    rt_sem_init(&s_gui_ctx.sema, "ui_pm", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&s_gui_ctx.lock, "ui_pm", RT_IPC_FLAG_FIFO);
}

void gui_pm_init(rt_device_t lcd, gui_pm_event_handler_t handler)
{
    RT_ASSERT(lcd);
    RT_ASSERT(handler);

    s_gui_ctx.lcd = lcd;
    s_gui_ctx.touch = rt_device_find("touch");
    s_gui_ctx.wheel = rt_device_find("wheel");
    s_gui_ctx.handler = handler;
    s_gui_ctx.lcd_opened = true;

    s_gui_ctx.state = GUI_STATE_ACTIVE;

}

sys_power_on_reason_t sys_get_power_on_reason(void)
{
    return s_sys_poweron_mng.power_on_reason;
}

void sys_set_is_power_on(bool is_poweron)
{
    s_sys_poweron_mng.is_poweron = is_poweron;
}

bool sys_get_is_power_on(void)
{
    return s_sys_poweron_mng.is_poweron;
}

#if defined (SOLUTION_WATCH)
    extern void app_blocking_when_low_voltage(void);
#endif

void sys_power_on(sys_power_on_reason_t reason)
{
    if (!s_sys_poweron_mng.is_poweron)
    {
        s_sys_poweron_mng.is_poweron = true;
        s_sys_poweron_mng.power_on_reason = reason;

        if (PM_STANDBY_BOOT != SystemPowerOnModeGet())
        {
            rt_sem_release(&s_sys_poweron_mng.sema);
        }
#ifdef BSP_PM_STANDBY_SHUTDOWN
        else
        {
            gui_pm_fsm(GUI_PM_ACTION_WAKEUP);
        }
#endif
    }
}

__WEAK void pm_shutdown(void)
{
#ifdef BSP_PM_STANDBY_SHUTDOWN
    rt_err_t err;
    s_sys_poweron_mng.is_poweron = false;
    gui_pm_fsm(GUI_PM_ACTION_SLEEP);
#else

    rt_hw_interrupt_disable();
    HAL_PMU_EnterHibernate();
    while (1) {};
#endif
}

static void sys_pwron_fsm_handle_evt_init(sys_poweron_mng_t *mng)
{
    rt_err_t err;

    switch (SystemPowerOnModeGet())
    {
    case PM_REBOOT_BOOT:
    case PM_COLD_BOOT:
    {
#ifdef BSP_PM_STANDBY_SHUTDOWN
        if (PMUC_WSR_RTC & pm_get_wakeup_src())
        {
            // RTC唤醒
            NVIC_EnableIRQ(RTC_IRQn);
            // TODO: move to RTC callback, let user code call gui_set_power_on_reason
            sys_power_on(GUI_POWER_ON_REASON_ALARM);
        }
        else
#endif
        {
            sys_power_on(GUI_POWER_ON_REASON_NORMAL);
        }
        break;
    }
    case PM_HIBERNATE_BOOT:
    case PM_SHUTDOWN_BOOT:
    {
        if (PMUC_WSR_RTC & pm_get_wakeup_src())
        {
            // RTC唤醒
            NVIC_EnableIRQ(RTC_IRQn);
            // TODO: move to RTC callback, let user code call gui_set_power_on_reason
            sys_power_on(GUI_POWER_ON_REASON_ALARM);
        }
#ifdef BSP_USING_CHARGER
        else if ((PMUC_WSR_PIN0 << (pm_get_charger_pin_wakeup())) & pm_get_wakeup_src())
        {
            //  LOG_I("pm_get_charger_pin_wakeup :%d",pm_get_charger_pin_wakeup());
            sys_power_on(GUI_POWER_ON_REASON_CHARGER);
        }
#endif
        else if (PMUC_WSR_PIN_ALL & pm_get_wakeup_src())
        {
            /* do nothing */
        }
        else if (0 == pm_get_wakeup_src())
        {
            RT_ASSERT(0);
        }
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    //long pressed detection time is 10s
    err = rt_sem_take(&mng->sema, rt_tick_from_millisecond(30000));
    if (-RT_ETIMEOUT == err)
    {
        LOG_I("shutdown again");
        pm_shutdown();
    }
    else if (RT_EOK == err)
    {
        /* do nothing */
    }
    else
    {
        RT_ASSERT(0);
    }
}

static void sys_pwron_fsm_handle_evt_standby_wakeup(sys_poweron_mng_t *mng)
{
    RT_ASSERT(PM_STANDBY_BOOT == SystemPowerOnModeGet());
    if (HPSYS_AON_WSR_RTC & pm_get_wakeup_src())
    {
        // RTC唤醒
        NVIC_EnableIRQ(RTC_IRQn);
        // TODO: move to RTC callback, let user code call gui_set_power_on_reason
        sys_power_on(GUI_POWER_ON_REASON_ALARM);
    }
    else if (0 == pm_get_wakeup_src())
    {
        RT_ASSERT(0);
    }
}

#if defined (SOLUTION_WATCH)
    bool app_is_blocking_when_low_voltage(void);
#endif

void sys_poweron_fsm(sys_pwron_evt_t evt)
{
    sys_poweron_mng_t *mng;

    mng = &s_sys_poweron_mng;
    LOG_I("pwr_fsm:%d, %d\n", mng->is_poweron, evt);
    if (!mng->is_poweron)
    {
        switch (evt)
        {
        case SYS_PWRON_EVT_INIT:
        {
            sys_pwron_fsm_handle_evt_init(mng);
            break;
        }
        case SYS_PWRON_EVT_BUTTON_LONG_PRESSED:
        {
            sys_power_on(GUI_POWER_ON_REASON_NORMAL);
            break;
        }
        case SYS_PWRON_EVT_BUTTON_RELEASED:
        {
#ifndef BSP_PM_STANDBY_SHUTDOWN
            LOG_I("shutdown again");
            pm_shutdown();
#endif
            break;
        }
        case SYS_PWRON_EVT_STANDBY_WAKEUP:
        {
            sys_pwron_fsm_handle_evt_standby_wakeup(mng);
            break;
        }
        default:
        {
            RT_ASSERT(0);
            break;
        }
        }
    }
    else
    {
    }
}


void sys_poweron_mng_init(void)
{
    sys_poweron_mng_t *mng;

    mng = &s_sys_poweron_mng;

    rt_sem_init(&mng->sema, "poweron", 0, RT_IPC_FLAG_FIFO);
    mng->is_poweron = false;
    mng->power_on_reason = GUI_POWER_ON_REASON_UNKNOWN;

    sys_poweron_fsm(SYS_PWRON_EVT_INIT);
}



#ifdef GUI_PM_METRICS_ENABLED

static void gui_pm_metrics_collect(void *user_data)
{
    gui_pm_stat_metrics_t *metrics;

    metrics = mc_alloc_metrics(METRICS_MW_GUI_PM_STAT, sizeof(gui_pm_stat_metrics_t));
    RT_ASSERT(metrics);
    metrics->lcd_brightness = s_gui_ctx.stat.lcd_brightness;
    metrics->lcd_on_time = s_gui_ctx.stat.lcd_on_time;
    metrics->lcd_idle_time = s_gui_ctx.stat.lcd_idle_time;
    s_gui_ctx.stat.lcd_on_time = 0;
    s_gui_ctx.stat.lcd_idle_time = 0;
    mc_save_metrics(metrics, true);
}

static int gui_pm_metrics_init(void)
{
    mc_err_t err;

    gui_pm_metrics_collector.callback = gui_pm_metrics_collect;
    gui_pm_metrics_collector.period = MC_PERIOD_EVERY_HOUR;
    gui_pm_metrics_collector.user_data = 0;

    err = mc_register_collector(&gui_pm_metrics_collector);
    RT_ASSERT(MC_OK == err);
    return 0;

}
INIT_APP_EXPORT(gui_pm_metrics_init);
#endif /* GUI_PM_METRICS_ENABLED */


#endif /* BSP_USING_PM */

